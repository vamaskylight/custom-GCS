"""XFeat, learned features (Apache 2.0), for a map of another season (--features xfeat).

ORB, AKAZE and SIFT find a picture on a map of the same season, but not on one of another: in a
test on 2026-10-29, pictures drawn from summer aerial photos were found on a summer map 20 times
out of 20, and on Esri's winter pictures of the same place (bare trees, long shadows) 0 times out
of 20. XFeat's semi-dense matching with its refinement (``match_xfeat_star``) found the winter map 19
times out of 20 with every check of the matcher on (middle 1.9 m), in about 0.3 s a picture on a
desktop processor (4 threads, 4000 points). Less exact than ORB on a map of the same season: see
"The map" in the README for the simulator flight.

XFeat is not part of this program. It needs PyTorch (CPU is enough) and XFeat's own code with its
weights, from https://github.com/verlab/accelerated_features (Apache 2.0, weights included):

    pip install torch tqdm
    git clone https://github.com/verlab/accelerated_features ~/xfeat-src

and ``--xfeat-dir`` pointing there (or VAMA_XFEAT_DIR). Only the matching with the map uses it:
the movement between two pictures a second apart (the same season) stays with ORB, which is fast.
"""

from __future__ import annotations

import contextlib
import io
import os
import sys
from pathlib import Path

import numpy as np

DEFAULT_TOP_K = 4000            # 2000 found 17 of 20 on the winter map, 1000 only 6


class XFeatError(RuntimeError):
    """XFeat cannot be used, in plain words."""


def default_dir() -> Path:
    """VAMA_XFEAT_DIR, else xfeat-src beside this computer's other VAMA tools."""
    env = os.environ.get("VAMA_XFEAT_DIR")
    if env:
        return Path(env)
    base = os.environ.get("LOCALAPPDATA")
    return (Path(base) / "vama-ml" / "xfeat-src") if base else (Path.home() / "xfeat-src")


class XFeatMatcher:
    def __init__(self, source_dir: Path | None = None, top_k: int = DEFAULT_TOP_K, threads: int | None = None) -> None:
        source_dir = Path(source_dir) if source_dir else default_dir()
        try:
            import torch
        except ImportError:
            raise XFeatError("PyTorch is not installed (pip install torch tqdm): --features xfeat needs it") from None
        weights = source_dir / "weights" / "xfeat.pt"
        if not (source_dir / "modules" / "xfeat.py").is_file() or not weights.is_file():
            raise XFeatError(f"no XFeat at {source_dir}: git clone https://github.com/verlab/accelerated_features "
                             "there, or give its folder with --xfeat-dir")
        if threads:
            torch.set_num_threads(int(threads))
        if str(source_dir) not in sys.path:
            sys.path.insert(0, str(source_dir))
        try:
            from modules.xfeat import XFeat
        except ImportError as exc:
            raise XFeatError(f"XFeat at {source_dir} cannot be loaded: {exc}") from None
        with contextlib.redirect_stdout(io.StringIO()):            # it prints where it loads from
            self.model = XFeat(weights=str(weights), top_k=int(top_k))
        self.top_k = int(top_k)
        self.source_dir = source_dir

    def match(self, image1: np.ndarray, mask1: np.ndarray, image2: np.ndarray,
              mask2: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Matched points (N x 2 each, picture 1 to picture 2) inside both masks, each point of
        picture 2 used once at most."""
        m0, m1 = self.model.match_xfeat_star(image1[..., None], image2[..., None], top_k=self.top_k)
        m0, m1 = np.asarray(m0, np.float32), np.asarray(m1, np.float32)
        if len(m0) == 0:
            return m0.reshape(0, 2), m1.reshape(0, 2)

        def inside(points: np.ndarray, mask: np.ndarray) -> np.ndarray:
            x = np.clip(points[:, 0].astype(int), 0, mask.shape[1] - 1)
            y = np.clip(points[:, 1].astype(int), 0, mask.shape[0] - 1)
            return mask[y, x] > 0

        keep = inside(m0, mask1) & inside(m1, mask2)
        m0, m1 = m0[keep], m1[keep]
        # One map point for one picture point, as for the other features (a point that "matches"
        # many is no evidence).
        cells = np.round(m1 * 2.0).astype(np.int64)
        _uniq, first, counts = np.unique(cells[:, 0] * 1_000_003 + cells[:, 1], return_index=True, return_counts=True)
        single = np.zeros(len(m1), bool)
        single[first[counts == 1]] = True
        return m0[single], m1[single]
