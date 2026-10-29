"""A bench check of the matching's speed: how long one picture takes to match on this computer, and so
how many pictures a second the drone's computer can match (vama_nav takes two a second).

No camera and no flight controller, nothing is sent anywhere: the pictures are drawn from aerial
photos (the test photos of fetch_test_map.py, or any MBTiles map) as the simulator's stand-in camera
draws them, along a straight flight at 500 m and 12 m/s, and matched with the map one after the other
as in flight (the movement between pictures too). The picture is the AI Camera's, 2028 x 1520.

    python -m vama_nav.bench --map davis-2020-z17.mbtiles --ground davis-2022-z18.mbtiles [--features xfeat]

It prints the computer, how many pictures gave a position and how close, each stage's time and the
pictures a second one matching thread manages.
"""

from __future__ import annotations

import argparse
import math
import os
import platform
import random
import sqlite3
import statistics
import sys
import time
from datetime import UTC, datetime
from pathlib import Path

import cv2
import numpy as np

from .geo import distance_m, offset
from .geometry import Lens, Pose
from .map_tiles import MapError, TileMap
from .matcher import FEATURES, Job, Matcher, MatchSettings
from .sim_camera import SimCamera

WARM_UP = 2                     # the first pictures are not counted (the libraries start up)
TWO_A_SECOND_S = 0.4            # a picture's middle time under this: two a second leave room for the rest
ONE_A_SECOND_S = 0.8
STAGES = ("rectify", "features", "map", "map_features", "pair", "xfeat", "fit")
MARGIN_M = 450.0                # the flight stays this far inside both maps (the picture's ground, the search)


def _size(text: str) -> tuple[int, int]:
    w, _, h = text.lower().partition("x")
    return int(w), int(h)


def _parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="vama_nav.bench", description=__doc__.split("\n\n")[0])
    p.add_argument("--map", type=Path, required=True, help="the drone's map (MBTiles)")
    p.add_argument("--ground", type=Path, required=True,
                   help="the photos the pictures are drawn from (MBTiles), of the same place as the map")
    p.add_argument("--features", default="orb", choices=FEATURES)
    p.add_argument("--xfeat-dir", type=Path, help="--features xfeat: XFeat's folder (default VAMA_XFEAT_DIR)")
    p.add_argument("--pictures", type=int, default=40, help="how many (default 40, about a minute on a Pi)")
    p.add_argument("--picture", type=_size, default=(2028, 1520), help="picture size (default 2028x1520)")
    p.add_argument("--hfov-deg", type=float, default=66.0, help="the lens's field of view across the picture")
    p.add_argument("--height-m", type=float, default=500.0,
                   help="the flight's height (default 500 m, the client's: a picture takes about twice as long as from 300 m)")
    p.add_argument("--speed-mps", type=float, default=12.0)
    p.add_argument("--interval-s", type=float, default=0.5, help="the pictures' spacing in flight (vama_nav's default)")
    p.add_argument("--threads", type=int, help="OpenCV's threads (default: as it starts)")
    p.add_argument("--seed", type=int, default=1)
    return p


def _area(tile_map: TileMap) -> tuple[float, float, float, float]:
    """(west, south, east, north) of the map's tiles at its zoom."""
    bounds = tile_map.meta.get("bounds")
    if bounds:
        west, south, east, north = (float(v) for v in bounds.split(","))
        return west, south, east, north
    with sqlite3.connect(f"file:{tile_map.path.as_posix()}?mode=ro", uri=True) as db:
        x0, x1, r0, r1 = db.execute("SELECT MIN(tile_column), MAX(tile_column), MIN(tile_row), MAX(tile_row) FROM tiles "
                                    "WHERE zoom_level=?", (tile_map.zoom,)).fetchone()
    rows = 2 ** tile_map.zoom
    north, west = tile_map.latlon(x0 * 256.0, (rows - 1 - r1) * 256.0)
    south, east = tile_map.latlon((x1 + 1) * 256.0, (rows - r0) * 256.0)
    return west, south, east, north


def _computer(features: str, matcher: Matcher) -> str:
    parts = [platform.platform(), platform.machine(), f"{os.cpu_count()} processors",
             f"Python {platform.python_version()}", f"OpenCV {cv2.__version__} ({cv2.getNumThreads()} threads)",
             f"numpy {np.__version__}"]
    if features == "xfeat":
        import torch

        parts.append(f"PyTorch {torch.__version__} ({torch.get_num_threads()} threads)")
    return ", ".join(parts)


def _spread(values: list[float], unit: str, digits: int = 2) -> str:
    v = sorted(values)
    return (f"middle {statistics.median(v):.{digits}f} {unit}, 1 in 10 over {v[int(0.9 * (len(v) - 1))]:.{digits}f}, "
            f"worst {v[-1]:.{digits}f}")


def run(args, out=print) -> int:
    """The bench check. Returns 0, or 3 when no picture gave a position."""
    if args.threads is not None:
        cv2.setNumThreads(int(args.threads))
    tile_map, ground = TileMap(args.map), TileMap(args.ground)
    try:
        matcher = Matcher(tile_map, Lens(args.hfov_deg),
                          MatchSettings(features=args.features, xfeat_dir=str(args.xfeat_dir or "")))
        a, b = _area(tile_map), _area(ground)
        west, south, east, north = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
        lat0, lon0 = 0.5 * (south + north), 0.5 * (west + east)
        span_m = args.pictures * args.interval_s * args.speed_mps
        if (distance_m(south, lon0, north, lon0) < span_m + 2 * MARGIN_M
                or distance_m(lat0, west, lat0, east) < 2 * MARGIN_M):
            out(f"the two maps share too little ground for {args.pictures} pictures {span_m / args.pictures:.0f} m apart "
                f"(the flight goes north and keeps {MARGIN_M:g} m inside both)")
            return 2
        rng = random.Random(args.seed)
        camera = SimCamera(ground, Lens(args.hfov_deg), size=args.picture, ground_alt_m=0.0,
                           rng=np.random.default_rng(args.seed))
        out(f"vama_nav bench check, {datetime.now(UTC).isoformat(timespec='minutes').replace('+00:00', 'Z')}")
        out(f"computer: {_computer(args.features, matcher)}")
        out(f"map: {tile_map.describe()}; pictures drawn from {ground.describe()}")
        out(f"{args.features} features, pictures {camera.size[0]}x{camera.size[1]} from {args.height_m:g} m, "
            f"{args.speed_mps * args.interval_s:g} m apart ({args.speed_mps:g} m/s, a picture each {args.interval_s:g} s)")
        times, errors, why = [], [], []
        stages: dict[str, list[float]] = {}
        for i in range(args.pictures + WARM_UP):
            lat, lon = offset(lat0, lon0, -0.5 * span_m + i * args.speed_mps * args.interval_s, 0.0)
            pose = Pose(math.radians(rng.uniform(-1.0, 1.0)), math.radians(-8.0 + rng.uniform(-1.0, 1.0)),
                        math.radians(rng.uniform(-2.0, 2.0)), args.height_m)
            picture = camera.draw(pose, lat, lon)
            r, angle = rng.uniform(0.0, 10.0), rng.uniform(0.0, 2.0 * math.pi)   # the flight controller's own error
            prior = offset(lat, lon, r * math.cos(angle), r * math.sin(angle))
            local_ns = 10_000_000_000 + int(i * args.interval_s * 1e9)
            began = time.perf_counter()
            result = matcher.process(Job(local_ns, picture, prior[0], prior[1], pose, 60.0))
            took = time.perf_counter() - began
            if i < WARM_UP:
                continue
            times.append(took)
            for name in STAGES:
                if name in result.timings:
                    stages.setdefault(name, []).append(result.timings[name])
            stages.setdefault("the rest", []).append(max(0.0, took - sum(result.timings.values())))
            if result.ok:
                errors.append(distance_m(lat, lon, result.lat, result.lon))
            else:
                why.append(result.why.split(" (")[0])
        out(f"{args.pictures} pictures matched one after the other (the first {WARM_UP} not counted: start-up)")
        out(f"positions: {len(errors)} of {args.pictures}" + (f", off by {_spread(errors, 'm', 1)} m" if errors else ""))
        if why:
            out("no position: " + "; ".join(f"{w} ({why.count(w)})" for w in sorted(set(why))))
        out(f"time a picture: {_spread(times, 's', 3)} s")
        out("  " + ", ".join(f"{name} {statistics.median(v):.3f}" for name, v in stages.items()) + " (middle, s)")
        middle = statistics.median(times)
        out(f"one matching thread: about {1.0 / middle:.1f} pictures a second")
        if middle <= TWO_A_SECOND_S:
            verdict = "two pictures a second (vama_nav's default): yes"
        elif middle <= ONE_A_SECOND_S:
            verdict = "two pictures a second: too slow here, it would skip pictures; one a second (--interval-s 1): yes"
        else:
            verdict = "even one picture a second is too slow here"
        out(f"{verdict} (two a second need a picture in under {TWO_A_SECOND_S:g} s, leaving room for the camera and the rest)")
        return 0 if errors else 3
    finally:
        tile_map.close()
        ground.close()


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.pictures < 1 or args.interval_s <= 0 or args.height_m <= 0:
        print("--pictures, --interval-s and --height-m must be positive", file=sys.stderr)
        return 2
    from .xfeat_match import XFeatError

    try:
        return run(args)
    except MapError as exc:
        print(f"map: {exc}", file=sys.stderr)
        return 2
    except XFeatError as exc:
        print(f"xfeat: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
