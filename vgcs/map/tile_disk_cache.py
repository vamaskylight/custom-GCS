"""On-disk cache for HTTP map tiles (Esri / OSM). Speeds repeat flights when WAN is unavailable."""

from __future__ import annotations

import hashlib
import os
import re
from pathlib import Path

from PySide6.QtGui import QImage

_CACHE_ROOT: Path | None = None
_ENABLED: bool | None = None


def _env_disabled() -> bool:
    return str(os.environ.get("VGCS_MAP_TILE_CACHE", "1") or "1").strip() == "0"


def is_enabled() -> bool:
    global _ENABLED
    if _ENABLED is None:
        _ENABLED = not _env_disabled()
    return bool(_ENABLED)


def default_cache_root() -> Path:
    """``~/.vgcs/tile-cache`` unless ``VGCS_MAP_TILE_CACHE_DIR`` is set."""
    global _CACHE_ROOT
    if _CACHE_ROOT is not None:
        return _CACHE_ROOT
    raw = str(os.environ.get("VGCS_MAP_TILE_CACHE_DIR", "") or "").strip()
    if raw:
        _CACHE_ROOT = Path(raw).expanduser().resolve()
    else:
        _CACHE_ROOT = (Path.home() / ".vgcs" / "tile-cache").resolve()
    try:
        _CACHE_ROOT.mkdir(parents=True, exist_ok=True)
    except OSError:
        pass
    return _CACHE_ROOT


def set_cache_root(path: str | Path | None) -> None:
    """Override cache directory (e.g. from QSettings). Pass ``None`` to reset to default."""
    global _CACHE_ROOT
    if path is None or not str(path).strip():
        _CACHE_ROOT = None
        return
    p = Path(path).expanduser().resolve()
    p.mkdir(parents=True, exist_ok=True)
    _CACHE_ROOT = p


def template_cache_key(template: str) -> str:
    """
    Stable folder name for a tile URL template (``{z}/{x}/{y}``), ignoring ``{s}`` subdomain.
    """
    t = str(template or "").strip()
    t = re.sub(r"\{s\}", "a", t)
    return hashlib.sha256(t.encode("utf-8")).hexdigest()[:20]


def tile_cache_path(
    template: str,
    z: int,
    x: int,
    y: int,
    *,
    root: Path | None = None,
) -> Path:
    base = root if root is not None else default_cache_root()
    return base / template_cache_key(template) / str(int(z)) / str(int(x)) / f"{int(y)}.png"


def read_cached_tile(
    template: str,
    z: int,
    x: int,
    y: int,
    *,
    root: Path | None = None,
) -> QImage:
    if not is_enabled() or not template or template == "{local}":
        return QImage()
    path = tile_cache_path(template, z, x, y, root=root)
    try:
        if not path.is_file() or path.stat().st_size < 64:
            return QImage()
        img = QImage(str(path))
        return img if not img.isNull() else QImage()
    except OSError:
        return QImage()


def write_cached_tile(
    template: str,
    z: int,
    x: int,
    y: int,
    img: QImage,
    *,
    root: Path | None = None,
) -> bool:
    if not is_enabled() or not template or template == "{local}":
        return False
    if img is None or img.isNull():
        return False
    path = tile_cache_path(template, z, x, y, root=root)
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(".png.part")
        if not img.save(str(tmp), "PNG"):
            try:
                tmp.unlink(missing_ok=True)
            except OSError:
                pass
            return False
        tmp.replace(path)
        return True
    except OSError:
        return False


def write_cached_tile_bytes(
    template: str,
    z: int,
    x: int,
    y: int,
    raw: bytes,
    *,
    root: Path | None = None,
) -> bool:
    if not raw or len(raw) < 64:
        return False
    img = QImage.fromData(raw)
    if img.isNull():
        return False
    return write_cached_tile(template, z, x, y, img, root=root)
