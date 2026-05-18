"""On-disk cache for HTTP map tiles (Esri / OSM). Speeds repeat flights when WAN is unavailable."""

from __future__ import annotations

import hashlib
import os
import re
from pathlib import Path

from PySide6.QtGui import QImage

_CACHE_ROOT: Path | None = None
_ENABLED: bool | None = None

# Alternate providers (used when the active template has no cache hit).
_FALLBACK_TEMPLATES: tuple[str, ...] = (
    "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
    "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
    "https://server.arcgisonline.com/ArcGIS/rest/services/World_Street_Map/MapServer/tile/{z}/{y}/{x}",
)


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


def read_cached_tile_loose(
    z: int,
    x: int,
    y: int,
    *,
    root: Path | None = None,
) -> QImage:
    """Load ``z/x/y.png`` from any provider folder under the cache root."""
    if not is_enabled():
        return QImage()
    base = root if root is not None else default_cache_root()
    try:
        if not base.is_dir():
            return QImage()
        for src_dir in base.iterdir():
            if not src_dir.is_dir():
                continue
            path = src_dir / str(int(z)) / str(int(x)) / f"{int(y)}.png"
            try:
                if path.is_file() and path.stat().st_size >= 64:
                    img = QImage(str(path))
                    if not img.isNull():
                        return img
            except OSError:
                continue
    except OSError:
        pass
    return QImage()


def read_cached_tile_any(
    template: str,
    z: int,
    x: int,
    y: int,
    *,
    root: Path | None = None,
) -> QImage:
    """Try active template, known fallbacks, then any cached provider at ``z/x/y``."""
    if not is_enabled():
        return QImage()
    primary = str(template or "").strip()
    tried: set[str] = set()
    for tmpl in (primary, *_FALLBACK_TEMPLATES):
        if not tmpl or tmpl == "{local}" or tmpl in tried:
            continue
        tried.add(tmpl)
        img = read_cached_tile(tmpl, z, x, y, root=root)
        if not img.isNull():
            return img
    return read_cached_tile_loose(z, x, y, root=root)


def _tile_xy_slippy(lat_deg: float, lon_deg: float, zoom: int) -> tuple[int, int]:
    import math

    lat_rad = math.radians(max(-85.0511, min(85.0511, float(lat_deg))))
    n = 2.0**int(zoom)
    xt = int((float(lon_deg) + 180.0) / 360.0 * n)
    yt = int(
        (1.0 - math.log(math.tan(lat_rad) + (1.0 / math.cos(lat_rad))) / math.pi) / 2.0 * n
    )
    return max(0, xt), max(0, yt)


def count_cached_tiles_near(
    lat: float,
    lon: float,
    zoom: int,
    *,
    radius: int = 2,
    root: Path | None = None,
) -> int:
    """Count cached PNG tiles in a ``(2r+1)²`` block (any provider)."""
    if not is_enabled():
        return 0
    cx, cy = _tile_xy_slippy(lat, lon, zoom)
    r = max(0, int(radius))
    n = 0
    for dx in range(-r, r + 1):
        for dy in range(-r, r + 1):
            x = cx + dx
            y = cy + dy
            if x < 0 or y < 0:
                continue
            img = read_cached_tile_loose(zoom, x, y, root=root)
            if not img.isNull():
                n += 1
    return n
