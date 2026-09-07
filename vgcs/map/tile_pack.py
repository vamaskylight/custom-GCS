"""Map tiles for a machine that will never be online.

Field constraint 2026-09-07: "clients don't want to connect to the internet.
Never connect to the internet." Every earlier answer to a blank or blurred
offline map assumed the operator could go online once and press "Cache area
offline". That is not available to them, so the tiles have to travel: download
a **tile pack** on any connected PC, carry it on a USB stick, import it on the
air-gapped one.

Two things had to change for that to work at all.

**The existing cache button only ever stored one zoom level.** It asked the
interactive tile loader for the current zoom plus two deeper ones, but that
loader drops any request not at the view zoom (``_NativeTileLoader.request``,
by design, so a wheel zoom never waits on stale tiles). The deeper levels were
silently discarded, which is why zooming in offline went blurry, then black.
The pack downloader here does not go through the interactive loader.

**The cache was sized for one field, not one mission.** 3 km around the view
and a 4000-tile cap cannot hold a 10 km sortie. A pack covers the *plan* -
every waypoint plus a margin - across a zoom range, with an honest count
before anything is fetched.

A pack is just ``<root>/<z>/<x>/<y>.png`` plus a ``pack.json`` naming the tile
source. Importing copies it into VGCS's own disk cache under the source's
hashed folder, which the map already checks before the network. So the client
keeps the normal satellite source, needs no mode switch, and the deep-upscale
fallback still fills whatever the pack does not reach.
"""

from __future__ import annotations

import json
import math
import shutil
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Iterable

from vgcs.map import native_tile_map as _ntm

__all__ = [
    "PackPlan",
    "PackResult",
    "plan_pack_for_area",
    "plan_pack_for_bbox",
    "download_tile_pack",
    "import_tile_pack",
    "pack_manifest_path",
    "PACK_MAX_TILES",
]

# A 10 km mission at zoom 14-18 is roughly 7000 tiles / 150 MB. This cap leaves
# room for that with a wide margin, while still refusing the "cache the whole
# district" request that would look like a hang and fill a USB stick.
PACK_MAX_TILES = 25_000
# Around a plan: enough that the aircraft drifting off the line, or an RTL
# home, is still on imagery. In kilometres.
PACK_MARGIN_KM = 1.0
DEFAULT_PACK_ZOOMS = (14, 15, 16, 17, 18)
_MANIFEST = "pack.json"
_KM_PER_DEG_LAT = 111.32


@dataclass(frozen=True)
class PackPlan:
    """What a pack would contain, before a byte is fetched."""

    template: str
    source_id: str
    tiles: tuple[tuple[int, int, int], ...]
    zooms: tuple[int, ...]
    over_cap: int = 0
    bbox: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)  # lat0, lon0, lat1, lon1

    @property
    def count(self) -> int:
        return len(self.tiles)

    @property
    def approx_mb(self) -> float:
        # Esri imagery tiles run 15-30 KB; 20 KB is a fair planning figure.
        return self.count * 20.0 / 1024.0


@dataclass
class PackResult:
    """What a download or import actually did."""

    stored: int = 0
    skipped_existing: int = 0
    failed: int = 0
    placeholders: int = 0
    cancelled: bool = False
    failures: list[str] = field(default_factory=list)

    @property
    def total_present(self) -> int:
        return self.stored + self.skipped_existing


# --------------------------------------------------------------------------- #
# Planning
# --------------------------------------------------------------------------- #


def plan_pack_for_bbox(
    lat0: float,
    lon0: float,
    lat1: float,
    lon1: float,
    *,
    template: str,
    zooms: Iterable[int] = DEFAULT_PACK_ZOOMS,
    margin_km: float = PACK_MARGIN_KM,
    max_tiles: int = PACK_MAX_TILES,
) -> PackPlan:
    """Every tile covering the box (plus margin) at each zoom, capped.

    The cap is applied shallow-first: if the plan must be cut, it loses the
    deepest zoom's outer tiles, never the overview that shows where you are.
    """
    lo_lat, hi_lat = sorted((float(lat0), float(lat1)))
    lo_lon, hi_lon = sorted((float(lon0), float(lon1)))
    mid_lat = (lo_lat + hi_lat) / 2.0
    dlat = float(margin_km) / _KM_PER_DEG_LAT
    dlon = float(margin_km) / (_KM_PER_DEG_LAT * max(0.05, math.cos(math.radians(mid_lat))))
    lo_lat -= dlat
    hi_lat += dlat
    lo_lon -= dlon
    hi_lon += dlon

    zs = tuple(sorted({int(z) for z in zooms if 3 <= int(z) <= 22}))
    max_z = _ntm._max_zoom_for_template(template, max(zs) if zs else 19)
    zs = tuple(z for z in zs if z <= max_z)

    wanted: list[tuple[int, int, int]] = []
    for z in zs:
        x_lo, y_hi = _ntm._tile_xy(lo_lat, lo_lon, z)
        x_hi, y_lo = _ntm._tile_xy(hi_lat, hi_lon, z)
        for tx in range(min(x_lo, x_hi), max(x_lo, x_hi) + 1):
            for ty in range(min(y_lo, y_hi), max(y_lo, y_hi) + 1):
                wanted.append((z, tx, ty))

    cap = max(0, int(max_tiles))
    over = max(0, len(wanted) - cap)
    tiles = tuple(wanted[:cap])
    return PackPlan(
        template=str(template),
        source_id=_ntm._tile_source_id(str(template)),
        tiles=tiles,
        zooms=tuple(sorted({t[0] for t in tiles})),
        over_cap=over,
        bbox=(lo_lat, lo_lon, hi_lat, hi_lon),
    )


def plan_pack_for_area(
    lat: float,
    lon: float,
    *,
    radius_km: float,
    template: str,
    zooms: Iterable[int] = DEFAULT_PACK_ZOOMS,
    max_tiles: int = PACK_MAX_TILES,
) -> PackPlan:
    """A square ``radius_km`` around one point. For a site with no plan yet."""
    r = max(0.1, float(radius_km))
    dlat = r / _KM_PER_DEG_LAT
    dlon = r / (_KM_PER_DEG_LAT * max(0.05, math.cos(math.radians(float(lat)))))
    return plan_pack_for_bbox(
        lat - dlat, lon - dlon, lat + dlat, lon + dlon,
        template=template, zooms=zooms, margin_km=0.0, max_tiles=max_tiles,
    )


def plan_pack_for_waypoints(
    points: Iterable[tuple[float, float]],
    *,
    template: str,
    zooms: Iterable[int] = DEFAULT_PACK_ZOOMS,
    margin_km: float = PACK_MARGIN_KM,
    max_tiles: int = PACK_MAX_TILES,
) -> PackPlan | None:
    """The box around a mission's points. ``None`` when there are none."""
    pts = [(float(a), float(b)) for a, b in points if a is not None and b is not None]
    pts = [(a, b) for a, b in pts if abs(a) > 1e-7 or abs(b) > 1e-7]
    if not pts:
        return None
    lats = [p[0] for p in pts]
    lons = [p[1] for p in pts]
    return plan_pack_for_bbox(
        min(lats), min(lons), max(lats), max(lons),
        template=template, zooms=zooms, margin_km=margin_km, max_tiles=max_tiles,
    )


# --------------------------------------------------------------------------- #
# Download
# --------------------------------------------------------------------------- #


def pack_tile_path(root: Path, z: int, x: int, y: int) -> Path:
    return Path(root) / str(z) / str(x) / f"{y}.png"


def pack_manifest_path(root: Path) -> Path:
    return Path(root) / _MANIFEST


def _tile_url(template: str, z: int, x: int, y: int) -> str:
    return (
        str(template)
        .replace("{s}", "a")
        .replace("{z}", str(z))
        .replace("{x}", str(x))
        .replace("{y}", str(y))
    )


def download_tile_pack(
    plan: PackPlan,
    dest: Path,
    *,
    progress: Callable[[int, int, PackResult], None] | None = None,
    cancel: threading.Event | None = None,
    fetch: Callable[[str], bytes] | None = None,
    accept: Callable[[bytes, int], bool] | None = None,
) -> PackResult:
    """Fetch every tile in ``plan`` into ``dest/z/x/y.png``.

    Skips tiles already present, so an interrupted run resumes. Refuses to
    store a placeholder ("Map data not yet available") - keeping one would make
    that square permanently blank offline, where nothing will ever replace it.

    ``fetch`` and ``accept`` are injectable for tests; the defaults are the
    same HTTP path and placeholder test the live map uses.
    """
    dest = Path(dest)
    dest.mkdir(parents=True, exist_ok=True)
    fetch_fn = fetch or (lambda url: _ntm.fetch_tile_http_bytes(url, timeout_s=8.0))
    accept_fn = accept or _default_accept
    result = PackResult()
    total = plan.count

    # Written first so a half-finished pack still names its source, and an
    # import of it lands in the right cache folder.
    _write_manifest(dest, plan)

    for i, (z, x, y) in enumerate(plan.tiles):
        if cancel is not None and cancel.is_set():
            result.cancelled = True
            break
        p = pack_tile_path(dest, z, x, y)
        if p.is_file() and p.stat().st_size > 0:
            result.skipped_existing += 1
        else:
            try:
                raw = fetch_fn(_tile_url(plan.template, z, x, y))
                if not raw:
                    raise OSError("empty body")
                if not accept_fn(raw, z):
                    result.placeholders += 1
                else:
                    p.parent.mkdir(parents=True, exist_ok=True)
                    p.write_bytes(raw)
                    result.stored += 1
            except Exception as e:  # one bad tile must not end the pack
                result.failed += 1
                if len(result.failures) < 8:
                    result.failures.append(f"{z}/{x}/{y}: {type(e).__name__}: {e}")
        if progress is not None:
            progress(i + 1, total, result)
    return result


def _default_accept(raw: bytes, z: int) -> bool:
    from PySide6.QtGui import QImage

    img = QImage.fromData(raw)
    if img.isNull():
        return False
    return _ntm._accept_map_tile(img, raw_byte_len=len(raw), zoom=z) is not None


def _write_manifest(dest: Path, plan: PackPlan) -> None:
    try:
        pack_manifest_path(dest).write_text(
            json.dumps(
                {
                    "format": "vgcs-tile-pack/1",
                    "template": plan.template,
                    "source_id": plan.source_id,
                    "zooms": list(plan.zooms),
                    "bbox": list(plan.bbox),
                    "tiles": plan.count,
                },
                indent=2,
            ),
            encoding="utf-8",
        )
    except Exception:
        pass


# --------------------------------------------------------------------------- #
# Import
# --------------------------------------------------------------------------- #


def read_manifest(root: Path) -> dict | None:
    try:
        return json.loads(pack_manifest_path(root).read_text(encoding="utf-8"))
    except Exception:
        return None


def iter_pack_tiles(root: Path) -> Iterable[tuple[int, int, int, Path]]:
    """Every ``z/x/y.png`` under ``root``; anything else is ignored."""
    root = Path(root)
    if not root.is_dir():
        return
    for zdir in root.iterdir():
        if not zdir.is_dir() or not zdir.name.isdigit():
            continue
        for xdir in zdir.iterdir():
            if not xdir.is_dir() or not xdir.name.isdigit():
                continue
            for f in xdir.iterdir():
                if f.suffix.lower() == ".png" and f.stem.isdigit() and f.is_file():
                    yield int(zdir.name), int(xdir.name), int(f.stem), f


def import_tile_pack(
    src: Path,
    *,
    fallback_source_id: str = "",
    cache_root: Path | None = None,
    progress: Callable[[int, PackResult], None] | None = None,
) -> tuple[PackResult, str]:
    """Copy a pack into VGCS's disk cache. Returns ``(result, source_id)``.

    The cache is keyed by a hash of the tile URL template, and that hash is the
    same on every machine, so a pack made on one PC lands exactly where another
    PC's map will look for it. The manifest carries the source; without one,
    the caller's current source is assumed.
    """
    src = Path(src)
    manifest = read_manifest(src) or {}
    source_id = str(manifest.get("source_id") or "").strip()
    if not source_id and manifest.get("template"):
        source_id = _ntm._tile_source_id(str(manifest["template"]))
    if not source_id:
        source_id = str(fallback_source_id or "").strip()
    if not source_id:
        raise ValueError("this folder has no pack.json and no tile source is active")

    root = Path(cache_root) if cache_root is not None else _ntm._TILE_CACHE_ROOT
    result = PackResult()
    n = 0
    for z, x, y, f in iter_pack_tiles(src):
        n += 1
        target = root / source_id / str(z) / str(x) / f"{y}.png"
        try:
            if target.is_file() and target.stat().st_size == f.stat().st_size:
                result.skipped_existing += 1
            else:
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(f, target)
                result.stored += 1
        except Exception as e:
            result.failed += 1
            if len(result.failures) < 8:
                result.failures.append(f"{z}/{x}/{y}: {type(e).__name__}: {e}")
        if progress is not None and n % 200 == 0:
            progress(n, result)
    return result, source_id


def pack_covers(root: Path, lat: float, lon: float, z: int) -> bool:
    """Whether a pack folder has the tile under a point at a zoom."""
    x, y = _ntm._tile_xy(float(lat), float(lon), int(z))
    return pack_tile_path(Path(root), int(z), x, y).is_file()
