"""The map the drone carries for the GPS-denied navigation (item A8, step 4).

``drone/gps_denied/vama_nav`` finds the drone's position by matching the down
camera's pictures with satellite pictures stored on the drone's Raspberry Pi:
one MBTiles file (map tiles in one SQLite file) at one zoom level. VGCS makes
that file from the same download as a tile pack (``tile_pack.py``): the tiles of
the flight area at ``DRONE_MAP_ZOOM``, fetched into VGCS's own map cache (tiles
already there are not fetched again, and the PC's map gets them too), then
packed into the one file the Pi reads. It is copied to the Pi by hand (a USB
stick, or over the network with the Pi on the bench): the radio link to the
flight controller cannot carry a gigabyte.

The zoom: the matching turns each picture north up at the map's scale. Zoom 17
is about 1.1 m a pixel at the client's latitude (20 degrees): from 500 m the
camera's picture is about 600 map pixels across, which holds plenty of
features, and 30 km around home is about 44,000 tiles (about 0.9 GB).

The pictures must be allowed for use offline on the drone. The usual online
maps (Google, Bing) do not allow it, and Esri's terms are not checked here: the
operator's choice of source, said where the file is made.

The ground's heights: over hills the matching needs them. They are a second small
MBTiles file beside the map (where ``vama_nav`` finds it), from the open Terrain Tiles
(on land mostly SRTM, 30 m, about zoom 12: a finer tile only repeats it). 30 km
around home is about 64 tiles, about 3 MB. There is no one licence for the set:
each source has its own terms, and credit is asked (``HEIGHTS_ATTRIBUTION``, kept
in the file). The pages call SRTM public domain, and also quote a copyright asserted
outside the United States. ETOPO1, which fills the sea and gaps, says it is not to
be used for navigation: the drone reads any height below 0 as the sea's level
(vama_nav/heights.py). The drone's README has what was read.
"""

from __future__ import annotations

import math
import sqlite3
import threading
from collections.abc import Callable, Iterable
from dataclasses import dataclass
from pathlib import Path

from vgcs.map.tile_pack import PackPlan, pack_tile_path, plan_pack_for_area, plan_pack_for_bbox, plan_pack_for_waypoints

DRONE_MAP_ZOOM = 17
# Around a mission: room for a detour, a drift off the line and the way home.
DRONE_MAP_MARGIN_KM = 2.0
# 30 km around home at zoom 17 is about 44,000 tiles. Twice that, then no more:
# a bigger file is a mistake (a whole state), not a flight area.
DRONE_MAP_MAX_TILES = 90_000
DRONE_MAP_FILE = "vama-map.mbtiles"

HEIGHTS_TEMPLATE = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png"
HEIGHTS_ZOOM = 12
HEIGHTS_ENCODING = "terrarium"              # the metadata vama_nav/heights.py looks for
# The credit the Terrain Tiles' pages ask for, all their sources: the file can be of any place.
HEIGHTS_ATTRIBUTION = (
    "Ground heights from the Terrain Tiles, https://registry.opendata.aws/terrain-tiles. "
    "ArcticDEM terrain data DEM(s) were created from DigitalGlobe, Inc., imagery and funded under National Science "
    "Foundation awards 1043681, 1559691, and 1542736; "
    "Australia terrain data \u00a9 Commonwealth of Australia (Geoscience Australia) 2017; "
    "Austria terrain data \u00a9 offene Daten \u00d6sterreichs - Digitales Gel\u00e4ndemodell (DGM) \u00d6sterreich; "
    "Canada terrain data contains information licensed under the Open Government Licence - Canada; "
    "Europe terrain data produced using Copernicus data and information funded by the European Union - EU-DEM layers; "
    "Global ETOPO1 terrain data U.S. National Oceanic and Atmospheric Administration; "
    "Mexico terrain data source: INEGI, Continental relief, 2016; "
    "New Zealand terrain data Copyright 2011 Crown copyright (c) Land Information New Zealand and the New Zealand "
    "Government (All rights reserved); "
    "Norway terrain data \u00a9 Kartverket; "
    "United Kingdom terrain data \u00a9 Environment Agency copyright and/or database right 2015. All rights reserved; "
    "United States 3DEP (formerly NED) and global GMTED2010 and SRTM terrain data courtesy of the U.S. Geological Survey."
)


@dataclass
class DroneMapResult:
    """What was written: the tiles in the file, the ones the download could not get."""

    path: Path | None = None
    tiles: int = 0
    missing: int = 0
    size_bytes: int = 0
    fmt: str = ""
    bounds: tuple[float, float, float, float] | None = None     # west, south, east, north
    cancelled: bool = False
    error: str = ""
    heights: DroneMapResult | None = None                       # the file of the ground's heights made beside the map


def plan_drone_map(
    points: Iterable[tuple[float, float]],
    *,
    template: str,
    center: tuple[float, float] | None = None,
    radius_km: float = 5.0,
    zoom: int = DRONE_MAP_ZOOM,
    margin_km: float = DRONE_MAP_MARGIN_KM,
    max_tiles: int = DRONE_MAP_MAX_TILES,
) -> PackPlan | None:
    """The tiles of the flight area at ``zoom``: the box round ``points`` (a mission and
    home) plus ``margin_km``, or without points a square ``radius_km`` round ``center``.
    None when there is neither."""
    plan = plan_pack_for_waypoints(points, template=template, zooms=(int(zoom),), margin_km=margin_km,
                                   max_tiles=max_tiles)
    if plan is not None:
        return plan
    if center is None:
        return None
    return plan_pack_for_area(center[0], center[1], radius_km=radius_km, template=template, zooms=(int(zoom),),
                              max_tiles=max_tiles)


def plan_drone_heights(plan: PackPlan, *, margin_km: float = DRONE_MAP_MARGIN_KM) -> PackPlan:
    """The ground's heights of a map's area. With room round it: the drone can be beyond the
    map's edge while its camera, leaning, still sees the map."""
    lat0, lon0, lat1, lon1 = plan.bbox
    return plan_pack_for_bbox(lat0, lon0, lat1, lon1, template=HEIGHTS_TEMPLATE, zooms=(HEIGHTS_ZOOM,), margin_km=margin_km)


def drone_heights_path(map_path: Path) -> Path:
    """The heights file of a map file: beside it, its name with "-heights" (as vama_nav/heights.py looks for it)."""
    path = Path(map_path)
    return path.with_name(f"{path.stem}-heights{path.suffix}")


def accept_heights_tile(raw: bytes, _zoom: int) -> bool:
    """Any PNG picture. The map's test for a placeholder is not for these: flat ground and the
    sea are one colour."""
    return _tile_format(raw) == "png"


def _tile_format(data: bytes) -> str:
    if data[:2] == b"\xff\xd8":
        return "jpg"
    if data[:8] == b"\x89PNG\r\n\x1a\n":
        return "png"
    if data[:4] == b"RIFF" and data[8:12] == b"WEBP":
        return "webp"
    return ""


def _tile_corner(z: int, x: int, y: int) -> tuple[float, float]:
    """The north-west corner of tile (x, y) of zoom z: (lat, lon)."""
    n = 2 ** z
    lon = x / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * y / n))))
    return lat, lon


def write_drone_map(
    pack_dir: Path,
    out_path: Path,
    plan: PackPlan,
    *,
    name: str = "VAMA drone map",
    attribution: str = "",
    description: str = "VAMA GPS-denied navigation: the drone's map (vama_nav --map)",
    encoding: str = "",
    cancel: threading.Event | None = None,
    progress: Callable[[int, int], None] | None = None,
) -> DroneMapResult:
    """Pack the plan's tiles found in ``pack_dir`` (``z/x/y.png``, as tile_pack keeps them)
    into one MBTiles file at ``out_path``. Rows count from the bottom (MBTiles' rule). The
    file is written beside it first and put in place at the end, so a stopped run leaves no
    half map where the Pi would read one."""
    out_path = Path(out_path)
    result = DroneMapResult()
    tmp = out_path.with_name(out_path.name + ".part")
    tmp.unlink(missing_ok=True)
    formats: set[str] = set()
    west = south = math.inf
    east = north = -math.inf
    zooms: set[int] = set()
    db = sqlite3.connect(tmp)
    try:
        db.executescript("CREATE TABLE metadata (name TEXT, value TEXT);"
                         "CREATE TABLE tiles (zoom_level INTEGER, tile_column INTEGER, tile_row INTEGER, tile_data BLOB);")
        total = plan.count
        for i, (z, x, y) in enumerate(plan.tiles):
            if cancel is not None and cancel.is_set():
                result.cancelled = True
                break
            tile = pack_tile_path(Path(pack_dir), z, x, y)
            try:
                data = tile.read_bytes() if tile.is_file() else b""
            except OSError:
                data = b""
            if not data:
                result.missing += 1
            else:
                db.execute("INSERT INTO tiles VALUES (?, ?, ?, ?)", (z, x, (1 << z) - 1 - y, data))
                result.tiles += 1
                zooms.add(z)
                kind = _tile_format(data)
                if kind:
                    formats.add(kind)
                n_lat, w_lon = _tile_corner(z, x, y)
                s_lat, e_lon = _tile_corner(z, x + 1, y + 1)
                west, south, east, north = min(west, w_lon), min(south, s_lat), max(east, e_lon), max(north, n_lat)
            if progress is not None and (i + 1 == total or (i + 1) % 500 == 0):
                progress(i + 1, total)
        if result.cancelled or result.tiles == 0:
            if not result.cancelled:
                result.error = "none of the area's map pictures are on this PC: the download got none"
            db.close()
            tmp.unlink(missing_ok=True)
            return result
        db.execute("CREATE UNIQUE INDEX tile_index ON tiles (zoom_level, tile_column, tile_row)")
        result.fmt = formats.pop() if len(formats) == 1 else ("mixed" if formats else "png")
        result.bounds = (west, south, east, north)
        meta = {"name": name, "format": "png" if result.fmt == "mixed" else result.fmt, "type": "baselayer",
                "version": "1.0", "description": description,
                "attribution": attribution, "minzoom": str(min(zooms)), "maxzoom": str(max(zooms)),
                "bounds": ",".join(f"{v:.7f}" for v in result.bounds)}
        if encoding:
            meta["encoding"] = encoding
        db.executemany("INSERT INTO metadata VALUES (?, ?)", sorted(meta.items()))
        db.commit()
    except BaseException:
        db.close()
        tmp.unlink(missing_ok=True)
        raise
    db.close()
    tmp.replace(out_path)
    result.path = out_path
    result.size_bytes = out_path.stat().st_size
    return result


def write_drone_heights(
    pack_dir: Path,
    out_path: Path,
    plan: PackPlan,
    *,
    cancel: threading.Event | None = None,
) -> DroneMapResult:
    """Pack the plan's heights tiles found in ``pack_dir`` into one MBTiles file, as ``write_drone_map``."""
    return write_drone_map(pack_dir, out_path, plan, name="VAMA ground heights", attribution=HEIGHTS_ATTRIBUTION,
                           description="VAMA GPS-denied navigation: the ground's heights (vama_nav --heights)",
                           encoding=HEIGHTS_ENCODING, cancel=cancel)


def read_drone_map(path: Path) -> dict:
    """What a drone map file holds: its tile count, zooms and metadata. Raises ValueError
    when it is not one."""
    try:
        db = sqlite3.connect(f"file:{Path(path).as_posix()}?mode=ro", uri=True)
        try:
            meta = dict(db.execute("SELECT name, value FROM metadata").fetchall())
            zooms = sorted(z for (z,) in db.execute("SELECT DISTINCT zoom_level FROM tiles"))
            tiles = int(db.execute("SELECT COUNT(*) FROM tiles").fetchone()[0])
        finally:
            db.close()
    except sqlite3.Error as exc:
        raise ValueError(f"{path} is not a map file (MBTiles): {exc}") from None
    return {"tiles": tiles, "zooms": zooms, "meta": meta}


def describe_drone_map_plan(plan: PackPlan) -> str:
    """For the question before the download: how many pictures, how big, which zoom."""
    zoom = plan.zooms[0] if plan.zooms else DRONE_MAP_ZOOM
    lat0, lon0, lat1, lon1 = plan.bbox
    mid = math.radians((lat0 + lat1) / 2.0)
    m_per_px = 40_075_016.686 * math.cos(mid) / (256 * 2 ** zoom)
    north_km = abs(lat1 - lat0) * 111.32
    east_km = abs(lon1 - lon0) * 111.32 * math.cos(mid)
    text = (f"{plan.count} map pictures at zoom {zoom} (about {m_per_px:.1f} m a pixel), about "
            f"{plan.approx_mb:.0f} MB, an area of {north_km:.1f} by {east_km:.1f} km")
    if plan.over_cap:
        text += f". {plan.over_cap} more were over the limit of {DRONE_MAP_MAX_TILES} and left out: make the area smaller"
    return text
