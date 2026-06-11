"""
M8 optional DEM — elevation lookup and ray–terrain intersection for hilly ground.

Supports:
- CSV point cloud (lat, lon, elev_m)
- ESRI ASCII Grid (.asc)
- GeoTIFF (.tif/.tiff) when ``rasterio`` is installed
"""

from __future__ import annotations

import csv
import math
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

_EARTH_RADIUS_M = 6_371_000.0
_CACHE_LOCK = threading.Lock()
_MODEL_CACHE: dict[str, DemElevationModel] = {}


@dataclass(frozen=True)
class DemElevationModel:
    """Sample ground elevation (m MSL) at WGS84 lat/lon."""

    kind: str
    path: str
    _fn: Callable[[float, float], float | None]

    def elevation_m(self, lat: float, lon: float) -> float | None:
        try:
            return self._fn(float(lat), float(lon))
        except Exception:
            return None

    @property
    def active(self) -> bool:
        return self.kind != "none"


def clear_dem_cache() -> None:
    with _CACHE_LOCK:
        _MODEL_CACHE.clear()


def _offset_lat_lon(lat_deg: float, lon_deg: float, north_m: float, east_m: float) -> tuple[float, float]:
    lat_rad = math.radians(lat_deg)
    dlat = north_m / _EARTH_RADIUS_M
    dlon = east_m / (_EARTH_RADIUS_M * max(1e-6, math.cos(lat_rad)))
    return lat_deg + math.degrees(dlat), lon_deg + math.degrees(dlon)


def _load_csv_points(path: Path) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                lat = float(row.get("lat") or row.get("latitude") or "")
                lon = float(row.get("lon") or row.get("longitude") or "")
                elev = float(
                    row.get("elev_m") or row.get("elevation_m") or row.get("elev") or ""
                )
                points.append((lat, lon, elev))
            except (TypeError, ValueError):
                continue
    return points


def _nearest_elevation(points: list[tuple[float, float, float]], lat: float, lon: float) -> float | None:
    if not points:
        return None
    best = None
    best_d = 1e30
    for plat, plon, pelev in points:
        d = (lat - plat) ** 2 + (lon - plon) ** 2
        if d < best_d:
            best_d = d
            best = pelev
    return best


def _load_esri_ascii(path: Path) -> tuple[
    list[list[float]], float, float, float, float, float | None
]:
    meta: dict[str, str] = {}
    rows: list[list[float]] = []
    with path.open(encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) == 2 and parts[0].lower() in (
                "ncols",
                "nrows",
                "xllcorner",
                "yllcorner",
                "xllcenter",
                "yllcenter",
                "cellsize",
                "nodata_value",
                "nodata_value",
            ):
                meta[parts[0].lower()] = parts[1]
                continue
            if len(parts) >= 2 and all(_is_float_token(p) for p in parts):
                rows.append([float(p) for p in parts])
    nrows = int(float(meta.get("nrows", len(rows) or 0)))
    ncols = int(float(meta.get("ncols", len(rows[0]) if rows else 0)))
    cell = float(meta.get("cellsize", 1.0))
    nodata_raw = meta.get("nodata_value")
    nodata = float(nodata_raw) if nodata_raw is not None else None
    if "xllcorner" in meta:
        x0 = float(meta["xllcorner"])
        y0 = float(meta["yllcorner"])
    else:
        x0 = float(meta.get("xllcenter", 0.0)) - cell / 2.0
        y0 = float(meta.get("yllcenter", 0.0)) - cell / 2.0
    if nrows <= 0 or ncols <= 0:
        return [], x0, y0, cell, cell, nodata
    grid = rows[:nrows]
    return grid, x0, y0, cell, cell, nodata


def _is_float_token(s: str) -> bool:
    try:
        float(s)
        return True
    except ValueError:
        return False


def _sample_esri_grid(
    grid: list[list[float]],
    x0: float,
    y0: float,
    cell_x: float,
    cell_y: float,
    nodata: float | None,
    lon: float,
    lat: float,
) -> float | None:
    if not grid or cell_x <= 0 or cell_y <= 0:
        return None
    nrows = len(grid)
    ncols = len(grid[0]) if grid else 0
    if ncols <= 0:
        return None
    # ESRI ASCII: first row is north (top); x increases east, y increases north in corner mode.
    col_f = (lon - x0) / cell_x - 0.5
    row_f = float(nrows) - (lat - y0) / cell_y - 0.5
    if col_f < 0 or row_f < 0 or col_f > ncols - 1 or row_f > nrows - 1:
        return None
    c0 = int(math.floor(col_f))
    r0 = int(math.floor(row_f))
    c1 = min(ncols - 1, c0 + 1)
    r1 = min(nrows - 1, r0 + 1)
    tx = col_f - c0
    ty = row_f - r0

    def _cell(r: int, c: int) -> float | None:
        try:
            v = float(grid[r][c])
        except (IndexError, TypeError, ValueError):
            return None
        if nodata is not None and abs(v - nodata) < 1e-3:
            return None
        return v

    v00 = _cell(r0, c0)
    v10 = _cell(r0, c1)
    v01 = _cell(r1, c0)
    v11 = _cell(r1, c1)
    vals = [v for v in (v00, v10, v01, v11) if v is not None]
    if not vals:
        return None
    if v00 is None or v10 is None or v01 is None or v11 is None:
        return sum(vals) / len(vals)
    return (
        v00 * (1 - tx) * (1 - ty)
        + v10 * tx * (1 - ty)
        + v01 * (1 - tx) * ty
        + v11 * tx * ty
    )


def _load_geotiff(path: Path) -> Callable[[float, float], float | None] | None:
    try:
        import rasterio  # type: ignore[import-untyped]
    except ImportError:
        return None
    try:
        ds = rasterio.open(path)
    except Exception:
        return None
    band = ds.read(1)
    nodata = ds.nodata
    transform = ds.transform

    def _sample(lat: float, lon: float) -> float | None:
        try:
            row, col = ds.index(lon, lat)
        except Exception:
            return None
        if row < 0 or col < 0 or row >= ds.height or col >= ds.width:
            return None
        try:
            v = float(band[row, col])
        except (IndexError, TypeError, ValueError):
            return None
        if nodata is not None and abs(v - float(nodata)) < 1e-3:
            return None
        if not math.isfinite(v):
            return None
        return v

    del transform  # keep ds alive via closure
    return _sample


def load_dem_model(path: str | Path | None) -> DemElevationModel | None:
    if path is None:
        return None
    p = Path(str(path).strip())
    if not p.is_file():
        return None
    suffix = p.suffix.lower()
    if suffix == ".csv":
        pts = _load_csv_points(p)
        if not pts:
            return None
        return DemElevationModel(
            kind="csv",
            path=str(p.resolve()),
            _fn=lambda lat, lon, _pts=pts: _nearest_elevation(_pts, lat, lon),
        )
    if suffix == ".asc":
        grid, x0, y0, cx, cy, nodata = _load_esri_ascii(p)
        if not grid:
            return None
        return DemElevationModel(
            kind="esri_ascii",
            path=str(p.resolve()),
            _fn=lambda lat, lon, g=grid, x0=x0, y0=y0, cx=cx, cy=cy, nd=nodata: _sample_esri_grid(
                g, x0, y0, cx, cy, nd, lon, lat
            ),
        )
    if suffix in (".tif", ".tiff"):
        fn = _load_geotiff(p)
        if fn is None:
            return None
        return DemElevationModel(kind="geotiff", path=str(p.resolve()), _fn=fn)
    return None


def get_shared_dem_model(path: str | Path | None) -> DemElevationModel | None:
    if path is None or not str(path).strip():
        return None
    key = str(Path(str(path).strip()).resolve())
    with _CACHE_LOCK:
        cached = _MODEL_CACHE.get(key)
        if cached is not None:
            return cached
        model = load_dem_model(key)
        if model is not None:
            _MODEL_CACHE[key] = model
        return model


def ray_intersect_terrain_msl(
    *,
    vehicle_lat: float,
    vehicle_lon: float,
    vehicle_alt_msl_m: float,
    dir_ned: tuple[float, float, float],
    elevation_m: Callable[[float, float], float | None],
    max_range_m: float = 4000.0,
    step_m: float = 3.0,
) -> tuple[float, float, float, float] | None:
    """
    March along a unit NED look ray until MSL altitude meets terrain elevation.

    Returns (north_m, east_m, horizontal_range_m, terrain_elev_m) or None.
    """
    dx, dy, dz = dir_ned
    mag = math.hypot(dx, dy, dz)
    if mag < 1e-9 or dz <= 1e-6:
        return None
    ux, uy, uz = dx / mag, dy / mag, dz / mag
    step = max(0.5, float(step_m))
    max_r = max(step, float(max_range_m))
    prev_t = 0.0
    prev_above = True
    t = step
    while t <= max_r:
        north = t * ux
        east = t * uy
        alt_msl = float(vehicle_alt_msl_m) - t * uz
        lat, lon = _offset_lat_lon(float(vehicle_lat), float(vehicle_lon), north, east)
        terr = elevation_m(lat, lon)
        if terr is None:
            prev_t = t
            t += step
            continue
        above = alt_msl > float(terr) + 0.35
        if not above and prev_above:
            # Refine between prev_t and t
            lo, hi = prev_t, t
            for _ in range(12):
                mid = (lo + hi) / 2.0
                mn = mid * ux
                me = mid * uy
                ma = float(vehicle_alt_msl_m) - mid * uz
                mlat, mlon = _offset_lat_lon(
                    float(vehicle_lat), float(vehicle_lon), mn, me
                )
                mterr = elevation_m(mlat, mlon)
                if mterr is None:
                    break
                if ma > float(mterr) + 0.2:
                    lo = mid
                else:
                    hi = mid
            hit_t = (lo + hi) / 2.0
            north_h = hit_t * ux
            east_h = hit_t * uy
            lat_h, lon_h = _offset_lat_lon(
                float(vehicle_lat), float(vehicle_lon), north_h, east_h
            )
            terr_h = elevation_m(lat_h, lon_h)
            if terr_h is None:
                terr_h = terr
            return north_h, east_h, math.hypot(north_h, east_h), float(terr_h)
        prev_above = above
        prev_t = t
        t += step
    return None
