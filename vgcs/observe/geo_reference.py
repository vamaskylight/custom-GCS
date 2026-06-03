"""
M8 — DOOAF (OO) orientation & geo-referencing (non-weapon).

Estimates a ground target lat/lon from vehicle pose, gimbal attitude, and a normalized
video click using a flat-earth ray–ground intersection (optional DEM offset).
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

_EARTH_RADIUS_M = 6_371_000.0


@dataclass(frozen=True)
class GeoReferenceResult:
    ok: bool
    target_lat: float | None = None
    target_lon: float | None = None
    target_alt_m: float | None = None
    horizontal_range_m: float | None = None
    quality: str = "insufficient"
    warning: str = ""
    method: str = "none"
    bearing_deg: float | None = None


def _deg2rad(d: float) -> float:
    return math.radians(float(d))


def _rot_x(roll_rad: float) -> list[list[float]]:
    c, s = math.cos(roll_rad), math.sin(roll_rad)
    return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]


def _rot_y(pitch_rad: float) -> list[list[float]]:
    c, s = math.cos(pitch_rad), math.sin(pitch_rad)
    return [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]


def _rot_z(yaw_rad: float) -> list[list[float]]:
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


def _mat_mul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    out = [[0.0, 0.0, 0.0] for _ in range(3)]
    for i in range(3):
        for j in range(3):
            out[i][j] = sum(a[i][k] * b[k][j] for k in range(3))
    return out


def _mat_vec(m: list[list[float]], v: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    )


def _offset_lat_lon(lat_deg: float, lon_deg: float, north_m: float, east_m: float) -> tuple[float, float]:
    lat_rad = _deg2rad(lat_deg)
    dlat = north_m / _EARTH_RADIUS_M
    dlon = east_m / (_EARTH_RADIUS_M * max(1e-6, math.cos(lat_rad)))
    return (lat_deg + math.degrees(dlat), lon_deg + math.degrees(dlon))


def _quality_label(
    *,
    gps_fix_type: int,
    gps_hdop: float | None,
    has_gimbal: bool,
    depression_deg: float,
    range_m: float,
) -> tuple[str, str]:
    warnings: list[str] = []
    if gps_fix_type < 3:
        warnings.append(f"GPS fix={gps_fix_type} (need 3D fix for best accuracy)")
    if gps_hdop is not None and gps_hdop > 2.0:
        warnings.append(f"GPS HDOP {gps_hdop:.1f} > 2.0")
    if not has_gimbal:
        warnings.append("gimbal attitude missing")
    if depression_deg < 5.0:
        warnings.append("look angle near horizon")
    if range_m > 5000.0:
        warnings.append("range > 5 km (flat-ground assumption weak)")

    if gps_fix_type < 2 or not has_gimbal or depression_deg < 1.0:
        return "insufficient", "; ".join(warnings) or "insufficient telemetry"

    if warnings:
        return "fair", "; ".join(warnings)
    if gps_fix_type >= 3 and (gps_hdop is None or gps_hdop <= 1.5) and depression_deg >= 15.0:
        return "good", ""
    return "fair", "; ".join(warnings)


class _DemLookup:
    """Optional nearest-neighbour DEM samples from a simple CSV (lat, lon, elev_m)."""

    def __init__(self, path: str | Path | None) -> None:
        self._points: list[tuple[float, float, float]] = []
        p = Path(path) if path else None
        if p is None or not p.is_file():
            return
        try:
            with p.open(newline="", encoding="utf-8") as f:
                r = csv.DictReader(f)
                for row in r:
                    lat = float(row.get("lat") or row.get("latitude") or "")
                    lon = float(row.get("lon") or row.get("longitude") or "")
                    elev = float(row.get("elev_m") or row.get("elevation_m") or row.get("elev") or "")
                    self._points.append((lat, lon, elev))
        except Exception:
            self._points.clear()

    def elevation_m(self, lat: float, lon: float) -> float | None:
        if not self._points:
            return None
        best = None
        best_d = 1e30
        for plat, plon, pelev in self._points:
            d = (lat - plat) ** 2 + (lon - plon) ** 2
            if d < best_d:
                best_d = d
                best = pelev
        return best


def compute_geo_reference(
    *,
    vehicle_lat: float | None,
    vehicle_lon: float | None,
    vehicle_heading_deg: float | None,
    vehicle_roll_deg: float | None = None,
    vehicle_pitch_deg: float | None = None,
    vehicle_rel_alt_m: float | None,
    vehicle_alt_msl_m: float | None = None,
    gimbal_yaw_deg: float | None,
    gimbal_pitch_deg: float | None,
    video_x_norm: float,
    video_y_norm: float,
    gps_fix_type: int = 0,
    gps_hdop: float | None = None,
    camera_hfov_deg: float = 62.0,
    camera_vfov_deg: float | None = None,
    dem_path: str | Path | None = None,
    dem_lookup: Callable[[float, float], float | None] | None = None,
) -> GeoReferenceResult:
    """
    Estimate ground intersection for a normalized video click (0..1, top-left origin).

    Body FRD (+X forward, +Y right, +Z down) and NED (+X north, +Y east, +Z down).
    Gimbal yaw about +Z, pitch about +Y (positive pitch = camera looks up).
    """
    if vehicle_lat is None or vehicle_lon is None:
        return GeoReferenceResult(ok=False, warning="vehicle position missing", method="none")
    if vehicle_rel_alt_m is None or float(vehicle_rel_alt_m) <= 0.5:
        return GeoReferenceResult(
            ok=False,
            warning="vehicle altitude AGL unknown or too low",
            method="none",
        )
    if gimbal_yaw_deg is None or gimbal_pitch_deg is None:
        return GeoReferenceResult(ok=False, warning="gimbal yaw/pitch missing", method="none")

    hfov = max(5.0, min(120.0, float(camera_hfov_deg)))
    vfov = float(camera_vfov_deg) if camera_vfov_deg is not None else hfov * 0.5625
    vfov = max(5.0, min(90.0, vfov))

    u = max(0.0, min(1.0, float(video_x_norm)))
    v = max(0.0, min(1.0, float(video_y_norm)))
    az_off = (u - 0.5) * hfov
    el_off = (v - 0.5) * vfov

    roll = _deg2rad(float(vehicle_roll_deg or 0.0))
    pitch = _deg2rad(float(vehicle_pitch_deg or 0.0))
    hdg = _deg2rad(float(vehicle_heading_deg or 0.0))
    g_yaw = _deg2rad(float(gimbal_yaw_deg))
    g_pitch = _deg2rad(float(gimbal_pitch_deg))

    r_ned_body = _mat_mul(_rot_z(hdg), _mat_mul(_rot_y(pitch), _rot_x(roll)))
    r_body_gimbal = _mat_mul(_rot_z(g_yaw), _rot_y(g_pitch))
    r_gimbal_cam = _mat_mul(_rot_y(_deg2rad(el_off)), _rot_z(_deg2rad(az_off)))
    r_ned_cam = _mat_mul(r_ned_body, _mat_mul(r_body_gimbal, r_gimbal_cam))
    dir_ned = _mat_vec(r_ned_cam, (1.0, 0.0, 0.0))

    dz = dir_ned[2]
    if dz <= 1e-4:
        return GeoReferenceResult(
            ok=False,
            warning="look ray does not intersect ground (near horizon)",
            method="ray_ground",
        )

    agl_m = float(vehicle_rel_alt_m)
    lookup = dem_lookup
    if lookup is None and dem_path:
        dem = _DemLookup(dem_path)
        if dem._points:
            lookup = dem.elevation_m

    ground_z_ned = agl_m
    method = "ray_ground_flat"
    if lookup is not None and vehicle_alt_msl_m is not None:
        try:
            elev = lookup(float(vehicle_lat), float(vehicle_lon))
            if elev is not None:
                ground_z_ned = max(0.5, float(vehicle_alt_msl_m) - float(elev))
                method = "ray_ground_dem"
        except Exception:
            pass

    t = ground_z_ned / dz
    north_m = t * dir_ned[0]
    east_m = t * dir_ned[1]
    range_m = math.hypot(north_m, east_m)
    bearing = (math.degrees(math.atan2(east_m, north_m)) + 360.0) % 360.0
    depression = math.degrees(math.atan2(dz, math.hypot(dir_ned[0], dir_ned[1])))

    tgt_lat, tgt_lon = _offset_lat_lon(float(vehicle_lat), float(vehicle_lon), north_m, east_m)
    tgt_alt: float | None = None
    if lookup is not None:
        try:
            tgt_alt = lookup(tgt_lat, tgt_lon)
        except Exception:
            tgt_alt = None
    if tgt_alt is None and vehicle_alt_msl_m is not None:
        tgt_alt = float(vehicle_alt_msl_m) - agl_m

    quality, warn = _quality_label(
        gps_fix_type=int(gps_fix_type or 0),
        gps_hdop=gps_hdop,
        has_gimbal=True,
        depression_deg=depression,
        range_m=range_m,
    )
    ok = quality != "insufficient"
    return GeoReferenceResult(
        ok=ok,
        target_lat=tgt_lat,
        target_lon=tgt_lon,
        target_alt_m=tgt_alt,
        horizontal_range_m=range_m,
        quality=quality,
        warning=warn,
        method=method,
        bearing_deg=bearing,
    )
