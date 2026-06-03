"""Distance and track helpers for observation targets (M8 measure)."""

from __future__ import annotations

import math
from typing import Any

_EARTH_RADIUS_M = 6_371_000.0


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r = _EARTH_RADIUS_M
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(min(1.0, math.sqrt(max(0.0, a))))


def observation_target_latlon(row: dict[str, Any]) -> tuple[float, float] | None:
    """Ground/map target for a logged observation row."""
    lat = row.get("target_lat")
    lon = row.get("target_lon")
    if lat is None or lon is None:
        lat = row.get("map_lat")
        lon = row.get("map_lon")
    if lat is None or lon is None:
        return None
    try:
        la = float(lat)
        lo = float(lon)
    except (TypeError, ValueError):
        return None
    if not (math.isfinite(la) and math.isfinite(lo)):
        return None
    if abs(la) < 1e-9 and abs(lo) < 1e-9:
        return None
    return la, lo


def target_track_from_observations(rows: list[dict[str, Any]]) -> list[tuple[float, float]]:
    out: list[tuple[float, float]] = []
    for row in rows:
        pt = observation_target_latlon(row)
        if pt is not None:
            out.append(pt)
    return out


# MAV_SENSOR_ROTATION_PITCH_270 — downward rangefinder on ArduPilot.
_MAV_ORIENTATION_PITCH_270 = 25


def is_downward_sensor_orientation(orientation: int) -> bool:
    o = int(orientation)
    return o in (_MAV_ORIENTATION_PITCH_270, 24, 26)


def resolve_vehicle_agl_m(
    *,
    relative_alt_m: float | None,
    rangefinder_down_m: float | None = None,
) -> tuple[float | None, str]:
    """
    Best-effort height above ground for M8 ray intersection.

    EKF ``relative_alt`` is often 0 on the bench; downward rangefinder (common on logs)
    is used as fallback when present.
    """
    try:
        rel = float(relative_alt_m) if relative_alt_m is not None else None
    except (TypeError, ValueError):
        rel = None
    if rel is not None and rel > 0.5:
        return rel, "ekf_relative"
    try:
        rf = float(rangefinder_down_m) if rangefinder_down_m is not None else None
    except (TypeError, ValueError):
        rf = None
    if rf is not None and 0.15 < rf < 200.0:
        return rf, "rangefinder_down"
    if rel is not None and rel > 0.05:
        return max(rel, 0.5), "ekf_relative_low"
    return None, ""


def segment_distances_m(track: list[tuple[float, float]]) -> list[float]:
    """Ground distance (m) for each consecutive pair in ``track``."""
    segs: list[float] = []
    for i in range(1, len(track)):
        a = track[i - 1]
        b = track[i]
        segs.append(haversine_m(a[0], a[1], b[0], b[1]))
    return segs
