"""
Vertical facade-plane width between two video marks (M8 measure).

Width on a wall = (slant range to the facade) × (horizontal angle between clicks in the video).
Ground lat/lon / bearing chord is not used (it measures the wrong plane).
"""

from __future__ import annotations

import math
from typing import Any

_MIN_FACADE_AGL_M = 2.5


def _video_xy(row: dict[str, Any]) -> tuple[float, float] | None:
    try:
        x = row.get("video_x_norm")
        y = row.get("video_y_norm")
        if x is None or y is None:
            return None
        return float(x), float(y)
    except (TypeError, ValueError):
        return None


def _observation_agl_m(row: dict[str, Any]) -> float | None:
    for key in ("rangefinder_down_m", "vehicle_rel_alt_m"):
        try:
            v = float(row.get(key) or 0)
        except (TypeError, ValueError):
            continue
        if v > 0.5:
            return v
    return None


def _horizontal_range_from_depression(agl_m: float, depression_deg: float) -> float | None:
    try:
        dep = float(depression_deg)
    except (TypeError, ValueError):
        return None
    if dep < 5.0 or dep > 89.0:
        return None
    return float(agl_m) / math.tan(math.radians(dep))


def _facade_agl_m(
    row_a: dict[str, Any],
    row_b: dict[str, Any],
    *,
    session_rf_floor_m: float | None = None,
) -> float | None:
    """AGL for facade width; ignore bogus high downward RF spikes (e.g. 45 m glitch)."""
    vals: list[float] = []
    for row in (row_a, row_b):
        v = _observation_agl_m(row)
        if v is not None and v > 0.5:
            vals.append(v)
    if not vals:
        return None
    agl = min(vals)
    floor = session_rf_floor_m
    if floor is not None and floor > 0.5:
        if agl > floor * 1.25:
            agl = floor
    if agl > 18.0:
        agl = 18.0
    return agl


def facade_plane_width_between_marks(
    row_a: dict[str, Any],
    row_b: dict[str, Any],
    *,
    hfov_deg: float = 62.0,
    session_rf_floor_m: float | None = None,
) -> float | None:
    """
    Width (m) on a vertical facade between two video clicks at similar height.

    Uses horizontal pixel span × HFOV and per-mark slant range from AGL + depression.
    """
    xy_a, xy_b = _video_xy(row_a), _video_xy(row_b)
    if xy_a is None or xy_b is None:
        return None
    dx = abs(xy_b[0] - xy_a[0])
    if dx < 0.03:
        return None

    agl = _facade_agl_m(row_a, row_b, session_rf_floor_m=session_rf_floor_m)
    if agl is None or agl < _MIN_FACADE_AGL_M:
        return None

    angle_h = dx * math.radians(float(hfov_deg))
    if angle_h <= 1e-5:
        return None

    rh_vals: list[float] = []
    for row in (row_a, row_b):
        dep = row.get("geo_depression_deg")
        if dep is not None:
            rh = _horizontal_range_from_depression(agl, float(dep))
            if rh is not None and rh > 0.5:
                rh_vals.append(rh)
        try:
            rg = float(row.get("geo_range_m") or 0)
        except (TypeError, ValueError):
            rg = 0.0
        if rg > 0.5:
            rh_vals.append(rg)

    if not rh_vals:
        return None

    rh = sum(rh_vals) / len(rh_vals)
    # Chord on the wall plane; rh * angle_h for small angles.
    if angle_h < 0.35:
        return rh * angle_h
    return 2.0 * rh * math.sin(angle_h / 2.0)
