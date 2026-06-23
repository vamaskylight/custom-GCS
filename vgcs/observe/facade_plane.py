"""
Vertical facade-plane width between two video marks (M8 measure).

Uses per-click horizontal range + bearing (camera ray), NOT haversine between
ground lat/lon (often 10–50 m wrong when clicks are on roofs/walls).
"""

from __future__ import annotations

import math
from typing import Any

_MIN_FACADE_AGL_M = 2.5
_MIN_VERTICAL_DY_NORM = 0.04
_MAX_VERTICAL_DX_NORM = 0.10


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


def _law_of_cosines_m(
    r1: float, r2: float, bearing_a_deg: float, bearing_b_deg: float
) -> float | None:
    try:
        ba = math.radians(float(bearing_a_deg))
        bb = math.radians(float(bearing_b_deg))
        ra = float(r1)
        rb = float(r2)
        if ra <= 0 or rb <= 0:
            return None
        return math.sqrt(ra * ra + rb * rb - 2.0 * ra * rb * math.cos(bb - ba))
    except (TypeError, ValueError):
        return None


def _pair_facade_agl_m(row_a: dict[str, Any], row_b: dict[str, Any]) -> float | None:
    """
    Session-stable height for facade width (same pair after OBSERVE Reset).

    Uses the best EKF rel-alt seen on either mark, not per-click resolved AGL
    (which could flip between EKF and downward RF and change the line by 2×).
    """
    from vgcs.observe.target_measure import session_facade_measure_agl_m

    agl, _src = session_facade_measure_agl_m([row_a, row_b])
    if agl is None:
        return None
    return min(float(agl), 80.0)


def _sanitized_click_range_m(row: dict[str, Any], agl_eff: float) -> float | None:
    """Per-click horizontal range; drop bad ground hits."""
    try:
        rg = float(row.get("geo_range_m") or 0)
    except (TypeError, ValueError):
        rg = 0.0
    dep = row.get("geo_depression_deg")
    rh_dep: float | None = None
    if dep is not None:
        rh_dep = _horizontal_range_from_depression(agl_eff, float(dep))
    if rg > 0.5 and rh_dep is not None and rh_dep > 0.5:
        if rg > rh_dep * 2.8 or rg > rh_dep + 20.0:
            return rh_dep
        return 0.5 * (rg + rh_dep)
    if rg > 0.5:
        return rg
    return rh_dep


def facade_plane_width_between_marks(
    row_a: dict[str, Any],
    row_b: dict[str, Any],
    *,
    hfov_deg: float = 62.0,
    session_rf_floor_m: float | None = None,
) -> float | None:
    """
    Width (m) on a vertical/oblique facade from two video clicks.

    Primary: law-of-cosines on per-click range + bearing (same as geo_reference rays).
    Fallback: pixel angle × mean depression range.
    """
    del session_rf_floor_m
    xy_a, xy_b = _video_xy(row_a), _video_xy(row_b)
    if xy_a is None or xy_b is None:
        return None
    dx = abs(xy_b[0] - xy_a[0])
    if dx < 0.03:
        return None

    agl_eff = _pair_facade_agl_m(row_a, row_b)
    if agl_eff is None or agl_eff < _MIN_FACADE_AGL_M:
        return None

    from vgcs.observe.target_measure import _fill_pair_geo_ranges

    filled = _fill_pair_geo_ranges(row_a, row_b, hfov_deg=hfov_deg)
    if filled is not None:
        rg1, rg2, b1, b2 = filled
        r1 = _sanitized_click_range_m(
            {**row_a, "geo_range_m": rg1, "geo_bearing_deg": b1}, agl_eff
        )
        r2 = _sanitized_click_range_m(
            {**row_b, "geo_range_m": rg2, "geo_bearing_deg": b2}, agl_eff
        )
        b1, b2 = b1, b2
    else:
        b1 = row_a.get("geo_bearing_deg")
        b2 = row_b.get("geo_bearing_deg")
        r1 = _sanitized_click_range_m(row_a, agl_eff)
        r2 = _sanitized_click_range_m(row_b, agl_eff)
    angle_h = dx * math.radians(float(hfov_deg))

    if (
        r1 is not None
        and r2 is not None
        and b1 is not None
        and b2 is not None
    ):
        d_law = _law_of_cosines_m(r1, r2, float(b1), float(b2))
        if d_law is not None and d_law > 0:
            rh_vals_law: list[float] = []
            for row in (row_a, row_b):
                dep = row.get("geo_depression_deg")
                if dep is None:
                    continue
                rh = _horizontal_range_from_depression(agl_eff, float(dep))
                if rh is not None and rh > 0.5:
                    rh_vals_law.append(rh)
            if dx >= 0.32 and rh_vals_law:
                rh_m = sum(rh_vals_law) / len(rh_vals_law)
                d_chord = 2.0 * rh_m * math.sin(angle_h / 2.0)
                return max(d_law, d_chord)
            return d_law

    if angle_h <= 1e-5:
        return None
    rh_vals: list[float] = []
    for row in (row_a, row_b):
        dep = row.get("geo_depression_deg")
        if dep is None:
            continue
        rh = _horizontal_range_from_depression(agl_eff, float(dep))
        if rh is not None and rh > 0.5:
            rh_vals.append(rh)
    if not rh_vals:
        return None
    rh = sum(rh_vals) / len(rh_vals)
    if angle_h < 0.35:
        return rh * angle_h
    return 2.0 * rh * math.sin(angle_h / 2.0)


def infer_ray_target_msl_m(
    vehicle_alt_msl_m: float,
    horizontal_range_m: float,
    depression_deg: float,
) -> float | None:
    """MSL along the camera ray at ``horizontal_range_m`` (vertical drop from drone)."""
    try:
        v_msl = float(vehicle_alt_msl_m)
        rh = float(horizontal_range_m)
        dep = float(depression_deg)
    except (TypeError, ValueError):
        return None
    if rh <= 0.5 or dep < 3.0 or dep > 89.0:
        return None
    drop = rh * math.tan(math.radians(dep))
    return v_msl - drop


def infer_ray_target_msl_from_row(row: dict[str, Any]) -> float | None:
    vmsl = row.get("vehicle_alt_msl_m")
    rh = row.get("geo_range_m")
    dep = row.get("geo_depression_deg")
    if vmsl is None or rh is None or dep is None:
        return None
    return infer_ray_target_msl_m(float(vmsl), float(rh), float(dep))


def _row_measure_agl_m(row: dict[str, Any]) -> float | None:
    for key in ("measure_agl_m", "ekf_rel_alt_m", "vehicle_rel_alt_m"):
        try:
            v = float(row.get(key) or 0)
        except (TypeError, ValueError):
            continue
        if v > 0.5:
            return v
    return _observation_agl_m(row)


def _row_horizontal_range_m(row: dict[str, Any], agl_m: float) -> float | None:
    try:
        rg = float(row.get("geo_range_m") or 0)
    except (TypeError, ValueError):
        rg = 0.0
    if rg > 0.5:
        return _sanitized_click_range_m(row, agl_m) or rg
    dep = row.get("geo_depression_deg")
    if dep is not None:
        return _horizontal_range_from_depression(agl_m, float(dep))
    return None


def infer_elevated_click_target_msl_from_row(
    row: dict[str, Any],
    *,
    hfov_deg: float = 62.0,
    camera_vfov_deg: float | None = None,
) -> float | None:
    """
    MSL for a single facade click above terrain at the same map footprint.

    DEM gives ground elevation; video Y and drone AGL/range give height above that
  footprint (roof/window aim point). Works when roof+base pair marks are not available.
    """
    xy = _video_xy(row)
    if xy is None:
        return None
    y_norm = xy[1]
    if y_norm >= 0.54:
        return None

    dem_raw = row.get("target_alt_m_dem")
    if dem_raw is None:
        dem_raw = row.get("target_alt_m")
    try:
        dem_val = float(dem_raw) if dem_raw is not None else None
    except (TypeError, ValueError):
        dem_val = None
    if dem_val is None:
        return None

    agl_m = _row_measure_agl_m(row)
    if agl_m is None or agl_m < _MIN_FACADE_AGL_M:
        return None

    rh = _row_horizontal_range_m(row, float(agl_m))
    if rh is None or rh < 1.0:
        return None

    dep_ground = math.degrees(math.atan(float(agl_m) / float(rh)))
    vfov = (
        float(camera_vfov_deg)
        if camera_vfov_deg is not None
        else float(hfov_deg) * 0.5625
    )
    el_off = (y_norm - 0.5) * vfov
    dep_from_video = dep_ground + el_off

    dep_geo = row.get("geo_depression_deg")
    if dep_geo is not None:
        dep_click = float(dep_geo)
        # Geo depression often matches footprint ground (DEM ray hit), not the click.
        if abs(dep_click - dep_ground) < 0.4 and el_off < -0.25:
            dep_click = dep_from_video
    else:
        dep_click = dep_from_video

    if dep_click >= dep_ground - 0.15:
        return None

    height = float(rh) * (
        math.tan(math.radians(dep_ground)) - math.tan(math.radians(dep_click))
    )
    if height < 0.8:
        return None

    vmsl = row.get("vehicle_alt_msl_m")
    msl_ray = (
        infer_ray_target_msl_m(float(vmsl), float(rh), dep_click)
        if vmsl is not None
        else None
    )
    msl_dem = dem_val + height
    if msl_ray is not None:
        return max(msl_dem, msl_ray)
    return msl_dem


def marks_suitable_for_facade_height(
    row_a: dict[str, Any],
    row_b: dict[str, Any],
) -> tuple[bool, str]:
    """Roof + base on one facade (similar X, different Y)."""
    xy_a, xy_b = _video_xy(row_a), _video_xy(row_b)
    if xy_a is None or xy_b is None:
        return False, "missing video coords"
    dx = abs(xy_b[0] - xy_a[0])
    dy = abs(xy_b[1] - xy_a[1])
    if dy < _MIN_VERTICAL_DY_NORM:
        return False, "marks too level for height"
    if dx > _MAX_VERTICAL_DX_NORM:
        return False, "marks not aligned vertically"
    agl = _pair_facade_agl_m(row_a, row_b)
    if agl is None or agl < _MIN_FACADE_AGL_M:
        return False, "height too low for measure"
    return True, ""


def _ordered_upper_lower_rows(
    row_a: dict[str, Any],
    row_b: dict[str, Any],
) -> tuple[dict[str, Any], dict[str, Any]] | None:
    xy_a, xy_b = _video_xy(row_a), _video_xy(row_b)
    if xy_a is None or xy_b is None:
        return None
    if xy_a[1] <= xy_b[1]:
        return row_a, row_b
    return row_b, row_a


def facade_vertical_height_between_marks(
    row_a: dict[str, Any],
    row_b: dict[str, Any],
    *,
    hfov_deg: float = 62.0,
    camera_vfov_deg: float | None = None,
) -> float | None:
    """
    Building height (m) from two video marks on the same facade (roof + base).

    Uses depression difference at mean horizontal range, ray MSL delta, and
    video vertical angle × slant range; returns the mean of plausible estimates.
    """
    ordered = _ordered_upper_lower_rows(row_a, row_b)
    if ordered is None:
        return None
    upper_row, lower_row = ordered
    xy_u, xy_l = _video_xy(upper_row), _video_xy(lower_row)
    if xy_u is None or xy_l is None:
        return None
    dy = abs(xy_l[1] - xy_u[1])
    if dy < _MIN_VERTICAL_DY_NORM:
        return None

    agl_eff = _pair_facade_agl_m(row_a, row_b)
    if agl_eff is None or agl_eff < _MIN_FACADE_AGL_M:
        from vgcs.observe.target_measure import session_facade_measure_agl_m

        agl_eff, _ = session_facade_measure_agl_m([row_a, row_b])
    if agl_eff is None or agl_eff < _MIN_FACADE_AGL_M:
        fb = _pair_height_from_video_y_and_range(
            upper_row,
            lower_row,
            hfov_deg=hfov_deg,
            camera_vfov_deg=camera_vfov_deg,
        )
        if fb is not None:
            return fb
        return None

    vfov = (
        float(camera_vfov_deg)
        if camera_vfov_deg is not None
        else float(hfov_deg) * 0.5625
    )
    dep_upper = upper_row.get("geo_depression_deg")
    dep_lower = lower_row.get("geo_depression_deg")
    estimates: list[float] = []

    r_upper = _sanitized_click_range_m(upper_row, agl_eff)
    r_lower = _sanitized_click_range_m(lower_row, agl_eff)
    if (
        dep_upper is not None
        and dep_lower is not None
        and r_upper is not None
        and r_lower is not None
    ):
        rh_mean = 0.5 * (r_upper + r_lower)
        dep_u = float(dep_upper)
        dep_l = float(dep_lower)
        if dep_l > dep_u + 0.3:
            h_dep = rh_mean * (
                math.tan(math.radians(dep_l)) - math.tan(math.radians(dep_u))
            )
            if h_dep > 0.5:
                estimates.append(h_dep)

    msl_upper = infer_ray_target_msl_from_row(upper_row)
    msl_lower = infer_ray_target_msl_from_row(lower_row)
    if msl_upper is not None and msl_lower is not None:
        h_msl = abs(msl_upper - msl_lower)
        if h_msl > 0.5:
            estimates.append(h_msl)

    angle_v = (xy_l[1] - xy_u[1]) * math.radians(vfov)
    if angle_v > 0.02:
        dep_avg: float | None = None
        if dep_upper is not None and dep_lower is not None:
            dep_avg = 0.5 * (float(dep_upper) + float(dep_lower))
        elif dep_lower is not None:
            dep_avg = float(dep_lower)
        if dep_avg is not None and dep_avg >= 5.0:
            slant = agl_eff / math.sin(math.radians(dep_avg))
            h_slant = slant * math.sin(angle_v)
            if h_slant > 0.5:
                estimates.append(h_slant)
            rh = agl_eff / math.tan(math.radians(dep_avg))
            h_rh = rh * math.tan(angle_v)
            if h_rh > 0.5:
                estimates.append(h_rh)

    if not estimates:
        fb = _pair_height_from_video_y_and_range(
            upper_row,
            lower_row,
            hfov_deg=hfov_deg,
            camera_vfov_deg=camera_vfov_deg,
        )
        if fb is not None:
            return fb
        return None

    h_dep: float | None = None
    if (
        dep_upper is not None
        and dep_lower is not None
        and r_upper is not None
        and r_lower is not None
    ):
        dep_u = float(dep_upper)
        dep_l = float(dep_lower)
        if dep_l > dep_u + 0.3:
            rh_mean = 0.5 * (r_upper + r_lower)
            h_dep = rh_mean * (
                math.tan(math.radians(dep_l)) - math.tan(math.radians(dep_u))
            )
            if h_dep > 0.5 and abs(r_upper - r_lower) <= max(5.0, 0.2 * rh_mean):
                return h_dep

    return sum(estimates) / len(estimates)


def _pair_height_from_video_y_and_range(
    row_upper: dict[str, Any],
    row_lower: dict[str, Any],
    *,
    hfov_deg: float = 62.0,
    camera_vfov_deg: float | None = None,
    min_dy: float = 0.04,
) -> float | None:
    """
    Low-hover fallback: estimate vertical separation from video Y and geo range.

    Used when drone AGL is below the full facade-ray threshold but the operator
  picked aim point and impact at different heights on the same facade.
    """
    xy_u, xy_l = _video_xy(row_upper), _video_xy(row_lower)
    if xy_u is None or xy_l is None:
        return None
    dy = float(xy_l[1]) - float(xy_u[1])
    if dy < min_dy:
        return None
    vfov = (
        float(camera_vfov_deg)
        if camera_vfov_deg is not None
        else float(hfov_deg) * 0.5625
    )
    angle_v = dy * math.radians(vfov)
    if angle_v < 0.012:
        return None
    rh_vals: list[float] = []
    for row in (row_upper, row_lower):
        for key in ("geo_range_m",):
            raw = row.get(key)
            if raw is None:
                continue
            try:
                val = float(raw)
            except (TypeError, ValueError):
                continue
            if val >= 3.0:
                rh_vals.append(val)
    if not rh_vals:
        return None
    rh = sum(rh_vals) / len(rh_vals)
    height = rh * math.tan(angle_v)
    return height if height >= 0.35 else None
