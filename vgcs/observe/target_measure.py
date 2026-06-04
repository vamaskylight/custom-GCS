"""Distance and track helpers for observation targets (M8 measure)."""

from __future__ import annotations

import math
from typing import Any

from vgcs.observe.facade_plane import facade_plane_width_between_marks

_EARTH_RADIUS_M = 6_371_000.0

# Max |Δy| in normalized video coords for "same horizontal" width (door/post L–R).
SAME_LEVEL_DY_NORM = 0.08
# Facade width needs real flight height; bench/on-ground RF ~0.2–1 m is invalid.
MIN_FACADE_AGL_M = 2.5
MEASURE_AGL_TOO_LOW_MSG = "Take off — height too low for measure"
# Show on-video hint when |Δy| exceeds this (marks visibly not on one horizontal line).
LEVEL_WARN_DY_NORM = 0.04
MARKS_NOT_LEVEL_HINT = "Marks not level — distance may read high."


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


# Below this EKF rel-alt (m), prefer downward RF for wall/facade rays when RF is present.
# Field logs: ekf=0 + rf≈26 → good ~3.5 m; ekf≈2.3 + rf≈26 → geo fails → ~2.1 m / km glitch.
_FACADE_EKF_TRUST_MIN_M = 3.0


def resolve_facade_ray_agl_m(
    *,
    relative_alt_m: float | None,
    rangefinder_down_m: float | None = None,
) -> tuple[float | None, str]:
    """
    AGL for geo-referencing facade/wall clicks.

    EKF rel-alt often reads 0–2 m on a roof/bench while downward RF (~25 m) matches the
    courtyard ground used successfully for ~3.5 m building openings.
    """
    try:
        rel = float(relative_alt_m) if relative_alt_m is not None else None
    except (TypeError, ValueError):
        rel = None
    try:
        rf = float(rangefinder_down_m) if rangefinder_down_m is not None else None
    except (TypeError, ValueError):
        rf = None
    if rf is not None and rf >= MIN_FACADE_AGL_M:
        if rel is None or rel < _FACADE_EKF_TRUST_MIN_M:
            return rf, "rangefinder_down_facade"
    if rel is not None and rel >= _FACADE_EKF_TRUST_MIN_M:
        return rel, "ekf_relative"
    return resolve_vehicle_agl_m(
        relative_alt_m=relative_alt_m,
        rangefinder_down_m=rangefinder_down_m,
    )


def observation_ekf_rel_alt_m(row: dict[str, Any]) -> float | None:
    """
    EKF relative altitude at mark time (not downward rangefinder).

    Legacy rows stored resolved AGL in ``vehicle_rel_alt_m``; skip when that value
    is clearly the rangefinder fallback.
    """
    try:
        ekf = row.get("ekf_rel_alt_m")
        if ekf is not None:
            v = float(ekf)
            if v > 0.05:
                return v
    except (TypeError, ValueError):
        pass
    src = str(row.get("agl_source") or "")
    try:
        rel = float(row.get("vehicle_rel_alt_m") or 0)
    except (TypeError, ValueError):
        return None
    if rel <= 0.05:
        return None
    if src == "rangefinder_down":
        return None
    try:
        rf = float(row.get("rangefinder_down_m") or 0)
        if rf > 0.5 and abs(rel - rf) < 1.0:
            return None
    except (TypeError, ValueError):
        pass
    return rel


def is_oblique_roof_context(row_a: dict[str, Any], row_b: dict[str, Any]) -> bool:
    """
    Low aerial oblique view: trust EKF height, not downward RF.

    Not set when EKF is below ``_FACADE_EKF_TRUST_MIN_M`` (bench/roof with RF to courtyard).
    """
    ekf_vals: list[float] = []
    rf_vals: list[float] = []
    for row in (row_a, row_b):
        e = observation_ekf_rel_alt_m(row)
        if e is not None and e >= _FACADE_EKF_TRUST_MIN_M:
            ekf_vals.append(e)
        try:
            rf = float(row.get("rangefinder_down_m") or 0)
            if rf > 0.5:
                rf_vals.append(rf)
        except (TypeError, ValueError):
            pass
    if not ekf_vals:
        return False
    rel = max(ekf_vals)
    rf = min(rf_vals) if rf_vals else None
    return rf is not None and rf > rel * 1.6


# Field: ~3.5 m openings at RF≈26 m; same clicks read ~2 m at RF≈14 m (courtyard below).
_LOW_RF_FACADE_REFERENCE_M = 26.0


def _low_rf_facade_width_boost(rows: list[dict[str, Any]], d_m: float) -> float:
    """Scale width when drone moved closer (downward RF drops but opening width unchanged)."""
    if d_m <= 0.5:
        return d_m
    rf_vals: list[float] = []
    for row in rows:
        try:
            v = float(row.get("rangefinder_down_m") or 0)
            if v > 0.5:
                rf_vals.append(v)
        except (TypeError, ValueError):
            pass
    if not rf_vals:
        return d_m
    rf = min(rf_vals)
    if rf >= 20.0 or rf < 8.0:
        return d_m
    boost = max(1.0, min(2.35, _LOW_RF_FACADE_REFERENCE_M / rf))
    return d_m * boost


def session_facade_measure_agl_m(rows: list[dict[str, Any]]) -> tuple[float | None, str]:
    """Stable facade height for a mark pair (Reset-safe)."""
    ekf_vals: list[float] = []
    rf_vals: list[float] = []
    for row in rows:
        e = observation_ekf_rel_alt_m(row)
        if e is not None and e > 0.05:
            ekf_vals.append(e)
        try:
            rf = float(row.get("rangefinder_down_m") or 0)
            if rf > 0.05:
                rf_vals.append(rf)
        except (TypeError, ValueError):
            pass
    ekf_max = max(ekf_vals) if ekf_vals else None
    rf_min = min(rf_vals) if rf_vals else None
    if rf_min is not None and rf_min >= MIN_FACADE_AGL_M:
        if ekf_max is None or ekf_max < _FACADE_EKF_TRUST_MIN_M:
            return rf_min, "rangefinder_down_facade"
    if ekf_max is not None and ekf_max >= _FACADE_EKF_TRUST_MIN_M:
        return ekf_max, "ekf_relative"
    if rf_min is not None and rf_min >= MIN_FACADE_AGL_M:
        return rf_min, "rangefinder_down"
    return session_measure_agl_m(rows)


def is_plausible_ground_range(
    agl_m: float,
    range_m: float,
    depression_deg: float,
    *,
    slack: float = 2.5,
) -> bool:
    """Reject flat-earth hits impossibly far for current height and look angle."""
    if range_m <= 0 or agl_m <= 0:
        return False
    agl = float(agl_m)
    rng = float(range_m)
    dep = float(depression_deg)
    # Bench / indoor: rangefinder ~1–2 m but shallow look projects kilometres away.
    if agl < 8.0 and rng > max(12.0, agl * 10.0):
        return False
    if dep < 8.0 and rng > 20.0:
        return False
    dep_use = max(dep, 8.0)
    expected = agl / math.tan(math.radians(dep_use))
    limit = max(15.0, expected * float(slack))
    return rng <= limit


def video_mark_span_norm(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.hypot(float(x2) - float(x1), float(y2) - float(y1))


def marks_level_dy_norm(row_a: dict[str, Any], row_b: dict[str, Any]) -> float | None:
    """Absolute Δy between two video marks, or None if not comparable."""
    xy_a, xy_b = _video_xy(row_a), _video_xy(row_b)
    if xy_a is None or xy_b is None:
        return None
    return abs(xy_b[1] - xy_a[1])


def marks_need_level_warning(row_a: dict[str, Any], row_b: dict[str, Any]) -> bool:
    dy = marks_level_dy_norm(row_a, row_b)
    return dy is not None and dy > LEVEL_WARN_DY_NORM


def append_marks_level_hint(label: str, row_a: dict[str, Any], row_b: dict[str, Any]) -> str:
    if not marks_need_level_warning(row_a, row_b):
        return label
    if not str(label or "").strip():
        return MARKS_NOT_LEVEL_HINT
    if MARKS_NOT_LEVEL_HINT in label:
        return label
    return f"{label} — {MARKS_NOT_LEVEL_HINT}"


def marks_same_height_band(
    row_a: dict[str, Any],
    row_b: dict[str, Any],
    *,
    max_dy_norm: float = SAME_LEVEL_DY_NORM,
) -> bool:
    """True when two video clicks are on the same horizontal band (e.g. pillar L/R)."""
    ya = row_a.get("video_y_norm")
    yb = row_b.get("video_y_norm")
    if ya is None or yb is None:
        return True
    try:
        return abs(float(yb) - float(ya)) <= float(max_dy_norm)
    except (TypeError, ValueError):
        return True


def session_peak_geo_range_m(rows: list[dict[str, Any]]) -> float:
    """Largest drone→target ground range in this OBSERVE session (reference for facade width)."""
    peak = 0.0
    for row in rows:
        try:
            r = float(row.get("geo_range_m") or 0)
        except (TypeError, ValueError):
            continue
        if r > peak:
            peak = r
    return peak


def _video_xy(row: dict[str, Any]) -> tuple[float, float] | None:
    vx = row.get("video_x_norm")
    vy = row.get("video_y_norm")
    if vx is None or vy is None:
        return None
    try:
        return float(vx), float(vy)
    except (TypeError, ValueError):
        return None


def session_measure_agl_m(rows: list[dict[str, Any]]) -> tuple[float | None, str]:
    """
    Height (m) used for facade width. Prefer EKF rel alt when rangefinder is on the ground.
    """
    rel_vals: list[float] = []
    rf_vals: list[float] = []
    for row in rows:
        e = observation_ekf_rel_alt_m(row)
        if e is not None and e > 0.5:
            rel_vals.append(e)
        try:
            rf = float(row.get("rangefinder_down_m") or 0)
            if rf > 0.05:
                rf_vals.append(rf)
        except (TypeError, ValueError):
            pass
    rel = max(rel_vals) if rel_vals else None
    rf = min(rf_vals) if rf_vals else None
    if rel is not None and rel >= MIN_FACADE_AGL_M:
        return rel, "ekf_relative"
    if rf is not None and rf >= MIN_FACADE_AGL_M:
        return rf, "rangefinder_down"
    if rel is not None and rel > rf_vals[0] if rf_vals else 0:
        if rel >= MIN_FACADE_AGL_M:
            return rel, "ekf_relative"
    if rf is not None:
        return None, f"rangefinder_down_{rf:.1f}m"
    if rel is not None:
        return None, f"relative_alt_{rel:.1f}m"
    return None, "missing"


def measure_agl_ok(rows: list[dict[str, Any]]) -> tuple[bool, str]:
    agl, src = session_facade_measure_agl_m(rows)
    if agl is not None and agl >= MIN_FACADE_AGL_M:
        return True, src
    if src.startswith("rangefinder_down_"):
        rf = src.replace("rangefinder_down_", "").replace("m", "")
        return False, f"{MEASURE_AGL_TOO_LOW_MSG} (RF {rf} m)"
    if src.startswith("relative_alt_"):
        return False, f"{MEASURE_AGL_TOO_LOW_MSG} ({src.replace('_', ' ')})"
    return False, MEASURE_AGL_TOO_LOW_MSG


def session_rangefinder_reference_m(rows: list[dict[str, Any]]) -> float | None:
    """
    Best downward AGL (m) for facade width in this session.

    Uses the **minimum** plausible rangefinder in the log so a single RF spike
    (e.g. 45 m) does not inflate wall width.
    """
    vals: list[float] = []
    for row in rows:
        try:
            v = float(row.get("rangefinder_down_m") or 0)
        except (TypeError, ValueError):
            continue
        if MIN_FACADE_AGL_M <= v < 35.0:
            vals.append(v)
    if not vals:
        agl, _ = session_measure_agl_m(rows)
        return agl if agl is not None and agl >= MIN_FACADE_AGL_M else None
    return min(vals)


def _observation_agl_m(row: dict[str, Any]) -> float | None:
    e = observation_ekf_rel_alt_m(row)
    if e is not None and e > 0.5:
        return e
    try:
        rf = float(row.get("rangefinder_down_m") or 0)
        if rf > 0.5:
            return rf
    except (TypeError, ValueError):
        pass
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


def _geo_lon_jump_deg(row_a: dict[str, Any], row_b: dict[str, Any]) -> float:
    pa = observation_target_latlon(row_a)
    pb = observation_target_latlon(row_b)
    if pa is None or pb is None:
        return 0.0
    return abs(float(pb[1]) - float(pa[1]))


def _facade_range_cap_m(
    agl_m: float | None,
    r1: float,
    r2: float,
    *,
    dx_norm: float,
    row_a: dict[str, Any] | None = None,
    row_b: dict[str, Any] | None = None,
) -> float | None:
    """
    Cap horizontal range for wide L–R spans (posts/walls).

    Downward RF AGL is not slant range to the opening; using full geo_range_m
    over-estimates width (20+ m) at altitude. Near the ground (RF ~5 m), the old
    ``agl * 0.30`` cap crushed valid ~4 m openings to ~2.5 m.
    """
    if dx_norm < 0.32:
        return None
    try:
        agl = float(agl_m) if agl_m is not None else 0.0
    except (TypeError, ValueError):
        agl = 0.0
    r_geo = max(r1, r2)
    if r_geo <= 0:
        return None
    if agl < 12.0:
        deps: list[float] = []
        for row in (row_a, row_b):
            if not row:
                continue
            dep = row.get("geo_depression_deg")
            if dep is None:
                continue
            try:
                deps.append(float(dep))
            except (TypeError, ValueError):
                continue
        if deps:
            dep_avg = max(18.0, min(75.0, sum(deps) / len(deps)))
            rh = _horizontal_range_from_depression(agl, dep_avg)
            if rh is not None and rh > 0.5:
                return min(r_geo, max(rh, agl * 1.05))
        return min(r_geo, max(6.0, agl * 1.2))
    cap = max(5.5, min(r_geo, agl * 0.30 if agl > 1.0 else r_geo))
    return cap


def video_facade_width_m(
    row_a: dict[str, Any],
    row_b: dict[str, Any],
    *,
    hfov_deg: float = 62.0,
    session_peak_range_m: float | None = None,
    facade_reference_range_m: float | None = None,
    calibrating_range_only: bool = False,
    allow_off_level: bool = False,
    apply_user_scale: bool = True,
    session_rf_floor_m: float | None = None,
) -> float | None:
    """
    Horizontal gap between two marks on a facade (same video height).

    Ground lat/lon separation is wrong for wall/pillar clicks (often 80+ m).
    Uses min of: law-of-cosines(range+bearing), video-angle × range, depression-based range.
    """
    xy_a, xy_b = _video_xy(row_a), _video_xy(row_b)
    if xy_a is None or xy_b is None:
        return None
    dx = abs(xy_b[0] - xy_a[0])
    dy = abs(xy_b[1] - xy_a[1])
    if dx < 0.03:
        return None
    if dy > SAME_LEVEL_DY_NORM and not allow_off_level:
        return None
    hfov_rad = math.radians(float(hfov_deg))
    angle_h = dx * hfov_rad
    if angle_h <= 1e-5:
        return None

    level_pair = dy <= SAME_LEVEL_DY_NORM
    agl, _facade_src = session_facade_measure_agl_m([row_a, row_b])
    if agl is None:
        agl = _observation_agl_m(row_a) or _observation_agl_m(row_b)

    rf_floor = session_rangefinder_reference_m([row_a, row_b])
    d_facade = facade_plane_width_between_marks(
        row_a, row_b, hfov_deg=hfov_deg, session_rf_floor_m=rf_floor
    )
    pa = observation_target_latlon(row_a)
    pb = observation_target_latlon(row_b)
    d_hav: float | None = None
    if pa is not None and pb is not None:
        d_hav = haversine_m(pa[0], pa[1], pb[0], pb[1])

    try:
        r1_pre = float(row_a.get("geo_range_m") or 0)
        r2_pre = float(row_b.get("geo_range_m") or 0)
    except (TypeError, ValueError):
        r1_pre = r2_pre = 0.0
    d_angle_ref_pre = 0.5 * (r1_pre + r2_pre) * angle_h if r1_pre > 0 and r2_pre > 0 else 0.0

    oblique_low = is_oblique_roof_context(row_a, row_b)

    if d_facade is not None and level_pair:
        trust_facade_only = oblique_low and dx < 0.35
        needs_uplift = (
            d_angle_ref_pre > 0.5
            and d_facade < d_angle_ref_pre * 0.85
            and dx >= 0.32
        )
        implausible = (
            d_angle_ref_pre > 0.5
            and d_facade > max(12.0, d_angle_ref_pre * 1.2)
            and d_facade > d_angle_ref_pre + 4.0
        )
        if trust_facade_only or (not needs_uplift and not implausible):
            if apply_user_scale:
                d_facade *= _segment_scale_short()
            d_facade = _low_rf_facade_width_boost([row_a, row_b], d_facade)
            return max(0.0, d_facade)

    candidates: list[float] = []

    try:
        r1 = float(row_a.get("geo_range_m") or 0)
        r2 = float(row_b.get("geo_range_m") or 0)
        b1 = row_a.get("geo_bearing_deg")
        b2 = row_b.get("geo_bearing_deg")
    except (TypeError, ValueError):
        r1 = r2 = 0.0
        b1 = b2 = None

    filled = _fill_pair_geo_ranges(row_a, row_b)
    if filled is not None:
        r1, r2, b1, b2 = filled
    if r1 > 0 and r2 > 0 and b1 is not None and b2 is not None:
        d_law = _law_of_cosines_m(r1, r2, float(b1), float(b2))
        if d_law is not None and d_law > 0:
            candidates.append(d_law)
        d_ang = 0.5 * (r1 + r2) * angle_h
        if d_ang > 0:
            candidates.append(d_ang)

    if agl is not None:
        for row in (row_a, row_b):
            dep = row.get("geo_depression_deg")
            if dep is None:
                continue
            rh = _horizontal_range_from_depression(agl, float(dep))
            if rh is not None and rh > 0.5:
                candidates.append(rh * angle_h)

    if not candidates:
        return None

    d = min(candidates)

    lon_jump = _geo_lon_jump_deg(row_a, row_b)
    agl_low = agl is not None and agl < 12.0
    wide_span = dx >= 0.32
    d_angle_ref = 0.5 * (r1 + r2) * angle_h if r1 > 0 and r2 > 0 else 0.0
    if (
        d_hav is not None
        and agl_low
        and wide_span
        and level_pair
        and lon_jump < 0.0003
    ):
        # Low-altitude facade: ground chord often >> tape width; do not treat as bad.
        bad_ground_haversine = d_hav > 50.0 or lon_jump > 0.00035
    else:
        bad_ground_haversine = (
            d_hav is not None
            and (
                d_hav > 12.0
                or lon_jump > 0.00035
                or (d_hav > 12.0 and d_hav > d * 2.5)
            )
        )

    geo_ok_pair = (
        str(row_a.get("geo_quality") or "") not in ("", "insufficient")
        and str(row_b.get("geo_quality") or "") not in ("", "insufficient")
    )

    trust_haversine = False
    if (
        level_pair
        and geo_ok_pair
        and d_hav is not None
        and 0.0 < d_hav < 50.0
        and not bad_ground_haversine
    ):
        if d_hav < 12.0 and (d_hav < d * 0.85 or d_hav < 8.0):
            d = d_hav
            trust_haversine = True
            if (
                r1 > 0
                and r2 > 0
                and b1 is not None
                and b2 is not None
                and angle_h > 0.08
                and d_hav < d_angle_ref * 0.55
            ):
                try:
                    cos_d = (r1 * r1 + r2 * r2 - d * d) / (2.0 * r1 * r2)
                    cos_d = max(-1.0, min(1.0, cos_d))
                    angle_bearing = math.acos(cos_d)
                    if angle_h > angle_bearing * 1.35 and angle_bearing > 0.008:
                        scale = min(1.55, (angle_h / angle_bearing) * 0.78)
                        d = d * scale
                except (TypeError, ValueError, ZeroDivisionError):
                    pass
        elif not (agl_low and wide_span and d_hav > 12.0):
            d = min(d, d_hav)

    if not calibrating_range_only and r1 > 0 and r2 > 0 and not trust_haversine:
        peak = float(session_peak_range_m or 0)
        facade_ref = float(facade_reference_range_m or 0)
        r_local = max(r1, r2)
        if d < d_angle_ref * 0.65 and not trust_haversine:
            uplift = d_angle_ref
            if facade_ref > r_local * 1.08:
                uplift = max(uplift, min(facade_ref * angle_h * 1.08, d_angle_ref * 1.35))
            elif peak > r_local * 1.25:
                blend = min(0.32, (peak / max(r_local, 1.0) - 1.0) * 0.12)
                r_ref = r_local + (peak - r_local) * blend
                uplift = max(uplift, min(r_ref * angle_h * 1.05, d_angle_ref * 1.35))
            d = max(d, uplift)

    r_cap = _facade_range_cap_m(agl, r1, r2, dx_norm=dx, row_a=row_a, row_b=row_b)
    if r_cap is not None:
        d_capped = r_cap * angle_h
        if trust_haversine and d > d_capped * 1.35:
            d = max(d * 0.92, d_capped)
        elif bad_ground_haversine or (not agl_low and not trust_haversine):
            d = min(d, d_capped)
        elif agl_low and d < d_capped * 0.92:
            d = max(d, min(d_capped, d_angle_ref or d_capped))

    if apply_user_scale and d < 15.0:
        d *= _segment_scale_short()
    d = _low_rf_facade_width_boost([row_a, row_b], d)
    if d > 45.0:
        return None
    return max(0.0, d)


def _fill_pair_geo_ranges(
    row_a: dict[str, Any], row_b: dict[str, Any]
) -> tuple[float, float, float, float] | None:
    """Use good mark range on wall when the partner geo failed (Reset / EKF glitch)."""
    try:
        r1 = float(row_a.get("geo_range_m") or 0)
        r2 = float(row_b.get("geo_range_m") or 0)
        b1 = row_a.get("geo_bearing_deg")
        b2 = row_b.get("geo_bearing_deg")
    except (TypeError, ValueError):
        return None
    if r1 > 0.5 and r2 > 0.5 and b1 is not None and b2 is not None:
        return r1, r2, float(b1), float(b2)
    if r1 > 0.5 and b1 is not None:
        return r1, r1, float(b1), float(b1)
    if r2 > 0.5 and b2 is not None:
        return r2, r2, float(b2), float(b2)
    return None


def segment_distance_video_fallback(
    row_a: dict[str, Any],
    row_b: dict[str, Any],
    *,
    hfov_deg: float = 62.0,
    range_m: float | None = None,
) -> float | None:
    """Width when geo lock failed — depression-based range, not raw downward RF × angle."""
    xy_a, xy_b = _video_xy(row_a), _video_xy(row_b)
    if xy_a is None or xy_b is None:
        return None
    dx = abs(xy_b[0] - xy_a[0])
    if dx < 0.03:
        return None
    angle_h = dx * math.radians(float(hfov_deg))
    agl, _ = session_facade_measure_agl_m([row_a, row_b])
    if agl is None:
        agl = range_m or session_rangefinder_reference_m([row_a, row_b])
    if agl is None or agl < 1.0:
        return None
    filled = _fill_pair_geo_ranges(row_a, row_b)
    if filled is not None:
        r1, r2, b1, b2 = filled
        d_law = _law_of_cosines_m(r1, r2, b1, b2)
        if d_law is not None and d_law > 0:
            d = d_law
            if d < 15.0:
                d *= _segment_scale_short()
            d = _low_rf_facade_width_boost([row_a, row_b], d)
            return max(0.0, d)

    candidates: list[float] = []
    for row in (row_a, row_b):
        dep = row.get("geo_depression_deg")
        if dep is not None:
            rh = _horizontal_range_from_depression(float(agl), float(dep))
            if rh is not None:
                candidates.append(rh * angle_h)
    if not candidates:
        dep_guess = 27.0
        rh = float(agl) / math.tan(math.radians(dep_guess))
        candidates.append(rh * angle_h)
    d = min(candidates)
    if d < 15.0:
        d *= _segment_scale_short()
    d = _low_rf_facade_width_boost([row_a, row_b], d)
    return max(0.0, d)


def cluster_observations_by_video_y(
    rows: list[dict[str, Any]],
    *,
    max_dy_norm: float = SAME_LEVEL_DY_NORM,
    require_geo: bool = False,
) -> list[list[dict[str, Any]]]:
    """Group video marks into horizontal bands (same pillar height)."""
    clusters: list[list[dict[str, Any]]] = []
    for row in rows:
        if require_geo and observation_target_latlon(row) is None:
            continue
        if str(row.get("kind") or "") != "video_mark" and _video_xy(row) is None:
            continue
        xy = _video_xy(row)
        if xy is None:
            continue
        _, y = xy
        placed = False
        for cluster in clusters:
            ref = _video_xy(cluster[0])
            if ref is None:
                continue
            if abs(y - ref[1]) <= float(max_dy_norm):
                cluster.append(row)
                placed = True
                break
        if not placed:
            clusters.append([row])
    return clusters


def session_facade_reference_range_m(
    rows: list[dict[str, Any]],
    *,
    hfov_deg: float = 62.0,
    max_dy_norm: float = 0.18,
) -> float:
    """
    Best horizontal range for facade width from any good L–R pair in the session.

    When the lower row measured ~4.2 m correctly, upper rows reuse this implied range
    instead of underestimated per-click geo_range_m.
    """
    hfov_rad = math.radians(float(hfov_deg))
    ref = session_peak_geo_range_m(rows)
    peak = ref
    for cluster in cluster_observations_by_video_y(rows, max_dy_norm=max_dy_norm):
        if len(cluster) < 2:
            continue
        ordered = sorted(
            cluster,
            key=lambda r: float((_video_xy(r) or (0.0, 0.0))[0]),
        )
        left, right = ordered[0], ordered[-1]
        if left is right:
            continue
        xy_l, xy_r = _video_xy(left), _video_xy(right)
        if xy_l is None or xy_r is None:
            continue
        dx = abs(xy_r[0] - xy_l[0])
        if dx < 0.03:
            continue
        angle_h = dx * hfov_rad
        d = video_facade_width_m(
            left,
            right,
            hfov_deg=hfov_deg,
            session_peak_range_m=peak,
            facade_reference_range_m=ref,
            calibrating_range_only=True,
        )
        if d is not None and angle_h > 1e-4 and d > 0.5:
            ref = max(ref, float(d) / angle_h)
    return ref


def band_width_partner_row(
    rows: list[dict[str, Any]],
    row: dict[str, Any],
    *,
    max_dy_norm: float = SAME_LEVEL_DY_NORM,
    min_dx_norm: float = 0.04,
) -> dict[str, Any] | None:
    """Other edge on the same horizontal band (pillar L↔R), not merely the previous click."""
    xy = _video_xy(row)
    if xy is None:
        return None
    x, y = xy
    best: dict[str, Any] | None = None
    best_dx = 0.0
    for other in rows:
        if other is row:
            continue
        if str(other.get("kind") or "") != "video_mark":
            if observation_target_latlon(other) is None:
                continue
        oxy = _video_xy(other)
        if oxy is None:
            continue
        ox, oy = oxy
        if abs(oy - y) > float(max_dy_norm):
            continue
        dx = abs(ox - x)
        if dx < float(min_dx_norm):
            continue
        if dx > best_dx:
            best_dx = dx
            best = other
    return best


def _format_segment_label(
    d: float,
    *,
    video_only: bool,
    video_span_norm: float | None,
    estimate: bool = False,
    level_ok: bool = True,
    facade_plane: bool = False,
) -> str:
    base = format_target_segment_label(
        d, video_span_norm=video_span_norm, force_show_meters=True
    )
    if not base:
        return ""
    if not level_ok:
        return MARKS_NOT_LEVEL_HINT
    if facade_plane and not base.startswith("distance unreliable"):
        base = f"{base} (wall)"
    if (video_only or estimate) and not base.startswith("distance unreliable"):
        return f"~{base}"
    return base


def _one_pair_video_segment(
    left: dict[str, Any],
    right: dict[str, Any],
    rows: list[dict[str, Any]],
    *,
    hfov_deg: float,
) -> tuple[float, float, float, float, str] | None:
    xy_l, xy_r = _video_xy(left), _video_xy(right)
    if xy_l is None or xy_r is None:
        return None
    if left is not right and xy_l[0] > xy_r[0]:
        left, right, xy_l, xy_r = right, left, xy_r, xy_l
    peak = session_peak_geo_range_m(rows)
    facade_ref = session_facade_reference_range_m(rows, hfov_deg=hfov_deg)
    rf_ref = session_rangefinder_reference_m(rows)
    tape_m = _tape_pair_matches(xy_l[0], xy_l[1], xy_r[0], xy_r[1])
    if tape_m is not None:
        return (xy_l[0], xy_l[1], xy_r[0], xy_r[1], f"{tape_m:.1f} m (wall, tape)")
    agl_ok, agl_msg = measure_agl_ok(rows)
    if not agl_ok:
        return (xy_l[0], xy_l[1], xy_r[0], xy_r[1], agl_msg)
    dy_lr = abs(xy_r[1] - xy_l[1])
    off_level = dy_lr > SAME_LEVEL_DY_NORM
    d = segment_distance_between_rows(
        left,
        right,
        hfov_deg=hfov_deg,
        session_peak_range_m=peak,
        facade_reference_range_m=facade_ref,
        require_same_height_band=not off_level,
        allow_off_level=off_level,
        session_rf_floor_m=rf_ref,
    )
    if d is None:
        d = segment_distance_video_fallback(
            left, right, hfov_deg=hfov_deg, range_m=rf_ref
        )
    if d is None:
        if not marks_need_level_warning(left, right):
            return None
        label = MARKS_NOT_LEVEL_HINT
    else:
        pix = video_mark_span_norm(xy_l[0], xy_l[1], xy_r[0], xy_r[1])
        est = _geo_lon_jump_deg(left, right) > 0.00035 or (
            str(left.get("geo_quality") or "") == "insufficient"
            and str(right.get("geo_quality") or "") == "insufficient"
        )
        label = _format_segment_label(
            d,
            video_only=est,
            video_span_norm=pix,
            estimate=est,
            level_ok=dy_lr <= SAME_LEVEL_DY_NORM,
            facade_plane=bool(
                left.get("geo_depression_deg") is not None
                and right.get("geo_depression_deg") is not None
            ),
        )
        label = append_marks_level_hint(label, left, right)
        if d is not None:
            pa = observation_target_latlon(left)
            pb = observation_target_latlon(right)
            if pa is not None and pb is not None:
                d_hav = haversine_m(pa[0], pa[1], pb[0], pb[1])
                if d_hav > max(8.0, float(d) * 3.0):
                    label = f"{label} — ground geo differs; trust wall line"
    if label:
        return (xy_l[0], xy_l[1], xy_r[0], xy_r[1], label)
    return None


def observation_facade_video_segments(
    rows: list[dict[str, Any]],
    *,
    hfov_deg: float = 62.0,
    max_dy_norm: float = SAME_LEVEL_DY_NORM,
    latest_pair_only: bool = True,
) -> list[tuple[float, float, float, float, str]]:
    """Measure line for video marks; default = only the last two Target clicks."""
    pair = _last_two_video_marks(rows)
    if latest_pair_only and pair is not None:
        seg = _one_pair_video_segment(pair[0], pair[1], rows, hfov_deg=hfov_deg)
        return [seg] if seg else []

    out: list[tuple[float, float, float, float, str]] = []
    peak = session_peak_geo_range_m(rows)
    facade_ref = session_facade_reference_range_m(
        rows, hfov_deg=hfov_deg, max_dy_norm=max_dy_norm
    )
    rf_ref = session_rangefinder_reference_m(rows)
    for cluster in cluster_observations_by_video_y(
        rows, max_dy_norm=max_dy_norm, require_geo=True
    ):
        if len(cluster) < 2:
            continue
        ordered = sorted(
            cluster,
            key=lambda r: float((_video_xy(r) or (0.0, 0.0))[0]),
        )
        left, right = ordered[0], ordered[-1]
        if left is right:
            continue
        xy_l, xy_r = _video_xy(left), _video_xy(right)
        if xy_l is None or xy_r is None:
            continue
        dy_lr = abs(xy_r[1] - xy_l[1])
        off_level = dy_lr > SAME_LEVEL_DY_NORM
        d = segment_distance_between_rows(
            left,
            right,
            hfov_deg=hfov_deg,
            session_peak_range_m=peak,
            facade_reference_range_m=facade_ref,
            require_same_height_band=not off_level,
            allow_off_level=off_level,
        )
        if d is None and off_level:
            d = segment_distance_video_fallback(
                left, right, hfov_deg=hfov_deg, range_m=rf_ref
            )
        if d is None and not marks_need_level_warning(left, right):
            continue
        pix = video_mark_span_norm(xy_l[0], xy_l[1], xy_r[0], xy_r[1])
        est = _geo_lon_jump_deg(left, right) > 0.00035 or bool(
            left.get("geo_quality") == "insufficient"
            or right.get("geo_quality") == "insufficient"
        )
        label = ""
        if d is not None:
            label = _format_segment_label(
                d,
                video_only=False,
                video_span_norm=pix,
                estimate=est,
                level_ok=dy_lr <= SAME_LEVEL_DY_NORM,
                facade_plane=bool(
                    left.get("geo_depression_deg") is not None
                    and right.get("geo_depression_deg") is not None
                ),
            )
        label = append_marks_level_hint(label, left, right)
        if label:
            out.append((xy_l[0], xy_l[1], xy_r[0], xy_r[1], label))
    if out:
        return out
    for cluster in cluster_observations_by_video_y(
        rows, max_dy_norm=max_dy_norm, require_geo=False
    ):
        if len(cluster) < 2:
            continue
        ordered = sorted(
            cluster,
            key=lambda r: float((_video_xy(r) or (0.0, 0.0))[0]),
        )
        left, right = ordered[0], ordered[-1]
        if left is right:
            continue
        xy_l, xy_r = _video_xy(left), _video_xy(right)
        if xy_l is None or xy_r is None:
            continue
        dy_lr = abs(xy_r[1] - xy_l[1])
        off_level = dy_lr > SAME_LEVEL_DY_NORM
        d = segment_distance_video_fallback(
            left, right, hfov_deg=hfov_deg, range_m=rf_ref
        )
        if d is None:
            d = video_facade_width_m(
                left,
                right,
                hfov_deg=hfov_deg,
                session_peak_range_m=peak,
                allow_off_level=off_level,
            )
        if d is None and not marks_need_level_warning(left, right):
            continue
        pix = video_mark_span_norm(xy_l[0], xy_l[1], xy_r[0], xy_r[1])
        label = ""
        if d is not None:
            label = _format_segment_label(
                d, video_only=True, video_span_norm=pix, estimate=True
            )
            label = append_marks_level_hint(label, left, right)
        else:
            label = MARKS_NOT_LEVEL_HINT
        if label:
            out.append((xy_l[0], xy_l[1], xy_r[0], xy_r[1], label))
    return out


_SEGMENT_SCALE_MIN = 0.15
_SEGMENT_SCALE_MAX = 3.00

# After tape Cal: force this distance on the calibrated mark pair (endpoints, 0..1 video).
_tape_pair_override: dict[str, float] | None = None
_TAPE_XY_TOL = 0.03


def _segment_scale_short() -> float:
    return get_segment_distance_scale()


def get_segment_distance_scale() -> float:
    try:
        from PySide6.QtCore import QSettings

        v = float(QSettings("VGCS", "VGCS").value("observe/segment_distance_scale", 1.0) or 1.0)
        return max(_SEGMENT_SCALE_MIN, min(_SEGMENT_SCALE_MAX, v))
    except Exception:
        return 1.0


def set_segment_distance_scale(scale: float) -> float:
    """Persist tape calibration multiplier (applied to all short facade widths)."""
    v = max(_SEGMENT_SCALE_MIN, min(_SEGMENT_SCALE_MAX, float(scale)))
    try:
        from PySide6.QtCore import QSettings

        QSettings("VGCS", "VGCS").setValue("observe/segment_distance_scale", v)
    except Exception:
        pass
    return v


def _video_marks_ordered(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [r for r in rows if str(r.get("kind") or "") == "video_mark" and _video_xy(r)]


def _last_two_video_marks(
    rows: list[dict[str, Any]],
) -> tuple[dict[str, Any], dict[str, Any]] | None:
    marks = _video_marks_ordered(rows)
    if len(marks) < 2:
        return None
    a, b = marks[-2], marks[-1]
    if a is b:
        return None
    return a, b


def clear_tape_pair_override() -> None:
    global _tape_pair_override
    _tape_pair_override = None


def _tape_pair_key(x1: float, y1: float, x2: float, y2: float) -> tuple[float, float, float, float]:
    if x1 > x2 or (abs(x1 - x2) < 1e-6 and y1 > y2):
        x1, y1, x2, y2 = x2, y2, x1, y1
    return (x1, y1, x2, y2)


def _tape_pair_matches(x1: float, y1: float, x2: float, y2: float) -> float | None:
    o = _tape_pair_override
    if not o:
        return None
    k = _tape_pair_key(x1, y1, x2, y2)
    ok = _tape_pair_key(
        float(o["x1"]), float(o["y1"]), float(o["x2"]), float(o["y2"])
    )
    if all(abs(k[i] - ok[i]) <= _TAPE_XY_TOL for i in range(4)):
        return float(o["known_m"])
    return None


def last_band_measure_width_m(
    rows: list[dict[str, Any]],
    *,
    hfov_deg: float = 62.0,
    apply_user_scale: bool = False,
) -> float | None:
    """Width between the last two Target clicks (for tape calibration)."""
    pair = _last_two_video_marks(rows)
    if pair is None:
        return None
    a, b = pair
    rf_floor = session_rangefinder_reference_m(rows)
    return segment_distance_between_rows(
        a,
        b,
        hfov_deg=hfov_deg,
        require_same_height_band=False,
        allow_off_level=True,
        apply_user_scale=apply_user_scale,
        session_rf_floor_m=rf_floor,
    )


def calibrate_segment_scale_from_tape(
    known_m: float,
    rows: list[dict[str, Any]],
    *,
    hfov_deg: float = 62.0,
) -> dict[str, float] | None:
    """
    Set ``observe/segment_distance_scale`` so the last measured band matches tape.

    Returns scale, raw_m, known_m or None if no valid last measure.
    """
    global _tape_pair_override
    try:
        known = float(known_m)
    except (TypeError, ValueError):
        return None
    if known < 0.05 or known > 200.0:
        return None
    pair = _last_two_video_marks(rows)
    if pair is None:
        return None
    a, b = pair
    agl_ok, _ = measure_agl_ok(rows)
    raw = last_band_measure_width_m(rows, hfov_deg=hfov_deg, apply_user_scale=False)
    if not agl_ok:
        xy_a, xy_b = _video_xy(a), _video_xy(b)
        if xy_a and xy_b:
            _tape_pair_override = {
                "x1": xy_a[0],
                "y1": xy_a[1],
                "x2": xy_b[0],
                "y2": xy_b[1],
                "known_m": known,
                "scale": 1.0,
            }
        return {
            "scale": 1.0,
            "raw_m": 0.0,
            "known_m": known,
            "scale_clamped": 0.0,
            "requested_scale": 1.0,
            "agl_blocked": 1.0,
        }
    if raw is None or raw < 0.05:
        return None
    requested_scale = known / raw
    scale = set_segment_distance_scale(requested_scale)
    clamped = abs(scale - requested_scale) > 0.02
    xy_a, xy_b = _video_xy(a), _video_xy(b)
    if xy_a and xy_b:
        x1, y1 = xy_a
        x2, y2 = xy_b
        _tape_pair_override = {
            "x1": x1,
            "y1": y1,
            "x2": x2,
            "y2": y2,
            "known_m": known,
            "scale": scale,
        }
    return {
        "scale": scale,
        "raw_m": float(raw),
        "known_m": known,
        "scale_clamped": float(clamped),
        "requested_scale": float(requested_scale),
    }


def segment_distance_between_rows(
    row_a: dict[str, Any],
    row_b: dict[str, Any],
    *,
    hfov_deg: float = 62.0,
    vfov_deg: float | None = None,
    session_peak_range_m: float | None = None,
    facade_reference_range_m: float | None = None,
    require_same_height_band: bool = True,
    calibrating_range_only: bool = False,
    allow_off_level: bool = False,
    apply_user_scale: bool = True,
    session_rf_floor_m: float | None = None,
) -> float | None:
    """Ground/facade separation between two observation marks (see ``video_facade_width_m``)."""
    if require_same_height_band and not marks_same_height_band(row_a, row_b):
        return None
    xa, ya = row_a.get("video_x_norm"), row_a.get("video_y_norm")
    xb, yb = row_b.get("video_x_norm"), row_b.get("video_y_norm")
    if xa is not None and xb is not None:
        return video_facade_width_m(
            row_a,
            row_b,
            hfov_deg=hfov_deg,
            session_peak_range_m=session_peak_range_m,
            facade_reference_range_m=facade_reference_range_m,
            calibrating_range_only=calibrating_range_only,
            allow_off_level=allow_off_level,
            apply_user_scale=apply_user_scale,
            session_rf_floor_m=session_rf_floor_m,
        )
    pa = observation_target_latlon(row_a)
    pb = observation_target_latlon(row_b)
    if pa is None or pb is None:
        return None
    d = haversine_m(pa[0], pa[1], pb[0], pb[1])
    if apply_user_scale and d < 15.0:
        d *= _segment_scale_short()
    return max(0.0, d)


def format_target_segment_label(
    geo_distance_m: float,
    *,
    video_span_norm: float | None = None,
    force_show_meters: bool = False,
) -> str:
    """
    Label for map/video measure line. Prefer a numeric estimate; avoid blank
    warnings when we already computed a capped facade width.
    """
    d = float(geo_distance_m)
    if (
        not force_show_meters
        and video_span_norm is not None
        and video_span_norm < 0.55
        and d > 45.0
    ):
        return "distance unreliable (~use ground marks)"
    if d >= 50.0:
        return "distance unreliable (check GPS/height)"
    if d < 100.0:
        return f"{d:.1f} m"
    return f"{d:.0f} m"


def segment_distances_m(track: list[tuple[float, float]]) -> list[float]:
    """Ground distance (m) for each consecutive pair in ``track``."""
    segs: list[float] = []
    for i in range(1, len(track)):
        a = track[i - 1]
        b = track[i]
        segs.append(haversine_m(a[0], a[1], b[0], b[1]))
    return segs
