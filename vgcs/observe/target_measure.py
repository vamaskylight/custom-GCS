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


def marks_same_height_band(
    row_a: dict[str, Any],
    row_b: dict[str, Any],
    *,
    max_dy_norm: float = 0.20,
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


def session_rangefinder_reference_m(rows: list[dict[str, Any]]) -> float | None:
    """Downward rangefinder (m) logged with marks — fallback range for video-only width."""
    best = 0.0
    for row in rows:
        for key in ("rangefinder_down_m", "vehicle_rel_alt_m"):
            try:
                v = float(row.get(key) or 0)
            except (TypeError, ValueError):
                continue
            if 1.0 < v < 500.0 and v > best:
                best = v
    return best if best > 1.0 else None


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


def video_facade_width_m(
    row_a: dict[str, Any],
    row_b: dict[str, Any],
    *,
    hfov_deg: float = 62.0,
    session_peak_range_m: float | None = None,
    facade_reference_range_m: float | None = None,
    calibrating_range_only: bool = False,
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
    hfov_rad = math.radians(float(hfov_deg))
    angle_h = dx * hfov_rad
    if angle_h <= 1e-5:
        return None

    candidates: list[float] = []

    try:
        r1 = float(row_a.get("geo_range_m") or 0)
        r2 = float(row_b.get("geo_range_m") or 0)
        b1 = row_a.get("geo_bearing_deg")
        b2 = row_b.get("geo_bearing_deg")
    except (TypeError, ValueError):
        r1 = r2 = 0.0
        b1 = b2 = None

    if r1 > 0 and r2 > 0 and b1 is not None and b2 is not None:
        d_law = _law_of_cosines_m(r1, r2, float(b1), float(b2))
        if d_law is not None and d_law > 0:
            candidates.append(d_law)
        d_ang = 0.5 * (r1 + r2) * angle_h
        if d_ang > 0:
            candidates.append(d_ang)

    agl = _observation_agl_m(row_a) or _observation_agl_m(row_b)
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

    pa = observation_target_latlon(row_a)
    pb = observation_target_latlon(row_b)
    d_hav: float | None = None
    if pa is not None and pb is not None:
        d_hav = haversine_m(pa[0], pa[1], pb[0], pb[1])

    trust_haversine = False
    if d_hav is not None and 0.0 < d_hav < 50.0:
        if d_hav < d * 0.75:
            # Ground geo points agree on a short span — trust over FOV×range (wall clicks).
            d = d_hav
            trust_haversine = True
        elif d_hav > d * 2.5:
            # Far-apart ground projection (classic wall/pillar bug) — ignore haversine.
            pass
        else:
            d = min(d, d_hav)

    if not calibrating_range_only and r1 > 0 and r2 > 0 and not trust_haversine:
        peak = float(session_peak_range_m or 0)
        facade_ref = float(facade_reference_range_m or 0)
        r_local = max(r1, r2)
        d_angle_ref = 0.5 * (r1 + r2) * angle_h
        if d < d_angle_ref * 0.65:
            uplift = d_angle_ref
            if facade_ref > r_local * 1.08:
                uplift = max(uplift, min(facade_ref * angle_h * 1.08, d_angle_ref * 1.35))
            elif peak > r_local * 1.25:
                blend = min(0.32, (peak / max(r_local, 1.0) - 1.0) * 0.12)
                r_ref = r_local + (peak - r_local) * blend
                uplift = max(uplift, min(r_ref * angle_h * 1.05, d_angle_ref * 1.35))
            d = max(d, uplift)

    if d < 15.0:
        d *= _segment_scale_short()
    return max(0.0, d)


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
    agl = range_m or session_rangefinder_reference_m([row_a, row_b])
    if agl is None or agl < 1.0:
        return None
    candidates: list[float] = []
    for row in (row_a, row_b):
        dep = row.get("geo_depression_deg")
        if dep is not None:
            rh = _horizontal_range_from_depression(float(agl), float(dep))
            if rh is not None:
                candidates.append(rh * angle_h)
    if not candidates:
        # Last resort: assume ~40° look so RF AGL is not used as horizontal range (was ~82 m).
        rh = float(agl) / math.tan(math.radians(40.0))
        candidates.append(rh * angle_h)
    d = min(candidates)
    if d < 15.0:
        d *= _segment_scale_short()
    return max(0.0, d)


def cluster_observations_by_video_y(
    rows: list[dict[str, Any]],
    *,
    max_dy_norm: float = 0.18,
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
    max_dy_norm: float = 0.18,
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


def _format_segment_label(d: float, *, video_only: bool, video_span_norm: float | None) -> str:
    base = format_target_segment_label(d, video_span_norm=video_span_norm)
    if video_only and base and base != "distance unreliable":
        return f"~{base}"
    return base


def observation_facade_video_segments(
    rows: list[dict[str, Any]],
    *,
    hfov_deg: float = 62.0,
    max_dy_norm: float = 0.18,
) -> list[tuple[float, float, float, float, str]]:
    """One measure line per height band: leftmost ↔ rightmost mark (pillar gap width)."""
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
        d = segment_distance_between_rows(
            left,
            right,
            hfov_deg=hfov_deg,
            session_peak_range_m=peak,
            facade_reference_range_m=facade_ref,
            require_same_height_band=False,
        )
        if d is None:
            continue
        pix = video_mark_span_norm(xy_l[0], xy_l[1], xy_r[0], xy_r[1])
        label = _format_segment_label(d, video_only=False, video_span_norm=pix)
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
        d = segment_distance_video_fallback(
            left, right, hfov_deg=hfov_deg, range_m=rf_ref
        )
        if d is None:
            continue
        pix = video_mark_span_norm(xy_l[0], xy_l[1], xy_r[0], xy_r[1])
        label = _format_segment_label(d, video_only=True, video_span_norm=pix)
        if label:
            out.append((xy_l[0], xy_l[1], xy_r[0], xy_r[1], label))
    return out


def _segment_scale_short() -> float:
    try:
        from PySide6.QtCore import QSettings

        v = float(QSettings("VGCS", "VGCS").value("observe/segment_distance_scale", 1.0) or 1.0)
        return max(0.85, min(1.35, v))
    except Exception:
        return 1.0


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
        )
    pa = observation_target_latlon(row_a)
    pb = observation_target_latlon(row_b)
    if pa is None or pb is None:
        return None
    d = haversine_m(pa[0], pa[1], pb[0], pb[1])
    if d < 15.0:
        d *= _segment_scale_short()
    return max(0.0, d)


def format_target_segment_label(
    geo_distance_m: float,
    *,
    video_span_norm: float | None = None,
) -> str:
    """
    Label for map/video measure line. Flags when geo distance disagrees with
    how close the two clicks are on the video (common indoors / wall marks).
    """
    d = float(geo_distance_m)
    if video_span_norm is not None and video_span_norm < 0.55 and d > 30.0:
        return "distance unreliable (~use ground marks)"
    if d >= 1000.0:
        return f"{d / 1000.0:.1f} km"
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
