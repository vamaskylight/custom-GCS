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


def cluster_observations_by_video_y(
    rows: list[dict[str, Any]],
    *,
    max_dy_norm: float = 0.18,
) -> list[list[dict[str, Any]]]:
    """Group video marks into horizontal bands (same pillar height)."""
    clusters: list[list[dict[str, Any]]] = []
    for row in rows:
        if observation_target_latlon(row) is None:
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
        d = segment_distance_between_rows(
            left,
            right,
            hfov_deg=hfov_deg,
            session_peak_range_m=peak,
            facade_reference_range_m=ref,
            require_same_height_band=False,
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
        label = format_target_segment_label(d, video_span_norm=pix)
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
    """
    Ground separation between two observation marks.

    Prefer range+bearing from the vehicle (stable for short spans); fall back to
    target lat/lon haversine. For facade width (two pillars), uses horizontal
    video angle × a stable range; reuses session peak range when elevated clicks
    underestimate distance.
    """
    if require_same_height_band and not marks_same_height_band(row_a, row_b):
        return None

    vfov = float(vfov_deg) if vfov_deg is not None else float(hfov_deg) * 0.5625
    hfov_rad = math.radians(float(hfov_deg))
    vfov_rad = math.radians(vfov)

    d_geo: float | None = None
    try:
        r1 = row_a.get("geo_range_m")
        r2 = row_b.get("geo_range_m")
        b1 = row_a.get("geo_bearing_deg")
        b2 = row_b.get("geo_bearing_deg")
        if r1 is not None and r2 is not None and b1 is not None and b2 is not None:
            ra = float(r1)
            rb = float(r2)
            ba = math.radians(float(b1))
            bb = math.radians(float(b2))
            d_geo = math.sqrt(ra * ra + rb * rb - 2.0 * ra * rb * math.cos(bb - ba))
    except Exception:
        d_geo = None
    if d_geo is None or d_geo <= 0:
        pa = observation_target_latlon(row_a)
        pb = observation_target_latlon(row_b)
        if pa is None or pb is None:
            return None
        d_geo = haversine_m(pa[0], pa[1], pb[0], pb[1])

    xa, ya = row_a.get("video_x_norm"), row_a.get("video_y_norm")
    xb, yb = row_b.get("video_x_norm"), row_b.get("video_y_norm")
    d = float(d_geo)
    if xa is not None and ya is not None and xb is not None and yb is not None:
        dx = abs(float(xb) - float(xa))
        dy = abs(float(yb) - float(ya))
        pix_span = video_mark_span_norm(float(xa), float(ya), float(xb), float(yb))
        angle_h = dx * hfov_rad
        angle = math.hypot(angle_h, dy * vfov_rad)
        try:
            r1 = float(row_a.get("geo_range_m") or 0)
            r2 = float(row_b.get("geo_range_m") or 0)
        except (TypeError, ValueError):
            r1 = r2 = 0.0
        r_local = max(r1, r2)
        peak = float(session_peak_range_m or 0)
        facade_ref = float(facade_reference_range_m or 0)
        if angle_h > 1e-4 and dx > 0.03:
            r_ref = r_local
            if not calibrating_range_only:
                if facade_ref > r_ref * 1.08:
                    r_ref = facade_ref
                elif peak > r_local * 1.25:
                    blend = min(0.38, (peak / max(r_local, 1.0) - 1.0) * 0.14)
                    r_ref = r_local + (peak - r_local) * blend
            d_facade = r_ref * angle_h
            d = max(d, d_facade)
        if calibrating_range_only:
            if d < 15.0:
                d *= _segment_scale_short()
            return max(0.0, d)
        if angle > 1e-4 and pix_span < 0.55 and dy < 0.2:
            r_avg = (r1 + r2) * 0.5
            if r_avg > 1.0:
                d_vid = r_avg * angle
                d = max(d, d_vid)
            if r1 > 1.0 and r2 > 1.0 and d > 0.3:
                try:
                    cos_d = (r1 * r1 + r2 * r2 - d * d) / (2.0 * r1 * r2)
                    cos_d = max(-1.0, min(1.0, cos_d))
                    angle_bearing = math.acos(cos_d)
                    if angle_bearing > 1e-4 and angle > angle_bearing * 1.08:
                        d = max(d, d * (angle / angle_bearing) * 0.95)
                except Exception:
                    pass
            if dy < 0.12 and pix_span < 0.55 and d < 10.0 and peak <= r_local * 1.12:
                d *= 1.12

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
    if video_span_norm is not None and video_span_norm < 0.4 and d > 25.0:
        return "distance unreliable"
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
