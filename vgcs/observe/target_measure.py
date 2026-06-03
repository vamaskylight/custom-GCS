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
) -> float | None:
    """
    Ground separation between two observation marks.

    Prefer range+bearing from the vehicle (stable for short spans); fall back to
    target lat/lon haversine. Optional video-angular check reduces underestimate
    when both clicks share similar height in the frame (e.g. doorway width).
    """
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
        angle = math.hypot(dx * hfov_rad, dy * vfov_rad)
        if angle > 1e-4 and pix_span < 0.55 and dy < 0.2:
            r1 = row_a.get("geo_range_m")
            r2 = row_b.get("geo_range_m")
            try:
                r_avg = (float(r1 or 0) + float(r2 or 0)) * 0.5
            except Exception:
                r_avg = 0.0
            if r_avg > 1.0:
                d_vid = r_avg * angle
                d = max(d, d_vid)
            try:
                ra = float(row_a.get("geo_range_m") or 0)
                rb = float(row_b.get("geo_range_m") or 0)
                if ra > 1.0 and rb > 1.0 and d > 0.3:
                    cos_d = (ra * ra + rb * rb - d * d) / (2.0 * ra * rb)
                    cos_d = max(-1.0, min(1.0, cos_d))
                    angle_bearing = math.acos(cos_d)
                    if angle_bearing > 1e-4 and angle > angle_bearing * 1.08:
                        d = max(d, d * (angle / angle_bearing) * 0.95)
            except Exception:
                pass
            # Tape on door/window width is often horizontal; ground chord is shorter.
            if dy < 0.2 and pix_span < 0.55 and d < 10.0:
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
