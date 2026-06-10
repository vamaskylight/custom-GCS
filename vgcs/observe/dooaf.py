"""
DOOAF (Detection, Observation, Orientation & Adjustment of Fire) session state.

Tracks artillery variables per field workflow:
  gun origin → intended target → drone observe → impact mark → fire correction.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from vgcs.observe.target_measure import haversine_m, observation_target_latlon

DOOAF_ROLE_SURVEY = "survey"
DOOAF_ROLE_INTENDED = "intended_target"
DOOAF_ROLE_IMPACT = "impact"
DOOAF_ROLE_GUN = "gun_origin"

DOOAF_ROLES = (
    DOOAF_ROLE_SURVEY,
    DOOAF_ROLE_INTENDED,
    DOOAF_ROLE_IMPACT,
    DOOAF_ROLE_GUN,
)

# Operator-facing labels (dropdown, status, reports).
DOOAF_ROLE_DISPLAY: dict[str, str] = {
    DOOAF_ROLE_INTENDED: "Actual target",
    DOOAF_ROLE_IMPACT: "Fall of shot",
    DOOAF_ROLE_GUN: "Artillery (gun)",
    DOOAF_ROLE_SURVEY: "Wall measure",
}

DOOAF_ROLE_TOOLTIPS: dict[str, str] = {
    DOOAF_ROLE_INTENDED: (
        "Planned impact point from military staff — where the round should land."
    ),
    DOOAF_ROLE_IMPACT: (
        "Mark fall of shot after firing — click burst or smoke on video. "
        "Set gun and actual target in DOOAF Setup first."
    ),
    DOOAF_ROLE_GUN: (
        "Artillery position — gun origin (use DOOAF Setup or click the map)."
    ),
    DOOAF_ROLE_SURVEY: (
        "Facade width measure with a tape — calibration only, not fire correction."
    ),
}


def dooaf_role_display(role: str) -> str:
    return DOOAF_ROLE_DISPLAY.get(str(role or ""), str(role or ""))


@dataclass(frozen=True)
class GeoPoint:
    lat: float
    lon: float
    alt_m: float | None = None


@dataclass(frozen=True)
class FireCorrection:
    """Correction to apply so the next round lands on the intended target."""

    range_correction_m: float
    deflection_correction_m: float
    miss_along_m: float
    miss_right_m: float
    range_gun_to_intended_m: float
    range_gun_to_impact_m: float
    bearing_gun_to_intended_deg: float
    impact_to_intended_m: float
    miss_east_m: float
    miss_north_m: float


@dataclass(frozen=True)
class DooafSession:
    gun: GeoPoint | None
    intended: GeoPoint | None
    impact: GeoPoint | None
    drone: GeoPoint | None
    correction: FireCorrection | None


@dataclass(frozen=True)
class DooafSettings:
    """Military-supplied fixed coordinates (persisted in QSettings)."""

    gun_lat: float | None = None
    gun_lon: float | None = None
    gun_alt_m: float | None = None
    target_lat: float | None = None
    target_lon: float | None = None
    target_alt_m: float | None = None


_QS_GUN_LAT = "dooaf/gun_lat"
_QS_GUN_LON = "dooaf/gun_lon"
_QS_GUN_ALT = "dooaf/gun_alt_m"
_QS_TARGET_LAT = "dooaf/target_lat"
_QS_TARGET_LON = "dooaf/target_lon"
_QS_TARGET_ALT = "dooaf/target_alt_m"


def initial_bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    y = math.sin(dlon) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dlon)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def latlon_delta_to_ne_m(
    lat0: float, lon0: float, lat1: float, lon1: float
) -> tuple[float, float]:
    """North / east offset (m) from (lat0, lon0) to (lat1, lon1)."""
    lat_rad = math.radians(0.5 * (lat0 + lat1))
    north = (lat1 - lat0) * 111_320.0
    east = (lon1 - lon0) * 111_320.0 * math.cos(lat_rad)
    return north, east


def compute_fire_correction(
    gun: GeoPoint,
    intended: GeoPoint,
    impact: GeoPoint,
) -> FireCorrection:
    """
    Gun-centric miss and correction.

    ``miss_along`` > 0 when impact is beyond intended along gun→target line.
    ``miss_right`` > 0 when impact is to the right of gun→target line.
    Corrections are the negation (what to add to firing data).
    """
    range_gt = haversine_m(gun.lat, gun.lon, intended.lat, intended.lon)
    range_gi = haversine_m(gun.lat, gun.lon, impact.lat, impact.lon)
    bearing_gt = initial_bearing_deg(gun.lat, gun.lon, intended.lat, intended.lon)
    bearing_gi = initial_bearing_deg(gun.lat, gun.lon, impact.lat, impact.lon)
    d_theta = math.radians(bearing_gi - bearing_gt)
    along = range_gi * math.cos(d_theta) - range_gt
    right = range_gi * math.sin(d_theta)
    miss_n, miss_e = latlon_delta_to_ne_m(
        intended.lat, intended.lon, impact.lat, impact.lon
    )
    return FireCorrection(
        range_correction_m=-along,
        deflection_correction_m=-right,
        miss_along_m=along,
        miss_right_m=right,
        range_gun_to_intended_m=range_gt,
        range_gun_to_impact_m=range_gi,
        bearing_gun_to_intended_deg=bearing_gt,
        impact_to_intended_m=haversine_m(
            intended.lat, intended.lon, impact.lat, impact.lon
        ),
        miss_east_m=miss_e,
        miss_north_m=miss_n,
    )


def _float_or_none(value: object) -> float | None:
    if value is None:
        return None
    try:
        v = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(v):
        return None
    return v


def point_from_row(row: dict[str, Any]) -> GeoPoint | None:
    pt = observation_target_latlon(row)
    if pt is None:
        return None
    return GeoPoint(pt[0], pt[1], _float_or_none(row.get("target_alt_m")))


def drone_from_row(row: dict[str, Any] | None) -> GeoPoint | None:
    if row is None:
        return None
    lat = _float_or_none(row.get("vehicle_lat"))
    lon = _float_or_none(row.get("vehicle_lon"))
    if lat is None or lon is None:
        return None
    return GeoPoint(lat, lon, _float_or_none(row.get("vehicle_rel_alt_m")))


def latest_mark(rows: list[dict[str, Any]], role: str) -> GeoPoint | None:
    for row in reversed(rows):
        if str(row.get("dooaf_role") or DOOAF_ROLE_SURVEY) != role:
            continue
        pt = point_from_row(row)
        if pt is not None:
            return pt
    return None


def latest_mark_row(
    rows: list[dict[str, Any]], role: str
) -> dict[str, Any] | None:
    for row in reversed(rows):
        if str(row.get("dooaf_role") or DOOAF_ROLE_SURVEY) != role:
            continue
        if point_from_row(row) is not None:
            return row
    return None


def point_from_latlon(
    *,
    lat: float | None,
    lon: float | None,
    alt_m: float | None = None,
) -> GeoPoint | None:
    if lat is None or lon is None:
        return None
    return GeoPoint(lat, lon, alt_m)


def gun_from_settings(
    *,
    gun_lat: float | None,
    gun_lon: float | None,
    gun_alt_m: float | None,
) -> GeoPoint | None:
    return point_from_latlon(lat=gun_lat, lon=gun_lon, alt_m=gun_alt_m)


def _qs_float(st: Any, key: str) -> float | None:
    raw = st.value(key)
    if raw is None or raw == "":
        return None
    try:
        v = float(raw)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(v):
        return None
    return v


def read_dooaf_settings(st: Any) -> DooafSettings:
    return DooafSettings(
        gun_lat=_qs_float(st, _QS_GUN_LAT),
        gun_lon=_qs_float(st, _QS_GUN_LON),
        gun_alt_m=_qs_float(st, _QS_GUN_ALT),
        target_lat=_qs_float(st, _QS_TARGET_LAT),
        target_lon=_qs_float(st, _QS_TARGET_LON),
        target_alt_m=_qs_float(st, _QS_TARGET_ALT),
    )


def merge_dooaf_settings(
    base: DooafSettings,
    update: DooafSettings,
) -> DooafSettings:
    """Keep base values where update leaves a coordinate pair empty."""
    gun = (
        (update.gun_lat, update.gun_lon, update.gun_alt_m)
        if update.gun_lat is not None and update.gun_lon is not None
        else (base.gun_lat, base.gun_lon, base.gun_alt_m)
    )
    target = (
        (update.target_lat, update.target_lon, update.target_alt_m)
        if update.target_lat is not None and update.target_lon is not None
        else (base.target_lat, base.target_lon, base.target_alt_m)
    )
    return DooafSettings(
        gun_lat=gun[0],
        gun_lon=gun[1],
        gun_alt_m=gun[2],
        target_lat=target[0],
        target_lon=target[1],
        target_alt_m=target[2],
    )


def apply_map_pick_to_settings(
    base: DooafSettings,
    role: str,
    lat: float,
    lon: float,
) -> DooafSettings:
    if role == DOOAF_ROLE_GUN:
        return DooafSettings(
            gun_lat=float(lat),
            gun_lon=float(lon),
            gun_alt_m=base.gun_alt_m,
            target_lat=base.target_lat,
            target_lon=base.target_lon,
            target_alt_m=base.target_alt_m,
        )
    if role == DOOAF_ROLE_INTENDED:
        return DooafSettings(
            gun_lat=base.gun_lat,
            gun_lon=base.gun_lon,
            gun_alt_m=base.gun_alt_m,
            target_lat=float(lat),
            target_lon=float(lon),
            target_alt_m=base.target_alt_m,
        )
    return base


def resolved_dooaf_settings(
    st: Any,
    rows: list[dict[str, Any]] | None = None,
) -> DooafSettings:
    """QSettings merged with latest gun/target map marks."""
    base = read_dooaf_settings(st)
    if not rows:
        return base
    gun = latest_mark(rows, DOOAF_ROLE_GUN)
    tgt = latest_mark(rows, DOOAF_ROLE_INTENDED)
    return DooafSettings(
        gun_lat=base.gun_lat if base.gun_lat is not None else (gun.lat if gun else None),
        gun_lon=base.gun_lon if base.gun_lon is not None else (gun.lon if gun else None),
        gun_alt_m=base.gun_alt_m if base.gun_alt_m is not None else (gun.alt_m if gun else None),
        target_lat=base.target_lat
        if base.target_lat is not None
        else (tgt.lat if tgt else None),
        target_lon=base.target_lon
        if base.target_lon is not None
        else (tgt.lon if tgt else None),
        target_alt_m=base.target_alt_m
        if base.target_alt_m is not None
        else (tgt.alt_m if tgt else None),
    )


def dooaf_settings_kwargs(settings: DooafSettings) -> dict[str, float | None]:
    return {
        "gun_lat": settings.gun_lat,
        "gun_lon": settings.gun_lon,
        "gun_alt_m": settings.gun_alt_m,
        "target_lat": settings.target_lat,
        "target_lon": settings.target_lon,
        "target_alt_m": settings.target_alt_m,
    }


def write_dooaf_settings(st: Any, settings: DooafSettings) -> None:
    for key, val in (
        (_QS_GUN_LAT, settings.gun_lat),
        (_QS_GUN_LON, settings.gun_lon),
        (_QS_GUN_ALT, settings.gun_alt_m),
        (_QS_TARGET_LAT, settings.target_lat),
        (_QS_TARGET_LON, settings.target_lon),
        (_QS_TARGET_ALT, settings.target_alt_m),
    ):
        if val is None:
            st.remove(key)
        else:
            st.setValue(key, float(val))


def validate_dooaf_settings(settings: DooafSettings) -> str | None:
    """Return error message, or None when coordinates are valid."""
    if settings.gun_lat is not None or settings.gun_lon is not None:
        if settings.gun_lat is None or settings.gun_lon is None:
            return "Artillery position needs both latitude and longitude."
        if not (-90.0 <= settings.gun_lat <= 90.0):
            return "Artillery latitude must be between -90 and 90."
        if not (-180.0 <= settings.gun_lon <= 180.0):
            return "Artillery longitude must be between -180 and 180."
    if settings.target_lat is not None or settings.target_lon is not None:
        if settings.target_lat is None or settings.target_lon is None:
            return "Actual target needs both latitude and longitude."
        if not (-90.0 <= settings.target_lat <= 90.0):
            return "Target latitude must be between -90 and 90."
        if not (-180.0 <= settings.target_lon <= 180.0):
            return "Target longitude must be between -180 and 180."
    return None


def build_dooaf_session(
    rows: list[dict[str, Any]],
    *,
    gun_lat: float | None = None,
    gun_lon: float | None = None,
    gun_alt_m: float | None = None,
    target_lat: float | None = None,
    target_lon: float | None = None,
    target_alt_m: float | None = None,
) -> DooafSession:
    gun = latest_mark(rows, DOOAF_ROLE_GUN) or gun_from_settings(
        gun_lat=gun_lat, gun_lon=gun_lon, gun_alt_m=gun_alt_m
    )
    intended = latest_mark(rows, DOOAF_ROLE_INTENDED) or point_from_latlon(
        lat=target_lat, lon=target_lon, alt_m=target_alt_m
    )
    impact = latest_mark(rows, DOOAF_ROLE_IMPACT)
    drone = drone_from_row(rows[-1] if rows else None)
    correction = None
    if gun is not None and intended is not None and impact is not None:
        correction = compute_fire_correction(gun, intended, impact)
    return DooafSession(
        gun=gun,
        intended=intended,
        impact=impact,
        drone=drone,
        correction=correction,
    )


def format_fire_correction(corr: FireCorrection) -> str:
    return (
        f"Δrange {corr.range_correction_m:+.0f} m, "
        f"Δdeflection {corr.deflection_correction_m:+.0f} m (R+), "
        f"miss {corr.impact_to_intended_m:.0f} m"
    )


def format_gimbal_yaw_direction(yaw_deg: float | None) -> str:
    """Human label for gimbal yaw (+ right, − left)."""
    if yaw_deg is None:
        return "N/A"
    y = float(yaw_deg)
    if abs(y) < 0.05:
        return "Yaw centre (0°)"
    if y > 0:
        return f"Yaw right {abs(y):.1f}°"
    return f"Yaw left {abs(y):.1f}°"


def format_gimbal_pitch_direction(pitch_deg: float | None) -> str:
    """Human label for gimbal pitch (+ up, − down)."""
    if pitch_deg is None:
        return "N/A"
    p = float(pitch_deg)
    if abs(p) < 0.05:
        return "Pitch level (0°)"
    if p > 0:
        return f"Pitch up {abs(p):.1f}°"
    return f"Pitch down {abs(p):.1f}°"


def observation_report_html_style() -> str:
    return (
        "body{font-family:Segoe UI,Arial,sans-serif;padding:20px;}"
        "table{border-collapse:collapse;width:100%;margin-bottom:14px;}"
        "th,td{border:1px solid #ccc;padding:6px;font-size:12px;}"
        "th{background:#f3f6fb;text-align:left;}"
        "h3{margin:16px 0 8px 0;font-size:15px;}"
        ".dooaf-fire-corr th,.dooaf-fire-corr td{color:#b71c1c;font-weight:600;"
        "background:#ffebee;}"
        ".dooaf-fire-corr h3{color:#b71c1c;}"
        ".dooaf-target-coords td{color:#0d47a1;font-weight:600;background:#e3f2fd;}"
        ".dooaf-impact-coords td{color:#1b5e20;font-weight:600;background:#e8f5e9;}"
        ".dooaf-camera td:last-child{font-weight:600;}"
    )


def observation_report_html_head(title: str = "Observation Summary") -> str:
    return (
        "<!doctype html><html><head><meta charset='utf-8'/>"
        f"<title>{title}</title>"
        f"<style>{observation_report_html_style()}</style>"
        "</head><body>"
    )


def format_camera_orientation_html(row: dict[str, Any] | None) -> str:
    if row is None:
        return ""
    yaw = _float_or_none(row.get("gimbal_yaw_deg"))
    pitch = _float_or_none(row.get("gimbal_pitch_deg"))
    yaw_raw = f"{yaw:.2f}°" if yaw is not None else "N/A"
    pitch_raw = f"{pitch:.2f}°" if pitch is not None else "N/A"
    return (
        "<h3>Camera / gimbal at observation</h3>"
        "<table class='dooaf-camera'><tbody>"
        f"<tr><td>Gimbal yaw</td><td>{yaw_raw} — {format_gimbal_yaw_direction(yaw)}</td></tr>"
        f"<tr><td>Gimbal pitch</td><td>{pitch_raw} — {format_gimbal_pitch_direction(pitch)}</td></tr>"
        "</tbody></table>"
    )


def format_dooaf_status(session: DooafSession) -> str:
    parts: list[str] = []
    if session.gun is not None:
        parts.append(f"{dooaf_role_display(DOOAF_ROLE_GUN)} set")
    if session.intended is not None:
        parts.append(f"{dooaf_role_display(DOOAF_ROLE_INTENDED)} set")
    if session.impact is not None:
        parts.append(f"{dooaf_role_display(DOOAF_ROLE_IMPACT)} marked")
    if session.correction is not None:
        parts.append(format_fire_correction(session.correction))
    if not parts:
        return "DOOAF: use DOOAF Setup for gun + target, then mark fall of shot"
    return "DOOAF: " + "; ".join(parts)


def format_dooaf_html_summary(
    session: DooafSession,
    *,
    observation_row: dict[str, Any] | None = None,
) -> str:
    def _pt(
        label: str,
        pt: GeoPoint | None,
        *,
        row_class: str = "",
    ) -> str:
        if pt is None:
            cls = f" class='{row_class}'" if row_class else ""
            return f"<tr{cls}><td>{label}</td><td colspan='2'>—</td></tr>"
        alt = f", alt {pt.alt_m:.1f} m" if pt.alt_m is not None else ""
        cls = f" class='{row_class}'" if row_class else ""
        return (
            f"<tr{cls}><td>{label}</td>"
            f"<td>{pt.lat:.7f}</td><td>{pt.lon:.7f}{alt}</td></tr>"
        )

    corr_rows = ""
    c = session.correction
    if c is not None:
        corr_rows = (
            "<div class='dooaf-fire-corr'>"
            "<h3>Fire correction</h3>"
            "<table>"
            f"<tr><td>Range correction (add)</td><td>{c.range_correction_m:+.1f} m</td></tr>"
            f"<tr><td>Deflection correction (add, R+)</td><td>{c.deflection_correction_m:+.1f} m</td></tr>"
            f"<tr><td>Miss along line</td><td>{c.miss_along_m:+.1f} m</td></tr>"
            f"<tr><td>Miss right</td><td>{c.miss_right_m:+.1f} m</td></tr>"
            f"<tr><td>Impact ↔ intended</td><td>{c.impact_to_intended_m:.1f} m</td></tr>"
            f"<tr><td>Miss north / east</td><td>{c.miss_north_m:+.1f} / {c.miss_east_m:+.1f} m</td></tr>"
            f"<tr><td>Gun → target range</td><td>{c.range_gun_to_intended_m:.1f} m</td></tr>"
            f"<tr><td>Gun → impact range</td><td>{c.range_gun_to_impact_m:.1f} m</td></tr>"
            f"<tr><td>Gun → target bearing</td><td>{c.bearing_gun_to_intended_deg:.1f}° "
            "(compass from gun to target, not gimbal)</td></tr>"
            "</table></div>"
        )
    obs_row = observation_row or None
    return (
        "<h3>DOOAF session</h3>"
        "<table><thead><tr><th>Variable</th><th>Lat</th><th>Lon</th></tr></thead><tbody>"
        + _pt(dooaf_role_display(DOOAF_ROLE_GUN), session.gun)
        + _pt(
            dooaf_role_display(DOOAF_ROLE_INTENDED),
            session.intended,
            row_class="dooaf-target-coords",
        )
        + _pt("Drone (last obs)", session.drone)
        + _pt(
            dooaf_role_display(DOOAF_ROLE_IMPACT),
            session.impact,
            row_class="dooaf-impact-coords",
        )
        + "</tbody></table>"
        + format_camera_orientation_html(obs_row)
        + corr_rows
    )


def dooaf_intended_impact_video_segment(
    rows: list[dict[str, Any]],
) -> tuple[float, float, float, float, str] | None:
    """Video overlay line from intended target mark to impact mark."""
    intended_row = latest_mark_row(rows, DOOAF_ROLE_INTENDED)
    impact_row = latest_mark_row(rows, DOOAF_ROLE_IMPACT)
    if intended_row is None or impact_row is None:
        return None
    ix = intended_row.get("video_x_norm")
    iy = intended_row.get("video_y_norm")
    jx = impact_row.get("video_x_norm")
    jy = impact_row.get("video_y_norm")
    if ix is None or iy is None or jx is None or jy is None:
        return None
    session = build_dooaf_session(rows)
    label = "impact"
    if session.correction is not None:
        label = format_fire_correction(session.correction)
    return (float(ix), float(iy), float(jx), float(jy), label)
