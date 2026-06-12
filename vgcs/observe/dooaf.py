"""
DOOAF (Detection, Observation, Orientation & Adjustment of Fire) session state.

Tracks artillery variables per field workflow:
  gun origin → intended target → drone observe → impact mark → fire correction.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from vgcs.observe.grid_reference import format_grid_reference
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
    *,
    alt_m: float | None = None,
) -> DooafSettings:
    if role == DOOAF_ROLE_GUN:
        gun_alt = float(alt_m) if alt_m is not None else base.gun_alt_m
        return DooafSettings(
            gun_lat=float(lat),
            gun_lon=float(lon),
            gun_alt_m=gun_alt,
            target_lat=base.target_lat,
            target_lon=base.target_lon,
            target_alt_m=base.target_alt_m,
        )
    if role == DOOAF_ROLE_INTENDED:
        tgt_alt = float(alt_m) if alt_m is not None else base.target_alt_m
        return DooafSettings(
            gun_lat=base.gun_lat,
            gun_lon=base.gun_lon,
            gun_alt_m=base.gun_alt_m,
            target_lat=float(lat),
            target_lon=float(lon),
            target_alt_m=tgt_alt,
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


def _html_esc(text: object) -> str:
    import html

    return html.escape(str(text if text is not None else ""), quote=True)


def _format_report_timestamp(ts: object) -> str:
    if ts is None or str(ts).strip() == "":
        return "—"
    raw = str(ts).strip()
    try:
        from datetime import datetime

        dt = datetime.fromisoformat(raw.replace("Z", "+00:00"))
        return dt.strftime("%Y-%m-%d %H:%M:%S")
    except Exception:
        return raw


def _report_section_card(title: str, body: str, *, extra_class: str = "") -> str:
    cls = "section-card"
    if extra_class:
        cls += f" {extra_class}"
    return (
        f"<section class='{cls}'>"
        f"<h3 class='section-title'>{_html_esc(title)}</h3>"
        f"{body}"
        "</section>"
    )


def format_geo_quality_badge(quality: object) -> str:
    q = str(quality or "").strip().lower()
    if not q:
        return "<span class='badge badge-muted'>—</span>"
    tone = "badge-muted"
    if q in ("good", "map_direct", "ok"):
        tone = "badge-good"
    elif q == "insufficient":
        tone = "badge-bad"
    elif q in ("weak", "degraded", "fair"):
        tone = "badge-warn"
    return f"<span class='badge {tone}'>{_html_esc(quality)}</span>"


def format_geo_method_badge(method: object) -> str:
    m = str(method or "").strip().lower()
    if not m:
        return "<span class='badge badge-muted'>—</span>"
    tone = "badge-info"
    label = str(method)
    if m == "ray_terrain_dem":
        tone = "badge-dem"
        label = "ray_terrain_dem (DEM)"
    elif m == "map_click":
        tone = "badge-muted"
    elif m.startswith("ray_ground"):
        tone = "badge-info"
    return f"<span class='badge {tone}'>{_html_esc(label)}</span>"


def observation_report_html_style() -> str:
    return (
        ":root{"
        "--bg:#eef2f7;--card:#fff;--border:#d8dee9;--text:#1e293b;--muted:#64748b;"
        "--header:#0f172a;--accent:#2563eb;--good:#15803d;--warn:#b45309;--bad:#b91c1c;"
        "--target:#1d4ed8;--impact:#15803d;--corr:#c2410c;--dem:#0369a1;"
        "}"
        "*{box-sizing:border-box;}"
        "body{margin:0;background:var(--bg);color:var(--text);"
        "font-family:Segoe UI,system-ui,-apple-system,Arial,sans-serif;line-height:1.45;}"
        ".report-page{max-width:1280px;margin:0 auto;padding:24px 20px 40px;}"
        ".report-header{background:linear-gradient(135deg,#0f172a 0%,#1e3a5f 100%);"
        "color:#f8fafc;border-radius:14px;padding:22px 26px;margin-bottom:20px;"
        "box-shadow:0 8px 24px rgba(15,23,42,.18);}"
        ".report-header h1{margin:0 0 6px;font-size:22px;font-weight:700;letter-spacing:.01em;}"
        ".report-meta{display:flex;flex-wrap:wrap;gap:10px 18px;font-size:13px;color:#cbd5e1;}"
        ".report-meta strong{color:#f8fafc;font-weight:600;}"
        ".section-card{background:var(--card);border:1px solid var(--border);border-radius:12px;"
        "padding:16px 18px;margin-bottom:16px;box-shadow:0 1px 3px rgba(15,23,42,.06);}"
        ".section-title{margin:0 0 12px;font-size:15px;font-weight:700;color:var(--text);}"
        ".data-table{width:100%;border-collapse:separate;border-spacing:0;font-size:12px;}"
        ".data-table th,.data-table td{border-bottom:1px solid var(--border);padding:8px 10px;"
        "vertical-align:top;text-align:left;}"
        ".data-table thead th{background:#f1f5f9;color:#334155;font-weight:600;font-size:11px;"
        "text-transform:uppercase;letter-spacing:.04em;border-bottom:2px solid var(--border);}"
        ".data-table tbody tr:last-child td{border-bottom:none;}"
        ".data-table tbody tr:hover td{background:#f8fafc;}"
        ".data-table .label-col{font-weight:600;color:#334155;width:28%;}"
        ".mono{font-family:Consolas,Monaco,ui-monospace,monospace;font-size:11px;}"
        ".muted{color:var(--muted);}"
        ".table-scroll{margin-top:4px;border:1px solid var(--border);border-radius:8px;"
        "overflow:auto;max-width:100%;background:#fff;}"
        ".table-scroll .data-table{margin:0;}"
        ".table-scroll thead th{position:sticky;top:0;z-index:1;box-shadow:0 1px 0 var(--border);}"
        ".badge{display:inline-block;padding:2px 8px;border-radius:999px;font-size:11px;"
        "font-weight:600;line-height:1.5;white-space:nowrap;}"
        ".badge-good{background:#dcfce7;color:var(--good);}"
        ".badge-bad{background:#fee2e2;color:var(--bad);}"
        ".badge-warn{background:#ffedd5;color:var(--warn);}"
        ".badge-info{background:#e0f2fe;color:#0369a1;}"
        ".badge-dem{background:#e0f2fe;color:var(--dem);border:1px solid #7dd3fc;}"
        ".badge-muted{background:#f1f5f9;color:var(--muted);}"
        ".dooaf-target-coords td{color:var(--target);font-weight:600;background:#eff6ff;}"
        ".dooaf-impact-coords td{color:var(--impact);font-weight:600;background:#ecfdf5;}"
        ".dooaf-fire-corr{border-color:#fed7aa;background:linear-gradient(180deg,#fff7ed 0%,#fff 40%);}"
        ".metrics-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));"
        "gap:12px;margin-bottom:14px;}"
        ".metric-card{background:#fff;border:1px solid #fdba74;border-radius:10px;padding:12px 14px;}"
        ".metric-label{font-size:11px;color:var(--muted);text-transform:uppercase;"
        "letter-spacing:.04em;margin-bottom:4px;}"
        ".metric-value{font-size:22px;font-weight:700;color:var(--corr);line-height:1.2;}"
        ".metric-sub{font-size:11px;color:var(--muted);margin-top:4px;}"
        ".camera-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:12px;}"
        ".camera-stat{background:#f8fafc;border:1px solid var(--border);border-radius:10px;padding:12px 14px;}"
        ".camera-stat .label{font-size:11px;color:var(--muted);text-transform:uppercase;"
        "letter-spacing:.04em;margin-bottom:6px;}"
        ".camera-stat .value{font-size:15px;font-weight:600;color:var(--text);}"
        ".path-cell{max-width:140px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;}"
        "th.col-geo,td.col-geo{background:#f0f9ff;}"
        "th.col-target,td.col-target{background:#f8fafc;}"
        ".log-entries{display:flex;flex-direction:column;gap:16px;}"
        ".log-entry{border:1px solid var(--border);border-radius:12px;background:#fff;overflow:hidden;}"
        ".log-entry-impact{border-color:#4ade80;box-shadow:0 0 0 1px rgba(74,222,128,.25);}"
        ".log-entry-head{display:flex;flex-wrap:wrap;align-items:center;gap:8px 12px;"
        "padding:12px 16px;background:#f8fafc;border-bottom:1px solid var(--border);}"
        ".log-entry-impact .log-entry-head{background:linear-gradient(90deg,#ecfdf5 0%,#f8fafc 100%);}"
        ".log-entry-index{font-weight:700;font-size:13px;color:var(--text);}"
        ".log-entry-time{font-size:12px;color:var(--muted);"
        "font-family:Consolas,Monaco,ui-monospace,monospace;}"
        ".log-entry-badges{display:flex;flex-wrap:wrap;align-items:center;gap:6px;margin-left:auto;}"
        ".log-metrics{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:0;"
        "border-bottom:1px solid var(--border);background:#fff;}"
        ".log-metric{padding:14px 16px;border-right:1px solid var(--border);min-height:78px;}"
        ".log-metric:last-child{border-right:none;}"
        ".log-metric-label{font-size:10px;text-transform:uppercase;letter-spacing:.05em;"
        "color:var(--muted);font-weight:600;margin-bottom:8px;}"
        ".log-metric-value{font-size:15px;font-weight:600;color:var(--text);line-height:1.3;}"
        ".log-metric-value.mgrs{font-family:Consolas,Monaco,ui-monospace,monospace;font-size:12px;}"
        ".log-metric-sub{font-size:11px;color:var(--muted);margin-top:4px;}"
        ".log-detail-table{width:100%;border-collapse:collapse;font-size:12px;}"
        ".log-detail-table th{width:34%;padding:9px 16px;text-align:left;font-weight:600;"
        "color:#475569;background:#fafbfc;border-bottom:1px solid var(--border);vertical-align:top;}"
        ".log-detail-table td{padding:9px 16px;border-bottom:1px solid var(--border);"
        "vertical-align:top;color:var(--text);}"
        ".log-detail-section td{background:#f1f5f9;color:#334155;font-size:10px;font-weight:700;"
        "text-transform:uppercase;letter-spacing:.06em;padding:7px 16px;border-bottom:1px solid var(--border);}"
        ".log-detail-table tbody tr:last-child th,.log-detail-table tbody tr:last-child td{border-bottom:none;}"
        ".mgrs-badge{display:inline-block;font-family:Consolas,Monaco,ui-monospace,monospace;"
        "font-size:12px;padding:4px 10px;background:#e2e8f0;border-radius:6px;"
        "white-space:nowrap;color:#334155;}"
        ".coord-pair{cursor:help;border-bottom:1px dotted #94a3b8;}"
        ".kind-badge{background:#e0e7ff;color:#3730a3;}"
        ".role-badge{background:#f1f5f9;color:#475569;}"
        ".file-link{color:var(--accent);text-decoration:none;border-bottom:1px solid transparent;}"
        ".file-link:hover{border-bottom-color:var(--accent);}"
        ".log-hint{font-size:12px;color:var(--muted);margin:0 0 12px;line-height:1.5;}"
        "@media (max-width:900px){"
        ".log-metrics{grid-template-columns:repeat(2,minmax(0,1fr));}"
        ".log-metric:nth-child(2){border-right:none;}"
        ".log-metric{border-bottom:1px solid var(--border);}"
        "}"
        "@media print{"
        "body{background:#fff;}.report-page{padding:0;max-width:none;}"
        ".report-header{box-shadow:none;border-radius:0;}"
        ".section-card{box-shadow:none;break-inside:avoid;}"
        ".table-scroll{overflow:visible;border:none;}"
        ".table-scroll thead th{position:static;}"
        ".log-entry{break-inside:avoid;}"
        ".log-metrics{grid-template-columns:repeat(4,minmax(0,1fr));}"
        "}"
    )


def observation_report_html_head(title: str = "Observation Report") -> str:
    return (
        "<!doctype html><html lang='en'><head><meta charset='utf-8'/>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'/>"
        f"<title>{_html_esc(title)}</title>"
        f"<style>{observation_report_html_style()}</style>"
        "</head><body><div class='report-page'>"
    )


def observation_report_html_footer() -> str:
    return "</div></body></html>"


def format_observation_report_header(entry_count: int, *, title: str = "Observation Report") -> str:
    from datetime import datetime, timezone

    exported = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC")
    return (
        "<header class='report-header'>"
        f"<h1>{_html_esc(title)}</h1>"
        "<div class='report-meta'>"
        f"<span><strong>Entries</strong> {int(entry_count)}</span>"
        f"<span><strong>Exported</strong> {exported}</span>"
        "<span><strong>Source</strong> VGCS observation export</span>"
        "</div></header>"
    )


def format_camera_orientation_html(row: dict[str, Any] | None) -> str:
    if row is None:
        return ""
    yaw = _float_or_none(row.get("gimbal_yaw_deg"))
    pitch = _float_or_none(row.get("gimbal_pitch_deg"))
    yaw_raw = f"{yaw:.2f}°" if yaw is not None else "N/A"
    pitch_raw = f"{pitch:.2f}°" if pitch is not None else "N/A"
    body = (
        "<div class='camera-grid'>"
        "<div class='camera-stat'>"
        "<div class='label'>Gimbal yaw</div>"
        f"<div class='value'>{_html_esc(yaw_raw)} — {_html_esc(format_gimbal_yaw_direction(yaw))}</div>"
        "</div>"
        "<div class='camera-stat'>"
        "<div class='label'>Gimbal pitch</div>"
        f"<div class='value'>{_html_esc(pitch_raw)} — {_html_esc(format_gimbal_pitch_direction(pitch))}</div>"
        "</div>"
        "</div>"
    )
    return _report_section_card("Camera / gimbal at observation", body, extra_class="dooaf-camera")


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
            return f"<tr{cls}><td>{label}</td><td colspan='3'>—</td></tr>"
        alt = f", alt {pt.alt_m:.1f} m" if pt.alt_m is not None else ""
        gr = format_grid_reference(pt.lat, pt.lon) or "—"
        cls = f" class='{row_class}'" if row_class else ""
        return (
            f"<tr{cls}><td>{label}</td>"
            f"<td>{pt.lat:.7f}</td><td>{pt.lon:.7f}</td>"
            f"<td>{gr}{alt}</td></tr>"
        )

    corr_rows = ""
    c = session.correction
    if c is not None:
        corr_rows = _report_section_card(
            "Fire correction",
            (
                "<div class='metrics-grid'>"
                "<div class='metric-card'>"
                "<div class='metric-label'>Range correction (add)</div>"
                f"<div class='metric-value'>{c.range_correction_m:+.1f} m</div>"
                "</div>"
                "<div class='metric-card'>"
                "<div class='metric-label'>Deflection (add, R+)</div>"
                f"<div class='metric-value'>{c.deflection_correction_m:+.1f} m</div>"
                "</div>"
                "<div class='metric-card'>"
                "<div class='metric-label'>Impact ↔ intended</div>"
                f"<div class='metric-value'>{c.impact_to_intended_m:.1f} m</div>"
                "<div class='metric-sub'>Total miss distance</div>"
                "</div>"
                "</div>"
                "<table class='data-table dooaf-fire-corr'>"
                "<tbody>"
                f"<tr><td class='label-col'>Miss along line</td><td>{c.miss_along_m:+.1f} m</td></tr>"
                f"<tr><td class='label-col'>Miss right</td><td>{c.miss_right_m:+.1f} m</td></tr>"
                f"<tr><td class='label-col'>Miss north / east</td>"
                f"<td>{c.miss_north_m:+.1f} m / {c.miss_east_m:+.1f} m</td></tr>"
                f"<tr><td class='label-col'>Gun → target range</td>"
                f"<td>{c.range_gun_to_intended_m:.1f} m</td></tr>"
                f"<tr><td class='label-col'>Gun → impact range</td>"
                f"<td>{c.range_gun_to_impact_m:.1f} m</td></tr>"
                f"<tr><td class='label-col'>Gun → target bearing</td>"
                f"<td>{c.bearing_gun_to_intended_deg:.1f}° "
                "<span class='muted'>(compass from gun to target, not gimbal)</span></td></tr>"
                "</tbody></table>"
            ),
            extra_class="dooaf-fire-corr",
        )
    obs_row = observation_row or None
    session_body = (
        "<table class='data-table'>"
        "<thead><tr><th>Variable</th><th>Lat</th><th>Lon</th><th>Grid ref (MGRS)</th></tr></thead>"
        "<tbody>"
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
    )
    return (
        _report_section_card("DOOAF session", session_body)
        + format_camera_orientation_html(obs_row)
        + corr_rows
    )


def _cell_text(val: object, cell_fn: Any | None = None) -> str:
    if cell_fn is not None:
        return str(cell_fn(val)).strip()
    return str(val if val is not None else "").strip()


def _is_missing_cell(val: object, cell_fn: Any | None = None) -> bool:
    s = _cell_text(val, cell_fn)
    return not s or s.upper() == "N/A"


def _format_scalar_cell(val: object, cell_fn: Any | None = None) -> str:
    if _is_missing_cell(val, cell_fn):
        return "<span class='muted'>—</span>"
    return _html_esc(_cell_text(val, cell_fn))


def _format_distance_m_html(val: object, cell_fn: Any | None = None) -> str:
    if _is_missing_cell(val, cell_fn):
        return "<span class='muted'>—</span>"
    try:
        return _html_esc(f"{float(val):.1f} m")
    except (TypeError, ValueError):
        return _format_scalar_cell(val, cell_fn)


def _format_alt_m_html(val: object, cell_fn: Any | None = None) -> str:
    if _is_missing_cell(val, cell_fn):
        return "<span class='muted'>—</span>"
    try:
        return _html_esc(f"{float(val):.2f} m")
    except (TypeError, ValueError):
        return _format_scalar_cell(val, cell_fn)


def _format_deg_html(val: object, cell_fn: Any | None = None) -> str:
    if _is_missing_cell(val, cell_fn):
        return "<span class='muted'>—</span>"
    try:
        return _html_esc(f"{float(val):.1f}°")
    except (TypeError, ValueError):
        return _format_scalar_cell(val, cell_fn)


def _format_hdop_html(val: object, cell_fn: Any | None = None) -> str:
    if _is_missing_cell(val, cell_fn):
        return "<span class='muted'>—</span>"
    try:
        return _html_esc(f"{float(val):.2f}")
    except (TypeError, ValueError):
        return _format_scalar_cell(val, cell_fn)


def _coords_populated(lat: object, lon: object, cell_fn: Any) -> bool:
    return not _is_missing_cell(lat, cell_fn) and not _is_missing_cell(lon, cell_fn)


def _same_coords(
    lat1: object,
    lon1: object,
    lat2: object,
    lon2: object,
    cell_fn: Any,
) -> bool:
    if not _coords_populated(lat1, lon1, cell_fn) or not _coords_populated(lat2, lon2, cell_fn):
        return False
    try:
        return (
            abs(float(lat1) - float(lat2)) < 1e-9
            and abs(float(lon1) - float(lon2)) < 1e-9
        )
    except (TypeError, ValueError):
        return _cell_text(lat1, cell_fn) == _cell_text(lat2, cell_fn) and _cell_text(
            lon1, cell_fn
        ) == _cell_text(lon2, cell_fn)


def _format_coord_pair_html(lat: object, lon: object, cell_fn: Any) -> str:
    lat_s = str(cell_fn(lat)).strip()
    lon_s = str(cell_fn(lon)).strip()
    if lat_s.upper() == "N/A" or lon_s.upper() == "N/A" or not lat_s or not lon_s:
        return "<span class='muted'>—</span>"
    try:
        lat_f = float(lat)
        lon_f = float(lon)
        short = f"{lat_f:.6f}, {lon_f:.6f}"
        full = f"{lat_f}, {lon_f}"
    except (TypeError, ValueError):
        short = f"{lat_s}, {lon_s}"
        full = short
    return (
        f"<span class='coord-pair mono' title='{_html_esc(full)}'>"
        f"{_html_esc(short)}</span>"
    )


def _format_mgrs_badge(gr: object) -> str:
    s = str(gr or "").strip()
    if not s:
        return "<span class='muted'>—</span>"
    return f"<span class='mgrs-badge'>{_html_esc(s)}</span>"


def _format_kind_badge(kind: object) -> str:
    k = str(kind or "").strip()
    if not k:
        return "<span class='badge badge-muted'>—</span>"
    return f"<span class='badge kind-badge'>{_html_esc(k.replace('_', ' '))}</span>"


def _format_role_badge(role: object) -> str:
    r = str(role or "").strip()
    if not r:
        return "<span class='badge badge-muted'>—</span>"
    label = DOOAF_ROLE_DISPLAY.get(r, r.replace("_", " "))
    tone = "role-badge"
    if r == DOOAF_ROLE_IMPACT:
        tone = "badge-good"
    elif r == DOOAF_ROLE_INTENDED:
        tone = "badge-info"
    elif r == DOOAF_ROLE_GUN:
        tone = "badge-muted"
    return f"<span class='badge {tone}'>{_html_esc(label)}</span>"


def _log_detail_section(title: str) -> str:
    return f"<tr class='log-detail-section'><td colspan='2'>{_html_esc(title)}</td></tr>"


def _log_detail_row(label: str, value_html: str) -> str:
    return f"<tr><th>{_html_esc(label)}</th><td>{value_html}</td></tr>"


def _row_has_gimbal_data(row: dict[str, object], cell_fn: Any) -> bool:
    keys = (
        "gimbal_yaw_deg",
        "gimbal_pitch_deg",
        "gimbal_yaw_direction",
        "gimbal_pitch_direction",
        "video_x_norm",
        "video_y_norm",
    )
    return any(not _is_missing_cell(row.get(k), cell_fn if k.endswith("_deg") or k.endswith("_norm") else None) for k in keys)


def _row_has_media(row: dict[str, object]) -> bool:
    return bool(str(row.get("snapshot_path") or "").strip() or str(row.get("clip_path") or "").strip())


def _row_has_vehicle_attitude(row: dict[str, object], cell_fn: Any) -> bool:
    return any(
        not _is_missing_cell(row.get(k), cell_fn)
        for k in ("vehicle_heading_deg", "vehicle_roll_deg", "vehicle_pitch_deg")
    )


def _row_has_geo_detail(row: dict[str, object], cell_fn: Any) -> bool:
    if not _is_missing_cell(row.get("geo_bearing_deg"), cell_fn):
        return True
    if not _is_missing_cell(row.get("geo_depression_deg"), cell_fn):
        return True
    if not _is_missing_cell(row.get("measure_agl_m"), cell_fn):
        return True
    if str(row.get("agl_source") or row.get("geo_agl_source") or "").strip():
        return True
    return False


def _row_has_fire_correction(row: dict[str, object], cell_fn: Any) -> bool:
    return any(
        not _is_missing_cell(row.get(k), cell_fn)
        for k in (
            "dooaf_range_correction_m",
            "dooaf_deflection_correction_m",
            "dooaf_miss_m",
        )
    )


def _position_section_title(row: dict[str, object]) -> str:
    role = str(row.get("dooaf_role") or "").strip()
    if role:
        return dooaf_role_display(role)
    kind = str(row.get("kind") or "").strip()
    if kind == "video_mark":
        return "Video mark (ground)"
    return "Map mark"


def _format_log_metrics_row(row: dict[str, object], cell_fn: Any) -> str:
    mgrs = str(
        row.get("target_grid_ref") or row.get("map_grid_ref") or ""
    ).strip()
    mgrs_html = (
        f"<span class='mgrs-badge'>{_html_esc(mgrs)}</span>"
        if mgrs
        else "<span class='muted'>—</span>"
    )
    geo_html = (
        format_geo_method_badge(row.get("geo_method"))
        + "<div class='log-metric-sub'>"
        + format_geo_quality_badge(row.get("geo_quality"))
        + "</div>"
    )
    sep_html = _format_distance_m_html(row.get("segment_distance_m"), cell_fn)
    range_html = _format_distance_m_html(row.get("geo_range_m"), cell_fn)
    alt_html = _format_alt_m_html(row.get("vehicle_rel_alt_m"), cell_fn)
    return (
        "<div class='log-metrics'>"
        "<div class='log-metric'>"
        "<div class='log-metric-label'>Grid reference</div>"
        f"<div class='log-metric-value mgrs'>{mgrs_html}</div>"
        "</div>"
        "<div class='log-metric'>"
        "<div class='log-metric-label'>Geo method</div>"
        f"<div class='log-metric-value'>{geo_html}</div>"
        "</div>"
        "<div class='log-metric'>"
        "<div class='log-metric-label'>Separation / range</div>"
        f"<div class='log-metric-value'>{sep_html}</div>"
        f"<div class='log-metric-sub'>Geo range {range_html}</div>"
        "</div>"
        "<div class='log-metric'>"
        "<div class='log-metric-label'>Drone rel. altitude</div>"
        f"<div class='log-metric-value'>{alt_html}</div>"
        "</div>"
        "</div>"
    )


def _format_path_cell(path: object) -> str:
    s = str(path or "").strip()
    if not s:
        return "<span class='muted'>—</span>"
    from pathlib import Path

    name = Path(s).name or s
    return (
        f"<span class='file-link mono' title='{_html_esc(s)}'>"
        f"{_html_esc(name)}</span>"
    )


def _format_observation_log_entry(
    idx: int,
    row: dict[str, object],
    cell_fn: Any,
) -> str:
    is_impact = str(row.get("dooaf_role") or "") == DOOAF_ROLE_IMPACT
    entry_cls = "log-entry log-entry-impact" if is_impact else "log-entry"
    warn = str(row.get("geo_warning") or "").strip()

    map_ok = _coords_populated(row.get("map_lat"), row.get("map_lon"), cell_fn)
    tgt_ok = _coords_populated(row.get("target_lat"), row.get("target_lon"), cell_fn)
    same_pos = _same_coords(
        row.get("map_lat"),
        row.get("map_lon"),
        row.get("target_lat"),
        row.get("target_lon"),
        cell_fn,
    )

    detail_rows: list[str] = []

    if map_ok and tgt_ok and same_pos:
        detail_rows.append(_log_detail_section(_position_section_title(row)))
        detail_rows.append(
            _log_detail_row(
                "Coordinates",
                _format_coord_pair_html(row.get("map_lat"), row.get("map_lon"), cell_fn),
            )
        )
        detail_rows.append(
            _log_detail_row(
                "Grid ref (MGRS)",
                _format_mgrs_badge(row.get("map_grid_ref") or row.get("target_grid_ref")),
            )
        )
        if not _is_missing_cell(row.get("target_alt_m"), cell_fn):
            detail_rows.append(
                _log_detail_row(
                    "Altitude (MSL)",
                    _format_alt_m_html(row.get("target_alt_m"), cell_fn),
                )
            )
    else:
        if map_ok:
            detail_rows.append(_log_detail_section("Map click"))
            detail_rows.append(
                _log_detail_row(
                    "Coordinates",
                    _format_coord_pair_html(row.get("map_lat"), row.get("map_lon"), cell_fn),
                )
            )
            detail_rows.append(
                _log_detail_row("Grid ref (MGRS)", _format_mgrs_badge(row.get("map_grid_ref")))
            )
            if not _is_missing_cell(row.get("target_alt_m"), cell_fn):
                detail_rows.append(
                    _log_detail_row(
                        "Altitude (MSL)",
                        _format_alt_m_html(row.get("target_alt_m"), cell_fn),
                    )
                )
        if tgt_ok:
            title = _position_section_title(row) if not map_ok else "Computed target"
            detail_rows.append(_log_detail_section(title))
            detail_rows.append(
                _log_detail_row(
                    "Coordinates",
                    _format_coord_pair_html(row.get("target_lat"), row.get("target_lon"), cell_fn),
                )
            )
            detail_rows.append(
                _log_detail_row("Grid ref (MGRS)", _format_mgrs_badge(row.get("target_grid_ref")))
            )
            if not _is_missing_cell(row.get("target_alt_m"), cell_fn):
                detail_rows.append(
                    _log_detail_row(
                        "Altitude (MSL)",
                        _format_alt_m_html(row.get("target_alt_m"), cell_fn),
                    )
                )

    if _row_has_geo_detail(row, cell_fn):
        detail_rows.append(_log_detail_section("Geo detail"))
        if not _is_missing_cell(row.get("geo_bearing_deg"), cell_fn):
            detail_rows.append(
                _log_detail_row(
                    "Geo bearing",
                    _format_deg_html(row.get("geo_bearing_deg"), cell_fn),
                )
            )
        if not _is_missing_cell(row.get("geo_depression_deg"), cell_fn):
            detail_rows.append(
                _log_detail_row(
                    "Depression angle",
                    _format_deg_html(row.get("geo_depression_deg"), cell_fn),
                )
            )
        if not _is_missing_cell(row.get("measure_agl_m"), cell_fn):
            detail_rows.append(
                _log_detail_row(
                    "Ray AGL used",
                    _format_alt_m_html(row.get("measure_agl_m"), cell_fn),
                )
            )
        agl_src = str(row.get("agl_source") or row.get("geo_agl_source") or "").strip()
        if agl_src:
            detail_rows.append(
                _log_detail_row("AGL source", _format_scalar_cell(agl_src))
            )

    if _coords_populated(row.get("vehicle_lat"), row.get("vehicle_lon"), cell_fn):
        detail_rows.append(_log_detail_section("Drone at observation"))
        detail_rows.append(
            _log_detail_row(
                "Coordinates",
                _format_coord_pair_html(row.get("vehicle_lat"), row.get("vehicle_lon"), cell_fn),
            )
        )
        detail_rows.append(
            _log_detail_row("Grid ref (MGRS)", _format_mgrs_badge(row.get("vehicle_grid_ref")))
        )
        if not _is_missing_cell(row.get("vehicle_rel_alt_m"), cell_fn):
            detail_rows.append(
                _log_detail_row(
                    "Rel. altitude",
                    _format_alt_m_html(row.get("vehicle_rel_alt_m"), cell_fn),
                )
            )

    if _row_has_vehicle_attitude(row, cell_fn):
        detail_rows.append(_log_detail_section("Vehicle attitude"))
        if not _is_missing_cell(row.get("vehicle_heading_deg"), cell_fn):
            detail_rows.append(
                _log_detail_row(
                    "Heading",
                    _format_deg_html(row.get("vehicle_heading_deg"), cell_fn),
                )
            )
        if not _is_missing_cell(row.get("vehicle_roll_deg"), cell_fn):
            detail_rows.append(
                _log_detail_row(
                    "Roll",
                    _format_deg_html(row.get("vehicle_roll_deg"), cell_fn),
                )
            )
        if not _is_missing_cell(row.get("vehicle_pitch_deg"), cell_fn):
            detail_rows.append(
                _log_detail_row(
                    "Pitch",
                    _format_deg_html(row.get("vehicle_pitch_deg"), cell_fn),
                )
            )

    if _row_has_fire_correction(row, cell_fn):
        detail_rows.append(_log_detail_section("Fire correction (this mark)"))
        if not _is_missing_cell(row.get("dooaf_range_correction_m"), cell_fn):
            detail_rows.append(
                _log_detail_row(
                    "Range correction (add)",
                    _format_distance_m_html(row.get("dooaf_range_correction_m"), cell_fn),
                )
            )
        if not _is_missing_cell(row.get("dooaf_deflection_correction_m"), cell_fn):
            detail_rows.append(
                _log_detail_row(
                    "Deflection (add, R+)",
                    _format_distance_m_html(row.get("dooaf_deflection_correction_m"), cell_fn),
                )
            )
        if not _is_missing_cell(row.get("dooaf_miss_m"), cell_fn):
            detail_rows.append(
                _log_detail_row(
                    "Impact ↔ intended",
                    _format_distance_m_html(row.get("dooaf_miss_m"), cell_fn),
                )
            )

    if _row_has_gimbal_data(row, cell_fn):
        detail_rows.append(_log_detail_section("Camera & video"))
        if not _is_missing_cell(row.get("gimbal_yaw_deg"), cell_fn):
            detail_rows.append(
                _log_detail_row("Gimbal yaw", _format_scalar_cell(row.get("gimbal_yaw_deg"), cell_fn))
            )
        if not _is_missing_cell(row.get("gimbal_pitch_deg"), cell_fn):
            detail_rows.append(
                _log_detail_row("Gimbal pitch", _format_scalar_cell(row.get("gimbal_pitch_deg"), cell_fn))
            )
        if not _is_missing_cell(row.get("gimbal_yaw_direction")):
            detail_rows.append(
                _log_detail_row("Yaw direction", _format_scalar_cell(row.get("gimbal_yaw_direction")))
            )
        if not _is_missing_cell(row.get("gimbal_pitch_direction")):
            detail_rows.append(
                _log_detail_row(
                    "Pitch direction",
                    _format_scalar_cell(row.get("gimbal_pitch_direction")),
                )
            )
        if not _is_missing_cell(row.get("video_x_norm"), cell_fn):
            detail_rows.append(
                _log_detail_row("Video X (norm)", _format_scalar_cell(row.get("video_x_norm"), cell_fn))
            )
        if not _is_missing_cell(row.get("video_y_norm"), cell_fn):
            detail_rows.append(
                _log_detail_row("Video Y (norm)", _format_scalar_cell(row.get("video_y_norm"), cell_fn))
            )

    gps_rows = (
        not _is_missing_cell(row.get("gps_fix_type"))
        or not _is_missing_cell(row.get("gps_satellites"))
        or not _is_missing_cell(row.get("gps_hdop"), cell_fn)
    )
    if gps_rows:
        detail_rows.append(_log_detail_section("GPS"))
        detail_rows.append(
            _log_detail_row("Fix type", _format_scalar_cell(row.get("gps_fix_type")))
        )
        detail_rows.append(
            _log_detail_row("Satellites", _format_scalar_cell(row.get("gps_satellites")))
        )
        detail_rows.append(_log_detail_row("HDOP", _format_hdop_html(row.get("gps_hdop"), cell_fn)))

    if _row_has_media(row):
        detail_rows.append(_log_detail_section("Media"))
        detail_rows.append(_log_detail_row("Snapshot", _format_path_cell(row.get("snapshot_path"))))
        detail_rows.append(_log_detail_row("Clip", _format_path_cell(row.get("clip_path"))))

    if warn:
        detail_rows.append(_log_detail_section("Notes"))
        detail_rows.append(
            _log_detail_row("Geo warning", f"<span class='muted'>{_html_esc(warn)}</span>")
        )

    detail_table = (
        "<table class='log-detail-table'><tbody>"
        + "".join(detail_rows)
        + "</tbody></table>"
    )

    return (
        f"<article class='{entry_cls}'>"
        "<header class='log-entry-head'>"
        f"<span class='log-entry-index'>#{idx}</span>"
        f"<span class='log-entry-time'>{_html_esc(_format_report_timestamp(row.get('timestamp_utc')))}</span>"
        "<div class='log-entry-badges'>"
        + _format_kind_badge(row.get("kind"))
        + _format_role_badge(row.get("dooaf_role"))
        + "</div></header>"
        + _format_log_metrics_row(row, cell_fn)
        + detail_table
        + "</article>"
    )


def format_observation_detailed_log_html(
    export_rows: list[dict[str, object]],
    cell_fn: Any,
) -> str:
    """Card-based detailed log — grouped fields instead of a wide scroll table."""
    if not export_rows:
        body = "<p class='muted'>No observation entries in this export.</p>"
        return _report_section_card("Detailed log", body)

    entries = "".join(
        _format_observation_log_entry(idx, row, cell_fn)
        for idx, row in enumerate(export_rows, start=1)
    )
    hint = (
        "<p class='log-hint'>Summary metrics above; expandable detail below. "
        "Hover lat/lon for full precision. "
        "<strong>Full raw export</strong> (every field, unrounded) is in the "
        "<span class='mono'>CSV</span> file saved beside this HTML.</p>"
    )
    body = hint + f"<div class='log-entries'>{entries}</div>"
    return _report_section_card("Detailed log", body)


def assemble_observation_report_html(
    entry_count: int,
    dooaf_summary_html: str,
    detailed_log_html: str,
    *,
    title: str = "Observation Report",
) -> str:
    return (
        observation_report_html_head(title=title)
        + format_observation_report_header(entry_count, title=title)
        + dooaf_summary_html
        + detailed_log_html
        + observation_report_html_footer()
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
