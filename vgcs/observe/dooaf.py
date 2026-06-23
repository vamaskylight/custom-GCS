"""
DOOAF (Detection, Observation, Orientation & Adjustment of Fire) session state.

Tracks artillery variables per field workflow:
  gun origin → intended target → drone observe → impact mark → fire correction.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any

from vgcs.observe.grid_reference import format_grid_reference
from vgcs.observe.dooaf_map_symbols import (
    bearing_deg as _dooaf_bearing_deg,
    svg_drone_marker as _fc_svg_marker_drone,
    svg_gun_marker as _fc_svg_marker_gun,
    svg_target_marker as _fc_svg_marker_crosshair,
)
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
    DOOAF_ROLE_IMPACT: "Impact Target",
    DOOAF_ROLE_GUN: "Artillery (gun)",
    DOOAF_ROLE_SURVEY: "Wall measure",
}

DOOAF_ROLE_TOOLTIPS: dict[str, str] = {
    DOOAF_ROLE_INTENDED: (
        "Planned impact point from military staff — where the round should land."
    ),
    DOOAF_ROLE_IMPACT: (
        "Mark Impact Target after firing — click burst or smoke on video. "
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
    miss_vertical_m: float | None = None
    elevation_correction_m: float | None = None


@dataclass(frozen=True)
class DooafSession:
    gun: GeoPoint | None
    intended: GeoPoint | None
    impact: GeoPoint | None
    drone: GeoPoint | None
    correction: FireCorrection | None
    building_height_m: float | None = None
    intended_dem_alt_m: float | None = None
    impact_dem_alt_m: float | None = None
    height_correction_m: float | None = None


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
_QS_PRESETS_JSON = "dooaf/presets_json"


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
    Horizontal miss is ground distance; vertical miss uses MSL altitudes when set.
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
    miss_vertical: float | None = None
    elev_correction: float | None = None
    if intended.alt_m is not None and impact.alt_m is not None:
        miss_vertical = float(intended.alt_m) - float(impact.alt_m)
        elev_correction = miss_vertical
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
        miss_vertical_m=miss_vertical,
        elevation_correction_m=elev_correction,
    )


def _dem_alt_from_row(row: dict[str, Any]) -> float | None:
    raw = row.get("target_alt_m_dem")
    if raw is None:
        raw = row.get("target_alt_m")
    return _float_or_none(raw)


def _default_observe_dem_path() -> str | None:
    try:
        from PySide6.QtCore import QSettings

        st = QSettings("VGCS", "VGCS")
        raw = st.value("observe/dem_path", "") or st.value("observe/dem_csv", "")
        p = str(raw or "").strip()
        return p or None
    except Exception:
        return None


def dem_alt_msl_at_mark(
    row: dict[str, Any] | None,
    pt: GeoPoint | None,
    *,
    dem_path: str | Path | None = None,
) -> float | None:
    """DEM terrain elevation (MSL) at a mark's footprint."""
    if row is not None:
        dem = _dem_alt_from_row(row)
        if dem is not None:
            return dem
    if pt is None:
        return None
    path = str(dem_path or "").strip() or _default_observe_dem_path()
    if not path:
        return None
    from vgcs.observe.dem import elevation_at_wgs84

    return elevation_at_wgs84(float(pt.lat), float(pt.lon), path)


def resolve_dooaf_mark_elevations(
    intended_row: dict[str, Any] | None,
    impact_row: dict[str, Any] | None,
    intended: GeoPoint | None,
    impact: GeoPoint | None,
    *,
    hfov_deg: float = 62.0,
) -> tuple[GeoPoint | None, GeoPoint | None, float | None]:
    """
    Apply roof+base pair height to DOOAF intended/impact MSL altitudes.

    When intended is picked on the roof and impact at the base (video), building
    height from facade geometry sets target roof MSL = base DEM + height.
    """
    from vgcs.observe.facade_plane import facade_vertical_height_between_marks

    building_h: float | None = None
    new_intended = intended
    new_impact = impact

    if (
        intended_row is None
        or impact_row is None
        or intended is None
        or impact is None
    ):
        return new_intended, new_impact, building_h

    pair_h = facade_vertical_height_between_marks(
        intended_row, impact_row, hfov_deg=hfov_deg
    )
    if pair_h is None or pair_h < 1.0:
        return new_intended, new_impact, building_h

    uy_i = intended_row.get("video_y_norm")
    uy_p = impact_row.get("video_y_norm")
    if uy_i is None or uy_p is None:
        return new_intended, new_impact, building_h

    try:
        y_intended = float(uy_i)
        y_impact = float(uy_p)
    except (TypeError, ValueError):
        return new_intended, new_impact, building_h

    if y_intended < y_impact:
        base_dem = _dem_alt_from_row(impact_row) or impact.alt_m
        if base_dem is not None:
            roof_alt = base_dem + pair_h
            new_intended = GeoPoint(intended.lat, intended.lon, roof_alt)
            new_impact = GeoPoint(impact.lat, impact.lon, base_dem)
            building_h = pair_h
            intended_row["building_height_m"] = pair_h
            intended_row["target_alt_method"] = "pair_facade_vertical"
            impact_row["target_alt_method"] = "terrain_dem"
    elif y_impact < y_intended:
        base_dem = _dem_alt_from_row(intended_row) or intended.alt_m
        if base_dem is not None:
            roof_alt = base_dem + pair_h
            new_impact = GeoPoint(impact.lat, impact.lon, roof_alt)
            new_intended = GeoPoint(intended.lat, intended.lon, base_dem)
            building_h = pair_h

    return new_intended, new_impact, building_h


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


def _drone_alt_msl_from_row(row: dict[str, Any]) -> float | None:
    """Drone position MSL — never treat EKF relative (above home) as sea level."""
    msl = _float_or_none(row.get("vehicle_alt_msl_m"))
    if msl is not None:
        return msl
    lat = _float_or_none(row.get("vehicle_lat"))
    lon = _float_or_none(row.get("vehicle_lon"))
    agl = _float_or_none(row.get("dem_ground_agl_m"))
    if lat is not None and lon is not None and agl is not None:
        dem = dem_alt_msl_at_mark(row, GeoPoint(lat, lon, None))
        if dem is not None:
            return float(dem) + float(agl)
    return None


def drone_from_row(row: dict[str, Any] | None) -> GeoPoint | None:
    if row is None:
        return None
    lat = _float_or_none(row.get("vehicle_lat"))
    lon = _float_or_none(row.get("vehicle_lon"))
    if lat is None or lon is None:
        return None
    return GeoPoint(lat, lon, _drone_alt_msl_from_row(row))


_SETUP_ROW_CONTEXT_KEYS = (
    "vehicle_lat",
    "vehicle_lon",
    "vehicle_heading_deg",
    "vehicle_roll_deg",
    "vehicle_pitch_deg",
    "ekf_rel_alt_m",
    "vehicle_rel_alt_m",
    "vehicle_alt_msl_m",
    "dem_ground_agl_m",
    "dem_ground_agl_source",
    "measure_agl_m",
    "agl_source",
    "rangefinder_down_m",
    "gimbal_yaw_deg",
    "gimbal_pitch_deg",
    "gps_fix_type",
    "gps_hdop",
    "camera_hfov_deg",
)


def _synthesize_setup_mark_row(
    role: str,
    lat: float,
    lon: float,
    video_x: float,
    video_y: float,
    template_row: dict[str, Any],
    *,
    dem_path: str | Path | None = None,
    alt_m: float | None = None,
) -> dict[str, Any]:
    """Rebuild a DOOAF Setup video pick as a mark row for elevation geometry."""
    from vgcs.observe.geo_reference import (
        compute_geo_reference,
        enrich_video_mark_target_altitude,
    )

    row: dict[str, Any] = {
        "kind": "video_mark",
        "dooaf_role": role,
        "target_lat": lat,
        "target_lon": lon,
        "video_x_norm": float(video_x),
        "video_y_norm": float(video_y),
    }
    if alt_m is not None:
        row["target_alt_m"] = alt_m
    for key in _SETUP_ROW_CONTEXT_KEYS:
        if template_row.get(key) is not None:
            row[key] = template_row[key]
    hfov = 62.0
    try:
        if template_row.get("camera_hfov_deg") is not None:
            hfov = float(template_row["camera_hfov_deg"])
    except (TypeError, ValueError):
        pass
    geo = compute_geo_reference(
        vehicle_lat=row.get("vehicle_lat"),  # type: ignore[arg-type]
        vehicle_lon=row.get("vehicle_lon"),  # type: ignore[arg-type]
        vehicle_heading_deg=row.get("vehicle_heading_deg"),  # type: ignore[arg-type]
        vehicle_roll_deg=row.get("vehicle_roll_deg"),  # type: ignore[arg-type]
        vehicle_pitch_deg=row.get("vehicle_pitch_deg"),  # type: ignore[arg-type]
        vehicle_rel_alt_m=row.get("vehicle_rel_alt_m") or row.get("ekf_rel_alt_m"),  # type: ignore[arg-type]
        vehicle_alt_msl_m=row.get("vehicle_alt_msl_m"),  # type: ignore[arg-type]
        rangefinder_down_m=row.get("rangefinder_down_m"),  # type: ignore[arg-type]
        gimbal_yaw_deg=row.get("gimbal_yaw_deg"),  # type: ignore[arg-type]
        gimbal_pitch_deg=row.get("gimbal_pitch_deg"),  # type: ignore[arg-type]
        video_x_norm=float(video_x),
        video_y_norm=float(video_y),
        gps_fix_type=int(template_row.get("gps_fix_type") or 0),
        gps_hdop=template_row.get("gps_hdop"),  # type: ignore[arg-type]
        camera_hfov_deg=hfov,
        dem_path=dem_path,
    )
    if geo.ok and geo.target_lat is not None and geo.target_lon is not None:
        row["target_lat"] = geo.target_lat
        row["target_lon"] = geo.target_lon
        if geo.target_alt_m is not None:
            row["target_alt_m"] = geo.target_alt_m
        row["geo_quality"] = geo.quality
        row["geo_method"] = geo.method
        row["geo_warning"] = geo.warning
        if geo.depression_deg is not None:
            row["geo_depression_deg"] = geo.depression_deg
        if geo.horizontal_range_m is not None:
            row["geo_range_m"] = geo.horizontal_range_m
        if geo.bearing_deg is not None:
            row["geo_bearing_deg"] = geo.bearing_deg
    enrich_video_mark_target_altitude(row)
    return row


def _fill_point_alt_m(
    pt: GeoPoint | None,
    row: dict[str, Any] | None,
    settings_alt_m: float | None,
    *,
    dem_path: str | Path | None = None,
) -> GeoPoint | None:
    """Prefer mark/settings altitude; fall back to DEM at the footprint."""
    if pt is None:
        return None
    alt = pt.alt_m
    if alt is None and settings_alt_m is not None:
        alt = float(settings_alt_m)
    if alt is None:
        alt = dem_alt_msl_at_mark(row, pt, dem_path=dem_path)
    if alt is None or alt == pt.alt_m:
        return pt
    return GeoPoint(pt.lat, pt.lon, alt)


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


@dataclass(frozen=True)
class DooafPreset:
    name: str
    settings: DooafSettings


def _preset_to_dict(preset: DooafPreset) -> dict[str, object]:
    s = preset.settings
    return {
        "name": str(preset.name),
        "gun_lat": s.gun_lat,
        "gun_lon": s.gun_lon,
        "gun_alt_m": s.gun_alt_m,
        "target_lat": s.target_lat,
        "target_lon": s.target_lon,
        "target_alt_m": s.target_alt_m,
    }


def load_dooaf_presets(st: Any) -> list[DooafPreset]:
    raw = str(st.value(_QS_PRESETS_JSON, "") or "").strip()
    if not raw:
        return []
    try:
        data = json.loads(raw)
    except (TypeError, ValueError, json.JSONDecodeError):
        return []
    if not isinstance(data, list):
        return []
    out: list[DooafPreset] = []
    for item in data:
        if not isinstance(item, dict):
            continue
        try:
            preset = DooafPreset(
                name=str(item.get("name") or "").strip(),
                settings=DooafSettings(
                    gun_lat=float(item["gun_lat"]) if item.get("gun_lat") not in (None, "") else None,
                    gun_lon=float(item["gun_lon"]) if item.get("gun_lon") not in (None, "") else None,
                    gun_alt_m=float(item["gun_alt_m"]) if item.get("gun_alt_m") not in (None, "") else None,
                    target_lat=float(item["target_lat"]) if item.get("target_lat") not in (None, "") else None,
                    target_lon=float(item["target_lon"]) if item.get("target_lon") not in (None, "") else None,
                    target_alt_m=float(item["target_alt_m"]) if item.get("target_alt_m") not in (None, "") else None,
                ),
            )
        except (TypeError, ValueError, KeyError):
            continue
        if preset.name:
            out.append(preset)
    return out


def save_dooaf_presets(st: Any, presets: list[DooafPreset]) -> None:
    payload = [_preset_to_dict(p) for p in presets]
    st.setValue(_QS_PRESETS_JSON, json.dumps(payload))


def upsert_dooaf_preset(st: Any, preset: DooafPreset) -> None:
    presets = [p for p in load_dooaf_presets(st) if p.name != preset.name]
    presets.append(preset)
    save_dooaf_presets(st, presets)


def delete_dooaf_preset(st: Any, name: str) -> None:
    want = str(name or "").strip()
    if not want:
        return
    save_dooaf_presets(st, [p for p in load_dooaf_presets(st) if p.name != want])


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


def enrich_dooaf_settings_elevation_from_dem(
    settings: DooafSettings,
    dem_path: str | None,
) -> DooafSettings:
    """Fill missing gun/target MSL altitude from DEM at each stored lat/lon."""
    path = str(dem_path or "").strip() or None
    if path is None:
        return settings
    from vgcs.observe.dem import elevation_at_wgs84

    gun_alt = settings.gun_alt_m
    tgt_alt = settings.target_alt_m
    if settings.gun_lat is not None and settings.gun_lon is not None and gun_alt is None:
        gun_alt = elevation_at_wgs84(
            float(settings.gun_lat), float(settings.gun_lon), path
        )
    if (
        settings.target_lat is not None
        and settings.target_lon is not None
        and tgt_alt is None
    ):
        tgt_alt = elevation_at_wgs84(
            float(settings.target_lat), float(settings.target_lon), path
        )
    if gun_alt == settings.gun_alt_m and tgt_alt == settings.target_alt_m:
        return settings
    return DooafSettings(
        gun_lat=settings.gun_lat,
        gun_lon=settings.gun_lon,
        gun_alt_m=gun_alt,
        target_lat=settings.target_lat,
        target_lon=settings.target_lon,
        target_alt_m=tgt_alt,
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
    dem_path: str | Path | None = None,
    setup_video_marks: dict[str, tuple[float, float]] | None = None,
) -> DooafSession:
    gun_row = latest_mark_row(rows, DOOAF_ROLE_GUN)
    intended_row = latest_mark_row(rows, DOOAF_ROLE_INTENDED)
    impact_row = latest_mark_row(rows, DOOAF_ROLE_IMPACT)
    gun = latest_mark(rows, DOOAF_ROLE_GUN) or gun_from_settings(
        gun_lat=gun_lat, gun_lon=gun_lon, gun_alt_m=gun_alt_m
    )
    gun = _fill_point_alt_m(gun, gun_row, gun_alt_m, dem_path=dem_path)
    intended = latest_mark(rows, DOOAF_ROLE_INTENDED) or point_from_latlon(
        lat=target_lat, lon=target_lon, alt_m=target_alt_m
    )
    intended = _fill_point_alt_m(intended, intended_row, target_alt_m, dem_path=dem_path)
    impact = latest_mark(rows, DOOAF_ROLE_IMPACT)
    impact = _fill_point_alt_m(impact, impact_row, None, dem_path=dem_path)
    drone = drone_from_row(rows[-1] if rows else None)
    template_row = impact_row or (rows[-1] if rows else None)
    if (
        intended_row is None
        and intended is not None
        and template_row is not None
        and setup_video_marks
        and DOOAF_ROLE_INTENDED in setup_video_marks
    ):
        vx, vy = setup_video_marks[DOOAF_ROLE_INTENDED]
        intended_row = _synthesize_setup_mark_row(
            DOOAF_ROLE_INTENDED,
            intended.lat,
            intended.lon,
            vx,
            vy,
            template_row,
            dem_path=dem_path,
            alt_m=intended.alt_m,
        )
        intended = point_from_row(intended_row) or intended
    building_height_m: float | None = None
    if intended is not None and impact is not None:
        intended, impact, building_height_m = resolve_dooaf_mark_elevations(
            intended_row,
            impact_row,
            intended,
            impact,
        )
    correction = None
    if gun is not None and intended is not None and impact is not None:
        correction = compute_fire_correction(gun, intended, impact)
    intended_dem = dem_alt_msl_at_mark(intended_row, intended, dem_path=dem_path)
    impact_dem = dem_alt_msl_at_mark(impact_row, impact, dem_path=dem_path)
    height_correction_m: float | None = None
    if intended is not None and impact is not None:
        if intended.alt_m is not None and impact.alt_m is not None:
            height_correction_m = float(intended.alt_m) - float(impact.alt_m)
    return DooafSession(
        gun=gun,
        intended=intended,
        impact=impact,
        drone=drone,
        correction=correction,
        building_height_m=building_height_m,
        intended_dem_alt_m=intended_dem,
        impact_dem_alt_m=impact_dem,
        height_correction_m=height_correction_m,
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


def _report_section_card(
    title: str,
    body: str,
    *,
    extra_class: str = "",
    section_id: str = "",
    subtitle: str = "",
) -> str:
    cls = "section-card"
    if extra_class:
        cls += f" {extra_class}"
    sid = f" id='{_html_esc(section_id)}'" if section_id else ""
    sub = (
        f"<p class='section-subtitle'>{_html_esc(subtitle)}</p>"
        if subtitle
        else ""
    )
    return (
        f"<section class='{cls}'{sid}>"
        "<div class='section-head'>"
        f"<h3 class='section-title'>{_html_esc(title)}</h3>"
        f"{sub}"
        "</div>"
        f"<div class='section-body'>{body}</div>"
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
    from vgcs.observe.observation_report_theme import REPORT_CSS

    return REPORT_CSS


def observation_report_html_script() -> str:
    from vgcs.observe.observation_report_theme import REPORT_SCRIPT

    return REPORT_SCRIPT


def _report_nav_link(section_id: str, label: str) -> str:
    return (
        f"<a href='#{_html_esc(section_id)}'>{_html_esc(label)}</a>"
    )


def format_report_nav_html(session: DooafSession | None = None) -> str:
    has_corr = session is not None and session.correction is not None
    links = [
        _report_nav_link("summary", "Summary"),
        _report_nav_link("guide", "Guide"),
    ]
    if has_corr:
        links.append(_report_nav_link("correction", "Correction"))
    links.extend(
        [
            _report_nav_link("positions", "Map"),
            _report_nav_link("glossary", "Glossary"),
            _report_nav_link("audit", "Audit"),
        ]
    )
    return (
        "<nav class='report-nav-wrap' aria-label='Report sections'>"
        f"<div class='report-nav'>{''.join(links)}</div>"
        "</nav>"
    )


def _header_kpi_html(session: DooafSession | None) -> str:
    c = session.correction if session is not None else None
    if c is None:
        return ""
    return (
        "<div class='hero-kpis'>"
        f"<div class='hero-kpi hero-kpi-miss'>"
        f"<span class='hero-kpi-val'>{c.impact_to_intended_m:.1f} m</span>"
        "<span class='hero-kpi-label'>Horizontal miss</span></div>"
        f"<div class='hero-kpi hero-kpi-range'>"
        f"<span class='hero-kpi-val'>{c.range_correction_m:+.1f} m</span>"
        "<span class='hero-kpi-label'>Range add</span></div>"
        f"<div class='hero-kpi hero-kpi-defl'>"
        f"<span class='hero-kpi-val'>{c.deflection_correction_m:+.1f} m</span>"
        "<span class='hero-kpi-label'>Deflection add</span></div>"
        "</div>"
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
    return (
        "<footer class='report-footer'>"
        "<strong>VGCS</strong> observation export · "
        "Pictures and summary are for quick decisions; CSV beside this file has full audit data."
        "</footer>"
        "<button type='button' class='back-to-top' id='back-to-top' "
        "aria-label='Back to top' title='Back to top'>↑</button>"
        f"<script>{observation_report_html_script()}</script>"
        "</div></body></html>"
    )


def format_observation_report_header(
    entry_count: int,
    *,
    title: str = "Observation Report",
    session: DooafSession | None = None,
) -> str:
    from datetime import datetime, timezone

    exported = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC")
    kpis = _header_kpi_html(session)
    return (
        "<header class='report-header'>"
        "<div class='report-header-inner'>"
        "<div class='report-header-text'>"
        "<div class='report-brand'>VGCS · Fire observation</div>"
        f"<h1>{_html_esc(title)}</h1>"
        "<div class='report-meta'>"
        f"<span class='report-meta-pill'><strong>Entries</strong> {int(entry_count)}</span>"
        f"<span class='report-meta-pill'><strong>Exported</strong> {exported}</span>"
        "<span class='report-meta-pill'><strong>Source</strong> VGCS</span>"
        "</div></div>"
        f"{kpis}"
        "</div></header>"
        + format_report_nav_html(session)
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


def _format_elev_msl_html(alt_m: float | None) -> str:
    if alt_m is None:
        return "<span class='muted'>—</span>"
    try:
        return f"<span class='elev-badge'>{float(alt_m):.1f} m MSL</span>"
    except (TypeError, ValueError):
        return "<span class='muted'>—</span>"


def _format_signed_correction_dir(
    value_m: float,
    *,
    pos_label: str,
    neg_label: str,
) -> str:
    """Format correction as e.g. ``+ Right 3.2 m`` or ``− Left 1.0 m``."""
    v = float(value_m)
    if abs(v) < 0.05:
        return "<span class='muted'>0.0 m</span>"
    if pos_label == "Right" and neg_label == "Left":
        if v > 0:
            return (
                f"<span class='lr-icon lr-pos' title='Right'>+</span> "
                f"{_html_esc(pos_label)} {v:.1f} m"
            )
        return (
            f"<span class='lr-icon lr-neg' title='Left'>−</span> "
            f"{_html_esc(neg_label)} {abs(v):.1f} m"
        )
    if v > 0:
        return f"{_html_esc(pos_label)} +{v:.1f} m"
    return f"{_html_esc(neg_label)} +{abs(v):.1f} m"


def _format_miss_dir(value_m: float, pos_label: str, neg_label: str) -> str:
    """Impact offset from target (+ = impact in positive direction)."""
    v = float(value_m)
    if abs(v) < 0.05:
        return "0.0 m"
    if v > 0:
        return f"{v:.1f} m ({pos_label})"
    return f"{abs(v):.1f} m ({neg_label})"


def _fc_svg_esc(text: str) -> str:
    return _html_esc(text).replace("'", "&#39;")


def _fc_inset_overlaps_markers(
    x0: float,
    y0: float,
    w: float,
    h: float,
    markers: list[tuple[float, float, float]],
    *,
    pad: float = 10.0,
) -> float:
    """Overlap penalty between an inset rect and diagram markers (x, y, radius)."""
    penalty = 0.0
    ex0, ey0 = x0 - pad, y0 - pad
    ex1, ey1 = x0 + w + pad, y0 + h + pad
    for mx, my, mr in markers:
        closest_x = max(ex0, min(mx, ex1))
        closest_y = max(ey0, min(my, ey1))
        dist = math.hypot(mx - closest_x, my - closest_y)
        if dist < mr:
            penalty += (mr - dist + 24.0) ** 2
    return penalty


def _fc_pick_inset_corner(
    inset_w: float,
    inset_h: float,
    plot_x: float,
    plot_y: float,
    plot_w: float,
    plot_h: float,
    markers: list[tuple[float, float, float]],
    *,
    margin: float = 6.0,
) -> tuple[float, float]:
    """Place gun/drone inset in the plot corner farthest from main markers."""
    m = margin
    options = [
        (plot_x + m, plot_y + m),
        (plot_x + plot_w - inset_w - m, plot_y + m),
        (plot_x + m, plot_y + plot_h - inset_h - m),
        (plot_x + plot_w - inset_w - m, plot_y + plot_h - inset_h - m),
    ]
    return min(
        options,
        key=lambda pos: _fc_inset_overlaps_markers(pos[0], pos[1], inset_w, inset_h, markers),
    )


def _fc_svg_gun_drone_inset(session: DooafSession, x0: float, y0: float, w: float, h: float) -> str:
    """Mini reference map: artillery + drone when coordinates are known."""
    pts: list[tuple[str, GeoPoint, str]] = []
    if session.gun is not None:
        pts.append(("G", session.gun, "#2563eb"))
    if session.drone is not None:
        pts.append(("D", session.drone, "#9333ea"))
    if len(pts) < 1:
        return ""
    ref_lat = sum(p.lat for _, p, _ in pts) / len(pts)
    ref_lon = sum(p.lon for _, p, _ in pts) / len(pts)
    en: list[tuple[str, float, float, str]] = []
    for label, pt, color in pts:
        n, e = latlon_delta_to_ne_m(ref_lat, ref_lon, pt.lat, pt.lon)
        en.append((label, e, n, color))
    span = max(max(abs(v[1]) for v in en), max(abs(v[2]) for v in en), 8.0)
    schematic_inset = len(en) == 2 and span < 8.0
    scale = min((w - 16.0) / (2.0 * span), (h - 16.0) / (2.0 * span), 3.5)
    cx = x0 + w / 2.0
    cy = y0 + h / 2.0
    parts = [
        f"<rect x='{x0:.1f}' y='{y0:.1f}' width='{w:.1f}' height='{h:.1f}' "
        "fill='#fff' fill-opacity='0.92' stroke='#cbd5e1' stroke-width='1' rx='6'/>",
        f"<text x='{cx:.1f}' y='{y0 + 10:.1f}' text-anchor='middle' font-size='7' "
        "fill='#64748b' font-weight='700'>Gun · Drone</text>",
    ]
    for label, east, north, color in en:
        if schematic_inset:
            if label == "G":
                px, py = cx - 15.0, cy + 5.0
            else:
                px, py = cx + 15.0, cy + 5.0
        else:
            px = cx + east * scale
            py = cy - north * scale
        if label == "G":
            brg = None
            if session.gun is not None and session.drone is not None:
                brg = _dooaf_bearing_deg(
                    session.gun.lat,
                    session.gun.lon,
                    session.drone.lat,
                    session.drone.lon,
                )
            parts.append(_fc_svg_marker_gun(px, py, scale=0.55, bearing_deg=brg))
            parts.append(
                f"<text x='{px:.1f}' y='{py + 14:.1f}' text-anchor='middle' font-size='6' "
                f"fill='#2563eb' font-weight='700'>Artillery</text>"
            )
        else:
            parts.append(_fc_svg_marker_drone(px, py, scale=0.55))
            parts.append(
                f"<text x='{px:.1f}' y='{py + 14:.1f}' text-anchor='middle' font-size='6' "
                f"fill='#7e22ce' font-weight='700'>Drone</text>"
            )
    return "".join(parts)


def _fc_svg_pill(cx: float, cy: float, text: str, stroke: str, fill: str, font_size: int = 10) -> str:
    """Centered label pill for diagram footers (avoids marker-cluster overlap)."""
    esc = _fc_svg_esc(text)
    w = max(len(text) * (font_size * 0.58) + 16.0, 72.0)
    h = font_size + 10.0
    return (
        f"<rect x='{cx - w / 2:.1f}' y='{cy - h / 2:.1f}' width='{w:.1f}' height='{h:.1f}' "
        f"fill='#fff' stroke='{stroke}' stroke-width='1.5' rx='8'/>"
        f"<text x='{cx:.0f}' y='{cy + font_size / 3:.1f}' text-anchor='middle' "
        f"font-size='{font_size}' fill='{fill}' font-weight='700'>{esc}</text>"
    )


def _fc_label_box_size(text: str, font_size: int) -> tuple[float, float]:
    return max(len(text) * font_size * 0.56 + 10.0, 36.0), font_size + 6.0


def _fc_label_boxes_overlap(
    ax: float,
    ay: float,
    aw: float,
    ah: float,
    bx: float,
    by: float,
    bw: float,
    bh: float,
    pad: float = 3.0,
) -> bool:
    return (
        ax - aw / 2 - pad < bx + bw / 2 + pad
        and ax + aw / 2 + pad > bx - bw / 2 - pad
        and ay - ah / 2 - pad < by + bh / 2 + pad
        and ay + ah / 2 + pad > by - bh / 2 - pad
    )


def _fc_label_hits_circle(
    lx: float,
    ly: float,
    w: float,
    h: float,
    cx: float,
    cy: float,
    r: float,
    pad: float = 5.0,
) -> bool:
    for px in (lx - w / 2, lx + w / 2):
        for py in (ly - h / 2, ly + h / 2):
            if math.hypot(px - cx, py - cy) < r + pad:
                return True
    return math.hypot(lx - cx, ly - cy) < r + pad


def _fc_svg_text_box(lx: float, ly: float, text: str, fill: str, font_size: int) -> str:
    w, h = _fc_label_box_size(text, font_size)
    esc = _fc_svg_esc(text)
    return (
        f"<rect x='{lx - w / 2:.1f}' y='{ly - h / 2:.1f}' width='{w:.1f}' height='{h:.1f}' "
        f"fill='#fff' fill-opacity='0.95' stroke='{fill}' stroke-width='1' rx='4'/>"
        f"<text x='{lx:.1f}' y='{ly + font_size / 3:.1f}' text-anchor='middle' "
        f"font-size='{font_size}' fill='{fill}' font-weight='700'>{esc}</text>"
    )


class _FcSvgLabelPlacer:
    """Place line labels with white boxes; avoid markers and other labels."""

    def __init__(self) -> None:
        self._placed: list[tuple[float, float, float, float]] = []
        self._circles: list[tuple[float, float, float]] = []

    def add_marker(self, cx: float, cy: float, r: float) -> None:
        self._circles.append((cx, cy, r))

    def add_blocked_zone(self, cx: float, cy: float, w: float, h: float) -> None:
        self._placed.append((cx, cy, w, h))

    def _blocked(self, lx: float, ly: float, w: float, h: float) -> bool:
        for cx, cy, r in self._circles:
            if _fc_label_hits_circle(lx, ly, w, h, cx, cy, r):
                return True
        for px, py, pw, ph in self._placed:
            if _fc_label_boxes_overlap(lx, ly, w, h, px, py, pw, ph):
                return True
        return False

    def place_on_segment(
        self,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        text: str,
        fill: str,
        font_size: int = 9,
        prefer_side: float = 1.0,
        min_len: float = 10.0,
    ) -> str:
        seg_len = math.hypot(x2 - x1, y2 - y1)
        if seg_len < min_len:
            return ""
        mx, my = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        w, h = _fc_label_box_size(text, font_size)
        dx, dy = x2 - x1, y2 - y1
        length = seg_len or 1.0
        px, py = -dy / length, dx / length
        candidates: list[tuple[float, float]] = []
        for mul in (prefer_side, -prefer_side, prefer_side * 2.0, -prefer_side * 2.0):
            dist = 10.0 + 7.0 * (abs(mul) - 1.0)
            sign = 1.0 if mul > 0 else -1.0
            candidates.append((mx + px * dist * sign, my + py * dist * sign))
        for lx, ly in candidates:
            if not self._blocked(lx, ly, w, h):
                self._placed.append((lx, ly, w, h))
                return _fc_svg_text_box(lx, ly, text, fill, font_size)
        return ""

    def place_at(
        self,
        lx: float,
        ly: float,
        text: str,
        fill: str,
        font_size: int = 9,
    ) -> str:
        w, h = _fc_label_box_size(text, font_size)
        for ox, oy in ((0.0, -12.0), (0.0, 12.0), (0.0, -22.0), (0.0, 22.0), (-20.0, 0.0), (20.0, 0.0)):
            tx, ty = lx + ox, ly + oy
            if not self._blocked(tx, ty, w, h):
                self._placed.append((tx, ty, w, h))
                return _fc_svg_text_box(tx, ty, text, fill, font_size)
        return ""


def _fc_svg_leader_label(
    placer: _FcSvgLabelPlacer,
    ax: float,
    ay: float,
    text: str,
    fill: str,
    font_size: int = 9,
    reach: float = 34.0,
) -> str:
    """Label offset from a point with a short leader (for tiny line segments)."""
    w, h = _fc_label_box_size(text, font_size)
    offsets = (
        (0.0, -reach),
        (-reach, 0.0),
        (reach, 0.0),
        (0.0, reach),
        (-reach * 0.75, -reach * 0.75),
        (reach * 0.75, -reach * 0.75),
        (-reach * 0.75, reach * 0.75),
        (reach * 0.75, reach * 0.75),
        (0.0, -reach * 1.5),
        (0.0, reach * 1.5),
    )
    for ox, oy in offsets:
        lx, ly = ax + ox, ay + oy
        if placer._blocked(lx, ly, w, h):
            continue
        placer._placed.append((lx, ly, w, h))
        return (
            f"<line x1='{ax:.1f}' y1='{ay:.1f}' x2='{lx:.1f}' y2='{ly:.1f}' "
            f"stroke='{fill}' stroke-width='1' stroke-dasharray='2,2'/>"
            + _fc_svg_text_box(lx, ly, text, fill, font_size)
        )
    return ""


def _fc_plan_place_label(
    placer: _FcSvgLabelPlacer,
    x1: float,
    y1: float,
    x2: float,
    y2: float,
    text: str,
    fill: str,
    font_size: int = 9,
    prefer_side: float = 1.0,
) -> str:
    """On-segment label when room allows; otherwise leader callout from midpoint."""
    lbl = placer.place_on_segment(
        x1, y1, x2, y2, text, fill, font_size, prefer_side, min_len=6.0
    )
    if lbl:
        return lbl
    mx, my = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    return _fc_svg_leader_label(placer, mx, my, text, fill, font_size)


def _fc_seg_can_label(seg_len: float, text: str, font_size: int, min_pad: float = 14.0) -> bool:
    w, _ = _fc_label_box_size(text, font_size)
    return seg_len >= w + min_pad


def _fc_diagram_footer_height(
    columns: list[tuple[str, list[str], str]],
    *,
    min_h: float = 48.0,
) -> float:
    """Footer height that fits the tallest column (header + value lines)."""
    line_step = 13.0
    value_start = 28.0
    value_font = 9.0
    bottom_pad = 8.0
    max_lines = max((len(lines) for _, lines, _ in columns), default=1)
    last_baseline = value_start + (max_lines - 1) * line_step
    return max(min_h, last_baseline + value_font + bottom_pad)


def _fc_svg_diagram_footer(
    bg_x: float,
    bg_w: float,
    footer_top: float,
    footer_h: float,
    columns: list[tuple[str, list[str], str]],
) -> str:
    """Fixed-width footer columns — headers + value lines, no overlap."""
    footer_h = max(footer_h, _fc_diagram_footer_height(columns, min_h=footer_h))
    parts = [
        f"<rect x='{bg_x:.0f}' y='{footer_top:.0f}' width='{bg_w:.0f}' height='{footer_h:.0f}' "
        "fill='#fff' stroke='#e2e8f0' stroke-width='1' rx='6'/>",
    ]
    col_n = len(columns)
    col_w = bg_w / col_n
    line_step = 13.0
    for i, (header, lines, color) in enumerate(columns):
        col_left = bg_x + col_w * i
        if i > 0:
            parts.append(
                f"<line x1='{col_left:.0f}' y1='{footer_top + 6:.0f}' x2='{col_left:.0f}' "
                f"y2='{footer_top + footer_h - 6:.0f}' stroke='#e2e8f0' stroke-width='1'/>"
            )
        hx = col_left + 10.0
        parts.append(
            f"<text x='{hx:.0f}' y='{footer_top + 14:.0f}' font-size='8' fill='#64748b' "
            f"font-weight='600'>{_fc_svg_esc(header)}</text>"
        )
        for j, line in enumerate(lines):
            parts.append(
                f"<text x='{hx:.0f}' y='{footer_top + 28.0 + j * line_step:.0f}' font-size='9' "
                f"fill='{color}' font-weight='700'>{_fc_svg_esc(line)}</text>"
            )
    return "".join(parts)


def _fc_svg_cluster_card(
    placer: _FcSvgLabelPlacer,
    anchor_x: float,
    anchor_y: float,
    rows: list[tuple[str, str]],
    bounds: tuple[float, float, float, float],
    prefer_left: bool,
    font_size: int = 9,
) -> str:
    """Miss-only label card offset from T/I cluster (never overlaps markers)."""
    if not rows:
        return ""
    line_h = font_size + 7.0
    pad_x, pad_y = 8.0, 5.0
    card_w = max(_fc_label_box_size(text, font_size)[0] for text, _ in rows) + pad_x * 2
    card_h = len(rows) * line_h + pad_y * 2
    xmin, ymin, xmax, ymax = bounds
    gap = 38.0
    if prefer_left:
        candidates = [
            (anchor_x - card_w / 2 - gap, anchor_y),
            (anchor_x - card_w / 2 - gap, anchor_y - card_h / 2 - 12.0),
            (anchor_x - card_w / 2 - gap, anchor_y + card_h / 2 + 12.0),
            (xmin + card_w / 2 + 8.0, anchor_y),
        ]
    else:
        candidates = [
            (anchor_x + card_w / 2 + gap, anchor_y),
            (anchor_x + card_w / 2 + gap, anchor_y - card_h / 2 - 12.0),
            (anchor_x + card_w / 2 + gap, anchor_y + card_h / 2 + 12.0),
            (xmax - card_w / 2 - 8.0, anchor_y),
        ]
    for cx, cy in candidates:
        if cx - card_w / 2 < xmin + 4 or cx + card_w / 2 > xmax - 4:
            continue
        if cy - card_h / 2 < ymin + 4 or cy + card_h / 2 > ymax - 4:
            continue
        if placer._blocked(cx, cy, card_w, card_h):
            continue
        placer._placed.append((cx, cy, card_w, card_h))
        left = cx - card_w / 2
        top = cy - card_h / 2
        parts = [
            f"<line x1='{anchor_x:.1f}' y1='{anchor_y:.1f}' x2='{cx:.1f}' y2='{cy:.1f}' "
            "stroke='#fdba74' stroke-width='1' stroke-dasharray='3,2'/>",
            f"<rect x='{left:.1f}' y='{top:.1f}' width='{card_w:.1f}' height='{card_h:.1f}' "
            "fill='#fff' stroke='#ea580c' stroke-width='1.2' rx='6'/>",
        ]
        for i, (text, fill) in enumerate(rows):
            ty = top + pad_y + (i + 0.72) * line_h
            parts.append(
                f"<text x='{cx:.1f}' y='{ty:.1f}' text-anchor='middle' font-size='{font_size}' "
                f"fill='{fill}' font-weight='700'>{_fc_svg_esc(text)}</text>"
            )
        return "".join(parts)
    return ""


def _fc_svg_callout_card(
    anchor_x: float,
    anchor_y: float,
    rows: list[tuple[str, str]],
    placer: _FcSvgLabelPlacer,
    bounds: tuple[float, float, float, float],
    font_size: int = 9,
) -> str:
    """Stacked callout for crowded clusters; leader line to anchor."""
    if not rows:
        return ""
    line_h = font_size + 7.0
    pad_x, pad_y = 8.0, 5.0
    card_w = max(_fc_label_box_size(text, font_size)[0] for text, _ in rows) + pad_x * 2
    card_h = len(rows) * line_h + pad_y * 2
    xmin, ymin, xmax, ymax = bounds
    gap = 32.0
    candidates: list[tuple[float, float]] = [
        (anchor_x - card_w / 2 - gap, anchor_y),
        (anchor_x + card_w / 2 + gap, anchor_y),
        (anchor_x - card_w / 2 - gap, anchor_y - card_h / 2 - gap),
        (anchor_x - card_w / 2 - gap, anchor_y + card_h / 2 + gap),
        (anchor_x + card_w / 2 + gap, anchor_y + card_h / 2 + gap),
        (anchor_x, anchor_y + card_h / 2 + gap + 8.0),
        (anchor_x, anchor_y - card_h / 2 - gap - 8.0),
        ((xmin + xmax) / 2.0, ymax - card_h / 2 - 6.0),
        (xmin + card_w / 2 + 8.0, anchor_y),
        (xmax - card_w / 2 - 8.0, anchor_y),
    ]

    for cx, cy in candidates:
        if cx - card_w / 2 < xmin + 2 or cx + card_w / 2 > xmax - 2:
            continue
        if cy - card_h / 2 < ymin + 2 or cy + card_h / 2 > ymax - 2:
            continue
        if placer._blocked(cx, cy, card_w, card_h):
            continue
        placer._placed.append((cx, cy, card_w, card_h))
        left = cx - card_w / 2
        top = cy - card_h / 2
        parts = [
            f"<line x1='{anchor_x:.1f}' y1='{anchor_y:.1f}' x2='{cx:.1f}' y2='{cy:.1f}' "
            "stroke='#94a3b8' stroke-width='1' stroke-dasharray='3,2'/>",
            f"<rect x='{left:.1f}' y='{top:.1f}' width='{card_w:.1f}' height='{card_h:.1f}' "
            "fill='#fff' stroke='#cbd5e1' stroke-width='1.2' rx='6'/>",
        ]
        for i, (text, fill) in enumerate(rows):
            ty = top + pad_y + (i + 0.72) * line_h
            parts.append(
                f"<text x='{cx:.1f}' y='{ty:.1f}' text-anchor='middle' font-size='{font_size}' "
                f"fill='{fill}' font-weight='700'>{_fc_svg_esc(text)}</text>"
            )
        return "".join(parts)
    return ""


def _fc_seg_prefer_side(
    x1: float,
    y1: float,
    x2: float,
    y2: float,
    avoid_x: float,
    avoid_y: float,
) -> float:
    """Perpendicular side (+1 / -1) that offsets the label away from a point."""
    mx, my = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    dx, dy = x2 - x1, y2 - y1
    length = math.hypot(dx, dy) or 1.0
    px, py = -dy / length, dx / length
    vx, vy = avoid_x - mx, avoid_y - my
    dot = px * vx + py * vy
    return -1.0 if dot > 0 else 1.0


def _fire_correction_workflow_html() -> str:
    return (
        "<div class='fc-workflow'>"
        "<div class='fc-workflow-step' data-step='1'>"
        "<strong>DOOAF Setup</strong>"
        "<span class='muted'>Pick on video = target</span></div>"
        "<div class='fc-workflow-step' data-step='2'>"
        "<strong>Target ON</strong>"
        "<span class='muted'>Click = Impact Target</span></div>"
        "<div class='fc-workflow-step' data-step='3'>"
        "<strong>Report</strong>"
        "<span class='muted'>Miss + correction</span></div>"
        "<div class='fc-workflow-step' data-step='4'>"
        "<strong>Course Correction</strong>"
        "<span class='muted'>Apply corrections</span></div>"
        "</div>"
    )


def _fc_axis_fit_scales(
    span_a: float,
    span_b: float,
    plot_w: float,
    plot_h: float,
    *,
    min_a: float = 12.0,
    min_b: float = 8.0,
    stretch_threshold: float = 1.1,
) -> tuple[float, float, bool]:
    """Independent axis scales when geographic aspect does not match the plot."""
    span_a = max(span_a, min_a)
    span_b = max(span_b, min_b)
    scale_a = plot_w / span_a
    scale_b = plot_h / span_b
    plot_aspect = plot_w / plot_h if plot_h > 0 else 1.0
    geo_aspect = span_a / span_b
    if geo_aspect > plot_aspect * stretch_threshold or geo_aspect < plot_aspect / stretch_threshold:
        return scale_a, scale_b, True
    uniform = min(scale_a, scale_b)
    return uniform, uniform, False


def _fc_plan_view_scales(
    span_e: float,
    span_n: float,
    plot_w: float,
    plot_h: float,
) -> tuple[float, float, bool]:
    """North-up plan scales; stretch the shorter axis when range dwarfs lateral miss."""
    return _fc_axis_fit_scales(span_e, span_n, plot_w, plot_h)


def _fire_correction_plan_svg(session: DooafSession, c: FireCorrection) -> str:
    """Plan-view SVG: gun, target, impact, miss vector (North up)."""
    gun = session.gun
    intended = session.intended
    impact = session.impact
    if gun is None or intended is None or impact is None:
        return ""
    ne_t = latlon_delta_to_ne_m(gun.lat, gun.lon, intended.lat, intended.lon)
    ne_i = latlon_delta_to_ne_m(gun.lat, gun.lon, impact.lat, impact.lon)
    miss_e = float(c.miss_east_m)
    miss_n = float(c.miss_north_m)
    corr_e = -miss_e
    corr_n = -miss_n
    range_text = f"{c.range_gun_to_intended_m:.1f} m"
    gi_text = f"{c.range_gun_to_impact_m:.1f} m"
    round_landed = [f"Miss {c.impact_to_intended_m:.1f} m"]
    if abs(miss_e) >= 0.05:
        round_landed.append(f"{abs(miss_e):.1f} m {'E' if miss_e >= 0 else 'W'}")
    if abs(miss_n) >= 0.05:
        round_landed.append(f"{abs(miss_n):.1f} m {'N' if miss_n >= 0 else 'S'}")
    corr_e_txt = f"{'East' if corr_e >= 0 else 'West'} {abs(corr_e):.1f} m"
    corr_n_txt = f"{'North' if corr_n >= 0 else 'South'} {abs(corr_n):.1f} m"
    footer_columns = [
        ("Distances", [f"gun→target {range_text}", f"gun→impact {gi_text}"], "#64748b"),
        ("Impact distance", round_landed, "#ea580c"),
        ("Course Correction", [corr_e_txt, corr_n_txt], "#0d9488"),
    ]
    east_pts = [0.0, ne_t[1], ne_i[1]]
    north_pts = [0.0, ne_t[0], ne_i[0]]
    ne_d: tuple[float, float] | None = None
    if session.drone is not None:
        ne_d = latlon_delta_to_ne_m(gun.lat, gun.lon, session.drone.lat, session.drone.lon)
    pad = 14.0
    min_e = min(east_pts) - pad
    max_e = max(east_pts) + pad
    min_n = min(north_pts) - pad
    max_n = max(north_pts) + pad
    mid_e = (min(east_pts) + max(east_pts)) / 2.0
    mid_n = (min(north_pts) + max(north_pts)) / 2.0
    span_e = max(max_e - min_e, 12.0)
    span_n = max(max_n - min_n, 12.0)
    vb_w, vb_h = 720.0, 520.0
    margin = 48.0
    graph_top = 28.0
    footer_gap = 8.0
    footer_h = _fc_diagram_footer_height(footer_columns)
    footer_top = vb_h - footer_h - 6.0
    graph_bottom = footer_top - footer_gap
    plot_bottom = graph_bottom - 20.0
    bg_x = margin - 8.0
    bg_w = vb_w - 2 * (margin - 8.0)
    plot_w = vb_w - 2 * margin
    plot_h = plot_bottom - graph_top - margin
    scale_e, scale_n, axis_stretched = _fc_plan_view_scales(span_e, span_n, plot_w, plot_h)
    cx_graph = bg_x + bg_w / 2.0
    cy_graph = (graph_top + plot_bottom) / 2.0

    def _xy(east: float, north: float) -> tuple[float, float]:
        x = cx_graph + (east - mid_e) * scale_e
        y = cy_graph - (north - mid_n) * scale_n
        return x, y

    geo_span_m = max(
        math.hypot(ne_t[0], ne_t[1]),
        math.hypot(ne_i[0], ne_i[1]),
        math.hypot(miss_e, miss_n),
        1.0,
    )
    plan_schematic = geo_span_m < 15.0
    if plan_schematic:
        tri_r = min(plot_w, plot_h) * 0.40
        gx = cx_graph - tri_r * 0.72
        gy = cy_graph
        tx, ty = cx_graph, cy_graph
        imp_pts = _fc_miss_plot_points(tx, ty, tri_r * 0.55, miss_e, miss_n)
        ix, iy = imp_pts.ix, imp_pts.iy
    else:
        gx, gy = _xy(0.0, 0.0)
        tx, ty = _xy(ne_t[1], ne_t[0])
        ix, iy = _xy(ne_i[1], ne_i[0])
    corner_x, corner_y = ix, ty
    cluster_x = (tx + ix) / 2.0
    cluster_y = (ty + iy) / 2.0
    bg_h = graph_bottom - graph_top
    fs = 9

    placer = _FcSvgLabelPlacer()
    placer.add_blocked_zone(bg_x + bg_w / 2, footer_top + footer_h / 2, bg_w + 4, footer_h + footer_gap + 4)
    placer.add_marker(gx, gy, 14.0)
    placer.add_marker(tx, ty, 15.0)
    placer.add_marker(ix, iy, 14.0)
    inset_w, inset_h = 78.0, 52.0
    inset_markers: list[tuple[float, float, float]] = [
        (gx, gy, 18.0),
        (tx, ty, 20.0),
        (ix, iy, 14.0),
    ]
    if ne_d is not None:
        dx_m, dy_m = _xy(ne_d[1], ne_d[0])
        if bg_x <= dx_m <= bg_x + bg_w and graph_top <= dy_m <= graph_bottom:
            inset_markers.append((dx_m, dy_m, 12.0))
    inset_x, inset_y = _fc_pick_inset_corner(
        inset_w, inset_h, bg_x, graph_top, bg_w, bg_h, inset_markers
    )
    placer.add_blocked_zone(inset_x + inset_w / 2, inset_y + inset_h / 2, inset_w + 12, inset_h + 12)

    parts: list[str] = [
        f"<svg class='fc-plan-svg' viewBox='0 0 {vb_w:.0f} {vb_h:.0f}' width='100%' "
        "xmlns='http://www.w3.org/2000/svg' font-family='Segoe UI,sans-serif'>",
        "<defs>",
        "<marker id='fc-arrow-miss' markerWidth='8' markerHeight='8' refX='6' refY='3' orient='auto'>",
        "<path d='M0,0 L6,3 L0,6 Z' fill='#ea580c'/>",
        "</marker>",
        "<marker id='fc-arrow-corr' markerWidth='8' markerHeight='8' refX='6' refY='3' orient='auto'>",
        "<path d='M0,0 L6,3 L0,6 Z' fill='#0d9488'/>",
        "</marker>",
        "</defs>",
        f"<text x='{cx_graph:.0f}' y='{graph_top - 8:.0f}' text-anchor='middle' font-size='11' "
        "fill='#64748b' font-weight='700'>N</text>",
        f"<line x1='{cx_graph:.0f}' y1='{graph_top - 4:.0f}' x2='{cx_graph:.0f}' "
        f"y2='{graph_top + 10:.0f}' stroke='#64748b' stroke-width='2'/>",
        f"<polygon points='{cx_graph:.0f},{graph_top - 8:.0f} {cx_graph - 5:.0f},"
        f"{graph_top - 2:.0f} {cx_graph + 5:.0f},{graph_top - 2:.0f}' fill='#64748b'/>",
        f"<rect x='{bg_x:.0f}' y='{graph_top:.0f}' width='{bg_w:.0f}' height='{bg_h:.0f}' "
        "fill='#f8fafc' rx='8'/>",
    ]
    if axis_stretched:
        parts.append(
            f"<text x='{bg_x + bg_w - 6:.0f}' y='{graph_bottom - 5:.0f}' text-anchor='end' "
            "font-size='7' fill='#94a3b8'>N/S scale exaggerated for visibility</text>"
        )
    if plan_schematic:
        parts.append(
            _fc_schematic_spacing_note(bg_x + bg_w - 6.0, graph_bottom - (16.0 if axis_stretched else 5.0))
        )
    parts.extend(
        [
        f"<line x1='{gx:.1f}' y1='{gy:.1f}' x2='{tx:.1f}' y2='{ty:.1f}' stroke='#94a3b8' "
        "stroke-width='1.5' stroke-dasharray='6,4'/>",
        f"<line x1='{gx:.1f}' y1='{gy:.1f}' x2='{ix:.1f}' y2='{iy:.1f}' stroke='#cbd5e1' "
        "stroke-width='1' stroke-dasharray='4,4'/>",
        _fc_svg_marker_gun(
            gx,
            gy,
            bearing_deg=_dooaf_bearing_deg(gun.lat, gun.lon, intended.lat, intended.lon),
        ),
        f"<text x='{gx:.1f}' y='{gy - 14:.1f}' text-anchor='middle' font-size='{fs}' fill='#2563eb' "
        "font-weight='600'>Gun</text>",
        _fc_svg_marker_crosshair(tx, ty),
        f"<circle cx='{ix:.1f}' cy='{iy:.1f}' r='10' fill='#dc2626'/>",
        f"<text x='{ix:.1f}' y='{iy + 4:.1f}' text-anchor='middle' font-size='{fs}' fill='#fff' "
        "font-weight='700'>I</text>",
        ]
    )
    if ne_d is not None:
        dx, dy = _xy(ne_d[1], ne_d[0])
        parts.append(f"<circle cx='{dx:.1f}' cy='{dy:.1f}' r='8' fill='#9333ea'/>")
        parts.append(
            f"<text x='{dx:.1f}' y='{dy + 3:.1f}' text-anchor='middle' font-size='8' fill='#fff' "
            "font-weight='700'>D</text>"
        )
    gt_side = _fc_seg_prefer_side(gx, gy, tx, ty, cluster_x, cluster_y)
    gi_side = _fc_seg_prefer_side(gx, gy, ix, iy, cluster_x, cluster_y)
    parts.append(
        placer.place_on_segment(gx, gy, tx, ty, range_text, "#64748b", fs, gt_side, min_len=56.0)
    )
    parts.append(
        placer.place_on_segment(gx, gy, ix, iy, gi_text, "#94a3b8", fs, gi_side, min_len=56.0)
    )

    if abs(miss_e) >= 0.05:
        parts.append(
            f"<line x1='{tx:.1f}' y1='{ty:.1f}' x2='{corner_x:.1f}' y2='{corner_y:.1f}' "
            "stroke='#fdba74' stroke-width='2' stroke-dasharray='4,3'/>"
        )
    if abs(miss_n) >= 0.05:
        parts.append(
            f"<line x1='{corner_x:.1f}' y1='{corner_y:.1f}' x2='{ix:.1f}' y2='{iy:.1f}' "
            "stroke='#fdba74' stroke-width='2' stroke-dasharray='4,3'/>"
        )

    parts.extend(
        [
            f"<line x1='{tx:.1f}' y1='{ty:.1f}' x2='{ix:.1f}' y2='{iy:.1f}' stroke='#ea580c' "
            "stroke-width='2.5' marker-end='url(#fc-arrow-miss)'/>",
            f"<line x1='{ix:.1f}' y1='{iy:.1f}' x2='{tx:.1f}' y2='{ty:.1f}' stroke='#0d9488' "
            "stroke-width='2' stroke-dasharray='5,3' marker-end='url(#fc-arrow-corr)'/>",
        ]
    )

    parts.append(_fc_svg_gun_drone_inset(session, inset_x, inset_y, inset_w, inset_h))

    parts.append(
        _fc_svg_diagram_footer(bg_x, bg_w, footer_top, footer_h, footer_columns)
    )
    parts.append("</svg>")
    return "".join(parts)


def _fc_miss_offset_label(value_m: float, pos_word: str, neg_word: str) -> str | None:
    v = float(value_m)
    if abs(v) < 0.05:
        return None
    return f"{abs(v):.1f} m {pos_word if v > 0 else neg_word}"


@dataclass(frozen=True)
class _FcMissPlotPoints:
    ix: float
    iy: float
    ex: float
    ey: float
    schematic: bool


def _fc_miss_plot_points(
    cx: float,
    cy: float,
    plot_r: float,
    h_m: float,
    v_m: float,
    *,
    vertical_down: bool = False,
    fill: float = 0.50,
    floor: float = 12.0,
    schematic_below_m: float = 12.0,
) -> _FcMissPlotPoints:
    """Place impact for readability: small misses use fixed diagram spacing, not true scale."""
    dist_m = math.hypot(h_m, v_m)
    if dist_m < 0.05:
        return _FcMissPlotPoints(cx, cy, cx, cy, False)
    if dist_m < schematic_below_m:
        scale = (plot_r * fill) / dist_m
        schematic = True
    else:
        span = max(abs(h_m), abs(v_m), floor)
        scale = plot_r / span
        schematic = False
    hx = h_m * scale
    vy = v_m * scale if vertical_down else -v_m * scale
    return _FcMissPlotPoints(cx + hx, cy + vy, cx + hx, cy, schematic)


def _fc_schematic_spacing_note(x: float, y: float) -> str:
    return (
        f"<text x='{x:.1f}' y='{y:.1f}' text-anchor='end' font-size='7' fill='#94a3b8'>"
        "Spacing for readability · values true</text>"
    )


def _fc_miss_vector_label_xy(
    cx: float,
    cy: float,
    ix: float,
    iy: float,
    *,
    offset: float = 16.0,
) -> tuple[float, float]:
    """Offset the miss label off the miss vector so it does not sit on component text."""
    mx, my = (cx + ix) / 2.0, (cy + iy) / 2.0
    dx, dy = ix - cx, iy - cy
    length = math.hypot(dx, dy)
    if length < 1e-6:
        return mx, my - offset
    px, py = -dy / length, dx / length
    return mx + px * offset, my + py * offset


def _fc_spread_position_markers(
    pt_xy: dict[str, tuple[float, float]],
    *,
    min_sep_px: float = 36.0,
) -> dict[str, tuple[float, float]]:
    """Separate overlapping Gun / Target / Impact icons; distances stay in the table."""
    cluster = [lbl for lbl in ("Gun", "Target", "Impact") if lbl in pt_xy]
    if len(cluster) < 2:
        return pt_xy
    pts = dict(pt_xy)
    max_sep = 0.0
    for i, a in enumerate(cluster):
        ax, ay = pts[a]
        for b in cluster[i + 1 :]:
            bx, by = pts[b]
            max_sep = max(max_sep, math.hypot(ax - bx, ay - by))
    if max_sep >= min_sep_px:
        return pts
    cx = sum(pts[lbl][0] for lbl in cluster) / len(cluster)
    cy = sum(pts[lbl][1] for lbl in cluster) / len(cluster)
    angles_deg = {"Gun": 205.0, "Target": 25.0, "Impact": 115.0}
    radius = min_sep_px * 0.92
    for lbl in cluster:
        ang = math.radians(angles_deg.get(lbl, 0.0))
        pts[lbl] = (cx + radius * math.cos(ang), cy - radius * math.sin(ang))
    return pts


def _fire_correction_gunline_svg(c: FireCorrection, *, session: DooafSession | None = None) -> str:
    """Gun-line miss view (target at centre) — same layout pattern as compass miss."""
    along = float(c.miss_along_m)
    right = float(c.miss_right_m)
    along_label = _fc_miss_offset_label(along, "Far", "Short")
    right_label = _fc_miss_offset_label(right, "Right", "Left")

    vb_w, vb_h, margin, r_tgt, r_imp = 400.0, 400.0, 52.0, 11.0, 10.0
    cx, cy = vb_w / 2.0, vb_h / 2.0
    plot_r = min(cx, cy) - margin
    pts = _fc_miss_plot_points(cx, cy, plot_r, along, right, vertical_down=True)
    ix, iy, ax, ay = pts.ix, pts.iy, pts.ex, pts.ey
    tx, ty = cx, cy
    miss_lx, miss_ly = _fc_miss_vector_label_xy(tx, ty, ix, iy)
    fs = 9

    parts: list[str] = [
        f"<svg class='fc-gunline-svg' viewBox='0 0 {vb_w:.0f} {vb_h:.0f}' width='100%' "
        "xmlns='http://www.w3.org/2000/svg' font-family='Segoe UI,sans-serif'>",
        f"<rect x='{margin:.0f}' y='{margin:.0f}' width='{vb_w - 2 * margin:.0f}' "
        f"height='{vb_h - 2 * margin:.0f}' fill='#f8fafc' rx='8'/>",
        f"<line x1='{margin:.0f}' y1='{cy:.0f}' x2='{vb_w - margin:.0f}' y2='{cy:.0f}' "
        "stroke='#cbd5e1' stroke-width='1' stroke-dasharray='4,4'/>",
        f"<line x1='{cx:.0f}' y1='{margin:.0f}' x2='{cx:.0f}' y2='{vb_h - margin:.0f}' "
        "stroke='#cbd5e1' stroke-width='1' stroke-dasharray='4,4'/>",
        f"<text x='{vb_w - margin - 4:.0f}' y='{cy + 4:.0f}' text-anchor='end' font-size='{fs}' "
        "fill='#64748b' font-weight='600'>Far</text>",
        f"<text x='{margin + 4:.0f}' y='{cy + 4:.0f}' font-size='{fs}' fill='#64748b' "
        "font-weight='600'>Short</text>",
        f"<text x='{cx:.0f}' y='{margin + 14:.0f}' text-anchor='middle' font-size='{fs + 1}' "
        "fill='#64748b' font-weight='700'>Left</text>",
        f"<text x='{cx:.0f}' y='{vb_h - margin - 6:.0f}' text-anchor='middle' font-size='{fs}' "
        "fill='#64748b' font-weight='600'>Right</text>",
        _fc_svg_marker_crosshair(tx, ty, r=r_tgt),
        f"<text x='{tx:.1f}' y='{ty - r_tgt - 6:.1f}' text-anchor='middle' font-size='{fs}' "
        "fill='#16a34a' font-weight='600'>Target</text>",
    ]
    if abs(along) >= 0.05:
        parts.append(
            f"<line x1='{tx:.1f}' y1='{ty:.1f}' x2='{ax:.1f}' y2='{ay:.1f}' stroke='#fdba74' "
            "stroke-width='2' stroke-dasharray='3,2'/>"
        )
    if abs(right) >= 0.05:
        parts.append(
            f"<line x1='{ax:.1f}' y1='{ay:.1f}' x2='{ix:.1f}' y2='{iy:.1f}' stroke='#fdba74' "
            "stroke-width='2' stroke-dasharray='3,2'/>"
        )
    if along_label:
        parts.append(
            f"<text x='{(tx + ax) / 2:.0f}' y='{cy + 16:.0f}' text-anchor='middle' "
            f"font-size='{fs}' fill='#ea580c'>{_fc_svg_esc(along_label)}</text>"
        )
    if right_label:
        parts.append(
            f"<text x='{ix + 8:.0f}' y='{(ay + iy) / 2:.0f}' font-size='{fs}' fill='#ea580c'>"
            f"{_fc_svg_esc(right_label)}</text>"
        )
    parts.extend(
        [
            f"<circle cx='{ix:.1f}' cy='{iy:.1f}' r='{r_imp:.0f}' fill='#dc2626'/>",
            f"<text x='{ix:.1f}' y='{iy + 4:.1f}' text-anchor='middle' font-size='{fs}' fill='#fff' "
            "font-weight='700'>I</text>",
            f"<text x='{ix:.1f}' y='{iy + r_imp + 14:.1f}' text-anchor='middle' font-size='{fs}' "
            "fill='#dc2626' font-weight='600'>Impact</text>",
            f"<line x1='{tx:.1f}' y1='{ty:.1f}' x2='{ix:.1f}' y2='{iy:.1f}' stroke='#ea580c' "
            "stroke-width='2.5'/>",
            f"<line x1='{ix:.1f}' y1='{iy:.1f}' x2='{tx:.1f}' y2='{ty:.1f}' stroke='#0d9488' "
            "stroke-width='2' stroke-dasharray='5,3'/>",
            f"<text x='{miss_lx:.0f}' y='{miss_ly:.0f}' text-anchor='middle' font-size='{fs + 1}' "
            f"fill='#ea580c' font-weight='700'>"
            f"{_fc_svg_esc(f'Miss {c.impact_to_intended_m:.1f} m')}</text>",
        ]
    )
    if pts.schematic:
        parts.insert(-1, _fc_schematic_spacing_note(vb_w - margin - 4.0, vb_h - margin + 2.0))
    if session is not None:
        inset_w, inset_h = 78.0, 52.0
        plot_w = vb_w - 2.0 * margin
        plot_h = vb_h - 2.0 * margin
        ix0, iy0 = _fc_pick_inset_corner(
            inset_w,
            inset_h,
            margin,
            margin,
            plot_w,
            plot_h,
            [(cx, cy, 14.0), (ix, iy, 12.0)],
        )
        parts.insert(-1, _fc_svg_gun_drone_inset(session, ix0, iy0, inset_w, inset_h))
    parts.append("</svg>")
    return "".join(parts)


def _fc_compass_span(miss_e: float, miss_n: float, *, floor: float = 10.0) -> float:
    return max(abs(miss_e), abs(miss_n), floor, 1.0)


def _fire_correction_compass_miss_svg(
    c: FireCorrection,
    *,
    compact: bool = False,
    session: DooafSession | None = None,
) -> str:
    """Target-centred compass: impact offset and E/N miss components."""
    miss_e = float(c.miss_east_m)
    miss_n = float(c.miss_north_m)
    if compact:
        vb_w, vb_h, margin, r_tgt, r_imp = 280.0, 280.0, 40.0, 9.0, 8.0
    else:
        vb_w, vb_h, margin, r_tgt, r_imp = 400.0, 400.0, 52.0, 11.0, 10.0
    cx, cy = vb_w / 2.0, vb_h / 2.0
    plot_r = min(cx, cy) - margin
    pts = _fc_miss_plot_points(cx, cy, plot_r, miss_e, miss_n)
    ix, iy, ex, ey = pts.ix, pts.iy, pts.ex, pts.ey
    miss_lx, miss_ly = _fc_miss_vector_label_xy(cx, cy, ix, iy)
    e_label = (
        f"{abs(miss_e):.1f} m {'E' if miss_e >= 0 else 'W'}"
        if abs(miss_e) >= 0.05
        else ""
    )
    n_label = (
        f"{abs(miss_n):.1f} m {'N' if miss_n >= 0 else 'S'}"
        if abs(miss_n) >= 0.05
        else ""
    )
    fs = 8 if compact else 9
    parts: list[str] = [
        f"<svg class='fc-compass-svg' viewBox='0 0 {vb_w:.0f} {vb_h:.0f}' width='100%' "
        "xmlns='http://www.w3.org/2000/svg' font-family='Segoe UI,sans-serif'>",
        f"<rect x='{margin:.0f}' y='{margin:.0f}' width='{vb_w - 2 * margin:.0f}' "
        f"height='{vb_h - 2 * margin:.0f}' fill='#f8fafc' rx='8'/>",
        f"<line x1='{cx:.0f}' y1='{margin:.0f}' x2='{cx:.0f}' y2='{vb_h - margin:.0f}' "
        "stroke='#cbd5e1' stroke-width='1' stroke-dasharray='4,4'/>",
        f"<line x1='{margin:.0f}' y1='{cy:.0f}' x2='{vb_w - margin:.0f}' y2='{cy:.0f}' "
        "stroke='#cbd5e1' stroke-width='1' stroke-dasharray='4,4'/>",
        f"<text x='{cx:.0f}' y='{margin + 14:.0f}' text-anchor='middle' font-size='{fs + 1}' "
        "fill='#64748b' font-weight='700'>N</text>",
        f"<text x='{vb_w - margin - 4:.0f}' y='{cy + 4:.0f}' text-anchor='end' font-size='{fs}' "
        "fill='#64748b' font-weight='600'>E</text>",
        f"<text x='{margin + 4:.0f}' y='{cy + 4:.0f}' font-size='{fs}' fill='#64748b' "
        "font-weight='600'>W</text>",
        f"<text x='{cx:.0f}' y='{vb_h - margin - 6:.0f}' text-anchor='middle' font-size='{fs}' "
        "fill='#64748b' font-weight='600'>S</text>",
        _fc_svg_marker_crosshair(cx, cy, r=r_tgt),
        f"<text x='{cx:.1f}' y='{cy - r_tgt - 6:.1f}' text-anchor='middle' font-size='{fs}' "
        "fill='#16a34a' font-weight='600'>Target</text>",
    ]
    if abs(miss_e) >= 0.05:
        parts.append(
            f"<line x1='{cx:.1f}' y1='{cy:.1f}' x2='{ex:.1f}' y2='{ey:.1f}' stroke='#fdba74' "
            "stroke-width='2' stroke-dasharray='3,2'/>"
        )
    if abs(miss_n) >= 0.05:
        parts.append(
            f"<line x1='{ex:.1f}' y1='{ey:.1f}' x2='{ix:.1f}' y2='{iy:.1f}' stroke='#fdba74' "
            "stroke-width='2' stroke-dasharray='3,2'/>"
        )
    if e_label:
        parts.append(
            f"<text x='{(cx + ex) / 2:.0f}' y='{cy + 16:.0f}' text-anchor='middle' "
            f"font-size='{fs}' fill='#ea580c'>{_fc_svg_esc(e_label)}</text>"
        )
    if n_label:
        parts.append(
            f"<text x='{ix + 8:.0f}' y='{(ey + iy) / 2:.0f}' font-size='{fs}' fill='#ea580c'>"
            f"{_fc_svg_esc(n_label)}</text>"
        )
    parts.extend(
        [
            f"<circle cx='{ix:.1f}' cy='{iy:.1f}' r='{r_imp:.0f}' fill='#dc2626'/>",
            f"<text x='{ix:.1f}' y='{iy + 4:.1f}' text-anchor='middle' font-size='{fs}' fill='#fff' "
            "font-weight='700'>I</text>",
            f"<text x='{ix:.1f}' y='{iy + r_imp + 14:.1f}' text-anchor='middle' font-size='{fs}' "
            "fill='#dc2626' font-weight='600'>Impact</text>",
            f"<line x1='{cx:.1f}' y1='{cy:.1f}' x2='{ix:.1f}' y2='{iy:.1f}' stroke='#ea580c' "
            "stroke-width='2.5'/>",
            f"<line x1='{ix:.1f}' y1='{iy:.1f}' x2='{cx:.1f}' y2='{cy:.1f}' stroke='#0d9488' "
            "stroke-width='2' stroke-dasharray='5,3'/>",
            f"<text x='{miss_lx:.0f}' y='{miss_ly:.0f}' text-anchor='middle' font-size='{fs + 1}' "
            f"fill='#ea580c' font-weight='700'>"
            f"{_fc_svg_esc(f'Miss {c.impact_to_intended_m:.1f} m')}</text>",
        ]
    )
    if pts.schematic:
        parts.insert(-1, _fc_schematic_spacing_note(vb_w - margin - 4.0, vb_h - margin + 2.0))
    if session is not None:
        inset_w, inset_h = 78.0, 52.0
        plot_w = vb_w - 2.0 * margin
        plot_h = vb_h - 2.0 * margin
        ix0, iy0 = _fc_pick_inset_corner(
            inset_w,
            inset_h,
            margin,
            margin,
            plot_w,
            plot_h,
            [(cx, cy, 14.0), (ix, iy, 12.0)],
        )
        parts.insert(-1, _fc_svg_gun_drone_inset(session, ix0, iy0, inset_w, inset_h))
    parts.append("</svg>")
    return "".join(parts)


def _fire_correction_aim_story_svg(
    c: FireCorrection,
    *,
    session: DooafSession | None = None,
) -> str:
    """Three-step story: aimed → landed → correct next round."""
    miss_e = float(c.miss_east_m)
    miss_n = float(c.miss_north_m)
    horiz_m = float(c.impact_to_intended_m)
    vb_w, vb_h = 720.0, 248.0
    col_w = vb_w / 3.0
    cols = [col_w * 0.5, col_w * 1.5, col_w * 2.5]
    graph_top = 50.0
    graph_bottom = 142.0
    target_y = 74.0
    label_y = 178.0
    tgt_r, imp_r = 11.0, 10.0
    half_col = col_w / 2.0 - 18.0
    avail_down = graph_bottom - target_y - imp_r - 4.0
    avail_up = target_y - graph_top - tgt_r - 4.0

    def _impact_xy(cx: float) -> tuple[float, float]:
        se = max(abs(miss_e), 0.5)
        sn = max(abs(miss_n), 0.5)
        px_e = min(half_col / se, 42.0 / _fc_compass_span(miss_e, miss_n, floor=20.0))
        if miss_n >= 0:
            px_n = min(avail_up / sn, px_e)
        else:
            px_n = min(avail_down / sn, px_e)
        px = min(px_e, px_n)
        ix = cx + miss_e * px
        iy = target_y - miss_n * px
        ix = max(cx - half_col, min(cx + half_col, ix))
        iy = max(graph_top + tgt_r, min(graph_bottom - imp_r, iy))
        return ix, iy

    def _panel_bg(idx: int) -> str:
        x0 = idx * col_w + 6.0
        return (
            f"<rect x='{x0:.0f}' y='{graph_top - 2:.0f}' width='{col_w - 12:.0f}' "
            f"height='{graph_bottom - graph_top + 36:.0f}' fill='#f8fafc' "
            "stroke='#e2e8f0' stroke-width='1' rx='10'/>"
        )

    def _story_heading(cx: float, title: str, subtitle: str) -> str:
        return (
            f"<text x='{cx:.0f}' y='24' text-anchor='middle' font-size='11' fill='#334155' "
            f"font-weight='700'>{_fc_svg_esc(title)}</text>"
            f"<text x='{cx:.0f}' y='38' text-anchor='middle' font-size='9' fill='#64748b'>"
            f"{_fc_svg_esc(subtitle)}</text>"
        )

    def _story_target(cx: float, aim_hint: bool = False) -> str:
        hint = ""
        if aim_hint:
            hint = (
                f"<text x='{cx:.0f}' y='{target_y - 16:.0f}' text-anchor='middle' font-size='8' "
                "fill='#0d9488' font-weight='700'>Aim here</text>"
            )
        return (
            hint
            + f"<circle cx='{cx:.0f}' cy='{target_y:.0f}' r='{tgt_r:.0f}' fill='#16a34a'/>"
            + f"<text x='{cx:.0f}' y='{target_y + 4:.0f}' text-anchor='middle' font-size='9' "
            "fill='#fff' font-weight='700'>T</text>"
        )

    def _story_impact(ix: float, iy: float) -> str:
        return (
            f"<circle cx='{ix:.1f}' cy='{iy:.1f}' r='{imp_r:.0f}' fill='#dc2626'/>"
            f"<text x='{ix:.1f}' y='{iy + 4:.1f}' text-anchor='middle' font-size='9' fill='#fff' "
            "font-weight='700'>I</text>"
        )

    def _story_pill(cx: float, text: str, stroke: str, fill: str) -> str:
        esc = _fc_svg_esc(text)
        w = max(len(text) * 6.0 + 18.0, 80.0)
        return (
            f"<rect x='{cx - w / 2:.1f}' y='{label_y - 14:.0f}' width='{w:.1f}' height='20' "
            f"fill='#fff' stroke='{stroke}' stroke-width='1.5' rx='8'/>"
            f"<text x='{cx:.0f}' y='{label_y:.0f}' text-anchor='middle' font-size='10' "
            f"fill='{fill}' font-weight='700'>{esc}</text>"
        )

    # Step 1
    panel1 = _panel_bg(0) + _story_heading(cols[0], "1 · You aimed at", "Actual target (green)") + _story_target(
        cols[0]
    )

    # Step 2 — orange miss
    cx2 = cols[1]
    ix2, iy2 = _impact_xy(cx2)
    panel2 = (
        _panel_bg(1)
        + _story_heading(cx2, "2 · Impact distance", "Impact Target (red)")
        + _story_target(cx2)
        + f"<line x1='{cx2:.0f}' y1='{target_y:.0f}' x2='{ix2:.1f}' y2='{iy2:.1f}' "
        "stroke='#ea580c' stroke-width='2.5'/>"
        + _story_impact(ix2, iy2)
        + _story_pill(cx2, f"Miss {horiz_m:.1f} m", "#fdba74", "#ea580c")
    )

    # Step 3 — teal correction (no orange line)
    cx3 = cols[2]
    ix3, iy3 = _impact_xy(cx3)
    panel3 = (
        _panel_bg(2)
        + _story_heading(cx3, "3 · Course Correction", "Move aim back to target")
        + _story_target(cx3, aim_hint=True)
        + _story_impact(ix3, iy3)
        + f"<line x1='{ix3:.1f}' y1='{iy3:.1f}' x2='{cx3:.0f}' y2='{target_y:.0f}' "
        "stroke='#0d9488' stroke-width='2.5' stroke-dasharray='6,4' "
        "marker-end='url(#fc-story-arrow)'/>"
        + _story_pill(cx3, "Apply correction", "#5eead4", "#0d9488")
    )

    dividers = (
        f"<line x1='{col_w:.0f}' y1='{graph_top:.0f}' x2='{col_w:.0f}' y2='{label_y + 14:.0f}' "
        "stroke='#e2e8f0' stroke-width='1'/>"
        f"<line x1='{col_w * 2:.0f}' y1='{graph_top:.0f}' x2='{col_w * 2:.0f}' "
        f"y2='{label_y + 14:.0f}' stroke='#e2e8f0' stroke-width='1'/>"
    )

    return "".join(
        [
            f"<svg class='fc-story-svg' viewBox='0 0 {vb_w:.0f} {vb_h:.0f}' width='100%' "
            "xmlns='http://www.w3.org/2000/svg' font-family='Segoe UI,sans-serif'>",
            "<defs>",
            "<marker id='fc-story-arrow' markerWidth='8' markerHeight='8' refX='6' refY='3' orient='auto'>",
            "<path d='M0,0 L6,3 L0,6 Z' fill='#0d9488'/>",
            "</marker>",
            "</defs>",
            dividers,
            panel1,
            panel2,
            panel3,
            "<text x='360' y='222' text-anchor='middle' font-size='9' fill='#64748b'>",
            f"{_fc_svg_esc('Step 2 = orange miss distance · Step 3 = teal correction arrow')}</text>",
            "</svg>",
        ]
    )


def _fire_correction_positions_svg(session: DooafSession) -> str:
    """All marks on one map: gun, target, impact, drone."""
    entries: list[tuple[str, GeoPoint, str, str]] = []
    if session.gun is not None:
        entries.append(("Gun", session.gun, "#2563eb", "G"))
    if session.intended is not None:
        entries.append(("Target", session.intended, "#16a34a", "T"))
    if session.impact is not None:
        entries.append(("Impact", session.impact, "#dc2626", "I"))
    if session.drone is not None:
        entries.append(("Drone", session.drone, "#9333ea", "D"))
    if len(entries) < 2:
        return ""
    ref_lat = sum(p.lat for _, p, _, _ in entries) / len(entries)
    ref_lon = sum(p.lon for _, p, _, _ in entries) / len(entries)
    en_pts: list[tuple[str, float, float, str, str]] = []
    for label, pt, color, letter in entries:
        north, east = latlon_delta_to_ne_m(ref_lat, ref_lon, pt.lat, pt.lon)
        en_pts.append((label, east, north, color, letter))
    east_vals = [e for _, e, _, _, _ in en_pts]
    north_vals = [n for _, _, n, _, _ in en_pts]
    mid_e = (min(east_vals) + max(east_vals)) / 2.0
    mid_n = (min(north_vals) + max(north_vals)) / 2.0
    span_e = max(max(east_vals) - min(east_vals), 20.0)
    span_n = max(max(north_vals) - min(north_vals), 20.0)
    c = session.correction
    range_gt = float(c.range_gun_to_intended_m) if c else 0.0
    range_gi = float(c.range_gun_to_impact_m) if c else 0.0
    miss_ti = float(c.impact_to_intended_m) if c else 0.0
    drone_dist = ""
    if session.drone is not None and session.intended is not None:
        d_n, d_e = latlon_delta_to_ne_m(
            session.intended.lat,
            session.intended.lon,
            session.drone.lat,
            session.drone.lon,
        )
        drone_dist = f"{math.hypot(d_n, d_e):.1f} m"
    dist_lines = [f"gun→target {range_gt:.1f} m", f"gun→impact {range_gi:.1f} m"]
    if drone_dist:
        dist_lines.append(f"target→drone {drone_dist}")
    miss_lines = [f"target→impact {miss_ti:.1f} m"] if miss_ti > 0 else ["—"]
    footer_columns = [
        ("Marks", ["G Gun · T Target", "I Impact · D Drone"], "#64748b"),
        ("Distances", dist_lines, "#64748b"),
        ("Miss", miss_lines, "#ea580c"),
    ]
    vb_w, vb_h = 640.0, 460.0
    margin = 44.0
    graph_top = 28.0
    footer_gap = 8.0
    footer_h = _fc_diagram_footer_height(footer_columns)
    footer_top = vb_h - footer_h - 6.0
    graph_bottom = footer_top - footer_gap
    bg_x = margin - 8.0
    bg_w = vb_w - 2 * (margin - 8.0)
    graph_h = graph_bottom - graph_top
    fs = 9
    scale = min((vb_w - 2 * margin) / span_e, graph_h / span_n)
    cx_graph = bg_x + bg_w / 2.0
    cy_graph = graph_top + graph_h / 2.0

    def _xy(east: float, north: float) -> tuple[float, float]:
        x = cx_graph + (east - mid_e) * scale
        y = cy_graph - (north - mid_n) * scale
        return x, y

    pt_xy: dict[str, tuple[float, float]] = {}
    for label, east, north, color, letter in en_pts:
        pt_xy[label] = _xy(east, north)
    pt_xy = _fc_spread_position_markers(pt_xy)

    placer = _FcSvgLabelPlacer()
    placer.add_blocked_zone(bg_x + bg_w / 2, footer_top + footer_h / 2, bg_w + 4, footer_h + footer_gap + 4)
    for xy in pt_xy.values():
        placer.add_marker(xy[0], xy[1], 13.0)

    parts: list[str] = [
        f"<svg class='fc-positions-svg' viewBox='0 0 {vb_w:.0f} {vb_h:.0f}' width='100%' "
        "xmlns='http://www.w3.org/2000/svg' font-family='Segoe UI,sans-serif'>",
        f"<rect x='{bg_x:.0f}' y='{graph_top:.0f}' width='{bg_w:.0f}' height='{graph_h:.0f}' "
        "fill='#f8fafc' rx='8'/>",
        f"<text x='{cx_graph:.0f}' y='{graph_top - 8:.0f}' text-anchor='middle' font-size='10' "
        "fill='#64748b' font-weight='700'>N</text>",
        f"<line x1='{cx_graph:.0f}' y1='{graph_top - 4:.0f}' x2='{cx_graph:.0f}' "
        f"y2='{graph_top + 10:.0f}' stroke='#64748b' stroke-width='2'/>",
        f"<polygon points='{cx_graph:.0f},{graph_top - 8:.0f} {cx_graph - 5:.0f},"
        f"{graph_top - 2:.0f} {cx_graph + 5:.0f},{graph_top - 2:.0f}' fill='#64748b'/>",
    ]

    if "Gun" in pt_xy and "Target" in pt_xy:
        gx, gy = pt_xy["Gun"]
        tx, ty = pt_xy["Target"]
        parts.append(
            f"<line x1='{gx:.1f}' y1='{gy:.1f}' x2='{tx:.1f}' y2='{ty:.1f}' stroke='#94a3b8' "
            "stroke-width='1.5' stroke-dasharray='6,4'/>"
        )
        if range_gt > 0:
            gt_side = _fc_seg_prefer_side(gx, gy, tx, ty, cx_graph, cy_graph)
            lbl = placer.place_on_segment(
                gx, gy, tx, ty, f"{range_gt:.1f} m", "#64748b", fs, gt_side, min_len=40.0
            )
            if lbl:
                parts.append(lbl)

    if "Gun" in pt_xy and "Impact" in pt_xy:
        gx, gy = pt_xy["Gun"]
        ix, iy = pt_xy["Impact"]
        parts.append(
            f"<line x1='{gx:.1f}' y1='{gy:.1f}' x2='{ix:.1f}' y2='{iy:.1f}' stroke='#cbd5e1' "
            "stroke-width='1' stroke-dasharray='4,4'/>"
        )
        if range_gi > 0:
            gi_side = _fc_seg_prefer_side(gx, gy, ix, iy, cx_graph, cy_graph)
            lbl = placer.place_on_segment(
                gx, gy, ix, iy, f"{range_gi:.1f} m", "#94a3b8", fs, gi_side, min_len=40.0
            )
            if lbl:
                parts.append(lbl)

    if "Target" in pt_xy and "Impact" in pt_xy:
        tx, ty = pt_xy["Target"]
        ix, iy = pt_xy["Impact"]
        parts.append(
            f"<line x1='{tx:.1f}' y1='{ty:.1f}' x2='{ix:.1f}' y2='{iy:.1f}' stroke='#ea580c' "
            "stroke-width='2.5'/>"
        )

    if "Target" in pt_xy and "Drone" in pt_xy:
        tx, ty = pt_xy["Target"]
        dx, dy = pt_xy["Drone"]
        parts.append(
            f"<line x1='{tx:.1f}' y1='{ty:.1f}' x2='{dx:.1f}' y2='{dy:.1f}' stroke='#c4b5fd' "
            "stroke-width='1.5' stroke-dasharray='4,3'/>"
        )

    gun_brg: float | None = None
    if session.gun is not None and session.intended is not None:
        gun_brg = _dooaf_bearing_deg(
            session.gun.lat,
            session.gun.lon,
            session.intended.lat,
            session.intended.lon,
        )

    for label, east, north, color, letter in en_pts:
        x, y = pt_xy[label]
        if label == "Gun":
            parts.append(_fc_svg_marker_gun(x, y, bearing_deg=gun_brg))
        elif label == "Target":
            parts.append(_fc_svg_marker_crosshair(x, y, r=10.0))
        else:
            parts.append(f"<circle cx='{x:.1f}' cy='{y:.1f}' r='10' fill='{color}'/>")
            parts.append(
                f"<text x='{x:.1f}' y='{y + 4:.1f}' text-anchor='middle' font-size='9' fill='#fff' "
                f"font-weight='700'>{letter}</text>"
            )
            continue
        parts.append(
            f"<text x='{x:.1f}' y='{y - 14:.1f}' text-anchor='middle' font-size='8' fill='{color}' "
            f"font-weight='600'>{label}</text>"
        )

    parts.append(
        _fc_svg_diagram_footer(bg_x, bg_w, footer_top, footer_h, footer_columns)
    )
    parts.append("</svg>")
    return "".join(parts)


def _fc_bar_row_html(
    label: str,
    value_m: float,
    pos_word: str,
    neg_word: str,
    max_scale: float,
    *,
    kind: str = "miss",
) -> str:
    v = float(value_m)
    if abs(v) < 0.05:
        return (
            f"<div class='fc-bar-row'><span class='fc-bar-label'>{_html_esc(label)}</span>"
            "<div class='fc-bar-track'></div>"
            "<span class='fc-bar-value muted'>0.0 m</span></div>"
        )
    pct = min(abs(v) / max_scale * 50.0, 49.5)
    if v >= 0:
        style = f"left:50%;width:{pct:.1f}%"
        dir_text = f"{pos_word} {abs(v):.1f} m"
    else:
        style = f"left:calc(50% - {pct:.1f}%);width:{pct:.1f}%"
        dir_text = f"{neg_word} {abs(v):.1f} m"
    return (
        f"<div class='fc-bar-row'><span class='fc-bar-label'>{_html_esc(label)}</span>"
        f"<div class='fc-bar-track'><div class='fc-bar-fill fc-bar-{kind}' style='{style}'></div></div>"
        f"<span class='fc-bar-value'>{_html_esc(dir_text)}</span></div>"
    )


def _fire_correction_bars_html(c: FireCorrection) -> str:
    miss_scale = max(
        abs(float(c.miss_east_m)),
        abs(float(c.miss_north_m)),
        abs(float(c.miss_along_m)),
        abs(float(c.miss_right_m)),
        5.0,
    )
    corr_e = -float(c.miss_east_m)
    corr_n = -float(c.miss_north_m)
    corr_scale = max(
        abs(corr_e),
        abs(corr_n),
        abs(float(c.deflection_correction_m)),
        abs(float(c.range_correction_m)),
        5.0,
    )
    miss_rows = (
        _fc_bar_row_html("East / West miss", c.miss_east_m, "East", "West", miss_scale, kind="miss")
        + _fc_bar_row_html(
            "North / South miss", c.miss_north_m, "North", "South", miss_scale, kind="miss"
        )
        + _fc_bar_row_html(
            "Along gun line", c.miss_along_m, "Far", "Short", miss_scale, kind="miss"
        )
        + _fc_bar_row_html(
            "Left / Right", c.miss_right_m, "Right", "Left", miss_scale, kind="miss"
        )
    )
    corr_rows = (
        _fc_bar_row_html("East / West add", corr_e, "East", "West", corr_scale, kind="corr")
        + _fc_bar_row_html("North / South add", corr_n, "North", "South", corr_scale, kind="corr")
        + _fc_bar_row_html(
            "Range add",
            float(c.range_correction_m),
            "Add",
            "Drop",
            corr_scale,
            kind="corr",
        )
        + _fc_bar_row_html(
            "Left / Right add",
            float(c.deflection_correction_m),
            "Right",
            "Left",
            corr_scale,
            kind="corr",
        )
    )
    return (
        "<div class='fc-bars-panel'>"
        "<div class='fc-bars-title'>Miss & correction bars (centre = zero)</div>"
        "<p class='log-hint' style='margin:0 0 10px'>Bar length shows distance; "
        "left/right or north/south label shows direction.</p>"
        + miss_rows
        + "<div class='fc-bars-title' style='margin-top:14px'>Corrections to add</div>"
        + corr_rows
        + "</div>"
    )


def _exec_panel_header_html(
    step: int,
    kind: str,
    title: str,
    subtitle: str,
    icon: str,
) -> str:
    return (
        f"<div class='exec-panel-header exec-panel-header-{kind}'>"
        f"<span class='exec-panel-step'>{step}</span>"
        f"<span class='exec-panel-icon' aria-hidden='true'>{icon}</span>"
        "<div class='exec-panel-text'>"
        f"<span class='exec-panel-title'>{_html_esc(title)}</span>"
        f"<span class='exec-panel-sub'>{_html_esc(subtitle)}</span>"
        "</div></div>"
    )


def _executive_miss_chip(value_m: float, pos_word: str, neg_word: str) -> str | None:
    v = float(value_m)
    if abs(v) < 0.5:
        return None
    if v > 0:
        icon = "<span class='lr-icon lr-pos' title='Positive'>+</span> "
        word = pos_word
    else:
        icon = "<span class='lr-icon lr-neg' title='Negative'>−</span> "
        word = neg_word
    return f"<li>{icon}<strong>{abs(v):.1f}</strong> m {word}</li>"


def _executive_miss_chips_html(c: FireCorrection) -> str:
    """Small chips: where impact landed relative to target."""
    chips: list[str] = []
    for part in (
        _executive_miss_chip(c.miss_north_m, "north", "south"),
        _executive_miss_chip(c.miss_east_m, "east", "west"),
        _executive_miss_chip(c.miss_along_m, "too far", "short"),
    ):
        if part:
            chips.append(part)
    if not chips:
        return ""
    return "<ul class='exec-miss-list'>" + "".join(chips) + "</ul>"


def _fire_correction_action_cards_html(
    c: FireCorrection,
    *,
    for_executive: bool = False,
) -> str:
    corr_e = -float(c.miss_east_m)
    corr_n = -float(c.miss_north_m)
    items: list[tuple[str, float, str, str, str, str, str]] = [
        ("East / West", corr_e, "East", "West", "→", "←", "fc-action-card-map"),
        ("North / South", corr_n, "North", "South", "↑", "↓", "fc-action-card-map"),
        (
            "Range",
            float(c.range_correction_m),
            "Add",
            "Drop",
            "⟶",
            "⟵",
            "fc-action-card-range",
        ),
        (
            "Deflection (R+)",
            float(c.deflection_correction_m),
            "Right",
            "Left",
            "↷",
            "↶",
            "fc-action-card-defl",
        ),
    ]
    if c.elevation_correction_m is not None:
        u = float(c.elevation_correction_m)
        if abs(u) >= 0.5:
            items.append(
                ("Elevation", u, "Up", "Down", "⬆", "⬇", "fc-action-card-elev")
            )
    cards: list[str] = []
    for label, val, pos, neg, ap, an, extra_cls in items:
        if abs(val) < 0.5:
            continue
        word = pos if val > 0 else neg
        arrow = ap if val > 0 else an
        cls = f"fc-action-card {extra_cls}".strip()
        if for_executive:
            value_html = (
                "<div class='fc-action-value'>"
                f"<span class='fc-action-dir'>{_html_esc(word)}</span>"
                f"<span class='fc-action-num'>{abs(val):.1f}</span>"
                "<span class='fc-action-unit'>m</span></div>"
            )
            badge = "<span class='fc-action-badge'>Correction</span>"
            sub = "<div class='fc-action-sub'>add to firing data</div>"
        else:
            badge = ""
            value_html = (
                f"<div class='fc-action-value'>"
                f"<span class='fc-action-dir'>{_html_esc(word)}</span> "
                f"<span class='fc-action-num'>{abs(val):.1f}</span>"
                "<span class='fc-action-unit'>m</span></div>"
            )
            sub = "<div class='fc-action-sub'>add on next round</div>"
        cards.append(
            f"<div class='{cls}'>"
            + badge
            + f"<div class='fc-action-arrow'>{arrow}</div>"
            f"<div class='fc-action-label'>{_html_esc(label)}</div>"
            + value_html
            + sub
            + "</div>"
        )
    if not cards:
        cards.append(
            "<div class='fc-action-card'>"
            "<div class='fc-action-value' style='font-size:14px'>On target</div>"
            "<div class='fc-action-sub'>No significant correction</div>"
            "</div>"
        )
    cards_html = f"<div class='fc-action-cards{' exec-corr-cards' if for_executive else ''}'>"
    cards_html += "".join(cards) + "</div>"
    if not for_executive:
        return (
            "<div class='fc-action-cards-title'>What to add — at a glance</div>"
            + cards_html
        )
    return (
        "<div class='exec-corr-panel'>"
        + _exec_panel_header_html(
            2,
            "corr",
            "What to add on the next round",
            "Fire correction values — add to your data",
            "+",
        )
        + "<div class='exec-panel-body'>"
        + cards_html
        + "<p class='exec-legend-note'>"
        + "<span class='legend-miss'>Orange on the map</span> = fall of shot. "
        + "<span class='legend-corr'>Teal cards</span> = add on the next round "
        + "(landed south → add north).</p>"
        + "</div></div>"
    )


def _fc_svg_signed_miss_label(
    x: float,
    y: float,
    value_m: float,
    pos_dir: str,
    neg_dir: str,
    *,
    fs: int = 10,
    anchor: str = "middle",
) -> str:
    """Orange miss label with green + / red − prefix (E/W/N/S)."""
    v = float(value_m)
    if abs(v) < 0.05:
        return ""
    sign = "+" if v > 0 else "−"
    sign_color = "#15803d" if v > 0 else "#b91c1c"
    direction = pos_dir if v > 0 else neg_dir
    return (
        f"<text x='{x:.1f}' y='{y:.1f}' text-anchor='{anchor}' font-size='{fs}' "
        f"fill='#ea580c' font-weight='700'>"
        f"<tspan fill='{sign_color}' font-weight='800'>{_fc_svg_esc(sign)}</tspan>"
        f"<tspan> {_fc_svg_esc(f'{abs(v):.1f} m {direction}')}</tspan></text>"
    )


def _fc_svg_edge_role_marker(
    cx: float,
    cy: float,
    plot_r: float,
    east_m: float,
    north_m: float,
    role: str,
    *,
    session: DooafSession,
    intended: GeoPoint,
) -> str:
    """Place gun or drone on the map rim toward its bearing from target."""
    dist = math.hypot(east_m, north_m)
    if dist < 1e-6:
        return ""
    ux = east_m / dist
    uy = north_m / dist
    px = cx + ux * (plot_r - 18.0)
    py = cy - uy * (plot_r - 18.0)
    label_y = py + (16.0 if role == "gun" else 15.0)
    if role == "gun":
        brg = None
        if session.gun is not None:
            brg = _dooaf_bearing_deg(
                session.gun.lat,
                session.gun.lon,
                intended.lat,
                intended.lon,
            )
        marker = _fc_svg_marker_gun(px, py, scale=0.46, bearing_deg=brg)
        label = "Artillery"
        color = "#2563eb"
    else:
        marker = _fc_svg_marker_drone(px, py, scale=0.5)
        label = "Drone"
        color = "#7e22ce"
    return (
        marker
        + f"<text x='{px:.1f}' y='{label_y:.1f}' text-anchor='middle' font-size='8' "
        f"fill='{color}' font-weight='700'>{label}</text>"
    )


def _executive_miss_map_svg(session: DooafSession, c: FireCorrection) -> str:
    """Summary map: target-centred miss, +/- labels, artillery & drone on rim."""
    miss_e = float(c.miss_east_m)
    miss_n = float(c.miss_north_m)
    vb_w, vb_h = 360.0, 360.0
    margin = 42.0
    r_tgt, r_imp = 10.0, 9.0
    fs = 10
    cx, cy = vb_w / 2.0, vb_h / 2.0
    plot_r = min(cx, cy) - margin
    pts = _fc_miss_plot_points(cx, cy, plot_r, miss_e, miss_n)
    ix, iy, ex, ey = pts.ix, pts.iy, pts.ex, pts.ey
    miss_lx, miss_ly = _fc_miss_vector_label_xy(cx, cy, ix, iy)
    intended = session.intended

    parts: list[str] = [
        f"<svg class='fc-compass-svg exec-miss-map-svg' viewBox='0 0 {vb_w:.0f} {vb_h:.0f}' "
        "width='100%' xmlns='http://www.w3.org/2000/svg' font-family='Segoe UI,sans-serif'>",
        f"<rect x='{margin:.0f}' y='{margin:.0f}' width='{vb_w - 2 * margin:.0f}' "
        f"height='{vb_h - 2 * margin:.0f}' fill='#f8fafc' rx='8'/>",
        f"<line x1='{cx:.0f}' y1='{margin:.0f}' x2='{cx:.0f}' y2='{vb_h - margin:.0f}' "
        "stroke='#cbd5e1' stroke-width='1' stroke-dasharray='4,4'/>",
        f"<line x1='{margin:.0f}' y1='{cy:.0f}' x2='{vb_w - margin:.0f}' y2='{cy:.0f}' "
        "stroke='#cbd5e1' stroke-width='1' stroke-dasharray='4,4'/>",
        f"<text x='{cx:.0f}' y='{margin + 14:.0f}' text-anchor='middle' font-size='{fs}' "
        "fill='#64748b' font-weight='700'>N</text>",
        f"<text x='{vb_w - margin - 4:.0f}' y='{cy + 4:.0f}' text-anchor='end' font-size='{fs}' "
        "fill='#64748b' font-weight='600'>E</text>",
        f"<text x='{margin + 4:.0f}' y='{cy + 4:.0f}' font-size='{fs}' fill='#64748b' "
        "font-weight='600'>W</text>",
        f"<text x='{cx:.0f}' y='{vb_h - margin - 6:.0f}' text-anchor='middle' font-size='{fs}' "
        "fill='#64748b' font-weight='600'>S</text>",
    ]
    if intended is not None:
        if session.gun is not None:
            g_n, g_e = latlon_delta_to_ne_m(
                intended.lat, intended.lon, session.gun.lat, session.gun.lon
            )
            parts.append(
                _fc_svg_edge_role_marker(
                    cx, cy, plot_r, g_e, g_n, "gun", session=session, intended=intended
                )
            )
        if session.drone is not None:
            d_n, d_e = latlon_delta_to_ne_m(
                intended.lat, intended.lon, session.drone.lat, session.drone.lon
            )
            parts.append(
                _fc_svg_edge_role_marker(
                    cx, cy, plot_r, d_e, d_n, "drone", session=session, intended=intended
                )
            )
    parts.append(_fc_svg_marker_crosshair(cx, cy, r=r_tgt))
    parts.append(
        f"<text x='{cx:.1f}' y='{cy - r_tgt - 7:.1f}' text-anchor='middle' font-size='{fs}' "
        "fill='#16a34a' font-weight='700'>Target</text>"
    )
    if abs(miss_e) >= 0.05:
        parts.append(
            f"<line x1='{cx:.1f}' y1='{cy:.1f}' x2='{ex:.1f}' y2='{ey:.1f}' stroke='#fdba74' "
            "stroke-width='2' stroke-dasharray='3,2'/>"
        )
    if abs(miss_n) >= 0.05:
        parts.append(
            f"<line x1='{ex:.1f}' y1='{ey:.1f}' x2='{ix:.1f}' y2='{iy:.1f}' stroke='#fdba74' "
            "stroke-width='2' stroke-dasharray='3,2'/>"
        )
    if abs(miss_e) >= 0.05:
        parts.append(_fc_svg_signed_miss_label((cx + ex) / 2.0, cy + 18.0, miss_e, "E", "W", fs=fs))
    if abs(miss_n) >= 0.05:
        parts.append(_fc_svg_signed_miss_label(ix + 10.0, (ey + iy) / 2.0, miss_n, "N", "S", fs=fs, anchor="start"))
    parts.extend(
        [
            f"<circle cx='{ix:.1f}' cy='{iy:.1f}' r='{r_imp:.0f}' fill='#dc2626'/>",
            f"<text x='{ix:.1f}' y='{iy + 3.5:.1f}' text-anchor='middle' font-size='{fs - 1}' "
            "fill='#fff' font-weight='700'>I</text>",
            f"<text x='{ix:.1f}' y='{iy + r_imp + 14:.1f}' text-anchor='middle' font-size='{fs}' "
            "fill='#dc2626' font-weight='700'>Impact</text>",
            f"<line x1='{cx:.1f}' y1='{cy:.1f}' x2='{ix:.1f}' y2='{iy:.1f}' stroke='#ea580c' "
            "stroke-width='2.5'/>",
            f"<line x1='{ix:.1f}' y1='{iy:.1f}' x2='{cx:.1f}' y2='{cy:.1f}' stroke='#0d9488' "
            "stroke-width='2' stroke-dasharray='5,3'/>",
            f"<text x='{miss_lx:.0f}' y='{miss_ly:.0f}' text-anchor='middle' font-size='{fs + 1}' "
            f"fill='#ea580c' font-weight='800'>"
            f"{_fc_svg_esc(f'Miss {c.impact_to_intended_m:.1f} m')}</text>",
            f"<text x='{margin + 4:.0f}' y='{vb_h - 10:.0f}' font-size='8' fill='#64748b'>"
            "<tspan fill='#15803d' font-weight='800'>+</tspan><tspan> / </tspan>"
            "<tspan fill='#b91c1c' font-weight='800'>−</tspan><tspan> = direction</tspan></text>",
        ]
    )
    if pts.schematic:
        parts.insert(-2, _fc_schematic_spacing_note(vb_w - margin - 4.0, vb_h - margin + 2.0))
    parts.append("</svg>")
    return "".join(parts)


def _executive_summary_visual_html(session: DooafSession) -> str:
    c = session.correction
    if c is None:
        return ""
    mini_map = _executive_miss_map_svg(session, c)
    miss_panel = (
        "<div class='exec-miss-panel'>"
        + _exec_panel_header_html(
            1,
            "miss",
            "Where the round landed",
            "Impact Target vs aim point on the map",
            "!",
        )
        + "<div class='exec-panel-body'>"
        + f"<div class='exec-compass-wrap'>{mini_map}</div>"
        + "<p class='exec-visual-caption'>"
        "<span class='fc-dot fc-dot-target'></span> Target · "
        "<span class='fc-dot fc-dot-impact'></span> Impact · "
        "<span class='fc-dot fc-dot-gun'></span> Artillery · "
        "<span class='fc-dot fc-dot-drone'></span> Drone · "
        "<span class='lr-icon lr-pos'>+</span>/<span class='lr-icon lr-neg'>−</span> direction"
        "</p>"
        + _executive_miss_chips_html(c)
        + "</div></div>"
    )
    bridge = (
        "<div class='exec-split-bridge' aria-hidden='true'>"
        "<span class='exec-bridge-arrow'>→</span>"
        "<span class='exec-bridge-text'>Apply opposite direction</span>"
        "</div>"
    )
    return (
        "<div class='exec-split'>"
        + miss_panel
        + bridge
        + _fire_correction_action_cards_html(c, for_executive=True)
        + "</div>"
    )


def format_fire_correction_diagram_html(session: DooafSession) -> str:
    """Visual-first fire correction: story, maps, compass, bars, action cards."""
    c = session.correction
    if c is None or session.gun is None or session.intended is None or session.impact is None:
        return ""
    plan = _fire_correction_plan_svg(session, c)
    gunline = _fire_correction_gunline_svg(c, session=session)
    compass = _fire_correction_compass_miss_svg(c, session=session)
    story = _fire_correction_aim_story_svg(c, session=session)
    bars = _fire_correction_bars_html(c)
    if not plan:
        return ""
    horiz_check = math.hypot(float(c.miss_east_m), float(c.miss_north_m))
    foot = (
        f"<p class='log-hint' style='margin-top:8px'>"
        f"Horizontal check: √(E²+N²) = <strong>{horiz_check:.1f} m</strong> "
        f"(report horizontal miss {c.impact_to_intended_m:.1f} m). "
        "<span class='muted'>Orange = miss · Teal = correction · Purple = drone.</span>"
        "</p>"
    )

    def _viz(title: str, svg: str, extra_class: str = "") -> str:
        cls = f"viz-card {extra_class}".strip()
        return (
            f"<div class='{cls}'>"
            f"<div class='viz-card-head'>{_html_esc(title)}</div>"
            f"<div class='viz-card-body'><div class='fc-diagram-wrap'>{svg}</div></div>"
            "</div>"
        )

    return (
        "<div class='fc-legend'>"
        "<span><i class='fc-dot fc-dot-gun'></i>Gun</span>"
        "<span><i class='fc-dot fc-dot-target'></i>Actual target</span>"
        "<span><i class='fc-dot fc-dot-impact'></i>Impact Target</span>"
        "<span><i class='fc-dot fc-dot-drone'></i>Drone</span>"
        "</div>"
        + _fire_correction_workflow_html()
        + "<div class='viz-card fc-story-wrap'>"
        "<div class='viz-card-head'>What happened — 3 steps</div>"
        f"<div class='viz-card-body'>{story}</div></div>"
        + "<div class='fc-diagram-grid fc-diagram-grid-plan'>"
        + _viz("Plan view (North up)", plan, "fc-plan-viz")
        + "</div>"
        + "<div class='fc-diagram-grid fc-diagram-grid-maps'>"
        + _viz("Gun line view", gunline, "fc-gunline-viz")
        + _viz("Compass miss (target at centre)", compass)
        + "</div>"
        + f"<div class='viz-card fc-bars-full'>{bars}</div>"
        + foot
    )


def _plain_offset_parts(value_m: float, pos_word: str, neg_word: str) -> str | None:
    v = float(value_m)
    if abs(v) < 0.5:
        return None
    return f"{abs(v):.1f} m {pos_word if v > 0 else neg_word}"


def _plain_horizontal_miss_sentence(c: FireCorrection) -> str:
    parts: list[str] = []
    n = _plain_offset_parts(c.miss_north_m, "north of", "south of")
    e = _plain_offset_parts(c.miss_east_m, "east of", "west of")
    if n:
        parts.append(n)
    if e:
        parts.append(e)
    if not parts:
        return "almost on the target horizontally"
    if len(parts) == 1:
        return parts[0] + " the target"
    return parts[0] + " and " + parts[1] + " the target"


def _plain_range_sentence(c: FireCorrection) -> str:
    along = float(c.miss_along_m)
    if along > 0.5:
        return f"The round landed about <strong>{along:.1f} m too far</strong> (beyond the target along the gun line)."
    if along < -0.5:
        return f"The round landed about <strong>{abs(along):.1f} m short</strong> (before the target along the gun line)."
    return "Range along the gun line was close to the target."


def _plain_vertical_sentence(c: FireCorrection) -> str | None:
    if c.miss_vertical_m is None:
        return None
    v = float(c.miss_vertical_m)
    if abs(v) < 0.5:
        return "Height difference was negligible."
    if v > 0:
        return f"The target was <strong>{v:.1f} m higher</strong> than where the round landed."
    return f"The target was <strong>{abs(v):.1f} m lower</strong> than where the round landed."


def _plain_correction_bullets(c: FireCorrection) -> list[str]:
    bullets: list[str] = []
    corr_e = -float(c.miss_east_m)
    corr_n = -float(c.miss_north_m)
    if abs(corr_e) >= 0.5:
        bullets.append(
            f"Move aim <strong>{'east' if corr_e > 0 else 'west'}</strong> by {abs(corr_e):.1f} m"
        )
    if abs(corr_n) >= 0.5:
        bullets.append(
            f"Move aim <strong>{'north' if corr_n > 0 else 'south'}</strong> by {abs(corr_n):.1f} m"
        )
    if abs(float(c.deflection_correction_m)) >= 0.5:
        d = float(c.deflection_correction_m)
        bullets.append(
            f"Deflection (left/right on gun line): <strong>{'right' if d > 0 else 'left'} {abs(d):.1f} m</strong>"
        )
    if abs(float(c.range_correction_m)) >= 0.5:
        r = float(c.range_correction_m)
        bullets.append(
            f"Range: <strong>{'add' if r > 0 else 'drop'} {abs(r):.1f} m</strong>"
        )
    if c.elevation_correction_m is not None and abs(float(c.elevation_correction_m)) >= 0.5:
        u = float(c.elevation_correction_m)
        bullets.append(
            f"Elevation: <strong>{'up' if u > 0 else 'down'} {abs(u):.1f} m</strong>"
        )
    if not bullets:
        bullets.append("No significant correction needed — repeat the same fire data.")
    return bullets


def format_executive_summary_html(session: DooafSession) -> str:
    """Plain-language opening: what happened and what to do next."""
    c = session.correction
    if c is None:
        body = (
            "<p class='report-executive-lead'>This export records observation marks from the drone. "
            "To get fire correction, complete <strong>DOOAF Setup</strong> (gun + actual target), "
            "then turn <strong>Target ON</strong> and click the <strong>fall of shot</strong> on the video.</p>"
        )
        return f"<section class='report-executive' id='summary'><div class='report-executive-head'><h2>Summary</h2></div><div class='report-executive-body'>{body}</div></section>"

    horiz = float(c.impact_to_intended_m)
    range_line = _plain_range_sentence(c)
    story_parts = [
        "<div class='exec-story-lead'>",
        f"<p><span class='exec-big'>{horiz:.1f} m</span> total miss from the intended target.</p>",
        f"<p>Impact was <strong>{_plain_horizontal_miss_sentence(c)}</strong>.</p>",
        f"<p>{range_line}</p>",
    ]
    vert = _plain_vertical_sentence(c)
    if vert:
        story_parts.append(f"<p>{vert}</p>")
    story_parts.append("</div>")
    story_lead = "".join(story_parts)

    return (
        "<section class='report-executive' id='summary'>"
        "<div class='report-executive-head'>"
        "<h2>Summary<span class='report-executive-badge'>Start here</span></h2>"
        "</div>"
        "<div class='report-executive-body'>"
        + story_lead
        + _executive_summary_visual_html(session)
        + "</div></section>"
    )


def _guide_card_html(
    section_id: str,
    accent: str,
    title: str,
    description: str,
    icon_svg: str,
    preview_html: str,
) -> str:
    return (
        f"<a class='guide-card guide-card--{accent}' href='#{_html_esc(section_id)}'>"
        "<div class='guide-card-top'>"
        f"<span class='guide-card-icon' aria-hidden='true'>{icon_svg}</span>"
        "<div>"
        f"<span class='guide-card-title'>{_html_esc(title)}</span>"
        f"<span class='guide-card-desc'>{_html_esc(description)}</span>"
        "</div></div>"
        f"<div class='guide-preview' aria-hidden='true'>{preview_html}</div>"
        "<span class='guide-card-link'>Jump to section →</span>"
        "</a>"
    )


def format_report_reading_guide_html() -> str:
    icon_summary = (
        "<svg viewBox='0 0 32 32' fill='none' xmlns='http://www.w3.org/2000/svg'>"
        "<circle cx='16' cy='16' r='7' fill='#16a34a'/>"
        "<circle cx='22' cy='22' r='6' fill='#dc2626'/>"
        "<line x1='16' y1='16' x2='22' y2='22' stroke='#ea580c' stroke-width='2'/>"
        "</svg>"
    )
    icon_story = (
        "<svg viewBox='0 0 32 32' fill='none' xmlns='http://www.w3.org/2000/svg'>"
        "<circle cx='8' cy='16' r='5' fill='#16a34a'/>"
        "<circle cx='16' cy='16' r='5' fill='#dc2626'/>"
        "<circle cx='24' cy='16' r='5' fill='#0d9488'/>"
        "<path d='M13 16h2M21 16h2' stroke='#94a3b8' stroke-width='2'/>"
        "</svg>"
    )
    icon_diagrams = (
        "<svg viewBox='0 0 32 32' fill='none' xmlns='http://www.w3.org/2000/svg'>"
        "<rect x='4' y='18' width='24' height='3' rx='1' fill='#cbd5e1'/>"
        "<rect x='4' y='12' width='16' height='3' rx='1' fill='#ea580c'/>"
        "<rect x='4' y='6' width='20' height='3' rx='1' fill='#0d9488'/>"
        "</svg>"
    )
    icon_tables = (
        "<svg viewBox='0 0 32 32' fill='none' xmlns='http://www.w3.org/2000/svg'>"
        "<rect x='6' y='8' width='20' height='16' rx='2' stroke='#94a3b8' stroke-width='2'/>"
        "<line x1='6' y1='14' x2='26' y2='14' stroke='#94a3b8' stroke-width='1.5'/>"
        "<line x1='14' y1='8' x2='14' y2='24' stroke='#94a3b8' stroke-width='1.5'/>"
        "<text x='16' y='20' text-anchor='middle' font-size='8' fill='#64748b'>#</text>"
        "</svg>"
    )
    icon_map = (
        "<svg viewBox='0 0 32 32' fill='none' xmlns='http://www.w3.org/2000/svg'>"
        "<circle cx='10' cy='20' r='4' fill='#2563eb'/>"
        "<circle cx='22' cy='12' r='4' fill='#16a34a'/>"
        "<circle cx='18' cy='22' r='3' fill='#dc2626'/>"
        "<circle cx='14' cy='10' r='3' fill='#9333ea'/>"
        "</svg>"
    )
    icon_nav = (
        "<svg viewBox='0 0 32 32' fill='none' xmlns='http://www.w3.org/2000/svg'>"
        "<rect x='6' y='9' width='20' height='3' rx='1.5' fill='#0369a1'/>"
        "<rect x='6' y='15' width='14' height='3' rx='1.5' fill='#7dd3fc'/>"
        "<rect x='6' y='21' width='18' height='3' rx='1.5' fill='#0369a1'/>"
        "</svg>"
    )
    preview_summary = (
        "<span class='guide-preview-dot' style='background:#16a34a'></span>"
        "<span class='guide-preview-dot' style='background:#dc2626'></span>"
        "<span class='guide-preview-bar' style='width:40px'></span>"
    )
    preview_story = "① aim → ② land → ③ fix"
    preview_diagrams = (
        "<span class='guide-preview-bar' style='width:28px'></span>"
        "<span class='guide-preview-bar guide-preview-bar-corr' style='width:20px'></span>"
    )
    preview_tables = "▸ click to expand numbers"
    preview_map = (
        "<span class='guide-preview-dot' style='background:#2563eb'></span>"
        "<span class='guide-preview-dot' style='background:#16a34a'></span>"
        "<span class='guide-preview-dot' style='background:#dc2626'></span>"
        "<span class='guide-preview-dot' style='background:#9333ea'></span>"
    )
    flow = (
        "<div class='guide-flow'>"
        "<span class='guide-flow-label'>Quick path</span>"
        "<a class='guide-flow-step' href='#summary'>"
        "<span class='guide-flow-num'>1</span>Summary</a>"
        "<span class='guide-flow-arrow'>→</span>"
        "<a class='guide-flow-step' href='#correction'>"
        "<span class='guide-flow-num'>2</span>Diagrams</a>"
        "<span class='guide-flow-arrow'>→</span>"
        "<a class='guide-flow-step' href='#positions'>"
        "<span class='guide-flow-num'>3</span>Map</a>"
        "<span class='guide-flow-arrow'>→</span>"
        "<a class='guide-flow-step' href='#glossary'>"
        "<span class='guide-flow-num'>4</span>Glossary</a>"
        "</div>"
    )
    cards = (
        "<div class='guide-cards'>"
        + _guide_card_html(
            "summary",
            "summary",
            "Green summary",
            "Compass + correction cards — the answer in plain view.",
            icon_summary,
            preview_summary,
        )
        + _guide_card_html(
            "correction",
            "story",
            "3-step story",
            "You aimed → round landed → what to add next.",
            icon_story,
            preview_story,
        )
        + _guide_card_html(
            "correction",
            "diagrams",
            "Diagrams & bars",
            "Map views and bar charts for distance and direction.",
            icon_diagrams,
            preview_diagrams,
        )
        + _guide_card_html(
            "correction",
            "tables",
            "Number tables",
            "Exact figures — hidden until you expand them.",
            icon_tables,
            preview_tables,
        )
        + _guide_card_html(
            "positions",
            "map",
            "Positions map",
            "Gun, target, impact, and drone on the ground.",
            icon_map,
            preview_map,
        )
        + _guide_card_html(
            "summary",
            "nav",
            "Sticky menu",
            "Use the bar at the top to jump to any section.",
            icon_nav,
            "↑ scroll · menu highlights active section",
        )
        + "</div>"
    )
    return (
        "<section class='reading-guide' id='guide'>"
        "<div class='reading-guide-head'>"
        "<h3>How to read this report</h3>"
        "<p class='reading-guide-intro'>Pictures first — follow the quick path below. "
        "Open tables only if you need exact numbers.</p>"
        "</div>"
        + flow
        + cards
        + "</section>"
    )


def format_report_glossary_html() -> str:
    return (
        "<section class='section-card report-glossary' id='glossary'>"
        "<div class='section-head'>"
        "<h3 class='section-title'>Glossary</h3>"
        "<p class='section-subtitle'>Short definitions for report terms.</p>"
        "</div>"
        "<div class='section-body'>"
        "<details>"
        "<summary>What do these terms mean?</summary>"
        "<dl>"
        "<dt>Actual target</dt><dd>Where you intended to hit (set in DOOAF Setup → Pick on video).</dd>"
        "<dt>Impact Target</dt><dd>Where the round actually landed (Target ON → click on video).</dd>"
        "<dt>Miss</dt><dd>How far the impact was from the target, and in which direction.</dd>"
        "<dt>Correction (add)</dt><dd>Values to <em>add</em> to your firing data on the next round.</dd>"
        "<dt>East / West · North / South</dt><dd>Directions on the map (compass axes).</dd>"
        "<dt>Left / Right (gun line)</dt><dd>Sideways miss relative to the line from gun to target.</dd>"
        "<dt>Range along line</dt><dd>Shorten (−) or lengthen (+) range along the gun→target direction.</dd>"
        "<dt>Up / Down</dt><dd>Vertical correction using elevation (metres above sea level).</dd>"
        "<dt>MGRS</dt><dd>Military grid reference for the marked point.</dd>"
        "<dt>MSL</dt><dd>Height above mean sea level.</dd>"
        "</dl>"
        "</details>"
        "</div></section>"
    )


def format_elevation_summary_html(session: DooafSession) -> str:
    """DEM elevation and height correction between target (green) and impact (red)."""
    tgt = session.intended
    imp = session.impact
    if tgt is None and imp is None:
        return ""
    rows: list[str] = []
    if session.intended_dem_alt_m is not None:
        rows.append(
            f"<tr><td class='label-col'>Target DEM elevation</td>"
            f"<td>{_format_elev_msl_html(session.intended_dem_alt_m)} "
            "<span class='muted'>(terrain at target footprint)</span></td></tr>"
        )
    if tgt is not None and tgt.alt_m is not None:
        dem = session.intended_dem_alt_m
        if dem is None or abs(float(tgt.alt_m) - float(dem)) >= 0.15:
            rows.append(
                f"<tr><td class='label-col'>Target elevation (corrected)</td>"
                f"<td>{_format_elev_msl_html(tgt.alt_m)} "
                "<span class='muted'>(aim point on structure / facade geometry)</span></td></tr>"
            )
        else:
            rows.append(
                f"<tr><td class='label-col'>Target elevation (corrected)</td>"
                f"<td>{_format_elev_msl_html(tgt.alt_m)} "
                "<span class='muted'>(same as terrain DEM at footprint)</span></td></tr>"
            )
    if session.impact_dem_alt_m is not None:
        rows.append(
            f"<tr><td class='label-col'>Impact DEM elevation</td>"
            f"<td>{_format_elev_msl_html(session.impact_dem_alt_m)} "
            "<span class='muted'>(terrain at impact footprint)</span></td></tr>"
        )
    if imp is not None and imp.alt_m is not None:
        dem = session.impact_dem_alt_m
        if dem is None or abs(float(imp.alt_m) - float(dem)) >= 0.15:
            rows.append(
                f"<tr><td class='label-col'>Impact elevation (corrected)</td>"
                f"<td>{_format_elev_msl_html(imp.alt_m)} "
                "<span class='muted'>(fall of shot on structure / facade geometry)</span></td></tr>"
            )
        else:
            rows.append(
                f"<tr><td class='label-col'>Impact elevation (corrected)</td>"
                f"<td>{_format_elev_msl_html(imp.alt_m)} "
                "<span class='muted'>(same as terrain DEM at footprint)</span></td></tr>"
            )
    if session.height_correction_m is not None:
        h = float(session.height_correction_m)
        rows.append(
            f"<tr><td class='label-col'>Height correction (target − impact)</td>"
            f"<td><strong>{h:+.1f} m</strong> "
            "<span class='muted'>(+ = target above impact)</span></td></tr>"
        )
    elif tgt is not None and imp is not None:
        rows.append(
            "<tr><td class='label-col'>Height correction (target − impact)</td>"
            "<td><span class='muted'>— "
            "(need target + impact video picks at different heights, or higher drone hover)</span>"
            "</td></tr>"
        )
    if not rows:
        return ""
    return _report_section_card(
        "Elevation & height (DEM)",
        (
            "<p class='log-hint'>Green target vs red impact: DEM ground at each "
            "footprint, corrected elevations for elevated points, and vertical "
            "separation between the two marks.</p>"
            "<table class='data-table dooaf-elevation-summary'><tbody>"
            + "".join(rows)
            + "</tbody></table>"
        ),
        extra_class="dooaf-elevation-summary",
    )


def format_client_fire_correction_html(session: DooafSession) -> str:
    """Client summary: East/West, North/South, Left/Right, Up/Down corrections."""
    c = session.correction
    if c is None:
        return ""
    corr_east = -c.miss_east_m
    corr_north = -c.miss_north_m
    corr_left_right = c.deflection_correction_m
    up_down = c.elevation_correction_m

    miss_rows = (
        f"<tr><td class='label-col'>East / West miss</td>"
        f"<td>{_format_miss_dir(c.miss_east_m, 'impact east', 'impact west')}</td></tr>"
        f"<tr><td class='label-col'>North / South miss</td>"
        f"<td>{_format_miss_dir(c.miss_north_m, 'impact north', 'impact south')}</td></tr>"
        f"<tr><td class='label-col'>Left / Right miss (gun line)</td>"
        f"<td>{_format_miss_dir(c.miss_right_m, 'impact right', 'impact left')}</td></tr>"
    )
    if c.miss_vertical_m is not None:
        mv = float(c.miss_vertical_m)
        if abs(mv) < 0.05:
            up_down_miss = "0.0 m"
        elif mv > 0:
            up_down_miss = f"{mv:.1f} m (target above impact)"
        else:
            up_down_miss = f"{abs(mv):.1f} m (target below impact)"
        miss_rows += (
            f"<tr><td class='label-col'>Up / Down miss</td><td>{up_down_miss}</td></tr>"
        )

    corr_rows = (
        f"<tr><td class='label-col'><strong>East / West (add)</strong></td>"
        f"<td>{_format_signed_correction_dir(corr_east, pos_label='East', neg_label='West')}</td></tr>"
        f"<tr><td class='label-col'><strong>North / South (add)</strong></td>"
        f"<td>{_format_signed_correction_dir(corr_north, pos_label='North', neg_label='South')}</td></tr>"
        f"<tr><td class='label-col'><strong>Left / Right (add, R+)</strong></td>"
        f"<td>{_format_signed_correction_dir(corr_left_right, pos_label='Right', neg_label='Left')}</td></tr>"
        f"<tr><td class='label-col'><strong>Range along line (add)</strong></td>"
        f"<td>{c.range_correction_m:+.1f} m</td></tr>"
    )
    if up_down is not None:
        corr_rows += (
            f"<tr><td class='label-col'><strong>Up / Down (add)</strong></td>"
            f"<td>{_format_signed_correction_dir(up_down, pos_label='Up', neg_label='Down')}</td></tr>"
        )

    return _report_section_card(
        "Fire correction",
        (
            "<p class='log-hint'>Visual explanation first — open tables below for exact numbers.</p>"
            + format_fire_correction_diagram_html(session)
            + "<details class='report-collapsible'>"
            "<summary>Exact miss & correction numbers (tables)</summary>"
            "<div class='report-collapsible-body'>"
            "<table class='data-table dooaf-client-corr'>"
            "<thead><tr><th colspan='2'>Miss — impact relative to target</th></tr></thead>"
            "<tbody>"
            + miss_rows
            + "</tbody></table>"
            "<table class='data-table dooaf-client-corr' style='margin-top:12px'>"
            "<thead><tr><th colspan='2'>Correction — add for next round</th></tr></thead>"
            "<tbody>"
            + corr_rows
            + "</tbody></table>"
            "</div></details>"
        ),
        extra_class="dooaf-client-corr",
        section_id="correction",
        subtitle="Diagrams show where the round missed and what to add next.",
    )


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
            return f"<tr{cls}><td>{label}</td><td colspan='4'>—</td></tr>"
        gr = format_grid_reference(pt.lat, pt.lon) or "—"
        cls = f" class='{row_class}'" if row_class else ""
        return (
            f"<tr{cls}><td>{label}</td>"
            f"<td>{pt.lat:.7f}</td><td>{pt.lon:.7f}</td>"
            f"<td><span class='mgrs-badge'>{_html_esc(gr)}</span></td>"
            f"<td>{_format_elev_msl_html(pt.alt_m)}</td></tr>"
        )

    corr_rows = ""
    c = session.correction
    elev_delta_row = ""
    if (
        session.intended is not None
        and session.impact is not None
        and session.intended.alt_m is not None
        and session.impact.alt_m is not None
    ):
        try:
            elev_delta = float(session.intended.alt_m) - float(session.impact.alt_m)
            elev_delta_row = (
                f"<tr><td class='label-col'>Target − impact elevation</td>"
                f"<td>{elev_delta:+.1f} m MSL "
                f"(target {_format_elev_msl_html(session.intended.alt_m)}, "
                f"impact {_format_elev_msl_html(session.impact.alt_m)})</td></tr>"
            )
        except (TypeError, ValueError):
            elev_delta_row = ""
    if c is not None:
        vert_card = ""
        if c.miss_vertical_m is not None:
            vert_card = (
                "<div class='metric-card'>"
                "<div class='metric-label'>Vertical miss (target − impact)</div>"
                f"<div class='metric-value'>{c.miss_vertical_m:+.1f} m</div>"
                "<div class='metric-sub'>MSL elevation difference</div>"
                "</div>"
            )
        elev_corr_row = ""
        if c.elevation_correction_m is not None:
            elev_corr_row = (
                f"<tr><td class='label-col'>Elevation correction (add, Up/Down)</td>"
                f"<td>{_format_signed_correction_dir(c.elevation_correction_m, pos_label='Up', neg_label='Down')}</td></tr>"
            )
        building_row = ""
        if session.height_correction_m is not None:
            building_row = (
                f"<tr><td class='label-col'>Height correction (target − impact)</td>"
                f"<td>{session.height_correction_m:+.1f} m</td></tr>"
            )
        corr_rows = _report_section_card(
            "Technical reference (fire geometry)",
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
                "<div class='metric-label'>Horizontal miss</div>"
                f"<div class='metric-value'>{c.impact_to_intended_m:.1f} m</div>"
                "<div class='metric-sub'>Ground distance intended ↔ impact</div>"
                "</div>"
                + vert_card
                + "</div>"
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
                + building_row
                + elev_corr_row
                + elev_delta_row
                + "</tbody></table>"
            ),
            extra_class="dooaf-fire-corr section-technical",
            section_id="technical",
            subtitle="Detailed geometry for engineers and audit.",
        )
    obs_row = observation_row or None
    positions_map = _fire_correction_positions_svg(session)
    map_block = ""
    if positions_map:
        map_block = (
            "<div class='viz-card' style='margin-bottom:14px'>"
            "<div class='viz-card-head'>All positions on the map</div>"
            f"<div class='viz-card-body'><div class='fc-diagram-wrap'>{positions_map}</div></div>"
            "</div>"
            "<p class='log-hint'>Blue dashed = gun to target. Grey dashed = gun to impact. "
            "Orange = miss (target to impact). Purple dashed = drone line of sight to target. "
            "Full distances and miss are in the table below the map.</p>"
        )
    session_table = (
        "<table class='data-table'>"
        "<thead><tr><th>Variable</th><th>Lat</th><th>Lon</th>"
        "<th>Grid ref (MGRS)</th><th>Elevation (MSL)</th></tr></thead>"
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
    session_body = map_block + (
        "<details class='report-collapsible'>"
        "<summary>Coordinate table (lat / lon / MGRS)</summary>"
        f"<div class='report-collapsible-body'>{session_table}</div>"
        "</details>"
    )
    return (
        format_executive_summary_html(session)
        + format_report_reading_guide_html()
        + format_client_fire_correction_html(session)
        + _report_section_card(
            "Positions",
            session_body,
            section_id="positions",
            subtitle="Gun, target, Impact Target, and drone — map plus coordinate table.",
        )
        + format_elevation_summary_html(session)
        + format_camera_orientation_html(obs_row)
        + corr_rows
        + format_report_glossary_html()
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
            "dooaf_east_correction_m",
            "dooaf_elevation_correction_m",
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


def _log_summary_rows(row: dict[str, object], cell_fn: Any) -> list[str]:
    """Top-of-entry summary fields in the same table layout as coordinate rows."""
    rows: list[str] = [_log_detail_section("Summary")]

    mgrs = str(row.get("target_grid_ref") or row.get("map_grid_ref") or "").strip()
    rows.append(
        _log_detail_row(
            "Grid reference",
            _format_mgrs_badge(mgrs) if mgrs else "<span class='muted'>—</span>",
        )
    )

    if not _is_missing_cell(row.get("geo_method")):
        rows.append(
            _log_detail_row("Geo method", format_geo_method_badge(row.get("geo_method")))
        )
    if not _is_missing_cell(row.get("geo_quality")):
        rows.append(
            _log_detail_row("Geo quality", format_geo_quality_badge(row.get("geo_quality")))
        )

    if not _is_missing_cell(row.get("segment_distance_m"), cell_fn):
        rows.append(
            _log_detail_row(
                "Separation",
                _format_distance_m_html(row.get("segment_distance_m"), cell_fn),
            )
        )
    if not _is_missing_cell(row.get("geo_range_m"), cell_fn):
        rows.append(
            _log_detail_row(
                "Geo range",
                _format_distance_m_html(row.get("geo_range_m"), cell_fn),
            )
        )

    ekf_val = row.get("ekf_rel_alt_m")
    if _is_missing_cell(ekf_val, cell_fn):
        ekf_val = row.get("vehicle_rel_alt_m")
    if not _is_missing_cell(ekf_val, cell_fn):
        rows.append(
            _log_detail_row(
                "EKF rel (above home)",
                _format_alt_m_html(ekf_val, cell_fn),
            )
        )

    dem_ground = row.get("dem_ground_agl_m")
    ray_agl = row.get("measure_agl_m")
    if not _is_missing_cell(dem_ground, cell_fn):
        rows.append(
            _log_detail_row(
                "Ground height (DEM)",
                _format_alt_m_html(dem_ground, cell_fn),
            )
        )
    elif not _is_missing_cell(ray_agl, cell_fn):
        rows.append(
            _log_detail_row(
                "Ray height (AGL)",
                _format_alt_m_html(ray_agl, cell_fn),
            )
        )

    return rows


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

    detail_rows: list[str] = _log_summary_rows(row, cell_fn)

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
        msl_alt = row.get("vehicle_alt_msl_m")
        if not _is_missing_cell(msl_alt, cell_fn):
            detail_rows.append(
                _log_detail_row(
                    "Altitude (MSL)",
                    _format_alt_m_html(msl_alt, cell_fn),
                )
            )
        ekf_alt = row.get("ekf_rel_alt_m")
        if _is_missing_cell(ekf_alt, cell_fn):
            ekf_alt = row.get("vehicle_rel_alt_m")
        if not _is_missing_cell(ekf_alt, cell_fn):
            detail_rows.append(
                _log_detail_row(
                    "EKF rel (above home)",
                    _format_alt_m_html(ekf_alt, cell_fn),
                )
            )
        if not _is_missing_cell(row.get("dem_ground_agl_m"), cell_fn):
            detail_rows.append(
                _log_detail_row(
                    "Ground height (DEM)",
                    _format_alt_m_html(row.get("dem_ground_agl_m"), cell_fn),
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
        if not _is_missing_cell(row.get("dooaf_target_dem_alt_m"), cell_fn):
            detail_rows.append(
                _log_detail_row(
                    "Target DEM elevation",
                    _format_alt_m_html(row.get("dooaf_target_dem_alt_m"), cell_fn),
                )
            )
        if not _is_missing_cell(row.get("dooaf_impact_dem_alt_m"), cell_fn):
            detail_rows.append(
                _log_detail_row(
                    "Impact DEM elevation",
                    _format_alt_m_html(row.get("dooaf_impact_dem_alt_m"), cell_fn),
                )
            )
        if not _is_missing_cell(row.get("dooaf_height_correction_m"), cell_fn):
            detail_rows.append(
                _log_detail_row(
                    "Height correction (target − impact)",
                    _format_distance_m_html(row.get("dooaf_height_correction_m"), cell_fn),
                )
            )
        if not _is_missing_cell(row.get("dooaf_east_correction_m"), cell_fn):
            detail_rows.append(
                _log_detail_row(
                    "East / West correction (add)",
                    _format_distance_m_html(row.get("dooaf_east_correction_m"), cell_fn),
                )
            )
        if not _is_missing_cell(row.get("dooaf_north_correction_m"), cell_fn):
            detail_rows.append(
                _log_detail_row(
                    "North / South correction (add)",
                    _format_distance_m_html(row.get("dooaf_north_correction_m"), cell_fn),
                )
            )
        if not _is_missing_cell(row.get("dooaf_elevation_correction_m"), cell_fn):
            detail_rows.append(
                _log_detail_row(
                    "Up / Down correction (add)",
                    _format_distance_m_html(row.get("dooaf_elevation_correction_m"), cell_fn),
                )
            )
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
                    "Left / Right correction (add, R+)",
                    _format_distance_m_html(row.get("dooaf_deflection_correction_m"), cell_fn),
                )
            )
        if not _is_missing_cell(row.get("dooaf_miss_m"), cell_fn):
            detail_rows.append(
                _log_detail_row(
                    "Horizontal miss",
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
        return _report_section_card(
            "Audit log",
            body,
            section_id="audit",
            subtitle="Full field dump for engineers.",
        )

    entries = "".join(
        _format_observation_log_entry(idx, row, cell_fn)
        for idx, row in enumerate(export_rows, start=1)
    )
    hint = (
        "<p class='log-hint'>For auditors and engineers: every recorded field from each "
        "observation mark. Hover lat/lon for full precision. "
        "<strong>Full raw export</strong> (every field, unrounded) is in the "
        "<span class='mono'>CSV</span> file saved beside this HTML.</p>"
    )
    body = hint + f"<div class='log-entries'>{entries}</div>"
    return _report_section_card(
        "Audit log",
        body,
        section_id="audit",
        subtitle="Full field dump for engineers.",
    )


def assemble_observation_report_html(
    entry_count: int,
    dooaf_summary_html: str,
    detailed_log_html: str,
    *,
    title: str = "Observation Report",
    session: DooafSession | None = None,
) -> str:
    return (
        observation_report_html_head(title=title)
        + format_observation_report_header(entry_count, title=title, session=session)
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
