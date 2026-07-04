"""DOOAF fire-correction math, geo, settings, and session assembly."""

from __future__ import annotations

import json
import math
from typing import Any

from vgcs.observe._dooaf_types import (
    DOOAF_ROLE_GUN,
    DOOAF_ROLE_IMPACT,
    DOOAF_ROLE_INTENDED,
    DOOAF_ROLE_SURVEY,
    DOOAF_ROLES,
    DooafPreset,
    DooafSession,
    DooafSettings,
    FireCorrection,
    GeoPoint,
    _SETUP_MARK_ROLE_ALIASES,
    dooaf_role_display,
)
from vgcs.observe.target_measure import (
    haversine_m,
    low_hover_ray_agl_m,
    observation_ekf_rel_alt_m,
    observation_target_latlon,
)


_QS_GUN_LAT = "dooaf/gun_lat"

_QS_GUN_LON = "dooaf/gun_lon"

_QS_GUN_ALT = "dooaf/gun_alt_m"

_QS_TARGET_LAT = "dooaf/target_lat"

_QS_TARGET_LON = "dooaf/target_lon"

_QS_TARGET_ALT = "dooaf/target_alt_m"

_QS_PRESETS_JSON = "dooaf/presets_json"

_QS_SETUP_VIDEO_MARKS_JSON = "dooaf/setup_video_marks_json"

_QS_FACADE_SLANT_M = "dooaf/facade_slant_range_m"

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


def _row_prefers_facade_geometry(row: dict[str, Any] | None) -> bool:
    """True when a mark row should use facade/ray geometry over pure footprint."""
    if not isinstance(row, dict):
        return False
    method = str(row.get("geo_method") or "").strip().lower()
    if method.startswith("ray_"):
        return False
    if "facade" in method:
        return True
    if method in {"lrf_slant", "lrf_lock"}:
        return True
    slant = _float_or_none(row.get("lrf_slant_range_m"))
    if slant is not None and slant >= 8.0:
        return True
    if bool(row.get("facade_uv_pick") or row.get("lrf_boresight_geo")):
        return True
    # Legacy rows can miss explicit facade method but carry full ray pair.
    if (
        row.get("geo_range_m") is not None
        and row.get("geo_bearing_deg") is not None
        and row.get("video_x_norm") is not None
        and row.get("video_y_norm") is not None
    ):
        return True
    return False


def _pair_prefers_facade_geometry(
    left: dict[str, Any] | None, right: dict[str, Any] | None
) -> bool:
    left_pref = _row_prefers_facade_geometry(left)
    right_pref = _row_prefers_facade_geometry(right)
    if left_pref and right_pref:
        return True
    # Allow one-sided facade/ray rows to seed the paired mark when the
    # counterpart is a video mark without explicit ground-ray method.
    for row in (left, right):
        if not isinstance(row, dict):
            return False
        method = str(row.get("geo_method") or "").strip().lower()
        if method.startswith("ray_terrain"):
            return False
    return left_pref or right_pref


def _c13_lrf_vfov_deg(hfov_deg: float) -> float:
    try:
        from vgcs.skydroid.adapter import _LRF_FOV_V_DEG as vfov  # type: ignore[attr-defined]

        return float(vfov)
    except Exception:
        return float(hfov_deg) * 0.5625


def _facade_intended_impact_vertical_m(
    intended_row: dict[str, Any] | None,
    impact_row: dict[str, Any] | None,
    *,
    hfov_deg: float,
) -> float | None:
    """Signed target−impact vertical separation (m) on a shared LRF facade lock."""
    if intended_row is None or impact_row is None:
        return None
    if intended_row.get("video_y_norm") is None or impact_row.get("video_y_norm") is None:
        return None
    slants: list[float] = []
    for row in (intended_row, impact_row):
        try:
            s = float(row.get("lrf_slant_range_m") or 0)
        except (TypeError, ValueError):
            continue
        if s >= 8.0:
            slants.append(s)
    if not slants:
        return None
    from vgcs.observe.facade_plane import facade_intended_impact_vertical_m

    return facade_intended_impact_vertical_m(
        intended_row,
        impact_row,
        hfov_deg=float(hfov_deg),
        camera_vfov_deg=_c13_lrf_vfov_deg(float(hfov_deg)),
    )


def _apply_facade_vertical_to_points(
    intended: GeoPoint,
    impact: GeoPoint,
    *,
    intended_row: dict[str, Any] | None,
    impact_row: dict[str, Any] | None,
    hfov_deg: float,
) -> tuple[GeoPoint, GeoPoint]:
    """Reconcile intended/impact MSL using video Y + LRF slant vertical separation."""
    vert = _facade_intended_impact_vertical_m(
        intended_row,
        impact_row,
        hfov_deg=hfov_deg,
    )
    if vert is None or abs(float(vert)) < 0.05:
        return intended, impact
    v = float(vert)
    if impact.alt_m is not None:
        return GeoPoint(intended.lat, intended.lon, float(impact.alt_m) + v), impact
    if intended.alt_m is not None:
        return intended, GeoPoint(impact.lat, impact.lon, float(intended.alt_m) - v)
    return intended, impact


def _facade_target_impact_separation_m(
    intended_row: dict[str, Any] | None,
    impact_row: dict[str, Any] | None,
    *,
    hfov_deg: float,
) -> float | None:
    """Wall-surface chord (m) from shared LRF slant + two video UV picks."""
    if intended_row is None or impact_row is None:
        return None
    if intended_row.get("video_x_norm") is None or impact_row.get("video_x_norm") is None:
        return None
    slants: list[float] = []
    for row in (intended_row, impact_row):
        try:
            s = float(row.get("lrf_slant_range_m") or 0)
        except (TypeError, ValueError):
            continue
        if s >= 8.0:
            slants.append(s)
    if not slants:
        return None
    from vgcs.observe.facade_plane import facade_slant_uv_separation_m

    return facade_slant_uv_separation_m(
        intended_row,
        impact_row,
        hfov_deg=float(hfov_deg),
        camera_vfov_deg=_c13_lrf_vfov_deg(float(hfov_deg)),
    )


def compute_fire_correction(
    gun: GeoPoint,
    intended: GeoPoint,
    impact: GeoPoint,
    *,
    gun_row: dict[str, Any] | None = None,
    intended_row: dict[str, Any] | None = None,
    impact_row: dict[str, Any] | None = None,
    camera_hfov_deg: float = 62.0,
) -> FireCorrection:
    """
    Gun-centric miss and correction.

    ``miss_along`` > 0 when impact is beyond intended along gun→target line.
    ``miss_right`` > 0 when impact is to the right of gun→target line.
    Corrections are the negation (what to add to firing data).
    Horizontal miss is ground distance; vertical miss uses MSL altitudes when set.

    When mark rows carry per-click ``geo_range_m`` / ``geo_bearing_deg``, gun→target
  and gun→impact ranges use ray geometry (law of cosines) instead of haversine on
    DEM footprints that often cluster on facade picks.
    """
    from vgcs.observe.target_measure import mark_pair_fire_range_m

    use_ray_gt = _pair_prefers_facade_geometry(gun_row, intended_row)
    use_ray_gi = _pair_prefers_facade_geometry(gun_row, impact_row)
    use_facade_ti = _pair_prefers_facade_geometry(intended_row, impact_row)

    if impact_row is not None and (use_ray_gt or use_ray_gi or use_facade_ti):
        template = gun_row or intended_row
        if template is not None:
            _enrich_mark_row_ray_geometry(impact_row, template)
    for row in (gun_row, intended_row):
        if row is not None and impact_row is not None and (
            use_ray_gt or use_ray_gi or use_facade_ti
        ):
            _enrich_mark_row_ray_geometry(row, impact_row)

    range_gt = haversine_m(gun.lat, gun.lon, intended.lat, intended.lon)
    range_gi = haversine_m(gun.lat, gun.lon, impact.lat, impact.lon)
    if use_ray_gt and gun_row is not None and intended_row is not None:
        ray_gt = mark_pair_fire_range_m(
            gun_row,
            intended_row,
            hfov_deg=camera_hfov_deg,
            footprint_m=range_gt,
        )
        if ray_gt is not None:
            range_gt = float(ray_gt)
    if use_ray_gi and gun_row is not None and impact_row is not None:
        ray_gi = mark_pair_fire_range_m(
            gun_row,
            impact_row,
            hfov_deg=camera_hfov_deg,
            footprint_m=range_gi,
        )
        if ray_gi is not None:
            range_gi = float(ray_gi)
    bearing_gt = initial_bearing_deg(gun.lat, gun.lon, intended.lat, intended.lon)
    bearing_gi = initial_bearing_deg(gun.lat, gun.lon, impact.lat, impact.lon)
    d_theta = math.radians(bearing_gi - bearing_gt)
    along = range_gi * math.cos(d_theta) - range_gt
    right = range_gi * math.sin(d_theta)
    miss_n, miss_e = latlon_delta_to_ne_m(
        intended.lat, intended.lon, impact.lat, impact.lon
    )
    map_footprint_miss_m = haversine_m(
        intended.lat, intended.lon, impact.lat, impact.lon
    )
    impact_to_intended_m = map_footprint_miss_m
    ti_facade = _facade_target_impact_separation_m(
        intended_row,
        impact_row,
        hfov_deg=camera_hfov_deg,
    )
    if use_facade_ti and ti_facade is not None and ti_facade >= 0.0:
        impact_to_intended_m = float(ti_facade)
        en_raw = math.hypot(miss_n, miss_e)
        if en_raw > 0.01:
            # Facade wall chord vs flat lat/lon footprint often diverge slightly;
            # scale E/N to the facade miss so report total matches compass/bars.
            scale = impact_to_intended_m / en_raw
            miss_n *= scale
            miss_e *= scale
    facade_vert = _facade_intended_impact_vertical_m(
        intended_row,
        impact_row,
        hfov_deg=camera_hfov_deg,
    )
    miss_vertical: float | None = None
    elev_correction: float | None = None
    if facade_vert is not None:
        miss_vertical = float(facade_vert)
        elev_correction = float(facade_vert)
    elif intended.alt_m is not None and impact.alt_m is not None:
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
        impact_to_intended_m=impact_to_intended_m,
        miss_east_m=miss_e,
        miss_north_m=miss_n,
        miss_vertical_m=miss_vertical,
        elevation_correction_m=elev_correction,
    )

FIRE_CORRECTION_MISS_CONSISTENCY_TOL_M = 2.0


def fire_correction_en_miss_m(c: FireCorrection) -> float:
    """Horizontal miss from East/North components: √(E² + N²)."""
    return math.hypot(float(c.miss_east_m), float(c.miss_north_m))


def fire_correction_miss_consistency_gap_m(c: FireCorrection) -> float:
    """|target→impact horizontal miss − √(E²+N²)|."""
    return abs(float(c.impact_to_intended_m) - fire_correction_en_miss_m(c))


def fire_correction_miss_is_consistent(
    c: FireCorrection,
    *,
    tol_m: float = FIRE_CORRECTION_MISS_CONSISTENCY_TOL_M,
) -> bool:
    return fire_correction_miss_consistency_gap_m(c) <= float(tol_m)


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
    ground_row: dict[str, Any] | None = None,
    hfov_deg: float = 62.0,
) -> tuple[GeoPoint | None, GeoPoint | None, float | None]:
    """
    Apply facade geometry to DOOAF intended/impact MSL altitudes.

    When a gun/ground video pick is available, target and impact heights are
    interpolated along the facade from terrain at the ground click. Otherwise
    falls back to a two-mark roof/base pair between target and impact.
    """
    from vgcs.observe.facade_plane import (
        facade_msl_heights_from_ground_mark,
        facade_msl_heights_from_horizon_marks,
        facade_vertical_height_between_marks,
    )

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
    min_pair_h = 0.35
    if pair_h is not None and pair_h >= min_pair_h:
        uy_i = intended_row.get("video_y_norm")
        uy_p = impact_row.get("video_y_norm")
        if uy_i is not None and uy_p is not None:
            try:
                y_intended = float(uy_i)
                y_impact = float(uy_p)
            except (TypeError, ValueError):
                y_intended = y_impact = 0.0
            else:
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
                        new_intended = GeoPoint(
                            intended.lat, intended.lon, roof_alt - pair_h
                        )
                        building_h = pair_h

    if (
        ground_row is not None
        and new_impact is not None
        and new_impact.alt_m is not None
        and new_intended is not None
    ):
        t_msl, _g_msl = facade_msl_heights_from_ground_mark(
            ground_row,
            intended_row,
            impact_row,
            float(new_impact.alt_m),
        )
        if t_msl is not None:
            new_intended = GeoPoint(intended.lat, intended.lon, t_msl)
            building_h = abs(float(new_impact.alt_m) - float(t_msl))
            intended_row["target_alt_method"] = "facade_ground_interpolate"
            impact_row["target_alt_method"] = "facade_ground_interpolate"
            if building_h is not None and building_h > 0.1:
                intended_row["building_height_m"] = building_h
        else:
            g_msl, t_msl_h, i_msl = facade_msl_heights_from_horizon_marks(
                ground_row,
                intended_row,
                impact_row,
                hfov_deg=hfov_deg,
            )
            if g_msl is not None and t_msl_h is not None and i_msl is not None:
                new_intended = GeoPoint(intended.lat, intended.lon, t_msl_h)
                new_impact = GeoPoint(impact.lat, impact.lon, i_msl)
                building_h = abs(float(t_msl_h) - float(i_msl))
                intended_row["target_alt_method"] = "horizon_video_interpolate"
                impact_row["target_alt_method"] = "horizon_video_interpolate"
                ground_row["resolved_alt_msl_m"] = g_msl
                if building_h > 0.1:
                    intended_row["building_height_m"] = building_h

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
    lat = _float_or_none(row.get("vehicle_lat"))
    lon = _float_or_none(row.get("vehicle_lon"))
    ekf = observation_ekf_rel_alt_m(row)
    dem = dem_alt_msl_at_mark(row, GeoPoint(lat, lon, None)) if lat is not None and lon is not None else None
    ground_agl = _float_or_none(row.get("dem_ground_agl_m"))
    if dem is not None and ground_agl is not None and ground_agl > 0.5:
        ekf_val = float(ekf) if ekf is not None else 0.0
        if ground_agl > ekf_val + 2.0 or ekf_val < 3.0:
            return float(dem) + float(ground_agl)
    msl = _float_or_none(row.get("vehicle_alt_msl_m"))
    if dem is not None and ekf is not None and ekf < 8.0:
        est = float(dem) + max(float(ekf), 0.5)
        if msl is not None and abs(float(msl) - est) > 18.0:
            return est
    if msl is not None:
        return msl
    agl = _float_or_none(row.get("dem_ground_agl_m"))
    if ekf is not None and ekf < 2.5 and agl is not None and agl > float(ekf) + 12.0:
        agl = max(float(ekf), 0.5)
    elif agl is None and ekf is not None and ekf > 0.05:
        agl = float(ekf)
    measure = _float_or_none(row.get("measure_agl_m"))
    if (
        ekf is not None
        and ekf < 6.0
        and measure is not None
        and measure > float(ekf) + 10.0
    ):
        agl = max(float(ekf), 0.5)
    elif agl is None and measure is not None:
        agl = measure
    if lat is not None and lon is not None and agl is not None and dem is not None:
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
    slant_raw = template_row.get("lrf_slant_range_m")
    if slant_raw is not None:
        try:
            row["lrf_slant_range_m"] = float(slant_raw)
        except (TypeError, ValueError):
            pass
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
        dem_terrain=True,
    )
    from vgcs.observe.target_measure import (
        low_hover_ray_agl_m,
        observation_cached_dem_ground_agl_m,
        observation_ekf_rel_alt_m,
        ray_agl_suspect_dem_mismatch,
        resolve_ray_agl_for_geo,
    )

    cached_dem = observation_cached_dem_ground_agl_m(template_row) or observation_cached_dem_ground_agl_m(row)
    ray_agl, ray_src = resolve_ray_agl_for_geo(
        relative_alt_m=row.get("ekf_rel_alt_m"),  # type: ignore[arg-type]
        rangefinder_down_m=row.get("rangefinder_down_m"),  # type: ignore[arg-type]
        video_y_norm=float(video_y),
        vehicle_alt_msl_m=row.get("vehicle_alt_msl_m"),  # type: ignore[arg-type]
        vehicle_lat=row.get("vehicle_lat"),  # type: ignore[arg-type]
        vehicle_lon=row.get("vehicle_lon"),  # type: ignore[arg-type]
        dem_path=str(dem_path) if dem_path else None,
        cached_dem_ground_agl_m=cached_dem,
    )
    needs_retry = (
        not geo.ok
        or geo.target_lat is None
        or ray_agl_suspect_dem_mismatch(ray_agl, ray_src, row.get("ekf_rel_alt_m"))
    )
    if needs_retry and ray_agl is not None:
        retry_agl = max(float(ray_agl), low_hover_ray_agl_m(observation_ekf_rel_alt_m(row)))
        retry_geo = compute_geo_reference(
            vehicle_lat=row.get("vehicle_lat"),  # type: ignore[arg-type]
            vehicle_lon=row.get("vehicle_lon"),  # type: ignore[arg-type]
            vehicle_heading_deg=row.get("vehicle_heading_deg"),  # type: ignore[arg-type]
            vehicle_roll_deg=row.get("vehicle_roll_deg"),  # type: ignore[arg-type]
            vehicle_pitch_deg=row.get("vehicle_pitch_deg"),  # type: ignore[arg-type]
            vehicle_rel_alt_m=row.get("ekf_rel_alt_m"),  # type: ignore[arg-type]
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
            dem_terrain=False,
            force_agl_m=retry_agl,
        )
        if retry_geo.ok and retry_geo.target_lat is not None:
            geo = retry_geo
            ray_agl = retry_agl
    if geo.ok:
        # Keep DOOAF Setup footprint; ray fields are for facade elevation geometry.
        row["target_lat"] = lat
        row["target_lon"] = lon
        if geo.target_alt_m is not None and row.get("target_alt_m") is None:
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
        if ray_agl is not None:
            row["measure_agl_m"] = ray_agl
            row["geo_agl_source"] = ray_src
            row["agl_source"] = ray_src
    enrich_video_mark_target_altitude(row)
    return row

def _enrich_mark_row_ray_geometry(
    row: dict[str, Any],
    template_row: dict[str, Any],
    *,
    dem_path: str | Path | None = None,
) -> None:
    """Fill per-click ray range/bearing on observation rows when enrich dropped them."""
    if row.get("geo_range_m") is not None and row.get("geo_bearing_deg") is not None:
        return
    vx = row.get("video_x_norm")
    vy = row.get("video_y_norm")
    lat = row.get("target_lat")
    lon = row.get("target_lon")
    if vx is None or vy is None or lat is None or lon is None:
        return
    try:
        synth = _synthesize_setup_mark_row(
            str(row.get("dooaf_role") or DOOAF_ROLE_IMPACT),
            float(lat),
            float(lon),
            float(vx),
            float(vy),
            template_row,
            dem_path=dem_path,
        )
    except (TypeError, ValueError):
        return
    for key in (
        "geo_range_m",
        "geo_bearing_deg",
        "geo_depression_deg",
        "measure_agl_m",
        "geo_agl_source",
        "agl_source",
    ):
        if row.get(key) is None and synth.get(key) is not None:
            row[key] = synth[key]

def _apply_geo_reference_to_mark_row(
    row: dict[str, Any],
    geo: Any,
    *,
    ray_agl: float | None,
    ray_src: str,
) -> None:
    from vgcs.observe.geo_reference import enrich_video_mark_target_altitude
    from vgcs.observe.target_measure import is_plausible_ground_range

    row["target_lat"] = geo.target_lat
    row["target_lon"] = geo.target_lon
    row["target_alt_m"] = geo.target_alt_m
    row["geo_quality"] = geo.quality
    row["geo_warning"] = geo.warning
    row["geo_method"] = geo.method
    if geo.depression_deg is not None:
        row["geo_depression_deg"] = geo.depression_deg
    else:
        row["geo_depression_deg"] = None
    long_range_ray = ray_src == "rangefinder_clamped_long" or str(
        geo.method or ""
    ).startswith("ray_slant_long")
    if (
        geo.horizontal_range_m is not None
        and geo.bearing_deg is not None
        and geo.depression_deg is not None
        and ray_agl is not None
        and (
            long_range_ray
            or str(geo.method or "") in ("ray_facade_retry", "forced_facade_retry")
            or is_plausible_ground_range(
                float(ray_agl),
                float(geo.horizontal_range_m),
                float(geo.depression_deg),
            )
        )
    ):
        row["geo_range_m"] = geo.horizontal_range_m
        row["geo_bearing_deg"] = geo.bearing_deg
    else:
        row["geo_range_m"] = None
        row["geo_bearing_deg"] = None
    if ray_agl is not None:
        row["measure_agl_m"] = ray_agl
        row["geo_agl_source"] = ray_src
        row["agl_source"] = ray_src
    enrich_video_mark_target_altitude(row)
    if row.get("target_lat") is None or row.get("target_lon") is None:
        row["target_lat"] = None
        row["target_lon"] = None

def _forced_ray_geo_for_row(
    row: dict[str, Any],
    *,
    dem_path: str | Path | None = None,
    camera_hfov_deg: float = 62.0,
    vehicle_alt_msl_m: float | None = None,
    force_agl_m: float | None = None,
) -> Any:
    from vgcs.observe.geo_reference import compute_geo_reference

    vx = row.get("video_x_norm")
    vy = row.get("video_y_norm")
    if vx is None or vy is None:
        return None
    ekf = observation_ekf_rel_alt_m(row)
    agl = force_agl_m if force_agl_m is not None else low_hover_ray_agl_m(ekf)
    return compute_geo_reference(
        vehicle_lat=row.get("vehicle_lat"),  # type: ignore[arg-type]
        vehicle_lon=row.get("vehicle_lon"),  # type: ignore[arg-type]
        vehicle_heading_deg=row.get("vehicle_heading_deg"),  # type: ignore[arg-type]
        vehicle_roll_deg=row.get("vehicle_roll_deg"),  # type: ignore[arg-type]
        vehicle_pitch_deg=row.get("vehicle_pitch_deg"),  # type: ignore[arg-type]
        vehicle_rel_alt_m=row.get("ekf_rel_alt_m"),  # type: ignore[arg-type]
        vehicle_alt_msl_m=vehicle_alt_msl_m or row.get("vehicle_alt_msl_m"),  # type: ignore[arg-type]
        rangefinder_down_m=row.get("rangefinder_down_m"),  # type: ignore[arg-type]
        gimbal_yaw_deg=row.get("gimbal_yaw_deg"),  # type: ignore[arg-type]
        gimbal_pitch_deg=row.get("gimbal_pitch_deg"),  # type: ignore[arg-type]
        video_x_norm=float(vx),
        video_y_norm=float(vy),
        gps_fix_type=int(row.get("gps_fix_type") or 0),
        gps_hdop=row.get("gps_hdop"),  # type: ignore[arg-type]
        camera_hfov_deg=float(camera_hfov_deg),
        dem_path=dem_path,
        dem_terrain=False,
        force_agl_m=float(agl),
    )

def normalize_dooaf_setup_video_marks(
    marks: dict[str, tuple[float, float]] | None,
) -> dict[str, tuple[float, float]] | None:
    """Map legacy setup keys (``gun``, ``intended``) to canonical DOOAF roles."""
    if not marks:
        return None
    out: dict[str, tuple[float, float]] = {}
    for role, pt in marks.items():
        key = _SETUP_MARK_ROLE_ALIASES.get(str(role), str(role))
        try:
            out[key] = (float(pt[0]), float(pt[1]))
        except (TypeError, ValueError, IndexError):
            continue
    return out or None

def _setup_video_marks_complete(
    marks: dict[str, tuple[float, float]] | None,
) -> bool:
    return bool(
        marks
        and DOOAF_ROLE_GUN in marks
        and DOOAF_ROLE_INTENDED in marks
    )

def _destination_point_m(
    lat_deg: float,
    lon_deg: float,
    bearing_deg: float,
    distance_m: float,
) -> tuple[float, float]:
    from vgcs.observe.geo_reference import _offset_lat_lon

    br = math.radians(float(bearing_deg))
    north = float(distance_m) * math.cos(br)
    east = float(distance_m) * math.sin(br)
    return _offset_lat_lon(float(lat_deg), float(lon_deg), north, east)

def refine_impact_geo_artillery_field(
    row: dict[str, Any],
    *,
    gun_lat: float | None,
    gun_lon: float | None,
    target_lat: float | None,
    target_lon: float | None,
    setup_video_marks: dict[str, tuple[float, float]] | None = None,
    dem_path: str | Path | None = None,
    camera_hfov_deg: float = 62.0,
    vehicle_alt_msl_m: float | None = None,
) -> bool:
    """
    Re-place impact on the building facade when DEM collapsed it near the gun.

    Common when the gun is in an open field (foreground video pick) and impact
    is at the building base on the same column as the roof target.
    """
    if str(row.get("dooaf_role") or "") != DOOAF_ROLE_IMPACT:
        return False
    if gun_lat is None or gun_lon is None or target_lat is None or target_lon is None:
        return False
    pt = observation_target_latlon(row)
    if pt is None:
        return False
    gt = haversine_m(float(gun_lat), float(gun_lon), float(target_lat), float(target_lon))
    gi = haversine_m(float(gun_lat), float(gun_lon), pt[0], pt[1])
    if gt < 8.0 or gi > gt * 0.55:
        return False
    marks = normalize_dooaf_setup_video_marks(setup_video_marks) or {}
    if DOOAF_ROLE_INTENDED not in marks:
        return False
    try:
        vx_t, vy_t = marks[DOOAF_ROLE_INTENDED]
        vx_i = float(row.get("video_x_norm"))  # type: ignore[arg-type]
        vy_i = float(row.get("video_y_norm"))  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return False
    if abs(vx_i - float(vx_t)) > 0.16:
        return False
    if vy_i < float(vy_t) - 0.06:
        return False

    geo = _forced_ray_geo_for_row(
        row,
        dem_path=dem_path,
        camera_hfov_deg=camera_hfov_deg,
        vehicle_alt_msl_m=vehicle_alt_msl_m,
    )
    if geo is not None and geo.ok and geo.target_lat is not None and geo.target_lon is not None:
        new_gi = haversine_m(float(gun_lat), float(gun_lon), geo.target_lat, geo.target_lon)
        if new_gi > gi * 1.35 and new_gi <= gt * 1.05:
            ekf = observation_ekf_rel_alt_m(row)
            retry_agl = low_hover_ray_agl_m(ekf)
            _apply_geo_reference_to_mark_row(
                row,
                geo,
                ray_agl=retry_agl,
                ray_src="ray_facade_retry",
            )
            row["geo_quality"] = "fair"
            row["geo_method"] = "ray_facade_retry"
            row["geo_warning"] = (
                "impact geo from facade ray (corrected from gun-cluster collapse; "
                "hover ≥3 m for GPS-quality geo)"
            )
            return True

    dy = max(0.0, vy_i - float(vy_t))
    offset_m = min(10.0, max(2.5, gt * 0.08 + dy * 18.0))
    bearing_tg = initial_bearing_deg(
        float(target_lat), float(target_lon), float(gun_lat), float(gun_lon)
    )
    new_lat, new_lon = _destination_point_m(
        float(target_lat), float(target_lon), bearing_tg, offset_m
    )
    new_gi = haversine_m(float(gun_lat), float(gun_lon), new_lat, new_lon)
    if new_gi <= gi * 1.2 or new_gi > gt * 1.02:
        return False
    row["target_lat"] = new_lat
    row["target_lon"] = new_lon
    row["geo_quality"] = "fair"
    row["geo_method"] = "dooaf_facade_from_target"
    row["geo_warning"] = (
        "impact footprint from target facade offset "
        f"({offset_m:.0f} m toward gun — primary ray landed near artillery)"
    )
    from vgcs.observe.geo_reference import enrich_video_mark_target_altitude

    enrich_video_mark_target_altitude(row)
    row["geo_range_m"] = None
    row["geo_bearing_deg"] = None
    return True

def refine_impact_geo_from_video_rays(
    row: dict[str, Any],
    *,
    target_lat: float | None,
    target_lon: float | None,
    setup_video_marks: dict[str, tuple[float, float]] | None = None,
    dem_path: str | Path | None = None,
    camera_hfov_deg: float = 62.0,
    vehicle_alt_msl_m: float | None = None,
) -> bool:
    """
    Re-geo impact when it collapsed onto the setup target footprint but video Y differs.
    """
    if str(row.get("dooaf_role") or "") != DOOAF_ROLE_IMPACT:
        return False
    if target_lat is None or target_lon is None:
        return False
    pt = observation_target_latlon(row)
    if pt is None:
        return False
    if haversine_m(pt[0], pt[1], float(target_lat), float(target_lon)) > 2.5:
        return False
    marks = merge_setup_video_marks(setup_video_marks) or {}
    if DOOAF_ROLE_INTENDED not in marks:
        return False
    try:
        _vx_i, vy_i = marks[DOOAF_ROLE_INTENDED]
        vy_p = float(row.get("video_y_norm"))  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return False
    if vy_p >= float(vy_i) - 0.03:
        return False
    geo = _forced_ray_geo_for_row(
        row,
        dem_path=dem_path,
        camera_hfov_deg=camera_hfov_deg,
        vehicle_alt_msl_m=vehicle_alt_msl_m,
    )
    if geo is None or not geo.ok or geo.target_lat is None or geo.target_lon is None:
        return False
    ekf = observation_ekf_rel_alt_m(row)
    retry_agl = low_hover_ray_agl_m(ekf)
    _apply_geo_reference_to_mark_row(
        row,
        geo,
        ray_agl=retry_agl,
        ray_src="ray_facade_retry",
    )
    if observation_target_latlon(row) is None:
        return False
    row["geo_quality"] = "fair"
    row["geo_method"] = "ray_facade_retry"
    warn = (
        "impact geo from low-hover ray (distinct from target footprint; "
        "hover ≥3 m for GPS-quality geo)"
    )
    row["geo_warning"] = warn
    return True

def apply_dooaf_impact_geo_fallback(
    row: dict[str, Any],
    *,
    target_lat: float | None = None,
    target_lon: float | None = None,
    setup_video_marks: dict[str, tuple[float, float]] | None = None,
    dem_path: str | Path | None = None,
    camera_hfov_deg: float = 62.0,
    vehicle_alt_msl_m: float | None = None,
) -> bool:
    """
    Fill impact footprint when the primary DEM ray fails (common on-ground / low EKF).

    Tries a low-hover ray retry, then DOOAF Setup target footprint when video picks
    align on the same facade column.
    """
    if str(row.get("dooaf_role") or "") != DOOAF_ROLE_IMPACT:
        return False
    if observation_target_latlon(row) is not None:
        return False
    vx = row.get("video_x_norm")
    vy = row.get("video_y_norm")
    if vx is None or vy is None:
        return False

    marks = merge_setup_video_marks(setup_video_marks) or {}
    ekf = observation_ekf_rel_alt_m(row)
    retry_agl = low_hover_ray_agl_m(ekf)

    geo = _forced_ray_geo_for_row(
        row,
        dem_path=dem_path,
        camera_hfov_deg=camera_hfov_deg,
        vehicle_alt_msl_m=vehicle_alt_msl_m,
        force_agl_m=retry_agl,
    )
    if geo is not None and geo.ok and geo.target_lat is not None and geo.target_lon is not None:
        _apply_geo_reference_to_mark_row(
            row,
            geo,
            ray_agl=retry_agl,
            ray_src="ray_facade_retry",
        )
        row["geo_quality"] = "fair"
        row["geo_method"] = "ray_facade_retry"
        row["geo_warning"] = (
            "impact geo from low-hover ray retry "
            f"(EKF {ekf:.1f} m — arm and hover ≥3 m for better accuracy)"
            if ekf is not None and ekf < 2.5
            else str(geo.warning or "impact geo from ray retry")
        )
        return True

    if refine_impact_geo_from_video_rays(
        row,
        target_lat=target_lat,
        target_lon=target_lon,
        setup_video_marks=marks,
        dem_path=dem_path,
        camera_hfov_deg=camera_hfov_deg,
        vehicle_alt_msl_m=vehicle_alt_msl_m,
    ):
        return True

    if target_lat is None or target_lon is None:
        return False
    if DOOAF_ROLE_INTENDED not in marks:
        return False
    try:
        vx_i, vy_i = marks[DOOAF_ROLE_INTENDED]
        vx_p = float(vx)
        vy_p = float(vy)
    except (TypeError, ValueError):
        return False
    if abs(vx_p - float(vx_i)) > 0.18:
        return False
    if vy_p >= float(vy_i) + 0.08:
        return False

    row["target_lat"] = float(target_lat)
    row["target_lon"] = float(target_lon)
    row["geo_quality"] = "fair"
    row["geo_method"] = "dooaf_setup_target_footprint"
    row["geo_warning"] = (
        "impact footprint from DOOAF target (primary ray failed — "
        "hover ≥3 m with GPS for accurate ground geo)"
    )
    from vgcs.observe.geo_reference import enrich_video_mark_target_altitude

    enrich_video_mark_target_altitude(row)
    return True

def dooaf_export_blockers(
    rows: list[dict[str, Any]],
    *,
    gun_lat: float | None = None,
    gun_lon: float | None = None,
    target_lat: float | None = None,
    target_lon: float | None = None,
    setup_video_marks: dict[str, tuple[float, float]] | None = None,
    dem_path: str | Path | None = None,
) -> list[str]:
    """Human-readable export warnings after impact geo fallback attempts."""
    warnings: list[str] = []
    impact_row = latest_mark_row(rows, DOOAF_ROLE_IMPACT)
    if impact_row is None:
        return warnings
    if observation_target_latlon(impact_row) is None:
        apply_dooaf_impact_geo_fallback(
            impact_row,
            target_lat=target_lat,
            target_lon=target_lon,
            setup_video_marks=setup_video_marks,
            dem_path=dem_path,
        )
    if observation_target_latlon(impact_row) is None:
        warnings.append(
            "Impact Target has no map position (geo failed). "
            "Arm, hover at least 3 m, wait for GPS, then mark fall of shot again."
        )
        return warnings
    if gun_lat is None or gun_lon is None or target_lat is None or target_lon is None:
        warnings.append(
            "DOOAF Setup incomplete (gun and/or target missing). "
            "Fire correction will be partial."
        )
    q = str(impact_row.get("geo_quality") or "")
    if q in ("insufficient", ""):
        warnings.append(
            "Impact geo quality is low. Hover higher and re-mark for better correction."
        )
    elif q == "fair" and str(impact_row.get("geo_method") or "").startswith(
        ("dooaf_setup", "ray_facade_retry")
    ):
        warnings.append(
            "Impact position is estimated (video mark saved; map footprint approximate). "
            "Hover ≥3 m for GPS-quality geo."
        )
    ekf = observation_ekf_rel_alt_m(impact_row)
    if ekf is None:
        try:
            raw_ekf = float(impact_row.get("ekf_rel_alt_m"))  # type: ignore[arg-type]
            if math.isfinite(raw_ekf):
                ekf = raw_ekf
        except (TypeError, ValueError):
            pass
    ground_agl = _float_or_none(impact_row.get("dem_ground_agl_m"))
    if ekf is not None and ekf < 0.0:
        warnings.append(
            f"EKF relative altitude is negative ({ekf:.1f} m) — home position is wrong. "
            "Re-arm on site or set home at current location before DOOAF. "
            "Gun/target distances from video-only picks will be unreliable."
        )
    elif ekf is not None and ekf < 2.5:
        if ground_agl is not None and ground_agl > float(ekf) + 5.0:
            warnings.append(
                f"EKF reads {ekf:.1f} m above home but terrain AGL is ~{ground_agl:.0f} m "
                "(rooftop / raised surface). Horizontal geo for skyline picks is approximate."
            )
        else:
            warnings.append(
                f"Drone was near ground (EKF {ekf:.1f} m) when impact was marked."
            )
    if (
        gun_lat is not None
        and gun_lon is not None
        and target_lat is not None
        and target_lon is not None
        and setup_video_marks
    ):
        marks = merge_setup_video_marks(setup_video_marks) or {}
        if DOOAF_ROLE_GUN in marks and DOOAF_ROLE_INTENDED in marks:
            try:
                gx, gy = marks[DOOAF_ROLE_GUN]
                tx, ty = marks[DOOAF_ROLE_INTENDED]
                sep_m = haversine_m(
                    float(gun_lat),
                    float(gun_lon),
                    float(target_lat),
                    float(target_lon),
                )
                video_far = abs(float(gx) - float(tx)) > 0.10
                skyline_band = float(ty) < 0.58 and float(gy) < 0.65
                if video_far and skyline_band and sep_m < 8.0:
                    warnings.append(
                        "Gun and target look far apart in video but map distance is only "
                        f"{sep_m:.0f} m — distant hillside / skyline picks cannot be ranged "
                        "accurately from video alone. Place gun and target with map picks "
                        "(or survey coordinates) for reliable fire distance."
                    )
                elif sep_m < 8.0 and video_far:
                    warnings.append(
                        "Gun and target are only "
                        f"{sep_m:.0f} m apart on the map but video picks are far apart — "
                        "skyline/horizon clicks are approximate. Prefer map pick or hover ≥5 m."
                    )
            except (TypeError, ValueError):
                pass
    return warnings

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

def read_dooaf_setup_video_marks(st: Any) -> dict[str, tuple[float, float]]:
    """DOOAF Setup video picks persisted across export (role → norm x, y)."""
    raw = str(st.value(_QS_SETUP_VIDEO_MARKS_JSON, "") or "").strip()
    if not raw:
        return {}
    try:
        data = json.loads(raw)
    except (TypeError, ValueError, json.JSONDecodeError):
        return {}
    if not isinstance(data, dict):
        return {}
    out: dict[str, tuple[float, float]] = {}
    for role, pair in data.items():
        if not isinstance(pair, (list, tuple)) or len(pair) < 2:
            continue
        try:
            out[str(role)] = (float(pair[0]), float(pair[1]))
        except (TypeError, ValueError):
            continue
    return out

def write_dooaf_setup_video_marks(
    st: Any, marks: dict[str, tuple[float, float]]
) -> None:
    if not marks:
        st.remove(_QS_SETUP_VIDEO_MARKS_JSON)
        return
    payload = {str(role): [float(xy[0]), float(xy[1])] for role, xy in marks.items()}
    st.setValue(_QS_SETUP_VIDEO_MARKS_JSON, json.dumps(payload))

def write_dooaf_setup_video_mark(st: Any, role: str, x: float, y: float) -> None:
    marks = read_dooaf_setup_video_marks(st)
    marks[str(role)] = (float(x), float(y))
    write_dooaf_setup_video_marks(st, marks)

def clear_dooaf_setup_video_mark(st: Any, role: str) -> None:
    marks = read_dooaf_setup_video_marks(st)
    if marks.pop(str(role), None) is None:
        return
    write_dooaf_setup_video_marks(st, marks)

def read_dooaf_facade_slant_range_m(st: Any) -> float | None:
    """Last LRF facade-lock slant (m) from DOOAF Setup / rapid UV picks."""
    raw = st.value(_QS_FACADE_SLANT_M, None)
    if raw in (None, ""):
        return None
    try:
        slant = float(raw)
    except (TypeError, ValueError):
        return None
    return slant if slant >= 8.0 else None


def write_dooaf_facade_slant_range_m(st: Any, slant_m: float) -> None:
    try:
        slant = float(slant_m)
    except (TypeError, ValueError):
        return
    if slant < 8.0:
        return
    st.setValue(_QS_FACADE_SLANT_M, slant)


def clear_dooaf_facade_slant_range_m(st: Any) -> None:
    st.remove(_QS_FACADE_SLANT_M)


def facade_slant_from_rows(rows: list[dict[str, Any]] | None) -> float | None:
    if not rows:
        return None
    for row in rows:
        raw = row.get("lrf_slant_range_m")
        if raw is None:
            continue
        try:
            slant = float(raw)
        except (TypeError, ValueError):
            continue
        if slant >= 8.0:
            return slant
    return None


def resolve_facade_slant_range_m(
    rows: list[dict[str, Any]] | None = None,
    *,
    explicit: float | None = None,
    st: Any | None = None,
) -> float | None:
    """Facade LRF slant for distance math — explicit kwarg, rows, then QSettings."""
    if explicit is not None:
        try:
            slant = float(explicit)
        except (TypeError, ValueError):
            slant = None
        else:
            if slant >= 8.0:
                return slant
    found = facade_slant_from_rows(rows)
    if found is not None:
        return found
    if st is not None:
        return read_dooaf_facade_slant_range_m(st)
    try:
        from PySide6.QtCore import QSettings

        return read_dooaf_facade_slant_range_m(QSettings("VGCS", "VGCS"))
    except Exception:
        return None


def apply_facade_slant_to_mark_row(
    row: dict[str, Any],
    slant_m: float,
) -> None:
    if row.get("lrf_slant_range_m") is not None:
        return
    try:
        slant = float(slant_m)
    except (TypeError, ValueError):
        return
    if slant >= 8.0:
        row["lrf_slant_range_m"] = slant


def apply_facade_slant_to_mark_rows(
    slant_m: float,
    *rows: dict[str, Any] | None,
) -> None:
    for row in rows:
        if row is not None:
            apply_facade_slant_to_mark_row(row, slant_m)


def merge_setup_video_marks(
    memory: dict[str, tuple[float, float]] | None = None,
    *,
    st: Any | None = None,
) -> dict[str, tuple[float, float]] | None:
    """Merge persisted QSettings marks with in-memory DOOAF Setup picks."""
    merged: dict[str, tuple[float, float]] = {}
    store = st
    if store is None:
        try:
            from PySide6.QtCore import QSettings

            store = QSettings("VGCS", "VGCS")
        except Exception:
            store = None
    if store is not None:
        merged.update(read_dooaf_setup_video_marks(store))
    if memory:
        for role, pt in memory.items():
            merged[str(role)] = (float(pt[0]), float(pt[1]))
    return normalize_dooaf_setup_video_marks(merged)

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
    facade_slant_range_m: float | None = None,
) -> DooafSession:
    setup_video_marks = merge_setup_video_marks(setup_video_marks)
    if not _setup_video_marks_complete(setup_video_marks):
        setup_video_marks = merge_setup_video_marks(None) or setup_video_marks
    gun_row = latest_mark_row(rows, DOOAF_ROLE_GUN)
    intended_row = latest_mark_row(rows, DOOAF_ROLE_INTENDED)
    impact_row = latest_mark_row(rows, DOOAF_ROLE_IMPACT)
    resolved_slant = resolve_facade_slant_range_m(
        rows, explicit=facade_slant_range_m
    )
    if resolved_slant is not None:
        for row in rows:
            apply_facade_slant_to_mark_row(row, resolved_slant)
        if impact_row is not None:
            apply_facade_slant_to_mark_row(impact_row, resolved_slant)
    gun = latest_mark(rows, DOOAF_ROLE_GUN) or gun_from_settings(
        gun_lat=gun_lat, gun_lon=gun_lon, gun_alt_m=gun_alt_m
    )
    gun = _fill_point_alt_m(gun, gun_row, gun_alt_m, dem_path=dem_path)
    intended = latest_mark(rows, DOOAF_ROLE_INTENDED) or point_from_latlon(
        lat=target_lat, lon=target_lon, alt_m=target_alt_m
    )
    intended = _fill_point_alt_m(intended, intended_row, target_alt_m, dem_path=dem_path)
    impact = latest_mark(rows, DOOAF_ROLE_IMPACT)
    if impact is None and impact_row is not None:
        apply_dooaf_impact_geo_fallback(
            impact_row,
            target_lat=target_lat,
            target_lon=target_lon,
            setup_video_marks=setup_video_marks,
            dem_path=dem_path,
        )
        impact = latest_mark(rows, DOOAF_ROLE_IMPACT) or point_from_row(impact_row)
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
        setup_footprint = intended
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
        if setup_footprint is not None:
            intended = GeoPoint(
                setup_footprint.lat,
                setup_footprint.lon,
                setup_footprint.alt_m,
            )
    ground_row = gun_row
    if (
        ground_row is None
        and gun is not None
        and template_row is not None
        and setup_video_marks
        and DOOAF_ROLE_GUN in setup_video_marks
    ):
        gx, gy = setup_video_marks[DOOAF_ROLE_GUN]
        ground_row = _synthesize_setup_mark_row(
            DOOAF_ROLE_GUN,
            gun.lat,
            gun.lon,
            gx,
            gy,
            template_row,
            dem_path=dem_path,
            alt_m=gun.alt_m,
        )
    if resolved_slant is not None:
        apply_facade_slant_to_mark_rows(
            resolved_slant,
            ground_row,
            intended_row,
            impact_row,
            gun_row,
        )
    if impact_row is not None and template_row is not None:
        _enrich_mark_row_ray_geometry(
            impact_row, template_row, dem_path=dem_path
        )
    for setup_row in (ground_row, intended_row):
        if setup_row is not None and impact_row is not None:
            _enrich_mark_row_ray_geometry(
                setup_row, impact_row, dem_path=dem_path
            )
    if (
        impact_row is not None
        and gun is not None
        and intended is not None
    ):
        refine_impact_geo_artillery_field(
            impact_row,
            gun_lat=gun.lat,
            gun_lon=gun.lon,
            target_lat=intended.lat,
            target_lon=intended.lon,
            setup_video_marks=setup_video_marks,
            dem_path=dem_path,
            vehicle_alt_msl_m=(
                template_row.get("vehicle_alt_msl_m") if template_row else None
            ),  # type: ignore[union-attr]
        )
        imp_pt = observation_target_latlon(impact_row)
        if imp_pt is not None:
            impact = GeoPoint(
                imp_pt[0],
                imp_pt[1],
                impact.alt_m if impact is not None else None,
            )
            impact = _fill_point_alt_m(impact, impact_row, None, dem_path=dem_path)
    building_height_m: float | None = None
    if intended is not None and impact is not None:
        intended, impact, building_height_m = resolve_dooaf_mark_elevations(
            intended_row,
            impact_row,
            intended,
            impact,
            ground_row=ground_row,
        )
        hfov_resolve = 62.0
        for src in (impact_row, intended_row, ground_row):
            if src is not None and src.get("camera_hfov_deg") is not None:
                try:
                    hfov_resolve = float(src["camera_hfov_deg"])
                except (TypeError, ValueError):
                    pass
                break
        intended, impact = _apply_facade_vertical_to_points(
            intended,
            impact,
            intended_row=intended_row,
            impact_row=impact_row,
            hfov_deg=hfov_resolve,
        )
        if building_height_m is None and intended.alt_m is not None and impact.alt_m is not None:
            bh = abs(float(intended.alt_m) - float(impact.alt_m))
            if bh >= 0.05:
                building_height_m = bh
        if (
            gun is not None
            and ground_row is not None
            and ground_row.get("resolved_alt_msl_m") is not None
        ):
            try:
                gun = GeoPoint(
                    gun.lat,
                    gun.lon,
                    float(ground_row["resolved_alt_msl_m"]),  # type: ignore[arg-type]
                )
            except (TypeError, ValueError):
                pass
    correction = None
    if gun is not None and intended is not None and impact is not None:
        hfov = 62.0
        for src in (impact_row, intended_row, ground_row):
            if src is not None and src.get("camera_hfov_deg") is not None:
                try:
                    hfov = float(src["camera_hfov_deg"])
                except (TypeError, ValueError):
                    pass
                break
        correction = compute_fire_correction(
            gun,
            intended,
            impact,
            gun_row=ground_row,
            intended_row=intended_row,
            impact_row=impact_row,
            camera_hfov_deg=hfov,
        )
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

