"""
M8 — DOOAF (OO) orientation & geo-referencing (non-weapon).

Estimates a ground target lat/lon from vehicle pose, gimbal attitude, and a normalized
video click using a flat-earth ray–ground intersection (optional DEM offset).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from vgcs.observe.dem import (
    DemElevationModel,
    get_shared_dem_model,
    load_dem_model,
    ray_intersect_terrain_msl,
)
from vgcs.observe.target_measure import (
    dem_ground_agl_m,
    is_long_range_video_click,
    is_plausible_ground_range,
    prefer_dem_ground_agl_over_ekf,
    resolve_facade_ray_agl_m,
    sanitize_dem_ground_agl_m,
    slant_horizontal_range_m,
)

_EARTH_RADIUS_M = 6_371_000.0

# EMA for mark-track vehicle pose (lat/lon/heading) — tames GPS jitter in flight.
_MARK_TRACK_POSE_SMOOTH_ALPHA = 0.28


def smooth_vehicle_pose_ema(
    store: dict[str, object],
    *,
    vehicle_lat: float | None,
    vehicle_lon: float | None,
    vehicle_heading_deg: float | None,
    alpha: float = _MARK_TRACK_POSE_SMOOTH_ALPHA,
) -> tuple[float | None, float | None, float | None]:
    """Low-pass vehicle pose for stable world→video mark projection while airborne."""
    a = max(0.05, min(1.0, float(alpha)))
    out_lat: float | None = None
    out_lon: float | None = None
    out_hdg: float | None = None
    if vehicle_lat is not None:
        try:
            cur = float(vehicle_lat)
            prev = store.get("smooth_vehicle_lat")
            out_lat = cur if prev is None else (1.0 - a) * float(prev) + a * cur
            store["smooth_vehicle_lat"] = out_lat
        except (TypeError, ValueError):
            pass
    if vehicle_lon is not None:
        try:
            cur = float(vehicle_lon)
            prev = store.get("smooth_vehicle_lon")
            out_lon = cur if prev is None else (1.0 - a) * float(prev) + a * cur
            store["smooth_vehicle_lon"] = out_lon
        except (TypeError, ValueError):
            pass
    if vehicle_heading_deg is not None:
        try:
            cur = float(vehicle_heading_deg)
            prev = store.get("smooth_vehicle_heading_deg")
            if prev is None:
                out_hdg = cur
            else:
                diff = ((cur - float(prev) + 180.0) % 360.0) - 180.0
                out_hdg = float(prev) + a * diff
            store["smooth_vehicle_heading_deg"] = out_hdg
        except (TypeError, ValueError):
            pass
    return out_lat, out_lon, out_hdg


@dataclass(frozen=True)
class GeoReferenceResult:
    ok: bool
    target_lat: float | None = None
    target_lon: float | None = None
    target_alt_m: float | None = None
    horizontal_range_m: float | None = None
    depression_deg: float | None = None
    quality: str = "insufficient"
    warning: str = ""
    method: str = "none"
    bearing_deg: float | None = None


def _deg2rad(d: float) -> float:
    return math.radians(float(d))


def _rot_x(roll_rad: float) -> list[list[float]]:
    c, s = math.cos(roll_rad), math.sin(roll_rad)
    return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]


def _rot_y(pitch_rad: float) -> list[list[float]]:
    c, s = math.cos(pitch_rad), math.sin(pitch_rad)
    return [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]


def _rot_z(yaw_rad: float) -> list[list[float]]:
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


def _mat_mul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    out = [[0.0, 0.0, 0.0] for _ in range(3)]
    for i in range(3):
        for j in range(3):
            out[i][j] = sum(a[i][k] * b[k][j] for k in range(3))
    return out


def _mat_vec(m: list[list[float]], v: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    )


def _mat_transpose(m: list[list[float]]) -> list[list[float]]:
    return [[m[r][c] for r in range(3)] for c in range(3)]


def _delta_ned_m(
    vehicle_lat: float,
    vehicle_lon: float,
    vehicle_alt_m: float | None,
    target_lat: float,
    target_lon: float,
    target_alt_m: float | None,
) -> tuple[float, float, float]:
    """NED offset (m) from vehicle to target; down is positive."""
    lat_rad = _deg2rad(vehicle_lat)
    north_m = math.radians(float(target_lat) - float(vehicle_lat)) * _EARTH_RADIUS_M
    east_m = (
        math.radians(float(target_lon) - float(vehicle_lon))
        * _EARTH_RADIUS_M
        * max(1e-6, math.cos(lat_rad))
    )
    v_alt = float(vehicle_alt_m) if vehicle_alt_m is not None else 0.0
    t_alt = float(target_alt_m) if target_alt_m is not None else v_alt
    down_m = v_alt - t_alt
    return float(north_m), float(east_m), float(down_m)


def project_wgs84_to_video_norm(
    *,
    target_lat: float,
    target_lon: float,
    target_alt_m: float | None = None,
    vehicle_lat: float | None,
    vehicle_lon: float | None,
    vehicle_heading_deg: float | None,
    vehicle_roll_deg: float | None = None,
    vehicle_pitch_deg: float | None = None,
    vehicle_alt_msl_m: float | None = None,
    gimbal_yaw_deg: float | None,
    gimbal_pitch_deg: float | None,
    camera_hfov_deg: float = 83.4,
    camera_vfov_deg: float | None = None,
) -> tuple[float, float] | None:
    """Project a WGS84 point into normalized companion video coords (inverse of ray geo)."""
    if vehicle_lat is None or vehicle_lon is None:
        return None
    if gimbal_yaw_deg is None or gimbal_pitch_deg is None:
        return None
    north_m, east_m, down_m = _delta_ned_m(
        float(vehicle_lat),
        float(vehicle_lon),
        vehicle_alt_msl_m,
        float(target_lat),
        float(target_lon),
        target_alt_m,
    )
    horiz = math.hypot(north_m, east_m)
    dist = math.hypot(horiz, down_m)
    if dist < 0.5:
        return None
    dir_ned = (north_m / dist, east_m / dist, down_m / dist)

    roll = _deg2rad(float(vehicle_roll_deg or 0.0))
    pitch = _deg2rad(float(vehicle_pitch_deg or 0.0))
    hdg = _deg2rad(float(vehicle_heading_deg or 0.0))
    g_yaw = _deg2rad(float(gimbal_yaw_deg))
    g_pitch = _deg2rad(float(gimbal_pitch_deg))

    r_ned_body = _mat_mul(_rot_z(hdg), _mat_mul(_rot_y(pitch), _rot_x(roll)))
    r_body_gimbal = _mat_mul(_rot_z(g_yaw), _rot_y(g_pitch))
    r_ned_gimbal = _mat_mul(r_ned_body, r_body_gimbal)
    v_gimbal = _mat_vec(_mat_transpose(r_ned_gimbal), dir_ned)
    vx, vy, vz = float(v_gimbal[0]), float(v_gimbal[1]), float(v_gimbal[2])
    horiz_g = math.hypot(vx, vy)
    if horiz_g < 1e-4 or vx < 0.02:
        return None

    az_deg = math.degrees(math.atan2(vy, vx))
    el_deg = math.degrees(math.atan2(-vz, horiz_g))
    hfov = max(5.0, min(120.0, float(camera_hfov_deg)))
    vfov = float(camera_vfov_deg) if camera_vfov_deg is not None else hfov * 0.5625
    vfov = max(5.0, min(90.0, vfov))
    u = 0.5 + az_deg / hfov
    v = 0.5 + el_deg / vfov
    return (float(u), float(v))


def _offset_lat_lon(lat_deg: float, lon_deg: float, north_m: float, east_m: float) -> tuple[float, float]:
    lat_rad = _deg2rad(lat_deg)
    dlat = north_m / _EARTH_RADIUS_M
    dlon = east_m / (_EARTH_RADIUS_M * max(1e-6, math.cos(lat_rad)))
    return (lat_deg + math.degrees(dlat), lon_deg + math.degrees(dlon))


def _quality_label(
    *,
    gps_fix_type: int,
    gps_hdop: float | None,
    has_gimbal: bool,
    depression_deg: float,
    range_m: float,
) -> tuple[str, str]:
    warnings: list[str] = []
    if gps_fix_type < 3:
        warnings.append(f"GPS fix={gps_fix_type} (need 3D fix for best accuracy)")
    if gps_hdop is not None and gps_hdop > 2.0:
        warnings.append(f"GPS HDOP {gps_hdop:.1f} > 2.0")
    if not has_gimbal:
        warnings.append("gimbal attitude missing")
    if depression_deg < 5.0:
        warnings.append("look angle near horizon")
    if range_m > 5000.0:
        warnings.append("range > 5 km (flat-ground assumption weak)")

    if gps_fix_type < 2 or not has_gimbal or depression_deg < 1.0:
        return "insufficient", "; ".join(warnings) or "insufficient telemetry"

    if warnings:
        return "fair", "; ".join(warnings)
    if gps_fix_type >= 3 and (gps_hdop is None or gps_hdop <= 1.5) and depression_deg >= 15.0:
        return "good", ""
    return "fair", "; ".join(warnings)


def _resolve_dem_lookup(
    dem_path: str | Path | None,
    dem_lookup: Callable[[float, float], float | None] | None,
) -> DemElevationModel | None:
    if dem_lookup is not None:
        return DemElevationModel(kind="callback", path="", _fn=dem_lookup)
    if dem_path is None or not str(dem_path).strip():
        return None
    model = get_shared_dem_model(dem_path)
    if model is not None:
        return model
    return load_dem_model(dem_path)


def compute_geo_reference(
    *,
    vehicle_lat: float | None,
    vehicle_lon: float | None,
    vehicle_heading_deg: float | None,
    vehicle_roll_deg: float | None = None,
    vehicle_pitch_deg: float | None = None,
    vehicle_rel_alt_m: float | None,
    vehicle_alt_msl_m: float | None = None,
    rangefinder_down_m: float | None = None,
    gimbal_yaw_deg: float | None,
    gimbal_pitch_deg: float | None,
    video_x_norm: float,
    video_y_norm: float,
    gps_fix_type: int = 0,
    gps_hdop: float | None = None,
    camera_hfov_deg: float = 62.0,
    camera_vfov_deg: float | None = None,
    dem_path: str | Path | None = None,
    dem_lookup: Callable[[float, float], float | None] | None = None,
    dem_terrain: bool = True,
    force_agl_m: float | None = None,
) -> GeoReferenceResult:
    """
    Estimate ground intersection for a normalized video click (0..1, top-left origin).

    Body FRD (+X forward, +Y right, +Z down) and NED (+X north, +Y east, +Z down).
    Gimbal yaw about +Z, pitch about +Y (positive pitch = camera looks up).
    """
    if vehicle_lat is None or vehicle_lon is None:
        return GeoReferenceResult(ok=False, warning="vehicle position missing", method="none")
    agl_m: float | None = None
    agl_src = ""
    if force_agl_m is not None:
        try:
            agl_m = max(0.5, float(force_agl_m))
            agl_src = "forced_facade_retry"
        except (TypeError, ValueError):
            agl_m = None
    if agl_m is None:
        agl_m, agl_src = resolve_facade_ray_agl_m(
            relative_alt_m=vehicle_rel_alt_m,
            rangefinder_down_m=rangefinder_down_m,
            video_y_norm=video_y_norm,
        )
        dem_agl, dem_src = dem_ground_agl_m(
            vehicle_alt_msl_m=vehicle_alt_msl_m,
            vehicle_lat=vehicle_lat,
            vehicle_lon=vehicle_lon,
            dem_path=str(dem_path or "") if dem_path else None,
        )
        dem_agl = sanitize_dem_ground_agl_m(dem_agl, vehicle_rel_alt_m)
        agl_m, agl_src = prefer_dem_ground_agl_over_ekf(
            relative_alt_m=vehicle_rel_alt_m,
            facade_agl_m=agl_m,
            facade_src=agl_src,
            dem_ground_agl_m=dem_agl,
            dem_ground_src=dem_src,
        )
    if agl_m is None:
        return GeoReferenceResult(
            ok=False,
            warning="vehicle altitude AGL unknown (need EKF rel alt or downward rangefinder)",
            method="none",
        )
    long_range = is_long_range_video_click(
        video_y_norm, rangefinder_down_m, vehicle_rel_alt_m
    )
    gimbal_assumed = False
    if gimbal_yaw_deg is None and gimbal_pitch_deg is None:
        gimbal_yaw_deg = 0.0
        gimbal_pitch_deg = 0.0
        gimbal_assumed = True
    elif gimbal_yaw_deg is None:
        gimbal_yaw_deg = 0.0
        gimbal_assumed = True
    elif gimbal_pitch_deg is None:
        gimbal_pitch_deg = 0.0
        gimbal_assumed = True

    hfov = max(5.0, min(120.0, float(camera_hfov_deg)))
    vfov = float(camera_vfov_deg) if camera_vfov_deg is not None else hfov * 0.5625
    vfov = max(5.0, min(90.0, vfov))

    u = max(0.0, min(1.0, float(video_x_norm)))
    v = max(0.0, min(1.0, float(video_y_norm)))
    az_off = (u - 0.5) * hfov
    el_off = (v - 0.5) * vfov

    roll = _deg2rad(float(vehicle_roll_deg or 0.0))
    pitch = _deg2rad(float(vehicle_pitch_deg or 0.0))
    hdg = _deg2rad(float(vehicle_heading_deg or 0.0))
    g_yaw = _deg2rad(float(gimbal_yaw_deg))
    g_pitch_deg = float(gimbal_pitch_deg)
    pitch_assumed = False
    # C13/Skydroid often reports ~0° (level) while the scene is oblique; rangefinder
    # DOWN confirms we are low — use a typical downward look for near-wall geo only.
    if (
        not long_range
        and "rangefinder" in agl_src
        and (abs(g_pitch_deg) < 15.0 or g_pitch_deg > 10.0)
    ):
        g_pitch_deg = -35.0
        pitch_assumed = True
    # Missing gimbal or C13/Skydroid ~0° while scene is oblique: infer look from click.
    gimbal_pitch_unreliable = gimbal_assumed or abs(g_pitch_deg) < 15.0
    if (
        gimbal_pitch_unreliable
        and not long_range
        and float(video_y_norm) > 0.55
    ):
        el_click = (float(video_y_norm) - 0.5) * vfov
        g_pitch_deg = -min(55.0, max(12.0, el_click + 18.0))
        pitch_assumed = True
    elif (
        gimbal_pitch_unreliable
        and not long_range
        and float(agl_m) < 25.0
        and (abs(g_pitch_deg) < 15.0 or g_pitch_deg > 10.0)
    ):
        # EKF-only AGL (no rangefinder): level gimbal read still needs downward look.
        g_pitch_deg = -35.0
        pitch_assumed = True
    g_pitch = _deg2rad(g_pitch_deg)

    r_ned_body = _mat_mul(_rot_z(hdg), _mat_mul(_rot_y(pitch), _rot_x(roll)))
    r_body_gimbal = _mat_mul(_rot_z(g_yaw), _rot_y(g_pitch))
    r_gimbal_cam = _mat_mul(_rot_y(_deg2rad(el_off)), _rot_z(_deg2rad(az_off)))
    r_ned_cam = _mat_mul(r_ned_body, _mat_mul(r_body_gimbal, r_gimbal_cam))
    dir_ned = _mat_vec(r_ned_cam, (1.0, 0.0, 0.0))

    dz = dir_ned[2]
    if dz <= 1e-4:
        if long_range:
            horiz = math.hypot(dir_ned[0], dir_ned[1])
            if horiz < 1e-4:
                return GeoReferenceResult(
                    ok=False,
                    warning="look ray parallel to horizon",
                    method="ray_ground",
                )
            bearing = (math.degrees(math.atan2(dir_ned[1], dir_ned[0])) + 360.0) % 360.0
            dep_est = max(5.0, abs(float(el_off)) + 3.0)
            range_use = slant_horizontal_range_m(float(agl_m), dep_est)
            if range_use is None:
                range_use = min(350.0, float(agl_m) * 6.0)
            return GeoReferenceResult(
                ok=True,
                quality="fair",
                warning="distant target — horizon slant range (not ground GPS)",
                method="ray_slant_long_range",
                horizontal_range_m=range_use,
                bearing_deg=bearing,
                depression_deg=dep_est,
            )
        return GeoReferenceResult(
            ok=False,
            warning="look ray does not intersect ground (near horizon)",
            method="ray_ground",
        )

    agl_m = float(agl_m)
    method = "ray_ground_flat"
    if agl_src == "rangefinder_down":
        method = "ray_ground_rangefinder_agl"
    dem_model = _resolve_dem_lookup(dem_path, dem_lookup)
    lookup = dem_model.elevation_m if dem_model is not None else None

    mag = math.hypot(dir_ned[0], dir_ned[1], dir_ned[2])
    dir_unit = (
        (dir_ned[0] / mag, dir_ned[1] / mag, dir_ned[2] / mag) if mag > 1e-9 else dir_ned
    )

    north_m: float | None = None
    east_m: float | None = None
    range_m: float | None = None
    tgt_alt: float | None = None
    terrain_hit = False

    if (
        bool(dem_terrain)
        and lookup is not None
        and vehicle_alt_msl_m is not None
    ):
        hit = ray_intersect_terrain_msl(
            vehicle_lat=float(vehicle_lat),
            vehicle_lon=float(vehicle_lon),
            vehicle_alt_msl_m=float(vehicle_alt_msl_m),
            dir_ned=dir_unit,
            elevation_m=lookup,
            max_range_m=min(5000.0, max(80.0, float(agl_m) * 400.0)),
            step_m=max(1.0, min(8.0, float(agl_m) / 4.0)),
        )
        if hit is not None:
            north_m, east_m, range_m, tgt_alt = hit
            terrain_hit = True
            method = "ray_terrain_dem"

    if not terrain_hit:
        ground_z_ned = agl_m
        if lookup is not None and vehicle_alt_msl_m is not None:
            try:
                elev = lookup(float(vehicle_lat), float(vehicle_lon))
                if elev is not None:
                    ground_z_ned = max(0.5, float(vehicle_alt_msl_m) - float(elev))
                    method = "ray_ground_dem"
            except Exception:
                pass
        t = ground_z_ned / dz
        north_m = t * dir_ned[0]
        east_m = t * dir_ned[1]
        range_m = math.hypot(north_m, east_m)

    bearing = (math.degrees(math.atan2(east_m, north_m)) + 360.0) % 360.0
    depression = math.degrees(math.atan2(dz, math.hypot(dir_ned[0], dir_ned[1])))

    if not is_plausible_ground_range(agl_m, range_m, depression):
        if long_range:
            range_use = slant_horizontal_range_m(agl_m, depression) or min(
                280.0, max(float(agl_m) * 2.0, range_m * 0.15)
            )
            quality, warn = _quality_label(
                gps_fix_type=int(gps_fix_type or 0),
                gps_hdop=gps_hdop,
                has_gimbal=True,
                depression_deg=depression,
                range_m=range_use,
            )
            extra = "distant target — slant range estimate (not ground GPS)"
            warn = f"{warn}; {extra}" if warn else extra
            if pitch_assumed:
                extra2 = "gimbal pitch assumed -35° (sensor read ~0°)"
                warn = f"{warn}; {extra2}" if warn else extra2
            return GeoReferenceResult(
                ok=True,
                quality=quality if quality != "insufficient" else "fair",
                warning=warn,
                method="ray_slant_long_range",
                horizontal_range_m=range_use,
                bearing_deg=bearing,
                depression_deg=depression,
            )
        return GeoReferenceResult(
            ok=False,
            warning=(
                f"computed ground range {range_m:.0f} m is unrealistic for {agl_m:.1f} m height "
                "(click on ground in lower video, pitch gimbal down; wall/horizon marks are not accurate)"
            ),
            method=method,
            horizontal_range_m=range_m,
            bearing_deg=bearing,
        )

    tgt_lat, tgt_lon = _offset_lat_lon(float(vehicle_lat), float(vehicle_lon), north_m, east_m)
    if tgt_alt is None and lookup is not None:
        try:
            tgt_alt = lookup(tgt_lat, tgt_lon)
        except Exception:
            tgt_alt = None
    if tgt_alt is None and vehicle_alt_msl_m is not None:
        tgt_alt = float(vehicle_alt_msl_m) - agl_m

    quality, warn = _quality_label(
        gps_fix_type=int(gps_fix_type or 0),
        gps_hdop=gps_hdop,
        has_gimbal=not gimbal_assumed or pitch_assumed,
        depression_deg=depression,
        range_m=range_m,
    )
    if gimbal_assumed and not pitch_assumed:
        extra = "gimbal attitude assumed level (0°, 0°)"
        warn = f"{warn}; {extra}" if warn else extra
    if pitch_assumed:
        extra = "gimbal pitch estimated from video click (sensor missing or ~0°)"
        warn = f"{warn}; {extra}" if warn else extra
    if terrain_hit and dem_model is not None:
        extra = f"terrain DEM ({dem_model.kind})"
        warn = f"{warn}; {extra}" if warn else extra
    if agl_m >= 70.0 or (depression is not None and float(depression) >= 50.0):
        extra = (
            "steep look / high AGL — ground geo less accurate "
            "(click low in frame, use rangefinder if available)"
        )
        warn = f"{warn}; {extra}" if warn else extra
        if quality == "good":
            quality = "fair"
    ok = quality != "insufficient"
    return GeoReferenceResult(
        ok=ok,
        target_lat=tgt_lat,
        target_lon=tgt_lon,
        target_alt_m=tgt_alt,
        horizontal_range_m=range_m,
        depression_deg=depression,
        quality=quality,
        warning=warn,
        method=method,
        bearing_deg=bearing,
    )


def compute_lrf_slant_geo(
    *,
    vehicle_lat: float | None,
    vehicle_lon: float | None,
    vehicle_heading_deg: float | None,
    vehicle_roll_deg: float | None = None,
    vehicle_pitch_deg: float | None = None,
    vehicle_alt_msl_m: float | None = None,
    gimbal_yaw_deg: float | None,
    gimbal_pitch_deg: float | None,
    slant_range_m: float,
    video_x_norm: float = 0.5,
    video_y_norm: float = 0.5,
    gps_fix_type: int = 0,
    gps_hdop: float | None = None,
    camera_hfov_deg: float = 83.4,
    camera_vfov_deg: float | None = None,
) -> GeoReferenceResult:
    """
    Ground lat/lon from C13 LRF slant range along the laser line-of-sight.

    Uses vehicle pose + gimbal attitude + measured slant range (not ray–ground guess).
    """
    if vehicle_lat is None or vehicle_lon is None:
        return GeoReferenceResult(ok=False, warning="vehicle position missing", method="none")
    try:
        slant = float(slant_range_m)
    except (TypeError, ValueError):
        return GeoReferenceResult(ok=False, warning="invalid LRF range", method="none")
    if slant < 0.5:
        return GeoReferenceResult(ok=False, warning="LRF range too short", method="none")

    gimbal_assumed = False
    gy = float(gimbal_yaw_deg) if gimbal_yaw_deg is not None else 0.0
    gp = float(gimbal_pitch_deg) if gimbal_pitch_deg is not None else 0.0
    if gimbal_yaw_deg is None or gimbal_pitch_deg is None:
        gimbal_assumed = True

    hfov = max(5.0, min(120.0, float(camera_hfov_deg)))
    vfov = float(camera_vfov_deg) if camera_vfov_deg is not None else hfov * 0.5625
    vfov = max(5.0, min(90.0, vfov))
    u = max(0.0, min(1.0, float(video_x_norm)))
    v = max(0.0, min(1.0, float(video_y_norm)))
    az_off = (u - 0.5) * hfov
    el_off = (v - 0.5) * vfov

    roll = _deg2rad(float(vehicle_roll_deg or 0.0))
    pitch = _deg2rad(float(vehicle_pitch_deg or 0.0))
    hdg = _deg2rad(float(vehicle_heading_deg or 0.0))
    g_yaw = _deg2rad(gy)
    g_pitch = _deg2rad(gp)

    r_ned_body = _mat_mul(_rot_z(hdg), _mat_mul(_rot_y(pitch), _rot_x(roll)))
    r_body_gimbal = _mat_mul(_rot_z(g_yaw), _rot_y(g_pitch))
    r_gimbal_cam = _mat_mul(_rot_y(_deg2rad(el_off)), _rot_z(_deg2rad(az_off)))
    r_ned_cam = _mat_mul(r_ned_body, _mat_mul(r_body_gimbal, r_gimbal_cam))
    dir_ned = _mat_vec(r_ned_cam, (1.0, 0.0, 0.0))
    mag = math.hypot(dir_ned[0], dir_ned[1], dir_ned[2])
    if mag < 1e-9:
        return GeoReferenceResult(ok=False, warning="invalid look direction", method="lrf_slant")
    dir_unit = (dir_ned[0] / mag, dir_ned[1] / mag, dir_ned[2] / mag)

    north_m = slant * dir_unit[0]
    east_m = slant * dir_unit[1]
    down_m = slant * dir_unit[2]
    horiz = math.hypot(north_m, east_m)
    depression = math.degrees(math.atan2(down_m, max(1e-6, horiz)))
    bearing = (math.degrees(math.atan2(east_m, north_m)) + 360.0) % 360.0
    tgt_lat, tgt_lon = _offset_lat_lon(float(vehicle_lat), float(vehicle_lon), north_m, east_m)
    tgt_alt: float | None = None
    if vehicle_alt_msl_m is not None:
        tgt_alt = float(vehicle_alt_msl_m) - down_m

    quality, warn = _quality_label(
        gps_fix_type=int(gps_fix_type or 0),
        gps_hdop=gps_hdop,
        has_gimbal=not gimbal_assumed,
        depression_deg=depression,
        range_m=horiz,
    )
    if gimbal_assumed:
        extra = "gimbal attitude assumed level (0°, 0°)"
        warn = f"{warn}; {extra}" if warn else extra
    if depression < 3.0:
        extra = "near-horizon LRF — ground geo less accurate"
        warn = f"{warn}; {extra}" if warn else extra
        if quality == "good":
            quality = "fair"

    return GeoReferenceResult(
        ok=quality != "insufficient",
        target_lat=tgt_lat,
        target_lon=tgt_lon,
        target_alt_m=tgt_alt,
        horizontal_range_m=horiz,
        depression_deg=depression,
        quality=quality,
        warning=warn,
        method="lrf_slant",
        bearing_deg=bearing,
    )


def enrich_video_mark_target_altitude(row: dict[str, object]) -> None:
    """
    Resolve ``target_alt_m`` for video marks: DEM ground vs ray-derived facade height.

    Stores ``target_alt_m_dem``, ``target_alt_m_ray``, and ``target_alt_method``.
    """
    from vgcs.observe.facade_plane import (
        infer_elevated_click_target_msl_from_row,
        infer_ray_target_msl_from_row,
    )

    dem_alt = row.get("target_alt_m")
    try:
        dem_val = float(dem_alt) if dem_alt is not None else None
    except (TypeError, ValueError):
        dem_val = None
    row["target_alt_m_dem"] = dem_val

    ray_alt = infer_ray_target_msl_from_row(row)  # type: ignore[arg-type]
    row["target_alt_m_ray"] = ray_alt

    hfov = 62.0
    try:
        if row.get("camera_hfov_deg") is not None:
            hfov = float(row.get("camera_hfov_deg"))
    except (TypeError, ValueError):
        pass
    elevated_alt = infer_elevated_click_target_msl_from_row(
        row,  # type: ignore[arg-type]
        hfov_deg=hfov,
    )
    row["target_alt_m_elevated"] = elevated_alt

    method = "terrain_dem"
    resolved: float | None = dem_val

    try:
        y_norm = float(row.get("video_y_norm")) if row.get("video_y_norm") is not None else 0.55
    except (TypeError, ValueError):
        y_norm = 0.55

    if elevated_alt is not None and dem_val is not None and y_norm < 0.54:
        resolved = elevated_alt
        method = "video_facade_elevated"
    elif ray_alt is not None and dem_val is not None:
        delta = ray_alt - dem_val
        if delta > 1.0 and y_norm < 0.52:
            resolved = ray_alt
            method = "ray_elevated"
        elif delta > 1.5:
            resolved = ray_alt
            method = "ray_facade"
        elif abs(delta) <= 1.5 and y_norm >= 0.48:
            resolved = dem_val
            method = "terrain_dem"
    elif ray_alt is not None and dem_val is None:
        resolved = ray_alt
        method = "ray_slant"

    row["target_alt_m"] = resolved
    row["target_alt_method"] = method


def apply_geo_reference_result_to_video_row(
    row: dict[str, object],
    geo: GeoReferenceResult,
    *,
    slant_range_m: float | None = None,
) -> None:
    """Copy a geo result onto a video mark / DOOAF pick row."""
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
    if geo.horizontal_range_m is not None:
        row["geo_range_m"] = geo.horizontal_range_m
    else:
        row["geo_range_m"] = None
    if geo.bearing_deg is not None:
        row["geo_bearing_deg"] = geo.bearing_deg
    else:
        row["geo_bearing_deg"] = None
    if slant_range_m is not None:
        try:
            row["lrf_slant_range_m"] = float(slant_range_m)
        except (TypeError, ValueError):
            row["lrf_slant_range_m"] = None
    enrich_video_mark_target_altitude(row)  # type: ignore[arg-type]
