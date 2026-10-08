"""
M8 — DOOAF (OO) orientation & geo-referencing (non-weapon).

Estimates a ground target lat/lon from vehicle pose, gimbal attitude, and a normalized
video click using a flat-earth ray–ground intersection (optional DEM offset).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Callable

from vgcs.observe.dem import (
    DemElevationModel,
    get_shared_dem_model,
    load_dem_model,
    ray_intersect_terrain_msl,
)
from vgcs.observe.target_measure import (
    MIN_FACADE_AGL_M,
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

# Below this height the ground point of a click is worked out as it was tuned
# on the bench and in the low hover: the look angle may be guessed
# (compute_geo_reference). From here up, above the take-off point, the drone
# flies, and a camera that says its angle is believed.
_GUESS_LOOK_BELOW_M = 25.0


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
    # False: the point rests on a guess (of the camera's angle) or on a height
    # that was not measured. It may be drawn and measured against on the
    # video. It is no DOOAF point: no target, no fall of shot, no correction.
    measured: bool = True
    # Why not, in the words of the log, and how far under the horizon the
    # click really looks when that is known (dooaf_popup.why_not_placed).
    not_measured_why: str = ""
    not_measured_look_deg: float | None = None


def _deg2rad(d: float) -> float:
    return math.radians(float(d))


def _gimbal_yaw_right_deg(gimbal_yaw_deg: float, left_positive: bool | None) -> float:
    """The gimbal yaw as the math below needs it: to the right of the nose is positive.

    ``left_positive`` says how the camera reports it (GimbalStatus.yaw_left_positive).
    The Skydroid C12 and C13 count a turn to the left as positive.
    """
    return -float(gimbal_yaw_deg) if left_positive else float(gimbal_yaw_deg)


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
    gimbal_yaw_left_positive: bool | None = None,
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
    g_yaw = _deg2rad(_gimbal_yaw_right_deg(float(gimbal_yaw_deg), gimbal_yaw_left_positive))
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
    # A point above the cross is higher in the picture, and the picture's v
    # counts from the top (see _click_tilt_deg).
    v = 0.5 - el_deg / vfov
    return (float(u), float(v))


def _click_tilt_deg(video_y_norm: float, vfov_deg: float) -> float:
    """How far a click looks UP from the cross, in degrees (down is negative).

    The picture's v counts from the top, so a click below the cross (v over
    0.5) looks further down. Until 2026-10-08 this was the other way round in
    the lat long math: a click below the cross was worked out as a look
    further UP. On the ground that put a lower click further away, on a wall
    higher up, and a mark drawn back on the video on the wrong side of the
    cross when the camera tilted. The aiming and the mark overlay, tuned on
    the camera in the field, always had it right (adapter._gimbal_pitch_target_deg,
    lrf_track_uv_from_attitude), and the cross itself was never affected.
    """
    return -(float(video_y_norm) - 0.5) * float(vfov_deg)


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
    range_is_measured: bool = False,
) -> tuple[str, str]:
    """Grade a geo fix. ``range_is_measured`` when the distance came from a
    rangefinder rather than from intersecting the look ray with the ground.

    That distinction decides whether a shallow look angle is disqualifying.
    Intersecting a ray with the ground divides by tan(depression), so near the
    horizon the answer explodes — at 3 m AGL, 0.3° vs 0.5° of depression moves
    the estimate by ~230 m, which is why anything under 1° is refused outright.
    A MEASURED range has no such term: the target sits at a known distance
    along the ray, and the same 0.3°-to-0.5° change moves it by well under a
    millimetre. Applying the ray-method's gate to a laser measurement threw
    away perfectly good fixes (field-observed 2026-08-17: a drone at 2.99 m AGL
    ranging a wall at 10-22 m could not place a DOOAF point at all, despite a
    3D GPS fix, 16 satellites and the laser returning clean ranges).
    """
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

    shallow_is_fatal = (not range_is_measured) and depression_deg < 1.0
    if gps_fix_type < 2 or not has_gimbal or shallow_is_fatal:
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


# Where the height above the ground comes from, when it is a measured one.
_MEASURED_HEIGHT_SOURCES = ("ekf_relative", "rangefinder_down", "rangefinder_down_facade")
_MEASURED_HEIGHT_SOURCES_IN_THE_AIR = ("dem_terrain", "dem_terrain_cached", "forced_facade_retry")


def _why_not_measured(
    *,
    gimbal_assumed: bool,
    long_range: bool,
    agl_m: float,
    agl_src: str,
    rel_alt_m: float | None,
) -> str:
    """Why the point of a click cannot be a measurement, or "" when it can.

    It takes the camera's own angle and the drone's own height. A height is
    measured when it comes from the autopilot's height above the take-off
    point or from a rangefinder that looks down and is not at its limit, and
    is at least the height of a hover (MIN_FACADE_AGL_M). A height from the
    terrain file, or one put in for a second try, counts when the drone is
    that high above its take-off point as well: for a drone that stands on
    the ground those are 12 m or 3 m that nobody measured.
    """
    if gimbal_assumed:
        return "camera angle not reported"
    if long_range:
        return "height above ground not measured (rangefinder at its limit)"
    in_the_air = rel_alt_m is not None and rel_alt_m >= MIN_FACADE_AGL_M
    known = agl_src in _MEASURED_HEIGHT_SOURCES or (
        agl_src in _MEASURED_HEIGHT_SOURCES_IN_THE_AIR and in_the_air
    )
    if not known or float(agl_m) < MIN_FACADE_AGL_M:
        return f"height above ground not measured ({float(agl_m):.1f} m, {agl_src or 'unknown'})"
    return ""


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
    gimbal_yaw_left_positive: bool | None = None,
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
    lens_hfov_deg: float | None = None,
    lens_vfov_deg: float | None = None,
) -> GeoReferenceResult:
    """
    Estimate ground intersection for a normalized video click (0..1, top-left origin).

    Body FRD (+X forward, +Y right, +Z down) and NED (+X north, +Y east, +Z down).
    Gimbal yaw about +Z, pitch about +Y (positive pitch = camera looks up).

    ``gimbal_yaw_deg`` is the camera's own number. ``gimbal_yaw_left_positive``
    says which way it counts (GimbalStatus.yaw_left_positive): the Skydroid
    C12 and C13 count a turn to the left as positive.

    ``lens_hfov_deg`` and ``lens_vfov_deg``: the camera's own lens at its
    zoom, when it is known. A measured point is worked out through it.
    ``camera_hfov_deg`` is the "Camera HFOV" setting, which the bench's way
    and the measuring marks were tuned with.

    The result says whether its point is measured (``measured``): the
    camera's own angle and a measured height. A point that is not may be
    drawn and measured against on the video, and is no DOOAF point.
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
    lens: tuple[float, float] | None = None
    try:
        if lens_hfov_deg is not None and lens_vfov_deg is not None:
            lens = (
                max(5.0, min(120.0, float(lens_hfov_deg))),
                max(5.0, min(90.0, float(lens_vfov_deg))),
            )
    except (TypeError, ValueError):
        lens = None

    roll = _deg2rad(float(vehicle_roll_deg or 0.0))
    pitch = _deg2rad(float(vehicle_pitch_deg or 0.0))
    hdg = _deg2rad(float(vehicle_heading_deg or 0.0))
    g_yaw = _deg2rad(_gimbal_yaw_right_deg(float(gimbal_yaw_deg), gimbal_yaw_left_positive))
    used_guess = [False]

    def solve(believe_camera: bool) -> GeoReferenceResult:
        """The point of the click, worked out in one of two ways.

        ``believe_camera``: the camera's own angle, and a click below the
        cross looks further down. Otherwise the way it was tuned on the
        bench in June to August: a camera angle within 15 degrees of level
        is replaced by 18 or 35 degrees (the C13 was seen to say 0 while it
        looked down), and the click's tilt has the reversed sign. That way
        gives no measurement. It is kept for the measuring marks on the
        video, whose results were tuned with it.
        """
        agl = float(agl_m)
        g_pitch_deg = float(gimbal_pitch_deg)
        pitch_assumed = False
        if not believe_camera:
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
                and float(agl) < _GUESS_LOOK_BELOW_M
                and (abs(g_pitch_deg) < 15.0 or g_pitch_deg > 10.0)
            ):
                # EKF-only AGL (no rangefinder): level gimbal read still needs downward look.
                g_pitch_deg = -35.0
                pitch_assumed = True
        g_pitch = _deg2rad(g_pitch_deg)
        # The click's own tilt from the cross: up is up (_click_tilt_deg) with the
        # camera's own angle. The bench's way keeps the reversed sign that its
        # guesses and its measuring results were tuned with.
        el_tilt = _click_tilt_deg(v, vfov) if believe_camera else el_off
        az_turn = az_off
        if believe_camera and lens is not None:
            # How far the click is from the cross, through the camera's own
            # lens. The setting that is used otherwise is 62 degrees unless
            # somebody changed it, and the C13's lens is 83 degrees wide.
            az_turn = (u - 0.5) * lens[0]
            el_tilt = _click_tilt_deg(v, lens[1])
        used_guess[0] = bool(gimbal_assumed or pitch_assumed)

        r_ned_body = _mat_mul(_rot_z(hdg), _mat_mul(_rot_y(pitch), _rot_x(roll)))
        r_body_gimbal = _mat_mul(_rot_z(g_yaw), _rot_y(g_pitch))
        r_gimbal_cam = _mat_mul(_rot_y(_deg2rad(el_tilt)), _rot_z(_deg2rad(az_turn)))
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
                range_use = slant_horizontal_range_m(float(agl), dep_est)
                if range_use is None:
                    range_use = min(350.0, float(agl) * 6.0)
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
                max_range_m=min(5000.0, max(80.0, float(agl) * 400.0)),
                step_m=max(1.0, min(8.0, float(agl) / 4.0)),
            )
            if hit is not None:
                north_m, east_m, range_m, tgt_alt = hit
                terrain_hit = True
                method = "ray_terrain_dem"

        if not terrain_hit:
            ground_z_ned = agl
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

        if not is_plausible_ground_range(agl, range_m, depression):
            if long_range:
                range_use = slant_horizontal_range_m(agl, depression) or min(
                    280.0, max(float(agl) * 2.0, range_m * 0.15)
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
                    f"computed ground range {range_m:.0f} m is unrealistic for {agl:.1f} m height "
                    "(click on ground in lower video, pitch gimbal down; wall/horizon marks are not accurate)"
                ),
                method=method,
                horizontal_range_m=range_m,
                bearing_deg=bearing,
                # The angle goes with the refusal: it is the reason, and the
                # operator is told it (dooaf_popup.why_not_placed).
                depression_deg=depression,
            )

        tgt_lat, tgt_lon = _offset_lat_lon(float(vehicle_lat), float(vehicle_lon), north_m, east_m)
        if tgt_alt is None and lookup is not None:
            try:
                tgt_alt = lookup(tgt_lat, tgt_lon)
            except Exception:
                tgt_alt = None
        if tgt_alt is None and vehicle_alt_msl_m is not None:
            tgt_alt = float(vehicle_alt_msl_m) - agl

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
        if agl >= 70.0 or (depression is not None and float(depression) >= 50.0):
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

    # Which of the two counts.
    #
    # A point is measured when the camera says its angle and the drone's height
    # above the ground is a measured one. Then the camera's way is the answer,
    # also when the answer is that there is no point. Below the flying height
    # the bench's way is still given when it has a guess to offer, marked as
    # not measured: the measuring marks on the video live on it. In a flight a
    # guess is no use to anyone.
    try:
        rel_alt_m = float(vehicle_rel_alt_m) if vehicle_rel_alt_m is not None else None
    except (TypeError, ValueError):
        rel_alt_m = None
    why_not = _why_not_measured(
        gimbal_assumed=gimbal_assumed,
        long_range=long_range,
        agl_m=float(agl_m),
        agl_src=agl_src,
        rel_alt_m=rel_alt_m,
    )
    if not why_not:
        measured = solve(True)
        flying = rel_alt_m is not None and rel_alt_m >= _GUESS_LOOK_BELOW_M
        if measured.target_lat is not None or flying:
            return measured
        tuned = solve(False)
        if not used_guess[0]:
            return measured
        return replace(
            tuned,
            measured=False,
            not_measured_why=measured.warning,
            not_measured_look_deg=measured.depression_deg,
        )
    return replace(solve(False), measured=False, not_measured_why=why_not)


def _lrf_camera_dir_ned_unit(
    *,
    vehicle_heading_deg: float | None,
    vehicle_roll_deg: float | None = None,
    vehicle_pitch_deg: float | None = None,
    gimbal_yaw_deg: float | None,
    gimbal_pitch_deg: float | None,
    gimbal_yaw_left_positive: bool | None = None,
    video_x_norm: float = 0.5,
    video_y_norm: float = 0.5,
    camera_hfov_deg: float = 83.4,
    camera_vfov_deg: float | None = None,
) -> tuple[tuple[float, float, float], bool] | None:
    """Unit camera look direction in NED (north, east, down)."""
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
    el_tilt = _click_tilt_deg(v, vfov)

    roll = _deg2rad(float(vehicle_roll_deg or 0.0))
    pitch = _deg2rad(float(vehicle_pitch_deg or 0.0))
    hdg = _deg2rad(float(vehicle_heading_deg or 0.0))
    g_yaw = _deg2rad(_gimbal_yaw_right_deg(gy, gimbal_yaw_left_positive))
    g_pitch = _deg2rad(gp)

    r_ned_body = _mat_mul(_rot_z(hdg), _mat_mul(_rot_y(pitch), _rot_x(roll)))
    r_body_gimbal = _mat_mul(_rot_z(g_yaw), _rot_y(g_pitch))
    r_gimbal_cam = _mat_mul(_rot_y(_deg2rad(el_tilt)), _rot_z(_deg2rad(az_off)))
    r_ned_cam = _mat_mul(r_ned_body, _mat_mul(r_body_gimbal, r_gimbal_cam))
    dir_ned = _mat_vec(r_ned_cam, (1.0, 0.0, 0.0))
    mag = math.hypot(dir_ned[0], dir_ned[1], dir_ned[2])
    if mag < 1e-9:
        return None
    dir_unit = (dir_ned[0] / mag, dir_ned[1] / mag, dir_ned[2] / mag)
    return dir_unit, gimbal_assumed


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
    gimbal_yaw_left_positive: bool | None = None,
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

    ray = _lrf_camera_dir_ned_unit(
        vehicle_heading_deg=vehicle_heading_deg,
        vehicle_roll_deg=vehicle_roll_deg,
        vehicle_pitch_deg=vehicle_pitch_deg,
        gimbal_yaw_deg=gimbal_yaw_deg,
        gimbal_yaw_left_positive=gimbal_yaw_left_positive,
        gimbal_pitch_deg=gimbal_pitch_deg,
        video_x_norm=video_x_norm,
        video_y_norm=video_y_norm,
        camera_hfov_deg=camera_hfov_deg,
        camera_vfov_deg=camera_vfov_deg,
    )
    if ray is None:
        return GeoReferenceResult(ok=False, warning="invalid look direction", method="lrf_slant")
    dir_unit, gimbal_assumed = ray

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
        # Laser measurement — a shallow look angle does not degrade it.
        range_is_measured=True,
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


def compute_lrf_facade_plane_geo(
    *,
    vehicle_lat: float | None,
    vehicle_lon: float | None,
    vehicle_heading_deg: float | None,
    vehicle_roll_deg: float | None = None,
    vehicle_pitch_deg: float | None = None,
    vehicle_alt_msl_m: float | None = None,
    gimbal_yaw_deg: float | None,
    gimbal_pitch_deg: float | None,
    gimbal_yaw_left_positive: bool | None = None,
    slant_range_m: float,
    video_x_norm: float = 0.5,
    video_y_norm: float = 0.5,
    gps_fix_type: int = 0,
    gps_hdop: float | None = None,
    camera_hfov_deg: float = 83.4,
    camera_vfov_deg: float | None = None,
    boresight_u: float = 0.5,
    boresight_v: float = 0.5,
) -> GeoReferenceResult:
    """
    Map a video UV to lat/lon on a vertical facade plane anchored by LRF at boresight.

    The LRF lock at crosshair defines a 3D anchor on the wall at ``slant_range_m``.
    A vertical plane through that anchor (normal = horizontal look-back toward the
    drone) is intersected with the click ray.  This yields wall coordinates instead
    of a ground footprint when the camera is near the horizon.
    """
    method = "lrf_facade_plane"
    if vehicle_lat is None or vehicle_lon is None:
        return GeoReferenceResult(ok=False, warning="vehicle position missing", method=method)
    try:
        slant = float(slant_range_m)
    except (TypeError, ValueError):
        return GeoReferenceResult(ok=False, warning="invalid LRF range", method=method)
    if slant < 0.5:
        return GeoReferenceResult(ok=False, warning="LRF range too short", method=method)

    pose_kw = dict(
        vehicle_heading_deg=vehicle_heading_deg,
        vehicle_roll_deg=vehicle_roll_deg,
        vehicle_pitch_deg=vehicle_pitch_deg,
        gimbal_yaw_deg=gimbal_yaw_deg,
        gimbal_yaw_left_positive=gimbal_yaw_left_positive,
        gimbal_pitch_deg=gimbal_pitch_deg,
        camera_hfov_deg=camera_hfov_deg,
        camera_vfov_deg=camera_vfov_deg,
    )
    bore = _lrf_camera_dir_ned_unit(
        video_x_norm=boresight_u,
        video_y_norm=boresight_v,
        **pose_kw,
    )
    click = _lrf_camera_dir_ned_unit(
        video_x_norm=video_x_norm,
        video_y_norm=video_y_norm,
        **pose_kw,
    )
    if bore is None or click is None:
        return GeoReferenceResult(ok=False, warning="invalid look direction", method=method)
    (b_n, b_e, b_d), gimbal_assumed = bore
    (c_n, c_e, c_d), _ = click

    horiz_b = math.hypot(b_n, b_e)
    if horiz_b < 1e-4:
        fallback = compute_lrf_slant_geo(
            vehicle_lat=vehicle_lat,
            vehicle_lon=vehicle_lon,
            vehicle_heading_deg=vehicle_heading_deg,
            vehicle_roll_deg=vehicle_roll_deg,
            vehicle_pitch_deg=vehicle_pitch_deg,
            vehicle_alt_msl_m=vehicle_alt_msl_m,
            gimbal_yaw_deg=gimbal_yaw_deg,
            gimbal_yaw_left_positive=gimbal_yaw_left_positive,
            gimbal_pitch_deg=gimbal_pitch_deg,
            slant_range_m=slant,
            video_x_norm=video_x_norm,
            video_y_norm=video_y_norm,
            gps_fix_type=gps_fix_type,
            gps_hdop=gps_hdop,
            camera_hfov_deg=camera_hfov_deg,
            camera_vfov_deg=camera_vfov_deg,
        )
        extra = "facade plane degenerate (vertical look)"
        warn = f"{fallback.warning}; {extra}" if fallback.warning else extra
        return GeoReferenceResult(
            ok=fallback.ok,
            target_lat=fallback.target_lat,
            target_lon=fallback.target_lon,
            target_alt_m=fallback.target_alt_m,
            horizontal_range_m=fallback.horizontal_range_m,
            bearing_deg=fallback.bearing_deg,
            depression_deg=fallback.depression_deg,
            quality=fallback.quality,
            warning=warn,
            method=method,
        )

    # Horizontal normal from wall toward drone (opposite horizontal look direction).
    n_n = -b_n / horiz_b
    n_e = -b_e / horiz_b
    anchor_n = slant * b_n
    anchor_e = slant * b_e
    anchor_d = slant * b_d
    plane_dot = n_n * anchor_n + n_e * anchor_e

    denom = n_n * c_n + n_e * c_e
    if abs(denom) < 1e-5:
        return GeoReferenceResult(
            ok=False,
            warning="click ray parallel to facade plane",
            method=method,
        )

    t_m = plane_dot / denom
    if t_m < 0.5:
        return GeoReferenceResult(
            ok=False,
            warning="facade intersection behind camera",
            method=method,
        )

    north_m = t_m * c_n
    east_m = t_m * c_e
    down_m = t_m * c_d
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
        # Laser measurement — a shallow look angle does not degrade it.
        range_is_measured=True,
    )
    if gimbal_assumed:
        extra = "gimbal attitude assumed level (0°, 0°)"
        warn = f"{warn}; {extra}" if warn else extra
    extra = "facade plane (LRF anchor at boresight)"
    if depression < 3.0:
        extra += "; near-horizon wall geometry"
    warn = f"{warn}; {extra}" if warn else extra
    if quality == "insufficient":
        quality = "fair"

    return GeoReferenceResult(
        ok=True,
        target_lat=tgt_lat,
        target_lon=tgt_lon,
        target_alt_m=tgt_alt,
        horizontal_range_m=horiz,
        depression_deg=depression,
        quality=quality,
        warning=warn,
        method=method,
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

    geo_method = str(row.get("geo_method") or "").strip().lower()
    if geo_method == "lrf_facade_plane":
        try:
            plane_alt = float(row.get("target_alt_m")) if row.get("target_alt_m") is not None else None
        except (TypeError, ValueError):
            plane_alt = None
        if plane_alt is not None:
            resolved = plane_alt
            method = "lrf_facade_plane"
            row["target_alt_method"] = method
            if resolved is not None:
                row["target_alt_m"] = resolved
            return

    # The height of a point that was measured is its own. A laser point is
    # where the laser hit, and a measured point of a ground ray is on the
    # ground.
    #
    # What follows was made for a click whose footprint is ground while the
    # click itself may be up on a wall: it adds height from the click's place
    # in the picture. On a measured point that is height nobody measured: 17 m
    # for a ground point 175 m away clicked a little above the middle of the
    # picture, 109 m for a laser point 891 m away. For a laser point on a wall
    # it adds the point's height above the ground a second time. The Altitude
    # line of the fire correction came from it (found 2026-10-08).
    #
    # target_alt_m_elevated above still says what that rule would have given.
    if dem_val is not None and (
        geo_method == "lrf_slant"
        or (row.get("geo_measured") is True and geo_method.startswith("ray_"))
    ):
        row["target_alt_m"] = dem_val
        row["target_alt_method"] = "laser_point" if geo_method == "lrf_slant" else "ground_ray"
        return

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


def note_whether_measured(row: dict[str, object], geo: object) -> None:
    """Write onto a mark's row whether its point was measured, and why not.

    Every place that puts a result's point on a row calls this, so that the
    row never keeps the answer of an earlier try. A row without the note (a
    session saved before 2026-10-08, a click on the map) counts as measured.
    """
    row["geo_measured"] = bool(getattr(geo, "measured", True))
    row["geo_not_measured_why"] = str(getattr(geo, "not_measured_why", "") or "")
    row["geo_not_measured_look_deg"] = getattr(geo, "not_measured_look_deg", None)


def row_point_is_measured(row: object) -> bool:
    """False only for a point that rests on a guess (GeoReferenceResult.measured)."""
    try:
        return row.get("geo_measured") is not False  # type: ignore[union-attr]
    except AttributeError:
        return True


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
    note_whether_measured(row, geo)
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


def should_project_lrf_mark_via_geo(
    *,
    lrf_slew: bool,
    has_geo: bool,
    rel_alt_m: float | None,
    vehicle_shift_m: float,
    heading_delta_deg: float | None,
    slant_range_m: float | None = None,
    min_airborne_alt_m: float = 8.0,
    min_shift_m: float = 1.5,
    min_heading_deg: float = 5.0,
    min_slant_for_airborne_geo_m: float = 25.0,
) -> bool:
    """Choose geo vs gimbal-attitude projection for a video mark.

    LRF click-to-aim locks on boresight; on the ground/low hover GPS jitter must not
    drive the overlay (use attitude until the aircraft is clearly airborne or has moved).

    Near-field LRF locks (<~25 m slant) stay on gimbal attitude even when the aircraft
    climbs — GPS geo error dominates at short range and makes dual DOOAF marks drift.
    """
    if not has_geo:
        return False
    if not lrf_slew:
        return True
    if slant_range_m is not None:
        try:
            if float(slant_range_m) < float(min_slant_for_airborne_geo_m):
                return False
        except (TypeError, ValueError):
            pass
    try:
        alt = float(rel_alt_m) if rel_alt_m is not None else -1.0
    except (TypeError, ValueError):
        alt = -1.0
    if alt >= float(min_airborne_alt_m):
        return True
    if float(vehicle_shift_m) >= float(min_shift_m):
        return True
    if heading_delta_deg is not None:
        try:
            if abs(float(heading_delta_deg)) >= float(min_heading_deg):
                return True
        except (TypeError, ValueError):
            pass
    return False
