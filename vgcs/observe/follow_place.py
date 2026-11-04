"""Where the object that VGCS's tracker follows is on the ground, for Follow Me (vgcs/app/object_follow.py).

The app's rules (apk/custom/src/Skydroid/SkydroidLink.cc, _placeObject and _learnGround, 2026-10-13), worked out
with VGCS's own ground ray (compute_geo_reference):
 - The object's direction is the tracked point of the picture, through the gimbal's angles and the drone's attitude.
   Never on old gimbal angles (GIMBAL_FRESH_S), never without a reported camera angle, and only a point that
   compute_geo_reference counts as measured.
 - The ground at the object is level with the take-off point, or as much higher as the laser found it. The laser
   never puts it LOWER than the take-off point: a shot that misses the object lands behind it, which would put the
   object further away, and then the drone would be flown nearer.
 - A look too flat for a sure place gives none (compute_geo_reference refuses it): no place is better than a wrong one.
 - How sure the place is: an angle error (0.5 degrees, 0.25 with the laser) and a height error (2 m, 0.7 m with the
   laser) along the look, at least 1 m.
 - VGCS's tracker sees the picture with the digital zoom in it, so the lens's field of view is divided by that zoom.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

from vgcs.observe.geo_reference import compute_geo_reference

PLACE_EVERY_S = 0.18          # a place at most this often (the app's kPlaceEveryMs)
PICTURE_AGE_S = 0.30          # how old a picture is when its place is worked out (the app's kFollowPictureAgeS)
GIMBAL_FRESH_S = 1.5          # older gimbal angles give no place (the app's kAttitudeFreshMs)
ANGLE_SIGMA_DEG = 0.5
HEIGHT_SIGMA_M = 2.0
ANGLE_SIGMA_LASER_DEG = 0.25
HEIGHT_SIGMA_LASER_M = 0.7
LEAST_SIGMA_M = 1.0
LASER_ON_CROSS_DEG = 1.5      # a laser shot counts for the object only when it was this near the cross
GROUND_SHOTS = 3
GROUND_FRESH_S = 15.0


@dataclass(frozen=True)
class Place:
    lat_deg: float
    lon_deg: float
    sigma_m: float            # one standard deviation, metres
    below_m: float            # the drone's height above the object when the laser measured the ground, else NaN
    look_down_deg: float


class GroundRise:
    """The ground at the object above the take-off point, from the laser: the middle of the last GROUND_SHOTS shots
    (of two, the lower: the one that brings the object less near), for GROUND_FRESH_S after the last."""

    def __init__(self) -> None:
        self._rises: deque[float] = deque(maxlen=GROUND_SHOTS)
        self._at_s: float | None = None

    def learn(self, height_m: float, range_m: float, look_down_deg: float, now_s: float) -> None:
        """A laser shot at the object: the drone's height above the take-off point, the range, the look's angle."""
        values = (height_m, range_m, look_down_deg, now_s)
        if not all(isinstance(x, (int, float)) and math.isfinite(x) for x in values) or range_m <= 0.0:
            return
        below = range_m * math.sin(math.radians(look_down_deg))
        self._rises.append(height_m - below)
        self._at_s = now_s

    def rise_m(self, now_s: float) -> float | None:
        if not self._rises or self._at_s is None or now_s - self._at_s >= GROUND_FRESH_S:
            return None
        ordered = sorted(self._rises)
        return ordered[(len(ordered) - 1) // 2]

    def forget(self) -> None:
        self._rises.clear()
        self._at_s = None


def _zoom(digital_zoom: float) -> float:
    """The digital zoom in the tracker's picture: a centred crop, never below 1x."""
    return digital_zoom if math.isfinite(digital_zoom) and digital_zoom >= 1.0 else 1.0


def off_cross_deg(u: float, v: float, hfov_deg: float, vfov_deg: float, digital_zoom: float = 1.0) -> float:
    """How far the tracked point (0..1 of the picture the tracker sees) is from the picture's cross, degrees."""
    zoom = _zoom(digital_zoom)
    return math.hypot((u - 0.5) * hfov_deg / zoom, (v - 0.5) * vfov_deg / zoom)


def place_object(ctx: dict, u: float, v: float, hfov_deg: float, vfov_deg: float, *,
                 rise_m: float | None = None, lens: tuple[float, float] | None = None,
                 digital_zoom: float = 1.0) -> Place | None:
    """The tracked object (u, v: 0..1 of the picture the tracker sees) on the ground, or None when it cannot be placed.

    ctx: VGCS's observation context (the drone's place, attitude and height above the take-off point, the gimbal's
    angles and their age). rise_m: the ground at the object above the take-off point from the laser (GroundRise),
    None for level ground. lens: the camera's own lens at its zoom when VGCS knows it.
    """
    yaw, pitch, age = ctx.get("gimbal_yaw_deg"), ctx.get("gimbal_pitch_deg"), ctx.get("gimbal_attitude_age_s")
    height = ctx.get("vehicle_rel_alt_m")
    if yaw is None or pitch is None or age is None or not (float(age) < GIMBAL_FRESH_S):
        return None  # never on old gimbal angles, nor on a guessed one
    if height is None or ctx.get("vehicle_lat") is None or ctx.get("vehicle_lon") is None:
        return None
    height = float(height)
    by_laser = rise_m is not None and math.isfinite(rise_m)
    agl = height - (max(0.0, rise_m) if by_laser else 0.0)
    zoom = _zoom(digital_zoom)
    lens_h, lens_v = (lens if lens is not None else (None, None))
    geo = compute_geo_reference(
        vehicle_lat=ctx.get("vehicle_lat"), vehicle_lon=ctx.get("vehicle_lon"),
        vehicle_heading_deg=ctx.get("vehicle_heading_deg"),
        vehicle_roll_deg=ctx.get("vehicle_roll_deg"), vehicle_pitch_deg=ctx.get("vehicle_pitch_deg"),
        vehicle_rel_alt_m=agl, vehicle_alt_msl_m=None, rangefinder_down_m=None,
        gimbal_yaw_deg=float(yaw), gimbal_pitch_deg=float(pitch),
        gimbal_yaw_left_positive=ctx.get("gimbal_yaw_left_positive"),
        video_x_norm=float(u), video_y_norm=float(v),
        gps_fix_type=int(ctx.get("gps_fix_type") or 0), gps_hdop=ctx.get("gps_hdop"),
        camera_hfov_deg=float(hfov_deg) / zoom, camera_vfov_deg=float(vfov_deg) / zoom,
        dem_path=None, dem_terrain=False,
        lens_hfov_deg=None if lens_h is None else float(lens_h) / zoom,
        lens_vfov_deg=None if lens_v is None else float(lens_v) / zoom,
    )
    if (geo is None or not geo.ok or not geo.measured or geo.target_lat is None or geo.target_lon is None
            or geo.depression_deg is None):
        return None
    look_down = max(1.0, float(geo.depression_deg))
    sine = math.sin(math.radians(look_down))
    metres_per_degree = agl / (sine * sine) * math.pi / 180.0
    angle_sigma = ANGLE_SIGMA_LASER_DEG if by_laser else ANGLE_SIGMA_DEG
    height_sigma = HEIGHT_SIGMA_LASER_M if by_laser else HEIGHT_SIGMA_M
    sigma = max(LEAST_SIGMA_M, math.hypot(metres_per_degree * angle_sigma,
                                          height_sigma / math.tan(math.radians(look_down))))
    below = height - rise_m if by_laser else math.nan
    return Place(float(geo.target_lat), float(geo.target_lon), sigma, below, float(geo.depression_deg))
