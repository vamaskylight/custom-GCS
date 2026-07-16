"""M14 — continuous gimbal-follow control loop math (pure functions, no I/O).

Converts a tracked target's pixel position into a gimbal speed command that
re-centers it, using the ACTIVE camera's real FOV (from its SkydroidCommandProfile)
rather than a hardcoded constant — this is a new code path, so unlike the
existing GOT+SUM-era aim math in vgcs/skydroid/adapter.py, it is camera-profile
aware from the start.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class FollowGains:
    """Proportional-control tuning for the follow loop.

    Conservative defaults: small deadband so the gimbal doesn't dither around
    dead-center, gain chosen so a target near the frame edge (large error)
    saturates at max_speed_dps rather than commanding an extreme rate. These
    are starting points for field tuning, not validated against real hardware.
    """

    deadband_deg: float = 0.5
    gain_dps_per_deg: float = 2.5
    max_speed_dps: float = 20.0


def target_offset_deg(
    box_center_x: float,
    box_center_y: float,
    *,
    frame_w: int,
    frame_h: int,
    fov_h_deg: float,
    fov_v_deg: float,
) -> tuple[float, float]:
    """Yaw/pitch offset (deg) of a tracked box's center from frame center.

    Same normalized-offset formula as adapter.py's _pixel_boresight_offset_deg,
    but takes fov_h_deg/fov_v_deg as parameters (the active camera profile's
    real values) instead of reading a hardcoded module constant.
    """
    ox = (float(box_center_x) - frame_w / 2.0) / max(1.0, frame_w / 2.0)
    oy = (float(box_center_y) - frame_h / 2.0) / max(1.0, frame_h / 2.0)
    dyaw = ox * (float(fov_h_deg) / 2.0)
    dpitch = oy * (float(fov_v_deg) / 2.0)
    return float(dyaw), float(dpitch)


def _speed_for_error_deg(error_deg: float, gains: FollowGains) -> float:
    if abs(error_deg) <= gains.deadband_deg:
        return 0.0
    raw = error_deg * gains.gain_dps_per_deg
    return max(-gains.max_speed_dps, min(gains.max_speed_dps, raw))


def follow_speed_command(
    dyaw_deg: float, dpitch_deg: float, *, gains: FollowGains | None = None
) -> tuple[float, float]:
    """(yaw_speed_dps, pitch_speed_dps) to re-center a target offset by
    (dyaw_deg, dpitch_deg). Positive dyaw/dpitch = target right/below center,
    matching _pixel_boresight_offset_deg's convention, so the gimbal should
    slew toward positive yaw/pitch to re-center it (same sign as the offset)."""
    g = gains or FollowGains()
    return (
        _speed_for_error_deg(float(dyaw_deg), g),
        _speed_for_error_deg(float(dpitch_deg), g),
    )
