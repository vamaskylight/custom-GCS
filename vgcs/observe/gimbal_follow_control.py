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

    Small deadband so the gimbal doesn't dither around dead-center; gain
    chosen so a target near the frame edge (large error) saturates at
    max_speed_dps rather than commanding an extreme rate.

    max_speed_dps raised 20 -> 40 on 2026-07-17: first confirmed-working
    field session showed yaw pinned at the old 20dps ceiling for many
    consecutive ticks while the target kept drifting further toward the
    frame edge rather than being reeled back in — the cap itself, not the
    gain or deadband, was the bottleneck (manual hold-button speed, 5dps
    default, is an unrelated slow/precise-aiming UI convenience, not a
    hardware ceiling — the gimbal was already sustaining 20dps commands
    fine). Gain/deadband left untouched so field results are attributable
    to this one change.
    """

    deadband_deg: float = 0.5
    gain_dps_per_deg: float = 2.5
    max_speed_dps: float = 40.0


def target_offset_deg(
    box_center_x: float,
    box_center_y: float,
    *,
    frame_w: int,
    frame_h: int,
    fov_h_deg: float,
    fov_v_deg: float,
) -> tuple[float, float]:
    """Yaw/pitch offset (deg) of a tracked box's center from frame center,
    in the real Skydroid GSY/GSP hardware sign convention: positive yaw =
    slew right, positive pitch = tilt up (matching the field-proven manual
    hold-button wiring in map_widget.py — UP button sends dy=+1 -> +pitch —
    and adapter.py's _gsp_pitch_rate_for_image_offset, which sends a
    NEGATIVE GSP rate to chase a target that is below frame center).

    dyaw keeps the image-space sign (target right of center -> positive,
    matching _pixel_boresight_offset_deg), but dpitch is the NEGATION of the
    raw image-space offset: a target below center (larger pixel y) needs the
    gimbal to tilt DOWN to converge on it, which is a negative pitch command.
    """
    ox = (float(box_center_x) - frame_w / 2.0) / max(1.0, frame_w / 2.0)
    oy = (float(box_center_y) - frame_h / 2.0) / max(1.0, frame_h / 2.0)
    dyaw = ox * (float(fov_h_deg) / 2.0)
    dpitch = -oy * (float(fov_v_deg) / 2.0)
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
    (dyaw_deg, dpitch_deg), where dyaw/dpitch are already in the real
    hardware sign convention from target_offset_deg (positive yaw = slew
    right, positive pitch = tilt up). The commanded speed keeps the same
    sign as the offset, since target_offset_deg already accounts for the
    pitch-axis inversion."""
    g = gains or FollowGains()
    return (
        _speed_for_error_deg(float(dyaw_deg), g),
        _speed_for_error_deg(float(dpitch_deg), g),
    )
