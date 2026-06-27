"""Near-field LRF mark pin — no overlay drift from gimbal attitude jitter."""

from __future__ import annotations

from vgcs.map.observation.mark_tracking_mixin import (
    VideoMarkTrackingMixin,
    _LRF_MARK_PIN_ATT_DEADBAND_DEG,
)


class _PinHost(VideoMarkTrackingMixin):
    def __init__(self, att: tuple[float, float]) -> None:
        self._att = att

    def _read_gimbal_attitude_pair(self) -> tuple[float, float] | None:
        return self._att


def test_gimbal_att_delta_wraps_yaw() -> None:
    dy, dp = VideoMarkTrackingMixin._gimbal_att_delta_deg((359.0, -8.0), (1.0, -8.0))
    assert dy < 3.0
    assert dp == 0.0


def test_near_field_pin_holds_uv_under_deadband() -> None:
    host = _PinHost((18.0, -8.0))
    track = {
        "near_field_pin": True,
        "pin_uv": (0.499, 0.503),
        "ref_att": (18.0, -8.0),
        "lrf_slew": True,
        "lrf_slant_range_m": 8.7,
    }
    # Tiny attitude jitter — mark must not wander.
    host._att = (18.2, -8.1)
    assert host._mark_track_near_field_pinned_uv(track) == (0.499, 0.503)
    host._att = (18.0 + _LRF_MARK_PIN_ATT_DEADBAND_DEG - 0.05, -8.0)
    assert host._mark_track_near_field_pinned_uv(track) == (0.499, 0.503)


def test_near_field_pin_releases_when_gimbal_pans() -> None:
    host = _PinHost((18.0, -8.0))
    track = {
        "near_field_pin": True,
        "pin_uv": (0.5, 0.5),
        "ref_att": (18.0, -8.0),
        "lrf_slew": True,
    }
    host._att = (22.0, -8.0)
    assert host._mark_track_near_field_pinned_uv(track) is None
