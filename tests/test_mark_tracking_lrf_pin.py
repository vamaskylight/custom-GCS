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


class _FacadeFreezeHost(VideoMarkTrackingMixin):
    def __init__(self) -> None:
        from vgcs.observe.dooaf_flight_session import DooafFacadeSession

        self._dooaf_facade_session = DooafFacadeSession()
        self._dooaf_setup_mark_track = {}
        self._dooaf_setup_video_marks = {"gun": (0.5, 0.5)}
        self._lrf_lock_in_progress = False

    def _observation_context(self) -> dict[str, object]:
        return {
            "gimbal_yaw_deg": 17.0,
            "gimbal_pitch_deg": -8.0,
            "vehicle_lat": 20.41,
            "vehicle_lon": 72.88,
        }


def test_click_pin_holds_gun_mark_at_pick_uv_mid_range() -> None:
    """67 m field log: mark stays on click UV under gimbal jitter, not geo drift."""
    host = _PinHost((-2.6, -10.1))
    stored = (0.334, 0.710)
    track = {
        "click_pin": True,
        "pin_uv": stored,
        "pin_ref_att": (-2.6, -10.1),
        "ref_uv": stored,
        "ref_att": (-16.27, 0.0),
        "lrf_slew": True,
        "lrf_slant_range_m": 67.0,
        "geo_lat": 20.409824,
        "geo_lon": 72.879738,
        "h_scale": 1.0,
        "v_scale": 1.0,
    }
    host._att = (-2.8, -10.0)
    uv = host._tracked_uv_from_store(track, stored)
    assert uv == stored

    # Target-pick slew (~12° pitch) — gun mark must move on screen, not stay glued.
    host._att = (5.14, 2.77)
    host._lrf_lock_in_progress = True
    host._pending_lrf_video_pick = type(
        "_P", (), {"purpose": "dooaf_setup", "pick_role": "target"}
    )()
    uv_slew = host._tracked_uv_from_store(track, stored)
    assert uv_slew is not None
    assert uv_slew != stored
    assert uv_slew[1] > stored[1]  # pitch up → ground point moves down on screen


def test_facade_freeze_disabled_during_target_slew() -> None:
    host = _FacadeFreezeHost()
    host._dooaf_facade_session.record_from_context(
        68.2,
        {
            **host._observation_context(),
            "vehicle_heading_deg": 180.0,
        },
    )
    host._lrf_lock_in_progress = True
    host._pending_lrf_video_pick = type(
        "_P", (), {"purpose": "dooaf_setup", "pick_role": "target"}
    )()
    assert host._facade_frozen_mark_uv(0.411, 0.686) is None


def test_facade_session_freezes_gun_mark_when_idle() -> None:
    host = _FacadeFreezeHost()
    host._dooaf_facade_session.record_from_context(
        8.7,
        {
            **host._observation_context(),
            "vehicle_heading_deg": 180.0,
        },
    )
    host._dooaf_setup_mark_track["gun"] = {
        "ref_uv": (0.5, 0.5),
        "ref_att": (17.0, -8.0),
        "h_scale": 1.1,
        "v_scale": 1.0,
        "lrf_slew": True,
    }
    uv = host._dooaf_mark_display_uv("gun", (0.5, 0.5))
    assert uv == (0.5, 0.5)


def test_facade_session_freezes_impact_observation_mark_uv() -> None:
    host = _FacadeFreezeHost()
    host._dooaf_facade_session.record_from_context(
        8.7,
        {
            **host._observation_context(),
            "vehicle_heading_deg": 180.0,
        },
    )
    row = {
        "kind": "video_mark",
        "geo_method": "lrf_facade_uv",
        "video_x_norm": 0.669,
        "video_y_norm": 0.422,
        "video_mark_track_ref_u": 0.669,
        "video_mark_track_ref_v": 0.422,
        "video_mark_track_ref_yaw": 17.0,
        "video_mark_track_ref_pitch": -8.0,
        "video_mark_track_h_scale": 1.0,
        "video_mark_track_v_scale": 1.0,
        "video_mark_lrf_slew": False,
        "video_mark_geo_lat": 20.410118,
        "video_mark_geo_lon": 72.879944,
        "lrf_slant_range_m": 8.7,
    }
    uv = host._observation_mark_display_uv(row, 0.669, 0.422)
    assert uv == (0.669, 0.422)
