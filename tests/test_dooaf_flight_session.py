"""Tests for in-flight DOOAF facade session (one LRF lock, rapid UV picks)."""

from __future__ import annotations

import time

from vgcs.observe.dooaf_flight_session import (
    DooafFacadeSession,
    build_facade_overlay_hint,
    mark_track_use_geo_in_flight,
)


def _ctx(
    *,
    vlat: float = 28.6139,
    vlon: float = 77.2090,
    hdg: float = 90.0,
    gy: float = 10.0,
    gp: float = -30.0,
    alt_msl: float = 520.0,
) -> dict:
    return {
        "vehicle_lat": vlat,
        "vehicle_lon": vlon,
        "vehicle_heading_deg": hdg,
        "gimbal_yaw_deg": gy,
        "gimbal_pitch_deg": gp,
        "vehicle_alt_msl_m": alt_msl,
        "gps_fix_type": 3,
        "gps_hdop": 0.8,
    }


def test_facade_session_record_and_uv_pick_valid():
    session = DooafFacadeSession()
    assert not session.has_lock
    base = _ctx()
    session.record_from_context(42.5, base)
    assert session.has_lock
    assert session.slant_range_m == 42.5
    assert session.uv_pick_valid(base)
    drifted = _ctx(gy=14.5, gp=-30.0)
    assert not session.uv_pick_valid(drifted, max_gimbal_delta_deg=4.0)
    moved = _ctx(vlat=28.61392, vlon=77.20902)
    assert session.uv_pick_valid(moved, max_vehicle_shift_m=4.0)


def test_facade_session_geo_from_uv_returns_ok():
    session = DooafFacadeSession()
    ctx = _ctx()
    session.record_from_context(50.0, ctx)
    geo = session.geo_from_uv(0.52, 0.48, hfov_deg=60.0, vfov_deg=34.0, ctx=ctx)
    assert geo.ok
    assert geo.target_lat is not None
    assert geo.target_lon is not None


def test_facade_session_rejects_bad_slant():
    session = DooafFacadeSession()
    session.record_from_context(0.1, _ctx())
    assert not session.has_lock


def test_facade_session_clear():
    session = DooafFacadeSession()
    session.record_from_context(30.0, _ctx())
    session.clear()
    assert not session.has_lock


def test_facade_session_expires_by_age():
    session = DooafFacadeSession()
    session.record_from_context(30.0, _ctx())
    session._lock.lock_mono = time.monotonic() - 700.0  # type: ignore[union-attr]
    assert not session.uv_pick_valid(_ctx(), max_age_s=600.0)


def test_build_facade_overlay_hint_ready_with_pending() -> None:
    text = build_facade_overlay_hint(
        slant_range_m=42.5,
        uv_pick_ready=True,
        pending_roles=["Gun", "Target", "Impact"],
    )
    assert text is not None
    title, subtitle = text
    assert "42.5 m" in title
    assert "Facade locked" in title
    assert "Gun" in subtitle
    assert "fast pick" in subtitle.lower()


def test_build_facade_overlay_hint_ready_complete() -> None:
    text = build_facade_overlay_hint(
        slant_range_m=30.0,
        uv_pick_ready=True,
        pending_roles=[],
    )
    assert text is not None
    assert "All marks set" in text[1]


def test_build_facade_overlay_hint_stale() -> None:
    text = build_facade_overlay_hint(
        slant_range_m=50.0,
        uv_pick_ready=False,
        pending_roles=["Impact"],
    )
    assert text is not None
    assert "stale" in text[0].lower()
    assert "50.0 m" in text[1]


def test_mark_track_use_geo_in_flight():
    assert mark_track_use_geo_in_flight(has_geo=True, rel_alt_m=20.0)
    assert not mark_track_use_geo_in_flight(has_geo=True, rel_alt_m=2.0)
    assert not mark_track_use_geo_in_flight(has_geo=False, rel_alt_m=20.0)
