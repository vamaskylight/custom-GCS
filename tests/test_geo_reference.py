"""Unit tests for M8 geo-referencing."""

from __future__ import annotations

import math

from vgcs.observe.geo_reference import compute_geo_reference


def _nadir_result():
    return compute_geo_reference(
        vehicle_lat=37.0,
        vehicle_lon=-122.0,
        vehicle_heading_deg=0.0,
        vehicle_rel_alt_m=100.0,
        gimbal_yaw_deg=0.0,
        gimbal_pitch_deg=-90.0,
        video_x_norm=0.5,
        video_y_norm=0.5,
        gps_fix_type=3,
        gps_hdop=1.0,
    )


def test_nadir_mark_near_vehicle():
    r = _nadir_result()
    assert r.ok
    assert r.target_lat is not None
    assert r.target_lon is not None
    assert r.horizontal_range_m is not None
    assert r.horizontal_range_m < 5.0
    assert abs(r.target_lat - 37.0) < 0.0001
    assert abs(r.target_lon - (-122.0)) < 0.0001
    assert r.quality in ("good", "fair")


def test_forward_oblique_increases_range():
    r = compute_geo_reference(
        vehicle_lat=37.0,
        vehicle_lon=-122.0,
        vehicle_heading_deg=0.0,
        vehicle_rel_alt_m=100.0,
        gimbal_yaw_deg=0.0,
        gimbal_pitch_deg=-30.0,
        video_x_norm=0.5,
        video_y_norm=0.5,
        gps_fix_type=3,
    )
    assert r.ok
    assert r.horizontal_range_m is not None
    assert r.horizontal_range_m > 50.0
    assert r.target_lat is not None
    assert r.target_lat > 37.0


def test_rejects_absurd_range_at_low_agl():
    # Level gimbal + low click: flat-earth hit hundreds of metres away.
    r = compute_geo_reference(
        vehicle_lat=20.4458,
        vehicle_lon=72.8630,
        vehicle_heading_deg=0.0,
        vehicle_rel_alt_m=1.5,
        rangefinder_down_m=None,
        gimbal_yaw_deg=0.0,
        gimbal_pitch_deg=0.0,
        video_x_norm=0.5,
        video_y_norm=0.48,
        gps_fix_type=3,
    )
    assert not r.ok
    assert "unrealistic" in (r.warning or "").lower()


def test_rangefinder_agl_fallback():
    r = compute_geo_reference(
        vehicle_lat=20.4458,
        vehicle_lon=72.8630,
        vehicle_heading_deg=0.0,
        vehicle_rel_alt_m=0.0,
        rangefinder_down_m=1.2,
        gimbal_yaw_deg=0.5,
        gimbal_pitch_deg=0.0,
        video_x_norm=0.4,
        video_y_norm=0.35,
        gps_fix_type=3,
    )
    assert r.ok
    assert r.target_lat is not None
    assert r.method in ("ray_ground_rangefinder_agl", "ray_ground_flat", "ray_ground_dem")


def test_assumed_gimbal_low_video_click():
    """Skydroid TOP UDP missing: still place a near HIT from click + low AGL."""
    r = compute_geo_reference(
        vehicle_lat=20.4458,
        vehicle_lon=72.8630,
        vehicle_heading_deg=45.0,
        vehicle_rel_alt_m=0.6,
        gimbal_yaw_deg=None,
        gimbal_pitch_deg=None,
        video_x_norm=0.2,
        video_y_norm=0.75,
        gps_fix_type=3,
        gps_hdop=1.0,
    )
    assert r.target_lat is not None
    assert r.target_lon is not None
    assert r.horizontal_range_m is not None
    assert r.horizontal_range_m < 25.0
    warn = (r.warning or "").lower()
    assert "assumed" in warn or "estimated" in warn


def test_horizon_ray_fails():
    r = compute_geo_reference(
        vehicle_lat=37.0,
        vehicle_lon=-122.0,
        vehicle_heading_deg=0.0,
        vehicle_rel_alt_m=100.0,
        gimbal_yaw_deg=0.0,
        gimbal_pitch_deg=0.0,
        video_x_norm=0.5,
        video_y_norm=0.5,
    )
    assert not r.ok
    assert "horizon" in (r.warning or "").lower() or r.quality == "insufficient"


def test_pixel_offset_changes_bearing():
    left = compute_geo_reference(
        vehicle_lat=37.0,
        vehicle_lon=-122.0,
        vehicle_heading_deg=90.0,
        vehicle_rel_alt_m=80.0,
        gimbal_yaw_deg=0.0,
        gimbal_pitch_deg=-45.0,
        video_x_norm=0.2,
        video_y_norm=0.5,
        gps_fix_type=3,
    )
    right = compute_geo_reference(
        vehicle_lat=37.0,
        vehicle_lon=-122.0,
        vehicle_heading_deg=90.0,
        vehicle_rel_alt_m=80.0,
        gimbal_yaw_deg=0.0,
        gimbal_pitch_deg=-45.0,
        video_x_norm=0.8,
        video_y_norm=0.5,
        gps_fix_type=3,
    )
    assert left.ok and right.ok
    assert left.bearing_deg is not None and right.bearing_deg is not None
    assert abs(left.bearing_deg - right.bearing_deg) > 1.0
