"""Unit tests for M8 geo-referencing."""

from __future__ import annotations

import math

from vgcs.observe.geo_reference import compute_geo_reference
from vgcs.observe.target_measure import prefer_dem_ground_agl_over_ekf


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


def test_apply_geo_reference_result_to_video_row_sets_lrf():
    from vgcs.observe.geo_reference import (
        GeoReferenceResult,
        apply_geo_reference_result_to_video_row,
    )

    geo = GeoReferenceResult(
        ok=True,
        target_lat=20.1,
        target_lon=72.2,
        target_alt_m=15.0,
        quality="good",
        warning="",
        method="lrf_slant",
        horizontal_range_m=42.0,
        bearing_deg=90.0,
        depression_deg=-12.0,
    )
    row: dict[str, object] = {"kind": "video_mark"}
    apply_geo_reference_result_to_video_row(row, geo, slant_range_m=45.5)
    assert row["target_lat"] == 20.1
    assert row["target_lon"] == 72.2
    assert row["geo_method"] == "lrf_slant"
    assert row["lrf_slant_range_m"] == 45.5
    assert row["geo_range_m"] == 42.0


def test_lrf_slant_geo_nadir():
    from vgcs.observe.geo_reference import compute_lrf_slant_geo

    r = compute_lrf_slant_geo(
        vehicle_lat=20.0,
        vehicle_lon=72.0,
        vehicle_heading_deg=0.0,
        gimbal_yaw_deg=0.0,
        gimbal_pitch_deg=-90.0,
        slant_range_m=100.0,
        gps_fix_type=3,
    )
    assert r.ok
    assert r.target_lat is not None
    assert r.target_lon is not None
    assert abs(float(r.target_lat) - 20.0) < 0.0002
    assert abs(float(r.target_lon) - 72.0) < 0.0002


def test_lrf_slant_geo_forward():
    from vgcs.observe.geo_reference import compute_lrf_slant_geo

    r = compute_lrf_slant_geo(
        vehicle_lat=20.0,
        vehicle_lon=72.0,
        vehicle_heading_deg=0.0,
        gimbal_yaw_deg=0.0,
        gimbal_pitch_deg=-45.0,
        slant_range_m=100.0,
        gps_fix_type=3,
    )
    assert r.ok
    assert r.target_lat is not None
    assert float(r.target_lat) > 20.0
    assert abs(float(r.target_lon) - 72.0) < 0.001


def test_lrf_slant_geo_boresight_round_trip():
    """After click-to-aim slew, geo is at boresight; inverse projection returns centre."""
    from vgcs.observe.geo_reference import (
        compute_lrf_slant_geo,
        project_wgs84_to_video_norm,
    )

    geo = compute_lrf_slant_geo(
        vehicle_lat=20.4100559,
        vehicle_lon=72.8799531,
        vehicle_heading_deg=45.0,
        vehicle_alt_msl_m=120.0,
        gimbal_yaw_deg=18.8,
        gimbal_pitch_deg=-18.0,
        slant_range_m=79.0,
        video_x_norm=0.5,
        video_y_norm=0.5,
        gps_fix_type=3,
        camera_hfov_deg=83.4,
        camera_vfov_deg=46.9,
    )
    assert geo.ok
    assert geo.target_lat is not None
    assert geo.target_lon is not None
    uv = project_wgs84_to_video_norm(
        target_lat=float(geo.target_lat),
        target_lon=float(geo.target_lon),
        target_alt_m=geo.target_alt_m,
        vehicle_lat=20.4100559,
        vehicle_lon=72.8799531,
        vehicle_heading_deg=45.0,
        vehicle_alt_msl_m=120.0,
        gimbal_yaw_deg=18.8,
        gimbal_pitch_deg=-18.0,
        camera_hfov_deg=83.4,
        camera_vfov_deg=46.9,
    )
    assert uv is not None
    assert abs(uv[0] - 0.5) < 0.02
    assert abs(uv[1] - 0.5) < 0.02


def test_geo_projection_shifts_when_vehicle_moves():
    """Marks must reproject when the drone moves (wind / LOITER drift)."""
    from vgcs.observe.geo_reference import (
        compute_lrf_slant_geo,
        project_wgs84_to_video_norm,
    )

    geo = compute_lrf_slant_geo(
        vehicle_lat=20.4100559,
        vehicle_lon=72.8799531,
        vehicle_heading_deg=45.0,
        vehicle_alt_msl_m=120.0,
        gimbal_yaw_deg=18.8,
        gimbal_pitch_deg=-18.0,
        slant_range_m=79.0,
        video_x_norm=0.5,
        video_y_norm=0.5,
        gps_fix_type=3,
        camera_hfov_deg=83.4,
        camera_vfov_deg=46.9,
    )
    assert geo.ok and geo.target_lat is not None and geo.target_lon is not None
    common = dict(
        target_lat=float(geo.target_lat),
        target_lon=float(geo.target_lon),
        target_alt_m=geo.target_alt_m,
        vehicle_heading_deg=45.0,
        vehicle_alt_msl_m=120.0,
        gimbal_yaw_deg=18.8,
        gimbal_pitch_deg=-18.0,
        camera_hfov_deg=83.4,
        camera_vfov_deg=46.9,
    )
    uv_a = project_wgs84_to_video_norm(
        vehicle_lat=20.4100559,
        vehicle_lon=72.8799531,
        **common,
    )
    uv_b = project_wgs84_to_video_norm(
        vehicle_lat=20.4101059,
        vehicle_lon=72.8799531,
        **common,
    )
    assert uv_a is not None and uv_b is not None
    assert abs(uv_a[0] - uv_b[0]) > 0.005 or abs(uv_a[1] - uv_b[1]) > 0.005


def test_dem_ground_agl_when_ekf_home_offset():
    """EKF rel above home can read ~7 m on the ground; DEM ground height is physical AGL."""
    agl, src = prefer_dem_ground_agl_over_ekf(
        relative_alt_m=6.87,
        facade_agl_m=6.87,
        facade_src="ekf_relative",
        dem_ground_agl_m=2.4,
        dem_ground_src="dem_terrain",
    )
    assert agl == 2.4
    assert src == "dem_terrain"


def test_dem_ground_agl_when_dem_above_ekf_at_altitude():
    """High hover: terrain AGL from DEM can exceed EKF-by-home by >1 m."""
    agl, src = prefer_dem_ground_agl_over_ekf(
        relative_alt_m=99.43,
        facade_agl_m=99.43,
        facade_src="ekf_relative",
        dem_ground_agl_m=100.99,
        dem_ground_src="dem_terrain",
    )
    assert agl == 100.99
    assert src == "dem_terrain"


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


def test_low_agl_level_gimbal_center_click_uses_estimated_pitch():
    # Level gimbal at bench height: infer downward look instead of km-flat-earth hit.
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
    assert r.ok
    assert r.horizontal_range_m is not None
    assert r.horizontal_range_m < 10.0
    assert "estimated" in (r.warning or "").lower()


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


def test_ekf_5m_level_gimbal_low_video_click():
    """Field case: ~5 m EKF rel alt, C13 0° pitch, click near bottom of frame."""
    r = compute_geo_reference(
        vehicle_lat=20.4458747,
        vehicle_lon=72.8632482,
        vehicle_heading_deg=331.0,
        vehicle_rel_alt_m=5.065,
        gimbal_yaw_deg=0.0,
        gimbal_pitch_deg=0.0,
        video_x_norm=0.23532494758909853,
        video_y_norm=0.8728632478632479,
        gps_fix_type=3,
        gps_hdop=5.384,
    )
    assert r.target_lat is not None
    assert r.target_lon is not None
    assert r.horizontal_range_m is not None
    assert r.horizontal_range_m < 80.0
    warn = (r.warning or "").lower()
    assert "estimated" in warn or "assumed" in warn or r.ok


def test_c13_level_gimbal_reading_low_video_click():
    """C13 reports 0,0 while oblique — same field log as bench/low EKF rel alt."""
    r = compute_geo_reference(
        vehicle_lat=20.4458136,
        vehicle_lon=72.8632475,
        vehicle_heading_deg=331.0,
        vehicle_rel_alt_m=0.316,
        gimbal_yaw_deg=0.0,
        gimbal_pitch_deg=0.0,
        video_x_norm=0.240041928721174,
        video_y_norm=0.719017094017094,
        gps_fix_type=3,
        gps_hdop=1.24,
    )
    assert r.ok
    assert r.target_lat is not None
    assert r.target_lon is not None
    assert r.horizontal_range_m is not None
    assert r.horizontal_range_m < 25.0
    assert "estimated" in (r.warning or "").lower()


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
