"""Client log 2026-06-04: RF clamped at 45 m, EKF ~0 m — must not read 6–8 m on ~4 m opening."""

from __future__ import annotations

from vgcs.observe.geo_reference import compute_geo_reference
from vgcs.observe.target_measure import (
    downward_rangefinder_plausible_m,
    resolve_facade_ray_agl_m,
    segment_distance_between_rows,
)

LAT, LON = 20.4458323, 72.8632341
RF = 45.0
HEADING = 310.0


def _mark(vx: float, vy: float, ekf: float) -> dict:
    agl, src = resolve_facade_ray_agl_m(
        relative_alt_m=ekf, rangefinder_down_m=RF, video_y_norm=vy
    )
    assert src == "rangefinder_clamped_facade"
    assert agl == 14.0
    g = compute_geo_reference(
        vehicle_lat=LAT,
        vehicle_lon=LON,
        vehicle_heading_deg=HEADING,
        vehicle_rel_alt_m=ekf,
        rangefinder_down_m=RF,
        gimbal_yaw_deg=0.0,
        gimbal_pitch_deg=0.0,
        video_x_norm=vx,
        video_y_norm=vy,
        camera_hfov_deg=62.0,
        gps_fix_type=3,
        gps_hdop=1.3,
    )
    assert g.horizontal_range_m is not None
    assert g.horizontal_range_m < 45.0
    return {
        "video_x_norm": vx,
        "video_y_norm": vy,
        "ekf_rel_alt_m": ekf,
        "rangefinder_down_m": RF,
        "geo_quality": g.quality,
        "geo_range_m": g.horizontal_range_m,
        "geo_bearing_deg": g.bearing_deg,
        "geo_depression_deg": g.depression_deg,
        "target_lat": g.target_lat,
        "target_lon": g.target_lon,
        "kind": "video_mark",
    }


def test_rf45_not_plausible_on_roof_bench():
    assert not downward_rangefinder_plausible_m(45.0, 0.9)
    assert downward_rangefinder_plausible_m(14.0, 0.9)


def test_clamped_rf_pair_near_four_metres():
    left = _mark(0.15146750524109015, 0.7232905982905983, ekf=0.931)
    right = _mark(0.22536687631027252, 0.7275641025641025, ekf=0.888)
    d = segment_distance_between_rows(left, right, hfov_deg=62.0)
    assert d is not None
    assert 3.5 <= d <= 5.5


def test_reset_mixed_ekf_zero_and_low():
    left = _mark(0.17662473794549266, 0.719017094017094, ekf=0.0)
    right = _mark(0.24790356394129978, 0.7200854700854701, ekf=0.96)
    d = segment_distance_between_rows(left, right, hfov_deg=62.0)
    assert d is not None
    assert 3.5 <= d <= 5.5
