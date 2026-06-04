"""Client log 2026-06-04: ekf≈4.5 m + rf≈14 m must use RF and stay near 4 m tape."""

from __future__ import annotations

from vgcs.observe.geo_reference import compute_geo_reference
from vgcs.observe.target_measure import (
    observation_facade_video_segments,
    prefer_facade_rangefinder_agl,
    segment_distance_between_rows,
)

LAT, LON = 20.4458244, 72.8632535
RF = 13.98


def _mark(vx: float, vy: float, ekf: float) -> dict:
    g = compute_geo_reference(
        vehicle_lat=LAT,
        vehicle_lon=LON,
        vehicle_heading_deg=317.0,
        vehicle_rel_alt_m=ekf,
        rangefinder_down_m=RF,
        gimbal_yaw_deg=0.0,
        gimbal_pitch_deg=0.0,
        video_x_norm=vx,
        video_y_norm=vy,
        camera_hfov_deg=62.0,
        gps_fix_type=3,
        gps_hdop=1.32,
    )
    return {
        "video_x_norm": vx,
        "video_y_norm": vy,
        "ekf_rel_alt_m": ekf,
        "rangefinder_down_m": RF,
        "geo_quality": g.quality,
        "geo_range_m": g.horizontal_range_m,
        "geo_bearing_deg": g.bearing_deg,
        "geo_depression_deg": g.depression_deg,
        "kind": "video_mark",
    }


def test_ekf_four_point_five_uses_rangefinder_not_ekf():
    assert prefer_facade_rangefinder_agl(4.5, RF)


def test_both_marks_ekf_four_stable_near_four_metres():
    left = _mark(0.24109014675052412, 0.7061965811965812, ekf=4.156)
    right = _mark(0.3055555555555556, 0.7115384615384616, ekf=4.337)
    assert left["geo_range_m"] is not None
    assert right["geo_range_m"] is not None
    d = segment_distance_between_rows(left, right, hfov_deg=62.0)
    assert d is not None
    assert 3.5 <= d <= 5.5


def test_mixed_ekf_zero_and_four_not_twenty_six_metres():
    left = _mark(0.23427672955974843, 0.7083333333333334, ekf=0.0)
    right = _mark(0.2976939203354298, 0.7147435897435898, ekf=4.53)
    d = segment_distance_between_rows(left, right, hfov_deg=62.0)
    assert d is not None
    assert d < 8.0
    seg = observation_facade_video_segments([left, right], hfov_deg=62.0)
    assert float(seg[0][4].split()[0]) < 8.0
