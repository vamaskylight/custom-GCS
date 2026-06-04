"""Image-2 case: skyline roof marks + RF 45 must not show ~4 m (wall)."""

from __future__ import annotations

from vgcs.observe.geo_reference import compute_geo_reference
from vgcs.observe.target_measure import (
    FACADE_DISTANT_TARGET_HINT,
    marks_suitable_for_facade_tape_measure,
    observation_facade_video_segments,
    resolve_facade_ray_agl_m,
    segment_distance_between_rows,
)

LAT, LON = 20.4458323, 72.8632341
RF_CLAMP = 45.0
RF_GOOD = 14.6
HEADING = 325.0


def _mark(vx: float, vy: float, *, ekf: float, rf: float) -> dict:
    agl, src = resolve_facade_ray_agl_m(relative_alt_m=ekf, rangefinder_down_m=rf)
    g = compute_geo_reference(
        vehicle_lat=LAT,
        vehicle_lon=LON,
        vehicle_heading_deg=HEADING,
        vehicle_rel_alt_m=ekf,
        rangefinder_down_m=rf,
        gimbal_yaw_deg=0.0,
        gimbal_pitch_deg=0.0,
        video_x_norm=vx,
        video_y_norm=vy,
        camera_hfov_deg=62.0,
        gps_fix_type=3,
        gps_hdop=0.7,
    )
    return {
        "video_x_norm": vx,
        "video_y_norm": vy,
        "ekf_rel_alt_m": ekf,
        "rangefinder_down_m": rf,
        "agl_source": src,
        "geo_agl_source": src,
        "measure_agl_m": agl,
        "geo_quality": g.quality,
        "geo_range_m": g.horizontal_range_m,
        "geo_bearing_deg": g.bearing_deg,
        "geo_depression_deg": g.depression_deg,
        "target_lat": g.target_lat,
        "target_lon": g.target_lon,
        "kind": "video_mark",
    }


def test_near_wall_low_video_still_measures():
    """Good case (image 1): lower frame, RF ~14.6 m."""
    left = _mark(0.18, 0.72, ekf=0.9, rf=RF_GOOD)
    right = _mark(0.25, 0.73, ekf=0.88, rf=RF_GOOD)
    assert marks_suitable_for_facade_tape_measure(left, right)[0]
    d = segment_distance_between_rows(left, right, hfov_deg=62.0)
    assert d is not None
    assert 3.0 <= d <= 5.5


def test_skyline_rf45_not_four_metre_wall():
    """Bad case (image 2): upper frame rooftop towers, RF clamped 45 m."""
    left = _mark(0.42, 0.32, ekf=0.5, rf=RF_CLAMP)
    right = _mark(0.52, 0.34, ekf=0.6, rf=RF_CLAMP)
    assert not marks_suitable_for_facade_tape_measure(left, right)[0]
    assert segment_distance_between_rows(left, right, hfov_deg=62.0) is None
    segs = observation_facade_video_segments([left, right], hfov_deg=62.0)
    assert len(segs) == 1
    assert FACADE_DISTANT_TARGET_HINT in segs[0][4]
    assert "(wall)" not in segs[0][4]
