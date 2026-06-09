"""Repro 8.5 m (wall): towers at y~0.58–0.65 with RF 45 must use long-range path."""

from __future__ import annotations

from vgcs.observe.geo_reference import compute_geo_reference
from vgcs.observe.target_measure import (
    facade_measure_context,
    observation_facade_video_segments,
    resolve_facade_ray_agl_m,
    segment_distance_between_rows,
)

LAT, LON = 20.4458323, 72.8632341
RF = 45.0


def _row(vx: float, vy: float, ekf: float = 0.5) -> dict:
    agl, src = resolve_facade_ray_agl_m(
        relative_alt_m=ekf, rangefinder_down_m=RF, video_y_norm=vy
    )
    g = compute_geo_reference(
        vehicle_lat=LAT,
        vehicle_lon=LON,
        vehicle_heading_deg=325.0,
        vehicle_rel_alt_m=ekf,
        rangefinder_down_m=RF,
        gimbal_yaw_deg=0.0,
        gimbal_pitch_deg=0.0,
        video_x_norm=vx,
        video_y_norm=vy,
        gps_fix_type=3,
        gps_hdop=0.7,
    )
    return {
        "video_x_norm": vx,
        "video_y_norm": vy,
        "ekf_rel_alt_m": ekf,
        "rangefinder_down_m": RF,
        "agl_source": src,
        "geo_agl_source": src,
        "measure_agl_m": agl,
        "geo_quality": g.quality,
        "geo_range_m": g.horizontal_range_m,
        "geo_bearing_deg": g.bearing_deg,
        "geo_depression_deg": g.depression_deg,
        "kind": "video_mark",
    }


def test_y_058_not_near_wall_eight_metres():
    left = _row(0.40, 0.58)
    right = _row(0.52, 0.60)
    assert (
        resolve_facade_ray_agl_m(relative_alt_m=0.5, rangefinder_down_m=RF, video_y_norm=0.58)[1]
        == "rangefinder_clamped_long"
    )
    assert facade_measure_context(left, right) == "long_range"
    d = segment_distance_between_rows(left, right, hfov_deg=62.0)
    assert d is not None
    assert d >= 40.0
    seg = observation_facade_video_segments([left, right], hfov_deg=62.0)
    assert "(distant)" in seg[0][4]
    assert "(wall)" not in seg[0][4]


def test_y_072_still_near_wall_fallback_when_rf_clamped():
    """Lower opening (image 1 style) keeps 14 m courtyard fallback when RF maxed."""
    left = _row(0.18, 0.72)
    right = _row(0.25, 0.73)
    assert facade_measure_context(left, right) == "near_wall"
    agl, src = resolve_facade_ray_agl_m(
        relative_alt_m=0.5, rangefinder_down_m=RF, video_y_norm=0.72
    )
    assert src == "rangefinder_clamped_facade"
    assert agl == 14.0
