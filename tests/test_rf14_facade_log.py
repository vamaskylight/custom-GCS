"""Client log 2026-06-04: RF~14 m, GPS HDOP, ~4 m tape opening."""

from __future__ import annotations

from vgcs.observe.geo_reference import compute_geo_reference
from vgcs.observe.target_measure import (
    observation_facade_video_segments,
    segment_distance_between_rows,
)

LAT, LON = 20.4459514, 72.863255
RF = 13.94
VX_L, VY_L = 0.29245283018867924, 0.7061965811965812
VX_R, VY_R = 0.3522012578616352, 0.7136752136752137


def _mark(vx: float, vy: float, ekf: float) -> dict:
    g = compute_geo_reference(
        vehicle_lat=LAT,
        vehicle_lon=LON,
        vehicle_heading_deg=320.0,
        vehicle_rel_alt_m=ekf,
        rangefinder_down_m=RF,
        gimbal_yaw_deg=0.0,
        gimbal_pitch_deg=0.0,
        video_x_norm=vx,
        video_y_norm=vy,
        camera_hfov_deg=62.0,
        gps_fix_type=2,
        gps_hdop=1.61,
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


def test_rf14_opening_near_four_metres():
    left = _mark(VX_L, VY_L, ekf=1.186)
    right = _mark(VX_R, VY_R, ekf=1.61)
    assert left["geo_range_m"] is not None
    assert right["geo_range_m"] is not None
    d = segment_distance_between_rows(left, right, hfov_deg=62.0)
    assert d is not None
    assert 3.2 <= d <= 4.8
    seg = observation_facade_video_segments([left, right], hfov_deg=62.0)
    assert len(seg) == 1
    shown = float(seg[0][4].split()[0])
    assert 3.2 <= shown <= 4.8
    assert "km" not in seg[0][4].lower()
