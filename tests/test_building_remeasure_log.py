"""Client log 2026-06-04: building facade ~3.5 m flips after OBSERVE Reset."""

from __future__ import annotations

from vgcs.observe.geo_reference import compute_geo_reference
from vgcs.observe.target_measure import (
    observation_facade_video_segments,
    resolve_facade_ray_agl_m,
    segment_distance_between_rows,
)

VX_L, VY_L = 0.269916142557652, 0.7094017094017094
VX_R, VY_R = 0.3333333333333333, 0.717948717948718
RF = 25.888
LAT, LON = 20.4465096, 72.8631787


def _geo_mark(vx: float, vy: float, *, ekf: float) -> dict:
    ray_agl, _ = resolve_facade_ray_agl_m(
        relative_alt_m=ekf, rangefinder_down_m=RF
    )
    g = compute_geo_reference(
        vehicle_lat=LAT,
        vehicle_lon=LON,
        vehicle_heading_deg=327.0,
        vehicle_rel_alt_m=ray_agl,
        rangefinder_down_m=RF,
        gimbal_yaw_deg=0.0,
        gimbal_pitch_deg=-35.0,
        video_x_norm=vx,
        video_y_norm=vy,
        camera_hfov_deg=62.0,
        gps_fix_type=3,
        gps_hdop=1.0,
    )
    return {
        "video_x_norm": vx,
        "video_y_norm": vy,
        "ekf_rel_alt_m": ekf,
        "vehicle_rel_alt_m": ekf,
        "rangefinder_down_m": RF,
        "measure_agl_m": ray_agl,
        "geo_quality": g.quality,
        "geo_range_m": g.horizontal_range_m if g.ok else None,
        "geo_bearing_deg": g.bearing_deg if g.ok else None,
        "geo_depression_deg": g.depression_deg if g.ok else None,
        "target_lat": g.target_lat,
        "target_lon": g.target_lon,
        "kind": "video_mark",
    }


def test_first_measure_ekf_zero_matches_remark_ekf_two():
    """Log: ekf=0 → ~3.5 m; after Reset ekf≈2.3 must not collapse to ~2.1 m."""
    left = _geo_mark(VX_L, VY_L, ekf=0.0)
    right = _geo_mark(VX_R, VY_R, ekf=0.0)
    d_first = segment_distance_between_rows(left, right, hfov_deg=62.0)
    assert d_first is not None
    assert 2.8 <= d_first <= 4.5

    left2 = _geo_mark(VX_L, VY_L, ekf=2.329)
    right2 = _geo_mark(VX_R, VY_R, ekf=2.215)
    assert left2["geo_quality"] != "insufficient"
    assert right2["geo_quality"] != "insufficient"
    d_reset = segment_distance_between_rows(left2, right2, hfov_deg=62.0)
    assert d_reset is not None
    assert abs(d_first - d_reset) < 0.6

    seg = observation_facade_video_segments([left2, right2], hfov_deg=62.0)
    assert len(segs := seg) == 1
    assert "km" not in segs[0][4].lower()
    assert float(segs[0][4].split()[0]) < 50.0
