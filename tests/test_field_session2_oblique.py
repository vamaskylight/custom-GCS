"""Field log 2026-06-04: session 2 roof measure (RF 23 m, rel alt ~3.8 m)."""

from __future__ import annotations

from vgcs.observe.facade_plane import facade_plane_width_between_marks
from vgcs.observe.target_measure import (
    haversine_m,
    observation_facade_video_segments,
    segment_distance_between_rows,
)


def test_session2_uses_rel_alt_not_downward_rf():
    """Downward RF is ground; width must not inflate to ~4 m from RF alone."""
    left = {
        "video_x_norm": 0.6750524109014675,
        "video_y_norm": 0.6228632478632479,
        "target_lat": 20.445930492572828,
        "target_lon": 72.86321071723926,
        "geo_range_m": 10.0,
        "geo_bearing_deg": 340.0,
        "geo_depression_deg": 28.0,
        "rangefinder_down_m": 23.38,
        "ekf_rel_alt_m": 3.8,
        "vehicle_rel_alt_m": 3.8,
        "kind": "video_mark",
        "geo_quality": "good",
    }
    right = {
        "video_x_norm": 0.8029350104821803,
        "video_y_norm": 0.6025641025641025,
        "target_lat": 20.446360280244637,
        "target_lon": 72.86314709341427,
        "geo_range_m": 10.5,
        "geo_bearing_deg": 349.0,
        "geo_depression_deg": 27.0,
        "rangefinder_down_m": 23.38,
        "ekf_rel_alt_m": 3.8,
        "vehicle_rel_alt_m": 3.8,
        "kind": "video_mark",
        "geo_quality": "good",
    }
    d_hav = haversine_m(
        left["target_lat"],
        left["target_lon"],
        right["target_lat"],
        right["target_lon"],
    )
    assert d_hav > 30.0
    d_fp = facade_plane_width_between_marks(left, right)
    assert d_fp is not None
    assert 0.8 <= d_fp <= 2.5
    d = segment_distance_between_rows(left, right, hfov_deg=62.0)
    assert d is not None
    assert 0.8 <= d <= 2.5
    segs = observation_facade_video_segments([left, right], hfov_deg=62.0)
    assert len(segs) == 1
    assert "ground geo differs" in segs[0][4]
