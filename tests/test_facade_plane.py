"""Facade-plane width helpers."""

from __future__ import annotations

from vgcs.observe.facade_plane import facade_plane_width_between_marks
from vgcs.observe.target_measure import (
    observation_facade_video_segments,
    segment_distance_between_rows,
)


def test_garage_four_metre_field_log():
    rf = 5.35
    left = {
        "video_x_norm": 0.2856394129979036,
        "video_y_norm": 0.297008547008547,
        "target_lat": 20.445839745245905,
        "target_lon": 72.8629600280516,
        "geo_range_m": 7.5,
        "geo_bearing_deg": -8.0,
        "geo_depression_deg": 42.0,
        "rangefinder_down_m": rf,
        "kind": "video_mark",
        "geo_quality": "good",
    }
    right = {
        "video_x_norm": 0.7672955974842768,
        "video_y_norm": 0.3002136752136752,
        "target_lat": 20.445938345938377,
        "target_lon": 72.8628582361341,
        "geo_range_m": 7.8,
        "geo_bearing_deg": 10.0,
        "geo_depression_deg": 41.0,
        "rangefinder_down_m": rf,
        "kind": "video_mark",
        "geo_quality": "good",
    }
    d_fp = facade_plane_width_between_marks(left, right)
    assert d_fp is not None
    assert 3.2 <= d_fp <= 4.6
    d = segment_distance_between_rows(left, right, hfov_deg=62.0)
    assert d is not None
    assert 3.2 <= d <= 4.6
    segs = observation_facade_video_segments([left, right], hfov_deg=62.0)
    assert len(segs) == 1
    assert "(wall)" in segs[0][4] or "tape" in segs[0][4]
