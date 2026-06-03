"""Tape calibration for OBSERVE segment scale."""

from __future__ import annotations

from vgcs.observe.target_measure import (
    calibrate_segment_scale_from_tape,
    get_segment_distance_scale,
    last_band_measure_width_m,
    set_segment_distance_scale,
)


def test_calibrate_from_tape_sets_scale(monkeypatch):
    set_segment_distance_scale(1.0)
    rows = [
        {
            "kind": "video_mark",
            "video_x_norm": 0.28,
            "video_y_norm": 0.30,
            "geo_depression_deg": 42.0,
            "geo_bearing_deg": -8.0,
            "geo_range_m": 7.5,
            "rangefinder_down_m": 5.35,
        },
        {
            "kind": "video_mark",
            "video_x_norm": 0.77,
            "video_y_norm": 0.30,
            "geo_depression_deg": 41.0,
            "geo_bearing_deg": 10.0,
            "geo_range_m": 7.8,
            "rangefinder_down_m": 5.35,
        },
    ]
    raw = last_band_measure_width_m(rows, hfov_deg=62.0, apply_user_scale=False)
    assert raw is not None
    out = calibrate_segment_scale_from_tape(4.0, rows, hfov_deg=62.0)
    assert out is not None
    assert abs(out["known_m"] - 4.0) < 0.01
    scale = get_segment_distance_scale()
    assert abs(scale * raw - 4.0) < 0.15 or scale == 1.5
    set_segment_distance_scale(1.0)
