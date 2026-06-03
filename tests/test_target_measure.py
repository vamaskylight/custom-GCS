"""Tests for observation target measure helpers."""

from __future__ import annotations

from vgcs.observe.target_measure import (
    format_target_segment_label,
    is_plausible_ground_range,
    observation_target_latlon,
    resolve_vehicle_agl_m,
    segment_distances_m,
    target_track_from_observations,
)


def test_track_and_segment():
    rows = [
        {"target_lat": 37.0, "target_lon": -122.0, "kind": "video_mark"},
        {"target_lat": 37.001, "target_lon": -122.0, "kind": "video_mark"},
    ]
    track = target_track_from_observations(rows)
    assert len(track) == 2
    segs = segment_distances_m(track)
    assert len(segs) == 1
    assert segs[0] > 100.0


def test_resolve_agl_from_rangefinder():
    agl, src = resolve_vehicle_agl_m(relative_alt_m=0.0, rangefinder_down_m=1.2)
    assert agl == 1.2
    assert src == "rangefinder_down"


def test_reject_unrealistic_range():
    assert not is_plausible_ground_range(1.5, 107.0, 0.8)
    assert is_plausible_ground_range(1.5, 5.0, 25.0)


def test_segment_label_unreliable():
    assert format_target_segment_label(107.0, video_span_norm=0.15) == "distance unreliable"


def test_map_mark_uses_map_latlon():
    row = {"map_lat": 1.5, "map_lon": 2.5, "geo_quality": "map_direct"}
    pt = observation_target_latlon(row)
    assert pt == (1.5, 2.5)
