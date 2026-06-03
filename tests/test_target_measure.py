"""Tests for observation target measure helpers."""

from __future__ import annotations

from vgcs.observe.target_measure import (
    observation_target_latlon,
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


def test_map_mark_uses_map_latlon():
    row = {"map_lat": 1.5, "map_lon": 2.5, "geo_quality": "map_direct"}
    pt = observation_target_latlon(row)
    assert pt == (1.5, 2.5)
