"""Tests for observation target measure helpers."""

from __future__ import annotations

from vgcs.observe.target_measure import (
    format_target_segment_label,
    haversine_m,
    is_plausible_ground_range,
    observation_target_latlon,
    resolve_vehicle_agl_m,
    segment_distance_between_rows,
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


def test_segment_label_one_decimal():
    assert format_target_segment_label(2.47) == "2.5 m"


def test_segment_distance_field_case():
    """Regression: client doorway ~2.5 m tape; haversine on geo ~2.1 m."""
    lat1, lon1 = 20.44590713981372, 72.86310082432684
    lat2, lon2 = 20.445891041183977, 72.86311164615348
    row_a = {
        "target_lat": lat1,
        "target_lon": lon1,
        "video_x_norm": 0.4088050314465409,
        "video_y_norm": 0.2777777777777778,
        "geo_range_m": 15.0,
        "geo_bearing_deg": 88.0,
    }
    row_b = {
        "target_lat": lat2,
        "target_lon": lon2,
        "video_x_norm": 0.5351153039832285,
        "video_y_norm": 0.26175213675213677,
        "geo_range_m": 15.5,
        "geo_bearing_deg": 96.0,
    }
    d_h = haversine_m(lat1, lon1, lat2, lon2)
    assert 2.0 < d_h < 2.3
    d = segment_distance_between_rows(row_a, row_b, hfov_deg=62.0)
    assert d is not None
    assert 2.3 <= d <= 2.8


def test_map_mark_uses_map_latlon():
    row = {"map_lat": 1.5, "map_lon": 2.5, "geo_quality": "map_direct"}
    pt = observation_target_latlon(row)
    assert pt == (1.5, 2.5)
