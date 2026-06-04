"""Tests for observation target measure helpers."""

from __future__ import annotations

from vgcs.observe.target_measure import (
    format_target_segment_label,
    haversine_m,
    is_plausible_ground_range,
    marks_same_height_band,
    observation_facade_video_segments,
    observation_target_latlon,
    resolve_vehicle_agl_m,
    segment_distance_between_rows,
    segment_distances_m,
    session_facade_reference_range_m,
    session_peak_geo_range_m,
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
    assert format_target_segment_label(107.0, video_span_norm=0.15).startswith(
        "distance unreliable"
    )


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
    assert 2.0 <= d <= 2.9


def test_map_mark_uses_map_latlon():
    row = {"map_lat": 1.5, "map_lon": 2.5, "geo_quality": "map_direct"}
    pt = observation_target_latlon(row)
    assert pt == (1.5, 2.5)


def test_cross_height_band_skips_segment():
    low = {"video_y_norm": 0.55, "video_x_norm": 0.2, "geo_range_m": 30.0, "geo_bearing_deg": 90.0}
    high = {"video_y_norm": 0.25, "video_x_norm": 0.7, "geo_range_m": 12.0, "geo_bearing_deg": 95.0}
    assert not marks_same_height_band(low, high)
    assert segment_distance_between_rows(low, high) is None


def test_elevated_width_uses_session_peak():
    """Upper pillar row underestimates range; calibrated range from lower L–R restores width."""
    dx = 0.27
    bottom_l = {
        "video_y_norm": 0.58,
        "video_x_norm": 0.25,
        "geo_range_m": 15.0,
        "geo_bearing_deg": 90.0,
        "target_lat": 20.0,
        "target_lon": 72.0,
    }
    bottom_r = {
        "video_y_norm": 0.56,
        "video_x_norm": 0.25 + dx,
        "geo_range_m": 15.5,
        "geo_bearing_deg": 98.0,
        "target_lat": 20.0002,
        "target_lon": 72.0003,
    }
    top_l = {
        "video_y_norm": 0.30,
        "video_x_norm": 0.28,
        "geo_range_m": 11.0,
        "geo_bearing_deg": 89.0,
        "target_lat": 20.0001,
        "target_lon": 72.0,
    }
    top_r = {
        "video_y_norm": 0.30,
        "video_x_norm": 0.28 + dx,
        "geo_range_m": 11.5,
        "geo_bearing_deg": 96.0,
        "target_lat": 20.0001,
        "target_lon": 72.0002,
    }
    rows = [bottom_l, bottom_r, top_l, top_r]
    peak = session_peak_geo_range_m(rows)
    assert peak >= 15.0
    d_low = segment_distance_between_rows(bottom_l, top_l, session_peak_range_m=peak)
    assert d_low is None
    facade_ref = session_facade_reference_range_m(rows, hfov_deg=62.0)
    d_top = segment_distance_between_rows(
        top_l,
        top_r,
        session_peak_range_m=peak,
        facade_reference_range_m=facade_ref,
        hfov_deg=62.0,
    )
    assert d_top is not None
    assert 3.5 <= d_top <= 5.5
    segs = observation_facade_video_segments(rows, hfov_deg=62.0)
    assert len(segs) == 2
    assert abs(float(segs[0][4].split()[0]) - float(segs[1][4].split()[0])) < 1.0


def test_multi_height_pillar_rows_match_bottom_width():
    """Bottom row ~4.2 m; upper row same dx but low geo_range must match after calibration."""
    hfov = 62.0
    dx = 0.22
    bottom_l = {
        "video_y_norm": 0.62,
        "video_x_norm": 0.22,
        "geo_range_m": 18.0,
        "geo_bearing_deg": 90.0,
        "target_lat": 20.0,
        "target_lon": 72.0,
    }
    bottom_r = {
        "video_y_norm": 0.60,
        "video_x_norm": 0.22 + dx,
        "geo_range_m": 18.5,
        "geo_bearing_deg": 98.0,
        "target_lat": 20.0002,
        "target_lon": 72.0003,
    }
    top_l = {
        "video_y_norm": 0.28,
        "video_x_norm": 0.24,
        "geo_range_m": 9.0,
        "geo_bearing_deg": 91.0,
        "target_lat": 20.0001,
        "target_lon": 72.0,
    }
    top_r = {
        "video_y_norm": 0.30,
        "video_x_norm": 0.24 + dx,
        "geo_range_m": 9.5,
        "geo_bearing_deg": 99.0,
        "target_lat": 20.0001,
        "target_lon": 72.0003,
    }
    rows = [bottom_l, bottom_r, top_l, top_r]
    segs = observation_facade_video_segments(rows, hfov_deg=hfov)
    assert len(segs) == 2
    d_bottom = float(segs[0][4].split()[0])
    d_top = float(segs[1][4].split()[0])
    assert 3.5 <= d_bottom <= 6.5
    assert abs(d_top - d_bottom) < 1.5


def test_vikas_log_pair_near_four_metres():
    """Field log: map ~2.8 m chord; tape on opening ~4 m."""
    rows = [
        {
            "video_x_norm": 0.16928721174004194,
            "video_y_norm": 0.27884615384615385,
            "geo_range_m": 37.0,
            "geo_bearing_deg": 333.0,
            "geo_depression_deg": 39.0,
            "target_lat": 20.4457752219297,
            "target_lon": 72.86313429502887,
            "rangefinder_down_m": 31.0,
            "kind": "video_mark",
        },
        {
            "video_x_norm": 0.690251572327044,
            "video_y_norm": 0.2724358974358974,
            "geo_range_m": 34.6,
            "geo_bearing_deg": 16.0,
            "geo_depression_deg": 42.0,
            "target_lat": 20.44575501266942,
            "target_lon": 72.86315040488074,
            "rangefinder_down_m": 31.0,
            "kind": "video_mark",
        },
    ]
    from vgcs.observe.target_measure import video_facade_width_m

    d = video_facade_width_m(rows[0], rows[1], hfov_deg=62.0)
    assert d is not None
    assert 3.6 <= d <= 4.6


def test_video_fallback_when_geo_insufficient():
    rows = [
        {
            "kind": "video_mark",
            "video_x_norm": 0.318,
            "video_y_norm": 0.517,
            "geo_quality": "insufficient",
            "rangefinder_down_m": 27.38,
        },
        {
            "kind": "video_mark",
            "video_x_norm": 0.834,
            "video_y_norm": 0.484,
            "geo_quality": "insufficient",
            "rangefinder_down_m": 27.38,
        },
    ]
    segs = observation_facade_video_segments(rows, hfov_deg=62.0)
    assert len(segs) == 1
    assert segs[0][4].startswith("~")
    assert float(segs[0][4].lstrip("~").split()[0]) > 5.0


def test_low_altitude_wide_span_not_capped_to_three_metres():
    """Field: RF ~5.3 m, tape ~4 m between posts; must not show ~2.5 m."""
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
    d = segment_distance_between_rows(left, right, hfov_deg=62.0)
    assert d is not None
    assert 3.0 <= d <= 5.0
    segs = observation_facade_video_segments([left, right], hfov_deg=62.0)
    assert len(segs) == 1
    shown = float(segs[0][4].split()[0])
    assert 3.0 <= shown <= 5.0
    assert "(wall)" in segs[0][4] or "(wall," in segs[0][4]
