"""OBSERVE Reset + re-mark: same video span must not flip 1.3 m vs 3.1 m."""

from __future__ import annotations

from vgcs.observe.target_measure import (
    observation_facade_video_segments,
    segment_distance_between_rows,
)

# Same car-gap clicks as field (narrow span, RF 23 m, EKF ~3.8 m).
VX_L, VY_L = 0.6703354297693921, 0.6175213675213675
VX_R, VY_R = 0.8034591194968553, 0.5993589743589743
RF = 23.38
EKF = 3.8


def _row(vx: float, vy: float, *, ekf: float | None, legacy_rf_as_rel: bool = False) -> dict:
    row: dict = {
        "video_x_norm": vx,
        "video_y_norm": vy,
        "geo_range_m": 10.0,
        "geo_bearing_deg": 340.0,
        "geo_depression_deg": 28.0,
        "rangefinder_down_m": RF,
        "kind": "video_mark",
        "geo_quality": "good",
        "target_lat": 20.44592,
        "target_lon": 72.86321,
    }
    if legacy_rf_as_rel:
        row["vehicle_rel_alt_m"] = RF
        row["agl_source"] = "rangefinder_down"
    else:
        row["ekf_rel_alt_m"] = ekf
        row["vehicle_rel_alt_m"] = ekf
        row["agl_source"] = "ekf_relative"
    return row


def test_same_span_stable_when_second_click_loses_ekf():
    """Simulates first measure OK, Reset, re-mark while EKF briefly reads 0 on one click."""
    good = [_row(VX_L, VY_L, ekf=EKF), _row(VX_R, VY_R, ekf=EKF)]
    mixed = [_row(VX_L, VY_L, ekf=EKF), _row(VX_R, VY_R, ekf=None, legacy_rf_as_rel=True)]

    d_good = segment_distance_between_rows(good[0], good[1], hfov_deg=62.0)
    d_mixed = segment_distance_between_rows(mixed[0], mixed[1], hfov_deg=62.0)
    assert d_good is not None and d_mixed is not None
    assert abs(d_good - d_mixed) < 0.35

    seg_good = observation_facade_video_segments(good, hfov_deg=62.0)[0][4]
    seg_mixed = observation_facade_video_segments(mixed, hfov_deg=62.0)[0][4]
    w_good = float(seg_good.split()[0])
    w_mixed = float(seg_mixed.split()[0])
    assert abs(w_good - w_mixed) < 0.35
