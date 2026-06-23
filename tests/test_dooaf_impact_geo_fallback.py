"""Impact geo fallback when primary ray fails (low EKF / on ground)."""

from __future__ import annotations

from vgcs.observe.dooaf import (
    DOOAF_ROLE_GUN,
    DOOAF_ROLE_IMPACT,
    DOOAF_ROLE_INTENDED,
    apply_dooaf_impact_geo_fallback,
    build_dooaf_session,
    dooaf_export_blockers,
)
from vgcs.observe.target_measure import observation_target_latlon


def _failed_impact_row() -> dict:
    return {
        "kind": "video_mark",
        "dooaf_role": DOOAF_ROLE_IMPACT,
        "target_lat": None,
        "target_lon": None,
        "video_x_norm": 0.2984375,
        "video_y_norm": 0.45686900958466453,
        "vehicle_lat": 20.4461082,
        "vehicle_lon": 72.8632194,
        "vehicle_rel_alt_m": 0.343,
        "ekf_rel_alt_m": 0.343,
        "dem_ground_agl_m": 45.76043548583985,
        "measure_agl_m": 45.76043548583985,
        "agl_source": "dem_terrain",
        "geo_quality": "insufficient",
        "gps_fix_type": 3,
        "gimbal_yaw_deg": 0.0,
        "gimbal_pitch_deg": -20.0,
        "camera_hfov_deg": 62.0,
    }


def test_footprint_fallback_when_ray_fails():
    row = _failed_impact_row()
    ok = apply_dooaf_impact_geo_fallback(
        row,
        target_lat=20.4470249,
        target_lon=72.8628042,
        setup_video_marks={
            DOOAF_ROLE_INTENDED: (0.298, 0.555),
            DOOAF_ROLE_GUN: (0.189, 0.818),
        },
    )
    assert ok is True
    pt = observation_target_latlon(row)
    assert pt is not None
    assert row.get("geo_method") in (
        "dooaf_setup_target_footprint",
        "ray_facade_retry",
    )
    assert row.get("geo_quality") == "fair"


def test_build_session_after_fallback():
    row = _failed_impact_row()
    apply_dooaf_impact_geo_fallback(
        row,
        target_lat=20.4470249,
        target_lon=72.8628042,
        setup_video_marks={DOOAF_ROLE_INTENDED: (0.298, 0.555)},
    )
    session = build_dooaf_session(
        [row],
        gun_lat=20.4469627,
        gun_lon=72.8626840,
        gun_alt_m=20.846,
        target_lat=20.4470249,
        target_lon=72.8628042,
        target_alt_m=20.846,
        setup_video_marks={
            DOOAF_ROLE_GUN: (0.189, 0.818),
            DOOAF_ROLE_INTENDED: (0.298, 0.555),
        },
    )
    assert session.impact is not None
    assert session.correction is not None


def test_export_blockers_warns_low_ekf():
    row = _failed_impact_row()
    apply_dooaf_impact_geo_fallback(
        row,
        target_lat=20.4470249,
        target_lon=72.8628042,
        setup_video_marks={DOOAF_ROLE_INTENDED: (0.298, 0.555)},
    )
    warns = dooaf_export_blockers(
        [row],
        gun_lat=20.4469627,
        gun_lon=72.8626840,
        target_lat=20.4470249,
        target_lon=72.8628042,
        setup_video_marks={DOOAF_ROLE_INTENDED: (0.298, 0.555)},
    )
    assert any("near ground" in w.lower() or "estimated" in w.lower() for w in warns)
