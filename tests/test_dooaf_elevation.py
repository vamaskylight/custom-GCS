"""DOOAF report elevation — MSL vs relative altitude."""

from __future__ import annotations

from vgcs.observe.dooaf import (
    DOOAF_ROLE_GUN,
    DOOAF_ROLE_IMPACT,
    DOOAF_ROLE_INTENDED,
    GeoPoint,
    build_dooaf_session,
    format_dooaf_html_summary,
    _drone_alt_msl_from_row,
)


def _row(role: str, lat: float, lon: float, **extra) -> dict:
    return {
        "dooaf_role": role,
        "target_lat": lat,
        "target_lon": lon,
        "kind": "video_mark",
        **extra,
    }


def test_drone_alt_msl_prefers_vehicle_msl_over_relative():
    row = {
        "vehicle_lat": 20.446,
        "vehicle_lon": 72.863,
        "vehicle_alt_msl_m": 23.2,
        "vehicle_rel_alt_m": 1.3,
    }
    assert _drone_alt_msl_from_row(row) == 23.2


def test_drone_alt_msl_from_dem_ground_agl_when_msl_missing():
    row = {
        "vehicle_lat": 20.446,
        "vehicle_lon": 72.863,
        "vehicle_rel_alt_m": 1.3,
        "dem_ground_agl_m": 1.3,
        "target_alt_m_dem": 21.9,
        "target_alt_m": 21.9,
    }
    assert _drone_alt_msl_from_row(row) == 23.2


def test_drone_alt_msl_uses_dem_ground_agl_on_rooftop():
    row = {
        "vehicle_lat": 20.4457593,
        "vehicle_lon": 72.8632767,
        "vehicle_alt_msl_m": 36.43,
        "ekf_rel_alt_m": 1.33,
        "dem_ground_agl_m": 14.29,
        "target_alt_m_dem": 22.14,
    }
    alt = _drone_alt_msl_from_row(row)
    assert alt is not None
    assert abs(float(alt) - 36.43) < 0.5


def test_drone_alt_msl_rejects_stale_vehicle_msl_on_low_hover():
    row = {
        "vehicle_lat": 20.4461082,
        "vehicle_lon": 72.8632194,
        "vehicle_alt_msl_m": 69.5,
        "ekf_rel_alt_m": 2.2,
        "dem_ground_agl_m": 2.0,
        "target_alt_m_dem": 21.9,
    }
    alt = _drone_alt_msl_from_row(row)
    assert alt is not None
    assert abs(float(alt) - 24.1) < 1.0
    assert abs(float(alt) - 69.5) > 10.0


def test_build_session_drone_uses_msl_in_coordinate_table():
    session = build_dooaf_session(
        [
            _row(DOOAF_ROLE_GUN, 20.4460144, 72.8632082, target_alt_m=21.9),
            _row(DOOAF_ROLE_INTENDED, 20.4460141, 72.8632132, target_alt_m=21.9),
            _row(
                DOOAF_ROLE_IMPACT,
                20.4459962,
                72.8632225,
                target_alt_m=21.9,
                vehicle_lat=20.4459821,
                vehicle_lon=72.8632314,
                vehicle_alt_msl_m=23.2,
                vehicle_rel_alt_m=1.3,
                dem_ground_agl_m=1.3,
            ),
        ],
        gun_alt_m=21.9,
    )
    assert session.drone is not None
    assert session.drone.alt_m == 23.2
    html = format_dooaf_html_summary(session)
    assert "Drone (last obs)" in html
    assert "23.2 m MSL" in html
    assert "1.3 m MSL" not in html


def test_gun_alt_filled_from_settings_when_mark_has_no_elevation():
    session = build_dooaf_session(
        [_row(DOOAF_ROLE_GUN, 12.0, 77.0)],
        gun_lat=12.0,
        gun_lon=77.0,
        gun_alt_m=42.5,
    )
    assert session.gun is not None
    assert session.gun.alt_m == 42.5
