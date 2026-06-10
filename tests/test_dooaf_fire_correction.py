"""DOOAF fire correction — gun / intended / impact variables."""

from __future__ import annotations

import math

from vgcs.observe.dooaf import (
    DOOAF_ROLE_GUN,
    DOOAF_ROLE_IMPACT,
    DOOAF_ROLE_INTENDED,
    DooafSettings,
    GeoPoint,
    apply_map_pick_to_settings,
    build_dooaf_session,
    compute_fire_correction,
    format_dooaf_html_summary,
    format_gimbal_pitch_direction,
    format_gimbal_yaw_direction,
    initial_bearing_deg,
    latlon_delta_to_ne_m,
    resolved_dooaf_settings,
    validate_dooaf_settings,
)


def _row(role: str, lat: float, lon: float, **extra) -> dict:
    return {
        "dooaf_role": role,
        "target_lat": lat,
        "target_lon": lon,
        "kind": "map_mark",
        **extra,
    }


def test_initial_bearing_north_east():
    assert abs(initial_bearing_deg(0.0, 0.0, 1.0, 0.0) - 0.0) < 0.1
    assert abs(initial_bearing_deg(0.0, 0.0, 0.0, 1.0) - 90.0) < 0.1


def test_fire_correction_on_line_short():
    gun = GeoPoint(12.0, 77.0)
    intended = GeoPoint(12.01, 77.0)
    impact = GeoPoint(12.009, 77.0)
    corr = compute_fire_correction(gun, intended, impact)
    assert corr.miss_along_m < 0
    assert corr.range_correction_m > 0
    assert abs(corr.miss_right_m) < 5.0
    assert abs(corr.deflection_correction_m) < 5.0


def test_fire_correction_right_deflection():
    gun = GeoPoint(12.0, 77.0)
    intended = GeoPoint(12.01, 77.0)
    north, east = latlon_delta_to_ne_m(12.01, 77.0, 12.01, 77.001)
    impact = GeoPoint(12.01, 77.0 + east / (111_320.0 * math.cos(math.radians(12.01))))
    corr = compute_fire_correction(gun, intended, impact)
    assert corr.miss_right_m > 0
    assert corr.deflection_correction_m < 0


def test_build_session_from_marks():
    rows = [
        _row(DOOAF_ROLE_GUN, 12.0, 77.0),
        _row(DOOAF_ROLE_INTENDED, 12.01, 77.0),
        _row(
            DOOAF_ROLE_IMPACT,
            12.0095,
            77.0,
            vehicle_lat=12.005,
            vehicle_lon=77.0,
            vehicle_rel_alt_m=120.0,
        ),
    ]
    session = build_dooaf_session(rows)
    assert session.gun is not None
    assert session.intended is not None
    assert session.impact is not None
    assert session.drone is not None
    assert session.drone.alt_m == 120.0
    assert session.correction is not None
    assert session.correction.impact_to_intended_m > 0


def test_gun_from_settings_fallback():
    rows = [_row(DOOAF_ROLE_INTENDED, 12.01, 77.0)]
    session = build_dooaf_session(rows, gun_lat=12.0, gun_lon=77.0)
    assert session.gun is not None
    assert session.gun.lat == 12.0


def test_target_from_settings_fallback():
    rows = [_row(DOOAF_ROLE_IMPACT, 12.0095, 77.0)]
    session = build_dooaf_session(
        rows,
        gun_lat=12.0,
        gun_lon=77.0,
        target_lat=12.01,
        target_lon=77.0,
    )
    assert session.intended is not None
    assert session.intended.lat == 12.01
    assert session.correction is not None


def test_validate_dooaf_settings_requires_pairs():
    assert validate_dooaf_settings(DooafSettings(gun_lat=1.0)) is not None
    assert validate_dooaf_settings(DooafSettings(gun_lat=1.0, gun_lon=2.0)) is None
    assert validate_dooaf_settings(DooafSettings()) is None


class _FakeSettings:
    def __init__(self, data: dict[str, float]) -> None:
        self._data = dict(data)

    def value(self, key: str):
        return self._data.get(key)

    def setValue(self, key: str, val: float) -> None:
        self._data[key] = val

    def remove(self, key: str) -> None:
        self._data.pop(key, None)


def test_resolved_dooaf_settings_from_map_marks():
    st = _FakeSettings(
        {
            "dooaf/gun_lat": 11.0,
            "dooaf/gun_lon": 76.0,
        }
    )
    rows = [_row(DOOAF_ROLE_INTENDED, 12.01, 77.0)]
    s = resolved_dooaf_settings(st, rows)
    assert s.gun_lat == 11.0
    assert s.target_lat == 12.01


def test_apply_map_pick_persists_single_role():
    base = DooafSettings(target_lat=12.01, target_lon=77.0)
    s = apply_map_pick_to_settings(base, DOOAF_ROLE_GUN, 12.0, 76.0)
    assert s.gun_lat == 12.0
    assert s.target_lat == 12.01


def test_settings_from_edits_empty():
    from PySide6.QtWidgets import QApplication, QDoubleSpinBox, QLineEdit

    app = QApplication.instance() or QApplication([])
    gun_lat = QLineEdit()
    gun_lon = QLineEdit()
    gun_alt = QDoubleSpinBox()
    gun_alt.setRange(-500.0, 12000.0)
    gun_alt.setValue(-500.0)
    tgt_lat = QLineEdit()
    tgt_lon = QLineEdit()
    tgt_alt = QDoubleSpinBox()
    tgt_alt.setRange(-500.0, 12000.0)
    tgt_alt.setValue(-500.0)
    from vgcs.map.dooaf_setup_dialog import settings_from_edits

    s = settings_from_edits(
        gun_lat=gun_lat,
        gun_lon=gun_lon,
        gun_alt=gun_alt,
        tgt_lat=tgt_lat,
        tgt_lon=tgt_lon,
        tgt_alt=tgt_alt,
    )
    assert s.gun_lat is None
    assert s.target_lat is None


def test_gimbal_direction_labels():
    assert format_gimbal_yaw_direction(28.3) == "Yaw right 28.3°"
    assert format_gimbal_yaw_direction(-5.0) == "Yaw left 5.0°"
    assert format_gimbal_pitch_direction(-10.1) == "Pitch down 10.1°"
    assert format_gimbal_pitch_direction(12.0) == "Pitch up 12.0°"


def test_dooaf_html_summary_highlights_and_camera():
    gun = GeoPoint(12.0, 77.0)
    intended = GeoPoint(12.01, 77.0)
    impact = GeoPoint(12.01, 77.001)
    session = build_dooaf_session(
        [
            _row(DOOAF_ROLE_GUN, gun.lat, gun.lon),
            _row(DOOAF_ROLE_INTENDED, intended.lat, intended.lon),
            _row(DOOAF_ROLE_IMPACT, impact.lat, impact.lon),
        ],
    )
    obs = {
        "gimbal_yaw_deg": 28.28,
        "gimbal_pitch_deg": -10.07,
        "dooaf_role": DOOAF_ROLE_IMPACT,
    }
    html = format_dooaf_html_summary(session, observation_row=obs)
    assert "dooaf-fire-corr" in html
    assert "dooaf-target-coords" in html
    assert "dooaf-impact-coords" in html
    assert "Yaw right 28.3°" in html
    assert "Pitch down 10.1°" in html
