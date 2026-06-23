"""Elevation summary when target is set in DOOAF Setup (not an observation row)."""

from __future__ import annotations

from vgcs.observe.dooaf import (
    DOOAF_ROLE_GUN,
    DOOAF_ROLE_IMPACT,
    DOOAF_ROLE_INTENDED,
    build_dooaf_session,
    format_elevation_summary_html,
    merge_setup_video_marks,
    read_dooaf_setup_video_marks,
    write_dooaf_setup_video_mark,
    write_dooaf_setup_video_marks,
    _synthesize_setup_mark_row,
)
from vgcs.observe.facade_plane import (
    _pair_height_from_video_y_and_range,
    facade_msl_heights_from_ground_mark,
)


def _impact_row() -> dict:
    return {
        "kind": "video_mark",
        "dooaf_role": DOOAF_ROLE_IMPACT,
        "target_lat": 20.445869,
        "target_lon": 72.8632646,
        "target_alt_m": 22.14,
        "target_alt_m_dem": 22.14,
        "video_x_norm": 0.293,
        "video_y_norm": 0.462,
        "vehicle_lat": 20.4459821,
        "vehicle_lon": 72.8632314,
        "vehicle_alt_msl_m": 24.2,
        "vehicle_rel_alt_m": 2.037,
        "ekf_rel_alt_m": 2.037,
        "dem_ground_agl_m": 2.0,
        "measure_agl_m": 2.037,
        "agl_source": "ekf_relative",
        "gimbal_yaw_deg": 0.0,
        "gimbal_pitch_deg": -25.0,
        "gps_fix_type": 3,
        "geo_quality": "good",
        "geo_method": "ray_dem",
        "geo_range_m": 42.0,
        "geo_depression_deg": 2.8,
        "geo_bearing_deg": 195.0,
        "camera_hfov_deg": 62.0,
    }


def test_pair_height_fallback_low_hover():
    upper = {**_impact_row(), "video_y_norm": 0.462}
    lower = {
        **_impact_row(),
        "video_y_norm": 0.563,
        "video_x_norm": 0.292,
    }
    h = _pair_height_from_video_y_and_range(upper, lower)
    assert h is not None
    assert h > 1.0


def test_build_session_uses_setup_video_mark_for_target_elevation():
    impact = _impact_row()
    session = build_dooaf_session(
        [impact],
        target_lat=20.4458765,
        target_lon=72.8632638,
        target_alt_m=22.14,
        setup_video_marks={DOOAF_ROLE_INTENDED: (0.292, 0.563)},
    )
    assert session.intended is not None
    assert session.impact is not None
    assert session.height_correction_m is not None
    assert abs(float(session.height_correction_m)) > 0.3
    assert session.impact.alt_m is not None
    assert session.intended.alt_m is not None
    assert session.impact.alt_m > session.intended.alt_m
    assert session.height_correction_m is not None
    assert float(session.height_correction_m) < -0.3
    html = format_elevation_summary_html(session)
    assert "same as terrain DEM at footprint" not in html or "Height correction" in html


def test_synthesize_setup_mark_row_carries_geo_range():
    row = _synthesize_setup_mark_row(
        DOOAF_ROLE_INTENDED,
        20.4458765,
        72.8632638,
        0.292,
        0.563,
        _impact_row(),
    )
    assert row.get("video_y_norm") == 0.563
    assert row.get("target_lat") == 20.4458765
    assert row.get("target_lon") == 72.8632638
    assert row.get("geo_range_m") is not None or row.get("target_alt_m") is not None


class _MemSettings:
    def __init__(self) -> None:
        self._data: dict[str, object] = {}

    def value(self, key: str, default: object = "") -> object:
        return self._data.get(key, default)

    def setValue(self, key: str, val: object) -> None:
        self._data[key] = val

    def remove(self, key: str) -> None:
        self._data.pop(key, None)


def test_persisted_setup_video_marks_used_at_export():
    st = _MemSettings()
    write_dooaf_setup_video_mark(st, DOOAF_ROLE_INTENDED, 0.291, 0.552)
    merged = merge_setup_video_marks(None, st=st)
    assert merged is not None
    assert merged[DOOAF_ROLE_INTENDED] == (0.291, 0.552)
    impact = {
        "kind": "video_mark",
        "dooaf_role": DOOAF_ROLE_IMPACT,
        "target_lat": 20.4458918409563,
        "target_lon": 72.86317959713733,
        "target_alt_m": 20.690956115722656,
        "target_alt_m_dem": 20.69,
        "video_x_norm": 0.28958333333333336,
        "video_y_norm": 0.45580404685835996,
        "vehicle_lat": 20.4458518,
        "vehicle_lon": 72.8631962,
        "vehicle_rel_alt_m": 6.331,
        "ekf_rel_alt_m": 6.331,
        "dem_ground_agl_m": 2.419043884277343,
        "measure_agl_m": 2.419043884277343,
        "agl_source": "dem_terrain",
        "gimbal_yaw_deg": 0.0,
        "gimbal_pitch_deg": -20.0,
        "gps_fix_type": 3,
        "geo_quality": "good",
        "geo_method": "ray_dem",
        "geo_range_m": 45.0,
        "geo_depression_deg": 4.0,
        "geo_bearing_deg": 180.0,
        "camera_hfov_deg": 62.0,
    }
    session = build_dooaf_session(
        [impact],
        gun_lat=20.4458977,
        gun_lon=72.8631695,
        gun_alt_m=20.69,
        target_lat=20.4459048,
        target_lon=72.8631746,
        target_alt_m=20.69,
        setup_video_marks=merge_setup_video_marks(None, st=st),
    )
    assert session.height_correction_m is not None
    assert float(session.height_correction_m) < -0.5
    assert session.impact is not None
    assert session.intended is not None
    assert session.impact.alt_m is not None
    assert session.intended.alt_m is not None
    assert session.impact.alt_m > session.intended.alt_m
    assert session.correction is not None
    assert session.correction.impact_to_intended_m < 5.0


def test_read_write_setup_video_marks_roundtrip():
    st = _MemSettings()
    write_dooaf_setup_video_marks(
        st,
        {
            DOOAF_ROLE_INTENDED: (0.291, 0.552),
            "gun_origin": (0.173, 0.825),
        },
    )
    marks = read_dooaf_setup_video_marks(st)
    assert marks[DOOAF_ROLE_INTENDED] == (0.291, 0.552)
    assert marks["gun_origin"] == (0.173, 0.825)


def _session_151219_impact() -> dict:
    return {
        "kind": "video_mark",
        "dooaf_role": DOOAF_ROLE_IMPACT,
        "target_lat": 20.446030988101327,
        "target_lon": 72.86317613166864,
        "target_alt_m": 23.516334533691406,
        "target_alt_m_dem": 23.52,
        "video_x_norm": 0.28958333333333336,
        "video_y_norm": 0.4547390841320554,
        "vehicle_lat": 20.4460273,
        "vehicle_lon": 72.8631693,
        "vehicle_rel_alt_m": 8.091,
        "ekf_rel_alt_m": 8.091,
        "dem_ground_agl_m": 1.3536654663085947,
        "measure_agl_m": 1.3536654663085947,
        "agl_source": "dem_terrain",
        "gimbal_yaw_deg": 0.0,
        "gimbal_pitch_deg": -20.0,
        "gps_fix_type": 3,
        "geo_quality": "good",
        "geo_method": "ray_dem",
        "geo_range_m": 45.0,
        "geo_depression_deg": 4.0,
        "geo_bearing_deg": 180.0,
        "camera_hfov_deg": 62.0,
    }


def test_ground_facade_interpolation_raises_target_above_gun():
    impact = _session_151219_impact()
    session = build_dooaf_session(
        [impact],
        gun_lat=20.4460427,
        gun_lon=72.8631619,
        gun_alt_m=23.516334533691406,
        target_lat=20.4460379,
        target_lon=72.8631693,
        target_alt_m=23.516334533691406,
        setup_video_marks={
            DOOAF_ROLE_GUN: (0.171, 0.825),
            DOOAF_ROLE_INTENDED: (0.290, 0.560),
        },
    )
    assert session.gun is not None
    assert session.intended is not None
    assert session.impact is not None
    assert session.gun.alt_m is not None
    assert session.intended.alt_m is not None
    assert session.impact.alt_m is not None
    assert session.intended.alt_m > session.gun.alt_m + 0.5
    assert session.impact.alt_m > session.intended.alt_m + 0.3
    assert session.height_correction_m is not None
    assert float(session.height_correction_m) < -0.3


def test_facade_msl_heights_from_ground_mark_orders_video_y():
    impact = _session_151219_impact()
    gun_row = _synthesize_setup_mark_row(
        DOOAF_ROLE_GUN,
        20.4460427,
        72.8631619,
        0.171,
        0.825,
        impact,
        alt_m=23.516334533691406,
    )
    tgt_row = _synthesize_setup_mark_row(
        DOOAF_ROLE_INTENDED,
        20.4460379,
        72.8631693,
        0.290,
        0.560,
        impact,
        alt_m=23.516334533691406,
    )
    intended_msl, ground_msl = facade_msl_heights_from_ground_mark(
        gun_row, tgt_row, impact, 27.0
    )
    assert intended_msl is not None and ground_msl is not None
    assert abs(ground_msl - 23.52) < 0.5
    assert 25.0 < intended_msl < 26.5
    assert intended_msl < 27.0
