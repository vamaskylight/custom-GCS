"""Building height from roof + base video marks (DOOAF / facade vertical)."""

from __future__ import annotations

from vgcs.observe.dooaf import (
    DOOAF_ROLE_IMPACT,
    DOOAF_ROLE_INTENDED,
    GeoPoint,
    build_dooaf_session,
    compute_fire_correction,
    resolve_dooaf_mark_elevations,
)
from vgcs.observe.facade_plane import (
    facade_vertical_height_between_marks,
    infer_elevated_click_target_msl_from_row,
    infer_ray_target_msl_m,
    marks_suitable_for_facade_height,
)
from vgcs.observe.geo_reference import enrich_video_mark_target_altitude


def _oblique_field_roof_row() -> dict:
    return {
        "video_x_norm": 0.468,
        "video_y_norm": 0.425,
        "target_lat": 20.4600,
        "target_lon": 72.8800,
        "geo_range_m": 74.0,
        "geo_bearing_deg": 180.0,
        "geo_depression_deg": 52.0,
        "vehicle_alt_msl_m": 111.3,
        "target_alt_m": 11.28,
        "target_alt_m_dem": 11.28,
        "ekf_rel_alt_m": 100.0,
        "vehicle_rel_alt_m": 100.0,
        "kind": "video_mark",
        "dooaf_role": DOOAF_ROLE_INTENDED,
    }


def _oblique_field_base_row() -> dict:
    return {
        "video_x_norm": 0.460,
        "video_y_norm": 0.518,
        "target_lat": 20.4600,
        "target_lon": 72.8800,
        "geo_range_m": 74.0,
        "geo_bearing_deg": 180.0,
        "geo_depression_deg": 55.5,
        "vehicle_alt_msl_m": 111.3,
        "target_alt_m": 11.28,
        "target_alt_m_dem": 11.28,
        "ekf_rel_alt_m": 100.0,
        "vehicle_rel_alt_m": 100.0,
        "kind": "video_mark",
        "dooaf_role": DOOAF_ROLE_IMPACT,
    }


def test_infer_ray_target_msl_m():
    roof_msl = infer_ray_target_msl_m(111.3, 74.0, 52.0)
    base_msl = infer_ray_target_msl_m(111.3, 74.0, 55.5)
    assert roof_msl is not None and base_msl is not None
    assert roof_msl > base_msl
    assert abs(roof_msl - base_msl) > 8.0


def test_facade_vertical_height_field_oblique():
    roof = _oblique_field_roof_row()
    base = _oblique_field_base_row()
    ok, _ = marks_suitable_for_facade_height(roof, base)
    assert ok
    h = facade_vertical_height_between_marks(roof, base)
    assert h is not None
    assert 8.0 <= h <= 22.0


def test_resolve_dooaf_elevations_roof_above_base():
    roof = _oblique_field_roof_row()
    base = _oblique_field_base_row()
    intended = GeoPoint(20.46, 72.88, 11.28)
    impact = GeoPoint(20.4601, 72.8801, 11.28)
    ni, np, bh = resolve_dooaf_mark_elevations(roof, base, intended, impact)
    assert bh is not None
    assert bh >= 8.0
    assert ni is not None and np is not None
    assert ni.alt_m is not None and np.alt_m is not None
    assert ni.alt_m > np.alt_m
    assert abs(np.alt_m - 11.28) < 0.5


def test_fire_correction_vertical_miss():
    gun = GeoPoint(12.0, 77.0, 50.0)
    intended = GeoPoint(12.01, 77.0, 31.0)
    impact = GeoPoint(12.0095, 77.0, 11.0)
    corr = compute_fire_correction(gun, intended, impact)
    assert corr.miss_vertical_m == 20.0
    assert corr.elevation_correction_m == 20.0
    assert corr.impact_to_intended_m < 200.0


def test_build_session_building_height():
    rows = [_oblique_field_roof_row(), _oblique_field_base_row()]
    session = build_dooaf_session(
        rows,
        gun_lat=20.45,
        gun_lon=72.86,
        target_lat=20.46,
        target_lon=72.88,
    )
    assert session.building_height_m is not None
    assert session.intended is not None and session.impact is not None
    assert session.intended.alt_m is not None and session.impact.alt_m is not None
    assert session.intended.alt_m > session.impact.alt_m
    assert session.correction is not None
    assert session.correction.miss_vertical_m is not None
    assert session.correction.miss_vertical_m > 5.0


def test_enrich_video_mark_keeps_ground_for_base_click():
    row = _oblique_field_base_row()
    enrich_video_mark_target_altitude(row)
    assert row.get("target_alt_m_dem") == 11.28
    assert row.get("target_alt_method") == "terrain_dem"


def _client_distant_target_row() -> dict:
    """Field log: ~788 m gun→target, ~102 m AGL, DEM ~11.3 m, click on structure."""
    return {
        "video_x_norm": 0.540,
        "video_y_norm": 0.467,
        "target_lat": 20.4091622,
        "target_lon": 72.8804531,
        "geo_range_m": 788.4,
        "geo_bearing_deg": 200.0,
        "geo_depression_deg": 7.37,
        "vehicle_alt_msl_m": 113.0,
        "target_alt_m": 11.29,
        "target_alt_m_dem": 11.29,
        "measure_agl_m": 102.0,
        "ekf_rel_alt_m": 100.4,
        "kind": "video_mark",
        "dooaf_role": DOOAF_ROLE_INTENDED,
    }


def test_infer_elevated_click_msl_distant_field_target():
    row = _client_distant_target_row()
    msl = infer_elevated_click_target_msl_from_row(row)
    assert msl is not None
    assert msl > 20.0
    assert msl < 35.0
    assert msl - 11.29 > 10.0


def test_enrich_video_mark_elevates_distant_facade_click():
    row = _client_distant_target_row()
    enrich_video_mark_target_altitude(row)
    assert row.get("target_alt_method") == "video_facade_elevated"
    assert row.get("target_alt_m") is not None
    assert float(row["target_alt_m"]) > 20.0
