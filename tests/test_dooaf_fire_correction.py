"""DOOAF fire correction — gun / intended / impact variables."""

from __future__ import annotations

import math
import re

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
    _fire_correction_plan_svg,
    _fire_correction_gunline_svg,
    _fire_correction_compass_miss_svg,
    _executive_miss_map_svg,
    _fire_correction_positions_svg,
    _fc_inset_overlaps_markers,
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


def test_fire_correction_uses_ray_range_for_facade_setup():
    """Session 181133 — DEM footprints ~8 m apart; rays ~32 m gun→target."""
    template = {
        "vehicle_lat": 20.4458179,
        "vehicle_lon": 72.8632723,
        "vehicle_heading_deg": 315.0,
        "vehicle_pitch_deg": -7.4,
        "vehicle_roll_deg": 0.0,
        "vehicle_alt_msl_m": 36.0,
        "ekf_rel_alt_m": -0.159,
        "dem_ground_agl_m": 13.41,
        "gimbal_pitch_deg": -20.0,
        "gimbal_yaw_deg": 0.09,
        "gps_fix_type": 3,
        "camera_hfov_deg": 62.0,
    }
    gun_row = {
        **template,
        "kind": "video_mark",
        "dooaf_role": DOOAF_ROLE_GUN,
        "target_lat": 20.4459722,
        "target_lon": 72.8631038,
        "video_x_norm": 0.217,
        "video_y_norm": 0.942,
        "geo_range_m": 66.21,
        "geo_bearing_deg": 297.18,
    }
    tgt_row = {
        **template,
        "kind": "video_mark",
        "dooaf_role": DOOAF_ROLE_INTENDED,
        "target_lat": 20.4459501,
        "target_lon": 72.8631776,
        "video_x_norm": 0.454,
        "video_y_norm": 0.548,
        "geo_range_m": 37.14,
        "geo_bearing_deg": 312.09,
    }
    imp_row = {
        **template,
        "kind": "video_mark",
        "dooaf_role": DOOAF_ROLE_IMPACT,
        "target_lat": 20.44600962508643,
        "target_lon": 72.86313377357139,
        "video_x_norm": 0.4828125,
        "video_y_norm": 0.6741214057507987,
        "geo_range_m": 28.25,
        "geo_bearing_deg": 313.93,
    }
    gun = GeoPoint(20.4459722, 72.8631038, 23.516)
    intended = GeoPoint(20.4459501, 72.8631776, 24.15)
    impact = GeoPoint(20.44600962508643, 72.86313377357139, 20.69)
    corr_map = compute_fire_correction(gun, intended, impact)
    corr_ray = compute_fire_correction(
        gun,
        intended,
        impact,
        gun_row=gun_row,
        intended_row=tgt_row,
        impact_row=imp_row,
    )
    assert corr_map.range_gun_to_intended_m < 12.0
    assert corr_ray.range_gun_to_intended_m > 25.0
    assert corr_ray.range_gun_to_intended_m < 40.0


def test_fire_correction_gun_impact_uses_ray_when_footprint_collapsed():
    """Session 183427 — impact footprint ~8 m from gun; ray ~29 m on same facade."""
    template = {
        "vehicle_lat": 20.4458179,
        "vehicle_lon": 72.8632723,
        "vehicle_heading_deg": 315.0,
        "vehicle_pitch_deg": -7.4,
        "vehicle_roll_deg": 0.0,
        "vehicle_alt_msl_m": 36.0,
        "ekf_rel_alt_m": 2.2,
        "dem_ground_agl_m": 13.94,
        "gimbal_pitch_deg": -20.0,
        "gimbal_yaw_deg": 0.09,
        "gps_fix_type": 3,
        "camera_hfov_deg": 62.0,
    }
    gun_row = {
        **template,
        "kind": "video_mark",
        "dooaf_role": DOOAF_ROLE_GUN,
        "target_lat": 20.4459722,
        "target_lon": 72.8629682,
        "video_x_norm": 0.294,
        "video_y_norm": 0.900,
        "geo_range_m": 59.49,
        "geo_bearing_deg": 301.97,
    }
    tgt_row = {
        **template,
        "kind": "video_mark",
        "dooaf_role": DOOAF_ROLE_INTENDED,
        "target_lat": 20.4459258,
        "target_lon": 72.8631324,
        "video_x_norm": 0.458,
        "video_y_norm": 0.485,
        "geo_range_m": 26.19,
        "geo_bearing_deg": 312.15,
    }
    imp_row = {
        **template,
        "kind": "video_mark",
        "dooaf_role": DOOAF_ROLE_IMPACT,
        "target_lat": 20.44600266027315,
        "target_lon": 72.86304599625547,
        "video_x_norm": 0.48020833333333335,
        "video_y_norm": 0.6134185303514377,
    }
    gun = GeoPoint(20.4459722, 72.8629682, 23.516)
    intended = GeoPoint(20.4459258, 72.8631324, 24.15)
    impact = GeoPoint(20.44600266027315, 72.86304599625547, 20.69)
    corr = compute_fire_correction(
        gun,
        intended,
        impact,
        gun_row=gun_row,
        intended_row=tgt_row,
        impact_row=imp_row,
    )
    assert corr.range_gun_to_intended_m > 25.0
    assert corr.range_gun_to_impact_m > 20.0
    assert corr.range_gun_to_impact_m > corr.range_gun_to_intended_m * 0.55


def test_build_session_183427_gun_impact_ray_distance():
    from vgcs.observe.dooaf import build_dooaf_session

    impact = {
        "kind": "video_mark",
        "dooaf_role": DOOAF_ROLE_IMPACT,
        "target_lat": 20.44600266027315,
        "target_lon": 72.86304599625547,
        "target_alt_m": 20.69,
        "video_x_norm": 0.48020833333333335,
        "video_y_norm": 0.6134185303514377,
        "vehicle_lat": 20.4458179,
        "vehicle_lon": 72.8632723,
        "vehicle_heading_deg": 315.0,
        "vehicle_pitch_deg": -7.4,
        "vehicle_roll_deg": 0.0,
        "vehicle_alt_msl_m": 36.0,
        "ekf_rel_alt_m": 2.2,
        "dem_ground_agl_m": 13.94,
        "gimbal_pitch_deg": -20.0,
        "gimbal_yaw_deg": 0.09,
        "gps_fix_type": 3,
        "camera_hfov_deg": 62.0,
    }
    session = build_dooaf_session(
        [impact],
        gun_lat=20.4459722,
        gun_lon=72.8629682,
        gun_alt_m=23.516,
        target_lat=20.4459258,
        target_lon=72.8631324,
        target_alt_m=20.691,
        setup_video_marks={
            DOOAF_ROLE_GUN: (0.294, 0.900),
            DOOAF_ROLE_INTENDED: (0.458, 0.485),
        },
        dem_path="output_hh.tif",
    )
    assert session.correction is not None
    assert session.correction.range_gun_to_intended_m > 25.0
    assert session.correction.range_gun_to_impact_m > 25.0
    assert session.correction.range_gun_to_impact_m > 8.5


def test_fire_correction_gun_impact_differ_without_dem_file():
    """Session 185254 — no DEM file but cached terrain AGL on impact row."""
    from vgcs.observe.dooaf import build_dooaf_session

    impact = {
        "kind": "video_mark",
        "dooaf_role": DOOAF_ROLE_IMPACT,
        "target_lat": 20.4459447776644,
        "target_lon": 72.8629922962151,
        "target_alt_m": 20.69,
        "video_x_norm": 0.38125,
        "video_y_norm": 0.5505857294994675,
        "vehicle_lat": 20.4458221,
        "vehicle_lon": 72.8632659,
        "vehicle_heading_deg": 315.0,
        "vehicle_pitch_deg": -7.4,
        "vehicle_roll_deg": 0.0,
        "vehicle_alt_msl_m": 36.0,
        "ekf_rel_alt_m": 1.846,
        "dem_ground_agl_m": 13.58,
        "agl_source": "dem_terrain",
        "gimbal_pitch_deg": -20.0,
        "gimbal_yaw_deg": 0.09,
        "gps_fix_type": 3,
        "camera_hfov_deg": 62.0,
    }
    session = build_dooaf_session(
        [impact],
        gun_lat=20.4458870,
        gun_lon=72.8629401,
        gun_alt_m=20.69,
        target_lat=20.4458676,
        target_lon=72.8631441,
        target_alt_m=22.14,
        setup_video_marks={
            DOOAF_ROLE_GUN: (0.187, 0.853),
            DOOAF_ROLE_INTENDED: (0.366, 0.448),
        },
        dem_path=None,
    )
    assert session.correction is not None
    gt = session.correction.range_gun_to_intended_m
    gi = session.correction.range_gun_to_impact_m
    assert gt > 25.0
    assert gi > 20.0
    assert abs(gt - gi) > 2.0
    assert gi < gt


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
            vehicle_alt_msl_m=120.0,
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
    intended = GeoPoint(12.01, 77.0, 100.0)
    impact = GeoPoint(12.01, 77.001, 95.0)
    session = build_dooaf_session(
        [
            _row(DOOAF_ROLE_GUN, gun.lat, gun.lon),
            _row(DOOAF_ROLE_INTENDED, intended.lat, intended.lon, target_alt_m=100.0),
            _row(DOOAF_ROLE_IMPACT, impact.lat, impact.lon, target_alt_m=95.0),
        ],
    )
    obs = {
        "gimbal_yaw_deg": 28.28,
        "gimbal_pitch_deg": -10.07,
        "dooaf_role": DOOAF_ROLE_IMPACT,
    }
    html = format_dooaf_html_summary(session, observation_row=obs)
    assert "dooaf-fire-corr" in html
    assert "dooaf-client-corr" in html
    assert "dooaf-elevation-summary" in html
    assert "East / West (add)" in html
    assert "dooaf-target-coords" in html
    assert "dooaf-impact-coords" in html
    assert "Yaw right 28.3°" in html
    assert "Pitch down 10.1°" in html
    assert "fc-plan-svg" in html
    assert "fc-gunline-svg" in html
    assert "Plan view (North up)" in html
    assert "fc-diagram-grid-plan" in html
    assert "fc-diagram-grid-maps" in html
    assert "Start here" in html
    assert "How to read this report" in html
    assert "guide-flow" in html
    assert "guide-card" in html
    assert "Jump to section" in html
    assert "exec-split" in html
    assert "Where the round landed" in html
    assert "What to add on the next round" in html
    assert "fc-compass-svg" in html
    assert "fc-story-svg" in html
    assert "Apply correction" in html
    assert "Aim here" in html
    assert "fc-positions-svg" in html
    assert "fc-action-cards" in html
    assert "Exact miss & correction numbers" in html


def test_executive_miss_map_has_icons_and_signed_labels():
    session = build_dooaf_session(
        [
            _row(DOOAF_ROLE_GUN, -35.3618203, 149.1715498),
            _row(DOOAF_ROLE_INTENDED, -35.3612780, 149.1628810),
            _row(
                DOOAF_ROLE_IMPACT,
                -35.3619500,
                149.1627000,
                vehicle_lat=-35.3633516,
                vehicle_lon=149.1652413,
            ),
        ],
    )
    assert session.correction is not None
    html = format_dooaf_html_summary(session)
    assert "exec-miss-map-svg" in html
    assert "Artillery</text>" in html
    assert "Drone</text>" in html
    assert "lr-icon lr-pos" in html
    assert "lr-icon lr-neg" in html
    assert "<tspan fill='#15803d'" in html


def test_plan_view_svg_centers_marks_in_plot():
    """Plan view should fill the plot area instead of clustering in a corner."""
    gun = GeoPoint(12.0, 77.0)
    intended = GeoPoint(12.00068, 77.0)
    impact = GeoPoint(12.00015, 76.99997)
    session = build_dooaf_session(
        [
            _row(DOOAF_ROLE_GUN, gun.lat, gun.lon),
            _row(DOOAF_ROLE_INTENDED, intended.lat, intended.lon),
            _row(DOOAF_ROLE_IMPACT, impact.lat, impact.lon),
        ],
    )
    assert session.correction is not None
    svg = _fire_correction_plan_svg(session, session.correction)
    assert "viewBox='0 0 720 520'" in svg
    xs = [float(x) for x in re.findall(r"<circle cx='([\d.]+)' cy='[\d.]+' r='(?:8|10)'", svg)]
    assert xs, "expected marker circles in plan SVG"
    mid_x = 320.0
    assert all(80.0 < x < 560.0 for x in xs), f"marks should stay centered in plot, got {xs}"
    assert any(abs(x - mid_x) < 120.0 for x in xs), f"marks should span near plot center, got {xs}"


def test_plan_view_stretches_long_range_shot_vertically():
    """788 m gun line with ~65 m miss should fill plan height, not a thin strip."""
    session = build_dooaf_session(
        [
            _row(DOOAF_ROLE_GUN, -35.3618203, 149.1715498),
            _row(DOOAF_ROLE_INTENDED, -35.3612780, 149.1628810),
            _row(
                DOOAF_ROLE_IMPACT,
                -35.3618554,
                149.1627737,
                vehicle_lat=-35.3633516,
                vehicle_lon=149.1652413,
            ),
        ],
    )
    assert session.correction is not None
    svg = _fire_correction_plan_svg(session, session.correction)
    assert "N/S scale exaggerated for visibility" in svg
    gun = re.search(r"Gun</text>", svg)
    impact = re.search(r"<circle cx='([\d.]+)' cy='([\d.]+)' r='10'", svg)
    assert gun and impact
    lines = re.findall(r"x1='([\d.]+)' y1='([\d.]+)' x2='([\d.]+)' y2='([\d.]+)'", svg)
    longest = max(
        lines,
        key=lambda ln: (float(ln[2]) - float(ln[0])) ** 2 + (float(ln[3]) - float(ln[1])) ** 2,
    )
    y_span = abs(float(longest[3]) - float(longest[1]))
    assert y_span > 80.0, f"gun-target line should use vertical space, got {y_span:.1f}px"


def test_gunline_view_matches_compass_readability():
    """Gun-line view uses plain Far/Short/Left/Right labels on a target-centred diagram."""
    session = build_dooaf_session(
        [
            _row(DOOAF_ROLE_GUN, -35.3618203, 149.1715498),
            _row(DOOAF_ROLE_INTENDED, -35.3612780, 149.1628810),
            _row(
                DOOAF_ROLE_IMPACT,
                -35.3619500,
                149.1627000,
                vehicle_lat=-35.3633516,
                vehicle_lon=149.1652413,
            ),
        ],
    )
    assert session.correction is not None
    svg = _fire_correction_gunline_svg(session.correction, session=session)
    assert "viewBox='0 0 400 400'" in svg
    assert "Target</text>" in svg
    assert "Impact</text>" in svg
    assert f"Miss {session.correction.impact_to_intended_m:.1f} m" in svg
    assert " m along" not in svg
    assert re.search(r" m R\"", svg) is None
    assert re.search(r"\d+\.\d+ m (Far|Short|Left|Right)", svg)
    assert "Course correction" not in svg
    target = re.search(r"font-weight='600'>Target</text>", svg)
    impact = re.search(r"<circle cx='([\d.]+)' cy='([\d.]+)' r='10'", svg)
    assert target and impact
    cx, cy = 200.0, 200.0
    ix, iy = float(impact.group(1)), float(impact.group(2))
    miss_px = math.hypot(ix - cx, iy - cy)
    assert miss_px > 80.0, f"miss vector should fill gun-line plot, got {miss_px:.1f}px"


def _small_miss_session():
    """~2 m miss: impact slightly south-east of intended, gun nearby."""
    gun = GeoPoint(12.0, 77.0)
    intended = GeoPoint(12.00005, 77.00005)
    dlat_s = -2.0 / 111_320.0
    dlon_e = 1.0 / (111_320.0 * math.cos(math.radians(intended.lat)))
    impact = GeoPoint(intended.lat + dlat_s, intended.lon + dlon_e)
    return build_dooaf_session(
        [
            _row(DOOAF_ROLE_GUN, gun.lat, gun.lon),
            _row(DOOAF_ROLE_INTENDED, intended.lat, intended.lon),
            _row(DOOAF_ROLE_IMPACT, impact.lat, impact.lon),
        ],
    )


def test_small_miss_uses_schematic_spacing():
    """Sub-12 m misses keep readable diagram spacing; values stay in labels."""
    session = _small_miss_session()
    assert session.correction is not None
    c = session.correction
    assert c.impact_to_intended_m < 5.0

    gunline = _fire_correction_gunline_svg(c, session=session)
    compass = _fire_correction_compass_miss_svg(c, session=session)
    executive = _executive_miss_map_svg(session, c)
    plan = _fire_correction_plan_svg(session, c)

    for svg in (gunline, compass, executive, plan):
        assert "Spacing for readability" in svg

    gun_impact = re.search(r"<circle cx='([\d.]+)' cy='([\d.]+)' r='10'", gunline)
    assert gun_impact
    ix, iy = float(gun_impact.group(1)), float(gun_impact.group(2))
    assert math.hypot(ix - 200.0, iy - 200.0) > 50.0

    exec_impact = re.search(r"<circle cx='([\d.]+)' cy='([\d.]+)' r='9'", executive)
    assert exec_impact
    ex, ey = float(exec_impact.group(1)), float(exec_impact.group(2))
    assert math.hypot(ex - 180.0, ey - 180.0) > 45.0


def test_inset_corner_avoids_clustered_target():
    """Gun/drone inset must not sit on top of the plan-view target marker."""
    from vgcs.observe.dooaf import _fc_pick_inset_corner

    inset_w, inset_h = 78.0, 52.0
    markers = [(71.6, 87.6, 20.0), (661.5, 241.3, 18.0), (58.5, 355.4, 14.0)]
    x0, y0 = _fc_pick_inset_corner(
        inset_w, inset_h, 40.0, 28.0, 664.0, 360.0, markers
    )
    assert _fc_inset_overlaps_markers(x0, y0, inset_w, inset_h, markers) < 1.0
    assert x0 > 500.0


def test_plan_view_inset_not_at_fixed_top_left():
    session = build_dooaf_session(
        [
            _row(DOOAF_ROLE_GUN, -35.3618203, 149.1715498),
            _row(DOOAF_ROLE_INTENDED, -35.3612780, 149.1628810),
            _row(
                DOOAF_ROLE_IMPACT,
                -35.3619500,
                149.1627000,
                vehicle_lat=-35.3633516,
                vehicle_lon=149.1652413,
            ),
        ],
    )
    assert session.correction is not None
    svg = _fire_correction_plan_svg(session, session.correction)
    inset = re.search(
        r"<rect x='([\d.]+)' y='([\d.]+)' width='78.0' height='52.0'[^>]*fill='#fff' fill-opacity='0.92'",
        svg,
    )
    assert inset
    ix, iy = float(inset.group(1)), float(inset.group(2))
    assert ix > 200.0, f"inset should move away from top-left cluster, got ({ix}, {iy})"
    greens = re.findall(r"<circle cx='([\d.]+)' cy='([\d.]+)'[^>]*#22c55e", svg)
    assert greens
    tx, ty = float(greens[0][0]), float(greens[0][1])
    overlap = _fc_inset_overlaps_markers(ix, iy, 78.0, 52.0, [(tx, ty, 20.0)])
    assert overlap < 1.0


def test_positions_map_spreads_overlapping_marks():
    """Gun, target, and impact icons separate when geographically coincident."""
    gun = GeoPoint(12.0, 77.0)
    intended = GeoPoint(12.00005, 77.00005)
    dlat_s = -2.0 / 111_320.0
    dlon_e = 1.0 / (111_320.0 * math.cos(math.radians(intended.lat)))
    impact = GeoPoint(intended.lat + dlat_s, intended.lon + dlon_e)
    session = build_dooaf_session(
        [
            _row(DOOAF_ROLE_GUN, gun.lat, gun.lon),
            _row(DOOAF_ROLE_INTENDED, intended.lat, intended.lon),
            _row(DOOAF_ROLE_IMPACT, impact.lat, impact.lon),
        ],
    )
    svg = _fire_correction_positions_svg(session)
    label_pts = {
        m.group(3): (float(m.group(1)), float(m.group(2)) + 14.0)
        for m in re.finditer(
            r"<text x='([\d.]+)' y='([\d.]+)'[^>]*>(Gun|Target)</text>",
            svg,
        )
    }
    impact_m = re.search(r"<circle cx='([\d.]+)' cy='([\d.]+)' r='10' fill='#dc2626'", svg)
    assert impact_m
    label_pts["Impact"] = (float(impact_m.group(1)), float(impact_m.group(2)))
    assert {"Gun", "Target", "Impact"} <= set(label_pts)
    gx, gy = label_pts["Gun"]
    tx, ty = label_pts["Target"]
    ix, iy = label_pts["Impact"]
    assert math.hypot(gx - tx, gy - ty) > 24.0
    assert math.hypot(ix - tx, iy - ty) > 24.0
