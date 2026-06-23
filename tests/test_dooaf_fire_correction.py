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
