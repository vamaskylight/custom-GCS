from __future__ import annotations

from vgcs.skydroid.protocol import (
    build_gaa_enable,
    build_gac_query,
    build_top_frame,
    build_tp_frame,
    decode_attitude_field_4char,
    encode_attitude_field_4char,
    extract_attitude_deg,
    parse_top_frame,
    tp_checksum,
)


def test_checksum_doc_examples() -> None:
    assert tp_checksum("#TPUD2wAWB01") == "44"
    assert tp_checksum("#TPUG2wGAA01") == "36"
    assert tp_checksum("#TPUG2wPTZ00") == "6A"
    assert tp_checksum("#tpUD2rIPV00") == "93"


def test_build_gaa_matches_doc() -> None:
    assert build_gaa_enable(1) == b"#TPUG2wGAA0136"


def test_parse_gac_response() -> None:
    raw = b"#TPUGCrGACEC780000000064"
    dec = parse_top_frame(raw)
    assert dec is not None
    assert dec.command == "GAC"
    yaw, pitch = extract_attitude_deg(dec)
    assert yaw is not None and abs(yaw + 50.0) < 0.01
    assert pitch is not None and abs(pitch) < 0.01


def test_attitude_field_roundtrip() -> None:
    field = encode_attitude_field_4char(-50.0)
    assert field == "EC78"
    assert decode_attitude_field_4char(field) == -50.0


def test_build_top_frame_ptz() -> None:
    frame = build_top_frame("PT_UP", {})
    assert frame == b"#TPUG2wPTZ016B"


def test_encode_speed_uses_0_1_deg_per_s_units() -> None:
    from vgcs.skydroid.protocol import encode_speed_2char

    # 5.0 deg/s -> 50 -> 0x32
    assert encode_speed_2char(5.0) == "32"
    # -5.0 deg/s -> -50 -> 0xCE
    assert encode_speed_2char(-5.0) == "CE"


def test_gsy_frame_5_deg_per_s() -> None:
    frame = build_top_frame("GSY", {"yaw": 5.0})
    assert frame == b"#TPUG2wGSY3264"


def test_gsm_stop_both_axes() -> None:
    frame = build_top_frame("GSM", {"yaw": 0.0, "pitch": 0.0})
    assert frame == b"#tpUG4wGSM0000F5"


def test_build_gac_query() -> None:
    frame = build_gac_query()
    assert frame.startswith(b"#TPUG2rGAC")


def test_build_ptz_nadir_one_click_down() -> None:
    """Topotek PTZ 0x0A = one-key downward (C13 manual)."""
    assert build_top_frame("PTZ_NADIR", {}) == b"#TPUG2wPTZ0A7B"


def test_gap_pitch_uses_0_01_degree_units() -> None:
    """-90° must be 0xDCD8 (0.01° units), not 0xFF4C (wrong 0.5° encoding)."""
    frame = build_top_frame("GAP", {"pitch": -90.0, "speed": 25.0})
    assert b"DCD8" in frame
    assert b"FF4C" not in frame


def test_gam_includes_pitch_and_yaw_speed() -> None:
    frame = build_top_frame("GAM", {"yaw": 0.0, "pitch": -90.0, "speed": 25.0})
    assert frame.startswith(b"#tpUG")
    assert b"GAM" in frame
    # yaw 0 + yaw spd + pitch -90 (DCD8) + pitch spd
    assert b"0000" in frame
    assert b"DCD8" in frame


def test_build_dzm_14x_matches_protocal_doc() -> None:
    from vgcs.skydroid.protocol import build_dzm_absolute_zoom

    frame = build_dzm_absolute_zoom(14.0)
    text = frame.decode("ascii")
    assert text == "#tpPD6wDZM00F08C84"


def test_build_mul_24x() -> None:
    from vgcs.skydroid.protocol import build_mul_optical_zoom

    assert build_mul_optical_zoom(24.0).decode("ascii") == "#tpPM4wMUL024003"


def test_zoom_burst_has_three_variants() -> None:
    from vgcs.skydroid.protocol import build_zoom_command_burst

    frames = build_zoom_command_burst(24.0)
    assert len(frames) == 3
    texts = [f.decode("ascii") for f in frames]
    assert any(t.startswith("#tpPD6wDZM") for t in texts)
    assert any(t.startswith("#tpUD6wDZM") for t in texts)
    assert any(t.startswith("#tpPM4wMUL") for t in texts)


def test_cam_zoom_maps_to_dzm_not_legacy() -> None:
    frame = build_top_frame("CAM_ZOOM", {"level": 30.0})
    assert frame.startswith(b"#tpPD")
    assert b"DZM" in frame
    assert b"00F12C" in frame
    assert not frame.startswith(b"$TOP")


def test_build_slr_query_matches_protocal_doc() -> None:
    from vgcs.skydroid.protocol import build_slr_query, decode_slr_distance_m, parse_top_frame

    assert build_slr_query() == b"#TPUD2rSLR0055"
    dec = parse_top_frame(b"#TPUD4rSLR0005BC")
    assert dec is not None
    assert dec.command == "SLR"
    assert decode_slr_distance_m("0005") is None  # below 5 m minimum (0x32 dm)
    assert decode_slr_distance_m("0032") == 5.0
    assert decode_slr_distance_m("0064") == 10.0
    assert decode_slr_distance_m("2710") == 1000.0


def test_zmc_zoom_in_udp_pm_format() -> None:
    frame = build_top_frame("ZMC", {"action": "in"})
    assert frame.decode("ascii") == "#tpPM2wZMC0299"


def test_dzm_step_zoom_in() -> None:
    frame = build_top_frame("DZM_STEP", {"action": "in"})
    assert b"DZM" in frame
    assert b"000C" in frame
