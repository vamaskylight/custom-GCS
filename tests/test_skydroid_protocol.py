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


def test_encode_speed_uses_0_5_deg_per_s_units() -> None:
    from vgcs.skydroid.protocol import encode_speed_2char

    # 5.0 deg/s -> 10 -> 0x0A (PROTOCAL.doc V1.1.6)
    assert encode_speed_2char(5.0) == "0A"
    # -5.0 deg/s -> -10 -> 0xF6
    assert encode_speed_2char(-5.0) == "F6"


def test_gsy_frame_5_deg_per_s() -> None:
    frame = build_top_frame("GSY", {"yaw": 5.0})
    assert frame == b"#TPUG2wGSY0A70"


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
    from vgcs.skydroid.protocol import (
        build_slr_query,
        build_slr_trigger,
        decode_slr_distance_m,
        parse_slr_distance_from_payload,
        parse_top_frame,
        slr_raw_hex,
    )

    assert build_slr_query() == b"#TPUD2rSLR0055"
    assert build_slr_trigger() == b"#TPUD2wSLR015B"
    dec = parse_top_frame(b"#TPUD4rSLR0005BC")
    assert dec is not None
    assert dec.command == "SLR"
    assert decode_slr_distance_m("0005") is None  # below 5 m minimum (0x32 dm)
    assert decode_slr_distance_m("0032") == 5.0
    assert decode_slr_distance_m("0064") == 10.0
    assert decode_slr_distance_m("0140") == 32.0
    assert decode_slr_distance_m("0208") == 52.0
    assert parse_slr_distance_from_payload(b"#TPUD4rSLR0140BC") == 32.0
    assert parse_slr_distance_from_payload(b"#TPUD4rSLR0208BC") == 52.0
    assert slr_raw_hex(b"#TPUD4rSLR0208BC") == "0208"


def test_build_got_and_sum_match_protocal_doc() -> None:
    from vgcs.skydroid.protocol import build_got_target, build_sum_track

    # §3.3.5 GOT — 640×360 on 1280×720 frame (variable #tp, 8 data chars).
    assert build_got_target(640, 360) == b"#tpUG8wGOT02800168D5"
    assert build_sum_track(confirm=True) == b"#TPUG2wSUM0162"
    assert build_sum_track(confirm=False) == b"#TPUG2wSUM0061"


def test_zmc_zoom_in_udp_pm_format() -> None:
    frame = build_top_frame("ZMC", {"action": "in"})
    assert frame.decode("ascii") == "#tpPM2wZMC0299"


def test_dzm_step_zoom_in() -> None:
    frame = build_top_frame("DZM_STEP", {"action": "in"})
    assert b"DZM" in frame
    assert b"000C" in frame


def test_slr_still_settling_detects_drift() -> None:
    from vgcs.skydroid.adapter import SkydroidTopUdpAdapter

    rising = [45.0, 47.0, 49.0, 50.5, 51.5, 52.0, 52.1, 52.2]
    assert SkydroidTopUdpAdapter._slr_still_settling(rising, 2.0) is True
    assert SkydroidTopUdpAdapter._slr_still_settling(rising, 3.5) is True
    assert SkydroidTopUdpAdapter._slr_still_settling(rising, 5.0) is False

    flat = [52.0, 52.1, 52.1, 52.2, 52.2, 52.2, 52.2, 52.2]
    assert SkydroidTopUdpAdapter._slr_still_settling(flat, 5.0) is False


def test_slr_post_move_samples() -> None:
    from vgcs.skydroid.adapter import SkydroidTopUdpAdapter

    samples = [81.4, 81.4, 81.4, 40.1, 42.1, 42.2]
    post = SkydroidTopUdpAdapter._slr_post_move_samples(samples, 81.4)
    assert post == [40.1, 42.1, 42.2]
    assert SkydroidTopUdpAdapter._slr_samples_moved_from_baseline(samples, 81.4) is True
    assert SkydroidTopUdpAdapter._slr_samples_moved_from_baseline([52.2, 52.3], 52.2) is False
    assert SkydroidTopUdpAdapter._slr_samples_moved_from_baseline([52.2], None) is False
    assert SkydroidTopUdpAdapter._slr_moved_from_baseline(52.0, None) is False


def test_gimbal_attitude_moved() -> None:
    from vgcs.skydroid.adapter import SkydroidTopUdpAdapter

    assert (
        SkydroidTopUdpAdapter._gimbal_attitude_moved((10.0, 5.0), (10.5, 5.0)) is True
    )
    assert (
        SkydroidTopUdpAdapter._gimbal_attitude_moved((10.0, 5.0), (10.0, 5.0)) is False
    )
    assert SkydroidTopUdpAdapter._gimbal_attitude_moved(None, (1.0, 2.0)) is None


def test_pixel_boresight_offset_deg() -> None:
    from vgcs.skydroid.adapter import SkydroidTopUdpAdapter

    dy, dp = SkydroidTopUdpAdapter._pixel_boresight_offset_deg(640, 360)
    assert abs(dy) < 0.01 and abs(dp) < 0.01
    dy2, dp2 = SkydroidTopUdpAdapter._pixel_boresight_offset_deg(676, 496)
    assert dy2 > 0.5
    assert dp2 > 2.0


def test_gimbal_total_move_deg() -> None:
    from vgcs.skydroid.adapter import SkydroidTopUdpAdapter

    deg = SkydroidTopUdpAdapter._gimbal_total_move_deg((-24.0, 0.0), (-14.0, 7.0))
    assert deg > 10.0


def test_angle_err_deg() -> None:
    from vgcs.skydroid.adapter import SkydroidTopUdpAdapter

    assert SkydroidTopUdpAdapter._angle_err_deg(10.0, 0.0) == 10.0
    assert SkydroidTopUdpAdapter._angle_err_deg(-5.0, 10.0) == -15.0
    assert abs(abs(SkydroidTopUdpAdapter._angle_err_deg(85.0, -85.0)) - 10.0) < 0.01


def test_axis_burst_duration_undershoots() -> None:
    from vgcs.skydroid.adapter import SkydroidTopUdpAdapter

    dur = SkydroidTopUdpAdapter._axis_burst_duration_s(10.6, 3.5)
    assert dur <= 1.8
    assert dur < 10.6 / 3.5


def test_gimbal_aim_ok_rejects_overshoot() -> None:
    from vgcs.skydroid.adapter import SkydroidTopUdpAdapter

    ok = SkydroidTopUdpAdapter._gimbal_aim_ok(
        SkydroidTopUdpAdapter(),
        (10.0, 0.0),
        (90.0, 26.0),
        yaw_tgt=-5.0,
        pitch_tgt=2.0,
        dyaw=-15.0,
        dpitch=-2.0,
    )
    assert ok is False


def test_slr_median_and_converged() -> None:
    from vgcs.skydroid.adapter import SkydroidTopUdpAdapter

    assert SkydroidTopUdpAdapter._slr_median([52.0, 52.2, 56.0]) == 52.2
    stable = [55.8, 56.0, 56.1, 56.0, 56.0]
    assert SkydroidTopUdpAdapter._slr_converged(stable) == 56.0
    climbing = [54.0, 55.0, 56.0, 57.0, 58.0]
    assert SkydroidTopUdpAdapter._slr_converged(climbing) is None


def test_try_accept_stable_slr_accepts_converged_samples() -> None:
    from vgcs.skydroid.adapter import SkydroidTopUdpAdapter

    adapter = SkydroidTopUdpAdapter()
    samples = [51.8, 52.0, 52.1, 52.0, 52.0]
    got = adapter._try_accept_stable_slr(samples, elapsed=2.0)
    assert got is not None
    assert abs(float(got) - 52.0) < 0.5


def test_try_accept_stable_slr_rejects_climbing() -> None:
    from vgcs.skydroid.adapter import SkydroidTopUdpAdapter

    adapter = SkydroidTopUdpAdapter()
    climbing = [30.0, 40.0, 50.0, 51.0, 52.0]
    got = adapter._try_accept_stable_slr(climbing, elapsed=2.0)
    assert got is None


def test_gsy_yaw_rate_inverted_on_c13() -> None:
    from vgcs.skydroid.adapter import SkydroidTopUdpAdapter

    assert SkydroidTopUdpAdapter._gsy_yaw_rate_for_offset(5.0, 3.0) == -3.0
    assert SkydroidTopUdpAdapter._gsy_yaw_rate_for_offset(-5.0, 3.0) == 3.0


def test_try_accept_gimbal_slew_slr_requires_range_move() -> None:
    from vgcs.skydroid.adapter import SkydroidTopUdpAdapter

    adapter = SkydroidTopUdpAdapter()
    samples = [29.8, 30.0, 30.0, 29.9, 30.0]
    got = adapter._try_accept_gimbal_slew_slr(
        samples,
        30.1,
        (-42.0, 0.0),
        (-47.0, 0.0),
        gimbal_slew_mono=0.0,
        yaw_tgt=-47.0,
        pitch_tgt=6.9,
        dyaw=-5.0,
        dpitch=-6.9,
    )
    assert got is None


def test_try_accept_lrf_lock_slr_rejects_unchanged_foreground() -> None:
    from vgcs.skydroid.adapter import SkydroidTopUdpAdapter

    adapter = SkydroidTopUdpAdapter()
    samples = [16.1, 16.2, 16.3, 16.3, 16.3]
    got = adapter._try_accept_lrf_lock_slr(
        samples,
        elapsed=8.0,
        pre_slr=16.3,
        align_attempted=True,
        click_offset_deg=14.0,
    )
    assert got is None


def test_try_accept_lrf_lock_slr_accepts_after_align_to_building() -> None:
    from vgcs.skydroid.adapter import SkydroidTopUdpAdapter

    adapter = SkydroidTopUdpAdapter()
    samples = [16.3, 40.0, 52.0, 52.1, 52.0, 52.0, 52.1]
    got = adapter._try_accept_lrf_lock_slr(
        samples,
        elapsed=8.0,
        pre_slr=16.3,
        align_attempted=True,
        click_offset_deg=14.0,
    )
    assert got is not None
    assert abs(float(got) - 52.0) < 0.5
