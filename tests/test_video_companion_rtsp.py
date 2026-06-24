"""Companion RTSP (Skydroid C13 / SIYI) reconnect policy."""

import os

import numpy as np

from vgcs.video.pipeline import (
    _companion_claim_rtsp_host,
    _companion_decode_max_dims,
    _companion_frame_should_hide,
    _companion_hevc_showall_enabled,
    _companion_release_rtsp_host,
    _companion_transport_after_hevc_glitch,
    _rgb_frame_has_decode_artifacts,
    _rtsp_transport_sequence,
    _rtsp_transport_sequence_with_override,
    _frozen_duplicate_kill_enabled,
    _hevc_stderr_line_indicates_glitch,
    _rgb_frame_looks_hevc_corrupt,
    _rtsp_url_is_companion_rtsp,
    _stall_watchdog_enabled,
    _video_stall_reconnect_s,
    notify_companion_preview_motion,
    set_companion_decode_gate,
)

C13_URL = "rtsp://192.168.144.108:554/stream=1"
SIYI_URL = "rtsp://192.168.144.25:8554/main.264"
LOCAL_URL = "rtsp://127.0.0.1:8554/live"


def test_companion_rtsp_detects_c13_and_siyi():
    assert _rtsp_url_is_companion_rtsp(C13_URL)
    assert _rtsp_url_is_companion_rtsp(SIYI_URL)
    assert not _rtsp_url_is_companion_rtsp(LOCAL_URL)


def test_companion_hevc_gentle_defaults():
    for url in (C13_URL, SIYI_URL):
        assert not _frozen_duplicate_kill_enabled(url)
        assert not _stall_watchdog_enabled(url)
        assert _video_stall_reconnect_s(url) == 12.0
    assert not _companion_hevc_showall_enabled()


def test_companion_frozen_kill_opt_in():
    os.environ["VGCS_COMPANION_FROZEN_RECONNECT"] = "1"
    try:
        assert _frozen_duplicate_kill_enabled(C13_URL)
    finally:
        os.environ.pop("VGCS_COMPANION_FROZEN_RECONNECT", None)


def test_non_companion_still_aggressive():
    assert _frozen_duplicate_kill_enabled(LOCAL_URL)
    assert _stall_watchdog_enabled(LOCAL_URL)
    assert _video_stall_reconnect_s(LOCAL_URL) == 3.0


def test_companion_decode_cap_applies_to_c13():
    os.environ.pop("VGCS_COMPANION_DECODE_MAX_W", None)
    os.environ.pop("VGCS_COMPANION_DECODE_MAX_H", None)
    w, h = _companion_decode_max_dims(C13_URL)
    assert w <= 960
    assert h <= 540
    w2, h2 = _companion_decode_max_dims(LOCAL_URL)
    assert w2 >= 1920 or int(os.environ.get("VGCS_VIDEO_DECODE_MAX_W", "1920") or 1920) == w2


def test_hevc_stderr_glitch_lines():
    assert _hevc_stderr_line_indicates_glitch(
        b"[hevc @ 0x1] Could not find ref with POC 5\n"
    )
    assert _hevc_stderr_line_indicates_glitch(
        b"[hevc @ 0x1] The cu_qp_delta 1094995529 is outside the valid range [-26, 25].\n"
    )
    assert not _hevc_stderr_line_indicates_glitch(b"[h264 @ 0x1] concealing errors\n")


def test_transport_override_prefers_udp_after_glitch():
    os.environ["VGCS_C13_RTSP_UDP_FALLBACK"] = "1"
    try:
        seq = _rtsp_transport_sequence_with_override(C13_URL, "auto", "udp")
        assert seq[0] == "udp"
        assert "tcp" in seq
    finally:
        os.environ.pop("VGCS_C13_RTSP_UDP_FALLBACK", None)


def test_hevc_glitch_flips_udp_to_tcp():
    seq = _rtsp_transport_sequence(C13_URL, "auto")
    assert _companion_transport_after_hevc_glitch("udp", seq) == "tcp"


def test_hevc_glitch_never_flips_tcp_to_udp_on_c13():
    seq = _rtsp_transport_sequence(C13_URL, "auto")
    assert _companion_transport_after_hevc_glitch("tcp", seq) is None


def test_rgb_corrupt_detector():
    h, w = 144, 256
    good = np.zeros((h, w, 3), dtype=np.uint8)
    good[:, :, 1] = 120
    corrupt = good.copy()
    rng = np.random.default_rng(0)
    for y in range(0, h - 16, 16):
        for x in range(0, w - 16, 16):
            if ((x // 16) + (y // 16)) % 3 != 0:
                continue
            corrupt[y : y + 16, x : x + 16] = rng.integers(
                0, 256, (16, 16, 3), dtype=np.uint8
            )
    assert _rgb_frame_looks_hevc_corrupt(corrupt, good)
    assert not _rgb_frame_looks_hevc_corrupt(good, good)
    assert not _rgb_frame_looks_hevc_corrupt(good, None)


def test_rgb_corrupt_detector_ignores_gimbal_pan():
    """Gimbal pan shifts the whole scene — must not freeze preview as 'corrupt'."""
    h, w = 144, 256
    good = np.zeros((h, w, 3), dtype=np.uint8)
    for y in range(h):
        good[y, :, 0] = (y * 5) % 256
        good[y, :, 1] = 80 + (y % 40)
    panned = np.zeros_like(good)
    shift = 48
    panned[:, shift:, :] = good[:, :-shift, :]
    panned[:, :shift, :] = 90
    assert not _rgb_frame_looks_hevc_corrupt(panned, good)


def test_magenta_artifact_detector():
    h, w = 120, 200
    good = np.zeros((h, w, 3), dtype=np.uint8)
    good[:, :, 1] = 100
    torn = good.copy()
    torn[h // 2 :, :, 0] = 240
    torn[h // 2 :, :, 1] = 20
    torn[h // 2 :, :, 2] = 240
    assert _rgb_frame_has_decode_artifacts(torn)
    hide, why = _companion_frame_should_hide(torn, good)
    assert hide and why == "artifact"
    assert not _rgb_frame_has_decode_artifacts(good)


def test_horizontal_band_artifact_detector():
    h, w = 120, 200
    good = np.zeros((h, w, 3), dtype=np.uint8)
    good[:, :, 1] = 90
    torn = good.copy()
    rng = np.random.default_rng(1)
    torn[h // 2 :, :, :] = rng.integers(40, 220, (h - h // 2, w, 3), dtype=np.uint8)
    assert _rgb_frame_has_decode_artifacts(torn)


def test_c13_rtsp_prefers_tcp_only():
    seq = _rtsp_transport_sequence(C13_URL, "auto")
    assert seq == ("tcp",)


def test_c13_rtsp_udp_fallback_opt_in():
    os.environ["VGCS_C13_RTSP_UDP_FALLBACK"] = "1"
    try:
        seq = _rtsp_transport_sequence(C13_URL, "auto")
        assert seq[0] == "tcp"
        assert "udp" in seq
    finally:
        os.environ.pop("VGCS_C13_RTSP_UDP_FALLBACK", None)


def test_partial_bottom_magenta_tear_detected():
    h, w = 120, 200
    frame = np.zeros((h, w, 3), dtype=np.uint8)
    frame[: h // 2, :, 1] = 90
    rng = np.random.default_rng(4)
    noise = rng.integers(0, 256, (h - h // 2, w), dtype=np.uint8)
    frame[h // 2 :, :, 0] = noise
    frame[h // 2 :, :, 1] = noise
    frame[h // 2 :, :, 2] = noise
    assert _rgb_frame_has_decode_artifacts(frame)


def test_structural_tear_seam_detected():
    h, w = 100, 160
    frame = np.zeros((h, w, 3), dtype=np.uint8)
    frame[:60, :, 1] = 100
    rng = np.random.default_rng(5)
    frame[60:, :, :] = rng.integers(0, 256, (h - 60, w, 3), dtype=np.uint8)
    assert _rgb_frame_has_decode_artifacts(frame)


def test_partial_bottom_magenta_patch_detected():
    """Top-half OK + patchy bottom macroblocks must not reach the preview."""
    h, w = 120, 200
    frame = np.zeros((h, w, 3), dtype=np.uint8)
    frame[: h // 2, :, 1] = 100
    rng = np.random.default_rng(3)
    for y in range(h // 2, h, 16):
        for x in range(0, w, 16):
            if rng.random() > 0.35:
                continue
            frame[y : y + 16, x : x + 16, 0] = 230
            frame[y : y + 16, x : x + 16, 1] = 35
            frame[y : y + 16, x : x + 16, 2] = 230
    assert _rgb_frame_has_decode_artifacts(frame)


def test_motion_preview_still_hides_artifact_frames():
    h, w = 120, 200
    good = np.zeros((h, w, 3), dtype=np.uint8)
    good[:, :, 1] = 100
    torn = good.copy()
    torn[h // 2 :, :, 0] = 240
    torn[h // 2 :, :, 1] = 20
    torn[h // 2 :, :, 2] = 240
    notify_companion_preview_motion(duration_s=3.0)
    hide, why = _companion_frame_should_hide(torn, good, motion_preview=True)
    assert hide and why == "artifact"


def test_companion_rtsp_host_single_owner():
    thermal = "rtsp://192.168.144.108:555/stream=2"
    set_companion_decode_gate(lambda sid: sid in ("day", "thermal"))
    try:
        assert _companion_claim_rtsp_host("day", C13_URL)
        assert not _companion_claim_rtsp_host("thermal", thermal)
        _companion_release_rtsp_host("day", C13_URL)
        assert _companion_claim_rtsp_host("thermal", thermal)
    finally:
        _companion_release_rtsp_host("thermal", thermal)
        set_companion_decode_gate(None)


def test_companion_motion_preview_passes_pan_frames():
    h, w = 64, 64
    last = np.zeros((h, w, 3), dtype=np.uint8)
    last[:, :, 1] = 100
    shifted = np.zeros_like(last)
    shift = 4
    shifted[:, shift:, :] = last[:, :-shift, :]
    shifted[:, :shift, :] = last[:, :shift, :]
    hide_still, _ = _companion_frame_should_hide(shifted, last, motion_preview=False)
    assert not hide_still
    notify_companion_preview_motion(duration_s=3.0)
    # Uniform new scene after gimbal slew — must not freeze on stale last-good compare.
    new_scene = np.full((h, w, 3), 140, dtype=np.uint8)
    hide_motion, _ = _companion_frame_should_hide(new_scene, last, motion_preview=True)
    assert not hide_motion
