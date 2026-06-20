"""Companion RTSP (Skydroid C13 / SIYI) reconnect policy."""

import os

from vgcs.video.pipeline import (
    _companion_hevc_showall_enabled,
    _frozen_duplicate_kill_enabled,
    _rgb24_frame_likely_hevc_corrupt,
    _rtsp_url_is_companion_rtsp,
    _stall_watchdog_enabled,
    _video_stall_reconnect_s,
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


def test_companion_hevc_showall_off_by_default():
    assert not _companion_hevc_showall_enabled(C13_URL)
    os.environ["VGCS_COMPANION_HEVC_SHOWALL"] = "1"
    try:
        assert _companion_hevc_showall_enabled(C13_URL)
    finally:
        os.environ.pop("VGCS_COMPANION_HEVC_SHOWALL", None)


def test_corrupt_gray_hevc_frame_detected():
    w, h = 640, 360
    raw = bytes([130, 130, 130] * (w * h))
    assert _rgb24_frame_likely_hevc_corrupt(raw, w, h)
    # Mostly natural color — should not be flagged
    natural = bytearray([40, 90, 30] * (w * h))
    assert not _rgb24_frame_likely_hevc_corrupt(bytes(natural), w, h)
