"""Companion RTSP behaviour when the app goes to background."""

from vgcs.video.pipeline import (
    _companion_app_is_background,
    notify_companion_app_background,
    notify_companion_app_foreground,
)


def test_companion_background_flag():
    notify_companion_app_foreground()
    assert not _companion_app_is_background()
    notify_companion_app_background()
    assert _companion_app_is_background()
    notify_companion_app_foreground()
    assert not _companion_app_is_background()
