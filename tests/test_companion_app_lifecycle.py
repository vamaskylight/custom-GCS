"""Companion RTSP behaviour when the app goes to background."""

from vgcs.video.pipeline import (
    _companion_app_is_background,
    _companion_feed_switch_active,
    notify_companion_app_background,
    notify_companion_app_foreground,
    notify_companion_feed_switch,
)


def test_companion_background_flag():
    notify_companion_app_foreground()
    assert not _companion_app_is_background()
    notify_companion_app_background()
    assert _companion_app_is_background()
    notify_companion_app_foreground()
    assert not _companion_app_is_background()


def test_companion_feed_switch_overrides_background_pause():
    notify_companion_app_foreground()
    notify_companion_app_background()
    assert _companion_app_is_background()
    notify_companion_feed_switch(duration_s=5.0)
    assert _companion_feed_switch_active()
    assert not _companion_app_is_background()
