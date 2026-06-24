from __future__ import annotations

from vgcs.video.camera_control import (
    CompositeGimbalCameraControl,
    NoopCameraControl,
    SkydroidCameraControl,
    ZOOM_MAX_PREVIEW,
    ZOOM_MAX_SKYDROID,
    ZOOM_STEP_PREVIEW,
    ZOOM_STEP_SKYDROID,
    camera_preview_applies_digital_zoom,
    camera_recording_applies_digital_zoom,
    camera_zoom_limits,
    uses_skydroid_top_camera,
)


def test_skydroid_zoom_limits_allow_30x() -> None:
    control = object.__new__(SkydroidCameraControl)
    zmin, zmax, zstep = camera_zoom_limits(control)
    assert zmin == 1.0
    assert zmax == ZOOM_MAX_SKYDROID == 30.0
    assert zstep == ZOOM_STEP_SKYDROID == 0.1


def test_preview_zoom_limits_stay_at_4x() -> None:
    zmin, zmax, zstep = camera_zoom_limits(NoopCameraControl())
    assert zmax == ZOOM_MAX_PREVIEW == 4.0
    assert zstep == ZOOM_STEP_PREVIEW == 0.25


def test_skydroid_day_preview_zoom_fallback_default_on() -> None:
    control = object.__new__(SkydroidCameraControl)
    assert camera_preview_applies_digital_zoom(control, "day") is True
    assert camera_preview_applies_digital_zoom(control, "thermal") is True
    assert camera_preview_applies_digital_zoom(NoopCameraControl(), "day") is True


def test_skydroid_day_rtsp_only_preview_when_env_off(monkeypatch) -> None:
    monkeypatch.setenv("VGCS_C13_SOFTWARE_PREVIEW_ZOOM", "0")
    control = object.__new__(SkydroidCameraControl)
    assert camera_preview_applies_digital_zoom(control, "day") is False
    assert camera_preview_applies_digital_zoom(control, "thermal") is True


def test_composite_wrapper_uses_skydroid_zoom_limits() -> None:
    inner = object.__new__(SkydroidCameraControl)
    wrapped = CompositeGimbalCameraControl(inner, None)
    zmin, zmax, zstep = camera_zoom_limits(wrapped)
    assert zmax == ZOOM_MAX_SKYDROID == 30.0
    assert zstep == ZOOM_STEP_SKYDROID == 0.1
    assert camera_preview_applies_digital_zoom(wrapped, "day") is True
    assert camera_preview_applies_digital_zoom(wrapped, "thermal") is True


def test_composite_skydroid_skips_mavlink_zoom_step() -> None:
    inner = object.__new__(SkydroidCameraControl)
    wrapped = CompositeGimbalCameraControl(inner, object())  # non-None mavlink placeholder
    assert uses_skydroid_top_camera(wrapped) is True


def test_recording_digital_zoom_only_for_thermal() -> None:
    assert camera_recording_applies_digital_zoom("thermal") is True
    assert camera_recording_applies_digital_zoom("THERMAL") is True
    assert camera_recording_applies_digital_zoom("day") is False
    assert camera_recording_applies_digital_zoom("") is False
