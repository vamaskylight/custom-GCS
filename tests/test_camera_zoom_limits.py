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
)


def test_skydroid_zoom_limits_allow_30x() -> None:
    control = object.__new__(SkydroidCameraControl)
    zmin, zmax, zstep = camera_zoom_limits(control)
    assert zmin == 1.0
    assert zmax == ZOOM_MAX_SKYDROID == 30.0
    assert zstep == ZOOM_STEP_SKYDROID == 1.0


def test_preview_zoom_limits_stay_at_4x() -> None:
    zmin, zmax, zstep = camera_zoom_limits(NoopCameraControl())
    assert zmax == ZOOM_MAX_PREVIEW == 4.0
    assert zstep == ZOOM_STEP_PREVIEW == 0.25


def test_preview_digital_zoom_always_applied() -> None:
    control = object.__new__(SkydroidCameraControl)
    assert camera_preview_applies_digital_zoom(control) is True
    assert camera_preview_applies_digital_zoom(NoopCameraControl()) is True


def test_composite_wrapper_uses_skydroid_zoom_limits() -> None:
    inner = object.__new__(SkydroidCameraControl)
    wrapped = CompositeGimbalCameraControl(inner, None)
    zmin, zmax, zstep = camera_zoom_limits(wrapped)
    assert zmax == ZOOM_MAX_SKYDROID == 30.0
    assert zstep == ZOOM_STEP_SKYDROID == 1.0
    assert camera_preview_applies_digital_zoom(wrapped) is True


def test_recording_digital_zoom_only_for_thermal() -> None:
    assert camera_recording_applies_digital_zoom("thermal") is True
    assert camera_recording_applies_digital_zoom("THERMAL") is True
    assert camera_recording_applies_digital_zoom("day") is False
    assert camera_recording_applies_digital_zoom("") is False
