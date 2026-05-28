"""Gimbal recenter / nadir preset helpers."""

from __future__ import annotations

from unittest.mock import MagicMock

from PySide6.QtCore import QSettings

from vgcs.video.camera_control import (
    MavlinkCameraControl,
    SkydroidCameraControl,
    SiyiCameraControl,
    gimbal_nadir_pitch_deg,
)


def test_gimbal_nadir_pitch_deg_default() -> None:
    st = QSettings("VGCS_test_gimbal", "VGCS_test_gimbal")
    st.remove("camera/gimbal_nadir_pitch_deg")
    assert gimbal_nadir_pitch_deg(st) == -90.0


def test_skydroid_gimbal_center_calls_ptz() -> None:
    cc = SkydroidCameraControl.__new__(SkydroidCameraControl)
    cc._adapter = MagicMock()
    cc.ptz = MagicMock()
    cc.gimbal_center()
    cc.ptz.assert_called_once_with("center")


def test_skydroid_gimbal_point_down_calls_ptz_nadir() -> None:
    cc = SkydroidCameraControl.__new__(SkydroidCameraControl)
    profile = MagicMock()
    profile.ptz_commands = {"nadir": ["PTZ_NADIR"]}
    cc._adapter = MagicMock()
    cc._adapter._profile = profile
    cc.ptz = MagicMock()
    cc.gimbal_point_down()
    cc.ptz.assert_called_once_with("nadir")


def test_mavlink_gimbal_center_queues_zero() -> None:
    t = MagicMock()
    cc = MavlinkCameraControl(t)
    cc.gimbal_center()
    t.queue_gimbal_nudge.assert_called_once_with(pitch_deg=0.0, yaw_deg=0.0)


def test_siyi_gimbal_point_down_sets_nadir_pitch() -> None:
    cc = SiyiCameraControl.__new__(SiyiCameraControl)
    cc._adapter = MagicMock()
    status = MagicMock()
    status.yaw_deg = 12.0
    cc._adapter.get_status.return_value = status
    cc.gimbal_point_down()
    cc._adapter.set_angle.assert_called_once_with(yaw=12.0, pitch=-90.0)
