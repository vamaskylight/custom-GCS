from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class GimbalCommand:
    yaw_deg: float | None = None
    pitch_deg: float | None = None
    roll_deg: float | None = None


class CameraControl(Protocol):
    """
    Stage 1 stub for camera payload control integration.

    Implementations may talk via MAVLink (mount control), vendor SDK (SIYI/Skydroid),
    or a companion computer API. M3 uses this interface as a stable plug-in point.
    """

    def set_zoom(self, level: float) -> None: ...

    def set_focus(self, level: float) -> None: ...

    def set_gimbal(self, cmd: GimbalCommand) -> None: ...


class NoopCameraControl:
    """Default safe implementation: does nothing (keeps UI responsive)."""

    def set_zoom(self, level: float) -> None:
        return

    def set_focus(self, level: float) -> None:
        return

    def set_gimbal(self, cmd: GimbalCommand) -> None:
        return


class MavlinkCameraControl:
    """
    Stage 1 implementation: send camera control via MAVLink commands.
    """

    def __init__(self, mavlink_thread) -> None:
        self._t = mavlink_thread

    def set_zoom(self, level: float) -> None:
        try:
            self._t.queue_camera_zoom(float(level))
        except Exception:
            return

    def set_focus(self, level: float) -> None:
        try:
            self._t.queue_camera_focus(float(level))
        except Exception:
            return

    def set_gimbal(self, cmd: GimbalCommand) -> None:
        try:
            pitch = float(cmd.pitch_deg) if cmd.pitch_deg is not None else 0.0
            yaw = float(cmd.yaw_deg) if cmd.yaw_deg is not None else 0.0
            self._t.queue_gimbal_nudge(pitch_deg=pitch, yaw_deg=yaw)
        except Exception:
            return

