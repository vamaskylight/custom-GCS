from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from urllib.parse import urlparse

from vgcs.skydroid import GimbalStatus, SkydroidTopUdpAdapter
from vgcs.skydroid.transport import begin_skydroid_session_log
from vgcs.siyi import SiyiGimbalUdpAdapter


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

    def handle_zoom_step(self, step: int, ui_level: float) -> None: ...

    def handle_focus_step(self, step: int) -> None: ...

    def set_focus(self, level: float) -> None: ...

    def set_gimbal(self, cmd: GimbalCommand) -> None: ...

    def ptz(self, action: str) -> None: ...

    def set_gimbal_speed(self, yaw: float, pitch: float) -> None: ...

    def camera_trigger_photo(self) -> None: ...

    def camera_toggle_record(self) -> None: ...

    def get_gimbal_status(self) -> GimbalStatus | None: ...


class NoopCameraControl:
    """Default safe implementation: does nothing (keeps UI responsive)."""

    def set_zoom(self, level: float) -> None:
        return

    def handle_zoom_step(self, step: int, ui_level: float) -> None:
        del step, ui_level
        return

    def handle_focus_step(self, step: int) -> None:
        del step
        return

    def set_focus(self, level: float) -> None:
        return

    def set_gimbal(self, cmd: GimbalCommand) -> None:
        return

    def ptz(self, action: str) -> None:
        return

    def set_gimbal_speed(self, yaw: float, pitch: float) -> None:
        return

    def camera_trigger_photo(self) -> None:
        return

    def camera_toggle_record(self) -> None:
        return

    def get_gimbal_status(self) -> GimbalStatus | None:
        return None


class MavlinkCameraControl:
    """
    Stage 1 implementation: send camera control via MAVLink commands.
    """

    def __init__(self, mavlink_thread) -> None:
        self._t = mavlink_thread
        self._recording = False

    def set_zoom(self, level: float) -> None:
        try:
            self._t.queue_camera_zoom(float(level))
        except Exception:
            return

    def handle_zoom_step(self, step: int, ui_level: float) -> None:
        del ui_level
        try:
            self._t.queue_camera_zoom_step(int(step))
        except Exception:
            return

    def handle_focus_step(self, step: int) -> None:
        try:
            self._t.queue_camera_focus_step(int(step))
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

    def ptz(self, action: str) -> None:
        # MAVLink path has no discrete PTZ semantic in M3; keep no-op.
        return

    def set_gimbal_speed(self, yaw: float, pitch: float) -> None:
        try:
            self._t.queue_gimbal_nudge(pitch_deg=float(pitch), yaw_deg=float(yaw))
        except Exception:
            return

    def camera_trigger_photo(self) -> None:
        try:
            self._t.queue_camera_trigger_photo()
        except Exception:
            return

    def camera_toggle_record(self) -> None:
        try:
            if self._recording:
                self._t.queue_camera_video_stop()
                self._recording = False
            else:
                self._t.queue_camera_video_start()
                self._recording = True
        except Exception:
            return

    def get_gimbal_status(self) -> GimbalStatus | None:
        try:
            getter = getattr(self._t, "get_cached_gimbal_status", None)
            if callable(getter):
                return getter()
        except Exception:
            pass
        return None


def resolve_skydroid_host(settings, *, default: str = "192.168.144.108") -> str:
    """Primary Skydroid TOP UDP host (first entry from resolve_skydroid_control_hosts)."""
    hosts = resolve_skydroid_control_hosts(settings, default=default)
    return hosts[0] if hosts else str(default)


def resolve_skydroid_control_hosts(settings, *, default: str = "192.168.144.108") -> list[str]:
    """Hosts to probe for C13 TOP attitude (camera IP, RTSP host, RC Wi-Fi gateway)."""
    from vgcs.skydroid.targets import resolve_skydroid_control_hosts as _resolve

    return _resolve(settings, default=default)


class CompositeGimbalCameraControl:
    """Primary camera backend with optional MAVLink gimbal attitude fallback for M7 reports."""

    def __init__(self, primary: CameraControl, mavlink_thread=None) -> None:
        self._primary = primary
        self._mavlink = mavlink_thread

    def close(self) -> None:
        close = getattr(self._primary, "close", None)
        if callable(close):
            close()

    def get_gimbal_status(self) -> GimbalStatus | None:
        st: GimbalStatus | None = None
        try:
            st = self._primary.get_gimbal_status()
        except Exception:
            st = None
        if st is not None and bool(getattr(st, "supported", False)):
            return st
        if self._mavlink is not None:
            try:
                getter = getattr(self._mavlink, "get_cached_gimbal_status", None)
                if callable(getter):
                    st2 = getter()
                    if st2 is not None and bool(getattr(st2, "supported", False)):
                        yaw = getattr(st2, "yaw_deg", None)
                        pitch = getattr(st2, "pitch_deg", None)
                        if yaw is not None or pitch is not None:
                            if not (
                                abs(float(yaw or 0.0)) < 0.05
                                and abs(float(pitch or 0.0)) < 0.05
                            ):
                                return st2
            except Exception:
                pass
        return st

    def __getattr__(self, name: str):
        return getattr(self._primary, name)


class SkydroidCameraControl:
    def __init__(
        self,
        *,
        host: str,
        hosts: list[str] | None = None,
        port: int = 19856,
        timeout_s: float = 0.25,
        retries: int = 2,
        log_path: str = "",
        profile_id: str = "c13_default",
    ) -> None:
        probe_hosts = list(hosts or [])
        if host and host not in probe_hosts:
            probe_hosts.insert(0, host)
        if log_path:
            begin_skydroid_session_log(
                log_path,
                host=probe_hosts[0] if probe_hosts else host,
                port=int(port),
                profile_id=profile_id,
            )
        self._adapter = SkydroidTopUdpAdapter(
            host=probe_hosts[0] if probe_hosts else host,
            hosts=probe_hosts[1:] if len(probe_hosts) > 1 else None,
            port=port,
            timeout_s=timeout_s,
            retries=retries,
            log_path=log_path,
            profile_id=profile_id,
        )
        self._adapter.start()

    def close(self) -> None:
        self._adapter.stop()

    def set_zoom(self, level: float) -> None:
        try:
            self._adapter.camera_zoom(float(level))
        except Exception:
            return

    def handle_zoom_step(self, step: int, ui_level: float) -> None:
        del step
        try:
            self._adapter.camera_zoom(float(ui_level))
        except Exception:
            return

    def handle_focus_step(self, step: int) -> None:
        try:
            self._adapter.camera_focus_step(int(step))
        except Exception:
            return

    def set_focus(self, level: float) -> None:
        # TOP focus command is not guaranteed for all firmware; use no-op.
        return

    def set_gimbal(self, cmd: GimbalCommand) -> None:
        yaw = float(cmd.yaw_deg) if cmd.yaw_deg is not None else 0.0
        pitch = float(cmd.pitch_deg) if cmd.pitch_deg is not None else 0.0
        try:
            self._adapter.set_angle(yaw=yaw, pitch=pitch)
        except Exception:
            return

    def ptz(self, action: str) -> None:
        try:
            self._adapter.ptz(str(action or ""))
        except Exception:
            return

    def set_gimbal_speed(self, yaw: float, pitch: float) -> None:
        try:
            self._adapter.set_speed(yaw=float(yaw), pitch=float(pitch))
        except Exception:
            return

    def camera_trigger_photo(self) -> None:
        try:
            self._adapter.camera_photo()
        except Exception:
            return

    def camera_toggle_record(self) -> None:
        try:
            self._adapter.camera_record_toggle()
        except Exception:
            return

    def get_gimbal_status(self) -> GimbalStatus | None:
        try:
            st = self._adapter.get_status_cached()
            if st is not None and bool(getattr(st, "supported", False)):
                return st
            return st
        except Exception:
            return None


def resolve_siyi_host(settings, *, default: str = "192.168.144.25") -> str:
    """SIYI companion IP from settings or RTSP stream URL hostname."""
    host = str(settings.value("camera/siyi_host", "") or "").strip()
    if host:
        return host
    for key in ("video/rtsp_day", "video/rtsp_thermal"):
        url = str(settings.value(key, "") or "").strip()
        if url.lower().startswith("rtsp://"):
            parsed = urlparse(url)
            if parsed.hostname:
                return str(parsed.hostname)
    return str(default)


class SiyiCameraControl:
    """SIYI Gimbal SDK over UDP (ZR10 / ZT6 / A8 mini — port 37260)."""

    def __init__(
        self,
        *,
        host: str,
        port: int = 37260,
        timeout_s: float = 0.25,
        retries: int = 1,
    ) -> None:
        self._adapter = SiyiGimbalUdpAdapter(
            host=host,
            port=port,
            timeout_s=timeout_s,
            retries=retries,
        )
        self._adapter.start()

    def close(self) -> None:
        self._adapter.stop()

    def set_zoom(self, level: float) -> None:
        del level
        return

    def handle_zoom_step(self, step: int, ui_level: float) -> None:
        del step, ui_level
        return

    def handle_focus_step(self, step: int) -> None:
        del step
        return

    def set_focus(self, level: float) -> None:
        del level
        return

    def set_gimbal(self, cmd: GimbalCommand) -> None:
        yaw = float(cmd.yaw_deg) if cmd.yaw_deg is not None else 0.0
        pitch = float(cmd.pitch_deg) if cmd.pitch_deg is not None else 0.0
        try:
            self._adapter.set_angle(yaw=yaw, pitch=pitch)
        except Exception:
            return

    def ptz(self, action: str) -> None:
        action_l = str(action or "").strip().lower()
        try:
            if action_l in ("up", "pitch_up"):
                self._adapter.set_rotation_speed(0.0, 30.0)
            elif action_l in ("down", "pitch_down"):
                self._adapter.set_rotation_speed(0.0, -30.0)
            elif action_l in ("left", "yaw_left"):
                self._adapter.set_rotation_speed(-30.0, 0.0)
            elif action_l in ("right", "yaw_right"):
                self._adapter.set_rotation_speed(30.0, 0.0)
            elif action_l in ("stop", "center", "home"):
                self._adapter.set_rotation_speed(0.0, 0.0)
        except Exception:
            return

    def set_gimbal_speed(self, yaw: float, pitch: float) -> None:
        try:
            self._adapter.set_rotation_speed(yaw=float(yaw), pitch=float(pitch))
        except Exception:
            return

    def camera_trigger_photo(self) -> None:
        try:
            self._adapter.camera_photo()
        except Exception:
            return

    def camera_toggle_record(self) -> None:
        try:
            self._adapter.camera_record_toggle()
        except Exception:
            return

    def get_gimbal_status(self) -> GimbalStatus | None:
        try:
            st = self._adapter.get_status()
            if st.supported:
                return st
            return self._adapter.request_attitude()
        except Exception:
            return None

