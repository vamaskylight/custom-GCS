from __future__ import annotations

import os
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Protocol

from PySide6.QtCore import QSettings
from urllib.parse import urlparse

from vgcs.skydroid import GimbalStatus, SkydroidTopUdpAdapter
from vgcs.skydroid.transport import begin_skydroid_session_log
from vgcs.siyi import SiyiGimbalUdpAdapter


@dataclass(frozen=True)
class GimbalCommand:
    yaw_deg: float | None = None
    pitch_deg: float | None = None
    roll_deg: float | None = None


def gimbal_nadir_pitch_deg(settings: QSettings | None = None) -> float:
    """Straight-down pitch for ``gimbal_point_down`` (SIYI/Skydroid often use −90°)."""
    st = settings if settings is not None else QSettings("VGCS", "VGCS")
    try:
        return float(st.value("camera/gimbal_nadir_pitch_deg", -90.0) or -90.0)
    except Exception:
        return -90.0


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

    def gimbal_center(self) -> None: ...

    def gimbal_point_down(self) -> None: ...


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

    def gimbal_center(self) -> None:
        return

    def gimbal_point_down(self) -> None:
        return


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

    def gimbal_center(self) -> None:
        try:
            self._t.queue_gimbal_nudge(pitch_deg=0.0, yaw_deg=0.0)
        except Exception:
            return

    def gimbal_point_down(self) -> None:
        pitch = gimbal_nadir_pitch_deg()
        try:
            self._t.queue_gimbal_nudge(pitch_deg=float(pitch), yaw_deg=0.0)
        except Exception:
            return


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

    def set_zoom(self, level: float) -> None:
        setter = getattr(self._primary, "set_zoom", None)
        if callable(setter):
            setter(float(level))
        if not uses_skydroid_top_camera(self):
            self._queue_mavlink_zoom_level(float(level))

    def handle_zoom_step(self, step: int, ui_level: float) -> None:
        handler = getattr(self._primary, "handle_zoom_step", None)
        if callable(handler):
            handler(int(step), float(ui_level))
        if (
            self._mavlink is not None
            and int(step) != 0
            and not uses_skydroid_top_camera(self)
        ):
            try:
                self._mavlink.queue_camera_zoom_step(int(step))
            except Exception:
                pass

    def _queue_mavlink_zoom_level(self, level: float) -> None:
        if self._mavlink is None:
            return
        try:
            self._mavlink.queue_camera_zoom(float(level))
        except Exception:
            pass


ZOOM_MIN = 1.0
ZOOM_MAX_PREVIEW = 4.0
ZOOM_MAX_SKYDROID = 30.0
ZOOM_STEP_PREVIEW = 0.25
# C13 MUL optical zoom uses 0.1× absolute steps on the lens (PROTOCAL M-address).
ZOOM_STEP_SKYDROID = 0.1


class SkydroidCameraControl:
    def __init__(
        self,
        *,
        host: str,
        hosts: list[str] | None = None,
        port: int = 5000,
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
            lvl = max(ZOOM_MIN, min(ZOOM_MAX_SKYDROID, float(level)))
            self._adapter.camera_zoom(lvl)
        except Exception:
            return

    def handle_zoom_step(self, step: int, ui_level: float) -> None:
        try:
            if int(step) == 0:
                return
            # PROTOCAL §4.7 / ZMC step per click (not fractional absolute DZM).
            self._adapter.camera_zoom_step(int(step))
            lvl = max(ZOOM_MIN, min(ZOOM_MAX_SKYDROID, float(ui_level)))
            # Sync absolute burst only on whole-number × (1×, 2×, 3×…).
            if abs(lvl - round(lvl)) < 0.05:
                self._adapter.camera_zoom(round(lvl))
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
        has_yaw = cmd.yaw_deg is not None and abs(float(cmd.yaw_deg)) >= 1e-6
        has_pitch = cmd.pitch_deg is not None and abs(float(cmd.pitch_deg)) >= 1e-6
        if not has_yaw and not has_pitch:
            return
        try:
            st = self._adapter.get_status_cached()
            base_yaw = float(st.yaw_deg) if st is not None and st.yaw_deg is not None else 0.0
            base_pitch = float(st.pitch_deg) if st is not None and st.pitch_deg is not None else 0.0
            yaw_tgt = (base_yaw + float(cmd.yaw_deg)) if has_yaw else None
            pitch_tgt = (base_pitch + float(cmd.pitch_deg)) if has_pitch else None
            self._adapter.set_angle_axes(
                yaw_deg=yaw_tgt,
                pitch_deg=pitch_tgt,
                approach_speed_dps=25.0,
            )
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

    def get_laser_range_m(self) -> float | None:
        """C13 SLR metres — locked value or live read while LRF armed."""
        try:
            return self._adapter.get_laser_range_m()
        except Exception:
            return None

    def set_lrf_armed(self, armed: bool) -> None:
        try:
            self._adapter.set_lrf_armed(bool(armed))
        except Exception:
            return

    def poll_live_laser_range_m(self) -> float | None:
        try:
            return self._adapter.poll_live_slr()
        except Exception:
            return None

    def is_lrf_armed(self) -> bool:
        try:
            return bool(self._adapter.is_lrf_armed())
        except Exception:
            return False

    def is_lrf_locked(self) -> bool:
        try:
            return bool(self._adapter.is_lrf_locked())
        except Exception:
            return False

    def lock_lrf_at_video_norm(
        self,
        u: float,
        v: float,
        *,
        frame_w: int = 1280,
        frame_h: int = 720,
        on_sample: Callable[[float], None] | None = None,
        hold_gimbal: bool | None = None,
        hold_slant_boresight: bool = False,
    ) -> float | None:
        """Lock LRF on video target (normalized 0..1 coords on companion video frame)."""
        try:
            return self._adapter.lock_lrf_target_at_norm(
                float(u),
                float(v),
                frame_w=int(frame_w),
                frame_h=int(frame_h),
                on_sample=on_sample,
                hold_gimbal=hold_gimbal,
                hold_slant_boresight=hold_slant_boresight,
            )
        except Exception:
            return None

    def unlock_lrf(self) -> None:
        try:
            self._adapter.unlock_lrf()
        except Exception:
            return

    def start_target_track_at_video_norm(
        self,
        u: float,
        v: float,
        *,
        frame_w: int = 1280,
        frame_h: int = 720,
    ) -> bool:
        try:
            fn = getattr(self._adapter, "start_visual_track_at_norm", None)
            if callable(fn):
                return bool(
                    fn(float(u), float(v), frame_w=int(frame_w), frame_h=int(frame_h))
                )
        except Exception:
            return False
        return False

    def stop_target_track(self) -> None:
        try:
            fn = getattr(self._adapter, "stop_visual_track", None)
            if callable(fn):
                fn()
        except Exception:
            pass

    def is_target_track_active(self) -> bool:
        try:
            fn = getattr(self._adapter, "is_visual_track_active", None)
            if callable(fn):
                return bool(fn())
        except Exception:
            return False
        return False

    def query_slr_distance_m(self, *, fresh: bool = True) -> float | None:
        try:
            fn = getattr(self._adapter, "query_slr_distance_m", None)
            if callable(fn):
                return fn(fresh=bool(fresh))
        except TypeError:
            try:
                return fn()
            except Exception:
                return None
        except Exception:
            return None
        return None

    def gimbal_center(self) -> None:
        try:
            self.ptz("center")
        except Exception:
            return

    @staticmethod
    def _skydroid_nadir_pitch_candidates() -> list[float]:
        """
        C13 firmware/sign conventions vary in the field.
        - If user set `camera/gimbal_nadir_pitch_deg`, trust it and use only that value.
        - Otherwise keep VGCS legacy default first (−90), then +90 fallback.
        """
        st = QSettings("VGCS", "VGCS")
        try:
            if st.contains("camera/gimbal_nadir_pitch_deg"):
                return [gimbal_nadir_pitch_deg(st)]
        except Exception:
            pass
        return [-90.0, 90.0]

    def gimbal_point_down(self) -> None:
        # Run on a worker so retries/telemetry checks never block UI.
        threading.Thread(target=self._run_skydroid_nadir_sequence, daemon=True).start()

    def _run_skydroid_nadir_sequence(self) -> None:
        """
        C13 field behavior varies by firmware, so do a hybrid:
        1) trigger vendor one-key-down preset (fast reaction),
        2) then converge with absolute angle commands for repeatable accuracy.
        """
        def _status_pitch() -> float | None:
            try:
                st = self._adapter.get_status_cached()
                if st is not None and st.pitch_deg is not None:
                    return float(st.pitch_deg)
            except Exception:
                return None
            return None

        def _status_yaw() -> float:
            try:
                st = self._adapter.get_status_cached()
                if st is not None and st.yaw_deg is not None:
                    return float(st.yaw_deg)
            except Exception:
                return 0.0
            return 0.0

        def _wait_for_pitch_update(timeout_s: float) -> float | None:
            deadline = time.monotonic() + max(0.02, float(timeout_s))
            last = _status_pitch()
            while time.monotonic() < deadline:
                time.sleep(0.05)
                cur = _status_pitch()
                if cur is not None:
                    last = cur
                    return cur
            return last

        base_pitch = _status_pitch()
        yaw_hold = _status_yaw()

        # 1) One-click down preset (C13 docs: PTZ 0x0A)
        if self._adapter._profile.ptz_commands.get("nadir"):
            try:
                self.ptz("nadir")
            except Exception:
                pass

        # 2) Absolute-angle convergence for precision; try both signs deterministically.
        candidates = self._skydroid_nadir_pitch_candidates()
        targets = [float(v) for v in candidates] if candidates else [-90.0]

        # Fast pass: issue immediate command(s) with minimal waiting.
        for target_pitch in targets:
            try:
                self._adapter.set_angle_axes(
                    yaw_deg=yaw_hold,
                    pitch_deg=target_pitch,
                    approach_speed_dps=30.0,
                )
            except Exception:
                continue
            cur_pitch = _wait_for_pitch_update(0.22)
            if cur_pitch is not None and abs(float(cur_pitch) - target_pitch) <= 2.0:
                return

        # Fine pass: 1-2 quick refinements.
        for target_pitch in targets:
            for speed_dps in (20.0, 14.0):
                try:
                    self._adapter.set_angle_axes(
                        yaw_deg=yaw_hold,
                        pitch_deg=target_pitch,
                        approach_speed_dps=speed_dps,
                    )
                    cur_pitch = _wait_for_pitch_update(0.16)
                    if cur_pitch is None:
                        continue
                    if abs(float(cur_pitch) - target_pitch) <= 2.0:
                        return
                except Exception:
                    continue

        # 3) Final movement fallback (same behavior users reported as "it moves").
        # Keep this even with telemetry, because some C13 firmwares reply attitude but
        # still ignore absolute-angle commands intermittently.
        try:
            self.ptz("down")
            time.sleep(0.12)
            self.ptz("stop")
        except Exception:
            pass


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
        try:
            self._adapter.camera_focus_step(int(step))
        except Exception:
            return

    def set_focus(self, level: float) -> None:
        del level
        return

    def set_gimbal(self, cmd: GimbalCommand) -> None:
        has_yaw = cmd.yaw_deg is not None and abs(float(cmd.yaw_deg)) >= 1e-6
        has_pitch = cmd.pitch_deg is not None and abs(float(cmd.pitch_deg)) >= 1e-6
        if not has_yaw and not has_pitch:
            return
        try:
            st = self._adapter.get_status()
            base_yaw = float(st.yaw_deg) if st.yaw_deg is not None else 0.0
            base_pitch = float(st.pitch_deg) if st.pitch_deg is not None else 0.0
            yaw_tgt = base_yaw + (float(cmd.yaw_deg) if has_yaw else 0.0)
            pitch_tgt = base_pitch + (float(cmd.pitch_deg) if has_pitch else 0.0)
            self._adapter.set_angle(yaw=yaw_tgt, pitch=pitch_tgt)
        except Exception:
            return

    def ptz(self, action: str) -> None:
        action_l = str(action or "").strip().lower()
        try:
            if action_l in ("up", "pitch_up"):
                self._adapter.set_rotation_speed(0.0, 5.0)
            elif action_l in ("down", "pitch_down"):
                self._adapter.set_rotation_speed(0.0, -5.0)
            elif action_l in ("left", "yaw_left"):
                self._adapter.set_rotation_speed(-5.0, 0.0)
            elif action_l in ("right", "yaw_right"):
                self._adapter.set_rotation_speed(5.0, 0.0)
            elif action_l in ("stop",):
                self._adapter.set_rotation_speed(0.0, 0.0)
            elif action_l in ("center", "home"):
                self._adapter.set_angle(0.0, 0.0)
                self._adapter.set_rotation_speed(0.0, 0.0)
        except Exception:
            return

    def gimbal_center(self) -> None:
        try:
            self._adapter.set_angle(0.0, 0.0)
            self._adapter.set_rotation_speed(0.0, 0.0)
        except Exception:
            return

    def gimbal_point_down(self) -> None:
        pitch = gimbal_nadir_pitch_deg()
        yaw_hold = 0.0
        try:
            st = self._adapter.get_status()
            if st.yaw_deg is not None:
                yaw_hold = float(st.yaw_deg)
        except Exception:
            pass
        try:
            self._adapter.set_angle(yaw=float(yaw_hold), pitch=float(pitch))
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


def resolve_camera_control_primary(control: object | None) -> object | None:
    """Unwrap ``CompositeGimbalCameraControl`` (and similar) to the real payload backend."""
    if control is None:
        return None
    primary = getattr(control, "_primary", None)
    return primary if primary is not None else control


def uses_skydroid_top_camera(control: object | None) -> bool:
    """True when C13/TOP UDP is the active camera backend (not MAVLink mount zoom)."""
    return isinstance(resolve_camera_control_primary(control), SkydroidCameraControl)


def camera_zoom_limits(control: object | None) -> tuple[float, float, float]:
    """Return ``(min, max, step)`` for the camera rail zoom UI."""
    primary = resolve_camera_control_primary(control)
    if isinstance(primary, SkydroidCameraControl):
        return (ZOOM_MIN, ZOOM_MAX_SKYDROID, ZOOM_STEP_SKYDROID)
    return (ZOOM_MIN, ZOOM_MAX_PREVIEW, ZOOM_STEP_PREVIEW)


def _c13_software_preview_zoom_enabled() -> bool:
    """Until RTSP reflects lens zoom, mirror rail level on preview (set env=0 for RTSP-only)."""
    return os.environ.get("VGCS_C13_SOFTWARE_PREVIEW_ZOOM", "1").strip().lower() not in (
        "0",
        "false",
        "no",
    )


def camera_preview_applies_digital_zoom(
    control: object | None,
    source_id: str = "",
) -> bool:
    """
    Software crop-zoom when RTSP does not yet show lens magnification.

    C13 day: hardware ZMC/DZM steps drive the lens; preview can mirror the rail
    (``VGCS_C13_SOFTWARE_PREVIEW_ZOOM=0`` for RTSP-only once hardware is confirmed).
    Thermal: wide RTSP — always software magnify.
    """
    primary = resolve_camera_control_primary(control)
    if isinstance(primary, SkydroidCameraControl):
        sid = str(source_id or "").strip().lower()
        if sid == "thermal":
            return True
        if sid in ("", "day") and _c13_software_preview_zoom_enabled():
            return True
        return False
    return True


def camera_recording_applies_digital_zoom(source_id: str, control: object | None = None) -> bool:
    """
    Bake preview digital zoom into saved RTSP video when the feed does not carry hardware zoom.

    Visible-light / day streams usually reflect TOP/MAVLink zoom in the RTSP encoder output;
    thermal RTSP stays wide while the UI magnifies via software crop-zoom.
    """
    _ = control
    return str(source_id or "").strip().lower() == "thermal"


def read_companion_laser_range_m(control: object | None) -> float | None:
    """C13 TOP SLR distance in metres (live when armed, frozen when locked)."""
    primary = resolve_camera_control_primary(control)
    getter = getattr(primary, "get_laser_range_m", None)
    if not callable(getter):
        return None
    try:
        dist = getter()
        if dist is None:
            return None
        return float(dist)
    except Exception:
        return None


def poll_companion_laser_range_m(control: object | None) -> float | None:
    """Fresh C13 SLR poll while LRF is armed."""
    primary = resolve_camera_control_primary(control)
    poll = getattr(primary, "poll_live_laser_range_m", None)
    if callable(poll):
        try:
            dist = poll()
            if dist is not None:
                return float(dist)
        except Exception:
            pass
    return read_companion_laser_range_m(control)

