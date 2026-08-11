from __future__ import annotations

import math
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
from vgcs.viewpro import ViewproGimbalTcpAdapter


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
        if not uses_skydroid_top_camera(self) and not uses_viewpro_camera(self):
            self._queue_mavlink_zoom_level(float(level))

    def handle_zoom_step(self, step: int, ui_level: float) -> None:
        handler = getattr(self._primary, "handle_zoom_step", None)
        if callable(handler):
            handler(int(step), float(ui_level))
        if (
            self._mavlink is not None
            and int(step) != 0
            and not uses_skydroid_top_camera(self)
            and not uses_viewpro_camera(self)
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
        except Exception as ex:
            print(f"[VGCS:skydroid] set_gimbal_speed failed: {ex!r}")
            return

    def active_camera_profile(self):
        """The active SkydroidCommandProfile (frame size, FOV, command tags)
        for whichever camera is actually connected — M14's AI-follow control
        loop needs this to use the right FOV, unlike the older GOT/SUM aim
        math which still reads C13's hardcoded module constants."""
        try:
            return self._adapter._profile
        except Exception:
            return None

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

    def is_m13_track_start_in_progress(self) -> bool:
        try:
            fn = getattr(self._adapter, "m13_track_start_in_progress", None)
            if callable(fn):
                return bool(fn())
        except Exception:
            return False
        return False

    def refresh_visual_track_keepalive(self) -> None:
        try:
            fn = getattr(self._adapter, "refresh_visual_track_keepalive", None)
            if callable(fn):
                fn()
        except Exception:
            pass

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


@dataclass(frozen=True)
class SiyiCameraProfile:
    """Frame size + lens FOV for M13/M14's pixel<->angle aim math — the SIYI
    counterpart of ``SkydroidCommandProfile`` (see command_map.py), minus the
    command-tag fields since the SIYI SDK is a binary frame protocol, not
    string commands.

    FOV is from the ZR10 manual (DOCS/SIYI-ZR10-HARDWARE-REFERENCE.md):
    vendor-stated diagonal 79.5° / horizontal 71.5° at 1x (widest) zoom.
    fov_v_deg is *derived* from those two via the rectilinear diag/H/V
    relation (tan(D/2)^2 = tan(H/2)^2 + tan(V/2)^2), not vendor-stated
    directly — hence frame_specs_confirmed=False, same honesty flag
    SkydroidCommandProfile uses for unmeasured camera specs.

    ZR10 has a real 10x OPTICAL zoom (unlike C13's fixed lens): this FOV
    only holds at 1x zoom. The aim-correction math has no zoom-level input
    yet, so gimbal follow will under/over-correct at higher zoom levels
    until FOV is made zoom-aware — a known limitation, not a bug.
    """

    profile_id: str = "zr10_default"
    frame_w: int = 1920
    frame_h: int = 1080
    fov_h_deg: float = 71.5
    fov_v_deg: float = 45.2
    frame_specs_confirmed: bool = False


@dataclass(frozen=True)
class ViewproCameraProfile:
    """Frame size + lens FOV for M13/M14's pixel<->angle aim math, Viewpro side.

    **The values below are the COLD-START FALLBACK ONLY.** Unlike the C13 (fixed
    lens) and the ZR10 (fixed profile that is only correct at 1x), this camera
    reports its true current FOV in every D1 status block, so
    ``ViewproCameraControl.active_camera_profile()`` rebuilds this dataclass from
    the live values on every call and these defaults are used only until the
    first status frame lands. That is what ``fov_is_live`` distinguishes.

    Provenance of the fallback numbers, weakest first:
    - ``fov_h_deg = 70.2`` is the field-observed reading from the camera's OWN
      on-screen display at 1.00x zoom (the same observation already cited in
      ``camera_reported_fov_deg``'s docstring), not a vendor spec.
    - ``fov_v_deg = 43.1`` is *derived* from it assuming a 16:9 sensor:
      2*atan(tan(35.1 deg) * 9/16) = 43.14 deg. That is a weaker derivation than
      ``SiyiCameraProfile``'s, which at least combines two vendor-stated numbers.
    - ``frame_w``/``frame_h`` are carried for contract parity with the other two
      profiles only; nothing in the M13/M14 runtime path reads them from here,
      and 1920x1080 is UNVERIFIED for this model.

    ``frame_specs_confirmed`` means something DIFFERENT here than on the other
    two profiles: it is True when the numbers came from the camera itself rather
    than from these defaults. ``fov_is_live`` exists so that distinction stays
    explicit rather than being inferred from a flag whose meaning shifted.
    """

    profile_id: str = "viewpro_default"
    frame_w: int = 1920
    frame_h: int = 1080
    fov_h_deg: float = 70.2
    fov_v_deg: float = 43.1
    frame_specs_confirmed: bool = False
    # Viewpro-only, no counterpart on the other profiles.
    zoom_x: float | None = None
    fov_is_live: bool = False


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
        self._profile = SiyiCameraProfile()
        self._adapter.start()

    def close(self) -> None:
        self._adapter.stop()

    def active_camera_profile(self) -> SiyiCameraProfile:
        """Frame size + FOV for M13/M14's aim math — see SiyiCameraProfile."""
        return self._profile

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


def resolve_viewpro_host(settings, *, default: str = "192.168.2.119") -> str:
    """Viewpro/ViewLink gimbal IP from settings or RTSP stream URL hostname.

    192.168.2.119 is the factory-default gimbal IP printed on the unit's
    label per the ViewLink manual (§2.2.3) — same fallback pattern as
    resolve_siyi_host/resolve_skydroid_host.
    """
    host = str(settings.value("camera/viewpro_host", "") or "").strip()
    if host:
        return host
    for key in ("video/rtsp_day", "video/rtsp_thermal"):
        url = str(settings.value(key, "") or "").strip()
        if url.lower().startswith("rtsp://"):
            parsed = urlparse(url)
            if parsed.hostname:
                return str(parsed.hostname)
    return str(default)


class ViewproCameraControl:
    """Viewpro/ViewLink gimbal — TCP control port (default 2000).

    Video (RTSP) and network/IP settings work like any other RTSP camera.
    Gimbal PTZ (speed + absolute angle), zoom, focus, photo, and record are
    implemented against the real ViewLink wire protocol (obtained from
    Viewpro after-sales support, 2026-07) — see
    vgcs/viewpro/protocol.py for the byte-level encode/decode (verified
    against the vendor's own worked examples) and
    DOCS/VIEWPRO-CAMERA-REFERENCE.md for what's confirmed vs. still
    uncertain (notably: which physical direction Focus+/Focus- move the
    lens has not been field-verified — the vendor doc's own worked examples
    for that pair are internally inconsistent with its command table).

    Absolute zoom/digital-zoom-level and absolute focus-level setpoints
    (``set_zoom``/``set_focus``) are not implemented — only relative
    step/hold control, matching the SIYI adapter's own scope.

    ``send_raw_command`` remains available as a debug/integration escape
    hatch (mirrors the ViewLink app's own "Extended Command" panel) for
    protocol features not yet wired in here (e.g. absolute zoom-to-level,
    tracking, OSD).
    """

    def __init__(
        self,
        *,
        host: str,
        port: int = 2000,
        timeout_s: float = 1.0,
    ) -> None:
        self._adapter = ViewproGimbalTcpAdapter(host=host, port=port, timeout_s=timeout_s)
        self._lrf_locked = False
        self._lrf_lock_error = ""
        self._adapter.start()

    def close(self) -> None:
        self._adapter.stop()

    def set_zoom(self, level: float) -> None:
        del level
        return

    def handle_zoom_step(self, step: int, ui_level: float) -> None:
        del ui_level
        if int(step) != 0:
            self._adapter.camera_zoom(int(step))

    def handle_focus_step(self, step: int) -> None:
        if int(step) != 0:
            self._adapter.camera_focus_step(int(step))

    def set_focus(self, level: float) -> None:
        del level
        return

    def set_gimbal(self, cmd: GimbalCommand) -> None:
        """cmd.pitch_deg is a nudge in the app-wide "positive = tilt UP"
        convention (matches Skydroid GSP / SIYI natively, and MavlinkCameraControl's
        queue_gimbal_nudge). A 2026-07-30 field test showed this matches Viewpro's
        own wire convention for the pitch axis too (raw positive = UP, empirically
        confirmed via the speed command — see set_gimbal_speed's docstring), so no
        sign flip is needed here: add straight through, same as yaw."""
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
        self._adapter.ptz(str(action or ""))

    def set_gimbal_speed(self, yaw: float, pitch: float) -> None:
        """Field test 2026-07-30: the vendor doc's worked absolute-angle examples
        say "pitch positive = DOWN", so an earlier fix here negated pitch to
        convert from the app's "positive = UP" convention. The client then
        reported the pitch button reversed (DOWN button tilted the camera UP) —
        i.e. for the actual SERVO_MANUAL_SPEED (velocity) command on this real
        unit, raw positive pitch is UP, the opposite of what the doc's angle-mode
        example implied. Passing pitch straight through (no flip) is therefore
        the empirically-correct behavior for this command; do not reintroduce a
        negation here without a fresh field confirmation."""
        self._adapter.set_rotation_speed(yaw=float(yaw), pitch=float(pitch))

    def camera_trigger_photo(self) -> None:
        self._adapter.camera_photo()

    def camera_toggle_record(self) -> None:
        self._adapter.camera_record_toggle()

    def get_gimbal_status(self) -> GimbalStatus | None:
        return self._adapter.get_status()

    def gimbal_center(self) -> None:
        self._adapter.center()

    def gimbal_point_down(self) -> None:
        self._adapter.look_down()

    # ---- Laser rangefinder (DOOAF / M8 geo-reference) ----
    #
    # Viewpro's laser fires along the camera boresight (frame centre) — the same
    # "aim the centre crosshair at the target, then lock" workflow the overlay
    # already documents. Unlike the C13 there is no offset laser to iteratively
    # align, so there is no gimbal slewing here at all.
    #
    # LRF is a per-model option on Viewpro and the protocol has no capability
    # query, so this self-detects: a unit without the hardware never reports a
    # range, get_laser_range_m() stays None, and DOOAF falls back to the ray/DEM
    # ground-intersection method exactly as it does today. No regression either way.

    # How far off frame centre a pick may be and still be measured by the laser,
    # as a fraction of frame WIDTH (converted to a real pixel-space radius in
    # lock_lrf_at_video_norm — see there for why width specifically, not a
    # fraction of the diagonal or of each axis independently). Beyond this the
    # boresight is simply not looking at the picked point, and returning the
    # centre's range for it would silently feed DOOAF a distance belonging to a
    # different piece of ground — an unacceptable failure mode in a
    # fire-correction system, so we decline and let the ray/DEM path handle it.
    _LRF_BORESIGHT_TOLERANCE_NORM = 0.12

    # How long to wait, after starting continuous ranging, for the background
    # poll thread to report a fresh distance before giving up.
    #
    # Field bug 2026-08-03: laser_range_once() (single-shot) is fire-and-forget
    # (see adapter docstring — never blocks the GUI thread), so reading
    # query_range_m() immediately after firing raced the camera's own
    # measurement — it returned None on a fresh session, or a stale distance
    # left over from a previous, different target, rather than genuinely
    # waiting. This runs on LrfLockTask's worker thread (map/observation/types.py),
    # never the GUI thread, so blocking here is safe — the same reason the C13
    # lock path's own (longer) alignment wait is safe. Added a wait loop for this.
    #
    # Field bug 2026-08-03 (round 2, REVERTED): tried having the wait loop
    # actively call request_status() itself every ~0.15s (on the theory that
    # passively waiting on the background poll thread's own ~2 Hz schedule was
    # racy — that thread's own status request can block up to the transport's
    # 1.0s socket timeout, so a poll cycle occasionally took ~1.5s). The NEXT
    # field log showed EVERY attempt failing (previously ~50% succeeded) followed
    # by the TCP link being reset by the remote host — correlated closely enough
    # with adding up to ~20 extra requests per lock attempt that this was reverted.
    # Not proven the extra traffic caused the reset (the RTSP video link glitched
    # around the same time too, suggesting a possible broader Wi-Fi hiccup), but
    # a stable link is worth more than a faster lock, so err conservative: back to
    # zero extra requests, passive cache-reads only.
    #
    # Field bug 2026-08-03 (round 3): the REAL fix — lock_lrf_at_video_norm was
    # firing laser_range_once() (single-shot). set_lrf_armed's own docstring
    # already said "Arm = continuous ranging, so the D1 status carries a live
    # distance the lock can read immediately" — continuous mode is what actually
    # makes the range show up reliably. The PROXIMITY panel's arm button uses
    # continuous mode and had partial success; this path used single-shot and
    # went 0-for-many in one full session. Switched to starting continuous
    # ranging (see lock_lrf_at_video_norm) instead, so the passive wait below
    # should now have something worth waiting for.
    # Budget for a shot whose laser is already streaming (warm) vs one that had
    # to start it (cold). Field data 2026-08-04: at a flat 3.0s the session was a
    # near coin-flip, with failures clustering on attempts after a pause and
    # rapid repeat clicks nearly always succeeding — continuous ranging takes a
    # variable time to produce its first measurement, so a cold shot needs a
    # longer budget than a warm one. Kept off the warm path so a genuine
    # no-return (sky, glass, too far) still fails promptly instead of hanging
    # the operator for six seconds.
    _LRF_RANGE_WAIT_S = 3.0
    _LRF_RANGE_WAIT_COLD_S = 6.0
    _LRF_RANGE_POLL_INTERVAL_S = 0.15
    # If a cold shot has produced nothing by here, re-send the start command once
    # in case it was dropped in transit. Deliberately ONE extra command, not the
    # ~20 the reverted round-2 active-polling added — a dropped command is a real
    # possibility on this link, hammering it is not the way to find out.
    _LRF_RESTART_AFTER_S = 2.0

    def _wait_for_fresh_range(self, *, cold_start: bool = True) -> float | None:
        """Block (worker thread only — see lock_lrf_at_video_norm's docstring)
        until an actual range measurement timestamped after this call started
        is available.

        Passively polls the cache — does NOT call request_status() itself; see
        the "round 2, REVERTED" note above for why.

        Keeps waiting rather than returning a None reading: the adapter now
        only timestamps real measurements, but treating "fresh timestamp" and
        "usable distance" as the same thing is exactly what broke this before
        (see the adapter's _update_from_reply comment), so don't give up until
        there is a genuine value or the deadline passes.
        """
        start = time.monotonic()
        budget = self._LRF_RANGE_WAIT_COLD_S if cold_start else self._LRF_RANGE_WAIT_S
        deadline = start + budget
        restart_at = (start + self._LRF_RESTART_AFTER_S) if cold_start else None
        while True:
            try:
                fresh_since = float(self._adapter.last_range_updated_mono())
            except Exception:
                fresh_since = 0.0
            if fresh_since >= start:
                try:
                    dist = self._adapter.query_range_m()
                except Exception:
                    dist = None
                if dist is not None:
                    return dist
            now = time.monotonic()
            if restart_at is not None and now >= restart_at:
                restart_at = None
                try:
                    self._adapter.laser_range_start()
                except Exception:
                    pass
            if now >= deadline:
                return None
            time.sleep(self._LRF_RANGE_POLL_INTERVAL_S)

    def reported_fov_deg(self) -> tuple[float, float] | None:
        """Live (hfov, vfov) in degrees straight from the camera's status frame.

        The C13 has a fixed lens, so DOOAF's single ``observe/camera_hfov_deg``
        setting is always right for it. This camera has a large optical zoom and
        reports its true current FOV continuously, so the static setting is only
        correct at one zoom level — see camera_reported_fov_deg.
        """
        try:
            return self._adapter.query_fov_deg()
        except Exception:
            return None

    def reported_zoom_x(self) -> float | None:
        try:
            return self._adapter.query_zoom_x()
        except Exception:
            return None

    def active_camera_profile(self) -> ViewproCameraProfile:
        """Frame/FOV profile for M13's aim math and M14's follow loop.

        **Rebuilt on every call, deliberately.** ``SiyiCameraControl`` snapshots
        its profile once in ``__init__`` because the ZR10 profile is a static
        vendor spec; doing that here would go stale the instant the operator
        zooms, while still passing any naive ``profile_id`` test.
        ``SkydroidCameraControl.active_camera_profile`` sets the precedent for a
        dynamic accessor — it returns the adapter's currently-live profile.

        Cost is two cached float reads plus a dataclass construction: no I/O and
        no lock, which matters because the follow loop calls this every dispatch.

        Goes through the module-level ``camera_reported_fov_deg`` rather than
        ``self._adapter.query_fov_deg()`` because that helper is the single place
        the 0 < fov < 180 sanity check lives — a zero FOV would divide the
        pixel->angle math into nonsense. Never raises: on any failure the caller
        gets the static fallback, which is wrong-but-sane, rather than an
        exception on the follow path.
        """
        try:
            live = camera_reported_fov_deg(self)
        except Exception:
            live = None
        if live is None:
            return ViewproCameraProfile()
        try:
            zoom = self.reported_zoom_x()
        except Exception:
            zoom = None
        hfov, vfov = float(live[0]), float(live[1])
        self._log_optics_once(hfov, vfov, zoom)
        return ViewproCameraProfile(
            fov_h_deg=hfov,
            fov_v_deg=vfov,
            zoom_x=zoom,
            frame_specs_confirmed=True,
            fov_is_live=True,
        )

    def _log_optics_once(self, hfov: float, vfov: float, zoom: float | None) -> None:
        """Log the optics triple whenever it changes.

        The "implied wide hfov" is the point of this line. hfov and zoom_x come
        from the same D1 block, so if our decode of both is right then
        ``2*atan(tan(hfov/2) * zoom)`` should stay ~constant near the lens's true
        wide-end FOV across every zoom level. That makes a single field session
        enough to confirm the assumption this whole feature rests on — instead of
        discovering a bad decode later as a follow loop that mis-steers.
        """
        sig = (round(hfov, 2), round(vfov, 2), None if zoom is None else round(zoom, 2))
        if sig == getattr(self, "_last_logged_optics", None):
            return
        self._last_logged_optics = sig
        implied = ""
        if zoom is not None and zoom > 0.0 and 0.0 < hfov < 180.0:
            try:
                wide = 2.0 * math.degrees(math.atan(math.tan(math.radians(hfov / 2.0)) * zoom))
                implied = f" (implied wide hfov={wide:.1f})"
            except (ValueError, OverflowError):
                implied = ""
        zoom_txt = "?" if zoom is None else f"{zoom:.1f}"
        print(
            f"[VGCS:viewpro] optics zoom={zoom_txt}x hfov={hfov:.1f} vfov={vfov:.1f}{implied}"
        )

    def get_laser_range_m(self) -> float | None:
        try:
            return self._adapter.query_range_m()
        except Exception:
            return None

    def poll_live_laser_range_m(self) -> float | None:
        return self.get_laser_range_m()

    def query_slr_distance_m(self, *, fresh: bool = True) -> float | None:
        """Slant range for M13's tracked target.

        **Worker-thread only.** ``fresh=True`` blocks up to
        ``_LRF_RANGE_WAIT_COLD_S`` (6.0s cold) / ``_LRF_RANGE_WAIT_S`` (3.0s
        warm) waiting for a genuinely new measurement, so it must never run on
        the GUI thread — M13 calls it from ``M13RangeTask`` on a QThreadPool
        worker, which is why this is safe there.

        Adds exactly two commands per shot (session begin + end), reusing the
        warm-laser session so a track that ranges repeatedly does not pay the
        cold-start cost every tick. The laser is left running only if the
        operator had already armed it — a track shot must not silently take
        ownership of an emitter the operator did not switch on.

        ``fresh=False`` returns the cached value and sends nothing.
        """
        if not fresh:
            return self.get_laser_range_m()
        was_armed = self.is_lrf_armed()
        try:
            already_streaming = bool(self._adapter.laser_range_begin_session())
        except Exception:
            return None
        try:
            return self._wait_for_fresh_range(cold_start=not already_streaming)
        finally:
            if not was_armed:
                try:
                    self._adapter.laser_range_end_session()
                except Exception:
                    pass

    def set_lrf_armed(self, armed: bool) -> None:
        try:
            self._adapter.set_lrf_armed(bool(armed))
        except Exception:
            return

    def is_lrf_armed(self) -> bool:
        try:
            return bool(self._adapter.is_lrf_armed())
        except Exception:
            return False

    def is_lrf_locked(self) -> bool:
        return bool(getattr(self, "_lrf_locked", False))

    def last_lrf_lock_error(self) -> str:
        """Human-readable reason the most recent lock_lrf_at_video_norm declined,
        '' on success. The generic CameraControl interface only propagates a
        distance-or-None through a worker-thread signal (see LrfLockTask), which
        collapses every decline into the same "LRF failed — retry" caption — fine
        for the C13, where a decline means the alignment slew itself failed, but
        misleading here, where the far more common cause is simply "the pick
        wasn't under the centre crosshair" (this camera has no offset laser to
        align, so it never slews). lrf_mixin's failure handling checks for this
        method and appends the result to the status text when present.
        """
        return str(getattr(self, "_lrf_lock_error", "") or "")

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
        """Range the picked point, or None if it is not under the boresight.

        ``hold_gimbal``/``hold_slant_boresight`` exist for interface parity with
        the C13 path only; they describe its gimbal-alignment behaviour, which
        does not apply to a centre-boresighted laser. ``frame_w``/``frame_h`` ARE
        used, to keep the tolerance check aspect-ratio-correct — see the offset
        calculation below.
        """
        del hold_gimbal, hold_slant_boresight
        # Kept short on purpose: this string doubles as the on-video overlay
        # caption's second line (native_video_overlay.py), which has no text
        # wrapping — a full explanation there renders an oversized box over the
        # video. The full explanation goes to the console print below instead.
        self._lrf_lock_error = ""
        measured = self._lrf_pick_offset_px(u, v, frame_w, frame_h)
        if measured is None:
            self._lrf_lock_error = "Invalid pick"
            return None
        offset_px, tolerance_px, du_px, dv_px = measured
        if offset_px > tolerance_px:
            self._lrf_lock_error = "Aim gimbal so target is under crosshair, then click"
            print(
                f"[VGCS:viewpro] LRF pick {offset_px:.0f}px off centre (dx={du_px:+.0f} "
                f"dy={dv_px:+.0f}, max {tolerance_px:.0f}px) — laser is boresighted to "
                "frame centre; aim the target under the centre marker, then lock. "
                "Falling back to ray/DEM for this pick."
            )
            return None
        # Field bug 2026-08-03 (round 3): fired laser_range_once() (single-shot)
        # here, but set_lrf_armed()'s own docstring already said the quiet part —
        # "Arm = continuous ranging, so the D1 status carries a live distance the
        # lock can read immediately". The PROXIMITY panel's arm button uses
        # continuous mode and had partial success; this path used single-shot and,
        # in one full field session, never once got a reading — strong evidence
        # single-shot alone doesn't reliably populate D1 the way continuous does.
        # Start continuous ranging (only if not already armed by the operator),
        # wait, then stop it again — leaving it running unless the operator
        # already had it on, since it's an eye-safety-relevant emitter, not just
        # a sensor (see set_lrf_armed's docstring).
        was_armed = self.is_lrf_armed()
        try:
            already_streaming = bool(self._adapter.laser_range_begin_session())
        except Exception:
            self._lrf_lock_error = "Laser command failed — check TCP link"
            return None
        fired_at = time.monotonic()
        try:
            dist = self._wait_for_fresh_range(cold_start=not already_streaming)
        finally:
            if not was_armed:
                try:
                    # Delayed stop, not immediate — see _LRF_IDLE_STOP_S in the
                    # adapter for why the stream is kept warm between locks.
                    self._adapter.laser_range_end_session()
                except Exception:
                    pass
        waited_s = time.monotonic() - fired_at
        laser_state = "warm" if already_streaming else "cold"
        if dist is None:
            self._lrf_lock_error = "No range reported — aim at a solid surface and retry"
            # Reports the ACTUAL wait and whether the laser was already streaming:
            # cold-vs-warm is the distinction that separates "laser still spinning
            # up" from "this surface genuinely returns nothing" (sky, glass, too
            # far), and a previous version of this message quoted a fixed timeout
            # it hadn't really waited, which sent the diagnosis the wrong way.
            print(
                f"[VGCS:viewpro] LRF no range after {waited_s:.1f}s ({laser_state} laser) — "
                "no return from this surface (sky/glass/too far), or the camera "
                "has no rangefinder."
            )
            return None
        print(f"[VGCS:viewpro] LRF range {float(dist):.1f} m in {waited_s:.1f}s ({laser_state} laser)")
        self._lrf_locked = True
        if on_sample is not None:
            try:
                on_sample(float(dist))
            except Exception:
                pass
        return float(dist)

    def unlock_lrf(self) -> None:
        self._lrf_locked = False
        try:
            self._adapter.set_lrf_armed(False)
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
        """Start the camera's ONBOARD AI tracker at a normalized video point.

        Returns True only once the camera confirms it engaged (see the adapter);
        a False return is the caller's cue to fall back to software tracking.

        ``frame_w``/``frame_h`` are accepted for interface parity and ignored:
        the tracker's coordinate space is a fixed ±960/±540 regardless of the
        decoded frame size, so the caller's nominal dimensions are irrelevant.
        """
        del frame_w, frame_h
        try:
            return bool(self._adapter.start_visual_track_at_norm(float(u), float(v)))
        except Exception:
            return False

    def is_target_track_active(self) -> bool:
        try:
            return bool(self._adapter.is_visual_track_active())
        except Exception:
            return False

    def is_m13_track_start_in_progress(self) -> bool:
        try:
            return bool(self._adapter.track_start_in_progress())
        except Exception:
            return False

    def stop_target_track(self) -> None:
        """Stop the onboard tracker and re-pin the gimbal."""
        try:
            self._adapter.stop_visual_track()
        except Exception:
            return

    def _lrf_pick_offset_px(
        self, u: float, v: float, frame_w: object, frame_h: object
    ) -> tuple[float, float, float, float] | None:
        """(offset_px, tolerance_px, du_px, dv_px) for a pick, or None if invalid.

        ONE definition of this calculation, deliberately — it encodes a
        field-learned lesson and must not be duplicated. Video is 16:9, not
        square, so comparing raw normalized (0..1) du/dv treats a pixel offset
        BELOW the crosshair as ~1.78x "further" than the same pixel offset
        beside it (720 tall vs 1280 wide). Field report 2026-08-03: an
        operator's click that looked accurate still declined at 0.27 "off"
        while three sloppier ones passed — that asymmetry was why. Scaling into
        real pixels first makes the tolerance round in screen space, matching
        what the crosshair actually looks like to the operator.
        """
        try:
            du = float(u) - 0.5
            dv = float(v) - 0.5
        except (TypeError, ValueError):
            return None
        try:
            fw = max(1.0, float(frame_w))
            fh = max(1.0, float(frame_h))
        except (TypeError, ValueError):
            fw, fh = 1280.0, 720.0
        du_px = du * fw
        dv_px = dv * fh
        offset_px = (du_px * du_px + dv_px * dv_px) ** 0.5
        tolerance_px = self._LRF_BORESIGHT_TOLERANCE_NORM * fw
        return offset_px, tolerance_px, du_px, dv_px

    # Tolerance for the AUTOMATIC M13 range gate — deliberately much tighter
    # than _LRF_BORESIGHT_TOLERANCE_NORM (0.12), which is operator-click slop
    # for a human aiming a crosshair by hand and works out to roughly 8 degrees
    # at this lens's wide end. That is far too loose to assert "the laser hit
    # the tracked object": at 0.12 the beam could be measuring something
    # metres away from the target and M13 would label the distance as the
    # target's. A tracking loop holds its subject to a few thousandths of
    # centre, so this only has to absorb tracking jitter.
    # NOT field-calibrated against the real beam divergence — tune with
    # VGCS_VIEWPRO_M13_RANGE_TOLERANCE_NORM once measured on hardware.
    _M13_RANGE_TOLERANCE_NORM = 0.02

    def _m13_range_tolerance_norm(self) -> float:
        raw = os.environ.get("VGCS_VIEWPRO_M13_RANGE_TOLERANCE_NORM", "").strip()
        if raw:
            try:
                return max(0.0, float(raw))
            except ValueError:
                pass
        return float(self._M13_RANGE_TOLERANCE_NORM)

    def lrf_can_measure_video_norm(
        self, u: float, v: float, *, frame_w: int = 1280, frame_h: int = 720
    ) -> bool:
        """True when (u, v) is close enough to boresight for an AUTOMATIC shot.

        Shares ``lock_lrf_at_video_norm``'s aspect-corrected geometry but NOT
        its tolerance — see _M13_RANGE_TOLERANCE_NORM for why the automatic
        gate must be far tighter than the operator-click one. Silent (no
        logging): the follow loop asks this every tick.
        """
        measured = self._lrf_pick_offset_px(u, v, frame_w, frame_h)
        if measured is None:
            return False
        offset_px, _click_tol_px, _du, _dv = measured
        try:
            fw = max(1.0, float(frame_w))
        except (TypeError, ValueError):
            fw = 1280.0
        return offset_px <= (self._m13_range_tolerance_norm() * fw)

    def lrf_boresight_tolerance_norm(self) -> float:
        """How far off frame centre a pick may be and still be measured.

        Exposed so the M13 range gate stays capability-shaped instead of
        vendor-testing, leaving backends without a boresight constraint
        provably untouched.
        """
        return float(self._LRF_BORESIGHT_TOLERANCE_NORM)

    def send_raw_command(self, payload: bytes) -> bytes:
        """Debug/integration escape hatch — see class docstring."""
        return self._adapter.send_raw_command(payload)


def resolve_camera_control_primary(control: object | None) -> object | None:
    """Unwrap ``CompositeGimbalCameraControl`` (and similar) to the real payload backend."""
    if control is None:
        return None
    primary = getattr(control, "_primary", None)
    return primary if primary is not None else control


def uses_skydroid_top_camera(control: object | None) -> bool:
    """True when C13/TOP UDP is the active camera backend (not MAVLink mount zoom)."""
    return isinstance(resolve_camera_control_primary(control), SkydroidCameraControl)


def uses_siyi_camera(control: object | None) -> bool:
    """True when the SIYI SDK UDP backend (ZR10/ZT6/A8 mini) is active."""
    return isinstance(resolve_camera_control_primary(control), SiyiCameraControl)


def uses_viewpro_camera(control: object | None) -> bool:
    """True when the Viewpro/ViewLink TCP backend is active.

    Viewpro cameras draw their own centre marker in the video feed itself
    (camera-side OSD), so the software's boresight crosshair overlay would
    duplicate it — see ``set_center_reticle_enabled`` on
    ``NativeVideoOverlayLayer``.
    """
    return isinstance(resolve_camera_control_primary(control), ViewproCameraControl)


def camera_lrf_can_measure_video_norm(
    control: object | None, u: float, v: float, *, frame_w: int = 1280, frame_h: int = 720
) -> bool:
    """Whether this backend's laser can actually measure the point at (u, v).

    Only cameras whose laser is constrained to boresight implement
    ``lrf_can_measure_video_norm``; everything else (the C13's steerable laser,
    any backend with no such limit) returns True here and is provably
    untouched by the M13 range gate.
    """
    fn = getattr(resolve_camera_control_primary(control), "lrf_can_measure_video_norm", None)
    if not callable(fn):
        return True
    try:
        return bool(fn(float(u), float(v), frame_w=int(frame_w), frame_h=int(frame_h)))
    except Exception:
        return True


def camera_reported_fov_deg(control: object | None) -> tuple[float, float] | None:
    """Live (hfov, vfov) degrees from the camera itself, or None if it can't report it.

    DOOAF's pixel->angle geo-referencing takes FOV from the single
    ``observe/camera_hfov_deg`` setting. That is exact for a FIXED lens (C13),
    but a zoom lens only matches it at one zoom level — a Viewpro field
    screenshot showed the camera's own OSD reading 70.2 deg at 1.00x while the
    setting default is 62.0, i.e. already wrong before zooming at all, and
    progressively worse zoomed in.

    Backends that genuinely know their current FOV should expose
    ``reported_fov_deg()``; callers fall back to the setting when this is None,
    so nothing changes for cameras that can't report it.
    """
    getter = getattr(resolve_camera_control_primary(control), "reported_fov_deg", None)
    if not callable(getter):
        return None
    try:
        fov = getter()
    except Exception:
        return None
    if not fov:
        return None
    try:
        hfov, vfov = float(fov[0]), float(fov[1])
    except (TypeError, ValueError, IndexError):
        return None
    if hfov <= 0.0 or vfov <= 0.0 or hfov >= 180.0 or vfov >= 180.0:
        return None
    return (hfov, vfov)


def camera_reports_payload_gimbal_attitude(control: object | None) -> bool:
    """True when the backend reads attitude straight off the payload gimbal.

    The distinction that matters for video geo-referencing is payload-direct vs.
    the MAVLink mount fallback — mount attitude does not describe where the
    camera is actually looking, so using it to anchor video marks puts them in
    the wrong place. Skydroid (TOP UDP), SIYI (SDK UDP) and Viewpro (ViewLink
    TCP B1) all report the real gimbal attitude; MAVLink/Noop do not.

    Unknown backends return False on purpose: falling back to the generic
    attitude path is harmless, while wrongly trusting a mount angle is not.
    """
    return isinstance(
        resolve_camera_control_primary(control),
        (SkydroidCameraControl, SiyiCameraControl, ViewproCameraControl),
    )


def supports_m13_track(control: object | None) -> bool:
    """Cameras M13 click-to-track can arm on.

    M13 needs three things from a backend, and this checks for exactly those
    rather than naming vendors:
      1. real payload-gimbal attitude — not the MAVLink mount fallback, whose
         angles do not describe where the camera is actually looking;
      2. a rate command (``set_gimbal_speed``) for the follow loop to actuate;
      3. a profile stating the lens FOV (``active_camera_profile``), without
         which the pixel<->angle math silently inherits another camera's lens.

    The attitude whitelist is the FIRST conjunct on purpose. Probing for
    capabilities alone would grant M13 to any test double — ``MagicMock``
    auto-creates every attribute, so all three probes would succeed — and to
    unknown future backends. Same conservative stance
    ``camera_reports_payload_gimbal_attitude`` documents: falling back is
    harmless, wrongly claiming the capability is not.

    Currently true for Skydroid TOP (firmware GOT+SUM, or C12's software
    AI-follow), SIYI SDK and Viewpro (both always software AI-follow — neither
    has an onboard track command this project drives).
    """
    primary = resolve_camera_control_primary(control)
    if primary is None:
        return False
    if not camera_reports_payload_gimbal_attitude(primary):
        return False
    return callable(getattr(primary, "set_gimbal_speed", None)) and callable(
        getattr(primary, "active_camera_profile", None)
    )


def camera_has_laser_rangefinder(control: object | None) -> bool:
    """False only for camera backends confirmed to have no onboard rangefinder
    (SIYI ZR10 — confirmed absent from its spec sheet). Every other backend,
    including unknown/test doubles, defaults True: the safe assumption is
    "try the SLR fetch", not "assume it's missing", since a fetch that comes
    back empty is harmless while skipping a real rangefinder would silently
    degrade M13 geo-reference accuracy. SIYI ZR10 instead falls back to the
    ray/DEM ground-intersection method
    (vgcs.observe.geo_reference.compute_geo_reference)."""
    return not isinstance(resolve_camera_control_primary(control), SiyiCameraControl)


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
    Viewpro: C1 zoom command drives the real lens and its RTSP already carries
    the optical magnification — cropping on top of that would double-zoom.
    """
    primary = resolve_camera_control_primary(control)
    if isinstance(primary, SkydroidCameraControl):
        sid = str(source_id or "").strip().lower()
        if sid == "thermal":
            return True
        if sid in ("", "day") and _c13_software_preview_zoom_enabled():
            return True
        return False
    if isinstance(primary, ViewproCameraControl):
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


def camera_supports_lrf_lock(control: object | None) -> bool:
    """True when the active backend implements the video-pick LRF lock
    (currently Skydroid and Viewpro). Used to gate the PROXIMITY panel's LRF
    arm/lock UI — that panel was hardcoded to `camera/provider == "skydroid"`,
    which kept it disabled for Viewpro even after ViewproCameraControl grew a
    real lock_lrf_at_video_norm. SIYI (no confirmed rangefinder on ZR10 — see
    camera_has_laser_rangefinder) and MAVLink/Noop correctly stay excluded
    since neither implements the method at all.
    """
    return callable(getattr(resolve_camera_control_primary(control), "lock_lrf_at_video_norm", None))


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

