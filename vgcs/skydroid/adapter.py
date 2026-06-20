from __future__ import annotations

import queue
import re
import threading
import time
from dataclasses import dataclass

from vgcs.skydroid.command_map import SkydroidCommandProfile, get_profile
from vgcs.skydroid.protocol import (
    build_got_target,
    build_slr_query,
    build_sum_track,
    build_top_frame,
    build_zoom_command_burst,
    decode_slr_distance_m,
    extract_attitude_deg,
    parse_top_frame,
)
from vgcs.skydroid.transport import TopUdpTransport

# PROTOCAL.doc: TOP UDP port 5000; lens/system also on 9003 on some C13 builds.
_C13_PROBE_PORTS = (5000, 9003, 19856)
_ZOOM_EXTRA_PORTS = (9003, 19853)
_PROBE_TIMEOUT_S = 0.12

# Motion commands: no UDP reply wait (C13 often has no timely ACK for GSY/GSP/GSM).
_MOTION_COMMANDS = frozenset(
    {
        "GSY",
        "GSP",
        "GSM",
        "GAY",
        "GAP",
        "GAM",
        "PTZ",
        "PT_UP",
        "PT_DOWN",
        "PT_LEFT",
        "PT_RIGHT",
        "PT_CENTER",
        "PT_STOP",
        "PTZ_UP",
        "PTZ_DOWN",
        "PTZ_LEFT",
        "PTZ_RIGHT",
        "PTZ_CENTER",
        "PTZ_STOP",
        "ZMC",
        "FCC",
    }
)


@dataclass(frozen=True)
class GimbalStatus:
    yaw_deg: float | None = None
    pitch_deg: float | None = None
    supported: bool = False
    updated_mono: float = 0.0


class SkydroidTopUdpAdapter:
    def __init__(
        self,
        *,
        host: str = "",
        hosts: list[str] | None = None,
        port: int = 5000,
        timeout_s: float = 0.25,
        retries: int = 1,
        rate_limit_hz: float = 25.0,
        log_path: str = "",
        profile_id: str = "c13_default",
    ) -> None:
        merged: list[str] = []
        for h in [str(host or "").strip(), *(hosts or [])]:
            if h and h not in merged:
                merged.append(h)
        if not merged:
            merged.append("192.168.144.108")
        self._hosts = merged
        self._active_host = merged[0]
        self._transport = TopUdpTransport(
            self._active_host,
            port,
            timeout_s=timeout_s,
            retries=retries,
            log_path=log_path,
        )
        self._queue: queue.Queue[tuple[list[str], dict[str, object], bool]] = queue.Queue(maxsize=128)
        self._status = GimbalStatus()
        self._status_lock = threading.Lock()
        self._profile: SkydroidCommandProfile = get_profile(profile_id)
        self._profile_id = str(profile_id or "c13_default")
        self._running = False
        self._worker: threading.Thread | None = None
        self._status_poller: threading.Thread | None = None
        self._probe_thread: threading.Thread | None = None
        self._probe_finished = threading.Event()
        self._probe_finished.set()
        self._gaa_enabled = False
        self._min_dt = 1.0 / max(1.0, float(rate_limit_hz))
        self._last_send_mono = 0.0
        self._laser_range_m: float | None = None
        self._laser_range_mono: float = 0.0
        self._last_slr_poll_mono: float = 0.0
        self._lrf_locked = False
        self._lrf_lock_external = False
        self._lrf_lock_x = 0
        self._lrf_lock_y = 0
        self._active_port = int(port)
        self._transport.set_datagram_handler(self._maybe_update_status)

    def active_endpoint(self) -> tuple[str, int, str]:
        return (str(self._active_host), int(self._active_port), str(self._profile_id))

    def get_laser_range_m(self, *, max_age_s: float = 4.0) -> float | None:
        """Last C13 SLR reading (TOP tag SLR, metres). None when stale or not locked."""
        with self._status_lock:
            if not self._lrf_locked:
                return None
            dist = self._laser_range_m
            ts = float(self._laser_range_mono or 0.0)
        if dist is None or ts <= 0.0:
            return None
        if time.monotonic() - ts > max(0.5, float(max_age_s)):
            return None
        return float(dist)

    def is_lrf_locked(self) -> bool:
        with self._status_lock:
            return bool(self._lrf_locked)

    def is_lrf_lock_external(self) -> bool:
        """True when lock came from RC/companion track, not a VGCS video pick."""
        with self._status_lock:
            return bool(self._lrf_lock_external)

    def get_lrf_lock_video_norm(
        self,
        *,
        frame_w: int = 1280,
        frame_h: int = 720,
    ) -> tuple[float, float] | None:
        with self._status_lock:
            if not self._lrf_locked:
                return None
            x_px = int(self._lrf_lock_x)
            y_px = int(self._lrf_lock_y)
        fw = max(1, int(frame_w))
        fh = max(1, int(frame_h))
        return (x_px / fw, y_px / fh)

    def lock_lrf_target_at_norm(
        self,
        u: float,
        v: float,
        *,
        frame_w: int = 1280,
        frame_h: int = 720,
    ) -> float | None:
        """GOT + SUM track confirm at video pixel, then read SLR range (blocking — run off UI thread)."""
        if not self.gimbal_telemetry_ok():
            return None
        fw = max(1, int(frame_w))
        fh = max(1, int(frame_h))
        x_px = max(0, min(fw, int(round(max(0.0, min(1.0, float(u))) * fw))))
        y_px = max(0, min(fh, int(round(max(0.0, min(1.0, float(v))) * fh))))
        active_port = int(self._transport._port)
        dist_m: float | None = None
        try:
            got = build_got_target(x_px, y_px, frame_w=fw, frame_h=fh)
            self._transport.send_and_receive(
                got, expect_reply=False, log=True, timeout_s=0.08
            )
            time.sleep(0.06)
            confirm = build_sum_track(confirm=True)
            self._transport.send_and_receive(
                confirm, expect_reply=False, log=True, timeout_s=0.08
            )
            time.sleep(0.12)
            for _ in range(6):
                frame = build_slr_query()
                reply = self._transport.send_and_receive(
                    frame, expect_reply=True, log=True, timeout_s=0.45
                )
                if reply:
                    self._maybe_update_slr(reply)
                with self._status_lock:
                    cand = self._laser_range_m
                if cand is not None:
                    dist_m = float(cand)
                    break
                time.sleep(0.1)
            with self._status_lock:
                if dist_m is not None:
                    self._lrf_locked = True
                    self._lrf_lock_external = False
                    self._lrf_lock_x = x_px
                    self._lrf_lock_y = y_px
                else:
                    self._lrf_locked = False
                    self._lrf_lock_external = False
                    self._laser_range_m = None
                    self._laser_range_mono = 0.0
            return dist_m
        except Exception:
            with self._status_lock:
                self._lrf_locked = False
            return None
        finally:
            self._transport._port = active_port

    def unlock_lrf(self) -> None:
        """Stop visual track and clear locked LRF reading."""
        active_port = int(self._transport._port)
        try:
            stop = build_sum_track(confirm=False)
            self._transport.send_and_receive(
                stop, expect_reply=False, log=True, timeout_s=0.08
            )
        except Exception:
            pass
        finally:
            self._transport._port = active_port
        with self._status_lock:
            self._lrf_locked = False
            self._lrf_lock_external = False
            self._laser_range_m = None
            self._laser_range_mono = 0.0
            self._lrf_lock_x = 0
            self._lrf_lock_y = 0

    def gimbal_telemetry_ok(self) -> bool:
        with self._status_lock:
            return bool(self._status.supported)

    def get_status_cached(self) -> GimbalStatus:
        with self._status_lock:
            return self._status

    def get_status(self) -> GimbalStatus:
        """Non-blocking: returns last background poll result (safe on UI thread)."""
        return self.get_status_cached()

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._probe_finished.clear()
        self._transport.start_listener()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()
        self._status_poller = threading.Thread(target=self._status_loop, daemon=True)
        self._status_poller.start()
        self._probe_thread = threading.Thread(target=self._probe_endpoints_bg, daemon=True)
        self._probe_thread.start()

    def stop(self) -> None:
        self._running = False
        try:
            self._queue.put_nowait(("NOOP", {}, False))
        except Exception:
            pass
        self._transport.close()

    def ptz(self, action: str) -> None:
        commands = self._profile.ptz_commands.get(str(action or "").strip().lower(), [])
        if not commands:
            return
        self._enqueue(commands, {}, False)

    def set_speed(self, yaw: float, pitch: float) -> None:
        y = float(yaw)
        p = float(pitch)
        commands = self._speed_commands_for(y, p)
        if not commands:
            return
        self._enqueue(commands, {"yaw": y, "pitch": p}, False)

    def set_angle(self, yaw: float, pitch: float) -> None:
        self.set_angle_axes(yaw_deg=float(yaw), pitch_deg=float(pitch))

    def set_angle_axes(
        self,
        *,
        yaw_deg: float | None = None,
        pitch_deg: float | None = None,
        approach_speed_dps: float = 25.0,
    ) -> None:
        """Absolute angle on one or both axes (GAY/GAP/GAM)."""
        params: dict[str, object] = {"speed": float(approach_speed_dps)}
        if yaw_deg is not None and pitch_deg is not None:
            self._enqueue(
                ["GAM"],
                {**params, "yaw": float(yaw_deg), "pitch": float(pitch_deg)},
                False,
            )
        elif yaw_deg is not None:
            self._enqueue(["GAY"], {**params, "yaw": float(yaw_deg)}, False)
        elif pitch_deg is not None:
            self._enqueue(["GAP"], {**params, "pitch": float(pitch_deg)}, False)

    @staticmethod
    def _speed_commands_for(yaw: float, pitch: float) -> list[str]:
        """Pick GSY / GSP / GSM so a zero axis does not mask the other (TOP spec)."""
        ay = abs(float(yaw)) >= 1e-6
        ap = abs(float(pitch)) >= 1e-6
        if not ay and not ap:
            return ["GSM"]
        if ay and ap:
            return ["GSM"]
        if ay:
            return ["GSY"]
        if ap:
            return ["GSP"]
        return []

    def camera_record_toggle(self) -> None:
        self._enqueue(self._profile.camera_commands.get("record_toggle", []), {}, True)

    def camera_photo(self) -> None:
        self._enqueue(self._profile.camera_commands.get("photo", []), {}, True)

    def camera_zoom(self, level: float) -> None:
        self._enqueue(["ZOOM_BURST"], {"level": float(level)}, False)

    def camera_focus_step(self, direction: int) -> None:
        key = "focus_in" if int(direction) < 0 else "focus_out"
        cmds = self._profile.camera_commands.get(key, [])
        self._enqueue(cmds, {}, False)
        self._enqueue(["FCC"], {"action": "stop"}, False)

    def poll_attitude_now(self) -> GimbalStatus | None:
        """Blocking poll — background threads only (never call from UI thread)."""
        for host in self._hosts:
            self._transport._host = host
            if self._poll_host_once(host):
                with self._status_lock:
                    if self._status.supported:
                        self._active_host = host
                        return self._status
        return None

    def _poll_host_once(self, host: str) -> bool:
        self._transport.set_route_host(host)
        self._transport._host = host
        if not self._gaa_enabled:
            try:
                self._transport.send_and_receive(
                    build_top_frame("GAA", {"hz": 5}),
                    expect_reply=True,
                    log=False,
                    timeout_s=_PROBE_TIMEOUT_S,
                )
                self._gaa_enabled = True
            except Exception:
                pass
        for status_cmd in self._profile.status_commands:
            try:
                frame = build_top_frame(status_cmd, {})
                reply = self._transport.send_and_receive(
                    frame,
                    expect_reply=True,
                    log=False,
                    timeout_s=_PROBE_TIMEOUT_S,
                )
                self._maybe_update_status(reply)
                with self._status_lock:
                    if self._status.supported:
                        return True
            except Exception:
                continue
        with self._status_lock:
            return bool(self._status.supported)

    def _poll_active_endpoint_once(self) -> None:
        self._transport._host = self._active_host
        self._transport._port = self._active_port
        self._poll_host_once(self._active_host)

    def _probe_endpoints_bg(self) -> None:
        try:
            self._probe_endpoints()
        finally:
            self._probe_finished.set()

    def _probe_endpoints(self) -> None:
        """Find host/port/profile that returns GAA/GAC/GAY attitude (C13 + RC gateway paths)."""
        configured = int(self._transport._port)
        ports: list[int] = []
        for p in (configured, *_C13_PROBE_PORTS):
            if int(p) not in ports:
                ports.append(int(p))
        profiles: list[SkydroidCommandProfile] = [self._profile]
        if self._profile.profile_id != "c13_alt":
            profiles.append(get_profile("c13_alt"))
        tried: list[str] = []
        for profile in profiles:
            prev = self._profile
            self._profile = profile
            for host in self._hosts:
                self._transport.set_route_host(host)
                self._transport._host = host
                for port in ports:
                    self._transport._port = int(port)
                    tried.append(f"{host}:{port}")
                    if self._poll_host_once(host):
                        self._active_host = host
                        self._active_port = int(port)
                        self._profile_id = profile.profile_id
                        print(
                            f"[VGCS:skydroid] gimbal OK via {host}:{port} profile={profile.profile_id}"
                        )
                        return
            self._profile = prev
        self._profile_id = self._profile.profile_id
        self._transport._host = self._active_host
        self._transport._port = configured
        print(
            f"[VGCS:skydroid] gimbal probe failed ({len(tried)} tries). "
            f"See logs/skydroid_top_udp.log — RTSP can work without TOP UDP from this PC."
        )

    @staticmethod
    def _is_motion_command(commands: list[str]) -> bool:
        return bool(commands) and str(commands[0]).upper() in _MOTION_COMMANDS

    def _drop_pending_motion_commands(self) -> None:
        """Keep only the latest motion command — avoids queue backlog and UI lag."""
        pending: list[tuple[list[str], dict[str, object], bool]] = []
        while True:
            try:
                pending.append(self._queue.get_nowait())
            except queue.Empty:
                break
        for item in pending:
            cmds, params, expect_reply = item
            if self._is_motion_command(cmds):
                continue
            try:
                self._queue.put_nowait((cmds, params, expect_reply))
            except queue.Full:
                pass

    def _enqueue(self, commands: list[str], params: dict[str, object], expect_reply: bool) -> None:
        if not commands:
            return
        if not self._running:
            self.start()
        if not expect_reply and self._is_motion_command(commands):
            self._drop_pending_motion_commands()
        try:
            self._queue.put_nowait((list(commands), params, expect_reply))
        except queue.Full:
            try:
                _ = self._queue.get_nowait()
            except Exception:
                pass
            try:
                self._queue.put_nowait((list(commands), params, expect_reply))
            except Exception:
                pass

    def _worker_loop(self) -> None:
        while self._running:
            try:
                commands, params, expect_reply = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if not self._running:
                break
            wait_s = self._min_dt - (time.monotonic() - self._last_send_mono)
            if not expect_reply:
                wait_s = min(wait_s, 0.03)
            if wait_s > 0:
                time.sleep(wait_s)
            self._last_send_mono = time.monotonic()
            if commands and commands[0] == "NOOP":
                continue
            if commands and commands[0] == "ZOOM_BURST":
                try:
                    level = float(params.get("level", 1.0) or 1.0)
                    self._send_zoom_burst(level)
                except Exception:
                    pass
                continue
            for command in commands:
                frame = build_top_frame(command, params)
                try:
                    reply = self._transport.send_and_receive(
                        frame,
                        expect_reply=expect_reply,
                        log=True,
                        timeout_s=0.08 if not expect_reply else None,
                    )
                    if expect_reply and reply:
                        self._maybe_update_status(reply)
                    break
                except Exception:
                    if not expect_reply:
                        break
                    continue

    def _send_zoom_burst(self, level: float) -> None:
        """Fire every known zoom frame on active + alternate TOP UDP ports (no reply wait)."""
        frames = build_zoom_command_burst(level)
        if not frames:
            return
        active_port = int(self._transport._port)
        ports: list[int] = []
        for p in (active_port, *_ZOOM_EXTRA_PORTS):
            if int(p) not in ports:
                ports.append(int(p))
        for port in ports:
            self._transport._port = int(port)
            for frame in frames:
                try:
                    self._transport.send_and_receive(
                        frame,
                        expect_reply=False,
                        log=True,
                        timeout_s=0.05,
                    )
                except Exception:
                    continue
        self._transport._port = active_port

    def _status_loop(self) -> None:
        while self._running:
            if not self._probe_finished.wait(timeout=0.25):
                continue
            self._poll_active_endpoint_once()
            now = time.monotonic()
            slr_active = False
            with self._status_lock:
                if self._lrf_locked:
                    slr_active = True
                elif (
                    self._laser_range_m is not None
                    and time.monotonic() - float(self._laser_range_mono or 0.0) <= 4.0
                ):
                    slr_active = True
            if slr_active and now - float(self._last_slr_poll_mono or 0.0) >= 1.0:
                self._last_slr_poll_mono = now
                self._poll_laser_range_once()
            interval = 1.0 if not self.gimbal_telemetry_ok() else 0.5
            time.sleep(interval)

    def _poll_laser_range_once(self) -> None:
        """C13 single laser rangefinding (PROTOCAL §4.22 SLR read on D-class)."""
        if not self.gimbal_telemetry_ok():
            return
        active_port = int(self._transport._port)
        try:
            frame = build_slr_query()
            reply = self._transport.send_and_receive(
                frame,
                expect_reply=True,
                log=False,
                timeout_s=0.35,
            )
            if reply:
                self._maybe_update_slr(reply)
        except Exception:
            pass
        finally:
            self._transport._port = active_port

    def _maybe_update_slr(self, payload: bytes) -> None:
        dec = parse_top_frame(payload)
        if dec is None or dec.command != "SLR":
            return
        data = str(dec.params.get("slr_data") or "")
        if not data and dec.raw:
            m = re.search(r"SLR([0-9A-Fa-f]{4})", dec.raw, re.IGNORECASE)
            if m:
                data = m.group(1)
        dist_m = decode_slr_distance_m(data)
        if dist_m is None:
            return
        with self._status_lock:
            self._laser_range_m = float(dist_m)
            self._laser_range_mono = time.monotonic()
            if not self._lrf_locked:
                self._lrf_locked = True
                self._lrf_lock_external = True
                if self._lrf_lock_x <= 0 and self._lrf_lock_y <= 0:
                    from vgcs.skydroid.protocol import _LRF_FRAME_H, _LRF_FRAME_W

                    self._lrf_lock_x = int(_LRF_FRAME_W) // 2
                    self._lrf_lock_y = int(_LRF_FRAME_H) // 2

    def _maybe_update_status(self, payload: bytes) -> None:
        dec = parse_top_frame(payload)
        if dec is None:
            return
        if dec.command == "SLR":
            self._maybe_update_slr(payload)
            return
        yaw, pitch = extract_attitude_deg(dec)
        if yaw is None and pitch is None:
            if dec.command not in self._profile.status_response_commands:
                return
        with self._status_lock:
            self._status = GimbalStatus(
                yaw_deg=yaw,
                pitch_deg=pitch,
                supported=(yaw is not None or pitch is not None),
                updated_mono=time.monotonic(),
            )
