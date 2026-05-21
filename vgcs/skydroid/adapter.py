from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass

from vgcs.skydroid.command_map import SkydroidCommandProfile, get_profile
from vgcs.skydroid.protocol import (
    build_top_frame,
    extract_attitude_deg,
    parse_top_frame,
)
from vgcs.skydroid.transport import TopUdpTransport

_C13_PROBE_PORTS = (5000, 14550, 14551)


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
        host: str,
        port: int = 5000,
        timeout_s: float = 0.25,
        retries: int = 1,
        rate_limit_hz: float = 25.0,
        log_path: str = "",
        profile_id: str = "c13_default",
    ) -> None:
        self._transport = TopUdpTransport(host, port, timeout_s=timeout_s, retries=retries, log_path=log_path)
        self._queue: queue.Queue[tuple[list[str], dict[str, object], bool]] = queue.Queue(maxsize=128)
        self._status = GimbalStatus()
        self._status_lock = threading.Lock()
        self._profile: SkydroidCommandProfile = get_profile(profile_id)
        self._running = False
        self._worker: threading.Thread | None = None
        self._status_poller: threading.Thread | None = None
        self._min_dt = 1.0 / max(1.0, float(rate_limit_hz))
        self._last_send_mono = 0.0
        self._active_port = int(port)
        self._transport.set_datagram_handler(self._maybe_update_status)

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._transport.start_listener()
        self._probe_ports()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()
        self._status_poller = threading.Thread(target=self._status_loop, daemon=True)
        self._status_poller.start()

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
        self._enqueue(commands, {}, True)

    def set_speed(self, yaw: float, pitch: float) -> None:
        self._enqueue(self._profile.speed_commands, {"yaw": float(yaw), "pitch": float(pitch)}, True)

    def set_angle(self, yaw: float, pitch: float) -> None:
        self._enqueue(self._profile.angle_commands, {"yaw": float(yaw), "pitch": float(pitch)}, True)

    def camera_record_toggle(self) -> None:
        self._enqueue(self._profile.camera_commands.get("record_toggle", []), {}, True)

    def camera_photo(self) -> None:
        self._enqueue(self._profile.camera_commands.get("photo", []), {}, True)

    def camera_zoom(self, level: float) -> None:
        self._enqueue(self._profile.camera_commands.get("zoom", []), {"level": float(level)}, True)

    def camera_focus_step(self, direction: int) -> None:
        key = "focus_in" if int(direction) < 0 else "focus_out"
        self._enqueue(self._profile.camera_commands.get(key, []), {}, True)

    def get_status(self) -> GimbalStatus:
        with self._status_lock:
            st = self._status
        if not st.supported or (time.monotonic() - float(st.updated_mono or 0.0)) > 2.0:
            self.poll_attitude_now()
            with self._status_lock:
                return self._status
        return st

    def poll_attitude_now(self) -> GimbalStatus | None:
        for status_cmd in self._profile.status_commands:
            try:
                frame = build_top_frame(status_cmd, {})
                reply = self._transport.send_and_receive(
                    frame, expect_reply=True, log=False
                )
                self._maybe_update_status(reply)
                with self._status_lock:
                    if self._status.supported:
                        return self._status
            except Exception:
                continue
        return None

    def _probe_ports(self) -> None:
        """Pick a UDP port that answers GAA/GAC (C13 may use 5000; radio forwards vary)."""
        host = self._transport._host
        if not host:
            return
        configured = int(self._transport._port)
        candidates: list[int] = []
        for p in (configured, *_C13_PROBE_PORTS):
            if p not in candidates:
                candidates.append(int(p))
        for port in candidates:
            self._transport._port = int(port)
            if self.poll_attitude_now() is not None:
                self._active_port = int(port)
                return
        self._transport._port = configured

    def _enqueue(self, commands: list[str], params: dict[str, object], expect_reply: bool) -> None:
        if not commands:
            return
        if not self._running:
            self.start()
        try:
            self._queue.put_nowait((list(commands), params, expect_reply))
        except queue.Full:
            # Drop oldest pressure by consuming one, then requeue.
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
            # Rate limit send path for continuous control.
            wait_s = self._min_dt - (time.monotonic() - self._last_send_mono)
            if wait_s > 0:
                time.sleep(wait_s)
            self._last_send_mono = time.monotonic()
            if commands and commands[0] == "NOOP":
                continue
            for command in commands:
                frame = build_top_frame(command, params)
                try:
                    reply = self._transport.send_and_receive(
                        frame, expect_reply=expect_reply, log=True
                    )
                    self._maybe_update_status(reply)
                    break
                except Exception:
                    continue

    def _status_loop(self) -> None:
        while self._running:
            self.poll_attitude_now()
            time.sleep(0.5)

    def _maybe_update_status(self, payload: bytes) -> None:
        dec = parse_top_frame(payload)
        if dec is None:
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

