from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass

from vgcs.skydroid.command_map import SkydroidCommandProfile, get_profile
from vgcs.skydroid.protocol import build_top_frame, parse_top_frame
from vgcs.skydroid.transport import TopUdpTransport


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

    def start(self) -> None:
        if self._running:
            return
        self._running = True
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
            return self._status

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
                    reply = self._transport.send_and_receive(frame, expect_reply=expect_reply)
                    self._maybe_update_status(reply)
                    break
                except Exception:
                    continue

    def _status_loop(self) -> None:
        while self._running:
            for status_cmd in self._profile.status_commands:
                try:
                    frame = build_top_frame(status_cmd, {})
                    reply = self._transport.send_and_receive(frame, expect_reply=True)
                    self._maybe_update_status(reply)
                    break
                except Exception:
                    continue
            time.sleep(1.0)

    def _maybe_update_status(self, payload: bytes) -> None:
        dec = parse_top_frame(payload)
        if dec is None:
            return
        if dec.command not in self._profile.status_response_commands:
            return
        yaw = _to_float(dec.params.get("yaw"))
        pitch = _to_float(dec.params.get("pitch"))
        with self._status_lock:
            self._status = GimbalStatus(
                yaw_deg=yaw,
                pitch_deg=pitch,
                supported=(yaw is not None or pitch is not None),
                updated_mono=time.monotonic(),
            )


def _to_float(v: object) -> float | None:
    try:
        if v is None:
            return None
        return float(v)
    except Exception:
        return None

