from __future__ import annotations

import threading
import time

from vgcs.skydroid.adapter import GimbalStatus
from vgcs.siyi.protocol import (
    CMD_AUTO_FOCUS,
    CMD_GIMBAL_ANGLE,
    CMD_GIMBAL_ATTITUDE,
    CMD_GIMBAL_ROTATION,
    CMD_MANUAL_FOCUS,
    CMD_MANUAL_ZOOM,
    CMD_PHOTO_RECORD,
    build_request,
    decode_attitude_deg,
    encode_angle_deg,
    encode_auto_focus,
    encode_manual_focus,
    encode_manual_zoom,
    encode_rotation_speed,
    parse_frame,
)
from vgcs.siyi.transport import SiyiUdpTransport


class SiyiGimbalUdpAdapter:
    """SIYI SDK UDP client (ZR10 default port 37260, CMD 0x0D attitude poll)."""

    def __init__(
        self,
        *,
        host: str,
        port: int = 37260,
        timeout_s: float = 0.25,
        retries: int = 1,
        poll_hz: float = 2.0,
    ) -> None:
        self._transport = SiyiUdpTransport(
            host, port, timeout_s=timeout_s, retries=retries
        )
        self._status = GimbalStatus()
        self._status_lock = threading.Lock()
        self._seq = 0
        self._seq_lock = threading.Lock()
        self._running = False
        self._poller: threading.Thread | None = None
        self._poll_dt = 1.0 / max(0.5, float(poll_hz))

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._poller = threading.Thread(target=self._poll_loop, daemon=True)
        self._poller.start()

    def stop(self) -> None:
        self._running = False
        self._transport.close()

    def get_status(self) -> GimbalStatus:
        with self._status_lock:
            return self._status

    def request_attitude(self) -> GimbalStatus | None:
        try:
            frame = self._request(CMD_GIMBAL_ATTITUDE)
            if frame is None:
                return None
            yaw, pitch, _roll = decode_attitude_deg(frame.data)
            st = GimbalStatus(
                yaw_deg=yaw,
                pitch_deg=pitch,
                supported=(yaw is not None or pitch is not None),
                updated_mono=time.monotonic(),
            )
            with self._status_lock:
                self._status = st
            return st
        except Exception:
            return None

    def set_angle(self, yaw: float, pitch: float) -> None:
        data = encode_angle_deg(yaw, pitch)
        self._request(CMD_GIMBAL_ANGLE, data, expect_reply=False)

    def set_rotation_speed(self, yaw: float, pitch: float) -> None:
        data = encode_rotation_speed(yaw, pitch)
        self._request(CMD_GIMBAL_ROTATION, data, expect_reply=False)

    def camera_photo(self) -> None:
        self._request(CMD_PHOTO_RECORD, bytes([0]), expect_reply=False)

    def camera_record_toggle(self) -> None:
        self._request(CMD_PHOTO_RECORD, bytes([2]), expect_reply=False)

    def camera_zoom(self, direction: int) -> None:
        """Manual zoom step: direction > 0 = zoom in, < 0 = zoom out, 0 = stop."""
        self._request(CMD_MANUAL_ZOOM, encode_manual_zoom(direction), expect_reply=False)

    def camera_focus_step(self, direction: int) -> None:
        """Manual focus step: direction > 0 = far (long shot), < 0 = near (close shot)."""
        self._request(CMD_MANUAL_FOCUS, encode_manual_focus(direction), expect_reply=False)
        # Send a stop command immediately after the step so the lens doesn't keep moving.
        self._request(CMD_MANUAL_FOCUS, encode_manual_focus(0), expect_reply=False)

    def camera_auto_focus(self, touch_x: int = 0, touch_y: int = 0) -> None:
        """Trigger one-shot autofocus (ZR10/ZT6/ZR30/ZT30 only)."""
        self._request(CMD_AUTO_FOCUS, encode_auto_focus(touch_x, touch_y), expect_reply=False)

    def _next_seq(self) -> int:
        with self._seq_lock:
            s = self._seq
            self._seq = (self._seq + 1) & 0xFFFF
            return s

    def _request(
        self, cmd_id: int, data: bytes = b"", *, expect_reply: bool = True
    ):
        pkt = build_request(cmd_id, data, seq=self._next_seq())
        if not expect_reply:
            self._transport.send_only(pkt)
            return None
        raw = self._transport.send_and_receive(pkt)
        frame = parse_frame(raw)
        if frame is None:
            return None
        if frame.cmd_id == cmd_id or frame.cmd_id == CMD_GIMBAL_ATTITUDE:
            yaw, pitch, _ = decode_attitude_deg(frame.data)
            if yaw is not None or pitch is not None:
                with self._status_lock:
                    self._status = GimbalStatus(
                        yaw_deg=yaw,
                        pitch_deg=pitch,
                        supported=True,
                        updated_mono=time.monotonic(),
                    )
        return frame

    def _poll_loop(self) -> None:
        while self._running:
            self.request_attitude()
            time.sleep(self._poll_dt)
