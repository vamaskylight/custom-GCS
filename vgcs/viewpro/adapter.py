"""Viewpro/ViewLink gimbal — TCP control-port client.

Implements the protocol from two vendor documents (obtained 2026-07 via
Viewpro after-sales support):
- "TCP control.pdf" — TCP envelope wrapping the serial protocol.
- "Viewpro Viewlink Serial Command Communication Protocol" V3.4.9 — the
  ViewLink serial frame format and command tables.

See vgcs/viewpro/protocol.py for the wire-format encode/decode, verified
byte-for-byte against every worked example in those documents (checksum
algorithm, gimbal speed/angle encoding, zoom/photo/record C1 bit-packing,
and B1/D1 status decoding all independently confirmed — see that module's
docstrings and DOCS/VIEWPRO-CAMERA-REFERENCE.md for what's confirmed vs.
still uncertain, notably the Focus+/Focus- direction).
"""

from __future__ import annotations

import threading
import time

from vgcs.skydroid import GimbalStatus
from vgcs.viewpro import protocol as vp
from vgcs.viewpro.transport import ViewproTcpTransport

_DEFAULT_SLEW_DPS = 10.0  # matches the manual's own worked speed-control examples
_ZOOM_SPEED = 7  # 1 (slowest) ~ 7 (fastest) per protocol doc
_FOCUS_SPEED = 4  # mid-range; protocol doc has no stated default
_FAILURE_LOG_INTERVAL_S = 5.0  # throttle: jog fires every 80ms, don't flood the console
# C1 zoom (0x08/0x09) is a continuous start command, not a discrete step — the
# protocol has no absolute zoom-to-level command (see DOCS/VIEWPRO-CAMERA-REFERENCE.md).
# The UI's Zoom +/- is a single click with no release event, so without an
# auto-stop a click would run the lens to its physical zoom limit in one shot.
# This duration is a starting point, not field-calibrated against a real lens —
# tune it (faster lens = shorter pulse) once verified against hardware.
_ZOOM_STEP_PULSE_S = 0.15


class ViewproGimbalTcpAdapter:
    """TCP client to the Viewpro gimbal core's control port (default 2000)."""

    def __init__(
        self,
        *,
        host: str,
        port: int = 2000,
        timeout_s: float = 1.0,
        poll_hz: float = 2.0,
    ) -> None:
        self._host = str(host or "")
        self._port = int(port)
        self._transport = ViewproTcpTransport(host, port, timeout_s=timeout_s)
        self._status = GimbalStatus()
        self._status_lock = threading.Lock()
        self._recording = False
        self._last_range_m: float | None = None
        self._running = False
        self._poller: threading.Thread | None = None
        self._poll_dt = 1.0 / max(0.5, float(poll_hz))
        self._last_failure_log_mono = 0.0
        self._logged_first_failure = False
        self._logged_first_success = False
        self._zoom_stop_timer: threading.Timer | None = None
        self._last_gimbal_log_mono = 0.0

    def _log_failure(self, where: str, exc: Exception) -> None:
        """Throttled diagnostic — without this, a bad host/port or a dead
        camera link fails completely silently (jog fires every 80ms; an
        unthrottled print per failure would flood the console)."""
        now = time.monotonic()
        if not self._logged_first_failure or (now - self._last_failure_log_mono) >= _FAILURE_LOG_INTERVAL_S:
            print(f"[VGCS:viewpro] TCP {where} to {self._host}:{self._port} failed: {exc!r}")
            self._last_failure_log_mono = now
            self._logged_first_failure = True

    def _log_first_success(self) -> None:
        if not self._logged_first_success:
            self._logged_first_success = True
            print(f"[VGCS:viewpro] TCP control link to {self._host}:{self._port} is up")

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._poller = threading.Thread(target=self._poll_loop, daemon=True)
        self._poller.start()

    def stop(self) -> None:
        self._running = False
        if self._zoom_stop_timer is not None:
            self._zoom_stop_timer.cancel()
            self._zoom_stop_timer = None
        self._transport.close()

    def get_status(self) -> GimbalStatus:
        with self._status_lock:
            return self._status

    def is_recording(self) -> bool:
        return self._recording

    def send_raw_command(self, payload: bytes) -> bytes:
        """Debug/integration escape hatch — send arbitrary already-framed bytes."""
        return self._transport.send_and_receive(payload)

    # ---- Status ----

    def request_status(self) -> GimbalStatus | None:
        pkt = vp.encode_heartbeat()
        try:
            reply = self._transport.send_and_receive(pkt)
        except Exception as exc:
            self._log_failure("status poll", exc)
            return None
        self._log_first_success()
        self._update_from_reply(reply)
        return self.get_status()

    def _update_from_reply(self, reply: bytes) -> None:
        if not reply:
            return
        parsed = vp.find_status_frame(reply)
        if parsed is None:
            return
        st = GimbalStatus(
            yaw_deg=parsed.get("yaw_deg"),
            pitch_deg=parsed.get("pitch_deg"),
            supported=True,
            updated_mono=time.monotonic(),
        )
        with self._status_lock:
            self._status = st
        rec = parsed.get("record_status")
        if rec is not None:
            self._recording = rec == 1
        if "range_m" in parsed:
            self._last_range_m = parsed["range_m"]

    def _poll_loop(self) -> None:
        while self._running:
            self.request_status()
            time.sleep(self._poll_dt)

    def _send(self, **kwargs) -> None:
        """Fire-and-forget — gimbal jog (set_gimbal_speed) fires every 80ms
        from a GUI-thread timer while a hold button is pressed
        (map_widget.py's _gimbal_hold_timer). Waiting for a TCP reply here
        (as SIYI's adapter learned the hard way for its own UDP jog path)
        would block the GUI thread up to the transport timeout on every
        tick. Status/attitude/record-state instead comes solely from the
        background poll loop's heartbeat (request_status), which runs off
        the GUI thread and can afford to block on a reply."""
        self._log_gimbal_command(kwargs)
        pkt = vp.encode_gimbal_camera_command(**kwargs)
        try:
            self._transport.send(pkt)
        except Exception as exc:
            self._log_failure("command send", exc)

    def _log_gimbal_command(self, kwargs: dict) -> None:
        """Throttled visibility into every A1 servo (gimbal-moving) command this
        process actually sends — added because a field report of the gimbal
        "moving on its own" turned up no VGCS-issued zoom/gimbal command at all
        in the session log; this makes the next repro conclusive either way
        (a VGCS command will show up here, or its absence points at the
        camera's own onboard behavior instead)."""
        servo = kwargs.get("servo", vp.SERVO_NO_CHANGE)
        if servo == vp.SERVO_NO_CHANGE:
            return
        now = time.monotonic()
        if now - self._last_gimbal_log_mono < 1.0:
            return
        self._last_gimbal_log_mono = now
        print(
            f"[VGCS:viewpro] gimbal cmd servo=0x{int(servo):02X} "
            f"p1={kwargs.get('servo_p1', 0)} p2={kwargs.get('servo_p2', 0)}"
        )

    # ---- Gimbal servo (A1) ----

    def ptz(self, action: str) -> None:
        """Pitch sign fixed 2026-07-30 from a field test of the equivalent
        SERVO_MANUAL_SPEED path in ViewproCameraControl.set_gimbal_speed: raw
        positive pitch drives the gimbal UP on this real unit (the vendor doc's
        absolute-angle worked example says positive = DOWN, but that doesn't
        hold for this velocity command in practice) — up=+raw, down=-raw."""
        action_l = str(action or "").strip().lower()
        raw = vp.speed_dps_to_raw(_DEFAULT_SLEW_DPS)
        if action_l in ("up", "pitch_up"):
            self._send(servo=vp.SERVO_MANUAL_SPEED, servo_p2=raw)
        elif action_l in ("down", "pitch_down"):
            self._send(servo=vp.SERVO_MANUAL_SPEED, servo_p2=-raw)
        elif action_l in ("left", "yaw_left"):
            self._send(servo=vp.SERVO_MANUAL_SPEED, servo_p1=-raw)
        elif action_l in ("right", "yaw_right"):
            self._send(servo=vp.SERVO_MANUAL_SPEED, servo_p1=raw)
        elif action_l == "stop":
            self._send(servo=vp.SERVO_MANUAL_SPEED, servo_p1=0, servo_p2=0)
        elif action_l in ("center", "home"):
            self._send(servo=vp.SERVO_HOME_POSITION)

    def set_angle(self, yaw: float, pitch: float) -> None:
        """Absolute angle, home position as 0 (servo 0x0B) — a single one-shot
        "turn to" command, not for continuous/high-frequency sends (doc's
        own caveat on this servo mode)."""
        self._send(
            servo=vp.SERVO_MANUAL_ABSOLUTE_ANGLE,
            servo_p1=vp.angle_deg_to_raw(yaw),
            servo_p2=vp.angle_deg_to_raw(pitch),
        )

    def set_rotation_speed(self, yaw: float, pitch: float) -> None:
        self._send(
            servo=vp.SERVO_MANUAL_SPEED,
            servo_p1=vp.speed_dps_to_raw(yaw),
            servo_p2=vp.speed_dps_to_raw(pitch),
        )

    def center(self) -> None:
        self._send(servo=vp.SERVO_HOME_POSITION)

    def look_down(self) -> None:
        self._send(servo=vp.SERVO_LOOK_DOWN)

    # ---- Camera / optical (C1) ----

    def camera_zoom(self, direction: int) -> None:
        """Pulse the continuous zoom-in/out command then auto-stop shortly after,
        so one UI click reads as a single zoom step instead of running the lens
        to its end of travel (see ``_ZOOM_STEP_PULSE_S``)."""
        if self._zoom_stop_timer is not None:
            self._zoom_stop_timer.cancel()
            self._zoom_stop_timer = None
        d = int(direction)
        if d > 0:
            self._send(c1_op=vp.C1_OP_FOV_MINUS_ZOOM_IN, c1_zoom_speed=_ZOOM_SPEED)
        elif d < 0:
            self._send(c1_op=vp.C1_OP_FOV_PLUS_ZOOM_OUT, c1_zoom_speed=_ZOOM_SPEED)
        else:
            self._send(c1_op=vp.C1_OP_STOP)
            return
        timer = threading.Timer(_ZOOM_STEP_PULSE_S, lambda: self._send(c1_op=vp.C1_OP_STOP))
        timer.daemon = True
        self._zoom_stop_timer = timer
        timer.start()

    def camera_focus_step(self, direction: int) -> None:
        d = int(direction)
        if d > 0:
            self._send(c1_op=vp.C1_OP_FOCUS_PLUS, c1_zoom_speed=_FOCUS_SPEED)
        elif d < 0:
            self._send(c1_op=vp.C1_OP_FOCUS_MINUS, c1_zoom_speed=_FOCUS_SPEED)
        else:
            self._send(c1_op=vp.C1_OP_STOP)

    def camera_auto_focus(self) -> None:
        self._send(c1_op=vp.C1_OP_AUTO_FOCUS)

    def camera_photo(self) -> None:
        self._send(c1_op=vp.C1_OP_TAKE_PICTURE)

    def camera_record_toggle(self) -> None:
        if self._recording:
            self._send(c1_op=vp.C1_OP_STOP_RECORD)
        else:
            self._send(c1_op=vp.C1_OP_START_RECORD)
        # Optimistic local flip; corrected from real device state (D1 record
        # status) on the next status reply if it disagrees.
        self._recording = not self._recording

    # ---- Laser rangefinder (only meaningful on LRF-equipped models) ----

    def laser_range_once(self) -> None:
        self._send(c1_lrf=vp.C1_LRF_SINGLE)

    def laser_range_start(self) -> None:
        self._send(c1_lrf=vp.C1_LRF_CONTINUOUS_START)

    def laser_range_stop(self) -> None:
        self._send(c1_lrf=vp.C1_LRF_STOP)

    def query_range_m(self) -> float | None:
        """Last known LRF range from periodic status (D1) — None if the
        connected gimbal has no rangefinder or hasn't reported one yet."""
        return self._last_range_m
