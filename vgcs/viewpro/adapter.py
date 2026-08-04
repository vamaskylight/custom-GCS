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

import os
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
# Post-jog position hold. SERVO_MANUAL_SPEED (0x01) is a *rate* mode: velocity 0
# means "no commanded motion", NOT "hold this position" — it leaves the gimbal in
# rate mode with no position lock, so it slowly drifts on gyro bias. Field report
# 2026-07-30 matched this exactly: no drift on connect (gimbal still in its own
# stabilised mode), drift starting only after the jog buttons had been used and
# then left idle, with VGCS provably sending nothing during the drift.
# The fix is to hand the gimbal an explicit position target once the jog settles.
# SERVO_MANUAL_RELATIVE_ANGLE (0x09) with all-zero params ("move by 0 degrees from
# where you are") is used rather than the absolute-angle servo (0x0B) on purpose:
# relative-zero needs no knowledge of the angle reference frame or of the
# pitch-sign convention, so it cannot jump the gimbal. Worst case it is a no-op.
# The delay lets the gimbal finish decelerating so the hold pins its true resting
# position. Set VGCS_VIEWPRO_POST_JOG_HOLD=0 to disable.
_POST_JOG_HOLD_DELAY_S = 0.35


def _post_jog_hold_enabled() -> bool:
    return os.environ.get("VGCS_VIEWPRO_POST_JOG_HOLD", "1").strip().lower() not in (
        "0",
        "false",
        "no",
    )


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
        self._last_range_mono = 0.0
        self._running = False
        self._poller: threading.Thread | None = None
        self._poll_dt = 1.0 / max(0.5, float(poll_hz))
        self._last_failure_log_mono = 0.0
        self._logged_first_failure = False
        self._logged_first_success = False
        self._zoom_stop_timer: threading.Timer | None = None
        self._last_gimbal_log_mono = 0.0
        self._last_gimbal_log_sig: tuple[int, int, int] | None = None
        self._position_hold_timer: threading.Timer | None = None
        self._last_servo_mode: int | None = None
        self._lrf_armed = False

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
        self._cancel_position_hold()
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
        self._note_servo_mode(parsed.get("servo_status"))
        rec = parsed.get("record_status")
        if rec is not None:
            self._recording = rec == 1
        # Only record an ACTUAL measurement. decode_d1 always includes the
        # "range_m" key but sets it to None when the laser hasn't measured
        # anything (raw value 0), so testing `"range_m" in parsed` bumped the
        # timestamp on every status frame at 2 Hz regardless. That made
        # _wait_for_fresh_range() (see ViewproCameraControl) think a fresh
        # reading had arrived ~0.5s after firing — on the next routine poll —
        # and return None instead of waiting for the laser. Field-observed
        # 2026-08-04 as "sometimes I have to mark twice before LRF shows data":
        # the first click gave up early, the laser finished a moment later and
        # the camera held that value in D1, so the second click found it
        # already there and succeeded instantly.
        range_m = parsed.get("range_m")
        if range_m is not None:
            self._last_range_m = range_m
            self._last_range_mono = time.monotonic()

    def _note_servo_mode(self, status: object) -> None:
        """Log the gimbal's own reported servo mode whenever it changes.

        The B1 status block carries the mode the gimbal believes it is in (2 Hz),
        and it was being decoded and discarded. Several modes move the gimbal with
        no command from us — azimuth scan, onboard tracking, manual RC (a
        transmitter stick), and manual-speed (a rate mode with no position lock).
        A field report of the gimbal drifting while idle is otherwise unfalsifiable
        from this end, since the logs already prove VGCS sends nothing during the
        drift; this says which mode it is actually sitting in.
        """
        if status is None:
            return
        mode = int(status) & 0x0F
        if mode == self._last_servo_mode:
            return
        prev = self._last_servo_mode
        self._last_servo_mode = mode
        prev_txt = (
            f" (was 0x{prev:02X} {vp.servo_mode_name(prev)})" if prev is not None else " (first report)"
        )
        print(f"[VGCS:viewpro] gimbal mode 0x{mode:02X} {vp.servo_mode_name(mode)}{prev_txt}")

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
        """Visibility into every A1 servo (gimbal-moving) command this process
        actually sends — added because a field report of the gimbal "moving on
        its own" turned up no VGCS-issued gimbal command at all in the session
        log.

        Every *change* in the command is logged; only identical repeats are
        rate-limited. The first version of this throttled purely on time, which
        swallowed the stop (p1=0 p2=0) that follows ~80ms after a jog — making
        the logs ambiguous about whether a slew was ever stopped, which is the
        single most important thing to know when diagnosing a runaway.
        """
        servo = kwargs.get("servo", vp.SERVO_NO_CHANGE)
        if servo == vp.SERVO_NO_CHANGE:
            return
        sig = (int(servo), int(kwargs.get("servo_p1", 0)), int(kwargs.get("servo_p2", 0)))
        now = time.monotonic()
        changed = sig != self._last_gimbal_log_sig
        if not changed and now - self._last_gimbal_log_mono < 1.0:
            return
        self._last_gimbal_log_mono = now
        self._last_gimbal_log_sig = sig
        print(
            f"[VGCS:viewpro] gimbal cmd servo=0x{sig[0]:02X} p1={sig[1]} p2={sig[2]}"
            + ("" if changed else " (held)")
        )

    # ---- Gimbal servo (A1) ----

    def ptz(self, action: str) -> None:
        """Any operator-driven move supersedes a pending post-jog hold.

        Pitch sign fixed 2026-07-30 from a field test of the equivalent
        SERVO_MANUAL_SPEED path in ViewproCameraControl.set_gimbal_speed: raw
        positive pitch drives the gimbal UP on this real unit (the vendor doc's
        absolute-angle worked example says positive = DOWN, but that doesn't
        hold for this velocity command in practice) — up=+raw, down=-raw."""
        action_l = str(action or "").strip().lower()
        raw = vp.speed_dps_to_raw(_DEFAULT_SLEW_DPS)
        self._cancel_position_hold()
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
            self._schedule_position_hold()
        elif action_l in ("center", "home"):
            self._send(servo=vp.SERVO_HOME_POSITION)

    def set_angle(self, yaw: float, pitch: float) -> None:
        """Absolute angle, home position as 0 (servo 0x0B) — a single one-shot
        "turn to" command, not for continuous/high-frequency sends (doc's
        own caveat on this servo mode)."""
        self._cancel_position_hold()
        self._send(
            servo=vp.SERVO_MANUAL_ABSOLUTE_ANGLE,
            servo_p1=vp.angle_deg_to_raw(yaw),
            servo_p2=vp.angle_deg_to_raw(pitch),
        )

    def set_rotation_speed(self, yaw: float, pitch: float) -> None:
        self._cancel_position_hold()
        yaw_raw = vp.speed_dps_to_raw(yaw)
        pitch_raw = vp.speed_dps_to_raw(pitch)
        self._send(
            servo=vp.SERVO_MANUAL_SPEED,
            servo_p1=yaw_raw,
            servo_p2=pitch_raw,
        )
        if yaw_raw == 0 and pitch_raw == 0:
            # Jog finished — don't leave the gimbal parked in rate mode (see
            # _POST_JOG_HOLD_DELAY_S: that is what makes it drift when idle).
            self._schedule_position_hold()

    def _cancel_position_hold(self) -> None:
        timer = self._position_hold_timer
        if timer is not None:
            timer.cancel()
            self._position_hold_timer = None

    def _schedule_position_hold(self) -> None:
        if not _post_jog_hold_enabled():
            return
        self._cancel_position_hold()
        timer = threading.Timer(_POST_JOG_HOLD_DELAY_S, self._apply_position_hold)
        timer.daemon = True
        self._position_hold_timer = timer
        timer.start()

    def _apply_position_hold(self) -> None:
        """Pin the gimbal at its current position after a jog (see _POST_JOG_HOLD_DELAY_S).

        A relative move of zero, so it needs no attitude readback and no knowledge
        of the angle reference frame or pitch-sign convention — it cannot move the
        gimbal, only give it something to hold onto.
        """
        self._position_hold_timer = None
        self._send(servo=vp.SERVO_MANUAL_RELATIVE_ANGLE)

    def center(self) -> None:
        self._cancel_position_hold()
        self._send(servo=vp.SERVO_HOME_POSITION)

    def look_down(self) -> None:
        self._cancel_position_hold()
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
        connected gimbal has no rangefinder or hasn't reported one yet.

        Cached rather than freshly polled on purpose: the background poll thread
        refreshes it at 2 Hz, so this is at most ~0.5s old, and reading the cache
        keeps this call itself non-blocking. It is used two ways with different
        freshness needs: the PROXIMITY panel's periodic display poll
        (`_refresh_c13_lrf_display`, GUI thread, 2 Hz) is fine with "at most
        0.5s old". A one-shot lock is NOT — `laser_range_once()` is
        fire-and-forget, so reading this immediately after firing can return
        `None` (nothing measured yet) or a STALE value from a previous, different
        target; see `last_range_updated_mono()` for how the lock path (which
        does run on a worker thread, not the GUI thread — see LrfLockTask) waits
        for a genuinely fresh reading instead of trusting this alone."""
        return self._last_range_m

    def last_range_updated_mono(self) -> float:
        """time.monotonic() timestamp of the last range update, 0.0 if none yet.
        Lets a caller confirm a reading is fresh (i.e. arrived after some marker
        time) rather than just non-None — see query_range_m's docstring."""
        return self._last_range_mono

    def set_lrf_armed(self, armed: bool) -> None:
        """Arm = continuous ranging, so the D1 status carries a live distance the
        lock can read immediately. Disarm stops the laser rather than leaving it
        firing (it is an eye-safety-relevant emitter, not just a sensor)."""
        want = bool(armed)
        if want:
            self.laser_range_start()
        else:
            self.laser_range_stop()
        self._lrf_armed = want

    def is_lrf_armed(self) -> bool:
        return bool(self._lrf_armed)

    def has_rangefinder(self) -> bool:
        """True once the gimbal has actually reported a range in its status.

        Runtime detection, because LRF is a per-model option on Viewpro and the
        protocol gives no capability query — a unit without the hardware simply
        never populates the D1 range field."""
        return self._last_range_m is not None
