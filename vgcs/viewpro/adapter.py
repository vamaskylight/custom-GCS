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
# How long continuous ranging keeps running after a lock finishes.
#
# Field data 2026-08-04: with the laser stopped immediately after every lock,
# results were a near coin-flip (~15 successes / ~12 timeouts in one session on
# a healthy link) — but the failures clustered on attempts made after a pause,
# while rapid repeat clicks (33.8/33.9/33.9/34.1/33.9 m back-to-back) almost
# always succeeded. That is the signature of continuous ranging taking a
# variable time to produce its FIRST measurement: stopping the laser after each
# lock made every single click pay that cold-start again.
#
# So don't stop instantly — leave it streaming briefly so a follow-up lock reads
# from an already-running stream, and auto-stop once genuinely idle (it is an
# eye-safety-relevant emitter, so "leave it on forever" is not an option).
# Set VGCS_VIEWPRO_LRF_IDLE_STOP_S to tune; 0 restores stop-immediately.
_LRF_IDLE_STOP_S = 10.0

# While a DOOAF/observation session is open the operator takes several ranges
# minutes apart (aiming the gimbal, working the dialog), so _LRF_IDLE_STOP_S
# expires between every one and each pick pays the laser's 5-7s cold start —
# field log 2026-08-18 shows EVERY range that session marked "(cold laser)",
# with two 10s timeouts. A session hold keeps the stream alive across the whole
# workflow so follow-up picks read from a running laser.
#
# Capped, because this is a firing laser: an operator who leaves the dialog open
# and walks away must not leave it emitting indefinitely. The cap releases the
# hold and the normal idle stop then shuts the laser down.
_LRF_SESSION_HOLD_MAX_S = 300.0


def _lrf_session_hold_max_s() -> float:
    raw = os.environ.get("VGCS_VIEWPRO_LRF_SESSION_HOLD_S", "").strip()
    if not raw:
        return _LRF_SESSION_HOLD_MAX_S
    try:
        return max(0.0, float(raw))
    except ValueError:
        return _LRF_SESSION_HOLD_MAX_S


def _lrf_idle_stop_s() -> float:
    raw = os.environ.get("VGCS_VIEWPRO_LRF_IDLE_STOP_S", "").strip()
    if not raw:
        return _LRF_IDLE_STOP_S
    try:
        return max(0.0, float(raw))
    except ValueError:
        return _LRF_IDLE_STOP_S


# --- Onboard AI tracker (M13/M14) ---
# How long to wait for the camera to CONFIRM a track actually engaged, via a
# fresh F1 status 2. Nothing is re-sent during the wait: camera_control.py's
# _LRF_RANGE_WAIT_S notes a field regression where ~20 extra status requests per
# lock correlated with the camera resetting the TCP connection, so the confirm
# loop reads the 2 Hz poller's own decodes and issues zero extra traffic.
_TRACK_CONFIRM_BUDGET_S = 2.5
_TRACK_CONFIRM_POLL_S = 0.1
# Let the point command land before the start command — E1 start carries no
# coordinate of its own, so it tracks whatever the point was last set to.
_TRACK_POINT_SETTLE_S = 0.08
# Beyond this, a cached F1/servo sample says nothing about the present.
_TRACK_STATUS_STALE_S = 3.0
# F1 status 1 (searching) and 3 (lost) are both normal mid-reacquisition, so a
# single non-tracking sample must not tear the track down.
_TRACK_LOST_GRACE_S = 2.5


def _track_sign(axis: str) -> int:
    raw = os.environ.get(f"VGCS_VIEWPRO_TRACK_SIGN_{axis.upper()}", "").strip()
    return -1 if raw in ("-1", "-") else 1


def _track_sign_x() -> int:
    return _track_sign("x")


def _track_sign_y() -> int:
    return _track_sign("y")


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
        self._lrf_streaming = False
        self._lrf_session_hold = False
        self._lrf_session_hold_mono = 0.0
        self._lrf_idle_stop_timer: threading.Timer | None = None
        self._last_hfov_deg: float | None = None
        self._last_vfov_deg: float | None = None
        self._last_zoom_x: float | None = None
        self._last_tracker_status: tuple[int, int] | None = None
        self._last_tracker_status_mono = 0.0
        self._last_servo_mode_mono = 0.0
        self._track_engaged = False
        self._track_start_lock = threading.Lock()
        self._track_lost_since_mono = 0.0
        self._track_hold_suppress_logged = False

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
        self._cancel_lrf_idle_stop()
        # Release the laser explicitly. Cancelling the idle timer alone would
        # leave a held session's laser emitting with the socket closed and
        # nothing left able to stop it.
        self._lrf_session_hold = False
        if self._lrf_streaming:
            self._lrf_streaming = False
            self._send(c1_lrf=vp.C1_LRF_STOP)
        # Stop the tracker BEFORE closing the socket — otherwise a camera left
        # tracking keeps driving the gimbal after VGCS has disconnected, with
        # nothing left able to tell it to stop.
        if self._track_engaged:
            self._track_engaged = False
            self._send_frame(vp.encode_e1(command=vp.E1_CMD_STOP), "track stop (shutdown)")
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
        # Live optics state. Unlike the C13's fixed lens, this camera has a big
        # optical zoom AND reports its true current FOV in every status frame, so
        # DOOAF's geo-referencing can use the real value instead of one static
        # setting. Guarded because a 0 here means "not reported" (and a zero FOV
        # would silently divide the pixel->angle maths into nonsense).
        hfov = parsed.get("hfov_deg")
        vfov = parsed.get("vfov_deg")
        if hfov is not None and vfov is not None and float(hfov) > 0.0 and float(vfov) > 0.0:
            self._last_hfov_deg = float(hfov)
            self._last_vfov_deg = float(vfov)
        zoom_x = parsed.get("zoom_x")
        if zoom_x is not None and float(zoom_x) > 0.0:
            self._last_zoom_x = float(zoom_x)
        self._note_tracker_status(parsed.get("track_status"), parsed.get("track_target_type"))

    def _note_tracker_status(self, status: object, target_type: object) -> None:
        """Record the onboard tracker's state, logging only on change.

        The TIMESTAMP advances on every decode, not only on transitions —
        start_visual_track_at_norm needs to distinguish "the camera reported
        TRACKING in a sample taken after we asked" from "it was already
        tracking before we asked" (the RC transmitter and the vendor app can
        both start this tracker independently). Stamping only on change would
        make an already-tracking camera instantly 'confirm' a command that
        never took effect. It also means a dead link stops advancing the age
        instead of freezing a stale status as though it were current.
        """
        if status is None:
            return
        self._last_tracker_status_mono = time.monotonic()
        sig = (int(status) & 0x03, int(target_type or 0) & 0x07)
        if sig == self._last_tracker_status:
            return
        self._last_tracker_status = sig
        name = vp.TRACK_STATUS_NAMES.get(sig[0], "unknown")
        print(f"[VGCS:viewpro] onboard tracker {name} (status={sig[0]} target_type={sig[1]})")

    def _send_frame(self, pkt: bytes, what: str) -> None:
        """Send an already-framed packet that is NOT the A1+C1+E1 combo.

        Sibling of _send, which is hardwired to encode_gimbal_camera_command and
        cannot emit a 0x1E/0x2E frame. Logged unconditionally (unlike the jog
        path's throttled logging) because these are rare, deliberate commands
        and their absence from a field log is itself diagnostic.
        """
        print(f"[VGCS:viewpro] {what}")
        try:
            self._transport.send(pkt)
        except Exception as exc:
            self._log_failure(f"{what} send", exc)

    def tracker_status_age_s(self) -> float:
        """Seconds since the last F1 decode, or a large number if never."""
        if self._last_tracker_status_mono <= 0.0:
            return float("inf")
        return time.monotonic() - self._last_tracker_status_mono

    def query_servo_mode(self) -> int | None:
        return self._last_servo_mode

    def start_visual_track_at_norm(self, u: float, v: float) -> bool:
        """Start the camera's onboard tracker at a normalized video point.

        **Worker-thread only** — blocks up to _TRACK_CONFIRM_BUDGET_S. Returns
        True ONLY when the camera reports F1 status 2 in a sample decoded after
        the start command went out. That is a strictly stronger contract than
        the Skydroid equivalent, which returns True whenever the send did not
        raise; here a send that the firmware ignores must not look like success,
        because the caller uses False to fall back to software tracking.

        Self-cleaning: on timeout it sends an explicit E1 stop, so a command the
        firmware half-accepted cannot leave the camera tracking something the
        operator did not ask for.
        """
        if not self._track_start_lock.acquire(blocking=False):
            return False
        try:
            # The camera is about to drive the gimbal; our own position hold
            # would fight it. Cancelled BEFORE any send so a hold already in
            # flight cannot land mid-engagement.
            self._cancel_position_hold()
            self._track_engaged = True
            self._track_lost_since_mono = 0.0
            self._track_hold_suppress_logged = False
            x_px, y_px = vp.track_point_from_norm(
                u, v, x_sign=_track_sign_x(), y_sign=_track_sign_y()
            )
            self._send_frame(
                vp.encode_track_point(x_px, y_px),
                f"track point -> ({x_px:+d},{y_px:+d}) px from centre (u={u:.3f} v={v:.3f})",
            )
            time.sleep(_TRACK_POINT_SETTLE_S)
            sent_mono = time.monotonic()
            self._send_frame(vp.encode_e1(command=vp.E1_CMD_START_TRACK), "track start")
            deadline = sent_mono + _TRACK_CONFIRM_BUDGET_S
            while time.monotonic() < deadline:
                time.sleep(_TRACK_CONFIRM_POLL_S)
                if self._last_tracker_status_mono < sent_mono:
                    continue  # only samples decoded AFTER we asked can confirm
                status = (self._last_tracker_status or (0, 0))[0]
                if status == 2:
                    servo = self._last_servo_mode
                    print(
                        f"[VGCS:viewpro] onboard track ENGAGED (F1 status=2, "
                        f"servo mode=0x{servo:02X})" if servo is not None
                        else "[VGCS:viewpro] onboard track ENGAGED (F1 status=2)"
                    )
                    return True
            last = self._last_tracker_status
            age = self.tracker_status_age_s()
            print(
                f"[VGCS:viewpro] onboard track did NOT engage within "
                f"{_TRACK_CONFIRM_BUDGET_S:.1f}s (last F1={last}, age={age:.1f}s) — "
                "stopping it and falling back to software tracking"
            )
            self._track_engaged = False
            self._send_frame(vp.encode_e1(command=vp.E1_CMD_STOP), "track stop (start failed)")
            return False
        except Exception as exc:
            self._track_engaged = False
            self._log_failure("track start", exc)
            return False
        finally:
            self._track_start_lock.release()

    def stop_visual_track(self) -> None:
        """Stop the onboard tracker and re-pin the gimbal.

        Order matters: the engaged flag is cleared FIRST so that
        _apply_position_hold's suppression check does not swallow the very hold
        that re-pins the gimbal — otherwise stopping a track would leave it in a
        rate mode with no position lock, reintroducing the idle-drift bug
        through a new door.
        """
        was = self._track_engaged
        self._track_engaged = False
        self._track_lost_since_mono = 0.0
        if was:
            self._send_frame(vp.encode_e1(command=vp.E1_CMD_STOP), "track stop")
        self._apply_position_hold()

    def is_visual_track_active(self) -> bool:
        """Whether the camera is still tracking, with a grace period.

        F1 status 1 (searching) and 3 (lost) are both normal while the tracker
        reacquires, and the caller's check is an undebounced single-sample kill
        switch — so a momentary non-2 must not tear the track down. A stale
        status means we simply do not know, which is reported as not-active.
        """
        if not self._track_engaged:
            return False
        if self.tracker_status_age_s() > _TRACK_STATUS_STALE_S:
            return False
        status = (self._last_tracker_status or (0, 0))[0]
        if status == 2:
            self._track_lost_since_mono = 0.0
            return True
        now = time.monotonic()
        if self._track_lost_since_mono <= 0.0:
            self._track_lost_since_mono = now
            return True
        return (now - self._track_lost_since_mono) < _TRACK_LOST_GRACE_S

    def track_start_in_progress(self) -> bool:
        return self._track_start_lock.locked()

    def _release_track_for_operator_move(self) -> None:
        """Hand the gimbal back when the operator aims it themselves.

        An operator jogging, centring or pointing the gimbal is an unambiguous
        instruction that they are aiming now — leaving the camera's tracker
        engaged would have the two fighting for the same axes. Cheap no-op when
        no track is running, so every operator-move entry point can call it
        unconditionally.

        Deliberately does NOT go through stop_visual_track(): that applies a
        position hold, which is exactly what the move about to happen does not
        want.
        """
        if not self._track_engaged:
            return
        self._track_engaged = False
        self._track_lost_since_mono = 0.0
        self._send_frame(
            vp.encode_e1(command=vp.E1_CMD_STOP), "track stop (operator took manual control)"
        )

    def _camera_owns_gimbal(self) -> bool:
        """True when the CAMERA is driving the gimbal, so we must not."""
        if self._track_engaged:
            return True
        if self._last_servo_mode == 0x06:
            return self._last_servo_mode_mono > 0.0 and (
                time.monotonic() - self._last_servo_mode_mono
            ) <= _TRACK_STATUS_STALE_S
        return False

    def query_tracker_status(self) -> tuple[int, int] | None:
        """(track_status, track_target_type) as last reported, or None."""
        return self._last_tracker_status

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
        # Stamped every decode, not only on change — see _note_tracker_status.
        # Servo mode 0x06 (TRACKING) corroborates the F1 confirm, and a stale
        # 0x06 must not be mistaken for the camera currently owning the gimbal.
        self._last_servo_mode_mono = time.monotonic()
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
        """Any operator-driven move supersedes a pending post-jog hold, and
        releases the onboard tracker — the operator taking manual control is an
        unambiguous instruction that they, not the camera, are now aiming.


        Pitch sign fixed 2026-07-30 from a field test of the equivalent
        SERVO_MANUAL_SPEED path in ViewproCameraControl.set_gimbal_speed: raw
        positive pitch drives the gimbal UP on this real unit (the vendor doc's
        absolute-angle worked example says positive = DOWN, but that doesn't
        hold for this velocity command in practice) — up=+raw, down=-raw."""
        action_l = str(action or "").strip().lower()
        raw = vp.speed_dps_to_raw(_DEFAULT_SLEW_DPS)
        self._cancel_position_hold()
        self._release_track_for_operator_move()
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
        self._release_track_for_operator_move()
        self._cancel_position_hold()
        self._send(
            servo=vp.SERVO_MANUAL_ABSOLUTE_ANGLE,
            servo_p1=vp.angle_deg_to_raw(yaw),
            servo_p2=vp.angle_deg_to_raw(pitch),
        )

    def set_rotation_speed(self, yaw: float, pitch: float) -> None:
        self._release_track_for_operator_move()
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
        if self._camera_owns_gimbal():
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
        # Re-checked AT FIRE TIME, not just at schedule time: a track can engage
        # during the 0.35s delay, and a hold landing then would yank the gimbal
        # off the target the camera is steering toward.
        if self._camera_owns_gimbal():
            if not self._track_hold_suppress_logged:
                self._track_hold_suppress_logged = True
                print("[VGCS:viewpro] position hold suppressed — camera owns the gimbal")
            return
        self._send(servo=vp.SERVO_MANUAL_RELATIVE_ANGLE)

    def center(self) -> None:
        self._release_track_for_operator_move()
        self._cancel_position_hold()
        self._send(servo=vp.SERVO_HOME_POSITION)

    def look_down(self) -> None:
        self._release_track_for_operator_move()
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
        self._cancel_lrf_idle_stop()
        self._lrf_streaming = False
        self._send(c1_lrf=vp.C1_LRF_STOP)

    def laser_range_begin_session(self) -> bool:
        """Start continuous ranging for a lock, or keep an already-running
        stream going. Returns True if it was ALREADY streaming (so the caller
        knows this shot didn't have to pay the laser's cold-start time).

        See _LRF_IDLE_STOP_S for why the stream is kept warm between locks.
        """
        self._cancel_lrf_idle_stop()
        if self._lrf_streaming:
            return True
        self._lrf_streaming = True
        self.laser_range_start()
        return False

    def laser_range_end_session(self) -> None:
        """Finish a lock without killing the laser immediately — schedule the
        stop after an idle grace period so a follow-up lock reads from the
        already-running stream instead of cold-starting again."""
        if _lrf_idle_stop_s() <= 0.0 and not self._lrf_session_hold_active():
            self.laser_range_stop()
            return
        self._schedule_lrf_idle_stop()

    def _on_lrf_idle_timeout(self) -> None:
        self._lrf_idle_stop_timer = None
        if self._lrf_armed:
            # Operator armed it explicitly via the PROXIMITY panel — theirs to stop.
            return
        if self._lrf_session_hold_active():
            # A DOOAF session is open and more picks are expected; keep the
            # laser warm and re-check after another idle period.
            self._schedule_lrf_idle_stop()
            return
        self._lrf_streaming = False
        self._send(c1_lrf=vp.C1_LRF_STOP)

    def _lrf_session_hold_active(self) -> bool:
        if not self._lrf_session_hold:
            return False
        max_s = _lrf_session_hold_max_s()
        if max_s > 0.0 and (time.monotonic() - self._lrf_session_hold_mono) > max_s:
            self._lrf_session_hold = False
            print(
                f"[VGCS:viewpro] LRF session hold expired after {max_s:.0f}s — "
                "letting the laser idle-stop (re-open the dialog to hold again)"
            )
            return False
        return True

    def _schedule_lrf_idle_stop(self) -> None:
        idle_s = _lrf_idle_stop_s()
        if idle_s <= 0.0:
            return
        self._cancel_lrf_idle_stop()
        timer = threading.Timer(idle_s, self._on_lrf_idle_timeout)
        timer.daemon = True
        self._lrf_idle_stop_timer = timer
        timer.start()

    def set_lrf_session_hold(self, enable: bool) -> None:
        """Hold the laser warm across a whole observation session.

        Turning it OFF does not stop the laser outright — it just lets the
        normal idle grace period take over, so a pick already in flight is not
        cut off mid-measurement.
        """
        want = bool(enable)
        if want == self._lrf_session_hold:
            if want:
                self._lrf_session_hold_mono = time.monotonic()  # refresh the cap
            return
        self._lrf_session_hold = want
        self._lrf_session_hold_mono = time.monotonic()
        print(f"[VGCS:viewpro] LRF session hold {'ON — keeping laser warm' if want else 'OFF'}")
        if want:
            if not self._lrf_streaming:
                self._lrf_streaming = True
                self.laser_range_start()
        elif self._lrf_streaming and not self._lrf_armed:
            self._schedule_lrf_idle_stop()

    def _cancel_lrf_idle_stop(self) -> None:
        timer = self._lrf_idle_stop_timer
        if timer is not None:
            timer.cancel()
            self._lrf_idle_stop_timer = None

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
        self._lrf_armed = want  # set first: _on_lrf_idle_timeout checks it
        if want:
            self._cancel_lrf_idle_stop()  # operator's session outlives any lock grace period
            self._lrf_streaming = True
            self.laser_range_start()
        else:
            self.laser_range_stop()

    def is_lrf_armed(self) -> bool:
        return bool(self._lrf_armed)

    def query_fov_deg(self) -> tuple[float, float] | None:
        """Live (horizontal, vertical) FOV in degrees as the camera reports it in
        its D1 status block, or None if it hasn't reported usable values yet.

        This tracks the optical zoom, which is exactly what DOOAF's pixel->angle
        maths needs and what a single static FOV setting cannot provide on a
        zoom lens."""
        if self._last_hfov_deg is None or self._last_vfov_deg is None:
            return None
        return (self._last_hfov_deg, self._last_vfov_deg)

    def query_zoom_x(self) -> float | None:
        """Live optical zoom factor as reported by the camera, or None."""
        return self._last_zoom_x

    def has_rangefinder(self) -> bool:
        """True once the gimbal has actually reported a range in its status.

        Runtime detection, because LRF is a per-model option on Viewpro and the
        protocol gives no capability query — a unit without the hardware simply
        never populates the D1 range field."""
        return self._last_range_m is not None
