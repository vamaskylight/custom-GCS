"""The link to the flight controller, in one thread (pymavlink is not safe to use from two).

It reads every message, sends GPS_INPUT RATE_HZ times a second, its heartbeat once a second, asks
for the messages it needs (again every REQUEST_EVERY_S, since a controller that restarts forgets
them), and reads the GPS settings (again every REQUEST_EVERY_S, until then camera positions are held
back).
"""

from __future__ import annotations

import logging
import threading
import time
from collections import Counter
from collections.abc import Callable

from pymavlink import mavutil

from .gps_input import ADVISED, NOT_READ, RATE_HZ, REQUIRED, Sender, settings_problem

log = logging.getLogger("vama_nav.runner")

MAV_AUTOPILOT_INVALID = 8
MAV_COMP_ID_AUTOPILOT1 = 1
MAV_MODE_FLAG_SAFETY_ARMED = 128
STREAMS_HZ = {24: 5.0}            # GPS_RAW_INT: GPS 1, passed on while disarmed
REQUEST_EVERY_S = 30.0
PARAMS_RETRY_S = 2.0              # while a setting is not read yet
HEARTBEAT_EVERY_S = 1.0
STATUS_EVERY_S = 10.0
POLL_S = 0.01                     # the longest wait for a message: sends stay within about 10 ms of their time


class Vehicle:
    """What the sender needs to know of the flight controller."""

    def __init__(self) -> None:
        self.autopilot: tuple[int, int] | None = None
        self.armed = False
        self.heartbeat_ns: int | None = None
        self.gps1: tuple | None = None            # (GPS_RAW_INT, arrival_ns)
        self.params: dict[str, float] = {}

    def hear(self, msg, arrival_ns: int) -> bool:
        """True when the message was the flight controller's."""
        kind = msg.get_type()
        src = (int(msg.get_srcSystem()), int(msg.get_srcComponent()))
        if (kind == "HEARTBEAT" and self.autopilot is None and src[1] == MAV_COMP_ID_AUTOPILOT1
                and int(getattr(msg, "autopilot", MAV_AUTOPILOT_INVALID)) != MAV_AUTOPILOT_INVALID):
            self.autopilot = src
            log.info("flight controller %d/%d", *src)
        if self.autopilot is None or src != self.autopilot:
            return False
        if kind == "HEARTBEAT":
            self.heartbeat_ns = int(arrival_ns)
            self.armed = bool(int(msg.base_mode) & MAV_MODE_FLAG_SAFETY_ARMED)
        elif kind == "GPS_RAW_INT":
            self.gps1 = (msg, int(arrival_ns))
        elif kind == "PARAM_VALUE":
            name = str(msg.param_id).rstrip("\x00")
            if name in REQUIRED or name in ADVISED:
                value = float(msg.param_value)
                if self.params.get(name) != value:
                    if name in ADVISED and value != ADVISED[name]:
                        log.warning("setting %s = %g (advised %g)", name, value, ADVISED[name])
                    else:
                        log.info("setting %s = %g", name, value)
                self.params[name] = value
        return True


class Runner:
    def __init__(self, conn, sender: Sender, source=None, clock: Callable[[], int] = time.monotonic_ns,
                 rate_hz: float = RATE_HZ) -> None:
        """``conn``: an open pymavlink connection. ``source``: an optional position source with
        ``hear(msg, arrival_ns)``, ``tick(now_ns)`` and the streams it needs in ``STREAMS_HZ``."""
        self.conn = conn
        self.sender = sender
        self.source = source
        self.clock = clock
        self.period_ns = int(1e9 / rate_hz)
        self.vehicle = Vehicle()
        self.sent: Counter = Counter()
        self.reason = ""
        self.held_back: str | None = settings_problem({})
        self.send_failing = False
        self.stop_event = threading.Event()
        self._next_send: int | None = None
        self._next_heartbeat = 0
        self._next_request = 0
        self._next_params = 0
        self._next_status: int | None = None
        self._asked_for: tuple[int, int] | None = None
        self._status_from: tuple[int, Counter] | None = None

    def streams(self) -> dict[int, float]:
        wanted = dict(STREAMS_HZ)
        wanted.update(getattr(self.source, "STREAMS_HZ", {}))
        return wanted

    def run(self) -> None:
        while not self.stop_event.is_set():
            wait_s = self.step(self.clock())
            try:
                msg = self.conn.recv_match(blocking=True, timeout=wait_s)
            except Exception as exc:          # a serial port that went away: wait, pymavlink reconnects
                log.warning("link: %s", exc)
                self.stop_event.wait(0.5)
                continue
            if msg is not None and msg.get_type() != "BAD_DATA":
                self.hear(msg, self.clock())

    def hear(self, msg, arrival_ns: int) -> None:
        self.vehicle.hear(msg, arrival_ns)
        if self.source is not None:
            try:
                self.source.hear(msg, arrival_ns)
            except Exception:
                log.exception("the position source could not use a %s", msg.get_type())

    def step(self, now_ns: int) -> float:
        """Do what is due at ``now_ns``. Returns how long to wait for a message (s)."""
        if self.source is not None:
            try:
                self.source.tick(now_ns)
            except Exception:
                log.exception("the position source failed")
        if self._next_send is None or now_ns >= self._next_send:
            self._send(self.clock())
            self._next_send = (now_ns if self._next_send is None else self._next_send) + self.period_ns
            if self._next_send <= now_ns:      # held up (a stalled port): no burst to catch up
                self._next_send = now_ns + self.period_ns
        try:
            self._housekeeping(now_ns)
        except Exception as exc:
            log.warning("link: %s", exc)
        return max(0.001, min(POLL_S, (self._next_send - self.clock()) / 1e9))

    def _send(self, now_ns: int) -> None:
        v = self.vehicle
        message, reason = self.sender.message(now_ns, v.armed, v.gps1, self.held_back)
        try:
            message.send(self.conn.mav, now_ns // 1000)
        except Exception as exc:
            if not self.send_failing:             # once, not ten times a second
                log.warning("GPS_INPUT not sent: %s", exc)
            self.send_failing = True
            return
        if self.send_failing:
            log.info("GPS_INPUT sent again")
            self.send_failing = False
        self.sent[message.source] += 1
        if reason != self.reason:
            log.info("GPS 2 now: %s", reason)
            self.reason = reason

    def _housekeeping(self, now_ns: int) -> None:
        mav = self.conn.mav
        if now_ns >= self._next_heartbeat:
            mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                               0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
            self._next_heartbeat = now_ns + int(HEARTBEAT_EVERY_S * 1e9)
        target = self.vehicle.autopilot
        if target is not None and (target != self._asked_for or now_ns >= self._next_request):
            for msg_id, hz in self.streams().items():
                mav.command_long_send(target[0], target[1], mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                      float(msg_id), float(int(1_000_000 / hz)), 0, 0, 0, 0, 0)
            for name in (*REQUIRED, *ADVISED):
                mav.param_request_read_send(target[0], target[1], name.encode(), -1)
            self._asked_for = target
            self._next_request = now_ns + int(REQUEST_EVERY_S * 1e9)
            self._next_params = now_ns + int(PARAMS_RETRY_S * 1e9)
        elif target is not None and self.held_back is not None and now_ns >= self._next_params:
            for name in REQUIRED:
                if name not in self.vehicle.params:
                    mav.param_request_read_send(target[0], target[1], name.encode(), -1)
            self._next_params = now_ns + int(PARAMS_RETRY_S * 1e9)
        held_back = settings_problem(self.vehicle.params)
        if held_back != self.held_back:
            if held_back is None:
                log.info("GPS settings right")
            elif not held_back.startswith(NOT_READ):     # still reading is no news; the status line says it
                log.warning("camera positions held back: %s", held_back)
            self.held_back = held_back
        if self._next_status is None:
            self._next_status = now_ns + int(STATUS_EVERY_S * 1e9)
            self._status_from = (now_ns, Counter(self.sent))
        elif now_ns >= self._next_status:
            since, before = self._status_from
            done = self.sent - before
            log.info("sent %d in %.1f s (camera fixes %d, carried forward %d, GPS 1 passed on %d, no fix %d)%s",
                     sum(done.values()), (now_ns - since) / 1e9, done["camera"], done["carried"], done["gps1"],
                     done["none"], f"; camera positions held back: {self.held_back}" if self.held_back else "")
            self._next_status = now_ns + int(STATUS_EVERY_S * 1e9)
            self._status_from = (now_ns, Counter(self.sent))
