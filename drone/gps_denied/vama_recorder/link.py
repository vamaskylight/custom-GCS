"""The MAVLink link to the flight controller.

One thread reads every message and stamps when it arrived. The same thread (pymavlink is not safe
to write from two) sends the recorder's heartbeat once a second, and asks the controller for the
messages the recorder needs at the rates it needs, again every REQUEST_EVERY_S, since a controller
that restarts forgets them.
"""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable

from pymavlink import mavutil

from .settings import Settings

log = logging.getLogger("vama_recorder.link")

# Message id: rate in Hz. ATTITUDE fast, because a picture's angles matter most (one degree is
# about 9 m on the ground from 500 m up).
STREAMS_HZ = {
    30: 20.0,    # ATTITUDE
    33: 10.0,    # GLOBAL_POSITION_INT
    24: 5.0,     # GPS_RAW_INT
    2: 1.0,      # SYSTEM_TIME (the GPS time)
    132: 5.0,    # DISTANCE_SENSOR (a rangefinder looking down, if the drone has one)
}
REQUEST_EVERY_S = 30.0
HEARTBEAT_EVERY_S = 1.0


class LinkError(RuntimeError):
    """The link cannot be opened, in plain words."""


class Link:
    def __init__(self, settings: Settings, on_message: Callable[[object, int], None],
                 target: Callable[[], tuple[int, int] | None], clock: Callable[[], int] = time.monotonic_ns) -> None:
        """``on_message(msg, arrival_ns)`` for every message. ``target()``: the flight controller's
        (system, component) once known, else None."""
        self.settings = settings
        self.on_message = on_message
        self.target = target
        self.clock = clock
        self.conn = None
        self.received = 0
        self.requests_sent = 0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def open(self) -> None:
        s = self.settings
        try:
            self.conn = mavutil.mavlink_connection(s.device, baud=s.baud, source_system=s.source_system,
                                                   source_component=s.source_component, autoreconnect=True)
        except Exception as exc:          # a serial port that is not there, a refused TCP port
            raise LinkError(f"cannot open the link {s.device} ({exc})") from None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="mavlink", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        conn, self.conn = self.conn, None
        if conn is not None:
            try:
                conn.close()
            except Exception:
                pass

    def _run(self) -> None:
        next_heartbeat = 0.0
        next_request = 0.0
        asked_for: tuple[int, int] | None = None
        while not self._stop.is_set():
            now = time.monotonic()
            try:
                if now >= next_heartbeat:
                    self.conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                                 mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                                                 mavutil.mavlink.MAV_STATE_ACTIVE)
                    next_heartbeat = now + HEARTBEAT_EVERY_S
                target = self.target()
                if target is not None and (target != asked_for or now >= next_request):
                    self._request_streams(target)
                    asked_for = target
                    next_request = now + REQUEST_EVERY_S
                msg = self.conn.recv_match(blocking=True, timeout=0.2)
            except Exception as exc:
                log.warning("link: %s", exc)
                self._stop.wait(0.5)
                continue
            if msg is None or msg.get_type() == "BAD_DATA":
                continue
            arrival = self.clock()
            self.received += 1
            try:
                self.on_message(msg, arrival)
            except Exception:
                log.exception("link: a message could not be used")

    def _request_streams(self, target: tuple[int, int]) -> None:
        for msg_id, hz in STREAMS_HZ.items():
            self.conn.mav.command_long_send(target[0], target[1], mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                            0, float(msg_id), float(int(1_000_000 / hz)), 0, 0, 0, 0, 0)
        self.requests_sent += 1
