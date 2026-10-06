"""What VGCS knows about the RTK corrections it passes from a base station to the drone.

Milestone M11, "RTK module connection" and "correction link status". The
drone's own messages say little about corrections (see vgcs.app.rtk_status: a
u-blox reports only its fix type). What VGCS can say for certain is its own
half: is the base connected, do corrections arrive, how many, which messages,
does the base know its position, and do they go out to the drone.

Pure: no Qt, no serial port, no link code.
"""

from __future__ import annotations

import time
from collections import deque

STATE_OFF = "off"
STATE_CONNECTING = "connecting"
STATE_CONNECTED = "connected"
STATE_ERROR = "error"

LEVEL_NA = "na"
LEVEL_OK = "ok"
LEVEL_WARN = "warn"

OFF_TEXT = "Off"

# A base sends its corrections once a second.
RATE_WINDOW_S = 5.0          # the rate is the mean over this long
QUIET_AFTER_S = 5.0          # no message for this long: the stream has stopped
TYPES_WINDOW_S = 15.0        # message types seen in this long are listed
# The base position (1005 or 1006) comes less often, every 1 to 10 s. Without
# it the drone's GPS cannot use the other messages at all.
STATION_MISSING_AFTER_S = 30.0
STATION_TYPES = (1005, 1006)


class CorrectionStream:
    """Follow the correction stream and say what to show about it."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        """Back to "off": no base configured, or the reader was stopped."""
        self._state = STATE_OFF
        self._source = ""
        self._detail = ""
        self._state_mono = 0.0
        self._frames: deque[tuple[float, int, int]] = deque()   # (time, message type, bytes)
        self._last_frame_mono: float | None = None
        self._first_frame_mono: float | None = None
        self._last_station_mono: float | None = None
        self._station: tuple[float, float, float] | None = None
        self._link_up = False
        self._total_bytes = 0

    # -- ingest ---------------------------------------------------------
    def set_state(self, state: str, source: str = "", detail: str = "", now: float | None = None) -> None:
        """The reader's state: connecting, connected, or an error with its reason."""
        now_f = time.monotonic() if now is None else float(now)
        if state == STATE_OFF:
            self.reset()
            return
        if state != self._state:
            self._state_mono = now_f
        if state != STATE_CONNECTED:
            # A new connection starts counting from nothing.
            self._frames.clear()
            self._last_frame_mono = None
            self._first_frame_mono = None
            self._last_station_mono = None
        self._state = state
        self._source = str(source or self._source)
        self._detail = str(detail or "")

    def add_frame(self, message_type: int, size: int, now: float | None = None) -> None:
        """One whole correction message arrived from the base."""
        now_f = time.monotonic() if now is None else float(now)
        self._frames.append((now_f, int(message_type), int(size)))
        self._last_frame_mono = now_f
        if self._first_frame_mono is None:
            self._first_frame_mono = now_f
        if int(message_type) in STATION_TYPES:
            self._last_station_mono = now_f
        self._total_bytes += int(size)
        while self._frames and now_f - self._frames[0][0] > TYPES_WINDOW_S:
            self._frames.popleft()

    def set_station(self, lat: float, lon: float, height_m: float) -> None:
        """The base's own position, from its 1005 or 1006 message."""
        self._station = (float(lat), float(lon), float(height_m))

    def set_link_up(self, up: bool) -> None:
        """Whether there is a drone to send the corrections to."""
        self._link_up = bool(up)

    # -- read -----------------------------------------------------------
    @property
    def state(self) -> str:
        return self._state

    def station(self) -> tuple[float, float, float] | None:
        return self._station

    def total_bytes(self) -> int:
        return self._total_bytes

    def age_s(self, now: float | None = None) -> float | None:
        """Seconds since the last correction message, or None when none has come."""
        if self._last_frame_mono is None:
            return None
        now_f = time.monotonic() if now is None else float(now)
        return max(0.0, now_f - self._last_frame_mono)

    def flowing(self, now: float | None = None) -> bool:
        """True while corrections arrive from the base."""
        age = self.age_s(now)
        return self._state == STATE_CONNECTED and age is not None and age <= QUIET_AFTER_S

    def rate_bps(self, now: float | None = None) -> float:
        """Bytes per second, as the mean over the last few seconds."""
        now_f = time.monotonic() if now is None else float(now)
        total = sum(size for when, _type, size in self._frames if now_f - when <= RATE_WINDOW_S)
        return total / RATE_WINDOW_S

    def message_types(self, now: float | None = None) -> list[int]:
        now_f = time.monotonic() if now is None else float(now)
        return sorted({mtype for when, mtype, _size in self._frames if now_f - when <= TYPES_WINDOW_S})

    def station_missing(self, now: float | None = None) -> bool:
        """Corrections flow, but the base has not sent its position for a long time."""
        now_f = time.monotonic() if now is None else float(now)
        if not self.flowing(now_f) or self._first_frame_mono is None:
            return False
        since = self._last_station_mono if self._last_station_mono is not None else self._first_frame_mono
        return now_f - since > STATION_MISSING_AFTER_S

    def level(self, now: float | None = None) -> str:
        if self._state == STATE_OFF:
            return LEVEL_NA
        if self.flowing(now) and self._link_up and not self.station_missing(now):
            return LEVEL_OK
        return LEVEL_WARN

    def text(self, now: float | None = None) -> str:
        """Text for the dashboard "RTK corrections" field."""
        now_f = time.monotonic() if now is None else float(now)
        if self._state == STATE_OFF:
            return OFF_TEXT
        if self._state == STATE_CONNECTING:
            return f"Connecting to {self._source}..."
        if self._state == STATE_ERROR:
            return f"{self._source}: {self._detail}" if self._detail else f"{self._source}: not connected"
        age = self.age_s(now_f)
        if age is None:
            return f"Connected to {self._source}, no corrections yet"
        if age > QUIET_AFTER_S:
            return f"No corrections for {age:.0f} s"
        rate = self.rate_bps(now_f)
        parts = [f"{rate / 1000.0:.1f} kB/s" if rate >= 1000.0 else f"{rate:.0f} B/s"]
        types = self.message_types(now_f)
        if types:
            parts.append(" ".join(str(t) for t in types))
        if self.station_missing(now_f):
            parts.append("no base position")
        if not self._link_up:
            parts.append("not sent, no drone link")
        return " · ".join(parts)

    def tooltip(self, now: float | None = None) -> str:
        now_f = time.monotonic() if now is None else float(now)
        if self._state == STATE_OFF:
            return (
                "No RTK base station is set. Set one in Application Settings, General, RTK base station. "
                "VGCS then passes its corrections to the drone."
            )
        if self._state == STATE_ERROR:
            return "VGCS cannot read the RTK base station. It tries again every few seconds."
        age = self.age_s(now_f)
        if self._state == STATE_CONNECTED and age is None:
            return (
                "The base is connected but sends no RTCM 3 messages. It may still be finding its own "
                "position (survey-in), or its output is set to something else."
            )
        if self.station_missing(now_f):
            return (
                "The base sends corrections but not its own position (message 1005 or 1006). "
                "The drone's GPS cannot use the corrections without it."
            )
        if self._station is not None:
            lat, lon, height = self._station
            return f"Base station at {lat:.6f}, {lon:.6f}, height {height:.1f} m. The numbers are the RTCM message types."
        return "The numbers are the RTCM message types the base sends."
