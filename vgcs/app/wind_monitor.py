"""Wind, as the vehicle reports it: its wind estimate and its onboard wind failsafe.

Agreed 2026-09-16: the failsafe itself runs on the drone, so it works with any
ground station and with the link down. The ground station shows the wind
estimate with a warning. This module owns both readings.

Two sources, both from the autopilot:

``WIND``
    ArduCopter's own estimate. It is sent only when the EKF drag parameters are
    set (``EK3_DRAG_BCOEF_X``, ``EK3_DRAG_BCOEF_Y``, ``EK3_DRAG_MCOEF``), so most
    aircraft send nothing at all. "No wind shown" must therefore never read as
    "no wind": with no estimate the reading is "N/A", not "0 m/s".

``NAMED_VALUE_FLOAT`` named ``WFS_*``
    The onboard wind failsafe script (``drone/scripts/vama_wind_failsafe.lua``):
    ``WFS_ACT`` is what it is set to do, sent all the time. ``WFS_MOT``,
    ``WFS_LEAN`` and ``WFS_PUSH`` are its readings, sent in flight.

Every reading has a lifetime. A value that stopped arriving is dropped, so an
old estimate can never be read as the current wind, and a script that stopped
running is reported as not running.

Pure: no Qt and no link code, so every rule here is unit tested.
"""

from __future__ import annotations

import math
import time

# A starting value, below what small multirotors usually hold. The real limit
# depends on the aircraft and its load, so it is a setting.
DEFAULT_WARN_MPS = 8.0
MAX_WARN_MPS = 40.0
# QSettings key of that setting. 0 turns the warning off.
KEY_WIND_WARN_MPS = "wind/warn_mps"

# WIND is requested at 1 Hz, and the script reports every 1 to 2 s.
WIND_STALE_AFTER_S = 5.0
FAILSAFE_STALE_AFTER_S = 8.0

# One warning when the wind crosses the level, then a reminder while it stays
# above. The level has to drop this far below before a new crossing counts, so a
# wind that sits on the level does not warn on every gust.
REPEAT_WARNING_S = 60.0
REARM_BELOW_MPS = 1.0
MIN_WARNING_GAP_S = 20.0

# A wind estimate above this is not wind. It is a broken estimate.
_MAX_PLAUSIBLE_MPS = 80.0
# And below this it is "no estimate yet", which the autopilot sends as zero.
_MIN_ESTIMATE_MPS = 0.02

ACTION_OFF = 0
ACTION_WARN = 1
ACTION_RTL = 2
ACTION_LAND = 3
_ACTION_TEXT = {
    ACTION_OFF: "Off",
    ACTION_WARN: "Warning only",
    ACTION_RTL: "RTL",
    ACTION_LAND: "Land",
}

FAILSAFE_PREFIX = "WFS_"
_NAME_ACTION = "WFS_ACT"
_NAME_MOTOR = "WFS_MOT"
_NAME_LEAN = "WFS_LEAN"
_NAME_PUSH = "WFS_PUSH"

LEVEL_NA = "na"
LEVEL_OK = "ok"
LEVEL_WARN = "warn"

NO_ESTIMATE = "N/A"
NOT_RUNNING = "Not running"

_COMPASS_POINTS = ("N", "NE", "E", "SE", "S", "SW", "W", "NW")


def compass_point(deg: float) -> str:
    """Eight-point name of a direction in degrees: 0 is N, 90 is E."""
    return _COMPASS_POINTS[int(((float(deg) % 360.0) + 22.5) // 45.0) % 8]


def clamp_warn_mps(value: object) -> float:
    """A warning level from a setting. 0 means no warning. Bad input gives the default."""
    try:
        level = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return DEFAULT_WARN_MPS
    if not math.isfinite(level):
        return DEFAULT_WARN_MPS
    return max(0.0, min(MAX_WARN_MPS, level))


def _number(value: object) -> float | None:
    try:
        number = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


class WindMonitor:
    """Hold the vehicle's wind readings and say what to show."""

    def __init__(self, warn_mps: float = DEFAULT_WARN_MPS) -> None:
        self._warn_mps = clamp_warn_mps(warn_mps)
        self.reset()

    def reset(self) -> None:
        """Forget every reading (link closed, new vehicle). The warning level stays."""
        self._speed_mps: float | None = None
        self._from_deg: float | None = None
        self._wind_mono = 0.0
        self._failsafe: dict[str, tuple[float, float]] = {}
        self._above = False
        self._last_warning_mono: float | None = None

    # -- settings -------------------------------------------------------
    @property
    def warn_mps(self) -> float:
        return self._warn_mps

    def set_warn_mps(self, value: object) -> None:
        self._warn_mps = clamp_warn_mps(value)

    # -- ingest ---------------------------------------------------------
    def update_wind(self, speed_mps: object, from_deg: object, now: float | None = None) -> bool:
        """Take one ``WIND`` message. ``from_deg`` is where the wind comes from.

        Returns False when the message is not a usable estimate.
        """
        speed = _number(speed_mps)
        direction = _number(from_deg)
        if speed is None or direction is None or speed < 0.0 or speed > _MAX_PLAUSIBLE_MPS:
            return False
        if speed < _MIN_ESTIMATE_MPS:
            # On the ground ArduCopter has no estimate yet and sends 0 m/s (seen on
            # 4.6.2 and 4.7.0 as "0 m/s from 180"). That is not calm air. A real
            # estimate in calm air still reads about 0.1 m/s.
            return False
        self._speed_mps = speed
        # ArduCopter sends -180 to 180.
        self._from_deg = direction % 360.0
        self._wind_mono = time.monotonic() if now is None else float(now)
        return True

    def update_failsafe_value(self, name: object, value: object, now: float | None = None) -> bool:
        """Take one ``NAMED_VALUE_FLOAT``. Returns False when it is not from the wind failsafe."""
        key = str(name or "").strip().strip("\x00").upper()
        number = _number(value)
        if not key.startswith(FAILSAFE_PREFIX) or number is None:
            return False
        self._failsafe[key] = (number, time.monotonic() if now is None else float(now))
        return True

    # -- wind estimate --------------------------------------------------
    def wind_speed_mps(self, now: float | None = None) -> float | None:
        """The estimate while it is current, else None."""
        if self._speed_mps is None:
            return None
        now_f = time.monotonic() if now is None else float(now)
        if now_f - self._wind_mono > WIND_STALE_AFTER_S:
            return None
        return self._speed_mps

    def wind_from_deg(self, now: float | None = None) -> float | None:
        return None if self.wind_speed_mps(now) is None else self._from_deg

    def wind_level(self, now: float | None = None) -> str:
        speed = self.wind_speed_mps(now)
        if speed is None:
            return LEVEL_NA
        if self._warn_mps > 0.0 and speed >= self._warn_mps:
            return LEVEL_WARN
        return LEVEL_OK

    def strip_text(self, now: float | None = None) -> str:
        """Short text for the map strip, or empty when there is no estimate."""
        speed = self.wind_speed_mps(now)
        if speed is None:
            return ""
        return f"Wind {speed:.0f} m/s {compass_point(self._from_deg or 0.0)}"

    def field_text(self, now: float | None = None) -> str:
        """Text for the dashboard field."""
        speed = self.wind_speed_mps(now)
        if speed is None:
            return NO_ESTIMATE
        direction = self._from_deg or 0.0
        return f"{speed:.1f} m/s from {compass_point(direction)} ({direction:.0f}°)"

    def field_tooltip(self, now: float | None = None) -> str:
        if self.wind_speed_mps(now) is None:
            return (
                "The drone sends no wind estimate. It sends one only when its drag "
                "parameters are set (EK3_DRAG_BCOEF_X, EK3_DRAG_BCOEF_Y, EK3_DRAG_MCOEF)."
            )
        if self._warn_mps > 0.0:
            return f"The drone's own wind estimate. Warning level {self._warn_mps:g} m/s."
        return "The drone's own wind estimate. The wind warning is off."

    def take_warning(self, now: float | None = None) -> str | None:
        """A warning text when the wind has just crossed the level, else None.

        Call it after each update. It also returns a reminder every
        ``REPEAT_WARNING_S`` while the wind stays above the level.
        """
        now_f = time.monotonic() if now is None else float(now)
        speed = self.wind_speed_mps(now_f)
        if speed is None or self._warn_mps <= 0.0:
            # No estimate is not calm air. Keep the state, so a short gap in the
            # stream does not turn into a second "crossing".
            return None
        if speed < self._warn_mps - REARM_BELOW_MPS:
            self._above = False
            return None
        if speed < self._warn_mps:
            return None
        # A new crossing warns at once, but never sooner than MIN_WARNING_GAP_S
        # after the last warning. While the wind stays above, it reminds.
        wait_s = REPEAT_WARNING_S if self._above else MIN_WARNING_GAP_S
        if self._last_warning_mono is not None and now_f - self._last_warning_mono < wait_s:
            return None
        self._above = True
        self._last_warning_mono = now_f
        return f"Wind {speed:.0f} m/s, above the {self._warn_mps:g} m/s warning level"

    # -- onboard wind failsafe ------------------------------------------
    def _failsafe_value(self, name: str, now_f: float) -> float | None:
        held = self._failsafe.get(name)
        if held is None or now_f - held[1] > FAILSAFE_STALE_AFTER_S:
            return None
        return held[0]

    def failsafe_action(self, now: float | None = None) -> int | None:
        """What the onboard script is set to do, or None when it is not running."""
        now_f = time.monotonic() if now is None else float(now)
        value = self._failsafe_value(_NAME_ACTION, now_f)
        if value is None:
            return None
        return int(round(value))

    def failsafe_level(self, now: float | None = None) -> str:
        action = self.failsafe_action(now)
        if action is None:
            return LEVEL_NA
        return LEVEL_OK if action in (ACTION_RTL, ACTION_LAND) else LEVEL_WARN

    def failsafe_text(self, now: float | None = None) -> str:
        """Text for the dashboard field: the action, and in flight the readings."""
        now_f = time.monotonic() if now is None else float(now)
        action = self.failsafe_action(now_f)
        if action is None:
            return NOT_RUNNING
        parts = [_ACTION_TEXT.get(action, f"Action {action}")]
        motor = self._failsafe_value(_NAME_MOTOR, now_f)
        lean = self._failsafe_value(_NAME_LEAN, now_f)
        push = self._failsafe_value(_NAME_PUSH, now_f)
        if motor is not None:
            parts.append(f"motors {motor:.0f}%")
        if lean is not None:
            parts.append(f"lean {lean:.0f}°")
        if push is not None and push >= 0.5:
            parts.append(f"pushed {push:.1f} m/s")
        return " · ".join(parts)

    def failsafe_tooltip(self, now: float | None = None) -> str:
        action = self.failsafe_action(now)
        if action is None:
            return (
                "The drone does not report a wind failsafe. The script "
                "vama_wind_failsafe.lua is not installed, or scripting is off (SCR_ENABLE)."
            )
        if action == ACTION_RTL:
            return "In wind it cannot hold, the drone returns home. If it cannot, it lands."
        if action == ACTION_LAND:
            return "In wind it cannot hold, the drone lands where it is."
        if action == ACTION_WARN:
            return "The drone reports strong wind but does not change the flight mode (WFS_ACTION 1)."
        return "The wind failsafe is installed but turned off (WFS_ACTION 0)."
