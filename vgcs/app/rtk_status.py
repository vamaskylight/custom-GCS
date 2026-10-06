"""RTK status of the vehicle's GPS, as the vehicle reports it (milestone M11).

Requirement 28: show the RTK fix type, the accuracy, and the correction link
status, as far as the GPS and RTK MAVLink messages carry them.

What the messages carry, checked in the ArduPilot source (libraries/AP_GPS):

``GPS_RAW_INT``
    ``fix_type`` is the whole RTK answer on most aircraft: 4 DGPS, 5 RTK Float,
    6 RTK Fixed. ``h_acc`` and ``v_acc`` are the receiver's own accuracy
    estimate in millimetres (0 when it gives none).

``GPS_RTK``
    Satellites in the RTK solution, the baseline to the base station, and the
    number of ambiguity hypotheses. Only the Septentrio (SBF), Swift (SBP) and
    Emlid (ERB) drivers send it. A u-blox receiver, the common one, sends
    nothing here, and its correction age is not in any message. So for a
    u-blox the fix type is the only sign that corrections arrive: Float or
    Fixed means they do.

Every reading has a lifetime, so a fix that stopped being reported is not
shown as the current one.

Pure: no Qt and no link code, so every rule here is unit tested.
"""

from __future__ import annotations

import math
import time

# MAVLink GPS_FIX_TYPE
FIX_NO_GPS = 0
FIX_NONE = 1
FIX_2D = 2
FIX_3D = 3
FIX_DGPS = 4
FIX_RTK_FLOAT = 5
FIX_RTK_FIXED = 6
FIX_STATIC = 7
FIX_PPP = 8

_FIX_NAMES = {
    FIX_NO_GPS: "No GPS",
    FIX_NONE: "No fix",
    FIX_2D: "2D fix",
    FIX_3D: "3D fix",
    FIX_DGPS: "DGPS",
    FIX_RTK_FLOAT: "RTK Float",
    FIX_RTK_FIXED: "RTK Fixed",
    FIX_STATIC: "Static",
    FIX_PPP: "PPP",
}

# GPS_RAW_INT is asked for twice a second.
GPS_STALE_AFTER_S = 5.0
RTK_STALE_AFTER_S = 8.0

# A receiver at the edge of a fixed solution flips between Float and Fixed.
# A change is only announced once it has lasted this long.
EVENT_STABLE_S = 2.0

LEVEL_NA = "na"
LEVEL_OK = "ok"
LEVEL_WARN = "warn"
LEVEL_PLAIN = ""

NO_DATA = "N/A"

# How a fix counts for the announcements: nothing special, RTK Float, RTK Fixed.
_CLASS_PLAIN = 0
_CLASS_FLOAT = 1
_CLASS_FIXED = 2


def fix_name(fix_type: object) -> str:
    """The name of a MAVLink GPS fix type, for example "RTK Fixed"."""
    try:
        number = int(fix_type)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return NO_DATA
    return _FIX_NAMES.get(number, f"Fix type {number}")


def _fix_class(fix_type: int) -> int:
    if fix_type == FIX_RTK_FIXED:
        return _CLASS_FIXED
    if fix_type == FIX_RTK_FLOAT:
        return _CLASS_FLOAT
    return _CLASS_PLAIN


def _positive(value: object) -> float | None:
    """A measurement that must be above zero. The messages use 0 for "not given"."""
    try:
        number = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number) or number <= 0.0:
        return None
    return number


def format_accuracy_m(value_m: float) -> str:
    """0.014 m reads "1.4 cm", 0.8 m reads "0.80 m": RTK accuracy is centimetres."""
    if value_m < 1.0:
        return f"{value_m * 100.0:.1f} cm" if value_m < 0.1 else f"{value_m:.2f} m"
    return f"{value_m:.1f} m"


class RtkStatus:
    """Hold the GPS fix, its accuracy and the RTK readings, and say what to show."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        """Forget everything (link closed, new vehicle)."""
        self._fix: int | None = None
        self._satellites: int | None = None
        self._h_acc_m: float | None = None
        self._v_acc_m: float | None = None
        self._gps_mono = 0.0
        self._rtk: dict[str, float] = {}
        self._rtk_mono = 0.0
        self._announced_class: int | None = None
        self._pending_class: int | None = None
        self._pending_since = 0.0

    # -- ingest ---------------------------------------------------------
    def update_gps(
        self,
        fix_type: object,
        *,
        satellites: object = None,
        h_acc_m: object = None,
        v_acc_m: object = None,
        now: float | None = None,
    ) -> bool:
        """Take one ``GPS_RAW_INT``. Returns False when it has no usable fix type."""
        try:
            fix = int(fix_type)  # type: ignore[arg-type]
        except (TypeError, ValueError):
            return False
        if fix < 0:
            return False
        self._fix = fix
        try:
            self._satellites = None if satellites is None else int(satellites)  # type: ignore[arg-type]
        except (TypeError, ValueError):
            self._satellites = None
        self._h_acc_m = _positive(h_acc_m)
        self._v_acc_m = _positive(v_acc_m)
        self._gps_mono = time.monotonic() if now is None else float(now)
        return True

    def update_rtk(
        self,
        *,
        satellites: object = None,
        baseline_m: object = None,
        hypotheses: object = None,
        now: float | None = None,
    ) -> None:
        """Take one ``GPS_RTK`` (only some receivers send it)."""
        reading: dict[str, float] = {}
        for key, value in (("satellites", satellites), ("baseline_m", baseline_m), ("hypotheses", hypotheses)):
            number = _positive(value)
            if number is not None:
                reading[key] = number
        self._rtk = reading
        self._rtk_mono = time.monotonic() if now is None else float(now)

    # -- read -----------------------------------------------------------
    def has_report(self) -> bool:
        """True once this link has reported a GPS state (it may be old by now)."""
        return self._fix is not None

    def fix_type(self, now: float | None = None) -> int | None:
        """The fix type while it is current, else None."""
        if self._fix is None:
            return None
        now_f = time.monotonic() if now is None else float(now)
        if now_f - self._gps_mono > GPS_STALE_AFTER_S:
            return None
        return self._fix

    def fix_text(self, now: float | None = None) -> str:
        fix = self.fix_type(now)
        return NO_DATA if fix is None else fix_name(fix)

    def header_tag(self, now: float | None = None) -> str:
        """Short text for the header, next to the satellite count. Empty for an ordinary fix."""
        fix = self.fix_type(now)
        if fix in (FIX_DGPS, FIX_RTK_FLOAT, FIX_RTK_FIXED, FIX_PPP):
            return fix_name(fix)
        return ""

    def level(self, now: float | None = None) -> str:
        fix = self.fix_type(now)
        if fix is None:
            return LEVEL_NA
        if fix == FIX_RTK_FIXED:
            return LEVEL_OK
        if fix == FIX_RTK_FLOAT:
            return LEVEL_WARN
        return LEVEL_PLAIN

    def accuracy_text(self, now: float | None = None) -> str:
        """The receiver's own accuracy estimate, or N/A when it gives none."""
        if self.fix_type(now) is None:
            return NO_DATA
        parts = []
        if self._h_acc_m is not None:
            parts.append(f"H {format_accuracy_m(self._h_acc_m)}")
        if self._v_acc_m is not None:
            parts.append(f"V {format_accuracy_m(self._v_acc_m)}")
        return " · ".join(parts) if parts else NO_DATA

    def _rtk_reading(self, now_f: float) -> dict[str, float]:
        if not self._rtk or now_f - self._rtk_mono > RTK_STALE_AFTER_S:
            return {}
        return self._rtk

    def field_text(self, now: float | None = None) -> str:
        """Text for the dashboard "RTK" field: the fix, then what is known about the corrections."""
        now_f = time.monotonic() if now is None else float(now)
        fix = self.fix_type(now_f)
        if fix is None:
            return NO_DATA
        if fix == FIX_RTK_FIXED:
            parts = ["RTK Fixed"]
        elif fix == FIX_RTK_FLOAT:
            parts = ["RTK Float"]
        elif fix == FIX_DGPS:
            parts = ["DGPS (no RTK)"]
        else:
            parts = [f"No RTK ({fix_name(fix)})"]
        rtk = self._rtk_reading(now_f)
        if "satellites" in rtk:
            parts.append(f"{rtk['satellites']:.0f} sats in solution")
        if "baseline_m" in rtk:
            baseline = rtk["baseline_m"]
            parts.append(f"base {baseline / 1000.0:.2f} km" if baseline >= 1000.0 else f"base {baseline:.0f} m")
        return " · ".join(parts)

    def field_tooltip(self, now: float | None = None) -> str:
        fix = self.fix_type(now)
        if fix is None:
            return "No GPS status from the drone."
        if fix == FIX_RTK_FIXED:
            return "Corrections arrive and the position is solved to centimetres."
        if fix == FIX_RTK_FLOAT:
            return "Corrections arrive, but the centimetre solution is not settled yet. Accuracy is decimetres."
        if fix == FIX_DGPS:
            return "The GPS uses a correction service, but not RTK."
        return (
            "The drone's GPS gets no RTK corrections, or it is not an RTK receiver. "
            "RTK needs a base station whose corrections reach the drone."
        )

    def take_event(self, now: float | None = None) -> tuple[str, bool] | None:
        """A text when the RTK state has changed and stayed changed, else None.

        Returns ``(text, is_warning)``. Reaching Float or Fixed is news. Losing
        it is a warning, because a mission flown on centimetres is then flown
        on metres.
        """
        now_f = time.monotonic() if now is None else float(now)
        fix = self.fix_type(now_f)
        if fix is None:
            # No report is not a lost fix. Wait for the next one.
            self._pending_class = None
            return None
        current = _fix_class(fix)
        if self._announced_class is None:
            # The first report of a link sets the baseline, without an announcement
            # for an ordinary fix. RTK that is already there is still worth saying.
            self._announced_class = _CLASS_PLAIN
        if current == self._announced_class:
            self._pending_class = None
            return None
        if self._pending_class != current:
            self._pending_class = current
            self._pending_since = now_f
            return None
        if now_f - self._pending_since < EVENT_STABLE_S:
            return None
        before = self._announced_class
        self._announced_class = current
        self._pending_class = None
        if current > before:
            return (fix_name(fix), False)
        if current == _CLASS_FLOAT:
            return ("RTK Fixed lost, now RTK Float", True)
        return (f"RTK lost, now {fix_name(fix)}", True)
