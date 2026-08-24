"""Stable operator-facing battery reading from noisy MAVLink telemetry.

Field report (2026-08-24): "battery voltage is not stable while the drone is
flying, it's continuously changing like 25V after that 23V then 22.04V".

Three separate causes fed that:

* ``SYS_STATUS.voltage_battery`` is the *instantaneous* pack voltage. ArduPilot
  sends the raw sample (``AP_BattMonitor::gcs_voltage``, sag-compensated only
  when ``BATT_OPTIONS`` sets ``GCS_Resting_Voltage``), so throttle changes swing
  it by volts and the old ``f"{v:.2f}V"`` header re-rendered every 500 ms.
* MAVLink "field unknown" sentinels were rendered as data: ``voltage_battery ==
  UINT16_MAX`` came out as ``65.54V`` and a not-measured ``0`` as ``0.00V``.
* Two different messages (``SYS_STATUS`` and ``BATTERY_STATUS``) wrote the same
  header cell with no arbitration between them.

This module owns all three: sentinel rejection, single-source arbitration, and
a spike-rejecting low-pass whose *published* value only moves once the pack has
genuinely moved. The last accepted instantaneous sample stays available via
:attr:`BatteryTracker.raw_voltage_v`, so nothing real is hidden.
"""

from __future__ import annotations

import math
from collections import deque

# MAVLink "field unknown" sentinels (common.xml).
MAVLINK_UNKNOWN_VOLTAGE_MV = 0xFFFF
MAVLINK_UNKNOWN_CELL_MV = 0xFFFF

# Median-of-N over accepted samples: kills a single corrupt/outlier reading
# without the lag a longer window would add.
_SPIKE_WINDOW = 3
# Exponential low-pass time constant. At the 2 Hz SYS_STATUS rate this settles a
# genuine step within ~10 s while erasing per-sample throttle sag.
_SMOOTH_TAU_S = 4.0
_CURRENT_TAU_S = 2.0
# The published value only moves when the filtered value moved at least this
# much. Without it the last digit dances forever on a perfectly healthy pack.
_DISPLAY_DEADBAND_V = 0.08
_DISPLAY_DEADBAND_A = 0.5
# No accepted sample for this long means the reading on screen is not live.
STALE_AFTER_S = 6.0

# Highest-priority source that is currently live wins; the other is ignored so
# the two can never fight over the same header cell.
_SOURCE_PRIORITY = ("SYS_STATUS", "BATTERY_STATUS")
_SOURCE_TAKEOVER_S = 3.0


def _ema_alpha(dt_s: float, tau_s: float) -> float:
    """Rate-independent EMA weight, so a 2 Hz and a 10 Hz stream smooth alike."""
    if tau_s <= 0.0:
        return 1.0
    if dt_s <= 0.0:
        # Two samples stamped at the same instant: no time has passed, so the
        # second one carries no new information for the filter.
        return 0.0
    return float(min(1.0, max(0.0, 1.0 - math.exp(-dt_s / tau_s))))


def _median(values: list[float]) -> float:
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return (ordered[mid - 1] + ordered[mid]) / 2.0


def voltage_v_from_mv(raw_mv: object) -> float | None:
    """Convert a MAVLink mV field to volts, or ``None`` when it means "unknown"."""
    try:
        mv = int(raw_mv)
    except (TypeError, ValueError):
        return None
    if mv <= 0 or mv == MAVLINK_UNKNOWN_VOLTAGE_MV:
        return None
    return float(mv) / 1000.0


def pack_voltage_v_from_cells(cells_mv: object) -> float | None:
    """Sum a ``BATTERY_STATUS.voltages`` cell array, skipping unknown cells."""
    if not isinstance(cells_mv, (list, tuple)):
        return None
    total_mv = 0
    for raw in cells_mv:
        try:
            mv = int(raw)
        except (TypeError, ValueError):
            continue
        # The array is zero-padded past the last real cell; UINT16_MAX marks a
        # cell the monitor cannot read.
        if mv <= 0 or mv == MAVLINK_UNKNOWN_CELL_MV:
            continue
        total_mv += mv
    if total_mv <= 0:
        return None
    return float(total_mv) / 1000.0


class BatteryTracker:
    """Arbitrate, filter and format the vehicle battery reading."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._samples: deque[float] = deque(maxlen=_SPIKE_WINDOW)
        self._filtered_v: float | None = None
        self._published_v: float | None = None
        self._raw_v: float | None = None
        self._filtered_a: float | None = None
        self._published_a: float | None = None
        self._remaining_pct: int | None = None
        self._last_accept_mono: float | None = None
        self._source_seen_mono: dict[str, float] = {}
        self._active_source: str | None = None

    # -- ingest ---------------------------------------------------------
    def preferred_source(self, now: float) -> str | None:
        """Which message type currently owns the reading (highest live priority)."""
        for name in _SOURCE_PRIORITY:
            seen = self._source_seen_mono.get(name)
            if seen is not None and (float(now) - seen) <= _SOURCE_TAKEOVER_S:
                return name
        return None

    def update(
        self,
        *,
        source: str,
        now: float,
        voltage_v: float | None,
        current_a: float | None = None,
        remaining_pct: int | None = None,
    ) -> bool:
        """Feed one telemetry sample. Returns True when it was accepted."""
        src = str(source)
        now_f = float(now)
        if voltage_v is None:
            # A frame with no usable voltage must not claim the source nor blank
            # a good reading.
            return False
        v = float(voltage_v)
        if not math.isfinite(v) or v <= 0.1:
            return False
        self._source_seen_mono[src] = now_f
        if self.preferred_source(now_f) != src:
            return False

        if src != self._active_source:
            # A different sensor is a different signal: carrying the old
            # source through the median window would blend two packs.
            self._active_source = src
            self._samples.clear()
            self._filtered_v = None
            self._filtered_a = None
            self._last_accept_mono = None

        last = self._last_accept_mono
        self._raw_v = v
        self._samples.append(v)
        median_v = _median(list(self._samples))

        if self._filtered_v is None or last is None:
            self._filtered_v = median_v
        else:
            self._filtered_v += _ema_alpha(now_f - last, _SMOOTH_TAU_S) * (
                median_v - self._filtered_v
            )

        if (
            self._published_v is None
            or abs(self._filtered_v - self._published_v) >= _DISPLAY_DEADBAND_V
        ):
            self._published_v = self._filtered_v

        if current_a is not None:
            a = float(current_a)
            if math.isfinite(a) and a >= 0.0:
                if self._filtered_a is None or last is None:
                    self._filtered_a = a
                else:
                    self._filtered_a += _ema_alpha(now_f - last, _CURRENT_TAU_S) * (
                        a - self._filtered_a
                    )
                if (
                    self._published_a is None
                    or abs(self._filtered_a - self._published_a) >= _DISPLAY_DEADBAND_A
                ):
                    self._published_a = self._filtered_a

        if remaining_pct is not None and int(remaining_pct) >= 0:
            self._remaining_pct = int(remaining_pct)

        self._last_accept_mono = now_f
        return True

    # -- read -----------------------------------------------------------
    @property
    def voltage_v(self) -> float | None:
        """Published (deadbanded) voltage — the number shown to the operator."""
        return self._published_v

    @property
    def raw_voltage_v(self) -> float | None:
        """Last accepted instantaneous sample, unfiltered."""
        return self._raw_v

    @property
    def current_a(self) -> float | None:
        return self._published_a

    @property
    def remaining_pct(self) -> int | None:
        return self._remaining_pct

    def is_stale(self, now: float) -> bool:
        if self._last_accept_mono is None:
            return True
        return (float(now) - self._last_accept_mono) > STALE_AFTER_S

    # -- format ---------------------------------------------------------
    def _pct_text(self) -> str:
        pct = self._remaining_pct
        return "N/A" if pct is None or pct < 0 else f"{pct}%"

    def header_text(self) -> str:
        """Compact header pill: one decimal is all an operator can act on."""
        if self._published_v is None:
            return "N/A"
        pct_text = self._pct_text()
        if pct_text == "N/A":
            return f"{self._published_v:.1f}V"
        return f"{self._published_v:.1f}V ({pct_text})"

    def detail_text(self) -> str:
        """Dashboard row: the same published voltage, plus current and remaining."""
        if self._published_v is None:
            return "—"
        amps = self._published_a
        current_text = "N/A" if amps is None else f"{amps:.1f} A"
        return f"{self._published_v:.2f} V, {current_text}, {self._pct_text()}"

    def detail_tooltip(self) -> str:
        """Where the unfiltered sample stays visible — nothing real is hidden."""
        if self._raw_v is None or self._published_v is None:
            return ""
        return (
            f"Live sample: {self._raw_v:.2f} V\n"
            f"Displayed: {self._published_v:.2f} V (smoothed over "
            f"{_SMOOTH_TAU_S:.0f} s — pack voltage sags with throttle)"
        )
