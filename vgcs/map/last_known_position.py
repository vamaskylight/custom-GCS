"""Remember where the aircraft was, so a lost link is not a lost aircraft.

Field report 2026-08-31: an aircraft went down under a battery failsafe and the
operator asked where it was. VGCS could not say. It had the position five times a
second for the whole flight and kept none of it — the console prints a position
only on connect, nothing is written to disk, and closing the window discarded
even the scrollback.

So this module does the minimum that would have answered the question:

* keep the most recent fix in memory, with a monotonic stamp so staleness is
  reportable rather than implied;
* write it to ``QSettings`` on a slow cadence, so it survives the app being
  closed or crashing — the aircraft was lost on a night when VGCS was restarted
  before anyone thought to look;
* hand it back formatted with an MGRS grid reference, because that is what the
  recovery party actually navigates with.

The write cadence is deliberately slow (``_PERSIST_MIN_INTERVAL_S``). At 5 Hz a
naive implementation would hammer the registry thousands of times per flight for
no benefit: what matters for recovery is the position to within a few seconds,
not the last one.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

from vgcs.map.app_settings import QS_APP, QS_ORG

_KEY_LAT = "last_known/lat"
_KEY_LON = "last_known/lon"
_KEY_ALT = "last_known/alt_msl_m"
_KEY_REL_ALT = "last_known/rel_alt_m"
_KEY_EPOCH = "last_known/epoch_s"

# Persisting every fix would write to the registry ~5x/second all flight for no
# recovery benefit. A few seconds of staleness is irrelevant when searching.
_PERSIST_MIN_INTERVAL_S = 10.0


@dataclass(frozen=True)
class LastKnownPosition:
    """A fix, and how old it is. Age is part of the answer, not a detail."""

    lat: float
    lon: float
    alt_msl_m: float | None = None
    rel_alt_m: float | None = None
    epoch_s: float | None = None

    def age_s(self, *, now: float | None = None) -> float | None:
        if self.epoch_s is None:
            return None
        return max(0.0, (time.time() if now is None else now) - float(self.epoch_s))

    def grid_reference(self) -> str:
        try:
            from vgcs.observe.grid_reference import latlon_to_mgrs

            return str(latlon_to_mgrs(self.lat, self.lon) or "")
        except Exception:
            return ""

    def describe(self, *, now: float | None = None) -> str:
        """One line a person can read out over a radio."""
        out = f"{self.lat:.7f}, {self.lon:.7f}"
        grid = self.grid_reference()
        if grid:
            out += f"  ({grid})"
        if self.rel_alt_m is not None:
            out += f"  {self.rel_alt_m:.0f} m AGL"
        age = self.age_s(now=now)
        if age is not None:
            out += f"  ·  {_format_age(age)} ago"
        return out


def _format_age(seconds: float) -> str:
    s = int(max(0.0, seconds))
    if s < 60:
        return f"{s}s"
    if s < 3600:
        return f"{s // 60}m {s % 60}s"
    return f"{s // 3600}h {(s % 3600) // 60}m"


class LastKnownPositionStore:
    """Holds the newest fix and mirrors it to disk on a slow cadence."""

    def __init__(self, *, settings_factory=None) -> None:
        self._current: LastKnownPosition | None = None
        self._last_persist_mono = 0.0
        self._settings_factory = settings_factory

    # -- recording ------------------------------------------------------- #

    def record(
        self,
        lat: float,
        lon: float,
        *,
        alt_msl_m: float | None = None,
        rel_alt_m: float | None = None,
        now_epoch: float | None = None,
        now_mono: float | None = None,
    ) -> None:
        try:
            la, lo = float(lat), float(lon)
        except (TypeError, ValueError):
            return
        # (0, 0) is what an autopilot reports before it has a fix. Storing it
        # would send a search party into the Gulf of Guinea.
        if abs(la) < 1e-9 and abs(lo) < 1e-9:
            return
        self._current = LastKnownPosition(
            lat=la,
            lon=lo,
            alt_msl_m=_f(alt_msl_m),
            rel_alt_m=_f(rel_alt_m),
            epoch_s=float(time.time() if now_epoch is None else now_epoch),
        )
        mono = time.monotonic() if now_mono is None else float(now_mono)
        if mono - self._last_persist_mono >= _PERSIST_MIN_INTERVAL_S:
            self._last_persist_mono = mono
            self.persist()

    def current(self) -> LastKnownPosition | None:
        return self._current

    # -- persistence ------------------------------------------------------ #

    def _settings(self):
        if self._settings_factory is not None:
            return self._settings_factory()
        from PySide6.QtCore import QSettings

        return QSettings(QS_ORG, QS_APP)

    def persist(self) -> bool:
        pos = self._current
        if pos is None:
            return False
        try:
            s = self._settings()
            s.setValue(_KEY_LAT, f"{pos.lat:.7f}")
            s.setValue(_KEY_LON, f"{pos.lon:.7f}")
            s.setValue(_KEY_ALT, "" if pos.alt_msl_m is None else f"{pos.alt_msl_m:.2f}")
            s.setValue(_KEY_REL_ALT, "" if pos.rel_alt_m is None else f"{pos.rel_alt_m:.2f}")
            s.setValue(_KEY_EPOCH, f"{float(pos.epoch_s or 0.0):.0f}")
            return True
        except Exception:
            return False

    def load(self) -> LastKnownPosition | None:
        """Restore the fix from a previous run. This is the whole point: the
        aircraft was lost, VGCS was restarted, and the position had to survive
        that to be of any use."""
        try:
            s = self._settings()
            lat = _f(s.value(_KEY_LAT, ""))
            lon = _f(s.value(_KEY_LON, ""))
            if lat is None or lon is None:
                return None
            pos = LastKnownPosition(
                lat=lat,
                lon=lon,
                alt_msl_m=_f(s.value(_KEY_ALT, "")),
                rel_alt_m=_f(s.value(_KEY_REL_ALT, "")),
                epoch_s=_f(s.value(_KEY_EPOCH, "")),
            )
        except Exception:
            return None
        if self._current is None:
            self._current = pos
        return pos


def _f(raw) -> float | None:
    if raw is None:
        return None
    text = str(raw).strip()
    if not text:
        return None
    try:
        return float(text)
    except (TypeError, ValueError):
        return None
