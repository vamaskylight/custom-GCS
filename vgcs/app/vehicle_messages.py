"""Arbitration for the single "Vehicle Msg" cell.

Field report (2026-08-24): "vehicle msg is not shown — like if gps having issue
then it says compass inconsistent".

Both halves of that had a cause:

*Not shown.* Three writers raced for one label and the vehicle lost every time.
``_on_telemetry`` set the STATUSTEXT, then called
``_refresh_dashboard_flight_state()`` in the same breath, which overwrote it
with the link-banner sentence — and that refresh also runs off
``GLOBAL_POSITION_INT`` (20 Hz) and ``VFR_HUD`` (10 Hz), so a real vehicle
message survived for well under 50 ms. On the map header, a 1 Hz timer tick
stamped ``"Gimbal Y/P: …"`` over the same cell.

*Wrong message.* Whatever did get through had no lifetime, so an old line sat
there indefinitely, indistinguishable from the current fault.

This board is the single owner of that cell. The link banner no longer writes
to it at all — it has its own widget, and duplicating it here is what buried
the vehicle. A STATUSTEXT holds the cell for a severity-scaled window; only a
message at least as severe may cut a holding one short. Once the window lapses
the text is kept but stamped with its age, so a stale fault can never be read
as the current one.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

# MAV_SEVERITY (common.xml): 0 EMERGENCY … 7 DEBUG. Lower is more severe.
SEVERITY_EMERGENCY = 0
SEVERITY_ALERT = 1
SEVERITY_CRITICAL = 2
SEVERITY_ERROR = 3
SEVERITY_WARNING = 4
SEVERITY_NOTICE = 5
SEVERITY_INFO = 6
SEVERITY_DEBUG = 7

# How long a message counts as *current*, by severity. Long enough for an
# operator watching the aircraft to look down and read it.
_HOLD_SECONDS: dict[int, float] = {
    SEVERITY_EMERGENCY: 30.0,
    SEVERITY_ALERT: 30.0,
    SEVERITY_CRITICAL: 25.0,
    SEVERITY_ERROR: 20.0,
    SEVERITY_WARNING: 15.0,
    SEVERITY_NOTICE: 10.0,
    SEVERITY_INFO: 8.0,
    SEVERITY_DEBUG: 4.0,
}
_DEFAULT_HOLD_S = 10.0

# GCS action feedback ("Mission uploaded (5)") — not something the vehicle said.
NOTICE_HOLD_S = 6.0

PLACEHOLDER = "—"


def hold_seconds_for(severity: int) -> float:
    return _HOLD_SECONDS.get(int(severity), _DEFAULT_HOLD_S)


def format_age(age_s: float) -> str:
    """Coarse age stamp. Quantised so the cell is not rewritten every frame."""
    age = max(0.0, float(age_s))
    if age < 60.0:
        return f"{int(age // 5) * 5}s ago"
    minutes = int(age // 60)
    return f"{minutes}m ago"


@dataclass(frozen=True)
class HeldMessage:
    text: str
    severity: int
    posted_mono: float
    expires_mono: float


class VehicleMessageBoard:
    """Decide what the one MESSAGE cell shows at any moment."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._held: HeldMessage | None = None
        self._notice: HeldMessage | None = None
        self._last_rendered: str | None = None

    # -- ingest ---------------------------------------------------------
    def push_vehicle_message(
        self,
        text: str,
        *,
        severity: int = SEVERITY_INFO,
        now: float | None = None,
    ) -> bool:
        """Offer a vehicle STATUSTEXT. Returns True when it takes the cell."""
        msg = str(text or "").strip()
        if not msg:
            return False
        now_f = time.monotonic() if now is None else float(now)
        sev = int(severity)
        held = self._held

        if held is not None and now_f < held.expires_mono:
            if msg == held.text:
                # A repeat of the same fault refreshes its hold rather than
                # letting it lapse while the condition is still present.
                self._held = HeldMessage(
                    msg,
                    min(sev, held.severity),
                    held.posted_mono,
                    now_f + hold_seconds_for(min(sev, held.severity)),
                )
                return True
            if sev > held.severity:
                # Less severe than the fault currently on screen: do not bury
                # it. The caller still keeps this line in the log panel.
                return False

        self._held = HeldMessage(msg, sev, now_f, now_f + hold_seconds_for(sev))
        return True

    def push_notice(
        self,
        text: str,
        *,
        hold_s: float = NOTICE_HOLD_S,
        now: float | None = None,
    ) -> None:
        """Post GCS-generated action feedback (mission upload, mode command)."""
        msg = str(text or "").strip()
        if not msg:
            return
        now_f = time.monotonic() if now is None else float(now)
        self._notice = HeldMessage(msg, SEVERITY_NOTICE, now_f, now_f + float(hold_s))

    def clear_vehicle_message(self) -> None:
        self._held = None
        self._notice = None

    # -- read -----------------------------------------------------------
    def current(self, now: float | None = None) -> str:
        """The text the cell should show right now."""
        now_f = time.monotonic() if now is None else float(now)
        held = self._held
        if held is not None and now_f < held.expires_mono:
            return held.text
        notice = self._notice
        if notice is not None and now_f < notice.expires_mono:
            return notice.text
        if held is not None:
            # Keep the information, but never let it read as current.
            return f"{held.text} ({format_age(now_f - held.posted_mono)})"
        return PLACEHOLDER

    def held_message(self, now: float | None = None) -> HeldMessage | None:
        """The vehicle message while it still counts as current."""
        now_f = time.monotonic() if now is None else float(now)
        held = self._held
        if held is None or now_f >= held.expires_mono:
            return None
        return held

    def take_render(self, now: float | None = None) -> str | None:
        """Return the text to paint, or ``None`` when it has not changed.

        The refresh path runs at 20 Hz off position telemetry; repainting an
        unchanged label from there is what made this cell flicker.
        """
        text = self.current(now)
        if text == self._last_rendered:
            return None
        self._last_rendered = text
        return text

    def invalidate_render(self) -> None:
        """Force the next :meth:`take_render` to repaint (after a widget reset)."""
        self._last_rendered = None
