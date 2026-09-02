"""How long a mission actually takes.

Two problems with the previous estimate, both reported 2026-09-02 by an operator
looking at a 10 km plan that read ``Time: 00:14:59``.

**It used one speed for the whole mission.** The figure came from
``mission_distance / _plan_hover_speed_mps``, a panel-level default of 11.18 m/s,
so the per-waypoint speeds the operator had typed (12.0 m/s) were ignored. The
number on screen did not correspond to the plan on screen.

**It was one way, and did not say so.** The sum ran between waypoints only: not
from the aircraft to the first waypoint, and not home again. On a 10 km route
that is half the flying. An operator reading "15 minutes" and sizing a battery
around it is planning for a flight that takes thirty — and this crew lost an
aircraft to a battery failsafe the week before.

So this returns the legs separately. The caller can then say what the mission
does *and* what getting home costs, which are different numbers whenever the
mission ends in a hold.
"""

from __future__ import annotations

from dataclasses import dataclass

from vgcs.mission.mission_plan import haversine_m

__all__ = ["MissionTiming", "estimate_mission_time"]

# Used only when a waypoint carries no usable speed of its own.
DEFAULT_SPEED_MPS = 5.0


@dataclass(frozen=True)
class MissionTiming:
    """Distances and durations, split so the return leg is never hidden."""

    outbound_m: float = 0.0
    outbound_s: float = 0.0
    return_m: float = 0.0
    """Straight-line distance from the last waypoint back to the start."""
    return_s: float = 0.0
    mission_returns_home: bool = False
    """True when the plan itself flies home (end action RTL). When False the
    return legs are still populated — the aircraft must get back regardless, and
    the battery has to cover it."""

    @property
    def total_m(self) -> float:
        """Everything the aircraft must fly, including getting home."""
        return self.outbound_m + self.return_m

    @property
    def total_s(self) -> float:
        return self.outbound_s + self.return_s

    @property
    def planned_s(self) -> float:
        """Just what the mission commands — excludes the return when it holds."""
        return self.outbound_s + (self.return_s if self.mission_returns_home else 0.0)


def estimate_mission_time(
    waypoints: list[object],
    *,
    start: tuple[float, float] | None = None,
    end_action: str = "hold",
    default_speed_mps: float = DEFAULT_SPEED_MPS,
) -> MissionTiming:
    """Time the plan leg by leg, at each leg's own speed.

    ``start`` is where the aircraft is now. Given, the leg out to the first
    waypoint counts — it is real flying time and was previously ignored.
    """
    pts = _points(waypoints)
    if not pts:
        return MissionTiming()

    legs: list[tuple[float, float]] = []   # (metres, speed for that leg)
    prev = start
    for lat, lon, speed in pts:
        if prev is not None:
            legs.append((haversine_m(prev[0], prev[1], lat, lon), speed))
        prev = (lat, lon)

    out_m = sum(d for d, _ in legs)
    out_s = sum(d / s for d, s in legs if s > 0.0)

    ret_m = 0.0
    ret_s = 0.0
    if start is not None and prev is not None:
        ret_m = haversine_m(prev[0], prev[1], start[0], start[1])
        # Home at the last leg's speed — the best guess available, and RTL_SPEED
        # is not something the plan panel knows.
        speed = pts[-1][2] if pts[-1][2] > 0.0 else default_speed_mps
        ret_s = ret_m / speed

    return MissionTiming(
        outbound_m=out_m,
        outbound_s=out_s,
        return_m=ret_m,
        return_s=ret_s,
        mission_returns_home=str(end_action or "").strip().lower() == "rtl",
    )


def format_hms(seconds: float) -> str:
    s = int(max(0.0, seconds))
    return f"{s // 3600:02d}:{(s % 3600) // 60:02d}:{s % 60:02d}"


def _points(waypoints) -> list[tuple[float, float, float]]:
    out: list[tuple[float, float, float]] = []
    for wp in waypoints or []:
        lat = _f(_attr(wp, "lat"))
        lon = _f(_attr(wp, "lon"))
        if lat is None or lon is None:
            continue
        if abs(lat) < 1e-9 and abs(lon) < 1e-9:
            continue
        spd = _f(_attr(wp, "speed_mps"))
        out.append((lat, lon, spd if spd and spd > 0.0 else DEFAULT_SPEED_MPS))
    return out


def _attr(wp, key):
    if isinstance(wp, dict):
        return wp.get(key)
    return getattr(wp, key, None)


def _f(raw):
    try:
        return None if raw is None else float(raw)
    except (TypeError, ValueError):
        return None
