"""How long a mission takes on the battery: the climb, the route, the hovering, the way home and the landing.

The battery check of Upload and Start Mission (vgcs/mission/long_range_safety.py)
used one speed for the whole round trip and left out the climb, the landing and
the wind: a known limit of the operator guide until 2026-10-20. This times each
part the way ArduCopter flies the mission VGCS uploads:

- the take-off, straight up to the take-off height (the first waypoint's height
  unless Launch Position sets one), at the drone's climb speed;
- each leg at its own speed, or longer when its change of height takes longer at
  the drone's climb or descent speed;
- the hover at the waypoints, as uploaded (whole seconds, at most an hour);
- the way home as RTL flies it: up to the RTL height when below it, straight home
  at the RTL speed, and the wait over home (RTL_LOIT_TIME);
- the landing: down to the low landing height at the high landing speed, the
  rest at the landing speed.

The wind. ArduCopter keeps its speed over the ground in wind only while it can
lean far enough, and a drone flown near its top speed cannot. In the simulator
(2026-10-20, 4.6.2) a quad that returned at 10 m/s came home at 4.2 m/s against
6 m/s of wind, about what 10 minus 6 says, and drew about the same current all
that longer time. VGCS does not know a drone's top speed, so it takes the return
speed as the most the drone flies through the air: against a headwind the way
home is slower over the ground by the wind's speed. A drone that can fly faster
than its return speed does better. When the wind is as fast as the way home, the
drone does not get home (wind_too_strong, the way home takes for ever).

The speeds are the drone's own (the 4.6 names in centimetres, the 4.7 names in
metres), or ArduPilot's defaults when they have not been read. Speeding up,
slowing down and turning at each waypoint are not in it: they add a little.

Pure Python, no Qt.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from vgcs.mission.mission_plan import MIN_WP_ALT_M, MIN_WP_SPEED_MPS, haversine_m
from vgcs.mission.waypoint_store import clamp_hover_seconds

__all__ = ["DroneSpeeds", "FlightTime", "SPEED_SETTINGS", "drone_speeds", "mission_flight_time"]

# ArduCopter's own defaults (4.6.2 and 4.7.0 alike, read from the simulator on 2026-10-20).
DEFAULT_CLIMB_MPS = 2.5        # WPNAV_SPEED_UP 250 cm/s, WP_SPD_UP 2.5 m/s
DEFAULT_DESCENT_MPS = 1.5      # WPNAV_SPEED_DN 150 cm/s, WP_SPD_DN 1.5 m/s
DEFAULT_LAND_MPS = 0.5         # LAND_SPEED 50 cm/s, LAND_SPD_MS 0.5 m/s
DEFAULT_LAND_LOW_M = 10.0      # LAND_ALT_LOW 1000 cm, LAND_ALT_LOW_M 10 m
DEFAULT_RTL_LOITER_S = 5.0     # RTL_LOIT_TIME 5000 ms
# A waypoint without a height or a speed of its own (as a new waypoint gets them).
DEFAULT_WP_ALT_M = 20.0
DEFAULT_WP_SPEED_MPS = 5.0

# The drone's settings this reads: (4.7 name in metres, 4.6 name in centimetres),
# and RTL_LOIT_TIME (milliseconds in both). VGCS reads them on connect
# (vgcs/app/vehicle_params.py).
_RENAMED = (
    ("WP_SPD_UP", "WPNAV_SPEED_UP"),
    ("WP_SPD_DN", "WPNAV_SPEED_DN"),
    ("LAND_SPD_MS", "LAND_SPEED"),
    ("LAND_SPD_HIGH_MS", "LAND_SPEED_HIGH"),
    ("LAND_ALT_LOW_M", "LAND_ALT_LOW"),
    ("RTL_ALT_M", "RTL_ALT"),
    ("RTL_SPEED_MS", "RTL_SPEED"),
    ("WP_SPD", "WPNAV_SPEED"),
)
SPEED_SETTINGS = tuple(name for pair in _RENAMED for name in pair) + ("RTL_LOIT_TIME",)


@dataclass(frozen=True)
class DroneSpeeds:
    """The drone's speeds that set how long a mission takes."""

    climb_mps: float = DEFAULT_CLIMB_MPS
    descent_mps: float = DEFAULT_DESCENT_MPS
    land_high_mps: float = DEFAULT_DESCENT_MPS   # down to land_low_m
    land_mps: float = DEFAULT_LAND_MPS           # the last land_low_m
    land_low_m: float = DEFAULT_LAND_LOW_M
    rtl_alt_m: float | None = None               # None: not known, the way home keeps its height
    rtl_speed_mps: float | None = None           # None: not known
    rtl_loiter_s: float = DEFAULT_RTL_LOITER_S
    read_from_drone: bool = False                # the climb and descent speeds are the drone's own


def _number(params: dict, name: str) -> float | None:
    try:
        value = float(params[name])
    except (KeyError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _either(params: dict, metres_name: str, centimetres_name: str) -> float | None:
    """A 4.7 setting in metres, else its 4.6 name in centimetres, in metres."""
    value = _number(params, metres_name)
    if value is not None:
        return value
    value = _number(params, centimetres_name)
    return None if value is None else value / 100.0


def drone_speeds(params: dict | None) -> DroneSpeeds:
    """The speeds from the drone's settings (4.6 or 4.7 names); ArduPilot's defaults for what is not read."""
    p = {str(k).strip().upper(): v for k, v in (params or {}).items()}
    climb, descent, land, land_high, land_low, rtl_alt, rtl_speed, wp_speed = (
        _either(p, metres, centimetres) for metres, centimetres in _RENAMED
    )
    loiter_ms = _number(p, "RTL_LOIT_TIME")

    read = climb is not None and climb > 0.0 and descent is not None and descent > 0.0
    climb = climb if climb is not None and climb > 0.0 else DEFAULT_CLIMB_MPS
    descent = descent if descent is not None and descent > 0.0 else DEFAULT_DESCENT_MPS
    land = land if land is not None and land > 0.0 else DEFAULT_LAND_MPS
    # 0 means "at the descent speed" in both versions.
    land_high = land_high if land_high is not None and land_high > 0.0 else descent
    land_low = land_low if land_low is not None and land_low >= 0.0 else DEFAULT_LAND_LOW_M
    # RTL_SPEED 0 means "at the mission speed setting" (WPNAV_SPEED or WP_SPD).
    if rtl_speed is None or rtl_speed <= 0.0:
        rtl_speed = wp_speed if wp_speed is not None and wp_speed > 0.0 else None
    return DroneSpeeds(
        climb_mps=climb,
        descent_mps=descent,
        land_high_mps=land_high,
        land_mps=land,
        land_low_m=land_low,
        rtl_alt_m=rtl_alt if rtl_alt is not None and rtl_alt >= 0.0 else None,
        rtl_speed_mps=rtl_speed,
        rtl_loiter_s=loiter_ms / 1000.0 if loiter_ms is not None and loiter_ms >= 0.0 else DEFAULT_RTL_LOITER_S,
        read_from_drone=read,
    )


@dataclass(frozen=True)
class FlightTime:
    """The parts of a mission's time on the battery, in seconds."""

    climb_s: float = 0.0
    route_s: float = 0.0
    hover_s: float = 0.0
    home_s: float = 0.0          # up to the RTL height, straight home, and the wait over home
    landing_s: float = 0.0
    home_speed_mps: float = 0.0  # the return speed (over the ground in still air)
    headwind_mps: float = 0.0

    @property
    def total_s(self) -> float:
        return self.climb_s + self.route_s + self.hover_s + self.home_s + self.landing_s

    @property
    def wind_too_strong(self) -> bool:
        """The headwind is as fast as the way home: the drone does not get home, unless it can fly faster."""
        return self.headwind_mps > 0.0 and not math.isfinite(self.home_s)

    def parts(self) -> str:
        """In words: "climb 0.2 min, route 4.1 min, way home 2.0 min against 6 m/s of wind, landing 0.5 min"."""
        words = [f"climb {self.climb_s / 60.0:.1f} min", f"route {self.route_s / 60.0:.1f} min"]
        if self.hover_s >= 3.0:   # less shows as 0.0 min
            words.append(f"hovering {self.hover_s / 60.0:.1f} min")
        home = f"way home {self.home_s / 60.0:.1f} min" if math.isfinite(self.home_s) else "no way home"
        if self.headwind_mps > 0.0:
            home += f" against {round(self.headwind_mps, 1):g} m/s of wind"
        words.append(home)
        words.append(f"landing {self.landing_s / 60.0:.1f} min")
        return ", ".join(words)


def _attr(wp, key):
    if isinstance(wp, dict):
        return wp.get(key)
    return getattr(wp, key, None)


def _value(raw, default: float, *, low: float = 0.0) -> float:
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) and value >= low else default


def _points(waypoints) -> list[tuple[float, float, float, float, float]]:
    """(lat, lon, height, speed, hover) of each waypoint with a position, as the upload sends them."""
    out = []
    for wp in waypoints or []:
        try:
            lat, lon = float(_attr(wp, "lat")), float(_attr(wp, "lon"))
        except (TypeError, ValueError):
            continue
        if not (math.isfinite(lat) and math.isfinite(lon)) or (abs(lat) < 1e-9 and abs(lon) < 1e-9):
            continue
        alt = max(MIN_WP_ALT_M, _value(_attr(wp, "alt_m"), DEFAULT_WP_ALT_M, low=float("-inf")))
        speed = max(MIN_WP_SPEED_MPS, _value(_attr(wp, "speed_mps"), DEFAULT_WP_SPEED_MPS, low=float("-inf")))
        # An endless or unreadable hover is none (clamp_hover_seconds cannot take infinity).
        hover = float(clamp_hover_seconds(_value(_attr(wp, "hover_s"), 0.0)))
        out.append((lat, lon, alt, speed, hover))
    return out


def _vertical_s(change_m: float, speeds: DroneSpeeds) -> float:
    if change_m > 0.0:
        return change_m / speeds.climb_mps
    return -change_m / speeds.descent_mps


def mission_flight_time(
    start: tuple[float, float] | None,
    waypoints: list[object],
    speeds: DroneSpeeds,
    *,
    home: tuple[float, float] | None = None,
    headwind_mps: float = 0.0,
    fallback_speed_mps: float | None = None,
    takeoff_alt_m: float | None = None,
) -> FlightTime:
    """The mission's time from the take-off at `start` until the landing at `home` (where RTL flies to).

    Heights are above home. The way home runs at the drone's RTL speed, else at
    `fallback_speed_mps`, else at the last waypoint's speed. `takeoff_alt_m` is
    the take-off height the upload sends (None: the first waypoint's height).
    """
    pts = _points(waypoints)
    if start is None or not pts:
        return FlightTime()
    home = start if home is None else home
    headwind = _value(headwind_mps, 0.0)

    takeoff = pts[0][2]
    if takeoff_alt_m is not None:
        takeoff = max(MIN_WP_ALT_M, _value(takeoff_alt_m, takeoff, low=float("-inf")))
    climb_s = takeoff / speeds.climb_mps
    route_s = 0.0
    prev_lat, prev_lon, prev_alt = float(start[0]), float(start[1]), takeoff
    for lat, lon, alt, speed, _hover in pts:
        flat_s = haversine_m(prev_lat, prev_lon, lat, lon) / speed
        route_s += max(flat_s, _vertical_s(alt - prev_alt, speeds))
        prev_lat, prev_lon, prev_alt = lat, lon, alt
    hover_s = sum(p[4] for p in pts)

    last_lat, last_lon, last_alt, last_speed, _ = pts[-1]
    rtl_alt = speeds.rtl_alt_m
    up_s = max(0.0, rtl_alt - last_alt) / speeds.climb_mps if rtl_alt is not None else 0.0
    home_alt = max(last_alt, rtl_alt) if rtl_alt is not None else last_alt
    speed_home = next(s for s in (_value(speeds.rtl_speed_mps, 0.0), _value(fallback_speed_mps, 0.0), last_speed)
                      if s > 0.0)
    distance = haversine_m(last_lat, last_lon, float(home[0]), float(home[1]))
    # Against the wind the way home is slower over the ground (see the module's text).
    ground = speed_home - headwind
    if distance < 1.0:
        cruise_s = 0.0                 # it ends over home
    elif ground > 0.0:
        cruise_s = distance / ground
    else:
        cruise_s = math.inf
    home_s = up_s + cruise_s + speeds.rtl_loiter_s
    high_part = max(0.0, home_alt - speeds.land_low_m)
    landing_s = high_part / speeds.land_high_mps + min(home_alt, speeds.land_low_m) / speeds.land_mps
    return FlightTime(
        climb_s=climb_s,
        route_s=route_s,
        hover_s=hover_s,
        home_s=home_s,
        landing_s=landing_s,
        home_speed_mps=speed_home,
        headwind_mps=headwind,
    )
