"""What the drone's proximity sensor (a LiDAR) sees, from the DISTANCE_SENSOR messages ArduPilot sends for it.

Client Stage 1.4 (A7 part 1, 2026-10-27). Read in ArduPilot's 4.7.0 source
(GCS_Common.cpp ``send_proximity``, AP_Proximity_Boundary_3D.cpp):

- One DISTANCE_SENSOR for each of 8 directions round the drone: ids 10 to 17,
  orientation 0 to 7 (ahead, then every 45 degrees clockwise), and one upward
  (id 19, orientation 24). Each carries the sensor's own range (min and max).
- Only while the sensor works (status Good).
- A direction only once it has had a reading since boot. A direction with no
  reading now comes as the sensor's maximum distance (0 for a sensor whose
  range ArduPilot does not know, such as the RPLidar A3).
- No direction at all while nothing is in range anywhere.

So silence is not a broken sensor. SYS_STATUS's proximity health bit says
whether it works (vgcs/app/obstacle_watch.py). Other DISTANCE_SENSOR ids (0 to
9) are rangefinders: they stay as they were.

The 8 directions come in one burst, and the link passes a DISTANCE_SENSOR to
the window at most every 0.15 s. Until 2026-10-27 that let one direction of
each burst through (mostly the one ahead), so they are kept here and sent on
together.

Pure Python, no Qt and no pymavlink, so it is tested without a link.
"""

from __future__ import annotations

import math

PROXIMITY_ID_START = 10
DIRECTIONS = 8
UP_ID = PROXIMITY_ID_START + DIRECTIONS + 1        # 19
ORIENTATION_UP = 24                                # MAV_SENSOR_ROTATION_PITCH_90
_NO_READING_CM = 0xFFFF
# A direction not heard for this long is no reading: the drone sends them several times a second.
DIRECTION_MAX_AGE_S = 2.0
# The drone's target is this far from the waypoint it flies to: it goes around something.
DETOUR_MIN_M = 3.0

_M_PER_DEG = 111_320.0


def is_proximity_id(sensor_id: object) -> bool:
    """A DISTANCE_SENSOR of the proximity sensor (a direction or upward), not a rangefinder."""
    try:
        number = int(sensor_id)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return False
    return PROXIMITY_ID_START <= number < PROXIMITY_ID_START + DIRECTIONS or number == UP_ID


def _reading_m(current_cm: object, max_cm: object) -> float | None:
    """The distance in metres, or None for "nothing in range" (no reading, or the sensor's maximum)."""
    try:
        current = int(current_cm)  # type: ignore[arg-type]
        top = int(max_cm)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None
    if current <= 0 or current >= _NO_READING_CM:
        return None
    if top > 0 and current >= top:
        return None
    return current / 100.0


class ProximitySectors:
    """The last reading in each of the 8 directions and upward, as the drone sent them."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        # Each: (distance in metres or None for nothing in range, when it was heard).
        self._directions: list[tuple[float | None, float] | None] = [None] * DIRECTIONS
        self._up: tuple[float | None, float] | None = None
        self.min_m = 0.0
        self.max_m = 0.0
        self.last_heard: float | None = None

    def hear(self, sensor_id: object, orientation: object, current_cm: object, min_cm: object, max_cm: object,
             now: float) -> bool:
        """One DISTANCE_SENSOR. True when it was the proximity sensor's (so it is not a rangefinder's)."""
        if not is_proximity_id(sensor_id):
            return False
        reading = (_reading_m(current_cm, max_cm), float(now))
        try:
            self.min_m = max(0.0, int(min_cm) / 100.0)  # type: ignore[arg-type]
            self.max_m = max(0.0, int(max_cm) / 100.0)  # type: ignore[arg-type]
        except (TypeError, ValueError):
            pass
        self.last_heard = float(now)
        number = int(sensor_id)  # type: ignore[arg-type]
        if number == UP_ID:
            self._up = reading
            return True
        try:
            index = int(orientation)  # type: ignore[arg-type]
        except (TypeError, ValueError):
            index = -1
        if not 0 <= index < DIRECTIONS:
            index = number - PROXIMITY_ID_START
        self._directions[index] = reading
        return True

    def snapshot(self, now: float) -> dict:
        """What the window gets: per direction its distance (None: nothing in range, or not heard lately)."""
        def fresh(item):
            return item is not None and now - item[1] <= DIRECTION_MAX_AGE_S

        return {
            "directions_m": [item[0] if fresh(item) else None for item in self._directions],
            "heard": [fresh(item) for item in self._directions],
            "up_m": self._up[0] if fresh(self._up) else None,
            "min_m": self.min_m,
            "max_m": self.max_m,
        }


def horizontal_distance_m(a: tuple[float, float], b: tuple[float, float]) -> float:
    """Metres between two positions, on a flat map of their own area (as the fence code does)."""
    east = (float(b[1]) - float(a[1])) * _M_PER_DEG * math.cos(math.radians(float(a[0])))
    north = (float(b[0]) - float(a[0])) * _M_PER_DEG
    return math.hypot(east, north)


def mission_target(target: tuple[float, float], waypoint: tuple[float, float] | None,
                   next_waypoint: tuple[float, float] | None = None) -> dict:
    """The drone's target in a mission, beside the waypoint it flies to.

    ArduCopter sends the path planner's point (OA_TYPE) as its target during a
    detour, and the waypoint itself otherwise (4.7.0 source,
    ``AC_WPNav::get_oa_wp_destination``). So a target away from the waypoint is
    a detour. Without the waypoint (a mission VGCS does not know, or an item
    that is not a waypoint) nothing is said about it.

    A target on the next waypoint is no detour either: the drone has reached
    this one, and its MISSION_CURRENT has not come yet (ArduPilot sends it with
    the stream, not at once, when a waypoint is reached).
    """
    out = {"lat": float(target[0]), "lon": float(target[1]), "detour": None, "off_m": None}
    if waypoint is None:
        return out
    off = horizontal_distance_m(waypoint, target)
    out["off_m"] = off
    out["detour"] = off > DETOUR_MIN_M and (
        next_waypoint is None or horizontal_distance_m(next_waypoint, target) > DETOUR_MIN_M)
    return out
