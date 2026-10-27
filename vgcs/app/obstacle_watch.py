"""What the drone's obstacle sensor (a LiDAR) sees, and what VGCS says about it.

Client Stage 1.4 (A7 part 1, 2026-10-27). The drone avoids by itself: ArduPilot
on the flight controller uses the sensor, and the ground station cannot make it
avoid. VGCS shows what the sensor sees and says what the drone does about it.

Inputs, all from the drone:

- the 8 directions and upward, gathered by the link (vgcs/link/proximity.py);
- SYS_STATUS's proximity bits: enabled (a sensor is set up, PRXn_TYPE) and
  healthy (it works). A healthy sensor that sends nothing has nothing in range;
- the flight mode, armed or not, and the avoidance settings;
- in a mission, the drone's target beside the waypoint it flies to (a detour).

What ArduCopter 4.7.0 does (its source, read 2026-10-27):

- Loiter, AltHold, FlowHold and ZigZag keep away from what the sensor sees when
  AVOID_ENABLE has its "proximity" bit (2). Loiter stops (AVOID_BEHAVE 1) or
  slides along the object (0), AVOID_MARGIN from it.
- PosHold does not (its loiter runs with avoidance off), nor Stabilize, Acro,
  Sport, Drift and the other modes.
- Auto, Guided, RTL and SmartRTL fly around an object only with the path
  planner on (OA_TYPE 1 or 3, BendyRuler; 2, Dijkstra, plans around fences
  only). They steer by waypoint navigation, which is the planner's.
- The drone sends no message when it stops or goes around. Only a refused
  arming has one ("PreArm: Proximity 45 deg, 0.42m (want > 0.6m)"), and VGCS
  shows the drone's messages already.

Pure: no Qt and no link code, so every rule here is unit tested.
"""

from __future__ import annotations

import math
import time

from vgcs.link.proximity import DIRECTION_MAX_AGE_S, DIRECTIONS

# SYS_STATUS sensor bit of the proximity sensor (MAV_SYS_STATUS_SENSOR_PROXIMITY).
SENSOR_PROXIMITY = 0x4000000
# The drone's settings this module reads (vgcs/app/vehicle_params.py reads them on connect).
OBSTACLE_PARAMS = ("AVOID_ENABLE", "AVOID_MARGIN", "AVOID_BEHAVE", "OA_TYPE")
# AVOID_ENABLE bit "use the proximity sensor".
AVOID_PROXIMITY_BIT = 2
# OA_TYPE values with BendyRuler, the path planner that goes around what a sensor sees.
OA_AROUND_OBJECTS = (1, 3)
# ArduPilot's own AVOID_MARGIN, while the drone's is not read yet.
DEFAULT_MARGIN_M = 2.0
# Warn this far beyond the stop distance (Loiter stops at it, so nearer than it is seldom seen),
# and only say "clear" again this much further out, so a reading on the line does not warn twice.
NEAR_EXTRA_M = 2.0
REARM_EXTRA_M = 1.0
MIN_WARNING_GAP_S = 5.0
REPEAT_WARNING_S = 30.0
# The drone says the sensor does not work for this long before VGCS says so (it flickers at start).
SENSOR_FAULT_AFTER_S = 3.0
# SYS_STATUS comes once or twice a second. Its sensor bits are not used when older than this (a lost
# link), so a last "it works" is not shown as "Clear" while nothing is heard.
FLAGS_MAX_AGE_S = 5.0
# The drone's target comes at 2 Hz in Auto, Guided and RTL.
TARGET_MAX_AGE_S = 3.0
DETOUR_EVENT_GAP_S = 10.0
# Radar and map scales, in metres: the first that holds the farthest object.
_SCALES_M = (10.0, 20.0, 30.0, 50.0, 100.0, 200.0)

STOP_MODES = frozenset({"LOITER", "ALT_HOLD", "FLOWHOLD", "ZIGZAG"})
PLANNER_MODES = frozenset({"AUTO", "GUIDED", "RTL", "AUTO_RTL", "SMART_RTL"})
_MODE_NAMES = {
    "LOITER": "Loiter", "ALT_HOLD": "AltHold", "FLOWHOLD": "FlowHold", "ZIGZAG": "ZigZag",
    "AUTO": "Auto", "GUIDED": "Guided", "RTL": "RTL", "AUTO_RTL": "Auto RTL", "POSHOLD": "PosHold",
    "STABILIZE": "Stabilize", "ACRO": "Acro", "SPORT": "Sport", "DRIFT": "Drift", "LAND": "Land",
    "BRAKE": "Brake", "SMART_RTL": "SmartRTL", "CIRCLE": "Circle", "FOLLOW": "Follow",
    "GUIDED_NOGPS": "Guided NoGPS", "AUTOTUNE": "AutoTune", "THROW": "Throw",
}

# Direction words of the 8 directions: 0 is ahead, then every 45 degrees clockwise.
DIRECTION_WORDS = ("ahead", "ahead right", "right", "behind right", "behind", "behind left", "left",
                   "ahead left")

STATE_NONE = "none"        # no proximity sensor known on this drone
STATE_OK = "ok"
STATE_FAULT = "fault"      # set up, and the drone says it does not work

FAULT_WORDS = "Obstacle sensor: no readings. The drone says its proximity sensor does not work."
WORKS_AGAIN_WORDS = "Obstacle sensor works again."


def _number(value: object) -> float | None:
    try:
        number = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def metres_words(distance_m: float) -> str:
    """"2.8 m" up to 10 m, "14 m" beyond."""
    return f"{distance_m:.1f} m" if distance_m < 10.0 else f"{distance_m:.0f} m"


def mode_key(mode: object) -> str:
    """A flight mode as VGCS keeps it: "LOITER", "ALT_HOLD", "AUTO_RTL"."""
    return str(mode or "").strip().upper().replace(" ", "_")


def mode_name(mode: object) -> str:
    key = mode_key(mode)
    return _MODE_NAMES.get(key, key.replace("_", " ").title())


def mode_effect(mode: object, params: dict) -> tuple[str, bool | None]:
    """What a flight mode does about an object, in one sentence, and whether it keeps away from it.

    ``params``: the drone's AVOID_ENABLE, AVOID_MARGIN, AVOID_BEHAVE and OA_TYPE, any of them missing
    while not read. The second value is None while that is not known (the mode, or OA_TYPE).
    Used for the warnings here and for the mode switch check of the settings page
    (vgcs/app/obstacle_setup.py).
    """
    key = mode_key(mode)
    if not key:
        return "The drone's mode is not known.", None
    name = mode_name(key)
    enable = params.get("AVOID_ENABLE")
    margin = params.get("AVOID_MARGIN")
    behave = params.get("AVOID_BEHAVE")
    oa = params.get("OA_TYPE")
    if key in STOP_MODES:
        if enable is not None and not int(enable) & AVOID_PROXIMITY_BIT:
            return f"{name} does not keep away from it: AVOID_ENABLE does not use the sensor.", False
        if key == "LOITER" and behave is not None:
            away = "" if margin is None else f" {margin:g} m"
            if int(behave) == 1:
                return (f"Loiter stops{away} from it." if away else "Loiter stops before it."), True
            return f"Loiter slides along it{',' + away + ' away' if away else ''}.", True
        return f"{name} keeps away from it.", True
    if key in PLANNER_MODES:
        if oa is None:
            return f"{name} flies around it only with the path planner on (OA_TYPE not read yet).", None
        if int(oa) in OA_AROUND_OBJECTS:
            return f"{name} flies around it (path planner on, OA_TYPE {int(oa)}).", True
        return f"{name} does not fly around it: the path planner is off (OA_TYPE {int(oa)}).", False
    return f"{name} does not avoid it.", False


def scale_m(directions_m: list, stop_m: float | None = None) -> float:
    """The radar's and the map's reach: the first of 10, 20, 30, 50, 100, 200 m that holds the farthest object."""
    farthest = max([d for d in directions_m if d is not None] + [2.0 * (stop_m or 0.0)], default=0.0)
    return next((s for s in _SCALES_M if s >= farthest), _SCALES_M[-1])


class ObstacleWatch:
    """Hold the obstacle sensor's readings and the drone's state, and say what to show and to warn."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        """Forget everything (link closed, another drone on screen)."""
        self._snapshot: dict | None = None
        self._snapshot_mono = 0.0
        self._enabled: bool | None = None
        self._healthy: bool | None = None
        self._flags_mono: float | None = None
        self._unhealthy_since: float | None = None
        self._fault_said = False
        self._mode = ""
        self._armed = False
        self._params: dict[str, float] = {}
        self._near = False
        self._near_mode = ""
        self._last_warning_mono: float | None = None
        self._target: dict | None = None
        self._target_mono = 0.0
        self._detour_said = False
        self._last_detour_event_mono: float | None = None

    # -- ingest ---------------------------------------------------------
    def update_sectors(self, payload: object, now: float | None = None) -> None:
        """The link's PROXIMITY snapshot (vgcs/link/proximity.py)."""
        if not isinstance(payload, dict):
            return
        directions = list(payload.get("directions_m") or [])[:DIRECTIONS]
        directions += [None] * (DIRECTIONS - len(directions))
        self._snapshot = {
            "directions_m": [_number(d) if d is not None else None for d in directions],
            "up_m": _number(payload.get("up_m")) if payload.get("up_m") is not None else None,
            "max_m": _number(payload.get("max_m")) or 0.0,
        }
        self._snapshot_mono = time.monotonic() if now is None else float(now)

    def update_sensor_flags(self, present: object, enabled: object, health: object, now: float | None = None) -> None:
        """SYS_STATUS's sensor bits. ``present`` is not used: ArduPilot sets it only once the sensor answered.

        ArduCopter 4.7.0 (GCS_Copter.cpp): enabled while a PRXn_TYPE is set, healthy while every
        sensor's status is Good (and while none is set up).
        """
        del present
        now_f = time.monotonic() if now is None else float(now)
        try:
            self._enabled = bool(int(enabled) & SENSOR_PROXIMITY)  # type: ignore[arg-type]
            healthy = bool(int(health) & SENSOR_PROXIMITY)  # type: ignore[arg-type]
        except (TypeError, ValueError):
            return
        self._flags_mono = now_f
        if healthy or not self._enabled:
            self._unhealthy_since = None
        elif self._unhealthy_since is None:
            self._unhealthy_since = now_f
        self._healthy = healthy

    def update_mode(self, mode: object, armed: bool) -> None:
        self._mode = mode_key(mode)
        self._armed = bool(armed)

    def update_params(self, params: object) -> None:
        """The drone's settings as VGCS holds them (any case of name). Only the avoidance ones are kept."""
        if not isinstance(params, dict):
            return
        for name, value in params.items():
            key = str(name).strip().upper()
            if key in OBSTACLE_PARAMS:
                number = _number(value)
                if number is not None:
                    self._params[key] = number

    def update_target(self, payload: object, now: float | None = None) -> None:
        """The link's MISSION_TARGET: where the drone flies now, and whether that is a detour."""
        if isinstance(payload, dict):
            self._target = dict(payload)
            self._target_mono = time.monotonic() if now is None else float(now)

    # -- what the sensor sees --------------------------------------------
    def _fresh(self, now_f: float) -> dict | None:
        if self._snapshot is None or now_f - self._snapshot_mono > DIRECTION_MAX_AGE_S:
            return None
        return self._snapshot

    def directions_m(self, now: float | None = None) -> list[float | None]:
        """The distance in each of the 8 directions (None: nothing in range, or nothing heard lately)."""
        snap = self._fresh(time.monotonic() if now is None else float(now))
        return list(snap["directions_m"]) if snap is not None else [None] * DIRECTIONS

    def up_m(self, now: float | None = None) -> float | None:
        snap = self._fresh(time.monotonic() if now is None else float(now))
        return None if snap is None else snap["up_m"]

    def nearest(self, now: float | None = None) -> tuple[int, float] | None:
        """(direction, distance) of the nearest object round the drone, or None."""
        found = [(d, i) for i, d in enumerate(self.directions_m(now)) if d is not None]
        if not found:
            return None
        distance, index = min(found)
        return index, distance

    def stop_m(self) -> float | None:
        """AVOID_MARGIN as the drone holds it, or None while it is not read."""
        return self._params.get("AVOID_MARGIN")

    def near_m(self) -> float:
        """Nearer than this, VGCS warns: the stop distance and NEAR_EXTRA_M."""
        margin = self.stop_m()
        return (DEFAULT_MARGIN_M if margin is None else margin) + NEAR_EXTRA_M

    def sensor_state(self, now: float | None = None) -> str:
        """STATE_OK, STATE_FAULT, or STATE_NONE (no sensor known, or nothing heard lately)."""
        now_f = time.monotonic() if now is None else float(now)
        flags_fresh = self._flags_mono is not None and now_f - self._flags_mono <= FLAGS_MAX_AGE_S
        enabled = bool(self._enabled) and flags_fresh
        if enabled and not self._healthy and self._unhealthy_since is not None:
            if now_f - self._unhealthy_since >= SENSOR_FAULT_AFTER_S:
                return STATE_FAULT
            return STATE_OK
        if enabled or self._fresh(now_f) is not None:
            return STATE_OK
        return STATE_NONE

    def field_text(self, now: float | None = None) -> str:
        """Text for the dashboard's obstacle field."""
        now_f = time.monotonic() if now is None else float(now)
        state = self.sensor_state(now_f)
        if state == STATE_NONE:
            return "N/A"
        if state == STATE_FAULT:
            return "Not working"
        near = self.nearest(now_f)
        if near is None:
            return "Clear"
        return f"{metres_words(near[1])} {DIRECTION_WORDS[near[0]]}"

    def level(self, now: float | None = None) -> str:
        now_f = time.monotonic() if now is None else float(now)
        state = self.sensor_state(now_f)
        if state == STATE_NONE:
            return "na"
        near = self.nearest(now_f)
        if state == STATE_FAULT or (near is not None and near[1] <= self.near_m()):
            return "warn"
        return "ok"

    def radar_view(self, now: float | None = None) -> dict:
        """Everything the radar card and the map draw, and their words."""
        now_f = time.monotonic() if now is None else float(now)
        directions = self.directions_m(now_f)
        near = self.nearest(now_f)
        stop = self.stop_m()
        state = self.sensor_state(now_f)
        seen = sum(1 for d in directions if d is not None)
        if state == STATE_FAULT:
            status = "Sensor not working"
        elif state == STATE_NONE:
            status = "No obstacle sensor"
        elif seen:
            status = f"LiDAR · {seen} of 8 directions"
        else:
            status = "Path clear"
        return {
            "directions_m": directions,
            "up_m": self.up_m(now_f),
            "nearest": near,
            "nearest_text": ("Clear" if near is None else metres_words(near[1])),
            "nearest_where": ("" if near is None else DIRECTION_WORDS[near[0]].capitalize()),
            "near": near is not None and near[1] <= self.near_m(),
            "near_m": self.near_m(),
            # Red within the stop distance (ArduPilot's own 2 m while the drone's is not read).
            "red_m": DEFAULT_MARGIN_M if stop is None else stop,
            "stop_m": stop,
            "scale_m": scale_m(directions, stop),
            "state": state,
            "status": status,
        }

    # -- what the drone does about it -------------------------------------
    def mode_words(self) -> str:
        """What the drone in its mode does about an object, in one sentence."""
        return mode_effect(self._mode, self._params)[0]

    def take_warning(self, now: float | None = None) -> str | None:
        """A warning when an armed drone has just come near an object, else None.

        Near is NEAR_EXTRA_M beyond the stop distance. Again when the mode
        changes while near (its words change), and a reminder every
        REPEAT_WARNING_S while it stays near. Nothing on the ground: people
        walk round a drone before take-off.
        """
        now_f = time.monotonic() if now is None else float(now)
        if not self._armed:
            self._near = False
            return None
        near = self.nearest(now_f)
        limit = self.near_m()
        if near is None or near[1] > limit + REARM_EXTRA_M:
            self._near = False
            return None
        if near[1] > limit:
            return None
        new = not self._near or self._mode != self._near_mode
        wait = MIN_WARNING_GAP_S if new else REPEAT_WARNING_S
        if self._last_warning_mono is not None and now_f - self._last_warning_mono < wait:
            return None
        self._near = True
        self._near_mode = self._mode
        self._last_warning_mono = now_f
        return f"Object {metres_words(near[1])} {DIRECTION_WORDS[near[0]]}. {self.mode_words()}"

    def take_sensor_event(self, now: float | None = None) -> tuple[str, bool] | None:
        """(words, is a warning) when the sensor has just stopped working, or works again.

        "Works again" only once the drone says so: nothing heard (a lost link) is not working.
        """
        state = self.sensor_state(now)
        if state == STATE_FAULT and not self._fault_said:
            self._fault_said = True
            return FAULT_WORDS, True
        if state == STATE_OK and self._fault_said:
            self._fault_said = False
            return WORKS_AGAIN_WORDS, False
        return None

    def detour(self, now: float | None = None) -> dict | None:
        """In a mission, the point the drone flies to while it goes around something, else None."""
        now_f = time.monotonic() if now is None else float(now)
        target = self._target
        if target is None or now_f - self._target_mono > TARGET_MAX_AGE_S:
            return None
        if mode_key(target.get("mode")) != "AUTO" or self._mode != "AUTO" or not target.get("detour"):
            return None
        lat, lon = _number(target.get("lat")), _number(target.get("lon"))
        if lat is None or lon is None:
            return None
        return {"lat": lat, "lon": lon, "wp_index": target.get("wp_index"), "off_m": target.get("off_m")}

    def take_detour_event(self, now: float | None = None) -> str | None:
        """Words once a detour starts ("Going around an obstacle on the way to waypoint 3"), else None."""
        now_f = time.monotonic() if now is None else float(now)
        point = self.detour(now_f)
        if point is None:
            self._detour_said = False
            return None
        if self._detour_said:
            return None
        if self._last_detour_event_mono is not None and now_f - self._last_detour_event_mono < DETOUR_EVENT_GAP_S:
            return None
        self._detour_said = True
        self._last_detour_event_mono = now_f
        wp = point.get("wp_index")
        return "Going around an obstacle" + ("" if wp is None else f" on the way to waypoint {int(wp) + 1}")
