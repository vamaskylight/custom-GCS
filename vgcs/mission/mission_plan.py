"""Mission plan <-> MAVLink mission item translation.

This is the pure, side-effect-free core of waypoint navigation. The MAVLink
worker (:mod:`vgcs.link.mavlink_thread`) owns the wire protocol; everything about
*what* a mission contains lives here so it can be unit tested without a link.

ArduPilot / AP_Mission semantics this module encodes
---------------------------------------------------
* MAVLink mission index 0 is the **home slot**. ArduCopter overwrites whatever is
  uploaded there with the vehicle's own home position, so the uploaded seq 0 item
  is a placeholder only.
* The first *stored* command is seq 1 (``AP_MISSION_FIRST_REAL_COMMAND``). AUTO
  from the ground needs ``NAV_TAKEOFF`` there or the vehicle reports
  "Missing Takeoff Cmd" and refuses to run the mission.
* ``DO_CHANGE_SPEED`` is a non-nav command that executes as it is passed. It is
  emitted only when the commanded speed actually changes, which keeps the mission
  short and makes a download round-trip reconstruct the same per-waypoint speeds.
* Because of the home slot, the takeoff item and the interleaved speed items, the
  MAVLink ``seq`` a vehicle reports in ``MISSION_CURRENT`` /
  ``MISSION_ITEM_REACHED`` is **not** the operator's waypoint number.
  :func:`MissionPlan.waypoint_index_for_seq` is the only supported mapping.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from pymavlink import mavutil

from vgcs.mission.waypoint_store import Waypoint

__all__ = [
    "MISSION_END_ACTIONS",
    "DEFAULT_MISSION_END_ACTION",
    "MissionItem",
    "MissionPlan",
    "build_mission_plan",
    "parse_downloaded_mission",
    "validate_waypoints",
    "normalize_end_action",
    "haversine_m",
]

_m = mavutil.mavlink

# Operator-selectable behaviour after the final waypoint.
#   hold — append nothing; ArduCopter holds position at the last waypoint (legacy behaviour)
#   rtl  — append NAV_RETURN_TO_LAUNCH: climb to RTL_ALT, fly home, land per RTL_ALT_FINAL
#   land — append NAV_LAND at the final waypoint's position
MISSION_END_ACTIONS: tuple[str, ...] = ("hold", "rtl", "land")
DEFAULT_MISSION_END_ACTION = "rtl"

# Plan sanity limits. These gate the *upload*, they are not vehicle parameters —
# ArduPilot enforces its own fence/altitude limits independently.
MIN_WP_ALT_M = 1.0
MAX_WP_ALT_M = 500.0
MIN_WP_SPEED_MPS = 0.1
MAX_WP_SPEED_MPS = 30.0
# Two waypoints closer than this are almost always a double-click, and ArduCopter
# can behave oddly on zero-length legs.
MIN_LEG_LENGTH_M = 1.0
# Warn (do not block) when the plan reaches further than this from waypoint 1.
FAR_FROM_START_WARN_M = 5000.0


@dataclass
class MissionItem:
    """One MAVLink mission item, addressed by its upload ``seq``."""

    seq: int
    command: int
    lat: float = 0.0
    lon: float = 0.0
    alt_m: float = 0.0
    p1: float = 0.0
    p2: float = 0.0
    p3: float = 0.0
    p4: float = 0.0
    frame: int = int(_m.MAV_FRAME_GLOBAL_RELATIVE_ALT)
    # 0-based index into the operator's waypoint list, or None for the home slot,
    # takeoff, speed changes and the terminal item.
    wp_index: int | None = None
    # Human label used in logs and the mission progress readout.
    label: str = ""


@dataclass
class MissionPlan:
    """A built mission: the items to upload plus the seq -> waypoint mapping."""

    items: list[MissionItem] = field(default_factory=list)
    waypoint_count: int = 0
    end_action: str = DEFAULT_MISSION_END_ACTION
    takeoff_alt_m: float = 0.0

    def __len__(self) -> int:
        return len(self.items)

    def waypoint_index_for_seq(self, seq: int) -> int | None:
        """0-based operator waypoint index for a MAVLink ``seq``, else ``None``.

        ``None`` means the vehicle is on a non-waypoint item (home slot, takeoff,
        a speed change or the terminal RTL/LAND).
        """
        try:
            s = int(seq)
        except (TypeError, ValueError):
            return None
        for item in self.items:
            if item.seq == s:
                return item.wp_index
        return None

    def seq_for_waypoint_index(self, wp_index: int) -> int | None:
        """MAVLink ``seq`` of the nav item for a 0-based waypoint index."""
        try:
            idx = int(wp_index)
        except (TypeError, ValueError):
            return None
        for item in self.items:
            if item.wp_index == idx and item.command == int(_m.MAV_CMD_NAV_WAYPOINT):
                return item.seq
        return None

    def label_for_seq(self, seq: int) -> str:
        try:
            s = int(seq)
        except (TypeError, ValueError):
            return ""
        for item in self.items:
            if item.seq == s:
                return item.label
        return ""

    def seq_map(self) -> dict[int, int | None]:
        """``{seq: wp_index or None}`` — cheap to hand across a Qt signal."""
        return {item.seq: item.wp_index for item in self.items}

    def labels(self) -> dict[int, str]:
        return {item.seq: item.label for item in self.items}


def normalize_end_action(value: object) -> str:
    """Coerce stored/settings values to a supported end action."""
    text = str(value or "").strip().lower()
    if text in MISSION_END_ACTIONS:
        return text
    # Tolerate legacy / alternate spellings seen in saved plans.
    if text in ("return", "return_to_launch", "returntolaunch"):
        return "rtl"
    if text in ("landing", "nav_land"):
        return "land"
    if text in ("none", "loiter", "hover", "stay"):
        return "hold"
    return DEFAULT_MISSION_END_ACTION


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in metres."""
    r = 6371000.0
    p1 = math.radians(float(lat1))
    p2 = math.radians(float(lat2))
    dp = p2 - p1
    dl = math.radians(float(lon2) - float(lon1))
    a = math.sin(dp / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
    return 2.0 * r * math.asin(min(1.0, math.sqrt(a)))


def _wp_field(wp: object, name: str, default: float) -> float:
    """Read a waypoint field from either a Waypoint or a plain dict."""
    if isinstance(wp, dict):
        raw = wp.get(name, default)
    else:
        raw = getattr(wp, name, default)
    if raw is None:
        return float(default)
    try:
        return float(raw)
    except (TypeError, ValueError):
        return float(default)


def validate_waypoints(
    waypoints: list[object],
    *,
    home: tuple[float, float] | None = None,
    max_alt_m: float = MAX_WP_ALT_M,
) -> tuple[list[str], list[str]]:
    """Pre-upload sanity check.

    Returns ``(errors, warnings)``. Errors mean the plan must not be uploaded;
    warnings are shown to the operator but may be accepted.
    """
    errors: list[str] = []
    warnings: list[str] = []

    if not waypoints:
        return (["Mission has no waypoints."], [])

    prev: tuple[float, float] | None = None
    first: tuple[float, float] | None = None
    for i, wp in enumerate(waypoints):
        n = i + 1
        lat = _wp_field(wp, "lat", 0.0)
        lon = _wp_field(wp, "lon", 0.0)
        alt = _wp_field(wp, "alt_m", 20.0)
        spd = _wp_field(wp, "speed_mps", 5.0)

        if not (-90.0 <= lat <= 90.0) or not (-180.0 <= lon <= 180.0):
            errors.append(f"WP {n}: latitude/longitude out of range ({lat:.6f}, {lon:.6f}).")
            continue
        if abs(lat) < 1e-7 and abs(lon) < 1e-7:
            errors.append(f"WP {n}: position is 0,0 — this is not a real waypoint.")
            continue
        if not math.isfinite(alt):
            errors.append(f"WP {n}: altitude is not a number.")
            continue
        if alt < MIN_WP_ALT_M:
            errors.append(f"WP {n}: altitude {alt:.1f} m is below the {MIN_WP_ALT_M:.0f} m minimum.")
        elif alt > float(max_alt_m):
            warnings.append(f"WP {n}: altitude {alt:.0f} m exceeds the {float(max_alt_m):.0f} m plan limit.")
        if spd < MIN_WP_SPEED_MPS:
            errors.append(f"WP {n}: speed {spd:.2f} m/s is below the {MIN_WP_SPEED_MPS:.1f} m/s minimum.")
        elif spd > MAX_WP_SPEED_MPS:
            warnings.append(f"WP {n}: speed {spd:.1f} m/s is unusually high.")

        if first is None:
            first = (lat, lon)
        if prev is not None:
            leg = haversine_m(prev[0], prev[1], lat, lon)
            if leg < MIN_LEG_LENGTH_M:
                warnings.append(f"WP {n}: only {leg:.1f} m from WP {n - 1} (duplicate waypoint?).")
        if first is not None:
            span = haversine_m(first[0], first[1], lat, lon)
            if span > FAR_FROM_START_WARN_M:
                warnings.append(f"WP {n}: {span / 1000.0:.1f} km from WP 1 — confirm this is intended.")
        prev = (lat, lon)

    if home is not None and first is not None:
        d = haversine_m(home[0], home[1], first[0], first[1])
        if d > FAR_FROM_START_WARN_M:
            warnings.append(
                f"WP 1 is {d / 1000.0:.1f} km from the vehicle — the takeoff leg will be long."
            )

    return (errors, warnings)


def build_mission_plan(
    waypoints: list[object],
    *,
    takeoff_alt_m: float | None = None,
    end_action: str = DEFAULT_MISSION_END_ACTION,
    default_speed_mps: float = 5.0,
) -> MissionPlan:
    """Translate operator waypoints into the MAVLink mission item list.

    Layout produced (``seq``: item):

    ``0``: NAV_WAYPOINT placeholder — ArduCopter replaces it with home.
    ``1``: NAV_TAKEOFF to ``takeoff_alt_m`` (defaults to WP 1's altitude).
    then, per waypoint: an optional DO_CHANGE_SPEED (only when the speed changes)
    followed by the NAV_WAYPOINT itself.
    finally, per ``end_action``: nothing (hold), NAV_RETURN_TO_LAUNCH or NAV_LAND.
    """
    action = normalize_end_action(end_action)
    plan = MissionPlan(items=[], waypoint_count=len(waypoints), end_action=action)
    if not waypoints:
        return plan

    first = waypoints[0]
    first_alt = max(MIN_WP_ALT_M, _wp_field(first, "alt_m", 20.0))
    if takeoff_alt_m is None:
        climb_m = first_alt
    else:
        climb_m = max(MIN_WP_ALT_M, float(takeoff_alt_m))
    plan.takeoff_alt_m = climb_m

    seq = 0
    # seq 0 — home slot placeholder. Carries WP1's position so that a vehicle which
    # does *not* rewrite seq 0 still ends up with a sane first target.
    plan.items.append(
        MissionItem(
            seq=seq,
            command=int(_m.MAV_CMD_NAV_WAYPOINT),
            lat=_wp_field(first, "lat", 0.0),
            lon=_wp_field(first, "lon", 0.0),
            alt_m=first_alt,
            label="Home slot",
        )
    )
    seq += 1

    plan.items.append(
        MissionItem(
            seq=seq,
            command=int(_m.MAV_CMD_NAV_TAKEOFF),
            alt_m=climb_m,
            label=f"Takeoff {climb_m:.0f} m",
        )
    )
    seq += 1

    commanded_speed: float | None = None
    for idx, wp in enumerate(waypoints):
        spd = max(MIN_WP_SPEED_MPS, _wp_field(wp, "speed_mps", default_speed_mps))
        # Only emit a speed change when it actually changes — a redundant
        # DO_CHANGE_SPEED before every waypoint bloats the mission and shifts every
        # downstream seq for no benefit.
        if commanded_speed is None or abs(spd - commanded_speed) > 1e-6:
            plan.items.append(
                MissionItem(
                    seq=seq,
                    command=int(_m.MAV_CMD_DO_CHANGE_SPEED),
                    # p1: 1 = groundspeed, p2: speed m/s, p3: throttle (-1 = no change)
                    p1=1.0,
                    p2=spd,
                    p3=-1.0,
                    label=f"Speed {spd:.1f} m/s",
                )
            )
            seq += 1
            commanded_speed = spd

        plan.items.append(
            MissionItem(
                seq=seq,
                command=int(_m.MAV_CMD_NAV_WAYPOINT),
                lat=_wp_field(wp, "lat", 0.0),
                lon=_wp_field(wp, "lon", 0.0),
                alt_m=max(MIN_WP_ALT_M, _wp_field(wp, "alt_m", 20.0)),
                wp_index=idx,
                label=f"WP {idx + 1}",
            )
        )
        seq += 1

    if action == "rtl":
        plan.items.append(
            MissionItem(
                seq=seq,
                command=int(_m.MAV_CMD_NAV_RETURN_TO_LAUNCH),
                label="Return to launch",
            )
        )
    elif action == "land":
        last = waypoints[-1]
        plan.items.append(
            MissionItem(
                seq=seq,
                command=int(_m.MAV_CMD_NAV_LAND),
                lat=_wp_field(last, "lat", 0.0),
                lon=_wp_field(last, "lon", 0.0),
                alt_m=0.0,
                label="Land",
            )
        )
    return plan


# Commands that carry an operator-visible position. Everything else downloaded
# from the vehicle (takeoff, speed changes, ROI, jumps, RTL/land) is mission
# plumbing and must not be shown as a waypoint on the map.
_NAV_WAYPOINT_COMMANDS = frozenset(
    {
        int(_m.MAV_CMD_NAV_WAYPOINT),
        int(_m.MAV_CMD_NAV_SPLINE_WAYPOINT),
        int(_m.MAV_CMD_NAV_LOITER_UNLIM),
        int(_m.MAV_CMD_NAV_LOITER_TURNS),
        int(_m.MAV_CMD_NAV_LOITER_TIME),
    }
)


def parse_downloaded_mission(
    rows: list[object],
    *,
    default_speed_mps: float = 5.0,
) -> tuple[list[Waypoint], str]:
    """Rebuild operator waypoints from downloaded mission items.

    Returns ``(waypoints, end_action)``.

    Filters out everything that is not a positioned nav command, replays
    ``DO_CHANGE_SPEED`` onto the waypoints that follow it, and recognises a
    trailing RTL/LAND so the plan's end action survives a download.

    Without this, a mission downloaded straight back from the vehicle contains the
    home slot, the takeoff item and every speed change — items whose lat/lon are
    ``0,0`` — and the map fills with waypoints in the Gulf of Guinea.
    """
    waypoints: list[Waypoint] = []
    end_action = "hold"
    speed = float(default_speed_mps)

    for row in rows:
        if not isinstance(row, dict):
            continue
        try:
            cmd = int(row.get("command", _m.MAV_CMD_NAV_WAYPOINT) or 0)
        except (TypeError, ValueError):
            continue
        seq = row.get("seq", None)
        # seq 0 is the vehicle's home position, never an operator waypoint.
        try:
            if seq is not None and int(seq) == 0:
                continue
        except (TypeError, ValueError):
            pass

        if cmd == int(_m.MAV_CMD_DO_CHANGE_SPEED):
            try:
                new_speed = float(row.get("p2", row.get("param2", 0.0)) or 0.0)
            except (TypeError, ValueError):
                new_speed = 0.0
            # param2 <= 0 means "no change" in the MAVLink spec.
            if new_speed > 0.0:
                speed = new_speed
            continue
        if cmd == int(_m.MAV_CMD_NAV_RETURN_TO_LAUNCH):
            end_action = "rtl"
            continue
        if cmd == int(_m.MAV_CMD_NAV_LAND):
            end_action = "land"
            continue
        if cmd not in _NAV_WAYPOINT_COMMANDS:
            continue

        lat = float(row.get("lat", 0.0) or 0.0)
        lon = float(row.get("lon", 0.0) or 0.0)
        # A positioned nav command at exactly 0,0 is a placeholder, not a location.
        if abs(lat) < 1e-9 and abs(lon) < 1e-9:
            continue
        alt = float(row.get("alt_m", 20.0) or 0.0)
        waypoints.append(
            Waypoint(
                lat=lat,
                lon=lon,
                alt_m=max(MIN_WP_ALT_M, alt),
                speed_mps=max(MIN_WP_SPEED_MPS, speed),
            )
        )

    return waypoints, end_action
