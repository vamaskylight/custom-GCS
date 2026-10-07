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

from vgcs.mission.waypoint_store import MAX_WP_HOVER_S, Waypoint, clamp_hover_seconds

__all__ = [
    "MISSION_END_ACTIONS",
    "DEFAULT_MISSION_END_ACTION",
    "MissionItem",
    "MissionPlan",
    "build_mission_plan",
    "DownloadedMission",
    "parse_downloaded_mission",
    "read_downloaded_mission",
    "plan_signature",
    "validate_waypoints",
    "normalize_end_action",
    "haversine_m",
    "MAX_WP_HOVER_S",
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

# How close an item read from the drone must be to count as the plan's item.
# The drone gives back what it stored: a place is a whole number of 1e-7
# degrees (the rest is cut off at the upload), a height a whole number of
# centimetres, and the other values are 32-bit numbers.
_SAME_PLACE_DEG = 2e-7
_SAME_HEIGHT_M = 0.02
_SAME_VALUE = 0.01


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
    # How many payload releases this mission carries, so the upload can say so
    # out loud. A servo command that fires unexpectedly is a dropped payload.
    drop_count: int = 0
    # Total seconds the aircraft will spend holding position at waypoints. This
    # is flight time that does not appear in the plan's distance, so it belongs
    # in the upload log next to the item count.
    hover_total_s: int = 0

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
        # Only nav items carry a waypoint number. In a mission made elsewhere
        # and read from the drone that can be a spline or a loiter point too.
        for item in self.items:
            if item.wp_index == idx:
                return item.seq
        return None

    def speed_for_seq(self, seq: int) -> float | None:
        """Ground speed (m/s) the plan asks for on the way to the waypoint at ``seq``.

        It is the last DO_CHANGE_SPEED before that item. ``None`` for anything
        that is not one of the operator's waypoints: the home slot, the
        take-off and the final return or landing fly at the drone's own speeds.

        Why this is needed at all: ArduCopter forgets the mission's speed
        whenever it enters AUTO, and a jump skips the speed items in between,
        so the link sends this speed itself (simulator, 2026-10-07).
        """
        item = self._speed_item_for_seq(seq)
        return None if item is None else float(item.p2)

    def speed_seq_for_seq(self, seq: int) -> int | None:
        """The mission item that sets the speed for the waypoint at ``seq``, or ``None``."""
        item = self._speed_item_for_seq(seq)
        return None if item is None else int(item.seq)

    def _speed_item_for_seq(self, seq: int) -> MissionItem | None:
        try:
            s = int(seq)
        except (TypeError, ValueError):
            return None
        target = next((item for item in self.items if item.seq == s), None)
        if target is None or target.wp_index is None:
            return None
        found: MissionItem | None = None
        for item in self.items:
            latest = -1 if found is None else found.seq
            if item.command != int(_m.MAV_CMD_DO_CHANGE_SPEED) or not (latest < item.seq < s):
                continue
            # p1: 0 air speed, 1 ground speed (a copter flies both the same
            # way), 2 climb, 3 descent. p2 of zero or less means "no change".
            if int(item.p1) not in (0, 1) or float(item.p2) <= 0.0:
                continue
            found = item
        return found

    def speed_for_waypoint_index(self, wp_index: int) -> float | None:
        seq = self.seq_for_waypoint_index(wp_index)
        return None if seq is None else self.speed_for_seq(seq)

    def position_for_seq(self, seq: int) -> tuple[float, float] | None:
        """Where the waypoint at ``seq`` is, or ``None`` for an item that is not a waypoint."""
        try:
            s = int(seq)
        except (TypeError, ValueError):
            return None
        for item in self.items:
            if item.seq == s and item.wp_index is not None:
                return float(item.lat), float(item.lon)
        return None

    def leg_can_start_again(self, seq: int) -> bool:
        """True when starting the leg to the waypoint at ``seq`` again drops nothing.

        Starting a leg again (MISSION_SET_CURRENT with its own item) makes the
        drone drop what is still running from the waypoint before. A payload
        release would be left open: its close command comes after a delay and
        would never run. So this is only true when nothing but speed changes
        sits between the nav item before and this one.
        """
        try:
            s = int(seq)
        except (TypeError, ValueError):
            return False
        by_seq = {item.seq: item for item in self.items}
        target = by_seq.get(s)
        if target is None or target.wp_index is None:
            return False
        before = s - 1
        while before >= 0:
            item = by_seq.get(before)
            if item is None:
                return False
            if item.command == int(_m.MAV_CMD_DO_CHANGE_SPEED):
                before -= 1
                continue
            # MAVLink's nav commands end at 95 (MAV_CMD_NAV_LAST).
            return int(item.command) <= 95
        return False

    # --- is this still the mission on the drone? -----------------------------
    #
    # The drone tells only the ground station that sent a mission that it has
    # a new one. Another station can replace the mission and VGCS hears nothing
    # (simulator, 2026-10-08), unless the number of items changes. So before
    # VGCS sends anything that counts on this plan, it reads the items
    # concerned from the drone and compares them with these.

    def items_to_check_for_seq(self, seq: int) -> list[int]:
        """The mission items the leg to the waypoint at ``seq`` depends on.

        The waypoint itself and the speed item that sets its speed. Where the
        plan says the leg can be started again (leg_can_start_again), also
        everything from the nav item before it up to it: a payload release
        there, put in by someone else, is what would make that unsafe.

        Item 0 is never one of them: the drone writes its home there. Empty for
        anything that is not one of the operator's waypoints.
        """
        try:
            s = int(seq)
        except (TypeError, ValueError):
            return []
        by_seq = {item.seq: item for item in self.items}
        target = by_seq.get(s)
        if target is None or target.wp_index is None:
            return []
        wanted = {s}
        speed_seq = self.speed_seq_for_seq(s)
        if speed_seq is not None:
            wanted.add(speed_seq)
        if self.leg_can_start_again(s):
            before = s - 1
            while before >= 1:
                item = by_seq.get(before)
                if item is None:
                    break
                wanted.add(before)
                if item.command != int(_m.MAV_CMD_DO_CHANGE_SPEED):
                    break         # the nav item before
                before -= 1
        return sorted(wanted)

    def same_item(self, seq: int, row: object) -> bool:
        """True when ``row``, an item read from the drone, is this plan's item at ``seq``.

        ``row`` is what a download gives for one item: command, frame, lat,
        lon, alt_m and p1 to p4. Compared are the command, its first two
        values (hover time, speed, servo output and pulse), the place and the
        height, and for an item with a place what its height is measured from.
        """
        try:
            s = int(seq)
        except (TypeError, ValueError):
            return False
        item = next((i for i in self.items if i.seq == s), None)
        if item is None or not isinstance(row, dict):
            return False

        def number(key: str) -> float:
            try:
                return float(row.get(key, 0.0) or 0.0)
            except (TypeError, ValueError):
                return float("nan")

        if int(number("command")) != int(item.command):
            return False
        # A comparison with "not a number" is never true, so a value that
        # cannot be read counts as different.
        if not (abs(number("p1") - float(item.p1)) <= _SAME_VALUE and abs(number("p2") - float(item.p2)) <= _SAME_VALUE):
            return False
        if not (abs(number("lat") - float(item.lat)) <= _SAME_PLACE_DEG
                and abs(number("lon") - float(item.lon)) <= _SAME_PLACE_DEG):
            return False
        if not abs(number("alt_m") - float(item.alt_m)) <= _SAME_HEIGHT_M:
            return False
        has_place = abs(float(item.lat)) > 1e-9 or abs(float(item.lon)) > 1e-9
        if has_place and int(number("frame")) != int(item.frame):
            return False
        return True

    def what_differs(self, rows: object) -> str:
        """In a few words, how the mission in ``rows`` differs from this one. Empty when it does not.

        ``rows`` is a whole mission as a download gives it. Item 0 is not
        compared: the drone writes its home there.
        """
        got = [row for row in (rows if isinstance(rows, list) else []) if isinstance(row, dict)]
        if len(got) != len(self.items):
            return f"it has {max(0, len(got) - 1)} items, VGCS knows {max(0, len(self.items) - 1)}"
        by_seq: dict[int, dict] = {}
        for row in got:
            try:
                by_seq[int(row.get("seq", -1))] = row
            except (TypeError, ValueError):
                continue
        for item in self.items:
            if item.seq == 0:
                continue
            if not self.same_item(item.seq, by_seq.get(item.seq)):
                return f"mission item {item.seq} is different"
        return ""

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


def _wp_drop_payload(wp: object) -> bool:
    """Whether this waypoint releases the payload on arrival."""
    if isinstance(wp, dict):
        return bool(wp.get("drop_payload", False))
    return bool(getattr(wp, "drop_payload", False))


def _wp_hover_s(wp: object) -> int:
    """Seconds to hold position at this waypoint, clamped to whole seconds."""
    if isinstance(wp, dict):
        return clamp_hover_seconds(wp.get("hover_s", 0))
    return clamp_hover_seconds(getattr(wp, "hover_s", 0))


def plan_signature(waypoints: list[object] | None) -> tuple[tuple[float, float, float, float, int, bool], ...]:
    """What makes two plans the same flight: each waypoint's place, height, speed, hover and drop.

    Used to tell whether the plan on the map is still the mission the drone
    holds. A command that names a waypoint by its number means the same on
    both sides only then.
    """
    rows = []
    for wp in waypoints or []:
        rows.append(
            (
                round(_wp_field(wp, "lat", 0.0), 7),
                round(_wp_field(wp, "lon", 0.0), 7),
                round(max(MIN_WP_ALT_M, _wp_field(wp, "alt_m", 20.0)), 2),
                round(max(MIN_WP_SPEED_MPS, _wp_field(wp, "speed_mps", 5.0)), 2),
                _wp_hover_s(wp),
                _wp_drop_payload(wp),
            )
        )
    return tuple(rows)


@dataclass(frozen=True)
class PayloadServo:
    """How the payload release is wired on this aircraft.

    Nothing here can be guessed from the plan: which output the servo is on and
    what pulse widths open and close it are properties of the airframe. They
    are settings, and a mission is built without any servo items at all when
    none is configured, so an unconfigured aircraft never gets a command it
    cannot honour.
    """

    channel: int = 9
    release_pwm: int = 1900
    reset_pwm: int = 1100
    hold_s: float = 1.0


def build_mission_plan(
    waypoints: list[object],
    *,
    takeoff_alt_m: float | None = None,
    end_action: str = DEFAULT_MISSION_END_ACTION,
    default_speed_mps: float = 5.0,
    servo: PayloadServo | None = None,
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

        # Hover on arrival. NAV_WAYPOINT param1 is ArduCopter's own hold time
        # in whole seconds (ModeAuto::do_nav_wp stores it as loiter_time_max),
        # so this needs no extra mission item and no extra seq to shift.
        #
        # It does move two things later, both correctly: MISSION_ITEM_REACHED
        # is sent when verify_nav_wp finally returns true, which is at the END
        # of the hold, so the map crosses the point off when the aircraft
        # actually leaves it. And a payload drop, being a DO_ command after
        # this one, fires at the end of the hover too: arrive, settle, release.
        hover_s = _wp_hover_s(wp)
        plan.items.append(
            MissionItem(
                seq=seq,
                command=int(_m.MAV_CMD_NAV_WAYPOINT),
                lat=_wp_field(wp, "lat", 0.0),
                lon=_wp_field(wp, "lon", 0.0),
                alt_m=max(MIN_WP_ALT_M, _wp_field(wp, "alt_m", 20.0)),
                p1=float(hover_s),
                wp_index=idx,
                label=f"WP {idx + 1}" + (f" (hover {hover_s} s)" if hover_s else ""),
            )
        )
        seq += 1
        plan.hover_total_s += hover_s

        # Payload drop, immediately after arriving at this point. Requested
        # 2026-09-11. DO_ commands run once the preceding NAV command has been
        # reached, so placing them here is what makes the servo fire on arrival
        # rather than on the way.
        #
        # These carry no wp_index on purpose: they are plumbing, and
        # waypoint_index_for_seq must keep translating MISSION_ITEM_REACHED to
        # the operator's waypoint number rather than to a servo command.
        if _wp_drop_payload(wp) and servo is not None:
            plan.items.append(
                MissionItem(
                    seq=seq,
                    command=int(_m.MAV_CMD_DO_SET_SERVO),
                    p1=float(servo.channel),
                    p2=float(servo.release_pwm),
                    label=f"Drop payload at WP {idx + 1}",
                )
            )
            seq += 1
            plan.drop_count += 1
            if servo.hold_s > 0.0:
                # Without the delay the servo is closed again in the same
                # instant it opened and the payload never clears the hatch.
                plan.items.append(
                    MissionItem(
                        seq=seq,
                        command=int(_m.MAV_CMD_CONDITION_DELAY),
                        p1=float(servo.hold_s),
                        label=f"Hold {servo.hold_s:.1f} s",
                    )
                )
                seq += 1
                plan.items.append(
                    MissionItem(
                        seq=seq,
                        command=int(_m.MAV_CMD_DO_SET_SERVO),
                        p1=float(servo.channel),
                        p2=float(servo.reset_pwm),
                        label="Close payload servo",
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


# The height reference the drone reports for a mission item with a place.
_FRAMES_ABOVE_SEA = frozenset({0, 5})        # MAV_FRAME_GLOBAL, and its _INT twin
_FRAMES_ABOVE_GROUND = frozenset({10, 11})   # MAV_FRAME_GLOBAL_TERRAIN_ALT, and its _INT twin

# Shown for a mission that sets no speed, when the drone's own default is not known.
_UNKNOWN_SPEED_MPS = 5.0

# A landing further than this from the last waypoint is at another place.
_SAME_PLACE_M = 2.0

# Commands a plan cannot hold, in the operator's words. Anything not listed is
# reported by its number.
_COMMAND_WORDS = {
    195: "camera aim point (ROI)",
    196: "camera aim point (ROI)",
    197: "camera aim point (ROI)",
    201: "camera aim point (ROI)",
    202: "camera setting",
    203: "camera trigger",
    206: "camera trigger by distance",
    2000: "camera trigger",
    2001: "camera trigger",
    2500: "video recording command",
    2501: "video recording command",
    204: "gimbal command",
    205: "gimbal command",
    1000: "gimbal command",
    183: "servo command",
    184: "servo command",
    181: "relay command",
    182: "relay command",
    177: "jump to another item",
    600: "jump to another item",
    601: "jump to another item",
    93: "delay",
    112: "delay",
    113: "height change",
    30: "height change",
    114: "wait for a distance",
    115: "turn to a heading",
    176: "mode change",
    179: "new home position",
    189: "landing start mark",
    207: "fence switch",
    208: "parachute command",
    211: "gripper command",
    92: "guided mode command",
    222: "guided mode command",
    94: "payload place",
    42600: "winch command",
}


@dataclass
class DownloadedMission:
    """A mission read from the drone, as far as a plan can hold it."""

    waypoints: list[Waypoint] = field(default_factory=list)
    end_action: str = "hold"
    # What the drone's mission has and the plan has not, one line each, in the
    # operator's words. Empty when an upload of the plan is the same mission.
    not_kept: list[str] = field(default_factory=list)


def _count(n: int, one: str) -> str:
    return f"1 {one}" if n == 1 else f"{n} {one}s"


def read_downloaded_mission(
    rows: list[object] | None,
    *,
    default_speed_mps: float | None = None,
    servo: PayloadServo | None = None,
) -> DownloadedMission:
    """Rebuild the plan from downloaded mission items, and say what it cannot hold.

    A plan holds waypoints (place, height above the launch point, speed, hover,
    payload release) and what happens after the last one. A mission can hold
    more. Until 2026-10-08 everything else was dropped without a word, and
    Start Mission uploads the plan on the map: so after a Download the payload
    was not released any more, and a mission made in another ground station
    lost its camera, servo and jump commands.

    ``servo`` is how the payload release is wired on this laptop. A servo
    command is a release only when it is that output and that pulse: uploaded
    again it is sent as exactly that, so anything else would move another
    servo than the mission meant.

    ``default_speed_mps`` is the drone's own mission speed (WPNAV_SPEED, or
    WP_SPD on 4.7). A mission that sets no speed is flown at it. ``None`` means
    it is not known.
    """
    mission = DownloadedMission()
    waypoints = mission.waypoints
    default_known = default_speed_mps is not None and float(default_speed_mps) > 0.0
    speed = float(default_speed_mps) if default_known else _UNKNOWN_SPEED_MPS
    speed_set = False
    skipped: dict[str, int] = {}           # words -> how many, in the order first met
    home_height: float | None = None       # the home's height above sea level, when the drone has a home
    took_off = False
    ended = False
    after_end = 0
    without_speed = 0
    above_sea = above_sea_left = above_ground = other_kind = 0
    landing_elsewhere = False
    held_s: float | None = None            # the hold of a release, when it is not this laptop's
    # A release is the servo opened, then (with a hold) a delay and the servo
    # closed: 1 after the opening, 2 after the delay, 0 otherwise.
    release_stage = 0
    released = False                       # the last waypoint has its release

    def skip(words: str) -> None:
        skipped[words] = skipped.get(words, 0) + 1

    def number(row: dict, key: str) -> float:
        try:
            return float(row.get(key, 0.0) or 0.0)
        except (TypeError, ValueError):
            return 0.0

    for row in rows or []:
        if not isinstance(row, dict) or "command" not in row:
            continue
        try:
            cmd = int(row.get("command") or 0)
        except (TypeError, ValueError):
            continue
        lat, lon = number(row, "lat"), number(row, "lon")
        placed = abs(lat) > 1e-9 or abs(lon) > 1e-9
        # seq 0 is the vehicle's home position, never an operator waypoint.
        try:
            is_home = row.get("seq", None) is not None and int(row.get("seq")) == 0
        except (TypeError, ValueError):
            is_home = False
        if is_home:
            if placed:
                home_height = number(row, "alt_m")
            continue
        if ended:
            # Nothing after the return or the landing is flown as part of this plan.
            after_end += 1
            continue
        p1, p2 = number(row, "p1"), number(row, "p2")
        stage, release_stage = release_stage, 0

        if cmd == int(_m.MAV_CMD_DO_CHANGE_SPEED):
            # p1: 0 air speed, 1 ground speed, 2 climb, 3 descent.
            if int(p1) in (0, 1):
                # param2 <= 0 means "no change" in the MAVLink spec.
                if p2 > 0.0:
                    speed = p2
                    speed_set = True
            else:
                skip("climb or descent speed")
            continue
        if cmd == int(_m.MAV_CMD_NAV_TAKEOFF):
            if took_off or waypoints:
                skip("take-off in the middle of the mission")
            took_off = True
            continue
        if cmd == int(_m.MAV_CMD_NAV_RETURN_TO_LAUNCH):
            mission.end_action = "rtl"
            ended = True
            continue
        if cmd == int(_m.MAV_CMD_NAV_LAND):
            mission.end_action = "land"
            ended = True
            if placed and waypoints:
                last = waypoints[-1]
                landing_elsewhere = haversine_m(last.lat, last.lon, lat, lon) > _SAME_PLACE_M
            continue
        if cmd in _NAV_WAYPOINT_COMMANDS:
            if not placed:
                skip("loiter without a place" if cmd != int(_m.MAV_CMD_NAV_WAYPOINT) else "waypoint without a place")
                continue
            alt = float(row.get("alt_m", 20.0) or 0.0)
            try:
                frame = int(row.get("frame", int(_m.MAV_FRAME_GLOBAL_RELATIVE_ALT)))
            except (TypeError, ValueError):
                frame = int(_m.MAV_FRAME_GLOBAL_RELATIVE_ALT)
            if frame in _FRAMES_ABOVE_SEA:
                if home_height is not None:
                    alt -= home_height
                    above_sea += 1
                else:
                    above_sea_left += 1
            elif frame in _FRAMES_ABOVE_GROUND:
                above_ground += 1
            if cmd != int(_m.MAV_CMD_NAV_WAYPOINT):
                other_kind += 1
            # NAV_WAYPOINT param1 is the hold time. NAV_LOITER_TIME uses param1 the
            # same way, and for the other loiter commands param1 is turns or a
            # radius, which is not a hover and must not be read as one.
            hover = 0
            if cmd in (int(_m.MAV_CMD_NAV_WAYPOINT), int(_m.MAV_CMD_NAV_LOITER_TIME)):
                hover = clamp_hover_seconds(row.get("p1", row.get("param1", 0)))
            if not speed_set and not default_known:
                without_speed += 1
            waypoints.append(
                Waypoint(
                    lat=lat,
                    lon=lon,
                    alt_m=max(MIN_WP_ALT_M, alt),
                    speed_mps=max(MIN_WP_SPEED_MPS, speed),
                    hover_s=hover,
                )
            )
            released = False
            continue
        if cmd == int(_m.MAV_CMD_DO_SET_SERVO) and servo is not None and waypoints and int(p1) == int(servo.channel):
            if int(p2) == int(servo.release_pwm) and not released:
                waypoints[-1].drop_payload = True
                released = True
                release_stage = 1
                continue
            if int(p2) == int(servo.reset_pwm) and stage in (1, 2):
                continue
        if cmd == int(_m.MAV_CMD_CONDITION_DELAY) and stage == 1:
            release_stage = 2
            if abs(p1 - float(servo.hold_s)) > 0.05:
                held_s = p1
            continue
        skip(_COMMAND_WORDS.get(cmd, f"command {cmd}"))

    notes = mission.not_kept
    notes.extend(f"{n} x {words}" for words, n in skipped.items())
    if held_s is not None and servo is not None:
        notes.append(
            f"the payload release is held open for {held_s:g} s on the drone, "
            f"and for {float(servo.hold_s):g} s in the settings of this laptop"
        )
    if above_sea:
        notes.append(
            f"{_count(above_sea, 'waypoint')} had its height above sea level. It is shown above the launch point here, "
            f"worked out with the height of the drone's home ({home_height:.0f} m)"
        )
    if above_sea_left:
        notes.append(
            f"{_count(above_sea_left, 'waypoint')} with a height above sea level, "
            "shown as if it were above the launch point: check it"
        )
    if above_ground:
        notes.append(
            f"{_count(above_ground, 'waypoint')} with a height above the ground (terrain), "
            "shown as if it were above the launch point"
        )
    if other_kind:
        notes.append(f"{_count(other_kind, 'waypoint')} of another kind (spline or loiter), shown as plain waypoints")
    if without_speed:
        notes.append(
            f"no speed is set for {_count(without_speed, 'waypoint')} at the start: the drone flies there "
            f"at its own default speed, and the rows show {_UNKNOWN_SPEED_MPS:.1f} m/s"
        )
    if landing_elsewhere:
        notes.append("the landing is at another place than the last waypoint")
    if after_end:
        notes.append(f"{_count(after_end, 'command')} after the return or landing")
    return mission


def parse_downloaded_mission(
    rows: list[object],
    *,
    default_speed_mps: float = 5.0,
    servo: PayloadServo | None = None,
) -> tuple[list[Waypoint], str]:
    """The waypoints and the end action of a downloaded mission.

    Returns ``(waypoints, end_action)``. See :func:`read_downloaded_mission`,
    which also says what the plan cannot hold.

    Without this, a mission downloaded straight back from the vehicle contains the
    home slot, the takeoff item and every speed change (items whose lat/lon are
    ``0,0``), and the map fills with waypoints in the Gulf of Guinea.
    """
    mission = read_downloaded_mission(rows, default_speed_mps=default_speed_mps, servo=servo)
    return mission.waypoints, mission.end_action
