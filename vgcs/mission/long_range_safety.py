"""Pre-upload checks for missions that fly beyond contact.

Client request 2026-09-01: autonomous flight out to 10 km. VGCS could already
upload and fly such a mission — the aircraft does not care how far the waypoints
are — so nothing here makes 10 km *possible*. What it does is refuse the
configurations that turn a long mission into a lost airframe.

The reason this module exists is a specific accident. On 2026-08-31 an aircraft
was lost at a fraction of this range with this in the log::

    STATUSTEXT: GCS + Battery Failsafe - Continuing Landing

The vehicle *landed* where it lost contact. At 200 m that is an inconvenience.
At 10 km it is an aircraft in a field nobody can reach, which is exactly what
happened. Every check below exists to catch one way that repeats.

Two rules shape the whole module:

**Absence of data is never treated as a fault.** A parameter that has not been
fetched yet is reported as unchecked, not as an error. Blocking an upload
because telemetry is slow would train operators to ignore the checks, and an
ignored safety check is worse than none. This mirrors the rule in
:mod:`vgcs.app.preflight_checklist`.

**Only genuinely unrecoverable configurations block.** A fence the mission
breaches, or a failsafe that lands the aircraft out of reach, are errors. Tight
margins are warnings, because the operator may know something the software does
not.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from vgcs.mission.fence import ON_EDGE_M, TYPE_ALT_MAX, TYPE_CIRCLE, TYPE_POLYGON, distance_inside_m, leaves
from vgcs.mission.mission_plan import haversine_m

__all__ = [
    "LongRangeReport",
    "check_long_range_mission",
    "LONG_RANGE_THRESHOLD_M",
]

# Beyond this, an operator has almost certainly lost sight of the aircraft and
# the failsafe behaviour stops being academic.
LONG_RANGE_THRESHOLD_M = 2000.0

# Terrain clearance RTL must keep above the highest ground on the route. The
# aircraft returns with nobody watching, so this is not a place for a fine
# margin.
RTL_TERRAIN_CLEARANCE_M = 30.0

# Fraction of endurance the round trip may consume before it is called tight.
ENDURANCE_WARN_FRACTION = 0.8

# The drone's flight time on one battery, in minutes, as the operator enters it
# in Plan Flight (the Mission tab). Stored under this key. Until 2026-10-14
# nothing on screen set it, so the check could only say it had no figure.
FLIGHT_TIME_SETTING = "plan/endurance_min"
FLIGHT_TIME_RANGE_MIN = (1.0, 300.0)


def flight_time_from_setting(raw) -> float | None:
    """The flight time (minutes) a stored setting holds, or None when it holds none that makes sense."""
    try:
        value = float(str(raw if raw is not None else "").strip().replace(",", "."))
    except (TypeError, ValueError):
        return None
    if not FLIGHT_TIME_RANGE_MIN[0] <= value <= FLIGHT_TIME_RANGE_MIN[1]:
        return None
    return value

# --- ArduCopter parameter values ------------------------------------------ #
# FS_GCS_ENABLE / FS_THR_ENABLE: the actions that bring the aircraft home.
# Anything else either lands it where it stands or does nothing at all.
_FS_GCS_RETURNS_HOME = {1, 3, 4}      # RTL, SmartRTL, SmartRTL-or-RTL
_FS_THR_RETURNS_HOME = {1, 4, 5}      # RTL, SmartRTL, SmartRTL-or-RTL
# BATT_FS_LOW_ACT: 0 None, 1 Land, 2 RTL, 3 SmartRTL-or-RTL, 4 SmartRTL-or-Land
_BATT_LOW_RETURNS_HOME = {2, 3}
# RTL_ALT is centimetres in ArduCopter, not metres. Getting this wrong by 100x
# would silently pass every terrain check.
_RTL_ALT_CM_PER_M = 100.0


def rtl_altitude_m(p: dict) -> tuple[float, str] | None:
    """The RTL height in metres, and the setting it came from.

    ArduCopter 4.7 replaced RTL_ALT (centimetres) with RTL_ALT_M (metres). A
    4.7 drone has no RTL_ALT at all, so reading only that name left this check
    "not read yet" on the client's 4.7 drone, every time.
    """
    metres = _f(p.get("RTL_ALT_M"))
    if metres is not None:
        return float(metres), "RTL_ALT_M"
    centimetres = _f(p.get("RTL_ALT"))
    if centimetres is not None:
        return float(centimetres) / _RTL_ALT_CM_PER_M, "RTL_ALT"
    return None


@dataclass
class LongRangeReport:
    """Findings, split by whether they should stop the upload."""

    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    unchecked: list[str] = field(default_factory=list)
    """Checks that could not run because the input was unavailable. Surfaced so
    "no errors" is never mistaken for "everything was verified"."""

    farthest_m: float = 0.0
    round_trip_m: float = 0.0

    @property
    def is_long_range(self) -> bool:
        return self.farthest_m >= LONG_RANGE_THRESHOLD_M

    @property
    def ok(self) -> bool:
        return not self.errors


def check_long_range_mission(
    waypoints: list[object],
    *,
    home: tuple[float, float] | None,
    params: dict | None = None,
    dem_elevation_fn=None,
    endurance_min: float | None = None,
    cruise_speed_mps: float | None = None,
    fence_polygon: list | None = None,
) -> LongRangeReport:
    """Assess a mission for the failure modes that lose distant aircraft.

    ``params`` is the vehicle parameter cache; missing keys are reported as
    unchecked rather than assumed safe.

    ``fence_polygon`` is the polygon fence the vehicle holds, as (lat, lon)
    corners: what VGCS sent or read. None when VGCS does not know it.
    """
    report = LongRangeReport()
    pts = _points(waypoints)
    if not pts or home is None:
        if home is None:
            report.unchecked.append("Home position unknown — range checks skipped")
        return report

    legs = _leg_distances(home, pts)
    report.farthest_m = max(haversine_m(home[0], home[1], la, lo) for la, lo in pts)
    # Out along the route and straight back to home, which is what RTL flies.
    report.round_trip_m = sum(legs) + haversine_m(home[0], home[1], pts[-1][0], pts[-1][1])

    p = {str(k).upper(): v for k, v in (params or {}).items()}
    _check_fence(report, p, home, waypoints, fence_polygon)
    if report.is_long_range:
        _check_failsafes(report, p)
        _check_rtl_altitude(report, p, home, pts, dem_elevation_fn)
        _check_endurance(report, endurance_min, cruise_speed_mps, waypoints)
    elif (_f(endurance_min) or 0.0) > 0.0:
        # A flight time that was given holds for any mission: a short one can
        # hover long (2026-10-14).
        _check_endurance(report, endurance_min, cruise_speed_mps, waypoints)
    return report


# --------------------------------------------------------------------------- #


def _check_fence(report: LongRangeReport, p: dict, start, waypoints, fence_polygon) -> None:
    """Does the mission stay inside the fence the vehicle uses?

    FENCE_TYPE says which limits are in use: 1 the max height, 2 the circle
    around home, 4 the polygon. Until 2026-10-14 only FENCE_RADIUS was looked
    at. On a vehicle that uses a polygon the radius is not in use, and a
    mission was refused for a fence that was not there, while the max height
    and the polygon were never looked at.
    """
    enable = _f(p.get("FENCE_ENABLE"))
    radius = _f(p.get("FENCE_RADIUS"))
    kinds = _f(p.get("FENCE_TYPE"))
    if enable is None:
        report.unchecked.append("Geofence (FENCE_ENABLE / FENCE_RADIUS not read yet)")
        return
    if enable < 0.5:
        return  # fence off; nothing to breach
    # Without FENCE_TYPE the circle is taken to be in use, as before: in
    # ArduPilot's own defaults every limit is on.
    bits = None if kinds is None else int(round(kinds))
    if bits is None or bits & TYPE_CIRCLE:
        if radius is None:
            report.unchecked.append("Geofence (FENCE_ENABLE / FENCE_RADIUS not read yet)")
        elif radius > 0.0 and report.farthest_m > radius:
            report.errors.append(
                f"Mission reaches {report.farthest_m / 1000.0:.2f} km from home but the "
                f"geofence radius is {radius:.0f} m. The vehicle will hit the fence "
                "mid-mission. Raise FENCE_RADIUS or shorten the route."
            )
    if bits is None:
        return
    if bits & TYPE_ALT_MAX:
        _check_fence_height(report, p, waypoints)
    if bits & TYPE_POLYGON:
        _check_fence_polygon(report, start, waypoints, fence_polygon)


def _numbers(numbers: list[int]) -> str:
    """"Waypoint 3", "Waypoints 2 and 5", "Waypoints 1, 2, 3 and 9 more"."""
    shown = [str(n) for n in numbers[:3]]
    more = len(numbers) - len(shown)
    if more > 0:
        return f"Waypoints {', '.join(shown)} and {more} more"
    if len(shown) == 1:
        return f"Waypoint {shown[0]}"
    return f"Waypoints {', '.join(shown[:-1])} and {shown[-1]}"


def _check_fence_height(report: LongRangeReport, p: dict, waypoints) -> None:
    alt_max = _f(p.get("FENCE_ALT_MAX"))
    if alt_max is None:
        report.unchecked.append("Fence max height (FENCE_ALT_MAX not read yet)")
        return
    if alt_max <= 0.0:
        return
    high: list[tuple[int, float]] = []
    for number, wp in enumerate(waypoints or [], start=1):
        alt = _f(_attr(wp, "alt_m"))
        # At the limit counts: the vehicle breaches when it reaches it.
        if alt is not None and alt >= alt_max:
            high.append((number, alt))
    if high:
        several = len(high) > 1
        report.errors.append(
            f"{_numbers([n for n, _alt in high])} {'are' if several else 'is'} planned at "
            f"{max(alt for _n, alt in high):.0f} m, but the fence's max height is {alt_max:.0f} m. "
            "The vehicle will hit the fence mid-mission. Lower the waypoint, or raise Max height "
            "(Plan Flight, Fence)."
        )


def _check_fence_polygon(report: LongRangeReport, start, waypoints, fence_polygon) -> None:
    if fence_polygon is None:
        # A warning, not a line in the log: one press reads the fence.
        report.warnings.append(
            "The fence on the vehicle may include a polygon that VGCS has not read, so this mission is "
            "not checked against it. Press Read fence from drone (Plan Flight, Fence) first."
        )
        return
    corners: list[tuple[float, float]] = []
    for row in fence_polygon:
        try:
            corners.append((float(row[0]), float(row[1])))
        except (TypeError, ValueError, IndexError):
            continue
    if len(corners) < 3:
        return  # the polygon is chosen as a limit, but the vehicle holds none
    places: list[tuple[int, tuple[float, float]]] = []
    for number, wp in enumerate(waypoints or [], start=1):
        la, lo = _wp_latlon(wp)
        if la is None or lo is None or (abs(la) < 1e-7 and abs(lo) < 1e-7):
            continue
        places.append((number, (la, lo)))
    outside = [(n, distance_inside_m(corners, la, lo)) for n, (la, lo) in places]
    # On the fence counts: the vehicle breaches the moment it is there.
    outside = [(n, inside) for n, inside in outside if inside < ON_EDGE_M]
    if outside:
        several = len(outside) > 1
        far = max(abs(inside) for _n, inside in outside)
        where = (f"outside the polygon fence (by up to {far:.0f} m)" if far >= 0.5
                 else "on the edge of the polygon fence")
        report.errors.append(
            f"{_numbers([n for n, _inside in outside])} {'are' if several else 'is'} {where}. "
            "The vehicle will hit the fence mid-mission. Move the "
            f"waypoint{'s' if several else ''}, or upload a larger fence."
        )
        return
    # Every waypoint is inside. A fence with a dent can still lie between two of them.
    route: list[tuple[str, tuple[float, float]]] = []
    if start is not None and distance_inside_m(corners, start[0], start[1]) >= 0.0:
        route.append(("the vehicle", (float(start[0]), float(start[1]))))
    route += [(f"waypoint {n}", place) for n, place in places]
    for (name_a, a), (name_b, b) in zip(route, route[1:]):
        if leaves(corners, a, b):
            report.errors.append(
                f"The straight way from {name_a} to {name_b} leaves the polygon fence and comes back in. "
                "The vehicle will hit the fence there. Add a waypoint to go around, or upload another fence."
            )
            return


def _check_failsafes(report: LongRangeReport, p: dict) -> None:
    """The check that exists because of the 2026-08-31 loss."""
    gcs = _f(p.get("FS_GCS_ENABLE"))
    thr = _f(p.get("FS_THR_ENABLE"))
    batt = _f(p.get("BATT_FS_LOW_ACT"))

    if gcs is None:
        report.unchecked.append("GCS failsafe action (FS_GCS_ENABLE not read yet)")
    elif int(gcs) == 0:
        report.errors.append(
            "GCS failsafe is disabled (FS_GCS_ENABLE=0). Losing the link at this "
            "range would leave the aircraft flying with no operator and no "
            "recovery behaviour."
        )
    elif int(gcs) not in _FS_GCS_RETURNS_HOME:
        report.errors.append(
            f"GCS failsafe is set to {int(gcs)}, which does not return home. "
            "At this range the aircraft would come down where it lost contact. "
            "Press Apply failsafes, or set FS_GCS_ENABLE=1 (RTL)."
        )

    if thr is None:
        report.unchecked.append("RC failsafe action (FS_THR_ENABLE not read yet)")
    elif int(thr) != 0 and int(thr) not in _FS_THR_RETURNS_HOME:
        report.errors.append(
            f"RC failsafe is set to {int(thr)}, which does not return home. "
            "Set FS_THR_ENABLE=1 (RTL)."
        )

    if batt is None:
        report.unchecked.append("Battery failsafe action (BATT_FS_LOW_ACT not read yet)")
    elif int(batt) == 0:
        report.warnings.append(
            "Low-battery failsafe is disabled (BATT_FS_LOW_ACT=0) — nothing will "
            "start the aircraft home when the pack runs down."
        )
    elif int(batt) not in _BATT_LOW_RETURNS_HOME:
        report.errors.append(
            f"Low-battery failsafe is set to {int(batt)} (land), not return home. "
            "On a long mission this lands the aircraft wherever it happens to be. "
            "Set BATT_FS_LOW_ACT=2 (RTL)."
        )


def _check_rtl_altitude(report, p: dict, home, pts, dem_elevation_fn) -> None:
    """RTL flies home with nobody watching, so it must clear the ground."""
    found = rtl_altitude_m(p)
    if found is None:
        report.unchecked.append("RTL altitude (RTL_ALT or RTL_ALT_M not read yet)")
        return
    if dem_elevation_fn is None:
        report.unchecked.append("Terrain along the route (no DEM loaded)")
        return
    rtl_alt_m, setting = found
    if rtl_alt_m <= 0.0:
        report.warnings.append(
            f"{setting} is 0, so the aircraft returns at its current altitude, "
            "whatever that happens to be."
        )
        return

    home_elev = _sample(dem_elevation_fn, home[0], home[1])
    peak, peak_at = _max_terrain(dem_elevation_fn, home, pts)
    if home_elev is None or peak is None:
        report.unchecked.append("Terrain along the route (DEM has no data here)")
        return

    # RTL_ALT is relative to home, so compare against terrain rise above home.
    rise = peak - home_elev
    if rtl_alt_m < rise + RTL_TERRAIN_CLEARANCE_M:
        report.errors.append(
            f"RTL altitude {rtl_alt_m:.0f} m is not enough to clear the ground on "
            f"the way home: terrain rises {rise:.0f} m above home near "
            f"{peak_at[0]:.5f}, {peak_at[1]:.5f}. Set {setting} to at least "
            f"{rise + RTL_TERRAIN_CLEARANCE_M:.0f} m"
            + (
                f" ({(rise + RTL_TERRAIN_CLEARANCE_M) * _RTL_ALT_CM_PER_M:.0f} in RTL_ALT)."
                if setting == "RTL_ALT"
                else "."
            )
        )


def _check_endurance(report, endurance_min, cruise_speed_mps, waypoints) -> None:
    speed = _f(cruise_speed_mps) or _planned_speed(waypoints)
    if speed is None or speed <= 0.0:
        report.unchecked.append("Flight time (no cruise speed available)")
        return
    flying = (report.round_trip_m / speed) / 60.0
    # The hover at the waypoints counts too. Until 2026-10-14 it did not.
    hovering = _planned_hover_s(waypoints) / 60.0
    minutes = flying + hovering
    how = f"at {speed:.1f} m/s" + (f", with {hovering:.0f} min of hovering" if hovering >= 0.5 else "")
    endur = _f(endurance_min)
    if endur is None or endur <= 0.0:
        report.warnings.append(
            f"Round trip is {report.round_trip_m / 1000.0:.1f} km, about {minutes:.0f} min {how}. "
            "Confirm the battery covers that with reserve. VGCS has no flight time to check it against: "
            "enter it in Plan Flight, Mission tab, \"Flight time on one battery (min)\"."
        )
        return
    if minutes > endur:
        report.errors.append(
            f"Round trip needs about {minutes:.0f} min {how}, but the flight time on one battery is "
            f"{endur:.0f} min. The aircraft cannot get home."
        )
    elif minutes > endur * ENDURANCE_WARN_FRACTION:
        report.warnings.append(
            f"Round trip needs about {minutes:.0f} min of the {endur:.0f} min flight time. "
            "That leaves very little reserve for wind or a diversion."
        )


def _planned_hover_s(waypoints) -> float:
    """The hover time of all waypoints together, in seconds."""
    total = 0.0
    for wp in waypoints or []:
        hover = _f(_attr(wp, "hover_s"))
        if hover is not None and math.isfinite(hover) and hover > 0.0:
            total += hover
    return total


# --------------------------------------------------------------------------- #


def _points(waypoints) -> list[tuple[float, float]]:
    out: list[tuple[float, float]] = []
    for wp in waypoints or []:
        la, lo = _wp_latlon(wp)
        if la is None or lo is None:
            continue
        if abs(la) < 1e-7 and abs(lo) < 1e-7:
            continue
        out.append((la, lo))
    return out


def _wp_latlon(wp):
    for lat_key, lon_key in (("lat", "lon"), ("latitude", "longitude"), ("x", "y")):
        la = _attr(wp, lat_key)
        lo = _attr(wp, lon_key)
        if la is not None and lo is not None:
            return (_f(la), _f(lo))
    return (None, None)


def _attr(wp, key):
    if isinstance(wp, dict):
        return wp.get(key)
    return getattr(wp, key, None)


def _leg_distances(home, pts) -> list[float]:
    legs = [haversine_m(home[0], home[1], pts[0][0], pts[0][1])]
    for a, b in zip(pts, pts[1:]):
        legs.append(haversine_m(a[0], a[1], b[0], b[1]))
    return legs


def _max_terrain(fn, home, pts):
    """Highest ground sampled along the route and the direct line home."""
    best = None
    best_at = home
    route = [home] + list(pts) + [home]
    for a, b in zip(route, route[1:]):
        for la, lo in _sample_line(a, b):
            e = _sample(fn, la, lo)
            if e is None:
                continue
            if best is None or e > best:
                best, best_at = e, (la, lo)
    return best, best_at


def _sample_line(a, b, *, step_m: float = 250.0, max_samples: int = 80):
    """Points along a leg. Stepping matters: sampling only the waypoints would
    miss the ridge between two of them, which is precisely what kills an
    aircraft returning on autopilot."""
    dist = haversine_m(a[0], a[1], b[0], b[1])
    n = max(1, min(int(max_samples), int(math.ceil(dist / max(1.0, step_m)))))
    for i in range(n + 1):
        t = i / n
        yield (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


def _sample(fn, lat, lon):
    try:
        v = fn(float(lat), float(lon))
    except Exception:
        return None
    return _f(v)


def _planned_speed(waypoints) -> float | None:
    """Slowest planned leg speed — the honest basis for a time estimate, since
    the mission is only as quick as its slowest section."""
    speeds = []
    for wp in waypoints or []:
        v = _f(_attr(wp, "speed_mps"))
        if v is not None and v > 0.0:
            speeds.append(v)
    return min(speeds) if speeds else None


def _f(raw):
    if raw is None:
        return None
    try:
        return float(raw)
    except (TypeError, ValueError):
        return None
