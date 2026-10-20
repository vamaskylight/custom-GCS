"""A polygon geofence: its corners, when it can be used, and what the drone stores for it.

The drone keeps a fence as a list of items, the same way it keeps a mission
(MAVLink 2, mission type "fence"). A polygon is one item for each corner, and
every corner item says how many corners its polygon has.

Measured in the ArduCopter simulator, 4.6.2 and 4.7.0, on 2026-10-14:

- VGCS used to send a polygon with FENCE_POINT messages. Both versions answer
  "Invalid FENCE_TOTAL" and store nothing, and with the polygon kind switched on
  the drone then refuses to arm ("Fences enabled, but none selected"). VGCS
  said "Polygon uploaded".
- The item list is taken, and comes back the same to the last digit (1e-7 degree).
- The drone checks that a polygon's corners add up. It does not check its shape:
  a polygon whose sides cross, and one with a corner given twice, are taken.
  So the shape is checked here.
- An upload replaces every fence item the drone holds, also another ground
  station's exclusion areas.
- A drone outside its polygon does not arm ("Vehicle breaching Polygon fence").
  In flight it turns back or lands the moment such a fence is switched on.
  So a fence that does not hold the drone is not sent to a flying drone.
- The simulator holds 70 items. A drone with less room refuses a larger fence.

Areas to stay out of (2026-10-21): polygons (item 5002) and circles (item 5004,
param1 the radius in metres). Measured in the simulator, 4.6.2 and 4.7.0:

- They are stored with an area to stay inside, or alone, and read back exactly.
  A fence of only such areas arms and works (FENCE_TYPE with the polygon bit).
- The drone does not arm inside one ("Vehicle breaching Polygon fence"). Asked
  within a second of the upload it armed once inside a circle: the drone checks
  a new fence in its own loop.
- In flight it turns back at the edge ("Polygon fence breached", then the
  FENCE_ACTION). In AUTO at 10 m/s it was 21 m inside before it turned, and the
  mission was over: it returned home and landed.

Pure Python, no Qt and no pymavlink, so it is tested without a link.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

# MAV_CMD numbers of the fence items.
CMD_RETURN_POINT = 5000
CMD_POLYGON_INCLUSION = 5001
CMD_POLYGON_EXCLUSION = 5002
CMD_CIRCLE_INCLUSION = 5003
CMD_CIRCLE_EXCLUSION = 5004

# FENCE_TYPE is a set of bits.
TYPE_ALT_MAX = 1
TYPE_CIRCLE = 2
TYPE_POLYGON = 4
TYPE_ALT_MIN = 8

MIN_CORNERS = 3
# The corner count is one byte on the drone.
MAX_CORNERS = 255
# Two corners nearer than this are one corner clicked twice.
MIN_CORNER_GAP_M = 1.0
# A corner this near to a side that is not its own lies on that side.
ON_SIDE_M = 0.5
# How far inside the fence the drone (and in flight its home point) has to be.
# A position on the ground wanders a few metres, and the drone acts the moment
# it is outside.
EDGE_MIN_M = 5.0
# How far below the height limit a flying drone has to be.
HEIGHT_MIN_M = 2.0
# The drone keeps a position in steps of 1e-7 degree, about 1 cm. A place
# nearer to the fence's edge than this is on the edge, as far as the drone can tell.
ON_EDGE_M = 0.02
# A circle to stay out of: its radius in metres.
NOFLY_RADIUS_RANGE_M = (5.0, 20_000.0)

# What the Fence tab says before a corner is drawn.
NO_POLYGON_WORDS = ("No polygon yet. Press Draw on the map, then click the map at each corner, "
                    "in order around the area.")
# What the Fence tab says before an area to stay out of is drawn.
NO_NOFLY_WORDS = ("No areas to stay out of. Press Draw a no-fly area and click its corners on the map, "
                  "or Add no-fly circles and click where each one goes.")

_M_PER_DEG = 111_320.0

Corner = tuple[float, float]   # latitude, longitude in degrees


def _flat(corners: list[Corner], origin: Corner | None = None) -> list[tuple[float, float]]:
    """The corners as metres east and north of `origin` (the first corner when none is given).

    A flat map of the fence's own area: exact enough for a fence of some
    kilometres, which is all a multicopter's fence ever is.
    """
    if not corners:
        return []
    lat0, lon0 = origin if origin is not None else corners[0]
    east = _M_PER_DEG * math.cos(math.radians(lat0))
    return [((lon - lon0) * east, (lat - lat0) * _M_PER_DEG) for lat, lon in corners]


def distance_m(a: Corner, b: Corner) -> float:
    (ax, ay), (bx, by) = _flat([a, b], a)
    return math.hypot(bx - ax, by - ay)


def _to_side_m(p: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
    """How far the point is from the side a-b."""
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    if length2 <= 0.0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _turn(a, b, c) -> float:
    """More than 0 when a, b, c turn left, less when they turn right."""
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _cross(a, b, c, d) -> bool:
    """The sides a-b and c-d cross each other.

    Only asked for sides that share no corner, and after the corner check: a
    side that only touches another has a corner on it, which is found there.
    """
    d1, d2 = _turn(c, d, a), _turn(c, d, b)
    d3, d4 = _turn(a, b, c), _turn(a, b, d)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def _goes_through(start, end, c, d) -> bool:
    """The flight start-end goes through the side c-d, somewhere between its own two ends.

    An end that lies on the side is not "between": a flight that starts on the
    fence and goes inward goes through nothing. A corner that lies on the
    flight's line belongs to one of its two sides only (as in
    distance_inside_m), so a flight out through a corner goes through one side.
    """
    d1, d2 = _turn(c, d, start), _turn(c, d, end)
    d3, d4 = _turn(start, end, c), _turn(start, end, d)
    return ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and ((d3 > 0) != (d4 > 0))


def _a_position(row: object) -> Corner | None:
    lat = lon = None
    if isinstance(row, dict):
        lat, lon = row.get("lat"), row.get("lon")
    elif isinstance(row, (list, tuple)) and len(row) >= 2:
        lat, lon = row[0], row[1]
    try:
        lat, lon = float(lat), float(lon)
    except (TypeError, ValueError):
        return None
    # (A number that is none, or without end, is in no range either.)
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        return None
    if abs(lat) < 1e-9 and abs(lon) < 1e-9:
        return None            # 0, 0: a position that was never set
    return lat, lon


def check_polygon(points: object) -> tuple[list[Corner], str]:
    """The corners of a fence drawn on the map, and what is wrong with them.

    Returns (corners, problem). problem is "" for a fence that can be sent,
    else one sentence for the operator. The drone itself takes a polygon whose
    sides cross (simulator, see the top of this file), and where "inside" is
    for such a shape nobody can tell from the map.
    """
    rows = list(points) if isinstance(points, (list, tuple)) else []
    corners: list[Corner] = []
    for number, row in enumerate(rows, start=1):
        position = _a_position(row)
        if position is None:
            return corners, f"Corner {number} is not a position on the map."
        corners.append(position)
    count = len(corners)
    if count < MIN_CORNERS:
        if count == 0:
            return corners, "The fence has no corners yet."
        return corners, f"A fence needs {MIN_CORNERS} corners or more. This one has {count}."
    if count > MAX_CORNERS:
        return corners, f"A fence can have {MAX_CORNERS} corners at most. This one has {count}."
    flat = _flat(corners)
    for i in range(count):
        for j in range(i + 1, count):
            if math.hypot(flat[j][0] - flat[i][0], flat[j][1] - flat[i][1]) < MIN_CORNER_GAP_M:
                return corners, f"Corners {i + 1} and {j + 1} are at the same place. Remove one of them."
    sides = [(i, (i + 1) % count) for i in range(count)]
    for k in range(count):
        for a, b in sides:
            if k in (a, b):
                continue
            if _to_side_m(flat[k], flat[a], flat[b]) < ON_SIDE_M:
                return corners, (f"Corner {k + 1} lies on the side from corner {a + 1} to corner {b + 1}. "
                                 "Move it, or remove it.")
    for first in range(count):
        a, b = sides[first]
        for second in range(first + 1, count):
            c, d = sides[second]
            if len({a, b, c, d}) < 4:
                continue           # neighbours share a corner
            if _cross(flat[a], flat[b], flat[c], flat[d]):
                return corners, (f"The side from corner {a + 1} to corner {b + 1} crosses the side from corner "
                                 f"{c + 1} to corner {d + 1}. Give the corners in order around the area.")
    return corners, ""


def distance_inside_m(corners: list[Corner], lat: float, lon: float) -> float:
    """How far inside the fence a position is: metres to the nearest side.

    Less than 0 for a position outside (then it is the distance to the fence).
    """
    if len(corners) < MIN_CORNERS:
        return float("-inf")
    flat = _flat(list(corners) + [(float(lat), float(lon))], corners[0])
    point, ring = flat[-1], flat[:-1]
    count = len(ring)
    nearest = min(_to_side_m(point, ring[i], ring[(i + 1) % count]) for i in range(count))
    inside = False
    px, py = point
    for i in range(count):
        (ax, ay), (bx, by) = ring[i], ring[(i + 1) % count]
        if (ay > py) != (by > py) and px < ax + (py - ay) * (bx - ax) / (by - ay):
            inside = not inside
    return nearest if inside else -nearest


def holds(corners: list[Corner], lat: float, lon: float, edge_m: float = EDGE_MIN_M) -> bool:
    """The position is inside the fence, and at least `edge_m` from its edge."""
    return distance_inside_m(corners, lat, lon) >= float(edge_m)


def leaves(corners: list[Corner], a: Corner, b: Corner) -> bool:
    """A straight flight from a to b is outside the fence at some point.

    Both ends inside is not enough for a fence with a dent: the line between
    them can cut across the dent. An end that lies on the fence itself is not
    outside. (A flight that runs along a side is on the fence all the way: it
    is told as leaving or as staying, either can come out.)
    """
    if len(corners) < MIN_CORNERS:
        return False
    if distance_inside_m(corners, a[0], a[1]) < 0.0 or distance_inside_m(corners, b[0], b[1]) < 0.0:
        return True
    flat = _flat(list(corners) + [(float(a[0]), float(a[1])), (float(b[0]), float(b[1]))], corners[0])
    ring, start, end = flat[:-2], flat[-2], flat[-1]
    count = len(ring)
    if any(_goes_through(start, end, ring[i], ring[(i + 1) % count]) for i in range(count)):
        return True
    # It goes through no side, so it is on one side of the fence all the way.
    # Its middle tells which: both ends can lie on the fence, with a dent between them.
    return distance_inside_m(corners, (a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0) < 0.0


def nearest_edge_m(corners: list[Corner], a: Corner, b: Corner) -> float:
    """How near a straight flight from a to b comes to the fence's edge, in metres (0 where it crosses it)."""
    if len(corners) < MIN_CORNERS:
        return float("inf")
    flat = _flat(list(corners) + [(float(a[0]), float(a[1])), (float(b[0]), float(b[1]))], corners[0])
    ring, start, end = flat[:-2], flat[-2], flat[-1]
    count = len(ring)
    nearest = float("inf")
    for i in range(count):
        c, d = ring[i], ring[(i + 1) % count]
        if _cross(start, end, c, d):
            return 0.0
        nearest = min(nearest, _to_side_m(start, c, d), _to_side_m(end, c, d),
                      _to_side_m(c, start, end), _to_side_m(d, start, end))
    return nearest


def outside_words(what: str, inside_m: float, edge_m: float = EDGE_MIN_M) -> str:
    """Why a fence does not hold a position, in the operator's words. `what`: "The drone", "The drone's home point"."""
    if inside_m < 0.0:
        return f"{what} is outside this fence, {abs(inside_m):.0f} m from its edge."
    return (f"{what} is only {inside_m:.0f} m inside this fence. "
            f"Its edge has to be {edge_m:.0f} m away or more.")


# --- areas to stay out of ---------------------------------------------------

@dataclass(frozen=True)
class NoFly:
    """An area to stay out of: a polygon (its corners), or a circle (its centre and radius)."""

    corners: tuple[Corner, ...] = ()
    centre: Corner | None = None
    radius_m: float = 0.0

    @property
    def is_circle(self) -> bool:
        return self.centre is not None

    def words(self) -> str:
        """"a circle of 50 m", "an area of 5 corners"."""
        if self.is_circle:
            return f"a circle of {self.radius_m:.0f} m"
        return f"an area of {_corner_words(len(self.corners))}"

    def drone_words(self) -> str:
        """As a list of what a drone holds names it: "a circle to stay out of (50 m)"."""
        if self.is_circle:
            return f"a circle to stay out of ({self.radius_m:.0f} m)"
        return f"an area to stay out of ({_corner_words(len(self.corners))})"

    def row(self) -> dict:
        """As the map, the link and a plan file keep it."""
        if self.is_circle:
            return {"circle": [self.centre[0], self.centre[1]], "radius_m": float(self.radius_m)}
        return {"polygon": [[lat, lon] for lat, lon in self.corners]}


def _radius(raw: object) -> float | None:
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return None
    return value if NOFLY_RADIUS_RANGE_M[0] <= value <= NOFLY_RADIUS_RANGE_M[1] else None


def check_nofly(rows: object) -> tuple[list[NoFly], str]:
    """The areas to stay out of drawn on the map, and what is wrong with the first one that cannot be sent.

    Returns (areas, problem). problem is "" when every area can be sent, else one
    sentence for the operator that names the area by its number.
    """
    areas: list[NoFly] = []
    for number, row in enumerate(list(rows) if isinstance(rows, (list, tuple)) else [], start=1):
        if not isinstance(row, dict):
            return areas, f"No-fly area {number} is not an area."
        if "circle" in row:
            centre = _a_position(row.get("circle"))
            if centre is None:
                return areas, f"No-fly area {number}: its centre is not a position on the map."
            radius = _radius(row.get("radius_m"))
            if radius is None:
                low, high = NOFLY_RADIUS_RANGE_M
                return areas, f"No-fly area {number}: a circle needs a radius of {low:.0f} to {high:.0f} m."
            areas.append(NoFly(centre=centre, radius_m=radius))
            continue
        corners, problem = check_polygon(row.get("polygon"))
        if problem:
            return areas, f"No-fly area {number}: {problem[:1].lower()}{problem[1:]}"
        areas.append(NoFly(corners=tuple(corners)))
    return areas, ""


def nofly_outside_m(area: NoFly, lat: float, lon: float) -> float:
    """How far outside the area a position is, in metres. Less than 0 inside it."""
    if area.is_circle:
        return distance_m(area.centre, (float(lat), float(lon))) - float(area.radius_m)
    return -distance_inside_m(list(area.corners), lat, lon)


def nofly_gap_m(area: NoFly, a: Corner, b: Corner) -> float:
    """How near a straight flight from a to b comes to the area, in metres: 0 where it enters it."""
    if nofly_outside_m(area, a[0], a[1]) <= 0.0 or nofly_outside_m(area, b[0], b[1]) <= 0.0:
        return 0.0
    if not area.is_circle:
        return nearest_edge_m(list(area.corners), a, b)
    (sx, sy), (ex, ey) = _flat([(float(a[0]), float(a[1])), (float(b[0]), float(b[1]))], area.centre)
    dx, dy = ex - sx, ey - sy
    length2 = dx * dx + dy * dy
    t = 0.0 if length2 <= 0.0 else max(0.0, min(1.0, -(sx * dx + sy * dy) / length2))
    return max(0.0, math.hypot(sx + t * dx, sy + t * dy) - float(area.radius_m))


def _same_place(a: Corner, b: Corner) -> bool:
    """Two positions are the same to the drone: equal to 1e-7 degree."""
    return (int(round(float(a[0]) * 1e7)) == int(round(float(b[0]) * 1e7))
            and int(round(float(a[1]) * 1e7)) == int(round(float(b[1]) * 1e7)))


def same_nofly(a: NoFly, b: NoFly) -> bool:
    """The same area to the drone: the same corners in the same order, or the same centre and radius."""
    if a.is_circle != b.is_circle:
        return False
    if a.is_circle:
        return _same_place(a.centre, b.centre) and abs(float(a.radius_m) - float(b.radius_m)) <= 0.01
    return len(a.corners) == len(b.corners) and all(_same_place(p, q) for p, q in zip(a.corners, b.corners))


def nofly_from_rows(rows: object) -> list[NoFly]:
    """The areas in rows the link or a plan file holds. A row that is no area is left out."""
    out: list[NoFly] = []
    for row in list(rows) if isinstance(rows, (list, tuple)) else []:
        areas, problem = check_nofly([row])
        if not problem:
            out.extend(areas)
    return out


# --- what goes over the link ----------------------------------------------

@dataclass(frozen=True)
class FenceItem:
    """One fence item as the drone stores it. A position is whole numbers of 1e-7 degree."""

    command: int
    p1: float
    lat_e7: int
    lon_e7: int


def polygon_items(corners: list[Corner]) -> list[FenceItem]:
    """The items of one inclusion polygon: the drone has to stay inside it."""
    count = float(len(corners))
    return [FenceItem(CMD_POLYGON_INCLUSION, count, int(round(lat * 1e7)), int(round(lon * 1e7)))
            for lat, lon in corners]


def fence_items(corners: list[Corner], nofly: list[NoFly]) -> list[FenceItem]:
    """The items of a fence: the area to stay inside (when there is one), then each area to stay out of."""
    items = polygon_items(corners) if corners else []
    for area in nofly:
        if area.is_circle:
            lat, lon = area.centre
            items.append(FenceItem(CMD_CIRCLE_EXCLUSION, float(area.radius_m), int(round(lat * 1e7)),
                                   int(round(lon * 1e7))))
            continue
        count = float(len(area.corners))
        items.extend(FenceItem(CMD_POLYGON_EXCLUSION, count, int(round(lat * 1e7)), int(round(lon * 1e7)))
                     for lat, lon in area.corners)
    return items


def item_from_row(row: dict) -> FenceItem:
    """A fence item from what the link read (command, p1, lat_e7, lon_e7)."""
    return FenceItem(int(row.get("command", 0) or 0), float(row.get("p1", 0.0) or 0.0),
                     int(row.get("lat_e7", 0) or 0), int(row.get("lon_e7", 0) or 0))


def same_items(sent: list[FenceItem], got: list[FenceItem]) -> bool:
    """The drone holds exactly what was sent."""
    if len(sent) != len(got):
        return False
    for a, b in zip(sent, got):
        if a.command != b.command or a.lat_e7 != b.lat_e7 or a.lon_e7 != b.lon_e7:
            return False
        if abs(float(a.p1) - float(b.p1)) > 0.01:
            return False
    return True


def _corner_words(count: int) -> str:
    return f"{count} corner{'' if count == 1 else 's'}"


@dataclass
class FenceOnDrone:
    """The fence items a drone holds, as VGCS can show them."""

    # The first inclusion polygon, the one VGCS draws. Empty when there is none.
    polygon: list[Corner] = field(default_factory=list)
    # The areas to stay out of, in the drone's order (VGCS draws them since 2026-10-21).
    nofly: list[NoFly] = field(default_factory=list)
    # What the fence holds besides, one line each. VGCS does not show these,
    # and an upload from VGCS removes them.
    other: list[str] = field(default_factory=list)
    # Not empty when the items do not make a whole fence.
    problem: str = ""
    item_count: int = 0


def read_fence(items: list[FenceItem]) -> FenceOnDrone:
    """Make sense of the fence items read from a drone."""
    out = FenceOnDrone(item_count=len(items))
    i = 0
    while i < len(items):
        item = items[i]
        if item.command in (CMD_POLYGON_INCLUSION, CMD_POLYGON_EXCLUSION):
            count = int(round(item.p1))
            ring = items[i:i + count]
            whole = (count >= MIN_CORNERS and len(ring) == count
                     and all(r.command == item.command and int(round(r.p1)) == count for r in ring))
            if not whole:
                out.problem = (f"The fence on the drone is not whole: item {i + 1} belongs to a polygon of "
                               f"{count} corners that is not all there.")
                return out
            corners = [(r.lat_e7 / 1e7, r.lon_e7 / 1e7) for r in ring]
            if item.command == CMD_POLYGON_INCLUSION and not out.polygon:
                out.polygon = corners
            elif item.command == CMD_POLYGON_INCLUSION:
                out.other.append(f"another area to stay inside ({_corner_words(count)})")
            else:
                out.nofly.append(NoFly(corners=tuple(corners)))
            i += count
            continue
        if item.command == CMD_CIRCLE_INCLUSION:
            out.other.append(f"a circle to stay inside ({item.p1:.0f} m)")
        elif item.command == CMD_CIRCLE_EXCLUSION:
            out.nofly.append(NoFly(centre=(item.lat_e7 / 1e7, item.lon_e7 / 1e7), radius_m=float(item.p1)))
        elif item.command == CMD_RETURN_POINT:
            out.other.append("a return point")
        else:
            out.other.append(f"an item VGCS does not know (command {item.command})")
        i += 1
    return out


# --- words ----------------------------------------------------------------

_ACTIONS = {0: "warn only", 1: "return to launch", 2: "land", 3: "SmartRTL, else return to launch",
            4: "brake, else land", 5: "SmartRTL, else land"}


def action_words(action: object) -> str:
    """FENCE_ACTION of a multicopter in words."""
    try:
        number = int(round(float(action)))
    except (TypeError, ValueError):
        return "not known"
    return _ACTIONS.get(number, f"action {number}")


def kinds_words(kinds: object) -> str:
    """FENCE_TYPE in words: which limits are switched on."""
    try:
        bits = int(round(float(kinds)))
    except (TypeError, ValueError):
        return "not known"
    names = []
    if bits & TYPE_ALT_MAX:
        names.append("height limit")
    if bits & TYPE_CIRCLE:
        names.append("circle around home")
    if bits & TYPE_POLYGON:
        names.append("polygon")
    if bits & TYPE_ALT_MIN:
        names.append("lowest height")
    return ", ".join(names) if names else "none"
