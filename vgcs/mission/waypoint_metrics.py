"""Numbers the plan bar shows for the waypoint an operator has selected.

Kept apart from the window so the arithmetic can be tested without a screen.

The distinction that matters here is azimuth versus bearing. Azimuth is the
direction of the leg the aircraft will fly into the point, measured from the
previous waypoint. Bearing is the direction from wherever the aircraft is right
now to that point, so an operator standing over the map can say "WP 2 is out
that way" and check it against the compass. They are the same number only when
the aircraft happens to be sitting on the previous waypoint.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from vgcs.mission.mission_plan import _wp_field, haversine_m

BLANK_ANGLE = "---"
BLANK_DISTANCE = "-.- m"
BLANK_GRADIENT = "-.-"

# Below this the two points are the same place as far as an operator is
# concerned, and a direction between them is noise rather than information.
MIN_BEARING_DIST_M = 1.0


@dataclass(frozen=True)
class WaypointMetrics:
    """Plan-bar strings for one selected waypoint. Blank means "not known"."""

    alt_diff_m: str = BLANK_DISTANCE
    gradient: str = BLANK_GRADIENT
    azimuth: str = BLANK_ANGLE
    bearing: str = BLANK_ANGLE
    dist_prev_wp_m: str = BLANK_DISTANCE


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Geographic bearing from point 1 to point 2, degrees clockwise from north."""
    lat1r = math.radians(float(lat1))
    lat2r = math.radians(float(lat2))
    dlon = math.radians(float(lon2) - float(lon1))
    x = math.sin(dlon) * math.cos(lat2r)
    y = math.cos(lat1r) * math.sin(lat2r) - math.sin(lat1r) * math.cos(lat2r) * math.cos(dlon)
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


def format_bearing(deg: float) -> str:
    """Whole degrees, wrapped, so 359.7 reads 0 and never 360."""
    return f"{int(round(float(deg))) % 360}°"


def _lat_lon(wp: object) -> tuple[float, float] | None:
    lat = _wp_field(wp, "lat", math.nan)
    lon = _wp_field(wp, "lon", math.nan)
    if math.isnan(lat) or math.isnan(lon):
        return None
    return lat, lon


def selected_waypoint_metrics(
    waypoints: list[object],
    index: int,
    *,
    vehicle_pos: tuple[float, float] | None = None,
    vehicle_alt_m: float | None = None,
) -> WaypointMetrics:
    """Describe waypoint ``index`` relative to the leg into it and to the aircraft.

    Anything that cannot be worked out is left blank on purpose. A plan bar that
    shows a stale or invented number is worse than one that admits it does not
    know, because an operator flies on what it says.
    """
    try:
        idx = int(index)
    except (TypeError, ValueError):
        return WaypointMetrics()
    if idx < 0 or idx >= len(waypoints):
        return WaypointMetrics()
    target = _lat_lon(waypoints[idx])
    if target is None:
        return WaypointMetrics()

    # The leg into this point starts at the waypoint before it; for the first
    # waypoint it starts wherever the aircraft is, because that is the leg it
    # will actually fly.
    if idx > 0:
        origin = _lat_lon(waypoints[idx - 1])
        origin_alt: float | None = _wp_field(waypoints[idx - 1], "alt_m", math.nan)
        if origin_alt is not None and math.isnan(origin_alt):
            origin_alt = None
    else:
        origin = vehicle_pos
        origin_alt = None if vehicle_alt_m is None else float(vehicle_alt_m)

    azimuth = BLANK_ANGLE
    dist_text = BLANK_DISTANCE
    dist_m: float | None = None
    if origin is not None:
        dist_m = haversine_m(origin[0], origin[1], target[0], target[1])
        dist_text = f"{dist_m:.1f} m"
        if dist_m >= MIN_BEARING_DIST_M:
            azimuth = format_bearing(bearing_deg(origin[0], origin[1], target[0], target[1]))

    bearing = BLANK_ANGLE
    if vehicle_pos is not None:
        to_vehicle_m = haversine_m(vehicle_pos[0], vehicle_pos[1], target[0], target[1])
        if to_vehicle_m >= MIN_BEARING_DIST_M:
            bearing = format_bearing(
                bearing_deg(vehicle_pos[0], vehicle_pos[1], target[0], target[1])
            )

    alt_diff_text = BLANK_DISTANCE
    gradient = BLANK_GRADIENT
    target_alt = _wp_field(waypoints[idx], "alt_m", math.nan)
    if origin_alt is not None and not math.isnan(target_alt):
        alt_diff = float(target_alt) - float(origin_alt)
        alt_diff_text = f"{alt_diff:+.1f} m"
        if dist_m is not None and dist_m >= MIN_BEARING_DIST_M:
            gradient = f"{alt_diff / dist_m * 100.0:+.1f}%"

    return WaypointMetrics(
        alt_diff_m=alt_diff_text,
        gradient=gradient,
        azimuth=azimuth,
        bearing=bearing,
        dist_prev_wp_m=dist_text,
    )
