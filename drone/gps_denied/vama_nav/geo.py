"""Web Mercator tiles (the usual z/x/y map tiles of 256 pixels), and metres on the ground near a point.

A map tile's pixels are square on the ground: at latitude ``lat`` one pixel of zoom ``z`` is
``m_per_px(lat, z)`` metres east and north alike, with north up. So a picture turned north up and
scaled to that size lies on the map by a shift alone (see camera.py).
"""

from __future__ import annotations

import math

TILE = 256
EQUATOR_M = 40_075_016.685578488     # 2 pi 6378137 m, the sphere of Web Mercator
M_PER_DEG = 111_318.84502145034      # metres per degree of latitude, as ArduPilot (LOCATION_SCALING_FACTOR)


def world_px(lat: float, lon: float, zoom: int) -> tuple[float, float]:
    """The point's pixel on the whole world's map at ``zoom`` (x east, y south, from the top left)."""
    n = TILE * 2 ** zoom
    x = (lon + 180.0) / 360.0 * n
    y = (1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n
    return x, y


def latlon(x: float, y: float, zoom: int) -> tuple[float, float]:
    """The latitude and longitude of the world map's pixel (x, y) at ``zoom``."""
    n = TILE * 2 ** zoom
    lon = x / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * y / n))))
    return lat, lon


def m_per_px(lat: float, zoom: int) -> float:
    """Metres on the ground per map pixel at ``lat`` and ``zoom``."""
    return EQUATOR_M * math.cos(math.radians(lat)) / (TILE * 2 ** zoom)


def offset(lat: float, lon: float, north_m: float, east_m: float) -> tuple[float, float]:
    """The point ``north_m`` and ``east_m`` from (lat, lon), for distances of a few kilometres."""
    return (lat + north_m / M_PER_DEG,
            lon + east_m / (M_PER_DEG * max(math.cos(math.radians(lat)), 0.01)))


def north_east(lat1: float, lon1: float, lat2: float, lon2: float) -> tuple[float, float]:
    """Metres north and east from the first point to the second (a few kilometres)."""
    return ((lat2 - lat1) * M_PER_DEG,
            (lon2 - lon1) * M_PER_DEG * math.cos(math.radians((lat1 + lat2) / 2.0)))


def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    return math.hypot(*north_east(lat1, lon1, lat2, lon2))
