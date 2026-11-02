"""The ground's heights (--heights), for hills: map tiles that hold heights instead of pictures.

A picture turned north up as if the ground were flat lies on the map by a shift alone only over flat
ground. Over a hill each point is nearer the camera than the height says, and lands off its place by
that difference times the tangent of its ray's angle from straight down: 100 m of hill seen 30 degrees
out is 58 m. The fit then takes the part of the picture that happens to agree. On a bench from 500 m,
10 degrees nose down, with the flight crossing the hills (the drone's angles exact, a summer map): over
hills of plus or minus 60 m and more the positions were up to 6 to 10 m off and some pictures gave none,
where with heights 8 m off they stayed within 3.4 m (2026-10-30). Over ground within 30 m it makes no
difference, and heights off by 8 m cost as much there as the hills do. In the simulator over hills of
plus or minus 100 m the mission flew on with the heights, and without them the EKF failsafe landed the
drone (the same day).

With the heights each ground point of the turned picture is taken from where the camera sees it at its
own height (geometry.rectify), and the drone's height is its height above the ground below it
(match_source.py).

The file is an MBTiles file like the map. Its tiles are PNG pictures in the "terrarium" coding of the
open Terrain Tiles: height = red x 256 + green + blue / 256 - 32768, in metres above sea level.
Heights below 0 are the sea's floor there: the camera sees the sea's surface, 0.

Real tiles have small bad patches, by steep walls and over water. A tile round the Sardar Sarovar lake
has 55 pixels more than 30 m off the heights round them, one at -456 m among 70 to 180 m; the plains
have none. Flown right over them a bad pixel costs almost nothing: the height below the drone goes into
both its own height and each point's rise. At home it would count for the whole flight: the ground at
home read 180 m off gave no position at all on a bench. So a pixel that far off takes the middle value
of the 5 by 5 round it. All the others stay exactly as they are.

VGCS makes the file beside the map ("Make the drone's map..."), named as ``beside_map`` says, and the
drone finds it there. Forgetting an option must not be what lands a drone.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from pathlib import Path

import cv2
import numpy as np

from .geo import TILE
from .map_tiles import MapError, TileMap

Rise = Callable[[np.ndarray, np.ndarray], np.ndarray]
ENCODING = "terrarium"              # the metadata "encoding" of a heights file
BAD_PIXEL_M = 30.0                  # this far from the 5 by 5 round it, a pixel is taken as bad


def terrarium_png(heights: np.ndarray) -> bytes:
    """Heights (metres) as a PNG picture in the terrarium coding."""
    value = np.round((np.asarray(heights, np.float64) + 32768.0) * 256.0).astype(np.int64)
    bgr = np.dstack([value & 255, (value >> 8) & 255, (value >> 16) & 255]).astype(np.uint8)
    ok, png = cv2.imencode(".png", bgr)
    if not ok:
        raise ValueError("the heights cannot be coded as a PNG picture")
    return png.tobytes()


class Heights(TileMap):
    """Reads one heights file, the file's finest zoom. Thread safe."""

    def __init__(self, path, zoom: int | None = None, cache_tiles: int = 64) -> None:
        super().__init__(path, zoom=zoom, cache_tiles=cache_tiles)
        if self.meta.get("encoding") != ENCODING:
            self.close()
            raise MapError(f"{self.path} is not a file of ground heights (its metadata has no encoding {ENCODING})")

    def _decode(self, data: bytes) -> np.ndarray | None:
        bgr = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            return None
        if bgr.shape[:2] != (TILE, TILE):
            bgr = cv2.resize(bgr, (TILE, TILE), interpolation=cv2.INTER_NEAREST)
        b, g, r = (bgr[..., i].astype(np.float64) for i in range(3))
        z = np.maximum(r * 256.0 + g + b / 256.0 - 32768.0, 0.0).astype(np.float32)
        around = cv2.medianBlur(z, 5)
        return np.where(np.abs(z - around) > BAD_PIXEL_M, around, z)

    def _raster(self, x0: int, y0: int, width: int, height: int) -> np.ndarray:
        """The heights from world pixel (x0, y0), ``width`` by ``height``: NaN where the file has none."""
        out = np.full((height, width), np.nan, np.float32)
        for ty in range(math.floor(y0 / TILE), math.floor((y0 + height - 1) / TILE) + 1):
            for tx in range(math.floor(x0 / TILE), math.floor((x0 + width - 1) / TILE) + 1):
                tile = self.tile(tx, ty)
                if tile is None:
                    continue
                ax, ay = max(x0, tx * TILE), max(y0, ty * TILE)
                bx, by = min(x0 + width, (tx + 1) * TILE), min(y0 + height, (ty + 1) * TILE)
                out[ay - y0:by - y0, ax - x0:bx - x0] = tile[ay - ty * TILE:by - ty * TILE, ax - tx * TILE:bx - tx * TILE]
        return out

    def at(self, lat: float, lon: float) -> float | None:
        """The ground's height above sea level at a place, or None when the file has none there."""
        x, y = self.px(lat, lon)
        # A heights pixel k spans k to k + 1 and holds the height of its middle (as a map's pixel, see geometry.render).
        x0, y0 = math.floor(x - 0.5), math.floor(y - 0.5)
        fx, fy = x - 0.5 - x0, y - 0.5 - y0
        r = self._raster(x0, y0, 2, 2)
        if np.isnan(r).any():
            return None
        return float(r[0, 0] * (1 - fx) * (1 - fy) + r[0, 1] * fx * (1 - fy) + r[1, 0] * (1 - fx) * fy + r[1, 1] * fx * fy)

    def around(self, lat: float, lon: float) -> Rise | None:
        """A function: for ground points (arrays of metres north and east of the place), how far each is
        above the ground at the place. None when the file has no height at the place. Ground the file
        does not have counts as level with the place."""
        below = self.at(lat, lon)
        if below is None:
            return None
        s = self.m_per_px(lat)
        cx, cy = self.px(lat, lon)

        def rise(north: np.ndarray, east: np.ndarray) -> np.ndarray:
            xs, ys = cx + np.asarray(east) / s, cy - np.asarray(north) / s
            x0, y0 = math.floor(float(xs.min()) - 1.0), math.floor(float(ys.min()) - 1.0)
            raster = self._raster(x0, y0, math.ceil(float(xs.max()) + 2.0) - x0, math.ceil(float(ys.max()) + 2.0) - y0)
            raster[np.isnan(raster)] = below
            # By hand, as in at(): cv2.remap takes at most 32767 points a side, and a row of rays can be longer.
            u, v = xs - x0 - 0.5, ys - y0 - 0.5
            i, j = u.astype(np.int64), v.astype(np.int64)
            fu, fv = u - i, v - j
            return ((raster[j, i] * (1 - fu) + raster[j, i + 1] * fu) * (1 - fv)
                    + (raster[j + 1, i] * (1 - fu) + raster[j + 1, i + 1] * fu) * fv) - below

        return rise


def beside_map(map_path: Path) -> Path:
    """The heights file of a map file: beside it, its name with "-heights"."""
    path = Path(map_path)
    return path.with_name(f"{path.stem}-heights{path.suffix}")


def heights_for_map(tile_map: TileMap, given: Path | None = None) -> Heights | None:
    """The heights to use with a map: the file given, else the one beside the map, else None. MapError for
    a file that is not one, and for a "map" that is one: the two lie side by side and are easily mixed up."""
    if tile_map.meta.get("encoding") == ENCODING:
        raise MapError(f"{tile_map.path} is a file of ground heights, not a map of pictures")
    path = given or beside_map(tile_map.path)
    return Heights(path) if given or path.is_file() else None
