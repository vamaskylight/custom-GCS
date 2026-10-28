"""The map on board: satellite pictures as map tiles in one MBTiles file (SQLite, z/x/y tiles).

MBTiles is a common format (one file, easy to copy to the card), and SQLite comes with Python. Its
rows count from the bottom (``tile_row`` is 2^z - 1 - y). One zoom level is used for the matching,
by default the file's finest. A tile that is not in the file is blank and marked as no map, so no
picture is matched to it.
"""

from __future__ import annotations

import math
import sqlite3
import threading
from collections import OrderedDict
from pathlib import Path

import cv2
import numpy as np

from .geo import TILE, latlon, m_per_px, world_px


class MapError(RuntimeError):
    """The map file cannot be used, in plain words."""


class TileMap:
    """Reads one MBTiles file. Thread safe."""

    def __init__(self, path: Path, zoom: int | None = None, cache_tiles: int = 400) -> None:
        self.path = Path(path)
        if not self.path.is_file():
            raise MapError(f"no map file at {self.path}")
        try:
            self._db = sqlite3.connect(f"file:{self.path.as_posix()}?mode=ro", uri=True, check_same_thread=False)
            self.meta = dict(self._db.execute("SELECT name, value FROM metadata").fetchall())
            zooms = [int(z) for (z,) in self._db.execute("SELECT DISTINCT zoom_level FROM tiles")]
        except sqlite3.Error as exc:
            raise MapError(f"{self.path} is not a map file (MBTiles): {exc}") from None
        if not zooms:
            raise MapError(f"{self.path} holds no tiles")
        if zoom is not None and zoom not in zooms:
            raise MapError(f"{self.path} has no tiles at zoom {zoom} (it has {sorted(zooms)})")
        self.zoom = int(zoom if zoom is not None else max(zooms))
        self.tiles = int(self._db.execute("SELECT COUNT(*) FROM tiles WHERE zoom_level=?", (self.zoom,)).fetchone()[0])
        self._lock = threading.Lock()
        self._cache: OrderedDict[tuple[int, int], np.ndarray | None] = OrderedDict()
        self._cache_tiles = int(cache_tiles)

    def describe(self) -> str:
        name = self.meta.get("name") or self.path.name
        return f"{name} (zoom {self.zoom}, {self.tiles} tiles, {self.path.name})"

    def m_per_px(self, lat: float) -> float:
        return m_per_px(lat, self.zoom)

    def px(self, lat: float, lon: float) -> tuple[float, float]:
        return world_px(lat, lon, self.zoom)

    def latlon(self, x: float, y: float) -> tuple[float, float]:
        return latlon(x, y, self.zoom)

    def tile(self, x: int, y: int) -> np.ndarray | None:
        """Tile (x, y) of the map's zoom as a grey picture, or None when the file does not have it."""
        key = (int(x), int(y))
        with self._lock:
            if key in self._cache:
                self._cache.move_to_end(key)
                return self._cache[key]
            row = self._db.execute("SELECT tile_data FROM tiles WHERE zoom_level=? AND tile_column=? AND tile_row=?",
                                   (self.zoom, key[0], (1 << self.zoom) - 1 - key[1])).fetchone()
            grey = None
            if row is not None:
                grey = cv2.imdecode(np.frombuffer(row[0], np.uint8), cv2.IMREAD_GRAYSCALE)
                if grey is not None and grey.shape != (TILE, TILE):
                    grey = cv2.resize(grey, (TILE, TILE), interpolation=cv2.INTER_AREA)
            self._cache[key] = grey
            while len(self._cache) > self._cache_tiles:
                self._cache.popitem(last=False)
            return grey

    def crop(self, x0: int, y0: int, width: int, height: int) -> tuple[np.ndarray, np.ndarray]:
        """The map's pixels from world pixel (x0, y0), ``width`` by ``height``: (grey, mask), the mask
        255 where the map has a picture and 0 where it has none."""
        image = np.zeros((height, width), np.uint8)
        mask = np.zeros((height, width), np.uint8)
        for ty in range(math.floor(y0 / TILE), math.floor((y0 + height - 1) / TILE) + 1):
            for tx in range(math.floor(x0 / TILE), math.floor((x0 + width - 1) / TILE) + 1):
                grey = self.tile(tx, ty)
                if grey is None:
                    continue
                # The part of this tile inside the crop, in tile and in crop pixels.
                ax, ay = max(x0, tx * TILE), max(y0, ty * TILE)
                bx, by = min(x0 + width, (tx + 1) * TILE), min(y0 + height, (ty + 1) * TILE)
                image[ay - y0:by - y0, ax - x0:bx - x0] = grey[ay - ty * TILE:by - ty * TILE, ax - tx * TILE:bx - tx * TILE]
                mask[ay - y0:by - y0, ax - x0:bx - x0] = 255
        return image, mask

    def close(self) -> None:
        with self._lock:
            self._db.close()


class TileWriter:
    """Writes an MBTiles file: for a map made on a PC (and the test maps)."""

    def __init__(self, path: Path, name: str, fmt: str = "jpg", attribution: str = "", description: str = "") -> None:
        self.path = Path(path)
        tmp = self.path.with_suffix(self.path.suffix + ".part")
        tmp.unlink(missing_ok=True)
        self._tmp = tmp
        self._db = sqlite3.connect(tmp)
        self._db.executescript("CREATE TABLE metadata (name TEXT, value TEXT);"
                               "CREATE TABLE tiles (zoom_level INTEGER, tile_column INTEGER, tile_row INTEGER, tile_data BLOB);"
                               "CREATE UNIQUE INDEX tile_index ON tiles (zoom_level, tile_column, tile_row);")
        self.meta = {"name": name, "format": fmt, "type": "baselayer", "version": "1.0",
                     "attribution": attribution, "description": description}
        self.zooms: set[int] = set()
        self.box: list[float] | None = None          # west, south, east, north

    def put(self, zoom: int, x: int, y: int, data: bytes) -> None:
        self._db.execute("INSERT OR REPLACE INTO tiles VALUES (?, ?, ?, ?)", (zoom, x, (1 << zoom) - 1 - y, data))
        self.zooms.add(zoom)
        north, west = latlon(x * TILE, y * TILE, zoom)
        south, east = latlon((x + 1) * TILE, (y + 1) * TILE, zoom)
        b = self.box
        self.box = [west, south, east, north] if b is None else [min(b[0], west), min(b[1], south),
                                                                  max(b[2], east), max(b[3], north)]

    def close(self) -> None:
        meta = dict(self.meta)
        if self.zooms:
            meta.update(minzoom=str(min(self.zooms)), maxzoom=str(max(self.zooms)))
        if self.box:
            meta["bounds"] = ",".join(f"{v:.7f}" for v in self.box)
        self._db.executemany("INSERT INTO metadata VALUES (?, ?)", sorted(meta.items()))
        self._db.commit()
        self._db.close()
        self._tmp.replace(self.path)
