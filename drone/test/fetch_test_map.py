"""Download the free aerial photos the matching tests use, once, into map files (MBTiles).

The photos are the USDA's NAIP (National Agriculture Imagery Program): 0.6 m a pixel, public
domain, served as map tiles by Microsoft's Planetary Computer. The test area is east Davis,
California (a town, a motorway, fields and a creek), taken in two different years: one year is the
drone's map, the other is what the stand-in camera sees, so the two differ as a map and the real
ground do (season, light, new buildings, crops).

They are only for tests, not a map for the client's area, and they stay outside the repo:
    %LOCALAPPDATA%\\vama-test-maps   (or the folder in VAMA_TEST_MAPS)

Usage:
    py fetch_test_map.py            all the test maps that are not there yet (about 10 MB)
    py fetch_test_map.py --list     what they are and whether they are there
"""
from __future__ import annotations

import argparse
import concurrent.futures
import math
import os
import pathlib
import sys
import time
import urllib.error
import urllib.request

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "gps_denied"))

from vama_nav.geo import TILE, world_px  # noqa: E402
from vama_nav.map_tiles import TileWriter  # noqa: E402

TEST_MAP_DIR = pathlib.Path(os.environ.get("VAMA_TEST_MAPS")
                            or pathlib.Path(os.environ.get("LOCALAPPDATA") or pathlib.Path.home()) / "vama-test-maps")
BOX = (-121.730, 38.529, -121.704, 38.551)        # west, south, east, north
URL = ("https://planetarycomputer.microsoft.com/api/data/v1/item/tiles/WebMercatorQuad/{z}/{x}/{y}@1x.jpg"
       "?collection=naip&item={item}&assets=image&asset_bidx=image%7C1%2C2%2C3")
ATTRIBUTION = "USDA Farm Service Agency, NAIP (public domain), through Microsoft Planetary Computer"
# name: (NAIP item, zoom, what it is for)
SETS = {
    "davis-2020-z17": ("ca_m_3812127_sw_10_060_20200617", 17, "the drone's map (June 2020, about 0.93 m a pixel)"),
    "davis-2022-z18": ("ca_m_3812127_sw_10_060_20220520", 18, "the ground the stand-in camera sees (May 2022)"),
    "davis-2018-z17": ("ca_m_3812127_sw_10_060_20180715_20190209", 17, "another year, for the offline check (July 2018)"),
}


def path_of(name: str) -> pathlib.Path:
    return TEST_MAP_DIR / f"{name}.mbtiles"


def tiles_of(box: tuple[float, float, float, float], zoom: int) -> list[tuple[int, int]]:
    west, south, east, north = box
    x0, y0 = (math.floor(v / TILE) for v in world_px(north, west, zoom))
    x1, y1 = (math.floor(v / TILE) for v in world_px(south, east, zoom))
    return [(x, y) for y in range(y0, y1 + 1) for x in range(x0, x1 + 1)]


def _get(url: str) -> bytes:
    for attempt in range(4):
        try:
            with urllib.request.urlopen(url, timeout=60) as answer:
                data = answer.read()
            if data[:2] != b"\xff\xd8":
                raise ValueError("the answer is not a JPEG picture")
            return data
        except (urllib.error.URLError, TimeoutError, ValueError, ConnectionError):
            if attempt == 3:
                raise
            time.sleep(2.0 * (attempt + 1))
    raise AssertionError("not reached")


def fetch(name: str, out: pathlib.Path | None = None, box=BOX, get=_get, workers: int = 4) -> pathlib.Path:
    item, zoom, what = SETS[name]
    out = out or path_of(name)
    out.parent.mkdir(parents=True, exist_ok=True)
    tiles = tiles_of(box, zoom)
    writer = TileWriter(out, name, "jpg", ATTRIBUTION, f"NAIP {item}: {what}")
    with concurrent.futures.ThreadPoolExecutor(workers) as pool:
        jobs = {pool.submit(get, URL.format(z=zoom, x=x, y=y, item=item)): (x, y) for x, y in tiles}
        for done, job in enumerate(concurrent.futures.as_completed(jobs), 1):
            x, y = jobs[job]
            writer.put(zoom, x, y, job.result())
            if done % 50 == 0 or done == len(tiles):
                print(f"  {name}: {done} of {len(tiles)} tiles", flush=True)
    writer.close()
    return out


def main(argv: list[str]) -> int:
    p = argparse.ArgumentParser(description="Downloads the aerial photos for the matching tests (NAIP, public domain).")
    p.add_argument("--list", action="store_true", help="only say what the test maps are and whether they are there")
    p.add_argument("names", nargs="*", help=f"some of: {', '.join(SETS)} (default: all)")
    args = p.parse_args(argv)
    names = args.names or list(SETS)
    unknown = [n for n in names if n not in SETS]
    if unknown:
        print(f"unknown test map {unknown}, known: {', '.join(SETS)}")
        return 2
    for name in names:
        path = path_of(name)
        there = path.is_file()
        if args.list or there:
            print(f"{name}: {SETS[name][2]}, {'there' if there else 'not there'} ({path})")
            continue
        print(f"{name}: {SETS[name][2]}, downloading to {path}")
        fetch(name)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
