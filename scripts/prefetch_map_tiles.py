#!/usr/bin/env python3
"""
Prefetch Esri satellite tiles into VGCS disk cache (for offline companion flights).

Example (run once with internet, at home):
  py scripts/prefetch_map_tiles.py --lat 20.4459777 --lon 72.8632065 --zoom 16 --radius 3

Tiles are stored under %USERPROFILE%\\.vgcs\\tile-cache (same path VGCS uses on the bench).
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

# Allow running from repo root without installing the package.
_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from vgcs.map import tile_disk_cache  # noqa: E402

_TEMPLATE = (
    "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
)


def _slippy_xy(lat: float, lon: float, z: int) -> tuple[int, int]:
    lat_rad = math.radians(max(-85.0511, min(85.0511, lat)))
    n = 2.0**z
    x = int((lon + 180.0) / 360.0 * n)
    y = int((1.0 - math.log(math.tan(lat_rad) + (1.0 / math.cos(lat_rad))) / math.pi) / 2.0 * n)
    return x, y


def main() -> int:
    ap = argparse.ArgumentParser(description="Prefetch map tiles into VGCS disk cache")
    ap.add_argument("--lat", type=float, required=True)
    ap.add_argument("--lon", type=float, required=True)
    ap.add_argument("--zoom", type=int, default=16)
    ap.add_argument("--radius", type=int, default=3, help="tile radius (3 => 7x7 grid)")
    args = ap.parse_args()
    cx, cy = _slippy_xy(args.lat, args.lon, args.zoom)
    r = max(0, int(args.radius))
    ok = fail = 0
    root = tile_disk_cache.default_cache_root()
    print(f"Cache root: {root}")
    for dx in range(-r, r + 1):
        for dy in range(-r, r + 1):
            x, y = cx + dx, cy + dy
            if x < 0 or y < 0:
                continue
            url = _TEMPLATE.replace("{z}", str(args.zoom)).replace("{x}", str(x)).replace(
                "{y}", str(y)
            )
            from vgcs.map.native_tile_map import _fetch_tile_http_or_file

            img, raw = _fetch_tile_http_or_file(url)
            if img.isNull() or not raw:
                fail += 1
                print(f"  miss z={args.zoom} x={x} y={y}")
                continue
            if tile_disk_cache.write_cached_tile_bytes(
                _TEMPLATE, args.zoom, x, y, raw, root=root
            ):
                ok += 1
                print(f"  ok   z={args.zoom} x={x} y={y}")
            else:
                fail += 1
    print(f"Done: {ok} cached, {fail} failed")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
