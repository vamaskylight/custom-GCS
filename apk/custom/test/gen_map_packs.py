"""Makes the small map packs that map_pack_test reads (apk/custom/test/map_packs).

They are made by VGCS's own pack code (vgcs/map/tile_pack.py, download_tile_pack),
so the test reads what VGCS writes. Nothing is fetched: the pictures are drawn
here and handed to VGCS's writer in place of the internet. Each picture differs,
so a picture stored under the wrong tile would be seen.

Run from the repository root with a Python that has VGCS's packages:
    py -3.14 apk/custom/test/gen_map_packs.py
"""

from __future__ import annotations

import hashlib
import os
import shutil
import sys
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["VGCS_NO_NETWORK"] = "1"

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from PySide6.QtCore import QBuffer, QIODevice  # noqa: E402
from PySide6.QtGui import QColor, QGuiApplication, QImage  # noqa: E402

from vgcs.map.tile_pack import download_tile_pack, plan_pack_for_area  # noqa: E402

OUT = Path(__file__).resolve().parent / "map_packs"

# The client's place (field videos of 2026-10-14).
LAT = 20.4458
LON = 72.8634

ESRI_IMAGERY = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
ESRI_STREETS = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Street_Map/MapServer/tile/{z}/{y}/{x}"
OSM = "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"

PACKS = [
    # name, template, zooms, radius in km, picture format (Esri's imagery is JPEG, as on the internet)
    ("esri_satellite", ESRI_IMAGERY, (14, 15, 16), 0.4, "JPG"),
    ("esri_streets", ESRI_STREETS, (15,), 0.4, "PNG"),
    ("osm_streets", OSM, (15,), 0.4, "PNG"),
]


def picture(url: str, fmt: str) -> bytes:
    """A 256 point picture of its own colour, with a square placed by the address."""
    h = hashlib.sha256(url.encode("utf-8")).digest()
    img = QImage(256, 256, QImage.Format.Format_RGB32)
    img.fill(QColor(h[0], h[1], h[2]))
    x0, y0 = h[3] % 200, h[4] % 200
    for y in range(y0, y0 + 40):
        for x in range(x0, x0 + 40):
            img.setPixelColor(x, y, QColor(255 - h[0], 255 - h[1], 255 - h[2]))
    buf = QBuffer()
    buf.open(QIODevice.OpenModeFlag.WriteOnly)
    assert img.save(buf, fmt), fmt
    return bytes(buf.data())


def main() -> int:
    app = QGuiApplication.instance() or QGuiApplication([])  # noqa: F841 (image writers need it)
    if OUT.exists():
        shutil.rmtree(OUT)
    for name, template, zooms, radius, fmt in PACKS:
        plan = plan_pack_for_area(LAT, LON, radius_km=radius, template=template, zooms=zooms)
        result = download_tile_pack(
            plan,
            OUT / name,
            fetch=lambda url, fmt=fmt: picture(url, fmt),
            accept=lambda raw, z: True,
        )
        print(f"{name}: {plan.count} tiles, zoom {plan.zooms}, stored {result.stored}, failed {result.failed}")
        if result.failed or result.stored != plan.count:
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
