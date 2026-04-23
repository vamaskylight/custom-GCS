"""JSON persistence for mission waypoints."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path


@dataclass
class Waypoint:
    lat: float
    lon: float
    alt_m: float = 20.0
    speed_mps: float = 5.0


def save_waypoints_json(path: str | Path, waypoints: list[Waypoint]) -> None:
    payload = {"version": 2, "waypoints": [asdict(wp) for wp in waypoints]}
    Path(path).write_text(json.dumps(payload, indent=2), encoding="utf-8")


def save_waypoints_kml(path: str | Path, waypoints: list[Waypoint]) -> None:
    """Write a minimal KML path (LineString) for mission preview / GIS tools."""
    coords = []
    for wp in waypoints:
        coords.append(f"{wp.lon:.8f},{wp.lat:.8f},{wp.alt_m:.2f}")
    coord_text = " ".join(coords)
    doc = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>VGCS Mission</name>
    <Placemark>
      <name>Waypoints</name>
      <LineString>
        <coordinates>{coord_text}</coordinates>
      </LineString>
    </Placemark>
  </Document>
</kml>
"""
    Path(path).write_text(doc, encoding="utf-8")


def load_waypoints_json(path: str | Path) -> list[Waypoint]:
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    rows = raw.get("waypoints", [])
    out: list[Waypoint] = []
    for row in rows:
        out.append(
            Waypoint(
                lat=float(row["lat"]),
                lon=float(row["lon"]),
                alt_m=float(row.get("alt_m", 20.0)),
                speed_mps=float(row.get("speed_mps", 5.0)),
            )
        )
    return out

