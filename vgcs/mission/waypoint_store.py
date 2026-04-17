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


def save_waypoints_json(path: str | Path, waypoints: list[Waypoint]) -> None:
    payload = {"version": 1, "waypoints": [asdict(wp) for wp in waypoints]}
    Path(path).write_text(json.dumps(payload, indent=2), encoding="utf-8")


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
            )
        )
    return out

