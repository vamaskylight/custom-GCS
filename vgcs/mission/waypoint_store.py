"""JSON persistence for mission waypoints."""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path


# ArduPilot stores a NAV_WAYPOINT hold time in a uint16, and a plan that parks
# the aircraft for longer than this is a typo rather than an intention.
MAX_WP_HOVER_S = 3600


def clamp_hover_seconds(raw: object) -> int:
    """Whole, non-negative seconds. Anything unreadable means no hover."""
    try:
        n = int(float(raw))
    except (TypeError, ValueError):
        return 0
    return max(0, min(MAX_WP_HOVER_S, n))


@dataclass
class Waypoint:
    lat: float
    lon: float
    alt_m: float = 20.0
    speed_mps: float = 5.0
    # Release the payload servo on arrival. Requested 2026-09-11: "i integrated
    # servo in our drone so if i plot the 5 waypoint ... suppose drone is
    # arrived point 1 then servo payload will drop".
    drop_payload: bool = False
    # Seconds to hold position on arrival before flying on. Requested
    # 2026-09-11: "before upload the mission i will set the hover time like 5s
    # or 10s that means drone will hover every point". Whole seconds, because
    # ArduPilot stores a NAV_WAYPOINT hold time as a uint16 of seconds.
    hover_s: int = 0


def save_waypoints_json(
    path: str | Path,
    waypoints: list[Waypoint],
    *,
    end_action: str | None = None,
    fence_polygon: list[tuple[float, float]] | None = None,
) -> None:
    """Write the plan file. ``end_action`` (hold/rtl/land) is stored when given, and so is the
    polygon fence drawn on the map (``fence_polygon``: its corners in order).

    Version 4 added the fence. Readers of older versions ignore it.
    """
    payload: dict[str, object] = {
        "version": 4,
        "waypoints": [asdict(wp) for wp in waypoints],
    }
    if end_action:
        payload["end_action"] = str(end_action)
    corners = [[float(lat), float(lon)] for lat, lon in (fence_polygon or [])]
    if corners:
        payload["fence"] = {"polygon": corners}
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
                # Absent in plans saved before payload drops existed, which
                # must load as "no drop" rather than dropping unexpectedly.
                drop_payload=bool(row.get("drop_payload", False)),
                # Likewise absent in older plans, which must load as "do not
                # hover" rather than stalling the mission at every point.
                hover_s=clamp_hover_seconds(row.get("hover_s", 0)),
            )
        )
    return out


def load_fence_polygon(path: str | Path) -> list[tuple[float, float]] | None:
    """The polygon fence stored with a plan file, its corners in order.

    None when the file holds no fence (plans saved before version 4, or without
    a fence). Raises ValueError when the file holds a fence that cannot be read.
    """
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    fence = raw.get("fence") if isinstance(raw, dict) else None
    if fence is None:
        return None
    rows = fence.get("polygon") if isinstance(fence, dict) else None
    if not isinstance(rows, list):
        raise ValueError("its fence has no list of corners")
    corners: list[tuple[float, float]] = []
    for n, row in enumerate(rows, start=1):
        try:
            lat, lon = float(row[0]), float(row[1])
        except (TypeError, ValueError, IndexError):
            raise ValueError(f"fence corner {n} is not a latitude and longitude") from None
        if not (math.isfinite(lat) and math.isfinite(lon) and -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            raise ValueError(f"fence corner {n} is not a latitude and longitude")
        corners.append((lat, lon))
    return corners


def load_mission_end_action(path: str | Path) -> str | None:
    """End action stored alongside a plan file, or ``None`` for pre-v3 files."""
    try:
        raw = json.loads(Path(path).read_text(encoding="utf-8"))
    except Exception:
        return None
    value = raw.get("end_action") if isinstance(raw, dict) else None
    return str(value) if value else None

