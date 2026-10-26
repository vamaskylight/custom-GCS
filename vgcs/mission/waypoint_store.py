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
    fence_nofly: list[dict] | None = None,
    fence_keepin: list[dict] | None = None,
    fence_union: bool = False,
    fence_return_point: tuple[float, float] | None = None,
) -> None:
    """Write the plan file. ``end_action`` (hold/rtl/land) is stored when given, and so is the
    polygon fence drawn on the map (``fence_polygon``: its corners in order) and its areas to
    stay out of (``fence_nofly``: {"polygon": [...]} or {"circle": [lat, lon], "radius_m": r}).
    So are its further areas to stay inside (``fence_keepin``, the same rows), whether the drone
    may be inside any of them (``fence_union``), and its return point (``fence_return_point``).

    Version 4 added the fence, version 5 the areas to stay out of, version 6 (2026-10-27) the
    areas to stay inside and the return point. Readers of older versions ignore what they do not
    know.
    """
    payload: dict[str, object] = {
        "version": 6,
        "waypoints": [asdict(wp) for wp in waypoints],
    }
    if end_action:
        payload["end_action"] = str(end_action)
    corners = [[float(lat), float(lon)] for lat, lon in (fence_polygon or [])]
    nofly = [dict(row) for row in (fence_nofly or []) if isinstance(row, dict)]
    keepin = [dict(row, inside=True) for row in (fence_keepin or []) if isinstance(row, dict)]
    if corners or nofly or keepin or fence_return_point is not None:
        fence: dict[str, object] = {"polygon": corners}
        if nofly:
            fence["nofly"] = nofly
        if keepin:
            fence["keepin"] = keepin
            fence["union"] = bool(fence_union)
        if fence_return_point is not None:
            fence["return_point"] = [float(fence_return_point[0]), float(fence_return_point[1])]
        payload["fence"] = fence
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


def load_fence_nofly(path: str | Path) -> list[dict] | None:
    """The areas to stay out of stored with a plan file (version 5), as the map takes them.

    None when the file holds none. Raises ValueError, naming the area, when one
    cannot be read.
    """
    from vgcs.mission.fence import check_nofly

    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    fence = raw.get("fence") if isinstance(raw, dict) else None
    rows = fence.get("nofly") if isinstance(fence, dict) else None
    if rows is None:
        return None
    if not isinstance(rows, list):
        raise ValueError("its areas to stay out of are not a list")
    areas, problem = check_nofly(rows)
    if problem:
        raise ValueError(problem[:1].lower() + problem[1:].rstrip("."))
    return [area.row() for area in areas]


def _fence_of(path: str | Path) -> dict | None:
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    fence = raw.get("fence") if isinstance(raw, dict) else None
    return fence if isinstance(fence, dict) else None


def load_fence_keepin(path: str | Path) -> list[dict] | None:
    """The further areas to stay inside stored with a plan file (version 6), as the map takes them.

    None when the file holds none. Raises ValueError, naming the area, when one cannot be read.
    """
    from vgcs.mission.fence import check_keepin

    fence = _fence_of(path)
    rows = fence.get("keepin") if fence is not None else None
    if rows is None:
        return None
    if not isinstance(rows, list):
        raise ValueError("its areas to stay inside are not a list")
    areas, problem = check_keepin(rows)
    if problem:
        raise ValueError(problem[:1].lower() + problem[1:].rstrip("."))
    return [area.row() for area in areas]


def load_fence_union(path: str | Path) -> bool | None:
    """Whether the drone may be inside any of the areas to stay inside (version 6). None when not stored."""
    fence = _fence_of(path)
    value = fence.get("union") if fence is not None else None
    return value if isinstance(value, bool) else None


def load_fence_return_point(path: str | Path) -> tuple[float, float] | None:
    """The return point stored with a plan file (version 6). None when there is none.

    Raises ValueError when the file holds one that is no position.
    """
    from vgcs.mission.fence import check_return_point

    fence = _fence_of(path)
    row = fence.get("return_point") if fence is not None else None
    if row is None:
        return None
    point, problem = check_return_point(row)
    if problem:
        raise ValueError("its return point is not a latitude and longitude")
    return point


def load_mission_end_action(path: str | Path) -> str | None:
    """End action stored alongside a plan file, or ``None`` for pre-v3 files."""
    try:
        raw = json.loads(Path(path).read_text(encoding="utf-8"))
    except Exception:
        return None
    value = raw.get("end_action") if isinstance(raw, dict) else None
    return str(value) if value else None

