"""Native Qt slippy-map view (no WebEngine): raster tiles, vehicle, waypoints, pan/zoom."""

from __future__ import annotations

import json
import math
import random
import re
from pathlib import Path
from urllib.request import Request, urlopen

from vgcs.map import tile_disk_cache

from PySide6.QtCore import QObject, QPointF, QRectF, Qt, QRunnable, QThreadPool, QTimer, QUrl, Signal
from PySide6.QtGui import QColor, QImage, QMouseEvent, QPainter, QPainterPath, QPen, QWheelEvent
from PySide6.QtWidgets import QWidget


def _clamp_lat(lat: float) -> float:
    return max(-85.05112878, min(85.05112878, lat))


def _lon_to_x(lon: float, z: int) -> float:
    n = 2.0**z
    return (lon + 180.0) / 360.0 * n


def _lat_to_y(lat: float, z: int) -> float:
    lat = _clamp_lat(lat)
    lat_rad = math.radians(lat)
    n = 2.0**z
    return (1.0 - math.log(math.tan(lat_rad) + (1.0 / math.cos(lat_rad))) / math.pi) / 2.0 * n


def _tile_xy(lat: float, lon: float, z: int) -> tuple[int, int]:
    x = int(_lon_to_x(lon, z))
    y = int(_lat_to_y(lat, z))
    n = 1 << z
    return max(0, min(n - 1, x)), max(0, min(n - 1, y))


class _TileFetchSignals(QObject):
    loaded = Signal(int, int, int, object)  # z, x, y, QImage or None


class _TileFetchTask(QRunnable):
    def __init__(
        self,
        z: int,
        x: int,
        y: int,
        url: str,
        template: str,
        bridge: _TileFetchSignals,
        *,
        http_enabled: bool = True,
        cache_root: Path | None = None,
    ) -> None:
        super().__init__()
        self._z = z
        self._x = x
        self._y = y
        self._url = url
        self._template = str(template or "")
        self._bridge = bridge
        self._http_enabled = bool(http_enabled)
        self._cache_root = cache_root

    def run(self) -> None:
        img = QImage()
        is_local = self._template == "{local}"
        if tile_disk_cache.is_enabled() and self._template and not is_local:
            img = tile_disk_cache.read_cached_tile_any(
                self._template, self._z, self._x, self._y, root=self._cache_root
            )
        # Offline folders use file paths — must load even when HTTP is disabled.
        if img.isNull() and (self._http_enabled or is_local or not self._url.startswith(("http://", "https://"))):
            img, raw = _fetch_tile_http_or_file(self._url)
            if (
                not img.isNull()
                and raw
                and tile_disk_cache.is_enabled()
                and self._template
                and not is_local
            ):
                tile_disk_cache.write_cached_tile_bytes(
                    self._template, self._z, self._x, self._y, raw, root=self._cache_root
                )
        self._bridge.loaded.emit(self._z, self._x, self._y, img)


class NativeTileMapView(QWidget):
    """Minimal interactive map: HTTP tiles, drag pan, wheel zoom, markers."""

    user_waypoints_changed = Signal()
    user_fence_changed = Signal()
    observation_map_click = Signal(float, float)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_OpaquePaintEvent, True)
        self.setMouseTracking(True)
        # Waypoint removal uses right-click; suppress the empty Qt context menu on the map.
        self.setContextMenuPolicy(Qt.ContextMenuPolicy.NoContextMenu)
        self.setMinimumHeight(200)
        self._zoom = 16.0
        self._max_zoom = 19
        self._center_lat = 37.7749
        self._center_lon = -122.4194
        self._vehicle_lat: float | None = None
        self._vehicle_lon: float | None = None
        self._heading_deg = 0.0
        self._waypoints: list[tuple[float, float]] = []
        self._fence_points: list[tuple[float, float]] = []
        self._fence_circle: tuple[float, float, float] | None = None  # lat, lon, radius_m
        self._track: list[tuple[float, float]] = []
        self._mission_nav_seq = 0
        self._add_wp_mode = False
        self._fence_draw_mode = False
        self._tile_template = (
            "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
        )
        self._tile_subdomains = "abc"
        self._offline_root: str | None = None
        self._tiles: dict[tuple[int, int, int], QImage] = {}
        self._tiles_inflight: set[tuple[int, int, int]] = set()
        self._tile_bridge = _TileFetchSignals(self)
        self._tile_bridge.loaded.connect(self._on_tile_loaded)
        self._pool = QThreadPool.globalInstance()
        self._dragging = False
        self._drag_last: QPointF | None = None
        # Waypoint / fence: add only on release if the gesture was a tap (not a map pan drag).
        self._add_wp_press_candidate = False
        self._add_wp_press_geo: tuple[float, float] | None = None
        self._add_wp_press_screen: QPointF | None = None
        self._fence_press_candidate = False
        self._fence_press_geo: tuple[float, float] | None = None
        self._fence_press_screen: QPointF | None = None
        self._obs_mark_mode = False
        self._obs_click_candidate = False
        self._press_for_obs: QPointF | None = None
        # Stores lat/lon pairs for OBSERVE -> Target marks (native rendering).
        self._observation_marks: list[tuple[float, float]] = []
        # When HTTP tiles fail (no internet / firewall), draw slippy grid + HUD instead of empty gray.
        self._remote_tiles_enabled = True
        self._http_tile_failures = 0
        self._tile_cache_hits = 0
        self._tile_cache_root: Path | None = (
            tile_disk_cache.default_cache_root() if tile_disk_cache.is_enabled() else None
        )
        self._pending_timer = QTimer(self)
        self._pending_timer.setSingleShot(True)
        self._pending_timer.setInterval(80)
        self._pending_timer.timeout.connect(self.update)
        self.setStyleSheet("background-color: #1a1d24;")
        # >1.0 enlarges the vehicle chevron (used when the map is mirrored into the small video-swap card).
        self._vehicle_arrow_scale = 1.0

    def set_vehicle_arrow_scale(self, factor: float) -> None:
        """Scale the on-map vehicle icon (1.0 = default). Clamped for sane line widths."""
        try:
            self._vehicle_arrow_scale = max(1.0, min(2.6, float(factor)))
        except Exception:
            self._vehicle_arrow_scale = 1.0
        self.update()

    # --- map state ---
    def set_center(self, lat: float, lon: float) -> None:
        self._center_lat = float(lat)
        self._center_lon = float(lon)
        self.update()

    def center_on_vehicle(self) -> None:
        if self._vehicle_lat is not None and self._vehicle_lon is not None:
            self.set_center(self._vehicle_lat, self._vehicle_lon)

    def set_zoom(self, z: float) -> None:
        try:
            self._zoom = max(3.0, min(float(self._max_zoom), float(z)))
        except Exception:
            pass
        self.update()

    def set_vehicle(self, lat: float, lon: float) -> None:
        self._vehicle_lat = float(lat)
        self._vehicle_lon = float(lon)
        self._track.append((self._vehicle_lat, self._vehicle_lon))
        if len(self._track) > 900:
            self._track = self._track[-600:]
        self.update()

    def set_heading(self, deg: float) -> None:
        try:
            self._heading_deg = float(deg) % 360.0
        except Exception:
            self._heading_deg = 0.0
        self.update()

    def clear_track(self) -> None:
        self._track.clear()
        self.update()

    def set_waypoint_rows(self, rows: list[list[float]]) -> None:
        self._waypoints = []
        for row in rows:
            if isinstance(row, list) and len(row) >= 2:
                try:
                    self._waypoints.append((float(row[0]), float(row[1])))
                except Exception:
                    continue
        self.update()

    def clear_waypoints(self) -> None:
        self._waypoints.clear()
        # Match legacy `clearWaypoints()` — stale MISSION_CURRENT.seq would otherwise trim the polyline wrong.
        self._mission_nav_seq = 0
        self.update()

    def clear_fence_points(self) -> None:
        self._fence_points.clear()
        self.update()

    def disable_plan_edit_modes(self) -> None:
        """Match legacy JS `disablePlanEditModes`: turn off waypoint/fence placement modes."""
        self._clear_pending_plan_map_taps()
        self._add_wp_mode = False
        self._fence_draw_mode = False
        if 0 < len(self._fence_points) < 3:
            self.clear_fence_points()
        self.update()

    def _clear_pending_plan_map_taps(self) -> None:
        """Drop a tap-in-progress (e.g. mode switch, right-click) without placing a point."""
        self._add_wp_press_candidate = False
        self._add_wp_press_geo = None
        self._add_wp_press_screen = None
        self._fence_press_candidate = False
        self._fence_press_geo = None
        self._fence_press_screen = None

    def clear_fence_visual(self) -> None:
        self._fence_points.clear()
        self._fence_circle = None
        self.update()

    def set_mission_nav_seq(self, seq: int) -> None:
        try:
            self._mission_nav_seq = max(0, int(seq))
        except Exception:
            self._mission_nav_seq = 0
        self.update()

    def set_remote_tiles_enabled(self, enabled: bool) -> None:
        """Stop HTTP tile fetches when the network cannot reach tile servers (URLError).

        Disk cache reads continue so previously viewed areas still show imagery offline.
        """
        on = bool(enabled)
        if on == bool(getattr(self, "_remote_tiles_enabled", True)):
            return
        self._remote_tiles_enabled = on
        if on:
            self._http_tile_failures = 0
        else:
            self._tiles_inflight.clear()
        self._tiles.clear()
        self.update()

    def set_offline_tile_root(self, root: str | Path) -> None:
        """Load tiles from ``root/z/x/y.png`` (no HTTP)."""
        p = Path(root).expanduser().resolve()
        if not p.is_dir():
            return
        self._offline_root = str(p)
        self._tile_template = "{local}"
        self._remote_tiles_enabled = False
        self._http_tile_failures = 0
        self._tiles_inflight.clear()
        self._tiles.clear()
        self._preload_local_tiles_around_center()
        self.update()

    def set_tile_source(self, template: str, _attribution: str, max_z: int) -> None:
        t = str(template or "").strip()
        if not t:
            return
        self._offline_root = None
        if t.startswith("file:"):
            path = QUrl(t).toLocalFile()
            if path:
                self.set_offline_tile_root(path)
            else:
                self._tile_template = t
        else:
            if t != "{local}":
                self._remote_tiles_enabled = True
                self._http_tile_failures = 0
            self._tile_template = t
        try:
            self._max_zoom = max(3, min(22, int(max_z)))
        except Exception:
            self._max_zoom = 19
        if self._tile_template != "{local}":
            self._tiles.clear()
            self.update()

    def set_low_spec(self, _on: bool) -> None:
        self.update()

    def set_observation_mark_mode(self, on: bool) -> None:
        """When True, a short click (no drag) on the map emits ``observation_map_click``."""
        self._obs_mark_mode = bool(on)
        if not self._obs_mark_mode:
            self._obs_click_candidate = False
            self._press_for_obs = None

    def add_observation_map_marker(self, lat: float, lon: float) -> None:
        """Add a visible target marker to the native (Qt) map."""
        try:
            self._observation_marks.append((float(lat), float(lon)))
        except Exception:
            return
        # Keep memory bounded if user spams clicks.
        if len(self._observation_marks) > 2000:
            self._observation_marks = self._observation_marks[-2000:]
        self.update()

    def clear_observation_marks(self) -> None:
        """Clear all visible OBSERVE -> Target markers."""
        try:
            self._observation_marks.clear()
        except Exception:
            pass
        self.update()

    def set_link_connected(self, _on: bool) -> None:
        self.update()

    def get_map_view_dict(self) -> dict[str, object]:
        return {
            "z": int(self._zoom),
            "lat": float(self._center_lat),
            "lng": float(self._center_lon),
            "template": str(self._tile_template or ""),
        }

    def waypoint_count(self) -> int:
        return len(self._waypoints)

    def waypoints_json(self) -> str:
        return json.dumps([[a, b] for a, b in self._waypoints])

    def fence_points_json(self) -> str:
        return json.dumps([[a, b] for a, b in self._fence_points])

    # --- JS compatibility (best-effort string dispatch) ---
    def eval_script(self, script: str) -> None:
        """Execute supported fragments from legacy Leaflet JS bridge (side effects only)."""
        # Split rough statements
        for part in re.split(r";\s*", script):
            p = part.strip()
            if not p:
                continue
            self._eval_one(p)

    def eval_script_with_callback(self, script: str, callback) -> bool:
        """If script matches a getter, invoke callback with value and return True."""
        s = script.strip().rstrip(";").strip()
        if s == "getWaypointCount()" or s == "getWaypointCount":
            try:
                callback(self.waypoint_count())
            except Exception:
                pass
            return True
        if "JSON.stringify(getWaypoints())" in s or s.endswith("getWaypoints())"):
            try:
                callback(self.waypoints_json())
            except Exception:
                pass
            return True
        if "JSON.stringify(getFencePolygon())" in s or "getFencePolygon())" in s:
            try:
                callback(self.fence_points_json())
            except Exception:
                pass
            return True
        if "JSON.stringify(getFencePoints())" in s or "getFencePoints())" in s:
            try:
                callback(self.fence_points_json())
            except Exception:
                pass
            return True
        if s.startswith("Date.now") or s == "Date.now()":
            try:
                import time as _t

                callback(int(_t.time() * 1000))
            except Exception:
                callback(0)
            return True
        if "window.__vgcsGetMapView" in s or "__vgcsGetMapView" in s:
            try:
                callback(json.dumps(self.get_map_view_dict()))
            except Exception:
                callback("{}")
            return True
        if "window.__vgcsGetOverlayInsets" in s:
            try:
                callback(json.dumps({"left": 170, "top": 58, "right": 220, "bottom": 130}))
            except Exception:
                callback("")
            return True
        if "window.__lastTileTemplate" in s:
            try:
                callback(str(self._tile_template or ""))
            except Exception:
                callback("")
            return True
        self.eval_script(script)
        return False

    def _eval_one(self, p: str) -> None:
        if not p or p.startswith("//"):
            return
        if "document.title" in p or p.strip().startswith("if (window."):
            return
        if "disablePlanEditModes" in p:
            self.disable_plan_edit_modes()
            return
        _skip_if = (
            "setVideoPreview",
            "setVideoSwapMode",
            "setNativeVideoOverlayMode",
            "setNativeHudMode",
            "clearAiOverlays",
            "clearVideoPreviewGrid",
            "setFlightStatus",
            "setHeader",
            "setTelemetryOverlay",
            "setPlan",
            "applyPlan",
            "setPlanRailTool",
            "setPlanFlight",
            "set3DEnabled",
            "setFencePolygon",
            "setObservationMarkMode",
            "clearObservationMarks",
            "addObservationMapMarker",
            "setAiOverlay",
        )
        if any(tok in p for tok in _skip_if):
            return
        if p.startswith("setVehicle("):
            m = re.match(r"setVehicle\(\s*([+-]?\d+\.?\d*)\s*,\s*([+-]?\d+\.?\d*)\s*\)", p)
            if m:
                self.set_vehicle(float(m.group(1)), float(m.group(2)))
            return
        if p.startswith("updateHeading("):
            m = re.match(r"updateHeading\(\s*([+-]?\d+\.?\d*)", p)
            if m:
                self.set_heading(float(m.group(1)))
            return
        if p.strip() == "centerOnVehicle()" or p.startswith("centerOnVehicle("):
            self.center_on_vehicle()
            return
        if "clearFlightTrack()" in p:
            self.clear_track()
            return
        m_wp = re.search(r"setWaypoints\s*\(\s*(\[[\s\S]*\])\s*\)", p)
        if m_wp:
            try:
                rows = json.loads(m_wp.group(1))
                if isinstance(rows, list):
                    self.set_waypoint_rows(rows)
            except Exception:
                pass
            return
        if p.startswith("enableAddWaypoint"):
            self._clear_pending_plan_map_taps()
            self._add_wp_mode = True
            self._fence_draw_mode = False
            return
        if p.startswith("enableFencePolygon"):
            self._clear_pending_plan_map_taps()
            self._fence_draw_mode = True
            self._add_wp_mode = False
            self.clear_fence_points()
            return
        if p.startswith("clearWaypoints"):
            self.clear_waypoints()
            return
        if "updateMissionRoutePolyline" in p or "window.__missionNavSeq" in p:
            m = re.search(r"__missionNavSeq\s*=\s*(\d+)", p)
            if m:
                self.set_mission_nav_seq(int(m.group(1)))
            return
        if p.startswith("setTileSource("):
            m = re.search(
                r"setTileSource\s*\(\s*(['\"])(.*?)\1\s*,\s*(['\"])(.*?)\3\s*,\s*(\d+)",
                p,
            )
            if m:
                tmpl = str(m.group(2) or "").strip()
                try:
                    max_z = int(m.group(5) or 19)
                except Exception:
                    max_z = 19
                self.set_tile_source(tmpl, "", max_z)
            else:
                args = _split_js_args(_extract_paren_payload(p, "setTileSource") or "")
                if len(args) >= 1:
                    tmpl = args[0].strip().strip("'").strip('"')
                    max_z = 19
                    if len(args) >= 3:
                        try:
                            max_z = int(float(args[2].strip()))
                        except Exception:
                            pass
                    self.set_tile_source(tmpl, "", max_z)
            return
        if p.startswith("clearFence"):
            self.clear_fence_visual()
            return
        if p.startswith("setFence("):
            m = re.match(
                r"setFence\(\s*([+-]?\d+\.?\d*)\s*,\s*([+-]?\d+\.?\d*)\s*,\s*([+-]?\d+\.?\d*)\s*\)",
                p,
            )
            if m:
                try:
                    self._fence_circle = (float(m.group(1)), float(m.group(2)), float(m.group(3)))
                except Exception:
                    self._fence_circle = None
                self.update()
            return
        if p.startswith("setLowSpecMode"):
            self.set_low_spec("true" in p.lower())
            return

    # --- painting ---
    def paintEvent(self, event) -> None:  # noqa: N802
        del event
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(26, 29, 36))
        w, h = self.width(), self.height()
        if w <= 1 or h <= 1:
            painter.end()
            return
        z = int(max(3, min(self._max_zoom, round(self._zoom))))
        tile_size = 256
        center_x = _lon_to_x(self._center_lon, z)
        center_y = _lat_to_y(self._center_lat, z)
        fx = center_x * tile_size - w / 2.0
        fy = center_y * tile_size - h / 2.0
        x0 = int(math.floor(fx / tile_size)) - 1
        y0 = int(math.floor(fy / tile_size)) - 1
        x1 = int(math.ceil((fx + w) / tile_size)) + 1
        y1 = int(math.ceil((fy + h) / tile_size)) + 1
        n = 1 << z
        self._paint_slippy_grid(painter, z, fx, fy, w, h, x0, y0, x1, y1)
        tiles_drawn = 0
        for tx in range(x0, x1 + 1):
            for ty in range(y0, y1 + 1):
                if tx < 0 or ty < 0 or tx >= n or ty >= n:
                    continue
                img = self._tile_image(z, tx, ty)
                if img is None or img.isNull():
                    continue
                px = int(tx * tile_size - fx)
                py = int(ty * tile_size - fy)
                painter.drawImage(px, py, img.scaled(tile_size, tile_size))
                tiles_drawn += 1

        if tiles_drawn == 0 and not self._remote_tiles_enabled:
            self._paint_offline_map_hint(
                painter,
                w,
                h,
                cache_enabled=tile_disk_cache.is_enabled(),
            )

        # Route polyline — full planned path on the map: vehicle → WP1 → WP2 → …
        # We intentionally do **not** trim from MISSION_CURRENT here: real missions vary
        # (extra DO_* items, different ordering), so mapping seq → waypoint index often skipped
        # WP1 while planning or after upload, which looked like a broken branch.
        rest = self._waypoints
        veh_ok = self._vehicle_lat is not None and self._vehicle_lon is not None
        route_pts: list[tuple[float, float]] = []
        if veh_ok and len(rest) >= 1:
            route_pts = [(float(self._vehicle_lat), float(self._vehicle_lon))] + list(rest)
        elif len(rest) >= 2:
            route_pts = list(rest)
        elif veh_ok and len(rest) == 1:
            route_pts = [
                (float(self._vehicle_lat), float(self._vehicle_lon)),
                rest[0],
            ]

        if len(route_pts) >= 2:
            pen = QPen(QColor(255, 199, 44, 220))
            pen.setWidth(3)
            painter.setPen(pen)
            prev: QPointF | None = None
            for lat, lon in route_pts:
                pt = self._project(lat, lon, z, fx, fy, w, h)
                if prev is not None:
                    painter.drawLine(prev, pt)
                prev = pt

        # Track
        if len(self._track) >= 2:
            pen = QPen(QColor(255, 120, 40, 180))
            pen.setWidth(2)
            painter.setPen(pen)
            prev = None
            for lat, lon in self._track[-400:]:
                pt = self._project(lat, lon, z, fx, fy, w, h)
                if prev is not None:
                    painter.drawLine(prev, pt)
                prev = pt

        # Circular fence preview (from setFence)
        if self._fence_circle is not None:
            flat_lat, flat_lon, r_m = self._fence_circle
            try:
                pts = _circle_ring_points(flat_lat, flat_lon, float(r_m), segments=28)
                pen = QPen(QColor(120, 255, 160, 210))
                pen.setWidth(2)
                painter.setPen(pen)
                prev = None
                for la, lo in pts:
                    pt = self._project(la, lo, z, fx, fy, w, h)
                    if prev is not None:
                        painter.drawLine(prev, pt)
                    prev = pt
            except Exception:
                pass

        # Fence polygon
        if len(self._fence_points) >= 2:
            pen = QPen(QColor(66, 200, 255, 200))
            pen.setWidth(2)
            painter.setPen(pen)
            pts = [self._project(a, b, z, fx, fy, w, h) for a, b in self._fence_points]
            for i in range(1, len(pts)):
                painter.drawLine(pts[i - 1], pts[i])
            if len(pts) >= 3:
                painter.drawLine(pts[-1], pts[0])

        # Waypoint markers
        painter.setPen(QPen(QColor(255, 255, 255), 1))
        for i, (lat, lon) in enumerate(self._waypoints):
            c = self._project(lat, lon, z, fx, fy, w, h)
            painter.setBrush(QColor(40, 130, 255, 220))
            painter.drawEllipse(c, 7, 7)
            painter.setPen(QPen(QColor(255, 255, 255)))
            painter.drawText(QRectF(c.x() - 16, c.y() - 8, 32, 16), Qt.AlignmentFlag.AlignCenter, str(i + 1))

        # Observation marks (OBSERVE -> Target)
        if self._observation_marks:
            for lat, lon in self._observation_marks:
                c = self._project(lat, lon, z, fx, fy, w, h)
                # Yellow crosshair + dot.
                painter.setPen(QPen(QColor(255, 210, 70, 230), 2))
                painter.setBrush(QColor(255, 210, 70, 210))
                painter.drawEllipse(c, 5, 5)
                painter.drawLine(int(c.x() - 10), int(c.y()), int(c.x() + 10), int(c.y()))
                painter.drawLine(int(c.x()), int(c.y() - 10), int(c.x()), int(c.y() + 10))

        # Vehicle — navigation-style arrow (matches compass HUD: sharp tip, concave base,
        # thick white outline, red fill, center dot). Heading=0 points toward -Y.
        if self._vehicle_lat is not None and self._vehicle_lon is not None:
            c = self._project(self._vehicle_lat, self._vehicle_lon, z, fx, fy, w, h)
            painter.translate(c)
            painter.rotate(self._heading_deg)
            try:
                painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
            except Exception:
                pass

            vb = float(getattr(self, "_vehicle_arrow_scale", 1.0))
            tip_y = -28.0 * vb
            base_y = 15.0 * vb
            half_w = 12.0 * vb
            # Control pulls the base upward in the middle → concave “inward” base.
            ctrl_x, ctrl_y = 0.0, 5.0 * vb

            arrow = QPainterPath()
            arrow.moveTo(0.0, tip_y)
            arrow.lineTo(-half_w, base_y)
            arrow.quadTo(ctrl_x, ctrl_y, half_w, base_y)
            arrow.closeSubpath()

            outline_white = QColor(255, 255, 255, 255)
            fill_red = QColor(240, 44, 52, 255)
            painter.setBrush(fill_red)
            pen_w = max(2.0, min(6.5, 3.0 * vb))
            painter.setPen(
                QPen(
                    outline_white,
                    pen_w,
                    Qt.PenStyle.SolidLine,
                    Qt.PenCapStyle.RoundCap,
                    Qt.PenJoinStyle.RoundJoin,
                )
            )
            painter.drawPath(arrow)

            # Center dot (like compass needle body).
            dot_r = max(2.4, 2.4 * vb)
            painter.setPen(QPen(QColor(255, 255, 255, 255), 1))
            painter.setBrush(QColor(255, 255, 255, 255))
            painter.drawEllipse(QPointF(0.0, 1.0 * vb), dot_r, dot_r)

            painter.resetTransform()

        painter.end()

    def _paint_slippy_grid(
        self,
        painter: QPainter,
        z: int,
        fx: float,
        fy: float,
        w: int,
        h: int,
        x0: int,
        y0: int,
        x1: int,
        y1: int,
    ) -> None:
        """Tile-boundary grid visible when satellite/OSM tiles are missing (companion Wi‑Fi, no WAN)."""
        tile_size = 256
        pen = QPen(QColor(52, 72, 104, 200), 1)
        painter.setPen(pen)
        for tx in range(x0, x1 + 2):
            px = int(tx * tile_size - fx)
            if -tile_size <= px <= w + tile_size:
                painter.drawLine(px, 0, px, h)
        for ty in range(y0, y1 + 2):
            py = int(ty * tile_size - fy)
            if -tile_size <= py <= h + tile_size:
                painter.drawLine(0, py, w, py)

    def _paint_offline_map_hint(
        self, painter: QPainter, w: int, h: int, *, cache_enabled: bool = False
    ) -> None:
        painter.setPen(QColor(180, 195, 220, 220))
        font = painter.font()
        font.setPointSize(10)
        painter.setFont(font)
        painter.drawText(12, 22, "Map tiles offline — grid + GPS")
        if cache_enabled:
            painter.drawText(
                12,
                40,
                "Pan over a cached area or fly once online to fill ~/.vgcs/tile-cache",
            )
        else:
            painter.drawText(12, 40, "Use Offline Tiles… or set VGCS_MAP_TILE_CACHE=1")

    def _project(self, lat: float, lon: float, z: int, fx: float, fy: float, w: int, h: int) -> QPointF:
        px = _lon_to_x(lon, z) * 256.0 - fx
        py = _lat_to_y(lat, z) * 256.0 - fy
        return QPointF(px, py)

    def _view_frame(self) -> tuple[int, float, float, int, int] | None:
        w, h = self.width(), self.height()
        if w <= 1 or h <= 1:
            return None
        z = int(max(3, min(self._max_zoom, round(self._zoom))))
        tile_size = 256
        center_x = _lon_to_x(self._center_lon, z)
        center_y = _lat_to_y(self._center_lat, z)
        fx = center_x * tile_size - w / 2.0
        fy = center_y * tile_size - h / 2.0
        return z, fx, fy, w, h

    def _waypoint_hit_index(self, pos: QPointF) -> int | None:
        """Screen hit-test for numbered waypoint discs (legacy: marker dblclick / contextmenu)."""
        vf = self._view_frame()
        if vf is None or not self._waypoints:
            return None
        z, fx, fy, w, h = vf
        hit_r2 = 20.0 * 20.0
        best_i: int | None = None
        best_d2 = hit_r2 + 1.0
        for i, (lat, lon) in enumerate(self._waypoints):
            c = self._project(lat, lon, z, fx, fy, w, h)
            dx = float(pos.x() - c.x())
            dy = float(pos.y() - c.y())
            d2 = dx * dx + dy * dy
            if d2 <= hit_r2 and d2 < best_d2:
                best_d2 = d2
                best_i = i
        return best_i

    def _remove_waypoint_index(self, idx: int) -> None:
        if 0 <= idx < len(self._waypoints):
            self._waypoints.pop(idx)
            # Editing the plan invalidates raw MISSION_CURRENT.seq until the next downlink.
            self._mission_nav_seq = 0
            self.update()
            self.user_waypoints_changed.emit()

    def _tile_url(self, z: int, x: int, y: int) -> str:
        tmpl = self._tile_template
        if tmpl == "{local}" and self._offline_root:
            return str(Path(self._offline_root) / str(z) / str(x) / f"{y}.png")
        if "{s}" in tmpl:
            s = random.choice(self._tile_subdomains)
            return (
                tmpl.replace("{s}", s)
                .replace("{z}", str(z))
                .replace("{x}", str(x))
                .replace("{y}", str(y))
            )
        return tmpl.replace("{z}", str(z)).replace("{x}", str(x)).replace("{y}", str(y))

    def _on_tile_loaded(self, z: int, x: int, y: int, img: object) -> None:
        key = (z, x, y)
        try:
            self._tiles_inflight.discard(key)
        except Exception:
            pass
        if img is None or not isinstance(img, QImage) or img.isNull():
            self._tiles.pop(key, None)
            if self._remote_tiles_enabled and self._tile_template != "{local}":
                self._http_tile_failures += 1
                if self._http_tile_failures >= 6:
                    self.set_remote_tiles_enabled(False)
        else:
            self._tiles[key] = img
            if self._http_tile_failures > 0:
                self._http_tile_failures = max(0, self._http_tile_failures - 1)
        self.update()

    def _read_disk_cache_sync(self, z: int, x: int, y: int) -> QImage | None:
        if not tile_disk_cache.is_enabled() or self._tile_template in ("", "{local}"):
            return None
        img = tile_disk_cache.read_cached_tile_any(
            self._tile_template, z, x, y, root=self._tile_cache_root
        )
        if img.isNull():
            return None
        self._tile_cache_hits += 1
        if self._tile_cache_hits == 1:
            try:
                print(
                    f"[VGCS:map] tile disk cache active "
                    f"({tile_disk_cache.default_cache_root()})"
                )
            except Exception:
                pass
        return img

    def reload_visible_tiles_from_cache(self) -> int:
        """Drop in-memory tiles and repaint from disk cache or offline folder."""
        self._tiles_inflight.clear()
        self._tiles.clear()
        self.update()
        if self._tile_template == "{local}" and self._offline_root:
            return self._reload_visible_local_tiles()
        if not tile_disk_cache.is_enabled():
            return 0
        vf = self._view_frame()
        if vf is None:
            return 0
        z, fx, fy, w, h = vf
        tile_size = 256
        x0 = int(math.floor(fx / tile_size)) - 1
        y0 = int(math.floor(fy / tile_size)) - 1
        x1 = int(math.ceil((fx + w) / tile_size)) + 1
        y1 = int(math.ceil((fy + h) / tile_size)) + 1
        hits = 0
        for tx in range(x0, x1 + 1):
            for ty in range(y0, y1 + 1):
                if self._read_disk_cache_sync(z, tx, ty) is not None:
                    hits += 1
        if hits:
            self.update()
        return hits

    def _read_local_tile_sync(self, z: int, x: int, y: int) -> QImage | None:
        root = self._offline_root
        if not root or self._tile_template != "{local}":
            return None
        p = Path(root) / str(z) / str(x) / f"{y}.png"
        if not p.is_file():
            return None
        img = QImage(str(p))
        if img.isNull():
            return None
        return img

    def _preload_local_tiles_around_center(self, *, radius: int = 4) -> int:
        """Load offline tiles around map center (works before the widget has a final size)."""
        z = int(max(3, min(self._max_zoom, round(self._zoom))))
        cx, cy = _tile_xy(self._center_lat, self._center_lon, z)
        hits = 0
        r = max(1, int(radius))
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                img = self._read_local_tile_sync(z, cx + dx, cy + dy)
                if img is None:
                    continue
                self._tiles[(z, cx + dx, cy + dy)] = img
                hits += 1
        if hits:
            self.update()
        return hits

    def _reload_visible_local_tiles(self) -> int:
        root = self._offline_root
        if not root:
            return 0
        vf = self._view_frame()
        if vf is None:
            return self._preload_local_tiles_around_center()
        z, fx, fy, w, h = vf
        tile_size = 256
        x0 = int(math.floor(fx / tile_size)) - 1
        y0 = int(math.floor(fy / tile_size)) - 1
        x1 = int(math.ceil((fx + w) / tile_size)) + 1
        y1 = int(math.ceil((fy + h) / tile_size)) + 1
        hits = 0
        for tx in range(x0, x1 + 1):
            for ty in range(y0, y1 + 1):
                img = self._read_local_tile_sync(z, tx, ty)
                if img is None:
                    continue
                self._tiles[(z, tx, ty)] = img
                hits += 1
        if hits:
            self.update()
        return hits

    def _tile_image(self, z: int, x: int, y: int) -> QImage | None:
        key = (z, x, y)
        if key in self._tiles:
            im = self._tiles[key]
            if not im.isNull():
                return im
            self._tiles.pop(key, None)
        cached = self._read_disk_cache_sync(z, x, y)
        if cached is not None:
            self._tiles[key] = cached
            return cached
        is_local = self._tile_template == "{local}"
        if is_local:
            local = self._read_local_tile_sync(z, x, y)
            if local is not None:
                self._tiles[key] = local
                return local
        http_on = bool(self._remote_tiles_enabled)
        if not http_on and not is_local and not tile_disk_cache.is_enabled():
            return None
        if key not in self._tiles_inflight:
            self._tiles_inflight.add(key)
            url = self._tile_url(z, x, y)
            try:
                self._pool.start(
                    _TileFetchTask(
                        z,
                        x,
                        y,
                        url,
                        self._tile_template,
                        self._tile_bridge,
                        http_enabled=http_on,
                        cache_root=self._tile_cache_root,
                    )
                )
            except Exception:
                self._tiles_inflight.discard(key)
        return None

    def showEvent(self, event) -> None:  # noqa: N802
        super().showEvent(event)
        if self._tile_template == "{local}" and self._offline_root:
            QTimer.singleShot(0, self._reload_visible_local_tiles)

    def resizeEvent(self, event) -> None:  # noqa: N802
        super().resizeEvent(event)
        self._tiles_inflight.clear()
        if self._tile_template == "{local}" and self._offline_root:
            if self.width() > 1 and self.height() > 1:
                self._reload_visible_local_tiles()
        else:
            self._tiles.clear()

    # --- input ---
    def nudge_center_by_pixels(self, dx: float, dy: float) -> None:
        """Pan the map by ``(dx, dy)`` screen pixels (same math as drag-pan in ``mouseMoveEvent``)."""
        if dx == 0.0 and dy == 0.0:
            return
        z = int(max(3, min(self._max_zoom, round(self._zoom))))
        wx = _lon_to_x(self._center_lon, z) * 256.0 - dx
        wy = _lat_to_y(self._center_lat, z) * 256.0 - dy
        n = 256.0 * (2.0**z)
        self._center_lon = (wx / n) * 360.0 - 180.0
        lat_rad = math.atan(math.sinh(math.pi * (1.0 - 2.0 * wy / n)))
        self._center_lat = _clamp_lat(math.degrees(lat_rad))
        self.update()

    def wheelEvent(self, event: QWheelEvent) -> None:
        delta = float(event.angleDelta().y())
        if delta == 0.0:
            return
        # ~1 zoom level per standard wheel detent (120°). The map paints at ``round(_zoom)`` tile z,
        # so the old fixed 0.45 step often left several notches with no visible change.
        step = (delta / 120.0) * 1.0
        self._zoom = max(3.0, min(float(self._max_zoom), self._zoom + step))
        self.update()

    def mouseDoubleClickEvent(self, event: QMouseEvent) -> None:  # noqa: N802 (Qt naming)
        if event.button() == Qt.MouseButton.LeftButton:
            idx = self._waypoint_hit_index(QPointF(event.position()))
            if idx is not None:
                self._remove_waypoint_index(idx)
                event.accept()
                return
        super().mouseDoubleClickEvent(event)

    def mousePressEvent(self, event: QMouseEvent) -> None:
        pos = QPointF(event.position())
        if event.button() == Qt.MouseButton.RightButton:
            self._clear_pending_plan_map_taps()
            idx = self._waypoint_hit_index(pos)
            if idx is not None:
                self._remove_waypoint_index(idx)
                event.accept()
                return
            super().mousePressEvent(event)
            return

        if event.button() == Qt.MouseButton.LeftButton:
            # clicks for waypoint / fence (unchanged; takes priority over observation mark)
            latlon = self._screen_to_lat_lon(pos)
            if latlon is not None:
                lat, lon = latlon
                if self._add_wp_mode:
                    hit = self._waypoint_hit_index(pos)
                    if hit is not None:
                        # Avoid stacking a new WP on top of an existing marker (Leaflet markers were non-bubbling).
                        super().mousePressEvent(event)
                        return
                    # Defer placement to release — small movement starts a pan instead (same idea as OBSERVE marks).
                    self._add_wp_press_candidate = True
                    self._add_wp_press_geo = (lat, lon)
                    self._add_wp_press_screen = QPointF(pos)
                    super().mousePressEvent(event)
                    return
                if self._fence_draw_mode:
                    self._fence_press_candidate = True
                    self._fence_press_geo = (lat, lon)
                    self._fence_press_screen = QPointF(pos)
                    super().mousePressEvent(event)
                    return
            if self._obs_mark_mode:
                self._obs_click_candidate = True
                self._press_for_obs = pos
                self._dragging = False
                self._drag_last = pos
                super().mousePressEvent(event)
                return
            self._dragging = True
            self._drag_last = pos
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        cur = QPointF(event.position())
        if self._obs_click_candidate and self._press_for_obs is not None:
            if (cur - self._press_for_obs).manhattanLength() > 10.0:
                self._obs_click_candidate = False
                self._dragging = True
                self._drag_last = cur
        if self._add_wp_press_candidate and self._add_wp_press_screen is not None:
            if (cur - self._add_wp_press_screen).manhattanLength() > 10.0:
                self._add_wp_press_candidate = False
                self._add_wp_press_geo = None
                self._add_wp_press_screen = None
                self._dragging = True
                self._drag_last = cur
        if self._fence_press_candidate and self._fence_press_screen is not None:
            if (cur - self._fence_press_screen).manhattanLength() > 10.0:
                self._fence_press_candidate = False
                self._fence_press_geo = None
                self._fence_press_screen = None
                self._dragging = True
                self._drag_last = cur
        if self._dragging and self._drag_last is not None:
            cur2 = QPointF(event.position())
            dx = float(cur2.x() - self._drag_last.x())
            dy = float(cur2.y() - self._drag_last.y())
            self._drag_last = cur2
            self.nudge_center_by_pixels(dx, dy)
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:
        if event.button() == Qt.MouseButton.LeftButton:
            if (
                self._add_wp_press_candidate
                and self._add_wp_press_geo is not None
                and self._add_wp_mode
            ):
                lat, lon = self._add_wp_press_geo
                self._waypoints.append((lat, lon))
                self._mission_nav_seq = 0
                self.update()
                self.user_waypoints_changed.emit()
            self._add_wp_press_candidate = False
            self._add_wp_press_geo = None
            self._add_wp_press_screen = None

            if (
                self._fence_press_candidate
                and self._fence_press_geo is not None
                and self._fence_draw_mode
            ):
                lat, lon = self._fence_press_geo
                self._fence_points.append((lat, lon))
                self.update()
                self.user_fence_changed.emit()
            self._fence_press_candidate = False
            self._fence_press_geo = None
            self._fence_press_screen = None

            if (
                self._obs_mark_mode
                and self._obs_click_candidate
                and self._press_for_obs is not None
                and not self._add_wp_mode
                and not self._fence_draw_mode
            ):
                latlon = self._screen_to_lat_lon(self._press_for_obs)
                if latlon is not None:
                    self.observation_map_click.emit(float(latlon[0]), float(latlon[1]))
            self._obs_click_candidate = False
            self._press_for_obs = None
            self._dragging = False
            self._drag_last = None
        super().mouseReleaseEvent(event)

    def _screen_to_lat_lon(self, pos: QPointF) -> tuple[float, float] | None:
        w, h = self.width(), self.height()
        if w <= 1 or h <= 1:
            return None
        z = int(max(3, min(self._max_zoom, round(self._zoom))))
        tile_size = 256
        center_x = _lon_to_x(self._center_lon, z)
        center_y = _lat_to_y(self._center_lat, z)
        fx = center_x * tile_size - w / 2.0
        fy = center_y * tile_size - h / 2.0
        px = pos.x() + fx
        py = pos.y() + fy
        x_t = px / tile_size
        y_t = py / tile_size
        lon = x_t / (2.0**z) * 360.0 - 180.0
        n = math.pi - 2.0 * math.pi * y_t / (2.0**z)
        lat = math.degrees(math.atan(0.5 * (math.exp(n) - math.exp(-n))))
        return (_clamp_lat(lat), float(max(-180, min(180, lon))))


def _circle_ring_points(center_lat: float, center_lon: float, radius_m: float, *, segments: int) -> list[tuple[float, float]]:
    r = max(10.0, float(radius_m))
    n = max(8, min(96, int(segments)))
    lat0 = math.radians(center_lat)
    lon0 = math.radians(center_lon)
    earth_r = 6_371_000.0
    ang = r / earth_r
    out: list[tuple[float, float]] = []
    for i in range(n + 1):
        theta = 2.0 * math.pi * (i / n)
        lat = math.asin(
            math.sin(lat0) * math.cos(ang) + math.cos(lat0) * math.sin(ang) * math.cos(theta)
        )
        lon = lon0 + math.atan2(
            math.sin(theta) * math.sin(ang) * math.cos(lat0),
            math.cos(ang) - math.sin(lat0) * math.sin(lat),
        )
        out.append((math.degrees(lat), math.degrees(lon)))
    return out


def _fetch_tile_http_or_file(url: str) -> tuple[QImage, bytes | None]:
    try:
        if url.startswith("http://") or url.startswith("https://"):
            req = Request(
                url,
                headers={
                    "User-Agent": (
                        "Mozilla/5.0 (Windows NT 10.0; Win64; x64) "
                        "AppleWebKit/537.36 (KHTML, like Gecko) Chrome/122.0.0.0 Safari/537.36"
                    ),
                    "Referer": (
                        "https://www.arcgis.com/"
                        if "arcgisonline.com" in url.lower()
                        else "https://www.openstreetmap.org/"
                    ),
                },
                method="GET",
            )
            with urlopen(req, timeout=2.5) as resp:
                raw = resp.read()
            img = QImage.fromData(raw)
            if img.isNull():
                return QImage(), None
            return img, raw
        p = Path(url)
        if p.is_file():
            img = QImage(str(p))
            return img, None
    except Exception:
        pass
    return QImage(), None


def _extract_paren_payload(src: str, fn: str) -> str | None:
    i = src.find(f"{fn}(")
    if i < 0:
        return None
    depth = 0
    start = -1
    for j in range(i + len(fn) + 1, len(src)):
        ch = src[j]
        if ch == "(":
            depth += 1
        elif ch == ")":
            if depth == 0:
                return src[i + len(fn) + 1 : j]
            depth -= 1
        elif start < 0 and ch not in " \t":
            start = j
    return None


def _split_js_args(payload: str) -> list[str]:
    """Very small splitter for setTileSource('url', 'attr', 19)."""
    out: list[str] = []
    cur = []
    in_str: str | None = None
    i = 0
    while i < len(payload):
        ch = payload[i]
        if in_str:
            if ch == in_str and (i == 0 or payload[i - 1] != "\\"):
                in_str = None
                out.append("".join(cur))
                cur = []
            else:
                cur.append(ch)
            i += 1
            continue
        if ch in "'\"":
            in_str = ch
            i += 1
            continue
        if ch == ",":
            if cur:
                out.append("".join(cur).strip())
                cur = []
            i += 1
            continue
        if ch.isspace() and not cur:
            i += 1
            continue
        cur.append(ch)
        i += 1
    if cur:
        out.append("".join(cur).strip())
    return [x for x in out if x]
