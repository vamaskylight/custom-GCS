"""Native Qt slippy-map view (no WebEngine): raster tiles, vehicle, waypoints, pan/zoom."""

from __future__ import annotations

import hashlib
import json
import math
import random
import re
import time
from pathlib import Path
from urllib.request import Request, build_opener

from PySide6.QtCore import QObject, QPointF, QRectF, Qt, QRunnable, QThreadPool, QTimer, QUrl, Signal
from PySide6.QtGui import QColor, QFont, QImage, QMouseEvent, QPainter, QPainterPath, QPen, QWheelEvent
from PySide6.QtWidgets import QWidget

try:
    from PySide6.QtCore import QEventLoop
    from PySide6.QtNetwork import QNetworkAccessManager, QNetworkReply, QNetworkRequest

    _HAS_QT_NETWORK = True
except Exception:
    _HAS_QT_NETWORK = False


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


_TILE_SIZE = 256
_TILE_MAX_ACTIVE_HTTP = 24
# Esri World Imagery is reliable through 19; higher LODs often return placeholders and stall the UI.
_ESRI_WORLD_IMAGERY_MAX_ZOOM = 19
_TILE_PLACEHOLDER_MAX_RETRIES = 3
_TILE_MEMORY_CAP = 640
_TILE_CACHE_ROOT = (Path.home() / ".vgcs" / "tile-cache").resolve()
_HTTP_OPENER = build_opener()
_BUNDLED_SEED_ROOT = (Path(__file__).resolve().parents[1] / "assets" / "companion_tile_seed").resolve()
_TILE_FETCH_ERRORS_LOGGED = 0
_UA = (
    "Mozilla/5.0 (Windows NT 10.0; Win64; x64) "
    "AppleWebKit/537.36 (KHTML, like Gecko) Chrome/122.0.0.0 Safari/537.36"
)


def bundled_seed_root() -> Path:
    return _BUNDLED_SEED_ROOT


def bundled_seed_tile_path(z: int, x: int, y: int) -> Path | None:
    p = _BUNDLED_SEED_ROOT / str(z) / str(x) / f"{y}.png"
    return p if p.is_file() else None


def _referer_for_url(url: str) -> str:
    u = url.lower()
    if "arcgisonline.com" in u or "arcgis.com" in u:
        return "https://www.arcgis.com/"
    if "openstreetmap.org" in u:
        return "https://www.openstreetmap.org/"
    return "https://www.arcgis.com/"


def _fetch_http_bytes_urllib(url: str, *, timeout_s: float = 5.0) -> bytes:
    req = Request(
        url,
        headers={
            "User-Agent": _UA,
            "Referer": _referer_for_url(url),
        },
        method="GET",
    )
    with _HTTP_OPENER.open(req, timeout=timeout_s) as resp:
        code = getattr(resp, "status", None) or resp.getcode()
        if int(code) >= 400:
            raise OSError(f"HTTP {code}")
        return resp.read()


def _fetch_http_bytes_qt(url: str, *, timeout_s: float = 5.0) -> bytes:
    if not _HAS_QT_NETWORK:
        raise RuntimeError("QtNetwork unavailable")
    loop = QEventLoop()
    out: list[bytes] = []
    err: list[str] = []
    nam = QNetworkAccessManager()
    req = QNetworkRequest(QUrl(url))
    req.setRawHeader(b"User-Agent", _UA.encode("ascii"))
    req.setRawHeader(b"Referer", _referer_for_url(url).encode("ascii"))
    reply = nam.get(req)

    def _done() -> None:
        try:
            if reply.error() == QNetworkReply.NetworkError.NoError:
                data = bytes(reply.readAll())
                if data:
                    out.append(data)
                else:
                    err.append("empty_body")
            else:
                err.append(str(reply.errorString() or reply.error()))
        finally:
            if loop.isRunning():
                loop.quit()

    reply.finished.connect(_done)
    QTimer.singleShot(int(max(500, timeout_s * 1000)), loop.quit)
    loop.exec()
    reply.deleteLater()
    if out:
        return out[0]
    raise OSError(err[0] if err else "timeout")


def fetch_tile_http_bytes(url: str, *, timeout_s: float = 5.0) -> bytes:
    """Fetch tile bytes — urllib first (worker-safe), then Qt network on the main thread path."""
    if url.startswith("http://") or url.startswith("https://"):
        try:
            return _fetch_http_bytes_urllib(url, timeout_s=timeout_s)
        except Exception:
            pass
        if _HAS_QT_NETWORK:
            return _fetch_http_bytes_qt(url, timeout_s=timeout_s)
        raise OSError("http fetch failed")
    raise ValueError("not an http url")


def _log_tile_fetch_error(url: str, detail: str) -> None:
    global _TILE_FETCH_ERRORS_LOGGED
    if _TILE_FETCH_ERRORS_LOGGED >= 6:
        return
    _TILE_FETCH_ERRORS_LOGGED += 1
    try:
        print(f"[VGCS:map-native] tile fetch failed ({detail}): {url[:160]}")
    except Exception:
        pass


def _tile_source_id(template: str) -> str:
    return hashlib.sha256(str(template or "").encode("utf-8")).hexdigest()[:16]


def _max_zoom_for_template(template: str, requested: int) -> int:
    """Pick a safe max zoom for the tile URL (Esri imagery has more LODs than streets/OSM)."""
    t = str(template or "").lower()
    try:
        z = int(requested)
    except Exception:
        z = 19
    if "world_imagery" in t:
        return max(3, min(_ESRI_WORLD_IMAGERY_MAX_ZOOM, z))
    return max(3, min(22, z))


def _disk_cache_path(source_id: str, z: int, x: int, y: int) -> Path:
    return _TILE_CACHE_ROOT / source_id / str(z) / str(x) / f"{y}.png"


def _prepare_tile_image(img: QImage) -> QImage:
    """Normalize to map tile size once (avoid per-frame scaling in paintEvent)."""
    if img.isNull():
        return QImage()
    if img.width() == _TILE_SIZE and img.height() == _TILE_SIZE:
        return img
    return img.scaled(
        _TILE_SIZE,
        _TILE_SIZE,
        Qt.AspectRatioMode.IgnoreAspectRatio,
        Qt.TransformationMode.FastTransformation,
    )


def _tile_image_is_placeholder(img: QImage, *, raw_byte_len: int = 0, zoom: int | None = None) -> bool:
    """
    Esri World Imagery often returns HTTP 200 gray tiles labeled
    "Map data not yet available" (especially right after zoom-in).
  """
    z = int(zoom) if zoom is not None else -1
    if img.isNull() or img.width() < 16 or img.height() < 16:
        return True
    w = img.width()
    h = img.height()
    sx = max(1, w // 16)
    sy = max(1, h // 16)
    n = 0
    sum_y = 0.0
    sum2_y = 0.0
    light_gray = 0
    colors: set[tuple[int, int, int]] = set()
    for y in range(0, h, sy):
        for x in range(0, w, sx):
            c = img.pixelColor(x, y)
            colors.add((c.red() // 16, c.green() // 16, c.blue() // 16))
            yy = 0.2126 * c.red() + 0.7152 * c.green() + 0.0722 * c.blue()
            sum_y += yy
            sum2_y += yy * yy
            if 175 <= c.red() <= 245 and 175 <= c.green() <= 245 and 175 <= c.blue() <= 245:
                light_gray += 1
            n += 1
    if n <= 0:
        return True
    mean = sum_y / n
    var_y = (sum2_y / n) - (mean * mean)
    # Flat gray Esri placeholder (see map_widget._TileProbeTask._classify_image).
    if 150.0 < mean < 245.0 and var_y < 180.0 and (light_gray / n) > 0.52:
        return True
    if raw_byte_len > 0 and 150.0 < mean < 235.0 and var_y < 50.0 and raw_byte_len < 8000:
        return True
    # SMPTE / color-bar test patterns — skip at high zoom (roofs/pavement look low-diversity).
    if z < 17 and len(colors) <= 24:
        return True
    return False


def _accept_map_tile(
    img: QImage, *, raw_byte_len: int = 0, zoom: int | None = None
) -> QImage | None:
    """Return prepared tile, or None if missing / Esri placeholder (never paint placeholders)."""
    if img.isNull():
        return None
    prepared = _prepare_tile_image(img)
    if prepared.isNull() or _tile_image_is_placeholder(
        prepared, raw_byte_len=raw_byte_len, zoom=zoom
    ):
        return None
    return prepared


class _TileFetchSignals(QObject):
    loaded = Signal(int, int, int, object)  # z, x, y, QImage or None


class _NativeTileLoader(QObject):
    """Main-thread QNetworkAccessManager tile fetcher (proxy/SSL match Qt WebEngine)."""

    loaded = Signal(int, int, int, object)

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._nam = QNetworkAccessManager(self) if _HAS_QT_NETWORK else None
        # (distance_from_center, z, x, y, url, source_id, use_disk_cache)
        self._queue: list[tuple[float, int, int, int, str, str, bool]] = []
        self._active = 0
        self._max_active = _TILE_MAX_ACTIVE_HTTP
        self._pool = QThreadPool.globalInstance()
        self._priority_z: int | None = None
        self._center_tx = 0
        self._center_ty = 0

    def set_view_zoom(self, z: int, center_tx: int, center_ty: int) -> None:
        """Drop queued fetches from other zoom levels (wheel zoom should not wait on stale tiles)."""
        self._priority_z = int(z)
        self._center_tx = int(center_tx)
        self._center_ty = int(center_ty)
        self._queue = [e for e in self._queue if e[1] == self._priority_z]

    def request(
        self,
        z: int,
        x: int,
        y: int,
        url: str,
        *,
        source_id: str,
        use_disk_cache: bool,
        distance: float = 0.0,
    ) -> None:
        if self._priority_z is not None and int(z) != self._priority_z:
            return
        self._queue.append((float(distance), z, x, y, url, source_id, use_disk_cache))
        self._queue.sort(key=lambda e: e[0])
        self._drain()

    def _drain(self) -> None:
        if self._nam is None:
            self._start_worker_fallback()
            return
        while self._active < self._max_active and self._queue:
            _dist, z, x, y, url, source_id, use_disk_cache = self._queue.pop(0)
            req = QNetworkRequest(QUrl(url))
            req.setRawHeader(b"User-Agent", _UA.encode("ascii"))
            req.setRawHeader(b"Referer", _referer_for_url(url).encode("ascii"))
            reply = self._nam.get(req)
            self._active += 1

            def _done(r=reply, zz=z, xx=x, yy=y, u=url, sid=source_id, udc=use_disk_cache) -> None:
                self._active = max(0, self._active - 1)
                try:
                    if r.error() == QNetworkReply.NetworkError.NoError:
                        raw = bytes(r.readAll())
                        img = QImage.fromData(raw)
                        if not img.isNull():
                            accepted = _accept_map_tile(img, raw_byte_len=len(raw), zoom=zz)
                            if accepted is not None:
                                if udc and sid and raw:
                                    try:
                                        cp = _disk_cache_path(sid, zz, xx, yy)
                                        cp.parent.mkdir(parents=True, exist_ok=True)
                                        cp.write_bytes(raw)
                                    except Exception:
                                        pass
                                self.loaded.emit(zz, xx, yy, accepted)
                                self._drain()
                                return
                except Exception:
                    pass
                # Never block the GUI thread with a synchronous urllib/Qt refetch here.
                self.loaded.emit(zz, xx, yy, QImage())
                self._drain()

            reply.finished.connect(_done)

    def _start_worker_fallback(self) -> None:
        if not self._queue:
            return
        _dist, z, x, y, url, source_id, use_disk_cache = self._queue.pop(0)
        bridge = _TileFetchSignals()
        bridge.loaded.connect(
            lambda az, ax, ay, im, tgt=self.loaded: tgt.emit(az, ax, ay, im),
            Qt.ConnectionType.QueuedConnection,
        )
        self._pool.start(
            _TileFetchTask(z, x, y, url, bridge, source_id=source_id, use_disk_cache=use_disk_cache)
        )
        if self._queue:
            QTimer.singleShot(0, self._start_worker_fallback)


class _TileFetchTask(QRunnable):
    def __init__(
        self,
        z: int,
        x: int,
        y: int,
        url: str,
        bridge: _TileFetchSignals,
        *,
        source_id: str = "",
        use_disk_cache: bool = True,
    ) -> None:
        super().__init__()
        self._z = z
        self._x = x
        self._y = y
        self._url = url
        self._bridge = bridge
        self._source_id = str(source_id or "")
        self._use_disk_cache = bool(use_disk_cache)

    def run(self) -> None:
        img = _fetch_tile_http_or_file(
            self._url,
            source_id=self._source_id,
            z=self._z,
            x=self._x,
            y=self._y,
            use_disk_cache=self._use_disk_cache,
        )
        self._bridge.loaded.emit(self._z, self._x, self._y, img)


class _DiskTileWarmTask(QRunnable):
    """Load disk/bundled tiles off the GUI thread (paint must never block on I/O)."""

    def __init__(
        self,
        items: list[tuple[int, int, int, str, bool]],
        bridge: _TileFetchSignals,
    ) -> None:
        super().__init__()
        self._items = items
        self._bridge = bridge

    def run(self) -> None:
        for z, x, y, source_id, use_disk_cache in self._items:
            seed = bundled_seed_tile_path(z, x, y)
            if seed is not None:
                try:
                    accepted = _accept_map_tile(QImage(str(seed)), zoom=z)
                    if accepted is not None:
                        self._bridge.loaded.emit(z, x, y, accepted)
                        continue
                except Exception:
                    pass
            if use_disk_cache and source_id:
                try:
                    cp = _disk_cache_path(source_id, z, x, y)
                    if cp.is_file():
                        accepted = _accept_map_tile(QImage(str(cp)), zoom=z)
                        if accepted is not None:
                            self._bridge.loaded.emit(z, x, y, accepted)
                        else:
                            try:
                                cp.unlink(missing_ok=True)
                            except Exception:
                                pass
                except Exception:
                    pass


class NativeTileMapView(QWidget):
    """Minimal interactive map: HTTP tiles, drag pan, wheel zoom, markers."""

    user_waypoints_changed = Signal()
    user_fence_changed = Signal()
    observation_map_click = Signal(float, float)
    zoom_changed = Signal(float)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_OpaquePaintEvent, True)
        self.setMouseTracking(True)
        # Waypoint removal uses right-click; suppress the empty Qt context menu on the map.
        self.setContextMenuPolicy(Qt.ContextMenuPolicy.NoContextMenu)
        self.setMinimumHeight(200)
        self._zoom = 16.0
        self._view_z = 16
        self._max_zoom = _ESRI_WORLD_IMAGERY_MAX_ZOOM
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
        self._preview_tiles: dict[tuple[int, int, int], QImage] = {}
        self._tiles_inflight: set[tuple[int, int, int]] = set()
        self._tile_retry_after: dict[tuple[int, int, int], float] = {}
        self._tile_retry_count: dict[tuple[int, int, int], int] = {}
        self._tile_loader = _NativeTileLoader(self)
        self._tile_loader.loaded.connect(self._on_tile_loaded)
        self._tile_source_id = _tile_source_id(self._tile_template)
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
        self._geo_referenced_marks: list[tuple[float, float]] = []
        self._dooaf_gun: tuple[float, float] | None = None
        self._dooaf_intended: tuple[float, float] | None = None
        self._dooaf_impact: tuple[float, float] | None = None
        # Consecutive ground targets (lat, lon) for measure lines + distance labels.
        self._target_track: list[tuple[float, float]] = []
        self._target_track_labels: list[str] = []
        self._pending_timer = QTimer(self)
        self._pending_timer.setSingleShot(True)
        self._pending_timer.setInterval(32)
        self._pending_timer.timeout.connect(self.update)
        self._disk_warm_bridge = _TileFetchSignals()
        self._disk_warm_bridge.loaded.connect(self._on_tile_loaded)
        self._pool = QThreadPool.globalInstance()
        self.setStyleSheet("background-color: #1a1d24;")
        # >1.0 enlarges the vehicle chevron (used when the map is mirrored into the small video-swap card).
        self._vehicle_arrow_scale = 1.0
        self._sync_view_zoom()
        QTimer.singleShot(0, self._warm_disk_tiles_for_viewport)

    def set_vehicle_arrow_scale(self, factor: float) -> None:
        """Scale the on-map vehicle icon (1.0 = default). Clamped for sane line widths."""
        try:
            self._vehicle_arrow_scale = max(1.0, min(2.6, float(factor)))
        except Exception:
            self._vehicle_arrow_scale = 1.0
        self.update()

    def loaded_tile_count(self) -> int:
        """Count in-memory tiles that decoded successfully (for startup health checks)."""
        try:
            return sum(1 for im in self._tiles.values() if not im.isNull())
        except Exception:
            return 0

    def local_viewport_has_tiles(self) -> bool:
        """True if the offline folder contains at least one tile for the current map center."""
        if self._tile_template != "{local}" or not self._offline_root:
            return False
        root = Path(self._offline_root)
        z = int(self._view_z)
        x, y = _tile_xy(self._center_lat, self._center_lon, z)
        if (root / str(z) / str(x) / f"{y}.png").is_file():
            return True
        if z != 16:
            x16, y16 = _tile_xy(self._center_lat, self._center_lon, 16)
            if (root / "16" / str(x16) / f"{y16}.png").is_file():
                return True
        return False

    # --- map state ---
    def set_center(self, lat: float, lon: float) -> None:
        self._center_lat = float(lat)
        self._center_lon = float(lon)
        self._warm_disk_tiles_for_viewport()
        self.prefetch_viewport_tiles()
        self.update()

    def center_on_vehicle(self) -> None:
        if self._vehicle_lat is not None and self._vehicle_lon is not None:
            self.set_center(self._vehicle_lat, self._vehicle_lon)

    def zoom_level(self) -> float:
        return float(self._zoom)

    def set_zoom(self, z: float) -> None:
        try:
            new_z = max(3.0, min(float(self._max_zoom), float(z)))
        except Exception:
            return
        if abs(new_z - self._zoom) < 1e-6:
            return
        self._zoom = new_z
        self._sync_view_zoom()
        self._schedule_repaint()
        try:
            self.zoom_changed.emit(self._zoom)
        except Exception:
            pass

    def _sync_view_zoom(self) -> None:
        nz = int(max(3, min(self._max_zoom, round(self._zoom))))
        if nz == self._view_z:
            return
        self._view_z = nz
        cx, cy = _tile_xy(self._center_lat, self._center_lon, nz)
        self._tile_loader.set_view_zoom(nz, cx, cy)
        self._tiles_inflight = {k for k in self._tiles_inflight if k[0] == nz}
        self._preview_tiles.clear()
        self._tile_retry_count.clear()
        self._prune_tile_memory()
        self._warm_disk_tiles_for_viewport()

    def _schedule_repaint(self) -> None:
        if not self._pending_timer.isActive():
            self._pending_timer.start()

    def _prune_tile_memory(self) -> None:
        if len(self._tiles) <= _TILE_MEMORY_CAP:
            return
        keep = {self._view_z, self._view_z - 1, self._view_z + 1}
        pruned: dict[tuple[int, int, int], QImage] = {}
        for key, im in self._tiles.items():
            if key[0] in keep:
                pruned[key] = im
        self._tiles = pruned

    def set_vehicle(self, lat: float, lon: float) -> None:
        self.set_vehicle_filtered(lat, lon, append_track=True)

    def set_vehicle_filtered(self, lat: float, lon: float, *, append_track: bool = True) -> None:
        lat_f, lon_f = float(lat), float(lon)
        self._vehicle_lat = lat_f
        self._vehicle_lon = lon_f
        if append_track:
            if self._track:
                last_lat, last_lon = self._track[-1]
                dy_m = (lat_f - last_lat) * 111_320.0
                dx_m = (lon_f - last_lon) * 111_320.0 * math.cos(math.radians(lat_f))
                if math.hypot(dx_m, dy_m) < 5.0:
                    append_track = False
            if append_track:
                self._track.append((lat_f, lon_f))
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

    def set_tile_source(self, template: str, _attribution: str, max_z: int) -> None:
        t = str(template or "").strip()
        if not t:
            return
        self._offline_root = None
        if t.startswith("file:"):
            path = QUrl(t).toLocalFile()
            if path:
                self._offline_root = str(Path(path).resolve())
                self._tile_template = "{local}"
            else:
                self._tile_template = t
        else:
            self._tile_template = t
        self._tile_source_id = _tile_source_id(self._tile_template)
        self._max_zoom = _max_zoom_for_template(self._tile_template, max_z)
        self._tiles.clear()
        self._preview_tiles.clear()
        self._tiles_inflight.clear()
        self._tile_retry_after.clear()
        self._tile_retry_count.clear()
        self._sync_view_zoom()
        self._warm_disk_tiles_for_viewport()
        self._schedule_repaint()

    def set_low_spec(self, on: bool) -> None:
        try:
            self._tile_loader._max_active = 12 if bool(on) else _TILE_MAX_ACTIVE_HTTP
        except Exception:
            pass
        self._schedule_repaint()

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

    def add_geo_referenced_marker(self, lat: float, lon: float) -> None:
        """M8 — cyan marker for video-derived ground intersection."""
        try:
            self._geo_referenced_marks.append((float(lat), float(lon)))
        except Exception:
            return
        if len(self._geo_referenced_marks) > 2000:
            self._geo_referenced_marks = self._geo_referenced_marks[-2000:]
        self.update()

    def set_observation_target_track(
        self,
        points: list[tuple[float, float]],
        *,
        segment_labels: list[str] | None = None,
    ) -> None:
        """M8 measure — draw lines and distance (m) between consecutive geo targets."""
        try:
            self._target_track = [(float(a), float(b)) for a, b in points]
        except Exception:
            self._target_track = []
        if segment_labels is not None:
            self._target_track_labels = [str(s) for s in segment_labels]
        else:
            self._target_track_labels = []
        self.update()

    def set_dooaf_overlay(
        self,
        *,
        gun: tuple[float, float] | None = None,
        intended: tuple[float, float] | None = None,
        impact: tuple[float, float] | None = None,
    ) -> None:
        """Fixed DOOAF points: gun origin, actual target, fall of shot."""
        try:
            self._dooaf_gun = (float(gun[0]), float(gun[1])) if gun is not None else None
            self._dooaf_intended = (
                (float(intended[0]), float(intended[1])) if intended is not None else None
            )
            self._dooaf_impact = (
                (float(impact[0]), float(impact[1])) if impact is not None else None
            )
        except Exception:
            self._dooaf_gun = None
            self._dooaf_intended = None
            self._dooaf_impact = None
        self.update()

    def clear_observation_marks(self) -> None:
        """Clear OBSERVE map/video markers (not DOOAF gun/target/HIT overlay)."""
        try:
            self._observation_marks.clear()
            self._geo_referenced_marks.clear()
            self._target_track.clear()
            self._target_track_labels.clear()
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
            # Position is applied by MapWidget.set_vehicle_position (GPS filtered). Ignore JS echo.
            return
        if p.startswith("updateHeading("):
            m = re.match(r"updateHeading\(\s*([+-]?\d+\.?\d*)", p)
            if m:
                self.set_heading(float(m.group(1)))
            return
        if p.strip() == "centerOnVehicle()" or p.startswith("centerOnVehicle("):
            # Follow/recenter is driven from MapWidget (filtered GPS); ignore JS echo.
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
        z = int(self._view_z)
        tile_size = _TILE_SIZE
        center_x = _lon_to_x(self._center_lon, z)
        center_y = _lat_to_y(self._center_lat, z)
        fx = center_x * tile_size - w / 2.0
        fy = center_y * tile_size - h / 2.0
        x0 = int(math.floor(fx / tile_size)) - 1
        y0 = int(math.floor(fy / tile_size)) - 1
        x1 = int(math.ceil((fx + w) / tile_size)) + 1
        y1 = int(math.ceil((fy + h) / tile_size)) + 1
        n = 1 << z
        for tx in range(x0, x1 + 1):
            for ty in range(y0, y1 + 1):
                if tx < 0 or ty < 0 or tx >= n or ty >= n:
                    continue
                img = self._tile_for_paint(z, tx, ty)
                if img is None or img.isNull():
                    continue
                px = int(tx * tile_size - fx)
                py = int(ty * tile_size - fy)
                painter.drawImage(px, py, img)

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

        # DOOAF fixed points (military gun / target / impact)
        def _dooaf_labeled_point(
            lat: float,
            lon: float,
            *,
            radius: int,
            fill: QColor,
            outline: QColor,
            label: str,
        ) -> None:
            c = self._project(lat, lon, z, fx, fy, w, h)
            painter.setPen(QPen(outline, 2))
            painter.setBrush(fill)
            painter.drawEllipse(c, radius, radius)
            font = QFont("Segoe UI", 8, QFont.Weight.Bold)
            painter.setFont(font)
            metrics = painter.fontMetrics()
            tw = metrics.horizontalAdvance(label) + 8
            th = metrics.height() + 4
            tx = int(c.x()) - tw // 2
            ty = int(c.y()) - radius - th - 4
            painter.fillRect(tx, ty, tw, th, QColor(0, 0, 0, 210))
            painter.setPen(QColor(240, 245, 255))
            painter.drawText(tx + 4, ty + metrics.ascent() + 2, label)

        if self._dooaf_gun is not None:
            gla, glo = self._dooaf_gun
            _dooaf_labeled_point(
                gla,
                glo,
                radius=9,
                fill=QColor(30, 70, 160, 230),
                outline=QColor(180, 210, 255, 240),
                label="GUN",
            )
        if self._dooaf_intended is not None:
            tla, tlo = self._dooaf_intended
            _dooaf_labeled_point(
                tla,
                tlo,
                radius=7,
                fill=QColor(40, 170, 80, 220),
                outline=QColor(180, 255, 200, 240),
                label="TARGET",
            )
        if self._dooaf_impact is not None:
            ila, ilo = self._dooaf_impact
            _dooaf_labeled_point(
                ila,
                ilo,
                radius=7,
                fill=QColor(220, 90, 40, 220),
                outline=QColor(255, 200, 140, 240),
                label="HIT",
            )
        if self._dooaf_gun is not None and self._dooaf_intended is not None:
            c0 = self._project(
                self._dooaf_gun[0], self._dooaf_gun[1], z, fx, fy, w, h
            )
            c1 = self._project(
                self._dooaf_intended[0], self._dooaf_intended[1], z, fx, fy, w, h
            )
            painter.setPen(QPen(QColor(80, 140, 255, 230), 3, Qt.PenStyle.SolidLine))
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.drawLine(c0, c1)
        if self._dooaf_gun is not None and self._dooaf_impact is not None:
            c0 = self._project(
                self._dooaf_gun[0], self._dooaf_gun[1], z, fx, fy, w, h
            )
            c1 = self._project(
                self._dooaf_impact[0], self._dooaf_impact[1], z, fx, fy, w, h
            )
            painter.setPen(QPen(QColor(120, 200, 255, 210), 2, Qt.PenStyle.DashLine))
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.drawLine(c0, c1)

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

        # M8 geo-referenced targets (video ray → ground)
        if self._geo_referenced_marks:
            for lat, lon in self._geo_referenced_marks:
                c = self._project(lat, lon, z, fx, fy, w, h)
                painter.setPen(QPen(QColor(60, 220, 255, 240), 2))
                painter.setBrush(QColor(40, 200, 255, 200))
                painter.drawEllipse(c, 6, 6)
                painter.drawLine(int(c.x() - 12), int(c.y()), int(c.x() + 12), int(c.y()))
                painter.drawLine(int(c.x()), int(c.y() - 12), int(c.x()), int(c.y() + 12))
            # Coordinate label on latest geo target.
            try:
                lat, lon = self._geo_referenced_marks[-1]
                c = self._project(lat, lon, z, fx, fy, w, h)
                caption = f"{float(lat):.6f}, {float(lon):.6f}"
                font = QFont("Segoe UI", 8, QFont.Weight.Bold)
                painter.setFont(font)
                metrics = painter.fontMetrics()
                tw = metrics.horizontalAdvance(caption) + 8
                th = metrics.height() + 4
                tx = int(c.x()) + 10
                ty = int(c.y()) - th - 6
                painter.fillRect(tx, ty, tw, th, QColor(0, 0, 0, 200))
                painter.setPen(QColor(120, 235, 255))
                painter.drawText(tx + 4, ty + metrics.ascent() + 2, caption)
            except Exception:
                pass

        # Measure lines between consecutive targets (ground distance).
        if len(self._target_track) >= 2:
            pen = QPen(QColor(255, 120, 200, 220), 2, Qt.PenStyle.DashLine)
            painter.setPen(pen)
            painter.setBrush(Qt.BrushStyle.NoBrush)
            font = QFont("Segoe UI", 9, QFont.Weight.Bold)
            painter.setFont(font)
            metrics = painter.fontMetrics()
            for i in range(1, len(self._target_track)):
                label = ""
                if i - 1 < len(self._target_track_labels):
                    label = str(self._target_track_labels[i - 1] or "").strip()
                if not label:
                    continue
                la, lo = self._target_track[i - 1]
                lb, lob = self._target_track[i]
                c0 = self._project(la, lo, z, fx, fy, w, h)
                c1 = self._project(lb, lob, z, fx, fy, w, h)
                painter.drawLine(c0, c1)
                mx = (c0.x() + c1.x()) / 2.0
                my = (c0.y() + c1.y()) / 2.0
                tw = metrics.horizontalAdvance(label) + 10
                th = metrics.height() + 6
                tx = int(mx - tw / 2)
                ty = int(my - th / 2)
                painter.fillRect(tx, ty, tw, th, QColor(0, 0, 0, 210))
                painter.setPen(QColor(255, 200, 240))
                painter.drawText(tx + 5, ty + metrics.ascent() + 3, label)

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

    def _project(self, lat: float, lon: float, z: int, fx: float, fy: float, w: int, h: int) -> QPointF:
        px = _lon_to_x(lon, z) * 256.0 - fx
        py = _lat_to_y(lat, z) * 256.0 - fy
        return QPointF(px, py)

    def _view_frame(self) -> tuple[int, float, float, int, int] | None:
        w, h = self.width(), self.height()
        if w <= 1 or h <= 1:
            return None
        z = int(self._view_z)
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
            p = Path(self._offline_root) / str(z) / str(x) / f"{y}.png"
            return QUrl.fromLocalFile(str(p.resolve())).toString()
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
        if int(z) != int(self._view_z):
            key = (z, x, y)
            try:
                self._tiles_inflight.discard(key)
            except Exception:
                pass
            return
        key = (z, x, y)
        try:
            self._tiles_inflight.discard(key)
        except Exception:
            pass
        if int(z) != int(self._view_z):
            return
        if img is None or not isinstance(img, QImage) or img.isNull():
            self._schedule_placeholder_retry(z, x, y)
            return
        if _tile_image_is_placeholder(img, zoom=z):
            self._schedule_placeholder_retry(z, x, y)
            return
        self._tiles[key] = _prepare_tile_image(img)
        self._preview_tiles.pop(key, None)
        self._tile_retry_after.pop(key, None)
        self._tile_retry_count.pop(key, None)
        self._schedule_repaint()

    def _schedule_placeholder_retry(self, z: int, x: int, y: int) -> None:
        """Esri placeholder or miss — keep parent upscale visible and retry later."""
        key = (int(z), int(x), int(y))
        self._tiles.pop(key, None)
        self._preview_tiles.pop(key, None)
        retries = int(self._tile_retry_count.get(key, 0) or 0) + 1
        self._tile_retry_count[key] = retries
        if retries > _TILE_PLACEHOLDER_MAX_RETRIES:
            return
        now = time.monotonic()
        wait = float(self._tile_retry_after.get(key, 0.0) or 0.0)
        if now < wait:
            return
        self._tile_retry_after[key] = now + 2.0
        try:
            cp = _disk_cache_path(self._tile_source_id, key[0], key[1], key[2])
            cp.unlink(missing_ok=True)
        except Exception:
            pass

        def _retry() -> None:
            if int(z) != int(self._view_z):
                return
            self._tile_retry_after.pop(key, None)
            self._queue_tile_fetch(int(z), int(x), int(y))

        QTimer.singleShot(2000, _retry)

    def _valid_cached_tile(self, key: tuple[int, int, int]) -> QImage | None:
        im = self._tiles.get(key)
        if im is None or im.isNull():
            return None
        return im

    def _tile_for_paint(self, z: int, x: int, y: int) -> QImage | None:
        key = (z, x, y)
        cached = self._valid_cached_tile(key)
        if cached is not None:
            return cached
        parent = self._parent_tile_child(z, x, y)
        if parent is not None and not parent.isNull():
            return parent
        self._queue_tile_fetch(z, x, y)
        return None

    def _parent_tile_child(self, z: int, x: int, y: int) -> QImage | None:
        """Upscale a parent tile quadrant so zoom feels instant while HTTP tiles arrive."""
        key = (z, x, y)
        preview = self._preview_tiles.get(key)
        if preview is not None and not preview.isNull():
            return preview
        # At high zoom, only 1 parent level (2×) — deeper upscales look blocky and stay visible too long.
        max_depth = 1 if z >= 17 or z >= int(self._max_zoom) - 1 else 3
        for depth in range(1, max_depth + 1):
            pz = z - depth
            if pz < 3:
                break
            shift = depth
            pkey = (pz, int(x) >> shift, int(y) >> shift)
            parent = self._valid_cached_tile(pkey)
            if parent is None:
                continue
            span = 1 << depth
            half = _TILE_SIZE // span
            ox = (x % span) * half
            oy = (y % span) * half
            try:
                child = parent.copy(ox, oy, half, half)
                if child.isNull():
                    continue
                scaled = child.scaled(
                    _TILE_SIZE,
                    _TILE_SIZE,
                    Qt.AspectRatioMode.IgnoreAspectRatio,
                    Qt.TransformationMode.FastTransformation,
                )
                if not scaled.isNull():
                    self._preview_tiles[key] = scaled
                return scaled
            except Exception:
                continue
        return None

    def _queue_tile_fetch(self, z: int, x: int, y: int) -> None:
        key = (z, x, y)
        if time.monotonic() < float(self._tile_retry_after.get(key, 0.0) or 0.0):
            return
        if key in self._tiles_inflight:
            return
        self._tiles_inflight.add(key)
        url = self._tile_url(z, x, y)
        use_disk_cache = self._tile_template != "{local}"
        cx, cy = _tile_xy(self._center_lat, self._center_lon, z)
        dist = abs(x - cx) + abs(y - cy)
        try:
            self._tile_loader.request(
                z,
                x,
                y,
                url,
                source_id=self._tile_source_id,
                use_disk_cache=use_disk_cache,
                distance=float(dist),
            )
        except Exception:
            self._tiles_inflight.discard(key)

    def prefetch_viewport_tiles(self) -> None:
        """Queue HTTP/disk fetches for visible tiles (call after pan/recenter/follow)."""
        vf = self._view_frame()
        if vf is None:
            return
        z, fx, fy, w, h = vf
        tile_size = _TILE_SIZE
        x0 = int(math.floor(fx / tile_size)) - 1
        y0 = int(math.floor(fy / tile_size)) - 1
        x1 = int(math.ceil((fx + w) / tile_size)) + 1
        y1 = int(math.ceil((fy + h) / tile_size)) + 1
        n = 1 << z
        for tx in range(x0, x1 + 1):
            for ty in range(y0, y1 + 1):
                if tx < 0 or ty < 0 or tx >= n or ty >= n:
                    continue
                key = (z, tx, ty)
                if key in self._tiles or key in self._tiles_inflight:
                    continue
                self._queue_tile_fetch(z, tx, ty)

    def _warm_disk_tiles_for_viewport(self) -> None:
        """Prefetch disk/bundled tiles for the current view without blocking paint."""
        vf = self._view_frame()
        if vf is None:
            return
        z, fx, fy, w, h = vf
        tile_size = _TILE_SIZE
        x0 = int(math.floor(fx / tile_size)) - 1
        y0 = int(math.floor(fy / tile_size)) - 1
        x1 = int(math.ceil((fx + w) / tile_size)) + 1
        y1 = int(math.ceil((fy + h) / tile_size)) + 1
        n = 1 << z
        use_disk_cache = self._tile_template != "{local}"
        items: list[tuple[int, int, int, str, bool]] = []
        seen: set[tuple[int, int, int]] = set()
        for tx in range(x0, x1 + 1):
            for ty in range(y0, y1 + 1):
                if tx < 0 or ty < 0 or tx >= n or ty >= n:
                    continue
                key = (z, tx, ty)
                if key in self._tiles or key in self._tiles_inflight or key in seen:
                    continue
                seen.add(key)
                items.append((z, tx, ty, self._tile_source_id, use_disk_cache))
        if z > 3:
            pn = 1 << (z - 1)
            for tx in range(x0, x1 + 1):
                for ty in range(y0, y1 + 1):
                    ptx, pty = tx >> 1, ty >> 1
                    pkey = (z - 1, ptx, pty)
                    if ptx < 0 or pty < 0 or ptx >= pn or pty >= pn:
                        continue
                    if pkey in self._tiles or pkey in self._tiles_inflight or pkey in seen:
                        continue
                    seen.add(pkey)
                    items.append((z - 1, ptx, pty, self._tile_source_id, use_disk_cache))
        if not items:
            return
        try:
            self._pool.start(_DiskTileWarmTask(items, self._disk_warm_bridge))
        except Exception:
            pass

    def resizeEvent(self, event) -> None:  # noqa: N802
        super().resizeEvent(event)
        # Keep the in-memory tile cache on resize — clearing forced a full re-download.
        self._warm_disk_tiles_for_viewport()
        self._schedule_repaint()

    # --- input ---
    def nudge_center_by_pixels(self, dx: float, dy: float) -> None:
        """Pan the map by ``(dx, dy)`` screen pixels (same math as drag-pan in ``mouseMoveEvent``)."""
        if dx == 0.0 and dy == 0.0:
            return
        z = int(self._view_z)
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
        # ~1 zoom level per standard wheel detent (120°).
        step = (delta / 120.0) * 1.0
        self.set_zoom(self._zoom + step)

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
        z = int(self._view_z)
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


def _fetch_tile_http_or_file(
    url: str,
    *,
    source_id: str = "",
    z: int = 0,
    x: int = 0,
    y: int = 0,
    use_disk_cache: bool = True,
) -> QImage:
    seed = bundled_seed_tile_path(z, x, y)
    if seed is not None:
        try:
            accepted = _accept_map_tile(QImage(str(seed)), zoom=z)
            if accepted is not None:
                return accepted
        except Exception:
            pass
    cache_path: Path | None = None
    if use_disk_cache and source_id:
        cache_path = _disk_cache_path(source_id, z, x, y)
        try:
            if cache_path.is_file():
                accepted = _accept_map_tile(QImage(str(cache_path)), zoom=z)
                if accepted is not None:
                    return accepted
                try:
                    cache_path.unlink(missing_ok=True)
                except Exception:
                    pass
        except Exception:
            pass
    try:
        if url.startswith("http://") or url.startswith("https://"):
            raw = fetch_tile_http_bytes(url, timeout_s=5.0)
            img = QImage.fromData(raw)
            if img.isNull():
                return QImage()
            accepted = _accept_map_tile(img, raw_byte_len=len(raw), zoom=z)
            if accepted is None:
                return QImage()
            if cache_path is not None and raw:
                try:
                    cache_path.parent.mkdir(parents=True, exist_ok=True)
                    cache_path.write_bytes(raw)
                except Exception:
                    pass
            return accepted
        local_path = QUrl(url).toLocalFile() if "://" in str(url) else str(url)
        p = Path(local_path)
        if p.is_file():
            accepted = _accept_map_tile(QImage(str(p)), zoom=z)
            return accepted if accepted is not None else QImage()
    except Exception as e:
        _log_tile_fetch_error(url, f"{type(e).__name__}: {e}")
    return QImage()


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
