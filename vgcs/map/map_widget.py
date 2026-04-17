"""M2 map scaffold with live position API and WebEngine/Leaflet integration."""

from __future__ import annotations

import json

from PySide6.QtCore import QTimer, Qt, QUrl
from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QLabel,
    QFileDialog,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from vgcs.mission import Waypoint, load_waypoints_json, save_waypoints_json


try:
    from PySide6.QtWebEngineWidgets import QWebEngineView

    HAS_WEBENGINE = True
except Exception:  # pragma: no cover - environment-specific availability
    QWebEngineView = None  # type: ignore[assignment]
    HAS_WEBENGINE = False


LEAFLET_HTML = """<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
  <style>
    html, body, #map { height:100%; margin:0; background:#1a1d24; }
    .leaflet-control-attribution { background: rgba(26,29,36,0.7); color:#a8b0c4; }
  </style>
</head>
<body>
  <div id="map"></div>
  <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
  <script>
    const map = L.map('map').setView([20, 0], 2);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 19,
      attribution: '&copy; OpenStreetMap contributors'
    }).addTo(map);

    let vehicleMarker = L.circleMarker([20,0], {
      radius: 7, color: '#4ade80', fillColor: '#4ade80', fillOpacity: 0.8
    }).addTo(map);
    let headingLine = L.polyline([[20,0], [20,0]], { color:'#fbbf24', weight:3 }).addTo(map);
    let waypoints = [];
    let addMode = false;

    function setVehicle(lat, lon) {
      vehicleMarker.setLatLng([lat, lon]);
      updateHeading(window.__heading || 0, lat, lon);
    }

    function updateHeading(deg, latArg, lonArg) {
      window.__heading = deg;
      const p = vehicleMarker.getLatLng();
      const lat = latArg !== undefined ? latArg : p.lat;
      const lon = lonArg !== undefined ? lonArg : p.lng;
      const len = 0.01;
      const rad = deg * Math.PI / 180.0;
      const lat2 = lat + len * Math.cos(rad);
      const lon2 = lon + len * Math.sin(rad);
      headingLine.setLatLngs([[lat, lon], [lat2, lon2]]);
    }

    function enableAddWaypoint() { addMode = true; }

    function clearWaypoints() {
      for (const wp of waypoints) map.removeLayer(wp);
      waypoints = [];
      return 0;
    }

    let fenceCircle = null;
    function setFence(lat, lon, radiusM) {
      if (fenceCircle) map.removeLayer(fenceCircle);
      fenceCircle = L.circle([lat, lon], {
        radius: radiusM,
        color: '#f87171',
        fillColor: '#f87171',
        fillOpacity: 0.08,
        weight: 2
      }).addTo(map);
    }

    function clearFence() {
      if (!fenceCircle) return 0;
      map.removeLayer(fenceCircle);
      fenceCircle = null;
      return 1;
    }

    function getWaypoints() {
      return waypoints.map(w => [w.getLatLng().lat, w.getLatLng().lng]);
    }

    function setWaypoints(points) {
      clearWaypoints();
      for (const p of points) {
        const m = L.marker([p[0], p[1]]).addTo(map);
        waypoints.push(m);
      }
    }

    function getWaypointCount() { return waypoints.length; }

    map.on('click', function(e) {
      if (!addMode) return;
      const m = L.marker(e.latlng).addTo(map);
      waypoints.push(m);
      addMode = false;
    });
  </script>
</body>
</html>
"""


class MapWidget(QWidget):
    """Map panel with Leaflet backend and waypoint click workflow."""
    waypoints_changed = Signal(list)  # list[Waypoint]
    mission_upload_requested = Signal(list)  # list[Waypoint]
    mission_download_requested = Signal()
    geofence_upload_requested = Signal(object)  # dict fence settings

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._lat: float | None = None
        self._lon: float | None = None
        self._heading: float | None = None
        self._waypoint_count = 0
        self._waypoints_model: list[Waypoint] = []
        self._web_ready = False
        self._is_3d_mode = False
        self._fence_radius_m = 80.0

        root = QVBoxLayout()
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(8)

        panel = QGroupBox("3D Map")
        panel_layout = QVBoxLayout()
        panel_layout.setSpacing(8)

        self._map_canvas = QFrame()
        self._map_canvas.setObjectName("statusChip")
        self._map_canvas_layout = QVBoxLayout()
        self._map_canvas_layout.setContentsMargins(0, 0, 0, 0)
        self._map_canvas_layout.setSpacing(0)
        self._map_canvas.setLayout(self._map_canvas_layout)

        self._status = QLabel("Map status: waiting for telemetry")
        self._status.setObjectName("telemetryValue")
        self._coords = QLabel("Lat/Lon: —")
        self._coords.setObjectName("telemetryValue")
        self._heading_label = QLabel("Heading: —")
        self._heading_label.setObjectName("telemetryValue")
        self._mission = QLabel("Mission WPs: 0")
        self._mission.setObjectName("telemetryValue")

        toolbar = QFrame()
        toolbar.setObjectName("statusChip")
        tools = QGridLayout()
        tools.setHorizontalSpacing(6)
        tools.setVerticalSpacing(6)
        self._btn_add_wp = QPushButton("Add WP")
        self._btn_clear_wp = QPushButton("Clear WPs")
        self._btn_upload = QPushButton("Upload Mission")
        self._btn_download = QPushButton("Download Mission")
        self._btn_export = QPushButton("Export Mission")
        self._btn_import = QPushButton("Import Mission")
        self._btn_3d = QPushButton("3D Toggle")
        self._btn_3d.setCheckable(True)
        self._fence_radius = QDoubleSpinBox()
        self._fence_radius.setRange(10.0, 5000.0)
        self._fence_radius.setDecimals(0)
        self._fence_radius.setSingleStep(10.0)
        self._fence_radius.setValue(80.0)
        self._fence_alt_max = QDoubleSpinBox()
        self._fence_alt_max.setRange(5.0, 2000.0)
        self._fence_alt_max.setDecimals(0)
        self._fence_alt_max.setSingleStep(5.0)
        self._fence_alt_max.setValue(120.0)
        self._btn_fence_apply = QPushButton("Apply Fence")
        self._btn_fence_clear = QPushButton("Clear Fence")
        self._default_alt = QDoubleSpinBox()
        self._default_alt.setRange(1.0, 500.0)
        self._default_alt.setDecimals(1)
        self._default_alt.setSingleStep(1.0)
        self._default_alt.setValue(20.0)
        self._wp_selector = QComboBox()
        self._wp_selector.setMinimumWidth(90)
        self._wp_alt = QDoubleSpinBox()
        self._wp_alt.setRange(1.0, 500.0)
        self._wp_alt.setDecimals(1)
        self._wp_alt.setSingleStep(1.0)
        self._wp_alt.setValue(20.0)
        self._btn_apply_wp_alt = QPushButton("Set WP Alt")
        self._btn_apply_all_alt = QPushButton("Set All Alt")
        tools.addWidget(self._btn_add_wp, 0, 0)
        tools.addWidget(self._btn_clear_wp, 0, 1)
        tools.addWidget(self._btn_upload, 0, 2)
        tools.addWidget(self._btn_download, 0, 3)
        tools.addWidget(self._btn_export, 0, 4)
        tools.addWidget(self._btn_import, 0, 5)
        tools.addWidget(self._btn_3d, 0, 6)
        tools.addWidget(QLabel("Fence R (m)"), 0, 7)
        tools.addWidget(self._fence_radius, 0, 8)
        tools.addWidget(QLabel("Fence Alt"), 0, 9)
        tools.addWidget(self._fence_alt_max, 0, 10)
        tools.addWidget(self._btn_fence_apply, 0, 11)
        tools.addWidget(self._btn_fence_clear, 0, 12)
        tools.addWidget(QLabel("Default Alt (m)"), 1, 0)
        tools.addWidget(self._default_alt, 1, 1)
        tools.addWidget(QLabel("WP"), 1, 2)
        tools.addWidget(self._wp_selector, 1, 3)
        tools.addWidget(QLabel("Alt (m)"), 1, 4)
        tools.addWidget(self._wp_alt, 1, 5)
        tools.addWidget(self._btn_apply_wp_alt, 1, 6)
        tools.addWidget(self._btn_apply_all_alt, 1, 7)
        toolbar.setLayout(tools)
        self._toolbar = toolbar

        status_box = QFrame()
        status_box.setObjectName("statusChip")
        status_layout = QGridLayout()
        status_layout.setHorizontalSpacing(12)
        status_layout.setVerticalSpacing(6)
        status_layout.addWidget(self._status, 0, 0, 1, 2)
        status_layout.addWidget(self._coords, 1, 0)
        status_layout.addWidget(self._heading_label, 1, 1)
        status_layout.addWidget(self._mission, 2, 0, 1, 2)
        status_box.setLayout(status_layout)
        self._status_box = status_box

        panel_layout.addWidget(self._map_canvas)
        panel_layout.addWidget(toolbar)
        panel_layout.addWidget(status_box)
        panel.setLayout(panel_layout)
        root.addWidget(panel)
        self.setLayout(root)

        self._btn_add_wp.clicked.connect(self._enable_add_waypoint_mode)
        self._btn_clear_wp.clicked.connect(self._clear_waypoints)
        self._btn_upload.clicked.connect(self._request_upload)
        self._btn_download.clicked.connect(self._request_download)
        self._btn_export.clicked.connect(self._export_mission)
        self._btn_import.clicked.connect(self._import_mission)
        self._btn_3d.clicked.connect(self._toggle_3d_mode)
        self._btn_fence_apply.clicked.connect(self._apply_geofence)
        self._btn_fence_clear.clicked.connect(self._clear_geofence)
        self._wp_selector.currentIndexChanged.connect(self._on_wp_selected)
        self._btn_apply_wp_alt.clicked.connect(self._apply_altitude_to_selected)
        self._btn_apply_all_alt.clicked.connect(self._apply_altitude_to_all)

        self._wp_poll = QTimer(self)
        self._wp_poll.setInterval(1000)
        self._wp_poll.timeout.connect(self._sync_waypoint_count_from_map)
        self._wp_poll.start()

        self._init_map_backend()

    def set_dashboard_mode(self, enabled: bool) -> None:
        """Hide map-edit controls for clean dashboard layout."""
        self._toolbar.setVisible(not enabled)
        self._status_box.setVisible(not enabled)

    def _init_map_backend(self) -> None:
        if HAS_WEBENGINE and QWebEngineView is not None:
            self._web = QWebEngineView()
            self._web.setMinimumHeight(260)
            self._web.setHtml(LEAFLET_HTML, QUrl("https://vgcs.local/"))
            self._web.loadFinished.connect(self._on_map_loaded)
            self._map_canvas_layout.addWidget(self._web)
            self._set_status("Map backend: Leaflet (WebEngine)")
            return

        placeholder = QLabel(
            "Qt WebEngine is not available. Install PySide6 WebEngine modules to enable the interactive map."
        )
        placeholder.setWordWrap(True)
        placeholder.setAlignment(Qt.AlignCenter)
        placeholder.setStyleSheet("color: #a8b0c4; padding: 20px;")
        self._map_canvas_layout.addWidget(placeholder)
        self._set_status("Map backend unavailable (placeholder mode)")

    def _on_map_loaded(self, ok: bool) -> None:
        self._web_ready = bool(ok)
        if self._web_ready:
            self._set_status("Map ready")
        else:
            self._set_status("Map failed to load")

    def _run_js(self, script: str, callback=None) -> None:
        if not getattr(self, "_web_ready", False):
            return
        if not hasattr(self, "_web"):
            return
        self._web.page().runJavaScript(script, callback)

    def _set_status(self, text: str) -> None:
        self._status.setText(f"Map status: {text}")

    def set_vehicle_position(self, lat: float, lon: float, *, relative_alt_m: float | None = None) -> None:
        self._lat = lat
        self._lon = lon
        if relative_alt_m is None:
            self._coords.setText(f"Lat/Lon: {lat:.7f}, {lon:.7f}")
        else:
            self._coords.setText(
                f"Lat/Lon: {lat:.7f}, {lon:.7f}  |  Rel Alt: {relative_alt_m:.1f} m"
            )
        self._run_js(f"setVehicle({lat:.8f}, {lon:.8f});")
        self._set_status("vehicle marker updated")

    def set_vehicle_heading(self, heading_deg: float) -> None:
        self._heading = heading_deg % 360.0
        self._heading_label.setText(f"Heading: {self._heading:.1f}°")
        self._run_js(f"updateHeading({self._heading:.2f});")

    def set_mission_waypoint_count(self, count: int) -> None:
        self._waypoint_count = max(0, int(count))
        self._mission.setText(f"Mission WPs: {self._waypoint_count}")

    def _enable_add_waypoint_mode(self) -> None:
        self._run_js("enableAddWaypoint();")
        self._set_status("click on map to add waypoint")

    def _clear_waypoints(self) -> None:
        self._run_js(
            "clearWaypoints();",
            callback=lambda _: self._after_waypoints_mutated(),
        )
        self._set_status("waypoints cleared")

    def _sync_waypoint_count_from_map(self) -> None:
        self._run_js("JSON.stringify(getWaypoints());", callback=self._on_waypoints_json)

    def _on_waypoints_json(self, payload: str | None) -> None:
        if not payload:
            self.set_mission_waypoint_count(0)
            self._waypoints_model = []
            self._rebuild_wp_selector()
            self.waypoints_changed.emit([])
            return
        try:
            rows = json.loads(payload)
        except Exception:
            return
        waypoints: list[Waypoint] = []
        for idx, row in enumerate(rows):
            if not (isinstance(row, list) and len(row) >= 2):
                continue
            lat = float(row[0])
            lon = float(row[1])
            alt = (
                self._waypoints_model[idx].alt_m
                if idx < len(self._waypoints_model)
                else float(self._default_alt.value())
            )
            waypoints.append(Waypoint(lat=lat, lon=lon, alt_m=alt))
        self._waypoints_model = waypoints
        self.set_mission_waypoint_count(len(waypoints))
        self._rebuild_wp_selector()
        self.waypoints_changed.emit(waypoints)

    def _after_waypoints_mutated(self) -> None:
        self._sync_waypoint_count_from_map()

    def _request_upload(self) -> None:
        self._run_js(
            "JSON.stringify(getWaypoints());",
            callback=lambda payload: self._emit_upload_from_json(payload),
        )

    def _emit_upload_from_json(self, payload: str | None) -> None:
        if not payload:
            self._set_status("No waypoints to upload")
            return
        try:
            rows = json.loads(payload)
            waypoints = []
            for idx, row in enumerate(rows):
                if not (isinstance(row, list) and len(row) >= 2):
                    continue
                lat = float(row[0])
                lon = float(row[1])
                alt = (
                    self._waypoints_model[idx].alt_m
                    if idx < len(self._waypoints_model)
                    else float(self._default_alt.value())
                )
                waypoints.append(Waypoint(lat=lat, lon=lon, alt_m=alt))
        except Exception:
            self._set_status("Mission parse error")
            return
        if not waypoints:
            self._set_status("No waypoints to upload")
            return
        self.mission_upload_requested.emit(waypoints)
        self._set_status(f"Mission upload requested ({len(waypoints)} WPs)")

    def _request_download(self) -> None:
        self.mission_download_requested.emit()
        self._set_status("Mission download requested")

    def _export_mission(self) -> None:
        path, _ = QFileDialog.getSaveFileName(
            self,
            "Export mission waypoints",
            "mission-waypoints.json",
            "JSON files (*.json)",
        )
        if not path:
            return

        def cb(payload: str | None) -> None:
            if not payload:
                self._set_status("No waypoints to export")
                return
            try:
                rows = json.loads(payload)
                waypoints = []
                for idx, row in enumerate(rows):
                    if not (isinstance(row, list) and len(row) >= 2):
                        continue
                    lat = float(row[0])
                    lon = float(row[1])
                    alt = (
                        self._waypoints_model[idx].alt_m
                        if idx < len(self._waypoints_model)
                        else float(self._default_alt.value())
                    )
                    waypoints.append(Waypoint(lat=lat, lon=lon, alt_m=alt))
                save_waypoints_json(path, waypoints)
                self._set_status(f"Mission exported ({len(waypoints)} WPs)")
            except Exception:
                self._set_status("Export failed")

        self._run_js("JSON.stringify(getWaypoints());", callback=cb)

    def _import_mission(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Import mission waypoints",
            "",
            "JSON files (*.json)",
        )
        if not path:
            return
        try:
            waypoints = load_waypoints_json(path)
        except Exception:
            self._set_status("Import failed")
            return
        rows = [[wp.lat, wp.lon] for wp in waypoints]
        self._waypoints_model = list(waypoints)
        js = f"setWaypoints({json.dumps(rows)});"
        self._run_js(js, callback=lambda _: self._after_waypoints_mutated())
        self._set_status(f"Mission imported ({len(waypoints)} WPs)")

    def set_waypoints(self, waypoints: list[Waypoint]) -> None:
        rows = [[wp.lat, wp.lon] for wp in waypoints]
        self._waypoints_model = list(waypoints)
        self._run_js(
            f"setWaypoints({json.dumps(rows)});",
            callback=lambda _: self._after_waypoints_mutated(),
        )
        self._set_status(f"Mission loaded ({len(waypoints)} WPs)")

    def _rebuild_wp_selector(self) -> None:
        current = self._wp_selector.currentIndex()
        self._wp_selector.blockSignals(True)
        self._wp_selector.clear()
        for idx in range(len(self._waypoints_model)):
            self._wp_selector.addItem(f"WP {idx + 1}", idx)
        self._wp_selector.blockSignals(False)
        if self._waypoints_model:
            self._wp_selector.setCurrentIndex(max(0, min(current, len(self._waypoints_model) - 1)))
            self._on_wp_selected(self._wp_selector.currentIndex())
        else:
            self._wp_alt.setValue(float(self._default_alt.value()))

    def _on_wp_selected(self, index: int) -> None:
        if 0 <= index < len(self._waypoints_model):
            self._wp_alt.setValue(float(self._waypoints_model[index].alt_m))

    def _apply_altitude_to_selected(self) -> None:
        idx = self._wp_selector.currentIndex()
        if idx < 0 or idx >= len(self._waypoints_model):
            self._set_status("No waypoint selected")
            return
        self._waypoints_model[idx].alt_m = float(self._wp_alt.value())
        self.waypoints_changed.emit(list(self._waypoints_model))
        self._set_status(f"Updated WP {idx + 1} altitude to {self._wp_alt.value():.1f} m")

    def _apply_altitude_to_all(self) -> None:
        if not self._waypoints_model:
            self._set_status("No waypoints available")
            return
        alt = float(self._wp_alt.value())
        for wp in self._waypoints_model:
            wp.alt_m = alt
        self.waypoints_changed.emit(list(self._waypoints_model))
        self._set_status(f"Updated all waypoint altitudes to {alt:.1f} m")

    def _toggle_3d_mode(self, enabled: bool) -> None:
        active = self.set_3d_enabled(enabled)
        if active != enabled:
            self._btn_3d.blockSignals(True)
            self._btn_3d.setChecked(active)
            self._btn_3d.blockSignals(False)

    def _apply_geofence(self) -> None:
        if self._lat is None or self._lon is None:
            self._set_status("Fence: waiting for vehicle position")
            return
        radius = float(self._fence_radius.value())
        self._fence_radius_m = radius
        self._run_js(f"setFence({self._lat:.8f}, {self._lon:.8f}, {radius:.1f});")
        self.geofence_upload_requested.emit(
            {
                "radius_m": radius,
                "alt_max_m": float(self._fence_alt_max.value()),
                "center_lat": self._lat,
                "center_lon": self._lon,
            }
        )
        self._set_status(f"Fence requested (r={radius:.0f}m)")

    def _clear_geofence(self) -> None:
        self._run_js("clearFence();")
        self._set_status("Fence cleared")

    def set_3d_enabled(self, enabled: bool) -> bool:
        """
        3D entry point for M2 acceptance.

        Full 3D rendering engine is deferred; this keeps an explicit toggle and
        returns fallback status so UI can report behavior clearly.
        """
        self._is_3d_mode = bool(enabled and HAS_WEBENGINE)
        if self._is_3d_mode:
            self._set_status("3D view entry enabled (fallback mode)")
            return True
        if enabled:
            self._set_status("3D view unavailable in this build; using 2D map")
        else:
            self._set_status("2D mode active")
        return False

