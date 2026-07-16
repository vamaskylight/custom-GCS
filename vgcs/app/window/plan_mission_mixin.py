"""MainWindow mixin — see vgcs.app.window package."""

from __future__ import annotations

import math
import time
from collections import deque
from pathlib import Path

from PySide6.QtCore import QEvent, QPoint, QSize, Qt, QSettings, QTimer
from PySide6.QtGui import (
    QColor,
    QGuiApplication,
    QIcon,
    QImage,
    QImageReader,
    QKeySequence,
    QPainter,
    QPixmap,
    QShortcut,
)
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDialog,
    QDoubleSpinBox,
    QFrame,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QInputDialog,
    QMenu,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QScrollBar,
    QSizePolicy,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
    QListWidget,
    QListWidgetItem,
    QStackedWidget,
    QSpinBox,
    QStyle,
    QTextEdit,
    QTabWidget,
    QRadioButton,
    QButtonGroup,
    QFileDialog,
)
from pymavlink import mavutil

from vgcs.app.window.helpers import (
    _mavlink_autopilot_label,
    _mavlink_vehicle_type_label,
    _settings_truthy,
)
from vgcs.app.gcs_style import gcs_stylesheet
from vgcs.app.runtime_ui import build_base_font, select_font_profile
from vgcs.mode import AP_COPTER_MODE_MAP, human_mode_name, modes_for_vehicle_type
from vgcs.mission import Waypoint
from vgcs.map import MapWidget
from vgcs.map.map_web_3d import HAS_WEBENGINE as HAS_MAP_WEBENGINE
from vgcs.app.widgets import CompassWidget
from vgcs.link.mavlink_thread import MavlinkThread
from vgcs.video.pipeline import VideoPipeline
from vgcs.video.widgets import CameraControlPanel
from vgcs.video.camera_control import (
    CompositeGimbalCameraControl,
    MavlinkCameraControl,
    NoopCameraControl,
    read_companion_laser_range_m,
    poll_companion_laser_range_m,
    SiyiCameraControl,
    SkydroidCameraControl,
    resolve_siyi_host,
    resolve_skydroid_control_hosts,
    resolve_skydroid_host,
)


class MainWindowPlanMissionMixin:
    """Extracted from MainWindow — uses host state via self."""

    def _on_map_waypoints_changed(self, waypoints: list) -> None:
        self._mission_table_updating = True
        try:
            self._mission_table.setRowCount(0)
            for i, wp in enumerate(waypoints):
                lat = getattr(wp, "lat", None)
                lon = getattr(wp, "lon", None)
                alt = getattr(wp, "alt_m", 20.0)
                speed = getattr(wp, "speed_mps", 5.0)
                if lat is None or lon is None:
                    continue
                row = self._mission_table.rowCount()
                self._mission_table.insertRow(row)
                item_idx = QTableWidgetItem(str(i + 1))
                item_idx.setFlags(item_idx.flags() & ~Qt.ItemFlag.ItemIsEditable)
                item_lat = QTableWidgetItem(f"{float(lat):.7f}")
                item_lon = QTableWidgetItem(f"{float(lon):.7f}")
                item_alt = QTableWidgetItem(f"{float(alt):.1f}")
                item_spd = QTableWidgetItem(f"{float(speed):.1f}")
                self._mission_table.setItem(row, 0, item_idx)
                self._mission_table.setItem(row, 1, item_lat)
                self._mission_table.setItem(row, 2, item_lon)
                self._mission_table.setItem(row, 3, item_alt)
                self._mission_table.setItem(row, 4, item_spd)
        finally:
            self._mission_table_updating = False
        self._refresh_plan_flight_metrics()

    def _on_mission_table_item_changed(self, item: QTableWidgetItem) -> None:
        if self._mission_table_updating:
            return
        # Inline editing for Lat/Lon/Alt/Speed columns.
        if item.column() not in (1, 2, 3, 4):
            return
        waypoints: list[Waypoint] = []
        normalize_needed = False
        for r in range(self._mission_table.rowCount()):
            lat_item = self._mission_table.item(r, 1)
            lon_item = self._mission_table.item(r, 2)
            alt_item = self._mission_table.item(r, 3)
            spd_item = self._mission_table.item(r, 4)
            if lat_item is None or lon_item is None or alt_item is None or spd_item is None:
                continue
            try:
                lat = float(lat_item.text().strip().replace(",", "."))
                lon = float(lon_item.text().strip().replace(",", "."))
                alt = float(alt_item.text().strip().replace(",", "."))
                spd = float(spd_item.text().strip().replace(",", "."))
            except ValueError:
                # Match the out-of-range case below: abort and tell the operator
                # instead of silently dropping this waypoint from the mission.
                self._append_log(
                    f"Invalid waypoint at row {r + 1}: could not parse lat/lon/alt/speed, edit ignored."
                )
                return
            if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
                self._append_log(
                    f"Invalid waypoint at row {r + 1}: lat/lon out of range, edit ignored."
                )
                return
            waypoints.append(Waypoint(lat=lat, lon=lon, alt_m=alt, speed_mps=max(0.1, spd)))
            # Normalize display formatting after accepted edit.
            lat_fmt = f"{lat:.7f}"
            lon_fmt = f"{lon:.7f}"
            alt_fmt = f"{alt:.1f}"
            spd_fmt = f"{max(0.1, spd):.1f}"
            if (
                lat_item.text() != lat_fmt
                or lon_item.text() != lon_fmt
                or alt_item.text() != alt_fmt
                or spd_item.text() != spd_fmt
            ):
                normalize_needed = True
        if not waypoints:
            return
        if normalize_needed:
            self._mission_table_updating = True
            try:
                for r, wp in enumerate(waypoints):
                    self._mission_table.item(r, 1).setText(f"{wp.lat:.7f}")
                    self._mission_table.item(r, 2).setText(f"{wp.lon:.7f}")
                    self._mission_table.item(r, 3).setText(f"{wp.alt_m:.1f}")
                    self._mission_table.item(r, 4).setText(f"{wp.speed_mps:.1f}")
            finally:
                self._mission_table_updating = False
        self._map_widget.set_waypoints(waypoints)
        if item.column() in (1, 2):
            self._append_log("Mission waypoint position updated from table.")
        elif item.column() == 3:
            self._append_log("Mission altitude updated from table.")
        else:
            self._append_log("Mission speed updated from table.")

    def _on_mission_upload_requested(self, waypoints: list) -> None:
        if self._thread is None or not self._thread.isRunning():
            QMessageBox.warning(self, "VGCS", "Connect vehicle before mission upload.")
            return
        if self._mission_upload_pending:
            self._append_log("Mission upload already in progress…")
            return
        payload = [
            {
                "lat": float(getattr(wp, "lat", 0.0)),
                "lon": float(getattr(wp, "lon", 0.0)),
                "alt_m": float(getattr(wp, "alt_m", 20.0)),
                "speed_mps": float(getattr(wp, "speed_mps", 5.0)),
            }
            for wp in waypoints
        ]
        takeoff_m = self._plan_takeoff_alt_m_from_launch_settings()
        if takeoff_m is not None and payload:
            payload[0] = {**payload[0], "takeoff_alt_m": float(takeoff_m)}
        self._mission_upload_pending = True
        self._thread.queue_mission_upload(payload)
        self._append_log(f"Mission upload queued: {len(payload)} WPs")
        self._set_top_vehicle_msg(f"Uploading mission ({len(payload)} WPs)…")

    def _on_mission_download_requested(self) -> None:
        if self._thread is None or not self._thread.isRunning():
            QMessageBox.warning(self, "VGCS", "Connect vehicle before mission download.")
            return
        self._thread.queue_mission_download()
        self._append_log("Mission download queued")

    def _on_mission_uploaded(self, count: int) -> None:
        self._mission_upload_pending = False
        self._append_log(f"Mission upload success: {count} WPs")
        self._set_top_vehicle_msg(f"Mission uploaded ({count})")
        QMessageBox.information(
            self, "Mission Upload", f"Mission uploaded successfully ({count} waypoints)."
        )

    def _on_mission_downloaded(self, items: object) -> None:
        rows = items if isinstance(items, list) else []
        self._append_log(f"Mission download success: {len(rows)} WPs")
        from vgcs.mission import Waypoint

        wps = [
            Waypoint(
                lat=float(row.get("lat", 0.0)),
                lon=float(row.get("lon", 0.0)),
                alt_m=float(row.get("alt_m", 20.0)),
            )
            for row in rows
            if isinstance(row, dict)
        ]
        self._map_widget.set_waypoints(wps, clear_plan_current_file=True)
        self._set_top_vehicle_msg(f"Mission downloaded ({len(wps)})")

    def _sync_plan_flight_chrome(self) -> None:
        """Enable/disable Plan Flight upload/save buttons from link + waypoint state."""
        link_ok = (
            self._thread is not None
            and self._thread.isRunning()
            and self._heartbeat_seen
        )
        n = len(getattr(self._map_widget, "_waypoints_model", []) or [])
        self._map_widget.refresh_plan_flight_chrome(link_ok=link_ok, waypoint_count=n)
        if self._plan_flight_layer_wanted:
            self._map_widget.set_plan_flight_visible(True)

    def _on_plan_flight_exited(self) -> None:
        n = len(getattr(self._map_widget, "_waypoints_model", []) or [])
        self._plan_flight_layer_wanted = False
        self._settings.setValue("plan_flight_layer_wanted", False)
        self._map_widget.set_plan_mission_start_stack(False)
        self._map_widget.set_plan_sequence_template("")
        self._sync_plan_flight_chrome()
        wp_line = f"{n} waypoint(s) on the map." if n else "No waypoints on the map."
        # Avoid a blocking dialog on every exit — that was easy to mistake for a fault.
        self._append_log(
            f"Plan Flight closed. {wp_line} "
            "Mission options stay in application settings; use Plan Upload or the map toolbar to send to the vehicle."
        )

    def _on_map_page_ready(self) -> None:
        self._restore_plan_mission_panel_to_map()
        if self._plan_flight_layer_wanted:
            self._map_widget.set_plan_flight_visible(True)
        self._sync_plan_flight_chrome()
        # Native 2D is default; 3D uses optional lazy-loaded WebEngine (see map_web_3d / legacy_leaflet_map.html).
        if getattr(self._map_widget, "_native_map", None) is not None:
            if not HAS_MAP_WEBENGINE:
                tip = (
                    "2D native map only — install PySide6 WebEngine to enable the 3D globe (Cesium) toggle."
                )
            else:
                tip = "Toggle 3D globe (WebEngine) or 2D native tiles."
            self._hdr_map_mode_btn.setEnabled(True)
            self._hdr_map_mode_btn.setToolTip(tip)
            self._btn_map_3d.setEnabled(True)
            self._btn_map_3d.setToolTip(tip)
            self._btn_map_3d.blockSignals(True)
            self._btn_map_3d.setChecked(False)
            self._btn_map_3d.blockSignals(False)
            self._sync_hdr_map_mode_btn_label()

    def _restore_plan_mission_panel_to_map(self) -> None:
        s = self._settings
        initial_wp_alt_m = float(
            s.value("plan_initial_wp_alt_m", s.value("plan_initial_wp_alt_ft", 164.0)) or 164.0
        )
        hover_mps = float(
            s.value("plan_hover_speed_mps", s.value("plan_hover_speed_mph", 11.18)) or 11.18
        )
        launch_alt_m = float(s.value("plan_launch_alt_m", s.value("plan_launch_alt_ft", 0.0)) or 0.0)
        state = {
            "altRef": str(s.value("plan_alt_ref", "rel") or "rel"),
            "initialWpAltM": initial_wp_alt_m,
            "hoverMps": hover_mps,
            "launchAltM": launch_alt_m,
            "launchLat": str(s.value("plan_launch_lat_str", "") or ""),
            "launchLon": str(s.value("plan_launch_lon_str", "") or ""),
            "wpMeta": self._map_widget.get_waypoint_meta(),
            "patternRowSpacingM": float(s.value("plan_pattern_row_spacing_m", 20.0) or 20.0),
            "patternPassWidthM": float(s.value("plan_pattern_pass_width_m", 80.0) or 80.0),
            "patternPassDepthM": float(s.value("plan_pattern_pass_depth_m", 60.0) or 60.0),
        }
        self._map_widget.apply_plan_mission_panel_state(state)
        self._apply_plan_mission_panel_to_model(state)

    def _ensure_plan_launch_from_vehicle_if_empty(self) -> None:
        """If mission launch lat/lon are unset, copy current vehicle position (survey / patterns)."""
        lat_s = str(self._settings.value("plan_launch_lat_str", "") or "").strip()
        lon_s = str(self._settings.value("plan_launch_lon_str", "") or "").strip()
        if lat_s and lon_s:
            return
        pos = self._map_widget.get_vehicle_position()
        if not pos:
            return
        lat, lon = pos
        self._settings.setValue("plan_launch_lat_str", f"{lat:.7f}")
        self._settings.setValue("plan_launch_lon_str", f"{lon:.7f}")
        self._restore_plan_mission_panel_to_map()

    def _on_plan_mission_panel_changed(self, data: object) -> None:
        if not isinstance(data, dict):
            return
        s = self._settings
        s.setValue("plan_alt_ref", str(data.get("altRef", "rel") or "rel"))
        initial_wp_alt_m = float(data.get("initialWpAltM", data.get("initialWpAltFt", 164.0)) or 164.0)
        hover_mps = float(data.get("hoverMps", data.get("hoverMph", 11.18)) or 11.18)
        launch_alt_m = float(data.get("launchAltM", data.get("launchAltFt", 0.0)) or 0.0)
        s.setValue("plan_initial_wp_alt_m", initial_wp_alt_m)
        s.setValue("plan_hover_speed_mps", hover_mps)
        s.setValue("plan_launch_alt_m", launch_alt_m)
        # Legacy keys kept in sync for compatibility with older builds/config.
        s.setValue("plan_initial_wp_alt_ft", initial_wp_alt_m)
        s.setValue("plan_hover_speed_mph", hover_mps)
        s.setValue("plan_launch_alt_ft", launch_alt_m)
        lat = str(data.get("launchLat", "") or "").strip()
        lon = str(data.get("launchLon", "") or "").strip()
        if lat and lat != "—":
            s.setValue("plan_launch_lat_str", lat)
        else:
            s.remove("plan_launch_lat_str")
        if lon and lon != "—":
            s.setValue("plan_launch_lon_str", lon)
        else:
            s.remove("plan_launch_lon_str")
        wp_meta = data.get("wpMeta")
        if isinstance(wp_meta, list):
            self._map_widget.apply_waypoint_meta(wp_meta)
        s.setValue(
            "plan_pattern_row_spacing_m",
            float(data.get("patternRowSpacingM", 20.0) or 20.0),
        )
        s.setValue(
            "plan_pattern_pass_width_m",
            float(data.get("patternPassWidthM", 80.0) or 80.0),
        )
        s.setValue(
            "plan_pattern_pass_depth_m",
            float(data.get("patternPassDepthM", 60.0) or 60.0),
        )
        self._apply_plan_mission_panel_to_model(data)

    def _default_wp_alt_m_for_plan_state(self, state: dict[str, object]) -> float:
        ref = str(state.get("altRef", "rel") or "rel").strip().lower()
        meters = float(state.get("initialWpAltM", state.get("initialWpAltFt", 164.0)) or 164.0)
        target_m = meters
        home_amsl = self._home_amsl_m
        if ref == "amsl" and home_amsl is not None:
            return max(1.0, target_m - float(home_amsl))
        return max(1.0, target_m)

    def _plan_takeoff_alt_m_from_launch_settings(self) -> float | None:
        """Relative altitude (m) for NAV_TAKEOFF when Launch Position alt is set; else None (use WP1 alt)."""
        meters = float(
            self._settings.value("plan_launch_alt_m", self._settings.value("plan_launch_alt_ft", 0.0))
            or 0.0
        )
        if meters <= 0.01:
            return None
        state = {
            "altRef": str(self._settings.value("plan_alt_ref", "rel") or "rel"),
            "initialWpAltM": meters,
        }
        return self._default_wp_alt_m_for_plan_state(state)

    def _apply_plan_mission_panel_to_model(self, state: dict[str, object]) -> None:
        self._map_widget.set_default_waypoint_alt_m(self._default_wp_alt_m_for_plan_state(state))
        speed_mps = float(state.get("hoverMps", state.get("hoverMph", 11.18)) or 11.18)
        self._plan_hover_speed_mps = max(0.5, speed_mps)
        self._maybe_refresh_map_web_overlays()

    def _refresh_plan_flight_metrics(self) -> None:
        # M2 plan bar live values (best-effort from real telemetry).
        heading_val = float(getattr(self, "_heading", 0.0) or 0.0)
        alt_diff_m = f"{self._map_rel_alt_m:.1f} m"
        gradient = "-.-"
        azimuth = f"{int(round(heading_val))}"
        heading = f"{int(round(heading_val))}"
        dist_prev_wp_m = "0.0 m"

        mission_distance_m = 0.0
        model = list(getattr(self._map_widget, "_waypoints_model", []))
        for i in range(1, len(model)):
            a = model[i - 1]
            b = model[i]
            mission_distance_m += self._haversine_m(float(a.lat), float(a.lon), float(b.lat), float(b.lon))
        mission_distance_text = f"{mission_distance_m:.0f} m"

        if self._armed_since is not None:
            elapsed = int(time.monotonic() - self._armed_since)
            mission_time = f"{elapsed // 3600:02d}:{(elapsed % 3600) // 60:02d}:{elapsed % 60:02d}"
        elif mission_distance_m > 1.0 and self._plan_hover_speed_mps > 0.5:
            eta_s = int(mission_distance_m / self._plan_hover_speed_mps)
            mission_time = f"{eta_s // 3600:02d}:{(eta_s % 3600) // 60:02d}:{eta_s % 60:02d}"
        else:
            mission_time = "00:00:00"

        max_telem_dist_m = f"{self._max_telem_dist_m:.0f} m"
        self._map_widget.set_plan_flight_metrics(
            alt_diff_m=alt_diff_m,
            gradient=gradient,
            azimuth=azimuth,
            heading=heading,
            dist_prev_wp_m=dist_prev_wp_m,
            mission_distance_m=mission_distance_text,
            mission_time=mission_time,
            max_telem_dist_m=max_telem_dist_m,
        )

    @staticmethod
    def _offset_lat_lon_m(lat_deg: float, lon_deg: float, east_m: float, north_m: float) -> tuple[float, float]:
        d_lat = north_m / 111_320.0
        cos_lat = math.cos(math.radians(lat_deg))
        d_lon = east_m / (111_320.0 * max(0.1, cos_lat))
        return lat_deg + d_lat, lon_deg + d_lon

    def _pattern_anchor_lat_lon(self) -> tuple[float, float] | None:
        """Centroid for M2 auto-generated patterns: vehicle + Mission fence radius (m) north.

        Keeps survey/corridor/structure grids from stacking on the home position; larger
        fence radius pushes the pattern farther away (same control as geofence upload).
        """
        ref = self._map_widget.get_vehicle_position()
        if ref is None:
            return None
        lat0, lon0 = ref
        spawn_m = max(0.0, float(self._geofence_radius_spin.value()))
        return self._offset_lat_lon_m(lat0, lon0, 0.0, spawn_m)

    def _plan_pattern_geometry_m(self) -> tuple[float, float, float]:
        """Row spacing, pass width, pass depth (m) from Mission panel / QSettings."""
        s = self._settings
        row = float(s.value("plan_pattern_row_spacing_m", 20.0) or 20.0)
        w_m = float(s.value("plan_pattern_pass_width_m", 80.0) or 80.0)
        h_m = float(s.value("plan_pattern_pass_depth_m", 60.0) or 60.0)
        row = max(3.0, min(300.0, row))
        w_m = max(15.0, min(2000.0, w_m))
        h_m = max(15.0, min(2000.0, h_m))
        return row, w_m, h_m

    def _build_m2_grid_pattern(self) -> list[Waypoint]:
        anchor = self._pattern_anchor_lat_lon()
        if anchor is None:
            return []
        lat0, lon0 = anchor
        line_spacing_m, width_m, height_m = self._plan_pattern_geometry_m()
        half_w = width_m / 2.0
        half_h = height_m / 2.0
        rows = max(2, int(round(height_m / line_spacing_m)) + 1)
        waypoints: list[Waypoint] = []
        alt_m = float(self._map_widget.get_default_waypoint_alt_m())
        for row in range(rows):
            north = -half_h + row * line_spacing_m
            left = self._offset_lat_lon_m(lat0, lon0, -half_w, north)
            right = self._offset_lat_lon_m(lat0, lon0, half_w, north)
            if row % 2 == 0:
                seq = (left, right)
            else:
                seq = (right, left)
            for lat, lon in seq:
                waypoints.append(Waypoint(lat=lat, lon=lon, alt_m=alt_m))
        return waypoints

    def _build_m2_corridor_pattern(self) -> list[Waypoint]:
        anchor = self._pattern_anchor_lat_lon()
        if anchor is None:
            return []
        lat0, lon0 = anchor
        line_spacing_m, length_m, depth_m = self._plan_pattern_geometry_m()
        line_spacing_m = max(3.0, line_spacing_m)
        half_len = length_m / 2.0
        n_rows = max(2, int(round(depth_m / line_spacing_m)) + 1)
        half_span = (n_rows - 1) * line_spacing_m / 2.0
        waypoints: list[Waypoint] = []
        alt_m = float(self._map_widget.get_default_waypoint_alt_m())
        for row in range(n_rows):
            north = -half_span + row * line_spacing_m
            left = self._offset_lat_lon_m(lat0, lon0, -half_len, north)
            right = self._offset_lat_lon_m(lat0, lon0, half_len, north)
            if row % 2 == 0:
                seq = (left, right)
            else:
                seq = (right, left)
            for lat, lon in seq:
                waypoints.append(Waypoint(lat=lat, lon=lon, alt_m=alt_m))
        return waypoints

    def _build_m2_structure_pattern(self) -> list[Waypoint]:
        anchor = self._pattern_anchor_lat_lon()
        if anchor is None:
            return []
        lat0, lon0 = anchor
        _row, w_m, h_m = self._plan_pattern_geometry_m()
        hw, hh = w_m / 2.0, h_m / 2.0
        corners = [
            self._offset_lat_lon_m(lat0, lon0, -hw, hh),
            self._offset_lat_lon_m(lat0, lon0, hw, hh),
            self._offset_lat_lon_m(lat0, lon0, hw, -hh),
            self._offset_lat_lon_m(lat0, lon0, -hw, -hh),
        ]
        alt_m = float(self._map_widget.get_default_waypoint_alt_m())
        waypoints = [Waypoint(lat=c[0], lon=c[1], alt_m=alt_m) for c in corners]
        waypoints.append(Waypoint(lat=corners[0][0], lon=corners[0][1], alt_m=alt_m))
        return waypoints

    def _on_plan_flight_action(self, action: str) -> None:
        a = (action or "").strip().lower()
        if a == "open":
            self._map_widget.open_mission_file()
            return
        if a == "bar_upload":
            self._map_widget.request_mission_upload_from_map()
            return
        if a == "save":
            self._map_widget.save_plan_mission_json(save_as=False)
            return
        if a == "save_as":
            self._map_widget.save_plan_mission_json(save_as=True)
            return
        if a == "save_kml":
            self._map_widget.save_plan_mission_kml()
            return
        if a == "vehicle_upload":
            self._map_widget.request_mission_upload_from_map()
            return
        if a == "vehicle_download":
            self._map_widget.request_mission_download_from_map()
            return
        if a == "vehicle_clear":
            if (
                QMessageBox.question(
                    self,
                    "Plan Flight",
                    "Remove all waypoints from the map?",
                )
                == QMessageBox.StandardButton.Yes
            ):
                self._map_widget.clear_map_waypoints()
                self._map_widget.set_plan_mission_start_stack(False)
                self._map_widget.set_plan_sequence_template("")
            return
        if a == "template_empty":
            if (
                QMessageBox.question(
                    self,
                    "Empty plan",
                    "Clear all waypoints?",
                )
                == QMessageBox.StandardButton.Yes
            ):
                self._map_widget.clear_map_waypoints()
                self._map_widget.set_plan_mission_start_stack(False)
                self._map_widget.set_plan_sequence_template("")
            return
        if a == "template_survey":
            self._map_widget.set_plan_sequence_template("survey")
            self._append_log("Plan template: Survey")
            self._ensure_plan_launch_from_vehicle_if_empty()
            wps = self._build_m2_grid_pattern()
            if not wps:
                self._map_widget.set_plan_sequence_template("")
                QMessageBox.warning(
                    self,
                    "Plan Flight",
                    "Survey template needs a vehicle GPS position.\nConnect and wait for position first.",
                )
                return
            self._map_widget.set_waypoints(wps, clear_plan_current_file=True)
            self._map_widget.set_plan_mission_start_stack(True, "Survey")
            self._map_widget.set_plan_rail_tool("Pattern")
            self._append_log(f"Survey template: {len(wps)} waypoints (M2 grid)")
            return
        if a == "template_corridor":
            self._map_widget.set_plan_mission_start_stack(False)
            self._map_widget.set_plan_sequence_template("corridor")
            self._append_log("Plan template: Corridor scan")
            self._ensure_plan_launch_from_vehicle_if_empty()
            wps = self._build_m2_corridor_pattern()
            if not wps:
                self._map_widget.set_plan_sequence_template("")
                QMessageBox.warning(
                    self,
                    "Plan Flight",
                    "Corridor template needs a vehicle GPS position.\nConnect and wait for position first.",
                )
                return
            self._map_widget.set_waypoints(wps, clear_plan_current_file=True)
            self._append_log(f"Corridor template: {len(wps)} waypoints")
            return
        if a == "template_structure":
            self._map_widget.set_plan_mission_start_stack(False)
            self._map_widget.set_plan_sequence_template("structure")
            self._append_log("Plan template: Structure scan (perimeter)")
            self._ensure_plan_launch_from_vehicle_if_empty()
            wps = self._build_m2_structure_pattern()
            if not wps:
                self._map_widget.set_plan_sequence_template("")
                QMessageBox.warning(
                    self,
                    "Plan Flight",
                    "Structure template needs a vehicle GPS position.\nConnect and wait for position first.",
                )
                return
            self._map_widget.set_waypoints(wps, clear_plan_current_file=True)
            self._append_log(f"Structure template: {len(wps)} waypoints")
            return
        if a == "fence_roi_tool":
            self._map_widget.set_plan_rail_tool("ROI")
            self._on_plan_tool_requested("roi")
            return

    def _on_plan_tool_requested(self, tool_name: str) -> None:
        tool = (tool_name or "").strip().lower()
        if not tool:
            return
        if tool == "file":
            self._append_log("Plan tool: File")
            return
        if tool == "takeoff":
            self._append_log("Plan tool: Takeoff")
            self._queue_nav_takeoff(self._takeoff_altitude_m(from_plan_rail=True))
            return
        if tool == "waypoint":
            self._append_log("Plan tool: Waypoint mode")
            self._map_widget.start_waypoint_planning()
            return
        if tool == "roi":
            self._append_log("Plan tool: ROI mode")
            self._map_widget.start_roi_planning()
            return
        if tool == "pattern":
            self._append_log("Plan tool: Pattern (M2 grid)")
            self._ensure_plan_launch_from_vehicle_if_empty()
            wps = self._build_m2_grid_pattern()
            if not wps:
                QMessageBox.warning(
                    self,
                    "Plan Flight",
                    "Pattern requires current vehicle position.\nConnect and wait for GPS position first.",
                )
                return
            self._map_widget.set_waypoints(wps, clear_plan_current_file=True)
            self._append_log(f"Pattern generated: {len(wps)} waypoints (M2 grid)")
            return
        if tool == "return":
            self._append_log("Plan tool: Return (RTL)")
            self._on_map_return_requested()
            return
        if tool == "center":
            self._append_log("Plan tool: Center map on vehicle")
            self._map_widget.center_on_vehicle()
            return
