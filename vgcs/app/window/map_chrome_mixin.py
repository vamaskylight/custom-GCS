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


class MainWindowMapChromeMixin:
    """Extracted from MainWindow — uses host state via self."""

    def _sync_hdr_map_mode_btn_label(self) -> None:
        """Match Web `hdrMapModeBtn`: label shows target mode (3D when in 2D, 2D when in 3D)."""
        real = bool(getattr(self._map_widget, "_is_3d_mode", False))
        self._hdr_map_mode_btn.setText("2D" if real else "3D")

    def _on_map_3d_mode_changed(self) -> None:
        """Native 3D flag updates asynchronously after WebEngine load / Cesium JS — keep header + M2 in sync."""
        self._sync_hdr_map_mode_btn_label()
        try:
            real = bool(getattr(self._map_widget, "_is_3d_mode", False))
            self._btn_map_3d.blockSignals(True)
            self._btn_map_3d.setChecked(real)
            self._btn_map_3d.blockSignals(False)
        except Exception:
            pass

    def _on_toggle_map_3d(self, enabled: bool) -> None:
        active = self._map_widget.set_3d_enabled(enabled)
        self._sync_hdr_map_mode_btn_label()
        if active != enabled:
            self._btn_map_3d.blockSignals(True)
            self._btn_map_3d.setChecked(active)
            self._btn_map_3d.blockSignals(False)
            return
        self._on_map_3d_mode_changed()

    def _on_map_toggle_3d_requested(self) -> None:
        current = bool(getattr(self._map_widget, "_is_3d_mode", False))
        self._on_toggle_map_3d(not current)

    def _scroll_main_to(self, widget: QWidget, *, y_margin: int = 20) -> None:
        """Scroll the main content area so ``widget`` is visible."""
        def _do() -> None:
            self._scroll.ensureWidgetVisible(widget, y_margin, y_margin)

        QTimer.singleShot(0, _do)

    def _on_map_menu_requested(self, gx: int, gy: int) -> None:
        # Always anchor under the header/logo area (fixed position),
        # not under the exact mouse click coordinate.
        self._on_logo_menu()

    def _on_toggle_map_3d_shortcut(self) -> None:
        current = bool(getattr(self._map_widget, "_is_3d_mode", False))
        self._on_toggle_map_3d(not current)
        self._append_log("Shortcut: toggle map 3D view (Ctrl+3)")

    def _on_logo_menu(self, anchor_pos: QPoint | None = None) -> None:
        # Wired to ``QPushButton.clicked`` → Qt invokes ``clicked(bool)``; ignore that payload.
        if anchor_pos is not None and not isinstance(anchor_pos, QPoint):
            anchor_pos = None
        menu = QMenu(self)
        menu.setToolTipsVisible(True)
        menu.setStyleSheet(
            "QMenu {"
            " background-color: #f4f4f4;"
            " color: #222222;"
            " border: 1px solid #8f8f8f;"
            " border-radius: 6px;"
            " padding: 8px 6px;"
            " }"
            "QMenu::item {"
            " background-color: #e9e9e9;"
            " margin: 4px 4px;"
            " padding: 10px 16px 10px 40px;"
            " border-radius: 2px;"
            " }"
            "QMenu::item:selected {"
            " background-color: #dddddd;"
            " color: #111111;"
            " }"
            "QMenu::icon { left: 12px; }"
            "QMenu::separator {"
            " height: 1px;"
            " margin: 6px 8px;"
            " background: #bfbfbf;"
            " }"
        )

        action_plan = menu.addAction("Plan Flight")
        action_plan.setToolTip("Mission planning and waypoint management")
        action_plan.setIcon(self._menu_icon("plan_flight.svg"))
        action_analyze = menu.addAction("Analyze Tools")
        action_analyze.setToolTip("Post-flight and real-time data review.")
        action_analyze.setIcon(self._menu_icon("analyze_tools.svg"))
        action_vehicle = menu.addAction("Vehicle Configuration")
        action_vehicle.setToolTip("Vehicle setup tools, including sensor calibration and quick controls.")
        action_vehicle.setIcon(self._menu_icon("flight_mode.svg"))
        action_settings = menu.addAction("Application Settings")
        action_settings.setToolTip("GCS-specific preferences.")
        action_settings.setIcon(self._menu_icon("app_settings.svg"))
        action_toggle_3d = menu.addAction("Toggle 3D View")
        action_toggle_3d.setToolTip("Switch between 2D and 3D map view.")
        menu.addSeparator()
        action_close = menu.addAction("Close VGCS")
        action_close.setToolTip("Exit the application.")
        action_close.setIcon(self._menu_icon("close_vgcs.svg"))

        if anchor_pos is not None and isinstance(anchor_pos, QPoint):
            pos = anchor_pos
        else:
            hdr = getattr(self, "_top_dashboard", None)
            if hdr is not None:
                # Flush under `#headerBar` (no gap); was logo bottom + 6px which showed map between bar and menu.
                pos = hdr.mapToGlobal(QPoint(0, hdr.height()))
            elif self._map_only_dashboard:
                pos = self._map_widget.mapToGlobal(QPoint(10, 56))
            else:
                pos = self._logo_btn.mapToGlobal(self._logo_btn.rect().bottomLeft())
        picked = menu.exec(pos)
        if picked is action_close:
            self.close()
            return
        if picked is action_plan:
            self._append_log("Menu: Plan Flight — mission waypoints")
            self._plan_flight_layer_wanted = True
            self._settings.setValue("plan_flight_layer_wanted", True)
            self._map_widget.set_plan_flight_visible(True)
            self._sync_plan_flight_chrome()
            if self._map_only_dashboard:
                self._scroll_main_to(self._map_widget)
            else:
                self._scroll_main_to(self._mission_list_panel)

                def _focus_mission_table() -> None:
                    if self._mission_table.rowCount() > 0:
                        self._mission_table.setCurrentCell(0, 1)
                    self._mission_table.setFocus()

                QTimer.singleShot(80, _focus_mission_table)
        elif picked is action_analyze:
            report = self._build_analyze_tools_report()
            QMessageBox.information(self, "Analyze Tools", report)
            self._append_log("Menu: Analyze Tools")
        elif picked is action_vehicle:
            self._show_flight_controls_dialog()
            self._append_log("Menu: Vehicle Configuration")
        elif picked is action_settings:
            self._append_log("Menu: Application Settings")
            self._show_application_settings_dialog()
        elif picked is action_toggle_3d:
            current = bool(getattr(self._map_widget, "_is_3d_mode", False))
            self._on_toggle_map_3d(not current)
            self._append_log("Menu: Toggle 3D View")

    def _build_analyze_tools_report(self) -> str:
        # M2 scope: quick mission/link analysis snapshot from live state.
        wp_count = len(getattr(self._map_widget, "_waypoints_model", []))
        status = self._status.text()
        hb = self._hb.text()
        mode = self._top_flight_mode.text()
        battery = self._top_battery.text()
        gps = self._top_gps_status_line()
        mission_distance_m = f"{self._max_telem_dist_m:.0f}"
        return (
            "Live Analysis Snapshot\n\n"
            f"Link: {status}\n"
            f"Heartbeat: {hb}\n"
            f"Flight mode: {mode}\n"
            f"Battery: {battery}\n"
            f"GPS/HDOP: {gps}\n"
            f"Mission waypoints: {wp_count}\n"
            f"Max telemetry distance: {mission_distance_m} m\n\n"
            "Use Plan Flight to edit/upload waypoints and use M2 controls for "
            "mode, takeoff/land, geofence, params, and tile source."
        )

    def _on_tiles_online(self) -> None:
        self._map_widget.activate_online_tiles()
        self._append_log("Map tiles: online source selected")

    def _on_tiles_offline(self) -> None:
        root = QFileDialog.getExistingDirectory(
            self,
            "Select offline tile root (contains z/x/y.png)",
            "",
        )
        if not root:
            return
        self._map_widget.activate_offline_tiles(root)
        self._append_log(f"Map tiles: offline source selected ({root})")
