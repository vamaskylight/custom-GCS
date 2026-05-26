"""Main application window — telemetry dashboard."""

from __future__ import annotations

import struct
import time
import math
from collections import deque
from pathlib import Path

from PySide6.QtCore import QEvent, QObject, QPoint, QSize, Qt, QSettings, QTimer
from PySide6.QtGui import (
    QColor,
    QGuiApplication,
    QIcon,
    QImage,
    QImageReader,
    QKeySequence,
    QMouseEvent,
    QPainter,
    QPixmap,
    QShortcut,
)
from PySide6.QtSvg import QSvgRenderer
from PySide6.QtWidgets import (
    QApplication,
    QBoxLayout,
    QCheckBox,
    QComboBox,
    QDialog,
    QFileDialog,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QInputDialog,
    QMainWindow,
    QMenu,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QScrollBar,
    QSizePolicy,
    QTableWidget,
    QTableWidgetItem,
    QDoubleSpinBox,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
    QListWidget,
    QListWidgetItem,
    QStackedWidget,
    QSpinBox,
)
from pymavlink import mavutil

from vgcs.app.gcs_style import gcs_stylesheet
from vgcs.app.runtime_ui import build_base_font, select_font_profile
from vgcs.mode import AP_COPTER_MODE_MAP, human_mode_name, modes_for_vehicle_type
from vgcs.mission import Waypoint
from vgcs.map import MapWidget
from vgcs.map.map_web_3d import HAS_WEBENGINE as HAS_MAP_WEBENGINE
from vgcs.app.widgets import CompassWidget
from vgcs.link.mavlink_thread import MavlinkThread
from vgcs.video.pipeline import VideoPipeline
from vgcs.video.widgets import CameraControlPanel, SplitVideoPanel
from vgcs.video.camera_control import (
    CompositeGimbalCameraControl,
    MavlinkCameraControl,
    NoopCameraControl,
    SiyiCameraControl,
    SkydroidCameraControl,
    resolve_siyi_host,
    resolve_skydroid_control_hosts,
    resolve_skydroid_host,
)


def _settings_truthy(val: object, default: bool = False) -> bool:
    if val is None:
        return default
    if isinstance(val, bool):
        return val
    s = str(val).strip().lower()
    if s in ("1", "true", "yes", "on"):
        return True
    if s in ("0", "false", "no", "off", ""):
        return False
    return default


def _mavlink_autopilot_label(ap: int) -> str:
    m = mavutil.mavlink
    table = {
        int(m.MAV_AUTOPILOT_GENERIC): "Generic",
        int(m.MAV_AUTOPILOT_ARDUPILOTMEGA): "ArduPilot",
        int(m.MAV_AUTOPILOT_OPENPILOT): "OpenPilot",
        int(m.MAV_AUTOPILOT_PX4): "PX4",
        int(m.MAV_AUTOPILOT_INVALID): "Invalid",
    }
    try:
        return table.get(int(ap), f"Autopilot {int(ap)}")
    except Exception:
        return "—"


def _mavlink_vehicle_type_label(vt: int) -> str:
    m = mavutil.mavlink
    table = {
        int(m.MAV_TYPE_GENERIC): "Generic",
        int(m.MAV_TYPE_FIXED_WING): "Fixed wing",
        int(m.MAV_TYPE_QUADROTOR): "Quadrotor",
        int(m.MAV_TYPE_COAXIAL): "Coaxial",
        int(m.MAV_TYPE_HELICOPTER): "Helicopter",
        int(m.MAV_TYPE_ANTENNA_TRACKER): "Antenna tracker",
        int(m.MAV_TYPE_GCS): "GCS",
        int(m.MAV_TYPE_AIRSHIP): "Airship",
        int(m.MAV_TYPE_FREE_BALLOON): "Balloon",
        int(m.MAV_TYPE_ROCKET): "Rocket",
        int(m.MAV_TYPE_GROUND_ROVER): "Rover",
        int(m.MAV_TYPE_SURFACE_BOAT): "Boat",
        int(m.MAV_TYPE_SUBMARINE): "Submarine",
        int(m.MAV_TYPE_HEXAROTOR): "Hexacopter",
        int(m.MAV_TYPE_OCTOROTOR): "Octocopter",
        int(m.MAV_TYPE_TRICOPTER): "Tricopter",
        int(m.MAV_TYPE_VTOL_DUOROTOR): "VTOL (duo)",
        int(m.MAV_TYPE_VTOL_QUADROTOR): "VTOL (quad)",
        int(m.MAV_TYPE_VTOL_TILTROTOR): "VTOL tilt",
    }
    try:
        return table.get(int(vt), f"Vehicle {int(vt)}")
    except Exception:
        return "—"


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("VGCS — Ground Control Station")
        self.resize(1024, 700)
        self.setMinimumSize(820, 560)

        self._settings = QSettings("VGCS", "VGCS")
        self._thread: MavlinkThread | None = None
        self._camera_control_backend: object | None = None
        self._timeout_s = float(self._settings.value("watchdog_timeout_s", 2.0))
        self._armed_since: float | None = None
        self._heartbeat_seen = False
        self._connect_attempt_active = False
        self._theme_name = str(self._settings.value("ui_theme", "Default"))
        self._theme_colors = self._build_theme_colors(self._theme_name)
        self._compact_ui = self._detect_compact_ui()
        self._last_vehicle_type: int | None = None
        # Shown at most once per link session: resetting on every STANDBY heartbeat caused
        # system_status flicker to re-open the modal right after the user dismissed it.
        self._arm_not_ready_alert_shown = False
        # Heartbeat system_status often stays BOOT/CALIBRATING for a few seconds after link-up;
        # avoid a blocking modal until it persists (reduces false alarms on SITL/real vehicles).
        self._arm_not_ready_since_mono: float | None = None
        self._recent_statustext: deque[str] = deque(maxlen=16)
        self._rid_live_available = False
        self._map_rel_alt_m = 0.0
        self._map_msl_alt_m = 0.0
        self._map_groundspeed_mps = 0.0
        self._map_climb_mps = 0.0
        self._heading = 0.0
        self._max_telem_dist_m = 0.0
        self._home_rel_alt_baseline_m: float | None = None
        self._home_lat: float | None = None
        self._home_lon: float | None = None
        self._home_amsl_m: float | None = None
        self._auto_center_pending = True
        self._plan_hover_speed_mps = 11.18 * 0.44704
        # WebEngine overlay refresh budget (~10 Hz) — avoids full-map flicker from 25–50 Hz MAVLink.
        self._last_map_overlay_refresh_s: float | None = None
        self._mission_upload_pending = False
        # After QInputDialog closes, the same click can “fall through” to the header and reopen Connect.
        self._suppress_header_connect_after_dialog = False

        # M3 video pipeline (camera sources + frame distribution).
        self._video = VideoPipeline(self)

        self._conn_label = QLabel("MAVLink connection string")
        self._conn_edit = QLineEdit()
        self._conn_edit.setText(
            str(self._settings.value("last_connection_string", "udp:127.0.0.1:14550"))
        )
        self._timeout_label = QLabel("Watchdog timeout (s)")
        self._timeout_spin = QDoubleSpinBox()
        self._timeout_spin.setRange(1.0, 10.0)
        self._timeout_spin.setSingleStep(0.5)
        self._timeout_spin.setValue(self._timeout_s)
        self._theme_label = QLabel("State color theme")
        self._theme_combo = QComboBox()
        self._theme_combo.addItems(["Default", "High Contrast", "Dark Friendly"])
        self._theme_combo.setCurrentText(self._theme_name)
        self._mode_label = QLabel("Flight mode cmd")
        self._mode_combo = QComboBox()
        self._mode_combo.addItems(list(AP_COPTER_MODE_MAP.values()))
        self._mode_combo.setCurrentText("LOITER")
        self._btn_set_mode = QPushButton("Set mode")
        self._btn_set_mode.setEnabled(False)
        self._takeoff_alt_spin = QDoubleSpinBox()
        self._takeoff_alt_spin.setRange(1.0, 200.0)
        self._takeoff_alt_spin.setDecimals(1)
        self._takeoff_alt_spin.setValue(15.0)
        self._btn_takeoff = QPushButton("Takeoff")
        self._btn_land = QPushButton("Land")
        self._btn_auto_takeoff = QPushButton("Auto takeoff")
        self._btn_auto_land = QPushButton("Auto land")
        self._btn_emergency_stop = QPushButton("EMERGENCY STOP")
        self._btn_emergency_stop.setToolTip(
            "Immediate motor stop (forced disarm). Use only if the drone is runaway/flyaway."
        )
        self._btn_emergency_stop.setEnabled(False)
        self._btn_emergency_stop.setStyleSheet(
            "QPushButton { background: #b91c1c; color: white; font-weight: 800; }"
            "QPushButton:hover { background: #dc2626; }"
            "QPushButton:disabled { background: #5b5b5b; color: #e5e7eb; }"
        )
        self._btn_takeoff.setEnabled(False)
        self._btn_land.setEnabled(False)
        self._btn_auto_takeoff.setEnabled(False)
        self._btn_auto_land.setEnabled(False)
        self._btn_apply_failsafe_preset = QPushButton("Apply M1 failsafes")
        self._btn_apply_failsafe_preset.setToolTip(
            "Configure basic M1 failsafes on ArduPilot:\n"
            "- GCS disconnect -> RTL\n"
            "- RC failsafe -> RTL\n"
            "- Battery failsafe (LOW) -> RTL, (CRIT) -> Land\n\n"
            "Note: battery failsafe requires BATT_LOW_VOLT or BATT_LOW_MAH to be set on the vehicle."
        )
        self._btn_apply_failsafe_preset.setEnabled(False)
        self._geofence_radius_spin = QDoubleSpinBox()
        self._geofence_radius_spin.setRange(10.0, 5000.0)
        self._geofence_radius_spin.setDecimals(0)
        self._geofence_radius_spin.setValue(80.0)
        self._geofence_radius_spin.setToolTip(
            "Circular geofence radius on the vehicle (meters).\n\n"
            "Plan Flight: Survey / Pattern / Corridor / Structure templates place their "
            "pattern this far north of the vehicle so the grid does not spawn on top of home."
        )
        self._geofence_alt_max_spin = QDoubleSpinBox()
        self._geofence_alt_max_spin.setRange(5.0, 2000.0)
        self._geofence_alt_max_spin.setDecimals(0)
        self._geofence_alt_max_spin.setValue(120.0)
        self._geofence_action_combo = QComboBox()
        self._geofence_action_combo.addItem("RTL (default)", 1.0)
        self._geofence_action_combo.addItem("Land", 2.0)
        self._geofence_action_combo.addItem("None (warn only)", 0.0)
        self._btn_apply_fence = QPushButton("Upload fence")
        self._btn_apply_fence.setEnabled(False)
        self._param_name_combo = QComboBox()
        self._param_name_combo.addItems(
            [
                "WPNAV_SPEED",
                "RTL_ALT",
                "FENCE_ENABLE",
                "FENCE_RADIUS",
                "ARMING_CHECK",
                "ACRO_OPTIONS",
                "ACRO_TRAINER",
                "SIMPLE",
                "SUPER_SIMPLE",
            ]
        )
        self._param_value_spin = QDoubleSpinBox()
        self._param_value_spin.setRange(-100000.0, 100000.0)
        self._param_value_spin.setDecimals(3)
        self._btn_params_refresh = QPushButton("Refresh params")
        self._btn_param_set = QPushButton("Set param")
        self._btn_params_refresh.setEnabled(False)
        self._btn_param_set.setEnabled(False)
        self._btn_tiles_online = QPushButton("Online tiles")
        self._btn_tiles_offline = QPushButton("Offline tiles…")
        self._last_params: dict[str, float] = {}

        self._airmode_check = QCheckBox("AirMode (ACRO_OPTIONS bit0)")
        self._acro_trainer_combo = QComboBox()
        self._acro_trainer_combo.addItems(
            [
                "0: Disabled",
                "1: Auto level",
                "2: Auto level + angle limit",
            ]
        )
        self._btn_apply_acro = QPushButton("Apply Acro options")
        self._btn_apply_acro.setEnabled(False)

        self._simple_check = QCheckBox("Simple mode (SIMPLE bitmask)")
        self._super_simple_check = QCheckBox("Super Simple (SUPER_SIMPLE bitmask)")
        self._btn_apply_simple = QPushButton("Apply Simple options")
        self._btn_apply_simple.setEnabled(False)

        self._btn_connect = QPushButton("Connect")
        self._btn_disconnect = QPushButton("Disconnect")
        self._btn_reset = QPushButton("Reset telemetry")
        self._btn_restore_defaults = QPushButton("Restore defaults")
        self._btn_disconnect.setEnabled(False)

        self._status, self._status_frame = self._make_status_chip("Link", "Disconnected")
        self._hb, self._hb_frame = self._make_status_chip("Heartbeat", "—")
        self._watchdog, self._watchdog_frame = self._make_status_chip(
            "Watchdog", f"Idle · {self._timeout_s:.1f}s"
        )
        self._apply_state_style(self._status, "bad")
        self._apply_state_style(self._hb, "na")
        self._apply_state_style(self._watchdog, "warn")

        self._compass = CompassWidget()
        self._map_widget = MapWidget(video_pipeline=self._video)
        self._map_widget.set_dashboard_mode(True)
        self._telemetry_body = self._build_telemetry_panel()
        self._mission_table_updating = False
        self._mission_table = QTableWidget(0, 5)
        self._mission_table.setHorizontalHeaderLabels(["WP", "Lat", "Lon", "Alt (m)", "Speed (m/s)"])
        self._mission_table.verticalHeader().setVisible(False)
        self._mission_table.setAlternatingRowColors(True)
        self._mission_table.setMinimumHeight(140)
        self._mission_table.horizontalHeader().setStretchLastSection(True)
        self._mission_table.horizontalHeader().setSectionResizeMode(0, self._mission_table.horizontalHeader().ResizeMode.ResizeToContents)
        self._mission_table.horizontalHeader().setSectionResizeMode(1, self._mission_table.horizontalHeader().ResizeMode.Stretch)
        self._mission_table.horizontalHeader().setSectionResizeMode(2, self._mission_table.horizontalHeader().ResizeMode.Stretch)
        self._mission_table.horizontalHeader().setSectionResizeMode(3, self._mission_table.horizontalHeader().ResizeMode.ResizeToContents)
        self._mission_table.horizontalHeader().setSectionResizeMode(4, self._mission_table.horizontalHeader().ResizeMode.ResizeToContents)
        self._mission_table.itemChanged.connect(self._on_mission_table_item_changed)
        self._top_dashboard = self._build_m2_top_dashboard()

        self._log = QTextEdit()
        self._log.setReadOnly(True)
        self._log.setPlaceholderText("MAVLink log…")
        self._log.setMinimumHeight(150)

        self._link_grid = QGridLayout()
        self._link_grid.setVerticalSpacing(6 if self._compact_ui else 8)
        self._link_grid.setColumnStretch(1, 1)

        self._btn_grid = QGridLayout()
        self._btn_grid.setHorizontalSpacing(6 if self._compact_ui else 8)
        self._btn_grid.setVerticalSpacing(6 if self._compact_ui else 8)

        link_box = QGroupBox("Connection")
        link_inner = QVBoxLayout()
        link_inner.addLayout(self._link_grid)
        link_inner.addLayout(self._btn_grid)
        link_box.setLayout(link_inner)
        self._link_box = link_box

        self._status_row = QHBoxLayout()
        self._status_row.setSpacing(8 if self._compact_ui else 12)
        self._status_row.addWidget(self._status_frame, 1)
        self._status_row.addWidget(self._hb_frame, 1)
        self._status_row.addWidget(self._watchdog_frame, 1)

        operations_widget = self._build_m2_operations_layout()
        self._operations_widget = operations_widget
        # Map-only UI (reference style)
        self._map_only_dashboard = True
        self._plan_flight_layer_wanted = _settings_truthy(
            self._settings.value("plan_flight_layer_wanted", False),
            default=False,
        )

        content_panel = QWidget()
        content_panel.setObjectName("contentRoot")
        self._content_layout = QVBoxLayout()
        self._content_layout.setSpacing(8 if self._compact_ui else 12)
        self._content_layout.addWidget(self._top_dashboard)
        # Stretch so the map/operations row consumes all space below the header (scroll content fills viewport).
        self._content_layout.addWidget(operations_widget, 1)
        self._content_layout.addWidget(link_box)
        self._m2_controls_panel = self._build_m2_controls_panel()
        self._content_layout.addWidget(self._m2_controls_panel)
        self._content_layout.addLayout(self._status_row)
        self._mission_list_panel = self._build_mission_list_panel()
        self._content_layout.addWidget(self._mission_list_panel)
        self._content_layout.addWidget(self._log)
        self._content_layout.setContentsMargins(0, 0, 0, 0)
        content_panel.setLayout(self._content_layout)

        self._scroll = QScrollArea()
        self._scroll.setWidgetResizable(True)
        self._scroll.setFrameShape(QFrame.NoFrame)
        self._scroll.setWidget(content_panel)
        self._scroll.viewport().setObjectName("contentViewport")

        central = QWidget()
        central.setObjectName("centralRoot")
        layout = QVBoxLayout()
        self._central_layout = layout
        layout.addWidget(self._scroll)
        # Keep the header flush to the window edges (top + full-width),
        # as requested by the UI direction.
        layout.setContentsMargins(0, 0, 0, 0)
        central.setLayout(layout)
        self.setCentralWidget(central)

        self._btn_connect.clicked.connect(self._on_connect)
        self._btn_disconnect.clicked.connect(self._on_disconnect)
        self._btn_reset.clicked.connect(self._on_reset_telemetry)
        self._btn_restore_defaults.clicked.connect(self._on_restore_defaults)
        self._btn_set_mode.clicked.connect(self._on_set_mode)
        self._btn_takeoff.clicked.connect(self._on_takeoff)
        self._btn_land.clicked.connect(self._on_land)
        self._btn_auto_takeoff.clicked.connect(self._on_auto_takeoff)
        self._btn_auto_land.clicked.connect(self._on_auto_land)
        self._btn_emergency_stop.clicked.connect(self._on_emergency_motor_stop)
        self._btn_apply_failsafe_preset.clicked.connect(self._on_apply_m1_failsafes)
        self._btn_apply_fence.clicked.connect(self._on_upload_fence)
        self._btn_params_refresh.clicked.connect(self._on_params_refresh)
        self._btn_param_set.clicked.connect(self._on_param_set)
        self._btn_tiles_online.clicked.connect(self._on_tiles_online)
        self._btn_tiles_offline.clicked.connect(self._on_tiles_offline)
        self._btn_apply_acro.clicked.connect(self._on_apply_acro_options)
        self._btn_apply_simple.clicked.connect(self._on_apply_simple_options)
        self._timeout_spin.valueChanged.connect(self._on_timeout_changed)
        self._theme_combo.currentTextChanged.connect(self._on_theme_changed)
        self._map_widget.waypoints_changed.connect(self._on_map_waypoints_changed)
        self._map_widget.mission_upload_requested.connect(self._on_mission_upload_requested)
        self._map_widget.mission_download_requested.connect(self._on_mission_download_requested)
        self._map_widget.geofence_upload_requested.connect(self._on_map_geofence_requested)
        self._map_widget.menu_requested.connect(self._on_map_menu_requested)
        self._map_widget.connect_requested.connect(self._on_map_connect_requested)
        self._map_widget.takeoff_requested.connect(self._on_takeoff)
        self._map_widget.return_requested.connect(self._on_map_return_requested)
        self._map_widget.plan_tool_requested.connect(self._on_plan_tool_requested)
        self._map_widget.plan_action_requested.connect(self._on_plan_flight_action)
        self._map_widget.plan_flight_exited.connect(self._on_plan_flight_exited)
        self._map_widget.waypoints_changed.connect(self._sync_plan_flight_chrome)
        self._map_widget.map_page_ready.connect(self._on_map_page_ready)
        self._map_widget.plan_mission_panel_changed.connect(self._on_plan_mission_panel_changed)
        self._map_widget.toggle_3d_requested.connect(self._on_map_toggle_3d_requested)
        self._map_widget.map_3d_mode_changed.connect(self._on_map_3d_mode_changed)
        self._map_widget.mission_start_requested.connect(self._on_map_mission_start_requested)

        self._flight_timer = QTimer(self)
        self._flight_timer.setInterval(1000)
        self._flight_timer.timeout.connect(self._on_flight_timer_tick)
        self._flight_timer.start()
        self._dev_reload_shortcut = QShortcut(QKeySequence("Ctrl+Shift+R"), self)
        self._dev_reload_shortcut.activated.connect(self._on_dev_reload)
        self._map_3d_shortcut = QShortcut(QKeySequence("Ctrl+3"), self)
        self._map_3d_shortcut.activated.connect(self._on_toggle_map_3d_shortcut)
        self._restore_window_geometry()
        self._fit_to_screen()
        self._apply_responsive_layout(self.width())
        self._set_preconnect_dashboard_mode(True)
        self._set_map_only_dashboard_mode(self._map_only_dashboard)
        # Default `#linkBanner` CSS tint (git e48c1a7) — not the red “communication lost” palette.
        self._set_dashboard_flight_status(
            "",
            "Disconnected - Click to manually connect 💬",
        )
        # Skydroid/SIYI gimbal polling must run before MAVLink connect (M7 Target reports).
        QTimer.singleShot(800, self._set_runtime_camera_control)

    def _wire_camera_control(self, cc: object) -> None:
        wrapped = CompositeGimbalCameraControl(cc, self._thread)
        self._camera_control_backend = wrapped
        self._map_widget.set_camera_control(wrapped)
        try:
            self._camera_panel.set_camera_control(wrapped)
        except Exception:
            pass

    def _detect_compact_ui(self) -> bool:
        screen = QGuiApplication.primaryScreen()
        if screen is None:
            return False
        area = screen.availableGeometry()
        return area.height() <= 800 or area.width() <= 1366

    def _apply_responsive_layout(self, width: int) -> None:
        narrow = width < 1120

        while self._link_grid.count():
            self._link_grid.takeAt(0)
        while self._btn_grid.count():
            self._btn_grid.takeAt(0)

        if narrow:
            self._link_grid.addWidget(self._conn_label, 0, 0)
            self._link_grid.addWidget(self._conn_edit, 1, 0, 1, 4)
            self._link_grid.addWidget(self._timeout_label, 2, 0)
            self._link_grid.addWidget(self._timeout_spin, 2, 1)
            self._link_grid.addWidget(self._theme_label, 2, 2)
            self._link_grid.addWidget(self._theme_combo, 2, 3)
            self._link_grid.addWidget(self._mode_label, 3, 0)
            self._link_grid.addWidget(self._mode_combo, 3, 1, 1, 2)
            self._link_grid.addWidget(self._btn_set_mode, 3, 3)

            self._btn_grid.addWidget(self._btn_connect, 0, 0)
            self._btn_grid.addWidget(self._btn_disconnect, 0, 1)
            self._btn_grid.addWidget(self._btn_reset, 1, 0)
            self._btn_grid.addWidget(self._btn_restore_defaults, 1, 1)
            self._btn_grid.setColumnStretch(0, 1)
            self._btn_grid.setColumnStretch(1, 1)
            self._center_row.setDirection(QBoxLayout.TopToBottom)
            self._footer_row.setDirection(QBoxLayout.TopToBottom)
            self._after_responsive_layout_changed()
        else:
            self._link_grid.addWidget(self._conn_label, 0, 0)
            self._link_grid.addWidget(self._conn_edit, 0, 1, 1, 3)
            self._link_grid.addWidget(self._timeout_label, 1, 0)
            self._link_grid.addWidget(self._timeout_spin, 1, 1)
            self._link_grid.addWidget(self._theme_label, 1, 2)
            self._link_grid.addWidget(self._theme_combo, 1, 3)
            self._link_grid.addWidget(self._mode_label, 2, 0)
            self._link_grid.addWidget(self._mode_combo, 2, 1, 1, 2)
            self._link_grid.addWidget(self._btn_set_mode, 2, 3)

            self._btn_grid.addWidget(self._btn_connect, 0, 0)
            self._btn_grid.addWidget(self._btn_disconnect, 0, 1)
            self._btn_grid.addWidget(self._btn_reset, 0, 2)
            self._btn_grid.addWidget(self._btn_restore_defaults, 0, 3)
            self._btn_grid.setColumnStretch(0, 1)
            self._btn_grid.setColumnStretch(1, 1)
            self._btn_grid.setColumnStretch(2, 1)
            self._btn_grid.setColumnStretch(3, 1)
            self._center_row.setDirection(QBoxLayout.LeftToRight)
            self._footer_row.setDirection(QBoxLayout.LeftToRight)
            self._after_responsive_layout_changed()

    def _after_responsive_layout_changed(self) -> None:
        if self._map_only_dashboard and self._plan_flight_layer_wanted:
            def _pin() -> None:
                self._scroll.verticalScrollBar().setValue(0)
                self._map_widget.set_plan_flight_visible(True)

            QTimer.singleShot(0, _pin)

    def _make_value_label(self) -> QLabel:
        lab = QLabel("—")
        lab.setObjectName("telemetryValue")
        lab.setTextInteractionFlags(Qt.TextSelectableByMouse)
        lab.setMinimumWidth(120)
        return lab

    def _make_status_chip(self, title: str, initial: str) -> tuple[QLabel, QFrame]:
        frame = QFrame()
        frame.setObjectName("statusChip")
        frame.setMinimumWidth(120 if self._compact_ui else 180)
        lay = QVBoxLayout()
        lay.setSpacing(4)
        t = QLabel(title)
        t.setObjectName("statusChipTitle")
        v = QLabel(initial)
        v.setObjectName("statusChipValue")
        v.setWordWrap(False)
        lay.addWidget(t)
        lay.addWidget(v)
        frame.setLayout(lay)
        return v, frame

    def _make_top_chip(self, title: str, initial: str = "—") -> tuple[QLabel, QFrame]:
        frame = QFrame()
        frame.setObjectName("statusChip")
        min_w = 128 if self._compact_ui else 154
        frame.setMinimumWidth(min_w)
        lay = QVBoxLayout()
        lay.setContentsMargins(10, 8, 10, 8)
        lay.setSpacing(4)
        t = QLabel(title)
        t.setObjectName("statusChipTitle")
        t.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        v = QLabel(initial)
        v.setObjectName("statusChipValue")
        v.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        v.setWordWrap(False)
        lay.addWidget(t)
        lay.addWidget(v)
        lay.addStretch(1)
        frame.setLayout(lay)
        return v, frame

    def _hdr_sep_widget(self) -> QFrame:
        """Legacy Web `.hdrSep`: 1×~24px vertical rule between `.hdrPill` items (e48c1a7)."""
        sep = QFrame()
        sep.setObjectName("hdrSep")
        sep.setFixedSize(1, 26)
        sep.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        return sep

    def _header_icons_dir(self) -> Path:
        return Path(__file__).resolve().parents[1] / "assets" / "header_icons"

    def _header_icon_pixmap(self, filename: str, size: int = 22) -> QPixmap:
        """Rasterize SVG from git `vgcs/assets/header_icons/` (same paths as Web template)."""
        path = self._header_icons_dir() / filename
        if not path.exists():
            return QPixmap()
        try:
            renderer = QSvgRenderer(str(path))
            img = QImage(size, size, QImage.Format.Format_ARGB32_Premultiplied)
            img.fill(Qt.GlobalColor.transparent)
            painter = QPainter(img)
            renderer.render(painter)
            painter.end()
            return QPixmap.fromImage(img)
        except Exception:
            return QPixmap()

    def _make_hdr_icon_pill(
        self,
        icon_filename: str,
        value: QLabel,
        *,
        icon_size: int = 22,
        min_w: int = 72,
    ) -> QWidget:
        """Legacy `.hdrPill`: icon + text row (git e48c1a7 map_widget HTML)."""
        wrap = QWidget()
        wrap.setObjectName("hdrPill")
        # Floor: icon + spacing so an empty label never collapses the pill; content sets real width.
        wrap.setMinimumWidth(max(min_w, icon_size + 6 + 4))
        wrap.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        row = QHBoxLayout(wrap)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(6)
        ic = QLabel()
        ic.setFixedSize(icon_size, icon_size)
        pm = self._header_icon_pixmap(icon_filename, icon_size)
        if not pm.isNull():
            ic.setPixmap(pm)
        value.setObjectName("hdrPillValue")
        value.setAlignment(Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft)
        value.setWordWrap(False)
        row.addWidget(ic, 0, Qt.AlignmentFlag.AlignVCenter)
        row.addWidget(value, 1, Qt.AlignmentFlag.AlignVCenter)
        return wrap

    def _make_hdr_gps_pill_widget(self) -> QWidget:
        """GPS pill: `gps.svg` + `.hdrTinyStack` two-line column (Web map_widget)."""
        wrap = QWidget()
        wrap.setObjectName("hdrPill")
        wrap.setMinimumWidth(96 if self._compact_ui else 104)
        wrap.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        row = QHBoxLayout(wrap)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(6)
        ic = QLabel()
        ic.setFixedSize(22, 22)
        pm = self._header_icon_pixmap("gps.svg", 22)
        if not pm.isNull():
            ic.setPixmap(pm)
        self._top_gps_sat = QLabel("—")
        self._top_gps_sat.setObjectName("hdrGpsStackLine")
        self._top_gps_hdop = QLabel("—")
        self._top_gps_hdop.setObjectName("hdrGpsStackLine")
        self._top_gps_sat.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        self._top_gps_hdop.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        stack = QVBoxLayout()
        stack.setContentsMargins(0, 0, 0, 0)
        stack.setSpacing(0)
        stack.addWidget(self._top_gps_sat)
        stack.addWidget(self._top_gps_hdop)
        row.addWidget(ic, 0, Qt.AlignmentFlag.AlignVCenter)
        row.addLayout(stack, 1)
        return wrap

    def _top_gps_status_line(self) -> str:
        """One-line GPS summary for popups/exports (sat line + HDOP line)."""
        return f"{self._top_gps_sat.text()} / {self._top_gps_hdop.text()}"

    def _apply_link_banner_palette(self, state: str) -> None:
        """Tint full `#linkBanner` like Web `setFlightStatus()` — not a separate arm rectangle.

        Empty ``state`` matches shipped CSS `#linkBanner { background: rgba(24,30,40,0.95); … }`
        (idle disconnected). Use ``red`` only for the JS ``else`` branch (communication lost).
        """
        st = (state or "").strip().lower()
        if st == "green":
            bg = "rgba(24, 82, 38, 0.96)"
            bd = "rgba(94, 214, 119, 0.95)"
            fg = "#e8ffe8"
            muted = "rgba(232, 255, 232, 0.58)"
        elif st == "yellow":
            bg = "rgba(120, 95, 24, 0.96)"
            bd = "rgba(247, 211, 92, 0.95)"
            fg = "#fff7dd"
            muted = "rgba(255, 247, 221, 0.58)"
        elif st == "red":
            bg = "rgba(124, 24, 24, 0.96)"
            bd = "rgba(245, 99, 99, 0.95)"
            fg = "#ffe8e8"
            muted = "rgba(255, 232, 232, 0.58)"
        else:
            bg = "rgba(24, 30, 40, 0.96)"
            bd = "rgba(72, 86, 110, 0.9)"
            fg = "#dbe3f3"
            muted = "rgba(244, 247, 255, 0.52)"

        qss = "\n".join(
            (
                f"QFrame#headerBar {{ background-color: {bg}; border-bottom: 1px solid {bd}; }}",
                f"QLabel#hdrPillTitle {{ color: {muted}; }}",
                f"QLabel#hdrPillValue {{ color: {fg}; }}",
                f"QLabel#hdrGpsStackLine {{ color: {fg}; }}",
                "QPushButton#headerFlightChipBtn { background: transparent; border: none; "
                f"color: {fg}; font-weight: 700; font-size: 14px; padding: 2px 4px; }}",
                "QPushButton#headerFlightChipBtn:hover { background-color: rgba(255,255,255,0.08); }",
                f"QLabel#linkBannerText {{ color: {fg}; font-size: 14px; font-weight: 600; }}",
            )
        )
        self._top_dashboard.setStyleSheet(qss)
        self._logo_btn.setStyleSheet(
            f"QPushButton {{ background: transparent; border: none; padding: 0; color: {fg}; "
            f"font-size: 15px; font-weight: 600; }}"
            f"QPushButton:hover {{ background-color: rgba(255,255,255,0.06); }}"
        )

    def _build_m2_top_dashboard(self) -> QFrame:
        bar = QFrame()
        bar.setObjectName("headerBar")
        bar.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        shell = QVBoxLayout()
        shell.setContentsMargins(12, 8, 12, 8)
        shell.setSpacing(0)
        header_outer = QHBoxLayout()
        header_outer.setContentsMargins(0, 0, 0, 0)
        header_outer.setSpacing(12 if self._compact_ui else 16)
        # Logo: match legacy Web #linkBannerLogo (height ~28px); scale slightly up for HiDPI.
        # We read intrinsic WxH from the file, then setScaledSize to a proportional
        # box (max edge capped). Final display scale uses scaledToHeight (uniform).
        logo_target_h = 28 if self._compact_ui else 32
        logo_decode_max = 2400  # longest edge for initial decode (memory bound)
        # Two-line hdr pills; Web banner min-height 46px — stack needs slightly more row pixels.
        chip_row_h = 52

        self._logo_btn = QPushButton("VGCS Logo")
        self._logo_btn.setObjectName("linkBannerLogo")
        self._logo_btn.clicked.connect(self._on_logo_menu)
        self._logo_btn.setFlat(True)
        self._logo_btn.setCursor(Qt.PointingHandCursor)
        self._logo_btn.setStyleSheet(
            "QPushButton { background: transparent; border: none; padding: 0; "
            "color: #dbe3f3; font-size: 15px; font-weight: 600; }"
            "QPushButton:hover { background: transparent; border: none; color: #f4f7ff; }"
            "QPushButton:pressed { background: transparent; border: none; }"
        )
        # Legacy Web `__LOGO_SRC__` → shipped asset (see git e48c1a7 map_widget template).
        logo_paths = (
            Path(__file__).resolve().parents[1] / "assets" / "Vama Logo.png",
            Path(__file__).resolve().parents[2] / "Vama Logo New.png",
            Path(__file__).resolve().parents[1] / "assets" / "vama_logo.jpg",
        )
        for logo_path in logo_paths:
            if not logo_path.exists():
                continue
            reader = QImageReader(str(logo_path))
            # Qt 6 defaults ~256MB allocation guard on IHDR × bpp before scaled decode.
            # Shipped `Vama Logo.png` has a very large declared canvas (~15k×6k) but small IDAT;
            # without this, read() returns null and the UI falls back to "VGCS Logo" text.
            reader.setAllocationLimit(512)
            reader.setAutoTransform(True)
            sz = reader.size()
            if sz.isValid():
                decode_sz = self._logo_scaled_decode_size(
                    sz.width(), sz.height(), logo_decode_max
                )
                reader.setScaledSize(decode_sz)
            elif logo_path.suffix.lower() == ".png":
                hdr = self._read_png_dimensions(logo_path)
                if hdr is not None:
                    decode_sz = self._logo_scaled_decode_size(
                        hdr[0], hdr[1], logo_decode_max
                    )
                    reader.setScaledSize(decode_sz)
            image = reader.read()
            if image.isNull():
                continue
            image = image.scaledToHeight(
                logo_target_h,
                Qt.TransformationMode.SmoothTransformation,
            )
            # Make near-black background pixels transparent so the logo
            # blends into the header instead of showing a black box.
            image = image.convertToFormat(QImage.Format_RGBA8888)
            w, h = image.width(), image.height()
            for y in range(h):
                for x in range(w):
                    c = image.pixelColor(x, y)
                    if c.red() < 8 and c.green() < 8 and c.blue() < 8:
                        image.setPixelColor(x, y, QColor(0, 0, 0, 0))
            pix = QPixmap.fromImage(image)
            icon = QIcon(pix)
            self._logo_btn.setIcon(icon)
            self._logo_btn.setIconSize(pix.size())
            self._logo_btn.setFixedSize(pix.size())
            self._logo_btn.setText("")
            break
        # Placeholder for layouts that still reference `_vehicle_msg_frame` — Web showed vehicle in `#linkBanner` only.
        self._vehicle_msg_frame = QWidget()
        self._vehicle_msg_frame.setFixedSize(0, 0)

        flight_wrap = QWidget()
        flight_wrap.setObjectName("hdrPill")
        # Content-sized: avoid MinimumExpanding + wide mins (left empty space in the banner).
        flight_wrap.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        fw_lay = QVBoxLayout(flight_wrap)
        fw_lay.setContentsMargins(0, 0, 4, 0)
        fw_lay.setSpacing(0)
        self._flight_status_btn = QPushButton("NOT READY TO ARM")
        self._flight_status_btn.setObjectName("headerFlightChipBtn")
        self._flight_status_btn.setCursor(Qt.PointingHandCursor)
        self._flight_status_btn.setSizePolicy(
            QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed
        )
        # Legacy Web `#linkBanner`: any header click except logo → VGCS_CONNECT_REQUEST (git e48c1a7).
        self._flight_status_btn.clicked.connect(self._on_map_connect_requested)
        fw_lay.addWidget(self._flight_status_btn)

        # git e48c1a7 map_widget: hold.svg → mode, link.svg → vehicle msg, gps.svg → stack, battery, remote_id, hdrMapModeBtn
        self._top_flight_mode = QLabel("—")
        mode_frame = self._make_hdr_icon_pill(
            "hold.svg", self._top_flight_mode, min_w=72 if self._compact_ui else 80
        )

        self._top_vehicle_msg = QLabel("—")
        self._top_vehicle_msg.setWordWrap(False)
        # No max width — long status (e.g. parameter download) was clipped; row scrolls if needed.
        self._top_vehicle_msg.setSizePolicy(
            QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed
        )
        vehicle_pill = self._make_hdr_icon_pill(
            "link.svg",
            self._top_vehicle_msg,
            icon_size=26,
            min_w=40 if self._compact_ui else 48,
        )

        gps_frame = self._make_hdr_gps_pill_widget()

        self._top_battery = QLabel("—")
        bat_frame = self._make_hdr_icon_pill(
            "battery.svg", self._top_battery, min_w=88 if self._compact_ui else 96
        )

        self._top_remote_id = QLabel("N/A")
        rid_frame = self._make_hdr_icon_pill(
            "remote_id.svg", self._top_remote_id, min_w=88 if self._compact_ui else 96
        )

        self._hdr_map_mode_btn = QPushButton("3D")
        self._hdr_map_mode_btn.setObjectName("hdrMapModeBtn")
        self._hdr_map_mode_btn.setFixedHeight(26)
        self._hdr_map_mode_btn.setCursor(Qt.PointingHandCursor)
        self._hdr_map_mode_btn.clicked.connect(self._on_map_toggle_3d_requested)

        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(0)
        left_layout.addWidget(self._logo_btn, 0, Qt.AlignLeft | Qt.AlignVCenter)

        chip_strip = QWidget()
        chip_strip.setFixedHeight(chip_row_h)
        chip_lay = QHBoxLayout(chip_strip)
        chip_lay.setContentsMargins(0, 0, 0, 0)
        chip_lay.setSpacing(12)

        chip_lay.addWidget(flight_wrap, 0, Qt.AlignVCenter)
        chip_lay.addWidget(self._hdr_sep_widget(), 0, Qt.AlignVCenter)
        chip_lay.addWidget(mode_frame, 0, Qt.AlignVCenter)
        chip_lay.addWidget(self._hdr_sep_widget(), 0, Qt.AlignVCenter)
        chip_lay.addWidget(vehicle_pill, 0, Qt.AlignVCenter)
        chip_lay.addWidget(self._hdr_sep_widget(), 0, Qt.AlignVCenter)
        chip_lay.addWidget(gps_frame, 0, Qt.AlignVCenter)
        chip_lay.addWidget(self._hdr_sep_widget(), 0, Qt.AlignVCenter)
        chip_lay.addWidget(bat_frame, 0, Qt.AlignVCenter)
        chip_lay.addWidget(self._hdr_sep_widget(), 0, Qt.AlignVCenter)
        chip_lay.addWidget(rid_frame, 0, Qt.AlignVCenter)
        chip_lay.addStretch(1)

        for w in (flight_wrap, mode_frame, vehicle_pill, gps_frame, bat_frame, rid_frame):
            w.setFixedHeight(chip_row_h)

        self._chip_strip = chip_strip

        header_scroll = QScrollArea()
        self._header_chip_scroll = header_scroll
        header_scroll.setObjectName("headerChipScroll")
        header_scroll.setWidget(chip_strip)
        # Let the chip row use the viewport width: trailing stretch absorbs slack; scroll if content overflows.
        header_scroll.setWidgetResizable(True)
        header_scroll.setFrameShape(QFrame.NoFrame)
        header_scroll.setHorizontalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAsNeeded
        )
        header_scroll.setVerticalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAlwaysOff
        )
        header_scroll.setMinimumHeight(chip_row_h)
        header_scroll.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        # Legacy Web: `#linkBannerDisconnected` vs `#linkBannerConnected` (git e48c1a7 — only yellow/green show pills).
        self._banner_disconnected_wrap = QWidget()
        self._banner_disconnected_wrap.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._banner_disconnected_wrap.setMinimumHeight(chip_row_h)
        dd_lay = QHBoxLayout(self._banner_disconnected_wrap)
        dd_lay.setContentsMargins(0, 0, 0, 0)
        dd_lay.setSpacing(0)
        self._link_banner_text = QLabel("Disconnected - Click to manually connect 💬")
        self._link_banner_text.setObjectName("linkBannerText")
        self._link_banner_text.setWordWrap(False)
        self._link_banner_text.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        dd_lay.addWidget(self._link_banner_text, 1, Qt.AlignmentFlag.AlignVCenter)

        self._banner_connected_wrap = QWidget()
        self._banner_connected_wrap.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._banner_connected_wrap.setMinimumHeight(chip_row_h)
        cc_lay = QHBoxLayout(self._banner_connected_wrap)
        cc_lay.setContentsMargins(0, 0, 0, 0)
        cc_lay.setSpacing(0)
        cc_lay.addWidget(header_scroll, 1)

        self._header_banner_stack = QStackedWidget()
        self._header_banner_stack.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._header_banner_stack.addWidget(self._banner_disconnected_wrap)
        self._header_banner_stack.addWidget(self._banner_connected_wrap)

        header_outer.addWidget(left_panel, 0, Qt.AlignVCenter)
        header_outer.addWidget(self._header_banner_stack, 1, Qt.AlignVCenter)
        header_outer.addWidget(self._hdr_map_mode_btn, 0, Qt.AlignVCenter)

        # Web padding 8+8px; inner row chip_row_h — fixed total bar height.
        _hdr_pad_v = shell.contentsMargins().top() + shell.contentsMargins().bottom()
        bar.setFixedHeight(chip_row_h + _hdr_pad_v)
        shell.addLayout(header_outer)
        bar.setLayout(shell)
        self._install_header_connect_click_filters(bar)
        return bar

    def _install_header_connect_click_filters(self, header_bar: QFrame) -> None:
        """Web `#linkBanner` click: open connect dialog except `#linkBannerLogo` / `#hdrMapModeBtn`."""
        hdr_btn = getattr(self, "_hdr_map_mode_btn", None)
        logo_btn = getattr(self, "_logo_btn", None)
        arm_btn = getattr(self, "_flight_status_btn", None)
        for w in list(header_bar.findChildren(QWidget)) + [header_bar]:
            if hdr_btn is not None and (w is hdr_btn or hdr_btn.isAncestorOf(w)):
                continue
            if logo_btn is not None and (w is logo_btn or logo_btn.isAncestorOf(w)):
                continue
            if arm_btn is not None and (w is arm_btn or arm_btn.isAncestorOf(w)):
                continue
            w.installEventFilter(self)

    def eventFilter(self, obj: QObject, event: QEvent) -> bool:
        if event.type() != QEvent.Type.MouseButtonPress:
            return super().eventFilter(obj, event)
        if not isinstance(event, QMouseEvent):
            return super().eventFilter(obj, event)
        if event.button() != Qt.MouseButton.LeftButton:
            return super().eventFilter(obj, event)

        header_bar = getattr(self, "_top_dashboard", None)
        if header_bar is None or not isinstance(obj, QWidget):
            return super().eventFilter(obj, event)
        if obj is not header_bar and not header_bar.isAncestorOf(obj):
            return super().eventFilter(obj, event)
        hdr_btn = getattr(self, "_hdr_map_mode_btn", None)
        if hdr_btn is not None and (obj is hdr_btn or hdr_btn.isAncestorOf(obj)):
            return super().eventFilter(obj, event)
        logo_btn = getattr(self, "_logo_btn", None)
        if logo_btn is not None and (obj is logo_btn or logo_btn.isAncestorOf(obj)):
            return super().eventFilter(obj, event)
        if isinstance(obj, QScrollBar):
            return super().eventFilter(obj, event)
        self._on_map_connect_requested()
        return super().eventFilter(obj, event)

    @staticmethod
    def _logo_scaled_decode_size(ow: int, oh: int, max_edge: int) -> QSize:
        """QSize for decode whose width/height ratio matches the source image."""
        if ow <= 0 or oh <= 0:
            return QSize(max_edge, max_edge)
        if max(ow, oh) <= max_edge:
            return QSize(ow, oh)
        s = max_edge / float(max(ow, oh))
        nw = max(1, int(round(ow * s)))
        nh = max(1, int(round(oh * s)))
        return QSize(nw, nh)

    def _read_png_dimensions(self, path: Path) -> tuple[int, int] | None:
        try:
            with path.open("rb") as f:
                header = f.read(24)
            if len(header) < 24:
                return None
            if header[:8] != b"\x89PNG\r\n\x1a\n":
                return None
            width, height = struct.unpack(">II", header[16:24])
            return int(width), int(height)
        except Exception:
            return None

    def _menu_icon(self, filename: str) -> QIcon:
        path = Path(__file__).resolve().parents[1] / "assets" / "menu_icons" / filename
        if path.exists():
            return QIcon(str(path))
        return QIcon()

    def _build_mission_list_panel(self) -> QGroupBox:
        box = QGroupBox("Mission waypoints")
        lay = QVBoxLayout()
        lay.addWidget(self._mission_table)
        box.setLayout(lay)
        return box

    def _build_m2_operations_layout(self) -> QWidget:
        root = QWidget()
        root.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        v = QVBoxLayout()
        self._operations_layout = v
        v.setSpacing(8 if self._compact_ui else 12)
        v.setContentsMargins(0, 0, 0, 0)

        self._center_row = QBoxLayout(QBoxLayout.LeftToRight)
        self._center_row.setSpacing(8 if self._compact_ui else 12)
        # Vehicle message lives in Web-style `#linkBanner` only (git e48c1a7); keep zero-width stub for compat.
        self._center_row.addWidget(self._vehicle_msg_frame, 0, Qt.AlignTop)
        self._center_row.addWidget(self._map_widget, 1)
        self._camera_panel = self._build_camera_control_panel()
        self._camera_panel.follow_triggered.connect(self._map_widget.set_video_follow_enabled)
        self._map_widget.video_follow_enabled_changed.connect(self._camera_panel.sync_video_follow_toggle)
        # Create the camera control backend used by split/video transforms,
        # but do not add it to the visible layout (removes the "Camera Control" UI panel).
        try:
            self._camera_panel.setVisible(False)
        except Exception:
            pass

        self._footer_row = QBoxLayout(QBoxLayout.LeftToRight)
        self._footer_row.setSpacing(8 if self._compact_ui else 12)
        self._split_camera_panel = self._build_split_camera_panel()
        self._footer_row.addWidget(self._split_camera_panel, 1)
        self._footer_row.addWidget(self._build_primary_flight_footer(), 1)
        self._footer_row.addWidget(self._build_compass_footer(), 1)
        self._footer_row.addWidget(self._build_nav_system_footer(), 1)
        self._footer_widget = QWidget()
        self._footer_widget.setLayout(self._footer_row)

        v.addLayout(self._center_row, 1)
        v.addWidget(self._footer_widget)
        root.setLayout(v)
        return root

    def _build_camera_control_panel(self) -> QGroupBox:
        panel = CameraControlPanel(self._video, self)
        # Keep existing 3D map toggle here (matches current layout expectations).
        row = QHBoxLayout()
        self._btn_map_3d = QPushButton("3D View")
        self._btn_map_3d.setCheckable(True)
        self._btn_map_3d.clicked.connect(self._on_toggle_map_3d)
        row.addWidget(self._btn_map_3d)
        row.addStretch(1)
        panel.layout().addItem(row)  # type: ignore[union-attr]
        return panel

    def _build_split_camera_panel(self) -> QGroupBox:
        # 2x2 split preview for up to 4 cameras.
        return SplitVideoPanel(self._video, self._camera_panel, self)

    def _build_primary_flight_footer(self) -> QGroupBox:
        box = QGroupBox("Primary Flight Data")
        lay = QVBoxLayout()
        self._footer_primary = QLabel("Alt — | Speed — | Time 00:00")
        self._footer_primary.setObjectName("telemetryValue")
        lay.addWidget(self._footer_primary)
        lay.addStretch()
        box.setLayout(lay)
        return box

    def _build_compass_footer(self) -> QGroupBox:
        box = QGroupBox("Compass/Attitude")
        lay = QVBoxLayout()
        lay.addWidget(self._compass, 0, Qt.AlignHCenter)
        box.setLayout(lay)
        return box

    def _build_nav_system_footer(self) -> QGroupBox:
        box = QGroupBox("Navigation System")
        lay = QVBoxLayout()
        self._footer_nav = QLabel("GPS — | HDOP — | RC —")
        self._footer_nav.setObjectName("telemetryValue")
        lay.addWidget(self._footer_nav)
        lay.addStretch()
        box.setLayout(lay)
        return box

    def _build_m2_controls_panel(self) -> QGroupBox:
        box = QGroupBox("M2 controls")
        lay = QGridLayout()
        lay.setHorizontalSpacing(8 if self._compact_ui else 10)
        lay.setVerticalSpacing(6 if self._compact_ui else 8)

        lay.addWidget(QLabel("Takeoff alt (m)"), 0, 0)
        lay.addWidget(self._takeoff_alt_spin, 0, 1)
        lay.addWidget(self._btn_takeoff, 0, 2)
        lay.addWidget(self._btn_land, 0, 3)
        lay.addWidget(self._btn_auto_takeoff, 0, 4)
        lay.addWidget(self._btn_auto_land, 0, 5)
        lay.addWidget(self._btn_emergency_stop, 0, 6, 1, 2)
        lay.addWidget(self._btn_apply_failsafe_preset, 0, 8, 1, 2)

        lay.addWidget(QLabel("Fence radius (m)"), 1, 0)
        lay.addWidget(self._geofence_radius_spin, 1, 1)
        lay.addWidget(QLabel("Fence alt max"), 1, 2)
        lay.addWidget(self._geofence_alt_max_spin, 1, 3)
        lay.addWidget(QLabel("Fence action"), 1, 4)
        lay.addWidget(self._geofence_action_combo, 1, 5)
        lay.addWidget(self._btn_apply_fence, 1, 6)

        lay.addWidget(QLabel("Param"), 2, 0)
        lay.addWidget(self._param_name_combo, 2, 1)
        lay.addWidget(QLabel("Value"), 2, 2)
        lay.addWidget(self._param_value_spin, 2, 3)
        lay.addWidget(self._btn_params_refresh, 2, 4)
        lay.addWidget(self._btn_param_set, 2, 5)
        lay.addWidget(self._btn_tiles_online, 3, 0, 1, 2)
        lay.addWidget(self._btn_tiles_offline, 3, 2, 1, 2)
        lay.addWidget(QLabel("Acro"), 4, 0)
        lay.addWidget(self._airmode_check, 4, 1, 1, 2)
        lay.addWidget(self._acro_trainer_combo, 4, 3)
        lay.addWidget(self._btn_apply_acro, 4, 4, 1, 2)
        lay.addWidget(QLabel("Simple"), 5, 0)
        lay.addWidget(self._simple_check, 5, 1, 1, 2)
        lay.addWidget(self._super_simple_check, 5, 3)
        lay.addWidget(self._btn_apply_simple, 5, 4, 1, 2)
        box.setLayout(lay)
        return box

    def _set_preconnect_dashboard_mode(self, enabled: bool) -> None:
        """
        Pre-connect visual mode: map-centric dashboard like reference image.
        Full M2 operator panels are shown after link-up.
        """
        # Keep map visible always.
        self._footer_widget.setVisible(not enabled)
        self._m2_controls_panel.setVisible(not enabled)
        self._mission_list_panel.setVisible(not enabled)
        self._log.setVisible(not enabled)
        self._status_frame.setVisible(not enabled)
        self._hb_frame.setVisible(not enabled)
        self._watchdog_frame.setVisible(not enabled)

    def _set_map_only_dashboard_mode(self, enabled: bool) -> None:
        """Map-centric layout: keep operator clutter hidden but retain native header (logo + flight chips)."""
        if not enabled:
            return
        self._central_layout.setContentsMargins(0, 0, 0, 0)
        self._content_layout.setContentsMargins(0, 0, 0, 0)
        self._content_layout.setSpacing(2)
        self._operations_layout.setContentsMargins(0, 0, 0, 0)
        self._operations_layout.setSpacing(0)
        self._center_row.setSpacing(0)
        self._top_dashboard.setVisible(True)
        self._link_box.setVisible(False)
        self._m2_controls_panel.setVisible(False)
        self._status_frame.setVisible(False)
        self._hb_frame.setVisible(False)
        self._watchdog_frame.setVisible(False)
        self._mission_list_panel.setVisible(False)
        self._log.setVisible(False)
        self._footer_widget.setVisible(False)
        try:
            self._split_camera_panel.setVisible(False)
        except Exception:
            pass
        self._vehicle_msg_frame.setVisible(False)
        self._camera_panel.setVisible(False)
        self._scroll.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self._scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

    def _build_header_bar(self) -> QFrame:
        bar = QFrame()
        bar.setObjectName("headerBar")
        bar.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        h = QHBoxLayout()
        h.setContentsMargins(12, 8, 12, 8)
        left = QVBoxLayout()
        title = QLabel("VGCS")
        title.setObjectName("headerTitle")
        sub = QLabel("Live MAVLink telemetry and link status.")
        sub.setObjectName("headerSubtitle")
        sub.setWordWrap(True)
        left.addWidget(title)
        left.addWidget(sub)
        h.addLayout(left, 1)
        bar.setLayout(h)
        return bar

    def _build_telemetry_panel(self) -> QWidget:
        self._fields = {}

        def add_field(key: str) -> QLabel:
            lab = self._make_value_label()
            self._fields[key] = lab
            return lab

        primary = QGroupBox("Primary flight data")
        pg = QGridLayout()
        pg.setHorizontalSpacing(12 if self._compact_ui else 16)
        pg.setVerticalSpacing(6 if self._compact_ui else 8)
        r = 0

        def row_pair(
            r_: int,
            t1: str,
            k1: str,
            t2: str,
            k2: str,
        ) -> int:
            l1 = QLabel(t1)
            l1.setStyleSheet("color: #7d869c;")
            l2 = QLabel(t2)
            l2.setStyleSheet("color: #7d869c;")
            pg.addWidget(l1, r_, 0)
            pg.addWidget(add_field(k1), r_, 1)
            pg.addWidget(l2, r_, 2)
            pg.addWidget(add_field(k2), r_, 3)
            return r_ + 1

        r = row_pair(r, "Armed", "armed", "Flight time", "flight_time")
        r = row_pair(r, "Ground speed", "groundspeed", "Air speed", "airspeed")
        r = row_pair(r, "Altitude (rel)", "alt_rel", "Altitude (MSL)", "alt_msl")
        ll = QLabel("Lat / Lon")
        ll.setStyleSheet("color: #7d869c;")
        pg.addWidget(ll, r, 0)
        lat_w = add_field("lat_lon")
        pg.addWidget(lat_w, r, 1, 1, 3)
        r += 1
        r = row_pair(r, "Heading", "heading", "Attitude (R/P/Y)", "attitude")
        primary.setLayout(pg)

        systems = QGroupBox("Navigation & systems")
        sg = QGridLayout()
        sg.setHorizontalSpacing(12 if self._compact_ui else 16)
        sg.setVerticalSpacing(6 if self._compact_ui else 8)
        sr = 0

        def row_sys(a: str, ka: str, b: str, kb: str) -> None:
            nonlocal sr
            la = QLabel(a)
            la.setStyleSheet("color: #7d869c;")
            lb = QLabel(b)
            lb.setStyleSheet("color: #7d869c;")
            sg.addWidget(la, sr, 0)
            sg.addWidget(add_field(ka), sr, 1)
            sg.addWidget(lb, sr, 2)
            sg.addWidget(add_field(kb), sr, 3)
            sr += 1

        row_sys("GPS", "gps", "Battery", "battery")
        row_sys("RC link", "rc_link", "Video link", "video_link")
        row_sys("Obstacle (prox)", "obstacle_prox", "Rangefinder", "rangefinder")
        row_sys("Battery failsafe", "failsafe_battery", "RC failsafe", "failsafe_rc")
        la = QLabel("Arm readiness")
        la.setStyleSheet("color: #7d869c;")
        sg.addWidget(la, sr, 0)
        sg.addWidget(add_field("arm_ready"), sr, 1, 1, 3)
        systems.setLayout(sg)

        self._fields["video_link"].setText("N/A")
        self._fields["obstacle_prox"].setText("N/A")
        self._fields["rangefinder"].setText("N/A")
        self._fields["arm_ready"].setText("Best-effort from telemetry")
        self._apply_state_style(self._fields["video_link"], "na")
        self._apply_state_style(self._fields["obstacle_prox"], "na")
        self._apply_state_style(self._fields["rangefinder"], "na")

        col = QWidget()
        v = QVBoxLayout()
        v.setSpacing(8 if self._compact_ui else 12)
        v.addWidget(primary)
        v.addWidget(systems)
        col.setLayout(v)
        return col

    def _apply_state_style(self, label: QLabel, state: str) -> None:
        label.setProperty("state_role", state)
        colors = self._theme_colors
        if state == "ok":
            label.setStyleSheet(f"color: {colors['ok']}; font-weight: 600;")
        elif state == "warn":
            label.setStyleSheet(f"color: {colors['warn']}; font-weight: 600;")
        elif state == "bad":
            label.setStyleSheet(f"color: {colors['bad']}; font-weight: 600;")
        elif state == "na":
            label.setStyleSheet(f"color: {colors['na']};")
        else:
            label.setProperty("state_role", "")
            label.setStyleSheet("")

    def _build_theme_colors(self, theme_name: str) -> dict[str, str]:
        themes = {
            "Default": {
                "ok": "#1b7f3b",
                "warn": "#b45f06",
                "bad": "#b00020",
                "na": "#666666",
            },
            "High Contrast": {
                "ok": "#0b7a0b",
                "warn": "#cc5500",
                "bad": "#d10000",
                "na": "#404040",
            },
            "Dark Friendly": {
                "ok": "#6ee7b7",
                "warn": "#fbbf24",
                "bad": "#f87171",
                "na": "#9ca3af",
            },
        }
        return themes.get(theme_name, themes["Default"])

    def _all_state_labels(self) -> list[QLabel]:
        return [self._status, self._hb, self._watchdog, *self._fields.values()]

    def _refresh_state_styles(self) -> None:
        for label in self._all_state_labels():
            state = str(label.property("state_role") or "")
            self._apply_state_style(label, state)

    def _set_ok_warn_field(self, key: str, is_ok: bool, ok_text: str = "OK") -> None:
        label = self._fields[key]
        if is_ok:
            label.setText(ok_text)
            self._apply_state_style(label, "ok")
        else:
            label.setText("WARN")
            self._apply_state_style(label, "bad")

    def _append_log(self, line: str) -> None:
        # Dashboard log panel can be hidden in map-only mode; always mirror to console.
        print(line, flush=True)
        self._log.append(line)
        self._log.verticalScrollBar().setValue(self._log.verticalScrollBar().maximum())

    def _refresh_footer_summary(self) -> None:
        self._footer_primary.setText(
            f"Alt {self._fields['alt_rel'].text()} | Speed {self._fields['groundspeed'].text()} | Time {self._fields['flight_time'].text()}"
        )
        self._footer_nav.setText(
            f"{self._fields['gps'].text()} | RC {self._fields['rc_link'].text()}"
        )

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
                lat = float(lat_item.text().strip())
                lon = float(lon_item.text().strip())
                alt = float(alt_item.text().strip())
                spd = float(spd_item.text().strip())
            except ValueError:
                continue
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
        self._top_vehicle_msg.setText(f"Uploading mission ({len(payload)} WPs)…")

    def _on_mission_download_requested(self) -> None:
        if self._thread is None or not self._thread.isRunning():
            QMessageBox.warning(self, "VGCS", "Connect vehicle before mission download.")
            return
        self._thread.queue_mission_download()
        self._append_log("Mission download queued")

    def _on_mission_uploaded(self, count: int) -> None:
        self._mission_upload_pending = False
        self._append_log(f"Mission upload success: {count} WPs")
        self._top_vehicle_msg.setText(f"Mission uploaded ({count})")
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
        self._top_vehicle_msg.setText(f"Mission downloaded ({len(wps)})")

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
        state = {
            "altRef": str(s.value("plan_alt_ref", "rel") or "rel"),
            "initialWpAltFt": float(s.value("plan_initial_wp_alt_ft", 164.0) or 164.0),
            "hoverMph": float(s.value("plan_hover_speed_mph", 11.18) or 11.18),
            "launchAltFt": float(s.value("plan_launch_alt_ft", 0.0) or 0.0),
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
        s.setValue("plan_initial_wp_alt_ft", float(data.get("initialWpAltFt", 164.0) or 164.0))
        s.setValue("plan_hover_speed_mph", float(data.get("hoverMph", 11.18) or 11.18))
        s.setValue("plan_launch_alt_ft", float(data.get("launchAltFt", 0.0) or 0.0))
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
        ft = float(state.get("initialWpAltFt", 164.0) or 164.0)
        target_m = ft * 0.3048
        home_amsl = self._home_amsl_m
        if ref == "amsl" and home_amsl is not None:
            return max(1.0, target_m - float(home_amsl))
        return max(1.0, target_m)

    def _plan_takeoff_alt_m_from_launch_settings(self) -> float | None:
        """Relative altitude (m) for NAV_TAKEOFF when Launch Position alt is set; else None (use WP1 alt)."""
        ft = float(self._settings.value("plan_launch_alt_ft", 0.0) or 0.0)
        if ft <= 0.01:
            return None
        state = {
            "altRef": str(self._settings.value("plan_alt_ref", "rel") or "rel"),
            "initialWpAltFt": ft,
        }
        return self._default_wp_alt_m_for_plan_state(state)

    def _apply_plan_mission_panel_to_model(self, state: dict[str, object]) -> None:
        self._map_widget.set_default_waypoint_alt_m(self._default_wp_alt_m_for_plan_state(state))
        mph = float(state.get("hoverMph", 11.18) or 11.18)
        self._plan_hover_speed_mps = max(0.5, mph * 0.44704)
        self._maybe_refresh_map_web_overlays()

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
        mission_distance_ft = f"{self._max_telem_dist_m * 3.28084:.0f}"
        return (
            "Live Analysis Snapshot\n\n"
            f"Link: {status}\n"
            f"Heartbeat: {hb}\n"
            f"Flight mode: {mode}\n"
            f"Battery: {battery}\n"
            f"GPS/HDOP: {gps}\n"
            f"Mission waypoints: {wp_count}\n"
            f"Max telemetry distance: {mission_distance_ft} ft\n\n"
            "Use Plan Flight to edit/upload waypoints and use M2 controls for "
            "mode, takeoff/land, geofence, params, and tile source."
        )

    def _show_application_settings_dialog(self) -> None:
        dlg = QDialog(self)
        dlg.setWindowTitle("Application Settings")
        dlg.setModal(True)
        dlg.resize(860, 520)
        dlg.setObjectName("appSettingsDialog")
        # Ensure consistent styling even if launched in a context where the
        # application stylesheet wasn't applied (e.g. external launcher/tests).
        try:
            app = QApplication.instance()
            qss = str(app.styleSheet() if app is not None else "").strip()
            base = qss if qss else gcs_stylesheet()
            # The global theme targets the main window roots; dialogs on some machines
            # (e.g. default light palette) need an explicit background.
            dlg.setStyleSheet(
                base
                + """
                QDialog#appSettingsDialog, QDialog#appSettingsDialog QWidget {
                    background-color: #1a1d24;
                    color: #e8eaef;
                }
                QDialog#appSettingsDialog QListWidget {
                    background-color: #12151c;
                    border: 1px solid #252d3d;
                    border-radius: 8px;
                    padding: 6px;
                }
                QDialog#appSettingsDialog QListWidget::item {
                    padding: 8px 10px;
                    border-radius: 6px;
                    color: #dbe1ee;
                }
                QDialog#appSettingsDialog QListWidget::item:selected {
                    background-color: #2d3a52;
                    color: #f0f4ff;
                }
                """
            )
        except Exception:
            pass
        root = QHBoxLayout(dlg)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(12)

        nav = QListWidget()
        nav.setFixedWidth(190)
        nav.setSpacing(2)
        nav.addItem(QListWidgetItem("General"))
        nav.addItem(QListWidgetItem("Video"))
        root.addWidget(nav, 0)

        stack = QStackedWidget()
        root.addWidget(stack, 1)

        # General page (keep as pointer to main UI, matching our app architecture).
        general = QWidget()
        g = QVBoxLayout(general)
        g.setContentsMargins(12, 12, 12, 12)
        g.setSpacing(10)
        g.addWidget(QLabel("General settings are available in the main window (Connection + Theme)."))
        g.addStretch(1)
        stack.addWidget(general)

        # Video page (QGC-like structure).
        video = QWidget()
        v = QVBoxLayout(video)
        v.setContentsMargins(12, 12, 12, 12)
        v.setSpacing(10)

        # Put settings in a scroll area so Apply/Close remain visible on small/high-DPI screens.
        video_scroll = QScrollArea()
        video_scroll.setWidgetResizable(True)
        video_scroll.setFrameShape(QFrame.Shape.NoFrame)
        v.addWidget(video_scroll, 1)

        video_body = QWidget()
        video_scroll.setWidget(video_body)
        vb = QVBoxLayout(video_body)
        vb.setContentsMargins(0, 0, 0, 0)
        vb.setSpacing(10)

        enabled = QCheckBox("Enable video streaming")
        vb.addWidget(enabled)
        video_enable_hint = QLabel(
            "When this is unchecked, stream URLs are saved but VGCS does not open or decode video — "
            "the map preview stays black. Check the box to actually connect to RTSP/UDP."
        )
        video_enable_hint.setWordWrap(True)
        video_enable_hint.setStyleSheet("color: #aab4c8; font-size: 11px;")
        vb.addWidget(video_enable_hint)

        source_group = QGroupBox("Video Source")
        sg = QGridLayout()
        sg.addWidget(QLabel("Source"), 0, 0)
        source_combo = QComboBox()
        source_combo.addItem("Video Stream Disabled", "disabled")
        source_combo.addItem("RTSP Video Stream", "rtsp")
        source_combo.addItem("UDP h.264 stream", "udp_h264")
        source_combo.addItem("UDP h.265 / HEVC stream", "udp_h265")
        sg.addWidget(source_combo, 0, 1)
        sg.addWidget(QLabel("URL hints"), 1, 0)
        url_hint = QLabel(
            "Examples:\n"
            "• RTSP — rtsp://192.168.x.x/stream (works on local radio/Wi‑Fi without internet).\n"
            "• UDP — udp://0.0.0.0:5600; pick h.264/h.265 for raw Annex B, or RTSP mode for MPEG‑TS UDP (auto-detect)."
        )
        url_hint.setWordWrap(True)
        url_hint.setStyleSheet("color: #aab4c8; font-size: 11px;")
        sg.addWidget(url_hint, 1, 1)
        source_group.setLayout(sg)
        vb.addWidget(source_group)

        conn_group = QGroupBox("Connection")
        cg = QGridLayout()
        cg.addWidget(QLabel("Stream URL 1 (Day / primary)"), 0, 0)
        rtsp_day = QLineEdit()
        rtsp_day.setPlaceholderText(
            "SIYI ZR10: rtsp://192.168.144.25:8554/main.264  (video2 not on all firmware)"
        )
        cg.addWidget(rtsp_day, 0, 1)
        cg.addWidget(QLabel("Stream URL 2 (Thermal / secondary)"), 1, 0)
        rtsp_th = QLineEdit()
        cg.addWidget(rtsp_th, 1, 1)
        conn_group.setLayout(cg)
        vb.addWidget(conn_group)

        camera_group = QGroupBox("Camera / gimbal control")
        cam_outer = QVBoxLayout()
        cam_outer.setSpacing(10)
        cam_row = QHBoxLayout()
        cam_row.addWidget(QLabel("Control provider"))
        camera_provider = QComboBox()
        camera_provider.addItem("MAVLink (ArduPilot mount)", "mavlink")
        camera_provider.addItem("SIYI SDK UDP (ZR10, ZT6, …)", "siyi")
        camera_provider.addItem("Skydroid TOP UDP (C13)", "skydroid")
        camera_provider.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        cam_row.addWidget(camera_provider, 1)
        cam_outer.addLayout(cam_row)

        camera_hint = QLabel()
        camera_hint.setWordWrap(True)
        camera_hint.setStyleSheet("color: #aab4c8; font-size: 11px;")
        cam_outer.addWidget(camera_hint)

        _hint_style = "color: #c8d0e0; font-size: 12px;"

        mavlink_panel = QWidget()
        mavlink_lay = QVBoxLayout(mavlink_panel)
        mavlink_lay.setContentsMargins(0, 4, 0, 0)
        mavlink_info = QLabel(
            "Gimbal and camera commands go through the flight controller (ArduPilot mount / MAV_CMD).\n"
            "M7 observation reports use MAVLink gimbal attitude when the FC publishes it.\n"
            "No extra camera IP or UDP settings are required."
        )
        mavlink_info.setWordWrap(True)
        mavlink_info.setStyleSheet(_hint_style)
        mavlink_lay.addWidget(mavlink_info)

        siyi_panel = QWidget()
        siyi_grid = QGridLayout(siyi_panel)
        siyi_grid.setContentsMargins(0, 4, 0, 0)
        siyi_grid.setHorizontalSpacing(12)
        siyi_grid.setVerticalSpacing(8)
        siyi_host = QLineEdit()
        siyi_host.setPlaceholderText("Empty = RTSP hostname or 192.168.144.25")
        siyi_port = QSpinBox()
        siyi_port.setRange(1, 65535)
        siyi_port.setValue(37260)
        siyi_timeout = QSpinBox()
        siyi_timeout.setRange(50, 5000)
        siyi_timeout.setValue(250)
        siyi_grid.addWidget(QLabel("Host / IP"), 0, 0)
        siyi_grid.addWidget(siyi_host, 0, 1)
        siyi_grid.addWidget(QLabel("UDP port"), 1, 0)
        siyi_grid.addWidget(siyi_port, 1, 1)
        siyi_grid.addWidget(QLabel("Timeout (ms)"), 2, 0)
        siyi_grid.addWidget(siyi_timeout, 2, 1)
        siyi_grid.setColumnStretch(1, 1)

        skydroid_panel = QWidget()
        skydroid_grid = QGridLayout(skydroid_panel)
        skydroid_grid.setContentsMargins(0, 4, 0, 0)
        skydroid_grid.setHorizontalSpacing(12)
        skydroid_grid.setVerticalSpacing(8)
        skydroid_host = QLineEdit()
        skydroid_host.setPlaceholderText(
            "Empty = RTSP host + RC Wi-Fi gateway + 192.168.144.108"
        )
        skydroid_port = QSpinBox()
        skydroid_port.setRange(1, 65535)
        skydroid_port.setValue(5000)
        skydroid_timeout = QSpinBox()
        skydroid_timeout.setRange(50, 5000)
        skydroid_timeout.setValue(250)
        skydroid_profile = QComboBox()
        skydroid_profile.addItem("C13 Default (GAA/GSY/GAY)", "c13_default")
        skydroid_profile.addItem("C13 Alternate (GAC/GSP/GAP)", "c13_alt")
        skydroid_grid.addWidget(QLabel("Host / IP"), 0, 0)
        skydroid_grid.addWidget(skydroid_host, 0, 1)
        skydroid_grid.addWidget(QLabel("UDP port"), 1, 0)
        skydroid_grid.addWidget(skydroid_port, 1, 1)
        skydroid_grid.addWidget(QLabel("Timeout (ms)"), 2, 0)
        skydroid_grid.addWidget(skydroid_timeout, 2, 1)
        skydroid_grid.addWidget(QLabel("Firmware profile"), 3, 0)
        skydroid_grid.addWidget(skydroid_profile, 3, 1)
        skydroid_grid.setColumnStretch(1, 1)

        camera_stack = QStackedWidget()
        camera_stack.addWidget(mavlink_panel)
        camera_stack.addWidget(siyi_panel)
        camera_stack.addWidget(skydroid_panel)
        cam_outer.addWidget(camera_stack)

        _CAMERA_PROVIDER_HINTS: dict[str, str] = {
            "mavlink": (
                "ArduPilot mount control over MAVLink. Connect the FC link before using gimbal buttons or M7 gimbal columns."
            ),
            "siyi": (
                "SIYI gimbal SDK (ZR10, ZT6, A8 mini): UDP port 37260 on the camera IP — usually the same host as your RTSP URL."
            ),
            "skydroid": (
                "Skydroid C13 TOP (PROTOCAL): UDP 192.168.144.108 port 5000 (#TP frames). "
                "RTSP: rtsp://192.168.144.108:554/stream=1. PC Ethernet 192.168.144.10/24."
            ),
        }
        _CAMERA_STACK_INDEX = {"mavlink": 0, "siyi": 1, "skydroid": 2}
        _RTSP_PLACEHOLDER = {
            "mavlink": "rtsp://host/stream or udp://0.0.0.0:5600",
            "siyi": "SIYI ZR10: rtsp://192.168.144.25:8554/main.264",
            "skydroid": "Skydroid C13: rtsp://192.168.144.108:554/stream=1",
        }

        def _sync_camera_provider_ui() -> None:
            pid = str(camera_provider.currentData() or "mavlink")
            camera_stack.setCurrentIndex(_CAMERA_STACK_INDEX.get(pid, 0))
            camera_hint.setText(_CAMERA_PROVIDER_HINTS.get(pid, ""))
            rtsp_day.setPlaceholderText(_RTSP_PLACEHOLDER.get(pid, _RTSP_PLACEHOLDER["mavlink"]))

        camera_provider.currentIndexChanged.connect(lambda _idx: _sync_camera_provider_ui())
        _sync_camera_provider_ui()

        camera_group.setLayout(cam_outer)
        vb.addWidget(camera_group)

        settings_group = QGroupBox("Settings")
        stg = QGridLayout()
        stg.addWidget(QLabel("Aspect ratio"), 0, 0)
        aspect = QComboBox()
        aspect.addItems(["Auto", "16:9", "4:3", "1:1"])
        stg.addWidget(aspect, 0, 1)
        low_latency = QCheckBox("Low latency mode")
        stg.addWidget(low_latency, 1, 0, 1, 2)
        stg.addWidget(QLabel("Video decode priority"), 2, 0)
        decode_prio = QComboBox()
        decode_prio.addItems(["Normal", "Prefer Hardware", "Prefer Software"])
        stg.addWidget(decode_prio, 2, 1)
        stg.addWidget(QLabel("RTSP transport"), 3, 0)
        rtsp_transport = QComboBox()
        rtsp_transport.addItem("Auto (LAN: TCP first · WAN: UDP first)", "auto")
        rtsp_transport.addItem("UDP", "udp")
        rtsp_transport.addItem("TCP", "tcp")
        stg.addWidget(rtsp_transport, 3, 1)
        settings_group.setLayout(stg)
        vb.addWidget(settings_group)

        storage_group = QGroupBox("Local Video Storage")
        lg = QGridLayout()
        lg.addWidget(QLabel("Record file format"), 0, 0)
        record_fmt = QComboBox()
        record_fmt.addItems(["mp4", "mkv"])
        lg.addWidget(record_fmt, 0, 1)
        auto_del = QCheckBox("Auto-delete saved recordings")
        lg.addWidget(auto_del, 1, 0, 1, 2)
        lg.addWidget(QLabel("Max storage usage (MB)"), 2, 0)
        max_mb = QSpinBox()
        max_mb.setRange(0, 200000)
        max_mb.setValue(10240)
        lg.addWidget(max_mb, 2, 1)
        storage_group.setLayout(lg)
        vb.addWidget(storage_group)

        split_default = QComboBox()
        split_default.addItems(["Single", "Split"])
        row3 = QHBoxLayout()
        row3.addWidget(QLabel("Default view"))
        row3.addWidget(split_default)
        row3.addStretch(1)
        vb.addLayout(row3)
        vb.addStretch(1)

        btn_row = QHBoxLayout()
        btn_apply = QPushButton("Apply")
        btn_close = QPushButton("Close")
        btn_row.addStretch(1)
        btn_row.addWidget(btn_apply)
        btn_row.addWidget(btn_close)
        v.addLayout(btn_row)
        stack.addWidget(video)

        def _on_nav_changed(idx: int) -> None:
            stack.setCurrentIndex(max(0, min(idx, stack.count() - 1)))

        nav.currentRowChanged.connect(_on_nav_changed)
        nav.setCurrentRow(1)  # open Video by default (matches client flow)

        # Load existing settings.
        s = self._settings
        enabled.setChecked(_settings_truthy(s.value("video/enabled", False), default=False))
        source_combo.setCurrentIndex(max(0, source_combo.findData(str(s.value("video/source", "rtsp") or "rtsp"))))
        rtsp_day.setText(str(s.value("video/rtsp_day", "") or ""))
        rtsp_th.setText(str(s.value("video/rtsp_thermal", "") or ""))
        split_default.setCurrentText(str(s.value("video/default_view", "Single") or "Single"))
        aspect.setCurrentText(str(s.value("video/aspect", "Auto") or "Auto"))
        low_latency.setChecked(_settings_truthy(s.value("video/low_latency", False), default=False))
        decode_prio.setCurrentText(str(s.value("video/decode_priority", "Normal") or "Normal"))
        rtsp_transport.setCurrentIndex(max(0, rtsp_transport.findData(str(s.value("video/rtsp_transport", "auto") or "auto"))))
        record_fmt.setCurrentText(str(s.value("video/record_format", "mp4") or "mp4"))
        auto_del.setChecked(_settings_truthy(s.value("video/auto_delete", False), default=False))
        try:
            max_mb.setValue(int(s.value("video/max_storage_mb", 10240) or 10240))
        except Exception:
            max_mb.setValue(10240)
        camera_provider.setCurrentIndex(
            max(0, camera_provider.findData(str(s.value("camera/provider", "mavlink") or "mavlink")))
        )
        skydroid_host.setText(str(s.value("camera/skydroid_host", "") or ""))
        try:
            skydroid_port.setValue(int(s.value("camera/skydroid_port", 5000) or 5000))
        except Exception:
            skydroid_port.setValue(5000)
        try:
            skydroid_timeout.setValue(int(s.value("camera/skydroid_timeout_ms", 250) or 250))
        except Exception:
            skydroid_timeout.setValue(250)
        skydroid_profile.setCurrentIndex(
            max(0, skydroid_profile.findData(str(s.value("camera/skydroid_profile", "c13_default") or "c13_default")))
        )
        siyi_host.setText(str(s.value("camera/siyi_host", "") or ""))
        try:
            siyi_port.setValue(int(s.value("camera/siyi_port", 37260) or 37260))
        except Exception:
            siyi_port.setValue(37260)
        try:
            siyi_timeout.setValue(int(s.value("camera/siyi_timeout_ms", 250) or 250))
        except Exception:
            siyi_timeout.setValue(250)
        _sync_camera_provider_ui()

        def _apply() -> None:
            # Return immediately from the click handler so Windows gets a repainted frame.
            def _commit_and_close() -> None:
                day_u = str(rtsp_day.text()).strip()
                th_u = str(rtsp_th.text()).strip()
                src_kind = str(source_combo.currentData() or "rtsp")
                if (
                    src_kind != "disabled"
                    and not bool(enabled.isChecked())
                    and (day_u or th_u)
                ):
                    try:
                        r = QMessageBox.question(
                            dlg,
                            "Enable video streaming?",
                            "You entered a stream URL, but 'Enable video streaming' is unchecked, so VGCS will "
                            "not open RTSP/UDP and the map preview will stay black.\n\n"
                            "Turn streaming on and save these settings?",
                            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                            QMessageBox.StandardButton.Yes,
                        )
                        if r == QMessageBox.StandardButton.Yes:
                            enabled.setChecked(True)
                    except Exception:
                        pass
                s.setValue("video/enabled", bool(enabled.isChecked()))
                s.setValue("video/source", str(source_combo.currentData() or "rtsp"))
                _day_rtsp = str(rtsp_day.text()).strip()
                try:
                    from vgcs.video.pipeline import _normalize_companion_rtsp_url

                    _day_rtsp = _normalize_companion_rtsp_url(_day_rtsp)
                    if _day_rtsp != str(rtsp_day.text()).strip():
                        rtsp_day.setText(_day_rtsp)
                except Exception:
                    pass
                s.setValue("video/rtsp_day", _day_rtsp)
                s.setValue("video/rtsp_thermal", str(rtsp_th.text()).strip())
                s.setValue("video/default_view", str(split_default.currentText()))
                s.setValue("video/aspect", str(aspect.currentText()))
                s.setValue("video/low_latency", bool(low_latency.isChecked()))
                s.setValue("video/decode_priority", str(decode_prio.currentText()))
                s.setValue("video/rtsp_transport", str(rtsp_transport.currentData() or "auto"))
                s.setValue("video/record_format", str(record_fmt.currentText()))
                s.setValue("video/auto_delete", bool(auto_del.isChecked()))
                s.setValue("video/max_storage_mb", int(max_mb.value()))
                s.setValue("camera/provider", str(camera_provider.currentData() or "mavlink"))
                s.setValue("camera/siyi_host", str(siyi_host.text()).strip())
                s.setValue("camera/siyi_port", int(siyi_port.value()))
                s.setValue("camera/siyi_timeout_ms", int(siyi_timeout.value()))
                s.setValue("camera/skydroid_host", str(skydroid_host.text()).strip())
                s.setValue("camera/skydroid_port", int(skydroid_port.value()))
                s.setValue("camera/skydroid_timeout_ms", int(skydroid_timeout.value()))
                s.setValue("camera/skydroid_profile", str(skydroid_profile.currentData() or "c13_default"))
                dlg.accept()
                # QDialog::accept() may process a nested event loop; a 0-ms timer can fire
                # *during* that teardown and run FFmpeg/WebEngine work while the dialog is
                # still closing → "Application Settings (Not Responding)". Defer past it.
                # Real companion RTSP often needs longer than localhost before the main window
                # should begin synchronous teardown of the old session.
                QTimer.singleShot(550, self._deferred_apply_saved_video_settings)

            QTimer.singleShot(0, _commit_and_close)

        btn_apply.clicked.connect(_apply)
        btn_close.clicked.connect(dlg.accept)

        dlg.exec()

    def _show_vehicle_configuration_help(self) -> None:
        QMessageBox.information(
            self,
            "Vehicle Configuration",
            "Vehicle configuration tools are in the main window under the map, in the "
            "group M2 controls (scroll down if needed):\n\n"
            "- Flight mode, takeoff/land (logo → Set Flight Mode when using map-only layout)\n"
            "- Geofence upload\n"
            "- Parameter refresh/set (WPNAV_SPEED, RTL_ALT, fence, ARMING_CHECK)\n"
            "- Map tiles online/offline\n\n"
            "Tip: connect first, then use Refresh params / Set param.",
        )

    def _show_vehicle_quick_controls_dialog(self, *, include_params: bool = True) -> None:
        """Backward compatibility shim for older callers."""
        self._show_flight_controls_dialog(open_advanced=bool(include_params))

    def _show_flight_controls_dialog(self, *, open_advanced: bool = False) -> None:
        dlg = QDialog(self)
        dlg.setWindowTitle("Vehicle Configuration")
        dlg.setModal(True)
        dlg.resize(720, 760)

        root = QVBoxLayout(dlg)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        header = QLabel("Vehicle Configuration")
        header.setObjectName("headerTitle")
        sub = QLabel("Calibration, flight mode, assist features, and advanced parameters.")
        sub.setObjectName("headerSubtitle")
        root.addWidget(header)
        root.addWidget(sub)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        root.addWidget(scroll, 1)

        body = QWidget()
        scroll.setWidget(body)
        lay = QVBoxLayout(body)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(10)

        can_send = bool(self._thread is not None and self._thread.isRunning())
        if not can_send:
            warn = QLabel("Connect vehicle to enable commands.")
            warn.setStyleSheet("color: #a8b0c4;")
            lay.addWidget(warn)

        # Sensor calibration
        cal_group = QGroupBox("Sensor calibration")
        cal_lay = QVBoxLayout()
        cal_lay.setSpacing(6)
        cal_hint = QLabel("Tip: keep the vehicle still unless prompted otherwise.")
        cal_hint.setStyleSheet("color: #7d869c; font-weight: 400;")
        cal_lay.addWidget(cal_hint)

        cal_btn_row1 = QHBoxLayout()
        btn_cal_accel = QPushButton("Accelerometer")
        btn_cal_compass = QPushButton("Compass")
        btn_cal_gyro = QPushButton("Gyro")
        cal_btn_row1.addWidget(btn_cal_accel)
        cal_btn_row1.addWidget(btn_cal_compass)
        cal_btn_row1.addWidget(btn_cal_gyro)
        cal_lay.addLayout(cal_btn_row1)

        cal_btn_row2 = QHBoxLayout()
        btn_cal_baro = QPushButton("Barometer")
        btn_cal_rc = QPushButton("RC")
        cal_btn_row2.addWidget(btn_cal_baro)
        cal_btn_row2.addWidget(btn_cal_rc)
        cal_btn_row2.addStretch()
        cal_lay.addLayout(cal_btn_row2)

        for b in (btn_cal_accel, btn_cal_compass, btn_cal_gyro, btn_cal_baro, btn_cal_rc):
            b.setEnabled(can_send)

        def _queue_cal(kind: str) -> None:
            if self._thread is None or not self._thread.isRunning():
                QMessageBox.warning(self, "VGCS", "Connect vehicle before calibration.")
                return
            try:
                self._thread.queue_preflight_calibration(kind)
                self._append_log(f"Calibration queued: {kind}")
            except Exception as e:
                QMessageBox.warning(self, "VGCS", f"Calibration failed to queue: {e}")

        btn_cal_accel.clicked.connect(lambda: _queue_cal("accel"))
        btn_cal_compass.clicked.connect(lambda: _queue_cal("compass"))
        btn_cal_gyro.clicked.connect(lambda: _queue_cal("gyro"))
        btn_cal_baro.clicked.connect(lambda: _queue_cal("baro"))
        btn_cal_rc.clicked.connect(lambda: _queue_cal("rc"))

        cal_group.setLayout(cal_lay)
        lay.addWidget(cal_group)

        # Flight mode + commands
        flight_group = QGroupBox("Flight mode")
        flight_lay = QVBoxLayout()
        flight_lay.setSpacing(8)

        combo = QComboBox()
        combo.addItems([self._mode_combo.itemText(i) for i in range(self._mode_combo.count())])
        current = self._mode_combo.currentText().strip()
        if current:
            combo.setCurrentText(current)
        combo.setEnabled(can_send)
        flight_lay.addWidget(combo)

        alt_row = QHBoxLayout()
        alt_row.addWidget(QLabel("Takeoff alt (m)"))
        alt_spin = QDoubleSpinBox()
        alt_spin.setRange(1.0, 200.0)
        alt_spin.setDecimals(1)
        alt_spin.setValue(float(self._takeoff_alt_spin.value()))
        alt_spin.setEnabled(can_send)
        alt_row.addWidget(alt_spin, 1)
        flight_lay.addLayout(alt_row)

        btn_row = QHBoxLayout()
        btn_set_mode = QPushButton("Set mode")
        btn_takeoff = QPushButton("Takeoff")
        btn_land = QPushButton("Land")
        btn_row.addWidget(btn_set_mode)
        btn_row.addWidget(btn_takeoff)
        btn_row.addWidget(btn_land)
        flight_lay.addLayout(btn_row)

        btn_row2 = QHBoxLayout()
        btn_auto_takeoff = QPushButton("Auto takeoff")
        btn_auto_land = QPushButton("Auto land")
        btn_row2.addWidget(btn_auto_takeoff)
        btn_row2.addWidget(btn_auto_land)
        btn_row2.addStretch()
        flight_lay.addLayout(btn_row2)

        flight_group.setLayout(flight_lay)
        lay.addWidget(flight_group)

        btn_set_mode.setEnabled(can_send)
        btn_takeoff.setEnabled(can_send)
        btn_land.setEnabled(can_send)
        btn_auto_takeoff.setEnabled(can_send)
        btn_auto_land.setEnabled(can_send)

        def _sync_alt_to_main() -> None:
            self._takeoff_alt_spin.setValue(float(alt_spin.value()))

        def _set_mode_from_dialog() -> None:
            mode_name = combo.currentText().strip()
            if not mode_name:
                return
            self._mode_combo.setCurrentText(mode_name)
            self._on_set_mode()

        btn_set_mode.clicked.connect(_set_mode_from_dialog)
        btn_takeoff.clicked.connect(lambda: (_sync_alt_to_main(), self._on_takeoff()))
        btn_land.clicked.connect(self._on_land)
        btn_auto_takeoff.clicked.connect(lambda: (_sync_alt_to_main(), self._on_auto_takeoff()))
        btn_auto_land.clicked.connect(self._on_auto_land)

        # Assist features
        assist_group = QGroupBox("Assist features")
        assist_lay = QVBoxLayout()
        assist_lay.setSpacing(8)

        airmode_dlg = QCheckBox("AirMode")
        trainer_dlg = QComboBox()
        trainer_dlg.addItems(["Acro Trainer: Off", "Acro Trainer: Level", "Acro Trainer: Level+Limit"])
        simple_dlg = QCheckBox("Simple")
        super_simple_dlg = QCheckBox("Super Simple")

        airmode_dlg.setEnabled(can_send)
        trainer_dlg.setEnabled(can_send)
        simple_dlg.setEnabled(can_send)
        super_simple_dlg.setEnabled(can_send)

        assist_lay.addWidget(airmode_dlg)
        assist_lay.addWidget(trainer_dlg)
        assist_lay.addWidget(simple_dlg)
        assist_lay.addWidget(super_simple_dlg)

        apply_features = QPushButton("Apply assist features")
        apply_features.setEnabled(can_send)
        assist_lay.addWidget(apply_features)

        assist_group.setLayout(assist_lay)
        lay.addWidget(assist_group)

        # Advanced parameters (collapsed)
        adv_box = QGroupBox("Advanced parameters")
        adv_box.setCheckable(True)
        adv_box.setChecked(bool(open_advanced))
        adv_inner = QVBoxLayout()
        adv_inner.setSpacing(8)

        p_row = QHBoxLayout()
        param_combo_dlg = QComboBox()
        for i in range(self._param_name_combo.count()):
            param_combo_dlg.addItem(self._param_name_combo.itemText(i))
        param_combo_dlg.setCurrentText(self._param_name_combo.currentText())
        dlg_spin = QDoubleSpinBox()
        dlg_spin.setRange(-100000.0, 100000.0)
        dlg_spin.setDecimals(3)
        dlg_spin.setValue(float(self._param_value_spin.value()))
        p_row.addWidget(QLabel("Param"))
        p_row.addWidget(param_combo_dlg, 2)
        p_row.addWidget(QLabel("Value"))
        p_row.addWidget(dlg_spin, 1)
        adv_inner.addLayout(p_row)

        p_btns = QHBoxLayout()
        btn_params_dlg = QPushButton("Refresh params")
        btn_param_set_dlg = QPushButton("Set param")
        btn_params_dlg.setEnabled(can_send)
        btn_param_set_dlg.setEnabled(can_send)
        p_btns.addWidget(btn_params_dlg)
        p_btns.addWidget(btn_param_set_dlg)
        adv_inner.addLayout(p_btns)

        adv_box.setLayout(adv_inner)
        lay.addWidget(adv_box)

        def _sync_params_dlg_to_main() -> None:
            self._param_name_combo.setCurrentText(param_combo_dlg.currentText())
            self._param_value_spin.setValue(float(dlg_spin.value()))

        def _params_refresh_from_dialog() -> None:
            _sync_params_dlg_to_main()
            self._on_params_refresh()
            dlg_spin.setValue(float(self._param_value_spin.value()))

        def _param_set_from_dialog() -> None:
            _sync_params_dlg_to_main()
            self._on_param_set()

        btn_params_dlg.clicked.connect(_params_refresh_from_dialog)
        btn_param_set_dlg.clicked.connect(_param_set_from_dialog)

        def _refresh_feature_controls_from_cache() -> None:
            opts = int(self._last_params.get("ACRO_OPTIONS", 0.0) or 0.0)
            airmode_dlg.blockSignals(True)
            airmode_dlg.setChecked(bool(opts & 1))
            airmode_dlg.blockSignals(False)
            trainer_val = int(self._last_params.get("ACRO_TRAINER", 2.0) or 2.0)
            trainer_dlg.blockSignals(True)
            trainer_dlg.setCurrentIndex(max(0, min(2, trainer_val)))
            trainer_dlg.blockSignals(False)
            simple_dlg.blockSignals(True)
            simple_dlg.setChecked(bool(int(self._last_params.get("SIMPLE", 0.0) or 0.0) != 0))
            simple_dlg.blockSignals(False)
            super_simple_dlg.blockSignals(True)
            super_simple_dlg.setChecked(
                bool(int(self._last_params.get("SUPER_SIMPLE", 0.0) or 0.0) != 0)
            )
            super_simple_dlg.blockSignals(False)

        def _apply_features_from_dialog() -> None:
            self._airmode_check.setChecked(airmode_dlg.isChecked())
            self._acro_trainer_combo.setCurrentIndex(trainer_dlg.currentIndex())
            self._simple_check.setChecked(simple_dlg.isChecked())
            self._super_simple_check.setChecked(super_simple_dlg.isChecked())
            self._on_apply_acro_options()
            self._on_apply_simple_options()

        apply_features.clicked.connect(_apply_features_from_dialog)

        # Refresh-on-open for assist features.
        if can_send and self._thread is not None:
            try:
                self._thread.queue_params_fetch(["ACRO_OPTIONS", "ACRO_TRAINER", "SIMPLE", "SUPER_SIMPLE"])
                self._append_log("Param fetch queued (assist features)")
            except Exception:
                pass

        def _on_params_snapshot_for_dialog(payload: object) -> None:
            _refresh_feature_controls_from_cache()

        if self._thread is not None:
            self._thread.params_snapshot.connect(_on_params_snapshot_for_dialog)

            def _disconnect_dialog_param_hook() -> None:
                try:
                    self._thread.params_snapshot.disconnect(_on_params_snapshot_for_dialog)
                except Exception:
                    pass

            dlg.finished.connect(_disconnect_dialog_param_hook)

        _refresh_feature_controls_from_cache()

        def _apply_all_from_dialog() -> bool:
            """Push flight mode, takeoff altitude, assist params, and optional advanced param to the vehicle."""
            if self._thread is None or not self._thread.isRunning():
                QMessageBox.warning(self, "VGCS", "Connect vehicle before applying configuration.")
                return False
            _sync_alt_to_main()
            _set_mode_from_dialog()
            _apply_features_from_dialog()
            if adv_box.isChecked():
                _param_set_from_dialog()
            extra = " + param" if adv_box.isChecked() else ""
            self._append_log(f"Vehicle configuration applied (OK): mode, takeoff alt, assist{extra}")
            return True

        def _on_dialog_ok() -> None:
            if _apply_all_from_dialog():
                dlg.accept()

        # Footer
        lay.addStretch(1)
        footer = QHBoxLayout()
        footer.addStretch(1)
        btn_ok = QPushButton("OK")
        btn_ok.setEnabled(can_send)
        btn_ok.setDefault(True)
        btn_ok.setAutoDefault(True)
        btn_ok.setToolTip(
            "Apply flight mode, takeoff altitude (for takeoff commands), assist features, "
            "and—if Advanced parameters is expanded—the selected parameter."
        )
        btn_close = QPushButton("Close")
        btn_close.setToolTip("Close without applying the above as one batch (per-section buttons still work).")
        footer.addWidget(btn_ok)
        footer.addWidget(btn_close)
        root.addLayout(footer)
        btn_ok.clicked.connect(_on_dialog_ok)
        btn_close.clicked.connect(dlg.reject)
        dlg.exec()
        self._scroll_main_to(self._m2_controls_panel)

    def _set_dashboard_flight_status(self, state: str, message: str) -> None:
        """Mirror legacy Web `setFlightStatus()` — full `#linkBanner` tint (git e48c1a7 map_widget)."""
        state_norm = (state or "").strip().lower()
        self._apply_link_banner_palette(state_norm)
        lb = getattr(self, "_link_banner_text", None)
        if lb is not None:
            lb.setText(message)
        stack = getattr(self, "_header_banner_stack", None)
        if stack is not None:
            # Web: `#linkBannerConnected` only when green/yellow; else `#linkBannerDisconnected`.
            stack.setCurrentIndex(1 if state_norm in ("green", "yellow") else 0)

        if state_norm == "green":
            self._flight_status_btn.setText("READY TO ARM")
            self._top_vehicle_msg.setText(message)
            self._map_widget.set_flight_status("green", message)
            return
        if state_norm == "yellow":
            self._flight_status_btn.setText("NOT READY TO ARM")
            self._top_vehicle_msg.setText(message)
            self._map_widget.set_flight_status("yellow", message)
            return
        if state_norm == "red":
            self._flight_status_btn.setText("NOT READY TO ARM")
            self._top_vehicle_msg.setText(message)
            self._map_widget.set_flight_status("red", message)
            return
        # Cold-start / idle disconnected: Web stylesheet `#linkBanner` neutral background — not maroon.
        self._flight_status_btn.setText("NOT READY TO ARM")
        self._top_vehicle_msg.setText(message)
        self._map_widget.set_flight_status("idle", message)

    def _push_map_flight_overlay(self) -> None:
        if self._armed_since is None:
            flight_time_text = "00:00:00"
        else:
            elapsed = int(time.monotonic() - self._armed_since)
            h = elapsed // 3600
            m = (elapsed % 3600) // 60
            s = elapsed % 60
            flight_time_text = f"{h:02d}:{m:02d}:{s:02d}"
        rel_display_m = float(self._map_rel_alt_m)
        if self._home_rel_alt_baseline_m is not None:
            rel_display_m -= float(self._home_rel_alt_baseline_m)
        if self._armed_since is None and float(self._map_groundspeed_mps) < 0.5:
            if abs(rel_display_m) < 1.5:
                rel_display_m = 0.0
        dist_home_m = 0.0
        try:
            pos = self._map_widget.get_vehicle_display_position()
            if pos is not None and self._home_lat is not None and self._home_lon is not None:
                dist_home_m = self._haversine_m(
                    float(self._home_lat), float(self._home_lon), pos[0], pos[1]
                )
            if self._armed_since is None and float(self._map_groundspeed_mps) < 0.5:
                if dist_home_m < 2.0:
                    dist_home_m = 0.0
        except Exception:
            dist_home_m = 0.0
        self._map_widget.set_flight_telemetry(
            relative_alt_m=rel_display_m,
            ground_speed_mps=float(self._map_groundspeed_mps),
            vertical_speed_mps=float(self._map_climb_mps),
            flight_time_text=flight_time_text,
            distance_from_home_m=dist_home_m,
        )

    def _sync_visible_map_overlay_metrics(self) -> None:
        """Update only the map overlay that is on-screen (avoids redundant WebEngine repaints)."""
        if self._plan_flight_layer_wanted:
            self._refresh_plan_flight_metrics()
        else:
            self._push_map_flight_overlay()

    @staticmethod
    def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        r = 6_371_000.0
        p1 = math.radians(lat1)
        p2 = math.radians(lat2)
        dp = math.radians(lat2 - lat1)
        dl = math.radians(lon2 - lon1)
        a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
        return 2 * r * math.asin(min(1.0, math.sqrt(max(0.0, a))))

    def _refresh_plan_flight_metrics(self) -> None:
        # M2 plan bar live values (best-effort from real telemetry).
        heading_val = float(getattr(self, "_heading", 0.0) or 0.0)
        alt_diff_ft = f"{self._map_rel_alt_m * 3.28084:.1f} ft"
        gradient = "-.-"
        azimuth = f"{int(round(heading_val))}"
        heading = f"{int(round(heading_val))}"
        dist_prev_wp_ft = "0.0 ft"

        mission_distance_m = 0.0
        model = list(getattr(self._map_widget, "_waypoints_model", []))
        for i in range(1, len(model)):
            a = model[i - 1]
            b = model[i]
            mission_distance_m += self._haversine_m(float(a.lat), float(a.lon), float(b.lat), float(b.lon))
        mission_distance_ft = f"{mission_distance_m * 3.28084:.0f} ft"

        if self._armed_since is not None:
            elapsed = int(time.monotonic() - self._armed_since)
            mission_time = f"{elapsed // 3600:02d}:{(elapsed % 3600) // 60:02d}:{elapsed % 60:02d}"
        elif mission_distance_m > 1.0 and self._plan_hover_speed_mps > 0.5:
            eta_s = int(mission_distance_m / self._plan_hover_speed_mps)
            mission_time = f"{eta_s // 3600:02d}:{(eta_s % 3600) // 60:02d}:{eta_s % 60:02d}"
        else:
            mission_time = "00:00:00"

        max_telem_dist_ft = f"{self._max_telem_dist_m * 3.28084:.0f} ft"
        self._map_widget.set_plan_flight_metrics(
            alt_diff_ft=alt_diff_ft,
            gradient=gradient,
            azimuth=azimuth,
            heading=heading,
            dist_prev_wp_ft=dist_prev_wp_ft,
            mission_distance_ft=mission_distance_ft,
            mission_time=mission_time,
            max_telem_dist_ft=max_telem_dist_ft,
        )

    def _maybe_refresh_map_web_overlays(self) -> None:
        """Push bottom telemetry + plan strip to the map page at a capped rate."""
        now = time.monotonic()
        if (
            self._last_map_overlay_refresh_s is not None
            and now - self._last_map_overlay_refresh_s < 0.1
        ):
            return
        self._last_map_overlay_refresh_s = now
        # Only refresh the overlay that is visible. Updating both every tick forces
        # Chromium to repaint hidden DOM (plan bar vs compass HUD) and causes map flicker.
        self._sync_visible_map_overlay_metrics()

    @staticmethod
    def _extract_remote_id_text(data: dict[str, object]) -> str:
        candidates = (
            "uas_id",
            "id_or_mac",
            "operator_id",
            "self_id",
            "description",
        )
        for key in candidates:
            raw = data.get(key)
            if raw is None:
                continue
            if isinstance(raw, str):
                txt = raw.strip().strip("\x00")
                if txt:
                    return txt
            elif isinstance(raw, (bytes, bytearray)):
                txt = raw.decode("ascii", errors="ignore").strip().strip("\x00")
                if txt:
                    return txt
            elif isinstance(raw, list):
                try:
                    txt = bytes(int(v) & 0xFF for v in raw).decode(
                        "ascii", errors="ignore"
                    ).strip().strip("\x00")
                except Exception:
                    txt = ""
                if txt:
                    return txt
        return ""

    def _reset_telemetry_fields(self) -> None:
        self._armed_since = None
        self._map_rel_alt_m = 0.0
        self._map_msl_alt_m = 0.0
        self._map_groundspeed_mps = 0.0
        self._map_climb_mps = 0.0
        self._heading = 0.0
        self._max_telem_dist_m = 0.0
        self._home_lat = None
        self._home_lon = None
        self._home_amsl_m = None
        self._home_rel_alt_baseline_m = None
        self._fields["armed"].setText("No")
        self._apply_state_style(self._fields["armed"], "warn")
        self._fields["flight_time"].setText("00:00")
        self._fields["lat_lon"].setText("—")
        self._fields["alt_rel"].setText("—")
        self._fields["alt_msl"].setText("—")
        self._fields["groundspeed"].setText("—")
        self._fields["airspeed"].setText("—")
        self._fields["heading"].setText("—")
        self._fields["attitude"].setText("—")
        self._fields["gps"].setText("—")
        self._fields["battery"].setText("—")
        self._fields["rc_link"].setText("—")
        self._fields["failsafe_battery"].setText("—")
        self._fields["failsafe_rc"].setText("—")
        self._fields["arm_ready"].setText("Best-effort from telemetry")
        self._fields["video_link"].setText("N/A")
        self._fields["obstacle_prox"].setText("N/A")
        self._fields["rangefinder"].setText("N/A")
        self._apply_state_style(self._fields["video_link"], "na")
        self._apply_state_style(self._fields["obstacle_prox"], "na")
        self._apply_state_style(self._fields["rangefinder"], "na")
        self._apply_state_style(self._fields["failsafe_battery"], "")
        self._apply_state_style(self._fields["failsafe_rc"], "")
        self._apply_state_style(self._fields["arm_ready"], "")
        self._apply_state_style(self._fields["rc_link"], "")
        self._map_widget.set_mission_waypoint_count(0)
        self._top_gps_sat.setText("—")
        self._top_gps_hdop.setText("—")
        self._top_flight_mode.setText("—")
        self._top_battery.setText("—")
        self._top_remote_id.setText("N/A")
        self._top_vehicle_msg.setText("—")
        self._map_widget.set_header_mode("—")
        self._map_widget.set_header_vehicle_msg("—")
        self._map_widget.set_header_gps(0, "N/A")
        self._map_widget.set_header_battery("N/A")
        self._map_widget.set_header_remote_id("N/A")
        self._map_widget.set_plan_vehicle_info("—", "—")
        self._mission_table_updating = True
        self._mission_table.setRowCount(0)
        self._mission_table_updating = False
        self._last_map_overlay_refresh_s = None
        self._sync_visible_map_overlay_metrics()
        self._refresh_footer_summary()
        self._sync_plan_flight_chrome()

    def _on_connect(self) -> None:
        if self._thread is not None and self._thread.isRunning():
            QMessageBox.warning(self, "VGCS", "Already connected.")
            return

        cs = self._conn_edit.text().strip()
        if not cs:
            QMessageBox.warning(self, "VGCS", "Enter a connection string.")
            return

        self._timeout_s = float(self._timeout_spin.value())
        self._settings.setValue("watchdog_timeout_s", self._timeout_s)
        self._settings.setValue("last_connection_string", cs)
        self._thread = MavlinkThread(cs, timeout_s=self._timeout_s)
        self._thread.log_line.connect(self._append_log)
        self._thread.error.connect(self._on_link_error)
        self._thread.link_up.connect(self._on_link_up)
        self._thread.link_down.connect(self._on_link_down)
        self._thread.heartbeat.connect(self._on_heartbeat)
        self._thread.telemetry.connect(self._on_telemetry)
        self._thread.link_timeout.connect(self._on_link_timeout)
        self._thread.mission_uploaded.connect(self._on_mission_uploaded)
        self._thread.mission_downloaded.connect(self._on_mission_downloaded)
        self._thread.mode_changed.connect(self._on_mode_change_result)
        self._thread.action_result.connect(self._on_action_result)
        self._thread.geofence_result.connect(self._on_geofence_result)
        self._thread.params_snapshot.connect(self._on_params_snapshot)
        self._thread.param_set_result.connect(self._on_param_set_result)
        self._thread.finished.connect(self._on_thread_finished)
        # Keep the click handler light: Skydroid / map camera wiring can touch sockets and QSettings.
        def _deferred_camera_after_connect() -> None:
            try:
                self._set_runtime_camera_control()
            except Exception:
                try:
                    self._map_widget.set_camera_control(NoopCameraControl())
                except Exception:
                    pass

        QTimer.singleShot(0, _deferred_camera_after_connect)

        self._btn_connect.setEnabled(False)
        self._conn_edit.setEnabled(False)
        self._timeout_spin.setEnabled(False)
        self._theme_combo.setEnabled(False)
        self._mode_combo.setEnabled(False)
        self._btn_set_mode.setEnabled(False)
        self._btn_takeoff.setEnabled(False)
        self._btn_land.setEnabled(False)
        self._btn_auto_takeoff.setEnabled(False)
        self._btn_auto_land.setEnabled(False)
        self._btn_emergency_stop.setEnabled(False)
        self._btn_apply_failsafe_preset.setEnabled(False)
        self._btn_apply_fence.setEnabled(False)
        self._btn_params_refresh.setEnabled(False)
        self._btn_param_set.setEnabled(False)
        self._btn_apply_acro.setEnabled(False)
        self._btn_apply_simple.setEnabled(False)
        self._btn_reset.setEnabled(False)
        self._heartbeat_seen = False
        self._connect_attempt_active = True
        self._arm_not_ready_alert_shown = False
        self._arm_not_ready_since_mono = None
        self._recent_statustext.clear()
        self._status.setText("Connecting…")
        self._apply_state_style(self._status, "warn")
        self._set_dashboard_flight_status("yellow", "Connecting to vehicle...")
        self._thread.start()

    def _on_disconnect(self) -> None:
        self._stop_camera_control_backend()
        if self._thread is not None:
            self._thread.stop()
            if self._thread.isRunning():
                self._thread.wait(8000)
        try:
            self._map_widget.set_camera_control(NoopCameraControl())
        except Exception:
            pass

    def _stop_camera_control_backend(self) -> None:
        cc = self._camera_control_backend
        self._camera_control_backend = None
        if cc is None:
            return
        try:
            close = getattr(cc, "close", None)
            if callable(close):
                close()
        except Exception:
            return

    def _deferred_apply_saved_video_settings(self) -> None:
        """Apply saved video + camera settings after the settings dialog has closed.

        RTSP teardown and WebEngine updates can take noticeable time; running that work
        while a modal dialog is still in ``exec()`` makes Windows report the app as hung.
        """
        def _apply_body() -> None:
            try:
                self._map_widget.apply_video_settings_for_settings_dialog()
            except Exception:
                pass
            QTimer.singleShot(600, self._deferred_apply_saved_video_settings_camera)

        # Yield one event-loop turn so the dialog teardown / WM_PAINT can finish before heavy work.
        QTimer.singleShot(0, _apply_body)

    def _deferred_apply_saved_video_settings_camera(self) -> None:
        """Run after staged video pipeline work so camera hot-swap does not pile on the same burst."""

        def _do() -> None:
            try:
                if self._thread is not None and self._thread.isRunning():
                    self._set_runtime_camera_control()
            except Exception:
                pass
            try:
                self._append_log("Video settings applied.")
            except Exception:
                pass

        QTimer.singleShot(0, _do)

    def _set_runtime_camera_control(self) -> None:
        self._stop_camera_control_backend()
        provider = str(self._settings.value("camera/provider", "mavlink") or "mavlink").strip().lower()
        if provider == "siyi":
            host = resolve_siyi_host(self._settings)
            port = int(self._settings.value("camera/siyi_port", 37260) or 37260)
            timeout_ms = int(self._settings.value("camera/siyi_timeout_ms", 250) or 250)
            cc = SiyiCameraControl(
                host=host,
                port=port,
                timeout_s=max(0.05, float(timeout_ms) / 1000.0),
            )
            self._wire_camera_control(cc)
            self._append_log(f"Camera control: SIYI SDK UDP {host}:{port} (gimbal attitude 0x0D)")
            return
        if provider == "skydroid":
            hosts = resolve_skydroid_control_hosts(self._settings)
            host = hosts[0] if hosts else resolve_skydroid_host(self._settings)
            port = int(self._settings.value("camera/skydroid_port", 5000) or 5000)
            timeout_ms = int(self._settings.value("camera/skydroid_timeout_ms", 250) or 250)
            profile_id = str(self._settings.value("camera/skydroid_profile", "c13_default") or "c13_default")
            cc = SkydroidCameraControl(
                host=host,
                hosts=hosts,
                port=port,
                timeout_s=max(0.05, float(timeout_ms) / 1000.0),
                log_path=str(Path.cwd() / "logs" / "skydroid_top_udp.log"),
                profile_id=profile_id,
            )
            self._wire_camera_control(cc)
            ah, ap, pid = cc._adapter.active_endpoint()
            tried = ", ".join(getattr(cc._adapter, "_hosts", [])[:5])
            self._append_log(
                f"Camera control: Skydroid TOP UDP {ah}:{ap} profile={pid} "
                f"(probe hosts: {tried}; MAVLink mount fallback)"
            )
            def _skydroid_gimbal_hint() -> None:
                try:
                    if cc._adapter.gimbal_telemetry_ok():
                        ah2, ap2, pid2 = cc._adapter.active_endpoint()
                        self._append_log(f"Skydroid gimbal OK: TOP UDP {ah2}:{ap2} profile={pid2}")
                        return
                except Exception:
                    pass
                self._append_log(
                    "Skydroid gimbal: no TOP attitude yet — if RTSP works on RC Wi-Fi hotspot, "
                    "set Host to the RC gateway (e.g. 192.168.43.1) or connect PC Ethernet to the camera"
                )

            QTimer.singleShot(4000, _skydroid_gimbal_hint)
            return
        cc = MavlinkCameraControl(self._thread)
        self._wire_camera_control(cc)
        self._append_log("Camera control: MAVLink mount / gimbal attitude")

    def _on_link_up(self) -> None:
        self._map_widget.clear_flight_track()
        self._map_widget.set_mission_nav_seq(0)
        self._auto_center_pending = True
        self._status.setText("Port open, waiting for heartbeat…")
        self._apply_state_style(self._status, "warn")
        self._btn_disconnect.setEnabled(True)
        self._watchdog.setText(f"OK · {self._timeout_s:.1f}s")
        self._apply_state_style(self._watchdog, "ok")
        self._set_dashboard_flight_status("yellow", "Link open - waiting for heartbeat")
        self._mode_combo.setEnabled(True)
        self._btn_set_mode.setEnabled(True)
        self._btn_takeoff.setEnabled(True)
        self._btn_land.setEnabled(True)
        self._btn_auto_takeoff.setEnabled(True)
        self._btn_auto_land.setEnabled(True)
        self._btn_emergency_stop.setEnabled(True)
        self._btn_apply_failsafe_preset.setEnabled(True)
        self._btn_apply_fence.setEnabled(True)
        self._btn_params_refresh.setEnabled(True)
        self._btn_param_set.setEnabled(True)
        self._btn_apply_acro.setEnabled(True)
        self._btn_apply_simple.setEnabled(True)
        # Do not mark connected in map UI until HEARTBEAT is actually received.
        self._map_widget.set_link_connected(False)
        self._set_preconnect_dashboard_mode(False)
        self._set_map_only_dashboard_mode(self._map_only_dashboard)
        self._sync_plan_flight_chrome()

    def _on_link_down(self) -> None:
        self._map_widget.clear_flight_track()
        self._map_widget.set_mission_nav_seq(0)
        self._auto_center_pending = True
        self._connect_attempt_active = False
        self._arm_not_ready_alert_shown = False
        self._arm_not_ready_since_mono = None
        self._rid_live_available = False
        self._mission_upload_pending = False
        self._status.setText("Disconnected")
        self._apply_state_style(self._status, "bad")
        self._hb.setText("—")
        self._heartbeat_seen = False
        self._apply_state_style(self._hb, "na")
        self._watchdog.setText(f"Idle · {self._timeout_s:.1f}s")
        self._apply_state_style(self._watchdog, "warn")
        self._compass.clear()
        self._btn_connect.setEnabled(True)
        self._conn_edit.setEnabled(True)
        self._timeout_spin.setEnabled(True)
        self._theme_combo.setEnabled(True)
        self._mode_combo.setEnabled(True)
        self._btn_reset.setEnabled(True)
        self._btn_disconnect.setEnabled(False)
        self._btn_set_mode.setEnabled(False)
        self._btn_takeoff.setEnabled(False)
        self._btn_land.setEnabled(False)
        self._btn_auto_takeoff.setEnabled(False)
        self._btn_auto_land.setEnabled(False)
        self._btn_emergency_stop.setEnabled(False)
        self._btn_apply_failsafe_preset.setEnabled(False)
        self._btn_apply_fence.setEnabled(False)
        self._btn_params_refresh.setEnabled(False)
        self._btn_param_set.setEnabled(False)
        self._btn_apply_acro.setEnabled(False)
        self._btn_apply_simple.setEnabled(False)
        self._reset_telemetry_fields()
        self._set_dashboard_flight_status(
            "red",
            "Communication lost - Not Ready to Arm",
        )
        self._map_widget.set_link_connected(False)
        self._set_preconnect_dashboard_mode(True)
        self._set_map_only_dashboard_mode(self._map_only_dashboard)
        self._sync_plan_flight_chrome()

    def _format_recent_vehicle_msgs_for_alert(self) -> str:
        """Summarize recent STATUSTEXT lines for the not-ready dialog."""
        if not self._recent_statustext:
            return ""
        keys = (
            "prearm",
            "pre-arm",
            "arm:",
            "disarm",
            "fence",
            "rangefinder",
            "gps",
            "ekf",
            "compass",
            "failsafe",
            "rc not",
            "throttle",
            "calib",
            "error",
            "fail",
        )
        picked: list[str] = []
        for line in self._recent_statustext:
            low = line.lower()
            if any(k in low for k in keys):
                picked.append(line)
        show = picked[-5:] if picked else list(self._recent_statustext)[-5:]
        if not show:
            return ""
        return "Recent vehicle messages:\n" + "\n".join(f"• {s}" for s in show)

    def _on_heartbeat(self, sysid: int, compid: int, mav_ver: int) -> None:
        if not self._heartbeat_seen:
            self._heartbeat_seen = True
            self._connect_attempt_active = False
            self._status.setText("Connected")
            self._apply_state_style(self._status, "ok")
            self._set_dashboard_flight_status("yellow", "Connected - validating arm checks")
            self._map_widget.set_link_connected(True)
        self._hb.setText(f"sys {sysid} · comp {compid} · mav {mav_ver}")
        self._apply_state_style(self._hb, "ok")
        if not self._rid_live_available:
            self._map_widget.set_header_remote_id(f"ID {sysid}")
        self._sync_plan_flight_chrome()

    def _on_telemetry(self, msg_type: str, payload: object) -> None:
        data = payload if isinstance(payload, dict) else {}
        if msg_type == "HEARTBEAT":
            armed = bool(data.get("armed", False))
            self._fields["armed"].setText("Yes" if armed else "No")
            self._apply_state_style(self._fields["armed"], "ok" if armed else "warn")
            if armed and self._armed_since is None:
                self._armed_since = time.monotonic()
            if not armed:
                self._armed_since = None
                self._fields["flight_time"].setText("00:00")
            system_status = int(data.get("system_status", 0))
            standby = int(mavutil.mavlink.MAV_STATE_STANDBY)
            arm_ready = system_status >= standby
            self._fields["arm_ready"].setText("Likely ready" if arm_ready else f"System status {system_status}")
            self._apply_state_style(self._fields["arm_ready"], "ok" if arm_ready else "warn")
            if arm_ready:
                # Do not clear _arm_not_ready_alert_shown here: brief STANDBY in a flickering
                # HEARTBEAT would re-arm the popup and make OK / title-bar close feel ignored.
                self._arm_not_ready_since_mono = None
                self._set_dashboard_flight_status(
                    "green",
                    "Parameter downloading... Ready to Arm",
                )
            else:
                self._set_dashboard_flight_status(
                    "yellow",
                    "Connected - Not Ready to Arm",
                )
                now = time.monotonic()
                if self._arm_not_ready_since_mono is None:
                    self._arm_not_ready_since_mono = now
                if (
                    not self._arm_not_ready_alert_shown
                    and (now - self._arm_not_ready_since_mono) >= 5.0
                ):
                    self._arm_not_ready_alert_shown = True
                    extra = self._format_recent_vehicle_msgs_for_alert()
                    body = (
                        "Vehicle connected, but the autopilot heartbeat still reports "
                        f"system_status={system_status} (not STANDBY / ready yet).\n\n"
                        "This often clears within a few seconds while the vehicle boots. "
                        "If it does not clear, check calibration, GPS/EKF, and other PreArm messages."
                    )
                    if extra:
                        body = f"{body}\n\n{extra}"
                    QMessageBox.warning(self, "Vehicle Msg", body)
            mode_text = human_mode_name(
                vehicle_type=int(data.get("vehicle_type", 0) or 0),
                custom_mode=int(data.get("custom_mode", 0) or 0),
            )
            self._sync_mode_options_for_vehicle(int(data.get("vehicle_type", 0) or 0))
            self._top_flight_mode.setText(mode_text)
            self._map_widget.set_header_mode(mode_text)
            # Keep mode dropdown aligned with the vehicle (dialog copies this combo at open).
            if mode_text and self._mode_combo.findText(mode_text) >= 0:
                self._mode_combo.blockSignals(True)
                self._mode_combo.setCurrentText(mode_text)
                self._mode_combo.blockSignals(False)
            ap = int(data.get("autopilot", 0) or 0)
            vt = int(data.get("vehicle_type", 0) or 0)
            self._map_widget.set_plan_vehicle_info(
                _mavlink_autopilot_label(ap),
                _mavlink_vehicle_type_label(vt),
            )
        elif msg_type == "GLOBAL_POSITION_INT":
            lat = float(data.get("lat", 0.0))
            lon = float(data.get("lon", 0.0))
            # Some vehicles output 0,0 before GPS has a valid fix. Never push that into the map,
            # otherwise the UI recenters to the Gulf of Guinea and loads misleading/placeholder tiles.
            if abs(lat) < 1e-9 and abs(lon) < 1e-9:
                return
            self._map_rel_alt_m = float(data.get("relative_alt_m", 0.0))
            self._map_msl_alt_m = float(data.get("alt_msl_m", 0.0))
            self._home_amsl_m = float(self._map_msl_alt_m) - float(self._map_rel_alt_m)
            if str(self._settings.value("plan_alt_ref", "rel") or "").lower() == "amsl":
                s = self._settings
                st = {
                    "altRef": str(s.value("plan_alt_ref", "rel") or "rel"),
                    "initialWpAltFt": float(s.value("plan_initial_wp_alt_ft", 164.0) or 164.0),
                }
                self._map_widget.set_default_waypoint_alt_m(
                    self._default_wp_alt_m_for_plan_state(st)
                )
            if self._home_lat is None or self._home_lon is None:
                self._home_lat = lat
                self._home_lon = lon
                self._home_rel_alt_baseline_m = float(data.get("relative_alt_m", 0.0))
            if self._home_lat is not None and self._home_lon is not None:
                try:
                    if self._map_widget.is_map_motion_armed():
                        pos = self._map_widget.get_vehicle_display_position()
                        if pos is not None:
                            d = self._haversine_m(
                                self._home_lat, self._home_lon, pos[0], pos[1]
                            )
                            self._max_telem_dist_m = max(self._max_telem_dist_m, d)
                except Exception:
                    pass
            self._fields["lat_lon"].setText(
                f"{data.get('lat', 0.0):.7f}, {data.get('lon', 0.0):.7f}"
            )
            self._fields["alt_rel"].setText(f"{data.get('relative_alt_m', 0.0):.1f} m")
            self._fields["alt_msl"].setText(f"{data.get('alt_msl_m', 0.0):.1f} m")
            self._map_widget.set_vehicle_position(
                lat,
                lon,
                relative_alt_m=float(data.get("relative_alt_m", 0.0)),
                groundspeed_mps=float(
                    data.get("groundspeed_mps", self._map_groundspeed_mps) or 0.0
                ),
            )
            if self._auto_center_pending:
                self._auto_center_pending = False
                self._map_widget.center_on_vehicle()
            # Fused course / velocity course matches map motion better than body yaw alone.
            course: float | None = None
            if data.get("hdg_deg") is not None:
                course = float(data["hdg_deg"])
            elif data.get("ground_track_deg") is not None:
                course = float(data["ground_track_deg"])
            if course is not None:
                self._heading = course
                self._fields["heading"].setText(f"{int(round(course))}°")
                self._compass.set_heading_deg(course)
                self._map_widget.set_vehicle_heading(course, source="gpi")
        elif msg_type == "MISSION_CURRENT":
            self._map_widget.set_mission_nav_seq(int(data.get("seq", 0) or 0))
        elif msg_type == "VFR_HUD":
            self._map_groundspeed_mps = float(data.get("groundspeed", 0.0))
            self._map_climb_mps = float(data.get("climb", 0.0))
            try:
                self._map_widget._update_map_motion_state(self._map_groundspeed_mps)
            except Exception:
                pass
            self._fields["groundspeed"].setText(f"{data.get('groundspeed', 0.0):.1f} m/s")
            self._fields["airspeed"].setText(f"{data.get('airspeed', 0.0):.1f} m/s")
            hd = float(data.get("heading", 0.0))
            self._fields["heading"].setText(f"{int(hd)}°")
            self._heading = hd
            self._compass.set_heading_deg(hd)
            self._map_widget.set_vehicle_heading(hd, source="vfr")
        elif msg_type == "ATTITUDE":
            self._fields["attitude"].setText(
                f"{data.get('roll_deg', 0.0):.1f} / "
                f"{data.get('pitch_deg', 0.0):.1f} / "
                f"{data.get('yaw_deg', 0.0):.1f} deg"
            )
            yaw_deg = float(data.get("yaw_deg", 0.0))
            hd_att = (yaw_deg + 360.0) % 360.0
            self._heading = hd_att
            self._compass.set_heading_deg(hd_att)
            self._map_widget.set_vehicle_heading(hd_att, source="att")
        elif msg_type == "GPS_RAW_INT":
            hdop = data.get("hdop")
            hdop_text = "N/A" if hdop is None else f"{hdop:.2f}"
            sat = int(data.get("satellites_visible", 0))
            fix_type = int(data.get("fix_type", 0) or 0)
            self._fields["gps"].setText(
                f"fix={fix_type} sat={sat} hdop={hdop_text}"
            )
            self._top_gps_sat.setText(str(sat))
            self._top_gps_hdop.setText(hdop_text)
            self._map_widget.set_header_gps(sat, hdop_text, fix_type=fix_type)
            # Fallback when GLOBAL_POSITION_INT is 0,0 / missing but GPS_RAW has a fix.
            raw_lat = data.get("lat")
            raw_lon = data.get("lon")
            if raw_lat is not None and raw_lon is not None:
                lat = float(raw_lat)
                lon = float(raw_lon)
                if fix_type >= 2 and (abs(lat) > 1e-9 or abs(lon) > 1e-9):
                    alt_msl = float(data.get("alt_msl_m", 0.0) or 0.0)
                    self._map_widget.set_vehicle_position(
                        lat,
                        lon,
                        relative_alt_m=None,
                        groundspeed_mps=float(self._map_groundspeed_mps),
                    )
                    if alt_msl:
                        self._map_msl_alt_m = alt_msl
        elif msg_type == "SYS_STATUS":
            pct = int(data.get("battery_remaining", -1))
            pct_text = "N/A" if pct < 0 else f"{pct}%"
            voltage = float(data.get("voltage_v", 0.0))
            current = float(data.get("current_a", -1.0))
            current_text = "N/A" if current < 0 else f"{current:.1f} A"
            self._fields["battery"].setText(
                f"{voltage:.2f} V, {current_text}, {pct_text}"
            )
            # Always show voltage in the header; percent alone is not actionable for operators.
            # Use 2 decimals so small real-time changes are visible (e.g. 11.80 -> 11.74).
            bat_header = (
                f"{voltage:.2f}V ({pct_text})" if pct_text != "N/A" else f"{voltage:.2f}V"
            )
            self._top_battery.setText(bat_header)
            self._map_widget.set_header_battery(bat_header)
            sensors_present = int(data.get("sensors_present", 0))
            sensors_enabled = int(data.get("sensors_enabled", 0))
            sensors_health = int(data.get("sensors_health", 0))
            battery_mask = int(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_BATTERY)
            rc_mask = int(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_RC_RECEIVER)
            battery_monitored = bool(sensors_present & battery_mask and sensors_enabled & battery_mask)
            rc_monitored = bool(sensors_present & rc_mask and sensors_enabled & rc_mask)
            battery_healthy = bool(sensors_health & battery_mask)
            rc_healthy = bool(sensors_health & rc_mask)
            battery_ok = (not battery_monitored or battery_healthy) and (pct < 0 or pct > 15)
            self._set_ok_warn_field("failsafe_battery", battery_ok)
            if rc_monitored:
                self._set_ok_warn_field("failsafe_rc", rc_healthy)
            else:
                self._fields["failsafe_rc"].setText("N/A")
                self._apply_state_style(self._fields["failsafe_rc"], "na")
        elif msg_type == "BATTERY_STATUS":
            # Real vehicles often report pack voltage here (Mission Planner simulator may not).
            # MAVLink: voltages[] in mV, battery_remaining in % (or -1).
            pct = int(data.get("battery_remaining", -1))
            pct_text = "N/A" if pct < 0 else f"{pct}%"
            v_mv = None
            try:
                v_arr = data.get("voltages")
                if isinstance(v_arr, (list, tuple)) and v_arr:
                    v0 = int(v_arr[0] or 0)
                    if v0 > 0:
                        v_mv = v0
            except Exception:
                v_mv = None
            voltage_v = float(data.get("voltage_v", 0.0) or 0.0)
            if (not voltage_v or voltage_v <= 0.1) and v_mv is not None:
                voltage_v = float(v_mv) / 1000.0
            if voltage_v <= 0.1:
                return
            bat_header = (
                f"{voltage_v:.2f}V ({pct_text})" if pct_text != "N/A" else f"{voltage_v:.2f}V"
            )
            self._top_battery.setText(bat_header)
            self._map_widget.set_header_battery(bat_header)
        elif msg_type == "OBSTACLE_DISTANCE":
            self._map_widget.set_obstacle_distance(data)
            prox, _ = self._map_widget.get_obstacle_sensor_summary()
            self._fields["obstacle_prox"].setText(prox)
            self._apply_state_style(
                self._fields["obstacle_prox"],
                "ok" if prox != "N/A" else "na",
            )
        elif msg_type == "DISTANCE_SENSOR":
            self._map_widget.set_distance_sensor(data)
            _, rf = self._map_widget.get_obstacle_sensor_summary()
            self._fields["rangefinder"].setText(rf)
            self._apply_state_style(
                self._fields["rangefinder"],
                "ok" if rf != "N/A" else "na",
            )
        elif msg_type == "RADIO_STATUS":
            self._fields["rc_link"].setText(
                f"rssi={int(data.get('rssi', 0))} remrssi={int(data.get('remrssi', 0))}"
            )
            self._apply_state_style(self._fields["rc_link"], "ok")
        elif msg_type == "STATUSTEXT":
            text = str(data.get("text", "")).strip()
            if text:
                self._recent_statustext.append(text)
                self._top_vehicle_msg.setText(text)
                self._map_widget.set_header_vehicle_msg(text)
                # STATUSTEXT can burst during param download; logging each line hammers QTextEdit.
                now = time.monotonic()
                last_log = float(getattr(self, "_last_statustext_log_mono", 0.0))
                if now - last_log >= 0.12:
                    self._last_statustext_log_mono = now
                    self._append_log(f"STATUSTEXT: {text}")
        elif msg_type.startswith("OPEN_DRONE_ID_"):
            rid_text = self._extract_remote_id_text(data)
            if rid_text:
                self._rid_live_available = True
                rid_display = f"RID: {rid_text}"
                self._top_remote_id.setText(rid_display)
                self._map_widget.set_header_remote_id(rid_display)
        self._refresh_footer_summary()
        self._maybe_refresh_map_web_overlays()

    def _on_set_mode(self) -> None:
        mode_name = self._mode_combo.currentText().strip()
        if not mode_name:
            return
        if self._thread is None or not self._thread.isRunning():
            QMessageBox.warning(self, "VGCS", "Connect vehicle before mode change.")
            return
        self._thread.queue_mode_change(mode_name)
        self._append_log(f"Mode change queued: {mode_name}")

    def _on_mode_change_result(self, mode_name: str, ok: bool) -> None:
        if ok:
            self._append_log(f"Mode change requested: {mode_name}")
            self._top_vehicle_msg.setText(f"Mode cmd: {mode_name}")
        else:
            self._append_log(f"Mode change failed: {mode_name}")
            self._top_vehicle_msg.setText("Mode change failed")

    def _takeoff_altitude_m(self, *, from_plan_rail: bool) -> float:
        """Target climb (m) for NAV_TAKEOFF: plan launch alt when set on rail; else dashboard spin."""
        if from_plan_rail:
            plan_alt = self._plan_takeoff_alt_m_from_launch_settings()
            if plan_alt is not None:
                return max(1.0, float(plan_alt))
        return max(1.0, float(self._takeoff_alt_spin.value()))

    def _queue_nav_takeoff(self, alt_m: float) -> None:
        if self._thread is None or not self._thread.isRunning():
            QMessageBox.warning(self, "VGCS", "Connect vehicle before takeoff command.")
            return
        alt = max(1.0, float(alt_m))
        self._thread.queue_takeoff(alt)
        self._append_log(f"Takeoff queued: {alt:.1f}m")

    def _on_takeoff(self) -> None:
        self._queue_nav_takeoff(self._takeoff_altitude_m(from_plan_rail=False))

    def _on_land(self) -> None:
        if self._thread is None or not self._thread.isRunning():
            QMessageBox.warning(self, "VGCS", "Connect vehicle before land command.")
            return
        self._thread.queue_land()
        self._append_log("Land queued")

    def _on_auto_takeoff(self) -> None:
        if self._thread is None or not self._thread.isRunning():
            QMessageBox.warning(self, "VGCS", "Connect vehicle before auto takeoff.")
            return
        alt = float(self._takeoff_alt_spin.value())
        self._thread.queue_auto_takeoff(alt)
        self._append_log(f"Auto takeoff queued: arm + takeoff {alt:.1f} m")

    def _on_auto_land(self) -> None:
        if self._thread is None or not self._thread.isRunning():
            QMessageBox.warning(self, "VGCS", "Connect vehicle before auto land.")
            return
        self._thread.queue_auto_land()
        self._append_log("Auto land queued (LAND mode or NAV_LAND)")

    def _on_emergency_motor_stop(self) -> None:
        if self._thread is None or not self._thread.isRunning():
            QMessageBox.warning(self, "VGCS", "Connect vehicle before emergency stop.")
            return
        value, ok = QInputDialog.getText(
            self,
            "Emergency motor stop",
            "This will STOP MOTORS immediately (forced disarm).\n"
            "This may crash the drone and cause injury/damage.\n\n"
            "Type STOP to confirm:",
            QLineEdit.EchoMode.Normal,
            "",
        )
        if not ok:
            return
        if str(value).strip().upper() != "STOP":
            QMessageBox.information(self, "VGCS", "Emergency stop cancelled.")
            return
        self._thread.queue_emergency_motor_stop()
        self._append_log("EMERGENCY STOP queued: forced motor stop")

    def _on_apply_m1_failsafes(self) -> None:
        if self._thread is None or not self._thread.isRunning():
            QMessageBox.warning(self, "VGCS", "Connect vehicle before applying failsafes.")
            return
        # M1 baseline:
        # - GCS disconnect: RTL (FS_GCS_ENABLE=1)
        # - RC failsafe: RTL (FS_THR_ENABLE=1)
        # - Battery failsafe: LOW -> RTL (BATT_FS_LOW_ACT=2), CRIT -> Land (BATT_FS_CRT_ACT=1)
        self._thread.queue_param_set("FS_GCS_ENABLE", 1.0)
        self._thread.queue_param_set("FS_THR_ENABLE", 1.0)
        self._thread.queue_param_set("BATT_FS_LOW_ACT", 2.0)
        self._thread.queue_param_set("BATT_FS_CRT_ACT", 1.0)
        self._append_log("Failsafe preset queued: GCS=RTL, RC=RTL, BATT low=RTL, batt crit=Land")
        try:
            low_v = float(self._last_params.get("BATT_LOW_VOLT", 0.0) or 0.0)
            low_mah = float(self._last_params.get("BATT_LOW_MAH", 0.0) or 0.0)
            if low_v <= 0.0 and low_mah <= 0.0:
                self._append_log(
                    "Note: Battery failsafe trigger is disabled (BATT_LOW_VOLT and BATT_LOW_MAH are 0). "
                    "Set a threshold on the vehicle to activate battery failsafe."
                )
        except Exception:
            pass

    def _on_upload_fence(self) -> None:
        if self._thread is None or not self._thread.isRunning():
            QMessageBox.warning(self, "VGCS", "Connect vehicle before fence upload.")
            return
        cfg = {
            "radius_m": float(self._geofence_radius_spin.value()),
            "alt_max_m": float(self._geofence_alt_max_spin.value()),
            "action": float(self._geofence_action_combo.currentData() or 1.0),
        }
        self._thread.queue_geofence_upload(cfg)
        self._append_log(
            f"Fence upload queued: r={cfg['radius_m']:.0f}m alt={cfg['alt_max_m']:.0f}m action={int(cfg['action'])}"
        )

    def _on_map_geofence_requested(self, cfg: object) -> None:
        if self._thread is None or not self._thread.isRunning():
            return
        if isinstance(cfg, dict):
            self._thread.queue_geofence_upload(cfg)

    def _suppress_header_connect_spurious_reopen(self) -> None:
        """Eat stray mouse-ups delivered to the banner right after a modal closes (Cancel/OK)."""
        self._suppress_header_connect_after_dialog = True
        QTimer.singleShot(450, self._clear_header_connect_suppression)

    def _clear_header_connect_suppression(self) -> None:
        self._suppress_header_connect_after_dialog = False

    def _on_map_connect_requested(self) -> None:
        # Header click must always request an explicit connection string.
        if self._suppress_header_connect_after_dialog:
            return
        current = self._conn_edit.text().strip()
        if not current:
            current = str(self._settings.value("last_connection_string", "udp:127.0.0.1:14550"))

        value, ok = QInputDialog.getText(
            self,
            "Connect Vehicle",
            "MAVLink connection string:",
            QLineEdit.EchoMode.Normal,
            current,
        )
        self._suppress_header_connect_spurious_reopen()
        if not ok:
            return
        connection_string = value.strip()
        if not connection_string:
            QMessageBox.warning(self, "VGCS", "Enter a connection string.")
            return
        # Ensure this always triggers a real connection attempt with the entered link.
        if self._thread is not None and self._thread.isRunning():
            self._on_disconnect()
        self._conn_edit.setText(connection_string)
        self._append_log(f"Manual connect requested: {connection_string}")
        self._on_connect()

    def _on_map_return_requested(self) -> None:
        if self._thread is None or not self._thread.isRunning():
            QMessageBox.warning(self, "VGCS", "Connect vehicle before return command.")
            return
        self._thread.queue_mode_change("RTL")
        self._append_log("Mode change queued: RTL")

    def _on_map_mission_start_requested(self) -> None:
        if self._thread is None or not self._thread.isRunning():
            QMessageBox.warning(self, "VGCS", "Connect vehicle before mission start.")
            return
        model = list(getattr(self._map_widget, "_waypoints_model", []))
        if not model:
            QMessageBox.warning(
                self,
                "Mission Start",
                "No waypoints available. Create/import waypoints first.",
            )
            return
        armed_text = self._fields.get("armed").text().strip().lower()
        if armed_text != "yes":
            QMessageBox.information(
                self,
                "Mission Start",
                "Vehicle is not armed.\nThe link will switch to an armable mode, arm, then run AUTO + mission start.",
            )
        payload = [{"lat": float(wp.lat), "lon": float(wp.lon), "alt_m": float(wp.alt_m)} for wp in model]
        takeoff_m = self._plan_takeoff_alt_m_from_launch_settings()
        if takeoff_m is not None and payload:
            payload[0] = {**payload[0], "takeoff_alt_m": float(takeoff_m)}
        self._thread.queue_mission_upload(payload)
        self._thread.queue_mission_start()
        self._append_log(
            f"Mission start queued: upload {len(payload)} WPs (+TAKEOFF item) + AUTO start"
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

    def _on_params_refresh(self) -> None:
        if self._thread is None or not self._thread.isRunning():
            QMessageBox.warning(self, "VGCS", "Connect vehicle before parameter fetch.")
            return
        names = [
            "WPNAV_SPEED",
            "RTL_ALT",
            "FENCE_ENABLE",
            "FENCE_RADIUS",
            "ARMING_CHECK",
            "ACRO_OPTIONS",
            "ACRO_TRAINER",
            "SIMPLE",
            "SUPER_SIMPLE",
        ]
        self._thread.queue_params_fetch(names)
        self._append_log("Param fetch queued")

    def _on_param_set(self) -> None:
        if self._thread is None or not self._thread.isRunning():
            QMessageBox.warning(self, "VGCS", "Connect vehicle before parameter set.")
            return
        name = self._param_name_combo.currentText().strip()
        value = float(self._param_value_spin.value())
        self._thread.queue_param_set(name, value)
        self._append_log(f"Param set queued: {name}={value}")

    def _on_action_result(self, action: str, ok: bool, detail: str) -> None:
        msg = f"{action.upper()} {'OK' if ok else 'FAIL'}: {detail}"
        self._append_log(msg)
        self._top_vehicle_msg.setText(msg[:80])

    def _on_geofence_result(self, ok: bool, detail: str) -> None:
        msg = f"Fence {'OK' if ok else 'FAIL'}: {detail}"
        self._append_log(msg)
        self._top_vehicle_msg.setText(msg[:80])

    def _on_params_snapshot(self, payload: object) -> None:
        data = dict(payload) if isinstance(payload, dict) else {}

        def _apply() -> None:
            self._apply_params_snapshot_payload(data)

        QTimer.singleShot(0, _apply)

    def _apply_params_snapshot_payload(self, data: dict) -> None:
        if not data:
            self._append_log("Param fetch: no values")
            return
        for k, v in data.items():
            try:
                self._last_params[str(k).strip().upper()] = float(v)
            except Exception:
                continue
        cur = self._param_name_combo.currentText().strip().upper()
        if cur in data:
            self._param_value_spin.setValue(float(data[cur]))
        self._refresh_acro_options_ui()
        n = len(data)
        if n <= 48:
            joined = ", ".join(f"{k}={v:.3f}" for k, v in sorted(data.items()))
            self._append_log(f"Params: {joined}")
        else:
            sample = ", ".join(f"{k}={v:.3f}" for k, v in list(sorted(data.items()))[:24])
            self._append_log(f"Params ({n} values, log truncated): {sample}, …")

    def _refresh_acro_options_ui(self) -> None:
        opts = int(self._last_params.get("ACRO_OPTIONS", 0.0) or 0.0)
        self._airmode_check.blockSignals(True)
        self._airmode_check.setChecked(bool(opts & 1))
        self._airmode_check.blockSignals(False)
        trainer = int(self._last_params.get("ACRO_TRAINER", 2.0) or 2.0)
        self._acro_trainer_combo.blockSignals(True)
        self._acro_trainer_combo.setCurrentIndex(max(0, min(2, trainer)))
        self._acro_trainer_combo.blockSignals(False)
        simple_mask = int(self._last_params.get("SIMPLE", 0.0) or 0.0)
        super_mask = int(self._last_params.get("SUPER_SIMPLE", 0.0) or 0.0)
        self._simple_check.blockSignals(True)
        self._simple_check.setChecked(simple_mask != 0)
        self._simple_check.blockSignals(False)
        self._super_simple_check.blockSignals(True)
        self._super_simple_check.setChecked(super_mask != 0)
        self._super_simple_check.blockSignals(False)

    def _on_apply_acro_options(self) -> None:
        if self._thread is None or not self._thread.isRunning():
            QMessageBox.warning(self, "VGCS", "Connect vehicle before applying Acro options.")
            return
        cur_opts = int(self._last_params.get("ACRO_OPTIONS", 0.0) or 0.0)
        want_airmode = bool(self._airmode_check.isChecked())
        new_opts = (cur_opts | 1) if want_airmode else (cur_opts & ~1)
        trainer = int(self._acro_trainer_combo.currentIndex())
        self._thread.queue_param_set("ACRO_OPTIONS", float(new_opts))
        self._thread.queue_param_set("ACRO_TRAINER", float(trainer))
        self._append_log(
            f"Acro options queued: ACRO_OPTIONS={new_opts} (AirMode={'on' if want_airmode else 'off'}), "
            f"ACRO_TRAINER={trainer}"
        )

    def _on_apply_simple_options(self) -> None:
        if self._thread is None or not self._thread.isRunning():
            QMessageBox.warning(self, "VGCS", "Connect vehicle before applying Simple options.")
            return
        # SIMPLE and SUPER_SIMPLE are bitmasks for flight-mode switch positions (1..6).
        # For a simple UI, we enable/disable for all switch positions at once (0x3F).
        simple_mask = 0x3F if self._simple_check.isChecked() else 0
        super_mask = 0x3F if self._super_simple_check.isChecked() else 0
        self._thread.queue_param_set("SIMPLE", float(simple_mask))
        self._thread.queue_param_set("SUPER_SIMPLE", float(super_mask))
        self._append_log(
            f"Simple options queued: SIMPLE={simple_mask} SUPER_SIMPLE={super_mask}"
        )

    def _on_param_set_result(self, name: str, ok: bool, detail: str) -> None:
        msg = f"Param {'OK' if ok else 'FAIL'}: {name} {detail}"
        self._append_log(msg)
        if ok:
            try:
                v = float(str(detail).strip())
            except Exception:
                v = None
            if v is not None:
                key = str(name).strip().upper()
                self._last_params[key] = v
                cur = self._param_name_combo.currentText().strip().upper()
                if cur == key:
                    self._param_value_spin.setValue(v)
                self._refresh_acro_options_ui()
            self._top_vehicle_msg.setText(msg[:80])
        else:
            self._top_vehicle_msg.setText(msg[:80])

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

    def _sync_mode_options_for_vehicle(self, vehicle_type: int) -> None:
        if self._last_vehicle_type == vehicle_type:
            return
        self._last_vehicle_type = vehicle_type
        modes = modes_for_vehicle_type(vehicle_type)
        current = self._mode_combo.currentText().strip()
        self._mode_combo.blockSignals(True)
        self._mode_combo.clear()
        self._mode_combo.addItems(modes)
        if current and current in modes:
            self._mode_combo.setCurrentText(current)
        self._mode_combo.blockSignals(False)

    def _on_link_timeout(self, elapsed_s: float) -> None:
        self._watchdog.setText(f"Lost · {elapsed_s:.1f}s no MAVLink")
        self._apply_state_style(self._watchdog, "bad")
        self._set_dashboard_flight_status(
            "red",
            "Communication lost - Not Ready to Arm",
        )
        self._append_log(
            f"GCS link watchdog triggered: no messages for {elapsed_s:.1f}s"
        )
        if self._connect_attempt_active and not self._heartbeat_seen:
            self._connect_attempt_active = False
            QMessageBox.warning(
                self,
                "Connection failed",
                f"No MAVLink heartbeat received within {elapsed_s:.1f}s.\n"
                f"Check connection string, port/protocol, and vehicle/SITL state.",
            )

    def _on_timeout_changed(self, value: float) -> None:
        self._timeout_s = float(value)
        self._settings.setValue("watchdog_timeout_s", self._timeout_s)
        self._watchdog.setText(f"OK · {self._timeout_s:.1f}s")
        self._apply_state_style(self._watchdog, "ok")

    def _on_reset_telemetry(self) -> None:
        self._hb.setText("—")
        self._apply_state_style(self._hb, "na")
        self._watchdog.setText(f"OK · {self._timeout_s:.1f}s")
        self._apply_state_style(self._watchdog, "ok")
        self._compass.clear()
        self._reset_telemetry_fields()
        self._arm_not_ready_alert_shown = False
        self._arm_not_ready_since_mono = None
        self._set_dashboard_flight_status("red", "Communication lost - Not Ready to Arm")
        self._append_log("Telemetry fields reset.")

    def _on_theme_changed(self, theme_name: str) -> None:
        self._theme_name = theme_name
        self._theme_colors = self._build_theme_colors(theme_name)
        self._settings.setValue("ui_theme", theme_name)
        self._refresh_state_styles()

    def _restore_window_geometry(self) -> None:
        screen = QGuiApplication.primaryScreen()
        area = screen.availableGeometry() if screen is not None else None
        geometry = self._settings.value("window_geometry")
        if geometry is not None:
            self.restoreGeometry(geometry)
        else:
            if area is None:
                self.resize(1024, 700)
            else:
                self.resize(min(1024, area.width() - 20), min(700, area.height() - 20))
        if self.width() < 820 or self.height() < 560:
            self.resize(920, 620)

    def _fit_to_screen(self) -> None:
        screen = QGuiApplication.primaryScreen()
        if screen is None:
            return
        area = screen.availableGeometry()
        max_w = max(820, area.width() - 12)
        max_h = max(560, area.height() - 12)
        target_w = min(self.width(), max_w)
        target_h = min(self.height(), max_h)
        if target_w != self.width() or target_h != self.height():
            self.resize(target_w, target_h)

    def resizeEvent(self, event) -> None:  # noqa: N802
        super().resizeEvent(event)
        self._apply_responsive_layout(event.size().width())

    def _on_restore_defaults(self) -> None:
        if self._thread is not None and self._thread.isRunning():
            QMessageBox.information(
                self, "VGCS", "Disconnect before restoring defaults."
            )
            return
        self._conn_edit.setText("udp:127.0.0.1:14550")
        self._timeout_spin.setValue(2.0)
        self._theme_combo.setCurrentText("Default")
        self._settings.setValue("last_connection_string", "udp:127.0.0.1:14550")
        self._settings.setValue("watchdog_timeout_s", 2.0)
        self._settings.setValue("ui_theme", "Default")
        self._append_log("UI defaults restored.")

    def _on_flight_timer_tick(self) -> None:
        try:
            cc = self._camera_control_backend
            if cc is not None:
                get_status = getattr(cc, "get_gimbal_status", None)
                if callable(get_status):
                    st = get_status()
                    if st is not None and bool(getattr(st, "supported", False)):
                        yaw = getattr(st, "yaw_deg", None)
                        pitch = getattr(st, "pitch_deg", None)
                        ys = "?" if yaw is None else f"{float(yaw):.1f}"
                        ps = "?" if pitch is None else f"{float(pitch):.1f}"
                        self._map_widget.set_header_vehicle_msg(f"Gimbal Y/P: {ys}/{ps}")
        except Exception:
            pass
        if self._armed_since is None:
            self._sync_visible_map_overlay_metrics()
            return
        elapsed = int(time.monotonic() - self._armed_since)
        mm = elapsed // 60
        ss = elapsed % 60
        self._fields["flight_time"].setText(f"{mm:02d}:{ss:02d}")
        self._sync_visible_map_overlay_metrics()

    def _on_link_error(self, text: str) -> None:
        self._append_log(f"Error: {text}")
        t = str(text or "")
        if "mission_upload" in t or "Mission upload" in t or "mission upload" in t.lower():
            self._mission_upload_pending = False
        self._set_dashboard_flight_status("red", "Communication lost - Not Ready to Arm")
        if self._connect_attempt_active and not self._heartbeat_seen:
            self._connect_attempt_active = False
            QMessageBox.warning(self, "Connection failed", f"{text}")

    def _on_dev_reload(self) -> None:
        app = QApplication.instance()
        if app is None:
            return
        profile = select_font_profile()
        app.setFont(build_base_font(profile, ui_scale=1.0))
        app.setStyleSheet(gcs_stylesheet(mono_family=profile.mono_family, ui_scale=1.0))
        self._append_log("Dev reload applied (Ctrl+Shift+R).")

    def _on_thread_finished(self) -> None:
        self._mission_upload_pending = False
        self._thread = None

    def closeEvent(self, event) -> None:  # noqa: N802 — Qt API
        self._on_disconnect()
        self._settings.setValue("window_geometry", self.saveGeometry())
        super().closeEvent(event)
