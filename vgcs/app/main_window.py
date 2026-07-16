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
    QFormLayout,
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
    QRadioButton,
    QButtonGroup,
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
    QStyle,
)
from pymavlink import mavutil

from vgcs.app.gcs_style import gcs_stylesheet
from vgcs.app.window import MainWindowMixins
from vgcs.app.window.helpers import _settings_truthy
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








class MainWindow(MainWindowMixins, QMainWindow):
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
        self._last_hb_ui_key: tuple[object, ...] | None = None
        self._hb_armed = False
        self._hb_arm_ready = False
        self._hb_system_status = 0
        self._hb_mode_text = "—"
        self._prearm_block_reason = ""
        self._prearm_block_until_mono = 0.0
        self._arm_denied_reason = ""
        self._arm_denied_until_mono = 0.0
        self._arm_ready_confirmed = False
        self._hb_connected_since_mono: float | None = None
        self._last_gps_fix_type: int = 0
        self._last_gps_sats: int = 0
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
        self._plan_hover_speed_mps = 11.18
        self._last_gps_lat: float | None = None
        self._last_gps_lon: float | None = None
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
        self._param_name_combo.currentTextChanged.connect(self._on_param_name_changed)
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

        app = QGuiApplication.instance()
        if app is not None:
            app.applicationStateChanged.connect(self._on_application_state_changed)

        self._flight_timer = QTimer(self)
        self._flight_timer.setInterval(1000)
        self._flight_timer.timeout.connect(self._on_flight_timer_tick)
        self._flight_timer.start()
        self._c13_lrf_timer = QTimer(self)
        self._c13_lrf_timer.setInterval(1000)
        self._c13_lrf_timer.timeout.connect(self._refresh_c13_lrf_display)
        self._c13_lrf_timer.start()
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

















