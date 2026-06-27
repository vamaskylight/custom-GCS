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


class MainWindowSettingsDialogsMixin:
    """Extracted from MainWindow — uses host state via self."""

    def _show_application_settings_dialog(self) -> None:
        dlg = QDialog(self)
        dlg.setWindowTitle("Application Settings")
        dlg.setModal(True)
        dlg.setMinimumSize(880, 580)
        dlg.resize(920, 640)
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
                QDialog#appSettingsDialog QGroupBox {
                    font-weight: 600;
                    border: 1px solid #2a3344;
                    border-radius: 8px;
                    margin-top: 14px;
                    padding: 14px 12px 12px 12px;
                }
                QDialog#appSettingsDialog QGroupBox::title {
                    subcontrol-origin: margin;
                    left: 10px;
                    padding: 0 6px;
                    color: #dbe1ee;
                }
                QDialog#appSettingsDialog QScrollArea {
                    background: transparent;
                    border: none;
                }
                QDialog#appSettingsDialog QLineEdit,
                QDialog#appSettingsDialog QComboBox,
                QDialog#appSettingsDialog QSpinBox,
                QDialog#appSettingsDialog QDoubleSpinBox {
                    background-color: #12151c;
                    border: 1px solid #252d3d;
                    border-radius: 6px;
                    padding: 6px 8px;
                    min-height: 28px;
                }
                QDialog#appSettingsDialog QRadioButton,
                QDialog#appSettingsDialog QCheckBox {
                    spacing: 8px;
                }
                """
            )
        except Exception:
            pass
        outer = QVBoxLayout(dlg)
        outer.setContentsMargins(12, 12, 12, 12)
        outer.setSpacing(10)
        content_row = QHBoxLayout()
        content_row.setSpacing(12)

        nav = QListWidget()
        nav.setFixedWidth(190)
        nav.setSpacing(2)
        nav.addItem(QListWidgetItem("General"))
        nav.addItem(QListWidgetItem("Video"))
        nav.addItem(QListWidgetItem("Observation"))
        content_row.addWidget(nav, 0)

        stack = QStackedWidget()
        stack.setMinimumHeight(420)
        content_row.addWidget(stack, 1)
        outer.addLayout(content_row, 1)

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

        def _settings_hint(text: str) -> QLabel:
            hint = QLabel(text)
            hint.setWordWrap(True)
            hint.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
            hint.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
            hint.setStyleSheet("color: #aab4c8; font-size: 11px;")
            return hint

        class _ScrollBodyWidthFilter(QObject):
            def __init__(self, scroll: QScrollArea, body: QWidget) -> None:
                super().__init__(body)
                self._scroll = scroll
                self._body = body

            def eventFilter(self, obj, event) -> bool:  # noqa: N802
                if (
                    obj is self._scroll.viewport()
                    and event.type() == QEvent.Type.Resize
                ):
                    width = self._scroll.viewport().width()
                    if width > 0:
                        self._body.setMinimumWidth(width)
                return False

        # Put settings in a scroll area so Apply/Close remain visible on small/high-DPI screens.
        video_scroll = QScrollArea()
        video_scroll.setWidgetResizable(True)
        video_scroll.setFrameShape(QFrame.Shape.NoFrame)
        video_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        v.addWidget(video_scroll, 1)

        video_body = QWidget()
        video_scroll.setWidget(video_body)
        width_filter = _ScrollBodyWidthFilter(video_scroll, video_body)
        video_scroll.viewport().installEventFilter(width_filter)
        QTimer.singleShot(
            0,
            lambda: video_body.setMinimumWidth(max(1, video_scroll.viewport().width())),
        )
        vb = QVBoxLayout(video_body)
        vb.setContentsMargins(0, 0, 4, 20)
        vb.setSpacing(12)

        enabled = QCheckBox("Enable video streaming")
        vb.addWidget(enabled)
        vb.addWidget(
            _settings_hint(
                "When this is unchecked, stream URLs are saved but VGCS does not open or decode "
                "video — the map preview stays black. Check the box to actually connect to RTSP/UDP."
            )
        )

        source_group = QGroupBox("Video Source")
        sg = QVBoxLayout()
        sg.setSpacing(8)
        source_row = QHBoxLayout()
        source_row.setSpacing(10)
        source_row.addWidget(QLabel("Source"))
        source_combo = QComboBox()
        source_combo.addItem("Video Stream Disabled", "disabled")
        source_combo.addItem("RTSP Video Stream", "rtsp")
        source_combo.addItem("UDP h.264 stream", "udp_h264")
        source_combo.addItem("UDP h.265 / HEVC stream", "udp_h265")
        source_row.addWidget(source_combo, 1)
        sg.addLayout(source_row)
        sg.addWidget(
            _settings_hint(
                "Examples:\n"
                "• RTSP — rtsp://192.168.x.x/stream (works on local radio/Wi‑Fi without internet).\n"
                "• UDP — udp://0.0.0.0:5600; pick h.264/h.265 for raw Annex B, or RTSP mode for "
                "MPEG‑TS UDP (auto-detect)."
            )
        )
        source_group.setLayout(sg)
        vb.addWidget(source_group)

        conn_group = QGroupBox("Connection")
        conn_form = QFormLayout()
        conn_form.setLabelAlignment(
            Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter
        )
        conn_form.setHorizontalSpacing(12)
        conn_form.setVerticalSpacing(10)
        conn_form.setFieldGrowthPolicy(QFormLayout.FieldGrowthPolicy.ExpandingFieldsGrow)
        rtsp_day = QLineEdit()
        rtsp_day.setPlaceholderText(
            "ZR10: rtsp://192.168.144.25:8554/main.264  (video2 not on all firmware)"
        )
        rtsp_th = QLineEdit()
        conn_form.addRow("Stream URL 1 (Day / primary)", rtsp_day)
        conn_form.addRow("Stream URL 2 (Thermal / secondary)", rtsp_th)
        conn_group.setLayout(conn_form)
        vb.addWidget(conn_group)

        camera_group = QGroupBox("Camera / gimbal control")
        cam_outer = QVBoxLayout()
        cam_outer.setSpacing(10)
        provider_group = QButtonGroup(camera_group)
        rb_mavlink = QRadioButton("MAVLink (ArduPilot mount)")
        rb_mavlink.setProperty("provider_id", "mavlink")
        rb_siyi = QRadioButton("ZR Gimbal Camera (UDP)")
        rb_siyi.setProperty("provider_id", "siyi")
        rb_skydroid = QRadioButton("C13 Gimbal Camera (UDP)")
        rb_skydroid.setProperty("provider_id", "skydroid")
        provider_wrap = QWidget()
        provider_wrap.setObjectName("cameraProviderWrap")
        provider_lay = QVBoxLayout(provider_wrap)
        provider_lay.setContentsMargins(10, 8, 10, 8)
        provider_lay.setSpacing(6)
        provider_title = QLabel("Control provider")
        provider_title.setStyleSheet("font-weight: 600; color: #dbe1ee;")
        provider_lay.addWidget(provider_title)
        for rb in (rb_mavlink, rb_siyi, rb_skydroid):
            provider_group.addButton(rb)
            provider_lay.addWidget(rb)
        provider_wrap.setStyleSheet(
            "QWidget#cameraProviderWrap {"
            "background-color: #12151c; border: 1px solid #252d3d; border-radius: 8px;"
            "}"
        )
        cam_outer.addWidget(provider_wrap)

        camera_hint = _settings_hint("")
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
        skydroid_gimbal_speed_yaw = QDoubleSpinBox()
        skydroid_gimbal_speed_yaw.setRange(0.5, 9.9)
        skydroid_gimbal_speed_yaw.setDecimals(1)
        skydroid_gimbal_speed_yaw.setSingleStep(0.5)
        skydroid_gimbal_speed_yaw.setSuffix(" deg/s")
        skydroid_gimbal_speed_yaw.setToolTip(
            "Hold ←/→ or ↑/↓: continuous slew (TOP GSY/GSP). Default 5 deg/s — lower for fine aim, higher for fast pan."
        )
        skydroid_grid.addWidget(QLabel("Hold slew rate"), 4, 0)
        skydroid_grid.addWidget(skydroid_gimbal_speed_yaw, 4, 1)
        skydroid_gimbal_tap_deg = None
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
                "ZR-series gimbal SDK (ZR10, ZT6, A8 mini): UDP port 37260 on the camera IP — usually the same host as your RTSP URL."
            ),
            "skydroid": (
                "C13 gimbal TOP (PROTOCAL): UDP 192.168.144.108 port 5000 (#TP frames). "
                "RTSP: rtsp://192.168.144.108:554/stream=1. PC Ethernet 192.168.144.10/24."
            ),
        }
        _CAMERA_STACK_INDEX = {"mavlink": 0, "siyi": 1, "skydroid": 2}
        _RTSP_PLACEHOLDER = {
            "mavlink": "rtsp://host/stream or udp://0.0.0.0:5600",
            "siyi": "ZR10: rtsp://192.168.144.25:8554/main.264",
            "skydroid": "C13: rtsp://192.168.144.108:554/stream=1",
        }

        def _camera_provider_id() -> str:
            for rb in (rb_mavlink, rb_siyi, rb_skydroid):
                if rb.isChecked():
                    return str(rb.property("provider_id") or "mavlink")
            return "mavlink"

        def _set_camera_provider_id(pid: str) -> None:
            want = str(pid or "mavlink").strip().lower()
            for rb in (rb_mavlink, rb_siyi, rb_skydroid):
                rb.setChecked(str(rb.property("provider_id") or "") == want)

        def _sync_camera_provider_ui() -> None:
            pid = _camera_provider_id()
            camera_stack.setCurrentIndex(_CAMERA_STACK_INDEX.get(pid, 0))
            camera_hint.setText(_CAMERA_PROVIDER_HINTS.get(pid, ""))
            rtsp_day.setPlaceholderText(_RTSP_PLACEHOLDER.get(pid, _RTSP_PLACEHOLDER["mavlink"]))

        provider_group.buttonClicked.connect(lambda _btn: _sync_camera_provider_ui())
        _sync_camera_provider_ui()

        def _maybe_enable_video_from_urls() -> None:
            if str(rtsp_day.text()).strip() or str(rtsp_th.text()).strip():
                enabled.setChecked(True)

        rtsp_day.textChanged.connect(lambda _t: _maybe_enable_video_from_urls())
        rtsp_th.textChanged.connect(lambda _t: _maybe_enable_video_from_urls())

        camera_group.setLayout(cam_outer)
        vb.addWidget(camera_group)

        settings_group = QGroupBox("Settings")
        stg = QGridLayout()
        stg.setHorizontalSpacing(12)
        stg.setVerticalSpacing(10)
        stg.setColumnStretch(1, 1)
        stg.setColumnMinimumWidth(0, 160)
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
        lg.setHorizontalSpacing(12)
        lg.setVerticalSpacing(10)
        lg.setColumnStretch(1, 1)
        lg.setColumnMinimumWidth(0, 160)
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
        stack.addWidget(video)

        observe = QWidget()
        ob = QVBoxLayout(observe)
        ob.setContentsMargins(12, 12, 12, 12)
        ob.setSpacing(10)
        observe_scroll = QScrollArea()
        observe_scroll.setWidgetResizable(True)
        observe_scroll.setFrameShape(QFrame.Shape.NoFrame)
        ob.addWidget(observe_scroll, 1)
        observe_body = QWidget()
        observe_scroll.setWidget(observe_body)
        ob_l = QVBoxLayout(observe_body)
        ob_l.setContentsMargins(0, 0, 0, 0)
        ob_l.setSpacing(10)

        dem_group = QGroupBox("Digital elevation (M8 geo)")
        dg = QGridLayout()
        dem_enabled = QCheckBox("Use DEM terrain intersection (hilly ground)")
        dem_enabled.setChecked(True)
        dg.addWidget(dem_enabled, 0, 0, 1, 2)
        dg.addWidget(QLabel("DEM file"), 1, 0)
        dem_path_edit = QLineEdit()
        dem_path_edit.setPlaceholderText("CSV, ESRI .asc, or GeoTIFF (.tif) — export from QGIS/SRTM")
        dg.addWidget(dem_path_edit, 1, 1)
        dem_browse = QPushButton("Browse…")
        dg.addWidget(dem_browse, 2, 1, Qt.AlignmentFlag.AlignLeft)
        dem_hint = QLabel(
            "Improves video→map HIT on slopes. CSV columns: lat, lon, elev_m. "
            "GeoTIFF needs pip install rasterio. Without DEM, flat-ground + drone height is used."
        )
        dem_hint.setWordWrap(True)
        dem_hint.setStyleSheet("color: #aab4c8; font-size: 11px;")
        dg.addWidget(dem_hint, 3, 0, 1, 2)
        dem_group.setLayout(dg)
        ob_l.addWidget(dem_group)

        cam_group = QGroupBox("Camera geo")
        cg_ob = QGridLayout()
        cg_ob.addWidget(QLabel("Horizontal FOV (°)"), 0, 0)
        observe_hfov = QDoubleSpinBox()
        observe_hfov.setRange(20.0, 120.0)
        observe_hfov.setValue(62.0)
        observe_hfov.setDecimals(1)
        cg_ob.addWidget(observe_hfov, 0, 1)
        cam_group.setLayout(cg_ob)
        ob_l.addWidget(cam_group)
        ob_l.addStretch(1)
        stack.addWidget(observe)

        btn_row = QHBoxLayout()
        btn_apply = QPushButton("Apply")
        btn_close = QPushButton("Close")
        btn_row.addStretch(1)
        btn_row.addWidget(btn_apply)
        btn_row.addWidget(btn_close)
        outer.addLayout(btn_row)

        def _browse_dem_file() -> None:
            path, _ = QFileDialog.getOpenFileName(
                dlg,
                "Select DEM file",
                str(dem_path_edit.text() or "").strip() or str(Path.home()),
                "Elevation (*.csv *.asc *.tif *.tiff);;All files (*.*)",
            )
            if path:
                dem_path_edit.setText(str(path))

        dem_browse.clicked.connect(_browse_dem_file)

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
        _set_camera_provider_id(str(s.value("camera/provider", "mavlink") or "mavlink"))
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
        try:
            skydroid_gimbal_speed_yaw.setValue(
                float(s.value("camera/skydroid_gimbal_speed_yaw", 5.0) or 5.0)
            )
        except Exception:
            skydroid_gimbal_speed_yaw.setValue(5.0)
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
        dem_path_edit.setText(
            str(
                s.value("observe/dem_path", "") or s.value("observe/dem_csv", "") or ""
            ).strip()
        )
        dem_enabled.setChecked(
            _settings_truthy(s.value("observe/dem_terrain_enabled", True), default=True)
        )
        try:
            observe_hfov.setValue(
                float(s.value("observe/camera_hfov_deg", 62.0) or 62.0)
            )
        except Exception:
            observe_hfov.setValue(62.0)

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
                _th_u = str(rtsp_th.text()).strip()
                s.setValue("video/rtsp_thermal", _th_u)
                if _day_rtsp and _th_u:
                    split_default.setCurrentText("Split")
                s.setValue("video/default_view", str(split_default.currentText()))
                s.setValue("video/aspect", str(aspect.currentText()))
                s.setValue("video/low_latency", bool(low_latency.isChecked()))
                s.setValue("video/decode_priority", str(decode_prio.currentText()))
                s.setValue("video/rtsp_transport", str(rtsp_transport.currentData() or "auto"))
                s.setValue("video/record_format", str(record_fmt.currentText()))
                s.setValue("video/auto_delete", bool(auto_del.isChecked()))
                s.setValue("video/max_storage_mb", int(max_mb.value()))
                s.setValue("camera/provider", _camera_provider_id())
                s.setValue("camera/siyi_host", str(siyi_host.text()).strip())
                s.setValue("camera/siyi_port", int(siyi_port.value()))
                s.setValue("camera/siyi_timeout_ms", int(siyi_timeout.value()))
                s.setValue("camera/skydroid_host", str(skydroid_host.text()).strip())
                sk_port = int(skydroid_port.value())
                if str(skydroid_profile.currentData() or "c13_default").startswith("c13") and sk_port != 5000:
                    try:
                        print(
                            f"[VGCS:skydroid] settings: TOP port {sk_port} corrected to "
                            "5000 (PROTOCAL.doc C13 gimbal UDP)"
                        )
                    except Exception:
                        pass
                    sk_port = 5000
                    skydroid_port.setValue(5000)
                s.setValue("camera/skydroid_port", sk_port)
                s.setValue("camera/skydroid_timeout_ms", int(skydroid_timeout.value()))
                s.setValue("camera/skydroid_profile", str(skydroid_profile.currentData() or "c13_default"))
                s.setValue("camera/skydroid_gimbal_speed_yaw", float(skydroid_gimbal_speed_yaw.value()))
                s.setValue("camera/skydroid_gimbal_speed_pitch", float(skydroid_gimbal_speed_yaw.value()))
                dem_p = str(dem_path_edit.text()).strip()
                s.setValue("observe/dem_path", dem_p)
                s.setValue("observe/dem_csv", dem_p)
                s.setValue("observe/dem_terrain_enabled", bool(dem_enabled.isChecked()))
                s.setValue("observe/camera_hfov_deg", float(observe_hfov.value()))
                try:
                    from vgcs.observe.dem import clear_dem_cache

                    clear_dem_cache()
                except Exception:
                    pass
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

        overlay_restore: dict[str, bool] = {}
        try:
            if self._map_widget is not None:
                overlay_restore = self._map_widget.suppress_floating_overlays()
            dlg.raise_()
            dlg.activateWindow()
            dlg.exec()
        finally:
            if self._map_widget is not None:
                self._map_widget.restore_floating_overlays(overlay_restore)

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
        cal_btn_row1.addWidget(btn_cal_accel)
        cal_btn_row1.addWidget(btn_cal_compass)
        cal_lay.addLayout(cal_btn_row1)

        cal_btn_row2 = QHBoxLayout()
        btn_cal_rc = QPushButton("RC")
        cal_btn_row2.addWidget(btn_cal_rc)
        cal_btn_row2.addStretch()
        cal_lay.addLayout(cal_btn_row2)

        for b in (btn_cal_accel, btn_cal_compass, btn_cal_rc):
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
