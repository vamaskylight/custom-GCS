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


class MainWindowLifecycleMixin:
    """Extracted from MainWindow — uses host state via self."""

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

    def _on_application_state_changed(self, state: Qt.ApplicationState) -> None:
        mw = getattr(self, "_map_widget", None)
        if mw is None:
            return
        # NOTE: Only pause the companion RTSP decode when the app is genuinely
        # hidden/suspended by the OS. ``ApplicationInactive`` on Windows fires on
        # any focus loss (clicking another window, a popup, a second monitor, or
        # even internal focus shifts while the GCS window is still fully visible).
        # Treating that as "background" froze the single-client C13 video every
        # time the operator interacted with other UI — the exact cause of "video
        # stops while tracking". Minimize/hide is handled authoritatively in
        # ``changeEvent`` (WindowStateChange) instead.
        if state == Qt.ApplicationState.ApplicationActive:
            try:
                mw.on_application_foreground()
            except Exception:
                pass
        elif state == Qt.ApplicationState.ApplicationSuspended:
            # Real OS suspend (e.g. sleep) — but if the window is still visible
            # and not minimized, keep decoding. Only pause when actually hidden.
            if self.isMinimized() or not self.isVisible():
                try:
                    mw.on_application_background()
                except Exception:
                    pass

    def changeEvent(self, event) -> None:  # noqa: N802 — Qt API
        # Minimize/restore is the *authoritative* signal for pausing the
        # single-client companion RTSP decode. We deliberately do NOT pause on
        # mere focus loss (see _on_application_state_changed) — only when the
        # window is truly minimized does the feed need to be released.
        if event.type() == QEvent.Type.WindowStateChange:
            mw = getattr(self, "_map_widget", None)
            if mw is not None:
                if self.isMinimized():
                    self._companion_decode_minimized = True
                    try:
                        mw.on_application_background()
                    except Exception:
                        pass
                elif getattr(self, "_companion_decode_minimized", False):
                    # Only resume if we were previously minimized — avoids
                    # spurious foreground refreshes on normal window-state flaps.
                    self._companion_decode_minimized = False
                    try:
                        mw.on_application_foreground()
                    except Exception:
                        pass
        super().changeEvent(event)

    def closeEvent(self, event) -> None:  # noqa: N802 — Qt API
        self._on_disconnect()
        self._settings.setValue("window_geometry", self.saveGeometry())
        super().closeEvent(event)
