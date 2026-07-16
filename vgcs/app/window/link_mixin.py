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


class MainWindowLinkMixin:
    """Extracted from MainWindow — uses host state via self."""

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
        if hasattr(self, "_hdr_connect_btn"):
            self._hdr_connect_btn.setEnabled(False)
        if hasattr(self, "_hdr_disconnect_btn"):
            self._hdr_disconnect_btn.setEnabled(False)
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
        self._arm_ready_confirmed = False
        self._hb_connected_since_mono = None
        self._status.setText("Connecting…")
        self._apply_state_style(self._status, "warn")
        self._set_dashboard_flight_status("yellow", "Connecting to vehicle...")
        self._thread.start()

    def _on_disconnect(self) -> None:
        if hasattr(self, "_hdr_disconnect_btn"):
            self._hdr_disconnect_btn.setEnabled(False)
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
                self._append_log("Video settings applied.")
            except Exception:
                pass
            # Skydroid probe / socket setup off the tail of RTSP teardown (still on GUI thread but
            # after the dialog and decode restart have yielded).
            QTimer.singleShot(80, self._set_runtime_camera_control)

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
        if hasattr(self, "_hdr_connect_btn"):
            self._hdr_connect_btn.setEnabled(False)
        if hasattr(self, "_hdr_disconnect_btn"):
            self._hdr_disconnect_btn.setEnabled(True)
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
        self._arm_ready_confirmed = False
        self._hb_connected_since_mono = None
        self._last_hb_ui_key = None
        self._last_synced_mode_text = None
        self._hb_armed = False
        self._hb_arm_ready = False
        self._hb_system_status = 0
        self._hb_mode_text = "—"
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
        if hasattr(self, "_hdr_connect_btn"):
            self._hdr_connect_btn.setEnabled(True)
        if hasattr(self, "_hdr_disconnect_btn"):
            self._hdr_disconnect_btn.setEnabled(False)
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

    def _on_link_error(self, text: str) -> None:
        self._append_log(f"Error: {text}")
        t = str(text or "")
        if "mission_upload" in t or "Mission upload" in t or "mission upload" in t.lower():
            self._mission_upload_pending = False
        self._set_dashboard_flight_status("red", "Communication lost - Not Ready to Arm")
        if self._connect_attempt_active and not self._heartbeat_seen:
            self._connect_attempt_active = False
            QMessageBox.warning(self, "Connection failed", f"{text}")
