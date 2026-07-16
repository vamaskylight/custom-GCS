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


class MainWindowFlightCommandsMixin:
    """Extracted from MainWindow — uses host state via self."""

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
            self._set_top_vehicle_msg(f"Mode cmd: {mode_name}")
        else:
            self._append_log(f"Mode change failed: {mode_name}")
            self._set_top_vehicle_msg("Mode change failed")

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
            "action": float(self._geofence_action_combo.currentData()),
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
