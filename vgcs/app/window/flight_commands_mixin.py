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


# Low and brief: a pre-flight test only has to show each motor turns and which
# way. The link thread clamps these again, so a mistake here cannot produce
# thrust. See MavlinkThread._motor_test.
MOTOR_TEST_THROTTLE_PCT = 8.0
MOTOR_TEST_DURATION_S = 2.0
MOTOR_TEST_GAP_S = 1.0

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
            self._post_gcs_notice(f"Mode cmd: {mode_name}")
        else:
            self._append_log(f"Mode change failed: {mode_name}")
            self._post_gcs_notice("Mode change failed")

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

    def run_motor_test(self, *, motors: int = 0) -> float:
        """Spin each motor in turn, one at a time.

        Requested 2026-09-02: "when it comes motor/ESC check then motor should
        rotate accordingly". The checklist row before this read a SYS_STATUS
        health bit, which cannot tell a reversed motor from a correct one.

        Sequential on purpose. The point is to see WHICH motor turns, in what
        order and which direction; spinning them together proves only that
        something moved. Motors are numbered in board output order, so motor 1
        is the output labelled 1 on the autopilot, not a position on the frame.

        Reached from the pre-flight dialog behind an explicit confirmation. The
        armed-state refusal lives in the link thread, which is the one place
        that knows the live armed state.

        Returns how many seconds the sequence will take, so a caller can show
        it as running; 0.0 when nothing was started.
        """
        thread = getattr(self, "_thread", None)
        if thread is None or not thread.isRunning():
            self._append_log("Motor test: connect the vehicle first")
            return 0.0
        self.stop_motor_test(quiet=True)   # never overlap two sequences

        count = int(motors) if int(motors) > 0 else self._motor_test_count()
        throttle = MOTOR_TEST_THROTTLE_PCT
        each_s = MOTOR_TEST_DURATION_S
        self._append_log(
            f"Motor test: {count} motors, {throttle:.0f}% for {each_s:.0f}s each, "
            "in board output order - PROPELLERS SHOULD BE OFF"
        )
        self._motor_test_timers = []
        for i in range(count):
            motor = i + 1
            # Spaced by the test duration plus a gap, so the operator can tell
            # one motor from the next instead of watching them blur together.
            delay_ms = int(i * (each_s + MOTOR_TEST_GAP_S) * 1000)
            timer = QTimer(self)
            timer.setSingleShot(True)
            timer.setInterval(delay_ms)
            timer.timeout.connect(
                lambda m=motor, t=thread, first=(i == 0), last=(i == count - 1):
                self._spin_one_motor(t, m, throttle, each_s, first=first, last=last)
            )
            self._motor_test_timers.append(timer)
            timer.start()
        # How long the caller should treat the sequence as running.
        return count * each_s + max(0, count - 1) * MOTOR_TEST_GAP_S

    def _spin_one_motor(
        self,
        thread,
        motor: int,
        throttle: float,
        seconds: float,
        *,
        first: bool = True,
        last: bool = True,
    ) -> None:
        """Command one motor. ``first`` is what tells the link thread this is a
        fresh run and the armed state is the operator's, not our own."""
        self._append_log(f"Motor test: motor {motor}")
        try:
            thread.queue_motor_test(
                motor=motor,
                throttle_pct=throttle,
                duration_s=seconds,
                sequence_start=first,
            )
        except Exception as e:
            self._append_log(f"Motor test: motor {motor} failed ({e})")
        if last:
            # The run is over. Forget the timers so stop_motor_test() knows
            # there is nothing to halt -- otherwise the next run opens by
            # sending a stop command nobody asked for.
            self._motor_test_timers = []

    def stop_motor_test(self, *, quiet: bool = False) -> None:
        """Abort the sequence.

        Cancelling the pending timers is the part that matters: if the operator
        sees something wrong on motor 1 they must be able to stop the remaining
        motors before they spin. The zero-throttle command cuts the one already
        turning; ArduPilot's motor test honours a new command for the same
        output. A motor still running after this is what the emergency stop is
        for.
        """
        timers = getattr(self, "_motor_test_timers", None) or []
        self._motor_test_timers = []
        if not timers:
            # Nothing was running. Say nothing and, in particular, send nothing:
            # run_motor_test() calls this first, and a stray command on every
            # start would be a motor command the operator never asked for.
            return
        pending = 0
        for timer in timers:
            try:
                if timer.isActive():
                    pending += 1
                timer.stop()
            except Exception:
                pass
        thread = getattr(self, "_thread", None)
        if thread is not None and thread.isRunning():
            try:
                # sequence_start=False: this belongs to the run being aborted,
                # so it must not be judged on an armed flag that the run itself
                # caused, or the abort would be refused exactly when it matters.
                thread.queue_motor_test(
                    motor=1, throttle_pct=0.0, duration_s=0.5, sequence_start=False
                )
            except Exception:
                pass
        if not quiet:
            self._append_log(f"Motor test stopped ({pending} motors not run)")

    def _motor_test_count(self) -> int:
        """Motor count from FRAME_CLASS when the parameter has been read.

        Falls back to a quadrotor's four rather than refusing: testing four
        outputs on a hexacopter still turns four real motors and tells the
        operator something, and an over-count simply fails on outputs that are
        not there.
        """
        try:
            frame = int(float(self._last_params.get("FRAME_CLASS", 0) or 0))
        except Exception:
            frame = 0
        # ArduCopter FRAME_CLASS -> motor outputs.
        return {
            1: 4,    # Quad
            2: 6,    # Hexa
            3: 8,    # Octa
            4: 8,    # OctaQuad
            5: 6,    # Y6
            7: 3,    # Tri
            10: 2,   # BiCopter
            12: 12,  # DodecaHexa
            14: 10,  # Deca
        }.get(frame, 4)

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
        if not self._confirm_mission_plan_is_sane(model, title="Mission Start"):
            return
        if not self._confirm_vehicle_ready_for_auto():
            return
        armed_text = self._fields.get("armed").text().strip().lower()
        if armed_text != "yes":
            QMessageBox.information(
                self,
                "Mission Start",
                "Vehicle is not armed.\nThe link will switch to an armable mode, arm, then run AUTO + mission start.",
            )
        # Reuses the upload payload builder so per-waypoint speed is not dropped here —
        # Start Mission used to upload every waypoint at the 5 m/s default.
        payload = self._mission_payload_from_waypoints(model)
        end_action = self._map_widget.get_mission_end_action()
        self._mission_upload_pending = True
        self._thread.queue_mission_upload(payload, end_action)
        self._thread.queue_mission_start()
        self._append_log(
            f"Mission start queued: upload {len(payload)} WPs (+TAKEOFF, end={end_action}) + AUTO start"
        )

    def _confirm_vehicle_ready_for_auto(self) -> bool:
        """AUTO needs a position solution. Warn on a weak one rather than blocking."""
        problems: list[str] = []
        if self._map_widget.get_vehicle_position() is None:
            problems.append("No vehicle GPS position has been received yet.")
        fix_type = int(getattr(self, "_last_gps_fix_type", 0) or 0)
        sats = int(getattr(self, "_last_gps_sats", 0) or 0)
        # GPS_FIX_TYPE_3D_FIX == 3; anything below that cannot hold a waypoint.
        if fix_type < 3:
            problems.append(f"GPS fix type {fix_type} — AUTO needs a 3D fix (3 or better).")
        if 0 < sats < 6:
            problems.append(f"Only {sats} satellites — marginal for autonomous navigation.")
        if not self._heartbeat_seen:
            problems.append("No MAVLink heartbeat has been seen on this link.")
        if not problems:
            return True
        detail = "\n".join(f"• {p}" for p in problems)
        for p in problems:
            self._append_log(f"Mission start check: {p}")
        answer = QMessageBox.question(
            self,
            "Mission Start",
            f"The vehicle may not be ready for AUTO:\n\n{detail}\n\nStart the mission anyway?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        return answer == QMessageBox.StandardButton.Yes

    def _on_map_mission_pause_requested(self) -> None:
        if self._thread is None or not self._thread.isRunning():
            QMessageBox.warning(self, "VGCS", "Connect vehicle before pausing the mission.")
            return
        self._thread.queue_mission_pause()
        self._append_log("Mission pause queued (BRAKE/LOITER hold)")
        self._post_gcs_notice("Pausing mission…")

    def _on_map_mission_resume_requested(self) -> None:
        if self._thread is None or not self._thread.isRunning():
            QMessageBox.warning(self, "VGCS", "Connect vehicle before resuming the mission.")
            return
        self._thread.queue_mission_resume()
        self._append_log("Mission resume queued (AUTO from current item)")
        self._post_gcs_notice("Resuming mission…")
