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
from vgcs.mission import Waypoint, plan_signature
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

    def _request_takeoff(self, alt_m: float) -> None:
        """Arm if needed, then take off. Confirmed first, because it flies.

        Reported 2026-09-10: "this takeoff and return button might be not
        working ... After I click on the takeoff button the drone will take off
        automatically right??" It did not, and could not.

        The button sent a bare MAV_CMD_NAV_TAKEOFF and nothing else. ArduPilot
        accepts that only from an armed vehicle already in GUIDED, and their
        screenshot shows LOITER and READY TO ARM, so the aircraft rejected it
        every time. Worse, no command result was ever read back, so VGCS
        reported "Takeoff command sent" and a success either way. A rejected
        command that reports success is how an operator learns to distrust the
        screen.

        VGCS already had the sequence that works, wired to a different button
        under a different name: mode, arm, wait for the vehicle to actually
        report armed, then take off. Two takeoffs that behave differently is
        the trap, so the button now does what its label says.
        """
        if self._thread is None or not self._thread.isRunning():
            QMessageBox.warning(self, "VGCS", "Connect vehicle before takeoff command.")
            return
        from vgcs.app.flight_action_dialogs import ask_takeoff_altitude

        # The popup takes the height rather than only confirming one set on
        # another screen, which is what "one popup should come for adding
        # altitude" asked for on 2026-09-10.
        alt = ask_takeoff_altitude(self, float(alt_m))
        if alt is None:
            self._append_log("Takeoff cancelled")
            return
        # Remembered, so the dashboard control and the popup never disagree
        # about the height the aircraft was last sent to.
        try:
            self._takeoff_alt_spin.setValue(float(alt))
        except Exception:
            pass
        self._thread.queue_auto_takeoff(float(alt))
        self._append_log(f"Takeoff queued: arm + climb to {alt:.1f} m")

    def _on_takeoff(self) -> None:
        self._request_takeoff(self._takeoff_altitude_m(from_plan_rail=False))

    def _on_land(self) -> None:
        """Say what will happen, then land in LAND mode.

        Same defect as Takeoff had (2026-09-10): this sent a bare NAV_LAND,
        which the vehicle can refuse without VGCS ever knowing, since no
        COMMAND_ACK is read anywhere. queue_auto_land sets LAND mode and falls
        back to NAV_LAND, which is the path that actually brings it down.
        """
        from vgcs.app.flight_action_dialogs import confirm_land

        if self._thread is None or not self._thread.isRunning():
            QMessageBox.warning(self, "VGCS", "Connect vehicle before land command.")
            return
        if not confirm_land(self):
            self._append_log("Land cancelled")
            return
        self._thread.queue_auto_land()
        self._append_log("Land queued: LAND mode")

    def _on_auto_takeoff(self) -> None:
        # The same take-off as the map's Takeoff button, popup included. Until
        # 2026-10-07 this one armed and climbed without asking (see _request_takeoff
        # on two take-offs that behave differently).
        self._request_takeoff(float(self._takeoff_alt_spin.value()))

    def _on_auto_land(self) -> None:
        # The same landing as the map's Land button, popup included.
        self._on_land()

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
        answer = QMessageBox.question(
            self,
            "Apply failsafes",
            "This writes four failsafe settings to the drone on screen:\n\n"
            "    Ground station link lost: return home (FS_GCS_ENABLE 1)\n"
            "    RC link lost: return home (FS_THR_ENABLE 1)\n"
            "    Battery low: return home (BATT_FS_LOW_ACT 2)\n"
            "    Battery critical: land (BATT_FS_CRT_ACT 1)\n\n"
            "The battery failsafe also needs its trigger level on the drone "
            "(BATT_LOW_VOLT or BATT_LOW_MAH). Apply now?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Cancel,
        )
        if answer != QMessageBox.StandardButton.Yes:
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

    def _upload_fence_from_plan_panel(self) -> None:
        """Plan Flight, Fence tab: the circle fence. The same upload as the
        dashboard's "Upload fence", which the map-first layout never showed."""
        cfg = None
        fn = getattr(self._map_widget, "plan_fence_settings", None)
        if callable(fn):
            cfg = fn()
        if cfg:
            self._geofence_radius_spin.setValue(float(cfg["radius_m"]))
            self._geofence_alt_max_spin.setValue(float(cfg["alt_max_m"]))
            i = self._geofence_action_combo.findData(float(cfg["action"]))
            if i >= 0:
                self._geofence_action_combo.setCurrentIndex(i)
        self._on_upload_fence()

    def _disable_fence_from_plan_panel(self) -> None:
        if self._thread is None or not self._thread.isRunning():
            QMessageBox.warning(self, "VGCS", "Connect vehicle before switching the fence off.")
            return
        answer = QMessageBox.question(
            self,
            "Switch fence off",
            "Switch the geofence off on the drone on screen?\n\nIt can then fly anywhere.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Cancel,
        )
        if answer != QMessageBox.StandardButton.Yes:
            return
        self._thread.queue_geofence_upload({"disable": True})
        self._append_log("Fence off queued")

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

    def _on_disarm(self) -> None:
        """Stop the motors after a normal landing.

        Requested 2026-09-11: "please add disarm button also it will helpful to
        us." Until now the only way to stop the motors from VGCS was EMERGENCY
        STOP, which forces the disarm past every check and is meant for a
        runaway, not for the end of a flight.

        This sends the plain command with no force override, so ArduPilot's own
        rule applies: it refuses a GCS disarm while it believes it is flying.
        The aircraft, not this button, is what keeps a spinning rotor spinning.
        """
        from vgcs.app.flight_action_dialogs import confirm_disarm

        if self._thread is None or not self._thread.isRunning():
            QMessageBox.warning(self, "VGCS", "Connect vehicle before disarm command.")
            return
        if not confirm_disarm(self):
            self._append_log("Disarm cancelled")
            return
        self._thread.queue_arm(False)
        self._append_log("Disarm queued")

    def _on_map_return_requested(self) -> None:
        """Say what will happen, then RTL.

        RTL was always the right command. What it lacked was the popup saying
        the aircraft is about to climb to its return altitude and fly home,
        which is not obvious from a button labelled Return.
        """
        from vgcs.app.flight_action_dialogs import confirm_return

        if self._thread is None or not self._thread.isRunning():
            QMessageBox.warning(self, "VGCS", "Connect vehicle before return command.")
            return
        if not confirm_return(self):
            self._append_log("Return cancelled")
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
        # Start Mission uploads the plan on the map first. Where the mission on
        # the drone has more in it than the plan can hold (camera, servo or
        # jump commands from another ground station), that would be their end.
        as_it_is = False
        not_kept = self._mission_not_kept_now()
        if not_kept:
            choice = self._ask_how_to_start(not_kept, self._plan_on_map_is_mission_on_drone())
            if choice not in ("as_is", "upload"):
                self._append_log("Mission start cancelled: the mission on the drone has more in it than the plan.")
                return
            as_it_is = choice == "as_is"
        armed_text = self._fields.get("armed").text().strip().lower()
        if armed_text != "yes":
            QMessageBox.information(
                self,
                "Mission Start",
                "Vehicle is not armed.\nThe link will switch to an armable mode, arm, then run AUTO + mission start.",
            )
        if as_it_is:
            # The link reads the mission again first, and starts nothing when
            # it is no longer the one the Download showed.
            self._thread.queue_mission_start(as_it_is=True)
            self._append_log("Mission start queued: the mission as it is on the drone, nothing uploaded")
            return
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

    def _start_choice_box(self, not_kept, can_start_as_is: bool):
        """The question Start Mission asks when the drone's mission has more than the plan.

        Returns the box and what each of its buttons means. Made apart from
        showing it, so that what it offers can be checked without a screen.
        """
        box = QMessageBox(self)
        box.setIcon(QMessageBox.Icon.Warning)
        box.setWindowTitle("Mission Start")
        text = (
            "The mission on the drone has more than this plan can hold:\n\n"
            f"{self._not_kept_bullets(not_kept)}\n\n"
        )
        answers = {}
        if can_start_as_is:
            text += (
                "Start Mission uploads the plan on the map first. That would replace the mission "
                "on the drone, and these would be gone or changed."
            )
            answers[box.addButton("Start it as it is on the drone", QMessageBox.ButtonRole.AcceptRole)] = "as_is"
        else:
            text += (
                "The plan on the map was changed after the Download, so it has to be uploaded to fly it. "
                "That replaces the mission on the drone, and these are gone or changed."
            )
        answers[box.addButton("Upload this plan and start", QMessageBox.ButtonRole.DestructiveRole)] = "upload"
        cancel = box.addButton("Cancel", QMessageBox.ButtonRole.RejectRole)
        answers[cancel] = "cancel"
        box.setDefaultButton(cancel)
        box.setEscapeButton(cancel)
        box.setText(text)
        return box, answers

    def _ask_how_to_start(self, not_kept, can_start_as_is: bool) -> str:
        """ "as_is", "upload" or "cancel"."""
        box, answers = self._start_choice_box(not_kept, can_start_as_is)
        try:
            box.exec()
            return answers.get(box.clickedButton(), "cancel")
        finally:
            box.deleteLater()

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

    def _on_map_mission_jump_requested(self, wp_index: int) -> None:
        """"Fly to WP" in Plan Flight: send the running mission to another waypoint.

        A waypoint is named by its number, and the number means the same on
        the map and on the drone only while the plan on the map is the mission
        the drone holds. So that is checked first, and the question says where
        the drone will go, how high and how fast.
        """
        title = "Fly to waypoint"
        if self._thread is None or not self._thread.isRunning():
            QMessageBox.warning(self, title, "Connect vehicle before sending it to a waypoint.")
            return
        if not bool(getattr(self, "_hb_armed", False)):
            QMessageBox.information(
                self,
                title,
                "The drone is not flying a mission.\n\nUse Start Mission to fly the plan from its take-off.",
            )
            return
        plan = list(self._map_widget._plan_waypoints_snapshot())
        index = int(wp_index)
        if not (0 <= index < len(plan)):
            return
        number = index + 1
        on_drone = self._thread.mission_on_drone()
        if on_drone is None:
            QMessageBox.warning(
                self,
                title,
                "VGCS does not know the mission this drone holds.\n\n"
                "Press Download (Plan Flight, File) to read it, then try again.",
            )
            return
        if tuple(on_drone) != plan_signature(plan):
            QMessageBox.warning(
                self,
                title,
                "The plan on the map is not the mission the drone holds.\n\n"
                f"It was changed after the last Upload or Download, so WP {number} here "
                f"may not be WP {number} on the drone.\n\n"
                "Upload the plan, or press Download, then try again.",
            )
            return
        wp = plan[index]
        where = f"{float(wp.alt_m):.0f} m and {float(getattr(wp, 'speed_mps', 5.0)):.1f} m/s"
        mode = str(getattr(self, "_hb_mode_text", "") or "").strip()
        if mode == "AUTO":
            text = (
                f"Send the drone to WP {number} now?\n\n"
                f"It flies there in a straight line from where it is, at {where}. "
                "The waypoints in between are skipped, and the mission goes on from there."
            )
        else:
            text = (
                f"Make WP {number} the next waypoint?\n\n"
                f"The drone is in {mode or 'another mode'}, not in AUTO, and stays where it is. "
                f"It flies to WP {number} ({where}) when you press Resume."
            )
        if any(bool(getattr(p, "drop_payload", False)) for p in plan):
            # Measured in the simulator: a jump while the release servo is
            # held open drops the rest of that release, and the servo stays
            # open for the rest of the flight.
            text += (
                "\n\nThis plan releases a payload. Do not do this in the seconds after a release: "
                "the release servo would stay open."
            )
        answer = QMessageBox.question(
            self,
            title,
            text,
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if answer != QMessageBox.StandardButton.Yes:
            return
        self._thread.queue_mission_set_current_wp(index)
        self._append_log(f"Mission jump requested: WP {number}")
        self._post_gcs_notice(f"Sending the drone to WP {number}…")
        self._map_widget.set_plan_mission_action_result(True, f"Sending the drone to WP {number}…")
