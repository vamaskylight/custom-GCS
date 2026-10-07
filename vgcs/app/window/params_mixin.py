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
from vgcs.app.vehicle_params import READ_ON_CONNECT, editable_on_this_drone
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


# Results of these link actions are sentences for the operator (see
# vgcs/link/mission_speed.py): a jump to a waypoint, a planned speed VGCS set
# again by itself, and a mission that was changed from another ground station.
_MISSION_ACTIONS_IN_WORDS = ("mission_set_current_wp", "mission_speed", "mission")


class MainWindowParamsMixin:
    """Extracted from MainWindow — uses host state via self."""

    def _on_params_refresh(self) -> None:
        if self._thread is None or not self._thread.isRunning():
            QMessageBox.warning(self, "VGCS", "Connect vehicle before parameter fetch.")
            return
        # Both the ArduCopter 4.6 and 4.7 names (vgcs/app/vehicle_params.py).
        self._thread.queue_params_fetch(list(READ_ON_CONNECT))
        self._append_log("Param fetch queued")

    def _read_vehicle_params_on_connect(self) -> None:
        """Read the settings VGCS checks, once the drone has answered.

        The 10 km mission check compares the plan with the RTL height, the
        fence and the failsafes, and it only had what "Refresh params" had
        fetched by hand. Without that it reported them "not read yet" and
        checked nothing (found during the M17 simulator test work, 2026-10-07).
        """
        thread = getattr(self, "_thread", None)
        if thread is None or not thread.isRunning():
            return
        thread.queue_params_fetch(list(READ_ON_CONNECT))

    def _refresh_param_name_list(self) -> None:
        """Keep only the "Set param" names this drone has (4.6 and 4.7 differ)."""
        combo = self._param_name_combo
        names = editable_on_this_drone(self._last_params)
        shown = [combo.itemText(i) for i in range(combo.count())]
        if shown == names:
            return
        current = combo.currentText().strip().upper()
        combo.blockSignals(True)
        combo.clear()
        combo.addItems(names)
        if current in names:
            combo.setCurrentText(current)
        combo.blockSignals(False)
        self._on_param_name_changed()

    def _on_param_name_changed(self, _text: str = "") -> None:
        name = self._param_name_combo.currentText().strip().upper()
        self._param_value_spin.setValue(float(self._last_params.get(name, 0.0)))

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
        if action in _MISSION_ACTIONS_IN_WORDS:
            # These come as whole sentences that begin with what matters, and
            # the header shows only a beginning. The whole text is in Plan
            # Flight, under the buttons the operator used.
            self._post_gcs_notice(str(detail))
            # "mission" is about the mission itself: not the one VGCS knows any
            # more, not started, its upload failed. That stays on screen on the
            # ground, until an Upload or a Download has worked.
            self._map_widget.set_plan_mission_action_result(bool(ok), str(detail), action == "mission")
            return
        self._post_gcs_notice(msg[:80])

    def _on_geofence_result(self, ok: bool, detail: str) -> None:
        msg = f"Fence {'OK' if ok else 'FAIL'}: {detail}"
        self._append_log(msg)
        self._post_gcs_notice(msg[:80])

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
        self._refresh_param_name_list()
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
            self._post_gcs_notice(msg[:80])
        else:
            self._post_gcs_notice(msg[:80])

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
