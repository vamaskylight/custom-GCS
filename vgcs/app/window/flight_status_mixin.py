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
from vgcs.app.vehicle_messages import SEVERITY_INFO
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


class MainWindowFlightStatusMixin:
    """Extracted from MainWindow — uses host state via self."""

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

        # The banner sentence goes to the banner only. It used to be stamped
        # into the MESSAGE cell as well, and because this runs off every
        # position packet (~20 Hz) it overwrote real vehicle STATUSTEXT within
        # a frame — the "vehicle msg is not shown" report. The cell now has one
        # owner; refresh it so an ageing message keeps its age stamp current.
        self._publish_vehicle_msg_cell()
        if state_norm == "green":
            self._flight_status_btn.setText("ARMED" if self._armed_since is not None else "READY TO ARM")
            self._flight_status_btn.setToolTip(
                "Flight mode and arm readiness are separate. "
                f"Current mode: {self._hb_mode_text or '—'}"
            )
            self._map_widget.set_flight_status("green", message)
            return
        if state_norm == "yellow":
            self._flight_status_btn.setText(self._flight_status_not_ready_label())
            self._flight_status_btn.setToolTip(str(message or ""))
            self._map_widget.set_flight_status("yellow", message)
            return
        if state_norm == "red":
            self._flight_status_btn.setText(self._flight_status_not_ready_label())
            self._flight_status_btn.setToolTip(str(message or ""))
            self._map_widget.set_flight_status("red", message)
            return
        # Cold-start / idle disconnected: Web stylesheet `#linkBanner` neutral background — not maroon.
        self._flight_status_btn.setText("NOT READY TO ARM")
        self._flight_status_btn.setToolTip("")
        self._map_widget.set_flight_status("idle", message)

    def _flight_status_not_ready_label(self) -> str:
        """Short header chip when the vehicle is connected but not armable."""
        reason = self._active_prearm_reason() or str(self._arm_denied_reason or "").strip()
        if reason:
            short = reason.replace("\n", " ")
            if len(short) > 34:
                short = short[:31] + "…"
            return f"NOT READY · {short}"
        standby = int(mavutil.mavlink.MAV_STATE_STANDBY)
        if int(self._hb_system_status) < standby:
            return "NOT READY · Booting"
        health = self._prearm_health
        if health is not None and health.is_authoritative(time.monotonic()) and not health.passing:
            # The vehicle says the checks fail but has not printed why yet
            # (ArduPilot repeats the reason only every 30 s).
            return "NOT READY · PreArm checks failing"
        if int(self._last_gps_fix_type or 0) < 3:
            return "NOT READY · Need 3D GPS"
        mode = str(self._hb_mode_text or "").strip()
        if mode:
            return f"NOT READY · {mode} OK, PreArm pending"
        return "NOT READY TO ARM"

    def _is_probably_flying(self) -> bool:
        """Best-effort airborne detector for header text."""
        if self._armed_since is None:
            return False
        rel_display_m = float(self._map_rel_alt_m)
        return abs(rel_display_m) >= 1.5 or float(self._map_groundspeed_mps) >= 1.2

    def _refresh_dashboard_flight_state(self) -> None:
        """Keep banner/button state aligned with latest heartbeat + motion cues."""
        self._hb_arm_ready = self._compute_hb_arm_ready(
            armed=bool(self._hb_armed),
            system_status=int(self._hb_system_status),
            mode_text=str(self._hb_mode_text or ""),
        )
        self._fields["arm_ready"].setText(self._arm_ready_field_text())
        self._apply_state_style(
            self._fields["arm_ready"],
            "ok" if self._hb_arm_ready else ("warn" if self._heartbeat_seen else ""),
        )
        now = time.monotonic()
        if now < float(self._arm_denied_until_mono):
            reason = str(self._arm_denied_reason or "").strip() or self._active_prearm_reason()
            msg = f"Arm denied - {reason}" if reason else "Arm denied"
            self._set_dashboard_flight_status("red", msg)
            return
        if self._hb_armed:
            mode_disp = str(self._hb_mode_text or "Unknown").strip()
            if self._is_probably_flying():
                self._set_dashboard_flight_status("green", f"In Flight - {mode_disp}")
                self._flight_status_btn.setText("IN FLIGHT")
            else:
                self._set_dashboard_flight_status("green", f"Armed - {mode_disp}")
                self._flight_status_btn.setText("ARMED")
            return
        if not self._hb_arm_ready:
            reason = self._active_prearm_reason()
            mode_disp = str(self._hb_mode_text or "").strip()
            if reason:
                msg = f"Connected - Not Ready to Arm ({reason})"
            elif self._prearm_verdict_is_failing():
                msg = (
                    "Connected - Not Ready to Arm (vehicle PreArm checks failing"
                    + (f", {mode_disp} mode)" if mode_disp else ")")
                )
            elif mode_disp:
                msg = (
                    f"Connected - {mode_disp} mode (not arm status). "
                    "Waiting for vehicle PreArm checks to pass."
                )
            else:
                msg = "Connected - Not Ready to Arm (waiting for PreArm OK)"
            self._set_dashboard_flight_status("red", msg)
            return
        mode_disp = str(self._hb_mode_text or "").strip()
        ready_msg = f"Ready to Arm - {mode_disp}" if mode_disp and mode_disp != "—" else "Ready to Arm"
        self._set_dashboard_flight_status("green", ready_msg)
        self._flight_status_btn.setText("READY TO ARM")

    def _normalize_mode_token(self, mode_text: str) -> str:
        return str(mode_text or "").strip().upper().replace(" ", "_").replace("-", "_")

    def _is_home_wait_prearm_reason(self) -> bool:
        reason = str(self._prearm_block_reason or "").strip().lower()
        return ("waiting for home" in reason) or ("ahrs" in reason and "home" in reason)

    def _is_non_gps_mode(self, mode_text: str) -> bool:
        mode = self._normalize_mode_token(mode_text)
        return mode in {"ALT_HOLD", "STABILIZE", "ACRO", "DRIFT", "SPORT"}

    def _prearm_verdict(self):
        """The autopilot PreArm verdict, or ``None`` when it is not usable."""
        health = getattr(self, "_prearm_health", None)
        if health is None or not health.is_authoritative(time.monotonic()):
            return None
        return health

    def _prearm_verdict_is_failing(self) -> bool:
        verdict = self._prearm_verdict()
        return verdict is not None and not verdict.passing

    def _prearm_block_active(self) -> bool:
        verdict = self._prearm_verdict()
        if verdict is not None:
            # The vehicle publishes the answer twice a second; a STATUSTEXT
            # window is only a stand-in for when it does not.
            return not verdict.passing
        if time.monotonic() >= float(self._prearm_block_until_mono):
            return False
        return bool(str(self._prearm_block_reason or "").strip())

    def _active_prearm_reason(self) -> str:
        """The PreArm reason, but only while it is still the current one.

        The string used to outlive its window: ``_prearm_block_active()`` was
        expiry-aware, yet every site that *rendered* the reason read
        ``_prearm_block_reason`` directly. That is how a resolved "Compass
        inconsistent" stayed on screen as the reason while the live fault was
        GPS. When the vehicle publishes a verdict it decides; otherwise the
        STATUSTEXT window does.
        """
        reason = str(self._prearm_block_reason or "").strip()
        if not reason:
            return ""
        verdict = self._prearm_verdict()
        if verdict is not None:
            return reason if not verdict.passing else ""
        if time.monotonic() >= float(self._prearm_block_until_mono):
            return ""
        return reason

    def _clear_prearm_block(self) -> None:
        self._prearm_block_reason = ""
        self._prearm_block_until_mono = 0.0
        self._arm_denied_reason = ""
        self._arm_denied_until_mono = 0.0

    def _arm_ready_field_text(self) -> str:
        """Dashboard "arm ready" row — say whether this is the vehicle or a guess."""
        verdict = self._prearm_verdict()
        if verdict is not None:
            if verdict.passing:
                return "Ready - vehicle PreArm OK"
            reason = self._active_prearm_reason()
            return f"PreArm: {reason}" if reason else "PreArm checks failing"
        if self._hb_arm_ready:
            return "Likely ready (no PreArm report)"
        reason = self._active_prearm_reason()
        if reason:
            return f"PreArm: {reason}"
        if self._heartbeat_seen:
            return f"System status {self._hb_system_status}"
        return "Best-effort from telemetry"

    def _compute_hb_arm_ready(self, *, armed: bool, system_status: int, mode_text: str) -> bool:
        """Whether the link banner should show green (ready / armed / in flight)."""
        standby = int(mavutil.mavlink.MAV_STATE_STANDBY)
        if system_status < standby:
            return False
        if armed:
            return True

        verdict = self._prearm_verdict()
        if verdict is not None:
            # ArduPilot puts the answer in every SYS_STATUS. Nothing inferred
            # can beat it: system_status stays STANDBY while PreArm fails, and
            # the failure text is only printed every 30 s, so "no PreArm
            # message lately" never meant the checks had passed.
            return bool(verdict.passing)

        # No verdict published (ARMING_CHECK=0, or SYS_STATUS not flowing):
        # fall back to the best-effort inference.
        mode_non_gps = self._is_non_gps_mode(mode_text)
        home_wait_ok = mode_non_gps and self._is_home_wait_prearm_reason()
        if self._prearm_block_active() and not home_wait_ok:
            return False

        # ALT_HOLD / STABILIZE / ACRO: no GPS PreArm — STANDBY without a PreArm fault is enough.
        if mode_non_gps:
            return True

        if self._arm_ready_confirmed or home_wait_ok:
            return True

        # GPS modes (LOITER, etc.): STANDBY + 3D GPS + no PreArm fault, after link settles.
        if int(getattr(self, "_last_gps_fix_type", 0) or 0) >= 3:
            since = getattr(self, "_hb_connected_since_mono", None)
            if since is not None and (time.monotonic() - float(since)) >= 12.0:
                return True

        return False

    def _update_prearm_gate_from_statustext(self, text: str, *, severity: int = SEVERITY_INFO) -> None:
        """Track the PreArm state from vehicle text (fallback when SYS_STATUS is silent).

        The prefix is the whole signal. ArduPilot emits every blocking check as
        ``PreArm: <reason>`` and every rejected arm attempt as ``Arm: <reason>``
        (``AP_Arming::check_failed``). The old classifier additionally demanded
        one of "wait/fail/not/deny/error" in the line, which none of the common
        reasons contain — "PreArm: Compass inconsistent", "PreArm: Need 3D Fix",
        "PreArm: GPS 1: Bad fix", "PreArm: Battery below minimum arming
        voltage" all fell straight through, so the gate never engaged and the
        banner stayed green on a vehicle that would not arm.
        """
        t = str(text or "").strip()
        if not t:
            return
        low = t.lower()
        # Note: no bare "armed" here. It matches "Disarming"/"Disarmed" and used
        # to clear a live PreArm block on the way down.
        if (
            "checks passed" in low
            or "prearm good" in low
            or low.startswith("arming motors")
            or low.startswith("armable")
        ):
            self._clear_prearm_block()
            self._arm_ready_confirmed = True
            return
        is_prearm_block = (
            low.startswith("prearm:")
            or low.startswith("pre-arm:")
            or low.startswith("arm:")
            or low.startswith("prearm ")
        )
        if not is_prearm_block:
            return
        reason = t.split(":", 1)[1].strip() if ":" in t else t
        self._prearm_block_reason = reason or t
        self._arm_ready_confirmed = False
        now = time.monotonic()
        # Sticky window for the fallback path only. ArduPilot reprints the
        # current failure every PREARM_DISPLAY_PERIOD (30 s), so give the next
        # reprint room to land before the reason is treated as stale.
        self._prearm_block_until_mono = max(float(self._prearm_block_until_mono), now + 35.0)
        if low.startswith("arm:"):
            self._arm_denied_reason = reason or t
            # Strong immediate feedback after an actual arm attempt is denied.
            self._arm_denied_until_mono = now + 8.0

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
        # Match FlyGCS / MAVLink: GLOBAL_POSITION_INT.relative_alt is already above home.
        # Do not subtract arm baseline again (that caused ~2–3 m low vs other GCS at 100 m).
        if self._armed_since is None and float(self._map_groundspeed_mps) < 0.5:
            if abs(rel_display_m) < 1.5:
                rel_display_m = 0.0
        dist_home_m = 0.0
        try:
            if (
                self._last_gps_lat is not None
                and self._last_gps_lon is not None
                and self._home_lat is not None
                and self._home_lon is not None
            ):
                dist_home_m = self._haversine_m(
                    float(self._home_lat),
                    float(self._home_lon),
                    float(self._last_gps_lat),
                    float(self._last_gps_lon),
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
