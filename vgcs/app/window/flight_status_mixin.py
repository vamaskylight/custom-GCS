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

        if state_norm == "green":
            self._flight_status_btn.setText("ARMED" if self._armed_since is not None else "READY TO ARM")
            self._flight_status_btn.setToolTip(
                "Flight mode and arm readiness are separate. "
                f"Current mode: {self._hb_mode_text or '—'}"
            )
            self._set_top_vehicle_msg(message)
            self._map_widget.set_flight_status("green", message)
            return
        if state_norm == "yellow":
            self._flight_status_btn.setText(self._flight_status_not_ready_label())
            self._flight_status_btn.setToolTip(str(message or ""))
            self._set_top_vehicle_msg(message)
            self._map_widget.set_flight_status("yellow", message)
            return
        if state_norm == "red":
            self._flight_status_btn.setText(self._flight_status_not_ready_label())
            self._flight_status_btn.setToolTip(str(message or ""))
            self._set_top_vehicle_msg(message)
            self._map_widget.set_flight_status("red", message)
            return
        # Cold-start / idle disconnected: Web stylesheet `#linkBanner` neutral background — not maroon.
        self._flight_status_btn.setText("NOT READY TO ARM")
        self._flight_status_btn.setToolTip("")
        self._set_top_vehicle_msg(message)
        self._map_widget.set_flight_status("idle", message)

    def _flight_status_not_ready_label(self) -> str:
        """Short header chip when the vehicle is connected but not armable."""
        reason = str(self._prearm_block_reason or self._arm_denied_reason or "").strip()
        if reason:
            short = reason.replace("\n", " ")
            if len(short) > 34:
                short = short[:31] + "…"
            return f"NOT READY · {short}"
        standby = int(mavutil.mavlink.MAV_STATE_STANDBY)
        if int(self._hb_system_status) < standby:
            return "NOT READY · Booting"
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
        if self._hb_arm_ready:
            self._fields["arm_ready"].setText("Likely ready")
            self._apply_state_style(self._fields["arm_ready"], "ok")
        elif self._prearm_block_reason:
            self._fields["arm_ready"].setText(f"PreArm: {self._prearm_block_reason}")
            self._apply_state_style(self._fields["arm_ready"], "warn")
        elif self._heartbeat_seen:
            self._fields["arm_ready"].setText(f"System status {self._hb_system_status}")
            self._apply_state_style(self._fields["arm_ready"], "warn")
        now = time.monotonic()
        if now < float(self._arm_denied_until_mono):
            reason = str(self._arm_denied_reason or self._prearm_block_reason or "").strip()
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
            reason = str(self._prearm_block_reason or "").strip()
            mode_disp = str(self._hb_mode_text or "").strip()
            if reason:
                msg = f"Connected - Not Ready to Arm ({reason})"
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

    def _prearm_block_active(self) -> bool:
        if time.monotonic() >= float(self._prearm_block_until_mono):
            return False
        return bool(str(self._prearm_block_reason or "").strip())

    def _compute_hb_arm_ready(self, *, armed: bool, system_status: int, mode_text: str) -> bool:
        """Whether the link banner should show green (ready / armed / in flight)."""
        standby = int(mavutil.mavlink.MAV_STATE_STANDBY)
        if system_status < standby:
            return False

        mode_non_gps = self._is_non_gps_mode(mode_text)
        home_wait_ok = mode_non_gps and self._is_home_wait_prearm_reason()
        if self._prearm_block_active() and not home_wait_ok:
            return False

        if armed:
            return True

        # ALT_HOLD / STABILIZE / ACRO: no GPS PreArm — STANDBY without a PreArm fault is enough.
        if mode_non_gps:
            return True

        if self._arm_ready_confirmed or home_wait_ok:
            return True

        # GPS modes (LOITER, etc.): STANDBY + 3D GPS + no PreArm fault, after link settles.
        if (
            system_status >= standby
            and int(getattr(self, "_last_gps_fix_type", 0) or 0) >= 3
            and not self._prearm_block_active()
        ):
            since = getattr(self, "_hb_connected_since_mono", None)
            if since is not None and (time.monotonic() - float(since)) >= 12.0:
                return True

        return False

    def _update_prearm_gate_from_statustext(self, text: str) -> None:
        t = str(text or "").strip()
        if not t:
            return
        low = t.lower()
        if any(
            k in low
            for k in (
                "armed",
                "armable",
                "ready to arm",
                "prearm: checks passed",
                "prearm checks passed",
                "checks passed",
            )
        ):
            self._prearm_block_reason = ""
            self._prearm_block_until_mono = 0.0
            self._arm_denied_reason = ""
            self._arm_denied_until_mono = 0.0
            self._arm_ready_confirmed = True
            return
        is_prearm_block = (
            ("prearm" in low or low.startswith("arm:"))
            and any(k in low for k in ("wait", "fail", "not", "deny", "error"))
            and "passed" not in low
        )
        if is_prearm_block:
            reason = t.split(":", 1)[-1].strip() if ":" in t else t
            self._prearm_block_reason = reason or t
            self._arm_ready_confirmed = False
            now = time.monotonic()
            # Keep gate sticky long enough to avoid "Ready to Arm" bounce between sparse STATUSTEXT.
            self._prearm_block_until_mono = max(float(self._prearm_block_until_mono), now + 30.0)
            if low.startswith("arm:"):
                self._arm_denied_reason = reason or t
                # Strong immediate feedback after an actual arm attempt is denied.
                self._arm_denied_until_mono = now + 8.0
            return

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
