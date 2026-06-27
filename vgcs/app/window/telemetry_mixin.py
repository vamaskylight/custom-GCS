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


class MainWindowTelemetryMixin:
    """Extracted from MainWindow — uses host state via self."""

    @staticmethod
    def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        r = 6_371_000.0
        p1 = math.radians(lat1)
        p2 = math.radians(lat2)
        dp = math.radians(lat2 - lat1)
        dl = math.radians(lon2 - lon1)
        a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
        return 2 * r * math.asin(min(1.0, math.sqrt(max(0.0, a))))

    @staticmethod
    def _extract_remote_id_text(data: dict[str, object]) -> str:
        candidates = (
            "uas_id",
            "id_or_mac",
            "operator_id",
            "self_id",
            "description",
        )
        for key in candidates:
            raw = data.get(key)
            if raw is None:
                continue
            if isinstance(raw, str):
                txt = raw.strip().strip("\x00")
                if txt:
                    return txt
            elif isinstance(raw, (bytes, bytearray)):
                txt = raw.decode("ascii", errors="ignore").strip().strip("\x00")
                if txt:
                    return txt
            elif isinstance(raw, list):
                try:
                    txt = bytes(int(v) & 0xFF for v in raw).decode(
                        "ascii", errors="ignore"
                    ).strip().strip("\x00")
                except Exception:
                    txt = ""
                if txt:
                    return txt
        return ""

    def _reset_telemetry_fields(self) -> None:
        self._armed_since = None
        self._map_rel_alt_m = 0.0
        self._map_msl_alt_m = 0.0
        self._map_groundspeed_mps = 0.0
        self._map_climb_mps = 0.0
        self._heading = 0.0
        self._max_telem_dist_m = 0.0
        self._home_lat = None
        self._home_lon = None
        self._home_amsl_m = None
        self._home_rel_alt_baseline_m = None
        self._last_gps_lat = None
        self._last_gps_lon = None
        self._fields["armed"].setText("No")
        self._apply_state_style(self._fields["armed"], "warn")
        self._fields["flight_time"].setText("00:00")
        self._fields["lat_lon"].setText("—")
        self._fields["alt_rel"].setText("—")
        self._fields["alt_msl"].setText("—")
        self._fields["groundspeed"].setText("—")
        self._fields["airspeed"].setText("—")
        self._fields["heading"].setText("—")
        self._fields["attitude"].setText("—")
        self._fields["gps"].setText("—")
        self._fields["battery"].setText("—")
        self._fields["rc_link"].setText("—")
        self._fields["failsafe_battery"].setText("—")
        self._fields["failsafe_rc"].setText("—")
        self._fields["arm_ready"].setText("Best-effort from telemetry")
        self._fields["video_link"].setText("N/A")
        self._fields["obstacle_prox"].setText("N/A")
        self._fields["rangefinder"].setText("N/A")
        self._apply_state_style(self._fields["video_link"], "na")
        self._apply_state_style(self._fields["obstacle_prox"], "na")
        self._apply_state_style(self._fields["rangefinder"], "na")
        self._apply_state_style(self._fields["failsafe_battery"], "")
        self._apply_state_style(self._fields["failsafe_rc"], "")
        self._apply_state_style(self._fields["arm_ready"], "")
        self._apply_state_style(self._fields["rc_link"], "")
        self._map_widget.set_mission_waypoint_count(0)
        self._top_gps_sat.setText("—")
        self._top_gps_hdop.setText("—")
        self._top_flight_mode.setText("—")
        self._top_battery.setText("—")
        self._top_remote_id.setText("N/A")
        self._set_top_vehicle_msg("—")
        self._map_widget.set_header_mode("—")
        self._map_widget.set_header_vehicle_msg("—")
        self._map_widget.set_header_gps(0, "N/A")
        self._map_widget.set_header_battery("N/A")
        self._map_widget.set_header_remote_id("N/A")
        self._map_widget.set_plan_vehicle_info("—", "—")
        self._mission_table_updating = True
        self._mission_table.setRowCount(0)
        self._mission_table_updating = False
        self._last_map_overlay_refresh_s = None
        self._sync_visible_map_overlay_metrics()
        self._refresh_footer_summary()
        self._sync_plan_flight_chrome()

    def _on_heartbeat(self, sysid: int, compid: int, mav_ver: int) -> None:
        if not self._heartbeat_seen:
            self._heartbeat_seen = True
            self._hb_connected_since_mono = time.monotonic()
            self._connect_attempt_active = False
            self._status.setText("Connected")
            self._apply_state_style(self._status, "ok")
            self._set_dashboard_flight_status("yellow", "Connected - validating arm checks")
            self._map_widget.set_link_connected(True)
        self._hb.setText(f"sys {sysid} · comp {compid} · mav {mav_ver}")
        self._apply_state_style(self._hb, "ok")
        if not self._rid_live_available:
            self._map_widget.set_header_remote_id(f"ID {sysid}")
        self._sync_plan_flight_chrome()

    def _on_telemetry(self, msg_type: str, payload: object) -> None:
        data = payload if isinstance(payload, dict) else {}
        if msg_type == "HEARTBEAT":
            armed = bool(data.get("armed", False))
            system_status = int(data.get("system_status", 0))
            mode_text = str(data.get("mode_text", "") or "").strip()
            if not mode_text:
                mode_text = human_mode_name(
                    vehicle_type=int(data.get("vehicle_type", 0) or 0),
                    custom_mode=int(data.get("custom_mode", 0) or 0),
                )
            hb_key = (armed, system_status, mode_text)
            arm_ready = self._compute_hb_arm_ready(
                armed=armed,
                system_status=system_status,
                mode_text=mode_text,
            )
            ui_key = (
                hb_key,
                arm_ready,
                str(self._prearm_block_reason or ""),
                int(self._prearm_block_until_mono),
                bool(self._arm_ready_confirmed),
            )
            if ui_key == getattr(self, "_last_hb_ui_key", None):
                return
            self._last_hb_ui_key = ui_key
            self._fields["armed"].setText("Yes" if armed else "No")
            self._apply_state_style(self._fields["armed"], "ok" if armed else "warn")
            if armed and self._armed_since is None:
                self._armed_since = time.monotonic()
                # Anchor HOME at arm transition using latest raw GPS sample.
                if self._last_gps_lat is not None and self._last_gps_lon is not None:
                    self._home_lat = float(self._last_gps_lat)
                    self._home_lon = float(self._last_gps_lon)
                self._home_rel_alt_baseline_m = float(self._map_rel_alt_m)
                self._max_telem_dist_m = 0.0
            if not armed:
                self._armed_since = None
                self._fields["flight_time"].setText("00:00")
            self._hb_armed = armed
            self._hb_system_status = system_status
            self._hb_arm_ready = arm_ready
            self._hb_mode_text = mode_text
            if arm_ready:
                self._fields["arm_ready"].setText("Likely ready")
            elif self._prearm_block_reason:
                self._fields["arm_ready"].setText(f"PreArm: {self._prearm_block_reason}")
            else:
                self._fields["arm_ready"].setText(f"System status {system_status}")
            self._apply_state_style(self._fields["arm_ready"], "ok" if arm_ready else "warn")
            if arm_ready:
                # Do not clear _arm_not_ready_alert_shown here: brief STANDBY in a flickering
                # HEARTBEAT would re-arm the popup and make OK / title-bar close feel ignored.
                self._arm_not_ready_since_mono = None
            else:
                now = time.monotonic()
                if self._arm_not_ready_since_mono is None:
                    self._arm_not_ready_since_mono = now
                if (
                    not self._arm_not_ready_alert_shown
                    and (now - self._arm_not_ready_since_mono) >= 5.0
                ):
                    self._arm_not_ready_alert_shown = True
                    extra = self._format_recent_vehicle_msgs_for_alert()
                    body = (
                        "Vehicle connected, but the autopilot heartbeat still reports "
                        f"system_status={system_status} (not STANDBY / ready yet).\n\n"
                        "This often clears within a few seconds while the vehicle boots. "
                        "If it does not clear, check calibration, GPS/EKF, and other PreArm messages."
                    )
                    if extra:
                        body = f"{body}\n\n{extra}"
                    QMessageBox.warning(self, "Vehicle Msg", body)
            self._refresh_dashboard_flight_state()
            self._sync_mode_options_for_vehicle(int(data.get("vehicle_type", 0) or 0))
            self._top_flight_mode.setText(mode_text)
            self._map_widget.set_header_mode(mode_text)
            # Keep mode dropdown aligned with the vehicle (dialog copies this combo at open).
            if mode_text and self._mode_combo.findText(mode_text) >= 0:
                self._mode_combo.blockSignals(True)
                self._mode_combo.setCurrentText(mode_text)
                self._mode_combo.blockSignals(False)
            ap = int(data.get("autopilot", 0) or 0)
            vt = int(data.get("vehicle_type", 0) or 0)
            self._map_widget.set_plan_vehicle_info(
                _mavlink_autopilot_label(ap),
                _mavlink_vehicle_type_label(vt),
            )
        elif msg_type == "GLOBAL_POSITION_INT":
            lat = float(data.get("lat", 0.0))
            lon = float(data.get("lon", 0.0))
            # Some vehicles output 0,0 before GPS has a valid fix. Never push that into the map,
            # otherwise the UI recenters to the Gulf of Guinea and loads misleading/placeholder tiles.
            if abs(lat) < 1e-9 and abs(lon) < 1e-9:
                return
            self._last_gps_lat = lat
            self._last_gps_lon = lon
            self._map_rel_alt_m = float(data.get("relative_alt_m", 0.0))
            self._map_msl_alt_m = float(data.get("alt_msl_m", 0.0))
            self._home_amsl_m = float(self._map_msl_alt_m) - float(self._map_rel_alt_m)
            if str(self._settings.value("plan_alt_ref", "rel") or "").lower() == "amsl":
                s = self._settings
                st = {
                    "altRef": str(s.value("plan_alt_ref", "rel") or "rel"),
                    "initialWpAltM": float(
                        s.value("plan_initial_wp_alt_m", s.value("plan_initial_wp_alt_ft", 164.0))
                        or 164.0
                    ),
                }
                self._map_widget.set_default_waypoint_alt_m(
                    self._default_wp_alt_m_for_plan_state(st)
                )
            if self._home_lat is None or self._home_lon is None:
                self._home_lat = lat
                self._home_lon = lon
                self._home_rel_alt_baseline_m = float(data.get("relative_alt_m", 0.0))
            if (
                self._armed_since is not None
                and self._home_lat is not None
                and self._home_lon is not None
            ):
                try:
                    d = self._haversine_m(self._home_lat, self._home_lon, lat, lon)
                    self._max_telem_dist_m = max(self._max_telem_dist_m, d)
                except Exception:
                    pass
            self._fields["lat_lon"].setText(
                f"{data.get('lat', 0.0):.7f}, {data.get('lon', 0.0):.7f}"
            )
            self._fields["alt_rel"].setText(f"{data.get('relative_alt_m', 0.0):.1f} m")
            self._fields["alt_msl"].setText(f"{data.get('alt_msl_m', 0.0):.1f} m")
            try:
                self._map_widget.set_vehicle_alt_msl(float(data.get("alt_msl_m", 0.0) or 0.0))
            except Exception:
                pass
            self._map_widget.set_vehicle_position(
                lat,
                lon,
                relative_alt_m=float(data.get("relative_alt_m", 0.0)),
                groundspeed_mps=float(
                    data.get("groundspeed_mps", self._map_groundspeed_mps) or 0.0
                ),
            )
            if self._auto_center_pending:
                self._auto_center_pending = False
                self._map_widget.center_on_vehicle()
            # Fused course / velocity course matches map motion better than body yaw alone.
            course: float | None = None
            if data.get("hdg_deg") is not None:
                course = float(data["hdg_deg"])
            elif data.get("ground_track_deg") is not None:
                course = float(data["ground_track_deg"])
            if course is not None:
                self._heading = course
                self._fields["heading"].setText(f"{int(round(course))}°")
                self._compass.set_heading_deg(course)
                self._map_widget.set_vehicle_heading(course, source="gpi")
            self._refresh_dashboard_flight_state()
        elif msg_type == "MISSION_CURRENT":
            self._map_widget.set_mission_nav_seq(int(data.get("seq", 0) or 0))
        elif msg_type == "VFR_HUD":
            self._map_groundspeed_mps = float(data.get("groundspeed", 0.0))
            self._map_climb_mps = float(data.get("climb", 0.0))
            try:
                self._map_widget._update_map_motion_state(self._map_groundspeed_mps)
            except Exception:
                pass
            self._fields["groundspeed"].setText(f"{data.get('groundspeed', 0.0):.1f} m/s")
            self._fields["airspeed"].setText(f"{data.get('airspeed', 0.0):.1f} m/s")
            hd = float(data.get("heading", 0.0))
            self._fields["heading"].setText(f"{int(hd)}°")
            self._heading = hd
            self._compass.set_heading_deg(hd)
            self._map_widget.set_vehicle_heading(hd, source="vfr")
            self._refresh_dashboard_flight_state()
        elif msg_type == "ATTITUDE":
            self._fields["attitude"].setText(
                f"{data.get('roll_deg', 0.0):.1f} / "
                f"{data.get('pitch_deg', 0.0):.1f} / "
                f"{data.get('yaw_deg', 0.0):.1f} deg"
            )
            try:
                self._map_widget.set_vehicle_attitude(
                    float(data.get("roll_deg", 0.0)),
                    float(data.get("pitch_deg", 0.0)),
                )
            except Exception:
                pass
            yaw_deg = float(data.get("yaw_deg", 0.0))
            hd_att = (yaw_deg + 360.0) % 360.0
            self._heading = hd_att
            self._compass.set_heading_deg(hd_att)
            self._map_widget.set_vehicle_heading(hd_att, source="att")
        elif msg_type == "GPS_RAW_INT":
            hdop = data.get("hdop")
            hdop_text = "N/A" if hdop is None else f"{hdop:.2f}"
            sat = int(data.get("satellites_visible", 0))
            fix_type = int(data.get("fix_type", 0) or 0)
            self._last_gps_fix_type = fix_type
            self._last_gps_sats = sat
            self._fields["gps"].setText(
                f"fix={fix_type} sat={sat} hdop={hdop_text}"
            )
            self._top_gps_sat.setText(str(sat))
            self._top_gps_hdop.setText(hdop_text)
            self._map_widget.set_header_gps(sat, hdop_text, fix_type=fix_type)
            if hdop is not None:
                try:
                    self._map_widget.set_gps_hdop(float(hdop))
                except Exception:
                    pass
            # Fallback when GLOBAL_POSITION_INT is 0,0 / missing but GPS_RAW has a fix.
            raw_lat = data.get("lat")
            raw_lon = data.get("lon")
            if raw_lat is not None and raw_lon is not None:
                lat = float(raw_lat)
                lon = float(raw_lon)
                if fix_type >= 2 and (abs(lat) > 1e-9 or abs(lon) > 1e-9):
                    alt_msl = float(data.get("alt_msl_m", 0.0) or 0.0)
                    self._map_widget.set_vehicle_position(
                        lat,
                        lon,
                        relative_alt_m=None,
                        groundspeed_mps=float(self._map_groundspeed_mps),
                    )
                    if alt_msl:
                        self._map_msl_alt_m = alt_msl
        elif msg_type == "SYS_STATUS":
            pct = int(data.get("battery_remaining", -1))
            pct_text = "N/A" if pct < 0 else f"{pct}%"
            voltage = float(data.get("voltage_v", 0.0))
            current = float(data.get("current_a", -1.0))
            current_text = "N/A" if current < 0 else f"{current:.1f} A"
            self._fields["battery"].setText(
                f"{voltage:.2f} V, {current_text}, {pct_text}"
            )
            # Always show voltage in the header; percent alone is not actionable for operators.
            # Use 2 decimals so small real-time changes are visible (e.g. 11.80 -> 11.74).
            bat_header = (
                f"{voltage:.2f}V ({pct_text})" if pct_text != "N/A" else f"{voltage:.2f}V"
            )
            self._top_battery.setText(bat_header)
            self._map_widget.set_header_battery(bat_header)
            sensors_present = int(data.get("sensors_present", 0))
            sensors_enabled = int(data.get("sensors_enabled", 0))
            sensors_health = int(data.get("sensors_health", 0))
            battery_mask = int(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_BATTERY)
            rc_mask = int(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_RC_RECEIVER)
            battery_monitored = bool(sensors_present & battery_mask and sensors_enabled & battery_mask)
            rc_monitored = bool(sensors_present & rc_mask and sensors_enabled & rc_mask)
            battery_healthy = bool(sensors_health & battery_mask)
            rc_healthy = bool(sensors_health & rc_mask)
            battery_ok = (not battery_monitored or battery_healthy) and (pct < 0 or pct > 15)
            self._set_ok_warn_field("failsafe_battery", battery_ok)
            if rc_monitored:
                self._set_ok_warn_field("failsafe_rc", rc_healthy)
            else:
                self._fields["failsafe_rc"].setText("N/A")
                self._apply_state_style(self._fields["failsafe_rc"], "na")
        elif msg_type == "BATTERY_STATUS":
            # Real vehicles often report pack voltage here (Mission Planner simulator may not).
            # MAVLink: voltages[] in mV, battery_remaining in % (or -1).
            pct = int(data.get("battery_remaining", -1))
            pct_text = "N/A" if pct < 0 else f"{pct}%"
            v_mv = None
            try:
                v_arr = data.get("voltages")
                if isinstance(v_arr, (list, tuple)) and v_arr:
                    v0 = int(v_arr[0] or 0)
                    if v0 > 0:
                        v_mv = v0
            except Exception:
                v_mv = None
            voltage_v = float(data.get("voltage_v", 0.0) or 0.0)
            if (not voltage_v or voltage_v <= 0.1) and v_mv is not None:
                voltage_v = float(v_mv) / 1000.0
            if voltage_v <= 0.1:
                return
            bat_header = (
                f"{voltage_v:.2f}V ({pct_text})" if pct_text != "N/A" else f"{voltage_v:.2f}V"
            )
            self._top_battery.setText(bat_header)
            self._map_widget.set_header_battery(bat_header)
        elif msg_type == "OBSTACLE_DISTANCE":
            self._map_widget.set_obstacle_distance(data)
            prox, _ = self._map_widget.get_obstacle_sensor_summary()
            self._fields["obstacle_prox"].setText(prox)
            self._apply_state_style(
                self._fields["obstacle_prox"],
                "ok" if prox != "N/A" else "na",
            )
        elif msg_type == "DISTANCE_SENSOR":
            self._map_widget.set_distance_sensor(data)
            _, rf = self._map_widget.get_obstacle_sensor_summary()
            self._fields["rangefinder"].setText(rf)
            self._apply_state_style(
                self._fields["rangefinder"],
                "ok" if rf != "N/A" else "na",
            )
        elif msg_type == "RADIO_STATUS":
            self._fields["rc_link"].setText(
                f"rssi={int(data.get('rssi', 0))} remrssi={int(data.get('remrssi', 0))}"
            )
            self._apply_state_style(self._fields["rc_link"], "ok")
        elif msg_type == "STATUSTEXT":
            text = str(data.get("text", "")).strip()
            if text:
                self._recent_statustext.append(text)
                self._update_prearm_gate_from_statustext(text)
                self._set_top_vehicle_msg(text)
                self._map_widget.set_header_vehicle_msg(text)
                self._refresh_dashboard_flight_state()
                # STATUSTEXT can burst during param download; logging each line hammers QTextEdit.
                now = time.monotonic()
                last_log = float(getattr(self, "_last_statustext_log_mono", 0.0))
                if now - last_log >= 0.12:
                    self._last_statustext_log_mono = now
                    self._append_log(f"STATUSTEXT: {text}")
        elif msg_type.startswith("OPEN_DRONE_ID_"):
            rid_text = self._extract_remote_id_text(data)
            if rid_text:
                self._rid_live_available = True
                rid_display = f"RID: {rid_text}"
                self._top_remote_id.setText(rid_display)
                self._map_widget.set_header_remote_id(rid_display)
        self._refresh_footer_summary()
        self._maybe_refresh_map_web_overlays()

    def _refresh_c13_lrf_display(self) -> None:
        """Poll C13 TOP SLR when LRF armed or locked."""
        if not bool(getattr(self, "_heartbeat_seen", False)):
            return
        provider = str(self._settings.value("camera/provider", "mavlink") or "mavlink").strip().lower()
        if provider != "skydroid":
            return
        cc = self._camera_control_backend
        try:
            self._map_widget.enable_c13_lrf_ui(True)
        except Exception:
            pass
        armed = False
        locking = False
        try:
            armed = bool(self._map_widget.is_c13_lrf_armed())
            locking = bool(self._map_widget.is_c13_lrf_locking())
        except Exception:
            pass
        is_locked = False
        if cc is not None:
            fn = getattr(cc, "is_lrf_locked", None)
            is_locked = bool(fn()) if callable(fn) else False
        dist = None
        if is_locked:
            dist = read_companion_laser_range_m(cc)
        elif armed or locking:
            dist = poll_companion_laser_range_m(cc)
        if dist is None:
            if is_locked:
                try:
                    self._fields["rangefinder"].setText("— (C13 locked, no SLR)")
                    self._apply_state_style(self._fields["rangefinder"], "idle")
                except Exception:
                    pass
            elif armed or locking:
                try:
                    hint = "locking…" if locking else "aim gimbal at target"
                    self._fields["rangefinder"].setText(f"— ({hint})")
                    self._apply_state_style(self._fields["rangefinder"], "idle")
                except Exception:
                    pass
            return
        if locking and not is_locked:
            text = f"{dist:.1f} m (C13 locking)"
        elif armed and not is_locked:
            text = f"{dist:.1f} m (C13 live)"
        else:
            text = f"{dist:.1f} m (C13 locked)"
        try:
            latlon = self._map_widget.get_c13_lrf_lock_latlon()
            if latlon is not None and is_locked:
                text = f"{text} · {latlon[0]:.6f}, {latlon[1]:.6f}"
        except Exception:
            pass
        try:
            self._fields["rangefinder"].setText(text)
            self._apply_state_style(self._fields["rangefinder"], "ok")
        except Exception:
            pass
        try:
            self._map_widget.set_companion_laser_range_m(dist)
        except Exception:
            pass
