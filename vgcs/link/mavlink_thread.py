"""
Background MAVLink reader for the GCS link layer.

Runs blocking pymavlink I/O off the GUI thread and emits decoded telemetry.
"""

from __future__ import annotations

import math
import re
import threading
import time
from collections import deque
from typing import Optional

from PySide6.QtCore import QThread, Signal
from pymavlink import mavutil


class MavlinkThread(QThread):
    """Connect to a MAVLink stream and report link/heartbeat/telemetry state."""

    link_up = Signal()
    link_down = Signal()
    heartbeat = Signal(int, int, int)  # system_id, component_id, mavlink_version
    telemetry = Signal(str, object)  # msg_type, field dictionary
    link_timeout = Signal(float)  # seconds without messages
    mission_uploaded = Signal(int)  # waypoint count
    mission_downloaded = Signal(object)  # list of dict waypoints
    mode_changed = Signal(str, bool)  # mode_name, success
    action_result = Signal(str, bool, str)  # action, success, detail
    geofence_result = Signal(bool, str)  # success, detail
    params_snapshot = Signal(object)  # dict[name] = value
    param_set_result = Signal(str, bool, str)  # name, success, detail
    error = Signal(str)
    log_line = Signal(str)

    def __init__(self, connection_string: str, timeout_s: float = 2.0, parent=None) -> None:
        super().__init__(parent)
        self._connection_string = connection_string
        self._timeout_s = timeout_s
        self._master: Optional[mavutil.mavlink_connection] = None
        self._running = False
        self._target_sysid = 1
        self._target_compid = 1
        self._cmd_lock = threading.Lock()
        self._cmd_queue: deque[tuple[str, object]] = deque()
        self._last_gcs_heartbeat_mono = 0.0
        self._streams_requested = False

    def queue_mission_upload(self, waypoints: list[dict]) -> None:
        with self._cmd_lock:
            self._cmd_queue.append(("mission_upload", waypoints))

    def queue_mission_download(self) -> None:
        with self._cmd_lock:
            self._cmd_queue.append(("mission_download", None))

    def queue_mission_start(self) -> None:
        with self._cmd_lock:
            self._cmd_queue.append(("mission_start", None))

    def queue_arm(self, arm: bool = True) -> None:
        with self._cmd_lock:
            self._cmd_queue.append(("arm", bool(arm)))

    def queue_mode_change(self, mode_name: str) -> None:
        with self._cmd_lock:
            self._cmd_queue.append(("mode_change", mode_name.strip()))

    def queue_takeoff(self, altitude_m: float) -> None:
        with self._cmd_lock:
            self._cmd_queue.append(("takeoff", float(altitude_m)))

    def queue_land(self) -> None:
        with self._cmd_lock:
            self._cmd_queue.append(("land", None))

    def queue_auto_takeoff(self, altitude_m: float) -> None:
        with self._cmd_lock:
            self._cmd_queue.append(("auto_takeoff", float(altitude_m)))

    def queue_auto_land(self) -> None:
        with self._cmd_lock:
            self._cmd_queue.append(("auto_land", None))

    def queue_geofence_upload(self, cfg: dict) -> None:
        with self._cmd_lock:
            self._cmd_queue.append(("geofence_upload", cfg))

    def queue_params_fetch(self, names: list[str]) -> None:
        with self._cmd_lock:
            self._cmd_queue.append(("params_fetch", list(names)))

    def queue_param_set(self, name: str, value: float) -> None:
        with self._cmd_lock:
            self._cmd_queue.append(("param_set", {"name": name, "value": float(value)}))

    def queue_preflight_calibration(self, kind: str) -> None:
        """Queue a sensor calibration request (best-effort, autopilot-dependent)."""
        with self._cmd_lock:
            self._cmd_queue.append(("preflight_calibration", str(kind or "").strip().lower()))

    def stop(self) -> None:
        self._running = False
        if self._master is not None:
            try:
                self._master.close()
            except Exception:
                pass
            self._master = None

    def run(self) -> None:
        self._running = True
        raw = self._connection_string.strip()

        # PyMAVLink often expects Windows serial as:
        #   mavutil.mavlink_connection("COM58", baud=115200)
        # Using "serial:COM58:115200" may behave differently across versions and can
        # lead to confusing parsing errors. We try the correct COM+baud form first.
        attempts: list[tuple[str, int | None]] = []

        # Normalize optional "serial:" prefix.
        trimmed = raw
        if raw.lower().startswith("serial:"):
            trimmed = raw[len("serial:") :]
        trimmed = trimmed.strip()
        # Accept friendly inputs like "COM58, 115200" / "COM58 : 115200".
        trimmed = re.sub(r"\s+", "", trimmed)

        m = re.match(r"^(COM\d+)[,:](\d+)$", trimmed, flags=re.IGNORECASE)
        if m:
            com = m.group(1).upper()
            baud = int(m.group(2))
            attempts = [
                (com, baud),
                (f"serial:{com}:{baud}", None),
                (f"serial:{com},{baud}", None),
            ]
        else:
            attempts = [(raw, None)]

        last_error: Exception | None = None
        first_error: Exception | None = None
        used_connection: str = raw

        for conn, baud in attempts:
            if baud is not None:
                self.log_line.emit(f"Using: {conn} baud={baud}")
            else:
                self.log_line.emit(f"Using: {conn}")

            try:
                kwargs = {"autoreconnect": True}
                if baud is not None:
                    kwargs["baud"] = baud
                self._master = mavutil.mavlink_connection(conn, **kwargs)
                used_connection = conn if baud is None else f"{conn}:{baud}"
                break
            except Exception as e:
                if first_error is None:
                    first_error = e
                last_error = e
                self.error.emit(str(e))
                self._master = None

        if self._master is None:
            preferred_error = first_error if first_error is not None else last_error
            hint = ""
            txt = str(preferred_error).lower() if preferred_error is not None else ""
            if any(k in txt for k in ("access is denied", "permission", "busy", "in use")):
                hint = " (port may be busy; close Mission Planner/QGC and retry)"
            self.error.emit(
                f"Failed to open MAVLink connection. First error: {preferred_error}{hint}"
            )
            self.link_down.emit()
            return

        self.log_line.emit(f"Opened: {used_connection}")

        self.link_up.emit()
        self.log_line.emit("Socket open; waiting for MAVLink telemetry...")
        try:
            self._send_gcs_heartbeat()
        except Exception:
            pass

        last_hb = 0.0
        last_msg = time.monotonic()
        timeout_notified = False
        while self._running and self._master is not None:
            self._process_pending_commands()
            self._maybe_send_gcs_heartbeat()
            try:
                msg = self._master.recv_match(
                    blocking=True,
                    timeout=1.0,
                )
            except Exception as e:
                if self._running:
                    self.error.emit(str(e))
                break

            if not self._running:
                break

            if msg is None:
                elapsed = time.monotonic() - last_msg
                if elapsed >= self._timeout_s and not timeout_notified:
                    timeout_notified = True
                    self.link_timeout.emit(elapsed)
                    self.log_line.emit(
                        f"Warning: no MAVLink messages for {elapsed:.1f}s"
                    )
                continue

            last_msg = time.monotonic()
            timeout_notified = False

            msg_type = msg.get_type()
            if msg_type == "HEARTBEAT":
                self._target_sysid = int(msg.get_srcSystem())
                self._target_compid = int(msg.get_srcComponent())
                now = time.monotonic()
                # Avoid flooding UI if vehicle sends HB fast
                if now - last_hb >= 0.5:
                    last_hb = now
                    self.heartbeat.emit(
                        int(msg.get_srcSystem()),
                        int(msg.get_srcComponent()),
                        int(getattr(msg, "mavlink_version", 0) or 0),
                    )
                self.log_line.emit(
                    f"HEARTBEAT sys={msg.get_srcSystem()} comp={msg.get_srcComponent()}"
                )
                armed = bool(getattr(msg, "base_mode", 0) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.telemetry.emit(
                    "HEARTBEAT",
                    {
                        "armed": armed,
                        "system_status": int(getattr(msg, "system_status", 0) or 0),
                        "custom_mode": int(getattr(msg, "custom_mode", 0) or 0),
                        "base_mode": int(getattr(msg, "base_mode", 0) or 0),
                        "vehicle_type": int(getattr(msg, "type", 0) or 0),
                        "autopilot": int(getattr(msg, "autopilot", 0) or 0),
                    },
                )
                if not self._streams_requested and int(self._target_sysid) > 0:
                    self._streams_requested = True
                    try:
                        self._request_telemetry_streams()
                    except Exception:
                        pass
            elif msg_type == "MISSION_CURRENT":
                self.telemetry.emit(
                    "MISSION_CURRENT",
                    {"seq": int(getattr(msg, "seq", 0) or 0)},
                )
            elif msg_type == "STATUSTEXT":
                self.telemetry.emit(
                    "STATUSTEXT",
                    {
                        "severity": int(getattr(msg, "severity", 0) or 0),
                        "text": str(getattr(msg, "text", "") or "").strip(),
                    },
                )
            elif msg_type == "VFR_HUD":
                self.telemetry.emit(
                    "VFR_HUD",
                    {
                        "airspeed": float(getattr(msg, "airspeed", 0.0) or 0.0),
                        "groundspeed": float(getattr(msg, "groundspeed", 0.0) or 0.0),
                        "heading": int(getattr(msg, "heading", 0) or 0),
                        "throttle": int(getattr(msg, "throttle", 0) or 0),
                        "climb": float(getattr(msg, "climb", 0.0) or 0.0),
                    },
                )
            elif msg_type == "ATTITUDE":
                self.telemetry.emit(
                    "ATTITUDE",
                    {
                        "roll_deg": float(getattr(msg, "roll", 0.0) or 0.0) * 57.2958,
                        "pitch_deg": float(getattr(msg, "pitch", 0.0) or 0.0) * 57.2958,
                        "yaw_deg": float(getattr(msg, "yaw", 0.0) or 0.0) * 57.2958,
                    },
                )
            elif msg_type == "GLOBAL_POSITION_INT":
                vx = float(getattr(msg, "vx", 0) or 0)
                vy = float(getattr(msg, "vy", 0) or 0)
                spd_cm = math.hypot(vx, vy)
                gt_deg: float | None = None
                if spd_cm > 50.0:
                    gt_deg = (math.degrees(math.atan2(vy, vx)) + 360.0) % 360.0
                raw_hdg = int(getattr(msg, "hdg", 0) or 0)
                hdg_deg: float | None = None
                if raw_hdg != 65535:
                    hdg_deg = (raw_hdg / 100.0) % 360.0
                gpi: dict[str, object] = {
                    "lat": float(getattr(msg, "lat", 0) or 0) / 1e7,
                    "lon": float(getattr(msg, "lon", 0) or 0) / 1e7,
                    "relative_alt_m": float(getattr(msg, "relative_alt", 0) or 0) / 1000.0,
                    "alt_msl_m": float(getattr(msg, "alt", 0) or 0) / 1000.0,
                }
                if hdg_deg is not None:
                    gpi["hdg_deg"] = hdg_deg
                if gt_deg is not None:
                    gpi["ground_track_deg"] = gt_deg
                self.telemetry.emit("GLOBAL_POSITION_INT", gpi)
            elif msg_type == "GPS_RAW_INT":
                eph_raw = int(getattr(msg, "eph", 0xFFFF) or 0xFFFF)
                hdop = None if eph_raw >= 0xFFFF else float(eph_raw) / 100.0
                self.telemetry.emit(
                    "GPS_RAW_INT",
                    {
                        "satellites_visible": int(getattr(msg, "satellites_visible", 0) or 0),
                        "fix_type": int(getattr(msg, "fix_type", 0) or 0),
                        "hdop": hdop,
                    },
                )
            elif msg_type == "SYS_STATUS":
                self.telemetry.emit(
                    "SYS_STATUS",
                    {
                        "voltage_v": float(getattr(msg, "voltage_battery", 0) or 0) / 1000.0,
                        "current_a": float(getattr(msg, "current_battery", -1) or -1) / 100.0,
                        "battery_remaining": int(getattr(msg, "battery_remaining", -1) or -1),
                        "sensors_present": int(
                            getattr(msg, "onboard_control_sensors_present", 0) or 0
                        ),
                        "sensors_enabled": int(
                            getattr(msg, "onboard_control_sensors_enabled", 0) or 0
                        ),
                        "sensors_health": int(
                            getattr(msg, "onboard_control_sensors_health", 0) or 0
                        ),
                    },
                )
            elif msg_type == "RADIO_STATUS":
                self.telemetry.emit(
                    "RADIO_STATUS",
                    {
                        "rssi": int(getattr(msg, "rssi", 0) or 0),
                        "remrssi": int(getattr(msg, "remrssi", 0) or 0),
                        "noise": int(getattr(msg, "noise", 0) or 0),
                        "remnoise": int(getattr(msg, "remnoise", 0) or 0),
                    },
                )
            elif msg_type.startswith("OPEN_DRONE_ID_"):
                payload: dict[str, object] = {}
                try:
                    raw = msg.to_dict()
                    if isinstance(raw, dict):
                        payload = raw
                except Exception:
                    payload = {}
                # Best-effort normalize of common byte-array identifier fields.
                for key in ("id_or_mac", "uas_id", "operator_id", "self_id"):
                    if key not in payload:
                        continue
                    value = payload.get(key)
                    if isinstance(value, (bytes, bytearray)):
                        payload[key] = value.decode("ascii", errors="ignore").strip("\x00 ").strip()
                    elif isinstance(value, list):
                        try:
                            payload[key] = bytes(int(v) & 0xFF for v in value).decode(
                                "ascii", errors="ignore"
                            ).strip("\x00 ").strip()
                        except Exception:
                            pass
                self.telemetry.emit(msg_type, payload)

        try:
            if self._master is not None:
                self._master.close()
        except Exception:
            pass
        self._master = None
        self.link_down.emit()
        self.log_line.emit("Link closed.")

    def _process_pending_commands(self) -> None:
        if self._master is None:
            return
        with self._cmd_lock:
            if not self._cmd_queue:
                return
            cmd, payload = self._cmd_queue.popleft()
        try:
            if cmd == "mission_upload":
                self._mission_upload(payload if isinstance(payload, list) else [])
            elif cmd == "mission_download":
                self._mission_download()
            elif cmd == "mission_start":
                self._mission_start()
            elif cmd == "arm":
                self._arm_disarm(bool(payload))
            elif cmd == "mode_change":
                self._mode_change(str(payload or ""))
            elif cmd == "takeoff":
                self._takeoff(float(payload or 0.0))
            elif cmd == "land":
                self._land()
            elif cmd == "auto_takeoff":
                self._auto_takeoff(float(payload or 0.0))
            elif cmd == "auto_land":
                self._auto_land()
            elif cmd == "geofence_upload":
                self._geofence_upload(payload if isinstance(payload, dict) else {})
            elif cmd == "params_fetch":
                self._params_fetch(payload if isinstance(payload, list) else [])
            elif cmd == "param_set":
                data = payload if isinstance(payload, dict) else {}
                self._param_set(str(data.get("name", "")), float(data.get("value", 0.0)))
            elif cmd == "preflight_calibration":
                self._preflight_calibration(str(payload or ""))
        except Exception as e:
            self.error.emit(f"Link command failed ({cmd}): {e}")

    def _preflight_calibration(self, kind: str) -> None:
        """
        Trigger MAV_CMD_PREFLIGHT_CALIBRATION with a single sensor bit set.

        Common ArduPilot mapping:
        - p1 gyro, p2 mag/compass, p3 baro, p4 rc, p5 accel
        """
        if self._master is None:
            self.action_result.emit("calibration", False, "Link not ready")
            self.error.emit("Calibration: link not ready")
            return
        k = (kind or "").strip().lower()
        p1 = p2 = p3 = p4 = p5 = 0.0
        label = "calibration"
        if k in ("gyro", "gyroscope"):
            p1 = 1.0
            label = "calibration_gyro"
        elif k in ("compass", "mag", "magnetometer"):
            p2 = 1.0
            label = "calibration_compass"
        elif k in ("baro", "barometer", "pressure"):
            p3 = 1.0
            label = "calibration_baro"
        elif k in ("rc", "radio"):
            p4 = 1.0
            label = "calibration_rc"
        elif k in ("accel", "accelerometer", "level"):
            p5 = 1.0
            label = "calibration_accel"
        else:
            self.action_result.emit("calibration", False, f"Unknown calibration: {kind}")
            self.error.emit(f"Calibration: unknown kind '{kind}'")
            return

        self._sync_link_targets()
        try:
            self._send_command_long(
                mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                p1=p1,
                p2=p2,
                p3=p3,
                p4=p4,
                p5=p5,
            )
            self.action_result.emit(label, True, "Command sent")
            self.log_line.emit(f"Calibration command sent: {label}")
        except Exception as e:
            self.action_result.emit(label, False, str(e))
            self.error.emit(f"Calibration failed ({label}): {e}")

    def _sync_link_targets(self) -> None:
        """Align pymavlink routing with the last vehicle HEARTBEAT (sys/comp)."""
        if self._master is None:
            return
        if self._target_sysid > 0:
            self._master.target_system = self._target_sysid
        self._master.target_component = self._target_compid

    def _send_gcs_heartbeat(self) -> None:
        if self._master is None:
            return
        self._master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )
        self._last_gcs_heartbeat_mono = time.monotonic()

    def _maybe_send_gcs_heartbeat(self, interval_s: float = 1.0) -> None:
        """ArduPilot/MAVProxy expect a GCS HEARTBEAT; mission handshakes can stall without it."""
        if self._master is None:
            return
        now = time.monotonic()
        if now - self._last_gcs_heartbeat_mono < interval_s:
            return
        try:
            self._send_gcs_heartbeat()
        except Exception:
            pass

    def _request_telemetry_streams(self) -> None:
        """Ask the vehicle for telemetry needed by VGCS UI.

        Note: real vehicles often require MAV_CMD_SET_MESSAGE_INTERVAL to reliably stream
        SYS_STATUS / GPS_RAW_INT / BATTERY_STATUS; relying only on request_data_stream_send
        can yield HEARTBEAT/STATUSTEXT only on some firmwares/configs.
        """
        if self._master is None:
            return
        self._sync_link_targets()
        ts = int(self._target_sysid)
        tc = int(self._target_compid)
        hz = 5
        try:
            self._master.mav.request_data_stream_send(
                ts,
                tc,
                mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                hz,
                1,
            )
            self._master.mav.request_data_stream_send(
                ts,
                tc,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
                hz,
                1,
            )
            # Extended status includes SYS_STATUS on many stacks (battery remaining, etc.)
            self._master.mav.request_data_stream_send(
                ts,
                tc,
                mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
                max(1, min(2, hz)),
                1,
            )
            # EXTRA2 commonly includes GPS_RAW_INT on ArduPilot/PX4.
            self._master.mav.request_data_stream_send(
                ts,
                tc,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,
                max(1, min(2, hz)),
                1,
            )
            self.log_line.emit(
                f"Requested MAV_DATA_STREAM_POSITION/EXTRA1 @ {hz} Hz (+EXT_STATUS/EXTRA2 @ {max(1, min(2, hz))} Hz)"
            )
        except Exception as e:
            self.log_line.emit(f"request_data_stream_send failed: {e}")
        try:
            interval_us = int(1_000_000 / max(1, hz))
            slow_interval_us = int(1_000_000 / 1)  # 1 Hz for heavy/slow-changing messages
            self._master.mav.command_long_send(
                ts,
                tc,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                float(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT),
                float(interval_us),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )
            self._master.mav.command_long_send(
                ts,
                tc,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                float(mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT),
                float(interval_us),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )
            # Battery/GPS UI depends on these; request explicitly for real vehicles.
            self._master.mav.command_long_send(
                ts,
                tc,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                float(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS),
                float(slow_interval_us),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )
            self._master.mav.command_long_send(
                ts,
                tc,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                float(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT),
                float(slow_interval_us),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )
            self._master.mav.command_long_send(
                ts,
                tc,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                float(mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS),
                float(slow_interval_us),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )
        except Exception:
            pass

    def _mission_clear_for_upload_best_effort(self) -> None:
        """Clear mission on vehicle before upload; avoids half-open mission sessions."""
        if self._master is None:
            return
        self._sync_link_targets()
        try:
            self._master.mav.mission_clear_all_send(
                self._target_sysid,
                self._target_compid,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )
        except TypeError:
            self._master.mav.mission_clear_all_send(
                self._target_sysid,
                self._target_compid,
            )
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and self._running and self._master is not None:
            self._maybe_send_gcs_heartbeat()
            ack = self._master.recv_match(
                type=["MISSION_ACK"],
                blocking=True,
                timeout=0.35,
            )
            if ack is None:
                continue
            mt = int(getattr(ack, "mission_type", 0) or 0)
            if mt != int(mavutil.mavlink.MAV_MISSION_TYPE_MISSION):
                continue
            break

    def _mission_upload(self, waypoints: list[dict]) -> None:
        if self._master is None:
            self.error.emit("Mission upload: link not ready")
            return
        if not waypoints:
            self.error.emit("Mission upload: no waypoints")
            return

        # ArduPilot AP_Mission: MAVLink index 0 is reserved for HOME — uploads to seq 0
        # are not stored as mission nav commands. The first real nav command is index 1
        # (AP_MISSION_FIRST_REAL_COMMAND). Put NAV_TAKEOFF at seq 1 or AUTO reports
        # "Missing Takeoff Cmd".
        mission_items: list[dict] = []
        first = waypoints[0]
        first_nav_alt = max(1.0, float(first.get("alt_m", 20.0)))
        # Optional: climb target for NAV_TAKEOFF (start / launch altitude), distinct from WP1 cruise alt.
        to_m = first.get("takeoff_alt_m", None)
        if to_m is None:
            takeoff_alt_m = first_nav_alt
        else:
            takeoff_alt_m = max(1.0, float(to_m))
        mission_items.append(
            {
                "cmd": int(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT),
                "lat": float(first.get("lat", 0.0)),
                "lon": float(first.get("lon", 0.0)),
                "alt_m": first_nav_alt,
            }
        )
        mission_items.append(
            {
                "cmd": int(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF),
                "lat": 0.0,
                "lon": 0.0,
                "alt_m": takeoff_alt_m,
            }
        )
        for wp in waypoints:
            spd = float(wp.get("speed_mps", 5.0) or 5.0)
            mission_items.append(
                {
                    "cmd": int(mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED),
                    "lat": 0.0,
                    "lon": 0.0,
                    "alt_m": 0.0,
                    # param1: 1=groundspeed, param2: speed (m/s), param3: throttle (-1 no change)
                    "p1": 1.0,
                    "p2": max(0.1, spd),
                    "p3": -1.0,
                    "p4": 0.0,
                }
            )
            mission_items.append(
                {
                    "cmd": int(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT),
                    "lat": float(wp.get("lat", 0.0)),
                    "lon": float(wp.get("lon", 0.0)),
                    "alt_m": max(1.0, float(wp.get("alt_m", 20.0))),
                    "p1": 0.0,
                    "p2": 0.0,
                    "p3": 0.0,
                    "p4": 0.0,
                }
            )

        count = len(mission_items)
        self.log_line.emit(
            f"Mission upload start: {len(waypoints)} WPs ({count} mission items incl. home slot + takeoff)"
        )
        self._sync_link_targets()
        self._mission_clear_for_upload_best_effort()

        def send_mission_count() -> None:
            try:
                self._master.mav.mission_count_send(
                    self._target_sysid,
                    self._target_compid,
                    count,
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                )
            except TypeError:
                self._master.mav.mission_count_send(
                    self._target_sysid,
                    self._target_compid,
                    count,
                )

        send_mission_count()

        sent = 0
        started = time.monotonic()
        last_count_tx = time.monotonic()
        while sent < count and self._running and self._master is not None:
            self._maybe_send_gcs_heartbeat()
            now = time.monotonic()
            if sent == 0 and (now - last_count_tx) >= 2.0:
                send_mission_count()
                last_count_tx = now
                self.log_line.emit("Mission upload: resend MISSION_COUNT (waiting vehicle request)")

            req = self._master.recv_match(
                type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
                blocking=True,
                timeout=1.0,
            )
            if req is None:
                if time.monotonic() - started > 25.0:
                    raise TimeoutError("mission upload timeout waiting request")
                continue

            if req.get_type() == "MISSION_ACK":
                mt = int(getattr(req, "mission_type", 0) or 0)
                if mt != int(mavutil.mavlink.MAV_MISSION_TYPE_MISSION):
                    continue
                ack_type = int(getattr(req, "type", mavutil.mavlink.MAV_MISSION_ERROR))
                if ack_type != int(mavutil.mavlink.MAV_MISSION_ACCEPTED):
                    raise RuntimeError(
                        f"mission upload rejected (MAV_MISSION_RESULT={ack_type})"
                    )
                # Stray ACCEPTED (e.g. late clear ACK) — keep waiting for MISSION_REQUEST.
                continue

            seq = int(getattr(req, "seq", sent) or sent)
            if seq < 0 or seq >= count:
                continue
            item = mission_items[seq]
            lat = float(item.get("lat", 0.0))
            lon = float(item.get("lon", 0.0))
            alt = float(item.get("alt_m", 20.0))
            cmd = int(item.get("cmd", int(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT)))
            p1 = float(item.get("p1", 0.0))
            p2 = float(item.get("p2", 0.0))
            p3 = float(item.get("p3", 0.0))
            p4 = float(item.get("p4", 0.0))
            try:
                self._master.mav.mission_item_int_send(
                    self._target_sysid,
                    self._target_compid,
                    seq,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    cmd,
                    0,
                    1,
                    p1,
                    p2,
                    p3,
                    p4,
                    int(lat * 1e7),
                    int(lon * 1e7),
                    alt,
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                )
            except TypeError:
                self._master.mav.mission_item_int_send(
                    self._target_sysid,
                    self._target_compid,
                    seq,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    cmd,
                    0,
                    1,
                    p1,
                    p2,
                    p3,
                    p4,
                    int(lat * 1e7),
                    int(lon * 1e7),
                    alt,
                )
            sent = max(sent, seq + 1)

        ack = self._master.recv_match(
            type=["MISSION_ACK"],
            blocking=True,
            timeout=5.0,
        )
        if ack is None:
            raise TimeoutError("mission upload timeout waiting ACK")
        ack_type = int(getattr(ack, "type", mavutil.mavlink.MAV_MISSION_ERROR))
        if ack_type != int(mavutil.mavlink.MAV_MISSION_ACCEPTED):
            raise RuntimeError(f"mission upload ACK type={ack_type}")
        self.log_line.emit(f"Mission upload complete: {count} mission items")
        self.mission_uploaded.emit(len(waypoints))

    def _mission_download(self) -> None:
        if self._master is None:
            self.error.emit("Mission download: link not ready")
            return
        self.log_line.emit("Mission download start")
        try:
            self._master.mav.mission_request_list_send(
                self._target_sysid,
                self._target_compid,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )
        except TypeError:
            self._master.mav.mission_request_list_send(
                self._target_sysid,
                self._target_compid,
            )
        cnt_msg = self._master.recv_match(
            type=["MISSION_COUNT"],
            blocking=True,
            timeout=5.0,
        )
        if cnt_msg is None:
            raise TimeoutError("mission download timeout waiting count")
        count = int(getattr(cnt_msg, "count", 0) or 0)
        items: list[dict] = []
        for seq in range(count):
            try:
                self._master.mav.mission_request_int_send(
                    self._target_sysid,
                    self._target_compid,
                    seq,
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                )
            except TypeError:
                self._master.mav.mission_request_send(
                    self._target_sysid,
                    self._target_compid,
                    seq,
                )
            itm = self._master.recv_match(
                type=["MISSION_ITEM_INT", "MISSION_ITEM"],
                blocking=True,
                timeout=3.0,
            )
            if itm is None:
                raise TimeoutError(f"mission download timeout seq={seq}")
            if itm.get_type() == "MISSION_ITEM_INT":
                lat = float(getattr(itm, "x", 0)) / 1e7
                lon = float(getattr(itm, "y", 0)) / 1e7
                alt = float(getattr(itm, "z", 20.0))
            else:
                lat = float(getattr(itm, "x", 0.0))
                lon = float(getattr(itm, "y", 0.0))
                alt = float(getattr(itm, "z", 20.0))
            items.append({"lat": lat, "lon": lon, "alt_m": alt})
        try:
            self._master.mav.mission_ack_send(
                self._target_sysid,
                self._target_compid,
                mavutil.mavlink.MAV_MISSION_ACCEPTED,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )
        except TypeError:
            self._master.mav.mission_ack_send(
                self._target_sysid,
                self._target_compid,
                mavutil.mavlink.MAV_MISSION_ACCEPTED,
            )
        self.log_line.emit(f"Mission download complete: {len(items)} WPs")
        self.mission_downloaded.emit(items)

    def _mode_change(self, mode_name: str) -> None:
        if self._master is None:
            self.mode_changed.emit(mode_name, False)
            self.error.emit("Mode change: link not ready")
            return
        if not mode_name:
            self.mode_changed.emit(mode_name, False)
            self.error.emit("Mode change: empty mode")
            return
        try:
            self._master.set_mode(mode_name)
            self.log_line.emit(f"Mode change requested: {mode_name}")
            self.mode_changed.emit(mode_name, True)
        except Exception as e:
            self.mode_changed.emit(mode_name, False)
            self.error.emit(f"Mode change failed: {e}")

    def _ensure_armable_mode_before_arm(self) -> None:
        """ArduCopter rejects arming in AUTO; switch to a manual mode first (SITL/GCS)."""
        if self._master is None:
            return
        for mode in ("STABILIZE", "ALT_HOLD", "LOITER"):
            try:
                self._master.set_mode(mode)
                self.log_line.emit(f"Pre-arm: switched to {mode} (required to arm)")
                time.sleep(0.15)
                return
            except Exception:
                continue

    def _wait_vehicle_armed(self, timeout_s: float = 8.0) -> bool:
        """Poll HEARTBEAT until vehicle reports SAFETY_ARMED or timeout."""
        if self._master is None:
            return False
        deadline = time.monotonic() + timeout_s
        while self._running and self._master is not None and time.monotonic() < deadline:
            self._maybe_send_gcs_heartbeat()
            msg = self._master.recv_match(
                type=["HEARTBEAT"],
                blocking=True,
                timeout=0.35,
            )
            if msg is None:
                continue
            if int(msg.get_srcSystem()) != int(self._target_sysid):
                continue
            if bool(getattr(msg, "base_mode", 0) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                return True
        return False

    def _mission_start(self) -> None:
        if self._master is None:
            self.action_result.emit("mission_start", False, "Link not ready")
            self.error.emit("Mission start: link not ready")
            return
        try:
            self._sync_link_targets()
            self._ensure_armable_mode_before_arm()
            self._send_command_long(
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                p1=1.0,
            )
            if not self._wait_vehicle_armed():
                # ArduPilot SITL: force-arm magic in param2 when pre-arm checks block normal arm.
                self.log_line.emit("Mission start: retry ARM with force override (param2=21196)")
                self._send_command_long(
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    p1=1.0,
                    p2=21196.0,
                )
                if not self._wait_vehicle_armed(6.0):
                    raise TimeoutError("arm timeout (still disarmed after arm command)")
            # Start at first *stored* nav command (seq 1); seq 0 is home-only in ArduPilot.
            self._master.mav.mission_set_current_send(
                self._target_sysid,
                self._target_compid,
                1,
            )
            self._master.set_mode("AUTO")
            self._send_command_long(
                mavutil.mavlink.MAV_CMD_MISSION_START,
                p1=0.0,
                p2=0.0,
            )
            self.action_result.emit("mission_start", True, "AUTO mission start sent")
            self.log_line.emit("Mission start command sent (AUTO)")
        except Exception as e:
            self.action_result.emit("mission_start", False, str(e))
            self.error.emit(f"Mission start failed: {e}")

    def _arm_disarm(self, arm: bool) -> None:
        if self._master is None:
            self.action_result.emit("arm", False, "Link not ready")
            self.error.emit("Arm: link not ready")
            return
        self._sync_link_targets()
        try:
            if arm:
                self._ensure_armable_mode_before_arm()
            self._send_command_long(
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                p1=1.0 if arm else 0.0,
            )
            self.action_result.emit("arm", True, "ARM sent" if arm else "DISARM sent")
            self.log_line.emit("Arm command sent" if arm else "Disarm command sent")
        except Exception as e:
            self.action_result.emit("arm", False, str(e))
            self.error.emit(f"Arm failed: {e}")

    def _send_command_long(
        self,
        command: int,
        *,
        p1: float = 0.0,
        p2: float = 0.0,
        p3: float = 0.0,
        p4: float = 0.0,
        p5: float = 0.0,
        p6: float = 0.0,
        p7: float = 0.0,
    ) -> None:
        if self._master is None:
            raise RuntimeError("Link not ready")
        self._master.mav.command_long_send(
            self._target_sysid,
            self._target_compid,
            command,
            0,
            p1,
            p2,
            p3,
            p4,
            p5,
            p6,
            p7,
        )

    def _send_nav_takeoff(self, altitude_m: float) -> None:
        alt = max(1.0, float(altitude_m))
        self._sync_link_targets()
        self._send_command_long(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            p7=alt,
        )

    def _takeoff(self, altitude_m: float) -> None:
        alt = max(1.0, float(altitude_m))
        try:
            self._send_nav_takeoff(alt)
            self.action_result.emit("takeoff", True, f"Target alt {alt:.1f} m")
            self.log_line.emit(f"Takeoff command sent: alt={alt:.1f}m")
        except Exception as e:
            self.action_result.emit("takeoff", False, str(e))
            self.error.emit(f"Takeoff failed: {e}")

    def _send_nav_land(self) -> None:
        self._sync_link_targets()
        self._send_command_long(mavutil.mavlink.MAV_CMD_NAV_LAND)

    def _land(self) -> None:
        try:
            self._send_nav_land()
            self.action_result.emit("land", True, "Landing command sent")
            self.log_line.emit("Land command sent")
        except Exception as e:
            self.action_result.emit("land", False, str(e))
            self.error.emit(f"Land failed: {e}")

    def _auto_takeoff(self, altitude_m: float) -> None:
        if self._master is None:
            self.action_result.emit("auto_takeoff", False, "Link not ready")
            self.error.emit("Auto takeoff: link not ready")
            return
        try:
            self._sync_link_targets()
            self._ensure_armable_mode_before_arm()
            self._send_command_long(
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                p1=1.0,
            )
            if not self._wait_vehicle_armed():
                self.log_line.emit("Auto takeoff: retry ARM with force override (param2=21196)")
                self._send_command_long(
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    p1=1.0,
                    p2=21196.0,
                )
                if not self._wait_vehicle_armed(6.0):
                    raise TimeoutError("arm timeout (still disarmed after arm command)")
            alt = max(1.0, float(altitude_m))
            self._send_nav_takeoff(alt)
            self.action_result.emit("auto_takeoff", True, f"Armed + takeoff {alt:.1f} m")
            self.log_line.emit(f"Auto takeoff: armed + NAV_TAKEOFF {alt:.1f} m")
        except Exception as e:
            self.action_result.emit("auto_takeoff", False, str(e))
            self.error.emit(f"Auto takeoff failed: {e}")

    def _auto_land(self) -> None:
        if self._master is None:
            self.action_result.emit("auto_land", False, "Link not ready")
            self.error.emit("Auto land: link not ready")
            return
        self._sync_link_targets()
        try:
            self._master.set_mode("LAND")
            self.action_result.emit("auto_land", True, "LAND mode engaged")
            self.log_line.emit("Auto land: LAND mode")
        except Exception as e:
            self.log_line.emit(f"Auto land: LAND mode failed ({e}); sending NAV_LAND")
            try:
                self._send_nav_land()
                self.action_result.emit("auto_land", True, "NAV_LAND command sent")
                self.log_line.emit("Auto land: NAV_LAND sent")
            except Exception as e2:
                self.action_result.emit("auto_land", False, str(e2))
                self.error.emit(f"Auto land failed: {e2}")

    def _param_set_value(self, name: str, value: float) -> None:
        if self._master is None:
            raise RuntimeError("Link not ready")
        self._master.mav.param_set_send(
            self._target_sysid,
            self._target_compid,
            name.encode("ascii", "ignore")[:16],
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )

    def _geofence_upload(self, cfg: dict) -> None:
        if bool(cfg.get("disable")):
            try:
                self._param_set_value("FENCE_ENABLE", 0.0)
                self.geofence_result.emit(True, "Fence disabled")
                return
            except Exception as e:
                self.geofence_result.emit(False, str(e))
                self.error.emit(f"Geofence disable failed: {e}")
                return

        points = cfg.get("points")
        if isinstance(points, list) and len(points) >= 3:
            try:
                self._upload_polygon_fence(points)
                self.geofence_result.emit(True, f"Polygon uploaded ({len(points)} pts)")
                return
            except Exception as e:
                self.error.emit(f"Polygon fence upload failed; fallback to circle: {e}")

        # M2 subset fallback: configure ArduPilot circular fence parameters.
        radius_m = max(10.0, float(cfg.get("radius_m", 80.0)))
        alt_max_m = max(5.0, float(cfg.get("alt_max_m", 120.0)))
        try:
            self._param_set_value("FENCE_ENABLE", 1.0)
            self._param_set_value("FENCE_TYPE", 2.0)  # circular fence
            self._param_set_value("FENCE_RADIUS", radius_m)
            self._param_set_value("FENCE_ALT_MAX", alt_max_m)
            self.geofence_result.emit(
                True,
                f"Fence enabled: radius={radius_m:.0f}m alt_max={alt_max_m:.0f}m",
            )
            self.log_line.emit(
                f"Geofence configured (radius={radius_m:.0f}m, alt={alt_max_m:.0f}m)"
            )
        except Exception as e:
            self.geofence_result.emit(False, str(e))
            self.error.emit(f"Geofence upload failed: {e}")

    def _upload_polygon_fence(self, points: list[object]) -> None:
        if self._master is None:
            raise RuntimeError("Link not ready")
        norm: list[tuple[float, float]] = []
        for row in points:
            if not (isinstance(row, list) or isinstance(row, tuple)) or len(row) < 2:
                continue
            lat = float(row[0])
            lon = float(row[1])
            norm.append((lat, lon))
        if len(norm) < 3:
            raise RuntimeError("Polygon requires >=3 points")
        self._param_set_value("FENCE_ENABLE", 1.0)
        self._param_set_value("FENCE_TYPE", 4.0)  # polygon fence
        self._param_set_value("FENCE_TOTAL", float(len(norm)))
        for idx, (lat, lon) in enumerate(norm):
            self._master.mav.fence_point_send(
                self._target_sysid,
                self._target_compid,
                idx,
                len(norm),
                lat,
                lon,
            )
            time.sleep(0.02)
        self.log_line.emit(f"Polygon geofence points sent: {len(norm)}")

    def _params_fetch(self, names: list[str]) -> None:
        if self._master is None:
            self.error.emit("Params fetch: link not ready")
            return
        out: dict[str, float] = {}
        for raw in names:
            name = str(raw).strip().upper()[:16]
            if not name:
                continue
            try:
                self._master.mav.param_request_read_send(
                    self._target_sysid,
                    self._target_compid,
                    name.encode("ascii", "ignore"),
                    -1,
                )
                msg = self._master.recv_match(
                    type=["PARAM_VALUE"], blocking=True, timeout=1.0
                )
                if msg is None:
                    continue
                got_name = str(getattr(msg, "param_id", "") or "").strip("\x00").upper()
                got_val = float(getattr(msg, "param_value", 0.0) or 0.0)
                if got_name:
                    out[got_name] = got_val
            except Exception:
                continue
        self.params_snapshot.emit(out)
        self.log_line.emit(f"Param fetch complete: {len(out)} values")

    def _param_set(self, name: str, value: float) -> None:
        param = str(name).strip().upper()[:16]
        if not param:
            self.param_set_result.emit(name, False, "Empty param name")
            return
        try:
            self._param_set_value(param, float(value))
            self.param_set_result.emit(param, True, f"{value}")
            self.log_line.emit(f"Param set: {param}={value}")
        except Exception as e:
            self.param_set_result.emit(param, False, str(e))
            self.error.emit(f"Param set failed: {param}: {e}")
