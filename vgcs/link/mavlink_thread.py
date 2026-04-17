"""
Background MAVLink reader for the GCS link layer.

Runs blocking pymavlink I/O off the GUI thread and emits decoded telemetry.
"""

from __future__ import annotations

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

    def queue_mission_upload(self, waypoints: list[dict]) -> None:
        with self._cmd_lock:
            self._cmd_queue.append(("mission_upload", waypoints))

    def queue_mission_download(self) -> None:
        with self._cmd_lock:
            self._cmd_queue.append(("mission_download", None))

    def queue_mode_change(self, mode_name: str) -> None:
        with self._cmd_lock:
            self._cmd_queue.append(("mode_change", mode_name.strip()))

    def queue_takeoff(self, altitude_m: float) -> None:
        with self._cmd_lock:
            self._cmd_queue.append(("takeoff", float(altitude_m)))

    def queue_land(self) -> None:
        with self._cmd_lock:
            self._cmd_queue.append(("land", None))

    def queue_geofence_upload(self, cfg: dict) -> None:
        with self._cmd_lock:
            self._cmd_queue.append(("geofence_upload", cfg))

    def queue_params_fetch(self, names: list[str]) -> None:
        with self._cmd_lock:
            self._cmd_queue.append(("params_fetch", list(names)))

    def queue_param_set(self, name: str, value: float) -> None:
        with self._cmd_lock:
            self._cmd_queue.append(("param_set", {"name": name, "value": float(value)}))

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

        last_hb = 0.0
        last_msg = time.monotonic()
        timeout_notified = False
        while self._running and self._master is not None:
            self._process_pending_commands()
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
                self.telemetry.emit(
                    "GLOBAL_POSITION_INT",
                    {
                        "lat": float(getattr(msg, "lat", 0) or 0) / 1e7,
                        "lon": float(getattr(msg, "lon", 0) or 0) / 1e7,
                        "relative_alt_m": float(getattr(msg, "relative_alt", 0) or 0) / 1000.0,
                        "alt_msl_m": float(getattr(msg, "alt", 0) or 0) / 1000.0,
                    },
                )
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
            elif cmd == "mode_change":
                self._mode_change(str(payload or ""))
            elif cmd == "takeoff":
                self._takeoff(float(payload or 0.0))
            elif cmd == "land":
                self._land()
            elif cmd == "geofence_upload":
                self._geofence_upload(payload if isinstance(payload, dict) else {})
            elif cmd == "params_fetch":
                self._params_fetch(payload if isinstance(payload, list) else [])
            elif cmd == "param_set":
                data = payload if isinstance(payload, dict) else {}
                self._param_set(str(data.get("name", "")), float(data.get("value", 0.0)))
        except Exception as e:
            self.error.emit(f"Link command failed ({cmd}): {e}")

    def _mission_upload(self, waypoints: list[dict]) -> None:
        if self._master is None:
            self.error.emit("Mission upload: link not ready")
            return
        if not waypoints:
            self.error.emit("Mission upload: no waypoints")
            return

        count = len(waypoints)
        self.log_line.emit(f"Mission upload start: {count} WPs")
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

        sent = 0
        started = time.monotonic()
        while sent < count and self._running and self._master is not None:
            req = self._master.recv_match(
                type=["MISSION_REQUEST_INT", "MISSION_REQUEST"],
                blocking=True,
                timeout=2.0,
            )
            if req is None:
                if time.monotonic() - started > 20.0:
                    raise TimeoutError("mission upload timeout waiting request")
                continue
            seq = int(getattr(req, "seq", sent) or sent)
            if seq < 0 or seq >= count:
                continue
            wp = waypoints[seq]
            lat = float(wp.get("lat", 0.0))
            lon = float(wp.get("lon", 0.0))
            alt = float(wp.get("alt_m", 20.0))
            try:
                self._master.mav.mission_item_int_send(
                    self._target_sysid,
                    self._target_compid,
                    seq,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    1 if seq == 0 else 0,
                    1,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
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
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    1 if seq == 0 else 0,
                    1,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    int(lat * 1e7),
                    int(lon * 1e7),
                    alt,
                )
            sent = max(sent, seq + 1)

        ack = self._master.recv_match(
            type=["MISSION_ACK"],
            blocking=True,
            timeout=3.0,
        )
        if ack is None:
            raise TimeoutError("mission upload timeout waiting ACK")
        ack_type = int(getattr(ack, "type", mavutil.mavlink.MAV_MISSION_ERROR))
        if ack_type != int(mavutil.mavlink.MAV_MISSION_ACCEPTED):
            raise RuntimeError(f"mission upload ACK type={ack_type}")
        self.log_line.emit(f"Mission upload complete: {count} WPs")
        self.mission_uploaded.emit(count)

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

    def _takeoff(self, altitude_m: float) -> None:
        alt = max(1.0, float(altitude_m))
        try:
            self._send_command_long(
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                p7=alt,
            )
            self.action_result.emit("takeoff", True, f"Target alt {alt:.1f} m")
            self.log_line.emit(f"Takeoff command sent: alt={alt:.1f}m")
        except Exception as e:
            self.action_result.emit("takeoff", False, str(e))
            self.error.emit(f"Takeoff failed: {e}")

    def _land(self) -> None:
        try:
            self._send_command_long(mavutil.mavlink.MAV_CMD_NAV_LAND)
            self.action_result.emit("land", True, "Landing command sent")
            self.log_line.emit("Land command sent")
        except Exception as e:
            self.action_result.emit("land", False, str(e))
            self.error.emit(f"Land failed: {e}")

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
        # M2 subset: configure ArduPilot circular fence parameters.
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
