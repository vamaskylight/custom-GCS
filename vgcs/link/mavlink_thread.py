"""
Background MAVLink reader for the GCS link layer.

Runs blocking pymavlink I/O off the GUI thread and emits decoded telemetry.
"""

from __future__ import annotations

import re
import time
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
    error = Signal(str)
    log_line = Signal(str)

    def __init__(self, connection_string: str, timeout_s: float = 2.0, parent=None) -> None:
        super().__init__(parent)
        self._connection_string = connection_string
        self._timeout_s = timeout_s
        self._master: Optional[mavutil.mavlink_connection] = None
        self._running = False

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
        # Normalize common serial formats so pymavlink always parses correctly.
        # Some environments treat `COM58:115200` as a network endpoint, causing:
        #   bind(): port must be 0-65535
        # Fix: convert to `serial:COM58:115200`.
        connection_string = self._connection_string.strip()
        m = re.match(r"^(COM\d+):(\d+)$", connection_string, flags=re.IGNORECASE)
        if m:
            connection_string = f"serial:{m.group(1)}:{m.group(2)}"
        self.log_line.emit(f"Using: {connection_string}")
        try:
            self.log_line.emit(f"Opening: {connection_string}")
            self._master = mavutil.mavlink_connection(
                connection_string,
                autoreconnect=True,
            )
        except Exception as e:
            self.error.emit(str(e))
            self.link_down.emit()
            return

        self.link_up.emit()
        self.log_line.emit("Socket open; waiting for MAVLink telemetry...")

        last_hb = 0.0
        last_msg = time.monotonic()
        timeout_notified = False
        while self._running and self._master is not None:
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
