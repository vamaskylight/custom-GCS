"""
Background MAVLink reader: connect, recv HEARTBEAT, emit status.

Runs blocking pymavlink I/O off the GUI thread.
"""

from __future__ import annotations

import time
from typing import Optional

from PySide6.QtCore import QThread, Signal
from pymavlink import mavutil


class MavlinkThread(QThread):
    """Connect to a MAVLink stream and report link + HEARTBEAT state."""

    link_up = Signal()
    link_down = Signal()
    heartbeat = Signal(int, int, int)  # system_id, component_id, mavlink_version
    error = Signal(str)
    log_line = Signal(str)

    def __init__(self, connection_string: str, parent=None) -> None:
        super().__init__(parent)
        self._connection_string = connection_string
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
        try:
            self.log_line.emit(f"Opening: {self._connection_string}")
            self._master = mavutil.mavlink_connection(
                self._connection_string,
                autoreconnect=True,
            )
        except Exception as e:
            self.error.emit(str(e))
            self.link_down.emit()
            return

        self.link_up.emit()
        self.log_line.emit("Socket open; waiting for HEARTBEAT…")

        last_hb = 0.0
        while self._running and self._master is not None:
            try:
                msg = self._master.recv_match(
                    type=["HEARTBEAT"],
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
                continue

            if msg.get_type() == "HEARTBEAT":
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

        try:
            if self._master is not None:
                self._master.close()
        except Exception:
            pass
        self._master = None
        self.link_down.emit()
        self.log_line.emit("Link closed.")
