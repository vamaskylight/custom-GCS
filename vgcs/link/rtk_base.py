"""Read RTK corrections from a base station (milestone M11, "RTK module connection").

The base is on a serial port of this PC (USB), or on the network. It sends
RTCM 3 messages. This thread reads them, checks each one, and hands every whole
message on. It never talks to the drone itself: the main window passes the
messages to the MAVLink link.

The base must already be set up to send RTCM 3 (and its own position, message
1005 or 1006). Setting up a base, for example a u-blox survey-in, is not done
here.
"""

from __future__ import annotations

import re
import socket
import time
from dataclasses import dataclass

from PySide6.QtCore import QThread, Signal

from vgcs.link.rtcm import Rtcm3Parser, station_position

DEFAULT_BAUD = 115200
_RETRY_S = 3.0
_READ_TIMEOUT_S = 0.5


@dataclass(frozen=True)
class BaseSource:
    """Where the base station is: a serial port with its speed, or a network address."""

    kind: str          # "serial" or "tcp"
    address: str       # "COM7" or "192.168.1.50"
    number: int        # baud rate or TCP port

    def label(self) -> str:
        return f"{self.address} at {self.number} baud" if self.kind == "serial" else f"{self.address}:{self.number}"


def parse_source(text: object) -> BaseSource | None:
    """Read what the operator typed. Returns None when it is not a base address.

    Accepted: ``COM7``, ``COM7:115200``, ``COM7,115200``, ``/dev/ttyUSB0:57600``,
    ``tcp:192.168.1.50:2101`` and ``192.168.1.50:2101``.
    """
    raw = re.sub(r"\s+", "", str(text or ""))
    if not raw:
        return None
    lowered = raw.lower()
    if lowered.startswith("serial:"):
        raw = raw[len("serial:"):]
        lowered = raw.lower()
    if lowered.startswith("tcp:"):
        found = re.fullmatch(r"([^:]+):(\d{1,5})", raw[4:])
        if found and 0 < int(found.group(2)) < 65536:
            return BaseSource("tcp", found.group(1), int(found.group(2)))
        return None
    found = re.fullmatch(r"(com\d+|/dev/[\w./-]+)(?:[:,](\d+))?", raw, flags=re.IGNORECASE)
    if found:
        port = found.group(1)
        if port.lower().startswith("com"):
            port = port.upper()
        return BaseSource("serial", port, int(found.group(2) or DEFAULT_BAUD))
    found = re.fullmatch(r"([^:/,]+):(\d{1,5})", raw)
    if found and 0 < int(found.group(2)) < 65536:
        return BaseSource("tcp", found.group(1), int(found.group(2)))
    return None


class RtkBaseThread(QThread):
    """Keep reading the base station, reconnecting when it goes away."""

    # One whole RTCM 3 message (header and checksum included), and its type number.
    corrections = Signal(bytes, int)
    # The base's own position: latitude, longitude, height in metres.
    station = Signal(float, float, float)
    # "connecting", "connected" or "error", and for an error its reason.
    state = Signal(str, str)

    def __init__(self, source: BaseSource, parent=None) -> None:
        super().__init__(parent)
        self._source = source
        self._running = False
        self._port = None

    def source(self) -> BaseSource:
        return self._source

    def stop(self) -> None:
        self._running = False
        port = self._port
        if port is not None:
            try:
                port.close()
            except Exception:
                pass

    # -- the two kinds of port, behind one small interface ---------------
    def _open(self):
        src = self._source
        if src.kind == "serial":
            import serial  # pyserial, already needed by pymavlink

            return serial.Serial(src.address, src.number, timeout=_READ_TIMEOUT_S)
        sock = socket.create_connection((src.address, src.number), timeout=5.0)
        sock.settimeout(_READ_TIMEOUT_S)
        return sock

    def _read(self, port) -> bytes | None:
        """Bytes that arrived, b"" for none yet, None when the other side closed."""
        if self._source.kind == "serial":
            return port.read(max(1, int(getattr(port, "in_waiting", 0) or 0)))
        try:
            data = port.recv(4096)
        except socket.timeout:
            return b""
        return data if data else None

    def _wait(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while self._running and time.monotonic() < end:
            time.sleep(0.05)

    def run(self) -> None:
        self._running = True
        while self._running:
            self.state.emit("connecting", "")
            try:
                port = self._open()
            except Exception as e:
                if not self._running:
                    break
                self.state.emit("error", _short_reason(e))
                self._wait(_RETRY_S)
                continue
            self._port = port
            self.state.emit("connected", "")
            parser = Rtcm3Parser()
            reason = ""
            try:
                while self._running:
                    data = self._read(port)
                    if data is None:
                        reason = "the base closed the connection"
                        break
                    for frame in parser.feed(data):
                        self.corrections.emit(frame.data, frame.message_type)
                        position = station_position(frame)
                        if position is not None:
                            self.station.emit(*position)
            except Exception as e:
                reason = _short_reason(e)
            finally:
                self._port = None
                try:
                    port.close()
                except Exception:
                    pass
            if not self._running:
                break
            self.state.emit("error", reason or "connection lost")
            self._wait(_RETRY_S)


def _short_reason(error: object) -> str:
    """An error as one short line for the dashboard."""
    text = str(error or "").strip().splitlines()[0] if str(error or "").strip() else type(error).__name__
    low = text.lower()
    if "access is denied" in low or "permissionerror" in low or "permission denied" in low:
        return "the port is in use by another program"
    if "could not open port" in low or "filenotfounderror" in low or "no such file" in low:
        return "no such port"
    if "refused" in low or "10061" in low:
        return "nothing is listening there"
    if "timed out" in low or "timeout" in low:
        return "no answer"
    return text[:80]
