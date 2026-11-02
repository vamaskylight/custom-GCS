"""The Skydroid C20's camera port: its zoom (row B10, 2026-11-03).

Skydroid's RC SDK (rcsdk 1.9.2 and 1.6.5, ``com.skydroid.rcsdk.common.payload.C20Camera``,
read with javap) opens the C20's camera on TCP port 8100 of the camera's address, and its
gimbal on port 5000 (TOP, ``TopTcpTransport``). The camera's commands are binary packets:
6 zero bytes, a group byte, a command byte, the payload's length (2 bytes, high byte first),
then the payload. The SDK's TCP layer writes them to the socket as they are. So the TOP lens
frames that went to port 5000 never zoomed it (the client, 2026-10-27 and 2026-10-30).

The zoom moves while it is told to: start in or out, then stop. Its zoom can be read back
(group 0x22, command 0x4A): the reply's payload byte 9 runs from 1 to 66, three values for
each 1x (the SDK's ``getZoom``), so 1x to 22x. None of this is tried on a C20.

The app has the same in C++ (apk/custom/src/Skydroid/C20Camera.*), checked against this file
by apk/custom/test/gen_c20_vectors.py.
"""

from __future__ import annotations

import select
import socket
import threading
import time
from collections.abc import Callable

PORT = 8100
HEADER_LEN = 10
GROUP_ZOOM = 0x22
CMD_ZOOM_MOVE = 0x31
CMD_ZOOM_READ = 0x4A
ZOOM_MAX = 22
# The SDK takes a zoom reply only with this much payload (its getZoom).
ZOOM_READ_MIN_PAYLOAD = 16


def build_packet(group: int, command: int, payload: bytes = b"") -> bytes:
    """6 zero bytes, group, command, the payload's length (high byte first), the payload
    (the SDK's ``C20Camera.sendCmdData``)."""
    data = bytes(payload)
    return bytes(6) + bytes([group & 0xFF, command & 0xFF, (len(data) >> 8) & 0xFF, len(data) & 0xFF]) + data


def build_zoom_in() -> bytes:
    return build_packet(GROUP_ZOOM, CMD_ZOOM_MOVE, b"\x0b\x00\x01")


def build_zoom_out() -> bytes:
    return build_packet(GROUP_ZOOM, CMD_ZOOM_MOVE, b"\x0c\x00\x01")


def build_zoom_stop() -> bytes:
    """The SDK's ``stopZoomOrFucus``: it stops a zoom and a focus move."""
    return build_packet(GROUP_ZOOM, CMD_ZOOM_MOVE, b"\x0b\x01\x01")


def build_zoom_query() -> bytes:
    return build_packet(GROUP_ZOOM, CMD_ZOOM_READ, b"\x00\x00")


def split_packets(chunk: bytes) -> list[bytes]:
    """The packets in what one read brought, cut by their length.

    The SDK takes each read as one packet and never looks at the length. So a packet whose
    length runs past the read, or a zoom reply that says less than its 16 bytes, takes the
    rest of the read, as the SDK would. Less than a header at the end is dropped.
    """
    out: list[bytes] = []
    i = 0
    while len(chunk) - i >= HEADER_LEN:
        length = (chunk[i + 8] << 8) | chunk[i + 9]
        end = i + HEADER_LEN + length
        short_zoom = chunk[i + 6] == GROUP_ZOOM and chunk[i + 7] == CMD_ZOOM_READ and length < ZOOM_READ_MIN_PAYLOAD
        if end > len(chunk) or short_zoom:
            out.append(bytes(chunk[i:]))
            break
        out.append(bytes(chunk[i:end]))
        i = end
    return out


def zoom_of_reply(packet: bytes) -> int | None:
    """The zoom (1 to 22) a zoom reply says, or None (another packet, too short, or a value
    the SDK has no zoom for)."""
    if len(packet) < HEADER_LEN + ZOOM_READ_MIN_PAYLOAD:
        return None
    if packet[6] != GROUP_ZOOM or packet[7] != CMD_ZOOM_READ:
        return None
    value = packet[HEADER_LEN + 9]
    if 1 <= value <= 3 * ZOOM_MAX:
        return (value - 1) // 3 + 1
    return None


class C20CameraPort:
    """One TCP connection to the camera port while the C20 is in use, and its zoom as it
    last said it.

    A thread keeps the connection (tried again 2 s after a failure, said once in the log)
    and reads the replies. ``send`` never waits for a connection: a packet sent while there
    is none is dropped, and says so by returning False.
    """

    _CONNECT_TIMEOUT_S = 1.0
    _RETRY_GAP_S = 2.0
    _SEND_TIMEOUT_S = 0.5

    def __init__(self, host: str, port: int = PORT, *, log: Callable[[str], None] | None = None) -> None:
        self._host = str(host or "").strip()
        self._port = int(port)
        self._log = log
        self._lock = threading.Lock()
        self._sock: socket.socket | None = None
        self._running = False
        self._thread: threading.Thread | None = None
        self._zoom_x: int | None = None
        self._zoom_mono = 0.0
        self._zoom_raw = ""
        self._said_cannot_connect = False

    @property
    def address(self) -> tuple[str, int]:
        return self._host, self._port

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="c20-camera-port", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._running = False
        thread, self._thread = self._thread, None
        if thread is not None:
            thread.join(timeout=1.5)
        with self._lock:
            self._drop()

    def connected(self) -> bool:
        with self._lock:
            return self._sock is not None

    def send(self, packet: bytes) -> bool:
        with self._lock:
            sock = self._sock
            if sock is None:
                return False
            try:
                sock.settimeout(self._SEND_TIMEOUT_S)
                sock.sendall(packet)
                return True
            except OSError as e:
                self._drop(f"lost ({e})")
                return False

    def zoom_x(self, max_age_s: float | None = None) -> int | None:
        """The zoom the camera last said (1 to 22), or None (never, or older than ``max_age_s``)."""
        with self._lock:
            if self._zoom_x is None:
                return None
            if max_age_s is not None and time.monotonic() - self._zoom_mono > max_age_s:
                return None
            return self._zoom_x

    def zoom_said_mono(self) -> float:
        """When the camera last said its zoom (time.monotonic), 0.0 if never."""
        with self._lock:
            return self._zoom_mono

    def zoom_raw(self) -> str:
        """The payload of its last zoom reply, in hex, for a field log."""
        with self._lock:
            return self._zoom_raw

    # -----------------------------------------------------------------------

    def _loop(self) -> None:
        while self._running:
            with self._lock:
                sock = self._sock
            if sock is None:
                if not self._connect():
                    deadline = time.monotonic() + self._RETRY_GAP_S
                    while self._running and time.monotonic() < deadline:
                        time.sleep(0.05)
                continue
            try:
                readable, _, _ = select.select([sock], [], [], 0.1)
            except (OSError, ValueError):
                continue  # closed meanwhile: the next round sees it
            if not readable:
                continue
            with self._lock:
                if self._sock is not sock:
                    continue
                try:
                    sock.settimeout(0.0)
                    data = sock.recv(4096)
                except (BlockingIOError, socket.timeout):
                    continue
                except OSError as e:
                    self._drop(f"lost ({e})")
                    continue
                if not data:
                    self._drop("closed by the camera")
                    continue
                for packet in split_packets(data):
                    zoom = zoom_of_reply(packet)
                    if zoom is not None:
                        self._zoom_x = zoom
                        self._zoom_mono = time.monotonic()
                        self._zoom_raw = packet[HEADER_LEN:].hex().upper()

    def _connect(self) -> bool:
        try:
            sock = socket.create_connection((self._host, self._port), timeout=self._CONNECT_TIMEOUT_S)
        except OSError as e:
            if not self._said_cannot_connect:
                self._said_cannot_connect = True
                self._say(f"cannot connect to {self._host}:{self._port} ({e})")
            return False
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        with self._lock:
            if not self._running:
                sock.close()
                return False
            self._sock = sock
        self._said_cannot_connect = False
        self._say(f"connected to {self._host}:{self._port}")
        return True

    def _drop(self, why: str = "") -> None:
        """With the lock held."""
        sock, self._sock = self._sock, None
        if sock is None:
            return
        try:
            sock.close()
        except OSError:
            pass
        if why:
            self._say(why)

    def _say(self, text: str) -> None:
        if self._log is not None:
            try:
                self._log(f"[VGCS:skydroid] C20 camera port: {text}")
            except Exception:
                pass
