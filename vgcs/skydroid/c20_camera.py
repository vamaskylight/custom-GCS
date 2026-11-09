"""The Skydroid C20's camera port: its zoom (row B10, 2026-11-03), its photos and recordings
(row B26, 2026-11-04), and the text that switches its torch (row B27).

Skydroid's RC SDK (rcsdk 1.9.2 and 1.6.5, ``com.skydroid.rcsdk.common.payload.C20Camera``,
read with javap) opens the C20's camera on TCP port 8100 of the camera's address, and its
gimbal on port 5000 (TOP, ``TopTcpTransport``). The camera's commands are binary packets:
6 zero bytes, a group byte, a command byte, the payload's length (2 bytes, high byte first),
then the payload. The SDK's TCP layer writes them to the socket as they are. So the TOP lens
frames that went to port 5000 never zoomed it (the client, 2026-10-27 and 2026-10-30).

The zoom moves while it is told to: start in or out, then stop. Its zoom can be read back
(group 0x22, command 0x4A): the reply's payload byte 9 runs from 1 to 66, three values for
each 1x (the SDK's ``getZoom``), so 1x to 22x. It zooms on the client's V20 (APK test build 19).

A photo, a record start and a record stop are group 0x33, command 0x81, with 32 bytes of
payload (the SDK's ``takePicture``, ``startRecordVideo``, ``stopRecordVideo``). TOP's CAP and
REC on port 5000 only made the RC's own copies (the client, build 19). The SDK counts a reply
with the same group and command and at least 32 bytes of payload as done, a shorter one as
failed (``DataAbortException``), and none as a time-out. What the payload says it does not read.

The torch is not on this port: ``C20Gimbal.toggleLED`` writes the text ``AT+LED -t1`` (on) or
``AT+LED -t0`` (off) on the gimbal connection, port 5000, and reads nothing back.

The camera's state (group 0x33, command 0x55, no payload) is asked as the SDK's ``ReadStateThread``
asks it: at once, then every 15 s, its answer left unread. The client's V20 closed a connection that
asked nothing every 20 to 24 s (row B29, 2026-11-05).

For a field log (row B33, 2026-11-09): the client's V20 stopped its video twice in one run, with
nothing in VGCS's log to say how it streams or what reached its camera port just before. So each
new connection also asks the camera's video settings and its version once (the SDK's
getEncodingConfig and getVersion, only read, never set), the log says them and says the state's
answer when it changes, and every packet of this port goes into the timed log of the camera's
commands (logs/skydroid_top_udp.log), beside the gimbal's.

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
GROUP_CAMERA = 0x33
CMD_CAPTURE = 0x81  # a photo, a record start, a record stop
# The camera's state (the SDK's getCameraState: group 0x33, command 0x55, no payload). The SDK's
# ReadStateThread asks it at once and then every 15 s, and does nothing with the answer. VGCS asked
# nothing while idle, and the client's V20 closed VGCS's connection every 20 to 24 s ("forcibly
# closed by the remote host", v20_log.txt of 2026-11-05, row B29): asked as the SDK does since 2026-11-06.
CMD_STATE = 0x55
STATE_ASK_S = 15.0
CAPTURE_PAYLOAD_LEN = 32
# The SDK takes a reply to a photo or record command as done only with this much payload.
CAPTURE_REPLY_MIN_PAYLOAD = 32
_CAPTURE_PHOTO = 0x0C
_CAPTURE_RECORD_START = 0x0B
_CAPTURE_RECORD_STOP = 0x0A
# The camera's video settings and its version (row B33, 2026-11-09: the client's V20 stopped its
# video twice in one run, and nothing in the log said how it streams). The SDK's getEncodingConfig
# (and getResolution, getBitrate, getFrameRate, getGOP, getEncodingFormat, which read the same reply)
# sends group 0x11, command 0x03, no payload, and takes a reply with at least 46 bytes of payload:
# byte 1 the resolution (4 720p, 5 960p, 6 1080p, 7 3M), byte 5 the bitrate in hundreds, byte 6 the
# frame rate plus 100, byte 7 the coding (10 H.264, 11 H.265), byte 13 the GOP. getVersion sends
# group 0x11, command 0x01 and hands its reply on unread. Both are only read here, never set.
GROUP_SETUP = 0x11
CMD_VERSION = 0x01
CMD_ENCODING_READ = 0x03
ENCODING_REPLY_MIN_PAYLOAD = 46
_SETUP_ASK_GAP_S = 0.5  # after the connect's state question, then between the two
_RESOLUTIONS = {4: "720p", 5: "960p", 6: "1080p", 7: "3M"}
_CODINGS = {10: "H.264", 11: "H.265"}
# A reply the SDK reads as one packet, and the payload it wants in it.
_REPLY_MIN_PAYLOAD = {
    (GROUP_ZOOM, CMD_ZOOM_READ): ZOOM_READ_MIN_PAYLOAD,
    (GROUP_CAMERA, CMD_CAPTURE): CAPTURE_REPLY_MIN_PAYLOAD,
    (GROUP_SETUP, CMD_ENCODING_READ): ENCODING_REPLY_MIN_PAYLOAD,
}


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


def build_state_query() -> bytes:
    """The SDK's getCameraState: no payload, so a length of 0."""
    return build_packet(GROUP_CAMERA, CMD_STATE)


def build_encoding_query() -> bytes:
    """The SDK's getEncodingConfig: group 0x11, command 0x03, no payload."""
    return build_packet(GROUP_SETUP, CMD_ENCODING_READ)


def build_version_query() -> bytes:
    """The SDK's getVersion: group 0x11, command 0x01, no payload."""
    return build_packet(GROUP_SETUP, CMD_VERSION)


def encoding_of_reply(packet: bytes) -> str | None:
    """The video settings a reply to ``build_encoding_query`` says, in words, as the SDK reads them
    (for example "H.264, 1080p, 30 fps, bitrate 4000, GOP 30"), or None (another packet, or less
    than the SDK takes). A value the SDK has no name for is given as its number."""
    if len(packet) < HEADER_LEN + ENCODING_REPLY_MIN_PAYLOAD:
        return None
    if packet[6] != GROUP_SETUP or packet[7] != CMD_ENCODING_READ:
        return None
    p = packet[HEADER_LEN:]
    resolution = _RESOLUTIONS.get(p[1], f"resolution {p[1]}")
    coding = _CODINGS.get(p[7], f"coding {p[7]}")
    return f"{coding}, {resolution}, {p[6] - 100} fps, bitrate {p[5] * 100}, GOP {p[13]}"


def _is_reply_to(packet: bytes, group: int, command: int) -> bool:
    return len(packet) >= HEADER_LEN and packet[6] == group and packet[7] == command


def _hex_and_text(data: bytes) -> str:
    """Bytes in hex, with their printable letters after them when there are some (a version)."""
    text = "".join(chr(b) for b in data if 32 <= b < 127).strip()
    return data.hex().upper() + (f" ({text})" if len(text) >= 3 else "")


def _build_capture(action: int) -> bytes:
    """32 bytes of payload, all zero but 0x0A and the action (the SDK's own)."""
    payload = bytearray(CAPTURE_PAYLOAD_LEN)
    payload[0] = 0x0A
    payload[1] = action
    return build_packet(GROUP_CAMERA, CMD_CAPTURE, bytes(payload))


def build_photo() -> bytes:
    return _build_capture(_CAPTURE_PHOTO)


def build_record_start() -> bytes:
    return _build_capture(_CAPTURE_RECORD_START)


def build_record_stop() -> bytes:
    return _build_capture(_CAPTURE_RECORD_STOP)


def build_torch(on: bool) -> bytes:
    """The text that switches the torch, for the gimbal connection (port 5000), not this port."""
    return b"AT+LED -t1\r\n" if on else b"AT+LED -t0\r\n"


def split_packets(chunk: bytes) -> list[bytes]:
    """The packets in what one read brought, cut by their length.

    The SDK takes each read as one packet and never looks at the length. So a packet whose
    length runs past the read, or a reply that says less than the SDK wants in it (16 bytes for
    a zoom, 32 for a photo or record), takes the rest of the read, as the SDK would. Less than
    a header at the end is dropped.
    """
    out: list[bytes] = []
    i = 0
    while len(chunk) - i >= HEADER_LEN:
        length = (chunk[i + 8] << 8) | chunk[i + 9]
        end = i + HEADER_LEN + length
        short = length < _REPLY_MIN_PAYLOAD.get((chunk[i + 6], chunk[i + 7]), 0)
        if end > len(chunk) or short:
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


def capture_reply_ok(packet: bytes) -> bool | None:
    """What a reply to a photo or record command says, as the SDK reads it: True (done), False
    (less than 32 bytes of payload, the SDK's failure), None (not such a reply)."""
    if len(packet) < HEADER_LEN or packet[6] != GROUP_CAMERA or packet[7] != CMD_CAPTURE:
        return None
    return len(packet) - HEADER_LEN >= CAPTURE_REPLY_MIN_PAYLOAD


class C20CameraPort:
    """One TCP connection to the camera port while the C20 is in use, its zoom as it last said
    it, and its replies to photo and record commands.

    A thread keeps the connection (tried again 2 s after a failure, said once in the log),
    asks the camera's state at once and every ``state_ask_s`` as the SDK does, and reads the
    replies. ``send`` never waits for a connection: a packet sent while there is none is
    dropped, and says so by returning False.

    For a field log (row B33): each new connection asks the camera's video settings and its
    version once, and the log says them; the log says the state's answer when it changes; and
    ``trace`` (if given) gets every packet sent ("TX") and read ("RX"), for the timed log of
    the camera's commands.
    """

    _CONNECT_TIMEOUT_S = 1.0
    _RETRY_GAP_S = 2.0
    _SEND_TIMEOUT_S = 0.5

    def __init__(self, host: str, port: int = PORT, *, log: Callable[[str], None] | None = None,
                 state_ask_s: float = STATE_ASK_S,
                 trace: Callable[[str, bytes], None] | None = None) -> None:
        self._host = str(host or "").strip()
        self._port = int(port)
        self._log = log
        self._trace = trace
        # A new connection asks the video settings and the version once, each in a packet of its
        # own a moment apart (the SDK waits for each answer; a camera may read a TCP read as one packet).
        self._setup_asks: list[tuple[float, bytes]] = []
        self._state_raw: str | None = None  # the payload of the last state answer, in hex
        self._encoding = ""  # the video settings the camera last said, in words
        self._state_ask_s = max(0.05, float(state_ask_s))
        self._next_state_ask = 0.0  # time.monotonic of the next state question
        self._lock = threading.Lock()
        self._sock: socket.socket | None = None
        self._running = False
        self._thread: threading.Thread | None = None
        self._zoom_x: int | None = None
        self._zoom_mono = 0.0
        self._zoom_raw = ""
        self._capture_count = 0  # replies to photo and record commands
        self._capture_ok = False
        self._capture_raw = ""
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
            except OSError as e:
                self._drop(f"lost ({e})")
                return False
        self._trace_packet("TX", packet)
        return True

    def video_settings(self) -> str:
        """The video settings the camera last said, in words ("" if it never said them)."""
        with self._lock:
            return self._encoding

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

    def capture_replies(self) -> tuple[int, bool, str]:
        """How many replies to photo and record commands came, whether the last one says done
        (capture_reply_ok), and its payload in hex."""
        with self._lock:
            return self._capture_count, self._capture_ok, self._capture_raw

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
            if time.monotonic() >= self._next_state_ask:
                self._next_state_ask = time.monotonic() + self._state_ask_s
                self.send(build_state_query())  # the SDK leaves its answer; the log says a change
            if self._setup_asks and time.monotonic() >= self._setup_asks[0][0]:
                self.send(self._setup_asks.pop(0)[1])
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
                packets = split_packets(data)
                said = self._read_setup_and_state(packets)
                for packet in packets:
                    zoom = zoom_of_reply(packet)
                    if zoom is not None:
                        self._zoom_x = zoom
                        self._zoom_mono = time.monotonic()
                        self._zoom_raw = packet[HEADER_LEN:].hex().upper()
                    done = capture_reply_ok(packet)
                    if done is not None:
                        self._capture_count += 1
                        self._capture_ok = done
                        self._capture_raw = packet[HEADER_LEN:].hex().upper()
            for packet in packets:
                self._trace_packet("RX", packet)
            for text in said:
                self._say(text)

    def _read_setup_and_state(self, packets: list[bytes]) -> list[str]:
        """With the lock held: what the log says about these packets (the video settings and the
        version, and the state's answer when it is new)."""
        said: list[str] = []
        for packet in packets:
            if _is_reply_to(packet, GROUP_SETUP, CMD_ENCODING_READ):
                words = encoding_of_reply(packet)
                if words is None:
                    said.append("video settings: an answer shorter than Skydroid's SDK takes "
                                f"({packet[HEADER_LEN:].hex().upper()})")
                else:
                    self._encoding = words
                    said.append(f"video settings: {words}")
            elif _is_reply_to(packet, GROUP_SETUP, CMD_VERSION):
                said.append(f"version: {_hex_and_text(packet[HEADER_LEN:])}")
            elif _is_reply_to(packet, GROUP_CAMERA, CMD_STATE):
                raw = packet[HEADER_LEN:].hex().upper()
                if raw != self._state_raw:
                    word = "state" if self._state_raw is None else "state changed"
                    self._state_raw = raw
                    said.append(f"{word}: {raw or 'no payload'}")
        return said

    def _trace_packet(self, direction: str, packet: bytes) -> None:
        if self._trace is None:
            return
        try:
            self._trace(direction, packet)
        except (OSError, ValueError, RuntimeError):
            return  # a field log that cannot be written never stops the camera port

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
        self._next_state_ask = 0.0  # a new connection asks the state at once, as the SDK's thread starts
        now = time.monotonic()
        self._setup_asks = [(now + _SETUP_ASK_GAP_S, build_encoding_query()),
                            (now + 2 * _SETUP_ASK_GAP_S, build_version_query())]
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
