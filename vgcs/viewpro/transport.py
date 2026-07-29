from __future__ import annotations

import socket
import threading


class ViewproTcpTransport:
    """TCP client to the Viewpro/ViewLink gimbal core (default port 2000).

    Per the ViewLink manual §2.2.4/§3.2.1(7): "TCP Settings: Enable TCP. The
    gimbal core acts as the TCP server. The port number can be modified.
    Supports simultaneous control by multiple clients. This is the default
    shipping configuration." UDP is documented as a point-to-point
    alternative but is not implemented here (TCP is the default and is
    sufficient for the current send-raw-command scope — see
    DOCS/VIEWPRO-CAMERA-REFERENCE.md).
    """

    def __init__(
        self,
        host: str,
        port: int = 2000,
        *,
        timeout_s: float = 1.0,
    ) -> None:
        self._host = str(host or "").strip()
        self._port = int(port)
        self._timeout_s = max(0.1, float(timeout_s))
        self._lock = threading.Lock()
        self._sock: socket.socket | None = None

    def open(self) -> None:
        with self._lock:
            if self._sock is not None:
                return
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self._timeout_s)
            sock.connect((self._host, self._port))
            self._sock = sock

    def close(self) -> None:
        with self._lock:
            if self._sock is None:
                return
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None

    def is_open(self) -> bool:
        with self._lock:
            return self._sock is not None

    def send(self, payload: bytes) -> None:
        self.open()
        with self._lock:
            assert self._sock is not None
            self._sock.sendall(payload)

    def send_and_receive(self, payload: bytes, *, max_bytes: int = 4096) -> bytes:
        self.open()
        with self._lock:
            assert self._sock is not None
            self._sock.sendall(payload)
            try:
                return self._sock.recv(max_bytes)
            except socket.timeout:
                return b""
