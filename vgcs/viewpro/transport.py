from __future__ import annotations

import socket
import threading
import time

# How long a failed connect suppresses further connect attempts from the
# fire-and-forget `send` path. The background poll loop keeps trying throughout,
# so the link still comes back on its own — this only stops the GUI thread from
# queueing up one blocking connect per command while the camera is away.
_CONNECT_RETRY_COOLDOWN_S = 2.0


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
        self._connect_blocked_until = 0.0

    def open(self, *, respect_cooldown: bool = False) -> None:
        """Connect if not already connected.

        ``respect_cooldown`` is for callers on the GUI thread. A connect to an
        absent camera blocks for the full socket timeout, and the gimbal jog
        path fires every 80 ms while a hold button is down — so with the camera
        away (a Viewpro sensor change takes its network stack down for seconds:
        field log 2026-08-26 shows port 2000 resetting and then timing out for
        ~40 s) every one of those ticks stalled the UI for a second. Once a
        connect has failed, those callers skip straight to the same failure the
        blocking attempt would have produced, and the background poll loop —
        which reconnects without the flag — brings the link back.
        """
        with self._lock:
            if self._sock is not None:
                return
            if respect_cooldown and time.monotonic() < self._connect_blocked_until:
                raise ConnectionError(
                    f"{self._host}:{self._port} unreachable "
                    "(waiting on the status poll to reconnect)"
                )
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self._timeout_s)
            try:
                sock.connect((self._host, self._port))
            except OSError:
                try:
                    sock.close()
                except Exception:
                    pass
                self._connect_blocked_until = time.monotonic() + _CONNECT_RETRY_COOLDOWN_S
                raise
            self._connect_blocked_until = 0.0
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
        # Fire-and-forget, called from the GUI thread — never block it on a
        # connect to a camera that has already refused one. See `open`.
        self.open(respect_cooldown=True)
        with self._lock:
            assert self._sock is not None
            try:
                self._sock.sendall(payload)
            except OSError:
                self._reset_broken_socket_locked()
                raise

    def send_and_receive(self, payload: bytes, *, max_bytes: int = 4096) -> bytes:
        self.open()
        with self._lock:
            assert self._sock is not None
            try:
                self._sock.sendall(payload)
                return self._sock.recv(max_bytes)
            except socket.timeout:
                return b""
            except OSError:
                self._reset_broken_socket_locked()
                raise

    def _reset_broken_socket_locked(self) -> None:
        """Called (with ``self._lock`` already held) when a send/recv fails at
        the OS level — e.g. the peer reset the connection. The socket object is
        now permanently unusable, but without this, ``open()`` would never
        reconnect: it only creates a new socket when ``self._sock is None``, so
        every future call would keep retrying the SAME dead socket forever.

        Field-observed 2026-08-03: once a single ConnectionResetError happened,
        this had no recovery for the rest of the session — dozens of identical
        errors on every retry, and the camera (gimbal/zoom/LRF, everything)
        stayed dead until the operator disconnected and reconnected the whole
        link by hand. Clearing the socket here lets the *next* call's
        ``open()`` establish a fresh connection instead.
        """
        try:
            self._sock.close()
        except Exception:
            pass
        self._sock = None
