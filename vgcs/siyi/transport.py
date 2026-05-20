from __future__ import annotations

import socket
import threading


class SiyiUdpTransport:
    def __init__(
        self,
        host: str,
        port: int = 37260,
        *,
        timeout_s: float = 0.25,
        retries: int = 1,
    ) -> None:
        self._host = str(host or "").strip()
        self._port = int(port)
        self._timeout_s = max(0.05, float(timeout_s))
        self._retries = max(0, int(retries))
        self._lock = threading.Lock()
        self._sock: socket.socket | None = None

    def open(self) -> None:
        with self._lock:
            if self._sock is not None:
                return
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(self._timeout_s)
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

    def send_only(self, payload: bytes) -> None:
        self.open()
        assert self._sock is not None
        self._sock.sendto(payload, (self._host, self._port))

    def send_and_receive(self, payload: bytes) -> bytes:
        self.open()
        last_error: Exception | None = None
        for _ in range(self._retries + 1):
            try:
                assert self._sock is not None
                self._sock.sendto(payload, (self._host, self._port))
                data, _addr = self._sock.recvfrom(4096)
                return data
            except Exception as e:
                last_error = e
                continue
        raise RuntimeError(f"SIYI UDP request failed: {last_error}")
