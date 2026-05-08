from __future__ import annotations

import socket
import threading
import time
from pathlib import Path


class TopUdpTransport:
    def __init__(
        self,
        host: str,
        port: int = 5000,
        *,
        timeout_s: float = 0.25,
        retries: int = 1,
        log_path: str = "",
    ) -> None:
        self._host = str(host or "").strip()
        self._port = int(port)
        self._timeout_s = max(0.05, float(timeout_s))
        self._retries = max(0, int(retries))
        self._log_path = str(log_path or "").strip()
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

    def send_and_receive(self, payload: bytes, expect_reply: bool = True) -> bytes:
        self.open()
        last_error: Exception | None = None
        for _attempt in range(self._retries + 1):
            try:
                assert self._sock is not None
                self._sock.sendto(payload, (self._host, self._port))
                self._append_log("TX", payload)
                if not expect_reply:
                    return b""
                data, _addr = self._sock.recvfrom(4096)
                self._append_log("RX", data)
                return data
            except Exception as e:
                last_error = e
                continue
        raise RuntimeError(f"TOP UDP request failed: {last_error}")

    def _append_log(self, direction: str, payload: bytes) -> None:
        if not self._log_path:
            return
        try:
            path = Path(self._log_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            text = (payload or b"").decode("ascii", errors="ignore").strip()
            line = f"{int(time.time() * 1000)} {direction} {text}\n"
            path.open("a", encoding="utf-8").write(line)
        except Exception:
            return

