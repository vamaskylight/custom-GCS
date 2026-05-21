from __future__ import annotations

import socket
import threading
import time
from datetime import datetime, timezone
from pathlib import Path


def begin_skydroid_session_log(
    log_path: str,
    *,
    host: str,
    port: int,
    profile_id: str,
) -> None:
    """Start a fresh Skydroid TOP UDP log (truncates previous file)."""
    path = Path(str(log_path or "").strip())
    if not path:
        return
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).astimezone().strftime("%Y-%m-%d %H:%M:%S")
        with path.open("w", encoding="utf-8") as f:
            f.write(f"# VGCS Skydroid TOP UDP — session started {stamp}\n")
            f.write(f"# target={host}:{int(port)} profile={profile_id}\n")
            f.write("# Lines: TX/RX = gimbal/camera commands; STATUS = attitude poll (hidden by default)\n")
            f.write("# ERR = send/receive failure\n\n")
    except Exception:
        return


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
        self._listener_running = False
        self._listener: threading.Thread | None = None
        self._on_datagram = None
        self._listener_paused = False

    def set_datagram_handler(self, handler) -> None:
        self._on_datagram = handler

    def open(self) -> None:
        with self._lock:
            if self._sock is not None:
                return
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(self._timeout_s)
            try:
                sock.bind(("", 0))
            except Exception:
                pass
            self._sock = sock

    def close(self) -> None:
        self._listener_running = False
        if self._listener is not None:
            try:
                self._listener.join(timeout=0.5)
            except Exception:
                pass
            self._listener = None
        with self._lock:
            if self._sock is None:
                return
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None

    def start_listener(self) -> None:
        if self._listener_running:
            return
        self.open()
        self._listener_running = True
        self._listener = threading.Thread(target=self._listen_loop, daemon=True)
        self._listener.start()

    def _listen_loop(self) -> None:
        while self._listener_running:
            try:
                with self._lock:
                    if self._listener_paused:
                        sock = None
                    else:
                        sock = self._sock
                if sock is None:
                    time.sleep(0.02)
                    continue
                sock.settimeout(0.2)
                data, _addr = sock.recvfrom(4096)
                if data and self._on_datagram is not None:
                    self._on_datagram(data)
            except socket.timeout:
                continue
            except Exception:
                if self._listener_running:
                    time.sleep(0.2)
                continue

    def send_and_receive(
        self,
        payload: bytes,
        expect_reply: bool = True,
        *,
        log: bool = True,
        timeout_s: float | None = None,
    ) -> bytes:
        self.open()
        last_error: Exception | None = None
        attempts = self._retries + 1
        with self._lock:
            self._listener_paused = True
            try:
                assert self._sock is not None
                prev_timeout = self._sock.gettimeout()
                use_timeout = self._timeout_s if timeout_s is None else max(0.05, float(timeout_s))
                self._sock.settimeout(use_timeout)
                try:
                    for _attempt in range(attempts):
                        try:
                            self._sock.sendto(payload, (self._host, self._port))
                            if log:
                                self._append_log("TX", payload)
                            if not expect_reply:
                                return b""
                            data, _addr = self._sock.recvfrom(4096)
                            if log:
                                self._append_log("RX", data)
                            return data
                        except Exception as e:
                            last_error = e
                            continue
                finally:
                    try:
                        self._sock.settimeout(prev_timeout)
                    except Exception:
                        pass
            finally:
                self._listener_paused = False
        self._append_log("ERR", str(last_error or "unknown").encode("ascii", errors="ignore"))
        raise RuntimeError(f"TOP UDP request failed: {last_error}")

    def _append_log(self, direction: str, payload: bytes) -> None:
        if not self._log_path:
            return
        try:
            path = Path(self._log_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            text = (payload or b"").decode("ascii", errors="ignore").strip()
            ts = datetime.now().astimezone().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            target = f"{self._host}:{self._port}"
            line = f"{ts}  {direction:5}  {target}  {text}\n"
            with path.open("a", encoding="utf-8") as f:
                f.write(line)
        except Exception:
            return
