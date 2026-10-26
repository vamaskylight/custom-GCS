from __future__ import annotations

import select
import socket
import threading
import time
from collections import deque
from collections.abc import Callable
from datetime import datetime, timezone
from pathlib import Path

from vgcs.skydroid.targets import local_ipv4_for_target


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
        self._bind_ip = ""

    def set_datagram_handler(self, handler) -> None:
        self._on_datagram = handler

    def _reopen_socket(self) -> None:
        with self._lock:
            if self._sock is not None:
                try:
                    self._sock.close()
                except Exception:
                    pass
                self._sock = None
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(self._timeout_s)
            lip = str(self._bind_ip or "").strip()
            try:
                if lip:
                    sock.bind((lip, 0))
                else:
                    sock.bind(("", 0))
            except Exception:
                try:
                    sock.bind(("", 0))
                except Exception:
                    pass
            self._sock = sock

    def set_route_host(self, host: str) -> None:
        """Re-bind UDP socket on the NIC used to reach ``host`` (hotspot + camera LAN)."""
        lip = local_ipv4_for_target(host) or ""
        if lip == self._bind_ip and self._sock is not None:
            return
        self._bind_ip = lip
        was_listening = self._listener_running
        if was_listening:
            self._listener_running = False
            if self._listener is not None:
                try:
                    self._listener.join(timeout=0.5)
                except Exception:
                    pass
                self._listener = None
        self._reopen_socket()
        if was_listening:
            self.start_listener()

    def open(self) -> None:
        if self._sock is not None:
            return
        self.set_route_host(self._host)
        if self._sock is None:
            self._reopen_socket()

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
        host: str | None = None,
        port: int | None = None,
    ) -> bytes:
        """``host``/``port`` override the instance's own ``_host``/``_port``
        for this call only. Field-reported: a background endpoint probe or
        zoom-burst on another thread temporarily repoints the shared
        ``_host``/``_port`` (e.g. to try an alternate port) with no
        synchronization against a concurrent send here — a real motion
        command could silently go out on the wrong port mid-probe. Callers on
        a hot/real-time path (gimbal motion) should pass the destination
        explicitly so this call is self-contained and immune to that race,
        rather than relying on the shared, frequently-mutated instance state.
        """
        self.open()
        last_error: Exception | None = None
        attempts = self._retries + 1
        target_host = self._host if host is None else str(host)
        target_port = self._port if port is None else int(port)
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
                            self._sock.sendto(payload, (target_host, target_port))
                            if log:
                                self._append_log("TX", payload, target=f"{target_host}:{target_port}")
                            if not expect_reply:
                                return b""
                            data, _addr = self._sock.recvfrom(4096)
                            if log:
                                self._append_log("RX", data, target=f"{target_host}:{target_port}")
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
        self._append_log(
            "ERR",
            str(last_error or "unknown").encode("ascii", errors="ignore"),
            target=f"{target_host}:{target_port}",
        )
        raise RuntimeError(f"TOP UDP request failed: {last_error}")

    def receive_matching(
        self,
        predicate: Callable[[bytes], bool],
        *,
        timeout_s: float = 1.0,
        log: bool = False,
    ) -> bytes | None:
        """Drain datagrams until ``predicate`` matches or timeout (shared UDP socket)."""
        self.open()
        with self._lock:
            self._listener_paused = True
            try:
                assert self._sock is not None
                prev_timeout = self._sock.gettimeout()
                deadline = time.monotonic() + max(0.05, float(timeout_s))
                stray_logged = False
                try:
                    while time.monotonic() < deadline:
                        remaining = deadline - time.monotonic()
                        self._sock.settimeout(max(0.05, remaining))
                        try:
                            data, _addr = self._sock.recvfrom(4096)
                        except socket.timeout:
                            continue
                        except Exception:
                            break
                        if log:
                            self._append_log("RX", data)
                        if predicate(data):
                            return data
                        if log and not stray_logged:
                            text = (data or b"").decode("ascii", errors="ignore")
                            self._append_log(
                                "RX-stray",
                                text.encode("ascii", errors="ignore"),
                            )
                            stray_logged = True
                finally:
                    try:
                        self._sock.settimeout(prev_timeout)
                    except Exception:
                        pass
            finally:
                self._listener_paused = False
        return None

    def _append_log(self, direction: str, payload: bytes, *, target: str | None = None) -> None:
        _append_top_log(self._log_path, direction, payload, target or f"{self._host}:{self._port}")


def _append_top_log(log_path: str, direction: str, payload: bytes, target: str) -> None:
    if not log_path:
        return
    try:
        path = Path(log_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        text = (payload or b"").decode("ascii", errors="ignore").strip()
        ts = datetime.now().astimezone().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        line = f"{ts}  {direction:5}  {target}  {text}\n"
        with path.open("a", encoding="utf-8") as f:
            f.write(line)
    except Exception:
        return


def split_top_frames(data: bytes) -> tuple[list[bytes], bytes]:
    """Cut the whole TOP frames off the front of a TCP byte stream.

    A frame is "#TP" or "#tp", the address (2), the data length (one hex digit),
    the control (1), the tag (3), the data and the checksum (2): 12 bytes plus
    the data. TCP can hand over half a frame, or several frames at once.
    Returns the frames and the bytes left over (the start of the next frame).
    Bytes that cannot start a frame are dropped.
    """
    frames: list[bytes] = []
    i = 0
    while True:
        starts = [p for p in (data.find(b"#TP", i), data.find(b"#tp", i)) if p >= 0]
        if not starts:
            # Keep a header that is cut off at the end ("#" or "#T").
            for keep in (2, 1):
                tail = data[-keep:]
                if len(data) - keep >= i and tail in (b"#", b"#T", b"#t"):
                    return frames, tail
            return frames, b""
        start = min(starts)
        if len(data) - start < 6:
            return frames, data[start:]
        try:
            size = 12 + int(data[start + 5 : start + 6].decode("ascii"), 16)
        except ValueError:
            i = start + 1  # not a frame after all
            continue
        if len(data) - start < size:
            return frames, data[start:]
        frames.append(data[start : start + size])
        i = start + size


class TopTcpTransport:
    """TOP over TCP, for a camera that takes it on TCP only: the Skydroid C20
    (TOP document V1.1.6, section 2.1.1, port 5000). The same calls as
    TopUdpTransport, so the adapter can use either.

    One connection is kept for everything, because the camera may take one TCP
    user only. The frames are cut out of the byte stream by their length. As
    over UDP, the reply to a request is the first frame that arrives after it.
    Frames that came before the request go to the frame handler, as the
    listener's do. A lost connection is opened again by the next request or by
    the listener. After a failed try, the next try waits a moment.
    """

    _CONNECT_TIMEOUT_S = 1.0
    _RETRY_GAP_S = 2.0

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
        self._peer: tuple[str, int] | None = None
        self._rest = b""
        self._frames: deque[bytes] = deque()
        self._failed_mono: dict[tuple[str, int], float] = {}  # last failed try per address
        self._listener_running = False
        self._listener: threading.Thread | None = None
        self._on_datagram = None
        self._bind_ip = ""

    def set_datagram_handler(self, handler) -> None:
        self._on_datagram = handler

    def set_route_host(self, host: str) -> None:
        """Connect from the NIC used to reach ``host`` (hotspot + camera LAN)."""
        lip = local_ipv4_for_target(host) or ""
        if lip == self._bind_ip:
            return
        self._bind_ip = lip
        with self._lock:
            self._drop()  # the next request connects from the new NIC

    def open(self) -> None:
        with self._lock:
            try:
                self._connect_to(self._host, self._port)
            except OSError:
                pass

    def close(self) -> None:
        self._listener_running = False
        if self._listener is not None:
            try:
                self._listener.join(timeout=0.5)
            except Exception:
                pass
            self._listener = None
        with self._lock:
            self._drop()

    def start_listener(self) -> None:
        if self._listener_running:
            return
        self._listener_running = True
        self._listener = threading.Thread(target=self._listen_loop, daemon=True)
        self._listener.start()

    def _listen_loop(self) -> None:
        while self._listener_running:
            frames: list[bytes] = []
            with self._lock:
                try:
                    if self._sock is None:  # else keep the address the last request used
                        self._connect_to(self._host, self._port)
                    self._fill(0.0)
                    frames = list(self._frames)
                    self._frames.clear()
                except OSError as e:
                    self._drop(f"lost ({e})")
                sock = self._sock
            self._hand_on(frames)
            if sock is None:
                time.sleep(0.2)
                continue
            # Wait for data without the lock, as the UDP listener does: a command
            # never waits for the listener (with the lock held here, a jog waited
            # up to 60 ms on loopback).
            try:
                select.select([sock], [], [], 0.05)
            except (OSError, ValueError):
                pass  # closed meanwhile: the next round sees it

    def send_and_receive(
        self,
        payload: bytes,
        expect_reply: bool = True,
        *,
        log: bool = True,
        timeout_s: float | None = None,
        host: str | None = None,
        port: int | None = None,
    ) -> bytes:
        """``host``/``port`` override the instance's own endpoint for this call
        (see TopUdpTransport.send_and_receive); over TCP that is a connection
        to that endpoint."""
        target_host = self._host if host is None else str(host)
        target_port = self._port if port is None else int(port)
        use_timeout = self._timeout_s if timeout_s is None else max(0.05, float(timeout_s))
        last_error: Exception | None = None
        earlier: list[bytes] = []
        reply: bytes | None = None
        with self._lock:
            for _attempt in range(self._retries + 1):
                try:
                    self._connect_to(target_host, target_port)
                    self._fill(0.0)  # what came before this request is not its reply
                    earlier.extend(self._frames)
                    self._frames.clear()
                    assert self._sock is not None
                    self._sock.settimeout(use_timeout)
                    self._sock.sendall(payload)
                    if log:
                        self._log("TX", payload, target_host, target_port)
                    if not expect_reply:
                        reply = b""
                        break
                    deadline = time.monotonic() + use_timeout
                    while not self._frames and time.monotonic() < deadline:
                        self._fill(max(0.01, deadline - time.monotonic()))
                    if self._frames:
                        reply = self._frames.popleft()
                        if log:
                            self._log("RX", reply, target_host, target_port)
                        break
                    last_error = TimeoutError("no reply")
                except OSError as e:
                    last_error = e
                    self._drop(f"lost ({e})")
        self._hand_on(earlier)
        if reply is not None:
            return reply
        self._log("ERR", str(last_error or "unknown").encode("ascii", errors="ignore"), target_host, target_port)
        raise RuntimeError(f"TOP TCP request failed: {last_error}")

    def receive_matching(
        self,
        predicate: Callable[[bytes], bool],
        *,
        timeout_s: float = 1.0,
        log: bool = False,
    ) -> bytes | None:
        """Read frames until ``predicate`` matches or the time is up. The other
        frames go to the frame handler."""
        others: list[bytes] = []
        found: bytes | None = None
        with self._lock:
            deadline = time.monotonic() + max(0.05, float(timeout_s))
            try:
                self._connect_to(self._host, self._port)
                while found is None and time.monotonic() < deadline:
                    if not self._frames:
                        self._fill(max(0.01, deadline - time.monotonic()))
                    while self._frames and found is None:
                        frame = self._frames.popleft()
                        if log:
                            self._log("RX", frame, self._host, self._port)
                        if predicate(frame):
                            found = frame
                        else:
                            others.append(frame)
            except OSError as e:
                self._drop(f"lost ({e})")
        self._hand_on(others)
        return found

    # -- with self._lock held ------------------------------------------------

    def _connect_to(self, host: str, port: int) -> None:
        if self._sock is not None and self._peer == (host, port):
            return
        self._drop()
        failed = self._failed_mono.get((host, port))
        if failed is not None and time.monotonic() - failed < self._RETRY_GAP_S:
            raise ConnectionError("waiting before the next try")
        source = (self._bind_ip, 0) if self._bind_ip else None
        try:
            sock = socket.create_connection((host, port), timeout=self._CONNECT_TIMEOUT_S, source_address=source)
        except OSError as e:
            if failed is None:  # said once, until it connects
                self._log("CONN", f"cannot connect ({e})".encode("ascii", errors="ignore"), host, port)
            self._failed_mono[(host, port)] = time.monotonic()
            raise
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._sock = sock
        self._peer = (host, port)
        self._failed_mono.pop((host, port), None)
        self._log("CONN", b"connected", host, port)

    def _drop(self, why: str = "") -> None:
        sock, self._sock = self._sock, None
        peer, self._peer = self._peer, None
        self._rest = b""
        self._frames.clear()
        if sock is None:
            return
        try:
            sock.close()
        except OSError:
            pass
        if why and peer is not None:
            self._log("CONN", why.encode("ascii", errors="ignore"), *peer)

    def _fill(self, wait_s: float) -> None:
        """Read what has arrived (waiting up to ``wait_s``) into whole frames."""
        assert self._sock is not None
        self._sock.settimeout(wait_s if wait_s > 0 else 0.0)
        while True:
            try:
                data = self._sock.recv(4096)
            except (socket.timeout, BlockingIOError):
                return
            if not data:
                raise ConnectionError("closed by the camera")
            frames, self._rest = split_top_frames(self._rest + data)
            self._frames.extend(frames)
            self._sock.settimeout(0.0)  # take the rest of what is there, without waiting

    # -----------------------------------------------------------------------

    def _hand_on(self, frames: list[bytes]) -> None:
        handler = self._on_datagram
        for frame in frames:
            if handler is not None:
                try:
                    handler(frame)
                except Exception:
                    pass

    def _log(self, direction: str, payload: bytes, host: str, port: int) -> None:
        _append_top_log(self._log_path, direction, payload, f"{host}:{port}")
