from __future__ import annotations

from dataclasses import dataclass
import ipaddress
import os
import threading
import time
import json
from urllib.parse import urlparse
import shutil
import subprocess
from typing import Optional, Protocol

from PySide6.QtCore import QObject, Signal, QMetaObject, Qt, Slot
from PySide6.QtGui import QImage
try:
    import numpy as np  # type: ignore[import-not-found]

    HAS_NUMPY = True
except Exception:  # pragma: no cover
    np = None  # type: ignore[assignment]
    HAS_NUMPY = False

try:
    from PySide6.QtMultimedia import (
        QCamera,
        QAudioOutput,
        QMediaCaptureSession,
        QMediaDevices,
        QMediaRecorder,
        QMediaPlayer,
        QImageCapture,
        QVideoSink,
    )

    HAS_MULTIMEDIA = True
except Exception:  # pragma: no cover - depends on platform build
    QCamera = None  # type: ignore[assignment]
    QAudioOutput = None  # type: ignore[assignment]
    QMediaCaptureSession = None  # type: ignore[assignment]
    QMediaDevices = None  # type: ignore[assignment]
    QMediaRecorder = None  # type: ignore[assignment]
    QMediaPlayer = None  # type: ignore[assignment]
    QImageCapture = None  # type: ignore[assignment]
    QVideoSink = None  # type: ignore[assignment]
    HAS_MULTIMEDIA = False


@dataclass(frozen=True)
class FrameMeta:
    source_id: str
    device_name: str
    timestamp_ms: int


@dataclass(frozen=True)
class VideoFrame:
    image: QImage
    meta: FrameMeta


def _rtsp_transport_order_auto(url: str) -> tuple[str, ...]:
    """
    Offline / local-radio links often break RTP/UDP (NAT, flaky Wi‑Fi) but work with
    RTSP-over-TCP (interleaved). Prefer TCP first on RFC1918 / link-local / loopback hosts.

    For public / unknown hostnames (typical cloud RTSP relays), try UDP first — many
    servers expect RTP/UDP when the path is reachable on the open internet.
    """
    default: tuple[str, ...] = ("udp", "tcp")
    raw = str(url or "").strip()
    if not raw.lower().startswith("rtsp://"):
        return default
    try:
        pu = urlparse(raw)
        host_raw = pu.hostname or ""
    except Exception:
        return default
    if not host_raw:
        return default
    hn = host_raw.strip().lower()
    if hn in ("localhost", "127.0.0.1", "::1"):
        return ("tcp", "udp")
    try:
        ip = ipaddress.ip_address(host_raw.strip("[]"))
        if ip.is_loopback or ip.is_link_local or ip.is_private:
            return ("tcp", "udp")
    except ValueError:
        pass
    return default


def _rtsp_transport_sequence(url: str, mode: str) -> tuple[str | None, ...]:
    """
    Transports to try when opening FFmpeg. Returns a single `(None,)` entry for inputs
    that are not RTSP (FFmpeg ignores `-rtsp_transport` anyway, but avoids confusing logs).
    """
    raw = str(url or "").strip()
    if _url_scheme(raw) != "rtsp":
        return (None,)
    m = str(mode or "auto").strip().lower()
    if m == "udp":
        return ("udp",)
    if m == "tcp":
        return ("tcp",)
    return _rtsp_transport_order_auto(raw)


def _ffmpeg_udp_raw_demux(url: str, udp_input_format: str) -> list[str]:
    """
    FFmpeg demuxer hints for bare UDP (-- h264/hevc Annex B); omitted for MPEG-TS (auto).
    """
    if _url_scheme(str(url or "").strip()) != "udp":
        return []
    fmt = str(udp_input_format or "").strip().lower()
    if fmt in ("h264", "264", "avc"):
        return ["-f", "h264"]
    if fmt in ("hevc", "265", "h265"):
        return ["-f", "hevc"]
    if fmt == "mpegts":
        return ["-f", "mpegts"]
    return []


def _ffmpeg_preflags_before_input(url: str, *, rtsp_transport: str | None) -> list[str]:
    """Flags immediately before `-i` (after optional `-f`)."""
    sc = _url_scheme(str(url or "").strip())
    out: list[str] = []
    if sc == "rtsp" and rtsp_transport in ("udp", "tcp"):
        out.extend(["-rtsp_transport", rtsp_transport])
        # Optional I/O stall limit (microseconds). Some Windows FFmpeg builds or cameras
        # mis-handle -rw_timeout and never output a frame — leave off unless set explicitly.
        rw = str(os.environ.get("VGCS_FFMPEG_RW_TIMEOUT_MS", "") or "").strip()
        if rw.isdigit() and int(rw) > 0:
            out.extend(["-rw_timeout", rw])
        # Low-latency RTSP: avoid large analyze windows (adds seconds of demux delay and
        # makes motion look like it "lags behind" then catches up). nobuffer matches UDP path.
        out.extend(
            [
                "-fflags",
                "nobuffer+discardcorrupt+genpts",
                "-analyzeduration",
                "1500000",
                "-probesize",
                "1000000",
                "-flags",
                "low_delay",
            ]
        )
    out.extend(["-err_detect", "ignore_err"])
    if sc == "udp":
        out.extend(
            [
                "-fflags",
                "nobuffer+genpts+discardcorrupt",
                "-analyzeduration",
                "2500000",
                "-probesize",
                "131072",
            ]
        )
    elif sc != "rtsp":
        out.extend(["-fflags", "+genpts+discardcorrupt"])
    return out


def _url_scheme(url: str) -> str:
    try:
        s = urlparse(str(url or "").strip()).scheme
        return str(s or "").strip().lower()
    except Exception:
        return ""


def _ffmpeg_vf_rgb_fixed_size(w: int, h: int) -> str:
    """
    Scale to fit inside WxH, then pad to exactly WxH.

    `force_original_aspect_ratio=decrease` alone can emit smaller frames when the
    stream aspect does not match the target box; our rawvideo reader assumes
    every packet is exactly w*h*3 bytes. Padding avoids silent decode drops
    (symptom: first frame OK, then frozen preview).
    """
    w = max(2, int(w))
    h = max(2, int(h))
    return (
        f"scale={w}:{h}:force_original_aspect_ratio=decrease:flags=fast_bilinear,"
        f"pad={w}:{h}:(ow-iw)/2:(oh-ih)/2:color=black"
    )


def _demux_preflags_for_url(url: str, *, rtsp_transport: str | None) -> list[str]:
    """Input-side demux flags for FFmpeg/ffprobe before the source URL."""
    sc = _url_scheme(url)
    base = ["-err_detect", "ignore_err"]
    if sc == "rtsp" and rtsp_transport in ("tcp", "udp"):
        return ["-rtsp_transport", rtsp_transport, "-fflags", "+genpts+discardcorrupt", *base]
    if sc == "udp":
        return [
            "-fflags",
            "nobuffer+genpts+discardcorrupt",
            "-analyzeduration",
            "2500000",
            "-probesize",
            "131072",
            *base,
        ]
    return ["-fflags", "+genpts+discardcorrupt", *base]


class AiVideoHook(Protocol):
    """
    AI modules can implement this protocol and register with `VideoPipeline`.

    The hook receives decoded frames (QImage) on the GUI thread; heavy work
    should be offloaded by the hook itself.
    """

    def on_frame(self, frame: VideoFrame) -> None: ...


class CameraSource(QObject):
    """
    Wrap a single Qt camera device and emit QImage frames via QVideoSink.
    """

    frame = Signal(object)  # VideoFrame
    started = Signal()
    stopped = Signal()
    error = Signal(str)

    def __init__(self, device, *, source_id: str, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.source_id = str(source_id)
        self._device = device
        self._device_name = getattr(device, "description", lambda: str(device))()
        self._camera: Optional[QCamera] = None
        self._sink: Optional[QVideoSink] = None
        self._session: Optional[QMediaCaptureSession] = None
        self._recorder: Optional[QMediaRecorder] = None
        self._image_capture: Optional[QImageCapture] = None
        self._running = False

    @property
    def device_name(self) -> str:
        return str(self._device_name)

    def ensure_backend(self) -> bool:
        if not HAS_MULTIMEDIA:
            return False
        if self._camera is not None and self._sink is not None and self._session is not None:
            return True
        try:
            self._sink = QVideoSink(self)
            self._sink.videoFrameChanged.connect(self._on_video_frame_changed)
            self._session = QMediaCaptureSession(self)
            self._session.setVideoSink(self._sink)
            self._camera = QCamera(self._device)
            self._session.setCamera(self._camera)

            # Optional capture/record helpers (used by M3 controls).
            try:
                self._image_capture = QImageCapture(self)
                self._session.setImageCapture(self._image_capture)
            except Exception:
                self._image_capture = None
            try:
                self._recorder = QMediaRecorder(self)
                self._session.setRecorder(self._recorder)
            except Exception:
                self._recorder = None
            return True
        except Exception as e:
            self.error.emit(str(e))
            self._camera = None
            self._sink = None
            self._session = None
            self._recorder = None
            self._image_capture = None
            return False

    def start(self) -> None:
        if self._running:
            return
        if not self.ensure_backend():
            self.error.emit("Multimedia backend unavailable")
            return
        try:
            assert self._camera is not None
            self._camera.start()
            self._running = True
            self.started.emit()
        except Exception as e:
            self.error.emit(str(e))

    def stop(self) -> None:
        if not self._running:
            return
        self._running = False
        try:
            if self._camera is not None:
                self._camera.stop()
        except Exception:
            pass
        self.stopped.emit()

    def take_photo(self, filename: str) -> bool:
        """
        Capture a photo if supported. Returns whether the request was accepted.
        """
        if self._image_capture is None:
            return False
        try:
            self._image_capture.captureToFile(str(filename))
            return True
        except Exception:
            return False

    def recorder(self) -> Optional[QMediaRecorder]:
        return self._recorder

    def _on_video_frame_changed(self, frame) -> None:
        try:
            img = frame.toImage()
        except Exception:
            return
        if img is None or img.isNull():
            return
        meta = FrameMeta(
            source_id=self.source_id,
            device_name=self.device_name,
            timestamp_ms=int(getattr(frame, "startTime", lambda: 0)() / 1000)
            if hasattr(frame, "startTime")
            else 0,
        )
        self.frame.emit(VideoFrame(img, meta))


class RtspSource(QObject):
    """
    Network video stream decoded with FFmpeg when possible (RTSP, UDP MPEG-TS/h264).

    QtMultimedia is optionally used only for URLs where FFmpeg-first is disabled.
    """

    frame = Signal(object)  # VideoFrame
    started = Signal()
    stopped = Signal()
    error = Signal(str)

    def __init__(
        self,
        *,
        url: str,
        source_id: str,
        label: str,
        transport: str = "auto",
        udp_input_format: str = "",
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self.source_id = str(source_id)
        self._url = str(url or "").strip()
        self._label = str(label or source_id)
        self._transport = str(transport or "auto").strip().lower()
        self._udp_input_format = str(udp_input_format or "").strip().lower()
        self._player: Optional[QMediaPlayer] = None
        self._sink: Optional[QVideoSink] = None
        self._audio: Optional[QAudioOutput] = None
        self._pyav_thread: Optional[threading.Thread] = None
        self._pyav_stop = threading.Event()
        self._using_pyav = False
        self._last_frame_mono = 0.0
        self._ffmpeg_proc: subprocess.Popen[bytes] | None = None
        self._ffmpeg_thread: Optional[threading.Thread] = None
        self._ffmpeg_stop = threading.Event()
        self._ffmpeg_dims: tuple[int, int] | None = None
        self._ffmpeg_last_frame_mono: float = 0.0
        self._rec_proc: subprocess.Popen[bytes] | None = None
        self._running = False

    @property
    def device_name(self) -> str:
        return self._label

    def set_url(self, url: str) -> None:
        self._url = str(url or "").strip()
        if self._player is not None:
            try:
                from PySide6.QtCore import QUrl

                self._player.setSource(QUrl(self._url))
            except Exception:
                pass
        # If using FFmpeg fallback, restart on next start().

    def ensure_backend(self) -> bool:
        if not HAS_MULTIMEDIA:
            return False
        if self._player is not None and self._sink is not None:
            return True
        try:
            self._sink = QVideoSink(self)
            self._sink.videoFrameChanged.connect(self._on_video_frame_changed)
            self._player = QMediaPlayer(self)
            try:
                self._audio = QAudioOutput(self)
                try:
                    self._audio.setMuted(True)
                except Exception:
                    try:
                        self._audio.setVolume(0.0)
                    except Exception:
                        pass
                self._player.setAudioOutput(self._audio)
            except Exception:
                self._audio = None
            self._player.setVideoSink(self._sink)
            try:
                from PySide6.QtCore import QUrl

                self._player.setSource(QUrl(self._url))
            except Exception:
                pass
            try:
                self._player.errorOccurred.connect(lambda _e, msg="": self.error.emit(str(msg)))
            except Exception:
                pass
            return True
        except Exception as e:
            self.error.emit(str(e))
            self._player = None
            self._sink = None
            self._audio = None
            return False

    def _prefer_ffmpeg_immediately(self) -> bool:
        """
        Use FFmpeg for RTSP directly when tools exist.

        QtMultimedia often fails to emit frames for RTSP on Windows (FFmpeg backend)
        while still opening the stream — leading to a useless 2s wait, duplicate RTSP
        sessions, and libav h264 noise. Skip unless VGCS_FORCE_QTMULTIMEDIA_RTSP=1.
        UDP / TCP bare streams are almost never handled well by Qt; always use FFmpeg.
        """
        if os.environ.get("VGCS_FORCE_QTMULTIMEDIA_RTSP", "").strip() == "1":
            return False
        if not HAS_NUMPY:
            return False
        if shutil.which("ffmpeg") is None or shutil.which("ffprobe") is None:
            return False
        u = (self._url or "").strip().lower()
        if u.startswith("udp://") or u.startswith("tcp://"):
            return True
        return u.startswith("rtsp://")

    def start(self) -> None:
        if self._running:
            return
        if self._prefer_ffmpeg_immediately():
            print(
                f"[VGCS:video] Stream: FFmpeg decoder (skip Qt Multimedia probe) url={self._url}"
            )
            self._last_frame_mono = 0.0
            self._using_pyav = False
            self._running = True
            self._start_ffmpeg()
            return
        # Prefer QtMultimedia first; if it doesn't produce frames shortly, fall back to FFmpeg.
        if not self.ensure_backend():
            self._start_ffmpeg()
            self._running = True
            return
        try:
            assert self._player is not None
            self._using_pyav = False
            self._last_frame_mono = 0.0
            self._player.play()
            self._running = True
            self.started.emit()
            # If no frames arrive soon, try FFmpeg.
            threading.Thread(target=self._maybe_fallback_to_ffmpeg, daemon=True).start()
        except Exception as e:
            self.error.emit(str(e))

    def stop(self) -> None:
        if not self._running:
            return
        self._running = False
        try:
            if self._player is not None:
                self._player.stop()
        except Exception:
            pass
        self._stop_ffmpeg()
        self.stop_recording()
        self.stopped.emit()

    def start_recording(self, filename: str) -> bool:
        """
        Record this RTSP stream to a file using ffmpeg.
        Re-encodes to H.264 yuv420p for broad compatibility.
        """
        path = str(filename or "").strip()
        if not path:
            return False
        if shutil.which("ffmpeg") is None:
            self.error.emit("ffmpeg not found in PATH (required for recording)")
            return False
        if self._rec_proc is not None and self._rec_proc.poll() is None:
            return True
        url = str(self._url or "").strip()
        if not url:
            self.error.emit("Stream URL is empty")
            return False
        sc = _url_scheme(url)
        cmd = ["ffmpeg", "-y", "-loglevel", "error"]
        cmd.extend(_ffmpeg_udp_raw_demux(url, self._udp_input_format))
        if sc == "rtsp":
            cmd += ["-rtsp_transport", "tcp"]
        elif sc == "udp":
            cmd += [
                "-fflags",
                "nobuffer+genpts+discardcorrupt",
                "-analyzeduration",
                "1500000",
                "-probesize",
                "65536",
            ]
        cmd += [
            "-i",
            url,
            "-an",
            "-c:v",
            "libx264",
            "-preset",
            "veryfast",
            "-tune",
            "zerolatency",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            path,
        ]
        try:
            # stderr=DEVNULL: see continuous decode Popen (pipe deadlock on Windows).
            self._rec_proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                stdin=subprocess.DEVNULL,
            )
            return True
        except Exception as e:
            self.error.emit(f"Record start failed: {e}")
            self._rec_proc = None
            return False

    def stop_recording(self) -> None:
        try:
            if self._rec_proc is None:
                return
            if self._rec_proc.poll() is None:
                try:
                    self._rec_proc.kill()
                except Exception:
                    try:
                        self._rec_proc.terminate()
                    except Exception:
                        pass
                try:
                    self._rec_proc.wait(timeout=0.2)
                except Exception:
                    pass
        finally:
            self._rec_proc = None

    def _on_video_frame_changed(self, frame) -> None:
        try:
            img = frame.toImage()
        except Exception:
            return
        if img is None or img.isNull():
            return
        self._last_frame_mono = time.monotonic()
        meta = FrameMeta(
            source_id=self.source_id,
            device_name=self.device_name,
            timestamp_ms=int(getattr(frame, "startTime", lambda: 0)() / 1000)
            if hasattr(frame, "startTime")
            else 0,
        )
        self.frame.emit(VideoFrame(img, meta))

    def _maybe_fallback_to_ffmpeg(self) -> None:
        # Wait briefly for QtMultimedia to deliver frames (no Qt API calls here — this runs
        # on a plain Python thread started in start()).
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.0:
            if self._last_frame_mono > 0:
                return
            time.sleep(0.05)
        # Stopping QMediaPlayer from this worker thread triggers
        # "QObject::killTimer: Timers cannot be stopped from another thread".
        # Marshal stop + FFmpeg handoff onto the thread that owns this QObject.
        QMetaObject.invokeMethod(
            self,
            "_qt_apply_ffmpeg_fallback",
            Qt.ConnectionType.QueuedConnection,
        )

    @Slot()
    def _qt_apply_ffmpeg_fallback(self) -> None:
        if not self._running:
            return
        # Frames may have started arriving before this slot runs.
        if self._last_frame_mono > 0:
            return
        print(
            f"[VGCS:video] QtMultimedia produced no frames; switching to FFmpeg fallback url={self._url}"
        )
        try:
            if self._player is not None:
                self._player.stop()
        except Exception:
            pass
        self._start_ffmpeg()

    def _start_ffmpeg(self) -> None:
        if not HAS_NUMPY:
            self.error.emit("numpy missing; FFmpeg RTSP fallback unavailable")
            return
        if self._ffmpeg_thread is not None and self._ffmpeg_thread.is_alive():
            return
        # Ensure QtMultimedia RTSP session is not still holding the stream.
        # Some RTSP servers reject/limit concurrent sessions per client.
        try:
            if self._player is not None:
                self._player.stop()
        except Exception:
            pass
        url = str(self._url or "").strip()
        if not url:
            self.error.emit("Stream URL is empty")
            return
        if shutil.which("ffmpeg") is None or shutil.which("ffprobe") is None:
            self.error.emit("ffmpeg/ffprobe not found in PATH (required for network video decode)")
            return
        self._ffmpeg_stop.clear()
        self._ffmpeg_thread = threading.Thread(target=self._ffmpeg_loop, daemon=True)
        self._ffmpeg_thread.start()
        self.started.emit()

    def _stop_ffmpeg(self) -> None:
        try:
            self._ffmpeg_stop.set()
        except Exception:
            pass
        self._close_ffmpeg_decode_proc()

    def _close_ffmpeg_decode_proc(self) -> None:
        """Terminate the continuous decode child only (does not set `_ffmpeg_stop`)."""
        p = self._ffmpeg_proc
        self._ffmpeg_proc = None
        if p is None:
            return
        try:
            out = p.stdout
            if out is not None:
                try:
                    out.close()
                except Exception:
                    pass
        except Exception:
            pass
        try:
            if p.poll() is None:
                # Do not block the GUI thread on wait(): Windows FFmpeg can take seconds to
                # exit cleanly after RTSP teardown, which freezes Application Settings → Apply.
                try:
                    p.kill()
                except Exception:
                    try:
                        p.terminate()
                    except Exception:
                        pass
                try:
                    p.wait(timeout=0.15)
                except Exception:
                    pass
        except Exception:
            pass

    def _ffprobe_dims(
        self, url: str, *, rtsp_transport: str | None = None, udp_input_format: str = ""
    ) -> tuple[int, int] | None:
        u = str(url or "").strip()
        if not u:
            return None
        sc = _url_scheme(u)
        is_rtsp = sc == "rtsp"
        tr = rtsp_transport if is_rtsp else None
        timeout_s = 12.0 if sc == "udp" else 8.5
        try:
            demux_hint = _ffmpeg_udp_raw_demux(u, udp_input_format or self._udp_input_format)
            probe_opts = _demux_preflags_for_url(u, rtsp_transport=tr)
            cmd = [
                "ffprobe",
                "-v",
                "error",
                *demux_hint,
                *probe_opts,
                "-select_streams",
                "v:0",
                "-show_entries",
                "stream=width,height",
                "-of",
                "json",
                u,
            ]
            p = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=timeout_s,
            )
            if p.returncode != 0:
                return None
            data = json.loads(p.stdout or "{}")
            streams = data.get("streams") or []
            if not streams:
                return None
            w = int(streams[0].get("width") or 0)
            h = int(streams[0].get("height") or 0)
            if w > 0 and h > 0:
                return (w, h)
            return None
        except Exception:
            return None

    def _read_exact(self, stream, n: int) -> bytes | None:
        buf = bytearray()
        while len(buf) < n and not self._ffmpeg_stop.is_set():
            chunk = stream.read(n - len(buf))
            if not chunk:
                return None
            buf.extend(chunk)
        return bytes(buf) if len(buf) == n else None

    def _ffmpeg_loop(self) -> None:
        url = str(self._url or "").strip()
        transport_for_probe = _rtsp_transport_sequence(url, self._transport)
        dims: tuple[int, int] | None = None
        # ffprobe opens its own RTSP session. Many companion cameras (e.g. 192.168.144.x)
        # accept only one client or stall the *next* decoder — preview stays black. Skip
        # by default; use 1280x720 canvas (scale+pad still matches the real frame).
        # Set VGCS_RTSP_FFPROBE=1 to restore probing for exotic URLs.
        if _url_scheme(url) == "rtsp" and str(os.environ.get("VGCS_RTSP_FFPROBE", "") or "").strip() != "1":
            print("[VGCS:video] RTSP: skip ffprobe (set VGCS_RTSP_FFPROBE=1 to enable size probe)")
        elif transport_for_probe:
            dims = self._ffprobe_dims(
                url,
                rtsp_transport=transport_for_probe[0],
                udp_input_format=self._udp_input_format,
            )
        dims = dims or (1280, 720)
        src_w, src_h = dims
        src_w = max(1, int(src_w))
        src_h = max(1, int(src_h))
        cap_w = int(os.environ.get("VGCS_VIDEO_DECODE_MAX_W", "1920") or 1920)
        cap_h = int(os.environ.get("VGCS_VIDEO_DECODE_MAX_H", "1080") or 1080)
        cap_w = max(640, min(1920, cap_w))
        cap_h = max(360, min(1080, cap_h))
        scale = min(float(cap_w) / float(src_w), float(cap_h) / float(src_h), 1.0)
        w = max(2, int(round(src_w * scale)))
        h = max(2, int(round(src_h * scale)))
        if (w % 2) != 0:
            w -= 1
        if (h % 2) != 0:
            h -= 1
        w = max(2, w)
        h = max(2, h)
        self._ffmpeg_dims = (w, h)
        frame_bytes = int(w) * int(h) * 3
        vf_rgb = _ffmpeg_vf_rgb_fixed_size(w, h)

        # UI emit cap: avoid unbounded QueuedConnection backlog on the GUI thread.
        # Default 20 Hz — preview still looks smooth; higher rates + map work freeze Windows
        # ("Not Responding") on typical laptops. Override with VGCS_VIDEO_PREVIEW_MAX_FPS.
        try:
            max_fps = float(str(os.environ.get("VGCS_VIDEO_PREVIEW_MAX_FPS", "20") or "20").strip())
        except Exception:
            max_fps = 20.0
        max_fps = max(8.0, min(60.0, max_fps))
        min_emit_dt = 1.0 / max_fps
        last_emit_mono = 0.0

        # If every transport fails once, wait before another full pass (encoder cooldown).
        round_backoff_s = 0.6
        max_round_backoff_s = 4.0

        while self._running and not self._ffmpeg_stop.is_set():
            transport_seq = _rtsp_transport_sequence(url, self._transport)
            round_ok = False
            for transport in transport_seq:
                if self._ffmpeg_stop.is_set() or not self._running:
                    break
                demux = _ffmpeg_udp_raw_demux(url, self._udp_input_format)
                tr_label = str(transport) if transport is not None else "n/a"
                print(f"[VGCS:video] FFmpeg decode try rtsp_transport={tr_label} url={url}")

                if _url_scheme(url) == "rtsp":
                    # Cooldown only needed when ffprobe ran (second session). Skip long wait
                    # when we go straight to decode (VGCS_RTSP_FFPROBE unset).
                    if str(os.environ.get("VGCS_RTSP_FFPROBE", "") or "").strip() == "1":
                        time.sleep(0.2)
                    else:
                        time.sleep(0.05)

                cmd_base = [
                    "ffmpeg",
                    "-hide_banner",
                    "-nostats",
                    "-loglevel",
                    "error",
                    *demux,
                    *_ffmpeg_preflags_before_input(url, rtsp_transport=transport),
                    "-i",
                    url,
                    "-an",
                    "-vf",
                    vf_rgb,
                    "-pix_fmt",
                    "rgb24",
                    "-f",
                    "rawvideo",
                    "pipe:1",
                ]

                reconnect_delay = 0.6
                empty_session_limit = 20
                empty_sessions = 0

                while self._running and not self._ffmpeg_stop.is_set():
                    try:
                        self._ffmpeg_proc = subprocess.Popen(
                            cmd_base,
                            stdout=subprocess.PIPE,
                            stderr=subprocess.DEVNULL,
                            stdin=subprocess.DEVNULL,
                        )
                    except Exception as e:
                        self.error.emit(f"FFmpeg start (continuous) failed: {e}")
                        self._ffmpeg_proc = None
                        empty_sessions += 1
                        if empty_sessions >= empty_session_limit:
                            break
                        time.sleep(reconnect_delay)
                        continue

                    if self._ffmpeg_proc.stdout is None:
                        self._close_ffmpeg_decode_proc()
                        empty_sessions += 1
                        if empty_sessions >= empty_session_limit:
                            break
                        time.sleep(reconnect_delay)
                        continue

                    frames_this_session = 0
                    while self._running and not self._ffmpeg_stop.is_set():
                        raw = self._read_exact(self._ffmpeg_proc.stdout, frame_bytes)
                        if raw is None:
                            break
                        # Drop frames *before* QImage/numpy decode. Previously we built a full QImage
                        # copy for every FFmpeg output frame then throttled emit — the CPU cost alone
                        # could starve the GUI thread (QueuedConnection backlog + map MAVLink work).
                        now = time.monotonic()
                        if (now - last_emit_mono) < min_emit_dt:
                            continue
                        try:
                            assert np is not None
                            arr = np.frombuffer(raw, dtype=np.uint8).reshape((h, w, 3))
                            qimg = QImage(arr.data, w, h, 3 * w, QImage.Format.Format_RGB888).copy()
                            last_emit_mono = time.monotonic()
                            meta = FrameMeta(
                                source_id=self.source_id,
                                device_name=self.device_name,
                                timestamp_ms=0,
                            )
                            self.frame.emit(VideoFrame(qimg, meta))
                            self._ffmpeg_last_frame_mono = last_emit_mono
                            frames_this_session += 1
                            round_ok = True
                        except Exception:
                            continue

                    proc = self._ffmpeg_proc
                    rc: int | None = None
                    try:
                        if proc is not None:
                            rc = proc.poll()
                    except Exception:
                        rc = None
                    self._close_ffmpeg_decode_proc()

                    if self._ffmpeg_stop.is_set() or not self._running:
                        break

                    if frames_this_session > 0:
                        empty_sessions = 0
                        print(
                            f"[VGCS:video] decode session ended (frames={frames_this_session}), "
                            f"reconnecting same transport={tr_label} rc={rc!r}"
                        )
                        time.sleep(reconnect_delay)
                        continue

                    empty_sessions += 1
                    if empty_sessions >= empty_session_limit:
                        print(
                            f"[VGCS:video] decode: no frames after {empty_session_limit} attempts "
                            f"on transport={tr_label}, trying next"
                        )
                        break
                    time.sleep(reconnect_delay)

                if self._ffmpeg_stop.is_set() or not self._running:
                    break

            if self._ffmpeg_stop.is_set() or not self._running:
                break
            if round_ok:
                round_backoff_s = 0.6
            else:
                print(
                    f"[VGCS:video] all transports exhausted for this pass; "
                    f"cooling down {round_backoff_s:.1f}s before retry"
                )
                time.sleep(round_backoff_s)
                round_backoff_s = min(max_round_backoff_s, round_backoff_s * 1.35)

        # User called stop() or thread tear-down; only close the child.
        self._close_ffmpeg_decode_proc()


class VideoPipeline(QObject):
    """
    Central registry of video sources + extension hook for future AI tracking.
    """

    sources_changed = Signal()
    active_source_changed = Signal(str)
    frame = Signal(object)  # VideoFrame (from active source)

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._sources: dict[str, object] = {}
        self._active_source_id: str = ""
        self._hooks: list[AiVideoHook] = []
        self._rtsp_day_url: str = ""
        self._rtsp_thermal_url: str = ""
        self._rtsp_transport: str = "auto"
        # Application Settings → Video → Source (`rtsp` | `udp_h264` | `udp_h265`).
        self._stream_kind: str = "rtsp"

        self.refresh_sources()

    def refresh_sources(self) -> None:
        # Always stop + drop old sources. Replacing the dict alone leaves QObject children
        # and FFmpeg threads alive → duplicate RTSP sessions, CPU spikes, and "Python is not
        # responding" when saving Video settings while connected.
        old_items = list(self._sources.items())
        self._sources = {}
        for _sid, src in old_items:
            try:
                if hasattr(src, "frame") and hasattr(src.frame, "disconnect"):
                    try:
                        src.frame.disconnect()
                    except Exception:
                        pass
            except Exception:
                pass
            try:
                if hasattr(src, "stop"):
                    src.stop()
            except Exception:
                pass
            try:
                src.deleteLater()
            except Exception:
                pass
        kind = str(self._stream_kind or "rtsp").strip().lower()
        udp_demux = ""
        day_lbl_base = "RTSP"
        if kind == "udp_h264":
            udp_demux = "h264"
            day_lbl_base = "UDP h.264"
        elif kind == "udp_h265":
            udp_demux = "hevc"
            day_lbl_base = "UDP h.265"
        if HAS_MULTIMEDIA and (self._rtsp_day_url or self._rtsp_thermal_url):
            if self._rtsp_day_url:
                day = RtspSource(
                    url=self._rtsp_day_url,
                    source_id="day",
                    label=f"Day ({day_lbl_base})",
                    transport=self._rtsp_transport,
                    udp_input_format=udp_demux,
                    parent=self,
                )
                day.frame.connect(self._on_source_frame, Qt.ConnectionType.QueuedConnection)
                self._sources["day"] = day
            if self._rtsp_thermal_url:
                th = RtspSource(
                    url=self._rtsp_thermal_url,
                    source_id="thermal",
                    label=f"Thermal ({day_lbl_base})",
                    transport=self._rtsp_transport,
                    udp_input_format=udp_demux,
                    parent=self,
                )
                th.frame.connect(self._on_source_frame, Qt.ConnectionType.QueuedConnection)
                self._sources["thermal"] = th
        if HAS_MULTIMEDIA and QMediaDevices is not None:
            try:
                devices = list(QMediaDevices.videoInputs())
            except Exception:
                devices = []
            for i, dev in enumerate(devices):
                src_id = f"cam{i}"
                src = CameraSource(dev, source_id=src_id, parent=self)
                src.frame.connect(self._on_source_frame, Qt.ConnectionType.QueuedConnection)
                self._sources[src_id] = src
        if not self._active_source_id and self._sources:
            self._active_source_id = next(iter(self._sources.keys()))
        self.sources_changed.emit()

    def set_rtsp_sources(
        self,
        *,
        day_url: str = "",
        thermal_url: str = "",
        transport: str = "auto",
        stream_kind: str = "rtsp",
    ) -> None:
        self._rtsp_day_url = str(day_url or "").strip()
        self._rtsp_thermal_url = str(thermal_url or "").strip()
        self._rtsp_transport = str(transport or "auto").strip().lower()
        sk = str(stream_kind or "rtsp").strip().lower()
        self._stream_kind = sk if sk in ("rtsp", "udp_h264", "udp_h265") else "rtsp"
        # Reset active source if it no longer exists.
        if self._active_source_id and self._active_source_id not in self._sources:
            self._active_source_id = ""
        self.refresh_sources()

    def sources(self) -> dict[str, object]:
        return dict(self._sources)

    def active_source_id(self) -> str:
        return str(self._active_source_id)

    def active_source(self) -> Optional[object]:
        return self._sources.get(self._active_source_id)

    def set_active_source(self, source_id: str) -> None:
        source_id = str(source_id or "")
        if source_id == self._active_source_id:
            return
        if source_id and source_id not in self._sources:
            return
        self._active_source_id = source_id
        self.active_source_changed.emit(source_id)

    def register_ai_hook(self, hook: AiVideoHook) -> None:
        if hook in self._hooks:
            return
        self._hooks.append(hook)

    def unregister_ai_hook(self, hook: AiVideoHook) -> None:
        try:
            self._hooks.remove(hook)
        except ValueError:
            pass

    def _on_source_frame(self, frame: VideoFrame) -> None:
        if frame.meta.source_id != self._active_source_id:
            return
        self.frame.emit(frame)
        for hook in list(self._hooks):
            try:
                hook.on_frame(frame)
            except Exception:
                # Hooks are best-effort; never crash the UI.
                pass

