from __future__ import annotations

from dataclasses import dataclass
import ipaddress
import os
from pathlib import Path
import threading
import time
import json
from urllib.parse import urlparse
import shutil
import subprocess
from typing import Optional, Protocol

# Common Herelink / Skydroid / IRCam companion Wi‑Fi video subnet (RTSP host lives here).
_COMPANION_RTSP_IPV4 = ipaddress.ip_network("192.168.144.0/24")

from PySide6.QtCore import QObject, QTimer, Signal, QMetaObject, Qt, Slot
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


def wait_qmedia_recorder_stopped(rec, *, timeout_s: float = 20.0) -> bool:
    """
    ``QMediaRecorder.stop()`` is asynchronous; the output file may be incomplete
    until the recorder returns to ``StoppedState`` (symptom: tiny / unplayable MP4).
    """
    if not HAS_MULTIMEDIA or rec is None:
        return True
    try:
        from PySide6.QtWidgets import QApplication

        stopped = QMediaRecorder.RecorderState.StoppedState
    except Exception:
        return True
    deadline = time.monotonic() + float(timeout_s)
    while time.monotonic() < deadline:
        try:
            if rec.recorderState() == stopped:
                return True
        except Exception:
            return True
        try:
            QApplication.processEvents()
        except Exception:
            pass
        time.sleep(0.02)
    return False


def suggested_recording_filename(extension: str = "mp4") -> str:
    """Default basename for saved screen/RTSP recordings (aligned with ``photo_YYYYMMDD_HHMMSS``)."""
    ext = str(extension or "mp4").strip().lower().lstrip(".")
    if ext not in ("mp4", "mov", "mkv"):
        ext = "mp4"
    return f"recording_{time.strftime('%Y%m%d_%H%M%S')}.{ext}"


def suggested_recording_save_path(directory: str | Path | None = None) -> str:
    """Full path for QFileDialog default when saving a recording."""
    d = Path.cwd() if directory is None else Path(directory)
    return str(d / suggested_recording_filename())


def suggested_photo_filename(extension: str = "jpg") -> str:
    ext = str(extension or "jpg").strip().lower().lstrip(".")
    if ext in ("jpeg",):
        ext = "jpg"
    if ext not in ("jpg", "png"):
        ext = "jpg"
    return f"photo_{time.strftime('%Y%m%d_%H%M%S')}.{ext}"


def suggested_photo_save_path(directory: str | Path | None = None, *, extension: str = "jpg") -> str:
    """Full path for QFileDialog default when saving a still photo."""
    d = Path.cwd() if directory is None else Path(directory)
    return str(d / suggested_photo_filename(extension=extension))


# QSettings key (VGCS / VGCS): last directory used in the photo Save dialog.
QS_KEY_LAST_PHOTO_SAVE_DIR = "media/last_photo_save_dir"


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
    Pick RTSP RTP transport order for FFmpeg.

    Many cloud / generic RTSP relays work best with UDP RTP first, then TCP interleaved.

    RFC1918 *generally* prefers TCP first (NAT / flaky Wi‑Fi). The companion subnet
    ``192.168.144.0/24`` matches **VLC/ffplay defaults** for many drone cameras: **TCP
    interleaved first**, then UDP RTP as a fallback (UDP-first used to stall forever on
    hosts that only answer TCP, so the second transport never ran).

    Loopback stays TCP-first (local mediamtx / ffmpeg publishers).
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
        if ip.is_loopback or ip.is_link_local:
            return ("tcp", "udp")
        if ip.version == 4 and ip in _COMPANION_RTSP_IPV4:
            return ("tcp", "udp")
        if ip.is_private:
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
        # Prefer operator choice, then the other RTP mode (many stacks differ).
        return ("udp", "tcp")
    if m == "tcp":
        return ("tcp", "udp")
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


def _rtsp_url_is_companion_link_subnet(url: str) -> bool:
    """True for rtsp://192.168.144.x/... on the usual drone companion Wi‑Fi."""
    try:
        pu = urlparse(str(url or "").strip())
        if str(pu.scheme or "").strip().lower() != "rtsp":
            return False
        ip = ipaddress.ip_address((pu.hostname or "").strip("[]"))
        return ip.version == 4 and ip in _COMPANION_RTSP_IPV4
    except Exception:
        return False


def _rtsp_url_is_siyi_style(url: str) -> bool:
    """SIYI ZR10 / ZT30 style paths on the companion link (main.264, video0–3)."""
    u = str(url or "").strip()
    if _url_scheme(u) != "rtsp" or not _rtsp_url_is_companion_link_subnet(u):
        return False
    try:
        path = (urlparse(u).path or "").lower()
    except Exception:
        return False
    return any(
        token in path
        for token in ("main.264", "video0", "video1", "video2", "video3")
    )


def _rtsp_should_ffprobe(url: str) -> bool:
    """Whether to open a short ffprobe session before decode (opt-in only).

    Auto ffprobe on SIYI URLs was disabled: it opens a second RTSP session and takes
    several seconds — combined with preview restarts that prevented any frame from arriving.
    Decode uses a 1280×720 canvas with scale+pad (``_ffmpeg_vf_rgb_fixed_size``).
    Set ``VGCS_RTSP_FFPROBE=1`` to probe dimensions explicitly.
    """
    flag = str(os.environ.get("VGCS_RTSP_FFPROBE", "") or "").strip().lower()
    if flag in ("0", "false", "no"):
        return False
    return flag in ("1", "true", "yes")


def _rtsp_socket_timeout_us(url: str) -> int:
    """
    FFmpeg RTSP demuxer ``-timeout`` (microseconds) for socket I/O.

    SIYI ZR10 and similar companion cameras can pause RTP while keeping TCP open;
  without this, the decode thread blocks on ``stdout.read()`` until the camera drops
    the session (~20–30 s), which looks like a frozen preview.
    Set ``VGCS_RTSP_IO_TIMEOUT_US=0`` to disable.
    """
    raw = str(os.environ.get("VGCS_RTSP_IO_TIMEOUT_US", "") or "").strip()
    if raw == "0":
        return 0
    if raw:
        try:
            return max(0, int(raw))
        except ValueError:
            pass
    if _url_scheme(str(url or "").strip()) != "rtsp":
        return 0
    # SIYI ZR10: do not pass demuxer -timeout — 4s caused Windows error -138 on reconnect
    # while the camera was still releasing the prior TCP session.
    if _rtsp_url_is_siyi_style(url):
        return 0
    if _rtsp_url_is_companion_link_subnet(url):
        return 8_000_000
    return 5_000_000


def _stall_watchdog_enabled(url: str) -> bool:
    """SIYI HEVC often blocks inside FFmpeg for >6s while healthy; killing causes freeze loops."""
    if _rtsp_url_is_siyi_style(url):
        return str(os.environ.get("VGCS_SIYI_STALL_WATCHDOG", "0") or "0").strip() == "1"
    return True


def _video_stall_reconnect_s(url: str) -> float:
    """Max seconds without a full raw frame (frozen-duplicate path; watchdog uses same value)."""
    raw = str(os.environ.get("VGCS_VIDEO_STALL_RECONNECT_S", "") or "").strip()
    if raw:
        try:
            return max(0.8, min(30.0, float(raw)))
        except ValueError:
            pass
    if _rtsp_url_is_siyi_style(url):
        return 12.0
    if _rtsp_url_is_companion_link_subnet(url):
        return 3.0
    return 3.0


def _rtsp_url_is_loopback(url: str) -> bool:
    """True for rtsp://127.0.0.1/... etc. Local publishers often misbehave with aggressive RTSP flags."""
    try:
        pu = urlparse(str(url or "").strip())
        if str(pu.scheme or "").strip().lower() != "rtsp":
            return False
        hn = (pu.hostname or "").strip().lower()
        if hn in ("localhost", "127.0.0.1", "::1"):
            return True
        ip = ipaddress.ip_address(hn.strip("[]"))
        return bool(ip.is_loopback)
    except Exception:
        return False


def _ffmpeg_preflags_before_input(
    url: str, *, rtsp_transport: str | None, low_latency: bool = False
) -> list[str]:
    """Flags immediately before `-i` (after optional `-f`)."""
    sc = _url_scheme(str(url or "").strip())
    out: list[str] = []
    if sc == "rtsp" and rtsp_transport in ("udp", "tcp"):
        out.extend(["-rtsp_transport", rtsp_transport])
        u = str(url or "").strip()
        io_us = _rtsp_socket_timeout_us(u)
        if io_us > 0:
            # Demuxer option (accepted on Windows gyan.dev builds); unblocks stalled SIYI ZR10 RTP.
            out.extend(["-timeout", str(io_us)])
        # Do not pass global `-rw_timeout` / `-stimeout`: several Windows FFmpeg builds reject them.
        # Local RTSP (mediamtx, ffmpeg publish) often needs a slightly longer demux window;
        # aggressive `nobuffer` can yield zero frames while ffplay still works.
        # Same for 192.168.144.x companion cameras: aggressive nobuffer can miss the first GOP.
        # Application Settings → Video → Low latency forces the aggressive path everywhere.
        if low_latency or not (
            _rtsp_url_is_loopback(u) or _rtsp_url_is_companion_link_subnet(u)
        ):
            out.extend(
                [
                    "-fflags",
                    "+genpts+discardcorrupt",
                    "-analyzeduration",
                    "8000000",
                    "-probesize",
                    "8000000",
                    "-flags",
                    "low_delay",
                ]
            )
        else:
            if _rtsp_url_is_siyi_style(u):
                # SIYI ZR10: gentler demux + igndts (nobuffer alone can stutter then freeze).
                out.extend(
                    [
                        "-fflags",
                        "+genpts+discardcorrupt+igndts",
                        "-analyzeduration",
                        "8000000",
                        "-probesize",
                        "8000000",
                        "-flags",
                        "low_delay",
                    ]
                )
            else:
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
        if _rtsp_url_is_siyi_style(u):
            out.extend(["-thread_queue_size", "512"])
            out.extend(["-ec", "guess_mvs+deblock"])
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
        low_latency: bool = False,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self.source_id = str(source_id)
        self._url = str(url or "").strip()
        self._label = str(label or source_id)
        self._transport = str(transport or "auto").strip().lower()
        self._udp_input_format = str(udp_input_format or "").strip().lower()
        self._low_latency = bool(low_latency)
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
        self._ffmpeg_last_decode_mono: float = 0.0
        self._ffmpeg_last_raw_sig: bytes | None = None
        self._ffmpeg_last_raw_change_mono: float = 0.0
        self._ffmpeg_session_started_mono: float = 0.0
        self._rec_proc: subprocess.Popen[bytes] | None = None
        self._rec_lock = threading.Lock()
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
            # Recover after a partial stop (decode thread exited but preview still "on").
            if self._prefer_ffmpeg_immediately():
                th = self._ffmpeg_thread
                if th is None or not th.is_alive():
                    self._start_ffmpeg()
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
        # Wake the decode thread immediately; never block the GUI thread on
        # `QMediaPlayer.stop()`, pipe teardown, or subprocess cleanup — those
        # routinely stall for seconds on Windows when RTSP is unreachable.
        try:
            self._ffmpeg_stop.set()
        except Exception:
            pass
        try:
            QMetaObject.invokeMethod(
                self,
                "_qt_stop_teardown",
                Qt.ConnectionType.QueuedConnection,
            )
        except Exception:
            self._qt_stop_teardown()

    def start_recording(self, filename: str) -> bool:
        """
        Record the **same** decoded RGB stream used for preview.

        A second FFmpeg pulling the RTSP URL directly often fails on companion cameras
        that only allow **one** RTSP client (symptom: a tiny broken ``.mp4``).
        """
        path = str(filename or "").strip()
        if not path:
            return False
        if shutil.which("ffmpeg") is None:
            self.error.emit("ffmpeg not found in PATH (required for recording)")
            return False
        with self._rec_lock:
            if self._rec_proc is not None and self._rec_proc.poll() is None:
                return True

        dec = self._ffmpeg_proc
        if dec is None or dec.poll() is not None:
            self.error.emit(
                "Recording needs an active live preview first (many cameras allow only one RTSP session)"
            )
            return False

        w, h = self._ffmpeg_dims or (1280, 720)
        w = max(2, int(w))
        h = max(2, int(h))
        if (w % 2) != 0:
            w -= 1
        if (h % 2) != 0:
            h -= 1
        w = max(2, w)
        h = max(2, h)

        try:
            fps = float(str(os.environ.get("VGCS_RECORD_RAW_FRAMERATE", "30") or "30").strip())
        except Exception:
            fps = 30.0
        fps = max(10.0, min(120.0, fps))

        cmd = [
            "ffmpeg",
            "-y",
            "-hide_banner",
            "-loglevel",
            "error",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "rgb24",
            "-video_size",
            f"{w}x{h}",
            "-framerate",
            str(fps),
            "-i",
            "pipe:0",
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
            proc = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )
        except Exception as e:
            self.error.emit(f"Record start failed: {e}")
            return False

        time.sleep(0.12)
        if proc.poll() is not None:
            err = ""
            try:
                if proc.stderr is not None:
                    err = (proc.stderr.read() or b"").decode("utf-8", errors="replace")[-800:]
            except Exception:
                pass
            self.error.emit(
                "Recording encoder exited immediately. "
                + (err.strip() or f"(ffmpeg exit code {proc.returncode})")
            )
            try:
                proc.stderr.close()
            except Exception:
                pass
            return False

        with self._rec_lock:
            self._rec_proc = proc
        return True

    def stop_recording(self) -> None:
        with self._rec_lock:
            p = self._rec_proc
            self._rec_proc = None
        if p is None:
            return
        try:
            sin = p.stdin
            if sin is not None:
                try:
                    sin.close()
                except Exception:
                    pass
            deadline = time.monotonic() + 45.0
            while p.poll() is None and time.monotonic() < deadline:
                time.sleep(0.05)
            if p.poll() is None:
                try:
                    p.terminate()
                except Exception:
                    pass
                deadline2 = time.monotonic() + 3.0
                while p.poll() is None and time.monotonic() < deadline2:
                    time.sleep(0.05)
            if p.poll() is None:
                try:
                    p.kill()
                except Exception:
                    pass
            try:
                p.wait(timeout=5)
            except Exception:
                pass
            try:
                if p.stderr is not None:
                    p.stderr.close()
            except Exception:
                pass
        except Exception:
            pass

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
            # Prior stop() may still be in ffprobe/teardown; end it before a new session.
            try:
                self._ffmpeg_stop.set()
            except Exception:
                pass
            try:
                self._close_ffmpeg_decode_proc()
            except Exception:
                pass
            try:
                self._ffmpeg_thread.join(timeout=3.0)
            except Exception:
                pass
            self._ffmpeg_thread = None
            try:
                self._ffmpeg_stop.clear()
            except Exception:
                pass
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

    @Slot()
    def _qt_stop_teardown(self) -> None:
        """Finish teardown on the QObject's thread (GUI): player, FFmpeg child, recording."""
        try:
            if self._player is not None:
                self._player.stop()
        except Exception:
            pass
        self._stop_ffmpeg_decode_proc_deferred()
        try:
            self.stop_recording()
        except Exception:
            pass
        try:
            self.stopped.emit()
        except Exception:
            pass

    def _stop_ffmpeg_decode_proc_deferred(self) -> None:
        """Queue `_close_ffmpeg_decode_proc` on this object's thread (safe from `stop()`)."""
        try:
            QMetaObject.invokeMethod(
                self,
                "_qt_close_ffmpeg_decode_proc",
                Qt.ConnectionType.QueuedConnection,
            )
        except Exception:
            self._close_ffmpeg_decode_proc()

    @Slot()
    def _qt_close_ffmpeg_decode_proc(self) -> None:
        self._close_ffmpeg_decode_proc()

    def _close_ffmpeg_decode_proc(self) -> None:
        """Terminate the continuous decode child only (does not set `_ffmpeg_stop`)."""
        p = self._ffmpeg_proc
        self._ffmpeg_proc = None
        if p is None:
            return
        # On Windows, closing the pipe before the child exits can block the calling thread.
        # Kill first so the reader thread wakes, then close the handle.
        try:
            if p.poll() is None:
                try:
                    p.kill()
                except Exception:
                    try:
                        p.terminate()
                    except Exception:
                        pass
        except Exception:
            pass
        try:
            out = p.stdout
            if out is not None:
                try:
                    out.close()
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

    def _udp_demux_try_sequence(self, url: str) -> list[str]:
        """UDP demux candidates: raw Annex B first, then MPEG-TS (common on companion links)."""
        if _url_scheme(str(url or "").strip()) != "udp":
            return [""]
        fmt = str(self._udp_input_format or "").strip().lower()
        if fmt in ("hevc", "265", "h265"):
            return ["hevc", "mpegts", ""]
        if fmt in ("h264", "264", "avc"):
            return ["h264", "mpegts", ""]
        return [fmt] if fmt else [""]

    def _emit_video_frame_safe(self, qimg: QImage, meta: FrameMeta) -> bool:
        """Emit on the decode thread; RtspSource may be deleted during settings Apply."""
        if not self._running or self._ffmpeg_stop.is_set():
            return False
        try:
            self.frame.emit(VideoFrame(qimg, meta))
            return True
        except RuntimeError:
            return False

    def _read_exact(self, stream, n: int) -> bytes | None:
        buf = bytearray()
        while len(buf) < n and not self._ffmpeg_stop.is_set():
            try:
                chunk = stream.read(n - len(buf))
            except Exception:
                return None
            if not chunk:
                return None
            buf.extend(chunk)
        return bytes(buf) if len(buf) == n else None

    def _start_ffmpeg_stall_watchdog(
        self,
        proc: subprocess.Popen[bytes],
        *,
        stall_s: float,
        transport_label: str,
        session_stop: threading.Event,
        frames_counter: list[int],
    ) -> threading.Thread:
        """
        Kill the decode child if no preview frame was emitted for *stall_s* seconds.

        ``select`` on pipe fds is not reliable on Windows, so we unblock stuck reads by
        terminating FFmpeg when SIYI ZR10 (and similar) pause RTP while keeping TCP open.
        """

        def _run() -> None:
            grace = max(0.8, float(stall_s))
            session_t0 = time.monotonic()
            while (
                not self._ffmpeg_stop.is_set()
                and not session_stop.is_set()
                and proc.poll() is None
            ):
                time.sleep(0.2)
                if int(frames_counter[0]) < 30:
                    if time.monotonic() - session_t0 < grace * 4.0:
                        continue
                    return
                sess0 = float(self._ffmpeg_session_started_mono or 0.0)
                if sess0 > 0.0 and time.monotonic() - sess0 < 8.0:
                    continue
                last = float(self._ffmpeg_last_decode_mono or self._ffmpeg_last_frame_mono or 0.0)
                if last <= 0.0:
                    continue
                if time.monotonic() - last < grace:
                    continue
                try:
                    print(
                        f"[VGCS:video] decode stall (no decoded frame for {grace:.1f}s), "
                        f"reconnecting transport={transport_label}"
                    )
                except Exception:
                    pass
                session_stop.set()
                try:
                    if proc.poll() is None:
                        proc.kill()
                except Exception:
                    pass
                return

        t = threading.Thread(target=_run, daemon=True)
        t.start()
        return t

    def _ffmpeg_loop(self) -> None:
        url = str(self._url or "").strip()
        transport_for_probe = _rtsp_transport_sequence(url, self._transport)
        dims: tuple[int, int] | None = None
        # ffprobe opens its own RTSP session. Skipped for generic companion URLs; enabled
        # automatically for SIYI ZR10-style paths (main.264 / video0–3). Override with
        # VGCS_RTSP_FFPROBE=0|1.
        if _url_scheme(url) == "rtsp" and not _rtsp_should_ffprobe(url):
            print(
                "[VGCS:video] RTSP: skip ffprobe "
                "(set VGCS_RTSP_FFPROBE=1 to force, auto for SIYI main.264/videoN)"
            )
        elif transport_for_probe:
            if _rtsp_url_is_siyi_style(url):
                print("[VGCS:video] RTSP: SIYI stream — probing video dimensions")
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
        if _rtsp_url_is_siyi_style(url):
            # ZR10 main.264 is often HEVC 1080p+; lighter preview decode = smoother pan on laptops.
            cap_w = min(cap_w, int(os.environ.get("VGCS_SIYI_DECODE_MAX_W", "960") or 960))
            cap_h = min(cap_h, int(os.environ.get("VGCS_SIYI_DECODE_MAX_H", "540") or 540))
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
            default_fps = "25" if _rtsp_url_is_siyi_style(url) else "20"
            max_fps = float(
                str(os.environ.get("VGCS_VIDEO_PREVIEW_MAX_FPS", default_fps) or default_fps).strip()
            )
        except Exception:
            max_fps = 25.0 if _rtsp_url_is_siyi_style(url) else 20.0
        max_fps = max(8.0, min(60.0, max_fps))
        min_emit_dt = 1.0 / max_fps
        last_emit_mono = 0.0
        stall_reconnect_s = _video_stall_reconnect_s(url)

        # If every transport fails once, wait before another full pass (encoder cooldown).
        round_backoff_s = 0.6
        max_round_backoff_s = 4.0

        if _rtsp_url_is_companion_link_subnet(url):
            print(
                f"[VGCS:video] RTSP companion link (192.168.144.x): transport order="
                f"{_rtsp_transport_sequence(url, self._transport)!r} (TCP interleaved first)"
            )
            if _rtsp_url_is_siyi_style(url) and not _stall_watchdog_enabled(url):
                print(
                    "[VGCS:video] SIYI ZR10: stall watchdog disabled "
                    "(HEVC decode gaps are normal; set VGCS_SIYI_STALL_WATCHDOG=1 to enable)"
                )

        while self._running and not self._ffmpeg_stop.is_set():
            transport_seq = _rtsp_transport_sequence(url, self._transport)
            round_ok = False
            for transport in transport_seq:
                if self._ffmpeg_stop.is_set() or not self._running:
                    break
                demux_try = self._udp_demux_try_sequence(url)
                for demux_fmt in demux_try:
                    if self._ffmpeg_stop.is_set() or not self._running:
                        break
                    if round_ok:
                        break
                    demux = _ffmpeg_udp_raw_demux(url, demux_fmt)
                    tr_label = str(transport) if transport is not None else "n/a"
                    demux_label = demux_fmt or "auto"
                    print(
                        f"[VGCS:video] FFmpeg decode try rtsp_transport={tr_label} "
                        f"udp_demux={demux_label} url={url}"
                    )

                    if _url_scheme(url) == "rtsp":
                        # Brief cooldown when ffprobe ran (second RTSP session).
                        if _rtsp_should_ffprobe(url):
                            time.sleep(0.25)
                        else:
                            time.sleep(0.05)

                    cmd_base = [
                        "ffmpeg",
                        "-hide_banner",
                        "-nostats",
                        "-loglevel",
                        "error",
                        *demux,
                        *_ffmpeg_preflags_before_input(
                            url, rtsp_transport=transport, low_latency=self._low_latency
                        ),
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

                    reconnect_delay = 2.0 if _rtsp_url_is_siyi_style(url) else 0.6
                    empty_session_limit = 20
                    empty_sessions = 0

                    while self._running and not self._ffmpeg_stop.is_set():
                        stderr_buf: list[bytes] = []
                        p: subprocess.Popen[bytes] | None = None
                        try:
                            p = subprocess.Popen(
                                cmd_base,
                                stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE,
                                stdin=subprocess.DEVNULL,
                            )
                            self._ffmpeg_proc = p

                            def _drain_stderr(proc: subprocess.Popen[bytes], buf: list[bytes]) -> None:
                                try:
                                    ep = proc.stderr
                                    if ep is None:
                                        return
                                    for _ in range(8000):
                                        line = ep.readline()
                                        if not line:
                                            break
                                        if len(buf) < 200:
                                            buf.append(line)
                                except Exception:
                                    pass

                            threading.Thread(target=_drain_stderr, args=(p, stderr_buf), daemon=True).start()
                        except Exception as e:
                            self.error.emit(f"FFmpeg start (continuous) failed: {e}")
                            self._ffmpeg_proc = None
                            empty_sessions += 1
                            if empty_sessions >= empty_session_limit:
                                break
                            time.sleep(reconnect_delay)
                            continue

                        if p.stdout is None:
                            self._close_ffmpeg_decode_proc()
                            empty_sessions += 1
                            if empty_sessions >= empty_session_limit:
                                break
                            time.sleep(reconnect_delay)
                            continue

                        frames_this_session = 0
                        frame_count_box = [0]
                        decode_warned = False
                        frozen_logged = False
                        self._ffmpeg_last_raw_sig = None
                        self._ffmpeg_last_raw_change_mono = time.monotonic()
                        session_stop = threading.Event()
                        if _stall_watchdog_enabled(url):
                            self._start_ffmpeg_stall_watchdog(
                                p,
                                stall_s=stall_reconnect_s,
                                transport_label=tr_label,
                                session_stop=session_stop,
                                frames_counter=frame_count_box,
                            )
                        try:
                            while self._running and not self._ffmpeg_stop.is_set():
                                raw = self._read_exact(p.stdout, frame_bytes)
                                if raw is None:
                                    break
                                now_decode = time.monotonic()
                                self._ffmpeg_last_decode_mono = now_decode
                                try:
                                    import hashlib

                                    raw_sig = hashlib.blake2b(raw, digest_size=8).digest()
                                except Exception:
                                    raw_sig = raw[:64]
                                if raw_sig != self._ffmpeg_last_raw_sig:
                                    self._ffmpeg_last_raw_sig = raw_sig
                                    self._ffmpeg_last_raw_change_mono = now_decode
                                else:
                                    frozen_s = max(
                                        stall_reconnect_s * 2.0,
                                        15.0 if _rtsp_url_is_siyi_style(url) else 6.0,
                                    )
                                    if now_decode - self._ffmpeg_last_raw_change_mono >= frozen_s:
                                        if not frozen_logged:
                                            frozen_logged = True
                                            try:
                                                print(
                                                    f"[VGCS:video] frozen duplicate frames "
                                                    f"(>{frozen_s:.1f}s), reconnecting "
                                                    f"transport={tr_label}"
                                                )
                                            except Exception:
                                                pass
                                        try:
                                            if p.poll() is None:
                                                p.kill()
                                        except Exception:
                                            pass
                                        break
                                with self._rec_lock:
                                    recp = self._rec_proc
                                    if (
                                        recp is not None
                                        and recp.poll() is None
                                        and recp.stdin is not None
                                    ):
                                        try:
                                            recp.stdin.write(raw)
                                        except BrokenPipeError:
                                            pass
                                        except Exception:
                                            pass
                                now = time.monotonic()
                                if (now - last_emit_mono) < min_emit_dt:
                                    continue
                                try:
                                    assert np is not None
                                    arr = np.frombuffer(raw, dtype=np.uint8).reshape((h, w, 3))
                                    qimg = QImage(
                                        arr.data, w, h, 3 * w, QImage.Format.Format_RGB888
                                    ).copy()
                                    last_emit_mono = time.monotonic()
                                    meta = FrameMeta(
                                        source_id=self.source_id,
                                        device_name=self.device_name,
                                        timestamp_ms=0,
                                    )
                                    if not self._emit_video_frame_safe(qimg, meta):
                                        break
                                    self._ffmpeg_last_frame_mono = last_emit_mono
                                    frames_this_session += 1
                                    frame_count_box[0] = frames_this_session
                                    if frames_this_session == 1:
                                        self._ffmpeg_session_started_mono = time.monotonic()
                                        try:
                                            print(
                                                f"[VGCS:video] first frame ok "
                                                f"{w}x{h} transport={tr_label} url={url}"
                                            )
                                        except Exception:
                                            pass
                                    round_ok = True
                                except Exception as ex:
                                    if not decode_warned:
                                        decode_warned = True
                                        try:
                                            print(
                                                f"[VGCS:video] rawvideo frame decode error (first): {ex!r}"
                                            )
                                        except Exception:
                                            pass
                                    continue
                        finally:
                            session_stop.set()

                        proc = self._ffmpeg_proc
                        rc: int | None = None
                        try:
                            if proc is not None:
                                rc = proc.poll()
                        except Exception:
                            rc = None
                        if frames_this_session == 0:
                            try:
                                tail = b"".join(stderr_buf)[-4000:]
                                if tail.strip():
                                    txt = tail.decode("utf-8", errors="replace")
                                    print(
                                        "[VGCS:video] ffmpeg stderr (session, no frames):\n" + txt
                                    )
                                    if b"404" in tail and b"video2" in url.lower().encode():
                                        print(
                                            "[VGCS:video] hint: this ZR10 has no /video2 stream — "
                                            "use rtsp://192.168.144.25:8554/main.264 in Video settings"
                                        )
                                    if b"Codec AVOption ec" in tail or b"Invalid argument" in tail:
                                        print(
                                            "[VGCS:video] hint: update GCS to latest build "
                                            "(FFmpeg SIYI flags were mis-placed; fixed in pipeline.py)"
                                        )
                                elif rc not in (0, None):
                                    print(
                                        f"[VGCS:video] ffmpeg exited rc={rc!r} (no stderr lines captured)"
                                    )
                            except Exception:
                                pass
                        self._close_ffmpeg_decode_proc()

                        if self._ffmpeg_stop.is_set() or not self._running:
                            break

                        if frames_this_session > 0:
                            empty_sessions = 0
                            print(
                                f"[VGCS:video] decode session ended (frames={frames_this_session}), "
                                f"reconnecting same transport={tr_label} rc={rc!r}"
                            )
                            cooldown = reconnect_delay
                            try:
                                tail_b = b"".join(stderr_buf)
                                if _rtsp_url_is_siyi_style(url):
                                    if (
                                        b"-138" in tail_b
                                        or b"-10054" in tail_b
                                        or b"Error number -138" in tail_b
                                        or b"Error number -10054" in tail_b
                                        or b"hevc" in tail_b.lower()
                                    ):
                                        cooldown = max(cooldown, 2.5)
                            except Exception:
                                pass
                            time.sleep(cooldown)
                            continue

                        empty_sessions += 1
                        if empty_sessions >= empty_session_limit:
                            print(
                                f"[VGCS:video] decode: no frames after {empty_session_limit} attempts "
                                f"on transport={tr_label} demux={demux_label}, trying next"
                            )
                            break
                        time.sleep(reconnect_delay)

                    if round_ok:
                        break

                if self._ffmpeg_stop.is_set() or not self._running:
                    break
                if round_ok:
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
        self._low_latency: bool = False
        # Application Settings → Video → Source (`rtsp` | `udp_h264` | `udp_h265`).
        self._stream_kind: str = "rtsp"
        self._defer_refresh_timer = QTimer(self)
        self._defer_refresh_timer.setSingleShot(True)
        self._defer_refresh_timer.timeout.connect(self.refresh_sources)
        self._refresh_sources_active = False
        self._pending_teardown: list[tuple[str, object]] = []

        # Never run the first `refresh_sources()` synchronously from `__init__`: on Windows,
        # USB camera enumeration / teardown can block the GUI before the event loop is ready.
        QTimer.singleShot(0, self.refresh_sources)

    def refresh_sources(self) -> None:
        # Always stop + drop old sources. Replacing the dict alone leaves QObject children
        # and FFmpeg threads alive → duplicate RTSP sessions, CPU spikes, and "Python is not
        # responding" when saving Video settings while connected.
        #
        # Do NOT call QApplication.processEvents() from inside this method: a scheduled
        # refresh can re-enter here mid-loop and corrupt `_sources`.
        if getattr(self, "_refresh_sources_active", False):
            try:
                self._defer_refresh_timer.start(1)
            except Exception:
                pass
            return
        self._refresh_sources_active = True
        try:
            try:
                self._defer_refresh_timer.stop()
            except Exception:
                pass
            self._pending_teardown = list(self._sources.items())
            self._sources = {}
            if self._pending_teardown:
                QTimer.singleShot(0, self._refresh_sources_teardown_one)
            else:
                self._refresh_sources_build_and_finish()
        except Exception:
            self._refresh_sources_active = False

    @Slot()
    def _refresh_sources_teardown_one(self) -> None:
        """Tear down at most one old source per event-loop slice (FFmpeg stop can block)."""
        try:
            if not self._pending_teardown:
                self._refresh_sources_build_and_finish()
                return
            _sid, src = self._pending_teardown.pop(0)
            try:
                if hasattr(src, "blockSignals"):
                    src.blockSignals(True)
            except Exception:
                pass
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
            # `RtspSource.stop()` queues teardown on the GUI thread; an immediate
            # `deleteLater()` can destroy the object before those slots run.
            try:
                _o = src

                def _deferred_delete() -> None:
                    try:
                        _o.deleteLater()
                    except Exception:
                        pass

                QTimer.singleShot(100, _deferred_delete)
            except Exception:
                try:
                    src.deleteLater()
                except Exception:
                    pass
            if self._pending_teardown:
                QTimer.singleShot(50, self._refresh_sources_teardown_one)
            else:
                self._refresh_sources_build_and_finish()
        except Exception:
            try:
                self._refresh_sources_active = False
            except Exception:
                pass

    def _refresh_sources_build_and_finish(self) -> None:
        """Recreate sources from current URL settings; clears `_refresh_sources_active` when done."""
        try:
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
                        low_latency=self._low_latency,
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
                        low_latency=self._low_latency,
                        parent=self,
                    )
                    th.frame.connect(self._on_source_frame, Qt.ConnectionType.QueuedConnection)
                    self._sources["thermal"] = th
            # Enumerating every `QMediaDevices.videoInputs()` entry and constructing `QCamera`
            # backends here runs on the GUI thread. On Windows this routinely takes multiple
            # seconds (camera stack / virtual cameras), which freezes the whole app after
            # Application Settings → Apply while RTSP is also being torn down/rebuilt.
            # Companion / drone video uses RTSP or UDP — skip local cameras in that case.
            # Set `VGCS_VIDEO_ENUM_LOCAL_CAMERAS=1` to force legacy behavior (USB-only preview).
            _force_usb = str(os.environ.get("VGCS_VIDEO_ENUM_LOCAL_CAMERAS", "") or "").strip() == "1"
            _urls_present = bool(
                str(self._rtsp_day_url or "").strip() or str(self._rtsp_thermal_url or "").strip()
            )
            _net_kind = kind in ("rtsp", "udp_h264", "udp_h265")
            _skip_usb_enum = _net_kind and _urls_present and not _force_usb
            if HAS_MULTIMEDIA and QMediaDevices is not None and not _skip_usb_enum:
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

            def _emit_done() -> None:
                try:
                    self.sources_changed.emit()
                except Exception:
                    pass
                try:
                    self._refresh_sources_active = False
                except Exception:
                    pass

            QTimer.singleShot(0, _emit_done)
        except Exception:
            self._refresh_sources_active = False

    def schedule_refresh_sources(self) -> None:
        """Coalesce `refresh_sources()` on the next event-loop tick (never blocks the caller)."""
        try:
            self._defer_refresh_timer.stop()
        except Exception:
            pass
        self._defer_refresh_timer.start(0)

    def set_rtsp_sources(
        self,
        *,
        day_url: str = "",
        thermal_url: str = "",
        transport: str = "auto",
        stream_kind: str = "rtsp",
        low_latency: bool = False,
        defer_refresh: bool = False,
    ) -> None:
        _ = defer_refresh
        day_u = str(day_url or "").strip()
        th_u = str(thermal_url or "").strip()
        # Same URL on day + thermal (common mis-config) opens two FFmpeg sessions; many
        # companion cameras tolerate only one client. SIYI ZR10 allows four, but one decode
        # path is enough for single-view and reduces load / stall risk.
        if day_u and th_u and day_u == th_u:
            th_u = ""
        self._rtsp_day_url = day_u
        self._rtsp_thermal_url = th_u
        self._rtsp_transport = str(transport or "auto").strip().lower()
        self._low_latency = bool(low_latency)
        sk = str(stream_kind or "rtsp").strip().lower()
        self._stream_kind = sk if sk in ("rtsp", "udp_h264", "udp_h265") else "rtsp"
        # Reset active source if it no longer exists.
        if self._active_source_id and self._active_source_id not in self._sources:
            self._active_source_id = ""
        # Always schedule: synchronous `refresh_sources()` from map load / settings Apply
        # blocks the GUI thread on FFmpeg RTSP teardown (wrong Wi‑Fi, unreachable host).
        self.schedule_refresh_sources()

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
