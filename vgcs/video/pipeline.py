from __future__ import annotations

from dataclasses import dataclass
import threading
import time
import json
import shutil
import subprocess
from typing import Optional, Protocol

from PySide6.QtCore import QObject, Signal
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
    RTSP video stream source (Day/Thermal) using QtMultimedia.

    Note: RTSP support depends on the underlying Qt multimedia backend/codecs.
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
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self.source_id = str(source_id)
        self._url = str(url or "").strip()
        self._label = str(label or source_id)
        self._transport = str(transport or "auto").strip().lower()
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

    def start(self) -> None:
        if self._running:
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
            self.error.emit("ffmpeg not found in PATH (required for RTSP recording)")
            return False
        if self._rec_proc is not None and self._rec_proc.poll() is None:
            return True
        url = str(self._url or "").strip()
        if not url:
            self.error.emit("RTSP URL is empty")
            return False
        cmd = [
            "ffmpeg",
            "-y",
            "-loglevel",
            "error",
            "-rtsp_transport",
            "tcp",
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
            self._rec_proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
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
                self._rec_proc.terminate()
                try:
                    self._rec_proc.wait(timeout=2.0)
                except Exception:
                    try:
                        self._rec_proc.kill()
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
        # Wait briefly for QtMultimedia to deliver frames.
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.0:
            if self._last_frame_mono > 0:
                return
            time.sleep(0.05)
        # No frames: fall back.
        print(f"[VGCS:video] QtMultimedia produced no frames; switching to FFmpeg fallback url={self._url}")
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
            self.error.emit("RTSP URL is empty")
            return
        if shutil.which("ffmpeg") is None or shutil.which("ffprobe") is None:
            self.error.emit("ffmpeg/ffprobe not found in PATH (required for RTSP fallback)")
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
        try:
            if self._ffmpeg_proc is not None and self._ffmpeg_proc.poll() is None:
                self._ffmpeg_proc.kill()
        except Exception:
            pass
        self._ffmpeg_proc = None

    def _ffprobe_dims(self, url: str) -> tuple[int, int] | None:
        try:
            p = subprocess.run(
                [
                    "ffprobe",
                    "-v",
                    "error",
                    "-select_streams",
                    "v:0",
                    "-show_entries",
                    "stream=width,height",
                    "-of",
                    "json",
                    url,
                ],
                capture_output=True,
                text=True,
                timeout=3.5,
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
        # For preview we don't need full-resolution decoding. Smaller raw frames
        # dramatically speed up "first frame" availability and reduce pipe size.
        dims = self._ffprobe_dims(url) or (320, 180)
        w, h = dims
        # Cap decode size to keep the raw pipe responsive.
        w = max(160, min(int(w), 640))
        h = max(90, min(int(h), 360))
        self._ffmpeg_dims = (w, h)
        frame_bytes = int(w) * int(h) * 3
        transports = ("udp", "tcp") if self._transport not in ("udp", "tcp") else (self._transport,)
        # Try UDP first, then TCP.
        # Many RTSP servers (and QGC-style clients) work over RTP/UDP even
        # when TCP is blocked/unavailable.
        for transport in transports:
            if self._ffmpeg_stop.is_set():
                break
            self._ffmpeg_last_frame_mono = 0.0
            print(f"[VGCS:video] FFmpeg fallback start transport={transport} url={url}")

            # Probe stage: decode exactly ONE frame first so we can capture
            # the real failure reason (codec support, RTP/transport, etc.)
            # instead of only "no frames produced".
            probe_cmd = [
                "ffmpeg",
                "-hide_banner",
                "-nostats",
                "-loglevel",
                "info",
                "-rtsp_transport",
                transport,
                "-rw_timeout",
                "20000000",  # 20s in microseconds
                "-fflags",
                "+genpts",
                "-i",
                url,
                "-an",
                "-vf",
                f"scale={w}:{h}",
                "-pix_fmt",
                "rgb24",
                "-frames:v",
                "1",
                "-f",
                "rawvideo",
                "pipe:1",
            ]

            probe_proc: subprocess.Popen[bytes] | None = None
            try:
                probe_proc = subprocess.Popen(
                    probe_cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    stdin=subprocess.DEVNULL,
                    bufsize=0,
                )
                assert probe_proc.stdout is not None
                assert probe_proc.stderr is not None
                out, err = probe_proc.communicate(timeout=25.0)
            except subprocess.TimeoutExpired:
                if probe_proc is not None:
                    try:
                        probe_proc.kill()
                    except Exception:
                        pass
                    try:
                        out, err = probe_proc.communicate(timeout=2.0)
                    except Exception:
                        out, err = b"", b""
                else:
                    out, err = b"", b""
            except Exception as e:
                try:
                    if probe_proc is not None and probe_proc.stderr is not None:
                        err = probe_proc.stderr.read()[:2000]
                    else:
                        err = b""
                except Exception:
                    err = b""
                self.error.emit(f"FFmpeg probe failed ({transport}): {e} " + err.decode(errors="ignore"))
                continue

            out_len = len(out or b"")
            # ffmpeg prints a lot of config/banner output; prefer the tail
            # which usually contains the actual RTSP/decoder failure line.
            err_bytes = err or b""
            err_tail = err_bytes[-2200:] if len(err_bytes) > 2200 else err_bytes
            err_str = err_tail.decode(errors="ignore").strip()
            if out_len < frame_bytes:
                # If stderr is empty, still provide the length as a hint.
                hint = err_str if err_str else f"no frames decoded (stdout_bytes={out_len}, expected={frame_bytes})"
                self.error.emit(f"FFmpeg RTSP error ({transport}): {hint}")
                continue

            try:
                # Decode the probed single frame.
                arr = np.frombuffer(out[:frame_bytes], dtype=np.uint8).reshape((h, w, 3))  # type: ignore[union-attr]
                qimg = QImage(arr.data, w, h, 3 * w, QImage.Format.Format_RGB888).copy()
                meta = FrameMeta(source_id=self.source_id, device_name=self.device_name, timestamp_ms=0)
                self.frame.emit(VideoFrame(qimg, meta))
                self._ffmpeg_last_frame_mono = time.monotonic()
            except Exception as e:
                self.error.emit(f"FFmpeg probe decode failed ({transport}): {e}")
                continue

            # Continuous stage: now that we know the transport+codec works,
            # start streaming frames to the pipe.
            cmd = [
                "ffmpeg",
                "-hide_banner",
                "-nostats",
                "-loglevel",
                "warning",
                "-rtsp_transport",
                transport,
                "-rw_timeout",
                "20000000",  # 20s in microseconds
                "-fflags",
                "+genpts",
                "-i",
                url,
                "-an",
                "-vf",
                f"scale={w}:{h}",
                "-pix_fmt",
                "rgb24",
                "-f",
                "rawvideo",
                "pipe:1",
            ]
            try:
                self._ffmpeg_proc = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    stdin=subprocess.DEVNULL,
                    bufsize=0,
                )
            except Exception as e:
                self.error.emit(f"FFmpeg start (continuous) failed: {e}")
                self._ffmpeg_proc = None
                continue

            if self._ffmpeg_proc.stdout is None:
                self._stop_ffmpeg()
                continue

            while not self._ffmpeg_stop.is_set():
                raw = self._read_exact(self._ffmpeg_proc.stdout, frame_bytes)
                if raw is None:
                    break
                try:
                    assert np is not None
                    arr = np.frombuffer(raw, dtype=np.uint8).reshape((h, w, 3))
                    qimg = QImage(arr.data, w, h, 3 * w, QImage.Format.Format_RGB888).copy()
                    meta = FrameMeta(source_id=self.source_id, device_name=self.device_name, timestamp_ms=0)
                    self.frame.emit(VideoFrame(qimg, meta))
                    self._ffmpeg_last_frame_mono = time.monotonic()
                except Exception:
                    continue

            # If process ended without any frames, try next transport.
            if self._ffmpeg_last_frame_mono > 0.0:
                break

            # Surface any remaining stderr.
            try:
                if self._ffmpeg_proc is not None and self._ffmpeg_proc.stderr is not None:
                    err_rem = self._ffmpeg_proc.stderr.read()[:4000]
                    decoded = err_rem.decode(errors="ignore") if err_rem else ""
                    if decoded.strip():
                        self.error.emit(f"FFmpeg RTSP error ({transport}): {decoded}")
            except Exception:
                pass
            self._ffmpeg_proc = None

        self._stop_ffmpeg()


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

        self.refresh_sources()

    def refresh_sources(self) -> None:
        self._sources = {}
        # RTSP sources take precedence if configured.
        if HAS_MULTIMEDIA and (self._rtsp_day_url or self._rtsp_thermal_url):
            if self._rtsp_day_url:
                day = RtspSource(
                    url=self._rtsp_day_url,
                    source_id="day",
                    label="Day (RTSP)",
                    transport=self._rtsp_transport,
                    parent=self,
                )
                day.frame.connect(self._on_source_frame)
                self._sources["day"] = day
            if self._rtsp_thermal_url:
                th = RtspSource(
                    url=self._rtsp_thermal_url,
                    source_id="thermal",
                    label="Thermal (RTSP)",
                    transport=self._rtsp_transport,
                    parent=self,
                )
                th.frame.connect(self._on_source_frame)
                self._sources["thermal"] = th
        # Local cameras (optional, still useful for development).
        if HAS_MULTIMEDIA and QMediaDevices is not None:
            try:
                devices = list(QMediaDevices.videoInputs())
            except Exception:
                devices = []
            for i, dev in enumerate(devices):
                src_id = f"cam{i}"
                src = CameraSource(dev, source_id=src_id, parent=self)
                src.frame.connect(self._on_source_frame)
                self._sources[src_id] = src
        if not self._active_source_id and self._sources:
            self._active_source_id = next(iter(self._sources.keys()))
        self.sources_changed.emit()

    def set_rtsp_sources(self, *, day_url: str = "", thermal_url: str = "", transport: str = "auto") -> None:
        self._rtsp_day_url = str(day_url or "").strip()
        self._rtsp_thermal_url = str(thermal_url or "").strip()
        self._rtsp_transport = str(transport or "auto").strip().lower()
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

