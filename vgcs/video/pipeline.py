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
from typing import Callable, Optional, Protocol

# Common Herelink / Skydroid / IRCam companion Wi‑Fi video subnet (RTSP host lives here).
_COMPANION_RTSP_IPV4 = ipaddress.ip_network("192.168.144.0/24")

# Bump when SIYI / RTSP decode behaviour changes (printed once per RtspSource decode thread).
_VIDEO_PIPELINE_REV = "2026-06-23-companion-motion-preview"

# C13 / SIYI: one RTSP client slot — serialize opens across day/thermal sources.
_COMPANION_RTSP_OPEN_LOCK = threading.Lock()
_COMPANION_RTSP_LAST_OPEN_MONO: float = 0.0
# Same camera IP (e.g. 192.168.144.108) may only host one FFmpeg RTSP session.
_COMPANION_RTSP_HOST_OWNER: dict[str, str] = {}
# While gimbal is slewing, never freeze preview on "corrupt" vs last-good (scene changed).
_companion_preview_motion_until: float = 0.0

# MapWidget registers which source ids may open FFmpeg (single view = day only).
_companion_decode_gate: Callable[[str], bool] | None = None


def notify_companion_preview_motion(*, duration_s: float = 2.5) -> None:
    """Extend motion-preview window (gimbal slew — show live frames, do not hold stale preview)."""
    global _companion_preview_motion_until
    until = time.monotonic() + max(0.25, float(duration_s))
    if until > _companion_preview_motion_until:
        _companion_preview_motion_until = until


def _companion_preview_motion_active() -> bool:
    return time.monotonic() < float(_companion_preview_motion_until or 0.0)


def set_companion_decode_gate(fn: Callable[[str], bool] | None) -> None:
    """Optional gate: return False to block FFmpeg for a source id (companion RTSP)."""
    global _companion_decode_gate
    _companion_decode_gate = fn


def _companion_rtsp_host(url: str) -> str:
    try:
        return str(urlparse(str(url or "").strip()).hostname or "").strip()
    except Exception:
        return ""


def _companion_decode_permitted(source_id: str, url: str) -> bool:
    if not _rtsp_url_is_companion_rtsp(str(url or "").strip()):
        return True
    sid = str(source_id or "").strip()
    if _companion_decode_gate is None:
        return True
    try:
        ok = bool(_companion_decode_gate(sid))
    except Exception:
        return True
    if not ok:
        try:
            print(
                f"[VGCS:video] companion decode blocked for {sid!r} "
                f"(single-view — C13 allows one RTSP client; url={url})"
            )
        except Exception:
            pass
    return ok


def _companion_claim_rtsp_host(source_id: str, url: str) -> bool:
    """One FFmpeg RTSP session per companion camera host (C13 hardware limit)."""
    u = str(url or "").strip()
    if not _rtsp_url_is_companion_rtsp(u):
        return True
    if not _companion_decode_permitted(source_id, u):
        return False
    if str(os.environ.get("VGCS_COMPANION_DUAL_RTSP", "") or "").strip() == "1":
        return True
    host = _companion_rtsp_host(u)
    if not host:
        return True
    sid = str(source_id or "").strip()
    with _COMPANION_RTSP_OPEN_LOCK:
        owner = _COMPANION_RTSP_HOST_OWNER.get(host)
        if owner is None or owner == sid:
            _COMPANION_RTSP_HOST_OWNER[host] = sid
            return True
        try:
            print(
                f"[VGCS:video] companion RTSP blocked for {sid!r} on {host} "
                f"(owned by {owner!r}; C13 allows one RTSP client — set Default view "
                f"to Single or rail ▦ off; VGCS_COMPANION_DUAL_RTSP=1 to override)"
            )
        except Exception:
            pass
        return False


def release_companion_rtsp_host(source_id: str, url: str) -> None:
    """Drop host ownership so another source (day ↔ thermal) may open RTSP."""
    _companion_release_rtsp_host(source_id, url)


def release_all_companion_rtsp_hosts() -> None:
    """Clear all C13 host locks (day ↔ thermal handoff)."""
    with _COMPANION_RTSP_OPEN_LOCK:
        _COMPANION_RTSP_HOST_OWNER.clear()


def _companion_release_rtsp_host(source_id: str, url: str) -> None:
    u = str(url or "").strip()
    if not _rtsp_url_is_companion_rtsp(u):
        return
    host = _companion_rtsp_host(u)
    if not host:
        return
    sid = str(source_id or "").strip()
    with _COMPANION_RTSP_OPEN_LOCK:
        if _COMPANION_RTSP_HOST_OWNER.get(host) == sid:
            _COMPANION_RTSP_HOST_OWNER.pop(host, None)

# Set when D3D11VA/hwdownload fails once (Impossible to convert / hwdownload on Windows).
_SIYI_HWACCEL_UNAVAILABLE = False

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


def apply_digital_zoom_rgb24(raw: bytes, w: int, h: int, zoom: float) -> bytes:
    """
    Center-crop digital zoom on packed RGB24 (matches preview ``_apply_digital_zoom``).

    Used when saving thermal RTSP video: the thermal encoder feed stays wide while the
    operator preview applies software magnification.
    """
    z = float(zoom)
    if z <= 1.001 or not raw or w <= 0 or h <= 0:
        return raw
    if not HAS_NUMPY or np is None:
        return raw
    cw = max(1, int(w / z))
    ch = max(1, int(h / z))
    x = max(0, (w - cw) // 2)
    y = max(0, (h - ch) // 2)
    try:
        arr = np.frombuffer(raw, dtype=np.uint8).reshape((h, w, 3))
        crop = arr[y : y + ch, x : x + cw, :]
        row_idx = (np.arange(h) * ch / h).astype(np.int32)
        row_idx = np.clip(row_idx, 0, ch - 1)
        col_idx = (np.arange(w) * cw / w).astype(np.int32)
        col_idx = np.clip(col_idx, 0, cw - 1)
        scaled = crop[row_idx][:, col_idx]
        return scaled.tobytes()
    except Exception:
        return raw


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
    if m == "auto" and _rtsp_url_is_siyi_style(raw):
        # ZR10: UDP RTP first (fewer -138 storms after HEVC glitch); TCP as fallback.
        if str(os.environ.get("VGCS_SIYI_RTSP_TCP_ONLY", "0") or "0").strip() == "1":
            return ("tcp",)
        if str(os.environ.get("VGCS_SIYI_RTSP_TCP_FIRST", "0") or "0").strip() == "1":
            return ("tcp", "udp")
        return ("udp", "tcp")
    # Skydroid C13 (stream=1 / stream=2): UDP first — TCP often triggers HEVC POC/RPS tears.
    if m == "auto" and _rtsp_url_is_companion_link_subnet(raw):
        try:
            path = (urlparse(raw).path or "").lower()
        except Exception:
            path = ""
        if "stream=" in path or "/stream" in path:
            if str(os.environ.get("VGCS_C13_RTSP_TCP_FIRST", "0") or "0").strip() == "1":
                return ("tcp", "udp")
            return ("udp", "tcp")
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


def _rtsp_url_is_companion_rtsp(url: str) -> bool:
    """Companion-link RTSP (SIYI ZR10, Skydroid C13 stream=1, etc.)."""
    return _url_scheme(str(url or "").strip()) == "rtsp" and _rtsp_url_is_companion_link_subnet(url)


def _normalize_companion_rtsp_url(url: str) -> str:
    """Map known-bad SIYI paths (e.g. /video2 → 404 on many ZR10 units) to main.264."""
    import re

    u = str(url or "").strip()
    if not u or _url_scheme(u) != "rtsp":
        return u
    if "192.168.144." not in u.lower():
        return u
    if not re.search(r"/video2/?(?:\?|$)", u, flags=re.IGNORECASE):
        return u
    fixed = re.sub(r"/video2(?=/?(?:\?|$))", "/main.264", u, count=1, flags=re.IGNORECASE)
    if fixed != u:
        try:
            print(f"[VGCS:video] SIYI URL remap (video2 → 404 on this firmware): {u!r} -> {fixed!r}")
        except Exception:
            pass
    return fixed


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
    # SIYI ZR10: 8s connect/read cap — fail faster, try UDP sooner (15s × 8 retries blocked UI).
    if _rtsp_url_is_siyi_style(url):
        return 8_000_000
    if _rtsp_url_is_companion_link_subnet(url):
        return 8_000_000
    return 5_000_000


def _frozen_duplicate_kill_enabled(url: str) -> bool:
    """Companion HEVC often repeats identical buffers while healthy — do not kill the session."""
    if _rtsp_url_is_companion_rtsp(url):
        frozen = str(os.environ.get("VGCS_COMPANION_FROZEN_RECONNECT", "") or "").strip()
        if not frozen:
            frozen = str(os.environ.get("VGCS_SIYI_FROZEN_RECONNECT", "0") or "0").strip()
        return frozen == "1"
    return True


def _stall_watchdog_enabled(url: str) -> bool:
    """Companion HEVC often blocks inside FFmpeg for >6s while healthy; killing causes freeze loops."""
    if _rtsp_url_is_companion_rtsp(url):
        stall = str(os.environ.get("VGCS_COMPANION_STALL_WATCHDOG", "") or "").strip()
        if not stall:
            stall = str(os.environ.get("VGCS_SIYI_STALL_WATCHDOG", "0") or "0").strip()
        return stall == "1"
    return True


def _hevc_stderr_line_indicates_glitch(line: bytes) -> bool:
    """Single FFmpeg stderr line that signals HEVC reference / slice loss."""
    if not line:
        return False
    tl = line.lower()
    return (
        b"could not find ref" in tl
        or b"cu_qp_delta" in tl
        or b"error constructing the frame rps" in tl
        or b"constructing the frame rps" in tl
        or (b"[hevc @" in tl and b"poc" in tl)
    )


def _siyi_hevc_glitch_tail(tail_b: bytes) -> bool:
    """True when FFmpeg stderr shows HEVC reference/RPS loss (common on ZR10 during pan/zoom)."""
    if not tail_b:
        return False
    tl = tail_b.lower()
    return (
        b"could not find ref" in tl
        or b"cu_qp_delta" in tl
        or b"error constructing the frame rps" in tl
        or b"constructing the frame rps" in tl
        or b"poc " in tl
    )


def _companion_hevc_showall_enabled() -> bool:
    """Opt-in: FFmpeg +showall emits partial HEVC frames (macroblock garbage). Default off."""
    raw = str(os.environ.get("VGCS_COMPANION_SHOWALL", "0") or "0").strip().lower()
    return raw in ("1", "on", "true", "yes")


def _companion_decode_max_dims(url: str) -> tuple[int, int]:
    """Preview decode cap for companion RTSP (Skydroid C13, SIYI ZR10, etc.)."""
    cap_w = int(os.environ.get("VGCS_VIDEO_DECODE_MAX_W", "1920") or 1920)
    cap_h = int(os.environ.get("VGCS_VIDEO_DECODE_MAX_H", "1080") or 1080)
    if not _rtsp_url_is_companion_rtsp(url):
        return cap_w, cap_h
    try:
        w_raw = str(
            os.environ.get("VGCS_COMPANION_DECODE_MAX_W")
            or os.environ.get("VGCS_SIYI_DECODE_MAX_W", "960")
            or "960"
        ).strip()
        h_raw = str(
            os.environ.get("VGCS_COMPANION_DECODE_MAX_H")
            or os.environ.get("VGCS_SIYI_DECODE_MAX_H", "540")
            or "540"
        ).strip()
        cap_w = min(cap_w, int(w_raw))
        cap_h = min(cap_h, int(h_raw))
    except ValueError:
        cap_w = min(cap_w, 960)
        cap_h = min(cap_h, 540)
    return cap_w, cap_h


def _rtsp_transport_sequence_with_override(
    url: str, mode: str, override: str | None
) -> tuple[str | None, ...]:
    """Like ``_rtsp_transport_sequence`` but prefers a sticky transport after HEVC glitch."""
    seq = list(_rtsp_transport_sequence(url, mode))
    ov = str(override or "").strip().lower()
    if ov in ("tcp", "udp") and ov in seq:
        seq = [ov] + [t for t in seq if t != ov]
    return tuple(seq)


def _rgb_frame_looks_hevc_corrupt(
    arr: "np.ndarray",
    prev: "np.ndarray | None",
    *,
    block: int = 16,
    stride: int = 16,
) -> bool:
    """
    Detect macroblock-soup HEVC corruption (patchy tile chaos vs last good frame).

    Gimbal pan/tilt produces a coherent global scene shift — that is motion, not corruption.
    """
    if not HAS_NUMPY or np is None or prev is None or prev.shape != arr.shape:
        return False
    h, w, _ = arr.shape
    if h < block or w < block:
        return False
    bad = 0
    tiles = 0
    tile_diffs: list[float] = []
    cur = arr.astype(np.int16)
    prv = prev.astype(np.int16)
    for y in range(0, h - block + 1, stride):
        for x in range(0, w - block + 1, stride):
            tiles += 1
            prv_tile = prv[y : y + block, x : x + block]
            tile = cur[y : y + block, x : x + block]
            diff_mean = float(np.abs(tile - prv_tile).mean())
            tile_diffs.append(diff_mean)
            if diff_mean > 38.0 and float(tile.std()) > 32.0:
                bad += 1
    if tiles < 8:
        return False
    mean_d = float(np.mean(tile_diffs))
    std_d = float(np.std(tile_diffs))
    high_diff = sum(1 for d in tile_diffs if d > 28.0)
    high_diff_ratio = high_diff / tiles
    unchanged_ratio = sum(1 for d in tile_diffs if d < 8.0) / tiles
    # Coherent gimbal motion: nearly every tile shifts by a similar amount.
    if (
        unchanged_ratio < 0.12
        and mean_d > 20.0
        and std_d < 14.0
    ):
        return False
    if mean_d > 18.0 and std_d < 15.0 and high_diff_ratio > 0.30:
        return False
    if high_diff_ratio > 0.38 and std_d < 18.0:
        return False
    # Patchy macroblock soup: mixed unchanged tiles + chaotic tiles (high diff spread).
    if std_d > 20.0 and mean_d > 8.0 and (bad / tiles) > 0.10:
        return True
    return (bad / tiles) > 0.22


def _rgb_frame_has_decode_artifacts(
    arr: "np.ndarray",
    *,
    strict: bool = True,
) -> bool:
    """
    Detect classic HEVC decode tears (magenta/green macroblock bands).

    These must never be shown — hold the last good preview instead.
    """
    if not HAS_NUMPY or np is None:
        return False
    h, w, _ = arr.shape
    if h < 8 or w < 8:
        return False
    r = arr[:, :, 0].astype(np.int16)
    g = arr[:, :, 1].astype(np.int16)
    b = arr[:, :, 2].astype(np.int16)
    pixels = float(h * w)
    # YUV plane misalignment: neon magenta (high R+B, low G).
    magenta = (r > 175) & (g < 85) & (b > 175)
    magenta_ratio = float(magenta.sum()) / pixels
    magenta_thr = 0.06 if strict else 0.14
    if magenta_ratio > magenta_thr:
        return True
    # Green macroblock smear.
    green_tear = (g > 195) & (r < 70) & (b < 70)
    green_thr = 0.06 if strict else 0.14
    if float(green_tear.sum()) / pixels > green_thr:
        return True
    # Thick horizontal tear bands (common in the user's field captures).
    row_magenta = magenta.mean(axis=1)
    row_thr = 0.30 if strict else 0.45
    if int((row_magenta > row_thr).sum()) >= (2 if strict else 3):
        return True
    # Horizontal noise bands / macroblock smear (white-gray grid in lower frame).
    gray = (0.299 * r + 0.587 * g + 0.114 * b).astype(np.float32)
    row_std = gray.std(axis=1)
    third = max(4, h // 3)
    top_std = float(np.median(row_std[:third]))
    bot_std = float(np.median(row_std[h - third :]))
    if bot_std > max(14.0, top_std * 2.5) and bot_std > top_std + 8.0:
        return True
    return False


def _companion_frame_should_hide(
    arr: "np.ndarray",
    last_good: "np.ndarray | None",
    *,
    motion_preview: bool = False,
) -> tuple[bool, str]:
    """Return (hide, reason) for companion preview quality gate."""
    strict_art = not motion_preview
    if _rgb_frame_has_decode_artifacts(arr, strict=strict_art):
        return True, "artifact"
    if motion_preview:
        return False, ""
    if last_good is not None and _rgb_frame_looks_hevc_corrupt(arr, last_good):
        return True, "corrupt"
    return False, ""


def _siyi_rtsp_camera_busy(tail_b: bytes) -> bool:
    """True when stderr indicates the camera refused or timed out RTSP (-138 / winsock)."""
    if not tail_b:
        return False
    tl = tail_b.lower()
    return (
        b"-138" in tl
        or b"error number -138" in tl
        or b"-10054" in tl
        or b"operation not permitted" in tl
    )


def _siyi_hevc_glitch_release_sleep(*, brief_session: bool = False) -> None:
    """Pause before reopening RTSP after HEVC POC/RPS loss (avoids -138 on immediate reconnect)."""
    raw = str(os.environ.get("VGCS_SIYI_HEVC_RELEASE_S", "3.5") or "3.5").strip()
    try:
        delay = max(1.0, min(8.0, float(raw)))
    except ValueError:
        delay = 3.5
    if brief_session:
        try:
            extra = float(
                str(os.environ.get("VGCS_SIYI_HEVC_BRIEF_RELEASE_S", "5.0") or "5.0").strip()
            )
        except ValueError:
            extra = 5.0
        delay = min(12.0, delay + max(0.0, extra))
    try:
        msg = (
            f"[VGCS:video] companion HEVC: waiting {delay:.1f}s for camera after glitch "
            "(do not open RC/VLC video)"
        )
        if brief_session:
            msg += " (brief session — longer release)"
        print(msg)
    except Exception:
        pass
    time.sleep(delay)


def _siyi_camera_release_sleep(
    tail_b: bytes, empty_sessions: int, *, fail_streak: int = 0
) -> None:
    """ZR10 often needs a few seconds to release the single RTSP client slot."""
    if not _siyi_rtsp_camera_busy(tail_b):
        return
    base = 3.0
    try:
        base = float(str(os.environ.get("VGCS_SIYI_RTSP_RELEASE_S", "3.0") or "3.0").strip())
    except ValueError:
        base = 3.0
    delay = min(
        20.0,
        base + 0.5 * max(0, int(empty_sessions) - 1) + 1.5 * max(0, int(fail_streak)),
    )
    try:
        print(
            f"[VGCS:video] companion RTSP: waiting {delay:.1f}s for camera RTSP slot "
            "(close VLC/handheld viewer first)"
        )
    except Exception:
        pass
    time.sleep(delay)


def _siyi_session_cooldown_s(tail_b: bytes, reconnect_delay: float) -> float:
    """
  Cooldown before reopening RTSP after a decode session ends.

  HEVC POC/RPS glitches need a *short* reconnect (new IDR). Network errors need longer.
    """
    if _siyi_hevc_glitch_tail(tail_b):
        raw = str(os.environ.get("VGCS_SIYI_HEVC_RECONNECT_S", "2.0") or "2.0").strip()
        try:
            return max(0.8, min(5.0, float(raw)))
        except ValueError:
            return 2.0
    tl = tail_b.lower() if tail_b else b""
    if (
        b"-138" in tl
        or b"-10054" in tl
        or b"error number -138" in tl
        or b"error number -10054" in tl
    ):
        return max(8.0, float(reconnect_delay))
    return min(3.0, max(1.0, float(reconnect_delay)))


def _video_stall_reconnect_s(url: str) -> float:
    """Max seconds without a full raw frame (frozen-duplicate path; watchdog uses same value)."""
    raw = str(os.environ.get("VGCS_VIDEO_STALL_RECONNECT_S", "") or "").strip()
    if raw:
        try:
            return max(0.8, min(30.0, float(raw)))
        except ValueError:
            pass
    if _rtsp_url_is_companion_rtsp(url):
        return 12.0
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


def _siyi_hwaccel_enabled(url: str) -> bool:
    """Off by default — software decode matches VLC on SIYI ZR10; set VGCS_SIYI_HWACCEL=1 to try GPU."""
    global _SIYI_HWACCEL_UNAVAILABLE
    if not _rtsp_url_is_siyi_style(url):
        return False
    if _SIYI_HWACCEL_UNAVAILABLE:
        return False
    raw = str(os.environ.get("VGCS_SIYI_HWACCEL", "0") or "0").strip().lower()
    return raw in ("1", "on", "true", "yes")


def _siyi_mark_hwaccel_unavailable() -> None:
    global _SIYI_HWACCEL_UNAVAILABLE
    _SIYI_HWACCEL_UNAVAILABLE = True


def _ffmpeg_preflags_before_input(
    url: str,
    *,
    rtsp_transport: str | None,
    low_latency: bool = False,
    siyi_hwaccel: bool = False,
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
            if _rtsp_url_is_companion_rtsp(u):
                # Companion HEVC (C13, SIYI): gentler demux; app-level reconnect only
                # (FFmpeg demuxer -reconnect joins mid-GOP → POC/RPS macroblock tears).
                out.extend(
                    [
                        "-fflags",
                        "+genpts+discardcorrupt+igndts",
                        "-analyzeduration",
                        "12000000",
                        "-probesize",
                        "12000000",
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
        if _rtsp_url_is_companion_rtsp(u):
            out.extend(["-thread_queue_size", "512"])
        if siyi_hwaccel and _rtsp_url_is_siyi_style(u):
            out.extend(["-hwaccel", "auto", "-extra_hw_frames", "8"])
    out.extend(["-err_detect", "ignore_err"])
    if (
        sc == "rtsp"
        and _rtsp_url_is_companion_rtsp(str(url or "").strip())
        and _companion_hevc_showall_enabled()
    ):
        # Opt-in only: +showall surfaces partial HEVC frames (macroblock garbage in preview).
        out.extend(["-flags2", "+showall"])
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


def _ffmpeg_vf_rgb_fixed_size(w: int, h: int, *, hwaccel: bool = False) -> str:
    """
    Scale to fit inside WxH, then pad to exactly WxH.

    `force_original_aspect_ratio=decrease` alone can emit smaller frames when the
    stream aspect does not match the target box; our rawvideo reader assumes
    every packet is exactly w*h*3 bytes. Padding avoids silent decode drops
    (symptom: first frame OK, then frozen preview).
    """
    w = max(2, int(w))
    h = max(2, int(h))
    core = (
        f"scale={w}:{h}:force_original_aspect_ratio=decrease:flags=fast_bilinear,"
        f"pad={w}:{h}:(ow-iw)/2:(oh-ih)/2:color=black"
    )
    if hwaccel:
        return f"hwdownload,format=nv12,{core}"
    return core


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
        self._ffmpeg_had_frame: bool = False
        self._restart_decode_last_mono: float = 0.0
        self._restart_decode_timer: QTimer | None = None
        self._siyi_last_rtsp_open_mono: float = 0.0
        self._siyi_138_fail_streak: int = 0
        self._siyi_last_open_tail: bytes = b""
        self._companion_transport_override: str | None = None
        self._rec_proc: subprocess.Popen[bytes] | None = None
        self._rec_lock = threading.Lock()
        self._rec_digital_zoom: float = 1.0
        self._rec_apply_digital_zoom: bool = False
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
        if shutil.which("ffmpeg") is None:
            return False
        u = (self._url or "").strip().lower()
        if u.startswith("udp://") or u.startswith("tcp://"):
            return True
        return u.startswith("rtsp://")

    def decode_recently_active(self, max_age_s: float = 10.0) -> bool:
        """True when FFmpeg is running and produced frames recently (or still connecting)."""
        if not self._running:
            return False
        thr = max(1.0, float(max_age_s))
        now = time.monotonic()
        last = float(self._ffmpeg_last_decode_mono or self._ffmpeg_last_frame_mono or 0.0)
        if last > 0.0 and (now - last) < thr:
            return True
        th = self._ffmpeg_thread
        if th is not None and th.is_alive() and not bool(self._ffmpeg_had_frame):
            sess = float(self._ffmpeg_session_started_mono or 0.0)
            if sess > 0.0 and (now - sess) < min(30.0, thr + 15.0):
                return True
        return False

    def restart_decode(self, *, delay_ms: int = 0) -> None:
        """Stop FFmpeg and open a fresh RTSP session (companion link-up / settings Apply)."""
        now = time.monotonic()
        if now - float(self._restart_decode_last_mono or 0.0) < 4.0:
            return
        self._restart_decode_last_mono = now
        u = str(self._url or "").strip()
        if delay_ms <= 0:
            delay_ms = 2800 if _rtsp_url_is_companion_link_subnet(u) else 450
        try:
            print(f"[VGCS:video] restart_decode scheduled ({delay_ms}ms) url={u}")
        except Exception:
            pass
        try:
            t = self._restart_decode_timer
            if t is not None:
                t.stop()
        except Exception:
            pass
        ms = max(200, int(delay_ms))
        self._restart_decode_timer = QTimer(self)
        self._restart_decode_timer.setSingleShot(True)
        if _rtsp_url_is_companion_link_subnet(u):
            # Soft restart: keep preview "running", join old FFmpeg before a new RTSP open
            # (ZR10 allows one client — overlapping connect → Error -138).
            self._restart_decode_timer.timeout.connect(self._deferred_companion_ffmpeg_restart)
        else:
            self.stop()
            self._restart_decode_timer.timeout.connect(self._deferred_start_after_restart)
        self._restart_decode_timer.start(ms)

    @Slot()
    def _deferred_companion_ffmpeg_restart(self) -> None:
        """Companion/SIYI: restart FFmpeg without stop()/start() tearing down preview state."""
        try:
            self._ffmpeg_stop.set()
        except Exception:
            pass
        try:
            self._close_ffmpeg_decode_proc()
        except Exception:
            pass
        th = self._ffmpeg_thread
        if th is not None and th.is_alive():
            try:
                th.join(timeout=12.0)
            except Exception:
                pass
        self._ffmpeg_thread = None
        try:
            self._ffmpeg_stop.clear()
        except Exception:
            pass
        if not self._running:
            self._running = True
        if self._prefer_ffmpeg_immediately():
            self._start_ffmpeg()
        else:
            self.start()

    @Slot()
    def _deferred_start_after_restart(self) -> None:
        if self._ffmpeg_stop.is_set():
            try:
                self._ffmpeg_stop.clear()
            except Exception:
                pass
        self.start()

    def start(self) -> None:
        try:
            t = self._restart_decode_timer
            if t is not None and t.isActive():
                t.stop()
        except Exception:
            pass
        if self._running:
            # Recover after a partial stop (decode thread exited but preview still "on").
            if self._prefer_ffmpeg_immediately():
                if not _companion_claim_rtsp_host(self.source_id, str(self._url or "")):
                    return
                th = self._ffmpeg_thread
                if th is None or not th.is_alive():
                    self._start_ffmpeg()
            return
        if self._prefer_ffmpeg_immediately():
            if not _companion_claim_rtsp_host(self.source_id, str(self._url or "")):
                return
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

    def companion_hard_stop_decode(self, *, join_s: float = 3.0) -> None:
        """Stop FFmpeg and release the C13 RTSP slot (day ↔ thermal IR switch)."""
        url = str(self._url or "").strip()
        self._running = False
        try:
            self._ffmpeg_stop.set()
        except Exception:
            pass
        try:
            self._close_ffmpeg_decode_proc()
        except Exception:
            pass
        th = self._ffmpeg_thread
        if th is not None and th.is_alive():
            try:
                th.join(timeout=max(0.2, float(join_s)))
            except Exception:
                pass
        self._ffmpeg_thread = None
        try:
            self._ffmpeg_stop.clear()
        except Exception:
            pass
        try:
            if self._player is not None:
                self._player.stop()
        except Exception:
            pass
        if url:
            release_companion_rtsp_host(self.source_id, url)

    def set_recording_preview_transform(
        self,
        *,
        digital_zoom: float = 1.0,
        apply_digital_zoom: bool = False,
    ) -> None:
        """Match saved RTSP video to on-screen preview transforms (thermal software zoom)."""
        with self._rec_lock:
            try:
                self._rec_digital_zoom = max(1.0, float(digital_zoom))
            except Exception:
                self._rec_digital_zoom = 1.0
            self._rec_apply_digital_zoom = bool(apply_digital_zoom)

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
        if not _companion_claim_rtsp_host(self.source_id, url):
            return
        if shutil.which("ffmpeg") is None:
            self.error.emit("ffmpeg not found in PATH (required for network video decode)")
            return
        if shutil.which("ffprobe") is None:
            try:
                print(
                    "[VGCS:video] ffprobe not in PATH — using default 1280x720 decode canvas"
                )
            except Exception:
                pass
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

    def _siyi_throttle_before_rtsp_open(self, url: str, tail_hint: bytes = b"") -> None:
        """Minimum gap between FFmpeg RTSP opens (reduces companion -138 / busy slot storms)."""
        if not _rtsp_url_is_companion_rtsp(url):
            return
        global _COMPANION_RTSP_LAST_OPEN_MONO
        streak = int(getattr(self, "_siyi_138_fail_streak", 0) or 0)
        min_gap = 5.0
        if streak >= 2 or _siyi_rtsp_camera_busy(tail_hint):
            min_gap = min(25.0, 8.0 + streak * 2.0)
        with _COMPANION_RTSP_OPEN_LOCK:
            now = time.monotonic()
            gap = now - float(_COMPANION_RTSP_LAST_OPEN_MONO or 0.0)
            if gap < min_gap:
                wait = min_gap - gap
                try:
                    print(
                        f"[VGCS:video] companion RTSP: throttling open ({wait:.1f}s, "
                        f"url={url}, avoid camera busy / -138)"
                    )
                except Exception:
                    pass
                time.sleep(wait)
            _COMPANION_RTSP_LAST_OPEN_MONO = time.monotonic()

    def _siyi_mark_rtsp_open_result(self, frames: int, tail: bytes) -> None:
        if not tail:
            return
        if frames > 0:
            self._siyi_138_fail_streak = 0
            self._siyi_last_open_tail = b""
            return
        if _siyi_rtsp_camera_busy(tail):
            self._siyi_138_fail_streak = int(getattr(self, "_siyi_138_fail_streak", 0) or 0) + 1
            self._siyi_last_open_tail = tail[-4000:]
            streak = self._siyi_138_fail_streak
            if streak >= 5 and streak % 5 == 0:
                try:
                    self.error.emit(
                        "Companion RTSP busy (-138). Close RC/VLC/handheld video, wait 10s, retry."
                    )
                except Exception:
                    pass

    def _ffmpeg_loop(self) -> None:
        try:
            print(f"[VGCS:video] pipeline rev={_VIDEO_PIPELINE_REV}")
        except Exception:
            pass
        url = _normalize_companion_rtsp_url(str(self._url or "").strip())
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
        comp_cap_w, comp_cap_h = _companion_decode_max_dims(url)
        if _rtsp_url_is_companion_rtsp(url):
            cap_w = min(cap_w, comp_cap_w)
            cap_h = min(cap_h, comp_cap_h)
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
        siyi_hw = _siyi_hwaccel_enabled(url)

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
        companion_rtsp = _rtsp_url_is_companion_rtsp(url)
        siyi_url = _rtsp_url_is_siyi_style(url)
        round_backoff_s = 2.0 if companion_rtsp else 0.6
        max_round_backoff_s = 15.0 if companion_rtsp else 4.0

        if companion_rtsp:
            print(
                f"[VGCS:video] RTSP companion link (192.168.144.x): transport order="
                f"{_rtsp_transport_sequence_with_override(url, self._transport, self._companion_transport_override)!r}"
            )
            if not _companion_hevc_showall_enabled():
                print(
                    "[VGCS:video] companion HEVC: artifact frames hidden "
                    "(hold last good preview; never show macroblock tears)"
                )
            if _rtsp_url_is_companion_rtsp(url):
                print(
                    f"[VGCS:video] companion decode cap {w}x{h} "
                    f"(set VGCS_COMPANION_DECODE_MAX_W/H to override)"
                )
            if not _stall_watchdog_enabled(url):
                print(
                    "[VGCS:video] companion HEVC: stall watchdog disabled "
                    "(decode gaps are normal; set VGCS_COMPANION_STALL_WATCHDOG=1 to enable)"
                )
            if not _frozen_duplicate_kill_enabled(url):
                print(
                    "[VGCS:video] companion HEVC: frozen-duplicate reconnect disabled "
                    "(set VGCS_COMPANION_FROZEN_RECONNECT=1 to enable)"
                )
            if siyi_url:
                mode = "hwaccel=auto" if siyi_hw else "sw (auto codec)"
                print(f"[VGCS:video] SIYI HEVC decode path: {mode}")

        while self._running and not self._ffmpeg_stop.is_set():
            transport_seq = _rtsp_transport_sequence_with_override(
                url, self._transport, self._companion_transport_override
            )
            round_ok = False
            url_fatal = False
            for transport in transport_seq:
                if self._ffmpeg_stop.is_set() or not self._running:
                    break
                transport_got_frames = False
                demux_try = self._udp_demux_try_sequence(url)
                for demux_fmt in demux_try:
                    if self._ffmpeg_stop.is_set() or not self._running:
                        break
                    # Only skip extra demux variants on this transport — never skip UDP
                    # because TCP worked earlier in the same round (fixes -138 stuck on TCP).
                    if transport_got_frames:
                        break
                    demux = _ffmpeg_udp_raw_demux(url, demux_fmt)
                    tr_label = str(transport) if transport is not None else "n/a"
                    eff_transport = self._companion_transport_override or transport
                    eff_label = str(eff_transport) if eff_transport is not None else tr_label
                    demux_label = demux_fmt or "auto"
                    print(
                        f"[VGCS:video] FFmpeg decode try rtsp_transport={eff_label} "
                        f"udp_demux={demux_label} url={url}"
                    )

                    if _url_scheme(url) == "rtsp":
                        # Brief cooldown when ffprobe ran (second RTSP session).
                        if _rtsp_should_ffprobe(url):
                            time.sleep(0.25)
                        else:
                            time.sleep(0.05)

                    reconnect_delay = 2.5 if companion_rtsp else 0.6
                    empty_session_limit = 4 if companion_rtsp else 20
                    empty_sessions = 0
                    tcp_138_streak = 0

                    while self._running and not self._ffmpeg_stop.is_set() and not url_fatal:
                        if companion_rtsp:
                            if not _companion_decode_permitted(self.source_id, url):
                                break
                            if not _companion_claim_rtsp_host(self.source_id, url):
                                break
                        use_transport = self._companion_transport_override or transport
                        tr_label = str(use_transport) if use_transport is not None else "n/a"
                        if companion_rtsp:
                            self._siyi_throttle_before_rtsp_open(
                                url, getattr(self, "_siyi_last_open_tail", b"") or b""
                            )
                        if siyi_url and (
                            transport_got_frames or self._companion_transport_override
                        ):
                            try:
                                print(
                                    "[VGCS:video] SIYI: opening new RTSP decode session "
                                    f"(transport={tr_label})"
                                )
                            except Exception:
                                pass
                        vf_rgb = _ffmpeg_vf_rgb_fixed_size(w, h, hwaccel=siyi_hw)
                        cmd_base = [
                            "ffmpeg",
                            "-hide_banner",
                            "-nostats",
                            "-loglevel",
                            "error",
                            *demux,
                            *_ffmpeg_preflags_before_input(
                                url,
                                rtsp_transport=use_transport,
                                low_latency=self._low_latency,
                                siyi_hwaccel=siyi_hw,
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
                        stderr_buf: list[bytes] = []
                        hevc_glitch_hold = {"until_mono": 0.0}
                        p: subprocess.Popen[bytes] | None = None
                        try:
                            p = subprocess.Popen(
                                cmd_base,
                                stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE,
                                stdin=subprocess.DEVNULL,
                            )
                            self._ffmpeg_proc = p

                            def _drain_stderr(
                                proc: subprocess.Popen[bytes],
                                buf: list[bytes],
                                hold: dict[str, float],
                            ) -> None:
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
                                        if companion_rtsp and _hevc_stderr_line_indicates_glitch(
                                            line
                                        ):
                                            hold["until_mono"] = time.monotonic() + 0.35
                                except Exception:
                                    pass

                            threading.Thread(
                                target=_drain_stderr,
                                args=(p, stderr_buf, hevc_glitch_hold),
                                daemon=True,
                            ).start()
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
                        waiting_logged = False
                        session_t0 = time.monotonic()
                        frozen_logged = False
                        corrupt_skip_logged = False
                        corrupt_skip_streak = 0
                        last_good_arr: "np.ndarray | None" = None
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
                        read_fail_streak = 0
                        try:
                            while self._running and not self._ffmpeg_stop.is_set():
                                if (
                                    frames_this_session == 0
                                    and not waiting_logged
                                    and time.monotonic() - session_t0 > 12.0
                                ):
                                    waiting_logged = True
                                    try:
                                        print(
                                            "[VGCS:video] still waiting for first RTSP frame "
                                            f"(>{12:.0f}s, transport={tr_label})"
                                        )
                                    except Exception:
                                        pass
                                raw = self._read_exact(p.stdout, frame_bytes)
                                if raw is None:
                                    if p.poll() is None and read_fail_streak < 40:
                                        read_fail_streak += 1
                                        time.sleep(0.05)
                                        continue
                                    break
                                read_fail_streak = 0
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
                                elif _frozen_duplicate_kill_enabled(url):
                                    frozen_s = max(stall_reconnect_s * 2.0, 6.0)
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
                                    rec_raw = raw
                                    if self._rec_apply_digital_zoom:
                                        z = float(self._rec_digital_zoom)
                                        if z > 1.001:
                                            rec_raw = apply_digital_zoom_rgb24(raw, w, h, z)
                                    if (
                                        recp is not None
                                        and recp.poll() is None
                                        and recp.stdin is not None
                                    ):
                                        try:
                                            recp.stdin.write(rec_raw)
                                        except BrokenPipeError:
                                            pass
                                        except Exception:
                                            pass
                                now = time.monotonic()
                                if (now - last_emit_mono) < min_emit_dt:
                                    continue
                                motion_preview = (
                                    companion_rtsp and _companion_preview_motion_active()
                                )
                                if (
                                    companion_rtsp
                                    and not motion_preview
                                    and now
                                    < float(hevc_glitch_hold.get("until_mono", 0.0) or 0.0)
                                ):
                                    if not corrupt_skip_logged:
                                        corrupt_skip_logged = True
                                        try:
                                            print(
                                                "[VGCS:video] companion HEVC: skipping frames "
                                                "during decoder glitch (hold last good preview)"
                                            )
                                        except Exception:
                                            pass
                                    continue
                                try:
                                    assert np is not None
                                    arr = np.frombuffer(raw, dtype=np.uint8).reshape((h, w, 3))
                                    if companion_rtsp and not _companion_hevc_showall_enabled():
                                        hide, why = _companion_frame_should_hide(
                                            arr,
                                            last_good_arr,
                                            motion_preview=motion_preview,
                                        )
                                        if hide:
                                            corrupt_skip_streak += 1
                                            if corrupt_skip_streak in (1, 12, 24, 36):
                                                try:
                                                    print(
                                                        "[VGCS:video] companion HEVC: "
                                                        f"hiding {why} frame "
                                                        f"(streak={corrupt_skip_streak}, "
                                                        "hold last good preview)"
                                                    )
                                                except Exception:
                                                    pass
                                            streak_limit = 999 if motion_preview else 45
                                            if corrupt_skip_streak >= streak_limit:
                                                try:
                                                    print(
                                                        "[VGCS:video] companion HEVC: "
                                                        "corrupt streak — reconnecting RTSP"
                                                    )
                                                except Exception:
                                                    pass
                                                try:
                                                    if p.poll() is None:
                                                        p.kill()
                                                except Exception:
                                                    pass
                                                break
                                            continue
                                        corrupt_skip_streak = 0
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
                                    if companion_rtsp:
                                        last_good_arr = arr.copy()
                                    self._ffmpeg_last_frame_mono = last_emit_mono
                                    frames_this_session += 1
                                    frame_count_box[0] = frames_this_session
                                    if frames_this_session == 1:
                                        self._ffmpeg_session_started_mono = time.monotonic()
                                        tag = (
                                            "reconnected"
                                            if self._ffmpeg_had_frame
                                            else "first"
                                        )
                                        self._ffmpeg_had_frame = True
                                        try:
                                            print(
                                                f"[VGCS:video] {tag} frame ok "
                                                f"{w}x{h} transport={tr_label} url={url}"
                                            )
                                        except Exception:
                                            pass
                                    transport_got_frames = True
                                    round_ok = True
                                    tcp_138_streak = 0
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
                            tail = b""
                            try:
                                tail = b"".join(stderr_buf)[-4000:]
                                if tail.strip():
                                    txt = tail.decode("utf-8", errors="replace")
                                    print(
                                        "[VGCS:video] ffmpeg stderr (session, no frames):\n" + txt
                                    )
                                    if companion_rtsp:
                                        tl = tail.lower()
                                        if b"codec avoption ec" in tl:
                                            reconnect_delay = max(reconnect_delay, 3.0)
                                        elif siyi_hw and (
                                            b"hwaccel" in tl
                                            or b"hwdownload" in tl
                                            or b"impossible to convert" in tl
                                            or b"invalid data" in tl
                                            or b"d3d11" in tl
                                            or b"dxva" in tl
                                            or b"no device" in tl
                                        ):
                                            siyi_hw = False
                                            _siyi_mark_hwaccel_unavailable()
                                            print(
                                                "[VGCS:video] SIYI: hwaccel failed, "
                                                "using software decode (VLC-style)"
                                            )
                                        elif (
                                            b"could not find ref" in tl
                                            or b"-10054" in tl
                                            or b"-138" in tl
                                            or b"error number -138" in tl
                                        ):
                                            siyi_hw = False
                                            if not _siyi_rtsp_camera_busy(tail):
                                                reconnect_delay = max(
                                                    reconnect_delay,
                                                    10.0
                                                    + min(8.0, float(empty_sessions) * 2.0),
                                                )
                                            if empty_sessions >= 3 and empty_sessions % 3 == 0:
                                                try:
                                                    print(
                                                        "[VGCS:video] companion RTSP connect timeout (-138): "
                                                        "close VLC/other viewers on this camera, confirm "
                                                        "192.168.144.x is reachable; will try UDP if TCP fails"
                                                    )
                                                except Exception:
                                                    pass
                                        elif b"invalid data found when processing input" in tl:
                                            reconnect_delay = max(
                                                reconnect_delay,
                                                8.0 + min(6.0, float(empty_sessions) * 1.5),
                                            )
                                    if b"operation not permitted" in tail.lower():
                                        try:
                                            self.error.emit(
                                                "RTSP blocked (Operation not permitted). "
                                                "Wait 5s, close other viewers, retry."
                                            )
                                        except Exception:
                                            pass
                                    if b"404" in tail or b"Not Found" in tail:
                                        url_fatal = True
                                        msg = (
                                            "RTSP stream not found (404). "
                                            "ZR10: use rtsp://192.168.144.25:8554/main.264 only."
                                        )
                                        try:
                                            self.error.emit(msg)
                                        except Exception:
                                            pass
                                    if b"Codec AVOption ec" in tail:
                                        print(
                                            "[VGCS:video] hint: remove -ec from FFmpeg cmd "
                                            f"(use pipeline rev {_VIDEO_PIPELINE_REV} or newer)"
                                        )
                                elif rc not in (0, None):
                                    print(
                                        f"[VGCS:video] ffmpeg exited rc={rc!r} (no stderr lines captured)"
                                    )
                            except Exception:
                                pass
                            if companion_rtsp and _siyi_rtsp_camera_busy(tail):
                                tcp_138_streak += 1
                                if (
                                    str(use_transport or "").lower() == "tcp"
                                    and tcp_138_streak >= 3
                                ):
                                    try:
                                        print(
                                            "[VGCS:video] companion RTSP: TCP connect failed 3× (-138), "
                                            "switching to UDP transport"
                                        )
                                    except Exception:
                                        pass
                                    break
                            else:
                                tcp_138_streak = 0
                            if companion_rtsp:
                                self._siyi_mark_rtsp_open_result(frames_this_session, tail)
                        self._close_ffmpeg_decode_proc()

                        if self._ffmpeg_stop.is_set() or not self._running:
                            break

                        if url_fatal:
                            print(
                                "[VGCS:video] stopping RTSP retries for this URL (404 / not found)"
                            )
                            time.sleep(15.0)
                            break

                        if frames_this_session > 0:
                            empty_sessions = 0
                            try:
                                tail_b = b"".join(stderr_buf)
                                tail_txt = (
                                    tail_b[-500:].decode("utf-8", errors="replace").strip()
                                    if tail_b
                                    else ""
                                )
                            except Exception:
                                tail_b = b""
                                tail_txt = ""
                            hevc_glitch = companion_rtsp and _siyi_hevc_glitch_tail(
                                tail_b
                            )
                            print(
                                f"[VGCS:video] decode session ended (frames={frames_this_session}), "
                                f"reconnecting same transport={tr_label} rc={rc!r}"
                                + (f" — {tail_txt[:200]}" if tail_txt else "")
                            )
                            cooldown = _siyi_session_cooldown_s(tail_b, reconnect_delay)
                            if companion_rtsp:
                                siyi_hw = False
                                try:
                                    if hevc_glitch:
                                        print(
                                            "[VGCS:video] companion HEVC glitch: reopening RTSP "
                                            "(hold last frame in preview)"
                                        )
                                        reconnect_delay = 2.5
                                    elif cooldown >= 6.0:
                                        print(
                                            f"[VGCS:video] companion RTSP: cooldown {cooldown:.0f}s "
                                            "before reconnect (network / RTSP)"
                                        )
                                except Exception:
                                    pass
                                if hevc_glitch:
                                    if (
                                        str(use_transport or "").lower() == "tcp"
                                        and "udp" in transport_seq
                                    ):
                                        self._companion_transport_override = "udp"
                                        try:
                                            print(
                                                "[VGCS:video] companion HEVC on TCP: "
                                                "next session uses UDP transport"
                                            )
                                        except Exception:
                                            pass
                                    _siyi_hevc_glitch_release_sleep(
                                        brief_session=frames_this_session < 50
                                    )
                                elif _siyi_rtsp_camera_busy(tail_b):
                                    _siyi_camera_release_sleep(
                                        tail_b,
                                        1,
                                        fail_streak=int(
                                            getattr(self, "_siyi_138_fail_streak", 0) or 0
                                        ),
                                    )
                                else:
                                    time.sleep(cooldown)
                            else:
                                time.sleep(cooldown)
                            if companion_rtsp:
                                self._siyi_mark_rtsp_open_result(frames_this_session, tail_b)
                            continue

                        empty_sessions += 1
                        tail_empty = b""
                        try:
                            tail_empty = b"".join(stderr_buf)[-4000:]
                        except Exception:
                            pass
                        if transport_got_frames and empty_sessions < 5:
                            # SIYI often drops RTSP after ~60s; keep retrying same transport.
                            time.sleep(max(reconnect_delay, 2.0))
                            continue
                        if empty_sessions >= empty_session_limit:
                            print(
                                f"[VGCS:video] decode: no frames after {empty_session_limit} attempts "
                                f"on transport={tr_label} demux={demux_label}, trying next"
                            )
                            break
                        if companion_rtsp and _siyi_rtsp_camera_busy(tail_empty):
                            _siyi_camera_release_sleep(
                                tail_empty,
                                empty_sessions,
                                fail_streak=int(
                                    getattr(self, "_siyi_138_fail_streak", 0) or 0
                                ),
                            )
                            self._siyi_mark_rtsp_open_result(0, tail_empty)
                        else:
                            time.sleep(reconnect_delay)
                            if companion_rtsp and tail_empty:
                                self._siyi_mark_rtsp_open_result(0, tail_empty)

                if self._ffmpeg_stop.is_set() or not self._running:
                    break
                if url_fatal:
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
        _companion_release_rtsp_host(self.source_id, url)


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
        day_u = _normalize_companion_rtsp_url(str(day_url or "").strip())
        th_u = _normalize_companion_rtsp_url(str(thermal_url or "").strip())
        tr = str(transport or "auto").strip().lower()
        sk = str(stream_kind or "rtsp").strip().lower()
        sk = sk if sk in ("rtsp", "udp_h264", "udp_h265") else "rtsp"
        ll = bool(low_latency)
        if day_u and th_u and day_u == th_u:
            th_u = ""
        unchanged = (
            day_u == self._rtsp_day_url
            and th_u == self._rtsp_thermal_url
            and tr == self._rtsp_transport
            and sk == self._stream_kind
            and ll == self._low_latency
            and bool(self._sources)
        )
        self._rtsp_day_url = day_u
        self._rtsp_thermal_url = th_u
        self._rtsp_transport = tr
        self._low_latency = ll
        self._stream_kind = sk
        if unchanged:
            return
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
