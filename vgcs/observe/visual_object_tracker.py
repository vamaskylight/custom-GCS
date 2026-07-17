"""M14 — software-side single-object visual tracker for continuous gimbal-follow.

Neither C12 nor C13 has a confirmed firmware capability to continuously track and
follow an arbitrary clicked target (see DOCS/SKYDROID-TOP-PROTOCOL.md — GOT/SUM are
C12-only per Skydroid's own protocol doc, and even on C12 there is no tracked-pixel
feedback from the firmware at all). This module tracks the target in software, on
the same video frames already flowing through the GCS, so the operator gets a real,
continuously-updated target position every frame regardless of what the camera
firmware does or doesn't support.
"""

from __future__ import annotations

import os
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class TrackBox:
    """Target bounding box in pixel space, top-left origin."""

    x: float
    y: float
    w: float
    h: float

    @property
    def center_xy(self) -> tuple[float, float]:
        return (self.x + self.w / 2.0, self.y + self.h / 2.0)


_ipp_disabled = False


def _disable_ipp_if_needed() -> None:
    """Turn off OpenCV's Intel IPP acceleration, once per process.

    Root cause of the field CSRT crash: this wheel is built with Intel IPP
    bundled in (`cv2.getBuildInformation()` shows "Intel IPP: 2022.2.0").
    IPP is a well-documented source of exactly this failure signature — a
    bare, non-descriptive native exception, only on some non-Intel-genuine
    CPUs, only in IPP-accelerated code paths (CSRT's internal DFT-based
    correlation is exactly that kind of path; simpler ops like cv2.resize
    either don't hit the same IPP routine or hit a more robust one, which is
    why `basic native op (cv2.resize) OK` didn't rule this out earlier).
    `cv2.ipp.setUseIPP(False)` is OpenCV's own supported switch for this —
    confirmed locally that CSRT still initializes and tracks correctly with
    IPP off (just loses IPP's speed optimization, not correctness), and cv2
    is used ONLY in this module in this codebase, so this has zero blast
    radius elsewhere. Safe to apply unconditionally, not just for the
    affected machine.
    """
    global _ipp_disabled
    if _ipp_disabled:
        return
    _ipp_disabled = True
    try:
        import cv2

        if hasattr(cv2, "ipp") and hasattr(cv2.ipp, "setUseIPP"):
            cv2.ipp.setUseIPP(False)
    except Exception:
        pass


def _weak_tracker_fallback_allowed() -> bool:
    """Off by default — see the field incident recorded in ``_tracker_candidates``.

    A crashed tracker.init() is recoverable (caught cleanly, "track failed"
    status, GCS keeps running). A crashed tracker.update() on some machines
    is NOT recoverable the same way: it took the whole application down with
    no exception, no traceback, mid-operation — losing all telemetry/control
    visibility, not just tracking. Until that's understood, defaulting to
    "fail cleanly, no track" is the safe choice over "silently run a tracker
    that might kill the GCS." Set VGCS_M14_ALLOW_WEAK_TRACKER_FALLBACK=1 to
    opt back in for a supervised/ground test.
    """
    raw = str(
        os.environ.get("VGCS_M14_ALLOW_WEAK_TRACKER_FALLBACK", "0") or "0"
    ).strip().lower()
    return raw in ("1", "true", "yes", "on")


def _tracker_candidates() -> list[tuple[str, "object"]]:
    """Ordered (label, factory) list of tracker constructors to try.

    CSRT (Channel and Spatial Reliability Tracking) is the accurate,
    occlusion-robust default — both its modern and legacy implementations
    are tried first.

    KCF/MOSSE were added as a further fallback after a field case where CSRT
    (both implementations, on a machine with every CPU SIMD extension this
    build could use, and where basic cv2 ops like resize worked fine) still
    threw a bare native C++ exception at init() every time. That part is
    safe — a failed init() is a caught, recoverable Python exception.

    But on that same machine, once KCF's init() *succeeded*, the whole GCS
    process crashed a couple seconds into tracking (right when the periodic
    update() calls would have started) — no exception, no traceback, just
    gone. A hard native crash inside update() can't be caught with
    try/except the way init() failures can, and taking down the entire GCS
    (losing telemetry/control visibility, not just tracking) is a much
    worse failure than "tracking doesn't start." So KCF/MOSSE are gated
    behind ``_weak_tracker_fallback_allowed()`` (default OFF) until this is
    understood — see [[m13-track-state]] memory / DOCS for the incident.
    """
    import cv2

    _disable_ipp_if_needed()

    candidates: list[tuple[str, "object"]] = []
    if hasattr(cv2, "TrackerCSRT_create"):
        candidates.append(("CSRT", cv2.TrackerCSRT_create))
    if hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerCSRT_create"):
        candidates.append(("CSRT-legacy", cv2.legacy.TrackerCSRT_create))
    if _weak_tracker_fallback_allowed():
        if hasattr(cv2, "TrackerKCF_create"):
            candidates.append(("KCF", cv2.TrackerKCF_create))
        if hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerKCF_create"):
            candidates.append(("KCF-legacy", cv2.legacy.TrackerKCF_create))
        if hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerMOSSE_create"):
            candidates.append(("MOSSE-legacy", cv2.legacy.TrackerMOSSE_create))
    if not candidates:
        raise RuntimeError(
            "cv2 has no tracker algorithms — install opencv-contrib-python-headless, "
            "not plain opencv-python(-headless), which lacks the tracker module"
        )
    return candidates


def _cv2_diagnostic_report() -> list[str]:
    """Multi-line diagnostic dump for when tracker init fails natively.

    A bare "Unknown C++ exception from OpenCV code" at tracker init (not at
    import) could mean a prebuilt wheel's SIMD-optimized code path (CSRT's
    HOG/FFT machinery) is hitting a CPU instruction set that isn't actually
    there — or it could mean the whole native cv2 runtime is broken on this
    machine, unrelated to tracking specifically. Every check below is
    independently wrapped so one failing check doesn't hide the others, and
    every failure prints the real exception instead of swallowing it — the
    previous version of this function returned the single word "unavailable"
    on ANY internal error, which itself turned out to be a dead end in the
    field: it gave no way to tell "no CPU flags to report" apart from
    "checking CPU flags itself crashed."
    """
    lines: list[str] = []
    try:
        import cv2

        lines.append(f"cv2 {cv2.__version__} @ {cv2.__file__}")
    except Exception as ex:
        lines.append(f"cv2 import/version check failed: {ex!r}")
        return lines

    # Does ANY native cv2 op work, or is the whole runtime broken (not a
    # CSRT-specific issue at all)?
    try:
        tiny = np.zeros((8, 8, 3), dtype=np.uint8)
        cv2.resize(tiny, (4, 4))
        lines.append("basic native op (cv2.resize) OK - runtime works, failure is tracker-specific")
    except Exception as ex:
        lines.append(f"basic native op (cv2.resize) ALSO FAILED: {ex!r} - native runtime is broken, not just CSRT")

    try:
        # '*'-prefixed entries are detected/available on this CPU; the rest
        # are compiled-in dispatch targets the CPU does NOT have.
        lines.append(f"detected CPU features: {cv2.getCPUFeaturesLine()!r}")
        lines.append(f"CPU count: {cv2.getNumberOfCPUs()}")
    except Exception as ex:
        lines.append(f"getCPUFeaturesLine() failed: {ex!r}")

    try:
        if hasattr(cv2, "ipp") and hasattr(cv2.ipp, "useIPP"):
            lines.append(f"Intel IPP active: {cv2.ipp.useIPP()} (should be False - disabled at startup)")
        else:
            lines.append("Intel IPP: cv2.ipp.useIPP() not available in this build")
    except Exception as ex:
        lines.append(f"IPP status check failed: {ex!r}")

    try:
        info = cv2.getBuildInformation()
        idx = info.find("CPU/HW features")
        if idx >= 0:
            excerpt = info[idx : idx + 700].strip()
            lines.append("build info (CPU/HW features):\n" + excerpt)
        else:
            lines.append("build info: no 'CPU/HW features' section found")
    except Exception as ex:
        lines.append(f"getBuildInformation() failed: {ex!r}")

    return lines


class _InProcessTracker:
    """The actual OpenCV correlation-tracker wrapper — runs the real cv2 calls
    in-process. NEVER instantiate this directly outside a worker process (see
    ``VisualObjectTracker`` below) — a native crash inside any of its methods
    takes down whatever process it's running in.

    Prefers CSRT (Channel and Spatial Reliability Tracking) — accurate and
    reasonably robust to partial occlusion/scale change for a single object
    once initialized on a bounding box, without needing a trained detection
    model. Falls back to KCF/MOSSE only if CSRT itself fails to initialize
    (see ``_tracker_candidates()``). None of these have a notion of object
    *class* — they track "whatever was in this box," which is exactly the
    click-to-track semantics this feature needs. Check ``algo_used`` after a
    successful ``start()`` to see which one actually initialized.
    """

    def __init__(self) -> None:
        self._tracker = None
        self._active = False
        self._lost_streak = 0
        self._algo_used = ""

    @property
    def algo_used(self) -> str:
        """Which tracker algorithm actually initialized (e.g. 'CSRT', 'KCF-legacy')."""
        return self._algo_used

    def start(self, frame_bgr: np.ndarray, bbox: TrackBox) -> bool:
        """Initialize tracking on ``bbox`` within ``frame_bgr``. Returns success.

        cv2's init() takes an integer (x, y, w, h) tuple — passing floats
        raises a cv2.error — and returns None on success rather than True, so
        "no exception and not an explicit False" is what counts as success.

        Tries every algorithm in ``_tracker_candidates()`` in order (CSRT
        first, weaker trackers only as fallback) — a native crash in one
        implementation/algorithm isn't necessarily present in another.
        """
        self.stop()
        int_bbox = (int(bbox.x), int(bbox.y), int(bbox.w), int(bbox.h))
        last_ex: Exception | None = None
        try:
            candidates = _tracker_candidates()
        except Exception as ex:
            print(f"[VGCS:m14] tracker init failed: {ex!r}")
            return False
        for label, factory in candidates:
            try:
                tracker = factory()
                ok = tracker.init(frame_bgr, int_bbox)
            except Exception as ex:
                last_ex = ex
                continue
            if ok is False:
                continue
            self._tracker = tracker
            self._active = True
            self._lost_streak = 0
            self._algo_used = label
            if label != candidates[0][0]:
                print(f"[VGCS:m14] {candidates[0][0]} failed, fell back to {label} tracker")
            return True
        # Every candidate failed — this used to be swallowed with no trace at
        # all (a missing opencv-contrib-python-headless install fails here
        # with zero console output, indistinguishable from any other cause).
        try:
            print(f"[VGCS:m14] tracker init failed (tried {[c[0] for c in candidates]}): {last_ex!r}")
            for line in _cv2_diagnostic_report():
                print(f"[VGCS:m14] diag: {line}")
        except Exception:
            pass
        return False

    def update(self, frame_bgr: np.ndarray) -> tuple[bool, TrackBox | None]:
        """Advance the tracker one frame. Returns (ok, box) — box is None if lost."""
        if not self._active or self._tracker is None:
            return False, None
        try:
            ok, box = self._tracker.update(frame_bgr)
        except Exception:
            ok, box = False, None
        if not ok or box is None:
            self._lost_streak += 1
            return False, None
        self._lost_streak = 0
        x, y, w, h = box
        return True, TrackBox(float(x), float(y), float(w), float(h))

    def is_active(self) -> bool:
        return bool(self._active)

    def lost_streak(self) -> int:
        return int(self._lost_streak)

    def stop(self) -> None:
        self._tracker = None
        self._active = False
        self._lost_streak = 0
        self._algo_used = ""


_M14_WORKER_START_TIMEOUT_S = 8.0
_M14_WORKER_UPDATE_TIMEOUT_S = 1.0


def _read_exactly(stream, n: int) -> bytes | None:
    buf = bytearray()
    while len(buf) < n:
        chunk = stream.read(n - len(buf))
        if not chunk:
            return None  # EOF — other end closed/died
        buf.extend(chunk)
    return bytes(buf)


def _send_framed(stream, obj) -> bool:
    """Length-prefixed pickle frame. Returns False (never raises) on failure —
    a broken pipe here just means the worker is gone, not a caller-facing error."""
    import pickle

    try:
        data = pickle.dumps(obj, protocol=pickle.HIGHEST_PROTOCOL)
        stream.write(len(data).to_bytes(4, "little"))
        stream.write(data)
        stream.flush()
        return True
    except Exception:
        return False


def _recv_framed(stream):
    """Inverse of ``_send_framed``. Returns None on EOF/any failure."""
    import pickle

    header = _read_exactly(stream, 4)
    if header is None:
        return None
    n = int.from_bytes(header, "little")
    data = _read_exactly(stream, n)
    if data is None:
        return None
    try:
        return pickle.loads(data)
    except Exception:
        return None


class VisualObjectTracker:
    """Process-isolated tracker: same public interface as ``_InProcessTracker``
    (``start``/``update``/``stop``/``is_active``/``lost_streak``/``algo_used``),
    but every actual cv2 call happens in a disposable child process (`python
    -m vgcs.observe.tracker_worker_main`, launched via ``subprocess.Popen``
    with stdin/stdout pipes — the same proven mechanism this codebase
    already uses for the FFmpeg video decode subprocess, deliberately NOT
    ``multiprocessing.Process``; see ``tracker_worker_main.py`` for why). If
    that process dies — for ANY reason, including a hard native crash that
    no Python try/except could ever catch — this class reports it as a
    normal tracking failure ("track failed" / "target lost") instead of the
    crash propagating into the GCS. A fresh worker process is spawned on
    each ``start()``, so a new click-to-track attempt always gets a clean
    process, not one carrying over whatever state (possibly corrupted) an
    earlier attempt left behind.
    """

    def __init__(self) -> None:
        self._proc = None
        self._resp_queue = None
        self._active = False
        self._lost_streak = 0
        self._algo_used = ""

    @property
    def algo_used(self) -> str:
        return self._algo_used

    def start(self, frame_bgr: np.ndarray, bbox: TrackBox) -> bool:
        self.stop()
        try:
            import subprocess
            import sys

            proc = subprocess.Popen(
                [sys.executable, "-m", "vgcs.observe.tracker_worker_main"],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=None,  # inherit console — worker's own diagnostic prints stay visible
            )
        except Exception as ex:
            print(f"[VGCS:m14] tracker worker process failed to start: {ex!r}")
            return False
        self._proc = proc
        self._resp_queue = self._start_reader(proc)
        resp = self._request(("start", frame_bgr, bbox), _M14_WORKER_START_TIMEOUT_S)
        if resp is None:
            if proc.poll() is not None:
                print("[VGCS:m14] tracker worker crashed during init (contained - GCS unaffected)")
            else:
                print("[VGCS:m14] tracker worker did not respond to init in time")
            self.stop()
            return False
        _, ok, algo = resp
        if not ok:
            self.stop()
            return False
        self._active = True
        self._lost_streak = 0
        self._algo_used = str(algo)
        return True

    def update(self, frame_bgr: np.ndarray) -> tuple[bool, TrackBox | None]:
        if not self._active or self._proc is None:
            return False, None
        if self._proc.poll() is not None:
            print("[VGCS:m14] tracker worker process died (contained - GCS unaffected)")
            self._active = False
            self._lost_streak += 1
            return False, None
        resp = self._request(("update", frame_bgr), _M14_WORKER_UPDATE_TIMEOUT_S)
        if resp is None:
            if self._proc.poll() is not None:
                print("[VGCS:m14] tracker worker process died during update (contained - GCS unaffected)")
            self._lost_streak += 1
            return False, None
        _, ok, box, lost = resp
        self._lost_streak = int(lost)
        if not ok or box is None:
            return False, None
        return True, box

    def is_active(self) -> bool:
        return bool(self._active)

    def lost_streak(self) -> int:
        return int(self._lost_streak)

    def stop(self) -> None:
        proc = self._proc
        self._proc = None
        self._resp_queue = None
        self._active = False
        self._lost_streak = 0
        self._algo_used = ""
        if proc is None:
            return
        try:
            if proc.stdin is not None:
                _send_framed(proc.stdin, ("stop",))
                proc.stdin.close()
        except Exception:
            pass
        try:
            proc.wait(timeout=1.5)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass

    def _start_reader(self, proc):
        """Background thread continuously reading framed responses off the
        worker's stdout into a queue — mirrors the existing `_drain_stderr`
        pattern already used for the FFmpeg subprocess in pipeline.py. A
        sentinel `None` is pushed when the stream ends (worker died/closed),
        so `_request`'s `queue.get()` never blocks forever on a dead worker
        beyond its explicit timeout.
        """
        import queue
        import threading

        q: "queue.Queue" = queue.Queue()

        def _reader() -> None:
            try:
                while True:
                    resp = _recv_framed(proc.stdout)
                    if resp is None:
                        break
                    q.put(resp)
            except Exception:
                pass
            finally:
                try:
                    q.put(None)
                except Exception:
                    pass

        threading.Thread(target=_reader, daemon=True).start()
        return q

    def _request(self, msg: tuple, timeout_s: float) -> tuple | None:
        """Send ``msg`` and wait up to ``timeout_s`` for a response.

        Returns None on ANY failure to get a well-formed reply — dead worker,
        broken pipe, or timeout are all treated identically by the caller.
        """
        proc = self._proc
        q = self._resp_queue
        if proc is None or q is None or proc.stdin is None:
            return None
        import queue

        if not _send_framed(proc.stdin, msg):
            return None
        try:
            return q.get(timeout=timeout_s)
        except queue.Empty:
            return None


def bbox_around_point(
    cx: float, cy: float, *, box_w: float, box_h: float, frame_w: int, frame_h: int
) -> TrackBox:
    """A ``box_w``x``box_h`` box centered on a click point, clamped to the frame."""
    half_w, half_h = float(box_w) / 2.0, float(box_h) / 2.0
    x = max(0.0, min(float(frame_w) - box_w, cx - half_w))
    y = max(0.0, min(float(frame_h) - box_h, cy - half_h))
    w = min(float(box_w), float(frame_w))
    h = min(float(box_h), float(frame_h))
    return TrackBox(x, y, w, h)
