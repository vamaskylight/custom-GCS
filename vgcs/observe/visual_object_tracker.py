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


class VisualObjectTracker:
    """Wraps a single OpenCV correlation tracker instance for one active track session.

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
