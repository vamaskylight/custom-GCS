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


def _create_cv2_tracker(*, prefer_legacy: bool = False):
    import cv2

    has_legacy = hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerCSRT_create")
    if prefer_legacy and has_legacy:
        return cv2.legacy.TrackerCSRT_create()
    if hasattr(cv2, "TrackerCSRT_create"):
        return cv2.TrackerCSRT_create()
    if has_legacy:
        return cv2.legacy.TrackerCSRT_create()
    raise RuntimeError(
        "cv2 has no TrackerCSRT — install opencv-contrib-python-headless, "
        "not plain opencv-python(-headless), which lacks the tracker module"
    )


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
    """Wraps a single OpenCV CSRT tracker instance for one active track session.

    CSRT (Channel and Spatial Reliability Tracking) is a classical correlation-
    filter tracker: accurate and reasonably robust to partial occlusion/scale
    change for a single object once initialized on a bounding box, without
    needing a trained detection model. It has no notion of object *class* — it
    tracks "whatever was in this box," which is exactly the click-to-track
    semantics this feature needs.
    """

    def __init__(self) -> None:
        self._tracker = None
        self._active = False
        self._lost_streak = 0

    def start(self, frame_bgr: np.ndarray, bbox: TrackBox) -> bool:
        """Initialize tracking on ``bbox`` within ``frame_bgr``. Returns success.

        cv2's init() takes an integer (x, y, w, h) tuple — passing floats
        raises a cv2.error — and returns None on success rather than True, so
        "no exception and not an explicit False" is what counts as success.
        """
        self.stop()
        int_bbox = (int(bbox.x), int(bbox.y), int(bbox.w), int(bbox.h))
        last_ex: Exception | None = None
        # Try the modern Tracking-module CSRT first, then fall back to the
        # legacy module's separate C++ implementation on failure — a native
        # crash in one code path isn't necessarily present in the other, and
        # this costs nothing when the modern path already works fine.
        for prefer_legacy in (False, True):
            try:
                tracker = _create_cv2_tracker(prefer_legacy=prefer_legacy)
                ok = tracker.init(frame_bgr, int_bbox)
            except Exception as ex:
                last_ex = ex
                continue
            if ok is False:
                continue
            self._tracker = tracker
            self._active = True
            self._lost_streak = 0
            if prefer_legacy:
                print("[VGCS:m14] modern TrackerCSRT failed, legacy TrackerCSRT succeeded")
            return True
        # Both attempts failed — this used to be swallowed with no trace at
        # all (a missing opencv-contrib-python-headless install fails here
        # with zero console output, indistinguishable from any other cause).
        try:
            print(f"[VGCS:m14] tracker init failed: {last_ex!r}")
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
