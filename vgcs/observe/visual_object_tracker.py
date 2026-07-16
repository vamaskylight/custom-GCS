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


def _create_cv2_tracker():
    import cv2

    if hasattr(cv2, "TrackerCSRT_create"):
        return cv2.TrackerCSRT_create()
    if hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerCSRT_create"):
        return cv2.legacy.TrackerCSRT_create()
    raise RuntimeError(
        "cv2 has no TrackerCSRT — install opencv-contrib-python-headless, "
        "not plain opencv-python(-headless), which lacks the tracker module"
    )


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
        try:
            tracker = _create_cv2_tracker()
            ok = tracker.init(
                frame_bgr,
                (int(bbox.x), int(bbox.y), int(bbox.w), int(bbox.h)),
            )
        except Exception:
            return False
        if ok is False:
            return False
        self._tracker = tracker
        self._active = True
        self._lost_streak = 0
        return True

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
