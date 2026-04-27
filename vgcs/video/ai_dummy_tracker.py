from __future__ import annotations

from dataclasses import dataclass

from PySide6.QtCore import QObject, Signal

from vgcs.video.pipeline import AiVideoHook, VideoFrame


@dataclass(frozen=True)
class DummyDetection:
    source_id: str
    timestamp_ms: int
    label: str


class DummyAiTracker(QObject, AiVideoHook):
    """
    Example AI hook that can be swapped with a real tracker later.

    This does no heavy compute; it just emits a heartbeat detection every N frames.
    """

    detection = Signal(object)  # DummyDetection

    def __init__(self, *, every_n_frames: int = 30, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._n = max(1, int(every_n_frames))
        self._count = 0

    def on_frame(self, frame: VideoFrame) -> None:
        self._count += 1
        if (self._count % self._n) != 0:
            return
        self.detection.emit(
            DummyDetection(
                source_id=frame.meta.source_id,
                timestamp_ms=int(frame.meta.timestamp_ms),
                label="dummy",
            )
        )

