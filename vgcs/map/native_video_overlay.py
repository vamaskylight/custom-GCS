"""Transparent overlay for native Qt video preview (M7 detection boxes + observation marks)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import QWidget


@dataclass(frozen=True)
class VideoOverlayDetection:
    """Normalized box within the video content rect (0..1)."""

    x: float
    y: float
    w: float
    h: float
    label: str = ""
    score: float | None = None


class NativeVideoOverlayLayer(QWidget):
    """
    Child of ``QLabel`` video preview — draws detection boxes/labels and manual video marks.

    Mouse events pass through to the parent label (observation clicks stay on the preview).
    """

    def __init__(self, parent: QWidget) -> None:
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self.setAttribute(Qt.WidgetAttribute.WA_NoSystemBackground, True)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)
        self.setStyleSheet("background: transparent;")
        self._content_rect: dict[str, float] | None = None
        self._detections: list[VideoOverlayDetection] = []
        # Normalized 0..1 relative to this widget (matches click handling on the QLabel).
        self._video_marks: list[tuple[float, float]] = []

    def set_content_rect(self, rect: dict[str, float] | None) -> None:
        self._content_rect = dict(rect) if rect else None
        self.update()

    def set_detections(self, items: list[VideoOverlayDetection] | list[dict[str, Any]]) -> None:
        out: list[VideoOverlayDetection] = []
        for raw in items:
            if isinstance(raw, VideoOverlayDetection):
                out.append(raw)
                continue
            try:
                out.append(
                    VideoOverlayDetection(
                        x=float(raw.get("x", 0)),
                        y=float(raw.get("y", 0)),
                        w=float(raw.get("w", 0)),
                        h=float(raw.get("h", 0)),
                        label=str(raw.get("label", "") or ""),
                        score=(
                            float(raw["score"])
                            if raw.get("score") is not None
                            else None
                        ),
                    )
                )
            except (TypeError, ValueError):
                continue
        self._detections = out
        self.update()

    def set_video_marks(self, marks: list[tuple[float, float]]) -> None:
        self._video_marks = [(float(x), float(y)) for x, y in marks]
        self.update()

    def clear_detections(self) -> None:
        self._detections.clear()
        self.update()

    def clear_video_marks(self) -> None:
        self._video_marks.clear()
        self.update()

    def clear_all(self) -> None:
        self._detections.clear()
        self._video_marks.clear()
        self.update()

    def resizeEvent(self, event) -> None:  # noqa: N802
        super().resizeEvent(event)
        self.update()

    def _content_qrect(self) -> tuple[float, float, float, float]:
        """Return (left, top, width, height) in widget coordinates for the video pixmap area."""
        w = float(max(1, self.width()))
        h = float(max(1, self.height()))
        cr = self._content_rect
        if not cr:
            return 0.0, 0.0, w, h
        left = float(cr.get("cr_left", 0.0)) + float(cr.get("ox", 0.0))
        top = float(cr.get("cr_top", 0.0)) + float(cr.get("oy", 0.0))
        pw = float(cr.get("pw", w))
        ph = float(cr.get("ph", h))
        return left, top, max(1.0, pw), max(1.0, ph)

    def paintEvent(self, event) -> None:  # noqa: N802
        del event
        if not self._detections and not self._video_marks:
            return
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        cl, ct, cw, ch = self._content_qrect()

        for det in self._detections:
            bx = cl + det.x * cw
            by = ct + det.y * ch
            bw = max(2.0, det.w * cw)
            bh = max(2.0, det.h * ch)
            p.setPen(QPen(QColor(80, 255, 140, 230), 2))
            p.setBrush(QColor(80, 255, 140, 36))
            p.drawRect(int(bx), int(by), int(bw), int(bh))
            caption = str(det.label or "target").strip()
            if det.score is not None:
                caption = f"{caption} {det.score:.0%}" if caption else f"{det.score:.0%}"
            if caption:
                font = QFont("Segoe UI", 9, QFont.Weight.Bold)
                p.setFont(font)
                metrics = p.fontMetrics()
                tw = metrics.horizontalAdvance(caption) + 10
                th = metrics.height() + 6
                tx = int(bx)
                ty = max(0, int(by) - th - 2)
                p.fillRect(tx, ty, tw, th, QColor(0, 0, 0, 190))
                p.setPen(QColor(220, 255, 230))
                p.drawText(tx + 5, ty + metrics.ascent() + 3, caption)

        ww = float(max(1, self.width()))
        wh = float(max(1, self.height()))
        for xn, yn in self._video_marks:
            cx = xn * ww
            cy = yn * wh
            p.setPen(QPen(QColor(255, 210, 70, 240), 2))
            p.setBrush(QColor(255, 80, 60, 220))
            p.drawEllipse(int(cx) - 5, int(cy) - 5, 10, 10)
            p.drawLine(int(cx - 12), int(cy), int(cx + 12), int(cy))
            p.drawLine(int(cx), int(cy - 12), int(cx), int(cy + 12))
