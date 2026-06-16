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


@dataclass(frozen=True)
class VideoOverlayMark:
    """Normalized video observation click (0..1) with optional DOOAF role."""

    x: float
    y: float
    role: str = ""
    index: int = 0


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
        self._video_marks: list[VideoOverlayMark] = []
        # Normalized segments (x1,y1,x2,y2) in widget coords + ground distance label.
        self._measure_segments: list[tuple[float, float, float, float, str]] = []

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

    def set_video_marks(
        self,
        marks: list[VideoOverlayMark | tuple[float, float] | dict[str, object]],
    ) -> None:
        out: list[VideoOverlayMark] = []
        for raw in marks:
            if isinstance(raw, VideoOverlayMark):
                out.append(raw)
                continue
            if isinstance(raw, tuple) and len(raw) >= 2:
                out.append(
                    VideoOverlayMark(float(raw[0]), float(raw[1]), str(raw[2] or ""), 0)
                    if len(raw) >= 3
                    else VideoOverlayMark(float(raw[0]), float(raw[1]))
                )
                continue
            try:
                out.append(
                    VideoOverlayMark(
                        x=float(raw.get("x", 0)),
                        y=float(raw.get("y", 0)),
                        role=str(raw.get("role", "") or ""),
                        index=int(raw.get("index", 0) or 0),
                    )
                )
            except (TypeError, ValueError, AttributeError):
                continue
        self._video_marks = out
        self.update()

    def set_target_measure_segments(
        self, segments: list[tuple[float, float, float, float, str]]
    ) -> None:
        """Draw dashed lines between video marks; label is ground distance (m)."""
        out: list[tuple[float, float, float, float, str]] = []
        for raw in segments:
            try:
                out.append(
                    (
                        float(raw[0]),
                        float(raw[1]),
                        float(raw[2]),
                        float(raw[3]),
                        str(raw[4] or ""),
                    )
                )
            except (IndexError, TypeError, ValueError):
                continue
        self._measure_segments = out
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
        self._measure_segments.clear()
        self.update()

    def resizeEvent(self, event) -> None:  # noqa: N802
        super().resizeEvent(event)
        self.update()

    @staticmethod
    def _split_measure_label(label: str) -> tuple[str, str]:
        sep = " — "
        if sep in label:
            a, b = label.split(sep, 1)
            return a.strip(), b.strip()
        return label.strip(), ""

    def _draw_measure_label(
        self,
        p: QPainter,
        ax: float,
        ay: float,
        bx: float,
        by: float,
        label: str,
    ) -> None:
        mx = (ax + bx) / 2.0
        my = (ay + by) / 2.0
        line1, line2 = self._split_measure_label(label)
        warn = "not level" in line2.lower() or "may read high" in line2.lower()
        lines = [line1] if line1 else []
        if line2:
            lines.append(line2)
        if not lines:
            return
        font_main = QFont("Segoe UI", 10, QFont.Weight.Bold)
        font_warn = QFont("Segoe UI", 8, QFont.Weight.DemiBold)
        pad_x, pad_y = 8, 6
        line_gap = 2
        p.setFont(font_main)
        m_main = p.fontMetrics()
        p.setFont(font_warn)
        m_warn = p.fontMetrics()
        tw = max(
            m_main.horizontalAdvance(lines[0]) if lines else 0,
            m_warn.horizontalAdvance(lines[1]) if len(lines) > 1 else 0,
        ) + pad_x * 2
        th = (
            (m_main.height() if lines else 0)
            + (m_warn.height() + line_gap if len(lines) > 1 else 0)
            + pad_y * 2
        )
        tx = int(mx - tw / 2)
        ty = int(my - th / 2)
        bg = QColor(80, 45, 10, 220) if warn and len(lines) > 1 else QColor(0, 0, 0, 200)
        p.fillRect(tx, ty, int(tw), int(th), bg)
        y_text = ty + pad_y + m_main.ascent()
        if lines[0]:
            p.setFont(font_main)
            p.setPen(QColor(255, 220, 120) if warn else QColor(255, 200, 240))
            p.drawText(tx + pad_x, y_text, lines[0])
        if len(lines) > 1:
            p.setFont(font_warn)
            p.setPen(QColor(255, 200, 100))
            y_text += m_main.height() + line_gap
            p.drawText(tx + pad_x, y_text + m_warn.ascent() - m_main.ascent(), lines[1])

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
        if not self._detections and not self._video_marks and not self._measure_segments:
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
        for x1, y1, x2, y2, label in self._measure_segments:
            ax = cl + x1 * cw
            ay = ct + y1 * ch
            bx = cl + x2 * cw
            by = ct + y2 * ch
            p.setPen(QPen(QColor(255, 120, 200, 230), 2, Qt.PenStyle.DashLine))
            p.drawLine(int(ax), int(ay), int(bx), int(by))
            if label:
                self._draw_measure_label(p, ax, ay, bx, by, label)

        for mark in self._video_marks:
            xn, yn = float(mark.x), float(mark.y)
            cx = cl + xn * cw
            cy = ct + yn * ch
            role = str(mark.role or "").strip().lower()
            if role == "intended_target":
                fill = QColor(34, 197, 94, 235)
                ring = QColor(187, 247, 208, 255)
            elif role == "impact":
                fill = QColor(239, 68, 68, 235)
                ring = QColor(254, 202, 202, 255)
            else:
                fill = QColor(255, 120, 60, 230)
                ring = QColor(255, 220, 120, 255)
            radius = 7
            p.setPen(QPen(ring, 3))
            p.setBrush(fill)
            p.drawEllipse(int(cx) - radius, int(cy) - radius, radius * 2, radius * 2)
            p.setPen(QPen(ring, 2))
            p.drawLine(int(cx - 14), int(cy), int(cx + 14), int(cy))
            p.drawLine(int(cx), int(cy - 14), int(cx), int(cy + 14))
            if mark.index > 0:
                label = str(mark.index)
                font = QFont("Segoe UI", 9, QFont.Weight.Bold)
                p.setFont(font)
                metrics = p.fontMetrics()
                tw = metrics.horizontalAdvance(label) + 8
                th = metrics.height() + 4
                tx = int(cx + 10)
                ty = int(cy - th - 8)
                p.fillRect(tx, ty, tw, th, QColor(0, 0, 0, 200))
                p.setPen(QColor(255, 255, 255))
                p.drawText(tx + 4, ty + metrics.ascent() + 2, label)
