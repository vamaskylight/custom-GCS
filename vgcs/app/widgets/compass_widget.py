"""Compass rose with heading needle for the telemetry dashboard."""

from __future__ import annotations

import math

from PySide6.QtCore import QPointF, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import QSizePolicy, QWidget


class CompassWidget(QWidget):
    """Painted compass: N/E/S/W rose and heading needle in degrees (0–360, CW from N)."""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._heading_deg = 0.0
        self._valid = False
        self.setMinimumSize(168, 168)
        self.setMaximumSize(220, 220)
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

    def set_heading_deg(self, deg: float) -> None:
        self._heading_deg = float(deg) % 360.0
        self._valid = True
        self.update()

    def clear(self) -> None:
        self._valid = False
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()
        cx = w / 2.0
        cy = h / 2.0
        radius = min(w, h) / 2.0 - 6.0

        painter.setPen(QPen(QColor("#3d465a"), 2))
        painter.setBrush(QColor("#1e222b"))
        painter.drawEllipse(QPointF(cx, cy), radius, radius)

        painter.setPen(QPen(QColor("#7d869c"), 1))
        for deg in range(0, 360, 30):
            rad = math.radians(deg)
            x1 = cx + (radius - 4) * math.sin(rad)
            y1 = cy - (radius - 4) * math.cos(rad)
            x2 = cx + (radius - 14) * math.sin(rad)
            y2 = cy - (radius - 14) * math.cos(rad)
            painter.drawLine(QPointF(x1, y1), QPointF(x2, y2))

        font = QFont()
        font.setPointSize(9)
        font.setBold(True)
        painter.setFont(font)
        painter.setPen(QColor("#c5cbe0"))
        for text, deg in (("N", 0), ("E", 90), ("S", 180), ("W", 270)):
            rad = math.radians(deg)
            tx = cx + (radius - 26) * math.sin(rad) - 6
            ty = cy - (radius - 26) * math.cos(rad) + 5
            painter.drawText(QPointF(tx, ty), text)

        painter.setPen(QColor("#9aa3b8"))
        painter.setFont(QFont())
        painter.drawText(8, h - 8, "Heading" if self._valid else "Heading —")

        if self._valid:
            rad = math.radians(self._heading_deg)
            painter.setPen(QPen(QColor("#4ade80"), 3))
            nx = cx + (radius - 18) * math.sin(rad)
            ny = cy - (radius - 18) * math.cos(rad)
            painter.drawLine(QPointF(cx, cy), QPointF(nx, ny))
            painter.setBrush(QColor("#4ade80"))
            painter.drawEllipse(QPointF(nx, ny), 4, 4)

            painter.setPen(QColor("#dce1ef"))
            painter.drawText(8, 18, f"{self._heading_deg:.0f}°")
