"""
Native Qt widgets for map camera rail — git `#cameraRail` core + reference slider/segment visuals.

Git `e48c1a7` defines `#cameraRail`, `#cameraTopRow`, `#camRecordBtn`, `#camTimer`, `#camSettingsBtn` only;
zoom/focus/gimbal/observe strips are native MAVLink controls styled to match product reference.
"""

from __future__ import annotations

from PySide6.QtCore import QPointF, QRectF, QSize, Qt, Signal
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)


class CamRecordArch(QWidget):
    """Record control under arched top border (`#camRecordArch` in map_widget QSS)."""

    def __init__(self, record_btn: QWidget, parent=None) -> None:
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        self._arch = QFrame()
        self._arch.setObjectName("camRecordArch")
        inner = QVBoxLayout(self._arch)
        inner.setContentsMargins(3, 2, 3, 2)
        inner.setSpacing(0)
        inner.addWidget(record_btn, 0, Qt.AlignmentFlag.AlignHCenter)
        lay.addWidget(self._arch)


class CamRailBiSlider(QWidget):
    """
    Dark horizontal track + **white circular thumb** (reference UI).
    - Zoom/focus: left / right half → discrete MAVLink steps (hidden −/+ buttons).
    - Gimbal (`tri_gimbal`): left / center / right thirds → yaw L, pitch ↑, yaw R.
    """

    left_step = Signal()
    right_step = Signal()
    center_step = Signal()

    def __init__(
        self,
        *,
        center_glyph: str = "",
        tri_gimbal: bool = False,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self._glyph = center_glyph
        self._tri = bool(tri_gimbal)
        self._thumb_t = 0.5  # 0..1 along track (visual only; idle centered)
        self.setMinimumHeight(28)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setCursor(Qt.CursorShape.PointingHandCursor)

    def paintEvent(self, event) -> None:  # noqa: N802
        del event
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = float(self.width()), float(self.height())
        outer = QRectF(2.5, 3.0, w - 5.0, h - 6.0)
        p.setPen(QPen(QColor(196, 209, 230, 48), 1.0))
        p.setBrush(QColor(22, 27, 38, 220))
        p.drawRoundedRect(outer, 7.0, 7.0)

        # Inner groove (darker channel)
        groove = outer.adjusted(6.0, 6.0, -6.0, -6.0)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor(10, 12, 20, 252))
        p.drawRoundedRect(groove, 4.0, 4.0)

        # Thumb travels along groove horizontal span
        gx0, gx1 = groove.left(), groove.right()
        span = max(1.0, gx1 - gx0)
        tx = gx0 + self._thumb_t * span
        cy = groove.center().y()
        tr = 5.5
        p.setPen(QPen(QColor(240, 245, 255, 55), 1.0))
        p.setBrush(QColor(248, 250, 255))
        p.drawEllipse(QRectF(tx - tr, cy - tr, 2 * tr, 2 * tr))

        if self._glyph and self._tri:
            p.setPen(QColor(28, 32, 44))
            font = QFont("Segoe UI", 9, QFont.Weight.Bold)
            p.setFont(font)
            p.drawText(QRectF(tx - 9, cy - 7, 18, 14), Qt.AlignmentFlag.AlignCenter, self._glyph)

    def mousePressEvent(self, event) -> None:  # noqa: N802
        if event.button() != Qt.MouseButton.LeftButton:
            return super().mousePressEvent(event)
        x = float(event.position().x())
        wpx = max(1, self.width())
        if self._tri and self._glyph:
            lo, hi = wpx * 0.33, wpx * 0.67
            if x < lo:
                self._thumb_t = 0.18
                self.left_step.emit()
            elif x > hi:
                self._thumb_t = 0.82
                self.right_step.emit()
            else:
                self._thumb_t = 0.5
                self.center_step.emit()
            self.update()
            return
        self._thumb_t = 0.25 if x < wpx * 0.5 else 0.75
        if x < wpx * 0.5:
            self.left_step.emit()
        else:
            self.right_step.emit()
        self.update()
        super().mousePressEvent(event)

    def sizeHint(self) -> QSize:
        return QSize(158, 28)


class CamObserveSegment(QFrame):
    """Segmented control: Target | Clip (reference OBSERVE primary row)."""

    def __init__(self, target_btn: QWidget, clip_btn: QWidget, parent=None) -> None:
        super().__init__(parent)
        self.setObjectName("camObserveSegment")
        self.setFrameShape(QFrame.Shape.NoFrame)
        hl = QHBoxLayout(self)
        hl.setContentsMargins(0, 0, 0, 0)
        hl.setSpacing(0)
        target_btn.setObjectName("observeTarget")
        clip_btn.setObjectName("observeClip")
        hl.addWidget(target_btn, 1)
        hl.addWidget(clip_btn, 1)
