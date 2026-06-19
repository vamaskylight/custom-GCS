"""
Native Qt widgets for map camera rail — git `#cameraRail` core + reference slider/segment visuals.

Git `e48c1a7` defines `#cameraRail`, `#cameraTopRow`, `#camRecordBtn`, `#camTimer`, `#camSettingsBtn` only;
zoom/focus/gimbal/observe strips are native MAVLink controls styled to match product reference.
"""

from __future__ import annotations

from PySide6.QtCore import QPointF, QRectF, QSize, Qt, Signal
from PySide6.QtGui import QColor, QFont, QIcon, QPainter, QPen
from PySide6.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from vgcs.observe.dooaf import DOOAF_ROLE_IMPACT


class CamRailShowHandle(QFrame):
    """Edge drawer tab — shown when the operator collapses the camera rail."""

    WIDTH_PX = 36
    HEIGHT_PX = 92

    clicked = Signal()

    def __init__(self, parent=None, *, icon: QIcon | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("camRailShowHandle")
        self.setFixedSize(self.WIDTH_PX, self.HEIGHT_PX)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setToolTip("Show camera panel")
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.setStyleSheet(
            "QFrame#camRailShowHandle {"
            " background: rgba(24, 30, 40, 0.96);"
            " border: 1px solid rgba(196, 209, 230, 90);"
            " border-right: none;"
            " border-top-left-radius: 12px;"
            " border-bottom-left-radius: 12px;"
            "}"
            "QFrame#camRailShowHandle:hover {"
            " background: rgba(46, 56, 72, 0.98);"
            " border-color: rgba(229, 237, 251, 130);"
            "}"
            "QLabel#camRailShowChevron {"
            " color: #dce5f5; font-size: 15px; font-weight: 700;"
            " background: transparent; border: none;"
            "}"
        )
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 14, 0, 12)
        lay.setSpacing(8)
        lay.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        icon_lbl = QLabel()
        icon_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        icon_lbl.setFixedSize(22, 22)
        if icon is not None and not icon.isNull():
            icon_lbl.setPixmap(icon.pixmap(22, 22))
        else:
            icon_lbl.setText("🎥")
            icon_lbl.setStyleSheet("font-size: 16px; background: transparent; border: none;")
        chev = QLabel("◀")
        chev.setObjectName("camRailShowChevron")
        chev.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lay.addWidget(icon_lbl, 0, Qt.AlignmentFlag.AlignHCenter)
        lay.addWidget(chev, 0, Qt.AlignmentFlag.AlignHCenter)
        lay.addStretch(1)

    def mouseReleaseEvent(self, event) -> None:  # type: ignore[override]
        if event.button() == Qt.MouseButton.LeftButton:
            self.clicked.emit()
            event.accept()
            return
        super().mouseReleaseEvent(event)


class CamRecordArch(QWidget):
    """Legacy arched record mount — prefer ``CamRecordTimerRow`` for compact rail."""

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


class CamRecordTimerRow(QWidget):
    """Shutter + elapsed timer on one row (saves vertical space vs stacked arch + timer)."""

    def __init__(self, record_btn: QWidget, timer_lbl: QWidget, parent=None) -> None:
        super().__init__(parent)
        self.setObjectName("camRecordTimerRow")
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 2, 0, 2)
        lay.setSpacing(8)
        lay.addStretch(1)
        lay.addWidget(record_btn, 0, Qt.AlignmentFlag.AlignVCenter)
        lay.addWidget(timer_lbl, 0, Qt.AlignmentFlag.AlignVCenter)
        lay.addStretch(1)


class CamRailGimbalPad(QWidget):
    """2×3 gimbal pad: [← ↑ →] / [⌂ ↓ 90°]."""

    def __init__(
        self,
        buttons: list[list[QWidget]],
        parent=None,
        *,
        btn_height: int = 30,
        btn_width: int = 40,
        grid_gap: int = 4,
    ) -> None:
        super().__init__(parent)
        from PySide6.QtWidgets import QGridLayout

        self.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        lay = QGridLayout(self)
        # Right padding keeps 1px borders inside the widget (avoids → / 90° clip).
        lay.setContentsMargins(0, 0, 4, 2)
        lay.setHorizontalSpacing(int(grid_gap))
        lay.setVerticalSpacing(int(grid_gap))
        cols = max((len(row) for row in buttons), default=0)
        rows = len(buttons)
        for row, row_btns in enumerate(buttons):
            for col, btn in enumerate(row_btns):
                lay.addWidget(btn, row, col, Qt.AlignmentFlag.AlignCenter)
        lay.activate()
        # Explicit size from button geometry + padding — layout minimumSize can be too small.
        pad_w = (
            int(cols) * int(btn_width)
            + max(0, int(cols) - 1) * int(grid_gap)
            + lay.contentsMargins().left()
            + lay.contentsMargins().right()
        )
        pad_h = (
            int(rows) * int(btn_height)
            + max(0, int(rows) - 1) * int(grid_gap)
            + lay.contentsMargins().top()
            + lay.contentsMargins().bottom()
        )
        sh = lay.sizeHint()
        self.setFixedSize(
            max(pad_w, sh.width()),
            max(pad_h, sh.height()),
        )


class CamObserveBlock(QWidget):
    """DOOAF observe: Target / Clip / Report / Reset + Setup."""

    setup_clicked = Signal()

    def __init__(
        self,
        target_btn: QWidget,
        clip_btn: QWidget,
        report_btn: QWidget,
        reset_btn: QWidget,
        parent=None,
    ) -> None:
        super().__init__(parent)
        v = QVBoxLayout(self)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(4)
        target_btn.setObjectName("observeTarget")
        clip_btn.setObjectName("observeClip")
        row1 = QHBoxLayout()
        row1.setContentsMargins(0, 0, 0, 0)
        row1.setSpacing(4)
        row1.addWidget(target_btn, 1)
        row1.addWidget(clip_btn, 1)
        row2 = QHBoxLayout()
        row2.setContentsMargins(0, 0, 0, 0)
        row2.setSpacing(4)
        row2.addWidget(report_btn, 1)
        row2.addWidget(reset_btn, 1)
        self.setup_btn = QPushButton("DOOAF Setup")
        self.setup_btn.setObjectName("observeDooafSetup")
        self.setup_btn.setToolTip(
            "Enter artillery position and actual target lat/lon from military grid."
        )
        self.setup_btn.clicked.connect(self.setup_clicked.emit)
        row_setup = QHBoxLayout()
        row_setup.setContentsMargins(0, 0, 0, 0)
        row_setup.setSpacing(4)
        row_setup.addWidget(self.setup_btn, 1)
        hint = QLabel("DOOAF Setup: Pick on video for target · Target ON = Impact Target (red)")
        hint.setObjectName("observeDooafHint")
        hint.setWordWrap(True)
        hint.setToolTip(
            "Set gun and actual target in DOOAF Setup (Pick on map or Pick on video). "
            "Then turn Target ON and click Impact Target on the video feed."
        )
        v.addLayout(row1)
        v.addLayout(row2)
        v.addLayout(row_setup)
        v.addWidget(hint)

    def current_dooaf_role(self) -> str:
        return DOOAF_ROLE_IMPACT


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

        groove = outer.adjusted(6.0, 6.0, -6.0, -6.0)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor(10, 12, 20, 252))
        p.drawRoundedRect(groove, 4.0, 4.0)

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
