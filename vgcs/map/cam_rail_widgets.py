"""
Native Qt widgets for map camera rail — git `#cameraRail` core + reference slider/segment visuals.

Git `e48c1a7` defines `#cameraRail`, `#cameraTopRow`, `#camRecordBtn`, `#camTimer`, `#camSettingsBtn` only;
zoom/focus/gimbal/observe strips are native MAVLink controls styled to match product reference.
"""

from __future__ import annotations

from PySide6.QtCore import QPointF, QRectF, QSize, Qt, Signal
from PySide6.QtGui import QColor, QFont, QIcon, QPainter, QPen
from PySide6.QtWidgets import (
    QButtonGroup,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from vgcs.app import ui_theme
from vgcs.observe.dooaf import DOOAF_ROLE_IMPACT, DOOAF_ROLE_INTENDED


class CamRailShowHandle(QFrame):
    """Edge tab — shown when the operator hides the camera panels; a click shows them again."""

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
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet(
            "QFrame#camRailShowHandle {"
            f" background-color: {ui_theme.OVERLAY_CARD}; border: 1px solid {ui_theme.OVERLAY_BORDER};"
            " border-right: none; border-top-left-radius: 16px; border-bottom-left-radius: 16px;"
            "}"
            "QFrame#camRailShowHandle:hover {"
            f" background-color: {ui_theme.OVERLAY_BUTTON_HOVER}; border-color: {ui_theme.OVERLAY_BORDER_HOVER};"
            "}"
            "QLabel { background: transparent; border: none; }"
        )
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 14, 0, 12)
        lay.setSpacing(10)
        lay.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        icon_lbl = QLabel()
        icon_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        icon_lbl.setFixedSize(22, 22)
        if icon is not None and not icon.isNull():
            icon_lbl.setPixmap(icon.pixmap(22, 22))
        chev = QLabel()
        chev.setObjectName("camRailShowChevron")
        chev.setAlignment(Qt.AlignmentFlag.AlignCenter)
        chev.setFixedSize(18, 18)
        chevron = ui_theme.icon("arrow_left", 18)
        if not chevron.isNull():
            chev.setPixmap(chevron.pixmap(18, 18))
        lay.addWidget(icon_lbl, 0, Qt.AlignmentFlag.AlignHCenter)
        lay.addWidget(chev, 0, Qt.AlignmentFlag.AlignHCenter)
        lay.addStretch(1)

    def mouseReleaseEvent(self, event) -> None:  # type: ignore[override]
        if event.button() == Qt.MouseButton.LeftButton:
            self.clicked.emit()
            event.accept()
            return
        super().mouseReleaseEvent(event)


class CamObserveBlock(QWidget):
    """DOOAF observe: Target / Clip / Report / Reset + Setup."""

    setup_clicked = Signal()
    dooaf_role_changed = Signal(str)
    laser_changed = Signal(str, bool)  # the role, and whether its clicks go to the laser

    # The switch under Set TGT and Set HIT: (off, on) for the role that is chosen.
    LASER_TEXT = {
        DOOAF_ROLE_INTENDED: ("Laser TGT: OFF", "Laser TGT: ON"),
        DOOAF_ROLE_IMPACT: ("Laser HIT: OFF", "Laser HIT: ON"),
    }
    LASER_TOOLTIP = {
        DOOAF_ROLE_INTENDED: (
            "ON: put the cross on the target, then click it. "
            "The laser measures that point.\n"
            "OFF: the target is placed from the picture."
        ),
        DOOAF_ROLE_IMPACT: (
            "ON: put the cross on the fall of shot, then click it. "
            "The laser measures that point.\n"
            "OFF: the fall of shot is placed from the picture."
        ),
    }

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

        # What a click on the video sets. It used to be the fall of shot and
        # nothing else, so the actual target could only be placed through the
        # DOOAF Setup dialog: two different ways to mark two points that the
        # operator thinks of as the same action. Requested 2026-09-09 after the
        # crew described a competitor setting both from the video.
        row_role = QHBoxLayout()
        row_role.setContentsMargins(0, 0, 0, 0)
        row_role.setSpacing(4)
        self.role_target_btn = QPushButton("Set TGT")
        self.role_target_btn.setObjectName("observeRoleTarget")
        self.role_target_btn.setCheckable(True)
        self.role_target_btn.setToolTip("A video or map click sets the actual target")
        self.role_impact_btn = QPushButton("Set HIT")
        self.role_impact_btn.setObjectName("observeRoleImpact")
        self.role_impact_btn.setCheckable(True)
        self.role_impact_btn.setChecked(True)
        self.role_impact_btn.setToolTip("A video or map click sets the fall of shot")
        self._role_group = QButtonGroup(self)
        self._role_group.setExclusive(True)
        self._role_group.addButton(self.role_target_btn)
        self._role_group.addButton(self.role_impact_btn)
        self.role_target_btn.toggled.connect(self._on_role_toggled)
        self.role_impact_btn.toggled.connect(self._on_role_toggled)
        row_role.addWidget(self.role_target_btn, 1)
        row_role.addWidget(self.role_impact_btn, 1)

        # How a click of the chosen role is measured. OFF: from the picture.
        # ON: the laser measures the point under the cross. One button for the
        # two roles, each with its own state: it shows the one of the role
        # that is chosen, "Laser TGT" or "Laser HIT", so it always says what
        # the next click does. Both start OFF: the crew asked on 2026-09-11
        # for the fall of shot without the laser, and Set TGT has always been
        # placed from the picture.
        #
        # Until 2026-10-08 the laser for the fall of shot was a setting with
        # no place on the screen, so a point that the picture cannot measure
        # (a drone on the ground, a look flatter than 8 degrees) could not be
        # marked on the video at all. The text says the state, because a
        # pressed button alone is easy to misread in the field.
        self._laser_on = {DOOAF_ROLE_INTENDED: False, DOOAF_ROLE_IMPACT: False}
        self.laser_btn = QPushButton(self.LASER_TEXT[DOOAF_ROLE_IMPACT][0])
        self.laser_btn.setObjectName("observeLaser")
        self.laser_btn.setCheckable(True)
        self.laser_btn.setToolTip(self.LASER_TOOLTIP[DOOAF_ROLE_IMPACT])
        self.laser_btn.toggled.connect(self._on_laser_toggled)
        row_laser = QHBoxLayout()
        row_laser.setContentsMargins(0, 0, 0, 0)
        row_laser.setSpacing(4)
        row_laser.addWidget(self.laser_btn, 1)

        # One line: the row of the laser switch above took the height of two,
        # and on a screen 720 pixels high the rail has none to spare. The rest
        # is in the tooltips.
        hint = QLabel("Target ON, then click the video.")
        hint.setObjectName("observeDooafHint")
        hint.setWordWrap(True)
        hint.setToolTip(
            "Turn Target ON, pick Set TGT or Set HIT, then click the point on the "
            "video feed or the map. DOOAF Setup still takes typed grid references.\n"
            "Laser TGT or Laser HIT: ON measures the point with the laser, under the cross."
        )
        v.addLayout(row1)
        v.addLayout(row2)
        v.addLayout(row_role)
        v.addLayout(row_laser)
        v.addLayout(row_setup)
        v.addWidget(hint)

    def _on_role_toggled(self, _checked: bool) -> None:
        self._show_laser_of_the_role()
        self.dooaf_role_changed.emit(self.current_dooaf_role())

    def _on_laser_toggled(self, checked: bool) -> None:
        role = self.current_dooaf_role()
        self._laser_on[role] = bool(checked)
        self._show_laser_of_the_role()
        self.laser_changed.emit(role, bool(checked))

    def _show_laser_of_the_role(self) -> None:
        """The switch shows the state of the role that is chosen. Nobody is told."""
        role = self.current_dooaf_role()
        on = bool(self._laser_on.get(role, False))
        self.laser_btn.blockSignals(True)
        try:
            self.laser_btn.setChecked(on)
        finally:
            self.laser_btn.blockSignals(False)
        self.laser_btn.setText(self.LASER_TEXT[role][1 if on else 0])
        self.laser_btn.setToolTip(self.LASER_TOOLTIP[role])

    def set_laser_for(self, role: str, on: bool) -> None:
        """A state that was read from the settings: shown, and nobody is told."""
        if str(role) not in self._laser_on:
            return
        self._laser_on[str(role)] = bool(on)
        self._show_laser_of_the_role()

    def laser_for(self, role: str) -> bool:
        return bool(self._laser_on.get(str(role), False))

    def set_dooaf_role(self, role: str) -> None:
        want_target = str(role) == DOOAF_ROLE_INTENDED
        btn = self.role_target_btn if want_target else self.role_impact_btn
        if not btn.isChecked():
            btn.setChecked(True)

    def current_dooaf_role(self) -> str:
        return (
            DOOAF_ROLE_INTENDED
            if self.role_target_btn.isChecked()
            else DOOAF_ROLE_IMPACT
        )


class CamM13TrackBlock(QWidget):
    """M13 moving-target track and Follow Me: two named pills in the camera bar.

    Track arms the visual track (click the target on the video, the gimbal follows it). Follow makes
    the DRONE follow the tracked object, so it keeps its name rather than an icon alone. The words of
    the old hint line are in Track's tooltip.
    """

    def __init__(self, track_btn: QWidget, follow_btn: QWidget | None = None, parent=None) -> None:
        super().__init__(parent)
        self.setObjectName("m13TrackBlock")
        v = QVBoxLayout(self)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(6)
        track_btn.setParent(self)
        track_btn.setObjectName("m13Track")
        track_btn.setToolTip(
            "Arm visual track — click a target on the day video feed. "
            "C13 firmware follows via GOT+SUM; map coords computed in GCS.\n"
            "Track ON, then click the target on the video: the gimbal follows it and the map draws its trail."
        )
        _pill(track_btn, "vama_lock")
        v.addWidget(track_btn)
        if follow_btn is not None:
            # Follow Me under Track, in the same block: shown and hidden with it.
            follow_btn.setParent(self)
            _pill(follow_btn, "vama_follow")
            v.addWidget(follow_btn)


def _pill(btn: QWidget, icon_name: str) -> None:
    """A named pill of the camera panels: icon, name, the panels' sheet gives its look."""
    btn.setIcon(ui_theme.icon(icon_name, 16))
    btn.setIconSize(QSize(16, 16))
    btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
    btn.setCursor(Qt.CursorShape.PointingHandCursor)


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
