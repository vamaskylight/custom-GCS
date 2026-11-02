"""The round proximity view, as Mission Planner shows it, in a window of its own (B25, the client 2026-10-31).

Nish: "Just like the mission planner proximity", "this proximity into hidden button so when we click on button
then we can see the proximity and make this proximity dialogue box in adjustable dialogue box", and of B19's four
bars: "the round view replace them". The PROXIMITY card opens it with its "Proximity view" button.

The drone in the middle, its nose up: the readings are the drone's own directions, so the top is ahead, not north.
Rings every tenth of the range (0.5 m at the 5 m it starts with, as in Mission Planner's picture), the degrees every
45, and an object as a red arc across its direction (45 degrees wide) at its distance, with the distance. Only inside
the range, chosen on the window's button (vgcs.app.obstacle_watch BAR_RANGE_CHOICES_M, saved by the main window).
OBSTACLE_DISTANCE from a companion computer is drawn the same way, a short arc for each of its bins. The window is
the user's to place and to size (both kept for the next time), and the view is drawn only when a reading changes.
"""

from __future__ import annotations

import math

from PySide6.QtCore import QPointF, QRectF, QSettings, Qt, Signal
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import QDialog, QHBoxLayout, QLabel, QMenu, QPushButton, QSizePolicy, QVBoxLayout, QWidget

from vgcs.app.obstacle_watch import BAR_RANGE_CHOICES_M, DEFAULT_BAR_RANGE_M, metres_words

GEOMETRY_SETTING = "proximity/window_geometry"

BACKGROUND = QColor(37, 39, 43)
GRID = QColor(200, 200, 200, 110)
GRID_TEXT = QColor(170, 174, 180)
RING_TEXT = QColor(40, 170, 70)  # Mission Planner's green ring labels
OBJECT_RED = QColor(240, 32, 32)  # the app's obstacle red (#f02020)
TEXT = QColor(235, 238, 244)
DRONE = QColor(170, 205, 50)

_LABEL_MARGIN_PX = 24
_ARC_WIDTH_PX = 5.0
_DIRECTION_HALF_DEG = 22.5 - 3.0  # an 8-direction arc, with a small gap to the next one
_RINGS = 10


class ProximityRoundView(QWidget):
    """The polar plot: rings, degrees, the drone, and the objects within the range."""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setObjectName("proximityRoundView")
        self.setMinimumSize(200, 200)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._view: dict | None = None
        self._bins: tuple | None = None  # (distances in cm, increment, offset, max m) of OBSTACLE_DISTANCE
        self._range_m = DEFAULT_BAR_RANGE_M

    # -- what it shows -------------------------------------------------------------------
    def set_view(self, view: dict | None) -> None:
        """The drone's proximity sensor (vgcs.app.obstacle_watch ``radar_view``), or None. Drawn again only when it changed."""
        new = dict(view) if isinstance(view, dict) else None
        if new == self._view:
            return
        self._view = new
        if new is not None and new.get("bar_range_m"):
            self._range_m = float(new["bar_range_m"])
        self.update()

    def set_obstacle_bins(self, distances_cm, increment_deg: float, offset_deg: float, max_m: float) -> None:
        """OBSTACLE_DISTANCE from a companion computer, when no 8-direction view comes."""
        new = (tuple(int(d) for d in distances_cm), float(increment_deg or 5.0), float(offset_deg or 0.0), float(max_m or 0.0))
        if new == self._bins:
            return
        self._bins = new
        self.update()

    def clear(self) -> None:
        if self._view is None and self._bins is None:
            return
        self._view = None
        self._bins = None
        self.update()

    def range_m(self) -> float:
        return self._range_m

    def view(self) -> dict | None:
        return self._view

    # -- where things go -------------------------------------------------------------------
    def geometry_px(self) -> tuple[float, float, float]:
        """The centre and the radius of the range's ring, in the widget."""
        w, h = float(self.width()), float(self.height())
        return w / 2.0, h / 2.0, max(10.0, min(w, h) / 2.0 - _LABEL_MARGIN_PX)

    def point_px(self, direction_deg: float, metres: float) -> QPointF:
        """Where an object in this direction (degrees from the nose, to the right) at this distance is drawn."""
        cx, cy, radius = self.geometry_px()
        r = radius * float(metres) / self._range_m
        rad = math.radians(float(direction_deg) - 90.0)
        return QPointF(cx + r * math.cos(rad), cy + r * math.sin(rad))

    def objects(self) -> list[tuple[float, float, float]]:
        """(direction in degrees, half width in degrees, metres) of each object within the range."""
        out: list[tuple[float, float, float]] = []
        view = self._view
        if view is not None:
            for index, metres in enumerate(list(view.get("directions_m") or [])[:8]):
                if metres is not None and float(metres) <= self._range_m:
                    out.append((index * 45.0, _DIRECTION_HALF_DEG, float(metres)))
            return out
        if self._bins is not None:
            distances, increment, offset, _max_m = self._bins
            for index, cm in enumerate(distances):
                if 0 < cm < 0xFFFF and cm / 100.0 <= self._range_m:
                    out.append(((offset + index * increment) % 360.0, increment * 0.45, cm / 100.0))
        return out

    # -- drawing ------------------------------------------------------------------------
    def paintEvent(self, event) -> None:  # noqa: N802
        del event
        p = QPainter(self)
        if not p.isActive():
            return
        try:
            self.paint(p)
        finally:
            p.end()

    def paint(self, p: QPainter) -> None:
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(QRectF(0, 0, self.width(), self.height()), BACKGROUND)
        cx, cy, radius = self.geometry_px()
        font_px = max(9, min(13, int(radius / 16)))
        font = QFont("Segoe UI")
        font.setPixelSize(font_px)
        p.setFont(font)

        # Rings every tenth of the range, and the degrees every 45.
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.setPen(QPen(GRID, 1.0))
        for i in range(1, _RINGS + 1):
            r = radius * i / _RINGS
            p.drawEllipse(QPointF(cx, cy), r, r)
        for k in range(8):
            rad = math.radians(k * 45.0 - 90.0)
            p.setPen(QPen(GRID, 1.0))
            p.drawLine(QPointF(cx, cy), QPointF(cx + radius * math.cos(rad), cy + radius * math.sin(rad)))
            lx = cx + (radius + font_px) * math.cos(rad)
            ly = cy + (radius + font_px) * math.sin(rad)
            p.setPen(GRID_TEXT)
            p.drawText(QRectF(lx - 20, ly - font_px, 40, 2 * font_px), int(Qt.AlignmentFlag.AlignCenter), f"{k * 45}")
        # The rings' distances on the line ahead, every ring when there is room, else every second one.
        p.setPen(RING_TEXT)
        every = 1 if radius / _RINGS >= font_px * 1.5 else 2
        for i in range(every, _RINGS + 1, every):
            r = radius * i / _RINGS
            p.drawText(QRectF(cx + 4, cy - r - 1, 60, font_px + 3), int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop),
                       f"{self._range_m * i / _RINGS:.1f}m")

        self._paint_drone(p, cx, cy, radius)

        # The objects: a red arc across each one's direction, at its distance.
        objects = self.objects()
        for direction, half, metres in objects:
            r = max(2.0, radius * metres / self._range_m)
            p.setPen(QPen(OBJECT_RED, _ARC_WIDTH_PX, Qt.PenStyle.SolidLine, Qt.PenCapStyle.FlatCap))
            p.setBrush(Qt.BrushStyle.NoBrush)
            # Qt counts from 3 o'clock, anticlockwise, in 16ths of a degree.
            start = 90.0 - (direction + half)
            p.drawArc(QRectF(cx - r, cy - r, 2 * r, 2 * r), int(start * 16), int(2 * half * 16))
        if self._view is not None:
            for direction, _half, metres in objects:
                rad = math.radians(direction - 90.0)
                r = radius * metres / self._range_m + font_px + 4
                tx, ty = cx + r * math.cos(rad), cy + r * math.sin(rad)
                if abs(direction) < 1.0:
                    tx -= 34  # straight ahead: left of the line, where the rings' distances are not
                p.setPen(TEXT)
                p.drawText(QRectF(tx - 32, ty - font_px, 64, 2 * font_px), int(Qt.AlignmentFlag.AlignCenter),
                           metres_words(metres))
            up = self._view.get("up_m")
            if up is not None:
                p.setPen(OBJECT_RED if float(up) <= float(self._view.get("red_m") or 0.0) else TEXT)
                p.drawText(QRectF(6, 4, 120, font_px + 4), int(Qt.AlignmentFlag.AlignLeft), f"UP {float(up):.1f} m")
        elif self._bins is None:
            p.setPen(GRID_TEXT)
            p.drawText(QRectF(cx - radius, cy + radius * 0.35, 2 * radius, font_px * 2), int(Qt.AlignmentFlag.AlignCenter),
                       "Waiting for the proximity sensor")

    @staticmethod
    def _paint_drone(p: QPainter, cx: float, cy: float, radius: float) -> None:
        """A small quad in the middle, its nose up, inside the first ring as Mission Planner draws it."""
        s = max(4.0, radius * 0.035)
        p.setPen(QPen(DRONE, max(1.5, s * 0.22)))
        p.setBrush(Qt.BrushStyle.NoBrush)
        for dx, dy in ((0.0, -1.0), (1.0, 0.0), (0.0, 1.0), (-1.0, 0.0)):
            p.drawLine(QPointF(cx, cy), QPointF(cx + dx * s, cy + dy * s))
            p.drawEllipse(QPointF(cx + dx * s * 1.4, cy + dy * s * 1.4), s * 0.55, s * 0.55)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(DRONE)
        p.drawEllipse(QPointF(cx, cy - s * 1.4), s * 0.22, s * 0.22)  # the nose


class ProximityWindow(QDialog):
    """The round view in a window of its own: not modal, sized and placed by the user (kept for the next time),
    with the range's list and the status underneath."""

    bar_range_chosen = Signal(float)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setObjectName("proximityWindow")
        self.setWindowTitle("Proximity")
        self.setModal(False)
        self.setSizeGripEnabled(True)
        self.setMinimumSize(260, 300)
        self.resize(460, 500)
        self.setStyleSheet(
            "QDialog#proximityWindow { background: #25272b; }"
            "QLabel { color: #d6dae2; background: transparent; }"
            "QPushButton { color: #e6eaf2; background: #343841; border: 1px solid #4a505c; border-radius: 4px;"
            " padding: 4px 10px; }"
            "QPushButton:hover { background: #3d424c; }"
        )
        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setSpacing(6)
        self._view = ProximityRoundView(self)
        lay.addWidget(self._view, 1)

        row = QHBoxLayout()
        row.setSpacing(8)
        self._range_button = QPushButton(self)
        self._range_button.setObjectName("proximityRangeBtn")
        self._range_button.setToolTip("How far the view reaches: an object farther away is not drawn")
        menu = QMenu(self._range_button)
        for metres in BAR_RANGE_CHOICES_M:
            action = menu.addAction(f"{metres:g} m")
            action.setCheckable(True)
            action.setData(metres)
            action.triggered.connect(lambda _checked=False, chosen=metres: self.bar_range_chosen.emit(chosen))
        self._range_button.setMenu(menu)
        self._status = QLabel(self)
        self._status.setObjectName("proximityStatus")
        close = QPushButton("Close", self)
        close.setObjectName("proximityCloseBtn")
        close.clicked.connect(self.close)
        row.addWidget(self._range_button)
        row.addWidget(self._status, 1)
        row.addWidget(close)
        lay.addLayout(row)
        self._geometry_restored = False
        self._show_range(self._view.range_m())

    def round_view(self) -> ProximityRoundView:
        return self._view

    def set_view(self, view: dict | None) -> None:
        self._view.set_view(view)
        if isinstance(view, dict):
            self._status.setText(str(view.get("status") or ""))
            self._show_range(self._view.range_m())
        else:
            self._status.setText("Waiting for the proximity sensor")

    def set_obstacle_bins(self, distances_cm, increment_deg: float, offset_deg: float, max_m: float) -> None:
        self._view.set_obstacle_bins(distances_cm, increment_deg, offset_deg, max_m)

    def clear(self) -> None:
        self._view.clear()
        self._status.setText("Waiting for the proximity sensor")

    def _show_range(self, metres: float) -> None:
        self._range_button.setText(f"Within {metres:g} m")  # the button draws its own arrow
        for action in self._range_button.menu().actions():
            action.setChecked(action.data() == metres)

    # -- its place and size, kept for the next time ----------------------------------------
    def showEvent(self, event) -> None:  # noqa: N802
        if not self._geometry_restored:
            self._geometry_restored = True
            saved = QSettings("VGCS", "VGCS").value(GEOMETRY_SETTING)
            if saved:
                try:
                    self.restoreGeometry(saved)
                except Exception:
                    pass
        super().showEvent(event)

    def hideEvent(self, event) -> None:  # noqa: N802
        try:
            QSettings("VGCS", "VGCS").setValue(GEOMETRY_SETTING, self.saveGeometry())
        except Exception:
            pass
        super().hideEvent(event)


__all__ = ["GEOMETRY_SETTING", "ProximityRoundView", "ProximityWindow"]
