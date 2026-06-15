"""
Bottom-right map HUD matching legacy Web `#mapFooterHud` / `#compass` / `#telemetryStrip`
(git e48c1a7 map_widget.py embedded HTML/CSS).
"""

from __future__ import annotations

import math

from PySide6.QtCore import QPoint, QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QFont, QIcon, QPainter, QPainterPath, QPen, QPixmap, QPolygonF
from PySide6.QtWidgets import (
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QVBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QWidget,
)

# Shared with native `#cameraRail` QSS in map_widget.py — keep in sync for HUD typography.
TELEMETRY_STRIP_VALUE_STYLE = "color: #dce5f5; font-size: 15px; font-weight: 600;"


class TelemetryStripIcon(QWidget):
    """Minimal line-art icons for the telemetry strip (and shared map action rail)."""

    _COLOR = QColor(220, 229, 245)

    def __init__(self, kind: str, parent=None, *, icon_size: int | None = None) -> None:
        super().__init__(parent)
        self._kind = kind
        sz = 20 if icon_size is None else max(16, int(icon_size))
        self.setFixedSize(sz, sz)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)
        self.setAutoFillBackground(False)

    def paintEvent(self, event) -> None:  # noqa: N802
        del event
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        stroke = max(1.35, 1.75 * min(w, h) / 20.0)
        pen = QPen(self._COLOR, stroke, Qt.PenStyle.SolidLine)
        pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
        p.setPen(pen)
        p.setBrush(Qt.BrushStyle.NoBrush)
        cx = w / 2.0
        cy = h / 2.0

        k = self._kind
        if k == "alt":
            m = 2.5
            y_top, y_bot = m + 1.5, h - m - 1.5
            shaft_t, shaft_b = y_top + 5.0, y_bot - 5.0
            p.drawLine(QPointF(cx, shaft_t), QPointF(cx, shaft_b))
            aw, ah = 4.2, 4.8
            p.drawLine(QPointF(cx, y_top), QPointF(cx - aw, y_top + ah))
            p.drawLine(QPointF(cx, y_top), QPointF(cx + aw, y_top + ah))
            p.drawLine(QPointF(cx, y_bot), QPointF(cx - aw, y_bot - ah))
            p.drawLine(QPointF(cx, y_bot), QPointF(cx + aw, y_bot - ah))
        elif k == "up":
            tip_y = 3.5
            stem_bot = h - 3.0
            p.drawLine(QPointF(cx, stem_bot), QPointF(cx, tip_y + 4.0))
            p.drawLine(QPointF(cx, tip_y), QPointF(cx - 4.6, tip_y + 5.2))
            p.drawLine(QPointF(cx, tip_y), QPointF(cx + 4.6, tip_y + 5.2))
        elif k == "timer":
            rr = min(w, h) * 0.36
            p.drawEllipse(QRectF(cx - rr, cy - rr + 0.5, 2 * rr, 2 * rr))
            p.drawLine(QPointF(cx, cy), QPointF(cx + rr * 0.45, cy - rr * 0.35))
            p.drawLine(QPointF(cx, cy), QPointF(cx - rr * 0.25, cy + rr * 0.4))
            kn = QRectF(cx - 1.8, 1.2, 3.6, 2.8)
            p.setBrush(self._COLOR)
            p.drawRoundedRect(kn, 1.0, 1.0)
            p.setBrush(Qt.BrushStyle.NoBrush)
        elif k == "person":
            p.drawEllipse(QRectF(cx - 4.0, 3.0, 8.0, 8.0))
            path = QPainterPath()
            path.moveTo(cx - 7.0, h - 2.5)
            path.quadTo(cx, cy + 2.5, cx + 7.0, h - 2.5)
            p.drawPath(path)
        elif k == "right":
            x0, x1 = 2.5, w - 2.5
            p.drawLine(QPointF(x0, cy), QPointF(x1 - 4.0, cy))
            p.drawLine(QPointF(x1, cy), QPointF(x1 - 5.2, cy - 3.8))
            p.drawLine(QPointF(x1, cy), QPointF(x1 - 5.2, cy + 3.8))
        elif k == "return_home":
            # RTL / return: stem down, bar left, chevron (same stroke language as "right", mirrored).
            m = max(2.0, min(w, h) * 0.12)
            x_r = w - m
            y_t = m + 1.0
            y_b = h - m - 1.5
            x_l = m + 1.5
            p.drawLine(QPointF(x_r, y_t), QPointF(x_r, y_b))
            p.drawLine(QPointF(x_r, y_b), QPointF(x_l, y_b))
            aw, ah = 4.2, 3.8
            p.drawLine(QPointF(x_l, y_b), QPointF(x_l + aw, y_b - ah))
            p.drawLine(QPointF(x_l, y_b), QPointF(x_l + aw, y_b + ah))
        elif k == "corner":
            path = QPainterPath()
            path.moveTo(w - 2.5, 3.5)
            path.lineTo(w - 2.5, h - 5.0)
            path.lineTo(5.0, h - 5.0)
            p.drawPath(path)
            p.drawLine(QPointF(5.0, h - 5.0), QPointF(10.0, h - 9.0))


class MapFooterCompass(QWidget):
    """176×176 compass card: inner disk, N/E/S/W, red needle, degree readout (Web parity).

    ``_map_bearing_deg`` rotates the cardinals/ticks with the map camera (3D) so N stays aligned
    with geographic north on screen; the needle uses (vehicle heading − map bearing).
    """

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._heading_deg = 0.0
        self._map_bearing_deg = 0.0
        self.setFixedSize(176, 176)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, False)

    def set_heading_deg(self, deg: float) -> None:
        self._heading_deg = float(deg) % 360.0
        self.update()

    def set_map_bearing_deg(self, deg: float) -> None:
        """Clockwise yaw of the map view from north (matches Cesium camera.heading in degrees)."""
        self._map_bearing_deg = float(deg) % 360.0
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w = self.width()
        h = self.height()
        cx = w / 2.0
        cy = h / 2.0
        outer_r = min(w, h) / 2.0 - 2.0
        inner_r = 76.0

        painter.setPen(QPen(QColor(255, 255, 255, 40), 1))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawEllipse(QPointF(cx, cy), outer_r, outer_r)

        painter.setPen(QPen(QColor(148, 160, 180, int(0.32 * 255)), 2))
        painter.setBrush(QColor(22, 26, 34, int(0.96 * 255)))
        painter.drawEllipse(QPointF(cx, cy), inner_r, inner_r)

        painter.setPen(QPen(QColor(240, 245, 255, int(0.55 * 255)), 1))
        mb = self._map_bearing_deg
        for tick_deg in range(0, 360, 30):
            rad = math.radians(tick_deg - mb)
            x1 = cx + (inner_r - 6) * math.sin(rad)
            y1 = cy - (inner_r - 6) * math.cos(rad)
            x2 = cx + (inner_r - 18) * math.sin(rad)
            y2 = cy - (inner_r - 18) * math.cos(rad)
            painter.drawLine(QPointF(x1, y1), QPointF(x2, y2))

        font = QFont("Segoe UI", 11)
        font.setBold(True)
        painter.setFont(font)
        painter.setPen(QColor("#f1f5ff"))
        # N/E/W/S on the ring (S was fixed bottom in legacy web; here it rotates with map bearing for 3D parity).
        for text, bearing in (("N", 0), ("E", 90), ("W", 270)):
            rad = math.radians(bearing - mb)
            tx = cx + (inner_r - 22) * math.sin(rad) - (6 if text == "N" else 5)
            ty = cy - (inner_r - 22) * math.cos(rad) + 5
            painter.drawText(QPointF(tx, ty), text)

        painter.translate(cx, cy)
        painter.rotate(self._heading_deg - mb)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor(255, 91, 78))
        # Git `#needle::before`: clip-path ~48px tall inside 152px inner — short kite, not full radius.
        tip_y = -0.33 * inner_r
        wing_x = 0.14 * inner_r
        wing_y = 0.23 * inner_r
        tail_y = 0.15 * inner_r  # slight inward notch (wing_y > tail_y in +Y-down coords)
        tip = QPolygonF(
            [
                QPointF(0, tip_y),
                QPointF(wing_x, wing_y),
                QPointF(0, tail_y),
                QPointF(-wing_x, wing_y),
            ]
        )
        painter.drawPolygon(tip)
        painter.setPen(QPen(QColor(244, 248, 255, int(0.92 * 255)), 2))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawPolygon(tip)
        painter.resetTransform()

        # Center pivot (git needle axis).
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor("#f4f7ff"))
        painter.drawEllipse(QPointF(cx, cy), 3.5, 3.5)

        # Git `#compassInner` (152px) layout: `#compassDeg { bottom: 30px }`, `#cS { bottom: 12px }`
        # → degree band sits above the "S" label (no overlap).
        inner_bottom = cy + inner_r
        deg_font = QFont("Segoe UI", 13)
        deg_font.setBold(True)
        painter.setFont(deg_font)
        painter.setPen(QColor("#f4f7ff"))
        deg_h = 22.0
        deg_rect = QRectF(0.0, inner_bottom - 30.0 - deg_h, float(w), deg_h)
        painter.drawText(
            deg_rect,
            Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignBottom,
            f"{self._heading_deg:.0f}°",
        )

        painter.setFont(font)
        painter.setPen(QColor("#f1f5ff"))
        rad_s = math.radians(180.0 - mb)
        stx = cx + (inner_r - 22) * math.sin(rad_s) - 5
        sty = cy - (inner_r - 22) * math.cos(rad_s) + 5
        painter.drawText(QPointF(stx, sty), "S")


class MapFooterTelemetryStrip(QFrame):
    """
    2×3 grid matching git `#telemetryStrip` (`setTelemetryOverlay` updates six spans).

    Row 1: ↕ rel alt (AGL) | ↑ vertical spd | ⏱ time
    Row 2: 👤 distance from home | → ground spd | ↳ rel alt
    """

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setObjectName("mapTelemetryStrip")
        self.setStyleSheet(
            "QFrame#mapTelemetryStrip {"
            " background: rgba(26, 33, 45, 215);"
            " border: 1px solid rgba(80, 92, 118, 107);"
            " border-radius: 12px;"
            "}"
        )
        grid = QGridLayout(self)
        grid.setContentsMargins(10, 6, 10, 6)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(6)

        val_style = TELEMETRY_STRIP_VALUE_STYLE

        # Return (cell wrapper, value label). Vector icons (`TelemetryStripIcon`) stay sharp vs Unicode.
        def _cell(kind: str) -> tuple[QWidget, QLabel]:
            wrap = QWidget()
            wrap.setSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)
            ic = TelemetryStripIcon(kind, wrap)
            tx = QLabel("", wrap)
            tx.setStyleSheet(val_style)
            tx.setWordWrap(False)
            tx.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
            tx.setSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)
            hl = QHBoxLayout(wrap)
            hl.setContentsMargins(2, 2, 2, 2)
            hl.setSpacing(8)
            hl.addWidget(ic, 0, Qt.AlignmentFlag.AlignVCenter)
            # Stretch so the label uses the full cell width when the grid is wider than the text minimum.
            hl.addWidget(tx, 1, Qt.AlignmentFlag.AlignVCenter)
            return wrap, tx

        w00, self._row1_alt = _cell("alt")
        w01, self._row1_mph = _cell("up")
        w02, self._row1_time = _cell("timer")
        w10, self._row2_msl = _cell("person")
        w11, self._row2_mph = _cell("right")
        w12, self._row2_alt = _cell("corner")

        grid.addWidget(w00, 0, 0)
        grid.addWidget(w01, 0, 1)
        grid.addWidget(w02, 0, 2)
        grid.addWidget(w10, 1, 0)
        grid.addWidget(w11, 1, 1)
        grid.addWidget(w12, 1, 2)
        for c in range(3):
            grid.setColumnStretch(c, 1)

        self._row1_alt.setText("0.0 m")
        self._row1_mph.setText("0.0 m/s")
        self._row1_time.setText("00:00:00")
        self._row2_msl.setText("0.0 m")
        self._row2_mph.setText("0.0 m/s")
        self._row2_alt.setText("0.0 m")

        self.setSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Minimum)

    def set_values(
        self,
        row1_alt_ft: str,
        row1_vspeed_mph: str,
        time_text: str,
        distance_home_ft: str,
        ground_speed_mph: str,
        row2_alt_ft: str | None = None,
    ) -> None:
        self._row1_alt.setText(row1_alt_ft)
        self._row1_mph.setText(row1_vspeed_mph)
        self._row1_time.setText(time_text)
        self._row2_msl.setText(distance_home_ft)
        self._row2_mph.setText(ground_speed_mph)
        self._row2_alt.setText(row2_alt_ft if row2_alt_ft is not None else row1_alt_ft)


def format_map_zoom_level(z: float) -> str:
    zf = float(z)
    if abs(zf - round(zf)) < 0.05:
        return f"z{int(round(zf))}"
    return f"z{zf:.1f}"


_MAP_ZOOM_BTN_SIDE = 32
_MAP_ZOOM_BTN_GAP = 4
_MAP_ZOOM_BTN_QSS = (
    "QPushButton#mapZoomPlusBtn, QPushButton#mapZoomMinusBtn {"
    "background-color: rgba(18, 26, 40, 0.88);"
    "color: #f5f8ff;"
    "border: 2px solid rgba(200, 218, 255, 0.95);"
    "border-radius: 6px;"
    "padding: 0px;"
    "}"
    "QPushButton#mapZoomPlusBtn:hover, QPushButton#mapZoomMinusBtn:hover {"
    "background-color: rgba(32, 46, 68, 0.92);"
    "border-color: #ffffff;"
    "color: #ffffff;"
    "}"
    "QPushButton#mapZoomPlusBtn:pressed, QPushButton#mapZoomMinusBtn:pressed {"
    "background-color: rgba(12, 18, 28, 0.95);"
    "border-color: #9eb6e8;"
    "}"
    "QLabel#mapZoomLevelLabel {"
    "background-color: rgba(18, 26, 40, 0.88);"
    "color: #f5f8ff;"
    "border: 2px solid rgba(200, 218, 255, 0.95);"
    "border-radius: 6px;"
    "padding: 0px;"
    "font-family: \"Segoe UI\", \"Roboto\", \"Helvetica Neue\", sans-serif;"
    "font-size: 13px;"
    "font-weight: 600;"
    "}"
)


def format_map_zoom_level_display(z: float) -> str:
    """Compact level for the stacked map zoom control (fits 32×32 chrome)."""
    zf = float(z)
    if abs(zf - round(zf)) < 0.05:
        return str(int(round(zf)))
    return f"{zf:.1f}"


class MapZoomControlPanel(QWidget):
    """Bottom-left map zoom: stacked + / level / − (matches minimap PiP control chrome)."""

    zoom_step_requested = Signal(int)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setObjectName("mapZoomControl")
        self.setStyleSheet("QWidget#mapZoomControl { background: transparent; border: none; }")
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(_MAP_ZOOM_BTN_GAP)
        btn_font = QFont()
        btn_font.setPointSize(16)
        btn_font.setWeight(QFont.Weight.Black)
        side = _MAP_ZOOM_BTN_SIDE
        self._btn_plus = QPushButton("+")
        self._btn_plus.setObjectName("mapZoomPlusBtn")
        self._btn_plus.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self._btn_plus.setToolTip("Zoom in")
        self._btn_plus.clicked.connect(lambda: self.zoom_step_requested.emit(1))
        self._btn_minus = QPushButton("-")
        self._btn_minus.setObjectName("mapZoomMinusBtn")
        self._btn_minus.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self._btn_minus.setToolTip("Zoom out")
        self._btn_minus.clicked.connect(lambda: self.zoom_step_requested.emit(-1))
        self._level = QLabel(format_map_zoom_level_display(16.0))
        self._level.setObjectName("mapZoomLevelLabel")
        self._level.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._level.setFixedSize(side, side)
        self._level.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        self._level.setStyleSheet(_MAP_ZOOM_BTN_QSS)
        for btn in (self._btn_plus, self._btn_minus):
            btn.setFont(btn_font)
            btn.setFixedSize(side, side)
            btn.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
            btn.setStyleSheet(_MAP_ZOOM_BTN_QSS)
        lay.addWidget(self._btn_plus, 0, Qt.AlignmentFlag.AlignHCenter)
        lay.addWidget(self._level, 0, Qt.AlignmentFlag.AlignHCenter)
        lay.addWidget(self._btn_minus, 0, Qt.AlignmentFlag.AlignHCenter)
        panel_h = side * 3 + _MAP_ZOOM_BTN_GAP * 2
        self.setFixedSize(side, panel_h)
        self.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        self.set_zoom_level(16.0)

    def set_zoom_level(self, z: float) -> None:
        level = format_map_zoom_level(z)
        display = format_map_zoom_level_display(z)
        try:
            self._level.setText(display)
        except Exception:
            pass
        tip = f"Map zoom {level}"
        self.setToolTip(tip)
        try:
            self._level.setToolTip(tip)
            self._btn_plus.setToolTip(f"Zoom in ({level})")
            self._btn_minus.setToolTip(f"Zoom out ({level})")
        except Exception:
            pass


def telemetry_strip_icon_as_qicon(kind: str, size: int = 24) -> QIcon:
    """Rasterize a ``TelemetryStripIcon`` for QToolButton / menus — matches strip line-art style."""
    sz = max(16, int(size))
    wgt = TelemetryStripIcon(kind, None, icon_size=sz)
    pm = QPixmap(sz, sz)
    pm.fill(Qt.GlobalColor.transparent)
    with QPainter(pm) as painter:
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        # PySide6: render(QPainter) alone is invalid; offset maps widget (0,0) to pixmap (0,0).
        wgt.render(painter, QPoint(0, 0))
    return QIcon(pm)
