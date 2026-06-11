"""Screen-space markers over Cesium — same paint as NativeTileMapView (2D map)."""

from __future__ import annotations

from PySide6.QtCore import QPointF, Qt
from PySide6.QtGui import QBrush, QColor, QFont, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import QVBoxLayout, QWidget

# Match native_tile_map.py DOOAF / vehicle styling exactly.
_DOOAF_STYLES: dict[str, dict] = {
    "gun": {
        "radius": 9,
        "fill": QColor(30, 70, 160, 230),
        "outline": QColor(180, 210, 255, 240),
        "label": "GUN",
    },
    "target": {
        "radius": 7,
        "fill": QColor(40, 170, 80, 220),
        "outline": QColor(180, 255, 200, 240),
        "label": "TARGET",
    },
    "hit": {
        "radius": 7,
        "fill": QColor(220, 90, 40, 220),
        "outline": QColor(255, 200, 140, 240),
        "label": "HIT",
    },
}


class Map3dMarkerOverlay(QWidget):
    """Transparent layer: identical shapes to the native 2D map at Cesium screen coords."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self.setAttribute(Qt.WidgetAttribute.WA_NoSystemBackground, True)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)
        self._items: list[dict] = []

    def set_items(self, items: list[dict]) -> None:
        self._items = list(items) if items else []
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802
        del event
        if not self._items:
            return
        p = QPainter(self)
        try:
            p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        except Exception:
            pass
        lines = [it for it in self._items if str(it.get("kind") or "") == "line"]
        marks = [it for it in self._items if str(it.get("kind") or "") != "line"]
        for it in lines:
            self._paint_line(p, it)
        for it in marks:
            kind = str(it.get("kind") or "")
            if kind == "vehicle":
                continue
            try:
                x = float(it.get("x"))
                y = float(it.get("y"))
            except (TypeError, ValueError):
                continue
            if not bool(it.get("visible", True)):
                continue
            if kind == "obs":
                self._paint_obs_mark(p, x, y, str(it.get("color") or "yellow"))
            elif kind == "dooaf":
                self._paint_dooaf(p, x, y, str(it.get("role") or "gun"))
            elif kind == "waypoint":
                self._paint_waypoint(p, x, y, int(it.get("index") or 1))
        for it in marks:
            if str(it.get("kind") or "") != "vehicle":
                continue
            try:
                x = float(it.get("x"))
                y = float(it.get("y"))
            except (TypeError, ValueError):
                continue
            if bool(it.get("visible", True)):
                self._paint_vehicle(p, x, y, float(it.get("heading") or 0))
        p.end()

    @staticmethod
    def _paint_line(p: QPainter, it: dict) -> None:
        try:
            x0 = float(it.get("x0"))
            y0 = float(it.get("y0"))
            x1 = float(it.get("x1"))
            y1 = float(it.get("y1"))
        except (TypeError, ValueError):
            return
        style = str(it.get("style") or "")
        if style == "gun_target":
            p.setPen(QPen(QColor(80, 140, 255, 230), 3, Qt.PenStyle.SolidLine))
        elif style == "gun_impact":
            p.setPen(QPen(QColor(120, 200, 255, 210), 2, Qt.PenStyle.DashLine))
        else:
            p.setPen(QPen(QColor(255, 120, 200, 220), 2, Qt.PenStyle.DashLine))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawLine(QPointF(x0, y0), QPointF(x1, y1))
        label = str(it.get("label") or "").strip()
        if label:
            mx = (x0 + x1) / 2.0
            my = (y0 + y1) / 2.0
            p.setFont(QFont("Segoe UI", 9, QFont.Weight.Bold))
            metrics = p.fontMetrics()
            tw = metrics.horizontalAdvance(label) + 10
            th = metrics.height() + 6
            tx = int(mx - tw / 2)
            ty = int(my - th / 2)
            p.fillRect(tx, ty, tw, th, QColor(0, 0, 0, 210))
            p.setPen(QColor(255, 200, 240))
            p.drawText(tx + 5, ty + metrics.ascent() + 3, label)

    @staticmethod
    def _paint_vehicle(p: QPainter, x: float, y: float, heading_deg: float) -> None:
        p.save()
        p.translate(x, y)
        p.rotate(heading_deg)
        vb = 1.0
        tip_y = -28.0 * vb
        base_y = 15.0 * vb
        half_w = 12.0 * vb
        ctrl_x, ctrl_y = 0.0, 5.0 * vb
        arrow = QPainterPath()
        arrow.moveTo(0.0, tip_y)
        arrow.lineTo(-half_w, base_y)
        arrow.quadTo(ctrl_x, ctrl_y, half_w, base_y)
        arrow.closeSubpath()
        p.setBrush(QColor(240, 44, 52, 255))
        p.setPen(
            QPen(
                QColor(255, 255, 255, 255),
                max(2.0, min(6.5, 3.0 * vb)),
                Qt.PenStyle.SolidLine,
                Qt.PenCapStyle.RoundCap,
                Qt.PenJoinStyle.RoundJoin,
            )
        )
        p.drawPath(arrow)
        dot_r = max(2.4, 2.4 * vb)
        p.setPen(QPen(QColor(255, 255, 255, 255), 1))
        p.setBrush(QColor(255, 255, 255, 255))
        p.drawEllipse(QPointF(0.0, 1.0 * vb), dot_r, dot_r)
        p.restore()

    @staticmethod
    def _paint_obs_mark(p: QPainter, x: float, y: float, color: str) -> None:
        if color == "cyan":
            fill = QColor(40, 200, 255, 200)
            stroke = QColor(60, 220, 255, 240)
            arm = 12
            radius = 6
        else:
            fill = QColor(255, 210, 70, 210)
            stroke = QColor(255, 210, 70, 230)
            arm = 10
            radius = 5
        p.setPen(QPen(stroke, 2))
        p.setBrush(fill)
        p.drawEllipse(QPointF(x, y), radius, radius)
        p.drawLine(int(x - arm), int(y), int(x + arm), int(y))
        p.drawLine(int(x), int(y - arm), int(x), int(y + arm))

    @staticmethod
    def _paint_dooaf(p: QPainter, x: float, y: float, role: str) -> None:
        st = _DOOAF_STYLES.get(role, _DOOAF_STYLES["gun"])
        radius = int(st["radius"])
        p.setPen(QPen(st["outline"], 2))
        p.setBrush(st["fill"])
        p.drawEllipse(QPointF(x, y), radius, radius)
        label = str(st["label"])
        p.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        metrics = p.fontMetrics()
        tw = metrics.horizontalAdvance(label) + 8
        th = metrics.height() + 4
        tx = int(x) - tw // 2
        ty = int(y) - radius - th - 4
        p.fillRect(tx, ty, tw, th, QColor(0, 0, 0, 210))
        p.setPen(QColor(240, 245, 255))
        p.drawText(tx + 4, ty + metrics.ascent() + 2, label)

    @staticmethod
    def _paint_waypoint(p: QPainter, x: float, y: float, index: int) -> None:
        p.setPen(QPen(QColor(255, 255, 255), 1))
        p.setBrush(QBrush(QColor(40, 130, 255, 220)))
        p.drawEllipse(QPointF(x, y), 7, 7)
        p.drawText(
            int(x - 16),
            int(y - 8),
            32,
            16,
            int(Qt.AlignmentFlag.AlignCenter),
            str(index),
        )


class Map3dLayer(QWidget):
    """Cesium WebEngine view container (markers render in-page + optional Qt overlay)."""

    def __init__(self, web_view: QWidget, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._web = web_view
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        lay.addWidget(web_view)
