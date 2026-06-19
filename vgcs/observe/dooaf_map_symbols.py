"""Shared DOOAF map markers — artillery gun and designated target crosshair."""

from __future__ import annotations

import math

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QPainter, QPen

# Gun (artillery position)
GUN_RING = QColor(29, 78, 216, 235)
GUN_RING_STROKE = QColor(191, 219, 254, 250)
GUN_HALO = QColor(37, 99, 235, 55)
GUN_BODY = QColor(147, 197, 253)
GUN_BREECH = QColor(59, 130, 246)
GUN_BARREL = QColor(219, 234, 254)
GUN_OUTLINE = QColor(30, 58, 138)

# Target (intended impact point)
TARGET_STROKE = QColor(34, 197, 94, 250)
TARGET_INNER = QColor(134, 239, 172, 230)
TARGET_HALO = QColor(34, 197, 94, 45)
TARGET_CENTER = QColor(22, 163, 74, 255)
TARGET_OUTLINE = QColor(220, 252, 231, 250)


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Geographic bearing from point 1 to point 2 (degrees clockwise from north)."""
    lat1r = math.radians(lat1)
    lat2r = math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    x = math.sin(dlon) * math.cos(lat2r)
    y = math.cos(lat1r) * math.sin(lat2r) - math.sin(lat1r) * math.cos(lat2r) * math.cos(dlon)
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


def _qt_rotation_for_bearing(bearing: float | None) -> float | None:
    """Map geographic bearing to Qt painter rotation (icon barrel defaults to east)."""
    if bearing is None:
        return None
    return float(bearing) - 90.0


def paint_gun_marker(
    painter: QPainter,
    cx: float,
    cy: float,
    *,
    scale: float = 1.0,
    bearing_deg: float | None = None,
) -> float:
    """Draw artillery-position marker; returns outer radius for label placement."""
    s = max(0.5, float(scale))
    ring_r = 13.0 * s

    painter.setPen(Qt.PenStyle.NoPen)
    painter.setBrush(GUN_HALO)
    painter.drawEllipse(QPointF(cx, cy), ring_r + 3.0 * s, ring_r + 3.0 * s)

    painter.setPen(QPen(GUN_RING_STROKE, max(2.0, 2.5 * s)))
    painter.setBrush(GUN_RING)
    painter.drawEllipse(QPointF(cx, cy), ring_r, ring_r)

    painter.save()
    painter.translate(cx, cy)
    rot = _qt_rotation_for_bearing(bearing_deg)
    if rot is not None:
        painter.rotate(rot)
    painter.scale(s, s)

    outline = QPen(GUN_OUTLINE, 1.4)
    outline.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
    painter.setPen(outline)

    painter.setBrush(GUN_BODY)
    painter.drawRoundedRect(QRectF(-7.0, -4.5, 10.0, 9.0), 2.0, 2.0)

    painter.setBrush(GUN_BREECH)
    painter.drawEllipse(QPointF(-2.0, 0.0), 4.0, 4.0)

    painter.setBrush(GUN_BARREL)
    painter.drawRoundedRect(QRectF(2.0, -2.0, 15.0, 4.0), 1.5, 1.5)
    painter.drawLine(QPointF(17.0, 0.0), QPointF(19.0, 0.0))

    painter.restore()
    return ring_r + 3.0 * s


def paint_target_marker(
    painter: QPainter,
    cx: float,
    cy: float,
    *,
    scale: float = 1.0,
) -> float:
    """Draw designated-target reticle; returns outer radius for label placement."""
    s = max(0.5, float(scale))
    r_outer = 12.0 * s
    r_inner = 5.0 * s
    gap = 3.0 * s
    ext = 5.0 * s

    painter.setPen(Qt.PenStyle.NoPen)
    painter.setBrush(TARGET_HALO)
    painter.drawEllipse(QPointF(cx, cy), r_outer + 4.0 * s, r_outer + 4.0 * s)

    painter.setPen(QPen(TARGET_OUTLINE, max(2.0, 2.5 * s)))
    painter.setBrush(Qt.BrushStyle.NoBrush)
    painter.drawEllipse(QPointF(cx, cy), r_outer, r_outer)

    painter.setPen(QPen(TARGET_INNER, max(1.0, 1.5 * s)))
    painter.drawEllipse(QPointF(cx, cy), r_inner, r_inner)

    cross_pen = QPen(TARGET_STROKE, max(2.0, 2.2 * s))
    cross_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
    painter.setPen(cross_pen)
    painter.drawLine(QPointF(cx - r_outer - ext, cy), QPointF(cx - gap, cy))
    painter.drawLine(QPointF(cx + gap, cy), QPointF(cx + r_outer + ext, cy))
    painter.drawLine(QPointF(cx, cy - r_outer - ext), QPointF(cx, cy - gap))
    painter.drawLine(QPointF(cx, cy + gap), QPointF(cx, cy + r_outer + ext))

    bracket = 4.5 * s
    bpen = QPen(TARGET_STROKE, max(1.5, 2.0 * s))
    bpen.setCapStyle(Qt.PenCapStyle.RoundCap)
    painter.setPen(bpen)
    off = r_outer * 0.78
    for sx, sy in ((1, 1), (-1, 1), (1, -1), (-1, -1)):
        bx = cx + sx * off
        by = cy + sy * off
        painter.drawLine(QPointF(bx, by), QPointF(bx + sx * bracket, by))
        painter.drawLine(QPointF(bx, by), QPointF(bx, by + sy * bracket))

    painter.setPen(Qt.PenStyle.NoPen)
    painter.setBrush(TARGET_CENTER)
    painter.drawEllipse(QPointF(cx, cy), 2.5 * s, 2.5 * s)

    return r_outer + ext


def _svg_gun_body() -> str:
    return (
        "<path d='M-7 2.5 L-7 -2.5 L4 -3.5 L4 3.5 Z' fill='#93c5fd' stroke='#1e3a8a' "
        "stroke-width='1.2' stroke-linejoin='round'/>"
        "<circle cx='-2' cy='0' r='4' fill='#3b82f6' stroke='#1e3a8a' stroke-width='1'/>"
        "<rect x='2' y='-2' width='15' height='4' rx='1.5' fill='#dbeafe' "
        "stroke='#1e3a8a' stroke-width='1'/>"
        "<line x1='17' y1='0' x2='19' y2='0' stroke='#1e3a8a' stroke-width='1.4' "
        "stroke-linecap='round'/>"
    )


def svg_gun_marker(
    cx: float,
    cy: float,
    *,
    scale: float = 1.0,
    bearing_deg: float | None = None,
) -> str:
    s = float(scale)
    rot = _qt_rotation_for_bearing(bearing_deg)
    inner = (
        f"<g transform='translate({cx:.1f},{cy:.1f}) scale({s:.2f})'>"
        f"{_svg_gun_body()}"
        "</g>"
    )
    if rot is None:
        ring = (
            f"<circle cx='{cx:.1f}' cy='{cy:.1f}' r='{13 * s:.1f}' fill='#1d4ed8' "
            f"fill-opacity='0.92' stroke='#bfdbfe' stroke-width='2.5'/>"
        )
        return ring + inner
    return (
        f"<g transform='translate({cx:.1f},{cy:.1f}) rotate({rot:.2f})'>"
        f"<circle cx='0' cy='0' r='{13 * s:.1f}' fill='#1d4ed8' fill-opacity='0.92' "
        f"stroke='#bfdbfe' stroke-width='2.5'/>"
        f"<g transform='scale({s:.2f})'>{_svg_gun_body()}</g>"
        "</g>"
    )


def svg_target_marker(cx: float, cy: float, *, r: float = 12.0) -> str:
    rr = float(r)
    gap = rr * 0.25
    ext = rr * 0.42
    off = rr * 0.78
    bracket = rr * 0.38
    parts = [
        f"<circle cx='{cx:.1f}' cy='{cy:.1f}' r='{rr + 4:.1f}' fill='#22c55e' fill-opacity='0.12'/>",
        f"<circle cx='{cx:.1f}' cy='{cy:.1f}' r='{rr:.1f}' fill='none' stroke='#dcfce7' "
        f"stroke-width='2.5'/>",
        f"<circle cx='{cx:.1f}' cy='{cy:.1f}' r='{rr * 0.42:.1f}' fill='none' stroke='#86efac' "
        f"stroke-width='1.5'/>",
        f"<line x1='{cx - rr - ext:.1f}' y1='{cy:.1f}' x2='{cx - gap:.1f}' y2='{cy:.1f}' "
        f"stroke='#22c55e' stroke-width='2.2' stroke-linecap='round'/>",
        f"<line x1='{cx + gap:.1f}' y1='{cy:.1f}' x2='{cx + rr + ext:.1f}' y2='{cy:.1f}' "
        f"stroke='#22c55e' stroke-width='2.2' stroke-linecap='round'/>",
        f"<line x1='{cx:.1f}' y1='{cy - rr - ext:.1f}' x2='{cx:.1f}' y2='{cy - gap:.1f}' "
        f"stroke='#22c55e' stroke-width='2.2' stroke-linecap='round'/>",
        f"<line x1='{cx:.1f}' y1='{cy + gap:.1f}' x2='{cx:.1f}' y2='{cy + rr + ext:.1f}' "
        f"stroke='#22c55e' stroke-width='2.2' stroke-linecap='round'/>",
    ]
    for sx, sy in ((1, 1), (-1, 1), (1, -1), (-1, -1)):
        bx = cx + sx * off
        by = cy + sy * off
        parts.append(
            f"<line x1='{bx:.1f}' y1='{by:.1f}' x2='{bx + sx * bracket:.1f}' y2='{by:.1f}' "
            f"stroke='#22c55e' stroke-width='2' stroke-linecap='round'/>"
        )
        parts.append(
            f"<line x1='{bx:.1f}' y1='{by:.1f}' x2='{bx:.1f}' y2='{by + sy * bracket:.1f}' "
            f"stroke='#22c55e' stroke-width='2' stroke-linecap='round'/>"
        )
    parts.append(
        f"<circle cx='{cx:.1f}' cy='{cy:.1f}' r='2.5' fill='#16a34a'/>"
    )
    return "".join(parts)
