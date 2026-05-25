"""
M9 — Sensor visualization & obstacle detection display (AeroGCS-style radar).

Native Qt (PySide6): hero polar plot + compact telemetry strip on a map HUD card.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

from PySide6.QtCore import QPointF, QRectF, QSize, Qt, QTimer
from PySide6.QtGui import (
    QColor,
    QConicalGradient,
    QFont,
    QFontMetrics,
    QPainter,
    QPainterPath,
    QPen,
    QPolygonF,
    QRadialGradient,
)
from PySide6.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from vgcs.map.map_footer_hud import TELEMETRY_STRIP_VALUE_STYLE, TelemetryStripIcon

_SENSOR_LASER = 0
_SENSOR_ULTRASOUND = 1
_SENSOR_INFRARED = 2
_SENSOR_RADAR = 3
_OBSTACLE_BIN_COUNT = 72
_NO_READING_CM = 0xFFFF

_RADAR_PLOT_PX = 192
# Strip slightly wider than radar so sensor labels + status are not clipped.
_STRIP_WIDTH_PX = 216
_PANEL_WIDTH_PX = _STRIP_WIDTH_PX

_HUD_TEXT = QColor(230, 238, 252)
_HUD_TEXT_DIM = QColor(130, 142, 168)
_HUD_ACCENT = QColor(64, 180, 255)
_HUD_RING = QColor(90, 118, 158)
_HUD_SAFE = QColor(52, 211, 153)
_HUD_WARN = QColor(251, 191, 36)
_HUD_DANGER = QColor(248, 113, 113)

# Match ``MapFooterTelemetryStrip`` in ``map_footer_hud.py``.
_TELEMETRY_STRIP_QSS = (
    "QFrame#obstacleTelemetryStrip {"
    " background: rgba(26, 33, 45, 215);"
    " border: 1px solid rgba(80, 92, 118, 107);"
    " border-radius: 12px;"
    "}"
)


def _sensor_family_label(sensor_type: int) -> str:
    if sensor_type == _SENSOR_RADAR:
        return "Radar"
    if sensor_type == _SENSOR_LASER:
        return "LiDAR"
    if sensor_type in (_SENSOR_ULTRASOUND, _SENSOR_INFRARED):
        return "Rangefinder"
    return "Proximity"


def _orientation_label(orientation: int) -> str:
    names = {
        0: "FWD",
        2: "FWD-R",
        4: "RIGHT",
        6: "AFT-R",
        8: "AFT",
        10: "AFT-L",
        12: "LEFT",
        14: "FWD-L",
        25: "DOWN",
        24: "UP",
    }
    return names.get(int(orientation), f"#{int(orientation)}")


def _mav_to_qt_deg(mav_deg: float) -> float:
    return (90.0 - float(mav_deg)) % 360.0


def _threat_color(dist_m: float, max_r_m: float, *, stale: bool) -> QColor:
    t = 1.0 - min(1.0, dist_m / max(max_r_m * 0.35, 1.0))
    if t < 0.45:
        col = QColor(_HUD_SAFE)
    elif t < 0.75:
        col = QColor(_HUD_WARN)
    else:
        col = QColor(_HUD_DANGER)
    col.setAlpha(90 if stale else 200)
    return col


@dataclass
class ObstacleDistanceState:
    sensor_type: int = _SENSOR_LASER
    distances_cm: list[int] = field(default_factory=lambda: [_NO_READING_CM] * _OBSTACLE_BIN_COUNT)
    increment_deg: float = 5.0
    angle_offset_deg: float = 0.0
    min_distance_m: float = 0.2
    max_distance_m: float = 30.0
    updated_mono: float = 0.0

    def nearest_m(self) -> float | None:
        best: int | None = None
        for d in self.distances_cm:
            if d <= 0 or d >= _NO_READING_CM:
                continue
            if best is None or d < best:
                best = d
        if best is None:
            return None
        return float(best) / 100.0

    def active_bin_count(self) -> int:
        return sum(1 for d in self.distances_cm if 0 < d < _NO_READING_CM)


@dataclass
class DistanceSensorState:
    sensor_type: int = _SENSOR_LASER
    orientation: int = 0
    current_distance_m: float | None = None
    min_distance_m: float = 0.0
    max_distance_m: float = 30.0
    updated_mono: float = 0.0


class ObstacleRadarCanvas(QWidget):
    """Hero polar plot — fills the bezel; vehicle forward = top."""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setFixedSize(_RADAR_PLOT_PX, _RADAR_PLOT_PX)
        self.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)
        self.setAutoFillBackground(False)
        self._obstacle = ObstacleDistanceState()
        self._stale = True
        self._has_proximity = False
        self._sweep_deg = 0.0
        self._anim = QTimer(self)
        self._anim.setInterval(45)
        self._anim.timeout.connect(self._advance_sweep)
        self._anim.start()

    def _advance_sweep(self) -> None:
        if self._has_proximity:
            return
        self._sweep_deg = (self._sweep_deg + 3.2) % 360.0
        self.update()

    def set_obstacle_state(self, state: ObstacleDistanceState) -> None:
        self._obstacle = state
        self._stale = (time.monotonic() - float(state.updated_mono or 0.0)) > 2.5
        self._has_proximity = state.active_bin_count() > 0
        self.update()

    @staticmethod
    def _annular_wedge(
        path: QPainterPath,
        cx: float,
        cy: float,
        r_in: float,
        r_out: float,
        mav_start_deg: float,
        mav_end_deg: float,
    ) -> None:
        qs = _mav_to_qt_deg(mav_start_deg)
        qe = _mav_to_qt_deg(mav_end_deg)
        span = qe - qs
        if span <= 0.1:
            span += 360.0
        rect_out = QRectF(cx - r_out, cy - r_out, 2 * r_out, 2 * r_out)
        rect_in = QRectF(cx - r_in, cy - r_in, 2 * r_in, 2 * r_in)
        path.arcMoveTo(rect_out, qs)
        path.arcTo(rect_out, qs, span)
        path.arcTo(rect_in, qs + span, -span)
        path.closeSubpath()

    def paintEvent(self, event) -> None:  # noqa: N802
        del event
        p = QPainter(self)
        if not p.isActive():
            return
        try:
            self._paint_radar(p)
        except Exception:
            pass

    def _paint_radar(self, p: QPainter) -> None:
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        cx, cy = w / 2.0, h / 2.0
        radius = min(w, h) * 0.46

        # Compass-style: faint outer ring + opaque inner disc (no square panel behind).
        p.setPen(QPen(QColor(255, 255, 255, 40), 1))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawEllipse(QPointF(cx, cy), radius * 1.02, radius * 1.02)

        inner_r = radius * 0.96
        p.setPen(QPen(QColor(148, 160, 180, 82), 2))
        p.setBrush(QColor(22, 26, 34, int(0.96 * 255)))
        p.drawEllipse(QPointF(cx, cy), inner_r, inner_r)
        radius = inner_r

        max_r_m = max(0.5, float(self._obstacle.max_distance_m or 30.0))
        ring_fracs = (0.33, 0.66, 1.0)
        f_ring = QFont("Segoe UI", 8)
        f_ring.setWeight(QFont.Weight.DemiBold)
        p.setFont(f_ring)

        for i, frac in enumerate(ring_fracs):
            rr = radius * frac * 0.92
            alpha = 95 if i < 2 else 130
            p.setPen(QPen(QColor(_HUD_RING.red(), _HUD_RING.green(), _HUD_RING.blue(), alpha), 1.2))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawEllipse(QRectF(cx - rr, cy - rr, 2 * rr, 2 * rr))
            dist_lbl = max_r_m * frac
            p.setPen(_HUD_TEXT_DIM)
            p.drawText(
                int(cx + rr - 22),
                int(cy - 6),
                20,
                12,
                int(Qt.AlignmentFlag.AlignRight),
                f"{dist_lbl:.0f}",
            )

        # Bearing ticks
        p.setPen(QPen(QColor(140, 160, 190, 90), 1.0))
        for deg in range(0, 360, 30):
            rad = math.radians(deg - 90.0)
            r0 = radius * 0.88
            r1 = radius * 0.96
            p.drawLine(
                QPointF(cx + r0 * math.cos(rad), cy + r0 * math.sin(rad)),
                QPointF(cx + r1 * math.cos(rad), cy + r1 * math.sin(rad)),
            )

        p.setPen(QPen(QColor(100, 130, 170, 50), 1.0))
        p.drawLine(int(cx - radius * 0.9), int(cy), int(cx + radius * 0.9), int(cy))
        p.drawLine(int(cx), int(cy - radius * 0.9), int(cx), int(cy + radius * 0.9))

        # Idle sweep
        if not self._has_proximity:
            sweep_rad = math.radians(self._sweep_deg - 90.0)
            sweep_grad = QConicalGradient(cx, cy, self._sweep_deg - 90.0)
            sweep_grad.setColorAt(0.0, QColor(64, 180, 255, 0))
            sweep_grad.setColorAt(0.08, QColor(64, 180, 255, 55))
            sweep_grad.setColorAt(0.2, QColor(64, 180, 255, 0))
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(sweep_grad)
            p.drawEllipse(QRectF(cx - radius * 0.92, cy - radius * 0.92, 1.84 * radius, 1.84 * radius))
            p.setPen(QPen(QColor(64, 180, 255, 120), 2.0))
            p.drawLine(
                QPointF(cx, cy),
                QPointF(cx + radius * 0.88 * math.cos(sweep_rad), cy + radius * 0.88 * math.sin(sweep_rad)),
            )

        obs = self._obstacle
        inc = float(obs.increment_deg or 5.0)
        off = float(obs.angle_offset_deg or 0.0)
        r_in = max(5.0, radius * 0.12)

        p.setPen(Qt.PenStyle.NoPen)
        for i, d_cm in enumerate(obs.distances_cm[:_OBSTACLE_BIN_COUNT]):
            if d_cm <= 0 or d_cm >= _NO_READING_CM:
                continue
            dist_m = min(float(d_cm) / 100.0, max_r_m)
            center_deg = off + float(i) * inc
            half = inc * 0.46
            r_out = radius * 0.9 * (dist_m / max_r_m)
            wedge_path = QPainterPath()
            self._annular_wedge(wedge_path, cx, cy, r_in, r_out, center_deg - half, center_deg + half)
            p.setBrush(_threat_color(dist_m, max_r_m, stale=self._stale))
            p.drawPath(wedge_path)

            rad = math.radians(center_deg - 90.0)
            r_px = radius * 0.9 * (dist_m / max_r_m)
            hx = cx + r_px * math.cos(rad)
            hy = cy + r_px * math.sin(rad)
            glow = QRadialGradient(hx, hy, 8)
            glow.setColorAt(0, QColor(255, 255, 255, 220))
            glow.setColorAt(1, QColor(255, 255, 255, 0))
            p.setBrush(glow)
            p.drawEllipse(QRectF(hx - 5, hy - 5, 10, 10))

        # North
        p.setPen(QColor(248, 113, 113, 230))
        f_n = QFont("Segoe UI", 9)
        f_n.setWeight(QFont.Weight.Bold)
        p.setFont(f_n)
        p.drawText(int(cx - 8), int(cy - radius + 8), 16, 14, int(Qt.AlignmentFlag.AlignCenter), "N")

        # Vehicle
        tri_r = max(8.0, radius * 0.1)
        tri = QPolygonF(
            [
                QPointF(cx, cy - tri_r),
                QPointF(cx - tri_r * 0.7, cy + tri_r * 0.58),
                QPointF(cx + tri_r * 0.7, cy + tri_r * 0.58),
            ]
        )
        p.setPen(QPen(_HUD_TEXT, 1.6))
        fill = _HUD_ACCENT if not self._stale else QColor(_HUD_ACCENT.red(), _HUD_ACCENT.green(), _HUD_ACCENT.blue(), 150)
        p.setBrush(fill)
        p.drawPolygon(tri)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor(255, 255, 255, 180))
        p.drawEllipse(QRectF(cx - 2.5, cy - 2.5, 5, 5))

        f_max = QFont("Segoe UI", 8)
        f_max.setWeight(QFont.Weight.DemiBold)
        p.setFont(f_max)
        p.setPen(QColor(241, 245, 255, 160))
        p.drawText(
            QRectF(cx - radius, cy + radius - 18, 2 * radius, 14),
            int(Qt.AlignmentFlag.AlignHCenter),
            f"MAX {max_r_m:.0f} m",
        )


class _StripMetricCell(QWidget):
    """Icon + value — same layout and typography as ``MapFooterTelemetryStrip`` cells."""

    def __init__(self, icon_kind: str, parent=None) -> None:
        super().__init__(parent)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(2, 2, 2, 2)
        lay.setSpacing(8)
        self._icon = TelemetryStripIcon(icon_kind, self)
        self._val = QLabel("—")
        self._val.setStyleSheet(TELEMETRY_STRIP_VALUE_STYLE)
        self._val.setWordWrap(False)
        self._val.setAlignment(
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter
        )
        lay.addWidget(self._icon, 0, Qt.AlignmentFlag.AlignVCenter)
        lay.addWidget(self._val, 1, Qt.AlignmentFlag.AlignVCenter)

    def set_value(self, text: str, *, alert: bool = False) -> None:
        self._val.setText(text)
        if alert:
            self._val.setStyleSheet("color: #f87171; font-size: 15px; font-weight: 600;")
        else:
            self._val.setStyleSheet(TELEMETRY_STRIP_VALUE_STYLE)


class _SensorSegment(QWidget):
    """Unified segmented sensor indicators — no chrome (matches map footer HUD)."""

    _FONT = QFont("Segoe UI", 12)
    _FONT.setWeight(QFont.Weight.DemiBold)

    _SEG_STYLE_IDLE = "color: #dce5f5; background: transparent; border: none;"
    _SEG_STYLE_ON = (
        "color: #ecfdf5; background: rgba(255, 255, 255, 0.08);"
        " border: none; border-radius: 6px;"
    )

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)
        self.setStyleSheet("background: transparent; border: none;")
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 2, 0, 2)
        lay.setSpacing(6)
        self._lidar = QLabel("LiDAR")
        self._radar = QLabel("Radar")
        self._rf = QLabel("RF")
        self._rf.setToolTip("Rangefinder")
        text_h = int(QFontMetrics(self._FONT).height()) + 4
        self.setMinimumHeight(text_h + 4)
        for lb in (self._lidar, self._radar, self._rf):
            lb.setFont(self._FONT)
            lb.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lb.setMinimumHeight(text_h)
            lb.setSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Minimum)
            lay.addWidget(lb)
        self.set_active(None)

    def set_active(self, which: str | None) -> None:
        for key, lb in (("lidar", self._lidar), ("radar", self._radar), ("rf", self._rf)):
            on = which == key
            lb.setStyleSheet(self._SEG_STYLE_ON if on else self._SEG_STYLE_IDLE)


class ObstacleRadarPanel(QFrame):
    """Top-left map HUD — floating radar + labels (no card chrome; parity with compass / telemetry)."""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setObjectName("obstacleRadarPanel")
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)
        self.setAutoFillBackground(False)
        self.setStyleSheet("QFrame#obstacleRadarPanel { background: transparent; border: none; }")

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(8)

        self._radar = ObstacleRadarCanvas(self)
        root.addWidget(self._radar, 0, Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop)

        metrics_strip = QFrame()
        metrics_strip.setObjectName("obstacleTelemetryStrip")
        metrics_strip.setStyleSheet(_TELEMETRY_STRIP_QSS)
        metrics_strip.setFixedWidth(_STRIP_WIDTH_PX)
        strip_lay = QVBoxLayout(metrics_strip)
        strip_lay.setContentsMargins(10, 6, 10, 6)
        strip_lay.setSpacing(6)

        row_metrics = QHBoxLayout()
        row_metrics.setSpacing(8)
        self._cell_nearest = _StripMetricCell("corner")
        self._cell_nearest.setToolTip("Nearest obstacle")
        self._cell_range = _StripMetricCell("alt")
        self._cell_range.setToolTip("Rangefinder distance")
        row_metrics.addWidget(self._cell_nearest, 1)
        row_metrics.addWidget(self._cell_range, 1)
        strip_lay.addLayout(row_metrics)

        row_sensors = QHBoxLayout()
        row_sensors.setSpacing(8)
        row_sensors.setContentsMargins(0, 0, 0, 0)
        self._sensor_segment = _SensorSegment()
        row_sensors.addWidget(self._sensor_segment, 1)
        status_wrap = QWidget()
        status_lay = QHBoxLayout(status_wrap)
        status_lay.setContentsMargins(0, 0, 0, 0)
        status_lay.setSpacing(6)
        self._status_dot = QLabel()
        self._status_dot.setFixedSize(7, 7)
        self._status_dot.setStyleSheet("background: #6b7280; border-radius: 4px;")
        self._lbl_status = QLabel("Waiting")
        status_font = QFont("Segoe UI", 12)
        status_font.setWeight(QFont.Weight.DemiBold)
        self._lbl_status.setFont(status_font)
        self._lbl_status.setStyleSheet("color: #dce5f5; background: transparent; border: none;")
        self._lbl_status.setMinimumHeight(int(QFontMetrics(status_font).height()) + 2)
        status_lay.addWidget(self._status_dot, 0, Qt.AlignmentFlag.AlignVCenter)
        status_lay.addWidget(self._lbl_status, 0, Qt.AlignmentFlag.AlignVCenter)
        row_sensors.addWidget(status_wrap, 0, Qt.AlignmentFlag.AlignVCenter)
        strip_lay.addLayout(row_sensors)

        root.addWidget(metrics_strip, 0, Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop)

        self._obstacle = ObstacleDistanceState()
        self._rangefinder = DistanceSensorState()
        self._proximity_stream_seen = False
        self._rangefinder_stream_seen = False

        self._stale_timer = QTimer(self)
        self._stale_timer.setInterval(500)
        self._stale_timer.timeout.connect(self._refresh_stale_ui)
        self._stale_timer.start()

        self.setFixedSize(_PANEL_WIDTH_PX, self._panel_height_px())

    @staticmethod
    def _panel_height_px() -> int:
        # Radar + 2-row telemetry strip (metrics, then sensors + status).
        strip_h = 6 + 6 + 32 + 6 + 30  # margins + row1 + gap + row2 (tall enough for 12px glyphs)
        return _RADAR_PLOT_PX + 8 + strip_h

    def sizeHint(self) -> QSize:  # noqa: N802
        return QSize(_PANEL_WIDTH_PX, self._panel_height_px())

    def minimumSizeHint(self) -> QSize:  # noqa: N802
        return self.sizeHint()

    def _set_status_mode(self, mode: str) -> None:
        colors = {
            "idle": "#6b7280",
            "live": "#34d399",
            "stale": "#fbbf24",
        }
        c = colors.get(mode, colors["idle"])
        self._status_dot.setStyleSheet(f"background: {c}; border-radius: 4px; min-width:8px; max-width:8px;")

    def set_vehicle_heading_deg(self, heading_deg: float) -> None:
        # Reserved for map-heading sync on the polar plot.
        del heading_deg

    def set_obstacle_distance(self, payload: dict) -> None:
        dists_raw = payload.get("distances_cm")
        distances: list[int] = [_NO_READING_CM] * _OBSTACLE_BIN_COUNT
        if isinstance(dists_raw, (list, tuple)):
            for i, v in enumerate(dists_raw[:_OBSTACLE_BIN_COUNT]):
                try:
                    distances[i] = int(v)
                except (TypeError, ValueError):
                    distances[i] = _NO_READING_CM

        st = ObstacleDistanceState(
            sensor_type=int(payload.get("sensor_type", _SENSOR_LASER) or 0),
            distances_cm=distances,
            increment_deg=float(payload.get("increment_deg", 5.0) or 5.0),
            angle_offset_deg=float(payload.get("angle_offset_deg", 0.0) or 0.0),
            min_distance_m=float(payload.get("min_distance_m", 0.2) or 0.2),
            max_distance_m=float(payload.get("max_distance_m", 30.0) or 30.0),
            updated_mono=time.monotonic(),
        )
        self._obstacle = st
        self._proximity_stream_seen = True
        self._radar.set_obstacle_state(st)

        nearest = st.nearest_m()
        if nearest is not None:
            close = nearest < max(2.0, st.max_distance_m * 0.2)
            self._cell_nearest.set_value(f"{nearest:.1f} m", alert=close)
        else:
            self._cell_nearest.set_value("—")

        bins = st.active_bin_count()
        fam = _sensor_family_label(st.sensor_type)
        if bins > 0:
            self._set_status_mode("live")
            self._lbl_status.setText(f"{bins} sectors")
        else:
            self._set_status_mode("live")
            self._lbl_status.setText("Clear")

        self._set_sensor_active(st.sensor_type)

    def set_distance_sensor(self, payload: dict) -> None:
        cur = payload.get("current_distance_m")
        try:
            cur_m = None if cur is None else float(cur)
        except (TypeError, ValueError):
            cur_m = None

        st = DistanceSensorState(
            sensor_type=int(payload.get("sensor_type", _SENSOR_LASER) or 0),
            orientation=int(payload.get("orientation", 0) or 0),
            current_distance_m=cur_m,
            min_distance_m=float(payload.get("min_distance_m", 0.0) or 0.0),
            max_distance_m=float(payload.get("max_distance_m", 30.0) or 30.0),
            updated_mono=time.monotonic(),
        )
        self._rangefinder = st
        self._rangefinder_stream_seen = True

        if cur_m is not None and cur_m >= 0:
            ori = _orientation_label(st.orientation)
            self._cell_range.set_value(f"{cur_m:.1f} m")
            self._lbl_status.setText(f"RF {ori} {cur_m:.1f}m")
            self._set_status_mode("live")
        else:
            self._cell_range.set_value("—")

        self._set_sensor_active(st.sensor_type, rangefinder=True)

    def _set_sensor_active(self, sensor_type: int, *, rangefinder: bool = False) -> None:
        if rangefinder:
            self._sensor_segment.set_active("rf")
        elif sensor_type == _SENSOR_RADAR:
            self._sensor_segment.set_active("radar")
        elif sensor_type == _SENSOR_LASER:
            self._sensor_segment.set_active("lidar")
        else:
            self._sensor_segment.set_active("rf")

    def _refresh_stale_ui(self) -> None:
        now = time.monotonic()
        obs_age = now - float(self._obstacle.updated_mono or 0.0)
        rf_age = now - float(self._rangefinder.updated_mono or 0.0)
        if self._obstacle.updated_mono <= 0 and self._rangefinder.updated_mono <= 0:
            self._set_status_mode("idle")
            if not self._proximity_stream_seen and not self._rangefinder_stream_seen:
                self._lbl_status.setText("Waiting")
            return
        if obs_age > 2.5 and rf_age > 2.5:
            self._lbl_status.setText("Stale")
            self._set_status_mode("stale")
            self._radar.set_obstacle_state(self._obstacle)
        elif obs_age <= 2.5:
            self._radar.set_obstacle_state(self._obstacle)
            if self._obstacle.active_bin_count() > 0:
                self._set_status_mode("live")

    def summary_text(self) -> tuple[str, str]:
        nearest = self._obstacle.nearest_m()
        n_text = "N/A" if nearest is None else f"{nearest:.1f} m"
        rf = self._rangefinder.current_distance_m
        if rf is None or rf < 0:
            r_text = "N/A"
        else:
            r_text = f"{rf:.1f} m ({_orientation_label(self._rangefinder.orientation)})"
        return n_text, r_text
