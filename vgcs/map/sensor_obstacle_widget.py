"""
M9: sensor visualization & obstacle detection display.

Native Qt (PySide6): unified map HUD card: labeled metrics + sensor pills + status. Its polar plot (the four sides
of B19 and the idle sweep) is a round view in a window of its own since B25 (the client, 2026-10-31: "this proximity
into hidden button"), opened with the card's "Proximity view" button (vgcs/map/proximity_window.py).
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

from PySide6.QtCore import QSize, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QFont, QFontMetrics
from PySide6.QtWidgets import (
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from vgcs.map.proximity_window import ProximityWindow
from vgcs.skydroid.protocol import format_slr_display_m

_SENSOR_LASER = 0
_SENSOR_ULTRASOUND = 1
_SENSOR_INFRARED = 2
_SENSOR_RADAR = 3
_OBSTACLE_BIN_COUNT = 72
_NO_READING_CM = 0xFFFF

_CARD_WIDTH_PX = 200
_VIEW_BUTTON_PX = 30
_PANEL_WIDTH_PX = _CARD_WIDTH_PX

_HUD_TEXT = QColor(230, 238, 252)

_VIEW_BTN_QSS = (
    "QPushButton#obstacleViewBtn {"
    " color: #dce5f5; font-size: 12px; font-weight: 600;"
    " background: rgba(36, 48, 68, 200);"
    " border: 1px solid rgba(100, 180, 255, 110);"
    " border-radius: 8px; padding: 0 6px;"
    "}"
    "QPushButton#obstacleViewBtn:hover { background: rgba(50, 66, 92, 230); }"
)

_CARD_QSS = (
    "QFrame#obstacleHudCard {"
    " background: rgba(26, 33, 45, 215);"
    " border: 1px solid rgba(80, 92, 118, 107);"
    " border-radius: 12px;"
    "}"
)

_LABEL_QSS = (
    "color: #8b95a8; font-size: 9px; font-weight: 600;"
    " letter-spacing: 0.6px; background: transparent; border: none;"
)
_VALUE_QSS = "color: #dce5f5; font-size: 15px; font-weight: 600; background: transparent; border: none;"
_VALUE_ALERT_QSS = "color: #f87171; font-size: 15px; font-weight: 600; background: transparent; border: none;"
_SUB_QSS = "color: #6b7280; font-size: 10px; font-weight: 600; background: transparent; border: none;"
_DIVIDER_QSS = "background: rgba(80, 92, 118, 80); border: none; max-height: 1px; min-height: 1px;"
_PILL_TRACK_QSS = (
    "QWidget#sensorPillTrack {"
    " background: rgba(18, 22, 30, 160);"
    " border: 1px solid rgba(80, 92, 118, 70);"
    " border-radius: 8px;"
    "}"
)
_PILL_IDLE = (
    "color: #8b95a8; font-size: 10px; font-weight: 600;"
    " background: transparent; border: none; border-radius: 6px; padding: 4px 0;"
)
_PILL_ON = (
    "color: #ecfdf5; font-size: 10px; font-weight: 600;"
    " background: rgba(52, 211, 153, 0.22);"
    " border: 1px solid rgba(52, 211, 153, 0.35);"
    " border-radius: 6px; padding: 3px 0;"
)
_LRF_BTN_QSS = (
    "QPushButton#lrfLockBtn {"
    " color: #64b4ff; font-size: 17px; font-weight: 600;"
    " background: rgba(36, 48, 68, 180);"
    " border: 1px solid rgba(100, 180, 255, 120);"
    " border-radius: 8px; padding: 0 6px; min-height: 24px; max-height: 28px;"
    "}"
    "QPushButton#lrfLockBtn:hover { background: rgba(50, 66, 92, 220); }"
    "QPushButton#lrfLockBtn:checked {"
    " color: #34d399; border-color: rgba(52, 211, 153, 160);"
    " background: rgba(32, 56, 48, 200);"
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
    # MAV_SENSOR_ORIENTATION: 0 ahead, then every 45 degrees clockwise (1 is ahead right, 2 right, ...),
    # 24 up, 25 down. It said "FWD-R" for 2 and "RIGHT" for 4 until 2026-10-27.
    names = {
        0: "FWD",
        1: "FWD-R",
        2: "RIGHT",
        3: "AFT-R",
        4: "AFT",
        5: "AFT-L",
        6: "LEFT",
        7: "FWD-L",
        25: "DOWN",
        24: "UP",
    }
    return names.get(int(orientation), f"#{int(orientation)}")


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


class _MetricBlock(QWidget):
    """Label + primary value + optional subtitle — one proximity readout column."""

    def __init__(self, title: str, *, subtitle: str = "", parent=None) -> None:
        super().__init__(parent)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(1)

        self._title = QLabel(title.upper())
        self._title.setStyleSheet(_LABEL_QSS)
        self._title.setAlignment(Qt.AlignmentFlag.AlignLeft)

        self._val = QLabel("—")
        self._val.setStyleSheet(_VALUE_QSS)
        self._val.setAlignment(Qt.AlignmentFlag.AlignLeft)
        self._val.setMinimumHeight(20)

        self._sub = QLabel(subtitle)
        self._sub.setStyleSheet(_SUB_QSS)
        self._sub.setAlignment(Qt.AlignmentFlag.AlignLeft)
        self._sub.setVisible(bool(subtitle))

        lay.addWidget(self._title)
        lay.addWidget(self._val)
        lay.addWidget(self._sub)

    def set_reading(self, value: str, *, subtitle: str = "", alert: bool = False) -> None:
        self._val.setText(value)
        self._val.setStyleSheet(_VALUE_ALERT_QSS if alert else _VALUE_QSS)
        if subtitle:
            self._sub.setText(subtitle)
            self._sub.setVisible(True)
        else:
            self._sub.clear()
            self._sub.setVisible(False)


class _LrfRangeBlock(QWidget):
    """Range column — MAVLink value or C13 lock icon until target is picked on video."""

    lock_clicked = Signal()

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, False)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(1)

        self._title = QLabel("RANGE")
        self._title.setStyleSheet(_LABEL_QSS)
        self._title.setAlignment(Qt.AlignmentFlag.AlignLeft)

        self._stack = QStackedWidget()
        self._stack.setStyleSheet("background: transparent; border: none;")

        self._btn = QPushButton("⌖")
        self._btn.setObjectName("lrfLockBtn")
        self._btn.setCheckable(True)
        self._btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._btn.setStyleSheet(_LRF_BTN_QSS)
        self._btn.setToolTip("Arm C13 LRF — click any target on video to slew and measure")
        self._btn.clicked.connect(self.lock_clicked.emit)
        btn_wrap = QWidget()
        btn_wrap.setStyleSheet("background: transparent; border: none;")
        btn_lay = QHBoxLayout(btn_wrap)
        btn_lay.setContentsMargins(0, 0, 0, 0)
        btn_lay.addWidget(self._btn, 0, Qt.AlignmentFlag.AlignLeft)
        btn_lay.addStretch(1)
        self._stack.addWidget(btn_wrap)

        val_wrap = QWidget()
        val_wrap.setStyleSheet("background: transparent; border: none;")
        val_lay = QVBoxLayout(val_wrap)
        val_lay.setContentsMargins(0, 0, 0, 0)
        val_lay.setSpacing(1)
        self._val = QLabel("—")
        self._val.setStyleSheet(_VALUE_QSS)
        self._val.setAlignment(Qt.AlignmentFlag.AlignLeft)
        self._val.setMinimumHeight(20)
        self._val.setCursor(Qt.CursorShape.PointingHandCursor)
        self._val.setToolTip("Click to unlock LRF target")
        val_lay.addWidget(self._val)
        self._stack.addWidget(val_wrap)

        self._sub = QLabel("Rangefinder")
        self._sub.setStyleSheet(_SUB_QSS)
        self._sub.setAlignment(Qt.AlignmentFlag.AlignLeft)

        lay.addWidget(self._title)
        lay.addWidget(self._stack)
        lay.addWidget(self._sub)

        self._c13_mode = False
        self._c13_state = "idle"
        self._val.mousePressEvent = self._on_val_pressed  # type: ignore[method-assign]

    def _on_val_pressed(self, event) -> None:
        if self._c13_mode and self._c13_state == "locked":
            self.lock_clicked.emit()
        if event is not None and hasattr(event, "accept"):
            event.accept()

    def set_reading(self, value: str, *, subtitle: str = "", alert: bool = False) -> None:
        """MAVLink rangefinder — show live distance."""
        self._c13_mode = False
        self._c13_state = "mavlink"
        self._btn.setChecked(False)
        self._val.setText(value)
        self._val.setStyleSheet(_VALUE_ALERT_QSS if alert else _VALUE_QSS)
        if subtitle:
            self._sub.setText(subtitle)
            self._sub.setVisible(True)
        else:
            self._sub.setText("Rangefinder")
            self._sub.setVisible(True)
        self._stack.setCurrentIndex(1)

    def set_c13_idle(self) -> None:
        self._c13_mode = True
        self._c13_state = "idle"
        self._btn.setChecked(False)
        self._btn.setText("⌖")
        self._btn.setToolTip("Arm C13 LRF — click any target on video to slew and measure")
        self._sub.setText("C13 LRF · tap to lock")
        self._sub.setVisible(True)
        self._stack.setCurrentIndex(0)

    def set_c13_armed(self) -> None:
        self._c13_mode = True
        self._c13_state = "armed"
        self._btn.setChecked(True)
        self._btn.setText("◎")
        self._btn.setToolTip(
            "Arm LRF, then click any target on video — camera slews and measures range"
        )
        self._sub.setText("Click target on video to measure")
        self._sub.setVisible(True)
        self._stack.setCurrentIndex(0)

    def set_c13_lrf_live_range_m(self, distance_m: float | None) -> None:
        """Live SLR while armed (before lock) — user aims gimbal using this readout."""
        if self._c13_state in ("locked", "locking"):
            return
        if not self._c13_mode:
            return
        self._c13_state = "armed"
        if distance_m is None:
            self._stack.setCurrentIndex(0)
            return
        try:
            cur_m = float(distance_m)
        except (TypeError, ValueError):
            return
        if cur_m < 0:
            return
        self._val.setText(format_slr_display_m(cur_m))
        self._val.setStyleSheet(_VALUE_QSS)
        self._sub.setText("Live SLR — aim gimbal, then click video")
        self._sub.setVisible(True)
        self._stack.setCurrentIndex(1)

    def set_c13_locked(self, distance_m: float, *, alert: bool = False, geo_label: str = "") -> None:
        self._c13_mode = True
        self._c13_state = "locked"
        self._btn.setChecked(False)
        self._val.setText(format_slr_display_m(distance_m))
        self._val.setStyleSheet(_VALUE_ALERT_QSS if alert else _VALUE_QSS)
        sub = str(geo_label or "").strip() or "C13 LRF · locked — tap value to unlock"
        self._sub.setText(sub)
        self._sub.setVisible(True)
        self._stack.setCurrentIndex(1)

    def set_c13_locking(self, distance_m: float | None, *, geo_label: str = "") -> None:
        """Live SLR readout while tracker settles (before lock confirmed)."""
        self._c13_mode = True
        self._c13_state = "locking"
        if distance_m is None:
            self._val.setText("…")
        else:
            self._val.setText(format_slr_display_m(distance_m))
        self._val.setStyleSheet(_VALUE_QSS)
        sub = str(geo_label or "").strip() or "C13 LRF · slewing to target…"
        self._sub.setText(sub)
        self._sub.setVisible(True)
        self._stack.setCurrentIndex(1)

    def set_c13_lock_failed(self) -> None:
        """Return to armed icon after a failed lock attempt."""
        self._c13_mode = True
        self._c13_state = "armed"
        self._btn.setChecked(True)
        self._btn.setText("◎")
        self._btn.setToolTip("Lock failed — click target on video again")
        self._sub.setText("Lock failed — click video to retry")
        self._sub.setVisible(True)
        self._stack.setCurrentIndex(0)

    def clear_c13(self) -> None:
        self._c13_mode = False
        self._c13_state = "idle"
        self.set_reading("—", subtitle="Rangefinder")


class _SensorPills(QWidget):
    """Segmented LiDAR / Radar / RF selector — active source highlighted."""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setObjectName("sensorPillTrack")
        self.setStyleSheet(_PILL_TRACK_QSS)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(3, 3, 3, 3)
        lay.setSpacing(2)

        self._lidar = QLabel("LiDAR")
        self._radar = QLabel("Radar")
        self._rf = QLabel("RF")
        self._rf.setToolTip("Rangefinder")

        pill_font = QFont("Segoe UI", 10)
        pill_font.setWeight(QFont.Weight.DemiBold)
        row_h = int(QFontMetrics(pill_font).height()) + 10
        self.setFixedHeight(row_h + 6)

        for lb in (self._lidar, self._radar, self._rf):
            lb.setFont(pill_font)
            lb.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lb.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            lb.setMinimumHeight(row_h)
            lay.addWidget(lb, 1)

        self.set_active(None)

    def set_active(self, which: str | None) -> None:
        for key, lb in (("lidar", self._lidar), ("radar", self._radar), ("rf", self._rf)):
            lb.setStyleSheet(_PILL_ON if which == key else _PILL_IDLE)


class ObstacleRadarPanel(QFrame):
    """Top-left map HUD, a single card: the round view's button + metrics + sensor pills + status."""

    c13_lrf_lock_clicked = Signal()
    # The range of the round view, chosen in its window (the main window saves it, vgcs.app.obstacle_watch).
    bar_range_chosen = Signal(float)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setObjectName("obstacleRadarPanel")
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)
        self.setAutoFillBackground(False)
        self.setStyleSheet("QFrame#obstacleRadarPanel { background: transparent; border: none; }")

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        card = QFrame()
        card.setObjectName("obstacleHudCard")
        card.setStyleSheet(_CARD_QSS)
        card.setFixedWidth(_CARD_WIDTH_PX)
        card.setToolTip(
            "MAVLink OBSTACLE_DISTANCE (LiDAR/Radar) and DISTANCE_SENSOR (rangefinder). "
            "Values appear when the autopilot publishes sensor messages."
        )
        card_lay = QVBoxLayout(card)
        card_lay.setContentsMargins(8, 8, 8, 8)
        card_lay.setSpacing(8)

        header = QLabel("PROXIMITY")
        header_font = QFont("Segoe UI", 9)
        header_font.setWeight(QFont.Weight.DemiBold)
        header.setFont(header_font)
        header.setStyleSheet(
            "color: #9fb0cc; letter-spacing: 1px; background: transparent; border: none;"
        )
        header.setAlignment(Qt.AlignmentFlag.AlignCenter)
        card_lay.addWidget(header)

        # The round view (B25): in its own window, which the user places and sizes, opened from here.
        self._view_button = QPushButton("Proximity view")
        self._view_button.setObjectName("obstacleViewBtn")
        self._view_button.setStyleSheet(_VIEW_BTN_QSS)
        self._view_button.setFixedHeight(_VIEW_BUTTON_PX)
        self._view_button.setCursor(Qt.CursorShape.PointingHandCursor)
        self._view_button.setToolTip("The obstacles round the drone, as Mission Planner shows them, in a window")
        self._view_button.clicked.connect(self.open_proximity_window)
        card_lay.addWidget(self._view_button)
        self._window: ProximityWindow | None = None

        divider = QFrame()
        divider.setStyleSheet(_DIVIDER_QSS)
        divider.setFixedHeight(1)
        card_lay.addWidget(divider)

        metrics = QGridLayout()
        metrics.setHorizontalSpacing(12)
        metrics.setVerticalSpacing(0)
        self._metric_nearest = _MetricBlock("Nearest", subtitle="LiDAR / Radar")
        self._metric_nearest.setToolTip("Shortest distance from proximity scan bins")
        self._metric_range = _LrfRangeBlock()
        self._metric_range.setToolTip(
            "C13 laser rangefinder — click icon, then pick target on video to show range"
        )
        self._metric_range.lock_clicked.connect(self.c13_lrf_lock_clicked.emit)
        metrics.addWidget(self._metric_nearest, 0, 0)
        metrics.addWidget(self._metric_range, 0, 1)
        metrics.setColumnStretch(0, 1)
        metrics.setColumnStretch(1, 1)
        card_lay.addLayout(metrics)

        self._sensor_pills = _SensorPills()
        card_lay.addWidget(self._sensor_pills)

        status_row = QWidget()
        status_row.setStyleSheet("background: transparent; border: none;")
        status_lay = QHBoxLayout(status_row)
        status_lay.setContentsMargins(2, 0, 2, 0)
        status_lay.setSpacing(6)
        self._status_dot = QLabel()
        self._status_dot.setFixedSize(8, 8)
        self._status_dot.setStyleSheet("background: #6b7280; border-radius: 4px;")
        self._lbl_status = QLabel("Waiting for sensors")
        status_font = QFont("Segoe UI", 10)
        status_font.setWeight(QFont.Weight.DemiBold)
        self._lbl_status.setFont(status_font)
        self._lbl_status.setStyleSheet("color: #9fb0cc; background: transparent; border: none;")
        self._lbl_status.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        self._lbl_status.setMinimumWidth(0)
        status_lay.addWidget(self._status_dot, 0, Qt.AlignmentFlag.AlignVCenter)
        status_lay.addWidget(self._lbl_status, 1, Qt.AlignmentFlag.AlignVCenter)
        card_lay.addWidget(status_row)

        outer.addWidget(card)

        self._obstacle = ObstacleDistanceState()
        self._rangefinder = DistanceSensorState()
        self._rangefinder_subtitle = "Rangefinder"
        # The drone's proximity sensor in 8 directions, as vgcs.app.obstacle_watch says it (or None).
        self._sector_view: dict | None = None
        self._proximity_stream_seen = False
        self._rangefinder_stream_seen = False
        self._active_sensor: str | None = None
        self._panel_connected_mono = 0.0
        self._c13_lrf_ui = False
        self._c13_lrf_armed = False
        self._c13_lrf_locked = False

        self._stale_timer = QTimer(self)
        self._stale_timer.setInterval(500)
        self._stale_timer.timeout.connect(self._refresh_stale_ui)
        self._stale_timer.start()

        self.setFixedSize(_PANEL_WIDTH_PX, self._panel_height_px())

    @staticmethod
    def _panel_height_px() -> int:
        # Card: header(16) + view button + divider + metrics(~48) + pills(~32) + status(~20) + margins/spacing
        return 8 + 16 + 8 + _VIEW_BUTTON_PX + 8 + 1 + 8 + 48 + 8 + 32 + 8 + 20 + 8

    # -- the round view's window (B25) ------------------------------------------------------
    def open_proximity_window(self) -> ProximityWindow:
        """Show the round view's window (made the first time), with what the card knows now."""
        if self._window is None:
            self._window = ProximityWindow(self.window())
            self._window.bar_range_chosen.connect(self.bar_range_chosen.emit)
        self._feed_window()
        self._window.show()
        self._window.raise_()
        self._window.activateWindow()
        return self._window

    def proximity_window(self) -> ProximityWindow | None:
        return self._window

    def _feed_window(self) -> None:
        window = self._window
        if window is None:
            return
        if self._sector_view is not None:
            window.set_view(self._sector_view)
        elif self._obstacle.active_bin_count() > 0:
            window.set_view(None)
            window.set_obstacle_bins(self._obstacle.distances_cm, self._obstacle.increment_deg,
                                     self._obstacle.angle_offset_deg, self._obstacle.max_distance_m)
        else:
            window.clear()

    def sizeHint(self) -> QSize:  # noqa: N802
        return QSize(_PANEL_WIDTH_PX, self._panel_height_px())

    def minimumSizeHint(self) -> QSize:  # noqa: N802
        return self.sizeHint()

    def _set_status_mode(self, mode: str) -> None:
        colors = {
            "idle": "#6b7280",
            "live": "#34d399",
            "stale": "#fbbf24",
            "alert": "#f87171",
        }
        c = colors.get(mode, colors["idle"])
        self._status_dot.setStyleSheet(
            f"background: {c}; border-radius: 4px; min-width:8px; max-width:8px;"
        )

    def _sync_status_line(self) -> None:
        if bool(getattr(self, "_c13_lrf_armed", False)):
            self._lbl_status.setText("Rangefinder · click video")
            self._set_status_mode("live")
            return
        if bool(getattr(self, "_c13_lrf_locked", False)):
            rf = self._rangefinder.current_distance_m
            if rf is not None and rf >= 0:
                self._lbl_status.setText("Rangefinder · locked")
                close = rf < 15.0
                self._set_status_mode("alert" if close else "live")
                return

        view = self._sector_view
        if view is not None:
            self._lbl_status.setText(str(view.get("status") or ""))
            state = str(view.get("state") or "")
            self._set_status_mode("alert" if view.get("near") or state == "fault" else "live")
            return

        now = time.monotonic()
        obs_age = now - float(self._obstacle.updated_mono or 0.0)
        rf_age = now - float(self._rangefinder.updated_mono or 0.0)
        obs_fresh = self._obstacle.updated_mono > 0 and obs_age <= 2.5
        rf_fresh = self._rangefinder.updated_mono > 0 and rf_age <= 2.5

        if not self._proximity_stream_seen and not self._rangefinder_stream_seen:
            if self._panel_connected_mono <= 0:
                self._lbl_status.setText("Waiting for sensors")
            elif time.monotonic() - self._panel_connected_mono > 8.0:
                self._lbl_status.setText("No sensor MAVLink yet")
            else:
                self._lbl_status.setText("Listening…")
            self._set_status_mode("idle")
            return

        if self._obstacle.updated_mono > 0 and self._rangefinder.updated_mono > 0:
            if obs_age > 2.5 and rf_age > 2.5:
                self._lbl_status.setText("Sensor data stale")
                self._set_status_mode("stale")
                return

        nearest = self._obstacle.nearest_m() if obs_fresh else None
        rf = self._rangefinder.current_distance_m if rf_fresh else None
        bins = self._obstacle.active_bin_count() if obs_fresh else 0

        if nearest is not None and rf is not None:
            if nearest <= rf:
                fam = _sensor_family_label(self._obstacle.sensor_type)
                self._lbl_status.setText(f"{fam} · {bins} hit{'s' if bins != 1 else ''}")
            else:
                ori = _orientation_label(self._rangefinder.orientation)
                self._lbl_status.setText(f"Rangefinder · {ori}")
            close = min(nearest, rf) < 2.0
            self._set_status_mode("alert" if close else "live")
            return

        if nearest is not None:
            fam = _sensor_family_label(self._obstacle.sensor_type)
            if bins > 0:
                self._lbl_status.setText(f"{fam} · {bins} hit{'s' if bins != 1 else ''}")
                close = nearest < max(2.0, self._obstacle.max_distance_m * 0.2)
                self._set_status_mode("alert" if close else "live")
            else:
                self._lbl_status.setText("Path clear")
                self._set_status_mode("live")
            return

        if rf is not None and rf >= 0:
            ori = _orientation_label(self._rangefinder.orientation)
            self._lbl_status.setText(f"Rangefinder · {ori}")
            close = rf < 2.0
            self._set_status_mode("alert" if close else "live")
            return

        if (self._obstacle.updated_mono > 0 and obs_age > 2.5) or (
            self._rangefinder.updated_mono > 0 and rf_age > 2.5
        ):
            self._lbl_status.setText("Sensor data stale")
            self._set_status_mode("stale")
            return

        self._lbl_status.setText("Listening")
        self._set_status_mode("live")

    def notify_link_connected(self, connected: bool) -> None:
        if connected:
            self._panel_connected_mono = time.monotonic()
        else:
            self._panel_connected_mono = 0.0
            self._proximity_stream_seen = False
            self._rangefinder_stream_seen = False
            self._sector_view = None
            self._obstacle = ObstacleDistanceState()
            self._feed_window()
            self._c13_lrf_ui = False
            self._c13_lrf_armed = False
            self._c13_lrf_locked = False
            self._metric_nearest.set_reading("—", subtitle="LiDAR / Radar")
            self._metric_range.clear_c13()
            self._sensor_pills.set_active(None)
            self._sync_status_line()

    def set_vehicle_heading_deg(self, heading_deg: float) -> None:
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
        if self._sector_view is None:
            self._feed_window()

        nearest = st.nearest_m()
        fam = _sensor_family_label(st.sensor_type)
        if nearest is not None:
            close = nearest < max(2.0, st.max_distance_m * 0.2)
            self._metric_nearest.set_reading(f"{nearest:.1f} m", subtitle=fam, alert=close)
        else:
            self._metric_nearest.set_reading("—", subtitle=fam)

        if st.sensor_type == _SENSOR_RADAR:
            self._set_sensor_active("radar")
        elif st.sensor_type == _SENSOR_LASER:
            self._set_sensor_active("lidar")
        else:
            self._set_sensor_active("lidar")

        self._sync_status_line()

    def set_proximity_view(self, view: dict | None) -> None:
        """The drone's proximity sensor (vgcs.app.obstacle_watch ``radar_view``).

        None takes the card back to OBSTACLE_DISTANCE and the waiting words. It comes several times a
        second, mostly unchanged: nothing is restyled then.
        """
        new = dict(view) if isinstance(view, dict) else None
        if new == self._sector_view:
            return
        self._sector_view = new
        self._feed_window()
        if self._sector_view is not None:
            self._proximity_stream_seen = True
            where = str(self._sector_view.get("nearest_where") or "") or "LiDAR"
            self._metric_nearest.set_reading(str(self._sector_view.get("nearest_text") or "—"), subtitle=where,
                                             alert=bool(self._sector_view.get("near")))
            if self._active_sensor != "lidar":
                self._set_sensor_active("lidar")
        self._sync_status_line()

    def set_distance_sensor(self, payload: dict) -> None:
        if bool(getattr(self, "_c13_lrf_ui", False)):
            return
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

        ori = _orientation_label(st.orientation)
        self._rangefinder_subtitle = ori
        if cur_m is not None and cur_m >= 0:
            close = cur_m < 2.0
            self._metric_range.set_reading(f"{cur_m:.1f} m", subtitle=ori, alert=close)
        else:
            self._metric_range.set_reading("—", subtitle="Rangefinder")

        # With the drone's own obstacle sensor on the card, its pill stays lit (a rangefinder beside
        # it would make the pills flicker between the two several times a second).
        if self._sector_view is None:
            self._set_sensor_active("rf")
        self._sync_status_line()

    def enable_c13_lrf_ui(self, enabled: bool = True) -> None:
        """Show C13 LRF lock icon instead of live distance until target is locked on video."""
        self._c13_lrf_ui = bool(enabled)
        if not self._c13_lrf_ui:
            self._c13_lrf_armed = False
            self._c13_lrf_locked = False
            self._metric_range.clear_c13()
            self._sync_status_line()
            return
        if not self._c13_lrf_locked:
            self._metric_range.set_c13_idle()
            self._set_sensor_active("rf")
            self._sync_status_line()

    def set_c13_lrf_lock_failed(self) -> None:
        """Restore armed icon after failed lock — keeps ⌖ visible for retry."""
        self._c13_lrf_armed = True
        self._c13_lrf_locked = False
        if not self._c13_lrf_ui:
            return
        self._metric_range.set_c13_lock_failed()
        self._set_sensor_active("rf")
        self._lbl_status.setText("Rangefinder · lock failed")
        self._set_status_mode("live")
        self._sync_status_line()

    def set_c13_lrf_armed(self, armed: bool) -> None:
        self._c13_lrf_armed = bool(armed)
        if not self._c13_lrf_ui:
            return
        if self._c13_lrf_armed:
            self._metric_range.set_c13_armed()
        elif not self._c13_lrf_locked:
            self._metric_range.set_c13_idle()
        self._set_sensor_active("rf")
        self._sync_status_line()

    def set_c13_lrf_locked(
        self,
        distance_m: float | None,
        *,
        geo_label: str = "",
    ) -> None:
        if distance_m is None:
            self._c13_lrf_locked = False
            self._c13_lrf_armed = False
            if self._c13_lrf_ui:
                self._metric_range.set_c13_idle()
            self._rangefinder = DistanceSensorState()
            self._sync_status_line()
            return
        try:
            cur_m = float(distance_m)
        except (TypeError, ValueError):
            return
        self._c13_lrf_locked = True
        self._c13_lrf_armed = False
        st = DistanceSensorState(
            sensor_type=_SENSOR_LASER,
            orientation=0,
            current_distance_m=cur_m,
            min_distance_m=5.0,
            max_distance_m=1000.0,
            updated_mono=time.monotonic(),
        )
        self._rangefinder = st
        self._rangefinder_subtitle = "C13 LRF"
        self._rangefinder_stream_seen = True
        close = cur_m < 15.0
        self._metric_range.set_c13_locked(
            cur_m, alert=close, geo_label=str(geo_label or "")
        )
        self._set_sensor_active("rf")
        self._sync_status_line()

    def set_c13_lrf_locking(
        self,
        distance_m: float | None,
        *,
        geo_label: str = "",
    ) -> None:
        """Show live range in PROXIMITY while LRF lock is in progress."""
        if not self._c13_lrf_ui:
            return
        try:
            cur_m = float(distance_m) if distance_m is not None else None
        except (TypeError, ValueError):
            cur_m = None
        self._metric_range.set_c13_locking(cur_m, geo_label=str(geo_label or ""))
        self._set_sensor_active("rf")
        self._lbl_status.setText("Rangefinder · locking")
        self._set_status_mode("live")

    def set_companion_lrf_range_m(self, distance_m: float | None) -> None:
        """C13 TOP SLR — live while armed, frozen when locked."""
        if bool(getattr(self, "_c13_lrf_locked", False)):
            try:
                cur_m = None if distance_m is None else float(distance_m)
            except (TypeError, ValueError):
                cur_m = None
            if cur_m is None or cur_m < 0:
                return
            self.set_c13_lrf_locked(cur_m)
            return
        if bool(getattr(self, "_c13_lrf_armed", False)):
            self.set_c13_lrf_live_range_m(distance_m)

    def _set_sensor_active(self, which: str | None) -> None:
        self._active_sensor = which
        self._sensor_pills.set_active(which)

    def _refresh_stale_ui(self) -> None:
        if self._obstacle.updated_mono <= 0 and self._rangefinder.updated_mono <= 0:
            return
        self._sync_status_line()

    def summary_text(self) -> tuple[str, str]:
        nearest = self._obstacle.nearest_m()
        n_text = "N/A" if nearest is None else f"{nearest:.1f} m"
        view = self._sector_view
        if view is not None:
            where = str(view.get("nearest_where") or "")
            n_text = str(view.get("nearest_text") or "N/A") + (f" {where.lower()}" if where else "")
        rf = self._rangefinder.current_distance_m
        if rf is None or rf < 0:
            r_text = "N/A"
        else:
            sub = str(getattr(self, "_rangefinder_subtitle", "Rangefinder") or "Rangefinder")
            r_text = f"{rf:.1f} m ({sub})"
        return n_text, r_text
