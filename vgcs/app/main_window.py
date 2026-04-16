"""Main application window — telemetry dashboard."""

from __future__ import annotations

import time

from PySide6.QtCore import Qt, QSettings, QTimer
from PySide6.QtGui import QGuiApplication
from PySide6.QtWidgets import (
    QBoxLayout,
    QComboBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QDoubleSpinBox,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
from pymavlink import mavutil

from vgcs.app.widgets import CompassWidget
from vgcs.link.mavlink_thread import MavlinkThread


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("VGCS — Ground Control Station")
        self.resize(1024, 700)
        self.setMinimumSize(820, 560)

        self._settings = QSettings("VGCS", "VGCS")
        self._thread: MavlinkThread | None = None
        self._timeout_s = float(self._settings.value("watchdog_timeout_s", 2.0))
        self._armed_since: float | None = None
        self._theme_name = str(self._settings.value("ui_theme", "Default"))
        self._theme_colors = self._build_theme_colors(self._theme_name)
        self._compact_ui = self._detect_compact_ui()

        self._conn_label = QLabel("MAVLink connection string")
        self._conn_edit = QLineEdit()
        self._conn_edit.setText(
            str(self._settings.value("last_connection_string", "udp:127.0.0.1:14550"))
        )
        self._timeout_label = QLabel("Watchdog timeout (s)")
        self._timeout_spin = QDoubleSpinBox()
        self._timeout_spin.setRange(1.0, 10.0)
        self._timeout_spin.setSingleStep(0.5)
        self._timeout_spin.setValue(self._timeout_s)
        self._theme_label = QLabel("State color theme")
        self._theme_combo = QComboBox()
        self._theme_combo.addItems(["Default", "High Contrast", "Dark Friendly"])
        self._theme_combo.setCurrentText(self._theme_name)

        self._btn_connect = QPushButton("Connect")
        self._btn_disconnect = QPushButton("Disconnect")
        self._btn_reset = QPushButton("Reset telemetry")
        self._btn_restore_defaults = QPushButton("Restore defaults")
        self._btn_disconnect.setEnabled(False)

        self._status, self._status_frame = self._make_status_chip("Link", "Disconnected")
        self._hb, self._hb_frame = self._make_status_chip("Heartbeat", "—")
        self._watchdog, self._watchdog_frame = self._make_status_chip(
            "Watchdog", f"Idle · {self._timeout_s:.1f}s"
        )
        self._apply_state_style(self._status, "bad")
        self._apply_state_style(self._hb, "na")
        self._apply_state_style(self._watchdog, "warn")

        self._compass = CompassWidget()
        self._telemetry_body = self._build_telemetry_panel()

        self._log = QTextEdit()
        self._log.setReadOnly(True)
        self._log.setPlaceholderText("MAVLink log…")
        self._log.setMinimumHeight(150)

        self._link_grid = QGridLayout()
        self._link_grid.setVerticalSpacing(6 if self._compact_ui else 8)
        self._link_grid.setColumnStretch(1, 1)

        self._btn_grid = QGridLayout()
        self._btn_grid.setHorizontalSpacing(6 if self._compact_ui else 8)
        self._btn_grid.setVerticalSpacing(6 if self._compact_ui else 8)

        link_box = QGroupBox("Connection")
        link_inner = QVBoxLayout()
        link_inner.addLayout(self._link_grid)
        link_inner.addLayout(self._btn_grid)
        link_box.setLayout(link_inner)
        self._link_box = link_box

        self._status_row = QHBoxLayout()
        self._status_row.setSpacing(8 if self._compact_ui else 12)
        self._status_row.addWidget(self._status_frame, 1)
        self._status_row.addWidget(self._hb_frame, 1)
        self._status_row.addWidget(self._watchdog_frame, 1)

        self._dash_row = QBoxLayout(QBoxLayout.LeftToRight)
        self._dash_row.setSpacing(8 if self._compact_ui else 12)
        self._dash_row.addWidget(self._telemetry_body, 1)
        right_col = QVBoxLayout()
        compass_title = QLabel("Compass")
        compass_title.setStyleSheet("color: #7d869c; font-weight: 600; font-size: 12px;")
        right_col.addWidget(compass_title)
        right_col.addWidget(self._compass, 0, Qt.AlignHCenter)
        right_col.addStretch()
        self._dash_row.addLayout(right_col, 0)

        content_panel = QWidget()
        self._content_layout = QVBoxLayout()
        self._content_layout.setSpacing(8 if self._compact_ui else 12)
        self._content_layout.addWidget(self._build_header_bar())
        self._content_layout.addWidget(link_box)
        self._content_layout.addLayout(self._status_row)
        self._content_layout.addLayout(self._dash_row)
        self._content_layout.addWidget(self._log)
        self._content_layout.setContentsMargins(0, 0, 0, 0)
        content_panel.setLayout(self._content_layout)

        self._scroll = QScrollArea()
        self._scroll.setWidgetResizable(True)
        self._scroll.setFrameShape(QFrame.NoFrame)
        self._scroll.setWidget(content_panel)

        central = QWidget()
        central.setObjectName("centralRoot")
        layout = QVBoxLayout()
        layout.addWidget(self._scroll)
        margin = 8 if self._compact_ui else 14
        layout.setContentsMargins(margin, margin, margin, margin)
        central.setLayout(layout)
        self.setCentralWidget(central)

        self._btn_connect.clicked.connect(self._on_connect)
        self._btn_disconnect.clicked.connect(self._on_disconnect)
        self._btn_reset.clicked.connect(self._on_reset_telemetry)
        self._btn_restore_defaults.clicked.connect(self._on_restore_defaults)
        self._timeout_spin.valueChanged.connect(self._on_timeout_changed)
        self._theme_combo.currentTextChanged.connect(self._on_theme_changed)

        self._flight_timer = QTimer(self)
        self._flight_timer.setInterval(1000)
        self._flight_timer.timeout.connect(self._on_flight_timer_tick)
        self._flight_timer.start()
        self._restore_window_geometry()
        self._fit_to_screen()
        self._apply_responsive_layout(self.width())

    def _detect_compact_ui(self) -> bool:
        screen = QGuiApplication.primaryScreen()
        if screen is None:
            return False
        area = screen.availableGeometry()
        return area.height() <= 800 or area.width() <= 1366

    def _apply_responsive_layout(self, width: int) -> None:
        narrow = width < 1120

        while self._link_grid.count():
            self._link_grid.takeAt(0)
        while self._btn_grid.count():
            self._btn_grid.takeAt(0)

        if narrow:
            self._link_grid.addWidget(self._conn_label, 0, 0)
            self._link_grid.addWidget(self._conn_edit, 1, 0, 1, 4)
            self._link_grid.addWidget(self._timeout_label, 2, 0)
            self._link_grid.addWidget(self._timeout_spin, 2, 1)
            self._link_grid.addWidget(self._theme_label, 2, 2)
            self._link_grid.addWidget(self._theme_combo, 2, 3)

            self._btn_grid.addWidget(self._btn_connect, 0, 0)
            self._btn_grid.addWidget(self._btn_disconnect, 0, 1)
            self._btn_grid.addWidget(self._btn_reset, 1, 0)
            self._btn_grid.addWidget(self._btn_restore_defaults, 1, 1)
            self._btn_grid.setColumnStretch(0, 1)
            self._btn_grid.setColumnStretch(1, 1)
            self._dash_row.setDirection(QBoxLayout.TopToBottom)
        else:
            self._link_grid.addWidget(self._conn_label, 0, 0)
            self._link_grid.addWidget(self._conn_edit, 0, 1, 1, 3)
            self._link_grid.addWidget(self._timeout_label, 1, 0)
            self._link_grid.addWidget(self._timeout_spin, 1, 1)
            self._link_grid.addWidget(self._theme_label, 1, 2)
            self._link_grid.addWidget(self._theme_combo, 1, 3)

            self._btn_grid.addWidget(self._btn_connect, 0, 0)
            self._btn_grid.addWidget(self._btn_disconnect, 0, 1)
            self._btn_grid.addWidget(self._btn_reset, 0, 2)
            self._btn_grid.addWidget(self._btn_restore_defaults, 0, 3)
            self._btn_grid.setColumnStretch(0, 1)
            self._btn_grid.setColumnStretch(1, 1)
            self._btn_grid.setColumnStretch(2, 1)
            self._btn_grid.setColumnStretch(3, 1)
            self._dash_row.setDirection(QBoxLayout.LeftToRight)

    def _make_value_label(self) -> QLabel:
        lab = QLabel("—")
        lab.setObjectName("telemetryValue")
        lab.setTextInteractionFlags(Qt.TextSelectableByMouse)
        lab.setMinimumWidth(120)
        return lab

    def _make_status_chip(self, title: str, initial: str) -> tuple[QLabel, QFrame]:
        frame = QFrame()
        frame.setObjectName("statusChip")
        frame.setMinimumWidth(120 if self._compact_ui else 180)
        lay = QVBoxLayout()
        lay.setSpacing(4)
        t = QLabel(title)
        t.setObjectName("statusChipTitle")
        v = QLabel(initial)
        v.setObjectName("statusChipValue")
        v.setWordWrap(False)
        lay.addWidget(t)
        lay.addWidget(v)
        frame.setLayout(lay)
        return v, frame

    def _build_header_bar(self) -> QFrame:
        bar = QFrame()
        bar.setObjectName("headerBar")
        bar.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        h = QHBoxLayout()
        h.setContentsMargins(12, 8, 12, 8)
        left = QVBoxLayout()
        title = QLabel("VGCS")
        title.setObjectName("headerTitle")
        sub = QLabel("Live MAVLink telemetry and link status.")
        sub.setObjectName("headerSubtitle")
        sub.setWordWrap(True)
        left.addWidget(title)
        left.addWidget(sub)
        h.addLayout(left, 1)
        bar.setLayout(h)
        return bar

    def _build_telemetry_panel(self) -> QWidget:
        self._fields = {}

        def add_field(key: str) -> QLabel:
            lab = self._make_value_label()
            self._fields[key] = lab
            return lab

        primary = QGroupBox("Primary flight data")
        pg = QGridLayout()
        pg.setHorizontalSpacing(12 if self._compact_ui else 16)
        pg.setVerticalSpacing(6 if self._compact_ui else 8)
        r = 0

        def row_pair(
            r_: int,
            t1: str,
            k1: str,
            t2: str,
            k2: str,
        ) -> int:
            l1 = QLabel(t1)
            l1.setStyleSheet("color: #7d869c;")
            l2 = QLabel(t2)
            l2.setStyleSheet("color: #7d869c;")
            pg.addWidget(l1, r_, 0)
            pg.addWidget(add_field(k1), r_, 1)
            pg.addWidget(l2, r_, 2)
            pg.addWidget(add_field(k2), r_, 3)
            return r_ + 1

        r = row_pair(r, "Armed", "armed", "Flight time", "flight_time")
        r = row_pair(r, "Ground speed", "groundspeed", "Air speed", "airspeed")
        r = row_pair(r, "Altitude (rel)", "alt_rel", "Altitude (MSL)", "alt_msl")
        ll = QLabel("Lat / Lon")
        ll.setStyleSheet("color: #7d869c;")
        pg.addWidget(ll, r, 0)
        lat_w = add_field("lat_lon")
        pg.addWidget(lat_w, r, 1, 1, 3)
        r += 1
        r = row_pair(r, "Heading", "heading", "Attitude (R/P/Y)", "attitude")
        primary.setLayout(pg)

        systems = QGroupBox("Navigation & systems")
        sg = QGridLayout()
        sg.setHorizontalSpacing(12 if self._compact_ui else 16)
        sg.setVerticalSpacing(6 if self._compact_ui else 8)
        sr = 0

        def row_sys(a: str, ka: str, b: str, kb: str) -> None:
            nonlocal sr
            la = QLabel(a)
            la.setStyleSheet("color: #7d869c;")
            lb = QLabel(b)
            lb.setStyleSheet("color: #7d869c;")
            sg.addWidget(la, sr, 0)
            sg.addWidget(add_field(ka), sr, 1)
            sg.addWidget(lb, sr, 2)
            sg.addWidget(add_field(kb), sr, 3)
            sr += 1

        row_sys("GPS", "gps", "Battery", "battery")
        row_sys("RC link", "rc_link", "Video link", "video_link")
        row_sys("Battery failsafe", "failsafe_battery", "RC failsafe", "failsafe_rc")
        la = QLabel("Arm readiness")
        la.setStyleSheet("color: #7d869c;")
        sg.addWidget(la, sr, 0)
        sg.addWidget(add_field("arm_ready"), sr, 1, 1, 3)
        systems.setLayout(sg)

        self._fields["video_link"].setText("N/A")
        self._fields["arm_ready"].setText("Best-effort from telemetry")
        self._apply_state_style(self._fields["video_link"], "na")

        col = QWidget()
        v = QVBoxLayout()
        v.setSpacing(8 if self._compact_ui else 12)
        v.addWidget(primary)
        v.addWidget(systems)
        col.setLayout(v)
        return col

    def _apply_state_style(self, label: QLabel, state: str) -> None:
        label.setProperty("state_role", state)
        colors = self._theme_colors
        if state == "ok":
            label.setStyleSheet(f"color: {colors['ok']}; font-weight: 600;")
        elif state == "warn":
            label.setStyleSheet(f"color: {colors['warn']}; font-weight: 600;")
        elif state == "bad":
            label.setStyleSheet(f"color: {colors['bad']}; font-weight: 600;")
        elif state == "na":
            label.setStyleSheet(f"color: {colors['na']};")
        else:
            label.setProperty("state_role", "")
            label.setStyleSheet("")

    def _build_theme_colors(self, theme_name: str) -> dict[str, str]:
        themes = {
            "Default": {
                "ok": "#1b7f3b",
                "warn": "#b45f06",
                "bad": "#b00020",
                "na": "#666666",
            },
            "High Contrast": {
                "ok": "#0b7a0b",
                "warn": "#cc5500",
                "bad": "#d10000",
                "na": "#404040",
            },
            "Dark Friendly": {
                "ok": "#6ee7b7",
                "warn": "#fbbf24",
                "bad": "#f87171",
                "na": "#9ca3af",
            },
        }
        return themes.get(theme_name, themes["Default"])

    def _all_state_labels(self) -> list[QLabel]:
        return [self._status, self._hb, self._watchdog, *self._fields.values()]

    def _refresh_state_styles(self) -> None:
        for label in self._all_state_labels():
            state = str(label.property("state_role") or "")
            self._apply_state_style(label, state)

    def _set_ok_warn_field(self, key: str, is_ok: bool, ok_text: str = "OK") -> None:
        label = self._fields[key]
        if is_ok:
            label.setText(ok_text)
            self._apply_state_style(label, "ok")
        else:
            label.setText("WARN")
            self._apply_state_style(label, "bad")

    def _append_log(self, line: str) -> None:
        self._log.append(line)
        self._log.verticalScrollBar().setValue(self._log.verticalScrollBar().maximum())

    def _reset_telemetry_fields(self) -> None:
        self._armed_since = None
        self._fields["armed"].setText("No")
        self._apply_state_style(self._fields["armed"], "warn")
        self._fields["flight_time"].setText("00:00")
        self._fields["lat_lon"].setText("—")
        self._fields["alt_rel"].setText("—")
        self._fields["alt_msl"].setText("—")
        self._fields["groundspeed"].setText("—")
        self._fields["airspeed"].setText("—")
        self._fields["heading"].setText("—")
        self._fields["attitude"].setText("—")
        self._fields["gps"].setText("—")
        self._fields["battery"].setText("—")
        self._fields["rc_link"].setText("—")
        self._fields["failsafe_battery"].setText("—")
        self._fields["failsafe_rc"].setText("—")
        self._fields["arm_ready"].setText("Best-effort from telemetry")
        self._fields["video_link"].setText("N/A")
        self._apply_state_style(self._fields["video_link"], "na")
        self._apply_state_style(self._fields["failsafe_battery"], "")
        self._apply_state_style(self._fields["failsafe_rc"], "")
        self._apply_state_style(self._fields["arm_ready"], "")
        self._apply_state_style(self._fields["rc_link"], "")

    def _on_connect(self) -> None:
        if self._thread is not None and self._thread.isRunning():
            QMessageBox.warning(self, "VGCS", "Already connected.")
            return

        cs = self._conn_edit.text().strip()
        if not cs:
            QMessageBox.warning(self, "VGCS", "Enter a connection string.")
            return

        self._timeout_s = float(self._timeout_spin.value())
        self._settings.setValue("watchdog_timeout_s", self._timeout_s)
        self._settings.setValue("last_connection_string", cs)
        self._thread = MavlinkThread(cs, timeout_s=self._timeout_s)
        self._thread.log_line.connect(self._append_log)
        self._thread.error.connect(self._on_link_error)
        self._thread.link_up.connect(self._on_link_up)
        self._thread.link_down.connect(self._on_link_down)
        self._thread.heartbeat.connect(self._on_heartbeat)
        self._thread.telemetry.connect(self._on_telemetry)
        self._thread.link_timeout.connect(self._on_link_timeout)
        self._thread.finished.connect(self._on_thread_finished)

        self._btn_connect.setEnabled(False)
        self._conn_edit.setEnabled(False)
        self._timeout_spin.setEnabled(False)
        self._theme_combo.setEnabled(False)
        self._btn_reset.setEnabled(False)
        self._status.setText("Connecting…")
        self._apply_state_style(self._status, "warn")
        self._thread.start()

    def _on_disconnect(self) -> None:
        if self._thread is not None:
            self._thread.stop()
            if self._thread.isRunning():
                self._thread.wait(3000)

    def _on_link_up(self) -> None:
        self._status.setText("Connected")
        self._apply_state_style(self._status, "ok")
        self._btn_disconnect.setEnabled(True)
        self._watchdog.setText(f"OK · {self._timeout_s:.1f}s")
        self._apply_state_style(self._watchdog, "ok")

    def _on_link_down(self) -> None:
        self._status.setText("Disconnected")
        self._apply_state_style(self._status, "bad")
        self._hb.setText("—")
        self._apply_state_style(self._hb, "na")
        self._watchdog.setText(f"Idle · {self._timeout_s:.1f}s")
        self._apply_state_style(self._watchdog, "warn")
        self._compass.clear()
        self._btn_connect.setEnabled(True)
        self._conn_edit.setEnabled(True)
        self._timeout_spin.setEnabled(True)
        self._theme_combo.setEnabled(True)
        self._btn_reset.setEnabled(True)
        self._btn_disconnect.setEnabled(False)
        self._reset_telemetry_fields()

    def _on_heartbeat(self, sysid: int, compid: int, mav_ver: int) -> None:
        self._hb.setText(f"sys {sysid} · comp {compid} · mav {mav_ver}")
        self._apply_state_style(self._hb, "ok")

    def _on_telemetry(self, msg_type: str, payload: object) -> None:
        data = payload if isinstance(payload, dict) else {}
        if msg_type == "HEARTBEAT":
            armed = bool(data.get("armed", False))
            self._fields["armed"].setText("Yes" if armed else "No")
            self._apply_state_style(self._fields["armed"], "ok" if armed else "warn")
            if armed and self._armed_since is None:
                self._armed_since = time.monotonic()
            if not armed:
                self._armed_since = None
                self._fields["flight_time"].setText("00:00")
            system_status = int(data.get("system_status", 0))
            arm_ready = system_status >= 3
            self._fields["arm_ready"].setText("Likely ready" if arm_ready else f"System status {system_status}")
            self._apply_state_style(self._fields["arm_ready"], "ok" if arm_ready else "warn")
        elif msg_type == "GLOBAL_POSITION_INT":
            self._fields["lat_lon"].setText(
                f"{data.get('lat', 0.0):.7f}, {data.get('lon', 0.0):.7f}"
            )
            self._fields["alt_rel"].setText(f"{data.get('relative_alt_m', 0.0):.1f} m")
            self._fields["alt_msl"].setText(f"{data.get('alt_msl_m', 0.0):.1f} m")
        elif msg_type == "VFR_HUD":
            self._fields["groundspeed"].setText(f"{data.get('groundspeed', 0.0):.1f} m/s")
            self._fields["airspeed"].setText(f"{data.get('airspeed', 0.0):.1f} m/s")
            hd = float(data.get("heading", 0.0))
            self._fields["heading"].setText(f"{int(hd)}°")
            self._compass.set_heading_deg(hd)
        elif msg_type == "ATTITUDE":
            self._fields["attitude"].setText(
                f"{data.get('roll_deg', 0.0):.1f} / "
                f"{data.get('pitch_deg', 0.0):.1f} / "
                f"{data.get('yaw_deg', 0.0):.1f} deg"
            )
            yaw_deg = float(data.get("yaw_deg", 0.0))
            self._compass.set_heading_deg((yaw_deg + 360.0) % 360.0)
        elif msg_type == "GPS_RAW_INT":
            hdop = data.get("hdop")
            hdop_text = "N/A" if hdop is None else f"{hdop:.2f}"
            self._fields["gps"].setText(
                f"fix={int(data.get('fix_type', 0))} sat={int(data.get('satellites_visible', 0))} hdop={hdop_text}"
            )
        elif msg_type == "SYS_STATUS":
            pct = int(data.get("battery_remaining", -1))
            pct_text = "N/A" if pct < 0 else f"{pct}%"
            voltage = float(data.get("voltage_v", 0.0))
            current = float(data.get("current_a", -1.0))
            current_text = "N/A" if current < 0 else f"{current:.1f} A"
            self._fields["battery"].setText(
                f"{voltage:.2f} V, {current_text}, {pct_text}"
            )
            sensors_present = int(data.get("sensors_present", 0))
            sensors_enabled = int(data.get("sensors_enabled", 0))
            sensors_health = int(data.get("sensors_health", 0))
            battery_mask = int(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_BATTERY)
            rc_mask = int(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_RC_RECEIVER)
            battery_monitored = bool(sensors_present & battery_mask and sensors_enabled & battery_mask)
            rc_monitored = bool(sensors_present & rc_mask and sensors_enabled & rc_mask)
            battery_healthy = bool(sensors_health & battery_mask)
            rc_healthy = bool(sensors_health & rc_mask)
            battery_ok = (not battery_monitored or battery_healthy) and (pct < 0 or pct > 15)
            self._set_ok_warn_field("failsafe_battery", battery_ok)
            if rc_monitored:
                self._set_ok_warn_field("failsafe_rc", rc_healthy)
            else:
                self._fields["failsafe_rc"].setText("N/A")
                self._apply_state_style(self._fields["failsafe_rc"], "na")
        elif msg_type == "RADIO_STATUS":
            self._fields["rc_link"].setText(
                f"rssi={int(data.get('rssi', 0))} remrssi={int(data.get('remrssi', 0))}"
            )
            self._apply_state_style(self._fields["rc_link"], "ok")

    def _on_link_timeout(self, elapsed_s: float) -> None:
        self._watchdog.setText(f"Lost · {elapsed_s:.1f}s no MAVLink")
        self._apply_state_style(self._watchdog, "bad")
        self._append_log(
            f"GCS link watchdog triggered: no messages for {elapsed_s:.1f}s"
        )

    def _on_timeout_changed(self, value: float) -> None:
        self._timeout_s = float(value)
        self._settings.setValue("watchdog_timeout_s", self._timeout_s)
        self._watchdog.setText(f"OK · {self._timeout_s:.1f}s")
        self._apply_state_style(self._watchdog, "ok")

    def _on_reset_telemetry(self) -> None:
        self._hb.setText("—")
        self._apply_state_style(self._hb, "na")
        self._watchdog.setText(f"OK · {self._timeout_s:.1f}s")
        self._apply_state_style(self._watchdog, "ok")
        self._compass.clear()
        self._reset_telemetry_fields()
        self._append_log("Telemetry fields reset.")

    def _on_theme_changed(self, theme_name: str) -> None:
        self._theme_name = theme_name
        self._theme_colors = self._build_theme_colors(theme_name)
        self._settings.setValue("ui_theme", theme_name)
        self._refresh_state_styles()

    def _restore_window_geometry(self) -> None:
        screen = QGuiApplication.primaryScreen()
        area = screen.availableGeometry() if screen is not None else None
        geometry = self._settings.value("window_geometry")
        if geometry is not None:
            self.restoreGeometry(geometry)
        else:
            if area is None:
                self.resize(1024, 700)
            else:
                self.resize(min(1024, area.width() - 20), min(700, area.height() - 20))
        if self.width() < 820 or self.height() < 560:
            self.resize(920, 620)

    def _fit_to_screen(self) -> None:
        screen = QGuiApplication.primaryScreen()
        if screen is None:
            return
        area = screen.availableGeometry()
        max_w = max(820, area.width() - 12)
        max_h = max(560, area.height() - 12)
        target_w = min(self.width(), max_w)
        target_h = min(self.height(), max_h)
        if target_w != self.width() or target_h != self.height():
            self.resize(target_w, target_h)

    def resizeEvent(self, event) -> None:  # noqa: N802
        super().resizeEvent(event)
        self._apply_responsive_layout(event.size().width())

    def _on_restore_defaults(self) -> None:
        if self._thread is not None and self._thread.isRunning():
            QMessageBox.information(
                self, "VGCS", "Disconnect before restoring defaults."
            )
            return
        self._conn_edit.setText("udp:127.0.0.1:14550")
        self._timeout_spin.setValue(2.0)
        self._theme_combo.setCurrentText("Default")
        self._settings.setValue("last_connection_string", "udp:127.0.0.1:14550")
        self._settings.setValue("watchdog_timeout_s", 2.0)
        self._settings.setValue("ui_theme", "Default")
        self._append_log("UI defaults restored.")

    def _on_flight_timer_tick(self) -> None:
        if self._armed_since is None:
            return
        elapsed = int(time.monotonic() - self._armed_since)
        mm = elapsed // 60
        ss = elapsed % 60
        self._fields["flight_time"].setText(f"{mm:02d}:{ss:02d}")

    def _on_link_error(self, text: str) -> None:
        self._append_log(f"Error: {text}")

    def _on_thread_finished(self) -> None:
        self._thread = None

    def closeEvent(self, event) -> None:  # noqa: N802 — Qt API
        self._on_disconnect()
        self._settings.setValue("window_geometry", self.saveGeometry())
        super().closeEvent(event)
