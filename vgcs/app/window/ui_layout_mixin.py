"""MainWindow mixin — see vgcs.app.window package."""

from __future__ import annotations

import math
import time
from collections import deque
from pathlib import Path

from PySide6.QtCore import QEvent, QPoint, QSize, Qt, QSettings, QTimer
from PySide6.QtGui import (
    QColor,
    QGuiApplication,
    QIcon,
    QImage,
    QImageReader,
    QKeySequence,
    QPainter,
    QPixmap,
    QShortcut,
)
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDialog,
    QDoubleSpinBox,
    QFrame,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QInputDialog,
    QMenu,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QScrollBar,
    QSizePolicy,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
    QListWidget,
    QListWidgetItem,
    QStackedWidget,
    QSpinBox,
    QStyle,
    QTextEdit,
    QTabWidget,
    QRadioButton,
    QButtonGroup,
    QFileDialog,
)
from pymavlink import mavutil

from vgcs.app.window.helpers import (
    _mavlink_autopilot_label,
    _mavlink_vehicle_type_label,
    _settings_truthy,
)
from vgcs.app.gcs_style import gcs_stylesheet
from vgcs.app.runtime_ui import build_base_font, select_font_profile
from vgcs.mode import AP_COPTER_MODE_MAP, human_mode_name, modes_for_vehicle_type
from vgcs.mission import Waypoint
from vgcs.map import MapWidget
from vgcs.map.map_web_3d import HAS_WEBENGINE as HAS_MAP_WEBENGINE
from vgcs.app.widgets import CompassWidget
from vgcs.link.mavlink_thread import MavlinkThread
from vgcs.video.pipeline import VideoPipeline
from vgcs.video.widgets import CameraControlPanel
from vgcs.video.camera_control import (
    CompositeGimbalCameraControl,
    MavlinkCameraControl,
    NoopCameraControl,
    read_companion_laser_range_m,
    poll_companion_laser_range_m,
    SiyiCameraControl,
    SkydroidCameraControl,
    resolve_siyi_host,
    resolve_skydroid_control_hosts,
    resolve_skydroid_host,
)


class MainWindowUiLayoutMixin:
    """Extracted from MainWindow — uses host state via self."""

    def _wire_camera_control(self, cc: object) -> None:
        wrapped = CompositeGimbalCameraControl(cc, self._thread)
        self._camera_control_backend = wrapped
        self._map_widget.set_camera_control(wrapped)
        try:
            self._camera_panel.set_camera_control(wrapped)
        except Exception:
            pass

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
            self._link_grid.addWidget(self._mode_label, 3, 0)
            self._link_grid.addWidget(self._mode_combo, 3, 1, 1, 2)
            self._link_grid.addWidget(self._btn_set_mode, 3, 3)

            self._btn_grid.addWidget(self._btn_connect, 0, 0)
            self._btn_grid.addWidget(self._btn_disconnect, 0, 1)
            self._btn_grid.addWidget(self._btn_reset, 1, 0)
            self._btn_grid.addWidget(self._btn_restore_defaults, 1, 1)
            self._btn_grid.setColumnStretch(0, 1)
            self._btn_grid.setColumnStretch(1, 1)
            self._center_row.setDirection(QBoxLayout.TopToBottom)
            self._footer_row.setDirection(QBoxLayout.TopToBottom)
            self._after_responsive_layout_changed()
        else:
            self._link_grid.addWidget(self._conn_label, 0, 0)
            self._link_grid.addWidget(self._conn_edit, 0, 1, 1, 3)
            self._link_grid.addWidget(self._timeout_label, 1, 0)
            self._link_grid.addWidget(self._timeout_spin, 1, 1)
            self._link_grid.addWidget(self._theme_label, 1, 2)
            self._link_grid.addWidget(self._theme_combo, 1, 3)
            self._link_grid.addWidget(self._mode_label, 2, 0)
            self._link_grid.addWidget(self._mode_combo, 2, 1, 1, 2)
            self._link_grid.addWidget(self._btn_set_mode, 2, 3)

            self._btn_grid.addWidget(self._btn_connect, 0, 0)
            self._btn_grid.addWidget(self._btn_disconnect, 0, 1)
            self._btn_grid.addWidget(self._btn_reset, 0, 2)
            self._btn_grid.addWidget(self._btn_restore_defaults, 0, 3)
            self._btn_grid.setColumnStretch(0, 1)
            self._btn_grid.setColumnStretch(1, 1)
            self._btn_grid.setColumnStretch(2, 1)
            self._btn_grid.setColumnStretch(3, 1)
            self._center_row.setDirection(QBoxLayout.LeftToRight)
            self._footer_row.setDirection(QBoxLayout.LeftToRight)
            self._after_responsive_layout_changed()

    def _after_responsive_layout_changed(self) -> None:
        if self._map_only_dashboard and self._plan_flight_layer_wanted:
            def _pin() -> None:
                self._scroll.verticalScrollBar().setValue(0)
                self._map_widget.set_plan_flight_visible(True)

            QTimer.singleShot(0, _pin)

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

    def _make_top_chip(self, title: str, initial: str = "—") -> tuple[QLabel, QFrame]:
        frame = QFrame()
        frame.setObjectName("statusChip")
        min_w = 128 if self._compact_ui else 154
        frame.setMinimumWidth(min_w)
        lay = QVBoxLayout()
        lay.setContentsMargins(10, 8, 10, 8)
        lay.setSpacing(4)
        t = QLabel(title)
        t.setObjectName("statusChipTitle")
        t.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        v = QLabel(initial)
        v.setObjectName("statusChipValue")
        v.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        v.setWordWrap(False)
        lay.addWidget(t)
        lay.addWidget(v)
        lay.addStretch(1)
        frame.setLayout(lay)
        return v, frame

    def _hdr_sep_widget(self) -> QFrame:
        """Legacy Web `.hdrSep`: 1×~24px vertical rule between `.hdrPill` items (e48c1a7)."""
        sep = QFrame()
        sep.setObjectName("hdrSep")
        sep.setFixedSize(1, 26)
        sep.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        return sep

    def _header_icons_dir(self) -> Path:
        return Path(__file__).resolve().parents[1] / "assets" / "header_icons"

    def _header_icon_pixmap(self, filename: str, size: int = 22) -> QPixmap:
        """Rasterize SVG from git `vgcs/assets/header_icons/` (same paths as Web template)."""
        path = self._header_icons_dir() / filename
        if not path.exists():
            return QPixmap()
        try:
            renderer = QSvgRenderer(str(path))
            img = QImage(size, size, QImage.Format.Format_ARGB32_Premultiplied)
            img.fill(Qt.GlobalColor.transparent)
            painter = QPainter(img)
            renderer.render(painter)
            painter.end()
            return QPixmap.fromImage(img)
        except Exception:
            return QPixmap()

    def _make_hdr_icon_pill(
        self,
        icon_filename: str,
        value: QLabel,
        *,
        icon_size: int = 22,
        min_w: int = 0,
    ) -> QWidget:
        """Legacy `.hdrPill`: icon + text row (git e48c1a7 map_widget HTML)."""
        wrap = QWidget()
        wrap.setObjectName("hdrPill")
        # Content-driven sizing with an optional floor for always-visible critical cells.
        wrap.setMinimumWidth(max(min_w, icon_size + 8))
        wrap.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        row = QHBoxLayout(wrap)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(6)
        ic = QLabel()
        ic.setFixedSize(icon_size, icon_size)
        pm = self._header_icon_pixmap(icon_filename, icon_size)
        if not pm.isNull():
            ic.setPixmap(pm)
        value.setObjectName("hdrPillValue")
        value.setAlignment(Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft)
        value.setWordWrap(False)
        value.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        row.addWidget(ic, 0, Qt.AlignmentFlag.AlignVCenter)
        row.addWidget(value, 1, Qt.AlignmentFlag.AlignVCenter)
        return wrap

    def _make_hdr_gps_pill_widget(self) -> QWidget:
        """GPS pill: `gps.svg` + `.hdrTinyStack` two-line column (Web map_widget)."""
        wrap = QWidget()
        wrap.setObjectName("hdrPill")
        wrap.setMinimumWidth(56 if self._compact_ui else 64)
        wrap.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        row = QHBoxLayout(wrap)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(6)
        ic = QLabel()
        ic.setFixedSize(22, 22)
        pm = self._header_icon_pixmap("gps.svg", 22)
        if not pm.isNull():
            ic.setPixmap(pm)
        self._top_gps_sat = QLabel("—")
        self._top_gps_sat.setObjectName("hdrGpsStackLine")
        self._top_gps_hdop = QLabel("—")
        self._top_gps_hdop.setObjectName("hdrGpsStackLine")
        self._top_gps_sat.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        self._top_gps_hdop.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        self._top_gps_sat.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        self._top_gps_hdop.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        stack = QVBoxLayout()
        stack.setContentsMargins(0, 0, 0, 0)
        stack.setSpacing(0)
        stack.addWidget(self._top_gps_sat)
        stack.addWidget(self._top_gps_hdop)
        row.addWidget(ic, 0, Qt.AlignmentFlag.AlignVCenter)
        row.addLayout(stack, 1)
        return wrap

    def _top_gps_status_line(self) -> str:
        """One-line GPS summary for popups/exports (sat line + HDOP line)."""
        return f"{self._top_gps_sat.text()} / {self._top_gps_hdop.text()}"

    def _set_top_vehicle_msg(self, message: object) -> None:
        """Allow truncation only for MESSAGE cell; keep full text in tooltip."""
        txt = str(message or "—")
        lbl = self._top_vehicle_msg
        max_px = 170 if self._compact_ui else 240
        elided = lbl.fontMetrics().elidedText(txt, Qt.TextElideMode.ElideRight, max_px)
        lbl.setText(elided)
        lbl.setToolTip(txt if elided != txt else "")

    def _apply_link_banner_palette(self, state: str) -> None:
        """Tint full `#linkBanner` like Web `setFlightStatus()` — not a separate arm rectangle.

        Empty ``state`` matches shipped CSS `#linkBanner { background: rgba(24,30,40,0.95); … }`
        (idle disconnected). Use ``red`` only for the JS ``else`` branch (communication lost).
        """
        st = (state or "").strip().lower()
        if st == "green":
            bg = "rgba(24, 82, 38, 0.96)"
            bd = "rgba(94, 214, 119, 0.95)"
            fg = "#e8ffe8"
            muted = "rgba(232, 255, 232, 0.58)"
        elif st == "yellow":
            bg = "rgba(120, 95, 24, 0.96)"
            bd = "rgba(247, 211, 92, 0.95)"
            fg = "#fff7dd"
            muted = "rgba(255, 247, 221, 0.58)"
        elif st == "red":
            bg = "rgba(124, 24, 24, 0.96)"
            bd = "rgba(245, 99, 99, 0.95)"
            fg = "#ffe8e8"
            muted = "rgba(255, 232, 232, 0.58)"
        else:
            bg = "rgba(24, 30, 40, 0.96)"
            bd = "rgba(72, 86, 110, 0.9)"
            fg = "#dbe3f3"
            muted = "rgba(244, 247, 255, 0.52)"

        qss = "\n".join(
            (
                f"QFrame#headerBar {{ background-color: {bg}; border-bottom: 1px solid {bd}; }}",
                f"QLabel#hdrPillTitle {{ color: {muted}; }}",
                f"QLabel#hdrPillValue {{ color: {fg}; }}",
                f"QLabel#hdrGpsStackLine {{ color: {fg}; }}",
                "QPushButton#headerFlightChipBtn { background: transparent; border: none; "
                f"color: {fg}; font-weight: 700; font-size: 14px; padding: 2px 4px; }}",
                "QPushButton#headerFlightChipBtn:hover { background-color: rgba(255,255,255,0.08); }",
                f"QLabel#linkBannerText {{ color: {fg}; font-size: 14px; font-weight: 600; }}",
                "QPushButton#hdrConnectBtn, QPushButton#hdrDisconnectBtn {"
                f" color: {fg}; font-size: 13px; font-weight: 700; padding: 4px 12px; "
                " border-radius: 8px; border: 1px solid rgba(255,255,255,0.22); }",
                "QPushButton#hdrConnectBtn { background-color: rgba(62, 126, 232, 0.20); }",
                "QPushButton#hdrConnectBtn:hover { background-color: rgba(62, 126, 232, 0.34); }",
                "QPushButton#hdrDisconnectBtn { background-color: rgba(210, 60, 60, 0.18); }",
                "QPushButton#hdrDisconnectBtn:hover { background-color: rgba(210, 60, 60, 0.30); }",
                "QPushButton#hdrConnectBtn:disabled, QPushButton#hdrDisconnectBtn:disabled {"
                " background-color: rgba(255,255,255,0.08); color: rgba(240,245,255,0.55);"
                " border: 1px solid rgba(255,255,255,0.14); }",
            )
        )
        self._top_dashboard.setStyleSheet(qss)
        self._logo_btn.setStyleSheet(
            f"QPushButton {{ background: transparent; border: none; padding: 0; color: {fg}; "
            f"font-size: 15px; font-weight: 600; }}"
            f"QPushButton:hover {{ background-color: rgba(255,255,255,0.06); }}"
        )

    def _build_m2_top_dashboard(self) -> QFrame:
        bar = QFrame()
        bar.setObjectName("headerBar")
        bar.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        shell = QVBoxLayout()
        shell.setContentsMargins(12, 8, 12, 8)
        shell.setSpacing(0)
        header_outer = QHBoxLayout()
        header_outer.setContentsMargins(0, 0, 0, 0)
        header_outer.setSpacing(12 if self._compact_ui else 16)
        # Logo: match legacy Web #linkBannerLogo (height ~28px); scale slightly up for HiDPI.
        # We read intrinsic WxH from the file, then setScaledSize to a proportional
        # box (max edge capped). Final display scale uses scaledToHeight (uniform).
        logo_target_h = 28 if self._compact_ui else 32
        logo_decode_max = 2400  # longest edge for initial decode (memory bound)
        # Two-line hdr pills; Web banner min-height 46px — stack needs slightly more row pixels.
        chip_row_h = 52

        self._logo_btn = QPushButton("VGCS Logo")
        self._logo_btn.setObjectName("linkBannerLogo")
        self._logo_btn.clicked.connect(self._on_logo_menu)
        self._logo_btn.setFlat(True)
        self._logo_btn.setCursor(Qt.PointingHandCursor)
        self._logo_btn.setStyleSheet(
            "QPushButton { background: transparent; border: none; padding: 0; "
            "color: #dbe3f3; font-size: 15px; font-weight: 600; }"
            "QPushButton:hover { background: transparent; border: none; color: #f4f7ff; }"
            "QPushButton:pressed { background: transparent; border: none; }"
        )
        # Legacy Web `__LOGO_SRC__` → shipped asset (see git e48c1a7 map_widget template).
        logo_paths = (
            Path(__file__).resolve().parents[1] / "assets" / "Vama Logo.png",
            Path(__file__).resolve().parents[2] / "Vama Logo New.png",
            Path(__file__).resolve().parents[1] / "assets" / "vama_logo.jpg",
        )
        for logo_path in logo_paths:
            if not logo_path.exists():
                continue
            reader = QImageReader(str(logo_path))
            # Qt 6 defaults ~256MB allocation guard on IHDR × bpp before scaled decode.
            # Shipped `Vama Logo.png` has a very large declared canvas (~15k×6k) but small IDAT;
            # without this, read() returns null and the UI falls back to "VGCS Logo" text.
            reader.setAllocationLimit(512)
            reader.setAutoTransform(True)
            sz = reader.size()
            if sz.isValid():
                decode_sz = self._logo_scaled_decode_size(
                    sz.width(), sz.height(), logo_decode_max
                )
                reader.setScaledSize(decode_sz)
            elif logo_path.suffix.lower() == ".png":
                hdr = self._read_png_dimensions(logo_path)
                if hdr is not None:
                    decode_sz = self._logo_scaled_decode_size(
                        hdr[0], hdr[1], logo_decode_max
                    )
                    reader.setScaledSize(decode_sz)
            image = reader.read()
            if image.isNull():
                continue
            image = image.scaledToHeight(
                logo_target_h,
                Qt.TransformationMode.SmoothTransformation,
            )
            # Make near-black background pixels transparent so the logo
            # blends into the header instead of showing a black box.
            image = image.convertToFormat(QImage.Format_RGBA8888)
            w, h = image.width(), image.height()
            for y in range(h):
                for x in range(w):
                    c = image.pixelColor(x, y)
                    if c.red() < 8 and c.green() < 8 and c.blue() < 8:
                        image.setPixelColor(x, y, QColor(0, 0, 0, 0))
            pix = QPixmap.fromImage(image)
            icon = QIcon(pix)
            self._logo_btn.setIcon(icon)
            self._logo_btn.setIconSize(pix.size())
            self._logo_btn.setFixedSize(pix.size())
            self._logo_btn.setText("")
            break
        # Placeholder for layouts that still reference `_vehicle_msg_frame` — Web showed vehicle in `#linkBanner` only.
        self._vehicle_msg_frame = QWidget()
        self._vehicle_msg_frame.setFixedSize(0, 0)

        flight_wrap = QWidget()
        flight_wrap.setObjectName("hdrPill")
        # Content-sized: avoid MinimumExpanding + wide mins (left empty space in the banner).
        flight_wrap.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        fw_lay = QVBoxLayout(flight_wrap)
        fw_lay.setContentsMargins(0, 0, 4, 0)
        fw_lay.setSpacing(0)
        self._flight_status_btn = QPushButton("NOT READY TO ARM")
        self._flight_status_btn.setObjectName("headerFlightChipBtn")
        self._flight_status_btn.setCursor(Qt.PointingHandCursor)
        self._flight_status_btn.setSizePolicy(
            QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed
        )
        # Legacy Web `#linkBanner`: any header click except logo → VGCS_CONNECT_REQUEST (git e48c1a7).
        self._flight_status_btn.setCursor(Qt.ArrowCursor)
        fw_lay.addWidget(self._flight_status_btn)

        # git e48c1a7 map_widget: hold.svg → mode, link.svg → vehicle msg, gps.svg → stack, battery, remote_id, hdrMapModeBtn
        self._top_flight_mode = QLabel("—")
        mode_frame = self._make_hdr_icon_pill(
            "hold.svg", self._top_flight_mode, min_w=112 if self._compact_ui else 124
        )

        self._top_vehicle_msg = QLabel("—")
        self._top_vehicle_msg.setWordWrap(False)
        # No max width — long status (e.g. parameter download) was clipped; row scrolls if needed.
        self._top_vehicle_msg.setSizePolicy(
            QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed
        )
        vehicle_pill = self._make_hdr_icon_pill(
            "link.svg",
            self._top_vehicle_msg,
            icon_size=26,
            min_w=76 if self._compact_ui else 88,
        )

        gps_frame = self._make_hdr_gps_pill_widget()

        self._top_battery = QLabel("—")
        bat_frame = self._make_hdr_icon_pill(
            "battery.svg", self._top_battery, min_w=126 if self._compact_ui else 140
        )

        self._top_remote_id = QLabel("N/A")
        rid_frame = self._make_hdr_icon_pill(
            "remote_id.svg", self._top_remote_id, min_w=88 if self._compact_ui else 96
        )

        self._hdr_map_mode_btn = QPushButton("3D")
        self._hdr_map_mode_btn.setObjectName("hdrMapModeBtn")
        self._hdr_map_mode_btn.setFixedHeight(26)
        self._hdr_map_mode_btn.setCursor(Qt.PointingHandCursor)
        self._hdr_map_mode_btn.clicked.connect(self._on_map_toggle_3d_requested)

        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(0)
        left_layout.addWidget(self._logo_btn, 0, Qt.AlignLeft | Qt.AlignVCenter)

        chip_strip = QWidget()
        chip_strip.setFixedHeight(chip_row_h)
        chip_lay = QHBoxLayout(chip_strip)
        chip_lay.setContentsMargins(0, 0, 0, 0)
        chip_lay.setSpacing(12)

        chip_lay.addWidget(flight_wrap, 0, Qt.AlignVCenter)
        chip_lay.addWidget(self._hdr_sep_widget(), 0, Qt.AlignVCenter)
        chip_lay.addWidget(mode_frame, 0, Qt.AlignVCenter)
        chip_lay.addWidget(self._hdr_sep_widget(), 0, Qt.AlignVCenter)
        chip_lay.addWidget(vehicle_pill, 0, Qt.AlignVCenter)
        chip_lay.addWidget(self._hdr_sep_widget(), 0, Qt.AlignVCenter)
        chip_lay.addWidget(gps_frame, 0, Qt.AlignVCenter)
        chip_lay.addWidget(self._hdr_sep_widget(), 0, Qt.AlignVCenter)
        chip_lay.addWidget(bat_frame, 0, Qt.AlignVCenter)
        chip_lay.addWidget(self._hdr_sep_widget(), 0, Qt.AlignVCenter)
        chip_lay.addWidget(rid_frame, 0, Qt.AlignVCenter)
        chip_lay.addStretch(1)

        for w in (flight_wrap, mode_frame, vehicle_pill, gps_frame, bat_frame, rid_frame):
            w.setFixedHeight(chip_row_h)

        self._chip_strip = chip_strip

        header_scroll = QScrollArea()
        self._header_chip_scroll = header_scroll
        header_scroll.setObjectName("headerChipScroll")
        header_scroll.setWidget(chip_strip)
        # Let the chip row use the viewport width: trailing stretch absorbs slack; scroll if content overflows.
        header_scroll.setWidgetResizable(True)
        header_scroll.setFrameShape(QFrame.NoFrame)
        header_scroll.setHorizontalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAsNeeded
        )
        header_scroll.setVerticalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAlwaysOff
        )
        header_scroll.setMinimumHeight(chip_row_h)
        header_scroll.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        # Legacy Web: `#linkBannerDisconnected` vs `#linkBannerConnected` (git e48c1a7 — only yellow/green show pills).
        self._banner_disconnected_wrap = QWidget()
        self._banner_disconnected_wrap.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._banner_disconnected_wrap.setMinimumHeight(chip_row_h)
        dd_lay = QHBoxLayout(self._banner_disconnected_wrap)
        dd_lay.setContentsMargins(0, 0, 0, 0)
        dd_lay.setSpacing(10)
        self._link_banner_text = QLabel("Disconnected - Click to manually connect 💬")
        self._link_banner_text.setObjectName("linkBannerText")
        self._link_banner_text.setWordWrap(False)
        self._link_banner_text.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        dd_lay.addWidget(self._link_banner_text, 1, Qt.AlignmentFlag.AlignVCenter)
        self._hdr_connect_btn = QPushButton("Connect")
        self._hdr_connect_btn.setObjectName("hdrConnectBtn")
        self._hdr_connect_btn.setCursor(Qt.PointingHandCursor)
        self._hdr_connect_btn.setFixedHeight(28)
        self._hdr_connect_btn.setMinimumWidth(112 if self._compact_ui else 124)
        self._hdr_connect_btn.setIcon(self.style().standardIcon(QStyle.StandardPixmap.SP_DialogApplyButton))
        self._hdr_connect_btn.setIconSize(QSize(14, 14))
        self._hdr_connect_btn.clicked.connect(self._on_map_connect_requested)
        dd_lay.addWidget(self._hdr_connect_btn, 0, Qt.AlignmentFlag.AlignVCenter)

        self._banner_connected_wrap = QWidget()
        self._banner_connected_wrap.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._banner_connected_wrap.setMinimumHeight(chip_row_h)
        cc_lay = QHBoxLayout(self._banner_connected_wrap)
        cc_lay.setContentsMargins(0, 0, 0, 0)
        cc_lay.setSpacing(10)
        cc_lay.addWidget(header_scroll, 1)
        self._hdr_disconnect_btn = QPushButton("Disconnect")
        self._hdr_disconnect_btn.setObjectName("hdrDisconnectBtn")
        self._hdr_disconnect_btn.setCursor(Qt.PointingHandCursor)
        self._hdr_disconnect_btn.setFixedHeight(28)
        self._hdr_disconnect_btn.setMinimumWidth(112 if self._compact_ui else 124)
        self._hdr_disconnect_btn.setIcon(self.style().standardIcon(QStyle.StandardPixmap.SP_DialogCancelButton))
        self._hdr_disconnect_btn.setIconSize(QSize(14, 14))
        self._hdr_disconnect_btn.setEnabled(False)
        self._hdr_disconnect_btn.clicked.connect(self._on_disconnect)
        cc_lay.addWidget(self._hdr_disconnect_btn, 0, Qt.AlignmentFlag.AlignVCenter)

        self._header_banner_stack = QStackedWidget()
        self._header_banner_stack.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._header_banner_stack.addWidget(self._banner_disconnected_wrap)
        self._header_banner_stack.addWidget(self._banner_connected_wrap)

        header_outer.addWidget(left_panel, 0, Qt.AlignVCenter)
        header_outer.addWidget(self._header_banner_stack, 1, Qt.AlignVCenter)
        header_outer.addWidget(self._hdr_map_mode_btn, 0, Qt.AlignVCenter)

        # Web padding 8+8px; inner row chip_row_h — fixed total bar height.
        _hdr_pad_v = shell.contentsMargins().top() + shell.contentsMargins().bottom()
        bar.setFixedHeight(chip_row_h + _hdr_pad_v)
        shell.addLayout(header_outer)
        bar.setLayout(shell)
        return bar

    def _logo_scaled_decode_size(ow: int, oh: int, max_edge: int) -> QSize:
        """QSize for decode whose width/height ratio matches the source image."""
        if ow <= 0 or oh <= 0:
            return QSize(max_edge, max_edge)
        if max(ow, oh) <= max_edge:
            return QSize(ow, oh)
        s = max_edge / float(max(ow, oh))
        nw = max(1, int(round(ow * s)))
        nh = max(1, int(round(oh * s)))
        return QSize(nw, nh)

    def _read_png_dimensions(self, path: Path) -> tuple[int, int] | None:
        try:
            with path.open("rb") as f:
                header = f.read(24)
            if len(header) < 24:
                return None
            if header[:8] != b"\x89PNG\r\n\x1a\n":
                return None
            width, height = struct.unpack(">II", header[16:24])
            return int(width), int(height)
        except Exception:
            return None

    def _menu_icon(self, filename: str) -> QIcon:
        path = Path(__file__).resolve().parents[1] / "assets" / "menu_icons" / filename
        if path.exists():
            return QIcon(str(path))
        return QIcon()

    def _build_mission_list_panel(self) -> QGroupBox:
        box = QGroupBox("Mission waypoints")
        lay = QVBoxLayout()
        lay.addWidget(self._mission_table)
        box.setLayout(lay)
        return box

    def _build_m2_operations_layout(self) -> QWidget:
        root = QWidget()
        root.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        v = QVBoxLayout()
        self._operations_layout = v
        v.setSpacing(8 if self._compact_ui else 12)
        v.setContentsMargins(0, 0, 0, 0)

        self._center_row = QBoxLayout(QBoxLayout.LeftToRight)
        self._center_row.setSpacing(8 if self._compact_ui else 12)
        # Vehicle message lives in Web-style `#linkBanner` only (git e48c1a7); keep zero-width stub for compat.
        self._center_row.addWidget(self._vehicle_msg_frame, 0, Qt.AlignTop)
        self._center_row.addWidget(self._map_widget, 1)
        self._camera_panel = self._build_camera_control_panel()
        self._camera_panel.follow_triggered.connect(self._map_widget.set_video_follow_enabled)
        self._map_widget.video_follow_enabled_changed.connect(self._camera_panel.sync_video_follow_toggle)
        # Create the camera control backend used by split/video transforms,
        # but do not add it to the visible layout (removes the "Camera Control" UI panel).
        try:
            self._camera_panel.setVisible(False)
        except Exception:
            pass

        self._footer_row = QBoxLayout(QBoxLayout.LeftToRight)
        self._footer_row.setSpacing(8 if self._compact_ui else 12)
        self._footer_row.addWidget(self._build_primary_flight_footer(), 1)
        self._footer_row.addWidget(self._build_compass_footer(), 1)
        self._footer_row.addWidget(self._build_nav_system_footer(), 1)
        self._footer_widget = QWidget()
        self._footer_widget.setLayout(self._footer_row)

        v.addLayout(self._center_row, 1)
        v.addWidget(self._footer_widget)
        root.setLayout(v)
        return root

    def _build_camera_control_panel(self) -> QGroupBox:
        panel = CameraControlPanel(self._video, self)
        # Keep existing 3D map toggle here (matches current layout expectations).
        row = QHBoxLayout()
        self._btn_map_3d = QPushButton("3D View")
        self._btn_map_3d.setCheckable(True)
        self._btn_map_3d.clicked.connect(self._on_toggle_map_3d)
        row.addWidget(self._btn_map_3d)
        row.addStretch(1)
        panel.layout().addItem(row)  # type: ignore[union-attr]
        return panel

    def _build_primary_flight_footer(self) -> QGroupBox:
        box = QGroupBox("Primary Flight Data")
        lay = QVBoxLayout()
        self._footer_primary = QLabel("Alt — | Speed — | Time 00:00")
        self._footer_primary.setObjectName("telemetryValue")
        lay.addWidget(self._footer_primary)
        lay.addStretch()
        box.setLayout(lay)
        return box

    def _build_compass_footer(self) -> QGroupBox:
        box = QGroupBox("Compass/Attitude")
        lay = QVBoxLayout()
        lay.addWidget(self._compass, 0, Qt.AlignHCenter)
        box.setLayout(lay)
        return box

    def _build_nav_system_footer(self) -> QGroupBox:
        box = QGroupBox("Navigation System")
        lay = QVBoxLayout()
        self._footer_nav = QLabel("GPS — | HDOP — | RC —")
        self._footer_nav.setObjectName("telemetryValue")
        lay.addWidget(self._footer_nav)
        lay.addStretch()
        box.setLayout(lay)
        return box

    def _build_m2_controls_panel(self) -> QGroupBox:
        box = QGroupBox("M2 controls")
        lay = QGridLayout()
        lay.setHorizontalSpacing(8 if self._compact_ui else 10)
        lay.setVerticalSpacing(6 if self._compact_ui else 8)

        lay.addWidget(QLabel("Takeoff alt (m)"), 0, 0)
        lay.addWidget(self._takeoff_alt_spin, 0, 1)
        lay.addWidget(self._btn_takeoff, 0, 2)
        lay.addWidget(self._btn_land, 0, 3)
        lay.addWidget(self._btn_auto_takeoff, 0, 4)
        lay.addWidget(self._btn_auto_land, 0, 5)
        lay.addWidget(self._btn_emergency_stop, 0, 6, 1, 2)
        lay.addWidget(self._btn_apply_failsafe_preset, 0, 8, 1, 2)

        lay.addWidget(QLabel("Fence radius (m)"), 1, 0)
        lay.addWidget(self._geofence_radius_spin, 1, 1)
        lay.addWidget(QLabel("Fence alt max"), 1, 2)
        lay.addWidget(self._geofence_alt_max_spin, 1, 3)
        lay.addWidget(QLabel("Fence action"), 1, 4)
        lay.addWidget(self._geofence_action_combo, 1, 5)
        lay.addWidget(self._btn_apply_fence, 1, 6)

        lay.addWidget(QLabel("Param"), 2, 0)
        lay.addWidget(self._param_name_combo, 2, 1)
        lay.addWidget(QLabel("Value"), 2, 2)
        lay.addWidget(self._param_value_spin, 2, 3)
        lay.addWidget(self._btn_params_refresh, 2, 4)
        lay.addWidget(self._btn_param_set, 2, 5)
        lay.addWidget(self._btn_tiles_online, 3, 0, 1, 2)
        lay.addWidget(self._btn_tiles_offline, 3, 2, 1, 2)
        lay.addWidget(QLabel("Acro"), 4, 0)
        lay.addWidget(self._airmode_check, 4, 1, 1, 2)
        lay.addWidget(self._acro_trainer_combo, 4, 3)
        lay.addWidget(self._btn_apply_acro, 4, 4, 1, 2)
        lay.addWidget(QLabel("Simple"), 5, 0)
        lay.addWidget(self._simple_check, 5, 1, 1, 2)
        lay.addWidget(self._super_simple_check, 5, 3)
        lay.addWidget(self._btn_apply_simple, 5, 4, 1, 2)
        box.setLayout(lay)
        return box

    def _set_preconnect_dashboard_mode(self, enabled: bool) -> None:
        """
        Pre-connect visual mode: map-centric dashboard like reference image.
        Full M2 operator panels are shown after link-up.
        """
        # Keep map visible always.
        self._footer_widget.setVisible(not enabled)
        self._m2_controls_panel.setVisible(not enabled)
        self._mission_list_panel.setVisible(not enabled)
        self._log.setVisible(not enabled)
        self._status_frame.setVisible(not enabled)
        self._hb_frame.setVisible(not enabled)
        self._watchdog_frame.setVisible(not enabled)

    def _set_map_only_dashboard_mode(self, enabled: bool) -> None:
        """Map-centric layout: keep operator clutter hidden but retain native header (logo + flight chips)."""
        if not enabled:
            return
        self._central_layout.setContentsMargins(0, 0, 0, 0)
        self._content_layout.setContentsMargins(0, 0, 0, 0)
        self._content_layout.setSpacing(2)
        self._operations_layout.setContentsMargins(0, 0, 0, 0)
        self._operations_layout.setSpacing(0)
        self._center_row.setSpacing(0)
        self._top_dashboard.setVisible(True)
        self._link_box.setVisible(False)
        self._m2_controls_panel.setVisible(False)
        self._status_frame.setVisible(False)
        self._hb_frame.setVisible(False)
        self._watchdog_frame.setVisible(False)
        self._mission_list_panel.setVisible(False)
        self._log.setVisible(False)
        self._footer_widget.setVisible(False)
        self._vehicle_msg_frame.setVisible(False)
        self._camera_panel.setVisible(False)
        self._scroll.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self._scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

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
        row_sys("Obstacle (prox)", "obstacle_prox", "LRF", "rangefinder")
        row_sys("Battery failsafe", "failsafe_battery", "RC failsafe", "failsafe_rc")
        la = QLabel("Arm readiness")
        la.setStyleSheet("color: #7d869c;")
        sg.addWidget(la, sr, 0)
        sg.addWidget(add_field("arm_ready"), sr, 1, 1, 3)
        systems.setLayout(sg)

        self._fields["video_link"].setText("N/A")
        self._fields["obstacle_prox"].setText("N/A")
        self._fields["rangefinder"].setText("N/A")
        self._fields["arm_ready"].setText("Best-effort from telemetry")
        self._apply_state_style(self._fields["video_link"], "na")
        self._apply_state_style(self._fields["obstacle_prox"], "na")
        self._apply_state_style(self._fields["rangefinder"], "na")

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
        # Dashboard log panel can be hidden in map-only mode; always mirror to console.
        print(line, flush=True)
        s = str(line or "")
        if s.startswith("HEARTBEAT"):
            # Serial link + companion HB lines flood QTextEdit and stall the GUI thread.
            return
        try:
            self._log.append(s)
            bar = self._log.verticalScrollBar()
            if bar is not None:
                bar.setValue(bar.maximum())
            # Cap widget size so Apply / Target never walk an unbounded document.
            doc = self._log.document()
            if doc is not None and doc.blockCount() > 400:
                cursor = self._log.textCursor()
                cursor.movePosition(cursor.MoveOperation.Start)
                cursor.movePosition(
                    cursor.MoveOperation.Down,
                    cursor.MoveMode.KeepAnchor,
                    doc.blockCount() - 300,
                )
                cursor.removeSelectedText()
        except Exception:
            pass

    def _refresh_footer_summary(self) -> None:
        self._footer_primary.setText(
            f"Alt {self._fields['alt_rel'].text()} | Speed {self._fields['groundspeed'].text()} | Time {self._fields['flight_time'].text()}"
        )
        self._footer_nav.setText(
            f"{self._fields['gps'].text()} | RC {self._fields['rc_link'].text()}"
        )
