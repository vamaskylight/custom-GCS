"""Native Qt port of the legacy WebEngine Plan Flight overlay (`#planFlightLayer`).

Recreates the same visual structure that lived in `map_widget.py` before the native
tile-map migration: top metrics bar, left tool rail, center File flyout, right tab
panel (Mission / Fence / Rally). The contract — signals, action ids, mission-panel
keys — mirrors the legacy JS bridge so `main_window._on_plan_flight_action` and
`_on_plan_mission_panel_changed` keep working unchanged.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from PySide6.QtCore import Qt, Signal, QTimer
from PySide6.QtGui import QColor, QMouseEvent, QPixmap, QRegion
from PySide6.QtWidgets import (
    QButtonGroup,
    QComboBox,
    QDoubleSpinBox,
    QFrame,
    QGraphicsDropShadowEffect,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)


_TOOLS: tuple[tuple[str, str], ...] = (
    ("File", "\u21A9"),
    ("Takeoff", "\u2191"),
    ("Waypoint", "\u2295"),
    ("ROI", "\u25C7"),
    ("Pattern", "\u25A6"),
    ("Center", "\u2726"),
)
_TOOL_HINTS: dict[str, str] = {
    "Takeoff": (
        "Sends NAV_TAKEOFF like the main Takeoff button. If Mission Start sets a "
        "Takeoff / launch altitude (m), that climb target is used; otherwise the "
        "dashboard Takeoff alt (m) applies. Vehicle must be connected."
    ),
    "Waypoint": (
        "Click on the map to place waypoints. Right-click or double-click a waypoint marker to remove it."
    ),
    "ROI": "Click on the map to add fence polygon vertices.",
    "Pattern": "Pattern templates use Mission → Pattern size (m). Pick Survey / Corridor / Structure again after changing spacing.",
    "Center": "Centers the map on the vehicle.",
}

_TEMPLATE_LABELS: dict[str, str] = {
    "survey": "Survey",
    "corridor": "Corridor Scan",
    "structure": "Structure Scan",
}

_PLAN_FLIGHT_QSS = """
QWidget#planFlightLayer {
    background: transparent;
}
QWidget#planFlightTopBar {
    background: rgba(32, 34, 40, 247);
    border-bottom: 1px solid rgba(70, 76, 88, 217);
}
QLabel#planExit {
    color: #f3f4f6;
    font-size: 18px;
    font-weight: 600;
    padding: 0 12px;
}
QLabel#planExit:hover {
    color: #facc15;
}
QPushButton#planBarUpload {
    min-height: 32px;
    padding: 0 18px;
    margin-right: 22px;
    border-radius: 6px;
    border: 1px solid rgba(110, 118, 135, 217);
    background: rgba(58, 62, 74, 242);
    color: #e8eaef;
    font-size: 13px;
    font-weight: 600;
}
QPushButton#planBarUpload:hover { background: rgba(72, 78, 92, 250); }
QPushButton#planBarUpload:disabled { color: rgba(232, 234, 239, 100); }
QLabel.pfLabel {
    color: #9ca3b0;
    font-size: 11px;
    font-weight: 600;
}
QLabel.pfMetric { color: #f3f4f6; font-size: 13px; }
QLabel.pfMetricValue { color: #ffffff; font-size: 13px; font-weight: 700; }
QWidget#planWorkspace { background: transparent; }
QFrame#planFlightToolRail {
    background: rgba(28, 30, 36, 245);
    border: 1px solid rgba(65, 70, 82, 230);
    border-radius: 8px;
}
QPushButton.planToolBtn {
    min-height: 52px;
    border-radius: 6px;
    border: 1px solid rgba(70, 76, 88, 217);
    background: rgba(40, 44, 52, 242);
    color: #e8eaef;
    font-size: 11px;
    font-weight: 600;
    padding: 4px 2px;
    text-align: center;
}
QPushButton.planToolBtn:hover { background: rgba(55, 60, 72, 250); }
QPushButton.planToolBtn:checked {
    background: #facc15;
    color: #111827;
    border-color: #ca8a04;
}
QFrame#planCenterPanel {
    background: rgba(36, 39, 48, 250);
    border: 1px solid rgba(70, 76, 88, 230);
    border-radius: 8px;
}
QLabel.planFileSectionTitle {
    color: #f9fafb;
    font-size: 13px;
    font-weight: 700;
}
QFrame.planTplCard {
    border: 1px solid rgba(90, 96, 110, 217);
    border-radius: 6px;
    background: rgba(28, 30, 36, 250);
}
QFrame.planTplCard:hover {
    border-color: rgba(250, 204, 21, 165);
}
QLabel.planTplLabel {
    background: rgba(24, 26, 32, 250);
    color: #e8eaef;
    font-size: 11px;
    font-weight: 600;
    padding: 6px 4px;
    border-top: 1px solid rgba(70, 76, 88, 217);
}
QLabel.planTplPreview {
    background: rgb(45, 55, 72);
    color: rgba(232, 234, 239, 180);
    font-size: 11px;
    min-height: 82px;
    border-top-left-radius: 5px;
    border-top-right-radius: 5px;
}
QPushButton.planFileBtn {
    min-height: 30px;
    padding: 0 12px;
    border-radius: 5px;
    border: 1px solid rgba(90, 96, 110, 230);
    background: rgba(48, 52, 62, 242);
    color: #e8eaef;
    font-size: 12px;
    font-weight: 600;
}
QPushButton.planFileBtn:hover:!disabled { background: rgba(62, 68, 82, 250); }
QPushButton.planFileBtn:disabled { color: rgba(232, 234, 239, 110); }
QPushButton.planFileBtnPrimary {
    background: rgba(88, 82, 118, 242);
    border-color: rgba(120, 110, 160, 217);
}
QPushButton.planFileBtnPrimary:hover:!disabled { background: rgba(100, 94, 132, 250); }
QFrame#planRightPanel {
    background: rgba(36, 38, 46, 240);
    border: 1px solid rgba(92, 96, 120, 217);
    border-radius: 6px;
}
QPushButton.planTab {
    border: none;
    background: #3a3d4a;
    color: #e8eaef;
    font-size: 12px;
    font-weight: 600;
    padding: 8px 4px;
    min-height: 36px;
    border-right: 1px solid rgba(0, 0, 0, 72);
}
QPushButton.planTab:hover:!checked { background: #45485a; }
QPushButton.planTab:checked {
    background: #f5e6a0;
    color: #111827;
    font-weight: 700;
}
QLabel.planSectionHeader {
    background: #4d5170;
    color: #f9fafb;
    font-size: 13px;
    font-weight: 600;
    padding: 9px 12px;
}
QFrame.planTabBody { background: #0c0c0e; }
QFrame.planTabBody[fenceMode="true"] { background: #14151a; }
QLabel.planFieldLabel {
    color: rgba(248, 250, 252, 224);
    font-size: 11px;
    font-weight: 600;
}
QComboBox.planRailSelect, QLineEdit.planRailInput, QDoubleSpinBox.planPatternSpin {
    min-height: 32px;
    padding: 4px 10px;
    border-radius: 5px;
    border: 1px solid rgba(100, 106, 124, 165);
    background: #ffffff;
    color: #111827;
    font-size: 12px;
    font-weight: 500;
}
QDoubleSpinBox.planPatternSpin::up-button, QDoubleSpinBox.planPatternSpin::down-button {
    width: 18px;
}
QComboBox.planRailSelect::drop-down { width: 22px; }
QLabel.planKvKey { color: rgba(210, 214, 222, 224); font-size: 12px; }
QLabel.planKvVal { color: #ffffff; font-size: 12px; font-weight: 700; }
QLabel.planNoteMission {
    color: rgba(218, 220, 228, 224);
    font-size: 11px;
}
QLabel.planHelpMuted {
    color: rgba(232, 234, 239, 184);
    font-size: 11px;
}
QPushButton#planStartMissionBtn {
    min-height: 36px;
    border-radius: 6px;
    border: 1px solid rgba(120, 90, 40, 217);
    background: #9a6b2d;
    color: #ffffff;
    font-size: 13px;
    font-weight: 700;
}
QPushButton#planStartMissionBtn:hover { background: #b07a34; }
QPushButton.planSeqRtlBtn {
    min-height: 32px;
    border-radius: 6px;
    border: 1px solid rgba(80, 86, 102, 230);
    background: #3d414d;
    color: #f3f4f6;
    font-size: 12px;
    font-weight: 600;
}
QPushButton.planSeqRtlBtn:hover { background: #4a4f5e; }
QPushButton.planGeoBtn {
    min-height: 34px;
    padding: 8px 12px;
    border-radius: 6px;
    border: 1px solid rgba(80, 86, 102, 230);
    background: #3d414d;
    color: #f3f4f6;
    font-size: 12px;
    font-weight: 600;
    text-align: center;
}
QPushButton.planGeoBtn:hover:!disabled { background: #4a4f5e; }
QPushButton.planGeoBtn:disabled { color: rgba(243, 244, 246, 110); }
QLabel.planGeoTitle { color: #ffffff; font-size: 12px; font-weight: 700; }
QLabel.planGeoLead, QLabel.planGeoStatus {
    color: rgba(232, 234, 239, 200);
    font-size: 12px;
}
QFrame.planRallyInfo {
    background: #121318;
    border: 1px solid rgba(80, 86, 102, 140);
    border-radius: 8px;
}
QLabel#planSeqPatternLabel {
    color: #facc15;
    font-size: 12px;
    font-weight: 700;
}
QFrame#planSeqPatternRow {
    background: rgba(60, 60, 70, 180);
    border: 1px solid rgba(110, 118, 135, 180);
    border-radius: 6px;
}
QFrame.planWpRow {
    background: rgba(20, 22, 28, 220);
    border: 1px solid rgba(70, 76, 88, 180);
    border-radius: 6px;
}
QLineEdit.planWpField {
    border: 1px solid rgba(100, 106, 124, 165);
    background: #ffffff;
    color: #111827;
    border-radius: 4px;
    padding: 3px 6px;
    font-size: 12px;
}
QLabel.planWpLabel {
    color: #f3f4f6;
    font-size: 12px;
    font-weight: 700;
}
QLabel.planWpUnit { color: #d1d5db; font-size: 11px; }
QPushButton#planSetLaunchMapCenterBtn {
    min-height: 32px;
    border-radius: 6px;
    border: 1px solid rgba(80, 86, 102, 230);
    background: #3d414d;
    color: #f3f4f6;
    font-size: 12px;
    font-weight: 600;
}
QPushButton#planSetLaunchMapCenterBtn:hover { background: #4a4f5e; }
QPushButton.planDetailsToggle {
    text-align: left;
    background: transparent;
    color: #f9fafb;
    font-size: 13px;
    font-weight: 600;
    border: none;
    border-top: 1px solid rgba(255, 255, 255, 60);
    padding: 12px 0 10px;
}
QPushButton.planDetailsToggle:hover { color: #facc15; }
"""


def _unit_to_m(v: float) -> float:
    return float(v)


def _m_to_unit(m: float) -> float:
    return float(m) if m else 0.0


def _unit_speed_to_mps(v: float) -> float:
    return float(v)


def _mps_to_unit_speed(mps: float) -> float:
    return float(mps) if mps else 0.0


def _vgcs_assets_dir() -> Path:
    return Path(__file__).resolve().parents[1] / "assets"


def _plan_tpl_image_path(action_id: str) -> Path | None:
    """PNG paths used by legacy WebEngine HTML (git 7a1f71b `map_widget._build_leaflet_html`)."""
    root = _vgcs_assets_dir()
    if action_id == "template_empty":
        for name in ("empty plan.png", "emtpy plan.png"):
            p = root / name
            if p.is_file():
                return p
        return None
    filenames = {
        "template_survey": "survey.png",
        "template_corridor": "Corridor Scan.png",
        "template_structure": "Structure Scan.png",
    }
    fn = filenames.get(action_id)
    if not fn:
        return None
    p = root / fn
    return p if p.is_file() else None


def _pixmap_cover(pm: QPixmap, w: int, h: int) -> QPixmap:
    """Scale + center crop like CSS `object-fit: cover` for the card preview band."""
    if pm.isNull() or w < 2 or h < 2:
        return pm
    scaled = pm.scaled(
        w,
        h,
        Qt.AspectRatioMode.KeepAspectRatioByExpanding,
        Qt.TransformationMode.SmoothTransformation,
    )
    x = max(0, (scaled.width() - w) // 2)
    y = max(0, (scaled.height() - h) // 2)
    return scaled.copy(x, y, w, h)


class _CollapsibleSection(QFrame):
    """Tiny `<details>` substitute: header button toggles inner body visibility."""

    def __init__(self, title: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setFrameShape(QFrame.Shape.NoFrame)
        v = QVBoxLayout(self)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(0)
        self._toggle = QPushButton(f"{title}   \u25BC")
        self._toggle.setProperty("class", "planDetailsToggle")
        self._toggle.setCheckable(True)
        self._toggle.setChecked(True)
        self._toggle.setCursor(Qt.CursorShape.PointingHandCursor)
        self._toggle.clicked.connect(self._on_toggle)
        v.addWidget(self._toggle)
        self._body = QWidget(self)
        self._body_layout = QVBoxLayout(self._body)
        self._body_layout.setContentsMargins(0, 6, 0, 12)
        self._body_layout.setSpacing(6)
        v.addWidget(self._body)
        self._title = title

    def body_layout(self) -> QVBoxLayout:
        return self._body_layout

    def _on_toggle(self, checked: bool) -> None:
        self._body.setVisible(bool(checked))
        arrow = "\u25BC" if checked else "\u25B6"
        self._toggle.setText(f"{self._title}   {arrow}")


class PlanTemplateCard(QFrame):
    """Create-plan thumbnail card: reliable click handling (child labels stay mouse-transparent)."""

    card_activated = Signal(str)

    def __init__(self, action_id: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._action_id = (action_id or "").strip()

    def mousePressEvent(self, event: QMouseEvent) -> None:  # noqa: N802
        if event.button() == Qt.MouseButton.LeftButton and self._action_id:
            self.card_activated.emit(self._action_id)
            event.accept()
            return
        super().mousePressEvent(event)


class PlanFlightPanel(QWidget):
    """Plan Flight overlay (native port).

    Visibility is managed by the parent (`MapWidget`) which resizes it to cover
    the map canvas. All user interactions are emitted as Qt signals so the
    Python-side handlers wired in `MainWindow` continue to work unchanged.
    """

    exit_requested = Signal()
    action_requested = Signal(str)
    tool_requested = Signal(str)
    mission_panel_changed = Signal(object)
    mission_start_requested = Signal()
    return_requested = Signal()
    set_launch_to_map_center_requested = Signal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("planFlightLayer")
        self.setStyleSheet(_PLAN_FLIGHT_QSS)
        # No styled background; the layer is transparent and only the chrome pieces are opaque.
        # A mask is applied so empty space around the chrome forwards clicks to the map below.
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, False)
        self.setAutoFillBackground(False)
        self._active_tool = "File"
        self._suppress_emit = False
        self._waypoint_count = 0
        self._link_ok = False
        self._template_id = ""
        self._mission_start_stack_on = False
        self._survey_label = "Survey"
        self._wp_meta: list[dict[str, float]] = []
        self._emit_timer = QTimer(self)
        self._emit_timer.setInterval(120)
        self._emit_timer.setSingleShot(True)
        self._emit_timer.timeout.connect(self._emit_mission_panel_state)

        # Build chrome pieces as direct children. No outer layout — `_relayout` positions
        # them manually so the empty space between them remains a click pass-through.
        self._top_bar = self._build_top_bar()
        self._tool_rail = self._build_tool_rail()
        self._center_panel = self._build_center_panel()
        self._right_panel = self._build_right_panel()

        self._tool_buttons[0].setChecked(True)
        self._update_center_panel_for_tool("File")
        self._refresh_chrome()
        self._render_waypoint_rows()
        self.set_sequence_template("")
        self._relayout()

    # ------------------------------------------------------------------ Top bar
    def _build_top_bar(self) -> QWidget:
        bar = QWidget(self)
        bar.setObjectName("planFlightTopBar")
        bar.setMinimumHeight(64)
        h = QHBoxLayout(bar)
        h.setContentsMargins(8, 6, 10, 6)
        h.setSpacing(12)
        h.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)

        exit_lbl = QLabel("\u2039 Exit Plan", bar)
        exit_lbl.setObjectName("planExit")
        exit_lbl.setCursor(Qt.CursorShape.PointingHandCursor)
        exit_lbl.mousePressEvent = lambda _e: self.exit_requested.emit()  # type: ignore[assignment]
        h.addWidget(exit_lbl, 0, Qt.AlignmentFlag.AlignVCenter)

        self._bar_upload = QPushButton("Upload", bar)
        self._bar_upload.setObjectName("planBarUpload")
        self._bar_upload.setCursor(Qt.CursorShape.PointingHandCursor)
        self._bar_upload.clicked.connect(lambda: self.action_requested.emit("bar_upload"))
        h.addWidget(self._bar_upload, 0, Qt.AlignmentFlag.AlignVCenter)

        self._pf_alt_diff = self._add_metric_group(
            h, "Selected Waypoint", [("Alt diff:", "0.0 m"), ("Gradient:", "-.-")]
        )
        self._pf_azimuth = self._add_metric_group(
            h, "", [("Azimuth:", "0"), ("Heading:", "nan")]
        )
        self._pf_dist = self._add_metric_group(
            h, "", [("Dist prev WP:", "0.0 m"), ("", "")]
        )
        # Breathing room: selected-WP metrics vs mission totals.
        h.addSpacing(32)
        self._pf_mission = self._add_metric_group(
            h,
            "Total Mission",
            [("Distance:", "0 m"), ("Time:", "00:00:00")],
        )
        h.addSpacing(24)
        self._pf_max_telem = self._add_metric_group(
            h, "", [("Max telem dist:", "0 m"), ("", "")]
        )

        # One trailing stretch: keep all metrics packed to the left; spare bar width stays on the right.
        h.addStretch(1)

        return bar

    def _add_metric_group(
        self,
        parent_layout: QHBoxLayout,
        label: str,
        rows: list[tuple[str, str]],
    ) -> dict[str, QLabel]:
        grp = QWidget()
        grp.setSizePolicy(QSizePolicy.Policy.Maximum, QSizePolicy.Policy.Preferred)
        v = QVBoxLayout(grp)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(2)
        if (label or "").strip():
            lbl = QLabel(label)
            lbl.setProperty("class", "pfLabel")
            v.addWidget(lbl)
        out: dict[str, QLabel] = {}
        for key, val in rows:
            if not (key or "").strip() and not (val or "").strip():
                continue
            row = QWidget()
            row.setSizePolicy(QSizePolicy.Policy.Maximum, QSizePolicy.Policy.Preferred)
            rl = QHBoxLayout(row)
            rl.setContentsMargins(0, 0, 0, 0)
            rl.setSpacing(3)
            if key:
                tag = QLabel(key)
                tag.setProperty("class", "pfMetric")
                rl.addWidget(tag)
            value_lbl = QLabel(val or "")
            value_lbl.setProperty("class", "pfMetricValue")
            rl.addWidget(value_lbl)
            v.addWidget(row)
            if key:
                out[key.strip().rstrip(":").lower()] = value_lbl
        parent_layout.addWidget(grp, 0, Qt.AlignmentFlag.AlignVCenter)
        return out

    # ------------------------------------------------------------------ Tool rail
    def _build_tool_rail(self) -> QWidget:
        rail = QFrame(self)
        rail.setObjectName("planFlightToolRail")
        rail.setFixedWidth(82)
        v = QVBoxLayout(rail)
        v.setContentsMargins(5, 5, 5, 5)
        v.setSpacing(4)
        self._tool_buttons: list[QPushButton] = []
        self._tool_group = QButtonGroup(self)
        self._tool_group.setExclusive(True)
        for name, glyph in _TOOLS:
            btn = QPushButton(f"{glyph}\n{name}", rail)
            btn.setProperty("class", "planToolBtn")
            btn.setCheckable(True)
            btn.setCursor(Qt.CursorShape.PointingHandCursor)
            btn.setMinimumHeight(52)
            btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            btn.clicked.connect(lambda _c=False, n=name: self._on_tool_clicked(n))
            v.addWidget(btn)
            self._tool_buttons.append(btn)
            self._tool_group.addButton(btn)
        v.addStretch(1)
        return rail

    def _on_tool_clicked(self, name: str) -> None:
        self.set_rail_tool(name)
        self.tool_requested.emit(name)

    def set_rail_tool(self, name: str) -> None:
        nm = (name or "").strip() or "File"
        self._active_tool = nm
        for btn in self._tool_buttons:
            label = btn.text().split("\n")[-1].strip()
            btn.setChecked(label.lower() == nm.lower())
        self._update_center_panel_for_tool(nm)

    # ------------------------------------------------------------------ Center
    def _build_center_panel(self) -> QWidget:
        panel = QFrame(self)
        panel.setObjectName("planCenterPanel")
        panel.setMinimumWidth(540)
        panel.setMaximumWidth(620)
        shadow = QGraphicsDropShadowEffect(panel)
        shadow.setBlurRadius(28)
        shadow.setOffset(0, 8)
        shadow.setColor(QColor(0, 0, 0, 120))
        panel.setGraphicsEffect(shadow)

        v = QVBoxLayout(panel)
        v.setContentsMargins(14, 12, 14, 14)
        v.setSpacing(0)

        self._center_stack = QStackedWidget(panel)
        v.addWidget(self._center_stack)

        self._center_stack.addWidget(self._build_file_flyout())
        self._center_stack.addWidget(self._build_other_tool_panel())
        return panel

    def _build_file_flyout(self) -> QWidget:
        host = QWidget()
        v = QVBoxLayout(host)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(16)

        # Create Plan
        v.addWidget(self._section_title("Create Plan"))
        grid = QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(8)
        templates: list[tuple[str, str]] = [
            ("Empty Plan", "template_empty"),
            ("Survey", "template_survey"),
            ("Corridor Scan", "template_corridor"),
            ("Structure Scan", "template_structure"),
        ]
        for idx, (label, action) in enumerate(templates):
            row, col = divmod(idx, 2)
            grid.addWidget(self._template_card(label, action), row, col)
        wrap = QWidget()
        wrap.setLayout(grid)
        v.addWidget(wrap)

        # Storage
        v.addWidget(self._section_title("Storage"))
        sto_row = QHBoxLayout()
        sto_row.setContentsMargins(0, 0, 0, 0)
        sto_row.setSpacing(8)
        self._btn_open = self._file_button("Open...", "open", primary=True)
        self._btn_save = self._file_button("Save", "save")
        self._btn_save_as = self._file_button("Save As...", "save_as")
        sto_row.addWidget(self._btn_open)
        sto_row.addWidget(self._btn_save)
        sto_row.addWidget(self._btn_save_as)
        sto_wrap = QWidget()
        sto_wrap.setLayout(sto_row)
        v.addWidget(sto_wrap)
        self._btn_kml = self._file_button("Save Mission Waypoints As KML...", "save_kml")
        v.addWidget(self._btn_kml)

        # Vehicle
        v.addWidget(self._section_title("Vehicle"))
        veh_row = QHBoxLayout()
        veh_row.setContentsMargins(0, 0, 0, 0)
        veh_row.setSpacing(8)
        self._btn_vup = self._file_button("Upload", "vehicle_upload")
        self._btn_vdown = self._file_button("Download", "vehicle_download")
        self._btn_vclr = self._file_button("Clear", "vehicle_clear")
        veh_row.addWidget(self._btn_vup)
        veh_row.addWidget(self._btn_vdown)
        veh_row.addWidget(self._btn_vclr)
        veh_wrap = QWidget()
        veh_wrap.setLayout(veh_row)
        v.addWidget(veh_wrap)

        v.addStretch(1)
        return host

    def _section_title(self, text: str) -> QLabel:
        lbl = QLabel(text)
        lbl.setProperty("class", "planFileSectionTitle")
        return lbl

    def _template_card(self, label: str, action_id: str) -> QFrame:
        card = PlanTemplateCard(action_id, self)
        card.setProperty("class", "planTplCard")
        card.setCursor(Qt.CursorShape.PointingHandCursor)
        card.setMinimumHeight(110)
        card.card_activated.connect(self.action_requested.emit)
        v = QVBoxLayout(card)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(0)
        # Preview band matches legacy `.planTplPrev` (~82px) + `object-fit: cover` image.
        _pw, _ph = 252, 82
        preview = QLabel()
        preview.setProperty("class", "planTplPreview")
        preview.setAlignment(Qt.AlignmentFlag.AlignCenter)
        preview.setMinimumHeight(_ph)
        preview.setMaximumHeight(_ph)
        preview.setScaledContents(False)
        preview.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        img_path = _plan_tpl_image_path(action_id)
        if img_path is not None:
            pm = QPixmap(str(img_path))
            if not pm.isNull():
                preview.setPixmap(_pixmap_cover(pm, _pw, _ph))
            else:
                preview.setText((label.split()[0] if label else "") or "")
        else:
            preview.setText((label.split()[0] if label else "") or "")
        v.addWidget(preview, 0)
        cap = QLabel(label)
        cap.setProperty("class", "planTplLabel")
        cap.setAlignment(Qt.AlignmentFlag.AlignCenter)
        cap.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        v.addWidget(cap, 0)
        return card

    def _file_button(self, label: str, action_id: str, *, primary: bool = False) -> QPushButton:
        btn = QPushButton(label)
        cls = "planFileBtn planFileBtnPrimary" if primary else "planFileBtn"
        btn.setProperty("class", cls)
        btn.setCursor(Qt.CursorShape.PointingHandCursor)
        btn.clicked.connect(lambda _c=False, a=action_id: self.action_requested.emit(a))
        return btn

    def _build_other_tool_panel(self) -> QWidget:
        host = QWidget()
        v = QVBoxLayout(host)
        v.setContentsMargins(4, 14, 4, 14)
        v.setSpacing(8)
        self._other_tool_hint = QLabel("Select a tool from the rail.")
        self._other_tool_hint.setWordWrap(True)
        self._other_tool_hint.setStyleSheet(
            "QLabel { color: #e8eaef; font-size: 13px; line-height: 1.45; }"
        )
        v.addWidget(self._other_tool_hint)
        v.addStretch(1)
        return host

    def _update_center_panel_for_tool(self, tool: str) -> None:
        is_file = tool.strip().lower() == "file"
        # Legacy `updatePlanToolPanel` hides `#planCenterPanel` entirely when the active
        # tool is not File, freeing the map area for Waypoint / ROI / Pattern clicks.
        self._center_stack.setCurrentIndex(0 if is_file else 1)
        if hasattr(self, "_center_panel"):
            self._center_panel.setVisible(is_file)
            self._relayout()
        self._other_tool_hint.setText(_TOOL_HINTS.get(tool, "Tool active."))

    # ------------------------------------------------------------------ Layout / mask
    def resizeEvent(self, event) -> None:  # noqa: N802 (Qt naming)
        super().resizeEvent(event)
        self._relayout()

    def showEvent(self, event) -> None:  # noqa: N802 (Qt naming)
        super().showEvent(event)
        self._relayout()

    def _relayout(self) -> None:
        if not hasattr(self, "_top_bar"):
            return
        w = max(1, self.width())
        h = max(1, self.height())
        top_h = max(64, self._top_bar.sizeHint().height())
        self._top_bar.setGeometry(0, 0, w, top_h)

        workspace_top = top_h + 10
        workspace_bottom_margin = 12
        avail_h = max(120, h - workspace_top - workspace_bottom_margin)

        # Tool rail: left edge, sized to its content.
        rail_w = max(72, self._tool_rail.sizeHint().width())
        rail_h = min(avail_h, max(120, self._tool_rail.sizeHint().height()))
        self._tool_rail.setGeometry(10, workspace_top, rail_w, rail_h)

        # Center panel: directly right of the rail when the File tool is active.
        center_x = 10 + rail_w + 10
        center_visible = self._center_panel.isVisible()
        center_w = 0
        if center_visible:
            center_w = max(540, min(620, self._center_panel.sizeHint().width()))
            center_w = min(center_w, max(280, w - center_x - 360))
            center_h = min(avail_h, max(220, self._center_panel.sizeHint().height()))
            self._center_panel.setGeometry(center_x, workspace_top, center_w, center_h)

        # Right panel: anchored to the right edge but pushed past the center panel if needed.
        right_w = self._right_panel.minimumWidth() or 340
        right_w = min(right_w, max(220, w - center_x - 24))
        right_h = min(avail_h, max(360, self._right_panel.sizeHint().height()))
        center_right_edge = center_x + center_w + (12 if center_visible else 0)
        right_x = max(center_right_edge if center_visible else (center_x + 12), w - right_w - 10)
        self._right_panel.setGeometry(right_x, workspace_top, right_w, right_h)

        self._update_mask()

    def _update_mask(self) -> None:
        if not hasattr(self, "_top_bar"):
            return
        region = QRegion(self._top_bar.geometry())
        region = region.united(QRegion(self._tool_rail.geometry()))
        if self._center_panel.isVisible():
            region = region.united(QRegion(self._center_panel.geometry()))
        region = region.united(QRegion(self._right_panel.geometry()))
        self.setMask(region)

    # ------------------------------------------------------------------ Right
    def _build_right_panel(self) -> QWidget:
        panel = QFrame(self)
        panel.setObjectName("planRightPanel")
        panel.setFixedWidth(340)
        panel.setMinimumHeight(420)
        v = QVBoxLayout(panel)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(0)

        tabs_row = QWidget(panel)
        th = QHBoxLayout(tabs_row)
        th.setContentsMargins(0, 0, 0, 0)
        th.setSpacing(0)
        self._tab_group = QButtonGroup(self)
        self._tab_group.setExclusive(True)
        self._tab_buttons: dict[str, QPushButton] = {}
        for key, label in [
            ("mission", "Mission"),
            ("fence", "Fence"),
            ("rally", "Rally"),
        ]:
            b = QPushButton(label, tabs_row)
            b.setProperty("class", "planTab")
            b.setCheckable(True)
            b.setCursor(Qt.CursorShape.PointingHandCursor)
            b.clicked.connect(lambda _c=False, k=key: self._activate_tab(k))
            th.addWidget(b, 1)
            self._tab_buttons[key] = b
            self._tab_group.addButton(b)
        v.addWidget(tabs_row)

        self._tab_stack = QStackedWidget(panel)
        v.addWidget(self._tab_stack, 1)

        self._tab_stack.addWidget(self._build_mission_tab())
        self._tab_stack.addWidget(self._build_fence_tab())
        self._tab_stack.addWidget(self._build_rally_tab())

        self._tab_buttons["mission"].setChecked(True)
        self._activate_tab("mission")
        return panel

    def _activate_tab(self, key: str) -> None:
        order = ["mission", "fence", "rally"]
        idx = order.index(key) if key in order else 0
        for k, b in self._tab_buttons.items():
            b.setChecked(k == order[idx])
        self._tab_stack.setCurrentIndex(idx)

    def _build_mission_tab(self) -> QWidget:
        body = QFrame()
        body.setProperty("class", "planTabBody")
        v = QVBoxLayout(body)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(0)

        self._mission_header = QLabel("Mission")
        self._mission_header.setProperty("class", "planSectionHeader")
        v.addWidget(self._mission_header)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setStyleSheet("QScrollArea { background: #0c0c0e; }")

        host = QWidget()
        host.setStyleSheet("QWidget { background: #0c0c0e; color: #e8eaef; }")
        body_v = QVBoxLayout(host)
        body_v.setContentsMargins(12, 12, 12, 12)
        body_v.setSpacing(10)

        # Takeoff card
        self._takeoff_card = QFrame()
        self._takeoff_card.setStyleSheet(
            "QFrame { background: rgba(20, 22, 28, 220); border: 1px solid rgba(60, 66, 80, 200); "
            "border-radius: 6px; }"
        )
        tc = QVBoxLayout(self._takeoff_card)
        tc.setContentsMargins(10, 10, 10, 10)
        tc.setSpacing(8)
        self._takeoff_title = QLabel("Takeoff")
        self._takeoff_title.setStyleSheet(
            "QLabel { color: #ffffff; font-size: 13px; font-weight: 700; }"
        )
        tc.addWidget(self._takeoff_title)
        self._takeoff_desc = QLabel(
            "Take off from the ground and ascend to specified altitude."
        )
        self._takeoff_desc.setWordWrap(True)
        self._takeoff_desc.setStyleSheet(
            "QLabel { color: rgba(232, 234, 239, 200); font-size: 12px; }"
        )
        tc.addWidget(self._takeoff_desc)

        self._takeoff_body = QWidget()
        tbv = QVBoxLayout(self._takeoff_body)
        tbv.setContentsMargins(0, 0, 0, 0)
        tbv.setSpacing(6)
        tbv.addWidget(self._field_label("All Altitudes"))
        self._alt_ref_combo = QComboBox()
        self._alt_ref_combo.setProperty("class", "planRailSelect")
        self._alt_ref_combo.addItem("Altitude Relative To Launch", "rel")
        self._alt_ref_combo.addItem("AMSL", "amsl")
        self._alt_ref_combo.addItem("AGL", "agl")
        self._alt_ref_combo.currentIndexChanged.connect(lambda _i: self._schedule_emit())
        tbv.addWidget(self._alt_ref_combo)
        tbv.addWidget(self._field_label("Initial Waypoint Alt"))
        self._initial_wp_alt = self._unit_input("164.0", "m")
        self._initial_wp_alt.textChanged.connect(lambda _t: self._schedule_emit())
        tbv.addWidget(self._initial_wp_alt)
        tc.addWidget(self._takeoff_body)
        body_v.addWidget(self._takeoff_card)

        # Pattern row
        self._pattern_row = QFrame()
        self._pattern_row.setObjectName("planSeqPatternRow")
        prl = QHBoxLayout(self._pattern_row)
        prl.setContentsMargins(10, 6, 10, 6)
        prl.setSpacing(8)
        self._pattern_glyph = QLabel("\u25A6")
        self._pattern_glyph.setStyleSheet("color: #facc15; font-size: 14px;")
        prl.addWidget(self._pattern_glyph)
        self._pattern_label = QLabel("Survey")
        self._pattern_label.setObjectName("planSeqPatternLabel")
        prl.addWidget(self._pattern_label)
        prl.addStretch(1)
        body_v.addWidget(self._pattern_row)
        self._pattern_row.setVisible(False)

        self._pattern_geometry_frame = QFrame()
        self._pattern_geometry_frame.setStyleSheet(
            "QFrame { background: rgba(20, 22, 28, 200); border: 1px solid rgba(60, 66, 80, 180); "
            "border-radius: 6px; }"
        )
        pg = QGridLayout(self._pattern_geometry_frame)
        pg.setContentsMargins(10, 8, 10, 8)
        pg.setHorizontalSpacing(8)
        pg.setVerticalSpacing(6)
        geo_title = QLabel("Pattern size (m)")
        geo_title.setStyleSheet("QLabel { color: #ffffff; font-size: 12px; font-weight: 700; }")
        pg.addWidget(geo_title, 0, 0, 1, 2)
        self._pattern_row_spacing_spin = QDoubleSpinBox()
        self._pattern_row_spacing_spin.setProperty("class", "planPatternSpin")
        self._pattern_row_spacing_spin.setRange(5.0, 200.0)
        self._pattern_row_spacing_spin.setDecimals(1)
        self._pattern_row_spacing_spin.setSingleStep(1.0)
        self._pattern_row_spacing_spin.setValue(20.0)
        self._pattern_row_spacing_spin.setToolTip(
            "Distance between parallel passes (row spacing). Smaller = tighter lawnmower lines."
        )
        self._pattern_pass_width_spin = QDoubleSpinBox()
        self._pattern_pass_width_spin.setProperty("class", "planPatternSpin")
        self._pattern_pass_width_spin.setRange(20.0, 800.0)
        self._pattern_pass_width_spin.setDecimals(1)
        self._pattern_pass_width_spin.setSingleStep(5.0)
        self._pattern_pass_width_spin.setValue(80.0)
        self._pattern_pass_width_spin.setToolTip("Width of the pattern east–west (survey grid / corridor length).")
        self._pattern_pass_depth_spin = QDoubleSpinBox()
        self._pattern_pass_depth_spin.setProperty("class", "planPatternSpin")
        self._pattern_pass_depth_spin.setRange(20.0, 800.0)
        self._pattern_pass_depth_spin.setDecimals(1)
        self._pattern_pass_depth_spin.setSingleStep(5.0)
        self._pattern_pass_depth_spin.setValue(60.0)
        self._pattern_pass_depth_spin.setToolTip(
            "Depth of the pattern north–south (survey rows span this distance)."
        )
        for sp in (
            self._pattern_row_spacing_spin,
            self._pattern_pass_width_spin,
            self._pattern_pass_depth_spin,
        ):
            sp.valueChanged.connect(lambda _v: self._schedule_emit())
        pg.addWidget(self._field_label("Row spacing"), 1, 0)
        pg.addWidget(self._pattern_row_spacing_spin, 1, 1)
        pg.addWidget(self._field_label("Pass width"), 2, 0)
        pg.addWidget(self._pattern_pass_width_spin, 2, 1)
        pg.addWidget(self._field_label("Pass depth"), 3, 0)
        pg.addWidget(self._pattern_pass_depth_spin, 3, 1)
        geo_hint = QLabel(
            "Adjust before or after choosing Survey / Corridor / Structure; apply the template again to rebuild."
        )
        geo_hint.setWordWrap(True)
        geo_hint.setProperty("class", "planHelpMuted")
        pg.addWidget(geo_hint, 4, 0, 1, 2)
        body_v.addWidget(self._pattern_geometry_frame)
        self._pattern_geometry_frame.setVisible(False)

        # Removed from Flight Plan page per UX request.
        self._seq_rtl_btn = QPushButton("Return To Launch")
        self._seq_rtl_btn.hide()

        # Start Mission
        self._start_mission_btn = QPushButton("Start Mission")
        self._start_mission_btn.setObjectName("planStartMissionBtn")
        self._start_mission_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._start_mission_btn.clicked.connect(self.mission_start_requested.emit)
        body_v.addWidget(self._start_mission_btn)

        # Waypoint details
        self._wp_details_box = QFrame()
        wd = QVBoxLayout(self._wp_details_box)
        wd.setContentsMargins(0, 4, 0, 0)
        wd.setSpacing(4)
        wd_title = QLabel("Waypoints & start")
        wd_title.setStyleSheet(
            "QLabel { color: #ffffff; font-size: 12px; font-weight: 700; padding: 6px 0; }"
        )
        wd.addWidget(wd_title)
        self._wp_rows_host = QWidget()
        self._wp_rows_layout = QVBoxLayout(self._wp_rows_host)
        self._wp_rows_layout.setContentsMargins(0, 0, 0, 0)
        self._wp_rows_layout.setSpacing(6)
        wd.addWidget(self._wp_rows_host)
        body_v.addWidget(self._wp_details_box)
        self._wp_details_box.setVisible(False)

        # Vehicle info collapsible
        self._vehicle_section = _CollapsibleSection("Vehicle Info")
        vbody = self._vehicle_section.body_layout()
        vrow = QHBoxLayout()
        vrow.setContentsMargins(0, 0, 0, 0)
        vrow.setSpacing(6)
        vrow.addWidget(self._kv_key("Firmware"))
        self._fw_value = self._kv_val("ArduPilot")
        vrow.addWidget(self._fw_value)
        vrow.addStretch(1)
        wrap1 = QWidget()
        wrap1.setLayout(vrow)
        vbody.addWidget(wrap1)
        vrow2 = QHBoxLayout()
        vrow2.setContentsMargins(0, 0, 0, 0)
        vrow2.setSpacing(6)
        vrow2.addWidget(self._kv_key("Vehicle"))
        self._veh_value = self._kv_val("Quadrotor")
        vrow2.addWidget(self._veh_value)
        vrow2.addStretch(1)
        wrap2 = QWidget()
        wrap2.setLayout(vrow2)
        vbody.addWidget(wrap2)
        note = QLabel(
            "The following speed values are used to calculate total mission time. "
            "They do not affect the flight speed for the mission."
        )
        note.setProperty("class", "planNoteMission")
        note.setWordWrap(True)
        vbody.addWidget(note)
        vbody.addWidget(self._field_label("Hover speed"))
        self._hover_input = self._unit_input("11.18", "m/s")
        self._hover_input.textChanged.connect(lambda _t: self._schedule_emit())
        vbody.addWidget(self._hover_input)
        body_v.addWidget(self._vehicle_section)

        # Launch position collapsible
        self._launch_section = _CollapsibleSection("Launch Position")
        lbody = self._launch_section.body_layout()
        lbody.addWidget(self._field_label("Altitude"))
        self._launch_alt = self._unit_input("0.0", "m")
        self._launch_alt.textChanged.connect(self._on_launch_alt_changed)
        lbody.addWidget(self._launch_alt)
        help_lbl = QLabel("Actual position set by vehicle at flight time.")
        help_lbl.setProperty("class", "planHelpMuted")
        help_lbl.setWordWrap(True)
        lbody.addWidget(help_lbl)
        lat_row = QHBoxLayout()
        lat_row.setContentsMargins(0, 0, 0, 0)
        lat_row.setSpacing(6)
        lat_row.addWidget(self._kv_key("Lat"))
        self._launch_lat_value = self._kv_val("\u2014")
        lat_row.addWidget(self._launch_lat_value)
        lat_row.addStretch(1)
        lat_wrap = QWidget()
        lat_wrap.setLayout(lat_row)
        lbody.addWidget(lat_wrap)
        lon_row = QHBoxLayout()
        lon_row.setContentsMargins(0, 0, 0, 0)
        lon_row.setSpacing(6)
        lon_row.addWidget(self._kv_key("Lon"))
        self._launch_lon_value = self._kv_val("\u2014")
        lon_row.addWidget(self._launch_lon_value)
        lon_row.addStretch(1)
        lon_wrap = QWidget()
        lon_wrap.setLayout(lon_row)
        lbody.addWidget(lon_wrap)
        self._set_launch_btn = QPushButton("Set To Map Center")
        self._set_launch_btn.setObjectName("planSetLaunchMapCenterBtn")
        self._set_launch_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._set_launch_btn.clicked.connect(self.set_launch_to_map_center_requested.emit)
        lbody.addWidget(self._set_launch_btn)
        body_v.addWidget(self._launch_section)

        body_v.addStretch(1)
        scroll.setWidget(host)
        v.addWidget(scroll, 1)
        return body

    def _build_fence_tab(self) -> QWidget:
        body = QFrame()
        body.setProperty("class", "planTabBody")
        body.setProperty("fenceMode", "true")
        v = QVBoxLayout(body)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(0)

        header = QLabel("GeoFence")
        header.setProperty("class", "planSectionHeader")
        v.addWidget(header)

        host = QWidget()
        host.setStyleSheet("QWidget { background: #14151a; color: #e8eaef; }")
        bv = QVBoxLayout(host)
        bv.setContentsMargins(12, 12, 12, 12)
        bv.setSpacing(14)

        lead = QLabel(
            "GeoFencing allows you to set a virtual fence around the area you want to fly in."
        )
        lead.setProperty("class", "planGeoLead")
        lead.setWordWrap(True)
        bv.addWidget(lead)

        hint = QLabel(
            "Draw a polygon with Polygon Fence, then use Upload fence on the dashboard "
            "to send it to the vehicle."
        )
        hint.setProperty("class", "planGeoLead")
        hint.setWordWrap(True)
        bv.addWidget(hint)

        bv.addWidget(self._geo_section("Insert GeoFence", [
            ("Polygon Fence", "fence_roi_tool", True),
            ("Circular Fence", "", False),
        ]))

        bv.addWidget(self._geo_section("Polygon Fences", []))
        self._geo_poly_status = QLabel("None")
        self._geo_poly_status.setProperty("class", "planGeoStatus")
        bv.addWidget(self._geo_poly_status)

        bv.addWidget(self._geo_section("Circular Fences", []))
        self._geo_circle_status = QLabel("None")
        self._geo_circle_status.setProperty("class", "planGeoStatus")
        bv.addWidget(self._geo_circle_status)

        bv.addWidget(self._geo_section("Breach Return Point", [
            ("Add Breach Return Point", "", False),
        ]))

        bv.addStretch(1)
        v.addWidget(host, 1)
        return body

    def _geo_section(self, title: str, buttons: list[tuple[str, str, bool]]) -> QWidget:
        host = QWidget()
        v = QVBoxLayout(host)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(6)
        lbl = QLabel(title)
        lbl.setProperty("class", "planGeoTitle")
        v.addWidget(lbl)
        for label, action_id, enabled in buttons:
            b = QPushButton(label)
            b.setProperty("class", "planGeoBtn")
            b.setEnabled(bool(enabled))
            b.setCursor(Qt.CursorShape.PointingHandCursor if enabled else Qt.CursorShape.ForbiddenCursor)
            if action_id:
                b.clicked.connect(lambda _c=False, a=action_id: self.action_requested.emit(a))
            v.addWidget(b)
        return host

    def _build_rally_tab(self) -> QWidget:
        body = QFrame()
        body.setProperty("class", "planTabBody")
        body.setProperty("fenceMode", "true")
        v = QVBoxLayout(body)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(0)
        header = QLabel("Rally Points")
        header.setProperty("class", "planSectionHeader")
        v.addWidget(header)
        host = QFrame()
        host.setProperty("class", "planRallyInfo")
        hv = QVBoxLayout(host)
        hv.setContentsMargins(14, 14, 14, 14)
        info = QLabel(
            "Rally Points provide alternate landing points when performing a Return to "
            "Launch (RTL). Rally editing is not implemented in M2."
        )
        info.setWordWrap(True)
        info.setStyleSheet("QLabel { color: #e8eaef; font-size: 12px; }")
        hv.addWidget(info)
        outer_host = QWidget()
        outer_host.setStyleSheet("QWidget { background: #14151a; }")
        oh = QVBoxLayout(outer_host)
        oh.setContentsMargins(12, 12, 12, 12)
        oh.addWidget(host)
        oh.addStretch(1)
        v.addWidget(outer_host, 1)
        return body

    # ------------------------------------------------------------------ Helpers
    def _field_label(self, text: str) -> QLabel:
        lbl = QLabel(text)
        lbl.setProperty("class", "planFieldLabel")
        return lbl

    def _unit_input(self, value: str, unit: str) -> QLineEdit:
        host = QLineEdit()
        host.setProperty("class", "planRailInput")
        host.setText(value)
        host.setPlaceholderText(unit)
        host.setClearButtonEnabled(False)
        host.setProperty("unit", unit)
        return host

    def _kv_key(self, text: str) -> QLabel:
        lbl = QLabel(text)
        lbl.setProperty("class", "planKvKey")
        return lbl

    def _kv_val(self, text: str) -> QLabel:
        lbl = QLabel(text)
        lbl.setProperty("class", "planKvVal")
        return lbl

    # ------------------------------------------------------------------ Public API
    def set_metrics(self, payload: dict[str, Any]) -> None:
        if not isinstance(payload, dict):
            return
        p = {str(k): str(v) for k, v in payload.items()}
        self._set_metric(self._pf_alt_diff, "alt diff", p.get("altDiffM", p.get("altDiffFt", "0.0 m")))
        self._set_metric(self._pf_alt_diff, "gradient", p.get("gradient", "-.-"))
        self._set_metric(self._pf_azimuth, "azimuth", p.get("azimuth", "0"))
        self._set_metric(self._pf_azimuth, "heading", p.get("heading", "nan"))
        self._set_metric(
            self._pf_dist, "dist prev wp", p.get("distPrevWpM", p.get("distPrevWpFt", "0.0 m"))
        )
        self._set_metric(
            self._pf_mission, "distance", p.get("missionDistanceM", p.get("missionDistanceFt", "0 m"))
        )
        self._set_metric(self._pf_mission, "time", p.get("missionTime", "00:00:00"))
        self._set_metric(
            self._pf_max_telem, "max telem dist", p.get("maxTelemDistM", p.get("maxTelemDistFt", "0 m"))
        )

    def _set_metric(self, group: dict[str, QLabel], key: str, value: str) -> None:
        if group and key in group:
            group[key].setText(value)

    def set_chrome_state(self, link_ok: bool, waypoint_count: int) -> None:
        self._link_ok = bool(link_ok)
        self._waypoint_count = max(0, int(waypoint_count))
        self._refresh_chrome()

    def _refresh_chrome(self) -> None:
        has = self._waypoint_count > 0
        link = self._link_ok
        self._bar_upload.setEnabled(link and has)
        self._btn_vup.setEnabled(link and has)
        self._btn_vdown.setEnabled(link)
        self._btn_save.setEnabled(has)
        self._btn_save_as.setEnabled(has)
        self._btn_kml.setEnabled(has)

    def set_sequence_template(self, template_id: str) -> None:
        tid = (template_id or "").strip().lower()
        self._template_id = tid
        is_survey = tid == "survey"
        label = _TEMPLATE_LABELS.get(tid, "")
        is_empty = not label
        self._pattern_row.setVisible(bool(label))
        self._pattern_geometry_frame.setVisible(bool(label))
        if label:
            self._pattern_label.setText(label)
        self._takeoff_title.setVisible(not (is_empty or is_survey))
        self._takeoff_desc.setVisible(not (is_empty or is_survey))
        self._seq_rtl_btn.setVisible(not (is_empty or is_survey))
        header_text = "Mission Start" if (is_empty or is_survey) else "Mission"
        self._mission_header.setText(header_text)

    def set_mission_start_stack(self, enabled: bool, survey_label: str = "Survey") -> None:
        self._mission_start_stack_on = bool(enabled)
        self._survey_label = str(survey_label or "Survey")
        self._mission_header.setText("Mission Start" if enabled else "Mission")
        self._takeoff_title.setVisible(not enabled)
        self._takeoff_desc.setVisible(not enabled)
        self._seq_rtl_btn.setVisible(not enabled)

    def set_vehicle_info(self, firmware: str, vehicle: str) -> None:
        self._fw_value.setText(firmware or "\u2014")
        self._veh_value.setText(vehicle or "\u2014")

    def apply_panel_state(self, state: dict[str, Any]) -> None:
        if not isinstance(state, dict):
            return
        self._suppress_emit = True
        try:
            alt_ref = str(state.get("altRef", "rel") or "rel")
            for i in range(self._alt_ref_combo.count()):
                if str(self._alt_ref_combo.itemData(i)) == alt_ref:
                    self._alt_ref_combo.setCurrentIndex(i)
                    break
            if "initialWpAltM" in state or "initialWpAltFt" in state:
                self._initial_wp_alt.setText(
                    str(state.get("initialWpAltM", state.get("initialWpAltFt")) or "164.0")
                )
            if "hoverMps" in state or "hoverMph" in state:
                self._hover_input.setText(str(state.get("hoverMps", state.get("hoverMph")) or "11.18"))
            if "launchAltM" in state or "launchAltFt" in state:
                self._launch_alt.setText(str(state.get("launchAltM", state.get("launchAltFt")) or "0.0"))
            lat = str(state.get("launchLat", "") or "").strip()
            lon = str(state.get("launchLon", "") or "").strip()
            self._launch_lat_value.setText(lat or "\u2014")
            self._launch_lon_value.setText(lon or "\u2014")
            wp_meta = state.get("wpMeta")
            if isinstance(wp_meta, list):
                self._wp_meta = []
                for row in wp_meta:
                    if isinstance(row, dict):
                        try:
                            self._wp_meta.append(
                                {
                                    "alt_m": float(row.get("alt_m", 0.0) or 0.0),
                                    "speed_mps": float(row.get("speed_mps", 0.0) or 0.0),
                                }
                            )
                        except Exception:
                            continue
                self._render_waypoint_rows()
            self._pattern_row_spacing_spin.setValue(
                float(state.get("patternRowSpacingM", 20.0) or 20.0)
            )
            self._pattern_pass_width_spin.setValue(
                float(state.get("patternPassWidthM", 80.0) or 80.0)
            )
            self._pattern_pass_depth_spin.setValue(
                float(state.get("patternPassDepthM", 60.0) or 60.0)
            )
        finally:
            self._suppress_emit = False

    def set_waypoint_count(self, count: int) -> None:
        n = max(0, int(count))
        if n == self._waypoint_count and len(self._wp_meta) == n:
            return
        self._waypoint_count = n
        self._refresh_chrome()
        base_alt_m = _unit_to_m(self._float(self._initial_wp_alt.text(), 164.0))
        base_spd_mps = _unit_speed_to_mps(self._float(self._hover_input.text(), 11.18))
        while len(self._wp_meta) < n:
            self._wp_meta.append({"alt_m": base_alt_m, "speed_mps": base_spd_mps})
        if len(self._wp_meta) > n:
            self._wp_meta = self._wp_meta[:n]
        self._render_waypoint_rows()

    def set_waypoint_meta(self, meta: list[dict[str, float]]) -> None:
        cleaned: list[dict[str, float]] = []
        for row in meta or []:
            if isinstance(row, dict):
                try:
                    cleaned.append(
                        {
                            "alt_m": float(row.get("alt_m", 0.0) or 0.0),
                            "speed_mps": float(row.get("speed_mps", 0.0) or 0.0),
                        }
                    )
                except Exception:
                    continue
        self._wp_meta = cleaned
        self._waypoint_count = len(cleaned)
        self._refresh_chrome()
        self._render_waypoint_rows()

    def get_waypoint_meta(self) -> list[dict[str, float]]:
        return [dict(m) for m in self._wp_meta]

    def get_active_tool(self) -> str:
        return self._active_tool

    # ------------------------------------------------------------------ WP rows
    def _render_waypoint_rows(self) -> None:
        while self._wp_rows_layout.count():
            item = self._wp_rows_layout.takeAt(0)
            w = item.widget() if item else None
            if w is not None:
                w.setParent(None)
                w.deleteLater()
        self._wp_start_alt_in = None
        n = self._waypoint_count
        self._wp_details_box.setVisible(n > 0)
        if n <= 0:
            return
        launch_ft_raw = self._launch_alt.text().strip().replace(",", ".")
        try:
            launch_ft = float(launch_ft_raw)
        except Exception:
            launch_ft = 0.0
        self._wp_rows_layout.addWidget(self._build_wp_row(-1, launch_ft, None, is_start=True))
        for i in range(n):
            meta = self._wp_meta[i] if i < len(self._wp_meta) else {"alt_m": 0.0, "speed_mps": 0.0}
            alt_ft = _m_to_unit(meta.get("alt_m", 0.0))
            spd_mph = _mps_to_unit_speed(meta.get("speed_mps", 0.0))
            self._wp_rows_layout.addWidget(self._build_wp_row(i, alt_ft, spd_mph))

    def _build_wp_row(
        self,
        idx: int,
        alt_ft: float,
        speed_mph: float | None,
        *,
        is_start: bool = False,
    ) -> QFrame:
        row = QFrame()
        row.setProperty("class", "planWpRow")
        h = QHBoxLayout(row)
        h.setContentsMargins(8, 6, 8, 6)
        h.setSpacing(8)
        title = QLabel("Start" if is_start else f"WP {idx + 1}")
        title.setProperty("class", "planWpLabel")
        title.setFixedWidth(54)
        h.addWidget(title)
        alt_in = QLineEdit()
        alt_in.setProperty("class", "planWpField")
        alt_in.setText(f"{alt_ft:.1f}")
        alt_in.setFixedWidth(76)
        h.addWidget(alt_in)
        unit_alt = QLabel("m")
        unit_alt.setProperty("class", "planWpUnit")
        h.addWidget(unit_alt)
        if not is_start:
            spd_in = QLineEdit()
            spd_in.setProperty("class", "planWpField")
            spd_in.setText(f"{(speed_mph or 0.0):.1f}")
            spd_in.setFixedWidth(76)
            h.addWidget(spd_in)
            unit_spd = QLabel("m/s")
            unit_spd.setProperty("class", "planWpUnit")
            h.addWidget(unit_spd)
            spd_in.textChanged.connect(
                lambda _t, ix=idx, w=spd_in: self._on_wp_field_changed(ix, "speed_mps", w.text())
            )
            spd_in.editingFinished.connect(
                lambda ix=idx, w=spd_in: self._on_wp_field_editing_finished(ix, "speed_mps", w)
            )
            alt_in.textChanged.connect(
                lambda _t, ix=idx, w=alt_in: self._on_wp_field_changed(ix, "alt_m", w.text())
            )
            alt_in.editingFinished.connect(
                lambda ix=idx, w=alt_in: self._on_wp_field_editing_finished(ix, "alt_m", w)
            )
        else:
            hint = QLabel("Takeoff / launch altitude (0 = use WP1 for takeoff).")
            hint.setStyleSheet("QLabel { color: rgba(232, 234, 239, 170); font-size: 11px; }")
            hint.setWordWrap(True)
            h.addWidget(hint, 1)
            self._wp_start_alt_in = alt_in
            alt_in.textChanged.connect(self._on_start_alt_changed)
        h.addStretch(1)
        return row

    def _on_launch_alt_changed(self, text: str) -> None:
        # Mirror direct edits to the Launch Position altitude field into the Start
        # row's own alt QLineEdit — they're separate widget instances (the Start row
        # is rebuilt by _render_waypoint_rows), so without this the Start row keeps
        # showing a stale value until something else happens to trigger a re-render.
        start_in = getattr(self, "_wp_start_alt_in", None)
        if start_in is not None and start_in.text() != text:
            start_in.blockSignals(True)
            try:
                start_in.setText(text)
            finally:
                start_in.blockSignals(False)
        self._schedule_emit()

    def _on_start_alt_changed(self, text: str) -> None:
        if self._suppress_emit:
            return
        ft = self._float(text, 0.0)
        self._launch_alt.blockSignals(True)
        try:
            self._launch_alt.setText(f"{ft:.1f}")
        finally:
            self._launch_alt.blockSignals(False)
        self._schedule_emit()

    def _on_wp_field_changed(self, idx: int, key: str, raw: str) -> None:
        if self._suppress_emit:
            return
        if idx < 0 or idx >= len(self._wp_meta):
            return
        value = self._float(raw, 0.0)
        if key == "alt_m":
            self._wp_meta[idx]["alt_m"] = max(0.3, _unit_to_m(value))
        elif key == "speed_mps":
            self._wp_meta[idx]["speed_mps"] = max(0.1, _unit_speed_to_mps(value))
        self._schedule_emit()

    def _on_wp_field_editing_finished(self, idx: int, key: str, widget: QLineEdit) -> None:
        # Normalize the visible text to the actually-clamped/transmitted value once the
        # operator finishes editing (Enter/focus-out), so a rejected value like a negative
        # altitude doesn't keep showing on screen while a different value ships in the mission.
        if idx < 0 or idx >= len(self._wp_meta):
            return
        meta = self._wp_meta[idx]
        if key == "alt_m":
            display_value = _m_to_unit(meta.get("alt_m", 0.0))
        elif key == "speed_mps":
            display_value = _mps_to_unit_speed(meta.get("speed_mps", 0.0))
        else:
            return
        text = f"{display_value:.1f}"
        if widget.text() != text:
            widget.blockSignals(True)
            try:
                widget.setText(text)
            finally:
                widget.blockSignals(False)

    # ------------------------------------------------------------------ Emit
    def _schedule_emit(self) -> None:
        if self._suppress_emit:
            return
        self._emit_timer.start()

    def _emit_mission_panel_state(self) -> None:
        if self._suppress_emit:
            return
        payload: dict[str, Any] = {
            "altRef": str(self._alt_ref_combo.currentData() or "rel"),
            "initialWpAltM": self._float(self._initial_wp_alt.text(), 164.0),
            "hoverMps": self._float(self._hover_input.text(), 11.18),
            "launchAltM": self._float(self._launch_alt.text(), 0.0),
            "launchLat": self._launch_lat_value.text().strip().replace("\u2014", ""),
            "launchLon": self._launch_lon_value.text().strip().replace("\u2014", ""),
            "wpMeta": [dict(m) for m in self._wp_meta],
            "patternRowSpacingM": float(self._pattern_row_spacing_spin.value()),
            "patternPassWidthM": float(self._pattern_pass_width_spin.value()),
            "patternPassDepthM": float(self._pattern_pass_depth_spin.value()),
        }
        self.mission_panel_changed.emit(payload)

    @staticmethod
    def _float(raw: object, default: float) -> float:
        try:
            s = str(raw or "").strip().replace(",", ".")
            return float(s) if s else float(default)
        except Exception:
            return float(default)

    # ------------------------------------------------------------------ Launch
    def set_launch_position(self, lat: float | None, lon: float | None) -> None:
        if lat is None or lon is None:
            return
        try:
            la = float(lat)
            lo = float(lon)
        except Exception:
            return
        self._launch_lat_value.setText(f"{la:.7f}")
        self._launch_lon_value.setText(f"{lo:.7f}")
        self._schedule_emit()
