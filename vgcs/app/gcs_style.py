"""Shared Qt stylesheet for VGCS (dark GCS-style shell)."""

import re


def _scale_px(css: str, scale: float) -> str:
    if abs(scale - 1.0) < 1e-3:
        return css

    def repl(match: re.Match[str]) -> str:
        value = float(match.group(1))
        scaled = max(1, int(round(value * scale)))
        return f"{scaled}px"

    return re.sub(r"(\d+(?:\.\d+)?)px", repl, css)


def gcs_stylesheet(*, mono_family: str = "Consolas", ui_scale: float = 1.0) -> str:
    """Fusion-friendly QSS: palette inspired by common GCS dark UIs (~#252a35 / #2c313c)."""
    css = """
    QMainWindow, QWidget#centralRoot, QWidget#contentRoot, QWidget#contentViewport, QDialog, QDialog QWidget {
        background-color: #1a1d24;
        color: #e8eaef;
    }
    QLabel {
        color: #dbe1ee;
    }
    QScrollArea, QAbstractScrollArea {
        background-color: #1a1d24;
        border: none;
    }
    QScrollArea > QWidget > QWidget {
        background-color: #1a1d24;
    }
    QGroupBox {
        font-weight: 600;
        font-size: 13px;
        color: #c9d2e8;
        border: 1px solid #343b4d;
        border-radius: 8px;
        margin-top: 14px;
        padding: 16px 12px 12px 12px;
        background-color: #22262f;
    }
    QGroupBox::title {
        subcontrol-origin: margin;
        subcontrol-position: top left;
        left: 14px;
        padding: 0 8px;
        color: #a8b0c4;
    }
    QLabel#headerTitle {
        font-size: 22px;
        font-weight: 700;
        color: #f0f2f7;
        letter-spacing: 0.5px;
    }
    QLabel#headerSubtitle {
        font-size: 12px;
        color: #7d869c;
    }
    QLabel#telemetryValue {
        font-family: "__MONO_FAMILY__", "Consolas", "Courier New", monospace;
        font-size: 13px;
        color: #dce1ef;
    }
    /*
     * Legacy Web map #linkBanner + .hdrPill + .hdrSep (git e48c1a7 map_widget.py):
     *   #linkBanner: rgba(24,30,40,0.95); border-bottom rgba(72,86,110,0.9); padding 8px 12px
     *   #linkBannerConnected: gap 12px; font 14px weight 600; color #f4f7ff
     *   .hdrPill: inline-flex, NO boxed background (flat on the tinted banner)
     *   .hdrSep: 1x24px rgba(210,220,240,0.55)
     *   setFlightStatus tints the FULL banner rgba + text color (green/yellow/red)
     * Non-map panels still use statusChip below.
     */
    QLabel#statusChipTitle {
        font-size: 11px;
        font-weight: 600;
        color: #9ca3b0;
        text-transform: uppercase;
        letter-spacing: 0.08em;
    }
    QLabel#statusChipValue {
        font-size: 13px;
        font-weight: 600;
        color: #f3f4f6;
    }
    QFrame#statusChip {
        background-color: rgba(45, 45, 58, 0.96);
        border: 1px solid rgba(72, 86, 110, 0.75);
        border-radius: 8px;
        padding: 0px;
    }
    QFrame#statusChip:hover {
        border-color: rgba(92, 106, 130, 0.9);
        background-color: rgba(52, 52, 64, 0.96);
    }
    QFrame#vehicleMsgPanel {
        background-color: rgba(45, 45, 58, 0.96);
        border: 1px solid rgba(72, 86, 110, 0.75);
        border-radius: 8px;
    }
    QWidget#hdrPill {
        background-color: transparent;
        border: none;
    }
    QLabel#hdrPillTitle {
        font-size: 10px;
        font-weight: 600;
        color: rgba(244, 247, 255, 0.55);
        text-transform: uppercase;
        letter-spacing: 0.06em;
    }
    QLabel#hdrPillValue {
        font-size: 14px;
        font-weight: 600;
        color: #f4f7ff;
    }
    QLabel#hdrGpsStackLine {
        font-size: 12px;
        font-weight: 600;
        color: #f4f7ff;
        line-height: 1.05;
    }
    QPushButton#hdrMapModeBtn {
        margin-left: 8px;
        min-width: 62px;
        height: 26px;
        border-radius: 13px;
        border: 1px solid rgba(210, 220, 240, 0.65);
        background-color: rgba(20, 30, 42, 0.7);
        color: #f1f6ff;
        font-size: 11px;
        font-weight: 700;
        letter-spacing: 0.02em;
        padding: 0 10px;
    }
    QPushButton#hdrMapModeBtn:hover {
        background-color: rgba(36, 50, 69, 0.9);
    }
    QFrame#hdrSep {
        background-color: rgba(210, 220, 240, 0.55);
        border: none;
        max-width: 1px;
        min-width: 1px;
        min-height: 24px;
        max-height: 28px;
    }
    QFrame#headerBar {
        background-color: rgba(24, 30, 40, 0.96);
        border-bottom: 1px solid rgba(72, 86, 110, 0.9);
        border-radius: 0px;
    }
    QScrollArea#headerChipScroll {
        background-color: transparent;
        border: none;
    }
    QScrollArea#headerChipScroll > QWidget > QWidget {
        background-color: transparent;
    }
    QScrollArea#headerChipScroll QScrollBar:horizontal {
        background: #1a1f2a;
        height: 6px;
        border-radius: 3px;
        margin: 0 0 2px 0;
    }
    QScrollArea#headerChipScroll QScrollBar::handle:horizontal {
        background: #4a5568;
        min-width: 24px;
        border-radius: 3px;
    }
    /* Flight/arm control: flat text on tinted banner (Web setFlightStatus — no separate red box). */
    QPushButton#headerFlightChipBtn {
        background-color: transparent;
        color: #dbe3f3;
        border: none;
        border-radius: 0px;
        padding: 2px 4px;
        font-weight: 700;
        font-size: 14px;
        min-height: 22px;
    }
    QPushButton#headerFlightChipBtn:hover {
        background-color: rgba(255, 255, 255, 0.06);
    }
    QPushButton#headerFlightChipBtn:pressed {
        background-color: rgba(255, 255, 255, 0.1);
    }
    QMenu {
        background-color: #1e2430;
        color: #e8edf9;
        border: 1px solid #3d465a;
        border-radius: 8px;
        padding: 6px 4px;
    }
    QMenu::item {
        padding: 8px 28px 8px 16px;
        border-radius: 4px;
    }
    QMenu::item:selected {
        background-color: #2d3a52;
        color: #f0f4ff;
    }
    QMenu::separator {
        height: 1px;
        margin: 6px 10px;
        background-color: #3d465a;
    }
    QLineEdit, QTextEdit, QComboBox, QDoubleSpinBox {
        background-color: #1e222b;
        color: #e8edf9;
        border: 1px solid #3d465a;
        border-radius: 6px;
        padding: 6px 10px;
        selection-background-color: #3d6fb8;
    }
    QLineEdit::placeholder, QTextEdit::placeholder {
        color: #7f88a0;
    }
    QComboBox QAbstractItemView {
        background-color: #202531;
        color: #e8edf9;
        selection-background-color: #3d6fb8;
    }
    QAbstractSpinBox {
        color: #e8edf9;
    }
    QPushButton {
        background-color: #2d3545;
        color: #e8edf9;
        border: 1px solid #4a5570;
        border-radius: 6px;
        padding: 8px 16px;
        font-weight: 600;
        min-height: 20px;
    }
    QPushButton:hover {
        background-color: #384056;
        border-color: #5c6a8a;
    }
    QPushButton:pressed {
        background-color: #252b38;
    }
    QPushButton:disabled {
        color: #8d97b2;
        border-color: #353b4a;
        background-color: #283042;
    }
    QSplitter::handle {
        background-color: #2c3344;
        height: 4px;
    }
    QTextEdit {
        font-family: "__MONO_FAMILY__", "Consolas", "Courier New", monospace;
        font-size: 12px;
    }
    QScrollBar:vertical {
        background: #1e222b;
        width: 5px;
        margin: 0;
    }
    QScrollBar::handle:vertical {
        background: #4a5568;
        min-height: 20px;
        border-radius: 3px;
    }
    """.replace("__MONO_FAMILY__", mono_family)
    return _scale_px(css, ui_scale)
