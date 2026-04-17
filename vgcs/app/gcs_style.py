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
    QMainWindow, QWidget#centralRoot, QWidget#contentRoot, QWidget#contentViewport {
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
    QLabel#statusChipTitle {
        font-size: 10px;
        font-weight: 600;
        color: #b8c2d9;
        text-transform: uppercase;
        letter-spacing: 0.08em;
    }
    QLabel#statusChipValue {
        font-size: 14px;
        font-weight: 600;
        color: #f2f4fa;
    }
    QFrame#statusChip {
        background-color: #1e2430;
        border: 1px solid #323a4d;
        border-radius: 10px;
        padding: 0px;
    }
    QFrame#statusChip:hover {
        border-color: #4f5d78;
        background-color: #232a38;
    }
    QFrame#vehicleMsgPanel {
        background-color: #1e2430;
        border: 1px solid #323a4d;
        border-radius: 10px;
    }
    QFrame#headerBar {
        background-color: #12151c;
        border-bottom: 1px solid #252d3d;
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
    QPushButton#headerFlightChipBtn {
        background-color: #2a3344;
        color: #f0f3fa;
        border: 1px solid #3d4a63;
        border-radius: 8px;
        padding: 6px 12px;
        font-weight: 600;
        font-size: 13px;
        min-height: 18px;
    }
    QPushButton#headerFlightChipBtn:hover {
        background-color: #344056;
        border-color: #5c6a8a;
    }
    QPushButton#headerFlightChipBtn:pressed {
        background-color: #232b3a;
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
