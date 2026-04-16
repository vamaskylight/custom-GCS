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
        font-size: 11px;
        color: #7d869c;
        text-transform: uppercase;
    }
    QLabel#statusChipValue {
        font-size: 13px;
        font-weight: 600;
    }
    QFrame#statusChip {
        background-color: #2a303c;
        border: 1px solid #3d465a;
        border-radius: 8px;
        padding: 8px 12px;
    }
    QFrame#headerBar {
        background-color: #16181e;
        border-bottom: 1px solid #2c3344;
        border-radius: 0px;
    }
    QLineEdit, QTextEdit, QComboBox, QDoubleSpinBox {
        background-color: #1e222b;
        border: 1px solid #3d465a;
        border-radius: 6px;
        padding: 6px 10px;
        selection-background-color: #3d6fb8;
    }
    QPushButton {
        background-color: #2d3545;
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
        color: #5c6375;
        border-color: #353b4a;
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
        width: 12px;
        margin: 0;
    }
    QScrollBar::handle:vertical {
        background: #4a5568;
        min-height: 24px;
        border-radius: 4px;
    }
    """.replace("__MONO_FAMILY__", mono_family)
    return _scale_px(css, ui_scale)
