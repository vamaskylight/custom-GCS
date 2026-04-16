"""Shared Qt stylesheet for VGCS (dark GCS-style shell)."""


def gcs_stylesheet() -> str:
    """Fusion-friendly QSS: palette inspired by common GCS dark UIs (~#252a35 / #2c313c)."""
    return """
    QMainWindow, QWidget#centralRoot {
        background-color: #1a1d24;
        color: #e8eaef;
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
        font-family: "Cascadia Mono", "Consolas", "JetBrains Mono", monospace;
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
        font-family: "Cascadia Mono", "Consolas", monospace;
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
    """
