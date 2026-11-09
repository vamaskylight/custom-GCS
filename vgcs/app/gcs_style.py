"""Shared Qt stylesheet for VGCS: the windows and dialogs in the Vama GCS app's dark look.

The colours are QGC's dark palette from ``vgcs/app/ui_theme.py`` (2026-11-09
restyle, the developer's "make the PC version look like the app"). Until then
this sheet had its own blue-grey colours, and several windows drew light menus
and boxes of their own.
"""

from __future__ import annotations

import re
import tempfile
from pathlib import Path

from vgcs.app import ui_theme as t


def _scale_px(css: str, scale: float) -> str:
    if abs(scale - 1.0) < 1e-3:
        return css

    def repl(match: re.Match[str]) -> str:
        value = float(match.group(1))
        scaled = max(1, int(round(value * scale)))
        return f"{scaled}px"

    return re.sub(r"(\d+(?:\.\d+)?)px", repl, css)


def _arrow_image_url(name: str, px: int = 12, color: str = t.TEXT_DIM) -> str:
    """A small arrow as a PNG file for the sheet's ``image:`` rules ("" when it cannot be made).

    PNG, not the SVG itself: an SVG in a style sheet needs Qt's SVG image plugin,
    which a packaged build might not carry. The arrows are drawn by QtSvg into a
    cache folder once per start.
    """
    try:
        pm = t.icon_pixmap(name, px, color)
        if pm.isNull():
            return ""
        folder = Path(tempfile.gettempdir()) / "vgcs-ui-cache"
        folder.mkdir(parents=True, exist_ok=True)
        path = folder / f"{name}-{px}-{color.strip('#')}.png"
        if not path.is_file() and not pm.save(str(path), "PNG"):
            return ""
        return path.as_posix()
    except OSError:  # no writable temp folder: Fusion's own arrows then
        return ""


def gcs_stylesheet(*, mono_family: str = "Consolas", ui_scale: float = 1.0) -> str:
    """Fusion-friendly QSS in QGC's dark colours, for the main window and every dialog."""
    down = _arrow_image_url("arrow_down")
    up = _arrow_image_url("arrow_up")
    # The list and number boxes' own arrows, in the theme's grey. Only when both pictures could be
    # made: a styled arrow without its picture is no arrow at all, while Fusion's own still shows.
    arrows = ""
    if down and up:
        arrows = f"""
    QComboBox {{
        padding-right: 28px;
    }}
    QComboBox::drop-down {{
        subcontrol-origin: padding;
        subcontrol-position: center right;
        width: 24px;
        border: none;
        background: transparent;
    }}
    QComboBox::down-arrow {{
        image: url("{down}");
        width: 12px;
        height: 12px;
    }}
    QSpinBox, QDoubleSpinBox {{
        padding-right: 22px;
    }}
    QSpinBox::up-button, QDoubleSpinBox::up-button {{
        subcontrol-origin: border;
        subcontrol-position: top right;
        width: 20px;
        border: none;
        background: transparent;
    }}
    QSpinBox::down-button, QDoubleSpinBox::down-button {{
        subcontrol-origin: border;
        subcontrol-position: bottom right;
        width: 20px;
        border: none;
        background: transparent;
    }}
    QSpinBox::up-arrow, QDoubleSpinBox::up-arrow {{
        image: url("{up}");
        width: 10px;
        height: 10px;
    }}
    QSpinBox::down-arrow, QDoubleSpinBox::down-arrow {{
        image: url("{down}");
        width: 10px;
        height: 10px;
    }}
"""
    css = f"""
    QMainWindow, QWidget#centralRoot, QWidget#contentRoot, QWidget#contentViewport, QDialog {{
        background-color: {t.WINDOW};
        color: {t.TEXT};
    }}
    QDialog QWidget {{
        color: {t.TEXT};
    }}
    QDialog QScrollArea, QDialog QScrollArea > QWidget > QWidget, QDialog QStackedWidget,
    QDialog QTabWidget > QWidget {{
        background-color: transparent;
    }}
    QLabel {{
        color: {t.TEXT};
        background-color: transparent;
    }}
    QScrollArea, QAbstractScrollArea {{
        background-color: {t.WINDOW};
        border: none;
    }}
    QScrollArea > QWidget > QWidget {{
        background-color: {t.WINDOW};
    }}
    QGroupBox {{
        font-weight: 600;
        font-size: {t.FONT_BODY}px;
        color: {t.TEXT};
        border: 1px solid {t.BORDER};
        border-radius: {t.RADIUS}px;
        margin-top: 14px;
        padding: 16px 12px 12px 12px;
        background-color: {t.WINDOW_SHADE};
    }}
    QGroupBox::title {{
        subcontrol-origin: margin;
        subcontrol-position: top left;
        left: 12px;
        padding: 0 6px;
        color: {t.TEXT_DIM};
    }}
    QLabel#headerTitle {{
        font-size: {t.FONT_TITLE}px;
        font-weight: 700;
        color: {t.TEXT};
    }}
    QLabel#headerSubtitle {{
        font-size: {t.FONT_BODY}px;
        color: {t.TEXT_DIM};
    }}
    QLabel#telemetryValue {{
        font-family: "__MONO_FAMILY__", "Consolas", "Courier New", monospace;
        font-size: {t.FONT_BODY}px;
        color: {t.TEXT};
    }}
    QLabel#statusChipTitle {{
        font-size: {t.FONT_SMALL}px;
        font-weight: 600;
        color: {t.TEXT_DIM};
        text-transform: uppercase;
    }}
    QLabel#statusChipValue {{
        font-size: {t.FONT_BODY}px;
        font-weight: 600;
        color: {t.TEXT};
    }}
    QFrame#statusChip, QFrame#vehicleMsgPanel {{
        background-color: {t.WINDOW_SHADE};
        border: 1px solid {t.BORDER};
        border-radius: {t.RADIUS}px;
        padding: 0px;
    }}
    QWidget#hdrPill {{
        background-color: transparent;
        border: none;
    }}
    QLabel#hdrPillTitle {{
        font-size: {t.FONT_SMALL}px;
        font-weight: 600;
        color: rgba(255, 255, 255, 140);
        text-transform: uppercase;
    }}
    QLabel#hdrPillValue {{
        font-size: {t.FONT_MEDIUM}px;
        font-weight: 600;
        color: {t.TEXT};
    }}
    QLabel#hdrGpsStackLine {{
        font-size: {t.FONT_BODY}px;
        font-weight: 600;
        color: {t.TEXT};
    }}
    QFrame#hdrSep {{
        background-color: rgba(255, 255, 255, 60);
        border: none;
        max-width: 1px;
        min-width: 1px;
        min-height: 24px;
        max-height: 28px;
    }}
    QFrame#headerBar {{
        background-color: {t.WINDOW};
        border-bottom: 1px solid rgba(255, 255, 255, 40);
        border-radius: 0px;
    }}
    QScrollArea#headerChipScroll {{
        background-color: transparent;
        border: none;
    }}
    QScrollArea#headerChipScroll > QWidget > QWidget {{
        background-color: transparent;
    }}
    QScrollArea#headerChipScroll QScrollBar:horizontal {{
        background: transparent;
        height: 5px;
        border-radius: 2px;
        margin: 0 0 2px 0;
    }}
    QScrollArea#headerChipScroll QScrollBar::handle:horizontal {{
        background: rgba(255, 255, 255, 90);
        min-width: 24px;
        border-radius: 2px;
    }}
    QPushButton#headerFlightChipBtn {{
        background-color: transparent;
        color: {t.TEXT};
        border: none;
        border-radius: 0px;
        padding: 2px 4px;
        font-weight: 700;
        font-size: {t.FONT_MEDIUM}px;
        min-height: 22px;
    }}
    QPushButton#headerFlightChipBtn:hover {{
        background-color: rgba(255, 255, 255, 15);
    }}
    QMenu {{
        background-color: {t.WINDOW_SHADE};
        color: {t.TEXT};
        border: 1px solid {t.BORDER};
        border-radius: {t.RADIUS}px;
        padding: 6px 4px;
    }}
    QMenu::item {{
        padding: 8px 28px 8px 14px;
        border-radius: 4px;
        background-color: transparent;
    }}
    QMenu::item:selected {{
        background-color: {t.HIGHLIGHT};
        color: {t.TEXT};
    }}
    QMenu::item:disabled {{
        color: {t.TEXT_FAINT};
    }}
    QMenu::separator {{
        height: 1px;
        margin: 6px 10px;
        background-color: {t.BORDER};
    }}
    QToolTip {{
        background-color: {t.WINDOW_SHADE};
        color: {t.TEXT};
        border: 1px solid {t.BORDER_STRONG};
        border-radius: 4px;
        padding: 6px 8px;
    }}
    QLineEdit, QTextEdit, QPlainTextEdit, QComboBox, QSpinBox, QDoubleSpinBox, QDateTimeEdit, QTimeEdit {{
        background-color: {t.WINDOW_SHADE_DARK};
        color: {t.TEXT};
        border: 1px solid {t.BORDER};
        border-radius: 6px;
        padding: 6px 10px;
        selection-background-color: {t.HIGHLIGHT};
        selection-color: {t.TEXT};
    }}
    QLineEdit:focus, QTextEdit:focus, QPlainTextEdit:focus, QComboBox:focus, QSpinBox:focus,
    QDoubleSpinBox:focus {{
        border: 1px solid {t.HIGHLIGHT};
    }}
    QLineEdit:disabled, QComboBox:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled, QTextEdit:disabled {{
        color: {t.TEXT_FAINT};
        background-color: {t.WINDOW};
        border-color: #3a3a3a;
    }}
    QLineEdit::placeholder, QTextEdit::placeholder {{
        color: {t.TEXT_FAINT};
    }}
    QComboBox QAbstractItemView {{
        background-color: {t.WINDOW_SHADE};
        color: {t.TEXT};
        border: 1px solid {t.BORDER};
        selection-background-color: {t.HIGHLIGHT};
        selection-color: {t.TEXT};
        outline: none;
    }}
    {arrows}
    QAbstractSpinBox {{
        color: {t.TEXT};
    }}
    QPushButton, QToolButton {{
        background-color: {t.BUTTON};
        color: {t.TEXT};
        border: 1px solid {t.BUTTON_BORDER};
        border-radius: 6px;
        padding: 7px 16px;
        font-weight: 600;
        min-height: 20px;
    }}
    QPushButton:hover, QToolButton:hover {{
        background-color: {t.BUTTON_HOVER};
        border-color: #6c6c78;
    }}
    QPushButton:pressed, QToolButton:pressed {{
        background-color: {t.BUTTON_PRESSED};
    }}
    QPushButton:checked, QToolButton:checked {{
        background-color: {t.HIGHLIGHT};
        border-color: {t.HIGHLIGHT_HOVER};
        color: {t.TEXT};
    }}
    QPushButton:default {{
        border: 1px solid {t.HIGHLIGHT};
    }}
    QPushButton:disabled, QToolButton:disabled {{
        color: {t.TEXT_FAINT};
        border-color: #3a3a40;
        background-color: #2c2c31;
    }}
    QCheckBox, QRadioButton {{
        color: {t.TEXT};
        spacing: 8px;
        background-color: transparent;
    }}
    QCheckBox:disabled, QRadioButton:disabled {{
        color: {t.TEXT_FAINT};
    }}
    QCheckBox::indicator, QRadioButton::indicator {{
        width: 16px;
        height: 16px;
        border: 1px solid {t.BORDER_STRONG};
        background-color: {t.WINDOW_SHADE_DARK};
    }}
    QCheckBox::indicator {{
        border-radius: 3px;
    }}
    QRadioButton::indicator {{
        border-radius: 8px;
    }}
    QCheckBox::indicator:checked {{
        background-color: {t.HIGHLIGHT};
        border-color: {t.HIGHLIGHT};
    }}
    QRadioButton::indicator:checked {{
        background-color: qradialgradient(cx:0.5, cy:0.5, radius:0.5, fx:0.5, fy:0.5,
            stop:0 {t.TEXT}, stop:0.35 {t.TEXT}, stop:0.45 {t.HIGHLIGHT}, stop:1 {t.HIGHLIGHT});
        border-color: {t.HIGHLIGHT};
    }}
    QCheckBox::indicator:hover, QRadioButton::indicator:hover {{
        border-color: {t.HIGHLIGHT};
    }}
    QTabWidget::pane {{
        border: 1px solid {t.BORDER};
        border-radius: {t.RADIUS}px;
        background-color: {t.WINDOW};
        top: -1px;
    }}
    QTabBar::tab {{
        background-color: {t.WINDOW_SHADE};
        color: {t.TEXT_DIM};
        border: 1px solid {t.BORDER};
        border-bottom: none;
        padding: 8px 16px;
        margin-right: 2px;
        border-top-left-radius: 6px;
        border-top-right-radius: 6px;
    }}
    QTabBar::tab:selected {{
        background-color: {t.WINDOW};
        color: {t.TEXT};
        border-color: {t.HIGHLIGHT};
    }}
    QTabBar::tab:hover:!selected {{
        color: {t.TEXT};
    }}
    QListWidget, QListView, QTreeWidget, QTreeView, QTableWidget, QTableView {{
        background-color: {t.WINDOW_SHADE_DARK};
        alternate-background-color: #2e2e2e;
        color: {t.TEXT};
        border: 1px solid {t.BORDER};
        border-radius: 6px;
        selection-background-color: {t.HIGHLIGHT};
        selection-color: {t.TEXT};
        gridline-color: #3a3a3a;
        outline: none;
    }}
    QListWidget::item, QListView::item {{
        padding: 6px 8px;
        border-radius: 4px;
    }}
    QListWidget::item:selected, QListView::item:selected {{
        background-color: {t.HIGHLIGHT};
        color: {t.TEXT};
    }}
    QHeaderView::section {{
        background-color: {t.WINDOW_SHADE};
        color: {t.TEXT_DIM};
        border: none;
        border-right: 1px solid {t.BORDER};
        border-bottom: 1px solid {t.BORDER};
        padding: 6px 8px;
        font-weight: 600;
    }}
    QTableCornerButton::section {{
        background-color: {t.WINDOW_SHADE};
        border: none;
    }}
    QProgressBar {{
        background-color: {t.WINDOW_SHADE_DARK};
        border: 1px solid {t.BORDER};
        border-radius: 6px;
        color: {t.TEXT};
        text-align: center;
        min-height: 16px;
    }}
    QProgressBar::chunk {{
        background-color: {t.HIGHLIGHT};
        border-radius: 5px;
    }}
    QSlider::groove:horizontal {{
        height: 4px;
        background: {t.BORDER};
        border-radius: 2px;
    }}
    QSlider::handle:horizontal {{
        width: 16px;
        height: 16px;
        margin: -6px 0;
        border-radius: 8px;
        background: {t.HIGHLIGHT};
    }}
    QSplitter::handle {{
        background-color: {t.BORDER};
        height: 4px;
    }}
    QTextEdit, QPlainTextEdit {{
        font-family: "__MONO_FAMILY__", "Consolas", "Courier New", monospace;
        font-size: 12px;
    }}
    QScrollBar:vertical {{
        background: transparent;
        width: 8px;
        margin: 0;
    }}
    QScrollBar::handle:vertical {{
        background: rgba(255, 255, 255, 70);
        min-height: 24px;
        border-radius: 4px;
    }}
    QScrollBar::handle:vertical:hover {{
        background: rgba(255, 255, 255, 120);
    }}
    QScrollBar:horizontal {{
        background: transparent;
        height: 8px;
        margin: 0;
    }}
    QScrollBar::handle:horizontal {{
        background: rgba(255, 255, 255, 70);
        min-width: 24px;
        border-radius: 4px;
    }}
    QScrollBar::add-line, QScrollBar::sub-line {{
        width: 0px;
        height: 0px;
        border: none;
        background: transparent;
    }}
    QScrollBar::add-page, QScrollBar::sub-page {{
        background: transparent;
    }}
    QMessageBox {{
        background-color: {t.WINDOW};
    }}
    QMessageBox QLabel {{
        color: {t.TEXT};
        font-size: {t.FONT_BODY}px;
    }}
    """.replace("__MONO_FAMILY__", mono_family)
    return _scale_px(css, ui_scale)
