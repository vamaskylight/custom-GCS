"""One look for every VGCS screen, taken from the Vama GCS app (2026-11-09).

The developer asked for VGCS to look like the app. The app is QGroundControl
with our layer on top: round icon buttons on translucent black rails over the
map and the video, white line icons, orange for a switch that is on, and QGC's
dark colours in its windows. The values here are the app's
(apk/custom/src/FlyViewCustomLayer.qml) and QGC's dark palette
(QGCPalette.cc), so the two products look alike.

Screens take their colours, sizes and icons from here. A colour written
anywhere else is a colour that drifts from the rest.
"""

from __future__ import annotations

from functools import cache, lru_cache
from pathlib import Path

from PySide6.QtCore import QByteArray, QSize, Qt
from PySide6.QtGui import QColor, QIcon, QPainter, QPixmap
from PySide6.QtWidgets import QPushButton, QSizePolicy, QWidget

from vgcs.paths import vgcs_assets_dir

try:
    from PySide6.QtSvg import QSvgRenderer
except ImportError:  # optional component on some Qt builds
    QSvgRenderer = None  # type: ignore[misc, assignment]

# --- Colours: QGC's dark palette -----------------------------------------------------------------

WINDOW = "#222222"  # QGC window
WINDOW_SHADE = "#333333"  # QGC windowShade: panels and group boxes
WINDOW_SHADE_DARK = "#282828"  # QGC windowShadeDark: fields and lists
BORDER = "#4a4a4a"
BORDER_STRONG = "#707070"  # QGC groupBorder
TEXT = "#ffffff"
TEXT_DIM = "#bfbfbf"  # QGC colorGrey
TEXT_FAINT = "#8c8c8c"
BUTTON = "#3d3d45"
BUTTON_HOVER = "#4b4b55"
BUTTON_PRESSED = "#2d2d33"
BUTTON_BORDER = "#56565f"
HIGHLIGHT = "#3a9bdc"  # QGC buttonHighlight: chosen, the main button of a window
HIGHLIGHT_HOVER = "#4fabe8"
ORANGE = "#de8500"  # QGC colorOrange: a switch that is on, a warning
GREEN = "#00e04b"  # QGC colorGreen: good
RED = "#f32836"  # QGC colorRed: danger
RED_TEXT = "#f85761"  # QGC warningText
YELLOW = "#f2bf1a"  # the app's torch

# The header bar takes the flight state's colour, as before, in deeper tones.
HEADER_STATES = {
    # state: (background, bottom line, text, faint text)
    "green": ("rgba(18, 92, 44, 245)", "rgba(0, 224, 75, 230)", "#eaffef", "rgba(234, 255, 239, 150)"),
    "yellow": ("rgba(122, 82, 8, 245)", "rgba(222, 133, 0, 240)", "#fff6e0", "rgba(255, 246, 224, 150)"),
    "red": ("rgba(128, 20, 26, 245)", "rgba(243, 40, 54, 240)", "#ffecec", "rgba(255, 236, 236, 150)"),
    "": ("rgba(34, 34, 34, 250)", "rgba(255, 255, 255, 40)", "#ffffff", "rgba(255, 255, 255, 140)"),
}

# --- The app's overlay: rails and round buttons over the map and the video ------------------------

OVERLAY_RAIL = "rgba(0, 0, 0, 102)"  # the app's panel colour, Qt.rgba(0, 0, 0, 0.4)
OVERLAY_CARD = "rgba(0, 0, 0, 150)"  # a box of numbers or words: a little darker, to read over a map
OVERLAY_BUTTON = "rgba(0, 0, 0, 140)"  # the app's button, 0.55
OVERLAY_BUTTON_HOVER = "rgba(70, 70, 70, 170)"
OVERLAY_BUTTON_PRESSED = "rgba(255, 255, 255, 89)"  # the app's pressed button, 0.35
OVERLAY_BORDER = "rgba(255, 255, 255, 77)"  # the app's button border, 0.3
OVERLAY_BORDER_HOVER = "rgba(255, 255, 255, 140)"
OVERLAY_DISABLED_BUTTON = "rgba(0, 0, 0, 70)"
OVERLAY_DISABLED_BORDER = "rgba(255, 255, 255, 34)"
OVERLAY_DISABLED_TEXT = "rgba(255, 255, 255, 115)"
RECORDING = "rgba(217, 31, 31, 230)"  # the app's record button while it records
TORCH_ON = "rgba(242, 191, 26, 230)"  # the app's torch button while the torch is on
ACTIVE = ORANGE  # the app's IR, lock and laser buttons while they are on
GOOD_FILL = "rgba(0, 150, 64, 235)"  # Follow Me running (the app: green)
INFO_FILL = "rgba(58, 155, 220, 235)"  # a window of the button is open (the app: buttonHighlight)

# --- Sizes ---------------------------------------------------------------------------------------

FONT_SMALL = 11
FONT_BODY = 13
FONT_MEDIUM = 15
FONT_LARGE = 18
FONT_TITLE = 22
FONT_FAMILY = '"Segoe UI", "Inter", "Noto Sans", Arial, sans-serif'

BUTTON_SIZE = 44  # the app's round button (2.4 font heights) on a laptop
BUTTON_SIZE_SMALL = 36
SHUTTER_SIZE = 54
RAIL_MARGIN = 8
RAIL_SPACING = 8
ICON_RATIO = 0.52  # the app draws its icon at 52 percent of the button
RADIUS = 8  # cards, boxes in windows
DISABLED_ICON_OPACITY = 0.45  # the app's disabled button

# Tones of a round button that is on (checked). Plain buttons turn orange like the app's.
TONE_FILLS = {
    "": (ACTIVE, "rgba(255, 210, 140, 230)"),
    "record": (RECORDING, "rgba(255, 150, 150, 230)"),
    "torch": (TORCH_ON, "rgba(255, 236, 170, 230)"),
    "go": (GOOD_FILL, "rgba(150, 255, 190, 230)"),
    "info": (INFO_FILL, "rgba(190, 228, 255, 230)"),
    "plain": (OVERLAY_BUTTON_PRESSED, "rgba(255, 255, 255, 200)"),
}


def ui_icons_dir() -> Path:
    """The line icons: the app's own (vama_*.svg) and VGCS's in the same style."""
    return vgcs_assets_dir() / "ui_icons"


@cache
def _svg_text(name: str) -> str:
    path = ui_icons_dir() / f"{name}.svg"
    try:
        return path.read_text(encoding="utf-8")
    except OSError:
        return ""


def icon_exists(name: str) -> bool:
    return bool(_svg_text(name))


# The pixmaps are drawn three times larger than shown, so the icon stays sharp
# on a screen scaled to 125, 150 or 200 percent.
_ICON_OVERSAMPLE = 3


@lru_cache(maxsize=512)
def icon_pixmap(name: str, px: int = 24, color: str = TEXT, opacity: float = 1.0) -> QPixmap:
    """The icon ``name`` at ``px`` logical pixels in ``color``. An empty pixmap if it cannot be drawn."""
    text = _svg_text(name)
    side = max(1, int(px))
    if not text or QSvgRenderer is None:
        return QPixmap()
    if color.lower() != "#ffffff":
        text = text.replace("#ffffff", color)
    renderer = QSvgRenderer(QByteArray(text.encode("utf-8")))
    if not renderer.isValid():
        return QPixmap()
    pm = QPixmap(side * _ICON_OVERSAMPLE, side * _ICON_OVERSAMPLE)
    pm.fill(Qt.GlobalColor.transparent)
    painter = QPainter(pm)
    painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
    painter.setOpacity(max(0.0, min(1.0, float(opacity))))
    renderer.render(painter)
    painter.end()
    pm.setDevicePixelRatio(float(_ICON_OVERSAMPLE))
    return pm


def icon(name: str, px: int = 24, color: str = TEXT) -> QIcon:
    """A QIcon of the line icon, dimmed like the app's buttons when its button is disabled."""
    normal = icon_pixmap(name, px, color)
    result = QIcon()
    if normal.isNull():
        return result
    result.addPixmap(normal, QIcon.Mode.Normal, QIcon.State.Off)
    result.addPixmap(normal, QIcon.Mode.Active, QIcon.State.Off)
    result.addPixmap(normal, QIcon.Mode.Selected, QIcon.State.Off)
    result.addPixmap(icon_pixmap(name, px, color, DISABLED_ICON_OPACITY), QIcon.Mode.Disabled, QIcon.State.Off)
    for mode in (QIcon.Mode.Normal, QIcon.Mode.Active, QIcon.Mode.Selected):
        result.addPixmap(normal, mode, QIcon.State.On)
    result.addPixmap(icon_pixmap(name, px, color, DISABLED_ICON_OPACITY), QIcon.Mode.Disabled, QIcon.State.On)
    return result


def svg_file_icon(path: Path, px: int = 20, *, recolor: dict[str, str] | None = None) -> QIcon:
    """An icon from any SVG file, with its colours swapped (for example dark menu icons made white)."""
    try:
        text = Path(path).read_text(encoding="utf-8")
    except OSError:
        return QIcon()
    for old, new in (recolor or {}).items():
        text = text.replace(old, new).replace(old.upper(), new)
    if QSvgRenderer is None:
        return QIcon()
    renderer = QSvgRenderer(QByteArray(text.encode("utf-8")))
    if not renderer.isValid():
        return QIcon()
    side = max(1, int(px))
    pm = QPixmap(side * _ICON_OVERSAMPLE, side * _ICON_OVERSAMPLE)
    pm.fill(Qt.GlobalColor.transparent)
    painter = QPainter(pm)
    painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
    renderer.render(painter)
    painter.end()
    pm.setDevicePixelRatio(float(_ICON_OVERSAMPLE))
    return QIcon(pm)


def menu_qss() -> str:
    """A menu in the app's dark look, with room for an icon on the left."""
    return (
        f"QMenu {{ background-color: {WINDOW_SHADE}; color: {TEXT}; border: 1px solid {BORDER};"
        f" border-radius: {RADIUS}px; padding: 6px 4px; font-family: {FONT_FAMILY}; font-size: {FONT_BODY}px; }}"
        "QMenu::item { background-color: transparent; margin: 1px 4px; padding: 9px 22px 9px 38px; border-radius: 6px; }"
        f"QMenu::item:selected {{ background-color: {HIGHLIGHT}; color: {TEXT}; }}"
        f"QMenu::item:disabled {{ color: {TEXT_FAINT}; }}"
        "QMenu::icon { left: 12px; }"
        f"QMenu::separator {{ height: 1px; margin: 6px 10px; background: {BORDER}; }}"
        f"QMenu::right-arrow {{ width: 10px; height: 10px; }}"
    )


def record_dot_pixmap(px: int = 18, color: str = "#ff3b3b") -> QPixmap:
    """The red dot of a record or shutter button, sharp at any scale."""
    side = max(2, int(px))
    pm = QPixmap(side * _ICON_OVERSAMPLE, side * _ICON_OVERSAMPLE)
    pm.fill(Qt.GlobalColor.transparent)
    painter = QPainter(pm)
    painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
    painter.setPen(Qt.PenStyle.NoPen)
    painter.setBrush(QColor(color))
    painter.drawEllipse(0, 0, side * _ICON_OVERSAMPLE, side * _ICON_OVERSAMPLE)
    painter.end()
    pm.setDevicePixelRatio(float(_ICON_OVERSAMPLE))
    return pm


# --- Round buttons ---------------------------------------------------------------------------------


def round_button_qss(size: int = BUTTON_SIZE, *, tone: str = "", text_px: int | None = None) -> str:
    """The app's round button: translucent black, a thin white ring, filled when it is on."""
    s = int(size)
    fill_on, border_on = TONE_FILLS.get(tone, TONE_FILLS[""])
    tpx = int(text_px) if text_px else max(FONT_SMALL, min(FONT_MEDIUM, s // 3))
    return (
        "QPushButton {"
        f" min-width: {s - 2}px; max-width: {s - 2}px; min-height: {s - 2}px; max-height: {s - 2}px;"
        f" border-radius: {s // 2}px; border: 1px solid {OVERLAY_BORDER};"
        f" background-color: {OVERLAY_BUTTON}; color: {TEXT}; padding: 0px; margin: 0px;"
        f" font-family: {FONT_FAMILY}; font-size: {tpx}px; font-weight: 700; outline: none;"
        "}"
        f"QPushButton:hover {{ background-color: {OVERLAY_BUTTON_HOVER}; border-color: {OVERLAY_BORDER_HOVER}; }}"
        f"QPushButton:pressed {{ background-color: {OVERLAY_BUTTON_PRESSED}; }}"
        f"QPushButton:checked {{ background-color: {fill_on}; border-color: {border_on}; }}"
        f"QPushButton:checked:hover {{ background-color: {fill_on}; border-color: {TEXT}; }}"
        f"QPushButton:disabled {{ background-color: {OVERLAY_DISABLED_BUTTON};"
        f" border-color: {OVERLAY_DISABLED_BORDER}; color: {OVERLAY_DISABLED_TEXT}; }}"
    )


def style_round_button(
    btn: QPushButton,
    *,
    icon_name: str | None = None,
    size: int = BUTTON_SIZE,
    tone: str = "",
    text_px: int | None = None,
) -> QPushButton:
    """Give ``btn`` the app's round look. Its text stays when there is no icon (for example "F+")."""
    s = int(size)
    btn.setProperty("vamaRound", True)
    btn.setProperty("vamaTone", tone)
    btn.setFixedSize(s, s)
    btn.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
    btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
    btn.setCursor(Qt.CursorShape.PointingHandCursor)
    if icon_name:
        btn.setProperty("vamaIcon", icon_name)  # which icon, for tests and for anyone reading the window
        side = max(12, round(s * ICON_RATIO))
        ic = icon(icon_name, side)
        if not ic.isNull():
            btn.setIcon(ic)
            btn.setIconSize(QSize(side, side))
            btn.setText("")
    btn.setStyleSheet(round_button_qss(s, tone=tone, text_px=text_px))
    return btn


def round_button(
    parent: QWidget | None = None,
    *,
    object_name: str = "",
    icon_name: str | None = None,
    text: str = "",
    size: int = BUTTON_SIZE,
    tooltip: str = "",
    checkable: bool = False,
    tone: str = "",
) -> QPushButton:
    """A new round button in the app's look."""
    btn = QPushButton(text, parent)
    if object_name:
        btn.setObjectName(object_name)
    btn.setCheckable(bool(checkable))
    if tooltip:
        btn.setToolTip(tooltip)
    return style_round_button(btn, icon_name=icon_name, size=size, tone=tone)


# --- Pills and panels ----------------------------------------------------------------------------


def pill_button_qss(height: int = 32, *, tone: str = "", text_px: int = FONT_BODY) -> str:
    """A text button over the map or the video: the round button's colours, rounded ends."""
    h = int(height)
    fill_on, border_on = TONE_FILLS.get(tone, TONE_FILLS[""])
    return (
        "QPushButton {"
        f" min-height: {h - 2}px; max-height: {h - 2}px; border-radius: {h // 2}px;"
        f" border: 1px solid {OVERLAY_BORDER}; background-color: {OVERLAY_BUTTON}; color: {TEXT};"
        f" padding: 0px 12px; font-family: {FONT_FAMILY}; font-size: {int(text_px)}px; font-weight: 600;"
        " outline: none;"
        "}"
        f"QPushButton:hover {{ background-color: {OVERLAY_BUTTON_HOVER}; border-color: {OVERLAY_BORDER_HOVER}; }}"
        f"QPushButton:pressed {{ background-color: {OVERLAY_BUTTON_PRESSED}; }}"
        f"QPushButton:checked {{ background-color: {fill_on}; border-color: {border_on}; }}"
        f"QPushButton:disabled {{ background-color: {OVERLAY_DISABLED_BUTTON};"
        f" border-color: {OVERLAY_DISABLED_BORDER}; color: {OVERLAY_DISABLED_TEXT}; }}"
    )


def rail_qss(object_name: str, radius: int, *, background: str = OVERLAY_RAIL, border: bool = False) -> str:
    """A translucent panel over the map or the video, like the app's rails."""
    edge = f"1px solid {OVERLAY_BORDER}" if border else "none"
    return (
        f"QFrame#{object_name} {{ background-color: {background}; border: {edge};"
        f" border-radius: {int(radius)}px; }}"
    )


# The flight buttons on the map's left edge: one strip like QGC's tool strip.
FLIGHT_BUTTON_W = 60
FLIGHT_BUTTON_H = 50
FLIGHT_ICON_SIZE = 22
FLIGHT_RAIL_PADDING = 5
FLIGHT_RAIL_SPACING = 3


def flight_rail_qss() -> str:
    """The strip and its buttons: translucent like the app's rails, a lighter press, white names."""
    r = 14
    return (
        f"QFrame#mapActionRail {{ background-color: {OVERLAY_CARD}; border: 1px solid {OVERLAY_DISABLED_BORDER};"
        f" border-radius: {r}px; }}"
        "QFrame#mapActionRail QToolButton {"
        f" background-color: transparent; border: 1px solid transparent; border-radius: 10px; color: {TEXT};"
        f" font-family: {FONT_FAMILY}; font-size: {FONT_SMALL}px; font-weight: 600; padding: 3px 0px 2px 0px;"
        "}"
        "QFrame#mapActionRail QToolButton:hover { background-color: rgba(255, 255, 255, 30);"
        " border-color: rgba(255, 255, 255, 60); }"
        f"QFrame#mapActionRail QToolButton:pressed {{ background-color: {OVERLAY_BUTTON_PRESSED}; }}"
        f"QFrame#mapActionRail QToolButton:disabled {{ color: {OVERLAY_DISABLED_TEXT}; }}"
    )


HEADER_BUTTON_HEIGHT = 32
ESTOP_RED = "#c62828"


def header_qss(state: str) -> str:
    """The header bar in one of its flight states ("green", "yellow", "red", or "" for none).

    QGC's dark toolbar, tinted by the state as VGCS always did, with pill buttons:
    Connect filled, Disconnect and 3D outlined, E-STOP red in every state.
    """
    bg, line, fg, faint = HEADER_STATES.get(state, HEADER_STATES[""])
    h = HEADER_BUTTON_HEIGHT
    r = h // 2
    font = (
        f" font-family: {FONT_FAMILY}; font-size: {FONT_BODY}px; font-weight: 700;"
        f" min-height: {h - 2}px; max-height: {h - 2}px;"
    )
    outline = (
        f" color: {fg}; background-color: rgba(255, 255, 255, 18); border: 1px solid rgba(255, 255, 255, 90);"
        f" border-radius: {r}px; padding: 0px 16px;" + font
    )
    return "\n".join(
        (
            f"QFrame#headerBar {{ background-color: {bg}; border: none; border-bottom: 2px solid {line}; }}",
            f"QLabel#hdrPillTitle {{ color: {faint}; }}",
            f"QLabel#hdrPillValue {{ color: {fg}; font-size: {FONT_MEDIUM}px; font-weight: 600; }}",
            f"QLabel#hdrGpsStackLine {{ color: {fg}; font-size: {FONT_BODY}px; font-weight: 600; }}",
            f"QFrame#hdrSep {{ background-color: {faint}; border: none; }}",
            (
                "QPushButton#headerFlightChipBtn { background: transparent; border: none;"
                f" color: {fg}; font-weight: 700; font-size: {FONT_MEDIUM}px; padding: 2px 6px; }}"
            ),
            "QPushButton#headerFlightChipBtn:hover { background-color: rgba(255, 255, 255, 20); border-radius: 6px; }",
            f"QLabel#linkBannerText {{ color: {fg}; font-size: {FONT_MEDIUM}px; font-weight: 600; }}",
            (
                f"QPushButton#hdrConnectBtn {{ color: {TEXT}; background-color: {HIGHLIGHT};"
                f" border: 1px solid {HIGHLIGHT_HOVER}; border-radius: {r}px; padding: 0px 18px;" + font + " }"
            ),
            f"QPushButton#hdrConnectBtn:hover {{ background-color: {HIGHLIGHT_HOVER}; }}",
            f"QPushButton#hdrDisconnectBtn, QPushButton#hdrMapModeBtn {{{outline} }}",
            (
                "QPushButton#hdrDisconnectBtn:hover, QPushButton#hdrMapModeBtn:hover {"
                " background-color: rgba(255, 255, 255, 45); border-color: rgba(255, 255, 255, 170); }"
            ),
            (
                "QPushButton#hdrConnectBtn:disabled, QPushButton#hdrDisconnectBtn:disabled,"
                " QPushButton#hdrMapModeBtn:disabled { background-color: rgba(255, 255, 255, 12);"
                " color: rgba(255, 255, 255, 110); border: 1px solid rgba(255, 255, 255, 40); }"
            ),
            (
                f"QPushButton#hdrEstopBtn {{ color: {TEXT}; background-color: {ESTOP_RED}; border: 1px solid #ff8a80;"
                f" border-radius: {r}px; padding: 0px 18px; font-family: {FONT_FAMILY}; font-size: {FONT_BODY}px;"
                f" font-weight: 800; letter-spacing: 1px; min-height: {h - 2}px; max-height: {h - 2}px; }}"
            ),
            "QPushButton#hdrEstopBtn:hover:enabled { background-color: #e53935; border-color: #ffcdd2; }",
            (
                "QPushButton#hdrEstopBtn:disabled { background-color: rgba(198, 40, 40, 90);"
                " color: rgba(255, 255, 255, 130); border-color: rgba(255, 138, 128, 90); }"
            ),
            (
                "QPushButton#hdrSilenceAlarmBtn { color: #1b1b1b; background-color: #f9a825; border: 1px solid #ffe082;"
                f" border-radius: {r}px; padding: 0px 16px;" + font + " }"
            ),
            "QPushButton#hdrSilenceAlarmBtn:hover { background-color: #fbc02d; }",
        )
    )


# --- The camera panels over the map and the video ---------------------------------------------------

CAMERA_BUTTON = 40  # the gimbal pad's arrows and the zoom
CAMERA_SMALL_BUTTON = 36  # the camera bar's switches
CAMERA_PILL_HEIGHT = 32  # Track, Follow, Observe, and the Observe panel's buttons
CAMERA_PANEL_GAP = 8  # between the bar, the pad and the Observe panel
CAMERA_PANEL_RADIUS = 22
OBSERVE_PANEL_WIDTH = 248  # two buttons a row: Target and Clip, Report and Reset, Set TGT and Set HIT


def record_icon(px: int = 20) -> QIcon:
    """The record button: a red dot, and a white stop square while it records (its red fill then)."""
    result = QIcon()
    dot = record_dot_pixmap(px)
    result.addPixmap(dot, QIcon.Mode.Normal, QIcon.State.Off)
    result.addPixmap(dot, QIcon.Mode.Active, QIcon.State.Off)
    side = max(2, int(px))
    stop = QPixmap(side * _ICON_OVERSAMPLE, side * _ICON_OVERSAMPLE)
    stop.fill(Qt.GlobalColor.transparent)
    painter = QPainter(stop)
    painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
    painter.setPen(Qt.PenStyle.NoPen)
    painter.setBrush(QColor(TEXT))
    inset = side * _ICON_OVERSAMPLE * 0.22
    edge = side * _ICON_OVERSAMPLE - 2 * inset
    painter.drawRoundedRect(int(inset), int(inset), int(edge), int(edge), 3 * _ICON_OVERSAMPLE, 3 * _ICON_OVERSAMPLE)
    painter.end()
    stop.setDevicePixelRatio(float(_ICON_OVERSAMPLE))
    result.addPixmap(stop, QIcon.Mode.Normal, QIcon.State.On)
    result.addPixmap(stop, QIcon.Mode.Active, QIcon.State.On)
    faded = record_dot_pixmap(px, "#7a3030")
    result.addPixmap(faded, QIcon.Mode.Disabled, QIcon.State.Off)
    return result


def shutter_icon(px: int = 22) -> QIcon:
    """The photo shutter: a white disc, like a camera app's."""
    return QIcon(record_dot_pixmap(px, TEXT))


def camera_panels_qss() -> str:
    """The camera bar, the gimbal pad and the Observe panel, and the text buttons in them.

    Round buttons carry their own sheet (style_round_button); this styles the panels, the zoom value,
    the record timer, and the text pills: Track, Follow, Observe and the Observe panel's buttons.
    """
    h = CAMERA_PILL_HEIGHT
    r = h // 2
    pill = (
        f" min-height: {h - 2}px; max-height: {h - 2}px; border-radius: {r}px; border: 1px solid {OVERLAY_BORDER};"
        f" background-color: {OVERLAY_BUTTON}; color: {TEXT}; padding: 0px 12px;"
        f" font-family: {FONT_FAMILY}; font-size: {FONT_BODY}px; font-weight: 600; outline: none;"
    )
    pills = (
        "QPushButton#observeTarget, QPushButton#observeClip, QPushButton#observeReport, QPushButton#observeReset,"
        " QPushButton#observeRoleTarget, QPushButton#observeRoleImpact, QPushButton#observeLaser,"
        " QPushButton#observeDooafSetup, QPushButton#m13Track, QPushButton#followObject,"
        " QPushButton#camObserveBtn"
    )
    hovers = ", ".join(f"{name.strip()}:hover" for name in pills.split(","))
    return "\n".join(
        (
            "QFrame#nativeCameraRailLayer { background: transparent; border: none; }",
            (
                "QFrame#nativeCameraRail, QFrame#cameraGimbalPanel {"
                f" background-color: {OVERLAY_RAIL}; border: 1px solid rgba(255, 255, 255, 26);"
                f" border-radius: {CAMERA_PANEL_RADIUS}px; }}"
            ),
            (
                "QFrame#cameraObservePanel {"
                f" background-color: {OVERLAY_CARD}; border: 1px solid rgba(255, 255, 255, 34); border-radius: 16px; }}"
            ),
            (
                "QFrame#cameraTopRow, QWidget#cameraModeRow, QWidget#cameraFocusRow, QWidget#m13TrackBlock,"
                " QWidget#cameraObserveHeader { background: transparent; border: none; }"
            ),
            (
                f"QLabel#camMagnificationLabel {{ color: {TEXT}; background: transparent; border: none;"
                f" font-family: {FONT_FAMILY}; font-size: {FONT_BODY}px; font-weight: 700; }}"
            ),
            (
                f"QLabel#camTimer {{ color: {TEXT}; background-color: {OVERLAY_BUTTON}; border: 1px solid {OVERLAY_BORDER};"
                f" border-radius: 11px; padding: 2px 8px; font-family: {FONT_FAMILY}; font-size: {FONT_SMALL + 1}px;"
                " font-weight: 600; }"
            ),
            (
                f"QLabel#camPanelTitle {{ color: {TEXT}; background: transparent; border: none;"
                f" font-family: {FONT_FAMILY}; font-size: {FONT_BODY}px; font-weight: 700; }}"
            ),
            (
                f"QLabel#observeDooafHint, QLabel#m13TrackHint {{ color: {TEXT_DIM}; background: transparent;"
                f" font-size: {FONT_SMALL}px; font-weight: 500; }}"
            ),
            f"{pills} {{{pill}}}",
            f"{hovers} {{ background-color: {OVERLAY_BUTTON_HOVER}; border-color: {OVERLAY_BORDER_HOVER}; }}",
            (
                f"QPushButton#observeTarget:checked, QPushButton#m13Track:checked {{ background-color: {ACTIVE};"
                " border-color: rgba(255, 210, 140, 230); }"
            ),
            (
                f"QPushButton#observeRoleTarget:checked, QPushButton#observeRoleImpact:checked,"
                f" QPushButton#camObserveBtn:checked {{ background-color: {INFO_FILL};"
                " border-color: rgba(190, 228, 255, 230); }"
            ),
            # A pressed switch must look pressed (tests/test_dooaf_laser_hit.py reads this rule).
            (
                f"QPushButton#observeLaser:checked {{ background-color: {ACTIVE}; border-color: rgba(255, 210, 140, 230);"
                " color: #ffffff; }"
            ),
            f"QPushButton#followObject:checked {{ background-color: {GOOD_FILL}; border-color: rgba(150, 255, 190, 230); }}",
            (
                f'QPushButton#observeClip[recording="true"] {{ background-color: {RECORDING};'
                " border-color: rgba(255, 150, 150, 230); font-weight: 700; }"
            ),
            (
                "QPushButton#observeDooafSetup { background-color: rgba(58, 155, 220, 150);"
                " border-color: rgba(190, 228, 255, 150); }"
            ),
            f"QPushButton#observeDooafSetup:hover {{ background-color: {INFO_FILL}; }}",
            "QPushButton:disabled { color: rgba(255, 255, 255, 115); }",
        )
    )


def overlay_label_qss(px: int = FONT_BODY, *, weight: int = 600, color: str = TEXT) -> str:
    return (
        f"color: {color}; font-family: {FONT_FAMILY}; font-size: {int(px)}px; font-weight: {int(weight)};"
        " background: transparent; border: none;"
    )
