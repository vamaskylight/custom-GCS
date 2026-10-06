"""Thermal colour modes (pseudo-colour) for the thermal picture.

Milestone M12, client requirement 27: "Thermal frame display with palette
switching (e.g. White Hot, Black Hot, Ironbow ...). Works with the agreed
thermal source or a recorded loop."

A recorded loop takes no camera commands, and each camera family has its own
palette command or none at all (Skydroid lists IMG for the C12 only), so the
colours are applied here, to the decoded picture. Every camera gets the same
modes, and a change shows on the next frame.

The modes map brightness to colour, so they expect the camera's plain
white-hot picture, which is its default. "As received" leaves the picture
alone, for a camera that already sends colours.
"""

from __future__ import annotations

from functools import lru_cache

from PySide6.QtGui import QImage, qRgb

# The picture exactly as the camera sends it (white hot by default).
AS_RECEIVED = "camera"

# id -> (name on screen, gradient stops from coldest to hottest).
# A stop is (position 0..1, (red, green, blue)).
_Stop = tuple[float, tuple[int, int, int]]
_PALETTES: dict[str, tuple[str, tuple[_Stop, ...]]] = {
    AS_RECEIVED: ("White hot (as received)", ()),
    "black_hot": ("Black hot", ((0.0, (255, 255, 255)), (1.0, (0, 0, 0)))),
    "ironbow": (
        "Ironbow",
        (
            (0.00, (0, 0, 0)),
            (0.15, (36, 0, 104)),
            (0.32, (122, 0, 152)),
            (0.48, (196, 32, 104)),
            (0.63, (236, 88, 24)),
            (0.78, (252, 160, 0)),
            (0.91, (255, 228, 72)),
            (1.00, (255, 255, 255)),
        ),
    ),
    "rainbow": (
        "Rainbow",
        (
            (0.00, (0, 0, 96)),
            (0.20, (0, 64, 255)),
            (0.40, (0, 220, 220)),
            (0.55, (0, 220, 0)),
            (0.72, (255, 235, 0)),
            (0.88, (255, 60, 0)),
            (1.00, (255, 255, 255)),
        ),
    ),
    # White hot, with the hottest part picked out in red.
    "red_hot": (
        "Red hot",
        ((0.00, (0, 0, 0)), (0.75, (191, 191, 191)), (0.90, (255, 128, 0)), (1.00, (255, 0, 0))),
    ),
    "green": ("Green", ((0.00, (0, 0, 0)), (0.65, (0, 208, 64)), (1.00, (208, 255, 208)))),
    "sepia": ("Sepia", ((0.00, (0, 0, 0)), (0.50, (152, 104, 56)), (1.00, (255, 240, 204)))),
}


def palette_ids() -> tuple[str, ...]:
    """Every mode, in the order the menu shows them."""
    return tuple(_PALETTES)


def palette_label(palette_id: str) -> str:
    return _PALETTES[normalize_palette_id(palette_id)][0]


def normalize_palette_id(value: object) -> str:
    """A saved or typed value as a known mode; anything else is "as received"."""
    pid = str(value or "").strip().lower()
    return pid if pid in _PALETTES else AS_RECEIVED


@lru_cache(maxsize=None)
def palette_colors(palette_id: str) -> tuple[tuple[int, int, int], ...]:
    """The 256 colours of a mode, from brightness 0 (coldest) to 255 (hottest).

    "As received" has none: it is not a mapping. Ask for its grey ramp with
    ``swatch_colors`` when a picture of it is needed.
    """
    stops = _PALETTES[normalize_palette_id(palette_id)][1]
    if not stops:
        return ()
    out: list[tuple[int, int, int]] = []
    for level in range(256):
        t = level / 255.0
        lo, hi = stops[0], stops[-1]
        for a, b in zip(stops, stops[1:]):
            if a[0] <= t <= b[0]:
                lo, hi = a, b
                break
        span = hi[0] - lo[0]
        f = 0.0 if span <= 0 else (t - lo[0]) / span
        out.append(tuple(int(round(lo[1][i] + (hi[1][i] - lo[1][i]) * f)) for i in range(3)))
    return tuple(out)


def swatch_colors(palette_id: str) -> tuple[tuple[int, int, int], ...]:
    """256 colours to draw a mode with (a grey ramp for "as received")."""
    return palette_colors(palette_id) or tuple((v, v, v) for v in range(256))


@lru_cache(maxsize=None)
def _color_table(palette_id: str) -> tuple[int, ...]:
    return tuple(qRgb(r, g, b) for r, g, b in palette_colors(palette_id))


def apply_palette(image: QImage, palette_id: str) -> QImage:
    """The picture in the given colour mode. "As received" returns it untouched.

    The work is three Qt conversions (to grey, to an indexed image whose colour
    table is the mode, back to RGB), so it costs about a millisecond a frame and
    no Python loop touches a pixel.
    """
    pid = normalize_palette_id(palette_id)
    if pid == AS_RECEIVED or image is None or image.isNull():
        return image
    grey = image
    if grey.format() != QImage.Format.Format_Grayscale8:
        grey = image.convertToFormat(QImage.Format.Format_Grayscale8)
    indexed = grey.convertToFormat(QImage.Format.Format_Indexed8)
    if indexed.isNull() or indexed.colorCount() != 256:
        return image  # not the grey table this relies on: leave the picture alone
    indexed.setColorTable(list(_color_table(pid)))
    return indexed.convertToFormat(QImage.Format.Format_RGB888)
