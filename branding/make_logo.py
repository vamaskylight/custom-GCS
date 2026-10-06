"""Make the VAMA GCS logo files (SVG and PNG) from the logo outlines.

Run from the repo root:

    py branding/make_logo.py
    py branding/make_logo.py --intro "vama_gcs_intro (1).html"

The second form also writes a copy of the client's 3D intro page with its
wordmark changed from "VAMA SKYLIGHT" to "VAMA GCS".

Where the shapes come from: vama_logo_outlines.json holds the drone mark and
the letters of the wordmark "VAMA SKYLIGHT", taken as they are from the intro
file the client sent on 2026-10-06. The app name is "VAMA GCS" (client,
2026-10-07). That needs a C, and the wordmark has none. Its G is a ring with a
tip at the top and a bar at the bottom, so the C here is the top half of that
ring and its mirror image: the same curve, the same stroke, the same tip.

Needs Pillow for the PNG files.
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUTLINES = HERE / "vama_logo_outlines.json"

NAME = "VAMA GCS"

ORANGE = "#E8812A"       # the mark, as in the client's intro
ON_DARK = "#DCE3EA"      # wordmark on a dark background (the intro's steel)
ON_LIGHT = "#1A212D"     # wordmark on a light background (the apps' dark slate)
DARK_BACKGROUND = "#070C14"   # the intro's background, for the preview only

# Gaps the wordmark does not have, because it has no "GC" and no "CS".
# Chosen by eye next to its own gaps (S to K is 72, I to G is 98).
EXTRA_GAPS = {"GC": 84, "CS": 62}

# In the intro the wordmark is drawn at 0.92 of the mark's scale.
TEXT_SCALE_STACKED = 0.92
# Mark bottom to the top of the letters, in mark units (the intro's spacing).
STACK_GAP = 197
# Side by side: the mark is this many letter heights tall, and this far from the name.
HORIZONTAL_MARK_HEIGHTS = 2.3
HORIZONTAL_GAP_HEIGHTS = 0.75

PNG_WIDTH = 2000
SUPERSAMPLE = 4

Polygon = list[list[float]]


def load_outlines() -> dict:
    data = json.loads(OUTLINES.read_text(encoding="utf-8"))
    data["letters"]["C"] = letter_c(data["letters"]["G"])
    return data


def letter_c(g: Polygon) -> Polygon:
    """A C made from the G: the top half of its ring, and the mirror of that half.

    The G's points run from the top of the ring down its left side (0 to 9),
    round the bottom to the bar (10 to 29), back along the inside (30 to 56) and
    over the tip (57 to 64). Points 9 and 10 sit one above and one below the
    middle of the ring, on its outer edge. Points 42 and 41 do the same inside.
    """
    assert len(g) == 65 and g[9][0] == g[10][0] and g[41][0] == g[42][0], "the G is not the drawing this was written for"
    middle = (g[9][1] + g[10][1]) / 2
    assert abs((g[41][1] + g[42][1]) / 2 - middle) <= 2, "the G's ring is not symmetric"

    def mirror(point: list[float]) -> list[float]:
        return [point[0], 2 * middle - point[1]]

    outer_top = g[57:65] + g[0:10]     # from the tip, over the top, down to the left side
    inner_top = g[42:57]               # from the left side back up to the inside of the tip
    return (outer_top
            + [mirror(p) for p in reversed(outer_top)]
            + [mirror(p) for p in reversed(inner_top)]
            + inner_top)


def word(data: dict, text: str) -> tuple[list[Polygon], float]:
    """The letters of `text` set in a row. Returns the polygons and the total width, in letter units."""
    gaps = dict(data["gaps"], **EXTRA_GAPS)
    polygons: list[Polygon] = []
    x = 0.0
    previous = ""
    for char in text:
        if char == " ":
            x += data["word_space"]
            previous = ""
            continue
        outline = data["letters"][char]
        if previous:
            x += gaps[previous + char]
        polygons.append([[px + x, py] for px, py in outline])
        x += max(px for px, _ in outline)
        previous = char
    return polygons, x


def place(polygons: list[Polygon], scale: float, dx: float, dy: float) -> list[Polygon]:
    return [[[px * scale + dx, py * scale + dy] for px, py in polygon] for polygon in polygons]


class Drawing:
    """Coloured polygons on a transparent page."""

    def __init__(self, width: float, height: float) -> None:
        self.width, self.height = width, height
        self.layers: list[tuple[str, list[Polygon]]] = []

    def add(self, colour: str, polygons: list[Polygon]) -> None:
        self.layers.append((colour, polygons))

    def svg(self, title: str) -> str:
        parts = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {self.width:.0f} {self.height:.0f}" '
                 f'width="{self.width:.0f}" height="{self.height:.0f}">', f"<title>{title}</title>"]
        for colour, polygons in self.layers:
            d = " ".join("M" + " L".join(f"{x:.1f} {y:.1f}" for x, y in polygon) + " Z" for polygon in polygons)
            parts.append(f'<path fill="{colour}" d="{d}"/>')
        parts.append("</svg>")
        return "\n".join(parts) + "\n"

    def png(self, width: int, background: str | None = None):
        from PIL import Image, ImageDraw

        scale = width / self.width
        big = SUPERSAMPLE
        size = (width * big, round(self.height * scale) * big)
        image = Image.new("RGBA", size, background or (0, 0, 0, 0))
        draw = ImageDraw.Draw(image)
        for colour, polygons in self.layers:
            for polygon in polygons:
                draw.polygon([(x * scale * big, y * scale * big) for x, y in polygon], fill=colour)
        return image.resize((size[0] // big, size[1] // big), Image.LANCZOS)


def mark_only(data: dict) -> Drawing:
    width, height = data["mark_size"]
    pad = round(height * 0.06)
    drawing = Drawing(width + 2 * pad, height + 2 * pad)
    drawing.add(ORANGE, place(data["mark"], 1.0, pad, pad))
    return drawing


def stacked(data: dict, text_colour: str) -> Drawing:
    """The mark above the name, as in the intro."""
    mark_w, mark_h = data["mark_size"]
    letters, text_w = word(data, NAME)
    text_w *= TEXT_SCALE_STACKED
    text_h = data["cap_height"] * TEXT_SCALE_STACKED
    pad = round(mark_h * 0.06)
    width = max(mark_w, text_w) + 2 * pad
    drawing = Drawing(width, pad + mark_h + STACK_GAP + text_h + pad)
    drawing.add(ORANGE, place(data["mark"], 1.0, (width - mark_w) / 2, pad))
    drawing.add(text_colour, place(letters, TEXT_SCALE_STACKED, (width - text_w) / 2, pad + mark_h + STACK_GAP))
    return drawing


def horizontal(data: dict, text_colour: str) -> Drawing:
    """The mark to the left of the name, for headers."""
    mark_w, mark_h = data["mark_size"]
    cap = data["cap_height"]
    letters, text_w = word(data, NAME)
    mark_scale = HORIZONTAL_MARK_HEIGHTS * cap / mark_h
    gap = HORIZONTAL_GAP_HEIGHTS * cap
    pad = round(cap * 0.35)
    height = mark_h * mark_scale + 2 * pad
    drawing = Drawing(pad + mark_w * mark_scale + gap + text_w + pad, height)
    drawing.add(ORANGE, place(data["mark"], mark_scale, pad, pad))
    drawing.add(text_colour, place(letters, 1.0, pad + mark_w * mark_scale + gap, (height - cap) / 2))
    return drawing


def write_logo_files(data: dict) -> list[Path]:
    written: list[Path] = []
    drawings = {
        "vama-mark": (mark_only(data), "VAMA"),
        "vama-gcs-stacked-on-dark": (stacked(data, ON_DARK), NAME),
        "vama-gcs-stacked-on-light": (stacked(data, ON_LIGHT), NAME),
        "vama-gcs-horizontal-on-dark": (horizontal(data, ON_DARK), NAME),
        "vama-gcs-horizontal-on-light": (horizontal(data, ON_LIGHT), NAME),
    }
    for name, (drawing, title) in drawings.items():
        svg = HERE / f"{name}.svg"
        svg.write_text(drawing.svg(title), encoding="utf-8", newline="\n")
        png = HERE / f"{name}.png"
        drawing.png(PNG_WIDTH).save(png, optimize=True)
        written += [svg, png]
    written.append(write_preview(drawings))
    return written


def write_preview(drawings: dict) -> Path:
    """One picture with every version on the background it is meant for."""
    from PIL import Image

    tiles = []
    for name, (drawing, _title) in drawings.items():
        background = "#F3F5F8" if name.endswith("on-light") else DARK_BACKGROUND
        tile = drawing.png(900, background)
        frame = Image.new("RGBA", (980, tile.height + 80), background)
        frame.alpha_composite(tile, (40, 40))
        tiles.append(frame)
    sheet = Image.new("RGBA", (980, sum(t.height for t in tiles)), DARK_BACKGROUND)
    y = 0
    for tile in tiles:
        sheet.alpha_composite(tile, (0, y))
        y += tile.height
    path = HERE / "preview.png"
    sheet.convert("RGB").save(path, optimize=True)
    return path


def write_intro(data: dict, source: Path) -> Path:
    """A copy of the client's intro page that spells the app name instead of the company name."""
    html = source.read_text(encoding="utf-8")
    found = re.search(r'("text":)(\[.*?\])(\};\s*\n)', html, re.S)
    if found is None:
        raise SystemExit(f"{source.name} is not the intro page this was written for (no wordmark data)")
    letters, _ = word(data, NAME)
    shapes = [{"o": [[round(x), round(y)] for x, y in polygon], "h": []} for polygon in letters]
    html = html[:found.start(2)] + json.dumps(shapes, separators=(",", ":")) + html[found.end(2):]
    target = HERE / "vama_gcs_intro.html"
    target.write_text(html, encoding="utf-8", newline="\n")
    return target


def main(argv: list[str]) -> int:
    data = load_outlines()
    for path in write_logo_files(data):
        print(f"wrote {path.relative_to(HERE.parent)}  ({path.stat().st_size} bytes)")
    if "--intro" in argv:
        path = write_intro(data, Path(argv[argv.index("--intro") + 1]))
        print(f"wrote {path.relative_to(HERE.parent)}  ({path.stat().st_size} bytes)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
