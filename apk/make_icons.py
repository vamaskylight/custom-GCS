"""Make every VAMA APK icon and logo from one source image.

Run from the repo root:

    python apk/make_icons.py
    python apk/make_icons.py path/to/new-logo.png

The default source is vgcs/assets/Vama Logo.png (orange mark on a
transparent background). Square icons put the mark on the dark VGCS
background colour, so they read well on any launcher.

Needs Pillow.
"""

from __future__ import annotations

import base64
import io
import sys
from pathlib import Path

from PIL import Image, ImageDraw

Image.MAX_IMAGE_PIXELS = None  # the source logo is 15171 x 6793

REPO = Path(__file__).resolve().parents[1]
CUSTOM = REPO / "apk" / "custom"
DEFAULT_SOURCE = REPO / "vgcs" / "assets" / "Vama Logo.png"

BACKGROUND = (0x25, 0x2A, 0x35, 255)  # VGCS dark window colour
MARK_WIDTH = 0.80  # share of a square icon's width taken by the mark

ANDROID_SIZES = {"ldpi": 36, "mdpi": 48, "hdpi": 72, "xhdpi": 96, "xxhdpi": 144, "xxxhdpi": 192}
ICO_SIZES = [16, 20, 24, 32, 40, 48, 64, 128, 256]


def load_mark(path: Path) -> Image.Image:
    """The logo cropped to its visible pixels, at a workable size."""
    im = Image.open(path).convert("RGBA")
    bbox = im.getbbox()
    if bbox:
        im = im.crop(bbox)
    if im.width > 2048:
        im = im.resize((2048, round(im.height * 2048 / im.width)), Image.LANCZOS)
    return im


def square_icon(mark: Image.Image, size: int) -> Image.Image:
    """The mark centred on the dark background, as a square."""
    canvas = Image.new("RGBA", (size, size), BACKGROUND)
    w = max(1, round(size * MARK_WIDTH))
    h = max(1, round(mark.height * w / mark.width))
    m = mark.resize((w, h), Image.LANCZOS)
    canvas.alpha_composite(m, ((size - w) // 2, (size - h) // 2))
    return canvas


def banner(mark: Image.Image, width: int, height: int) -> Image.Image:
    """The mark fitted inside a dark banner (Windows installer header)."""
    canvas = Image.new("RGBA", (width, height), BACKGROUND)
    scale = min(width * 0.9 / mark.width, height * 0.8 / mark.height)
    m = mark.resize((max(1, round(mark.width * scale)), max(1, round(mark.height * scale))), Image.LANCZOS)
    canvas.alpha_composite(m, ((width - m.width) // 2, (height - m.height) // 2))
    return canvas


def white_mark(mark: Image.Image, width: int) -> Image.Image:
    """The mark in plain white, keeping its shape (alpha).

    QGC tints menu icons by their shape only, so the colour does not matter
    there; white also works anywhere the icon is shown as it is.
    """
    m = mark.resize((width, round(mark.height * width / mark.width)), Image.LANCZOS)
    white = Image.new("RGBA", m.size, (255, 255, 255, 0))
    white.putalpha(m.getchannel("A"))
    return white


def position_arrow(mark: Image.Image, size: int) -> Image.Image:
    """Map marker for the phone or RC position when its heading is known.

    Same shape as QGC's QGCLogoArrow.svg (an arrowhead pointing up; QGC turns
    it to the heading): dark fill, orange edge, the mark inside.
    """
    scale = 4  # draw large, then shrink, for smooth edges
    s = size * scale
    canvas = Image.new("RGBA", (s, s), (0, 0, 0, 0))
    draw = ImageDraw.Draw(canvas)
    points = [(s * 0.50, s * 0.03), (s * 0.97, s * 0.97), (s * 0.50, s * 0.76), (s * 0.03, s * 0.97)]
    edge = mark.getpixel((mark.width // 2, int(mark.height * 0.95)))  # the mark's orange
    draw.polygon(points, fill=BACKGROUND, outline=edge[:3] + (255,), width=round(s * 0.035))
    w = round(s * 0.46)
    m = mark.resize((w, round(mark.height * w / mark.width)), Image.LANCZOS)
    canvas.alpha_composite(m, ((s - w) // 2, round(s * 0.52) - m.height // 2))
    return canvas.resize((size, size), Image.LANCZOS)


def svg_wrapping(png: Image.Image) -> str:
    """An SVG that embeds a PNG, for places QGC expects an SVG file."""
    buf = io.BytesIO()
    png.save(buf, format="PNG", optimize=True)
    data = base64.b64encode(buf.getvalue()).decode("ascii")
    w, h = png.size
    return (
        f'<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" '
        f'width="{w}" height="{h}" viewBox="0 0 {w} {h}">\n'
        f'  <image width="{w}" height="{h}" xlink:href="data:image/png;base64,{data}"/>\n'
        f"</svg>\n"
    )


def main() -> int:
    source = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_SOURCE
    mark = load_mark(source)
    written = []

    for density, size in ANDROID_SIZES.items():
        out = CUSTOM / "android" / "res" / f"drawable-{density}" / "icon.png"
        square_icon(mark, size).save(out, optimize=True)
        written.append(out)

    icon256 = square_icon(mark, 256)
    out = CUSTOM / "res" / "icons" / "custom_qgroundcontrol.png"
    square_icon(mark, 128).save(out, optimize=True)
    written.append(out)
    for out in (CUSTOM / "res" / "icons" / "custom_qgroundcontrol.ico",
                CUSTOM / "deploy" / "windows" / "WindowsQGC.ico"):
        icon256.save(out, sizes=[(s, s) for s in ICO_SIZES])
        written.append(out)
    try:
        out = CUSTOM / "res" / "icons" / "custom_qgroundcontrol.icns"
        square_icon(mark, 1024).save(out)
        written.append(out)
    except Exception as e:  # .icns is only for macOS builds, which we do not ship
        print(f"skipped .icns: {e}")

    out = CUSTOM / "res" / "icons" / "custom_qgroundcontrol.svg"
    out.write_text(svg_wrapping(icon256), encoding="utf-8")
    written.append(out)

    # Full logo shown inside the app: the mark alone, transparent background.
    logo = mark.resize((600, round(mark.height * 600 / mark.width)), Image.LANCZOS)
    out = CUSTOM / "res" / "Images" / "QGCLogoFull.svg"
    out.write_text(svg_wrapping(logo), encoding="utf-8")
    written.append(out)

    # Menu icon in place of QGC's logo (Settings button; QGC tints it).
    out = CUSTOM / "res" / "Images" / "vama_logo_white.svg"
    out.write_text(svg_wrapping(white_mark(mark, 300)), encoding="utf-8")
    written.append(out)

    # Position marker with heading, in place of QGC's QGCLogoArrow.svg.
    out = CUSTOM / "res" / "Images" / "QGCLogoArrow.svg"
    out.write_text(svg_wrapping(position_arrow(mark, 160)), encoding="utf-8")
    written.append(out)

    out = CUSTOM / "deploy" / "windows" / "installheader.bmp"
    banner(mark, 150, 57).convert("RGB").save(out)
    written.append(out)

    for p in written:
        print(f"wrote {p.relative_to(REPO)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
