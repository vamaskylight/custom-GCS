"""Small Qt image helpers shared by map / observation code."""

from __future__ import annotations

from pathlib import Path

from PySide6.QtGui import QImage


def save_qimage_to_path(img: QImage, path: Path) -> bool:
    """Write ``QImage`` using extension: ``.png`` → PNG, else JPEG."""
    try:
        if path.suffix.lower() == ".png":
            return bool(img.save(str(path), "PNG"))
        return bool(img.save(str(path), "JPG", 92))
    except Exception:
        return False
