"""QImage <-> numpy conversion for feeding live video frames into the M14
visual tracker (vgcs/observe/visual_object_tracker.py). Isolated here so the
tracker module itself stays Qt-free and testable without a QApplication.
"""

from __future__ import annotations

import numpy as np
from PySide6.QtGui import QImage


def qimage_to_bgr_array(img: QImage) -> np.ndarray | None:
    """Convert a QImage to a contiguous HxWx3 BGR uint8 array, or None if empty.

    Channel order doesn't matter to CSRT (it just needs a consistent 3-channel
    image frame to frame), so any common QImage format converts fine via
    Format_BGR888 — no need to special-case every possible input format.
    """
    if img is None or img.isNull():
        return None
    w, h = img.width(), img.height()
    if w <= 0 or h <= 0:
        return None
    conv = img.convertToFormat(QImage.Format.Format_BGR888)
    bytes_per_line = conv.bytesPerLine()
    buf = conv.constBits()
    arr = np.frombuffer(buf, dtype=np.uint8, count=bytes_per_line * h)
    arr = arr.reshape(h, bytes_per_line)[:, : w * 3].reshape(h, w, 3)
    return np.ascontiguousarray(arr)
