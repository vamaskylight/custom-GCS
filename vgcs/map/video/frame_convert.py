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

    Field-confirmed use-after-free: the returned array MUST own its own
    memory, never a view into ``conv``'s pixel buffer. ``conv`` is a local
    QImage that goes out of scope the moment this function returns, and
    ``np.ascontiguousarray()`` is a no-op — NOT a copy — whenever its input
    is already contiguous, which it always is here for any frame whose
    width makes ``bytes_per_line`` equal ``w*3`` exactly (e.g. 960, this
    codebase's actual companion decode width — see
    ``_companion_decode_max_dims`` in vgcs/video/pipeline.py). Whether
    Qt/PySide6's ``constBits()`` buffer keeps the QImage's underlying data
    alive past that point is a binding/platform detail we can't rely on —
    on one field machine it evidently didn't, and the caller (M14's tracker,
    vgcs/observe/visual_object_tracker.py) crashed with a bare Windows
    access violation reading through this array's memory well after this
    function had already returned. ``.copy()`` unconditionally allocates
    fresh, independently-owned memory — the only way to be certain there is
    no reference to Qt-managed memory left in the returned array, on any
    platform/binding version.
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
    return arr.copy()
