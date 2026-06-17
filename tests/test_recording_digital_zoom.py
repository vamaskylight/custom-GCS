from __future__ import annotations

import numpy as np

from vgcs.video.pipeline import apply_digital_zoom_rgb24


def _solid_rgb24(w: int, h: int, rgb: tuple[int, int, int]) -> bytes:
    arr = np.zeros((h, w, 3), dtype=np.uint8)
    arr[:, :] = rgb
    return arr.tobytes()


def test_apply_digital_zoom_rgb24_identity_at_1x() -> None:
    raw = _solid_rgb24(4, 4, (10, 20, 30))
    assert apply_digital_zoom_rgb24(raw, 4, 4, 1.0) == raw


def test_apply_digital_zoom_rgb24_crops_center_region() -> None:
    w = h = 6
    arr = np.zeros((h, w, 3), dtype=np.uint8)
    arr[2:4, 2:4, :] = (255, 0, 0)
    raw = arr.tobytes()
    out = apply_digital_zoom_rgb24(raw, w, h, 3.0)
    out_arr = np.frombuffer(out, dtype=np.uint8).reshape((h, w, 3))
    assert out_arr[0, 0, 0] == 255
    assert out_arr[-1, -1, 0] == 255
