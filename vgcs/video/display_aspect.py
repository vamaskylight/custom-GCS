"""The shape the video picture is drawn in: Application Settings, Video, "Aspect ratio".

"Auto" draws the picture as it comes (a 16:9 canvas for network streams, see
vgcs/video/pipeline.py, with FFmpeg's own black bars around a 4:3 camera).
"16:9", "4:3" and "1:1" draw the whole picture in that shape, as large as fits
and centred, for a camera whose stream has the wrong shape. The setting did
nothing until 2026-10-20.

Clicks on the video are measured on the drawn picture, so they keep pointing at
the same place of the camera's view in any shape.
"""

from __future__ import annotations

ASPECTS: dict[str, float] = {"16:9": 16.0 / 9.0, "4:3": 4.0 / 3.0, "1:1": 1.0}


def forced_aspect(setting: object) -> float | None:
    """Width over height for a chosen shape, or None for "Auto" (and anything unknown)."""
    return ASPECTS.get(str(setting or "").strip())


def fit_size(width: int, height: int, aspect: float) -> tuple[int, int]:
    """The largest size of that shape inside width x height, in whole pixels (at least 1 x 1)."""
    w, h = max(1, int(width)), max(1, int(height))
    if aspect <= 0:
        return w, h
    if w / h > aspect:
        return max(1, round(h * aspect)), h
    return w, max(1, round(w / aspect))


__all__ = ["ASPECTS", "fit_size", "forced_aspect"]
