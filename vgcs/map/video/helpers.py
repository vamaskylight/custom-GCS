"""Video UI helpers shared by map video mixins."""

from __future__ import annotations



def _format_video_zoom_label(z: float) -> str:
    return f"{max(1.0, float(z)):.1f}x"
