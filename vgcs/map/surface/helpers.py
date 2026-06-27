"""Small helpers for map tile / web fallback paths."""

from __future__ import annotations

import os


def _web_2d_fallback_allowed() -> bool:
    """2D map is NativeTileMapView only; WebEngine Leaflet is opt-in for debugging."""
    return str(os.environ.get("VGCS_ALLOW_WEB_2D_FALLBACK", "0") or "0").strip() == "1"
