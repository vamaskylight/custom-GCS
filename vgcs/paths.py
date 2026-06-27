"""Stable paths to shipped VGCS package resources."""

from __future__ import annotations

from pathlib import Path

_VGCS_PKG = Path(__file__).resolve().parent


def vgcs_assets_dir() -> Path:
    """Shipped assets: logos, header/menu icons, plan templates, tile seed, etc."""
    return _VGCS_PKG / "assets"
