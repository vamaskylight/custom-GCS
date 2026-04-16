"""Runtime UI hardening for cross-machine consistency."""

from __future__ import annotations

import os
from dataclasses import dataclass

from PySide6.QtCore import Qt
from PySide6.QtGui import QFont, QFontDatabase, QGuiApplication


@dataclass(frozen=True)
class UiFontProfile:
    """Selected font families for UI and monospace surfaces."""

    ui_family: str
    mono_family: str


def detect_ui_scale() -> float:
    """Derive a conservative UI scale and allow manual override."""
    override = os.getenv("VGCS_UI_SCALE", "").strip()
    if override:
        try:
            parsed = float(override)
            return max(0.85, min(parsed, 2.0))
        except ValueError:
            pass

    screen = QGuiApplication.primaryScreen()
    if screen is None:
        return 1.0

    logical_dpi = float(screen.logicalDotsPerInch() or 96.0)
    width = int(screen.geometry().width() or 0)
    scale = logical_dpi / 96.0

    # On large FHD+ screens at 100%, fixed-pixel QSS can look too small.
    if scale <= 1.01 and width >= 1920:
        scale = 1.15

    return max(0.9, min(scale, 1.6))


def configure_high_dpi_policy() -> None:
    """Respect OS scaling to avoid tiny UI on high-DPI screens."""
    QGuiApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough
    )


def select_font_profile() -> UiFontProfile:
    """Pick best-available UI and monospace fonts from local machine."""
    families = {name.lower(): name for name in QFontDatabase().families()}

    ui_candidates = ["Segoe UI", "Inter", "Noto Sans", "Arial", "Sans Serif"]
    mono_candidates = [
        "Cascadia Mono",
        "Consolas",
        "JetBrains Mono",
        "Courier New",
        "Monospace",
    ]

    def pick(candidates: list[str]) -> str:
        for fam in candidates:
            found = families.get(fam.lower())
            if found:
                return found
        return candidates[-1]

    return UiFontProfile(ui_family=pick(ui_candidates), mono_family=pick(mono_candidates))


def build_base_font(profile: UiFontProfile, ui_scale: float = 1.0) -> QFont:
    """Create a readable baseline font that survives style fallback."""
    font = QFont(profile.ui_family)
    font.setPointSize(max(10, int(round(10 * ui_scale))))
    font.setHintingPreference(QFont.HintingPreference.PreferFullHinting)
    return font

