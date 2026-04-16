"""Runtime UI hardening for cross-machine consistency."""

from __future__ import annotations

from dataclasses import dataclass

from PySide6.QtCore import Qt
from PySide6.QtGui import QFont, QFontDatabase, QGuiApplication


@dataclass(frozen=True)
class UiFontProfile:
    """Selected font families for UI and monospace surfaces."""

    ui_family: str
    mono_family: str


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


def build_base_font(profile: UiFontProfile) -> QFont:
    """Create a readable baseline font that survives style fallback."""
    font = QFont(profile.ui_family)
    font.setPointSize(10)
    font.setHintingPreference(QFont.HintingPreference.PreferFullHinting)
    return font

