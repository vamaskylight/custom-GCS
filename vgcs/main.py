"""VGCS application entrypoint."""

import os
import sys

from PySide6.QtWidgets import QApplication

from vgcs.app.gcs_style import gcs_stylesheet
from vgcs.app.main_window import MainWindow
from vgcs.app.runtime_ui import (
    apply_qt_scale_override,
    build_base_font,
    configure_high_dpi_policy,
    select_font_profile,
)


def _merge_unique_chromium_flag_tokens(*chunks: str) -> str:
    seen: set[str] = set()
    out: list[str] = []
    for chunk in chunks:
        for part in str(chunk).split():
            if not part or part in seen:
                continue
            seen.add(part)
            out.append(part)
    return " ".join(out)


def _apply_webengine_chromium_flags_from_env() -> None:
    """Configure Chromium flags for Qt WebEngine (must run before QApplication).

    - On Windows, ``--disable-gpu-compositing`` is applied by default so the map
      page composites more reliably (fewer black/ghosted regions). Opt out with:
      ``VGCS_WEBENGINE_DISABLE_STABLE_DEFAULT=1``
    - Append more switches via ``VGCS_WEBENGINE_CHROMIUM_FLAGS`` (merged into
      ``QTWEBENGINE_CHROMIUM_FLAGS`` without duplicate tokens).
    """
    key = "QTWEBENGINE_CHROMIUM_FLAGS"
    base = os.environ.get(key, "").strip()
    extra = os.environ.get("VGCS_WEBENGINE_CHROMIUM_FLAGS", "").strip()

    stable = "--disable-gpu-compositing"
    opt_out = os.environ.get("VGCS_WEBENGINE_DISABLE_STABLE_DEFAULT", "").strip().lower()
    if sys.platform == "win32" and opt_out not in ("1", "true", "yes", "on"):
        base = _merge_unique_chromium_flag_tokens(base, stable)
    base = _merge_unique_chromium_flag_tokens(base, extra)
    if base:
        os.environ[key] = base


def main() -> int:
    _apply_webengine_chromium_flags_from_env()
    # Must happen before QApplication to affect Qt layout metrics.
    apply_qt_scale_override()
    configure_high_dpi_policy()
    app = QApplication(sys.argv)
    ui_scale = 1.0
    profile = select_font_profile()
    app.setFont(build_base_font(profile, ui_scale=ui_scale))
    app.setStyle("Fusion")
    app.setStyleSheet(gcs_stylesheet(mono_family=profile.mono_family, ui_scale=ui_scale))
    win = MainWindow()
    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
