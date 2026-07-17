"""VGCS application entrypoint."""

import os
import sys
from pathlib import Path

from PySide6.QtWidgets import QApplication

from vgcs.app.gcs_style import gcs_stylesheet
from vgcs.app.main_window import MainWindow
from vgcs.app.runtime_ui import (
    apply_qt_scale_override,
    build_base_font,
    configure_high_dpi_policy,
    select_font_profile,
)

# Kept open for the process lifetime — faulthandler needs a live file object.
_crash_log_file = None


def _enable_crash_diagnostics() -> None:
    """Dump a C-level traceback to logs/crash_trace.log on a hard native crash
    (segfault / access violation) instead of the process just vanishing with
    zero output — which is what a native cv2 tracker crash looked like in the
    field (see vgcs/observe/visual_object_tracker.py): click track, "track
    armed" printed, then the whole app closed with no exception, no
    traceback, nothing. A plain try/except cannot catch that class of
    failure — it terminates the process below Python's exception machinery —
    but faulthandler installs a low-level fault handler that can still write
    out what was running at the moment of the crash before the OS kills it.
    """
    global _crash_log_file
    try:
        import faulthandler

        log_dir = Path.cwd() / "logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        _crash_log_file = open(log_dir / "crash_trace.log", "a", encoding="utf-8")
        faulthandler.enable(file=_crash_log_file, all_threads=True)
    except Exception:
        pass


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
    import multiprocessing

    # No-op unless frozen (PyInstaller etc.) — must be the very first call if
    # it ever does matter. M14's tracker now runs in a child process
    # (vgcs/observe/visual_object_tracker.py) so a native cv2 crash can't
    # take the whole GCS down; this is standard multiprocessing hygiene for
    # a future frozen build, cheap insurance today.
    multiprocessing.freeze_support()
    _enable_crash_diagnostics()
    _apply_webengine_chromium_flags_from_env()
    # Must happen before QApplication to affect Qt layout metrics.
    apply_qt_scale_override()
    configure_high_dpi_policy()
    app = QApplication(sys.argv)
    app.setQuitOnLastWindowClosed(False)
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
