"""VGCS application entrypoint."""

import os
import signal
import sys
from pathlib import Path

from PySide6.QtCore import QCoreApplication, QTimer
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


def install_ctrl_c_handler(app, win, *, tick_ms: int = 250) -> "QTimer | None":
    """Make Ctrl+C in the console close VGCS cleanly.

    Two things stop it from working on its own. Python only notices a pending
    SIGINT while it is executing Python code, and inside ``app.exec()`` the
    process sits in Qt's C++ event loop, so the keypress waits for the next
    slot to run - and a KeyboardInterrupt raised inside a Qt slot does not stop
    the loop anyway. So: a handler that closes the window (disconnect, save
    geometry, stop the camera backend) and quits the app, plus a timer whose
    only job is to hand control back to Python often enough for the handler to
    run. A second Ctrl+C while shutdown is still in progress exits hard.
    """
    state = {"presses": 0}

    def _on_sigint(_signum, _frame) -> None:
        state["presses"] += 1
        if state["presses"] >= 2:
            print("[VGCS] Ctrl+C again - exiting now", flush=True)
            os._exit(130)
        print("[VGCS] Ctrl+C - closing VGCS", flush=True)
        try:
            win.close()
        except Exception:
            pass
        try:
            app.quit()
        except Exception:
            pass

    try:
        signal.signal(signal.SIGINT, _on_sigint)
    except (ValueError, OSError):
        return None  # not the main thread / unsupported: leave the default
    if QCoreApplication.instance() is None:
        return None
    timer = QTimer()
    timer.setInterval(max(50, int(tick_ms)))
    timer.timeout.connect(lambda: None)
    timer.start()
    return timer


def main() -> int:
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
    _ctrl_c_tick = install_ctrl_c_handler(app, win)  # keep a reference: a dropped QTimer stops
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
