"""Auto-restart VGCS on Python file changes."""

from __future__ import annotations

import os
import subprocess
import sys
import threading
import time
from pathlib import Path

from watchdog.events import FileSystemEvent, FileSystemEventHandler
from watchdog.observers import Observer


ROOT = Path(__file__).resolve().parents[1]
WATCH_DIRS = [ROOT / "vgcs"]

# Coalesce rapid saves (IDE auto-save, format-on-save, multi-file refactors). Without this, switching
# focus to the running app can save several open buffers and the watcher restarts the process many
# times in a row — operators often mistake that for "the 3D button restarted the app."
_DEBOUNCE_S = float(os.environ.get("VGCS_DEV_RESTART_DEBOUNCE_S", "1.8"))


class RestartHandler(FileSystemEventHandler):
    def __init__(self, restart_cb) -> None:
        super().__init__()
        self._restart_cb = restart_cb
        self._lock = threading.Lock()
        self._timer: threading.Timer | None = None
        self._pending_path: str | None = None

    @staticmethod
    def _should_ignore(path_lower: str) -> bool:
        if "__pycache__" in path_lower:
            return True
        if ".venv" in path_lower or "venv\\" in path_lower or "venv/" in path_lower:
            return True
        if "site-packages" in path_lower:
            return True
        return False

    def cancel_pending(self) -> None:
        with self._lock:
            if self._timer is not None:
                self._timer.cancel()
                self._timer = None
            self._pending_path = None

    def _fire_restart(self) -> None:
        with self._lock:
            self._timer = None
            pending = self._pending_path
            self._pending_path = None
        if pending:
            print(f"[dev] restarting after file change (debounced {_DEBOUNCE_S:.1f}s): {pending}")
        else:
            print(f"[dev] restarting after file change (debounced {_DEBOUNCE_S:.1f}s)")
        self._restart_cb()

    def on_any_event(self, event: FileSystemEvent) -> None:
        try:
            path = str(event.src_path)
        except Exception:
            return
        pl = path.lower()
        # Reload on .py (app code) and .html (vendored Leaflet/Cesium page); HTML is read once at
        # page load, so editing legacy_leaflet_map.html requires a full restart to take effect.
        if not (pl.endswith(".py") or pl.endswith(".html")):
            return
        if self._should_ignore(pl):
            return
        with self._lock:
            self._pending_path = path
            if self._timer is not None:
                self._timer.cancel()
            self._timer = threading.Timer(_DEBOUNCE_S, self._fire_restart)
            self._timer.daemon = True
            self._timer.start()


class Runner:
    def __init__(self) -> None:
        self.proc: subprocess.Popen[str] | None = None

    def start(self) -> None:
        env = os.environ.copy()
        self.proc = subprocess.Popen([sys.executable, "-m", "vgcs"], cwd=str(ROOT), env=env)
        print(f"[dev] started pid={self.proc.pid}")

    def stop(self) -> None:
        if self.proc is None:
            return
        if self.proc.poll() is not None:
            self.proc = None
            return
        self.proc.terminate()
        try:
            self.proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.proc.kill()
        self.proc = None

    def restart(self) -> None:
        self.stop()
        self.start()


def main() -> int:
    runner = Runner()
    runner.start()

    observer = Observer()
    handler = RestartHandler(runner.restart)
    for d in WATCH_DIRS:
        observer.schedule(handler, str(d), recursive=True)
    observer.start()
    print("[dev] watching:", ", ".join(str(d) for d in WATCH_DIRS))
    print(f"[dev] restart debounce: {_DEBOUNCE_S:.1f}s (set VGCS_DEV_RESTART_DEBOUNCE_S to override)")
    print("[dev] press Ctrl+C to stop")

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        handler.cancel_pending()
        observer.stop()
        observer.join()
        runner.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
