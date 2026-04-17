"""Auto-restart VGCS on Python file changes."""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from pathlib import Path

from watchdog.events import FileSystemEvent, FileSystemEventHandler
from watchdog.observers import Observer


ROOT = Path(__file__).resolve().parents[1]
WATCH_DIRS = [ROOT / "vgcs"]


class RestartHandler(FileSystemEventHandler):
    def __init__(self, restart_cb) -> None:
        super().__init__()
        self._restart_cb = restart_cb
        self._last = 0.0

    def on_any_event(self, event: FileSystemEvent) -> None:
        path = event.src_path.lower()
        if not path.endswith(".py"):
            return
        now = time.monotonic()
        if now - self._last < 0.4:
            return
        self._last = now
        print(f"[dev] change detected: {event.src_path}")
        self._restart_cb()


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
    print("[dev] press Ctrl+C to stop")

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        observer.stop()
        observer.join()
        runner.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

