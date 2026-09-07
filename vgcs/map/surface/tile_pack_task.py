"""Run a tile-pack download off the GUI thread and report back.

Same shape as tile_probe: a QRunnable on the global pool, a QObject bridge
whose signals cross to the main thread. Progress is throttled here so a
7000-tile pack does not post 7000 status updates.
"""

from __future__ import annotations

import threading
import time
from pathlib import Path

from PySide6.QtCore import QObject, QRunnable, Signal

from vgcs.map.tile_pack import PackPlan, PackResult, download_tile_pack


class TilePackBridge(QObject):
    progress = Signal(int, int, int, int)      # done, total, stored, failed
    finished = Signal(object, str)             # PackResult, dest


class TilePackTask(QRunnable):
    def __init__(self, plan: PackPlan, dest: Path, bridge: TilePackBridge,
                 cancel: threading.Event) -> None:
        super().__init__()
        self._plan = plan
        self._dest = Path(dest)
        self._bridge = bridge
        self._cancel = cancel
        self.setAutoDelete(True)

    def run(self) -> None:  # pragma: no cover - network dependent
        last = 0.0

        def _progress(done: int, total: int, r: PackResult) -> None:
            nonlocal last
            now = time.monotonic()
            if done == total or now - last >= 0.5:
                last = now
                self._bridge.progress.emit(done, total, r.stored, r.failed)

        try:
            result = download_tile_pack(
                self._plan, self._dest, progress=_progress, cancel=self._cancel
            )
        except Exception as e:
            result = PackResult(failed=1, failures=[f"{type(e).__name__}: {e}"])
        self._bridge.finished.emit(result, str(self._dest))
