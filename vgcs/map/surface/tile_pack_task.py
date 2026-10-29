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


class DroneMapTask(QRunnable):
    """The drone's map (vgcs/map/drone_map.py): the tiles downloaded like a pack, into
    ``pack_dir`` (VGCS's own cache of the source), then packed into one MBTiles file.
    Reports through the same bridge; its ``finished`` carries a ``(PackResult,
    DroneMapResult)`` pair."""

    def __init__(self, plan: PackPlan, pack_dir: Path, out_path: Path, bridge: TilePackBridge,
                 cancel: threading.Event, attribution: str = "", fetch=None) -> None:
        super().__init__()
        self._plan = plan
        self._pack_dir = Path(pack_dir)
        self._out = Path(out_path)
        self._bridge = bridge
        self._cancel = cancel
        self._attribution = attribution
        self._fetch = fetch
        self.setAutoDelete(True)

    def run(self) -> None:
        from vgcs.map.drone_map import DroneMapResult, write_drone_map

        last = 0.0

        def _progress(done: int, total: int, r: PackResult) -> None:
            nonlocal last
            now = time.monotonic()
            if done == total or now - last >= 0.5:
                last = now
                self._bridge.progress.emit(done, total, r.stored, r.failed)

        try:
            pack = download_tile_pack(self._plan, self._pack_dir, progress=_progress, cancel=self._cancel,
                                      fetch=self._fetch)
        except Exception as e:
            pack = PackResult(failed=1, failures=[f"{type(e).__name__}: {e}"])
        drone = DroneMapResult(cancelled=pack.cancelled)
        if not pack.cancelled:
            try:
                drone = write_drone_map(self._pack_dir, self._out, self._plan, attribution=self._attribution,
                                        cancel=self._cancel)
            except Exception as e:
                drone = DroneMapResult(error=f"{type(e).__name__}: {e}")
        self._bridge.finished.emit((pack, drone), str(self._out))
