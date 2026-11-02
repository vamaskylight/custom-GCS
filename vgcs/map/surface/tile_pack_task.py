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
    heights_progress = Signal(int, int)        # done, total (the drone map's heights, after its pictures)


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


# So many tiles refused in a row: the server does not answer, and each try can wait for
# two timeouts. What came is kept, and the map stands without the rest.
HEIGHTS_GIVE_UP_AFTER = 5


class DroneMapTask(QRunnable):
    """The drone's map (vgcs/map/drone_map.py): the tiles downloaded like a pack, into
    ``pack_dir`` (VGCS's own cache of the source), then packed into one MBTiles file.
    With ``heights_plan`` the ground's heights follow the same way, from ``heights_dir``
    into a file beside the map. Reports through the same bridge; its ``finished`` carries
    a ``(PackResult, DroneMapResult)`` pair, the heights' result in ``DroneMapResult.heights``."""

    def __init__(self, plan: PackPlan, pack_dir: Path, out_path: Path, bridge: TilePackBridge,
                 cancel: threading.Event, attribution: str = "", fetch=None,
                 heights_plan: PackPlan | None = None, heights_dir: Path | None = None) -> None:
        super().__init__()
        self._plan = plan
        self._pack_dir = Path(pack_dir)
        self._out = Path(out_path)
        self._bridge = bridge
        self._cancel = cancel
        self._attribution = attribution
        self._fetch = fetch
        self._heights_plan = heights_plan
        self._heights_dir = Path(heights_dir) if heights_dir is not None else None
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
        if drone.path is not None and self._heights_plan is not None:
            drone.heights = self._make_heights()
        self._bridge.finished.emit((pack, drone), str(self._out))

    def _make_heights(self):
        from vgcs.map.drone_map import (
            DroneMapResult,
            accept_heights_tile,
            drone_heights_path,
            write_drone_heights,
        )

        last = 0.0
        refused = in_a_row = 0
        stop = threading.Event()
        out = drone_heights_path(self._out)

        def _progress(done: int, total: int, r: PackResult) -> None:
            nonlocal last, refused, in_a_row
            in_a_row = in_a_row + 1 if r.failed + r.placeholders > refused else 0
            refused = r.failed + r.placeholders
            if self._cancel.is_set() or in_a_row >= HEIGHTS_GIVE_UP_AFTER:
                stop.set()
            now = time.monotonic()
            if done == total or now - last >= 0.5:
                last = now
                self._bridge.heights_progress.emit(done, total)

        try:
            out.unlink(missing_ok=True)         # an older one, maybe of another area: the drone would take it
            download_tile_pack(self._heights_plan, self._heights_dir, progress=_progress, cancel=stop,
                               fetch=self._fetch, accept=accept_heights_tile)
            return write_drone_heights(self._heights_dir, out, self._heights_plan, cancel=self._cancel)
        except Exception as e:
            return DroneMapResult(error=f"{type(e).__name__}: {e}")
