"""Python's garbage collection runs in the main thread, and nowhere else.

Python starts a collection of cyclic garbage in whichever thread happens to
make the object that fills its count. VGCS runs Python in other threads all
the time: the link thread, the map's tile tasks, the video. A collection in
one of them destroys there whatever has become garbage, Qt objects too, and a
Qt object has to be destroyed in its own thread. PySide moves the delete to
the main thread only for QWidget, QWindow and QQuickItem. A timer, a shortcut,
a layout or any other QObject is destroyed where the collection runs.

Found on 2026-10-14: the test suite died in five runs of six that way, a full
collection in a tile task destroying what earlier tests had left (see
tests/conftest.py). VGCS itself had the same weakness, with less garbage.

So Python's own collection is switched off, and a timer in the main thread
collects by Python's own rules (MainThreadCollector.tick).

This is about cycles only. An object whose last reference goes is still freed
at once, in the thread that lets it go.

VGCS_GC_IN_ANY_THREAD=1 leaves Python's own collection on, as before.
"""

from __future__ import annotations

import gc
import os
import time

from PySide6.QtCore import QCoreApplication, QObject, QThread, QTimer

# How often the main thread looks. A young collection of a second's garbage
# takes well under a millisecond in VGCS (measured 2026-10-14 with the window
# open), a full one about 50 ms.
INTERVAL_MS = 1000

_collector: "MainThreadCollector | None" = None


class MainThreadCollector(QObject):
    """Collects Python's cyclic garbage from a timer in its own thread (the main thread)."""

    def __init__(self, parent: QObject | None = None, *, collector=gc, interval_ms: int = INTERVAL_MS) -> None:
        super().__init__(parent)
        self._gc = collector
        self._threshold = tuple(collector.get_threshold())
        # The old generation's size after the last full collection. Python
        # collects everything only when it has grown by a quarter since then.
        self._old_after_full = len(collector.get_objects(2))
        self._was_enabled = collector.isenabled()
        self._timer = QTimer(self)
        self._timer.setInterval(max(50, int(interval_ms)))
        self._timer.timeout.connect(self.tick)
        # How many collections of each generation, and the longest one: for the log and the tests.
        self.collections = [0, 0, 0]
        self.longest_ms = 0.0

    def start(self) -> None:
        self._was_enabled = self._gc.isenabled()
        self._gc.disable()
        self._timer.start()

    def stop(self) -> None:
        """Python's own collection goes back to what it was before start()."""
        self._timer.stop()
        if self._was_enabled:
            self._gc.enable()

    def tick(self) -> int | None:
        """One look, by Python's own rules. Returns the generation collected, or None.

        As Python does it: nothing while the young generation's count is not
        over its threshold. Then the oldest generation whose count is over its
        threshold, where the old one also has to have grown by a quarter since
        the last full collection (so that a large program is not scanned whole
        every few seconds).
        """
        young, middle, old = self._gc.get_count()
        t_young, t_middle, t_old = self._threshold
        if young <= t_young:
            return None
        generation = 0
        if old > t_old:
            grown = len(self._gc.get_objects(2)) - self._old_after_full
            if grown >= self._old_after_full / 4:
                generation = 2
        if generation == 0 and middle > t_middle:
            generation = 1
        started = time.perf_counter()
        self._gc.collect(generation)
        took_ms = (time.perf_counter() - started) * 1000.0
        if generation == 2:
            self._old_after_full = len(self._gc.get_objects(2))
        self.collections[generation] += 1
        self.longest_ms = max(self.longest_ms, took_ms)
        return generation


def collect_in_main_thread_only(app: QCoreApplication | None = None) -> MainThreadCollector | None:
    """Switch Python's own collection off and collect from a timer in the main thread.

    Once a process. Call it in the main thread, after the QApplication is made.
    Returns the collector, or None when it is not used: VGCS_GC_IN_ANY_THREAD=1,
    no application yet, or not the application's thread.
    """
    global _collector
    if _collector is not None:
        return _collector
    if os.environ.get("VGCS_GC_IN_ANY_THREAD", "").strip().lower() in ("1", "true", "yes", "on"):
        print("[VGCS:gc] VGCS_GC_IN_ANY_THREAD is set: Python collects garbage in any thread", flush=True)
        return None
    app = app or QCoreApplication.instance()
    if app is None or QThread.currentThread() is not app.thread():
        return None
    _collector = MainThreadCollector(app)
    _collector.start()
    print(f"[VGCS:gc] garbage is collected in the main thread only, every {INTERVAL_MS / 1000.0:g} s", flush=True)
    return _collector
