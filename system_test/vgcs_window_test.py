"""The whole VGCS window against the real ArduCopter simulator, and how well it keeps up (milestone M17).

Usage, from the repo root:

    py system_test/vgcs_window_test.py                    ArduCopter 4.7.0, a 3 minute hover at real speed
    py system_test/vgcs_window_test.py 4.6.2 --minutes 5
    py system_test/vgcs_window_test.py --speedup 10       ten times the message rate, a stress test
    py system_test/vgcs_window_test.py --report out.md
    py system_test/vgcs_window_test.py --profile prof.txt  where the window's thread spends its time

What it does: builds the real main window (off-screen), types the simulator's
address into the connection box and presses Connect, then flies through the
window's own button handlers: take-off, a mode change, a hover, a radio
silence, landing. A second connection to the simulator checks what really
happened, and the window's own labels are read to check what the operator
would have seen.

While the drone hovers it measures:
- how late the window's event loop runs (a 20 ms timer: how late each tick is),
- the time the window spends in its telemetry handler, per message,
- the CPU time of VGCS (this process, less the checking connection's thread),
- memory (private bytes) at the start and the end of the hover.

The map's web view runs in separate QtWebEngine processes, which are not
counted here.

The user's own VGCS settings are never touched: every QSettings in this
process is sent to an INI file in a temporary folder before VGCS is imported.
"""

from __future__ import annotations

import ctypes
import math
import os
import pathlib
import re
import statistics
import sys
import tempfile
import threading
import time
from ctypes import wintypes

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("VGCS_NO_NETWORK", "1")
os.environ.setdefault("QT_QPA_FONTDIR", "C:/Windows/Fonts")

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "drone" / "test"))

# --- the user's settings stay untouched ------------------------------------
from PySide6 import QtCore  # noqa: E402

SETTINGS_DIR = pathlib.Path(tempfile.mkdtemp(prefix="vgcs-window-test-"))
_RealQSettings = QtCore.QSettings


class _TestSettings(_RealQSettings):
    """QSettings("VGCS", "VGCS") and friends write to the Windows registry.
    Here every one of them is an INI file in a temporary folder."""

    def __init__(self, *args, **kwargs):
        if args and isinstance(args[0], str) and not args[0].lower().endswith(".ini"):
            org = args[0]
            app = args[1] if len(args) > 1 and isinstance(args[1], str) else "default"
            super().__init__(str(SETTINGS_DIR / f"{org}-{app}.ini"), _RealQSettings.Format.IniFormat)
        elif not args:
            super().__init__(str(SETTINGS_DIR / "default.ini"), _RealQSettings.Format.IniFormat)
        else:
            super().__init__(*args, **kwargs)


QtCore.QSettings = _TestSettings

from PySide6.QtCore import QEventLoop, QTimer  # noqa: E402
from PySide6.QtWidgets import QApplication, QMessageBox  # noqa: E402

from sitl_session import Sitl, wsl  # noqa: E402

VGCS_PORT = "tcp:127.0.0.1:5762"


# --- measuring ---------------------------------------------------------------

class _MemoryCounters(ctypes.Structure):
    _fields_ = [
        ("cb", wintypes.DWORD), ("PageFaultCount", wintypes.DWORD),
        ("PeakWorkingSetSize", ctypes.c_size_t), ("WorkingSetSize", ctypes.c_size_t),
        ("QuotaPeakPagedPoolUsage", ctypes.c_size_t), ("QuotaPagedPoolUsage", ctypes.c_size_t),
        ("QuotaPeakNonPagedPoolUsage", ctypes.c_size_t), ("QuotaNonPagedPoolUsage", ctypes.c_size_t),
        ("PagefileUsage", ctypes.c_size_t), ("PeakPagefileUsage", ctypes.c_size_t),
        ("PrivateUsage", ctypes.c_size_t),
    ]


_kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
_kernel32.GetCurrentProcess.restype = wintypes.HANDLE
_kernel32.K32GetProcessMemoryInfo.argtypes = [wintypes.HANDLE, ctypes.POINTER(_MemoryCounters), wintypes.DWORD]
_kernel32.K32GetProcessMemoryInfo.restype = wintypes.BOOL


def private_mb() -> float:
    counters = _MemoryCounters()
    counters.cb = ctypes.sizeof(counters)
    if not _kernel32.K32GetProcessMemoryInfo(_kernel32.GetCurrentProcess(), ctypes.byref(counters), counters.cb):
        raise OSError(ctypes.get_last_error(), "GetProcessMemoryInfo failed")
    return counters.PrivateUsage / (1024 * 1024)


class LagMeter:
    """A 20 ms timer on the window's thread. How late each tick comes is how long the window was busy."""

    PERIOD_MS = 20

    def __init__(self) -> None:
        self.late_ms: list[float] = []
        self._last = time.perf_counter()
        self.recording = False
        self.timer = QTimer()
        self.timer.setTimerType(QtCore.Qt.TimerType.PreciseTimer)
        self.timer.timeout.connect(self._tick)
        self.timer.start(self.PERIOD_MS)

    def _tick(self) -> None:
        now = time.perf_counter()
        if self.recording:
            self.late_ms.append(max(0.0, (now - self._last) * 1000.0 - self.PERIOD_MS))
        self._last = now


class HandlerClock:
    """Wraps the window's telemetry slot and times every call."""

    def __init__(self, window) -> None:
        self.times: dict[str, list[float]] = {}
        self.recording = False
        inner = window._on_telemetry

        def timed(kind, payload):
            t0 = time.perf_counter()
            inner(kind, payload)
            if self.recording:
                self.times.setdefault(kind, []).append((time.perf_counter() - t0) * 1000.0)

        window._on_telemetry = timed       # before Connect, so the link connects to this


class Observer:
    """The checking connection, read in its own thread so the window's thread is the window's alone."""

    def __init__(self, sim: Sitl) -> None:
        self.sim = sim
        self.running = True
        self.cpu_s = 0.0
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self) -> None:
        start = time.thread_time()
        while self.running:
            try:
                msg = self.sim.mav.recv_match(blocking=True, timeout=0.2)
            except Exception:
                time.sleep(0.05)
                continue
            if msg is not None:
                self.sim._take(msg)
            self.cpu_s = time.thread_time() - start

    def stop(self) -> None:
        self.running = False
        self.thread.join(timeout=3)

    def rc(self, **channels: int) -> None:
        values = [65535] * 8
        for key, pwm in channels.items():
            values[int(key[2:]) - 1] = pwm
        for _ in range(3):
            self.sim.mav.mav.rc_channels_override_send(self.sim.mav.target_system, self.sim.mav.target_component, *values)
            time.sleep(0.05)


def wait_qt(condition, seconds: float) -> bool:
    """Run the window's event loop, as the real app does, until the condition holds."""
    if condition():
        return True
    loop = QEventLoop()
    end = time.monotonic() + seconds
    check = QTimer()

    def poll() -> None:
        if condition() or time.monotonic() >= end:
            loop.quit()

    check.timeout.connect(poll)
    check.start(50)
    loop.exec()
    check.stop()
    return bool(condition())


def spread(values: list[float]) -> str:
    if not values:
        return "no samples"
    ordered = sorted(values)

    def pick(q: float) -> float:
        return ordered[min(len(ordered) - 1, int(q * len(ordered)))]

    return (f"median {statistics.median(ordered):.2f} ms, 95% {pick(0.95):.2f} ms, "
            f"99% {pick(0.99):.2f} ms, worst {ordered[-1]:.1f} ms ({len(ordered)} samples)")


class Report:
    def __init__(self) -> None:
        self.lines: list[tuple[bool, str]] = []
        self.numbers: list[str] = []

    def check(self, ok, text: str) -> bool:
        self.lines.append((bool(ok), text))
        print(("    ok    " if ok else "    FAIL  ") + text)
        sys.stdout.flush()
        return bool(ok)

    def note(self, text: str) -> None:
        self.numbers.append(text)
        print("    ..    " + text)
        sys.stdout.flush()

    @property
    def failed(self) -> int:
        return sum(1 for ok, _ in self.lines if not ok)


def make_offline_tiles(root: pathlib.Path) -> int:
    """A folder of map tiles around VGCS's start view and the simulator's home.

    The window runs with VGCS's own offline map, so the map really draws tiles
    and the tile loader really runs, with nothing fetched from the internet.
    Each tile is a pattern of coloured squares: a flat grey one would be taken
    for Esri's "no imagery" tile and thrown away.
    """
    import random

    from PySide6.QtGui import QColor, QImage, QPainter

    from vgcs.map.native_tile_map import _tile_image_is_placeholder, _tile_xy

    made = 0
    for lat, lon in ((37.7749, -122.4194), (20.4347, 72.8696)):
        for z in range(10, 18):
            cx, cy = _tile_xy(lat, lon, z)
            for x in range(cx - 5, cx + 6):
                for y in range(cy - 4, cy + 5):
                    path = root / str(z) / str(x) / f"{y}.png"
                    if path.is_file():
                        continue
                    rnd = random.Random(hash((z, x, y)))
                    img = QImage(256, 256, QImage.Format.Format_RGB32)
                    painter = QPainter(img)
                    for by in range(0, 256, 32):
                        for bx in range(0, 256, 32):
                            painter.fillRect(bx, by, 32, 32, QColor(rnd.randrange(40, 200), rnd.randrange(60, 200), rnd.randrange(30, 160)))
                    painter.end()
                    assert not _tile_image_is_placeholder(img, zoom=z)
                    path.parent.mkdir(parents=True, exist_ok=True)
                    img.save(str(path), "PNG")
                    made += 1
    return made


def banner(w) -> str:
    label = getattr(w, "_link_banner_text", None)
    return label.text() if label is not None else ""


# --- the session ---------------------------------------------------------------

def session(version: str, minutes: float, speedup: int, r: Report, profile_to: pathlib.Path | None = None) -> None:
    app = QApplication.instance() or QApplication(sys.argv[:1])
    import vgcs.app.flight_action_dialogs as dialogs
    from vgcs.app.main_window import MainWindow
    from vgcs.app.vehicle_params import RENAMED_IN_4_7

    # The confirmation popups answer themselves: the operator said yes.
    dialogs.ask_takeoff_altitude = lambda _parent, alt: 15.0
    dialogs.confirm_land = lambda _parent: True
    popups: list[str] = []
    QMessageBox.warning = staticmethod(lambda _p, title, text, *a, **k: popups.append(f"{title}: {text}") or QMessageBox.StandardButton.Ok)
    QMessageBox.information = staticmethod(lambda _p, title, text, *a, **k: popups.append(f"{title}: {text}") or QMessageBox.StandardButton.Ok)
    # Questions (closing or disconnecting with a drone armed) are answered yes
    # and counted, so a failed run never waits on one.
    QMessageBox.question = staticmethod(lambda _p, title, text, *a, **k: popups.append(f"{title}: {text}") or QMessageBox.StandardButton.Yes)

    # The map draws from a local tile folder, never the internet (see make_offline_tiles).
    from vgcs.map.app_settings import QS_APP, QS_ORG
    from vgcs.map.surface.settings_keys import _KEY_MAP_OFFLINE_TILE_ROOT, _KEY_MAP_TILE_MODE

    tiles = SETTINGS_DIR / "tiles"
    r.note(f"offline map tiles made for the test: {make_offline_tiles(tiles)} (in {tiles})")
    store = QtCore.QSettings(QS_ORG, QS_APP)
    store.setValue(_KEY_MAP_TILE_MODE, "offline")
    store.setValue(_KEY_MAP_OFFLINE_TILE_ROOT, str(tiles))
    store.sync()

    sim = Sitl(version, {}, {"RC_OVERRIDE_TIME": -1, "SIM_WIND_SPD": 0}, speedup=speedup)
    observer = Observer(sim)
    # Every request the map's tile loader makes, to prove none goes to the internet.
    from vgcs.map import native_tile_map

    internet: list[str] = []
    real_request = native_tile_map._NativeTileLoader.request

    def watched_request(self, z, x, y, url, **kwargs):
        if str(url).lower().startswith(("http://", "https://")):
            internet.append(str(url))
        return real_request(self, z, x, y, url, **kwargs)

    native_tile_map._NativeTileLoader.request = watched_request
    w = MainWindow()
    w.resize(1600, 900)
    w.show()
    lag = LagMeter()
    clock = HandlerClock(w)
    logs: list[str] = []
    real_append = w._append_log
    w._append_log = lambda line: (logs.append(str(line)), real_append(line))[1]
    banners: list[tuple[float, str]] = []      # every banner the operator was shown
    real_banner = w._set_dashboard_flight_status
    w._set_dashboard_flight_status = lambda state, message: (banners.append((time.monotonic(), str(message))), real_banner(state, message))[1]
    try:
        wait_qt(lambda: False, 5.0)
        # Idle: the window open, nothing connected. What it costs by itself.
        cpu_start, wall_start, observer_cpu_start = time.process_time(), time.monotonic(), observer.cpu_s
        lag.recording = True
        wait_qt(lambda: False, 20.0)
        lag.recording = False
        idle_wall = time.monotonic() - wall_start
        idle_cpu = (time.process_time() - cpu_start) - (observer.cpu_s - observer_cpu_start)
        r.note(f"idle (window open, not connected): CPU {100.0 * idle_cpu / idle_wall:.0f} % of one core, "
               f"event loop {spread(lag.late_ms)}")
        lag.late_ms.clear()

        w._conn_edit.setText(VGCS_PORT)
        w._btn_connect.click()
        r.check(wait_qt(lambda: w._heartbeat_seen, 30), f"Connect: the window says {w._status.text()!r}")
        r.check(wait_qt(lambda: w._top_flight_mode.text() == sim.mode, 10),
                f"the header shows the drone's mode ({w._top_flight_mode.text()!r}, the drone is in {sim.mode})")
        # On screen, not only computed (the map-first layout hid them until 2026-10-07).
        r.check(w._hdr_estop_btn.isVisible() and w._hdr_estop_btn.isEnabled(), "E-STOP is in the header and works")
        w._show_vehicle_status_dialog()
        shown = [k for k in ("signing", "rtk", "gps_accuracy", "wind_failsafe", "arm_ready") if w._fields[k].isVisible()]
        r.check(len(shown) == 5, f"Vehicle status shows signing, RTK, GPS accuracy, wind failsafe and arm readiness on screen ({shown})")

        # Too early: no position yet. The drone refuses, and the window must
        # say why without claiming the link is lost.
        early = (sim.ekf_flags & 0x18) != 0x18
        if early:
            clicked = time.monotonic()
            w._on_takeoff()
            wait_qt(lambda: any("Auto takeoff failed" in line or "AUTO_TAKEOFF FAIL" in line for line in logs), 20)
            refused = [line for line in logs if "AUTO_TAKEOFF FAIL" in line]
            r.check(bool(refused), f"take-off before the position is ready is refused, in the drone's words: {refused[-1][:100] if refused else 'nothing'!r}")
            wait_qt(lambda: False, 2.0)
            false_alarm = [m for t, m in banners if t >= clicked and "Communication lost" in m]
            r.check(not false_alarm, f"and the banner never claimed the link was lost, not even for a moment ({false_alarm[:1] or banner(w)!r})")
            r.check(not sim.armed, "the drone did not arm")

        r.check(wait_qt(lambda: (sim.ekf_flags & 0x18) == 0x18, 120), "the drone has its position")
        r.check(wait_qt(lambda: "Ready" in w._fields["arm_ready"].text(), 30), f"the window says ready to arm ({w._fields['arm_ready'].text()!r})")

        # The settings read on connect, under this firmware's names.
        new_names = version.startswith("4.7")
        has = [n for n in (RENAMED_IN_4_7.values() if new_names else RENAMED_IN_4_7.keys())]
        r.check(wait_qt(lambda: all(n in w._last_params for n in has) and "FS_GCS_ENABLE" in w._last_params, 20),
                f"the settings VGCS checks were read on connect ({', '.join(has)}, FS_GCS_ENABLE ...)")
        listed = [w._param_name_combo.itemText(i) for i in range(w._param_name_combo.count())]
        wrong = [n for n in listed if n not in w._last_params]
        r.check(not wrong, f"the 'Set param' list has only this drone's names ({len(listed)} names)" + (f", not {wrong}" if wrong else ""))

        # Take-off through the window's own button handler.
        w._on_takeoff()
        r.check(wait_qt(lambda: sim.armed, 30), "Take-off: the drone armed")
        r.check(wait_qt(lambda: sim.alt > 14.0, 90), f"Take-off: it climbed to 15 m ({sim.alt:.1f} m)")
        r.check(wait_qt(lambda: w._fields["armed"].text() == "Yes", 5), f"the window shows armed ({w._fields['armed'].text()!r})")
        wait_qt(lambda: False, 3.0)
        shown = w._fields["alt_rel"].text()
        number = re.search(r"-?\d+(\.\d+)?", shown)
        r.check(number is not None and abs(float(number.group()) - sim.alt) < 1.5, f"the window shows the height ({shown!r}, the drone is at {sim.alt:.1f} m)")

        # A mode change from the window's mode list and button.
        observer.rc(rc3=1500)
        w._mode_combo.setCurrentText("LOITER")
        w._on_set_mode()
        r.check(wait_qt(lambda: sim.mode == "LOITER", 10), f"Set mode: the drone is in LOITER ({sim.mode})")
        r.check(wait_qt(lambda: w._top_flight_mode.text() == "LOITER", 5), f"the header shows LOITER ({w._top_flight_mode.text()!r})")
        r.check(wait_qt(lambda: any("Mode change confirmed" in line for line in logs), 5), "the log says the drone confirmed it")

        # The hover: measured.
        wait_qt(lambda: False, 3.0)
        memory_start = private_mb()
        cpu_start, wall_start, observer_cpu_start = time.process_time(), time.monotonic(), observer.cpu_s
        lag.recording = clock.recording = True
        traced = None
        if os.environ.get("VGCS_TEST_TRACEMALLOC"):
            import tracemalloc

            tracemalloc.start(12)
            traced = tracemalloc.take_snapshot()
        profiler = None
        if profile_to is not None:
            import cProfile

            profiler = cProfile.Profile()
            profiler.enable()
        memory_series = [(0.0, memory_start)]
        sampler = QTimer()
        sampler.timeout.connect(lambda: memory_series.append((time.monotonic() - wall_start, private_mb())))
        sampler.start(30_000)
        wait_qt(lambda: False, minutes * 60.0)
        sampler.stop()
        if traced is not None:
            import tracemalloc

            after_snap = tracemalloc.take_snapshot()
            grown = after_snap.compare_to(traced, "lineno")
            r.note("Python memory that grew most during the hover (tracemalloc):")
            for stat in grown[:15]:
                r.note(f"  {stat.size_diff / 1024:+.0f} kB in {stat.count_diff:+d} blocks: {stat.traceback[0]}")
            for stat in after_snap.compare_to(traced, "traceback")[:3]:
                r.note(f"  traceback of {stat.size_diff / 1024:+.0f} kB:")
                for frame in list(stat.traceback)[-8:]:
                    r.note(f"      {frame}")
            r.note(f"Python total traced now: {tracemalloc.get_traced_memory()[0] / 1048576:.1f} MB")
            tracemalloc.stop()
        if profiler is not None:
            profiler.disable()
            import io
            import pstats

            text = io.StringIO()
            stats = pstats.Stats(profiler, stream=text)
            stats.sort_stats("tottime").print_stats(30)
            stats.sort_stats("cumulative").print_stats(40)
            profile_to.write_text(text.getvalue(), encoding="utf-8")
            r.note(f"profile of the window's thread during the hover written to {profile_to}")
        lag.recording = clock.recording = False
        wall = time.monotonic() - wall_start
        cpu = (time.process_time() - cpu_start) - (observer.cpu_s - observer_cpu_start)
        memory_end = private_mb()
        r.check(sim.mode == "LOITER" and sim.alt > 13.0, f"the drone hovered the whole time ({sim.mode}, {sim.alt:.1f} m)")
        r.note(f"hover measured for {wall:.0f} s at simulator speed x{speedup}")
        r.note(f"window event loop, lateness of a 20 ms timer: {spread(lag.late_ms)}")
        total = sum(len(v) for v in clock.times.values())
        r.note(f"telemetry messages handled by the window: {total} ({total / wall:.0f} per second)")
        for kind in sorted(clock.times, key=lambda k: -sum(clock.times[k]))[:6]:
            r.note(f"  {kind}: {len(clock.times[kind])} calls, {spread(clock.times[kind])}")
        busy = sum(sum(v) for v in clock.times.values()) / 1000.0
        r.note(f"time in the telemetry handler: {busy:.2f} s of {wall:.0f} s ({100.0 * busy / wall:.1f} % of the window's thread)")
        r.note(f"CPU used by VGCS (all its threads, without the checking connection): {100.0 * cpu / wall:.0f} % of one core")
        r.note(f"memory (private bytes): {memory_start:.0f} MB at the start, {memory_end:.0f} MB after ({memory_end - memory_start:+.1f} MB)")
        r.note("memory every 30 s: " + ", ".join(f"{m:.0f}" for _t, m in memory_series) + " MB")
        half = [m for t, m in memory_series if t >= wall / 2.0]
        if len(half) >= 2:
            r.note(f"memory over the second half of the hover: {half[-1] - half[0]:+.1f} MB")
        worst = max(lag.late_ms) if lag.late_ms else 0.0
        r.check(worst < 250.0, f"the window never froze for a quarter of a second (worst {worst:.0f} ms late)")
        late = sum(1 for x in lag.late_ms if x > 50.0)
        r.check(late <= max(3, len(lag.late_ms) // 100), f"at most 1 in 100 timer ticks over 50 ms late ({late} of {len(lag.late_ms)})")
        r.check(memory_end - memory_start < 50.0 * minutes, f"no memory growth to speak of ({memory_end - memory_start:+.1f} MB in {minutes:g} min)")

        # The radio goes quiet, then comes back.
        wsl("pkill -STOP -x arducopter")
        r.check(wait_qt(lambda: w._watchdog.text().startswith("Lost"), 8), f"silence: the window says the link is lost ({w._watchdog.text()!r})")
        r.check("Communication lost" in banner(w), f"and the banner says so ({banner(w)!r})")
        r.check(w._hdr_banner_disconnect_btn.isVisible() and w._hdr_banner_disconnect_btn.isEnabled(),
                "and the header still offers Disconnect")
        wait_qt(lambda: False, 4.0)
        wsl("pkill -CONT -x arducopter")
        r.check(wait_qt(lambda: not w._watchdog.text().startswith("Lost"), 15), f"back: the window shows the link again ({w._watchdog.text()!r})")
        r.check(wait_qt(lambda: "Communication lost" not in banner(w), 10), f"and the banner clears ({banner(w)!r})")

        # Land through the window's own button handler.
        w._on_land()
        r.check(wait_qt(lambda: sim.mode == "LAND", 10), f"Land: the drone is landing ({sim.mode})")
        r.check(wait_qt(lambda: not sim.armed, 120 if speedup == 1 else 60), "Land: it landed and disarmed")
        r.check(wait_qt(lambda: w._fields["armed"].text() == "No", 5), f"the window shows disarmed ({w._fields['armed'].text()!r})")

        w._on_disconnect()
        r.check(wait_qt(lambda: w._thread is None or not w._thread.isRunning(), 10), "Disconnect: the link thread stopped")
        r.check(wait_qt(lambda: banner(w).startswith("Disconnected"), 5),
                f"and the banner says Disconnected, not that the link was lost ({banner(w)!r})")
        r.check(not popups, f"no unexpected popup came up {popups[:2]}")
        r.check(not internet, f"the map asked the internet for nothing, it drew from the offline tiles ({len(internet)} requests: {internet[:2]})")
    finally:
        lag.timer.stop()
        try:
            w.close()
        except Exception:
            pass
        observer.stop()
        sim.close()
        app.processEvents()


def write_report(path: pathlib.Path, version: str, minutes: float, speedup: int, r: Report) -> None:
    lines = ["# VGCS window test results", "",
             f"Made by `system_test/vgcs_window_test.py`, ArduCopter {version}, "
             f"{minutes:g} minute hover at simulator speed x{speedup}.", "",
             "## Checks", ""]
    lines += [f"- {'ok' if ok else '**FAIL**'}: {text}" for ok, text in r.lines]
    lines += ["", "## Measurements", ""]
    lines += [f"- {text.strip()}" for text in r.numbers]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")


def main(argv: list[str]) -> int:
    version = next((a for a in argv if re.fullmatch(r"\d+\.\d+\.\d+", a)), "4.7.0")
    minutes = float(argv[argv.index("--minutes") + 1]) if "--minutes" in argv else 3.0
    speedup = int(argv[argv.index("--speedup") + 1]) if "--speedup" in argv else 1
    r = Report()
    print(f"\n===== VGCS window, ArduCopter {version}, speed x{speedup}, {minutes:g} min hover")
    began = time.monotonic()
    try:
        profile_to = pathlib.Path(argv[argv.index("--profile") + 1]) if "--profile" in argv else None
        session(version, minutes, speedup, r, profile_to)
    except Exception as exc:
        import traceback

        traceback.print_exc()
        r.check(False, f"the session stopped: {type(exc).__name__}: {exc}")
    print(f"\n{len(r.lines) - r.failed} of {len(r.lines)} checks passed in {time.monotonic() - began:.0f} s")
    if "--report" in argv:
        target = pathlib.Path(argv[argv.index("--report") + 1])
        write_report(target, version, minutes, speedup, r)
        print(f"report written to {target}")
    import shutil

    shutil.rmtree(SETTINGS_DIR, ignore_errors=True)    # the test's settings and map tiles
    return 1 if r.failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
