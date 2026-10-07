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
silence, landing. Then a mission in Plan Flight: waypoints clicked on the map,
a row edited, Upload, Download. A second connection to the simulator checks
what really happened (it reads the mission off the drone itself), and the
window's own labels and fields are read to check what the operator would have
seen.

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


class MissionReader:
    """Reads the mission off the drone on the checking connection, item by item,
    as another ground station would. Nothing of VGCS is in the way."""

    def __init__(self, sim: Sitl) -> None:
        self.sim = sim
        self.count: int | None = None
        self.items: dict[int, object] = {}
        self.done = False
        take = sim._take

        def taking(msg) -> None:
            kind = msg.get_type()
            if kind == "MISSION_COUNT" and self.count is None:
                self.count = int(msg.count)
                self._next(0)
            elif kind == "MISSION_ITEM_INT" and self.count is not None and not self.done:
                self.items[int(msg.seq)] = msg
                self._next(int(msg.seq) + 1)
            take(msg)

        sim._take = taking

    def _next(self, seq: int) -> None:
        mav = self.sim.mav
        if self.count is not None and seq < self.count:
            mav.mav.mission_request_int_send(mav.target_system, mav.target_component, seq)
        else:
            mav.mav.mission_ack_send(mav.target_system, mav.target_component, 0)
            self.done = True

    def read(self) -> None:
        self.count, self.items, self.done = None, {}, False
        mav = self.sim.mav
        mav.mav.mission_request_list_send(mav.target_system, mav.target_component)

    def waypoints(self) -> list[tuple[float, float | None]]:
        """(height in m, speed in m/s) of each waypoint, as the drone will fly them."""
        speed = None
        out: list[tuple[float, float | None]] = []
        for seq in sorted(self.items):
            item = self.items[seq]
            if int(item.command) == 178:                    # DO_CHANGE_SPEED
                speed = round(float(item.param2), 1)
            elif int(item.command) == 16 and seq > 0:        # NAV_WAYPOINT (item 0 is home)
                out.append((round(float(item.z), 1), speed))
        return out

    def places(self) -> list[tuple[float, float]]:
        return [(item.x / 1e7, item.y / 1e7) for seq, item in sorted(self.items.items())
                if int(item.command) == 16 and seq > 0]


class MissionSender:
    """Sends a mission on the checking connection, as another ground station would.

    Each item: (frame, command, (p1, p2, p3, p4), lat, lon, height). The drone
    asks for them one by one. Its requests are read by the checking
    connection's own thread, like everything else that arrives there.
    """

    def __init__(self, sim: Sitl) -> None:
        self.sim = sim
        self.items: list[tuple] = []
        self.answer: int | None = None       # the drone's answer at the end: 0 is accepted
        self.active = False
        take = sim._take

        def taking(msg) -> None:
            kind = msg.get_type()
            if self.active and kind in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
                self._item(int(msg.seq))
            elif self.active and kind == "MISSION_ACK":
                self.answer = int(msg.type)
                self.active = False
            take(msg)

        sim._take = taking

    def _item(self, seq: int) -> None:
        if not 0 <= seq < len(self.items):
            return
        frame, command, p, lat, lon, height = self.items[seq]
        mav = self.sim.mav
        mav.mav.mission_item_int_send(mav.target_system, mav.target_component, seq, frame, command, 0, 1,
                                      p[0], p[1], p[2], p[3], int(lat * 1e7), int(lon * 1e7), height)

    def send(self, items: list[tuple]) -> None:
        self.items, self.answer, self.active = list(items), None, True
        mav = self.sim.mav
        mav.mav.mission_count_send(mav.target_system, mav.target_component, len(items))


def press_in_the_next_box(button_text: str, seen: list[dict]) -> QTimer:
    """Press a button of the next message box that comes up, as the operator would.

    What the box says and offers is noted in ``seen``. A box with no such
    button is closed (its Cancel), so a failed run never waits on one.
    """
    timer = QTimer()

    def look() -> None:
        box = next((x for x in QApplication.topLevelWidgets() if isinstance(x, QMessageBox) and x.isVisible()), None)
        if box is None:
            return
        timer.stop()
        buttons = {b.text(): b for b in box.buttons()}
        seen.append({"text": box.text(), "buttons": list(buttons)})
        button = buttons.get(button_text) or box.escapeButton()
        if button is not None:
            button.click()
        else:
            box.reject()

    timer.timeout.connect(look)
    timer.start(50)
    return timer


def the_mission_as_the_drone_holds_it(w, sim: Sitl, r: Report, popups: list[str], logs: list[str],
                                      reader: "MissionReader", speedup: int) -> None:
    """Download shows the mission the drone holds, and VGCS does not replace or start another one unasked.

    Until 2026-10-08 a Download lost the "Drop payload" marks, and the next
    Start Mission uploaded the plan without them: the payload was not released
    any more. A mission made in another ground station lost its camera, servo
    and jump commands the same way, without a word.
    """
    from PySide6.QtWidgets import QCheckBox, QLineEdit

    mw = w._map_widget
    panel = mw._plan_flight_panel
    before = len(popups)

    def result_line() -> str:
        """The line under the mission buttons, as far as it is on screen."""
        label = panel._mission_result_label
        return label.text() if label.isVisible() else ""

    def rows() -> list[tuple[float, float]]:
        shown: list[tuple[float, float]] = []
        while True:
            n = len(shown) + 1
            alt = panel.findChild(QLineEdit, f"planWpAlt{n}")
            spd = panel.findChild(QLineEdit, f"planWpSpeed{n}")
            if alt is None or spd is None:
                return shown
            shown.append((float(alt.text()), float(spd.text())))

    def drop_boxes() -> list[bool]:
        """The "Drop payload" box of each row, as the operator sees it."""
        out: list[bool] = []
        while True:
            box = panel.findChild(QCheckBox, f"planWpDrop{len(out) + 1}")
            if box is None:
                return out
            out.append(box.isChecked())

    def count(text: str) -> int:
        return sum(1 for line in logs if text in line)

    def on_the_drone() -> list[tuple[int, float, float]]:
        """(command, p1, p2) of every item the drone holds, read on the checking connection."""
        reader.read()
        wait_qt(lambda: reader.done, 20)
        return [(int(i.command), round(float(i.param1), 1), round(float(i.param2), 1)) for _seq, i in sorted(reader.items.items())]

    def download() -> list[str]:
        """Press Download. Returns the popups it brought up (the question is answered yes)."""
        seen = count("Mission download success")
        panel._btn_vdown.click()
        wait_qt(lambda: count("Mission download success") > seen, 30)
        wait_qt(lambda: False, 0.5)
        said = popups[before:]
        del popups[before:]
        return said

    # --- VGCS's own mission, with a payload release --------------------------------------
    panel.findChild(QCheckBox, "planWpDrop3").click()
    wait_qt(lambda: False, 0.6)          # the panel sends an edit on after a short wait
    seen = count("Mission upload success")
    panel._bar_upload.click()
    r.check(wait_qt(lambda: count("Mission upload success") > seen, 30), '"Drop payload" ticked on WP 3, and Upload: VGCS reports the mission uploaded')
    del popups[before:]
    held = on_the_drone()
    release = [(183, 9.0, 1900.0), (183, 9.0, 1100.0)]
    r.check([row for row in held if row[0] == 183] == release and [row[0] for row in held][-5:] == [16, 183, 112, 183, 20],
            f"the drone holds the release after WP 3: servo 9 to 1900, a wait, servo 9 back to 1100 ({[row[0] for row in held]})")

    panel.findChild(QCheckBox, "planWpDrop3").click()      # unticked on the map, and not uploaded
    wait_qt(lambda: False, 0.6)
    r.check(drop_boxes() == [False, False, False] and not mw._waypoints_model[2].drop_payload, "the box unticked on the map (not uploaded)")
    said = download()
    r.check(drop_boxes() == [False, False, True] and bool(mw._waypoints_model[2].drop_payload),
            f'Download: the "Drop payload" box of WP 3 is ticked again, as the drone holds it ({drop_boxes()})')
    r.check(len(said) == 1 and "from the drone" in said[0], f"it asked before it replaced the plan, and had nothing to warn about ({len(said)} popups)")
    r.check([row for row in on_the_drone() if row[0] == 183] == release, "and the drone still holds its release")

    # --- A mission made in another ground station -----------------------------------------
    home_lat, home_lon = 20.4347, 72.8696            # where the simulated drone stands (sitl_session default)

    def place(north_m: float, east_m: float) -> tuple[float, float]:
        return home_lat + north_m / 111_320.0, home_lon + east_m / (111_320.0 * math.cos(math.radians(home_lat)))

    a, b, c = place(400.0, 0.0), place(400.0, 400.0), place(400.0, 480.0)

    def theirs(second) -> list[tuple]:
        return [(3, 16, (0, 0, 0, 0), a[0], a[1], 0.0),             # the home slot
                (3, 22, (0, 0, 0, 0), 0.0, 0.0, 30.0),              # take-off to 30 m
                (3, 16, (3, 0, 0, 0), a[0], a[1], 30.0),            # a waypoint with a 3 s hover
                (3, 203, (0, 0, 0, 0), 0.0, 0.0, 0.0),              # a camera trigger
                (3, 183, (10, 1500, 0, 0), 0.0, 0.0, 0.0),          # a servo that is not the payload release
                (3, 16, (0, 0, 0, 0), second[0], second[1], 30.0),
                (3, 20, (0, 0, 0, 0), 0.0, 0.0, 0.0)]               # return to launch

    sender = MissionSender(sim)
    sender.send(theirs(b))
    r.check(wait_qt(lambda: sender.answer == 0, 20), f"another ground station's mission is on the drone (the drone answered {sender.answer})")
    r.check(wait_qt(lambda: "changed" in result_line() and "Download" in result_line(), 10),
            f"VGCS sees another number of items and says so under the mission buttons ({result_line()!r})")
    said = download()
    r.check(len(said) == 2 and "more than this plan can hold" in said[1] and "1 x camera trigger" in said[1] and "1 x servo command" in said[1],
            f"Download: VGCS says what the plan cannot hold ({said[1][:140] if len(said) > 1 else said!r})")
    own = rows()[0][1] if rows() else 0.0
    r.check(rows() == [(30.0, own), (30.0, own)] and own > 0.0,
            f"the rows show its two waypoints at 30 m, at the drone's own speed since it sets none ({rows()})")

    # Upload, answered no.
    questions: list[str] = []
    yes = QMessageBox.question
    QMessageBox.question = staticmethod(lambda _p, title, text, *a, **k: questions.append(f"{title}: {text}") or QMessageBox.StandardButton.No)
    seen = count("Mission upload success")
    try:
        panel._bar_upload.click()
        wait_qt(lambda: False, 2.0)
    finally:
        QMessageBox.question = yes
    r.check(len(questions) == 1 and "Upload anyway?" in questions[0] and "1 x camera trigger" in questions[0],
            f"Upload asks first, and names what would be gone ({questions[0][:120] if questions else 'no question'!r})")
    held = on_the_drone()
    r.check(count("Mission upload success") == seen and (203, 0.0, 0.0) in held and len(held) == 7,
            f"answered no: nothing is uploaded, the drone keeps its mission with the camera trigger ({[row[0] for row in held]})")
    del popups[before:]

    # Start Mission, left alone.
    boxes: list[dict] = []
    timer = press_in_the_next_box("Cancel", boxes)
    panel._start_mission_btn.click()
    timer.stop()
    wait_qt(lambda: False, 1.5)
    offered = sorted(boxes[0]["buttons"]) if boxes else []
    r.check(offered == ["Cancel", "Start it as it is on the drone", "Upload this plan and start"] and "1 x camera trigger" in boxes[0]["text"],
            f"Start Mission asks, and offers to start the mission as it is on the drone ({offered})")
    r.check(not sim.armed and popups[before:] == [] and count("Mission upload success") == seen, "Cancel: nothing is uploaded and the drone is not armed")

    # The other station changes its mission. The same number of items: the drone tells VGCS nothing.
    line_before = result_line()
    sender.send(theirs(c))
    r.check(wait_qt(lambda: sender.answer == 0, 20), "the other station sends its mission again, with the second waypoint 80 m further east")
    wait_qt(lambda: False, 3.0)
    r.check(result_line() == line_before, "VGCS is told nothing of it: the map still shows the mission as it was")
    boxes = []
    timer = press_in_the_next_box("Start it as it is on the drone", boxes)
    panel._start_mission_btn.click()
    timer.stop()
    wanted = "Not started: the mission on the drone changed since the Download"
    r.check(wait_qt(lambda: result_line().startswith(wanted), 20) and "Download" in result_line(),
            f"start as it is: VGCS reads the mission again, finds another one, starts nothing and says why ({result_line()!r})")
    wait_qt(lambda: False, 3.0)
    r.check(not sim.armed and sim.mode != "AUTO", f"the drone was not armed ({sim.mode})")
    del popups[before:]

    # After a new Download the same start flies it, with nothing uploaded.
    said = download()
    r.check(len(said) == 2 and "1 x camera trigger" in said[1], "a new Download shows the mission that is on the drone now")
    boxes = []
    timer = press_in_the_next_box("Start it as it is on the drone", boxes)
    panel._start_mission_btn.click()
    timer.stop()
    r.check(wait_qt(lambda: sim.armed and sim.mode == "AUTO", 60), f"start as it is: armed and in AUTO ({sim.mode})")
    r.check(wait_qt(lambda: sim.alt > 27.0, 90), f"it took off to the 30 m of that mission ({sim.alt:.1f} m)")
    r.check(count("Mission upload success") == seen and (203, 0.0, 0.0) in on_the_drone(),
            "nothing was uploaded: the drone flies its mission with the camera trigger in it")
    said = popups[before:]
    r.check(len(said) == 1 and "not armed" in said[0], f"the operator was told that it arms, and nothing else ({len(said)} popups)")
    del popups[before:]

    w._on_land()
    r.check(wait_qt(lambda: sim.mode == "LAND", 10), f"Land: the drone is landing ({sim.mode})")
    r.check(wait_qt(lambda: not sim.armed, 180 if speedup == 1 else 60), "Land: it landed and disarmed")
    del popups[before:]


def fly_the_plan(w, sim: Sitl, r: Report, popups: list[str], reader: "MissionReader",
                 want: list[tuple[float, float]], speedup: int) -> None:
    """Start Mission, then the mission controls in Plan Flight while it flies.

    Measured on the checking connection, 2026-10-07: after Pause and Resume the
    drone flew its own default speed (10 m/s) instead of the plan's, and a jump
    to another waypoint kept the speed of the leg before. "Fly to WP" itself
    could not be reached at all: it was in the menu of a table the window does
    not show.
    """
    from PySide6.QtWidgets import QLineEdit

    panel = w._map_widget._plan_flight_panel
    before = len(popups)
    item_of = [seq for seq, item in sorted(reader.items.items()) if int(item.command) == 16 and seq > 0]
    flying_to = {"item": None}
    take = sim._take

    def taking(msg) -> None:
        if msg.get_type() == "MISSION_CURRENT":
            flying_to["item"] = int(msg.seq)
        take(msg)

    sim._take = taking

    def speed() -> float:
        return math.hypot(sim.vn, sim.ve)

    def settles_at(want_mps: float, what: str) -> float:
        """Five seconds in a row within half a metre per second of it. Returns the fastest seen on the way."""
        seen: list[float] = []
        for _ in range(60):
            wait_qt(lambda: False, 1.0 / max(1, speedup))
            seen.append(speed())
            if len(seen) >= 5 and all(abs(v - want_mps) < 0.5 for v in seen[-5:]):
                break
        last = seen[-5:]
        r.check(len(last) == 5 and all(abs(v - want_mps) < 0.5 for v in last),
                f"{what}: it settles at {min(last):.1f} to {max(last):.1f} m/s, the plan says {want_mps:.1f} m/s (after {len(seen)} s)")
        return max(seen)

    def result_line() -> str:
        return panel._mission_result_label.text()

    # The hover earlier in this test left the throttle stick at half (an RC
    # override that stays). A drone is armed with the stick down.
    sticks = [65535] * 8
    sticks[2] = 1000
    for _ in range(3):
        sim.mav.mav.rc_channels_override_send(sim.mav.target_system, sim.mav.target_component, *sticks)
        time.sleep(0.05)
    wait_qt(lambda: False, 1.0)

    panel._start_mission_btn.click()
    r.check(wait_qt(lambda: sim.armed and sim.mode == "AUTO", 60), f"Start Mission: armed and in AUTO ({sim.mode})")
    r.check(wait_qt(lambda: sim.alt > want[0][0] - 2.5, 60), f"it took off to the height of WP 1 ({sim.alt:.1f} m)")
    said = popups[before:]
    r.check(len(said) == 2 and "not armed" in said[0] and "uploaded successfully" in said[1],
            f"Start Mission told the operator it arms, and that the mission is on the drone ({len(said)} popups)")
    del popups[before:]
    settles_at(want[0][1], "to WP 1")
    r.check(wait_qt(lambda: panel._mission_progress_label.text() == "Flying to WP 1 of 3", 5),
            f"Plan Flight says where it flies ({panel._mission_progress_label.text()!r})")

    # Fly to WP 3, through the button.
    r.check(panel._fly_to_row.isVisible() and panel._fly_to_btn.isEnabled(), '"Fly to WP" is on screen while the mission flies')
    panel._fly_to_combo.setCurrentIndex(panel._fly_to_combo.findData(2))
    panel._fly_to_btn.click()
    said = popups[before:]
    r.check(len(said) == 1 and "WP 3" in said[0] and f"{want[2][0]:.0f} m and {want[2][1]:.1f} m/s" in said[0],
            f"it asks first, with the waypoint's height and speed ({said})")
    del popups[before:]
    told = f"Flying to WP 3 at {want[2][1]:.1f} m/s"
    r.check(wait_qt(lambda: result_line() == told, 10), f"VGCS reports it under the button ({result_line()!r})")
    r.check(wait_qt(lambda: flying_to["item"] == item_of[2], 5),
            f"and the drone really is on its way to WP 3 (mission item {flying_to['item']}, WP 3 is item {item_of[2]})")
    settles_at(want[2][1], "to WP 3 after Fly to WP")
    r.check(panel._mission_progress_label.text() == "Flying to WP 3 of 3", f"Plan Flight follows ({panel._mission_progress_label.text()!r})")

    # Pause and Resume.
    panel._mission_pause_btn.click()
    r.check(wait_qt(lambda: sim.mode in ("BRAKE", "LOITER", "POSHOLD"), 10), f"Pause: the drone holds ({sim.mode})")
    wait_qt(lambda: speed() < 0.3, 15)
    panel._mission_resume_btn.click()
    r.check(wait_qt(lambda: sim.mode == "AUTO", 10), "Resume: back in AUTO")
    told = f"WP 3: planned {want[2][1]:.1f} m/s set again after the return to AUTO"
    r.check(wait_qt(lambda: result_line() == told, 10), f"VGCS says that it set the planned speed again ({result_line()!r})")
    fastest = settles_at(want[2][1], "to WP 3 after Pause and Resume")
    if speedup == 1:
        r.check(fastest < want[2][1] + 0.6, f"and it never flew faster than planned on the way ({fastest:.1f} m/s at most)")
    r.check(flying_to["item"] == item_of[2], f"still on its way to WP 3 (mission item {flying_to['item']})")

    # The plan is changed on the map and not uploaded: its numbers are no longer the drone's.
    field = panel.findChild(QLineEdit, "planWpAlt1")
    field.selectAll()
    from PySide6.QtCore import Qt
    from PySide6.QtTest import QTest

    QTest.keyClicks(field, "60")
    QTest.keyClick(field, Qt.Key.Key_Return)
    wait_qt(lambda: False, 0.5)
    panel._fly_to_combo.setCurrentIndex(panel._fly_to_combo.findData(0))
    panel._fly_to_btn.click()
    wait_qt(lambda: False, 1.5)
    said = popups[before:]
    r.check(len(said) == 1 and "not the mission the drone holds" in said[0],
            f"a plan changed on the map after the upload is not jumped in: VGCS says why ({len(said)} popups)")
    r.check(flying_to["item"] == item_of[2] and sim.mode == "AUTO", f"and the drone flies on to WP 3 (mission item {flying_to['item']})")
    del popups[before:]

    w._on_land()
    r.check(wait_qt(lambda: sim.mode == "LAND", 10), f"Land: the drone is landing ({sim.mode})")
    r.check(wait_qt(lambda: not sim.armed, 120 if speedup == 1 else 60), "Land: it landed and disarmed")
    r.check(wait_qt(lambda: not panel._fly_to_row.isVisible(), 5), '"Fly to WP" leaves the screen on the ground')
    r.check(popups[before:] == [], f"nothing else came up during the flight ({popups[before:]})")
    del popups[before:]


def plan_flight_mission(w, sim: Sitl, r: Report, popups: list[str], logs: list[str], speedup: int = 1) -> None:
    """Plan Flight as the operator uses it: waypoints clicked on the map, a row
    edited, Upload, Download, and then the plan flown.

    Until 2026-10-07 the rows showed numbers that were not the plan's: 164.0 m
    and 11.18 m/s on a waypoint stored at 20 m and 5 m/s, and the old rows on a
    plan that had been downloaded.
    """
    from PySide6.QtCore import QPoint, Qt
    from PySide6.QtTest import QTest
    from PySide6.QtWidgets import QLineEdit

    mw = w._map_widget
    panel = mw._plan_flight_panel
    nm = mw._native_map
    before = len(popups)

    def tool(name: str) -> None:
        next(b for b in panel._tool_buttons if b.text().split("\n")[-1].strip() == name).click()

    def type_in(field: QLineEdit, text: str) -> None:
        field.selectAll()
        QTest.keyClicks(field, text)
        QTest.keyClick(field, Qt.Key.Key_Return)

    def click_map(dx: int, dy: int) -> None:
        QTest.mouseClick(nm, Qt.MouseButton.LeftButton, Qt.KeyboardModifier.NoModifier,
                         nm.rect().center() + QPoint(dx, dy))
        wait_qt(lambda: False, 0.4)

    def rows() -> list[tuple[float, float]]:
        """The text in each row's height and speed field: what the operator reads."""
        shown: list[tuple[float, float]] = []
        while True:
            n = len(shown) + 1
            alt = panel.findChild(QLineEdit, f"planWpAlt{n}")
            spd = panel.findChild(QLineEdit, f"planWpSpeed{n}")
            if alt is None or spd is None:
                return shown
            shown.append((float(alt.text()), float(spd.text())))

    # Menu, Plan Flight (what the menu entry does), then the tools on the left.
    w._plan_flight_layer_wanted = True
    mw.set_plan_flight_visible(True)
    w._sync_plan_flight_chrome()
    wait_qt(lambda: False, 0.5)
    r.check(panel.isVisible(), "Plan Flight opens")
    tool("Center")
    tool("Waypoint")
    wait_qt(lambda: False, 0.3)
    click_map(40, -60)
    click_map(120, -20)
    r.check(len(mw._waypoints_model) == 2 and rows() == [(20.0, 5.0), (20.0, 5.0)],
            f"two clicks on the map make two waypoints, and their rows show 20 m and 5 m/s ({rows()})")

    # Another height and speed for waypoint 2, and for the waypoints still to come.
    type_in(panel.findChild(QLineEdit, "planWpAlt2"), "33")
    type_in(panel.findChild(QLineEdit, "planWpSpeed2"), "7")
    type_in(panel._initial_wp_alt, "26")
    # A slow third waypoint: after Resume a leg slower than about 6 m/s is
    # where the drone overshoots when the leg is not started again.
    type_in(panel._initial_wp_speed, "3")
    wait_qt(lambda: False, 0.5)          # the panel sends an edit on after a short wait
    click_map(60, 70)
    want = [(20.0, 5.0), (33.0, 7.0), (26.0, 3.0)]
    r.check(rows() == want, f"waypoint 2 edited in its row, and a third one added at the new initial values: the rows show {rows()}")

    r.check(panel._bar_upload.isVisible() and panel._bar_upload.isEnabled(), "the Upload button is on screen and can be pressed")
    panel._bar_upload.click()
    r.check(wait_qt(lambda: any("Mission upload success" in line for line in logs), 30), "Upload: VGCS reports the mission uploaded")
    said = popups[before:]
    r.check(len(said) == 1 and "uploaded successfully (3 waypoints)" in said[0], f"and tells the operator so, with no warning to answer ({said})")
    del popups[before:]

    reader = MissionReader(sim)
    reader.read()
    r.check(wait_qt(lambda: reader.done, 20), f"the checking connection read the mission off the drone ({len(reader.items)} of {reader.count} items)")
    r.check(reader.waypoints() == want and rows() == want,
            f"the drone holds the heights and speeds the rows show: {reader.waypoints()} (rows: {rows()})")
    planned = [(float(p.lat), float(p.lon)) for p in mw._waypoints_model]
    places = reader.places()
    r.check(len(places) == len(planned) and all(abs(a[0] - b[0]) < 1e-6 and abs(a[1] - b[1]) < 1e-6 for a, b in zip(places, planned)),
            "and the places clicked on the map")
    last = reader.items[max(reader.items)] if reader.items else None
    r.check(last is not None and int(last.command) == 20 and panel.mission_end_action() == "rtl",
            "the mission ends with return to launch, as the panel says")

    # The operator changes rows without uploading. One value is typed at the
    # very moment a heartbeat is handled: every heartbeat shows the panel again,
    # and a refresh of the rows there took such a value back.
    typed: list[float] = []
    real_sync = w._sync_plan_flight_chrome

    def type_then_sync() -> None:
        if not typed:
            typed.append(time.monotonic())
            type_in(panel.findChild(QLineEdit, "planWpAlt1"), "99")
        real_sync()

    w._sync_plan_flight_chrome = type_then_sync
    r.check(wait_qt(lambda: bool(typed), 5), "a heartbeat came while Plan Flight was open")
    del w._sync_plan_flight_chrome
    type_in(panel.findChild(QLineEdit, "planWpSpeed3"), "2")
    wait_qt(lambda: False, 1.5)          # more heartbeats pass
    changed = [(99.0, 5.0), (33.0, 7.0), (26.0, 2.0)]
    stored = [(round(float(p.alt_m), 1), round(float(p.speed_mps), 1)) for p in mw._waypoints_model]
    r.check(rows() == changed and stored == changed,
            f"a value typed as a heartbeat arrives stays, and so does the next one: the rows show {rows()} (the plan: {stored})")

    # Then the operator asks the drone what it holds. A downloaded plan of the
    # same size used to keep the old rows.
    tool("File")
    r.check(panel._btn_vdown.isVisible() and panel._btn_vdown.isEnabled(), "the Download button is on screen and can be pressed")
    seen = sum(1 for line in logs if "Mission download success" in line)
    panel._btn_vdown.click()          # it asks first, and the test answers yes
    said = popups[before:]
    r.check(len(said) == 1 and "3 waypoints" in said[0] and "from the drone" in said[0],
            f"Download asks before it replaces the plan on the map ({said})")
    del popups[before:]
    r.check(wait_qt(lambda: sum(1 for line in logs if "Mission download success" in line) > seen, 30), "Download: VGCS reports the mission downloaded")
    wait_qt(lambda: False, 0.5)
    stored = [(round(float(p.alt_m), 1), round(float(p.speed_mps), 1)) for p in mw._waypoints_model]
    r.check(rows() == want and stored == want, f"and the rows show the mission as the drone holds it: {rows()} (the plan: {stored})")
    r.check(popups[before:] == [], f"and nothing else came up ({popups[before:]})")
    del popups[before:]

    fly_the_plan(w, sim, r, popups, reader, want, speedup)
    the_mission_as_the_drone_holds_it(w, sim, r, popups, logs, reader, speedup)

    panel.exit_requested.emit()          # "Exit Plan"
    wait_qt(lambda: False, 0.3)
    r.check(not panel.isVisible(), "Exit Plan closes Plan Flight")


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

        # Plan Flight, on the ground: what the rows show is what the drone gets.
        plan_flight_mission(w, sim, r, popups, logs, speedup)

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
