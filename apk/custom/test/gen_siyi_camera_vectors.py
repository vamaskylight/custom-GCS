"""Replay action scripts through VGCS's SIYI camera code and write what it sends,
as checks for SiyiCamera's C++ test.

The scripts run through the real vgcs/siyi/adapter.py (SiyiGimbalUdpAdapter) and
vgcs/video/camera_control.py (SiyiCameraControl), with a fake clock, fake timers
and a fake socket: nothing reaches the network. The C++ test plays the same
scripts and must send the same frames at the same times. The zoom click is the
app's own (VGCS's zoom buttons send no SIYI zoom), so its checks are in the C++
test only.

Run from the repo root after changing either side:

    python apk/custom/test/gen_siyi_camera_vectors.py

It writes apk/custom/test/siyi_camera_vectors.inc.
"""

from __future__ import annotations

import itertools
import os
import struct
import sys
import threading
import time
import types
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))

from vgcs.siyi import adapter as sa  # noqa: E402
from vgcs.siyi import protocol as sp  # noqa: E402
from vgcs.video import camera_control as cc  # noqa: E402

OUT = Path(__file__).with_name("siyi_camera_vectors.inc")


class Clock:
    now_ms = 0


class FakeTimer:
    """threading.Timer that fires when the script reaches its time."""

    pending: list["FakeTimer"] = []
    counter = itertools.count()

    def __init__(self, delay_s: float, fn) -> None:
        self.due_ms = Clock.now_ms + round(delay_s * 1000)
        self.order = next(FakeTimer.counter)
        self.fn = fn
        self.cancelled = False
        self.daemon = False

    def start(self) -> None:
        FakeTimer.pending.append(self)

    def cancel(self) -> None:
        self.cancelled = True


class FakeTransport:
    def __init__(self) -> None:
        self.sent: list[tuple[int, bytes]] = []
        self.reply = b""

    def send_only(self, pkt: bytes) -> None:
        self.sent.append((Clock.now_ms, bytes(pkt)))

    def send_and_receive(self, pkt: bytes) -> bytes:
        self.sent.append((Clock.now_ms, bytes(pkt)))
        if not self.reply:
            raise TimeoutError("no answer")
        return self.reply

    def close(self) -> None:
        pass


# The adapter's own view of time and timers only (the real modules stay as they are).
sa.time = types.SimpleNamespace(monotonic=lambda: Clock.now_ms / 1000.0, sleep=time.sleep)
sa.threading = types.SimpleNamespace(Timer=FakeTimer, Lock=threading.Lock, Thread=threading.Thread)
# "Point down" reads its pitch from the VGCS settings: never read the real ones here.
cc.gimbal_nadir_pitch_deg = lambda settings=None: -90.0


def attitude(yaw: float, pitch: float, roll: float = 0.0, *, cmd: int = sp.CMD_GIMBAL_ATTITUDE) -> str:
    data = struct.pack("<hhh", round(yaw * 10), round(pitch * 10), round(roll * 10)) + bytes(6)
    body = struct.pack("<HBHHB", sp.STX, sp.CTRL_IS_ACK, len(data), 0, cmd) + data
    return (body + struct.pack("<H", sp.crc16(body))).hex()


# (time ms, action, number a, number b, text)
SCRIPTS: dict[str, list[tuple]] = {
    "gimbal_speed": [
        (1000, "speed", 5.0, -5.0, ""), (1100, "speed", 0.0, 0.0, ""), (1200, "speed", 12.34, -0.05, ""),
        (1300, "speed", 500.0, -500.0, ""), (1400, "speed", float("nan"), 0.0, ""),
    ],
    "ptz_buttons": [
        (1000, "ptz", 0, 0, "up"), (1100, "ptz", 0, 0, "down"), (1200, "ptz", 0, 0, "left"),
        (1300, "ptz", 0, 0, "right"), (1400, "ptz", 0, 0, "stop"), (1500, "ptz", 0, 0, "center"),
        (1600, "ptz", 0, 0, "home"), (1700, "ptz", 0, 0, "spin"), (1800, "ptz", 0, 0, " Pitch_Up "),
    ],
    "center_and_point_down": [
        (1000, "center", 0, 0, ""), (1100, "point_down", 0, 0, ""),
        (1200, "poll", 0, 0, attitude(37.5, -12.0)), (1300, "point_down", 0, 0, ""),
        (1400, "poll", 0, 0, ""), (1500, "point_down", 0, 0, ""),
    ],
    "angles": [
        (1000, "angle", 30.0, -45.0, ""), (1100, "angle", 3276.8, 0.0, ""), (1200, "angle", -179.95, 0.05, ""),
        (1300, "angle", 0.0, -3276.8, ""),
    ],
    "focus_steps": [
        (1000, "focus", 1, 0, ""), (1050, "wait", 0, 0, ""), (1070, "wait", 0, 0, ""), (1200, "wait", 0, 0, ""),
        (1240, "wait", 0, 0, ""), (1300, "focus", -1, 0, ""), (1350, "focus", 1, 0, ""), (1600, "wait", 0, 0, ""),
        (1700, "focus", 1, 0, ""), (1710, "focus", 0, 0, ""), (2000, "wait", 0, 0, ""),
    ],
    "photo_record_autofocus": [
        (1000, "photo", 0, 0, ""), (1100, "record_toggle", 0, 0, ""), (1200, "record_toggle", 0, 0, ""),
        (1300, "autofocus", 0, 0, ""),
    ],
    # VGCS reads one answer per poll and takes any frame in it as the attitude. The app
    # gets every datagram, so it takes the attitude from attitude frames only: answers
    # of another command are checked in the C++ test alone.
    "a_broken_answer_changes_nothing": [
        (1000, "poll", 0, 0, attitude(10.0, -20.0, 1.5)),
        (1100, "poll", 0, 0, attitude(-45.5, 30.0)[:-4] + "0000"),
        (1300, "point_down", 0, 0, ""),
    ],
}
FINAL_MS = 10000


def run(script: list[tuple]) -> tuple[list[tuple[int, bytes]], float | None]:
    Clock.now_ms = 0
    FakeTimer.pending = []
    adapter = sa.SiyiGimbalUdpAdapter(host="192.0.2.1")  # never used: the transport is replaced
    adapter._transport = FakeTransport()
    control = object.__new__(cc.SiyiCameraControl)
    control._adapter = adapter
    control._profile = cc.SiyiCameraProfile()

    def advance(to_ms: int) -> None:
        Clock.now_ms = to_ms
        while True:
            due = [t for t in FakeTimer.pending if not t.cancelled and t.due_ms <= to_ms]
            if not due:
                return
            first = min(due, key=lambda t: (t.due_ms, t.order))
            FakeTimer.pending.remove(first)
            first.fn()

    for t_ms, action, a, b, text in script:
        advance(t_ms)
        if action == "speed":
            control.set_gimbal_speed(a, b)
        elif action == "ptz":
            control.ptz(text)
        elif action == "center":
            control.gimbal_center()
        elif action == "point_down":
            control.gimbal_point_down()
        elif action == "angle":
            try:
                adapter.set_angle(a, b)
            except struct.error:
                pass
        elif action == "focus":
            adapter.camera_focus_step(int(a))
        elif action == "autofocus":
            adapter.camera_auto_focus()
        elif action == "photo":
            control.camera_trigger_photo()
        elif action == "record_toggle":
            control.camera_toggle_record()
        elif action == "poll":
            adapter._transport.reply = bytes.fromhex(text)
            adapter.request_attitude()
        elif action != "wait":
            raise ValueError(action)
    advance(FINAL_MS)
    yaw = adapter.get_status().yaw_deg
    return adapter._transport.sent, yaw


def cpp_str(text: str) -> str:
    return '"' + text.replace("\\", "\\\\").replace('"', '\\"') + '"'


def num(v: float) -> str:
    return 'std::nan("")' if v != v else repr(float(v))


lines: list[str] = [f"constexpr long long kFinalMs = {FINAL_MS};", ""]
total = 0
for name, script in SCRIPTS.items():
    sent, yaw = run(script)
    total += len(sent)
    lines.append(f"// {name}")
    lines.append(f"const Step kSteps_{name}[] = {{")
    for t_ms, action, a, b, text in script:
        lines.append(f"    {{{t_ms}, {cpp_str(action)}, {num(a)}, {num(b)}, {cpp_str(text)}}},")
    lines.append("};")
    lines.append(f"const Sent kSent_{name}[] = {{")
    for t_ms, pkt in sent:
        lines.append(f'    {{{t_ms}, "{pkt.hex().upper()}"}},')
    if not sent:
        lines.append('    {-1, ""},')
    lines.append("};")
    has_yaw = yaw is not None
    lines.append(f"CHECK_SCRIPT({name}, {'true' if has_yaw else 'false'}, {float(yaw or 0.0)!r});")
    lines.append("")

OUT.write_text(
    "// Generated by gen_siyi_camera_vectors.py from vgcs/siyi/adapter.py and\n"
    "// vgcs/video/camera_control.py. Do not edit.\n"
    "// Each script ends with a tick at kFinalMs.\n\n" + "\n".join(lines),
    encoding="utf-8",
    newline="\n",
)
print(f"wrote {OUT.name}: {len(SCRIPTS)} scripts, {total} frames")
