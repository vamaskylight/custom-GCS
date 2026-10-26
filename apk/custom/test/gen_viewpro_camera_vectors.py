"""Replay action scripts through VGCS's Viewpro camera code and write what it sends,
as checks for ViewproCamera's C++ test.

The scripts run through the real vgcs/viewpro/adapter.py (ViewproGimbalTcpAdapter)
and vgcs/video/camera_control.py (ViewproCameraControl), with a fake clock, fake
timers and a fake socket: nothing reaches the network. The C++ test plays the same
scripts and must send the same frames at the same times.

Run from the repo root after changing either side:

    python apk/custom/test/gen_viewpro_camera_vectors.py

It writes apk/custom/test/viewpro_camera_vectors.inc.
"""

from __future__ import annotations

import os
import sys
import threading
import time
import types
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.pop("VGCS_VIEWPRO_POST_JOG_HOLD", None)

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))

from vgcs.video import camera_control as cc  # noqa: E402
from vgcs.viewpro import adapter as va  # noqa: E402
from vgcs.viewpro import protocol as vp  # noqa: E402

OUT = Path(__file__).with_name("viewpro_camera_vectors.inc")


class Clock:
    now_ms = 0


class FakeTimer:
    """threading.Timer that fires when the script reaches its time."""

    pending: list["FakeTimer"] = []

    def __init__(self, delay_s: float, fn) -> None:
        self.due_ms = Clock.now_ms + round(delay_s * 1000)
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

    def send(self, pkt: bytes) -> None:
        self.sent.append((Clock.now_ms, bytes(pkt)))

    def send_and_receive(self, pkt: bytes, **_kw) -> bytes:
        raise AssertionError("the scripts do not poll")

    def close(self) -> None:
        pass


# The adapter's own view of time and timers only (the real modules stay as they are).
va.time = types.SimpleNamespace(monotonic=lambda: Clock.now_ms / 1000.0, sleep=time.sleep)
va.threading = types.SimpleNamespace(Timer=FakeTimer, Lock=threading.Lock, Thread=threading.Thread)


def status_frame(*, servo_mode: int = 0x0D, record: int = 0, range_dm: int = 0, zoom_tenths: int = 10) -> bytes:
    """A status frame as the camera sends it over TCP (T1 F1 B1 D1 in an envelope)."""
    t1 = bytes(22)
    f1 = bytes([0])
    b1 = bytes([((servo_mode & 0x0F) << 4) | 0x07, 0xFF, 0x1B, 0xF0, 0xE3, 0x8C])
    d1 = bytes([0x00, (range_dm >> 16) & 0xFF, 0x00, record & 0x03, (range_dm >> 8) & 0xFF, range_dm & 0xFF,
                0x10, 0xD6, 0x1B, 0x6C, (zoom_tenths >> 8) & 0xFF, zoom_tenths & 0xFF])
    return vp.wrap_tcp(vp.build_serial_frame(vp.CMD_STATUS_T1F1B1D1, t1 + f1 + b1 + d1))


# (time ms, action, number a, number b, text)
SCRIPTS: dict[str, list[tuple]] = {
    "jog_then_hold": [
        (1000, "speed", 10.0, 0.0, ""), (1080, "speed", 0.0, 0.0, ""), (1300, "wait", 0, 0, ""),
        (1500, "wait", 0, 0, ""),
    ],
    "a_new_move_cancels_the_hold": [
        (1000, "speed", 0.0, 5.0, ""), (1100, "speed", 0.0, 0.0, ""), (1300, "speed", -5.0, 0.0, ""),
        (1400, "speed", 0.0, 0.0, ""), (1600, "center", 0, 0, ""), (2000, "wait", 0, 0, ""),
        (2100, "speed", 0.004, -0.004, ""), (2600, "wait", 0, 0, ""),
    ],
    "a_jog_before_the_hold_takes_it_back": [
        (1000, "speed", 0.0, 0.0, ""), (1100, "speed", 5.0, 0.0, ""), (1500, "wait", 0, 0, ""),
        (1600, "speed", 0.0, 0.0, ""), (1700, "look_down", 0, 0, ""), (2000, "wait", 0, 0, ""),
        (2100, "speed", 0.0, 0.0, ""), (2200, "angle", 10.0, -10.0, ""), (2600, "wait", 0, 0, ""),
        (2700, "ptz", 0, 0, "stop"), (2800, "ptz", 0, 0, "spin"), (3200, "wait", 0, 0, ""),
    ],
    "ptz_buttons": [
        (1000, "ptz", 0, 0, "up"), (1200, "ptz", 0, 0, "stop"), (1300, "ptz", 0, 0, "down"),
        (1400, "ptz", 0, 0, "left"), (1500, "ptz", 0, 0, "right"), (1600, "ptz", 0, 0, "stop"),
        (2000, "wait", 0, 0, ""), (2100, "ptz", 0, 0, "home"), (2200, "ptz", 0, 0, "spin"),
        (2300, "ptz", 0, 0, " UP "), (2400, "ptz", 0, 0, "pitch_down"), (2500, "ptz", 0, 0, "yaw_left"),
        (2600, "ptz", 0, 0, "center"),
    ],
    "zoom_clicks": [
        (1000, "zoom", 1, 0, ""), (1100, "wait", 0, 0, ""), (1150, "wait", 0, 0, ""), (1200, "zoom", -1, 0, ""),
        (1300, "zoom", 1, 0, ""), (1500, "wait", 0, 0, ""), (1600, "zoom", 0, 0, ""), (1700, "zoom", 5, 0, ""),
        (1760, "zoom", 0, 0, ""), (2000, "wait", 0, 0, ""),
    ],
    "focus_photo": [
        (1000, "focus", 1, 0, ""), (1100, "focus", -1, 0, ""), (1200, "focus", 0, 0, ""),
        (1300, "autofocus", 0, 0, ""), (1400, "photo", 0, 0, ""),
    ],
    "recording_follows_the_camera": [
        (1000, "record", 1, 0, ""), (1100, "record", 1, 0, ""), (1200, "status", 0, 0, status_frame(record=0).hex()),
        (1300, "record", 1, 0, ""), (1400, "status", 0, 0, status_frame(record=1).hex()), (1500, "record", 0, 0, ""),
        (1600, "record_toggle", 0, 0, ""), (1700, "record_toggle", 0, 0, ""), (1800, "record", 0, 0, ""),
    ],
    "sensor": [(1000, "sensor", 1, 0, ""), (1100, "sensor", 0, 0, ""), (1200, "sensor", 1, 0, "")],
    "the_cameras_tracker_keeps_the_gimbal": [
        (1000, "status", 0, 0, status_frame(servo_mode=0x06).hex()), (1100, "speed", 5.0, 0.0, ""),
        (1200, "speed", 0.0, 0.0, ""), (1700, "wait", 0, 0, ""),
        (1800, "status", 0, 0, status_frame(servo_mode=0x01).hex()), (1900, "speed", 0.0, 0.0, ""),
        (2300, "wait", 0, 0, ""),
    ],
    "the_tracker_takes_the_gimbal_before_the_hold": [
        (1000, "speed", 0.0, 0.0, ""), (1100, "status", 0, 0, status_frame(servo_mode=0x06).hex()),
        (1400, "wait", 0, 0, ""),
    ],
    "an_old_tracker_report_counts_no_more": [
        (1000, "status", 0, 0, status_frame(servo_mode=0x06).hex()), (4100, "speed", 0.0, 0.0, ""),
        (4500, "wait", 0, 0, ""),
    ],
    "angles": [
        (1000, "angle", 45.0, -30.0, ""), (1100, "angle", 190.0, 0.0, ""), (1200, "look_down", 0, 0, ""),
        (1300, "center", 0, 0, ""), (1400, "angle", -0.0027, 720.5, ""),
    ],
}
FINAL_MS = 10000


def run(script: list[tuple]) -> tuple[list[tuple[int, bytes]], bool, bool]:
    Clock.now_ms = 0
    FakeTimer.pending = []
    adapter = va.ViewproGimbalTcpAdapter(host="192.0.2.1")  # never connected: the transport is replaced
    adapter._transport = FakeTransport()
    control = object.__new__(cc.ViewproCameraControl)
    control._adapter = adapter

    def advance(to_ms: int) -> None:
        # What is due goes out at the step's time, stops before holds (the C++ tick's order).
        Clock.now_ms = to_ms
        while True:
            due = [t for t in FakeTimer.pending if not t.cancelled and t.due_ms <= to_ms]
            if not due:
                return
            due.sort(key=lambda t: (getattr(t.fn, "__name__", "") == "_apply_position_hold", t.due_ms))
            first = due[0]
            FakeTimer.pending.remove(first)
            first.fn()

    for t_ms, action, a, b, text in script:
        advance(t_ms)
        if action == "speed":
            control.set_gimbal_speed(a, b)
        elif action == "ptz":
            adapter.ptz(text)
        elif action == "center":
            adapter.center()
        elif action == "look_down":
            adapter.look_down()
        elif action == "angle":
            adapter.set_angle(a, b)
        elif action == "zoom":
            adapter.camera_zoom(int(a))
        elif action == "focus":
            adapter.camera_focus_step(int(a))
        elif action == "autofocus":
            adapter.camera_auto_focus()
        elif action == "photo":
            adapter.camera_photo()
        elif action == "record":
            control.camera_set_record(bool(a))
        elif action == "record_toggle":
            adapter.camera_record_toggle()
        elif action == "sensor":
            adapter.set_video_sensor("ir" if a else "eo")
        elif action == "status":
            adapter._update_from_reply(bytes.fromhex(text))
        elif action != "wait":
            raise ValueError(action)
    advance(FINAL_MS)
    return adapter._transport.sent, adapter.is_recording(), adapter.video_sensor() == "ir"


def cpp_str(text: str) -> str:
    return '"' + text.replace("\\", "\\\\").replace('"', '\\"') + '"'


lines: list[str] = [f"constexpr long long kFinalMs = {FINAL_MS};", ""]
for name, script in SCRIPTS.items():
    sent, recording, ir = run(script)
    lines.append(f"// {name}")
    lines.append(f"const Step kSteps_{name}[] = {{")
    for t_ms, action, a, b, text in script:
        lines.append(f"    {{{t_ms}, {cpp_str(action)}, {float(a)!r}, {float(b)!r}, {cpp_str(text)}}},")
    lines.append("};")
    lines.append(f"const Sent kSent_{name}[] = {{")
    for t_ms, pkt in sent:
        lines.append(f'    {{{t_ms}, "{pkt.hex().upper()}"}},')
    if not sent:
        lines.append('    {-1, ""},')
    lines.append("};")
    lines.append(f"CHECK_SCRIPT({name}, {'true' if recording else 'false'}, {'true' if ir else 'false'});")
    lines.append("")

OUT.write_text(
    "// Generated by gen_viewpro_camera_vectors.py from vgcs/viewpro/adapter.py and\n"
    "// vgcs/video/camera_control.py. Do not edit.\n"
    "// Each script ends with a tick at kFinalMs.\n\n" + "\n".join(lines),
    encoding="utf-8",
    newline="\n",
)
print(f"wrote {OUT.name}: {len(SCRIPTS)} scripts, {sum(len(run(s)[0]) for s in SCRIPTS.values())} frames")
