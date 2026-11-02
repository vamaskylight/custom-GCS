"""Run scripts through VGCS's position source and write what it says, as checks for the app's
C++ port (src/PositionSource.h, A8 step 4).

The scripts run through the real vgcs/app/position_source.py (pure, no Qt, no link) with
made-up times. The C++ test (test/link/position_source_test.cc) plays the same scripts and
must say the same after each step: the source, the "Position source" line, its colour, the
tag beside the satellites, the tooltip lines, and the note on a change in the air.

Run from the repo root after changing either side:

    python apk/custom/test/gen_position_source_vectors.py

It writes apk/custom/test/position_source_vectors.inc.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))

from vgcs.app.position_source import PositionSource  # noqa: E402

OUT = Path(__file__).with_name("position_source_vectors.inc")

CAMERA_SETTINGS = {"GPS2_TYPE": 14, "GPS_AUTO_SWITCH": 4}
NAN = float("nan")

# Each step: (time in s, what, ...). "gps1": fix. "gps2": fix, accuracy in m (None: not given),
# satellites. "armed": True or False. "settings": the names VGCS reads. "reset": a new link.
# "tick": nothing arrives (VGCS shows the source again once a second).
SCRIPTS: dict[str, list[tuple]] = {
    "no_gps2_shows_nothing": [
        (0.0, "settings", CAMERA_SETTINGS),
        (0.2, "gps1", 3),
        (0.5, "armed", True),
        (1.0, "gps1", 3),
        (2.0, "gps1", 0),
        (4.5, "tick"),
        (5.0, "gps1", 0),
        (8.0, "tick"),
    ],
    "the_camera_takes_over_in_the_air": [
        (0.0, "settings", CAMERA_SETTINGS),
        (0.1, "gps1", 3),
        (0.2, "gps2", 3, 1.5, 10),
        (1.0, "armed", True),
        (1.1, "gps1", 3),
        (3.2, "gps2", 3, 1.5, 10),
        (4.0, "gps1", 1),
        (5.0, "gps2", 3, 4.25, 0),
        (6.1, "gps2", 3, 4.25, 0),
        (7.0, "tick"),
        (8.0, "gps1", 3),
        (9.0, "tick"),
        (10.5, "gps1", 3),
        (11.0, "tick"),
    ],
    "a_flicker_is_not_news": [
        (0.0, "settings", CAMERA_SETTINGS),
        (0.0, "gps1", 3),
        (0.0, "gps2", 3, 1.2, 0),
        (0.5, "armed", True),
        (1.0, "gps1", 3),
        (3.0, "gps2", 3, 1.2, 0),
        (3.5, "tick"),
        (4.0, "gps1", 1),
        (4.5, "gps2", 3, 1.2, 0),
        (5.5, "gps1", 3),
        (6.0, "gps2", 3, 1.2, 0),
        (8.0, "tick"),
    ],
    "no_position_in_the_air": [
        (0.0, "settings", CAMERA_SETTINGS),
        (0.0, "gps1", 3),
        (0.0, "gps2", 3, 2.0, 0),
        (0.5, "armed", True),
        (1.0, "gps1", 3),
        (3.1, "tick"),
        (4.0, "gps1", 0),
        (4.1, "gps2", 0, None, 0),
        (5.0, "gps1", 2),
        (6.2, "tick"),
        (7.0, "gps2", 3, 1.0, 0),
        (8.0, "tick"),
        (9.5, "tick"),
    ],
    "nothing_on_the_ground": [
        (0.0, "settings", CAMERA_SETTINGS),
        (0.0, "gps1", 0),
        (0.1, "gps2", 0, None, 0),
        (1.0, "gps2", 3, None, 0),
        (2.0, "gps2", 2, None, 0),
        (7.5, "gps1", 0),
        (8.0, "tick"),
    ],
    "a_second_real_gps": [
        (0.0, "settings", {"GPS2_TYPE": 1, "GPS_AUTO_SWITCH": 1}),
        (0.0, "gps1", 3),
        (0.0, "gps2", 3, 0.8, 12),
        (0.5, "armed", True),
        (1.0, "gps1", 3),
        (3.5, "tick"),
        (4.0, "gps1", 2),
        (4.5, "gps2", 3, 0.8, 12),
        (6.5, "tick"),
    ],
    "the_old_setting_name": [
        (0.0, "gps2", 3, 1.0, 0),
        (0.0, "gps1", 3),
        (0.5, "settings", {"GPS_TYPE2": 14, "GPS_AUTO_SWITCH": 4}),
        (1.0, "gps1", 3),
        (1.5, "settings", {"GPS2_TYPE": 1, "GPS_TYPE2": 14}),
        (2.0, "gps2", 3, 1.0, 0),
        (2.5, "settings", {"GPS2_TYPE": 0}),
        (3.0, "gps1", 3),
        (3.5, "settings", {"GPS2_TYPE": 14.0, "OTHER": 3}),
        (4.0, "gps1", 3),
    ],
    "auto_switch_not_4": [
        (0.0, "settings", {"GPS2_TYPE": 14, "GPS_AUTO_SWITCH": 1}),
        (0.0, "gps2", 3, 1.0, 0),
        (0.5, "settings", {"GPS_AUTO_SWITCH": 2.9}),
        (1.0, "gps1", 3),
        (1.5, "settings", {"GPS_AUTO_SWITCH": 4}),
        (2.0, "gps1", 3),
        (2.5, "settings", {"GPS2_TYPE": 5, "GPS_AUTO_SWITCH": 0}),
        (3.0, "gps1", 3),
    ],
    "readings_expire": [
        (0.0, "settings", CAMERA_SETTINGS),
        (0.0, "gps1", 3),
        (0.0, "gps2", 3, 1.0, 0),
        (5.0, "tick"),
        (5.001, "tick"),
        (6.0, "gps2", 3, 2.0, 0),
        (7.0, "gps1", 0),
        (11.5, "tick"),
        (12.5, "tick"),
        # In the air GPS 2's words in GPS 1's line show its lifetime too.
        (20.0, "armed", True),
        (20.0, "gps1", 3),
        (20.0, "gps2", 3, 1.0, 0),
        (23.0, "gps1", 3),
        (25.0, "tick"),
        (25.001, "tick"),
    ],
    "accuracy_words": [
        (0.0, "settings", CAMERA_SETTINGS),
        (0.0, "armed", True),
        (0.0, "gps1", 0),
        (0.1, "gps2", 3, None, 0),
        (0.2, "gps2", 3, 0.0, 0),
        (0.3, "gps2", 3, -1.0, 0),
        (0.4, "gps2", 3, NAN, 0),
        (0.5, "gps2", 3, 0.04, 0),
        (0.6, "gps2", 3, 0.05, 0),
        (0.7, "gps2", 3, 4.25, 0),
        (0.8, "gps2", 3, 12.35, 0),
        (0.9, "gps2", 3, 4294967.295, 0),
        (1.0, "gps1", 3),
        (1.1, "gps2", 2, 3.0, 0),
        (1.2, "gps2", 6, 3.0, 0),
        (7.0, "gps1", 3),
    ],
    "disarming_forgets_the_wait": [
        (0.0, "settings", CAMERA_SETTINGS),
        (0.0, "gps1", 3),
        (0.0, "gps2", 3, 1.0, 0),
        (0.5, "armed", True),
        (1.0, "gps1", 3),
        (3.5, "tick"),
        (4.0, "gps1", 1),
        (4.2, "gps2", 3, 1.0, 0),
        (5.0, "armed", False),
        (5.5, "gps2", 3, 1.0, 0),
        (6.0, "armed", True),
        (6.5, "gps2", 3, 1.0, 0),
        (7.5, "gps2", 3, 1.0, 0),
        (8.6, "tick"),
    ],
    "a_silence_keeps_the_wait": [
        (0.0, "settings", CAMERA_SETTINGS),
        (0.0, "gps1", 3),
        (0.0, "gps2", 3, 1.0, 0),
        (0.2, "armed", True),
        (1.0, "gps1", 3),
        (3.5, "tick"),
        (4.0, "gps1", 1),
        (4.0, "gps2", 3, 1.0, 0),
        (9.5, "tick"),
        (10.0, "gps2", 3, 1.0, 0),
    ],
    "a_new_link": [
        (0.0, "settings", CAMERA_SETTINGS),
        (0.0, "gps1", 3),
        (0.0, "gps2", 3, 1.0, 0),
        (0.5, "armed", True),
        (1.0, "gps1", 1),
        (3.5, "gps2", 3, 1.0, 0),
        (10.0, "reset"),
        (11.0, "gps1", 3),
        (11.5, "settings", CAMERA_SETTINGS),
        (12.0, "gps2", 3, 1.0, 0),
        (12.5, "armed", True),
        (13.0, "gps1", 3),
        (15.5, "tick"),
    ],
    "gps2_alone": [
        (0.0, "settings", CAMERA_SETTINGS),
        (0.0, "gps2", 3, 1.0, 0),
        (0.5, "armed", True),
        (1.0, "gps2", 3, 1.0, 0),
        (3.1, "tick"),
        (4.0, "gps2", 3, 1.0, 0),
    ],
}

PUBLISHED = {"gps1", "gps2", "tick"}  # the steps after which VGCS also takes the note


def c_string(text: str | None) -> str:
    if text is None:
        return "nullptr"
    assert text.isascii(), text
    return json.dumps(text)


def c_number(value: float) -> str:
    if math.isnan(value):
        return "kNaN"
    return repr(float(value))


def setting(params: dict, name: str) -> str:
    return c_number(float(params[name])) if name in params else "kAbsent"


def run(name: str, steps: list[tuple]) -> list[str]:
    source = PositionSource()
    rows = []
    for step in steps:
        t, what = step[0], step[1]
        fix, acc, sats, params = 0, 0.0, 0, {}
        if what == "gps1":
            fix = step[2]
            source.update_gps1(fix, now=t)
        elif what == "gps2":
            fix, acc_m, sats = step[2], step[3], step[4]
            acc = 0.0 if acc_m is None else acc_m
            source.update_gps2(fix, h_acc_m=acc_m, satellites=sats, now=t)
        elif what == "armed":
            fix = 1 if step[2] else 0
            source.armed = bool(step[2])
        elif what == "settings":
            params = step[2]
            source.update_settings(params)
        elif what == "reset":
            source.reset()
        else:
            assert what == "tick", what
        expected = (source.source(t), source.field_text(t), source.level(t), source.header_tag(t), source.tooltip(t))
        event = source.take_event(t) if what in PUBLISHED else None
        text, warning = event if event is not None else (None, False)
        rows.append(
            f"    {{{c_number(t)}, {c_string(what)}, {fix}, {c_number(acc)}, {sats}, "
            f"{setting(params, 'GPS2_TYPE')}, {setting(params, 'GPS_TYPE2')}, {setting(params, 'GPS_AUTO_SWITCH')},\n"
            f"     {c_string(expected[0])}, {c_string(expected[1])}, {c_string(expected[2])}, "
            f"{c_string(expected[3])},\n     {c_string(expected[4])},\n"
            f"     {c_string(text)}, {'true' if warning else 'false'}}},"
        )
    return rows


def main() -> None:
    out = [
        "// Generated by gen_position_source_vectors.py from vgcs/app/position_source.py. Do not edit.",
        "// Each step: the time (s), what arrives and its values, then what VGCS's PositionSource says",
        "// after it: the source, the line, its colour, the tag, the tooltip, and the note (nullptr: none).",
        "",
    ]
    for name, steps in SCRIPTS.items():
        out.append(f"// {name}")
        out.append(f"const Step kSteps_{name}[] = {{")
        out.extend(run(name, steps))
        out.append("};")
        out.append(f"CHECK_SCRIPT({name});")
        out.append("")
    OUT.write_text("\n".join(out), encoding="utf-8", newline="\n")
    print(f"wrote {OUT} ({len(SCRIPTS)} scripts)")


if __name__ == "__main__":
    main()
