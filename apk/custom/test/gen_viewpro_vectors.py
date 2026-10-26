"""Generate C++ test vectors for ViewproProtocol from the VGCS Python protocol.

The APK must send exactly the bytes VGCS sends. This script runs the real
vgcs/viewpro/protocol.py and writes its output as C++ checks, so the C++ test
compares against Python, not against values typed by hand.

Run from the repo root after changing either side:

    python apk/custom/test/gen_viewpro_vectors.py

It writes apk/custom/test/viewpro_protocol_vectors.inc.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))

from vgcs.viewpro import protocol as p  # noqa: E402

OUT = Path(__file__).with_name("viewpro_protocol_vectors.inc")


def hexs(data: bytes | None) -> str:
    return "" if data is None else data.hex().upper()


lines: list[str] = ["// CHECK_HEX(cpp_expression giving Bytes, expected hex)"]

# Serial frames and the TCP envelope
for cmd, data in ((p.CMD_HEARTBEAT, b"\x00"), (0x1C, b"\x03"), (p.CMD_A1_C1_E1, bytes(14)), (0x01, b""),
                  (0x7F, bytes(range(60))), (0xFF, bytes(range(70)))):
    lines.append(f'CHECK_HEX(buildSerialFrame({cmd:#04x}, fromHex("{hexs(data)}")), "{hexs(p.build_serial_frame(cmd, data))}");')
for inner in (b"", b"\x55\xaa\xdc\x05\x1c\x03\x83\x99", bytes(range(255)), bytes([0xFF] * 300)):
    lines.append(f'CHECK_HEX(wrapTcp(fromHex("{hexs(inner)}")), "{hexs(p.wrap_tcp(inner))}");')

# A1, C1 and the combined command
for args in ((p.SERVO_MANUAL_SPEED, 1000, -2000, 0, 0), (p.SERVO_MANUAL_ABSOLUTE_ANGLE, 0, 16384, 0, -8192),
             (p.SERVO_NO_CHANGE, 0, 0, 0, 0), (0x1FF, 40000, -40000, 32767, -32768)):
    lines.append(f'CHECK_HEX(encodeA1({", ".join(str(a) for a in args)}), "{hexs(p.encode_a1(*args))}");')
for op, zs, sensor, lrf in ((p.C1_OP_FOV_MINUS_ZOOM_IN, 3, 0, 0),
                            (p.C1_OP_TAKE_PICTURE, 0, 0, 0), (p.C1_OP_START_RECORD, 0, 0, 0),
                            (p.C1_OP_STOP_RECORD, 0, 0, 0), (0, 0, p.C1_SENSOR_IR, 0), (0, 0, 0, p.C1_LRF_SINGLE),
                            (0x7F, 7, 7, 7), (0xFF, 9, 9, 9), (-1, -1, -1, -1)):
    lines.append(f'CHECK_HEX(encodeC1({op}, {zs}, {sensor}, {lrf}), '
                 f'"{hexs(p.encode_c1(op=op, zoom_speed=zs, sensor=sensor, lrf=lrf))}");')

COMMANDS = [
    dict(),
    dict(servo=p.SERVO_MANUAL_SPEED, servo_p1=p.speed_dps_to_raw(12.5), servo_p2=p.speed_dps_to_raw(-3.25)),
    dict(servo=p.SERVO_MANUAL_ABSOLUTE_ANGLE, servo_p2=p.angle_deg_to_raw(45.0), servo_p4=p.angle_deg_to_raw(-30.0)),
    dict(servo=p.SERVO_HOME_POSITION),
    dict(servo=p.SERVO_LOOK_DOWN),
    dict(c1_op=p.C1_OP_FOV_MINUS_ZOOM_IN, c1_zoom_speed=4),
    dict(c1_op=p.C1_OP_STOP),
    dict(c1_op=p.C1_OP_TAKE_PICTURE),
    dict(c1_op=p.C1_OP_START_RECORD),
    dict(c1_sensor=p.C1_SENSOR_IR),
    dict(c1_lrf=p.C1_LRF_SINGLE),
]
CPP_FIELD = {"servo": "servo", "servo_p1": "servoP1", "servo_p2": "servoP2", "servo_p3": "servoP3",
             "servo_p4": "servoP4", "c1_op": "c1Op", "c1_zoom_speed": "c1ZoomSpeed", "c1_sensor": "c1Sensor",
             "c1_lrf": "c1Lrf"}
for kw in COMMANDS:
    sets = "".join(f" c.{CPP_FIELD[k]} = {v};" for k, v in kw.items())
    lines.append(f'CHECK_COMMAND({{{sets} }}, "{hexs(p.encode_gimbal_camera_command(**kw))}");')
lines.append(f'CHECK_HEX(encodeHeartbeat(), "{hexs(p.encode_heartbeat())}");')

# Angle and speed fields, with the wrap at 180 and the .5 rounding cases
lines.append("")
lines.append("// CHECK_INT(cpp_expression, expected)")
for deg in (0.0, 1.0, -1.0, 45.0, 90.0, -90.0, 179.99, 180.0, -180.0, 181.0, -181.0, 359.0, 720.5, -540.25,
            0.0027465820312500, 0.00274658203125 / 2, 1e-12, -1e-12, 33.75):
    lines.append(f"CHECK_INT(angleDegToRaw({deg!r}), {p.angle_deg_to_raw(deg)});")
for dps in (0.0, 0.005, 0.015, 0.025, -0.005, 12.5, -3.25, 100.0, 327.67, 1000.0):
    lines.append(f"CHECK_INT(speedDpsToRaw({dps!r}), {p.speed_dps_to_raw(dps)});")
lines.append("")
lines.append("// CHECK_REAL(cpp_expression, expected)")
for raw in (0, 1, -1, 16384, -16384, 32767, -32768, 12345):
    lines.append(f"CHECK_REAL(angleRawToDeg({raw}), {p.angle_raw_to_deg(raw)!r});")

# Parsing what the camera sends
lines.append("")
lines.append("// CHECK_PARSE_SERIAL(payload hex, has frame, cmd, data hex)")
good = p.build_serial_frame(0x1C, b"\x03")
for raw in (good, good + b"\x00\x11", bytes.fromhex("55AADC051C038399"), good[:-1] + bytes([good[-1] ^ 1]),
            good[:6], b"\x55\xaa\xdd" + good[3:], bytes.fromhex("55AADC02100012"), bytes.fromhex("55AADC03100013"),
            p.build_serial_frame(p.CMD_STATUS_T1F1B1D1, bytes(41)),
            p.build_serial_frame(0x01, b""),  # a whole frame of 6 bytes, which VGCS does not take
            b""):
    parsed = p.parse_serial_frame(raw)
    if parsed is None:
        lines.append(f'CHECK_PARSE_SERIAL("{hexs(raw)}", false, 0, "");')
    else:
        lines.append(f'CHECK_PARSE_SERIAL("{hexs(raw)}", true, {parsed[0]}, "{hexs(parsed[1])}");')
lines.append("")
lines.append("// CHECK_UNWRAP(payload hex, has inner, inner hex)")
wrapped = p.wrap_tcp(good)
for raw in (wrapped, wrapped + b"\xeb", wrapped[:-1] + bytes([wrapped[-1] ^ 1]), wrapped[:-1], b"\xeb\x91" + wrapped[2:],
            p.wrap_tcp(b""), b"\xeb\x90\x00", b""):
    inner = p.unwrap_tcp(raw)
    lines.append(f'CHECK_UNWRAP("{hexs(raw)}", {"true" if inner is not None else "false"}, "{hexs(inner)}");')

# Status: the doc's worked B1 example, a whole status frame, frames hidden in noise
lines.append("")
lines.append("// CHECK_B1(data hex, has, yaw, pitch, roll, servo status)")
for data in (bytes.fromhex("D7FF1BF0E38C"), bytes.fromhex("00000000"), bytes.fromhex("FFFF7FFF8000"),
             bytes.fromhex("0800C0004000")):
    d = p.decode_b1(data)
    if d is None:
        lines.append(f'CHECK_B1("{hexs(data)}", false, 0.0, 0.0, 0.0, 0);')
    else:
        lines.append(f'CHECK_B1("{hexs(data)}", true, {d[0]!r}, {d[1]!r}, {d[2]!r}, {d[3]});')
lines.append("")
lines.append("// CHECK_D1(data hex, has, record status, has range, range, vfov, hfov, zoom)")
for data in (bytes(12), bytes.fromhex("00010002" "0102" "0BB8" "1388" "00FA"), bytes.fromhex("FFFFFFFFFFFFFFFFFFFFFFFF"),
             bytes(11)):
    d = p.decode_d1(data)
    if d is None:
        lines.append(f'CHECK_D1("{hexs(data)}", false, 0, false, 0.0, 0.0, 0.0, 0.0);')
    else:
        has_range = d["range_m"] is not None
        lines.append(f'CHECK_D1("{hexs(data)}", true, {d["record_status"]}, {"true" if has_range else "false"}, '
                     f'{(d["range_m"] or 0.0)!r}, {d["vfov_deg"]!r}, {d["hfov_deg"]!r}, {d["zoom_x"]!r});')

t1 = bytes(range(1, 23))
f1 = bytes([0b01010010])
b1 = bytes.fromhex("D7FF1BF0E38C")
d1 = bytes.fromhex("00010002" "0102" "0BB8" "1388" "00FA")
status_full = p.wrap_tcp(p.build_serial_frame(p.CMD_STATUS_T1F1B1D1, t1 + f1 + b1 + d1))
status_short = p.build_serial_frame(p.CMD_STATUS_T1F1B1D1, t1 + f1 + b1 + d1[:5])
lines.append("")
lines.append("// CHECK_STATUS(payload hex, has, yaw, pitch, roll, servo, track source, track status, track type, has camera,")
lines.append("//              record, has range, range, vfov, hfov, zoom)")
for raw in (status_full, b"\x00\x55\xaa" + status_full + b"\x01", status_short,
            p.build_serial_frame(p.CMD_HEARTBEAT, b"\x00") + status_full,
            # Another frame long enough to decode as a status comes first: it is not one.
            p.build_serial_frame(0x41, bytes(range(41))) + status_full,
            p.build_serial_frame(0x41, bytes(range(41))),
            p.build_serial_frame(p.CMD_STATUS_T1F1B1D1, t1 + f1 + b1[:5]), b"\x55\xaa\xdc", b""):
    s = p.find_status_frame(raw)
    if s is None:
        lines.append(f'CHECK_STATUS("{hexs(raw)}", false, 0.0, 0.0, 0.0, 0, 0, 0, 0, false, 0, false, 0.0, 0.0, 0.0, 0.0);')
        continue
    cam = "range_m" in s or "record_status" in s
    has_range = cam and s.get("range_m") is not None
    lines.append(
        f'CHECK_STATUS("{hexs(raw)}", true, {s["yaw_deg"]!r}, {s["pitch_deg"]!r}, {s["roll_deg"]!r}, {s["servo_status"]}, '
        f'{s["track_source"]}, {s["track_status"]}, {s["track_target_type"]}, {"true" if cam else "false"}, '
        f'{s.get("record_status", 0)}, {"true" if has_range else "false"}, {(s.get("range_m") or 0.0)!r}, '
        f'{s.get("vfov_deg", 0.0)!r}, {s.get("hfov_deg", 0.0)!r}, {s.get("zoom_x", 0.0)!r});'
    )

OUT.write_text(
    "// Generated by gen_viewpro_vectors.py from vgcs/viewpro/protocol.py. Do not edit.\n\n" + "\n".join(lines) + "\n",
    encoding="utf-8",
    newline="\n",
)
print(f"wrote {OUT.name}: {sum(1 for x in lines if x.startswith('CHECK_'))} checks")
