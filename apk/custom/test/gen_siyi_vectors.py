"""Generate C++ test vectors for SiyiProtocol from the VGCS Python protocol.

The APK must send exactly the bytes VGCS sends. This script runs the real
vgcs/siyi/protocol.py and writes its output as C++ checks, so the C++ test
compares against Python, not against values typed by hand.

Run from the repo root after changing either side:

    python apk/custom/test/gen_siyi_vectors.py

It writes apk/custom/test/siyi_protocol_vectors.inc.
"""

from __future__ import annotations

import math
import struct
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))

from vgcs.siyi import protocol as p  # noqa: E402

OUT = Path(__file__).with_name("siyi_protocol_vectors.inc")


def hexs(data: bytes | None) -> str:
    return "" if data is None else data.hex().upper()


def safe(fn):
    """The Python result, or None where it raises (struct.error, ValueError, OverflowError)."""
    try:
        return fn()
    except (struct.error, ValueError, OverflowError):
        return None


def num(v: float) -> str:
    if math.isnan(v):
        return "std::nan(\"\")"
    if math.isinf(v):
        return "HUGE_VAL" if v > 0 else "-HUGE_VAL"
    return repr(float(v))


lines: list[str] = []

# CRC16, with the start value of the C++ default and another.
CRC_INPUTS = [b"", b"\x00", b"\x01", b"\xff", bytes(range(256)), b"\x55\x66\x01\x00\x00\x00\x00\x0d", b"123456789"]
lines.append("// CHECK_U16(cpp_expression, expected)")
for data in CRC_INPUTS:
    for init in (0, 0x1234):
        lines.append(f'CHECK_U16(crc16(fromHex("{hexs(data)}"), {init:#06x}), {p.crc16(data, init):#06x});')

# Requests
lines.append("")
lines.append("// CHECK_HEX(cpp_expression giving Bytes, expected hex)")
REQUESTS = [
    (p.CMD_GIMBAL_ATTITUDE, b"", 0),
    (p.CMD_GIMBAL_ATTITUDE, b"", 1),
    (p.CMD_GIMBAL_ANGLE, p.encode_angle_deg(30.0, -45.0), 7),
    (p.CMD_PHOTO_RECORD, bytes([0]), 0xFFFF),
    (p.CMD_PHOTO_RECORD, bytes([2]), 0x1234),
    (p.CMD_MANUAL_ZOOM, p.encode_manual_zoom(-1), 300),
    (p.CMD_AUTO_FOCUS, p.encode_auto_focus(480, 270), 2),
    (0xFF, bytes(range(40)), 65535),
]
for cmd, data, seq in REQUESTS:
    lines.append(f'CHECK_HEX(buildRequest({cmd:#04x}, fromHex("{hexs(data)}"), {seq}), "{hexs(p.build_request(cmd, data, seq=seq))}");')

# Encoders, with the .5 rounding cases, the ends of each field and values that do not fit.
for y, pt in ((0.0, 0.0), (30.0, -45.0), (0.05, -0.05), (0.15, 0.25), (-0.15, -0.25), (179.9, -90.0),
              (3276.7, -3276.8), (3276.75, 0.0), (3276.8, 0.0), (0.0, -3276.85), (-3276.9, 0.0),
              (1e9, 0.0), (float("nan"), 0.0), (0.0, float("inf"))):
    lines.append(f'CHECK_HEX(encodeAngle({num(y)}, {num(pt)}), "{hexs(safe(lambda y=y, pt=pt: p.encode_angle_deg(y, pt)))}");')
for y, pt in ((0.0, 0.0), (5.0, -5.0), (0.05, -0.05), (0.15, 0.25), (9.95, -9.95), (10.0, -10.0), (10.04, -10.06),
              (55.5, -1e300), (1e308, 0.0), (float("nan"), 1.0), (1.0, float("-inf"))):
    lines.append(f'CHECK_HEX(encodeRotationSpeed({num(y)}, {num(pt)}), '
                 f'"{hexs(safe(lambda y=y, pt=pt: p.encode_rotation_speed(y, pt)))}");')
for d in (1, -1, 0, 5, -7):
    lines.append(f'CHECK_HEX(encodeManualZoom({d}), "{hexs(p.encode_manual_zoom(d))}");')
    lines.append(f'CHECK_HEX(encodeManualFocus({d}), "{hexs(p.encode_manual_focus(d))}");')
for x, y in ((0, 0), (480, 270), (1920, 1080), (65535, 65536), (-1, -2)):
    lines.append(f'CHECK_HEX(encodeAutoFocus({x}, {y}), "{hexs(p.encode_auto_focus(x, y))}");')
lines.append(f'CHECK_HEX(encodeAutoFocus(), "{hexs(p.encode_auto_focus())}");')


# Frames the camera sends: (bytes) -> parse result
def answer(cmd: int, data: bytes, seq: int = 3, ctrl: int = p.CTRL_IS_ACK) -> bytes:
    body = struct.pack("<HBHHB", p.STX, ctrl, len(data), seq, cmd) + data
    return body + struct.pack("<H", p.crc16(body))


att = struct.pack("<hhh", 1234, -456, 7) + bytes(6)
good = answer(p.CMD_GIMBAL_ATTITUDE, att)
FRAMES = [
    good,
    good + b"\x00\x01trailing",
    answer(p.CMD_GIMBAL_ANGLE, struct.pack("<hhh", -1800, 900, -5), seq=65535),
    answer(p.CMD_PHOTO_RECORD, b"", ctrl=p.CTRL_NEED_ACK),
    p.build_request(p.CMD_GIMBAL_ATTITUDE, b"", seq=9),
    good[:-1] + bytes([good[-1] ^ 0xFF]),  # a wrong CRC
    good[:9],                               # too short
    b"\x55\x67" + good[2:],                 # another start
    good[:-3],                              # the data is cut off
    b"",
]
lines.append("")
lines.append("// CHECK_PARSE(payload hex, has frame, ctrl, seq, cmd, data hex)")
for raw in FRAMES:
    f = p.parse_frame(raw)
    if f is None:
        lines.append(f'CHECK_PARSE("{hexs(raw)}", false, 0, 0, 0, "");')
    else:
        lines.append(f'CHECK_PARSE("{hexs(raw)}", true, {f.ctrl}, {f.seq}, {f.cmd_id}, "{hexs(f.data)}");')

# Attitude
lines.append("")
lines.append("// CHECK_ATTITUDE(data hex, has value, yaw, pitch, roll)")
for data in (att, struct.pack("<hhh", -32768, 32767, 0), struct.pack("<hhh", 1, -1, 5), att[:5], b""):
    yaw, pitch, roll = p.decode_attitude_deg(data)
    if yaw is None:
        lines.append(f'CHECK_ATTITUDE("{hexs(data)}", false, 0.0, 0.0, 0.0);')
    else:
        lines.append(f'CHECK_ATTITUDE("{hexs(data)}", true, {yaw!r}, {pitch!r}, {roll!r});')

OUT.write_text(
    "// Generated by gen_siyi_vectors.py from vgcs/siyi/protocol.py. Do not edit.\n\n" + "\n".join(lines) + "\n",
    encoding="utf-8",
    newline="\n",
)
print(f"wrote {OUT.name}: {sum(1 for x in lines if x.startswith('CHECK_'))} checks")
