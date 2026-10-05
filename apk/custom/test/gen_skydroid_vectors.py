"""Generate C++ test vectors for SkydroidTop from the VGCS Python protocol.

The APK must send exactly the bytes VGCS sends. This script runs the real
vgcs/skydroid/protocol.py builders and writes their output as C++ checks, so
the C++ test compares against Python, not against values typed by hand.

Run from the repo root after changing either side:

    python apk/custom/test/gen_skydroid_vectors.py

It writes apk/custom/test/skydroid_top_vectors.inc.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))

from vgcs.skydroid import protocol as p  # noqa: E402

OUT = Path(__file__).with_name("skydroid_top_vectors.inc")


def cpp_str(value: str) -> str:
    bs = chr(92)
    escaped = value.replace(bs, bs + bs).replace('"', bs + '"')
    escaped = escaped.replace(chr(13), bs + 'r').replace(chr(10), bs + 'n').replace(chr(9), bs + 't')
    return '"' + escaped + '"'


def frame(value) -> str:
    """Python builder result as text ('' for None or an error)."""
    if value is None:
        return ""
    return value.decode("ascii")


def safe(fn):
    try:
        return fn()
    except ValueError:
        return None


# (C++ expression, Python callable). Both must describe the same call.
CASES: list[tuple[str, object]] = []


def case(cpp: str, fn) -> None:
    CASES.append((cpp, fn))


# Checksum and frame builder
for body in ("#TPUG2wGAA01", "#tpUG4wGSM0A0A", "", "#TPUD2rDZM00"):
    case(f"checksum({cpp_str(body)})", lambda b=body: p.tp_checksum(b))
case('buildTpFrame(\'G\', \'w\', "GAA", "01")', lambda: frame(p.build_tp_frame(dest="G", control="w", tag="GAA", data="01")))
case('buildTpFrame(\'G\', \'w\', "GA", "01")', lambda: frame(safe(lambda: p.build_tp_frame(dest="G", control="w", tag="GA", data="01"))))
case('buildTpFrame(\'D\', \'w\', "REC", "1")', lambda: frame(safe(lambda: p.build_tp_frame(dest="D", control="w", tag="REC", data="1"))))
case('buildTpFrame(\'G\', \'w\', "GSM", "0123456789ABCDEF")',
     lambda: frame(safe(lambda: p.build_tp_frame(dest="G", control="w", tag="GSM", data="0123456789ABCDEF"))))

# Speed and angle encoders, including .5 rounding cases
for v in (0.0, 0.25, 0.5, 0.75, 1.25, -0.25, -0.75, 10.0, -10.0, 63.5, 64.0, 100.0, -100.0, 1.0):
    case(f"encodeSpeed2({v!r})", lambda v=v: p.encode_speed_2char(v))
for v in (0.0, 0.005, 0.015, 0.025, -0.005, -0.015, 12.345, -12.345, 90.0, -90.0, 327.67, -327.68, 400.0, -400.0):
    case(f"encodeAttitudeField4({v!r})", lambda v=v: p.encode_attitude_field_4char(v))

# Gimbal
for hz in (0, 1, 5, 50, 100, 200):
    case(f"buildGaaEnable({hz})", lambda hz=hz: frame(p.build_gaa_enable(hz)))
case("buildGacQuery()", lambda: frame(p.build_gac_query()))
for a in ("stop", "up", "down", "left", "right", "center", "nadir", "down_once", "point_down", "UP", " left ", "spin"):
    case(f"buildPtz({cpp_str(a)})", lambda a=a: frame(p.build_ptz(a)))
for y, pt in ((0.0, 0.0), (10.0, -10.0), (30.0, 5.5), (-63.5, 63.5), (200.0, -200.0), (0.25, -0.25)):
    case(f"buildGimbalSpeed({y!r}, {pt!r})", lambda y=y, pt=pt: frame(p.build_gimbal_speed(y, pt)))
for tag, v in (("GSY", 10.0), ("GSP", -12.5), ("GSR", 3.0)):
    case(f"buildGimbalSpeedAxis({cpp_str(tag)}, {v!r})",
         lambda tag=tag, v=v: frame(p.build_top_frame(tag, {"yaw": v, "pitch": v, "roll": v, "speed": v})))
for tag, d, s in (("GAY", 45.0, 16.0), ("GAP", -90.0, 16.0), ("GAR", 0.0, 8.0), ("GAY", -170.25, 30.0), ("GAX", 1.0, 1.0)):
    case(f"buildGimbalAngleAxis({cpp_str(tag)}, {d!r}, {s!r})",
         lambda tag=tag, d=d, s=s: frame(safe(lambda: p.build_gimbal_angle_axis(tag, d, s))))
case("buildGimbalAngles(30.0, -45.0, 25.0, 20.0)",
     lambda: frame(p.build_top_frame("GAM", {"yaw": 30.0, "pitch": -45.0, "yaw_speed": 25.0, "pitch_speed": 20.0})))

# Camera and system
case("buildPhoto()", lambda: frame(p.build_top_frame("CAM_PHOTO")))
case("buildRecord()", lambda: frame(p.build_top_frame("CAM_REC")))
case('buildSystemCommand("rec", "01", false)', lambda: frame(p.build_system_command("rec", "01", write=False)))
case('buildSystemCommand("IMG", "0102")', lambda: frame(p.build_system_command("IMG", "0102")))

# Laser
case("buildSlrQuery()", lambda: frame(p.build_slr_query()))
case("buildSlrQuery('G')", lambda: frame(p.build_slr_query(dest="G")))
case("buildSlrTrigger()", lambda: frame(p.build_slr_trigger()))

# Tracking
for x, y in ((640, 360), (0, 0), (1280, 720), (-5, 900), (1500, -1)):
    case(f"buildGotTarget({x}, {y})", lambda x=x, y=y: frame(p.build_got_target(x, y)))
case("buildSumTrack(true)", lambda: frame(p.build_sum_track(confirm=True)))
case("buildSumTrack(false)", lambda: frame(p.build_sum_track(confirm=False)))

# Zoom and focus
for z in (1.0, 2.5, 14.0, 30.0, 0.05, 0.15, 35.0):
    case(f"buildDzmAbsoluteZoom({z!r})", lambda z=z: frame(p.build_dzm_absolute_zoom(z)))
    case(f"buildDzmAbsoluteZoomUd({z!r})", lambda z=z: frame(p.build_dzm_absolute_zoom_ud(z)))
    case(f"buildMulOpticalZoom({z!r})", lambda z=z: frame(p.build_mul_optical_zoom(z)))
case("buildDzmAbsoluteZoom(5.0, 3)", lambda: frame(p.build_dzm_absolute_zoom(5.0, camera_x0=3)))
for a in ("in", "out", "tele", "wide", "+", "-", "x"):
    case(f"buildDzmZoomStepV47({cpp_str(a)})", lambda a=a: frame(safe(lambda: p.build_dzm_zoom_step_v47(a))))
case("buildDzmQuery()", lambda: frame(p.build_dzm_query()))
for lens in ("long", "tele", "short", "wide", "mid"):
    case(f"buildDzmLensSelect({cpp_str(lens)})", lambda lens=lens: frame(safe(lambda: p.build_dzm_lens_select(lens))))
for n in (0, 1, 2, 4, 5):
    case(f"buildDzmPreset({n})", lambda n=n: frame(safe(lambda: p.build_dzm_preset(n))))
for a in ("stop", "in", "out", "tele", "wide", "x"):
    case(f"buildDzmStepZoom({cpp_str(a)})", lambda a=a: frame(safe(lambda: p.build_dzm_step_zoom(a))))
    case(f"buildZmcZoom({cpp_str(a)})", lambda a=a: frame(safe(lambda: p.build_zmc_zoom(a))))
for a in ("stop", "near", "far", "in", "out", "auto", "x"):
    case(f"buildFccFocus({cpp_str(a)})", lambda a=a: frame(safe(lambda: p.build_fcc_focus(a))))

# Lists of frames, joined with "|" for comparison
LIST_CASES: list[tuple[str, object]] = []
for z in (1.0, 7.5, 40.0):
    LIST_CASES.append((f"buildOpticalZoomFrames({z!r})", lambda z=z: [frame(f) for f in p.build_optical_zoom_frames(z)]))
for d in (1, -1, 0):
    LIST_CASES.append((f"buildDzmStepFrames({d})", lambda d=d: [frame(f) for f in p.build_dzm_step_frames(d)]))
    LIST_CASES.append((f"buildC13ZoomStepFrames({d})", lambda d=d: [frame(f) for f in p.build_c13_zoom_step_frames(d)]))
LIST_CASES.append(("buildDzmHomeFrames()", lambda: [frame(f) for f in p.build_dzm_home_frames()]))

# Decoders: (C++ expression giving std::optional<number>, Python result or None)
DECODE_CASES: list[tuple[str, object]] = []
for f in ("0000", "1194", "EE6C", "FFFF", "7FFF", "8000", " 0064 ", "00G1", "123", "ee6c"):
    DECODE_CASES.append((f"decodeAttitudeField4({cpp_str(f)})", lambda f=f: p.decode_attitude_field_4char(f)))
p.set_slr_max_range_m(None)
for f in ("0032", "0031", "2710", "2711", "01F4", "FFFF", "01f4", "0x01F4", "12"):
    DECODE_CASES.append((f"decodeSlrDecimeters({cpp_str(f)})", lambda f=f: p.decode_slr_decimeters(f)))
for f in ("00", "46", "8B", "FF", "1", "123", "0x46"):
    DECODE_CASES.append((f"decodeDzmStep({cpp_str(f)})", lambda f=f: p.decode_dzm_step(f)))

SLR_MAX_CASES = [None, 500.0, 1000.0, 1200.0, 1500.4, 99999.0]

# Replies: (raw frame, expected fields)
REPLIES = [
    "#TPGU4rGAC11940000000022",
    "#tpGUCrGAC1194EE6C000A00",
    "#TPDU4rSLR01F4F4",
    "#TPDU4rSLRFFFF00",
    "#TPDU2rDZM4600",
    "#TPDU2wDZM0A00",
    "  #tpUG4wGSM0A0A1C\r\n",
    "#TPXX2rGAC00",
    "#TPUG2xGAA0100",
    "hello",
    "#TPUG2wGAA01",
]


def main() -> int:
    lines = ["// Generated by gen_skydroid_vectors.py from vgcs/skydroid/protocol.py. Do not edit.", ""]
    lines.append("// CHECK_FRAME(cpp_expression, expected_text)")
    for cpp, fn in CASES:
        lines.append(f"CHECK_FRAME({cpp}, {cpp_str(fn())});")
    lines.append("")
    lines.append("// CHECK_FRAMES(cpp_expression, expected_frames_joined_by_bar)")
    for cpp, fn in LIST_CASES:
        lines.append(f"CHECK_FRAMES({cpp}, {cpp_str('|'.join(fn()))});")
    lines.append("")
    lines.append("// CHECK_DECODE(cpp_expression, has_value, value)")
    for cpp, fn in DECODE_CASES:
        v = fn()
        lines.append(f"CHECK_DECODE({cpp}, {'true' if v is not None else 'false'}, {float(v) if v is not None else 0.0!r});")
    lines.append("")
    lines.append("// CHECK_SLR_MAX(metres_or_nan, expected_dm)")
    for m in SLR_MAX_CASES:
        expected = int(round(p.set_slr_max_range_m(m) * 10))
        arg = "std::nullopt" if m is None else repr(m)
        lines.append(f"CHECK_SLR_MAX({arg}, {expected});")
    p.set_slr_max_range_m(None)
    lines.append("")
    lines.append("// CHECK_REPLY(raw, parsed, tag, ctrl, has_yaw, yaw, has_pitch, pitch, has_roll, roll, has_slr, slr_dm, has_dzm, dzm_step)")
    for raw in REPLIES:
        dec = p.parse_tp_frame(raw.encode("ascii"))
        if dec is None:
            lines.append(f"CHECK_REPLY({cpp_str(raw)}, false, \"\", 'r', false, 0.0, false, 0.0, false, 0.0, false, 0, false, 0);")
            continue
        q = dec.params

        def opt(key, conv=float):
            return ("true", repr(conv(q[key]))) if key in q else ("false", repr(conv(0)))
        hy, yv = opt("yaw")
        hp, pv = opt("pitch")
        hr, rv = opt("roll")
        hs, sv = opt("slr_dm", int)
        hd, dv = opt("dzm_step", int)
        lines.append(
            f"CHECK_REPLY({cpp_str(raw)}, true, {cpp_str(dec.command)}, '{q['ctrl']}', "
            f"{hy}, {yv}, {hp}, {pv}, {hr}, {rv}, {hs}, {sv}, {hd}, {dv});"
        )
    OUT.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {OUT.relative_to(REPO)}: {len(CASES)} frames, {len(LIST_CASES)} lists, "
          f"{len(DECODE_CASES)} decodes, {len(SLR_MAX_CASES)} laser limits, {len(REPLIES)} replies")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
