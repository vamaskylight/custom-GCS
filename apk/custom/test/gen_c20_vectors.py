"""Generate C++ test vectors for C20Camera from the VGCS Python code.

The APK must send the bytes VGCS sends to the C20's camera port. This script runs the real
vgcs/skydroid/c20_camera.py and writes its output as C++ checks, so the C++ test compares
against Python, not against values typed by hand. (Both are also checked against the bytes of
Skydroid's own SDK, in c20_camera_test.cc and tests/test_skydroid_c20_camera.py.)

Run from the repo root after changing either side:

    python apk/custom/test/gen_c20_vectors.py

It writes apk/custom/test/c20_camera_vectors.inc.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))

from vgcs.skydroid import c20_camera as c  # noqa: E402

OUT = Path(__file__).with_name("c20_camera_vectors.inc")


def hexs(data: bytes) -> str:
    return data.hex().upper()


def reply(value: int, payload_len: int = 16, declared: int | None = None, group: int = c.GROUP_ZOOM,
          command: int = c.CMD_ZOOM_READ) -> bytes:
    payload = bytearray(payload_len)
    if payload_len > 9:
        payload[9] = value & 0xFF
    n = payload_len if declared is None else declared
    return bytes(6) + bytes([group, command, (n >> 8) & 0xFF, n & 0xFF]) + bytes(payload)


def capture_reply(payload_len: int = 32, declared: int | None = None) -> bytes:
    """A reply to a photo or record command (row B26): what it holds is not known, so 0x5A."""
    return reply(0, payload_len, declared, group=c.GROUP_CAMERA, command=c.CMD_CAPTURE)[:10] + b"\x5a" * payload_len


def capture_of(packet: bytes) -> str:
    done = c.capture_reply_ok(packet)
    return "-" if done is None else ("1" if done else "0")


def main() -> None:
    lines = [
        "// Written by gen_c20_vectors.py from vgcs/skydroid/c20_camera.py. Do not edit.",
        "",
        "// name, packet in hex",
        "static const struct { const char *name; const char *hex; } kC20Packets[] = {",
    ]
    for name, packet in (("zoomIn", c.build_zoom_in()), ("zoomOut", c.build_zoom_out()),
                         ("zoomStop", c.build_zoom_stop()), ("zoomQuery", c.build_zoom_query()),
                         ("stateQuery", c.build_state_query()),
                         ("photo", c.build_photo()), ("recordStart", c.build_record_start()),
                         ("recordStop", c.build_record_stop()),
                         ("torchOn", c.build_torch(True)), ("torchOff", c.build_torch(False))):
        lines.append(f'    {{"{name}", "{hexs(packet)}"}},')
    lines += ["};", ""]

    # What one read brings: the packets it is cut into, and the zoom each one says (0: none).
    reads: list[tuple[str, bytes]] = [
        ("one reply at 1x", reply(1)),
        ("one reply at 3, still 1x", reply(3)),
        ("one reply at 4, 2x", reply(4)),
        ("one reply at 66, 22x", reply(66)),
        ("67: no zoom", reply(67)),
        ("0: no zoom", reply(0)),
        ("200: no zoom (a negative byte to Java)", reply(200)),
        ("a longer payload", reply(31, payload_len=24)),
        ("two replies in one read", reply(10) + reply(40)),
        ("another packet, then a reply", c.build_packet(0x33, 0x55, bytes(5)) + reply(7)),
        ("a reply too short", reply(5, payload_len=12)),
        ("a reply that says 0 bytes: the rest of the read", reply(22, declared=0)),
        ("a length past the read: the rest of the read", reply(13, declared=40)),
        ("half a header left over", reply(16) + bytes(4)),
        ("another group", reply(9, group=0x33)),
        ("an empty read", b""),
        # Replies to a photo or record command (row B26): done with 32 bytes or more, as the SDK.
        ("a photo or record reply, 32 bytes: done", capture_reply(32)),
        ("a photo or record reply, 40 bytes: done", capture_reply(40)),
        ("a photo or record reply, 31 bytes: failed", capture_reply(31)),
        ("a photo or record reply, no payload: failed", capture_reply(0)),
        ("one that says 2 bytes with 32 after it: the rest of the read, done", capture_reply(32, declared=2)),
        ("a zoom reply, then a photo or record reply", reply(10) + capture_reply(32)),
        ("a photo or record reply, then a zoom reply", capture_reply(32) + reply(10)),
        ("a photo or record reply past the read: the rest of it, failed", capture_reply(20, declared=32)),
    ]
    lines += [
        "// a read, the packets it is cut into (hex, '|' between them), the zoom of each (',' between, 0 for",
        "// none), and what each says to a photo or record command (',' between: 1 done, 0 failed, - not a reply to one)",
        "static const struct { const char *name; const char *read; const char *packets; const char *zooms;",
        "                      const char *captures; } kC20Reads[] = {",
    ]
    for name, data in reads:
        packets = c.split_packets(data)
        zooms = ",".join(str(c.zoom_of_reply(p) or 0) for p in packets)
        captures = ",".join(capture_of(p) for p in packets)
        lines.append(f'    {{"{name}", "{hexs(data)}", "{"|".join(hexs(p) for p in packets)}", "{zooms}", "{captures}"}},')
    lines += ["};", ""]
    OUT.write_text("\n".join(lines), encoding="utf-8", newline="\n")
    print(f"wrote {OUT} ({len(reads)} reads)")


if __name__ == "__main__":
    main()
