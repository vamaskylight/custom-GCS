from __future__ import annotations

import argparse
import random
import socket
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from vgcs.skydroid.protocol import (
    encode_attitude_field_4char,
    parse_top_frame,
    tp_checksum,
)

PROFILES: dict[str, set[str]] = {
    "c13_default": {"GAA", "GAC", "PTZ", "GSY", "GSP", "GSM", "GAY", "GAP", "GAM"},
    "c13_alt": {"GAC", "GAA", "PTZ", "GSP", "GSY", "GSM", "GAP", "GAY", "GAM"},
}


def _gac_reply(yaw: float, pitch: float, roll: float = 0.0) -> bytes:
    data = (
        encode_attitude_field_4char(yaw)
        + encode_attitude_field_4char(pitch)
        + encode_attitude_field_4char(roll)
    )
    body = f"#TPUGCrGAC{data}"
    return f"{body}{tp_checksum(body)}".encode("ascii")


def main() -> int:
    ap = argparse.ArgumentParser(description="Skydroid TOP-UDP simulator (#TP protocol)")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--profile", choices=sorted(PROFILES.keys()), default="c13_default")
    args = ap.parse_args()
    supported = PROFILES[str(args.profile)]

    yaw = 12.5
    pitch = -3.25
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.host, int(args.port)))
    print(f"[sim] #TP UDP on {args.host}:{args.port} profile={args.profile}")
    while True:
        raw, addr = sock.recvfrom(4096)
        dec = parse_top_frame(raw)
        if dec is None:
            continue
        tag = dec.command.upper()
        if tag not in supported and tag != "GAC":
            continue
        if tag in ("GAA", "GAC") or tag in {"GSY", "GSP", "GSM", "GAY", "GAP", "GAM", "PTZ"}:
            yaw += random.uniform(-0.4, 0.4)
            pitch += random.uniform(-0.25, 0.25)
            sock.sendto(_gac_reply(yaw, pitch), addr)


if __name__ == "__main__":
    raise SystemExit(main())
