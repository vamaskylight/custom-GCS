from __future__ import annotations

import argparse
import random
import socket
import time

PROFILES: dict[str, dict[str, object]] = {
    "c13_default": {
        "speed_cmds": {"GSY", "GSP", "GSM"},
        "angle_cmds": {"GAY", "GAP", "GAM"},
        "status_cmds": {"GAA", "GAC"},
        "status_reply": "GAA",
        "ptz_cmds": {"PT_UP", "PT_DOWN", "PT_LEFT", "PT_RIGHT", "PT_CENTER", "PT_STOP"},
        "camera_cmds": {"CAM_REC", "CAM_RECORD", "CAM_SNAP", "CAM_PHOTO", "CAM_ZOOM", "CAM_Z"},
    },
    "c13_alt": {
        "speed_cmds": {"GSP", "GSY", "GSM"},
        "angle_cmds": {"GAP", "GAY", "GAM"},
        "status_cmds": {"GAC", "GAA"},
        "status_reply": "GAC",
        "ptz_cmds": {"PTZ_UP", "PTZ_DOWN", "PTZ_LEFT", "PTZ_RIGHT", "PTZ_CENTER", "PTZ_STOP"},
        "camera_cmds": {"CAM_RECORD", "CAM_REC", "CAM_PHOTO", "CAM_SNAP", "CAM_Z", "CAM_ZOOM"},
    },
}


def _checksum(text: str) -> int:
    out = 0
    for ch in text.encode("ascii", errors="ignore"):
        out ^= int(ch)
    return out & 0xFF


def _frame(command: str, **params: object) -> bytes:
    body = ",".join(["TOP", command] + [f"{k}={v}" for k, v in params.items()])
    return f"${body}*{_checksum(body):02X}\r\n".encode("ascii", errors="ignore")


def _parse(raw: bytes) -> tuple[str, dict[str, str]] | None:
    text = raw.decode("ascii", errors="ignore").strip()
    if not text:
        return None
    if text.startswith("$"):
        text = text[1:]
    if "*" in text:
        text = text.split("*", 1)[0]
    parts = [p.strip() for p in text.split(",") if p.strip()]
    if len(parts) < 2:
        return None
    cmd = parts[1].upper()
    params: dict[str, str] = {}
    for part in parts[2:]:
        if "=" not in part:
            continue
        k, v = part.split("=", 1)
        params[k.strip().lower()] = v.strip()
    return cmd, params


def main() -> int:
    ap = argparse.ArgumentParser(description="Skydroid TOP-UDP simulator (M4)")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument(
        "--profile",
        choices=sorted(PROFILES.keys()),
        default="c13_default",
        help="Simulate firmware command profile behavior",
    )
    ap.add_argument(
        "--strict-profile",
        action="store_true",
        help="Only ACK commands from selected profile (unknown commands return ok=0)",
    )
    args = ap.parse_args()
    profile = PROFILES[str(args.profile)]

    yaw = 0.0
    pitch = 0.0
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.host, int(args.port)))
    print(f"[sim] listening on udp://{args.host}:{args.port} profile={args.profile} strict={bool(args.strict_profile)}")
    while True:
        raw, addr = sock.recvfrom(4096)
        parsed = _parse(raw)
        if parsed is None:
            continue
        cmd, params = parsed
        status_cmds = set(profile["status_cmds"])
        speed_cmds = set(profile["speed_cmds"])
        angle_cmds = set(profile["angle_cmds"])
        ptz_cmds = set(profile["ptz_cmds"])
        camera_cmds = set(profile["camera_cmds"])
        all_supported = status_cmds | speed_cmds | angle_cmds | ptz_cmds | camera_cmds
        if bool(args.strict_profile) and cmd not in all_supported:
            sock.sendto(_frame("ACK", cmd=cmd, ok=0, reason="unsupported_in_profile"), addr)
            continue
        if cmd in status_cmds:
            yaw += random.uniform(-0.4, 0.4)
            pitch += random.uniform(-0.25, 0.25)
            sock.sendto(
                _frame(str(profile["status_reply"]), yaw=f"{yaw:.2f}", pitch=f"{pitch:.2f}", ts=int(time.time())),
                addr,
            )
            continue
        if cmd in (speed_cmds | angle_cmds):
            try:
                yaw = float(params.get("yaw", yaw))
                pitch = float(params.get("pitch", pitch))
            except Exception:
                pass
        sock.sendto(_frame("ACK", cmd=cmd, ok=1), addr)


if __name__ == "__main__":
    raise SystemExit(main())

