#!/usr/bin/env python3
"""Probe Skydroid C13 TOP UDP gimbal attitude (run from repo root on the field laptop)."""

from __future__ import annotations

import argparse
import socket
import sys
import time
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from vgcs.skydroid.command_map import SKYDROID_PROFILES
from vgcs.skydroid.protocol import build_top_frame, extract_attitude_deg, parse_top_frame
from vgcs.skydroid.targets import local_ipv4_for_target

_PORTS = (5000, 14550, 14551)


def _probe(host: str, port: int, profile_id: str, timeout_s: float) -> bool:
    profile = SKYDROID_PROFILES.get(profile_id, SKYDROID_PROFILES["c13_default"])
    lip = local_ipv4_for_target(host)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout_s)
    try:
        if lip:
            sock.bind((lip, 0))
            print(f"  bind local {lip} -> {host}:{port}")
        else:
            sock.bind(("", 0))
    except Exception as e:
        print(f"  bind failed: {e}")
        return False
    for cmd in profile.status_commands:
        frame = build_top_frame(cmd, {})
        try:
            sock.sendto(frame, (host, port))
            data, addr = sock.recvfrom(4096)
            dec = parse_top_frame(data)
            yaw, pitch = extract_attitude_deg(dec)
            print(f"  OK {addr[0]}:{addr[1]} cmd={cmd} reply={dec.command if dec else '?'} yaw={yaw} pitch={pitch}")
            if yaw is not None or pitch is not None:
                return True
        except socket.timeout:
            print(f"  -- timeout cmd={cmd}")
        except Exception as e:
            print(f"  -- error cmd={cmd}: {e}")
    return False


def main() -> int:
    ap = argparse.ArgumentParser(description="Skydroid C13 TOP UDP gimbal probe")
    ap.add_argument(
        "--hosts",
        default="192.168.43.1,192.168.144.108",
        help="Comma-separated IPs to try",
    )
    ap.add_argument("--profile", default="c13_default", choices=sorted(SKYDROID_PROFILES))
    ap.add_argument("--timeout", type=float, default=0.35)
    args = ap.parse_args()
    hosts = [h.strip() for h in str(args.hosts).split(",") if h.strip()]
    print(f"[probe] hosts={hosts} ports={_PORTS} profile={args.profile}")
    for host in hosts:
        print(f"\n=== {host} ===")
        for port in _PORTS:
            print(f"port {port}:")
            if _probe(host, int(port), str(args.profile), float(args.timeout)):
                print(f"\nSUCCESS: use Host={host} UDP port={port} in VGCS settings")
                return 0
            time.sleep(0.05)
    print("\nFAILED: no TOP attitude on any host/port. Video RTSP can still work.")
    print("Try: PC Ethernet to camera (get 192.168.144.x), or Skydroid app on same PC.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
