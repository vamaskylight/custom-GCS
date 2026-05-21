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


def _print_network_context(hosts: list[str]) -> None:
    lip = local_ipv4_for_target(hosts[0]) if hosts else None
    print(f"[network] PC route IP toward camera LAN: {lip or '(unknown)'}")
    if lip and lip.startswith("192.168.43."):
        print(
            "[network] You are on the RC hotspot (192.168.43.x). RTSP is often bridged; "
            "TOP UDP gimbal replies usually require PC on 192.168.144.x (Ethernet to camera)."
        )


def _listen_udp(port: int, seconds: float) -> None:
    """Wait for unsolicited TOP/UDP from the camera (some firmware pushes attitude)."""
    print(f"\n[listen] UDP port {port} for {seconds:.1f}s (unsolicited datagrams)...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("", int(port)))
        sock.settimeout(0.5)
        deadline = time.monotonic() + max(0.5, float(seconds))
        got = 0
        while time.monotonic() < deadline:
            try:
                data, addr = sock.recvfrom(4096)
                got += 1
                text = data.decode("ascii", errors="replace").strip()[:120]
                dec = parse_top_frame(data)
                yaw, pitch = extract_attitude_deg(dec)
                print(f"  RX from {addr[0]}:{addr[1]} ({len(data)} B) {text!r} yaw={yaw} pitch={pitch}")
            except socket.timeout:
                continue
        if got == 0:
            print("  (no unsolicited UDP received)")
    except Exception as e:
        print(f"  listen failed: {e}")
    finally:
        sock.close()


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
    ap.add_argument(
        "--listen",
        type=float,
        default=2.0,
        metavar="SEC",
        help="After polls, listen on port 5000 for unsolicited UDP (0=skip)",
    )
    ap.add_argument(
        "--try-alt-profile",
        action="store_true",
        help="Also try c13_alt command names if default profile fails",
    )
    args = ap.parse_args()
    hosts = [h.strip() for h in str(args.hosts).split(",") if h.strip()]
    profiles = [str(args.profile)]
    if bool(args.try_alt_profile) and "c13_alt" not in profiles:
        profiles.append("c13_alt")
    print(f"[probe] hosts={hosts} ports={_PORTS} profiles={profiles}")
    _print_network_context(hosts)
    for profile_id in profiles:
        if len(profiles) > 1:
            print(f"\n--- profile {profile_id} ---")
        for host in hosts:
            print(f"\n=== {host} ===")
            for port in _PORTS:
                print(f"port {port}:")
                if _probe(host, int(port), profile_id, float(args.timeout)):
                    print(
                        f"\nSUCCESS: VGCS Settings → Host={host}, UDP port={port}, "
                        f"profile={profile_id}"
                    )
                    return 0
                time.sleep(0.05)
    if float(args.listen) > 0:
        _listen_udp(5000, float(args.listen))
    print("\nFAILED: no TOP attitude on any host/port. Video RTSP can still work.")
    print("Conclusion: this PC path cannot reach C13 gimbal UDP (not a VGCS bug).")
    print("Next steps:")
    print("  1) Connect PC Ethernet to the camera; PC IP should become 192.168.144.x")
    print("  2) Re-run: py tools/skydroid_gimbal_probe.py --hosts 192.168.144.108")
    print("  3) If still FAILED, gimbal may only be available on the phone app / UART, not PC LAN")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
