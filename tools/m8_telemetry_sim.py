"""
M8 desk test — UDP MAVLink source for GPS + attitude + gimbal (MOUNT_ORIENTATION).

Run alongside VGCS connected to ``udp:127.0.0.1:14550`` (default).

  py -3 tools/m8_telemetry_sim.py

In VGCS: Connect → udp:127.0.0.1:14550, enable video, Target ON, click video center,
then Report. CSV should show target_lat/target_lon with geo_quality good/fair.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from pymavlink import mavutil  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="M8 geo-referencing MAVLink UDP simulator")
    ap.add_argument("--bind", default="127.0.0.1:14550", help="listen address (host:port)")
    ap.add_argument("--hz", type=float, default=5.0, help="telemetry rate")
    ap.add_argument("--lat", type=float, default=37.4275, help="vehicle latitude")
    ap.add_argument("--lon", type=float, default=-122.1697, help="vehicle longitude")
    ap.add_argument("--rel-alt-m", type=float, default=80.0, help="AGL for geo intersection")
    ap.add_argument("--gimbal-pitch-deg", type=float, default=-45.0, help="mount pitch (down negative)")
    ap.add_argument("--gimbal-yaw-deg", type=float, default=15.0, help="mount yaw")
    args = ap.parse_args()

    host, port_s = str(args.bind).rsplit(":", 1)
    port = int(port_s)

    mav = mavutil.mavlink_connection(
        f"udpin:{host}:{port}",
        dialect="ardupilotmega",
        source_system=1,
        source_component=1,
    )
    print(f"[m8-sim] listening on udpin:{host}:{port}")
    print("[m8-sim] VGCS: Connect → udp:127.0.0.1:14550")
    print("[m8-sim] Camera provider: MAVLink (or Skydroid with mavlink fallback)")
    print("[m8-sim] Target ON → click video → Report → check target_lat/lon in CSV")
    print("[m8-sim] Ctrl+C to stop")

    lat_e7 = int(float(args.lat) * 1e7)
    lon_e7 = int(float(args.lon) * 1e7)
    rel_cm = int(float(args.rel_alt_m) * 1000)
    msl_cm = rel_cm + 50_000
    interval = 1.0 / max(0.5, float(args.hz))
    last_tx = 0.0

    while True:
        mav.recv_match(blocking=False, timeout=0.05)
        now = time.monotonic()
        if now - last_tx < interval:
            continue
        last_tx = now
        t_us = int(time.time() * 1_000_000)

        mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_QUADROTOR,
            mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )
        mav.mav.global_position_int_send(
            t_us,
            lat_e7,
            lon_e7,
            msl_cm,
            rel_cm,
            0,
            0,
            0,
            0,
        )
        mav.mav.gps_raw_int_send(
            t_us,
            3,
            lat_e7,
            lon_e7,
            msl_cm,
            65535,
            0,
            0,
            12,
            120,
            80,
        )
        mav.mav.attitude_send(
            t_us,
            math.radians(2.0),
            math.radians(-3.0),
            math.radians(45.0),
            0.0,
            0.0,
            0.0,
        )
        mav.mav.mount_orientation_send(
            float(args.gimbal_yaw_deg),
            float(args.gimbal_pitch_deg),
            0.0,
        )

        print(
            f"\r[m8-sim] lat={args.lat:.5f} lon={args.lon:.5f} "
            f"agl={args.rel_alt_m:.0f}m gimbal y={args.gimbal_yaw_deg:.0f} p={args.gimbal_pitch_deg:.0f}",
            end="",
            flush=True,
        )


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n[m8-sim] stopped")
        raise SystemExit(0)
