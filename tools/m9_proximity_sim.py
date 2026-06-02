"""
M9 desk test — UDP MAVLink source for OBSTACLE_DISTANCE + DISTANCE_SENSOR.

Run alongside VGCS connected to ``udp:127.0.0.1:14550`` (default).

  py -3 tools/m9_proximity_sim.py

Move the simulated obstacle by editing ``--bearing-deg`` / ``--distance-m`` or watch
the default sweep animation on the map **OBSTACLE SENSORS** panel.
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
from pymavlink.dialects.v20 import ardupilotmega as mav_apm  # noqa: E402


def _make_distances_cm(
    *,
    bearing_deg: float,
    distance_m: float,
    beam_deg: float = 15.0,
    max_distance_m: float = 30.0,
) -> list[int]:
    """Fill 72 bins (5° increment) with one obstacle arc."""
    no_reading = 0xFFFF
    distances = [no_reading] * 72
    half = max(1.0, beam_deg / 2.0)
    dist_cm = int(max(20, min(max_distance_m, distance_m)) * 100)
    for i in range(72):
        center = float(i) * 5.0
        delta = (center - bearing_deg + 180.0) % 360.0 - 180.0
        if abs(delta) <= half:
            distances[i] = dist_cm
    return distances


def main() -> int:
    ap = argparse.ArgumentParser(description="M9 proximity MAVLink UDP simulator")
    ap.add_argument("--bind", default="127.0.0.1:14550", help="listen address (host:port)")
    ap.add_argument("--sysid", type=int, default=1)
    ap.add_argument("--compid", type=int, default=1)
    ap.add_argument("--hz", type=float, default=5.0, help="telemetry rate")
    ap.add_argument("--bearing-deg", type=float, default=-1.0, help="fixed bearing (0=fwd); -1 = sweep")
    ap.add_argument("--distance-m", type=float, default=4.5, help="obstacle distance")
    ap.add_argument("--rangefinder-m", type=float, default=3.2, help="DISTANCE_SENSOR distance")
    args = ap.parse_args()

    host, port_s = str(args.bind).rsplit(":", 1)
    port = int(port_s)

    mav = mavutil.mavlink_connection(
        f"udpin:{host}:{port}",
        dialect="ardupilotmega",
        source_system=255,
        source_component=190,
    )
    print(f"[m9-sim] listening on udpin:{host}:{port}")
    print("[m9-sim] In VGCS: Connect → udp:127.0.0.1:14550")
    print("[m9-sim] Ctrl+C to stop")

    target = None
    t0 = time.monotonic()
    interval = 1.0 / max(0.5, float(args.hz))

    while True:
        msg = mav.recv_match(blocking=False, timeout=0.05)
        if msg is not None and hasattr(msg, "get_srcSystem"):
            try:
                target = (msg.get_srcSystem(), msg.get_srcComponent())
            except Exception:
                pass

        now = time.monotonic()
        if target is None:
            continue
        if now - getattr(main, "_last_tx", 0.0) < interval:
            continue
        main._last_tx = now  # type: ignore[attr-defined]

        ts, tc = target
        bearing = float(args.bearing_deg)
        if bearing < 0:
            bearing = (math.sin((now - t0) * 0.7) * 0.5 + 0.5) * 340.0

        distances = _make_distances_cm(bearing_deg=bearing, distance_m=float(args.distance_m))

        mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_QUADROTOR,
            mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )
        mav.mav.obstacle_distance_send(
            int(time.time() * 1_000_000),
            mav_apm.MAV_DISTANCE_SENSOR_LASER,
            distances,
            0,  # increment (5° when zero per MAVLink spec)
            20,  # min_distance cm
            int(float(args.distance_m) * 100 * 3),  # max_distance cm
            5.0,  # increment_f (degrees)
            0.0,  # angle_offset (degrees)
            mav_apm.MAV_FRAME_BODY_FRD,
        )
        mav.mav.distance_sensor_send(
            int(time.time() * 1_000_000),
            0,  # id
            mav_apm.MAV_DISTANCE_SENSOR_LASER,
            mav_apm.MAV_SENSOR_ROTATION_PITCH_270,  # DOWN
            20,
            int(float(args.rangefinder_m) * 100 * 3),
            int(float(args.rangefinder_m) * 100),
            0,  # covariance
        )

        print(
            f"\r[m9-sim] → sys={ts} bearing={bearing:5.1f}° dist={args.distance_m:.1f}m rf={args.rangefinder_m:.1f}m",
            end="",
            flush=True,
        )


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n[m9-sim] stopped")
        raise SystemExit(0)
