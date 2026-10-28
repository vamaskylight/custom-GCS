"""python -m vama_nav --sim-truth [--device TCP_OR_SERIAL] [--control FILE] ..."""

from __future__ import annotations

import argparse
import contextlib
import logging
import os
import random
import signal
import sys
from pathlib import Path

from . import __version__

MAV_COMP_ID_GPS2 = 221


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="vama_nav",
                                description="Sends the drone's position from the camera to the flight controller "
                                            "as its second GPS (GPS-denied navigation, step 3a).")
    p.add_argument("--sim-truth", action="store_true",
                   help="positions from the ArduCopter simulator's true position with noise (the only source for now)")
    p.add_argument("--device", default="/dev/serial0", help="serial port, or tcp:HOST:PORT (default /dev/serial0)")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--source-system", type=int, default=1, help="the drone's system id (default 1)")
    p.add_argument("--source-component", type=int, default=MAV_COMP_ID_GPS2)
    p.add_argument("--noise-m", type=float, default=2.0, help="--sim-truth: position error, standard deviation")
    p.add_argument("--speed-noise", type=float, default=0.2, help="--sim-truth: speed error (m/s), standard deviation")
    p.add_argument("--every-s", type=float, default=1.0, help="--sim-truth: seconds between two positions")
    p.add_argument("--delay-s", type=float, default=0.4, help="--sim-truth: seconds a position takes to be ready")
    p.add_argument("--match-above-m", type=float, default=15.0, help="--sim-truth: no positions below this height")
    p.add_argument("--control", type=Path, help="--sim-truth: a JSON file a test changes (offset, pause)")
    p.add_argument("--seed", type=int, help="--sim-truth: the same errors each run")
    p.add_argument("--version", action="version", version=f"vama_nav {__version__}")
    args = p.parse_args(argv)
    if not args.sim_truth:
        print("no position source: the picture matching (step 3) is not written yet. --sim-truth uses the "
              "ArduCopter simulator's true position.", file=sys.stderr)
        return 2
    for name in ("noise_m", "speed_noise", "delay_s"):
        if getattr(args, name) < 0:
            print(f"--{name.replace('_', '-')} cannot be below 0", file=sys.stderr)
            return 2
    if args.every_s <= 0:
        print("--every-s must be more than 0", file=sys.stderr)
        return 2

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s",
                        datefmt="%Y-%m-%d %H:%M:%S", stream=sys.stderr)
    log = logging.getLogger("vama_nav")
    # MAVLink 2 from the first byte: pymavlink reads this when it is first imported (by .runner).
    os.environ.setdefault("MAVLINK20", "1")
    from pymavlink import mavutil

    from .gps_input import Sender
    from .runner import Runner
    from .sim_truth import SimTruth

    try:
        conn = mavutil.mavlink_connection(args.device, baud=args.baud, source_system=args.source_system,
                                          source_component=args.source_component, autoreconnect=True)
    except Exception as exc:              # a serial port that is not there, a refused TCP port
        log.error("cannot open the link %s (%s)", args.device, exc)
        return 4
    sender = Sender()
    source = SimTruth(sender.offer, noise_m=args.noise_m, speed_noise_mps=args.speed_noise, every_s=args.every_s,
                      delay_s=args.delay_s, match_above_m=args.match_above_m, control=args.control,
                      rng=random.Random(args.seed))
    runner = Runner(conn, sender, source)
    log.info("vama_nav %s on %s: GPS_INPUT for GPS 2, positions from the simulator's truth (noise %.1f m, "
             "one each %.1f s, ready %.1f s later, above %.0f m)", __version__, args.device, args.noise_m,
             args.every_s, args.delay_s, args.match_above_m)

    def stop(signum, _frame) -> None:
        log.info("stopping (signal %s)", signum)
        runner.stop_event.set()

    for name in ("SIGTERM", "SIGINT"):
        if hasattr(signal, name):
            signal.signal(getattr(signal, name), stop)
    try:
        runner.run()
    finally:
        log.info("sent %d (camera fixes %d, carried forward %d, GPS 1 passed on %d, no fix %d); camera positions "
                 "%d made, %d refused", sum(runner.sent.values()), runner.sent["camera"], runner.sent["carried"],
                 runner.sent["gps1"], runner.sent["none"], source.made, sender.refused)
        with contextlib.suppress(Exception):
            conn.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
