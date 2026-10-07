"""Simulated drones for a live demo of VGCS (milestone M17).

Starts ArduCopter simulators in WSL, in real time, and keeps them running.
Connect VGCS (VGCS.exe or `py -m vgcs`) to the address this prints, and fly
them from VGCS as if they were real drones. Ctrl+C stops them.

    py system_test/demo_sitl.py                  # one drone, ArduCopter 4.7.0
    py system_test/demo_sitl.py --drones 3       # three drones, for the Fleet panel
    py system_test/demo_sitl.py --version 4.6.2
    py system_test/demo_sitl.py --wind 9         # a steady 9 m/s wind from the north
    py system_test/demo_sitl.py --wind 9 --wind-failsafe   # plus the wind failsafe script

Everything stays on this computer: the simulators listen on 127.0.0.1 only.
Set up once as in system_test/README.md. Only one set of simulators can run at
a time, so do not start a system test while this runs.
"""
from __future__ import annotations

import argparse
import math
import pathlib
import sys
import time

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "drone" / "test"))

from sitl_session import Sitl  # noqa: E402

DEFAULT_HOME = (20.4347, 72.8696)  # the system tests' home
SPACING_M = 40.0                   # drones of a fleet stand this far apart, west to east
WIND_SCRIPT = REPO / "drone" / "scripts" / "vama_wind_failsafe.lua"
WIND_READY = "Wind failsafe: ready"
# Drag values of the simulated quadcopter, so that the autopilot estimates the wind
# (the same as drone/test/wind_failsafe_test.py).
DRAG = {"EK3_DRAG_BCOEF_X": 17.209, "EK3_DRAG_BCOEF_Y": 17.209, "EK3_DRAG_MCOEF": 0.209}


def home_of(lat: float, lon: float, index: int) -> str:
    east = index * SPACING_M
    return f"{lat},{lon + east / (111_320.0 * math.cos(math.radians(lat))):.7f},30,0"


def main() -> int:
    ap = argparse.ArgumentParser(description="Simulated drones for a live VGCS demo.")
    ap.add_argument("--version", default="4.7.0", choices=["4.6.2", "4.7.0"], help="ArduCopter version")
    ap.add_argument("--drones", type=int, default=1, choices=[1, 2, 3], help="how many drones")
    ap.add_argument("--home", default=f"{DEFAULT_HOME[0]},{DEFAULT_HOME[1]}",
                    help="latitude,longitude of the first drone")
    ap.add_argument("--wind", type=float, default=0.0, help="steady wind in m/s")
    ap.add_argument("--wind-dir", type=float, default=0.0, help="where the wind comes from, degrees")
    ap.add_argument("--wind-failsafe", action="store_true",
                    help="run drone/scripts/vama_wind_failsafe.lua on the drones")
    ap.add_argument("--wind-action", type=int, default=1, choices=[0, 1, 2, 3],
                    help="WFS_ACTION for the script: 0 off, 1 warning only (as installed), 2 RTL, 3 land")
    ap.add_argument("--minutes", type=float, default=0.0, help="stop by itself after this long (0: only Ctrl+C)")
    args = ap.parse_args()

    lat, lon = (float(v) for v in args.home.split(","))
    params: dict[str, float] = {}
    if args.wind or args.wind_failsafe:
        # SIM_WIND_T 1: the same wind at every height.
        params.update({"SIM_WIND_SPD": args.wind, "SIM_WIND_DIR": args.wind_dir, "SIM_WIND_T": 1}, **DRAG)
    scripts: dict[str, pathlib.Path] = {}
    script_params: dict[str, float] = {}
    ready = ""
    if args.wind_failsafe:
        scripts["vama_wind_failsafe.lua"] = WIND_SCRIPT
        params.update({"SCR_ENABLE": 1, "SCR_HEAP_SIZE": 300000})
        script_params["WFS_ACTION"] = args.wind_action
        ready = WIND_READY

    sims: list[Sitl] = []
    try:
        # Instance 0 first: it clears out any simulator still running.
        for i in range(args.drones):
            print(f"Starting drone {i + 1} (ArduCopter {args.version})...", flush=True)
            sims.append(Sitl(args.version, scripts, params, script_params=script_params, script_ready=ready,
                             speedup=1, home=home_of(lat, lon, i), instance=i, sysid=i + 1))
        print()
        for i, sim in enumerate(sims):
            print(f"Drone {i + 1} (system id {i + 1}): in VGCS, connect to tcp:127.0.0.1:{sim.port + 2}")
        print(f"A second ground station can use tcp:127.0.0.1:{sims[0].port + 3} (for the signing demo).")
        print("The drones' own messages show below. Ctrl+C stops the drones.")
        print(flush=True)
        shown = [0] * len(sims)  # every message since start-up, including the script's "ready"
        stop_at = time.monotonic() + args.minutes * 60.0 if args.minutes > 0 else math.inf
        while time.monotonic() < stop_at:
            for i, sim in enumerate(sims):
                sim.pump(0.2)
                for _, text in sim.texts[shown[i]:]:
                    print(f"[{time.strftime('%H:%M:%S')}] drone {i + 1}: {text}", flush=True)
                shown[i] = len(sim.texts)
        print("Time is up.")
    except KeyboardInterrupt:
        print("\nStopping the drones.")
    finally:
        # Instance 0 last: closing it stops every simulator.
        for sim in reversed(sims):
            sim.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
