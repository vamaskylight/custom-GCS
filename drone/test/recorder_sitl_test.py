"""Try the flight recorder (GPS-denied navigation, step 1) against the ArduCopter simulator.

The recorder runs as its own program, as on the Raspberry Pi, with a stand-in camera, and talks to
the simulated flight controller on a telemetry port. This test flies the drone, and compares every
picture's row with the simulator's own samples, which it reads on another port.

Usage:
    py recorder_sitl_test.py              4.6.2 and 4.7.0
    py recorder_sitl_test.py 4.7.0        one version

The simulator runs in real time (speedup 1): the recorder matches the flight controller's clock to
the computer's, one second to one second, as on a drone.

Ports:
    5760  this test: it flies the drone and reads its position and angles (10 a second)
    5763  the recorder, as on a telemetry port of the Cube
"""
from __future__ import annotations

import bisect
import csv
import json
import math
import os
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile
import time

os.environ.setdefault("MAVLINK20", "1")

from sitl_session import Sitl  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
PACKAGE_DIR = HERE.parent / "gps_denied"
VERSIONS = ["4.6.2", "4.7.0"]
RECORDER_PORT = 5763
START_HEIGHT_M = 20.0
FLY_NORTH_S = 15.0


class Report:
    def __init__(self, name: str) -> None:
        self.name = name
        self.lines: list[str] = []
        self.failed = 0

    def check(self, ok: bool, text: str) -> bool:
        self.lines.append(("    ok    " if ok else "    FAIL  ") + text)
        if not ok:
            self.failed += 1
        return ok

    def note(self, text: str) -> None:
        self.lines.append("          " + text)


class Truth:
    """What the simulator itself says on the test's port, by its time_boot_ms."""

    def __init__(self, sim: Sitl) -> None:
        self.position: list[tuple] = []   # (boot_ms, lat, lon, rel_alt)
        self.attitude: list[tuple] = []   # (boot_ms, yaw, roll, pitch)
        self.armed: list[tuple] = []      # (boot_ms of the last ATTITUDE, armed)
        self.arrivals: list[tuple] = []   # (this computer's clock when it arrived, time_boot_ms)
        take = sim._take

        def taking(msg) -> None:
            kind = msg.get_type()
            if kind in ("ATTITUDE", "GLOBAL_POSITION_INT"):
                self.arrivals.append((time.monotonic_ns(), int(msg.time_boot_ms)))
            if kind == "GLOBAL_POSITION_INT":
                self.position.append((msg.time_boot_ms, msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0))
            elif kind == "ATTITUDE":
                self.attitude.append((msg.time_boot_ms, math.degrees(msg.yaw) % 360.0, math.degrees(msg.roll),
                                      math.degrees(msg.pitch)))
            take(msg)
            if kind == "HEARTBEAT" and msg.get_srcComponent() == 1 and self.attitude:
                self.armed.append((self.attitude[-1][0], sim.armed))

        sim._take = taking

    @staticmethod
    def _at(samples: list[tuple], t: float, angle: int | None = None):
        times = [s[0] for s in samples]
        i = bisect.bisect_left(times, t)
        if i == 0 or i >= len(samples):
            return None
        a, b = samples[i - 1], samples[i]
        f = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
        out = []
        for k, (va, vb) in enumerate(zip(a[1:], b[1:], strict=True)):
            out.append(va + f * ((((vb - va) + 180.0) % 360.0) - 180.0) if k == angle else va + f * (vb - va))
        return out

    def position_at(self, t: float):
        return self._at(self.position, t)

    def boot_ms_at(self, local_ns: int) -> float | None:
        """The simulator's time_boot_ms at this computer's ``local_ns``, from this test's own link: the
        fastest message of each tenth of a second within one second around it, and a line through them
        (the simulator's clock runs a few percent slower than real time)."""
        fastest: dict[int, tuple[int, int]] = {}
        for arrival, boot in self.arrivals:
            if abs(arrival - local_ns) <= 1_000_000_000:
                k = arrival // 100_000_000
                offset = arrival - boot * 1_000_000
                if k not in fastest or offset < fastest[k][1]:
                    fastest[k] = (arrival, offset)
        if len(fastest) < 5:
            return None
        pts = list(fastest.values())
        mean_t = sum(a for a, _o in pts) / len(pts)
        mean_o = sum(o for _a, o in pts) / len(pts)
        var = sum((a - mean_t) ** 2 for a, _o in pts)
        slope = sum((a - mean_t) * (o - mean_o) for a, o in pts) / var if var else 0.0
        return (local_ns - (mean_o + slope * (local_ns - mean_t))) / 1e6

    def attitude_at(self, t: float):
        return self._at(self.attitude, t, angle=0)


def metres_between(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    north = (lat2 - lat1) * 111_320.0
    east = (lon2 - lon1) * 111_320.0 * math.cos(math.radians(lat1))
    return math.hypot(north, east)


def recorder_command(config: pathlib.Path, *extra: str) -> list[str]:
    return [sys.executable, "-m", "vama_recorder", "--config", str(config), "--fake-camera", *extra]


def recorder_in_flight(version: str, r: Report) -> None:
    """The recorder on the Cube's telemetry port: the bench check passes, a flight is recorded from 20 m until disarming, one picture a second, each row at the drone's real position and angles at its moment, and the recorder's own stream requests come through."""
    work = pathlib.Path(tempfile.mkdtemp(prefix="vama-recorder-A-"))
    config = work / "recorder.ini"
    config.write_text(f"[link]\ndevice = tcp:127.0.0.1:{RECORDER_PORT}\n[camera]\ninterval_s = 1.0\n"
                      f"[recording]\nfolder = {work / 'flights'}\nstart_height_m = {START_HEIGHT_M:g}\n"
                      "min_free_mb = 100\n", encoding="utf-8")
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    sim = Sitl(version, {}, {"RC_OVERRIDE_TIME": -1, "SIM_WIND_SPD": 0}, speedup=1)
    proc = None
    log_file = (work / "recorder.out").open("w", encoding="utf-8")
    try:
        truth = Truth(sim)
        sim.pump(3)
        check = subprocess.run(recorder_command(config, "--check"), cwd=PACKAGE_DIR, capture_output=True, text=True,
                               timeout=120, env=env)
        r.check(check.returncode == 0, "the bench check passes against the simulated flight controller"
                + ("" if check.returncode == 0 else ": " + " | ".join(check.stdout.splitlines()[-6:])))
        for line in check.stdout.splitlines():
            if line.startswith(("OK      ATTITUDE", "OK      GLOBAL", "PROBLEM")):
                r.note("check: " + line.strip())
        proc = subprocess.Popen(recorder_command(config, "--flights", "1"), cwd=PACKAGE_DIR, env=env,
                                stdout=log_file, stderr=subprocess.STDOUT)
        sim.pump(3)
        r.check(proc.poll() is None, "the recorder runs and waits for the flight")
        sim.arm_and_takeoff(40)
        r.check(sim.armed and sim.alt > 37, f"armed and at 40 m ({sim.alt:.1f} m)")
        end = time.monotonic() + FLY_NORTH_S
        while time.monotonic() < end:
            sim.velocity(5.0, 0.0)
            sim.pump(1.0)
        r.check(sim.north > 40, f"flew north ({sim.north:.0f} m)")
        sim.set_mode("LAND")
        sim.pump(120, until=lambda last: not sim.armed)
        r.check(not sim.armed, "landed and disarmed")
        try:
            code = proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            code = None
        r.check(code == 0, f"the recorder ended by itself after the flight (exit {code})")
    finally:
        if proc is not None and proc.poll() is None:
            proc.kill()
        log_file.close()
        sim.close()

    folders = sorted(p for p in (work / "flights").iterdir() if p.is_dir()) if (work / "flights").is_dir() else []
    if not r.check(len(folders) == 1, f"one flight folder ({[p.name for p in folders]})"):
        r.note(f"kept for a look: {work}")
        return
    folder = folders[0]
    info = json.loads((folder / "flight.json").read_text(encoding="utf-8"))
    rows = list(csv.DictReader((folder / "frames.csv").open(encoding="utf-8")))
    r.check(info["end_reason"] == "disarmed" and info["frames"] == info["rows"] == len(rows),
            f"closed at disarming, every picture has its row ({info['end_reason']}, {info['frames']} pictures, "
            f"{len(rows)} rows)")
    steps = [(int(b["local_ns"]) - int(a["local_ns"])) / 1e9 for a, b in zip(rows, rows[1:], strict=False)]
    r.check(bool(steps) and all(abs(x - 1.0) < 0.1 for x in steps),
            f"one picture a second ({min(steps, default=0):.2f} to {max(steps, default=0):.2f} s apart)")
    above = [t for t, *_rest in truth.position if _rest[2] >= START_HEIGHT_M]
    disarmed_at = next((t for t, armed in truth.armed if not armed and above and t > above[0]), None)
    if rows and above and disarmed_at is not None:
        first, last = float(rows[0]["ap_boot_ms"]), float(rows[-1]["ap_boot_ms"])
        r.check(abs(first - above[0]) < 1500 and float(rows[0]["rel_alt_m"]) >= START_HEIGHT_M - 1.0,
                f"from {START_HEIGHT_M:g} m (first picture at {float(rows[0]['rel_alt_m']):.1f} m)")
        r.check(0 <= disarmed_at - last < 2500, f"until disarming (last picture {(disarmed_at - last) / 1000:.1f} s "
                                                f"before it)")
    timing = []
    for row in rows:
        should = truth.boot_ms_at(int(row["local_ns"]))
        if should is not None:
            timing.append(float(row["ap_boot_ms"]) - should)
    if r.check(len(timing) >= len(rows) - 2 and len(timing) > 10,
               f"each picture's moment checked against this test's own link ({len(timing)} of {len(rows)})"):
        # 50 ms: the simulator's clock runs a few percent slow and unevenly, and this test's own estimate
        # goes over another link (2026-10-28: worst 24 to 26 ms, middle 2.5 to 3 ms; the first way of
        # matching the clocks was a second behind). The unit tests check against an exact truth at 3 ms.
        worst = max(abs(x) for x in timing)
        r.check(worst < 50.0, f"each picture placed on the flight controller's clock within 50 ms (worst "
                              f"{worst:.1f} ms, middle {sorted(timing)[len(timing) // 2]:.1f} ms)")
    worst_pos = worst_alt = worst_yaw = 0.0
    compared = interpolated = 0
    for row in rows:
        t = float(row["ap_boot_ms"])
        pos = truth.position_at(t)
        att = truth.attitude_at(t)
        if pos is None or att is None:
            continue
        compared += 1
        interpolated += row["position_how"] == "interpolated" and row["attitude_how"] == "interpolated"
        worst_pos = max(worst_pos, metres_between(pos[0], pos[1], float(row["lat"]), float(row["lon"])))
        worst_alt = max(worst_alt, abs(pos[2] - float(row["rel_alt_m"])))
        worst_yaw = max(worst_yaw, abs((((float(row["yaw_deg"]) - att[0]) + 180.0) % 360.0) - 180.0))
    r.check(compared >= len(rows) - 2 and compared > 10, f"rows compared with the simulator's own samples "
                                                         f"({compared} of {len(rows)})")
    r.check(interpolated >= compared - 1, f"each row between the controller's samples ({interpolated} of {compared})")
    r.check(worst_pos < 1.0, f"position within 1 m of the simulator's at the same moment (worst {worst_pos:.2f} m)")
    r.check(worst_alt < 0.5, f"height within 0.5 m (worst {worst_alt:.2f} m)")
    r.check(worst_yaw < 2.0, f"heading within 2 degrees (worst {worst_yaw:.2f})")
    r.check(all((folder / row["image"]).is_file() for row in rows), "every row's picture is there")
    kinds: dict[str, int] = {}
    for line in (folder / "telemetry.jsonl").read_text(encoding="utf-8").splitlines():
        kind = json.loads(line)["mavpackettype"]
        kinds[kind] = kinds.get(kind, 0) + 1
    seconds = max(1.0, len(rows))
    r.check(kinds.get("ATTITUDE", 0) / seconds >= 15, f"the recorder's own stream requests came through: ATTITUDE "
                                                       f"{kinds.get('ATTITUDE', 0) / seconds:.1f} a second (asked 20)")
    r.check(kinds.get("GLOBAL_POSITION_INT", 0) / seconds >= 7,
            f"GLOBAL_POSITION_INT {kinds.get('GLOBAL_POSITION_INT', 0) / seconds:.1f} a second (asked 10)")
    r.check(bool(rows) and rows[0]["utc"].endswith("Z") and int(rows[0]["gps_fix"] or 0) >= 3,
            f"GPS time and fix in the rows ({rows[0]['utc'] if rows else '-'}, fix {rows[0]['gps_fix'] if rows else '-'})")
    log_text = (work / "flights" / "recorder.log").read_text(encoding="utf-8") if (work / "flights" / "recorder.log").exists() else ""
    errors = [line for line in log_text.splitlines() if " ERROR " in line]
    r.check(not errors, f"no errors in the recorder's log {errors[:2]}")
    r.note(f"clock offset {info.get('clock_offset_ms_start')} ms at the start, {info.get('clock_offset_ms_end')} ms at "
           f"the end, controller restarts {info.get('controller_restarts')}")
    if r.failed:
        r.note(f"kept for a look: {work}")
    else:
        shutil.rmtree(work, ignore_errors=True)


CASES = [recorder_in_flight]


def main(argv: list[str]) -> int:
    versions = [a for a in argv if re.fullmatch(r"\d+\.\d+\.\d+", a)] or VERSIONS
    failed_cases = 0
    began = time.monotonic()
    for version in versions:
        print(f"\n===== ArduCopter {version}")
        for case in CASES:
            report = Report(case.__name__)
            try:
                case(version, report)
            except Exception as exc:  # a broken simulator run is a failed case, not a crash of the whole run
                report.check(False, f"the case stopped: {exc}")
            print(f"  {'FAIL' if report.failed else 'PASS'}  {case.__name__}: {(case.__doc__ or '').strip()}")
            for line in report.lines:
                print(line)
            sys.stdout.flush()
            failed_cases += 1 if report.failed else 0
    total = len(CASES) * len(versions)
    print(f"\n{total - failed_cases} of {total} cases passed in {time.monotonic() - began:.0f} s")
    return 1 if failed_cases else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
