"""Try the GPS_INPUT sender (GPS-denied navigation, step 3a) against the ArduCopter simulator.

The sender runs as its own program, as it will on the Raspberry Pi, on a telemetry port of the
simulated flight controller. Its positions are made from the simulator's true position plus noise,
like a camera's: one a second, ready 0.4 s after its moment, only above 15 m. The drone flies a
mission in AUTO. Its real GPS is switched off for two legs and on again, and at the end both
sources go, to show that the camera's positions were what kept it on course.

Usage:
    py gps_input_sitl_test.py              4.6.2 and 4.7.0
    py gps_input_sitl_test.py 4.7.0        one version

The simulator runs in real time (speedup 1): the sender sends ten times a second of this computer's
clock, and ArduPilot calls a GPS unhealthy below about five a second of its own.

Ports:
    5760  this test: it flies the drone and reads the simulator's true position (SIM_STATE)
    5763  the sender, as on a telemetry port of the Cube, made private (MAV3_OPTIONS 2 on 4.7,
          SERIAL2_OPTIONS 1024 on 4.6) so its messages do not go on to the other links
"""
from __future__ import annotations

import bisect
import json
import math
import os
import pathlib
import re
import shutil
import statistics
import subprocess
import sys
import tempfile
import time

os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil  # noqa: E402

from sitl_session import GPS_OFF, Sitl  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
PACKAGE_DIR = HERE.parent / "gps_denied"
VERSIONS = ["4.6.2", "4.7.0"]
SENDER_PORT = 5763
HOME = (20.4347, 72.8696)                  # Sitl's home
ALT_M = 40.0
LEG_M = 250.0
NOISE_M = 2.0
OFFSET_M = 150.0
NAV_PARAMS = {"GPS2_TYPE": 14, "GPS_AUTO_SWITCH": 4, "GPS_PRIMARY": 0, "GPS2_DELAY_MS": 20}
PRIVATE = {"4.6": {"SERIAL2_OPTIONS": 1024}, "4.7": {"MAV3_OPTIONS": 2}}
WP4 = (0.0, 60.0)
# (command, (param 1 to 4), metres north and east of home, height). Take-off and RTL have no place.
MISSION = [
    (16, (0, 0, 0, 0), 0.0, 0.0, 0.0),             # 0: home (the drone keeps its own)
    (22, (0, 0, 0, 0), None, None, ALT_M),         # 1: take off to 40 m
    (16, (0, 0, 0, 0), LEG_M, 0.0, ALT_M),         # 2: WP1
    (16, (0, 0, 0, 0), LEG_M, LEG_M, ALT_M),       # 3: WP2
    (16, (0, 0, 0, 0), 0.0, LEG_M, ALT_M),         # 4: WP3
    (19, (120, 0, 0, 0), *WP4, ALT_M),             # 5: WP4, hold there 2 minutes
    (20, (0, 0, 0, 0), None, None, 0.0),           # 6: RTL
]


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


def metres_between(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    north = (lat2 - lat1) * 111_320.0
    east = (lon2 - lon1) * 111_320.0 * math.cos(math.radians(lat1))
    return math.hypot(north, east)


def place(north_m: float, east_m: float) -> tuple[float, float]:
    return (HOME[0] + north_m / 111_320.0,
            HOME[1] + east_m / (111_320.0 * math.cos(math.radians(HOME[0]))))


class Watch:
    """What the simulator says on this test's port, by this computer's clock: its true position
    (SIM_STATE), the EKF's (GLOBAL_POSITION_INT), GPS 1 and GPS 2 as ArduPilot sees them."""

    def __init__(self, sim: Sitl) -> None:
        self.truth: list[tuple] = []        # (arrival_ns, lat, lon)
        self.ekf: list[tuple] = []          # (arrival_ns, lat, lon, height above home)
        self.gps1: list[tuple] = []         # (arrival_ns, fix, lat, lon)
        self.gps2: list[tuple] = []         # (arrival_ns, fix, lat, lon)
        self.ekf_status: list[tuple] = []   # (arrival_ns, velocity test, position test)
        self.mission_seq = 0
        self.forwarded = 0                  # the sender's GPS_INPUT, passed on to this port
        take = sim._take

        def taking(msg) -> None:
            kind = msg.get_type()
            now = time.monotonic_ns()
            if kind == "SIM_STATE":
                self.truth.append((now, msg.lat_int / 1e7, msg.lon_int / 1e7))
            elif kind == "GLOBAL_POSITION_INT":
                self.ekf.append((now, msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0))
            elif kind == "GPS_RAW_INT":
                self.gps1.append((now, int(msg.fix_type), msg.lat / 1e7, msg.lon / 1e7))
            elif kind == "GPS2_RAW":
                self.gps2.append((now, int(msg.fix_type), msg.lat / 1e7, msg.lon / 1e7))
            elif kind == "EKF_STATUS_REPORT":
                self.ekf_status.append((now, float(msg.velocity_variance), float(msg.pos_horiz_variance)))
            elif kind == "MISSION_CURRENT":
                self.mission_seq = int(msg.seq)
            elif kind == "GPS_INPUT":
                self.forwarded += 1
            take(msg)

        sim._take = taking

    def truth_at(self, t_ns: int) -> tuple[float, float] | None:
        times = [s[0] for s in self.truth]
        i = bisect.bisect_left(times, t_ns)
        if i == 0 or i >= len(self.truth):
            return None
        a, b = self.truth[i - 1], self.truth[i]
        if b[0] - a[0] > 300_000_000:
            return None
        f = (t_ns - a[0]) / (b[0] - a[0])
        return a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2])

    def errors(self, samples: list[tuple], since_ns: int, until_ns: int, fixes=None) -> list[float]:
        """How far each sample (time, ..., lat, lon) was from the true position at its moment."""
        out = []
        for s in samples:
            if since_ns <= s[0] <= until_ns and (fixes is None or s[1] in fixes):
                truth = self.truth_at(s[0])
                if truth is not None:
                    out.append(metres_between(truth[0], truth[1], s[-2], s[-1]))
        return out

    def ekf_errors(self, since_ns: int, until_ns: int) -> list[float]:
        return self.errors([(t, lat, lon) for t, lat, lon, _h in self.ekf], since_ns, until_ns)

    def height(self) -> float:
        return self.ekf[-1][3] if self.ekf else 0.0


def upload(sim: Sitl, items: list[tuple]) -> int | None:
    """The mission on the test's link. Returns the drone's answer (0 is accepted), or None."""
    mav = sim.mav
    mav.mav.mission_count_send(mav.target_system, mav.target_component, len(items), 0)
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        msg = mav.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"], blocking=True, timeout=2)
        if msg is None:
            continue
        if msg.get_type() == "MISSION_ACK":
            return int(msg.type)
        command, p, north, east, height = items[int(msg.seq)]
        lat, lon = (0.0, 0.0) if north is None else place(north, east)
        mav.mav.mission_item_int_send(mav.target_system, mav.target_component, int(msg.seq),
                                      mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, command, 0, 1,
                                      p[0], p[1], p[2], p[3], int(lat * 1e7), int(lon * 1e7), height, 0)
    return None


def set_gps1(sim: Sitl, on: bool) -> None:
    name, off = GPS_OFF[sim.version[:3]]
    sim.set_param(name, (1 - off) if on else off)


def control(path: pathlib.Path, **values) -> None:
    """Steer the sender's stand-in camera (see vama_nav/sim_truth.py)."""
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(values), encoding="utf-8")
    os.replace(tmp, path)


def summary(values: list[float]) -> str:
    if not values:
        return "no samples"
    return f"middle {statistics.median(values):.1f} m, worst {max(values):.1f} m, {len(values)} samples"


def mission_without_gps(version: str, r: Report) -> None:
    """Arming needs the sender; GPS 2 is GPS 1 passed on while disarmed, no fix when armed low, camera positions above 15 m; a camera position 150 m off is ignored while GPS 1 works; with GPS 1 off the mission flies on the camera's positions; GPS 1 back is taken again; without both the EKF failsafe lands the drone; the sender's port stays private."""
    work = pathlib.Path(tempfile.mkdtemp(prefix="vama-nav-A-"))
    ctl = work / "control.json"
    control(ctl)
    params = {"RC_OVERRIDE_TIME": -1, "SIM_WIND_SPD": 0, **NAV_PARAMS, **PRIVATE[version[:3]]}
    sim = Sitl(version, {}, params, speedup=1)
    proc = None
    log_path = work / "sender.log"
    log_file = log_path.open("w", encoding="utf-8")
    try:
        w = Watch(sim)
        sim.mav.mav.command_long_send(sim.mav.target_system, sim.mav.target_component,
                                      mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 108, 100_000, 0, 0, 0, 0, 0)

        # No sender yet: GPS 2 is set up but has no fix, so ArduPilot must refuse to arm.
        sim.pump(120, until=lambda last: "EKF_STATUS_REPORT" in last
                 and (last["EKF_STATUS_REPORT"].flags & 0x18) == 0x18
                 and "GPS_RAW_INT" in last and last["GPS_RAW_INT"].fix_type >= 3)
        sim.rc(rc3=1000)
        sim.set_mode("GUIDED")
        since = sim.sim_s
        for _ in range(3):
            sim.mav.mav.command_long_send(sim.mav.target_system, sim.mav.target_component,
                                          mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
            sim.pump(2, until=lambda last: sim.armed)
        r.check(not sim.armed and sim.saw("GPS 2: Bad fix", since),
                "without the sender ArduPilot does not arm (" + " | ".join(sim.texts_with("GPS 2", since)[-1:]) + ")")

        env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
        proc = subprocess.Popen([sys.executable, "-m", "vama_nav", "--sim-truth", "--device",
                                 f"tcp:127.0.0.1:{SENDER_PORT}", "--noise-m", str(NOISE_M), "--control", str(ctl),
                                 "--seed", "7"], cwd=PACKAGE_DIR, env=env, stdout=log_file, stderr=subprocess.STDOUT)
        started = time.monotonic_ns()
        sim.pump(30, until=lambda last: any(t > started and fix >= 3 for t, fix, *_ in w.gps2[-3:]))
        echo = [s for s in w.gps2 if s[0] > started and s[1] >= 3]
        if r.check(bool(echo), "disarmed, GPS 2 has a 3D fix: the sender passes GPS 1 on"):
            gps1 = [s for s in w.gps1 if s[0] <= echo[-1][0]]
            apart = metres_between(gps1[-1][2], gps1[-1][3], echo[-1][2], echo[-1][3]) if gps1 else 99.0
            r.check(apart < 1.0, f"GPS 2 is where GPS 1 is ({apart:.2f} m apart)")

        r.check(upload(sim, MISSION) == 0, "mission accepted (take-off, 3 legs of 250 m, a hold, RTL)")
        sim.arm_and_takeoff(8)
        r.check(sim.armed, "the drone arms with GPS 2 set up and the sender running")
        low_from = time.monotonic_ns()
        sim.pump(3)
        low = [fix for t, fix, *_ in w.gps2 if t > low_from + 500_000_000]
        r.check(bool(low) and max(low) <= 1,
                f"armed at {w.height():.0f} m: GPS 2 has no fix (fix types {sorted(set(low))})")

        sim.rc(rc3=1500)
        sim.set_mode("AUTO")
        sim.pump(60, until=lambda last: w.mission_seq >= 2 and w.height() > ALT_M - 2)
        r.check(w.mission_seq >= 2 and w.height() > ALT_M - 2, f"climbed to {w.height():.0f} m in AUTO")
        climbed = time.monotonic_ns()
        sim.pump(3)
        before = w.errors(w.gps2, climbed - 8_000_000_000, time.monotonic_ns(), fixes=(2, 3))
        kinds = {fix for t, fix, *_ in w.gps2 if t > climbed - 8_000_000_000}
        r.check({2, 3} <= kinds, f"above 15 m GPS 2 carries the camera's positions: 3D fixes and 2D between "
                                 f"(fix types {sorted(kinds)})")
        r.check(bool(before) and statistics.median(before) < 4.0 and max(before) < 12.0,
                f"GPS 2 near the true position ({summary(before)}, noise {NOISE_M:g} m)")

        # A picture matched to the wrong place, while GPS 1 works: ArduPilot must not follow it.
        control(ctl, offset_north_m=OFFSET_M)
        wrong_from = time.monotonic_ns()
        sim.pump(10)
        control(ctl)
        wrong_to = time.monotonic_ns()
        sim.pump(3)
        off = w.errors(w.gps2, wrong_from + 2_500_000_000, wrong_to, fixes=(2, 3))
        r.check(bool(off) and min(off) > OFFSET_M - 30, f"the wrong positions reached GPS 2 ({summary(off)})")
        held = w.ekf_errors(wrong_from, wrong_to + 3_000_000_000)
        r.check(bool(held) and max(held) < 2.0, f"... and the drone kept to GPS 1 ({summary(held)})")

        sim.pump(90, until=lambda last: w.mission_seq >= 3)
        r.check(w.mission_seq >= 3, "WP1 reached")
        set_gps1(sim, False)
        gps_off = time.monotonic_ns()
        off_since = sim.sim_s
        sim.pump(120, until=lambda last: w.mission_seq >= 5 or not sim.armed)
        wp3 = time.monotonic_ns()
        r.check(w.mission_seq >= 5, f"GPS 1 off at WP1: the mission flew on to WP2 and WP3 "
                                    f"({(wp3 - gps_off) / 1e9:.0f} s, mission item {w.mission_seq})")
        lost = [fix for t, fix, *_ in w.gps1 if gps_off + 6_000_000_000 < t < wp3]
        r.check(bool(lost) and max(lost) < 3, f"GPS 1 had no fix there (fix types {sorted(set(lost))})")
        modes = [mode for t, mode in sim.mode_log if t >= off_since]
        r.check(sim.mode == "AUTO" and not modes, f"still AUTO, no mode change ({modes})")
        on_camera = w.ekf_errors(gps_off, wp3)
        r.check(bool(on_camera) and statistics.median(on_camera) < 4.0 and max(on_camera) < 10.0,
                f"the EKF stayed near the true position ({summary(on_camera)})")
        bad = [t for t in sim.texts_with("", off_since) if re.search(r"glitch|variance|failsafe|yaw reset", t, re.I)]
        r.check(not bad, f"no GPS glitch, EKF variance or failsafe message {bad[:3]}")
        tests = [(v, p) for t, v, p in w.ekf_status if gps_off < t < wp3]
        if tests:
            r.note(f"EKF test ratios while on the camera: velocity up to {max(v for v, _p in tests):.2f}, "
                   f"position up to {max(p for _v, p in tests):.2f} (1 is the gate)")

        set_gps1(sim, True)
        gps_on = time.monotonic_ns()
        sim.pump(12)
        back = w.ekf_errors(gps_on + 9_000_000_000, time.monotonic_ns())
        fix_back = [fix for t, fix, *_ in w.gps1 if t > gps_on + 9_000_000_000]
        r.check(bool(back) and max(back) < 1.5 and bool(fix_back) and min(fix_back) >= 3,
                f"GPS 1 back at WP3: taken again within 10 s ({summary(back)})")

        # Both gone at WP4: nothing holds the position, so the EKF failsafe must land the drone.
        sim.pump(60, until=lambda last: w.mission_seq >= 5 and w.truth
                 and metres_between(*place(*WP4), w.truth[-1][1], w.truth[-1][2]) < 10.0)
        control(ctl, pause=True)
        set_gps1(sim, False)
        both_since = sim.sim_s
        both_from = time.monotonic_ns()
        sim.pump(60, until=lambda last: sim.mode == "LAND")
        landing = time.monotonic_ns()
        sim.pump(3)                               # the text comes after the heartbeat with the new mode
        drift = w.ekf_errors(both_from, landing)
        r.check(sim.mode == "LAND" and sim.saw("EKF Failsafe", both_since),
                f"GPS 1 and the camera both gone: the EKF failsafe lands the drone, "
                f"{(landing - both_from) / 1e9:.0f} s later, the EKF {max(drift, default=0):.0f} m off by then "
                f"({' | '.join(sim.texts_with('EKF', both_since)[:2])})")
        sim.pump(150, until=lambda last: not sim.armed)
        r.check(not sim.armed, "landed and disarmed")

        r.check(w.forwarded == 0, f"the sender's port is private: none of its messages reached this port "
                                  f"({w.forwarded} did)")
        r.check(proc.poll() is None, "the sender still runs")
    finally:
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
        log_file.close()
        sim.close()

    text = log_path.read_text(encoding="utf-8", errors="replace")
    rates = [int(n) / float(s) for n, s in re.findall(r"sent (\d+) in ([\d.]+) s", text)]
    r.check(len(rates) >= 10 and all(9.5 <= x <= 10.5 for x in rates),
            f"ten messages a second throughout ({min(rates, default=0):.1f} to {max(rates, default=0):.1f}, "
            f"{len(rates)} periods)")
    r.check("GPS settings right" in text, "the sender read the GPS settings and found them right")
    problems = [line for line in text.splitlines() if " ERROR " in line or "Traceback" in line]
    r.check(not problems, f"no errors in the sender's log {problems[:2]}")
    if r.failed:
        r.note(f"kept for a look: {work}")
    else:
        shutil.rmtree(work, ignore_errors=True)


CASES = [mission_without_gps]


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
