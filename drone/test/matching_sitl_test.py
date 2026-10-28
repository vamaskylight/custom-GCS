"""Try the picture matching (GPS-denied navigation, step 3b) against the ArduCopter simulator.

vama_nav runs as its own program, as it will on the Raspberry Pi, on a telemetry port of the
simulated flight controller. It has no camera here: its stand-in camera draws what a camera under
the drone would see at the simulator's true position and angles, from aerial photos of 2022. The
drone's map is the same place photographed in 2020, so the pictures and the map differ as the
real ground and a map do. vama_nav matches each picture with the map and sends the position as GPS 2.

The drone flies a mission at 300 m over east Davis, California (the test photos, see
fetch_test_map.py). Its real GPS is switched off for two legs of 800 m, during which the ground
shows nothing for a few seconds (fog, water), and on again.

Usage:
    py fetch_test_map.py                 once: the test photos (about 10 MB, outside the repo)
    py matching_sitl_test.py             4.6.2 and 4.7.0
    py matching_sitl_test.py 4.7.0       one version

The simulator runs in real time (speedup 1), like gps_input_sitl_test.py: about 9 minutes a version.

Ports:
    5760  this test: it flies the drone and reads the simulator's true position (SIM_STATE)
    5763  vama_nav, as on a telemetry port of the Cube, made private
"""
from __future__ import annotations

import bisect
import csv
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

from fetch_test_map import path_of  # noqa: E402
from gps_input_sitl_test import NAV_PARAMS, PRIVATE, Report, Watch, control, metres_between, set_gps1, summary  # noqa: E402
from sitl_session import Sitl  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
PACKAGE_DIR = HERE.parent / "gps_denied"
VERSIONS = ["4.6.2", "4.7.0"]
SENDER_PORT = 5763
HOME = (38.5375, -121.7135)                # east Davis: the edge of the town and the orchards
HOME_ALT_M = 15.0
ALT_M = 300.0
LEG_M = 800.0
SPEED_MPS = 12.0                           # the client's speed
MAP, GROUND = "davis-2020-z17", "davis-2022-z18"
BLANK_S = 4.0
FLY_PARAMS = {"WPNAV_SPEED": SPEED_MPS * 100, "WPNAV_SPEED_UP": 500, "WPNAV_SPEED_DN": 500, "FENCE_ENABLE": 0}
# (command, (param 1 to 4), metres north and east of home, height). Take-off and RTL have no place.
MISSION = [
    (16, (0, 0, 0, 0), 0.0, 0.0, 0.0),                 # 0: home (the drone keeps its own)
    (22, (0, 0, 0, 0), None, None, ALT_M),             # 1: take off to 300 m
    (16, (0, 0, 0, 0), LEG_M, 0.0, ALT_M),             # 2: WP1, north through the town
    (16, (0, 0, 0, 0), LEG_M, -LEG_M, ALT_M),          # 3: WP2, west
    (16, (0, 0, 0, 0), 0.0, -LEG_M, ALT_M),            # 4: WP3, south
    (16, (0, 0, 0, 0), 0.0, 0.0, ALT_M),               # 5: WP4, east, back over home
    (20, (0, 0, 0, 0), None, None, 0.0),               # 6: RTL
]


def place(north_m: float, east_m: float) -> tuple[float, float]:
    return (HOME[0] + north_m / 111_320.0,
            HOME[1] + east_m / (111_320.0 * math.cos(math.radians(HOME[0]))))


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


class Truth:
    """The simulator's true position and speed (SIM_STATE), by this computer's clock."""

    def __init__(self, sim: Sitl) -> None:
        self.samples: list[tuple] = []      # (arrival_ns, lat, lon, vn, ve)
        take = sim._take

        def taking(msg) -> None:
            if msg.get_type() == "SIM_STATE":
                self.samples.append((time.monotonic_ns(), msg.lat_int / 1e7, msg.lon_int / 1e7, float(msg.vn),
                                     float(msg.ve)))
            take(msg)

        sim._take = taking

    def at(self, t_ns: int) -> tuple | None:
        i = bisect.bisect_left(self.samples, t_ns, key=lambda s: s[0])
        if i == 0 or i >= len(self.samples):
            return None
        a, b = self.samples[i - 1], self.samples[i]
        if b[0] - a[0] > 300_000_000:
            return None
        f = (t_ns - a[0]) / (b[0] - a[0])
        return tuple(x + f * (y - x) for x, y in zip(a[1:], b[1:], strict=True))


def judged(results: pathlib.Path, truth: Truth, since_ns: int, until_ns: int) -> dict[str, list[float]]:
    """From vama_nav's results file: how far each matched position (at its picture's moment) and each
    sent fix (position and speed, at the fix's moment) were from the truth."""
    out: dict[str, list[float]] = {"matched": [], "sent": [], "speed": [], "speed_along": [], "speed_across": []}
    with results.open(encoding="utf-8") as f:
        for row in csv.DictReader(f):
            t = int(row["local_ns"])
            if not since_ns <= t <= until_ns:
                continue
            if row["ok"] == "1" and (at := truth.at(t)):
                out["matched"].append(metres_between(at[0], at[1], float(row["lat"]), float(row["lon"])))
            if row["sent"] == "1" and (at := truth.at(int(row["fix_local_ns"]))):
                out["sent"].append(metres_between(at[0], at[1], float(row["fix_lat"]), float(row["fix_lon"])))
                dn, de = float(row["fix_vn"]) - at[2], float(row["fix_ve"]) - at[3]
                out["speed"].append(math.hypot(dn, de))
                speed = math.hypot(at[2], at[3])
                if speed > 3:
                    ux, uy = at[2] / speed, at[3] / speed
                    out["speed_along"].append(dn * ux + de * uy)
                    out["speed_across"].append(-dn * uy + de * ux)
    return out


def spread(values: list[float]) -> str:
    if not values:
        return "no samples"
    return (f"middle {statistics.median(values):.2f}, worst {max(values, key=abs):.2f}, "
            f"{len(values)} samples")


def mission_on_matched_positions(version: str, r: Report) -> None:
    """GPS 2 is GPS 1 passed on while disarmed; above 50 m it carries matched positions, near the true position while GPS 1 works; with GPS 1 off for two legs of 800 m at 300 m and 12 m/s the mission flies on them, through a few seconds of blank ground; GPS 1 back is taken again; the drone lands; the port stays private and the messages keep their rate."""
    missing = [name for name in (MAP, GROUND) if not path_of(name).is_file()]
    if missing:
        r.check(False, f"the test photos are not there ({', '.join(missing)}): run py fetch_test_map.py once")
        return
    work = pathlib.Path(tempfile.mkdtemp(prefix="vama-match-A-"))
    ctl = work / "control.json"
    control(ctl)
    params = {"RC_OVERRIDE_TIME": -1, "SIM_WIND_SPD": 0, **NAV_PARAMS, **PRIVATE[version[:3]], **FLY_PARAMS}
    sim = Sitl(version, {}, params, speedup=1, home=f"{HOME[0]},{HOME[1]},{HOME_ALT_M:g},0")
    proc = None
    log_path = work / "nav.log"
    results = work / "results.csv"
    flights = work / "flights"
    ini = work / "recorder.ini"
    ini.write_text("\n".join(["[recording]", f"folder = {flights}", "start_height_m = 30", "min_free_mb = 100", ""]),
                   encoding="utf-8")
    log_file = log_path.open("w", encoding="utf-8")
    w = Watch(sim)
    truth = Truth(sim)
    try:
        sim.mav.mav.command_long_send(sim.mav.target_system, sim.mav.target_component,
                                      mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 108, 100_000, 0, 0, 0, 0, 0)
        sim.pump(120, until=lambda last: "EKF_STATUS_REPORT" in last
                 and (last["EKF_STATUS_REPORT"].flags & 0x18) == 0x18
                 and "GPS_RAW_INT" in last and last["GPS_RAW_INT"].fix_type >= 3)

        env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
        proc = subprocess.Popen([sys.executable, "-m", "vama_nav", "--map", str(path_of(MAP)), "--sim-camera",
                                 str(path_of(GROUND)), "--device", f"tcp:127.0.0.1:{SENDER_PORT}", "--control", str(ctl),
                                 "--seed", "7", "--results", str(results), "--record", "--config", str(ini)],
                                cwd=PACKAGE_DIR, env=env, stdout=log_file, stderr=subprocess.STDOUT)
        started = time.monotonic_ns()
        sim.pump(30, until=lambda last: any(t > started and fix >= 3 for t, fix, *_ in w.gps2[-3:]))
        r.check(any(t > started and fix >= 3 for t, fix, *_ in w.gps2),
                "disarmed, GPS 2 has a 3D fix: vama_nav passes GPS 1 on")

        r.check(upload(sim, MISSION) == 0, "mission accepted (take-off to 300 m, 4 legs of 800 m, RTL)")
        sim.arm_and_takeoff(8)
        r.check(sim.armed, "the drone arms with GPS 2 set up and vama_nav running")
        sim.rc(rc3=1500)
        sim.set_mode("AUTO")
        sim.pump(150, until=lambda last: w.mission_seq >= 2 and w.height() > ALT_M - 5)
        r.check(w.mission_seq >= 2 and w.height() > ALT_M - 5, f"climbed to {w.height():.0f} m in AUTO")

        # GPS 1 works on the way to WP1: the matched positions reach GPS 2, and how near they are.
        sim.pump(120, until=lambda last: w.mission_seq >= 3)
        r.check(w.mission_seq >= 3, "WP1 reached (800 m north)")
        on_gps1 = w.errors(w.gps2, started + 60_000_000_000, time.monotonic_ns(), fixes=(3,))
        r.check(len(on_gps1) >= 30 and statistics.median(on_gps1) < 5.0 and max(on_gps1) < 15.0,
                f"while GPS 1 works, the matched positions (3D on GPS 2) are near the true position "
                f"({summary(on_gps1)})")
        j = judged(results, truth, started + 60_000_000_000, time.monotonic_ns())
        r.note(f"GPS 1 on, matched at the picture's moment: {spread(j['matched'])} m; sent fix: {spread(j['sent'])} m")
        r.note(f"GPS 1 on, sent speed minus true: {spread(j['speed'])} m/s, along the track "
               f"{spread(j['speed_along'])}, across {spread(j['speed_across'])}")

        set_gps1(sim, False)
        gps_off = time.monotonic_ns()
        off_since = sim.sim_s
        sim.pump(150, until=lambda last: w.mission_seq >= 4 or not sim.armed)
        r.check(w.mission_seq >= 4, f"GPS 1 off at WP1: the drone flew to WP2 on the matched positions "
                                    f"({(time.monotonic_ns() - gps_off) / 1e9:.0f} s)")
        # Halfway to WP3 the ground shows nothing for a few seconds: no position, then positions again.
        sim.pump(40, until=lambda last: w.truth and metres_between(*place(LEG_M / 2, -LEG_M),
                                                                   w.truth[-1][1], w.truth[-1][2]) < 30)
        control(ctl, blank=True)
        blank_from = time.monotonic_ns()
        sim.pump(BLANK_S)
        control(ctl)
        blank_to = time.monotonic_ns()
        sim.pump(120, until=lambda last: w.mission_seq >= 5 or not sim.armed)
        wp3 = time.monotonic_ns()
        r.check(w.mission_seq >= 5, f"... and on to WP3 ({(wp3 - gps_off) / 1e9:.0f} s with GPS 1 off, "
                                    f"mission item {w.mission_seq})")
        lost = [fix for t, fix, *_ in w.gps1 if gps_off + 6_000_000_000 < t < wp3]
        r.check(bool(lost) and max(lost) < 3, f"GPS 1 had no fix there (fix types {sorted(set(lost))})")
        modes = [mode for t, mode in sim.mode_log if t >= off_since]
        r.check(sim.mode == "AUTO" and not modes, f"still AUTO, no mode change ({modes})")
        on_camera = w.ekf_errors(gps_off, wp3)
        r.check(bool(on_camera) and statistics.median(on_camera) < 5.0 and max(on_camera) < 15.0,
                f"the EKF stayed near the true position ({summary(on_camera)})")
        fixes = [t for t, fix, *_ in w.gps2 if gps_off < t < wp3 and fix == 3]
        gaps = [(b - a) / 1e9 for a, b in zip(fixes, fixes[1:], strict=False)]
        blank_gap = max((g for a, g in zip(fixes, gaps, strict=False) if blank_from - 3e9 < a < blank_to), default=0)
        r.check(len(fixes) >= 0.7 * (wp3 - gps_off) / 1e9,
                f"a matched position for {len(fixes)} of {(wp3 - gps_off) / 1e9:.0f} seconds")
        # The stand-in reads its control file once a second: the blank lasts BLANK_S, give or take 1 s.
        r.check(BLANK_S - 1.5 <= blank_gap < BLANK_S + 4,
                f"no position while the ground showed nothing for about {BLANK_S:g} s: no 3D fix for "
                f"{blank_gap:.1f} s")
        after_blank = w.ekf_errors(blank_to + 5_000_000_000, wp3)
        r.check(bool(after_blank) and max(after_blank) < 10.0, f"the EKF near the true position again after it "
                                                               f"({summary(after_blank)})")
        bad = [t for t in sim.texts_with("", off_since) if re.search(r"glitch|variance|failsafe|yaw reset", t, re.I)]
        r.check(not bad, f"no GPS glitch, EKF variance or failsafe message {bad[:3]}")
        tests = [(v, p) for t, v, p in w.ekf_status if gps_off < t < wp3]
        if tests:
            r.note(f"EKF test ratios while on the camera: velocity up to {max(v for v, _p in tests):.2f}, "
                   f"position up to {max(p for _v, p in tests):.2f} (1 is the gate)")
        gps2_off = w.errors(w.gps2, gps_off, wp3, fixes=(3,))
        r.note(f"the matched positions (3D on GPS 2) with GPS 1 off: {summary(gps2_off)}")
        j = judged(results, truth, gps_off, wp3)
        r.note(f"GPS 1 off, matched at the picture's moment: {spread(j['matched'])} m; sent fix: {spread(j['sent'])} m")
        r.note(f"GPS 1 off, sent speed minus true: {spread(j['speed'])} m/s, along the track "
               f"{spread(j['speed_along'])}, across {spread(j['speed_across'])}")

        set_gps1(sim, True)
        gps_on = time.monotonic_ns()
        sim.pump(12)
        back = w.ekf_errors(gps_on + 9_000_000_000, time.monotonic_ns())
        r.check(bool(back) and max(back) < 2.0, f"GPS 1 back at WP3: taken again ({summary(back)})")

        sim.pump(400, until=lambda last: not sim.armed)
        r.check(not sim.armed, f"back home, landed and disarmed (mode {sim.mode})")
        r.check(w.forwarded == 0, f"vama_nav's port is private: none of its messages reached this port "
                                  f"({w.forwarded} did)")
        r.check(proc.poll() is None, "vama_nav still runs")
    finally:
        # The tracks, for a look afterwards: the truth and the EKF, GPS 2 as ArduPilot saw it.
        with (work / "tracks.csv").open("w", newline="", encoding="utf-8") as f:
            rows = csv.writer(f)
            rows.writerow(["kind", "t_ns", "lat", "lon", "fix"])
            rows.writerows(("truth", t, lat, lon, "") for t, lat, lon, *_ in truth.samples)
            rows.writerows(("ekf", t, lat, lon, "") for t, lat, lon, _h in w.ekf)
            rows.writerows(("gps2", t, lat, lon, fix) for t, fix, lat, lon in w.gps2)
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
    r.check(len(rates) >= 20 and all(9.5 <= x <= 10.5 for x in rates),
            f"ten messages a second throughout ({min(rates, default=0):.1f} to {max(rates, default=0):.1f}, "
            f"{len(rates)} periods)")
    r.check("GPS settings right" in text, "vama_nav read the GPS settings and found them right")
    took = [float(x) for x in re.findall(r"matching ([\d.]+) s a picture", text)]
    if took:
        r.note(f"matching took {statistics.median(took):.2f} s a picture (middle of the status lines) on this PC")
    summary_line = [line for line in text.splitlines() if "no position because" in line]
    if summary_line:
        r.note(summary_line[-1].split(" INFO ", 1)[-1])
    problems = [line for line in text.splitlines() if " ERROR " in line or "Traceback" in line]
    r.check(not problems, f"no errors in vama_nav's log {problems[:2]}")

    # Recorded while matching, then step 2 on that flight: the pictures matched again on this PC.
    folders = sorted(flights.iterdir()) if flights.is_dir() else []
    rows = 0
    if folders:
        with (folders[0] / "frames.csv").open(encoding="utf-8") as f:
            rows = sum(1 for _ in csv.DictReader(f))
    r.check(len(folders) == 1 and rows >= 300, f"vama_nav --record kept one flight folder with {rows} pictures")
    if folders:
        # From 150 m: on the way up and down the camera sees little of the map (300 m is the flight's height).
        replay = subprocess.run([sys.executable, "-m", "vama_nav.replay", str(folders[0]), "--map", str(path_of(MAP)),
                                 "--match-above-m", "150"],
                                cwd=PACKAGE_DIR, capture_output=True, text=True, timeout=900,
                                env=dict(os.environ, PYTHONDONTWRITEBYTECODE="1"))
        found = re.search(r"high enough to try (\d+), a position from (\d+)", replay.stdout)
        tried, matched = (int(found.group(1)), int(found.group(2))) if found else (0, 0)
        r.check(replay.returncode == 0 and tried > 0 and matched >= 0.85 * tried,
                f"step 2 on that recorded flight: a position from {matched} of {tried} pictures from 150 m")
        for line in replay.stdout.splitlines():
            if "position error" in line or "speed between" in line or "seems" in line:
                r.note("step 2: " + line.strip())
    if r.failed:
        r.note(f"kept for a look: {work}")
    else:
        shutil.rmtree(work, ignore_errors=True)


CASES = [mission_on_matched_positions]


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
