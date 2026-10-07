"""Fly the real VGCS link code against the real ArduCopter simulator (milestone M17, "full SITL testing").

Usage, from the repo root:

    py system_test/vgcs_sitl_test.py                  every case on ArduCopter 4.6.2 and 4.7.0
    py system_test/vgcs_sitl_test.py 4.7.0            one version
    py system_test/vgcs_sitl_test.py -k mission       only the cases with "mission" in the name
    py system_test/vgcs_sitl_test.py --report out.md  also write the results as a Markdown table

What is real here: the ArduCopter program itself (the same code that flies the
drone, with a simulated airframe and sensors), and VGCS's own MavlinkThread,
which is the only thing in VGCS that talks to a drone. Every command below goes
through the same queue_... call that the buttons use.

What checks the result: a second, independent MAVLink connection to the
simulator (drone/test/sitl_session.py). VGCS saying "done" is never taken as
proof. The simulator has to show it.

Set-up: see system_test/README.md (WSL and the simulator programs, as for
drone/test).
"""

from __future__ import annotations

import math
import os
import pathlib
import re
import sys
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("VGCS_NO_NETWORK", "1")

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "drone" / "test"))

from PySide6.QtCore import Qt  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

from sitl_session import GPS_OFF, Sitl, wsl  # noqa: E402
from vgcs.app.arm_readiness import parse_prearm_health  # noqa: E402
from vgcs.link.mavlink_thread import MavlinkThread  # noqa: E402

VERSIONS = ["4.6.2", "4.7.0"]
HOME = (20.4347, 72.8696)                 # where the simulated drone stands (sitl_session default)
VGCS_PORT = "tcp:127.0.0.1:5762"          # the simulator's second telemetry port
BASE = {"RC_OVERRIDE_TIME": -1, "SIM_WIND_SPD": 0}

# Parameter names that changed between the two versions. VGCS has to work with both.
SPEED_PARAM = {"4.6": ("WPNAV_SPEED", 500.0), "4.7": ("WP_SPD", 5.0)}          # 5 m/s


def offset_to_lat_lon(north_m: float, east_m: float) -> tuple[float, float]:
    lat = HOME[0] + north_m / 111_320.0
    lon = HOME[1] + east_m / (111_320.0 * math.cos(math.radians(HOME[0])))
    return lat, lon


def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    north = (lat2 - lat1) * 111_320.0
    east = (lon2 - lon1) * 111_320.0 * math.cos(math.radians(lat1))
    return math.hypot(north, east)


class Report:
    def __init__(self, name: str, doc: str) -> None:
        self.name = name
        self.doc = doc
        self.lines: list[tuple[bool, str]] = []

    def check(self, ok: bool, text: str) -> bool:
        self.lines.append((bool(ok), text))
        return bool(ok)

    @property
    def failed(self) -> int:
        return sum(1 for ok, _ in self.lines if not ok)


class Recorder:
    """Everything VGCS's link thread says, with the time it said it.

    Connected directly, so the notes are taken in the link's own thread and
    this test needs no Qt event loop.
    """

    def __init__(self, link: MavlinkThread) -> None:
        self.t0 = time.monotonic()
        self.telemetry: dict[str, list[tuple[float, dict]]] = {}
        self.actions: list[tuple[float, str, bool, str]] = []
        self.modes: list[tuple[float, str, bool]] = []
        self.progress: list[tuple[float, dict]] = []
        self.logs: list[tuple[float, str]] = []
        self.errors: list[tuple[float, str]] = []
        self.uploaded: list[int] = []
        self.downloaded: list[list] = []
        self.params: list[dict] = []
        self.param_sets: list[tuple[str, bool, str]] = []
        self.fences: list[tuple[bool, str]] = []
        self.link_events: list[tuple[float, str]] = []
        direct = Qt.ConnectionType.DirectConnection
        link.telemetry.connect(lambda kind, payload: self.telemetry.setdefault(kind, []).append((self.now(), dict(payload))), direct)
        link.action_result.connect(lambda action, ok, detail: self.actions.append((self.now(), action, bool(ok), detail)), direct)
        link.mode_changed.connect(lambda mode, ok: self.modes.append((self.now(), mode, bool(ok))), direct)
        link.mission_progress.connect(lambda payload: self.progress.append((self.now(), dict(payload))), direct)
        link.log_line.connect(lambda line: self.logs.append((self.now(), line)), direct)
        link.error.connect(lambda line: self.errors.append((self.now(), line)), direct)
        link.mission_uploaded.connect(lambda count: self.uploaded.append(int(count)), direct)
        link.mission_downloaded.connect(lambda items: self.downloaded.append(list(items)), direct)
        link.params_snapshot.connect(lambda values: self.params.append(dict(values)), direct)
        link.param_set_result.connect(lambda name, ok, detail: self.param_sets.append((name, bool(ok), detail)), direct)
        link.geofence_result.connect(lambda ok, detail: self.fences.append((bool(ok), detail)), direct)
        link.link_up.connect(lambda: self.link_events.append((self.now(), "up")), direct)
        link.link_down.connect(lambda: self.link_events.append((self.now(), "down")), direct)
        link.link_timeout.connect(lambda seconds: self.link_events.append((self.now(), f"timeout {seconds:.1f}")), direct)
        link.heartbeat.connect(lambda *_a: self.link_events.append((self.now(), "heartbeat")), direct)

    def now(self) -> float:
        return time.monotonic() - self.t0

    def last(self, kind: str) -> dict:
        rows = self.telemetry.get(kind) or []
        return rows[-1][1] if rows else {}

    def action(self, name: str, since: float = 0.0):
        """The newest result of an action after `since`: (ok, detail), or None."""
        for when, action, ok, detail in reversed(self.actions):
            if action == name and when >= since:
                return ok, detail
        return None

    def mode_text(self) -> str:
        return str(self.last("HEARTBEAT").get("mode_text", ""))

    def texts(self, since: float = 0.0) -> list[str]:
        return [p.get("text", "") for when, p in self.telemetry.get("STATUSTEXT", []) if when >= since]


class Flight:
    """One simulated drone with VGCS's link connected to it."""

    def __init__(self, version: str, params: dict | None = None, speedup: int = 10) -> None:
        self.version = version
        self.sim = Sitl(version, {}, dict(BASE, **(params or {})), speedup=speedup)
        self.link = MavlinkThread(VGCS_PORT)
        self.rec = Recorder(self.link)
        self.link.start()

    def wait(self, condition, sim_seconds: float) -> bool:
        """Let the simulated clock run until the condition holds. False when it never did."""
        if condition():
            return True
        return self.sim.fly(sim_seconds, until=condition)

    def wait_real(self, condition, seconds: float) -> bool:
        """Wait in real seconds. For VGCS's own timers (retries, giving up), which
        run on the PC's clock, not the simulator's ten times faster one."""
        if condition():
            return True
        self.sim.pump(seconds, until=lambda _last: condition())
        return bool(condition())

    def ready_to_fly(self) -> bool:
        """Wait until the simulated drone has its position (EKF and GPS), as before any real flight."""
        return self.wait(lambda: (self.sim.ekf_flags & 0x18) == 0x18 and self.rec.last("GPS_RAW_INT").get("fix_type", 0) >= 3, 120)

    def close(self) -> None:
        try:
            self.link.stop()
            self.link.wait(4000)
        finally:
            self.sim.close()


def fly_at(sim: Sitl, north_mps: float, east_mps: float, up_mps: float) -> None:
    """In GUIDED: fly at this speed. Sent again every second, it times out after three."""
    sim.mav.mav.set_position_target_local_ned_send(
        0, sim.mav.target_system, sim.mav.target_component, 1,   # MAV_FRAME_LOCAL_NED
        0b0000_1101_1100_0111, 0, 0, 0, north_mps, east_mps, -up_mps, 0, 0, 0, 0, 0)


def square_mission(side_m: float = 60.0, alt_m: float = 20.0) -> list[dict]:
    """Four waypoints: north, north-east, east, and back over home."""
    out = []
    for north, east in ((side_m, 0.0), (side_m, side_m), (0.0, side_m), (0.0, 0.0)):
        lat, lon = offset_to_lat_lon(north, east)
        out.append({"lat": lat, "lon": lon, "alt_m": alt_m, "speed_mps": 5.0})
    return out


# --- the cases -------------------------------------------------------------

def connect_and_telemetry(version: str, r: Report) -> None:
    """VGCS connects, asks for its data, and shows what the drone really reports."""
    f = Flight(version)
    try:
        rec, sim = f.rec, f.sim
        r.check(f.wait(lambda: any(e[1] == "heartbeat" for e in rec.link_events), 30), "VGCS saw the drone's heartbeat")
        up = [e for e in rec.link_events if e[1] == "up"]
        r.check(bool(up), "the link reported itself up")

        # Before the drone has its position, the autopilot refuses to arm. VGCS must show that, not "ready".
        sys_status = rec.last("SYS_STATUS")
        first = parse_prearm_health(sensors_present=sys_status.get("sensors_present", 0), sensors_enabled=sys_status.get("sensors_enabled", 0),
                                    sensors_health=sys_status.get("sensors_health", 0), now=time.monotonic()) if sys_status else None
        early_verdict = None if first is None else (first.reported, first.passing)

        r.check(f.ready_to_fly(), "the drone got its position (EKF and GPS ready)")
        f.wait(lambda: False, 5)
        wanted = ["HEARTBEAT", "GLOBAL_POSITION_INT", "ATTITUDE", "VFR_HUD", "SYS_STATUS", "GPS_RAW_INT", "BATTERY_STATUS"]
        missing = [kind for kind in wanted if not rec.telemetry.get(kind)]
        r.check(not missing, f"every kind of data VGCS shows arrives ({', '.join(wanted)})" + (f". Missing: {missing}" if missing else ""))

        gpi = rec.last("GLOBAL_POSITION_INT")
        off = distance_m(HOME[0], HOME[1], gpi.get("lat", 0.0), gpi.get("lon", 0.0))
        r.check(off < 3.0, f"the position VGCS shows is where the drone stands ({off:.1f} m off)")
        r.check(abs(gpi.get("relative_alt_m", 99.0)) < 1.0, f"height above home reads {gpi.get('relative_alt_m', 99.0):.2f} m on the ground")
        gps = rec.last("GPS_RAW_INT")
        r.check(gps.get("fix_type", 0) >= 3 and gps.get("satellites_visible", 0) >= 6, f"GPS reads fix {gps.get('fix_type')} with {gps.get('satellites_visible')} satellites")
        status = rec.last("SYS_STATUS")
        r.check(10.0 < status.get("voltage_v", 0.0) < 14.0, f"battery reads {status.get('voltage_v', 0.0):.2f} V (the simulated pack is 12.6 V)")
        att = rec.last("ATTITUDE")
        r.check(abs(att.get("roll_deg", 99)) < 2 and abs(att.get("pitch_deg", 99)) < 2, "the drone reads level on the ground")

        now = parse_prearm_health(sensors_present=status.get("sensors_present", 0), sensors_enabled=status.get("sensors_enabled", 0),
                                  sensors_health=status.get("sensors_health", 0), now=time.monotonic())
        r.check(now.reported and now.passing, "the autopilot's own verdict reads ready to arm")
        if early_verdict is not None and early_verdict[0]:
            r.check(early_verdict[1] is False, "and before it had its position, the same verdict read not ready")
        r.check(not rec.errors, f"VGCS reported no error {[e[1] for e in rec.errors][:3]}")
    finally:
        f.close()


def modes_and_arming_on_the_ground(version: str, r: Report) -> None:
    """Mode changes, arm, disarm and the emergency stop, checked on the simulator."""
    # DISARM_DELAY 0: the autopilot must not disarm by itself on the ground while this is tested.
    f = Flight(version, params={"DISARM_DELAY": 0})
    try:
        rec, sim, link = f.rec, f.sim, f.link
        r.check(f.ready_to_fly(), "the drone is ready")
        for mode in ("GUIDED", "LOITER", "ALT_HOLD", "STABILIZE"):
            since = rec.now()
            link.queue_mode_change(mode)
            took = f.wait(lambda: sim.mode == mode, 15)

            def said() -> list:
                return [m for m in rec.modes if m[0] >= since and m[1] == mode]

            f.wait_real(lambda: bool(said()), 6)
            r.check(took and bool(said()) and said()[-1][2], f"mode {mode}: the drone is in it ({sim.mode}) and VGCS says so")
            if said():
                r.check(said()[-1][0] - since < 3.0, f"mode {mode}: VGCS confirmed it {said()[-1][0] - since:.1f} s after the click")
            r.check(f.wait(lambda: rec.mode_text() == mode, 10), f"mode {mode}: VGCS's own mode display follows ({rec.mode_text()})")

        link.queue_mode_change("GUIDED")
        f.wait(lambda: sim.mode == "GUIDED", 15)
        since = rec.now()
        link.queue_arm(True)
        r.check(f.wait(lambda: sim.armed, 20), "arm: the drone armed")
        r.check(f.wait(lambda: rec.last("HEARTBEAT").get("armed") is True, 10), "arm: VGCS shows armed")
        since = rec.now()
        link.queue_arm(False)
        r.check(f.wait(lambda: not sim.armed, 20), "disarm: the drone disarmed")
        result = None
        f.wait(lambda: rec.action("arm", since) is not None, 10)
        result = rec.action("arm", since)
        r.check(result is not None and result[0], f"disarm: VGCS reports it ({result})")

        link.queue_arm(True)
        r.check(f.wait(lambda: sim.armed, 20), "armed again for the emergency stop")
        since = rec.now()
        link.queue_emergency_motor_stop()
        r.check(f.wait(lambda: not sim.armed, 10), "emergency stop: the motors stopped (disarmed)")
        r.check(not [e for e in rec.errors if "Skipped" not in e[1]], f"VGCS reported no error {[e[1] for e in rec.errors][:3]}")
    finally:
        f.close()


def refusals_say_why(version: str, r: Report) -> None:
    """Without a position the drone refuses take-off on the ground and LOITER in the air. VGCS must say so in the drone's words, and leave the mode as it was."""
    # The EKF and dead-reckoning failsafes would change the mode by themselves
    # when the position goes. Off, so what is tested is VGCS's report.
    f = Flight(version, params={"FS_EKF_THRESH": 0, "FS_DR_ENABLE": 0})
    try:
        rec, sim, link = f.rec, f.sim, f.link
        gps_name, gps_off_value = GPS_OFF[version[:3]]
        # By MAVLink right after start-up, before the GPS has its first fix.
        # The same setting in the start-up file did not stop the simulated
        # GPS on 4.7.0.
        sim.gps_off()
        r.check(f.wait(lambda: bool(rec.telemetry.get("SYS_STATUS")), 30), "VGCS is connected")
        f.wait(lambda: False, 30)
        r.check((sim.ekf_flags & 0x18) == 0, f"the drone has no position (EKF flags {sim.ekf_flags:#x})")
        r.check(rec.last("GPS_RAW_INT").get("fix_type", 0) < 2, f"VGCS shows no GPS fix (fix type {rec.last('GPS_RAW_INT').get('fix_type')})")

        # On the ground ArduCopter takes any mode (it checks at arming), so the
        # refusal to look for here is the arming one.
        mode_before = sim.mode
        since = rec.now()
        link.queue_auto_takeoff(10.0)
        f.wait_real(lambda: rec.action("auto_takeoff", since) is not None, 30)
        result = rec.action("auto_takeoff", since)
        r.check(result is not None and result[0] is False, f"take-off without GPS is reported as refused ({result})")
        r.check(not sim.armed, "and the drone did not arm")
        reason = "" if result is None else result[1]
        r.check("PreArm" in reason or "Arm:" in reason, f"the drone's own reason is quoted: {reason[:110]!r}")
        f.wait(lambda: sim.mode == mode_before, 10)
        r.check(sim.mode == mode_before, f"the mode was put back to {mode_before} after the refused take-off (it is in {sim.mode})")

        # In the air: GPS back, take off, then lose the position in ALT_HOLD.
        sim.set_param(gps_name, 1 - gps_off_value)
        r.check(f.ready_to_fly(), "GPS back on: the drone has its position again")
        link.queue_auto_takeoff(15.0)
        r.check(f.wait(lambda: sim.alt > 14.0, 80), f"flying ({sim.alt:.1f} m)")
        f.wait(lambda: False, 5)
        sim.rc(rc3=1500)                  # throttle stick in the middle: ALT_HOLD keeps its height
        link.queue_mode_change("ALT_HOLD")
        r.check(f.wait(lambda: sim.mode == "ALT_HOLD", 15), f"VGCS switched it to ALT_HOLD ({sim.mode})")
        sim.gps_off()
        r.check(f.wait(lambda: (sim.ekf_flags & 0x18) == 0, 120), f"in the air the drone lost its position (EKF flags {sim.ekf_flags:#x})")
        since = rec.now()
        link.queue_mode_change("LOITER")
        f.wait_real(lambda: any(m[0] >= since and m[1] == "LOITER" for m in rec.modes), 8)
        said = [m for m in rec.modes if m[0] >= since and m[1] == "LOITER"]
        r.check(sim.mode == "ALT_HOLD", f"the drone refused LOITER without a position (it is in {sim.mode})")
        r.check(bool(said) and said[-1][2] is False, f"VGCS does not report the refused mode change as done ({said[-1][2] if said else 'nothing said'})")
        why = [e[1] for e in rec.errors if e[0] >= since and "LOITER" in e[1]]
        r.check(bool(why) and "position" in why[-1].lower(), f"VGCS gives the drone's reason: {why[-1] if why else 'nothing'!r}")

        since = rec.now()
        link.queue_auto_land()
        r.check(f.wait(lambda: sim.mode == "LAND", 15), f"land without GPS: the drone is landing ({sim.mode})")
        f.wait_real(lambda: rec.action("auto_land", since) is not None, 6)
        result = rec.action("auto_land", since)
        r.check(result is not None and result[0], f"land: VGCS reports it ({result})")
        r.check(f.wait(lambda: not sim.armed, 150), "land: it landed and disarmed")
    finally:
        f.close()


def parameters(version: str, r: Report) -> None:
    """The settings VGCS reads and writes exist on this firmware, and a write is confirmed by the drone."""
    f = Flight(version)
    try:
        rec, sim, link = f.rec, f.sim, f.link
        r.check(f.ready_to_fly(), "the drone is ready")
        # What VGCS reads on connect and on "Refresh params": both firmware's names.
        from vgcs.app.vehicle_params import EDITABLE, READ_ON_CONNECT, RENAMED_IN_4_7, editable_on_this_drone

        names = list(READ_ON_CONNECT)
        t0 = time.monotonic()
        link.queue_params_fetch(names)
        f.wait_real(lambda: bool(rec.params), 15)
        took = time.monotonic() - t0
        got = rec.params[-1] if rec.params else {}
        for old, new in RENAMED_IN_4_7.items():
            has = [n for n in (old, new) if n in got]
            r.check(len(has) == 1, f"{old} / {new}: the drone has exactly one of the two names ({has})")
        must_have = [n for n in names if n not in RENAMED_IN_4_7 and n not in RENAMED_IN_4_7.values()]
        missing = [n for n in must_have if n not in got]
        r.check(not missing, f"every other setting VGCS reads is on this firmware ({len(must_have) - len(missing)} of {len(must_have)})"
                + (f". Missing: {missing}" if missing else ""))
        r.check(took < 8.0, f"the read finished in {took:.1f} s (the names it lacks cost one retry, not a second each)")
        wrong = []
        for name, value in got.items():
            real = sim.get_param(name)
            if real is None or abs(real - value) > 1e-3:
                wrong.append(f"{name}: VGCS read {value}, the drone has {real}")
        r.check(not wrong, f"every value VGCS read is the drone's own ({len(got)} values)" + (f": {wrong[:3]}" if wrong else ""))
        listed = editable_on_this_drone(got)
        r.check(all(n in got for n in listed) and len(listed) >= len(EDITABLE) - 3,
                f"the 'Set param' list shows only names this drone has ({len(listed)}: {', '.join(listed[:4])}, ...)")
        positions = len(rec.telemetry.get("GLOBAL_POSITION_INT", []))
        r.check(positions > 0, "positions kept arriving while the settings were read")

        speed_name, speed_value = SPEED_PARAM[version[:3]]
        before = sim.get_param(speed_name)
        r.check(before is not None, f"this firmware's speed setting is {speed_name} ({before})")
        new_value = (before or speed_value) * 0.8
        link.queue_param_set(speed_name, new_value)
        f.wait_real(lambda: any(p[0] == speed_name for p in rec.param_sets), 6)
        after = sim.get_param(speed_name)
        r.check(after is not None and abs(after - new_value) < 1e-2, f"a setting written by VGCS is on the drone ({speed_name} = {after})")
        said = [p for p in rec.param_sets if p[0] == speed_name]
        r.check(bool(said) and said[-1][1], f"and VGCS reports it as written, with the value the drone echoed ({said[-1][1:] if said else 'nothing said'})")

        link.queue_param_set("NOT_A_SETTING", 3.0)
        f.wait_real(lambda: any(p[0] == "NOT_A_SETTING" for p in rec.param_sets), 8)
        said = [p for p in rec.param_sets if p[0] == "NOT_A_SETTING"]
        r.check(bool(said) and said[-1][1] is False, f"a setting the drone does not have is reported as not written ({said[-1][1:] if said else 'nothing said'})")

        old_name = {"4.6": "WP_SPD", "4.7": "WPNAV_SPEED"}[version[:3]]
        link.queue_param_set(old_name, 400.0)
        f.wait_real(lambda: any(p[0] == old_name for p in rec.param_sets), 8)
        said = [p for p in rec.param_sets if p[0] == old_name]
        r.check(bool(said) and said[-1][1] is False, f"the other firmware's name ({old_name}) is reported as not written ({said[-1][1] if said else 'nothing said'})")
    finally:
        f.close()


def mission_upload_and_download(version: str, r: Report) -> None:
    """A mission goes to the drone and comes back the same."""
    f = Flight(version)
    try:
        rec, link = f.rec, f.link
        r.check(f.ready_to_fly(), "the drone is ready")
        mission = square_mission()
        link.queue_mission_upload(mission, "rtl")
        r.check(f.wait(lambda: bool(rec.uploaded), 60), f"upload: VGCS reports {rec.uploaded[-1] if rec.uploaded else 0} waypoints sent")
        link.queue_mission_download()
        r.check(f.wait(lambda: bool(rec.downloaded), 60), "download: the mission came back")
        items = rec.downloaded[-1] if rec.downloaded else []
        waypoints = [i for i in items if i.get("command") == 16 and i.get("seq", 0) > 0]
        r.check(len(waypoints) == len(mission), f"the drone holds {len(waypoints)} waypoints (sent {len(mission)})")
        worst = 0.0
        for sent, back in zip(mission, waypoints):
            worst = max(worst, distance_m(sent["lat"], sent["lon"], back.get("lat", 0.0), back.get("lon", 0.0)))
            if abs(back.get("alt_m", -1) - sent["alt_m"]) > 0.01:
                r.check(False, f"waypoint height came back as {back.get('alt_m')} m, sent {sent['alt_m']} m")
        r.check(worst < 0.05, f"every waypoint came back in the same place (largest difference {worst * 100:.1f} cm)")
        commands = [i.get("command") for i in items]
        r.check(22 in commands, "the mission starts with a take-off item")
        r.check(commands[-1] == 20, f"the mission ends with return to launch, as chosen (last item {commands[-1]})")
        r.check(not rec.errors, f"VGCS reported no error {[e[1] for e in rec.errors][:3]}")
    finally:
        f.close()


def mission_flight(version: str, r: Report) -> None:
    """A whole mission: start from the ground, pause, resume, skip a waypoint, return home and land."""
    f = Flight(version)
    try:
        rec, sim, link = f.rec, f.sim, f.link
        r.check(f.ready_to_fly(), "the drone is ready")
        mission = square_mission()
        link.queue_mission_upload(mission, "rtl")
        r.check(f.wait(lambda: bool(rec.uploaded), 60), "the mission is on the drone")
        since = rec.now()
        link.queue_mission_start()
        r.check(f.wait(lambda: sim.armed and sim.mode == "AUTO", 40), f"mission start: armed and in AUTO ({sim.mode})")
        f.wait_real(lambda: rec.action("mission_start", since) is not None, 6)
        result = rec.action("mission_start", since)
        r.check(result is not None and result[0], f"VGCS reports the start once the drone is in AUTO ({result})")
        r.check(f.wait(lambda: sim.alt > 18.0, 60), f"it took off to the mission height ({sim.alt:.1f} m)")

        def reached() -> list[int]:
            return [p["wp_index"] for _t, p in rec.progress if p.get("reached") and p.get("wp_index") is not None]

        r.check(f.wait(lambda: 0 in reached(), 90), "waypoint 1 reached, and VGCS says so")
        lat, lon = offset_to_lat_lon(sim.north, sim.east)
        off = distance_m(mission[0]["lat"], mission[0]["lon"], lat, lon)
        r.check(off < 5.0, f"the drone really is at waypoint 1 then ({off:.1f} m from it)")

        # Pause on the way to waypoint 2.
        f.wait(lambda: False, 4)
        since = rec.now()
        link.queue_mission_pause()
        r.check(f.wait(lambda: sim.mode in ("BRAKE", "LOITER", "POSHOLD"), 15), f"pause: the drone holds ({sim.mode})")
        f.wait_real(lambda: rec.action("mission_pause", since) is not None, 6)
        result = rec.action("mission_pause", since)
        r.check(result is not None and result[0] and sim.mode in result[1], f"pause: VGCS reports the hold mode the drone is in ({result})")
        f.wait(lambda: False, 8)
        speed = math.hypot(sim.vn, sim.ve)
        r.check(speed < 0.5, f"pause: it stands still ({speed:.2f} m/s)")
        held_at = (sim.north, sim.east)
        since = rec.now()
        link.queue_mission_resume()
        r.check(f.wait(lambda: sim.mode == "AUTO", 15), "resume: back in AUTO")
        f.wait_real(lambda: rec.action("mission_resume", since) is not None, 6)
        result = rec.action("mission_resume", since)
        r.check(result is not None and result[0], f"resume: VGCS reports it ({result})")
        r.check(f.wait(lambda: 1 in reached(), 90), "resume: it went on to waypoint 2 (it did not start again from waypoint 1)")
        r.check(reached().count(0) == 1, "waypoint 1 was not flown twice")

        # Skip waypoint 3: jump to waypoint 4.
        since = rec.now()
        link.queue_mission_set_current_wp(3)
        r.check(f.wait(lambda: 3 in reached(), 120), "jump: waypoint 4 reached")
        r.check(2 not in reached(), f"jump: waypoint 3 was skipped (reached: {[i + 1 for i in reached()]})")

        # The return to launch is the mission's last item, so the drone stays in AUTO for it.
        r.check(f.wait(lambda: not sim.armed, 240), "end of mission: it came back, landed and disarmed by itself")
        home_off = math.hypot(sim.north, sim.east)
        r.check(home_off < 3.0, f"it landed at home ({home_off:.1f} m from the take-off point)")
        r.check(abs(held_at[0]) + abs(held_at[1]) > 1.0, "the pause happened away from home, in flight")
        r.check(not rec.errors, f"VGCS reported no error {[e[1] for e in rec.errors][:3]}")
    finally:
        f.close()


def takeoff_fence_and_land(version: str, r: Report) -> None:
    """Take-off from VGCS, a geofence that turns the drone back at its edge and at its height limit, and landing from VGCS."""
    # ArduCopter stops a GUIDED drone short of its fence by itself (fence
    # avoidance, AVOID_ENABLE), and refuses a fly-to point outside it. Both
    # off here, by flying on speed commands with avoidance off, so the fence
    # itself is what gets tested: as a pilot flying by hand would breach it.
    f = Flight(version, params={"AVOID_ENABLE": 0})
    try:
        rec, sim, link = f.rec, f.sim, f.link
        r.check(f.ready_to_fly(), "the drone is ready")
        link.queue_geofence_upload({"radius_m": 40.0, "alt_max_m": 30.0, "action": 1.0})
        f.wait_real(lambda: bool(rec.fences), 8)
        r.check(bool(rec.fences) and rec.fences[-1][0], f"fence: VGCS reports it set once the drone echoed every setting ({rec.fences[-1] if rec.fences else None})")
        r.check(sim.get_param("FENCE_ENABLE") == 1.0, "fence: it is switched on in the drone")
        radius = sim.get_param("FENCE_RADIUS")
        r.check(radius is not None and abs(radius - 40.0) < 0.5, f"fence: the drone has the 40 m radius ({radius})")
        kinds = int(sim.get_param("FENCE_TYPE") or 0)
        r.check(kinds & 1 and kinds & 2, f"fence: both the circle and the height limit are switched on (FENCE_TYPE {kinds})")

        since = rec.now()
        link.queue_auto_takeoff(15.0)
        r.check(f.wait(lambda: sim.armed, 40), "take-off: armed")
        r.check(f.wait(lambda: sim.alt > 14.5, 60), f"take-off: climbed to 15 m ({sim.alt:.1f} m)")
        f.wait(lambda: False, 5)    # the take-off is finished before anything else is asked
        result = rec.action("auto_takeoff", since)
        r.check(result is not None and result[0], f"VGCS reports the take-off ({result})")
        shown = rec.last("GLOBAL_POSITION_INT").get("relative_alt_m", 0.0)
        r.check(abs(shown - sim.alt) < 1.5, f"the height VGCS shows is the drone's height ({shown:.1f} m against {sim.alt:.1f} m)")

        # Fly out of the fence, as a pilot might. The fence must bring it back.
        text_since = rec.now()
        out = sim.fly(60, until=lambda: sim.mode == "RTL", each_second=lambda: fly_at(sim, 4.0, 0.0, 0.0))
        r.check(out, f"fence: the drone turned back by itself ({sim.mode}) at {math.hypot(sim.north, sim.east):.0f} m")
        far = math.hypot(sim.north, sim.east)
        r.check(far < 60.0, f"fence: it did not get far past the 40 m line ({far:.0f} m)")
        r.check(f.wait(lambda: rec.mode_text() == "RTL", 10), "fence: VGCS shows RTL")
        f.wait(lambda: False, 3)
        r.check(any("fence" in t.lower() for t in rec.texts(text_since)), f"fence: the drone's message reached VGCS {[t for t in rec.texts(text_since) if 'ence' in t][:2]}")

        # Back over home, take control again from VGCS and climb through the height limit.
        r.check(f.wait(lambda: math.hypot(sim.north, sim.east) < 6.0, 90), f"fence: RTL brought it back over home ({math.hypot(sim.north, sim.east):.0f} m)")
        since = rec.now()
        link.queue_mode_change("GUIDED")
        r.check(f.wait(lambda: sim.mode == "GUIDED", 15), f"in flight: VGCS switched it to GUIDED ({sim.mode})")
        f.wait_real(lambda: any(m[0] >= since and m[1] == "GUIDED" for m in rec.modes), 6)
        said = [m for m in rec.modes if m[0] >= since and m[1] == "GUIDED"]
        r.check(bool(said) and said[-1][2], f"in flight: VGCS confirms GUIDED ({said[-1] if said else 'nothing said'})")
        top = [0.0]

        def note_height(done=lambda: False):
            def check() -> bool:
                top[0] = max(top[0], sim.alt)
                return done()
            return check

        breached = sim.fly(60, until=note_height(lambda: sim.mode == "RTL"), each_second=lambda: fly_at(sim, 0.0, 0.0, 2.0))
        r.check(breached, f"height limit: the drone turned back by itself ({sim.mode}) at {sim.alt:.0f} m")
        f.wait(note_height(), 10)
        r.check(top[0] < 36.0, f"height limit: it did not get far past 30 m (highest {top[0]:.1f} m)")

        since = rec.now()
        link.queue_auto_land()
        r.check(f.wait(lambda: sim.mode == "LAND", 15), f"land: the drone is landing ({sim.mode})")
        f.wait_real(lambda: rec.action("auto_land", since) is not None, 6)
        result = rec.action("auto_land", since)
        r.check(result is not None and result[0], f"land: VGCS reports LAND once the drone is in it ({result})")
        r.check(f.wait(lambda: not sim.armed, 120), "land: it landed and disarmed")
        n_fences = len(rec.fences)
        link.queue_geofence_upload({"disable": True})
        f.wait_real(lambda: len(rec.fences) > n_fences, 8)
        r.check(len(rec.fences) > n_fences and rec.fences[-1][0], f"fence: VGCS reports it off ({rec.fences[-1]})")
        r.check(sim.get_param("FENCE_ENABLE") == 0.0, "fence: switched off in the drone")
    finally:
        f.close()


def link_silence(version: str, r: Report) -> None:
    """The radio link goes silent in flight, then comes back. VGCS must notice within its watchdog time and recover by itself."""
    f = Flight(version)
    try:
        rec, sim, link = f.rec, f.sim, f.link
        r.check(f.ready_to_fly(), "the drone is ready")
        link.queue_auto_takeoff(10.0)
        r.check(f.wait(lambda: sim.alt > 8.0, 80), "the drone is flying")
        r.check(bool(rec.telemetry.get("GLOBAL_POSITION_INT")), "VGCS was getting its position")
        # Freeze the simulator. The connection stays open and nothing arrives,
        # which is what a radio out of range looks like (the client's links are
        # UDP radios). The simulated clock stops too, so the drone does not fall.
        t_quiet = rec.now()
        wsl("pkill -STOP -x arducopter")
        end = time.monotonic() + 10.0
        while time.monotonic() < end and not any(e[0] >= t_quiet and e[1].startswith("timeout") for e in rec.link_events):
            time.sleep(0.05)
        noticed = [e for e in rec.link_events if e[0] >= t_quiet and e[1].startswith("timeout")]
        after = None
        if noticed:
            # From the last message VGCS really received, not from the stop
            # command (starting wsl.exe takes a moment of its own).
            last_data = max(t for rows in rec.telemetry.values() for t, _p in rows if t <= noticed[0][0])
            after = noticed[0][0] - last_data
        r.check(bool(noticed), f"VGCS reported the silence ({after:.1f} s after the last data)" if noticed else "VGCS reported the silence")
        if after is not None:
            r.check(after < 3.5, "within its 2 second watchdog and one read of the link")
        r.check(not [e for e in rec.link_events if e[0] >= t_quiet and e[1] == "down"], "and it kept the link open, waiting for the drone")
        time.sleep(5.0)
        t_back = rec.now()
        wsl("pkill -CONT -x arducopter")
        f.wait_real(lambda: any(e[0] >= t_back and e[1] == "heartbeat" for e in rec.link_events), 10)
        back = [e for e in rec.link_events if e[0] >= t_back and e[1] == "heartbeat"]
        r.check(bool(back), f"the link came back by itself ({back[0][0] - t_back:.1f} s after the drone spoke again)" if back else "the link came back by itself")
        n = len(rec.telemetry.get("GLOBAL_POSITION_INT", []))
        f.wait(lambda: len(rec.telemetry.get("GLOBAL_POSITION_INT", [])) > n + 5, 20)
        r.check(len(rec.telemetry.get("GLOBAL_POSITION_INT", [])) > n + 5, "positions flow again")
        r.check(sim.alt > 8.0 and sim.armed, f"the drone is still flying ({sim.alt:.1f} m)")
        link.stop()
        r.check(link.wait(5000), "the link thread closes cleanly")
    finally:
        f.close()


CASES = [connect_and_telemetry, modes_and_arming_on_the_ground, refusals_say_why, parameters,
         mission_upload_and_download, mission_flight, takeoff_fence_and_land, link_silence]


def write_report(path: pathlib.Path, results: list[tuple[str, Report, float]]) -> None:
    lines = ["# VGCS simulator test results", "",
             "Made by `system_test/vgcs_sitl_test.py`. Each line is one thing checked on the simulated drone.", ""]
    for version in sorted({v for v, _r, _s in results}):
        lines += [f"## ArduCopter {version}", "", "| Case | Result | Checks | Time |", "|------|--------|--------|------|"]
        for v, report, seconds in results:
            if v == version:
                ok = len(report.lines) - report.failed
                lines.append(f"| {report.name} | {'PASS' if not report.failed else 'FAIL'} | {ok} of {len(report.lines)} | {seconds:.0f} s |")
        lines.append("")
        for v, report, _seconds in results:
            if v != version:
                continue
            lines += [f"### {report.name}", "", report.doc, ""]
            lines += [f"- {'ok' if ok else '**FAIL**'}: {text}" for ok, text in report.lines]
            lines.append("")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")


def main(argv: list[str]) -> int:
    app = QApplication.instance() or QApplication(sys.argv[:1])  # noqa: F841  (Qt objects need one)
    versions = [a for a in argv if re.fullmatch(r"\d+\.\d+\.\d+", a)] or VERSIONS
    only = argv[argv.index("-k") + 1] if "-k" in argv else ""
    cases = [c for c in CASES if only in c.__name__]
    results: list[tuple[str, Report, float]] = []
    failed = 0
    began = time.monotonic()
    for version in versions:
        print(f"\n===== ArduCopter {version}")
        for case in cases:
            report = Report(case.__name__, (case.__doc__ or "").strip())
            start = time.monotonic()
            try:
                case(version, report)
            except Exception as exc:   # a broken run is a failed case, not the end of the whole test
                report.check(False, f"the case stopped: {type(exc).__name__}: {exc}")
            seconds = time.monotonic() - start
            results.append((version, report, seconds))
            print(f"  {'FAIL' if report.failed else 'PASS'}  {case.__name__}: {report.doc}  ({seconds:.0f} s)")
            for ok, text in report.lines:
                print(("    ok    " if ok else "    FAIL  ") + text)
            sys.stdout.flush()
            failed += 1 if report.failed else 0
    print(f"\n{len(results) - failed} of {len(results)} cases passed in {time.monotonic() - began:.0f} s")
    if "--report" in argv:
        target = pathlib.Path(argv[argv.index("--report") + 1])
        write_report(target, results)
        print(f"report written to {target}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
