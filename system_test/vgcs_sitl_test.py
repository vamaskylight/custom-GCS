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

# --- the user's settings stay untouched ------------------------------------
# The link reads the payload servo settings when it builds a mission.
# QSettings("VGCS", "VGCS") is the Windows registry, the user's own settings:
# here every QSettings is an INI file in a temporary folder, set up before
# anything of VGCS is imported (the same as system_test/vgcs_window_test.py).
import tempfile  # noqa: E402

from PySide6 import QtCore  # noqa: E402

SETTINGS_DIR = pathlib.Path(tempfile.mkdtemp(prefix="vgcs-sitl-test-"))
_RealQSettings = QtCore.QSettings


class _TestSettings(_RealQSettings):
    def __init__(self, *args, **kwargs):
        if args and isinstance(args[0], str) and not args[0].lower().endswith(".ini"):
            org = args[0]
            app = args[1] if len(args) > 1 and isinstance(args[1], str) else "default"
            super().__init__(str(SETTINGS_DIR / f"{org}-{app}.ini"), _RealQSettings.Format.IniFormat)
        elif not args:
            super().__init__(str(SETTINGS_DIR / "default.ini"), _RealQSettings.Format.IniFormat)
        else:
            super().__init__(*args, **kwargs)


QtCore.QSettings = _TestSettings

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
        f.wait_real(lambda: rec.action("mission_set_current_wp", since) is not None, 8)
        result = rec.action("mission_set_current_wp", since)
        r.check(result is not None and result[0] and "WP 4" in result[1], f"jump: VGCS reports it once the drone shows it ({result})")
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


def mission_speeds(version: str, r: Report) -> None:
    """The drone flies each leg at the speed the plan says: also after a pause, after a jump to another waypoint, and after a mode change."""
    # Long legs, so a speed has time to settle and a jump always has far to go.
    # North, north-east, east, south-east: two legs at 4 m/s, two at 9 m/s.
    # The drone's own default (WP_SPD or WPNAV_SPEED) is 10 m/s.
    planned = (4.0, 4.0, 9.0, 9.0)
    mission = []
    for (north, east), speed in zip(((300.0, 0.0), (300.0, 300.0), (0.0, 300.0), (-300.0, 300.0)), planned):
        lat, lon = offset_to_lat_lon(north, east)
        mission.append({"lat": lat, "lon": lon, "alt_m": 20.0, "speed_mps": speed})
    f = Flight(version)
    try:
        rec, sim, link = f.rec, f.sim, f.link
        sim.current_item = None
        take = sim._take

        def taking(msg) -> None:
            if msg.get_type() == "MISSION_CURRENT":
                sim.current_item = int(msg.seq)
            take(msg)

        sim._take = taking      # the checking connection notes which item the drone is on

        def speed() -> float:
            return math.hypot(sim.vn, sim.ve)

        def flies_at(want: float, what: str, never_faster: bool = False) -> None:
            """The drone settles at this speed: six seconds in a row within half a metre per second of it.

            Braking, turning and speeding up come first, so the speed on the
            way there is reported, and judged only where the drone starts from
            a standstill (never_faster): there it must not overshoot.
            """
            seen: list[float] = []

            def settled() -> bool:
                last = seen[-6:]
                return len(last) == 6 and all(abs(v - want) < 0.5 for v in last)

            sim.fly(70, until=settled, each_second=lambda: seen.append(speed()))
            last = seen[-6:] or [speed()]
            r.check(settled(), f"{what}: it settles at {min(last):.1f} to {max(last):.1f} m/s, the plan says {want:.0f} m/s "
                               f"(after {len(seen)} s, {max(seen):.1f} m/s at most on the way)")
            if never_faster:
                # Measured without the fresh start of the leg: up to 6.3 m/s on a 4 m/s leg.
                r.check(max(seen) < want + 0.7, f"{what}: and never faster than planned on the way ({max(seen):.1f} m/s at most)")

        def on_its_way_to(number: int, what: str) -> None:
            """The checking connection has its own message timing, so give it a moment."""
            f.wait(lambda: sim.current_item == item_of[number], 6)
            r.check(sim.current_item == item_of[number], f"{what} (mission item {sim.current_item})")

        def result(name: str, since: float):
            f.wait_real(lambda: rec.action(name, since) is not None, 8)
            return rec.action(name, since)

        # The layout VGCS sends: 0 home, 1 take-off, 2 speed, 3 WP1, 4 WP2, 5 speed, 6 WP3, 7 WP4, 8 return.
        item_of = {1: 3, 2: 4, 3: 6, 4: 7}

        r.check(f.ready_to_fly(), "the drone is ready")
        link.queue_mission_upload(mission, "rtl")
        r.check(f.wait(lambda: bool(rec.uploaded), 60), "the mission is on the drone: WP 1 and 2 at 4 m/s, WP 3 and 4 at 9 m/s")
        link.queue_mission_start()
        r.check(f.wait(lambda: sim.armed and sim.mode == "AUTO", 40), f"mission start: armed and in AUTO ({sim.mode})")
        r.check(f.wait(lambda: sim.alt > 18.0, 60), f"it took off ({sim.alt:.1f} m)")
        flies_at(4.0, "to WP 1")

        # Pause and resume. Entering AUTO again, the drone forgets the mission's speed.
        since = rec.now()
        link.queue_mission_pause()
        r.check(f.wait(lambda: sim.mode in ("BRAKE", "LOITER", "POSHOLD"), 15), f"pause: the drone holds ({sim.mode})")
        f.wait(lambda: speed() < 0.3, 20)
        since = rec.now()
        link.queue_mission_resume()
        r.check(f.wait(lambda: sim.mode == "AUTO", 15), "resume: back in AUTO")
        flies_at(4.0, "to WP 1 after pause and resume", never_faster=True)
        on_its_way_to(1, "and it is still on its way to WP 1")

        # A jump forward, over the speed item of WP 3.
        since = rec.now()
        link.queue_mission_set_current_wp(2)
        got = result("mission_set_current_wp", since)
        r.check(got is not None and got[0] and "WP 3" in got[1] and "9.0 m/s" in got[1], f"jump to WP 3: VGCS reports it ({got})")
        on_its_way_to(3, "and the drone really is on its way to WP 3")
        flies_at(9.0, "to WP 3 after the jump")

        # A jump back, to a slower waypoint.
        since = rec.now()
        link.queue_mission_set_current_wp(0)
        got = result("mission_set_current_wp", since)
        r.check(got is not None and got[0] and "WP 1" in got[1] and "4.0 m/s" in got[1], f"jump back to WP 1: VGCS reports it ({got})")
        on_its_way_to(1, "and the drone is on its way to WP 1")
        flies_at(4.0, "to WP 1 after the jump back")

        # A jump while paused: the drone stays where it is, and goes there on resume.
        link.queue_mission_pause()
        r.check(f.wait(lambda: sim.mode in ("BRAKE", "LOITER", "POSHOLD"), 15), f"pause again: the drone holds ({sim.mode})")
        f.wait(lambda: speed() < 0.3, 20)
        since = rec.now()
        link.queue_mission_set_current_wp(3)
        got = result("mission_set_current_wp", since)
        r.check(got is not None and got[0] and "WP 4" in got[1] and "Resume" in got[1],
                f"jump to WP 4 while paused: VGCS says it flies there on Resume ({got})")
        f.wait(lambda: False, 5)
        r.check(sim.mode != "AUTO" and speed() < 0.5 and sim.current_item == item_of[4],
                f"the drone still holds, with WP 4 as its next waypoint ({sim.mode}, {speed():.1f} m/s, mission item {sim.current_item})")
        link.queue_mission_resume()
        r.check(f.wait(lambda: sim.mode == "AUTO", 15), "resume: back in AUTO")
        flies_at(9.0, "to WP 4 after the resume")

        # Any other way out of AUTO and back, here the mode list.
        link.queue_mode_change("LOITER")
        r.check(f.wait(lambda: sim.mode == "LOITER", 15), "mode list: LOITER")
        f.wait(lambda: speed() < 1.0, 25)
        link.queue_mode_change("AUTO")
        r.check(f.wait(lambda: sim.mode == "AUTO", 15), "mode list: AUTO again")
        flies_at(9.0, "to WP 4 after LOITER and AUTO from the mode list")

        again = [(ok, text) for _t, name, ok, text in rec.actions if name == "mission_speed"]
        r.check(len(again) == 3 and all(ok for ok, _text in again),
                f"VGCS told the operator each time it set the planned speed again ({len(again)} times, the last: {again[-1][1] if again else 'none'!r})")
        restarted = [line for _t, line in rec.logs if "started again" in line]
        r.check(len(restarted) == 3, f"and each time it started the leg again at that speed ({len(restarted)} times)")
        r.check(not rec.errors, f"VGCS reported no error {[e[1] for e in rec.errors][:3]}")
    finally:
        f.close()


def upload_as_another_station(sim: Sitl, items: list[tuple]) -> int | None:
    """Send a mission on the checking connection, as another ground station would.

    Each item: (frame, command, (p1, p2, p3, p4), lat, lon, height). Returns the
    drone's answer (0 is accepted), or None when it never answered.
    """
    mav = sim.mav
    mav.mav.mission_count_send(mav.target_system, mav.target_component, len(items))
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        msg = mav.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"], blocking=True, timeout=2)
        if msg is None:
            continue
        if msg.get_type() == "MISSION_ACK":
            return int(msg.type)
        frame, command, p, lat, lon, height = items[int(msg.seq)]
        mav.mav.mission_item_int_send(mav.target_system, mav.target_component, int(msg.seq), frame, command, 0, 1,
                                      p[0], p[1], p[2], p[3], int(lat * 1e7), int(lon * 1e7), height)
    return None


def missions_as_they_come_back(version: str, r: Report) -> None:
    """A Download shows the mission as it is: VGCS's own with its payload releases, and another station's with what a plan cannot hold named."""
    from pymavlink import mavutil

    from vgcs.app.vehicle_params import mission_speed_mps
    from vgcs.link.payload_servo_settings import payload_servo_from_settings
    from vgcs.mission import plan_signature, read_downloaded_mission

    M = mavutil.mavlink
    f = Flight(version)
    try:
        rec, sim, link = f.rec, f.sim, f.link
        r.check(f.ready_to_fly(), "the drone is ready")
        servo = payload_servo_from_settings()
        name = "WP_SPD" if version.startswith("4.7") else "WPNAV_SPEED"
        own_speed = mission_speed_mps({name: sim.get_param(name)})
        r.check(own_speed is not None, f"the drone's own mission speed is {own_speed} m/s ({name})")

        def read(rows):
            return read_downloaded_mission(rows, default_speed_mps=own_speed, servo=servo)

        def items_of(rows) -> list[tuple]:
            return [(row["command"], round(row["p1"], 3), round(row["p2"], 3), round(row["lat"], 6), round(row["lon"], 6),
                     round(row["alt_m"], 2)) for row in rows[1:]]          # without the home slot

        def download() -> list[dict]:
            before = len(rec.downloaded)
            link.queue_mission_download()
            f.wait(lambda: len(rec.downloaded) > before, 60)
            return rec.downloaded[-1] if len(rec.downloaded) > before else []

        def same_waypoints(got, sent_rows) -> bool:
            """The same plan after the trip over the radio, where a place is a whole number of 1e-7 degrees."""
            if len(got) != len(sent_rows):
                return False
            for wp, row in zip(got, sent_rows):
                if distance_m(wp.lat, wp.lon, row["lat"], row["lon"]) > 0.05:
                    return False
                if abs(wp.alt_m - row["alt_m"]) > 0.01 or abs(wp.speed_mps - row["speed_mps"]) > 0.01:
                    return False
                if wp.hover_s != row["hover_s"] or wp.drop_payload != row["drop_payload"]:
                    return False
            return True

        # VGCS's own mission, with a hover and two payload releases.
        sent = []
        for (north, east), speed, hover, drop in (((60.0, 0.0), 6.0, 4, True), ((60.0, 120.0), 6.0, 0, False), ((0.0, 120.0), 9.0, 0, True)):
            lat, lon = offset_to_lat_lon(north, east)
            sent.append({"lat": lat, "lon": lon, "alt_m": 25.0, "speed_mps": speed, "hover_s": hover, "drop_payload": drop})
        link.queue_mission_upload(sent, "rtl")
        r.check(f.wait(lambda: len(rec.uploaded) == 1, 60), "VGCS's mission with two payload releases is on the drone")
        first = download()
        mission = read(first)
        r.check([wp.drop_payload for wp in mission.waypoints] == [True, False, True],
                f"download: the payload releases come back on their waypoints ({[wp.drop_payload for wp in mission.waypoints]})")
        r.check(same_waypoints(mission.waypoints, sent) and mission.end_action == "rtl",
                "and every other value: place, height, speed, hover, and the return at the end")
        r.check(mission.not_kept == [], f"nothing is left that the plan cannot hold ({mission.not_kept})")
        again = [{"lat": wp.lat, "lon": wp.lon, "alt_m": wp.alt_m, "speed_mps": wp.speed_mps, "hover_s": wp.hover_s,
                  "drop_payload": wp.drop_payload} for wp in mission.waypoints]
        link.queue_mission_upload(again, mission.end_action)
        r.check(f.wait(lambda: len(rec.uploaded) == 2, 60), "the downloaded plan is uploaded again")
        second = download()
        r.check(items_of(second) == items_of(first) and len(first) == 14,
                f"and the drone holds the same mission as before, item for item ({len(first)} items, {len(second)} after)")
        # After a download the window tells the link which plan the mission is.
        link.set_mission_on_drone(plan_signature(read(second).waypoints), ())

        # Another ground station sends a mission of its own.
        a, b, c = offset_to_lat_lon(80.0, 0.0), offset_to_lat_lon(80.0, 80.0), offset_to_lat_lon(0.0, 80.0)
        rel, sea = M.MAV_FRAME_GLOBAL_RELATIVE_ALT, M.MAV_FRAME_GLOBAL
        theirs = [
            (rel, M.MAV_CMD_NAV_WAYPOINT, (0, 0, 0, 0), a[0], a[1], 0.0),              # the home slot
            (rel, M.MAV_CMD_NAV_TAKEOFF, (0, 0, 0, 0), 0.0, 0.0, 30.0),
            (rel, M.MAV_CMD_DO_SET_ROI, (0, 0, 0, 0), b[0], b[1], 0.0),
            (rel, M.MAV_CMD_NAV_WAYPOINT, (3, 0, 0, 0), a[0], a[1], 30.0),
            (rel, M.MAV_CMD_DO_DIGICAM_CONTROL, (0, 0, 0, 0), 0.0, 0.0, 0.0),
            (rel, M.MAV_CMD_DO_SET_SERVO, (10, 1500, 0, 0), 0.0, 0.0, 0.0),
            (sea, M.MAV_CMD_NAV_WAYPOINT, (0, 0, 0, 0), b[0], b[1], 640.0),            # height above sea level
            (rel, M.MAV_CMD_NAV_SPLINE_WAYPOINT, (0, 0, 0, 0), c[0], c[1], 30.0),
            (rel, M.MAV_CMD_DO_CHANGE_SPEED, (2, 1.5, -1, 0), 0.0, 0.0, 0.0),          # a climb speed
            (rel, M.MAV_CMD_NAV_LOITER_TIME, (10, 0, 0, 0), a[0], a[1], 30.0),
            (rel, M.MAV_CMD_DO_JUMP, (3, 1, 0, 0), 0.0, 0.0, 0.0),
            (rel, M.MAV_CMD_NAV_LAND, (0, 0, 0, 0), b[0], b[1], 0.0),
        ]
        f.wait_real(lambda: False, 5.5)        # the answer to VGCS's own upload is long past
        since = rec.now()
        r.check(link.mission_on_drone() is not None, "before: VGCS knows the mission on the drone")
        answer = upload_as_another_station(sim, theirs)
        r.check(answer == 0, f"another ground station's mission is on the drone (the drone answered {answer})")
        f.wait_real(lambda: rec.action("mission", since) is not None, 6)
        told = rec.action("mission", since)
        r.check(told is not None and not told[0] and "changed" in told[1] and "Download" in told[1],
                f"VGCS notices (the drone reports another number of items) and says so ({told})")
        r.check(link.mission_on_drone() is None, "and no longer takes its own plan for the mission on the drone")

        rows = download()
        mission = read(rows)
        home_height = float(rows[0]["alt_m"]) if rows else 0.0
        r.check(len(rows) == len(theirs), f"download: every item of it comes back ({len(rows)} of {len(theirs)})")
        r.check(len(mission.waypoints) == 4 and mission.end_action == "land",
                f"the plan shows its 4 waypoints and the landing ({len(mission.waypoints)}, {mission.end_action})")
        heights = [round(wp.alt_m, 1) for wp in mission.waypoints]
        r.check(heights == [30.0, round(640.0 - home_height, 1), 30.0, 30.0],
                f"the height above sea level is shown above the launch point: {heights} (the drone's home is at {home_height:.1f} m)")
        r.check([wp.speed_mps for wp in mission.waypoints] == [own_speed] * 4,
                f"it sets no speed, so the plan shows the drone's own ({[wp.speed_mps for wp in mission.waypoints]})")
        r.check(mission.waypoints[0].hover_s == 3 and mission.waypoints[3].hover_s == 10, "the hover times are kept")
        want = ["1 x camera aim point (ROI)", "1 x camera trigger", "1 x servo command", "1 x climb or descent speed",
                "1 x jump to another item"]
        r.check(mission.not_kept[:5] == want, f"what the plan cannot hold is named: {mission.not_kept[:5]}")
        rest = " | ".join(mission.not_kept[5:])
        r.check("1 waypoint had its height above sea level" in rest and "2 waypoints of another kind" in rest
                and "the landing is at another place" in rest and len(mission.not_kept) == 8,
                f"and what it shows differently: {rest}")
        r.check(not rec.errors, f"VGCS reported no error {[e[1] for e in rec.errors][:3]}")
    finally:
        f.close()


def another_stations_mission(version: str, r: Report) -> None:
    """Another ground station replaces the mission with one of the same size and VGCS is told nothing. VGCS finds out before it uses its own plan: at a start, at a jump, at a resume, and from the drone's word in flight."""
    from pymavlink import mavutil

    from vgcs.app.vehicle_params import mission_speed_mps
    from vgcs.link.payload_servo_settings import payload_servo_from_settings
    from vgcs.mission import plan_signature, read_downloaded_mission

    M = mavutil.mavlink
    f = Flight(version)
    try:
        rec, sim, link = f.rec, f.sim, f.link
        sim.current_item = None
        take = sim._take

        def taking(msg) -> None:
            if msg.get_type() == "MISSION_CURRENT":
                sim.current_item = int(msg.seq)
            take(msg)

        sim._take = taking      # the checking connection notes which item the drone is on

        def speed() -> float:
            return math.hypot(sim.vn, sim.ve)

        def their_mission(places, leg_speed: float) -> list[tuple]:
            """Eight items, always: home slot, take-off, one speed item, four waypoints, return."""
            rel = M.MAV_FRAME_GLOBAL_RELATIVE_ALT
            first = offset_to_lat_lon(*places[0])
            items = [(rel, M.MAV_CMD_NAV_WAYPOINT, (0, 0, 0, 0), first[0], first[1], 0.0),
                     (rel, M.MAV_CMD_NAV_TAKEOFF, (0, 0, 0, 0), 0.0, 0.0, 20.0),
                     (rel, M.MAV_CMD_DO_CHANGE_SPEED, (1, leg_speed, -1, 0), 0.0, 0.0, 0.0)]
            for north, east in places:
                lat, lon = offset_to_lat_lon(north, east)
                items.append((rel, M.MAV_CMD_NAV_WAYPOINT, (0, 0, 0, 0), lat, lon, 20.0))
            items.append((rel, M.MAV_CMD_NAV_RETURN_TO_LAUNCH, (0, 0, 0, 0), 0.0, 0.0, 0.0))
            return items

        # Long legs: the drone is still on its first one at the end of every step.
        north_east = ((1500.0, 0.0), (1500.0, 1500.0), (0.0, 1500.0), (-500.0, 1500.0))
        south_west = ((-1500.0, 0.0), (-1500.0, -1500.0), (0.0, -1500.0), (500.0, -1500.0))

        servo = payload_servo_from_settings()
        name = "WP_SPD" if version.startswith("4.7") else "WPNAV_SPEED"
        own_speed = mission_speed_mps({name: sim.get_param(name)})

        def download_and_know() -> list[dict]:
            """A Download, and what the window does with it: it tells the link which plan the mission is."""
            before = len(rec.downloaded)
            link.queue_mission_download()
            f.wait(lambda: len(rec.downloaded) > before, 60)
            rows = rec.downloaded[-1] if len(rec.downloaded) > before else []
            plan = read_downloaded_mission(rows, default_speed_mps=own_speed, servo=servo)
            link.set_mission_on_drone(plan_signature(plan.waypoints), tuple(plan.not_kept))
            return rows

        def replaced_without_a_word(places, leg_speed: float, what: str) -> float:
            """The other station sends its mission. Returns the time just before."""
            since = rec.now()
            answer = upload_as_another_station(sim, their_mission(places, leg_speed))
            f.wait_real(lambda: False, 3.0)
            r.check(answer == 0 and rec.action("mission", since) is None and link.mission_on_drone() is not None,
                    f"{what}: the other station replaces the mission with one of the same size (the drone answered "
                    f"{answer}). VGCS is told nothing, and still takes the old one for the drone's")
            return since

        def said(action: str, since: float, seconds: float = 8.0):
            f.wait_real(lambda: rec.action(action, since) is not None, seconds)
            return rec.action(action, since)

        not_the_one = "the mission on the drone is not the one VGCS knows"

        r.check(f.ready_to_fly(), "the drone is ready")
        r.check(upload_as_another_station(sim, their_mission(north_east, 5.0)) == 0,
                "another ground station's mission is on the drone: four waypoints to the north and east, 5 m/s")
        rows = download_and_know()
        r.check(len(rows) == 8 and link.mission_on_drone() is not None, f"VGCS downloads it ({len(rows)} items)")

        # --- "Start it as it is on the drone" ------------------------------------------------
        since = replaced_without_a_word(south_west, 5.0, "on the ground")
        link.queue_mission_start(as_it_is=True)
        told = said("mission", since)
        r.check(told is not None and not told[0] and told[1].startswith("Not started: the mission on the drone changed since the Download")
                and "Download" in told[1], f"start as it is: VGCS reads the mission again, finds another one, and starts nothing ({told})")
        f.wait(lambda: sim.armed, 8)
        r.check(not sim.armed and sim.mode != "AUTO", f"the drone was not armed ({sim.mode})")
        r.check(link.mission_on_drone() is None, "and VGCS no longer takes the old mission for the drone's")

        rows = download_and_know()
        since = rec.now()
        link.queue_mission_start(as_it_is=True)
        r.check(f.wait(lambda: sim.armed and sim.mode == "AUTO", 40),
                f"after a new Download the same start works: armed and in AUTO ({sim.mode})")
        r.check(any("still holds the downloaded mission" in line for when, line in rec.logs if when >= since),
                "VGCS read all 8 items again and compared them before it armed")
        r.check(f.wait(lambda: sim.alt > 18.0, 60), f"it took off ({sim.alt:.1f} m)")
        f.wait(lambda: sim.vn < -4.0, 40)
        r.check(sim.vn < -4.0 and abs(sim.ve) < 1.0, f"and flies south, to the first waypoint of the mission that is on the drone "
                                                    f"(north {sim.vn:+.1f}, east {sim.ve:+.1f} m/s)")

        # --- a jump --------------------------------------------------------------------------
        link.queue_mission_pause()
        r.check(f.wait(lambda: sim.mode in ("BRAKE", "LOITER", "POSHOLD"), 15), f"pause: the drone holds ({sim.mode})")
        f.wait(lambda: speed() < 0.3, 20)
        item_before = sim.current_item
        since = replaced_without_a_word(north_east, 7.0, "while it holds")
        since = rec.now()
        link.queue_mission_set_current_wp(3)
        told = said("mission_set_current_wp", since)
        r.check(told is not None and not told[0] and told[1].startswith("WP 4 not taken") and not_the_one in told[1],
                f"Fly to WP 4: VGCS asks the drone for the waypoint first, finds another one, and sends no jump ({told})")
        f.wait(lambda: False, 5)
        r.check(sim.mode != "AUTO" and speed() < 0.5 and sim.current_item == item_before,
                f"the drone still holds and its next item is the same ({sim.mode}, {speed():.1f} m/s, mission item {sim.current_item})")
        r.check(link.mission_on_drone() is None, "and VGCS no longer takes the old mission for the drone's")

        # --- a resume ------------------------------------------------------------------------
        rows = download_and_know()
        since = replaced_without_a_word(south_west, 3.0, "while it holds, again")
        since = rec.now()
        link.queue_mission_resume()
        r.check(f.wait(lambda: sim.mode == "AUTO", 15), "resume: back in AUTO")
        told = said("mission_speed", since)
        r.check(told is not None and not told[0] and told[1].startswith("Planned speed NOT set again") and not_the_one in told[1],
                f"VGCS asks the drone for the waypoint first, finds another one, and sends no speed ({told})")
        seen: list[float] = []
        sim.fly(40, until=lambda: len(seen) >= 8 and all(abs(v - 10.0) < 0.5 for v in seen[-6:]),
                each_second=lambda: seen.append(speed()))
        last = seen[-6:] or [speed()]
        r.check(len(last) == 6 and all(abs(v - 10.0) < 0.5 for v in last),
                f"the drone flies at its own {min(last):.1f} to {max(last):.1f} m/s. The plan VGCS had says 7 m/s: that was not sent")
        r.check(not any(ok for when, action, ok, _text in rec.actions if action == "mission_speed" and when >= since),
                "and VGCS did not report a speed as set")

        # --- the drone's own word, in flight ---------------------------------------------------
        rows = download_and_know()
        r.check(len(rows) == 8 and link.mission_on_drone() is not None, "VGCS downloads the mission the drone flies")
        since = rec.now()
        answer = upload_as_another_station(sim, their_mission(north_east, 5.0))
        told = said("mission", since)
        heard = [text for text in rec.texts(since) if text.lower().startswith("auto mission changed")]
        r.check(answer == 0 and len(heard) >= 1,
                f"in flight the other station replaces the mission again: this time the drone says so on every link ({len(heard)} x {heard[:1]})")
        r.check(told is not None and not told[0] and told[1].startswith("Another ground station changed the mission")
                and link.mission_on_drone() is None, f"VGCS drops its plan at once and says why ({told})")

        # --- VGCS's own upload in flight is not another station's --------------------------------
        mine = []
        for north, east in north_east:
            lat, lon = offset_to_lat_lon(north, east)
            mine.append({"lat": lat, "lon": lon, "alt_m": 20.0, "speed_mps": 4.0})
        uploads = len(rec.uploaded)
        since = rec.now()
        link.queue_mission_upload(mine, "rtl")
        r.check(f.wait(lambda: len(rec.uploaded) > uploads, 60), "VGCS uploads a mission of its own while the drone flies")
        f.wait_real(lambda: False, 4.0)
        heard = [text for text in rec.texts(since) if text.lower().startswith("auto mission changed")]
        r.check(rec.action("mission", since) is None and link.mission_on_drone() is not None,
                f"the drone's word about that one ({len(heard)} x) is not taken for another station's: VGCS keeps its plan")
        link.queue_mission_pause()
        r.check(f.wait(lambda: sim.mode in ("BRAKE", "LOITER", "POSHOLD"), 15), f"pause: the drone holds ({sim.mode})")
        f.wait(lambda: speed() < 0.3, 20)
        since = rec.now()
        link.queue_mission_resume()
        r.check(f.wait(lambda: sim.mode == "AUTO", 15), "resume: back in AUTO")
        told = said("mission_speed", since)
        r.check(told is not None and told[0] and "4.0 m/s set again" in told[1],
                f"the check finds VGCS's own mission on the drone, and the planned speed is set again ({told})")
        seen = []
        sim.fly(60, until=lambda: len(seen) >= 8 and all(abs(v - 4.0) < 0.5 for v in seen[-6:]),
                each_second=lambda: seen.append(speed()))
        last = seen[-6:] or [speed()]
        r.check(len(last) == 6 and all(abs(v - 4.0) < 0.5 for v in last) and max(seen) < 4.7,
                f"and the drone flies it: {min(last):.1f} to {max(last):.1f} m/s, {max(seen):.1f} m/s at most on the way")

        expected = ("Mission start: not started", "Mission jump failed: WP 4 not taken")
        others = [line for _when, line in rec.errors if not line.startswith(expected)]
        r.check(not others, f"VGCS reported no other error than the start and the jump it refused {others[:3]}")
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


def signed_commands(version: str, r: Report) -> None:
    """Command signing (M16): once the drone holds VGCS's key, another ground station without it cannot command it."""
    from pymavlink import mavutil

    f = Flight(version)
    outsider = None
    try:
        rec, sim, link = f.rec, f.sim, f.link
        r.check(f.ready_to_fly(), "the drone is ready")
        # Another ground station on the simulator's third port, without the key.
        outsider = mavutil.mavlink_connection(f"tcp:127.0.0.1:{sim.port + 3}", source_system=250)
        r.check(outsider.wait_heartbeat(timeout=20) is not None, "an outsider ground station is connected too")

        def outsider_mode(name: str) -> None:
            outsider.set_mode(outsider.mode_mapping()[name])

        outsider_mode("GUIDED")
        r.check(f.wait(lambda: sim.mode == "GUIDED", 15), f"without signing the outsider can change the mode ({sim.mode})")
        outsider_mode("STABILIZE")
        f.wait(lambda: sim.mode == "STABILIZE", 15)

        def signing_state() -> str:
            rows = rec.telemetry.get("SIGNING", [])
            return rows[-1][1].get("state", "") if rows else ""

        key = bytes(range(1, 33))
        link.queue_signing_key(key, 9)
        f.wait_real(lambda: signing_state() == "drone_unsigned", 8)
        r.check(signing_state() == "drone_unsigned", f"VGCS signs, and sees that the drone does not yet ({signing_state()})")
        since = rec.now()
        link.queue_signing_to_drone(True)
        f.wait_real(lambda: rec.action("signing", since) is not None, 10)
        result = rec.action("signing", since)
        r.check(result is not None and result[0], f"key sent: VGCS reports that the drone took it ({result})")
        # The state comes in the link's next message, a moment after the result
        # (read at once, this check failed in one run of several).
        f.wait_real(lambda: signing_state() == "drone_signs", 5)
        r.check(signing_state() == "drone_signs", f"the drone signs with VGCS's key ({signing_state()})")

        for _ in range(4):
            outsider_mode("GUIDED")
            f.wait(lambda: False, 2)
        r.check(sim.mode == "STABILIZE", f"the outsider's mode change is ignored now ({sim.mode})")
        outsider.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        f.wait(lambda: False, 5)
        r.check(not sim.armed, "and its arm command too")
        outsider.mav.setup_signing_send(1, 1, bytes(32), 0)
        f.wait(lambda: False, 5)
        f.wait_real(lambda: False, 2)
        r.check(signing_state() == "drone_signs", f"the outsider cannot take the key away ({signing_state()})")

        since = rec.now()
        link.queue_mode_change("LOITER")
        r.check(f.wait(lambda: sim.mode == "LOITER", 15), f"VGCS's signed mode change is obeyed ({sim.mode})")
        f.wait_real(lambda: any(m[0] >= since and m[1] == "LOITER" for m in rec.modes), 6)
        said = [m for m in rec.modes if m[0] >= since and m[1] == "LOITER"]
        r.check(bool(said) and said[-1][2], "and confirmed")

        since = rec.now()
        link.queue_signing_to_drone(False)
        f.wait_real(lambda: rec.action("signing", since) is not None, 10)
        result = rec.action("signing", since)
        r.check(result is not None and result[0], f"key removed from the drone ({result})")
        outsider_mode("ALT_HOLD")
        r.check(f.wait(lambda: sim.mode == "ALT_HOLD", 15), f"without the key on the drone the outsider is obeyed again ({sim.mode})")
        r.check(not [e for e in rec.errors if "Signing" in e[1]], f"no signing error {[e[1] for e in rec.errors if 'Signing' in e[1]][:2]}")
    finally:
        if outsider is not None:
            outsider.close()
        f.close()


CASES = [connect_and_telemetry, modes_and_arming_on_the_ground, refusals_say_why, parameters,
         mission_upload_and_download, mission_flight, mission_speeds, missions_as_they_come_back,
         another_stations_mission, takeoff_fence_and_land, link_silence, signed_commands]


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
