"""Try the VAMA lock script (part 1 of the firmware lock) in the ArduCopter simulator.

Usage:
    py lock_test.py                  all cases on 4.6.2 and 4.7.0
    py lock_test.py 4.7.0            one version
    py lock_test.py -k bound         only the cases with "bound" in the name
    py lock_test.py --script other.lua   test another copy of the script

The simulator's ports stand for the drone's:
    5760  MAVLink channel 0, the USB port. This test drives the drone from it
          (parameters, the key, arm and disarm, and the sticks through RC override).
    5762  channel 1, a telemetry port: the bound ground station.
    5763  channel 2, another telemetry port: other ground stations.
"""
from __future__ import annotations

import os
import pathlib
import re
import sys
import time

# MAVLink 2 from the first byte: signing needs it.
os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil  # noqa: E402

from sitl_session import Sitl  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
SCRIPT = HERE.parent / "scripts" / "vama_lock.lua"
TAG = "VAMA lock"
READY = "VAMA lock: ready"
VERSIONS = ["4.6.2", "4.7.0"]

# FS_GCS_ENABLE 0: a lost ground station must not start a failsafe in these tests.
BASE = {"SCR_ENABLE": 1, "SCR_HEAP_SIZE": 300000, "RC_OVERRIDE_TIME": -1, "FS_GCS_ENABLE": 0}

KEY = bytes(range(1, 33))
WRONG_KEY = bytes(range(101, 133))
BOUND_PORT, OTHER_PORT = 5762, 5763
MAV_TYPE_ONBOARD_CONTROLLER = 18

# Every arming check off: ARMING_CHECK 0 on 4.6, ARMING_SKIPCHK -1 on 4.7.
CHECKS_OFF = {"4.6": ("ARMING_CHECK", 0), "4.7": ("ARMING_SKIPCHK", -1)}
# Some checks off: only the barometer check on 4.6, the GPS check skipped on 4.7.
SOME_CHECKS_OFF = {"4.6": ("ARMING_CHECK", 2), "4.7": ("ARMING_SKIPCHK", 8)}


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


class Station:
    """A ground station on another port of the simulator. beat() sends one heartbeat.

    key: it signs everything with it. A station first reads what the drone
    sent, as a real ground station does: a signed message moves its signing
    clock to the drone's (the simulator's clock runs 10 times faster).
    """

    def __init__(self, port: int, key: bytes | None = None, sysid: int = 255,
                 mav_type: int = mavutil.mavlink.MAV_TYPE_GCS, link_id: int = 7) -> None:
        self.conn = mavutil.mavlink_connection(f"tcp:127.0.0.1:{port}", source_system=sysid,
                                               source_component=190, retries=20)
        if self.conn.wait_heartbeat(timeout=30) is None:
            raise RuntimeError(f"no heartbeat from the drone on port {port}")
        self.mav_type = mav_type
        self.signed_from_drone = False
        if key is not None:
            self.conn.setup_signing(key, sign_outgoing=True, link_id=link_id)

    def drain(self) -> None:
        while True:
            msg = self.conn.recv_match(blocking=False)
            if msg is None:
                return
            if msg.get_type() == "HEARTBEAT" and getattr(msg, "_signed", False):
                self.signed_from_drone = True

    def beat(self) -> None:
        self.drain()
        self.conn.mav.heartbeat_send(self.mav_type, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                                     mavutil.mavlink.MAV_STATE_ACTIVE)

    def beat_after_others(self, count: int) -> None:
        """A busy link: `count` heartbeats that are not a ground station's, then this station's own."""
        self.drain()
        for _ in range(count):
            self.conn.mav.heartbeat_send(MAV_TYPE_ONBOARD_CONTROLLER, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                                         mavutil.mavlink.MAV_STATE_ACTIVE)
        self.conn.mav.heartbeat_send(self.mav_type, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                                     mavutil.mavlink.MAV_STATE_ACTIVE)

    def beat_with_broken_checksum(self) -> None:
        """A signed heartbeat whose checksum is wrong. ArduPilot hands such messages to scripts."""
        self.drain()
        msg = self.conn.mav.heartbeat_encode(self.mav_type, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                                             mavutil.mavlink.MAV_STATE_ACTIVE)
        frame = bytearray(msg.pack(self.conn.mav))
        payload_len = frame[1]
        frame[10 + payload_len] ^= 0xFF   # the first checksum byte
        self.conn.write(bytes(frame))

    def close(self) -> None:
        try:
            self.conn.close()
        except Exception:
            pass


# --- helpers ---------------------------------------------------------------

def start(version: str, enable: int | None = 1, params: dict | None = None) -> Sitl:
    script_params = {} if enable is None else {"VLCK_ENABLE": enable}
    sim = Sitl(version, {"vama_lock.lua": SCRIPT}, dict(BASE, **(params or {})),
               script_params=script_params, script_ready=READY)
    # Arming needs the position estimate and a GPS fix.
    sim.pump(120, until=lambda last: "EKF_STATUS_REPORT" in last and (last["EKF_STATUS_REPORT"].flags & 0x18) == 0x18
             and "GPS_RAW_INT" in last and last["GPS_RAW_INT"].fix_type >= 3)
    sim.rc(rc3=1000, rc4=1500)
    return sim


def give_key(sim: Sitl, key: bytes = KEY) -> None:
    """Put the signing key on the drone over USB (channel 0 takes it unsigned)."""
    stamp = int((time.time() - 1420070400) * 100000)
    for _ in range(3):
        sim.mav.mav.setup_signing_send(sim.mav.target_system, sim.mav.target_component, key, stamp)
        sim.pump(0.3)


def lock_texts(sim: Sitl, since: float = 0.0) -> list[str]:
    return [t for t in sim.texts_with(TAG, since) if not t.startswith(READY)]


def lock_state(sim: Sitl) -> float:
    return sim.named.get("VLCK", -1)


def fly_with(sim: Sitl, seconds: float, *stations: Station, until=None) -> bool:
    """Let the simulated clock run, each station sending a heartbeat every simulated second."""
    def each_second() -> None:
        for station in stations:
            station.beat()
    return sim.fly(seconds, until=until, each_second=each_second)


def stick_arm(sim: Sitl, *stations: Station, seconds: float = 5) -> bool:
    """Throttle down and yaw right, as a pilot arms with the sticks."""
    sim.rc(rc3=1000, rc4=2000)
    fly_with(sim, seconds, *stations, until=lambda: sim.armed)
    sim.rc(rc4=1500)
    return sim.armed


def command_arm(sim: Sitl, *stations: Station, force: bool = False) -> bool:
    """An arm command over USB, as a ground station sends it. force: skip the arming checks."""
    sim.mav.mav.command_long_send(sim.mav.target_system, sim.mav.target_component,
                                  mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196 if force else 0,
                                  0, 0, 0, 0, 0)
    fly_with(sim, 3, *stations, until=lambda: sim.armed)
    return sim.armed


def disarm(sim: Sitl, *stations: Station) -> None:
    for _ in range(5):
        sim.mav.mav.command_long_send(sim.mav.target_system, sim.mav.target_component,
                                      mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196, 0, 0, 0, 0, 0)
        fly_with(sim, 2, *stations, until=lambda: not sim.armed)
        if not sim.armed:
            return
    raise RuntimeError("could not disarm")


def refused_by_the_lock(sim: Sitl, since: float) -> bool:
    return any("VAMA locked" in t for t in sim.texts_with("locked", since))


def script_errors(sim: Sitl) -> list[str]:
    return [t for _, t in sim.texts
            if "VAMA lock: error" in t or t.startswith("Lua:") or ("Scripting: " in t and "error" in t.lower())]


CHECKS_OFF_TEXT = "VAMA lock: arming checks are all off, no lock"


def common_checks(sim: Sitl, r: Report, checks_off_since: float | None = None) -> None:
    """checks_off_since: when the case turned all arming checks off (None: never)."""
    early = [t for w, t in sim.texts if CHECKS_OFF_TEXT in t and (checks_off_since is None or w < checks_off_since)]
    r.check(not early, f"no word of the arming checks while they are on {early[:1]}")
    errors = script_errors(sim)
    r.check(not errors, "the script reported no error" + (f": {errors[:2]}" if errors else ""))
    # A status text holds 50 characters. A longer one arrives in pieces.
    long_texts = [t for t in sim.texts_with(TAG) if len(t) > 50]
    r.check(not long_texts, "every text fits one status message" + (f": {long_texts[:2]}" if long_texts else ""))


# --- the cases -------------------------------------------------------------

def off_by_default(version: str, r: Report) -> None:
    """Installed with VLCK_ENABLE at its default 0: nothing changes, the sticks arm."""
    sim = start(version, enable=None)
    try:
        sim.fly(3)
        r.check(sim.saw("VAMA lock: ready (off)"), "it says that it is ready and off")
        r.check(sim.get_param("VLCK_ENABLE") == 0, "VLCK_ENABLE is 0 after the install")
        r.check(lock_state(sim) == 0, f"VLCK says off ({lock_state(sim)})")
        r.check(stick_arm(sim), "the sticks arm the drone, with no ground station")
        disarm(sim)
        common_checks(sim, r)
    finally:
        sim.close()


def locked_without_a_station(version: str, r: Report) -> None:
    """Key on the drone and the lock on, no ground station: no arming by sticks or by command."""
    sim = start(version)
    try:
        give_key(sim)
        sim.fly(3)
        r.check(sim.saw("VAMA lock: locked, no signed ground station"), f"it says that it is locked {lock_texts(sim)}")
        r.check(lock_state(sim) == 1, f"VLCK says locked ({lock_state(sim)})")
        since = sim.sim_s
        r.check(not stick_arm(sim), "the sticks do not arm the drone")
        r.check(refused_by_the_lock(sim, since), f"the arming check names the lock {sim.texts_with('arm', since)[-3:]}")
        since = sim.sim_s
        r.check(not command_arm(sim), "an arm command does not arm it either")
        r.check(refused_by_the_lock(sim, since), "and that refusal names the lock too")
        common_checks(sim, r)
    finally:
        sim.close()


def bound_station_unlocks(version: str, r: Report) -> None:
    """The ground station with the key unlocks it while it is connected, and the lock comes back after."""
    sim = start(version)
    station = None
    try:
        give_key(sim)
        station = Station(BOUND_PORT, key=KEY)
        fly_with(sim, 6, station)
        r.check(station.signed_from_drone, "the drone signs with the key")
        r.check(sim.saw("VAMA lock: unlocked by GCS 255, channel 1"), f"it says who unlocked it {lock_texts(sim)}")
        r.check(lock_state(sim) == 2, f"VLCK says unlocked ({lock_state(sim)})")
        r.check(stick_arm(sim, station), "the sticks arm the drone")
        disarm(sim, station)
        r.check(command_arm(sim, station), "an arm command arms it too")
        disarm(sim, station)
        # The station goes away: locked again after VLCK_HOLD (5 s).
        gone = sim.sim_s
        sim.fly(8, until=lambda: sim.saw("locked, no signed", gone))
        locked_at = [w for w, t in sim.texts if w >= gone and "locked, no signed" in t]
        after = (locked_at[0] - gone) if locked_at else -1
        r.check(4 <= after <= 7, f"locked again {after:.1f} s after the last heartbeat (VLCK_HOLD 5)")
        since = sim.sim_s
        r.check(not stick_arm(sim), "then the sticks do not arm it")
        r.check(refused_by_the_lock(sim, since), "and the arming check names the lock")
        common_checks(sim, r)
    finally:
        if station is not None:
            station.close()
        sim.close()


def others_stay_locked(version: str, r: Report) -> None:
    """With the key on the drone, nothing but a signed ground station on a telemetry port unlocks it."""
    sim = start(version)
    stations: list[Station] = []
    try:
        give_key(sim)
        sim.fly(2)

        def stays_locked(what: str, station: Station | None = None, send=None) -> None:
            since = sim.sim_s

            def each_second() -> None:
                if send is not None:
                    send()
                elif station is not None:
                    station.beat()
            sim.fly(7, each_second=each_second)
            unlocked = [t for t in sim.texts_with("unlocked", since)]
            r.check(not unlocked and lock_state(sim) == 1, f"{what}: still locked {unlocked}")

        plain = Station(OTHER_PORT)
        stations.append(plain)
        stays_locked("a ground station without the key", plain)
        r.check(not sim.texts_with("unsigned GCS"), "ArduPilot drops its messages before the script sees them")
        plain.close()

        wrong = Station(OTHER_PORT, key=WRONG_KEY)
        stations.append(wrong)
        stays_locked("a ground station with another key", wrong)
        wrong.close()

        onboard = Station(OTHER_PORT, key=KEY, mav_type=MAV_TYPE_ONBOARD_CONTROLLER)
        stations.append(onboard)
        stays_locked("a signed heartbeat that is not a ground station's", onboard)
        onboard.close()

        broken = Station(OTHER_PORT, key=KEY, link_id=9)
        stations.append(broken)
        stays_locked("a signed ground station heartbeat with a wrong checksum", send=broken.beat_with_broken_checksum)
        # The same station, correct this time: the port itself is fine.
        since = sim.sim_s
        fly_with(sim, 4, broken)
        r.check(sim.saw("unlocked by GCS 255, channel 2", since), "the same station with a correct checksum unlocks it")
        broken.close()
        sim.fly(8)

        # USB takes anything, so nothing on it unlocks: the main connection signs with the key.
        sim.mav.setup_signing(KEY, sign_outgoing=True, link_id=3)

        def usb_beat() -> None:
            sim.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                       0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
        stays_locked("a signed ground station on USB", send=usb_beat)
        r.check(sim.saw("VAMA lock: USB does not unlock, use the radio link"), "and it says that USB does not unlock")
        common_checks(sim, r)
    finally:
        for station in stations:
            station.close()
        sim.close()


def no_key_on_the_drone(version: str, r: Report) -> None:
    """Without the key the lock still stops the sticks, but any ground station can turn it off. It says so."""
    sim = start(version)
    plain = None
    try:
        plain = Station(OTHER_PORT)
        fly_with(sim, 4, plain)
        r.check(sim.saw("VAMA lock: unsigned GCS 255 on channel 2, no key?"), f"it names the unsigned ground station {lock_texts(sim)}")
        r.check(lock_state(sim) == 1, f"still locked ({lock_state(sim)})")
        r.check(not stick_arm(sim, plain), "the sticks do not arm the drone")
        # The known weakness: without the key anyone can turn the lock off.
        plain.conn.mav.param_set_send(1, 1, b"VLCK_ENABLE", 0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        fly_with(sim, 3, plain)
        r.check(sim.get_param("VLCK_ENABLE") == 0, "a ground station without a key turned the lock off (why the key is needed)")
        r.check(stick_arm(sim, plain), "and then the sticks arm it")
        disarm(sim, plain)
        common_checks(sim, r)
    finally:
        if plain is not None:
            plain.close()
        sim.close()


def settings_need_the_key(version: str, r: Report) -> None:
    """With the key on the drone, a ground station without it cannot turn the lock off."""
    sim = start(version)
    plain = None
    try:
        give_key(sim)
        plain = Station(OTHER_PORT)
        for _ in range(3):
            plain.conn.mav.param_set_send(1, 1, b"VLCK_ENABLE", 0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            fly_with(sim, 2, plain)
        r.check(sim.get_param("VLCK_ENABLE") == 1, "VLCK_ENABLE is still 1")
        r.check(not stick_arm(sim, plain), "and the sticks do not arm the drone")
        common_checks(sim, r)
    finally:
        if plain is not None:
            plain.close()
        sim.close()


def flight_is_never_disturbed(version: str, r: Report) -> None:
    """Once armed the lock does nothing: losing the ground station in flight changes nothing."""
    sim = start(version)
    station = None
    try:
        give_key(sim)
        station = Station(BOUND_PORT, key=KEY)
        fly_with(sim, 6, station)
        sim.set_mode("GUIDED")
        r.check(command_arm(sim, station), "armed with the bound station connected")
        sim.mav.mav.command_long_send(sim.mav.target_system, sim.mav.target_component,
                                      mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 15)
        fly_with(sim, 40, station, until=lambda: sim.alt >= 14)
        r.check(sim.alt >= 14, f"took off ({sim.alt:.1f} m)")
        lost = sim.sim_s
        modes_before = len(sim.mode_log)
        sim.fly(30)   # no heartbeats from the station
        r.check(sim.armed and sim.mode == "GUIDED" and len(sim.mode_log) == modes_before,
                f"still armed in GUIDED after 30 s without the station ({sim.mode})")
        r.check(not lock_texts(sim, lost), f"no lock text in flight {lock_texts(sim, lost)}")
        r.check(sim.alt >= 13, f"still at its height ({sim.alt:.1f} m)")
        sim.set_mode("LAND")
        sim.fly(90, until=lambda: not sim.armed)
        r.check(not sim.armed, "landed and disarmed by itself")
        sim.fly(3)
        r.check(sim.saw("locked, no signed ground station", lost), "locked after the landing")
        common_checks(sim, r)
    finally:
        if station is not None:
            station.close()
        sim.close()


def known_limits(version: str, r: Report) -> None:
    """What the lock cannot stop: a forced arm (needs USB or the key), and arming with all checks off. It names the second."""
    sim = start(version)
    try:
        give_key(sim)
        sim.fly(2)
        r.check(command_arm(sim, force=True), "a forced arm over USB arms the drone (known limit)")
        disarm(sim)
        name, value = SOME_CHECKS_OFF[version[:3]]
        sim.set_param(name, value)
        sim.fly(3)
        r.check(not stick_arm(sim), f"with only some checks off ({name} {value}) the lock still stops the sticks")
        name, value = CHECKS_OFF[version[:3]]
        off_since = sim.sim_s
        sim.set_param(name, value)
        sim.fly(3)
        r.check(sim.saw(CHECKS_OFF_TEXT, off_since), f"it says that the checks are off ({name} {value})")
        r.check(stick_arm(sim), "with all checks off the sticks arm it (known limit)")
        disarm(sim)
        common_checks(sim, r, checks_off_since=off_since)
    finally:
        sim.close()


def busy_link_keeps_it_unlocked(version: str, r: Report) -> None:
    """15 other heartbeats a second before the ground station's on the same link: it stays unlocked."""
    sim = start(version)
    station = None
    try:
        give_key(sim)
        station = Station(BOUND_PORT, key=KEY)
        fly_with(sim, 4, station)
        r.check(lock_state(sim) == 2, f"unlocked by the bound station ({lock_state(sim)})")
        since = sim.sim_s
        sim.fly(25, each_second=lambda: station.beat_after_others(15))
        locked = sim.texts_with("locked, no signed", since)
        r.check(not locked and lock_state(sim) == 2, f"still unlocked after 25 s of a busy link {locked[:1]}")
        common_checks(sim, r)
    finally:
        if station is not None:
            station.close()
        sim.close()


CASES = [off_by_default, locked_without_a_station, bound_station_unlocks, others_stay_locked,
         no_key_on_the_drone, settings_need_the_key, flight_is_never_disturbed, known_limits,
         busy_link_keeps_it_unlocked]


def main(argv: list[str]) -> int:
    global SCRIPT
    versions = [a for a in argv if re.fullmatch(r"\d+\.\d+\.\d+", a)] or VERSIONS
    only = argv[argv.index("-k") + 1] if "-k" in argv else ""
    if "--script" in argv:
        SCRIPT = pathlib.Path(argv[argv.index("--script") + 1]).resolve()
    cases = [c for c in CASES if only in c.__name__]
    failed_cases = 0
    began = time.monotonic()
    print(f"Script under test: {SCRIPT}")
    for version in versions:
        print(f"\n===== ArduCopter {version}")
        for case in cases:
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
    total = len(cases) * len(versions)
    print(f"\n{total - failed_cases} of {total} cases passed in {time.monotonic() - began:.0f} s")
    return 1 if failed_cases else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
