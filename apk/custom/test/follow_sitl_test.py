"""Fly Follow Me in the ArduCopter simulator, with the app's own code.

The guide (src/Skydroid/FollowGuide.cc) is built into a DLL and gets what it gets in the
app: the drone's telemetry, and positions of an object. Here the object is made up (a
walker, a runner, a car), its positions are a little late and noisy like those from the
video, and the speeds the guide answers with go to the simulated drone exactly as the
app sends them.

Usage:
    py follow_sitl_test.py                 all cases on ArduCopter 4.6.2 and 4.7.0
    py follow_sitl_test.py 4.7.0           one version
    py follow_sitl_test.py -k walk         only the cases with "walk" in the name
    py follow_sitl_test.py --source DIR    the guide from another copy of src/Skydroid
    py follow_sitl_test.py --trace         every case second by second

Nothing leaves this computer: the simulator runs in WSL (see drone/README.md for the
set-up) and the test talks to it on localhost.
"""
from __future__ import annotations

import ctypes
import hashlib
import math
import os
import pathlib
import random
import subprocess
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[2] / "drone" / "test"))

from pymavlink import mavutil  # noqa: E402
from sitl_session import Sitl  # noqa: E402

VERSIONS = ["4.6.2", "4.7.0"]
HOME = (19.8816785, 73.8199197, 600.0, 45.0)  # lat, lon, height above sea, heading
EARTH_M = 6371000.0
GPP = pathlib.Path(r"C:\Qt\Tools\mingw1310_64\bin\g++.exe")

# ArduCopter's limits as shipped, which the cases count on.
GUIDED_TIMEOUT_S = 3.0
# The simulated quadcopter leans 30 degrees at the most and holds its place in up to 10 m/s of wind.
HOLDABLE_WIND = 8
TOO_MUCH_WIND = 22

OFF, FOLLOWING, WAITING = 0, 1, 2


def build_guide(source: pathlib.Path) -> ctypes.CDLL:
    """The guide as a DLL. One DLL for each state of the sources, so a changed copy is never mixed up with another."""
    files = [HERE / "follow_guide_c.cc", source / "FollowGuide.cc", source / "FollowGuide.h"]
    stamp = hashlib.sha256(b"".join(f.read_bytes() for f in files)).hexdigest()[:12]
    out = pathlib.Path(os.environ["TEMP"]) / "vama-apk-tests"
    out.mkdir(exist_ok=True)
    dll = out / f"follow_guide_{stamp}.dll"
    if not dll.exists():
        env = dict(os.environ, PATH=str(GPP.parent) + os.pathsep + os.environ["PATH"])
        done = subprocess.run([str(GPP), "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror", "-shared", "-static",
                               "-I", str(source), str(files[0]), str(files[1]), "-o", str(dll)],
                              capture_output=True, text=True, env=env)
        if done.returncode != 0:
            raise SystemExit("the guide did not compile:\n" + done.stderr)
    lib = ctypes.CDLL(str(dll))
    d, i, p = ctypes.c_double, ctypes.c_int, ctypes.c_void_p
    lib.fg_new.restype = p
    lib.fg_free.argtypes = [p]
    lib.fg_set_limits.argtypes = [p, d, d]
    lib.fg_drone.argtypes = [p, d, d, d, i, d, d, i, d, i, i, i, i, d]
    lib.fg_fix.argtypes = [p, d, d, d, i, d, d]
    lib.fg_forget_object.argtypes = [p]
    lib.fg_start.argtypes = [p, i, d]
    lib.fg_start.restype = i
    lib.fg_stop.argtypes = [p, d]
    lib.fg_tick.argtypes = [p, i, i, d, ctypes.POINTER(d), ctypes.POINTER(d), ctypes.POINTER(d)]
    lib.fg_tick.restype = i
    lib.fg_state.argtypes = [p]
    lib.fg_why.argtypes = [p]
    lib.fg_why_name.argtypes = [i]
    lib.fg_why_name.restype = ctypes.c_char_p
    lib.fg_object_known.argtypes = [p, d]
    lib.fg_object_from_drone.argtypes = [p, d, ctypes.POINTER(d), ctypes.POINTER(d)]
    for name in ("fg_distance", "fg_place_sigma"):
        getattr(lib, name).argtypes = [p, d]
        getattr(lib, name).restype = d
    for name in ("fg_kept_distance", "fg_off_asked"):
        getattr(lib, name).argtypes = [p]
        getattr(lib, name).restype = d
    lib.fg_constant.argtypes = [ctypes.c_char_p]
    lib.fg_constant.restype = d
    return lib


class Guide:
    def __init__(self, lib: ctypes.CDLL, speed_limit: float = 8.0, range_limit: float = 500.0) -> None:
        self.lib = lib
        self.h = lib.fg_new()
        lib.fg_set_limits(self.h, speed_limit, range_limit)

    def close(self) -> None:
        self.lib.fg_free(self.h)

    def constant(self, name: str) -> float:
        value = self.lib.fg_constant(name.encode())
        if math.isnan(value):
            raise KeyError(name)
        return value

    def why(self) -> str:
        return self.lib.fg_why_name(self.lib.fg_why(self.h)).decode()

    def state(self) -> int:
        return self.lib.fg_state(self.h)


def to_geo(north: float, east: float) -> tuple[float, float]:
    lat = HOME[0] + math.degrees(north / EARTH_M)
    lon = HOME[1] + math.degrees(east / (EARTH_M * math.cos(math.radians(HOME[0]))))
    return lat, lon


def to_local(lat: float, lon: float) -> tuple[float, float]:
    north = math.radians(lat - HOME[0]) * EARTH_M
    east = math.radians(lon - HOME[1]) * EARTH_M * math.cos(math.radians(HOME[0]))
    return north, east


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


class Flight:
    """One case: an object, the guide, and the simulated drone, for some simulated seconds."""

    def __init__(self, sim: Sitl, lib: ctypes.CDLL, obj, *, speed_limit: float = 8.0, range_limit: float = 500.0,
                 seed: int = 7, fix_sigma: float = 1.2, distance: float = 30.0, start_after: float = 6.0,
                 position_hz: float = 5.0, telemetry_late: float = 0.15, command_late: float = 0.10,
                 trace: bool = False) -> None:
        self.sim = sim
        self.lib = lib
        self.guide = Guide(lib, speed_limit, range_limit)
        self.random = random.Random(seed)
        self.fix_sigma = fix_sigma
        self.trace = trace
        self.mask = int(self.guide.constant("kSpeedTypeMask"))
        self.mask_no_heading = int(self.guide.constant("kSpeedTypeMaskNoHeading"))
        self.frame = int(self.guide.constant("kSpeedFrame"))
        self.guided = int(self.guide.constant("kCopterGuided"))
        self.holding = {int(self.guide.constant(n)) for n in ("kCopterLoiter", "kCopterPosHold", "kCopterBrake")}
        self.tick_s = self.guide.constant("kTickS")
        self.brake_s = self.guide.constant("kBrakeS")
        # The radio link: telemetry reaches the app this late, and a command the drone this late.
        self.telemetry_late = telemetry_late
        self.command_late = command_late
        self.telemetry: list[tuple[float, tuple]] = []
        self.commands: list[tuple[float, float, float, float]] = []
        # The clock: the simulator's own, from its messages.
        self.now = 0.0
        self.t0 = None             # the case starts here: times below count from it
        self.start_after = start_after
        # The drone as its telemetry says.
        self.custom_mode = -1
        self.armed = False
        self.flying = True
        self.gps3d = False
        self.yaw = 0.0
        self.north = self.east = self.height = 0.0
        self.vn = self.ve = 0.0
        self.have_position = False
        # The object: where it is at a time (seconds of the case), metres north and east of where the
        # drone is when the case starts, `distance` metres further north.
        self.path = obj
        self.distance = distance
        self.origin = (0.0, 0.0)
        # What happens on the way (set by the case).
        self.seen = lambda t: True
        self.lock_active = lambda t: True
        self.fixes_come = lambda t: True
        self.app_alive = lambda t: True
        self.at: list[tuple[float, object]] = []     # (time, what to do then)
        self.stop_by_operator_at = -1.0
        # What is recorded.
        self.started = False
        self.start_why = ""
        self.ended_at = None
        self.end_why = ""
        self.kept = 0.0
        self.true_offset = (0.0, 0.0)
        self.errors: list[tuple[float, float]] = []   # (time, metres off its place)
        self.least_distance = 1e9
        self.least_distance_late = 1e9
        self.most_asked = 0.0
        self.most_flown = 0.0
        self.most_closing_inside = 0.0
        self.own_closing = 0.0
        self.distance_before = -1.0
        self.side_changes = 0
        self.side_before = 0
        self.near_s = 0.0
        self.waited_s = 0.0
        self.most_off_asked = 0.0
        self.yaw_at_start = None
        self.most_yaw_change = 0.0
        self.yaw_change_now = 0.0
        self.most_lean = 0.0
        self.height_at_start = None
        self.most_height_change = 0.0
        self.sends: list[float] = []                  # times of every speed sent
        self.at_end = None                            # (north, east) at the end
        self.speed_log: list[tuple[float, float]] = []  # (time, the drone's speed) after the end
        self.mode_at_end = ""
        self.next_fix = None
        self.next_tick = None
        self.next_trace = 0.0
        self.message_rate(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 25.0)
        self.message_rate(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, position_hz)

    # --- the simulator ----------------------------------------------------
    def message_rate(self, message_id: int, hz: float) -> None:
        self.sim.mav.mav.command_long_send(self.sim.mav.target_system, self.sim.mav.target_component,
                                           mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, message_id,
                                           1e6 / hz, 0, 0, 0, 0, 0)

    def object_at(self, t: float) -> tuple[float, float]:
        n, e = self.path(max(0.0, t))
        return self.origin[0] + self.distance + n, self.origin[1] + e

    def _take(self, msg) -> None:
        self.sim._take(msg)
        kind = msg.get_type()
        if kind == "ATTITUDE":
            self.now = max(self.now, msg.time_boot_ms / 1000.0)
            self.yaw = math.degrees(msg.yaw)
        elif kind == "HEARTBEAT" and msg.get_srcComponent() == 1:
            self.custom_mode = msg.custom_mode
            self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        elif kind == "EXTENDED_SYS_STATE":
            self.flying = msg.landed_state in (mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,
                                               mavutil.mavlink.MAV_LANDED_STATE_TAKEOFF,
                                               mavutil.mavlink.MAV_LANDED_STATE_LANDING)
        elif kind == "GPS_RAW_INT":
            self.gps3d = msg.fix_type >= 3
        elif kind == "GLOBAL_POSITION_INT":
            self.now = max(self.now, msg.time_boot_ms / 1000.0)
            self.north, self.east = to_local(msg.lat / 1e7, msg.lon / 1e7)
            self.height = msg.relative_alt / 1000.0
            self.vn, self.ve = msg.vx / 100.0, msg.vy / 100.0
            self.have_position = True
            if self.t0 is not None:
                mode = 0 if self.custom_mode == self.guided else (1 if self.custom_mode in self.holding else 2)
                # hdg: hundredths of a degree, 65535 when the drone does not know it.
                self.telemetry.append((self.now + self.telemetry_late,
                                       (msg.lat / 1e7, msg.lon / 1e7, self.height, 1, self.vn, self.ve,
                                        int(msg.hdg != 65535), msg.hdg / 100.0,
                                        int(self.armed), int(self.flying), int(self.gps3d), mode)))

    def _radio(self) -> None:
        """What is on its way: telemetry to the app, commands to the drone."""
        while self.telemetry and self.telemetry[0][0] <= self.now:
            values = self.telemetry.pop(0)[1]
            if self.app_alive(self.now - self.t0):
                self.lib.fg_drone(self.guide.h, *values, self.now)
        while self.commands and self.commands[0][0] <= self.now:
            _, north, east, heading = self.commands.pop(0)
            # As VehicleFollowDrone sends it: the heading to keep as a yaw angle from north,
            # or, when the guide has none, a yaw rate of 0.
            if math.isnan(heading):
                mask, yaw = self.mask_no_heading, 0.0
            else:
                mask, yaw = self.mask, math.radians((heading + 180.0) % 360.0 - 180.0)
            self.sim.mav.mav.set_position_target_local_ned_send(
                0, self.sim.mav.target_system, self.sim.mav.target_component, self.frame, mask,
                0, 0, 0, north, east, 0, 0, 0, 0, yaw, 0)

    def send_speed(self, north: float, east: float, heading: float) -> None:
        self.commands.append((self.now + self.command_late, north, east, heading))
        self.sends.append(self.now - self.t0)

    def ask_guided(self) -> None:
        # What QGC's Vehicle::setFlightMode sends for ArduPilot.
        self.sim.mav.mav.command_long_send(self.sim.mav.target_system, self.sim.mav.target_component,
                                           mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                           mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, self.guided, 0, 0, 0, 0, 0)

    # --- the app's side -----------------------------------------------------
    def _fix(self, t: float) -> None:
        if not (self.lock_active(t) and self.seen(t) and self.fixes_come(t)):
            return
        n, e = self.object_at(t - 0.35)       # how old a picture really is
        n += self.random.gauss(0.0, self.fix_sigma)
        e += self.random.gauss(0.0, self.fix_sigma)
        lat, lon = to_geo(n, e)
        self.lib.fg_fix(self.guide.h, lat, lon, max(1.5, self.fix_sigma), 0, 0.0, self.now - 0.30)

    def _tick(self, t: float) -> None:
        guide = self.guide
        if not self.started and t >= self.start_after:
            self.started = True
            why = self.lib.fg_start(guide.h, int(self.lock_active(t) and self.seen(t)), self.now)
            self.start_why = self.lib.fg_why_name(why).decode()
            if why == 0:
                self.kept = self.lib.fg_kept_distance(guide.h)
                on, oe = self.object_at(t)
                self.true_offset = (self.north - on, self.east - oe)
                self.yaw_at_start = self.yaw
                self.height_at_start = self.height
                if self.custom_mode != self.guided:
                    self.ask_guided()
        if self.started and 0.0 <= self.stop_by_operator_at <= t and guide.state() != OFF:
            self.lib.fg_stop(guide.h, self.now)
            self._ended(t)
        was = guide.state() != OFF
        north, east, heading = ctypes.c_double(0.0), ctypes.c_double(0.0), ctypes.c_double(0.0)
        send = self.lib.fg_tick(guide.h, int(self.lock_active(t)), int(self.seen(t)), self.now,
                                ctypes.byref(north), ctypes.byref(east), ctypes.byref(heading))
        if send:
            self.send_speed(north.value, east.value, heading.value)
            self.most_asked = max(self.most_asked, math.hypot(north.value, east.value))
        if was and guide.state() == OFF and self.ended_at is None:
            self._ended(t)
        if guide.state() == WAITING:
            self.waited_s += self.tick_s
        if guide.state() == OFF:
            return
        on, oe = self.object_at(t)
        distance = math.hypot(on - self.north, oe - self.east)
        self.least_distance = min(self.least_distance, distance)
        self.most_flown = max(self.most_flown, math.hypot(self.vn, self.ve))
        self.most_off_asked = max(self.most_off_asked, self.lib.fg_off_asked(guide.h))
        min_distance = guide.constant("kMinDistanceM")
        if send and min_distance - 3.0 > distance > 1e-6 and 0.0 <= self.distance_before and distance < self.distance_before:
            closing = (north.value * (on - self.north) + east.value * (oe - self.east)) / distance
            self.own_closing += max(0.0, closing) * self.tick_s
        self.distance_before = distance
        # The rule itself, by the guide's own estimate: no speed towards the object inside the least distance.
        seen_n, seen_e = ctypes.c_double(0.0), ctypes.c_double(0.0)
        if send and self.lib.fg_object_from_drone(guide.h, self.now, ctypes.byref(seen_n), ctypes.byref(seen_e)):
            seen_distance = math.hypot(seen_n.value, seen_e.value)
            if 1e-6 < seen_distance < min_distance:
                self.most_closing_inside = max(self.most_closing_inside,
                                               (north.value * seen_n.value + east.value * seen_e.value) / seen_distance)
        side = 1 if self.north - on > 1.0 else (-1 if self.north - on < -1.0 else 0)
        if side:
            self.side_changes += 1 if (self.side_before and side != self.side_before) else 0
            self.side_before = side
        if guide.why() == "Near":
            self.near_s += self.tick_s
        if self.yaw_at_start is not None:
            turned = abs((self.yaw - self.yaw_at_start + 180.0) % 360.0 - 180.0)
            self.most_yaw_change = max(self.most_yaw_change, turned)
            self.yaw_change_now = turned
            self.most_lean = max(self.most_lean, self.sim.lean_deg)
            self.most_height_change = max(self.most_height_change, abs(self.height - self.height_at_start))
        if guide.state() == FOLLOWING and t > self.start_after + 10.0:
            self.errors.append((t, math.hypot(self.north - on - self.true_offset[0], self.east - oe - self.true_offset[1])))

    def _ended(self, t: float) -> None:
        self.ended_at = t
        self.end_why = self.guide.why()
        self.at_end = (self.north, self.east)

    def run(self, seconds: float) -> "Flight":
        sim = self.sim
        deadline = time.monotonic() + seconds / sim.speedup * 4 + 60
        # Wait for the first telemetry of this case, then start its clock.
        self.have_position = False
        while not self.have_position and time.monotonic() < deadline:
            msg = sim.mav.recv_match(blocking=True, timeout=0.2)
            if msg is not None:
                self._take(msg)
        self.t0 = self.now
        self.origin = (self.north, self.east)
        self.next_fix = 0.05
        self.next_tick = 0.1
        events = sorted(self.at, key=lambda item: item[0])
        while self.now - self.t0 < seconds and time.monotonic() < deadline:
            msg = sim.mav.recv_match(blocking=True, timeout=0.2)
            if msg is None:
                continue
            self._take(msg)
            self._radio()
            t = self.now - self.t0
            while events and t >= events[0][0]:
                events.pop(0)[1]()
            alive = self.app_alive(t)
            while t >= self.next_fix:
                self.next_fix += 0.2
                if alive:
                    self._fix(t)
            while t >= self.next_tick:
                self.next_tick += self.tick_s
                if alive:
                    self._tick(t)
            if self.ended_at is not None or not alive:
                self.speed_log.append((t, math.hypot(self.vn, self.ve)))
            if t > seconds - 10.0 and self.guide.state() != OFF:
                on, oe = self.object_at(t)
                self.least_distance_late = min(self.least_distance_late, math.hypot(on - self.north, oe - self.east))
            if self.trace and t >= self.next_trace:
                self.next_trace += 1.0
                on, oe = self.object_at(t)
                print(f"      {t:6.1f} s  object {on - self.origin[0]:7.1f} {oe - self.origin[1]:7.1f}"
                      f"  drone {self.north - self.origin[0]:7.1f} {self.east - self.origin[1]:7.1f}"
                      f"  speed {self.vn:5.2f} {self.ve:5.2f}  yaw {self.yaw:6.1f}  height {self.height:5.1f}"
                      f"  {sim.mode}  guide {self.guide.state()} {self.guide.why()}"
                      f"  seen distance {self.lib.fg_distance(self.guide.h, self.now):5.1f}")
        self.mode_at_end = sim.mode
        self.following_at_end = self.guide.state() != OFF
        self.end_distance = None
        if self.have_position:
            on, oe = self.object_at(self.now - self.t0)
            self.end_offset = (self.north - on, self.east - oe)
            self.end_distance = math.hypot(*self.end_offset)
        return self

    # --- what came of it ------------------------------------------------------
    def error_stats(self, last_seconds: float | None = None) -> tuple[float, float]:
        """(95 %, most) of how far the drone was off its place."""
        values = [e for t, e in self.errors if last_seconds is None or t > self.errors[-1][0] - last_seconds]
        if not values:
            return 0.0, 0.0
        values.sort()
        return values[int(0.95 * (len(values) - 1))], values[-1]

    def moved_after_end(self) -> float:
        if self.at_end is None:
            return 0.0
        return math.hypot(self.north - self.at_end[0], self.east - self.at_end[1])

    def sends_after(self, t: float) -> int:
        return sum(1 for s in self.sends if s > t)

    def standing_after(self, since: float) -> float | None:
        """How long after `since` the drone's speed was under 0.3 m/s."""
        return next((t - since for t, speed in self.speed_log if t >= since and speed < 0.3), None)

    def close(self) -> None:
        self.guide.close()


# --- what the object does ---------------------------------------------------------

def stands(t):
    return 0.0, 0.0


def walks_east(mps, from_s=8.0):
    return lambda t: (0.0, mps * max(0.0, t - from_s))


def walks_square(mps, side_s, from_s=8.0):
    def path(t):
        left = max(0.0, t - from_s)
        n = e = 0.0
        side = 0
        while left > 0.0:
            s = min(left, side_s)
            n += (0.0, 1.0, 0.0, -1.0)[side] * mps * s
            e += (1.0, 0.0, -1.0, 0.0)[side] * mps * s
            left -= s
            side = (side + 1) % 4
        return n, e
    return path


def runs_a_circle(mps, radius, from_s=8.0):
    def path(t):
        angle = mps * max(0.0, t - from_s) / radius
        return radius * math.sin(angle), radius * (1.0 - math.cos(angle))
    return path


def drives_off(mps, accel, from_s=8.0):
    def path(t):
        since = max(0.0, t - from_s)
        ramp = mps / accel
        if since < ramp:
            return 0.0, 0.5 * accel * since * since
        return 0.0, 0.5 * accel * ramp * ramp + mps * (since - ramp)
    return path


def comes_at_the_drone(mps, from_s=10.0):
    return lambda t: (-mps * max(0.0, t - from_s), 0.0)


def passes_the_drone(mps, fast_s, slow_mps, from_s=10.0):
    def path(t):
        since = max(0.0, t - from_s)
        return -(mps * min(since, fast_s) + slow_mps * max(0.0, since - fast_s)), 0.0
    return path


# --- the cases ------------------------------------------------------------------------

def case_walk(sim, lib, opts):
    r = Report("walk: a walker going straight (1.4 m/s)")
    f = Flight(sim, lib, walks_east(1.4), **opts).run(100.0)
    p95, most = f.error_stats()
    r.check(f.start_why == "None" and f.following_at_end, f"Follow starts out of Loiter and goes on ({f.start_why}, {f.end_why})")
    r.check(abs(f.kept - 30.0) < 3.0, f"it keeps the distance of the start: {f.kept:.1f} m of 30")
    r.check(p95 < 4.0 and most < 6.0, f"the drone keeps its place: 95 % within {p95:.1f} m, most {most:.1f} m")
    r.check(f.most_yaw_change < 3.0, f"its nose does not turn: {f.most_yaw_change:.1f} degrees at the most")
    r.check(f.most_height_change < 1.0, f"its height does not change: {f.most_height_change:.2f} m at the most")
    r.check(f.most_asked < 3.6, f"no speed far above the walker's own: {f.most_asked:.1f} m/s asked at the most")
    r.note(f"the drone's speed was off the speed asked by {f.most_off_asked:.2f} m/s at the most")
    f.close()
    return r


def case_square(sim, lib, opts):
    r = Report("square: a walker who turns corners")
    f = Flight(sim, lib, walks_square(1.4, 20.0), **opts).run(120.0)
    p95, most = f.error_stats()
    r.check(f.following_at_end and p95 < 8.0 and most < 10.5, f"the drone stays with the walker: 95 % within {p95:.1f} m, most {most:.1f} m")
    r.check(f.most_yaw_change < 3.0 and f.most_height_change < 1.0,
            f"nose and height stay: {f.most_yaw_change:.1f} degrees, {f.most_height_change:.2f} m")
    r.note(f"off the speed asked by {f.most_off_asked:.2f} m/s at the most")
    f.close()
    return r


def case_square_slow_telemetry(sim, lib, opts):
    r = Report("square, telemetry 3 times a second (QGC's own rate)")
    f = Flight(sim, lib, walks_square(1.4, 20.0), **dict(opts, position_hz=3.0)).run(120.0)
    p95, most = f.error_stats()
    r.check(f.following_at_end and p95 < 8.5 and most < 11.0, f"the drone stays with the walker: 95 % within {p95:.1f} m, most {most:.1f} m")
    f.close()
    return r


def case_circle(sim, lib, opts):
    r = Report("circle: a runner on a curve (4 m/s)")
    f = Flight(sim, lib, runs_a_circle(4.0, 40.0), **opts).run(110.0)
    p95, most = f.error_stats()
    r.check(f.following_at_end and p95 < 10.0 and most < 12.0, f"the drone stays with the runner: 95 % within {p95:.1f} m, most {most:.1f} m")
    r.check(f.most_yaw_change < 3.0, f"its nose does not turn on the curve: {f.most_yaw_change:.1f} degrees at the most")
    r.note(f"off the speed asked by {f.most_off_asked:.2f} m/s at the most")
    f.close()
    return r


def case_car(sim, lib, opts):
    r = Report("car: drives off to 10 m/s, speed limit 12")
    f = Flight(sim, lib, drives_off(10.0, 1.5), **dict(opts, speed_limit=12.0, range_limit=2000.0)).run(100.0)
    p95, most = f.error_stats()
    late95, _ = f.error_stats(last_seconds=30.0)
    r.check(f.following_at_end, f"Follow goes on ({f.end_why}): a drone that obeys is not taken for one that does not")
    r.check(most < 40.0 and late95 < 6.0, f"the drone falls {most:.1f} m behind while the car speeds up, and is {late95:.1f} m off afterwards")
    r.check(f.most_asked <= 12.0 + 1e-6, f"never above the speed limit: {f.most_asked:.2f} m/s asked")
    r.check(f.most_height_change < 1.5, f"its height stays at this speed: {f.most_height_change:.2f} m")
    # A drone that leans hard reads a few degrees of yaw off for as long as it leans. (Without the
    # heading in the commands the nose turned 45 degrees here.)
    r.check(f.most_yaw_change < 6.0,
            f"its nose is not turned: {f.most_yaw_change:.1f} degrees off at the most, {f.yaw_change_now:.1f} at the end, "
            f"while it leans up to {f.most_lean:.0f} degrees")
    r.note(f"off the speed asked by {f.most_off_asked:.2f} m/s at the most; flew up to {f.most_flown:.1f} m/s")
    f.close()
    return r


def case_range(sim, lib, opts):
    r = Report("range limit 60 m")
    f = Flight(sim, lib, walks_east(1.4), **dict(opts, range_limit=60.0)).run(110.0)
    r.check(f.end_why == "TooFar", f"Follow ends at the range limit ({f.end_why} at {f.ended_at} s)")
    if f.at_end:
        gone = math.hypot(f.at_end[0] - f.origin[0], f.at_end[1] - f.origin[1])
        r.check(58.0 < gone < 66.0, f"{gone:.1f} m from where it started")
    r.check(f.moved_after_end() < 4.0, f"the drone stops there: {f.moved_after_end():.1f} m more")
    r.check(f.mode_at_end == "GUIDED", f"and holds its place in Guided ({f.mode_at_end})")
    f.close()
    return r


def case_comes_at(sim, lib, opts):
    r = Report("a walker comes at the drone")
    f = Flight(sim, lib, comes_at_the_drone(1.4), **opts).run(80.0)
    r.check(f.following_at_end and f.least_distance > 23.0, f"the drone backs away and keeps its distance: {f.least_distance:.1f} m at the least")
    f.close()
    return r


def case_passes(sim, lib, opts):
    r = Report("a runner passes under the drone (7 m/s, the drone may fly 5), then walks")
    f = Flight(sim, lib, passes_the_drone(7.0, 12.0, 1.4), **dict(opts, speed_limit=5.0, distance=25.0)).run(100.0)
    r.check(f.start_why == "None" and f.least_distance < 8.0, f"(the runner does get under the drone: {f.least_distance:.1f} m)")
    r.check(f.following_at_end and f.side_changes == 1, f"Follow goes on, and the drone never flies over the runner to get back to its side ({f.side_changes} changes of side)")
    r.check(f.most_closing_inside <= 1e-9 and f.own_closing <= 0.8,
            f"no speed towards the runner inside 15 m: {f.most_closing_inside:.3f} m/s as the guide sees it, {f.own_closing:.2f} m in truth")
    r.check(f.near_s > 1.0, f"the guide says the object is near for {f.near_s:.1f} s")
    if f.end_distance is not None:
        # The runner goes south: behind them is north of them.
        r.check(f.end_offset[0] > 15.0 and abs(f.end_offset[1]) < 8.0 and 20.0 < f.least_distance_late and f.end_distance < 31.0,
                f"at the end it follows from behind at its distance: {f.end_offset[0]:.1f} m behind, {f.end_offset[1]:.1f} m beside")
    r.check(f.most_yaw_change < 3.0, f"its nose does not turn: {f.most_yaw_change:.1f} degrees")
    f.close()
    return r


def case_hidden(sim, lib, opts):
    r = Report("hidden for 1.5 s, twice")
    f = Flight(sim, lib, walks_east(1.4), **opts)
    f.seen = lambda t: not (30.0 < t < 31.5 or 60.0 < t < 61.5)
    f.run(100.0)
    p95, most = f.error_stats()
    r.check(f.following_at_end and 2.0 < f.waited_s < 6.0, f"it waits while the object is hidden ({f.waited_s:.1f} s) and goes on")
    r.check(p95 < 5.5, f"and catches up: 95 % within {p95:.1f} m")
    f.close()
    return r


def case_lock_ends(sim, lib, opts):
    r = Report("the lock ends")
    f = Flight(sim, lib, walks_east(3.0), **opts)
    f.lock_active = lambda t: t < 40.0
    f.run(60.0)
    r.check(f.end_why == "LockEnded" and abs(f.ended_at - 40.0) < 0.5, f"Follow ends at once ({f.end_why} at {f.ended_at:.1f} s)")
    still = f.standing_after(40.0)
    r.check(still is not None and still < 5.0 and f.moved_after_end() < 6.0,
            f"the drone stops: standing after {still} s, {f.moved_after_end():.1f} m further")
    r.check(f.sends_after(40.0 + f.brake_s + 0.4) == 0 and f.sends_after(40.0) >= 5,
            f"\"stop\" is sent for {f.brake_s} s ({f.sends_after(40.0)} times), then nothing more")
    r.check(f.mode_at_end == "GUIDED", f"it holds its place in Guided ({f.mode_at_end})")
    f.close()
    return r


def case_operator_stops(sim, lib, opts):
    r = Report("the operator stops Follow")
    f = Flight(sim, lib, walks_east(3.0), **opts)
    f.stop_by_operator_at = 40.0
    f.run(55.0)
    still = f.standing_after(40.0)
    r.check(f.end_why == "Stopped" and still is not None and still < 5.0 and f.moved_after_end() < 6.0,
            f"the drone stops ({f.end_why}): standing after {still} s, {f.moved_after_end():.1f} m further")
    f.close()
    return r


def case_app_dies(sim, lib, opts):
    r = Report("the app goes silent while a runner is followed (4 m/s)")
    f = Flight(sim, lib, walks_east(4.0), **opts)
    f.app_alive = lambda t: t < 40.0
    state = {}
    f.at.append((39.9, lambda: state.update(n=f.north, e=f.east)))
    f.run(60.0)
    still = f.standing_after(40.0)
    moved = math.hypot(f.north - state["n"], f.east - state["e"])
    r.check(still is not None and still < GUIDED_TIMEOUT_S + 4.0, f"the drone stops by itself: standing {still} s after the last command")
    r.check(moved < 4.0 * GUIDED_TIMEOUT_S + 10.0, f"after flying on for {moved:.1f} m (GUID_TIMEOUT is {GUIDED_TIMEOUT_S} s)")
    r.check(f.mode_at_end == "GUIDED", f"it holds its place in Guided ({f.mode_at_end})")
    f.close()
    return r


def case_pilot_loiter(sim, lib, opts):
    r = Report("the pilot switches to Loiter")
    f = Flight(sim, lib, walks_east(3.0), **opts)
    f.at.append((40.0, lambda: sim.request_mode("LOITER")))
    f.run(55.0)
    r.check(f.end_why == "PilotTookOver" and f.ended_at < 41.5, f"Follow ends within a second ({f.end_why} at {f.ended_at} s)")
    r.check(f.sends_after(f.ended_at or 0.0) == 0, f"and sends nothing at all afterwards ({f.sends_after(f.ended_at or 0.0)} messages)")
    r.check(f.mode_at_end == "LOITER", f"the drone stays in the pilot's mode ({f.mode_at_end})")
    f.close()
    return r


def case_pilot_rtl(sim, lib, opts):
    r = Report("RTL while following")
    f = Flight(sim, lib, walks_east(3.0), **opts)
    f.at.append((40.0, lambda: sim.request_mode("RTL")))
    f.run(50.0)
    r.check(f.end_why == "PilotTookOver" and f.sends_after(f.ended_at or 0.0) == 0,
            f"Follow ends and sends nothing ({f.end_why}, {f.sends_after(f.ended_at or 0.0)} messages)")
    r.check(f.mode_at_end == "RTL", f"the drone flies home ({f.mode_at_end})")
    f.close()
    return r


def case_wind_ok(sim, lib, opts):
    r = Report(f"walk in {HOLDABLE_WIND} m/s of wind")
    sim.set_param("SIM_WIND_SPD", HOLDABLE_WIND)
    try:
        f = Flight(sim, lib, walks_square(1.4, 20.0), **opts).run(110.0)
    finally:
        sim.set_param("SIM_WIND_SPD", 0)
    p95, most = f.error_stats()
    r.check(f.following_at_end and p95 < 8.5, f"the drone stays with the walker ({f.end_why}): 95 % within {p95:.1f} m, most {most:.1f} m")
    # Leaning into the wind, the drone reads a few degrees of yaw off for as long as it leans.
    r.check(f.most_yaw_change < 6.0,
            f"its nose is not turned in the wind: {f.most_yaw_change:.1f} degrees off at the most, {f.yaw_change_now:.1f} at the end, "
            f"while it leans up to {f.most_lean:.0f} degrees")
    r.note(f"off the speed asked by {f.most_off_asked:.2f} m/s at the most")
    f.close()
    return r


def case_wind_too_much(sim, lib, opts):
    r = Report(f"wind of {TOO_MUCH_WIND} m/s comes up: the drone cannot fly what it is asked")
    f = Flight(sim, lib, walks_east(1.4), **opts)
    f.at.append((30.0, lambda: sim.set_param("SIM_WIND_SPD", TOO_MUCH_WIND)))
    try:
        f.run(60.0)
    finally:
        sim.set_param("SIM_WIND_SPD", 0)
    not_obeyed_s = f.guide.constant("kNotObeyedS")
    r.check(f.end_why == "NotObeyed", f"Follow ends ({f.end_why} at {f.ended_at} s)")
    if f.ended_at is not None:
        r.check(30.0 + not_obeyed_s < f.ended_at < 30.0 + not_obeyed_s + 8.0, f"about {not_obeyed_s:.0f} s after the drone is blown off")
    f.close()
    return r


CASES = [case_walk, case_square, case_square_slow_telemetry, case_circle, case_car, case_range, case_comes_at,
         case_passes, case_hidden, case_lock_ends, case_operator_stops, case_app_dies, case_pilot_loiter,
         case_pilot_rtl, case_wind_ok, case_wind_too_much]


def back_to_loiter(sim: Sitl) -> None:
    """Between two cases: hover in Loiter where the drone is, standing."""
    sim.rc(rc3=1500)
    if sim.mode != "LOITER":
        sim.set_mode("LOITER")
    sim.pump(60, until=lambda last: math.hypot(sim.vn, sim.ve) < 0.3 and sim.alt > 25.0)
    if sim.alt < 25.0:
        # After RTL the drone is low or on the ground: up again.
        sim.set_mode("GUIDED")
        if not sim.armed:
            sim.arm_and_takeoff(30.0)
        else:
            sim.goto(sim.north, sim.east, 30.0)
            sim.pump(60, until=lambda last: sim.alt > 28.5)
        sim.rc(rc3=1500)
        sim.set_mode("LOITER")
        sim.pump(20, until=lambda last: math.hypot(sim.vn, sim.ve) < 0.3)


def main() -> int:
    args = sys.argv[1:]
    only = ""
    source = HERE.parent / "src" / "Skydroid"
    trace = "--trace" in args
    speedup = 4
    versions = []
    i = 0
    while i < len(args):
        if args[i] == "-k":
            only = args[i + 1]
            i += 1
        elif args[i] == "--source":
            source = pathlib.Path(args[i + 1])
            i += 1
        elif args[i] == "--speedup":
            speedup = int(args[i + 1])
            i += 1
        elif args[i] != "--trace":
            versions.append(args[i])
        i += 1
    lib = build_guide(source)
    cases = [c for c in CASES if only.lower() in c.__name__.lower()]
    failed = 0
    for version in versions or VERSIONS:
        print(f"ArduCopter {version}")
        sim = Sitl(version, {}, {"RC_OVERRIDE_TIME": -1, "SIM_WIND_SPD": 0, "SIM_WIND_DIR": 0, "SIM_WIND_T": 1},
                   speedup=speedup, home=",".join(str(v) for v in HOME))
        try:
            sim.hover_in_loiter(30.0)
            for case in cases:
                back_to_loiter(sim)
                report = case(sim, lib, {"trace": trace})
                print(("  PASS  " if not report.failed else "  FAIL  ") + report.name)
                for line in report.lines:
                    print(line)
                failed += report.failed
        finally:
            sim.close()
    print("all passed" if not failed else f"{failed} checks failed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
