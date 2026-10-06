"""Fly the wind failsafe script in the ArduCopter simulator.

Usage:
    py wind_failsafe_test.py                 all cases on 4.6.2 and 4.7.0
    py wind_failsafe_test.py 4.7.0           one version
    py wind_failsafe_test.py -k blown        only the cases with "blown" in the name
    py wind_failsafe_test.py --script other.lua   test another copy of the script

The simulated quadcopter leans at most 30 degrees, and with that it holds its
place in up to 10 m/s of wind. So 9 m/s is "strong but fine" and 14 m/s is
"too strong" in the cases below. The wind comes from the north.
"""
from __future__ import annotations

import math
import pathlib
import re
import sys
import time

from sitl_session import Sitl

HERE = pathlib.Path(__file__).resolve().parent
SCRIPT = HERE.parent / "scripts" / "vama_wind_failsafe.lua"
TAG = "Wind failsafe:"
READY = "Wind failsafe: ready"
VERSIONS = ["4.6.2", "4.7.0"]

# SIM_WIND_T 1: the same wind at every height (the simulator's default makes it weaker near the ground).
BASE = {"SCR_ENABLE": 1, "SCR_HEAP_SIZE": 300000, "RC_OVERRIDE_TIME": -1,
        "SIM_WIND_SPD": 0, "SIM_WIND_DIR": 0, "SIM_WIND_T": 1}
# Drag values of the simulated quadcopter, so that the autopilot estimates the wind.
DRAG = {"EK3_DRAG_BCOEF_X": 17.209, "EK3_DRAG_BCOEF_Y": 17.209, "EK3_DRAG_MCOEF": 0.209}

HOLDABLE_WIND = 9
TOO_MUCH_WIND = 14
# With this the simulated motors run at about 92 % in a calm hover.
WEAK_SPIN_MAX = 0.61

ACTION_OFF, ACTION_WARN, ACTION_RTL, ACTION_LAND = 0, 1, 2, 3

# The lean limit has another name and unit in each version: centidegrees in 4.6, degrees in 4.7.
LEAN_LIMIT_15 = {"4.6": ("ANGLE_MAX", 1500), "4.7": ("ATC_ANGLE_MAX", 15)}


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


def start(version: str, action: int | None = None, params: dict | None = None,
          script_params: dict | None = None) -> Sitl:
    extra = dict(script_params or {})
    if action is not None:
        extra["WFS_ACTION"] = action
    return Sitl(version, {"vama_wind_failsafe.lua": SCRIPT}, dict(BASE, **(params or {})),
                script_params=extra, script_ready=READY)


def wfs_texts(sim: Sitl, since: float = 0.0) -> list[str]:
    return [t for t in sim.texts_with(TAG, since) if not t.startswith(READY)]


def trips(sim: Sitl, since: float = 0.0) -> list[str]:
    """Texts that say the failsafe acted (not the early warnings)."""
    return [t for t in wfs_texts(sim, since) if "pushed back" in t or re.search(r"motors \d+% for", t)]


def wind_warnings(sim: Sitl) -> list[str]:
    return [t for t in wfs_texts(sim) if re.search(r"Wind failsafe: wind \d", t)]


def script_errors(sim: Sitl) -> list[str]:
    return [t for _, t in sim.texts if "Wind failsafe: error" in t or t.startswith("Lua:") or "Scripting: " in t and "error" in t.lower()]


def common_checks(sim: Sitl, r: Report) -> None:
    errors = script_errors(sim)
    r.check(not errors, "the script reported no error" + (f": {errors[:2]}" if errors else ""))
    # A status text holds 50 characters. A longer one arrives in pieces.
    long_texts = [t for t in sim.texts_with(TAG) if len(t) > 50]
    r.check(not long_texts, "every text fits one status message" + (f": {long_texts[:2]}" if long_texts else ""))
    ready = [t for t in sim.texts_with(READY)]
    r.check(len(ready) == 1 and ready[0].endswith(")"), f"it said that it is ready, with its action: {ready}")


def home_distance(sim: Sitl) -> float:
    return math.hypot(sim.north, sim.east)


def wait_mode(sim: Sitl, mode: str, limit_s: float, each_second=None) -> float:
    """Fly until the mode is reached (or the limit). Returns the simulated time of the change.

    The mode change arrives a moment before the text that explains it, so two
    more seconds are flown to collect that text.
    """
    sim.fly(limit_s, until=lambda: sim.mode == mode, each_second=each_second)
    when = sim.sim_s
    sim.fly(2, each_second=each_second)
    return when


# --- the cases -------------------------------------------------------------

def calm_flight(version: str, r: Report) -> None:
    """No wind: hover, fast legs and stops. Nothing may happen."""
    sim = start(version, ACTION_RTL)
    try:
        sim.hover_in_loiter(20)
        sim.fly(20)
        n = sim.named
        r.check(40 <= n.get("WFS_MOT", -1) <= 70, f"motor output in a calm hover is {n.get('WFS_MOT', -1):.0f} % (expected 40 to 70)")
        r.check(n.get("WFS_LEAN", 99) < 3, f"lean in a calm hover is {n.get('WFS_LEAN', 99):.1f} deg")
        r.check(n.get("WFS_PUSH", 99) < 0.2, f"pushed-away speed in a calm hover is {n.get('WFS_PUSH', 99):.2f} m/s")
        sim.set_mode("GUIDED")
        sim.goto(-150, 0, 20)
        sim.fly(40, until=lambda: sim.north < -145)
        r.check(sim.north < -140, f"flew the 150 m leg (now at {sim.north:.0f} m)")
        sim.goto(0, 0, 20)
        sim.fly(40, until=lambda: sim.north > -5)
        sim.set_mode("LOITER")
        sim.fly(10)
        r.check(not wfs_texts(sim), f"no warning and no action in calm air {wfs_texts(sim)}")
        r.check(sim.mode == "LOITER", f"mode is still LOITER ({sim.mode})")
        common_checks(sim, r)
    finally:
        sim.close()


def strong_wind_that_it_can_hold(version: str, r: Report) -> None:
    """9 m/s: the drone holds. It must warn, and never act, also on fast legs."""
    sim = start(version, ACTION_RTL)
    try:
        sim.hover_in_loiter(20)
        sim.fly(10)
        n0, e0 = sim.north, sim.east
        sim.set_param("SIM_WIND_SPD", HOLDABLE_WIND)
        sim.fly(45)
        moved = math.hypot(sim.north - n0, sim.east - e0)
        r.check(moved < 3, f"it holds its place in {HOLDABLE_WIND} m/s (moved {moved:.1f} m)")
        r.check(bool(sim.texts_with("to hold position")), f"it warns about the lean angle {wfs_texts(sim)}")
        r.check(abs(sim.named.get("WFS_LEAN", -99) - sim.lean_deg) < 2,
                f"the script reads the lean angle right ({sim.named.get('WFS_LEAN', -99):.1f} against {sim.lean_deg:.1f} deg)")
        sim.set_mode("GUIDED")
        sim.goto(-100, 0, 20)          # with the wind
        sim.fly(30)
        sim.goto(-60, 0, 20)           # against the wind, slow
        sim.fly(35)
        sim.goto(-60, 60, 20)          # wind from the side
        sim.fly(30)
        r.check(not trips(sim), f"no action on legs with, against and across the wind {trips(sim)}")
        r.check(sim.mode == "GUIDED", f"mode is still GUIDED ({sim.mode})")
        lean_warnings = sim.texts_with("to hold position")
        r.check(len(lean_warnings) <= 3, f"the warning is not repeated more than once a minute ({len(lean_warnings)} in about 140 s)")
        common_checks(sim, r)
    finally:
        sim.close()


def pilot_drifts_with_the_wind(version: str, r: Report) -> None:
    """In LOITER the pilot lets the drone go with a strong wind, then flies into it. No action."""
    sim = start(version, ACTION_RTL)
    try:
        sim.hover_in_loiter(20)
        sim.set_param("SIM_WIND_SPD", HOLDABLE_WIND)
        sim.fly(15)
        for pwm, seconds in ((1570, 20), (1650, 20), (1500, 12), (1100, 15), (1500, 8)):
            sim.rc(rc2=pwm, rc3=1500)
            sim.fly(seconds)
        r.check(sim.north < -20, f"the drone did drift with the wind (now {sim.north:.0f} m)")
        r.check(not trips(sim), f"no action while the pilot steers {trips(sim)}")
        r.check(sim.mode == "LOITER", f"mode is still LOITER ({sim.mode})")
        common_checks(sim, r)
    finally:
        sim.close()


def blown_away_home_upwind(version: str, r: Report) -> None:
    """14 m/s: pushed away with motors far from maximum. RTL, then LAND because RTL loses too. Once per flight."""
    sim = start(version, ACTION_RTL)
    try:
        sim.hover_in_loiter(20)
        sim.fly(12)
        t0 = sim.sim_s
        peak_motor = [0.0]

        def watch_motor():
            peak_motor[0] = max(peak_motor[0], sim.named.get("WFS_MOT", 0.0))

        sim.set_param("SIM_WIND_SPD", TOO_MUCH_WIND)
        t_rtl = wait_mode(sim, "RTL", 30, each_second=watch_motor)
        r.check(sim.mode == "RTL", f"the script switched to RTL ({sim.mode})")
        r.check(any("pushed back" in t and t.endswith("RTL") for t in trips(sim)), f"it said why: {trips(sim)}")
        r.check(t_rtl - t0 <= 15, f"RTL came {t_rtl - t0:.0f} s after the wind (limit 15 s), {home_distance(sim):.0f} m from home")
        r.check(peak_motor[0] < 80, f"the motors never came near their maximum ({peak_motor[0]:.0f} %), so the motor check alone would not have acted")
        t_land = wait_mode(sim, "LAND", 40)
        r.check(sim.mode == "LAND", f"RTL was pushed away too, so the script landed ({sim.mode})")
        r.check(any(t.endswith("RTL is pushed back, LAND") for t in trips(sim)), f"it said why: {trips(sim)[-1:]}")
        r.check(13 <= t_land - t_rtl <= 25, f"RTL had {t_land - t_rtl:.0f} s to try (expected 13 to 25 s)")
        # The pilot takes over. The script must not act a second time.
        seen = len(trips(sim))
        sim.set_mode("LOITER")
        sim.fly(25)
        r.check(sim.mode == "LOITER", f"after the pilot chose LOITER the script left it alone ({sim.mode})")
        r.check(len(trips(sim)) == seen, f"no second action in the same flight {trips(sim)[seen:]}")
        common_checks(sim, r)
    finally:
        sim.close()


def blown_away_home_downwind(version: str, r: Report) -> None:
    """Home is downwind: RTL gets closer to home, so it is left alone until it has passed home."""
    sim = start(version, ACTION_RTL)
    try:
        sim.arm_and_takeoff(20)
        sim.goto(150, 0, 20)
        sim.fly(40, until=lambda: sim.north > 145)
        sim.rc(rc3=1500)
        sim.set_mode("LOITER")
        sim.fly(6)
        r.check(sim.north > 140, f"start 150 m upwind of home (at {sim.north:.0f} m)")
        sim.set_param("SIM_WIND_SPD", TOO_MUCH_WIND)
        wait_mode(sim, "RTL", 30)
        r.check(sim.mode == "RTL", f"the script switched to RTL ({sim.mode}), {home_distance(sim):.0f} m from home")
        closest = [home_distance(sim)]
        wait_mode(sim, "LAND", 70, each_second=lambda: closest.__setitem__(0, min(closest[0], home_distance(sim))))
        r.check(closest[0] < 30, f"RTL was left alone while it flew home (closest {closest[0]:.0f} m)")
        r.check(sim.mode == "LAND", f"over home it cannot hold either, so the script landed ({sim.mode})")
        r.check(sim.north < 0, f"the landing started after it had passed home (at {sim.north:.0f} m)")
        common_checks(sim, r)
    finally:
        sim.close()


def motors_at_their_limit(version: str, r: Report) -> None:
    """Motors at 92 % in a hover: warning, then RTL. After landing, a new flight can act again."""
    sim = start(version, ACTION_RTL)
    try:
        sim.hover_in_loiter(20)
        sim.fly(12)
        t0 = sim.sim_s
        sim.set_param("MOT_SPIN_MAX", WEAK_SPIN_MAX)
        t_rtl = wait_mode(sim, "RTL", 30)
        r.check(sim.mode == "RTL", f"the script switched to RTL ({sim.mode})")
        r.check(bool(sim.texts_with("Wind failsafe: motors at")), f"it warned first {wfs_texts(sim)}")
        r.check(any(re.search(r"motors \d+% for \d+s, RTL", t) for t in trips(sim)), f"it said why: {trips(sim)}")
        r.check(t_rtl - t0 <= 20, f"RTL came {t_rtl - t0:.0f} s after the motors went high (limit 20 s)")
        # Land, then fly again: the script must be ready for a new flight.
        sim.set_param("MOT_SPIN_MAX", 0.95)
        sim.fly(90, until=lambda: not sim.armed)
        r.check(not sim.armed, "RTL landed and disarmed")
        first = len(trips(sim))
        sim.hover_in_loiter(20)
        sim.fly(12)
        sim.set_param("MOT_SPIN_MAX", WEAK_SPIN_MAX)
        wait_mode(sim, "RTL", 30)
        r.check(sim.mode == "RTL" and len(trips(sim)) > first, f"in the next flight it acted again ({sim.mode}) {trips(sim)[first:]}")
        common_checks(sim, r)
    finally:
        sim.close()


def default_is_warning_only(version: str, r: Report) -> None:
    """As installed (WFS_ACTION 1) the script only reports. The mode never changes."""
    sim = start(version)
    try:
        value = sim.get_param("WFS_ACTION")
        r.check(value is not None and abs(value - 1) < 0.01, f"WFS_ACTION is 1 after installing ({value})")
        sim.fly(5)
        r.check(sim.named.get("WFS_ACT") == 1, f"on the ground it already reports its action to the ground station ({sim.named.get('WFS_ACT')})")
        sim.hover_in_loiter(20)
        sim.fly(12)
        sim.set_param("SIM_WIND_SPD", TOO_MUCH_WIND)
        sim.fly(35)
        found = trips(sim)
        r.check(len(found) == 1 and "pushed back" in found[0] and "(warning only)" in found[0], f"one report, marked warning only: {found}")
        r.check(sim.mode == "LOITER", f"the mode did not change ({sim.mode})")
        common_checks(sim, r)
    finally:
        sim.close()


def action_land(version: str, r: Report) -> None:
    """WFS_ACTION 3: land where it is."""
    sim = start(version, ACTION_LAND)
    try:
        sim.hover_in_loiter(20)
        sim.fly(12)
        sim.set_param("SIM_WIND_SPD", TOO_MUCH_WIND)
        wait_mode(sim, "LAND", 30)
        r.check(sim.mode == "LAND", f"the script switched to LAND ({sim.mode})")
        r.check(any("pushed back" in t and t.endswith(", LAND") for t in trips(sim)), f"it said why: {trips(sim)}")
        common_checks(sim, r)
    finally:
        sim.close()


def action_off(version: str, r: Report) -> None:
    """WFS_ACTION 0: no text, no readings, no mode change. Only "installed, off"."""
    sim = start(version, ACTION_OFF)
    try:
        sim.hover_in_loiter(20)
        sim.fly(12)
        sim.set_param("SIM_WIND_SPD", TOO_MUCH_WIND)
        sim.fly(30)
        r.check(not wfs_texts(sim), f"no text {wfs_texts(sim)}")
        r.check("WFS_MOT" not in sim.named, "no readings sent")
        r.check(sim.named.get("WFS_ACT") == 0, f"it still reports that it is installed and off ({sim.named.get('WFS_ACT')})")
        r.check(sim.mode == "LOITER", f"the mode did not change ({sim.mode})")
        common_checks(sim, r)
    finally:
        sim.close()


def no_gps_means_land(version: str, r: Report) -> None:
    """GPS lost: RTL is refused, so the script lands instead."""
    # FS_EKF_THRESH 0 keeps the autopilot's own EKF failsafe out of the way, so that
    # the script's fallback is what gets tested.
    sim = start(version, ACTION_RTL, params={"FS_EKF_THRESH": 0})
    try:
        sim.hover_in_loiter(20)
        sim.fly(10)
        sim.gps_off()
        sim.fly(60, until=lambda: (sim.ekf_flags & 0x18) == 0)
        r.check((sim.ekf_flags & 0x18) == 0, f"the autopilot has lost its position (EKF flags {sim.ekf_flags:#x})")
        r.check(sim.mode == "LOITER", f"still in LOITER before the test ({sim.mode})")
        sim.set_param("MOT_SPIN_MAX", WEAK_SPIN_MAX)
        wait_mode(sim, "LAND", 30)
        r.check(sim.mode == "LAND", f"the script landed ({sim.mode})")
        r.check(any(t.endswith("no RTL, LAND") for t in trips(sim)), f"it said RTL was not possible: {trips(sim)}")
        common_checks(sim, r)
    finally:
        sim.close()


def pilot_modes_are_left_alone(version: str, r: Report) -> None:
    """ALT_HOLD: the pilot flies. No action, whatever the wind and the motors do. LOITER after that does act."""
    sim = start(version, ACTION_RTL)
    try:
        sim.arm_and_takeoff(20)
        sim.rc(rc3=1500)
        sim.set_mode("ALT_HOLD")
        sim.set_param("SIM_WIND_SPD", TOO_MUCH_WIND)
        sim.fly(20)
        sim.set_param("MOT_SPIN_MAX", WEAK_SPIN_MAX)
        sim.fly(20)
        r.check(sim.named.get("WFS_MOT", 0) >= 85, f"the motors were high ({sim.named.get('WFS_MOT', 0):.0f} %)")
        r.check(not wfs_texts(sim), f"no text in ALT_HOLD {wfs_texts(sim)}")
        r.check(sim.mode == "ALT_HOLD", f"the mode did not change ({sim.mode})")
        sim.set_param("MOT_SPIN_MAX", 0.95)
        sim.fly(8)
        sim.set_mode("LOITER")
        wait_mode(sim, "RTL", 40)
        r.check(sim.mode == "RTL" and bool(trips(sim)), f"in LOITER the script does act ({sim.mode}) {trips(sim)}")
        common_checks(sim, r)
    finally:
        sim.close()


def modes_without_a_hold_point(version: str, r: Report) -> None:
    """Some modes give the script no hold point. Flying slowly with the wind is left alone, a blow-away is not."""
    sim = start(version, ACTION_RTL)
    try:
        sim.arm_and_takeoff(20)
        sim.set_param("SIM_WIND_SPD", 6)
        sim.fly(15)
        # GUIDED with a speed command: 3 m/s to the south, with a 6 m/s wind from the north.
        # The drone still leans north to hold that speed, so its thrust is against its movement.
        n0 = sim.north
        peak_push = [0.0]

        def keep_flying():
            sim.velocity(-3.0, 0.0)
            peak_push[0] = max(peak_push[0], sim.named.get("WFS_PUSH", 0.0))

        keep_flying()
        sim.fly(25, each_second=keep_flying)
        r.check(sim.north < n0 - 50, f"it flew with the wind ({n0 - sim.north:.0f} m)")
        r.check(peak_push[0] >= 2.0, f"its thrust was against the movement ({peak_push[0]:.1f} m/s)")
        r.check(sim.named.get("WFS_LEAN", 99) < 15, f"it leaned little ({sim.named.get('WFS_LEAN', 99):.0f} deg)")
        r.check(not trips(sim), f"no action: it is not working hard {trips(sim)}")
        # POSHOLD, sticks centred, in a wind it cannot hold.
        sim.rc(rc3=1500)
        sim.set_mode("POSHOLD")
        sim.fly(10)
        sim.set_param("SIM_WIND_SPD", TOO_MUCH_WIND)
        wait_mode(sim, "RTL", 40)
        r.check(sim.mode == "RTL" and bool(trips(sim)), f"a wind it cannot hold does trip in POSHOLD ({sim.mode}) {trips(sim)}")
        common_checks(sim, r)
    finally:
        sim.close()


def slow_stop_with_the_wind_behind(version: str, r: Report) -> None:
    """A fast leg with a strong wind from behind, then stop. The stop takes long, but it is a stop, not a blow-away."""
    # WFS_TIME 2 s (the shortest), so that the stop lasts clearly longer than the time that would trip.
    sim = start(version, ACTION_RTL, script_params={"WFS_TIME": 2})
    try:
        sim.arm_and_takeoff(20)
        # Just under the 10 m/s that this drone can hold, so it has little lean left to stop with.
        sim.set_param("SIM_WIND_SPD", 9.7)
        sim.fly(15)

        def go():
            sim.velocity(-12.0, 0.0)

        go()
        sim.fly(15, each_second=go)
        r.check(sim.vn < -8, f"it flies fast with the wind ({-sim.vn:.0f} m/s)")
        pushed_seconds = [0]

        def stop():
            sim.velocity(0.0, 0.0)
            if sim.named.get("WFS_PUSH", 0.0) >= 1.0 and sim.named.get("WFS_LEAN", 0.0) >= 20:
                pushed_seconds[0] += 1

        stop()
        sim.fly(45, each_second=stop)
        r.check(pushed_seconds[0] >= 4, f"the stop took long: {pushed_seconds[0]} s with the thrust against the movement and a hard lean (WFS_TIME is 2 s)")
        r.check(abs(sim.vn) < 1.0, f"it did stop ({abs(sim.vn):.1f} m/s)")
        r.check(not trips(sim), f"no action: it was getting slower all the time {trips(sim)}")
        r.check(sim.mode == "GUIDED", f"mode is still GUIDED ({sim.mode})")
        common_checks(sim, r)
    finally:
        sim.close()


def small_lean_limit(version: str, r: Report) -> None:
    """A drone that may lean 15 degrees at most. It is pushed away while it leans less than WFS_LEAN, and the script still acts."""
    sim = start(version, ACTION_RTL, params=dict([LEAN_LIMIT_15[version[:3]]]))
    try:
        sim.hover_in_loiter(20)
        sim.fly(12)
        t0 = sim.sim_s
        peak_lean = [0.0]
        # 9 m/s needs 26 degrees of lean: fine for a 30 degree drone, too much for this one.
        sim.set_param("SIM_WIND_SPD", HOLDABLE_WIND)
        t_rtl = wait_mode(sim, "RTL", 40,
                          each_second=lambda: peak_lean.__setitem__(0, max(peak_lean[0], sim.named.get("WFS_LEAN", 0.0))))
        r.check(10 < peak_lean[0] < 17, f"it leaned to its own limit and no further ({peak_lean[0]:.0f} deg, WFS_LEAN is 20)")
        r.check(sim.mode == "RTL" and bool(trips(sim)), f"the script still acted ({sim.mode}) {trips(sim)}")
        r.check(t_rtl - t0 <= 20, f"RTL came {t_rtl - t0:.0f} s after the wind (limit 20 s)")
        common_checks(sim, r)
    finally:
        sim.close()


def pilot_landing_is_left_alone(version: str, r: Report) -> None:
    """The pilot lands in strong wind. The script must not pull it back up into RTL."""
    sim = start(version, ACTION_RTL)
    try:
        sim.hover_in_loiter(30)
        sim.fly(10)
        sim.set_mode("LAND")
        sim.set_param("SIM_WIND_SPD", TOO_MUCH_WIND)
        sim.fly(90, until=lambda: not sim.armed)
        r.check(not trips(sim), f"no action during the landing {trips(sim)}")
        r.check(sim.mode == "LAND", f"the mode stayed LAND ({sim.mode})")
        common_checks(sim, r)
    finally:
        sim.close()


def rtl_landing_is_left_alone(version: str, r: Report) -> None:
    """RTL is already landing at home when the wind comes. The script must not interfere."""
    sim = start(version, ACTION_RTL)
    try:
        sim.hover_in_loiter(20)
        sim.fly(10)
        sim.set_mode("RTL")
        # RTL waits over home for a few seconds, then starts down.
        sim.fly(40, until=lambda: sim.alt < 17)
        r.check(sim.alt < 17 and sim.mode == "RTL", f"RTL is coming down ({sim.alt:.0f} m, {sim.mode})")
        sim.set_param("SIM_WIND_SPD", TOO_MUCH_WIND)
        sim.fly(90, until=lambda: not sim.armed)
        r.check(not trips(sim), f"no action during the landing {trips(sim)}")
        r.check(sim.mode == "RTL", f"the mode stayed RTL ({sim.mode})")
        common_checks(sim, r)
    finally:
        sim.close()


def wind_estimate_warning(version: str, r: Report) -> None:
    """With the drag parameters set, WFS_WSPD warns from the autopilot's wind estimate."""
    sim = start(version, ACTION_WARN, params=DRAG, script_params={"WFS_WSPD": 7})
    try:
        sim.hover_in_loiter(20)
        sim.fly(10)
        r.check(not wind_warnings(sim), "no wind warning in calm air")
        # The wind builds up in steps. After one sudden jump from 0 to 9 m/s the
        # autopilot's estimate stays stuck near 2 m/s (seen on 4.7.0), which is
        # why the script's own checks do not depend on that estimate.
        for wind, seconds in ((3, 8), (6, 8), (HOLDABLE_WIND, 30)):
            sim.set_param("SIM_WIND_SPD", wind)
            sim.fly(seconds)
        found = wind_warnings(sim)
        speeds = [float(m.group(1)) for m in (re.search(r"wind (\d+) m/s", t) for t in found) if m]
        r.check(bool(speeds) and all(7 <= s <= 11 for s in speeds), f"it warned with the estimated speed in a {HOLDABLE_WIND} m/s wind: {found}")
        r.check(not trips(sim), f"no action {trips(sim)}")
        common_checks(sim, r)
    finally:
        sim.close()


def gusty_wind(version: str, r: Report) -> None:
    """Gusts: 7 m/s with gusts must not act. 14 m/s with gusts must still act."""
    sim = start(version, ACTION_RTL, params={"SIM_WIND_TURB": 2})
    try:
        sim.hover_in_loiter(20)
        sim.set_param("SIM_WIND_SPD", 7)
        sim.fly(90)
        r.check(not trips(sim), f"no action in a gusty 7 m/s wind {trips(sim)}")
        r.check(sim.mode == "LOITER", f"mode is still LOITER ({sim.mode})")
        t0 = sim.sim_s
        sim.set_param("SIM_WIND_TURB", 3)
        sim.set_param("SIM_WIND_SPD", TOO_MUCH_WIND)
        t_rtl = wait_mode(sim, "RTL", 40)
        r.check(sim.mode == "RTL" and bool(trips(sim)), f"a gusty {TOO_MUCH_WIND} m/s wind still trips ({sim.mode}) after {t_rtl - t0:.0f} s")
        common_checks(sim, r)
    finally:
        sim.close()


CASES = [calm_flight, strong_wind_that_it_can_hold, pilot_drifts_with_the_wind, blown_away_home_upwind,
         blown_away_home_downwind, motors_at_their_limit, default_is_warning_only, action_land, action_off,
         no_gps_means_land, pilot_modes_are_left_alone, modes_without_a_hold_point, slow_stop_with_the_wind_behind,
         small_lean_limit, pilot_landing_is_left_alone,
         rtl_landing_is_left_alone,
         wind_estimate_warning, gusty_wind]


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
