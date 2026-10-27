"""The settings page of the drone's obstacle sensor: what VGCS reads, what it writes, and what it says first.

Client Stage 1.4, A7 item 1 (2026-10-27). The client's LiDAR is a Slamtec
RPLidar A3M1 on TELEM2 of a CubeOrangePlus. The page sets up a 360 degree
RPLidar as the first proximity sensor, how the drone keeps away from what it
sees, and the path planner. It reads every setting first and writes only what
changes, each one confirmed by the drone (vgcs/link/confirmations.py), and
reads them back.

ArduPilot facts (4.7.0 source and the simulator, 4.6.2 and 4.7.0, 2026-10-27):

- ``PRX1_TYPE`` 5 is the RPLidar driver (A1, A2, A2M12, C1 and S1 by name; an
  A3 is "unknown" and still read). ``PRX1_ORIENT`` 0 on top, 1 upside down.
  ``PRX1_YAW_CORR`` turns its readings. ``PRX1_MIN`` and ``PRX1_MAX`` leave out
  readings nearer or farther (0: none left out); they do not give the A3 a
  range.
- The port: ``SERIALn_PROTOCOL`` 11 (Lidar360) and ``SERIALn_BAUD`` the
  sensor's speed (an A3 256000). ``BRD_SERn_RTSCTS`` 0 (no flow control) on
  ports that have it; the simulator has none. SERIAL0 is USB.
- ``AVOID_ENABLE`` bit 2 uses the sensor in Loiter, AltHold, FlowHold and
  ZigZag; ``AVOID_MARGIN`` the stop distance; ``AVOID_BEHAVE`` 1 stop, 0 slide.
- ``OA_TYPE`` the path planner of Auto, Guided, RTL and SmartRTL (1 BendyRuler
  goes around what a sensor sees, 2 Dijkstra around fences, 3 both).
- The drone takes the sensor, the port and the planner only at its next start.
  It refuses to restart while armed, and keeps the settings over a restart.
- The mode switch: ``FLTMODE1`` to ``FLTMODE6``, on ``FLTMODE_CH``.

Pure Python, no Qt: every rule here is unit tested (tests/test_obstacle_setup.py).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from vgcs.app.obstacle_watch import AVOID_PROXIMITY_BIT, mode_effect, mode_name
from vgcs.mode.mode_mapping import AP_COPTER_MODE_MAP

PRX_RPLIDAR = 5
PROTOCOL_NONE = -1
PROTOCOL_LIDAR360 = 11
PORTS = tuple(range(1, 9))          # SERIAL1 to SERIAL8; SERIAL0 is USB

# Speeds as ArduPilot keeps them (SERIALn_BAUD) and the RPLidar models at each.
BAUD_WORDS = {1: "1200", 2: "2400", 4: "4800", 9: "9600", 19: "19200", 38: "38400", 57: "57600",
              111: "111100", 115: "115200", 230: "230400", 256: "256000", 460: "460800", 500: "500000",
              921: "921600", 1500: "1500000", 2000: "2000000"}
MODELS = (
    ("a3", "RPLidar A3, A2M12 or S1 (256000 baud)", 256),
    ("a1a2", "RPLidar A1 or A2M8 (115200 baud)", 115),
    ("c1", "RPLidar C1 (460800 baud)", 460),
)
DEFAULT_MODEL = "a3"
# Nish, 2026-10-26: the A3 goes on TELEM2 (SERIAL2 on a Cube), TELEM1 is in use.
DEFAULT_PORT = 2
PORT_HINTS = {1: "TELEM1 on a Cube", 2: "TELEM2 on a Cube", 3: "GPS1 on a Cube", 4: "GPS2 on a Cube"}
PROTOCOL_WORDS = {
    -1: "nothing", 1: "MAVLink 1", 2: "MAVLink 2", 3: "FrSky D", 4: "FrSky SPort", 5: "GPS", 7: "Alexmos gimbal",
    8: "SToRM32 gimbal", 9: "rangefinder", 10: "FrSky passthrough", 11: "Lidar360", 13: "beacon", 15: "SBUS out",
    16: "ESC telemetry", 18: "optical flow", 20: "NMEA out", 23: "RC input", 28: "scripting", 32: "MSP",
    33: "DJI FPV", 35: "ADS-B", 42: "DisplayPort", 43: "MAVLink high latency",
}
MAVLINK_PROTOCOLS = (1, 2, 43)

PLANNER_CHOICES = (
    (0, "Off: Auto, Guided and RTL fly on"),
    (1, "Around what the sensor sees (BendyRuler)"),
    (2, "Around fences only (Dijkstra)"),
    (3, "Both (Dijkstra and BendyRuler)"),
)
PLANNER_WHY = {0: "the path planner off", 1: "the path planner around what the sensor sees",
               2: "the path planner around fences only", 3: "the path planner around fences and what the sensor sees"}
MARGIN_RANGE_M = (1.0, 10.0)        # AVOID_MARGIN
RANGE_LIMIT_M = (0.0, 500.0)        # PRX1_MIN, PRX1_MAX
YAW_RANGE_DEG = (-180.0, 180.0)     # PRX1_YAW_CORR

# Mode switch positions: the PWM of FLTMODE_CH (ArduCopter's own limits).
SWITCH_PWM = ("up to 1230", "1231 to 1360", "1361 to 1490", "1491 to 1620", "1621 to 1749", "1750 and more")

# A setting the drone takes only at its next start.
_TAKEN_AT_START = ("PRX1_TYPE", "OA_TYPE")


def port_setting(port: int, kind: str) -> str:
    """SERIAL2_PROTOCOL, SERIAL2_BAUD, BRD_SER2_RTSCTS."""
    return f"BRD_SER{port}_RTSCTS" if kind == "RTSCTS" else f"SERIAL{port}_{kind}"


def taken_at_start(name: str) -> bool:
    return name in _TAKEN_AT_START or (name.startswith("SERIAL") and name.endswith(("_PROTOCOL", "_BAUD"))) \
        or (name.startswith("BRD_SER") and name.endswith("_RTSCTS"))


def read_names() -> list[str]:
    """Every setting the page reads (a drone lacks some: BRD_SERn_RTSCTS on most ports, SERIAL8 on many)."""
    names = ["PRX1_TYPE", "PRX1_ORIENT", "PRX1_YAW_CORR", "PRX1_MIN", "PRX1_MAX", "PRX2_TYPE",
             "AVOID_ENABLE", "AVOID_MARGIN", "AVOID_BEHAVE", "OA_TYPE", "FLTMODE_CH"]
    names += [f"FLTMODE{i}" for i in range(1, 7)]
    for port in PORTS:
        names += [port_setting(port, "PROTOCOL"), port_setting(port, "BAUD"), port_setting(port, "RTSCTS")]
    return names


def _number(value: object) -> float | None:
    try:
        number = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _same(a: float | None, b: float) -> bool:
    return a is not None and abs(a - b) <= 1e-4 * max(1.0, abs(b))


def clean_values(values: dict) -> dict[str, float]:
    """The drone's settings by upper-case name, numbers only."""
    out: dict[str, float] = {}
    for name, value in (values or {}).items():
        number = _number(value)
        if number is not None:
            out[str(name).strip().upper()] = number
    return out


def protocol_words(protocol: float | None) -> str:
    if protocol is None:
        return "not on this drone"
    return PROTOCOL_WORDS.get(int(protocol), f"protocol {int(protocol)}")


def port_words(port: int, values: dict[str, float]) -> str:
    """"SERIAL2 (TELEM2 on a Cube): MAVLink 2 now"."""
    hint = PORT_HINTS.get(port)
    head = f"SERIAL{port}" + (f" ({hint})" if hint else "")
    return f"{head}: {protocol_words(values.get(port_setting(port, 'PROTOCOL')))} now"


def ports_on_drone(values: dict[str, float]) -> list[int]:
    return [p for p in PORTS if port_setting(p, "PROTOCOL") in values]


def lidar_ports(values: dict[str, float]) -> list[int]:
    return [p for p in ports_on_drone(values) if int(values[port_setting(p, "PROTOCOL")]) == PROTOCOL_LIDAR360]


def model_baud(model: str) -> int:
    return next((baud for key, _words, baud in MODELS if key == model), MODELS[0][2])


@dataclass(frozen=True)
class Choices:
    """What the page shows and the operator picks."""

    lidar_on: bool = False
    model: str = DEFAULT_MODEL
    port: int = DEFAULT_PORT
    upside_down: bool = False
    yaw_corr_deg: float = 0.0
    min_m: float = 0.0
    max_m: float = 0.0
    keep_away: bool = True
    stop_m: float = 2.0
    stop: bool = False
    planner: int = 0


def choices_from_drone(values: dict) -> Choices:
    """The page as the drone is set up now. Where it has no LiDAR port yet: TELEM2, an A3."""
    v = clean_values(values)
    ports = ports_on_drone(v)
    found = lidar_ports(v)
    if found:
        port = found[0]
    elif DEFAULT_PORT in ports:
        port = DEFAULT_PORT
    else:
        free = [p for p in ports if int(v[port_setting(p, "PROTOCOL")]) == PROTOCOL_NONE]
        port = (free or ports or [DEFAULT_PORT])[0]
    model = DEFAULT_MODEL
    if found:
        baud = v.get(port_setting(port, "BAUD"))
        model = next((key for key, _words, b in MODELS if baud is not None and int(baud) == b), DEFAULT_MODEL)
    enable = v.get("AVOID_ENABLE")
    return Choices(
        lidar_on=int(v.get("PRX1_TYPE", 0)) == PRX_RPLIDAR,
        model=model,
        port=port,
        upside_down=int(v.get("PRX1_ORIENT", 0)) == 1,
        yaw_corr_deg=v.get("PRX1_YAW_CORR", 0.0),
        min_m=v.get("PRX1_MIN", 0.0),
        max_m=v.get("PRX1_MAX", 0.0),
        keep_away=True if enable is None else bool(int(enable) & AVOID_PROXIMITY_BIT),
        stop_m=v.get("AVOID_MARGIN", 2.0),
        stop=int(v.get("AVOID_BEHAVE", 0)) == 1,
        planner=int(v.get("OA_TYPE", 0)),
    )


@dataclass(frozen=True)
class Change:
    name: str
    old: float
    new: float
    why: str

    def words(self) -> str:
        return f"{self.name}: {self.old:g} to {self.new:g} ({self.why})"


@dataclass
class Plan:
    """What writing the page would change, and what to say before."""

    changes: list[Change] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    refusal: str | None = None
    left_out: list[str] = field(default_factory=list)   # settings this drone lacks, so not written

    @property
    def restart_names(self) -> list[str]:
        return [c.name for c in self.changes if taken_at_start(c.name)]

    def values_after(self, values: dict) -> dict[str, float]:
        out = clean_values(values)
        for c in self.changes:
            out[c.name] = c.new
        return out


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


def plan_writes(values: dict, choices: Choices, armed: bool) -> Plan:
    """The settings to write so that the drone matches the page. Only what changes.

    Refused while the drone is armed, and when it has no proximity settings.
    """
    v = clean_values(values)
    plan = Plan()
    if armed:
        plan.refusal = ("The drone is armed. Obstacle settings are written on the ground only: the sensor, its port "
                        "and the planner change only at the drone's next start.")
        return plan
    if "PRX1_TYPE" not in v:
        plan.refusal = ("This drone has no PRX1_TYPE: its firmware has no obstacle sensor (proximity) support, or "
                        "its settings are not read yet. Press Read from drone.")
        return plan

    def want(name: str, new: float, why: str) -> None:
        if name not in v:
            plan.left_out.append(name)
            return
        if not _same(v[name], new):
            plan.changes.append(Change(name, v[name], float(new), why))

    c = choices
    if c.lidar_on:
        if port_setting(c.port, "PROTOCOL") not in v:
            plan.refusal = f"This drone has no SERIAL{c.port}. Choose another port."
            return plan
        want("PRX1_TYPE", PRX_RPLIDAR, "the RPLidar")
        protocol = int(v[port_setting(c.port, "PROTOCOL")])
        if protocol in MAVLINK_PROTOCOLS:
            plan.warnings.append(
                f"SERIAL{c.port} speaks {protocol_words(protocol)} now. If a telemetry radio, the remote control's "
                "link or a companion computer is on it, it stops working after the restart, and a ground station "
                "on it loses the drone.")
        elif protocol not in (PROTOCOL_NONE, PROTOCOL_LIDAR360):
            plan.warnings.append(f"SERIAL{c.port} is for {protocol_words(protocol)} now. That stops after the restart.")
        want(port_setting(c.port, "PROTOCOL"), PROTOCOL_LIDAR360, f"the LiDAR on SERIAL{c.port}")
        want(port_setting(c.port, "BAUD"), model_baud(c.model), f"the LiDAR's speed, {BAUD_WORDS[model_baud(c.model)]}")
        # Only where the board has it (want leaves out what the drone lacks; the simulator has none).
        want(port_setting(c.port, "RTSCTS"), 0, "no flow control for the LiDAR")
        for other in lidar_ports(v):
            if other != c.port:
                want(port_setting(other, "PROTOCOL"), PROTOCOL_NONE, f"the LiDAR is no longer on SERIAL{other}")
        want("PRX1_ORIENT", 1 if c.upside_down else 0, "mounted upside down" if c.upside_down else "mounted on top")
        want("PRX1_YAW_CORR", round(_clamp(c.yaw_corr_deg, *YAW_RANGE_DEG)), "the LiDAR's turn")
        low = _clamp(c.min_m, *RANGE_LIMIT_M)
        high = _clamp(c.max_m, *RANGE_LIMIT_M)
        if high and low >= high:
            plan.refusal = (f"Readings nearer than {low:g} m and farther than {high:g} m would all be left out. "
                            "Make the farther limit larger, or 0 for none.")
            return plan
        want("PRX1_MIN", low, "readings nearer are left out" if low else "no near limit")
        want("PRX1_MAX", high, "readings farther are left out" if high else "no far limit")
    elif int(v.get("PRX1_TYPE", 0)) != 0:
        want("PRX1_TYPE", 0, "no obstacle sensor")

    if "AVOID_ENABLE" in v:
        enable = int(v["AVOID_ENABLE"])
        new_enable = enable | AVOID_PROXIMITY_BIT if c.keep_away else enable & ~AVOID_PROXIMITY_BIT
        want("AVOID_ENABLE", new_enable, "keep away from what the sensor sees" if c.keep_away
             else "do not keep away from what the sensor sees")
        want("AVOID_MARGIN", round(_clamp(c.stop_m, *MARGIN_RANGE_M), 1), "the stop distance")
        want("AVOID_BEHAVE", 1 if c.stop else 0, "stop before it" if c.stop else "slide along it")
    else:
        plan.left_out.append("AVOID_ENABLE")
    if c.planner not in PLANNER_WHY:
        plan.refusal = f"Path planner {c.planner} is not one this page knows."
        return plan
    want("OA_TYPE", c.planner, PLANNER_WHY[c.planner])

    sensor_after = c.lidar_on or int(v.get("PRX2_TYPE", 0)) != 0
    if c.keep_away and not sensor_after:
        plan.warnings.append("No obstacle sensor is set up: keeping away from objects does nothing until one is.")
    if c.planner in (1, 3) and not sensor_after:
        plan.warnings.append("No obstacle sensor is set up: the path planner then goes around fences only.")
    return plan


def summary(values: dict) -> str:
    """One line for the Vehicle Configuration window: what the drone has now."""
    v = clean_values(values)
    if "PRX1_TYPE" not in v:
        return "Not read yet."
    kind = int(v["PRX1_TYPE"])
    if kind == PRX_RPLIDAR:
        ports = lidar_ports(v)
        if not ports_on_drone(v):
            sensor = "RPLidar (PRX1_TYPE 5)"         # the ports are read by the settings window only
        else:
            sensor = "RPLidar on " + (", ".join(f"SERIAL{p}" for p in ports) if ports else "no port (none is Lidar360)")
    elif kind == 0:
        sensor = "No obstacle sensor (PRX1_TYPE 0)"
    else:
        sensor = f"Obstacle sensor type {kind} (PRX1_TYPE)"
    parts = [sensor]
    if "AVOID_ENABLE" in v:
        if int(v["AVOID_ENABLE"]) & AVOID_PROXIMITY_BIT:
            margin = v.get("AVOID_MARGIN")
            how = "stop" if int(v.get("AVOID_BEHAVE", 0)) == 1 else "slide"
            parts.append(f"keeps away, {how}" + ("" if margin is None else f" {margin:g} m from it"))
        else:
            parts.append("does not keep away (AVOID_ENABLE)")
    if "OA_TYPE" in v:
        oa = int(v["OA_TYPE"])
        parts.append(PLANNER_WHY.get(oa, f"OA_TYPE {oa}"))
    return ", ".join(parts) + "."


def read_back_mismatches(changes: list[Change], values: dict) -> list[str]:
    """The written settings the drone does not hold now (empty: all of them read back)."""
    v = clean_values(values)
    out = []
    for c in changes:
        got = v.get(c.name)
        if got is None:
            out.append(f"{c.name}: not read back")
        elif not _same(got, c.new):
            out.append(f"{c.name}: the drone holds {got:g}, not {c.new:g}")
    return out


@dataclass(frozen=True)
class SwitchPosition:
    position: int
    pwm: str
    mode: str          # "Loiter", or "mode 99"
    words: str         # what that mode does about an object
    avoids: bool | None


def mode_switch(values: dict) -> list[SwitchPosition]:
    """What each position of the mode switch does about an object, with these settings (ArduCopter)."""
    v = clean_values(values)
    params = {name: v[name] for name in ("AVOID_ENABLE", "AVOID_MARGIN", "AVOID_BEHAVE", "OA_TYPE") if name in v}
    out = []
    for i in range(1, 7):
        number = v.get(f"FLTMODE{i}")
        if number is None:
            continue
        key = AP_COPTER_MODE_MAP.get(int(number))
        if key is None:
            out.append(SwitchPosition(i, SWITCH_PWM[i - 1], f"mode {int(number)}", "a mode VGCS does not know.", None))
            continue
        words, avoids = mode_effect(key, params)
        out.append(SwitchPosition(i, SWITCH_PWM[i - 1], mode_name(key), words, avoids))
    return out


def mode_switch_warnings(values: dict) -> list[str]:
    """One line for each switch position whose mode does not keep away from an object."""
    return [f"Mode switch position {p.position} ({p.pwm}): {p.words}" for p in mode_switch(values) if p.avoids is False]

