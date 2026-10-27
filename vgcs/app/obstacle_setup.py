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
- The TF02 Pro (the client's second sensor, one direction): ``RNGFND1_TYPE`` 19
  on a port with ``SERIALn_PROTOCOL`` 9 at 115200, ``RNGFND1_ORIENT`` 0 ahead
  or 25 down. Its range is ``RNGFND1_MIN``/``RNGFND1_MAX`` in metres on 4.7,
  ``RNGFND1_MIN_CM``/``RNGFND1_MAX_CM`` in centimetres on 4.6. Looking ahead
  it is an obstacle sensor too (``PRXn_TYPE`` 4, which uses the rangefinders
  that look sideways or up; ArduPilot then stops sending it as a rangefinder).
  The proximity slots go in order: an empty PRX1 before a used PRX2 stops
  arming ("PRX1: Not Connected"), and a type 4 slot with only a downward
  rangefinder has no data ("PRX2: No Data").

Pure Python, no Qt: every rule here is unit tested (tests/test_obstacle_setup.py).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from vgcs.app.obstacle_watch import AVOID_PROXIMITY_BIT, mode_effect, mode_name
from vgcs.mode.mode_mapping import AP_COPTER_MODE_MAP

PRX_RPLIDAR = 5
PRX_RANGEFINDER = 4
RNGFND_TF02 = 19
PROTOCOL_NONE = -1
PROTOCOL_LIDAR360 = 11
PROTOCOL_RANGEFINDER = 9
TF02_BAUD = 115
# Benewake TF02 Pro: 0.1 to 40 m (a white target; 13.5 m at 10 % reflectivity).
TF02_RANGE_M = (0.1, 40.0)
TF02_LIMIT_M = (0.01, 100.0)
ORIENT_AHEAD = 0
ORIENT_DOWN = 25
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
_TAKEN_AT_START = ("PRX1_TYPE", "PRX2_TYPE", "OA_TYPE", "RNGFND1_TYPE")


def port_setting(port: int, kind: str) -> str:
    """SERIAL2_PROTOCOL, SERIAL2_BAUD, BRD_SER2_RTSCTS."""
    return f"BRD_SER{port}_RTSCTS" if kind == "RTSCTS" else f"SERIAL{port}_{kind}"


def taken_at_start(name: str) -> bool:
    return name in _TAKEN_AT_START or (name.startswith("SERIAL") and name.endswith(("_PROTOCOL", "_BAUD"))) \
        or (name.startswith("BRD_SER") and name.endswith("_RTSCTS"))


def read_names() -> list[str]:
    """Every setting the page reads (a drone lacks some: BRD_SERn_RTSCTS on most ports, SERIAL8 on many)."""
    names = ["PRX1_TYPE", "PRX1_ORIENT", "PRX1_YAW_CORR", "PRX1_MIN", "PRX1_MAX", "PRX2_TYPE",
             "AVOID_ENABLE", "AVOID_MARGIN", "AVOID_BEHAVE", "OA_TYPE", "FLTMODE_CH",
             "RNGFND1_TYPE", "RNGFND1_ORIENT", "RNGFND1_MIN", "RNGFND1_MAX", "RNGFND1_MIN_CM", "RNGFND1_MAX_CM",
             "RNGFND2_TYPE"]
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


def rangefinder_ports(values: dict[str, float]) -> list[int]:
    return [p for p in ports_on_drone(values) if int(values[port_setting(p, "PROTOCOL")]) == PROTOCOL_RANGEFINDER]


def range_settings(values: dict[str, float]) -> tuple[str, str, float] | None:
    """The rangefinder's range on this firmware: (near name, far name, units in a metre), or None.

    4.7: RNGFND1_MIN and RNGFND1_MAX in metres. 4.6: RNGFND1_MIN_CM and RNGFND1_MAX_CM in centimetres.
    """
    if "RNGFND1_MIN" in values and "RNGFND1_MAX" in values:
        return "RNGFND1_MIN", "RNGFND1_MAX", 1.0
    if "RNGFND1_MIN_CM" in values and "RNGFND1_MAX_CM" in values:
        return "RNGFND1_MIN_CM", "RNGFND1_MAX_CM", 100.0
    return None


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
    tf02_on: bool = False
    tf02_port: int = 0
    tf02_ahead: bool = True
    tf02_min_m: float = TF02_RANGE_M[0]
    tf02_max_m: float = TF02_RANGE_M[1]


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
    tf02 = int(v.get("RNGFND1_TYPE", 0)) == RNGFND_TF02
    found_rf = rangefinder_ports(v)
    if found_rf:
        tf02_port = found_rf[0]
    else:
        free = [p for p in ports if int(v[port_setting(p, "PROTOCOL")]) == PROTOCOL_NONE and p != port]
        tf02_port = (free or [p for p in ports if p != port] or [0])[0]
    tf02_min, tf02_max = TF02_RANGE_M
    names = range_settings(v)
    if tf02 and names is not None:
        tf02_min, tf02_max = v[names[0]] / names[2], v[names[1]] / names[2]
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
        tf02_on=tf02,
        tf02_port=tf02_port,
        tf02_ahead=not tf02 or int(v.get("RNGFND1_ORIENT", ORIENT_AHEAD)) != ORIENT_DOWN,
        tf02_min_m=tf02_min,
        tf02_max_m=tf02_max,
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

    def port_in_use(port: int, ours: tuple[int, ...]) -> None:
        protocol = int(v[port_setting(port, "PROTOCOL")])
        if protocol in MAVLINK_PROTOCOLS:
            plan.warnings.append(
                f"SERIAL{port} speaks {protocol_words(protocol)} now. If a telemetry radio, the remote control's "
                "link or a companion computer is on it, it stops working after the restart, and a ground station "
                "on it loses the drone.")
        elif protocol not in (PROTOCOL_NONE, *ours):
            plan.warnings.append(f"SERIAL{port} is for {protocol_words(protocol)} now. That stops after the restart.")

    # The proximity slots: the LiDAR is PRX1, a TF02 Pro looking ahead is type 4 in the first slot the LiDAR
    # leaves (an empty PRX1 before a used PRX2 stops arming).
    tf02_ahead = c.tf02_on and c.tf02_ahead
    prx1 = PRX_RPLIDAR if c.lidar_on else (PRX_RANGEFINDER if tf02_ahead else 0)
    prx2 = PRX_RANGEFINDER if c.lidar_on and tf02_ahead else 0
    why_prx = {PRX_RPLIDAR: "the RPLidar", PRX_RANGEFINDER: "the TF02 Pro as an obstacle sensor",
               0: "no obstacle sensor"}
    if c.lidar_on and port_setting(c.port, "PROTOCOL") not in v:
        plan.refusal = f"This drone has no SERIAL{c.port}. Choose another port."
        return plan
    want("PRX1_TYPE", prx1, why_prx[prx1])
    if c.lidar_on:
        port_in_use(c.port, (PROTOCOL_LIDAR360,))
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
    # PRX2 only as the TF02 Pro's slot: written when the TF02 needs it, or when it held the TF02 (type 4 beside
    # a TF02 as RNGFND1) and no longer does. Any other second sensor is left alone.
    old_prx2 = int(v.get("PRX2_TYPE", 0))
    tf02_now = int(v.get("RNGFND1_TYPE", 0)) == RNGFND_TF02
    prx2_after = old_prx2
    if prx2:
        if "PRX2_TYPE" not in v:
            plan.refusal = "This drone has no PRX2_TYPE: the LiDAR and the TF02 Pro need two obstacle sensor slots."
            return plan
        if old_prx2 not in (0, PRX_RANGEFINDER):
            plan.warnings.append(f"PRX2 is an obstacle sensor of type {old_prx2} now. The TF02 Pro takes its place.")
        want("PRX2_TYPE", prx2, why_prx[prx2])
        prx2_after = prx2
    elif old_prx2 == PRX_RANGEFINDER and (tf02_now or c.tf02_on):
        want("PRX2_TYPE", 0, "the TF02 Pro is not in this slot any more")
        prx2_after = 0

    if c.tf02_on:
        if "RNGFND1_TYPE" not in v:
            plan.refusal = "This drone has no RNGFND1_TYPE: its firmware has no rangefinder support."
            return plan
        if port_setting(c.tf02_port, "PROTOCOL") not in v:
            plan.refusal = f"This drone has no SERIAL{c.tf02_port}. Choose another port for the TF02 Pro."
            return plan
        if c.lidar_on and c.tf02_port == c.port:
            plan.refusal = f"The LiDAR and the TF02 Pro are both on SERIAL{c.port}. Each needs a port of its own."
            return plan
        old_type = int(v["RNGFND1_TYPE"])
        if old_type not in (0, RNGFND_TF02):
            plan.warnings.append(f"RNGFND1 is a rangefinder of type {old_type} now. The TF02 Pro takes its place.")
        want("RNGFND1_TYPE", RNGFND_TF02, "the TF02 Pro")
        port_in_use(c.tf02_port, (PROTOCOL_RANGEFINDER,))
        want(port_setting(c.tf02_port, "PROTOCOL"), PROTOCOL_RANGEFINDER, f"the TF02 Pro on SERIAL{c.tf02_port}")
        want(port_setting(c.tf02_port, "BAUD"), TF02_BAUD, f"the TF02 Pro's speed, {BAUD_WORDS[TF02_BAUD]}")
        want(port_setting(c.tf02_port, "RTSCTS"), 0, "no flow control for the TF02 Pro")
        others = [p for p in rangefinder_ports(v) if p != c.tf02_port]
        if others and int(v.get("RNGFND2_TYPE", 0)) == 0:
            # ArduPilot gives the serial rangefinders the rangefinder ports in order: an old one before the new
            # one would be the TF02's.
            for other in others:
                want(port_setting(other, "PROTOCOL"), PROTOCOL_NONE, f"the TF02 Pro is no longer on SERIAL{other}")
        elif others:
            plan.warnings.append(
                f"SERIAL{others[0]} is a rangefinder port too, and RNGFND2 is set up: ArduPilot gives the serial "
                "rangefinders these ports in order. Check that the TF02 Pro gets its own.")
        want("RNGFND1_ORIENT", ORIENT_AHEAD if c.tf02_ahead else ORIENT_DOWN,
             "looks ahead" if c.tf02_ahead else "looks down")
        low = _clamp(c.tf02_min_m, *TF02_LIMIT_M)
        high = _clamp(c.tf02_max_m, *TF02_LIMIT_M)
        if low >= high:
            plan.refusal = f"The TF02 Pro's range {low:g} to {high:g} m is empty. Make the far end larger."
            return plan
        names = range_settings(v)
        if names is None:
            plan.left_out.append("RNGFND1_MIN")
        else:
            near, far, per_m = names
            # 4.6 keeps whole centimetres.
            want(near, round(low * per_m) if per_m != 1.0 else round(low, 2), "the TF02 Pro's nearest reading")
            want(far, round(high * per_m) if per_m != 1.0 else round(high, 2), "the TF02 Pro's farthest reading")
    elif tf02_now:
        want("RNGFND1_TYPE", 0, "no TF02 Pro")

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

    sensor_after = bool(prx1) or bool(prx2_after)
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
    elif kind == PRX_RANGEFINDER:
        sensor = "A rangefinder as the obstacle sensor (PRX1_TYPE 4)"
    else:
        sensor = f"Obstacle sensor type {kind} (PRX1_TYPE)"
    parts = [sensor]
    if int(v.get("RNGFND1_TYPE", 0)) == RNGFND_TF02:
        orient = int(v["RNGFND1_ORIENT"]) if "RNGFND1_ORIENT" in v else None
        looks = {None: "", ORIENT_AHEAD: " looking ahead", ORIENT_DOWN: " looking down"}.get(
            orient, f" (RNGFND1_ORIENT {orient})")
        on = rangefinder_ports(v)
        parts.append("TF02 Pro" + looks + (f" on SERIAL{on[0]}" if on else ""))
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

