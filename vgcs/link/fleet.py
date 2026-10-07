"""Several drones at once (milestone M15, client requirement 14).

Each drone has its own connection and its own MavlinkThread, and they all run
side by side. One of them is the active drone: the window shows it in full
and its buttons command it, exactly as with a single drone. Making another
drone active rewires the window to that drone's thread. The others keep
running, so switching never drops a link.

What every drone reports is kept in a small summary here (mode, armed,
position, height, battery, GPS, link state, its last warning). The fleet panel
and the map draw all drones from these summaries, active or not.

A drone that is not active can still need the operator: its link can drop, or
the autopilot can warn (battery, fence, EKF). Those come out of `alert`, so the
window can put them in front of the operator whichever drone is on screen.

Each drone needs its own connection (its own port or address). Two drones on
one shared radio link (one port, two system ids) are not split apart here.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from PySide6.QtCore import QObject, Signal

# ArduPilot STATUSTEXT severity: 0 emergency ... 4 warning, 5 notice, 6 info.
# Warnings and worse from a drone that is not on screen are passed on.
ALERT_SEVERITY_MAX = 4

LINK_CONNECTING = "connecting"   # thread started, port not open yet
LINK_WAITING = "waiting"         # port open, no heartbeat yet
LINK_UP = "up"
LINK_LOST = "lost"               # was up, nothing heard for the watchdog time
LINK_CLOSED = "closed"           # the thread has ended


@dataclass
class VehicleSummary:
    """What one drone reports, kept small for the fleet panel and the map."""

    name: str
    connection: str
    link: str = LINK_CONNECTING
    sysid: int = 0
    mode: str = ""
    armed: bool = False
    lat: Optional[float] = None
    lon: Optional[float] = None
    alt_rel_m: Optional[float] = None
    heading_deg: Optional[float] = None
    groundspeed_mps: Optional[float] = None
    battery_pct: Optional[int] = None
    voltage_v: Optional[float] = None
    gps_fix: int = 0
    satellites: int = 0
    mission_text: str = ""
    last_text: str = ""
    last_text_severity: int = 7
    last_result: str = ""
    signing: str = ""               # vgcs/link/signing.py state, "" before the first report
    last_heard_mono: float = 0.0
    lost_since_mono: Optional[float] = None
    history: list = field(default_factory=list)   # recent alerts, newest last


def link_text(s: VehicleSummary, now: float | None = None) -> str:
    """The link state in words, for the fleet panel."""
    if s.link == LINK_UP:
        return f"OK (sys {s.sysid})" if s.sysid else "OK"
    if s.link == LINK_LOST:
        if s.lost_since_mono is not None:
            secs = (time.monotonic() if now is None else now) - s.lost_since_mono
            return f"LOST {secs:.0f} s"
        return "LOST"
    if s.link == LINK_WAITING:
        return "Waiting for heartbeat"
    if s.link == LINK_CLOSED:
        return "Disconnected"
    return "Connecting"


def battery_text(s: VehicleSummary) -> str:
    parts = []
    if s.battery_pct is not None and s.battery_pct >= 0:
        parts.append(f"{s.battery_pct} %")
    if s.voltage_v is not None and s.voltage_v > 0.5:
        parts.append(f"{s.voltage_v:.1f} V")
    return " ".join(parts) if parts else "N/A"


def gps_text(s: VehicleSummary) -> str:
    names = {0: "No GPS", 1: "No fix", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK Float", 6: "RTK Fixed"}
    name = names.get(int(s.gps_fix), f"fix {s.gps_fix}")
    return f"{name}, {s.satellites} sats" if s.gps_fix >= 2 else name


def altitude_text(s: VehicleSummary) -> str:
    return "N/A" if s.alt_rel_m is None else f"{s.alt_rel_m:.1f} m"


def signing_text(s: VehicleSummary) -> str:
    """Whether only signed commands reach the drone (M16), in a word or two."""
    return {
        "off": "off",
        "waiting": "waiting",
        "drone_signs": "signed",
        "drone_unsigned": "NOT signed",
        "key_mismatch": "OTHER KEY",
        "mavlink1": "MAVLink 1",
    }.get(s.signing, "")


def row_cells(s: VehicleSummary, now: float | None = None) -> list[str]:
    """One fleet panel row: name, link, mode, armed, height, battery, GPS, signing, mission, last message."""
    return [
        s.name,
        link_text(s, now),
        s.mode or "N/A",
        "ARMED" if s.armed else "disarmed",
        altitude_text(s),
        battery_text(s),
        gps_text(s),
        signing_text(s),
        s.mission_text or "",
        s.last_result or s.last_text or "",
    ]


class FleetVehicle(QObject):
    """One drone: its link thread and its summary."""

    alert = Signal(str, str)      # vehicle id, text

    def __init__(self, vid: str, name: str, connection: str, thread) -> None:
        super().__init__()
        self.vid = vid
        self.thread = thread
        self.summary = VehicleSummary(name=name, connection=connection)
        # Queued to this object's thread (the window's), so the summary is
        # only ever touched there.
        thread.link_up.connect(self._on_link_up)
        thread.link_down.connect(self._on_link_down)
        thread.heartbeat.connect(self._on_heartbeat)
        thread.link_timeout.connect(self._on_link_timeout)
        thread.telemetry.connect(self._on_telemetry)
        thread.mission_progress.connect(self._on_mission_progress)
        thread.action_result.connect(self._on_action_result)
        thread.mode_changed.connect(self._on_mode_changed)
        thread.finished.connect(self._on_finished)

    @property
    def name(self) -> str:
        return self.summary.name

    def start(self) -> None:
        self.summary.link = LINK_CONNECTING
        self.thread.start()

    def stop(self, wait_ms: int = 8000) -> None:
        self.thread.stop()
        if self.thread.isRunning():
            self.thread.wait(wait_ms)

    def is_running(self) -> bool:
        return bool(self.thread.isRunning())

    def _alert(self, text: str) -> None:
        self.summary.history = (self.summary.history + [text])[-20:]
        self.alert.emit(self.vid, text)

    # --- what the link thread says -------------------------------------
    def _on_link_up(self) -> None:
        self.summary.link = LINK_WAITING

    def _on_heartbeat(self, sysid: int, _compid: int, _mav: int) -> None:
        was_lost = self.summary.link == LINK_LOST
        self.summary.link = LINK_UP
        self.summary.sysid = int(sysid)
        self.summary.last_heard_mono = time.monotonic()
        self.summary.lost_since_mono = None
        if was_lost:
            self._alert("link back")

    def _on_link_timeout(self, _elapsed: float) -> None:
        if self.summary.link == LINK_UP:
            self.summary.link = LINK_LOST
            self.summary.lost_since_mono = time.monotonic()
            self._alert("link lost")

    def _on_link_down(self) -> None:
        was = self.summary.link
        self.summary.link = LINK_CLOSED
        if was in (LINK_UP, LINK_LOST):
            self._alert("link closed")

    def _on_finished(self) -> None:
        self.summary.link = LINK_CLOSED

    def _on_telemetry(self, kind: str, payload: object) -> None:
        d = payload if isinstance(payload, dict) else {}
        s = self.summary
        s.last_heard_mono = time.monotonic()
        if kind == "HEARTBEAT":
            s.armed = bool(d.get("armed", False))
            mode = str(d.get("mode_text", "") or "").strip()
            if mode:
                s.mode = mode
        elif kind == "GLOBAL_POSITION_INT":
            lat, lon = d.get("lat"), d.get("lon")
            if lat is not None and lon is not None and (abs(float(lat)) > 1e-9 or abs(float(lon)) > 1e-9):
                s.lat, s.lon = float(lat), float(lon)
            if d.get("relative_alt_m") is not None:
                s.alt_rel_m = float(d["relative_alt_m"])
            if d.get("hdg_deg") is not None:
                s.heading_deg = float(d["hdg_deg"])
            if d.get("groundspeed_mps") is not None:
                s.groundspeed_mps = float(d["groundspeed_mps"])
        elif kind == "SYS_STATUS":
            mv = int(d.get("voltage_mv", 0) or 0)
            if 0 < mv < 0xFFFF:
                s.voltage_v = mv / 1000.0
            pct = int(d.get("battery_remaining", -1))
            s.battery_pct = pct if pct >= 0 else s.battery_pct
        elif kind == "GPS_RAW_INT":
            s.gps_fix = int(d.get("fix_type", 0) or 0)
            s.satellites = int(d.get("satellites_visible", 0) or 0)
        elif kind == "SIGNING":
            was = s.signing
            s.signing = str(d.get("state", "") or "")
            if s.signing == "key_mismatch" and was != "key_mismatch":
                self._alert("signs with another key: it ignores VGCS's commands")
        elif kind == "STATUSTEXT":
            text = str(d.get("text", "") or "").strip()
            # 0 is EMERGENCY, so not "or 7": that would hide the worst ones.
            severity = int(d["severity"]) if d.get("severity") is not None else 7
            if text:
                s.last_text = text
                s.last_text_severity = severity
                s.last_result = ""
                if severity <= ALERT_SEVERITY_MAX:
                    self._alert(text)

    def _on_mission_progress(self, payload: object) -> None:
        d = payload if isinstance(payload, dict) else {}
        label = str(d.get("label", "") or "").strip()
        if label:
            self.summary.mission_text = label

    def _on_action_result(self, action: str, ok: bool, detail: str) -> None:
        self.summary.last_result = f"{action}: {'OK' if ok else 'FAILED'} {detail}".strip()

    def _on_mode_changed(self, mode: str, ok: bool) -> None:
        self.summary.last_result = f"mode {mode}: {'OK' if ok else 'FAILED'}"


class Fleet(QObject):
    """The drones of one VGCS session, in the order they were added."""

    alert = Signal(str, str)           # vehicle id, text
    vehicles_changed = Signal()
    active_changed = Signal(str)       # vehicle id, or "" for none

    def __init__(self, thread_factory: Callable[[str, float], object] | None = None) -> None:
        super().__init__()
        if thread_factory is None:
            from vgcs.link.mavlink_thread import MavlinkThread

            def thread_factory(connection: str, timeout_s: float):
                return MavlinkThread(connection, timeout_s=timeout_s)
        self._make_thread = thread_factory
        self._vehicles: dict[str, FleetVehicle] = {}
        self._next = 1
        self.active_id = ""

    # --- members ---------------------------------------------------------
    def vehicles(self) -> list[FleetVehicle]:
        return list(self._vehicles.values())

    def get(self, vid: str) -> FleetVehicle | None:
        return self._vehicles.get(vid)

    def active(self) -> FleetVehicle | None:
        return self._vehicles.get(self.active_id)

    def find_by_thread(self, thread) -> FleetVehicle | None:
        for v in self._vehicles.values():
            if v.thread is thread:
                return v
        return None

    def find_by_connection(self, connection: str) -> FleetVehicle | None:
        key = _connection_key(connection)
        for v in self._vehicles.values():
            if _connection_key(v.summary.connection) == key:
                return v
        return None

    def add(self, connection: str, *, name: str = "", timeout_s: float = 2.0, start: bool = True) -> FleetVehicle:
        """Connect one more drone. A closed drone on the same connection is replaced.

        Two running drones cannot share a connection: the second would fail
        to open the port, or both would read the same drone.
        """
        connection = str(connection or "").strip()
        if not connection:
            raise ValueError("enter a connection, for example udpin:0.0.0.0:14551")
        same = self.find_by_connection(connection)
        if same is not None:
            if same.is_running():
                raise ValueError(f"{same.name} already uses {connection}")
            vid, default_name = same.vid, same.name
            self._drop(same)
        else:
            vid, default_name = f"v{self._next}", f"Drone {self._next}"
            self._next += 1
        name = str(name or "").strip() or default_name
        if any(v.name == name for v in self._vehicles.values()):
            raise ValueError(f"the name {name} is already used")
        vehicle = FleetVehicle(vid, name, connection, self._make_thread(connection, float(timeout_s)))
        vehicle.alert.connect(self.alert)
        self._vehicles[vid] = vehicle
        if not self.active_id:
            self.active_id = vid
            self.active_changed.emit(vid)
        if start:
            vehicle.start()
        self.vehicles_changed.emit()
        return vehicle

    def remove(self, vid: str) -> None:
        """Disconnect one drone and take it out of the fleet."""
        vehicle = self._vehicles.get(vid)
        if vehicle is None:
            return
        vehicle.stop()
        self._drop(vehicle)
        self.vehicles_changed.emit()

    def _drop(self, vehicle: FleetVehicle) -> None:
        self._vehicles.pop(vehicle.vid, None)
        if self.active_id == vehicle.vid:
            self.active_id = ""
            self.active_changed.emit("")

    def set_active(self, vid: str) -> FleetVehicle | None:
        vehicle = self._vehicles.get(vid)
        if vehicle is None or vid == self.active_id:
            return vehicle
        self.active_id = vid
        self.active_changed.emit(vid)
        return vehicle

    def stop_all(self) -> None:
        for vehicle in list(self._vehicles.values()):
            vehicle.stop()

    # --- commands to several drones --------------------------------------
    def connected(self) -> list[FleetVehicle]:
        """Drones whose link is up (or lost for a moment: they may still hear us)."""
        return [v for v in self._vehicles.values()
                if v.is_running() and v.summary.link in (LINK_UP, LINK_LOST)]

    def command_all(self, command: str) -> list[str]:
        """Send one command to every connected drone. Returns the names it went to.

        Only safe, stopping commands: hold position, return home, land.
        Missions stay per drone, so two drones are never sent the same route.
        """
        sent = []
        for vehicle in self.connected():
            t = vehicle.thread
            if command == "hold":
                t.queue_mission_pause()
            elif command == "rtl":
                t.queue_mode_change("RTL")
            elif command == "land":
                t.queue_auto_land()
            else:
                raise ValueError(f"unknown fleet command {command!r}")
            vehicle.summary.last_result = f"{command.upper()} sent"
            sent.append(vehicle.name)
        return sent


def _connection_key(connection: str) -> str:
    return "".join(str(connection or "").split()).lower()
