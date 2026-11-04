"""Follow Me's drone in VGCS: the active drone's link (MavlinkThread), as object_follow.FollowDrone wants it.

The app's VehicleFollowDrone (apk/custom/src/Skydroid/VehicleFollowDrone.*) on VGCS's link: the autopilot's
HEARTBEAT (mode, armed, type), GPS_RAW_INT (a 3D fix) and GLOBAL_POSITION_INT (the place, the height above the
take-off point, the speed over the ground, the heading) make the telemetry. Guided is asked with VGCS's own mode
change, and the speed goes out as SET_POSITION_TARGET_LOCAL_NED (MavlinkThread.queue_follow_speed).

Follow keeps the drone it was started on: when VGCS's active drone changes (a fleet), other_drone is said, and
Follow ends without a word to either drone.
"""

from __future__ import annotations

import math

from vgcs.app.follow_guide import (
    COPTER_BRAKE,
    COPTER_GUIDED,
    COPTER_LOITER,
    COPTER_POSHOLD,
    Drone,
    Mode,
)
from vgcs.app.object_follow import FollowDrone

# MAV_TYPE of a multicopter: quadrotor, coaxial, hexarotor, octorotor, tricopter, dodecarotor, decarotor.
_MULTIROTORS = frozenset({2, 3, 13, 14, 15, 29, 35})
_ARDUPILOT = 3          # MAV_AUTOPILOT_ARDUPILOTMEGA
# VGCS hears no landed state: the drone counts as flying when it is armed and this high above its take-off point
# (Follow starts at follow_guide.MIN_HEIGHT_M and ends under END_HEIGHT_M anyway).
FLYING_HEIGHT_M = 1.0


def _number(value) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return math.nan
    return number


class LinkFollowDrone(FollowDrone):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._thread = None
        self._reset()

    def _reset(self) -> None:
        self._custom_mode: int | None = None
        self._armed = False
        self._gps3d = False
        self._vehicle_type: int | None = None
        self._autopilot: int | None = None
        self._mode_text = ""

    def set_thread(self, thread) -> None:
        """VGCS's active drone's link (None: none). Another one than before: other_drone."""
        if thread is self._thread:
            return  # the same drone named again: nothing changes
        if self._thread is not None:
            try:
                self._thread.telemetry.disconnect(self._on_telemetry)
            except (RuntimeError, TypeError):
                pass
        self._thread = thread
        self._reset()  # nothing of the last drone counts for this one
        if thread is not None:
            thread.telemetry.connect(self._on_telemetry)
        self.other_drone.emit()

    def thread(self):
        return self._thread

    def _is_copter(self) -> bool:
        return self._autopilot == _ARDUPILOT and self._vehicle_type in _MULTIROTORS

    def _mode(self) -> Mode:
        if not self._is_copter():
            return Mode.OTHER
        if self._custom_mode == COPTER_GUIDED:
            return Mode.GUIDED
        if self._custom_mode in (COPTER_LOITER, COPTER_POSHOLD, COPTER_BRAKE):
            return Mode.HOLDING
        return Mode.OTHER

    def _on_telemetry(self, msg_type: str, payload) -> None:
        if not isinstance(payload, dict):
            return
        if msg_type == "HEARTBEAT":
            self._custom_mode = int(payload.get("custom_mode") or 0)
            self._armed = bool(payload.get("armed"))
            self._vehicle_type = int(payload.get("vehicle_type") or 0)
            self._autopilot = int(payload.get("autopilot") or 0)
            self._mode_text = str(payload.get("mode_text") or "")
        elif msg_type == "GPS_RAW_INT":
            self._gps3d = int(payload.get("fix_type") or 0) >= 3
        elif msg_type == "GLOBAL_POSITION_INT":
            lat, lon = _number(payload.get("lat")), _number(payload.get("lon"))
            if not (math.isfinite(lat) and math.isfinite(lon)) or (lat == 0.0 and lon == 0.0):
                return  # ArduPilot sends this without a position too, for the height alone
            height = _number(payload.get("relative_alt_m"))
            self.telemetry.emit(Drone(
                lat_deg=lat, lon_deg=lon, height_m=height,
                north_mps=_number(payload.get("vn_mps")), east_mps=_number(payload.get("ve_mps")),
                heading_deg=_number(payload.get("hdg_deg")),
                armed=self._armed, flying=self._armed and height >= FLYING_HEIGHT_M,
                gps3d=self._gps3d, mode=self._mode()))

    def _running(self) -> bool:
        thread = self._thread
        if thread is None:
            return False
        running = getattr(thread, "isRunning", None)
        return bool(running()) if callable(running) else True

    def has_drone(self) -> bool:
        return self._running()

    def why_not(self) -> str:
        if not self._running():
            return "Follow cannot start: no drone is connected."
        if self._autopilot is not None and not self._is_copter():
            return "Follow works with an ArduPilot multicopter only."
        return ""

    def mode_name(self) -> str:
        return self._mode_text or "an unknown mode"

    def ask_guided(self) -> None:
        if self._running() and self._is_copter():
            self._thread.queue_mode_change("GUIDED")

    def send_speed(self, north_mps: float, east_mps: float, heading_deg: float) -> None:
        if self._running() and self._is_copter():
            self._thread.queue_follow_speed(north_mps, east_mps, heading_deg)
