"""Shared helpers for MainWindow mixins."""

from __future__ import annotations

import struct

from pymavlink import mavutil


def _mavlink_autopilot_label(ap: int) -> str:
    m = mavutil.mavlink
    table = {
        int(m.MAV_AUTOPILOT_GENERIC): "Generic",
        int(m.MAV_AUTOPILOT_ARDUPILOTMEGA): "ArduPilot",
        int(m.MAV_AUTOPILOT_OPENPILOT): "OpenPilot",
        int(m.MAV_AUTOPILOT_PX4): "PX4",
        int(m.MAV_AUTOPILOT_INVALID): "Invalid",
    }
    try:
        return table.get(int(ap), f"Autopilot {int(ap)}")
    except Exception:
        return "—"

def _mavlink_vehicle_type_label(vt: int) -> str:
    m = mavutil.mavlink
    table = {
        int(m.MAV_TYPE_GENERIC): "Generic",
        int(m.MAV_TYPE_FIXED_WING): "Fixed wing",
        int(m.MAV_TYPE_QUADROTOR): "Quadrotor",
        int(m.MAV_TYPE_COAXIAL): "Coaxial",
        int(m.MAV_TYPE_HELICOPTER): "Helicopter",
        int(m.MAV_TYPE_ANTENNA_TRACKER): "Antenna tracker",
        int(m.MAV_TYPE_GCS): "GCS",
        int(m.MAV_TYPE_AIRSHIP): "Airship",
        int(m.MAV_TYPE_FREE_BALLOON): "Balloon",
        int(m.MAV_TYPE_ROCKET): "Rocket",
        int(m.MAV_TYPE_GROUND_ROVER): "Rover",
        int(m.MAV_TYPE_SURFACE_BOAT): "Boat",
        int(m.MAV_TYPE_SUBMARINE): "Submarine",
        int(m.MAV_TYPE_HEXAROTOR): "Hexacopter",
        int(m.MAV_TYPE_OCTOROTOR): "Octocopter",
        int(m.MAV_TYPE_TRICOPTER): "Tricopter",
        int(m.MAV_TYPE_VTOL_DUOROTOR): "VTOL (duo)",
        int(m.MAV_TYPE_VTOL_QUADROTOR): "VTOL (quad)",
        int(m.MAV_TYPE_VTOL_TILTROTOR): "VTOL tilt",
    }
    try:
        return table.get(int(vt), f"Vehicle {int(vt)}")
    except Exception:
        return "—"

def _settings_truthy(val: object, default: bool = False) -> bool:
    if val is None:
        return default
    if isinstance(val, bool):
        return val
    s = str(val).strip().lower()
    if s in ("1", "true", "yes", "on"):
        return True
    if s in ("0", "false", "no", "off", ""):
        return False
    return default
