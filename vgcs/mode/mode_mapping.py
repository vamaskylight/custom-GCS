"""ArduPilot custom_mode to human-readable mode names."""

from __future__ import annotations

AP_COPTER_MODE_MAP = {
    0: "STABILIZE",
    1: "ACRO",
    2: "ALT_HOLD",
    3: "AUTO",
    4: "GUIDED",
    5: "LOITER",
    6: "RTL",
    7: "CIRCLE",
    9: "LAND",
    11: "DRIFT",
    13: "SPORT",
    14: "FLIP",
    15: "AUTOTUNE",
    16: "POSHOLD",
    17: "BRAKE",
    18: "THROW",
    19: "AVOID_ADSB",
    20: "GUIDED_NOGPS",
    21: "SMART_RTL",
}

AP_PLANE_MODE_MAP = {
    0: "MANUAL",
    1: "CIRCLE",
    2: "STABILIZE",
    3: "TRAINING",
    4: "ACRO",
    5: "FBWA",
    6: "FBWB",
    7: "CRUISE",
    8: "AUTOTUNE",
    10: "AUTO",
    11: "RTL",
    12: "LOITER",
    15: "GUIDED",
    16: "INITIALISING",
    17: "QSTABILIZE",
    18: "QHOVER",
    19: "QLOITER",
    20: "QLAND",
    21: "QRTL",
    22: "QAUTOTUNE",
    23: "QACRO",
}

AP_ROVER_MODE_MAP = {
    0: "MANUAL",
    1: "ACRO",
    3: "STEERING",
    4: "HOLD",
    5: "LOITER",
    6: "FOLLOW",
    7: "SIMPLE",
    10: "AUTO",
    11: "RTL",
    12: "SMART_RTL",
    15: "GUIDED",
    16: "INITIALISING",
}


def human_mode_name(
    *, vehicle_type: int | None, custom_mode: int | None
) -> str:
    """Return best-effort mode text from MAVLink vehicle type + custom_mode."""
    if custom_mode is None:
        return "—"
    mode = int(custom_mode)
    # MAV_TYPE_QUADROTOR/HEX/OCTO share Copter mapping in ArduPilot.
    if vehicle_type in {2, 13, 14, 15, 16}:
        return AP_COPTER_MODE_MAP.get(mode, f"MODE({mode})")
    # MAV_TYPE_FIXED_WING
    if vehicle_type == 1:
        return AP_PLANE_MODE_MAP.get(mode, f"MODE({mode})")
    # MAV_TYPE_GROUND_ROVER
    if vehicle_type == 10:
        return AP_ROVER_MODE_MAP.get(mode, f"MODE({mode})")
    # Unknown vehicle type; still show numeric mode.
    return f"MODE({mode})"

