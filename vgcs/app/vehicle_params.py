"""The ArduCopter settings VGCS reads, under the names each firmware uses.

Found by the simulator test (M17, 2026-10-07) on ArduCopter 4.7.0, the
firmware of one of the client's drones. Three of the settings VGCS asked for
do not exist there:

    4.6 name        4.7 name          what changed
    WPNAV_SPEED     WP_SPD            cm/s became m/s
    RTL_ALT         RTL_ALT_M         centimetres became metres
    ARMING_CHECK    ARMING_SKIPCHK    "checks to run" (1 = all) became
                                      "checks to skip" (0 = none)

So both names are asked for, and the "Set param" list keeps the ones the drone
answered. Asking for a name the drone lacks costs nothing any more: reading is
answered in the background (vgcs/link/confirmations.py).

Pure Python, no Qt.
"""

from __future__ import annotations

RENAMED_IN_4_7 = {
    "WPNAV_SPEED": "WP_SPD",
    "RTL_ALT": "RTL_ALT_M",
    "ARMING_CHECK": "ARMING_SKIPCHK",
}

# The list under "Set param", in this order. Both names of a renamed setting.
EDITABLE = (
    "WPNAV_SPEED",
    "WP_SPD",
    "RTL_ALT",
    "RTL_ALT_M",
    "FENCE_ENABLE",
    "FENCE_RADIUS",
    "ARMING_CHECK",
    "ARMING_SKIPCHK",
    "ACRO_OPTIONS",
    "ACRO_TRAINER",
    "SIMPLE",
    "SUPER_SIMPLE",
)

# Read as well, because VGCS checks them: the 10 km mission check
# (vgcs/mission/long_range_safety.py) and the motor test.
CHECKED = (
    "FENCE_ACTION",
    "FENCE_TYPE",
    "FENCE_ALT_MAX",
    "FS_GCS_ENABLE",
    "FS_THR_ENABLE",
    "BATT_FS_LOW_ACT",
    "BATT_FS_CRT_ACT",
    "BATT_LOW_VOLT",
    "BATT_LOW_MAH",
    "FRAME_CLASS",
)

# Read once the drone has answered, and again on "Refresh params". The mission
# check used to see only what the operator had refreshed by hand, so it said
# "not read yet" for the RTL height and the failsafes and checked nothing.
READ_ON_CONNECT = EDITABLE + CHECKED


def editable_on_this_drone(known: dict) -> list[str]:
    """The "Set param" names this drone has.

    Before anything has been read, the whole list: nothing is known yet, and an
    empty list would look like a drone without settings.
    """
    keys = {str(k).strip().upper() for k in known}
    if not keys.intersection(EDITABLE):
        return list(EDITABLE)
    return [name for name in EDITABLE if name in keys]
