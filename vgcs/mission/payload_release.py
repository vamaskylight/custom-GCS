"""Will the drone move the servo of the payload release? (2026-10-14)

A waypoint with "Drop payload" puts DO_SET_SERVO commands into the mission, on
the servo output set in Application Settings, General, Payload release.
ArduPilot moves an output for a mission only when the output is free for it
(AP_ServoRelayEvents::do_set_servo, ArduPilot 4.7): its function
(SERVOn_FUNCTION) is 0 (none), 1 (manual), 22 or 23 (sprayer), 28 (gripper),
or 51 to 66 or 140 to 155 (an RC input passed through). On any other output
it says "ServoRelayEvent: Channel n is already in use" and goes on with the
mission: the drone flies the route, and the payload stays on.

VGCS reads the function of the output set with the drone's other settings
(vgcs/app/window/params_mixin.py). Not read: one line in the log, as the
other mission checks do. VGCS cannot tell an output the drone lacks from a
setting not read yet, so that is not checked.

Pure Python, so it is tested without a link.
"""

from __future__ import annotations

from vgcs.mission.mission_plan import _wp_drop_payload

# The output functions ArduPilot lets a mission move (see above).
FREE_FUNCTIONS = frozenset({0, 1, 22, 23, 28, *range(51, 67), *range(140, 156)})


def servo_function_param(channel: int) -> str:
    """The drone's setting that says what servo output ``channel`` is used for."""
    return f"SERVO{channel}_FUNCTION"


def motor_number(function: int) -> int | None:
    """The motor an output function drives, or None.

    ArduPilot: motors 1 to 8 are functions 33 to 40, 9 to 12 are 82 to 85,
    13 to 32 are 160 to 179.
    """
    if 33 <= function <= 40:
        return function - 32
    if 82 <= function <= 85:
        return function - 73
    if 160 <= function <= 179:
        return function - 147
    return None


def _wp_words(numbers: list[int]) -> str:
    """"WP 3", "WP 2 and 5", "WP 1, 2, 3 and 9 more", as the plan names its rows."""
    shown = [str(n) for n in numbers[:3]]
    more = len(numbers) - len(shown)
    if more > 0:
        return f"WP {', '.join(shown)} and {more} more"
    if len(shown) == 1:
        return f"WP {shown[0]}"
    return f"WP {', '.join(shown[:-1])} and {shown[-1]}"


def check_payload_release(waypoints, servo, params) -> tuple[list[str], list[str]]:
    """(errors, unchecked) for the payload release of a plan. Nothing when no waypoint drops.

    ``servo``: vgcs.mission.mission_plan.PayloadServo. ``params``: the drone's
    settings as read (name: value).
    """
    drops = [n for n, wp in enumerate(waypoints, start=1) if _wp_drop_payload(wp)]
    if not drops:
        return [], []
    channel = servo.channel
    key = servo_function_param(channel)
    known = {k.upper(): v for k, v in params.items()}
    if key not in known:
        return [], [f"Payload release on servo output {channel} ({key} not read from the drone)"]
    function = int(known[key])
    if function in FREE_FUNCTIONS:
        return [], []
    motor = motor_number(function)
    what = f"motor {motor}" if motor is not None else f"function {function}"
    return [
        f"Servo output {channel} is used for {what} on the drone ({key} = {function}). The drone will not move "
        f"it for the payload release at {_wp_words(drops)}: nothing would drop (the drone says \"Channel "
        f"{channel} is already in use\"). Set the output the release servo is wired to: Application Settings, "
        "General, Payload release."
    ], []
