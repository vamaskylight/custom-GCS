"""How the payload release is wired, from the settings of this laptop.

Which output the servo is on, and the pulse widths that open and close it, are
properties of the airframe and cannot be read from a plan. The link reads them
when it builds a mission, and the window reads them when it reads a mission
back from the drone: a servo command is the payload release only when it is
this output and this pulse (vgcs.mission.read_downloaded_mission). Both must
see the same values, so both read them here.

Set on screen in Application Settings, General, Payload release. Until
2026-10-14 there was no place on screen for them, so every drone got output 9.
"""

from __future__ import annotations

from PySide6.QtCore import QSettings

from vgcs.map.app_settings import QS_APP, QS_ORG
from vgcs.mission.mission_plan import PayloadServo

KEY_CHANNEL = "mission/payload_servo_channel"
KEY_RELEASE_PWM = "mission/payload_servo_release_pwm"
KEY_RESET_PWM = "mission/payload_servo_reset_pwm"
KEY_HOLD_S = "mission/payload_servo_hold_s"
# ArduPilot numbers servo outputs 1 to 32 (SERVO1_FUNCTION to SERVO32_FUNCTION).
CHANNEL_RANGE = (1, 32)
# The pulse widths a servo takes, in microseconds.
PWM_RANGE_US = (800, 2200)
# How long the release stays open before the close pulse. 0: it stays open.
HOLD_RANGE_S = (0.0, 10.0)


def payload_servo_from_settings() -> PayloadServo:
    """The payload servo as set, or the documented defaults. A value out of its range counts as not set."""
    st = QSettings(QS_ORG, QS_APP)
    default = PayloadServo()

    def _num(key, fallback, cast, low, high):
        try:
            value = cast(st.value(key, fallback))
        except (TypeError, ValueError):
            return fallback
        return value if low <= value <= high else fallback

    return PayloadServo(
        channel=_num(KEY_CHANNEL, default.channel, int, *CHANNEL_RANGE),
        release_pwm=_num(KEY_RELEASE_PWM, default.release_pwm, int, *PWM_RANGE_US),
        reset_pwm=_num(KEY_RESET_PWM, default.reset_pwm, int, *PWM_RANGE_US),
        hold_s=_num(KEY_HOLD_S, default.hold_s, float, *HOLD_RANGE_S),
    )


def save_payload_servo(servo: PayloadServo) -> None:
    """Keep the payload servo for the next upload, and for reading a mission back."""
    st = QSettings(QS_ORG, QS_APP)
    st.setValue(KEY_CHANNEL, servo.channel)
    st.setValue(KEY_RELEASE_PWM, servo.release_pwm)
    st.setValue(KEY_RESET_PWM, servo.reset_pwm)
    st.setValue(KEY_HOLD_S, servo.hold_s)
