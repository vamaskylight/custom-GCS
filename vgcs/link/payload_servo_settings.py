"""How the payload release is wired, from the settings of this laptop.

Which output the servo is on, and the pulse widths that open and close it, are
properties of the airframe and cannot be read from a plan. The link reads them
when it builds a mission, and the window reads them when it reads a mission
back from the drone: a servo command is the payload release only when it is
this output and this pulse (vgcs.mission.read_downloaded_mission). Both must
see the same values, so both read them here.
"""

from __future__ import annotations

from PySide6.QtCore import QSettings

from vgcs.map.app_settings import QS_APP, QS_ORG
from vgcs.mission.mission_plan import PayloadServo


def payload_servo_from_settings() -> PayloadServo:
    """The payload servo as set under mission/payload_servo_*, or the documented defaults."""
    st = QSettings(QS_ORG, QS_APP)

    def _num(key, default, cast):
        try:
            return cast(st.value(key, default))
        except (TypeError, ValueError):
            return cast(default)

    return PayloadServo(
        channel=_num("mission/payload_servo_channel", 9, int),
        release_pwm=_num("mission/payload_servo_release_pwm", 1900, int),
        reset_pwm=_num("mission/payload_servo_reset_pwm", 1100, int),
        hold_s=_num("mission/payload_servo_hold_s", 1.0, float),
    )
