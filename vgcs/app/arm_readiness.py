"""Authoritative arm readiness from ``SYS_STATUS``, not from guesswork.

Field report (2026-08-24): "in the dashboard VGCS say loiter mode ready to arm
but actual drone is not ready to Arm".

The old gate inferred readiness: ``system_status >= MAV_STATE_STANDBY`` plus a
3D GPS fix plus 12 s since link-up plus no *recent* PreArm STATUSTEXT. Every
term of that is wrong for ArduPilot:

* ``system_status`` stays ``MAV_STATE_STANDBY`` while PreArm checks fail — it
  never reports arm readiness at all.
* ``AP_Arming::update()`` runs the PreArm checks continuously but only *prints*
  the failure every ``PREARM_DISPLAY_PERIOD`` (30 s), so "no PreArm text lately"
  means nothing.
* A 3D fix satisfies exactly one of the ~30 PreArm checks.

ArduPilot does publish the answer, in every ``SYS_STATUS``
(``GCS.cpp::get_sensor_status_flags``)::

    control_sensors_present |= MAV_SYS_STATUS_PREARM_CHECK;
    if (AP::arming().get_enabled_checks()) {
        control_sensors_enabled |= MAV_SYS_STATUS_PREARM_CHECK;
        if (hal.util->get_soft_armed() || AP_Notify::flags.pre_arm_check) {
            control_sensors_health |= MAV_SYS_STATUS_PREARM_CHECK;
        }
    }

So: *enabled* means the vehicle is running arming checks and its verdict is
meaningful; *health* is that verdict. When the bit is not enabled
(``ARMING_CHECK=0``) there is no verdict to read and the caller must fall back
to its heuristic.
"""

from __future__ import annotations

from dataclasses import dataclass

# common.xml: MAV_SYS_STATUS_PREARM_CHECK = 0x10000000, "pre-arm check status.
# Always healthy when armed".
MAV_SYS_STATUS_PREARM_CHECK = 0x10000000

# SYS_STATUS is requested at 2 Hz; three missed frames means the verdict on
# screen is no longer live and the heuristic takes over again.
PREARM_HEALTH_MAX_AGE_S = 6.0


@dataclass(frozen=True)
class PrearmHealth:
    """The vehicle-reported PreArm verdict, with the freshness to trust it."""

    reported: bool
    """True when the vehicle runs arming checks and publishes their verdict."""

    passing: bool
    """The verdict itself. Only meaningful while :attr:`reported`."""

    updated_mono: float

    def is_authoritative(self, now: float, max_age_s: float = PREARM_HEALTH_MAX_AGE_S) -> bool:
        """Whether callers may use :attr:`passing` instead of guessing."""
        if not self.reported:
            return False
        return (float(now) - float(self.updated_mono)) <= float(max_age_s)


def parse_prearm_health(
    *,
    sensors_present: int,
    sensors_enabled: int,
    sensors_health: int,
    now: float,
) -> PrearmHealth:
    """Read the PreArm verdict out of a ``SYS_STATUS`` sensor-flag triplet."""
    bit = MAV_SYS_STATUS_PREARM_CHECK
    present = bool(int(sensors_present) & bit)
    enabled = bool(int(sensors_enabled) & bit)
    healthy = bool(int(sensors_health) & bit)
    return PrearmHealth(
        reported=present and enabled,
        passing=healthy,
        updated_mono=float(now),
    )
