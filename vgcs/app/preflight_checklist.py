"""The pre-arm checklist shown when a vehicle connects.

Requested 2026-09-01: "when our drone is connected then one popup should come
and in that popup battery voltage, gps sats and motors are ready or not ... if
sats are good then in front of sats a right button is automatically visible".

The design rule here is the one already established in :mod:`vgcs.app.
arm_readiness`: **do not invent a verdict the vehicle already publishes.**
ArduPilot runs ~30 arming checks and reports the result in every ``SYS_STATUS``.
A GCS that greens a tick because it likes the satellite count, while the vehicle
is refusing to arm for a reason the GCS never modelled, is worse than no
checklist — it tells the operator they are ready when they are not. That exact
failure was reported on 2026-08-24 ("VGCS say ready to arm but actual drone is
not ready to Arm").

So the vehicle's own verdict is the headline row and the only authoritative one.
The battery and GPS rows are advisory: they exist because the operator asked to
see those numbers, and their thresholds are for spotting an obviously-unfit
aircraft, not for deciding airworthiness.
"""

from __future__ import annotations

from dataclasses import dataclass

# Advisory thresholds. These do NOT decide whether the aircraft may fly — the
# autopilot does. They flag the obvious cases an operator would want to see
# before walking out to the pad.
GPS_MIN_SATS_ADVISORY = 6          # ArduPilot's own GPS_MIN_SATS default
GPS_FIX_3D = 3                     # GPS_FIX_TYPE_3D_FIX
HDOP_ADVISORY_MAX = 2.0            # above this, position quality is poor
BATTERY_REMAINING_ADVISORY_PCT = 30.0
# Below this, no flight pack is attached — the autopilot is running on USB and
# whatever the voltage sense reads is not a battery. Even a 2S pack sits above
# 6 V, so 5 V cannot be a real one.
BATTERY_MIN_PLAUSIBLE_V = 5.0

# ArduPilot publishes motor/ESC health in the same SYS_STATUS sensor triplet as
# everything else. MOTOR_OUTPUTS covers the servo/ESC output path; PROPULSION is
# reported by firmware with ESC telemetry and says more when present, so it is
# preferred. Neither is a substitute for actually spinning the motors — see
# _motors_check.
MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS = 0x00008000
MAV_SYS_STATUS_SENSOR_PROPULSION = 0x40000000

STATUS_PASS = "pass"
STATUS_FAIL = "fail"
STATUS_UNKNOWN = "unknown"


@dataclass(frozen=True)
class PreflightCheck:
    """One row of the checklist."""

    key: str
    label: str
    status: str
    detail: str
    authoritative: bool = False
    """True only for the vehicle's own verdict. Advisory rows say so, so a
    green tick beside 'Battery' is never mistaken for permission to fly."""

    @property
    def tick(self) -> str:
        return {STATUS_PASS: "OK", STATUS_FAIL: "X"}.get(self.status, "?")


def build_preflight_checks(
    *,
    prearm_reported: bool = False,
    prearm_passing: bool = False,
    prearm_authoritative: bool = False,
    prearm_message: str = "",
    gps_fix_type: int | None = None,
    gps_sats: int | None = None,
    gps_hdop: float | None = None,
    battery_voltage_v: float | None = None,
    battery_remaining_pct: float | None = None,
    sensors_present: int | None = None,
    sensors_enabled: int | None = None,
    sensors_health: int | None = None,
) -> list[PreflightCheck]:
    """Assemble the checklist rows. Pure — no Qt, no telemetry plumbing."""
    checks: list[PreflightCheck] = []

    # --- the vehicle's verdict: the only row that decides anything --------- #
    if prearm_authoritative and prearm_reported:
        if prearm_passing:
            checks.append(
                PreflightCheck(
                    "prearm", "Vehicle arming checks", STATUS_PASS,
                    "Autopilot reports all pre-arm checks passing",
                    authoritative=True,
                )
            )
        else:
            detail = prearm_message.strip() or "Autopilot is refusing to arm"
            checks.append(
                PreflightCheck(
                    "prearm", "Vehicle arming checks", STATUS_FAIL, detail,
                    authoritative=True,
                )
            )
    else:
        checks.append(
            PreflightCheck(
                "prearm", "Vehicle arming checks", STATUS_UNKNOWN,
                "No verdict from the autopilot yet"
                + (" (ARMING_CHECK may be disabled)" if prearm_reported is False else ""),
                authoritative=True,
            )
        )

    # --- advisory rows ---------------------------------------------------- #
    checks.append(_gps_check(gps_fix_type, gps_sats, gps_hdop))
    checks.append(_battery_check(battery_voltage_v, battery_remaining_pct))
    checks.append(_motors_check(sensors_present, sensors_enabled, sensors_health))
    return checks


def _gps_check(fix_type, sats, hdop) -> PreflightCheck:
    fix = _i(fix_type)
    n = _i(sats)
    if fix is None and n is None:
        return PreflightCheck("gps", "GPS", STATUS_UNKNOWN, "No GPS telemetry yet")
    bits: list[str] = []
    if n is not None:
        bits.append(f"{n} satellites")
    if fix is not None:
        bits.append("3D fix" if fix >= GPS_FIX_3D else f"fix type {fix} (no 3D fix)")
    h = _f(hdop)
    if h is not None:
        bits.append(f"HDOP {h:.2f}")
    ok = (
        fix is not None
        and fix >= GPS_FIX_3D
        and n is not None
        and n >= GPS_MIN_SATS_ADVISORY
        and (h is None or h <= HDOP_ADVISORY_MAX)
    )
    return PreflightCheck(
        "gps", "GPS", STATUS_PASS if ok else STATUS_FAIL, " · ".join(bits)
    )


def _battery_check(voltage_v, remaining_pct) -> PreflightCheck:
    """Battery row.

    The percentage cannot be trusted on its own. ArduPilot derives
    ``battery_remaining`` from consumed mAh against ``BATT_CAPACITY``, so with
    nothing drawn it reports ~full — including when there is no pack at all.
    Field report 2026-09-02: USB only, no battery connected, and this row showed
    a green tick beside "0.39 V · 99%".

    So voltage is checked first, as the test of whether a battery is even
    present, and the percentage is only meaningful once it is.
    """
    v = _f(voltage_v)
    pct = _f(remaining_pct)
    if v is None and pct is None:
        return PreflightCheck("battery", "Battery", STATUS_UNKNOWN, "No battery telemetry yet")

    if v is not None and v < BATTERY_MIN_PLAUSIBLE_V:
        detail = f"No battery connected ({v:.2f} V)"
        if pct is not None:
            detail += f" — the {pct:.0f}% reading is not meaningful on USB power"
        return PreflightCheck("battery", "Battery", STATUS_FAIL, detail)

    bits: list[str] = []
    if v is not None:
        bits.append(f"{v:.2f} V")
    if pct is not None:
        bits.append(f"{pct:.0f}%")
    if pct is not None:
        ok = pct >= BATTERY_REMAINING_ADVISORY_PCT
    else:
        ok = True
        bits.append("no capacity estimate — voltage only")
    return PreflightCheck(
        "battery", "Battery", STATUS_PASS if ok else STATUS_FAIL, " · ".join(bits)
    )


def _motors_check(present, enabled, health) -> PreflightCheck:
    """Motor / ESC output health, as the vehicle reports it.

    Requested 2026-09-02: "like battery, GPS, motor test should be there in the
    PreArm check".

    This is a HEALTH READOUT, not a motor test. It says the autopilot's output
    path and ESCs are reporting themselves healthy; it does not spin anything,
    so it cannot detect a reversed motor, a swapped output or a bad prop. A row
    claiming "motors tested" on the strength of a status bit would be a lie an
    operator might fly on. Spinning motors is a deliberate, guarded action and
    belongs on its own control, not in a popup that opens by itself next to an
    armed aircraft.
    """
    p, e, h = _i(present), _i(enabled), _i(health)
    if p is None or e is None or h is None:
        return PreflightCheck(
            "motors", "Motors / ESCs", STATUS_UNKNOWN, "No motor telemetry yet"
        )
    for bit, label in (
        (MAV_SYS_STATUS_SENSOR_PROPULSION, "Propulsion"),
        (MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS, "Motor outputs"),
    ):
        if (p & bit) and (e & bit):
            healthy = bool(h & bit)
            return PreflightCheck(
                "motors",
                "Motors / ESCs",
                STATUS_PASS if healthy else STATUS_FAIL,
                f"{label} reported {'healthy' if healthy else 'UNHEALTHY'} "
                "(status only — motors not spun)",
            )
    return PreflightCheck(
        "motors", "Motors / ESCs", STATUS_UNKNOWN,
        "This autopilot does not report motor health",
    )


def checklist_is_ready(checks: list[PreflightCheck]) -> bool:
    """Ready only when the AUTHORITATIVE row passes.

    A failing advisory row is worth showing but must not, on its own, claim the
    aircraft cannot fly — and a passing one must never claim it can.
    """
    for c in checks:
        if c.authoritative:
            return c.status == STATUS_PASS
    return False


def checklist_summary(checks: list[PreflightCheck]) -> str:
    if checklist_is_ready(checks):
        advisory_fail = [c.label for c in checks if not c.authoritative and c.status == STATUS_FAIL]
        if advisory_fail:
            return "Vehicle reports ready to arm — but check " + ", ".join(advisory_fail)
        return "Vehicle reports ready to arm"
    for c in checks:
        if c.authoritative and c.status == STATUS_FAIL:
            return f"Not ready to arm — {c.detail}"
    return "Arm readiness unknown — waiting for the autopilot"


def _i(raw) -> int | None:
    try:
        return None if raw is None else int(raw)
    except (TypeError, ValueError):
        return None


def _f(raw) -> float | None:
    try:
        return None if raw is None else float(raw)
    except (TypeError, ValueError):
        return None
