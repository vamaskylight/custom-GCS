"""DOOAF result trust / confidence assessment.

A fire-correction number must never be presented as confident when the geometry or sensors
don't support it. This module turns a :class:`DooafSession` into a small, typed set of
findings and an overall confidence tier, so the UI and the HTML report can show a single,
honest "how much to trust this" banner instead of a bare point answer.

Severity:
  block  — the result is not usable as a fire correction (missing data / no position).
  warn   — usable but accuracy is materially degraded; the operator must know.
  info   — an expected caveat for this kind of shot (e.g. facade vertical from FOV).

Confidence tiers: good > caution > low > unusable.
"""

from __future__ import annotations

from dataclasses import dataclass

from vgcs.observe._dooaf_types import DooafSession

SEVERITY_BLOCK = "block"
SEVERITY_WARN = "warn"
SEVERITY_INFO = "info"

CONFIDENCE_GOOD = "good"
CONFIDENCE_CAUTION = "caution"
CONFIDENCE_LOW = "low"
CONFIDENCE_UNUSABLE = "unusable"

_CONFIDENCE_RANK = {
    CONFIDENCE_GOOD: 0,
    CONFIDENCE_CAUTION: 1,
    CONFIDENCE_LOW: 2,
    CONFIDENCE_UNUSABLE: 3,
}

# Below this look-down angle the horizontal footprint is very sensitive to attitude error.
_NEAR_HORIZON_WARN_DEG = 3.0
# GPS quality gates.
_HDOP_WARN = 2.0
# Airborne floor — below this the vehicle pose / geo is unreliable.
_NEAR_GROUND_WARN_M = 2.5


@dataclass(frozen=True)
class TrustFinding:
    severity: str
    code: str
    message: str


@dataclass(frozen=True)
class DooafTrust:
    confidence: str
    findings: tuple[TrustFinding, ...]

    @property
    def is_usable(self) -> bool:
        return self.confidence != CONFIDENCE_UNUSABLE

    @property
    def blocks(self) -> list[TrustFinding]:
        return [f for f in self.findings if f.severity == SEVERITY_BLOCK]

    @property
    def warnings(self) -> list[TrustFinding]:
        return [f for f in self.findings if f.severity == SEVERITY_WARN]

    @property
    def infos(self) -> list[TrustFinding]:
        return [f for f in self.findings if f.severity == SEVERITY_INFO]


_CONFIDENCE_LABEL = {
    CONFIDENCE_GOOD: "Good confidence",
    CONFIDENCE_CAUTION: "Use with caution",
    CONFIDENCE_LOW: "Low confidence",
    CONFIDENCE_UNUSABLE: "Not usable",
}


def confidence_label(confidence: str) -> str:
    return _CONFIDENCE_LABEL.get(str(confidence), str(confidence))


def assess_dooaf_trust(session: DooafSession | None) -> DooafTrust:
    """Assess how much a computed DOOAF result can be trusted."""
    findings: list[TrustFinding] = []
    add = findings.append

    if session is None:
        return DooafTrust(CONFIDENCE_UNUSABLE, ())

    # --- completeness: the three marks + a computed correction --------------------------
    missing = []
    if session.gun is None:
        missing.append("gun")
    if session.intended is None:
        missing.append("target")
    if session.impact is None:
        missing.append("impact")
    if missing:
        add(
            TrustFinding(
                SEVERITY_BLOCK,
                "incomplete",
                "Missing " + ", ".join(missing)
                + " — a full fire correction needs gun, target, and impact.",
            )
        )
    if session.impact is not None and (
        session.impact.lat is None or session.impact.lon is None
    ):
        add(
            TrustFinding(
                SEVERITY_BLOCK,
                "impact_no_position",
                "Impact has no map position (geo failed) — re-mark the fall of shot.",
            )
        )
    if not missing and session.correction is None:
        add(
            TrustFinding(
                SEVERITY_BLOCK,
                "no_correction",
                "Fire correction could not be computed from the marks.",
            )
        )

    # --- geo quality of the impact -----------------------------------------------------
    q = str(session.impact_geo_quality or "").lower()
    if q in ("insufficient", "none", ""):
        if session.impact is not None:
            add(
                TrustFinding(
                    SEVERITY_WARN,
                    "geo_quality_low",
                    "Impact geo quality is insufficient — hover higher / steadier and re-mark.",
                )
            )
    elif q == "fair":
        add(
            TrustFinding(
                SEVERITY_INFO,
                "geo_quality_fair",
                "Impact geo quality is 'fair' — position is a reasonable estimate, not survey grade.",
            )
        )

    # --- geometry: near-horizon look angle ---------------------------------------------
    dep = session.impact_depression_deg
    if dep is not None and abs(float(dep)) < _NEAR_HORIZON_WARN_DEG:
        add(
            TrustFinding(
                SEVERITY_WARN,
                "near_horizon",
                f"Near-horizon shot (look-down {abs(float(dep)):.1f}°) — horizontal position is "
                "very sensitive to attitude error; treat the range as approximate.",
            )
        )

    # --- facade geometry: vertical rests on the FOV assumption --------------------------
    method = str(session.impact_geo_method or "").lower()
    if "facade" in method:
        add(
            TrustFinding(
                SEVERITY_INFO,
                "facade_vertical",
                "Facade (wall) shot — the vertical (height) correction is derived from the "
                "video click and lens FOV, so its exact metres depend on FOV calibration.",
            )
        )
    if not session.dem_footprint_reliable:
        add(
            TrustFinding(
                SEVERITY_INFO,
                "dem_hidden",
                "Terrain (DEM) elevations were unreliable at this angle and are omitted; the "
                "correction uses the facade-based height.",
            )
        )

    # --- vehicle / GPS state -----------------------------------------------------------
    fix = session.gps_fix_type
    if fix is not None and int(fix) < 3:
        add(
            TrustFinding(
                SEVERITY_WARN,
                "gps_fix",
                f"GPS fix type {int(fix)} (no 3D fix) — coordinates may be off by many metres.",
            )
        )
    hdop = session.gps_hdop
    if hdop is not None and float(hdop) > _HDOP_WARN:
        add(
            TrustFinding(
                SEVERITY_WARN,
                "gps_hdop",
                f"GPS HDOP {float(hdop):.1f} (> {_HDOP_WARN}) — degraded horizontal accuracy.",
            )
        )
    ekf = session.impact_ekf_rel_alt_m
    if ekf is not None and float(ekf) < 0.0:
        add(
            TrustFinding(
                SEVERITY_WARN,
                "home_altitude",
                f"EKF altitude is negative ({float(ekf):.1f} m) — home/launch reference is wrong; "
                "video-derived distances will be unreliable.",
            )
        )
    elif ekf is not None and float(ekf) < _NEAR_GROUND_WARN_M:
        add(
            TrustFinding(
                SEVERITY_WARN,
                "near_ground",
                f"Drone was near the ground (EKF {float(ekf):.1f} m) when marking — geo is weak.",
            )
        )

    return DooafTrust(_confidence_from(findings), tuple(findings))


def _confidence_from(findings: list[TrustFinding]) -> str:
    if any(f.severity == SEVERITY_BLOCK for f in findings):
        return CONFIDENCE_UNUSABLE
    warns = sum(1 for f in findings if f.severity == SEVERITY_WARN)
    if warns >= 2:
        return CONFIDENCE_LOW
    if warns == 1:
        return CONFIDENCE_CAUTION
    return CONFIDENCE_GOOD
