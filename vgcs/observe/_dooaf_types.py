"""DOOAF roles, dataclasses, and operator-facing labels."""

from __future__ import annotations

from dataclasses import dataclass


DOOAF_ROLE_SURVEY = "survey"

DOOAF_ROLE_INTENDED = "intended_target"

DOOAF_ROLE_IMPACT = "impact"

DOOAF_ROLE_GUN = "gun_origin"

DOOAF_ROLES = (
    DOOAF_ROLE_SURVEY,
    DOOAF_ROLE_INTENDED,
    DOOAF_ROLE_IMPACT,
    DOOAF_ROLE_GUN,
)

_SETUP_MARK_ROLE_ALIASES: dict[str, str] = {
    "gun": DOOAF_ROLE_GUN,
    "gun_origin": DOOAF_ROLE_GUN,
    "intended": DOOAF_ROLE_INTENDED,
    "target": DOOAF_ROLE_INTENDED,
    "intended_target": DOOAF_ROLE_INTENDED,
    "impact": DOOAF_ROLE_IMPACT,
}

DOOAF_ROLE_DISPLAY: dict[str, str] = {
    DOOAF_ROLE_INTENDED: "Actual target",
    DOOAF_ROLE_IMPACT: "Impact Target",
    DOOAF_ROLE_GUN: "Artillery (gun)",
    DOOAF_ROLE_SURVEY: "Wall measure",
}

DOOAF_ROLE_TOOLTIPS: dict[str, str] = {
    DOOAF_ROLE_INTENDED: (
        "Planned impact point from military staff — where the round should land."
    ),
    DOOAF_ROLE_IMPACT: (
        "Mark Impact Target after firing — click burst or smoke on video. "
        "Set gun and actual target in DOOAF Setup first."
    ),
    DOOAF_ROLE_GUN: (
        "Artillery position — gun origin (use DOOAF Setup or click the map)."
    ),
    DOOAF_ROLE_SURVEY: (
        "Facade width measure with a tape — calibration only, not fire correction."
    ),
}

def dooaf_role_display(role: str) -> str:
    return DOOAF_ROLE_DISPLAY.get(str(role or ""), str(role or ""))

@dataclass(frozen=True)
class GeoPoint:
    lat: float
    lon: float
    alt_m: float | None = None

@dataclass(frozen=True)
class FireCorrection:
    """Correction to apply so the next round lands on the intended target."""

    range_correction_m: float
    deflection_correction_m: float
    miss_along_m: float
    miss_right_m: float
    range_gun_to_intended_m: float
    range_gun_to_impact_m: float
    bearing_gun_to_intended_deg: float
    impact_to_intended_m: float
    miss_east_m: float
    miss_north_m: float
    miss_vertical_m: float | None = None
    elevation_correction_m: float | None = None

@dataclass(frozen=True)
class DooafSession:
    gun: GeoPoint | None
    intended: GeoPoint | None
    impact: GeoPoint | None
    drone: GeoPoint | None
    correction: FireCorrection | None
    building_height_m: float | None = None
    intended_dem_alt_m: float | None = None
    impact_dem_alt_m: float | None = None
    height_correction_m: float | None = None
    dem_available: bool = False
    # False when the DEM terrain-at-footprint elevations are untrustworthy — facade (wall)
    # geometry or a near-horizon look angle, where the ground footprint is ill-defined. The
    # report then hides those DEM rows and keeps only the facade-corrected elevations.
    dem_footprint_reliable: bool = True

@dataclass(frozen=True)
class DooafSettings:
    """Military-supplied fixed coordinates (persisted in QSettings)."""

    gun_lat: float | None = None
    gun_lon: float | None = None
    gun_alt_m: float | None = None
    target_lat: float | None = None
    target_lon: float | None = None
    target_alt_m: float | None = None

@dataclass(frozen=True)
class DooafPreset:
    name: str
    settings: DooafSettings

