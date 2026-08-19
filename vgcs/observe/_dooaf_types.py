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
    # Trust signals (from the impact mark) so a result's confidence can be assessed anywhere
    # the session is available — see vgcs.observe.dooaf_trust.assess_dooaf_trust.
    impact_geo_quality: str | None = None
    impact_geo_method: str | None = None
    impact_depression_deg: float | None = None
    impact_ekf_rel_alt_m: float | None = None
    gps_fix_type: int | None = None
    gps_hdop: float | None = None
    # True when no gun was surveyed and its position was synthesised purely to
    # fix the firing direction (see assumed_gun_bearing_deg). The gun's lat/lon
    # and every gun→x range are then MEANINGLESS and must not be reported as if
    # measured — only the along/right decomposition is real.
    gun_is_assumed: bool = False
    assumed_gun_bearing_deg: float | None = None

@dataclass(frozen=True)
class DooafSettings:
    """Military-supplied fixed coordinates (persisted in QSettings)."""

    gun_lat: float | None = None
    gun_lon: float | None = None
    gun_alt_m: float | None = None
    target_lat: float | None = None
    target_lon: float | None = None
    target_alt_m: float | None = None
    # Compass bearing FROM GUN TO TARGET, used when the gun is not surveyed at
    # all — the operator marks only target and impact and the artillery is taken
    # as sitting on a known side. Client request 2026-08-19: "they directly mark
    # on Target and impact point, artillery position fixed by South direction …
    # so that we get fire correction data from the north direction" — a gun to
    # the SOUTH firing NORTH, which is 0.0 here.
    #
    # None = normal mode: the gun is picked and the bearing derived from it.
    # Any other value means gun_lat/gun_lon are not required.
    assumed_gun_bearing_deg: float | None = None


# Used when no gun is surveyed and no direction was chosen — artillery to the
# south firing north, which is the deployment convention this was built for.
DEFAULT_ASSUMED_GUN_BEARING_DEG = 0.0

# Where the artillery is taken to sit relative to the target, and the resulting
# gun→target firing bearing. South-of-target fires north (0°), and so on.
ASSUMED_GUN_DIRECTIONS: tuple[tuple[str, float], ...] = (
    ("South of target (fires north)", 0.0),
    ("West of target (fires east)", 90.0),
    ("North of target (fires south)", 180.0),
    ("East of target (fires west)", 270.0),
)


@dataclass(frozen=True)
class DooafPreset:
    name: str
    settings: DooafSettings

