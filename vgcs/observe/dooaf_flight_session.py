"""
DOOAF in-flight session: one LRF facade lock, then fast UV picks for nearby points.

Gun / target / impact on the same building face share slant range and lock pose so
relative geometry stays coherent in LOITER (no three 60 s slews).
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any

from vgcs.observe.geo_reference import GeoReferenceResult, compute_lrf_slant_geo


@dataclass
class FacadeLockSnapshot:
    """Vehicle + gimbal pose and LRF slant at facade lock."""

    slant_range_m: float
    vehicle_lat: float
    vehicle_lon: float
    vehicle_heading_deg: float
    gimbal_yaw_deg: float
    gimbal_pitch_deg: float
    vehicle_roll_deg: float | None = None
    vehicle_pitch_deg: float | None = None
    vehicle_alt_msl_m: float | None = None
    gps_fix_type: int = 0
    gps_hdop: float | None = None
    lock_mono: float = field(default_factory=time.monotonic)


class DooafFacadeSession:
    """Shared facade lock for rapid UV picks while airborne."""

    def __init__(self) -> None:
        self._lock: FacadeLockSnapshot | None = None

    @property
    def has_lock(self) -> bool:
        return self._lock is not None

    @property
    def slant_range_m(self) -> float | None:
        if self._lock is None:
            return None
        return float(self._lock.slant_range_m)

    def clear(self) -> None:
        self._lock = None

    def record_from_context(
        self,
        slant_range_m: float,
        ctx: dict[str, Any],
    ) -> None:
        """Store facade lock after a successful LRF lock."""
        try:
            slant = float(slant_range_m)
        except (TypeError, ValueError):
            return
        if slant < 0.5:
            return
        vlat = ctx.get("vehicle_lat")
        vlon = ctx.get("vehicle_lon")
        gy = ctx.get("gimbal_yaw_deg")
        gp = ctx.get("gimbal_pitch_deg")
        hdg = ctx.get("vehicle_heading_deg")
        if vlat is None or vlon is None or gy is None or gp is None or hdg is None:
            return
        self._lock = FacadeLockSnapshot(
            slant_range_m=slant,
            vehicle_lat=float(vlat),
            vehicle_lon=float(vlon),
            vehicle_heading_deg=float(hdg),
            gimbal_yaw_deg=float(gy),
            gimbal_pitch_deg=float(gp),
            vehicle_roll_deg=_float_or_none(ctx.get("vehicle_roll_deg")),
            vehicle_pitch_deg=_float_or_none(ctx.get("vehicle_pitch_deg")),
            vehicle_alt_msl_m=_float_or_none(ctx.get("vehicle_alt_msl_m")),
            gps_fix_type=int(ctx.get("gps_fix_type") or 0),
            gps_hdop=_float_or_none(ctx.get("gps_hdop")),
        )

    def uv_pick_valid(
        self,
        ctx: dict[str, Any],
        *,
        max_gimbal_delta_deg: float = 10.0,
        max_vehicle_shift_m: float = 4.0,
        max_age_s: float = 600.0,
    ) -> bool:
        """True when a UV-only pick can reuse the facade lock."""
        lock = self._lock
        if lock is None:
            return False
        if time.monotonic() - float(lock.lock_mono) > float(max_age_s):
            return False
        gy = ctx.get("gimbal_yaw_deg")
        gp = ctx.get("gimbal_pitch_deg")
        if gy is None or gp is None:
            return False
        dy = abs(float(gy) - float(lock.gimbal_yaw_deg))
        dp = abs(float(gp) - float(lock.gimbal_pitch_deg))
        if dy > float(max_gimbal_delta_deg) or dp > float(max_gimbal_delta_deg):
            return False
        clat = ctx.get("vehicle_lat")
        clon = ctx.get("vehicle_lon")
        if clat is None or clon is None:
            return False
        shift = _haversine_m(
            float(lock.vehicle_lat),
            float(lock.vehicle_lon),
            float(clat),
            float(clon),
        )
        return shift <= float(max_vehicle_shift_m)

    def geo_from_uv(
        self,
        u: float,
        v: float,
        *,
        hfov_deg: float,
        vfov_deg: float | None = None,
        ctx: dict[str, Any] | None = None,
    ) -> GeoReferenceResult:
        """World geo for a video UV using the facade lock slant and lock pose."""
        lock = self._lock
        if lock is None:
            return GeoReferenceResult(ok=False, warning="no facade lock", method="none")
        # Facade UV geo always uses the LRF-lock gimbal pose — not live GAC (yaw lags in LOITER).
        g_yaw = float(lock.gimbal_yaw_deg)
        g_pitch = float(lock.gimbal_pitch_deg)
        return compute_lrf_slant_geo(
            vehicle_lat=lock.vehicle_lat,
            vehicle_lon=lock.vehicle_lon,
            vehicle_heading_deg=lock.vehicle_heading_deg,
            vehicle_roll_deg=lock.vehicle_roll_deg,
            vehicle_pitch_deg=lock.vehicle_pitch_deg,
            vehicle_alt_msl_m=lock.vehicle_alt_msl_m,
            gimbal_yaw_deg=g_yaw,
            gimbal_pitch_deg=g_pitch,
            slant_range_m=float(lock.slant_range_m),
            video_x_norm=float(u),
            video_y_norm=float(v),
            gps_fix_type=int(lock.gps_fix_type),
            gps_hdop=lock.gps_hdop,
            camera_hfov_deg=float(hfov_deg),
            camera_vfov_deg=vfov_deg,
        )


def build_facade_overlay_hint(
    *,
    slant_range_m: float | None,
    uv_pick_ready: bool,
    pending_roles: list[str],
) -> tuple[str, str] | None:
    """Title + subtitle for the in-video facade lock banner."""
    if slant_range_m is None:
        return None
    slant_txt = f"{float(slant_range_m):.1f} m"
    if uv_pick_ready:
        title = f"Facade locked — LRF {slant_txt}"
        if pending_roles:
            labels = " · ".join(str(r) for r in pending_roles)
            subtitle = f"Click on video (fast pick): {labels}"
        else:
            subtitle = "All marks set — confirm DOOAF Setup or export REPORT"
        return title, subtitle
    title = "Facade stale — re-lock LRF on building"
    subtitle = f"Last LRF {slant_txt}; gimbal or drone moved too far"
    return title, subtitle


def mark_track_use_geo_in_flight(
    *,
    has_geo: bool,
    rel_alt_m: float | None,
    min_airborne_alt_m: float = 3.0,
) -> bool:
    """In flight, always project stored world points through current vehicle pose."""
    if not has_geo:
        return False
    try:
        alt = float(rel_alt_m) if rel_alt_m is not None else -1.0
    except (TypeError, ValueError):
        alt = -1.0
    return alt >= float(min_airborne_alt_m)


def _float_or_none(raw: object) -> float | None:
    if raw is None:
        return None
    try:
        return float(raw)
    except (TypeError, ValueError):
        return None


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r = 6_371_000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = (
        math.sin(dp / 2) ** 2
        + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    )
    return 2.0 * r * math.asin(min(1.0, math.sqrt(a)))
