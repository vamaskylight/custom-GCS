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

from vgcs.observe.geo_reference import (
    GeoReferenceResult,
    _gimbal_yaw_right_deg,
    _lrf_camera_dir_ned_unit,
    compute_lrf_facade_plane_geo,
)

# A laser point is taken for a point in the air when it is this much higher
# above the ground under the drone than it is far away from the drone. Ground
# on a slope of 45 degrees is as high as it is far away, and the margin is for
# the height itself, which is only known to some metres.
_IN_THE_AIR_MARGIN_M = 15.0

# How far from the cross a click may be for the laser to stand for it. The
# laser measures at the cross and nowhere else. This is the C13's own limit
# for a lock with the camera held still (adapter._LRF_HOLD_MAX_CLICK_OFFSET_DEG).
# The Viewpro's is wider.
LASER_MAX_OFF_CROSS_DEG = 4.0


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
    # The height above the take-off point, for the check that the drone is
    # still where the lock was made (uv_pick_valid).
    vehicle_rel_alt_m: float | None = None
    gps_fix_type: int = 0
    gps_hdop: float | None = None
    # Which way gimbal_yaw_deg counts (GimbalStatus.yaw_left_positive).
    gimbal_yaw_left_positive: bool = False
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

    @property
    def ground_range_m(self) -> float | None:
        """Horizontal (map) distance to the locked point — slant corrected for tilt."""
        if self._lock is None:
            return None
        return slant_to_ground_range_m(
            self._lock.slant_range_m, self._lock.gimbal_pitch_deg
        )

    def clear(self) -> None:
        self._lock = None

    def record_from_context(
        self,
        slant_range_m: float,
        ctx: dict[str, Any],
    ) -> None:
        """Store facade lock after a successful LRF lock.

        The new lock takes the place of the one before. When it cannot be
        stored (no position, no camera angles), none is held: the one before
        was kept, and a click made for the new lock was then placed from the
        old one.
        """
        self._lock = None
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
            gimbal_yaw_left_positive=bool(ctx.get("gimbal_yaw_left_positive") or False),
            gimbal_pitch_deg=float(gp),
            vehicle_roll_deg=_float_or_none(ctx.get("vehicle_roll_deg")),
            vehicle_pitch_deg=_float_or_none(ctx.get("vehicle_pitch_deg")),
            vehicle_alt_msl_m=_float_or_none(ctx.get("vehicle_alt_msl_m")),
            vehicle_rel_alt_m=_float_or_none(ctx.get("ekf_rel_alt_m")),
            gps_fix_type=int(ctx.get("gps_fix_type") or 0),
            gps_hdop=_float_or_none(ctx.get("gps_hdop")),
        )

    def camera_turn_since_lock_deg(self, ctx: dict[str, Any]) -> float | None:
        """How far the camera's look has turned since the lock, in degrees.

        The larger of the turn to a side and the turn up or down. None when
        there is no lock, or the camera does not say its angles now.

        To a side, the look is the drone's heading and the camera's yaw
        together. A drone that turns with a camera that follows it has turned
        the look. A camera that holds its look while the drone turns under it
        has not. Until 2026-10-09 the camera's own yaw was compared alone: with
        a camera that follows the drone, the drone could turn right round and
        the lock was still held. Without a heading now it is compared alone,
        as before.
        """
        lock = self._lock
        if lock is None:
            return None
        yaw = _float_or_none(ctx.get("gimbal_yaw_deg"))
        pitch = _float_or_none(ctx.get("gimbal_pitch_deg"))
        if yaw is None or pitch is None:
            return None
        look_now = _gimbal_yaw_right_deg(yaw, bool(ctx.get("gimbal_yaw_left_positive") or False))
        look_then = _gimbal_yaw_right_deg(
            float(lock.gimbal_yaw_deg), bool(lock.gimbal_yaw_left_positive)
        )
        heading = _float_or_none(ctx.get("vehicle_heading_deg"))
        if heading is not None:
            look_now += heading
            look_then += float(lock.vehicle_heading_deg)
        to_a_side = abs((look_now - look_then + 180.0) % 360.0 - 180.0)
        return max(to_a_side, abs(pitch - float(lock.gimbal_pitch_deg)))

    def why_not_held(
        self,
        ctx: dict[str, Any],
        *,
        max_gimbal_delta_deg: float = 10.0,
        max_vehicle_shift_m: float = 8.0,
        max_age_s: float = 600.0,
    ) -> str:
        """Why the lock does not stand for another pick any more, or "" while it does.

        In a few words, for the banner on the video.

        ``max_vehicle_shift_m`` counts up and down as well as sideways. It
        counted sideways only, so a lock made on the ground stayed "locked"
        through a take-off straight up, for ten minutes.
        """
        lock = self._lock
        if lock is None:
            return "no lock was made"
        if time.monotonic() - float(lock.lock_mono) > float(max_age_s):
            return f"it is more than {float(max_age_s) / 60.0:.0f} minutes old"
        turn = self.camera_turn_since_lock_deg(ctx)
        if turn is None:
            return "the camera does not say its angles"
        if turn > float(max_gimbal_delta_deg):
            return f"the camera was turned {turn:.0f} degrees"
        clat = ctx.get("vehicle_lat")
        clon = ctx.get("vehicle_lon")
        if clat is None or clon is None:
            return "the drone's position is not known"
        shift = _haversine_m(
            float(lock.vehicle_lat),
            float(lock.vehicle_lon),
            float(clat),
            float(clon),
        )
        if shift > float(max_vehicle_shift_m):
            return f"the drone has moved {shift:.0f} m"
        # The same height reference on both sides, the smoother one first.
        for now_key, then in (
            ("ekf_rel_alt_m", lock.vehicle_rel_alt_m),
            ("vehicle_alt_msl_m", lock.vehicle_alt_msl_m),
        ):
            now = _float_or_none(ctx.get(now_key))
            if now is not None and then is not None:
                moved = abs(now - float(then))
                if moved > float(max_vehicle_shift_m):
                    return f"the drone has moved {moved:.0f} m up or down"
                return ""
        return ""

    def uv_pick_valid(
        self,
        ctx: dict[str, Any],
        *,
        max_gimbal_delta_deg: float = 10.0,
        max_vehicle_shift_m: float = 8.0,
        max_age_s: float = 600.0,
    ) -> bool:
        """True when a UV-only pick can reuse the facade lock (see why_not_held)."""
        return not self.why_not_held(
            ctx,
            max_gimbal_delta_deg=max_gimbal_delta_deg,
            max_vehicle_shift_m=max_vehicle_shift_m,
            max_age_s=max_age_s,
        )

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
        return compute_lrf_facade_plane_geo(
            vehicle_lat=lock.vehicle_lat,
            vehicle_lon=lock.vehicle_lon,
            vehicle_heading_deg=lock.vehicle_heading_deg,
            vehicle_roll_deg=lock.vehicle_roll_deg,
            vehicle_pitch_deg=lock.vehicle_pitch_deg,
            vehicle_alt_msl_m=lock.vehicle_alt_msl_m,
            gimbal_yaw_deg=g_yaw,
            gimbal_yaw_left_positive=bool(lock.gimbal_yaw_left_positive),
            gimbal_pitch_deg=g_pitch,
            slant_range_m=float(lock.slant_range_m),
            video_x_norm=float(u),
            video_y_norm=float(v),
            gps_fix_type=int(lock.gps_fix_type),
            gps_hdop=lock.gps_hdop,
            camera_hfov_deg=float(hfov_deg),
            camera_vfov_deg=vfov_deg,
        )


def slant_to_ground_range_m(
    slant_range_m: float | None, pitch_deg: float | None
) -> float | None:
    """Horizontal (map) distance from a laser slant range and gimbal pitch.

    ``ground = slant × cos(pitch)``. The C13 laser returns the SLANT (line-of-sight)
    range; at a steep down-look that is much longer than the distance along the ground.
    The ground distance is what matches the map and the operator's eye. Returns the raw
    slant when the pitch is unknown.
    """
    try:
        s = float(slant_range_m)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None
    if pitch_deg is None:
        return s
    try:
        p = math.radians(float(pitch_deg))
    except (TypeError, ValueError):
        return s
    return abs(s * math.cos(p))


@dataclass(frozen=True)
class LaserPointInTheAir:
    """A laser range that cannot be a point on the ground."""

    slant_range_m: float
    drone_height_m: float  # above the ground under the drone, as far as VGCS knows it
    point_height_m: float  # of the laser point above that same ground

    def short_text(self) -> str:
        """One line, for the caption on the video."""
        return (
            f"Laser {self.slant_range_m:.1f} m is in the air "
            f"(drone {self.drone_height_m:.0f} m up)"
        )

    def _what_was_seen(self) -> str:
        return (
            f"The laser gave {self.slant_range_m:.1f} m, but the drone is "
            f"{self.drone_height_m:.0f} m above the ground.\n"
            f"A point that close is about {self.point_height_m:.0f} m up in the air, "
            "not on the ground."
        )

    def question(self) -> str:
        """Asked of the operator, who alone can see which of the two it is."""
        return (
            f"{self._what_was_seen()}\n\n"
            "Is the target on a building or a tower right beside the drone?"
        )

    def not_set_text(self) -> str:
        """After the operator said that it is not."""
        return (
            "The target is not set: the laser did not measure it.\n"
            "Put the cross on the target and lock again, or fly closer."
        )

    def text(self) -> str:
        """The whole of it, where nobody was asked."""
        return (
            f"{self._what_was_seen()}\n"
            "So the laser did not measure the target.\n\n"
            "Put the cross on the target and lock again, or fly closer."
        )


def laser_point_in_the_air(
    slant_range_m: float | None, ctx: dict[str, Any]
) -> LaserPointInTheAir | None:
    """Say when a laser range cannot be a point on the ground, else None.

    Seen on 2026-10-08: the client flew 83 m up over open ground, the camera
    5 degrees under the horizon, and the laser lock gave 18.5 m. VGCS put the
    target 18 m in front of the drone, 81 m up in the air, and worked out a
    fire correction from there.

    The point is where the look direction and the range put it. It is in the
    air when it is higher above the ground under the drone than it is far away
    from the drone, by more than a margin: no slope up to 45 degrees reaches
    it. The drone's height is the lower of what the autopilot and the terrain
    file say, so that a drone standing on a roof or a hill is not taken to be
    high up. None also when the height is not known.

    One thing looks the same and is right: a tower or a tall building right
    beside the drone. Geometry cannot tell them apart, so the caller asks the
    operator (DooafOperationsMixin._ask_whether_a_point_in_the_air_is_a_building).
    """
    try:
        slant = float(slant_range_m)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None
    heights = [
        h
        for h in (_float_or_none(ctx.get(key)) for key in ("ekf_rel_alt_m", "measure_agl_m"))
        if h is not None and math.isfinite(h)
    ]
    if not heights or not math.isfinite(slant) or slant < 0.5:
        return None
    drone_height = min(heights)
    down: float | None = None
    if ctx.get("gimbal_yaw_deg") is not None and ctx.get("gimbal_pitch_deg") is not None:
        try:
            ray = _lrf_camera_dir_ned_unit(
                vehicle_heading_deg=ctx.get("vehicle_heading_deg"),  # type: ignore[arg-type]
                vehicle_roll_deg=ctx.get("vehicle_roll_deg"),  # type: ignore[arg-type]
                vehicle_pitch_deg=ctx.get("vehicle_pitch_deg"),  # type: ignore[arg-type]
                gimbal_yaw_deg=ctx.get("gimbal_yaw_deg"),  # type: ignore[arg-type]
                gimbal_yaw_left_positive=ctx.get("gimbal_yaw_left_positive"),  # type: ignore[arg-type]
                gimbal_pitch_deg=ctx.get("gimbal_pitch_deg"),  # type: ignore[arg-type]
            )
        except (TypeError, ValueError):
            ray = None
        if ray is not None:
            down = float(ray[0][2])
    if down is None:
        # The look direction is not known. Whatever it is, the point is no
        # lower than straight down, and height minus distance is no less than this.
        point_height = drone_height - slant
        spare = drone_height - slant * math.sqrt(2.0)
    else:
        point_height = drone_height - slant * down
        spare = point_height - slant * math.sqrt(max(0.0, 1.0 - down * down))
    if spare <= _IN_THE_AIR_MARGIN_M:
        return None
    return LaserPointInTheAir(
        slant_range_m=slant, drone_height_m=drone_height, point_height_m=point_height
    )


def format_lrf_range_label(
    slant_range_m: float | None, ground_range_m: float | None
) -> str:
    """'35 m ground · LRF 44.8 m' — lead with the map distance, keep slant for reference."""
    if slant_range_m is None:
        return ""
    slant_txt = f"{float(slant_range_m):.1f} m"
    if ground_range_m is not None:
        return f"{float(ground_range_m):.0f} m ground · LRF {slant_txt}"
    return f"LRF {slant_txt}"


def build_facade_overlay_hint(
    *,
    slant_range_m: float | None,
    uv_pick_ready: bool,
    pending_roles: list[str],
    ground_range_m: float | None = None,
    ended_why: str = "",
) -> tuple[str, str] | None:
    """Title + subtitle for the banner of a laser lock on the video.

    The words are for a lock on anything. Until 2026-10-09 they were "Facade
    locked" and "Facade stale, re-lock LRF on building", from the days of the
    tests on a building, also over open ground. ``ended_why``: why the lock
    does not stand for another pick any more (DooafFacadeSession.why_not_held).
    """
    if slant_range_m is None:
        return None
    rng_txt = format_lrf_range_label(slant_range_m, ground_range_m)
    if uv_pick_ready:
        title = f"Laser lock: {rng_txt}"
        if pending_roles:
            # What is still missing, and no word on how to set it. This line
            # said "Click on video (fast pick)". A gun was never placed from
            # this lock, a fall of shot is not since 2026-10-08, and a target
            # is placed from it only on the wall of the lock, after the
            # operator is asked.
            labels = " · ".join(str(r) for r in pending_roles)
            subtitle = f"Still to set: {labels}"
        else:
            subtitle = "All marks set: confirm DOOAF Setup or export REPORT"
        return title, subtitle
    title = "Laser lock ended"
    why = str(ended_why or "").strip()
    subtitle = f"Last {rng_txt}: {why}" if why else f"Last {rng_txt}"
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
