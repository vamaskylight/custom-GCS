"""Field of view at the camera's current zoom, for DOOAF pixel-to-angle maths.

Geo-referencing a video pick turns an offset from the middle of the picture
into an angle off boresight, and that conversion needs the field of view the
picture was actually taken at. Zoom in and the field of view narrows, so a
static wide-angle number turns a small offset into a large angle and puts the
mark hundreds of metres from where the operator clicked.

Reported 2026-09-09 while planning a long-range DOOAF test: "We can zoom on the
actual target or impact Target". The laser flow was already safe, because it
slews the target to the middle of the frame and converts at boresight where the
field of view cancels out. Every other path converts an off-centre click, and
none of them knew the zoom.

The C13 is a 30x optical zoom (confirmed by the crew, against a stale note in
camera_control.py claiming a fixed lens). It sends zoom commands but reports
nothing back, so the best we have is the level we asked for. Viewpro reports its
true field of view continuously, which is better and is preferred when present.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# Where the numbers came from, so a report can say so rather than implying the
# camera was measured when it was only asked.
FOV_SOURCE_CAMERA = "camera"        # the camera told us its current FOV
FOV_SOURCE_COMMANDED = "commanded"  # narrowed by the zoom level we commanded
FOV_SOURCE_WIDE = "wide"            # no zoom information; the lens wide-open

# A zoom below 1 would widen the lens past its own optics, and anything above
# this is not a real camera. Both mean bad input, and neither should be trusted
# to scale an angle.
MIN_ZOOM_X = 1.0
MAX_ZOOM_X = 100.0


@dataclass(frozen=True)
class CameraFov:
    """Horizontal and vertical field of view in degrees, and where they came from."""

    hfov_deg: float
    vfov_deg: float
    zoom_x: float
    source: str

    @property
    def is_measured(self) -> bool:
        """True only when the camera itself reported this, not when we assumed it."""
        return self.source == FOV_SOURCE_CAMERA


def clamp_zoom_x(zoom_x: float | None) -> float:
    """A usable zoom multiplier; 1.0 for anything missing or nonsensical."""
    try:
        z = float(zoom_x)
    except (TypeError, ValueError):
        return 1.0
    if not math.isfinite(z):
        return 1.0
    return max(MIN_ZOOM_X, min(MAX_ZOOM_X, z))


def fov_at_zoom(fov_deg: float, zoom_x: float | None) -> float:
    """Narrow a field of view by an optical zoom factor.

    Zoom multiplies the focal length, and the field of view follows the tangent
    of the half angle, not the angle itself::

        tan(fov_z / 2) = tan(fov_wide / 2) / zoom

    Dividing the angle directly is the tempting shortcut and it is wrong by
    enough to matter: 83.4 degrees at 10x is 10.2 degrees, where dividing gives
    8.3, a 22 percent error in every angle derived from it. The gap grows with
    the wide-angle value, which is exactly the C13's situation.
    """
    try:
        wide = float(fov_deg)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(wide) or wide <= 0.0:
        return 0.0
    wide = min(wide, 179.0)
    z = clamp_zoom_x(zoom_x)
    if z <= 1.0:
        return wide
    half = math.atan(math.tan(math.radians(wide) / 2.0) / z)
    return math.degrees(half) * 2.0


def resolve_camera_fov(
    *,
    wide_hfov_deg: float,
    wide_vfov_deg: float | None = None,
    reported: tuple[float, float] | None = None,
    zoom_x: float | None = None,
) -> CameraFov:
    """The field of view to geo-reference a pick with, and its provenance.

    A camera that reports its own field of view is believed over any arithmetic,
    because it accounts for the real lens rather than a nominal zoom step.
    """
    if reported is not None:
        try:
            rh, rv = float(reported[0]), float(reported[1])
        except (TypeError, ValueError, IndexError):
            rh = rv = 0.0
        if _is_sane_fov(rh) and _is_sane_fov(rv):
            return CameraFov(rh, rv, clamp_zoom_x(zoom_x), FOV_SOURCE_CAMERA)

    wide_h = float(wide_hfov_deg)
    wide_v = float(wide_vfov_deg) if wide_vfov_deg else wide_h * 0.5625
    z = clamp_zoom_x(zoom_x)
    if z <= 1.0:
        return CameraFov(wide_h, wide_v, 1.0, FOV_SOURCE_WIDE)
    return CameraFov(
        fov_at_zoom(wide_h, z),
        fov_at_zoom(wide_v, z),
        z,
        FOV_SOURCE_COMMANDED,
    )


def _is_sane_fov(deg: float) -> bool:
    return math.isfinite(deg) and 0.0 < deg < 180.0
