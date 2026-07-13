"""DOOAF overlay-tracking tuning constants — one documented source of truth.

These thresholds were tuned against real C13 field behaviour. They were previously scattered
across the tracking code as bare literals, which made them hard to find, reason about, or
adjust as a set. Each is documented here with WHY it has the value it does, so a future
change is a deliberate, reviewable decision rather than a guess.

Every value is degrees unless the name ends in ``_M`` (metres). The tracking mixin imports
these under its historic private names, so behaviour is unchanged — this only centralises
and documents them.
"""

from __future__ import annotations

# --- Video-overlay mark tracking ------------------------------------------------------

# LRF-boresight gun / facade marks: show the dot steady on its pick point while the gimbal
# PITCH is within this of the lock pose, then hide it (edge arrow). Small so the dot leaves
# cleanly the moment the operator tilts toward the next point, but above the ~1.3° gimbal
# drift seen during a hold-lock so it does not flicker. Camera-FOLLOW moves YAW, not pitch,
# so the hide is keyed to pitch only — see mark_tracking_mixin._setup_mark_panned_off.
BORESIGHT_HIDE_DEG = 3.0

# Ground / facade-UV picks world-anchor only after a deliberate pan past this. In LOITER with
# camera-FOLLOW the C13 GAC yaw settles 4–8° after lock with no operator input; this deadband
# keeps the mark frozen through that involuntary settle so it doesn't chase the camera.
PAN_TRACK_GIMBAL_DEADBAND_DEG = 8.0

# Near-field LRF picks pin at the click until the gimbal pans past this. Below the near-field
# slant threshold, GPS geo projection jitters, so a tighter attitude deadband holds the dot.
LRF_MARK_PIN_ATT_DEADBAND_DEG = 1.25

# Attitude-track (GAC) engages once the gimbal moves past this — same math as the LRF reticle.
ATTITUDE_TRACK_GIMBAL_DEADBAND_DEG = 0.5

# Below this LRF slant, GPS geo projection jitters; pin the overlay until the gimbal pans.
NEAR_FIELD_LRF_PIN_SLANT_M = 25.0

# Loiter/wind drift: anchor ground picks to their saved geo once the drone shifts this far.
GEO_TRACK_VEHICLE_SHIFT_M = 2.0

# Freeze-while-holding: once world-anchored, only recompute the on-screen point when the
# gimbal slews past this angle / the drone drifts past this distance — kills sensor-noise
# tremble while the operator is aiming.
WA_HOLD_STILL_DEG = 0.6
WA_HOLD_STILL_M = 0.75

# NOTE: the mark-track vehicle-pose EMA factor (0.28) lives in geo_reference.py
# (_MARK_TRACK_POSE_SMOOTH_ALPHA) next to the projection that uses it; documented here for
# discoverability but intentionally not duplicated as a value.
