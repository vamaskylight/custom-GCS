"""VGCS's own tracker on the video as Follow Me's lock (vgcs/app/object_follow.py, FollowLock).

The track (vgcs/map/observation/track_mixin.py) tells this object when it starts and ends, whether the last picture
showed the object, and where the object is on the ground (vgcs/observe/follow_place.py). Only VGCS's own software
tracker can be followed: a camera's own tracker does not say where the object is in the picture.
"""

from __future__ import annotations

import time

from vgcs.app.object_follow import FollowLock
from vgcs.observe.follow_place import PICTURE_AGE_S, Place

SEEN_FRESH_S = 1.0   # the last tracked picture older than this: the lock does not see the object now


class TrackFollowLock(FollowLock):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._active = False
        self._ours = False
        self._seen = False
        self._seen_mono = -1e9

    def lock_active(self) -> bool:
        return self._active

    def lock_by_us(self) -> bool:
        return self._ours

    def lock_seen(self) -> bool:
        return (self._active and self._ours and self._seen
                and time.monotonic() - self._seen_mono <= SEEN_FRESH_S)

    def not_ours_text(self) -> str:
        return ("Follow needs VGCS's own tracker on the object. This camera follows the object with its own tracker, "
                "which does not say where the object is in the picture.")

    # --- what the track tells it ---

    def set_lock(self, active: bool, ours: bool) -> bool:
        """The track runs (active), with VGCS's own tracker (ours). True when that changed."""
        active = bool(active)
        ours = active and bool(ours)
        if (active, ours) == (self._active, self._ours):
            return False
        self._active, self._ours = active, ours
        self._seen = False
        self.lock_changed.emit()
        return True

    def saw(self, seen: bool) -> None:
        """The last tracked picture showed the object (seen) or not."""
        self._seen = bool(seen)
        if self._seen:
            self._seen_mono = time.monotonic()

    def place(self, place: Place) -> None:
        self.object_placed.emit(place.lat_deg, place.lon_deg, place.sigma_m, place.below_m,
                                round(PICTURE_AGE_S * 1000.0))
