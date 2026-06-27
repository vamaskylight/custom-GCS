"""Observation, DOOAF, and C13 LRF logic extracted from MapWidget."""

from __future__ import annotations

from vgcs.map.observation.context_mixin import ObservationContextMixin
from vgcs.map.observation.dooaf_mixin import DooafOperationsMixin
from vgcs.map.observation.lrf_mixin import LrfVideoLockMixin
from vgcs.map.observation.mark_tracking_mixin import VideoMarkTrackingMixin
from vgcs.map.observation.session_mixin import ObservationSessionMixin
from vgcs.map.observation.types import PendingLrfVideoPick

class MapObservationMixins(
    ObservationContextMixin,
    VideoMarkTrackingMixin,
    DooafOperationsMixin,
    ObservationSessionMixin,
    LrfVideoLockMixin,
):
    """Composable mixins mixed into :class:`vgcs.map.map_widget.MapWidget`."""


__all__ = [
    "DooafOperationsMixin",
    "LrfVideoLockMixin",
    "MapObservationMixins",
    "ObservationContextMixin",
    "ObservationSessionMixin",
    "PendingLrfVideoPick",
    "VideoMarkTrackingMixin",
]
