"""Live video preview, pipeline, minimap, and recording — extracted from MapWidget."""

from __future__ import annotations

from vgcs.map.video.minimap_mixin import NativeMinimapMixin
from vgcs.map.video.pipeline_mixin import VideoPipelineMixin
from vgcs.map.video.preview_ui_mixin import VideoPreviewUiMixin
from vgcs.map.video.recording_mixin import VideoRecordingMixin


class MapVideoMixins(
    VideoPreviewUiMixin,
    NativeMinimapMixin,
    VideoPipelineMixin,
    VideoRecordingMixin,
):
    """Composable video mixins for :class:`vgcs.map.map_widget.MapWidget`."""


__all__ = [
    "MapVideoMixins",
    "NativeMinimapMixin",
    "VideoPipelineMixin",
    "VideoPreviewUiMixin",
    "VideoRecordingMixin",
]
