"""Map HUD, tiles, plan flight, vehicle telemetry, 3D, JS bridge, camera rail."""

from __future__ import annotations

from vgcs.map.surface.camera_rail_mixin import CameraRailMixin
from vgcs.map.surface.hud_layout_mixin import NativeHudLayoutMixin
from vgcs.map.surface.map_3d_mixin import Map3dMixin
from vgcs.map.surface.map_tiles_mixin import MapTilesMixin
from vgcs.map.surface.plan_mission_mixin import PlanMissionMixin
from vgcs.map.surface.vehicle_telemetry_mixin import VehicleTelemetryMixin
from vgcs.map.surface.web_bridge_mixin import WebBridgeMixin


class MapSurfaceMixins(
    NativeHudLayoutMixin,
    MapTilesMixin,
    PlanMissionMixin,
    VehicleTelemetryMixin,
    Map3dMixin,
    WebBridgeMixin,
    CameraRailMixin,
):
    """Composable surface mixins for :class:`vgcs.map.map_widget.MapWidget`."""


__all__ = [
    "CameraRailMixin",
    "Map3dMixin",
    "MapSurfaceMixins",
    "MapTilesMixin",
    "NativeHudLayoutMixin",
    "PlanMissionMixin",
    "VehicleTelemetryMixin",
    "WebBridgeMixin",
]
