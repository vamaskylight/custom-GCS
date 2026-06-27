"""QSettings keys for map tiles, plan flight, and camera rail."""

from __future__ import annotations


_KEY_CAMERA_RAIL_VISIBLE = "map/camera_rail_visible"

_KEY_MAP_LOW_SPEC_MODE = "map_low_spec_mode"  # 'auto' | 'on' | 'off'

_KEY_MAP_OFFLINE_TILE_ROOT = "map_offline_tile_root"

_KEY_MAP_TILE_MODE = "map_tile_mode"  # 'esri_streets' | 'osm' | 'sat' | 'offline'

_KEY_MAP_WEBCAM_ENABLED = "map_webcam_enabled"

_KEY_PLAN_CURRENT_MISSION_JSON = "plan_current_mission_json"

_KEY_PLAN_LAST_MISSION_JSON_LEGACY = "plan_last_mission_json"  # legacy; read fallback only

_KEY_TOOLBAR_EXPORT_MISSION_JSON = "toolbar_export_mission_json"
