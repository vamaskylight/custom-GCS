"""Extract HUD / tiles / plan / vehicle / 3D / bridge / camera-rail from map_widget.py."""

from __future__ import annotations

import ast
import textwrap
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAP_WIDGET = ROOT / "vgcs" / "map" / "map_widget.py"
OUT_PKG = ROOT / "vgcs" / "map" / "surface"

HUD_LAYOUT = {
    "set_dashboard_mode",
    "resizeEvent",
    "showEvent",
    "_refresh_native_overlay_insets",
    "_map_canvas_rect_on_panel",
    "_raise_panel_flight_overlays",
    "_stack_native_overlays_above_tile_map",
    "_sync_native_map_vehicle_arrow_scale",
    "_on_main_map_zoom_step",
    "_sync_native_map_zoom_label",
    "_camera_rail_visible_pref",
    "_camera_rail_may_appear",
    "_set_camera_rail_panel_visible",
    "_sync_camera_rail_panel_visibility",
    "_position_camera_rail_show_tab",
    "suppress_floating_overlays",
    "restore_floating_overlays",
    "_layout_native_hud",
    "_sync_map_action_rail_enabled",
    "set_link_connected",
    "set_flight_status",
    "set_header_mode",
    "set_header_vehicle_msg",
    "set_header_gps",
    "set_header_battery",
    "set_header_remote_id",
    "_layout_plan_flight_panel",
    "_plan_flight_layer_obscures_native_camera_ui",
    "_set_map_footer_hud_visible",
}

MAP_TILES = {
    "_native_map_tile_count",
    "_promote_native_map_if_ready",
    "_show_map_main_surface",
    "_activate_startup_tile_source",
    "_native_tile_startup_check",
    "_native_tile_startup_check_final",
    "_sync_web_map_center_from_native",
    "_relayout_web_map_view",
    "_ensure_native_map_visible",
    "_ensure_map_tiles_visible",
    "_activate_web_2d_fallback",
    "_probe_current_tiles",
    "_probe_current_tiles_from_payload",
    "_on_tile_probe_result",
    "_set_webcam_enabled",
    "_on_perf_mode_changed",
    "_apply_low_spec_mode",
    "_maybe_autodetect_low_spec",
    "_init_map_backend",
    "_on_native_user_waypoints_changed",
    "_on_map_loaded",
    "center_on_vehicle",
    "_enable_fence_polygon_mode",
    "_set_esri_street_tiles",
    "_set_osm_tiles",
    "_set_satellite_tiles",
    "_pick_offline_tiles",
    "activate_esri_street_tiles",
    "activate_osm_tiles",
    "activate_satellite_tiles",
    "activate_offline_tiles",
    "_apply_geofence",
    "_clear_geofence",
}

PLAN_MISSION = {
    "_on_plan_panel_exit",
    "_sync_native_plan_edit_mode_for_rail_tool",
    "_on_plan_panel_tool",
    "_on_plan_panel_mission_changed",
    "_on_plan_panel_waypoints_changed",
    "_on_plan_panel_set_launch_to_map_center",
    "set_plan_flight_visible",
    "set_plan_flight_metrics",
    "refresh_plan_flight_chrome",
    "set_plan_rail_tool",
    "apply_plan_mission_panel_state",
    "set_plan_sequence_template",
    "set_plan_mission_start_stack",
    "set_plan_vehicle_info",
    "get_default_waypoint_alt_m",
    "set_default_waypoint_alt_m",
    "request_mission_upload_from_map",
    "request_mission_download_from_map",
    "clear_map_waypoints",
    "clear_plan_current_mission_path",
    "start_waypoint_planning",
    "start_roi_planning",
    "open_mission_file",
    "get_vehicle_position",
    "set_mission_nav_seq",
    "set_mission_waypoint_count",
    "_enable_add_waypoint_mode",
    "_clear_waypoints",
    "_after_clear_waypoints",
    "_sync_waypoint_count_from_map",
    "_on_waypoint_count",
    "_on_waypoints_json",
    "_after_waypoints_mutated",
    "_plan_waypoints_snapshot",
    "_request_upload",
    "_waypoints_from_map_json",
    "_request_download",
    "_plan_current_mission_path",
    "save_plan_mission_json",
    "save_plan_mission_kml",
    "_export_mission",
    "_import_mission",
    "set_waypoints",
    "get_waypoint_meta",
    "apply_waypoint_meta",
    "_rebuild_wp_selector",
    "_on_wp_selected",
    "_apply_altitude_to_selected",
    "_apply_altitude_to_all",
    "_apply_speed_to_selected",
    "_apply_speed_to_all",
}

VEHICLE_TELEMETRY = {
    "_haversine_m",
    "_update_map_motion_state",
    "_apply_map_vehicle_heading",
    "set_vehicle_position",
    "set_vehicle_attitude",
    "set_vehicle_alt_msl",
    "set_gps_hdop",
    "set_vehicle_heading",
    "clear_flight_track",
    "set_obstacle_distance",
    "set_distance_sensor",
    "get_obstacle_sensor_summary",
    "is_map_motion_armed",
    "get_vehicle_display_position",
    "set_flight_telemetry",
    "_gps_available_for_geo_pick",
}

MAP_3D = {
    "_set_3d_marker_overlay_active",
    "_layout_map_3d_marker_overlay",
    "_refresh_3d_marker_overlay",
    "_on_3d_marker_overlay_json",
    "_build_3d_overlay_payload",
    "_sync_3d_map_overlays",
    "_emit_map_3d_mode_changed",
    "_ensure_web_3d_view",
    "_inject_legacy_html_hud_hide",
    "_on_web_3d_load_finished",
    "set_3d_enabled",
    "_on_3d_toggle_result",
    "_prime_3d_vehicle_coords_js",
    "_recenter_3d_after_enable",
    "_toggle_3d_mode",
}

WEB_BRIDGE = {
    "_on_web_title_changed",
    "_map_uses_legacy_web_bridge",
    "_run_js",
    "_set_status",
    "_schedule_vehicle_pose_js",
    "_flush_vehicle_pose_js",
    "on_application_background",
    "on_application_foreground",
}

CAMERA_RAIL = {
    "set_camera_control",
    "_apply_digital_zoom",
    "_ensure_obs_clip_banner",
    "_position_obs_clip_banner",
    "_obs_clip_ui_preparing",
    "_obs_clip_update_countdown_labels",
    "_show_obs_clip_banner",
    "_hide_obs_clip_banner",
    "_obs_clip_countdown_tick",
    "_obs_clip_ui_finished",
    "_obs_clip_ui_failed",
    "_obs_cell",
    "_on_native_split_rail_toggled",
    "_commit_native_split_rail_toggle",
    "_on_camera_rail_mode_id_clicked",
    "_sync_native_cam_timer_visibility",
    "_on_native_follow_rail_toggled",
    "_commit_native_follow_rail_toggle",
    "_sync_native_camera_rail_toggles",
    "_native_gimbal_uses_ptz_hold",
    "_native_gimbal_ptz_action",
    "_gimbal_hold_speeds",
    "_wire_native_gimbal_hold_button",
    "_on_gimbal_hold_tick",
    "_native_gimbal_speed_start",
    "_native_gimbal_speed_stop",
    "_native_gimbal_center",
    "_native_gimbal_point_down",
    "_siyi_autofocus_adapter",
    "_trigger_gimbal_stop_autofocus",
}

MIXIN_RULES: list[tuple[str, set[str]]] = [
    ("hud_layout_mixin", HUD_LAYOUT),
    ("map_tiles_mixin", MAP_TILES),
    ("plan_mission_mixin", PLAN_MISSION),
    ("vehicle_telemetry_mixin", VEHICLE_TELEMETRY),
    ("map_3d_mixin", MAP_3D),
    ("web_bridge_mixin", WEB_BRIDGE),
    ("camera_rail_mixin", CAMERA_RAIL),
]

NAME_TO_MIXIN: dict[str, str] = {}
for mixin, names in MIXIN_RULES:
    for n in names:
        NAME_TO_MIXIN[n] = mixin

TILE_PROBE_CLASSES = {"_TileProbeBridge", "_TileProbeTask"}

MODULE_HELPERS = {
    "_web_2d_fallback_allowed": "helpers.py",
}

CONSTANT_NAMES = {
    "_MAP_MOVE_ARM_SPEED_MPS",
    "_MAP_MOVE_DISARM_SPEED_MPS",
    "_MAP_MOVE_ARM_SAMPLES",
    "_MAP_MOVE_DISARM_SAMPLES",
    "_MAP_POSITION_MIN_MOVE_M",
    "_MAP_HUD_TOP_PX",
    "_MAP_ACTION_RAIL_LEFT_PX",
    "_MAP_ACTION_RAIL_TOP_PX",
    "_NATIVE_CAM_RAIL_TOP_PX",
    "_MAP_HUD_MARGIN_PX",
    "_MAP_ACTION_RAIL_HEIGHT_PX",
    "_OBSTACLE_PANEL_TOP_PX",
    "_OBSTACLE_PANEL_MAX_H_PX",
    "_MAP_HUD_GLASS_BG",
    "_MAP_HUD_GLASS_BORDER",
    "_WEB_MAP_RELAYOUT_JS",
}

SETTINGS_KEY_NAMES = {
    "_KEY_MAP_OFFLINE_TILE_ROOT",
    "_KEY_MAP_WEBCAM_ENABLED",
    "_KEY_MAP_LOW_SPEC_MODE",
    "_KEY_MAP_TILE_MODE",
    "_KEY_PLAN_CURRENT_MISSION_JSON",
    "_KEY_TOOLBAR_EXPORT_MISSION_JSON",
    "_KEY_PLAN_LAST_MISSION_JSON_LEGACY",
    "_KEY_CAMERA_RAIL_VISIBLE",
}

HIDE_LEGACY_HTML_HUD_JS = textwrap.dedent(
    """\
        _HIDE_LEGACY_HTML_HUD_JS = (
            "(function(){"
            "var s=document.getElementById('vgcs_3d_hide_overlays_style');"
            "if(!s){s=document.createElement('style');s.id='vgcs_3d_hide_overlays_style';"
            "document.head.appendChild(s);}"
            "s.textContent='#linkBanner,#actionRail,#planFlightLayer,#cameraRail,"
            "#mapFooterHud,#telemetryStrip,#compass,#hdrMapModeBtn,#videoPreview"
            "{display:none !important;}"
            "#mapWrap>.overlay,#map3dMarkerCanvas{pointer-events:none !important;}';"
            "})();"
        )
    """
)


def _lines(path: Path) -> list[str]:
    return path.read_text(encoding="utf-8").splitlines(keepends=True)


def _class_body_nodes(tree: ast.Module, class_name: str) -> list[ast.stmt]:
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == class_name:
            return list(node.body)
    raise RuntimeError(f"class {class_name} not found")


def _stmt_source(lines: list[str], node: ast.stmt) -> str:
    start = node.lineno - 1
    end = getattr(node, "end_lineno", node.lineno)
    return "".join(lines[start:end])


def _module_level_assignments(tree: ast.Module, names: set[str]) -> dict[str, ast.stmt]:
    out: dict[str, ast.stmt] = {}
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for t in node.targets:
                if isinstance(t, ast.Name) and t.id in names:
                    out[t.id] = node
        elif isinstance(node, ast.FunctionDef) and node.name in names:
            out[node.name] = node
    return out


def _strip_orphan_staticmethods(text: str) -> str:
    lines = text.splitlines(keepends=True)
    out: list[str] = []
    i = 0
    while i < len(lines):
        line = lines[i]
        if line.strip() == "@staticmethod":
            j = i + 1
            while j < len(lines) and not lines[j].strip():
                j += 1
            if j < len(lines) and lines[j].lstrip().startswith("def "):
                i += 1
                continue
        out.append(line)
        i += 1
    return "".join(out)


def main() -> None:
    lines = _lines(MAP_WIDGET)
    tree = ast.parse("".join(lines))
    map_nodes = _class_body_nodes(tree, "MapWidget")

    buckets: dict[str, list[str]] = {m: [] for m, _ in MIXIN_RULES}
    remove_ranges: list[tuple[int, int]] = []

    for node in map_nodes:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            mixin = NAME_TO_MIXIN.get(node.name)
            if mixin is None:
                continue
            buckets[mixin].append(_stmt_source(lines, node))
            remove_ranges.append((node.lineno, node.end_lineno or node.lineno))
        elif isinstance(node, ast.Assign):
            for t in node.targets:
                if isinstance(t, ast.Name) and t.id == "_HIDE_LEGACY_HTML_HUD_JS":
                    remove_ranges.append((node.lineno, node.end_lineno or node.lineno))

    # Module-level tile probe classes
    tile_probe_parts: list[str] = [
        '"""HTTP tile probe used by map startup health checks."""\n\nfrom __future__ import annotations\n\n'
        "from PySide6.QtCore import QObject, QRunnable, Signal\n"
        "from PySide6.QtGui import QImage\n\n"
        "from vgcs.map.native_tile_map import fetch_tile_http_bytes\n\n"
    ]
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name in TILE_PROBE_CLASSES:
            tile_probe_parts.append(_stmt_source(lines, node))
            remove_ranges.append((node.lineno, node.end_lineno or node.lineno))

    OUT_PKG.mkdir(parents=True, exist_ok=True)
    (OUT_PKG / "tile_probe.py").write_text("\n\n".join(tile_probe_parts), encoding="utf-8")

    # constants.py + settings_keys.py + helpers.py
    mod_assigns = _module_level_assignments(tree, CONSTANT_NAMES | SETTINGS_KEY_NAMES)
    mod_fns = _module_level_assignments(tree, set(MODULE_HELPERS))

    const_parts = ['"""Map surface layout / motion constants."""\n\nfrom __future__ import annotations\n\nimport os\n\n']
    const_order = [
        "_MAP_HUD_TOP_PX",
        "_MAP_ACTION_RAIL_LEFT_PX",
        "_MAP_ACTION_RAIL_TOP_PX",
        "_NATIVE_CAM_RAIL_TOP_PX",
        "_MAP_HUD_MARGIN_PX",
        "_MAP_ACTION_RAIL_HEIGHT_PX",
        "_OBSTACLE_PANEL_TOP_PX",
        "_OBSTACLE_PANEL_MAX_H_PX",
        "_MAP_HUD_GLASS_BG",
        "_MAP_HUD_GLASS_BORDER",
        "_MAP_MOVE_ARM_SPEED_MPS",
        "_MAP_MOVE_DISARM_SPEED_MPS",
        "_MAP_MOVE_ARM_SAMPLES",
        "_MAP_MOVE_DISARM_SAMPLES",
        "_MAP_POSITION_MIN_MOVE_M",
        "_WEB_MAP_RELAYOUT_JS",
    ]
    for name in const_order:
        if name in mod_assigns:
            const_parts.append(_stmt_source(lines, mod_assigns[name]))
            remove_ranges.append(
                (mod_assigns[name].lineno, mod_assigns[name].end_lineno or mod_assigns[name].lineno)
            )
    (OUT_PKG / "constants.py").write_text("\n".join(const_parts), encoding="utf-8")

    settings_parts = ['"""QSettings keys for map tiles, plan flight, and camera rail."""\n\nfrom __future__ import annotations\n\n']
    for name in sorted(SETTINGS_KEY_NAMES):
        if name in mod_assigns:
            settings_parts.append(_stmt_source(lines, mod_assigns[name]))
            remove_ranges.append(
                (mod_assigns[name].lineno, mod_assigns[name].end_lineno or mod_assigns[name].lineno)
            )
    (OUT_PKG / "settings_keys.py").write_text("\n".join(settings_parts), encoding="utf-8")

    helper_parts = ['"""Small helpers for map tile / web fallback paths."""\n\nfrom __future__ import annotations\n\nimport os\n\n']
    for name, node in mod_fns.items():
        if name in MODULE_HELPERS:
            helper_parts.append(_stmt_source(lines, node))
            remove_ranges.append((node.lineno, node.end_lineno or node.lineno))
    (OUT_PKG / "helpers.py").write_text("\n".join(helper_parts), encoding="utf-8")

    mixin_header = '"""MapWidget surface mixin — see vgcs.map.surface package."""\n\nfrom __future__ import annotations\n\n'

    mixin_imports = {
        "hud_layout_mixin": textwrap.dedent(
            """\
            import json

            from PySide6.QtCore import QPoint, QSettings, QTimer, Qt
            from PySide6.QtWidgets import QApplication

            from vgcs.map.app_settings import QS_APP, QS_ORG
            from vgcs.map.cam_rail_widgets import CamRailShowHandle
            from vgcs.map.surface.constants import (
                _CAM_RAIL_GIMBAL_GRID_GAP,
                _CAM_RAIL_LAYER_INSET,
                _CAM_RAIL_PAD_BTN_W,
                _MAP_ACTION_RAIL_HEIGHT_PX,
                _MAP_ACTION_RAIL_LEFT_PX,
                _MAP_ACTION_RAIL_TOP_PX,
                _MAP_HUD_MARGIN_PX,
                _MAP_HUD_TOP_PX,
                _NATIVE_CAM_RAIL_CONTENT_MIN_WIDTH_PX,
                _NATIVE_CAM_RAIL_TOP_PX,
                _OBSTACLE_PANEL_MAX_H_PX,
                _OBSTACLE_PANEL_TOP_PX,
            )
            from vgcs.map.surface.settings_keys import _KEY_CAMERA_RAIL_VISIBLE
            """
        ),
        "map_tiles_mixin": textwrap.dedent(
            """\
            import json
            import math
            import os
            from pathlib import Path

            from PySide6.QtCore import QSettings, QThreadPool, QTimer, Qt, QUrl
            from PySide6.QtWidgets import QFileDialog

            from vgcs.map.app_settings import QS_APP, QS_ORG
            from vgcs.map.legacy_leaflet_build import build_leaflet_html
            from vgcs.map.map_web_3d import HAS_WEBENGINE as HAS_WEBENGINE_3D
            from vgcs.map.native_tile_map import NativeTileMapView
            from vgcs.map.surface.constants import _WEB_MAP_RELAYOUT_JS
            from vgcs.map.surface.helpers import _web_2d_fallback_allowed
            from vgcs.map.surface.settings_keys import (
                _KEY_MAP_LOW_SPEC_MODE,
                _KEY_MAP_OFFLINE_TILE_ROOT,
                _KEY_MAP_TILE_MODE,
                _KEY_MAP_WEBCAM_ENABLED,
            )
            from vgcs.map.surface.tile_probe import _TileProbeBridge, _TileProbeTask
            """
        ),
        "plan_mission_mixin": textwrap.dedent(
            """\
            import json
            from pathlib import Path

            from PySide6.QtCore import QSettings, QTimer
            from PySide6.QtWidgets import QFileDialog, QMessageBox

            from vgcs.map.app_settings import QS_APP, QS_ORG
            from vgcs.map.surface.settings_keys import (
                _KEY_PLAN_CURRENT_MISSION_JSON,
                _KEY_PLAN_LAST_MISSION_JSON_LEGACY,
                _KEY_TOOLBAR_EXPORT_MISSION_JSON,
            )
            from vgcs.mission import Waypoint, load_waypoints_json, save_waypoints_json, save_waypoints_kml
            """
        ),
        "vehicle_telemetry_mixin": textwrap.dedent(
            """\
            import json
            import math

            from PySide6.QtCore import QTimer

            from vgcs.map.surface.constants import (
                _MAP_MOVE_ARM_SAMPLES,
                _MAP_MOVE_ARM_SPEED_MPS,
                _MAP_MOVE_DISARM_SAMPLES,
                _MAP_MOVE_DISARM_SPEED_MPS,
                _MAP_POSITION_MIN_MOVE_M,
            )
            """
        ),
        "map_3d_mixin": textwrap.dedent(
            """\
            import json

            from PySide6.QtCore import QTimer

            from vgcs.map.legacy_leaflet_build import build_leaflet_html
            from vgcs.map.map_3d_marker_overlay import Map3dLayer, Map3dMarkerOverlay
            from vgcs.map.map_web_3d import HAS_WEBENGINE as HAS_WEBENGINE_3D, assets_base_url, create_map_3d_web_view
            from vgcs.map.surface.helpers import _web_2d_fallback_allowed
            """
        ),
        "web_bridge_mixin": textwrap.dedent(
            """\
            import base64
            import json
            import time

            from PySide6.QtCore import QPoint, QTimer

            from vgcs.video.pipeline import notify_companion_app_background, notify_companion_app_foreground
            """
        ),
        "camera_rail_mixin": textwrap.dedent(
            """\
            import math
            import time

            from PySide6.QtCore import QTimer, Qt
            from PySide6.QtGui import QImage
            from PySide6.QtWidgets import QLabel

            from vgcs.video.camera_control import camera_zoom_limits
            """
        ),
    }

    class_names = {
        "hud_layout_mixin": "NativeHudLayoutMixin",
        "map_tiles_mixin": "MapTilesMixin",
        "plan_mission_mixin": "PlanMissionMixin",
        "vehicle_telemetry_mixin": "VehicleTelemetryMixin",
        "map_3d_mixin": "Map3dMixin",
        "web_bridge_mixin": "WebBridgeMixin",
        "camera_rail_mixin": "CameraRailMixin",
    }

    for mixin_file, class_name in class_names.items():
        body = buckets[mixin_file]
        extra = ""
        if mixin_file == "map_3d_mixin":
            extra = "\n" + HIDE_LEGACY_HTML_HUD_JS + "\n"
        print(f"Wrote {mixin_file}.py ({len(body)} methods)")
        content = (
            mixin_header
            + mixin_imports.get(mixin_file, "")
            + f"\n\nclass {class_name}:\n"
            + '    """Extracted from MapWidget — uses host widget state via self."""\n'
            + (extra if mixin_file == "map_3d_mixin" else "")
            + "\n"
            + "\n".join(body)
        )
        (OUT_PKG / f"{mixin_file}.py").write_text(content, encoding="utf-8")

    init_py = textwrap.dedent(
        '''\
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
        '''
    )
    (OUT_PKG / "__init__.py").write_text(init_py, encoding="utf-8")

    remove_ranges.sort(key=lambda r: r[0], reverse=True)
    drop: set[int] = set()
    for start, end in remove_ranges:
        drop.update(range(start, end + 1))
    new_lines = [line for i, line in enumerate(lines, start=1) if i not in drop]

    src = "".join(new_lines)
    src = _strip_orphan_staticmethods(src)

    if "MapSurfaceMixins" not in src:
        src = src.replace(
            "from vgcs.map.video import MapVideoMixins",
            "from vgcs.map.video import MapVideoMixins\nfrom vgcs.map.surface import MapSurfaceMixins",
            1,
        )
        src = src.replace(
            "class MapWidget(MapObservationMixins, MapVideoMixins, QWidget):",
            "class MapWidget(MapObservationMixins, MapVideoMixins, MapSurfaceMixins, QWidget):",
            1,
        )

    # Wire tile probe import
    if "from vgcs.map.surface.tile_probe import" not in src:
        src = src.replace(
            "from vgcs.map.observation.types import",
            "from vgcs.map.surface.tile_probe import _TileProbeBridge, _TileProbeTask\n"
            "from vgcs.map.observation.types import",
            1,
        )

    # Constants / settings now live under surface/
    surface_import = textwrap.dedent(
        """\
        from vgcs.map.surface.constants import (
            _MAP_HUD_GLASS_BG,
            _MAP_HUD_GLASS_BORDER,
            _MAP_HUD_MARGIN_PX,
            _MAP_HUD_TOP_PX,
            _NATIVE_CAM_RAIL_TOP_PX,
            _MAP_MOVE_ARM_SAMPLES,
            _MAP_MOVE_ARM_SPEED_MPS,
            _MAP_MOVE_DISARM_SAMPLES,
            _MAP_MOVE_DISARM_SPEED_MPS,
            _MAP_POSITION_MIN_MOVE_M,
        )
        from vgcs.map.surface.settings_keys import (
            _KEY_CAMERA_RAIL_VISIBLE,
            _KEY_MAP_LOW_SPEC_MODE,
            _KEY_MAP_OFFLINE_TILE_ROOT,
            _KEY_MAP_TILE_MODE,
            _KEY_MAP_WEBCAM_ENABLED,
            _KEY_PLAN_CURRENT_MISSION_JSON,
            _KEY_PLAN_LAST_MISSION_JSON_LEGACY,
            _KEY_TOOLBAR_EXPORT_MISSION_JSON,
        )
        """
    )
    if "from vgcs.map.surface.constants import" not in src:
        src = src.replace(
            "from vgcs.map.app_settings import QS_ORG, QS_APP",
            "from vgcs.map.app_settings import QS_ORG, QS_APP\n" + surface_import,
            1,
        )

    # Remove duplicate module-level definitions if still present
    for block_start in (
        "_QS_NS = ",
        "_KEY_MAP_OFFLINE_TILE_ROOT = ",
        "_MAP_MOVE_ARM_SPEED_MPS = ",
        "def _web_2d_fallback_allowed",
        "_WEB_MAP_RELAYOUT_JS = ",
    ):
        if block_start in src:
            pass  # cleaned by remove_ranges

    out_path = MAP_WIDGET.with_suffix(".py.new")
    out_path.write_text(src, encoding="utf-8")
    out_path.replace(MAP_WIDGET)
    print(f"Updated map_widget.py — removed {len(remove_ranges)} blocks")


if __name__ == "__main__":
    main()
