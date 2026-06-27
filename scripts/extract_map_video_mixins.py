"""Extract video preview / pipeline / minimap methods from map_widget.py into mixins."""

from __future__ import annotations

import ast
import textwrap
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAP_WIDGET = ROOT / "vgcs" / "map" / "map_widget.py"
OUT_PKG = ROOT / "vgcs" / "map" / "video"

PREVIEW_UI = {
    "_on_native_video_click",
    "_apply_native_video_click_layout",
    "_split_hit_slot_in_composite",
    "_split_slot_from_video_click",
    "_pick_split_fullscreen_source_from_click",
    "_pick_primary_split_source_id",
    "_ensure_split_fullscreen_focus",
    "_prepare_video_swap_layout",
    "_mini_video_pip_rect",
    "_video_stream_configured",
    "_show_mini_video_pip_shell",
    "_set_native_video_pip_placeholder",
    "_layout_native_video_preview",
    "_ensure_video_pro_hud_visible",
    "_raise_flight_hud_above_video",
    "_video_zoom_limits",
    "_effective_preview_digital_zoom",
    "_apply_video_recording_preview_transform",
    "_sync_video_recording_preview_transform",
    "_sync_native_video_zoom_label",
    "_retry_native_video_pixmap",
    "_native_video_click_mirror_x",
    "_native_video_click_norm",
    "_native_video_content_rect",
    "_sync_native_video_overlay",
    "_render_native_video_preview",
    "_schedule_split_preview_render",
    "_flush_split_preview_render",
    "_render_split_fullscreen_waiting",
    "_render_native_split_preview",
    "_split_cell_label",
    "_render_native_split_grid_4",
    "_draw_split_cell_label",
    "_best_native_preview_frame",
    "_apply_native_split_mode_changed",
    "_seed_split_cache_from_last_frame",
    "_on_mavlink_link_show_mini_video",
    "_companion_has_dual_feed",
    "_companion_show_ir_button",
    "_companion_switch_active_feed",
    "_sync_native_thermal_feed_button",
    "_on_native_thermal_feed_toggled",
    "_companion_video_decode_gate",
    "_sync_video_split_from_settings",
    "_uses_companion_rtsp",
    "_should_defer_companion_rtsp_decode",
    "_companion_decode_running",
    "_companion_wire_preview_ui",
    "_companion_start_decode_if_needed",
    "_request_companion_video_restart",
    "_reapply_preview_zoom_now",
}

MINIMAP = {
    "_schedule_minimap_grab_refresh",
    "_on_native_minimap_image_press",
    "_on_native_minimap_image_wheel",
    "_on_native_minimap_image_move",
    "_on_native_minimap_image_release",
    "_on_native_minimap_plus_clicked",
    "_on_native_minimap_minus_clicked",
    "_native_minimap_set_zoom",
    "_raise_native_minimap_zoom_buttons",
    "_update_native_minimap_from_web_grab",
    "_render_native_minimap_fallback",
    "_native_minimap_tile_bad",
    "_fetch_tile_image",
    "_schedule_native_minimap_refresh",
    "_update_native_minimap",
    "_minimap_click_to_lat_lon",
}

PIPELINE = {
    "_hook_video_pipeline_sources_changed",
    "_detach_video_pipeline_frame_slots",
    "_connect_video_pipeline_frame_slots",
    "_on_video_pipeline_sources_changed",
    "_ensure_video_preview_backend",
    "_configure_video_pipeline",
    "_read_video_settings",
    "_video_record_suffix",
    "_video_preview_should_run",
    "_mini_video_pip_allowed",
    "_auto_start_mini_video_pip",
    "_apply_video_settings_read_toolbar",
    "apply_video_settings_for_settings_dialog",
    "apply_video_settings",
    "_schedule_video_preview_after_settings",
    "_restart_video_preview_after_settings",
    "_operator_preview_source_id",
    "_video_source_by_id",
    "_operator_preview_video_source",
    "_video_preview_source_ids_to_run",
    "_refresh_companion_video_after_foreground",
    "_start_video_decode_sources",
    "_stop_idle_video_decode_sources",
    "_video_gui_stall_recovery_enabled",
    "_on_video_preview_stall_check",
    "_start_video_preview",
    "_stop_video_preview_begin",
    "_silence_pipeline_video_sources",
    "_stop_video_preview_stop_sources",
    "_stop_video_preview_end",
    "_stop_video_preview",
    "_on_pipeline_frame",
    "_on_pipeline_frame_for",
    "_on_video_frame_encoded_for",
    "_on_video_frame_encoded",
    "_push_video_preview_any_to_overlay",
    "_cache_preview_raw_frame",
    "_preview_image_copy_for_snapshot",
    "_notify_companion_gimbal_motion",
    "set_video_follow_enabled",
}

RECORDING = {
    "_trigger_hardware_photo",
    "_sync_payload_hardware_recording",
    "_capture_photo_quick",
    "_flash_photo_feedback",
    "_clear_photo_flash",
    "_obs_clip_ui_recording_started",
    "_sync_native_record_button_for_rail_mode",
    "_format_native_cam_recording_duration",
    "_ensure_native_cam_recording_tick_timer",
    "_on_native_cam_recording_tick",
    "_start_native_cam_recording_tick_timer",
    "_stop_native_cam_recording_tick_timer",
    "_on_native_record_center_clicked",
    "_on_native_record_toggled",
}

MIXIN_RULES: list[tuple[str, set[str]]] = [
    ("preview_ui_mixin", PREVIEW_UI),
    ("minimap_mixin", MINIMAP),
    ("pipeline_mixin", PIPELINE),
    ("recording_mixin", RECORDING),
]

NAME_TO_MIXIN: dict[str, str] = {}
for mixin, names in MIXIN_RULES:
    for n in names:
        NAME_TO_MIXIN[n] = mixin

MODULE_HELPERS = {
    "_format_video_zoom_label": "helpers.py",
}


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


def _module_level_functions(tree: ast.Module) -> dict[str, ast.FunctionDef]:
    out: dict[str, ast.FunctionDef] = {}
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name in MODULE_HELPERS:
            out[node.name] = node
    return out


def main() -> None:
    lines = _lines(MAP_WIDGET)
    tree = ast.parse("".join(lines))
    map_nodes = _class_body_nodes(tree, "MapWidget")

    buckets: dict[str, list[str]] = {m: [] for m, _ in MIXIN_RULES}
    remove_ranges: list[tuple[int, int]] = []

    for node in map_nodes:
        if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        mixin = NAME_TO_MIXIN.get(node.name)
        if mixin is None:
            continue
        buckets[mixin].append(_stmt_source(lines, node))
        remove_ranges.append((node.lineno, node.end_lineno or node.lineno))

    OUT_PKG.mkdir(parents=True, exist_ok=True)

    # helpers.py
    mod_fns = _module_level_functions(tree)
    helper_parts = [
        '"""Video UI helpers shared by map video mixins."""\n\nfrom __future__ import annotations\n\n'
    ]
    for name, node in mod_fns.items():
        helper_parts.append(_stmt_source(lines, node))
        remove_ranges.append((node.lineno, node.end_lineno or node.lineno))
    (OUT_PKG / "helpers.py").write_text("\n\n".join(helper_parts), encoding="utf-8")

    mixin_header = '"""MapWidget video mixin — see vgcs.map.video package."""\n\nfrom __future__ import annotations\n\n'

    mixin_imports = {
        "preview_ui_mixin": textwrap.dedent(
            """\
            from PySide6.QtCore import QPointF, Qt, QTimer
            from PySide6.QtGui import QImage, QPainter, QPixmap
            from PySide6.QtWidgets import QLabel

            from vgcs.map.native_video_overlay import NativeVideoOverlayLayer
            from vgcs.map.video.helpers import _format_video_zoom_label
            from vgcs.video.camera_control import (
                camera_preview_applies_digital_zoom,
                camera_recording_applies_digital_zoom,
                camera_zoom_limits,
            )
            from vgcs.video.pipeline import (
                VideoFrame,
                notify_companion_feed_switch,
                notify_companion_preview_motion,
                release_companion_rtsp_host,
                set_companion_decode_gate,
            )
            """
        ),
        "minimap_mixin": textwrap.dedent(
            """\
            import time
            from urllib.request import Request

            from PySide6.QtCore import QPoint, QPointF, Qt, QTimer
            from PySide6.QtGui import QColor, QImage, QPainter, QPen, QPixmap
            from PySide6.QtWidgets import QApplication

            from vgcs.map.native_tile_map import fetch_tile_http_bytes
            """
        ),
        "pipeline_mixin": textwrap.dedent(
            """\
            import os
            import time

            from PySide6.QtCore import QSettings, QThreadPool, QTimer
            from PySide6.QtGui import QImage, QPixmap
            from PySide6.QtWidgets import QMessageBox

            from vgcs.map.app_settings import QS_APP, QS_ORG
            from vgcs.map.video.helpers import _format_video_zoom_label
            from vgcs.video.pipeline import (
                HAS_MULTIMEDIA,
                VideoFrame,
                VideoPipeline,
                QS_KEY_LAST_PHOTO_SAVE_DIR,
                notify_companion_app_background,
                notify_companion_app_foreground,
                notify_companion_preview_motion,
                release_all_companion_rtsp_hosts,
                release_companion_rtsp_host,
                set_companion_decode_gate,
                suggested_photo_save_path,
                suggested_recording_save_path,
                wait_qmedia_recorder_stopped,
            )
            """
        ),
        "recording_mixin": textwrap.dedent(
            """\
            from pathlib import Path

            from PySide6.QtCore import QTimer
            from PySide6.QtGui import QImage
            from PySide6.QtWidgets import QMessageBox

            from vgcs.map.image_io import save_qimage_to_path
            from vgcs.video.pipeline import (
                suggested_photo_save_path,
                suggested_recording_save_path,
                wait_qmedia_recorder_stopped,
            )
            """
        ),
    }

    class_names = {
        "preview_ui_mixin": "VideoPreviewUiMixin",
        "minimap_mixin": "NativeMinimapMixin",
        "pipeline_mixin": "VideoPipelineMixin",
        "recording_mixin": "VideoRecordingMixin",
    }

    for mixin_file, class_name in class_names.items():
        body = buckets[mixin_file]
        print(f"Wrote {mixin_file}.py ({len(body)} methods)")
        content = (
            mixin_header
            + mixin_imports.get(mixin_file, "")
            + f"\n\nclass {class_name}:\n"
            + '    """Extracted from MapWidget — uses host widget state via self."""\n\n'
            + "\n".join(body)
        )
        (OUT_PKG / f"{mixin_file}.py").write_text(content, encoding="utf-8")

    # __init__.py
    init_py = textwrap.dedent(
        '''\
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
        '''
    )
    (OUT_PKG / "__init__.py").write_text(init_py, encoding="utf-8")

    remove_ranges.sort(key=lambda r: r[0], reverse=True)
    new_lines = list(lines)
    for start, end in remove_ranges:
        del new_lines[start - 1 : end]

    src = "".join(new_lines)
    if "MapVideoMixins" not in src:
        src = src.replace(
            "from vgcs.map.observation import MapObservationMixins",
            "from vgcs.map.observation import MapObservationMixins\n"
            "from vgcs.map.video import MapVideoMixins",
            1,
        )
        src = src.replace(
            "class MapWidget(MapObservationMixins, QWidget):",
            "class MapWidget(MapObservationMixins, MapVideoMixins, QWidget):",
            1,
        )
        # Preview UI mixin should import helper without underscore conflict
        pass

    MAP_WIDGET.write_text(src, encoding="utf-8")
    print(f"Updated map_widget.py — removed {len(remove_ranges)} blocks")


if __name__ == "__main__":
    main()
