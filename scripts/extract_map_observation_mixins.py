"""One-off extractor: observation / DOOAF / LRF methods from map_widget.py into mixins."""

from __future__ import annotations

import ast
import textwrap
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAP_WIDGET = ROOT / "vgcs" / "map" / "map_widget.py"
OUT_PKG = ROOT / "vgcs" / "map" / "observation"

# Classes to move out of map_widget (before MapWidget).
TASK_CLASSES = {
    "_ObservationSnapshotBridge",
    "_PendingLrfVideoPick",
    "_LrfLockBridge",
    "_LrfLockTask",
    "_ObservationSnapshotTask",
    "_ObservationExportBridge",
    "_ObservationExportTask",
}

MIXIN_RULES: list[tuple[str, set[str]]] = [
    (
        "context_mixin",
        {
            "_observation_context",
            "_set_observation_mark_mode",
            "_current_observe_dooaf_role",
            "_observation_mark_active",
        },
    ),
    (
        "mark_tracking_mixin",
        {
            "_build_video_mark_track",
            "_pending_lrf_dooaf_pick_role",
            "_sync_dooaf_setup_mark_from_lrf_slew_progress",
            "_sync_dooaf_setup_track_from_lrf_lock",
            "_hide_lrf_video_reticle_keep_range",
            "_register_dooaf_setup_mark_track",
            "_attach_lock_vehicle_pose_to_track",
            "_vehicle_airborne_for_mark_track",
            "_mark_track_use_geo_projection",
            "_video_mark_tracking_active",
            "_sync_video_mark_track_timer",
            "_refresh_tracked_video_marks_light",
            "_apply_video_mark_gimbal_track_to_row",
            "_project_geo_to_video_norm",
            "_attitude_mark_uv_from_track",
            "_project_mark_uv_unclamped",
            "_mark_uv_on_screen",
            "_make_offscreen_hint",
            "_tracked_uv_from_store",
            "_dooaf_mark_display_uv",
            "_persist_mark_track_smooth_keys",
            "_observation_mark_display_uv",
            "_video_overlay_offscreen_hints",
            "_video_overlay_marks",
        },
    ),
    (
        "dooaf_mixin",
        {
            "_dooaf_settings_store",
            "_dooaf_session_kwargs",
            "_resolved_dooaf_settings",
            "_ensure_dooaf_impact_visible_on_map",
            "_refresh_dooaf_map_overlay",
            "_on_native_observation_map_click",
            "_end_dooaf_map_pick",
            "_preview_dooaf_overlay",
            "_dooaf_lrf_geo_enabled",
            "_apply_lrf_slant_geo_to_row",
            "_append_lrf_fallback_warning",
            "_begin_c13_lrf_video_lock_for_pick",
            "_complete_pending_dooaf_setup_lrf_pick",
            "_dooaf_video_pick_failed",
            "_dooaf_facade_pending_pick_labels",
            "_refresh_dooaf_facade_overlay_hint",
            "_dooaf_facade_uv_pick_ready",
            "_geo_from_facade_uv_pick",
            "_facade_lock_gimbal_att",
            "_complete_dooaf_setup_facade_uv_pick",
            "_capture_lrf_lock_start_vehicle_pose",
            "_vehicle_shift_during_lrf_lock_m",
            "_try_record_dooaf_facade_session",
            "_refresh_dooaf_facade_overlay_after_change",
            "_handle_dooaf_video_pick",
            "_prepare_dooaf_video_pick",
            "_begin_dooaf_pick",
            "_begin_dooaf_map_pick",
            "_begin_dooaf_video_pick",
            "_sync_dooaf_settings_from_dialog",
            "_on_dooaf_setup_coordinates_changed",
            "_show_dooaf_setup_dialog",
            "_commit_dooaf_setup_dialog",
            "_enrich_observation_geo_reference",
            "_compute_video_pick_geo",
            "_observe_dem_path",
            "_dem_elevation_at",
            "_m8_geo_settings",
        },
    ),
    (
        "session_mixin",
        {
            "_log_observation",
            "_log_observation_impl",
            "_complete_pending_observation_lrf_pick",
            "_log_observation_after_geo",
            "_schedule_video_marks_overlay_refresh",
            "_flush_video_marks_overlay",
            "_observation_measure_labels_and_segments",
            "_observation_video_measure_segments",
            "_refresh_observation_measure_overlays",
            "_schedule_observation_snapshot",
            "_on_observation_snapshot_saved",
            "_fill_observation_snapshot",
            "_observation_export_dir",
            "_capture_observation_clip",
            "_stop_observation_clip_rtsp",
            "_stop_observation_clip_rec",
            "_finish_observation_clip",
            "_clear_observations",
            "_export_observations",
            "_on_observation_export_finished",
            "_write_observation_html_summary",
            "_rebuild_observation_map_markers",
            "_warn_gps_unavailable_for_pick",
        },
    ),
    (
        "lrf_mixin",
        {
            "_reset_c13_lrf_for_observe_reset",
            "_read_gimbal_attitude_pair",
            "_lrf_reticle_tracking_active",
            "_capture_lrf_track_ref",
            "_calibrate_lrf_track_after_lock",
            "_clear_lrf_track_ref",
            "_update_lrf_reticle_track",
            "_clear_lrf_lock_geo",
            "_format_lrf_geo_label",
            "_c13_lrf_geo_fov",
            "_update_lrf_lock_geo",
            "_refresh_lrf_lock_overlay",
            "_sync_dooaf_video_marks_on_overlay",
            "set_companion_laser_range_m",
            "enable_c13_lrf_ui",
            "_c13_lrf_is_locked",
            "_on_c13_lrf_icon_clicked",
            "_clear_lrf_failed_reticle",
            "is_c13_lrf_armed",
            "is_c13_lrf_locking",
            "get_c13_lrf_lock_latlon",
            "_sync_lrf_armed_backend",
            "_arm_c13_lrf_lock",
            "_cancel_c13_lrf_arm",
            "_unlock_c13_lrf",
            "_begin_c13_lrf_video_lock",
            "_on_c13_lrf_lock_progress",
            "_on_c13_lrf_lock_finished",
        },
    ),
]

NAME_TO_MIXIN: dict[str, str] = {}
for mixin, names in MIXIN_RULES:
    for n in names:
        NAME_TO_MIXIN[n] = mixin


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


def main() -> None:
    lines = _lines(MAP_WIDGET)
    tree = ast.parse("".join(lines))
    map_nodes = _class_body_nodes(tree, "MapWidget")

    buckets: dict[str, list[str]] = {m: [] for m, _ in MIXIN_RULES}
    task_sources: list[str] = []
    remove_ranges: list[tuple[int, int]] = []

    for node in map_nodes:
        if isinstance(node, ast.ClassDef) and node.name in TASK_CLASSES:
            task_sources.append(_stmt_source(lines, node))
            remove_ranges.append((node.lineno, node.end_lineno or node.lineno))
            continue
        if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        mixin = NAME_TO_MIXIN.get(node.name)
        if mixin is None:
            continue
        buckets[mixin].append(_stmt_source(lines, node))
        remove_ranges.append((node.lineno, node.end_lineno or node.lineno))

    OUT_PKG.mkdir(parents=True, exist_ok=True)

    # types.py — task/bridge classes + re-exports
    types_header = textwrap.dedent(
        '''\
        """Observation / LRF async tasks and pick descriptors."""

        from __future__ import annotations

        import csv
        from dataclasses import dataclass
        from pathlib import Path

        from PySide6.QtCore import QObject, QRunnable, Signal
        from PySide6.QtGui import QImage

        from vgcs.map.image_io import save_qimage_to_path
        from vgcs.observe.dooaf import (
            DOOAF_ROLE_IMPACT,
            assemble_observation_report_html,
            build_dooaf_session,
            format_dooaf_html_summary,
            format_gimbal_pitch_direction,
            format_gimbal_yaw_direction,
            latest_mark_row,
            merge_setup_video_marks,
        )
        from vgcs.observe.grid_reference import format_grid_reference
        from vgcs.observe.dooaf import format_observation_detailed_log_html

        '''
    )
    (OUT_PKG / "types.py").write_text(types_header + "\n\n".join(task_sources), encoding="utf-8")

    mixin_header = textwrap.dedent(
        '''\
        """MapWidget mixin — see vgcs.map.observation package."""

        from __future__ import annotations

        '''
    )

    mixin_imports = {
        "context_mixin": textwrap.dedent(
            '''\
            from PySide6.QtCore import Qt

            from vgcs.observe.dooaf import DOOAF_ROLE_IMPACT
            from vgcs.observe.target_measure import dem_ground_agl_m, resolve_ray_agl_for_geo, sanitize_dem_ground_agl_m
            '''
        ),
        "mark_tracking_mixin": textwrap.dedent(
            '''\
            from PySide6.QtCore import QTimer

            from vgcs.map.native_video_overlay import VideoOverlayMark, VideoOverlayOffscreenHint, offscreen_hint_edge_uv
            from vgcs.observe.dooaf import dooaf_role_display
            from vgcs.observe.dooaf_flight_session import mark_track_use_geo_in_flight
            from vgcs.observe.geo_reference import project_wgs84_to_video_norm, should_project_lrf_mark_via_geo
            from vgcs.observe.target_measure import haversine_m
            '''
        ),
        "dooaf_mixin": textwrap.dedent(
            '''\
            from PySide6.QtCore import QSettings, Qt, QTimer
            from PySide6.QtWidgets import QDialog, QMessageBox

            from vgcs.map.app_settings import QS_APP, QS_ORG
            from vgcs.map.dooaf_setup_dialog import DOOAF_PICK_GUN, DOOAF_PICK_TARGET, DooafSetupDialog
            from vgcs.map.native_video_overlay import VideoOverlayFacadeHint
            from vgcs.map.observation.types import PendingLrfVideoPick
            from vgcs.observe.dooaf import (
                DOOAF_ROLE_GUN,
                DOOAF_ROLE_IMPACT,
                DOOAF_ROLE_INTENDED,
                DOOAF_ROLE_SURVEY,
                DooafSettings,
                _apply_geo_reference_to_mark_row,
                _forced_ray_geo_for_row,
                apply_dooaf_impact_geo_fallback,
                apply_map_pick_to_settings,
                build_dooaf_session,
                clear_dooaf_setup_video_mark,
                dooaf_role_display,
                dooaf_settings_kwargs,
                enrich_dooaf_settings_elevation_from_dem,
                format_dooaf_status,
                latest_mark,
                merge_dooaf_settings,
                merge_setup_video_marks,
                read_dooaf_settings,
                refine_impact_geo_from_video_rays,
                resolved_dooaf_settings,
                write_dooaf_settings,
                write_dooaf_setup_video_mark,
            )
            from vgcs.observe.dooaf_flight_session import build_facade_overlay_hint
            from vgcs.observe.geo_reference import (
                apply_geo_reference_result_to_video_row,
                compute_geo_reference,
                compute_lrf_slant_geo,
            )
            from vgcs.observe.target_measure import (
                haversine_m,
                low_hover_ray_agl_m,
                ray_agl_suspect_dem_mismatch,
            )
            from vgcs.video.camera_control import NoopCameraControl
            '''
        ),
        "session_mixin": textwrap.dedent(
            '''\
            from datetime import datetime, timezone
            from pathlib import Path

            import os
            import shutil
            import tempfile
            import time
            from datetime import datetime, timezone
            from pathlib import Path

            from PySide6.QtCore import QSettings, QThreadPool, QTimer, Qt, QUrl
            from PySide6.QtGui import QDesktopServices, QImage
            from PySide6.QtWidgets import QFileDialog, QMessageBox

            from vgcs.map.app_settings import QS_APP, QS_ORG
            from vgcs.map.image_io import save_qimage_to_path
            from vgcs.map.observation.types import (
                ObservationExportBridge,
                ObservationExportTask,
                ObservationSnapshotBridge,
                ObservationSnapshotTask,
                PendingLrfVideoPick,
            )
            from vgcs.observe.dooaf import (
                DOOAF_ROLE_IMPACT,
                DOOAF_ROLE_INTENDED,
                DOOAF_ROLE_SURVEY,
                apply_dooaf_impact_geo_fallback,
                assemble_observation_report_html,
                build_dooaf_session,
                dooaf_export_blockers,
                format_dooaf_html_summary,
                format_dooaf_status,
                format_observation_detailed_log_html,
                latest_mark,
                latest_mark_row,
            )
            from vgcs.observe.grid_reference import format_grid_reference
            from vgcs.observe.target_measure import (
                MARKS_NOT_LEVEL_HINT,
                band_width_partner_row,
                clear_tape_pair_override,
                format_target_segment_label,
                haversine_m,
                marks_need_level_warning,
                marks_same_height_band,
                measure_agl_ok,
                observation_building_height_segments,
                observation_facade_video_segments,
                observation_target_latlon,
                segment_distance_between_rows,
                segment_distance_video_fallback,
                session_facade_reference_range_m,
                session_peak_geo_range_m,
                session_rangefinder_reference_m,
                target_track_from_observations,
                video_mark_span_norm,
            )
            '''
        ),
        "lrf_mixin": textwrap.dedent(
            '''\
            from PySide6.QtCore import QThreadPool, QTimer

            from vgcs.map.native_video_overlay import VideoOverlayLrfLock
            from vgcs.map.observation.types import LrfLockBridge, LrfLockTask, PendingLrfVideoPick
            from vgcs.observe.dooaf import DOOAF_ROLE_INTENDED
            from vgcs.observe.geo_reference import compute_lrf_slant_geo
            from vgcs.skydroid.protocol import format_slr_display_m
            from vgcs.video.pipeline import notify_companion_lrf_lock
            '''
        ),
    }

    class_names = {
        "context_mixin": "ObservationContextMixin",
        "mark_tracking_mixin": "VideoMarkTrackingMixin",
        "dooaf_mixin": "DooafOperationsMixin",
        "session_mixin": "ObservationSessionMixin",
        "lrf_mixin": "LrfVideoLockMixin",
    }

    for mixin_file, class_name in class_names.items():
        body = buckets[mixin_file]
        if not body:
            print(f"WARN: empty mixin {mixin_file}")
            continue
        content = (
            mixin_header
            + mixin_imports.get(mixin_file, "")
            + f"\n\nclass {class_name}:\n"
            + "    \"\"\"Extracted from MapWidget — uses host widget state via self.\"\"\"\n\n"
            + "\n".join(body)
        )
        (OUT_PKG / f"{mixin_file}.py").write_text(content, encoding="utf-8")
        print(f"Wrote {mixin_file}.py ({len(body)} methods)")

    # Rename classes in types.py for public API
    types_text = (OUT_PKG / "types.py").read_text(encoding="utf-8")
    renames = [
        ("class _PendingLrfVideoPick", "class PendingLrfVideoPick"),
        ("class _LrfLockBridge", "class LrfLockBridge"),
        ("class _LrfLockTask", "class LrfLockTask"),
        ("class _ObservationSnapshotBridge", "class ObservationSnapshotBridge"),
        ("class _ObservationSnapshotTask", "class ObservationSnapshotTask"),
        ("class _ObservationExportBridge", "class ObservationExportBridge"),
        ("class _ObservationExportTask", "class ObservationExportTask"),
        ("_PendingLrfVideoPick", "PendingLrfVideoPick"),
        ("_LrfLockBridge", "LrfLockBridge"),
        ("_ObservationSnapshotBridge", "ObservationSnapshotBridge"),
        ("_ObservationExportBridge", "ObservationExportBridge"),
    ]
    for old, new in renames:
        types_text = types_text.replace(old, new)
    types_text = types_text.replace("_save_qimage_to_path", "save_qimage_to_path")
    (OUT_PKG / "types.py").write_text(types_text, encoding="utf-8")

    # Remove extracted lines from map_widget (bottom-up)
    remove_ranges.sort(key=lambda r: r[0], reverse=True)
    new_lines = list(lines)
    for start, end in remove_ranges:
        del new_lines[start - 1 : end]

    src = "".join(new_lines)
    # Insert mixin imports and base classes
    if "MapObservationMixins" not in src:
        import_block = textwrap.dedent(
            '''\
            from vgcs.map.observation import MapObservationMixins
            from vgcs.map.observation.types import PendingLrfVideoPick as _PendingLrfVideoPick

            '''
        )
        src = src.replace(
            "class MapWidget(QWidget):",
            import_block + "class MapWidget(MapObservationMixins, QWidget):",
            1,
        )
        # Replace references to moved bridge classes in __init__
        replacements = [
            ("_LrfLockBridge", "LrfLockBridge"),
            ("_LrfLockTask", "LrfLockTask"),
            ("_ObservationSnapshotBridge", "ObservationSnapshotBridge"),
            ("_ObservationExportBridge", "ObservationExportBridge"),
        ]
        for old, new in replacements:
            src = src.replace(old, new)
        src = src.replace(
            "from vgcs.map.observation.types import PendingLrfVideoPick as _PendingLrfVideoPick\n\n",
            "from vgcs.map.observation.types import (\n"
            "    LrfLockBridge,\n"
            "    LrfLockTask,\n"
            "    ObservationExportBridge,\n"
            "    ObservationSnapshotBridge,\n"
            "    PendingLrfVideoPick,\n"
            "    PendingLrfVideoPick as _PendingLrfVideoPick,\n"
            ")\n\n",
            1,
        )
        src = src.replace(
            "def _save_qimage_to_path(img: QImage, path: Path) -> bool:\n"
            '    """Write ``QImage`` using extension: ``.png`` → PNG, else JPEG."""\n'
            "    try:\n"
            "        if path.suffix.lower() == \".png\":\n"
            "            return bool(img.save(str(path), \"PNG\"))\n"
            "        return bool(img.save(str(path), \"JPG\", 92))\n"
            "    except Exception:\n"
            "        return False\n",
            "def _save_qimage_to_path(img: QImage, path: Path) -> bool:\n"
            '    """Compatibility wrapper — see :func:`vgcs.map.image_io.save_qimage_to_path`."""\n'
            "    from vgcs.map.image_io import save_qimage_to_path\n\n"
            "    return save_qimage_to_path(img, path)\n",
            1,
        )
        src = src.replace(
            "class MapWidget(MapObservationMixins, QWidget):",
            "class MapWidget(MapObservationMixins, QWidget):",
            1,
        )

    MAP_WIDGET.write_text(src, encoding="utf-8")
    print(f"Updated map_widget.py — removed {len(remove_ranges)} blocks")


if __name__ == "__main__":
    main()
