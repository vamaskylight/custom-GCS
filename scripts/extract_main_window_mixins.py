"""Extract MainWindow methods into composable mixins under vgcs/app/window/."""

from __future__ import annotations

import ast
import textwrap
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN_WINDOW = ROOT / "vgcs" / "app" / "main_window.py"
OUT_PKG = ROOT / "vgcs" / "app" / "window"

UI_LAYOUT = {
    "_wire_camera_control",
    "_detect_compact_ui",
    "_apply_responsive_layout",
    "_after_responsive_layout_changed",
    "_make_value_label",
    "_make_status_chip",
    "_make_top_chip",
    "_hdr_sep_widget",
    "_header_icons_dir",
    "_header_icon_pixmap",
    "_make_hdr_icon_pill",
    "_make_hdr_gps_pill_widget",
    "_top_gps_status_line",
    "_set_top_vehicle_msg",
    "_apply_link_banner_palette",
    "_build_m2_top_dashboard",
    "_logo_scaled_decode_size",
    "_read_png_dimensions",
    "_menu_icon",
    "_build_mission_list_panel",
    "_build_m2_operations_layout",
    "_build_camera_control_panel",
    "_build_primary_flight_footer",
    "_build_compass_footer",
    "_build_nav_system_footer",
    "_build_m2_controls_panel",
    "_set_preconnect_dashboard_mode",
    "_set_map_only_dashboard_mode",
    "_build_header_bar",
    "_build_telemetry_panel",
    "_apply_state_style",
    "_build_theme_colors",
    "_all_state_labels",
    "_refresh_state_styles",
    "_set_ok_warn_field",
    "_append_log",
    "_refresh_footer_summary",
}

PLAN_MISSION = {
    "_on_map_waypoints_changed",
    "_on_mission_table_item_changed",
    "_on_mission_upload_requested",
    "_on_mission_download_requested",
    "_on_mission_uploaded",
    "_on_mission_downloaded",
    "_sync_plan_flight_chrome",
    "_on_plan_flight_exited",
    "_on_map_page_ready",
    "_restore_plan_mission_panel_to_map",
    "_ensure_plan_launch_from_vehicle_if_empty",
    "_on_plan_mission_panel_changed",
    "_default_wp_alt_m_for_plan_state",
    "_plan_takeoff_alt_m_from_launch_settings",
    "_apply_plan_mission_panel_to_model",
    "_refresh_plan_flight_metrics",
    "_offset_lat_lon_m",
    "_pattern_anchor_lat_lon",
    "_plan_pattern_geometry_m",
    "_build_m2_grid_pattern",
    "_build_m2_corridor_pattern",
    "_build_m2_structure_pattern",
    "_on_plan_flight_action",
    "_on_plan_tool_requested",
}

MAP_CHROME = {
    "_sync_hdr_map_mode_btn_label",
    "_on_map_3d_mode_changed",
    "_on_toggle_map_3d",
    "_on_map_toggle_3d_requested",
    "_scroll_main_to",
    "_on_map_menu_requested",
    "_on_toggle_map_3d_shortcut",
    "_on_logo_menu",
    "_build_analyze_tools_report",
    "_on_tiles_online",
    "_on_tiles_offline",
}

SETTINGS_DIALOGS = {
    "_show_application_settings_dialog",
    "_show_vehicle_configuration_help",
    "_show_vehicle_quick_controls_dialog",
    "_show_flight_controls_dialog",
}

FLIGHT_STATUS = {
    "_set_dashboard_flight_status",
    "_flight_status_not_ready_label",
    "_is_probably_flying",
    "_refresh_dashboard_flight_state",
    "_normalize_mode_token",
    "_is_home_wait_prearm_reason",
    "_is_non_gps_mode",
    "_prearm_block_active",
    "_compute_hb_arm_ready",
    "_update_prearm_gate_from_statustext",
    "_push_map_flight_overlay",
    "_sync_visible_map_overlay_metrics",
    "_maybe_refresh_map_web_overlays",
}

LINK = {
    "_on_connect",
    "_on_disconnect",
    "_stop_camera_control_backend",
    "_deferred_apply_saved_video_settings",
    "_deferred_apply_saved_video_settings_camera",
    "_set_runtime_camera_control",
    "_on_link_up",
    "_on_link_down",
    "_format_recent_vehicle_msgs_for_alert",
    "_on_link_timeout",
    "_on_link_error",
}

TELEMETRY = {
    "_on_heartbeat",
    "_on_telemetry",
    "_reset_telemetry_fields",
    "_extract_remote_id_text",
    "_haversine_m",
    "_refresh_c13_lrf_display",
}

FLIGHT_COMMANDS = {
    "_on_set_mode",
    "_on_mode_change_result",
    "_takeoff_altitude_m",
    "_queue_nav_takeoff",
    "_on_takeoff",
    "_on_land",
    "_on_auto_takeoff",
    "_on_auto_land",
    "_on_emergency_motor_stop",
    "_on_apply_m1_failsafes",
    "_on_upload_fence",
    "_on_map_geofence_requested",
    "_suppress_header_connect_spurious_reopen",
    "_clear_header_connect_suppression",
    "_on_map_connect_requested",
    "_on_map_return_requested",
    "_on_map_mission_start_requested",
}

PARAMS = {
    "_on_params_refresh",
    "_on_param_set",
    "_on_action_result",
    "_on_geofence_result",
    "_on_params_snapshot",
    "_apply_params_snapshot_payload",
    "_refresh_acro_options_ui",
    "_on_apply_acro_options",
    "_on_apply_simple_options",
    "_on_param_set_result",
    "_sync_mode_options_for_vehicle",
}

WINDOW_LIFECYCLE = {
    "_on_timeout_changed",
    "_on_reset_telemetry",
    "_on_theme_changed",
    "_restore_window_geometry",
    "_fit_to_screen",
    "resizeEvent",
    "_on_restore_defaults",
    "_on_flight_timer_tick",
    "_on_dev_reload",
    "_on_thread_finished",
    "_on_application_state_changed",
    "changeEvent",
    "closeEvent",
}

MIXIN_RULES: list[tuple[str, set[str]]] = [
    ("ui_layout_mixin", UI_LAYOUT),
    ("plan_mission_mixin", PLAN_MISSION),
    ("map_chrome_mixin", MAP_CHROME),
    ("settings_dialogs_mixin", SETTINGS_DIALOGS),
    ("flight_status_mixin", FLIGHT_STATUS),
    ("link_mixin", LINK),
    ("telemetry_mixin", TELEMETRY),
    ("flight_commands_mixin", FLIGHT_COMMANDS),
    ("params_mixin", PARAMS),
    ("window_lifecycle_mixin", WINDOW_LIFECYCLE),
]

NAME_TO_MIXIN: dict[str, str] = {}
for mixin, names in MIXIN_RULES:
    for n in names:
        NAME_TO_MIXIN[n] = mixin

MODULE_HELPERS = {
    "_settings_truthy": "helpers.py",
    "_mavlink_autopilot_label": "helpers.py",
    "_mavlink_vehicle_type_label": "helpers.py",
}


def _lines(path: Path) -> list[str]:
    return path.read_text(encoding="utf-8").splitlines(keepends=True)


def _class_body(tree: ast.Module, name: str) -> list[ast.stmt]:
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == name:
            return list(node.body)
    raise RuntimeError(f"class {name} not found")


def _stmt(lines: list[str], node: ast.stmt) -> str:
    start = node.lineno - 1
    if isinstance(node, ast.ClassDef) and node.decorator_list:
        start = node.decorator_list[0].lineno - 1
    end = getattr(node, "end_lineno", node.lineno)
    return "".join(lines[start:end])


def _strip_orphan_staticmethods(text: str) -> str:
    out: list[str] = []
    lines = text.splitlines(keepends=True)
    i = 0
    while i < len(lines):
        if lines[i].strip() == "@staticmethod":
            j = i + 1
            while j < len(lines) and not lines[j].strip():
                j += 1
            if j < len(lines) and lines[j].lstrip().startswith("def "):
                i += 1
                continue
        out.append(lines[i])
        i += 1
    return "".join(out)


def main() -> None:
    lines = _lines(MAIN_WINDOW)
    tree = ast.parse("".join(lines))
    nodes = _class_body(tree, "MainWindow")

    buckets: dict[str, list[str]] = {m: [] for m, _ in MIXIN_RULES}
    remove: list[tuple[int, int]] = []

    for node in nodes:
        if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        mixin = NAME_TO_MIXIN.get(node.name)
        if mixin is None:
            continue
        buckets[mixin].append(_stmt(lines, node))
        remove.append((node.lineno, node.end_lineno or node.lineno))

    mod_fns: dict[str, ast.FunctionDef] = {}
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name in MODULE_HELPERS:
            mod_fns[node.name] = node
            remove.append((node.lineno, node.end_lineno or node.lineno))

    OUT_PKG.mkdir(parents=True, exist_ok=True)

    helper_parts = [
        '"""Shared helpers for MainWindow mixins."""\n\nfrom __future__ import annotations\n\n'
        "import struct\n\n"
        "from pymavlink import mavutil\n\n"
    ]
    for name in sorted(MODULE_HELPERS):
        if name in mod_fns:
            helper_parts.append(_stmt(lines, mod_fns[name]))
    (OUT_PKG / "helpers.py").write_text("\n".join(helper_parts), encoding="utf-8")

    mixin_header = '"""MainWindow mixin — see vgcs.app.window package."""\n\nfrom __future__ import annotations\n\n'
    shared_imports = textwrap.dedent(
        """\
        import math
        import time
        from collections import deque
        from pathlib import Path

        from PySide6.QtCore import QEvent, QObject, QPoint, QSize, Qt, QSettings, QTimer
        from PySide6.QtGui import (
            QColor,
            QGuiApplication,
            QIcon,
            QImage,
            QImageReader,
            QKeySequence,
            QPainter,
            QPixmap,
            QShortcut,
        )
        from PySide6.QtWidgets import (
            QApplication,
            QCheckBox,
            QComboBox,
            QDialog,
            QDoubleSpinBox,
            QFrame,
            QFormLayout,
            QGridLayout,
            QGroupBox,
            QHBoxLayout,
            QLabel,
            QLineEdit,
            QInputDialog,
            QMenu,
            QMessageBox,
            QPushButton,
            QScrollArea,
            QScrollBar,
            QSizePolicy,
            QTableWidget,
            QTableWidgetItem,
            QVBoxLayout,
            QWidget,
            QListWidget,
            QListWidgetItem,
            QStackedWidget,
            QSpinBox,
            QStyle,
            QTextEdit,
            QTabWidget,
            QRadioButton,
            QButtonGroup,
            QFileDialog,
        )
        from pymavlink import mavutil

        from vgcs.app.window.helpers import (
            _mavlink_autopilot_label,
            _mavlink_vehicle_type_label,
            _settings_truthy,
        )
        from vgcs.app.gcs_style import gcs_stylesheet
        from vgcs.app.runtime_ui import build_base_font, select_font_profile
        from vgcs.mode import AP_COPTER_MODE_MAP, human_mode_name, modes_for_vehicle_type
        from vgcs.mission import Waypoint
        from vgcs.map import MapWidget
        from vgcs.map.map_web_3d import HAS_WEBENGINE as HAS_MAP_WEBENGINE
        from vgcs.app.widgets import CompassWidget
        from vgcs.link.mavlink_thread import MavlinkThread
        from vgcs.video.pipeline import VideoPipeline
        from vgcs.video.widgets import CameraControlPanel
        from vgcs.video.camera_control import (
            CompositeGimbalCameraControl,
            MavlinkCameraControl,
            NoopCameraControl,
            read_companion_laser_range_m,
            poll_companion_laser_range_m,
            SiyiCameraControl,
            SkydroidCameraControl,
            resolve_siyi_host,
            resolve_skydroid_control_hosts,
            resolve_skydroid_host,
        )
        """
    )

    class_names = {
        "ui_layout_mixin": "MainWindowUiLayoutMixin",
        "plan_mission_mixin": "MainWindowPlanMissionMixin",
        "map_chrome_mixin": "MainWindowMapChromeMixin",
        "settings_dialogs_mixin": "MainWindowSettingsDialogsMixin",
        "flight_status_mixin": "MainWindowFlightStatusMixin",
        "link_mixin": "MainWindowLinkMixin",
        "telemetry_mixin": "MainWindowTelemetryMixin",
        "flight_commands_mixin": "MainWindowFlightCommandsMixin",
        "params_mixin": "MainWindowParamsMixin",
        "window_lifecycle_mixin": "MainWindowLifecycleMixin",
    }

    for mixin_file, class_name in class_names.items():
        body = buckets[mixin_file]
        print(f"Wrote {mixin_file}.py ({len(body)} methods)")
        content = (
            mixin_header
            + shared_imports
            + f"\n\nclass {class_name}:\n"
            + '    """Extracted from MainWindow — uses host state via self."""\n\n'
            + "\n".join(body)
        )
        (OUT_PKG / f"{mixin_file}.py").write_text(content, encoding="utf-8")

    init_py = textwrap.dedent(
        '''\
        """Composable mixins for :class:`vgcs.app.main_window.MainWindow`."""

        from __future__ import annotations

        from vgcs.app.window.flight_commands_mixin import MainWindowFlightCommandsMixin
        from vgcs.app.window.flight_status_mixin import MainWindowFlightStatusMixin
        from vgcs.app.window.link_mixin import MainWindowLinkMixin
        from vgcs.app.window.map_chrome_mixin import MainWindowMapChromeMixin
        from vgcs.app.window.params_mixin import MainWindowParamsMixin
        from vgcs.app.window.plan_mission_mixin import MainWindowPlanMissionMixin
        from vgcs.app.window.settings_dialogs_mixin import MainWindowSettingsDialogsMixin
        from vgcs.app.window.telemetry_mixin import MainWindowTelemetryMixin
        from vgcs.app.window.ui_layout_mixin import MainWindowUiLayoutMixin
        from vgcs.app.window.window_lifecycle_mixin import MainWindowLifecycleMixin


        class MainWindowMixins(
            MainWindowUiLayoutMixin,
            MainWindowPlanMissionMixin,
            MainWindowMapChromeMixin,
            MainWindowSettingsDialogsMixin,
            MainWindowFlightStatusMixin,
            MainWindowLinkMixin,
            MainWindowTelemetryMixin,
            MainWindowFlightCommandsMixin,
            MainWindowParamsMixin,
            MainWindowLifecycleMixin,
        ):
            """Mixin bundle for the GCS main window."""


        __all__ = [
            "MainWindowMixins",
            "MainWindowFlightCommandsMixin",
            "MainWindowFlightStatusMixin",
            "MainWindowLifecycleMixin",
            "MainWindowLinkMixin",
            "MainWindowMapChromeMixin",
            "MainWindowParamsMixin",
            "MainWindowPlanMissionMixin",
            "MainWindowSettingsDialogsMixin",
            "MainWindowTelemetryMixin",
            "MainWindowUiLayoutMixin",
        ]
        '''
    )
    (OUT_PKG / "__init__.py").write_text(init_py, encoding="utf-8")

    # Remove module helpers from main_window (already extracted).
    for fn in MODULE_HELPERS:
        drop_fn = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == fn]
        for n in drop_fn:
            if (n.lineno, n.end_lineno or n.lineno) not in remove:
                remove.append((n.lineno, n.end_lineno or n.lineno))

    drop: set[int] = set()
    for start, end in remove:
        drop.update(range(start, end + 1))
    new_lines = [line for i, line in enumerate(lines, start=1) if i not in drop]
    src = _strip_orphan_staticmethods("".join(new_lines))

    if "class MainWindow" in src:
        lines_out = src.splitlines(keepends=True)
        init_idx = next(
            (i for i, ln in enumerate(lines_out) if ln.strip().startswith("def __init__")),
            None,
        )
        if init_idx is not None:
            end_idx = len(lines_out)
            for j in range(init_idx + 1, len(lines_out)):
                stripped = lines_out[j].strip()
                if lines_out[j].startswith("    def ") or lines_out[j].strip() == "@staticmethod":
                    end_idx = j
                    break
            if end_idx < len(lines_out):
                src = "".join(lines_out[:end_idx])

    if "MainWindowMixins" not in src:
        src = src.replace(
            "class MainWindow(QMainWindow):",
            "class MainWindow(MainWindowMixins, QMainWindow):",
            1,
        )
        src = src.replace(
            "from vgcs.app.gcs_style import gcs_stylesheet",
            "from vgcs.app.gcs_style import gcs_stylesheet\n"
            "from vgcs.app.window import MainWindowMixins\n"
            "from vgcs.app.window.helpers import _settings_truthy",
            1,
        )

    # Drop leftover module-level helpers if still present.
    for fn in MODULE_HELPERS:
        src = "\n".join(
            line for line in src.splitlines() if not line.startswith(f"def {fn}(")
        ) + "\n"

    out_path = MAIN_WINDOW.with_suffix(".py.new")
    out_path.write_text(src, encoding="utf-8")
    out_path.replace(MAIN_WINDOW)
    print(f"Updated main_window.py — removed {len(remove)} blocks")


if __name__ == "__main__":
    main()
