"""Split vgcs/observe/dooaf.py into types / correction / report with a thin facade."""

from __future__ import annotations

import ast
import textwrap
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DOOAF = ROOT / "vgcs" / "observe" / "dooaf.py"
OUT_TYPES = ROOT / "vgcs" / "observe" / "_dooaf_types.py"
OUT_CORRECTION = ROOT / "vgcs" / "observe" / "_dooaf_correction.py"
OUT_REPORT = ROOT / "vgcs" / "observe" / "_dooaf_report.py"

REPORT_FIRST_LINE = 1727  # _html_esc

TYPE_CLASSES = {"GeoPoint", "FireCorrection", "DooafSession", "DooafSettings", "DooafPreset"}
REPORT_CLASSES = {"_FcSvgLabelPlacer", "_FcMissPlotPoints"}
CORRECTION_TAIL = {"dooaf_intended_impact_video_segment"}
TYPE_FUNCS = {"dooaf_role_display"}


def _lines(path: Path) -> list[str]:
    return path.read_text(encoding="utf-8").splitlines(keepends=True)


def _stmt(lines: list[str], node: ast.stmt) -> str:
    start = node.lineno - 1
    if isinstance(node, ast.ClassDef) and node.decorator_list:
        start = node.decorator_list[0].lineno - 1
    end = getattr(node, "end_lineno", node.lineno)
    return "".join(lines[start:end])


def main() -> None:
    lines = _lines(DOOAF)
    tree = ast.parse("".join(lines))

    type_parts: list[str] = []
    correction_parts: list[str] = []
    report_parts: list[str] = []
    remove: list[tuple[int, int]] = []

    for node in tree.body:
        if isinstance(node, ast.ClassDef):
            if node.name in TYPE_CLASSES:
                type_parts.append(_stmt(lines, node))
                remove.append((node.lineno, node.end_lineno or node.lineno))
            elif node.name in REPORT_CLASSES:
                report_parts.append(_stmt(lines, node))
                remove.append((node.lineno, node.end_lineno or node.lineno))
        elif isinstance(node, ast.FunctionDef):
            if node.name in TYPE_FUNCS:
                type_parts.append(_stmt(lines, node))
            elif node.lineno >= REPORT_FIRST_LINE and node.name not in CORRECTION_TAIL:
                report_parts.append(_stmt(lines, node))
            else:
                correction_parts.append(_stmt(lines, node))
            remove.append((node.lineno, node.end_lineno or node.lineno))
        elif isinstance(node, ast.Assign):
            names = [t.id for t in node.targets if isinstance(t, ast.Name)]
            if any(
                n.startswith("DOOAF_") or n in ("_SETUP_MARK_ROLE_ALIASES",)
                for n in names
            ):
                type_parts.append(_stmt(lines, node))
                remove.append((node.lineno, node.end_lineno or node.lineno))
            elif any(n.startswith("_QS_") for n in names):
                correction_parts.append(_stmt(lines, node))
                remove.append((node.lineno, node.end_lineno or node.lineno))
            elif node.lineno < REPORT_FIRST_LINE:
                correction_parts.append(_stmt(lines, node))
                remove.append((node.lineno, node.end_lineno or node.lineno))
        elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            name = node.target.id
            if name.startswith("DOOAF_") or name == "_SETUP_MARK_ROLE_ALIASES":
                type_parts.append(_stmt(lines, node))
                remove.append((node.lineno, node.end_lineno or node.lineno))
            elif name.startswith("_QS_"):
                correction_parts.append(_stmt(lines, node))
                remove.append((node.lineno, node.end_lineno or node.lineno))
            elif node.lineno < REPORT_FIRST_LINE:
                correction_parts.append(_stmt(lines, node))
                remove.append((node.lineno, node.end_lineno or node.lineno))

    types_header = textwrap.dedent(
        '''\
        """DOOAF roles, dataclasses, and operator-facing labels."""

        from __future__ import annotations

        from dataclasses import dataclass


        '''
    )
    correction_header = textwrap.dedent(
        '''\
        """DOOAF fire-correction math, geo, settings, and session assembly."""

        from __future__ import annotations

        import json
        import math
        from typing import Any

        from vgcs.observe._dooaf_types import (
            DOOAF_ROLE_GUN,
            DOOAF_ROLE_IMPACT,
            DOOAF_ROLE_INTENDED,
            DOOAF_ROLE_SURVEY,
            DOOAF_ROLES,
            DooafPreset,
            DooafSession,
            DooafSettings,
            FireCorrection,
            GeoPoint,
            _SETUP_MARK_ROLE_ALIASES,
            dooaf_role_display,
        )
        from vgcs.observe.target_measure import (
            haversine_m,
            low_hover_ray_agl_m,
            observation_ekf_rel_alt_m,
            observation_target_latlon,
        )


        '''
    )
    report_header = textwrap.dedent(
        '''\
        """DOOAF / observation HTML reports and fire-correction diagrams."""

        from __future__ import annotations

        import math
        from dataclasses import dataclass
        from typing import Any

        from vgcs.observe._dooaf_correction import (
            _float_or_none,
            build_dooaf_session,
            format_fire_correction,
            format_gimbal_pitch_direction,
            format_gimbal_yaw_direction,
            initial_bearing_deg,
            latlon_delta_to_ne_m,
            latest_mark_row,
        )
        from vgcs.observe._dooaf_types import (
            DOOAF_ROLE_GUN,
            DOOAF_ROLE_IMPACT,
            DOOAF_ROLE_INTENDED,
            DooafSession,
            FireCorrection,
            GeoPoint,
            dooaf_role_display,
        )
        from vgcs.observe.dooaf_map_symbols import (
            bearing_deg as _dooaf_bearing_deg,
            svg_drone_marker as _fc_svg_marker_drone,
            svg_gun_marker as _fc_svg_marker_gun,
            svg_target_marker as _fc_svg_marker_crosshair,
        )
        from vgcs.observe.grid_reference import format_grid_reference


        '''
    )

    OUT_TYPES.write_text(types_header + "\n".join(type_parts) + "\n", encoding="utf-8")
    OUT_CORRECTION.write_text(
        correction_header + "\n".join(correction_parts) + "\n", encoding="utf-8"
    )
    OUT_REPORT.write_text(report_header + "\n".join(report_parts) + "\n", encoding="utf-8")

    facade = textwrap.dedent(
        '''\
        """
        DOOAF (Detection, Observation, Orientation & Adjustment of Fire) session state.

        Facade re-export — implementation split across _dooaf_types, _dooaf_correction, _dooaf_report.
        """

        from __future__ import annotations

        from vgcs.observe import _dooaf_correction, _dooaf_report, _dooaf_types


        def _reexport(module: object) -> None:
            g = globals()
            for name, val in vars(module).items():
                if name.startswith("__"):
                    continue
                g[name] = val


        _reexport(_dooaf_types)
        _reexport(_dooaf_correction)
        _reexport(_dooaf_report)
        '''
    )
    DOOAF.write_text(facade, encoding="utf-8")
    print(
        f"Wrote types ({len(type_parts)} blocks), "
        f"correction ({len(correction_parts)}), report ({len(report_parts)})"
    )


if __name__ == "__main__":
    main()
