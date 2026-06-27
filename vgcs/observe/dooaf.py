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
