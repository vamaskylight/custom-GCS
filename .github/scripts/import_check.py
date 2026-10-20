"""Import every module of the vgcs package, as a CI check on a clean checkout.

The unit test suite (tests/) is kept out of the repo, so CI cannot run it. This
check still catches what a clean checkout gets wrong: a module that imports a
file that was never committed, a syntax error, or a class that cannot be built.

Run from the repo root:  python .github/scripts/import_check.py
"""

from __future__ import annotations

import importlib
import os
import pathlib
import pkgutil
import sys
import tempfile
import traceback

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
# The map's tile fetcher reads this, so nothing here reaches a tile server.
os.environ.setdefault("VGCS_NO_NETWORK", "1")

from PySide6 import QtCore  # noqa: E402

# VGCS modules bind QSettings when imported, and QSettings("VGCS", "VGCS") is the
# user's own settings (the registry on Windows). Every QSettings goes to an INI
# file in a temporary folder instead, the same as tests/conftest.py.
_SETTINGS_DIR = pathlib.Path(tempfile.mkdtemp(prefix="vgcs-import-check-"))
_RealQSettings = QtCore.QSettings


class _CheckSettings(_RealQSettings):
    def __init__(self, *args, **kwargs):
        if args and isinstance(args[0], str) and not args[0].lower().endswith(".ini"):
            org = args[0]
            app = args[1] if len(args) > 1 and isinstance(args[1], str) else "default"
            super().__init__(str(_SETTINGS_DIR / f"{org}-{app}.ini"), _RealQSettings.Format.IniFormat)
        elif not args:
            super().__init__(str(_SETTINGS_DIR / "default.ini"), _RealQSettings.Format.IniFormat)
        else:
            super().__init__(*args, **kwargs)


QtCore.QSettings = _CheckSettings

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

import vgcs  # noqa: E402


def main() -> int:
    names = [m.name for m in pkgutil.walk_packages(vgcs.__path__, "vgcs.")]
    failed: list[tuple[str, str]] = []
    for name in names:
        try:
            importlib.import_module(name)
        except Exception:
            failed.append((name, traceback.format_exc()))
    print(f"{len(names) - len(failed)} of {len(names)} vgcs modules import")
    for name, trace in failed:
        print(f"\nFAILED: {name}\n{trace}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
