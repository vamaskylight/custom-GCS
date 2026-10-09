"""Which build of VGCS is running, for the window title.

The exe prints its build stamp at the start of its log. Nothing on the screen
said it, so "which exe do you have?" could only be answered with a log file
(2026-10-09: the client judged the new work from an exe built six days before
it). The window title names the build now, and a run from source names its
commit.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

from vgcs import __version__

_STAMP_FILE = "vgcs_build.txt"
# "VGCS 0.1.0 build 56aa70d with local changes (2026-10-03 04:19 UTC)", written by packaging/vgcs.spec.
_STAMP = re.compile(r"build\s+(?P<commit>[0-9a-f]{4,40})(?P<changed>\s+with local changes)?\s*(?:\((?P<when>[^)]*)\))?")


def _bundle_stamp(bundle: Path | None) -> str:
    if bundle is None:
        return ""
    try:
        return (bundle / _STAMP_FILE).read_text(encoding="utf-8").strip()
    except OSError:
        return ""


def _source_commit(repo: Path) -> str:
    """The checked-out commit, read from .git without starting git."""
    try:
        git = repo / ".git"
        head = (git / "HEAD").read_text(encoding="utf-8").strip()
        if head.startswith("ref:"):
            ref = head.split(":", 1)[1].strip()
            ref_file = git / ref
            if ref_file.is_file():
                head = ref_file.read_text(encoding="utf-8").strip()
            else:
                head = ""
                for line in (git / "packed-refs").read_text(encoding="utf-8").splitlines():
                    if line.endswith(" " + ref):
                        head = line.split(" ", 1)[0]
                        break
        return head[:7] if re.fullmatch(r"[0-9a-f]{7,40}", head) else ""
    except OSError:
        return ""


def build_label(bundle: Path | None = None, repo: Path | None = None) -> str:
    """Short words for the build: "build e3ee0d1 (2026-10-08 22:30 UTC)" or "source e3ee0d1"."""
    if bundle is None and getattr(sys, "frozen", False):
        bundle = Path(getattr(sys, "_MEIPASS", Path(sys.executable).parent))
    stamp = _bundle_stamp(bundle)
    if stamp:
        found = _STAMP.search(stamp)
        if found:
            label = f"build {found.group('commit')}"
            if found.group("changed"):
                label += " with local changes"
            if found.group("when"):
                label += f" ({found.group('when')})"
            return label
        return stamp
    if bundle is not None:
        return "build not named"
    commit = _source_commit(Path(__file__).resolve().parents[2] if repo is None else repo)
    return f"source {commit}" if commit else "source"


def window_title(base: str = "VGCS — Ground Control Station") -> str:
    return f"{base}   {__version__} {build_label()}"
