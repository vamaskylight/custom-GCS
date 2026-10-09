"""Find FFmpeg when it is not on PATH, and say so when there is none.

VGCS runs FFmpeg as a separate program for the camera video and for recording,
and looks it up on PATH (``shutil.which``). VGCS.exe brings its own FFmpeg and
puts it first on PATH (packaging/vgcs_launcher.py). A run from source uses the
FFmpeg of the PC.

On a PC without FFmpeg the video area stayed on "connecting" and the reason was
only in the console (client log of 2026-10-09: "ffmpeg not found in PATH"). So
before the window opens the usual places are looked through, the first FFmpeg
found is put on PATH, and when there is none the video area says what is
missing and how to get it.
"""

from __future__ import annotations

import os
import shutil
from collections.abc import Iterable, Mapping
from pathlib import Path

# The folder that holds ffmpeg.exe. The exe build reads the same name.
FFMPEG_DIR_ENV = "VGCS_FFMPEG_DIR"

INSTALL_COMMAND = "winget install Gyan.FFmpeg"
FETCH_COMMAND = "py packaging\\build_exe.py --ffmpeg-only"

# For the video area, which is a small box: short lines.
MISSING_ON_SCREEN = (
    "No video: FFmpeg is not installed on this PC.\n"
    "Install it, then start VGCS again:\n"
    f"{INSTALL_COMMAND}"
)

MISSING_IN_CONSOLE = (
    "[VGCS:video] FFmpeg was not found on this PC. The camera video and recording need it.",
    f"[VGCS:video]   Install it:  {INSTALL_COMMAND}   (then open a new terminal and start VGCS again)",
    f"[VGCS:video]   or fetch the tested one into this folder:  {FETCH_COMMAND}",
    f"[VGCS:video]   or set {FFMPEG_DIR_ENV} to the folder that holds ffmpeg.exe.",
    "[VGCS:video]   VGCS.exe has FFmpeg inside and needs none of this.",
)


def _exe_name() -> str:
    return "ffmpeg.exe" if os.name == "nt" else "ffmpeg"


def has_ffmpeg(folder: Path | str) -> bool:
    """Whether this folder holds FFmpeg. The file itself is looked at:
    ``shutil.which`` with a path also searches the current folder on Windows."""
    try:
        return (Path(folder) / _exe_name()).is_file()
    except OSError:
        return False


def ffmpeg_available() -> bool:
    """The question the video pipeline asks before it starts FFmpeg."""
    return shutil.which("ffmpeg") is not None


def _newest_first(folders: Iterable[Path]) -> list[Path]:
    def changed(path: Path) -> float:
        try:
            return path.stat().st_mtime
        except OSError:
            return 0.0

    return sorted(folders, key=changed, reverse=True)


def candidate_dirs(env: Mapping[str, str] | None = None, repo_root: Path | None = None) -> list[Path]:
    """Folders where an FFmpeg may be, the most deliberate choice first."""
    env = os.environ if env is None else env
    out: list[Path] = []

    chosen = str(env.get(FFMPEG_DIR_ENV, "") or "").strip()
    if chosen:
        out.append(Path(chosen))

    # The tested FFmpeg that packaging/build_exe.py downloads into the repo.
    root = Path(__file__).resolve().parents[2] if repo_root is None else Path(repo_root)
    try:
        out.extend(sorted((p for p in (root / "build" / "ffmpeg").glob("ffmpeg-*") if p.is_dir()), reverse=True))
    except OSError:
        pass

    local = str(env.get("LOCALAPPDATA", "") or "").strip()
    if local:
        base = Path(local)
        # A VGCS.exe that was started on this PC unpacked its FFmpeg here.
        try:
            out.extend(_newest_first((base / "VGCS" / "app").glob("*/_internal/ffmpeg")))
        except OSError:
            pass
        # winget: its links folder is on PATH only in terminals opened after the install.
        out.append(base / "Microsoft" / "WinGet" / "Links")
        try:
            out.extend(_newest_first((base / "Microsoft" / "WinGet" / "Packages").glob("Gyan.FFmpeg*/*/bin")))
        except OSError:
            pass

    for name in ("ProgramFiles", "ProgramFiles(x86)"):
        base_text = str(env.get(name, "") or "").strip()
        if base_text:
            out.append(Path(base_text) / "ffmpeg" / "bin")
    drive = str(env.get("SystemDrive", "") or "").strip()
    if drive:
        out.append(Path(drive + os.sep) / "ffmpeg" / "bin")
    data = str(env.get("ProgramData", "") or "").strip()
    if data:
        out.append(Path(data) / "chocolatey" / "bin")
    home = str(env.get("USERPROFILE", "") or "").strip()
    if home:
        out.append(Path(home) / "scoop" / "shims")
    return out


def ensure_ffmpeg_on_path(candidates: Iterable[Path] | None = None, say=print) -> Path | None:
    """The folder of the FFmpeg VGCS will run, put on PATH when it was not. None: there is none.

    An FFmpeg already on PATH is left as it is: the PC's own choice comes first.
    """
    found = shutil.which("ffmpeg")
    if found:
        return Path(found).parent
    for folder in candidate_dirs() if candidates is None else candidates:
        if has_ffmpeg(folder):
            os.environ["PATH"] = str(folder) + os.pathsep + os.environ.get("PATH", "")
            say(f"[VGCS:video] FFmpeg is not on PATH. Using the one in {folder}")
            return Path(folder)
    for line in MISSING_IN_CONSOLE:
        say(line)
    return None
