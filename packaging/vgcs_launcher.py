"""Entry point of the packaged VGCS.exe (PyInstaller build, see packaging/README.md).

Running from source does not use this file. "python -m vgcs" is unchanged.

Three things the source relies on work differently inside the exe:

1. sys.executable is VGCS.exe, not python.exe. The tracker and the detector
   start their worker process with "sys.executable -m <module>". Without the
   handling below, that call would open a second VGCS window instead.
2. FFmpeg is looked up on PATH. When the build bundles FFmpeg, that copy is
   put first on PATH, so the machine does not need FFmpeg installed.
3. Logs, captures and reports are written under the current folder. The exe
   first moves to one fixed, writable data folder (VGCS in Documents). Output
   then lands in the same place however the exe was started, and it survives
   when the program folder is replaced by a newer build.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

# "VGCS.exe -m <module>" only runs modules of the app itself.
_RUNNABLE_MODULE_PREFIX = "vgcs."


def bundle_dir() -> Path:
    """Folder with the bundled files ("_internal" next to VGCS.exe)."""
    return Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parent))


def build_label() -> str:
    """Build stamp written by vgcs.spec, for example "VGCS 0.1.0 build 56aa70d (...)"."""
    try:
        return (bundle_dir() / "vgcs_build.txt").read_text(encoding="utf-8").strip()
    except OSError:
        return "VGCS (no build stamp)"


def run_module_from_argv(argv: list[str]) -> int | None:
    """Run "VGCS.exe -m vgcs.<module> [args]" the way "python -m" does.

    Returns None for a normal start (no "-m"), so the GUI starts.
    """
    if len(argv) < 2 or argv[1] != "-m":
        return None
    module = argv[2] if len(argv) > 2 else ""
    if not module.startswith(_RUNNABLE_MODULE_PREFIX):
        print(f"[VGCS] VGCS.exe -m only runs VGCS modules, not {module!r}", file=sys.stderr)
        return 2
    import runpy

    sys.argv = [argv[0], *argv[3:]]
    runpy.run_module(module, run_name="__main__", alter_sys=True)
    return 0


def make_output_safe_for_pipes() -> None:
    """Write stdout and stderr as UTF-8, one line at a time, when they are pipes.

    The single VGCS.exe runs VGCS without a console and sends its output
    through a pipe into a log file. Python would then encode as cp1252, and a
    print with a character outside it (pipeline.py prints an arrow) would
    raise inside VGCS. It would also hold output back in a buffer, which a
    crash would lose.
    """
    for stream in (sys.stdout, sys.stderr):
        if stream is not None and hasattr(stream, "reconfigure") and not stream.isatty():
            stream.reconfigure(encoding="utf-8", errors="backslashreplace", line_buffering=True)


def put_bundled_ffmpeg_on_path() -> Path | None:
    """Put the bundled FFmpeg first on PATH and return its folder.

    Returns None when this build has no FFmpeg. The app then uses the FFmpeg
    installed on the machine, exactly like running from source.
    """
    ffmpeg_dir = bundle_dir() / "ffmpeg"
    if not (ffmpeg_dir / "ffmpeg.exe").is_file():
        return None
    os.environ["PATH"] = str(ffmpeg_dir) + os.pathsep + os.environ.get("PATH", "")
    return ffmpeg_dir


def choose_data_dir() -> Path:
    """Writable folder for logs, captures and reports.

    First choice is Documents/VGCS, then %LOCALAPPDATA%/VGCS, then the
    current folder.
    """
    candidates: list[Path] = []
    documents = _documents_dir()
    if documents is not None:
        candidates.append(documents / "VGCS")
    local_app_data = os.environ.get("LOCALAPPDATA", "")
    if local_app_data:
        candidates.append(Path(local_app_data) / "VGCS")
    for folder in candidates:
        if _is_writable_dir(folder):
            return folder
    return Path.cwd()


def _documents_dir() -> Path | None:
    """The user's Documents folder, also when OneDrive or a policy moved it."""
    try:
        from PySide6.QtCore import QStandardPaths

        location = QStandardPaths.writableLocation(QStandardPaths.StandardLocation.DocumentsLocation)
    except Exception:
        location = ""
    return Path(location) if location else None


def _is_writable_dir(folder: Path) -> bool:
    try:
        folder.mkdir(parents=True, exist_ok=True)
        probe = folder / ".vgcs-write-test"
        probe.write_bytes(b"")
        probe.unlink()
        return True
    except OSError:
        return False


def main() -> int:
    make_output_safe_for_pipes()
    # Worker processes come first: they must start fast and need nothing else.
    code = run_module_from_argv(sys.argv)
    if code is not None:
        return code
    put_bundled_ffmpeg_on_path()
    data_dir = choose_data_dir()
    if "--selfcheck" in sys.argv[1:]:
        from vgcs_selfcheck import run_selfcheck

        return run_selfcheck(build=build_label(), data_dir=data_dir)
    os.chdir(data_dir)
    print(f"[VGCS] {build_label()}", flush=True)
    print(f"[VGCS] data folder (logs, captures, reports): {data_dir}", flush=True)
    from vgcs.main import main as vgcs_main

    return vgcs_main()


if __name__ == "__main__":
    raise SystemExit(main())
