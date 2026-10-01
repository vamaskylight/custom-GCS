# -*- mode: python ; coding: utf-8 -*-
# PyInstaller spec for VGCS.exe. Build with:  py -3.14 packaging/build_exe.py
# See packaging/README.md. This builds the folder (dist/VGCS). build_exe.py then
# wraps that folder into the single VGCS.exe. Environment variables it sets:
#   VGCS_FFMPEG_DIR  folder with ffmpeg.exe and ffprobe.exe to bundle (optional)
#   VGCS_ICON        .ico file for the exe (optional)

import os
import re
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

from PyInstaller.utils.hooks import (
    collect_data_files,
    collect_delvewheel_libs_directory,
    collect_submodules,
)

REPO = Path(SPECPATH).resolve().parent
PACKAGING = REPO / "packaging"
sys.path.insert(0, str(REPO))  # so collect_submodules("vgcs") can import the package


def git(*args):
    return subprocess.run(
        ["git", *args], cwd=REPO, capture_output=True, text=True, check=True
    ).stdout


def vgcs_data_files():
    """Every non-Python file of the vgcs package that git tracks.

    git decides what ships: gitignored files are internal (for example
    vgcs/assets/models/MODEL_SOURCE.md) and must not reach the client.
    """
    files = []
    for rel in git("ls-files", "-z", "--", "vgcs").split("\0"):
        if not rel or rel.endswith((".py", ".pyc")) or not (REPO / rel).is_file():
            continue
        files.append((str(REPO / rel), str(Path(rel).parent)))
    untracked = [
        rel
        for rel in git("ls-files", "-z", "--others", "--exclude-standard", "--", "vgcs").split("\0")
        if rel and not rel.endswith(".py")
    ]
    for rel in untracked:
        print(f"WARNING: not bundled because git does not track it: {rel}")
    return files


def build_stamp_file():
    """Write vgcs_build.txt. The exe prints it at start, so a field log names its build."""
    version = re.search(
        r'__version__\s*=\s*"([^"]+)"', (REPO / "vgcs" / "__init__.py").read_text(encoding="utf-8")
    ).group(1)
    commit = git("rev-parse", "--short", "HEAD").strip()
    changed = " with local changes" if git("status", "--porcelain", "--untracked-files=no").strip() else ""
    when = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC")
    path = Path(workpath) / "vgcs_build.txt"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(f"VGCS {version} build {commit}{changed} ({when})\n", encoding="utf-8")
    return str(path)


def mgrs_library():
    """mgrs loads libmgrs.<abi>.pyd with ctypes from the folder above its package.

    Nothing imports it, so PyInstaller cannot find it on its own. It goes to the
    bundle root, which is the folder above the bundled mgrs package.
    """
    import importlib.util

    site = Path(importlib.util.find_spec("mgrs").origin).parent.parent
    found = sorted(site.glob("libmgrs*.pyd"))
    if not found:
        raise SystemExit(f"libmgrs*.pyd not found in {site}")
    return [(str(p), ".") for p in found]


FFMPEG_DOCS = ("LICENSE", "LICENSE.txt", "README.txt", "FFMPEG-SOURCE.txt")


def ffmpeg_files():
    """(binaries, datas) for the bundled FFmpeg, from VGCS_FFMPEG_DIR."""
    folder = os.environ.get("VGCS_FFMPEG_DIR", "").strip()
    if not folder:
        print("NOTE: FFmpeg is not bundled. The exe will use FFmpeg from PATH, like running from source.")
        return [], []
    folder = Path(folder)
    for exe in ("ffmpeg.exe", "ffprobe.exe"):
        if not (folder / exe).is_file():
            raise SystemExit(f"{exe} not found in VGCS_FFMPEG_DIR={folder}")
    # The programs, plus the DLLs of a "shared" FFmpeg build. Not ffplay: VGCS does not use it.
    binaries = [
        (str(p), "ffmpeg")
        for p in folder.iterdir()
        if p.suffix.lower() in (".exe", ".dll") and p.name.lower() != "ffplay.exe"
    ]
    # FFmpeg is GPL: its license and source notes travel with it.
    datas = [(str(folder / name), "ffmpeg") for name in FFMPEG_DOCS if (folder / name).is_file()]
    return binaries, datas


def unneeded_reason(dest):
    """Why VGCS does not need a file PyInstaller collected, or None to keep it."""
    d = dest.replace("\\", "/")
    name = d.rsplit("/", 1)[-1]
    if d.startswith(("PySide6/qml/", "PySide6/plugins/qmltooling/")):
        return "Qt QML files (VGCS has no QML screens)"
    if d.startswith("PySide6/translations/") and d != "PySide6/translations/qtwebengine_locales/en-US.pak":
        return "Qt translations (VGCS is in English; WebEngine falls back to en-US)"
    if d.startswith("PySide6/resources/") and (
        ".debug." in name or name.startswith("qtwebengine_devtools_resources")
    ):
        return "Qt WebEngine debug-build and developer-tools files"
    if d.startswith("cv2/") and name.startswith("opencv_videoio_ffmpeg"):
        return "OpenCV video-file plugin (VGCS decodes video with FFmpeg)"
    return None


def unused_qt_dlls(binaries):
    """Destinations of Qt6*.dll files that nothing left in the bundle loads.

    PyInstaller collects the DLLs of every QML module (3D, charts, PDF and so
    on). With the QML files gone, those DLLs are dead weight. A Qt DLL stays
    when another kept binary imports it, directly or through other DLLs. This
    reads the DLL import tables, so nothing is guessed.
    """
    import pefile

    wanted = [
        pefile.DIRECTORY_ENTRY["IMAGE_DIRECTORY_ENTRY_IMPORT"],
        pefile.DIRECTORY_ENTRY["IMAGE_DIRECTORY_ENTRY_DELAY_IMPORT"],
    ]

    def imports(path):
        pe = pefile.PE(path, fast_load=True)
        pe.parse_data_directories(directories=wanted)
        names = {
            entry.dll.decode("ascii", "replace").lower()
            for table in ("DIRECTORY_ENTRY_IMPORT", "DIRECTORY_ENTRY_DELAY_IMPORT")
            for entry in getattr(pe, table, [])
        }
        pe.close()
        return names

    def is_qt_dll(dest):
        d = dest.replace("\\", "/")
        name = d.rsplit("/", 1)[-1].lower()
        return d.count("/") == 1 and d.startswith("PySide6/") and name.startswith("qt6") and name.endswith(".dll")

    qt = {Path(dest).name.lower(): (dest, src) for dest, src, _ in binaries if is_qt_dll(dest)}
    todo = [src for dest, src, _ in binaries if not is_qt_dll(dest)]
    reached, seen = set(), set()
    while todo:
        src = todo.pop()
        if src in seen:
            continue
        seen.add(src)
        for name in imports(src):
            if name in qt and name not in reached:
                reached.add(name)
                todo.append(qt[name][1])
    return {dest for name, (dest, _src) in qt.items() if name not in reached}


def trim(binaries, datas):
    """Drop what VGCS never uses, and print what went and why."""
    dropped = {}
    kept = []
    for toc in (binaries, datas):
        keep = []
        for entry in toc:
            reason = unneeded_reason(entry[0])
            if reason:
                dropped.setdefault(reason, []).append(entry)
            else:
                keep.append(entry)
        kept.append(keep)
    binaries, datas = kept
    dead = unused_qt_dlls(binaries)
    dropped["Qt DLLs used only by QML modules"] = [e for e in binaries if e[0] in dead]
    binaries = [e for e in binaries if e[0] not in dead]
    for reason, entries in dropped.items():
        mb = sum(os.path.getsize(e[1]) for e in entries) / 1e6
        print(f"TRIM: {len(entries):4d} files, {mb:6.1f} MB  {reason}")
    return binaries, datas


rasterio_datas, rasterio_binaries = collect_delvewheel_libs_directory("rasterio")
ffmpeg_binaries, ffmpeg_datas = ffmpeg_files()

hiddenimports = [
    # Both vgcs worker modules are started by name ("-m ..."), never imported.
    # Collecting the whole package also covers any worker added later.
    *collect_submodules("vgcs"),
    # rasterio's compiled modules import each other in C, invisible to PyInstaller.
    *collect_submodules("rasterio"),
    # pymavlink imports its dialect by name at run time. MAVLink 1 is the
    # default until the first MAVLink 2 packet arrives, so both are needed.
    # A missing one makes pymavlink try to regenerate it, which fails in the exe.
    "pymavlink.dialects.v10.ardupilotmega",
    "pymavlink.dialects.v20.ardupilotmega",
]

datas = [
    *vgcs_data_files(),
    # gdal_data and proj_data: without proj.db, GeoTIFF DEMs lose their CRS.
    *collect_data_files("rasterio"),
    *rasterio_datas,
    *ffmpeg_datas,
    (build_stamp_file(), "."),
]

binaries = [*mgrs_library(), *rasterio_binaries, *ffmpeg_binaries]

a = Analysis(
    [str(PACKAGING / "vgcs_launcher.py")],
    pathex=[str(REPO), str(PACKAGING)],
    binaries=binaries,
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    # Not used by VGCS at run time. rasterio.plot imports matplotlib only if
    # installed; lxml is only for pymavlink's message generator.
    excludes=["tkinter", "matplotlib", "IPython", "pytest", "lxml"],
    noarchive=False,
    optimize=0,
)
a.binaries, a.datas = trim(a.binaries, a.datas)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    # A folder build. PyInstaller's own one-file mode unpacks everything at
    # every start (25 to 45 s here). build_exe.py's single file unpacks once.
    exclude_binaries=True,
    name="VGCS",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,  # UPX breaks some Qt and OpenCV DLLs and upsets antivirus scanners
    # Keep the console: VGCS and its worker processes log there, and field
    # reports are built from that log.
    console=True,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=os.environ.get("VGCS_ICON", "").strip() or None,
)
coll = COLLECT(exe, a.binaries, a.datas, strip=False, upx=False, upx_exclude=[], name="VGCS")
