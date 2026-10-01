r"""Build VGCS.exe. See packaging/README.md.

Run it with the Python you use for VGCS (3.14):

    py -3.14 packaging/build_exe.py            one single file: dist\VGCS.exe
    py -3.14 packaging/build_exe.py --folder   a folder: dist\VGCS\VGCS.exe, plus a zip

Steps:
  1. Create build/venv on the first run, then install the pinned packages.
  2. Download the pinned FFmpeg build on the first run, and check its SHA-256.
  3. Make the exe icon from the VGCS logo.
  4. Run PyInstaller with packaging/vgcs.spec. This makes the folder dist/VGCS.
  5. Single file: pack that folder into one VGCS.exe (single_file_launcher.cs).
     Folder: zip dist/VGCS for delivery.
  6. Run "VGCS.exe --selfcheck" on the result. Stop if any check fails.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import re
import shutil
import subprocess
import sys
import urllib.request
import zipfile
from pathlib import Path

TESTED_PYTHON = "3.14"

# FFmpeg bundled into VGCS.exe: Gyan's "essentials" build (GPL v3, has libx264).
# The GitHub mirror keeps every release, so this link keeps working.
# To change version: update all three lines, rebuild, and run the self-check.
FFMPEG_VERSION = "9.0.2"
FFMPEG_URL = (
    f"https://github.com/GyanD/codexffmpeg/releases/download/{FFMPEG_VERSION}/"
    f"ffmpeg-{FFMPEG_VERSION}-essentials_build.zip"
)
FFMPEG_SHA256 = "60f467265b1e312373dbcd92200c2618a74850f98d3d078e94296bb3fa2047ba"
FFMPEG_KEEP = ("ffmpeg.exe", "ffprobe.exe", "LICENSE", "README.txt")

REPO = Path(__file__).resolve().parents[1]
PACKAGING = REPO / "packaging"
BUILD = REPO / "build"
DIST = REPO / "dist"
APP_DIR = DIST / "VGCS"
VENV_PYTHON = BUILD / "venv" / "Scripts" / "python.exe"
# The C# compiler of the .NET Framework, which is part of Windows 10 and 11.
CSC = Path(os.environ.get("WINDIR", r"C:\Windows")) / "Microsoft.NET" / "Framework64" / "v4.0.30319" / "csc.exe"

# Runs in the build venv, which has Pillow. The logo is wide, so it is
# trimmed and centred on a square canvas before it is scaled to icon sizes.
ICON_SCRIPT = r"""
import sys
from PIL import Image

Image.MAX_IMAGE_PIXELS = None  # the logo is 15171 x 6793 pixels
source, target = sys.argv[1], sys.argv[2]
logo = Image.open(source).convert("RGBA")
logo = logo.crop(logo.getbbox())
side = max(logo.size)
square = Image.new("RGBA", (side, side), (0, 0, 0, 0))
square.paste(logo, ((side - logo.width) // 2, (side - logo.height) // 2))
square = square.resize((256, 256), Image.Resampling.LANCZOS)
square.save(target, sizes=[(16, 16), (24, 24), (32, 32), (48, 48), (64, 64), (128, 128), (256, 256)])
"""


def run(cmd: list, **kwargs) -> None:
    print("\n>", " ".join(str(c) for c in cmd), flush=True)
    subprocess.run([str(c) for c in cmd], check=True, **kwargs)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def prepare_venv(install: bool) -> None:
    if not VENV_PYTHON.is_file():
        run([sys.executable, "-m", "venv", BUILD / "venv"])
    if install:
        run([VENV_PYTHON, "-m", "pip", "install", "--upgrade", "pip"])
        run([
            VENV_PYTHON, "-m", "pip", "install",
            "-r", REPO / "requirements.txt",
            "-r", PACKAGING / "requirements-build.txt",
            "-c", PACKAGING / "constraints.txt",
        ])
    version = subprocess.run(
        [str(VENV_PYTHON), "-c", "import sys; print('%d.%d' % sys.version_info[:2])"],
        capture_output=True, text=True, check=True,
    ).stdout.strip()
    if version != TESTED_PYTHON:
        print(f"\nWARNING: the build venv uses Python {version}, but VGCS is tested on {TESTED_PYTHON}.")
        print(f"Delete {BUILD / 'venv'} and run this script with py -{TESTED_PYTHON}.")


def prepare_ffmpeg() -> Path:
    """The pinned FFmpeg, downloaded and checked once, then kept in build/ffmpeg."""
    folder = BUILD / "ffmpeg" / f"ffmpeg-{FFMPEG_VERSION}"
    if all((folder / name).is_file() for name in FFMPEG_KEEP):
        return folder
    archive = BUILD / "ffmpeg" / FFMPEG_URL.rsplit("/", 1)[1]
    if not archive.is_file() or sha256(archive) != FFMPEG_SHA256:
        archive.parent.mkdir(parents=True, exist_ok=True)
        print(f"\n> download {FFMPEG_URL} (about 115 MB)", flush=True)
        partial = archive.with_suffix(".part")
        urllib.request.urlretrieve(FFMPEG_URL, partial)
        partial.replace(archive)
    actual = sha256(archive)
    if actual != FFMPEG_SHA256:
        archive.unlink()
        sys.exit(f"The FFmpeg download has SHA-256 {actual}, expected {FFMPEG_SHA256}. Not using it.")
    folder.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(archive) as z:
        for info in z.infolist():
            name = info.filename.rsplit("/", 1)[-1]
            if name in FFMPEG_KEEP:
                (folder / name).write_bytes(z.read(info))
    (folder / "FFMPEG-SOURCE.txt").write_text(
        f"FFmpeg {FFMPEG_VERSION}, essentials build by Gyan Doshi (www.gyan.dev).\n"
        f"Downloaded from {FFMPEG_URL}\n"
        f"SHA-256 of that zip: {FFMPEG_SHA256}\n"
        "License: GNU GPL version 3, see LICENSE.\n"
        "The exact FFmpeg source commit is named in README.txt.\n"
        "VGCS runs FFmpeg as a separate program.\n",
        encoding="utf-8",
    )
    return folder


def make_icon() -> Path:
    icon = BUILD / "vgcs.ico"
    run([VENV_PYTHON, "-c", ICON_SCRIPT, REPO / "vgcs" / "assets" / "Vama Logo.png", icon])
    return icon


def run_pyinstaller(icon: Path, ffmpeg_dir: Path | None) -> None:
    env = dict(os.environ, VGCS_ICON=str(icon))
    env.pop("VGCS_FFMPEG_DIR", None)
    if ffmpeg_dir is not None:
        env["VGCS_FFMPEG_DIR"] = str(ffmpeg_dir)
    run(
        [
            VENV_PYTHON, "-m", "PyInstaller", "--noconfirm", "--clean",
            "--distpath", DIST, "--workpath", BUILD / "pyinstaller",
            PACKAGING / "vgcs.spec",
        ],
        env=env,
        cwd=REPO,
    )


def read_stamp() -> tuple[str, str, bool]:
    """(version, commit, has local changes) from the build stamp vgcs.spec wrote."""
    stamp = (APP_DIR / "_internal" / "vgcs_build.txt").read_text(encoding="utf-8")
    m = re.match(r"VGCS (\S+) build (\S+)( with local changes)?", stamp)
    if m is None:
        sys.exit(f"Unexpected build stamp: {stamp!r}")
    return m.group(1), m.group(2), bool(m.group(3))


def make_single_file(icon: Path) -> Path:
    """Pack dist/VGCS into one dist/VGCS.exe that unpacks itself once per version."""
    if not CSC.is_file():
        sys.exit(f"The C# compiler of the .NET Framework is missing: {CSC}")
    work = BUILD / "single"
    shutil.rmtree(work, ignore_errors=True)
    work.mkdir(parents=True)
    print(f"\n> zip {APP_DIR} into the single exe's program data", flush=True)
    payload = Path(shutil.make_archive(str(work / "payload"), "zip", root_dir=APP_DIR))
    version, commit, changed = read_stamp()
    # Names the unpacked folder. The hash makes every different build unpack
    # fresh. Kept short: Windows limits the full path of every unpacked file.
    ident = f"{commit}-{sha256(payload)[:8]}"
    description = f"{version} build {commit}" + (" with local changes" if changed else "")
    numbers = ".".join((re.findall(r"\d+", version) + ["0"] * 4)[:4])
    (work / "BuildInfo.cs").write_text(
        "using System.Reflection;\n"
        '[assembly: AssemblyTitle("VGCS")]\n'
        '[assembly: AssemblyProduct("VGCS Ground Control Station")]\n'
        f'[assembly: AssemblyInformationalVersion("{description}")]\n'
        f'[assembly: AssemblyFileVersion("{numbers}")]\n'
        f'[assembly: AssemblyVersion("{numbers}")]\n'
        f'static class BuildInfo {{ public const string Id = "{ident}"; }}\n',
        encoding="utf-8",
    )
    shutil.copy2(PACKAGING / "single_file_launcher.cs", work)
    shutil.copy2(icon, work / "vgcs.ico")
    # Run in build/single with plain file names: the compiler then never sees a path with spaces.
    run(
        [
            CSC, "/nologo", "/target:exe", "/platform:x64", "/optimize+",
            "/out:VGCS.exe", "/win32icon:vgcs.ico", "/resource:payload.zip,VGCS.payload.zip",
            "/reference:System.IO.Compression.dll",
            "/reference:System.IO.Compression.FileSystem.dll",
            "/reference:System.Windows.Forms.dll",
            "single_file_launcher.cs", "BuildInfo.cs",
        ],
        cwd=work,
    )
    exe = DIST / "VGCS.exe"
    shutil.move(str(work / "VGCS.exe"), exe)
    payload.unlink()  # it is inside VGCS.exe now
    return exe


def make_zip() -> Path:
    version, commit, changed = read_stamp()
    name = f"VGCS-{version}-{commit}" + ("-modified" if changed else "")
    print(f"\n> zip {APP_DIR} -> {DIST / (name + '.zip')}", flush=True)
    return Path(shutil.make_archive(str(DIST / name), "zip", root_dir=DIST, base_dir=APP_DIR.name))


def run_selfcheck(exe: Path) -> None:
    print(f"\n> {exe} --selfcheck", flush=True)
    result = subprocess.run([str(exe), "--selfcheck"], cwd=exe.parent)
    if result.returncode != 0:
        sys.exit(f"\nSelf-check failed ({result.returncode} check(s)). This build is not ready to ship.")


def main() -> int:
    parser = argparse.ArgumentParser(description="Build VGCS.exe (see packaging/README.md).")
    parser.add_argument(
        "--folder", action="store_true",
        help="deliver the folder (dist\\VGCS) and a zip of it, instead of one single VGCS.exe",
    )
    ffmpeg = parser.add_mutually_exclusive_group()
    ffmpeg.add_argument(
        "--ffmpeg-dir", type=Path,
        help="bundle the ffmpeg.exe and ffprobe.exe from this folder instead of the pinned download",
    )
    ffmpeg.add_argument("--no-ffmpeg", action="store_true", help="do not bundle FFmpeg")
    parser.add_argument("--skip-install", action="store_true", help="do not run pip (faster rebuilds)")
    parser.add_argument("--no-zip", action="store_true", help="with --folder: do not zip dist/VGCS")
    args = parser.parse_args()
    if os.name != "nt":
        sys.exit("VGCS.exe can only be built on Windows.")

    prepare_venv(install=not args.skip_install)
    ffmpeg_dir = None if args.no_ffmpeg else (args.ffmpeg_dir or prepare_ffmpeg())
    icon = make_icon()
    run_pyinstaller(icon, ffmpeg_dir)
    if args.folder:
        run_selfcheck(APP_DIR / "VGCS.exe")
        print(f"\nBuilt {APP_DIR / 'VGCS.exe'}")
        if not args.no_zip:
            archive = make_zip()
            print(f"Zip to deliver: {archive} ({archive.stat().st_size / 1e6:.0f} MB)")
        return 0
    exe = make_single_file(icon)
    run_selfcheck(exe)
    print(f"\nBuilt {exe}")
    print(f"Deliver this one file: {exe} ({exe.stat().st_size / 1e6:.0f} MB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
