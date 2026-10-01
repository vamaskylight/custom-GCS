# VGCS.exe: the Windows program

This folder builds VGCS as one single Windows program, `VGCS.exe`.
The client copies that one file and runs it.
Nothing needs installing: no Python, no FFmpeg, no Visual C++ runtime.
Running from source (`python -m vgcs`) does not change.

## Build it

Run this from the repository root, with the Python you use for VGCS:

```
py -3.14 packaging\build_exe.py
```

Result: `dist\VGCS.exe` (about 370 MB).
Send the client only that one file.
The `dist\VGCS` folder next to it is the folder build that is packed inside `VGCS.exe`.
It is not needed for delivery.

The first run creates `build\venv` (about 1 GB on disk) and downloads FFmpeg (115 MB) into `build\ffmpeg`.
Later runs reuse both.
Add `--skip-install` to skip the package check and save a minute.

The script:

1. installs the exact package versions from `constraints.txt` into `build\venv`,
2. downloads the pinned FFmpeg build and checks its SHA-256,
3. makes the exe icon from the VGCS logo,
4. runs PyInstaller with `vgcs.spec`, which makes the folder `dist\VGCS`,
5. packs that folder into the single `dist\VGCS.exe`,
6. runs `VGCS.exe --selfcheck` and stops if any check fails.

The first line VGCS prints names the git commit it was built from.
If the build had uncommitted changes, it says "with local changes".
Build releases from a clean, committed tree.

## What the client sees

- No console window. Only the VGCS window opens.
  FFmpeg and the tracker workers run without windows too.
- The first start of a new version takes about 20 to 30 seconds.
  `VGCS.exe` unpacks itself once, to `%LOCALAPPDATA%\VGCS\app\<version>` (about 820 MB).
  A small window shows the progress.
- Every later start takes about 3 to 7 seconds.
- Everything VGCS prints goes to a log file: `Documents\VGCS\logs\VGCS-<date>_<time>.log`.
  There is one file for each start, and the newest 30 are kept.
  When reporting a problem, send the log of that session.
- If VGCS stops with an error, a message box says so and names the log file.
- When a new `VGCS.exe` runs, the unpacked copies of older versions are deleted, unless a VGCS is still running from them.
- If a file of the unpacked copy goes missing (for example removed by an antivirus program), the next start unpacks again.
- To unpack somewhere else, for example off a small C: drive, set the environment variable `VGCS_UNPACK_DIR` to a folder.
  Keep its path short, such as `C:\VGCS`: Windows limits every file path to about 250 characters.
  If the folder is too deep, VGCS.exe says so before it writes anything.

The PC needs 64-bit Windows 10 (version 1809 or newer) or Windows 11, and about 1.2 GB of free disk space.

### Messages on the first start

- **"Windows protected your PC"** (SmartScreen): click "More info", then "Run anyway".
  This happens because the exe is not code-signed.
  A code-signing certificate removes this message and also reduces antivirus false alarms.
- **Windows Firewall** asks whether VGCS may use the network.
  Click "Allow" and tick both private and public networks.
  If this is cancelled, MAVLink over UDP will not arrive.

### Where things are kept

- The unpacked program: `%LOCALAPPDATA%\VGCS\app`.
- Logs: `Documents\VGCS\logs`.
  `VGCS-<date>_<time>.log` is the session log; `crash_trace.log` gets a trace if VGCS crashes hard.
  The environment variable `VGCS_LOG_DIR` moves the session logs elsewhere.
- Captures and reports: `Documents\VGCS`.
- Settings: the Windows registry (`HKEY_CURRENT_USER\Software\VGCS\VGCS`).
  These are the same settings as the source version on the same machine.
- Map tile cache: `%USERPROFILE%\.vgcs\tile-cache`, as before.

## FFmpeg

VGCS uses FFmpeg for live video and for recording, so `VGCS.exe` includes it.
The build downloads Gyan's FFmpeg 9.0.2 "essentials" build from its GitHub mirror.
The version, link and SHA-256 are at the top of `build_exe.py`.
Its `ffmpeg.exe` and `ffprobe.exe` are bundled.

Every FFmpeg option VGCS passes was checked against FFmpeg 9.0.2.
The self-check also records, probes and decodes a short clip with the bundled copy.
After changing the FFmpeg version, rebuild and make sure the self-check passes.

- `--ffmpeg-dir C:\path\to\bin` bundles another FFmpeg instead.
  It must be a GPL build, because recording uses `libx264`.
- `--no-ffmpeg` bundles none. VGCS then uses the FFmpeg on the machine's PATH.

License note: FFmpeg is GPL version 3.
Its `LICENSE`, its `README.txt` (which names the exact source commit) and `FFMPEG-SOURCE.txt` travel inside the build, in `_internal\ffmpeg`.
When you give FFmpeg to others, you must also make its source code available.
VGCS only runs FFmpeg as a separate program, so VGCS itself is not affected.

## What was left out

PyInstaller collects more than VGCS uses.
The spec removes it, and the build log lists it on its `TRIM:` lines:

- Qt QML files and the Qt DLLs only they need (VGCS has no QML screens),
- Qt translations (VGCS is in English; Qt WebEngine falls back to en-US),
- Qt WebEngine debug-build and developer-tools files,
- OpenCV's video-file plugin (VGCS decodes video with FFmpeg),
- `lxml` (only pymavlink's message generator uses it).

A Qt DLL is only removed when nothing left in the build imports it.
The spec reads the DLL import tables to decide, so nothing is guessed.
This saves about 260 MB and about 2,900 files.

## Self-check

```
VGCS.exe --selfcheck
```

It checks the parts that break when packaging goes wrong:
bundled files, MAVLink, serial ports, MGRS, GeoTIFF DEM (GDAL and PROJ), OpenCV,
the tracker and detector worker processes, FFmpeg, and the Qt WebEngine map page (with the network blocked).
It writes one line per check.
FAIL means the build is broken.
WARN means something is missing on that machine.
Run this way, a message box shows the result and names the log file with the details.
Ask the client to run it and send that log when VGCS misbehaves on their machine.
(The Windows "Run" box works: Windows+R, then the full path of VGCS.exe followed by `--selfcheck`.)

When a script reads the output, for example `VGCS.exe --selfcheck > result.txt`,
VGCS.exe writes it there instead, and shows no message box.
The build script and the tests rely on this.

`py packaging\vgcs_selfcheck.py` runs the same checks from source.
If a check fails in the exe but passes from source, the problem is in the packaging.

## The folder build

```
py -3.14 packaging\build_exe.py --folder
```

This skips step 5 and delivers `dist\VGCS\` (VGCS.exe plus `_internal`) and a zip of it.
It starts in about 2 seconds every time, including the first.
Its `VGCS.exe` opens a console window with the live log, which is handy while developing.
Use it where a folder is acceptable, or to test a build quickly.

## How it works (for maintainers)

Files:

- `vgcs.spec`: what goes into the build, and what is trimmed.
- `vgcs_launcher.py`: the entry point inside the build.
- `single_file_launcher.cs`: the single `VGCS.exe`, which unpacks the folder build.
- `vgcs_selfcheck.py`: the checks above.
- `build_exe.py`: the build steps.
- `constraints.txt`: exact package versions.
- `requirements-build.txt`: PyInstaller and Pillow.

`vgcs_launcher.py` handles three things that differ inside the exe:

1. `sys.executable` is `VGCS.exe`, not `python.exe`.
   The tracker and detector start their workers with `sys.executable -m <module>`.
   The launcher runs that module instead of opening a second VGCS window.
   Any `vgcs.*` module works, so a new worker needs no change here.
   It does need a check in `vgcs_selfcheck.py`.
2. FFmpeg is found on PATH.
   The launcher puts the bundled FFmpeg first on PATH.
3. VGCS writes under the current folder.
   The launcher changes to `Documents\VGCS` first.

It also switches stdout and stderr to UTF-8, a line at a time, when they are pipes.
In the single exe they are, because the output goes into the log file.
Otherwise Python would use cp1252 there, and a print with a character outside it
(`vgcs/video/pipeline.py` prints an arrow) would raise inside VGCS.

The spec handles what PyInstaller cannot find by itself:

- Data files: every non-Python file under `vgcs/` that git tracks.
  Gitignored files are internal and are not shipped, for example `MODEL_SOURCE.md`.
  **A new asset must be committed to be bundled.**
  The build prints a warning for untracked files.
- `mgrs` loads its native library with ctypes, so the spec adds it by hand.
- `rasterio` needs its GDAL and PROJ data and its DLL folder.
- `pymavlink` imports its dialect by name at run time.
  Both the MAVLink 1 and MAVLink 2 `ardupilotmega` dialects are listed.

`single_file_launcher.cs` is compiled with the C# compiler of the .NET Framework,
which is part of Windows, so it needs nothing installed to build or to run.
It supports C# 5 only.
It carries the folder build as a zip resource.
The version folder name includes a hash of that zip, so every different build unpacks fresh.
A lock file in each version folder, plus a check for running `VGCS.exe` processes,
keeps it from deleting a version that is still in use.

It is a windowed program, so it has no console window.
It starts the unpacked VGCS (a console program) with a hidden console.
FFmpeg and the worker processes share that hidden console, so none of them shows a window,
and no code in VGCS had to change for that.
It reads VGCS's output and writes it to the session log.
When its own output is a pipe or a file, it passes VGCS's output there instead,
and it never shows a window or a message box.

`tests/test_single_file_launcher.py` tests these rules with a tiny fake program.

Why not PyInstaller's own one-file mode:
it unpacks everything to `%TEMP%` at every start, and antivirus scans the unpacked files again each time.
Measured here, every start took 24 to 44 seconds.

To upgrade a package, change its pin in `constraints.txt`, rebuild, and check the self-check passes.
