# VGCS (Custom Ground Control System)



**VGCS** is a desktop Ground Control Station for **ArduPilot** vehicles. It is built with **Python**, **PySide6** (Qt 6), and **pymavlink** (MAVLink over serial, UDP, TCP, etc.).



| | |

|---|---|

| **Package version** | `0.1.0` (see `vgcs/__init__.py`) |

| **Python** | 3.10+ recommended |

| **Current milestone** | **M2** — map-first operator dashboard, plan tools, mission + geofence ops |



---



## Table of contents



1. [What this repository contains](#what-this-repository-contains)

2. [Features (today)](#features-today)

3. [Several drones (fleet)](#several-drones-fleet)

4. [Command signing (MAVLink 2)](#command-signing-mavlink-2)

5. [Requirements](#requirements)

6. [Clone and first-time setup](#clone-and-first-time-setup)

7. [Setup — Windows](#setup--windows)

8. [Setup — Linux](#setup--linux)

9. [Run the application](#run-the-application)

10. [Connect to ArduPilot SITL](#connect-to-arduopilot-sitl)

11. [Project layout](#project-layout)

12. [Architecture note](#architecture-note)

13. [Troubleshooting](#troubleshooting)

14. [For contributors](#for-contributors)



---



## What this repository contains



This repository holds two products:

- **VGCS** (`VGCS.exe`): the Windows ground station with DOOAF, written in Python.
- **VAMA APK**: an Android ground station built on QGroundControl (C++), without DOOAF.

They share no code. VGCS stays at the repository root, so `python -m vgcs` works as before.



- **`vgcs/`** — the active application: Qt main window, MAVLink worker thread, entrypoint (`python -m vgcs`).

- **`packaging/`**: builds the standalone Windows program `VGCS.exe` (see [packaging/README.md](packaging/README.md)).

- **`apk/`**: the VAMA APK, an Android app built on QGroundControl for customers who do not need DOOAF (see [apk/README.md](apk/README.md)).

- **`drone/`**: what runs on the drone itself, such as the high wind failsafe script for the flight controller (see [drone/README.md](drone/README.md)).

- **`Ground-Control-Station-for-UAV/`** — **legacy / reference-only** material; not part of the new VGCS codebase and is **gitignored** for normal work.



---



## Features (today)



At **M2**, VGCS provides a **map-first GCS dashboard** with live telemetry overlays, operator actions, mission planning/editing, geofence controls, and MAVLink command workflows (mode, takeoff/land, params). Video controls are integrated in the map UI with a preview overlay.



| Capability | Description |

|------------|-------------|

| **Connection** | pymavlink-style string, watchdog timeout, theme presets; settings persist locally. |

| **Connect / Disconnect** | Background MAVLink thread; UI stays responsive. |

| **Telemetry** | Core flight and navigation fields from the agreed M1 telemetry set. |

| **Compass** | Heading needle (VFR_HUD / attitude yaw). |

| **Log** | Connection and telemetry log. |

**Milestone status:** M2 implementation is complete.



---



## Several drones (fleet)

VGCS can connect several drones at once (milestone M15, client requirement 14).

- Each drone needs its own connection, for example its own UDP port (`udpin:0.0.0.0:14551`, `udpin:0.0.0.0:14552`) or its own address. Two drones sharing one radio link (one port, two system ids) are not supported yet.
- Connect the first drone with **Connect**. Add the others in the logo menu: **Fleet (several drones)**, then **Connect drone**.
- The window shows one drone in full: the drone on screen. Its buttons, the plan upload and the settings act on that drone only.
- **Show and command it** in the Fleet panel puts another drone on screen. The other links stay open, so switching never drops a drone.
- The map shows the other drones as smaller blue arrows with their name and height (grey while their link is lost). The window title names the drone on screen and the size of the fleet.
- The Fleet panel lists every drone: link, mode, armed, height, battery, GPS, mission progress and last message.
- A warning from a drone that is not on screen (link lost, battery, fence, EKF) shows in the message line and the log, with the drone's name.

Which drone gets a command:

| Command | Goes to |
|---------|---------|
| Plan and upload a mission | The drone on screen only |
| Take-off, mode change, land, arm, fence, settings | The drone on screen only |
| Hold, Return home, Land in the Fleet panel | The drone selected in the panel |
| Hold all, Return home all, Land all | Every connected drone |

VGCS never sends one mission to several drones: the same route would fly them into each other.

Tested with three ArduCopter simulators at once, on 4.6.2 and 4.7.0 (`system_test/vgcs_fleet_test.py`).

Not yet: the 3D view shows only the drone on screen, and the video and camera follow the camera settings, not the drone on screen.

---

## Command signing (MAVLink 2)

VGCS can sign every command it sends (milestone M16, client requirement 29). A drone that holds the same key ignores commands from any ground station without it.

1. Application Settings, General, **Command signing**: **Set passphrase...** (at least 8 characters, longer is safer).
2. Connect the drone, disarmed, and press **Send key to the drone**. VGCS reports when the drone signs with the key.
3. Menu, **Vehicle status**: **Command signing** shows what the drone on screen does. The Fleet panel shows it for every drone.

- The same passphrase works in the VAMA APK (QGroundControl's MAVLink signing keys): VGCS makes the key from it the same way.
- The passphrase is never stored. The key is, encrypted for the Windows user (DPAPI).
- A lost passphrase is not a lost drone: ArduPilot always accepts a new key over USB.
- Signing proves who sent a command. It does not encrypt: telemetry and video can still be received.

What is and is not protected: `DOCS/M16-SECURITY.md`. Tested against ArduCopter 4.6.2 and 4.7.0 in the simulator (`system_test/vgcs_sitl_test.py`, case `signed_commands`).

---

## Requirements



### Software



| Item | Notes |

|------|--------|

| **Python** | **3.10 or newer** (64-bit recommended on Windows) |

| **pip** | Usually included; upgrade if needed: `python -m pip install --upgrade pip` |

| **OS** | Windows 10/11 or a recent Linux desktop (Ubuntu 22.04+, Fedora, etc.) |

| **ArduPilot SITL** | Optional for development, but **required** to complete the M0 “first link” test; see [ArduPilot SITL docs](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html) |



### FFmpeg (for the camera video)

VGCS shows the camera video and records it with FFmpeg, a separate program.
`VGCS.exe` has FFmpeg inside. A run from source (`python -m vgcs`) uses the FFmpeg of the PC.

- VGCS looks for `ffmpeg` on `PATH`, then in `build\ffmpeg` of this folder, in a `VGCS.exe` that ran on this PC, and in the usual install places.
- Without FFmpeg the video area says "No video: FFmpeg is not installed on this PC", and the console says how to get it.
- To get it, use one of these:
  - `winget install Gyan.FFmpeg`, then open a new terminal.
  - `py packaging\build_exe.py --ffmpeg-only`: downloads the FFmpeg that VGCS is tested with (9.0.2, 115 MB) into `build\ffmpeg`.
  - Set `VGCS_FFMPEG_DIR` to the folder that holds `ffmpeg.exe`.

The window title names what is running: the build of the exe (`build e3ee0d1 (date)`), or `source` and the commit for a run from source.



### Hardware (for this project)



You do **not** need special drone hardware on your desk to **write code** or to **test against SITL**. A normal laptop or desktop is enough.



| Use case | What you need |

|----------|----------------|

| **Coding + running VGCS** | **64-bit** PC, **8 GB RAM** minimum (**16 GB** more comfortable), a few **GB free disk** for Python, venv, and repos. **No dedicated GPU** required for the basic UI. |

| **SITL on the same machine** | Same as above; SITL uses **CPU** (multiple cores help). Close heavy apps if the machine feels slow. |

| **Real vehicle later** | Not part of M0: flight controller with ArduPilot, telemetry link, etc. The GCS side remains a normal PC over USB or network. |



**Bottom line:** If the PC runs Windows or Linux smoothly for everyday development, it is usually sufficient for VGCS + SITL.



### Python packages (`requirements.txt`)



| Package | Purpose |

|---------|---------|

| **pymavlink** | MAVLink decode/encode and transport (`mavutil.mavlink_connection`, …) |

| **PySide6** | Qt 6 bindings for the desktop UI |



Constraints in `requirements.txt`:



```text

pymavlink>=2.4.40,<3

PySide6>=6.6.0,<7

```



---



## Clone and first-time setup



1. **Clone** the repository (HTTPS or SSH — use your team’s URL):



   ```bash

   git clone <your-repo-url>

   cd GCS

   ```



2. **Create a virtual environment** (always recommended so system Python stays clean).



3. **Install dependencies**:



   ```bash

   pip install -r requirements.txt

   ```



4. **Run** (see [Run the application](#run-the-application)).



Do **not** commit `.venv/` — it stays local.



---



## Setup — Windows



Use **Command Prompt** or **PowerShell** from the repo root (example path `C:\dev\GCS`):



```bat

cd C:\dev\GCS

python -m venv .venv

.venv\Scripts\activate

python -m pip install --upgrade pip

pip install -r requirements.txt

```



- If `python` is not found, try `py -3` (Python launcher) or install Python from [python.org](https://www.python.org/downloads/) and tick **“Add Python to PATH”**.



Deactivate when done:



```bat

deactivate

```



---



## Setup — Linux



From a terminal in the repo root:



```bash

cd /path/to/GCS

python3 -m venv .venv

source .venv/bin/activate

python -m pip install --upgrade pip

pip install -r requirements.txt

```



If the GUI fails with **Qt / XCB / platform plugin** errors, install base graphics libraries. On **Debian/Ubuntu** this often resolves it:



```bash

sudo apt update

sudo apt install -y libxcb-xinerama0 libxcb-cursor0 libxkbcommon-x11-0 libegl1

```



Deactivate:



```bash

deactivate

```



---



## Run the application



With the virtual environment **activated** and the working directory at the **repository root**:



```bash

python -m vgcs

```

*(Typo: the module name is **`vgcs`**, not `vcgs`.)*

You should see the **VGCS** window: connection settings, status chips, telemetry panels, compass, and log.



**Sanity checks:**



- Window opens without Python tracebacks.

- **Disconnect** is disabled until you connect (depending on state); after a successful connect cycle, you can disconnect cleanly.

To build a standalone Windows program (`VGCS.exe`) that needs no Python, see [packaging/README.md](packaging/README.md).



---

## Dev Fast Loop

For near-immediate UI iteration while coding:

```bash
py dev.py
```

This watches `vgcs/**/*.py` and restarts the app automatically on save.

Equivalent (older) command:

```bash
py tools/dev_autorestart.py
```

Inside the running app, you can also press:

```text
Ctrl+Shift+R
```

to re-apply fonts/styles without a full process restart.

---



## Connect to ArduPilot SITL



1. **Start SITL first** (your team’s vehicle type and options). Example (paths vary by install):



   ```bash

   sim_vehicle.py -v ArduCopter --console --map

   ```



2. **Start VGCS** (`python -m vgcs`).



3. In the UI, set the **MAVLink connection string** to match SITL. Many setups use:



   ```text

   udp:127.0.0.1:14550

   ```



4. Click **Connect**. The log should show an open socket and **HEARTBEAT** messages; the status line should show system/component IDs.



5. Stop SITL or click **Disconnect** — the UI should return to a disconnected state.



**Alternate connection styles** (when defaults do not match your setup):



- `tcp:127.0.0.1:5760` — common for some SITL serial bridges.

- `udpin:0.0.0.0:14550` — listen mode when the vehicle connects *to* the GCS.



Confirm the port and direction in your SITL console (“bind” / output lines) and adjust the string accordingly. More background: [ArduPilot SITL documentation](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html).



---



## Project layout



```text

GCS/                          # repository root

  requirements.txt            # Python dependencies

  README.md                   # this file

  packaging/                  # VGCS.exe build (see packaging/README.md)

  apk/                        # VAMA APK, QGroundControl custom build (see apk/README.md)

  drone/                      # scripts for the flight controller, with simulator tests (see drone/README.md)

  vgcs/

    __init__.py               # package version

    __main__.py               # enables: python -m vgcs

    main.py                   # QApplication + MainWindow

    app/

      main_window.py          # Qt UI: connection string, buttons, log

    link/

      mavlink_thread.py       # QThread + pymavlink HEARTBEAT loop

```



---



## Architecture note



- The **Qt GUI** runs on the main thread (`QApplication`).

- **pymavlink** I/O runs in a **`QThread`** (`MavlinkThread`) so the window stays responsive during blocking `recv` calls.

- Signals/slots bridge **HEARTBEAT** and log lines back to the UI safely.



---



## Troubleshooting



| Problem | What to try |

|---------|-------------|

| `ModuleNotFoundError: PySide6` / `pymavlink` | Activate `.venv` and run `pip install -r requirements.txt` again from repo root. |

| `python` not found (Windows) | Use `py -3` or reinstall Python with PATH enabled. |

| No camera video, the video area says FFmpeg is not installed | Install FFmpeg, see [FFmpeg (for the camera video)](#ffmpeg-for-the-camera-video). |

| Linux: Qt/XCB errors | Install the `apt` packages listed in [Setup — Linux](#setup--linux). |

| No HEARTBEAT after Connect | SITL not running; wrong port or protocol; firewall blocking UDP; typo in connection string. Compare with SITL console output. |

| Wrong port vs SITL | Check SITL console for bind / output lines and match `udp:` / `tcp:` / `udpin:` accordingly. |



---



## For contributors



### Branches



| Pattern | Use for |

|---------|---------|

| **`main`** | Stable, review-ready code. Protect it if your host allows (no direct pushes, PRs only). |

| **`feature/<short-name>`** | New capability (e.g. `feature/telemetry-panel`). |

| **`fix/<short-name>`** | Bugfixes (e.g. `fix/reconnect-race`). |



Use **lowercase**, **hyphens** for words, and keep names **short and descriptive**. Avoid long-lived personal branches; merge or rebase often.



### Pull requests



1. **One PR per logical change** — easier review and safer rollback.

2. **Describe what and why** — a few sentences in the PR body; link an issue if you use an issue tracker.

3. **Run the app** — `python -m vgcs` starts; connect to SITL if your change touches the link or UI.

4. **Keep diffs focused** — match existing style; do not reformat unrelated files.

5. **Respond to review** — push follow-up commits or comments until reviewers are satisfied.



Exact rules (required reviewers, CI) depend on your Git host — align with the team lead.


