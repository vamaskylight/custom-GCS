# VGCS (Custom Ground Control System)



**VGCS** is a desktop Ground Control Station for **ArduPilot** vehicles. It is built with **Python**, **PySide6** (Qt 6), and **pymavlink** (MAVLink over serial, UDP, TCP, etc.).



| | |

|---|---|

| **Package version** | `0.1.0` (see `vgcs/__init__.py`) |

| **Python** | 3.10+ recommended |

| **Current milestone** | **M1** — telemetry dashboard, link watchdog, GCS-style UI shell |



---



## Table of contents



1. [What this repository contains](#what-this-repository-contains)

2. [Features (today)](#features-today)

3. [Requirements](#requirements)

4. [Clone and first-time setup](#clone-and-first-time-setup)

5. [Setup — Windows](#setup--windows)

6. [Setup — Linux](#setup--linux)

7. [Run the application](#run-the-application)

8. [Connect to ArduPilot SITL](#connect-to-arduopilot-sitl)

9. [Project layout](#project-layout)

10. [Architecture note](#architecture-note)

11. [Troubleshooting](#troubleshooting)

12. [For contributors](#for-contributors)



---



## What this repository contains



- **`vgcs/`** — the active application: Qt main window, MAVLink worker thread, entrypoint (`python -m vgcs`).

- **`Ground-Control-Station-for-UAV/`** — **legacy / reference-only** material; not part of the new VGCS codebase and is **gitignored** for normal work.



---



## Features (today)



At **M1**, VGCS provides a **dark GCS-style dashboard**: link settings, status chips, grouped flight/system telemetry, **compass** + heading, configurable link watchdog, and message log. **Map, mission editing, and video** are planned for **M2 / M3**.



| Capability | Description |

|------------|-------------|

| **Connection** | pymavlink-style string, watchdog timeout, theme presets; settings persist locally. |

| **Connect / Disconnect** | Background MAVLink thread; UI stays responsive. |

| **Telemetry** | Core flight and navigation fields from the agreed M1 telemetry set. |

| **Compass** | Heading needle (VFR_HUD / attitude yaw). |

| **Log** | Connection and telemetry log. |

**Next milestone planning:** aligned to M2 dashboard spec and implementation checklist.



---



## Requirements



### Software



| Item | Notes |

|------|--------|

| **Python** | **3.10 or newer** (64-bit recommended on Windows) |

| **pip** | Usually included; upgrade if needed: `python -m pip install --upgrade pip` |

| **OS** | Windows 10/11 or a recent Linux desktop (Ubuntu 22.04+, Fedora, etc.) |

| **ArduPilot SITL** | Optional for development, but **required** to complete the M0 “first link” test; see [ArduPilot SITL docs](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html) |



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



---

## Dev Fast Loop

For near-immediate UI iteration while coding:

```bash
python tools/dev_autorestart.py
```

This watches `vgcs/**/*.py` and restarts the app automatically on save.

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


