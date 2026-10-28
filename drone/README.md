# Drone side

This folder holds what runs on the drone itself, not in a ground station.

VGCS (the Windows exe) is in `vgcs/`, and the VAMA APK is in `apk/`.
What is in this folder works with both of them, and with the link down.

## Layout

```text
drone/
  README.md                       this file
  scripts/
    vama_wind_failsafe.lua        the high wind failsafe (copy it to the SD card)
    vama_wind_failsafe.md         what it does, how to install it, its parameters and messages
    vama_lock.lua                 the VAMA lock, part 1 of the firmware lock (copy it to the SD card)
    vama_lock.md                  what it does and does not stop, how to bind a drone, parameters, messages
  gps_denied/                     GPS-denied navigation (item A8); step 1, the flight recorder for a
                                  Raspberry Pi on the drone (see gps_denied/README.md, RECORDER.md)
  test/
    setup_sitl.sh                 downloads the ArduCopter simulator programs (run once, inside WSL)
    sitl_session.py               starts a simulated drone and talks MAVLink to it
    wind_failsafe_test.py         flies the wind failsafe in the simulator
    lock_test.py                  tries the VAMA lock in the simulator
    recorder_sitl_test.py         flies the flight recorder against the simulator
```

## High wind failsafe

Read [scripts/vama_wind_failsafe.md](scripts/vama_wind_failsafe.md).

Short version:

1. Set `SCR_ENABLE` to 1 on the flight controller and reboot.
2. Copy `scripts/vama_wind_failsafe.lua` to the SD card folder `APM/scripts/` and reboot.
3. Fly the first flights with `WFS_ACTION` 1 (warning only). Then set it to 2 (RTL).

## VAMA lock

Read [scripts/vama_lock.md](scripts/vama_lock.md).

Short version: once the team's MAVLink signing key is on the drone and `VLCK_ENABLE` is 1, the drone arms only while VGCS or the VAMA app is connected to it over the radio with that key.
It never acts in flight.
It does not stop someone who opens the drone (USB, SD card, flashing): that is part 2 of the firmware lock.

## Simulator tests

The scripts are tested in ArduPilot's own simulator (SITL), with the real ArduCopter 4.6.2 and 4.7.0 programs.
Nothing is sent over the network: the simulator runs in WSL and the test talks to it on this computer.

What you need:

- Windows with WSL and the `Ubuntu-24.04` distribution.
- Python on Windows with `pymavlink` (`py -m pip install pymavlink`).

Set up once (downloads about 14 MB from firmware.ardupilot.org and github.com):

```powershell
wsl -d Ubuntu-24.04 -- bash "/mnt/e/My project/GCS/drone/test/setup_sitl.sh"
```

Change the path if the repo is in another folder.

Run:

```powershell
cd drone\test
py wind_failsafe_test.py               # all cases on both versions, about 10 minutes
py wind_failsafe_test.py 4.7.0         # one version
py wind_failsafe_test.py -k blown      # only the cases with "blown" in their name
py lock_test.py                        # the VAMA lock: all cases on both versions, about 3 minutes
```

Each case flies one situation and prints `PASS` or `FAIL` with the reason.
The cases of the VAMA lock are listed in [scripts/vama_lock.md](scripts/vama_lock.md).
The wind failsafe's are below.
The simulated clock runs 10 times faster than real time.

| Case | What it proves |
|------|----------------|
| `calm_flight` | No wind: nothing happens, also on fast legs and hard stops |
| `strong_wind_that_it_can_hold` | 9 m/s: a warning, and no action on legs with, against and across the wind |
| `pilot_drifts_with_the_wind` | In LOITER the pilot may let the drone go with the wind |
| `blown_away_home_upwind` | 14 m/s: RTL in 7 s with motors at 58 %, then LAND because RTL is pushed away too. Once per flight. |
| `blown_away_home_downwind` | Home downwind: RTL is left alone while it gets closer |
| `motors_at_their_limit` | Motors at 92 %: warning, then RTL. The next flight can act again. |
| `default_is_warning_only` | As installed, the flight mode never changes |
| `action_land`, `action_off` | The other `WFS_ACTION` values |
| `no_gps_means_land` | RTL refused without a position, so LAND |
| `pilot_modes_are_left_alone` | ALT_HOLD: no action |
| `modes_without_a_hold_point` | GUIDED with speed commands, and POSHOLD |
| `slow_stop_with_the_wind_behind` | A long stop after a fast leg is a stop, not a blow-away |
| `small_lean_limit` | A drone that may lean only 15 degrees is still protected |
| `pilot_landing_is_left_alone`, `rtl_landing_is_left_alone` | A landing is never interrupted |
| `wind_estimate_warning` | `WFS_WSPD` with the drag parameters |
| `gusty_wind` | Gusts at 7 m/s do not trip, gusts at 14 m/s do |

The simulated quadcopter leans at most 30 degrees and holds its place in up to 10 m/s of wind.
A real drone has other numbers, so these tests prove the logic, not the limits of a real drone.

## GPS-denied navigation, the flight recorder

Read [gps_denied/RECORDER.md](gps_denied/RECORDER.md) (the set-up on the drone) and [gps_denied/README.md](gps_denied/README.md) (how it works).

Short version: on a Raspberry Pi 5 with a Pi camera looking straight down and three wires to a free serial port of the Cube (MAVLink 2), `sudo bash install.sh`.
During day flights it saves a picture a second with the drone's position and angles at that moment.
