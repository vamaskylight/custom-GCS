# VGCS system tests (milestone M17)

These tests run VGCS against the real ArduCopter program in ArduPilot's simulator (SITL).
The unit tests in `tests/` check pieces of VGCS one at a time.
These check that the pieces work together with a real autopilot, on both firmware versions the client flies (4.6.2 and 4.7.0).

Nothing goes over the network.
The simulator runs in WSL on this computer, and VGCS talks to it through `127.0.0.1`.

## What is real and what is simulated

Real:

- the ArduCopter program itself, the same code that flies the drones (built for the simulator),
- VGCS's link code (`vgcs/link/mavlink_thread.py`), and in the window test the whole VGCS window.

Simulated: the airframe, the motors, the sensors, the GPS and the wind.

VGCS saying "done" is never taken as proof.
A second, independent connection to the simulator (`drone/test/sitl_session.py`) checks what the drone really did.

## Set up once

The same set-up as the wind failsafe tests, see [drone/README.md](../drone/README.md):

- Windows with WSL and the `Ubuntu-24.04` distribution,
- `py` (Python on Windows) with `pymavlink` and `PySide6`,
- the simulator programs, downloaded once with `drone/test/setup_sitl.sh`.

## Run

From the repo root:

```powershell
py system_test/vgcs_sitl_test.py                      # 10 cases on 4.6.2 and 4.7.0, about 5 minutes
py system_test/vgcs_sitl_test.py 4.7.0 -k mission     # one version, only cases with "mission" in the name
py system_test/vgcs_sitl_test.py --report results.md  # also write the results as Markdown

py system_test/vgcs_window_test.py                    # the whole window, 4.7.0, 3 minute hover and a mission planned and flown, about 8 minutes
py system_test/vgcs_window_test.py 4.6.2 --minutes 1
py system_test/vgcs_window_test.py --speedup 10       # ten times the message rate: a stress test
py system_test/vgcs_window_test.py --report window.md

py system_test/vgcs_fleet_test.py                     # three drones at once, 4.7.0, about 1 minute
py system_test/vgcs_fleet_test.py 4.6.2 --report fleet.md
```

Only one simulator can run at a time (it always uses TCP port 5760).
Do not run two of these scripts, or a wind failsafe test, at the same time.

## Live demo: drones to fly by hand

`demo_sitl.py` starts simulated drones in real time and leaves the flying to you.
Connect VGCS (the exe or `py -m vgcs`) to the address it prints, and use VGCS as with a real drone.
The demo plan for the client is `DOCS/M17-DEMO-PLAN.md`.

```powershell
py system_test/demo_sitl.py                            # one drone, 4.7.0: connect VGCS to tcp:127.0.0.1:5762
py system_test/demo_sitl.py --drones 3                 # three drones 40 m apart, for the Fleet panel (5762, 5772, 5782)
py system_test/demo_sitl.py --version 4.6.2
py system_test/demo_sitl.py --wind 9 --wind-failsafe   # wind, and the wind failsafe script as installed (warning only)
py system_test/demo_sitl.py --wind 14 --wind-failsafe --wind-action 2   # too much wind: the script sends the drone home
py system_test/demo_sitl.py --minutes 30               # stops by itself after 30 minutes
```

It prints the drones' own messages as they come.
Ctrl+C stops the drones.
A second ground station can join on `tcp:127.0.0.1:5763` (for the command signing demo).

## `vgcs_sitl_test.py`: the link against the drone

VGCS's `MavlinkThread` connects to the simulator's second port (`tcp:127.0.0.1:5762`), and every command goes through the same `queue_...` call the buttons use.
The simulated clock runs 10 times faster than real time.

| Case | What it flies |
|------|---------------|
| connect_and_telemetry | connect, every kind of data arrives, position, height, GPS, battery and the arm verdict match the drone |
| modes_and_arming_on_the_ground | four mode changes confirmed by the drone, arm, disarm, emergency motor stop |
| refusals_say_why | no GPS: take-off refused with the drone's reason and the mode put back; in the air LOITER refused "requires position"; landing without GPS |
| parameters | the settings VGCS reads exist on this firmware (4.6 and 4.7 names differ), a write is confirmed, a missing name is reported as not written |
| mission_upload_and_download | a 4 waypoint mission goes up and comes back the same, to the centimetre |
| mission_flight | start from the ground, pause, resume, skip a waypoint, return and land at home |
| takeoff_fence_and_land | a 40 m fence with a 30 m height limit: the drone turns back at both, VGCS shows RTL and the drone's message, landing from VGCS |
| link_silence | the radio goes quiet in flight (the simulator is frozen), VGCS reports it within its 2 second watchdog, and recovers by itself |
| signed_commands | VGCS gives the drone its signing key: an outsider ground station (third port) can no longer change the mode, arm, or remove the key, VGCS still can, and after the key is removed the outsider is obeyed again |

## `vgcs_window_test.py`: the window, and how well it keeps up

Builds the real main window off-screen, types the simulator's address into the connection box and presses Connect.
Then it flies through the window's own button handlers: a take-off the drone refuses, a real take-off, a mode change, a hover, a radio silence, and landing.
The window's own labels are read to check what the operator would have seen, and the controls the operator needs are checked to be on screen (E-STOP, Disconnect while the link is lost, the Vehicle status window).

During the hover it measures how late the window's event loop runs, the time spent in the telemetry handler, CPU and memory.
The map's web view runs in separate QtWebEngine processes, which are not counted.

Your own VGCS settings are never touched.
Every QSettings in the test process is sent to an INI file in a temporary folder before VGCS is imported.

Nothing is fetched from the internet.
The map draws from an offline tile folder the test makes for itself (coloured squares around the start view and the simulator's home), and a check fails if the map asks for any internet tile.
Do not remove that check: before 2026-10-07 VGCS's no-network switch did not cover the map, and a test run fetched satellite tiles and wrote them into `~/.vgcs/tile-cache`.

## `vgcs_fleet_test.py`: several drones (M15)

Three simulators run side by side (instances 0, 1 and 2 of `drone/test/sitl_session.py`, system ids 1, 2 and 3, homes 40 m apart).
The real window connects drone 1 with the Connect button, and drones 2 and 3 the way the Fleet panel adds them.

It checks the acceptance test of client requirement 14:

- all three links are up, each reads its own drone, and the map and the Fleet panel show all three,
- putting another drone on screen does not drop the others (both stay heard),
- the window's take-off goes to the drone on screen only, and an uploaded mission lands on the drone on screen only,
- return home to one drone, land to all,
- one drone frozen (radio silence): only its link is lost, and the window names it,
- disconnecting one drone leaves the others, and closing the window stops every link.

## Traps (each cost time once)

- ArduCopter accepts any mode while it is disarmed: it checks the mode when it arms.
  A "refused mode" has to be tested in the air.
- ArduCopter refuses a fly-to point outside its fence, and in GUIDED it stops short of the fence by itself (`AVOID_ENABLE`).
  To breach a fence on purpose, fly on speed commands with `AVOID_ENABLE 0`.
- `SIM_GPS1_ENABLE 0` in the start-up file did not stop the simulated GPS on 4.7.0.
  Set it by MAVLink after start-up (`Sitl.gps_off()`).
- pymavlink sends a mode change as `MAV_CMD_DO_SET_MODE`.
  ArduPilot answers it with an ack at once, and the STATUSTEXT with the reason comes a moment later.
- VGCS's timers (retries, giving up) run on the PC's clock, the simulator's runs 10 times faster.
  Wait for VGCS results with `Flight.wait_real`, not `Flight.wait`.
- Freezing the simulator (`pkill -STOP -x arducopter`) is a radio silence: the connection stays open and nothing arrives.
  Killing it is a closed TCP connection, which is a different case (see the test report).
- `wsl.exe -- bash -lc "..."` lets the outer shell expand every `$` first (`$!` and `$x` came out empty).
  `sitl_session.py` uses `wsl.exe --exec`, which passes the command on untouched.
- ArduCopter 4.7 renamed SYSID_THISMAV to MAV_SYSID. The simulator's `--sysid` option sets either.
- An empty mission counts 0 items on 4.6.2 and 1 (home) on 4.7.0.
- Arming refuses "Throttle (RC3) is not neutral" while an RC override holds the throttle in the middle.
  Set the throttle to the middle after the take-off, not before.
