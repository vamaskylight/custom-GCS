# VAMA high wind failsafe

`vama_wind_failsafe.lua` is a script that runs on the flight controller (ArduCopter 4.6 or newer).
It brings the drone home, or lands it, when the wind is too strong for it.

It runs on the drone, not in the ground station.
So it works with VGCS, with the VAMA APK, with any other app, and also when the radio link is lost.

**Status: tested in the ArduCopter simulator only (4.6.2 and 4.7.0). It has not flown on a real drone yet.**
It is installed as "warning only", so that the first real flights cannot change the flight mode.

## What it watches

It watches only while the autopilot holds a position or flies a route by itself:
AUTO, GUIDED, LOITER, CIRCLE, POSHOLD, BRAKE, FOLLOW, ZIGZAG, and the return modes (RTL, SMART_RTL, AUTO_RTL).
In STABILIZE, ALT_HOLD and other pilot modes it does nothing, because the pilot flies.

| Check | It trips when | Why |
|-------|---------------|-----|
| Motors at their limit | The highest motor output is at or above `WFS_MOT_PCT` (90 %) for `WFS_TIME` (5 s) | No thrust is left: wind, weak battery, heavy load or a failing motor |
| Pushed away | The drone leans hard against the wind and still moves away from its target at `WFS_PUSH_SPD` (1 m/s) or more for `WFS_TIME` (5 s), without slowing down | The wind is stronger than the drone can fly against |

"Leans hard" means `WFS_LEAN` (20 degrees) or more, or motors at `WFS_WARN_PCT` (80 %) or more.

### Why there are two checks

The plan was one check: motors close to maximum.
The simulator showed that this check alone does not catch a drone that is blown away.

ArduPilot limits the lean angle (30 degrees by default).
When the wind needs more lean than that, the drone is pushed away, and its motors stay far from maximum.

Simulated quadcopter in LOITER at 20 m, 30 degree lean limit:

| Wind (m/s) | Highest motor | Lean | Moved in 30 s |
|-----------:|--------------:|-----:|--------------:|
| 0 | 53 % | 0° | 0 m |
| 6 | 54 % | 16° | 0 m |
| 8 | 55 % | 23° | 0 m |
| 9 | 56 % | 26° | 0 m |
| 10 | 58 % | 30° | 0 m |
| 11 | 58 % | 30° | 30 m |
| 12 | 58 % | 30° | 65 m |
| 14 | 58 % | 30° | 138 m |
| 17 | 59 % | 30° | 266 m |

At 14 m/s this drone was pushed away at about 5 m/s with its motors at 58 %.
A check at 90 % would never have acted.
The numbers are from the simulator's quadcopter. A real drone has other numbers, but the same rule.

## What it does

`WFS_ACTION` sets what happens when a check trips.

| WFS_ACTION | Result |
|-----------:|--------|
| 0 | Off. The script only reports that it is installed. |
| 1 | Warning only (how it is installed). It sends a message and never changes the flight mode. |
| 2 | RTL. If RTL is refused, it lands. If RTL is pushed away too, it lands. |
| 3 | LAND where the drone is. |

More details:

- RTL needs a position. If GPS is lost or jammed, the autopilot refuses RTL, and the script lands instead.
- RTL gets 10 seconds to stop the drift. If the drone is then still pushed away for `WFS_TIME`, the script lands it. Each second in such wind takes the drone further away.
- This also holds for an RTL that the pilot or another failsafe started.
- RTL is left alone while it gets closer to home. If home is downwind, RTL flies there first.
- It acts **once per flight**. After that the pilot's mode changes are left alone, until the drone has landed and disarmed.
- A landing that has started (LAND mode, or the last part of RTL) is never interrupted.

## Install

1. Connect with Mission Planner or QGroundControl.
2. Set `SCR_ENABLE` to `1` and reboot the flight controller.
3. Copy `vama_wind_failsafe.lua` to the SD card, into the folder `APM/scripts/`.
   In Mission Planner use CONFIG, then MAVFtp. Or take the SD card out and copy the file with a card reader.
4. Reboot the flight controller.
5. Refresh the parameters. The parameters that start with `WFS_` are now in the list. This proves that the script runs.
6. In VGCS, open the pre-flight popup. The line "Wind failsafe" says "Warning only".

At start the script also sends the message `Wind failsafe: ready (warning only)`.
You see it only if the ground station was already connected at that moment.

If the `WFS_` parameters are not there:

- `SCR_ENABLE` is not 1, or the flight controller was not rebooted after it was set.
- The file is not in `APM/scripts/`.
- A message that starts with `Lua:` shows an error. Send us that message.
- Another script uses the same parameter table number. Change `PARAM_TABLE_KEY = 196` in the script to another free number from 0 to 200.

Do the same on every drone. The script is not part of the firmware, so a new SD card has no script.

## First flights

1. Leave `WFS_ACTION` at `1` (warning only).
2. Fly a normal flight in normal wind: take off, hover, climb, fly a mission, RTL.
3. Look at the "Wind failsafe" value in VGCS during the flight. It shows the motor output and the lean angle.
4. A normal flight must show **no** `Wind failsafe:` message.
   - `Wind failsafe: motors at 8x%` in a normal flight means the drone has little thrust to spare. Tell us the hover value before changing a limit.
5. When normal flights are quiet, set `WFS_ACTION` to `2`.

Never raise a limit only to make a warning go away.

## Parameters

| Parameter | Default | Meaning |
|-----------|--------:|---------|
| `WFS_ACTION` | 1 | 0 off, 1 warning only, 2 RTL, 3 LAND |
| `WFS_MOT_PCT` | 90 | Motor output (%) that trips the motor check. 0 turns that check off. |
| `WFS_TIME` | 5 | Seconds a condition must last before the script acts (2 to 30). |
| `WFS_WARN_PCT` | 80 | Motor output (%) for the early warning. It also counts as "leans hard". 0 turns the warning off. |
| `WFS_PUSH_SPD` | 1 | Speed (m/s) of being pushed away that trips the pushed check. 0 turns that check off. |
| `WFS_LEAN` | 20 | Lean angle (degrees) that counts as "leans hard". Also the level of the lean warning. |
| `WFS_WSPD` | 0 | Wind speed (m/s) for a warning from the autopilot's wind estimate. 0 is off. Needs the drag parameters, see below. |

The script reads the drone's own settings (`MOT_SPIN_MIN`, `MOT_SPIN_MAX`, `MOT_PWM_MIN`, `MOT_PWM_MAX`, the lean limit).
"Motor output 100 %" is `MOT_SPIN_MAX`, the highest output the autopilot gives a motor.
`WFS_LEAN` is never taken as more than 80 % of the drone's lean limit.

## Messages

All messages start with `Wind failsafe:`.

| Message | Meaning |
|---------|---------|
| `Wind failsafe: ready (warning only)` | The script started. In brackets is what it is set to do. |
| `Wind failsafe: leaning 26 deg to hold position` | Early warning. The drone leans hard just to stay in place. At most once a minute. |
| `Wind failsafe: motors at 84%` | Early warning. The motors are above `WFS_WARN_PCT`. At most once a minute. |
| `Wind failsafe: wind 9 m/s` | Early warning from the wind estimate (`WFS_WSPD`). At most once a minute. |
| `Wind failsafe: pushed back 4 m/s, RTL` | The pushed check tripped, and the script started RTL. |
| `Wind failsafe: motors 93% for 5s, RTL` | The motor check tripped, and the script started RTL. |
| `... , LAND` | The script started LAND (`WFS_ACTION` 3). |
| `... , no RTL, LAND` | RTL was refused (no position), so the script started LAND. |
| `Wind failsafe: RTL is pushed back, LAND` | RTL was pushed away too, so the script started LAND. |
| `... (warning only)` | A check tripped, but `WFS_ACTION` is 1, so nothing was changed. |
| `Wind failsafe: error ...` | The script has a fault. Send us the message. |

The script also sends values for the ground station:

| Value | When | Meaning |
|-------|------|---------|
| `WFS_ACT` | Always, every 2 s | `WFS_ACTION`. It tells the ground station that the script runs. |
| `WFS_MOT` | In flight, every second | Highest motor output, % |
| `WFS_LEAN` | In flight, every second | Lean angle, degrees |
| `WFS_PUSH` | In flight, every second | Speed at which the drone is pushed away, m/s |

## What VGCS shows

- **Pre-flight popup**: a line "Wind failsafe". It says if the script runs and what it will do.
  "Not running on this drone" means the script is missing or scripting is off.
- **Vehicle status** (VGCS menu): "Wind failsafe" with the action, the motor output and the lean angle.
- **Message line**: every `Wind failsafe:` message. Warnings and actions stay there for 15 to 25 seconds.
- **Wind**: the drone's wind estimate on the map strip and in Vehicle status, with a warning level
  (Application Settings, General, Wind warning). It shows `N/A` until the drag parameters are set.

## The wind estimate

ArduCopter can estimate the wind from how much the drone must lean.
It needs three parameters that describe the drone's drag. They are different for every drone type.

| Parameter | How it is found |
|-----------|-----------------|
| `EK3_DRAG_BCOEF_X` | Mass in kg, divided by the frontal area in m² |
| `EK3_DRAG_BCOEF_Y` | Mass in kg, divided by the side area in m² |
| `EK3_DRAG_MCOEF` | From the log of one test flight |

ArduPilot describes the method here: https://ardupilot.org/copter/docs/airspeed-estimation.html

Without them the drone sends no wind estimate, VGCS shows `N/A`, and `WFS_WSPD` does nothing.
The two checks above do **not** need the wind estimate.

One thing seen in the simulator (4.7.0): after one sudden jump from 0 to 9 m/s the estimate stayed stuck near 2 m/s.
With wind that built up in steps it was right.
So the wind estimate is for the display and for an early warning. The failsafe never depends on it.

## Limits

- Tested in the simulator only. The first real flights must be in "warning only".
- Landing in very strong wind can tip the drone over. It is still better than a drone that is blown away.
- A slow drift, below `WFS_PUSH_SPD`, only gives the lean warning. It is not acted on.
- RTL that stands still against the wind (not pushed back, but no progress) is not acted on. The battery failsafe stays the last line of defence.
- The pushed check needs a position. Without GPS the autopilot's own EKF failsafe acts first.
- The script sees movement, not its cause. A drone that circles away from its hold point because of a compass problem can look like wind to it.
  The motor check also trips for a weak battery, a heavy load or a failing motor. So read the log before blaming the wind.
- The script does nothing in pilot modes (STABILIZE, ALT_HOLD, ACRO).
- Modes without a hold point (POSHOLD, GUIDED with speed commands) are judged by the lean angle and the movement only.

## Tests

The simulator tests are in `drone/test`. See `drone/README.md`.
