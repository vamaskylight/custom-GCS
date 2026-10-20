# VAMA lock (firmware lock, part 1)

`vama_lock.lua` is a script that runs on the flight controller (ArduCopter 4.6 or newer).
Once the drone is bound, it arms only while a bound ground station is connected.

**Status: tested in the ArduCopter simulator only (4.6.2 and 4.7.0). It has not run on a real drone yet.**
It is installed switched off (`VLCK_ENABLE` 0), so copying it to a drone changes nothing until the lock is turned on.

## What "bound" means

- The drone holds the team's MAVLink signing key.
- A bound ground station signs everything it sends with the same key.
  VGCS and the VAMA app both do this once the team's signing passphrase is set in them.
- With the key on the drone, ArduPilot itself drops every unsigned or wrongly signed message on its telemetry ports.
  Nobody without the key can command the drone, change its settings or turn the lock off over the radio.
- Signing cannot stop a pilot who only uses the sticks. The lock does that.
  Arming by sticks, by a switch or by a command is refused until a signed heartbeat of a ground station arrives on a telemetry port.
  Ground stations send one every second.
  The drone stays unlocked while they come, and for `VLCK_HOLD` seconds (5) after the last one.
- It does not depend on the remote control.
  Any remote control works with it, as long as the ground station on it (the VAMA app) signs with the team's key.

## What it does not do

The lock is only part of the arming checks.

- **It never acts in flight.** A drone that is armed is left alone. In the simulator, a drone that lost its ground station for 30 s in flight stayed armed, in its mode and at its height. After the landing it is locked again.
- **USB is open.** ArduPilot takes unsigned messages on the USB port even with the key, so whoever has the USB port can turn the lock off (`VLCK_ENABLE` 0 or `SCR_ENABLE` 0) or remove the key. For the same reason a ground station on USB never unlocks the drone.
- **The SD card and the firmware are open.** Removing the script from the SD card, or flashing a normal ArduPilot, removes the lock.
- **A forced arm passes.** A ground station can arm "forced", which skips all arming checks. That needs the key or the USB port.
- **With all arming checks off it cannot work.** With `ARMING_CHECK` 0 (4.6) or `ARMING_SKIPCHK` -1 (4.7), ArduCopter skips the lock's check. The script says so every 30 s. Changing these settings also needs the key or the USB port.
- **Without the key on the drone it is weak.** Any ground station can then set `VLCK_ENABLE` to 0. The script says "unsigned GCS ... no key?" when it sees a ground station that does not sign.

So part 1 stops anyone who has the drone's radio link or another remote control, but not someone who opens the drone.
Part 2 of the firmware lock is meant for that: the lock built into the firmware, ArduPilot's secure bootloader (it starts only firmware signed with the team's key), and the USB port. Part 2 depends on the remote control model.

## Install and bind

Once per drone:

1. Put the team's signing key on the drone. USB is the safest link for this.
   - VGCS: Application Settings, General, "Command signing (MAVLink 2)": "Set passphrase..." with the team's passphrase, then "Send key to the drone".
   - VAMA app: Settings, Telemetry, MAVLink 2 Signing, "Add Signing Key" with the same passphrase, then "Enable" with the drone connected.
   - Both apps make the same key from the same passphrase.
2. Set `SCR_ENABLE` to 1 and reboot the flight controller.
3. Copy `vama_lock.lua` to the SD card folder `APM/scripts/` and reboot.
   The message "VAMA lock: ready (off)" comes.
4. With the bound ground station connected over the radio (not USB), set `VLCK_ENABLE` to 1.
   The message "VAMA lock: unlocked by GCS 255, channel 1" comes (the numbers can differ).
5. Check it. Close the ground station, or switch off its radio, and wait 5 seconds.
   "VAMA lock: locked, no signed ground station" comes.
   Arming with the sticks is refused with "Arm: VAMA locked: no signed ground station".
   Open the ground station again: "VAMA lock: unlocked ...", and the sticks arm.

A drone with the lock arms only while VGCS or the VAMA app is connected to it over the radio, with the team's key.

**Keep the passphrase safe.** Without it nobody can unlock the drone over the radio.
With the USB port and Mission Planner or QGroundControl, `VLCK_ENABLE` can still be set to 0.

**Bench work over USB:** set `VLCK_ENABLE` to 0 over USB, and back to 1 afterwards.

**After a scripting restart, reboot.** ArduPilot gives a script a new arming check each time it starts, and the old one waits forever ("Waiting for auxiliary authorisation"). After three restarts there is none left. A reboot clears this.

**No other script may read MAVLink heartbeats.** ArduPilot keeps one message queue for all scripts. The script warns "another script reads heartbeats" when it is not alone. The VAMA wind failsafe reads none.

## Parameters

| Parameter | Default | Meaning |
|-----------|--------:|---------|
| `VLCK_ENABLE` | 0 | 1: the drone arms only while a ground station that signs with its key is connected on a telemetry port. Set it only after the key is on the drone. 0: off. |
| `VLCK_HOLD` | 5 | Seconds a signed heartbeat keeps the drone unlocked (2 to 60). |

The parameter table key is 197 (the wind failsafe uses 196).

## Messages

| Message | Meaning |
|---------|---------|
| VAMA lock: ready (on) / ready (off) | The script started. |
| VAMA lock: locked, no signed ground station | No signed ground station for `VLCK_HOLD` seconds: arming is refused. |
| PreArm (or Arm): VAMA locked: no signed ground station | ArduPilot's arming check, from the lock. |
| VAMA lock: unlocked by GCS 255, channel 1 | A signed ground station (system id 255) on MAVLink channel 1. Arming is allowed. |
| VAMA lock: off (VLCK_ENABLE 0) | The lock is off. Arming is as without the script. |
| VAMA lock: unsigned GCS 255 on channel 2, no key? | A ground station that does not sign reached the script. With the key on the drone ArduPilot drops those, so the drone has no key, or that channel takes unsigned messages. |
| VAMA lock: USB does not unlock, use the radio link | A ground station on USB while the drone is locked. |
| VAMA lock: arming checks are all off, no lock | `ARMING_CHECK` 0 or `ARMING_SKIPCHK` -1: the lock cannot stop arming. |
| VAMA lock: another script reads heartbeats | Another script took the heartbeat messages. The lock may miss its ground station. |
| VAMA lock: error line ... | A mistake in the script. The drone stays locked (or off, when `VLCK_ENABLE` is 0). |

The script also sends `VLCK` (NAMED_VALUE_FLOAT) once a second: 0 off, 1 locked, 2 unlocked.

## How it decides

10 times a second it reads the heartbeats that arrived, and counts only one that:

- is a heartbeat of a ground station (MAVLink vehicle type GCS),
- carries a signature (MAVLink 2),
- has a correct checksum (ArduCopter 4.7 hands messages with a wrong checksum to scripts too, without checking their signature; 4.6 does not),
- came in on a channel other than 0 (USB).

With the key on the drone, ArduPilot has already checked the signature of every such message on a telemetry port, and dropped the others.

## Tests

`drone/test/lock_test.py` runs these cases in the ArduCopter simulator, on 4.6.2 and 4.7.0 (see `drone/README.md`):

| Case | What it proves |
|------|----------------|
| `off_by_default` | Installed, nothing changes: the sticks arm |
| `locked_without_a_station` | Key and lock on, no ground station: neither the sticks nor an arm command arm it, and the refusal names the lock |
| `bound_station_unlocks` | The ground station with the key unlocks it, sticks and commands arm, and 4 to 7 s after it goes the lock is back |
| `others_stay_locked` | A ground station without the key, one with another key, a signed heartbeat that is not a ground station's, a signed one with a wrong checksum, and a signed one on USB: all stay locked |
| `no_key_on_the_drone` | Without the key the sticks are still stopped, the script names the unsigned ground station, and that ground station can turn the lock off |
| `settings_need_the_key` | With the key, a ground station without it cannot turn the lock off |
| `flight_is_never_disturbed` | Losing the ground station in flight changes nothing; locked after the landing |
| `known_limits` | A forced arm passes; with all arming checks off the sticks arm it, and the script says so (and never while they are on) |
| `busy_link_keeps_it_unlocked` | 15 other heartbeats a second on the ground station's link: it stays unlocked |

`system_test/vgcs_sitl_test.py` has one more case, `vama_lock_with_vgcs`: the real VGCS link code puts its key on the drone, turns the lock on, unlocks and arms the drone; when VGCS goes away the drone is locked again.
