# GPS-denied navigation (item A8)

The drone finds its own position without GPS: a fixed camera under the drone looks straight down, and a small computer on the drone matches that picture with map pictures stored on board, and gives the position to the flight controller as a GPS (`GPS_INPUT`).
It runs on the drone, not in VGCS. The requirements are in the internal notes (A8).

The steps:

1. **The recorder** (this folder, done in code 2026-10-28): during normal day flights it saves the down camera's picture once a second, with the drone's position and angles at that moment. This is the data to train and test on.
2. The matching, tried offline on the recorded flights.
3. The matching live on the Pi, sending `GPS_INPUT`. In the simulator first.
   **3a, the way into ArduPilot (`vama_nav`), done in code 2026-10-29:** tried in the simulator with the simulator's true position plus noise. The matching itself (3b) is not written.
4. VGCS and the app: load the map before the flight, show the position source and its quality.
5. Night.

## This folder

```text
gps_denied/
  README.md                  this file
  RECORDER.md                the set-up and use, for the team on the drone
  recorder.ini               the settings, each with a note (installed as /etc/vama-recorder.ini)
  install.sh                 installs the recorder on the Raspberry Pi as a service
  vama-recorder.service      the service
  vama_recorder/             the recorder (Python 3.11 or newer, pymavlink, picamera2 on the Pi)
    __main__.py                python -m vama_recorder [--check] [--fake-camera] [--flights N]
    settings.py                the settings file
    clock.py                   the flight controller's clock on the Pi's clock
    telemetry.py               the controller's messages, the drone's state at any moment
    camera.py                  the Pi camera (picamera2), and a stand-in without one
    flight.py                  one flight's folder: pictures, frames.csv, telemetry.jsonl, flight.json
    link.py                    the MAVLink link, its stream requests and heartbeat
    recorder.py                the recorder's loop, and the bench check
  vama_nav/                  our position into ArduPilot as its GPS 2 (step 3a; Python 3.11 or newer, pymavlink)
    __main__.py                python -m vama_nav --sim-truth [--device ...] (no other source yet)
    gps_input.py               what each GPS_INPUT holds and when, and the settings it checks
    runner.py                  the link: GPS_INPUT ten times a second, heartbeat, stream and setting requests
    sim_truth.py               the stand-in camera: the simulator's true position with noise
```

## How each picture gets its moment

The camera stamps each frame on the Pi's clock (libcamera's SensorTimestamp, a monotonic clock).
The flight controller stamps its messages on its own clock (time_boot_ms).
For each quarter second the recorder keeps the message that arrived fastest, and draws a straight line through the last three seconds of those: that is the difference of the two clocks, free of the link's jitter, and it follows them as they drift apart.
A picture's row is written a second later, so the line runs through the picture's moment.
Its position, heights and speed (GLOBAL_POSITION_INT, 10 a second) and its angles (ATTITUDE, 20 a second) are then interpolated between the controller's own samples at that moment.

## Step 3a: our position into ArduPilot (vama_nav)

`vama_nav` sends `GPS_INPUT` for GPS 2 ten times a second (ArduPilot calls a GPS unhealthy below about five):

| The drone | GPS 2 says |
|---|---|
| Disarmed | GPS 1's own fix, passed on. ArduPilot arms only when every GPS it knows has a 3D fix, and a camera on the ground matches nothing. GPS 1 is still checked in full before take-off. |
| Armed, with a camera fix less than 2 s old | Each fix once as a 3D fix, carried forward with its own speed to the moment it is sent. The other nine messages of each second carry it forward as a 2D fix, which the EKF does not use. |
| Armed, no camera fix or an older one | No fix |

ArduPilot uses GPS 2 only while GPS 1 has no 3D fix (`GPS_AUTO_SWITCH` 4).
The sender reads the settings below from the flight controller, and sends no camera position until they are right.

Why only one 3D message per fix: ArduPilot's EKF takes each 3D message as a new measurement with its own error.
When each fix went ten times (carried forward), the EKF followed every fix's jump.
In the simulator it drifted 15 m, and the EKF failsafe landed the drone 25 s after GPS 1 went off.
With one 3D message per fix the same flight held within about 2 m.

What else is in each message, and why, is at the top of `vama_nav/gps_input.py` (ArduPilot's `AP_GPS_MAV`, 4.6 and 4.7).

### The flight controller's settings (not on the drone yet)

Do not set these before the matching runs on the drone.
With `GPS2_TYPE` 14 and nothing sending, ArduPilot refuses to arm ("GPS 2: Bad fix").

| Setting | Value | Why |
|---|---|---|
| `GPS2_TYPE` | 14 | GPS 2 is a MAVLink GPS: the Pi |
| `GPS_AUTO_SWITCH` | 4 | GPS 1 whenever it has a 3D fix, GPS 2 only when it has none. With 1 (the default, the GPS with more satellites) or 2 (blend) ArduPilot could take the camera's position while GPS 1 works, so the sender holds camera positions back |
| `GPS_PRIMARY` | 0 | GPS 1 is the main GPS |
| `GPS2_DELAY_MS` | 20 | The positions are for the moment they are sent. 0 means 200 ms for a MAVLink GPS: 2.4 m along the track at 12 m/s |
| The Pi's port | `SERIALn_PROTOCOL` 2, `SERIALn_BAUD` 115 | As for the recorder |
| `MAVk_OPTIONS` | add 2 | Makes the Pi's port private. Without it ArduPilot passes the ten messages a second on to the ground station's radio (seen in the simulator). The MAVk entries are the MAVLink ports in order, from SERIAL0 up: USB is MAV1, TELEM1 MAV2, and the Pi's port the next port with protocol 1 or 2 (MAV3 when TELEM2 is the lidar's). ArduPilot 4.6: `SERIALn_OPTIONS`, add 1024 |

The sender needs ArduPilot 4.6 or newer (before it, `GPS_TYPE2` and `GPS_DELAY_MS2`).

### Running it

```bash
python -m vama_nav --sim-truth --device tcp:127.0.0.1:5763 [--noise-m 2] [--every-s 1] [--delay-s 0.4] [--match-above-m 15] [--control FILE] [--seed N]
```

`--sim-truth` is the only position source for now: the simulator's true position (SIM_STATE) with noise, one a second, ready 0.4 s after its moment, only above 15 m.
A real flight controller does not send SIM_STATE, so the program has nothing to send on the drone yet.
`--control` takes a small JSON file that a test changes during the flight: an offset (a picture matched to the wrong place) or a pause (ground that matches nothing).

### Open for step 3 (the matching on the Pi)

- The matching hands each fix to `Sender.offer()`: the picture's moment on the Pi's clock, the position, the speed over the ground (from the shift between two pictures), and honest accuracies.
- The speed is sent as it was at the fix's moment. At the start of each leg in the simulator, the EKF's speed test reached 0.8 of its gate (4.7). The speed's own moment (the middle between two pictures) and its accuracy are to be tuned on real flights.
- A wrong match must be caught before it is offered. While GPS 1 works ArduPilot ignores it (tested with 150 m), but without GPS 1 the EKF would take it.
- The recorder and the matching both need the camera and the serial port: one program, or a MAVLink router.
- A GPS 1 that is spoofed (a wrong position with a 3D fix) keeps priority. Only a GPS 1 without a 3D fix hands over to the camera.
- VGCS and the app (step 4) show GPS 2 and where the position comes from.

## Tests

- `tests/test_gps_denied_recorder.py` (local, like the other tests): the settings, the clock (also a clock that drifts 3 percent), the interpolation (also across north), when to record, the files, whole flights with a simulated drone whose messages arrive 5 to 40 ms late, the Pi camera with a stand-in for picamera2, the bench check.
- `../test/recorder_sitl_test.py`: the recorder as its own program against the ArduCopter simulator (4.6.2 and 4.7.0), a real flight, each picture's moment and state checked against the simulator's own samples on another port.
- `tests/test_gps_denied_nav.py` (local): each GPS_INPUT as ArduPilot decodes it, what is sent when, the settings check, the stand-in camera, the link loop on a stand-in link and clock, the command line.
- `../test/gps_input_sitl_test.py`: the sender as its own program against the ArduCopter simulator (4.6.2 and 4.7.0) in real time. Without the sender the drone does not arm. A mission in AUTO: GPS 1 off for two legs of 250 m (flown on the camera's positions), back on, then both gone (the EKF failsafe lands the drone). Also a camera position 150 m off while GPS 1 works (ignored), and the private port.

Not tried yet on a Raspberry Pi, a Pi camera or a real flight controller.
