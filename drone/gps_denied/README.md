# GPS-denied navigation (item A8)

The drone finds its own position without GPS: a fixed camera under the drone looks straight down, and a small computer on the drone matches that picture with map pictures stored on board, and gives the position to the flight controller as a GPS (`GPS_INPUT`).
It runs on the drone, not in VGCS. The requirements are in the internal notes (A8).

The steps:

1. **The recorder** (this folder, done in code 2026-10-28): during normal day flights it saves the down camera's picture once a second, with the drone's position and angles at that moment. This is the data to train and test on.
2. The matching, tried offline on the recorded flights.
3. The matching live on the Pi, sending `GPS_INPUT`. In the simulator first.
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
```

## How each picture gets its moment

The camera stamps each frame on the Pi's clock (libcamera's SensorTimestamp, a monotonic clock).
The flight controller stamps its messages on its own clock (time_boot_ms).
For each quarter second the recorder keeps the message that arrived fastest, and draws a straight line through the last three seconds of those: that is the difference of the two clocks, free of the link's jitter, and it follows them as they drift apart.
A picture's row is written a second later, so the line runs through the picture's moment.
Its position, heights and speed (GLOBAL_POSITION_INT, 10 a second) and its angles (ATTITUDE, 20 a second) are then interpolated between the controller's own samples at that moment.

## Tests

- `tests/test_gps_denied_recorder.py` (local, like the other tests): the settings, the clock (also a clock that drifts 3 percent), the interpolation (also across north), when to record, the files, whole flights with a simulated drone whose messages arrive 5 to 40 ms late, the Pi camera with a stand-in for picamera2, the bench check.
- `../test/recorder_sitl_test.py`: the recorder as its own program against the ArduCopter simulator (4.6.2 and 4.7.0), a real flight, each picture's moment and state checked against the simulator's own samples on another port.

Not tried yet on a Raspberry Pi, a Pi camera or a real flight controller.
