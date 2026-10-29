# GPS-denied navigation (item A8)

The drone finds its own position without GPS: a fixed camera under the drone looks straight down, and a small computer on the drone matches that picture with map pictures stored on board, and gives the position to the flight controller as a GPS (`GPS_INPUT`).
It runs on the drone, not in VGCS. The requirements are in the internal notes (A8).

The steps:

1. **The recorder** (this folder, done in code 2026-10-28): during normal day flights it saves the down camera's picture once a second, with the drone's position and angles at that moment. This is the data to train and test on.
2. The matching, tried offline on the recorded flights.
   **The tool is ready (`python -m vama_nav.replay`, 2026-10-29)** and waits for the client's recorded flights.
3. The matching live on the Pi, sending `GPS_INPUT`. In the simulator first.
   **3a, the way into ArduPilot (`vama_nav`), done in code 2026-10-29:** tried in the simulator with the simulator's true position plus noise.
   **3b, the matching (`vama_nav --map`), done in code 2026-10-29:** tried in the simulator on real aerial photos of two different years. Not tried on a Pi or a Pi camera.
4. VGCS and the app: load the map before the flight, show the position source and its quality.
   **The VGCS part, done in code 2026-10-29:** "Make the drone's map..." in the Map tiles menu, and the position source (GPS 1 or the camera) shown in VGCS. The app part is not started.
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
  vama_nav/                  the matching and our position into ArduPilot as its GPS 2 (steps 2 and 3;
                             Python 3.11 or newer, pymavlink, numpy, OpenCV)
    __main__.py                python -m vama_nav --map MAP.mbtiles [--sim-camera ...] | --sim-truth
    gps_input.py               what each GPS_INPUT holds and when, and the settings it checks
    runner.py                  the link: GPS_INPUT ten times a second, heartbeat, stream and setting requests
    geo.py                     map tiles and metres on the ground
    map_tiles.py               the map on board: one MBTiles file
    geometry.py                where the camera looks; a picture turned north up; a picture drawn from a map
    matcher.py                 one picture placed on the map, its checks, the movement between pictures
    track.py                   which positions agree with the last ones
    xfeat_match.py             --features xfeat: learned features for a map of another season (needs PyTorch)
    match_source.py            the camera, the matching and the fixes, in their own threads
    recording.py               --record: the recorder's flight folders from the same pictures
    replay.py                  step 2: python -m vama_nav.replay FLIGHT... --map MAP.mbtiles
    sim_camera.py              the stand-in camera for the simulator: the ground drawn from a map
    sim_truth.py               step 3a's stand-in: the simulator's true position with noise
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
| The Pi's port | `SERIALn_PROTOCOL` 2, `SERIALn_BAUD` 115 | As for the recorder. The matching (step 3b) asks for ATTITUDE 100 times a second, about 4 KB/s of the 11.5 KB/s at 115200; 921 (921600) leaves more room |
| `MAVk_OPTIONS` | add 2 | Makes the Pi's port private. Without it ArduPilot passes the ten messages a second on to the ground station's radio (seen in the simulator). The MAVk entries are the MAVLink ports in order, from SERIAL0 up: USB is MAV1, TELEM1 MAV2, and the Pi's port the next port with protocol 1 or 2 (MAV3 when TELEM2 is the lidar's). ArduPilot 4.6: `SERIALn_OPTIONS`, add 1024 |

The sender needs ArduPilot 4.6 or newer (before it, `GPS_TYPE2` and `GPS_DELAY_MS2`).

### Running it

```bash
python -m vama_nav --sim-truth --device tcp:127.0.0.1:5763 [--noise-m 2] [--every-s 1] [--delay-s 0.4] [--match-above-m 15] [--control FILE] [--seed N]
```

`--sim-truth` is step 3a's stand-in source: the simulator's true position (SIM_STATE) with noise, one a second, ready 0.4 s after its moment, only above 15 m.
A real flight controller does not send SIM_STATE.
`--control` takes a small JSON file that a test changes during the flight: an offset (a picture matched to the wrong place) or a pause (ground that matches nothing).
The matching (step 3b, below) is the real source.

A GPS 1 that is spoofed (a wrong position with a 3D fix) keeps priority. Only a GPS 1 without a 3D fix hands over to the camera.

## Step 3b: the picture matching (vama_nav --map)

`vama_nav --map MAP.mbtiles` takes a picture with the down camera every second while the drone is armed and at least 50 m above home, and finds it on the map:

1. The picture is turned north up at the map's scale, with the drone's angles and height at the picture's moment (`geometry.py`). At 12 m/s a quadcopter flies about 10 degrees nose down, so its camera does not look at the point below it: from 500 m its picture's middle is about 90 m behind it. The angles take that out.
2. Features (ORB, from OpenCV) are found in the picture and in the map around where the flight controller thinks the drone is: 60 m around, more while it has neither GPS 1 nor a recent camera position (5 m more each second, at most 400 m). The pairs that agree on one shift place the point below the drone on the map: the drone's position (`matcher.py`).
3. A position is used only when it passes every check below, and when it agrees with the last positions (`track.py`).
4. It is moved to the flight controller's newest sample by the controller's own movement since the picture (a few tenths of a second, which its sensors measure well), and goes to the sender of step 3a as a fix (`match_source.py`).

The speed is the flight controller's own speed, corrected by how far the ground's shift over the last 3 seconds (against the oldest picture of up to 3 s back) differs from the controller's own movement over the same time.
That is right whatever the drone did in between, a turn too.
Over 3 seconds rather than 1: each picture is turned with the flight controller's angles, and a change in their error between the two pictures shows as a false movement (0.15 degrees is 0.8 m from 300 m).
Without GPS 1 that error changes faster, and the speed from one second was up to 4 m/s off in the simulator; over 3 seconds it was at most 0.7 m/s.
A picture that gives no position can still give the speed.

Without GPS 1 the EKF corrects its roll and pitch at each camera fix, and those corrections are what changes the angles' error between two pictures.
So the program also takes the drone's own turn between the two pictures from its gyros: ATTITUDE's turn rates, which it asks for 100 times a second, summed step by step (`geometry.turn_of`).
What the angles changed beyond that turn is taken out of the shift (`geometry.tilt_shift_m`).
Two things make the gyros' turn right:
- The rates come through the gyros' low-pass filter and are late by about 0.225 s divided by `INS_GYRO_FILTER` (11 ms at the default 20 Hz). The program reads that setting from the flight controller. Without it, entering and leaving a turn the shift was about 1 m off from 300 m.
- ATTITUDE's rates are corrected by the EKF's own estimate of the gyros' bias, which lags (the simulator drifts its gyros on purpose; 0.02 to 0.04 degrees a second were left). What is left is learnt from the EKF's own turn over about 40 s, where the EKF's angles are right.
Without enough rates (fewer than about 33 a second, or a gap) the shift is used as measured, as before.

The program takes two pictures a second (`--interval-s 0.5`).
Two pictures half a second apart see nearly the same ground, and their errors are nearly the same (a second apart they were 55 to 80 % alike without GPS 1, in the simulator).
So each fix says it is less sure by the square root of the fixes a second (1.4 times at two a second, its position and its speed), and a second of fixes weighs what one fix a second did.
The EKF then makes two half corrections a second instead of one whole one, and the drone tips less.
The weight goes by the time since the last fix sent: when a picture is skipped (the matching too slow, no position), the next one weighs fully.
The heading and height errors are followed by the second, not by the picture (`follow`, 30 % a second).

The camera, the matching, the recording and the link each run in their own thread, so `GPS_INPUT` goes out ten times a second however long a picture takes, and whatever the card does.

### What stops a wrong position

A position that is not sure is no position: the flight controller then holds on its own sensors for a few seconds, and after that its EKF failsafe acts.

| Check | No position when |
|---|---|
| Enough pairs agree | fewer than 15 feature pairs agree on one place, or they are bunched in one spot of the picture |
| One map feature for one picture feature | a map feature chosen by two picture features is dropped (it "matches" everything) |
| The heading and the height fit | the place is turned more than 45 degrees, or scaled more than 25 percent, from the flight controller's heading and height |
| One place only | a second place fits nearly as well (rows of houses or trees repeat) |
| Near where the drone should be | the place is outside the search distance |
| The track | the position is more than 25 m (and 3 m more for each second since) from the last positions, moved on by the flight controller's own movement. Two positions in a row that agree start a new track. The first position of a flight waits for a second one |
| The speed | not used while the drone tips fast, or when it is more than 4 m/s from the flight controller's own. The flight controller's speed is then sent with an accuracy of 1 m/s |

The matcher follows the flight controller's heading error and height error from what the matches show.
Each picture is turned with the heading corrected by the error found so far.
A picture that still turns more than 2 degrees is tried a second time with its own turn taken out.
This matters: in the simulator the EKF reset its heading by 20 degrees at a corner ("emergency yaw reset"), and before this every picture after it was refused.

Each fix carries its accuracy: the map's own error (3 m), the drone's angles' error (0.3 degrees, which is 1.6 m from 300 m), the fit's error, and the drone's tipping during the picture's time error (20 ms).
A picture's moment matters: while the drone tips at 20 degrees a second, 20 ms moves the picture by 2 m from 300 m.
On the Pi the moment is the camera's own (libcamera's SensorTimestamp), placed on the flight controller's clock as in the recorder.

### The map

One MBTiles file (map tiles in an SQLite file, one file to copy to the card).
The finest zoom in it is used.
Zoom 17 is about 1.1 m a pixel at the client's latitude (about 0.9 m in the test area).

VGCS makes it: "Make the drone's map..." in the logo menu's Map tiles (`vgcs/map/drone_map.py`).
It covers the mission plan and 2 km round it, or a square round the drone of 1 to 40 km.
It says how many map pictures that is and how big (30 km round home is about 44,000 pictures, about 0.9 GB), and that the pictures' licence must allow their use offline on the drone.
It downloads them like a tile pack, into VGCS's own map cache (pictures already there are not fetched again), from the map's satellite source (Esri World Imagery by default), and packs zoom 17 into one file.
Copy that file to the Pi (a USB stick, or over the network with the Pi on the bench) and give it to `vama_nav --map`.
The radio link to the flight controller cannot carry it.

**The map must look like the ground in flight.**
In a test on 2026-10-29, pictures drawn from the summer photos of 2022 matched the summer map of 2020 30 times out of 30 (middle 1.3 m).
Against Esri's pictures of the same place (winter: bare trees and long shadows) they matched 0 times out of 20, with ORB, AKAZE and SIFT alike.
So with those features, a map of another season or another time of day can give no position at all.

**XFeat bridges the seasons** (`--features xfeat`, `vama_nav/xfeat_match.py`).
XFeat is a set of learned features (Apache 2.0, CVPR 2024), and its semi-dense matching found the same pictures on Esri's winter map 19 times out of 20 (middle 1.9 m, worst 3.4 m), with every check of the matcher on.
On the summer map it found 20 of 20 (middle 2.8 m).
It takes about 0.3 s a picture on this PC (4 threads), against 0.07 s for ORB. On the Pi 5 it is not measured yet.
Only the matching with the map uses it: the movement between two pictures stays with ORB.
It needs PyTorch (CPU is enough) and XFeat's own folder with its weights:

```bash
pip install torch tqdm
git clone https://github.com/verlab/accelerated_features ~/xfeat-src
python -m vama_nav --map MAP.mbtiles --features xfeat --xfeat-dir ~/xfeat-src ...
```

**In the simulator** (ArduCopter 4.7.0, `matching_sitl_test.py 4.7.0 --map davis-esri-z17 --features xfeat`, 2026-10-29) the drone flew both legs with GPS 1 off on XFeat's positions from the winter map.
It had a position for 156 of 172 seconds, the EKF stayed within 3.6 m (middle) and 13.7 m (worst), and no failsafe came.
One check failed: after the 4 seconds of blank ground the EKF was up to 13.7 m off, where the test wants under 10 m.
The same test with ORB on the summer map, the same day: 2.9 m and 6.4 m, and 5.8 m after the blank.
On the winter map the positions are less exact (middle 3.2 m against 2.6 m, with stretches of 5 to 9 m), and the drone corrects more on them.
With GPS 1 off it tipped 8 degrees a second (middle, by the gyro), against 0.2 with GPS 1.
The speed between two pictures is not sent while the drone tips that fast (over 0.8 m/s of possible error), so it went out for only 30 of 164 pictures, and the EKF swung further.
Step 2 on that flight: a position for 466 of 519 pictures, middle 2.4 m, worst 14.5 m (the summer map with ORB: 499 of 516, middle 1.8 m, worst 4.9 m).

The best map is still one of the same season, or the client's own survey flight of the area (made into map tiles).

### Running it

On the Pi (not tried yet). It needs Python 3.11 or newer with pymavlink, numpy and OpenCV (`sudo apt install python3-opencv python3-numpy` on Raspberry Pi OS Bookworm, OpenCV 4.6):

```bash
python -m vama_nav --map ~/map.mbtiles --device /dev/serial0 --baud 115200 [--record --config /etc/vama-recorder.ini] [--results results.csv]
```

In the simulator, without a camera:

```bash
python -m vama_nav --map MAP.mbtiles --sim-camera GROUND.mbtiles --device tcp:127.0.0.1:5763 [--control FILE] [--seed N]
```

| Option | Default | What |
|---|---|---|
| `--map` | | the map on board (MBTiles) |
| `--features` | `orb` | `orb` (the fastest), `akaze` or `sift`: all in OpenCV, free to use. `xfeat`: learned features for a map of another season (PyTorch, see "The map") |
| `--xfeat-dir` | `VAMA_XFEAT_DIR` | `--features xfeat`: XFeat's folder |
| `--hfov-deg` | 66 | the camera's field of view across the picture (the Raspberry Pi AI Camera) |
| `--mount-yaw-deg`, `--mount-pitch-deg`, `--mount-roll-deg` | 0 | how the camera sits on the drone: turned (90 is the top of the picture to the right side), looking ahead, looking right. Step 2 measures them |
| `--interval-s` | 0.5 | seconds between two pictures. Two a second made the drone tip less than one (see "In the simulator"); a matching too slow for that skips pictures by itself |
| `--match-above-m` | 50 | no pictures below this height above home |
| `--search-m` | 60 | how far from the flight controller's position the map is searched |
| `--record` | off | also keep the pictures in the recorder's flight folders (one camera, one program; at two pictures a second, twice the space of the recorder alone); `--config` is the recorder's settings file |
| `--results` | | a CSV file with one row per picture: what was found, what was sent |
| `--sim-camera` | | no camera: pictures drawn from this map (another year) at the simulator's true position and angles |

`--sim-camera` draws what a camera under the drone would see at the simulator's true position and angles (SIM_STATE).
It blurs the picture a little, darkens its corners as a lens does, changes its brightness and adds sensor noise.
Its picture's moment is the flight controller's own time of that truth: the simulator sends SIM_STATE right after the ATTITUDE of the same moment.
Its `--control` file blanks the ground (fog, water) or stops the camera.
Not simulated: the rolling shutter, motion blur, clouds, shadows of the moment, hills.

### In the simulator

`../test/matching_sitl_test.py` flies ArduCopter 4.6.2 and 4.7.0 in real time, at 300 m and 12 m/s over east Davis, California.
The drone's map is the photos of 2020 (zoom 17, about 0.9 m a pixel); the stand-in camera sees those of 2022.
GPS 1 is off for two legs of 800 m, with about 4 s of blank ground in the middle (2026-10-29 night: two pictures a second, the gyros' turn between pictures):

| | 4.6.2 | 4.7.0 |
|---|---|---|
| Positions while GPS 1 works (middle, worst) | 2.0 m, 5.1 m | 1.9 m, 4.3 m |
| Speeds while GPS 1 works (middle, worst) | 0.09, 0.45 m/s | 0.08, 0.41 m/s |
| GPS 1 off: seconds with a position | 165 of 167 (315 fixes) | 168 of 170 (320 fixes; counted from its tracks afterwards) |
| Positions with GPS 1 off | 2.8 m, 6.6 m | 2.1 m, 5.2 m |
| Speeds with GPS 1 off | 0.31, 0.76 m/s | 0.35, 1.5 m/s |
| The EKF with GPS 1 off | 3.7 m, 5.4 m | 3.7 m, 6.1 m |
| The drone's tipping with GPS 1 off (middle; 1 in 10 over) | 1.1; 4.8 degrees a second | 1.6; 5.8 degrees a second |
| The EKF's roll and pitch error with GPS 1 off (middle) | 0.28 degrees | 0.24 degrees |
| The EKF's speed and position checks (1 is the limit) | 0.16, 0.11 | 0.34, 0.11 |
| The blank ground (about 4 s) | no position for 4.0 s | no position for 4.5 s |
| Step 2 on the flight it recorded (pictures from 150 m) | 815 of 829, middle 1.7 m | 1007 of 1040, middle 1.7 m |

With GPS 1 the drone tipped 0.2 degrees a second (middle) and the EKF's roll and pitch were 0.10 to 0.15 degrees off.
How the drone tipped with GPS 1 off, in the same test:

| | 4.6.2 | 4.7.0 |
|---|---|---|
| One picture a second, as in step 3b | | 4.1; 12.1 degrees a second |
| One picture a second, the gyros' turn | 1.4; 7.3 | 2.7; 9.4 (and 2.7; 9.9 in another flight) |
| Two pictures a second, the gyros' turn | 1.1; 4.8 | 1.6; 5.8 |

The matched positions were 2.1 m off (middle) before the gyros' turn and 1.8 to 2.1 m with it (4.7.0).
The EKF's own position did not get better: 3.1 to 3.8 m in the middle in five flights with the gyros' turn, 2.7 to 3.2 m in four without, its worst 5.4 to 6.1 m against 4.4 to 13 m. The flights differ that much among themselves.

No GPS glitch, EKF warning or mode change. GPS 1 back was taken within 10 s.
`GPS_INPUT` went out ten times a second throughout, and the port stayed private.
A picture took about 0.08 s to match on this PC (ORB).

What it took, each found in the simulator on the way:

- A picture's moment must match the angles it is turned with. The stand-in camera first used the moment the truth reached the program (tens of milliseconds late), and at a corner the speeds were 2 to 4 m/s off.
- Those speeds made the EKF reset its heading by 20 degrees ("emergency yaw reset"). Every picture after it was refused as turned too far, and the EKF failsafe landed the drone. Now the matcher follows the heading error.
- Without GPS 1 the EKF's angles were up to 2 degrees off, and that error changed from second to second. With the speed from one second it was up to 4.5 m/s off, the drone weaved and the EKF was up to 22 m off. Over 3 seconds the speed stayed within 1.7 m/s and the EKF within 13 m.
- Forcing the recording to the card (fsync) in the link's thread held `GPS_INPUT` up for 1.8 s. The recording has its own thread now.
- With GPS 1 off the drone tipped 4 to 8 degrees a second (0.2 with GPS 1). The EKF corrects its roll and pitch at each camera fix; those corrections went into the shift between pictures, from there into the speed and back into the EKF. The gyros' turn between the pictures takes them out (see "Step 3b"). The measured shift then had 30 to 50 % less error with GPS 1 off.
- Taken as they come, the gyros' rates made the shift worse in turns (twice the error of no correction): they are about 11 ms late through the gyros' filter.
- A speed from the shift's mean alone, brought to the picture's moment by the flight controller's own speed change, was nearer the truth on paper (0.26 against 0.40 m/s). But it tells the EKF its speed error of a second or more ago as if it were now: flown, the drone tipped 7.9 degrees a second and the EKF was 11 m off. The flight controller's speed plus the shift's correction stays.

### Open for step 3

- Hills: the height above the ground is the flight controller's height above home, corrected by what the matches show (at most 25 percent). Steep ground needs ground heights.
- A drone that is lost (no position for long, more than 400 m off) or starts without GPS needs a search over the whole map: an AI model (for example DINOv2) finds roughly where, then the matching. That model is for the Hailo AI HAT+.
- The Pi camera's pictures for the matching (picamera2) are written but not tried. The matching's speed on the Pi 5 is not measured.
- Water, fog, night and fields without features give no position. Over these the flight controller holds a few seconds, then its EKF failsafe acts.
- Where the map pictures for the client's area come from, and their licence.
- A map of another season gives no position with ORB (see "The map"). XFeat finds it, and in the simulator the drone flew on it, less exactly (the EKF up to 13.7 m off, one check failed). Its speed on the Pi 5 is not measured: at 0.3 s a picture here, the Pi may need fewer points or the Hailo.
- On camera positions the drone still tips 5 to 8 times as much as on GPS 1 (1.1 to 1.6 against 0.2 degrees a second, middle, at two pictures a second with the gyros' turn). Whether the Pi 5 matches two pictures a second is not measured; if not, it skips pictures by itself. The winter map's flight with XFeat (8 degrees a second, one check failed) is not flown again yet.
- The app (step 4): show GPS 2 and where the position comes from.

## Step 2: the matching on recorded flights (vama_nav.replay)

```bash
python -m vama_nav.replay ~/vama-flights/001_2026-11-02_06-10-00Z [...] --map MAP.mbtiles [--out results.csv]
```

Each picture of a recorded flight (`frames.csv` and `images/`, from vama_recorder or from `vama_nav --record`) is matched as it would be in the air.
Its angles and height come from the recorded row.
The search starts up to `--prior-error-m` (30 m) from the recorded position, because the flight controller without GPS is not exact either.
The recorded GPS position is the truth.

It says how often a picture gives a position, how close the positions are, how close the speed between pictures is, why the others gave none, and how long a picture takes on this computer.
It also measures how the camera sits on the drone: a camera that looks a little ahead puts every position ahead of the truth by the height times that angle.
Its suggested `--mount-yaw-deg` comes first; then run it again for `--mount-pitch-deg` and `--mount-roll-deg` (a turned camera also moves the positions sideways while the drone leans).
The other options are those of `vama_nav` (`--features`, `--hfov-deg`, the mount angles, `--match-above-m`, `--search-m`).

## Tests

- `tests/test_gps_denied_recorder.py` (local, like the other tests): the settings, the clock (also a clock that drifts 3 percent), the interpolation (also across north), when to record, the files, whole flights with a simulated drone whose messages arrive 5 to 40 ms late, the Pi camera with a stand-in for picamera2, the bench check.
- `../test/recorder_sitl_test.py`: the recorder as its own program against the ArduCopter simulator (4.6.2 and 4.7.0), a real flight, each picture's moment and state checked against the simulator's own samples on another port.
- `tests/test_gps_denied_nav.py` (local): each GPS_INPUT as ArduPilot decodes it, what is sent when, the settings check, step 3a's stand-in, the link loop on a stand-in link and clock, the command line.
- `../test/gps_input_sitl_test.py`: the sender as its own program against the ArduCopter simulator (4.6.2 and 4.7.0) in real time. Without the sender the drone does not arm. A mission in AUTO: GPS 1 off for two legs of 250 m (flown on the camera's positions), back on, then both gone (the EKF failsafe lands the drone). Also a camera position 150 m off while GPS 1 works (ignored), and the private port.
- `tests/test_gps_denied_match.py` (local; its XFeat test runs only with PyTorch and XFeat): the map file, the camera's geometry (a white spot drawn and turned north up lands where it is), the matching on a made-up aerial map and a second "year" of it, each check against a wrong position, the heading and height that are followed, the track, the stand-in camera and its moments, the source with a simulated flight controller (positions and speeds against the truth), recording, step 2 (also the camera's mount angles), the command lines. Two pictures a second: each fix's weight by the time since the last one, the follow by the second, the movement still over 3 s at five a second, the program's default. The gyros' turn between two pictures: the rates summed (order, ends, gaps), the shift a tilt the drone did not make gives, the measured shift freed of the angles' error, the EKF's tilt corrections kept out of the speed, the bias learnt from the EKF, the rates' lag from INS_GYRO_FILTER, ATTITUDE at 100 Hz.
- `../test/fetch_test_map.py`: downloads the test photos once (USDA NAIP, public domain, east Davis, California, about 10 MB) into `%LOCALAPPDATA%\vama-test-maps`, outside the repo.
- `../test/matching_sitl_test.py`: the matching as its own program against the ArduCopter simulator (4.6.2 and 4.7.0) in real time, at 300 m and 12 m/s over the test photos. The map is the photos of 2020, the stand-in camera sees those of 2022. GPS 1 off for two legs of 800 m, a few seconds of blank ground, GPS 1 back. Then step 2 on the flight it recorded. It also notes how much the drone tipped and how far the EKF's roll and pitch were off (ATTITUDE at 100 Hz on its own port, beside SIM_STATE); `--keep` keeps its folder also when it passes.

Not tried yet on a Raspberry Pi, a Pi camera or a real flight controller.
