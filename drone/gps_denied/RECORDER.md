# VAMA flight recorder: set-up and use

This is step 1 of the GPS-denied navigation work.
A Raspberry Pi on the drone saves the picture of a camera that looks straight down, once a second, during normal day flights.
With each picture it saves the drone's GPS position, heights, speed and angles at the moment of that picture.
These flights are the data to train and test the picture matching on (step 2).

The recorder does not control the drone. It only listens to the flight controller and asks it for data.

## 1. What you need

| Part | What |
|------|------|
| Computer | Raspberry Pi 5 (yours has 16 GB) with Raspberry Pi OS (64-bit, Bookworm or newer) |
| Camera | A Raspberry Pi camera on the Pi's camera port. Yours is the AI Camera (Sony IMX500): it works here as a normal camera. Camera Module 3 (the normal one, not the Wide), the HQ Camera and the Global Shutter Camera work too |
| Card | Your 64 GB card is fine to start. With the size 2028x1520 (part 6) a picture is about 1 MB: about 15 hours of flight. At the full size it is about 4 MB |
| Power | 5 V and at least 5 A for the Pi, from its own regulator on the drone battery. A cooler on the Pi |
| Wires | Three wires from the Pi to a free serial port of the flight controller (part 4) |

## 2. Mount the camera

- Fixed under the drone, looking straight down. Not on the gimbal.
- On a damped mount, so the pictures are sharp.
- Nothing of the drone in the picture (legs, propellers, the lidar).
- Note which way the top of the picture points. If it points to the drone's nose, `mount_yaw_deg` stays 0. If it points to the right side, set 90 (part 6).
- The AI Camera, the HQ and the Global Shutter camera have a focus ring: set it to far (infinity) by hand before mounting, and check the test picture of part 7. Camera Module 3 focuses by itself (`focus = far`).

## 3. Set up the Pi

1. Turn on the serial port on the Pi's pins (GPIO 14 and 15):
   `sudo raspi-config`, then Interface Options, Serial Port.
   "Login shell over serial": No.
   "Serial port hardware": Yes.
   Reboot.
2. Check that the port is there: `ls -l /dev/serial0` must show a line.
   If not, add the line `dtparam=uart0=on` to `/boot/firmware/config.txt` and reboot.
3. Check the camera: `rpicam-hello --list-cameras` must list it (for Camera Module 3: imx708).

## 4. Wire the Pi to the flight controller

Use a free serial port of the Cube. On the Cube Orange Plus, TELEM1 is SERIAL1, TELEM2 is SERIAL2, GPS1 is SERIAL3 and GPS2 is SERIAL4.
TELEM1 is in use, and TELEM2 is for the RPLidar. Please tell me which port you use.

On a Brahma H7 with ArduPilot, use T3 and R3: that is SERIAL3, so n is 3 below (its SERIAL1 is MAVLink, 2 is GPS, 4 DisplayPort, 6 RC in, 7 MAVLink with flow control, 8 ESC telemetry).
Its 5 V supply gives 3 A for everything on the board, so the Pi must have its own 5 V, 5 A supply.

| Raspberry Pi | Cube port |
|------|------|
| Pin 8 (GPIO 14, TX) | RX |
| Pin 10 (GPIO 15, RX) | TX |
| Pin 6 (ground) | GND |

Do not connect the Cube port's 5 V pin to the Pi. The Pi has its own power.

On the Cube (Mission Planner or QGroundControl, full parameter list), for that port's number n:

| Setting | Value |
|------|------|
| `SERIALn_PROTOCOL` | 2 (MAVLink 2) |
| `SERIALn_BAUD` | 115 (115200) |

Then reboot the Cube.

## 5. Install the recorder

Copy the `gps_denied` folder to the Pi (a USB stick, or `scp`), then on the Pi:

```bash
cd gps_denied
sudo bash install.sh
```

It needs the internet once (it installs `pymavlink`).
Reboot the Pi once after it, so your user may use the serial port and the camera.
From then on the recorder starts by itself at every power on.

## 6. Settings

The settings are in `/etc/vama-recorder.ini`. Each one has a note.
The ones you may change:

| Setting | Default | What |
|------|------|------|
| `device`, `baud` | `/dev/serial0`, `115200` | The serial port, and the same speed as `SERIALn_BAUD` |
| `interval_s` | 1.0 | Seconds between two pictures |
| `size` | full | The picture size. For the AI Camera use `2028x1520`: about 0.3 m of ground per pixel from 500 m, which is enough, and pictures of about 1 MB |
| `focus` | far | Camera Module 3: focus fixed at far. Use `none` for the AI Camera, the HQ or the Global Shutter camera (their focus is set by hand) |
| `mount_yaw_deg` | 0 | Which way the top of the picture points (part 2) |
| `start_height_m` | 30 | Pictures from this height above home, while armed |

After a change: `sudo systemctl restart vama-recorder`.

## 7. Check it on the bench

With the drone powered and the Pi connected, propellers off:

```bash
sudo systemctl stop vama-recorder
cd /opt/vama-recorder && venv/bin/python -m vama_recorder --check
sudo systemctl start vama-recorder
```

Each line starts with OK, NOTE or PROBLEM. The last line says "All checks passed." when all is well.
It also saves one test picture as `~/vama-flights/check.jpg`. Look at it: sharp, and nothing of the drone in it.
"GPS: no 3D fix yet" indoors is normal.

## 8. Fly

- Fly normal day flights. The recorder starts taking pictures when the drone is armed and above 30 m, and stops when it disarms.
- Fly over different ground: fields, villages, roads, water edges. Straight lines and turns.
- Fly at the height you plan to use (500 m) when you can, and some flights lower.
- Each flight is a new folder in `~/vama-flights` on the Pi, for example `003_2026-10-30_09-41-12Z` (the GPS time in UTC).

## 9. After the flight

Copy the flight folders from the Pi (a USB stick, or `scp -r`) and send me one or two first.
In each folder:

| File | What |
|------|------|
| `images/` | The pictures, `000001.jpg` and on |
| `frames.csv` | One row per picture: its time, the drone's position, heights, speed, heading and angles at that moment, the GPS fix, the flight mode |
| `telemetry.jsonl` | All the flight controller's messages of that flight, to work the data again later |
| `flight.json` | The settings, the camera, the start and the end |

Please also tell me the camera model and the Cube port you used.

## 10. When something is wrong

- What the recorder says: `journalctl -u vama-recorder -f`, or the file `~/vama-flights/recorder.log`.
- "no heartbeat from the flight controller": check the three wires (TX to RX, RX to TX), and `SERIALn_PROTOCOL` and `SERIALn_BAUD` on the Cube.
- "no camera found": check the ribbon cable, and `rpicam-hello --list-cameras`.
- "the card is nearly full": copy the flights off the card and delete them on the Pi.
- No pictures in a flight: the drone was below `start_height_m`, or the GPS height was not known yet.
