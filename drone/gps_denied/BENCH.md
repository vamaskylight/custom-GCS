# Bench check: how fast the Pi matches pictures

The drone's own position will come from the down camera's pictures, matched with a map.
The program takes two pictures a second.
This check tells whether the Pi 5 is fast enough for that.

It takes about two minutes on the bench. It draws its pictures as from 500 m, the planned height.
It needs no camera, no flight controller and no flight.
Nothing is sent to the drone: the program draws its own pictures from aerial photos and matches them, one after the other, as in flight.

## 1. What you need

- The Pi 5 with the recorder installed (`RECORDER.md`, part 5), and its cooler on.
- From me: the folder `vama_nav`, and two map files, `davis-2020-z17.mbtiles` and `davis-2022-z18.mbtiles` (aerial photos of a town in California, public domain, about 9 MB together).

## 2. Put the files on the Pi

Copy them to the Pi (a USB stick, or `scp`). Then on the Pi:

```bash
sudo apt install -y python3-opencv
sudo cp -r vama_nav /opt/vama-recorder/
mkdir -p ~/vama-test-maps
cp davis-2020-z17.mbtiles davis-2022-z18.mbtiles ~/vama-test-maps/
```

## 3. Run it

```bash
cd /opt/vama-recorder
venv/bin/python -m vama_nav.bench --map ~/vama-test-maps/davis-2020-z17.mbtiles --ground ~/vama-test-maps/davis-2022-z18.mbtiles
```

It prints about ten lines, for example from my PC:

```
positions: 40 of 40, off by middle 0.9 m, 1 in 10 over 1.3, worst 2.3 m
time a picture: middle 0.147 s, 1 in 10 over 0.183, worst 0.205 s
one matching thread: about 6.8 pictures a second
two pictures a second (vama_nav's default): yes
```

The Pi will be slower than my PC. Please send me all the lines it prints (a photo of the screen is fine too).

## 4. Only if I ask: the same with XFeat

XFeat finds the position on a map of another season (for example a winter map for a summer flight). It needs PyTorch, about 200 MB more on the Pi:

```bash
sudo apt install -y git
python3 -m venv --system-site-packages ~/xfeat-venv
~/xfeat-venv/bin/pip install torch tqdm
git clone https://github.com/verlab/accelerated_features ~/xfeat-src
cd /opt/vama-recorder
~/xfeat-venv/bin/python -m vama_nav.bench --map ~/vama-test-maps/davis-2020-z17.mbtiles --ground ~/vama-test-maps/davis-2022-z18.mbtiles --features xfeat --xfeat-dir ~/xfeat-src --pictures 20
```

Please send me those lines too.

## When something is wrong

- "No module named cv2": step 2's first line was not run.
- "No module named vama_nav": run it from `/opt/vama-recorder` (the `cd` line), and check that `/opt/vama-recorder/vama_nav` is there.
- "map: no map file": the map files are not in `~/vama-test-maps`.
- Anything else: send me the whole screen.
