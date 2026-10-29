"""python -m vama_nav --map MAP.mbtiles [--sim-camera GROUND.mbtiles] [--device TCP_OR_SERIAL] ...
python -m vama_nav --sim-truth [--device TCP_OR_SERIAL] [--control FILE] ..."""

from __future__ import annotations

import argparse
import contextlib
import logging
import os
import random
import signal
import statistics
import sys
from pathlib import Path

from . import __version__

MAV_COMP_ID_GPS2 = 221


def _size(text: str) -> tuple[int, int]:
    parts = text.lower().split("x")
    if len(parts) != 2 or not all(p.isdigit() for p in parts) or not all(64 <= int(p) <= 10000 for p in parts):
        raise argparse.ArgumentTypeError(f"a picture size is width x height, for example 2028x1520 (not '{text}')")
    return int(parts[0]), int(parts[1])


def _parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="vama_nav",
                                description="Sends the drone's position from the down camera to the flight controller "
                                            "as its second GPS (GPS-denied navigation, step 3).")
    p.add_argument("--map", type=Path, help="the map on board (MBTiles): match the camera's pictures with it")
    p.add_argument("--sim-camera", type=Path, metavar="GROUND",
                   help="--map: no camera, pictures made from this map (MBTiles, another year) at the ArduCopter "
                        "simulator's true position")
    p.add_argument("--sim-truth", action="store_true",
                   help="no matching: the ArduCopter simulator's true position with noise (step 3a's stand-in)")
    p.add_argument("--device", default="/dev/serial0", help="serial port, or tcp:HOST:PORT (default /dev/serial0)")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--source-system", type=int, default=1, help="the drone's system id (default 1)")
    p.add_argument("--source-component", type=int, default=MAV_COMP_ID_GPS2)
    m = p.add_argument_group("matching (--map)")
    m.add_argument("--map-zoom", type=int, help="the map's zoom level to match with (default: its finest)")
    m.add_argument("--features", default="orb", choices=("orb", "akaze", "sift", "xfeat"),
                   help="default orb (the fastest). xfeat: learned features for a map of another season, needs PyTorch "
                        "and XFeat's folder (see vama_nav/xfeat_match.py)")
    m.add_argument("--xfeat-dir", type=Path, help="--features xfeat: XFeat's folder (default VAMA_XFEAT_DIR)")
    m.add_argument("--hfov-deg", type=float, default=66.0,
                   help="the camera's field of view across the picture (default 66, the Raspberry Pi AI Camera)")
    m.add_argument("--mount-yaw-deg", type=float, default=0.0,
                   help="0: the top of the picture toward the drone's nose, 90: toward its right side")
    m.add_argument("--mount-pitch-deg", type=float, default=0.0, help="the camera looks this far ahead of straight down")
    m.add_argument("--mount-roll-deg", type=float, default=0.0, help="the camera looks this far right of straight down")
    m.add_argument("--interval-s", type=float, default=0.5,
                   help="seconds between two pictures (default 0.5: two a second, each fix then weighs half, and "
                        "the drone tips less; a matching too slow for that skips pictures by itself)")
    p.add_argument("--match-above-m", type=float,
                   help="no positions below this height above home (default 50 m with --map, 15 m with --sim-truth)")
    m.add_argument("--search-m", type=float, default=60.0,
                   help="how far from the flight controller's position the map is searched")
    m.add_argument("--config", type=Path,
                   help="the recorder's settings file: the Pi camera's settings, and with --record where and when")
    m.add_argument("--record", action="store_true",
                   help="also keep the pictures in flight folders, as vama_recorder does (one camera, one program)")
    m.add_argument("--results", type=Path, help="a CSV file with one row per picture: what was found and sent")
    m.add_argument("--picture", type=_size, default=(2028, 1520), help="--sim-camera: picture size (default 2028x1520)")
    m.add_argument("--ground-alt-m", type=float,
                   help="--sim-camera: the ground's height above sea level (default: the simulator's first height)")
    t = p.add_argument_group("the simulator's truth (--sim-truth)")
    t.add_argument("--noise-m", type=float, default=2.0, help="position error, standard deviation")
    t.add_argument("--speed-noise", type=float, default=0.2, help="speed error (m/s), standard deviation")
    t.add_argument("--every-s", type=float, default=1.0, help="seconds between two positions")
    t.add_argument("--delay-s", type=float, default=0.4, help="seconds a position takes to be ready")
    p.add_argument("--control", type=Path, help="a stand-in's JSON file a test changes during the flight")
    p.add_argument("--seed", type=int, help="a stand-in's errors the same each run")
    p.add_argument("--version", action="version", version=f"vama_nav {__version__}")
    return p


def _check(args) -> str | None:
    sources = ("--map MAP.mbtiles (the picture matching, with --sim-camera in the simulator), or --sim-truth "
               "(the simulator's true position)")
    if not args.sim_truth and not args.map:
        return f"no position source: {sources}"
    if args.sim_truth and args.map:
        return f"one position source only: {sources}"
    if args.sim_camera and not args.map:
        return "--sim-camera needs --map"
    if args.match_above_m is None:
        args.match_above_m = 15.0 if args.sim_truth else 50.0
    for name in ("noise_m", "speed_noise", "delay_s", "match_above_m", "mount_pitch_deg", "mount_roll_deg"):
        value = getattr(args, name)
        if name.startswith("mount"):
            if abs(value) > 30:
                return f"--{name.replace('_', '-')} must be from -30 to 30"
        elif value < 0:
            return f"--{name.replace('_', '-')} cannot be below 0"
    if args.every_s <= 0:
        return "--every-s must be more than 0"
    if not 20 <= args.hfov_deg <= 120:
        return "--hfov-deg must be from 20 to 120"
    if not 0.2 <= args.interval_s <= 10:
        return "--interval-s must be from 0.2 to 10"
    if args.search_m <= 0:
        return "--search-m must be more than 0"
    return None


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    problem = _check(args)
    if problem:
        print(problem, file=sys.stderr)
        return 2

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s",
                        datefmt="%Y-%m-%d %H:%M:%S", stream=sys.stderr)
    log = logging.getLogger("vama_nav")
    # MAVLink 2 from the first byte: pymavlink reads this when it is first imported (by .runner).
    os.environ.setdefault("MAVLINK20", "1")
    from pymavlink import mavutil

    from .gps_input import Sender
    from .runner import Runner

    sender = Sender()
    maps = []
    if args.sim_truth:
        from .sim_truth import SimTruth

        source = SimTruth(sender.offer, noise_m=args.noise_m, speed_noise_mps=args.speed_noise, every_s=args.every_s,
                          delay_s=args.delay_s, match_above_m=args.match_above_m, control=args.control,
                          rng=random.Random(args.seed))
        what = (f"positions from the simulator's truth (noise {args.noise_m:.1f} m, one each {args.every_s:.1f} s, "
                f"ready {args.delay_s:.1f} s later, above {args.match_above_m:.0f} m)")
    else:
        import numpy as np

        from vama_recorder.camera import CameraError

        from .geometry import Lens
        from .map_tiles import MapError, TileMap
        from .match_source import MatchSource, SourceSettings
        from .matcher import Matcher, MatchSettings

        from vama_recorder.settings import SettingsError, load
        from vama_recorder.telemetry import Telemetry

        lens = Lens(args.hfov_deg, args.mount_yaw_deg, args.mount_pitch_deg, args.mount_roll_deg)
        try:
            recorder_settings = load(args.config)
        except SettingsError as exc:
            print(f"settings: {exc}", file=sys.stderr)
            return 2
        try:
            tile_map = TileMap(args.map, zoom=args.map_zoom)
            maps.append(tile_map)
            if args.sim_camera:
                from .sim_camera import SimCamera

                ground = TileMap(args.sim_camera)
                maps.append(ground)
                camera = SimCamera(ground, lens, size=args.picture, ground_alt_m=args.ground_alt_m,
                                   control=args.control, rng=np.random.default_rng(args.seed))
            else:
                from vama_recorder.camera import PiCamera

                camera = PiCamera(recorder_settings)
        except MapError as exc:
            print(f"map: {exc}", file=sys.stderr)
            return 2
        from .xfeat_match import XFeatError

        try:
            matcher = Matcher(tile_map, lens, MatchSettings(features=args.features,
                                                            xfeat_dir=str(args.xfeat_dir or "")))
        except XFeatError as exc:
            print(f"xfeat: {exc}", file=sys.stderr)
            _close(None, maps)
            return 2
        telemetry = Telemetry()
        recording = None
        if args.record:
            from .recording import Recording

            recording = Recording(recorder_settings, telemetry, camera.describe, interval_s=args.interval_s)
        source = MatchSource(sender.offer, camera, matcher,
                             SourceSettings(interval_s=args.interval_s, match_above_m=args.match_above_m,
                                            search_m=args.search_m), telemetry=telemetry, results=args.results,
                             recording=recording)
        try:
            source.start()
        except CameraError as exc:
            log.error("camera: %s", exc)
            return 3
        what = (f"positions from the camera's pictures matched with {tile_map.describe()}, {args.features} features, "
                f"a picture each {args.interval_s:g} s above {args.match_above_m:g} m"
                + (f", the stand-in camera on {ground.describe()}" if args.sim_camera else "")
                + (f", recording flights in {recorder_settings.flights_dir} from {recorder_settings.start_height_m:g} m"
                   if args.record else ""))

    try:
        conn = mavutil.mavlink_connection(args.device, baud=args.baud, source_system=args.source_system,
                                          source_component=args.source_component, autoreconnect=True)
    except Exception as exc:              # a serial port that is not there, a refused TCP port
        log.error("cannot open the link %s (%s)", args.device, exc)
        _close(source, maps)
        return 4
    runner = Runner(conn, sender, source)
    log.info("vama_nav %s on %s: GPS_INPUT for GPS 2, %s", __version__, args.device, what)

    def stop(signum, _frame) -> None:
        log.info("stopping (signal %s)", signum)
        runner.stop_event.set()

    for name in ("SIGTERM", "SIGINT"):
        if hasattr(signal, name):
            signal.signal(getattr(signal, name), stop)
    try:
        runner.run()
    finally:
        log.info("sent %d (camera fixes %d, carried forward %d, GPS 1 passed on %d, no fix %d); camera positions "
                 "%d offered, %d refused", sum(runner.sent.values()), runner.sent["camera"], runner.sent["carried"],
                 runner.sent["gps1"], runner.sent["none"], sender.offered, sender.refused)
        counts = getattr(source, "counts", None)
        if counts is not None:
            took = statistics.median(source.match_seconds) if source.match_seconds else 0.0
            log.info("pictures %d, positions sent %d, held by the track %d, no position %d, matching %.2f s a "
                     "picture; no position because: %s", counts["pictures"], counts["sent"], counts["held (track)"],
                     counts["no position"], took,
                     "; ".join(f"{k} ({n})" for k, n in source.no_position.most_common(6)) or "-")
        _close(source, maps)
        with contextlib.suppress(Exception):
            conn.close()
    return 0


def _close(source, maps) -> None:
    close = getattr(source, "close", None)
    if close is not None:
        with contextlib.suppress(Exception):
            close()
    for m in maps:
        with contextlib.suppress(Exception):
            m.close()


if __name__ == "__main__":
    sys.exit(main())
