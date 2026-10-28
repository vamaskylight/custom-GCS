"""python -m vama_recorder [--config FILE] [--check] [--fake-camera] [--flights N]"""

from __future__ import annotations

import argparse
import logging
import logging.handlers
import os
import signal
import sys
from pathlib import Path

from . import __version__
from .settings import DEFAULT_FILE, SettingsError, load


def _logging(folder: Path) -> None:
    fmt = logging.Formatter("%(asctime)s %(levelname)s %(message)s", "%Y-%m-%d %H:%M:%S")
    root = logging.getLogger()
    root.setLevel(logging.INFO)
    to_screen = logging.StreamHandler(sys.stderr)       # the service's journal
    to_screen.setFormatter(fmt)
    root.addHandler(to_screen)
    try:
        folder.mkdir(parents=True, exist_ok=True)
        to_file = logging.handlers.RotatingFileHandler(folder / "recorder.log", maxBytes=5_000_000, backupCount=3,
                                                       encoding="utf-8")
        to_file.setFormatter(fmt)
        root.addHandler(to_file)
    except OSError as exc:
        logging.getLogger("vama_recorder").warning("no log file in %s: %s", folder, exc)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="vama_recorder",
                                description="Saves the down camera's pictures with the drone's position and "
                                            "angles, during day flights (GPS-denied navigation, step 1).")
    p.add_argument("--config", type=Path, help=f"settings file (default {DEFAULT_FILE}, else built-in defaults)")
    p.add_argument("--check", action="store_true", help="check the camera, the card and the link, then stop")
    p.add_argument("--fake-camera", action="store_true", help="no camera: a stand-in picture (bench runs, tests)")
    p.add_argument("--flights", type=int, default=0, help="stop after this many flights (0: never)")
    p.add_argument("--version", action="version", version=f"vama_recorder {__version__}")
    args = p.parse_args(argv)
    try:
        settings = load(args.config)
    except SettingsError as exc:
        print(f"settings: {exc}", file=sys.stderr)
        return 2

    # MAVLink 2 from the first byte: pymavlink reads this when it is first imported (by .link).
    os.environ.setdefault("MAVLINK20", "1")
    from .camera import CameraError, FakeCamera, PiCamera
    from .link import LinkError
    from .recorder import Recorder, check

    camera = FakeCamera(settings) if args.fake_camera else PiCamera(settings)
    if args.check:
        return check(settings, camera)
    _logging(settings.flights_dir)
    log = logging.getLogger("vama_recorder")
    recorder = Recorder(settings, camera, stop_after_flights=args.flights)

    def stop(signum, _frame) -> None:
        log.info("stopping (signal %s)", signum)
        recorder.stop_event.set()

    for name in ("SIGTERM", "SIGINT"):
        if hasattr(signal, name):
            signal.signal(getattr(signal, name), stop)
    try:
        return recorder.run()
    except CameraError as exc:
        log.error("camera: %s", exc)
        return 3
    except LinkError as exc:
        log.error("link: %s", exc)
        return 4


if __name__ == "__main__":
    sys.exit(main())
