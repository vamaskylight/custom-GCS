"""One flight's folder.

    NNN_<time>/
      images/000001.jpg ...      the down camera's pictures
      frames.csv                 one row per picture: its time, and the drone's state at that moment
      telemetry.jsonl            every flight controller message of interest, one JSON object a line,
                                 with "t_ns", this computer's clock when it arrived
      flight.json                the settings, the camera, the controller, the start and the end

frames.csv is written as the flight goes and flushed often, so a power cut loses at most a few rows.
A row waits SETTLE_NS after its picture, and until the controller's samples after the picture's
moment have arrived (at most WAIT_NS), so its moment and its state are interpolated, not guessed.
"""

from __future__ import annotations

import csv
import json
import os
import socket
from datetime import datetime
from pathlib import Path

from . import __version__

SETTLE_NS = 1_000_000_000      # the clock estimate then has messages on both sides of the picture
WAIT_NS = 2_500_000_000        # and no row waits longer than this
FSYNC_EVERY = 10

FRAME_FIELDS = [
    "frame", "image", "local_ns", "ap_boot_ms", "utc",
    "lat", "lon", "alt_msl_m", "rel_alt_m", "vn_mps", "ve_mps", "vd_mps", "heading_deg",
    "roll_deg", "pitch_deg", "yaw_deg",
    "position_how", "position_gap_ms", "attitude_how", "attitude_gap_ms",
    "exposure_us", "gain", "lens_position",
    "gps_fix", "gps_sats", "gps_hdop", "rangefinder_m", "armed", "mode",
]
_DECIMALS = {"lat": 7, "lon": 7, "ap_boot_ms": 1, "gps_hdop": 2, "gain": 3, "lens_position": 3}

LOGGED_MESSAGES = {"HEARTBEAT", "ATTITUDE", "GLOBAL_POSITION_INT", "GPS_RAW_INT", "SYSTEM_TIME",
                   "DISTANCE_SENSOR", "RANGEFINDER", "VFR_HUD", "EKF_STATUS_REPORT", "STATUSTEXT"}


def _cell(name: str, value) -> str:
    if value is None:
        return ""
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, float):
        return f"{value:.{_DECIMALS.get(name, 3)}f}"
    return str(value)


class Flight:
    def __init__(self, root: Path, name: str, info: dict) -> None:
        self.folder = Path(root) / name
        (self.folder / "images").mkdir(parents=True)
        self.info = dict(info)
        self.info.update(name=name, recorder_version=__version__, host=socket.gethostname(),
                         started_local=datetime.now().astimezone().isoformat(timespec="seconds"),
                         frames=0, rows=0)
        self._frames_file = open(self.folder / "frames.csv", "w", newline="", encoding="utf-8")
        self._rows = csv.writer(self._frames_file)
        self._rows.writerow(FRAME_FIELDS)
        self._frames_file.flush()
        self._telemetry_file = open(self.folder / "telemetry.jsonl", "w", encoding="utf-8")
        self.pending: list[dict] = []
        self.frames = 0
        self.rows = 0
        self.closed = False
        self._write_info()

    @staticmethod
    def next_name(root: Path, stamp: str) -> str:
        """001_<stamp>, or the next number after the flights already in ``root``."""
        last = 0
        root = Path(root)
        if root.is_dir():
            for p in root.iterdir():
                head = p.name.split("_", 1)[0]
                if p.is_dir() and head.isdigit():
                    last = max(last, int(head))
        return f"{last + 1:03d}_{stamp}"

    def image_path(self, extension: str) -> tuple[Path, str]:
        """The next picture's file, and its name as frames.csv gives it."""
        self.frames += 1
        name = f"{self.frames:06d}.{extension}"
        return self.folder / "images" / name, f"images/{name}"

    def add_frame(self, image: str, shot, snapshot: dict) -> None:
        """A picture taken: its row waits for the controller's samples around its moment.

        ``snapshot``: what was known when it was taken (GPS fix, rangefinder, armed, mode)."""
        row = {"frame": self.frames, "image": image, "local_ns": int(shot.sensor_ns),
               "exposure_us": shot.exposure_us, "gain": shot.gain, "lens_position": shot.lens_position}
        row.update(snapshot)
        self.pending.append(row)

    def finalize(self, telemetry, now_ns: int, force: bool = False) -> int:
        """Write the rows whose moment the controller's samples now cover (all of them with ``force``)."""
        written, keep = 0, []
        for row in self.pending:
            boot = telemetry.boot_ms_at(row["local_ns"])
            age = now_ns - row["local_ns"]
            ready = age >= SETTLE_NS and boot is not None and telemetry.covered(boot)
            if not (force or ready or age > WAIT_NS):
                keep.append(row)
                continue
            if boot is not None:
                row.update(telemetry.state_at(boot), ap_boot_ms=boot, utc=telemetry.utc_at(boot))
            self._rows.writerow([_cell(k, row.get(k)) for k in FRAME_FIELDS])
            written += 1
        self.pending = keep
        if written:
            self.rows += written
            self._frames_file.flush()
            if self.rows % FSYNC_EVERY < written:
                os.fsync(self._frames_file.fileno())
        return written

    def log_message(self, msg, arrival_ns: int) -> None:
        if self.closed or msg.get_type() not in LOGGED_MESSAGES:
            return
        d = msg.to_dict()
        d["t_ns"] = int(arrival_ns)
        self._telemetry_file.write(json.dumps(d, separators=(",", ":"), default=str) + "\n")

    def close(self, telemetry, now_ns: int, reason: str, extra: dict | None = None) -> None:
        if self.closed:
            return
        self.finalize(telemetry, now_ns, force=True)
        self.closed = True
        self.info.update(frames=self.frames, rows=self.rows, end_reason=reason,
                         ended_local=datetime.now().astimezone().isoformat(timespec="seconds"))
        self.info.update(extra or {})
        for f in (self._frames_file, self._telemetry_file):
            f.flush()
            os.fsync(f.fileno())
            f.close()
        self._write_info()

    def _write_info(self) -> None:
        tmp = self.folder / "flight.json.tmp"
        tmp.write_text(json.dumps(self.info, indent=1, default=str) + "\n", encoding="utf-8")
        os.replace(tmp, self.folder / "flight.json")
