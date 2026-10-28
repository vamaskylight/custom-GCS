"""Step 2: the matching tried on recorded flights, on a PC.

    python -m vama_nav.replay FLIGHT_FOLDER [...] --map MAP.mbtiles [--out results.csv]

Each picture of a flight recorded by vama_recorder (frames.csv and images/) is matched with the map
as it would be in the air: its angles and height from the flight controller at the picture's
moment, and a search around a position that is off by up to --prior-error-m (the flight controller
without GPS is not exact either). The recorded GPS position is the truth. It says how often a
picture gives a position, how close it is, how close the speed between pictures is, why the
others gave none, and how long a picture takes on this computer.

From the positions' errors it also estimates how the camera sits on the drone: a camera that looks
a little ahead puts every position ahead of the truth by the height times that angle, and a camera
that is turned turns every picture. The suggested --mount-pitch-deg, --mount-roll-deg and
--mount-yaw-deg are those angles. Set the turn first: a turned camera also moves the positions
sideways while the drone leans, so the other two are read again after it.
"""

from __future__ import annotations

import argparse
import csv
import math
import random
import statistics
import sys
import time
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path

import cv2

from .geo import north_east, offset
from .geometry import Lens, Pose
from .map_tiles import MapError, TileMap
from .matcher import FEATURES, Job, Matcher, MatchSettings

OUT_FIELDS = ["flight", "frame", "image", "local_ns", "height_m", "ok", "why", "error_m", "error_n_m", "error_e_m",
              "error_ahead_m", "error_right_m", "inliers", "pairs", "turn_deg", "scale", "horiz_acc_m",
              "speed_error_mps", "seconds"]


@dataclass
class Summary:
    pictures: int = 0
    tried: int = 0                      # high enough, with angles and a position
    matched: int = 0
    errors: list = field(default_factory=list)          # metres
    speed_errors: list = field(default_factory=list)    # m/s
    ahead: list = field(default_factory=list)           # the error toward the nose, over the height
    right: list = field(default_factory=list)
    turns: list = field(default_factory=list)
    scales: list = field(default_factory=list)
    seconds: list = field(default_factory=list)
    why: Counter = field(default_factory=Counter)

    def add(self, other: Summary) -> None:
        for name in ("pictures", "tried", "matched"):
            setattr(self, name, getattr(self, name) + getattr(other, name))
        for name in ("errors", "speed_errors", "ahead", "right", "turns", "scales", "seconds"):
            getattr(self, name).extend(getattr(other, name))
        self.why.update(other.why)

    def lines(self) -> list[str]:
        out = [f"pictures {self.pictures}, high enough to try {self.tried}, a position from {self.matched}"
               + (f" ({100.0 * self.matched / self.tried:.0f} %)" if self.tried else "")]
        if self.errors:
            e = sorted(self.errors)
            out.append(f"position error: middle {statistics.median(e):.1f} m, 95 % within "
                       f"{e[min(len(e) - 1, int(0.95 * len(e)))]:.1f} m, worst {e[-1]:.1f} m")
        if self.speed_errors:
            out.append(f"speed between pictures, error: middle {statistics.median(self.speed_errors):.2f} m/s, "
                       f"worst {max(self.speed_errors):.2f} m/s")
        if len(self.ahead) >= 10:
            pitch = math.degrees(math.atan(statistics.median(self.ahead)))
            roll = math.degrees(math.atan(statistics.median(self.right)))
            yaw = statistics.median(self.turns)
            # A camera that is turned also moves the positions sideways while the drone leans, so
            # the turn is set first, and the other two read again.
            out.append(f"the camera seems turned {yaw:+.2f} degrees: add it to --mount-yaw-deg, then run this again "
                       f"if it is more than 0.5")
            out.append(f"the camera seems to look {pitch:+.2f} degrees ahead and {roll:+.2f} degrees right: add these "
                       f"to --mount-pitch-deg and --mount-roll-deg")
            out.append(f"height above the ground against the flight controller's: x {statistics.median(self.scales):.3f} "
                       "(not 1: hills, or the barometer)")
        if self.seconds:
            out.append(f"matching took {statistics.median(self.seconds):.2f} s a picture on this computer "
                       f"(worst {max(self.seconds):.2f} s)")
        for why, n in self.why.most_common(8):
            out.append(f"no position ({n}): {why}")
        return out


def _number(row: dict, name: str) -> float | None:
    try:
        value = float(row.get(name) or "nan")
    except ValueError:
        return None
    return value if math.isfinite(value) else None


def replay(folder: Path, matcher: Matcher, match_above_m: float = 50.0, prior_error_m: float = 30.0,
           search_m: float = 60.0, rng: random.Random | None = None, rows_out=None) -> Summary:
    """Match every picture of one recorded flight. ``rows_out``: a csv.writer for OUT_FIELDS, or None."""
    rng = rng or random.Random(0)
    s = Summary()
    seen: dict[int, tuple[float, float]] = {}               # recent pictures tried: local_ns: (lat, lon)
    matcher.forget()
    with (folder / "frames.csv").open(encoding="utf-8", newline="") as f:
        frames = list(csv.DictReader(f))
    for row in frames:
        s.pictures += 1
        values = {k: _number(row, k) for k in ("lat", "lon", "rel_alt_m", "roll_deg", "pitch_deg", "yaw_deg")}
        local_ns = int(row["local_ns"])
        if any(v is None for v in values.values()) or values["rel_alt_m"] < match_above_m:
            seen.clear()
            matcher.forget()
            continue
        image = cv2.imread(str(folder / row["image"]), cv2.IMREAD_GRAYSCALE)
        if image is None:
            s.why["the picture file cannot be read"] += 1
            continue
        s.tried += 1
        lat, lon = values["lat"], values["lon"]
        r = rng.uniform(0.0, prior_error_m)
        a = rng.uniform(0.0, 2.0 * math.pi)
        prior = offset(lat, lon, r * math.cos(a), r * math.sin(a))
        pose = Pose(math.radians(values["roll_deg"]), math.radians(values["pitch_deg"]),
                    math.radians(values["yaw_deg"]), values["rel_alt_m"])
        result = matcher.process(Job(local_ns, image, prior[0], prior[1], pose, search_m))
        s.seconds.append(result.seconds)
        out = {"flight": folder.name, "frame": row.get("frame", ""), "image": row["image"], "local_ns": local_ns,
               "height_m": f"{values['rel_alt_m']:.1f}", "ok": int(result.ok), "why": result.why,
               "inliers": result.inliers, "pairs": result.pairs, "seconds": f"{result.seconds:.3f}"}
        mv = result.movement
        if mv is not None and mv.since_ns in seen:
            dt = (local_ns - mv.since_ns) / 1e9
            tn, te = north_east(*seen[mv.since_ns], lat, lon)
            speed_error = math.hypot(mv.north_m - tn, mv.east_m - te) / dt
            s.speed_errors.append(speed_error)
            out["speed_error_mps"] = f"{speed_error:.2f}"
        seen[local_ns] = (lat, lon)
        for old in [t for t in seen if t < local_ns - 10 * 1_000_000_000]:
            del seen[old]
        if result.ok:
            s.matched += 1
            en, ee = north_east(lat, lon, result.lat, result.lon)
            error = math.hypot(en, ee)
            yaw = pose.yaw
            ahead = en * math.cos(yaw) + ee * math.sin(yaw)
            right = -en * math.sin(yaw) + ee * math.cos(yaw)
            s.errors.append(error)
            s.ahead.append(ahead / pose.height_m)
            s.right.append(right / pose.height_m)
            s.turns.append(result.turn_deg)
            s.scales.append(result.scale)
            out.update(error_m=f"{error:.2f}", error_n_m=f"{en:.2f}", error_e_m=f"{ee:.2f}", error_ahead_m=f"{ahead:.2f}",
                       error_right_m=f"{right:.2f}", turn_deg=f"{result.turn_deg:.2f}", scale=f"{result.scale:.4f}",
                       horiz_acc_m=f"{result.horiz_acc_m:.2f}")
        else:
            s.why[result.why.split(" (")[0]] += 1
        if rows_out is not None:
            rows_out.writerow([out.get(k, "") for k in OUT_FIELDS])
    return s


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="vama_nav.replay",
                                description="Tries the picture matching on flights recorded by vama_recorder (step 2).")
    p.add_argument("flights", nargs="+", type=Path, help="flight folders (each with frames.csv and images/)")
    p.add_argument("--map", type=Path, required=True, help="the map (MBTiles) of the flights' area")
    p.add_argument("--map-zoom", type=int, help="the map's zoom level to match with (default: its finest)")
    p.add_argument("--features", default="orb", choices=FEATURES)
    p.add_argument("--hfov-deg", type=float, default=66.0, help="the camera's field of view across the picture")
    p.add_argument("--mount-yaw-deg", type=float, default=0.0)
    p.add_argument("--mount-pitch-deg", type=float, default=0.0)
    p.add_argument("--mount-roll-deg", type=float, default=0.0)
    p.add_argument("--match-above-m", type=float, default=50.0, help="pictures below this height are not tried")
    p.add_argument("--prior-error-m", type=float, default=30.0,
                   help="the search starts up to this far from the recorded position")
    p.add_argument("--search-m", type=float, default=60.0, help="how far from that start the map is searched")
    p.add_argument("--out", type=Path, help="a CSV file with one row per picture")
    p.add_argument("--seed", type=int, default=0)
    args = p.parse_args(argv)
    if args.prior_error_m > args.search_m:
        print("--prior-error-m cannot be more than --search-m: the right place would be outside the search",
              file=sys.stderr)
        return 2
    missing = [str(f) for f in args.flights if not (f / "frames.csv").is_file()]
    if missing:
        print(f"no frames.csv in: {', '.join(missing)}", file=sys.stderr)
        return 2
    try:
        tile_map = TileMap(args.map, zoom=args.map_zoom)
    except MapError as exc:
        print(f"map: {exc}", file=sys.stderr)
        return 2
    matcher = Matcher(tile_map, Lens(args.hfov_deg, args.mount_yaw_deg, args.mount_pitch_deg, args.mount_roll_deg),
                      MatchSettings(features=args.features))
    rng = random.Random(args.seed)
    out_file = open(args.out, "w", newline="", encoding="utf-8") if args.out else None  # noqa: SIM115
    rows = csv.writer(out_file) if out_file else None
    if rows:
        rows.writerow(OUT_FIELDS)
    total = Summary()
    began = time.monotonic()
    try:
        print(f"map: {tile_map.describe()}, {args.features} features")
        for folder in args.flights:
            s = replay(folder, matcher, args.match_above_m, args.prior_error_m, args.search_m, rng, rows)
            total.add(s)
            print(f"\n{folder.name}")
            for line in s.lines():
                print(f"  {line}")
        if len(args.flights) > 1:
            print("\nall flights")
            for line in total.lines():
                print(f"  {line}")
        print(f"\n{time.monotonic() - began:.0f} s")
    finally:
        if out_file:
            out_file.close()
        tile_map.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
