"""Parse observation HTML and recompute DOOAF distances from session logs."""
from __future__ import annotations

import math
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from vgcs.observe._dooaf_correction import build_dooaf_session, compute_fire_correction
from vgcs.observe._dooaf_types import DOOAF_ROLE_GUN, DOOAF_ROLE_IMPACT, DOOAF_ROLE_INTENDED, GeoPoint
from vgcs.observe.target_measure import haversine_m


def main() -> None:
    html_path = Path(r"c:\Users\Miny\Downloads\observations_20260630_175954.html")
    html = html_path.read_text(encoding="utf-8")
    dists = re.findall(r"gun.{0,5}target \d+\.\d+ m|gun.{0,5}impact \d+\.\d+ m|target.{0,5}impact \d+\.\d+ m", html)
    print("HTML distances:", dists)
    heroes = re.findall(r"hero-kpi-val'>([^<]+)", html)
    print("Hero KPIs:", heroes)

    gun = GeoPoint(20.4095884, 72.8797578, 13.759)
    target = GeoPoint(20.4096783, 72.8796854, None)
    impact = GeoPoint(20.4096776, 72.8796511, None)
    print("Haversine gun-target:", haversine_m(gun.lat, gun.lon, target.lat, target.lon))
    print("Haversine gun-impact:", haversine_m(gun.lat, gun.lon, impact.lat, impact.lon))
    print("Haversine target-impact:", haversine_m(target.lat, target.lon, impact.lat, impact.lon))

    setup_marks = {
        DOOAF_ROLE_GUN: (0.5, 0.5),
        DOOAF_ROLE_INTENDED: (0.640, 0.156),
    }
    impact_row = {
        "kind": "video_mark",
        "dooaf_role": DOOAF_ROLE_IMPACT,
        "target_lat": 20.409677557865766,
        "target_lon": 72.87965108172915,
        "video_x_norm": 0.6839622641509434,
        "video_y_norm": 0.24731182795698925,
        "geo_quality": "fair",
        "geo_method": "lrf_facade_uv",
        "gimbal_yaw_deg": -8.0,
        "gimbal_pitch_deg": -15.0,
        "vehicle_lat": 20.41009,
        "vehicle_lon": 72.87993,
        "vehicle_heading_deg": 215.0,
        "vehicle_alt_msl_m": 25.0,
        "ekf_rel_alt_m": 11.795,
        "lrf_slant_range_m": 52.9,
    }
    session = build_dooaf_session(
        [impact_row],
        gun_lat=gun.lat,
        gun_lon=gun.lon,
        gun_alt_m=gun.alt_m,
        target_lat=target.lat,
        target_lon=target.lon,
        setup_video_marks=setup_marks,
    )
    c = session.correction
    if c:
        print("Session correction:")
        print(f"  gun->target: {c.range_gun_to_intended_m:.2f} m")
        print(f"  gun->impact: {c.range_gun_to_impact_m:.2f} m")
        print(f"  target->impact: {c.impact_to_intended_m:.2f} m")
        print(f"  miss horizontal: {math.hypot(c.miss_east_m, c.miss_north_m):.2f} m")


if __name__ == "__main__":
    main()
