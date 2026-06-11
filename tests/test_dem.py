"""M8 DEM terrain intersection."""

from __future__ import annotations

import tempfile
from pathlib import Path

from vgcs.observe.dem import (
    load_dem_model,
    ray_intersect_terrain_msl,
)
from vgcs.observe.geo_reference import compute_geo_reference


def test_ray_intersect_flat_terrain():
    def elev(_lat: float, _lon: float) -> float:
        return 100.0

    hit = ray_intersect_terrain_msl(
        vehicle_lat=20.445,
        vehicle_lon=72.863,
        vehicle_alt_msl_m=110.0,
        dir_ned=(0.6, 0.0, 0.8),
        elevation_m=elev,
        max_range_m=500.0,
        step_m=2.0,
    )
    assert hit is not None
    _n, _e, _r, terr = hit
    assert abs(terr - 100.0) < 1.0


def test_load_csv_dem():
    with tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False, encoding="utf-8") as f:
        f.write("lat,lon,elev_m\n20.445,72.863,100.0\n")
        path = f.name
    try:
        model = load_dem_model(path)
        assert model is not None
        assert model.elevation_m(20.445, 72.863) == 100.0
    finally:
        Path(path).unlink(missing_ok=True)


def test_geo_terrain_dem_method():
    with tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False, encoding="utf-8") as f:
        f.write("lat,lon,elev_m\n20.4458,72.8630,50.0\n")
        path = f.name
    try:
        r = compute_geo_reference(
            vehicle_lat=20.4458,
            vehicle_lon=72.8630,
            vehicle_heading_deg=0.0,
            vehicle_rel_alt_m=15.0,
            vehicle_alt_msl_m=65.0,
            gimbal_yaw_deg=0.0,
            gimbal_pitch_deg=-25.0,
            video_x_norm=0.5,
            video_y_norm=0.65,
            gps_fix_type=3,
            dem_path=path,
            dem_terrain=True,
        )
        assert r.target_lat is not None
        assert r.method in ("ray_terrain_dem", "ray_ground_dem", "ray_ground_flat")
    finally:
        Path(path).unlink(missing_ok=True)
