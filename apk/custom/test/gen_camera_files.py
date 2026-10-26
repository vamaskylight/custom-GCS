"""Writes camera files with VGCS's writer, for the check that both apps read each other's files.

Task AA: a camera added in VGCS works in the app once its file is copied to the remote
control, and the other way round. This writes sample cameras the way VGCS saves them
(vgcs/skydroid/camera_files.py) into camera_files/vgcs. camera_profile_test (link/) reads
them with the app's reader, and writes the app's own files of the same cameras into
camera_files/apk when VAMA_WRITE_CAMERA_FILES=1. VGCS's test tests/test_camera_files_cross_app.py
reads those with VGCS's reader, and checks that this script still writes camera_files/vgcs.

Run from the repository root:  py -3.14 apk/custom/test/gen_camera_files.py
"""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))

from vgcs.skydroid.camera_files import (  # noqa: E402
    ZOOM_NONE,
    Camera,
    CameraStore,
    Lens,
    apk_built_in_cameras,
    built_in_cameras,
)

FOLDER = Path(__file__).resolve().parent / "camera_files"


def sample_cameras() -> list[Camera]:
    """Every kind of camera: copies of the built-in ones, and cameras with other values in every field.

    The app's Viewpro and SIYI cameras (task AK) are files VGCS writes and reads but does not use.
    """
    c12, c13, c14, c20 = built_in_cameras()
    a20ktr, zr10 = apk_built_in_cameras()
    return [
        replace(c12, name="Copy of V12"),
        replace(c13, name="Copy of V13"),
        replace(c14, name="Copy of V14 Pro"),
        replace(c20, name="Copy of V20"),  # TOP over TCP
        replace(a20ktr, name="Copy of A20KTR"),  # Viewpro
        replace(zr10, name="Copy of ZR10"),  # SIYI
        replace(
            c13, name="Bare camera", host="10.0.0.5", port=6000, day_video_url="rtsp://10.0.0.5/live",
            thermal_video_url="", laser=False, photo=False, record=False, own_tracker=False,
        ),
        replace(
            c14, name="Three lenses",
            lenses=(Lens(70.0, 52.5, 0, 40), Lens(30.5, 22.9, 41, 120), Lens(8.25, 6.2, 121, 200)),
            fov_h_deg=70.0, fov_v_deg=52.5, sensor_w=4000, sensor_h=3000, crop_step_w=10, crop_step_h=7,
        ),
        replace(c13, name="Fixed lens", zoom=ZOOM_NONE, fov_h_deg=95.0, fov_v_deg=55.5),
        replace(
            c13, name="Odd numbers", fov_h_deg=61.25, fov_v_deg=34.45, max_zoom=12.5,
            thermal_fov_h_deg=32.84, thermal_fov_v_deg=26.35, thermal_fov_sure=True,
            pitch_min_deg=-120.0, pitch_max_deg=45.0, yaw_limit_deg=170.0, laser_max_m=2500.0,
            upper_gimbal_header=True,
        ),
    ]


def write_vgcs_files(folder: Path) -> list[str]:
    """The sample cameras as VGCS saves them, in a folder of their own: their ids."""
    folder.mkdir(parents=True, exist_ok=True)
    for old in folder.glob("*.json"):
        old.unlink()
    store = CameraStore(folder)
    store.load()
    ids = []
    for cam in sample_cameras():
        why, saved = store.save(cam)
        if why:
            raise SystemExit(f"{cam.name}: {why}")
        ids.append(saved)
    return ids


if __name__ == "__main__":
    ids = write_vgcs_files(FOLDER / "vgcs")
    print(f"{len(ids)} camera files in {FOLDER / 'vgcs'}: {', '.join(ids)}")
