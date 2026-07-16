from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class SkydroidCommandProfile:
    profile_id: str
    ptz_commands: dict[str, list[str]]
    speed_commands: list[str]
    angle_commands: list[str]
    status_commands: list[str]
    status_response_commands: list[str]
    camera_commands: dict[str, list[str]]
    # Video frame size (px) and lens field of view (deg) used by the M13/LRF
    # pixel<->angle aim math. Defaults are C13's — 83.4/46.9, NOT the
    # datasheet's nominal 77.4/48.8, because 83.4/46.9 is load-bearing for
    # real field-log regression tests (see adapter.py's _LRF_FOV_H_DEG
    # comment) and is very likely an as-built calibration, not a guess.
    # frame_specs_confirmed=False marks a profile whose frame/FOV numbers are
    # an unconfirmed placeholder, not a measured or vendor-confirmed value for
    # this model — aim accuracy on that camera should not be trusted until
    # it's flipped to True.
    frame_w: int = 1280
    frame_h: int = 720
    fov_h_deg: float = 83.4
    fov_v_deg: float = 46.9
    frame_specs_confirmed: bool = True


SKYDROID_PROFILES: dict[str, SkydroidCommandProfile] = {
    "c13_default": SkydroidCommandProfile(
        profile_id="c13_default",
        ptz_commands={
            "up": ["PT_UP"],
            "down": ["PT_DOWN"],
            "left": ["PT_LEFT"],
            "right": ["PT_RIGHT"],
            "center": ["PT_CENTER"],
            "nadir": ["PTZ_NADIR"],
            "stop": ["PT_STOP"],
        },
        speed_commands=["GSY", "GSP", "GSM"],
        angle_commands=["GAY", "GAP", "GAM"],
        status_commands=["GAC", "GAA"],
        status_response_commands=["GAA", "GAC", "GAY", "GAP", "ACK"],
        camera_commands={
            "record_toggle": ["CAM_REC", "CAM_RECORD"],
            "photo": ["CAM_SNAP", "CAM_PHOTO"],
            "zoom": ["CAM_ZOOM", "CAM_Z"],
            "focus_in": ["CAM_FOCUS_NEAR", "CAM_FN", "FOCUS_NEAR"],
            "focus_out": ["CAM_FOCUS_FAR", "CAM_FF", "FOCUS_FAR"],
        },
    ),
    "c13_alt": SkydroidCommandProfile(
        profile_id="c13_alt",
        ptz_commands={
            "up": ["PTZ_UP", "PT_UP"],
            "down": ["PTZ_DOWN", "PT_DOWN"],
            "left": ["PTZ_LEFT", "PT_LEFT"],
            "right": ["PTZ_RIGHT", "PT_RIGHT"],
            "center": ["PTZ_CENTER", "PT_CENTER"],
            "nadir": ["PTZ_NADIR", "PT_NADIR"],
            "stop": ["PTZ_STOP", "PT_STOP"],
        },
        speed_commands=["GSP", "GSY", "GSM"],
        angle_commands=["GAP", "GAY", "GAM"],
        status_commands=["GAC", "GAA"],
        status_response_commands=["GAC", "GAA", "GAY", "GAP", "ACK"],
        camera_commands={
            "record_toggle": ["CAM_RECORD", "CAM_REC"],
            "photo": ["CAM_PHOTO", "CAM_SNAP"],
            "zoom": ["CAM_Z", "CAM_ZOOM"],
            "focus_in": ["CAM_FOCUS_NEAR", "CAM_FN", "FOCUS_NEAR"],
            "focus_out": ["CAM_FOCUS_FAR", "CAM_FF", "FOCUS_FAR"],
        },
    ),
    "c12_default": SkydroidCommandProfile(
        profile_id="c12_default",
        # Skydroid's TOP Protocol doc lists GSY/GSP/GAY/GAP/GAM/PTZ as shared
        # across the C10/C10Pro/C12/C20 gimbal family, so the C13 command tags
        # carry over. GOT (target lock) and SUM (track confirm) are the ones
        # actually documented as C12-only per that same spec.
        ptz_commands={
            "up": ["PT_UP"],
            "down": ["PT_DOWN"],
            "left": ["PT_LEFT"],
            "right": ["PT_RIGHT"],
            "center": ["PT_CENTER"],
            "nadir": ["PTZ_NADIR"],
            "stop": ["PT_STOP"],
        },
        speed_commands=["GSY", "GSP", "GSM"],
        angle_commands=["GAY", "GAP", "GAM"],
        status_commands=["GAC", "GAA"],
        status_response_commands=["GAA", "GAC", "GAY", "GAP", "ACK"],
        camera_commands={
            "record_toggle": ["CAM_REC", "CAM_RECORD"],
            "photo": ["CAM_SNAP", "CAM_PHOTO"],
            "zoom": ["CAM_ZOOM", "CAM_Z"],
            "focus_in": ["CAM_FOCUS_NEAR", "CAM_FN", "FOCUS_NEAR"],
            "focus_out": ["CAM_FOCUS_FAR", "CAM_FF", "FOCUS_FAR"],
        },
        # Per Skydroid's C12 datasheet: visible-light HFOV/VFOV 100/52 deg,
        # image transmission resolution 1280x720 (same transmission size as
        # C13, despite a wider lens and higher record/photo resolution).
        frame_w=1280,
        frame_h=720,
        fov_h_deg=100.0,
        fov_v_deg=52.0,
        frame_specs_confirmed=True,
    ),
}


def get_profile(profile_id: str) -> SkydroidCommandProfile:
    pid = str(profile_id or "").strip().lower()
    return SKYDROID_PROFILES.get(pid, SKYDROID_PROFILES["c13_default"])

