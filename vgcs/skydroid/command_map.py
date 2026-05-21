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


SKYDROID_PROFILES: dict[str, SkydroidCommandProfile] = {
    "c13_default": SkydroidCommandProfile(
        profile_id="c13_default",
        ptz_commands={
            "up": ["PT_UP"],
            "down": ["PT_DOWN"],
            "left": ["PT_LEFT"],
            "right": ["PT_RIGHT"],
            "center": ["PT_CENTER"],
            "stop": ["PT_STOP"],
        },
        speed_commands=["GSY", "GSP", "GSM"],
        angle_commands=["GAY", "GAP", "GAM"],
        status_commands=["GAA", "GAC", "GAY"],
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
            "stop": ["PTZ_STOP", "PT_STOP"],
        },
        speed_commands=["GSP", "GSY", "GSM"],
        angle_commands=["GAP", "GAY", "GAM"],
        status_commands=["GAC", "GAA", "GAY"],
        status_response_commands=["GAC", "GAA", "GAY", "GAP", "ACK"],
        camera_commands={
            "record_toggle": ["CAM_RECORD", "CAM_REC"],
            "photo": ["CAM_PHOTO", "CAM_SNAP"],
            "zoom": ["CAM_Z", "CAM_ZOOM"],
            "focus_in": ["CAM_FOCUS_NEAR", "CAM_FN", "FOCUS_NEAR"],
            "focus_out": ["CAM_FOCUS_FAR", "CAM_FF", "FOCUS_FAR"],
        },
    ),
}


def get_profile(profile_id: str) -> SkydroidCommandProfile:
    pid = str(profile_id or "").strip().lower()
    return SKYDROID_PROFILES.get(pid, SKYDROID_PROFILES["c13_default"])

