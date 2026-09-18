from __future__ import annotations

from dataclasses import dataclass

# How the GCS knows the camera's zoom, which decides how far the pixel<->angle
# aim math can be trusted once the operator zooms:
#  - "commanded_optical": C12/C13. The camera reports nothing back, so the FOV
#    is the wide-angle figure narrowed by the level we last commanded
#    (vgcs/observe/camera_fov.py).
#  - "dzm_step": C14 Pro. Zoom is a DZM *step* (TOP V1.2.0 section 4.6), not a
#    multiplier: steps [0, 70) are the short-focus lens and [70, 140] the
#    long-focus lens. The camera answers a DZM read with its current step, so
#    unlike the C13 the state is observable - but the step->magnification
#    formula for this model is blank in the vendor document.
ZOOM_MODEL_COMMANDED_OPTICAL = "commanded_optical"
ZOOM_MODEL_DZM_STEP = "dzm_step"


@dataclass(frozen=True)
class SkydroidLens:
    """One visible-light lens of a multi-lens gimbal, at its native (uncropped) FOV.

    ``dzm_step_min``/``dzm_step_max`` are the inclusive DZM step range the vendor
    document assigns to this lens.
    """

    name: str
    fov_h_deg: float
    fov_v_deg: float
    dzm_step_min: int
    dzm_step_max: int


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
    # --- Fields below were added for C14 Pro. Every default reproduces the
    # C12/C13 behaviour exactly, so the existing profiles do not set them. ---
    display_name: str = ""
    # When True the adapter's LRF/M13/DOOAF aim math uses fov_h_deg/fov_v_deg
    # from this profile. False keeps the adapter's legacy module constants
    # (83.4/46.9), which are field-validated on C13 and which C12 has always
    # run on - see DOCS/SKYDROID-CAMERA-SPECS.md before flipping it for those.
    aim_fov_from_profile: bool = False
    zoom_model: str = ZOOM_MODEL_COMMANDED_OPTICAL
    # Visible lenses in DZM step order; empty for single-lens cameras.
    lenses: tuple[SkydroidLens, ...] = ()
    # False = the FOV at any zoom other than the first lens's first step is
    # unknown, so video picks are refused there instead of silently converting
    # a click with the wrong angle (a wrong DOOAF coordinate looks normal).
    zoom_fov_confirmed: bool = True
    # True when the camera applies zoom in its own encoder, so the RTSP picture
    # is already magnified and the preview must not crop on top of it.
    preview_carries_zoom: bool = False
    thermal_fov_h_deg: float | None = None
    thermal_fov_v_deg: float | None = None
    # Longest SLR reading accepted as real. 1000 m is the C13 laser and the
    # documented SLR field limit (0x2710 dm).
    laser_max_m: float = 1000.0
    # True = variable-length gimbal frames (GSM/GAY/GAP/GAM/GOT) go out with the
    # upper-case #TP header the vendor document requires for all G-class frames.
    # False = the lower-case #tp VGCS has always sent, which C13 accepts.
    g_frames_upper_header: bool = False
    # Profile the endpoint probe retries with when this one gets no attitude.
    # "" disables the retry - it must for any camera whose optics differ from
    # C13, or a successful retry would swap this camera onto C13's lens.
    probe_fallback_profile_id: str = "c13_alt"

    def lens_for_dzm_step(self, step: int | None) -> SkydroidLens | None:
        """The lens a DZM step selects, or None if unknown / single-lens."""
        if step is None:
            return None
        try:
            n = int(step)
        except (TypeError, ValueError):
            return None
        for lens in self.lenses:
            if int(lens.dzm_step_min) <= n <= int(lens.dzm_step_max):
                return lens
        return None

    @property
    def zoom_step_max(self) -> int:
        """Highest DZM step across all lenses (0 for single-lens cameras)."""
        return max((int(lens.dzm_step_max) for lens in self.lenses), default=0)


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
    "c14pro_default": SkydroidCommandProfile(
        profile_id="c14pro_default",
        display_name="C14 Pro",
        # TOP V1.2.0 (2026-07-21) lists C14PRO on PTZ, GSY/GSP/GSM, GAY/GAP/GAM,
        # GAA/GAC, GOT/SUM, REC, CAP, DZM, IMG, TSM, TAS, SLR, MOD, IPV, VOM and
        # VER - the same tags and frame formats C13 uses, so the command lists
        # are C13's. Nothing here has been run against real C14 Pro hardware yet.
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
        # Vendor C14 Pro spec table: transmission 1280x720; short-focus lens
        # (5.4 mm) 61.4/47.9/73.8 deg; long-focus lens (25 mm) 14.7/11.1/18.3.
        # NOT the plain C14, whose short lens is 80/61. Datasheet values, not an
        # as-built calibration like C13's 83.4/46.9 - expect to refine them
        # from the first field logs.
        frame_w=1280,
        frame_h=720,
        fov_h_deg=61.4,
        fov_v_deg=47.9,
        frame_specs_confirmed=True,
        aim_fov_from_profile=True,
        zoom_model=ZOOM_MODEL_DZM_STEP,
        lenses=(
            SkydroidLens("short", 61.4, 47.9, 0, 69),
            SkydroidLens("long", 14.7, 11.1, 70, 140),
        ),
        # The C14 line of the DZM step->magnification formula is empty in
        # V1.2.0 (asked of Skydroid 2026-09-17). Flip to True only together
        # with code that turns a step into a field of view.
        zoom_fov_confirmed=False,
        preview_carries_zoom=True,
        thermal_fov_h_deg=32.84,
        thermal_fov_v_deg=26.35,
        # Laser is specified to 1200 m ("≥1200 m" on a building), past the
        # 1000 m the SLR section documents. The 4-hex-char field can carry it;
        # allow some headroom so a real long reading is not thrown away.
        laser_max_m=1500.0,
        # Follow the document to the letter on a camera nobody here has driven
        # yet; VGCS_TOP_G_HEADER=lower reverts it in the field.
        g_frames_upper_header=True,
        probe_fallback_profile_id="",
    ),
}


def get_profile(profile_id: str) -> SkydroidCommandProfile:
    pid = str(profile_id or "").strip().lower()
    return SKYDROID_PROFILES.get(pid, SKYDROID_PROFILES["c13_default"])

