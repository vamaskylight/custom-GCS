from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Mapping


# --- Yunzhuo / Skydroid TOP (#TP) per PROTOCAL.doc (UDP port 5000) ---


def tp_checksum(body_without_crc: str) -> str:
    """ASCII sum of all bytes before checksum, as two uppercase hex digits."""
    total = sum(ord(ch) for ch in body_without_crc) & 0xFF
    return f"{total:02X}"


def build_tp_frame(
    *,
    dest: str,
    control: str,
    tag: str,
    data: str = "",
    src: str = "U",
    variable: bool | None = None,
) -> bytes:
    """
    Build a TOP frame (#TP fixed or #tp variable).

    Example: build_tp_frame(dest="G", control="w", tag="GAA", data="01")
             -> b"#TPUG2wGAA0136"
    """
    dest_c = str(dest or "G").strip().upper()[:1]
    src_c = str(src or "U").strip().upper()[:1]
    addr = f"{src_c}{dest_c}"
    ctrl = str(control or "w").strip().lower()[:1]
    tag_s = str(tag or "").strip().upper()
    if len(tag_s) != 3:
        raise ValueError(f"tag must be 3 characters, got {tag_s!r}")
    data_s = str(data or "")
    use_var = variable if variable is not None else len(data_s) > 2
    if use_var:
        header = "#tp"
        if len(data_s) > 0x0F:
            raise ValueError("variable #tp data length max 15 chars")
        length_ch = format(len(data_s), "X")[:1].upper()
    else:
        header = "#TP"
        if len(data_s) != 2:
            raise ValueError(f"fixed #TP frame requires 2 data chars, got {len(data_s)}")
        length_ch = "2"
    body = f"{header}{addr}{length_ch}{ctrl}{tag_s}{data_s}"
    return f"{body}{tp_checksum(body)}".encode("ascii", errors="ignore")


def encode_speed_2char(deg_per_s: float) -> str:
    """Gimbal speed field: signed byte in 0.1 deg/s units (Yunzhuo TOP / Topotek), two hex ASCII chars."""
    units = int(round(float(deg_per_s) / 0.1))
    units = max(-99, min(99, units))
    return f"{units & 0xFF:02X}"


def encode_angle_4char(deg: float) -> str:
    """Gimbal angle command field: int16 in 0.01° units (Topotek GAY/GAP/GAM)."""
    return encode_attitude_field_4char(deg)


def encode_attitude_field_4char(deg: float) -> str:
    """Telemetry / GAC fields: int16 in 0.01 deg units (four hex ASCII chars)."""
    units = int(round(float(deg) * 100.0))
    units = max(-32768, min(32767, units))
    if units < 0:
        units = (units + 0x10000) & 0xFFFF
    return f"{units:04X}"


def decode_attitude_field_4char(field: str) -> float | None:
    """Decode four hex ASCII chars (0.01 deg) to degrees."""
    s = str(field or "").strip().upper()
    if len(s) != 4 or not re.fullmatch(r"[0-9A-F]{4}", s):
        return None
    raw = int(s, 16)
    if raw >= 0x8000:
        raw -= 0x10000
    return raw / 100.0


def build_gaa_enable(hz: int = 5) -> bytes:
    """Enable active gimbal attitude push (GAA); hz 1–100."""
    rate = max(1, min(100, int(hz)))
    return build_tp_frame(dest="G", control="w", tag="GAA", data=f"{rate:02X}")


def build_gac_query() -> bytes:
    """Query gimbal attitude once (GAC read)."""
    return build_tp_frame(dest="G", control="r", tag="GAC", data="00")


def build_ptz(action: str) -> bytes | None:
    codes = {
        "stop": "00",
        "up": "01",
        "down": "02",
        "left": "03",
        "right": "04",
        "center": "05",
        "nadir": "0A",
        "down_once": "0A",
        "point_down": "0A",
    }
    code = codes.get(str(action or "").strip().lower())
    if code is None:
        return None
    return build_tp_frame(dest="G", control="w", tag="PTZ", data=code)


def build_gimbal_speed(yaw_deg_s: float, pitch_deg_s: float) -> bytes:
    y = encode_speed_2char(yaw_deg_s)
    p = encode_speed_2char(pitch_deg_s)
    return build_tp_frame(dest="G", control="w", tag="GSM", data=f"{y}{p}")


def build_gimbal_angle_axis(axis_tag: str, deg: float, speed: float = 16.0) -> bytes:
    """Single-axis angle command (GAY yaw / GAP pitch). trailing speed in 0.1 deg/s units (0–99)."""
    tag = str(axis_tag or "").strip().upper()
    if tag not in ("GAY", "GAP", "GAR"):
        raise ValueError(f"unsupported angle tag {tag!r}")
    ang = encode_angle_4char(deg)
    spd = encode_speed_2char(speed)
    return build_tp_frame(dest="G", control="w", tag=tag, data=f"{ang}{spd}", variable=True)


def build_system_command(tag: str, data: str, *, write: bool = True) -> bytes:
    """D-class system command (record, photo, etc.) U -> D."""
    return build_tp_frame(
        dest="D",
        control="w" if write else "r",
        tag=str(tag).upper()[:3],
        data=data,
        src="U",
        variable=len(str(data)) != 2,
    )


def build_pod_camera_command(
    tag: str,
    data: str,
    *,
    dest: str = "M",
    write: bool = True,
) -> bytes:
    """UDP camera commands (ZMC/FCC/DZM/…) use Pod addressing per PROTOCAL.doc (#tpPM… / #tpPD…)."""
    return build_tp_frame(
        dest=str(dest or "M").strip().upper()[:1],
        src="P",
        control="w" if write else "r",
        tag=str(tag).upper()[:3],
        data=str(data),
        variable=True,
    )


def build_dzm_absolute_zoom(zoom_x: float, *, camera_x0: int = 0) -> bytes:
    """
    C13 digital zoom absolute set (PROTOCAL §14.1, DZM tag).

    ``zoom_x`` is visible-light multiplier (0.1× steps); 14× → ``#tpPD6wDZM00F08C84``.
    """
    units = int(round(max(0.0, min(300.0, float(zoom_x) * 10.0))))
    mult_hex = f"{units:03X}"
    data = f"{int(camera_x0) & 0xFF:02X}F{mult_hex}"
    return build_pod_camera_command("DZM", data, dest="D")


def build_dzm_absolute_zoom_ud(zoom_x: float, *, camera_x0: int = 0) -> bytes:
    """DZM with User→Device addressing (UART / some UDP firmware paths)."""
    units = int(round(max(0.0, min(300.0, float(zoom_x) * 10.0))))
    mult_hex = f"{units:03X}"
    data = f"{int(camera_x0) & 0xFF:02X}F{mult_hex}"
    return build_tp_frame(dest="D", src="U", control="w", tag="DZM", data=data, variable=True)


def build_mul_optical_zoom(zoom_x: float) -> bytes:
    """Optical magnification (PROTOCAL §9.1, MUL). 24.0× → ``#tpPM4wMUL0240…``."""
    units = int(round(max(1.0, min(300.0, float(zoom_x) * 10.0))))
    return build_pod_camera_command("MUL", f"{units:04d}", dest="M")


def build_zoom_command_burst(zoom_x: float) -> list[bytes]:
    """
    All known C13 zoom frames for one UI level (field firmware varies by port/addressing).
    """
    lvl = max(1.0, min(30.0, float(zoom_x)))
    return [
        build_dzm_absolute_zoom(lvl),
        build_dzm_absolute_zoom_ud(lvl),
        build_mul_optical_zoom(lvl),
    ]


def build_dzm_step_zoom(action: str) -> bytes:
    """Digital zoom step in/out/stop (PROTOCAL §14.1: 0C/0D with 0E stop)."""
    act = str(action or "").strip().lower()
    code = {"stop": "0E", "in": "0C", "out": "0D", "tele": "0C", "wide": "0D"}.get(act)
    if code is None:
        raise ValueError(f"unsupported DZM step action {action!r}")
    return build_pod_camera_command("DZM", f"00{code}", dest="D")


def build_zmc_zoom(action: str) -> bytes:
    """Optical zoom step (PROTOCAL §1.1, ZMC). UDP: 01=out, 02=in; pair with stop."""
    act = str(action or "").strip().lower()
    code = {"stop": "00", "out": "01", "in": "02", "wide": "01", "tele": "02"}.get(act)
    if code is None:
        raise ValueError(f"unsupported ZMC action {action!r}")
    return build_pod_camera_command("ZMC", code, dest="M")


def build_fcc_focus(action: str) -> bytes:
    """Focus control (PROTOCAL §2.1, FCC tag)."""
    act = str(action or "").strip().lower()
    code = {
        "stop": "00",
        "near": "02",
        "far": "01",
        "in": "02",
        "out": "01",
        "auto": "10",
    }.get(act)
    if code is None:
        raise ValueError(f"unsupported FCC action {action!r}")
    return build_pod_camera_command("FCC", code, dest="M")


# --- Legacy $TOP,...*XOR (kept for simulators / old captures) ---


def _xor_checksum(data: str) -> int:
    value = 0
    for ch in data.encode("ascii", errors="ignore"):
        value ^= int(ch)
    return value & 0xFF


def build_legacy_top_frame(command: str, params: Mapping[str, object] | None = None) -> bytes:
    cmd = str(command or "").strip().upper()
    if not cmd:
        raise ValueError("command is required")
    parts = ["TOP", cmd]
    for key, value in (params or {}).items():
        k = str(key or "").strip().lower()
        if not k:
            continue
        parts.append(f"{k}={value}")
    body = ",".join(parts)
    checksum = _xor_checksum(body)
    return f"${body}*{checksum:02X}\r\n".encode("ascii", errors="ignore")


def build_top_frame(command: str, params: Mapping[str, object] | None = None) -> bytes:
    """
    Build a TOP command frame.

    Maps high-level command names to official #TP frames when recognized;
    otherwise falls back to legacy $TOP,... format.
    """
    cmd = str(command or "").strip().upper()
    if not cmd:
        raise ValueError("command is required")
    p = params or {}

    if cmd == "GAA":
        hz = int(p.get("hz", p.get("rate", 5)) or 5)
        return build_gaa_enable(hz)
    if cmd == "GAC":
        return build_gac_query()

    # PTZ aliases
    ptz_alias = {
        "PT_UP": "up",
        "PT_DOWN": "down",
        "PT_LEFT": "left",
        "PT_RIGHT": "right",
        "PT_CENTER": "center",
        "PT_STOP": "stop",
        "PTZ_UP": "up",
        "PTZ_DOWN": "down",
        "PTZ_LEFT": "left",
        "PTZ_RIGHT": "right",
        "PTZ_CENTER": "center",
        "PTZ_STOP": "stop",
        "PTZ_NADIR": "nadir",
        "PT_NADIR": "nadir",
    }
    if cmd in ptz_alias:
        frame = build_ptz(ptz_alias[cmd])
        if frame is not None:
            return frame
    if cmd == "PTZ":
        act = str(p.get("action", "stop"))
        frame = build_ptz(act)
        if frame is not None:
            return frame

    # Speed
    if cmd in ("GSY", "GSP", "GSR"):
        spd = float(p.get("speed", p.get("yaw", p.get("pitch", 0.0)) or 0.0))
        if cmd == "GSP":
            spd = float(p.get("pitch", p.get("speed", 0.0)) or 0.0)
        elif cmd == "GSR":
            spd = float(p.get("roll", p.get("speed", 0.0)) or 0.0)
        else:
            spd = float(p.get("yaw", p.get("speed", 0.0)) or 0.0)
        return build_tp_frame(dest="G", control="w", tag=cmd, data=encode_speed_2char(spd))

    if cmd == "GSM":
        yaw = float(p.get("yaw", 0.0) or 0.0)
        pitch = float(p.get("pitch", 0.0) or 0.0)
        return build_gimbal_speed(yaw, pitch)

    # Angle
    if cmd in ("GAY", "GAP", "GAR", "GAM"):
        if cmd == "GAM":
            yaw = float(p.get("yaw", 0.0) or 0.0)
            pitch = float(p.get("pitch", 0.0) or 0.0)
            spd = float(p.get("speed", 25.0) or 25.0)
            yaw_spd = float(p.get("yaw_speed", spd) or spd)
            pitch_spd = float(p.get("pitch_speed", spd) or spd)
            return build_tp_frame(
                dest="G",
                control="w",
                tag="GAM",
                data=(
                    f"{encode_angle_4char(yaw)}"
                    f"{encode_speed_2char(yaw_spd)}"
                    f"{encode_angle_4char(pitch)}"
                    f"{encode_speed_2char(pitch_spd)}"
                ),
                variable=True,
            )
        deg = float(
            p.get(
                "yaw" if cmd == "GAY" else "pitch" if cmd == "GAP" else "roll",
                p.get("angle", 0.0),
            )
            or 0.0
        )
        spd = float(p.get("speed", 16.0) or 16.0)
        return build_gimbal_angle_axis(cmd, deg, spd)

    # Camera / system (best-effort fixed 2-byte data)
    cam_map = {
        "CAM_REC": ("REC", "01"),
        "CAM_RECORD": ("REC", "01"),
        "CAM_SNAP": ("CAP", "01"),
        "CAM_PHOTO": ("CAP", "01"),
    }
    if cmd in cam_map:
        tag, data = cam_map[cmd]
        return build_system_command(tag, data)

    if cmd in ("CAM_ZOOM", "CAM_Z"):
        level = float(p.get("level", p.get("zoom", 1.0)) or 1.0)
        return build_dzm_absolute_zoom(level)

    if cmd == "ZOOM_BURST":
        # Expanded in adapter worker — placeholder frame.
        level = float(p.get("level", 1.0) or 1.0)
        return build_dzm_absolute_zoom(level)

    if cmd == "ZMC":
        return build_zmc_zoom(str(p.get("action", "stop")))

    if cmd == "DZM_STEP":
        return build_dzm_step_zoom(str(p.get("action", "stop")))

    if cmd == "FCC":
        return build_fcc_focus(str(p.get("action", "stop")))

    if cmd in ("CAM_FOCUS_NEAR", "CAM_FN", "FOCUS_NEAR"):
        return build_fcc_focus("near")
    if cmd in ("CAM_FOCUS_FAR", "CAM_FF", "FOCUS_FAR"):
        return build_fcc_focus("far")

    return build_legacy_top_frame(cmd, p)


@dataclass(frozen=True)
class DecodedTopFrame:
    command: str
    params: dict[str, str]
    raw: str
    protocol: str = "tp"  # "tp" | "legacy"


_TP_RE = re.compile(
    r"^#tp([UMDEG]{2})([0-9A-F])([wr])([A-Z]{3})(.*?)([0-9A-F]{2})$",
    re.IGNORECASE | re.DOTALL,
)


def parse_tp_frame(raw: bytes) -> DecodedTopFrame | None:
    text = (raw or b"").decode("ascii", errors="ignore").strip()
    if not text.upper().startswith("#TP"):
        return None
    m = _TP_RE.match(text)
    if not m:
        return None
    _addr, _length, _ctrl, tag, data, _crc = m.groups()
    tag_u = tag.upper()
    params: dict[str, str] = {"tag": tag_u}
    if tag_u == "GAC" and len(data) >= 12:
        params["yaw_hex"] = data[0:4]
        params["pitch_hex"] = data[4:8]
        params["roll_hex"] = data[8:12]
        yaw = decode_attitude_field_4char(data[0:4])
        pitch = decode_attitude_field_4char(data[4:8])
        roll = decode_attitude_field_4char(data[8:12])
        if yaw is not None:
            params["yaw"] = f"{yaw:.4f}"
        if pitch is not None:
            params["pitch"] = f"{pitch:.4f}"
        if roll is not None:
            params["roll"] = f"{roll:.4f}"
    return DecodedTopFrame(command=tag_u, params=params, raw=text, protocol="tp")


def parse_legacy_top_frame(raw: bytes) -> DecodedTopFrame | None:
    text = (raw or b"").decode("ascii", errors="ignore").strip()
    if not text:
        return None
    if text.startswith("$"):
        text = text[1:]
    if "*" in text:
        text = text.split("*", 1)[0]
    parts = [p.strip() for p in text.split(",") if p.strip()]
    if len(parts) < 2:
        return None
    command = parts[1].upper()
    params: dict[str, str] = {}
    bare_nums: list[str] = []
    for part in parts[2:]:
        if "=" in part:
            k, v = part.split("=", 1)
            params[k.strip().lower()] = v.strip()
        else:
            bare_nums.append(part)
    if bare_nums:
        if "yaw" not in params and len(bare_nums) >= 1:
            params["yaw"] = bare_nums[0]
        if "pitch" not in params and len(bare_nums) >= 2:
            params["pitch"] = bare_nums[1]
        if "roll" not in params and len(bare_nums) >= 3:
            params["roll"] = bare_nums[2]
    return DecodedTopFrame(command=command, params=params, raw=(raw or b"").decode("ascii", errors="ignore"), protocol="legacy")


def parse_top_frame(raw: bytes) -> DecodedTopFrame | None:
    dec = parse_tp_frame(raw)
    if dec is not None:
        return dec
    return parse_legacy_top_frame(raw)


_ATTITUDE_KEYS: tuple[tuple[str, str], ...] = (
    ("yaw", "yaw"),
    ("yaw_deg", "yaw"),
    ("pan", "yaw"),
    ("y", "yaw"),
    ("pitch", "pitch"),
    ("pitch_deg", "pitch"),
    ("tilt", "pitch"),
    ("p", "pitch"),
)


def extract_attitude_deg(dec: DecodedTopFrame | None) -> tuple[float | None, float | None]:
    """Parse yaw/pitch from a TOP frame (GAC/GAA replies and async telemetry)."""
    if dec is None:
        return None, None
    yaw_v: float | None = None
    pitch_v: float | None = None
    for src, dst in _ATTITUDE_KEYS:
        if dst == "yaw" and yaw_v is not None:
            continue
        if dst == "pitch" and pitch_v is not None:
            continue
        raw = dec.params.get(src)
        if raw is None:
            continue
        val = _to_float(raw)
        if val is None:
            continue
        if dst == "yaw":
            yaw_v = val
        else:
            pitch_v = val
    return yaw_v, pitch_v


def _to_float(v: object) -> float | None:
    try:
        if v is None:
            return None
        s = str(v).strip().replace("°", "")
        if not s:
            return None
        return float(s)
    except Exception:
        return None
