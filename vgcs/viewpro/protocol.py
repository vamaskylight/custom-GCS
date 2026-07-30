"""Viewpro/ViewLink gimbal wire protocol.

Two layers, both verified byte-for-byte against the worked examples in the
vendor documents (2026-07, via Viewpro support after-sales):
- "TCP control.pdf"
- "Viewpro Viewlink Serial Command Communication Protocol" V3.4.9

Layer 1 — ViewLink serial frame (what the gimbal core actually parses):
    55 AA DC | length(6 bits)+frame_counter(2 bits) | cmd_id | data(N bytes,
    big-endian) | checksum

    checksum = XOR of (length byte with frame_counter bits masked off),
    cmd_id, and every data byte. Confirmed against the doc's own C snippet
    and multiple worked examples (e.g. `55 AA DC 05 1C 03 83 99`).

Layer 2 — TCP envelope (what actually goes on the wire over the TCP control
port, default 2000):
    EB 90 | inner_frame_length(1 byte) | inner_frame_bytes | checksum

    checksum = 8-bit sum (mod 256) of inner_frame_bytes. Confirmed against
    the doc's own C snippet and the worked "Center gimbal" / "Status query"
    examples.
"""

from __future__ import annotations

SERIAL_HEADER = bytes([0x55, 0xAA, 0xDC])
TCP_HEADER = bytes([0xEB, 0x90])

# Frame IDs (subset actually used here — see the doc's full Data Flow table
# in DOCS/VIEWPRO-CAMERA-REFERENCE.md for the rest).
CMD_A1_C1_E1 = 0x30  # combo: gimbal servo (A1, 9B) + optical (C1, 2B) + tracking (E1, 3B) = 14B body
CMD_HEARTBEAT = 0x10  # console -> payload heartbeat; payload replies with CMD_STATUS
CMD_STATUS_T1F1B1D1 = 0x40  # payload -> console periodic status (41B body: T1 22 + F1 1 + B1 6 + D1 12)

# --- A1 servo control (9 bytes: servo(1) + 4x signed int16 params, big-endian) ---
SERVO_MOTOR_ON_OFF = 0x00
SERVO_MANUAL_SPEED = 0x01  # params1/2 = yaw/pitch velocity, 1bit=0.01 deg/s, signed
# params1/3 = yaw/pitch velocity (0=default), params2/4 = yaw/pitch angle x360/65536,
# measured FROM THE CURRENT POSITION. Sent with all-zero params it is a "move by 0
# degrees" request, which hands the gimbal a position target at exactly where it
# already is — see ViewproGimbalTcpAdapter._apply_position_hold for why that matters.
SERVO_MANUAL_RELATIVE_ANGLE = 0x09
SERVO_HOME_POSITION = 0x04
SERVO_MANUAL_ABSOLUTE_ANGLE = 0x0B  # home position as 0; NOT for high-frequency/continuous sends per doc
SERVO_LOOK_DOWN = 0x12  # "pitch orthographic", look straight down
SERVO_NO_CHANGE = 0x0F  # "do not change servo state" — use when only sending an optical (C1) command

# Sign convention confirmed from worked examples (Right 90/Down 90 -> +yaw/+pitch;
# Left 90/Up 30 -> -yaw/-pitch): yaw positive = right, pitch positive = down.

# --- C1 optical control (2 bytes, bit-packed) ---
# bit15-13: LRF cmd (3b) | bit12-6: operation command (7b) | bit5-3: zoom/param speed (3b) | bit2-0: sensor select (3b)
C1_OP_STOP = 0x01  # stop focus, stop zoom
C1_OP_FOV_PLUS_ZOOM_OUT = 0x08
C1_OP_FOV_MINUS_ZOOM_IN = 0x09
# NOTE: the command table (Serial Command Protocol doc, section on C1 Operation
# Command 1) lists 0x0A=Focus+ and 0x0B=Focus-. The doc's OWN worked examples in
# "4.2.2 EO camera Focus" ("Manual focus+" -> C1=0x02C8, "Manual focus-" -> C1=0x0288)
# decode to the OPPOSITE of the table (0x02C8 decodes to op=0x0B, 0x0288 decodes to
# op=0x0A) — every other worked example in the document (zoom, photo, record, auto/
# manual focus mode) matches the table exactly, so this looks like a copy/paste slip
# in that one pair of examples, not a table error. Implemented per the table; the
# physical near/far direction for each has NOT been field-verified — see
# DOCS/VIEWPRO-CAMERA-REFERENCE.md.
C1_OP_FOCUS_PLUS = 0x0A
C1_OP_FOCUS_MINUS = 0x0B
C1_OP_AUTO_FOCUS = 0x19
C1_OP_MANUAL_FOCUS_MODE = 0x1A
C1_OP_TAKE_PICTURE = 0x13
C1_OP_START_RECORD = 0x14
C1_OP_STOP_RECORD = 0x15

C1_LRF_NONE = 0x00
C1_LRF_SINGLE = 0x01
C1_LRF_CONTINUOUS_START = 0x02
C1_LRF_STOP = 0x05

_T1_LEN = 22
_F1_LEN = 1
_B1_LEN = 6
_D1_LEN = 12
_B1_OFFSET = _T1_LEN + _F1_LEN
_D1_OFFSET = _T1_LEN + _F1_LEN + _B1_LEN


def _serial_checksum(length_byte: int, payload: bytes) -> int:
    cs = length_byte & 0xFF
    for b in payload:
        cs ^= b
    return cs & 0xFF


def build_serial_frame(cmd_id: int, data: bytes = b"") -> bytes:
    """Build a ViewLink serial-protocol frame (frame_counter always 0)."""
    body_len = (len(data) + 3) & 0x3F
    payload = bytes([cmd_id & 0xFF]) + data
    checksum = _serial_checksum(body_len, payload)
    return SERIAL_HEADER + bytes([body_len]) + payload + bytes([checksum])


def parse_serial_frame(payload: bytes) -> tuple[int, bytes] | None:
    """Return (cmd_id, data) for the frame at the START of payload, or None.

    Does not resync/search — use find_status_frame() to locate a frame
    anywhere within a raw recv() buffer.
    """
    if len(payload) < 7 or payload[0:3] != SERIAL_HEADER:
        return None
    length_byte = payload[3]
    body_len = length_byte & 0x3F
    total = 3 + body_len
    if body_len < 3 or len(payload) < total:
        return None
    cmd_id = payload[4]
    n = body_len - 3
    data = payload[5 : 5 + n]
    checksum = payload[3 + body_len - 1]
    expected = _serial_checksum(body_len, bytes([cmd_id]) + data)
    if checksum != expected:
        return None
    return cmd_id, data


def wrap_tcp(inner_frame: bytes) -> bytes:
    length = len(inner_frame) & 0xFF
    checksum = sum(inner_frame) & 0xFF
    return TCP_HEADER + bytes([length]) + inner_frame + bytes([checksum])


def unwrap_tcp(payload: bytes) -> bytes | None:
    if len(payload) < 4 or payload[0:2] != TCP_HEADER:
        return None
    length = payload[2]
    if len(payload) < 3 + length + 1:
        return None
    inner = payload[3 : 3 + length]
    checksum = payload[3 + length]
    if (sum(inner) & 0xFF) != checksum:
        return None
    return inner


def _i16(v: int) -> bytes:
    v = max(-32768, min(32767, int(v)))
    return v.to_bytes(2, "big", signed=True)


def _u16(v: int) -> bytes:
    return (int(v) & 0xFFFF).to_bytes(2, "big", signed=False)


def encode_a1(servo: int, p1: int = 0, p2: int = 0, p3: int = 0, p4: int = 0) -> bytes:
    return bytes([servo & 0xFF]) + _i16(p1) + _i16(p2) + _i16(p3) + _i16(p4)


def encode_c1(*, op: int = 0, zoom_speed: int = 0, sensor: int = 0, lrf: int = 0) -> bytes:
    value = ((lrf & 0x7) << 13) | ((op & 0x7F) << 6) | ((zoom_speed & 0x7) << 3) | (sensor & 0x7)
    return _u16(value)


def encode_gimbal_camera_command(
    *,
    servo: int = SERVO_NO_CHANGE,
    servo_p1: int = 0,
    servo_p2: int = 0,
    servo_p3: int = 0,
    servo_p4: int = 0,
    c1_op: int = 0,
    c1_zoom_speed: int = 0,
    c1_sensor: int = 0,
    c1_lrf: int = 0,
) -> bytes:
    """Build the ready-to-send TCP bytes for the A1+C1+E1 combo frame (CMD 0x30).

    E1 (tracking) is always sent as "no action" — this project doesn't drive
    Viewpro's onboard tracker.
    """
    a1 = encode_a1(servo, servo_p1, servo_p2, servo_p3, servo_p4)
    c1 = encode_c1(op=c1_op, zoom_speed=c1_zoom_speed, sensor=c1_sensor, lrf=c1_lrf)
    e1 = bytes(3)
    inner = build_serial_frame(CMD_A1_C1_E1, a1 + c1 + e1)
    return wrap_tcp(inner)


def encode_heartbeat() -> bytes:
    return wrap_tcp(build_serial_frame(CMD_HEARTBEAT, bytes([0x00])))


def angle_deg_to_raw(deg: float) -> int:
    """1bit = 360/65536 degree, signed int16.

    The wire field is circular (-180..+180, wrapping at the ends — same
    physical point at +180 and -180), so wrap the input into that range
    first. Without this, an absolute-angle target computed as
    ``base_yaw + delta`` (see ``ViewproCameraControl.set_gimbal``) that
    lands past +-180 gets silently clamped to the nearest int16 edge by
    ``_i16`` instead of wrapping — the gimbal then appears to hard-stop at
    +-180 even though it can rotate continuously through 360 degrees.
    """
    wrapped = ((float(deg) + 180.0) % 360.0) - 180.0
    return int(round(wrapped * 65536.0 / 360.0))


def angle_raw_to_deg(raw: int) -> float:
    return float(raw) * 360.0 / 65536.0


def speed_dps_to_raw(dps: float) -> int:
    """1bit = 0.01 deg/s, signed int16."""
    return int(round(float(dps) * 100.0))


def decode_b1(data: bytes) -> tuple[float, float, float, int] | None:
    """Decode a 6-byte B1 (gimbal attitude) block -> (yaw_deg, pitch_deg, roll_deg, servo_status).

    Verified against the doc's worked status-frame example: bytes
    `D7 FF 1B F0 E3 8C` -> yaw 39.28 deg, pitch -40.0 deg, roll -0.03 deg,
    servo_status 0x0D (manual RC mode).
    """
    if len(data) < 6:
        return None
    b1_0 = data[0]
    servo_status = (b1_0 >> 4) & 0x0F
    roll_high4 = b1_0 & 0x0F
    roll_low8 = data[1]
    roll_raw12 = (roll_high4 << 8) | roll_low8
    roll_deg = roll_raw12 * 180.0 / 4095.0 - 90.0
    yaw_raw = int.from_bytes(data[2:4], "big", signed=True)
    pitch_raw = int.from_bytes(data[4:6], "big", signed=True)
    return angle_raw_to_deg(yaw_raw), angle_raw_to_deg(pitch_raw), roll_deg, servo_status


def decode_d1(data: bytes) -> dict | None:
    """Decode a 12-byte D1 (camera status) block.

    LRF range is a 3-byte value split as byte[1] (high byte) + bytes[4:6]
    (low word) — confirmed against the doc's changelog note ("the distance
    measurement value occupies three bytes") and the worked example, where
    both halves are 0 and the doc annotates that exact value as "Distance
    measurement 0m (0x000000)".
    """
    if len(data) < _D1_LEN:
        return None
    record_status = int.from_bytes(data[2:4], "big") & 0x3
    range_raw = (data[1] << 16) | (data[4] << 8) | data[5]
    range_m = (range_raw * 0.1) if range_raw > 0 else None
    vfov_deg = int.from_bytes(data[6:8], "big") * 0.01
    hfov_deg = int.from_bytes(data[8:10], "big") * 0.01
    zoom_x = int.from_bytes(data[10:12], "big") * 0.1
    return {
        "record_status": record_status,
        "range_m": range_m,
        "vfov_deg": vfov_deg,
        "hfov_deg": hfov_deg,
        "zoom_x": zoom_x,
    }


def decode_status_frame(data: bytes) -> dict | None:
    """Decode a CMD_STATUS_T1F1B1D1 (0x40) body -> gimbal attitude + camera status."""
    if len(data) < _B1_OFFSET + _B1_LEN:
        return None
    decoded_b1 = decode_b1(data[_B1_OFFSET : _B1_OFFSET + _B1_LEN])
    if decoded_b1 is None:
        return None
    yaw, pitch, roll, servo_status = decoded_b1
    out = {"yaw_deg": yaw, "pitch_deg": pitch, "roll_deg": roll, "servo_status": servo_status}
    d1 = decode_d1(data[_D1_OFFSET : _D1_OFFSET + _D1_LEN])
    if d1 is not None:
        out.update(d1)
    return out


def find_status_frame(payload: bytes) -> dict | None:
    """Scan a raw recv() buffer for the first valid status frame (CMD 0x40)."""
    idx = 0
    while True:
        pos = payload.find(SERIAL_HEADER, idx)
        if pos < 0:
            return None
        parsed = parse_serial_frame(payload[pos:])
        if parsed is not None:
            cmd_id, data = parsed
            if cmd_id == CMD_STATUS_T1F1B1D1:
                decoded = decode_status_frame(data)
                if decoded is not None:
                    return decoded
        idx = pos + 1
