"""RTCM 3 correction data: finding the messages in a byte stream, and packing them for MAVLink.

Milestone M11, "RTK module connection". An RTK base station sends corrections
as RTCM 3 messages. The drone's GPS needs them to reach RTK Float and RTK
Fixed. VGCS reads them from the base (serial port or network) and passes them
to the drone in ``GPS_RTCM_DATA`` messages, as Mission Planner and
QGroundControl do.

RTCM 3 frame (RTCM 10403): the byte 0xD3, six zero bits, a 10 bit length, the
message, and a 24 bit checksum (CRC-24Q) over everything before it. The first
12 bits of the message are its type number (1005 is the base position, 1077 a
GPS observation, and so on).

Only whole frames with a right checksum are passed on. A base that is not
configured yet sends other things on the same port (NMEA text, u-blox binary),
and half a correction is worse for the drone's GPS than none.

Pure: no Qt, no serial port, no MAVLink connection.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

PREAMBLE = 0xD3
_HEADER_LEN = 3
_CRC_LEN = 3
MAX_FRAME_LEN = _HEADER_LEN + 1023 + _CRC_LEN

# GPS_RTCM_DATA carries at most this much, and at most four pieces per block.
MAVLINK_PIECE_LEN = 180
MAVLINK_BLOCK_LEN = MAVLINK_PIECE_LEN * 4

_CRC24Q_POLY = 0x1864CFB


def _make_crc_table() -> list[int]:
    table = []
    for byte in range(256):
        crc = byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= _CRC24Q_POLY
        table.append(crc & 0xFFFFFF)
    return table


_CRC_TABLE = _make_crc_table()


def crc24q(data: bytes) -> int:
    """The checksum RTCM 3 uses (CRC-24Q)."""
    crc = 0
    for byte in data:
        crc = ((crc << 8) & 0xFFFFFF) ^ _CRC_TABLE[((crc >> 16) ^ byte) & 0xFF]
    return crc


@dataclass(frozen=True)
class RtcmFrame:
    """One whole RTCM 3 message, as it came off the wire (header and checksum included)."""

    message_type: int
    data: bytes


def build_frame(message: bytes) -> bytes:
    """Wrap a message (starting with its 12 bit type) into an RTCM 3 frame. For tests and tools."""
    if len(message) > 1023:
        raise ValueError("an RTCM 3 message holds at most 1023 bytes")
    head = bytes([PREAMBLE, (len(message) >> 8) & 0x03, len(message) & 0xFF]) + message
    return head + crc24q(head).to_bytes(3, "big")


class Rtcm3Parser:
    """Find whole RTCM 3 frames in a stream that may hold anything else too."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.frames = 0          # frames with a right checksum
        self.crc_errors = 0      # looked like a frame, wrong checksum
        self.skipped_bytes = 0   # bytes that were not part of any frame

    def reset(self) -> None:
        self._buffer.clear()

    def feed(self, data: bytes) -> list[RtcmFrame]:
        """Add received bytes. Returns the frames that are now complete, in order."""
        self._buffer += data
        out: list[RtcmFrame] = []
        buf = self._buffer
        pos = 0
        while True:
            start = buf.find(PREAMBLE, pos)
            if start < 0:
                self.skipped_bytes += len(buf) - pos
                pos = len(buf)
                break
            self.skipped_bytes += start - pos
            pos = start
            if len(buf) - pos < _HEADER_LEN:
                break                                   # wait for the rest of the header
            if buf[pos + 1] & 0xFC:
                # The six bits after 0xD3 are always zero. This 0xD3 is just a data byte.
                pos += 1
                self.skipped_bytes += 1
                continue
            length = ((buf[pos + 1] & 0x03) << 8) | buf[pos + 2]
            total = _HEADER_LEN + length + _CRC_LEN
            if len(buf) - pos < total:
                break                                   # wait for the rest of the frame
            frame = bytes(buf[pos:pos + total])
            if crc24q(frame[:-_CRC_LEN]) != int.from_bytes(frame[-_CRC_LEN:], "big"):
                self.crc_errors += 1
                pos += 1
                self.skipped_bytes += 1
                continue
            message_type = ((frame[3] << 4) | (frame[4] >> 4)) if length >= 2 else 0
            out.append(RtcmFrame(message_type, frame))
            self.frames += 1
            pos += total
        del buf[:pos]
        return out


def to_mavlink_pieces(data: bytes, sequence: int) -> tuple[list[tuple[int, bytes]], int]:
    """Cut correction data into ``GPS_RTCM_DATA`` pieces. Returns ``(flags, bytes)`` pairs and the next sequence number.

    The rules are ArduPilot's (AP_GPS::handle_gps_rtcm_fragment):

    - Up to 180 bytes go in one message that is not marked as fragmented.
    - More than that is cut into pieces of 180 bytes, numbered 0 to 3, all with
      the same sequence number. The autopilot knows the block is complete when
      it gets a piece shorter than 180 bytes, or all four.
    - So a block of exactly 360 or 540 bytes needs one more, empty piece.
    - A block holds at most 720 bytes. Longer data is sent as several blocks.
      The GPS reads a byte stream, so a correction may span two blocks.
    """
    pieces: list[tuple[int, bytes]] = []
    for block_start in range(0, len(data), MAVLINK_BLOCK_LEN):
        block = data[block_start:block_start + MAVLINK_BLOCK_LEN]
        seq = sequence & 0x1F
        sequence = (sequence + 1) & 0x1F
        if len(block) <= MAVLINK_PIECE_LEN:
            pieces.append((seq << 3, block))
            continue
        parts = [block[i:i + MAVLINK_PIECE_LEN] for i in range(0, len(block), MAVLINK_PIECE_LEN)]
        if len(block) % MAVLINK_PIECE_LEN == 0 and len(parts) < 4:
            parts.append(b"")
        for index, part in enumerate(parts):
            pieces.append((1 | (index << 1) | (seq << 3), part))
    return pieces, sequence


# --- the base station's own position (messages 1005 and 1006) --------------

_WGS84_A = 6378137.0
_WGS84_E2 = 6.69437999014e-3


def _bits(data: bytes, start: int, count: int, signed: bool = False) -> int:
    value = 0
    for i in range(start, start + count):
        value = (value << 1) | ((data[i // 8] >> (7 - i % 8)) & 1)
    if signed and value & (1 << (count - 1)):
        value -= 1 << count
    return value


def ecef_to_geodetic(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Earth-centred coordinates in metres to latitude, longitude (degrees) and height (metres), WGS 84."""
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - _WGS84_E2))
    height = 0.0
    for _ in range(8):
        n = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * math.sin(lat) ** 2)
        height = p / math.cos(lat) - n
        lat = math.atan2(z, p * (1.0 - _WGS84_E2 * n / (n + height)))
    return math.degrees(lat), math.degrees(lon), height


def station_position(frame: RtcmFrame) -> tuple[float, float, float] | None:
    """The base station's latitude, longitude and height from a 1005 or 1006 message, else None.

    A base sends this once it knows where it stands (surveyed in, or a fixed
    position typed in). Without it the drone's GPS cannot use the corrections.
    """
    if frame.message_type not in (1005, 1006):
        return None
    message = frame.data[_HEADER_LEN:-_CRC_LEN]
    if len(message) < 19:
        return None
    # type 12, station 12, ITRF year 6, four flag bits, then X 38, two bits, Y 38, two bits, Z 38
    x = _bits(message, 34, 38, signed=True) * 1e-4
    y = _bits(message, 74, 38, signed=True) * 1e-4
    z = _bits(message, 114, 38, signed=True) * 1e-4
    if abs(x) < 1.0 and abs(y) < 1.0 and abs(z) < 1.0:
        return None          # all zero: the base does not know its position yet
    return ecef_to_geodetic(x, y, z)


def build_station_message(lat_deg: float, lon_deg: float, height_m: float, station_id: int = 0) -> bytes:
    """A 1005 frame for a base at this position. For tests and tools."""
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    n = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * math.sin(lat) ** 2)
    xyz = (
        (n + height_m) * math.cos(lat) * math.cos(lon),
        (n + height_m) * math.cos(lat) * math.sin(lon),
        (n * (1.0 - _WGS84_E2) + height_m) * math.sin(lat),
    )
    bits = 0
    count = 0

    def put(value: int, width: int) -> None:
        nonlocal bits, count
        bits = (bits << width) | (value & ((1 << width) - 1))
        count += width

    put(1005, 12)
    put(station_id, 12)
    put(0, 6)       # ITRF year
    put(0b1110, 4)  # GPS, GLONASS and Galileo supported, physical reference station
    put(round(xyz[0] * 1e4), 38)
    put(0, 2)
    put(round(xyz[1] * 1e4), 38)
    put(0, 2)
    put(round(xyz[2] * 1e4), 38)
    assert count == 152
    return build_frame(bits.to_bytes(19, "big"))
