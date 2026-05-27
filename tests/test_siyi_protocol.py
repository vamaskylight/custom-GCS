from __future__ import annotations

from vgcs.siyi.protocol import encode_angle_deg, encode_rotation_speed


def test_encode_rotation_speed_pitch_only() -> None:
    assert encode_rotation_speed(0.0, 5.0) == bytes([0, 50])


def test_encode_rotation_speed_yaw_only() -> None:
    assert encode_rotation_speed(-5.0, 0.0) == bytes([-50 & 0xFF, 0])


def test_encode_rotation_speed_stop() -> None:
    assert encode_rotation_speed(0.0, 0.0) == bytes([0, 0])


def test_encode_angle_deg_is_four_bytes() -> None:
    assert len(encode_angle_deg(10.0, 20.0)) == 4
