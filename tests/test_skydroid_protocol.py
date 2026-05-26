from __future__ import annotations

from vgcs.skydroid.protocol import (
    build_gaa_enable,
    build_gac_query,
    build_top_frame,
    build_tp_frame,
    decode_attitude_field_4char,
    encode_attitude_field_4char,
    extract_attitude_deg,
    parse_top_frame,
    tp_checksum,
)


def test_checksum_doc_examples() -> None:
    assert tp_checksum("#TPUD2wAWB01") == "44"
    assert tp_checksum("#TPUG2wGAA01") == "36"
    assert tp_checksum("#TPUG2wPTZ00") == "6A"
    assert tp_checksum("#tpUD2rIPV00") == "93"


def test_build_gaa_matches_doc() -> None:
    assert build_gaa_enable(1) == b"#TPUG2wGAA0136"


def test_parse_gac_response() -> None:
    raw = b"#TPUGCrGACEC780000000064"
    dec = parse_top_frame(raw)
    assert dec is not None
    assert dec.command == "GAC"
    yaw, pitch = extract_attitude_deg(dec)
    assert yaw is not None and abs(yaw + 50.0) < 0.01
    assert pitch is not None and abs(pitch) < 0.01


def test_attitude_field_roundtrip() -> None:
    field = encode_attitude_field_4char(-50.0)
    assert field == "EC78"
    assert decode_attitude_field_4char(field) == -50.0


def test_build_top_frame_ptz() -> None:
    frame = build_top_frame("PT_UP", {})
    assert frame == b"#TPUG2wPTZ016B"


def test_build_gac_query() -> None:
    frame = build_gac_query()
    assert frame.startswith(b"#TPUG2rGAC")
