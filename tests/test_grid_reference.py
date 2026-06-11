"""Grid reference (MGRS) conversion for observation reports."""

from __future__ import annotations

import pytest

from vgcs.observe.grid_reference import (
    format_grid_reference,
    format_mgrs_display,
    grid_reference_available,
    latlon_to_mgrs,
)


@pytest.mark.skipif(not grid_reference_available(), reason="mgrs package not installed")
def test_latlon_to_mgrs_india_sample():
    raw = latlon_to_mgrs(20.4458915, 72.8632389, precision=5)
    assert raw is not None
    assert raw.startswith("43Q")
    assert format_mgrs_display(raw) == "43Q BC 77080 62277"


def test_format_grid_reference_invalid():
    assert format_grid_reference(None, None) == ""
    assert format_grid_reference(0.0, 0.0) == ""


def test_format_mgrs_display_spacing():
    assert format_mgrs_display("43QBC7708062277") == "43Q BC 77080 62277"


@pytest.mark.skipif(not grid_reference_available(), reason="mgrs package not installed")
def test_format_grid_reference_roundtrip_coords():
    gr = format_grid_reference(37.7749, -122.4194)
    assert " " in gr
    assert gr.startswith("10S")
