"""Tests for native video overlay helpers."""

from __future__ import annotations

from vgcs.map.native_video_overlay import (
    NativeVideoOverlayLayer,
    VideoOverlayFacadeHint,
    offscreen_hint_edge_uv,
)


def test_facade_hint_dataclass() -> None:
    hint = VideoOverlayFacadeHint(
        title="Facade locked — LRF 42.5 m",
        subtitle="Click on video (fast pick): Gun · Target",
        ready=True,
    )
    assert hint.ready is True
    assert "Facade locked" in hint.title


def test_offscreen_hint_edge_uv_left() -> None:
    ex, ey, angle = offscreen_hint_edge_uv(-0.35, 0.52)
    assert ex < 0.1
    assert 0.0 < ey < 1.0
    assert 160.0 < angle < 200.0


def test_offscreen_hint_edge_uv_right() -> None:
    ex, ey, angle = offscreen_hint_edge_uv(1.42, 0.48)
    assert ex > 0.9
    assert 0.0 < ey < 1.0
    assert -20.0 < angle < 20.0


def test_offscreen_hint_edge_uv_above() -> None:
    ex, ey, angle = offscreen_hint_edge_uv(0.51, -0.2)
    assert ey < 0.1
    assert 0.0 < ex < 1.0
    assert -100.0 < angle < -80.0
