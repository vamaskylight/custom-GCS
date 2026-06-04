"""Pytest hooks for VGCS tests."""

from __future__ import annotations

import pytest

from vgcs.observe.target_measure import clear_tape_pair_override, set_segment_distance_scale


@pytest.fixture(autouse=True)
def _reset_observe_tape_state() -> None:
    """Tape Cal persists in-process globals; isolate tests."""
    clear_tape_pair_override()
    set_segment_distance_scale(1.0)
    yield
    clear_tape_pair_override()
    set_segment_distance_scale(1.0)
