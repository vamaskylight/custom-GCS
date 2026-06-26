"""DOOAF setup marks must not follow observation impact LRF slews."""

from __future__ import annotations

from vgcs.map.map_widget import MapWidget
from vgcs.observe.dooaf import DOOAF_ROLE_GUN, DOOAF_ROLE_IMPACT, DOOAF_ROLE_INTENDED


def test_pending_lrf_dooaf_pick_role_only_for_setup() -> None:
    from vgcs.map.map_widget import _PendingLrfVideoPick

    setup = _PendingLrfVideoPick(
        purpose="dooaf_setup",
        u=0.5,
        v=0.5,
        pick_role=DOOAF_ROLE_GUN,
    )
    impact = _PendingLrfVideoPick(
        purpose="observation",
        u=0.8,
        v=0.5,
        label="Impact Target",
    )
    assert MapWidget._pending_lrf_dooaf_pick_role(setup) == DOOAF_ROLE_GUN
    assert MapWidget._pending_lrf_dooaf_pick_role(impact) is None
    assert MapWidget._pending_lrf_dooaf_pick_role(None) is None


def test_impact_pick_must_not_default_to_intended_target_role() -> None:
    from vgcs.map.map_widget import _PendingLrfVideoPick

    pending = _PendingLrfVideoPick(
        purpose="observation",
        u=0.83,
        v=0.49,
        label="Impact Target",
        pick_role="",
    )
    role = MapWidget._pending_lrf_dooaf_pick_role(pending)
    assert role is None
    assert DOOAF_ROLE_IMPACT != DOOAF_ROLE_INTENDED
