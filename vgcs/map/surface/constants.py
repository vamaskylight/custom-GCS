"""Map surface layout / motion constants."""

from __future__ import annotations

import os

_MAP_HUD_TOP_PX = 10
_MAP_ACTION_RAIL_LEFT_PX = 10
_MAP_ACTION_RAIL_TOP_PX = _MAP_HUD_TOP_PX
_NATIVE_CAM_RAIL_TOP_PX = _MAP_HUD_TOP_PX
_MAP_HUD_MARGIN_PX = 12
# Fallback only. The obstacle radar sits directly below the action rail, and
# this constant used to decide where. It was hand-summed and wrong: the buttons
# are 56 px with borders, not 54, so three of them plus spacing needed 184 and
# got 158. Adding Land made the gap obvious - the radar covered Return's label
# and hid "Go to" completely (reported 2026-09-10 with a screenshot).
#
# The layout now measures the rail instead, so this is only used before the
# rail exists. Do not go back to arithmetic here; the two drift apart every
# time a button is added.
# Five buttons of 50 px, 3 px apart, 5 px padding (ui_theme.FLIGHT_*, 2026-11-09 restyle).
_MAP_ACTION_RAIL_HEIGHT_PX = 5 * 50 + 4 * 3 + 2 * 5
_OBSTACLE_PANEL_TOP_PX = _MAP_ACTION_RAIL_TOP_PX + _MAP_ACTION_RAIL_HEIGHT_PX + 8
_OBSTACLE_PANEL_MAX_H_PX = 360

# The camera panels take their sizes from vgcs/app/ui_theme.py (2026-11-09 restyle).

# Fixed mini-video PiP (bottom-left).
_MINI_VIDEO_PIP_W_PX = 236
_MINI_VIDEO_PIP_H_PX = 132
_MAP_HUD_GLASS_BG = "rgba(26, 33, 45, 215)"
_MAP_HUD_GLASS_BORDER = "rgba(80, 92, 118, 107)"
_MAP_MOVE_ARM_SPEED_MPS = 1.0
_MAP_MOVE_DISARM_SPEED_MPS = 0.35
_MAP_MOVE_ARM_SAMPLES = 5
_MAP_MOVE_DISARM_SAMPLES = 15
_MAP_POSITION_MIN_MOVE_M = 2.0

MAP_BACKEND_BUILD = "2026-05-18-native2d-telemetry"

_WEB_MAP_RELAYOUT_JS = """
(function(){
  try {
    if (typeof window.__vgcsRelayoutMap2d === 'function') {
      window.__vgcsRelayoutMap2d();
      return;
    }
    var m2 = document.getElementById('map2d');
    var m3 = document.getElementById('map3d');
    if (m2) m2.style.display = 'block';
    if (m3) m3.style.display = 'none';
    if (typeof map !== 'undefined' && map) {
      map.invalidateSize(true);
      if (window.__vgcsPendingCenter && window.__vgcsPendingCenter.length === 2) {
        var zz = window.__vgcsPendingZoom;
        if (typeof zz !== 'number' || !isFinite(zz)) zz = map.getZoom();
        map.setView(window.__vgcsPendingCenter, zz, {animate: false});
      }
    }
  } catch (e) {
    try { console.log('[diag] relayout err=' + String(e)); } catch (e2) {}
  }
})();
"""
