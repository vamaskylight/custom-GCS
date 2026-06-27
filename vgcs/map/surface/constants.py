"""Map surface layout / motion constants."""

from __future__ import annotations

import os

_MAP_HUD_TOP_PX = 10
_MAP_ACTION_RAIL_LEFT_PX = 10
_MAP_ACTION_RAIL_TOP_PX = _MAP_HUD_TOP_PX
_NATIVE_CAM_RAIL_TOP_PX = _MAP_HUD_TOP_PX
_MAP_HUD_MARGIN_PX = 12
_MAP_ACTION_RAIL_HEIGHT_PX = 54 + 8 + 54
_OBSTACLE_PANEL_TOP_PX = _MAP_ACTION_RAIL_TOP_PX + _MAP_ACTION_RAIL_HEIGHT_PX + 8
_OBSTACLE_PANEL_MAX_H_PX = 360
_MAP_HUD_GLASS_BG = "rgba(26, 33, 45, 215)"
_MAP_HUD_GLASS_BORDER = "rgba(80, 92, 118, 107)"
_MAP_MOVE_ARM_SPEED_MPS = 1.0
_MAP_MOVE_DISARM_SPEED_MPS = 0.35
_MAP_MOVE_ARM_SAMPLES = 5
_MAP_MOVE_DISARM_SAMPLES = 15
_MAP_POSITION_MIN_MOVE_M = 2.0

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
