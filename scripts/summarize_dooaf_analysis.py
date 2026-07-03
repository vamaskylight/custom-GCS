#!/usr/bin/env python3
"""Print key metrics from analyze_dooaf_recording JSON."""

from __future__ import annotations

import json
import sys
from pathlib import Path


def main() -> int:
    path = Path(sys.argv[1])
    r = json.loads(path.read_text(encoding="utf-8"))
    print(f"duration_s={r['duration_s']} pan_segments={len(r['pan_segments'])}")
    print(f"gun_zone_drifts={len(r['gun_zone_drifts'])} overlay_drifts={len(r['overlay_drifts'])}")
    for d in r["gun_zone_drifts"][:15]:
        print(
            f"  gun t={d['t0_s']:.1f}-{d['t1_s']:.1f} "
            f"drift={d['drift_uv']:.4f} flow={d.get('mean_flow_px', 0):.1f} "
            f"{d['from_uv']} -> {d['to_uv']}"
        )
    high = [
        (s["t_s"], s.get("mean_flow_px", 0))
        for s in r["samples"]
        if float(s.get("mean_flow_px", 0)) > 8.0
    ]
    print(f"high_motion_samples={len(high)}")
    for t, f in high[:25]:
        print(f"  t={t:.1f} flow={f:.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
