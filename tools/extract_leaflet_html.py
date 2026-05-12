"""One-shot: extract LEAFLET_HTML from git revision into vgcs/map/legacy_leaflet_map.html."""
from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REV = sys.argv[1] if len(sys.argv) > 1 else "e48c1a7"
path_in_repo = "vgcs/map/map_widget.py"

raw = subprocess.check_output(
    ["git", "show", f"{REV}:{path_in_repo}"],
    cwd=ROOT,
    text=True,
    encoding="utf-8",
    errors="replace",
)
# Closing delimiter is """ followed by newline then non-quote (class/def/comment).
m = re.search(
    r'^LEAFLET_HTML = """(.*?)"""\s*\n(?:class |def |\w|\Z)',
    raw,
    flags=re.DOTALL | re.MULTILINE,
)
if not m:
    print("Could not parse LEAFLET_HTML", file=sys.stderr)
    sys.exit(1)
html = m.group(1)
out = ROOT / "vgcs" / "map" / "legacy_leaflet_map.html"
out.parent.mkdir(parents=True, exist_ok=True)
out.write_text(html, encoding="utf-8")
print(f"Wrote {out} ({len(html)} chars)")
