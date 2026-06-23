import re
import sys
from pathlib import Path

p = Path(sys.argv[1])
html = p.read_text(encoding="utf-8")

for m in re.finditer(
    r"hero-kpi-val'>([^<]+)</span><span class='hero-kpi-label'>([^<]+)",
    html,
):
    print("KPI:", m.group(2).strip(), "=", m.group(1).strip())

for m in re.finditer(r"label-col'>([^<]+)</td><td>([^<]{1,300})", html):
    lab, val = m.group(1), m.group(2)
    if any(
        k in lab.lower()
        for k in ("elev", "height", "up / down", "vertical", "miss along", "miss right")
    ):
        print(f"{lab} => {val[:100]}")

for m in re.finditer(r"exec-big'>([^<]+)", html):
    print("exec miss:", m.group(1))
