"""What product surface did the agent actually TOUCH?

A defect can only be encountered on a surface the agent reaches. Before
reading any before/after difference, this says which of the fixed surfaces
A1_husky_swarm_10 puts an agent on at all -- and therefore which fixes this
task can measure and which it structurally cannot.
"""
import json
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from defect_scan import load_pairs  # noqa: E402

SURFACES = {
    "harness_service": r"(?i)(omnisim_harness|:6789|:6790|127\.0\.0\.1:678)",
    "  /world/load": r"(?i)/world/load",
    "  /world/screenshot": r"(?i)/world/screenshot",
    "  /sim/reset": r"(?i)/sim/reset",
    "  /sim/step": r"(?i)/sim/step",
    "  /robots": r"(?i)/robots\b",
    "  /scene/*": r"(?i)/scene/",
    "capture_service": r"(?i)(omnisim_capture|:6791|:6792)",
    "  /capture/screenshot": r"(?i)/capture/screenshot",
    "set_viewpoint.py": r"(?i)set_viewpoint(\.py)?",
    "docs/reference read": r"(?i)docs[\\/]reference[\\/]",
    "run-headless": r"(?i)run-headless|run_headless",
    "supervisor/controller": r"(?i)(Supervisor\(\)|from omnisim import|from controller import)",
}


def main():
    rows = []
    for cell in sys.argv[1:]:
        pairs, _t0, _src = load_pairs(cell)
        blob = "\n".join("%s %s" % (p[2] or "", p[3]) for p in pairs)
        res = "\n".join(p[4][:8000] for p in pairs)
        counts = {}
        for name, rx in SURFACES.items():
            counts[name] = len(re.findall(rx, blob)) + 0
        rows.append((os.path.basename(cell), len(pairs), counts))
    names = list(SURFACES)
    w = max(len(n) for n in names) + 2
    hdr = "surface".ljust(w) + "".join(r[0][-14:].rjust(16) for r in rows)
    print(hdr)
    print("-" * len(hdr))
    print("(tool calls)".ljust(w) + "".join(str(r[1]).rjust(16) for r in rows))
    for n in names:
        print(n.ljust(w) + "".join(str(r[2][n]).rjust(16) for r in rows))


if __name__ == "__main__":
    main()
