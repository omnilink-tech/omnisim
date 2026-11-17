"""Record WHAT was compared, so the arms can be re-derived later."""
import hashlib
import json
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tree_digest import tree_digest  # noqa: E402

REAL = "o:/omnisim"
ARMS = {"A_before": ("o:/omnisim-ab/armA", "fc98988675bd7cd72712846f777db7a51bf92a74"),
        "B_after": ("o:/omnisim-ab/armB", "8976b86d1cd2d4c9a70a1e0dd20acd71bf889d2d")}
PRODUCT_PATHS = ["scripts/harness", "scripts/capture", "docs/reference"]
PRODUCT_FILES = ["resources/nodes/WorldInfo.wrl", "scripts/dev/set_viewpoint.py",
                 "AGENTS.md"]


def sha(p):
    try:
        with open(p, "rb") as f:
            return hashlib.sha256(f.read()).hexdigest()
    except OSError:
        return None


def main():
    out = {"utc": subprocess.run(["date", "-u", "+%FT%TZ"], capture_output=True,
                                 text=True).stdout.strip(),
           "task": "A1_husky_swarm_10", "model": "claude-opus-5",
           "cells_per_arm": 3, "session_budget_s": 600,
           "machine": subprocess.run(
               ["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"],
               capture_output=True, text=True).stdout.strip(),
           "arms": {}}
    for name, (root, _declared) in ARMS.items():
        commit = subprocess.run(["git", "-C", root, "rev-parse", "HEAD"],
                                capture_output=True, text=True).stdout.strip()
        a = {"worktree": os.path.abspath(root), "commit": commit,
             "engine_sha256": sha(os.path.join(
                 root, "msys64/mingw64/bin/omnisim-bin.exe")),
             "libcontroller_sha256": sha(os.path.join(
                 root, "lib/controller/Controller.dll")),
             "product": {}}
        for p in PRODUCT_PATHS:
            full = os.path.join(root, p)
            if os.path.isdir(full):
                d, n = tree_digest(full)
                a["product"][p] = {"tree_sha256": d, "files": n}
        for p in PRODUCT_FILES:
            a["product"][p] = sha(os.path.join(root, p))
        d, n = tree_digest(os.path.join(
            root, "tests/benchmarks/agentbench/cc_lane"))
        a["instrument_cc_lane_sha256"] = d
        a["instrument_cc_lane_files"] = n
        out["arms"][name] = a
    same = (out["arms"]["A_before"]["instrument_cc_lane_sha256"] ==
            out["arms"]["B_after"]["instrument_cc_lane_sha256"])
    out["instrument_identical"] = same
    out["instrument_source"] = "HEAD (8976b86d1), overlaid into both worktrees"
    dest = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "setup_manifest.json")
    with open(dest, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=1)
    print(json.dumps(out, indent=1))
    print("\nwrote %s" % dest)


if __name__ == "__main__":
    main()
