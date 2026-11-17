# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Walk the payload set of the drop demo and report how many carries survive.

The fault in `projects/samples/demos/worlds/debug/omniarm6_drop_fault.omniworld`
is INTERMITTENT in the only way a bitwise-deterministic simulator allows: the
physics repeats exactly, so the thing that varies has to be the part. Five
payloads go through the same cell; at the faulty friction two of them are
dropped and three are carried.

    python scripts/dev/drop_payload_sweep.py            # the faulty world
    python scripts/dev/drop_payload_sweep.py --both     # faulty, then the fix

Measured on this tree (see the demo README for machine attribution):

    newtonGroundMu 1.5 (faulty)   3 of 5 carried -- 0.30 and 0.35 kg DROP
    newtonGroundMu 6   (fixed)    5 of 5 carried

!! `placed` IS NOT A SUCCESS TEST. A dropped block regularly lands within the
0.10 m place tolerance and scores `placed=True`. This script reports `carried`
-- was the part still airborne in the gripper at the carry sample -- and prints
both so the difference is visible.

!! STAY INSIDE 0.15-0.35 kg. The behaviour is NOT monotonic in mass outside that
band: 0.45 kg holds at the faulty friction and drops at the healthy one. The
band where mass is an ordered dial is the band the demo uses.

Each run materialises a temporary sibling world next to the original (the
robot's URDF `url` is relative, so the copy has to live in the same directory)
and deletes it again, even on Ctrl-C.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import tempfile
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
DEBUG_WORLDS = REPO_ROOT / "projects" / "samples" / "demos" / "worlds" / "debug"
FAULTY = DEBUG_WORLDS / "omniarm6_drop_fault.omniworld"
FIXED = DEBUG_WORLDS / "omniarm6_drop_fault_fixed.omniworld"

PAYLOADS_KG = (0.15, 0.20, 0.25, 0.30, 0.35)

# Anchored on the whole physics line. `mass 0.30` alone would also be a fine
# anchor here, but the sibling trap in this world family is real: the string
# `newtonGroundMu 6` appears twice, once in a header comment, and a naive
# substitution patches the comment and silently produces a world that did not
# change at all.
MASS_RE = re.compile(r"(physics Physics \{ density -1 mass )([0-9.]+)( \})")
MU_RE = re.compile(r"^  newtonGroundMu ([0-9.]+)\s*$", re.M)


def world_mu(world: Path) -> str:
    m = MU_RE.search(world.read_text(encoding="utf-8"))
    return m.group(1) if m else "?"


def make_variant(world: Path, mass_kg: float, tmp_dir: Path) -> Path:
    """A sibling copy of `world` with the block mass rewritten."""
    src = world.read_text(encoding="utf-8")
    out, n = MASS_RE.subn(lambda m: "%s%.2f%s" % (m.group(1), mass_kg, m.group(3)),
                          src)
    if n != 1:
        raise SystemExit("expected exactly one block mass in %s, found %d"
                         % (world, n))
    # !! SIBLING, not tmp_dir: DEF OMNIARM6 URDFRobot's `url` is relative
    # ("../../../../robots/..."), and the URDF loader does not honour
    # omnisim://, so a copy elsewhere cannot find its robot.
    dest = world.with_name("_sweep_%s_%03d.omniworld"
                           % (os.getpid(), round(mass_kg * 100)))
    dest.write_text(out, encoding="utf-8", newline="\n")
    return dest


def run_one(world: Path, tmp_dir: Path, duration: int = 90) -> dict:
    """One headless pick. Returns the controller's result dict."""
    tag = world.stem
    env = dict(os.environ)
    env["PICK_AUTOQUIT"] = "1"          # ~12 s instead of the full duration
    # !! Without PICK_OUT the controller overwrites its own TRACKED
    # _real_pick_result.json next to its source.
    result = tmp_dir / ("%s.json" % tag)
    env["PICK_OUT"] = str(result)
    # !! Without this every run writes the SHARED repo-root omnisim_log.txt.
    env["OMNISIM_LOG_PATH"] = str(tmp_dir / ("%s.log" % tag))

    t0 = time.time()
    proc = subprocess.run(
        [sys.executable, "-m", "omnisim", "run-headless", str(world),
         "--duration", str(duration)],
        cwd=str(REPO_ROOT), env=env, capture_output=True, text=True)
    wall = time.time() - t0

    if not result.is_file():
        return {"error": "no result file; run-headless rc=%d\n%s"
                         % (proc.returncode, proc.stdout[-800:]), "wall_s": wall}
    out = json.loads(result.read_text(encoding="utf-8"))
    out["wall_s"] = wall
    # !! rc=1 is EXPECTED on a dropped run: PICK_AUTOQUIT makes the controller
    # call simulationQuit(1) when its own verdict is not ok. It is not a load
    # failure and the result JSON is complete.
    out["rc"] = proc.returncode
    return out


def sweep(world: Path, payloads=PAYLOADS_KG, duration: int = 90) -> list:
    mu = world_mu(world)
    print("\n=== %s  (newtonGroundMu %s) ===" % (world.name, mu))
    print("%-9s %-8s %-8s %-8s %-9s %s"
          % ("mass", "carried", "pinched", "placed", "drift", "wall"))
    rows = []
    with tempfile.TemporaryDirectory(prefix="omnisim_drop_sweep_") as td:
        tmp_dir = Path(td)
        for mass in payloads:
            variant = None
            try:
                variant = make_variant(world, mass, tmp_dir)
                r = run_one(variant, tmp_dir, duration=duration)
            finally:
                if variant is not None and variant.exists():
                    variant.unlink()
            if "error" in r:
                print("%-9s %s" % ("%.2f kg" % mass, r["error"].splitlines()[0]))
                rows.append({"mass_kg": mass, "error": r["error"]})
                continue
            print("%-9s %-8s %-8s %-8s %-9s %.1f s"
                  % ("%.2f kg" % mass,
                     "YES" if r.get("carried") else "DROP",
                     "yes" if r.get("pinched") else "no",
                     "yes" if r.get("placed") else "no",
                     "%.1f mm" % (1000 * r.get("drift_m", 0.0)),
                     r["wall_s"]))
            rows.append({"mass_kg": mass, "carried": bool(r.get("carried")),
                         "pinched": bool(r.get("pinched")),
                         "placed": bool(r.get("placed")),
                         "drift_mm": round(1000 * r.get("drift_m", 0.0), 1),
                         "wall_s": round(r["wall_s"], 1)})
    good = sum(1 for r in rows if r.get("carried"))
    print("-> %d of %d carried  (mu=%s)" % (good, len(rows), mu))
    placed_but_dropped = [r["mass_kg"] for r in rows
                          if r.get("placed") and not r.get("carried")]
    if placed_but_dropped:
        print("   !! %s landed inside the place tolerance after being DROPPED "
              "-- `placed` would have scored %s of %d as success."
              % (", ".join("%.2f kg" % m for m in placed_but_dropped),
                 sum(1 for r in rows if r.get("placed")), len(rows)))
    return rows


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--world", type=Path, default=FAULTY)
    ap.add_argument("--both", action="store_true",
                    help="sweep the faulty world and then the fixed one")
    ap.add_argument("--fixed", action="store_true",
                    help="sweep the fixed world only")
    ap.add_argument("--duration", type=int, default=90,
                    help="ceiling passed to run-headless; PICK_AUTOQUIT ends "
                         "each run at the controller's verdict (~12 s)")
    ap.add_argument("--json", metavar="PATH", help="also write the rows here")
    args = ap.parse_args(argv)

    worlds = [FAULTY, FIXED] if args.both else [FIXED if args.fixed else args.world]
    t0 = time.time()
    report = {}
    for w in worlds:
        if not w.is_file():
            raise SystemExit("no such world: %s" % w)
        report[w.name] = sweep(w, duration=args.duration)
    print("\ntotal wall: %.1f s" % (time.time() - t0))

    if args.both:
        f = sum(1 for r in report[FAULTY.name] if r.get("carried"))
        g = sum(1 for r in report[FIXED.name] if r.get("carried"))
        print("FAULTY %d/5 carried   FIXED %d/5 carried" % (f, g))
    if args.json:
        Path(args.json).write_text(json.dumps(report, indent=1), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
