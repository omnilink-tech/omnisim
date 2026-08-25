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

"""newton_coverage — the Newton coverage meter (newton-ode-replacement-plan.md W0.2).

THE dashboard for the Newton->ODE-replacement effort. For each world in the corpus it loads the world
headless and reads the OMNISIM_PROBE_GATE report that OmWorld::finalize() writes: every top-level
articulation's resolved physics backend + capability-gate verdict. It aggregates that into the headline
coverage number + the fallback-reason histogram that tells us which gap to close next.

The probe is AUTHORITATIVE (the real OmSolid gate, not a Python re-derivation that would drift) and
COMPLETE (every topSolid, including pinned/explicit robots a gate-only probe misses).

Per articulation the probe records:  backend=<ode|newton|auto>  gate=<capable|mesh|joint>
  - backend in {auto, newton}  -> Newton-ELIGIBLE (runs Newton when the runtime is present)
  - backend == ode             -> WAS "ODE". SINCE 2026-08-08 THIS IS A DEFECT, NOT A CATEGORY: src/ode
                                  was DELETED (commit bdc02139), so an articulation resolving to `ode`
                                  has NO PHYSICS IMPLEMENTATION behind it -- it will not fall, will not
                                  collide, and will not error. `gate` still says WHY it resolved that
                                  way: {mesh, joint} = a Newton capability gap, `capable` = a deliberate
                                  world pin that is now a live bug. Read any non-zero `ode` count in the
                                  output as a to-do list, not as a coverage percentage.

WHAT THIS METER NO LONGER HAS A PARTNER FOR. It measures ELIGIBILITY (what the gate allows). The plan's
full metric also required "matches ODE within the physics-oracle tolerance", and that faithful-match
layer (scripts/dev/faithful_check.py) is now RETIRED and refuses to run -- its reference arm was ODE.
So eligibility is not a stand-in for fidelity, and nothing in the tree measures fidelity today; see
docs/developer/ode-retirement-campaign.md for that gap.

Run from PowerShell. Build omnisim-bin first (the probe lives in it).

Usage:
    python scripts/dev/newton_coverage.py                       # default corpus (robots + samples)
    python scripts/dev/newton_coverage.py --worlds a.wbt b.wbt  # explicit worlds
    python scripts/dev/newton_coverage.py --glob "projects/robots/**/worlds/*.wbt"
    python scripts/dev/newton_coverage.py --json _scratch/coverage.json
"""

import argparse
import glob as globmod
import json
import os
import re
import subprocess
import sys
import time

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if REPO not in sys.path:
    sys.path.insert(0, REPO)
from omnisim.paths import resolve_omnisim_binary  # noqa: E402

# Cross-platform binary resolution (Windows msys64, Linux bin/, macOS bundle).
# Fall back to the legacy Windows path so the exists() check still prints a
# helpful location on an unbuilt checkout.
BIN = resolve_omnisim_binary() or os.path.join(REPO, "msys64", "mingw64", "bin", "omnisim-bin.exe")

RE_ART = re.compile(r"^articulation=(.*) backend=(\S+) gate=(\S+)\s*$")
LOAD_TIMEOUT_S = 90  # heavy factory worlds take ~30s to finalize; the probe exits(0) the instant it's done


def default_corpus():
    pats = ["projects/robots/**/worlds/*.omniworld", "projects/samples/**/worlds/*.omniworld",
            "projects/robots/**/worlds/*.wbt", "projects/samples/**/worlds/*.wbt"]
    worlds = []
    for p in pats:
        for w in globmod.glob(os.path.join(REPO, p), recursive=True):
            if os.path.basename(w).startswith("."):  # skip .omniquad_rl_env* scratch worlds
                continue
            worlds.append(w)
    return sorted(set(worlds))


def _launch_and_capture(world, out):
    """One attempt: launch headless, poll for the report, kill, parse. Returns arts list or None."""
    try:
        os.remove(out)
    except OSError:
        pass
    env = dict(os.environ)
    env["OMNISIM_PROBE_GATE"] = out
    # RETIRED KNOBS, SCRUBBED ON PURPOSE: both name a deleted backend (bdc02139) and a stale
    # export in the parent shell would make every articulation report `ode`.
    env.pop("OMNISIM_FORCE_ODE", None)
    env.pop("OMNISIM_LEGACY", None)
    env.setdefault("WEBOTS_HOME", REPO)
    env.setdefault("OMNISIM_HOME", REPO)
    # POLL for the report file + kill (do NOT wait for the process to exit): the sweep writes the whole
    # report then close()s it (so a non-empty file IS complete) and only then exit(0)s -- and that exit can
    # hang in static destructors on heavy worlds. Polling makes us robust to that: as soon as the report is
    # there we have everything, so kill and move on. A world that never finalizes never writes -> timeout.
    proc = subprocess.Popen([BIN, world, "--mode=fast", "--no-rendering", "--batch", "--stdout", "--stderr"],
                            cwd=REPO, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    deadline = time.time() + LOAD_TIMEOUT_S
    while time.time() < deadline:
        if os.path.exists(out) and os.path.getsize(out) > 0:
            time.sleep(0.4)  # the report is flushed at close() in one shot, so non-empty == complete
            break
        if proc.poll() is not None:
            break
        time.sleep(0.5)
    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            proc.kill()
            try:
                proc.wait(timeout=4)
            except subprocess.TimeoutExpired:
                pass
    if not os.path.exists(out):
        return None
    arts = []
    with open(out, "r", errors="replace") as fh:
        for line in fh:
            m = RE_ART.match(line.rstrip("\n"))
            if m:
                arts.append({"name": m.group(1), "backend": m.group(2), "gate": m.group(3)})
    return arts if arts else None


def probe_world(world):
    """Load the world headless; return articulation dicts or None. Retries the back-to-back launch race
    (a fresh launch is reliable; rapid relaunches occasionally miss -- the physics oracle's §3.5 flake)."""
    out = os.path.join(REPO, "_scratch", "cov_%s.txt" % re.sub(r"[^A-Za-z0-9_.-]", "_", os.path.basename(world)))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    for _ in range(5):
        arts = _launch_and_capture(world, out)
        if arts:
            return arts
        time.sleep(2.0)  # let the prior process fully die + ports free before relaunching
    return None


def classify(arts):
    """Per-world rollup."""
    total = len(arts)
    eligible = sum(1 for a in arts if a["backend"] in ("newton", "auto"))
    ode = total - eligible
    gaps = [a for a in arts if a["gate"] in ("mesh", "joint")]
    if total == 0:
        cls = "empty"
    elif eligible == total:
        cls = "newton"      # every articulation is Newton-eligible
    elif eligible == 0:
        cls = "ode"
    else:
        cls = "mixed"
    return {"total": total, "eligible": eligible, "ode": ode, "gaps": gaps, "class": cls}


def main():
    ap = argparse.ArgumentParser(description="Newton coverage meter (newton-ode-replacement-plan.md W0.2)")
    ap.add_argument("--worlds", nargs="*", help="explicit world paths (default: robots + samples corpus)")
    ap.add_argument("--glob", help="glob pattern (relative to repo root) for the corpus")
    ap.add_argument("--json", help="also write the full result as JSON to this path")
    args = ap.parse_args()

    if not os.path.exists(BIN):
        print("[coverage] omnisim-bin not found at %s -- build it first." % BIN)
        return 0

    if args.worlds:
        corpus = [w if os.path.isabs(w) else os.path.join(REPO, w) for w in args.worlds]
    elif args.glob:
        corpus = sorted(globmod.glob(os.path.join(REPO, args.glob), recursive=True))
    else:
        corpus = default_corpus()
    if not corpus:
        print("[coverage] empty corpus.")
        return 0

    print("[coverage] Newton coverage meter -- %d worlds (run from PowerShell)\n" % len(corpus))
    results = {}
    tot_art = tot_elig = 0
    mesh_gaps = joint_gaps = 0
    cls_count = {"newton": 0, "mixed": 0, "ode": 0, "empty": 0, "noload": 0}
    gap_worlds = {}  # world -> set of gap reasons

    for w in corpus:
        rel = os.path.relpath(w, REPO)
        arts = probe_world(w)
        if arts is None:
            cls_count["noload"] += 1
            print("  %-58s  NO DATA (load failed / timed out)" % rel[-58:])
            results[rel] = {"class": "noload"}
            continue
        c = classify(arts)
        results[rel] = {"class": c["class"], "total": c["total"], "eligible": c["eligible"],
                        "articulations": arts}
        tot_art += c["total"]
        tot_elig += c["eligible"]
        cls_count[c["class"]] = cls_count.get(c["class"], 0) + 1
        for a in c["gaps"]:
            mesh_gaps += 1 if a["gate"] == "mesh" else 0
            joint_gaps += 1 if a["gate"] == "joint" else 0
            gap_worlds.setdefault(rel, set()).add(a["gate"])
        tag = {"newton": "NEWTON", "mixed": "mixed ", "ode": "ODE   ", "empty": "empty "}[c["class"]]
        gapstr = ""
        if c["gaps"]:
            gapstr = "  gaps: " + ",".join(sorted({a["gate"] for a in c["gaps"]}))
        print("  %-58s  %s  %d/%d eligible%s" % (rel[-58:], tag, c["eligible"], c["total"], gapstr))

    nloaded = len([r for r in results.values() if r.get("class") != "noload"])
    print("\n" + "=" * 78)
    print("[coverage] SUMMARY")
    print("  worlds: %d total, %d loaded, %d no-data" % (len(corpus), nloaded, cls_count["noload"]))
    if nloaded:
        full = cls_count["newton"]
        print("  WORLD coverage   : %d/%d (%.0f%%) run entirely on Newton (every articulation eligible)"
              % (full, nloaded, 100.0 * full / nloaded))
        print("                     (%d mixed, %d all-ODE, %d empty)"
              % (cls_count["mixed"], cls_count["ode"], cls_count["empty"]))
    if tot_art:
        print("  ARTICULATION cov : %d/%d (%.0f%%) of all top-level articulations are Newton-eligible"
              % (tot_elig, tot_art, 100.0 * tot_elig / tot_art))
    print("  CAPABILITY GAPS  : %d mesh, %d joint (the W1/W2 targets -- what to close first)"
          % (mesh_gaps, joint_gaps))
    if gap_worlds:
        print("  worlds with a gap:")
        for w in sorted(gap_worlds):
            print("    %-56s  %s" % (w[-56:], ",".join(sorted(gap_worlds[w]))))
    print("=" * 78)

    if args.json:
        with open(args.json, "w") as fh:
            json.dump({"summary": {"worlds": len(corpus), "loaded": nloaded,
                                   "world_newton": cls_count["newton"], "art_total": tot_art,
                                   "art_eligible": tot_elig, "mesh_gaps": mesh_gaps,
                                   "joint_gaps": joint_gaps},
                       "worlds": results}, fh, indent=2)
        print("[coverage] wrote %s" % args.json)
    return 0


if __name__ == "__main__":
    sys.exit(main())
