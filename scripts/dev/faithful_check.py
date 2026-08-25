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

"""faithful_check — RETIRED (2026-08-08): this tool's entire question needed ODE, and ODE is gone.

WHAT IT DID. newton_coverage.py measures gate ELIGIBILITY (which backend resolves); this measured
FAITHFULNESS: does an eligible world actually TRACK ODE? For each world it ran the sim twice --
Newton (default) and forced-ODE (OMNISIM_FORCE_ODE=1) -- with the in-binary OMNISIM_PROBE_TRAJ probe
(every step it dumps each articulation ROOT's world position), then compared the two trajectories per
root over time, classifying on an EARLY window (before chaos amplifies) plus the quasi-static worlds,
because two different solvers legitimately diverge on a genuinely dynamic scene.

WHY IT CANNOT RUN. **src/ode was DELETED (commit bdc02139).** The reference arm cannot exist, and both
ways it could fail are SILENT -- the tool would print a plausible table either way:

  * The current engine IGNORES OMNISIM_FORCE_ODE (verified 2026-08-08: the run comes up on Newton and
    writes a normal .newton.json sidecar). The "ODE" arm would then be a second NEWTON run, every world
    would score PERFECTLY faithful, and the headline would be 100% against the >=99% bar -- exactly the
    number this campaign wanted, and entirely fictional.
  * An earlier build the same day still honoured it, and then the run was FROZEN (on
    tests/physics/worlds/contact_points.omniworld a body sat bit-identical to its authored pose for 3000 ms).
    Every world would have scored maximally UNFAITHFUL in proportion to how much it moved -- a ranking
    of how alive each world is, labelled as fidelity.

WHAT THIS MEANS FOR THE CAMPAIGN, stated rather than quietly dropped: corpus faithfulness was last
measured at ~35-40% against a >=99% bar, and the campaign's Wave A items A3/A4 (re-arm the coverage
meter, run faithful_check over all 701 worlds) are now UNANSWERABLE, not merely unrun. The
substitution that happened in practice -- scripts/dev/newton_readiness_sweep.py -- answers
LOAD-AND-STEP, not fidelity. docs/developer/ode-retirement-campaign.md records that gap; this refusal
is here so nobody re-derives a fidelity number from a still image.

The frozen ODE reference values that survive are in tests/goldens/ode_oracle_goldens.json.
For a live per-world verdict use:  python scripts/dev/newton_readiness_sweep.py --glob "<pattern>"
"""

import argparse
import glob as globmod
import math
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

SIM_MS = 1000.0       # how much sim time to compare (the probe exits after this)
EARLY_MS = 300.0      # the early window the faithful verdict keys on (before chaos amplifies)
FAITHFUL_M = 0.03     # early-window max root divergence (m) that counts as faithful
DRIFT_M = 0.30        # below this = drifting; above = diverging
LOAD_TIMEOUT_S = 90
RETRIES = 5

RE_LINE = re.compile(r"^(-?\d+(?:\.\d+)?)\t(.*)\t(-?\d+(?:\.\d+)?)\t(-?\d+(?:\.\d+)?)\t(-?\d+(?:\.\d+)?)(?:\t([AR]))?$")


def _one_run(world, out, force_ode):
    try:
        os.remove(out)
    except OSError:
        pass
    env = dict(os.environ)
    env["OMNISIM_PROBE_TRAJ"] = out
    env["OMNISIM_PROBE_TRAJ_MS"] = str(int(SIM_MS))
    env.pop("OMNISIM_LEGACY", None)
    env.pop("OMNISIM_FORCE_WREN", None)
    if force_ode:
        # UNREACHABLE: main() refuses before any run (src/ode deleted, bdc02139). Raise rather
        # than set the retired var, so re-enabling the reference arm cannot silently produce a
        # frozen-scene "trajectory" that this tool would score as maximal unfaithfulness.
        raise AssertionError(
            "the forced-ODE reference arm was retired with src/ode (commit bdc02139); "
            "setting OMNISIM_FORCE_ODE=1 yields a run with no physics, not an ODE run")
    env.pop("OMNISIM_FORCE_ODE", None)
    # Newton's intended config for static-world fidelity: register static colliders (so the floor's
    # plane/mesh exists -- else dynamic props fall through it) + substep contact. Caller-set values win.
    env.setdefault("OMNISIM_NEWTON_STATICS", "1")
    env.setdefault("OMNISIM_NEWTON_SUBSTEPS", "4")
    env.setdefault("WEBOTS_HOME", REPO)
    env.setdefault("OMNISIM_HOME", REPO)
    proc = subprocess.Popen([BIN, world, "--mode=fast", "--no-rendering", "--batch", "--stdout", "--stderr"],
                            cwd=REPO, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    deadline = time.time() + LOAD_TIMEOUT_S
    while time.time() < deadline:
        if proc.poll() is not None:  # the probe _Exit()s when the sim-time budget is reached
            break
        time.sleep(0.5)
    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            proc.kill()
    if not os.path.exists(out) or os.path.getsize(out) == 0:
        return None
    traj = {}  # name -> {tms -> (x,y,z)}
    tags = {}  # name -> 'A' (articulated robot/mechanism) | 'R' (rigid free prop / static); '' if untagged
    with open(out, "r", errors="replace") as fh:
        for line in fh:
            m = RE_LINE.match(line.rstrip("\n"))
            if m:
                tms = round(float(m.group(1)), 1)
                traj.setdefault(m.group(2), {})[tms] = (float(m.group(3)), float(m.group(4)), float(m.group(5)))
                tags[m.group(2)] = m.group(6) or tags.get(m.group(2), "")
    return (traj, tags) if traj else None


def capture(world, tag, force_ode):
    out = os.path.join(REPO, "_scratch", "traj_%s_%s.txt" % (re.sub(r"[^A-Za-z0-9_.-]", "_", os.path.basename(world)), tag))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    for _ in range(RETRIES):
        traj = _one_run(world, out, force_ode)
        if traj:
            return traj
        time.sleep(1.5)
    return None


def compare(newton, ode, tags):
    """Per shared root, the max position divergence over the EARLY window and the FULL window, with the
    root's A/R tag (articulated robot vs rigid free-prop/static) attached for the meter split."""
    rows = []
    for name in sorted(set(newton) & set(ode)):
        nt, ot = newton[name], ode[name]
        early = full = 0.0
        for tms in sorted(set(nt) & set(ot)):
            nx, ny, nz = nt[tms]
            ox, oy, oz = ot[tms]
            d = math.sqrt((nx - ox) ** 2 + (ny - oy) ** 2 + (nz - oz) ** 2)
            full = max(full, d)
            if tms <= EARLY_MS:
                early = max(early, d)
        rows.append((name, early, full, tags.get(name, "")))
    return rows


def verdict_of(early):
    return "FAITHFUL" if early < FAITHFUL_M else ("drift   " if early < DRIFT_M else "DIVERGE ")


def main():
    ap = argparse.ArgumentParser(
        description="RETIRED: Newton-vs-ODE faithful-match check (W5.4). ODE was deleted.")
    ap.add_argument("--worlds", nargs="*")
    ap.add_argument("--glob")
    args = ap.parse_args()

    # ---- REFUSAL (2026-08-08) --------------------------------------------
    print("[faithful] RETIRED -- this tool will not run. It measured Newton against a forced-ODE")
    print("[faithful] reference arm, and src/ode was DELETED (commit bdc02139).")
    print("[faithful]")
    print("[faithful] It is refused rather than left runnable because it would NOT fail loudly:")
    print("[faithful] OMNISIM_FORCE_ODE now selects a backend with no implementation, which yields a")
    print("[faithful] FROZEN scene (measured 2026-08-08: zero motion in 3000 ms), so every world")
    print("[faithful] would score maximally UNFAITHFUL in proportion to how much it moved, and the")
    print("[faithful] table would look like a fidelity report.")
    print("[faithful]")
    print("[faithful] Corpus faithfulness was last ~35-40%% against a >=99%% bar and is now")
    print("[faithful] UNANSWERABLE, not merely unrun (campaign items A3/A4). The tool that replaced")
    print("[faithful] it in practice answers LOAD-AND-STEP, not fidelity:")
    print("[faithful]     python scripts/dev/newton_readiness_sweep.py --glob \"<pattern>\"")
    print("[faithful] Frozen ODE reference values: tests/goldens/ode_oracle_goldens.json")
    return 2

    if not os.path.exists(BIN):
        print("[faithful] omnisim-bin not found -- build it first.")
        return 0
    if args.worlds:
        corpus = [w if os.path.isabs(w) else os.path.join(REPO, w) for w in args.worlds]
    elif args.glob:
        corpus = sorted(globmod.glob(os.path.join(REPO, args.glob), recursive=True))
    else:
        print("[faithful] pass --worlds or --glob")
        return 0

    print("[faithful] Newton-vs-ODE trajectory match -- %d worlds, %dms sim, early window %dms (run from PowerShell)"
          % (len(corpus), int(SIM_MS), int(EARLY_MS)))
    print("[faithful] headline = ROBOT (articulated) verdict; free props/static (rigid) reported for context --")
    print("[faithful] a loose prop rolling to a different-but-valid rest pose under a different solver is a")
    print("[faithful] fidelity FACT, not a Newton bug (untunable: more substeps EJECT it -- measured).\n")
    n_faithful = n_total = 0          # ROBOT (articulated) worlds
    n_prop_clean = n_prop_total = 0   # worlds whose rigid props also stayed faithful
    untagged_warned = False
    for w in corpus:
        rel = os.path.relpath(w, REPO)
        nt = capture(w, "newton", False)
        od = capture(w, "ode", True)
        if nt is None or od is None:
            print("  %-52s  NO DATA (newton=%s ode=%s)" % (rel[-52:], nt is not None, od is not None))
            continue
        nt_traj, nt_tags = nt
        od_traj, _od_tags = od
        rows = compare(nt_traj, od_traj, nt_tags)
        if not rows:
            print("  %-52s  NO SHARED ROOTS" % rel[-52:])
            continue
        tagged = any(r[3] for r in rows)
        if not tagged and not untagged_warned:
            print("  [faithful] NOTE: roots are untagged (stale binary?) -- robot/prop split unavailable; "
                  "treating ALL roots as one bucket.")
            untagged_warned = True
        art = [r for r in rows if r[3] == "A"] if tagged else rows
        rig = [r for r in rows if r[3] != "A"] if tagged else []
        # ROBOT (articulated) verdict -- the headline
        if art:
            r_early = max(r[1] for r in art)
            r_full = max(r[2] for r in art)
            r_worst = max(art, key=lambda r: r[1])[0]
            r_verd = verdict_of(r_early)
            n_total += 1
            if r_early < FAITHFUL_M:
                n_faithful += 1
        else:
            r_early = r_full = 0.0
            r_worst = "-"
            r_verd = "(no robot)"
        # RIGID (free prop / static) context
        if rig:
            p_early = max(r[1] for r in rig)
            p_worst = max(rig, key=lambda r: r[1])[0]
            n_prop_total += 1
            if p_early < FAITHFUL_M:
                n_prop_clean += 1
            prop_str = "  | props %s prop_early=%.3fm (%s)" % (verdict_of(p_early).strip(), p_early, p_worst[:18])
        else:
            prop_str = ""
        print("  %-52s  robot %s robot_early=%.3fm full=%.3fm (%s)%s"
              % (rel[-52:], r_verd, r_early, r_full, r_worst[:18], prop_str))

    print("\n" + "=" * 92)
    if n_total:
        print("[faithful] ROBOT fidelity: %d/%d worlds FAITHFUL (articulated-root early divergence < %.0f mm)."
              % (n_faithful, n_total, FAITHFUL_M * 1000))
    if n_prop_total:
        print("[faithful] free-prop bucket: %d/%d worlds also had every loose prop stay < %.0f mm. Props above"
              % (n_prop_clean, n_prop_total, FAITHFUL_M * 1000))
        print("[faithful] that bar is EXPECTED solver chaos on free bodies, NOT a Newton bug -- read the robot column.")
    if not n_total and not n_prop_total:
        print("[faithful] no worlds compared.")
    print("=" * 92)
    return 0


if __name__ == "__main__":
    sys.exit(main())
