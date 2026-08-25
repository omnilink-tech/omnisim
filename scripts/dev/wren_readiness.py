#!/usr/bin/env python3
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
"""wren_readiness.py -- is it safe to delete WREN yet?

WHY THIS EXISTS. "Are we ready to delete WREN?" has been answerable only by argument, and an
argument is exactly the wrong instrument for a one-way door. This runs the eight checks
defined in docs/developer/wren-deletion-runbook.md and prints one line each. Ready means all
eight green. Anything else names the red line.

THE TWO RULES THAT MAKE IT HONEST
  * A check this tool CANNOT run is reported MANUAL or ASSUMED. It is never counted green.
    A readiness tool that green-lights what it did not measure is worse than no tool.
  * R5 is "no NEW red", not "all green". Several tests are red on BOTH renderers today
    (camera_color, pen_box, camera_recognition, a cadshape_node wgpu panic). So the
    pre-deletion baseline must be RECORDED (`--record-baseline`) while WREN still works, and
    compared later. Recording it after the fact is not possible, which is why this exists now
    rather than at D1.

  python scripts/dev/wren_readiness.py                 # run what can be run
  python scripts/dev/wren_readiness.py --record-baseline
  python scripts/dev/wren_readiness.py --json out.json

Engine-touching checks are wrapped in scripts/dev/thermal_guard.py (75 C ceiling) because this
is the owner's thermally limited laptop; --no-guard opts out and says so in the output.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
BASELINE = REPO / "docs" / "developer" / "wren-readiness-baseline.json"

GREEN, RED, MANUAL, ASSUMED, SKIP = "GREEN", "RED", "MANUAL", "ASSUMED", "SKIPPED"

# R7 -- the silent-failure list. Every one of these compiles clean when broken, which is why
# it cannot be automated away and must not be quietly counted green.
R7_ITEMS = [
    "cloth/softbody still ANIMATE (not frozen at rest pose) on the wgpu main view",
    "gizmo handles are DRAWN where they are draggable (arrow.obj / circular_arrow.obj present)",
    "all 21 View > Optional Rendering items still respond (OmWrenRenderingContext survived)",
    "video recording produces a real file (OmVideoRecorder's readback path)",
    "--no-window renders (the second, headless window)",
    "the GPU-memory field in About shows a real figure OR an explicit unavailable (never 0 MB)",
]


def run(cmd, timeout=1800, guard=True):
    """Run a command, optionally under the thermal guard. Returns (rc, output).

    ⚠️ TREE-KILL ON TIMEOUT, NOT CHILD-KILL. The first baseline recording DEADLOCKED on its
    7th world: subprocess.run(timeout=...) killed the direct child (the thermal guard) when
    the timeout fired, then kept waiting for the captured PIPES to close -- and the orphaned
    grandchild engine inherited those pipe handles and never exits, so the wait never ends.
    That is the documented Windows orphan problem wearing a Python hat: the timeout fired
    AND the loop still hung forever (engine alive 20+ min against a 7-min timeout, zero
    further [r5] lines). So: Popen, and on expiry kill the whole process TREE first, then
    drain the pipes -- which now close, because nothing holds them.
    """
    if guard:
        cmd = [sys.executable, str(REPO / "scripts" / "dev" / "thermal_guard.py"),
               "run", "--ceiling", "75", "--interval", "5", "--"] + cmd
    try:
        p = subprocess.Popen(cmd, cwd=str(REPO), stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, text=True, errors="replace")
    except OSError as e:
        return 127, f"could not run: {e}"
    try:
        out, _ = p.communicate(timeout=timeout)
        return p.returncode, out or ""
    except subprocess.TimeoutExpired:
        if os.name == "nt":
            subprocess.run(["taskkill", "/T", "/F", "/PID", str(p.pid)],
                           capture_output=True, text=True)
        else:
            p.kill()
        try:
            out, _ = p.communicate(timeout=30)
        except subprocess.TimeoutExpired:
            out = ""
        # Sweep ORPHANED engines only -- an engine whose parent is dead is free heat with no
        # owner (the documented reclaim rule). ⚠️ Never a bare `taskkill /IM omnisim-bin.exe`:
        # this laptop is shared with other lanes, and a by-name sweep would kill THEIR live
        # engines, whose parents are alive and which are not ours to stop.
        if os.name == "nt":
            _reap_orphaned_engines()
        return 124, (out or "") + "\nTIMEOUT (process tree killed)"


def _reap_orphaned_engines():
    """Kill omnisim-bin processes whose parent no longer exists. Best-effort, quiet."""
    try:
        q = subprocess.run(
            ["wmic", "process", "where", "name='omnisim-bin.exe'",
             "get", "processid,parentprocessid"], capture_output=True, text=True, timeout=15)
        alive = subprocess.run(["tasklist", "/FO", "CSV", "/NH"],
                               capture_output=True, text=True, timeout=15)
        live_pids = {ln.split('","')[1].strip('"') for ln in alive.stdout.splitlines()
                     if '","' in ln}
        for ln in q.stdout.splitlines():
            parts = ln.split()
            if len(parts) == 2 and all(x.isdigit() for x in parts):
                ppid, pid = parts
                if ppid not in live_pids:
                    subprocess.run(["taskkill", "/T", "/F", "/PID", pid],
                                   capture_output=True, text=True)
    except (OSError, subprocess.SubprocessError):
        pass


def check_r1():
    rc, out = run([sys.executable, "scripts/dev/wren_deletion_audit.py", "--fail-on-blocking"],
                  timeout=900, guard=False)
    line = next((l for l in out.splitlines() if "VERDICT" in l), "(no verdict line)")
    note = ("audit scans GIT-TRACKED files only, and its dead-include detector cannot see Wr* "
            "STRUCT usage -- a green R1 still needs R2")
    return (GREEN if rc == 0 else RED), line.strip(), note


def _find_make():
    """`make` lives in msys, which is not on the default PATH of this tool's subprocesses.
    Resolving it here instead of hoping is what keeps R3 an instrument, not a coin flip."""
    import shutil as _sh
    m = _sh.which("make")
    if m:
        return m
    for cand in (r"C:\msys64\usr\bin\make.exe", r"C:\msys64\mingw64\bin\make.exe"):
        if os.path.exists(cand):
            return cand
    return None


def check_r3():
    make = _find_make()
    if not make:
        return MANUAL, "make not found on PATH or in C:/msys64 -- run `make tests-smoke` by hand", \
               "an instrument that cannot start is UNMEASURED, not red"
    env = os.environ.copy()
    env["PATH"] = r"C:\msys64\usr\bin;C:\msys64\mingw64\bin;" + env.get("PATH", "")
    try:
        p = subprocess.run([make, "tests-smoke"], cwd=str(REPO), capture_output=True, text=True,
                           timeout=2400, errors="replace", env=env)
        rc, out = p.returncode, (p.stdout or "") + (p.stderr or "")
    except subprocess.TimeoutExpired:
        return MANUAL, "smoke suite TIMED OUT at 2400s -- unmeasured, not red", ""
    ok = "Smoke run passed" in out or "smoke world(s) passed" in out
    fails = [l.strip() for l in out.splitlines() if "[smoke] FAILED" in l]
    return (GREEN if ok and rc == 0 else RED), (fails[0] if fails else "smoke suite"), ""


def _test_group(group):
    """Per-world verdicts for one suite group: {"red": [...], "no_verdict": [...]}.

    WHY PER-WORLD AND NOT `test-group`: the group runner is killed after ~5 worlds by a
    PRE-EXISTING engine abort (wgpu-native panic in wgpuDeviceCreateBindGroup, exit
    0xC0000409, triggered by the suite's back-to-back engine spawning; A/B-attributed
    2026-08-23 -- it reproduces identically with every campaign hatch disabled) -- and
    test_suite.py prints "Test suite complete" even after the abort, so a group run can
    look whole while being a fiction. The first baseline recorded exactly that fiction:
    an api red-list ending alphabetically one world before the known-red camera_* set.
    Serialised per-world runs dodge the spawn race entirely, and a world that yields
    neither an OK nor a FAILURE line is recorded as `no_verdict` -- visible, never
    silently counted green.
    """
    wdir = REPO / "tests" / group / "worlds"
    worlds = sorted([p for p in wdir.iterdir()
                     if p.suffix in (".omniworld", ".wbt") and not p.name.startswith(".")])
    red, no_verdict = [], []
    for w in worlds:
        stem = w.stem
        rc, out = run([sys.executable, "scripts/dev/omnisim_dev.py", "test-world",
                       str(w.relative_to(REPO)), "--nomake"], timeout=420)
        if f"FAILURE with {stem}" in out:
            red.append(stem)
        elif f"OK: {stem}" in out:
            pass
        else:
            no_verdict.append(stem)
        print(f"  [r5] {stem}: "
              + ("RED" if stem in red else ("ok" if stem not in no_verdict else "NO-VERDICT")),
              flush=True)
    return {"red": sorted(red), "no_verdict": sorted(no_verdict)}


def check_r5(baseline, record):
    """no NEW red vs the recorded pre-deletion baseline."""
    groups = {}
    for g in ("api", "rendering"):
        groups[g] = _test_group(g)
    if record:
        BASELINE.write_text(json.dumps({"recorded_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                                                     time.gmtime()),
                                        "groups": groups}, indent=1), encoding="utf-8")
        return MANUAL, f"baseline RECORDED to {BASELINE.name}: " + json.dumps(groups), \
               "re-run without --record-baseline to compare"
    if not baseline:
        return MANUAL, "no baseline recorded -- run --record-baseline while WREN still works", \
               "R5 cannot be evaluated without it, and it cannot be recorded after the fact"
    def _reds(entry):
        # schema v2 is {"red": [...], "no_verdict": [...]}; a no-verdict world counts as
        # red-equivalent for comparison -- "stopped producing a verdict" is a regression too.
        if isinstance(entry, dict):
            return set(entry.get("red", [])) | set(entry.get("no_verdict", []))
        return set(entry or [])
    new = {g: sorted(_reds(groups.get(g)) - _reds(baseline["groups"].get(g))) for g in groups}
    total = sum(len(v) for v in new.values())
    detail = json.dumps({g: v for g, v in new.items() if v}) if total else "no new failures"
    return (GREEN if total == 0 else RED), detail, f"baseline {baseline.get('recorded_utc','?')}"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--record-baseline", action="store_true",
                    help="record today's red tests as the R5 baseline (do this while WREN works)")
    ap.add_argument("--quick", action="store_true", help="R1 only -- the cheap static gate")
    ap.add_argument("--no-guard", action="store_true", help="do not wrap engine runs in thermal_guard")
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    baseline = None
    if BASELINE.exists():
        try:
            baseline = json.loads(BASELINE.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            baseline = None

    rows = []
    st, detail, note = check_r1()
    rows.append(("R1", "no WREN dependency outside the removal set", st, detail, note))

    if args.quick:
        rows.append(("R2-R8", "not run (--quick)", SKIP, "", ""))
    else:
        rows.append(("R2", "engine builds with WREN compiled out", MANUAL,
                     "run `make release` after D1.4", "the only real proof of R1"))
        st, detail, note = check_r3()
        rows.append(("R3", "smoke suite green", st, detail, note))
        rows.append(("R4", "world corpus still loads", MANUAL,
                     "python scripts/dev/batch_validate.py --glob 'projects/**/*.omniworld' -j4",
                     "delete leftover .harness_* siblings first or the count inflates"))
        st, detail, note = check_r5(baseline, args.record_baseline)
        rows.append(("R5", "no NEW red vs the pre-deletion baseline", st, detail, note))
        rows.append(("R6", "every parity item ported or deliberately retired", MANUAL,
                     "see Phase P gates in the runbook",
                     "a retirement is green ONLY if the field still parses and warns by name"))
        rows.append(("R7", "silent-failure list individually disproven", MANUAL,
                     "; ".join(R7_ITEMS),
                     "each of these COMPILES CLEAN when broken -- R2/R3 cannot see them"))
        rows.append(("R8", "wgpu-native builds+initialises on every platform", ASSUMED,
                     "owner-assumed satisfied 2026-08-22; needs a Linux run",
                     "with WREN gone there is no third backend"))

    w = max(len(r[1]) for r in rows)
    print("\n" + "=" * (w + 34))
    print("WREN DELETION READINESS")
    print("=" * (w + 34))
    for rid, name, st, detail, note in rows:
        print(f"  [{st:^7}] {rid:5} {name.ljust(w)}")
        if detail:
            print(f"            {detail[:160]}")
        if note:
            print(f"            note: {note[:150]}")
    greens = [r for r in rows if r[2] == GREEN]
    reds = [r for r in rows if r[2] == RED]
    unproven = [r for r in rows if r[2] in (MANUAL, ASSUMED, SKIP)]
    print("-" * (w + 34))
    if reds:
        verdict = "NOT READY -- " + ", ".join(r[0] for r in reds) + " red"
    elif unproven:
        verdict = ("NOT PROVEN -- " + ", ".join(r[0] for r in unproven)
                   + " not measured by this tool. Not the same as ready.")
    else:
        verdict = "READY (all eight green)"
    print(f"VERDICT: {verdict}")
    print(f"         {len(greens)} green, {len(reds)} red, {len(unproven)} unproven")
    print("=" * (w + 34) + "\n")

    if args.json:
        Path(args.json).write_text(json.dumps(
            [{"id": r[0], "check": r[1], "status": r[2], "detail": r[3], "note": r[4]}
             for r in rows], indent=1), encoding="utf-8")
    return 1 if reds else 0


if __name__ == "__main__":
    sys.exit(main())
