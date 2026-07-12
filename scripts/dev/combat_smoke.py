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

"""combat_smoke -- headless stability + containment check for a robot_combat world under Newton.

Runs a combat world for a bounded sim-time with the in-binary OMNISIM_PROBE_TRAJ probe, relying
on the world's OWN WorldInfo Newton flags (newtonStatics / newtonRobotColliders / newtonSubsteps)
-- it does NOT set any solver env override, so it verifies the baked-in config actually works.
Checks (completion plan P3):

  * every articulation root stays FINITE (no XPBD NaN from high closing-speed ram impacts),
  * the fighter bots MOVE (wheels carry load -- a chassis collider that pinned the body would
    leave them stuck),
  * the bots stay within the arena footprint (newtonStatics walls contain them; without it they
    drive off the implicit ground plane).

Reads arena_size + fighter names from the director's customData in the .wbt. Run from PowerShell
(Newton needs warp); retries the known intermittent cold-load crash.

Usage:
    python scripts/dev/combat_smoke.py <world.wbt> [sim_ms]
"""

import json
import math
import os
import re
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
BIN = REPO / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe"
INFRA = {"battlebox", "damage_director", "duel_tracker", "sun_marker_driver",
         "match_director", "broadcast_director", "harness_supervisor"}


def read_world_cfg(world_abs):
    """Pull arena_size + fighters + output from the director customData in the .wbt.

    customData holds JSON whose quotes are backslash-escaped for the .wbt string, so un-escape
    the .wbt string-escaping and json.loads the first block that looks like a director config.
    """
    txt = Path(world_abs).read_text(encoding="utf-8", errors="replace")
    arena, fighters, out_name = None, [], None
    # Worlds carry several customData blocks (one per bot + the director). Capture arena_size
    # from whichever block has it, but only finalize the fighter list from the DIRECTOR block
    # (the one with "fighters" or, for ORC queens mode, "teams") so static scenery isn't
    # mistaken for a bot in the fallback path.
    for m in re.finditer(r'customData\s+"((?:[^"\\]|\\.)*)"', txt):
        js = m.group(1).replace('\\"', '"').replace('\\\\', '\\')
        try:
            d = json.loads(js)
        except Exception:
            continue
        if not isinstance(d, dict):
            continue
        if arena is None and "arena_size" in d:
            arena = d["arena_size"]
        flist = d.get("fighters")
        if not flist and isinstance(d.get("teams"), dict):
            flist = [b for team in d["teams"].values() for b in (team or [])]
        if flist:
            fighters = flist
            arena = d.get("arena_size", arena)
            if d.get("output"):
                out_name = Path(d["output"]).name
            break
    return arena, fighters, out_name


def run(world_abs, traj, sim_ms):
    env = dict(os.environ)
    env["OMNISIM_HOME"] = str(REPO)
    env["WEBOTS_HOME"] = str(REPO)
    env["OMNISIM_PROBE_TRAJ"] = str(traj)
    env["OMNISIM_PROBE_TRAJ_MS"] = str(sim_ms)
    env.pop("OMNISIM_FORCE_ODE", None)
    env.pop("OMNISIM_LEGACY", None)
    # Per-launch log path: back-to-back launches all writing the shared install-root
    # omnisim_log.txt contend on it (a fresh launch can't open the file the prior one is still
    # releasing -> early exit -> empty trajectory). Isolate it per tool (AGENTS.md §3e) so the
    # shared-log contention can't manufacture a flake on top of the real cold-load crash.
    env["OMNISIM_LOG_PATH"] = str(traj.with_suffix(".omnisim_log.txt"))
    # NB: deliberately NO OMNISIM_NEWTON_* overrides -- the world's WorldInfo flags must stand on
    # their own. (OMNISIM_PROBE_TRAJ does not control the solver.)
    # Redirect child output to a real .log (NOT DEVNULL): on Windows, DEVNULL can hand the
    # spawned local controllers invalid inheritable std handles, which is the SYSTEMATIC cause of
    # the flaky "no result" first launch (default-flip-plan.md §3.5). A real file gives valid
    # inheritable handles (like a Start-Process launch, which never flaked).
    log_path = traj.with_suffix(".launch.log")
    for attempt in range(1, 6):
        try:
            traj.unlink()
        except OSError:
            pass
        with open(log_path, "w", encoding="utf-8", errors="replace") as logf:
            proc = subprocess.Popen(
                [str(BIN), world_abs, "--mode=fast", "--no-rendering", "--batch"],
                cwd=str(REPO), env=env, stdout=logf, stderr=subprocess.STDOUT)
            deadline = time.time() + 150
            while time.time() < deadline and proc.poll() is None:
                time.sleep(0.5)
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    proc.kill()
        if traj.exists() and traj.stat().st_size > 0:
            return True
        print("  [retry %d/5] empty trajectory (cold-load crash) -- retrying" % attempt)
        time.sleep(2.0)
    return False


def main():
    if len(sys.argv) < 2:
        print("usage: combat_smoke.py <world.wbt> [sim_ms]")
        return 2
    world = sys.argv[1]
    sim_ms = int(sys.argv[2]) if len(sys.argv) > 2 else 8000
    world_abs = world if os.path.isabs(world) else str(REPO / world)
    if not BIN.exists():
        print("[combat_smoke] omnisim-bin not found -- build it first.")
        return 0

    arena, fighters, out_name = read_world_cfg(world_abs)
    traj = REPO / "_scratch" / "combat_smoke_traj.txt"
    traj.parent.mkdir(parents=True, exist_ok=True)
    print("[combat_smoke] %s sim=%dms arena=%s fighters=%s (Newton, WorldInfo flags only)"
          % (world, sim_ms, arena, fighters))

    if not run(world_abs, traj, sim_ms):
        print("RESULT: FAIL (no trajectory after retries)")
        return 1

    roots = {}
    tags = {}
    rx = re.compile(r"^(-?\d+(?:\.\d+)?)\t([^\t]*)\t"
                    r"(-?\d+(?:\.\d+)?)\t(-?\d+(?:\.\d+)?)\t(-?\d+(?:\.\d+)?)(?:\t([AR]))?")
    for line in traj.read_text(errors="replace").splitlines():
        m = rx.match(line)
        if m:
            roots.setdefault(m.group(2), []).append(
                (float(m.group(1)), float(m.group(3)),
                 float(m.group(4)), float(m.group(5))))
            if m.group(6):
                tags[m.group(2)] = m.group(6)

    # Prefer the director-declared fighter list; otherwise fall back to the articulated
    # ('A'-tagged) roots, excluding infra supervisors and detached debris -- so static
    # scenery / supervisors are never mistaken for a bot.
    if fighters:
        bots = [n for n in roots if n in fighters]
    else:
        bots = [n for n in roots if tags.get(n) == "A"
                and n not in INFRA and not n.lower().startswith("detached")]

    half = (arena * 0.5 + 0.5) if arena else None   # +0.5 m wall-thickness slack
    all_finite = all(v == v and abs(v) < 1e6 for pts in roots.values() for p in pts for v in p[1:])

    print("  roots seen: %s" % sorted(roots))
    ok = all_finite
    for b in sorted(bots):
        pts = sorted(roots.get(b, []))
        if len(pts) < 2:
            print("  %-8s <2 samples" % b)
            ok = False
            continue
        disp = math.dist(pts[0][1:], pts[-1][1:])
        max_xy = max(math.hypot(p[1], p[2]) for p in pts)
        finite = all(v == v and abs(v) < 1e6 for p in pts for v in p[1:])
        contained = (half is None) or (max_xy <= half)
        moved = disp > 0.02
        print("  %-8s moved=%.3fm  max_xy=%.2fm  contained=%s  moved_ok=%s  finite=%s"
              % (b, disp, max_xy, contained, moved, finite))
        ok = ok and finite and moved and contained

    # Bonus: did a match complete + write a scorecard?
    if out_name:
        sc = REPO / "_scratch" / out_name
        if sc.exists():
            try:
                d = json.loads(sc.read_text())
                print("  scorecard: winner=%s reason=%s t=%ss"
                      % (d.get("winner"), d.get("win_reason"), d.get("match_t_s")))
            except Exception:
                print("  scorecard present but unparseable: %s" % sc)
        else:
            print("  (no scorecard -- match did not end within %dms)" % sim_ms)

    print("  all positions finite: %s" % all_finite)
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
