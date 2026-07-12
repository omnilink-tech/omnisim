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

"""detach_smoke -- exercise battlebot part-detachment under Newton (completion plan P2).

The battlebot damage director's "win" mechanic detaches a broken wheel/weapon by exporting the
Solid's VRML, re-importing it as a free root-child Solid, transferring velocity, and removing the
owning HingeJoint (exportString -> importMFNodeFromString -> setVelocity -> parent.remove). That
is the most Newton-fragile dynamic-graph mutation in robot_combat: it must (a) re-register the
freed Solid as a Newton dynamic body and (b) rebuild the parent articulation with one fewer joint
WITHOUT freezing the bot or NaN-ing the shared articulation (the failure mode behind the original
chassis-freeze bug).

This runs a robot_combat world headless under Newton with the director's default-off
OMNISIM_DAMAGE_FORCE_DETACH hook, then checks the OMNISIM_PROBE_TRAJ output + the director's
stderr for the post-conditions. Run from PowerShell (Newton needs warp). Build omnisim-bin first.

Usage:
    python scripts/dev/detach_smoke.py [world] [parts] [sim_ms]
      world  default projects/robot_combat/battlebots/worlds/battlebox_duel.wbt
      parts  default "fl_wheel,weapon" (detached from fighters[0] at t=2.0s)
      sim_ms default 5000
"""

import math
import os
import re
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
BIN = REPO / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe"
DETACH_T = 2.0


def main():
    world = sys.argv[1] if len(sys.argv) > 1 else \
        "projects/robot_combat/battlebots/worlds/battlebox_duel.wbt"
    parts = sys.argv[2] if len(sys.argv) > 2 else "fl_wheel,weapon"
    sim_ms = int(sys.argv[3]) if len(sys.argv) > 3 else 5000
    world_abs = world if os.path.isabs(world) else str(REPO / world)

    if not BIN.exists():
        print("[detach_smoke] omnisim-bin not found -- build it first.")
        return 0

    traj = REPO / "_scratch" / "detach_smoke_traj.txt"
    errlog = REPO / "_scratch" / "detach_smoke_stderr.txt"
    traj.parent.mkdir(parents=True, exist_ok=True)
    for f in (traj, errlog):
        try:
            f.unlink()
        except OSError:
            pass

    env = dict(os.environ)
    env["OMNISIM_HOME"] = str(REPO)
    env["WEBOTS_HOME"] = str(REPO)          # override any system Webots install
    env["OMNISIM_PROBE_TRAJ"] = str(traj)
    env["OMNISIM_PROBE_TRAJ_MS"] = str(sim_ms)
    env["OMNISIM_NEWTON_STATICS"] = "1"     # Newton-at-its-best contact config
    env["OMNISIM_NEWTON_SUBSTEPS"] = "4"
    # Per-launch log path (AGENTS.md §3e): isolate from the shared install-root omnisim_log.txt so
    # rapid relaunch can't add a log-open contention flake on top of the real cold-load crash below.
    env["OMNISIM_LOG_PATH"] = str(REPO / "_scratch" / "detach_smoke.omnisim_log.txt")
    env["OMNISIM_DAMAGE_FORCE_DETACH"] = parts
    env["OMNISIM_DAMAGE_FORCE_DETACH_T"] = str(DETACH_T)
    env.pop("OMNISIM_FORCE_ODE", None)
    env.pop("OMNISIM_LEGACY", None)

    print("[detach_smoke] %s parts=%s sim=%dms detach@%.1fs (Newton)"
          % (world, parts, sim_ms, DETACH_T))
    # Multi-robot Newton worlds crash intermittently on cold load (Qt teardown race, exit 1),
    # producing an empty trajectory -- retry until we get data (same lever as faithful_check).
    exited_clean, rc = False, None
    for attempt in range(1, 6):
        for f in (traj, errlog):
            try:
                f.unlink()
            except OSError:
                pass
        with open(errlog, "w", encoding="utf-8", errors="replace") as ef:
            proc = subprocess.Popen(
                [str(BIN), world_abs, "--mode=fast", "--no-rendering",
                 "--batch", "--stdout", "--stderr"],
                cwd=str(REPO), env=env, stdout=ef, stderr=ef)
            deadline = time.time() + 150
            while time.time() < deadline and proc.poll() is None:
                time.sleep(0.5)
            exited_clean = proc.poll() is not None
            rc = proc.poll()
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    proc.kill()
        if traj.exists() and traj.stat().st_size > 0:
            break
        print("  [retry %d/5] empty trajectory (cold-load crash rc=%s) -- retrying" % (attempt, rc))
        time.sleep(2.0)

    err = errlog.read_text(errors="replace") if errlog.exists() else ""

    def has(pat):
        return bool(re.search(pat, err, re.I))

    # omnisim-bin is windows-subsystem: controller stderr is usually NOT capturable to a file,
    # so the trajectory probe (a real file oracle), not [damage_director] log lines, is the
    # source of truth. Log markers below are reported only as a bonus when present.
    fired_log = "FORCE-DETACH" in err
    gone_log = ("GONE" in err) and ("STILL_PRESENT" not in err)

    roots = {}
    rx = re.compile(r"^(-?\d+(?:\.\d+)?)\t([^\t]*)\t"
                    r"(-?\d+(?:\.\d+)?)\t(-?\d+(?:\.\d+)?)\t(-?\d+(?:\.\d+)?)")
    if traj.exists():
        for line in traj.read_text(errors="replace").splitlines():
            m = rx.match(line)
            if m:
                roots.setdefault(m.group(2), []).append(
                    (float(m.group(1)), float(m.group(3)),
                     float(m.group(4)), float(m.group(5))))

    def moved_after(name, t0):
        pts = [p for p in roots.get(name, []) if p[0] >= t0 * 1000.0]
        if len(pts) < 2:
            return None
        a, b = pts[0], pts[-1]
        return math.dist(a[1:], b[1:])

    def finite(name):
        return all(v == v and abs(v) < 1e6 for p in roots.get(name, []) for v in p[1:])

    # The freed Solid must (a) appear (re-registered as a Newton body), (b) at ~the detach time
    # -- proving the hook fired, not incidental damage -- and (c) MOVE (a frozen/unregistered
    # body would hold its detach pose). Together with the parent bot continuing to move and every
    # root staying finite, that is the full "no freeze / no NaN" post-condition.
    detached_roots = [n for n in roots if n.lower().startswith("detached")]
    det_first_ms = det_disp = None
    if detached_roots:
        pts = sorted(roots[detached_roots[0]])
        det_first_ms = pts[0][0]
        det_disp = max(math.dist(pts[0][1:], p[1:]) for p in pts)
    red_move = moved_after("red", DETACH_T)
    blue_move = moved_after("blue", DETACH_T)
    all_finite = all(finite(n) for n in roots)
    on_time = det_first_ms is not None and abs(det_first_ms - DETACH_T * 1000.0) <= 64.0
    dynamic = (det_disp or 0.0) > 0.02

    print("  process exited cleanly: %s (rc=%s)" % (exited_clean, rc))
    print("  roots seen:             %s" % sorted(roots))
    print("  freed Solid:            %s  first@%sms (detach@%.0fms, on-time=%s)  max-disp=%s m"
          % (detached_roots or "NONE", det_first_ms, DETACH_T * 1000.0, on_time,
             None if det_disp is None else round(det_disp, 4)))
    print("  freed Solid is dynamic: %s (moved under gravity, not frozen)" % dynamic)
    print("  bot moved post-detach:  red=%s m  blue=%s m"
          % (None if red_move is None else round(red_move, 4),
             None if blue_move is None else round(blue_move, 4)))
    print("  all positions finite:   %s" % all_finite)
    print("  (log bonus) FORCE-DETACH=%s GONE=%s  [controller stderr often empty]"
          % (fired_log, gone_log))

    ok = (exited_clean and rc == 0 and all_finite
          and bool(detached_roots) and on_time and dynamic
          and (red_move or 0.0) > 0.01)
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
