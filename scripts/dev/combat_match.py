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

"""combat_match -- run a robot_combat world to a bounded match end under Newton and report the
scorecard (completion plan P4).

Sets the Newton damage-parity env (OMNISIM_DAMAGE_VEL_SMOOTH, to de-jitter the mass*|dv| proxy)
and a bounded OMNISIM_DAMAGE_TIMER_S so a match finishes quickly, then reads the director's
scorecard JSON (now resolved under <repo>/_scratch). PASS = a scorecard was written AND some
damage occurred (a part broke or a bot was immobilized), i.e. ramming now deals damage under
Newton. Run from PowerShell (Newton needs warp); retries the cold-load crash.

Usage: python scripts/dev/combat_match.py <world.omniworld> [timer_s] [vel_smooth]
"""
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))
from omnisim.paths import resolve_omnisim_binary  # noqa: E402

# Cross-platform binary resolution (Windows msys64, Linux bin/, macOS bundle).
# Fall back to the legacy Windows path so BIN.exists() checks still print a
# helpful location on an unbuilt checkout.
BIN = Path(resolve_omnisim_binary() or REPO / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe")


def scorecard_name(world_abs):
    txt = Path(world_abs).read_text(encoding="utf-8", errors="replace")
    for m in re.finditer(r'customData\s+"((?:[^"\\]|\\.)*)"', txt):
        js = m.group(1).replace('\\"', '"').replace('\\\\', '\\')
        try:
            d = json.loads(js)
        except Exception:
            continue
        if isinstance(d, dict) and d.get("output"):
            return Path(d["output"]).name
    return "battlebox_match.json"


def main():
    if len(sys.argv) < 2:
        print("usage: combat_match.py <world.omniworld> [timer_s] [vel_smooth]")
        return 2
    world = sys.argv[1]
    timer_s = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
    vel_smooth = sys.argv[3] if len(sys.argv) > 3 else "5"
    # argv[4] used to select an "ode" A/B baseline arm. src/ode is DELETED
    # (bdc02139) -- Newton is the only backend, so there is nothing to A/B
    # against. Refuse the word rather than accept it and run Newton anyway.
    backend = sys.argv[4] if len(sys.argv) > 4 else "newton"
    if backend != "newton":
        print("[combat_match] backend %r is not available: ODE was deleted "
              "(src/ode, commit bdc02139) and Newton is the only physics "
              "backend. Drop the argument." % backend)
        return 2
    world_abs = world if os.path.isabs(world) else str(REPO / world)
    if not BIN.exists():
        print("[combat_match] omnisim-bin not found -- build it first.")
        return 0

    sc = REPO / "_scratch" / scorecard_name(world_abs)
    sc.parent.mkdir(parents=True, exist_ok=True)
    sim_ms = int((timer_s + 8.0) * 1000)  # exit a bit after the match clock so the scorecard lands first

    env = dict(os.environ)
    env["OMNISIM_HOME"] = str(REPO)
    env["WEBOTS_HOME"] = str(REPO)
    env["OMNISIM_DAMAGE_TIMER_S"] = str(timer_s)
    env["OMNISIM_DAMAGE_VEL_SMOOTH"] = vel_smooth
    env["OMNISIM_PROBE_TRAJ"] = str(REPO / "_scratch" / "combat_match_traj.txt")
    env["OMNISIM_PROBE_TRAJ_MS"] = str(sim_ms)
    # RETIRED KNOBS, SCRUBBED ON PURPOSE: OMNISIM_FORCE_ODE / OMNISIM_LEGACY
    # select a backend that no longer exists. A stale export in the operator's
    # shell must not silently steer this measurement.
    env.pop("OMNISIM_FORCE_ODE", None)
    env.pop("OMNISIM_LEGACY", None)

    print("[combat_match] %s backend=%s timer=%.0fs vel_smooth=%s -> %s"
          % (world, backend, timer_s, vel_smooth, sc.name))
    for attempt in range(1, 5):
        try:
            sc.unlink()
        except OSError:
            pass
        proc = subprocess.Popen(
            [str(BIN), world_abs, "--mode=fast", "--no-rendering", "--batch"],
            cwd=str(REPO), env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        deadline = time.time() + sim_ms / 1000.0 * 3 + 120
        while time.time() < deadline and proc.poll() is None:
            time.sleep(1.0)
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                proc.kill()
        if sc.exists():
            break
        print("  [retry %d/4] no scorecard yet (cold-load crash?)" % attempt)
        time.sleep(2.0)

    if not sc.exists():
        print("RESULT: FAIL (no scorecard written)")
        return 1

    d = json.loads(sc.read_text())
    print("  scorecard: %s" % sc)
    print("  winner=%s reason=%s t=%ss" % (d.get("winner"), d.get("win_reason"), d.get("match_t_s")))
    for f in d.get("fighters", []):
        print("    %-8s immobile=%s reason=%s wheels=%s weapon_intact=%s broken=%s"
              % (f.get("name"), f.get("immobile"), f.get("disqualify_reason"),
                 f.get("wheels_remaining"), f.get("weapon_intact"), f.get("broken_parts")))
    any_dmg = any(f.get("broken_parts") or f.get("immobile") for f in d.get("fighters", []))
    print("RESULT:", "PASS (ramming dealt damage)" if any_dmg
          else "INCONCLUSIVE (no parts broken/immobilized -- bots may not have engaged in the window)")
    return 0 if any_dmg else 2


if __name__ == "__main__":
    sys.exit(main())
