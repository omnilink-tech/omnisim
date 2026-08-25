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

"""drive_test -- run the 2-rover straight-line wheel-drive test and report rolling vs flapping.

Runs projects/robot_combat/worlds/tests/drive_test.omniworld headless (the drive_demo controllers each
write _scratch/drive_<name>.csv), then for each rover reports net +X travel, lateral drift, chassis
tilt, and -- the key signal -- per-wheel angle winding: a ROLLING wheel winds monotonically
(net_angle ~ omega*t, little backtrack); a FLAPPING wheel oscillates (small net, large back-and-forth).

Usage: python scripts/dev/drive_test.py [newton] [sim_ms]
(the second backend arm was `ode`; src/ode is DELETED (bdc02139) so `newton` is the only value)
Run from PowerShell (Newton needs warp). Retries the cold-load crash.
"""
import os
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
WORLD = REPO / "projects" / "robot_combat" / "worlds" / "tests" / "drive_test.omniworld"
OMEGA = 12.0
ROVERS = ["flapper", "roller"]


def run(backend, sim_ms):
    csvs = {r: REPO / "_scratch" / ("drive_%s.csv" % r) for r in ROVERS}
    env = dict(os.environ)
    env["OMNISIM_HOME"] = str(REPO)
    env["WEBOTS_HOME"] = str(REPO)
    env["OMNISIM_PROBE_TRAJ"] = str(REPO / "_scratch" / "drive_probe.txt")
    env["OMNISIM_PROBE_TRAJ_MS"] = str(sim_ms)
    # Per-launch log path (AGENTS.md §3e): keep back-to-back launches off the shared install-root
    # omnisim_log.txt so a lingering handle from the prior run can't fail the next launch's log open.
    env["OMNISIM_LOG_PATH"] = str(REPO / "_scratch" / "drive_test.omnisim_log.txt")
    # RETIRED KNOBS, SCRUBBED ON PURPOSE: OMNISIM_FORCE_ODE / OMNISIM_LEGACY name
    # a backend that no longer exists (src/ode deleted, bdc02139). A stale export
    # in the operator's shell must not steer this measurement.
    env.pop("OMNISIM_FORCE_ODE", None)
    env.pop("OMNISIM_LEGACY", None)
    for kv in sys.argv[3:]:        # extra KEY=VAL env knobs (e.g. OMNISIM_NEWTON_CYLINDER_AS_SPHERE=1)
        if "=" in kv:
            k, v = kv.split("=", 1)
            env[k] = v
            print("  [env] %s=%s" % (k, v))
    # Redirect child output to a real .log (NOT DEVNULL): on Windows, DEVNULL can hand the
    # spawned drive_demo controllers invalid inheritable std handles, the SYSTEMATIC cause of the
    # flaky "no result" first launch (default-flip-plan.md §3.5). A real file gives valid
    # inheritable handles (like a Start-Process launch, which never flaked).
    log_path = REPO / "_scratch" / "drive_test.launch.log"
    for attempt in range(1, 6):
        for c in csvs.values():
            try:
                c.unlink()
            except OSError:
                pass
        with open(log_path, "w", encoding="utf-8", errors="replace") as logf:
            proc = subprocess.Popen(
                [str(BIN), str(WORLD), "--mode=fast", "--no-rendering", "--batch"],
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
        if all(c.exists() and c.stat().st_size > 80 for c in csvs.values()):
            return csvs
        print("  [retry %d/5] missing CSV data (cold-load crash?)" % attempt)
        time.sleep(2.0)
    return None


def analyze(csv):
    rows = []
    for line in csv.read_text(errors="replace").splitlines()[1:]:
        p = line.split(",")
        if len(p) >= 9:
            try:
                rows.append([float(x) for x in p[:9]])
            except ValueError:
                pass
    if len(rows) < 3:
        return None
    t0, t1 = rows[0], rows[-1]
    dur = t1[0] - t0[0]
    net_x = t1[1] - t0[1]
    lat = max(abs(r[2] - t0[2]) for r in rows)
    tilt = max(r[4] for r in rows)
    # per-wheel: net angle + total variation (winding vs oscillation)
    wheel_net, wheel_tv = [], []
    for w in range(5, 9):
        seq = [r[w] for r in rows]
        wheel_net.append(seq[-1] - seq[0])
        wheel_tv.append(sum(abs(seq[i + 1] - seq[i]) for i in range(len(seq) - 1)))
    mean_net = sum(wheel_net) / 4.0
    mean_tv = sum(wheel_tv) / 4.0
    expect = OMEGA * dur
    # monotonic if the signed winding ~ the total path (little backtracking)
    monotonic = mean_tv <= abs(mean_net) * 1.25 + 1.0
    return dict(dur=dur, net_x=net_x, lat=lat, tilt=tilt, mean_net=mean_net,
               mean_tv=mean_tv, expect=expect, monotonic=monotonic)


def main():
    backend = sys.argv[1] if len(sys.argv) > 1 else "newton"
    if backend != "newton":
        print("[drive_test] backend %r is not available: ODE was deleted "
              "(src/ode, commit bdc02139) and Newton is the only physics "
              "backend. Drop the argument." % backend)
        return 2
    sim_ms = int(sys.argv[2]) if len(sys.argv) > 2 else 10000
    if not BIN.exists():
        print("[drive_test] omnisim-bin not found -- build it first.")
        return 0
    print("[drive_test] backend=%s sim=%dms omega=%.0f (expect wheel angle ~%.0f rad if rolling)"
          % (backend, sim_ms, OMEGA, OMEGA * sim_ms / 1000.0))
    csvs = run(backend, sim_ms)
    if not csvs:
        print("RESULT: FAIL (no CSV data after retries)")
        return 1
    ok_all = True
    for r in ROVERS:
        a = analyze(csvs[r])
        if a is None:
            print("  %-8s NO DATA" % r)
            ok_all = False
            continue
        rolling = (a["net_x"] > 1.5 and a["mean_net"] > 0.4 * a["expect"]
                   and a["monotonic"] and a["lat"] < 0.6)
        verdict = "ROLLING" if rolling else "FLAPPING/STUCK"
        print("  %-8s net_x=%6.2fm  lat=%4.2fm  tilt_max=%5.1fdeg  wheel_net=%6.1f rad "
              "(expect ~%.0f)  wheel_path=%6.1f  monotonic=%s  -> %s"
              % (r, a["net_x"], a["lat"], a["tilt"], a["mean_net"], a["expect"],
                 a["mean_tv"], a["monotonic"], verdict))
        if r == "roller" and not rolling:
            ok_all = False
    print("RESULT:", "PASS (roller rolls)" if ok_all else "SEE ABOVE")
    return 0 if ok_all else 2


if __name__ == "__main__":
    sys.exit(main())
