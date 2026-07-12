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

"""physics_oracle — dual-backend physics verifier (default-flip-plan.md §3.3, physics half).

Runs the SAME world twice through omnisim-bin, headless:
  (A) reference: OMNISIM_LEGACY=1  -> every Solid forced to ODE (the legacy oracle)
  (B) candidate: migration default -> Newton when available, else ODE

Both runs use the oracle_dumper supervisor, which writes each DEF'd Solid's absolute world position
per step to a CSV. The runner aligns the two CSVs column-for-column and reports, per body, the maximum
position divergence and the first step at which they part by more than a threshold. This is how the
retained legacy stack VERIFIES the new one — the explicit reason the plan keeps ODE/WREN as a fallback.

Run from PowerShell (NOT MSYS bash), so the embedded interpreter finds Warp and run (B) actually
exercises Newton instead of silently falling back to ODE:

    python scripts/dev/physics_oracle.py [--world tests/physics/worlds/oracle_drop.wbt] [--steps 200]

Exit code 0 always (it's a measurement tool, not a pass/fail gate — the gate that bakes a threshold in
is the Newton-golden smoke, §3.4). Prints a divergence table.
"""

import argparse
import csv
import math
import os
import subprocess
import sys
import tempfile
import time

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
BIN = os.path.join(REPO, "msys64", "mingw64", "bin", "omnisim-bin.exe")
DEFAULT_WORLD = os.path.join(REPO, "tests", "physics", "worlds", "oracle_drop.wbt")


def warn_if_already_running():
    """A concurrent omnisim-bin (e.g. one the user is watching) can hold the controller comms port, so
    a run yields no CSV (a flaky "no result"). We do NOT kill it — that would close the user's own
    instance — we just warn so the result is interpretable. The oracle's own two runs are sequential
    (subprocess.run waits), so they never collide with each other."""
    try:
        out = subprocess.run(["tasklist", "/FI", "IMAGENAME eq omnisim-bin.exe"],
                             timeout=15, capture_output=True, text=True)
        if "omnisim-bin.exe" in (out.stdout or ""):
            print("[oracle] WARNING: an omnisim-bin is already running; a 'no result' below may be a")
            print("         port collision, not a real divergence. Close other instances and re-run.")
    except Exception:
        pass


def run_once(world, out_csv, steps, force_legacy):
    env = dict(os.environ)
    env["OMNISIM_ORACLE_OUT"] = out_csv
    env["OMNISIM_ORACLE_STEPS"] = str(steps)
    env.pop("OMNISIM_LEGACY", None)
    env.pop("OMNISIM_FORCE_ODE", None)
    if force_legacy:
        env["OMNISIM_LEGACY"] = "1"
    # --batch: no dialogs; --mode=fast: run as quick as possible; --minimize: no visible window.
    # NOTE: we deliberately do NOT pass --port. It rebinds only the extern/streaming server; a
    # locally-spawned controller (oracle_dumper) does NOT inherit it and then can't connect at all
    # (verified: every run yields "no result" with --port set). The launch-race source fix lives in the
    # harness (local-controller IPC), not here; the capture() retry is the mitigation meanwhile.
    # The controller calls simulationQuit() after `steps`, so the process exits on its own.
    # --no-rendering: a physics oracle reads body positions, never pixels. Skipping the
    # WREN/GL init makes each launch faster AND removes a cold-start race (the renderer
    # spin-up on a minimized window) that intermittently yielded "no/short data" on rapid
    # sequential launches (§3.5). Matches the headless_runner launch flags, which never flaked.
    args = [BIN, world, "--batch", "--mode=fast", "--minimize", "--no-rendering"]
    # Redirect child output to a real file (NOT DEVNULL): on Windows, DEVNULL can hand the spawned
    # controller invalid std handles, which correlates with the flaky "no result" first launch. A real
    # file gives valid inheritable handles (matching a Start-Process launch, which never flaked).
    log_path = out_csv + ".log"
    try:
        with open(log_path, "w", encoding="utf-8", errors="replace") as logf:
            subprocess.run(args, env=env, timeout=180, stdout=logf, stderr=subprocess.STDOUT)
    except subprocess.TimeoutExpired:
        pass  # the CSV is flushed per step, so a hung quit still leaves usable data


def load(out_csv):
    if not os.path.exists(out_csv):
        return None, None
    with open(out_csv, newline="", encoding="utf-8") as f:
        rows = list(csv.reader(f))
    if len(rows) < 2:
        return None, None
    header = rows[0]
    data = {}
    for r in rows[1:]:
        if len(r) != len(header):
            continue
        try:
            data[int(r[0])] = [float(x) for x in r[1:]]
        except ValueError:
            continue
    return header, data


CAPTURE_ATTEMPTS = 8  # raised 3->5 (2026-06-07) ->8 (2026-06-10): a single launch is 100%-reliable, but a
# back-to-back launch can lose the local-controller startup race and yield NO csv. The margin at 5 was
# too thin — observed a clean standalone gate recover only on attempt 5/5 (~20s of cumulative settle),
# and a gate run issued right after the render gate's self-check (which crashes ~2-in-3 and briefly
# contends) exhaust all 5. 8 attempts with the growing backoff below budget ~56s of cumulative settle,
# giving the contended resource comfortably more time to release. A clean launch still returns on
# attempt 1 — the extra attempts only cost wall-clock on the flake path. The real root-cause fix is the
# harness-level local-controller IPC (tracked in default-flip-plan.md §3.5); this keeps the gate standing.


def capture(world, out_csv, steps, force_legacy, label):
    """Run once and load; retry on empty/short output. A single omnisim-bin launch reliably produces a
    CSV, but a launch issued right after a previous one can lose the local-controller startup race and
    yield no CSV (the same race that makes the wgpu sensor smoke flap; default-flip-plan.md §3.5). We
    retry with a GROWING settle (2,4,6,8,10,12,14s) rather than a flat 3s: the failure is the prior run's
    controller resources not yet released, so each retry gives them more time. This keeps the gate
    standing on the intermittent flake until the harness-level IPC fix lands."""
    for attempt in range(1, CAPTURE_ATTEMPTS + 1):
        if os.path.exists(out_csv):
            os.remove(out_csv)
        run_once(world, out_csv, steps, force_legacy)
        header, data = load(out_csv)
        if header is not None and len(data) >= max(2, int(steps * 0.5)):
            if attempt > 1:
                print("[oracle]   %s recovered on attempt %d." % (label, attempt))
            return header, data
        if attempt < CAPTURE_ATTEMPTS:
            print("[oracle]   %s attempt %d yielded no/short data; settling + retrying..."
                  % (label, attempt))
            time.sleep(2 * attempt)
    return load(out_csv)


def bodies_from_header(header):
    # header is step, <DEF>_x, <DEF>_y, <DEF>_z, ...  -> ordered unique DEF names
    names = []
    for col in header[1:]:
        base = col.rsplit("_", 1)[0]
        if base not in names:
            names.append(base)
    return names


def gate(world, steps, tmp, require_newton=False):
    """Pass/fail physics gate (default-flip-plan.md §3.4, Option C: PHYSICAL INVARIANTS).

    Asserting Newton == ODE is wrong (different solvers legitimately differ), and a Newton trajectory
    golden isn't portable (Warp/GPU output varies by hardware). So instead we assert invariants any sane
    rigid-body result must satisfy, which ARE portable and catch the real Newton failure modes:
      INV1  finite + bounded     — no NaN/inf, nothing flies off or sinks through the floor (explosion)
      INV2  settles by the end   — every body comes to rest (no perpetual drift / oscillation / launch)
      INV3  reproducible         — two runs agree to ~1 cm (catches gross non-determinism / chaos)
    Gates whichever backend resolved: on a Newton box this gates Newton; elsewhere ODE. Exit 0 = pass."""
    if require_newton:
        # Engine-level assert (defense in depth with the behavioral ODE-divergence check
        # below): make the default-backend run's binary WbLog::fatal (non-zero exit) if the
        # Newton runtime can't initialise, instead of silently degrading to ODE. The ODE-ref
        # run sets OMNISIM_LEGACY, which short-circuits resolve() before the Newton ctor, so
        # this never fires on it.
        os.environ["OMNISIM_REQUIRE_NEWTON"] = "1"
    warn_if_already_running()
    runs = []
    for i in range(2):
        h, d = capture(world, os.path.join(tmp, "gate%d.csv" % i), steps, False, "gate-run%d" % i)
        if h is None:
            print("[gate] FAIL: capture failed on run %d (harness/world wiring)." % i)
            return 1
        runs.append((h, d))

    h0, d0 = runs[0]
    bodies = bodies_from_header(h0)
    order = sorted(d0)
    print("\n[gate] %d bodies, %d steps\n" % (len(bodies), len(order)))
    ok = True

    # INV1: finite + bounded. Generous bounds — this catches NaN/inf and gross explosion, not fine detail.
    bound_xy, z_lo, z_hi = 50.0, -1.0, 50.0
    bad = None
    for s in order:
        row = d0[s]
        for bi in range(len(bodies)):
            x, y, z = row[bi * 3], row[bi * 3 + 1], row[bi * 3 + 2]
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                bad = (s, bodies[bi], "non-finite")
            elif abs(x) > bound_xy or abs(y) > bound_xy or z < z_lo or z > z_hi:
                bad = (s, bodies[bi], "out-of-bounds z=%.2f" % z)
            if bad:
                break
        if bad:
            break
    print("  INV1 finite+bounded (no NaN/explosion):   %s%s"
          % ("PASS" if bad is None else "FAIL", "" if bad is None else "  @step %d %s (%s)" % bad))
    ok = ok and bad is None

    # INV2: settled. Spread of each body's position over the final 10% of steps must be tiny.
    tail = order[int(len(order) * 0.9):] or order[-1:]
    settle_eps = 0.03
    worst, worst_body = 0.0, "-"
    for bi, name in enumerate(bodies):
        comp_spread = 0.0
        for c in range(3):
            vals = [d0[s][bi * 3 + c] for s in tail]
            comp_spread = max(comp_spread, max(vals) - min(vals))
        if comp_spread > worst:
            worst, worst_body = comp_spread, name
    print("  INV2 settled by end (tail spread<%.2fm):   %s  (worst %s=%.4fm)"
          % (settle_eps, "PASS" if worst <= settle_eps else "FAIL", worst_body, worst))
    ok = ok and worst <= settle_eps

    # INV3: reproducible. Two runs should agree to ~1 cm (allows GPU float noise; catches chaos).
    h1, d1 = runs[1]
    det_eps, maxd = 0.01, 0.0
    if h1 != h0:
        maxd = float("inf")
    else:
        for s in sorted(set(d0) & set(d1)):
            for a, b in zip(d0[s], d1[s]):
                maxd = max(maxd, abs(a - b))
    print("  INV3 reproducible (run0~run1 <%.2fm):      %s  (max diff %.4gm)"
          % (det_eps, "PASS" if maxd <= det_eps else "FAIL", maxd))
    ok = ok and maxd <= det_eps

    # --require-newton: assert the default backend was ACTUALLY Newton, not a silent ODE
    # fallback (which happens when warp/newton isn't importable in the launch environment).
    # Behavioral check: run the same world forced to ODE and require the default run to
    # DIVERGE from it — Newton's solver legitimately differs from ODE at contact (oracle_drop
    # ~0.27 m), whereas a fell-back-to-ODE default matches ODE to ~0. This keeps a Newton-box
    # gate from passing green while really only exercising ODE. (Plain --gate stays portable:
    # on a no-GPU box it gates whichever backend resolved; --require-newton is for the
    # Newton-default pre-push hook on a machine that MUST have Newton.)
    if require_newton:
        ho, do = capture(world, os.path.join(tmp, "ode_ref.csv"), steps, True, "ode-ref")
        nd = 0.0
        if ho is not None and ho == h0:
            for s in sorted(set(d0) & set(do)):
                for a, b in zip(d0[s], do[s]):
                    nd = max(nd, abs(a - b))
        newton_active = ho is not None and ho == h0 and nd >= 0.01
        print("  REQUIRE-NEWTON (default diverges from ODE >=1cm):  %s  (max diff %.4gm)"
              % ("PASS" if newton_active else "FAIL", nd))
        if not newton_active:
            print("\n[gate] FAIL - --require-newton: the default backend matched ODE, so Newton was")
            print("       NOT active (warp/newton not importable in this launch environment, or the")
            print("       build is NEWTON=OFF). Install the runtime (pip install newton warp-lang) or")
            print("       launch where the embedded interpreter finds it. No silent ODE-as-Newton pass.")
            return 1
        ok = ok and newton_active

    print("\n[gate] %s" % ("PASS - physics is sane, settled, reproducible." if ok
                           else "FAIL - invariant(s) above violated (a real regression)."))
    print("[gate] portable: gates whichever backend resolved (Newton when available, else ODE); no")
    print("       hardware-specific golden. raw CSVs+logs: %s" % tmp)
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default=DEFAULT_WORLD)
    ap.add_argument("--steps", type=int, default=200)
    ap.add_argument("--eps", type=float, default=1e-3, help="divergence threshold in metres")
    ap.add_argument("--gate", action="store_true",
                    help="pass/fail mode: assert physical invariants on the default backend (CI gate, "
                         "exit 1 on violation) instead of the ODE-vs-Newton divergence report")
    ap.add_argument("--require-newton", action="store_true",
                    help="with --gate: also FAIL if the default backend isn't actually Newton (it fell "
                         "back to ODE because warp/newton wasn't importable). For the Newton-default "
                         "pre-push hook on a machine that must have Newton; catches a silent fallback.")
    args = ap.parse_args()

    if not os.path.exists(BIN):
        print("omnisim-bin not found at %s — build it first." % BIN)
        return 0

    if args.gate:
        tmp = tempfile.mkdtemp(prefix="physics_gate_")
        # Settling needs ample steps (highest faller lands ~step 130 at dt=4ms); default to 400 if the
        # caller didn't raise --steps so INV2 isn't tripped by bodies still in flight.
        return gate(args.world, max(args.steps, 400), tmp, args.require_newton)

    warn_if_already_running()
    tmp = tempfile.mkdtemp(prefix="physics_oracle_")
    ode_csv = os.path.join(tmp, "ode.csv")
    new_csv = os.path.join(tmp, "newton.csv")

    print("[oracle] reference run (OMNISIM_LEGACY=1 -> ODE) ...")
    h1, d1 = capture(args.world, ode_csv, args.steps, True, "reference/ODE")
    print("[oracle] candidate run (migration default -> Newton when available) ...")
    h2, d2 = capture(args.world, new_csv, args.steps, False, "candidate/Newton")
    if h1 is None or h2 is None:
        print("[oracle] FAILED to capture trajectories (controller/world wiring?).")
        print("         ode rows=%s newton rows=%s  (dir: %s)"
              % (None if d1 is None else len(d1), None if d2 is None else len(d2), tmp))
        return 0
    if h1 != h2:
        print("[oracle] header mismatch between runs:\n  ODE:    %s\n  Newton: %s" % (h1, h2))
        return 0

    bodies = bodies_from_header(h1)
    steps = sorted(set(d1) & set(d2))
    print("\n[oracle] %d bodies, %d aligned steps, eps=%g m\n" % (len(bodies), len(steps), args.eps))

    overall_max = 0.0
    print("  %-12s %14s %16s" % ("body", "max |dpos| (m)", "first-diverge step"))
    print("  " + "-" * 46)
    for bi, name in enumerate(bodies):
        cx = 1 + bi * 3  # offset into the value row (no step col there); d[*] excludes step
        base = bi * 3
        max_d = 0.0
        first = None
        for s in steps:
            a = d1[s]
            b = d2[s]
            if base + 2 >= len(a) or base + 2 >= len(b):
                continue
            dx = a[base] - b[base]
            dy = a[base + 1] - b[base + 1]
            dz = a[base + 2] - b[base + 2]
            dist = (dx * dx + dy * dy + dz * dz) ** 0.5
            if dist > max_d:
                max_d = dist
            if first is None and dist > args.eps:
                first = s
        overall_max = max(overall_max, max_d)
        print("  %-12s %14.6f %16s" % (name, max_d, "-" if first is None else str(first)))

    print("\n[oracle] overall max divergence: %.6f m" % overall_max)
    if overall_max < args.eps:
        print("[oracle] NOTE: near-zero divergence everywhere. Either Newton matches ODE to tol, OR")
        print("         Newton was unavailable and run (B) fell back to ODE (check the build / Warp /")
        print("         that you launched from PowerShell). A real Newton contact run diverges at landing.")
    else:
        print("[oracle] Newton diverges from the ODE reference at contact (as expected); the table above")
        print("         localizes where. This is legacy verifying the new backend.")
    print("[oracle] raw CSVs: %s" % tmp)
    return 0


if __name__ == "__main__":
    sys.exit(main())
