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

"""physics_oracle — the Newton physical-invariants gate (was the dual-backend physics verifier).

⚠️ HALF OF THIS TOOL IS RETIRED (2026-08-08). It had two modes:

  --gate   ALIVE, and it is the mode the pre-push hook runs. Asserts PHYSICAL INVARIANTS that any
           sane rigid-body result must satisfy, on the default backend. It never needed ODE — see
           gate() below for the invariants and why they were chosen over a cross-backend diff.

  (default) DEAD, and now REFUSED. The divergence report ran the SAME world twice — (A) reference
           with OMNISIM_LEGACY=1, every Solid forced to ODE; (B) candidate on the migration default
           (Newton) — and diffed the two per-body trajectories. **src/ode was DELETED (commit
           bdc02139)**, so arm (A) cannot exist. Both of its possible fates are useless and one is
           actively dangerous: the current engine IGNORES OMNISIM_LEGACY (verified 2026-08-08 — the
           run comes up on Newton and writes a normal sidecar), so arm (A) would be a second NEWTON
           run and this tool would print ~0 divergence as if the backends agreed. An earlier build
           the same day still honoured it, and then the run was FROZEN (measured on
           tests/physics/worlds/contact_points.omniworld: the cone sat bit-identical to its authored pose
           for 3000 ms while the Newton arm rolled it 0.9 m), making the report a diff against a
           still image. A false "they match" is the worst output this tool could produce.

Note that gate() was ALREADY the answer to this problem, for a reason that outlived ODE: asserting
Newton == ODE is wrong (different solvers legitimately differ) and a Newton trajectory golden is not
portable (Warp/GPU output varies by hardware), so the gate asserts backend-independent invariants
instead. That reasoning is why there is still a working gate here at all.

The frozen ODE reference numbers are in tests/goldens/ode_oracle_goldens.json.

Run from PowerShell (NOT MSYS bash), so the embedded interpreter finds Warp and the run actually
exercises Newton:

    python scripts/dev/physics_oracle.py --gate [--require-newton]
    python scripts/dev/physics_oracle.py --gate --world tests/physics/worlds/oracle_drop.omniworld

Exit code: --gate returns non-zero on an invariant violation; the retired report mode returns 2.
"""

import argparse
import csv
import json
import math
import os
import subprocess
import sys
import tempfile
import time

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if REPO not in sys.path:
    sys.path.insert(0, REPO)
from omnisim.paths import resolve_omnisim_binary  # noqa: E402

# Cross-platform binary resolution (Windows msys64, Linux bin/, macOS bundle).
# Fall back to the legacy Windows path so the exists() check still prints a
# helpful location on an unbuilt checkout.
BIN = resolve_omnisim_binary() or os.path.join(REPO, "msys64", "mingw64", "bin", "omnisim-bin.exe")
DEFAULT_WORLD = os.path.join(REPO, "tests", "physics", "worlds", "oracle_drop.omniworld")


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


def sidecar_for(out_csv):
    """Path of the Newton backend-verdict sidecar for the run that wrote `out_csv`."""
    return log_for(out_csv) + ".newton.json"


def log_for(out_csv):
    """Per-run engine log path. Per-run (not the shared install-root omnisim_log.txt) so the
    `.newton.json` verdict sidecar beside it is unambiguously THIS run's -- AGENTS.md §3e."""
    return out_csv + ".engine.log"


def run_once(world, out_csv, steps, force_legacy):
    env = dict(os.environ)
    env["OMNISIM_ORACLE_OUT"] = out_csv
    env["OMNISIM_ORACLE_STEPS"] = str(steps)
    env["OMNISIM_LOG_PATH"] = log_for(out_csv)
    # RETIRED KNOBS, SCRUBBED: OMNISIM_LEGACY / OMNISIM_FORCE_ODE name a backend that no longer
    # exists (src/ode deleted, commit bdc02139). `force_legacy` is dead too -- kept in the
    # signature only because the retired report path below still passes it; it is IGNORED, because
    # honouring it would produce a frozen, physics-free run that a caller would read as data.
    env.pop("OMNISIM_LEGACY", None)
    env.pop("OMNISIM_FORCE_ODE", None)
    for stale in (log_for(out_csv), sidecar_for(out_csv)):
        try:
            os.remove(stale)
        except OSError:
            pass
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
        # below): make the default-backend run's binary OmLog::fatal (non-zero exit) if the
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

    # --require-newton: assert the default backend was ACTUALLY Newton.
    #
    # CHANGED 2026-08-08. This used to be a BEHAVIOURAL cross-backend check: run the same world
    # forced to ODE and require the default run to diverge from it by >=1 cm, on the theory that
    # Newton's solver legitimately differs from ODE at contact while a fell-back-to-ODE default
    # would match ODE to ~0. src/ode was DELETED (commit bdc02139), which breaks that check in the
    # sneaky direction: the forced arm now produces a FROZEN scene, so ANY run in which something
    # moved diverges from it and passes. It would no longer distinguish Newton from ODE -- only
    # "physics happened" from "physics did not". It also cost a whole extra engine launch.
    #
    # The correct proof is the engine's own race-free verdict sidecar (AGENTS.md): OmNewtonBackend::
    # finalizeWorld writes <OMNISIM_LOG_PATH>.newton.json, and OmLog deletes any stale copy when it
    # truncates the log at startup, so its presence means Newton drove THIS run. degraded=true flags
    # a solver fallback. No second launch, no inference from absence.
    if require_newton:
        sc_path = sidecar_for(os.path.join(tmp, "gate0.csv"))
        verdict, detail = None, "sidecar ABSENT at %s" % sc_path
        try:
            with open(sc_path, "r", encoding="utf-8", errors="replace") as fh:
                verdict = json.load(fh)
            detail = "backend=%s solver=%s degraded=%s finalised=%s" % (
                verdict.get("backend"), verdict.get("solver"),
                verdict.get("degraded"), verdict.get("finalised"))
        except (OSError, ValueError) as exc:
            if verdict is not None:
                detail = "sidecar unreadable: %s" % exc
        newton_active = bool(verdict and verdict.get("backend") == "newton"
                             and not verdict.get("degraded")
                             and verdict.get("finalised", True))
        print("  REQUIRE-NEWTON (engine verdict sidecar):  %s  (%s)"
              % ("PASS" if newton_active else "FAIL", detail))
        if not newton_active:
            print("\n[gate] FAIL - --require-newton: the engine did not write a clean Newton verdict")
            print("       for this run. Either warp/newton is not importable by the embedded")
            print("       interpreter (install: pip install newton warp-lang; and on Windows launch")
            print("       from PowerShell, not MSYS bash), the world never reached finalize inside")
            print("       the step budget, or the solver degraded. There is NO second backend to have")
            print("       fallen back to any more (src/ode deleted, bdc02139) -- a failure here means")
            print("       the run had no working physics at all, not that it used the old one.")
            return 1
        ok = ok and newton_active

    print("\n[gate] %s" % ("PASS - physics is sane, settled, reproducible." if ok
                           else "FAIL - invariant(s) above violated (a real regression)."))
    print("[gate] portable: backend-independent invariants, no hardware-specific golden. There is only")
    print("       ONE backend now (src/ode deleted, bdc02139), so plain --gate no longer 'gates")
    print("       whichever resolved'; --require-newton proves the engine's own verdict on top.")
    print("       raw CSVs+logs: %s" % tmp)
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default=DEFAULT_WORLD)
    ap.add_argument("--steps", type=int, default=200)
    ap.add_argument("--eps", type=float, default=1e-3, help="divergence threshold in metres")
    ap.add_argument("--gate", action="store_true",
                    help="THE ONLY LIVE MODE. Pass/fail: assert physical invariants on the default "
                         "backend (CI gate, exit 1 on violation). Without it the tool used to print "
                         "an ODE-vs-Newton divergence report; that mode is retired and exits 2 "
                         "(src/ode deleted, commit bdc02139).")
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

    # ---- RETIRED: the ODE-vs-Newton divergence report --------------------
    print("[oracle] The divergence report is RETIRED and will not be run.")
    print("[oracle] It needed a reference arm forced to ODE (OMNISIM_LEGACY=1). ODE was DELETED")
    print("[oracle] (src/ode, commit bdc02139), so that arm cannot exist -- and the failure is SILENT,")
    print("[oracle] not loud: the engine now IGNORES the var, so the reference run would be a second")
    print("[oracle] Newton run and this tool would report ~0 divergence, i.e. a false 'the two")
    print("[oracle] backends agree'. On an earlier build the same day the var WAS honoured and the")
    print("[oracle] run was FROZEN instead (zero motion in 3000 ms). Neither is a reference.")
    print("[oracle]")
    print("[oracle] Use --gate instead: the PHYSICAL-INVARIANTS mode, which never needed a second")
    print("[oracle] backend and is what the pre-push hook runs:")
    print("[oracle]     python scripts/dev/physics_oracle.py --gate --require-newton")
    print("[oracle] Frozen ODE reference values: tests/goldens/ode_oracle_goldens.json")
    return 2


if __name__ == "__main__":
    sys.exit(main())
