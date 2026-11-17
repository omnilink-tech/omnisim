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

"""dual_backend_oracle — the unified verifier (architectural-baseline.md §2 B1). HALF-DUAL NOW.

⚠️ THE PHYSICS HALF IS NO LONGER DUAL (2026-08-08). This tool was built on "use legacy to verify the
new architecture" (default-flip-plan.md §3.3): run a world on legacy (ODE + WREN) and on the new stack
(Newton + wgpu) and report per-arm divergence. **src/ode was DELETED (commit bdc02139)**, so the
physics arm has no legacy side. The RENDER arm is unaffected — WREN is still the default main-view
renderer and wgpu is still opt-in, so WREN-vs-wgpu remains a genuine dual comparison.

It is a thin front-end over the arm-specific oracles, so each arm stays the single source of truth for
its own launch handling, retries, and tolerances:

  PHYSICS arm  (🟦 Newton/physics session): Newton PHYSICAL INVARIANTS, not a cross-backend diff.
               Delegates to scripts/dev/physics_oracle.py --gate, which asserts properties any sane
               rigid-body result must satisfy (finite / settled / run-to-run reproducible) plus, with
               --require-newton, the engine's own `.newton.json` verdict that Newton actually drove the
               run. The old ODE-vs-Newton divergence REPORT mode of that script is retired and exits 2,
               so this front-end always passes --gate. Note the invariants approach was ALREADY
               preferred over a cross-backend diff before the deletion, for a reason that outlived it:
               asserting Newton == ODE is wrong (different solvers legitimately differ) and a Newton
               trajectory golden is not portable across GPUs.

  RENDER arm   (🟥 render session): WREN vs wgpu. Two layers, picked up automatically:
               (1) per-world image diff — the §3.3 "render the same Viewpoint under WREN and wgpu and
                   image-diff them" tool. This is the render session's SEAM: drop in
                   scripts/dev/render_oracle.py exposing a `--world W` CLI (exit 0 = within tol) and
                   this front-end calls it. Until it exists, the render arm reports it as UNWIRED
                   (honest, not a fake pass).
               (2) standing sensor/parity regression — scripts/dev/wgpu_sensor_regression.py (fixed
                   worlds, exists today). Used as the render-arm regression check in --gate mode when
                   no per-world render_oracle.py is present.

Why a front-end rather than merging the scripts: the two arms have genuinely different shapes (physics
asserts invariants on a trajectory; render diffs an image) and different runtime needs, but a developer
debugging a world wants ONE command that says "is this world's physics sane, and here is where wgpu
diverges from WREN, for THIS world." That single pane is the deliverable.

Run from PowerShell (NOT MSYS bash) so the embedded interpreter finds Warp — otherwise the physics
arm's Newton run cannot come up at all. There is no second backend to fall back to any more (src/ode
deleted, bdc02139), so the failure is now loud rather than a false "they match".

Usage:
    python scripts/dev/dual_backend_oracle.py [--world W] [--steps N]
    python scripts/dev/dual_backend_oracle.py --gate            # pass/fail (CI/pre-push shape)
    python scripts/dev/dual_backend_oracle.py --arms physics    # one arm only (physics|render|both)

Exit code:
    report mode: 0 always (measurement tool).
    --gate mode: 0 iff every selected, WIRED arm passed; non-zero otherwise. An UNWIRED render arm in
                 --gate mode is reported and, by default, does NOT fail the gate (use --strict-render
                 to make an unwired render arm a failure once the render session owns it).
"""

import argparse
import os
import subprocess
import sys

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DEV = os.path.join(REPO, "scripts", "dev")
if REPO not in sys.path:
    sys.path.insert(0, REPO)
from omnisim.paths import resolve_omnisim_binary  # noqa: E402

# Cross-platform binary resolution (Windows msys64, Linux bin/, macOS bundle).
# Fall back to the legacy Windows path so the exists() check still prints a
# helpful location on an unbuilt checkout.
BIN = resolve_omnisim_binary() or os.path.join(REPO, "msys64", "mingw64", "bin", "omnisim-bin.exe")

PHYSICS_ORACLE = os.path.join(DEV, "physics_oracle.py")
RENDER_ORACLE = os.path.join(DEV, "render_oracle.py")          # the render session's seam (may not exist yet)
SENSOR_REGRESSION = os.path.join(DEV, "wgpu_sensor_regression.py")
DEFAULT_WORLD = os.path.join(REPO, "tests", "physics", "worlds", "oracle_drop.omniworld")


def _run(argv, label):
    """Run a child arm, streaming its output through, and return its exit code. Each arm owns its own
    launch retries and env handling, so we just inherit stdout/stderr and report the verdict."""
    print("\n" + "=" * 78)
    print("[dual-oracle] %s arm  ->%s" % (label, " ".join(os.path.basename(a) for a in argv[1:2]) or argv))
    print("=" * 78, flush=True)
    try:
        rc = subprocess.run(argv, cwd=REPO).returncode
    except FileNotFoundError:
        print("[dual-oracle] %s arm: interpreter/script not found: %s" % (label, argv))
        return 127
    return rc


def run_physics_arm(world, steps, gate):
    # ALWAYS --gate since 2026-08-08. physics_oracle.py had two modes: the ODE-vs-Newton divergence
    # REPORT (no flag) and the physical-invariants GATE (--gate). src/ode was DELETED (commit
    # bdc02139), so the report mode is retired and exits 2; passing no flag here would make this arm
    # look like a failure when it is the tool that is gone. The invariants gate is portable (it asserts
    # backend-independent properties, so it runs on a no-GPU box too); --require-newton is left to the
    # caller's environment, which is where the pre-push hook adds it.
    argv = [sys.executable, PHYSICS_ORACLE, "--world", world, "--steps", str(steps), "--gate"]
    del gate  # no longer selects a mode; the gate is the only mode
    return _run(argv, "PHYSICS (Newton invariants)")


def run_render_arm(world, gate, strict_render):
    """Prefer the render session's per-world WREN-vs-wgpu diff (render_oracle.py) when present; else
    fall back to the standing sensor regression as the render-arm check. Honest about the seam."""
    if os.path.exists(RENDER_ORACLE):
        # Label is "wgpu sanity", not "WREN<->wgpu": the parity arm died with WREN at D1.4
        # (976b9449d) and render_oracle.py now grades draw count / degenerate transforms /
        # id-coverage / non-background pixels. The --world contract and exit code are unchanged.
        return _run([sys.executable, RENDER_ORACLE, "--world", world], "RENDER (wgpu sanity, per-world)")
    # No per-world render oracle yet — that's the render session's B1 half.
    print("\n" + "=" * 78)
    print("[dual-oracle] RENDER (WREN<->wgpu) arm")
    print("=" * 78)
    print("[dual-oracle] per-world render_oracle.py is UNWIRED (render session's B1 seam: a --world")
    print("              WREN-vs-wgpu image diff, default-flip-plan.md §3.3). Falling back to the")
    print("              standing sensor/parity regression (fixed worlds) as the render-arm check.", flush=True)
    if not gate:
        print("[dual-oracle] (report mode) skipping the fixed-world regression; run it directly for the")
        print("              full per-sensor report:  python scripts/dev/wgpu_sensor_regression.py")
        return None  # None = arm not evaluated (neither pass nor fail) in report mode
    rc = _run([sys.executable, SENSOR_REGRESSION], "RENDER (wgpu sensor regression, fixed worlds)")
    if not strict_render:
        # The fixed-world regression is a real check, so honor its result; but if it can't run here
        # (e.g. no Vulkan build) we don't want an unwired render arm to red the whole gate by default.
        return rc
    return rc


def main():
    ap = argparse.ArgumentParser(description="Unified dual-backend (legacy<->new) oracle — one command, both arms")
    ap.add_argument("--world", default=DEFAULT_WORLD,
                    help="world to verify both ways (physics arm; the render per-world diff uses it too)")
    ap.add_argument("--steps", type=int, default=200, help="physics-arm steps")
    ap.add_argument("--arms", default="both", choices=["both", "physics", "render"],
                    help="which arm(s) to run (default both)")
    ap.add_argument("--gate", action="store_true",
                    help="pass/fail mode: exit non-zero if any selected wired arm fails (CI/pre-push shape)")
    ap.add_argument("--strict-render", action="store_true",
                    help="in --gate mode, treat an unwired/unrunnable render arm as a failure "
                         "(use once the render session owns the render arm)")
    args = ap.parse_args()

    if not os.path.exists(BIN):
        print("omnisim-bin not found at %s — build it first." % BIN)
        return 0

    print("[dual-oracle] world: %s" % os.path.relpath(args.world, REPO))
    print("[dual-oracle] arms : %s   mode: %s" % (args.arms, "GATE" if args.gate else "report"))
    print("[dual-oracle] NOTE: run from PowerShell so the physics arm's Newton run finds Warp (else it")
    print("              silently falls back to ODE and divergence reads ~0).")

    results = {}  # arm -> exit code, or None if not evaluated
    if args.arms in ("both", "physics"):
        results["physics"] = run_physics_arm(args.world, args.steps, args.gate)
    if args.arms in ("both", "render"):
        results["render"] = run_render_arm(args.world, args.gate, args.strict_render)

    print("\n" + "#" * 78)
    print("[dual-oracle] PER-ARM SUMMARY")
    for arm, rc in results.items():
        if rc is None:
            verdict = "not evaluated (report mode; render per-world diff unwired)"
        elif rc == 0:
            verdict = "PASS" if args.gate else "ran (see divergence table above)"
        else:
            verdict = "FAIL (exit %d)" % rc
        print("[dual-oracle]   %-8s : %s" % (arm, verdict))
    print("#" * 78)

    if not args.gate:
        return 0
    # Gate: fail iff any evaluated arm returned non-zero.
    failed = [a for a, rc in results.items() if rc not in (0, None)]
    if failed:
        print("[dual-oracle] GATE FAIL: %s" % ", ".join(failed))
        return 1
    print("[dual-oracle] GATE PASS (all selected wired arms green).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
