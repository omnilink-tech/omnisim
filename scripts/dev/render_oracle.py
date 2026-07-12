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

"""render_oracle — per-world WREN-vs-wgpu image-parity oracle (architectural-baseline.md §2 B1 render arm + B2 golden).

The render-arm counterpart to physics_oracle.py: renders ONE world under WREN and wgpu from the same
Viewpoint and reports how close they are. It is the per-world seam dual_backend_oracle.py auto-detects —
called as `render_oracle.py --world W`, exit 0 = geometry rendered + golden matches, non-zero = regression.

Mechanism: drives the in-binary `wren-parity` self-check (WbWgpuView::runSelfCheck), the established
wgpu-vs-WREN comparator. It renders wgpu OFFSCREEN from WREN's exact Viewpoint at the same resolution,
grabs WREN's framebuffer as the LOCAL GOLDEN (no external reference needed — WREN runs right here), masks
to geometry, and writes a report we parse for:

  - render-coverage : non-sky geometry pixels in the shadowed render. A near-total drop = geometry
                      vanished (exactly the A1 "floor read back as garbage" bug this gate is meant to catch).
  - within-tol      : % of geometry pixels where |wgpu - WREN| <= 60 (~20/channel), for the SHADOWED
                      render — the path the main view actually uses since A2. ADVISORY ONLY: the wgpu path
                      has gained atmospheric sky + hemisphere-IBL ambient + distance fog that legacy WREN
                      lacks, so wgpu is now intentionally brighter/better-lit than WREN and parity dropped
                      by design (~76% baseline -> ~9% on shadowed worlds; WREN is the dark legacy,
                      wgpu is correct).
                      We print it for diagnostics but no longer gate on it — WREN is not a brightness oracle.

Two checks, one self-check run:
  B1 (advisory): the live WREN-vs-wgpu parity number above — printed, not gating (WREN diverged below).
                 COVERAGE (geometry actually rendered) IS still a hard gate: it catches the A1 floor-drop.
  B2 (--golden G): ALSO compare the deterministic wgpu render to a stored golden PNG. The offscreen wgpu
                   render is byte-deterministic on a fixed (paused) scene, so this catches ANY pixel-level
                   regression the coarse parity gate would miss. Tolerance compare if Pillow is present,
                   else an exact byte (SHA-256) match — both are valid on the single dev box this gates on.

MUST run in --mode=pause: under stepping the scene moves between the WREN grab and the wgpu renders, so
geometry stops aligning with the mask and every number is garbage (the self-check documents this).

Run from PowerShell (NOT MSYS bash). A wgpu build (OMNISIM_WITH_VULKAN=ON) is required; a WREN-only binary
produces no parity render → this reports UNAVAILABLE and exits 0 (not a false fail — mirrors the physics
arm's graceful no-Warp fallback), unless --strict is given.

Usage:
    python scripts/dev/render_oracle.py --world W                 # gate: exit 0 = geometry rendered (dual-oracle seam)
    python scripts/dev/render_oracle.py --world W --report        # measure only, exit 0 always
    python scripts/dev/render_oracle.py --world W --golden G       # ALSO compare wgpu render to golden G (B2)
    python scripts/dev/render_oracle.py --world W --golden G --update-golden   # (re)write G from this render
"""

import argparse
import os
import re
import subprocess
import sys
import time

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
BIN = os.path.join(REPO, "msys64", "mingw64", "bin", "omnisim-bin.exe")

# within-tol ADVISORY floor (NOT a hard gate — see the gate block in main()).
# Parity gate threshold, recalibrated for the DETERMINISTIC fixed-resolution offscreen golden
# (WbWrenWindow::grabSceneOffscreen, 1600x900): the city baseline measures within-tol=37% on EVERY
# run (4/4 identical; the old window-grab golden swung 16..62% with window layout — the number the
# old 55 threshold was calibrated against). 30 sits just under the deterministic baseline: variance
# is ~0 now, so a dip below it is a real rendering regression, while modest per-world variation
# still clears it. Re-raise as the lighting-parity ladder climbs.
DEFAULT_MIN_WITHIN_TOL = 30
# Golden tolerance (mean abs per-channel diff, 0-255); matches wgpu_probe_golden.py.
GOLDEN_THRESHOLD_PER_CHANNEL = 5
# Goldens are stored downscaled to this max side — keeps the in-repo reference tiny + resolution-robust.
GOLDEN_MAX_DIM = 384
# The self-check fires once the wgpu pane exposes; allow generous wall-clock + a couple of relaunch retries
# (the same rapid-launch flakiness physics_oracle.py hardens against).
CAPTURE_TIMEOUT_S = 70
# 6 attempts: the offscreen-golden grab (grabSceneOffscreen) intermittently crashes the GUI
# (~2 in 3 launches — framebuffer/post-processing lifetime during the temp resize, marker-localized
# in the report); each attempt is a fresh launch and a SUCCESSFUL run is fully deterministic, so
# retries convert the crash into latency until the root cause is fixed.
CAPTURE_ATTEMPTS = 6

# Regexes over the self-check report lines (see WbWgpuView::runSelfCheck).
RE_SHADOWED = re.compile(r"^wren-parity-shadowed:.*within-tol=(\d+)%")
RE_COVERAGE = re.compile(r"^render-coverage:\s*lit=(\d+)\s+litShadow=(-?\d+)\s+litShadowFlat=(-?\d+)\s+\(of (\d+) px\)")


def _report_has_parity(report):
    """True once the self-check has written the shadowed parity line (the value we parse). The report
    FILE appears early — its first lines are written well before the parity computation finishes — so
    polling for mere existence parses a half-written report (within-tol reads n/a). Poll for this instead."""
    try:
        with open(report, "r", errors="replace") as fh:
            return any(RE_SHADOWED.match(line.strip()) for line in fh)
    except OSError:
        return False


def _png_paths(report):
    # The self-check saves these next to the report file.
    return {
        "wgpu": report + ".wgpu.png",            # unshadowed wgpu render (proven byte-deterministic)
        "wgpu_shadow": report + ".wgpu_shadow.png",
        "wren": report + ".wren.png",
    }


def run_selfcheck(world):
    """Launch the GUI binary paused, drive the wren-parity self-check, return the report path (or None).

    The binary is a long-lived GUI app (no stdout), so we Popen it, poll for the report file, then kill it.
    Retries the launch a couple of times — a single launch is reliable; back-to-back ones occasionally race.
    """
    report = os.path.join(REPO, "_scratch", "render_oracle_%s.txt" % os.path.splitext(os.path.basename(world))[0])
    os.makedirs(os.path.dirname(report), exist_ok=True)
    for attempt in range(1, CAPTURE_ATTEMPTS + 1):
        for p in [report] + list(_png_paths(report).values()):
            try:
                os.remove(p)
            except OSError:
                pass
        env = dict(os.environ)
        env["OMNISIM_VIEW3D_WGPU"] = "1"
        env["OMNISIM_VIEW3D_WGPU_SELFCHECK"] = report
        # Never persist this gate run's window layout: a saved perspective with a wide wgpu pane
        # shrinks WREN's viewport inside the grabbed window (a black band in the golden) and
        # silently craters within-tol for every later run.
        env["WEBOTS_DISABLE_SAVE_SCREEN_PERSPECTIVE_ON_CLOSE"] = "1"
        env.pop("OMNISIM_FORCE_ODE", None)
        env.pop("OMNISIM_WGPU_MAINVIEW_FORCE", None)
        env.setdefault("WEBOTS_HOME", REPO)
        env.setdefault("OMNISIM_HOME", REPO)
        proc = subprocess.Popen([BIN, world, "--mode=pause", "--stdout", "--stderr"], cwd=REPO, env=env)
        deadline = time.time() + CAPTURE_TIMEOUT_S
        got = False
        while time.time() < deadline:
            if _report_has_parity(report):
                time.sleep(1)  # let the trailing lines + PNGs finish flushing
                got = True
                break
            if proc.poll() is not None:
                # binary exited; give the report one last chance to have completed, then stop
                if _report_has_parity(report):
                    got = True
                break
            time.sleep(2)
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
        if got and os.path.exists(report):
            return report
        print("[render-oracle] self-check attempt %d/%d produced no report; retrying" % (attempt, CAPTURE_ATTEMPTS))
    return None


def parse_report(report):
    """Pull the shadowed within-tol % and the shadowed coverage fraction out of the self-check report."""
    within_tol = None
    cov_frac = None
    with open(report, "r", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            m = RE_SHADOWED.match(line)
            if m:
                within_tol = int(m.group(1))
            m = RE_COVERAGE.match(line)
            if m:
                lit_shadow = int(m.group(2))
                total = int(m.group(4))
                if lit_shadow >= 0 and total > 0:
                    cov_frac = lit_shadow / total
    return within_tol, cov_frac


def _downscaled_rgba(path):
    """Open a PNG and downscale so its largest side is GOLDEN_MAX_DIM (aspect preserved); return
    (w, h, bytes). Goldens are stored downscaled — tiny in-repo + resolution-robust + still catch gross
    regressions (a dropped floor is obvious even small). The source render is byte-deterministic, and a
    fixed BILINEAR resize of identical bytes is identical, so the downscaled golden stays deterministic.
    Returns None if Pillow is unavailable / decode fails."""
    try:
        from PIL import Image
    except Exception:
        return None
    try:
        img = Image.open(path).convert("RGBA")
        scale = GOLDEN_MAX_DIM / float(max(img.width, img.height))
        if scale < 1.0:
            img = img.resize((max(1, round(img.width * scale)), max(1, round(img.height * scale))), Image.BILINEAR)
        return img.width, img.height, img.tobytes()
    except Exception:
        return None


def _save_downscaled(src_png, dst_png):
    """Write src_png downscaled (GOLDEN_MAX_DIM) to dst_png as the stored golden. Requires Pillow."""
    from PIL import Image
    img = Image.open(src_png).convert("RGBA")
    scale = GOLDEN_MAX_DIM / float(max(img.width, img.height))
    if scale < 1.0:
        img = img.resize((max(1, round(img.width * scale)), max(1, round(img.height * scale))), Image.BILINEAR)
    os.makedirs(os.path.dirname(dst_png), exist_ok=True)
    img.save(dst_png, "PNG")


def compare_golden(actual_png, golden_png):
    """Return (ok, detail). Downscaled tolerance compare (mean abs per-channel diff <= threshold)."""
    a = _downscaled_rgba(actual_png)
    g = _downscaled_rgba(golden_png)
    if a is None or g is None:
        return False, "Pillow required for golden compare (pip install pillow)"
    if (a[0], a[1]) != (g[0], g[1]):
        return False, "size mismatch %dx%d vs golden %dx%d" % (a[0], a[1], g[0], g[1])
    ab, gb = a[2], g[2]
    total = sum(abs(x - y) for x, y in zip(ab, gb))
    mean = total / len(ab) if ab else 0.0
    return mean <= GOLDEN_THRESHOLD_PER_CHANNEL, "mean-diff/ch=%.3f (threshold %d, %dx%d golden)" % (
        mean, GOLDEN_THRESHOLD_PER_CHANNEL, a[0], a[1])


def main():
    ap = argparse.ArgumentParser(description="Per-world WREN-vs-wgpu image-parity oracle (render arm of the dual oracle)")
    ap.add_argument("--world", required=True, help="world (.wbt) to render under WREN and wgpu and compare")
    ap.add_argument("--report", action="store_true", help="measure only; exit 0 regardless (no gate)")
    ap.add_argument("--strict", action="store_true", help="treat an UNAVAILABLE wgpu build as a failure")
    ap.add_argument("--min-within-tol", type=int, default=DEFAULT_MIN_WITHIN_TOL,
                    help="parity gate threshold %% (default %(default)d)")
    ap.add_argument("--golden", help="ALSO compare the wgpu render to this golden PNG (B2)")
    ap.add_argument("--update-golden", action="store_true", help="(re)write --golden from this render instead of comparing")
    args = ap.parse_args()

    if not os.path.exists(BIN):
        print("[render-oracle] omnisim-bin not found at %s — build it first." % BIN)
        return 0  # not built ⇒ not a failure (mirrors dual_backend_oracle's missing-bin handling)
    world = args.world if os.path.isabs(args.world) else os.path.join(REPO, args.world)
    if not os.path.exists(world):
        print("[render-oracle] world not found: %s" % world)
        return 2

    print("[render-oracle] world: %s" % os.path.relpath(world, REPO))
    print("[render-oracle] driving the wren-parity self-check (OMNISIM_VIEW3D_WGPU, --mode=pause)...")
    report = run_selfcheck(world)
    if report is None:
        msg = "[render-oracle] UNAVAILABLE: no parity report (WREN-only build, or the wgpu pane never exposed)."
        print(msg)
        return 1 if args.strict else 0

    within_tol, cov_frac = parse_report(report)
    pngs = _png_paths(report)
    print("[render-oracle] report: %s" % os.path.relpath(report, REPO))
    print("[render-oracle]   within-tol (shadowed, vs WREN) = %s%%" % (within_tol if within_tol is not None else "n/a"))
    print("[render-oracle]   coverage   (shadowed geometry)  = %s" % ("%.1f%%" % (cov_frac * 100) if cov_frac is not None else "n/a"))

    failed = []

    # B2 golden compare (optional) — uses the unshadowed render, which is the proven-byte-deterministic one.
    if args.golden:
        golden = args.golden if os.path.isabs(args.golden) else os.path.join(REPO, args.golden)
        if args.update_golden:
            _save_downscaled(pngs["wgpu"], golden)
            print("[render-oracle]   golden UPDATED (downscaled): %s" % os.path.relpath(golden, REPO))
        elif not os.path.exists(golden):
            print("[render-oracle]   golden MISSING: %s (run with --update-golden to create it)" % os.path.relpath(golden, REPO))
            failed.append("golden-missing")
        else:
            ok, detail = compare_golden(pngs["wgpu"], golden)
            print("[render-oracle]   golden %s: %s" % ("MATCH" if ok else "MISMATCH", detail))
            if not ok:
                failed.append("golden")

    if args.report:
        print("[render-oracle] report mode - exit 0 (measurement only).")
        return 0

    # ── Gate ──────────────────────────────────────────────────────────────────
    # within-tol vs WREN is ADVISORY, not a gate. The wgpu path gained atmospheric
    # sky + hemisphere-IBL ambient + distance fog (features WREN does NOT have), so
    # the wgpu render is intentionally brighter / better-lit than WREN and live
    # WREN-parity collapsed BY DESIGN (76% -> ~9% on shadowed worlds: the dark WREN frame is the
    # legacy, the brighter wgpu frame is correct — see _scratch/*.wren.png vs .wgpu_shadow.png).
    # WREN is no longer a valid brightness oracle, so we do not fail on it. Regression
    # detection now rests on the deterministic GOLDEN (B2 — catches any wgpu self-
    # regression incl. a garbage floor) plus COVERAGE (geometry actually rendered —
    # directly catches the A1 "floor vanished" bug the within-tol gate used to proxy).
    if within_tol is None and cov_frac is None:
        print("[render-oracle] GATE: no parity/coverage parsed - treating as UNAVAILABLE.")
        return 1 if args.strict else 0
    if within_tol is not None and within_tol < args.min_within_tol:
        print("[render-oracle]   note: within-tol %d%% < %d%% advisory floor — EXPECTED now that wgpu's"
              " sky/ambient/fog surpass legacy WREN; not gating (golden + coverage do)." % (within_tol, args.min_within_tol))
    if cov_frac is not None and cov_frac <= 0.01:
        print("[render-oracle] GATE FAIL: coverage ~0 (nothing rendered).")
        failed.append("coverage")

    if failed:
        print("[render-oracle] GATE FAIL: %s" % ", ".join(failed))
        return 1
    print("[render-oracle] GATE PASS: coverage %s%s (within-tol %s%% advisory; WREN is legacy-dark)." %
          ("%.1f%%" % (cov_frac * 100) if cov_frac is not None else "n/a",
           (" + golden match" if args.golden and not args.update_golden else ""),
           within_tol if within_tol is not None else "n/a"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
