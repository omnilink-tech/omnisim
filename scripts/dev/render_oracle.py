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

# D1.5 (WREN deleted at D1.4, commit 976b9449d): the WREN renderer no longer exists, so
# every WREN arm this tool could drive is RETIRED -- the engine warns about and ignores
# the retired selectors, and a "WREN arm" run renders wgpu. The frozen WREN reference
# images live in tests/rendering/wren_reference/ (captured pre-deletion). The tool itself
# is kept: its wgpu arms and its A/B harness remain useful.
"""render_oracle — per-world WREN-vs-wgpu image-parity oracle (architectural-baseline.md §2 B1 render arm + B2 golden).

The render-arm counterpart to physics_oracle.py: renders ONE world under WREN and wgpu from the same
Viewpoint and reports how close they are. It is the per-world seam dual_backend_oracle.py auto-detects —
called as `render_oracle.py --world W`, exit 0 = geometry rendered + golden matches, non-zero = regression.

Mechanism: drives the in-binary `wren-parity` self-check (OmWgpuView::runSelfCheck), the established
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
import io
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

# within-tol ADVISORY floor (NOT a hard gate — see the gate block in main()).
# Parity gate threshold, recalibrated for the DETERMINISTIC fixed-resolution offscreen golden
# (OmWrenWindow::grabSceneOffscreen, 1600x900): the city baseline measures within-tol=37% on EVERY
# run (4/4 identical; the old window-grab golden swung 16..62% with window layout — the number the
# old 55 threshold was calibrated against). 30 sits just under the deterministic baseline: variance
# is ~0 now, so a dip below it is a real rendering regression, while modest per-world variation
# still clears it. Re-raise as the lighting-parity ladder climbs.
# RETIRED with the WREN parity arm (D1.4, 976b9449d): nothing reads this any more.
DEFAULT_MIN_WITHIN_TOL = 30
# Golden tolerance (mean abs per-channel diff, 0-255); matches wgpu_probe_golden.py.
GOLDEN_THRESHOLD_PER_CHANNEL = 5
# Goldens are stored downscaled to this max side — keeps the in-repo reference tiny + resolution-robust.
GOLDEN_MAX_DIM = 384
# The self-check fires once the wgpu pane exposes. MEASURED 2026-08-24 (machine 9722d23d12a3,
# RTX 3060 laptop, warehouse_husky): a SUCCESSFUL self-check completes in 24.5 s, so 70 s is ~2.8x
# headroom. A crashed or failed launch EXITS the process and is detected in under a second, so it
# never spends this budget -- only a true hang does.
CAPTURE_TIMEOUT_S = 70
# 3 attempts, down from 6. The 6 was sized for the offscreen-GOLDEN grab (grabSceneOffscreen), which
# crashed the GUI ~2 in 3 launches -- that code was part of the WREN arm and died with it at D1.4
# (976b9449d). What is left is ordinary launch flakiness, which exits fast and so retries cheaply.
# Retrying does NOT fix a hang, so extra attempts buy only wall-clock: at 6 x 70 s this gate burned
# 7m02s on every render-touching push (measured 2026-08-24) before reporting UNAVAILABLE and exit 0.
CAPTURE_ATTEMPTS = 3
# Fail the gate when id-coverage is at or below this FRACTION of the pick buffer. 0.01 preserves the
# old `render-coverage` gate's threshold exactly (it failed on cov_frac <= 0.01).
DEFAULT_MIN_COVERAGE = 0.01

# Regexes over the self-check report lines (see OmWgpuView::runSelfCheck).
#
# ⚠️ THESE MARKERS WERE REPLACED 2026-08-24, AND THE REASON IS WORTH KEEPING. This tool used to
# poll for `wren-parity-shadowed:` and parse `render-coverage:`. Both strings were DELETED FROM THE
# ENGINE with WREN at D1.4 (976b9449d, 2026-08-23) -- `grep -rn wren-parity-shadowed src/` returns
# nothing -- so the poll could never succeed again. THE FAILURE WAS SILENT AND EXPENSIVE:
# run_selfcheck() timed out all 6 attempts (7m02s measured), returned None, main() printed
# UNAVAILABLE and, because this gate is lenient by design, exited 0 -- so .githooks/pre-push printed
# "Render gate passed" while asserting nothing whatsoever. D1.5 (1c4f1b413) added a retirement
# COMMENT to this file and never touched the polling logic, which is how it survived ~10 days of
# render-touching pushes.
#
# The current wgpu self-check emits the markers below instead. They carry the same meaning the old
# coverage gate carried: did scene geometry actually reach the renderer and get rasterized.
RE_DRAWS = re.compile(r"^collectWorldDraws = (\d+) shapes")
RE_DEGEN = re.compile(r"^degenerate-draw scan: (\d+) of (\d+) draws")
RE_IDCOV = re.compile(r"^id-coverage = (\d+) / (\d+)")
RE_SHOT = re.compile(r"^screenshot: \d+x\d+ render=\d+ non-bg-pixels=(\d+) saved=(\d+)")
# The screenshot step always writes ONE of three forms (the full line, "render failed", or "target
# unusable"), so its PRESENCE -- not its success -- is what marks the render battery finished.
RE_SHOT_ANY = re.compile(r"^screenshot:")

# Every marker the gate grades on. The report FILE appears early, well before these are written, so
# polling for mere existence parses a half-written report.
_REQUIRED = (RE_DRAWS, RE_DEGEN, RE_IDCOV, RE_SHOT_ANY)


def _report_complete(report):
    """True once the self-check has written every marker the gate grades on.

    Requiring the whole SET -- rather than whichever line happens to be last -- keeps this honest if
    the engine's battery is ever reordered: a marker that moves still has to appear."""
    try:
        with open(report, "r", errors="replace") as fh:
            report_lines = [ln.strip() for ln in fh]
    except OSError:
        return False
    return all(any(rx.match(ln) for ln in report_lines) for rx in _REQUIRED)


def _png_paths(report):
    # OmWgpuView writes its screenshot to OMNISIM_VIEW3D_WGPU_SELFCHECK + ".png" (the `screenshot:`
    # step of runSelfCheck). The old .wgpu.png / .wgpu_shadow.png / .wren.png were products of the
    # WREN-parity arm and are no longer produced by ANY build -- pointing --golden at them compared
    # against a file that never appears, which is a silent pass.
    return {"wgpu": report + ".png"}


def run_selfcheck(world):
    """Launch the GUI binary paused, drive the wgpu self-check, return the report path (or None).

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
        # RETIRED KNOB, SCRUBBED ON PURPOSE: OMNISIM_FORCE_ODE names a backend that no
        # longer exists (src/ode deleted, commit bdc02139). It matters even for a RENDER
        # comparison -- a physics-free run settles bodies differently (i.e. not at all),
        # so a stale export would change the pixels being diffed.
        env.pop("OMNISIM_FORCE_ODE", None)
        env.pop("OMNISIM_LEGACY", None)
        env.pop("OMNISIM_WGPU_MAINVIEW_FORCE", None)
        env.setdefault("WEBOTS_HOME", REPO)
        env.setdefault("OMNISIM_HOME", REPO)
        proc = subprocess.Popen([BIN, world, "--mode=pause", "--stdout", "--stderr"], cwd=REPO, env=env)
        deadline = time.time() + CAPTURE_TIMEOUT_S
        got = False
        while time.time() < deadline:
            if _report_complete(report):
                time.sleep(1)  # let the trailing lines + PNGs finish flushing
                got = True
                break
            if proc.poll() is not None:
                # binary exited; give the report one last chance to have completed, then stop
                if _report_complete(report):
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
    """Pull the measured render facts out of the self-check report.

    Returns a dict. A value is None when its marker was ABSENT -- never a substituted default, so an
    unmeasured quantity can never be graded as if it had been measured."""
    out = {"draws": None, "degenerate": None, "id_cov": None, "non_bg": None, "shot_saved": None}
    with open(report, "r", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            m = RE_DRAWS.match(line)
            if m:
                out["draws"] = int(m.group(1))
            m = RE_DEGEN.match(line)
            if m:
                out["degenerate"] = int(m.group(1))
            m = RE_IDCOV.match(line)
            if m:
                covered, total = int(m.group(1)), int(m.group(2))
                if total > 0:
                    out["id_cov"] = covered / total
            m = RE_SHOT.match(line)
            if m:
                out["non_bg"] = int(m.group(1))
                out["shot_saved"] = m.group(2) == "1"
    return out


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


# A healthy report, trimmed to the four lines the gate grades on. Values are the ones MEASURED on
# warehouse_husky (2026-08-24, machine 9722d23d12a3): 1052 draws, 0 degenerate, 12.45% id-coverage,
# 58065 non-background pixels.
_HEALTHY = """collectWorldDraws = 1052 shapes (1037 with albedo texture uploaded)
degenerate-draw scan: 0 of 1052 draws non-finite or |translation|>1e4; max|translation|=15
id-coverage = 28676 / 230400  (12.4462%)
screenshot: 640x480 render=1 non-bg-pixels=58065 saved=1  [PASS]
"""

# Each case is (name, report text, expected exit code). The four RED cases are the regressions this
# gate exists to catch, written as the report the engine WOULD emit if they happened.
_SELF_TEST_CASES = (
    ("healthy", _HEALTHY, 0),
    ("nothing-rendered (the A1 'floor vanished' shape)",
     _HEALTHY.replace("id-coverage = 28676 / 230400  (12.4462%)",
                      "id-coverage = 0 / 230400  (0.0000%)"), 1),
    ("no-geometry-collected",
     _HEALTHY.replace("collectWorldDraws = 1052 shapes", "collectWorldDraws = 0 shapes"), 1),
    ("degenerate-transforms",
     _HEALTHY.replace("degenerate-draw scan: 0 of 1052", "degenerate-draw scan: 7 of 1052"), 1),
    ("blank-frame",
     _HEALTHY.replace("non-bg-pixels=58065", "non-bg-pixels=12"), 1),
    # Not merely bad but UNREADABLE: an engine that died mid-battery leaves a partial report. That
    # must read as UNAVAILABLE (lenient exit 0), never as a silent pass with facts invented.
    ("truncated-report", "collectWorldDraws = 1052 shapes (1037 with albedo texture uploaded)\n", 0),
)


def _self_test():
    """Prove the gate can FAIL. Runs this same CLI over synthesized reports -- no engine, no GPU.

    This exists because the failure it replaces was invisible: the tool polled for a marker the
    engine had stopped emitting, so it reported UNAVAILABLE and exited 0 forever while the pre-push
    hook printed "Render gate passed". A gate nobody can see go red is indistinguishable from that.
    """
    import subprocess
    import tempfile

    ok = True
    with tempfile.TemporaryDirectory() as tmp:
        for name, text, expected in _SELF_TEST_CASES:
            path = os.path.join(tmp, "report.txt")
            io.open(path, "w", encoding="utf-8").write(text)
            proc = subprocess.run([sys.executable, os.path.abspath(__file__), "--from-report", path],
                                  capture_output=True, text=True)
            good = proc.returncode == expected
            ok = ok and good
            print("[render-oracle self-test] %-46s expected exit %d, got %d  [%s]"
                  % (name, expected, proc.returncode, "OK" if good else "WRONG"))
            if not good:
                print(proc.stdout)
    print("[render-oracle self-test] %s" % ("PASS - the gate is red-capable." if ok else "FAILED"))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description="Per-world wgpu render oracle (render arm of the dual oracle)")
    ap.add_argument("--world", help="world (.omniworld/.wbt) to render under wgpu and grade")
    ap.add_argument("--from-report", help="grade an EXISTING self-check report instead of launching "
                                          "the engine (debugging, and what --self-test drives)")
    ap.add_argument("--self-test", action="store_true",
                    help="prove this gate can go RED: synthesize known-bad reports and assert it fails them")
    ap.add_argument("--report", action="store_true", help="measure only; exit 0 regardless (no gate)")
    ap.add_argument("--strict", action="store_true", help="treat an UNAVAILABLE wgpu build as a failure")
    ap.add_argument("--min-coverage", type=float, default=DEFAULT_MIN_COVERAGE,
                    help="fail if id-coverage is <= this FRACTION of the pick buffer (default %(default)s)")
    # Retired with the WREN parity arm (D1.4, 976b9449d). Still ACCEPTED so an existing caller does
    # not die on an unknown flag, but it selects nothing -- and it says so rather than pretending.
    ap.add_argument("--min-within-tol", type=int, default=None, help=argparse.SUPPRESS)
    ap.add_argument("--golden", help="ALSO compare the wgpu render to this golden PNG (B2)")
    ap.add_argument("--update-golden", action="store_true", help="(re)write --golden from this render instead of comparing")
    args = ap.parse_args()
    if args.min_within_tol is not None:
        print("[render-oracle] note: --min-within-tol is RETIRED (the WREN parity arm died with WREN "
              "at D1.4, 976b9449d) and is IGNORED. The gate is id-coverage + draw sanity: --min-coverage.")

    if args.self_test:
        return _self_test()

    if args.from_report:
        report = args.from_report if os.path.isabs(args.from_report) else os.path.join(REPO, args.from_report)
        if not os.path.exists(report):
            print("[render-oracle] report not found: %s" % report)
            return 2
        print("[render-oracle] grading saved report (no engine launch): %s" % report)
    else:
        if not args.world:
            print("[render-oracle] --world is required (or pass --from-report / --self-test).")
            return 2
        if not os.path.exists(BIN):
            print("[render-oracle] omnisim-bin not found at %s — build it first." % BIN)
            return 0  # not built ⇒ not a failure (mirrors dual_backend_oracle's missing-bin handling)
        world = args.world if os.path.isabs(args.world) else os.path.join(REPO, args.world)
        if not os.path.exists(world):
            print("[render-oracle] world not found: %s" % world)
            return 2

        print("[render-oracle] world: %s" % os.path.relpath(world, REPO))
        print("[render-oracle] driving the wgpu self-check (OMNISIM_VIEW3D_WGPU, --mode=pause)...")
        report = run_selfcheck(world)
    if report is None:
        msg = ("[render-oracle] UNAVAILABLE: the wgpu self-check wrote no complete report "
               "(wgpu-native unavailable, or the pane never exposed).")
        print(msg)
        return 1 if args.strict else 0

    facts = parse_report(report)
    pngs = _png_paths(report)
    print("[render-oracle] report: %s" % report)
    print("[render-oracle]   collectWorldDraws = %s shapes"
          % (facts["draws"] if facts["draws"] is not None else "n/a"))
    print("[render-oracle]   degenerate draws  = %s"
          % (facts["degenerate"] if facts["degenerate"] is not None else "n/a"))
    print("[render-oracle]   id-coverage       = %s"
          % ("%.2f%%" % (facts["id_cov"] * 100) if facts["id_cov"] is not None else "n/a"))
    print("[render-oracle]   screenshot non-bg = %s px (saved=%s)"
          % (facts["non_bg"] if facts["non_bg"] is not None else "n/a",
             facts["shot_saved"] if facts["shot_saved"] is not None else "n/a"))

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

    # ── Gate ───────────────────────────────────────────────────────────────────────
    # WHAT THIS GATES, post-D1.4: that scene geometry actually reached the renderer and got
    # rasterized. That is the A1 "the floor vanished" regression, and it is the same property the
    # old `render-coverage:` gate asserted -- the MARKER changed, the MEANING did not.
    #
    # The WREN within-tol arm is GONE, not merely advisory: there is no second in-process renderer
    # left to compare against (OmWgpuView::runSelfCheck says exactly that where the parity block
    # used to be), so the number cannot be produced at all. The frozen WREN reference images live
    # in tests/rendering/wren_reference/.
    #
    # NOT gated here, deliberately: a per-world draw COUNT or a coverage FLOOR would be a golden
    # fitted to one world, and this tool is pointed at arbitrary worlds. Tight per-world regression
    # detection is the --golden compare (B2), which stays opt-in per caller.
    if all(facts[k] is None for k in ("draws", "id_cov", "non_bg")):
        print("[render-oracle] GATE: no render facts parsed - treating as UNAVAILABLE.")
        return 1 if args.strict else 0
    if facts["draws"] == 0:
        print("[render-oracle] GATE FAIL: collectWorldDraws = 0 (no geometry reached the renderer).")
        failed.append("no-draws")
    if facts["degenerate"]:
        print("[render-oracle] GATE FAIL: %d degenerate draw(s) (non-finite or |translation|>1e4)."
              % facts["degenerate"])
        failed.append("degenerate-draws")
    if facts["id_cov"] is not None and facts["id_cov"] <= args.min_coverage:
        print("[render-oracle] GATE FAIL: id-coverage %.4f%% <= %.4f%% (nothing rendered)."
              % (facts["id_cov"] * 100, args.min_coverage * 100))
        failed.append("coverage")
    if facts["shot_saved"] is False:
        print("[render-oracle] GATE FAIL: the wgpu screenshot did not render/save.")
        failed.append("screenshot")
    elif facts["non_bg"] is not None and facts["non_bg"] <= 1000:
        print("[render-oracle] GATE FAIL: screenshot has %d non-background pixels (blank frame)."
              % facts["non_bg"])
        failed.append("blank-frame")

    if failed:
        print("[render-oracle] GATE FAIL: %s" % ", ".join(failed))
        return 1
    print("[render-oracle] GATE PASS: %s draws, %s degenerate, id-coverage %s, %s non-bg px%s." % (
        facts["draws"] if facts["draws"] is not None else "n/a",
        facts["degenerate"] if facts["degenerate"] is not None else "n/a",
        "%.2f%%" % (facts["id_cov"] * 100) if facts["id_cov"] is not None else "n/a",
        facts["non_bg"] if facts["non_bg"] is not None else "n/a",
        " + golden match" if args.golden and not args.update_golden else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
