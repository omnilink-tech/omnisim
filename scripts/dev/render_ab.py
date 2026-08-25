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
# D1.5 (WREN deleted at D1.4, commit 976b9449d): the WREN renderer no longer exists, so
# every WREN arm this tool could drive is RETIRED -- the engine warns about and ignores
# the retired selectors, and a "WREN arm" run renders wgpu. The frozen WREN reference
# images live in tests/rendering/wren_reference/ (captured pre-deletion). The tool itself
# is kept: its wgpu arms and its A/B harness remain useful.
"""render_ab.py — the WREN-retirement campaign's verification gate.

Renders one or more worlds twice (arm A vs arm B, differing only by environment)
through the wgpu main view, diffs the frames pixel-wise, and reports the numbers
that decide whether a port changed what the user sees.

Why this exists: every phase of the WREN retirement (see
docs/developer/wren-retirement-plan.md) has the same gate — "does the ported path
render what the old path rendered, and does it still cost the same?". Doing that by
hand is slow and easy to get subtly wrong (see the TRAPS below, each of which
produced a wrong answer at least once).

Usage
-----
  # W1a-style A/B: default arm vs the exact-revert hatch
  python scripts/dev/render_ab.py \
      --world projects/samples/demos/worlds/rendering/beauty_bench.omniworld \
      --world projects/samples/demos/worlds/showcase/city_traffic.omniworld \
      --arm-b OMNISIM_WGPU_NATIVE_MESH=0

  # baseline capture only (one arm), with perf
  python scripts/dev/render_ab.py --world <w> --no-diff --perf

  --json <path>   machine-readable results
  --frame N       dump frame (default 300; the city needs ~500 to settle its bake)
  --settle N      extra seconds to wait beyond the frame estimate

TRAPS this encodes (all measured, all cost real time before being understood)
---------------------------------------------------------------------------
 1. A windowed run is REQUIRED. --batch / --minimize / --no-rendering never repaint
    the wgpu main view, so the dump never fires and you get an empty result that
    looks like a render failure.
 2. Never kill the engine the moment the PNG appears. The dump is written
    progressively; killing mid-write leaves a TRUNCATED file that PIL reports as
    "image file is truncated" — or worse, that a naive size check reads as a real
    difference (a 1.1 MB truncated file vs a 3.2 MB complete one is not a
    rendering change). This script waits for the size to stop growing.
 3. Compare like with like. Two runs are not frame-synchronised, so TAA and any
    animated content (clouds drift, day-night, driven robots) put a small floor
    under the diff. Judge on `pixels_over_threshold`, not on mean != 0.
 4. One engine at a time. Concurrent runs contend for the GPU and skew renderMs,
    and this laptop is thermally limited.
 5. THIS SCRIPT IS STRUCTURALLY BLIND TO THE GEOMETRY OVERLAYS. The MAINVIEW dump reads back
    inside the scene render, before the W4a overlay pass runs (drawOverlayLines is documented
    present-path only, which is also why screenshots come out clean). So a MATCH here says
    NOTHING about selection outlines, bounding boxes, joint axes or any other View > Optional
    Rendering item -- it is the dump's contract, not evidence of absence. Verify those with the
    `ovCalled/ovBatches/ovVerts/ovOk` fields in the wgpu report line instead. This cost six
    separate attempts before it was understood.
 6. MEASURE THE NOISE FLOOR BEFORE YOU READ A DIFF (--noise-floor). On a world with a driven
    robot the run-to-run floor can EXCEED the effect you are looking for: measured on
    warehouse_husky, the "signal" was 989 px over threshold and the same-arm-twice floor was
    1060. On a static world (same world, controller "<none>") the floor is exactly 0 and the
    frames are bit-identical -- which is the only condition under which a small diff means
    anything. Prefer a static world; if you cannot have one, report the floor beside the diff.
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
BIN = REPO / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe"
if not BIN.exists():  # linux / macos layout
    for cand in (REPO / "bin" / "omnisim-bin", REPO / "Contents" / "MacOS" / "omnisim"):
        if cand.exists():
            BIN = cand
            break


def parse_env(pairs):
    out = {}
    for p in pairs or []:
        if "=" not in p:
            sys.exit(f"--arm env must be KEY=VALUE, got: {p}")
        k, v = p.split("=", 1)
        out[k] = v
    return out


def wait_for_complete(path: Path, timeout_s: float) -> bool:
    """True once `path` exists and its size has been stable for two polls.

    Trap 2: the dump is written progressively, so 'the file exists' is NOT
    'the file is complete'.
    """
    deadline = time.time() + timeout_s
    last = -1
    stable = 0
    while time.time() < deadline:
        if path.exists():
            size = path.stat().st_size
            if size > 0 and size == last:
                stable += 1
                if stable >= 2:
                    return True
            else:
                stable = 0
            last = size
        time.sleep(0.5)
    return path.exists() and path.stat().st_size > 0


def render(world: Path, out_png: Path, frame: int, extra_env: dict, log: Path,
           report: Path | None, settle: float) -> dict:
    env = os.environ.copy()
    env["OMNISIM_HOME"] = str(REPO)
    env["OMNISIM_WGPU_MAINVIEW_DUMP"] = str(out_png)
    env["OMNISIM_WGPU_MAINVIEW_DUMP_FRAME"] = str(frame)
    env["OMNISIM_LOG_PATH"] = str(log)
    if report:
        env["OMNISIM_WGPU_REPORT"] = str(report)
    env.update(extra_env)
    for stale in (out_png, log):
        stale.unlink(missing_ok=True)

    # Trap 1: windowed, realtime. Anything else never repaints.
    proc = subprocess.Popen([str(BIN), str(world), "--mode=realtime"], cwd=str(REPO), env=env)
    # Budget generously and RETURN EARLY: wait_for_complete exits the moment the PNG stops
    # growing, so a healthy run costs what it costs and only a broken one pays the ceiling.
    # Measured on machine 9722d23d12a3: world load + first OmniLight bake alone is ~12-16 s
    # before frame 0 of the counter, and --mode=realtime is vsync-paced well under 60 fps, so
    # a naive frame/60 estimate under-waits and reports a false "no frame".
    budget = frame / 25.0 + 20.0 + settle
    done = wait_for_complete(out_png, budget)
    time.sleep(1.0)  # let the last write flush before the kill
    proc.terminate()
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()

    errors = warnings = 0
    if log.exists():
        text = log.read_text(errors="replace").splitlines()
        errors = sum(1 for ln in text if ln.startswith("ERROR"))
        warnings = sum(1 for ln in text if ln.startswith("WARNING"))
    render_ms = None
    if report and report.exists():
        vals = []
        for ln in report.read_text(errors="replace").splitlines():
            if "renderMs=" in ln and "draws=" in ln:
                try:
                    vals.append(int(ln.split("renderMs=")[1].split()[0]))
                except (IndexError, ValueError):
                    pass
        if len(vals) > 1:
            render_ms = sorted(vals[1:])[len(vals[1:]) // 2]  # median, skipping frame 0
    return {"dumped": done, "errors": errors, "warnings": warnings, "render_ms": render_ms}


def diff(a_png: Path, b_png: Path, threshold: int) -> dict:
    try:
        from PIL import Image, ImageChops
        import numpy as np
    except ImportError:
        sys.exit("render_ab.py needs Pillow + numpy (pip install Pillow numpy)")
    a = Image.open(a_png).convert("RGB")
    b = Image.open(b_png).convert("RGB")
    if a.size != b.size:
        return {"same_size": False, "a_size": a.size, "b_size": b.size}
    arr = np.asarray(ImageChops.difference(a, b), dtype=np.int32).sum(axis=2)
    return {
        "same_size": True,
        "mean_abs": round(float(arr.mean()), 4),
        "max_abs": int(arr.max()),
        "pixels_over_threshold": int((arr > threshold).sum()),
        "threshold": threshold,
        "total_pixels": int(arr.size),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", action="append", required=True,
                    help="repo-relative or absolute world path (repeatable)")
    ap.add_argument("--arm-a", action="append", default=[], metavar="KEY=VALUE",
                    help="env for arm A (default: none = the shipped default)")
    ap.add_argument("--arm-b", action="append", default=[], metavar="KEY=VALUE",
                    help="env for arm B (e.g. the exact-revert hatch)")
    ap.add_argument("--frame", type=int, default=300)
    ap.add_argument("--settle", type=float, default=0.0)
    ap.add_argument("--threshold", type=int, default=30,
                    help="per-pixel summed-RGB difference counted as a REAL change (default 30, "
                         "which is above the TAA/animation floor)")
    ap.add_argument("--no-diff", action="store_true", help="render arm A only (baseline capture)")
    ap.add_argument("--noise-floor", action="store_true",
                    help="run arm A TWICE and diff those (trap 6). The result is the run-to-run "
                         "floor for this world; any real A/B must clear it to mean anything.")
    ap.add_argument("--perf", action="store_true", help="also collect renderMs")
    ap.add_argument("--out-dir", default=None, help="where frames land (default: a temp dir)")
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    if not BIN.exists():
        sys.exit(f"simulator binary not found at {BIN}")
    out_dir = Path(args.out_dir) if args.out_dir else Path(os.environ.get("TEMP", "/tmp")) / "render_ab"
    out_dir.mkdir(parents=True, exist_ok=True)

    env_a, env_b = parse_env(args.arm_a), parse_env(args.arm_b)
    results = []
    for w in args.world:
        world = Path(w) if Path(w).is_absolute() else REPO / w
        if not world.exists():
            print(f"!! world not found: {world}")
            results.append({"world": w, "error": "not found"})
            continue
        stem = world.stem
        print(f"== {stem}")
        row = {"world": w}
        a_png = out_dir / f"{stem}__A.png"
        row["arm_a"] = render(world, a_png, args.frame, env_a, out_dir / f"{stem}__A.log",
                              out_dir / f"{stem}__A.report" if args.perf else None, args.settle)
        print(f"   arm A: dumped={row['arm_a']['dumped']} errors={row['arm_a']['errors']}"
              + (f" renderMs={row['arm_a']['render_ms']}" if args.perf else ""))
        if not args.no_diff:
            b_png = out_dir / f"{stem}__B.png"
            # --noise-floor: arm B is arm A again, so the diff IS the run-to-run floor.
            env_b_eff = dict(env_a) if args.noise_floor else env_b
            row["arm_b"] = render(world, b_png, args.frame, env_b_eff, out_dir / f"{stem}__B.log",
                                  out_dir / f"{stem}__B.report" if args.perf else None, args.settle)
            print(f"   arm B: dumped={row['arm_b']['dumped']} errors={row['arm_b']['errors']}"
                  + (f" renderMs={row['arm_b']['render_ms']}" if args.perf else ""))
            if row["arm_a"]["dumped"] and row["arm_b"]["dumped"]:
                row["diff"] = diff(a_png, b_png, args.threshold)
                d = row["diff"]
                if d.get("same_size"):
                    verdict = "MATCH" if d["pixels_over_threshold"] == 0 else "DIFFERS"
                    print(f"   diff: {verdict} mean={d['mean_abs']} max={d['max_abs']} "
                          f"px>{d['threshold']}={d['pixels_over_threshold']}")
                    row["verdict"] = verdict
                else:
                    print(f"   diff: SIZE MISMATCH {d['a_size']} vs {d['b_size']}")
                    row["verdict"] = "SIZE_MISMATCH"
            else:
                row["verdict"] = "NO_FRAME"
                print("   diff: skipped — an arm produced no frame")
        results.append(row)

    if args.json:
        Path(args.json).write_text(json.dumps(results, indent=1))
        print(f"\nwrote {args.json}")
    bad = [r for r in results if r.get("verdict") in ("DIFFERS", "NO_FRAME", "SIZE_MISMATCH")
           or r.get("error")]
    print(f"\n{len(results) - len(bad)}/{len(results)} worlds MATCH")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
