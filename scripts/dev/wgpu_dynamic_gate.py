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
"""wgpu_dynamic_gate.py -- the P1 / P9 / P2 / P11 gate of docs/developer/wren-deletion-runbook.md.

Asks whether a wgpu-rendered Camera SEES what WREN shows it, and -- for anything that
moves -- whether it sees it MOVE. Five cases today: the node types that are not Solids
(Cloth/SoftBody = P1, GranularGroup = P9, Track + Muscle = P2), plus the one material
path a Solid could lose (a legacy `Appearance`'s texture = P11).

WHY BOTH QUESTIONS. collectWorldDraws walks Solids; those three node types hang off
OmBaseNode and each needs its own collect. The deformables got one for the main view in
`d5897ff7d` and it had exactly ONE caller in the tree, so every wgpu Camera / RangeFinder
/ Lidar saw nothing while the screen looked right. The obvious fix -- call the same
collector from the sensors -- then walks into a SECOND, quieter defect: the collector
decided "do I need to re-upload?" from a function-local static keyed on the simulation
clock, and each device owns its OWN mesh cache. Whichever renderer ran first in a step
consumed the clock edge; the rest took the first-upload branch once and never updated.
The result animates correctly on screen and is FROZEN in the sensor image, and it passes
any single-screenshot test. So this script's third assertion -- the image changes across
well-separated frames -- is the one that matters, and it is why the gate is a script
rather than an eyeball.

THREE ARMS, one binary, all through value-parsed hatches:
  red        OMNISIM_WGPU_SENSOR_DYNAMIC=0 -> the gate world's camera image must equal the
             control world's (same scene, dynamic node removed). Proves the instrument can
             go red, on the same build that goes green.
  green      default -> the two must differ.
  animation  default -> two samples from the SAME run, at well-separated steps, must differ.

Usage
-----
  python scripts/dev/wgpu_dynamic_gate.py                          # every case
  python scripts/dev/wgpu_dynamic_gate.py --case cloth             # P1 only
  python scripts/dev/wgpu_dynamic_gate.py --case granular          # P9 only
  python scripts/dev/wgpu_dynamic_gate.py --case track             # P2, instanced belt
  python scripts/dev/wgpu_dynamic_gate.py --case muscle            # P2, procedural spheroid
  python scripts/dev/wgpu_dynamic_gate.py --case legacy_texture    # P11
  python scripts/dev/wgpu_dynamic_gate.py --json out.json

Each case names its own exact-revert hatch (`red_env`), so RED and GREEN are two runs of
ONE binary. A case whose scene is static sets `animate: False`; a case with an extra
material assertion sets `assert_chart`.

⚠ Wrap the invocation in scripts/dev/thermal_guard.py on the owner's laptop; this script
runs engines one at a time but does not police temperature itself.

⚠ The granular case needs CUDA. Without it GranularGroup is inert and the wgpu path draws
nothing ON PURPOSE (OmGranularGroup::wgpuParticles declines rather than reproducing WREN's
pile of spheres at the origin), so the case reports INCONCLUSIVE, never FAIL.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
WORLDS = REPO / "projects" / "samples" / "demos" / "worlds" / "rendering"

CASES = {
    "cloth": {
        "world": WORLDS / "camera_cloth_wgpu_smoke.omniworld",
        "control": WORLDS / "camera_cloth_wgpu_control.omniworld",
        "device": "cloth_cam",
        "steps": "40,120,240",
        "needs_cuda": False,
        "runbook": "P1",
        "red_env": {"OMNISIM_WGPU_SENSOR_DYNAMIC": "0"},
    },
    "granular": {
        "world": WORLDS / "camera_granular_wgpu_smoke.omniworld",
        # No control WORLD: the exact-revert hatch IS the control arm, so the red run uses
        # OMNISIM_WGPU_GRANULAR=0 on the same file. (The cloth case keeps a control world
        # because its red arm has to answer "identical with and WITHOUT the cloth in frame".)
        "control": None,
        "device": "grain_cam",
        "steps": "20,60,150",
        "needs_cuda": True,
        "runbook": "P9",
        "red_env": {"OMNISIM_WGPU_GRANULAR": "0"},
    },
    # ---- P2: Muscle and Track, the two remaining per-step-varying node types ----------
    # Track is INSTANCED geometry (N copies of a real OmGeometry placed by belt-path model
    # matrices) and Muscle is PROCEDURAL (a spheroid re-synthesised every step from its own
    # height/radius), so they share nothing but the collect they hang off. Both get a
    # control world with the animated content removed, and both must ANIMATE -- a belt
    # frozen at element 0 and a muscle frozen at its rest bulge each pass a single
    # screenshot, which is exactly the class of defect the per-cache upload epoch exists
    # to prevent (see OmWgpuMeshCache::vertexEpochIs).
    "track": {
        "world": WORLDS / "camera_track_wgpu_smoke.omniworld",
        "control": WORLDS / "camera_track_wgpu_control.omniworld",
        "device": "track_cam",
        "steps": "40,160,320",
        "needs_cuda": False,
        "runbook": "P2",
        "red_env": {"OMNISIM_WGPU_TRACK": "0"},
    },
    "muscle": {
        "world": WORLDS / "camera_muscle_wgpu_smoke.omniworld",
        "control": WORLDS / "camera_muscle_wgpu_control.omniworld",
        "device": "muscle_cam",
        "steps": "40,160,320",
        "needs_cuda": False,
        "runbook": "P2",
        "red_env": {"OMNISIM_WGPU_MUSCLE": "0"},
    },
    # ---- P11: a legacy `Appearance`'s texture never reached the GPU -------------------
    # NOT a dynamic-content case -- nothing in this world moves -- but the same three-arm
    # instrument answers it exactly, and one re-runnable gate beats two. `animate` is False
    # because a static scene must NOT be asked to change across frames.
    "legacy_texture": {
        "world": WORLDS / "camera_legacy_texture_wgpu_smoke.omniworld",
        "control": WORLDS / "camera_legacy_texture_wgpu_control.omniworld",
        "device": "chart_cam",
        "steps": "20,40",
        "needs_cuda": False,
        "runbook": "P11",
        "red_env": {"OMNISIM_WGPU_LEGACY_TEXTURE": "0"},
        "animate": False,
        # WREN forces a TEXTURED Material's diffuseColor to white, so the chart must reach
        # the image at full saturation. The board's authored diffuseColor is a green
        # (0.25 0.65 0.30): a port that uploads the texture but skips the white-forcing
        # renders every chart patch green-tinted and cannot produce a strongly red-dominant
        # pixel. `hues` is the count of distinct 6-bit-quantised colours in the frame.
        "assert_chart": True,
    },
}


def find_binary() -> Path:
    home = Path(os.environ.get("OMNISIM_HOME") or REPO)
    for cand in (home / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe",
                 home / "bin" / "omnisim-bin",
                 home / "Contents" / "MacOS" / "omnisim"):
        if cand.exists():
            return cand
    raise SystemExit(f"omnisim-bin not found under {home}")


def read_ppm(path: Path):
    """Minimal binary-P6 reader -- no PIL dependency, the gate must run anywhere."""
    data = path.read_bytes()
    if not data.startswith(b"P6"):
        raise ValueError(f"{path} is not a binary PPM")
    fields, i = [], 2
    while len(fields) < 3:
        while i < len(data) and data[i] in b" \t\r\n":
            i += 1
        if i < len(data) and data[i:i + 1] == b"#":
            while i < len(data) and data[i] not in b"\r\n":
                i += 1
            continue
        j = i
        while j < len(data) and data[j] not in b" \t\r\n":
            j += 1
        fields.append(int(data[i:j]))
        i = j
    i += 1  # single whitespace after maxval
    w, h, _maxval = fields
    return w, h, data[i:i + w * h * 3]


def diff(a: Path, b: Path, threshold: int = 12):
    """(pixels over threshold, mean abs channel delta, max abs channel delta)."""
    wa, ha, pa = read_ppm(a)
    wb, hb, pb = read_ppm(b)
    if (wa, ha) != (wb, hb):
        raise ValueError(f"size mismatch {wa}x{ha} vs {wb}x{hb}")
    over = 0
    total = 0
    worst = 0
    for i in range(wa * ha):
        dr = abs(pa[i * 3] - pb[i * 3])
        dg = abs(pa[i * 3 + 1] - pb[i * 3 + 1])
        db = abs(pa[i * 3 + 2] - pb[i * 3 + 2])
        total += dr + dg + db
        m = max(dr, dg, db)
        if m > worst:
            worst = m
        if m > threshold:
            over += 1
    return over, total / float(wa * ha * 3), worst


def chart_stats(path: Path):
    """(distinct 6-bit-quantised colours, best red-dominance margin) of one PPM.

    The P11 assertion in two numbers. `hues` separates "a picture reached the GPU" from
    "one flat lit colour did"; `red_margin` is max(r - max(g, b)) over the frame, which a
    green-TINTED chart cannot make large -- so it catches a port that uploads the texture
    but forgets WREN's force-diffuseColor-to-white rule for a textured Material.
    """
    w, h, px = read_ppm(path)
    seen = set()
    red_margin = -255
    for i in range(w * h):
        r, g, b = px[i * 3], px[i * 3 + 1], px[i * 3 + 2]
        seen.add((r >> 2, g >> 2, b >> 2))
        m = r - max(g, b)
        if m > red_margin:
            red_margin = m
    return len(seen), red_margin


def run(binary: Path, world: Path, out_dir: Path, device: str, steps: str,
        extra_env: dict, duration: float, attempts: int, verbose: bool):
    """One engine, headless, sampling `steps`. Returns (samples, log_text)."""
    home = Path(os.environ.get("OMNISIM_HOME") or REPO)
    tmp = Path(tempfile.gettempdir())
    log_path = tmp / f"wgpu_dyn_gate_{os.getpid()}_{out_dir.name}.txt"
    samples_txt = out_dir / "samples.txt"
    for attempt in range(1, attempts + 1):
        if out_dir.exists():
            for f in out_dir.iterdir():
                f.unlink()
        out_dir.mkdir(parents=True, exist_ok=True)
        log_path.unlink(missing_ok=True)
        env = os.environ.copy()
        env["OMNISIM_HOME"] = str(home)
        env["OMNISIM_LOG_PATH"] = str(log_path)
        env["OMNISIM_CAM_SAMPLE_DIR"] = str(out_dir)
        env["OMNISIM_CAM_SAMPLE_DEVICE"] = device
        env["OMNISIM_CAM_SAMPLE_STEPS"] = steps
        mingw = home / "msys64" / "mingw64" / "bin"
        env["PATH"] = f"{mingw};{binary.parent};" + env.get("PATH", "")
        env.update(extra_env)
        cmd = [str(binary), str(world), "--minimize", "--batch", "--no-rendering",
               "--mode=fast", "--stdout", "--stderr"]
        # A real file, not DEVNULL: on Windows DEVNULL hands the spawned controller invalid
        # std handles and the first launch flakes (same root cause as physics_oracle.py).
        proc_out = open(str(log_path) + ".proc", "w", encoding="utf-8", errors="replace")
        proc = subprocess.Popen(cmd, env=env, stdout=proc_out, stderr=subprocess.STDOUT)
        start = time.time()
        want = "done last_sample_step"
        while time.time() - start < duration:
            if samples_txt.exists() and want in samples_txt.read_text(errors="replace"):
                break
            if proc.poll() is not None:
                break
            time.sleep(0.4)
        try:
            proc.terminate()
            proc.wait(timeout=10)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass
        try:
            proc_out.close()
        except Exception:
            pass
        log_text = log_path.read_text(errors="replace") if log_path.exists() else ""
        text = samples_txt.read_text(errors="replace") if samples_txt.exists() else ""
        samples = {}
        for line in text.splitlines():
            if line.startswith("step="):
                parts = dict(p.split("=", 1) for p in line.split() if "=" in p)
                samples[int(parts["step"])] = parts
        if samples:
            return samples, log_text
        if verbose:
            print(f"    attempt {attempt}: no samples, retrying")
    return {}, (log_path.read_text(errors="replace") if log_path.exists() else "")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--case", choices=sorted(CASES) + ["all"], default="all")
    ap.add_argument("--attempts", type=int, default=3)
    ap.add_argument("--duration", type=float, default=90.0, help="per-run wall-clock ceiling")
    ap.add_argument("--threshold", type=int, default=12, help="per-channel diff threshold")
    ap.add_argument("--json", type=Path)
    ap.add_argument("--keep", type=Path, default=REPO / "_scratch" / "wgpu_dynamic_gate")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    binary = find_binary()
    print(f"[gate] binary: {binary}")
    names = sorted(CASES) if args.case == "all" else [args.case]
    results = {}
    failures = []

    for name in names:
        c = CASES[name]
        print(f"\n[gate] ==== {name} ({c['runbook']}) ====")
        for w in (c["world"], c["control"]):
            if w is not None and not w.exists():
                raise SystemExit(f"world not found: {w}")
        base = Path(args.keep) / name
        res = {"runbook": c["runbook"], "world": str(c["world"])}

        # --- GREEN arm: default build, the world with the dynamic node --------------
        green, glog = run(binary, c["world"], base / "green", c["device"], c["steps"],
                          {}, args.duration, args.attempts, args.verbose)
        res["green_samples"] = {str(k): v for k, v in green.items()}
        if not green:
            failures.append(f"{name}: GREEN run produced no samples")
            print(f"[gate] FAIL {name}: green run produced no samples")
            results[name] = res
            continue
        if c["needs_cuda"] and "CUDA initialized" not in glog:
            print(f"[gate] INCONCLUSIVE {name}: no CUDA on this box; "
                  f"GranularGroup is inert and the wgpu path declines to draw by design")
            res["verdict"] = "inconclusive_no_cuda"
            results[name] = res
            continue

        steps = sorted(green)
        # --- ANIMATION: two samples from ONE run, well separated --------------------
        # Opt-out (`animate: False`) for a case whose scene is deliberately STATIC -- asking
        # a still life to change across frames would be an assertion that must fail.
        if c.get("animate", True):
            a, b = base / "green" / green[steps[0]]["ppm"], base / "green" / green[steps[-1]]["ppm"]
            over, mean, worst = diff(a, b, args.threshold)
            res["animation"] = {"a_step": steps[0], "b_step": steps[-1],
                                "px_over": over, "mean_abs": mean, "max_abs": worst}
            ok = over > 0
            print(f"[gate] {'PASS' if ok else 'FAIL'} {name} ANIMATION: step {steps[0]} vs "
                  f"{steps[-1]} -> {over} px over {args.threshold}, mean {mean:.3f}, max {worst}")
            if not ok:
                failures.append(f"{name}: the sensor image does not change across steps "
                                f"(frozen at first upload -- the per-cache epoch bug)")

        # --- RED arm + the visibility comparison ------------------------------------
        red_env = c.get("red_env", {})
        if c["control"] is not None:
            # A control WORLD: red = the case's own exact-revert hatch on BOTH worlds, and
            # the two must then agree -- i.e. the content this case is about contributes
            # exactly nothing on that arm.
            ctrl_red, _ = run(binary, c["control"], base / "control_red", c["device"], c["steps"],
                              red_env, args.duration,
                              args.attempts, args.verbose)
            gate_red, _ = run(binary, c["world"], base / "gate_red", c["device"], c["steps"],
                              red_env, args.duration,
                              args.attempts, args.verbose)
            ctrl_green, _ = run(binary, c["control"], base / "control_green", c["device"],
                                c["steps"], {}, args.duration, args.attempts, args.verbose)
            if not (ctrl_red and gate_red and ctrl_green):
                failures.append(f"{name}: a control/red run produced no samples")
                results[name] = res
                continue
            s = steps[-1]
            r_over, r_mean, r_worst = diff(base / "gate_red" / gate_red[s]["ppm"],
                                           base / "control_red" / ctrl_red[s]["ppm"], args.threshold)
            g_over, g_mean, g_worst = diff(base / "green" / green[s]["ppm"],
                                           base / "control_green" / ctrl_green[s]["ppm"],
                                           args.threshold)
            res["red"] = {"px_over": r_over, "mean_abs": r_mean, "max_abs": r_worst}
            res["green"] = {"px_over": g_over, "mean_abs": g_mean, "max_abs": g_worst}
            red_ok = r_over == 0
            green_ok = g_over > r_over and g_over > 0
            hatch = ",".join(sorted(red_env)) or "(none)"
            print(f"[gate] {'PASS' if red_ok else 'FAIL'} {name} RED ({hatch}=0, "
                  f"gate-vs-control): {r_over} px over threshold, max {r_worst}")
            print(f"[gate] {'PASS' if green_ok else 'FAIL'} {name} GREEN (default, "
                  f"gate-vs-control): {g_over} px over threshold, max {g_worst}")
            if not red_ok:
                failures.append(f"{name}: with the hatch OFF the sensor still differs "
                                f"({r_over} px) -- the red arm is not a revert")
            if not green_ok:
                failures.append(f"{name}: with the hatch ON the sensor does not see the "
                                f"content this case is about")

            # --- CHART: the picture reached the image, at full saturation ------------
            # ⚠ BOTH numbers are RELATIVE to the control world, and the absolute form of
            # the first one was MEASURED to be worthless: the pre-P11 image -- a flat
            # green board under a gradient sky -- already carries 38 distinct 6-bit hues,
            # so an absolute "hues >= 12" PASSED on a build where the texture provably
            # contributed 0 pixels (2026-08-22, machine 9722d23d12a3). Only the difference
            # against the same scene with the `texture` field removed says anything.
            if c.get("assert_chart"):
                hues, red = chart_stats(base / "green" / green[s]["ppm"])
                chues, cred = chart_stats(base / "control_green" / ctrl_green[s]["ppm"])
                res["chart"] = {"hues": hues, "control_hues": chues,
                                "red_margin": red, "control_red_margin": cred}
                hue_ok = hues >= chues + 8
                # WREN forces a TEXTURED Material's diffuseColor to white
                # (OmMaterial.cpp:151-152), so the chart's red patches must arrive
                # red-DOMINANT. A port that uploads the texture but skips the forcing
                # multiplies every patch by the board's authored green and cannot.
                red_ok = red >= cred + 40
                print(f"[gate] {'PASS' if hue_ok else 'FAIL'} {name} CHART hues: {hues} vs "
                      f"{chues} on the untextured control (need >= +8)")
                print(f"[gate] {'PASS' if red_ok else 'FAIL'} {name} CHART white-forcing: "
                      f"red_margin {red} vs {cred} on the control (need >= +40)")
                if not hue_ok:
                    failures.append(f"{name}: the textured image carries {hues} distinct "
                                    f"colours against {chues} untextured -- the texture is "
                                    f"not reaching the GPU")
                if not red_ok:
                    failures.append(f"{name}: no strongly red-dominant pixel (margin {red} "
                                    f"vs {cred}) -- the chart is tinted by diffuseColor, "
                                    f"i.e. WREN's force-diffuse-to-white rule for a "
                                    f"textured Material was not reproduced")
        else:
            # Granular: red = the node's own hatch, same world. The image must lose pixels.
            red, _ = run(binary, c["world"], base / "red", c["device"], c["steps"],
                         red_env, args.duration, args.attempts,
                         args.verbose)
            if not red:
                failures.append(f"{name}: RED run produced no samples")
                results[name] = res
                continue
            s = steps[-1]
            over2, mean2, worst2 = diff(base / "green" / green[s]["ppm"],
                                        base / "red" / red[s]["ppm"], args.threshold)
            res["red_vs_green"] = {"px_over": over2, "mean_abs": mean2, "max_abs": worst2}
            ok2 = over2 > 0
            print(f"[gate] {'PASS' if ok2 else 'FAIL'} {name} RED/GREEN (OMNISIM_WGPU_GRANULAR): "
                  f"{over2} px over threshold, mean {mean2:.3f}, max {worst2}")
            if not ok2:
                failures.append(f"{name}: OMNISIM_WGPU_GRANULAR makes no difference to the "
                                f"sensor image -- particles are not reaching it")
        results[name] = res

    if args.json:
        args.json.write_text(json.dumps(results, indent=2), encoding="utf-8")
        print(f"\n[gate] wrote {args.json}")
    print("\n[gate] " + ("ALL PASS" if not failures else f"{len(failures)} FAILURE(S)"))
    for f in failures:
        print(f"[gate]   - {f}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
