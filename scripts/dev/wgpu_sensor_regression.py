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

"""Regression guard for the wgpu sensor pipeline (engine-migration-plan.md R5).

Runs the wgpu camera + RangeFinder smoke worlds headless across every sensor
mode and asserts the known-good readback values, so the R5 family
(grayscale depth, R32Float real-meters depth, the WbRangeFinder device node,
the WbCamera->WbWgpuSceneRenderer migration, and the R5d Lidar radial-range
path) can't silently regress.

This is the repeatable form of the manual checks used to land each increment:

  camera_wgpu_scene_smoke (64x48, cyan box front face 0.7 m away, far 10):
    DEPTH unset  lit       -> center BGRA (0,141,141)         [gamma / lit path]
    DEPTH=1      grayscale -> center BGRA (18,18,18)          [R5]
    DEPTH=2      F32 depth -> log center=0.7000 m             [R5b]
    DEPTH=3      radial    -> log center=0.7000 corner=0.7887 [R5d]
  rangefinder_wgpu_smoke (OMNISIM_RANGEFINDER_WGPU=1):
    getRangeImage center   -> 0.7000 m                        [R5c device node]
  lidar_wgpu_smoke (OMNISIM_LIDAR_WGPU=1, single-layer narrow-FOV):
    getRangeImage profile  -> range[theta] = 0.70/cos(theta)  [R5e device node]
                              center 0.7000 m, monotonic off-axis
  lidar_wgpu_multilayer_smoke (OMNISIM_LIDAR_WGPU=1, 7-layer asymmetric):
    centre layer profile + above>below orientation              [R5f multi-layer]
  lidar_wgpu_widefov_smoke (OMNISIM_LIDAR_WGPU=1, fov 2.0 = 2 frustums):
    profile across seam + seam continuity + empty flanks        [R5g wide-FOV]
  lidar_wgpu_azimuth_smoke (OMNISIM_LIDAR_WGPU=1, box offset +y):
    box hits only +theta columns (no left-right mirror)         [R5g azimuth]
  lidar_wgpu_ml_widefov_smoke (OMNISIM_LIDAR_WGPU=1, 7-layer + fov 2.0):
    centre-layer seam profile + above>below orientation         [R5h ml+wide]
  lidar_wgpu_tilt_smoke (OMNISIM_LIDAR_WGPU=1, tiltAngle 0.3):
    centre = 0.70/cos(tilt) (scan pitched up, correct sign)     [R5i tilt]

Needs a Vulkan-capable GPU + the wgpu build (OMNISIM_WITH_VULKAN=ON, wgpu_native
on PATH). Run from PowerShell on Windows (see engine-migration-plan.md notes on
the embedded-interpreter env). Retries the known flaky first-load crash.

Usage:
    python scripts/dev/wgpu_sensor_regression.py [--attempts 4] [--verbose]

Exit code 0 = all cases pass; 1 = a mismatch (regression) or a case never ran.
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import tempfile
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
WORLDS = REPO_ROOT / "projects" / "samples" / "demos" / "worlds" / "rendering"
CAM_WORLD = WORLDS / "camera_wgpu_scene_smoke.wbt"
EMIS_WORLD = WORLDS / "camera_wgpu_emissive_smoke.wbt"
SPEC_WORLD = WORLDS / "camera_wgpu_specular_smoke.wbt"
SPEC_ROUGH_WORLD = WORLDS / "camera_wgpu_specular_rough_smoke.wbt"
RF_WORLD = WORLDS / "rangefinder_wgpu_smoke.wbt"
LIDAR_WORLD = WORLDS / "lidar_wgpu_smoke.wbt"
LIDAR_ML_WORLD = WORLDS / "lidar_wgpu_multilayer_smoke.wbt"
LIDAR_WF_WORLD = WORLDS / "lidar_wgpu_widefov_smoke.wbt"
LIDAR_AZ_WORLD = WORLDS / "lidar_wgpu_azimuth_smoke.wbt"
LIDAR_MLWF_WORLD = WORLDS / "lidar_wgpu_ml_widefov_smoke.wbt"
LIDAR_TILT_WORLD = WORLDS / "lidar_wgpu_tilt_smoke.wbt"
LIDAR_ROT_WORLD = WORLDS / "lidar_wgpu_rotating_smoke.wbt"
LIDAR_ML_ROT_WORLD = WORLDS / "lidar_wgpu_ml_rotating_smoke.wbt"
LIDAR_WIDE_ROT_WORLD = WORLDS / "lidar_wgpu_wide_rotating_smoke.wbt"


def find_binary() -> Path:
    home = Path(os.environ.get("OMNISIM_HOME") or os.environ.get("WEBOTS_HOME") or REPO_ROOT)
    cand = home / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe"
    if cand.exists():
        return cand
    raise SystemExit(f"omnisim-bin.exe not found under {home} (set OMNISIM_HOME to the checkout)")


def run_world(binary: Path, world: Path, extra_env: dict, attempts: int, verbose: bool):
    """Run a world headless; return (result_text, log_text) or (None, log_text).

    Retries the flaky first-load crash up to `attempts` times (a successful run
    writes the controller result file)."""
    home = Path(os.environ.get("OMNISIM_HOME") or REPO_ROOT)
    tmp = Path(tempfile.gettempdir())
    res_path = tmp / f"wgpu_reg_result_{os.getpid()}.txt"
    log_path = tmp / f"wgpu_reg_log_{os.getpid()}.txt"
    cmd = [str(binary), str(world), "--minimize", "--batch", "--no-rendering",
           "--mode=fast", "--stdout", "--stderr"]
    for attempt in range(1, attempts + 1):
        for p in (res_path, log_path):
            try:
                p.unlink()
            except FileNotFoundError:
                pass
        env = os.environ.copy()
        env["OMNISIM_HOME"] = str(home)
        env["OMNISIM_LOG_PATH"] = str(log_path)
        env["OMNISIM_R33B_RESULT_PATH"] = str(res_path)
        mingw = home / "msys64" / "mingw64" / "bin"
        env["PATH"] = f"{mingw};{binary.parent};" + env.get("PATH", "")
        env.update(extra_env)
        # Valid std handles (a real file, not DEVNULL): on Windows, DEVNULL can hand the spawned
        # controller invalid std handles -> flaky "no result" first launch. Same root cause + fix as
        # physics_oracle.py (default-flip-plan.md §3.5). The retry loop now covers only residual flake.
        proc_out = open(str(log_path) + ".proc", "w", encoding="utf-8", errors="replace")
        proc = subprocess.Popen(cmd, env=env, stdout=proc_out, stderr=subprocess.STDOUT)
        start = time.time()
        while time.time() - start < 14:
            if res_path.exists():
                break
            if proc.poll() is not None and not res_path.exists():
                break
            time.sleep(0.4)
        try:
            proc.terminate()
            proc.wait(timeout=8)
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
        if res_path.exists():
            return res_path.read_text(errors="replace"), log_text
        if verbose:
            print(f"    attempt {attempt}: no result (flaky load), retrying")
    return None, (log_path.read_text(errors="replace") if log_path.exists() else "")


def approx(a: float, b: float, tol: float = 0.01) -> bool:
    return abs(a - b) <= tol


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--attempts", type=int, default=4, help="retries per case for the flaky load")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    binary = find_binary()
    for w in (CAM_WORLD, EMIS_WORLD, SPEC_WORLD, SPEC_ROUGH_WORLD, RF_WORLD, LIDAR_WORLD,
              LIDAR_ML_WORLD, LIDAR_WF_WORLD, LIDAR_AZ_WORLD, LIDAR_MLWF_WORLD, LIDAR_TILT_WORLD):
        if not w.exists():
            raise SystemExit(f"world not found: {w}")
    print(f"[wgpu-reg] binary: {binary}")

    failures: list[str] = []

    def check(name: str, ok: bool, detail: str):
        tag = "PASS" if ok else "FAIL"
        print(f"[wgpu-reg] {tag}  {name}: {detail}")
        if not ok:
            failures.append(f"{name}: {detail}")

    # --- Camera cases -------------------------------------------------------
    # (label, env, result-file checker, log checker) ; checkers return (ok, detail)
    def cam_center_bgra(res: str):
        m = re.search(r"BGRA=\((\d+),(\d+),(\d+)", res)
        return (m.group(1), m.group(2), m.group(3)) if m else None

    # lit / gamma path
    res, log = run_world(binary, CAM_WORLD, {}, args.attempts, args.verbose)
    if res is None:
        check("camera/lit", False, "no result after retries")
    else:
        bgra = cam_center_bgra(res)
        check("camera/lit (gamma)", bgra == ("0", "141", "141"),
              f"center BGRA={bgra} (expect 0,141,141)")

    # DEPTH=1 grayscale (R5)
    res, log = run_world(binary, CAM_WORLD, {"OMNISIM_CAMERA_DEPTH": "1"}, args.attempts, args.verbose)
    if res is None:
        check("camera/depth1", False, "no result after retries")
    else:
        bgra = cam_center_bgra(res)
        check("camera/depth1 (R5 grayscale)", bgra == ("18", "18", "18"),
              f"center BGRA={bgra} (expect 18,18,18)")

    # DEPTH=2 F32 real-meters (R5b)
    res, log = run_world(binary, CAM_WORLD, {"OMNISIM_CAMERA_DEPTH": "2"}, args.attempts, args.verbose)
    m = re.search(r"depth-F32 real-meters: center=([0-9.]+) m", log)
    if res is None or not m:
        check("camera/depth2 (R5b F32)", False, f"no F32 log line (res={'ok' if res else 'none'})")
    else:
        check("camera/depth2 (R5b F32)", approx(float(m.group(1)), 0.70),
              f"center={m.group(1)} m (expect ~0.7000)")

    # DEPTH=3 radial range (R5d): center == planar, corner > center
    res, log = run_world(binary, CAM_WORLD, {"OMNISIM_CAMERA_DEPTH": "3"}, args.attempts, args.verbose)
    m = re.search(r"radial-range: center=([0-9.]+) m corner=([0-9.]+) m", log)
    if res is None or not m:
        check("camera/depth3 (R5d radial)", False, f"no radial-range log line (res={'ok' if res else 'none'})")
    else:
        center, corner = float(m.group(1)), float(m.group(2))
        ok = approx(center, 0.70) and approx(corner, 0.7887, 0.02) and corner > center
        check("camera/depth3 (R5d radial)", ok,
              f"center={center} corner={corner} (expect ~0.70 / ~0.7887, corner>center)")

    # LIGHTDEPTH=1 light-space clip-depth (T1.2 CSM shadow-map half): the box
    # rendered from a fixed overhead ortho light reads NDC depth (clip.z),
    # ≈ (4.7-0.05)/9.95 = 0.4673 at the centre. Locks the ortho light viewProj +
    # clip.z pipeline + the clipDepth pipeline-selection fix. Default off → the
    # lit/depth/range paths are byte-unchanged (covered by camera/lit + golden).
    res, log = run_world(binary, CAM_WORLD, {"OMNISIM_CAMERA_LIGHTDEPTH": "1"}, args.attempts, args.verbose)
    m = re.search(r"CSM light-space clip-depth: center=([0-9.]+)", log)
    if res is None or not m:
        check("camera lightdepth (T1.2 CSM)", False,
              f"no CSM light-depth log line (res={'ok' if res else 'none'})")
    else:
        check("camera lightdepth (T1.2 CSM)", approx(float(m.group(1)), 0.4673, 0.01),
              f"center={m.group(1)} (expect ~0.4673 = (4.7-0.05)/9.95)")

    # --- RangeFinder device node (R5c) -------------------------------------
    res, log = run_world(binary, RF_WORLD, {"OMNISIM_RANGEFINDER_WGPU": "1"}, args.attempts, args.verbose)
    m = re.search(r"center\(\d+,\d+\)=([0-9.]+) m", res or "")
    if res is None or not m:
        check("rangefinder (R5c device)", False, f"no center range (res={'ok' if res else 'none'})")
    else:
        check("rangefinder (R5c device)", approx(float(m.group(1)), 0.70),
              f"center={m.group(1)} m (expect ~0.7000)")

    # --- T1.1 AgX emissive HDR source --------------------------------------
    # camera_wgpu_emissive_smoke: black baseColor + emissiveColor 0 0 1 ×
    # emissiveIntensity 4 (effective HDR blue 4.0). Two assertions pin the
    # emissive→pad1→AgX chain end to end:
    #   AgX OFF → centre (0,0,0): the plain lit shader ignores emissive, so a
    #     black-base box is black — proves non-AgX paths never read pad1.
    #   AgX ON  → centre RGB ≈ (248,144,144): the 4.0 blue is AgX-compressed
    #     (B≈248, not clipped) AND desaturated toward white (R,G lift 0→~144).
    #     A regression in harvest/pack/shader would drop R,G back to 0 (no
    #     desaturation) or B to 0 (emissive lost).
    res_off, _ = run_world(binary, EMIS_WORLD, {"OMNISIM_CAMERA_AGX": "0"}, args.attempts, args.verbose)
    bgra_off = cam_center_bgra(res_off or "")
    check("camera emissive AgX-off (zero-break)", bgra_off == ("0", "0", "0"),
          f"center BGRA={bgra_off} (expect 0,0,0 — plain shader ignores emissive)")

    res_on, _ = run_world(binary, EMIS_WORLD, {"OMNISIM_CAMERA_AGX": "1"}, args.attempts, args.verbose)
    bgra_on = cam_center_bgra(res_on or "")
    if bgra_on is None:
        check("camera emissive AgX-on (T1.1 HDR)", False, "no result after retries")
    else:
        # cam_center_bgra returns the three stored channel bytes in buffer order
        # (c0,c1,c2); for the pure-blue emissive the blue channel reads back as
        # the THIRD byte. AgX-on ground truth = (144,144,248): the >1 blue is
        # compressed to c2≈248 (highest, not clipped) and the other two lift to
        # c0≈c1≈144 (desaturation toward white). Generous bands absorb sub-pixel
        # / driver variance; a regression in harvest/pack/shader collapses c0,c1
        # back toward 0 (no desaturation) or c2 toward 0 (emissive lost).
        c0, c1, c2 = int(bgra_on[0]), int(bgra_on[1]), int(bgra_on[2])
        ok = c2 > 200 and 90 < c0 < 200 and 90 < c1 < 200 and abs(c0 - c1) < 40
        check("camera emissive AgX-on (T1.1 HDR)", ok,
              f"center bytes=({c0},{c1},{c2}) (expect blue compressed c2>200 + "
              f"desaturated c0,c1~144)")

    # --- T1.1 AgX specular highlight ---------------------------------------
    # Grey 0.5 box, flat front face under the fixed light. specular smoke world =
    # roughness 0.85 (broad lobe); rough twin = roughness 1.0 (smoothness 0 →
    # specular branch skipped). Pins the smoothness→pad1.w + camPos→pad0.yzw +
    # worldPos→Blinn-Phong chain. Measured on the rebuilt binary (centre byte):
    #   AgX-off       → (71,71,71)    pure diffuse (plain shader ignores specular)
    #   AgX-on rough  → (146,146,146) AgX of diffuse-only
    #   AgX-on smooth → (175,175,175) the isolated +29 specular lift
    res_so, _ = run_world(binary, SPEC_WORLD, {"OMNISIM_CAMERA_AGX": "0"}, args.attempts, args.verbose)
    bgra_so = cam_center_bgra(res_so or "")
    check("camera specular AgX-off (zero-break)", bgra_so == ("71", "71", "71"),
          f"center BGRA={bgra_so} (expect 71,71,71 — plain shader ignores specular)")

    res_ss, _ = run_world(binary, SPEC_WORLD, {"OMNISIM_CAMERA_AGX": "1"}, args.attempts, args.verbose)
    res_sr, _ = run_world(binary, SPEC_ROUGH_WORLD, {"OMNISIM_CAMERA_AGX": "1"}, args.attempts, args.verbose)
    bgra_ss = cam_center_bgra(res_ss or "")
    bgra_sr = cam_center_bgra(res_sr or "")
    if bgra_ss is None or bgra_sr is None:
        check("camera specular AgX-on (T1.1 highlight)", False,
              f"no result (smooth={'ok' if bgra_ss else 'none'}, rough={'ok' if bgra_sr else 'none'})")
    else:
        smooth_v = int(bgra_ss[0])
        rough_v = int(bgra_sr[0])
        neutral = bgra_ss[0] == bgra_ss[1] == bgra_ss[2]
        ok = neutral and smooth_v > rough_v + 15
        check("camera specular AgX-on (T1.1 highlight)", ok,
              f"smooth={smooth_v} vs rough={rough_v} (expect smooth>rough+15, neutral grey)")

    # --- Lidar device node (R5e), single-layer narrow-FOV ------------------
    # The controller already asserts the full range[theta]=0.70/cos(theta)
    # profile + monotonicity and stamps PASS/OBSERVE; we gate on its verdict
    # and re-check the center here.
    res, log = run_world(binary, LIDAR_WORLD, {"OMNISIM_LIDAR_WGPU": "1"}, args.attempts, args.verbose)
    m = re.search(r"center\(\d+\)=([0-9.]+) m", res or "")
    passed = res is not None and res.startswith("PASS lidar-wgpu")
    if res is None or not m:
        check("lidar (R5e device)", False, f"no center range (res={'ok' if res else 'none'})")
    else:
        check("lidar (R5e device)", passed and approx(float(m.group(1)), 0.70),
              f"center={m.group(1)} m, controller verdict={'PASS' if passed else 'OBSERVE'} "
              f"(expect ~0.7000 + analytic profile pass)")

    # --- Lidar multi-layer (R5f), 7 layers, asymmetric elevation -----------
    # The controller asserts centre 0.70 + centre-layer profile + the
    # above>below orientation (vertical-flip detector); we gate on its verdict.
    res, log = run_world(binary, LIDAR_ML_WORLD, {"OMNISIM_LIDAR_WGPU": "1"}, args.attempts, args.verbose)
    m = re.search(r"center\(L\d+,\d+\)=([0-9.]+) m", res or "")
    passed = res is not None and res.startswith("PASS lidar-wgpu-multilayer")
    if res is None or not m:
        check("lidar multi-layer (R5f)", False, f"no center range (res={'ok' if res else 'none'})")
    else:
        check("lidar multi-layer (R5f)", passed and approx(float(m.group(1)), 0.70),
              f"center={m.group(1)} m, controller verdict={'PASS' if passed else 'OBSERVE'} "
              f"(expect ~0.7000 + profile + orientation pass)")

    # --- Lidar wide-FOV (R5g), 2-frustum stitch ----------------------------
    # The controller asserts centre 0.70 + profile across the seam + seam
    # continuity + empty flanks; we gate on its verdict.
    res, log = run_world(binary, LIDAR_WF_WORLD, {"OMNISIM_LIDAR_WGPU": "1"}, args.attempts, args.verbose)
    passed = res is not None and res.startswith("PASS lidar-wgpu-widefov")
    check("lidar wide-FOV (R5g)", passed,
          (res or "no result").strip() if not passed else "2-frustum stitch, seam-continuous")

    # --- Lidar azimuth orientation (R5g): +theta = left --------------------
    res, log = run_world(binary, LIDAR_AZ_WORLD, {"OMNISIM_LIDAR_WGPU": "1"}, args.attempts, args.verbose)
    passed = res is not None and res.startswith("PASS lidar-wgpu-azimuth")
    check("lidar azimuth (+theta=left)", passed,
          (res or "no result").strip() if not passed else "+y box -> +theta columns (no mirror)")

    # --- Lidar multi-layer + wide-FOV (R5h) --------------------------------
    res, log = run_world(binary, LIDAR_MLWF_WORLD, {"OMNISIM_LIDAR_WGPU": "1"}, args.attempts, args.verbose)
    passed = res is not None and res.startswith("PASS lidar-wgpu-ml-widefov")
    check("lidar multi-layer+wide (R5h)", passed,
          (res or "no result").strip() if not passed else "7-layer 2-frustum: seam profile + orientation")

    # --- Lidar tilt (R5i): scan pitched up by tiltAngle --------------------
    res, log = run_world(binary, LIDAR_TILT_WORLD, {"OMNISIM_LIDAR_WGPU": "1"}, args.attempts, args.verbose)
    passed = res is not None and res.startswith("PASS lidar-wgpu-tilt")
    check("lidar tilt (R5i)", passed,
          (res or "no result").strip() if not passed else "tilt 0.3 -> centre 0.70/cos(tilt)")

    # --- Lidar rotating head (R5j): stateful per-step yaw ------------------
    # The head yaws each step; the wgpu window is rendered at the previous
    # rotating angle so it lands at the same global column WREN produces. The
    # one-sided (right) box must land in the exact predicted column band
    # (anti-mirror + anti-offset) -- the crux a convention-blind test misses.
    res, log = run_world(binary, LIDAR_ROT_WORLD, {"OMNISIM_LIDAR_WGPU": "1"}, args.attempts, args.verbose)
    passed = res is not None and res.startswith("PASS lidar-wgpu-rotating")
    check("lidar rotating (R5j)", passed,
          (res or "no result").strip() if not passed else "exact-column placement == WREN oracle")

    # --- Lidar multi-layer rotating (R5k): azimuth + per-layer elevation -----
    # 7-layer rotating head, box front-RIGHT + offset UP. Asserts the centre
    # layer's box lands in the exact predicted column band (azimuth) AND more
    # layers above centre see it than below (elevation orientation) -- both the
    # R5j rotating stitch and the R5f vertical resample holding at once.
    res, log = run_world(binary, LIDAR_ML_ROT_WORLD, {"OMNISIM_LIDAR_WGPU": "1"}, args.attempts, args.verbose)
    passed = res is not None and res.startswith("PASS lidar-wgpu-ml-rotating")
    check("lidar multi-layer rotating (R5k)", passed,
          (res or "no result").strip() if not passed else "centre-layer exact column + above>below orientation")

    # --- Lidar wide-FOV rotating (R5l): multi-frustum + rotating window ------
    # Wide fov (2.0 -> 2 frustums) + 7 layers + rotating head, box front-RIGHT +
    # offset UP. Composes the R5g multi-frustum azimuth stitch with the rotating
    # window; same azimuth + elevation discriminants as R5k. The last Lidar
    # config to reach wgpu.
    res, log = run_world(binary, LIDAR_WIDE_ROT_WORLD, {"OMNISIM_LIDAR_WGPU": "1"}, args.attempts, args.verbose)
    passed = res is not None and res.startswith("PASS lidar-wgpu-wide-rotating")
    check("lidar wide-FOV rotating (R5l)", passed,
          (res or "no result").strip() if not passed else "multi-frustum stitch + rotating placement + orientation")

    print()
    if failures:
        print(f"[wgpu-reg] REGRESSION: {len(failures)} case(s) failed:")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("[wgpu-reg] ALL PASS -- wgpu sensor pipeline intact "
          "(R5 / R5b / R5c / R5d / R5e / R5f / R5g / R5h / R5i / R5j / R5k / R5l)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
