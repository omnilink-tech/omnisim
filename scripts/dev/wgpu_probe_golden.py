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

"""R3.6 + R3.6b golden-image regression harness for the wgpu render path.

Two capture modes:

  Probe mode (default): runs the OmniSim binary with
  OMNISIM_PROBE_WGPU=N and OMNISIM_WGPU_PROBE_DIR pointing at a
  scratch directory; the in-binary probes write one .ppm per render
  path that fired (clearAndRead, triangle, mesh, mesh+MVP, textured,
  instanced, adapter-mesh).

  World mode (--world): boots a real .wbt through headless_runner +
  the Camera-side wgpu route (R3.3b). The camera_wgpu_smoke
  controller dumps the Camera readback as PPM when OMNISIM_R33B_PPM_PATH
  is set. This is the path that exercises actual world-load through
  to wgpu Camera rendering, not just the standalone probe RTTs.

Two action modes (orthogonal to capture mode):

    --update    Write the captured .ppm files to docs/developer/wgpu-golden/
                as the new reference set. Run after a verified-good
                wgpu rendering change.
    --check     (default) Compare the captured .ppm files against the
                reference set under docs/developer/wgpu-golden/; exit
                non-zero if any frame's mean absolute pixel diff is
                > the threshold (default 2% per channel = 5/255).
"""

import argparse
import os
import shutil
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
OMNISIM_BIN = REPO_ROOT / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe"
GOLDEN_DIR = REPO_ROOT / "docs" / "developer" / "wgpu-golden"
DEFAULT_THRESHOLD_PER_CHANNEL = 5  # out of 255 ~ 2%


def read_ppm(path: Path) -> tuple[int, int, bytes]:
    """Read a binary P6 ppm; return (width, height, rgb_bytes)."""
    with path.open("rb") as f:
        magic = f.readline().strip()
        if magic != b"P6":
            raise ValueError(f"{path}: not a P6 ppm (got {magic!r})")
        # Skip comment lines, parse width/height
        while True:
            line = f.readline()
            if not line.startswith(b"#"):
                break
        w, h = (int(x) for x in line.split())
        maxval = int(f.readline().strip())
        if maxval != 255:
            raise ValueError(f"{path}: only 8-bit ppm supported (got max {maxval})")
        rgb = f.read(w * h * 3)
        return w, h, rgb


def diff_ppm(reference: Path, actual: Path, threshold: int) -> tuple[bool, float]:
    """Return (within_threshold, mean_abs_diff_per_channel)."""
    wr, hr, ref = read_ppm(reference)
    wa, ha, act = read_ppm(actual)
    if (wr, hr) != (wa, ha):
        return False, float("inf")
    total = 0
    for r, a in zip(ref, act):
        d = r - a
        total += d if d >= 0 else -d
    mean = total / len(ref)
    return mean <= threshold, mean


def run_probe(probe_level: int, out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["OMNISIM_PROBE_WGPU"] = str(probe_level)
    env["OMNISIM_WGPU_PROBE_DIR"] = str(out_dir)
    # --help is the cheapest way to invoke the binary: it triggers the
    # probe (which fires before OmApplication actually does anything),
    # then exits cleanly.
    result = subprocess.run(
        [str(OMNISIM_BIN), "--help"],
        env=env,
        capture_output=True,
        timeout=30,
        cwd=str(REPO_ROOT),
    )
    if result.returncode != 0:
        sys.stderr.write(f"omnisim-bin exited with {result.returncode}\n")
        sys.stderr.write(result.stderr.decode("utf-8", "replace"))
        sys.exit(2)


def run_world(world: Path, ppm_name: str, out_dir: Path, duration: int = 4,
              attempts: int = 4) -> None:
    """Boot a .wbt headless and capture the wgpu-Camera readback as PPM.

    The world must declare a Camera with `renderBackend "wgpu"` and use
    the `camera_wgpu_smoke` controller (or compatible). The controller
    writes its PPM dump to OMNISIM_R33B_PPM_PATH; we point it at
    `out_dir/<ppm_name>`.

    Retries up to `attempts` times for the known flaky first-load crash
    (a camera world's first headless launch intermittently dies in a Qt
    thread-destroy before the controller dumps the PPM; a relaunch
    succeeds). Without this, --world mode fails spuriously and can't be a
    standing CI gate. A capture succeeds the moment the PPM file appears.
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    ppm_path = out_dir / ppm_name
    runner = REPO_ROOT / "scripts" / "dev" / "headless_runner.py"
    env = os.environ.copy()
    env["OMNISIM_R33B_PPM_PATH"] = str(ppm_path)
    # Use a per-run result txt path so concurrent invocations don't stomp.
    env["OMNISIM_R33B_RESULT_PATH"] = str(out_dir / f"{ppm_name}.result.txt")
    last_rc = None
    last_err = ""
    for attempt in range(1, attempts + 1):
        if ppm_path.exists():
            ppm_path.unlink()
        result = subprocess.run(
            ["py", "-3", str(runner), str(world), "--duration", str(duration)],
            env=env,
            capture_output=True,
            timeout=duration + 30,
            cwd=str(REPO_ROOT),
        )
        last_rc = result.returncode
        last_err = result.stderr.decode("utf-8", "replace")
        if ppm_path.exists():
            if attempt > 1:
                sys.stderr.write(f"[golden] {ppm_name} captured on attempt {attempt}\n")
            return
        sys.stderr.write(f"[golden] attempt {attempt}/{attempts}: no PPM "
                         f"(flaky first-load), retrying\n")
    sys.stderr.write(f"No PPM captured at {ppm_path} after {attempts} attempts "
                     f"(last rc={last_rc}). Does the world's controller dump to "
                     f"OMNISIM_R33B_PPM_PATH?\n{last_err}")
    sys.exit(2)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--update", action="store_true",
                   help="overwrite the reference set with captured frames")
    p.add_argument("--probe-level", type=int, default=10,
                   help="OMNISIM_PROBE_WGPU value (1-10, default 10 = all probes "
                        "including the R3.5b QImage adapter at probe=10)")
    p.add_argument("--world", type=str, default=None,
                   help="path to a .wbt to boot through headless_runner instead of "
                        "running the standalone probes (R3.6b). The world must use "
                        "the camera_wgpu_smoke controller (or compatible). "
                        "Saved as 'world_<basename>.ppm' under wgpu-golden/.")
    p.add_argument("--threshold", type=int, default=DEFAULT_THRESHOLD_PER_CHANNEL,
                   help="max mean absolute per-channel diff (0-255)")
    p.add_argument("--ref-name", type=str, default=None,
                   help="override the captured/compared filename (world mode only). "
                        "Use to capture a variant of the same world under a distinct "
                        "reference, e.g. an AgX-tonemapped frame "
                        "(OMNISIM_CAMERA_AGX=1 ... --ref-name world_camera_wgpu_golden_agx.ppm).")
    args = p.parse_args()

    if not OMNISIM_BIN.exists():
        sys.stderr.write(f"omnisim-bin not found at {OMNISIM_BIN}\n")
        return 2

    # World mode runs through a stable _scratch dir (not tempfile) —
    # the OS short-path mangling (8.3 `SOMEUS~1` aliases for long
    # usernames) inside the TEMP path was tripping up the controller
    # process's path resolution. Probe mode is fine with tempdir since the C++
    # probe writes via Qt's QFile which handles short paths cleanly.
    if args.world:
        scratch_dir = REPO_ROOT / "_scratch" / "wgpu_world_capture"
        scratch_dir.mkdir(parents=True, exist_ok=True)
        # Clean any stale frames from a prior --world run.
        for f in scratch_dir.glob("*.ppm"):
            f.unlink()
        for f in scratch_dir.glob("*.result.txt"):
            f.unlink()

        world = Path(args.world).resolve()
        if not world.exists():
            sys.stderr.write(f"world not found: {world}\n")
            return 2
        ppm_name = args.ref_name if args.ref_name else f"world_{world.stem}.ppm"
        # 12 s: the camera world needs load + a few controller steps + PPM dump;
        # 4 s was too tight and produced spurious "no PPM" even with retries.
        run_world(world, ppm_name, scratch_dir, duration=12)
        tmp_path = scratch_dir
    else:
        tmp = tempfile.mkdtemp(prefix="wgpu_probe_")
        tmp_path = Path(tmp)
        run_probe(args.probe_level, tmp_path)

    captured = sorted(tmp_path.glob("*.ppm"))
    if not captured:
        sys.stderr.write(f"No .ppm files captured in {tmp_path}; "
                          "is the dump path being read?\n")
        return 2

    if args.update:
        GOLDEN_DIR.mkdir(parents=True, exist_ok=True)
        for ppm in captured:
            shutil.copy(ppm, GOLDEN_DIR / ppm.name)
            print(f"[update] {ppm.name}")
        return 0

    if not GOLDEN_DIR.exists():
        sys.stderr.write(f"No reference set at {GOLDEN_DIR}. "
                          "Run with --update first.\n")
        return 2

    failures = []
    for ppm in captured:
        ref = GOLDEN_DIR / ppm.name
        if not ref.exists():
            print(f"[NEW] {ppm.name} has no reference; rerun with --update")
            failures.append(ppm.name)
            continue
        ok, mean = diff_ppm(ref, ppm, args.threshold)
        tag = "PASS" if ok else "FAIL"
        print(f"[{tag}] {ppm.name} mean-abs-diff = {mean:.2f} / 255 "
              f"(threshold {args.threshold})")
        if not ok:
            failures.append(ppm.name)

    if failures:
        sys.stderr.write(f"\n{len(failures)} regression(s): {failures}\n")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
