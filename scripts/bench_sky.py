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

"""Headless multi-shot sky bench.

For each shot in the shotlist, generates a temporary world file with
the sun direction (and optional exposure/colour/intensity/preset)
baked into the .wbt, swaps the `sun_control` supervisor to
`sky_render`, launches OmniSim, and waits for the one-shot screenshot
controller to exit.

We bake one world per shot rather than mutating at runtime because
WbBackground.applySkyBoxToWren isn't safe to call twice in the same
session (the IBL cubemap goes green on re-bake).

Usage:
    python scripts/bench_sky.py <source.wbt> <shotlist.json> <output_dir>
"""

from __future__ import annotations

import json
import math
import os
import re
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
_BIN_DIR = REPO_ROOT / "msys64" / "mingw64" / "bin"
# Prefer the canonical omnisim-bin.exe; fall back to the legacy webots.exe alias.
WEBOTS_EXE = next(
    (p for p in (_BIN_DIR / "omnisim-bin.exe", _BIN_DIR / "webots.exe") if p.exists()),
    _BIN_DIR / "omnisim-bin.exe",
)


def direction_from_azel(az_deg: float, el_deg: float) -> tuple:
    az = math.radians(az_deg)
    el = math.radians(el_deg)
    tx = math.sin(az) * math.cos(el)
    ty = math.cos(az) * math.cos(el)
    tz = math.sin(el)
    return (-tx, -ty, -tz)


def patch_world(src_text: str, shot: dict) -> str:
    """Bake sun + viewpoint + atmosphere fields into the world text."""
    text = src_text

    # 1. Swap controller name so the one-shot screenshot runs.
    if 'controller "sun_control"' not in text:
        raise SystemExit("source world: no `controller \"sun_control\"` to patch")
    text = text.replace('controller "sun_control"', 'controller "sky_render"')

    # 2. Sun direction (DEF SUN DirectionalLight `direction` field).
    dx, dy, dz = direction_from_azel(float(shot["az_deg"]), float(shot["el_deg"]))
    text = re.sub(
        r'(DEF SUN DirectionalLight \{[^}]*?\n\s+direction)\s+\S+\s+\S+\s+\S+',
        rf'\1 {dx:.6f} {dy:.6f} {dz:.6f}',
        text,
        count=1,
        flags=re.DOTALL,
    )

    # 3. Optional sun colour.
    if "sun_color" in shot:
        c = shot["sun_color"]
        text = re.sub(
            r'(DEF SUN DirectionalLight \{[^}]*?\n\s+color)\s+\S+\s+\S+\s+\S+',
            rf'\1 {c[0]:.3f} {c[1]:.3f} {c[2]:.3f}',
            text,
            count=1,
            flags=re.DOTALL,
        )

    # 4. Optional sun intensity.
    if "sun_intensity" in shot:
        text = re.sub(
            r'(DEF SUN DirectionalLight \{[^}]*?\n\s+intensity)\s+\S+',
            rf'\1 {float(shot["sun_intensity"]):.3f}',
            text,
            count=1,
            flags=re.DOTALL,
        )

    # 5. Optional viewpoint exposure.
    if "exposure" in shot:
        text = re.sub(
            r'(DEF VIEW Viewpoint \{[^}]*?\n\s+exposure)\s+\S+',
            rf'\1 {float(shot["exposure"]):.3f}',
            text,
            count=1,
            flags=re.DOTALL,
        )

    # 6. Optional atmosphere preset.
    if "preset" in shot:
        text = re.sub(
            r'(DEF SKY Background \{[^}]*?\n\s+atmosphericSky)\s+"[^"]*"',
            rf'\1 "{shot["preset"]}"',
            text,
            count=1,
            flags=re.DOTALL,
        )

    return text


def render_shot(src: Path, shot: dict, out_dir: Path) -> Path:
    name = shot["name"]
    bench_world = src.with_name(f"_bench_{src.stem}_{name}.wbt")
    bench_world.write_text(patch_world(src.read_text(encoding="utf-8"), shot),
                           encoding="utf-8")
    out_png = out_dir / f"{name}.png"
    if out_png.exists():
        out_png.unlink()

    env = os.environ.copy()
    env["OMNISIM_SKY_RENDER_OUTPUT"] = str(out_png)
    env["OMNISIM_SKY_RENDER_SETTLE"] = str(shot.get("settle_steps", 24))

    cmd = [
        str(WEBOTS_EXE),
        "--mode=run",
        "--stdout",
        "--stderr",
        str(bench_world),
    ]
    start = time.time()
    try:
        result = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=180)
    finally:
        if os.environ.get("OMNISIM_SKY_BENCH_KEEP"):
            print(f"  (kept {bench_world})")
        else:
            try:
                bench_world.unlink()
            except OSError:
                pass

    ok = out_png.exists()
    elapsed = time.time() - start
    print(f"[bench_sky] {name}  az={shot['az_deg']:+.1f} el={shot['el_deg']:+.1f}  "
          f"{'OK' if ok else 'FAIL'} in {elapsed:.1f}s", flush=True)
    if not ok:
        print("--- stdout ---")
        print(result.stdout)
        print("--- stderr ---")
        print(result.stderr)
    return out_png


def main():
    if len(sys.argv) != 4:
        raise SystemExit("usage: bench_sky.py <source.wbt> <shotlist.json> <output_dir>")
    src = Path(sys.argv[1]).resolve()
    shotlist = Path(sys.argv[2]).resolve()
    out_dir = Path(sys.argv[3]).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    config = json.loads(shotlist.read_text(encoding="utf-8"))
    shots = config["shots"]
    print(f"[bench_sky] {len(shots)} shots, source={src.name}, out={out_dir}", flush=True)

    for shot in shots:
        render_shot(src, shot, out_dir)

    print(f"[bench_sky] done")


if __name__ == "__main__":
    main()
