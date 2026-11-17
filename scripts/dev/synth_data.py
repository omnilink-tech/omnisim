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
"""Synthetic-data generator: seeded domain randomization -> aligned RGB / depth / instance datasets.

Drives the wgpu main view (the full render stack: scattered sky, PCSS, SSR, GTAO, clouds) through
the engine's synthetic-data dump (``OMNISIM_WGPU_SYNTH_DUMP``, see
docs/developer/synthetic-data.md). Per sample it randomizes, from one seed:

  - SUN direction   (azimuth uniform, elevation in [12, 65] deg)   -- edits the world's OmniSimSun
  - CLOUD cover     (uniform in [0.10, 0.60])                       -- OMNISIM_WGPU_CLOUD_COVER
  - CAMERA pose     (orbit shell around --target: radius/azimuth/elevation uniform) -- edits Viewpoint

then runs one engine instance windowed (`--mode=realtime`; the wgpu main view never repaints in
--batch/--minimize, so a desktop session is required), waits for the dump, and collects:

  sample_NNN/rgb_*.png     tonemapped RGB (the full wgpu stack)
  sample_NNN/depth_*.png   uint16 MILLIMETRE depth, 0 = no hit
  sample_NNN/inst_*.png    per-solid instance ids (id = R + G*256 + B*65536, 0 = background)
  sample_NNN/meta_*.json   camera intrinsics/extrinsics, light rig, id -> node-name mapping

plus a top-level ``dataset.json`` manifest. Deterministic: same (world, seed, samples) -> same
poses and light rigs (the images are as deterministic as the renderer).

Example:
  python scripts/dev/synth_data.py --world projects/samples/demos/worlds/rendering/beauty_bench.omniworld \
         --out .local-runs/synth_demo --samples 12 --seed 7
"""

import argparse
import json
import math
import os
import random
import re
import shutil
import subprocess
import sys
import time

REPO = os.environ.get("OMNISIM_HOME") or os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


def engine_binary():
    win = os.path.join(REPO, "msys64", "mingw64", "bin", "omnisim-bin.exe")
    lin = os.path.join(REPO, "bin", "omnisim-bin")
    if os.path.exists(win):
        return win
    if os.path.exists(lin):
        return lin
    sys.exit("omnisim-bin not found under OMNISIM_HOME=%s -- build first" % REPO)


def look_at_axis_angle(pos, target):
    """Axis-angle orientation for the OmniSim camera (forward +X, up +Z) looking pos -> target."""
    f = [target[i] - pos[i] for i in range(3)]
    n = math.sqrt(sum(c * c for c in f))
    f = [c / n for c in f]
    up = (0.0, 0.0, 1.0)
    r = [f[1] * up[2] - f[2] * up[1], f[2] * up[0] - f[0] * up[2], f[0] * up[1] - f[1] * up[0]]
    rn = math.sqrt(sum(c * c for c in r)) or 1e-9
    r = [c / rn for c in r]
    u2 = [r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2], r[0] * f[1] - r[1] * f[0]]
    y = [u2[1] * f[2] - u2[2] * f[1], u2[2] * f[0] - u2[0] * f[2], u2[0] * f[1] - u2[1] * f[0]]
    # rotation matrix columns = (f, y, u2); convert to axis-angle
    m = [[f[0], y[0], u2[0]], [f[1], y[1], u2[1]], [f[2], y[2], u2[2]]]
    tr = m[0][0] + m[1][1] + m[2][2]
    ang = math.acos(max(-1.0, min(1.0, (tr - 1.0) / 2.0)))
    s = 2.0 * math.sin(ang) or 1e-9
    ax = [(m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s]
    return ax, ang


def edit_world(text, cam_pos, cam_axis, cam_angle, sun_dir):
    ori = "%.5f %.5f %.5f %.5f" % (cam_axis[0], cam_axis[1], cam_axis[2], cam_angle)
    pos = "%.3f %.3f %.3f" % tuple(cam_pos)
    number = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
    text = re.sub(
        rf"(Viewpoint\s*\{{[^}}]*?\borientation\s+){number}(?:\s+{number}){{3}}",
        lambda m: m.group(1) + ori,
        text,
        count=1,
        flags=re.S,
    )
    text = re.sub(
        rf"(Viewpoint\s*\{{[^}}]*?\bposition\s+){number}(?:\s+{number}){{2}}",
        lambda m: m.group(1) + pos,
        text,
        count=1,
        flags=re.S,
    )
    if sun_dir is not None:
        sd = "%.4f %.4f %.4f" % tuple(sun_dir)
        text, n = re.subn(
            rf"(OmniSimSun\s*\{{[^}}]*?\bdirection\s+){number}(?:\s+{number}){{2}}",
            lambda m: m.group(1) + sd,
            text,
            count=1,
            flags=re.S,
        )
        if n == 0:
            print("  [warn] world has no OmniSimSun direction to randomize -- sun left as authored")
    return text


def absolutize_local_assets(text, world_dir):
    """Keep local quoted asset references valid after copying a world to the output tree."""
    def replace(match):
        value = match.group(1)
        if "://" in value or os.path.isabs(value):
            return match.group(0)
        candidate = os.path.normpath(os.path.join(world_dir, value))
        if not os.path.exists(candidate):
            return match.group(0)
        return '"%s"' % candidate.replace("\\", "/")

    return re.sub(r'"([^"\r\n]+)"', replace, text)


def run_sample(binary, world_path, out_dir, cloud_cover, frame, timeout_s):
    env = dict(os.environ)
    env["OMNISIM_HOME"] = REPO
    env["OMNISIM_WGPU_SYNTH_DUMP"] = out_dir
    env["OMNISIM_WGPU_MAINVIEW_DUMP_FRAME"] = str(frame)
    env["OMNISIM_WGPU_CLOUD_COVER"] = "%.3f" % cloud_cover
    env["OMNISIM_LOG_PATH"] = os.path.join(out_dir, "engine_log.txt")
    env.pop("OMNISIM_WGPU_MAINVIEW_DUMP", None)
    if os.name == "nt":
        runtime = os.path.join(os.path.dirname(binary), "newton-runtime")
        runtime_site = os.path.join(runtime, "site-packages")
        if os.path.exists(os.path.join(runtime, "python.exe")):
            env["PATH"] = runtime + os.pathsep + env.get("PATH", "")
        if os.path.isdir(runtime_site):
            env["PYTHONPATH"] = runtime_site + os.pathsep + env.get("PYTHONPATH", "")
    proc = subprocess.Popen([binary, world_path, "--mode=realtime"], env=env,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    meta = os.path.join(out_dir, "meta_%06d.json" % frame)
    deadline = time.time() + timeout_s
    ok = False
    try:
        while time.time() < deadline:
            if os.path.exists(meta):
                ok = True
                time.sleep(1.5)  # let the last PNG finish writing
                break
            if proc.poll() is not None:
                break
            time.sleep(0.5)
    finally:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
    expected = ["rgb_%06d.png" % frame, "depth_%06d.png" % frame, "inst_%06d.png" % frame,
                "meta_%06d.json" % frame]
    have = [f for f in expected if os.path.exists(os.path.join(out_dir, f))]
    return ok and len(have) == 4, have


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", default="projects/samples/demos/worlds/rendering/beauty_bench.omniworld")
    ap.add_argument("--out", required=True)
    ap.add_argument("--samples", type=int, default=8)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--frame", type=int, default=60, help="engine frame to dump (after settle)")
    ap.add_argument("--timeout", type=float, default=90.0, help="per-sample engine timeout (s)")
    ap.add_argument("--target", default="0,0,1.2", help="camera look-at point x,y,z")
    ap.add_argument("--radius", default="7,15", help="camera orbit radius range min,max (m)")
    ap.add_argument("--cam-elev", default="8,40", help="camera elevation range min,max (deg)")
    ap.add_argument("--sun-elev", default="12,65", help="sun elevation range min,max (deg)")
    ap.add_argument("--cloud", default="0.10,0.60", help="cloud cover range min,max")
    ap.add_argument("--no-random-sun", action="store_true")
    ap.add_argument("--keep-worlds", action="store_true", help="keep the per-sample world files")
    args = ap.parse_args()

    binary = engine_binary()
    world_src = os.path.join(REPO, args.world) if not os.path.isabs(args.world) else args.world
    if not os.path.exists(world_src):
        sys.exit("world not found: %s" % world_src)
    base = open(world_src, encoding="utf-8").read()
    base = absolutize_local_assets(base, os.path.dirname(world_src))
    target = tuple(float(v) for v in args.target.split(","))
    r_lo, r_hi = (float(v) for v in args.radius.split(","))
    ce_lo, ce_hi = (float(v) for v in args.cam_elev.split(","))
    se_lo, se_hi = (float(v) for v in args.sun_elev.split(","))
    cl_lo, cl_hi = (float(v) for v in args.cloud.split(","))

    os.makedirs(args.out, exist_ok=True)
    manifest = {"world": args.world, "seed": args.seed, "frame": args.frame, "samples": []}
    n_ok = 0
    for i in range(args.samples):
        rng = random.Random((args.seed << 20) + i)
        # camera on an orbit shell, always looking at the target
        az = rng.uniform(0.0, 2.0 * math.pi)
        el = math.radians(rng.uniform(ce_lo, ce_hi))
        rad = rng.uniform(r_lo, r_hi)
        cam = (target[0] + rad * math.cos(el) * math.cos(az),
               target[1] + rad * math.cos(el) * math.sin(az),
               target[2] + rad * math.sin(el))
        ax, ang = look_at_axis_angle(cam, target)
        # sun: direction the LIGHT TRAVELS (down = negative z)
        sun = None
        if not args.no_random_sun:
            saz = rng.uniform(0.0, 2.0 * math.pi)
            sel = math.radians(rng.uniform(se_lo, se_hi))
            sun = (-math.cos(sel) * math.cos(saz), -math.cos(sel) * math.sin(saz), -math.sin(sel))
        cover = rng.uniform(cl_lo, cl_hi)

        sdir = os.path.join(args.out, "sample_%03d" % i)
        os.makedirs(sdir, exist_ok=True)
        wpath = os.path.join(sdir, "world.omniworld")
        with open(wpath, "w", encoding="utf-8", newline="\n") as f:
            f.write(edit_world(base, cam, ax, ang, sun))

        t0 = time.time()
        ok, have = run_sample(binary, wpath, sdir, cover, args.frame, args.timeout)
        if not ok:  # one retry (the documented ~1-in-3 cold-launch flake class)
            print("  sample %03d incomplete (%d/4) -- retrying once" % (i, len(have)))
            ok, have = run_sample(binary, wpath, sdir, cover, args.frame, args.timeout)
        dt = time.time() - t0
        print("sample %03d: %s in %.1fs  (cam az %.0f deg el %.0f deg r %.1f m, sun el %s, cloud %.2f)"
              % (i, "OK" if ok else "FAILED", dt, math.degrees(az), math.degrees(el), rad,
                 ("%.0f deg" % math.degrees(math.asin(-sun[2]))) if sun else "authored", cover))
        if ok:
            n_ok += 1
        if not args.keep_worlds and os.path.exists(wpath):
            os.remove(wpath)
        manifest["samples"].append({
            "dir": "sample_%03d" % i, "ok": ok,
            "camera": {"position": list(cam), "target": list(target),
                       "azimuth_deg": math.degrees(az), "elevation_deg": math.degrees(el),
                       "radius_m": rad},
            "sun_direction": list(sun) if sun else None,
            "cloud_cover": cover,
        })
        with open(os.path.join(args.out, "dataset.json"), "w", encoding="utf-8") as f:
            json.dump(manifest, f, indent=2)

    print("done: %d/%d samples -> %s" % (n_ok, args.samples, args.out))
    return 0 if n_ok == args.samples else 1


if __name__ == "__main__":
    sys.exit(main())
