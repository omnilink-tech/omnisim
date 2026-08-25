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
"""render_quality — objective, NO-REFERENCE quality measure for the wgpu main view.

The deliverable of the "stop judging by WREN-match" correction: WREN is the legacy renderer we are
*replacing*, so "matches WREN" is the wrong bar — wgpu should be free to look different, ideally better.
This tool judges a wgpu render ON ITS OWN MERITS (no reference image), catching the failure modes that
actually matter for a viewport:

  * crushed shadows  — too many pixels collapsed to near-black (the regression a single-world-tuned
                       "ambient 0" caused on every other world: lost detail in shadowed regions);
  * blown highlights — too many pixels clipped to white (over-exposed);
  * dead frame       — almost no contrast / nearly uniform (a wall-facing viewpoint or a failed render);
  * tonal balance    — mean luminance sitting in a healthy mid band, not floored or ceiled.

It renders the ACTUAL main-view path (renderMainFrameViaWgpu, via OMNISIM_WGPU_MAINVIEW_FORCE) — i.e.
what a default flip would actually ship — NOT the wren-parity self-check. Pass --png to measure an
existing dump instead of rendering.

Caveat: a no-reference measure on a world's DEFAULT viewpoint cannot tell a bad *render* from a bad
*camera angle* (some demo worlds frame a wall or the dark). It flags "too dark / flat" either way —
which is honest (that frame is unhelpful), but read low scores on known-bad-viewpoint worlds with that
in mind. The point is to catch tonal/shadow regressions where geometry IS in frame, with NO WREN bar.

Run from PowerShell (needs the wgpu-ON binary + a GPU).
    python scripts/dev/render_quality.py --world W [--frame N] [--no-csm]
    python scripts/dev/render_quality.py --png some_render.png
"""

import argparse
import os
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

# Healthy-frame bands (0-255). Tuned to flag the real failure modes, not to be fussy.
MEAN_LUMA_MIN, MEAN_LUMA_MAX = 22.0, 205.0  # not floored to black, not ceiled to white
NEAR_BLACK_T = 20                            # max(RGB) below this = a "black" pixel
CRUSHED_FRAC_MAX = 0.55                      # >55% black pixels = crushed (or a black viewpoint)
BLOWN_T = 250                                # min(RGB) above this = a clipped-white pixel
BLOWN_FRAC_MAX = 0.15
CONTRAST_MIN = 9.0                           # std(luma) below this = a dead/flat frame
# "Shadow detail": of the darker pixels, how many are PURE black (no fill). High = crushed shadows.
SHADOW_LUMA_T = 55
PURE_BLACK_T = 8
SHADOW_PURE_FRAC_MAX = 0.80


def render(world, frame, csm):
    out = os.path.join(REPO, "_scratch", "rq_%s.png" % os.path.splitext(os.path.basename(world))[0])
    os.makedirs(os.path.dirname(out), exist_ok=True)
    try:
        os.remove(out)
    except OSError:
        pass
    env = dict(os.environ)
    env["WEBOTS_HOME"] = REPO
    env["OMNISIM_HOME"] = REPO
    env["OMNISIM_WGPU_MAINVIEW_FORCE"] = "1"
    if csm:
        env["OMNISIM_WGPU_MAINVIEW_CSM"] = "1"
    env["OMNISIM_WGPU_MAINVIEW_DUMP"] = out
    env["OMNISIM_WGPU_MAINVIEW_DUMP_FRAME"] = str(frame)
    proc = subprocess.Popen([BIN, world, "--mode=run", "--minimize"], cwd=REPO, env=env)
    last, stable, deadline = -1, 0, time.time() + 80
    while time.time() < deadline and stable < 3:
        time.sleep(1)
        if proc.poll() is not None and not os.path.exists(out):
            break
        if os.path.exists(out):
            sz = os.path.getsize(out)
            stable = stable + 1 if (sz == last and sz > 0) else 0
            last = sz
    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            proc.kill()
    return out if os.path.exists(out) and os.path.getsize(out) > 0 else None


def measure(png):
    try:
        from PIL import Image
        import numpy as np
    except Exception:
        print("[render-quality] needs Pillow + numpy (pip install pillow numpy)")
        return None
    try:
        a = np.asarray(Image.open(png).convert("RGB")).astype("float32").reshape(-1, 3)
    except Exception as e:
        print("[render-quality] cannot read %s: %s" % (png, e))
        return None
    luma = 0.2126 * a[:, 0] + 0.7152 * a[:, 1] + 0.0722 * a[:, 2]
    mx, mn = a.max(1), a.min(1)
    n = len(luma)
    near_black = float((mx < NEAR_BLACK_T).mean())
    blown = float((mn > BLOWN_T).mean())
    contrast = float(luma.std())
    dark = luma < SHADOW_LUMA_T
    shadow_pure = float((luma[dark] < PURE_BLACK_T).mean()) if dark.any() else 0.0
    return {
        "mean_luma": float(luma.mean()),
        "near_black_frac": near_black,
        "blown_frac": blown,
        "contrast": contrast,
        "shadow_pure_frac": shadow_pure,  # crushed-shadow regression metric
        "colorfulness": float(a.std(0).mean()),
    }


def verdict(m):
    fails = []
    if not (MEAN_LUMA_MIN <= m["mean_luma"] <= MEAN_LUMA_MAX):
        fails.append("mean-luma %.0f outside [%g,%g]" % (m["mean_luma"], MEAN_LUMA_MIN, MEAN_LUMA_MAX))
    if m["near_black_frac"] > CRUSHED_FRAC_MAX:
        fails.append("crushed %.0f%% black px" % (m["near_black_frac"] * 100))
    if m["blown_frac"] > BLOWN_FRAC_MAX:
        fails.append("blown %.0f%% clipped px" % (m["blown_frac"] * 100))
    if m["contrast"] < CONTRAST_MIN:
        fails.append("flat (contrast %.1f)" % m["contrast"])
    if m["shadow_pure_frac"] > SHADOW_PURE_FRAC_MAX:
        fails.append("crushed shadows (%.0f%% of darks are pure black)" % (m["shadow_pure_frac"] * 100))
    return fails


def main():
    ap = argparse.ArgumentParser(description="No-reference quality measure for the wgpu main view")
    ap.add_argument("--world", help="world (.wbt) to render via the main-view path and measure")
    ap.add_argument("--png", help="measure an existing render PNG instead of rendering")
    ap.add_argument("--frame", type=int, default=200, help="dump frame (default 200; lower for short worlds)")
    ap.add_argument("--no-csm", action="store_true", help="single-cascade path instead of multi-cascade")
    args = ap.parse_args()

    png = args.png
    if not png:
        if not args.world:
            print("[render-quality] need --world or --png")
            return 2
        if not os.path.exists(BIN):
            print("[render-quality] omnisim-bin not found — build it first.")
            return 2
        world = args.world if os.path.isabs(args.world) else os.path.join(REPO, args.world)
        png = render(world, args.frame, not args.no_csm)
        if not png:
            print("[render-quality] %-34s NO RENDER (world quit early / failed to load)"
                  % os.path.basename(args.world))
            return 1

    m = measure(png)
    if m is None:
        return 2
    fails = verdict(m)
    label = os.path.basename(args.world) if args.world else os.path.basename(png)
    print("[render-quality] %-34s luma=%5.1f nearBlack=%4.1f%% blown=%4.1f%% contrast=%5.1f "
          "shadowPure=%4.1f%% colorful=%4.1f  ->  %s"
          % (label, m["mean_luma"], m["near_black_frac"] * 100, m["blown_frac"] * 100, m["contrast"],
             m["shadow_pure_frac"] * 100, m["colorfulness"], "HEALTHY" if not fails else "FLAGS: " + "; ".join(fails)))
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
