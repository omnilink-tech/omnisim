#!/usr/bin/env python3
"""Watch the evolved champions in a window WITHOUT cooking the GPU.

The physics is already realtime-capable (measured 4.16 ms/step against the 8 ms
budget, bench_realtime.py), so a windowed engine in its default realtime mode
sleeps between steps. What made watching expensive was the per-frame wgpu
realism stack -- screen-space passes whose cost tracks window resolution, not
scene contents, so five creatures on a bare plane cost nearly what a full city
does. This launcher turns off the passes a flat-arena demo cannot benefit from:

    OMNISIM_WGPU_SSR=0          screen-space reflections (nothing reflective here)
    OMNISIM_WGPU_TAA=0          temporal anti-aliasing
    OMNISIM_WGPU_VOLUMETRIC=0   volumetric sun shafts
    OMNISIM_WGPU_PCSS=0         contact-hardening shadows -> plain CSM
                                (shadows stay; they carry real information)

All four are VALUE-parsed (=0 disables -- verified in
src/omnisim/render/OmWgpuRenderTarget.cpp; this matters because several
OmniSim env vars are presence-gated and =0 arms them).

Deliberately NOT touched:
  * OMNILIGHT -- the GI bake is async CPU work (~0.4 s once); frame cost is
    unchanged by it, so disabling it dulls the image and saves no GPU.
  * shadows entirely (OMNISIM_WGPU_NO_SHADOW) -- the creatures' shadows are the
    main depth cue on a flat arena.
  * --mode -- the engine's default realtime mode is the whole point: physics
    finishes in ~half the tick and the engine idles the remainder. Never pass
    --mode=fast to a windowed session; it uncaps stepping and burns everything.

    python projects/alife/watch.py             # launch windowed, lean
    python projects/alife/watch.py --full      # full realism stack (pretty, hot)
    python projects/alife/watch.py --dry-run   # print env + command, run nothing
"""
import argparse
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(ROOT, "..", ".."))
WORLD = os.path.join(ROOT, "worlds", "alife_champions.omniworld")

LEAN = {
    "OMNISIM_WGPU_SSR": "0",
    "OMNISIM_WGPU_TAA": "0",
    "OMNISIM_WGPU_VOLUMETRIC": "0",
    "OMNISIM_WGPU_PCSS": "0",
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--full", action="store_true",
                    help="keep the full realism stack (SSR/TAA/volumetrics/PCSS)")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    if not os.path.exists(WORLD):
        sys.exit("no showcase world -- run projects/alife/showcase.py first")

    env = dict(os.environ)
    if not args.full:
        env.update(LEAN)

    cmd = [sys.executable, "-m", "omnisim", "run-world",
           os.path.relpath(WORLD, REPO)]
    profile = "FULL realism stack" if args.full else "lean (%s)" % ", ".join(
        "%s=%s" % kv for kv in sorted(LEAN.items()))
    print("profile: %s" % profile)
    print("command: %s  (cwd %s)" % (" ".join(cmd), REPO))
    if args.dry_run:
        return
    subprocess.Popen(cmd, cwd=REPO, env=env)
    print("launched. Close the window when done -- and nothing else keeps running.")


if __name__ == "__main__":
    main()
