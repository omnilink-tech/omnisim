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

"""Watch the living ecosystem in a window WITHOUT cooking the GPU.

Regenerates the LAST epoch's world (from _run/life/epoch_NN/) with
`config.watch = true` -- so the director runs forever instead of quitting at
`epoch_s` -- into worlds/alife_life.omniworld, then launches it windowed with
the lean render profile from watch.py:

    OMNISIM_WGPU_SSR=0          screen-space reflections (nothing reflective here)
    OMNISIM_WGPU_TAA=0          temporal anti-aliasing
    OMNISIM_WGPU_VOLUMETRIC=0   volumetric sun shafts
    OMNISIM_WGPU_PCSS=0         contact-hardening shadows -> plain CSM
                                (shadows stay; they carry real information)

All four are VALUE-parsed (=0 disables -- verified in
src/omnisim/render/OmWgpuRenderTarget.cpp; this matters because several
OmniSim env vars are presence-gated and =0 arms them). The physics is
realtime-capable (bench_realtime.py: 4.16 ms/step against the 8 ms tick for
five creatures), so a windowed engine in its default realtime mode idles
between steps; the GPU cost was the per-frame realism stack, whose cost
tracks window resolution rather than scene contents.

Deliberately NOT touched:
  * OMNILIGHT -- the GI bake is async CPU work (~0.4 s once); frame cost is
    unchanged by it, so disabling it dulls the image and saves no GPU.
  * shadows entirely (OMNISIM_WGPU_NO_SHADOW) -- the creatures' shadows are
    the main depth cue on a flat arena.
  * --mode -- the engine's default realtime mode is the whole point. NEVER
    pass --mode=fast to a windowed session; it uncaps stepping and burns
    everything.

And never leave a second engine running (check with PowerShell
`Get-Process omnisim-bin`; `pgrep` does not exist in this Git Bash and its
failure reads as an all-clear). The ecosystem driver and this launcher
share _run/life/config.json, so do not run them at the same time.

    python projects/alife/watch_life.py             # launch windowed, lean
    python projects/alife/watch_life.py --full      # full realism stack (pretty, hot)
    python projects/alife/watch_life.py --dry-run   # print env + command, run nothing
    python projects/alife/watch_life.py --epoch 3   # a specific epoch, not the last
"""
import argparse
import json
import os
import re
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

ROOT = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(ROOT, "..", ".."))
RUN = os.path.join(ROOT, "_run", "life")
WORLD = os.path.join(ROOT, "worlds", "alife_life.omniworld")
CONTROLLER = "terrarium_life"

# Verbatim from watch.py -- the two launchers must never drift.
LEAN = {
    "OMNISIM_WGPU_SSR": "0",
    "OMNISIM_WGPU_TAA": "0",
    "OMNISIM_WGPU_VOLUMETRIC": "0",
    "OMNISIM_WGPU_PCSS": "0",
}


def latest_epoch_dir(epoch=None):
    """The newest _run/life/epoch_NN that holds a population, or None."""
    if not os.path.isdir(RUN):
        return None
    found = []
    for name in os.listdir(RUN):
        m = re.fullmatch(r"epoch_(\d+)", name)
        if m and os.path.exists(os.path.join(RUN, name, "population.json")):
            found.append((int(m.group(1)), os.path.join(RUN, name)))
    if epoch is not None:
        for n, d in found:
            if n == epoch:
                return d
        return None
    return max(found)[1] if found else None


def regenerate(edir):
    """Copy the epoch's inputs back to _run/life/ with watch=true and write
    the watch world. The director reads _run/life/, never epoch_NN/."""
    from alife import worldgen2 as W2    # noqa: E402  (implementer A)
    from alife import scene as S         # noqa: E402

    with open(os.path.join(edir, "population.json"), encoding="utf-8") as f:
        pop = json.load(f)
    with open(os.path.join(edir, "config.json"), encoding="utf-8") as f:
        config = json.load(f)
    config["watch"] = True
    os.makedirs(RUN, exist_ok=True)
    with open(os.path.join(RUN, "population.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump(pop, f, indent=1)
    with open(os.path.join(RUN, "config.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump(config, f, indent=1)
    W2.write_world(pop, WORLD,
                   scene_lines=S.scene_lines(config["arena"], config["food_pool"], CONTROLLER),
                   controller=CONTROLLER)
    return pop, config


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--full", action="store_true",
                    help="keep the full realism stack (SSR/TAA/volumetrics/PCSS)")
    ap.add_argument("--epoch", type=int, default=None,
                    help="regenerate this epoch instead of the latest")
    ap.add_argument("--dry-run", action="store_true",
                    help="print env + command, regenerate nothing, run nothing")
    ap.add_argument("--build-only", action="store_true",
                    help="regenerate the watch world and stop (no window)")
    args = ap.parse_args()

    edir = latest_epoch_dir(args.epoch)
    # A fresh clone has no _run/ (gitignored). projects/alife/seeds/ ships the
    # population + config of the last evolved epoch so the demo runs anywhere.
    if edir is None and os.path.exists(os.path.join(ROOT, "seeds", "population.json")):
        edir = os.path.join(ROOT, "seeds")
    env = dict(os.environ)
    if not args.full:
        env.update(LEAN)
    cmd = [sys.executable, "-m", "omnisim", "run-world",
           os.path.relpath(WORLD, REPO)]
    profile = "FULL realism stack" if args.full else "lean (%s)" % ", ".join(
        "%s=%s" % kv for kv in sorted(LEAN.items()))

    print("source : %s" % (os.path.relpath(edir, REPO) if edir else
                           "NO EPOCH FOUND under %s -- run ecosystem.py first"
                           % os.path.relpath(RUN, REPO)))
    print("world  : %s  (config.watch = true)" % os.path.relpath(WORLD, REPO))
    print("profile: %s" % profile)
    print("env    : %s" % (" ".join("%s=%s" % kv for kv in sorted(LEAN.items()))
                           if not args.full else "(inherited)"))
    print("command: %s  (cwd %s)" % (" ".join(cmd), REPO))
    if args.dry_run:
        return
    if edir is None:
        sys.exit("no epoch to watch -- run projects/alife/ecosystem.py first")

    pop, config = regenerate(edir)
    print("regenerated: %d creatures, arena %g m, food pool %d" % (
        len(pop), config["arena"], config["food_pool"]))
    if args.build_only:
        return
    subprocess.Popen(cmd, cwd=REPO, env=env)
    print("launched. Close the window when done -- and nothing else keeps running.")


if __name__ == "__main__":
    main()
