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

"""Watch the reef in a window WITHOUT cooking the GPU.

Regenerates the LAST epoch's world (from _run/metazoa/epoch_NN/, else the
shipped projects/metazoa/seeds/) with `config.watch = true` -- so the
director runs forever instead of quitting at `epoch_s` -- into
worlds/metazoa_watch.omniworld, then launches it windowed with the lean
render profile from projects/alife/watch.py:

    OMNISIM_WGPU_SSR=0          screen-space reflections (nothing reflective here)
    OMNISIM_WGPU_TAA=0          temporal anti-aliasing
    OMNISIM_WGPU_VOLUMETRIC=0   volumetric sun shafts
    OMNISIM_WGPU_PCSS=0         contact-hardening shadows -> plain CSM
                                (shadows stay; they carry real information)

All four are VALUE-parsed (=0 disables -- verified in
src/omnisim/render/OmWgpuRenderTarget.cpp; several other OmniSim env vars
are presence-gated and =0 arms them).  NEVER pass --mode=fast to a windowed
session; the engine's default realtime mode is the whole point.

Never leave a second engine running (PowerShell `Get-Process omnisim-bin`;
`pgrep` does not exist in Git Bash and its failure reads as an all-clear).
The epoch driver and this launcher share _run/metazoa/config.json, so do
not run them at the same time.

    python projects/metazoa/watch_metazoa.py              # launch windowed, lean
    python projects/metazoa/watch_metazoa.py --full       # full realism stack (hot)
    python projects/metazoa/watch_metazoa.py --dry-run    # print env + command only
    python projects/metazoa/watch_metazoa.py --build-only # regenerate, no window
    python projects/metazoa/watch_metazoa.py --epoch 3    # a specific epoch
"""
import argparse
import json
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT)

REPO = os.path.normpath(os.path.join(ROOT, "..", ".."))


def _rel(path):
    try:
        return os.path.relpath(path, REPO)
    except ValueError:                      # different Windows drive
        return os.path.abspath(path)


RUN = os.environ.get("METAZOA_RUN_DIR") or os.path.join(ROOT, "_run", "metazoa")
SEEDS = os.path.join(ROOT, "seeds")
WORLD = os.path.join(ROOT, "worlds", "metazoa_watch.omniworld")
CONTROLLER = "metazoa_world"
N_PATCHES = 5

# Verbatim from projects/alife/watch.py -- the launchers must never drift.
LEAN = {
    "OMNISIM_WGPU_SSR": "0",
    "OMNISIM_WGPU_TAA": "0",
    "OMNISIM_WGPU_VOLUMETRIC": "0",
    "OMNISIM_WGPU_PCSS": "0",
}


def latest_epoch_dir(epoch=None):
    """The newest _run/metazoa/epoch_NN that holds a reef, or None."""
    if not os.path.isdir(RUN):
        return None
    found = []
    for name in os.listdir(RUN):
        m = re.fullmatch(r"epoch_(\d+)", name)
        if m and os.path.exists(os.path.join(RUN, name, "reef.json")):
            found.append((int(m.group(1)), os.path.join(RUN, name)))
    if epoch is not None:
        for n, d in found:
            if n == epoch:
                return d
        return None
    return max(found)[1] if found else None


def source_dir(epoch=None):
    edir = latest_epoch_dir(epoch)
    # A fresh clone has no _run/ (gitignored); seeds/ ships the last evolved
    # epoch's reef + config so the demo runs anywhere.
    if edir is None and epoch is None and os.path.exists(os.path.join(SEEDS, "reef.json")):
        edir = SEEDS
    return edir


def regenerate(edir, world=WORLD):
    """Copy the epoch's inputs back to _run/metazoa/ with watch=true and
    write the watch world.  The director reads _run/metazoa/, never
    epoch_NN/."""
    from mz import worldgen as W          # noqa: E402  (implementer A)
    from mz import scene as S             # noqa: E402

    with open(os.path.join(edir, "reef.json"), encoding="utf-8") as f:
        reef = json.load(f)
    with open(os.path.join(edir, "config.json"), encoding="utf-8") as f:
        config = json.load(f)
    config["watch"] = True
    os.makedirs(RUN, exist_ok=True)
    with open(os.path.join(RUN, "reef.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump(reef, f, indent=1)
    with open(os.path.join(RUN, "config.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump(config, f, indent=1)
    W.write_world(reef["cells"], world,
                  scene_lines=S.scene_lines(config.get("arena", reef.get("arena", 18.0)),
                                            config.get("n_patches", N_PATCHES), CONTROLLER),
                  controller=CONTROLLER)
    bal = S.brace_balance(open(world, encoding="utf-8").read())
    if not bal["balanced"]:
        sys.exit("generated world has unbalanced braces: %s" % bal)
    return reef, config


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--full", action="store_true",
                    help="keep the full realism stack (SSR/TAA/volumetrics/PCSS)")
    ap.add_argument("--epoch", type=int, default=None,
                    help="regenerate this epoch instead of the latest")
    ap.add_argument("--dry-run", action="store_true",
                    help="print env + command, regenerate nothing, run nothing")
    ap.add_argument("--build-only", action="store_true",
                    help="regenerate the watch world and stop (no window)")
    args = ap.parse_args(argv)

    edir = source_dir(args.epoch)
    env = dict(os.environ)
    if not args.full:
        env.update(LEAN)
    cmd = [sys.executable, "-m", "omnisim", "run-world", _rel(WORLD)]
    profile = "FULL realism stack" if args.full else "lean (%s)" % ", ".join(
        "%s=%s" % kv for kv in sorted(LEAN.items()))

    print("source : %s" % (_rel(edir) if edir else
                           "NO EPOCH FOUND under %s and no seeds/ -- run metazoa.py first"
                           % _rel(RUN)))
    print("world  : %s  (config.watch = true)" % _rel(WORLD))
    print("profile: %s" % profile)
    print("env    : %s" % (" ".join("%s=%s" % kv for kv in sorted(LEAN.items()))
                           if not args.full else "(inherited)"))
    print("command: %s  (cwd %s)" % (" ".join(cmd), REPO))
    if args.dry_run:
        return 0
    if edir is None:
        sys.exit("no epoch to watch -- run projects/metazoa/metazoa.py first")

    reef, config = regenerate(edir)
    print("regenerated: %d cells (%d organisms, %d free, %d parked), arena %g m" % (
        len(reef["cells"]), len(reef.get("organisms", [])), len(reef.get("free", [])),
        len(reef.get("parked", [])), config.get("arena", reef.get("arena", 0))))
    if args.build_only:
        return 0
    subprocess.Popen(cmd, cwd=REPO, env=env)
    print("launched. Close the window when done -- and nothing else keeps running.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
