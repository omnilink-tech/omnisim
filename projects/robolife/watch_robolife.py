#!/usr/bin/env python3
"""Watch RoboLife in a window WITHOUT cooking the GPU.

Regenerates the LAST epoch's fleet (from _run/robolife/epoch_NN/, else
projects/robolife/seeds/, else a fresh default fleet) with `config.watch =
true` -- so the supervisor runs forever instead of quitting at `epoch_s` --
into worlds/robolife_watch.omniworld, then launches it windowed with the
lean render profile (verbatim from projects/alife/watch_life.py):

    OMNISIM_WGPU_SSR=0          screen-space reflections
    OMNISIM_WGPU_TAA=0          temporal anti-aliasing
    OMNISIM_WGPU_VOLUMETRIC=0   volumetric sun shafts
    OMNISIM_WGPU_PCSS=0         contact-hardening shadows -> plain CSM

All four are VALUE-parsed (=0 disables). Deliberately NOT touched: OMNILIGHT
(async CPU bake, no frame cost), shadows entirely (the main depth cue), and
--mode (NEVER pass --mode=fast to a windowed session).

Never leave a second engine running (PowerShell `Get-Process omnisim-bin`;
`pgrep` does not exist in Git Bash). The epoch driver and this launcher
share _run/robolife/fleet.json, so do not run them at the same time.

    python projects/robolife/watch_robolife.py             # launch windowed, lean
    python projects/robolife/watch_robolife.py --full      # full realism stack
    python projects/robolife/watch_robolife.py --dry-run   # print env + command
    python projects/robolife/watch_robolife.py --build-only
    python projects/robolife/watch_robolife.py --epoch 3
"""
import argparse
import json
import os
import re
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import robolife as RL   # noqa: E402

ROOT = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(ROOT, "..", ".."))
RUN = os.path.join(ROOT, "_run", "robolife")
WORLD = os.path.join(ROOT, "worlds", "robolife_watch.omniworld")

# Verbatim from projects/alife/watch.py -- the launchers must never drift.
LEAN = {
    "OMNISIM_WGPU_SSR": "0",
    "OMNISIM_WGPU_TAA": "0",
    "OMNISIM_WGPU_VOLUMETRIC": "0",
    "OMNISIM_WGPU_PCSS": "0",
}


def latest_epoch_dir(epoch=None):
    if not os.path.isdir(RUN):
        return None
    found = []
    for name in os.listdir(RUN):
        m = re.fullmatch(r"epoch_(\d+)", name)
        if m and os.path.exists(os.path.join(RUN, name, "fleet.json")):
            found.append((int(m.group(1)), os.path.join(RUN, name)))
    if epoch is not None:
        for n, d in found:
            if n == epoch:
                return d
        return None
    return max(found)[1] if found else None


def source_fleet(epoch=None):
    """(fleet dict, description). Epoch dir, else seeds, else a fresh
    default fleet built in memory (and said so)."""
    edir = latest_epoch_dir(epoch)
    if edir is not None:
        with open(os.path.join(edir, "fleet.json"), encoding="utf-8") as f:
            return json.load(f), os.path.relpath(edir, REPO)
    if os.path.exists(RL.SEEDS):
        with open(RL.SEEDS, encoding="utf-8") as f:
            return json.load(f), os.path.relpath(RL.SEEDS, REPO)
    import random
    args = RL.parse_args([])
    rng = random.Random(args.seed)
    fleet = RL.build_fleet(RL.initial_lineages(args.alive, rng), args, 0, rng)
    return fleet, "FRESH default fleet (no _run epoch, no seeds/fleet.json)"


def regenerate(fleet):
    """Write the fleet back to _run/robolife/ with watch=true and write the
    watch world. The supervisor reads _run/robolife/, never epoch_NN/."""
    fleet = json.loads(json.dumps(fleet))
    fleet["config"]["watch"] = True
    RL.write_fleet(fleet)
    RL.write_world(fleet, WORLD)
    return fleet


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--full", action="store_true", help="keep the full realism stack")
    ap.add_argument("--epoch", type=int, default=None, help="regenerate this epoch, not the latest")
    ap.add_argument("--dry-run", action="store_true", help="print env + command, run nothing")
    ap.add_argument("--build-only", action="store_true", help="regenerate the watch world and stop")
    args = ap.parse_args()

    fleet, src = source_fleet(args.epoch)
    env = dict(os.environ)
    if not args.full:
        env.update(LEAN)
    cmd = [sys.executable, "-m", "omnisim", "run-world", os.path.relpath(WORLD, REPO)]
    profile = "FULL realism stack" if args.full else "lean (%s)" % ", ".join(
        "%s=%s" % kv for kv in sorted(LEAN.items()))
    cfg = fleet["config"]
    print("source : %s" % src)
    print("fleet  : %d robots (%d alive), %d modules (%d loose), arena %g m" % (
        len(fleet["robots"]), sum(1 for r in fleet["robots"] if r["alive_at_start"]),
        len(fleet["modules"]), sum(1 for m in fleet["modules"] if m["loose_at_start"]),
        cfg["arena"]))
    print("world  : %s  (config.watch = true)" % os.path.relpath(WORLD, REPO))
    print("profile: %s" % profile)
    print("env    : %s" % (" ".join("%s=%s" % kv for kv in sorted(LEAN.items()))
                           if not args.full else "(inherited)"))
    print("command: %s  (cwd %s)" % (" ".join(cmd), REPO))
    try:
        RL._worldgen()
        print("worldgen: rl.worldgen present")
    except ImportError as exc:
        print("worldgen: rl.worldgen NOT importable (%s) -- regenerate will fail" % exc)
    if args.dry_run:
        return
    regenerate(fleet)
    print("regenerated: %s" % os.path.relpath(WORLD, REPO))
    if args.build_only:
        return
    subprocess.Popen(cmd, cwd=REPO, env=env)
    print("launched. Close the window when done -- and nothing else keeps running.")


if __name__ == "__main__":
    main()
