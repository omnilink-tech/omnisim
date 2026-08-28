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

"""Epoch driver for RoboLife (DESIGN.md, "Energy and lifecycle": epoch driver).

Each epoch:
  1. build the fleet: N robot slots (`--alive` alive at start, one lineage
     each), M pooled modules (`--loose` loose in the arena, the rest parked
     on the crypt), 3 pads, the bay at the centre
  2. write _run/robolife/fleet.json, regenerate the world
     (rl.worldgen.write_world [A] + rl.scene.scene_lines)
  3. run ONE headless engine; the robolife_world supervisor runs the rules
     for `epoch_s` of sim time and writes _run/robolife/epoch_result.json
  4. score every lineage = fabrications*10 + modules_docked +
     charge_collected/50, keep the top half, refill with mutations of the
     keepers' best genomes (new lineage ids)

Exactly ONE engine process runs at a time (thermal limit; and `run-headless`
reads a shared log unless OMNISIM_LOG_PATH is unique, which it is here).

  python projects/robolife/robolife.py --epochs 6 --robots 6 --alive 4 \
      --modules 14 --loose 10 --epoch-s 240 --arena 24 --seed 7
  python projects/robolife/robolife.py --dry-run --epochs 2     # plan only
  python projects/robolife/robolife.py --resume --epochs 3      # continue
  python projects/robolife/robolife.py --write-seeds            # seeds/fleet.json

Persisted under _run/robolife/epoch_NN/: fleet.json, the world,
epoch_result.json, telemetry.json, lineages.json (scores + verdicts) and the
engine log; _run/robolife/state.json carries the lineage roster between
invocations (that is what --resume reads).
"""
import argparse
import json
import math
import os
import random
import shutil
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from rl import energy as E   # noqa: E402  (pure; always importable)
from rl import scene as S    # noqa: E402

ROOT = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(ROOT, "..", ".."))
RUN = os.path.join(ROOT, "_run", "robolife")
WORLD = os.path.join(ROOT, "worlds", "robolife.omniworld")
STATE = os.path.join(RUN, "state.json")
SEEDS = os.path.join(ROOT, "seeds", "fleet.json")
CONTROLLER_WORLD = "robolife_world"
CONTROLLER_ROBOT = "robolife_robot"
# The supervisor quits at epoch_s; the slack covers load + finalize on a cold
# disk. `run-headless --duration` is a wall-clock SLEEP, so the supervisor's
# simulationQuit(0) is what actually ends the run early.
DURATION_SLACK_S = 60
SPAWN_RING = 0.30          # alive robots start on a ring at this x arena
MIN_GAP_M = 2.0            # loose modules keep this far from pads / bay / each other


# --------------------------------------------------------------------------
# worldgen seam (owned by implementer A)
#
# Imported lazily so `--dry-run`, `--write-seeds` and `--help` work before
# the module lands. The ONE call this driver makes, exactly:
#
#   rl.worldgen.write_world(robots, modules, path,
#                           scene_lines=<list of str>, controller="robolife_robot")
#
# where `robots` / `modules` are the lists written to fleet.json (below) --
# every robot carries slot, id, lineage, alive_at_start, genome, pos [x,y,z],
# yaw, charge_frac, docked; every module carries id, type, loose (+ the
# supervisor's loose_at_start twin), pos, yaw. worldgen reads slot /
# alive_at_start / pos / yaw and id / type / loose / pos / yaw, parks the
# rest itself (rl.worldgen.robot_park_position / module_park_position --
# rl.energy's park translations equal them), and emits DEF ROBOT_<slot> /
# MODULE_<id>. It RETURNS the placements it authored.
# --------------------------------------------------------------------------

def _worldgen():
    import importlib
    return importlib.import_module("rl.worldgen")


# --------------------------------------------------------------------------
# fleet
# --------------------------------------------------------------------------

def _far_enough(p, others, gap):
    return all(math.hypot(p[0] - q[0], p[1] - q[1]) >= gap for q in others)


def build_fleet(lineages, args, epoch, rng, watch=False):
    """-> {config, robots, modules}. Alive robots take the lineage roster in
    order (one founder each); the remaining slots are parked with no
    lineage until a fabrication fills them."""
    arena = float(args.arena)
    pads = S.default_pads(arena)
    bay = [0.0, 0.0]
    n_alive = min(args.alive, len(lineages), args.robots)
    robots = []
    ring = SPAWN_RING * arena
    for i in range(args.robots):
        alive = i < n_alive
        if alive:
            a = 2 * math.pi * (i + 0.5) / n_alive
            pos = [round(ring * math.cos(a), 3), round(ring * math.sin(a), 3), E.ROBOT_SPAWN_Z]
            yaw = round(rng.uniform(-math.pi, math.pi), 4)
            lin = lineages[i]
            genome = E.clamp_genome(lin["genome"])
            rid = "%s_g%d_%02d" % (lin["id"], epoch, i)
            lineage = lin["id"]
        else:
            pos = E.robot_park_translation(i)
            yaw, lineage, rid = 0.0, None, "pool_%02d" % i
            genome = E.default_genome()
        robots.append({"slot": i, "id": rid, "lineage": lineage, "alive_at_start": alive,
                       "genome": genome, "pos": pos, "yaw": yaw,
                       "charge_frac": E.START_FRAC if alive else 0.0,
                       "docked": {"front": None, "rear": None}})

    keep_out = [tuple(p) for p in pads] + [tuple(bay)] + [tuple(r["pos"][:2]) for r in robots if r["alive_at_start"]]
    modules = []
    h = arena / 2.0 - 1.5
    placed = []
    for j in range(args.modules):
        loose = j < args.loose
        mtype = E.MODULE_TYPES[j % len(E.MODULE_TYPES)]
        if loose:
            for _ in range(200):
                p = (rng.uniform(-h, h), rng.uniform(-h, h))
                if _far_enough(p, keep_out + placed, MIN_GAP_M):
                    break
            placed.append(p)
            pos = [round(p[0], 3), round(p[1], 3), E.MODULE_SPAWN_Z]
            yaw = round(rng.uniform(-math.pi, math.pi), 4)
        else:
            pos, yaw = E.module_park_translation(j), 0.0
        # `loose` is the key rl.worldgen.module_placements reads; `loose_at_start`
        # is the one the supervisor reads (its `loose` is live state).
        modules.append({"id": j, "type": mtype, "loose": loose, "loose_at_start": loose,
                        "pos": pos, "yaw": yaw})

    config = {
        "arena": arena, "pads": pads, "bay": bay, "epoch_s": args.epoch_s,
        "watch": bool(watch), "epoch": epoch, "seed": args.seed,
        "robots": args.robots, "alive": args.alive, "modules": args.modules,
        "loose": args.loose, "lineages": [lin["id"] for lin in lineages[:n_alive]],
    }
    return {"config": config, "robots": robots, "modules": modules}


def initial_lineages(n, rng):
    return [{"id": "L%d" % k, "genome": E.random_genome(rng), "born_epoch": 0,
             "parent": None} for k in range(n)]


def score_lineages(result):
    """epoch_result.json -> {lineage: {score, ...}}; tolerant of gaps."""
    per = result.get("lineages", {}) if isinstance(result, dict) else {}
    out = {}
    for lid, rec in per.items():
        if not isinstance(rec, dict):
            continue
        fab = float(rec.get("fabrications", 0) or 0)
        dock = float(rec.get("modules_docked", 0) or 0)
        ch = float(rec.get("charge_collected_wh", 0) or 0)
        out[lid] = {
            "score": fab * E.SCORE_FAB + dock + ch / E.SCORE_CHARGE_DIV,
            "fabrications": int(fab), "modules_docked": int(dock),
            "charge_collected_wh": round(ch, 2), "deaths": rec.get("deaths", 0),
            "impacts": rec.get("impacts", 0), "distance_m": rec.get("distance_m", 0.0),
            "mean_lifespan_s": rec.get("mean_lifespan_s"),
            "best_genome": rec.get("best_genome"),
        }
    return out


def select_and_refill(lineages, scores, rng, epoch, next_n):
    """Keep the top ceil(K/2) by score (best genome carried), refill with
    mutate() of random keepers as NEW lineages."""
    ranked = sorted(lineages, key=lambda lin: -scores.get(lin["id"], {}).get("score", 0.0))
    n_keep = int(math.ceil(len(lineages) / 2.0))
    keepers = []
    for lin in ranked[:n_keep]:
        k = dict(lin)
        best = scores.get(lin["id"], {}).get("best_genome")
        if isinstance(best, dict) and not E.validate(E.clamp_genome(best)):
            k["genome"] = E.clamp_genome(best)
        keepers.append(k)
    nxt = list(keepers)
    while len(nxt) < len(lineages):
        parent = keepers[rng.randrange(len(keepers))]
        lid = "L%d" % next_n
        next_n += 1
        nxt.append({"id": lid, "genome": E.mutate(parent["genome"], rng),
                    "born_epoch": epoch + 1, "parent": parent["id"]})
    return nxt, ranked[:n_keep], next_n


# --------------------------------------------------------------------------
# one epoch
# --------------------------------------------------------------------------

def epoch_dir(epoch):
    return os.path.join(RUN, "epoch_%02d" % epoch)


def write_fleet(fleet, path=None):
    path = path or os.path.join(RUN, "fleet.json")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(fleet, f, indent=1)
    return path


def write_world(fleet, path, controller_world=CONTROLLER_WORLD):
    W = _worldgen()
    cfg = fleet["config"]
    lines = S.scene_lines(cfg["arena"], controller_world, pads=cfg["pads"], bay=cfg["bay"])
    os.makedirs(os.path.dirname(path), exist_ok=True)
    W.write_world(fleet["robots"], fleet["modules"], path,
                  scene_lines=lines, controller=CONTROLLER_ROBOT)


def engine_command(duration):
    return [sys.executable, "-m", "omnisim", "run-headless",
            os.path.relpath(WORLD, REPO), "--duration", str(duration)]


def run_epoch(epoch, duration):
    """Run ONE engine on the current world; return epoch_result or None."""
    edir = epoch_dir(epoch)
    os.makedirs(edir, exist_ok=True)
    result_path = os.path.join(RUN, "epoch_result.json")
    if os.path.exists(result_path):
        os.remove(result_path)
    log = os.path.join(edir, "engine.log")
    for suffix in ("", ".newton.json", ".stdout", ".stderr"):
        try:
            os.remove(log + suffix)
        except OSError:
            pass
    env = dict(os.environ)
    env["OMNISIM_LOG_PATH"] = log
    p = subprocess.run(engine_command(duration), cwd=REPO, env=env,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       text=True, timeout=duration + 180)
    with open(os.path.join(edir, "run_headless.out"), "w", encoding="utf-8") as f:
        f.write(p.stdout)
    if not os.path.exists(result_path):
        print("    ENGINE PRODUCED NO epoch_result.json (exit %d)" % p.returncode)
        print("    " + "\n    ".join(p.stdout.strip().splitlines()[-6:]))
        return None
    with open(result_path, encoding="utf-8") as f:
        return json.load(f)


def persist_epoch(epoch, lineages, scores, kept_ids):
    edir = epoch_dir(epoch)
    os.makedirs(edir, exist_ok=True)
    for name in ("fleet.json", "epoch_result.json", "telemetry.json"):
        src = os.path.join(RUN, name)
        if os.path.exists(src):
            shutil.copyfile(src, os.path.join(edir, name))
    if os.path.exists(WORLD):
        shutil.copyfile(WORLD, os.path.join(edir, os.path.basename(WORLD)))
    rows = []
    for lin in lineages:
        rec = dict(scores.get(lin["id"], {}))
        rec.update({"id": lin["id"], "kept": lin["id"] in kept_ids,
                    "born_epoch": lin.get("born_epoch", 0), "parent": lin.get("parent"),
                    "genome": lin["genome"]})
        rows.append(rec)
    with open(os.path.join(edir, "lineages.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump(rows, f, indent=1)


def _desc(g):
    return "v%.2f chg%.2f greed%.2f caut%.1f pref[%s]" % (
        g["cruise_speed"], g["charge_at"], g["greed"], g["caution"],
        " ".join("%s%.1f" % (t[0], g["module_pref"][t]) for t in E.MODULE_TYPES))


def print_table(epoch, lineages, scores, kept_ids, wall_s):
    print("  %-5s %8s %4s %5s %8s %6s %7s  %s" % (
        "lin", "score", "fab", "docks", "chg_wh", "deaths", "life_s", "verdict"))
    for lin in sorted(lineages, key=lambda l: -scores.get(l["id"], {}).get("score", 0.0)):
        r = scores.get(lin["id"], {})
        life = r.get("mean_lifespan_s")
        print("  %-5s %8.2f %4d %5d %8.1f %6d %7s  %s  %s" % (
            lin["id"], r.get("score", 0.0), r.get("fabrications", 0),
            r.get("modules_docked", 0), r.get("charge_collected_wh", 0.0),
            r.get("deaths", 0), ("%.0f" % life) if life is not None else "-",
            "KEEP" if lin["id"] in kept_ids else "cull", _desc(lin["genome"])))
    print("  wall %.0f s" % wall_s)


# --------------------------------------------------------------------------
# state
# --------------------------------------------------------------------------

def load_state():
    if not os.path.exists(STATE):
        return None
    with open(STATE, encoding="utf-8") as f:
        return json.load(f)


def save_state(state):
    os.makedirs(RUN, exist_ok=True)
    with open(STATE, "w", encoding="utf-8", newline="\n") as f:
        json.dump(state, f, indent=1)


def _tuple_to_list(t):
    return [_tuple_to_list(x) if isinstance(x, tuple) else x for x in t] \
        if isinstance(t, tuple) else t


def _untuple(x):
    if isinstance(x, list):
        return tuple(_untuple(v) for v in x)
    return x


# --------------------------------------------------------------------------
# dry run
# --------------------------------------------------------------------------

def dry_run(args, lineages, next_epoch, next_n, rng, have_worldgen):
    duration = args.epoch_s + DURATION_SLACK_S
    print("DRY RUN -- nothing is written, no engine is launched.")
    print("plan: %d epoch(s) starting at epoch %d; %d robot slots (%d alive at start), "
          "%d modules (%d loose), 3 pads, bay at (0, 0)" % (
              args.epochs, next_epoch, args.robots, args.alive, args.modules, args.loose))
    print("arena %g m, epoch %d s, seed %d, pads %s" % (
        args.arena, args.epoch_s, args.seed, S.default_pads(args.arena)))
    print("rl.worldgen [A]: %s" % ("present" if have_worldgen else
                                    "ABSENT -- a real run needs it; the plan below is unaffected"))
    print("rl.modules  [A]: %s" % E.module_source())
    for e in range(next_epoch, next_epoch + args.epochs):
        fleet = build_fleet(lineages, args, e, rng)
        print("\n=== epoch %d ===" % e)
        for rb in fleet["robots"]:
            if rb["alive_at_start"]:
                print("  slot %d  %-12s lineage %-4s at (%6.2f, %6.2f) yaw %5.2f  %s" % (
                    rb["slot"], rb["id"], rb["lineage"], rb["pos"][0], rb["pos"][1],
                    rb["yaw"], _desc(rb["genome"])))
            else:
                print("  slot %d  %-12s parked at crypt (%.1f, %.1f)" % (
                    rb["slot"], rb["id"], rb["pos"][0], rb["pos"][1]))
        loose = [m for m in fleet["modules"] if m["loose_at_start"]]
        parked = [m for m in fleet["modules"] if not m["loose_at_start"]]
        print("  modules: %d loose [%s], %d parked" % (
            len(loose), " ".join("%d:%s" % (m["id"], m["type"][:3]) for m in loose), len(parked)))
        print("  inputs : %s" % os.path.relpath(os.path.join(RUN, "fleet.json"), REPO))
        print("  world  : %s  (rl.worldgen.write_world(robots, modules, path, "
              "scene_lines=rl.scene.scene_lines(%g, %r, pads=..., bay=...), controller=%r))"
              % (os.path.relpath(WORLD, REPO), args.arena, CONTROLLER_WORLD, CONTROLLER_ROBOT))
        print("  log    : OMNISIM_LOG_PATH=%s" % os.path.relpath(
            os.path.join(epoch_dir(e), "engine.log"), REPO))
        print("  command: %s  (cwd %s)" % (" ".join(engine_command(duration)), REPO))
        print("  result : %s -> keep top %d of %d lineages, refill by mutate()" % (
            os.path.relpath(os.path.join(RUN, "epoch_result.json"), REPO),
            int(math.ceil(len(lineages) / 2.0)), len(lineages)))
        # Selection cannot be simulated without a result; pretend the first
        # half kept so the roster shape of the next epoch is visible.
        n_keep = int(math.ceil(len(lineages) / 2.0))
        keepers = lineages[:n_keep]
        nxt = list(keepers)
        while len(nxt) < len(lineages):
            # "*" marks a lineage that does not exist yet: which keeper it
            # descends from is decided by the real scores.
            nxt.append({"id": "L%d*" % next_n, "genome": E.mutate(keepers[0]["genome"], rng),
                        "born_epoch": e + 1, "parent": keepers[0]["id"]})
            next_n += 1
        lineages = nxt


# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------

def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--epochs", type=int, default=6)
    ap.add_argument("--robots", type=int, default=6, help="pooled robot slots")
    ap.add_argument("--alive", type=int, default=4, help="robots alive at epoch start")
    ap.add_argument("--modules", type=int, default=14, help="pooled modules")
    ap.add_argument("--loose", type=int, default=10, help="modules loose at epoch start")
    ap.add_argument("--epoch-s", type=int, default=240, help="simulated seconds per epoch")
    ap.add_argument("--arena", type=float, default=24.0, help="floor side S in metres")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--resume", action="store_true", help="continue from _run/robolife/state.json")
    ap.add_argument("--dry-run", action="store_true", help="print the plan; write nothing, run nothing")
    ap.add_argument("--write-seeds", action="store_true",
                    help="write projects/robolife/seeds/fleet.json (epoch 0 fleet) and stop")
    args = ap.parse_args(argv)
    if args.alive > args.robots:
        ap.error("--alive cannot exceed --robots")
    if args.loose > args.modules:
        ap.error("--loose cannot exceed --modules")
    return args


def main(argv=None):
    args = parse_args(argv)
    try:
        _worldgen()
        have_worldgen = True
    except ImportError as exc:
        have_worldgen = False
        if not (args.dry_run or args.write_seeds):
            sys.exit("cannot import rl.worldgen (%s) -- required for a real run" % exc)
        print("note: rl.worldgen not importable (%s)" % exc)

    rng = random.Random(args.seed)
    state = load_state() if args.resume else None
    if state:
        lineages = state["lineages"]
        next_epoch = state["next_epoch"]
        next_n = state["next_lineage_n"]
        if state.get("rng"):
            rng.setstate(_untuple(state["rng"]))
        print("resumed: %d lineages at epoch %d" % (len(lineages), next_epoch))
    else:
        lineages = initial_lineages(args.alive, rng)
        next_epoch, next_n = 0, args.alive

    if args.write_seeds:
        fleet = build_fleet(lineages, args, 0, rng)
        path = write_fleet(fleet, SEEDS)
        print("wrote %s (%d robots, %d modules)" % (os.path.relpath(path, REPO),
                                                     len(fleet["robots"]), len(fleet["modules"])))
        return
    if args.dry_run:
        dry_run(args, lineages, next_epoch, next_n, rng, have_worldgen)
        return

    duration = args.epoch_s + DURATION_SLACK_S
    t_start = time.time()
    for epoch in range(next_epoch, next_epoch + args.epochs):
        print("\n=== epoch %d  (%d slots, %d alive, %d modules, %d s sim) ===" % (
            epoch, args.robots, args.alive, args.modules, args.epoch_s))
        t0 = time.time()
        fleet = build_fleet(lineages, args, epoch, rng)
        write_fleet(fleet)
        write_world(fleet, WORLD)

        result = run_epoch(epoch, duration)
        if result is None:
            persist_epoch(epoch, lineages, {}, set())
            print("  epoch failed; stopping (state NOT advanced -- --resume re-runs it)")
            break

        scores = score_lineages(result)
        nxt, kept, next_n = select_and_refill(lineages, scores, rng, epoch, next_n)
        kept_ids = {lin["id"] for lin in kept}
        persist_epoch(epoch, lineages, scores, kept_ids)
        print_table(epoch, lineages, scores, kept_ids, time.time() - t0)

        lineages = nxt
        save_state({"next_epoch": epoch + 1, "next_lineage_n": next_n,
                    "lineages": lineages, "rng": _tuple_to_list(rng.getstate()),
                    "args": vars(args)})

    print("\n=== done (%.0f s total); roster for the next epoch: %s ===" % (
        time.time() - t_start, ", ".join(lin["id"] for lin in lineages)))


if __name__ == "__main__":
    main()
