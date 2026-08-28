#!/usr/bin/env python3
"""Epoch driver for the living ecosystem (DESIGN_v2.md, "Epoch driver").

Each epoch:
  1. build the population: K species x M slots, `alive` per species alive at
     start, every slot's brain = the species' best brain from the previous
     epoch, mutated (`mutate_brain`)
  2. write _run/life/population.json + config.json, regenerate the world
     (worldgen2.write_world + scene.scene_lines)
  3. run ONE headless engine; the terrarium_life director runs the ecology
     for `epoch_s` of sim time and writes _run/life/epoch_result.json
  4. score every species = births*10 + eaten + mean_lifespan_s/10, keep the
     top ceil(K/2), refill with `mutate_body` of the keepers (new species ids)

MORPHOLOGY EVOLVES because the world is REGENERATED every epoch -- runtime
spawn has no physics, so a body plan can only be created at load. Brains
evolve INSIDE an epoch (offspring inherit a mutated brain); the driver only
carries each species' best brain across the boundary.

Exactly ONE engine process runs at a time (thermal limit; and `run-headless`
reads a shared log unless OMNISIM_LOG_PATH is unique, which it is here).

  python projects/alife/ecosystem.py --epochs 6 --species 4 --slots 4 \
      --alive 2 --epoch-s 120 --arena 14 --food-pool 24 --food-active 12
  python projects/alife/ecosystem.py --dry-run --epochs 2     # plan only
  python projects/alife/ecosystem.py --resume --epochs 3      # continue

Persisted under _run/life/epoch_NN/: population.json, config.json, the
world, epoch_result.json, species.json (scores + verdicts) and the engine
log; _run/life/state.json carries the species roster between invocations
(that is what --resume reads).
"""
import argparse
import copy
import json
import math
import os
import random
import shutil
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

ROOT = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(ROOT, "..", ".."))
RUN = os.path.join(ROOT, "_run", "life")
WORLD = os.path.join(ROOT, "worlds", "terrarium_life.omniworld")
STATE = os.path.join(RUN, "state.json")
CONTROLLER = "terrarium_life"
FOOD_RESPAWN_S = [4, 9]
# The director quits at epoch_s; the extra 60 s covers load + finalize on a
# cold disk. `run-headless --duration` is a wall-clock SLEEP, so the director's
# simulationQuit(0) is what actually ends the run early.
DURATION_SLACK_S = 60


# --------------------------------------------------------------------------
# genome2 / worldgen2 seam (both owned by implementer A)
#
# Imported lazily so `--dry-run` and `--help` work before the modules land.
# The calls this driver makes, exactly (signatures as in alife/genome2.py):
#
#   genome2.seed_species(rng, k)                    -> [genome] x k, species sp0..sp{k-1}
#   genome2.mutate_body(genome, rng, new_species_id) -> genome (between epochs)
#   genome2.mutate_brain(genome, rng, gid)           -> genome (per slot / at birth)
#   genome2.validate(genome)                         -> list of problems ([] = ok)
#   genome2.describe(genome)                         -> str (tables only)
#   worldgen2.write_world(pop, path, scene_lines=<list of str>, controller=str)
#
# The driver re-stamps `id` / `species` / `parent` on every genome it emits
# (contract ids: "<species>_g<epoch>_<slot>"), so the mutators' own naming
# is never what lands in population.json.
# --------------------------------------------------------------------------

def _import(name):
    import importlib
    return importlib.import_module("alife." + name)


def _brain_child(G2, parent, rng, gid, tries=12):
    """mutate_brain until validate() is clean. The engine refuses what
    validate rejects, and one bad genome costs the whole epoch (no physics
    at all), so an unlucky mutation falls back to the unmutated parent."""
    for _ in range(tries):
        child = G2.mutate_brain(parent, rng, gid)
        if not G2.validate(child):
            return child
    return copy.deepcopy(parent)


def _body_child(G2, parent, rng, sid, tries=12):
    for _ in range(tries):
        child = G2.mutate_body(parent, rng, sid)
        if not G2.validate(child):
            return child
    return copy.deepcopy(parent)


def _stamp(g, gid, species, parent):
    g["id"] = gid
    g["species"] = species
    g["parent"] = parent
    return g


# --------------------------------------------------------------------------
# population
# --------------------------------------------------------------------------

def build_population(species, slots, alive, epoch, rng, G2=None):
    """K species x M slots -> list of genome dicts with `slot` and
    `alive_at_start`. Each species record carries its founder genome (the
    body) and `best_brain` (None before the first epoch)."""
    pop = []
    for k, sp in enumerate(species):
        founder = sp["founder"]
        seed = copy.deepcopy(founder)
        if sp.get("best_brain"):
            seed["brain"] = copy.deepcopy(sp["best_brain"])
        for m in range(slots):
            gid = "%s_g%d_%02d" % (sp["id"], epoch, m)
            if G2 is None or m == 0:
                g = copy.deepcopy(seed)             # slot 0: one unmutated carrier
            else:
                g = _brain_child(G2, seed, rng, gid)
            _stamp(g, gid, sp["id"], seed.get("id"))
            g["slot"] = k * slots + m
            g["alive_at_start"] = m < alive
            pop.append(g)
    return pop


def initial_species(n, rng, G2):
    """seed_species hands back sp0..sp{n-1}, hues evenly spaced, all valid."""
    species = []
    for g in G2.seed_species(rng, n):
        sid = g["species"]
        _stamp(g, "%s_founder" % sid, sid, None)
        species.append({"id": sid, "founder": g, "best_brain": None,
                        "born_epoch": 0, "parent_species": None})
    return species


def score_species(result):
    """`epoch_result.json` -> {species_id: {score, births, ...}}.

    Tolerates both `{"species": {sp0: {...}}}` and a bare `{sp0: {...}}`
    top level; missing counters score as zero rather than crashing the run."""
    per = result.get("species", result) if isinstance(result, dict) else {}
    out = {}
    for sid, rec in per.items():
        if not isinstance(rec, dict):
            continue
        births = float(rec.get("births", 0) or 0)
        eaten = float(rec.get("eaten", 0) or 0)
        life = float(rec.get("mean_lifespan_s", 0) or 0)
        out[sid] = {
            "score": births * 10.0 + eaten + life / 10.0,
            "births": births, "deaths": rec.get("deaths", 0),
            "eaten": eaten, "peak_pop": rec.get("peak_pop", 0),
            "mean_lifespan_s": life,
            "best_brain": _brain_of(rec.get("best_brain")),
        }
    return out


def _brain_of(rec):
    """best_brain may be written as a bare brain dict or as a whole genome."""
    if isinstance(rec, dict) and "brain" in rec and isinstance(rec["brain"], dict):
        return rec["brain"]
    return rec if isinstance(rec, dict) else None


def select_and_refill(species, scores, rng, epoch, next_n, G2):
    """Keep the top ceil(K/2) by score, refill with mutate_body of keepers."""
    ranked = sorted(species, key=lambda sp: -scores.get(sp["id"], {}).get("score", 0.0))
    n_keep = int(math.ceil(len(species) / 2.0))
    keepers = []
    for sp in ranked[:n_keep]:
        k = dict(sp)
        rec = scores.get(sp["id"], {})
        if rec.get("best_brain"):
            k["best_brain"] = rec["best_brain"]
        keepers.append(k)

    nxt = list(keepers)
    while len(nxt) < len(species):
        parent = keepers[rng.randrange(len(keepers))]
        sid = "sp%d" % next_n
        next_n += 1
        seed = copy.deepcopy(parent["founder"])
        if parent.get("best_brain"):
            seed["brain"] = copy.deepcopy(parent["best_brain"])
        child = _body_child(G2, seed, rng, sid)
        _stamp(child, "%s_founder" % sid, sid, parent["founder"]["id"])
        nxt.append({"id": sid, "founder": child, "best_brain": child.get("brain"),
                    "born_epoch": epoch + 1, "parent_species": parent["id"]})
    return nxt, ranked[:n_keep], next_n


# --------------------------------------------------------------------------
# one epoch
# --------------------------------------------------------------------------

def epoch_dir(epoch):
    return os.path.join(RUN, "epoch_%02d" % epoch)


def write_inputs(pop, config):
    os.makedirs(RUN, exist_ok=True)
    with open(os.path.join(RUN, "population.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump(pop, f, indent=1)
    with open(os.path.join(RUN, "config.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump(config, f, indent=1)


def engine_command(duration):
    return [sys.executable, "-m", "omnisim", "run-headless",
            os.path.relpath(WORLD, REPO), "--duration", str(duration)]


def run_epoch(epoch, duration, quiet=True):
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
    cmd = engine_command(duration)
    p = subprocess.run(cmd, cwd=REPO, env=env,
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


def persist_epoch(epoch, species, scores, kept_ids):
    edir = epoch_dir(epoch)
    os.makedirs(edir, exist_ok=True)
    for name in ("population.json", "config.json", "epoch_result.json", "telemetry.json"):
        src = os.path.join(RUN, name)
        if os.path.exists(src):
            shutil.copyfile(src, os.path.join(edir, name))
    if os.path.exists(WORLD):
        shutil.copyfile(WORLD, os.path.join(edir, os.path.basename(WORLD)))
    rows = []
    for sp in species:
        rec = dict(scores.get(sp["id"], {}))
        rec["id"] = sp["id"]
        rec["kept"] = sp["id"] in kept_ids
        rec["born_epoch"] = sp.get("born_epoch", 0)
        rec["parent_species"] = sp.get("parent_species")
        rows.append(rec)
    with open(os.path.join(edir, "species.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump(rows, f, indent=1)


def print_table(epoch, species, scores, kept_ids, wall_s, G2=None):
    print("  %-6s %8s %7s %7s %6s %5s %9s  %s" % (
        "species", "score", "births", "deaths", "eaten", "peak", "life_s", "verdict"))
    for sp in sorted(species, key=lambda s: -scores.get(s["id"], {}).get("score", 0.0)):
        r = scores.get(sp["id"], {})
        desc = ""
        if G2 is not None and hasattr(G2, "describe"):
            try:
                desc = "  " + G2.describe(sp["founder"])
            except Exception:       # describe is cosmetic; never fail an epoch on it
                desc = ""
        print("  %-6s %8.2f %7d %7d %6d %5d %9.1f  %s%s" % (
            sp["id"], r.get("score", 0.0), r.get("births", 0), r.get("deaths", 0),
            r.get("eaten", 0), r.get("peak_pop", 0), r.get("mean_lifespan_s", 0.0),
            "KEEP" if sp["id"] in kept_ids else "cull", desc))
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


def make_config(args, epoch, watch=False):
    return {
        "arena": args.arena,
        "food_pool": args.food_pool,
        "food_active_max": args.food_active,
        "food_respawn_s": list(FOOD_RESPAWN_S),
        "epoch_s": args.epoch_s,
        "watch": bool(watch),
        "epoch": epoch,
        "species": args.species,
        "slots": args.slots,
        "alive": args.alive,
        "seed": args.seed,
    }


# --------------------------------------------------------------------------
# dry run
# --------------------------------------------------------------------------

def dry_run(args, species, next_epoch, next_n, G2, rng):
    duration = args.epoch_s + DURATION_SLACK_S
    print("DRY RUN -- nothing is written, no engine is launched.")
    print("plan: %d epoch(s) starting at epoch %d; %d species x %d slots = %d creatures, "
          "%d/species alive at start" % (args.epochs, next_epoch, args.species, args.slots,
                                          args.species * args.slots, args.alive))
    print("arena %g m, food pool %d, food active <= %d, respawn %s s, epoch %d s"
          % (args.arena, args.food_pool, args.food_active, FOOD_RESPAWN_S, args.epoch_s))
    print("genome2: %s" % ("present" if G2 is not None else
                           "ABSENT -- population shown with placeholder ids"))
    if species is None:
        species = [{"id": "sp%d" % k, "founder": {"id": "sp%d_founder" % k, "body": {},
                                                   "brain": {}}, "best_brain": None}
                   for k in range(args.species)]
    for e in range(next_epoch, next_epoch + args.epochs):
        pop = build_population(species, args.slots, args.alive, e, rng, G2)
        print("\n=== epoch %d ===" % e)
        for sp in species:
            ids = [g["id"] for g in pop if g["species"] == sp["id"]]
            alive = sum(1 for g in pop if g["species"] == sp["id"] and g["alive_at_start"])
            desc = ""
            if G2 is not None and hasattr(G2, "describe"):
                try:
                    desc = "  " + G2.describe(sp["founder"])
                except Exception:
                    desc = ""
            print("  %-5s slots %s  alive %d/%d  brain %s%s" % (
                sp["id"], "%d..%d" % (min(g["slot"] for g in pop if g["species"] == sp["id"]),
                                      max(g["slot"] for g in pop if g["species"] == sp["id"])),
                alive, len(ids), "carried" if sp.get("best_brain") else "founder", desc))
        print("  inputs : %s" % os.path.relpath(os.path.join(RUN, "population.json"), REPO))
        print("           %s" % os.path.relpath(os.path.join(RUN, "config.json"), REPO))
        print("  world  : %s" % os.path.relpath(WORLD, REPO))
        print("  log    : OMNISIM_LOG_PATH=%s" % os.path.relpath(
            os.path.join(epoch_dir(e), "engine.log"), REPO))
        print("  command: %s  (cwd %s)" % (" ".join(engine_command(duration)), REPO))
        print("  result : %s -> keep top %d of %d, refill by mutate_body" % (
            os.path.relpath(os.path.join(RUN, "epoch_result.json"), REPO),
            int(math.ceil(args.species / 2.0)), args.species))
        # Selection cannot be simulated without a result; pretend every keeper
        # is the first half so the roster shape of the next epoch is visible.
        species = [dict(sp, best_brain=sp["founder"].get("brain") or {"carried": True})
                   for sp in species[:int(math.ceil(args.species / 2.0))]]
        n_new = next_n + (e - next_epoch) * (args.species - len(species))
        while len(species) < args.species:
            # "*" marks a species that does not exist yet: which keeper it
            # descends from is decided by the real scores.
            species.append({"id": "sp%d*" % n_new, "founder": species[0]["founder"],
                            "best_brain": None})
            n_new += 1


# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--epochs", type=int, default=6)
    ap.add_argument("--species", type=int, default=4, help="K species per epoch")
    ap.add_argument("--slots", type=int, default=4, help="M pooled slots per species")
    ap.add_argument("--alive", type=int, default=2, help="slots alive at epoch start")
    ap.add_argument("--epoch-s", type=int, default=120, help="simulated seconds per epoch")
    ap.add_argument("--arena", type=float, default=14.0, help="floor side S in metres")
    ap.add_argument("--food-pool", type=int, default=24)
    ap.add_argument("--food-active", type=int, default=12)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--resume", action="store_true",
                    help="continue from _run/life/state.json")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the plan; write nothing, run nothing")
    args = ap.parse_args()
    if args.alive > args.slots:
        ap.error("--alive cannot exceed --slots")

    # genome2 is wanted even by --dry-run (for the population summary);
    # worldgen2 only by a real run. Each is reported separately when missing.
    G2 = W2 = S = None
    for name in ("genome2", "worldgen2", "scene"):
        try:
            mod = _import(name)
        except ImportError as e:
            if not args.dry_run:
                sys.exit("cannot import alife.%s (%s) -- required for a real run" % (name, e))
            print("note: alife.%s not importable (%s)" % (name, e))
            continue
        if name == "genome2":
            G2 = mod
        elif name == "worldgen2":
            W2 = mod
        else:
            S = mod

    rng = random.Random(args.seed)
    state = load_state() if args.resume else None
    if state:
        species = state["species"]
        next_epoch = state["next_epoch"]
        next_n = state["next_species_n"]
        rng.setstate(tuple(_untuple(state["rng"])) if state.get("rng") else rng.getstate())
        print("resumed: %d species at epoch %d" % (len(species), next_epoch))
    else:
        species = initial_species(args.species, rng, G2) if G2 is not None else None
        next_epoch, next_n = 0, args.species

    if args.dry_run:
        dry_run(args, species, next_epoch, next_n, G2, rng)
        return

    duration = args.epoch_s + DURATION_SLACK_S
    t_start = time.time()
    for epoch in range(next_epoch, next_epoch + args.epochs):
        print("\n=== epoch %d  (%d species x %d slots, %d s sim) ===" % (
            epoch, len(species), args.slots, args.epoch_s))
        t0 = time.time()
        pop = build_population(species, args.slots, args.alive, epoch, rng, G2)
        write_inputs(pop, make_config(args, epoch))
        W2.write_world(pop, WORLD,
                       scene_lines=S.scene_lines(args.arena, args.food_pool, CONTROLLER),
                       controller=CONTROLLER)

        result = run_epoch(epoch, duration)
        if result is None:
            persist_epoch(epoch, species, {}, set())
            print("  epoch failed; stopping (state NOT advanced -- --resume re-runs it)")
            break

        scores = score_species(result)
        nxt, kept, next_n = select_and_refill(species, scores, rng, epoch, next_n, G2)
        kept_ids = {sp["id"] for sp in kept}
        persist_epoch(epoch, species, scores, kept_ids)
        print_table(epoch, species, scores, kept_ids, time.time() - t0, G2)

        species = nxt
        save_state({"next_epoch": epoch + 1, "next_species_n": next_n,
                    "species": species, "rng": _tuple_to_list(rng.getstate()),
                    "args": vars(args)})

    print("\n=== done (%.0f s total); roster for the next epoch: %s ===" % (
        time.time() - t_start, ", ".join(sp["id"] for sp in species)))


def _tuple_to_list(t):
    return [_tuple_to_list(x) if isinstance(x, tuple) else x for x in t] \
        if isinstance(t, tuple) else t


def _untuple(x):
    """random.getstate() is nested tuples; JSON turns them into lists."""
    if isinstance(x, list):
        return tuple(_untuple(v) for v in x)
    return x


if __name__ == "__main__":
    main()
