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

"""Epoch driver for Metazoa (PLAN.md P4, DESIGN.md "Reef + surface + driver").

Each epoch:
  1. build the reef: `--organisms` two-cell organisms (one lineage each,
     genome + body plan carried from the previous epoch, mutated), 12 free
     cells, and every remaining cell of `--cells` PARKED on the crypt slab
     (cell count is conserved -- runtime spawn has no physics, so every cell
     exists from load)
  2. write _run/metazoa/reef.json + config.json, regenerate the world
     (`mz.worldgen.write_world` + `mz.scene.scene_lines`)
  3. check the GPU temperature (never launch above 70 C), run ONE headless
     engine with `OMNISIM_LOG_PATH` under _run/metazoa/epoch_NN/; the
     `metazoa_world` director runs the ecology for `--epoch-s` of sim time
     and writes _run/metazoa/epoch_result.json
  4. score every lineage (DESIGN: divisions*10 + cells recruited + light
     collected/10 + mean organism length), keep the top half, refill by
     mutating keepers (new lineage ids), persist

Exactly ONE engine process runs at a time (thermal limit; and run-headless
reads a shared log unless OMNISIM_LOG_PATH is unique, which it is here).

  python projects/metazoa/metazoa.py --epochs 8 --cells 24 --organisms 6 \
      --epoch-s 240 --arena 18 --seed 7
  python projects/metazoa/metazoa.py --dry-run --epochs 2     # plan only
  python projects/metazoa/metazoa.py --resume --epochs 3      # continue

Persisted under _run/metazoa/epoch_NN/: reef.json, config.json, the world,
epoch_result.json, lineages.json (scores + verdicts) and the engine log;
_run/metazoa/state.json carries the lineage roster between invocations
(that is what --resume reads).  `METAZOA_RUN_DIR` relocates _run/metazoa
(tests use it).

--------------------------------------------------------------------------
Seams to A / B / C (imported lazily so `--dry-run` and `--help` work before
the modules land; each absent module is reported once and stubbed):

  mz.cell.chain_poses(head_pose, n, gap=0.01, dock_rotations=None)
      -> n dicts {"pos": [x, y, z], "yaw", "roll"}; cell k sits at head +
         R_head*(k*(0.12+gap), 0, 0), rolled 90 deg * dock_rotations[k]
         (relative to the HEAD).  Also mz.cell.SPAWN_Z.  [A]  Fallback: the
         same geometry, written here from DESIGN "Docking geometry".
  mz.worldgen.write_world(cells, path, scene_lines=<list of str>, controller=str)
      cells = reef["cells"]: [{id, pos, yaw, roll, parked, organism,
      dock_rotation, charge_wh}] -- worldgen reads id/pos/yaw/roll/parked
      (+ optional colour/ring/rotation) and ignores the rest.  [A]
  mz.organism.random_genome(rng) / mutate(genome, rng) / validate(genome)
      -> [] when ok; random_bodyplan(rng) / mutate_bodyplan(bp, rng).  [B]
  _run/metazoa/epoch_result.json  (written by the director, C's
      ecology.Reef.epoch_result()): {"lineages": {lid: {divisions, recruited,
      light_wh, mean_length, best_genome?, best_bodyplan?, ...}}} or bare.
--------------------------------------------------------------------------
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

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT)

REPO = os.path.normpath(os.path.join(ROOT, "..", ".."))
RUN = os.environ.get("METAZOA_RUN_DIR") or os.path.join(ROOT, "_run", "metazoa")
WORLD = os.path.join(ROOT, "worlds", "metazoa.omniworld")
STATE = os.path.join(RUN, "state.json")
CONTROLLER = "metazoa_world"
N_PATCHES = 5
FREE_CELLS = 12
CELL_LEN = 0.12            # two 6 cm blocks
CELL_Z = 0.03 + 0.005      # half a block + drop clearance
DOCK_GAP = 0.01
CHARGE_START_WH = 6.0
CHARGE_START_WH = float(os.environ.get("METAZOA_START_WH", CHARGE_START_WH))   # diagnostic override
# The director quits at epoch_s; the extra 45 s covers load + finalize on a
# cold disk. `run-headless --duration` is a wall-clock SLEEP, so the
# director's simulationQuit(0) is what actually ends the run early.
DURATION_SLACK_S = 45
GPU_LIMIT_C = 70.0
GPU_POLL_S = 60.0


def _import(name):
    import importlib
    return importlib.import_module("mz." + name)


def _rel(path):
    """relpath for printing; falls back to the absolute path when the two
    are on different Windows drives (relpath raises there)."""
    try:
        return os.path.relpath(path, REPO)
    except ValueError:
        return os.path.abspath(path)


def load_modules(required, note=print):
    """{"cell": mod|None, "worldgen": ..., "organism": ..., "ecology": ...,
    "scene": ...}.  Missing modules are None with one note each; when
    `required` they abort instead."""
    mods = {}
    for name in ("cell", "worldgen", "organism", "ecology", "scene"):
        try:
            mods[name] = _import(name)
        except ImportError as e:
            if required and name in ("worldgen", "scene"):
                sys.exit("cannot import mz.%s (%s) -- required for a real run" % (name, e))
            note("note: mz.%s not importable (%s) -- using a stub" % (name, e))
            mods[name] = None
    return mods


# --------------------------------------------------------------------------
# genome / body plan (B's seam, with stubs)
# --------------------------------------------------------------------------

STUB_GENOME = {"A": 0.8, "omega": 5.0, "dphi": 1.57, "bias_pitch": 0.0, "bias_yaw": 0.0,
               "branch_phase": 0.0, "branch_scale": 0.5, "steer_gain": 0.3}
STUB_BODYPLAN = {"target_length": 4, "dock_rotation_pattern": [0, 1], "branch_rule": "none"}


def random_genome(ORG, rng):
    if ORG is not None and hasattr(ORG, "random_genome"):
        return ORG.random_genome(rng)
    g = dict(STUB_GENOME)
    g["A"] = round(rng.uniform(0.3, 1.2), 3)
    g["omega"] = round(rng.uniform(2.0, 8.0), 3)
    g["dphi"] = round(rng.uniform(0.6, 2.4), 3)
    return g


def random_bodyplan(ORG, rng):
    if ORG is not None and hasattr(ORG, "random_bodyplan"):
        return ORG.random_bodyplan(rng)
    bp = copy.deepcopy(STUB_BODYPLAN)
    bp["target_length"] = rng.randint(2, 8)
    return bp


def mutate_genome(ORG, genome, rng, tries=12):
    """mutate until validate() is clean; an unlucky mutation falls back to
    the unmutated parent (one bad genome must never cost an epoch)."""
    if ORG is None or not hasattr(ORG, "mutate"):
        g = dict(genome)
        g["A"] = round(min(1.2, max(0.3, g.get("A", 0.8) + rng.gauss(0, 0.1))), 3)
        return g
    validate = getattr(ORG, "validate", lambda g: [])
    for _ in range(tries):
        child = ORG.mutate(genome, rng)
        if not validate(child):
            return child
    return copy.deepcopy(genome)


def mutate_bodyplan(ORG, bp, rng):
    if ORG is not None and hasattr(ORG, "mutate_bodyplan"):
        return ORG.mutate_bodyplan(bp, rng)
    child = copy.deepcopy(bp)
    if rng.random() < 0.3:
        child["target_length"] = min(8, max(2, child.get("target_length", 4) + rng.choice((-1, 1))))
    return child


# --------------------------------------------------------------------------
# placement (A's seam, with the DESIGN fallback)
# --------------------------------------------------------------------------

def spawn_z(CELL=None):
    return float(getattr(CELL, "SPAWN_Z", CELL_Z)) if CELL is not None else CELL_Z


def chain_placement_fallback(head_xy, yaw, n, dock_rotations, gap=DOCK_GAP, z=CELL_Z):
    """DESIGN 'Docking geometry', A's convention: cell k's origin at the
    head's origin + R_head*(k*(0.12 + gap), 0, 0), same yaw, rolled by
    90 deg * dock_rotations[k] (relative to the head) so hinge axes can
    alternate pitch / yaw."""
    out = []
    x, y = float(head_xy[0]), float(head_xy[1])
    dx, dy = math.cos(yaw), math.sin(yaw)
    for k in range(int(n)):
        rot = int(dock_rotations[k]) % 4 if dock_rotations and k < len(dock_rotations) else 0
        out.append({"pos": [round(x + k * (CELL_LEN + gap) * dx, 4),
                            round(y + k * (CELL_LEN + gap) * dy, 4), z],
                    "yaw": round(yaw, 4), "roll": round(rot * math.pi / 2.0, 4),
                    "dock_rotation": rot})
    return out


def chain_placement(CELL, head_xy, yaw, n, dock_rotations, gap=DOCK_GAP, note=print):
    """A's `chain_poses` if present; a TypeError from a different signature
    falls back to the DESIGN geometry, noted once per call."""
    if CELL is not None and hasattr(CELL, "chain_poses"):
        try:
            raw = CELL.chain_poses((head_xy[0], head_xy[1], spawn_z(CELL), yaw, 0.0), n,
                                   gap=gap, dock_rotations=list(dock_rotations or []))
            return [_norm_placement(p, k, dock_rotations) for k, p in enumerate(raw)]
        except TypeError as e:
            note("note: mz.cell.chain_poses signature differs (%s) -- fallback geometry" % e)
    return chain_placement_fallback(head_xy, yaw, n, dock_rotations, gap, spawn_z(CELL))


def _norm_placement(p, k, dock_rotations):
    if isinstance(p, dict):
        d = dict(p)
        if "pos" not in d:
            d["pos"] = [d.pop("x", 0.0), d.pop("y", 0.0), d.pop("z", CELL_Z)]
    else:                                   # tolerate (x, y, z, yaw, roll) tuples
        p = list(p) + [0.0] * 5
        d = {"pos": [p[0], p[1], p[2]], "yaw": p[3], "roll": p[4]}
    d["pos"] = [round(float(v), 4) for v in d["pos"]]
    d["yaw"] = round(float(d.get("yaw", 0.0)), 4)
    d["roll"] = round(float(d.get("roll", 0.0)), 4)
    d.setdefault("dock_rotation",
                 int(dock_rotations[k]) % 4 if dock_rotations and k < len(dock_rotations) else 0)
    return d


def park_position(k, z=CELL_Z):
    """Crypt slab x 57..93, y 57..63 (mz.scene.CRYPT_*): 24 slots per row at
    1.5 m pitch, three rows."""
    row, col = k // 24, k % 24
    return [58.0 + 1.5 * col, 60.0 + 1.5 * ((row % 3) - 1), z]


# --------------------------------------------------------------------------
# the reef
# --------------------------------------------------------------------------

def initial_lineages(n, rng, ORG):
    return [{"id": "L%d" % k, "genome": random_genome(ORG, rng),
             "bodyplan": random_bodyplan(ORG, rng), "born_epoch": 0, "parent": None}
            for k in range(int(n))]


def build_reef(lineages, n_cells, arena, epoch, rng, mods=None, n_free=FREE_CELLS, seed_len=4,
               note=print):
    """Lineages -> reef dict: `organisms` (two cells each, one per lineage),
    `free` cells scattered inside the EDGE frame, the rest `parked` on the
    crypt.  Cell ids are 0..n_cells-1, every cell exactly once (conserved)."""
    mods = mods or {}
    CELL, SCN = mods.get("cell"), mods.get("scene")
    if SCN is not None:
        inner = SCN.edge_half_side(arena)
    else:
        inner = float(arena) / 2.0 - 0.2 - 0.6
    n_org = len(lineages)
    seed_len = max(1, int(seed_len))
    if seed_len * n_org > int(n_cells):
        raise ValueError("%d organisms x %d cells need %d cells, only %d"
                         % (n_org, seed_len, seed_len * n_org, n_cells))
    n_free = max(0, min(int(n_free), int(n_cells) - seed_len * n_org))

    cells, organisms = [], []
    next_id = 0
    r_ring = min(float(arena) * 0.22, inner - 1.0)
    for k, ln in enumerate(lineages):
        a = 2.0 * math.pi * k / max(1, n_org)
        head = (r_ring * math.cos(a), r_ring * math.sin(a))
        yaw = a + math.pi / 2.0                          # tangential heading
        pattern = ln["bodyplan"].get("dock_rotation_pattern") or [0]
        # cycle the body plan pattern over the whole chain: chain_placement does not,
        # and [0, 1] on four cells came out pitch/yaw/pitch/PITCH (measured)
        pattern_n = [pattern[k % len(pattern)] for k in range(seed_len)] if pattern else [0] * seed_len
        placed = chain_placement(CELL, head, yaw, seed_len, pattern_n, note=note)
        oid = "%s_e%d" % (ln["id"], epoch)
        members = []
        for p in placed:
            c = dict(p, id=next_id, organism=oid, parked=False, charge_wh=CHARGE_START_WH)
            cells.append(c)
            members.append(next_id)
            next_id += 1
        organisms.append({"id": oid, "lineage": ln["id"], "members": members,
                          "genome": copy.deepcopy(ln["genome"]),
                          "bodyplan": copy.deepcopy(ln["bodyplan"]),
                          "parent": ln.get("parent")})

    free = []
    taken = [(c["pos"][0], c["pos"][1]) for c in cells]
    lim = inner - 0.5
    for _ in range(n_free):
        for _try in range(200):
            x, y = rng.uniform(-lim, lim), rng.uniform(-lim, lim)
            if all((x - tx) ** 2 + (y - ty) ** 2 > 0.6 ** 2 for tx, ty in taken):
                break
        taken.append((x, y))
        cells.append({"id": next_id, "pos": [round(x, 4), round(y, 4), spawn_z(CELL)],
                      "yaw": round(rng.uniform(-math.pi, math.pi), 4), "roll": 0.0,
                      "dock_rotation": 0, "organism": None, "parked": False,
                      "charge_wh": CHARGE_START_WH})
        free.append(next_id)
        next_id += 1

    parked = []
    k = 0
    while next_id < int(n_cells):
        cells.append({"id": next_id, "pos": park_position(k, spawn_z(CELL)), "yaw": 0.0,
                      "roll": 0.0, "dock_rotation": 0, "organism": None, "parked": True,
                      "charge_wh": 0.0})
        parked.append(next_id)
        next_id += 1
        k += 1

    return {"epoch": int(epoch), "arena": float(arena), "n_cells": int(n_cells),
            "cells": cells, "organisms": organisms, "free": free, "parked": parked}


def check_conserved(reef):
    ids = sorted(c["id"] for c in reef["cells"])
    if ids != list(range(reef["n_cells"])):
        raise AssertionError("cell ids not 0..%d once each: %s" % (reef["n_cells"] - 1, ids))
    docked = [i for o in reef["organisms"] for i in o["members"]]
    if len(set(docked)) != len(docked):
        raise AssertionError("a cell belongs to two organisms")
    if len(docked) + len(reef["free"]) + len(reef["parked"]) != reef["n_cells"]:
        raise AssertionError("docked + free + parked != n_cells")
    return True


# --------------------------------------------------------------------------
# scoring / selection
# --------------------------------------------------------------------------

def score_lineages(result):
    """`epoch_result.json` -> {lineage_id: {score, ...}} (DESIGN: divisions*10
    + recruited + light/10 + mean organism length).  Tolerates a bare dict
    and missing counters (scored as zero)."""
    per = result.get("lineages", result) if isinstance(result, dict) else {}
    out = {}
    for lid, rec in per.items():
        if not isinstance(rec, dict):
            continue
        divisions = float(rec.get("divisions", 0) or 0)
        recruited = float(rec.get("recruited", rec.get("cells_recruited", 0)) or 0)
        light = float(rec.get("light_wh", rec.get("light_collected", 0)) or 0)
        length = float(rec.get("mean_length", rec.get("mean_organism_length", 0)) or 0)
        out[lid] = {"score": divisions * 10.0 + recruited + light / 10.0 + length,
                    "divisions": divisions, "recruited": recruited, "light_wh": light,
                    "mean_length": length, "deaths": rec.get("deaths", 0),
                    "genome": _dict_or_none(rec.get("best_genome", rec.get("genome"))),
                    "bodyplan": _dict_or_none(rec.get("best_bodyplan", rec.get("bodyplan")))}
    return out


def _dict_or_none(v):
    return v if isinstance(v, dict) else None


def select_and_refill(lineages, scores, rng, epoch, next_n, ORG=None):
    """Keep the top ceil(K/2) by score; refill to K with mutated keepers."""
    ranked = sorted(lineages, key=lambda ln: -scores.get(ln["id"], {}).get("score", 0.0))
    n_keep = int(math.ceil(len(lineages) / 2.0))
    keepers = []
    for ln in ranked[:n_keep]:
        k = dict(ln)
        rec = scores.get(ln["id"], {})
        if rec.get("genome"):
            k["genome"] = rec["genome"]           # the evolved-in-epoch genome
        if rec.get("bodyplan"):
            k["bodyplan"] = rec["bodyplan"]
        keepers.append(k)
    nxt = list(keepers)
    while len(nxt) < len(lineages):
        parent = keepers[rng.randrange(len(keepers))]
        nxt.append({"id": "L%d" % next_n,
                    "genome": mutate_genome(ORG, parent["genome"], rng),
                    "bodyplan": mutate_bodyplan(ORG, parent["bodyplan"], rng),
                    "born_epoch": epoch + 1, "parent": parent["id"]})
        next_n += 1
    return nxt, ranked[:n_keep], next_n


# --------------------------------------------------------------------------
# thermal protocol (DESIGN: never launch above 70 C)
# --------------------------------------------------------------------------

def gpu_temperature(runner=None):
    """Read the GPU temperature via nvidia-smi; None when unavailable."""
    def _default(cmd):
        return subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              text=True, timeout=15).stdout
    runner = runner or _default
    try:
        out = runner(["nvidia-smi", "--query-gpu=temperature.gpu", "--format=csv,noheader"])
    except (OSError, subprocess.SubprocessError):
        return None
    for tok in str(out or "").replace(",", " ").split():
        try:
            return float(tok)
        except ValueError:
            continue
    return None


def wait_for_cool(limit=GPU_LIMIT_C, poll_s=GPU_POLL_S, temp=gpu_temperature,
                  sleep=time.sleep, log=print, max_wait_s=3600.0):
    """Block while the GPU is above `limit`; returns the last reading
    (None = nvidia-smi unavailable, logged, run proceeds)."""
    waited = 0.0
    while True:
        t = temp()
        if t is None:
            log("  thermal: nvidia-smi unavailable -- check skipped")
            return None
        if t <= limit:
            log("  thermal: GPU %.0f C (limit %.0f) -- ok" % (t, limit))
            return t
        if waited >= max_wait_s:
            raise RuntimeError("GPU still %.0f C after %.0f s; refusing to launch" % (t, waited))
        log("  thermal: GPU %.0f C > %.0f -- waiting %.0f s" % (t, limit, poll_s))
        sleep(poll_s)
        waited += poll_s


def engine_still_running():
    """Best-effort: is an omnisim-bin still alive after the run?  (Windows
    `tasklist`; `pgrep` does not exist in Git Bash.)"""
    try:
        if os.name == "nt":
            out = subprocess.run(["tasklist", "/FI", "IMAGENAME eq omnisim-bin.exe"],
                                 stdout=subprocess.PIPE, text=True, timeout=15).stdout
            return "omnisim-bin.exe" in out
        out = subprocess.run(["pgrep", "-x", "omnisim-bin"], stdout=subprocess.PIPE,
                             text=True, timeout=15).stdout
        return bool(out.strip())
    except (OSError, subprocess.SubprocessError):
        return False


# --------------------------------------------------------------------------
# one epoch
# --------------------------------------------------------------------------

def epoch_dir(epoch):
    return os.path.join(RUN, "epoch_%02d" % epoch)


def write_inputs(reef, config):
    os.makedirs(RUN, exist_ok=True)
    for name, obj in (("reef.json", reef), ("config.json", config)):
        with open(os.path.join(RUN, name), "w", encoding="utf-8", newline="\n") as f:
            json.dump(obj, f, indent=1)


def engine_command(duration):
    return [sys.executable, "-m", "omnisim", "run-headless",
            os.path.relpath(WORLD, REPO), "--duration", str(duration)]


def run_epoch(epoch, duration, log=print):
    """Thermal check, then ONE engine on the current world; returns
    epoch_result or None."""
    edir = epoch_dir(epoch)
    os.makedirs(edir, exist_ok=True)
    result_path = os.path.join(RUN, "epoch_result.json")
    if os.path.exists(result_path):
        os.remove(result_path)
    logp = os.path.join(edir, "engine.log")
    for suffix in ("", ".newton.json", ".stdout", ".stderr"):
        try:
            os.remove(logp + suffix)
        except OSError:
            pass
    wait_for_cool(log=log)
    env = dict(os.environ)
    env["OMNISIM_LOG_PATH"] = logp
    cmd = engine_command(duration)
    p = subprocess.run(cmd, cwd=REPO, env=env, stdout=subprocess.PIPE,
                       stderr=subprocess.STDOUT, text=True, timeout=duration + 180)
    with open(os.path.join(edir, "run_headless.out"), "w", encoding="utf-8") as f:
        f.write(p.stdout)
    if engine_still_running():
        log("  WARNING: an omnisim-bin is still running after the epoch -- "
            "stop it before the next run (Get-Process omnisim-bin)")
    if not os.path.exists(result_path):
        log("    ENGINE PRODUCED NO epoch_result.json (exit %d)" % p.returncode)
        log("    " + "\n    ".join(p.stdout.strip().splitlines()[-6:]))
        return None
    with open(result_path, encoding="utf-8") as f:
        return json.load(f)


def persist_epoch(epoch, lineages, scores, kept_ids):
    edir = epoch_dir(epoch)
    os.makedirs(edir, exist_ok=True)
    for name in ("reef.json", "config.json", "epoch_result.json", "telemetry.json"):
        src = os.path.join(RUN, name)
        if os.path.exists(src):
            shutil.copyfile(src, os.path.join(edir, name))
    if os.path.exists(WORLD):
        shutil.copyfile(WORLD, os.path.join(edir, os.path.basename(WORLD)))
    rows = []
    for ln in lineages:
        rec = dict(scores.get(ln["id"], {}))
        rec.update({"id": ln["id"], "kept": ln["id"] in kept_ids,
                    "born_epoch": ln.get("born_epoch", 0), "parent": ln.get("parent"),
                    "genome": ln.get("genome"), "bodyplan": ln.get("bodyplan")})
        rows.append(rec)
    with open(os.path.join(edir, "lineages.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump(rows, f, indent=1)


def print_table(epoch, lineages, scores, kept_ids, wall_s):
    print("  %-6s %8s %5s %7s %8s %6s  %s" % (
        "lineage", "score", "div", "recruit", "light_wh", "len", "verdict"))
    for ln in sorted(lineages, key=lambda l: -scores.get(l["id"], {}).get("score", 0.0)):
        r = scores.get(ln["id"], {})
        print("  %-6s %8.2f %5d %7d %8.2f %6.2f  %s  target_length=%s" % (
            ln["id"], r.get("score", 0.0), r.get("divisions", 0), r.get("recruited", 0),
            r.get("light_wh", 0.0), r.get("mean_length", 0.0),
            "KEEP" if ln["id"] in kept_ids else "cull",
            ln.get("bodyplan", {}).get("target_length")))
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
    return {"arena": args.arena, "n_patches": N_PATCHES, "epoch_s": args.epoch_s,
            "watch": bool(watch), "epoch": epoch, "cells": args.cells,
            "organisms": args.organisms, "free_cells": FREE_CELLS, "seed": args.seed,
            "dim": 1.0, "time_scale": 20.0, "controller": CONTROLLER}


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

def dry_run(args, lineages, next_epoch, next_n, mods, rng):
    # a moving v3 reef runs ~1.5-2x slower than realtime at 16-24 cells
    duration = int(args.epoch_s * 1.9) + DURATION_SLACK_S
    print("DRY RUN -- nothing is written, no engine is launched.")
    print("plan: %d epoch(s) starting at epoch %d; %d cells = %d organisms x 2 + %d free "
          "+ %d parked; arena %g m, %d light patches, epoch %d s (+%d s slack)"
          % (args.epochs, next_epoch, args.cells, args.organisms,
             max(0, min(FREE_CELLS, args.cells - 2 * args.organisms)),
             max(0, args.cells - 2 * args.organisms - FREE_CELLS), args.arena, N_PATCHES,
             args.epoch_s, DURATION_SLACK_S))
    print("modules: " + ", ".join("mz.%s=%s" % (k, "present" if v else "STUB")
                                  for k, v in sorted(mods.items())))
    for e in range(next_epoch, next_epoch + args.epochs):
        reef = build_reef(lineages, args.cells, args.arena, e, rng, mods, seed_len=args.seed_length)
        check_conserved(reef)
        print("\n=== epoch %d ===" % e)
        for o in reef["organisms"]:
            print("  %-8s lineage %-4s cells %s  target_length %s  A=%.2f omega=%.2f" % (
                o["id"], o["lineage"], o["members"], o["bodyplan"].get("target_length"),
                float(o["genome"].get("A", 0)), float(o["genome"].get("omega", 0))))
        print("  free cells %s; parked %s" % (reef["free"], reef["parked"] or "none"))
        print("  inputs : %s" % _rel(os.path.join(RUN, "reef.json")))
        print("           %s" % _rel(os.path.join(RUN, "config.json")))
        print("  world  : %s" % _rel(WORLD))
        print("  log    : OMNISIM_LOG_PATH=%s" % _rel(os.path.join(epoch_dir(e), "engine.log")))
        print("  thermal: nvidia-smi --query-gpu=temperature.gpu (wait while > %.0f C)" % GPU_LIMIT_C)
        print("  command: %s  (cwd %s)" % (" ".join(engine_command(duration)), REPO))
        print("  result : %s -> keep top %d of %d lineages, refill by mutation" % (
            _rel(os.path.join(RUN, "epoch_result.json")),
            int(math.ceil(args.organisms / 2.0)), args.organisms))
        # Selection cannot be simulated without a result; pretend the first
        # half are the keepers so the next roster's shape is visible.
        fake = {ln["id"]: {"score": float(len(lineages) - i)} for i, ln in enumerate(lineages)}
        lineages, _kept, next_n = select_and_refill(lineages, fake, rng, e, next_n,
                                                    mods.get("organism"))
        for ln in lineages:
            if ln.get("born_epoch") == e + 1:
                ln["id"] = ln["id"] + "*"       # "*": which keeper it descends from
                                                # is decided by the real scores


# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--epochs", type=int, default=8)
    ap.add_argument("--cells", type=int, default=24, help="total cells (conserved)")
    ap.add_argument("--organisms", type=int, default=6, help="seeded organisms at start")
    ap.add_argument("--seed-length", type=int, default=4,
                    help="cells per seeded organism (P1c: 2-cell chains barely move, 4-chains crawl at 0.06 m/s)")
    ap.add_argument("--epoch-s", type=int, default=240, help="simulated seconds per epoch")
    ap.add_argument("--arena", type=float, default=18.0, help="floor side in metres")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--resume", action="store_true", help="continue from state.json")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the plan; write nothing, run nothing")
    args = ap.parse_args(argv)
    if 2 * args.organisms > args.cells:
        ap.error("--organisms x 2 cannot exceed --cells")

    mods = load_modules(required=not args.dry_run)
    ORG = mods.get("organism")
    rng = random.Random(args.seed)
    state = load_state() if args.resume else None
    if state:
        lineages = state["lineages"]
        next_epoch, next_n = state["next_epoch"], state["next_lineage_n"]
        if state.get("rng"):
            rng.setstate(_untuple(state["rng"]))
        print("resumed: %d lineages at epoch %d" % (len(lineages), next_epoch))
    else:
        lineages = initial_lineages(args.organisms, rng, ORG)
        next_epoch, next_n = 0, args.organisms

    if args.dry_run:
        dry_run(args, lineages, next_epoch, next_n, mods, rng)
        return 0

    W, S = mods["worldgen"], mods["scene"]
    # a moving v3 reef runs ~1.5-2x slower than realtime at 16-24 cells
    duration = int(args.epoch_s * 1.9) + DURATION_SLACK_S
    t_start = time.time()
    for epoch in range(next_epoch, next_epoch + args.epochs):
        print("\n=== epoch %d  (%d cells, %d lineages, %d s sim) ===" % (
            epoch, args.cells, len(lineages), args.epoch_s))
        t0 = time.time()
        reef = build_reef(lineages, args.cells, args.arena, epoch, rng, mods, seed_len=args.seed_length)
        check_conserved(reef)
        write_inputs(reef, make_config(args, epoch))
        # v3 cells (rollers on the bottom AND a side face) + 4 substeps: the
        # P1c configuration whose chains translate and steer (README).
        W.write_world(reef["cells"], WORLD,
                      scene_lines=S.scene_lines(args.arena, N_PATCHES, CONTROLLER),
                      controller=CONTROLLER, rollers="v3", substeps=4,
                      arena=args.arena)
        bal = S.brace_balance(open(WORLD, encoding="utf-8").read())
        if not bal["balanced"]:
            sys.exit("generated world has unbalanced braces: %s" % bal)

        result = run_epoch(epoch, duration)
        if result is None:
            persist_epoch(epoch, lineages, {}, set())
            print("  epoch failed; stopping (state NOT advanced -- --resume re-runs it)")
            return 1

        scores = score_lineages(result)
        nxt, kept, next_n = select_and_refill(lineages, scores, rng, epoch, next_n, ORG)
        kept_ids = {ln["id"] for ln in kept}
        persist_epoch(epoch, lineages, scores, kept_ids)
        print_table(epoch, lineages, scores, kept_ids, time.time() - t0)

        lineages = nxt
        save_state({"next_epoch": epoch + 1, "next_lineage_n": next_n,
                    "lineages": lineages, "rng": _tuple_to_list(rng.getstate()),
                    "args": vars(args)})

    print("\n=== done (%.0f s total); roster for the next epoch: %s ===" % (
        time.time() - t_start, ", ".join(ln["id"] for ln in lineages)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
