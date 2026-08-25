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

"""variants.py -- runs a ladder rung under NON-DEFAULT engine settings.

    python tests/benchmarks/ladder0/omnisim/variants.py warp
    python tests/benchmarks/ladder0/omnisim/variants.py grasp
    python tests/benchmarks/ladder0/omnisim/variants.py all

**NOTHING THIS FILE PRODUCES IS A LADDER ROW.**  The ladder table is one scene
per rung across three simulators; the moment a row could come from a scene the
contract did not describe, the table stops being comparable and nothing in it
says which scene produced it.  So the arm's ``run()`` has no scene parameter,
these variants live outside it, and their output is labelled ``[variant]``
throughout.

They exist because a red row is only half a finding.  "A grasp with a 9x
Coulomb margin does not hold" is a measurement; "and it is the friction cone,
not the grip force, and here is the single field that changes it" is an
attribution, and this repo's own rule is that a red verdict is not publishable
until it has been chased to a mechanism.  Each variant below changes ONE thing
against the honest scene and reports the same reduced measurements, so the
difference is attributable rather than merely different.

THE TWO VARIANTS
----------------
``warp`` -- rung 5 under ``newtonSolver "mujoco_warp"``.  The engine's raycast
service is documented to freeze there while the CPU ``mj_step`` path is fine,
and rung 5 is the instrument built to see it.  Running it here rather than
hiding it means the divergence is a number in the results instead of a
paragraph in a README.

``grasp`` -- rung 8 through the shipped friction-grasp recipe, one field at a
time.  It is HISTORY as of 2026-08-12: it was written while the honest rung-8
scene declared nothing beyond the contract and the grasp crept out, and its job
was to find which field was load-bearing.  It found two, CONTRACT.md 3b then
settled what an arm is allowed to declare, and the honest scene now declares
them -- so these entries stack ON TOP of a working grasp rather than rescuing a
broken one.  The families that answer the live questions are below.

``r8defaults`` -- CONTRACT.md 3b R5: the honest scene with every declaration
REMOVED.  A required, published companion to the ladder row, and the only thing
anyone may quote for "out of the box".

``r8cone`` / ``r8impratio`` / ``r8pyr_impratio`` / ``r8noslip`` /
``r8noslip_ell`` -- CONTRACT.md 3b R3: the sweeps that make each declared value
a converged BUDGET rather than a fitted number, plus the controls that separate
the friction cone from the constraint impedance.

``r8ke`` / ``r8ke_raw`` / ``guide`` -- the shipped friction-grasp recipe
measured field by field against a grasp that WORKS.  A field that makes a
working grasp worse is worth more to a reader than four that change nothing.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import sys

HERE = os.path.abspath(os.path.dirname(__file__))
LADDER0 = os.path.dirname(HERE)
if LADDER0 not in sys.path:
    sys.path.insert(0, LADDER0)

import analysis                                   # noqa: E402
import rungs                                      # noqa: E402


def _load_sibling(stem):
    key = "ladder0_omnisim_%s" % stem
    if key in sys.modules:
        return sys.modules[key]
    sp = importlib.util.spec_from_file_location(
        key, os.path.join(HERE, "%s.py" % stem))
    mod = importlib.util.module_from_spec(sp)
    sys.modules[key] = mod
    sp.loader.exec_module(mod)
    return mod


arm = _load_sibling("arm")
worldgen = _load_sibling("worldgen")

VARIANT_PREFIX = "_variant_"

# name -> (rung, {WorldInfo fields to add or replace}, one-line why[, fault])
#
# Each entry is CUMULATIVE with the ones above it in its family, so the sweep
# reads as "and then this field", which is what makes a single field
# attributable.  The values are the shipped friction-grasp recipe's
# (docs/guide/friction-grasp.md); nothing here was tuned by this file.
VARIANTS = {
    # TWO warp probes, and the SECOND is the one that matters.  Rung 5's scene
    # is entirely static -- a kinematic carrier, a static wall, no dynamic body
    # at all -- so a ray cast against a stale copy of the scene would still
    # come back right, because nothing in that scene ever moves.  Rung 6 puts
    # the same sensor on a body whose pose the SOLVER produces, which is the
    # only version of the question that can distinguish "the ray tracks" from
    # "the scene never changed".
    "warp5": (5, {"newtonSolver": '"mujoco_warp"'},
              "GPU solver, STATIC scene -- cannot refute a stale-scene freeze"),
    "warp6": (6, {"newtonSolver": '"mujoco_warp"'},
              "GPU solver, sensor on a DYNAMIC body -- the decisive probe"),
    # mujoco_warp has NO noslip field and its put_model RAISES on a non-zero
    # one, so the engine must DECLINE the request and warn rather than pass it
    # down (which would abort the solver build) or apply it silently (which
    # would report a knob as active that cannot exist there).  This probe
    # exists because an untested decline branch is how a knob comes to mean
    # two different things on two paths.
    "warp8_noslip": (8, {"newtonSolver": '"mujoco_warp"',
                         "newtonNoslipIterations": "5"},
                     "GPU solver + a noslip request it CANNOT honour -- must "
                     "warn, decline, and keep running"),
    "g1_cone": (8, {"newtonCone": '"elliptic"'},
                "the EXACT Coulomb cone instead of the pyramidal inscription"),
    "g2_impratio": (8, {"newtonCone": '"elliptic"', "newtonImpratio": "100"},
                    "+ a stiffer frictional-to-normal constraint impedance"),
    "g3_ke": (8, {"newtonCone": '"elliptic"', "newtonImpratio": "100",
                  "newtonContactKe": "8000", "newtonContactKd": "200"},
              "+ the recipe's firmer contact"),
    "g4_iters": (8, {"newtonCone": '"elliptic"', "newtonImpratio": "100",
                     "newtonContactKe": "8000", "newtonContactKd": "200",
                     "newtonIterations": "150", "newtonLsIterations": "50"},
                 "+ the recipe's full solver effort (the whole recipe)"),
    # The rung-8 fault battery, run on the configuration where the baseline
    # grasp HOLDS.  It answers a question the self-test cannot while the
    # honest row is red: are rung 8's assertions actually surgical, or do they
    # simply all fail together?  ``no_traverse`` must redden ``place_x`` ALONE.
    "g2_no_traverse": (8, {"newtonCone": '"elliptic"',
                           "newtonImpratio": "100"},
                       "the no_traverse fault where the grasp holds -- "
                       "place_x must go red ALONE", "no_traverse"),
    "g2_no_grip": (8, {"newtonCone": '"elliptic"', "newtonImpratio": "100"},
                   "the CAUSAL CONTROL where the grasp holds -- fingers open, "
                   "the payload must stay on the table", "no_grip"),
}

FAMILY = {"warp": ["warp5", "warp6", "warp8_noslip"],
          "grasp": ["g1_cone", "g2_impratio", "g3_ke", "g4_iters"],
          "graspfaults": ["g2_no_traverse", "g2_no_grip"]}


# --------------------------------------------------------------------------
# Rung 11 on the GPU path -- the njmax question, and the reason it is HERE
# --------------------------------------------------------------------------
#
# The ladder row is CPU ``mj_step``.  The constraint-buffer cliff is a
# ``mujoco_warp`` phenomenon, and CONTRACT.md section 3E is explicit that a
# variant is never the headline -- so it is measured here, beside the row, one
# field at a time.
#
# WHAT IS ACTUALLY BEING TESTED, because an earlier instrument tested the wrong
# thing and honestly reported that it could not make its own detector bite.
# ``newtonNjmax`` is a FLOOR, not a cap: the runtime's own note says "newton
# raises a too-small njmax to the INITIAL nefc at construction", so the cap that
# governs truncation is ``max(requested_or_256, nefc_at_t0)``.  Setting the
# field LOWER therefore cannot starve anything.  What starves a fleet is
# spawning it CLEAR of the ground, so nefc at t = 0 is ~0, the 256 default
# stands, and a settled 4-wheel fleet needing 32 N rows overflows from N = 9.
#
# So the pair below is: the honest scene (spawned in contact, a generous 4096
# declared) and the starved one (spawned clear, the declaration removed).  If
# the second cannot be made to overflow either, then TWO independent
# instruments have failed to reproduce the documented ~9-robot threshold and
# it should stop being repeated as fact -- which is the most useful thing this
# variant can produce.
R11_WARP = {"newtonSolver": '"mujoco_warp"'}
R11_NO_BUDGET = {"newtonNjmax": None, "newtonNconmax": None}

for _n in (8, 16):
    VARIANTS["n%d_warp" % _n] = (
        11, dict(R11_WARP),
        "N=%d on the GPU path, fleet spawned IN CONTACT with the contract's "
        "generous 4096 budget -- the control" % _n, "none", {"n": _n})
    VARIANTS["n%d_warp_starve" % _n] = (
        11, dict(R11_WARP, **R11_NO_BUDGET),
        "N=%d on the GPU path, fleet spawned CLEAR so nefc at t=0 is ~0 and "
        "the 256 default stands against %d needed rows -- the starve"
        % (_n, 32 * _n), "starve_budget", {"n": _n})
    VARIANTS["n%d_cpu_starve" % _n] = (
        11, dict(R11_NO_BUDGET),
        "N=%d starved on the CPU mj_step path -- the control that says "
        "whether the cliff is GPU-only" % _n, "starve_budget", {"n": _n})
    # THE SINGLE-CHANGE STARVE, added after the first pair measured two things
    # at once.  ``starve_budget`` was designed to remove the budget AND spawn
    # the fleet clear, on the briefed premise that a fleet spawning IN CONTACT
    # would have its cap auto-raised to the peak by newton's njmax floor.
    # MEASURED: the engine reports ``initial nefc=0`` on the in-contact scene
    # too, so the floor never rises above the request and the spawn clearance
    # is not what matters.  This variant changes ONLY the declaration.
    VARIANTS["n%d_warp_nobudget" % _n] = (
        11, dict(R11_WARP, **R11_NO_BUDGET),
        "N=%d on the GPU path, fleet spawned IN CONTACT, budget declaration "
        "REMOVED -- the single-change starve" % _n, "none", {"n": _n})

FAMILY["r11warp"] = ["n8_warp", "n8_warp_starve", "n16_warp",
                     "n16_warp_starve"]
FAMILY["r11cpu"] = ["n8_cpu_starve", "n16_cpu_starve"]
FAMILY["r11budget"] = ["n8_warp_nobudget", "n16_warp_nobudget"]
#: The whole constraint-budget study in one artefact.  ``variants.json`` is
#: written per invocation, so a study split across three commands leaves only
#: the last one on disk -- and a study whose controls are in a file somebody
#: overwrote is not a study.
FAMILY["r11"] = (FAMILY["r11warp"] + FAMILY["r11budget"] + FAMILY["r11cpu"])


# --------------------------------------------------------------------------
# CONTRACT.md 3b -- the evidence the fair-defaults rule REQUIRES
# --------------------------------------------------------------------------
#
# R3 says a declared value must be a BUDGET, not a fit: sweep it, publish the
# sweep, and show the answer is insensitive to the exact value over a stated
# range.  R5 says the arm must also publish the same scene with every
# declaration REMOVED.  Both are measurements, so both live here rather than in
# prose, and both are regenerable by anyone with the tree.
#
# ``None`` as a field value DELETES that field from the honest world, which is
# what makes the R5 datum expressible once the honest world declares something:
# it is the honest scene minus the declarations and nothing else.
R8_DECLARED = ("newtonNoslipIterations", "newtonCone", "newtonImpratio")


def _grasp_variant(name, why, fault="none", **fields):
    VARIANTS[name] = (8, {k: v for k, v in fields.items()}, why, fault)
    return name


# R5: the defaults datum.  Every R2 declaration removed, nothing else touched.
_grasp_variant("r5_defaults",
               "R5 DATUM: every R2 declaration removed -- what this engine "
               "does with the contract's scene and nothing else",
               **{k: None for k in R8_DECLARED})

# R3: the cone.  A boolean, so 'converged' means 'the two values are both
# measured and the difference is attributable to the one field'.
FAMILY["r8cone"] = [
    _grasp_variant("cone_pyramidal",
                   "the engine default cone, alone (control for cone_elliptic)",
                   newtonNoslipIterations=None, newtonCone='"pyramidal"',
                   newtonImpratio=None),
    _grasp_variant("cone_elliptic",
                   "the exact Coulomb cone, alone",
                   newtonNoslipIterations=None, newtonCone='"elliptic"',
                   newtonImpratio=None),
]

# R3: the frictional-to-normal impedance ratio, on the elliptic cone.
FAMILY["r8impratio"] = [
    _grasp_variant("impratio_%g" % v,
                   "elliptic cone + newtonImpratio %g" % v,
                   newtonNoslipIterations=None, newtonCone='"elliptic"',
                   newtonImpratio=("%g" % v))
    for v in (1, 2, 4, 10, 30, 100, 300)
]

# R3: the noslip budget, on the ENGINE DEFAULT cone -- the like-for-like
# counterpart of the MuJoCo arm's own sweep, which was run at ITS default cone.
FAMILY["r8noslip"] = [
    _grasp_variant("noslip_%d" % n,
                   "default cone + newtonNoslipIterations %d" % n,
                   newtonNoslipIterations=("%d" % n), newtonCone=None,
                   newtonImpratio=None)
    for n in (0, 1, 2, 3, 5, 8, 20)
]

# ATTRIBUTION: is it the CONE or the IMPRATIO?  The two are recommended
# together everywhere (MuJoCo's own docs, this repo's grasp guide), which is
# exactly why they have to be separated before either is named as the cause.
# impratio is DEFINED against the elliptic cone -- on the pyramidal one MuJoCo
# applies it by scaling the pyramid's friction directions -- so "pyramidal +
# impratio" is a real, legal configuration and the right control.
FAMILY["r8pyr_impratio"] = [
    _grasp_variant("pyr_impratio_%g" % v,
                   "DEFAULT (pyramidal) cone + newtonImpratio %g -- the "
                   "control that separates the cone from the impedance" % v,
                   newtonNoslipIterations=None, newtonCone=None,
                   newtonImpratio=("%g" % v))
    for v in (10, 100)
]

# ATTRIBUTION: does the noslip pass add anything ON TOP of the declared
# configuration?  If it does not, it does not belong in the declaration --
# R2 admits a setting, it does not oblige one.
FAMILY["r8noslip_ell"] = [
    _grasp_variant("noslip_ell_%d" % n,
                   "elliptic cone + newtonImpratio 10 + newtonNoslipIterations %d" % n,
                   newtonNoslipIterations=(None if n == 0 else "%d" % n),
                   newtonCone='"elliptic"', newtonImpratio="10")
    for n in (0, 5)
]

FAMILY["r8defaults"] = ["r5_defaults"]

# THE SHIPPED GRASP RECIPE, MEASURED AGAINST A GRASP THAT WORKS.
# docs/guide/friction-grasp.md prescribes a five-field recipe.  Every one of
# those fields is measured here as a single change against the rung-8
# configuration that is known to carry the payload -- which is the only way to
# tell "this field is load-bearing" from "this field is in the recipe".  A
# field that makes a working grasp worse is worth more to a reader than four
# that make no difference.
# Variants that must NOT be told the ke they run at.  Everywhere else the
# driver is handed it and recomputes the interference, so the sweep changes
# contact stiffness with the GRIP FORCE HELD.  This one deliberately does not,
# because it is the configuration a reader following the shipped grasp guide
# actually gets: they add `newtonContactKe 8000` to the world and their
# controller's finger command does not change.  Both cases are worth having.
NO_KE_HOOK = {"ke_8000_uncompensated"}

FAMILY["r8ke_raw"] = [
    _grasp_variant("ke_8000_uncompensated",
                   "newtonContactKe 8000 with the finger command UNCHANGED -- "
                   "what a reader following the shipped recipe gets, where the "
                   "same interference now develops a larger force",
                   newtonContactKe="8000"),
]

FAMILY["r8ke"] = [
    # ke ALONE, with kd left at the engine default, so the stiffness is
    # separated from the damping it is always quoted with.  The grip force is
    # held constant throughout: the driver is handed the ke it is running at
    # and recomputes the interference from it (engine_facts.NEWTON_CONTACT_KE),
    # so this sweep changes contact stiffness and nothing else.
    _grasp_variant("ke_%d" % v,
                   "newtonContactKe %d on a WORKING grasp, kd left at the "
                   "engine default" % v,
                   newtonContactKe="%d" % v)
    for v in (4000, 6000, 8000)
]

FAMILY["guide"] = [
    _grasp_variant("guide_ke",
                   "the recipe's firmer contact (newtonContactKe 8000 / Kd 200) "
                   "on top of a WORKING grasp -- the driver is told, so the "
                   "interference is recomputed and the grip force is held at "
                   "RUNG8_GRIP_N",
                   newtonContactKe="8000", newtonContactKd="200"),
    _grasp_variant("guide_iters",
                   "the recipe's solver effort (newtonIterations 150 / "
                   "newtonLsIterations 50) on top of a WORKING grasp",
                   newtonIterations="150", newtonLsIterations="50"),
    _grasp_variant("guide_condim4",
                   "the recipe's newtonCondim 4 (torsional friction) on top of "
                   "a WORKING grasp",
                   newtonCondim="4"),
    _grasp_variant("guide_full_recipe",
                   "the WHOLE shipped recipe verbatim, replacing this arm's "
                   "declaration",
                   newtonContactKe="8000", newtonContactKd="200",
                   newtonCone='"elliptic"', newtonImpratio="100",
                   newtonIterations="150", newtonLsIterations="50",
                   newtonCondim="4"),
]


def variant_world(name):
    """Write the variant's ``.wbt`` beside the honest one and return its path.

    Built by editing the honest world's ``WorldInfo`` and NOTHING else, so the
    geometry, the masses and the commanded schedule are provably the same scene
    -- which is the only reason the difference is attributable to the field.

    A field whose value is ``None`` is DELETED rather than set.  That is what
    makes CONTRACT.md 3b's R5 datum expressible: the honest scene minus its
    declarations, produced from the honest generator rather than from a second
    copy of the world that could drift from it.

    The edit is scoped to the ``WorldInfo`` BLOCK, not to the file: a bare
    line-key match would rewrite any node field that happened to share a name,
    and the variant would then be changing two things while reporting one.
    """
    spec = VARIANTS[name]
    rung, fields = spec[0], dict(spec[1])
    fault = spec[3] if len(spec) > 3 else "none"
    run = spec[4] if len(spec) > 4 else None
    text = worldgen.wbt(rung, fault, run)
    head, sep, tail = text.partition("WorldInfo {\n")
    if not sep:
        raise RuntimeError("no WorldInfo block in the rung-%d world" % rung)
    close = tail.index("\n}\n")
    body, rest = tail[:close + 1], tail[close + 1:]
    out = []
    for line in body.splitlines(True):
        key = line.strip().split(" ")[0] if line.strip() else ""
        if key in fields:
            v = fields.pop(key)
            if v is not None:                    # None deletes the line
                out.append("  %s %s\n" % (key, v))
        else:
            out.append(line)
    live = {k: v for k, v in fields.items() if v is not None}
    out.extend("  %s %s\n" % (k, v) for k, v in live.items())
    text = head + sep + "".join(out) + rest
    text = text.replace("# Arm: omnisim.  Fault: none.",
                        "# Arm: omnisim.  VARIANT %r -- NOT a ladder row.\n"
                        "# %s" % (name, VARIANTS[name][2]))
    path = os.path.join(worldgen.WORLDS, "%s%s_rung%d.wbt"
                        % (VARIANT_PREFIX, name, rung))
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    return path


#: Sample every N steps for the constraint-buffer telemetry.  The engine logs
#: the observed peak ``nefc``/``ncon`` against the ALLOCATED cap, which is the
#: only way to see a truncation whose own warning is a ``wp.printf`` inside a
#: solver kernel -- discarded entirely on Windows, where the engine is a
#: GUI-subsystem binary.
CONSTRAINT_STATS_EVERY = 25


def _constraint_peaks(text):
    """Pull ``nefc=<peak>/<cap>`` lines out of a captured engine log."""
    out = []
    for line in (text or "").splitlines():
        if "constraint peak" in line or "CONSTRAINT BUFFER OVERFLOW" in line \
                or "constraint buffers" in line:
            out.append(line.strip())
    return out[-12:]


def run_variant(name, out_root, timeout_s=300.0, binary=None):
    spec = VARIANTS[name]
    rung, fields, why = spec[0], spec[1], spec[2]
    fault = spec[3] if len(spec) > 3 else "none"
    run = spec[4] if len(spec) > 4 else None
    world = variant_world(name)
    d = os.path.join(out_root, "variant_%s" % name)
    # A variant that changes the CONTACT STIFFNESS must tell the driver, or the
    # rung-8 interference -- which is derived from ke -- would silently change
    # the grip force too and the variant would be sweeping two things.  See
    # engine_facts.NEWTON_CONTACT_KE.
    env_extra = {}
    if "newtonContactKe" in fields and name not in NO_KE_HOOK:
        env_extra["LADDER0_NEWTON_KE"] = fields["newtonContactKe"]
    if rung in rungs.MULTI_RUN:
        env_extra["LADDER0_STRIDE"] = str(arm.STRIDE.get(rung, 1))
        env_extra["LADDER0_TAG"] = str((run or {}).get("tag")
                                       or "n%d" % (run or {}).get("n", 1))
    if rung == 11:
        env_extra["OMNISIM_NEWTON_CONSTRAINT_STATS"] = \
            str(CONSTRAINT_STATS_EVERY)
        # PER-VARIANT, never the default.  The runtime's own default is
        # ``.build_tmp/newton_solver.log`` -- one shared file in a tree three
        # lanes work in at once, which would make the telemetry of two runs
        # indistinguishable after the fact.
        os.makedirs(d, exist_ok=True)
        env_extra["OMNISIM_NEWTON_LOG"] = os.path.join(d, "newton_solver.log")
    samples, meta = arm.launch(rung, d, world, fault=fault, binary=binary,
                               timeout_s=timeout_s,
                               env_extra=env_extra or None)
    # A multi-run rung reduced from ONE run: the reducer wants the amendment-A
    # shape, and wrapping it here keeps the reduction shared rather than
    # growing a second one for variants.  ``robots_seen`` then compares one
    # fleet against the whole sweep's total and is MEANINGLESS on a variant --
    # which is reported below rather than quietly hidden.
    if rung in rungs.MULTI_RUN and "runs" not in samples:
        tag = env_extra.get("LADDER0_TAG")
        samples = {"rung": rung, "sim": samples.get("sim"),
                   "runs": [dict(samples, tag=tag, params=dict(run or {}))],
                   "wall": samples.get("wall")}
    m = analysis.reduce_samples(samples, exit_code=meta.get("exit_code"))
    checks = [c.as_dict() for c in rungs.check_rung(rung, m)]
    row = {"variant": name, "rung": rung, "why": why, "world": world,
           "fault": fault, "run": run,
           "declared": {k: v for k, v in fields.items()},
           "checks": checks, "error": meta.get("error"),
           "binary": meta.get("binary"),
           "binary_sha256": meta.get("binary_sha256"),
           "newton": meta.get("newton"),
           "single_run_of_multi": rung in rungs.MULTI_RUN,
           "measurement": {k: v for k, v in m.items()
                           if not isinstance(v, (list, dict))}}
    if rung == 11:
        row["per_n"] = m.get("per_n")
        row["constraint_peaks"] = []
        for p in (os.path.join(d, "newton_solver.log"),
                  os.path.join(d, "engine.log")):
            try:
                with open(p, "r", encoding="utf-8", errors="replace") as f:
                    row["constraint_peaks"] += _constraint_peaks(f.read())
            except OSError:
                pass
    return row


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("family", choices=sorted(FAMILY) + ["all"])
    ap.add_argument("--out", default=os.path.join(LADDER0, "results",
                                                  "variants"))
    ap.add_argument("--timeout", type=float, default=300.0)
    ap.add_argument("--bin", default=None,
                    help="engine binary (A/B work; a scratch build does not "
                         "clobber the one a running GUI has locked)")
    args = ap.parse_args(argv)

    names = ([n for f in sorted(FAMILY) for n in FAMILY[f]]
             if args.family == "all" else FAMILY[args.family])
    out_root = os.path.abspath(args.out)
    os.makedirs(out_root, exist_ok=True)

    print()
    print("VARIANT RUNS -- NOT LADDER ROWS.  Each changes ONE thing against")
    print("the honest scene so a red row can be chased to a mechanism.")
    print("=" * 100)
    rows = []
    for name in names:
        row = run_variant(name, out_root, timeout_s=args.timeout,
                          binary=args.bin)
        rows.append(row)
        bad = [c["name"] for c in row["checks"] if not c["ok"]]
        print()
        print("[variant] %-12s rung %d -- %s" % (name, row["rung"],
                                                 row["why"]))
        print("          %s" % ("ALL GREEN" if not bad
                                else "RED: " + ", ".join(bad)))
        for c in row["checks"]:
            print("          %-20s %14.6g  vs %-12.6g tol %-10.4g %s"
                  % (c["name"],
                     float("nan") if c["measured"] is None else c["measured"],
                     c["expected"], c["tol"], "ok" if c["ok"] else "FAIL"))
        if row.get("single_run_of_multi"):
            print("          NOTE: one run of a multi-run rung -- "
                  "'robots_seen' compares this fleet against the whole "
                  "sweep's total and is meaningless here")
        for line in row.get("constraint_peaks") or []:
            print("          [engine] %s" % line)
        if row["error"]:
            print("          ! %s" % str(row["error"]).replace("\n", " ")[:200])
    with open(os.path.join(out_root, "variants.json"), "w",
              encoding="utf-8") as f:
        json.dump(rows, f, indent=1)
    print()
    print("results: %s" % out_root)
    return 0


if __name__ == "__main__":
    sys.exit(main())
