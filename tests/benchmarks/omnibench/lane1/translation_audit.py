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

"""translation_audit.py — does the model we hand MuJoCo match the world we were given?

THE INSTRUMENT THE ODE DELETION REMOVED, REBUILT ON A BETTER PRINCIPLE.

Every Newton *integration* defect this project has found was found by running one
`.wbt` through two independent paths and diffing. `bdc02139` deleted the second
path, and `docs/benchmarks/correctness-scope.md` records the consequence as the
one genuinely open correctness gap:

    "Nothing can currently distinguish 'the solver got this wrong' from 'we
    handed the solver the wrong model' on a real world. What should replace it
    ... is not decided."

This is that replacement, and it answers the question more directly than the ODE
diff ever did. A two-backend diff reports *disagreement* and leaves you to work
out which side is wrong; an audit reports **"the world says gravity is 3.72 m/s^2
and the model MuJoCo stepped has 9.81"**, which names the wrong side. It also
needs one engine, not two, so nothing has to be kept alive to run it.

WHAT IT COMPARES

  authored side   the `.wbt`'s own text (WorldInfo, and what the file declares)
  model side      `OMNISIM_NEWTON_DUMP_MJMODEL` -- the EXACT mjModel the solver
                  stepped, dumped at finalize by OmNewtonBackend

Between those two sits every line of translation code we own: the parser, the
coordinate-system resolution, the inertia composition, the friction mapping, the
statics registration. That is the layer with the project's entire track record of
real bugs, and the layer no external arm can see -- bare `mujoco` and `pybullet`
validate the SOLVER, and neither of them reads `.wbt`.

WHAT IT WOULD HAVE CAUGHT (each of these actually shipped)

  * gravity never plumbed -- every Newton world ran at -9.81 regardless of
    `WorldInfo.gravity`                                   -> check `gravity`
  * `coordinateSystem` never reaching the solver -- all 210 NUE worlds had
    gravity projected to exactly zero and never fell (c77cbe98)
                                                          -> check `gravity`
  * a husky-wheel inertia preset applied to every `Solid` without an explicit
    `inertiaMatrix`                                       -> check `mass`
  * `contactProperties` / `coulombFriction` declared in ~202 worlds and read by
    nothing                                               -> check `ignored_fields`
  * a frictionless contact (`condim=1`) making a declared mu unreachable, which
    makes friction fidelity unmeasurable rather than wrong -> check `condim`

NOT A PHYSICS ORACLE. It says nothing about whether MuJoCo integrated correctly
-- that is lane 1's analytic ground truth, which this does not touch and does not
replace. The two are complements: lane 1 asks "is the trajectory right?", this
asks "were we even simulating the world in the file?". A lane-1 scene can pass
this audit and still be wrong, and vice versa. Read both.

USAGE

    python translation_audit.py --world path/to/world.omniworld
    python translation_audit.py --lane1              # every lane-1 dt=4 scene
    python translation_audit.py --world W --json out.json

Exit code is 1 if any ERROR finding is raised, else 0 -- so it can gate a build.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
OMNIBENCH = HERE.parent
REPO = OMNIBENCH.parent.parent.parent
sys.path.insert(0, str(OMNIBENCH))

from common import engine_launch  # noqa: E402

# Fields a world may still DECLARE that reach nothing in the engine. Value is
# what to use instead (None = the capability is simply gone). Sourced from
# AGENTS.md's "Anything about VRML / .wbt primitives" row and
# docs/reference/worldinfo.md, both of which mark these parsed-but-unread.
IGNORED_FIELDS = {
    "contactProperties": "newtonGroundMu / newtonContactKe / newtonContactKd",
    "coulombFriction": "newtonGroundMu",
    "CFM": "newtonContactKd",
    "ERP": "newtonContactKe",
    "broadphase": None,
    "physicsDisableTime": None,          # Newton has no body sleep
    "physicsDisableLinearThreshold": None,
    "physicsDisableAngularThreshold": None,
    "immersionProperties": None,         # Fluid/ImmersionProperties deleted
}

SEV_ERROR, SEV_WARN, SEV_INFO = "ERROR", "WARN", "INFO"


# --------------------------------------------------------------------------
# authored side: the .wbt's own text

def _worldinfo_block(text):
    """The body of `WorldInfo { ... }`, brace-matched (fields nest)."""
    m = re.search(r"\bWorldInfo\s*\{", text)
    if not m:
        return ""
    i = m.end() - 1
    depth = 0
    for j in range(i, len(text)):
        if text[j] == "{":
            depth += 1
        elif text[j] == "}":
            depth -= 1
            if depth == 0:
                return text[i + 1:j]
    return text[i + 1:]


def parse_world(path):
    """Authored contract: what the FILE says, independent of any engine."""
    text = Path(path).read_text(encoding="utf-8", errors="replace")
    wi = _worldinfo_block(text)
    out = {"path": str(path), "worldinfo_found": bool(wi)}

    def scalar(name, cast=float):
        m = re.search(r"^\s*%s\s+([-+0-9.eE]+)\s*$" % re.escape(name), wi,
                      re.MULTILINE)
        return cast(m.group(1)) if m else None

    def string(name):
        m = re.search(r'^\s*%s\s+"([^"]*)"' % re.escape(name), wi, re.MULTILINE)
        return m.group(1) if m else None

    # An OMITTED field is not an unknown: WorldInfo has documented defaults
    # (docs/reference/worldinfo.md -- gravity 9.81, basicTimeStep 32), and most
    # worlds in the tree declare neither. Falling back to "cannot evaluate"
    # there would make the audit useless on exactly the corpus it exists to
    # check, so the schema default is used and recorded as such.
    out["gravity"] = scalar("gravity")
    out["gravity_declared"] = out["gravity"] is not None
    if out["gravity"] is None:
        out["gravity"] = 9.81
    out["basicTimeStep"] = scalar("basicTimeStep")
    out["basicTimeStep_declared"] = out["basicTimeStep"] is not None
    if out["basicTimeStep"] is None:
        out["basicTimeStep"] = 32.0
    out["coordinateSystem"] = string("coordinateSystem") or "ENU"
    out["newtonSolver"] = string("newtonSolver")
    out["newtonSubsteps"] = scalar("newtonSubsteps")
    out["newtonGroundMu"] = scalar("newtonGroundMu")
    out["newtonStatics"] = bool(re.search(r"^\s*newtonStatics\s+TRUE", wi,
                                          re.MULTILINE))
    # The ODE-path friction declaration, kept as NUMBERS so the audit can show
    # the contradiction rather than only naming the field.
    #
    # ⚠ ALL of them: `contactProperties` is a LIST, one entry per material pair,
    # so a world can declare several different frictions. `newtonGroundMu` is a
    # single global value and cannot represent that, which makes the migration
    # unsound for such worlds -- they need per-material contact support, not a
    # rewrite. Collect every value so the difference is detectable instead of
    # silently collapsing to the first one.
    cfs = [float(x) for x in
           re.findall(r"coulombFriction\s*\[\s*([-+0-9.eE]+)", text)]
    out["coulombFriction_all"] = cfs
    out["coulombFriction"] = cfs[0] if cfs else None
    out["coulombFriction_uniform"] = (
        bool(cfs) and all(abs(v - cfs[0]) <= 1e-9 for v in cfs))
    # Declared-but-unread fields, searched over the WHOLE file: ContactProperties
    # is a WorldInfo child, but immersionProperties hangs off a Solid.
    out["declared_ignored"] = sorted(
        f for f in IGNORED_FIELDS
        if re.search(r"(?<![A-Za-z])%s(?![A-Za-z])" % re.escape(f), text))
    # A Solid collides only through boundingObject; count for the geom check.
    out["n_bounding_objects"] = len(re.findall(r"\bboundingObject\b", text))
    out["n_solid"] = len(re.findall(r"\bSolid\s*\{", text))
    # Motor devices the file asks for. Counted so the audit can catch motors that
    # are accepted and silently ignored (AGENTS.md: motorised BallJoint and
    # Hinge2Joint do not actuate), which otherwise shows up only as "the robot
    # does not move". URDFRobot bodies are expanded from a URL at parse time and
    # their motors are NOT in this text, so a zero here means "this file declares
    # none", not "this scene has none" -- the check is one-directional.
    out["n_motor_devices"] = len(re.findall(
        r"\b(?:RotationalMotor|LinearMotor)\s*\{", text))
    out["has_urdf_robot"] = bool(re.search(r"\bURDFRobot\s*\{", text))
    return out


def expected_gravity_vec(magnitude, coordinate_system):
    """MuJoCo-frame gravity from the two fields that decide it.

    `WorldInfo.gravity` is an SFFloat MAGNITUDE (docs/reference/worldinfo.md),
    and `coordinateSystem` names which world axis is Up by the position of 'U'
    in the string: ENU -> +Z up, NUE and EUN -> +Y up. Gravity is -magnitude on
    that axis. Getting this pairing wrong is not hypothetical: it is exactly how
    210 NUE worlds ran at gravity (0,0,0) until c77cbe98.
    """
    if magnitude is None:
        return None
    cs = (coordinate_system or "ENU").upper()
    if "U" not in cs or len(cs) != 3:
        return None
    v = [0.0, 0.0, 0.0]
    v[cs.index("U")] = -float(magnitude)
    return v


# --------------------------------------------------------------------------
# model side: OMNISIM_NEWTON_DUMP_MJMODEL

def _floats(s):
    """Floats from a dumped sequence, tolerating numpy reprs.

    The dump interpolates live arrays, so a value can arrive as
    `np.float64(-9.81)` rather than `-9.81`. A naive number regex then matches
    the `64` inside the TYPE NAME, and `gravity` comes back as six values
    instead of three -- which made the gravity check skip itself and report
    nothing. A check that cannot run must never look like a check that passed,
    so the wrappers are stripped before parsing (and `audit()` now reports any
    check it could not evaluate).
    """
    s = re.sub(r"\bnp\.\w+\(", "(", s)
    return [float(x) for x in re.findall(r"[-+]?\d[\d.eE+-]*", s)]


def parse_dump(path):
    """The mjModel the solver actually stepped, as dumped at finalize."""
    text = Path(path).read_text(encoding="utf-8", errors="replace")
    if text.startswith("DUMP FAILED"):
        return {"dump_failed": text.strip()}
    out = {"bodies": [], "geoms": [], "dofs": [], "joints": [], "n_act": 0}
    for line in text.splitlines():
        if line.startswith("nq="):
            for k, v in re.findall(r"(\w+)=(\d+)", line):
                out[k] = int(v)
        elif line.startswith("opt:"):
            m = re.search(r"timestep=([-+0-9.eE]+)", line)
            if m:
                out["timestep"] = float(m.group(1))
            m = re.search(r"gravity=\[([^\]]*)\]", line)
            if m:
                out["gravity"] = _floats(m.group(1))
            m = re.search(r"iterations=(\d+)", line)
            if m:
                out["iterations"] = int(m.group(1))
        elif line.startswith("opt cone="):
            m = re.search(r"cone=(\d+)", line)
            if m:
                out["cone"] = int(m.group(1))
            m = re.search(r"impratio=([-+0-9.eE]+)", line)
            if m:
                out["impratio"] = float(m.group(1))
        elif line.startswith("body "):
            m = re.match(r"body (\d+) (\S+)\s+mass=([-+0-9.eE]+)", line)
            if m:
                b = {"id": int(m.group(1)), "name": m.group(2),
                     "mass": float(m.group(3))}
                mi = re.search(r"inertia=\[([^\]]*)\]", line)
                if mi:
                    b["inertia"] = _floats(mi.group(1))
                out["bodies"].append(b)
        elif line.startswith("jnt "):
            m = re.match(r"jnt (\d+) (\S+)\s+type=(\d+) limited=(\d+) "
                         r"range=\[([^\]]*)\]", line)
            if m:
                out["joints"].append({
                    "id": int(m.group(1)), "name": m.group(2),
                    "type": int(m.group(3)), "limited": int(m.group(4)),
                    "range": _floats(m.group(5))})
        elif line.startswith("geom "):
            m = re.match(r"geom (\d+) (\S+)\s+body=(\d+) type=(\d+) condim=(\d+) "
                         r"contype=(\d+) conaff=(\d+) fric=\[([^\]]*)\]", line)
            if m:
                out["geoms"].append({
                    "id": int(m.group(1)), "name": m.group(2),
                    "body": int(m.group(3)), "type": int(m.group(4)),
                    "condim": int(m.group(5)), "contype": int(m.group(6)),
                    "conaff": int(m.group(7)), "friction": _floats(m.group(8))})
        elif line.startswith("act "):
            out["n_act"] += 1
        elif line.startswith("dof "):
            m = re.match(r"dof (\d+) damping=([-+0-9.eE]+) armature=([-+0-9.eE]+)",
                         line)
            if m:
                out["dofs"].append({"id": int(m.group(1)),
                                    "damping": float(m.group(2)),
                                    "armature": float(m.group(3))})
    return out


def dump_model(world, timeout_s=180, keep_dir=None):
    """Load `world` once, headless, and return the parsed mjModel dump.

    The engine writes the dump at world-finalize, so this needs a load that
    REACHES finalize -- the same condition the .newton.json sidecar records, and
    the same reason a too-short run proves nothing (AGENTS.md).
    """
    # ABSOLUTE, always: the launcher runs with cwd=REPO, so a relative world
    # path silently resolves against the repo root instead of the caller's dir
    # -- and the engine then exits 0 on the failed load, so only the missing
    # sidecar reveals it (the "headless PASS is not a content check" trap).
    world = os.path.abspath(world)
    tmp = keep_dir or tempfile.mkdtemp(prefix="xlate_audit_")
    os.makedirs(tmp, exist_ok=True)
    stem = Path(world).stem
    dump_path = os.path.join(tmp, stem + ".mjmodel.txt")
    log_path = os.path.join(tmp, stem + ".engine.log")
    console = os.path.join(tmp, stem + ".console.log")
    for p in (dump_path, log_path, log_path + ".newton.json"):
        if os.path.exists(p):
            os.remove(p)

    binpath = engine_launch.resolve_binary(REPO)
    if not binpath:
        return {"error": "omnisim-bin not found -- build first"}, {}, None
    env = engine_launch.build_env("newton", log_path, repo=REPO)
    env["OMNISIM_NEWTON_DUMP_MJMODEL"] = dump_path
    # A model audit must never inherit a scene knob from the parent shell: the
    # dump would then describe that shell, not the world.
    for k in ("OMNISIM_NEWTON_GROUND_MU", "OMNISIM_NEWTON_CONTACT_KD",
              "OMNISIM_NEWTON_CONTACT_KE", "OMNISIM_NEWTON_CONE",
              "OMNISIM_NEWTON_IMPRATIO", "OMNISIM_NEWTON_SUBSTEPS"):
        env.pop(k, None)

    # Back-to-back launches lose the controller-connect race often enough that
    # lane 1's runner retries by default, and a missing dump is indistinguishable
    # from a real "never reached finalize" without one. Retry on the same
    # condition rather than reporting a flake as a contract violation.
    def attempt(_n):
        rc_, wall_, to_ = engine_launch.launch_once(
            binpath, world, env, console, timeout_s,
            extra_args=("--stdout", "--stderr"), cwd=REPO)
        return rc_, wall_, to_, os.path.exists(dump_path)

    (rc, wall_s, timed_out, _got), _tries = engine_launch.launch_with_retries(
        attempts=3, attempt_fn=attempt, success_fn=lambda r: r[3],
        label=stem, log=None)
    verdict = engine_launch.newton_verdict(log_path)
    meta = {"rc": rc, "wall_s": round(wall_s, 2), "timed_out": timed_out,
            "verdict": verdict, "dump_path": dump_path}
    if not os.path.exists(dump_path):
        return {"error": "no dump written (world never reached finalize?)"}, meta, tmp
    return parse_dump(dump_path), meta, tmp


# --------------------------------------------------------------------------
# the audit

def _f(check, severity, msg, **extra):
    d = {"check": check, "severity": severity, "message": msg}
    d.update(extra)
    return d


def audit(world, model, meta):
    """Compare the authored contract against the model the solver stepped."""
    findings = []
    add = findings.append

    if model.get("error"):
        add(_f("dump", SEV_ERROR, "no mjModel to audit: %s" % model["error"]))
        return findings
    if model.get("dump_failed"):
        add(_f("dump", SEV_ERROR, "the engine's dump hook failed: %s"
               % model["dump_failed"]))
        return findings

    verdict = (meta or {}).get("verdict") or {}
    label, why = engine_launch.engine_attribution(verdict)
    if label != "omnisim-newton":
        add(_f("attribution", SEV_ERROR,
               "the model cannot be attributed to a verified Newton run: %s" % why))

    # --- gravity: magnitude AND direction ---------------------------------
    want_g = expected_gravity_vec(world.get("gravity"),
                                  world.get("coordinateSystem"))
    got_g = model.get("gravity")
    # A check that cannot run is a FINDING, never silence. This one skipped
    # itself for a whole session because the dump's numpy reprs broke the
    # number parse -- and an audit that prints nothing reads as an audit that
    # passed. Say so instead.
    if want_g is None:
        add(_f("gravity", SEV_WARN,
               "cannot evaluate: the world declares gravity=%r with "
               "coordinateSystem=%r, which this audit could not turn into an "
               "expected vector" % (world.get("gravity"),
                                    world.get("coordinateSystem"))))
    elif got_g is None or len(got_g) != 3:
        add(_f("gravity", SEV_WARN,
               "cannot evaluate: the model dump yielded gravity=%r, not a "
               "3-vector -- the dump format changed and this parser did not"
               % (got_g,)))
    if want_g is not None and got_g is not None and len(got_g) == 3:
        dev = max(abs(a - b) for a, b in zip(want_g, got_g))
        if dev > 1e-6:
            add(_f("gravity", SEV_ERROR,
                   "the world declares gravity %g m/s^2 with coordinateSystem "
                   "%r, i.e. %s, but the model MuJoCo stepped has %s (max "
                   "component deviation %.4g)"
                   % (world["gravity"], world["coordinateSystem"], want_g,
                      got_g, dev),
                   expected=want_g, actual=got_g, deviation=dev))
        else:
            add(_f("gravity", SEV_INFO,
                   "gravity %s matches the declared %g m/s^2 on %s"
                   % (got_g, world["gravity"], world["coordinateSystem"])))
        if got_g is not None and all(abs(c) < 1e-9 for c in got_g) \
                and world.get("gravity"):
            add(_f("gravity", SEV_ERROR,
                   "the model has ZERO gravity while the world declares %g -- "
                   "the c77cbe98 signature (coordinateSystem never reaching the "
                   "solver: bodies never fall)" % world["gravity"]))

    # --- timestep ---------------------------------------------------------
    # ⚠ THE DUMP'S `opt.timestep` IS NOT THE TIMESTEP THE SOLVER STEPS AT, and
    # this check used to treat it as though it were.
    #
    # OmNewtonBackend assigns `_sv.mj_model.opt.timestep = sub_dt` INSIDE the
    # per-tick step loop, whereas OMNISIM_NEWTON_SAVE_MJCF /
    # OMNISIM_NEWTON_DUMP_MJMODEL write the model at world FINALIZE -- before
    # any tick has run. So the dumped value is a construction-time default.
    # Measured 2026-08-09 on three lane-1 worlds whose basicTimeStep is 1, 4
    # and 16 ms: ALL THREE dump `timestep=0.002`. It tracks nothing.
    #
    # What that did here: `substeps = basicTimeStep / 0.002` gave 2 for a 4 ms
    # world (truth: 1) and 8 for a 16 ms world (truth: 1) as INFO, and for any
    # world under 2 ms the ratio fell below 1 and this raised a **false
    # ERROR** -- `--world t3_roll_dt1.wbt` exited 1 on a correct world. It
    # stayed hidden because `--lane1` only audits at dt = 4 ms, where the
    # wrong answer is merely wrong rather than red.
    #
    # The substep count is knowable WITHOUT the dump -- it is the engine's own
    # precedence, env > WorldInfo.newtonSubsteps > 1 -- and sub_dt then follows
    # by construction. So that is what is checked, and the dumped value is
    # reported as the construction artefact it is rather than asserted on.
    want_dt = (world.get("basicTimeStep") or 0) / 1000.0
    got_dt = model.get("timestep")
    if want_dt:
        env_ss = os.environ.get("OMNISIM_NEWTON_SUBSTEPS")
        src = "OMNISIM_NEWTON_SUBSTEPS"
        n = None
        if env_ss is not None:
            try:
                n = max(1, int(env_ss))
            except ValueError:
                n = None
        if n is None:
            ws = world.get("newtonSubsteps")
            if ws is not None:
                n, src = max(1, int(ws)), "WorldInfo.newtonSubsteps"
            else:
                n, src = 1, "default"
        sub_dt = want_dt / n
        add(_f("timestep", SEV_INFO,
               "basicTimeStep %g ms / %d substeps (%s) = sub_dt %g s. NOTE the "
               "dumped opt.timestep (%s) is a CONSTRUCTION-TIME default -- the "
               "step loop overwrites it every tick, so it is not evidence "
               "either way" % (world["basicTimeStep"], n, src, sub_dt,
                               ("%g s" % got_dt) if got_dt else "absent"),
               substeps=n, sub_dt_s=sub_dt, dumped_timestep_s=got_dt,
               dumped_timestep_is_authoritative=False))

    # --- friction ---------------------------------------------------------
    geoms = model.get("geoms", [])
    mu = world.get("newtonGroundMu")
    if geoms:
        frictionless = [g for g in geoms if g["condim"] == 1]
        if frictionless and mu:
            add(_f("condim", SEV_ERROR,
                   "%d of %d geoms have condim=1 (FRICTIONLESS: mu is not "
                   "consulted at all) while the world declares newtonGroundMu "
                   "%g -- the declared friction cannot reach the contact"
                   % (len(frictionless), len(geoms), mu),
                   frictionless=[g["name"] for g in frictionless[:8]]))
        # The declared-vs-effective friction contradiction, with NUMBERS. The
        # engine warns about this at load; the audit records what the model
        # actually got, because "the field is ignored" and "the contact runs at
        # 1.0 while the file says 0" are different sizes of wrong.
        cf = world.get("coulombFriction")
        if cf is not None:
            eff = sorted({round(g["friction"][0], 6) for g in geoms
                          if g["friction"]})
            if not (len(eff) == 1 and abs(eff[0] - cf) < 1e-6):
                add(_f("friction_contract", SEV_ERROR,
                       "the world declares ContactProperties.coulombFriction "
                       "%g, but that is an ODE-path field the Newton backend "
                       "does not read: the model's tangential friction is %s. "
                       "The world is not self-describing -- a reader of the "
                       ".wbt cannot predict the contact it will get"
                       % (cf, eff), declared=cf, effective=eff))
        if mu is not None:
            hit = [g for g in geoms if g["friction"]
                   and abs(g["friction"][0] - mu) < 1e-6]
            if not hit:
                seen = sorted({round(g["friction"][0], 4) for g in geoms
                               if g["friction"]})
                add(_f("friction", SEV_WARN,
                       "no geom carries the declared newtonGroundMu %g; the "
                       "model's tangential frictions are %s" % (mu, seen),
                       declared=mu, model_values=seen))
            else:
                add(_f("friction", SEV_INFO,
                       "%d/%d geoms carry the declared mu %g"
                       % (len(hit), len(geoms), mu)))
    else:
        add(_f("geoms", SEV_ERROR,
               "the model has NO geoms -- nothing in this world can collide. A "
               "Solid collides only through its boundingObject; the file "
               "declares %d Solid(s) and %d boundingObject(s)"
               % (world.get("n_solid", 0), world.get("n_bounding_objects", 0))))

    # --- mass -------------------------------------------------------------
    bodies = model.get("bodies", [])
    # body 0 is MuJoCo's static worldbody and is massless by construction.
    movable = [b for b in bodies if b["id"] != 0]
    bad = [b for b in movable if not (b["mass"] == b["mass"])  # NaN
           or b["mass"] < 0]
    if bad:
        add(_f("mass", SEV_ERROR,
               "%d bodies have a non-finite or negative mass: %s"
               % (len(bad), [(b["name"], b["mass"]) for b in bad[:6]])))
    if movable and all(b["mass"] == 0.0 for b in movable):
        add(_f("mass", SEV_ERROR,
               "every non-world body has mass 0 -- nothing can respond to a "
               "force (%d bodies)" % len(movable)))

    # --- inertia: checkable from first principles -------------------------
    # A rigid body's principal moments must be POSITIVE and satisfy the triangle
    # inequality (Ia + Ib >= Ic for every permutation). No reference model is
    # needed and no scene knowledge is needed -- any violation is a physically
    # impossible body, whatever produced it. This is the generic form of a defect
    # that actually shipped: a husky-wheel inertia preset applied to every Solid
    # without an explicit inertiaMatrix.
    for b in bodies:
        I = b.get("inertia")
        if not I or len(I) != 3 or b["id"] == 0:
            continue                       # worldbody is massless by construction
        if b["mass"] <= 0:
            continue                       # a static body carries no inertia
        if any(v <= 0 for v in I):
            add(_f("inertia", SEV_ERROR,
                   "body %r has a non-positive principal moment %s — not a "
                   "physically possible rigid body" % (b["name"], I),
                   body=b["name"], inertia=I))
            continue
        a, bb, c = sorted(I)
        if a + bb < c * (1 - 1e-6):
            add(_f("inertia", SEV_ERROR,
                   "body %r violates the inertia triangle inequality: %s "
                   "(sorted %g + %g < %g). No rigid body has this inertia "
                   "tensor, so the value was fabricated rather than derived "
                   "from geometry" % (b["name"], I, a, bb, c),
                   body=b["name"], inertia=I))

    # --- joint limits ------------------------------------------------------
    # A limit the .wbt declared and the model did not receive is invisible to
    # every trajectory metric until something drives into it.
    for j in model.get("joints", []):
        r = j.get("range") or []
        if j.get("limited") and len(r) == 2 and r[0] >= r[1]:
            add(_f("joint_limits", SEV_ERROR,
                   "joint %r is limited but its range is empty or inverted %s"
                   % (j["name"], r), joint=j["name"], range=r))

    # --- actuators ---------------------------------------------------------
    # AGENTS.md records motorised BallJoint/Hinge2Joint as accepted-and-silently-
    # ignored. A world whose motors produced NO actuators is that defect, and it
    # is otherwise only visible as "the robot does not move".
    if world.get("n_motor_devices") and not model.get("n_act"):
        add(_f("actuators", SEV_ERROR,
               "the world declares %d motor device(s) but the model has ZERO "
               "actuators: those motors are accepted and silently ignored "
               "(the known BallJoint / Hinge2Joint defect), so setPosition and "
               "setVelocity reach nothing"
               % world["n_motor_devices"], declared=world["n_motor_devices"]))

    # --- declared-but-unread fields ---------------------------------------
    for field in world.get("declared_ignored", []):
        alt = IGNORED_FIELDS.get(field)
        add(_f("ignored_fields", SEV_WARN,
               "the world declares %r, which the engine PARSES BUT DOES NOT "
               "READ%s. Anything this world asserts through that field is not "
               "in the model above."
               % (field, " -- use %s instead" % alt if alt else
                  " (the capability was removed with ODE)"),
               field=field, replacement=alt))

    return findings


def audit_world(world, timeout_s=180):
    w = parse_world(world)
    model, meta, tmp = dump_model(world, timeout_s=timeout_s)
    findings = audit(w, model, meta)
    return {
        "world": str(world),
        "authored": w,
        "model": {k: v for k, v in model.items()
                  if k not in ("bodies", "geoms", "dofs")},
        "n_bodies": len(model.get("bodies", [])),
        "n_geoms": len(model.get("geoms", [])),
        "run": meta,
        "findings": findings,
        "errors": sum(1 for f in findings if f["severity"] == SEV_ERROR),
        "warnings": sum(1 for f in findings if f["severity"] == SEV_WARN),
    }


# --------------------------------------------------------------------------
# static sweep: the corpus-scale half, with no engine

def audit_static(world):
    """File-side findings only -- no engine, no load, milliseconds per world.

    The systemic corpus defect is statically DECIDABLE, which is what makes a
    whole-tree sweep affordable: `ContactProperties.coulombFriction` is an
    ODE-path field the Newton backend does not read, and `newtonGroundMu`
    defaults to 1.0 when absent. So a world that declares `coulombFriction 0.5`
    and no `newtonGroundMu` provably runs at 1.0 -- no engine needed to know it.

    Soundness note: this reasons about the world AS AUTHORED. A launcher that
    exports `OMNISIM_NEWTON_GROUND_MU` (as lane 1's runner legitimately does)
    changes what the run measures but not what the FILE says, and the file is
    what a third party gets. Findings are reported in those terms.
    """
    w = parse_world(world)
    findings = []
    cf, mu = w.get("coulombFriction"), w.get("newtonGroundMu")
    if cf is not None and not w.get("coulombFriction_uniform", True):
        # A different, HARDER defect than "the value never arrives": the value
        # cannot even be expressed. Newton exposes one global friction, so a
        # world declaring per-material pairs loses that structure entirely --
        # no rewrite fixes it, and reporting it as the same finding would imply
        # the migration could.
        findings.append(_f(
            "friction_per_material", SEV_ERROR,
            "declares %d per-material frictions %s via contactProperties, but "
            "Newton exposes ONE global friction (newtonGroundMu): the "
            "per-material structure cannot be represented at all, and the "
            "contact runs at the default 1.0"
            % (len(w["coulombFriction_all"]),
               sorted(set(w["coulombFriction_all"]))),
            declared=sorted(set(w["coulombFriction_all"]))))
    elif cf is not None:
        if mu is None:
            if abs(cf - 1.0) > 1e-9:
                findings.append(_f(
                    "friction_unreachable", SEV_ERROR,
                    "declares ContactProperties.coulombFriction %g and no "
                    "newtonGroundMu, so the contact runs at the engine default "
                    "1.0: the declared friction cannot reach the solver"
                    % cf, declared=cf, effective=1.0))
            else:
                findings.append(_f(
                    "friction_unreachable", SEV_WARN,
                    "declares coulombFriction 1.0, which the engine does not "
                    "read -- it coincides with the default, so the physics is "
                    "right by accident rather than by declaration"))
        elif abs(cf - mu) > 1e-9:
            findings.append(_f(
                "friction_contradiction", SEV_ERROR,
                "declares coulombFriction %g AND newtonGroundMu %g -- two "
                "different frictions, only the second of which is read"
                % (cf, mu), declared=cf, effective=mu))
    for field in w.get("declared_ignored", []):
        if field == "coulombFriction" and cf is not None:
            continue          # already reported above, with numbers
        alt = IGNORED_FIELDS.get(field)
        findings.append(_f("ignored_fields", SEV_WARN,
                           "declares %r, which is parsed but not read%s"
                           % (field, " (use %s)" % alt if alt else ""),
                           field=field))
    return w, findings


# Paths whose `.wbt` files are NOT part of the live corpus. Counting them
# inflates the defect number and, worse, invites "fixing" them: a world under
# `results/` is a RECORDED ARTEFACT of what some agent produced on a date, and
# rewriting it falsifies the record. Learned by counting 259 and finding most of
# them were ladder-run deliverables and worktree copies.
SWEEP_EXCLUDE_DIRS = (
    ".claude",          # worktree copies of the whole tree
    "results",          # recorded run artefacts (ladder cells, campaign output)
    "_scratch",
    "__pycache__",
    "node_modules",
    ".build_tmp",
)
# Engine/harness scratch copies, written next to a world at run time.
SWEEP_EXCLUDE_PREFIXES = (".omnisim_", ".harness_", ".capture_")


def _in_live_corpus(path, root):
    rel = path.relative_to(root)
    if any(part in SWEEP_EXCLUDE_DIRS for part in rel.parts):
        return False
    return not path.name.startswith(SWEEP_EXCLUDE_PREFIXES)


# Launch scripts align their assignments, so allow generous non-numeric padding
# between the name and the value: `$env:OMNISIM_NEWTON_GROUND_MU   = "1.5"`,
# `export OMNISIM_NEWTON_GROUND_MU=2.0`, `set OMNISIM_NEWTON_GROUND_MU=0.5`.
# A too-tight bound (12) silently matched nothing and let the guard pass.
_LAUNCHER_MU_RE = re.compile(
    r"OMNISIM_NEWTON_GROUND_MU[^0-9\n]{0,40}?([0-9]*\.?[0-9]+)")


def launcher_friction(world_path):
    """The friction a sibling launch script forces for this world, if any.

    A world is often launched by a `run_<stem>.*` script next to it that exports
    OMNISIM_NEWTON_GROUND_MU, and the env var BEATS the field. If that value
    disagrees with what the world declares, the world and the run are two
    different experiments and no rewrite can reconcile them -- see `apply_fix`.
    Returns None when there is no such script or no such export.
    """
    world = Path(world_path)
    for cand in sorted(world.parent.glob("run_%s.*" % world.stem)):
        try:
            text = cand.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        m = _LAUNCHER_MU_RE.search(text)
        if m:
            try:
                return float(m.group(1))
            except ValueError:
                pass
    return None


def apply_fix(path):
    """Declare the world's own friction in the field the engine reads.

    Inserts `newtonGroundMu <coulombFriction>` into WorldInfo. This is a
    SELF-DESCRIPTION fix, not a retune: it makes a bare load of the file behave
    the way the file already claims. For any launcher that exports
    OMNISIM_NEWTON_GROUND_MU the change is numerically inert -- the env var
    takes precedence (`_contact_value`: env > field > default) -- so the runs a
    project actually performs are unaffected, and only the previously-broken
    bare load moves.

    ⚠ It still CHANGES what a bare load simulates (from the default 1.0 to the
    declared value), so it is opt-in per directory and never applied to
    recorded artefacts. Returns (changed: bool, reason: str).
    """
    raw = Path(path).read_bytes()
    crlf = b"\r\n" in raw
    text = raw.decode("utf-8", errors="replace")
    if crlf:
        text = text.replace("\r\n", "\n")

    w = parse_world(path)
    cf, mu = w.get("coulombFriction"), w.get("newtonGroundMu")
    if cf is None:
        return False, "declares no coulombFriction"
    if mu is not None:
        return False, "already declares newtonGroundMu %g" % mu
    if abs(cf - 1.0) <= 1e-9:
        return False, "declared friction equals the engine default"
    # μ=0 IS expressible since the sentinel fix (2026-08-09): `newtonGroundMu`
    # defaults to -1 and negative means unset, so 0 now declares a frictionless
    # world instead of silently meaning "use 1.0". ⚠ A world written with
    # `newtonGroundMu 0` therefore behaves DIFFERENTLY on a binary older than
    # that fix, where 0 still reads as unset -> 1.0. That is a real version
    # split and the reason the field carries a comment rather than a bare number.
    if not w.get("coulombFriction_uniform", True):
        return False, ("declares %d different per-material frictions %s; "
                       "newtonGroundMu is a single GLOBAL value and cannot "
                       "represent them — needs per-material contact support, "
                       "not a rewrite"
                       % (len(w["coulombFriction_all"]),
                          sorted(set(w["coulombFriction_all"]))))
    launcher_mu = launcher_friction(path)
    if launcher_mu is not None and abs(launcher_mu - cf) > 1e-9:
        # THE RULE I BROKE ONCE, NOW ENFORCED. Three OMNIARM6 demos were rewritten
        # to declare `newtonGroundMu 5` from their own coulombFriction while
        # their run_*.ps1 exports 1.5. The edit is inert (env wins), but it
        # makes the file AUTHORITATIVELY state a friction the sanctioned run
        # does not use -- which is the exact objection that keeps this migration
        # away from projects/policies. Somebody has to decide which number is
        # intended; a sweep cannot.
        return False, ("declares coulombFriction %g but a sibling launcher "
                       "exports OMNISIM_NEWTON_GROUND_MU=%g — the world and the "
                       "run disagree, and declaring either would make the file "
                       "authoritative without making it true. Reconcile them by "
                       "hand." % (cf, launcher_mu))

    m = re.search(r"\bWorldInfo\s*\{", text)
    if not m:
        return False, "no WorldInfo block"
    depth, close = 0, None
    for j in range(m.end() - 1, len(text)):
        if text[j] == "{":
            depth += 1
        elif text[j] == "}":
            depth -= 1
            if depth == 0:
                close = j
                break
    if close is None:
        return False, "unterminated WorldInfo block"

    line_start = text.rfind("\n", 0, close) + 1
    indent = "  "
    ins = ("%s# translation_audit: ContactProperties.coulombFriction is an "
           "ODE-path field the\n"
           "%s# Newton backend does not read; this declares the same friction "
           "where it is read.\n"
           "%snewtonGroundMu %s\n" % (indent, indent, indent, _fmt_num(cf)))
    text = text[:line_start] + ins + text[line_start:]

    if crlf:
        text = text.replace("\n", "\r\n")
    Path(path).write_bytes(text.encode("utf-8"))
    return True, "declared newtonGroundMu %g" % cf


def _fmt_num(x):
    s = "%.10g" % x
    return s


def sweep(root, quiet=True, include_artifacts=False):
    """Static-audit every .wbt under `root`. Returns (reports, counts).

    Restricted to the LIVE corpus by default -- see `SWEEP_EXCLUDE_DIRS`.
    """
    root = Path(root)
    worlds = sorted(p for p in [*root.rglob("*.omniworld"), *root.rglob("*.wbt")]
                    if include_artifacts or _in_live_corpus(p, root))
    reports, n_err, n_warn = [], 0, 0
    by_check = {}
    for wp in worlds:
        try:
            w, findings = audit_static(str(wp))
        except Exception as e:                      # a malformed world is data
            findings = [_f("parse", SEV_WARN,
                           "could not read: %s" % type(e).__name__)]
            w = {}
        errs = [f for f in findings if f["severity"] == SEV_ERROR]
        n_err += len(errs)
        n_warn += sum(1 for f in findings if f["severity"] == SEV_WARN)
        for f in findings:
            key = "%s/%s" % (f["severity"], f["check"])   # str: JSON-writable
            by_check[key] = by_check.get(key, 0) + 1
        if findings:
            reports.append({"world": str(wp), "findings": findings,
                            "errors": len(errs)})
        if errs and not quiet:
            print("  ERROR %s: %s" % (wp.name, errs[0]["message"]))
    return reports, {"worlds": len(worlds), "errors": n_err,
                     "warnings": n_warn, "by_check": by_check}


# --------------------------------------------------------------------------
# self-test: prove the audit can DISCRIMINATE, not merely print

_PROBE_WORLD = """#VRML_SIM R2025a utf8

# GENERATED by translation_audit.py --self-test — do not hand-edit.
WorldInfo {
  gravity %(gravity)g
  basicTimeStep 4
  FPS 30
  randomSeed 42
  coordinateSystem "%(cs)s"
  newtonSolver "mujoco"
  newtonStatics TRUE
%(mu_line)s}
Viewpoint {
  position -4 -6 3
  orientation -0.15 0.05 0.99 1.2
}
Background {
  skyColor [ 0.15 0.17 0.22 ]
}
DEF FLOOR Solid {
  translation 0 0 -0.1
  name "floor"
  children [
    DEF FLOOR_SHAPE Shape {
      appearance PBRAppearance { baseColor 0.5 0.5 0.55 roughness 1 metalness 0 }
      geometry Box { size 4 4 0.2 }
    }
  ]
  boundingObject USE FLOOR_SHAPE
}
DEF BALL Solid {
  translation 0 0 1.1
  name "ball"
  children [
    DEF BALL_SHAPE Shape {
      appearance PBRAppearance { baseColor 0.3 0.65 0.85 roughness 0.9 metalness 0 }
      geometry Sphere { radius 0.1 subdivision 3 }
    }
  ]
  boundingObject USE BALL_SHAPE
  physics Physics { density -1 mass 1 }
}
DEF OMNIBENCH_RECORDER Robot {
  name "omnibench_recorder"
  controller "omnibench_recorder"
  controllerArgs [ "--bodies=BALL" "--duration=2" "--label=probe" ]
  supervisor TRUE
}
"""

# Each case pins a plumbing path that HAS been broken in this tree. They are
# differential by construction: if `coordinateSystem` stopped reaching the
# solver, `nue_earth` fails while `enu_earth` still passes, which is the
# signature c77cbe98 fixed and no single-arm probe could have separated.
SELF_TEST_CASES = [
    {"name": "enu_earth", "gravity": 9.81, "cs": "ENU", "mu": None,
     "why": "control: the default convention everything else is compared to"},
    {"name": "nue_earth", "gravity": 9.81, "cs": "NUE", "mu": None,
     "why": "coordinateSystem must reach the solver (c77cbe98: all 210 NUE "
            "worlds ran at gravity 0 and never fell)"},
    {"name": "eun_earth", "gravity": 9.81, "cs": "EUN", "mu": None,
     "why": "the third convention, Up on +Y like NUE"},
    {"name": "enu_mars", "gravity": 3.72, "cs": "ENU", "mu": None,
     "why": "WorldInfo.gravity must reach the solver (it once did not: every "
            "Newton world ran at -9.81 regardless of the field)"},
    {"name": "enu_mu", "gravity": 9.81, "cs": "ENU", "mu": 0.3,
     "why": "newtonGroundMu must reach the contact"},
]


def self_test(timeout_s=180, keep=None):
    """Generate probe worlds, audit each, and assert the expected verdict.

    A green audit means nothing unless the audit can also go red. This runs the
    positive arms live (they must produce NO error) and one synthetic negative
    arm offline (a deliberately mismatched model, which MUST produce an error).
    """
    # Probe worlds MUST live in lane1/worlds/, not a temp dir. A controller is
    # resolved relative to the world's project layout (worlds/ and controllers/
    # as siblings), so a world in /tmp cannot find `omnibench_recorder`, never
    # terminates, and every probe burns the full launch timeout -- 180 s of
    # looking like a slow audit rather than a broken one. Written with a
    # reserved prefix and removed afterwards.
    out_dir = keep or str(HERE / "worlds")
    os.makedirs(out_dir, exist_ok=True)
    rows, failures, written = [], [], []

    for case in SELF_TEST_CASES:
        mu_line = ("  newtonGroundMu %g\n" % case["mu"]) if case["mu"] else ""
        text = _PROBE_WORLD % {"gravity": case["gravity"], "cs": case["cs"],
                               "mu_line": mu_line}
        wpath = os.path.join(out_dir, "_audit_probe_" + case["name"] + ".wbt")
        Path(wpath).write_text(text, encoding="utf-8")
        written.append(wpath)
        print("[self-test] %-10s %s" % (case["name"], case["why"]), flush=True)
        rep = audit_world(wpath, timeout_s=timeout_s)

        want = expected_gravity_vec(case["gravity"], case["cs"])
        got = (rep.get("model") or {}).get("gravity")
        grav = [f for f in rep["findings"] if f["check"] == "gravity"]
        evaluated = any(f["severity"] == SEV_INFO for f in grav)
        if not evaluated:
            failures.append("%s: the gravity check did not EVALUATE (%s)"
                            % (case["name"], [f["message"][:70] for f in grav]))
        if rep["errors"]:
            failures.append("%s: unexpected ERROR findings: %s"
                            % (case["name"],
                               [f["message"][:90] for f in rep["findings"]
                                if f["severity"] == SEV_ERROR]))
        rows.append({"case": case["name"], "expected_gravity": want,
                     "model_gravity": got, "errors": rep["errors"],
                     "evaluated": evaluated})
        print("           expected %s  model %s  -> %s"
              % (want, got, "OK" if evaluated and not rep["errors"] else "FAIL"))

    # NEGATIVE ARM (offline): hand the comparator a model that disagrees with
    # the world and require it to say so. Without this the whole suite could be
    # a function that returns "fine".
    fake_world = {"gravity": 9.81, "coordinateSystem": "ENU",
                  "declared_ignored": [], "n_solid": 1,
                  "n_bounding_objects": 1, "basicTimeStep": 4.0}
    fake_model = {"gravity": [0.0, 0.0, -9.81 / 2], "timestep": 0.002,
                  "bodies": [{"id": 0, "name": "world", "mass": 0.0},
                             {"id": 1, "name": "b", "mass": 1.0}],
                  "geoms": [{"id": 0, "name": "g", "body": 0, "type": 0,
                             "condim": 3, "contype": 1, "conaff": 1,
                             "friction": [1.0, 0.005, 0.0]}]}
    ok_verdict = {"present": True, "degraded": False, "finalised": True,
                  "solver": "mujoco"}
    neg = audit(fake_world, fake_model, {"verdict": ok_verdict})
    caught = [f for f in neg if f["check"] == "gravity"
              and f["severity"] == SEV_ERROR]
    print("[self-test] negative  a model whose gravity is HALF the declared "
          "value must raise ERROR")
    print("           -> %s" % ("OK" if caught else "FAIL — the audit cannot "
                                "go red, so its green means nothing"))
    if not caught:
        failures.append("negative arm: a halved-gravity model raised no ERROR")

    # A zero-gravity model must also be caught by its own named signature.
    neg0 = audit(fake_world, dict(fake_model, gravity=[0.0, 0.0, 0.0]),
                 {"verdict": ok_verdict})
    if not any("ZERO gravity" in f["message"] for f in neg0):
        failures.append("negative arm: a zero-gravity model did not raise the "
                        "c77cbe98 signature")

    # The probe worlds are scaffolding, not scenes: gen_worlds.py owns this
    # directory and a stray _audit_probe_*.wbt would look like a lane-1 world.
    if not keep:
        for p in written:
            try:
                os.remove(p)
            except OSError:
                pass

    print()
    if failures:
        print("[self-test] FAILED:")
        for f in failures:
            print("  - %s" % f)
        return 1, rows
    print("[self-test] PASS — %d live probes evaluated, negative arms caught"
          % len(rows))
    return 0, rows


def main():
    ap = argparse.ArgumentParser(
        description="Audit the MuJoCo model OmniSim builds against the .wbt it "
                    "was built from (the translation layer no external arm sees).")
    ap.add_argument("--world", help="path to a .wbt")
    ap.add_argument("--lane1", action="store_true",
                    help="audit every lane-1 scene at dt=4 ms")
    ap.add_argument("--json", help="write the full report here")
    ap.add_argument("--timeout", type=int, default=180)
    ap.add_argument("--quiet", action="store_true",
                    help="suppress INFO findings")
    ap.add_argument("--sweep", metavar="DIR",
                    help="STATIC audit of every .wbt under DIR -- no engine, "
                         "milliseconds per world. Answers the corpus-scale "
                         "question (does the file declare physics that cannot "
                         "reach the solver?) without 200 engine loads.")
    ap.add_argument("--fix", action="store_true",
                    help="with --sweep: rewrite each affected world to declare "
                         "newtonGroundMu = its own coulombFriction. Inert for "
                         "any launcher that exports OMNISIM_NEWTON_GROUND_MU "
                         "(env wins); changes what a BARE load simulates. "
                         "Never touches recorded artefacts.")
    ap.add_argument("--include-artifacts", action="store_true",
                    help="also sweep recorded run artefacts (results/, "
                         "worktree copies, harness scratch worlds). They are "
                         "excluded by default: a world under results/ is a "
                         "record of what an agent produced, not a world anyone "
                         "loads, and it must never be 'fixed'.")
    ap.add_argument("--self-test", action="store_true",
                    help="prove the audit can discriminate: live probes for "
                         "each coordinate system + gravity value, plus a "
                         "synthetic mismatch that MUST be caught")
    args = ap.parse_args()

    if args.self_test:
        rc, rows = self_test(timeout_s=args.timeout)
        if args.json:
            Path(args.json).write_text(json.dumps(rows, indent=2),
                                       encoding="utf-8")
        return rc

    if args.sweep:
        reports, counts = sweep(args.sweep, quiet=args.quiet,
                                include_artifacts=args.include_artifacts)
        worst = sorted(reports, key=lambda r: -r["errors"])
        for rep in worst:
            if not rep["errors"]:
                continue
            print("%s" % Path(rep["world"]).relative_to(Path(args.sweep)))
            for f in rep["findings"]:
                if f["severity"] == SEV_ERROR:
                    print("  ERROR %-24s %s" % (f["check"], f["message"]))
        print("\n[sweep] %d worlds scanned: %d ERROR, %d WARN, %d world(s) "
              "with at least one ERROR"
              % (counts["worlds"], counts["errors"], counts["warnings"],
                 sum(1 for r in reports if r["errors"])))
        for key, n in sorted(counts["by_check"].items(), key=lambda kv: -kv[1]):
            print("   %-32s %d" % (key, n))
        if args.fix:
            fixed, skipped = 0, []
            for rep in reports:
                if not rep["errors"]:
                    continue
                changed, why = apply_fix(rep["world"])
                if changed:
                    fixed += 1
                else:
                    skipped.append((Path(rep["world"]).name, why))
            print("\n[fix] rewrote %d world(s)" % fixed)
            for name, why in skipped:
                print("   SKIP %-44s %s" % (name, why))
        if args.json:
            Path(args.json).write_text(
                json.dumps({"counts": counts, "reports": reports}, indent=2),
                encoding="utf-8")
            print("[sweep] report -> %s" % args.json)
        return 1 if counts["errors"] and not args.fix else 0

    worlds = []
    if args.lane1:
        worlds = sorted(str(p) for p in (HERE / "worlds").glob("*_dt4.wbt"))
        if not worlds:
            print("no lane-1 worlds; run gen_worlds.py first")
            return 2
    elif args.world:
        worlds = [args.world]
    else:
        ap.error("pass --world or --lane1")

    reports, n_err = [], 0
    for w in worlds:
        print("[audit] %s ..." % Path(w).name, flush=True)
        rep = audit_world(w, timeout_s=args.timeout)
        reports.append(rep)
        n_err += rep["errors"]
        for f in rep["findings"]:
            if args.quiet and f["severity"] == SEV_INFO:
                continue
            print("  %-5s %-16s %s" % (f["severity"], f["check"], f["message"]))

    print("\n[audit] %d world(s): %d ERROR, %d WARN"
          % (len(reports), n_err, sum(r["warnings"] for r in reports)))
    if args.json:
        Path(args.json).write_text(json.dumps(reports, indent=2), encoding="utf-8")
        print("[audit] report -> %s" % args.json)
    return 1 if n_err else 0


if __name__ == "__main__":
    sys.exit(main())
