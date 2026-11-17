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

"""rung18.py -- bare MuJoCo replaying RECORDED REAL cube tosses.

THIS ARM'S ROW IS THE REFERENCE ROW, and that is the only reason rung 18 has a
MuJoCo column at all.  ``rungs.check_rung18_embed_gap`` scores every other arm
on ``|its mean position error - THIS arm's, on the same tosses|``.  OmniSim's
solver IS MuJoCo, reached through a ``.wbt`` -> newton -> mjModel translation
layer, so a gap between the two columns is that layer and not the physics.  It
is the one check on the ladder we can lose and cannot win, and this file is the
ruler it is measured with -- which means every shortcut taken here comes back as
somebody else's unexplained deficit.

WHAT IS READ RATHER THAN DECLARED
---------------------------------
``tests/benchmarks/omnibench/lane1r`` owns the recording, its licence, the
cube's mass / geometry / inertia, the sampling rate, the quaternion convention,
the body-to-world angular-velocity rotation and the table's geometry.  All of it
is imported by path and used unmodified: the dataset module through the
contract's ``rungs.rung18_dataset()``, the scene through ``scenes.mjcf(18)``
(which reads lane1r's own ``.wbt``), and the replay grid through lane1r's
``run.py``.  **If this ladder and lane1r ever disagree, lane1r is right**, and
the only way to keep that true is to hold no second copy of any of it.

FOUR THINGS THAT ARE EASY TO GET WRONG AND SILENT WHEN YOU DO
-------------------------------------------------------------
1. **The dataset's omega is in the BODY frame** (lane1r measured that: 93x
   separation against the next hypothesis).  It is rotated to world by
   ``D.omega_to_world`` and then back into the body frame for MuJoCo -- see 2.
   Passing the raw column through leaves every toss spinning about the wrong
   axis while still looking like a plausible tumble.  That is this rung's
   ``wrong_omega_frame`` fault, and it exists because it is the mistake.

2. **A MuJoCo free joint's ``qvel`` is world-frame LINEAR and body-frame
   ANGULAR.**  Measured here rather than remembered: with the body rotated 90
   degrees about z, ``qvel[3:6] = (1,0,0)`` reads back through
   ``mj_objectVelocity`` as a world angular velocity of ``(0,1,0)``.  So the
   world-frame omega the recording asks for has to be rotated INTO the body
   frame on the way in, and the readback rotated back OUT on the way out.  A
   harness that skipped both would be self-consistent and wrong.

3. **The IC readback is a readback.**  ``got_vel`` / ``got_omega_world`` come
   out of ``data.qvel`` after the engine has stepped, never from the request.
   This tree has shipped ``setVelocity`` being silently dropped at t = 0, which
   would otherwise present here as poor real-world agreement and be charged to
   the contact model.

4. **The recording grid is lane1r's, including its one-step offset.**  lane1r's
   controller sets the IC, steps ONCE to verify it took, and only then records
   its ``t = 0`` sample -- so its published trajectory carries a 1 ms label
   offset.  This file reproduces that exactly.  It is not the offset that would
   be right in isolation; it is the one the OmniSim arm also has, and
   ``embed_gap`` is a DIFFERENCE of the two arms' errors against the same
   recording.  A "corrected" grid here would move this arm by ~2 mm at typical
   toss speeds -- 2% of cube width, against a 5-point tolerance -- and charge
   it to OmniSim's translation layer.

THE WHOLE ``WorldInfo`` IS TRANSLATED, INCLUDING THE CONTACT DECLARATION
-----------------------------------------------------------------------
lane1r's world declares ``newtonCone "elliptic"`` and ``newtonImpratio 10``.
Both are read out of that file and translated into MJCF ``cone`` / ``impratio``
by ``scenes.rung18_declarations()``, which carries the rule, the citation and
the honest note about when the decision was taken.  The short version: rungs
0-8 run MuJoCo's own defaults because their scenes are the LADDER's; rung 18's
scene is lane1r's, and translating a world's masses while dropping the two
fields that say how its friction is solved replays a different scene and
reports it as lane1r's.

IT IS NOT A SMALL EFFECT AND BOTH RUNS ARE PUBLISHED.  Measured here over the
contract's 50 tosses: MuJoCo's default pyramidal cone scores 43.023 % of cube
width and the elliptic cone scores 24.845 %.  The second reproduces OmniSim's
own published lane1r campaign on the same 50 tosses -- 24.845205 % -- to five
decimal places, which is the strongest evidence available that the
``.wbt`` -> newton -> mjModel translation layer costs essentially nothing on
this rung and that the 18-point apparent deficit was a cone shape.  Reproduce
the whole table with ``python rung18.py --sweep``; it is not a ladder row.
"""

from __future__ import annotations

import importlib.util
import inspect
import os
import sys
import time

HERE = os.path.abspath(os.path.dirname(__file__))
LADDER0 = os.path.dirname(HERE)


def _load_shared():
    key = "ladder0_mujoco_shared"
    if key in sys.modules:
        return sys.modules[key]
    sp = importlib.util.spec_from_file_location(
        key, os.path.join(HERE, "shared.py"))
    mod = importlib.util.module_from_spec(sp)
    sys.modules[key] = mod
    sp.loader.exec_module(mod)
    return mod


shared = _load_shared()
spec = shared.spec
scenes = shared.sibling("scenes")

SIM = "mujoco"

def _run_module():
    """This arm's ``run.py``, however it happens to be loaded.

    ``sys.modules`` may already hold it as ``ladder0_mujoco_run`` (through
    ``shared.sibling``) or as ``__main__`` (``python run.py --rung 18``), and
    executing the file a second time under a second name would put two
    catalogues in one process -- exactly the split ``shared.py`` exists to
    prevent.  So it is looked up BY FILE first and only loaded if absent.
    """
    want = os.path.join(HERE, "run.py")
    for mod in list(sys.modules.values()):
        f = getattr(mod, "__file__", None)
        if f and os.path.abspath(f) == want:
            return mod
    return shared.sibling("run")


#: Faults this file injects.  Anything else is REPORTED as unsupported rather
#: than quietly run as a control -- "the fault did not happen" must never read
#: as "the fault produced no failure".
#:
#: READ FROM ``run.RUNG_FAULTS``, never re-listed here: the arm's catalogue is
#: one place and this is the other consumer of it.
FAULTS = ("none",) + tuple(_run_module().RUNG_FAULTS[18])

#: This interpreter's own start, epoch seconds.
PROC_START = time.time()


def _lane1r_runner():
    """lane1r's runner module, by path.

    Wanted for exactly two numbers -- ``TABLE_TOP`` and ``DURATION_MS`` -- and
    for its ``run_one`` signature, which is where the 1 ms recording grid is
    declared.  ``omnibench/`` goes on ``sys.path`` because lane1r imports
    ``common``; both entries are APPENDED, so nothing in this ladder can be
    shadowed by a same-named module over there.
    """
    key = "ladder0_lane1r_run"
    if key in sys.modules:
        return sys.modules[key]
    repo = os.path.abspath(os.path.join(LADDER0, os.pardir, os.pardir,
                                        os.pardir))
    lane1r = os.path.join(repo, *spec.RUNG18_LANE1R)
    for p in (os.path.dirname(lane1r), lane1r):
        if p not in sys.path:
            sys.path.append(p)
    path = os.path.join(lane1r, "run.py")
    if not os.path.isfile(path):
        raise ImportError("lane1r runner not found at %s -- rung 18's replay "
                          "grid belongs to that lane and this arm may not "
                          "substitute one of its own" % path)
    sp = importlib.util.spec_from_file_location(key, path)
    mod = importlib.util.module_from_spec(sp)
    sys.modules[key] = mod
    sp.loader.exec_module(mod)
    return mod


def _record_step_ms(lane1r):
    """lane1r's recording interval, in ms, READ from its runner.

    It is the default of ``run_one(step_ms=...)`` and of the ``--step-ms``
    flag, and it is the interval lane1r's controller advances per recorded
    sample.  Read rather than written down, because this arm and the OmniSim
    arm have to record on the SAME grid for ``embed_gap`` to be a difference of
    errors rather than a difference of sampling.
    """
    p = inspect.signature(lane1r.run_one).parameters.get("step_ms")
    if p is None or not isinstance(p.default, int) or p.default < 1:
        raise ValueError("lane1r.run_one no longer declares an integer "
                         "step_ms default; rung 18 cannot infer the recording "
                         "grid and will not guess one")
    return int(p.default)


# --------------------------------------------------------------------------
# one toss
# --------------------------------------------------------------------------

def _reference(traj, index, table_top, edge):
    """The RECORDING, carried in the sample document beside the replay.

    It travels with the run because the shared reducer is simulator-neutral and
    must not import a dataset, and because a document that carries its own
    ground truth can be re-scored after the fact: a mismatch between what was
    replayed and what it was scored against becomes visible rather than assumed.
    """
    return {
        "index": int(index),
        "t": [float(v) for v in traj["t"]],
        "pos": [[float(c) for c in row] for row in traj["pos"]],
        "quat": [[float(c) for c in row] for row in traj["quat"]],
        "cube_edge_m": float(edge),
        "table_top": float(table_top),
        "scale_mode": traj["scale_mode"],
        "source": "DAIRLab/dair_pll ContactNets cube toss, BSD-3-Clause, "
                  "vendored at %s" % "/".join(spec.RUNG18_LANE1R),
    }


def _toss(mj, model, index, fault, D, cfg):
    """Replay one recorded toss.  Returns a run record."""
    import numpy as np

    traj = D.load(index, scale=spec.RUNG18_SCALE)
    q0 = np.asarray(traj["q0"], dtype=float)          # wxyz
    p0 = np.asarray(traj["p0"], dtype=float)
    w0 = np.asarray(traj["w0"], dtype=float)          # BODY frame
    v0 = np.asarray(traj["v0"], dtype=float)          # world frame
    R0 = D.quat_to_matrix(q0)                         # body -> world
    w_world = D.omega_to_world(q0, w0)

    # What the engine is ASKED for, and what the record says was WANTED.  The
    # two differ only under ic_drop_velocity, and that is the entire fault: the
    # engine genuinely never receives the velocity, exactly as a dropped
    # setVelocity at t=0 does, while the record still knows what the toss was
    # supposed to be.
    want_vel, want_omega = v0.copy(), w_world.copy()
    ask_vel, ask_omega = v0.copy(), w_world.copy()
    if fault == "ic_drop_velocity":
        ask_vel = np.zeros(3)
        ask_omega = np.zeros(3)
    elif fault == "wrong_omega_frame":
        # The BODY column handed through as if it were world frame.  ``want``
        # follows the ask, so the IC check correctly stays green: the engine
        # accepted precisely what it was given and the error is the harness's.
        ask_omega = w0.copy()
        want_omega = w0.copy()

    data = mj.MjData(model)
    qa, va = cfg["qpos_adr"], cfg["qvel_adr"]
    data.qpos[qa:qa + 3] = [p0[0], p0[1], p0[2] + cfg["table_top"]]
    data.qpos[qa + 3:qa + 7] = q0
    data.qvel[va:va + 3] = ask_vel
    # World -> body: a MuJoCo free joint's angular qvel is in the BODY frame.
    data.qvel[va + 3:va + 6] = R0.T @ ask_omega
    mj.mj_forward(model, data)

    n_sub = cfg["substeps"]
    body = cfg["body"]

    def advance():
        for _ in range(n_sub):
            mj.mj_step2(model, data)
            mj.mj_step1(model, data)

    def read_world_vel():
        """The engine's OWN state, mapped to world.  Never the request."""
        lin = [float(c) for c in data.qvel[va:va + 3]]
        q = np.asarray(data.qpos[qa + 3:qa + 7], dtype=float)
        ang = D.quat_to_matrix(q) @ np.asarray(data.qvel[va + 3:va + 6],
                                               dtype=float)
        return lin, [float(c) for c in ang]

    # lane1r's convention, matched deliberately: one step so the write lands,
    # the readback, THEN the sample labelled t = 0.  See the module docstring.
    advance()
    got_vel, got_omega = read_world_vel()

    T, P, Q = [], [], []

    def sample(k):
        T.append(k * cfg["step_s"])
        P.append([float(c) for c in data.xpos[body]])
        Q.append([float(c) for c in data.xquat[body]])       # wxyz

    sample(0)
    for k in range(1, cfg["n_steps"] + 1):
        advance()
        sample(k)

    err_v = float(np.linalg.norm(np.asarray(got_vel) - want_vel))
    err_w = float(np.linalg.norm(np.asarray(got_omega) - want_omega))
    ic = {
        "want_vel": [float(c) for c in want_vel], "got_vel": got_vel,
        "want_omega_world": [float(c) for c in want_omega],
        "got_omega_world": got_omega,
        # The readback happens after one step, by which point gravity has
        # legitimately changed vz.  Subtracting exactly that leaves the write.
        "grav_step": D.GRAVITY * cfg["step_s"],
        "vel_err": err_v, "omega_err": err_w,
        "vel_rel": err_v / max(float(np.linalg.norm(want_vel)), 1e-9),
        "omega_rel": err_w / max(float(np.linalg.norm(want_omega)), 1e-9),
        "asked_omega_world": [float(c) for c in ask_omega],
        "omega_body": [float(c) for c in w0],
    }
    return {
        "tag": "toss%04d" % index,
        "params": {"index": int(index)},
        "pid": os.getpid(), "proc_start": PROC_START,
        "t": T, "pos": P, "quat": Q, "ic": ic,
        "reference": _reference(traj, index, cfg["table_top"],
                                D.CUBE_EDGE_M),
        "steps": len(T),
        "physics_steps": (cfg["n_steps"] + 1) * n_sub,
    }


# --------------------------------------------------------------------------

def run(out_dir, fault="none", subset=None, timeout_s=None, **_kw):
    """One rung-18 cell: every toss of the contract's subset.  Never raises."""
    t_cell = time.time()
    fault = fault or "none"
    meta = {
        "sim": SIM, "rung": 18, "fault": fault, "error": None, "exit_code": 0,
        "multi_run": True,
        "engine": "bare mujoco, CPU mj_step (no OmniSim, no Newton, no GPU)",
        "ground_truth": "external measurement of reality (CONTRACT.md "
                        "amendment F): 550 recorded cube tosses, "
                        "DAIRLab/dair_pll, BSD-3-Clause",
        # CONTRACT.md 3b R4: a departure from an engine default is declared or
        # it did not happen.  Filled in below from lane1r's own WorldInfo --
        # never written here -- with the engine default it departs from, one
        # line of why, and what the sweep measured it to be worth.
        "rung18_solver_declarations": {},
        "reference_arm_caveat":
            "This arm is the reference for rung 18's cross-arm embed_gap, so "
            "it replays lane1r's WHOLE WorldInfo, contact declaration "
            "included.  The R5 datum -- the same 50 tosses with every "
            "declaration removed, i.e. MuJoCo's own defaults -- is 43.023 % of "
            "cube width against 24.845 % declared, and is reproducible with "
            "`python mujoco/rung18.py --sweep`.  Quote neither for the other.",
        "scale_mode": spec.RUNG18_SCALE,
        "steps": 0, "compile_s": 0.0, "startup_s": 0.0, "step_s": 0.0,
        "total_s": 0.0, "us_per_step": None,
    }
    indices = (spec.RUNG18_FAULT_INDICES if fault != "none"
               else spec.RUNG18_INDICES)
    if subset == "fault":
        indices = spec.RUNG18_FAULT_INDICES
    elif subset == "full":
        indices = spec.RUNG18_INDICES
    samples = {"rung": 18, "sim": SIM, "fault": fault,
               "requested": list(indices), "runs": []}

    if fault not in FAULTS:
        meta["error"] = ("this arm does not implement rung-18 fault %r "
                         "(supported: %s)" % (fault, ", ".join(FAULTS)))
        return samples, meta

    try:
        import mujoco as mj
        import numpy                                  # noqa: F401
        D = spec.rung18_dataset()
        lane1r = _lane1r_runner()
        facts = scenes.rung18_facts()
    except Exception as exc:                          # noqa: BLE001
        meta["error"] = ("rung 18 needs mujoco, numpy and lane1r: %r" % (exc,))
        return samples, meta

    # Two independent readings of the table's height -- lane1r's runner and
    # lane1r's own world file -- and they have to agree.  It is the one number
    # the score subtracts back off every sample, so a silent disagreement would
    # shift this whole column against the recording.
    if abs(float(lane1r.TABLE_TOP) - facts["table_top_m"]) > 1e-12:
        meta["error"] = ("lane1r's runner says the table top is at %r and its "
                         "world puts it at %r; rung 18 will not choose"
                         % (lane1r.TABLE_TOP, facts["table_top_m"]))
        return samples, meta

    try:
        step_ms = _record_step_ms(lane1r)
        dt = facts["timestep_s"]
        sub = step_ms / 1000.0 / dt
        if abs(round(sub) - sub) > 1e-9:
            raise ValueError("lane1r records every %d ms on a %.6f s timestep, "
                             "which is not a whole number of steps"
                             % (step_ms, dt))
        t0 = time.perf_counter()
        xml = scenes.mjcf(18, fault=fault)
        model = mj.MjModel.from_xml_string(xml)
        meta["compile_s"] = time.perf_counter() - t0
    except Exception as exc:                          # noqa: BLE001
        meta["error"] = "rung 18 scene could not be built: %r" % (exc,)
        return samples, meta

    jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "cube_free")
    cfg = {
        "qpos_adr": int(model.jnt_qposadr[jid]),
        "qvel_adr": int(model.jnt_dofadr[jid]),
        "body": mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "cube"),
        "table_top": float(lane1r.TABLE_TOP),
        "step_s": step_ms / 1000.0,
        "substeps": int(round(sub)),
        "n_steps": int(round(float(lane1r.DURATION_MS) / step_ms)),
    }
    meta["rung18_solver_declarations"] = scenes.rung18_declarations()
    meta.update({
        "mujoco_version": mj.__version__,
        "lane1r_dataset": D.__file__,
        "lane1r_world": facts["lane1r_world"],
        "lane1r_duration_ms": float(lane1r.DURATION_MS),
        "record_step_ms": step_ms,
        "timestep_s": dt,
        "substeps_per_sample": cfg["substeps"],
        "cube": {k: facts[k] for k in ("cube_edge_m", "cube_mass_kg",
                                       "cube_inertia_kg_m2", "mu")},
        "table_top_m": cfg["table_top"],
        "solver": {
            "integrator": str(mj.mjtIntegrator(model.opt.integrator)).split(
                ".")[-1],
            "solver": str(mj.mjtSolver(model.opt.solver)).split(".")[-1],
            "cone": str(mj.mjtCone(model.opt.cone)).split(".")[-1],
            "impratio": float(model.opt.impratio),
            "iterations": int(model.opt.iterations),
            "timestep": float(model.opt.timestep),
            "gravity_z": float(model.opt.gravity[2]),
            "body_mass": [float(v) for v in model.body_mass],
            "body_inertia": [[float(c) for c in row]
                             for row in model.body_inertia],
            "ngeom": int(model.ngeom),
        },
    })

    t_steps = time.perf_counter()
    failed = []
    for i in indices:
        try:
            rec = _toss(mj, model, int(i), fault, D, cfg)
        except Exception as exc:                      # noqa: BLE001
            # A gap is a MEASUREMENT, not an exception: it lands in
            # ``tosses_scored`` as a red rather than aborting the row, because
            # an aborted row is indistinguishable from one never run.
            rec = {"tag": "toss%04d" % int(i), "params": {"index": int(i)},
                   "error": "%s: %s" % (type(exc).__name__, exc)}
            failed.append("%s(%s)" % (i, rec["error"][:60]))
        samples["runs"].append(rec)
        meta["steps"] += int(rec.get("physics_steps") or 0)
    meta["step_s"] = time.perf_counter() - t_steps
    meta["total_s"] = meta["step_s"] + meta["compile_s"]
    meta["us_per_step"] = (meta["step_s"] / meta["steps"] * 1e6
                           if meta["steps"] else None)
    if failed:
        meta["gaps"] = failed
    meta["proc_t0"] = t_cell
    meta["proc_t1"] = time.time()
    samples["wall"] = {"t_start": t_cell, "t_first_step": t_cell,
                       "t_end": meta["proc_t1"]}
    return samples, meta


# --------------------------------------------------------------------------
# the R5 datum + the attribution sweep -- NEVER a ladder row
# --------------------------------------------------------------------------

#: (label, the MJCF ``<option>`` fragment).  ``""`` is MuJoCo's own defaults,
#: which is the CONTRACT.md 3b R5 datum: the same scene with every declaration
#: removed.  The rest exist to attribute the difference to a field rather than
#: to argue about it, and R3 is why the impratio row is here at all -- a
#: setting has to be shown INERT or shown to matter, and either answer needs
#: the row.
SWEEP = (
    ("MuJoCo defaults  (R5 datum)", ""),
    ("cone=elliptic", ' cone="elliptic"'),
    ("impratio=10", ' impratio="10"'),
    ("cone=elliptic impratio=10  (lane1r's world -- the shipped row)",
     ' cone="elliptic" impratio="10"'),
)


def sweep(subset=None):
    """Score the contract's toss set under each configuration.  Prints a table.

    NOT A LADDER ROW.  The ladder is one scene per rung (CONTRACT.md section
    2), and a variant that could be quoted as one is exactly the thing
    amendment E forbids.  This exists so the shipped row's two declarations are
    a BUDGET with a published sweep behind them rather than two values that
    happened to work.
    """
    analysis = shared.analysis
    base = scenes.solver_options
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results",
                       "rung18_sweep")
    rows = []
    try:
        for label, opt in SWEEP:
            scenes.solver_options = (
                lambda rung, _o=opt: _o if int(rung) == 18 else base(rung))
            samples, mt = run(out, fault="none", subset=subset)
            m = analysis.reduce_samples(samples, exit_code=mt.get("exit_code"))
            rows.append((label, m))
            print("  %-58s pos %8.3f %%   rot %7.3f deg   tunnel %.4g   n=%d"
                  % (label, m.get("real_pos_err") or float("nan"),
                     m.get("real_rot_err") or float("nan"),
                     m.get("tunnel_depth") or 0.0, len(m.get("per_toss") or {})))
    finally:
        scenes.solver_options = base
    return rows


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(
        description="ladder0 rung 18 -- bare MuJoCo vs recorded cube tosses")
    ap.add_argument("--sweep", action="store_true",
                    help="the R5 defaults datum and the attribution sweep; "
                         "NOT a ladder row")
    ap.add_argument("--fault", choices=sorted(FAULTS), default="none")
    ap.add_argument("--subset", choices=("fault", "full"), default=None)
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    here = os.path.dirname(os.path.abspath(__file__))
    if a.sweep:
        print("rung 18 -- position error against %d recorded tosses, by "
              "contact configuration" % len(spec.RUNG18_FAULT_INDICES
                                            if a.subset == "fault"
                                            else spec.RUNG18_INDICES))
        sweep(subset=a.subset)
        return 0
    samples, mt = run(a.out or os.path.join(here, "results", "rung18"),
                      fault=a.fault, subset=a.subset)
    m = shared.analysis.reduce_samples(samples, exit_code=mt.get("exit_code"))
    for chk in spec.check_rung(18, m):
        print("  %-22s %s %-14s exp %-10s tol %-10s"
              % (chk.name, "OK " if chk.ok else "RED",
                 "%.6g" % chk.measured if isinstance(chk.measured, float)
                 else chk.measured, chk.expected, chk.tol))
    if mt.get("error"):
        print("  ! %s" % mt["error"])
    return 0


__all__ = ["run", "FAULTS", "sweep", "SWEEP"]


if __name__ == "__main__":
    sys.exit(main())
