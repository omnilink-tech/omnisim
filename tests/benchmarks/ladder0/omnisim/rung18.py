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

"""rung18.py -- the OmniSim arm's replay of RECORDED REAL cube tosses.

Rung 18 is the one rung whose expected values are not derived from mechanics.
They are a measurement of physical reality: 550 tosses of an acrylic cube onto
a wooden table, AprilTag/TagSLAM tracked at 148 Hz, vendored with its licence
under ``tests/benchmarks/omnibench/lane1r``.  CONTRACT.md amendment F admits
exactly that and nothing else -- a measurement of a SIMULATOR remains
forbidden, including ours and including a previous build.

WHAT THIS FILE IS AND IS NOT
----------------------------
It is a **bridge**, not a second implementation.  lane1r owns the dataset, the
cube's mass / geometry / inertia, the sampling rate, the quaternion convention,
the body-to-world angular-velocity rotation, the world file and the replay
controller, and it re-derives its calibration on every run rather than trusting
the dataset's own metadata.  All of that is imported by path and used
unmodified.  **If this ladder and lane1r ever disagree about the cube's
inertia, lane1r is right**, and the only way to keep that true is to have no
second copy of it here.

What the ladder adds is the part lane1r does not have: a contract-owned toss
subset, an acceptance band with a derivation, a whole-run tunnelling invariant,
an initial-condition assertion, and a fault battery.

THREE FAULTS, AND WHERE EACH IS INJECTED
----------------------------------------
* ``ic_drop_velocity`` -- the engine is handed ZERO velocities while the run
  record keeps the RECORDED ones as what was wanted.  That is the exact shape
  of the ``setVelocity`` silently dropped at t = 0 that this tree has shipped:
  the engine's own readback then disagrees with the initial condition the toss
  was supposed to have, and ``replay_ic_fidelity`` is the check that sees it.
  ``real_pos_err`` reds too, which is the point -- without the IC assertion,
  a dropped velocity presents as poor real-world agreement and gets charged to
  the contact model.
* ``wrong_omega_frame`` -- the dataset's BODY-frame angular velocity is handed
  through as if it were world frame.  lane1r documents this as a silent error
  that "leaves every toss spinning about the wrong axis while still looking
  like a plausible tumble".  ``replay_ic_fidelity`` must stay GREEN here and
  that is deliberate: the engine accepted exactly what it was given, so
  blaming it would be measuring the wrong thing.  Only ``real_rot_err`` sees
  it.
* ``table_hologram`` -- the table loses its ``boundingObject``.  The cube falls
  through and lands on the engine's implicit z = 0 plane half a metre below,
  and only the whole-run ``tunnel_depth`` sees it: a replay that fell through
  the table can still score a plausible MEAN position error.

THE HONEST RUN USES LANE1R'S WORLD VERBATIM.  The two scene-level faults
cannot, so they are emitted here as a text derivation of that same file, with
the single edit visible in the diff, and they run through a six-line shim
controller that loads lane1r's replay controller by path rather than copying
it.  The honest row therefore has zero drift from lane1r; the fault rows have
one documented edit each.
"""

from __future__ import annotations

import importlib.util
import json
import os
import sys
import time

HERE = os.path.abspath(os.path.dirname(__file__))
LADDER0 = os.path.dirname(HERE)
REPO = os.path.abspath(os.path.join(LADDER0, os.pardir, os.pardir, os.pardir))
WORLDS = os.path.join(HERE, "worlds")
if LADDER0 not in sys.path:
    sys.path.insert(0, LADDER0)

import rungs                                     # noqa: E402

LANE1R = os.path.join(REPO, *rungs.RUNG18_LANE1R)
SHIM_CONTROLLER = "ladder0_lane1r_replay"

#: Faults this file injects.  Anything else is REPORTED as unsupported rather
#: than quietly run as a control -- "the fault did not happen" must never read
#: as "the fault produced no failure".
FAULTS = ("none", "ic_drop_velocity", "wrong_omega_frame", "table_hologram")


def _lane1r():
    """lane1r's runner module, by path, with its own imports satisfied.

    ``omnibench/`` goes on ``sys.path`` because lane1r imports ``common``; both
    entries are appended rather than prepended so nothing in this ladder can be
    shadowed by a same-named module over there.
    """
    key = "ladder0_lane1r_run"
    if key in sys.modules:
        return sys.modules[key]
    for p in (os.path.dirname(LANE1R), LANE1R):
        if p not in sys.path:
            sys.path.append(p)
    path = os.path.join(LANE1R, "run.py")
    if not os.path.isfile(path):
        raise ImportError("lane1r runner not found at %s" % path)
    sp = importlib.util.spec_from_file_location(key, path)
    mod = importlib.util.module_from_spec(sp)
    sys.modules[key] = mod
    sp.loader.exec_module(mod)
    return mod


# --------------------------------------------------------------------------
# fault scenes -- DERIVED from lane1r's world, never re-authored
# --------------------------------------------------------------------------

_FAULT_HEADER = """# GENERATED by tests/benchmarks/ladder0/omnisim/rung18.py -- do not hand-edit.
# Ladder0 rung 18 FAULT SCENE: %s
#
# Derived by text edit from %s, which is the honest scene and belongs to
# lane 1R.  The ONLY differences are the one named above and the controller
# name -- a world outside lane1r's project directory cannot reach lane1r's
# controllers, so it runs through the %s shim, which loads lane1r's own
# replay controller by path rather than copying it.
"""


def fault_world(fault):
    """Emit (and return the path of) a derived fault scene."""
    src = os.path.join(LANE1R, "worlds", "cube_toss.wbt")
    with open(src, "r", encoding="utf-8") as f:
        text = f.read()
    if fault == "table_hologram":
        needle = "  boundingObject USE TABLE_SHAPE\n"
        if needle not in text:
            raise RuntimeError("lane1r's table has no 'boundingObject USE "
                               "TABLE_SHAPE' line for the fault to remove -- "
                               "the world changed and this fault would "
                               "silently not happen")
        text = text.replace(needle, "", 1)
        note = ("the table's boundingObject is REMOVED, so it renders, appears "
                "in the scene tree and stops nothing")
    else:
        raise ValueError("no scene-level fault %r" % (fault,))
    text = text.replace('controller "lane1r_replay"',
                        'controller "%s"' % SHIM_CONTROLLER, 1)
    # ⚠ THE PROVENANCE COMMENT GOES AFTER THE FIRST LINE, NOT BEFORE IT.  The
    # engine validates the `#VRML_SIM R2025a utf8` magic as literally the first
    # line of the file; a comment above it makes the whole world fail to load
    # with "Invalid header".  MEASURED: the controller then never starts, so it
    # never calls simulationQuit, the engine free-runs to the runner's timeout,
    # and one toss costs 240 s instead of 9 -- which reads as "the fault hangs
    # the simulator" rather than "the file has a comment in the wrong place".
    first, nl, rest = text.partition("\n")
    header = _FAULT_HEADER % (note, os.path.relpath(src, REPO),
                              SHIM_CONTROLLER)
    out = os.path.join(WORLDS, "_fault_%s_rung18.wbt" % fault)
    os.makedirs(WORLDS, exist_ok=True)
    with open(out, "w", encoding="utf-8", newline="\n") as f:
        f.write(first + nl + "\n" + header + "\n" + rest)
    return out


# --------------------------------------------------------------------------
# one toss
# --------------------------------------------------------------------------

def _reference(traj, index, table_top, edge):
    """The RECORDING, carried in the sample document beside the replay.

    It travels with the run for two reasons.  The reducer is
    simulator-neutral and must not import a dataset; and a sample document that
    carries its own ground truth is auditable after the fact -- the row can be
    re-scored without the dataset being present, and a mismatch between what
    was replayed and what it was scored against becomes visible rather than
    assumed.
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
                  "vendored at %s" % "/".join(rungs.RUNG18_LANE1R),
    }


def _toss(arm, index, out_dir, fault, binary, timeout_s, lane1r):
    """Replay one recorded toss.  Returns a run record, or one with an error."""
    import numpy as np

    tag = "toss%04d" % index
    d = os.path.join(out_dir, tag)
    os.makedirs(d, exist_ok=True)
    npz = os.path.join(d, "replay.npz")
    log_path = os.path.join(d, "engine.log")
    console = os.path.join(d, "console.log")
    for stale in (npz, log_path + ".newton.json"):
        if os.path.exists(stale):
            os.remove(stale)

    traj = lane1r.D.load(index, scale=rungs.RUNG18_SCALE)
    envs, ic_dbg = lane1r.build_ic(traj)
    world = (fault_world(fault) if fault == "table_hologram"
             else str(lane1r.WORLD))

    want_vel = list(envs["LANE1R_VEL"].split())
    want_omg = list(envs["LANE1R_OMEGA_WORLD"].split())
    if fault == "ic_drop_velocity":
        # The engine is told NOTHING about the toss's velocity while the record
        # keeps the recorded one as what was wanted.  This is the shape of
        # setVelocity being silently dropped at t = 0, and it is injected into
        # the RUN -- the engine genuinely never receives it -- never into the
        # measurement.
        envs = dict(envs, LANE1R_VEL="0 0 0", LANE1R_OMEGA_WORLD="0 0 0")
    elif fault == "wrong_omega_frame":
        # Hand the BODY-frame column through as if it were world frame: the
        # documented silent error.  The engine accepts exactly what it is
        # given, so replay_ic_fidelity must stay green and only real_rot_err
        # may see it.
        envs = dict(envs, LANE1R_OMEGA_WORLD="%.12g %.12g %.12g"
                    % tuple(float(x) for x in traj["w0"]))
        want_omg = envs["LANE1R_OMEGA_WORLD"].split()

    binpath = arm.resolve_binary(binary)
    if binpath is None:
        return {"tag": tag, "params": {"index": index},
                "error": "omnisim-bin not found"}
    env = arm._env(log_path, os.path.join(d, "unused_samples.json"))
    env.update(envs)
    env["LANE1R_OUT"] = npz
    env["LANE1R_LABEL"] = tag
    env["LANE1R_DURATION_MS"] = "%.6g" % lane1r.DURATION_MS
    env["LANE1R_STEP_MS"] = "1"
    # A stale export in the operator's shell would win over the world's own
    # declared friction and silently retune the whole rung.
    for k in ("OMNISIM_NEWTON_GROUND_MU", "OMNISIM_NEWTON_CONTACT_KE",
              "OMNISIM_NEWTON_CONTACT_KD"):
        env.pop(k, None)

    argv = [binpath, world] + list(arm.BASE_ARGS)
    rc, t0, t1, timed_out, pid = arm.spawn(argv, env, console, timeout_s)
    rec = {"tag": tag, "params": {"index": int(index)}, "pid": pid,
           "proc_start": t0, "world": world, "exit_code": rc,
           "timed_out": bool(timed_out),
           "newton": arm._sidecar(log_path)}
    if not os.path.exists(npz):
        rec["error"] = ("no npz: the replay controller did not finish "
                        "(rc=%s, timed_out=%s)" % (rc, timed_out))
        return rec

    z = np.load(npz, allow_pickle=True)
    n = int(z["n_samples"])
    step_ms = float(z["step_ms"])
    ic = json.loads(str(z["ic"]))
    ic["want_vel"] = [float(v) for v in want_vel]
    ic["want_omega_world"] = [float(v) for v in want_omg]
    # The readback happens after ONE step, by which point gravity has
    # legitimately changed vz.  Subtracting exactly that leaves the write.
    ic["grav_step"] = rungs.G * (step_ms / 1000.0)
    rec.update({
        "t": [float(v) for v in z["t"][:n]],
        "pos": [[float(c) for c in row] for row in z["pos"][:n]],
        "quat": [[float(c) for c in row] for row in z["quat"][:n]],
        "ic": ic,
        "reference": _reference(traj, index, lane1r.TABLE_TOP,
                                lane1r.D.CUBE_EDGE_M),
        "steps": n,
        "wall": {"t_start": t0, "t_first_step": t0, "t_end": t1},
    })
    rec["ic_debug"] = {k: (v if not hasattr(v, "tolist") else v.tolist())
                       for k, v in ic_dbg.items()}
    return rec


# --------------------------------------------------------------------------

def run(out_dir, fault="none", binary=None, timeout_s=240.0, subset=None):
    """One rung-18 cell: every toss of the contract's subset.  Never raises."""
    t_cell = time.time()
    meta = {"sim": "omnisim", "rung": 18, "fault": fault, "error": None,
            "exit_code": 0, "timed_out": False, "multi_run": True,
            "engine": "OmniSim Newton -> SolverMuJoCo, CPU mj_step",
            # CONTRACT.md 3b R4: every departure from an engine default, with
            # the default it departs from and one line of why.  Rung 18's
            # scene is lane1r's, so these are lane1r's declarations and they
            # are reported rather than chosen here.
            "rung18_solver_declarations": {
                "newtonCone": {"value": "elliptic", "engine_default": "\"\" "
                               "(pyramidal)", "why": "a pyramidal cone is a "
                               "polygon INSCRIBED in the Coulomb cone the "
                               "contract declares; the ellipse asks for the "
                               "model, not a better one"},
                "newtonImpratio": {"value": 10, "engine_default": "0 -> "
                                   "MuJoCo's stock 1", "why": "frictional "
                                   "constraint impedance relative to the "
                                   "normal one; approximation accuracy, not a "
                                   "change of model"},
            },
            "ground_truth": "external measurement of reality (CONTRACT.md "
                            "amendment F): 550 recorded cube tosses, "
                            "DAIRLab/dair_pll, BSD-3-Clause",
            }
    indices = (rungs.RUNG18_FAULT_INDICES if fault != "none"
               else rungs.RUNG18_INDICES)
    if subset == "fault":
        indices = rungs.RUNG18_FAULT_INDICES
    elif subset == "full":
        indices = rungs.RUNG18_INDICES
    samples = {"rung": 18, "sim": "omnisim", "fault": fault,
               "requested": list(indices), "runs": []}

    if fault not in FAULTS:
        meta["error"] = ("this arm does not implement rung-18 fault %r "
                         "(supported: %s)" % (fault, ", ".join(FAULTS)))
        return samples, meta

    arm = sys.modules.get("ladder0_arm_omnisim")
    if arm is None:                                # direct execution
        sp = importlib.util.spec_from_file_location(
            "ladder0_arm_omnisim", os.path.join(HERE, "arm.py"))
        arm = importlib.util.module_from_spec(sp)
        sys.modules["ladder0_arm_omnisim"] = arm
        sp.loader.exec_module(arm)
    try:
        lane1r = _lane1r()
        import numpy                              # noqa: F401
    except Exception as exc:                       # noqa: BLE001
        meta["error"] = "rung 18 needs lane1r and numpy: %r" % (exc,)
        return samples, meta

    binpath = arm.resolve_binary(binary)
    meta["binary"] = binpath
    meta["binary_sha256"] = arm._sha256(binpath) if binpath else None
    meta["binary_mtime"] = arm._mtime(binpath) if binpath else None
    meta["lane1r_dataset"] = lane1r.D.__file__
    meta["lane1r_world"] = str(lane1r.WORLD)
    meta["calibration"] = {"scale_mode": rungs.RUNG18_SCALE,
                           "note": "as published -- the baselines this rung's "
                                   "bound comes from were NOT scale corrected"}

    failed = []
    for i in indices:
        try:
            rec = _toss(arm, i, out_dir, fault, binary, timeout_s, lane1r)
        except Exception as exc:                   # noqa: BLE001
            rec = {"tag": "toss%04d" % i, "params": {"index": int(i)},
                   "error": "%s: %s" % (type(exc).__name__, exc)}
        samples["runs"].append(rec)
        if rec.get("error"):
            failed.append("%s(%s)" % (i, rec["error"][:60]))
        if rec.get("exit_code"):
            meta["exit_code"] = rec["exit_code"]
        meta["timed_out"] = meta["timed_out"] or bool(rec.get("timed_out"))
    if failed:
        # A gap is a MEASUREMENT, not an exception: it lands in
        # ``tosses_unscored`` as a red rather than aborting the row, because an
        # aborted row is indistinguishable from a row that was never run.
        meta["gaps"] = failed
    sid = next((r.get("newton") for r in samples["runs"] if r.get("newton")),
               None)
    meta["newton"] = sid
    meta["proc_t0"] = t_cell
    meta["proc_t1"] = time.time()
    samples["wall"] = {"t_start": t_cell, "t_first_step": t_cell,
                       "t_end": meta["proc_t1"]}
    return samples, meta


__all__ = ["run", "FAULTS", "fault_world"]
