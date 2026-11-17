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

"""The grader-owned **observer** for a MuJoCo run. Requires ``mujoco``.

This is the MuJoCo answer to "who measures the run", and it is the one design
decision in this arm that deserves an argument rather than a paragraph.

On OmniSim and on upstream Webots the grader injects a Supervisor *robot* into
the world: an extra participant the engine steps alongside the agent's own
controllers. MuJoCo has no such thing. There is no controller process, no scene
graph service and no in-world scripting layer -- **an MJCF file is inert data
and the agent's own Python program owns the stepping loop**. So the only place
a grader can stand is inside that loop.

It stands there by wrapping ``mujoco.mj_step`` (and the ``mj_step1``/``mj_step2``
split) *before* the agent's driver is executed, and then executing the driver
unchanged. The driver decides everything it decided before -- what to load, what
to actuate, when to stop -- and every completed integration step passes through
one function that samples it. Three properties this buys, each of which the
alternatives lose:

* **The agent's program is not rewritten and not constrained.** The obvious
  alternative -- requiring the deliverable to expose ``def control(model,
  data)`` so the grader can own the loop -- would impose a calling convention
  that MuJoCo itself does not impose, on the one comparator whose whole
  advantage is that a short free-form Python driver is idiomatic. That is
  exactly the "crippled competitor" failure SPEC 6.2 exists to prevent.
* **The measurement is not the agent's to author.** The other alternative --
  letting the driver log its own trajectory -- hands the thing being measured
  the pen. Every number below is read from ``MjModel``/``MjData`` by this
  module and written by this module, outside the agent's workspace.
* **Termination belongs to the grader.** MuJoCo has no ``--duration``, no
  auto-exit and no ``simulationQuit()``; a driver's loop runs until the driver
  says so. When the recorded simulated time reaches the task's window this
  module raises :class:`DurationReached` out of ``mj_step`` to unwind the
  driver. It derives from ``BaseException`` on purpose, so an ordinary
  ``except Exception:`` in the agent's code cannot swallow the grader's clock.

What this module does NOT see, stated here rather than discovered later:

* a driver that steps through a path this module has not wrapped -- MJX /
  ``mujoco.mjx``, ``mujoco.rollout``, or a C extension calling ``mj_step``
  directly. Those produce **zero samples**, which is reported as "no samples",
  never as "it did not move";
* direct ``d.qpos`` / ``d.qvel`` scripting between steps. The poses that
  results in ARE recorded (they are the state at the end of a step), but the
  *reason* the body moved is invisible here -- see ``robot_class`` below;
* anything before the first ``mj_step``. The t=0 inventory is taken at the
  first observed step, which is the earliest instant this module exists at.
"""

from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np

import mujoco

#: Rows in the pose series are capped so a pathological scene cannot exhaust
#: memory; the truncation is published, never silent.
DEFAULT_BODY_CAP = 256

#: Distinct contact PAIRS kept with their first witness. The two vacuity
#: counters are totals and are never capped.
DEFAULT_PAIR_CAP = 512

#: ``mjJNT_FREE`` is a body's 6-DoF floating base, not an articulation. Webots
#: and OmniSim do not count a floating base as a joint (there is no joint node
#: for it), so counting it here would make the same robot report one more joint
#: on this arm than on theirs and would quietly change what R1.2's ">= 2
#: joints" and A1's ">= 4 joints" mean per simulator.
_FREE = int(mujoco.mjtJoint.mjJNT_FREE)
_PLANE = int(mujoco.mjtGeom.mjGEOM_PLANE)


class DurationReached(BaseException):
    """The grader's clock ran out. Raised out of ``mj_step`` to unwind.

    ``BaseException``, not ``Exception``: a driver wrapping its loop in
    ``except Exception:`` must not be able to catch the grader's termination
    and keep stepping. This is the same reason ``KeyboardInterrupt`` is one.
    """


class WallClockExceeded(BaseException):
    """The wall-clock guard fired -- a driver stepping a tiny dt forever."""


# --- name/geometry helpers (pure, unit-testable) -----------------------------


def names_of(model, objtype, n):
    """``[name or "" ...]`` for one object type, indexed by id."""
    out = []
    for i in range(n):
        nm = mujoco.mj_id2name(model, objtype, i)
        out.append(nm or "")
    return out


def geom_world_aabb(model, data, gid):
    """One geom's world-space AABB in metres, or ``(None, None, why)``.

    MuJoCo publishes ``model.geom_aabb`` as ``(cx, cy, cz, ex, ey, ez)`` in the
    geom's OWN frame; the world box is that box rotated by ``geom_xmat`` and
    translated by ``geom_xpos``, which for an oriented box is the standard
    ``|R| @ extent`` bound.

    ⚠ **A ``plane`` is a special case and it is not a rounding detail.** A
    MuJoCo plane is mathematically INFINITE for collision, and its
    ``geom_aabb`` says so (+-5e9 m). Returning that would put a floor's AABB
    over the whole scene and make every geometric assertion meaningless; making
    one up would be worse. So a plane reports the box of its DRAWN extent
    (``geom_size[0:2]``, the half-lengths MuJoCo renders) and the caller is
    told, via the third return value, that the collision surface is unbounded.
    A plane with a zero size in x or y is drawn infinite too, and then there is
    no honest finite box at all: ``(None, None, ...)``.
    """
    gt = int(model.geom_type[gid])
    pos = np.asarray(data.geom_xpos[gid], dtype=float)
    rot = np.asarray(data.geom_xmat[gid], dtype=float).reshape(3, 3)
    if gt == _PLANE:
        sx, sy = float(model.geom_size[gid][0]), float(model.geom_size[gid][1])
        if sx <= 0.0 or sy <= 0.0:
            return None, None, ("a MuJoCo plane geom is INFINITE for collision "
                                "and this one is drawn infinite too "
                                "(geom_size x or y is 0), so it has no honest "
                                "finite world AABB")
        local_c = np.zeros(3)
        local_e = np.array([sx, sy, 0.0])
        why = ("a MuJoCo plane geom is INFINITE for collision; this AABB is "
               "its DRAWN extent (geom_size %.4g x %.4g m) and understates the "
               "surface a body can actually rest on" % (sx, sy))
    else:
        local_c = np.asarray(model.geom_aabb[gid][:3], dtype=float)
        local_e = np.asarray(model.geom_aabb[gid][3:], dtype=float)
        why = ""
    centre = pos + rot @ local_c
    ext = np.abs(rot) @ local_e
    return tuple(centre - ext), tuple(centre + ext), why


def union_aabb(boxes):
    """Componentwise union of ``[(lo, hi), ...]``, or ``(None, None)``."""
    los = [np.asarray(lo, dtype=float) for lo, hi in boxes if lo is not None]
    his = [np.asarray(hi, dtype=float) for lo, hi in boxes if hi is not None]
    if not los or not his:
        return None, None
    return (tuple(np.min(np.stack(los), axis=0)),
            tuple(np.max(np.stack(his), axis=0)))


def body_geoms(model, bid):
    """Geom ids attached to body ``bid``."""
    adr = int(model.body_geomadr[bid])
    num = int(model.body_geomnum[bid])
    return list(range(adr, adr + num)) if adr >= 0 else []


def subtree_joint_ids(model, root, *, include_free=False):
    """Joint ids in ``root``'s subtree, floating base excluded by default."""
    out = []
    for j in range(model.njnt):
        b = int(model.jnt_bodyid[j])
        if int(model.body_rootid[b]) != int(root):
            continue
        if not include_free and int(model.jnt_type[j]) == _FREE:
            continue
        out.append(j)
    return out


def actuated_roots(model):
    """``{root body id: [why, ...]}`` -- which subtrees an actuator can drive.

    **This is the "what is a robot in MJCF" rule, and it is a decision, not a
    reading.** MJCF has no ``Robot`` node: a ``<body>`` is a body, and the same
    element describes a manipulator link, a falling crate and a wall. So the
    predicate has to be built out of something MJCF *does* say, and the one
    thing that distinguishes a machine from a prop is that **something can
    drive it**: an actuator whose transmission resolves into that subtree.

    Every transmission type is followed, because refusing to follow one would
    silently declare a legitimately actuated robot to be scenery:

    ``mjTRN_JOINT`` / ``mjTRN_JOINTINPARENT``
        the joint's body.
    ``mjTRN_TENDON``
        the tendon's wrap objects -- joints, sites and wrapping geoms -- each
        resolved to its body. A tendon spanning two subtrees marks BOTH.
    ``mjTRN_SITE`` / ``mjTRN_SLIDERCRANK``
        the site's body (a thruster, a general force, a crank).
    ``mjTRN_BODY``
        the body itself (adhesion).

    The failure modes are enumerated in ``BRINGUP.md`` and are real: an
    actuated conveyor counts as a robot, and a body driven by nothing but
    direct ``qpos``/``qvel`` writes does not. The second is why the recorder
    ALSO observes applied forces at run time (:meth:`Recorder.forced_roots`) --
    a subtree that received ``qfrc_applied``/``xfrc_applied`` was being driven,
    whatever the model declares.
    """
    trn = mujoco.mjtTrn
    out = {}

    def mark(bid, why):
        if bid is None or int(bid) <= 0:
            return
        root = int(model.body_rootid[int(bid)])
        if root <= 0:
            return
        out.setdefault(root, [])
        if why not in out[root]:
            out[root].append(why)

    for a in range(model.nu):
        tt = int(model.actuator_trntype[a])
        oid = int(model.actuator_trnid[a][0])
        aname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, a) or \
            "#%d" % a
        if tt in (int(trn.mjTRN_JOINT), int(trn.mjTRN_JOINTINPARENT)):
            if 0 <= oid < model.njnt:
                jname = (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT,
                                           oid) or "#%d" % oid)
                mark(model.jnt_bodyid[oid],
                     "actuator %r drives joint %r" % (aname, jname))
        elif tt == int(trn.mjTRN_TENDON):
            if 0 <= oid < model.ntendon:
                for b in _tendon_bodies(model, oid):
                    mark(b, "actuator %r drives tendon #%d" % (aname, oid))
        elif tt in (int(trn.mjTRN_SITE), int(trn.mjTRN_SLIDERCRANK)):
            if 0 <= oid < model.nsite:
                mark(model.site_bodyid[oid],
                     "actuator %r acts at a site" % aname)
        elif tt == int(trn.mjTRN_BODY):
            mark(oid, "actuator %r is an adhesion actuator on this body"
                 % aname)
    return out


def _tendon_bodies(model, tid):
    """Bodies a tendon's wrap path touches."""
    wrap = mujoco.mjtWrap
    adr = int(model.tendon_adr[tid])
    num = int(model.tendon_num[tid])
    out = []
    for k in range(adr, adr + num):
        wt = int(model.wrap_type[k])
        oid = int(model.wrap_objid[k])
        if wt == int(wrap.mjWRAP_JOINT) and 0 <= oid < model.njnt:
            out.append(int(model.jnt_bodyid[oid]))
        elif wt == int(wrap.mjWRAP_SITE) and 0 <= oid < model.nsite:
            out.append(int(model.site_bodyid[oid]))
        elif wt in (int(wrap.mjWRAP_SPHERE), int(wrap.mjWRAP_CYLINDER)) \
                and 0 <= oid < model.ngeom:
            out.append(int(model.geom_bodyid[oid]))
    return out


def _model_name(model):
    """The ``<mujoco model="...">`` name, or ``""``.

    ``model.names`` is one NUL-separated blob and the model's own name is the
    first entry in it; there is no accessor.
    """
    try:
        raw = bytes(model.names)
        return raw.split(b"\x00", 1)[0].decode("utf-8", "replace")
    except Exception:                                   # noqa: BLE001
        return ""


def model_info(model):
    """Everything needed to attribute the run to an engine + solver."""
    opt = model.opt
    return {
        "mujoco_version": mujoco.mj_versionString(),
        "mujoco_python_version": getattr(mujoco, "__version__", None),
        "model_name": _model_name(model),
        "solver": mujoco.mjtSolver(int(opt.solver)).name,
        "integrator": mujoco.mjtIntegrator(int(opt.integrator)).name,
        "cone": mujoco.mjtCone(int(opt.cone)).name,
        "jacobian": mujoco.mjtJacobian(int(opt.jacobian)).name,
        "timestep_s": float(opt.timestep),
        "iterations": int(opt.iterations),
        "gravity": [float(v) for v in opt.gravity],
        "impratio": float(opt.impratio),
        "nbody": int(model.nbody), "ngeom": int(model.ngeom),
        "njnt": int(model.njnt), "nu": int(model.nu),
        "nq": int(model.nq), "nv": int(model.nv),
    }


# --- the recorder ------------------------------------------------------------


class Recorder:
    """Wraps ``mujoco.mj_step`` and writes the grader's artifact set.

    Install it, run the agent's driver, call :meth:`finish`. Never raises out
    of the observation path except the two termination signals, which are the
    point.
    """

    def __init__(self, out_dir, *, duration_s=None, wall_limit_s=None,
                 contact_stride=1, body_cap=DEFAULT_BODY_CAP,
                 pair_cap=DEFAULT_PAIR_CAP, driver_label="", model_path=None):
        self.out_dir = Path(out_dir)
        self.duration_s = float(duration_s) if duration_s else None
        self.wall_limit_s = float(wall_limit_s) if wall_limit_s else None
        self.contact_stride = max(1, int(contact_stride))
        self.body_cap = int(body_cap)
        self.pair_cap = int(pair_cap)
        self.driver_label = driver_label or ""
        self.model_path = str(model_path) if model_path else None

        self._orig = {}
        self.installed = False
        self.model = None
        self.info = None
        self.steps = 0
        self.t = []
        self._t_prev = None           # the last RAW data.time seen
        self._t_acc = 0.0             # ...and the monotone accumulation of it
        self.resets = []              # every backwards jump, with its step
        self._rows = []               # per tracked body: list of (x, y, z)
        self.tracked = []             # body ids, parallel to _rows
        self.bodies_truncated = False
        self.t0 = None                # the frozen inventory document
        self.t0_frozen = None
        self.t0_time = None

        # contacts
        self.total_observed = 0
        self.distinct_named = 0
        self._pairs = {}
        self._parts = []
        self.pairs_truncated = False
        self.contact_steps = 0

        # run-time "something drove this" channels
        self._forced_dofs = None
        self._forced_bodies = None
        self._ctrl_touched = False

        self.stopped_by = None
        self.driver_error = None
        self.warnings_seen = {}
        self.notes = []
        self.tamper = []
        self._t_wall0 = None

    # -- installation ------------------------------------------------------
    def install(self):
        """Wrap the stepping entry points. Idempotent."""
        if self.installed:
            return self
        self._t_wall0 = time.monotonic()
        for name in ("mj_step", "mj_step1", "mj_step2"):
            self._orig[name] = getattr(mujoco, name)
        mujoco.mj_step = self._make_hook("mj_step", sample=True)
        mujoco.mj_step1 = self._make_hook("mj_step1", sample=False)
        mujoco.mj_step2 = self._make_hook("mj_step2", sample=True)
        self.installed = True
        return self

    def uninstall(self):
        for name, fn in self._orig.items():
            setattr(mujoco, name, fn)
        self.installed = False

    def hook_intact(self):
        """Is the observation path still ours?

        The agent's driver runs in this process, so it *could* put the original
        ``mj_step`` back and step unobserved. That is this arm's analogue of
        the world-project controller shadowing the OmniSim adapter refuses to
        grade, and it is reported rather than assumed away.
        """
        if not self.installed:
            return None
        return all(getattr(mujoco, n).__name__ == "_agentbench_%s" % n
                   for n in self._orig)

    def _make_hook(self, name, *, sample):
        orig = self._orig[name]
        rec = self

        def hook(*args, **kwargs):
            model = args[0] if args else kwargs.get("m")
            data = args[1] if len(args) > 1 else kwargs.get("d")
            if rec.model is None and model is not None and data is not None:
                rec._bind(model, data)
            out = orig(*args, **kwargs)
            if sample and model is not None and data is not None:
                rec._after_step(model, data)
            return out

        hook.__name__ = "_agentbench_%s" % name
        hook.__doc__ = ("AgentBench observation hook around mujoco.%s -- "
                        "delegates unchanged and samples the result." % name)
        return hook

    # -- the frozen t=0 scan ----------------------------------------------
    def _bind(self, model, data):
        """First observed step: freeze the scene and build the inventory.

        The scan runs on a **deep copy** of ``MjData`` so nothing the grader
        does can perturb the run it is measuring -- not the derived arrays, and
        (critically) not a user control callback, which ``mj_forward`` would
        have invoked a second time. ``mj_kinematics`` is enough for every
        geometric question here and calls no user code at all.
        """
        self.model = model
        self.info = model_info(model)
        self.t0_time = float(data.time)
        self.t0_frozen = bool(self.steps == 0 and float(data.time) == 0.0)
        scratch = data
        try:
            import copy
            scratch = copy.deepcopy(data)
        except Exception as exc:                        # noqa: BLE001
            self.notes.append(
                "the t=0 scan could not deep-copy MjData (%r); it ran on the "
                "live MjData instead. Only derived arrays are recomputed by "
                "mj_kinematics, so the integrated state is unchanged, but say "
                "so rather than imply an isolated read." % (exc,))
        try:
            mujoco.mj_kinematics(model, scratch)
            # ...and the camera/light poses, which mj_kinematics does not do;
            # neither call touches user callbacks or the integrated state.
            mujoco.mj_camlight(model, scratch)
        except Exception as exc:                        # noqa: BLE001
            self.notes.append("mj_kinematics failed during the t=0 scan: %r"
                              % (exc,))
        # The per-step channels are armed FIRST and unconditionally: a failure
        # while building the inventory must not leave the observation path
        # half-initialised, because an exception raised out of mj_step is
        # indistinguishable, from the driver's side, from the agent's own code
        # failing -- a grader crash wearing an agent failure's clothes.
        allb = list(range(1, model.nbody))
        self.tracked = allb[:self.body_cap]
        self.bodies_truncated = len(allb) > len(self.tracked)
        self._rows = [[] for _ in self.tracked]
        self._forced_dofs = np.zeros(model.nv, dtype=bool)
        self._forced_bodies = np.zeros(model.nbody, dtype=bool)
        try:
            self._parts = self._build_participants(model)
        except Exception as exc:                        # noqa: BLE001
            self._parts = []
            self.notes.append(
                "the contact participant table could not be built (%r), so "
                "NO contact was scanned in this run; the contact channel is "
                "unanswered rather than empty." % (exc,))
        try:
            self.t0 = self._inventory(model, scratch)
        except Exception as exc:                        # noqa: BLE001
            self.t0 = None
            self.notes.append(
                "the t=0 scene scan FAILED (%r), so this run has no inventory "
                "and no world-space bounds. The pose series and the contact "
                "scan below are unaffected." % (exc,))

    def _inventory(self, model, data):
        """The t=0 document: bodies, links and world-attached static geoms.

        **Why world geoms get their own entries.** In MJCF, static scenery is
        idiomatically written as bare ``<geom>`` elements on ``<worldbody>`` --
        five obstacle boxes are five geoms on ONE body (``world``). Folding
        them into that body would report the scene as a single object whose
        AABB spans everything, which is not a conservative simplification: it
        is a wrong measurement. Each world-attached geom is therefore its own
        inventory entry, which is also what makes an obstacle audit possible
        here without any name convention at all.
        """
        bnames = names_of(model, mujoco.mjtObj.mjOBJ_BODY, model.nbody)
        gnames = names_of(model, mujoco.mjtObj.mjOBJ_GEOM, model.ngeom)
        act = actuated_roots(model)
        plane_notes = []

        def geom_box(gid):
            lo, hi, why = geom_world_aabb(model, data, gid)
            if why:
                plane_notes.append("geom %r: %s" % (gnames[gid] or "#%d" % gid,
                                                    why))
            return lo, hi

        bodies = []
        for i in range(1, model.nbody):
            root = int(model.body_rootid[i])
            gids = body_geoms(model, i)
            lo, hi = union_aabb([geom_box(g) for g in gids])
            is_root = int(model.body_parentid[i]) == 0
            jids = subtree_joint_ids(model, i) if is_root else []
            bodies.append({
                "key": "body:%d" % i,
                "name": bnames[i],
                "type": "body",
                "id": i,
                "root": root,
                "top_level": is_root,
                "position": [float(v) for v in data.xpos[i]],
                "aabb_min": None if lo is None else [float(v) for v in lo],
                "aabb_max": None if hi is None else [float(v) for v in hi],
                "n_joints": len(jids) if is_root else None,
                "n_joints_incl_free": (len(subtree_joint_ids(
                    model, i, include_free=True)) if is_root else None),
                "dofnum": int(model.body_dofnum[i]),
                "mass_kg": float(model.body_mass[i]),
                # body_weldid == 0 is MuJoCo's own statement that this body is
                # rigidly welded to the world: physics cannot move it.
                "movable": int(model.body_weldid[i]) != 0,
                "geoms": [gnames[g] or "#%d" % g for g in gids],
                "actuated_reasons": list(act.get(i, [])) if is_root else [],
                "parent_key": (None if is_root else "body:%d" % root),
            })

        world_geoms = []
        for g in range(model.ngeom):
            if int(model.geom_bodyid[g]) != 0:
                continue
            lo, hi = geom_box(g)
            world_geoms.append({
                "key": "geom:%d" % g,
                "name": gnames[g],
                "type": "world_geom",
                "id": g,
                "geom_type": mujoco.mjtGeom(int(model.geom_type[g])).name,
                "position": [float(v) for v in data.geom_xpos[g]],
                "aabb_min": None if lo is None else [float(v) for v in lo],
                "aabb_max": None if hi is None else [float(v) for v in hi],
                "movable": False,
                "mass_kg": 0.0,
            })
        return {"bodies": bodies, "world_geoms": world_geoms,
                "cameras": self._cameras(model, data),
                "plane_notes": sorted(set(plane_notes)),
                "actuated_roots": {str(k): v for k, v in act.items()}}

    def _cameras(self, model, data):
        """Every ``<camera>`` in the model, as world position + axes + FOV.

        MuJoCo's camera frame looks down its own **-z** with **+y** up (the
        OpenGL convention), and ``cam_fovy`` is the FULL **vertical** angle in
        DEGREES. Converting that into the neutral schema's world-frame unit
        vectors and radians is exactly the per-simulator step the adapter
        boundary exists to absorb, so it happens here and not in a grader.

        The interactive viewer's *free* camera is deliberately absent: it is
        not part of the model, it is not in ``MjData``, and a headless run does
        not have one. A MuJoCo scene's camera is a scene element or it does not
        exist.
        """
        out = []
        for c in range(model.ncam):
            rot = np.asarray(data.cam_xmat[c], dtype=float).reshape(3, 3)
            res = None
            if hasattr(model, "cam_resolution"):
                r = model.cam_resolution[c]
                if int(r[0]) > 0 and int(r[1]) > 0:
                    res = [int(r[0]), int(r[1])]
            out.append({
                "name": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA,
                                          c) or "",
                "id": c,
                "position": [float(v) for v in data.cam_xpos[c]],
                "forward": [float(-rot[0, 2]), float(-rot[1, 2]),
                            float(-rot[2, 2])],
                "up": [float(rot[0, 1]), float(rot[1, 1]), float(rot[2, 1])],
                "fovy_deg": float(model.cam_fovy[c]),
                "resolution": res,
            })
        return out

    # -- per-step ----------------------------------------------------------
    def _after_step(self, model, data):
        self.steps += 1
        # ⚠ ``data.time`` IS NOT A MONOTONE CLOCK, and this is not a hypothesis:
        # when the solve diverges MuJoCo raises mjWARN_BADQACC and calls
        # mj_resetData, which sets time back to 0. A recording window keyed on
        # data.time therefore NEVER CLOSES on a diverging run -- measured on
        # this arm's own bring-up, where a badly conditioned scene reached
        # 311,846 steps at a reported 0.015 s of "simulated time" and was
        # stopped only by the wall-clock guard. So the window is driven by an
        # ACCUMULATED clock that ignores the discontinuities, and every
        # discontinuity is recorded and published instead of being smoothed
        # away: the state after a reset is the model's INITIAL state, not a
        # continuation, and any core integrating this path must be told.
        t_now = float(data.time)
        if self._t_prev is not None and t_now < self._t_prev - 1e-12:
            self.resets.append({"step": int(self.steps),
                                "from_t_s": float(self._t_prev),
                                "to_t_s": t_now,
                                "accumulated_s": float(self._t_acc)})
            gain = max(0.0, t_now)
        else:
            gain = t_now - (self._t_prev if self._t_prev is not None else 0.0)
        self._t_acc += max(0.0, gain)
        self._t_prev = t_now
        self.t.append(self._t_acc)
        xpos = data.xpos
        for k, bid in enumerate(self.tracked):
            p = xpos[bid]
            self._rows[k].append((float(p[0]), float(p[1]), float(p[2])))

        if self.steps % self.contact_stride == 0:
            self._scan_contacts(model, data)
            self.contact_steps += 1

        # Driven-ness channels: what the model does not declare, the run can
        # still show. Cheap numpy reductions, same stride as the contact scan.
        if self.steps % self.contact_stride == 0:
            try:
                if data.qfrc_applied.size:
                    self._forced_dofs |= np.abs(data.qfrc_applied) > 0.0
                if data.xfrc_applied.size:
                    self._forced_bodies |= np.any(
                        np.abs(data.xfrc_applied) > 0.0, axis=1)
                if not self._ctrl_touched and data.ctrl.size:
                    self._ctrl_touched = bool(np.any(data.ctrl != 0.0))
            except Exception:                           # noqa: BLE001
                pass

        for w in range(int(mujoco.mjtWarning.mjNWARNING)):
            n = int(data.warning[w].number)
            if n:
                self.warnings_seen[mujoco.mjtWarning(w).name] = n

        if self.duration_s is not None and self._t_acc >= self.duration_s:
            self.stopped_by = "duration"
            raise DurationReached(
                "the grader's %g s recording window is full" % self.duration_s)
        if self.wall_limit_s is not None and self._t_wall0 is not None \
                and (time.monotonic() - self._t_wall0) > self.wall_limit_s:
            self.stopped_by = "wall_limit"
            raise WallClockExceeded(
                "the grader's %g s wall-clock guard fired after %d steps"
                % (self.wall_limit_s, self.steps))

    def _scan_contacts(self, model, data):
        ncon = int(data.ncon)
        parts = self._parts
        if ncon <= 0 or not parts:
            return
        self.total_observed += ncon
        geoms = np.asarray(data.contact.geom[:ncon], dtype=int)
        for row in range(ncon):
            a = parts[int(geoms[row][0])]
            b = parts[int(geoms[row][1])]
            if a[0] != b[0] and a[1] and b[1]:
                self.distinct_named += 1
            key = (a[0], b[0]) if a[0] <= b[0] else (b[0], a[0])
            if key in self._pairs:
                continue
            if len(self._pairs) >= self.pair_cap:
                self.pairs_truncated = True
                continue
            pos = data.contact.pos[row]
            self._pairs[key] = {
                "a": a[1], "b": b[1],
                "a_key": a[0], "b_key": b[0],
                "a_root": a[2], "b_root": b[2],
                # The precise geoms/links behind the subtree-level names, kept
                # so the detail is not lost by the resolution rule above.
                "a_detail": a[3], "b_detail": b[3],
                "self_pair": a[0] == b[0],
                "point": [float(pos[0]), float(pos[1]), float(pos[2])],
                "step": int(self.steps), "t_s": float(data.time),
            }

    def _build_participants(self, model):
        """Per-geom contact identity, precomputed once.

        **Who a contact names, and why it is the SUBTREE rather than the link.**
        A geom belongs to a body, and that body may be a link deep inside an
        articulated machine. Reporting the link would mean a rover's wheel
        striking a wall is recorded as ``wheel_left / wall_north``, and a
        grader asking "did the ROBOT hit anything" -- which is what every
        collision assertion in the suite asks -- would not see it. The other
        two arms already resolve a contact to the owning robot subtree, so
        naming the link here would make the same assertion easier to pass on
        this arm than on theirs. Direction matters: that bias would flatter
        MuJoCo, and a comparison must not lean either way.

        So the participant is **the top-level body of the geom's kinematic
        subtree** (``body_rootid``), with one exception: a geom attached
        directly to ``<worldbody>`` is named by the GEOM, because that is the
        only name such a wall or obstacle has -- MJCF's idiom for static
        scenery is a bare geom, and collapsing all of them into ``world``
        would make "which wall" unanswerable.

        A consequence, stated rather than hidden: two links of the SAME robot
        touching each other produce a self-pair, which is exactly the shape of
        the ``(id, id)`` bug that hid A1.3 for weeks. It is kept -- the pair is
        real -- but it does not count towards ``distinct_named``, so it can
        never masquerade as evidence that the contact pipeline can name two
        distinct bodies.

        Precomputed because the contact scan is the hot loop of a long run: a
        60 s window at MuJoCo's 2 ms default is 30,000 steps, and resolving two
        names per contact point through ``mj_id2name`` inside that loop costs
        more than the physics.
        """
        parts = []
        for gid in range(model.ngeom):
            bid = int(model.geom_bodyid[gid])
            gname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM,
                                      gid) or ""
            if bid == 0:
                parts.append(("geom:%d" % gid, gname, 0, gname))
                continue
            root = int(model.body_rootid[bid])
            rname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY,
                                      root) or ""
            bname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY,
                                      bid) or ""
            detail = gname or bname
            if bid != root and bname:
                detail = "%s (%s)" % (detail, bname)
            parts.append(("body:%d" % root, rname, root, detail))
        return parts

    # -- results -----------------------------------------------------------
    def forced_roots(self):
        """Subtrees observed receiving an applied force/torque during the run.

        The second half of the robot predicate. A body that no actuator drives
        but that the driver pushed with ``qfrc_applied`` / ``xfrc_applied`` was
        being driven, and calling it scenery because the MJCF declared no
        ``<actuator>`` would be reading the file instead of the run.
        """
        if self.model is None or self._forced_dofs is None:
            return {}
        m = self.model
        out = {}
        for j in range(m.njnt):
            adr = int(m.jnt_dofadr[j])
            n = {int(mujoco.mjtJoint.mjJNT_FREE): 6,
                 int(mujoco.mjtJoint.mjJNT_BALL): 3}.get(int(m.jnt_type[j]), 1)
            if adr >= 0 and bool(np.any(self._forced_dofs[adr:adr + n])):
                root = int(m.body_rootid[int(m.jnt_bodyid[j])])
                out.setdefault(root, []).append(
                    "qfrc_applied was written on a dof of this subtree")
        for b in range(1, m.nbody):
            if bool(self._forced_bodies[b]):
                root = int(m.body_rootid[b])
                out.setdefault(root, []).append(
                    "xfrc_applied was written on a body of this subtree")
        return {k: sorted(set(v)) for k, v in out.items()}

    def _kind_and_parent(self, bid, robot_roots):
        root = int(self.model.body_rootid[bid])
        if int(self.model.body_parentid[bid]) != 0:
            return "link", "body:%d" % root
        return ("robot" if root in robot_roots else "solid"), None

    def finish(self, *, complete=None, driver_error=None, wall_s=None):
        """Write the artifact set. Never raises."""
        self.out_dir.mkdir(parents=True, exist_ok=True)
        if driver_error is not None:
            self.driver_error = driver_error
        intact = self.hook_intact()
        if intact is False:
            self.tamper.append(
                "the driver replaced mujoco.mj_step during the run, so an "
                "unknown number of integration steps were not observed. This "
                "run is not a measurement.")
        self.uninstall()

        act = dict((int(k), list(v)) for k, v in
                   ((self.t0 or {}).get("actuated_roots") or {}).items())
        forced = self.forced_roots()
        robot_roots = set(act) | set(forced)

        dt = None
        if len(self.t) >= 2:
            dt = float(np.median(np.diff(np.asarray(self.t, dtype=float))))
        recorded_s = (float(self.t[-1] - self.t[0]) if len(self.t) >= 1
                      else None)
        if complete is None:
            complete = bool(self.steps > 0 and self.driver_error is None)

        # -- trajectory (CSV + a JSON header; a JSON array of 30k x N x 3
        #    floats is tens of megabytes and helps nobody) ------------------
        csv_path = self.out_dir / "trajectory.csv"
        tracks = []
        try:
            with open(csv_path, "w", encoding="utf-8", newline="") as fh:
                head = ["t"]
                for bid in self.tracked:
                    nm = (mujoco.mj_id2name(self.model,
                                            mujoco.mjtObj.mjOBJ_BODY, bid)
                          or "") if self.model is not None else ""
                    head += ["%s.x" % (nm or bid), "%s.y" % (nm or bid),
                             "%s.z" % (nm or bid)]
                fh.write(",".join(head) + "\n")
                for s in range(len(self.t)):
                    vals = ["%.9g" % self.t[s]]
                    for row in self._rows:
                        x, y, z = row[s]
                        vals += ["%.9g" % x, "%.9g" % y, "%.9g" % z]
                    fh.write(",".join(vals) + "\n")
        except OSError as exc:
            self.notes.append("could not write trajectory.csv: %r" % (exc,))
        if self.model is not None:
            for bid in self.tracked:
                kind, parent = self._kind_and_parent(bid, robot_roots)
                tracks.append({
                    "key": "body:%d" % bid,
                    "name": mujoco.mj_id2name(self.model,
                                              mujoco.mjtObj.mjOBJ_BODY, bid)
                    or "",
                    "kind": kind, "parent": parent})

        _write(self.out_dir / "trajectory.json", {
            "csv": csv_path.name,
            "n_samples": len(self.t),
            "dt_s": dt,
            "model_timestep_s": (self.info or {}).get("timestep_s"),
            "recorded_s": recorded_s,
            "complete": bool(complete),
            "bodies": tracks,
            "bodies_truncated": self.bodies_truncated,
            "body_cap": self.body_cap,
            # ``t`` is ACCUMULATED simulated time, not raw data.time: see
            # _after_step. Non-zero here means the series has a physical
            # discontinuity at those steps.
            "n_state_resets": len(self.resets),
            "t_is_accumulated": True,
        })

        # -- roster / t=0 ---------------------------------------------------
        t0 = dict(self.t0 or {})
        if self.t0 is None:
            # An empty ``bodies`` list with no error would read as "the scene
            # has nothing in it", which is a measurement. This is not one.
            t0["error"] = ("the t=0 scene scan failed; see the recorder notes "
                           "in completion.json. No inventory was produced -- "
                           "this is an absent measurement, not an empty scene")
        t0.update({
            "t_s": self.t0_time, "frozen": self.t0_frozen,
            "robot_roots": sorted(int(r) for r in robot_roots),
            "actuated_roots": {str(k): v for k, v in act.items()},
            "forced_roots": {str(k): v for k, v in forced.items()},
            "ctrl_touched": self._ctrl_touched,
            "source_model": self.model_path,
        })
        _write(self.out_dir / "roster.json", t0)

        # -- contacts -------------------------------------------------------
        _write(self.out_dir / "contacts.json", {
            "supported": self.model is not None,
            "steps": self.contact_steps,
            "stride": self.contact_stride,
            "window_s": recorded_s,
            "total_observed": self.total_observed,
            "distinct_named": self.distinct_named,
            "pairs": list(self._pairs.values()),
            "pairs_truncated": self.pairs_truncated,
            "pair_cap": self.pair_cap,
        })

        # -- completion + model info ---------------------------------------
        _write(self.out_dir / "completion.json", {
            "complete": bool(complete),
            "quit_called": self.stopped_by in ("duration", "wall_limit"),
            "stopped_by": self.stopped_by or (
                "driver_raised" if self.driver_error else "driver_returned"),
            "recorded_s": recorded_s, "steps": self.steps, "dt_s": dt,
            "target_duration_s": self.duration_s,
            "state_resets": list(self.resets),
            "n_state_resets": len(self.resets),
            "driver": self.driver_label,
            "driver_error": self.driver_error,
            "wall_s": wall_s,
            "hook_intact": intact,
            "tamper": list(self.tamper),
            "warnings": dict(self.warnings_seen),
            "notes": list(self.notes),
        })
        _write(self.out_dir / "model_info.json", dict(
            self.info or {}, model_path=self.model_path,
            observed=self.model is not None))
        return self.out_dir


def _write(path, doc):
    try:
        Path(path).write_text(json.dumps(doc, indent=1, sort_keys=True,
                                         default=_jsonable),
                              encoding="utf-8")
    except (OSError, TypeError, ValueError):
        pass


def _jsonable(v):
    if isinstance(v, (np.integer,)):
        return int(v)
    if isinstance(v, (np.floating,)):
        return float(v)
    if isinstance(v, np.ndarray):
        return v.tolist()
    return str(v)


__all__ = ["DEFAULT_BODY_CAP", "DEFAULT_PAIR_CAP", "DurationReached",
           "Recorder", "WallClockExceeded", "actuated_roots", "body_geoms",
           "geom_world_aabb", "model_info", "names_of", "subtree_joint_ids",
           "union_aabb"]
