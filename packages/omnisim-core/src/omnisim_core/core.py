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

"""The stepping core and the controller API it serves.

`Core` is `mj_step` on the engine's own compiled model plus the rules the
engine applies around it, each one found by comparing recorded engine runs
with this code until the states matched bit for bit:

  * the joint velocity / position clamp runs once per TICK, after the last
    substep, and on a tick where it changes anything the whole state passes
    through float32 (the engine edits state through float32 arrays);
  * on the very first tick the state passes through float32 between substeps;
  * a supervisor pose write lands one tick late, zeroes the velocity of the
    whole robot it moves, and passes the state through float32;
  * motor targets are carried as float32;
  * every tick ends with a position/contact refresh, so readers see the
    finished step.

`Supervisor` and friends are the part of the `omnisim` controller API the
core serves. Anything else raises `CoreUnsupported`: an unmodelled call is
refused, never answered with a guess.
"""

from __future__ import annotations

import json
import math
import os
import time
import types

from .errors import CoreUnsupported
from .runtime import ensure_runtime

_ok, _why = ensure_runtime()
if not _ok:
    raise ImportError(_why)

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

P_SPEC = int(mujoco.mjtState.mjSTATE_FULLPHYSICS) | int(mujoco.mjtState.mjSTATE_WARMSTART)

# Files of a compiled world (written by the engine hook, scripts/core/compile_hook.py, and by names.match)
MODEL, STATE0, CLAMP, ENGINE, META = "model.mjb", "state0.npy", "clamp.npz", "engine.json", "meta.json"


class Core:
    """mj_step + the engine's post-step joint clamp + the end-of-tick Cartesian refresh."""

    substeps = 1                   # MuJoCo steps per engine tick (WorldInfo.newtonSubsteps)
    recorder = None                # timeline.Recorder: told each tick's commands and pose writes, then its state

    def __init__(self, model, state0):
        self.m = model if isinstance(model, mujoco.MjModel) else mujoco.MjModel.from_binary_path(str(model))
        self.d = mujoco.MjData(self.m)
        mujoco.mj_setState(self.m, self.d, np.asarray(state0, dtype=np.float64), P_SPEC)
        self.ctrl = np.zeros(self.m.nu)
        self.pending_pose = []         # supervisor pose writes waiting for the end of the next step
        self.ticks = 0
        self.clamp_ticks = 0
        self.dt = float(self.m.opt.timestep)
        # FORCE MODE (an EFFORT-built joint: OMNISIM_NEWTON_TORQUE_MODE=1 compiles joints with no actuator). The
        # torque or force each such joint was last commanded, per dof; it persists until the next command, as the
        # engine re-sends a motor's raw input every tick. Untouched (and qfrc_applied never written) until a
        # controller calls setTorque/setForce, so a world that never does steps exactly as before.
        self.force = np.zeros(self.m.nv)
        self.force_on = False
        self.gearboxes = {}            # dof -> gearbox.prepare(...) state: the runtime's gearbox joint
        self._bound = {}               # dof -> the tick its gearbox was bound
        self.attach_log = []           # [tick, dof, spec] per gearbox, in attach order (timeline.py replays them)
        self.drawn_log = []            # [tick, node, field, setter, value] per write to a drawn node (core film replays them)
        self._seen_dofs = set()        # the runtime reads the gearbox channel on a slot's FIRST force command only
        self._dirty_next = False       # the engine re-syncs MuJoCo from its float32 state on the tick after an edit
        self.set_clamp(np.zeros(0, dtype=np.intp), np.zeros(0, dtype=np.intp), np.zeros(0), np.zeros(0), np.zeros(0), np.zeros(0))
        mujoco.mj_step1(self.m, self.d)

    @classmethod
    def from_compiled(cls, cdir):
        """A core on a compiled world's model, starting state and clamp tables."""
        cdir = str(cdir)
        with open(os.path.join(cdir, ENGINE), encoding="utf-8") as f:
            eng = json.load(f)
        if not eng.get("ok"):
            raise CoreUnsupported(eng.get("why") or "the engine did not compile this world for the core")
        core = cls(os.path.join(cdir, MODEL), np.load(os.path.join(cdir, STATE0)))
        core.substeps = int(eng.get("substeps", 1))
        clamp_path = os.path.join(cdir, CLAMP)
        if eng.get("has_clamp") and os.path.exists(clamp_path):
            z = np.load(clamp_path)
            m = core.m
            slack_rot, slack_lin = eng.get("slack", [0.05, 0.005])
            q = z["qi"].astype(np.intp)
            slk = np.array([slack_lin if m.jnt_type[np.nonzero(m.jnt_qposadr == qi)[0][0]] == mujoco.mjtJoint.mjJNT_SLIDE else slack_rot
                            for qi in q])
            core.set_clamp(q, z["qdi"].astype(np.intp), np.where(z["vlim"] > 0, z["vlim"], np.inf), z["lo"], z["hi"], slk)
        return core

    def set_clamp(self, q, qd, vlim, lo, hi, slack):
        """Clamp tables: qpos index, dof index, velocity limit (inf = none), range and slack per scalar joint."""
        self._q, self._qd, self._vlim, self._lo, self._hi, self._slk = q, qd, vlim, lo, hi, slack

    def step(self):
        m, d = self.m, self.d
        writes, self.pending_pose = self.pending_pose, []
        if self.recorder is not None:
            self.recorder.begin(self, writes)
        d.ctrl[:] = self.ctrl
        dirty, self._dirty_next = self.ticks == 0 or self._dirty_next, False
        forced = self.force_on or bool(self.gearboxes)
        for dof, g in self.gearboxes.items():
            # the runtime binds on the token and takes no motor torque that tick; from the next tick on the rotor is
            # driven by whatever the motor last sent (the engine re-sends a motor's raw input every tick)
            g["T_motor"] = 0.0 if self._bound.get(dof) == self.ticks else float(self.force[dof])
        for i in range(self.substeps):        # the engine's joint clamp runs once per tick, after the last substep
            if forced:
                self._apply_forces()
            mujoco.mj_step(m, d)
            if dirty and i < self.substeps - 1:
                # On a tick after the engine edited its state (the first tick, a clamp, a pose write) it copies
                # its float32 state into MuJoCo before every substep; otherwise a tick's substeps stay in MuJoCo.
                d.qpos[:] = d.qpos.astype(np.float32)
                d.qvel[:] = d.qvel.astype(np.float32)
        if len(self._q):
            q, v = d.qpos[self._q], d.qvel[self._qd]
            if np.any(np.abs(v) > self._vlim) or np.any((q < self._lo) | (q > self._hi)):
                self._clamp()
        if writes:
            self._dirty_next = True
            d.qpos[:] = d.qpos.astype(np.float32)
            d.qvel[:] = d.qvel.astype(np.float32)
            for qa, da, pos, quat in writes:
                if pos is not None:
                    d.qpos[qa:qa + 3] = np.asarray(pos, dtype=np.float32)
                if quat is not None:
                    d.qpos[qa + 3:qa + 7] = np.asarray(quat, dtype=np.float32)
                d.qvel[da] = 0.0
        mujoco.mj_step1(m, d)          # fresh poses and contacts for whoever reads next
        self.ticks += 1
        if self.recorder is not None:
            self.recorder.end(self)

    def _clamp(self):
        # The engine edits the state through float32 arrays on these ticks, so the whole
        # qpos/qvel passes through float32. Reproduced so results can be compared exactly.
        d = self.d
        q32 = d.qpos.astype(np.float32)
        v32 = d.qvel.astype(np.float32)
        changed = False
        for i in range(len(self._q)):
            qi, vi = self._q[i], self._qd[i]
            vl = self._vlim[i]
            if vl > 0.0:
                if float(v32[vi]) > vl:
                    v32[vi] = vl
                    changed = True
                elif float(v32[vi]) < -vl:
                    v32[vi] = -vl
                    changed = True
            lo, hi, slk = self._lo[i], self._hi[i], self._slk[i]
            if lo != hi:
                x = float(q32[qi])
                if x < lo - slk:
                    q32[qi] = lo - slk
                    changed = True
                    if v32[vi] < 0.0:
                        v32[vi] = 0.0
                elif x > hi + slk:
                    q32[qi] = hi + slk
                    changed = True
                    if v32[vi] > 0.0:
                        v32[vi] = 0.0
        if changed:
            d.qpos[:] = q32.astype(np.float64)
            d.qvel[:] = v32.astype(np.float64)
            self.clamp_ticks += 1
            self._dirty_next = True

    # ---- force mode and the gearbox joint (opt-in: nothing below runs until a controller commands a torque)
    def _apply_forces(self):
        """The runtime's control.joint_f for one sub-step: float32, the commanded torques, every gearbox's coupling
        torque computed from the float32 joint state its solver hands the runtime. MuJoCo receives it as qfrc_applied."""
        f = self.force.astype(np.float32)
        if self.gearboxes:
            from . import gearbox as gb
            q, v = self.d.qpos, self.d.qvel
            for dof, g in self.gearboxes.items():
                f[dof] = gb.advance(g, self.dt, float(np.float32(q[g["q_idx"]])), float(np.float32(v[dof])))
        self.d.qfrc_applied[:] = f

    def command_force(self, jid, tau):
        """A motor's (clamped) setTorque/setForce on joint `jid`: the runtime's set_joint_force."""
        dof = int(self.m.jnt_dofadr[jid])
        if dof in self.gearboxes:
            self.force[dof] = tau
            return
        if dof not in self._seen_dofs:
            self._seen_dofs.add(dof)
            if self._try_bind(jid, dof, tau):
                self.force[dof] = tau          # the engine keeps re-sending the token until the motor is commanded again
                return
        self.force[dof] = tau
        self.force_on = True

    def _try_bind(self, jid, dof, tau):
        from . import gearbox as gb
        if gb.disabled():
            return False
        try:
            decl = gb.find_declaration(tau)
            if decl is None:
                return False
            spec = decl.get("spec") or {}
            if any(self.m.actuator_trnid[a][0] == jid for a in range(self.m.nu)):
                ack = {"error": "the core puts a gearbox only on an EFFORT-built joint (OMNISIM_NEWTON_TORQUE_MODE=1); "
                                "this one has the engine's servo actuators"}
            else:
                ack = self.attach_gearbox(dof, spec)
        except Exception as e:                                 # noqa: BLE001 - the ack carries it, as in the runtime
            ack = {"error": repr(e)}
        ack.update({"token": tau, "dof": dof, "backend": "core"})
        gb.write_ack(ack)
        return "error" not in ack

    def attach_gearbox(self, dof, spec):
        """Put a gearbox on the hinge at `dof` (as the runtime's gearbox_attach), bound at this tick. -> ack."""
        from . import gearbox as gb
        j = int(self.m.dof_jntid[dof])
        g, ack = gb.prepare(spec, int(self.m.jnt_qposadr[j]), dof)
        if g is None:
            return ack
        self.gearboxes[dof] = g
        self._bound[dof] = int(self.ticks)
        self.attach_log.append([int(self.ticks), int(dof), dict(spec)])
        return ack

    def gear_state(self):
        """{dof: [rotor angle (None before its first sub-step), rotor speed]} of every gearbox."""
        return {int(dof): [g["th_m"], g["w_m"]] for dof, g in self.gearboxes.items()}

    def set_gear_state(self, st):
        for dof, (th, w) in (st or {}).items():
            g = self.gearboxes[int(dof)]
            g["th_m"], g["w_m"] = (None if th is None else float(th)), float(w)

    def state(self):
        out = np.zeros(mujoco.mj_stateSize(self.m, P_SPEC))
        mujoco.mj_getState(self.m, self.d, out, P_SPEC)
        return out


# ---------------------------------------------------------------- the controller API the core serves
def _ignore(*_a, **_kw):
    """What an unserved call returns in lenient mode (it is counted in the run's notes)."""
    return None


class _Refusing:
    """Base of every API object: a call the core does not serve is refused by name."""
    _kind = "object"

    def __getattr__(self, k):
        if k.startswith("__") or k in ("sim", "name"):
            raise AttributeError(k)
        return self.sim.unsupported("%s.%s on %s" % (self._kind, k, getattr(self, "name", "?")), result=_ignore)


class _Field:
    def __init__(self, getter):
        self._g = getter

    def getSFString(self):
        return self._g()

    def getSFVec3f(self):
        return list(self._g())

    def getSFNode(self):
        return self._g()


class _StoredField:
    """A field of a node that is only drawn. Writes are kept; nothing physical depends on them."""

    def __init__(self, owner, name):
        self._owner, self._name, self._value = owner, name, None

    def _set(self, v, setter="setSFVec3f"):
        self._value = v
        core = getattr(getattr(self._owner, "sim", None), "core", None)
        if core is not None:           # the picture of the run: `core film` plays these writes back on the engine
            core.drawn_log.append([int(core.ticks), self._owner.name, self._name, setter,
                                   list(v) if isinstance(v, (list, tuple)) else v])

    def setSFVec3f(self, v):
        self._set(v, "setSFVec3f")

    def setSFRotation(self, v):
        self._set(v, "setSFRotation")

    def setSFFloat(self, v):
        self._set(v, "setSFFloat")

    def setSFBool(self, v):
        self._set(v, "setSFBool")

    def setSFString(self, v):
        self._set(v, "setSFString")

    def setSFInt32(self, v):
        self._set(v, "setSFInt32")

    def setSFColor(self, v):
        self._set(v, "setSFColor")

    def setSFVec2f(self, v):
        self._set(v, "setSFVec2f")

    def _get(self):
        if self._value is None:
            raise CoreUnsupported("reading field %r of the drawn node %s: the core does not hold its authored value"
                                  % (self._name, self._owner.name))
        return self._value

    getSFVec3f = getSFRotation = getSFFloat = getSFBool = getSFString = getSFInt32 = getSFColor = getSFVec2f = _get


class VisualNode(_Refusing):
    """A node that only changes the picture (a drawn tool, a glow). The core keeps what is written to it."""
    _kind = "drawn node"

    def __init__(self, sim, nid, name):
        self.sim, self.id, self.name, self._fields = sim, nid, name, {}

    def getId(self):
        return self.id

    def getDef(self):
        return self.name

    def getField(self, name):
        if name not in self._fields:
            self._fields[name] = _StoredField(self, name)
        return self._fields[name]


class _PoseField:
    """translation / rotation of a free body. A write does what the engine does for a supervisor pose write:
    the body is moved, its robot's velocities are zeroed, and the whole state passes through float32."""

    def __init__(self, node, kind):
        self.node, self.kind = node, kind
        m = node.sim.core.m
        j = int(m.body_jntadr[node.body])
        self.qa = int(m.jnt_qposadr[j])
        root = int(m.body_rootid[node.body])
        # the engine zeroes the velocity of the whole robot, not only of its root body
        self.da = np.array([k for k in range(m.nv) if int(m.body_rootid[m.dof_bodyid[k]]) == root], dtype=np.intp)

    def _write(self, pos=None, quat=None):
        # In the engine a supervisor pose write reaches physics one tick late: it lands after the
        # next physics step, not before it. Queued here; Core.step applies it at the end of that step.
        self.node.sim.core.pending_pose.append((self.qa, self.da, pos, quat))

    def getSFVec3f(self):
        return [float(np.float32(x)) for x in self.node.sim.core.d.qpos[self.qa:self.qa + 3]]

    def setSFVec3f(self, v):
        self._write(pos=[float(v[0]), float(v[1]), float(v[2])])

    def getSFRotation(self):
        w, x, y, z = self.node.sim.core.d.qpos[self.qa + 3:self.qa + 7]
        sn = math.sqrt(x * x + y * y + z * z)
        return [0.0, 0.0, 1.0, 0.0] if sn < 1e-12 else [x / sn, y / sn, z / sn, 2.0 * math.atan2(sn, w)]

    def setSFRotation(self, r):
        n = math.sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]) or 1.0
        h = 0.5 * float(r[3])
        sn = math.sin(h)
        self._write(quat=[math.cos(h), r[0] / n * sn, r[1] / n * sn, r[2] / n * sn])


class Node(_Refusing):
    _kind = "Node"

    def __init__(self, sim, nid, name, body=None, bodies=None, fields=None, parent=None):
        self.sim, self.id, self.name, self.body = sim, nid, name, body
        self.bodies = set(bodies if bodies is not None else ([body] if body is not None else []))
        self.fields = fields or {}
        self.parent = parent

    def getId(self):
        return self.id

    def getDef(self):
        return self.name

    def getParentNode(self):
        return self.parent

    def _need_body(self, what):
        if self.body is None or self.body <= 0:
            self.sim.unsupported("Node.%s on %s, which has no body in the model" % (what, self.name), always=True)

    def getPosition(self):
        self._need_body("getPosition")
        return [float(np.float32(x)) for x in self.sim.core.d.xpos[self.body]]

    def getOrientation(self):
        self._need_body("getOrientation")
        return [float(np.float32(x)) for x in self.sim.core.d.xmat[self.body]]

    def getVelocity(self):
        self._need_body("getVelocity")
        v = np.zeros(6)
        mujoco.mj_objectVelocity(self.sim.core.m, self.sim.core.d, mujoco.mjtObj.mjOBJ_BODY, self.body, v, 0)
        return [float(x) for x in (v[3], v[4], v[5], v[0], v[1], v[2])]

    def getField(self, name):
        if name == "name":
            return _Field(lambda: self.name)
        if name in ("translation", "rotation") and self.body is not None and self.body > 0:
            m = self.sim.core.m
            j = int(m.body_jntadr[self.body])
            if j >= 0 and m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
                return _PoseField(self, name)
        if name in self.fields:
            return _Field(self.fields[name])
        return self.sim.unsupported("Node.getField(%r) on %s" % (name, self.name), result=None)

    def getContactPoints(self, include_descendants=False):
        c = self.sim.core
        m, d = c.m, c.d
        out = []
        mine = self.bodies if include_descendants else ({self.body} if self.body is not None else self.bodies)
        for i in range(d.ncon):
            con = d.contact[i]
            b1, b2 = int(m.geom_bodyid[con.geom1]), int(m.geom_bodyid[con.geom2])
            for b in (b1, b2):
                if b in mine:
                    out.append(ContactPoint(
                        point=[float(x) for x in con.pos], depth=float(max(0.0, -con.dist)),
                        node_id=(self.sim.body_node[b].id if include_descendants and b in self.sim.body_node else self.id)))
                    break
        return out


class ContactPoint(types.SimpleNamespace):
    """omnisim.ContactPoint: the point (MuJoCo's contact position) and the node it belongs to."""

    def getPoint(self):
        return list(self.point)

    def getNodeId(self):
        return self.node_id


class StaticNode(_Refusing):
    """A static Solid nothing carries (no Physics, no moving ancestor): its authored pose, which is what the engine
    reports for it all run (doubles: it is never read back from the solver's float32 state). Everything else is
    refused."""
    _kind = "Node"

    def __init__(self, sim, nid, name, pos, quat):
        self.sim, self.id, self.name = sim, nid, name
        self.pos = [float(x) for x in pos]
        x, y, z, w = (float(v) for v in quat)
        self.mat = [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
                    2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
                    2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]

    def getId(self):
        return self.id

    def getDef(self):
        return self.name

    def getPosition(self):
        return list(self.pos)

    def getOrientation(self):
        return list(self.mat)

    def getVelocity(self):
        return [0.0] * 6

    def getField(self, name):
        if name == "name":
            return _Field(lambda: self.name)
        return self.sim.unsupported("Node.getField(%r) on the static Solid %s" % (name, self.name), result=None)


def _quat_mat(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class FrameNode(_Refusing):
    """A DEF'd Pose or Transform fixed to a robot's Solid (a tool point): the body's pose as the solver has it
    (float32 position and xyzw quaternion), composed in double with the frame's authored offset.

    NOT ENGINE-EXACT. The engine answers through its scene tree (each Solid's Newton pose projected into its
    parent's frame and composed back up the chain), and on an arm's tool point the two differ by up to 0.1 mm
    (engine 0.5749 / 0.1001 m against the core's 0.575 / 0.1 m at the reach pose, the physics bit-identical). Each
    read is counted in the run's report (`lenient_notes`), so a result that used one says so."""

    NOTE = "Node.getPosition/getOrientation on a robot frame (DEF'd Pose): computed, not engine-exact (up to 0.1 mm)"
    _kind = "Node"

    def __init__(self, sim, nid, name, body, pos, quat, drawn_only=False):
        self.sim, self.id, self.name, self.body = sim, nid, name, int(body)
        self.t = np.array(pos, dtype=np.float64)
        self.R = _quat_mat(*(float(v) for v in quat))
        self.drawn_only, self._fields = bool(drawn_only), {}

    def _body(self):
        d = self.sim.core.d
        p = np.array([float(np.float32(v)) for v in d.xpos[self.body]])
        w, x, y, z = (float(np.float32(v)) for v in d.xquat[self.body])      # MuJoCo is wxyz
        return p, _quat_mat(x, y, z, w)

    def getId(self):
        return self.id

    def getDef(self):
        return self.name

    def _note(self):
        notes = self.sim.notes
        notes[self.NOTE] = notes.get(self.NOTE, 0) + 1

    def getPosition(self):
        self._note()
        p, R = self._body()
        return [float(v) for v in p + R @ self.t]

    def getOrientation(self):
        self._note()
        _, R = self._body()
        return [float(v) for v in (R @ self.R).reshape(-1)]

    def getField(self, name):
        if name == "name":
            return _Field(lambda: self.name)
        if self.drawn_only:
            # Nothing physical under this frame (a drawn tool, a spinning bit): what is written to it only changes
            # the picture, so it is kept like a VisualNode's field. getPosition still answers the authored frame.
            if name not in self._fields:
                self._fields[name] = _StoredField(self, name)
            return self._fields[name]
        return self.sim.unsupported("Node.getField(%r) on the frame %s" % (name, self.name), result=None)


class PositionSensor(_Refusing):
    _kind = "PositionSensor"

    def __init__(self, sim, jid, name):
        self.sim, self.jid, self.name, self._tag = sim, jid, name, ("sensor", jid)
        self.enabled = False

    def enable(self, period):
        self.enabled = True

    def disable(self):
        self.enabled = False

    def getSamplingPeriod(self):
        return int(self.sim.dt_ms)

    def getName(self):
        return self.name

    def getValue(self):
        c = self.sim.core
        if c.ticks == 0 or not self.enabled:
            return float("nan")
        m = c.m
        x = float(np.float32(c.d.qpos[m.jnt_qposadr[self.jid]]))      # the engine serves float32 state
        lo, hi = m.jnt_range[self.jid]
        return min(max(x, float(lo)), float(hi)) if lo != hi else x


class Motor(_Refusing):
    _kind = "Motor"

    def __init__(self, sim, jid, name, vmax, fmax, servo=True, sensor_name=None):
        self.sim, self.jid, self.name, self._tag = sim, jid, name, ("motor", jid)
        self.vmax, self.fmax, self.servo = vmax, fmax, servo
        self.sensor = PositionSensor(sim, jid, sensor_name or name.replace("_motor", "_sensor"))
        self.target = None
        # the model carries one position actuator and one velocity actuator per joint -- or none at all, when the
        # world was compiled with OMNISIM_NEWTON_TORQUE_MODE=1 (an EFFORT joint: the motor only takes a torque)
        acts = [a for a in range(sim.core.m.nu) if sim.core.m.actuator_trnid[a][0] == jid]
        self.effort = not acts
        self.a_pos, self.a_vel = (acts[0], acts[1]) if acts else (None, None)
        self.available = fmax

    def getName(self):
        return self.name

    def setTorque(self, tau):
        """OmMotor::setForceOrTorque: clamped to the available torque, then the runtime's set_joint_force."""
        if not self.effort:
            return self.sim.unsupported("Motor.setTorque/setForce on %s, a joint built with the engine's servo actuators "
                                        "(compile the world with OMNISIM_NEWTON_TORQUE_MODE=1)" % self.name)
        tau = float(tau)
        if math.isnan(tau):
            return                     # libController refuses NaN with an error and sends nothing
        if abs(tau) > self.available:
            tau = self.available if tau >= 0.0 else -self.available
        self.sim.core.command_force(self.jid, tau)

    setForce = setTorque

    def setPosition(self, q):
        if self.effort:
            return self.sim.unsupported("Motor.setPosition on %s, an EFFORT joint (force mode)" % self.name)
        if not self.servo or math.isinf(q):
            return        # a joint without limits is a velocity wheel in the engine: setPosition is ignored
        self.target = float(q)
        self.sim.core.ctrl[self.a_pos] = float(np.float32(q))        # the engine carries targets as float32

    def setVelocity(self, v):
        if self.effort:
            return self.sim.unsupported("Motor.setVelocity on %s, an EFFORT joint (force mode)" % self.name)
        if not self.servo:
            self.sim.core.ctrl[self.a_vel] = float(np.float32(v))
            return
        if abs(float(v) - self.vmax) > 1e-9:
            self.sim.unsupported("Motor.setVelocity(%r) below the maximum %r on the position servo %s" % (v, self.vmax, self.name))

    def setAvailableForce(self, f):
        if abs(float(f) - self.fmax) > 1e-9:
            self.sim.unsupported("Motor.setAvailableForce(%r) other than the maximum %r on %s" % (f, self.fmax, self.name))

    setAvailableTorque = setAvailableForce

    def getMaxVelocity(self):
        return self.vmax

    def enableTorqueFeedback(self, period):
        pass

    enableForceFeedback = enableTorqueFeedback

    def getTorqueFeedback(self):
        return 0.0      # the engine reports 0 too (not measured on this backend)

    getForceFeedback = getTorqueFeedback

    def getVelocity(self):
        return self.vmax

    def getMaxForce(self):
        return self.fmax

    getMaxTorque = getAvailableForce = getAvailableTorque = getMaxForce

    def getMinPosition(self):
        return float(self.sim.core.m.jnt_range[self.jid][0])

    def getMaxPosition(self):
        return float(self.sim.core.m.jnt_range[self.jid][1])

    def getTargetPosition(self):
        return self.target if self.target is not None else 0.0

    def getPositionSensor(self):
        return self.sensor


class SimQuit(Exception):
    """A controller called simulationQuit."""


class Supervisor(_Refusing):
    """The robot a controller holds. One per controller; the runner sets `_next_cfg` before the script starts."""
    _kind = "Robot"
    _next_cfg = None

    def __init__(self):
        cfg = Supervisor._next_cfg
        if cfg is None:
            raise RuntimeError("omnisim_core: a controller constructed a Robot outside a core run")
        self.cfg, self.sim = cfg, self
        self.name = cfg.get("robot_name", "robot")
        self.core, self.dt_ms, self.max_ticks = cfg["core"], cfg["dt_ms"], cfg["max_ticks"]
        self.strict = bool(cfg.get("strict", True))
        self.notes = cfg.setdefault("notes", {})
        self.wall_core = 0.0
        self._t_ticks, self._t_ms = 0, 0.0          # getTime's millisecond count, grown as the engine grows it
        self.devices = {}
        for mo in cfg["motors"]:
            motor = Motor(self, mo["jid"], mo["name"], mo["vmax"], mo["fmax"], servo=mo["servo"], sensor_name=mo.get("sensor"))
            self.devices[mo["name"]] = motor
            self.devices[motor.sensor.name] = motor.sensor
        self.body_node, self.by_id = {}, {}
        self._nid = 100
        for b, name in cfg["bodies"].items():
            n = Node(self, self._new_id(), name, body=b)
            self.body_node[b] = n
            self.by_id[n.id] = n
        robot_bodies = cfg["robot_bodies"]
        self.robot_node = Node(self, self._new_id(), cfg["robot_def"], body=(robot_bodies[0] if robot_bodies else None), bodies=robot_bodies)
        self.by_id[self.robot_node.id] = self.robot_node
        self.defs = {cfg["robot_def"]: self.robot_node}
        for d, b in cfg["defs"].items():
            self.defs[d] = self.body_node[b]
        self._visual = {}
        self._static = {}
        # device -> owning node -> joint -> endPoint, the route a controller walks to find a finger's Solid
        self.dev_nodes = {}
        for mo in cfg["motors"]:
            child = self.body_node.get(int(self.core.m.jnt_bodyid[mo["jid"]]))
            joint = Node(self, self._new_id(), "joint_of_" + mo["name"], fields={"endPoint": (lambda c=child: c)})
            for tag in (("motor", mo["jid"]), ("sensor", mo["jid"])):
                self.dev_nodes[tag] = Node(self, self._new_id(), "device_of_" + mo["name"], parent=joint)

    def _new_id(self):
        self._nid += 1
        return self._nid

    def unsupported(self, what, result=None, always=False):
        """Refuse (strict) or count (lenient) a call the core does not model."""
        if self.strict or always:
            raise CoreUnsupported("the core does not model " + what)
        self.notes[what] = self.notes.get(what, 0) + 1
        return result

    def getBasicTimeStep(self):
        return float(self.dt_ms)

    def getTime(self):
        # OmRobot sends 0.001 * OmSimulationState::time(), a millisecond count grown by basicTimeStep each tick.
        # Not ticks * dt / 1000: 0.001 * k and k / 1000 differ in the last bit for some k, and a controller that
        # ends a phase on `getTime() - t0 >= T` then ends it one tick apart (measured on an arm payload test).
        n = self.core.ticks
        if n < self._t_ticks:
            self._t_ticks, self._t_ms = 0, 0.0
        while self._t_ticks < n:
            self._t_ms += self.dt_ms
            self._t_ticks += 1
        return 0.001 * self._t_ms

    def getName(self):
        return self.name

    def getCustomData(self):
        return self.cfg.get("custom_data", "")

    def getDevice(self, name):
        dev = self.devices.get(name)
        if dev is not None:
            return dev
        if name in self.cfg.get("other_devices", ()) or not self.cfg.get("devices_complete", False):
            return self.unsupported("the device %r of %s" % (name, self.name), result=None)
        return None            # the world declares no such device: the engine answers None too

    def getFromDef(self, name):
        obj = self.cfg.get("objects", {}).get(name)
        if obj is not None:
            return obj
        n = self.defs.get(name)
        if n is not None:
            return n
        st = self.cfg.get("static_defs", {}).get(name)
        if st is not None:
            if name not in self._static:
                self._static[name] = StaticNode(self, self._new_id(), name, st["pos"], st["quat"])
                self.by_id[self._static[name].id] = self._static[name]
            return self._static[name]
        fr = self.cfg.get("frames", {}).get(name)
        if fr is not None:
            if name not in self._static:
                self._static[name] = FrameNode(self, self._new_id(), name, fr["body"], fr["pos"], fr["quat"],
                                               fr.get("drawn_only", False))
                self.by_id[self._static[name].id] = self._static[name]
            return self._static[name]
        leaf = name.split(".")[-1]
        kind = self.cfg.get("all_defs", {}).get(leaf)
        if kind is None:
            if self.cfg.get("defs_complete", False):
                return None    # no such DEF in the world: the engine answers None too
            return self.unsupported("getFromDef(%r): part of this world could not be read" % name, result=None)
        if kind in self.cfg.get("physical_types", ()):
            return self.unsupported("getFromDef(%r), a %s the core has no handle for" % (name, kind), result=None)
        if leaf not in self._visual:
            self._visual[leaf] = VisualNode(self, self._new_id(), leaf)
            self.by_id[self._visual[leaf].id] = self._visual[leaf]
        return self._visual[leaf]

    def getFromId(self, nid):
        return self.by_id.get(nid)

    def getFromDevice(self, device):
        return self.dev_nodes.get(getattr(device, "_tag", device))

    def getSelf(self):
        return self.robot_node

    def step(self, ms):
        sch = self.cfg.get("scheduler")
        if sch is not None:
            return sch(self, ms)          # the runner owns time: hand control back until this controller is due
        t = time.perf_counter()
        for _ in range(max(1, int(round(ms / self.dt_ms)))):
            if self.core.ticks >= self.max_ticks:
                return -1
            self.core.step()
            hook = self.cfg.get("on_tick")
            if hook:
                hook(self.core)
        self.wall_core += time.perf_counter() - t
        return 0

    def simulationQuit(self, code):
        raise SimQuit(code)


Robot = Supervisor
