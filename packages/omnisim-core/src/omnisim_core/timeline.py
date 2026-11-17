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

"""Time control for a core run: record, checkpoint, replay, branch, diff.

A core run is one MuJoCo state stepped by one loop, and everything that reaches the physics from
outside goes through two doors: the actuator commands and the supervisor's pose writes. Record
those per tick, keep the full physics state every so often (a keyframe), and a finished run can
be:

  * REPLAYED exactly: the same commands on the same model give the same state, bit for bit.
    Every tick's state is fingerprinted while recording, so a replay proves it.
  * OPENED AT ANY TICK: the nearest keyframe is restored and stepped forward. What was touching
    what, with what force, where every joint stood against its limits.
  * BRANCHED: from tick N, with a change (a slippery block, a heavier part, a weaker servo, a
    shove), the rest of the run is stepped again and recorded as a run of its own.
  * DIFFED against another run: the first tick the two differ, whether the commands or the
    physics differed first, and which joints and bodies went where.

WHAT A BRANCH IS NOT. A branch replays the RECORDED commands: the controller is not there to
react to the change. It answers "with these commands, would the grasp have held?". To let the
controller react, run the world again with the same change scheduled (`record(...,
intervention=...)`); that run can be checked tick by tick against this one up to the change,
which also says whether the controller is deterministic at all.

NOT in a recording: a controller's own memory, a MillingStock's height map (the stock is carved,
not stepped; a twin run's axes are recorded, its part is not), anything a controller read from
a socket or a clock.
"""

from __future__ import annotations

import copy
import hashlib
import json
import shutil
from pathlib import Path

from .errors import CoreUnsupported
from .runtime import ensure_runtime

_ok, _why = ensure_runtime()
if not _ok:
    raise ImportError(_why)

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from . import core as core_mod  # noqa: E402

FORMAT = 1
FULL = int(mujoco.mjtState.mjSTATE_INTEGRATION)
DATA, INFO = "timeline.npz", "timeline.json"
EDIT_KINDS = ("friction", "mass", "gain", "qpos", "qvel", "pos", "vel", "ctrl")
MODEL_EDITS = ("friction", "mass", "gain")


def state_hash(core):
    """A fingerprint of the positions and velocities, as a uint64."""
    h = hashlib.blake2b(core.d.qpos.tobytes(), digest_size=8)
    h.update(core.d.qvel.tobytes())
    return int.from_bytes(h.digest(), "little")


def full_state(core):
    out = np.zeros(mujoco.mj_stateSize(core.m, FULL))
    mujoco.mj_getState(core.m, core.d, out, FULL)
    return out


def restore(core, state, ticks):
    """Put the core at a keyframe: the full physics state, the tick count, and fresh derived quantities."""
    mujoco.mj_setState(core.m, core.d, np.asarray(state, dtype=np.float64), FULL)
    core.ticks = int(ticks)
    core.ctrl[:] = core.d.ctrl
    core.pending_pose = []
    mujoco.mj_step1(core.m, core.d)


# ---------------------------------------------------------------- names
def name_tables(meta_names, model):
    """-> (body name -> id, body id -> name, motor name -> motor dict). Names come from the world; ids from the model."""
    by_name, by_id, motors = {}, {}, {}

    def put(name, b):
        if name and b is not None and int(b) > 0:
            by_name.setdefault(str(name), int(b))
            by_id.setdefault(int(b), str(name))

    for dn, info in (meta_names.get("defs") or {}).items():
        put(dn, info.get("body"))
    for sname, b in (meta_names.get("solid_body") or {}).items():
        put(sname, b)
    for rob in meta_names.get("robots") or []:
        for mo in rob.get("motors") or []:
            motors[mo["name"]] = mo
            put(mo.get("child_link"), mo.get("body"))
        if rob.get("bodies"):
            put(rob.get("def") or rob.get("name"), rob.get("root_body"))
    for b in range(1, model.nbody):
        by_id.setdefault(b, mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or "body_%d" % b)
        by_name.setdefault(by_id[b], b)
    by_id[0] = "world"
    return by_name, by_id, motors


# ---------------------------------------------------------------- edits (what a branch or an intervention changes)
def parse_edit(text):
    """'friction:BLOCK=0.3' -> {'kind': 'friction', 'target': 'BLOCK', 'value': [0.3]}"""
    try:
        head, value = text.split("=", 1)
        kind, target = head.split(":", 1)
        values = [float(x) for x in value.split(",")]
    except ValueError as exc:
        raise ValueError("an edit reads kind:target=value (for example friction:BLOCK=0.3), got %r" % text) from exc
    kind, target = kind.strip(), target.strip()
    if kind not in EDIT_KINDS:
        raise ValueError("unknown edit %r (known: %s)" % (kind, ", ".join(EDIT_KINDS)))
    need = 3 if kind in ("pos", "vel") else 1
    if len(values) != need:
        raise ValueError("%s takes %d value(s), got %d in %r" % (kind, need, len(values), text))
    return {"kind": kind, "target": target, "value": values}


def _body(target, by_name, model):
    if target.startswith("#"):
        return int(target[1:])
    if target not in by_name:
        raise CoreUnsupported("no body named %r in this run (known: %s)" % (target, ", ".join(sorted(by_name))[:300]))
    return by_name[target]


def _motor(target, motors):
    if target not in motors:
        raise CoreUnsupported("no motor named %r in this run (known: %s)" % (target, ", ".join(sorted(motors))[:300]))
    return motors[target]


def apply_edits(core, edits, tables, model_only=False):
    """Apply edits to a core. -> the persistent command overrides [(actuator, value)] among them."""
    by_name, _, motors = tables
    m, d = core.m, core.d
    overrides, touched_state, touched_mass = [], False, False
    for e in edits:
        kind, target, v = e["kind"], e["target"], e["value"]
        if kind == "friction":
            b = _body(target, by_name, m)
            geoms = np.nonzero(m.geom_bodyid == b)[0]
            if not len(geoms):
                raise CoreUnsupported("the body %r has no collider to give a friction to" % target)
            # MuJoCo uses the LARGER friction of two touching geoms unless one has priority: give this body's
            # colliders priority, so its contacts really use the value asked for.
            m.geom_friction[geoms, 0] = v[0]
            m.geom_priority[geoms] = int(np.delete(m.geom_priority, geoms).max(initial=0)) + 1
        elif kind == "mass":
            b = _body(target, by_name, m)
            if m.body_mass[b] <= 0.0 or v[0] <= 0.0:
                raise CoreUnsupported("mass edit needs a body with mass and a positive value (%r)" % target)
            s = v[0] / float(m.body_mass[b])
            m.body_mass[b] *= s
            m.body_inertia[b] *= s
            touched_mass = True
        elif kind == "gain":
            mo = _motor(target, motors)
            if mo.get("effort"):
                raise CoreUnsupported("gain edit on %r, an EFFORT joint (force mode): it has no servo gain" % target)
            for a in (mo["a_pos"], mo["a_vel"]):
                m.actuator_gainprm[a, 0] *= v[0]
                m.actuator_biasprm[a, 1:3] *= v[0]
        elif model_only:
            continue
        elif kind in ("qpos", "qvel"):
            mo = _motor(target, motors)
            if kind == "qpos":
                d.qpos[m.jnt_qposadr[mo["jid"]]] = v[0]
            else:
                d.qvel[m.jnt_dofadr[mo["jid"]]] = v[0]
            touched_state = True
        elif kind in ("pos", "vel"):
            b = _body(target, by_name, m)
            j = int(m.body_jntadr[b])
            if j < 0 or m.jnt_type[j] != mujoco.mjtJoint.mjJNT_FREE:
                raise CoreUnsupported("%s edit needs a free body; %r is jointed or fixed" % (kind, target))
            if kind == "pos":
                d.qpos[m.jnt_qposadr[j]:m.jnt_qposadr[j] + 3] = v
            else:
                d.qvel[m.jnt_dofadr[j]:m.jnt_dofadr[j] + 3] = v
            touched_state = True
        elif kind == "ctrl":
            mo = _motor(target, motors)
            if mo.get("effort"):
                raise CoreUnsupported("ctrl edit on %r, an EFFORT joint (force mode): its command is a torque" % target)
            overrides.append((int(mo["a_pos"] if mo["servo"] else mo["a_vel"]), float(v[0])))
    if touched_mass:
        mujoco.mj_setConst(m, mujoco.MjData(m))       # derived constants, on scratch data: the run's state is untouched
    if touched_state or touched_mass or any(e["kind"] in MODEL_EDITS for e in edits):
        mujoco.mj_step1(m, d)                          # poses and contacts for whoever reads next
    return overrides


# ---------------------------------------------------------------- recording
DRAWN = "drawn.npz"


def save_drawn(out_dir, log):
    """The writes the run's controllers made to drawn nodes (a spinning bit, a blur sleeve, a tool swap), compactly:
    tick, which (node, field, setter), and up to four numbers. Nothing physical; `core film` replays them."""
    if not log:
        return
    keys, rows = [], []
    index = {}
    for tick, node, field, setter, value in log:
        vals = value if isinstance(value, list) else [value]
        if not all(isinstance(x, (int, float, bool)) for x in vals) or len(vals) > 4:
            continue                   # a string or a long value: not a pose, not replayed
        k = index.setdefault((node, field, setter, len(vals)), len(keys))
        if k == len(keys):
            keys.append([node, field, setter, len(vals)])
        rows.append((tick, k, [float(x) for x in vals] + [0.0] * (4 - len(vals))))
    if rows:
        np.savez_compressed(Path(out_dir) / DRAWN, keys=json.dumps(keys), tick=np.array([r[0] for r in rows], np.int64),
                            key=np.array([r[1] for r in rows], np.int32), value=np.array([r[2] for r in rows], np.float64))


class Recorder:
    """Attach to a Core (`core.recorder = Recorder(core)`): every tick's commands, pose writes and state fingerprint,
    and the full state every `keyframe_every` ticks."""

    def __init__(self, core, keyframe_every=250):
        self.every = max(1, int(keyframe_every))
        self.ctrl, self.hash, self.writes = [], [], []
        self.key_tick, self.key_state = [], []
        # force mode: each tick's commanded torques, the gearboxes in the order they were bound, and every
        # gearbox's rotor state at each keyframe (the rotor is integrated state the MuJoCo state does not hold)
        self.force, self.attach, self.key_gear = [], [], []
        self._attach_seen = len(core.attach_log)
        self.first_tick = int(core.ticks)
        self._core = core              # its drawn_log (writes to drawn nodes) is saved with the recording
        self.edit_tick, self.edits, self.parent = None, [], None
        self._key(core)

    def _key(self, core):
        self.key_tick.append(int(core.ticks))
        self.key_state.append(full_state(core))
        self.key_gear.append({str(k): v for k, v in core.gear_state().items()})

    def begin(self, core, writes):
        self.ctrl.append(core.ctrl.copy())
        self.force.append(core.force.copy())
        if len(core.attach_log) > self._attach_seen:
            self.attach.extend(core.attach_log[self._attach_seen:])
            self._attach_seen = len(core.attach_log)
        if writes:
            self.writes.append([int(core.ticks), [[int(qa), [int(x) for x in da], pos, quat] for qa, da, pos, quat in writes]])

    def end(self, core):
        self.hash.append(state_hash(core))
        if core.ticks % self.every == 0:
            self._key(core)

    def save(self, out_dir, compiled_dir, info=None):
        """Write a self-contained recording: the compiled world it ran on, and the timeline."""
        out_dir, compiled_dir = Path(out_dir), Path(compiled_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        if out_dir.resolve() != compiled_dir.resolve():
            for name in (core_mod.MODEL, core_mod.STATE0, core_mod.CLAMP, core_mod.ENGINE, core_mod.META):
                if (compiled_dir / name).exists():
                    shutil.copy2(compiled_dir / name, out_dir / name)
        nu = len(self.ctrl[0]) if self.ctrl else 0
        arrays = dict(ctrl=np.array(self.ctrl, dtype=np.float64).reshape(len(self.ctrl), nu),
                      hash=np.array(self.hash, dtype=np.uint64), key_tick=np.array(self.key_tick, dtype=np.int64),
                      key_state=np.array(self.key_state, dtype=np.float64))
        forced = bool(self.attach) or any(np.any(f) for f in self.force)
        if forced:                     # a run that never commanded a torque keeps the recording it always had
            arrays["force"] = np.array(self.force, dtype=np.float64)
        np.savez_compressed(out_dir / DATA, **arrays)
        save_drawn(out_dir, getattr(self._core, "drawn_log", None))
        doc = {"format": FORMAT, "ticks": self.first_tick + len(self.ctrl), "keyframe_every": self.every, "writes": self.writes,
               "edit_tick": self.edit_tick, "edits": self.edits, "parent": self.parent}
        if forced:
            doc.update(gearboxes=self.attach, key_gear=self.key_gear)
        doc.update(info or {})
        (out_dir / INFO).write_text(json.dumps(doc, indent=1), encoding="utf-8")
        return out_dir


class Recording:
    """A saved run. `at(tick)` is the run at that tick; `advance` steps it on with the recorded commands."""

    def __init__(self, path):
        self.path = Path(path)
        if not (self.path / INFO).exists():
            raise FileNotFoundError("no recording in %s (expected %s)" % (self.path, INFO))
        self.info = json.loads((self.path / INFO).read_text(encoding="utf-8"))
        if self.info.get("format") != FORMAT:
            raise CoreUnsupported("recording format %r (this code reads %d)" % (self.info.get("format"), FORMAT))
        z = np.load(self.path / DATA)
        self.ctrl, self.hash = z["ctrl"], z["hash"]
        self.key_tick, self.key_state = z["key_tick"], z["key_state"]
        self.force = z["force"] if "force" in z.files else None
        self._attach = self.info.get("gearboxes") or []
        self._attach_at = {}
        for t, dof, spec in self._attach:
            self._attach_at.setdefault(int(t), []).append((int(dof), spec))
        self.key_gear = self.info.get("key_gear") or []
        self.ticks = int(self.info["ticks"])
        self.edit_tick, self.edits = self.info.get("edit_tick"), self.info.get("edits") or []
        self.meta = json.loads((self.path / core_mod.META).read_text(encoding="utf-8"))
        self.dt_ms = float(self.meta["names"]["dt_ms"])
        self._writes = {int(t): [(qa, np.array(da, dtype=np.intp), pos, quat) for qa, da, pos, quat in ws] for t, ws in self.info.get("writes") or []}
        self._tables = None

    def tables(self, core):
        if self._tables is None:
            self._tables = name_tables(self.meta["names"], core.m)
        return self._tables

    def tick_of(self, seconds):
        return max(0, min(self.ticks, int(round(float(seconds) * 1000.0 / self.dt_ms))))

    def new_core(self):
        core = core_mod.Core.from_compiled(self.path)
        core._edits_applied = False
        return core

    def _restore_key(self, core, k):
        """Put `core` at keyframe k: the gearboxes bound before it, the MuJoCo state, the rotors' state."""
        kt = int(self.key_tick[k])
        for t, dof, spec in self._attach:
            if int(t) < kt and int(dof) not in core.gearboxes:
                core.attach_gearbox(int(dof), spec)
        restore(core, self.key_state[k], kt)
        if self.key_gear:
            core.set_gear_state(self.key_gear[k])
        if self.force is not None and kt > 0:
            core.force[:] = self.force[kt - 1]

    def _feed(self, core, t):
        """What a controller sent before tick t besides the actuator commands: torques, gearbox declarations."""
        if self.force is not None:
            core.force[:] = self.force[t]
            core.force_on = True
        for dof, spec in self._attach_at.get(t, ()):
            if dof not in core.gearboxes:
                core.attach_gearbox(dof, spec)

    def at(self, tick):
        """A core standing where this run stood after `tick` ticks (before any change scheduled AT that tick)."""
        tick = int(tick)
        if not 0 <= tick <= self.ticks:
            raise ValueError("tick %d is outside this run (0 to %d)" % (tick, self.ticks))
        core = self.new_core()
        k = int(np.searchsorted(self.key_tick, tick, side="right")) - 1
        kt = int(self.key_tick[k])
        if self.edit_tick is not None and kt > self.edit_tick:
            apply_edits(core, self.edits, self.tables(core), model_only=True)      # the keyframe already holds the changed state
            core._edits_applied = True
        self._restore_key(core, k)
        self.advance(core, tick)
        return core

    def advance(self, core, to_tick, on_tick=None):
        """Step `core` to `to_tick` with the recorded commands and pose writes."""
        to_tick = min(int(to_tick), self.ticks)
        while core.ticks < to_tick:
            t = core.ticks
            if self.edit_tick is not None and t == self.edit_tick and not getattr(core, "_edits_applied", False):
                apply_edits(core, self.edits, self.tables(core))
                core._edits_applied = True
            core.ctrl[:] = self.ctrl[t]
            self._feed(core, t)
            core.pending_pose = list(self._writes.get(t, ()))
            core.step()
            if on_tick is not None:
                on_tick(core)
        return core

    def verify(self):
        """Step the whole run again from its recorded commands. -> report; `exact` when every tick's fingerprint matches."""
        core = self.new_core()
        self._restore_key(core, 0)
        first, n = None, 0

        def check(c):
            nonlocal first, n
            n += 1
            if first is None and state_hash(c) != int(self.hash[c.ticks - 1]):
                first = c.ticks - 1

        self.advance(core, self.ticks, on_tick=check)
        # and each keyframe must bring back exactly the state the straight run had there
        bad_keys = []
        for k in range(1, len(self.key_tick)):
            c = self.new_core()
            if self.edit_tick is not None and int(self.key_tick[k]) > self.edit_tick:
                apply_edits(c, self.edits, self.tables(c), model_only=True)
            self._restore_key(c, k)
            if state_hash(c) != int(self.hash[int(self.key_tick[k]) - 1]):
                bad_keys.append(int(self.key_tick[k]))
        # and stepping on from a keyframe in the middle must arrive where the straight run did
        seek_ok = True
        if len(self.key_tick) > 2:
            mid = int(self.key_tick[len(self.key_tick) // 2])
            c = self.at(mid)
            self.advance(c, self.ticks)
            seek_ok = state_hash(c) == int(self.hash[self.ticks - 1])
        return {"ticks": n, "exact": first is None and not bad_keys and seek_ok, "first_tick_that_differs": first,
                "keyframes": int(len(self.key_tick)), "keyframes_that_differ": bad_keys, "seek_from_the_middle_arrives_the_same": seek_ok}


# ---------------------------------------------------------------- looking at a moment
def snapshot(rec, core, forces=True):
    """Everything at the core's current tick: joints against their limits and commands, bodies, contacts with forces."""
    m, d = core.m, core.d
    _, by_id, motors = rec.tables(core)
    f = d
    if forces:
        f = copy.copy(d)                       # contact and actuator forces for THIS state, without touching the run
        f.ctrl[:] = core.ctrl
        mujoco.mj_forward(m, f)
    joints = []
    for name, mo in motors.items():
        j = mo["jid"]
        q, v = float(d.qpos[m.jnt_qposadr[j]]), float(d.qvel[m.jnt_dofadr[j]])
        lo, hi = float(m.jnt_range[j][0]), float(m.jnt_range[j][1])
        if mo.get("effort"):
            dof = int(m.jnt_dofadr[j])
            row = {"motor": name, "position": q, "velocity": v, "torque_command": float(core.force[dof]), "servo": False,
                   "effort": True, "gearbox": dof in core.gearboxes}
        else:
            row = {"motor": name, "position": q, "velocity": v, "command": float(core.ctrl[mo["a_pos"] if mo["servo"] else mo["a_vel"]]),
                   "servo": bool(mo["servo"])}
        if lo != hi:
            row.update(min=lo, max=hi, at_limit=bool(q <= lo + 1e-4 or q >= hi - 1e-4))
        if forces:
            if mo.get("effort"):
                row["applied_torque"] = float(f.qfrc_applied[m.jnt_dofadr[j]])     # the gearbox's coupling, on a gearbox joint
            else:
                row["actuator_force"] = float(f.qfrc_actuator[m.jnt_dofadr[j]])
        joints.append(row)
    bodies = []
    for b in range(1, m.nbody):
        vel = np.zeros(6)
        mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, b, vel, 0)
        bodies.append({"body": by_id[b], "position": [float(x) for x in d.xpos[b]], "quaternion_wxyz": [float(x) for x in d.xquat[b]],
                       "linear_velocity": [float(x) for x in vel[3:]], "angular_velocity": [float(x) for x in vel[:3]]})
    contacts, wrench = [], np.zeros(6)
    for i in range(f.ncon):
        c = f.contact[i]
        row = {"a": by_id[int(m.geom_bodyid[c.geom1])], "b": by_id[int(m.geom_bodyid[c.geom2])], "position": [float(x) for x in c.pos],
               "depth_mm": float(max(0.0, -c.dist)) * 1e3, "friction": float(c.friction[0])}
        if forces:
            mujoco.mj_contactForce(m, f, i, wrench)
            row.update(normal_force=float(wrench[0]), friction_force=float(math_hypot(wrench[1], wrench[2])))
        contacts.append(row)
    return {"tick": int(core.ticks), "time_s": round(core.ticks * rec.dt_ms / 1000.0, 6), "joints": joints, "bodies": bodies, "contacts": contacts}


def math_hypot(a, b):
    return float(np.hypot(a, b))


def events(rec, from_tick=0, to_tick=None, limit=2000):
    """What happened between two ticks, found by replaying: contacts that began and ended, joints that reached a limit."""
    to_tick = rec.ticks if to_tick is None else min(int(to_tick), rec.ticks)
    core = rec.at(from_tick)
    m = core.m
    _, by_id, motors = rec.tables(core)
    limited = [(n, mo["jid"]) for n, mo in motors.items() if m.jnt_range[mo["jid"]][0] != m.jnt_range[mo["jid"]][1]]

    def pairs(c):
        return {tuple(sorted((by_id[int(m.geom_bodyid[c.d.contact[i].geom1])], by_id[int(m.geom_bodyid[c.d.contact[i].geom2])])))
                for i in range(c.d.ncon)}

    def pinned(c):
        out = set()
        for n, j in limited:
            q = c.d.qpos[m.jnt_qposadr[j]]
            if q <= m.jnt_range[j][0] + 1e-4 or q >= m.jnt_range[j][1] - 1e-4:
                out.add(n)
        return out

    state = {"pairs": pairs(core), "pinned": pinned(core)}
    out = []

    def see(c):
        if len(out) >= limit:
            return
        p, q = pairs(c), pinned(c)
        t = {"tick": int(c.ticks), "time_s": round(c.ticks * rec.dt_ms / 1000.0, 4)}
        for a, b in sorted(p - state["pairs"]):
            out.append({**t, "type": "contact.began", "a": a, "b": b})
        for a, b in sorted(state["pairs"] - p):
            out.append({**t, "type": "contact.ended", "a": a, "b": b})
        for n in sorted(q - state["pinned"]):
            out.append({**t, "type": "joint.limit_hit", "motor": n})
        for n in sorted(state["pinned"] - q):
            out.append({**t, "type": "joint.limit_left", "motor": n})
        state["pairs"], state["pinned"] = p, q

    rec.advance(core, to_tick, on_tick=see)
    return {"from_tick": int(from_tick), "to_tick": int(to_tick), "touching_at_start": sorted(list(x) for x in pairs(rec.at(from_tick))),
            "events": out, "truncated": len(out) >= limit}


# ---------------------------------------------------------------- branching
def branch(rec, at_tick, edits, out_dir, to_tick=None):
    """From `at_tick`, apply `edits` and step the rest of the run again with the RECORDED commands. -> the new recording's path.

    The controller is not in a branch: it cannot react to the change. See the module's note.
    """
    at_tick = int(at_tick)
    if rec.edit_tick is not None:
        raise CoreUnsupported("this run already carries a change (at tick %d): branch from the original run, with all the changes"
                              % rec.edit_tick)
    if not 0 <= at_tick < rec.ticks:
        raise ValueError("tick %d is outside this run (0 to %d)" % (at_tick, rec.ticks - 1))
    to_tick = rec.ticks if to_tick is None else min(int(to_tick), rec.ticks)
    core = rec.at(at_tick)
    r = Recorder(core, rec.info.get("keyframe_every", 250))
    # the first at_tick ticks ARE the parent's
    r.first_tick = 0
    r.ctrl = [row.copy() for row in rec.ctrl[:at_tick]]
    r.force = [row.copy() for row in rec.force[:at_tick]] if rec.force is not None else [np.zeros(core.m.nv) for _ in range(at_tick)]
    r.attach = [list(a) for a in rec._attach if int(a[0]) < at_tick]
    r.hash = [int(h) for h in rec.hash[:at_tick]]
    keep = [k for k in range(len(rec.key_tick)) if int(rec.key_tick[k]) <= at_tick]
    r.key_tick = [int(rec.key_tick[k]) for k in keep]
    r.key_state = [rec.key_state[k] for k in keep]
    r.key_gear = [rec.key_gear[k] for k in keep] if rec.key_gear else [{} for _ in keep]
    r.writes = [w for w in (rec.info.get("writes") or []) if int(w[0]) < at_tick]
    r.edit_tick, r.edits = at_tick, list(edits)
    r.parent = {"path": str(rec.path), "at_tick": at_tick}
    overrides = apply_edits(core, edits, rec.tables(core))
    core._edits_applied = True
    core.recorder = r
    while core.ticks < to_tick:
        t = core.ticks
        core.ctrl[:] = rec.ctrl[t]
        rec._feed(core, t)
        for a, v in overrides:
            core.ctrl[a] = float(np.float32(v))
        core.pending_pose = list(rec._writes.get(t, ()))
        core.step()
    core.recorder = None
    return r.save(out_dir, rec.path, {"kind": "branch", "world": rec.info.get("world"), "controllers_in_the_loop": False})


class Intervention:
    """A change scheduled into a live run (controllers in the loop): applied when the run reaches `tick`."""

    def __init__(self, tick, edits, meta_names):
        self.tick, self.edits, self.meta_names = int(tick), list(edits), meta_names
        self.overrides, self.done = [], False

    def before_step(self, core):
        if not self.done and core.ticks == self.tick:
            self.overrides = apply_edits(core, self.edits, name_tables(self.meta_names, core.m))
            self.done = True
        for a, v in self.overrides:
            core.ctrl[a] = float(np.float32(v))


# ---------------------------------------------------------------- diff
def _first(mask):
    idx = np.nonzero(mask)[0]
    return int(idx[0]) if len(idx) else None


def diff(a, b, top=8, samples=12):
    """Compare two recordings of the same world. -> where they part, why (commands or physics first), and how far."""
    ca, cb = a.new_core(), b.new_core()
    if (ca.m.nq, ca.m.nv, ca.m.nu) != (cb.m.nq, cb.m.nv, cb.m.nu):
        raise CoreUnsupported("the two runs are not the same world (state sizes differ)")
    n = min(a.ticks, b.ticks)
    cmd_a, cmd_b = a.ctrl[:n], b.ctrl[:n]
    if a.force is not None or b.force is not None:      # in force mode the torques are the commands
        fa = a.force[:n] if a.force is not None else np.zeros((n, ca.m.nv))
        fb = b.force[:n] if b.force is not None else np.zeros((n, cb.m.nv))
        cmd_a, cmd_b = np.hstack([cmd_a, fa]), np.hstack([cmd_b, fb])
    first_cmd = _first(np.any(cmd_a != cmd_b, axis=1))
    first_state = _first(a.hash[:n] != b.hash[:n])
    rep = {"a": str(a.path), "b": str(b.path), "ticks": {"a": a.ticks, "b": b.ticks, "compared": n},
           "changes": {"a": {"at_tick": a.edit_tick, "edits": a.edits}, "b": {"at_tick": b.edit_tick, "edits": b.edits}},
           "identical": first_state is None and first_cmd is None and a.ticks == b.ticks,
           "first_tick_commands_differ": first_cmd, "first_tick_state_differs": first_state}
    if first_state is None:
        rep["verdict"] = ("the two runs are the same for all %d ticks compared" % n) if first_cmd is None else \
            "the commands differ from tick %d but the state never does" % first_cmd
        return rep
    dt = a.dt_ms / 1000.0
    rep["first_difference_time_s"] = round((first_state + 1) * dt, 4)
    if first_cmd is None or first_cmd > first_state:
        rep["verdict"] = ("the physics parted first (tick %d) while the commands were still the same: a different model, "
                          "state or change, not a different decision by a controller" % first_state)
    else:
        rep["verdict"] = "the commands parted first (tick %d), the state followed at tick %d: a controller decided differently" % (first_cmd, first_state)
    _, by_id, motors = a.tables(ca)
    m = ca.m

    def gap(t):
        xa, xb = a.at(t), b.at(t)
        jd = [{"motor": name, "a": float(xa.d.qpos[m.jnt_qposadr[mo["jid"]]]), "b": float(xb.d.qpos[m.jnt_qposadr[mo["jid"]]])} for name, mo in motors.items()]
        for row in jd:
            row["difference"] = row["b"] - row["a"]
        bd = [{"body": by_id[i], "a": [float(x) for x in xa.d.xpos[i]], "b": [float(x) for x in xb.d.xpos[i]],
               "distance_mm": float(np.linalg.norm(xa.d.xpos[i] - xb.d.xpos[i])) * 1e3} for i in range(1, m.nbody)]
        return xa, xb, sorted(jd, key=lambda r: -abs(r["difference"])), sorted(bd, key=lambda r: -r["distance_mm"])

    xa, xb, jd, bd = gap(n)
    rep["at_the_end"] = {"tick": n, "joints": [r for r in jd if r["difference"] != 0.0][:top], "bodies": [r for r in bd if r["distance_mm"] > 0.0][:top]}

    def touching(c):
        return {tuple(sorted((by_id[int(m.geom_bodyid[c.d.contact[i].geom1])], by_id[int(m.geom_bodyid[c.d.contact[i].geom2])]))) for i in range(c.d.ncon)}

    ta, tb = touching(xa), touching(xb)
    rep["at_the_end"]["touching_only_in_a"] = sorted(list(x) for x in ta - tb)
    rep["at_the_end"]["touching_only_in_b"] = sorted(list(x) for x in tb - ta)
    # how the gap grew: the body that ends up furthest apart, sampled between the first difference and the end
    if bd and bd[0]["distance_mm"] > 0.0:
        worst = bd[0]["body"]
        ticks = sorted({int(round(x)) for x in np.linspace(first_state + 1, n, samples)})
        growth = []
        for t in ticks:
            _, _, _, bdt = gap(t)
            growth.append({"tick": t, "time_s": round(t * dt, 4), "distance_mm": next(r["distance_mm"] for r in bdt if r["body"] == worst)})
        rep["growth"] = {"body": worst, "samples": growth}
    return rep
