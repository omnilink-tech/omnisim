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

"""Reading one MuJoCo run off disk. **No neutral schema in here.**

The *file* half of the MuJoCo column, shaped exactly like the Webots arm's
:mod:`agentbench.adapters.webots.recording`: it finds and parses the artifacts a
run left behind, records what was missing and what failed to parse, and hands
the parsed documents to :mod:`ladder.adapters.mujoco.evidence` for the mapping
into the neutral bundle. Nothing here imports the neutral schema and **nothing
here raises** -- a broken simulator is a measurement, not an exception (adapter
rule 1).

Why the artifact set is shaped like this
----------------------------------------

MuJoCo is a **library**, not an application. There is no simulator process to
launch, no world to load, no controller to spawn, no log file to read, and no
scene-tree service to query -- there is ``mj_loadXML``, ``mj_step``, and
whatever the caller writes down. That is a very different starting position
from either of the other two columns and it cuts both ways:

*it gives us more*
    the recorder owns the loop, so a t=0 scan is frozen **by construction**
    (nothing can advance the clock while it runs) and the pose series really is
    one sample per physics step rather than one per polling round.

*it gives us less*
    every fact that the other columns read out of an engine-written log or a
    verdict sidecar has to be produced by the runner itself. So this module
    carefully distinguishes the one **engine-authored** artifact
    (``model_saved.xml``, written by ``mj_saveLastXML``) from the runner's own
    attestations, and the evidence builder cites them separately. A finalize
    witness that is only the driver saying "I finished" is a weaker witness
    than one the engine wrote, and the row says so.

======================  ====================================================
file                    what it must contain
======================  ====================================================
``trajectory.json``     ``{"dt_s", "recorded_s", "complete",
                        "record_stride_steps", "physics_timestep_s",
                        "bodies": [{"name", "id", "t": [...],
                        "xyz": [[x,y,z], ...]}]}``. THE primary input.
``roster.json``         ``{"t_s", "frozen", "bodies": [...]}`` -- per body:
                        ``name``, ``id``, ``position``, ``aabb_min`` /
                        ``aabb_max``, ``mass_kg``, ``n_joints``,
                        ``n_joints_subtree``, ``has_free_joint``,
                        ``in_robot_subtree``.
``contacts.json``       ``{"supported", "steps", "window_s",
                        "total_observed", "distinct_named",
                        "emit_stride_steps", "pairs": [...]}``. **Omit the
                        file entirely** if no contact scan ran: absent becomes
                        ``None``, which the core reports as vacuous. A file
                        claiming ``supported: true`` with no counters is worse
                        than no file.
``completion.json``     the driver's attestation plus the compiled model's
                        shape and the engine facts read off ``mjOption``.
``model_saved.xml``     engine-authored (``mj_saveLastXML``). Its existence
                        and non-emptiness is the compile witness.
``process.json``        ``{"exit_code", "timed_out", "wall_s", "command",
                        "mujoco_version"}``.
``build.json``          the URDF -> MJCF recipe. Recorded, never graded.
console text            ``stdout.log`` / ``stderr.log``. MuJoCo writes no log
                        file, so this is the only channel carrying its
                        compiler errors and its ``mju_warning`` text.
======================  ====================================================
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

CANDIDATES = {
    "trajectory": ("trajectory.json", "traj.json"),
    "roster": ("roster.json", "bodies.json"),
    "contacts": ("contacts.json",),
    "completion": ("completion.json", "recorder.json"),
    "process": ("process.json", "run.json"),
    "build": ("build.json",),
    # T2's own channels. Absent on a T1 run and on any column that has no
    # ladder-side T2 sampler, which is the honest reading: the file's absence
    # makes every T2 channel fall back and the gap is reported as OURS.
    "t2": ("t2.json",),
    # T3's own channels. Absent on a T1/T2 run and on any column that has no
    # ladder-side T3 sampler -- same reading as ``t2`` above: the file's
    # absence makes every T3 channel fall back and the gap is reported as OURS.
    "t3": ("t3.json",),
    # T4's. Its CHANNELS are T3's -- the tier replaces an assertion, not an
    # evidence contract -- but the file is its own, because the support channel
    # is recorded twice on this rung (the attested total, and the xfrc-only
    # reading that demonstrates what a column which ignored constraint forces
    # would have published).
    "t4": ("t4.json",),
}

CONSOLE_CANDIDATES = ("stdout.log", "stderr.log", "console.log", "run.log")

# The engine-authored compile witness, in preference order.
SAVED_MODEL_CANDIDATES = ("model_saved.xml", "model.xml")

SEARCH_SUBDIRS = ("", "out", "mujoco")


class MujocoRun:
    """The artifacts of one MuJoCo run, parsed but not interpreted."""

    def __init__(self, run_dir=None):
        self.run_dir = Path(run_dir) if run_dir else None
        self.trajectory = None
        self.roster = None
        self.contacts = None
        self.completion = None
        self.process = None
        self.build = None
        self.t2 = None                 # T2's channels; None on a T1 run
        self.t3 = None                 # T3's channels; None on a T1/T2 run
        self.t4 = None                 # T4's; None on a T1/T2/T3 run
        self.console_text = None       # None == not captured (never "")
        self.saved_model_path = None
        self.saved_model_bytes = None  # None == none was looked at
        self.scene = None              # the deliverable the run loaded
        self.files = {}
        self.missing = {}
        self.errors = {}

    @property
    def any_evidence(self):
        return bool(self.trajectory or self.roster or self.contacts
                    or self.completion or self.process
                    or self.console_text is not None)

    @property
    def saved_model_nonempty(self):
        """True/False once a saved model was located; None when none was."""
        if self.saved_model_bytes is None:
            return None
        return self.saved_model_bytes > 0

    def as_dict(self):
        traj = self.trajectory or {}
        bodies = traj.get("bodies") or []
        return {
            "run_dir": str(self.run_dir) if self.run_dir else None,
            "scene": str(self.scene) if self.scene else None,
            "files": {k: str(v) for k, v in self.files.items()},
            "missing": dict(self.missing),
            "parse_errors": dict(self.errors),
            "trajectory": ({
                "n_bodies": len(bodies),
                "n_samples": (len(bodies[0].get("t") or []) if bodies else 0),
                "dt_s": traj.get("dt_s"),
                "physics_timestep_s": traj.get("physics_timestep_s"),
                "record_stride_steps": traj.get("record_stride_steps"),
            } if self.trajectory else None),
            "roster_bodies": (len((self.roster or {}).get("bodies") or [])
                              if self.roster else None),
            "contacts": ({k: (self.contacts or {}).get(k)
                          for k in ("supported", "steps", "total_observed",
                                    "distinct_named", "emit_stride_steps",
                                    "records_truncated", "error")}
                         if self.contacts else None),
            "completion": {k: v for k, v in (self.completion or {}).items()
                           if k != "robot_subtree_bodies"} or None,
            "process": self.process,
            "build": ({"steps": [s.get("step") for s in
                                 (self.build.get("steps") or [])],
                       "problems": self.build.get("problems"),
                       "mass_kg_declared_by_urdf":
                           self.build.get("mass_kg_declared_by_urdf"),
                       "mass_kg_in_the_compiled_model":
                           self.build.get("mass_kg_in_the_compiled_model")}
                      if self.build else None),
            "t2": ({"samples": len(self.t2.get("t_s") or []),
                    "object": (self.t2.get("object") or {}).get("body"),
                    "end_effector": (self.t2.get("end_effector")
                                     or {}).get("body"),
                    "container": (self.t2.get("container") or {}).get("body"),
                    "support_surfaces": len(self.t2.get("support_surfaces")
                                            or []),
                    "grip_mechanism": (self.t2.get("grip") or {}
                                       ).get("mechanism"),
                    "driver_error": (self.t2.get("driver") or {}).get("error")}
                   if self.t2 else None),
            "t3": ({"samples": len(self.t3.get("t_s") or []),
                    "base": (self.t3.get("base_pose") or {}).get("body"),
                    "standing_z_m": (self.t3.get("standing")
                                     or {}).get("z_m"),
                    "gait_contacts": len((self.t3.get("gait") or {}
                                          ).get("contacts") or []),
                    "gait_queries": len((self.t3.get("gait") or {}
                                         ).get("sample_times") or []),
                    "support_attested": (self.t3.get("support")
                                         or {}).get("attested"),
                    "controller_loaded": (self.t3.get("controller")
                                          or {}).get("loaded"),
                    "driver_error": (self.t3.get("driver") or {}).get("error")}
                   if self.t3 else None),
            "t4": ({"samples": len(self.t4.get("t_s") or []),
                    "base": (self.t4.get("base_pose") or {}).get("body"),
                    "standing_z_m": (self.t4.get("standing")
                                     or {}).get("z_m"),
                    "gait_contacts": len((self.t4.get("gait") or {}
                                          ).get("contacts") or []),
                    "gait_queries": len((self.t4.get("gait") or {}
                                         ).get("sample_times") or []),
                    "support_attested": (self.t4.get("support")
                                         or {}).get("attested"),
                    # The one field that is T4's alone: whether anything held
                    # this robot that mjData.xfrc_applied could not see.
                    "equality_reaction_live_samples":
                        (self.t4.get("support") or {}
                         ).get("equality_reaction_was_live_in_samples"),
                    "neq": (self.t4.get("constraint_rig") or {}).get("neq"),
                    "controller_loaded": (self.t4.get("controller")
                                          or {}).get("loaded"),
                    "driver_error": (self.t4.get("driver") or {}).get("error")}
                   if self.t4 else None),
            "console_chars": (None if self.console_text is None
                              else len(self.console_text)),
            "saved_model": (str(self.saved_model_path)
                            if self.saved_model_path else None),
            "saved_model_bytes": self.saved_model_bytes,
        }


def read_run(run_dir=None, **overrides):
    """Read one MuJoCo run directory. **Never raises.**

    Every logical artifact may be overridden by keyword with an explicit path
    (``trajectory=``, ``roster=``, ``contacts=``, ``completion=``,
    ``process=``, ``build=``, ``console=``, ``saved_model=``).
    """
    run = MujocoRun(run_dir)
    for name, cands in CANDIDATES.items():
        path = _pick(run_dir, cands, overrides.get(name))
        if path is None:
            run.missing[name] = ("not found (tried: %s)" % ", ".join(cands)
                                 if run_dir else
                                 "no run_dir and no explicit path given")
            continue
        doc, err = _read_json(path)
        run.files[name] = path
        if err:
            run.errors[name] = err
        else:
            setattr(run, name, doc)

    console_paths = []
    console = overrides.get("console")
    if console is not None:
        for c in ([console] if isinstance(console, (str, Path))
                  else list(console)):
            p = Path(c)
            if p.is_file():
                console_paths.append(p)
    else:
        for d in _search_dirs(run_dir):
            for nm in CONSOLE_CANDIDATES:
                p = d / nm
                if p.is_file():
                    console_paths.append(p)
    if console_paths:
        chunks = []
        for p in console_paths:
            try:
                chunks.append(p.read_text(encoding="utf-8", errors="replace"))
            except OSError as exc:
                run.errors["console"] = repr(exc)
        run.console_text = "\n".join(chunks)
        run.files["console"] = (console_paths[0] if len(console_paths) == 1
                                else [str(p) for p in console_paths])
    else:
        run.missing["console"] = (
            "no console capture: MuJoCo writes NO log file, so with nothing "
            "captured from the runner's stdout/stderr its compiler errors and "
            "mju_warning text are unavailable for this run")

    saved = _pick(run_dir, SAVED_MODEL_CANDIDATES, overrides.get("saved_model"))
    if saved is None and run.completion:
        cand = run.completion.get("saved_xml")
        if cand and Path(cand).is_file():
            saved = Path(cand)
    if saved is not None:
        run.saved_model_path = Path(saved)
        try:
            run.saved_model_bytes = int(run.saved_model_path.stat().st_size)
        except OSError as exc:
            run.errors["saved_model"] = repr(exc)
        run.files["saved_model"] = run.saved_model_path
    else:
        run.missing["saved_model"] = (
            "no mj_saveLastXML output located, so the one ENGINE-authored "
            "witness that a model compiled is absent and 'did it finalize' "
            "rests on the driver's own record alone")

    for doc in (run.completion, run.build):
        s = (doc or {}).get("scene")
        if s:
            run.scene = s
            break
    return run


def _search_dirs(run_dir):
    if run_dir is None:
        return []
    base = Path(run_dir)
    return [base / s if s else base for s in SEARCH_SUBDIRS]


def _pick(run_dir, candidates, override=None):
    if override is not None:
        p = Path(override)
        return p if p.is_file() else None
    for d in _search_dirs(run_dir):
        for nm in candidates:
            p = d / nm
            if p.is_file():
                return p
    return None


def _read_json(path):
    try:
        doc = json.loads(Path(path).read_text(encoding="utf-8",
                                              errors="replace"))
    except OSError as exc:
        return None, "unreadable: %r" % (exc,)
    except ValueError as exc:
        return None, "not valid JSON: %r" % (exc,)
    if not isinstance(doc, dict):
        return None, "expected a JSON object, got %s" % type(doc).__name__
    return doc, None


# --- trajectory.json -> arrays ----------------------------------------------


class Arrays:
    """``trajectory.json`` as numpy, plus every problem found reading it."""

    def __init__(self):
        self.names = []
        self.ids = []
        self.t = None                # (N,) simulated seconds
        self.xyz = None              # (n_bodies, N, 3) metres
        self.dt_s = None
        self.recorded_s = None
        self.complete = None
        self.physics_dt_s = None
        self.stride = None
        self.problems = []
        self.fatal = None


def to_arrays(doc):
    """Parse ``trajectory.json`` into ``(n_bodies, N, 3)``. Never raises.

    Same tolerances as the Webots reader and for the same reasons: ragged
    sample counts are **truncated to the shortest common length and the
    truncation published**, never padded or interpolated (``REQUIRED_EVIDENCE``
    says "not resampled" outright), and a disagreeing time base is reported
    rather than reconciled. Non-finite samples are left exactly as they are --
    a NaN is a physical result and the core's floors will see it.
    """
    a = Arrays()
    if not isinstance(doc, dict):
        a.fatal = "trajectory document is not a JSON object"
        return a
    bodies = doc.get("bodies")
    if not isinstance(bodies, list) or not bodies:
        a.fatal = "trajectory document carries no 'bodies' list"
        return a

    series, lengths = [], []
    for i, b in enumerate(bodies):
        if not isinstance(b, dict):
            a.problems.append("body %d is not an object; dropped" % i)
            continue
        name = str(b.get("name") or "")
        t = _floats(b.get("t"))
        xyz = _xyz(b.get("xyz"))
        if t is None or xyz is None:
            a.problems.append("body %r has no usable t/xyz; dropped"
                              % (name or i))
            continue
        if len(t) != xyz.shape[0]:
            n = min(len(t), xyz.shape[0])
            a.problems.append("body %r: t has %d samples, xyz has %d; "
                              "truncated to %d"
                              % (name or i, len(t), xyz.shape[0], n))
            t, xyz = t[:n], xyz[:n]
        series.append((name, b.get("id"), t, xyz))
        lengths.append(len(t))

    if not series:
        a.fatal = "no body in the trajectory carried a usable t/xyz pair"
        return a
    n = min(lengths)
    if n < 1:
        a.fatal = "the trajectory carries zero samples"
        return a
    if max(lengths) != n:
        a.problems.append("bodies disagree on the sample count (%d..%d); "
                          "truncated to the shortest, %d"
                          % (n, max(lengths), n))

    a.names = [s[0] for s in series]
    a.ids = [s[1] for s in series]
    a.t = series[0][2][:n]
    a.xyz = np.stack([s[3][:n] for s in series], axis=0)
    for name, _i, t, _ in series[1:]:
        if not np.allclose(t[:n], a.t, atol=1e-9, rtol=0.0):
            a.problems.append(
                "body %r does not share the first body's time base; the "
                "bundle carries ONE t vector and it is the first body's"
                % (name or "?"))
            break

    dt = doc.get("dt_s")
    a.dt_s = float(dt) if _is_num(dt) else (
        float(np.median(np.diff(a.t))) if n >= 2 else None)
    rec = doc.get("recorded_s")
    a.recorded_s = (float(rec) if _is_num(rec)
                    else float(a.t[-1] - a.t[0]) if n >= 1 else None)
    if "complete" in doc:
        a.complete = bool(doc.get("complete"))
    a.physics_dt_s = (float(doc["physics_timestep_s"])
                      if _is_num(doc.get("physics_timestep_s")) else None)
    a.stride = (int(doc["record_stride_steps"])
                if _is_num(doc.get("record_stride_steps")) else None)
    if n >= 2 and not np.all(np.diff(a.t) > 0):
        a.problems.append("the time base is not strictly increasing; the "
                          "neutral schema asks for monotone t and this is not")
    return a


def _is_num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _floats(seq):
    if not isinstance(seq, (list, tuple)) or not seq:
        return None
    try:
        return np.asarray([float(v) for v in seq], dtype=float)
    except (TypeError, ValueError):
        return None


def _xyz(seq):
    if not isinstance(seq, (list, tuple)) or not seq:
        return None
    try:
        arr = np.asarray(seq, dtype=float)
    except (TypeError, ValueError):
        return None
    if arr.ndim != 2 or arr.shape[1] < 3:
        return None
    return arr[:, :3]


def triple(seq):
    """``seq`` as a 3-tuple of floats, or None."""
    if not isinstance(seq, (list, tuple)) or len(seq) < 3:
        return None
    try:
        return (float(seq[0]), float(seq[1]), float(seq[2]))
    except (TypeError, ValueError):
        return None


__all__ = ["Arrays", "CANDIDATES", "MujocoRun", "read_run", "to_arrays",
           "triple"]
