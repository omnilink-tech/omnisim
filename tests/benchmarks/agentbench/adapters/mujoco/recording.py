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

"""Reading one MuJoCo run off disk. **No neutral schema, and no mujoco, here.**

The file half of this arm, split from the mapping half exactly as the Webots
adapter is split: this module finds and parses the artifacts
:mod:`agentbench.adapters.mujoco.recorder` wrote and hands them to
:mod:`agentbench.adapters.mujoco.evidence`. It imports neither ``mujoco`` nor
the neutral dataclasses, so the mapping is testable from fixtures on a machine
with no simulator of any kind, and nothing here raises -- a broken run is a
measurement.

The artifact set
----------------

=====================  =====================================================
file                   what it carries
=====================  =====================================================
``trajectory.json``    the pose series HEADER: ``{csv, n_samples, dt_s,
                       model_timestep_s, recorded_s, complete, bodies:
                       [{key, name, kind, parent}], bodies_truncated}``. The
                       samples themselves are in the CSV named by ``csv`` --
                       a 60 s run at MuJoCo's 2 ms default is 30,000 rows and
                       a JSON array of them is tens of megabytes of numbers
                       nobody reads.
``trajectory.csv``     ``t`` then three columns per tracked body, in the order
                       ``bodies`` lists them. Simulated seconds, metres.
``roster.json``        the FROZEN t=0 scan: ``bodies`` (every MuJoCo body,
                       with its world AABB, joint count, mass and movability),
                       ``world_geoms`` (every geom hanging directly off
                       ``<worldbody>``, which is how MJCF writes static
                       scenery), ``robot_roots`` / ``actuated_roots`` /
                       ``forced_roots`` (the robot predicate and its evidence),
                       ``plane_notes``, ``frozen``, ``t_s``.
``contacts.json``      ``{supported, steps, stride, window_s, total_observed,
                       distinct_named, pairs: [...], pairs_truncated}``. The
                       pairs are DEDUPED by participant pair with their first
                       witness; the two counters are totals over every scanned
                       step and are what tells an empty result apart from a
                       query that never names anything.
``completion.json``    ``{complete, quit_called, stopped_by, recorded_s,
                       steps, dt_s, driver, driver_error, hook_intact, tamper,
                       warnings, notes}`` -- the recorder's own attestation.
``model_info.json``    the engine attribution channel: mujoco version, solver,
                       integrator, cone, timestep, gravity, model size.
``model_load.json``    did the agent's MJCF compile, INDEPENDENTLY of whether
                       the driver worked. This arm's "did the world load".
``process.json``       the parent's facts: exit code, timeout, wall time, the
                       interpreter and its ``import mujoco`` probe, the model
                       and driver paths and the rule that found the driver.
``stdout.log`` /       whatever the child printed. MuJoCo writes no log file
``stderr.log``         and has no engine log format at all -- see
                       ``evidence._error_lines`` for what is and is not
                       treated as an error line, and why.
=====================  =====================================================

Every one of them is optional and every one may be malformed; what was found,
what was missing and what failed to parse is recorded on the :class:`MujocoRun`
and published in the row.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np

CANDIDATES = {
    "trajectory": ("trajectory.json",),
    "roster": ("roster.json",),
    "contacts": ("contacts.json",),
    "completion": ("completion.json",),
    "process": ("process.json",),
    "model_info": ("model_info.json",),
    "model_load": ("model_load.json",),
}

CONSOLE_CANDIDATES = ("stdout.log", "stderr.log", "console.log")

SEARCH_SUBDIRS = ("", "out", "mujoco")


class MujocoRun:
    """The artifacts of one MuJoCo run, parsed but not interpreted."""

    def __init__(self, run_dir=None):
        self.run_dir = Path(run_dir) if run_dir else None
        self.model = None
        self.driver = None
        self.trajectory = None
        self.roster = None
        self.contacts = None
        self.completion = None
        self.process = None
        self.model_info = None
        self.model_load = None
        self.console_text = None        # None == not captured (never "")
        self.csv_path = None
        self.files = {}
        self.missing = {}
        self.errors = {}

    @property
    def any_evidence(self):
        """Did this run leave ANY artifact at all?"""
        return bool(self.trajectory or self.roster or self.contacts
                    or self.completion or self.process or self.model_info
                    or self.model_load or self.console_text is not None)

    @property
    def compiled(self):
        """True/False when the model-load probe ran; None when it did not."""
        if not self.model_load:
            return None
        v = self.model_load.get("compiled")
        return None if v is None else bool(v)

    def as_dict(self):
        """A compact, JSON-safe record for the verdict. No arrays."""
        traj = self.trajectory or {}
        return {
            "run_dir": str(self.run_dir) if self.run_dir else None,
            "model": str(self.model) if self.model else None,
            "driver": str(self.driver) if self.driver else None,
            "files": {k: str(v) for k, v in self.files.items()},
            "missing": dict(self.missing),
            "parse_errors": dict(self.errors),
            "trajectory": {
                "n_bodies": len(traj.get("bodies") or []),
                "n_samples": traj.get("n_samples"),
                "dt_s": traj.get("dt_s"),
                "model_timestep_s": traj.get("model_timestep_s"),
                "bodies_truncated": traj.get("bodies_truncated"),
            } if self.trajectory else None,
            "roster_bodies": (len((self.roster or {}).get("bodies") or [])
                              if self.roster else None),
            "roster_world_geoms": (
                len((self.roster or {}).get("world_geoms") or [])
                if self.roster else None),
            "contacts": ({k: (self.contacts or {}).get(k)
                          for k in ("supported", "steps", "stride",
                                    "total_observed", "distinct_named",
                                    "pairs_truncated", "error")}
                         if self.contacts else None),
            "completion": self.completion,
            "process": _process_summary(self.process),
            "model_info": self.model_info,
            "model_load": self.model_load,
            "console_chars": (None if self.console_text is None
                              else len(self.console_text)),
        }


def _process_summary(doc):
    """``process.json`` without the interpreter probe's file paths."""
    if not doc:
        return None
    out = dict(doc)
    probe = out.get("interpreter_probe")
    if isinstance(probe, dict):
        out["interpreter_probe"] = {k: probe.get(k) for k in
                                    ("ok", "python", "mujoco_version",
                                     "error")}
    return out


def read_run(run_dir=None, **overrides):
    """Read one MuJoCo run. **Never raises.**

    ``run_dir`` is where the launcher wrote its artifacts; any logical name may
    be overridden with an explicit path (``trajectory=``, ``roster=``, ...).
    """
    run = MujocoRun(run_dir)

    for name, cands in CANDIDATES.items():
        path = _pick(run_dir, cands, overrides.get(name))
        if path is None:
            run.missing[name] = ("not found (tried: %s)" % ", ".join(cands)
                                 if run_dir else "no run_dir and no explicit "
                                                 "path given")
            continue
        doc, err = _read_json(path)
        run.files[name] = path
        if err:
            run.errors[name] = err
        else:
            setattr(run, name, doc)

    # -- console: concatenate every capture there is ----------------------
    console_paths = []
    con = overrides.get("console")
    if con is not None:
        for c in ([con] if isinstance(con, (str, Path)) else list(con)):
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
            "no console capture: MuJoCo writes no log file of its own, so with "
            "nothing captured from the child's stdout/stderr there is no "
            "record of what it printed")

    # -- the pose-series CSV named by the header --------------------------
    name = (run.trajectory or {}).get("csv")
    if name:
        p = _pick(run_dir, (str(name),), overrides.get("trajectory_csv"))
        if p is None:
            run.missing["trajectory_csv"] = (
                "trajectory.json names %r but that file is not here" % name)
        else:
            run.csv_path = p
            run.files["trajectory_csv"] = p

    for doc in (run.process, run.model_load, run.model_info, run.roster):
        m = (doc or {}).get("model") or (doc or {}).get("model_path") \
            or (doc or {}).get("source_model")
        if m:
            run.model = m
            break
    run.driver = (run.process or {}).get("driver") \
        or (run.completion or {}).get("driver")
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


# --- trajectory.csv -> arrays -------------------------------------------------


class Arrays:
    """The pose series as numpy, plus every problem found reading it."""

    def __init__(self):
        self.names = []
        self.keys = []
        self.kinds = []
        self.parents = []
        self.t = None                # (N,) simulated seconds
        self.xyz = None              # (n_bodies, N, 3) metres
        self.dt_s = None
        self.recorded_s = None
        self.complete = None
        self.problems = []
        self.fatal = None


def to_arrays(doc, csv_path):
    """``trajectory.json`` + its CSV -> ``(n_bodies, N, 3)``. Never raises.

    The header's ``bodies`` list is the column map: three CSV columns per
    entry, in order, after the leading ``t``. A CSV whose width disagrees with
    the map is truncated to whichever is shorter and the disagreement is
    published -- a silently dropped row would shift every index after it, and
    a core that resolves a body by name and then indexes the array would
    measure the wrong body.

    Nothing is interpolated, padded or resampled: a resampled trajectory is not
    a measurement, and ``REQUIRED_EVIDENCE`` says "not resampled" outright.
    Non-finite samples are left exactly as they are -- a NaN is a physical
    result (MuJoCo's ``mjWARN_BADQACC`` resets the state when the solve
    diverges) and the core's floors will see it.
    """
    a = Arrays()
    if not isinstance(doc, dict):
        a.fatal = "trajectory header is not a JSON object"
        return a
    bodies = doc.get("bodies")
    if not isinstance(bodies, list) or not bodies:
        a.fatal = "the trajectory header carries no 'bodies' column map"
        return a
    if csv_path is None:
        a.fatal = "the trajectory header is here but its CSV is not"
        return a
    try:
        with open(str(csv_path), newline="", encoding="utf-8") as fh:
            rows = list(csv.reader(fh))
    except OSError as exc:
        a.fatal = "the trajectory CSV is unreadable: %r" % (exc,)
        return a
    if len(rows) < 2:
        a.fatal = "the trajectory CSV carries no samples"
        return a

    data = []
    for i, row in enumerate(rows[1:], 1):
        try:
            data.append([float(v) for v in row])
        except (TypeError, ValueError):
            a.problems.append("CSV row %d is not numeric; dropped" % i)
    if not data:
        a.fatal = "no numeric sample rows in the trajectory CSV"
        return a
    width = min(len(r) for r in data)
    if max(len(r) for r in data) != width:
        a.problems.append("CSV rows disagree on width (%d..%d); truncated to "
                          "the narrowest" % (width, max(len(r) for r in data)))
    arr = np.asarray([r[:width] for r in data], dtype=float)

    n_from_csv = (width - 1) // 3
    n = min(n_from_csv, len(bodies))
    if n_from_csv != len(bodies):
        a.problems.append(
            "the column map lists %d bodies but the CSV carries %d; using the "
            "first %d, so no row is silently re-labelled"
            % (len(bodies), n_from_csv, n))
    if n < 1:
        a.fatal = "the trajectory CSV has no body columns"
        return a

    a.t = arr[:, 0]
    a.xyz = np.empty((n, arr.shape[0], 3), dtype=float)
    for i in range(n):
        a.xyz[i] = arr[:, 1 + 3 * i:4 + 3 * i]
    for b in bodies[:n]:
        b = b if isinstance(b, dict) else {}
        a.names.append(str(b.get("name") or ""))
        a.keys.append(str(b.get("key") or ""))
        a.kinds.append(str(b.get("kind") or ""))
        p = b.get("parent")
        a.parents.append(str(p) if isinstance(p, str) and p else None)
    if not any(a.kinds):
        a.kinds, a.parents = [], []

    dt = doc.get("dt_s")
    a.dt_s = float(dt) if _is_num(dt) else (
        float(np.median(np.diff(a.t))) if len(a.t) >= 2 else None)
    rec = doc.get("recorded_s")
    a.recorded_s = (float(rec) if _is_num(rec)
                    else float(a.t[-1] - a.t[0]) if len(a.t) else None)
    if "complete" in doc:
        a.complete = bool(doc.get("complete"))
    if len(a.t) >= 2 and not np.all(np.diff(a.t) > 0):
        a.problems.append("the time base is not strictly increasing; the "
                          "neutral schema asks for monotone t and this is not")
    return a


def _is_num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def triple(seq):
    """``seq`` as a 3-tuple of floats, or None. Positions and AABB corners."""
    if not isinstance(seq, (list, tuple)) or len(seq) < 3:
        return None
    try:
        return (float(seq[0]), float(seq[1]), float(seq[2]))
    except (TypeError, ValueError):
        return None


__all__ = ["Arrays", "CANDIDATES", "MujocoRun", "read_run", "to_arrays",
           "triple"]
