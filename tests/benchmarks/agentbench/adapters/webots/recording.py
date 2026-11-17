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

"""Reading one upstream-Webots run off disk. **No neutral schema in here.**

This module is the *file* half of the Webots adapter: it finds and parses the
artifacts an upstream run leaves behind and hands them to
:mod:`agentbench.adapters.webots.evidence`, which does the mapping into
:class:`~agentbench.graders.evidence.EvidenceBundle`. Nothing here imports the
neutral schema, and nothing here raises -- a broken simulator is a measurement
(adapter rule 1).

The artifact set, and why it is shaped like this
-----------------------------------------------

Upstream Webots R2025a gives a benchmark *much* less than OmniSim does
(``docs/developer/webots-control-baseline.md`` §3, §6): **no** ``--duration``,
**no** auto-exit, **no** HTTP surface, **no** scene-tree query, and -- the one
that shapes this whole module -- **no log file at all**. So every fact below has
to come from either (a) an in-world ``Supervisor`` recorder that the grader owns,
or (b) the process the grader launched.

===================  =======================================================
file                 what it must contain
===================  =======================================================
``trajectory.json``  ``{"dt_s": float, "bodies": [{"name", "t": [...],
                     "xyz": [[x, y, z], ...]}, ...]}`` -- simulated seconds and
                     metres, sampled by the recorder once per basic timestep.
                     THE primary input; everything else is optional. Each body
                     may also carry ``"kind"`` (``robot`` / ``link`` /
                     ``solid``) and ``"parent"`` (the owning body's key, for a
                     link). Both are OPTIONAL and additive: a document without
                     them parses exactly as it did before they existed, and the
                     bundle then reports the rows as unclassified rather than
                     guessing that they are robots.
``roster.json``      ``{"t_s", "frozen", "synchronization", "bodies": [...]}``
                     -- one entry per body with ``name``, ``def``/``id``,
                     ``type`` (``getTypeName()``), ``base_type``
                     (``getBaseTypeName()``), ``n_joints``, ``controller``,
                     ``has_physics`` and either ``aabb_min``/``aabb_max`` or
                     ``bounds: {bbox_min, bbox_max}``. It may also carry
                     ``links`` / ``named_solids`` -- the opt-in extra track
                     kinds -- which are deliberately NOT folded into
                     ``bodies``: that list becomes the roster the cores count
                     robots in, and a link is not a top-level body.
``contacts.json``    ``{"supported", "steps", "window_s", "total_observed",
                     "distinct_named", "pairs": [...]}``. **Omit the file
                     entirely if the recorder did not run the contact scan** --
                     an absent file becomes ``contacts=None``, which the core
                     reports as a vacuous clause. A file claiming
                     ``supported: true`` with no witness counters is worse than
                     no file.
``completion.json``  ``{"complete", "quit_called", "recorded_s", "steps",
                     "dt_s"}`` -- the recorder's own attestation that it
                     reached the requested simulated time and called
                     ``simulationQuit()``. On upstream this is the ONLY clean
                     way a headless run ends, so it is also the only
                     non-circular evidence that the world finalized.
``process.json``     ``{"exit_code", "timed_out", "wall_s", "attempts_used",
                     "perf_log", "webots_version", "command"}``.
console text         ``console.log`` / ``stdout.log`` / ``stderr.log`` -- what
                     ``--stdout --stderr`` produced. Upstream writes no log
                     file, so this is the *only* channel carrying the engine's
                     own ``ERROR:`` lines and its ``Starting controller:``
                     lines.
perf log             ``--log-performance`` output. Its **non-emptiness** is
                     independent evidence of a clean world close: upstream
                     flushes it only in ``worldClosed()``
                     (``WbPerformanceLog.cpp:123-150``), so a killed run leaves
                     it empty.
===================  =======================================================

Every one of these is optional and every one of them may be malformed. What is
found, what is missing and what failed to parse is recorded on the
:class:`WebotsRun` and published in the verdict's ``adapter_measurements`` --
"the file was not there" is a finding, not a crash.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

# Logical artifact -> candidate filenames, first existing wins. Several spellings
# are accepted because the Webots arm of the comparison is a separate lane and a
# grader that only accepts one filename is a grader that reports "no evidence"
# for a run that produced plenty.
CANDIDATES = {
    "trajectory": ("trajectory.json", "traj.json"),
    "roster": ("roster.json", "bodies.json"),
    "contacts": ("contacts.json",),
    "completion": ("completion.json", "recorder.json", "meta.json"),
    "process": ("process.json", "run.json"),
}

# Console captures are CONCATENATED, not first-wins: stdout and stderr are two
# files in the obvious layout and the engine's ERROR: lines can be on either.
CONSOLE_CANDIDATES = ("console.log", "console.txt", "stdout.log", "stderr.log",
                      "webots_stdout.log", "webots_stderr.log", "run.log")

PERF_CANDIDATES = ("perf.txt", "perf_log.txt")

# Subdirectories of ``run_dir`` that are also searched for all of the above.
# The Webots lane keeps its console and perf output in ``out/`` while the
# trajectory sits at the top, and a reader that insists on one flat directory
# reports "no evidence" for a run that produced plenty. Which path was actually
# read is always published in ``WebotsRun.files``.
SEARCH_SUBDIRS = ("", "out")


class WebotsRun:
    """The artifacts of one upstream-Webots run, parsed but not interpreted."""

    def __init__(self, run_dir=None):
        self.run_dir = Path(run_dir) if run_dir else None
        self.world = None               # the .wbt the run loaded, if recorded
        self.trajectory = None          # parsed JSON docs, or None
        self.roster = None
        self.contacts = None
        self.completion = None
        self.process = None
        self.console_text = None        # None == not captured (never "")
        self.perf_path = None
        self.perf_bytes = None          # None == no perf log was looked at
        self.files = {}                 # logical name -> path actually read
        self.missing = {}               # logical name -> why it is not here
        self.errors = {}                # logical name -> parse failure

    # -- derived -----------------------------------------------------------
    @property
    def any_evidence(self):
        """Did this run leave ANY artifact at all?

        Distinguishes "upstream ran and we read it" from "the adapter was
        pointed at a directory that has nothing to do with a Webots run", which
        is the difference between a gradeable row and an unattributable one.
        """
        return bool(self.trajectory or self.roster or self.contacts
                    or self.completion or self.process
                    or self.console_text is not None)

    @property
    def perf_nonempty(self):
        """True/False when a perf log was located; None when none was."""
        if self.perf_bytes is None:
            return None
        return self.perf_bytes > 0

    def as_dict(self):
        """A compact, JSON-safe record for the verdict. No arrays."""
        traj = self.trajectory or {}
        bodies = traj.get("bodies") or []
        return {
            "run_dir": str(self.run_dir) if self.run_dir else None,
            "world": str(self.world) if self.world else None,
            "files": {k: str(v) for k, v in self.files.items()},
            "missing": dict(self.missing),
            "parse_errors": dict(self.errors),
            "trajectory": {
                "n_bodies": len(bodies),
                "n_samples": (len(bodies[0].get("t") or []) if bodies else 0),
                "dt_s": traj.get("dt_s"),
            } if self.trajectory else None,
            "roster_bodies": (len((self.roster or {}).get("bodies") or [])
                              if self.roster else None),
            "contacts": ({k: (self.contacts or {}).get(k)
                          for k in ("supported", "steps", "total_observed",
                                    "distinct_named", "error")}
                         if self.contacts else None),
            "completion": self.completion,
            "process": self.process,
            "console_chars": (None if self.console_text is None
                              else len(self.console_text)),
            "perf_log": str(self.perf_path) if self.perf_path else None,
            "perf_bytes": self.perf_bytes,
        }


def read_run(run_dir=None, *, trajectory=None, roster=None, contacts=None,
             completion=None, process=None, console=None, perf=None):
    """Read one Webots run. **Never raises.**

    ``run_dir`` is where the Webots lane wrote its artifacts; every other
    argument overrides one file's path (an absolute path to a ``trajectory.json``
    living somewhere else entirely is the common case while the two lanes are
    still separate).
    """
    run = WebotsRun(run_dir)
    overrides = {"trajectory": trajectory, "roster": roster,
                 "contacts": contacts, "completion": completion,
                 "process": process}

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

    # -- console: concatenate every capture we can find --------------------
    console_paths = []
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
        run.files["console"] = console_paths[0] if len(console_paths) == 1 \
            else [str(p) for p in console_paths]
    else:
        run.missing["console"] = (
            "no console capture: upstream Webots writes NO log file, so with "
            "nothing captured from --stdout/--stderr the engine's own error "
            "stream is unavailable for this run")

    # -- perf log: presence AND size, because size is the finalize witness --
    perf_path = perf
    if perf_path is None and run.process:
        perf_path = run.process.get("perf_log")
    perf_path = _pick(run_dir, PERF_CANDIDATES, perf_path)
    ambiguous = None
    if perf_path is None and perf is None:
        for d in _search_dirs(run_dir):
            globbed = sorted(d.glob("perf*.txt"))
            if len(globbed) == 1:
                perf_path = globbed[0]
                break
            if len(globbed) > 1:
                # REFUSE TO GUESS. The perf log's non-emptiness is a finalize
                # witness, so picking one of several runs' logs would attest that
                # THIS world closed cleanly using a DIFFERENT run's evidence --
                # precisely the cross-contamination that voids a comparison.
                ambiguous = ("%d perf logs in %s (%s); none chosen -- pass the "
                             "right one as perf=<path>, because a finalize "
                             "witness taken from another run is worse than none"
                             % (len(globbed), d.name or ".",
                                ", ".join(p.name for p in globbed[:4])))
                break
    if perf_path is not None:
        run.perf_path = Path(perf_path)
        try:
            run.perf_bytes = int(run.perf_path.stat().st_size)
        except OSError as exc:
            run.errors["perf"] = repr(exc)
        run.files["perf"] = run.perf_path
    else:
        run.missing["perf"] = (ambiguous
                               or "no --log-performance output located")

    # -- which world was it? -----------------------------------------------
    for doc in (run.completion, run.process, run.roster, run.trajectory):
        w = (doc or {}).get("world")
        if w:
            run.world = w
            break
    return run


def _search_dirs(run_dir):
    """``run_dir`` and the subdirectories worth looking in, in priority order."""
    if run_dir is None:
        return []
    base = Path(run_dir)
    return [base / s if s else base for s in SEARCH_SUBDIRS]


def _pick(run_dir, candidates, override=None):
    """The first readable path: an explicit override, else a candidate name."""
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
        self.t = None                # (N,) simulated seconds
        self.xyz = None              # (n_bodies, N, 3) metres
        self.dt_s = None
        self.recorded_s = None
        self.complete = None
        self.problems = []           # human-readable, published in the row
        self.fatal = None            # set => no usable arrays at all
        # Parallel to ``names``, one entry per surviving row. Empty strings /
        # None where the recorder said nothing -- never a guessed "robot",
        # because an unclassified row must stay unclassified (adapter rule 2).
        self.kinds = []
        self.parents = []


def to_arrays(doc):
    """Parse ``trajectory.json`` into ``(n_bodies, N, 3)``. Never raises.

    Tolerances that are deliberate rather than lazy:

    * **ragged sample counts** -- the schema gives every body its own ``t``, so
      a recorder that lost a body's last sample would otherwise make the whole
      run unreadable. Rows are truncated to the shortest common length and the
      truncation is published. Nothing is interpolated or padded: a resampled
      trajectory is not a measurement (adapter rule 3, and ``REQUIRED_EVIDENCE``
      says "not resampled" outright).
    * **disagreeing time bases** -- if the bodies' ``t`` vectors are not the
      same clock, the first body's is used and the disagreement is published.
      One shared ``t`` is what the neutral schema has.

    Non-finite samples are left exactly as they are. A NaN is a physical
    result on this arm (``webots-control-baseline.md`` §5.3: a ``rotation``
    axis-angle of exactly +-pi reproducibly sent a Moose non-finite inside ten
    steps) and the core's floors will see it.
    """
    a = Arrays()
    if not isinstance(doc, dict):
        a.fatal = "trajectory document is not a JSON object"
        return a
    bodies = doc.get("bodies")
    if not isinstance(bodies, list) or not bodies:
        a.fatal = "trajectory document carries no 'bodies' list"
        return a

    series = []
    lengths = []
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
                              "truncated to %d" % (name or i, len(t),
                                                   xyz.shape[0], n))
            t, xyz = t[:n], xyz[:n]
        kind = b.get("kind")
        kind = str(kind) if isinstance(kind, str) else ""
        parent = b.get("parent")
        parent = str(parent) if isinstance(parent, str) and parent else None
        series.append((name, t, xyz, kind, parent))
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
    a.t = series[0][1][:n]
    a.xyz = np.stack([s[2][:n] for s in series], axis=0)
    # Only publish the descriptive vectors when the recorder said SOMETHING;
    # a list of empty strings would look like an answer ("classified: nothing
    # is a link") when it is the absence of one.
    if any(s[3] for s in series):
        a.kinds = [s[3] for s in series]
        a.parents = [s[4] for s in series]

    for name, t in ((s[0], s[1]) for s in series[1:]):
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
    out = []
    for row in seq:
        if not isinstance(row, (list, tuple)) or len(row) < 3:
            return None
        try:
            out.append([float(row[0]), float(row[1]), float(row[2])])
        except (TypeError, ValueError):
            return None
    return np.asarray(out, dtype=float)


def triple(seq):
    """``seq`` as a 3-tuple of floats, or None. Used for positions and AABBs."""
    if not isinstance(seq, (list, tuple)) or len(seq) < 3:
        return None
    try:
        return (float(seq[0]), float(seq[1]), float(seq[2]))
    except (TypeError, ValueError):
        return None


__all__ = ["Arrays", "CANDIDATES", "WebotsRun", "read_run", "to_arrays",
           "triple"]
