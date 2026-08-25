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

"""OmniSim adapter: run an agent's world STANDALONE with the grader's recorder.

This is the ground-truth reader for AgentBench on OmniSim. It:

  1. writes a sibling copy of the agent's world with the
     ``agentbench_recorder`` Robot appended -- **a sibling, in the agent's own
     directory**, because ``URDFRobot { url ... }`` is resolved relative to the
     world file (``WbUrdfImporter::expandUrdfRobotBlocks``), so relocating the
     world silently breaks every URDF reference;
  2. launches ``omnisim-bin`` headless through omnibench's shared
     ``engine_launch`` (same argv shape, same env hygiene, same tree-kill);
  3. reads back the recorder CSV, the recorder meta, the t=0 phase-A scan, the
     engine log and the ``<log>.newton.json`` backend-verdict sidecar.

The recorder controller is found via ``WEBOTS_EXTRA_PROJECT_PATH``
(``WbProject::extraProjects``), so it stays inside ``tests/benchmarks/
agentbench/`` instead of being installed into ``projects/default/``.
"""

from __future__ import annotations

import csv
import json
import re
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from agentbench.common.paths import (  # noqa: E402
    AGENTBENCH, REPO, as_wbt_path, engine_launch)
from agentbench.common.worldtext import INJECTED_PREFIX  # noqa: E402

RECORDER_STANZA = """
# --- appended by the AgentBench grader (SPEC 4.5.1) -------------------------
Robot {
  name "agentbench_recorder"
  controller "agentbench_recorder"
  supervisor TRUE
  controllerArgs [
%s
  ]
}
"""


# The engine's own name for the directory that marks a project root.
WORLDS_DIR = "worlds"

# Controllers the GRADER owns. If a world's own project can supply one of
# these, the grader is no longer the one measuring.
GRADER_CONTROLLERS = ("agentbench_recorder", "harness_supervisor")

# Where each of them legitimately lives. A world whose project root IS one of
# these directories is using the real thing, not shadowing it.
CANONICAL_CONTROLLER_DIRS = {
    "agentbench_recorder": AGENTBENCH / "controllers" / "agentbench_recorder",
    "harness_supervisor": (REPO / "projects" / "default" / "controllers"
                           / "harness_supervisor"),
}


def project_root_for_world(world):
    """The project root the ENGINE will resolve for ``world``.

    A transcription of ``WbProject::projectPathFromWorldFile``
    (``src/omnisim/core/WbProject.cpp``): walk UP from the world file's own
    directory for an ancestor literally named ``worlds`` and return its
    parent; when there is none -- which is every AgentBench scratch world --
    fall back to the world's parent's parent.

    This is not a detail. It is what makes the tamper check below possible to
    state exactly: get the root wrong and the check looks in a directory the
    engine will never read.
    """
    world_dir = Path(world).resolve().parent
    cur = world_dir
    while True:
        if cur.name == WORLDS_DIR:
            return cur.parent
        if cur.parent == cur:
            break
        cur = cur.parent
    return world_dir.parent


def controller_shadow_check(world):
    """Grader-owned controllers the world's OWN project would supply.

    ``[]`` means clean. A non-empty list means the run must be refused, not
    graded, because the numbers it would produce are the agent's.

    ``WbRobot::updateControllerDir`` (``src/omnisim/nodes/WbRobot.cpp``)
    searches ``WbProject::current()->controllersPath()`` -- i.e.
    ``<project>/controllers/<name>/`` -- **before** the extra projects that
    ``WEBOTS_EXTRA_PROJECT_PATH`` adds, and it selects on directory EXISTENCE
    alone. So for a scratch world, whose project root resolves to the run
    directory, anything under ``<run_dir>/controllers/agentbench_recorder/``
    wins over the grader's copy:

      * a fake recorder there emits whatever ``phaseB.csv`` and
        ``phase_a.json`` the agent wants and calls ``simulationQuit(0)``,
        which is the entire core-physical basis of A1 (A1.3, A1.5, A1.6,
        A1.7, A1.8) authored by the thing being measured;
      * an EMPTY directory there is enough to win the existence test, so the
        real recorder never loads and the cell scores INVALID -- a broken
        harness -- instead of FAIL.

    Refusing before launch is the enforcing half; scratch-only write roots
    (``runner/isolation.Sandbox.create``) are the other. Both are needed:
    ``run_shell`` cannot be path-confined on a bare host, so the guard alone
    is a speed bump, and this check is what a published run rests on.
    """
    root = project_root_for_world(world)
    hits = []
    for name in GRADER_CONTROLLERS:
        p = root / "controllers" / name
        if not p.exists():
            continue
        canonical = CANONICAL_CONTROLLER_DIRS.get(name)
        try:
            if canonical is not None and p.resolve() == canonical.resolve():
                continue          # the shipped copy, in its own project
        except OSError:
            pass
        try:
            entries = sorted(c.name for c in p.iterdir())
        except OSError:
            entries = None        # unreadable, but it EXISTS, which is what
            #                       the engine's selection turns on
        hits.append({"controller": name, "path": str(p),
                     "project_root": str(root), "entries": entries})
    return hits


def describe_tamper(hits):
    return ("TAMPER: the world's own project (%s) supplies grader-owned "
            "controller(s) %s -- the engine searches <project>/controllers/ "
            "BEFORE WEBOTS_EXTRA_PROJECT_PATH, so the grader's recorder would "
            "not be the thing measuring this run. Refused, not graded."
            % (hits[0]["project_root"],
               ", ".join("%s (%s)" % (h["controller"], h["path"])
                         for h in hits)))


class PhaseBResult:
    """Everything the grader is allowed to believe about a standalone run."""

    def __init__(self, run_dir):
        self.run_dir = Path(run_dir)
        self.world = None
        self.injected_world = None
        self.rc = None
        self.wall_s = None
        self.timed_out = False
        self.log_path = None
        self.log_text = ""
        self.sidecar = {"present": False}
        self.meta = None
        self.phase_a = None
        self.t = None            # (N,) sim seconds
        self.xyz = None          # (n_tracks, N, 3) metres
        self.error = None
        self.attempts_used = 1
        # The env the CHILD actually got. Attribution must read this and not
        # os.environ: engine_launch.build_env POPS OMNISIM_FORCE_ODE /
        # OMNISIM_LEGACY / OMNISIM_REQUIRE_NEWTON out of the child env, so an
        # operator with OMNISIM_FORCE_ODE=1 exported gets a Newton-driven run
        # that the parent's environment would have mis-attributed to ODE.
        self.launch_env = None
        # Non-empty == the run was REFUSED before launch: the world's own
        # project could have shadowed the grader's recorder.
        self.tamper = []

    # -- derived -----------------------------------------------------------
    @property
    def n_tracks(self):
        """Rows in ``xyz`` -- robots, then links, then solids."""
        return 0 if self.xyz is None else int(self.xyz.shape[0])

    @property
    def n_robots(self):
        """Rows that are top-level ROBOT bodies.

        Read from the recorder's track map when there is one, so the name keeps
        meaning what it says now that a run can also carry link and solid rows.
        Falls back to the row count for a recording made before the track map
        existed -- where every row WAS a robot, so the two agree.
        """
        tracks = self.tracks
        if tracks:
            return sum(1 for tr in tracks if tr.get("kind") == "robot")
        return self.n_tracks

    @property
    def roster(self):
        return (self.meta or {}).get("robots", [])

    @property
    def scene_scan(self):
        """The recorder's NAME-FREE t=0 scene scan, or ``None``.

        ``None`` for a recording written before 2026-08-09, which every caller
        must treat as "the scan did not run" -- never as "the scene has no
        non-robot body". The two are the difference between an instrument gap
        and a measurement, and R1.3 spent a day reporting one as the other.
        """
        return (self.phase_a or {}).get("t0_scene")

    @property
    def run_contacts(self):
        """The recorder's RUN-LONG contact watch, or ``None``.

        ``None`` for a recording written before 2026-08-09, which every caller
        must read as "the watch did not run" -- never as "nothing was hit".
        That distinction is the whole point of the channel: R1.5 credited a
        rover that finished OUTSIDE its own walled arena with zero collisions
        because the only contact channel was robot-robot and one sample wide.
        """
        return (self.meta or {}).get("run_contacts")

    @property
    def tracks(self):
        """The recorder's CSV column map, one entry per xyz row, in order.

        ``[]`` for a recording written before 2026-08-09; every caller must
        treat that as "robots only", which is exactly what it was.
        """
        return (self.meta or {}).get("tracks") or []

    def xy(self, i):
        return self.xyz[i][:, :2]

    def z(self, i):
        return self.xyz[i][:, 2]

    def as_dict(self):
        return {
            "world": str(self.world) if self.world else None,
            "injected_world": (str(self.injected_world)
                               if self.injected_world else None),
            "rc": self.rc, "wall_s": self.wall_s, "timed_out": self.timed_out,
            "log_path": str(self.log_path) if self.log_path else None,
            "sidecar": self.sidecar,
            "recorder_complete": bool((self.meta or {}).get("complete")),
            "rows": (self.meta or {}).get("rows"),
            "n_robots": self.n_robots,
            "n_tracks": self.n_tracks,
            "n_link_tracks": (self.meta or {}).get("n_link_tracks"),
            "n_solid_tracks": (self.meta or {}).get("n_solid_tracks"),
            "links_truncated": (self.meta or {}).get("links_truncated"),
            "scene_scan": ({k: (self.scene_scan or {}).get(k)
                            for k in ("supported", "cap", "found", "bounded",
                                      "truncated")}
                           if self.scene_scan is not None else None),
            "run_contacts": ({k: (self.run_contacts or {}).get(k)
                              for k in ("supported", "every_n_steps",
                                        "steps_sampled", "total_observed",
                                        "distinct_named", "unpaired",
                                        "truncated", "error")}
                             if self.run_contacts is not None else None),
            "run_contact_pairs": [
                "%s/%s" % (p.get("a"), p.get("b"))
                for p in ((self.run_contacts or {}).get("pairs") or [])][:8],
            "attempts_used": self.attempts_used,
            "launch_backend_env": backend_env_knobs(self.launch_env),
            "tamper": list(self.tamper),
            "error": self.error,
        }


def inject_recorder(world: Path, out_csv: Path, *, duration, settle,
                    phase_a=True, contact_steps=10, tag="run",
                    solids=(), links=0, solid_tracks=None,
                    scan_solids=None, run_contacts=None,
                    robot_contacts_only=False, contact_witness_defs=()) -> Path:
    """Write ``<worlddir>/_agentbench_<tag>.wbt`` = world + recorder stanza.

    ``links``, ``solid_tracks`` and ``scan_solids`` are emitted ONLY when they
    differ from the recorder's own defaults, so a task that asks for none of
    them produces a byte-identical stanza to the one every already-frozen run
    used. ``scan_solids`` is the cap on the name-free t=0 scene scan; leaving
    it ``None`` takes the recorder's default (on), and ``0`` turns the scan off
    for a task that does not want to pay for it.
    """
    world = Path(world)
    sibling = world.with_name("%s%s_%s.wbt" % (INJECTED_PREFIX, tag,
                                               world.stem))
    args = [
        '    "--out=%s"' % as_wbt_path(out_csv),
        '    "--duration=%r"' % float(duration),
        '    "--settle=%r"' % float(settle),
        '    "--contact-steps=%d"' % int(contact_steps),
        '    "--phase-a=%d"' % (1 if phase_a else 0),
    ]
    if solids:
        args.append('    "--solids=%s"' % ",".join(solids))
    if links:
        args.append('    "--links=%d"' % int(links))
    if solid_tracks:
        args.append('    "--solid-tracks=%s"' % solid_tracks)
    if scan_solids is not None:
        args.append('    "--scan-solids=%d"' % int(scan_solids))
    if run_contacts is not None:
        args.append('    "--run-contacts=%d"' % int(run_contacts))
    if robot_contacts_only:
        args.append('    "--robot-contacts-only=1"')
    if contact_witness_defs:
        args.append('    "--contact-witness-defs=%s"' %
                    ",".join(contact_witness_defs))
    text = world.read_text(encoding="utf-8", errors="replace")
    if not text.endswith("\n"):
        text += "\n"
    sibling.write_text(text + RECORDER_STANZA % "\n".join(args),
                       encoding="utf-8")
    return sibling


def _read_csv(path):
    with open(path, newline="", encoding="utf-8") as fh:
        rows = list(csv.reader(fh))
    if len(rows) < 2:
        return None, None
    data = np.array([[float(v) for v in r] for r in rows[1:]], dtype=float)
    t = data[:, 0]
    n = (data.shape[1] - 1) // 3
    xyz = np.empty((n, data.shape[0], 3), dtype=float)
    for i in range(n):
        xyz[i] = data[:, 1 + 3 * i: 4 + 3 * i]
    return t, xyz


def run_standalone(world, run_dir, *, attempts=3, **kw):
    """Run ``world`` headless with the recorder injected, retrying the known
    back-to-back-launch flake. Never raises.

    Measured on this machine: roughly one launch in three dies during startup
    with ``QWaitCondition: Destroyed while threads are still waiting`` and no
    diagnostics, producing no CSV -- the same race omnibench's
    ``launch_with_retries`` exists for. A retry is only taken when the engine
    log shows NO ``ERROR:`` lines, i.e. there is no evidence the agent's own
    world is at fault; a world that genuinely fails to load is a FAIL and must
    not be retried into looking clean. ``attempts_used`` is recorded on the
    result and surfaces in the row as a deviation.
    """
    last = None
    for attempt in range(1, attempts + 1):
        res = _run_standalone_once(world, run_dir, **kw)
        res.attempts_used = attempt
        last = res
        # Retry ONLY when the stack broke. Samples, an ERROR: line about the
        # world, and a recorder that completed with nothing to record are all
        # verdicts -- retrying any of them would launder a result.
        if not stack_broke(res):
            return res
        if attempt < attempts:
            time.sleep(2.0 * attempt)
    return last


def ph_error_lines(log_text):
    return [ln for ln in (log_text or "").splitlines()
            if ln.startswith("ERROR:")]


def recorder_finished(res):
    """True when the grader-owned recorder ran to completion and quit.

    Read from ``<out>.meta.json`` -- the recorder writes it before it starts,
    rewrites it with ``complete`` / ``quit_called`` at ``finish()``, and the
    grader deletes any stale copy before every launch. So this is the
    recorder's own attestation that it built the scene, walked it, stepped to
    the target sim time and quit, independent of how many rows it had to
    write.
    """
    meta = (res.meta or {}) if res is not None else {}
    return bool(meta.get("complete") and meta.get("quit_called"))


def stack_broke(res):
    """True when phase B produced no trajectory for a reason that is NOT the
    artifact's fault -- the launch died, not the world.

    This is the INVALID-vs-FAIL discriminator, and it must NOT be a row count.
    A world with no robots in it is a legitimate **FAIL**: the recorder loads
    it, finds nothing to track, writes a header-only CSV, records
    ``"complete": true`` in its meta and quits 0. ``_read_csv`` returns
    ``(None, None)`` for that file exactly as it does for a launch that never
    produced a CSV at all -- so gating on "no samples" scored a valid-but-empty
    world as "our harness broke", burning the launch retries on the way.
    Gate on the recorder's own completion flag instead: if it finished, the
    stack worked and the verdict stands.
    """
    if res is None or res.xyz is not None:
        return False
    if getattr(res, "tamper", None):
        return False              # refused, not broken: a TAMPER verdict
    if ph_error_lines(res.log_text):
        return False              # the engine named a fault in the world: FAIL
    if recorder_finished(res):
        return False              # it ran; there was nothing to record
    return True


def _run_standalone_once(world, run_dir, *, duration=30.0, settle=1.0,
                         phase_a=True, contact_steps=10, backend=None,
                         timeout_s=900.0, tag="phaseB", keep_injected=False,
                         extra_env=None, solids=(), links=0,
                         solid_tracks=None, scan_solids=None,
                         run_contacts=None, robot_contacts_only=False,
                         contact_witness_defs=()):
    """One launch. Never raises."""
    res = PhaseBResult(run_dir)
    run_dir = Path(run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)
    res.world = Path(world)

    # BEFORE anything is written or launched: could this world's own project
    # supply the grader's recorder? If so the measurement would be the agent's
    # to author, and no result from this launch is worth having.
    res.tamper = controller_shadow_check(res.world)
    if res.tamper:
        res.error = describe_tamper(res.tamper)
        return res

    binpath = engine_launch.resolve_binary(REPO)
    if binpath is None:
        res.error = "omnisim-bin not found"
        return res

    out_csv = run_dir / ("%s.csv" % tag)
    res.log_path = run_dir / ("%s_engine.log" % tag)
    console = run_dir / ("%s_console.log" % tag)

    try:
        res.injected_world = inject_recorder(
            res.world, out_csv, duration=duration, settle=settle,
            phase_a=phase_a, contact_steps=contact_steps, tag=tag,
            solids=solids, links=links, solid_tracks=solid_tracks,
            scan_solids=scan_solids, run_contacts=run_contacts,
            robot_contacts_only=robot_contacts_only,
            contact_witness_defs=contact_witness_defs)
    except OSError as exc:
        res.error = "could not write the injected world: %r" % (exc,)
        return res

    extra = {"WEBOTS_EXTRA_PROJECT_PATH": str(AGENTBENCH)}
    if extra_env:
        extra.update(extra_env)
    env = engine_launch.build_env(backend, res.log_path, repo=REPO,
                                  extra=extra)
    res.launch_env = env
    # Deleting the sidecar is belt-and-braces: WbLog already removes a stale
    # copy when it truncates the log at startup, so its presence afterwards
    # means Newton drove THIS run.
    for p in (Path(str(res.log_path) + ".newton.json"), out_csv,
              Path(str(out_csv) + ".meta.json"),
              Path(str(out_csv) + ".phase_a.json")):
        try:
            p.unlink()
        except OSError:
            pass

    try:
        res.rc, res.wall_s, res.timed_out = engine_launch.launch_once(
            binpath, res.injected_world, env, console, timeout_s)
    except Exception as exc:  # noqa: BLE001
        res.error = "launch failed: %r" % (exc,)
        return res
    finally:
        if not keep_injected:
            try:
                res.injected_world.unlink()
            except OSError:
                pass

    try:
        res.log_text = Path(res.log_path).read_text(encoding="utf-8",
                                                    errors="replace")
    except OSError:
        res.log_text = ""
    res.sidecar = engine_launch.newton_verdict(res.log_path)

    meta_p = Path(str(out_csv) + ".meta.json")
    if meta_p.exists():
        try:
            res.meta = json.loads(meta_p.read_text(encoding="utf-8"))
        except ValueError as exc:
            res.error = "recorder meta unreadable: %r" % (exc,)
    pa_p = Path(str(out_csv) + ".phase_a.json")
    if pa_p.exists():
        try:
            res.phase_a = json.loads(pa_p.read_text(encoding="utf-8"))
        except ValueError:
            res.phase_a = None
    if out_csv.exists():
        try:
            res.t, res.xyz = _read_csv(out_csv)
        except (OSError, ValueError) as exc:
            res.error = "recorder csv unreadable: %r" % (exc,)
    elif res.error is None:
        res.error = ("no recorder CSV: the run never reached the recording "
                     "window (rc=%s, timed_out=%s)" % (res.rc, res.timed_out))
    return res


BACKEND_ENV_KEYS = ("OMNISIM_FORCE_ODE", "OMNISIM_LEGACY",
                    "OMNISIM_REQUIRE_NEWTON")


def backend_env_knobs(launch_env):
    """The backend-selecting variables the CHILD was given, for the row."""
    if launch_env is None:
        return None
    return {k: launch_env.get(k) for k in BACKEND_ENV_KEYS
            if launch_env.get(k) is not None}


def newton_attribution(sidecar, world_text="", *, launch_env=None):
    """SPEC A1.10: which backend drove the run, or None when unattributable.

    Newton: the race-free ``<log>.newton.json`` verdict. Anything else is
    *unattributed* and the caller must mark the run INVALID -- an unattributed
    physics result is not a result.

    ⚠️ THE ODE BRANCH WAS REMOVED, 2026-08-08, AND ITS REMOVAL IS A CORRECTNESS
    FIX, NOT HOUSEKEEPING. This function used to attribute a run to
    ``backend: "ode"`` on either an explicit ``physicsBackend "ode"`` pin in the
    world or ``OMNISIM_FORCE_ODE`` / ``OMNISIM_LEGACY`` in the launch env, and
    that attribution made the row GRADEABLE. **src/ode was DELETED (commit
    bdc02139)**, so neither signal identifies a physics engine any more: both
    now describe a run that is not an ODE run. The two signals fail
    differently and both are disqualifying. The ENV knobs are simply IGNORED by
    the current engine (verified 2026-08-08: the run comes up on Newton and
    writes a normal sidecar), so attributing to "ode" would label a Newton row
    as ODE. The FIELD pin is still honoured, and the engine warns what it now
    means: "This Solid asks for physicsBackend \"ode\", which no longer selects
    a physics engine ... The node will have NO gravity and NO contact -- it is a
    visual-only body." Grading either as "omnisim-ode" is exactly the laundering
    the paragraph below was written to prevent, one level up.

    So an ODE pin/knob no longer produces an attribution at all -- it returns
    ``None``, the run is INVALID, and the operator is told why. That is the
    honest verdict: not "ODE drove it", but "nothing did".

    ``launch_env`` is the resolved CHILD environment
    (``PhaseBResult.launch_env``), never ``os.environ``. That distinction is
    the whole point: ``engine_launch.build_env`` POPS all three backend knobs
    out of the child env and re-adds one only when a backend was explicitly
    requested, so reading the parent's environment attributes a Newton-driven
    run to ODE while citing a variable the engine never saw -- laundering a run
    A1.10 says must be INVALID into a graded, falsely-attributed row.
    ``launch_env=None`` means "the launch env is not known here", and an
    unknown launch env is never evidence FOR a backend.
    """
    if sidecar.get("present"):
        return {"backend": "newton",
                "solver": sidecar.get("solver"),
                "degraded": bool(sidecar.get("degraded")),
                "finalised": sidecar.get("finalised"),
                "source": "<log>.newton.json sidecar"}
    # No ODE branch: see the docstring. A pin or a retired knob is not an
    # engine, so it cannot attribute a run.
    return None


def ode_request_detected(world_text="", *, launch_env=None):
    """Was a run ASKING for the deleted ODE backend? -> a human-readable reason.

    Not an attribution (see ``newton_attribution``): this exists so an INVALID
    row can say *why* it is unattributable instead of only "sidecar absent",
    which reads like a slow load rather than a dead knob.
    """
    env = launch_env or {}
    for key in ("OMNISIM_FORCE_ODE", "OMNISIM_LEGACY"):
        if env.get(key) == "1":
            return ("%s=1 was in the launch env. That knob is RETIRED: ODE was "
                    "deleted (src/ode, commit bdc02139). The engine now IGNORES "
                    "it, so the run was NOT an 'ODE run' -- it was a Newton run "
                    "with a misleading knob set. Unset it." % key)
    if _ODE_PIN.search(world_text or ""):
        return ('the world carries an explicit physicsBackend "ode" pin. That '
                "field still parses AND is still honoured, but ODE was deleted "
                "(src/ode, commit bdc02139), so the engine warns that the "
                "pinned node gets NO gravity and NO contact -- a visual-only "
                "body. Delete the field to simulate it with Newton.")
    return None


# WorldInfo.defaultPhysicsBackend pins a whole world; Solid/Robot
# .physicsBackend pins one body. Either is an explicit, auditable ODE choice.
_ODE_PIN = re.compile(r'(?:default)?[Pp]hysicsBackend\s+"ode"')
