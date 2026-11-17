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

"""OmniSim phase B for the ladder: run a deliverable cold, with OUR sampler.

**This file is ADAPTER code.** It is allowed to know what a scene file is and
where an engine binary lives; the graders are not.

What is reused, by import, and never copied
-------------------------------------------

===============================================  ==========================
reused                                            from
===============================================  ==========================
``resolve_binary`` / ``build_env`` /              omnibench, via
``launch_once`` / ``newton_verdict``              ``agentbench.common.paths``
``PhaseBResult`` (the phase-B result shape)       ``agentbench.adapters.
                                                  omnisim.headless``
``project_root_for_world`` (the engine's own      same
project resolution, for the tamper check)
``build_bundle`` (run -> neutral bundle)          ``agentbench.adapters.
                                                  omnisim.evidence``
===============================================  ==========================

So this module is a launcher plus two mappings. The launcher injects
``ladder_recorder`` instead of ``agentbench_recorder``; the mappings turn what
that sampler writes into (a) the ``phase_a`` shape the frozen bundle builder
already reads, so t=0 geometry and the vacuity witness arrive through the
published contract rather than a side door, and (b) the ladder's own
:class:`SupportContactObservation`.

The tamper check is not optional
--------------------------------

The engine searches ``<project>/controllers/<name>/`` **before** the extra
project paths, and it selects on directory EXISTENCE alone. For a scratch
deliverable, whose project root resolves to the run directory, anything at
``<run_dir>/controllers/ladder_recorder/`` wins over ours -- and a fake
sampler there authors every metre in the row. An empty directory there is
enough to win, which turns a cell into a broken harness instead of a FAIL.
Both are refused before launch, never graded.
"""

from __future__ import annotations

import csv
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from agentbench.adapters.omnisim import headless as ab_headless  # noqa: E402
from agentbench.adapters.omnisim.evidence import (  # noqa: E402,F401
    build_bundle)
from agentbench.common.paths import (  # noqa: E402
    REPO, as_wbt_path, engine_launch)
from agentbench.common.worldtext import INJECTED_PREFIX  # noqa: E402
from ladder.adapters.omnisim import channels as ladder_channels  # noqa: E402
from ladder.graders.ladder_evidence import (  # noqa: E402
    SupportContact, SupportContactObservation)

LADDER = Path(__file__).resolve().parents[2]          # tests/benchmarks/ladder
SAMPLER = "ladder_recorder"
CANONICAL_SAMPLER_DIR = LADDER / "controllers" / SAMPLER

SAMPLER_STANZA = """
# --- appended by the capability-ladder grader ------------------------------
Robot {
  name "%s"
  controller "%s"
  supervisor TRUE
  controllerArgs [
%%s
  ]
}
""" % (SAMPLER, SAMPLER)

SUPPORT_SOURCE = (
    "the grader-owned sampler's per-step contact scan across the recorded "
    "window (the scene's own contact query, paired by world point), kept to "
    "pairs where exactly ONE side resolves to a robot subtree and the other "
    "does not")

TRAJECTORY_SOURCE = (
    "the grader-owned sampler: every robot's world pose once per basic "
    "timestep, written as %.17g CSV")


# --- the tamper check --------------------------------------------------------


def sampler_shadow_check(world):
    """Grader-owned sampler directories the world's OWN project would supply.

    ``[]`` means clean. Reuses the engine's own project resolution so the
    check looks where the engine will actually look; see the module docstring
    for why an EMPTY directory is as disqualifying as a fake one.
    """
    root = ab_headless.project_root_for_world(world)
    hits = list(ab_headless.controller_shadow_check(world))
    p = root / "controllers" / SAMPLER
    if p.exists():
        try:
            same = p.resolve() == CANONICAL_SAMPLER_DIR.resolve()
        except OSError:
            same = False
        if not same:
            try:
                entries = sorted(c.name for c in p.iterdir())
            except OSError:
                entries = None
            hits.append({"controller": SAMPLER, "path": str(p),
                         "project_root": str(root), "entries": entries})
    return hits


# --- launching ---------------------------------------------------------------


def inject_sampler(world, out_csv, *, duration, settle, stride, surfaces=(),
                   tag="phaseB", tier="t1", record_stride=1):
    """Write ``<worlddir>/_agentbench_<tag>_<stem>.wbt`` = world + sampler.

    A **sibling, in the deliverable's own directory**, because asset
    references inside a scene resolve relative to the scene file: relocating
    it silently breaks every one of them. The prefix is shared with the
    AgentBench injector on purpose, so the same cleanup and the same
    "not-an-artifact" discovery rule cover both.
    """
    world = Path(world)
    sibling = world.with_name("%s%s_%s.wbt" % (INJECTED_PREFIX, tag,
                                               world.stem))
    args = ['    "--out=%s"' % as_wbt_path(out_csv),
            '    "--duration=%r"' % float(duration),
            '    "--settle=%r"' % float(settle),
            '    "--contact-stride=%d"' % int(stride),
            '    "--tier=%s"' % str(tier),
            '    "--record-stride=%d"' % int(record_stride)]
    if surfaces:
        args.append('    "--surfaces=%s"' % ",".join(surfaces))
    text = world.read_text(encoding="utf-8", errors="replace")
    if not text.endswith("\n"):
        text += "\n"
    sibling.write_text(text + SAMPLER_STANZA % "\n".join(args),
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


#: Attempts at a cold launch before giving up. **Four, not two, and the
#: difference is measured.** The sampler is a controller process and its IPC
#: attach races the engine's startup: when it loses, the engine loads the world,
#: steps it, exits 0 and writes a clean log, and NO motion file appears -- which
#: a grader reads as "the run produced nothing", i.e. as the agent failing.
#:
#: Measured 2026-08-02 on the T1 world, same config every time, ODE:
#:
#:   attempts=2 -> 2 of 4 sequential runs produced no data (both attempts lost)
#:   attempts=4 -> 4 of 4, one of them needing a second try
#:
#: The successful runs were bit-identical (0.0495 m, 8.08 s, to four decimals),
#: so the flake is purely the attach and never the physics. Raising this is a
#: workaround for that race, not a fix for it: the race itself is worth closing.
LAUNCH_ATTEMPTS = 4


def run_standalone(world, run_dir, *, attempts=LAUNCH_ATTEMPTS, **kw):
    """Run ``world`` cold with the ladder's sampler injected. Never raises.

    Retries for the known back-to-back-launch race, taken **only** when the
    stack broke -- an error line about the deliverable is a verdict and
    retrying it would launder a result. ``attempts_used`` rides on the result
    and belongs in the row as a deviation.

    ⚠ **This generic name defaults to ``tier="all"``, and that default is a
    measured fix rather than a taste.** ``ladder.graders.t2`` -- alone among
    the four shims -- has no tier-specific preference loop: it calls
    ``run_standalone`` by that exact name and nothing else. When this hook
    defaulted to T1's cheap mode, ``t2.run_and_grade(sim="omnisim")`` ran a
    sampler that writes no tier-channel document at all, and the T2 verdict
    came back with four channels "unanswered" -- i.e. blaming OUR scaffolding
    -- on a run that could have answered every one of them. Defaulting the
    generic hook to the full scan fixes that **without editing a grader**:
    :func:`t1_run_standalone` pins the cheap mode for the one tier that does
    not need the extra channels, and every other entry point gets them.
    """
    kw.setdefault("tier", "all")
    kw.setdefault("record_stride", RECORD_STRIDE)
    last = None
    for attempt in range(1, attempts + 1):
        res = _run_once(world, run_dir, **kw)
        res.attempts_used = attempt
        last = res
        if not ab_headless.stack_broke(res):
            return res
        if attempt < attempts:
            time.sleep(2.0 * attempt)
    return last


#: How often the tier-channel pose series is written, in basic timesteps. The
#: T1 CSV stays every step; the tier document carries orientation for every
#: named body in the scene, so the same rate would be ~12x the bytes for a
#: series every threshold in T2/T3/T4 reads over seconds, not milliseconds.
RECORD_STRIDE = 5


def t1_run_standalone(world, run_dir, **kw):
    """T1's phase B, named explicitly so the preference loops find it first."""
    kw.setdefault("tier", "t1")
    return run_standalone(world, run_dir, **kw)


def t2_run_standalone(world, run_dir, **kw):
    """T2's phase B: the T1 outputs plus the tier-channel document."""
    kw.setdefault("tier", "t2")
    kw.setdefault("record_stride", RECORD_STRIDE)
    return run_standalone(world, run_dir, **kw)


def t3_run_standalone(world, run_dir, **kw):
    """T3's phase B: the T1 outputs plus the tier-channel document."""
    kw.setdefault("tier", "t3")
    kw.setdefault("record_stride", RECORD_STRIDE)
    return run_standalone(world, run_dir, **kw)


def t4_run_standalone(world, run_dir, **kw):
    """T4's phase B: the T1 outputs plus the tier-channel document."""
    kw.setdefault("tier", "t4")
    kw.setdefault("record_stride", RECORD_STRIDE)
    return run_standalone(world, run_dir, **kw)


def _run_once(world, run_dir, *, duration=60.0, settle=0.5, stride=10,
              surfaces=(), backend=None, timeout_s=900.0, tag="phaseB",
              keep_injected=False, extra_env=None, tier="t1",
              record_stride=1):
    res = ab_headless.PhaseBResult(run_dir)
    res.support = None
    res.channels = None
    res.tier = tier
    run_dir = Path(run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)
    res.world = Path(world)

    res.tamper = sampler_shadow_check(res.world)
    if res.tamper:
        res.error = ab_headless.describe_tamper(res.tamper)
        return res

    binpath = engine_launch.resolve_binary(REPO)
    if binpath is None:
        res.error = "the engine binary was not found"
        return res

    out_csv = run_dir / ("%s.csv" % tag)
    res.log_path = run_dir / ("%s_engine.log" % tag)
    console = run_dir / ("%s_console.log" % tag)

    try:
        res.injected_world = inject_sampler(
            res.world, out_csv, duration=duration, settle=settle,
            stride=stride, surfaces=surfaces, tag=tag, tier=tier,
            record_stride=record_stride)
    except OSError as exc:
        res.error = "could not write the injected scene: %r" % (exc,)
        return res

    extra = {"WEBOTS_EXTRA_PROJECT_PATH": str(LADDER)}
    if extra_env:
        extra.update(extra_env)
    env = engine_launch.build_env(backend, res.log_path, repo=REPO,
                                  extra=extra)
    res.launch_env = env
    for p in (Path(str(res.log_path) + ".newton.json"), out_csv,
              Path(str(out_csv) + ".meta.json"),
              Path(str(out_csv) + ".support.json"),
              Path(str(out_csv) + ".channels.json")):
        try:
            p.unlink()
        except OSError:
            pass

    try:
        res.rc, res.wall_s, res.timed_out = engine_launch.launch_once(
            binpath, res.injected_world, env, console, timeout_s)
    except Exception as exc:  # noqa: BLE001  (adapter rule 1: never raise)
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
            res.error = "the sampler's meta file is unreadable: %r" % (exc,)
    sup_p = Path(str(out_csv) + ".support.json")
    if sup_p.exists():
        try:
            res.support = json.loads(sup_p.read_text(encoding="utf-8"))
        except ValueError:
            res.support = None
    chan_p = Path(str(out_csv) + ".channels.json")
    if chan_p.exists():
        try:
            res.channels = json.loads(chan_p.read_text(encoding="utf-8"))
        except ValueError:
            res.channels = None
    res.channels_path = str(chan_p)
    res.phase_a = phase_a_from_support(res.support)

    if out_csv.exists():
        try:
            res.t, res.xyz = _read_csv(out_csv)
        except (OSError, ValueError) as exc:
            res.error = "the sampler's CSV is unreadable: %r" % (exc,)
    elif res.error is None:
        res.error = ("no sample file: the run never reached the recording "
                     "window (rc=%s, timed_out=%s)" % (res.rc, res.timed_out))
    return res


# --- the two mappings --------------------------------------------------------


def phase_a_from_support(support):
    """The ladder sampler's output, in the shape the frozen builder reads.

    Deliberately routed through the published contract rather than a side
    door: t=0 geometry and the vacuity witness arrive in ``build_bundle``
    exactly as they would from the AgentBench sampler, so both columns' t=0
    halves are like for like. ``robot_robot_contacts`` is empty because T1
    does not ask about robot-robot contact -- the pairs the ladder cares
    about go through :func:`support_observation` instead.
    """
    if not support:
        return None
    bodies = support.get("t0_bodies") or []
    w = support.get("witness") or {}
    return {
        "t0_robots": [b for b in bodies if b.get("robot_class")],
        "t0_solids": [b for b in bodies if not b.get("robot_class")],
        "robot_robot_contacts": [],
        "contact_steps": int(w.get("steps_sampled") or 0),
        "contact_witness": {
            "supported": bool(w.get("supported")),
            "total_observed": w.get("total_observed"),
            "distinct_named": w.get("distinct_named"),
            "steps_sampled": w.get("steps_sampled"),
            "error": w.get("error")},
        "observe_error": support.get("observe_error"),
        "bounds_error": support.get("bounds_error"),
        "dt_ms": support.get("dt_ms"),
    }


#: ⚠ **MEASURED, and it decides how a red contact assertion must be read.**
#: The supervisor contact query is fed from the ODE collision callback and the
#: Newton backend never populates it. The query still runs, still reports
#: ``supported: true`` and still reports **no error** -- it simply sees
#: nothing. Measured on one unchanged T1 probe scene: **1008** support
#: contacts on ODE, **0** on Newton over 126 sampled steps. Newton is the
#: engine's DEFAULT where its runtime is present, so an agent who authors an
#: ordinary scene gets the blind path.
#:
#: This is not ``scaffolding_defect_ours`` -- the grader's sampler works and
#: proves it works on the other backend -- and it is not a physics shortfall
#: either. It is ``no_measurement_surface``, and because the cell runner's own
#: classifier cannot see the difference (a supplied-but-empty channel reads to
#: it as a measured shortfall), the sentence has to travel **inside the
#: channel** so a reviewer re-labelling under §3.4 has it in the row.
NEWTON_CONTACT_BLINDNESS = (
    "NO CONTACT WAS OBSERVABLE ON THIS RUN'S BACKEND. The engine reports "
    "backend=%s: OmniSim's supervisor contact query (WbSolid's contact-point "
    "list) is fed from the ODE collision callback and the Newton backend "
    "never populates it, so the query runs cleanly, reports no error and "
    "returns nothing. Measured on one unchanged probe scene: 1008 support "
    "contacts on ODE and 0 on Newton over 126 sampled steps. A red contact "
    "assertion on this run is therefore a MEASUREMENT-SURFACE gap in the "
    "simulator (blocker no_measurement_surface), not a shortfall by the "
    "scene and not a channel we failed to build")


def backend_note(phase_b):
    """The blindness sentence when this run was Newton-backed, else ``""``.

    Read from the engine's own ``.newton.json`` verdict sidecar, which the
    launcher already collected -- never from a log scrape.
    """
    side = getattr(phase_b, "sidecar", None) or {}
    backend = str(side.get("backend") or "")
    if backend and backend != "ode":
        return NEWTON_CONTACT_BLINDNESS % (
            backend + ("/" + str(side["solver"]) if side.get("solver")
                       else ""))
    return ""


def support_observation(phase_b):
    """``SupportContactObservation`` for one phase-B run, or ``None``.

    ``None`` means *"there was no run to read"* and lets the shim fall back
    and report the gap. A run that happened and saw nothing returns an
    observation with an empty pair list and its witness counters intact --
    which is a measurement, and a very different thing.
    """
    support = (getattr(phase_b, "support", None)
               if phase_b is not None else None)
    if not support:
        return None
    w = support.get("witness") or {}
    pairs = [SupportContact(robot_body=c.get("robot_body"),
                            surface_body=c.get("surface_body"),
                            surface_is_robot=c.get("surface_is_robot"),
                            point=(tuple(c["point"]) if c.get("point")
                                   else None),
                            t_s=c.get("t_s"), step=c.get("step"))
             for c in (support.get("pairs") or [])]
    err = w.get("error")
    if not err and not pairs:
        err = backend_note(phase_b) or None
    return SupportContactObservation(
        pairs=pairs, supported=bool(w.get("supported")),
        total_observed=w.get("total_observed"),
        distinct_named=w.get("distinct_named"),
        steps_sampled=int(w.get("steps_sampled") or 0),
        window_s=support.get("window_s"),
        source=SUPPORT_SOURCE, error=err)


# --- the tier channels (T2/T3/T4) --------------------------------------------
#
# The three hooks are thin on purpose: locate the document the grader-owned
# sampler wrote, then hand it to the PURE builder in ``channels.py``. That is
# what keeps the whole T2-T4 surface testable with no engine binary, and it is
# the same split the MuJoCo column uses between ``recording.py`` and its
# runners.

T2_CHANNEL_KEYS = ladder_channels.T2_CHANNEL_KEYS
T3_CHANNEL_KEYS = ladder_channels.T3_CHANNEL_KEYS
T4_CHANNEL_KEYS = ladder_channels.T4_CHANNEL_KEYS

#: Which task file each rung's declared body name is read from. **Task data,
#: read from the task file** -- the same file, and the same field, that
#: ``ladder.graders.t3.build_evidence`` reads its own fallback's ``robot``
#: from (``task.robot_name``). The adapter is not choosing anything: the T3/T4
#: shims pass their channel hook only a ``surface``, so a column that records
#: every named body (as this one does, on purpose) has to be told which name
#: the tier declared before it can hand back a series for it.
RUNG_TASKS = {"t3": "T3_quadruped", "t4": "T4_humanoid"}


def _channel_doc(phase_b=None, *, run_dir=None):
    """The tier document behind whatever the shim passed. Never raises.

    Three shapes reach here, and all three are legitimate: a phase-B result
    object (a fresh run), a path (grading an EXISTING run directory -- a
    re-grade, a fixture, a run somebody else performed), or nothing.
    """
    doc = getattr(phase_b, "channels", None)
    if isinstance(doc, dict):
        return doc
    cands = []
    for c in (run_dir, phase_b, getattr(phase_b, "run_dir", None)):
        if isinstance(c, (str, Path)):
            cands.append(Path(c))
    for base in cands:
        if base.is_file() and base.name.endswith(".channels.json"):
            hits = [base]
        elif base.is_dir():
            hits = sorted(base.glob("*.channels.json"))
        else:
            hits = []
        for p in hits:
            try:
                got = json.loads(p.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                continue
            if isinstance(got, dict):
                return got
    return None


def _declared_robot_name(rung):
    """The tier's own ``robot.declared_name``, from its own ``meta.json``."""
    try:
        from ladder import tasks as ladder_tasks
        return ladder_tasks.get(RUNG_TASKS[rung]).robot_name
    except Exception:  # noqa: BLE001  (adapter rule 1: never raise)
        return ""


def t2_channels(phase_b=None, *, roles=None, run_dir=None):
    """T2's eight channels from one phase-B run. ``{}`` when there was none."""
    doc = _channel_doc(phase_b, run_dir=run_dir)
    if not doc or roles is None:
        return {}
    return ladder_channels.build_t2(doc, roles,
                                    backend_note=backend_note(phase_b))


def t3_channels(phase_b=None, *, surface=None, run_dir=None, robot_name=None):
    """T3's eight channels from one phase-B run. ``{}`` when there was none."""
    doc = _channel_doc(phase_b, run_dir=run_dir)
    if not doc:
        return {}
    name = robot_name or _declared_robot_name("t3")
    return ladder_channels.build_t3(doc, surface, name,
                                    backend_note=backend_note(phase_b))


def t4_channels(phase_b=None, *, surface=None, run_dir=None, robot_name=None):
    """T4's eight channels. The same eight T3 asks for, from the same scan."""
    doc = _channel_doc(phase_b, run_dir=run_dir)
    if not doc:
        return {}
    name = robot_name or _declared_robot_name("t4")
    return ladder_channels.build_t4(doc, surface, name,
                                    backend_note=backend_note(phase_b))


__all__ = ["SAMPLER", "LADDER", "RECORD_STRIDE", "RUNG_TASKS",
           "T2_CHANNEL_KEYS", "T3_CHANNEL_KEYS", "T4_CHANNEL_KEYS",
           "build_bundle", "inject_sampler",
           "run_standalone", "t1_run_standalone", "t2_run_standalone",
           "t3_run_standalone", "t4_run_standalone",
           "sampler_shadow_check", "phase_a_from_support",
           "support_observation", "backend_note", "t2_channels",
           "t3_channels",
           "t4_channels"]


def l2_run_standalone(world, run_dir, **kw):
    """L2's phase B on this column: T1's, unchanged.

    Aliased rather than reimplemented on purpose. L2 grades the SAME robot on
    the SAME floor as T1 and differs only in its thresholds and in the loop
    clauses, so sharing the sampler means a difference between an L2 row and a
    T1 row here is the target and never the instrument.
    """
    return t1_run_standalone(world, run_dir, **kw)


def p1_run_standalone(world, run_dir, **kw):
    """P1's phase B on this column: the same sampler T1 and L2 use.

    P1 grades a route from a pose series, which is exactly what the ladder
    recorder already writes, so nothing new is needed here beyond a name the
    rung's lookup can find.
    """
    return t1_run_standalone(world, run_dir, **kw)
