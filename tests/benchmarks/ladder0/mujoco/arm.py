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

"""arm.py -- the MuJoCo arm's CONTRACT.md section 5 face.

``run.py`` is this arm's own entry point and does the work; this file is the
three-function interface ``run_ladder.py`` discovers, so the MuJoCo column
appears in the shared table beside OmniSim and upstream Webots instead of
being reported as an arm that is not in the tree.

It is a thin adapter and deliberately so: it launches nothing, computes no
expected value and owns no tolerance.  The one thing it adds is that ``run()``
CANNOT RAISE -- a broken simulator is a measurement, and an exception here
would abort the row, which is indistinguishable from a row that was never run.

This arm runs IN-PROCESS.  The other two spawn an engine, so their
``proc_t0``/``proc_t1`` are real process spawn and exit stamps; here they mark
entry to and exit from ``run()``, which is the same quantity for a library
call.  The startup column is therefore genuinely ~0 for this arm rather than
missing, and that is a true statement about it: there is no process to start.

Module loading: siblings come from ``shared.sibling()``, by path and under an
arm-qualified name.  Never ``import run`` -- ``sys.modules`` is shared with the
other two arms inside ``run_ladder.py``'s process (see ``shared.py``).
"""

from __future__ import annotations

import importlib.util
import json
import os
import platform
import sys
import time

HERE = os.path.abspath(os.path.dirname(__file__))

NAME = "mujoco"


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
_run = shared.sibling("run")

# CONTRACT.md section 6.  Every one is a physical corruption of the scene or
# the actuation; a fault this arm cannot inject is REPORTED as unsupported,
# never quietly run as a control -- "the fault did not happen" must not read as
# "the fault produced no failure".
#
# READ FROM ``run.FAULTS``, never re-listed here.  A second copy of this set is
# a place for the two to disagree, and the way they disagree is the worst one
# available: the arm answers "I do not implement that fault" for a fault it
# does implement, and the self-test scores the proof as MISSING rather than as
# failed.  That happened -- the rung 5-8 faults were live in run.py for a whole
# build while this frozenset still named only the first five.
SUPPORTED_FAULTS = frozenset(["none"]) | frozenset(_run.FAULTS)


def available():
    """(True, None) if this arm can run here; (False, why) if it cannot."""
    ok, detail = _run.available()
    return (True, None) if ok else (False, "mujoco not importable: %s" % detail)


def run(rung, out_dir, fault="none", subset=None, timeout_s=None, **_kw):
    """Run one cell.  Returns ``(samples, meta)`` and never raises.

    Rungs 9, 11 and 18 are MULTI-RUN cells (CONTRACT.md amendment A): one call,
    several runs of one contract-owned scene family.  ``subset`` reaches rung
    18, whose fault battery runs the contract's small toss set -- and runs it
    for the BASELINE the fault is judged against too, or the two would not be
    comparable.  The set of subsets an arm may have run is the contract's
    (``rungs.RUNG18_SUBSETS``); this arm only picks between them by name.
    """
    rung = int(rung)
    fault = fault or "none"
    try:
        os.makedirs(out_dir, exist_ok=True)
    except OSError:
        pass

    meta = {
        "sim": NAME, "rung": rung, "fault": fault,
        "engine": "bare mujoco, CPU mj_step (no OmniSim, no Newton, no GPU)",
        "error": None, "exit_code": None,
        "proc_t0": None, "proc_t1": None,
        "python": sys.version.split()[0], "machine": platform.node(),
    }
    empty = {"rung": rung, "sim": NAME, "t": [], "steps": 0}

    if fault not in SUPPORTED_FAULTS:
        meta["error"] = ("this arm does not implement fault %r (supported: %s)"
                         % (fault, ", ".join(sorted(SUPPORTED_FAULTS))))
        return empty, meta

    ok, why = available()
    if not ok:
        meta["error"] = why
        return empty, meta

    # Refresh THIS rung's committed model, best effort.
    #
    # ``run_rung`` does not read the file -- it compiles ``scenes.mjcf(rung)``
    # in memory -- so the write is housekeeping on a committed artefact and
    # must never be able to fail a measurement.  It could: this step used to
    # rewrite all nine models on every cell and abort the cell if any write
    # raised, and it DID, on ``OSError(22)`` while two other lanes were working
    # in the same tree.  A row that reads "scene generation failed" for a run
    # whose scene was never in doubt is a false negative, and a false negative
    # in an instrument is worse than a missing one because it gets attributed
    # to the simulator.
    #
    # Idempotent, too: the file is only opened when its content would change,
    # which removes the write race in the common case rather than surviving it.
    meta["model_refresh"] = "unchanged"
    try:
        scenes = shared.sibling("scenes")
        path = scenes.model_path(rung)
        fresh = scenes.mjcf(rung)
        try:
            with open(path, "r", encoding="utf-8") as fh:
                stale = fh.read() != fresh
        except OSError:
            stale = True
        if stale:
            with open(path, "w", encoding="utf-8", newline="\n") as fh:
                fh.write(fresh)
            meta["model_refresh"] = "rewritten"
    except Exception as exc:                        # noqa: BLE001
        # Reported, never fatal.  The scene the run actually used is built
        # from the contract in memory either way.
        meta["model_refresh"] = "failed: %r" % (exc,)

    meta["proc_t0"] = time.time()
    try:
        if rung in shared.spec.MULTI_RUN:
            samples, run_meta = _run.run_cell(
                rung, out_dir, fault=(None if fault == "none" else fault),
                subset=subset,
                **({"timeout_s": float(timeout_s)} if timeout_s else {}))
        else:
            samples, run_meta = _run.run_rung(
                rung, fault=(None if fault == "none" else fault))
    except Exception as exc:                        # noqa: BLE001
        meta["proc_t1"] = time.time()
        meta["error"] = "%s: %s" % (type(exc).__name__, exc)
        return empty, meta
    meta["proc_t1"] = time.time()

    samples["sim"] = NAME
    for key in ("mujoco_version", "solver", "compile_s", "us_per_step",
                "integrator_requested", "scene_is_contract_verbatim",
                # multi-run cells (amendment A) and rung 18's provenance
                "multi_run", "run_tags", "runs", "distinct_processes", "gaps",
                "lane1r_dataset", "lane1r_world", "record_step_ms",
                "timestep_s", "substeps_per_sample", "cube", "table_top_m",
                "scale_mode", "ground_truth", "rung18_solver_declarations",
                "reference_arm_caveat"):
        if key in run_meta:
            meta[key] = run_meta[key]
    if run_meta.get("error"):
        meta["error"] = run_meta["error"]
    meta["exit_code"] = run_meta.get("exit_code", 0)
    meta["engine_version"] = run_meta.get("mujoco_version")

    try:
        with open(os.path.join(out_dir, "samples.json"), "w",
                  encoding="utf-8") as f:
            json.dump(samples, f)
    except OSError:
        pass
    return samples, meta


__all__ = ["NAME", "available", "run", "SUPPORTED_FAULTS"]
