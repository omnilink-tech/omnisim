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

"""OmniBench run_all — one entry point for the whole suite (SPEC.md layout).

Orchestrates the existing lane CLIs as subprocesses, streams their output to
per-lane logs under the out dir, tolerates per-run failures (a failed run is
a recorded gap in MANIFEST.json, never a crash of the suite), fills the
lane-1 OmniSim metrics via score.py, and finishes by writing summary.md.

    python tests/benchmarks/omnibench/run_all.py \
        --lanes 1,2,3,4 --engines mujoco,pybullet,omnisim \
        --backends newton --dt-ms all

!! `--backends ode` is REFUSED since 2026-08-08: src/ode was DELETED (commit
bdc02139). Lane 1's ODE arm was the suite's cross-backend correctness
reference; with it gone, lane 1 measures Newton against the PER-SCENE ANALYTIC
ground truth in common/scenes.py (score.py reads no engine's numbers as a
reference), with mujoco and pybullet as the surviving external cross-checks.
There is NO frozen file of lane-1 ODE values -- tests/goldens/
ode_oracle_goldens.json is DEVICE-parity evidence with zero T1-T7 scenes. The
ODE arm's 105 lane-1 rows in results*/ are history and permanently unrepeatable.

(This docstring is argparse's `description`, i.e. what `--help` writes to
stdout, so it must stay ASCII: a decorative U+26A0 here crashed `--help` itself
with UnicodeEncodeError under Windows cp1252.)

Default out dir: tests/benchmarks/omnibench/results/<machine_id>/<utc-date>/

Parallelism: `--parallel K` runs K lane-1 OmniSim engine instances
concurrently (the engine is multi-instance-safe; per-run OMNISIM_LOG_PATH +
recordings are already unique). Defaults: 4 on Linux, 1 on Windows (the
back-to-back controller-connect launch race worsens under parallelism there).
`--overlap-lanes` additionally runs lane 2 (GPU) concurrently with lane 1's
CPU-bound matrix. Physics is unaffected — only scheduling changes; the
results.jsonl is rewritten in canonical matrix order after a parallel pass.

Windows nuance (hard-won): launch THIS script from PowerShell, not MSYS
bash, when the omnisim/newton lanes are in play — an MSYS environment can
make the engine's embedded interpreter miss warp and silently fall back to
ODE (the sidecar check then fails the run loudly, but the time is wasted).
"""
from __future__ import annotations

import argparse
import datetime
import json
import statistics
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent          # tests/benchmarks/omnibench
REPO = HERE.parents[2]
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
if str(HERE / "lane1") not in sys.path:
    sys.path.insert(0, str(HERE / "lane1"))

from common import results as results_mod  # noqa: E402
from common import scenes  # noqa: E402
import gen_worlds  # noqa: E402

DT_SWEEP = list(scenes.DT_SWEEP_MS)             # (1, 2, 4, 8, 16, 32)
LANE1_TESTS = list(scenes.TESTS)                # T1..T7
REF_ENGINES = ("mujoco", "pybullet")

# Sanity thresholds for the auto-FINDINGS section: (metric, cmp, threshold),
# keyed by SPEC test name. cmp ">" means value > threshold is a finding.
THRESHOLDS = {
    "bounce":          [("bounce_height_rmse_rel", ">", 0.1)],
    "incline":         [("stick_violation_max_m", ">", 0.001),
                        ("slide_accel_rel_err", ">", 0.1),
                        ("transition_angle_err_deg", ">", 1.0)],
    "roll":            [("roll_accel_rel_err", ">", 0.1),
                        ("slip_ratio", ">", 0.1)],
    "pendulum_energy": [("energy_drift_rel", ">", 0.1)],
    "momentum":        [("linear_momentum_max", ">", 0.1),
                        ("angular_momentum_drift_rel", ">", 0.1)],
    "stack":           [("stack_survivors", "<", 10),
                        ("settle_creep_m_s", ">", 0.001),
                        ("max_penetration_m", ">", 0.005)],
    "spin":            [("angmom_drift_rel", ">", 0.1),
                        ("rot_ke_drift_rel", ">", 0.1)],
}

# Per-test metric column order for the lane-1 summary tables.
LANE1_METRICS = {
    "bounce": ["bounce_height_rmse_rel"],
    "incline": ["stick_violation_max_m", "slide_accel_rel_err",
                "transition_angle_err_deg"],
    "roll": ["roll_accel_rel_err", "slip_ratio"],
    "pendulum_energy": ["energy_drift_rel", "energy_drift_slope"],
    "momentum": ["linear_momentum_max", "angular_momentum_drift_rel"],
    "stack": ["stack_survivors", "settle_creep_m_s", "max_penetration_m"],
    "spin": ["angmom_drift_rel", "rot_ke_drift_rel"],
}


def log(msg):
    # worker threads (parallel lane-1 matrix, overlapped lane 2) tag their
    # lines so interleaved output stays attributable
    t = threading.current_thread()
    tag = "" if t is threading.main_thread() else "[%s] " % t.name
    print("[run_all] %s%s" % (tag, msg), flush=True)


def utc_now():
    return datetime.datetime.now(datetime.timezone.utc) \
        .strftime("%Y-%m-%dT%H:%M:%SZ")


class Manifest:
    """Crash-safe suite manifest: rewritten after every recorded event.

    Thread-safe: record_run/record_gap/flush may be called concurrently from
    the parallel lane-1 workers and the overlapped lane-2 thread; an RLock
    serializes every mutate+rewrite (record_* call flush while holding it).
    """

    @classmethod
    def load(cls, path):
        """Re-open an existing manifest (gap-retry / summary passes)."""
        inst = cls.__new__(cls)
        inst._lock = threading.RLock()
        inst.path = Path(path)
        inst.data = json.loads(inst.path.read_text(encoding="utf-8"))
        return inst

    def __init__(self, path, args):
        self._lock = threading.RLock()
        self.path = Path(path)
        self.data = {
            "suite": results_mod.SUITE,
            "machine": results_mod.machine_fingerprint(),
            "utc_start": utc_now(),
            "utc_end": None,
            "args": vars(args).copy(),
            "runs": [],
            "gaps": [],
        }
        self.data["args"] = {k: (str(v) if isinstance(v, Path) else v)
                             for k, v in self.data["args"].items()}
        self.flush()

    def flush(self):
        with self._lock:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.path.with_suffix(".json.tmp")
            tmp.write_text(json.dumps(self.data, indent=2), encoding="utf-8")
            tmp.replace(self.path)

    def record_run(self, name, cmd, status, rc, wall_s, log_file, note=None):
        with self._lock:
            self.data["runs"].append({
                "name": name, "cmd": [str(c) for c in cmd], "status": status,
                "rc": rc, "wall_s": round(wall_s, 1),
                "log": str(log_file), "note": note, "utc": utc_now()})
            self.flush()

    def record_gap(self, lane, reason, **ident):
        gap = {"lane": lane, **ident, "reason": reason, "utc": utc_now()}
        with self._lock:
            self.data["gaps"].append(gap)
            self.flush()
        log("GAP recorded: %s" % gap)

    def finish(self):
        with self._lock:
            self.data["utc_end"] = utc_now()
            self.flush()


def run_cmd(manifest, name, cmd, log_file, timeout_s, cwd=REPO):
    """Run one lane CLI, streaming stdout+stderr to log_file. Returns rc
    (None on timeout/OSError). Never raises."""
    log_file = Path(log_file)
    log_file.parent.mkdir(parents=True, exist_ok=True)
    log("%s: %s" % (name, " ".join(str(c) for c in cmd)))
    t0 = time.perf_counter()
    rc, note = None, None
    try:
        with open(log_file, "a", encoding="utf-8", errors="replace") as lf:
            lf.write("\n===== %s @ %s =====\n$ %s\n"
                     % (name, utc_now(), " ".join(str(c) for c in cmd)))
            lf.flush()
            proc = subprocess.Popen([str(c) for c in cmd], cwd=str(cwd),
                                    stdout=lf, stderr=subprocess.STDOUT)
            rc = proc.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        note = "timeout after %ds; tree-killed" % timeout_s
        try:
            from common import engine_launch
            engine_launch.kill_tree(proc)
            proc.wait(timeout=30)
        except Exception:
            pass
    except OSError as e:
        note = "launch failed: %r" % e
    wall = time.perf_counter() - t0
    status = "completed" if rc == 0 else \
        ("completed_nonzero" if rc is not None else "failed")
    manifest.record_run(name, cmd, status, rc, wall, log_file, note)
    log("%s: %s (rc=%s, %.0fs)" % (name, status, rc, wall))
    return rc


# ---------------------------------------------------------------------------
# Lane 1 — reference engines
# ---------------------------------------------------------------------------
def _lane1_rows(out_dir):
    p = out_dir / "lane1" / "results.jsonl"
    return results_mod.read_rows(p) if p.exists() else []


def _have_row(rows, test_name, engine, dt):
    return any(r["test"] == test_name and r["engine"] == engine
               and abs(float(r["dt_ms"]) - dt) < 1e-9 for r in rows)


def _sweep_recordings(out_dir):
    """Keep results/ clean: recordings (npz) live under lane1/recordings/."""
    lane1 = out_dir / "lane1"
    rec = lane1 / "recordings"
    rec.mkdir(parents=True, exist_ok=True)
    for p in lane1.glob("*.npz"):
        target = rec / p.name
        if target.exists():
            target.unlink()
        p.rename(target)


def lane1_reference(manifest, out_dir, engines, dts, timeout_s):
    for engine in engines:
        try:
            __import__(engine)
        except ImportError as e:
            for dt in dts:
                manifest.record_gap(1, "python package %r unavailable: %r"
                                    % (engine, e), engine=engine, dt_ms=dt,
                                    test="all")
            continue
        runner = HERE / "lane1" / ("run_%s.py" % engine)
        lane_log = out_dir / "logs" / ("lane1_%s.log" % engine)
        for dt in dts:
            run_cmd(manifest, "lane1-%s-dt%g" % (engine, dt),
                    [sys.executable, runner, "--test", "all",
                     "--dt-ms", str(dt), "--out", out_dir / "lane1"],
                    lane_log, timeout_s)
            # localize: any test still missing gets one individual re-run,
            # then a recorded gap (a mid-loop crash must not cost the rest).
            rows = _lane1_rows(out_dir)
            for tid in LANE1_TESTS:
                tname = scenes.TEST_NAMES[tid]
                if _have_row(rows, tname, engine, dt):
                    continue
                run_cmd(manifest, "lane1-%s-%s-dt%g-retry" % (engine, tid, dt),
                        [sys.executable, runner, "--test", tid,
                         "--dt-ms", str(dt), "--out", out_dir / "lane1"],
                        lane_log, timeout_s)
                if not _have_row(_lane1_rows(out_dir), tname, engine, dt):
                    manifest.record_gap(1, "no result row after retry",
                                        engine=engine, test=tname, dt_ms=dt)
        _sweep_recordings(out_dir)


# ---------------------------------------------------------------------------
# Lane 1 — OmniSim (serial by default; --parallel K runs K engine instances
# concurrently — the engine is multi-instance-safe: auto TCP port in
# [1234,1244] + port-salted tmp/IPC dir, and every run already gets a unique
# OMNISIM_LOG_PATH from common/engine_launch.py)
# ---------------------------------------------------------------------------
def _run_and_score_one(manifest, backend, tid, dt, runner, rec_dir,
                       results_path, lane_log, timeout_s, run_timeout_s,
                       score_lock=None):
    """One (backend, test, dt) engine run + immediate scoring. score_lock
    (parallel mode) serializes the results.jsonl append + fingerprint."""
    engine = "omnisim-%s" % backend
    name = "lane1-omnisim-%s-%s-dt%d" % (backend, tid, dt)
    run_cmd(manifest, name,
            [sys.executable, runner, "--test", tid,
             "--dt-ms", str(dt), "--backend", backend,
             "--out", rec_dir, "--timeout", str(run_timeout_s)],
            lane_log, timeout_s)
    if score_lock is None:
        score_one_omnisim(manifest, tid, dt, backend, engine,
                          rec_dir, results_path)
    else:
        with score_lock:
            score_one_omnisim(manifest, tid, dt, backend, engine,
                              rec_dir, results_path)


def lane1_omnisim(manifest, out_dir, backends, dts, timeout_s, run_timeout_s,
                  parallel=1):
    import score_omnisim  # noqa: F401  (fail early if scoring deps missing)
    rec_dir = out_dir / "lane1" / "recordings"
    rec_dir.mkdir(parents=True, exist_ok=True)
    runner = HERE / "lane1" / "run_omnisim.py"
    results_path = out_dir / "lane1" / "results.jsonl"

    combos = [(backend, tid, dt) for backend in backends
              for tid in LANE1_TESTS for dt in dts]

    if parallel <= 1:
        for backend, tid, dt in combos:
            lane_log = out_dir / "logs" / ("lane1_omnisim_%s.log" % backend)
            _run_and_score_one(manifest, backend, tid, dt, runner, rec_dir,
                               results_path, lane_log, timeout_s,
                               run_timeout_s)
        return

    # Parallel-safety: run_omnisim.py lazily gen_worlds.generate()s a missing
    # world, and generate() writes BOTH backend variants of a (stem, dt) —
    # two concurrent runners at the same (stem, dt) on different backends
    # would race-write the same .wbt. Pre-generate everything serially.
    gen_worlds.generate(dts=dts)

    score_lock = threading.Lock()   # results.jsonl append order + fingerprint

    def one(combo):
        backend, tid, dt = combo
        # per-combo console log: K children appending to one shared per-
        # backend file would interleave their engine-runner output
        lane_log = out_dir / "logs" / ("lane1_omnisim_%s_%s_dt%d.log"
                                       % (backend, tid, dt))
        _run_and_score_one(manifest, backend, tid, dt, runner, rec_dir,
                           results_path, lane_log, timeout_s, run_timeout_s,
                           score_lock=score_lock)

    log("lane1 omnisim matrix: %d combos across %d workers"
        % (len(combos), parallel))
    with ThreadPoolExecutor(max_workers=parallel,
                            thread_name_prefix="w") as ex:
        for f in [ex.submit(one, c) for c in combos]:
            f.result()   # workers never raise by design; surface if one does
    _sort_lane1_omnisim_rows(results_path, backends, dts)


def _sort_lane1_omnisim_rows(results_path, backends, dts):
    """Deterministic results.jsonl after a parallel matrix: completion order
    is scheduling-dependent, so rewrite with the omnisim rows in canonical
    (backend, test, dt) matrix order. Reference-engine rows keep their
    original (serial) order ahead of them; row contents are untouched
    (raw-line rewrite, atomic replace)."""
    if not results_path.exists():
        return
    lines = [ln for ln in
             results_path.read_text(encoding="utf-8").splitlines()
             if ln.strip()]

    def key(line):
        try:
            r = json.loads(line)
        except ValueError:
            return (0, 0, 0, 0)
        eng = r.get("engine", "")
        if not eng.startswith("omnisim-"):
            return (0, 0, 0, 0)          # stable sort keeps original order
        backend = eng.split("-", 1)[1]
        tid = scenes.NAME_TO_TEST.get(r.get("test"), "")
        b = backends.index(backend) if backend in backends else len(backends)
        t = (LANE1_TESTS.index(tid) if tid in LANE1_TESTS
             else len(LANE1_TESTS))
        try:
            d = dts.index(int(r.get("dt_ms")))
        except (TypeError, ValueError):
            d = len(dts)
        return (1, b, t, d)

    lines.sort(key=key)
    tmp = results_path.with_suffix(".jsonl.tmp")
    tmp.write_text("\n".join(lines) + "\n", encoding="utf-8")
    tmp.replace(results_path)


def score_one_omnisim(manifest, tid, dt, backend, engine, rec_dir,
                      results_path):
    """Read the runner's per-run run.json artefacts, verify, adapt + score
    the npz, and append one SPEC row. Any failure = a recorded gap."""
    import score_omnisim
    tname = scenes.TEST_NAMES[tid]
    stems = gen_worlds.TESTS[tid]
    runs = []
    for stem in stems:
        rj = rec_dir / ("%s_dt%d_%s.run.json" % (stem, dt, backend))
        if not rj.exists():
            manifest.record_gap(1, "runner produced no run.json for %s" % stem,
                                engine=engine, test=tname, dt_ms=dt)
            return
        runs.append(json.loads(rj.read_text(encoding="utf-8")))
    bad = [r for r in runs if not r.get("ok")]
    if bad:
        manifest.record_gap(
            1, "run(s) failed verification: %s"
            % "; ".join("%s: %s" % (r["scene"], r.get("problems"))
                        for r in bad),
            engine=engine, test=tname, dt_ms=dt)
        return
    try:
        metrics, notes, extras, adapted = score_omnisim.score_run(
            tid, dt, backend, rec_dir)
    except Exception as e:  # noqa: BLE001 — a scoring crash is a gap
        manifest.record_gap(1, "scoring failed: %r" % e,
                            engine=engine, test=tname, dt_ms=dt)
        return
    walls = [r["wall_ms_per_step"] for r in runs
             if isinstance(r.get("wall_ms_per_step"), (int, float))]
    deviations = []
    for r in runs:
        for d in r.get("deviations", []):
            if d not in deviations:
                deviations.append(d)
    deviations += notes
    # backend attribution: fingerprint against the first run's engine log so
    # the row's machine block carries the physics/solver verdict of THIS run
    engine_log = rec_dir / ("%s_dt%d_%s.engine.log" % (stems[0], dt, backend))
    machine = results_mod.machine_fingerprint(
        engine_log_path=str(engine_log) if engine_log.exists() else None) \
        if engine_log.exists() else None
    results_mod.write_row(
        results_path, test=tname, engine=engine, dt_ms=dt, metrics=metrics,
        wall_ms_per_step=(statistics.mean(walls) if walls else 0.0),
        steps=min(r.get("steps", 0) for r in runs),
        sim_seconds=max(r.get("sim_seconds") or 0.0 for r in runs),
        deviations=deviations, machine=machine)
    log("scored %s %s dt=%d: %s"
        % (engine, tname, dt,
           {k: (round(v, 6) if isinstance(v, float) else v)
            for k, v in metrics.items()}))


def retry_lane1_gaps(manifest, out_dir, run_timeout_s):
    """Re-run the lane-1 omnisim (test, dt, backend) combos recorded as gaps,
    with the calibrated 8-attempt launch margin (physics_oracle/determinism).
    A combo that fails AGAIN is kept as a permanent gap (marked retried); a
    resolved combo's gap entry is removed and the row is scored+appended."""
    import os as _os
    rec_dir = out_dir / "lane1" / "recordings"
    runner = HERE / "lane1" / "run_omnisim.py"
    results_path = out_dir / "lane1" / "results.jsonl"
    lane_log = out_dir / "logs" / "lane1_omnisim_retry.log"

    todo = []
    for g in list(manifest.data["gaps"]):
        eng = g.get("engine", "")
        if g.get("lane") == 1 and eng.startswith("omnisim-") \
                and not g.get("retried"):
            backend = eng.split("-", 1)[1]
            tid = scenes.NAME_TO_TEST.get(g.get("test"))
            if tid is None or g.get("dt_ms") is None:
                continue
            todo.append((g, tid, int(g["dt_ms"]), backend, eng))
    log("gap-retry pass: %d lane-1 omnisim gaps to retry" % len(todo))

    env_note = {"OMNIBENCH_LAUNCH_ATTEMPTS": "8"}
    _os.environ.update(env_note)  # inherited by the runner subprocesses
    for g, tid, dt, backend, eng in todo:
        name = "retry-lane1-omnisim-%s-%s-dt%d" % (backend, tid, dt)
        run_cmd(manifest, name,
                [sys.executable, runner, "--test", tid, "--dt-ms", str(dt),
                 "--backend", backend, "--out", rec_dir,
                 "--timeout", str(run_timeout_s)],
                lane_log, timeout_s=7200)
        # re-check: same verification as the main pass
        n_gaps_before = len(manifest.data["gaps"])
        manifest.data["gaps"].remove(g)
        manifest.flush()
        score_one_omnisim(manifest, tid, dt, backend, eng, rec_dir,
                          results_path)
        if len(manifest.data["gaps"]) >= n_gaps_before:
            # score_one_omnisim re-recorded a gap -> permanent, mark it
            manifest.data["gaps"][-1]["retried"] = True
            manifest.data["gaps"][-1]["reason"] += \
                " [PERMANENT: failed again on the 8-attempt retry pass]"
            manifest.flush()
            log("PERMANENT gap: %s %s dt=%d" % (eng, tid, dt))
        else:
            log("gap resolved on retry: %s %s dt=%d" % (eng, tid, dt))


# ---------------------------------------------------------------------------
# Lanes 2 / 3
# ---------------------------------------------------------------------------
def lane2(manifest, out_dir, tiers, batches, timeout_s):
    out = out_dir / "lane2" / "throughput.jsonl"
    out.parent.mkdir(parents=True, exist_ok=True)
    cmd = [sys.executable, HERE / "lane2" / "run_throughput.py",
           "--tiers", tiers, "--out", out, "--batch"] + \
        [str(b) for b in batches]
    rc = run_cmd(manifest, "lane2-throughput", cmd,
                 out_dir / "logs" / "lane2.log", timeout_s)
    if not out.exists():
        manifest.record_gap(2, "no throughput.jsonl produced (rc=%s)" % rc,
                            engine="omnisim-newton", test="throughput_go2")


def lane3(manifest, out_dir, backends, timeout_s):
    l3 = out_dir / "lane3"
    l3.mkdir(parents=True, exist_ok=True)
    logs = out_dir / "logs"
    for backend in backends:
        out = l3 / "determinism.jsonl"
        rc = run_cmd(manifest, "lane3-determinism-%s" % backend,
                     [sys.executable, HERE / "lane3" / "determinism.py",
                      "--backend", backend, "--out", out],
                     logs / "lane3_determinism.log", timeout_s)
        rows = results_mod.read_rows(out) if out.exists() else []
        if not any(r["engine"] == "omnisim-%s" % backend for r in rows):
            manifest.record_gap(3, "determinism produced no rows (rc=%s)" % rc,
                                engine="omnisim-%s" % backend,
                                test="determinism")
    out = l3 / "parity.jsonl"
    rc = run_cmd(manifest, "lane3-parity",
                 [sys.executable, HERE / "lane3" / "parity.py", "--out", out],
                 logs / "lane3_parity.log", timeout_s)
    if not out.exists():
        manifest.record_gap(3, "parity produced no rows (rc=%s)" % rc,
                            engine="omnisim-newton", test="parity_structural")
    out = l3 / "driveability.jsonl"
    rc = run_cmd(manifest, "lane3-driveability",
                 [sys.executable, HERE / "lane3" / "driveability.py",
                  "--out", out],
                 logs / "lane3_driveability.log", timeout_s)
    if not out.exists():
        manifest.record_gap(3, "driveability produced no rows (rc=%s)" % rc,
                            engine="omnisim", test="driveability")


def lane4(manifest, out_dir, timeout_s, probe_timeout, envelope=True):
    """Lane 4 — capability coverage, resource envelope, CPU-only.

    Three sub-stages, each its own results file so a failure in one does not
    take the others with it. Coverage is the one that must always run: it is
    the only lane in the suite that answers "what can this thing simulate",
    and every other lane's number is conditional on the answer.
    """
    l4 = out_dir / "lane4"
    l4.mkdir(parents=True, exist_ok=True)
    logs = out_dir / "logs"

    out = l4 / "coverage.jsonl"
    rc = run_cmd(manifest, "lane4-coverage",
                 [sys.executable, HERE / "lane4" / "run_coverage.py",
                  "--out", out, "--timeout", str(probe_timeout)],
                 logs / "lane4_coverage.log", timeout_s)
    rows = results_mod.read_rows(out) if out.exists() else []
    if not any(r["test"] == "capability_summary" for r in rows):
        manifest.record_gap(4, "coverage produced no summary row (rc=%s)" % rc,
                            engine="omnisim-newton", test="capability_summary")

    out = l4 / "cpu_only.jsonl"
    rc = run_cmd(manifest, "lane4-cpu-only",
                 [sys.executable, HERE / "lane4" / "cpu_only.py", "--out", out],
                 logs / "lane4_cpu_only.log", timeout_s)
    if not out.exists():
        manifest.record_gap(4, "cpu-only produced no rows (rc=%s)" % rc,
                            engine="omnisim-newton",
                            test="cpu_only_no_cuda_visible")

    if envelope:
        out = l4 / "envelope.jsonl"
        rc = run_cmd(manifest, "lane4-envelope",
                     [sys.executable, HERE / "lane4" / "run_envelope.py",
                      "--out", out],
                     logs / "lane4_envelope.log", timeout_s)
        if not out.exists():
            manifest.record_gap(4, "envelope produced no rows (rc=%s)" % rc,
                                engine="omnisim-newton", test="envelope_boxes")


def _findings_lane4(cov_rows, env_rows, cpu_rows):
    """Capability findings, in the order a reader needs them.

    `broken` first and loudest: it is the verdict that a load-only smoke test
    reports as PASS, so it is the one nothing else in the suite can surface.
    `absent` is NEVER a finding — no cloth solver is a scope statement, not a
    defect — and `inconclusive` is reported separately because an instrument
    failure is not evidence about the engine.
    """
    out = []
    for r in cov_rows:
        if not r["test"].startswith("capability:"):
            continue
        m = r.get("metrics") or {}
        pid = r["test"].split(":", 1)[1]
        if m.get("verdict") == "broken":
            out.append("[lane4] %s: BROKEN -- %s" % (pid, m.get("note")))
        elif m.get("verdict") == "inconclusive":
            out.append("[lane4] %s: no result (INSTRUMENT failure, not an "
                       "engine finding) -- %s" % (pid, m.get("note")))
        if m.get("doc_agrees") is False:
            out.append("[lane4] %s: DOC MISMATCH -- documented as %r, measured "
                       "%r. A feature listed broken that now works misleads as "
                       "much as the reverse."
                       % (pid, m.get("documented_as"), m.get("verdict")))
    for r in env_rows:
        m = r.get("metrics") or {}
        if r["test"].startswith("envelope_summary_"):
            if m.get("first_n_with_constraint_overflow") is not None:
                out.append(
                    "[lane4] envelope %s/%s: SILENT constraint-buffer overflow "
                    "from N=%s (mujoco_warp drops rows past njmax with a clean "
                    "log and exit 0)"
                    % (m.get("scene"), m.get("solver"),
                       m.get("first_n_with_constraint_overflow")))
            elif (m.get("constraint_telemetry_sampled")
                  and not m.get("cliff_detector_validated")):
                # An instrument that never fired has not been shown to work.
                # Reporting its silence as a pass is how a dead check survives.
                out.append(
                    "[lane4] envelope %s/%s: no constraint overflow seen, but "
                    "the detector was never observed to FIRE -- treat as "
                    "UNVALIDATED, not as a pass (see the row's deviations)"
                    % (m.get("scene"), m.get("solver")))
        elif m.get("measured") is False:
            out.append("[lane4] envelope %s n=%s: no measurement survived"
                       % (r["test"], m.get("n")))
    for r in cpu_rows:
        m = r.get("metrics") or {}
        if m.get("verdict") not in (None, "works"):
            out.append("[lane4] cpu-only (no CUDA visible): %s -- %s"
                       % (str(m.get("verdict")).upper(), m.get("note")))
    return out


def translation_audit(manifest, out_dir, timeout_s):
    """Audit the .wbt -> mjModel translation for the scenes lane 1 just measured.

    Lane 1 scores TRAJECTORIES against analytic ground truth. It cannot see a
    world whose declared physics never reached the solver -- the scene simply
    measures something else, accurately. That is not hypothetical: the first
    corpus run found 10 of 13 lane-1 scenes declaring a friction the model never
    received (2026-08-09), and it is the class of defect the deleted ODE arm used
    to catch by construction.

    Run as its own stage so a campaign cannot report lane-1 numbers without also
    reporting whether the worlds behind them describe themselves.
    """
    out = out_dir / "lane1" / "translation_audit.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    rc = run_cmd(manifest, "lane1-translation-audit",
                 [sys.executable, HERE / "lane1" / "translation_audit.py",
                  "--lane1", "--quiet", "--json", out],
                 out_dir / "logs" / "lane1_translation_audit.log", timeout_s)
    if not out.exists():
        manifest.record_gap(1, "translation audit produced no report (rc=%s)" % rc,
                            engine="omnisim-newton", test="translation_audit")
    return out


def _findings_translation(report_path):
    """Raise every translation ERROR as a campaign finding.

    An ERROR here means the model the solver stepped CONTRADICTS the world file
    it was built from, so any lane-1 row for that scene measures a world other
    than the one on disk -- and a third party loading it gets different physics.
    Reported per (check, world) so one systemic defect across N scenes is
    visible as N rows rather than collapsed into one.
    """
    out = []
    try:
        with open(report_path, "r", encoding="utf-8") as f:
            reports = json.load(f)
    except (OSError, ValueError):
        return out
    for rep in reports:
        world = Path(rep.get("world", "?")).name
        for f_ in rep.get("findings", []):
            if f_.get("severity") != "ERROR":
                continue
            out.append("[translation] %s: %s — %s"
                       % (world, f_.get("check"), f_.get("message")))
    if out:
        out.append("[translation] %d contract violation(s): the model the solver "
                   "stepped does not match the .wbt it was built from. Lane-1 rows "
                   "for these scenes measure a world other than the one on disk."
                   % (len(out),))
    return out


# ---------------------------------------------------------------------------
# summary.md
# ---------------------------------------------------------------------------
def _fmt(v):
    if v is None:
        return "—"
    if isinstance(v, bool):
        return str(v)
    if isinstance(v, float):
        if v == 0:
            return "0"
        if abs(v) >= 1000 or abs(v) < 1e-3:
            return "%.3g" % v
        return "%.4g" % v
    return str(v)


def _findings_lane1(rows):
    out = []
    for r in rows:
        checks = THRESHOLDS.get(r["test"], [])
        m = r.get("metrics") or {}
        for key, cmp, thr in checks:
            v = m.get(key)
            if not isinstance(v, (int, float)) or isinstance(v, bool):
                continue
            if (cmp == ">" and v > thr) or (cmp == "<" and v < thr):
                out.append("[lane1] %s %s dt=%g ms: %s=%s (%s %g)"
                           % (r["test"], r["engine"], r["dt_ms"], key,
                              _fmt(v), cmp, thr))
    return out


def _findings_lane2(rows):
    out = []
    for r in rows:
        m = r.get("metrics") or {}
        st = m.get("status")
        if st in ("dropped", "not_run"):
            out.append("[lane2] %s tier=%s batch=%s: status=%s (%s)"
                       % (r["engine"], m.get("tier"), m.get("batch"), st,
                          m.get("reason", "")))
        if m.get("idle_guard_ok") is False:
            out.append("[lane2] %s tier=%s batch=%s: IDLE-SCENE GUARD FAILED"
                       % (r["engine"], m.get("tier"), m.get("batch")))
    return out


def _findings_attribution(rows):
    """Flag rows whose PRODUCING BACKEND was never verified.

    A row labelled `omnisim-unverified` is a measurement whose engine could not
    be proven from the `.newton.json` verdict sidecar. That label exists because
    the old fallback was `omnisim-ode`, which post-bdc02139 named an engine that
    does not exist -- an unverifiable row read as a legacy-backend measurement.
    Unverified is honest but it is still NOT a publishable attribution, so it
    surfaces as a finding rather than sitting quietly in `deviations`.
    """
    out = []
    seen = set()
    for r in rows:
        if r.get("engine") != "omnisim-unverified":
            continue
        why = next((d for d in (r.get("deviations") or [])
                    if isinstance(d, str) and d.startswith("engine=omnisim-unverified")),
                   "no reason recorded")
        key = (r.get("test"), why)
        if key in seen:
            continue
        seen.add(key)
        out.append("[attribution] %s: engine UNVERIFIED — do not publish this row "
                   "as a backend result. %s" % (r.get("test"), why))
    return out


def _findings_lane3(det_rows, par_rows, drv_rows):
    out = []
    for r in det_rows:
        g = (r.get("metrics") or {}).get("grade")
        if g != "bitwise":
            out.append("[lane3] %s %s: determinism grade=%s (!= bitwise, "
                       "max|d|=%s, first_div_step=%s)"
                       % (r["test"], r["engine"], g,
                          _fmt((r["metrics"] or {}).get("max_abs_dev")),
                          (r["metrics"] or {}).get("first_div_step")))
    for r in par_rows:
        m = r.get("metrics") or {}
        if m.get("pass") is not True:
            out.append("[lane3] parity_structural (%s): pass=%s "
                       "real_physics_gaps=%s"
                       % ((r.get("deviations") or ["?"])[0], m.get("pass"),
                          m.get("real_physics_gaps")))
    for r in drv_rows:
        m = r.get("metrics") or {}
        if r["test"] == "driveability_summary":
            if m.get("probes_passed") != m.get("probes_total"):
                out.append("[lane3] driveability: %s/%s probes passed"
                           % (m.get("probes_passed"), m.get("probes_total")))
        elif m.get("pass") is False:
            out.append("[lane3] %s: FAIL (latency %s ms)"
                       % (r["test"], _fmt(m.get("latency_ms"))))
    return out


def write_summary(out_dir, manifest):
    lane1_rows = _lane1_rows(out_dir)
    p2 = out_dir / "lane2" / "throughput.jsonl"
    lane2_rows = results_mod.read_rows(p2) if p2.exists() else []
    l3 = out_dir / "lane3"
    det = results_mod.read_rows(l3 / "determinism.jsonl") \
        if (l3 / "determinism.jsonl").exists() else []
    par = results_mod.read_rows(l3 / "parity.jsonl") \
        if (l3 / "parity.jsonl").exists() else []
    drv = results_mod.read_rows(l3 / "driveability.jsonl") \
        if (l3 / "driveability.jsonl").exists() else []

    fp = results_mod.machine_fingerprint()
    mach = fp.get("machine") or {}
    binary = fp.get("binary") or {}
    lines = []
    w = lines.append
    w("# OmniBench summary — %s" % (mach.get("id", "unknown-machine")))
    w("")
    w("Generated %s by run_all.py (suite %s)." % (utc_now(), results_mod.SUITE))
    w("")
    w("| machine | value |")
    w("|---|---|")
    w("| machine id | %s |" % mach.get("id"))
    w("| host | %s |" % mach.get("host"))
    w("| GPU | %s |" % fp.get("gpu"))
    w("| CPU | %s (%s cores) |" % (mach.get("cpu"), mach.get("cores")))
    w("| OS | %s |" % fp.get("os"))
    w("| python | %s (numpy %s) |" % (fp.get("python"), fp.get("numpy")))
    w("| engine binary sha256 | %s |" % (binary.get("sha256") or "?"))
    w("")
    w("All numbers below are attributable to THIS machine only "
      "(SPEC honesty rule 4).")
    w("")

    # ---------------- lane 1 ----------------
    if lane1_rows:
        w("## Lane 1 — physics correctness (analytic ground truth)")
        w("")
        engines = sorted({r["engine"] for r in lane1_rows})
        for tid in LANE1_TESTS:
            tname = scenes.TEST_NAMES[tid]
            trows = [r for r in lane1_rows if r["test"] == tname]
            if not trows:
                continue
            metrics = LANE1_METRICS[tname]
            w("### %s `%s`" % (tid, tname))
            w("")
            w("| engine | dt (ms) | " + " | ".join(metrics)
              + " | wall ms/step |")
            w("|---|---|" + "---|" * (len(metrics) + 1))
            for engine in engines:
                for dt in DT_SWEEP:
                    row = next((r for r in trows if r["engine"] == engine
                                and abs(float(r["dt_ms"]) - dt) < 1e-9), None)
                    if row is None:
                        w("| %s | %g | " % (engine, dt)
                          + " | ".join("GAP" for _ in metrics) + " | — |")
                        continue
                    m = row.get("metrics") or {}
                    w("| %s | %g | " % (engine, dt)
                      + " | ".join(_fmt(m.get(k)) for k in metrics)
                      + " | %s |" % _fmt(row.get("wall_ms_per_step")))
            w("")
            # deviations, deduped per engine across the dt sweep
            devs = {}
            for r in trows:
                for d in r.get("deviations", []):
                    devs.setdefault(r["engine"], [])
                    if d not in devs[r["engine"]]:
                        devs[r["engine"]].append(d)
            if devs:
                w("<details><summary>deviations (%s)</summary>" % tname)
                w("")
                for engine in sorted(devs):
                    w("- **%s**:" % engine)
                    for d in devs[engine]:
                        w("  - %s" % d)
                w("")
                w("</details>")
                w("")

    # ---------------- lane 2 ----------------
    if lane2_rows:
        w("## Lane 2 — throughput (three-tier, Go2)")
        w("")
        w("| engine | tier | batch | env-steps/s | ms/control-step | status |")
        w("|---|---|---|---|---|---|")
        for r in lane2_rows:
            m = r.get("metrics") or {}
            w("| %s | %s | %s | %s | %s | %s |"
              % (r["engine"], m.get("tier"), m.get("batch"),
                 _fmt(m.get("env_steps_per_s")),
                 _fmt(m.get("ms_per_control_step")),
                 m.get("status", "ok")))
        w("")
        raws = {m.get("batch"): m.get("env_steps_per_s") for m in
                (r["metrics"] for r in lane2_rows
                 if r["engine"] == "mujoco-warp-raw"
                 and r["metrics"].get("tier") == "sim_only"
                 and r["metrics"].get("status") not in ("dropped", "not_run"))}
        newts = {m.get("batch"): m.get("env_steps_per_s") for m in
                 (r["metrics"] for r in lane2_rows
                  if r["engine"] == "omnisim-newton"
                  and r["metrics"].get("tier") == "sim_only"
                  and r["metrics"].get("status") not in ("dropped", "not_run"))}
        common_b = sorted(set(raws) & set(newts))
        if common_b:
            w("OmniSim embedded-solver / raw mujoco-warp sim_only ratio "
              "(same batch): "
              + ", ".join("batch %s: %.3gx" % (b, raws[b] / newts[b])
                          for b in common_b if newts[b]))
            w("")

    # ---------------- lane 3 ----------------
    if det or par or drv:
        w("## Lane 3 — determinism / parity / driveability")
        w("")
    if det:
        w("### 3a determinism")
        w("")
        w("| test | engine | grade | max abs dev | first div step | steps |")
        w("|---|---|---|---|---|---|")
        for r in det:
            m = r.get("metrics") or {}
            w("| %s | %s | %s | %s | %s | %s |"
              % (r["test"], r["engine"], m.get("grade"),
                 _fmt(m.get("max_abs_dev")), m.get("first_div_step"),
                 m.get("compared_steps")))
        w("")
    if par:
        w("### 3b train==deploy structural parity")
        w("")
        w("| config | real physics gaps | repr diffs | pass |")
        w("|---|---|---|---|")
        for r in par:
            m = r.get("metrics") or {}
            cfg = next((d[len("config: "):] for d in r.get("deviations", [])
                        if d.startswith("config: ")), "?")
            w("| %s | %s | %s | %s |"
              % (cfg, m.get("real_physics_gaps"),
                 m.get("representational_diffs"), m.get("pass")))
        w("")
    if drv:
        w("### 3c agent driveability")
        w("")
        w("| probe | pass | latency (ms) |")
        w("|---|---|---|")
        for r in drv:
            m = r.get("metrics") or {}
            if r["test"] == "driveability_summary":
                continue
            w("| %s | %s | %s |"
              % (r["test"].replace("driveability_", ""), m.get("pass"),
                 _fmt(m.get("latency_ms"))))
        summ = next((r for r in drv if r["test"] == "driveability_summary"),
                    None)
        if summ:
            m = summ["metrics"]
            w("")
            w("**Score: %s/%s = %s** (engine %s)"
              % (m.get("probes_passed"), m.get("probes_total"),
                 m.get("score"), summ["engine"]))
        w("")

    # ---------------- findings ----------------
    l4cov = out_dir / "lane4" / "coverage.jsonl"
    l4env = out_dir / "lane4" / "envelope.jsonl"
    l4cpu = out_dir / "lane4" / "cpu_only.jsonl"
    cov = results_mod.read_rows(l4cov) if l4cov.exists() else []
    envl = results_mod.read_rows(l4env) if l4env.exists() else []
    cpu = results_mod.read_rows(l4cpu) if l4cpu.exists() else []
    findings = (_findings_lane1(lane1_rows) + _findings_lane2(lane2_rows)
                + _findings_lane3(det, par, drv)
                + _findings_lane4(cov, envl, cpu)
                + _findings_attribution(lane1_rows + lane2_rows + det + par + drv)
                + _findings_translation(out_dir / "lane1" / "translation_audit.json"))
    w("## FINDINGS (auto: sanity-threshold breaches)")
    w("")
    if findings:
        for f in findings:
            w("- %s" % f)
    else:
        w("- none")
    w("")
    gaps = manifest.data.get("gaps", [])
    w("## Gaps (recorded in MANIFEST.json)")
    w("")
    if gaps:
        for g in gaps:
            w("- lane%s %s %s dt=%s: %s"
              % (g.get("lane"), g.get("engine", ""), g.get("test", ""),
                 g.get("dt_ms", "—"), g.get("reason")))
    else:
        w("- none")
    w("")

    (out_dir / "summary.md").write_text("\n".join(lines), encoding="utf-8")
    log("summary.md written (%d findings, %d gaps)"
        % (len(findings), len(gaps)))


# ---------------------------------------------------------------------------
def ensure_gitignore():
    gi = HERE / "results" / ".gitignore"
    if not gi.exists():
        gi.parent.mkdir(parents=True, exist_ok=True)
        gi.write_text(
            "# results/ is gitignored except committed campaign summaries\n"
            "# (SPEC.md): only jsonl rows, summary.md and MANIFEST.json are\n"
            "# committable; recordings/logs/worlds stay local.\n"
            "*\n"
            "!*/\n"
            "!.gitignore\n"
            "!*.jsonl\n"
            "!summary.md\n"
            "!MANIFEST.json\n"
            "recordings/\n"
            "logs/\n",
            encoding="utf-8")


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lanes", default="1,2,3,4")
    ap.add_argument("--engines", default="mujoco,pybullet,omnisim")
    # RETIRED 2026-08-08: the default used to be "ode,newton". src/ode was
    # DELETED (commit bdc02139) -- there is one backend, so asking for "ode"
    # is an error, not a lane.
    ap.add_argument("--backends", default="newton",
                    help="OmniSim physics backends to run. 'newton' is the "
                         "only value; 'ode' is refused (src/ode deleted, "
                         "bdc02139).")
    ap.add_argument("--dt-ms", default="all",
                    help="'all' or comma list from %s" % DT_SWEEP)
    ap.add_argument("--out", type=Path, default=None,
                    help="default results/<machine_id>/<utc-date>/ under "
                         "tests/benchmarks/omnibench/")
    ap.add_argument("--skip-translation-audit", action="store_true",
                    help="skip the .wbt->mjModel contract audit that follows "
                         "the lane-1 matrix (it costs one engine load per "
                         "scene; skipping it means the campaign cannot say "
                         "whether its worlds describe themselves)")
    ap.add_argument("--lane1-run-timeout", type=float, default=300.0,
                    help="wall seconds per omnisim engine launch")
    ap.add_argument("--parallel", type=int, default=None,
                    help="concurrent engine instances for the lane-1 OmniSim "
                         "matrix (default 4 on Linux, 1 elsewhere; max 24). "
                         "The cap USED to be 8 'because the auto-port range "
                         "[1234,1244] is 11 wide' -- that range has been 61 "
                         "slots (PORT_SCAN_SPAN=60, OmTcpServer.cpp:87) since "
                         "66f0c378, so ports are no longer the binding "
                         "constraint. The real limits are the controller-"
                         "connect launch race (mitigated by "
                         "OMNIBENCH_LAUNCH_ATTEMPTS with growing settle) and "
                         "the fact that KILLED engines keep holding their "
                         "ports -- which is why the cap is 24 and not 60: the "
                         "range has to absorb concurrent engines PLUS the "
                         "residue of every engine a timeout killed. Raise "
                         "deliberately and watch for launch-race gaps in "
                         "MANIFEST.json. Windows defaults to serial: the race "
                         "worsens under parallelism and burns retry attempts")
    ap.add_argument("--overlap-lanes", action="store_true",
                    help="launch lane 2 (GPU) concurrently with lane 1's "
                         "CPU-bound OmniSim matrix (lane-2 outputs and tier "
                         "C's per-tag OMNISIM_LOG_PATH are disjoint from "
                         "lane 1's per-run paths)")
    ap.add_argument("--lane4-probe-timeout", type=float, default=200.0,
                    help="per-capability-probe wall-clock cap, seconds")
    ap.add_argument("--skip-lane4-envelope", action="store_true",
                    help="run capability coverage + cpu-only but skip the "
                         "scaling sweep (the long half of lane 4)")
    ap.add_argument("--lane2-tiers", default="raw,A,B,C")
    ap.add_argument("--lane2-batches", type=int, nargs="+",
                    default=[256, 1024, 4096])
    ap.add_argument("--summary-only", action="store_true",
                    help="just (re)write summary.md from an existing out dir")
    ap.add_argument("--retry-gaps", action="store_true",
                    help="re-run the lane-1 omnisim gaps recorded in an "
                         "existing MANIFEST.json (8-attempt launch margin), "
                         "then rewrite summary.md; second failures become "
                         "permanent gaps")
    args = ap.parse_args(argv)

    if args.parallel is None:
        args.parallel = 4 if sys.platform.startswith("linux") else 1
    if not 1 <= args.parallel <= 24:
        ap.error("--parallel must be in [1, 24]. The engine auto-port range is "
                 "[1234, 1294] (PORT_SCAN_SPAN=60), so 24 concurrent engines "
                 "leave 37 slots for the harness, lane-2 tier C, and the ports "
                 "still held by engines a timeout killed -- which is the "
                 "failure 66f0c378 widened the span for. Above 24, expect "
                 "launch-race gaps rather than a port error.")

    lanes = {s.strip() for s in args.lanes.split(",") if s.strip()}
    engines = [s.strip() for s in args.engines.split(",") if s.strip()]
    backends = [s.strip() for s in args.backends.split(",") if s.strip()]
    dead = [b for b in backends if b != "newton"]
    if dead:
        ap.error("--backends %s: ODE was DELETED (src/ode, commit bdc02139) and "
                 "Newton is the only physics backend. Lane 1's ODE arm was its "
                 "cross-backend reference and is retired; what the lane still "
                 "measures is Newton vs the PER-SCENE ANALYTIC ground truth "
                 "encoded in common/scenes.py -- score.py reads no engine's "
                 "numbers as a reference at all -- with mujoco and pybullet as "
                 "the surviving external cross-checks (--engines). There is NO "
                 "frozen file of lane-1 ODE values: tests/goldens/"
                 "ode_oracle_goldens.json is DEVICE-parity evidence (raycast, "
                 "inertia, weld, touch_force, receiver/lightsensor/radar "
                 "occlusion, kinematic) and contains zero T1-T7 scenes. The ODE "
                 "arm's own 105 lane-1 rows survive only as history under "
                 "results*/ and are permanently unrepeatable. "
                 "Use --backends newton." % ",".join(dead))
    dts = DT_SWEEP if args.dt_ms == "all" else \
        [int(s) for s in args.dt_ms.split(",") if s.strip()]
    for d in dts:
        if d not in DT_SWEEP:
            ap.error("dt %s not in the SPEC sweep %s" % (d, DT_SWEEP))

    fp = results_mod.machine_fingerprint()
    machine_id = (fp.get("machine") or {}).get("id", "unknown-machine")
    out_dir = args.out or (HERE / "results" / machine_id
                           / datetime.datetime.now(datetime.timezone.utc)
                           .strftime("%Y-%m-%d"))
    out_dir = Path(out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    ensure_gitignore()
    log("machine %s -> %s" % (machine_id, out_dir))

    manifest_path = out_dir / "MANIFEST.json"
    if (args.summary_only or args.retry_gaps) and manifest_path.exists():
        manifest = Manifest.load(manifest_path)
    else:
        manifest = Manifest(manifest_path, args)

    if args.summary_only:
        write_summary(out_dir, manifest)
        return 0
    if args.retry_gaps:
        retry_lane1_gaps(manifest, out_dir, args.lane1_run_timeout)
        write_summary(out_dir, manifest)
        manifest.finish()
        return 0

    t0 = time.perf_counter()
    lane2_wanted = "2" in lanes and "omnisim" in engines
    lane1_omnisim_wanted = "1" in lanes and "omnisim" in engines
    lane2_thread = None
    if args.overlap_lanes and lane2_wanted and lane1_omnisim_wanted:
        # lane 2 is GPU-bound (in-process warp tiers + its own engine child
        # for tier C, per-tag OMNISIM_LOG_PATH); lane 1's matrix is CPU-bound
        # engine instances — overlap them, join before lane 3 / summary.
        lane2_thread = threading.Thread(
            target=lane2, name="lane2",
            args=(manifest, out_dir, args.lane2_tiers, args.lane2_batches,
                  10800))
        lane2_thread.start()
        log("overlap-lanes: lane 2 running concurrently with lane 1")
    if "1" in lanes:
        ref = [e for e in engines if e in REF_ENGINES]
        if ref:
            lane1_reference(manifest, out_dir, ref, dts, timeout_s=3600)
        if "omnisim" in engines:
            lane1_omnisim(manifest, out_dir, backends, dts,
                          timeout_s=7200,
                          run_timeout_s=args.lane1_run_timeout,
                          parallel=args.parallel)
            # Numbers and the worlds behind them travel together: a lane-1 row
            # is only about the scene on disk if the translation held.
            if not args.skip_translation_audit:
                translation_audit(manifest, out_dir, timeout_s=3600)
    if lane2_thread is not None:
        lane2_thread.join()
    elif lane2_wanted:
        lane2(manifest, out_dir, args.lane2_tiers, args.lane2_batches,
              timeout_s=10800)
    if "3" in lanes and "omnisim" in engines:
        lane3(manifest, out_dir, backends, timeout_s=10800)
    if "4" in lanes and "omnisim" in engines:
        lane4(manifest, out_dir, timeout_s=10800,
              probe_timeout=args.lane4_probe_timeout,
              envelope=not args.skip_lane4_envelope)

    write_summary(out_dir, manifest)
    manifest.finish()
    wall = time.perf_counter() - t0
    log("suite done in %.1f min; %d runs, %d gaps -> %s"
        % (wall / 60.0, len(manifest.data["runs"]),
           len(manifest.data["gaps"]), out_dir))
    return 0


if __name__ == "__main__":
    sys.exit(main())
