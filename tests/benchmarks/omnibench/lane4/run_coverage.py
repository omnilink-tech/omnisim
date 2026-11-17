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

"""OmniBench lane 4a — run the capability coverage probes.

    python tests/benchmarks/omnibench/lane4/run_coverage.py
    python .../run_coverage.py --probes joint. --out /tmp/cov.jsonl
    python .../run_coverage.py --family device --keep

One SPEC row per probe. `metrics.verdict` is one of works / degraded / broken /
absent / inconclusive (capabilities.py documents what each means and why
`broken` vs `absent` is the distinction the lane exists for).

Three rules this runner enforces, each because the alternative produces a
number that looks like evidence and is not:

1. **A probe whose instrument failed is `inconclusive`, never `broken`.**
   An engine crash, a missing recording, a controller that never started —
   none of those are statements about the capability. They are reported
   separately and excluded from the coverage score.
2. **The backend is proven per probe, not assumed.** Every row carries the
   engine label from the `.newton.json` verdict sidecar of ITS OWN run; a run
   that cannot be attributed is published as `omnisim-unverified` with the
   reason in `deviations` (common/engine_launch.py owns that rule).
3. **The documentation is audited against the measurement.** A probe carrying
   `documented_as` raises a finding when the two disagree — in EITHER
   direction. A feature listed as broken that now works is as much a defect in
   the record as one listed as working that is broken, and the second kind is
   what sends an agent down a three-hour dead end.
"""
from __future__ import annotations

import argparse
import fnmatch
import json
import os
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
OMNIBENCH = HERE.parent
REPO = HERE.parents[3]
for p in (str(HERE), str(OMNIBENCH), str(REPO)):
    if p not in sys.path:
        sys.path.insert(0, p)

import capabilities as caps                      # noqa: E402
import gen_worlds                                # noqa: E402
from common import engine_launch                 # noqa: E402
from common import results as results_mod        # noqa: E402

DEFAULT_OUT = HERE / "results" / "coverage.jsonl"
SUITE_TEST_PREFIX = "capability"


def log(msg):
    # Fold to ASCII before printing. The registry's prose legitimately carries
    # typographic punctuation, but a decorative glyph on stdout has already
    # crashed a cp1252 console in this suite (lane 1R) -- and a runner that
    # dies mid-campaign reads as an engine failure, not as an encoding bug.
    print(gen_worlds._ascii(str(msg)), flush=True)


# ---------------------------------------------------------------------------
# engine log diagnostics
# ---------------------------------------------------------------------------
def read_diagnostics(log_path):
    """ERROR/WARNING lines from the engine's own log for one run."""
    out = []
    try:
        with open(log_path, "r", encoding="utf-8", errors="replace") as f:
            for line in f:
                s = line.strip()
                if not s:
                    continue
                up = s.upper()
                if "ERROR" in up or "WARNING" in up:
                    out.append(s[:400])
    except OSError:
        pass
    return out


def read_log_capture(log_path, needles, limit=40):
    """Engine log lines containing any of `needles`, AT ANY SEVERITY.

    Separate from read_diagnostics() on purpose: that one keeps ERROR/WARNING
    only, and the lines a particle probe needs are `OmLog::info` --
    "Cloth 'DEF SHEET Cloth': registered 441 particles (20 x 20 cells) at
    Newton particle offset 0." A probe declaring `log_capture` gets these
    handed to its assertion under the recording key `engine_log`; a probe that
    declares nothing reads no file and pays nothing.

    `limit` bounds what a chatty per-step log can push into a result row.
    """
    if not needles:
        return []
    out = []
    try:
        with open(log_path, "r", encoding="utf-8", errors="replace") as f:
            for line in f:
                s = line.strip()
                if s and any(n in s for n in needles):
                    out.append(s[:400])
                    if len(out) >= limit:
                        break
    except OSError:
        pass
    return out


# ---------------------------------------------------------------------------
# one probe
# ---------------------------------------------------------------------------
def run_probe(probe, binpath, work_dir, timeout_s, keep=False, override=None):
    """Launch the probe's world and return (verdict, extras) where extras
    carries everything the row needs: engine label, timings, diagnostics.

    `override` is the revert-hatch escape (see --env). It is applied LAST, so
    it beats both the scrubbed operator environment and the probe's own `env`,
    and it is recorded in the row's `deviations` so a control run can never be
    read as an ordinary measurement.
    """
    world = gen_worlds.world_path(probe)
    if not world.exists():
        gen_worlds.generate()
    run_id = probe.id.replace(".", "_")
    out_json = work_dir / (run_id + ".probe.json")
    log_path = work_dir / (run_id + ".engine.log")
    console = work_dir / (run_id + ".console.log")
    for p in (out_json, log_path, Path(str(log_path) + ".newton.json"), console):
        if p.exists():
            p.unlink()

    env = engine_launch.build_env("newton", log_path, repo=REPO)
    env["OMNIBENCH_OUT"] = str(out_json)
    # Never inherit a stale scene knob from the operator's shell: the whole
    # point of a capability probe is that the WORLD states its own physics.
    for k in ("OMNISIM_NEWTON_GROUND_MU", "OMNISIM_NEWTON_CONTACT_KD",
              "OMNISIM_NEWTON_CONTACT_KE", "OMNISIM_NEWTON_STATICS",
              "OMNISIM_NEWTON_NATIVE_CONTACTS", "OMNISIM_NEWTON_BALL_HINGE2",
              # Joint-servo gains: an operator with these exported would see
              # joint.hinge_motor_position_unloaded PASS and the whole
              # position-control finding would silently disappear. The probe
              # that WANTS them declares them in `probe.env`, which is applied
              # below and therefore still wins.
              "OMNISIM_NEWTON_TARGET_KE", "OMNISIM_NEWTON_TARGET_KD",
              # Constraint caps: these change contact solving at scale.
              "OMNISIM_NEWTON_NJMAX", "OMNISIM_NEWTON_NCONMAX",
              # The pre-W3.1 external-wrench revert hatch. An operator with
              # this exported would see phenomenon.supervisor_external_force
              # measure `broken` and the whole finding would be an artefact of
              # their shell. It is still reachable deliberately, through --env,
              # which marks the row a CONTROL RUN.
              "OMNISIM_NEWTON_NO_EXT_FORCE"):
        env.pop(k, None)
    env.update({k: str(v) for k, v in probe.env.items()})
    hatch_note = None
    if override:
        env.update({k: str(v) for k, v in override.items()})
        hatch_note = ("CONTROL RUN, not a measurement of the shipped default: "
                      "launched with %s forced. Used to attribute a verdict to "
                      "an engine change rather than to the machine."
                      % ", ".join("%s=%s" % kv for kv in sorted(override.items())))

    base_args, dev_note = None, None
    if probe.needs_rendering:
        # Drop --no-rendering only: a render-dependent device has no image
        # pipeline without it and the controller blocks forever on a frame
        # that never arrives.
        base_args = tuple(x for x in engine_launch.BASE_ARGS
                          if x != "--no-rendering")
        dev_note = ("this probe's device needs the renderer, so it runs "
                    "WITHOUT --no-rendering; it is not comparable to the "
                    "headless rows and says nothing about image quality")
    rc, wall_s, timed_out = engine_launch.launch_once(
        binpath, world, env, console, timeout_s, cwd=REPO,
        base_args=base_args,
        # Forward the controller's own stdout/stderr. Without these the
        # console log is EMPTY for every probe, so a controller that died at
        # import is indistinguishable from an engine that hung -- which is
        # exactly how the first lidar hang presented.
        extra_args=("--stdout", "--stderr"))
    verdict_sidecar = engine_launch.newton_verdict(log_path)
    engine, attribution_reason = engine_launch.engine_attribution(verdict_sidecar)
    diagnostics = read_diagnostics(log_path)

    extras = {
        "engine": engine,
        "rc": rc,
        "wall_s": wall_s,
        "timed_out": timed_out,
        "diagnostics": diagnostics,
        "deviations": [d for d in (attribution_reason, dev_note, hatch_note)
                       if d],
        "steps": 0,
        "sim_seconds": 0.0,
        "wall_ms_per_step": 0.0,
        "problems": [],
    }

    if probe.kind == caps.KIND_STATIC:
        v = classify_static(probe, diagnostics, rc, timed_out)
        if not keep:
            _cleanup(console)
        return v, extras

    if not out_json.exists():
        v = caps.Verdict(
            caps.INCONCLUSIVE,
            {"rc": rc, "timed_out": timed_out,
             "engine_diagnostics": diagnostics[:6]},
            "the prober produced no recording (rc=%s, timed_out=%s) -- this is "
            "an INSTRUMENT failure, not a statement about the capability"
            % (rc, timed_out))
        return v, extras

    with open(out_json, "r", encoding="utf-8") as f:
        rec = json.load(f)
    # Reserved key. Every measure spec the prober emits is prefixed by its kind
    # (pos_/quat_/joint_/sensor_/contacts_/node_exists_/device_exists_/camera_/
    # radio_), so `engine_log` cannot collide with a recorded series.
    if probe.log_capture:
        rec["engine_log"] = read_log_capture(log_path, probe.log_capture)
        extras["engine_log"] = rec["engine_log"]
    meta = rec.get("meta") or {}
    extras["steps"] = int(meta.get("steps") or 0)
    extras["sim_seconds"] = float(meta.get("sim_seconds") or 0.0)
    extras["wall_ms_per_step"] = float(meta.get("wall_ms_per_step") or 0.0)
    extras["problems"] = list(meta.get("problems") or [])

    if extras["steps"] <= 0:
        v = caps.Verdict(
            caps.INCONCLUSIVE,
            {"steps": extras["steps"], "problems": extras["problems"][:6]},
            "the prober recorded zero steps -- instrument failure")
        return v, extras

    try:
        v = probe.assertion(rec)
    except Exception as exc:                       # a broken assertion is ours
        v = caps.Verdict(caps.INCONCLUSIVE, {"assertion_error": repr(exc)},
                         "the probe's own assertion raised %r" % (exc,))
    if not keep:
        _cleanup(out_json, console)
    return v, extras


def _cleanup(*paths):
    for p in paths:
        try:
            Path(p).unlink()
        except OSError:
            pass


def classify_static(probe, diagnostics, rc, timed_out):
    """Static probe: did the ENGINE refuse the declaration?

    `absent` requires a diagnostic naming the declared feature. Nothing
    matching does NOT mean the feature works — it means the declaration was
    swallowed, which a static test cannot resolve, so the verdict is capped at
    `degraded` and every diagnostic is attached for a human to read.
    """
    hits = [d for d in diagnostics
            if any(tok.lower() in d.lower() for tok in probe.absent_markers)]
    ev = {"engine_refused": bool(hits), "matched_diagnostics": hits[:4],
          "diagnostic_count": len(diagnostics), "rc": rc}
    if hits:
        return caps.Verdict(
            caps.ABSENT, ev,
            "the engine refused or ignored the declaration: %s" % hits[0][:200])
    return caps.Verdict(
        caps.DEGRADED, ev,
        "the declaration was accepted with no diagnostic naming it, so this "
        "static probe cannot tell 'implemented' from 'silently swallowed'. "
        "Treat as UNRESOLVED, not as working.")


# ---------------------------------------------------------------------------
# rows
# ---------------------------------------------------------------------------
def build_row(probe, verdict, extras, machine):
    doc_agrees = None
    if probe.documented_as is not None and verdict.verdict != caps.INCONCLUSIVE:
        doc_agrees = (probe.documented_as == verdict.verdict)
    metrics = {
        "verdict": verdict.verdict,
        "family": probe.family,
        "claim": probe.claim,
        "kind": probe.kind,
        "note": verdict.note,
        "evidence": verdict.evidence,
        "physical_claim": (probe.assertion.__doc__ or "").strip()
                          if probe.assertion else None,
        "documented_as": probe.documented_as,
        "doc_agrees": doc_agrees,
        "prober_problems": extras["problems"][:8],
        "engine_diagnostics": extras["diagnostics"][:6],
        # The raw lines behind a log_capture verdict, so the parsed number in
        # `evidence` can be checked against what the engine actually said
        # without re-running. Absent for probes that declare no log_capture.
        "engine_log_capture": extras.get("engine_log", [])[:6],
    }
    return results_mod.make_row(
        test="%s:%s" % (SUITE_TEST_PREFIX, probe.id),
        engine=extras["engine"],
        dt_ms=probe.dt_ms,
        metrics=metrics,
        wall_ms_per_step=extras["wall_ms_per_step"],
        steps=extras["steps"],
        sim_seconds=extras["sim_seconds"],
        deviations=extras["deviations"],
        machine=machine)


def summary_row(rows, machine, wall_s):
    counts = {v: 0 for v in caps.VERDICTS}
    for r in rows:
        counts[r["metrics"]["verdict"]] += 1
    scored = sum(counts[v] for v in (caps.WORKS, caps.DEGRADED, caps.BROKEN,
                                     caps.ABSENT))
    doc_mismatch = [r["test"] for r in rows
                    if r["metrics"].get("doc_agrees") is False]
    return results_mod.make_row(
        test="capability_summary",
        engine="omnisim-newton",
        dt_ms=0,
        metrics={
            "probes_total": len(rows),
            "probes_scored": scored,
            "works": counts[caps.WORKS],
            "degraded": counts[caps.DEGRADED],
            "broken": counts[caps.BROKEN],
            "absent": counts[caps.ABSENT],
            "inconclusive": counts[caps.INCONCLUSIVE],
            # The headline is deliberately NOT "works/total": `absent` is an
            # honest scope statement (no cloth solver is not a defect), so
            # folding it into a percentage would make the engine look broken
            # for things it never claimed. Coverage is works/(works+degraded+
            # broken) -- of the capabilities that EXIST, how many work.
            "implemented_working_frac": (
                counts[caps.WORKS] /
                max(1, counts[caps.WORKS] + counts[caps.DEGRADED]
                    + counts[caps.BROKEN])),
            "doc_mismatches": doc_mismatch,
            "doc_mismatch_count": len(doc_mismatch),
            "sweep_wall_s": wall_s,
        },
        wall_ms_per_step=0.0, steps=0, sim_seconds=0.0,
        deviations=["scope: rigid-body simulation only; rendering quality is "
                    "out of OmniBench's scope by design"],
        machine=machine)


def select(probes, patterns, family):
    out = []
    for p in probes:
        if family and p.family != family:
            continue
        if patterns and not any(p.id.startswith(pat) or fnmatch.fnmatch(p.id, pat)
                                for pat in patterns):
            continue
        out.append(p)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    ap.add_argument("--probes", nargs="*", default=None,
                    help="id prefixes or globs, e.g. joint. device.touch*")
    ap.add_argument("--family", choices=caps.FAMILIES, default=None)
    ap.add_argument("--timeout", type=float, default=180.0,
                    help="per-probe wall-clock cap, seconds")
    ap.add_argument("--keep", action="store_true",
                    help="keep per-probe recordings and console logs")
    ap.add_argument("--work-dir", type=Path, default=None)
    ap.add_argument("--list", action="store_true",
                    help="print the probe list and exit (runs nothing)")
    ap.add_argument("--env", action="append", default=None, metavar="KEY=VAL",
                    help="force an engine env var for these probes, e.g. "
                         "--env OMNISIM_NEWTON_BALL_HINGE2=0. THIS IS FOR "
                         "REVERT-HATCH CONTROLS ONLY: when two machines "
                         "disagree, the first question is whether the engine "
                         "changed between their builds, and re-running the "
                         "probe under the matching hatch answers it without a "
                         "rebuild. The rows are marked as control runs in "
                         "`deviations`; never merge them into a coverage "
                         "sweep.")
    a = ap.parse_args(argv)

    override = {}
    for kv in (a.env or []):
        if "=" not in kv:
            log("--env wants KEY=VAL, got %r" % kv)
            return 2
        k, v = kv.split("=", 1)
        override[k] = v

    problems = caps.validate_registry()
    if problems:
        log("REGISTRY INVALID:")
        for e in problems:
            log("  - " + e)
        return 2

    probes = select(caps.PROBES, a.probes, a.family)
    if a.list:
        for p in probes:
            log("%-46s %-11s %-8s %s"
                % (p.id, p.family, p.kind, p.claim))
        log("(%d probes)" % len(probes))
        return 0
    if not probes:
        log("no probes selected")
        return 2

    binpath = engine_launch.resolve_binary(REPO)
    if binpath is None or not Path(binpath).exists():
        log("omnisim-bin not found -- build first (AGENTS.md section 2)")
        return 2
    gen_worlds.generate()

    work_dir = a.work_dir or (HERE / "results" / "_work")
    work_dir.mkdir(parents=True, exist_ok=True)
    out = Path(a.out)
    if out.exists():
        out.unlink()

    log("lane4a coverage: %d probes, binary %s" % (len(probes), binpath))
    t0 = time.perf_counter()
    rows = []
    for i, probe in enumerate(probes, 1):
        log("[%2d/%2d] %-46s ..." % (i, len(probes), probe.id))
        verdict, extras = run_probe(probe, binpath, work_dir, a.timeout,
                                    keep=a.keep, override=override or None)
        # AFTER the run, never before: the fingerprint's `physics`/`solver`
        # fields are read from THIS run's .newton.json verdict sidecar, which
        # does not exist until the engine reaches world-finalize. Collecting
        # it up front silently stamps every row "UNKNOWN (engine log not
        # found)" while still looking like attribution.
        machine = results_mod.machine_fingerprint(
            engine_log_path=str(work_dir / (probe.id.replace(".", "_")
                                            + ".engine.log")))
        row = build_row(probe, verdict, extras, machine)
        results_mod.append_row(out, row)
        rows.append(row)
        flag = ""
        if row["metrics"].get("doc_agrees") is False:
            flag = "  <-- DOC SAYS %s" % probe.documented_as
        log("         %-12s %s%s"
            % (verdict.verdict.upper(), (verdict.note or "")[:100], flag))

    srow = summary_row(rows, results_mod.machine_fingerprint(),
                       time.perf_counter() - t0)
    results_mod.append_row(out, srow)

    m = srow["metrics"]
    log("")
    log("=" * 72)
    log("lane4a coverage: %d probes in %.1f s" % (len(rows),
                                                  time.perf_counter() - t0))
    log("  works %d | degraded %d | broken %d | absent %d | inconclusive %d"
        % (m["works"], m["degraded"], m["broken"], m["absent"],
           m["inconclusive"]))
    log("  of the capabilities that EXIST, %.0f%% work"
        % (100.0 * m["implemented_working_frac"]))
    if m["doc_mismatch_count"]:
        log("  DOC MISMATCHES (%d):" % m["doc_mismatch_count"])
        for t in m["doc_mismatches"]:
            log("    - " + t)
    log("  -> %s" % out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
