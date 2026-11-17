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

"""Measure how closely OmniSim's simulation clock tracks wall clock.

Hardware-in-the-loop is the reason this exists. A simulator that is merely
*correct* is useless to a device on the other end of a cable: the device runs on
wall clock and will not wait, so the question "does one second of simulated time
take one second?" has to be answered numerically before any HIL claim is made.

This launches ``omnisim-bin`` headless on a probe world, waits for the
``hil_timing_probe`` controller to record a paired (sim, wall) clock sample on
every tick, and reduces that recording to a real-time factor, a jitter
distribution and a cumulative drift.

Read the verdict narrowly. It says the run held its declared thresholds on this
machine, at this scene size, over this many ticks. It does not say the engine is
suitable for a particular device, and it says nothing about tail behaviour
beyond the percentiles printed.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import statistics
import subprocess
import sys
import tempfile
import time
from pathlib import Path

# The realtime factor must land inside a BAND, not merely above a floor.
# 0.98 is the caller-supplied lower bar: 2% slow over a run is roughly the most
# a device-facing loop can absorb before its own timeouts start firing. The
# ceiling exists because this instrument's first run measured a world pacing 2.5%
# FAST -- the fractional-basicTimeStep truncation -- and a one-sided floor
# reported that as held. For hardware in the loop a sim running fast is not a
# lesser fault than one running slow: the device falls behind instead of ahead,
# and nothing in the engine says so.
DEFAULT_REALTIME_FLOOR = 0.98
DEFAULT_REALTIME_CEILING = 1.02

# p99 per-tick jitter, as a fraction of the nominal step, to pass. A quarter of
# a step is a deliberately loose bar for a general-purpose desktop OS -- neither
# Windows nor stock Linux is a real-time kernel, and the scheduler quantum alone
# is a large fraction of an 8 ms step. It is stated in the output precisely
# because it is a choice and not a physical constant.
DEFAULT_JITTER_P99_FRACTION = 0.25

# A tick whose wall interval exceeds nominal by more than this is an overrun.
OVERRUN_FRACTION = 0.20


def repo_root() -> Path:
    # tools/ -> omnisim-hil/ -> packages/ -> repo root
    return Path(__file__).resolve().parents[3]


def find_binary(root: Path) -> Path:
    candidates = [
        root / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe",
        root / "bin" / "omnisim-bin",
        root / "Contents" / "MacOS" / "omnisim",
    ]
    for c in candidates:
        if c.is_file():
            return c
    raise SystemExit(
        "omnisim-bin not found. Looked in:\n  " + "\n  ".join(str(c) for c in candidates)
    )


def sha256_of(path: Path) -> str | None:
    try:
        h = hashlib.sha256()
        with open(path, "rb") as fh:
            for chunk in iter(lambda: fh.read(1 << 20), b""):
                h.update(chunk)
        return h.hexdigest()
    except OSError:
        return None


def fingerprint(root: Path, log_path: Path, binary: Path) -> dict:
    """Machine attribution, preferring the repo's own fingerprint module.

    A timing number is meaningless without the box it came from, and this repo
    is developed across several. env_fingerprint gives a stable machine id plus
    the engine binary hash and the Newton verdict; if it cannot be imported the
    fallback below is recorded under a different source name so a reader can
    tell which they are looking at rather than finding a silently thinner dict.
    """
    sys.path.insert(0, str(root / "projects" / "policies" / "common"))
    try:
        import env_fingerprint  # type: ignore

        fp = env_fingerprint.collect(engine_log_path=str(log_path), repo_root=str(root))
        return {
            "source": "env_fingerprint",
            "machine": fp.get("machine"),
            "gpu": fp.get("gpu"),
            "os": fp.get("os"),
            "build": fp.get("build"),
            "binary": fp.get("binary"),
            "physics": fp.get("physics"),
            "solver": fp.get("solver"),
            "warnings": fp.get("warnings"),
        }
    except Exception as exc:  # noqa: BLE001 - never let attribution abort a run
        return {
            "source": "fallback_minimal",
            "fallback_reason": "%s: %s" % (type(exc).__name__, exc),
            "machine": {
                "os": platform.system(),
                "platform": platform.platform(terse=True),
                "processor": platform.processor(),
                "cpu_count": os.cpu_count(),
            },
            "binary": {"path": str(binary), "sha256": sha256_of(binary)},
            "physics": "unverified",
        }
    finally:
        if sys.path and sys.path[0] == str(root / "projects" / "policies" / "common"):
            sys.path.pop(0)


def newton_sidecar(log_path: Path) -> dict | None:
    side = Path(str(log_path) + ".newton.json")
    try:
        return json.loads(side.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def run_once(
    *,
    binary: Path,
    root: Path,
    world: Path,
    mode: str,
    ticks: int,
    step_ms: int,
    tag: str,
    workdir: Path,
    timeout_s: float,
) -> dict:
    """One engine launch. Returns the raw recording plus run metadata."""
    rec_path = workdir / ("recording_%s.json" % tag)
    log_path = workdir / ("omnisim_log_%s.txt" % tag)
    for stale in (rec_path, log_path, Path(str(log_path) + ".newton.json")):
        try:
            stale.unlink()
        except OSError:
            pass

    env = dict(os.environ)
    env["OMNISIM_HOME"] = str(root)
    # Unique per run: the default is a single shared file in the install root,
    # so two runs writing it would leave neither diagnosable afterwards.
    env["OMNISIM_LOG_PATH"] = str(log_path)

    cmd = [
        str(binary),
        str(world),
        "--batch",
        "--mode=%s" % mode,
        "--no-rendering",
        "--minimize",
        "--stdout",
        "--stderr",
        # --port is deliberately left at its default so the engine auto-scans
        # for a free slot and never collides with an instance already running.
    ]

    # controllerArgs in the world set the defaults; these override per run.
    env["OMNISIM_HIL_TICKS"] = str(ticks)
    env["OMNISIM_HIL_OUT"] = str(rec_path)
    env["OMNISIM_HIL_STEP_MS"] = str(step_ms)

    t0 = time.perf_counter()
    proc = subprocess.run(
        cmd,
        env=env,
        cwd=str(root),
        capture_output=True,
        text=True,
        errors="replace",
        timeout=timeout_s,
    )
    wall_total = time.perf_counter() - t0

    recording = None
    if rec_path.is_file():
        try:
            recording = json.loads(rec_path.read_text(encoding="utf-8"))
        except ValueError as exc:
            recording = None
            note = "recording unparseable: %s" % exc
        else:
            note = None
    else:
        note = "controller wrote no recording at %s" % rec_path

    return {
        "tag": tag,
        "mode": mode,
        "world": str(world),
        "cmd": cmd,
        "exit_code": proc.returncode,
        "launch_wall_s": wall_total,
        "recording": recording,
        "recording_path": str(rec_path),
        "log_path": str(log_path),
        "newton_sidecar": newton_sidecar(log_path),
        "note": note,
        "stdout_tail": proc.stdout[-4000:] if proc.stdout else "",
        "stderr_tail": proc.stderr[-4000:] if proc.stderr else "",
    }


def percentile(sorted_vals: list[float], q: float) -> float:
    """Nearest-rank percentile. No interpolation: with a few hundred samples the
    interpolated value implies a precision the sample size does not support."""
    if not sorted_vals:
        return float("nan")
    k = max(0, min(len(sorted_vals) - 1, int(round(q * (len(sorted_vals) - 1)))))
    return sorted_vals[k]


def analyse(recording: dict) -> dict:
    """Reduce a paired (sim, wall) recording to pacing statistics."""
    samples = recording.get("samples") or []
    if len(samples) < 2:
        return {"ok": False, "reason": "fewer than 2 samples"}

    sim = [s[0] for s in samples]
    wall = [s[1] for s in samples]

    sim_elapsed = sim[-1] - sim[0]
    wall_elapsed = wall[-1] - wall[0]
    realtime_factor = (sim_elapsed / wall_elapsed) if wall_elapsed > 0 else float("inf")

    intervals = [wall[i] - wall[i - 1] for i in range(1, len(wall))]
    sim_intervals = [sim[i] - sim[i - 1] for i in range(1, len(sim))]

    # Nominal is the sim-time step the engine actually delivered, taken as the
    # median of the sim intervals rather than the requested value: the request
    # is a deadline, and if the engine delivered something else the jitter must
    # be measured against what was delivered or it measures the wrong thing.
    nominal_sim_s = statistics.median(sim_intervals)
    nominal_s = nominal_sim_s

    jitter = [abs(iv - nominal_s) for iv in intervals]
    s_int = sorted(intervals)
    s_jit = sorted(jitter)

    # Drift: at each tick, how far the sim clock has fallen behind (negative) or
    # run ahead of (positive) the wall clock since the first sample.
    drift = [(sim[i] - sim[0]) - (wall[i] - wall[0]) for i in range(len(sim))]
    max_abs_drift = max(abs(d) for d in drift)

    overrun_threshold = nominal_s * (1.0 + OVERRUN_FRACTION)
    overruns = sum(1 for iv in intervals if iv > overrun_threshold)

    return {
        "ok": True,
        "ticks": len(samples),
        "sim_elapsed_s": sim_elapsed,
        "wall_elapsed_s": wall_elapsed,
        "realtime_factor": realtime_factor,
        "nominal_step_ms": nominal_s * 1000.0,
        "nominal_step_source": "median of delivered sim intervals",
        "interval_ms": {
            "mean": statistics.fmean(intervals) * 1000.0,
            "median": statistics.median(intervals) * 1000.0,
            "p95": percentile(s_int, 0.95) * 1000.0,
            "p99": percentile(s_int, 0.99) * 1000.0,
            "max": s_int[-1] * 1000.0,
            "min": s_int[0] * 1000.0,
            "stdev": (statistics.stdev(intervals) * 1000.0) if len(intervals) > 1 else 0.0,
        },
        "jitter_ms": {
            "mean": statistics.fmean(jitter) * 1000.0,
            "median": statistics.median(jitter) * 1000.0,
            "p95": percentile(s_jit, 0.95) * 1000.0,
            "p99": percentile(s_jit, 0.99) * 1000.0,
            "max": s_jit[-1] * 1000.0,
        },
        "drift_ms": {
            "final": drift[-1] * 1000.0,
            "max_abs": max_abs_drift * 1000.0,
            "sign_convention": "positive = sim clock ahead of wall clock",
        },
        "overruns": {
            "count": overruns,
            "fraction": overruns / len(intervals),
            "threshold_ms": overrun_threshold * 1000.0,
            "definition": "wall interval exceeded nominal by more than %d%%"
            % int(OVERRUN_FRACTION * 100),
        },
        "basic_time_step_ms": recording.get("basic_time_step_ms"),
        "basic_time_step_is_integer": recording.get("basic_time_step_is_integer"),
        "step_ms_requested": recording.get("step_ms_requested"),
        "perf_counter_resolution_s": recording.get("perf_counter_resolution_s"),
    }


def verdict(stats: dict, mode: str, realtime_floor: float, jitter_fraction: float,
            realtime_ceiling: float = DEFAULT_REALTIME_CEILING) -> dict:
    """A scoped verdict. Never a bare pass: the thresholds travel with it."""
    if not stats.get("ok"):
        return {"applicable": False, "reason": stats.get("reason", "no statistics")}

    if mode != "realtime":
        return {
            "applicable": False,
            "reason": "mode=%s is not real-time paced; a realtime verdict does not apply"
            % mode,
            "realtime_factor": stats["realtime_factor"],
        }

    nominal_ms = stats["nominal_step_ms"]
    jitter_budget_ms = nominal_ms * jitter_fraction
    rf = stats["realtime_factor"]
    too_slow = rf < realtime_floor
    too_fast = rf > realtime_ceiling
    held_realtime = not (too_slow or too_fast)
    held_jitter = stats["jitter_ms"]["p99"] <= jitter_budget_ms

    return {
        "applicable": True,
        "held_realtime": held_realtime,
        "realtime_deviation": "too_slow" if too_slow else ("too_fast" if too_fast else "in_band"),
        "held_jitter_p99": held_jitter,
        "held_both": held_realtime and held_jitter,
        "thresholds": {
            "realtime_factor_floor": realtime_floor,
            "realtime_factor_ceiling": realtime_ceiling,
            "jitter_p99_budget_ms": jitter_budget_ms,
            "jitter_p99_budget_fraction_of_step": jitter_fraction,
        },
        "measured": {
            "realtime_factor": rf,
            "jitter_p99_ms": stats["jitter_ms"]["p99"],
        },
        "scope": (
            "One machine, one scene, %d ticks at a %.3f ms nominal step. "
            "Says nothing about other hardware, larger scenes, or tail behaviour "
            "beyond the percentiles reported."
        )
        % (stats["ticks"], nominal_ms),
    }


def format_report(result: dict) -> str:
    lines = []
    a = lines.append
    a("OmniSim real-time pacing measurement")
    a("=" * 68)
    fp = result["fingerprint"]
    mach = fp.get("machine") or {}
    a("machine        : %s" % (mach.get("id") or mach.get("platform") or "unknown"))
    a("  attribution  : %s" % fp.get("source"))
    if fp.get("fallback_reason"):
        a("  fallback     : %s" % fp["fallback_reason"])
    a("  gpu          : %s" % (fp.get("gpu") or "n/a"))
    a("  os           : %s" % (fp.get("os") or mach.get("platform") or "n/a"))
    binary = fp.get("binary") or {}
    a("  binary sha256: %s" % (binary.get("sha256") or "unknown"))
    a("  git commit   : %s" % (fp.get("build") or "unknown"))
    a("world          : %s" % result["world"])
    a("mode           : %s" % result["mode"])
    a("runs           : %d" % len(result["runs"]))
    a("")

    for run in result["runs"]:
        a("-- run %s (exit %s) --" % (run["tag"], run["exit_code"]))
        side = run.get("newton_sidecar")
        if side:
            a("   newton      : finalised, solver=%s" % side.get("solver"))
        else:
            a("   newton      : NO SIDECAR (Newton did not finalise this run)")
        st = run.get("stats")
        if not st or not st.get("ok"):
            a("   FAILED      : %s" % (run.get("note") or (st or {}).get("reason")))
            if run.get("stderr_tail"):
                a("   stderr tail : %s" % run["stderr_tail"][-500:].replace("\n", " | "))
            a("")
            continue
        a("   ticks       : %d" % st["ticks"])
        a("   basicTimeStep: %s ms (integer: %s)"
          % (st["basic_time_step_ms"], st["basic_time_step_is_integer"]))
        a("   nominal step: %.4f ms (%s)" % (st["nominal_step_ms"], st["nominal_step_source"]))
        a("   sim elapsed : %.4f s" % st["sim_elapsed_s"])
        a("   wall elapsed: %.4f s" % st["wall_elapsed_s"])
        a("   REALTIME FACTOR : %.5f  (sim/wall; 1.0 = tracks wall clock)"
          % st["realtime_factor"])
        iv = st["interval_ms"]
        a("   wall interval ms: mean %.4f  median %.4f  p95 %.4f  p99 %.4f  max %.4f  stdev %.4f"
          % (iv["mean"], iv["median"], iv["p95"], iv["p99"], iv["max"], iv["stdev"]))
        jt = st["jitter_ms"]
        a("   jitter ms       : mean %.4f  median %.4f  p95 %.4f  p99 %.4f  max %.4f"
          % (jt["mean"], jt["median"], jt["p95"], jt["p99"], jt["max"]))
        dr = st["drift_ms"]
        a("   drift ms        : final %+.3f  max_abs %.3f  (%s)"
          % (dr["final"], dr["max_abs"], dr["sign_convention"]))
        ov = st["overruns"]
        a("   overruns        : %d / %d (%.2f%%), %s"
          % (ov["count"], st["ticks"] - 1, ov["fraction"] * 100.0, ov["definition"]))
        v = run.get("verdict") or {}
        if v.get("applicable"):
            th = v["thresholds"]
            rt = "HELD" if v["held_realtime"] else ("MISSED (%s)" % v["realtime_deviation"])
            a("   VERDICT     : realtime %s (band %.3f..%.3f), jitter p99 %s (<= %.4f ms)"
              % (rt, th["realtime_factor_floor"], th["realtime_factor_ceiling"],
                 "HELD" if v["held_jitter_p99"] else "MISSED", th["jitter_p99_budget_ms"]))
            a("   scope       : %s" % v["scope"])
        else:
            a("   VERDICT     : not applicable -- %s" % v.get("reason"))
        a("")

    agg = result.get("aggregate")
    if agg and agg.get("runs") > 1:
        a("-- aggregate over %d runs --" % agg["runs"])
        a("   realtime factor: mean %.5f  min %.5f  max %.5f"
          % (agg["realtime_factor"]["mean"], agg["realtime_factor"]["min"],
             agg["realtime_factor"]["max"]))
        a("   jitter p99 ms  : mean %.4f  max %.4f"
          % (agg["jitter_p99_ms"]["mean"], agg["jitter_p99_ms"]["max"]))
        a("")
    return "\n".join(lines)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Measure OmniSim's real-time pacing and per-tick jitter."
    )
    ap.add_argument("--world", default=None,
                    help="probe world (default: packages/omnisim-hil/worlds/hil_timing_probe.omniworld)")
    ap.add_argument("--mode", choices=["realtime", "fast"], default="realtime")
    ap.add_argument("--ticks", type=int, default=1200)
    ap.add_argument("--step-ms", type=int, default=8)
    ap.add_argument("--repeat", type=int, default=1)
    ap.add_argument("--out", default=None, help="write the JSON result here")
    ap.add_argument("--json", action="store_true", help="print JSON to stdout instead of a report")
    ap.add_argument("--realtime-floor", type=float, default=DEFAULT_REALTIME_FLOOR)
    ap.add_argument("--realtime-ceiling", type=float, default=DEFAULT_REALTIME_CEILING)
    ap.add_argument("--jitter-p99-fraction", type=float, default=DEFAULT_JITTER_P99_FRACTION)
    ap.add_argument("--timeout", type=float, default=0.0,
                    help="per-run timeout in seconds (default: derived from ticks and mode)")
    ap.add_argument("--keep-workdir", action="store_true")
    args = ap.parse_args(argv)

    root = repo_root()
    binary = find_binary(root)
    world = Path(args.world) if args.world else (
        root / "packages" / "omnisim-hil" / "worlds" / "hil_timing_probe.omniworld"
    )
    if not world.is_file():
        raise SystemExit("world not found: %s" % world)

    if args.timeout > 0:
        timeout_s = args.timeout
    else:
        # Real-time mode cannot finish faster than ticks * step; add generous
        # headroom for engine startup, world load and Newton finalize.
        nominal = args.ticks * args.step_ms / 1000.0
        timeout_s = (nominal + 120.0) if args.mode == "realtime" else 300.0

    workdir = Path(tempfile.mkdtemp(prefix="omnisim_hil_"))
    runs = []
    try:
        for i in range(max(1, args.repeat)):
            tag = "%s_%02d" % (args.mode, i + 1)
            run = run_once(
                binary=binary, root=root, world=world, mode=args.mode,
                ticks=args.ticks, step_ms=args.step_ms, tag=tag,
                workdir=workdir, timeout_s=timeout_s,
            )
            if run["recording"]:
                run["stats"] = analyse(run["recording"])
                run["verdict"] = verdict(
                    run["stats"], args.mode, args.realtime_floor,
                    args.jitter_p99_fraction, args.realtime_ceiling,
                )
                # The raw per-tick pairs are large and are not part of the
                # result contract; the recording file keeps them if wanted.
                run["recording"] = {
                    k: v for k, v in run["recording"].items() if k != "samples"
                }
            runs.append(run)

        good = [r for r in runs if (r.get("stats") or {}).get("ok")]
        aggregate = None
        if good:
            rfs = [r["stats"]["realtime_factor"] for r in good]
            p99s = [r["stats"]["jitter_ms"]["p99"] for r in good]
            aggregate = {
                "runs": len(good),
                "realtime_factor": {"mean": statistics.fmean(rfs), "min": min(rfs), "max": max(rfs)},
                "jitter_p99_ms": {"mean": statistics.fmean(p99s), "max": max(p99s)},
            }

        # Fingerprint from the last run's log so the Newton verdict it reports
        # belongs to a run in this result rather than to whatever ran before.
        last_log = Path(runs[-1]["log_path"])
        result = {
            "schema": "omnisim-hil/realtime-measurement/1",
            "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "world": str(world),
            "mode": args.mode,
            "ticks_requested": args.ticks,
            "step_ms_requested": args.step_ms,
            "fingerprint": fingerprint(root, last_log, binary),
            "runs": runs,
            "aggregate": aggregate,
        }

        if args.out:
            Path(args.out).parent.mkdir(parents=True, exist_ok=True)
            Path(args.out).write_text(json.dumps(result, indent=2), encoding="utf-8")

        if args.json:
            print(json.dumps(result, indent=2))
        else:
            print(format_report(result))

        return 0 if good else 1
    finally:
        if not args.keep_workdir:
            import shutil
            shutil.rmtree(workdir, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())
