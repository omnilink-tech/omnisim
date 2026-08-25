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

"""run_omnisim.py — OmniBench lane-1 runner for the OmniSim engine.

Runs the generated lane-1 worlds headless through omnisim-bin and verifies that
a real trajectory came out.

    python tests/benchmarks/omnibench/lane1/run_omnisim.py \
        --test T1 --dt-ms 4 --backend newton --out results/omnibench_local

    --test     T1..T7 or "all" (T2 expands to its 7 incline-angle worlds)
    --dt-ms    one of 1,2,4,8,16,32 or "all"
    --backend  newton (the only value; `ode` is REFUSED — see below)
    --out      output dir (npz + meta + run.json + logs per run)

!! THE ODE ARM IS RETIRED (2026-08-08). `--backend ode` used to run an
`_odepin` world twin and was lane 1's *reference* arm. src/ode was DELETED
(commit bdc02139), so passing `ode` is refused rather than silently served by
Newton — a mislabelled row would be worse than a missing one.

What lane 1's reference actually IS: the per-scene closed-form values in
`common/scenes.py`, applied by `lane1/score.py`, which reads NO engine's numbers
as a reference. `run_mujoco.py` / `run_pybullet.py` are the surviving external
cross-checks — they corroborate a scene, they are not the truth for it.
There is NO frozen file of lane-1 ODE values, and one used to be cited here:
`tests/goldens/ode_oracle_goldens.json` is a different body of evidence (29
measurements over 8 DEVICE-parity families — raycast, native_inertia, weld,
touch_force, receiver/lightsensor/radar occlusion, kinematic_native — with zero
T1–T7 scenes, and two families noting ODE was never a numerical oracle there).
The ODE arm's own 105 lane-1 rows survive only as history under `results*/` and
are permanently unrepeatable: lane 1 has lost its only in-engine corroborating
implementation.

Per run it emits <stem>_dt<dt>_<backend>.{npz, npz.meta.json, run.json,
engine.log, console.log}. Metrics are computed later by score.py from the
.npz — this runner records trajectories + the backend verdict, never scores.

Verification rules (hard-won in this repo — do not soften):
  * Exit code 0 proves NOTHING. The .npz must exist with a plausible row
    count; otherwise the engine-log tail is dumped.
  * Newton proof is the race-free sidecar `<OMNISIM_LOG_PATH>.newton.json`
    (must exist, degraded=false); scraping the log is the buggy legacy path.
    OMNISIM_REQUIRE_NEWTON=1 is also set so a fallback dies loudly.
  * A recorder that produces 0 rows is the stale-libController signature
    (engine<->libController IPC-nonce split hangs every controller at zero
    ticks while exiting 0) — reported loudly, gated by `python -m omnisim
    doctor`, never retried forever.
  * Every run is wall-clock capped (default 180 s) and the process tree is
    killed on expiry (taskkill /T on Windows).

Newton deviations recorded per run (the Newton backend does not consume
ContactProperties):
  * friction: single global knob OMNISIM_NEWTON_GROUND_MU, set per scene
    (T1 -> 0, T2 -> 0.5, T3/T6 -> 0.8);
  * restitution: no mapping exists (contact compliance ke/kd only) — T1's
    e=0.8 is not expressible; the measured bounce IS the finding;
  * solver pinned to "mujoco" in every world (XPBD contact pathologies);
  * T2/T6 worlds pin newtonCone "elliptic" + newtonImpratio 10 (MuJoCo-stock
    pyramidal cone creeps near the cone boundary; the global default is
    deliberately unchanged pending champion re-verification).
"""

import argparse
import datetime
import json
import os
import sys

HERE = os.path.abspath(os.path.dirname(__file__))
OMNIBENCH = os.path.dirname(HERE)
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
sys.path.insert(0, HERE)
if OMNIBENCH not in sys.path:
    sys.path.insert(0, OMNIBENCH)

import gen_worlds  # noqa: E402  (owned sibling — scene registry + world paths)
from common import engine_launch  # noqa: E402  (shared per-platform launcher)

try:
    import numpy as np
except ImportError:
    np = None

# Newton global friction knob per scene (the Newton backend has no
# per-ContactProperties friction; OMNISIM_NEWTON_GROUND_MU is the only dial).
NEWTON_MU = {"T1": "0.0", "T2": "0.5", "T3": "0.8", "T6": "0.8"}

LAUNCH_ATTEMPTS = int(os.environ.get("OMNIBENCH_LAUNCH_ATTEMPTS", "3"))
# back-to-back launches can lose the local-controller startup race (same
# flake physics_oracle.py mitigates); a clean launch succeeds on attempt 1,
# retries only cost time on the flake path. In a long back-to-back campaign
# the measured flake rate is much higher than in isolated verification —
# run_all's gap-retry pass sets OMNIBENCH_LAUNCH_ATTEMPTS=8 (the calibrated
# physics_oracle/determinism margin).

# --stdout --stderr on top of the shared headless base args: the recorder's
# own prints land in the console log for post-mortems.
EXTRA_ARGS = ("--stdout", "--stderr")


def find_bin():
    return engine_launch.resolve_binary(REPO)


def tail(path, n=25):
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            return "".join(f.readlines()[-n:])
    except OSError:
        return "<no log at %s>" % path


def check_npz(npz_path, expected_steps):
    """-> (ok, rows, problems[])"""
    problems = []
    if not os.path.exists(npz_path):
        return False, 0, ["npz missing"]
    if np is None:
        return True, -1, ["numpy unavailable in runner python; npz unverified"]
    try:
        data = np.load(npz_path)
    except Exception as e:  # noqa: BLE001
        return False, 0, ["npz unreadable: %s" % e]
    if "t" not in data:
        return False, 0, ["npz has no 't' key"]
    rows = int(data["t"].shape[0])
    if rows - 1 < int(0.9 * expected_steps):
        problems.append("short trajectory: %d rows, expected ~%d"
                        % (rows, expected_steps + 1))
    for k in data.files:
        if not np.all(np.isfinite(data[k])):
            problems.append("non-finite values in '%s'" % k)
    return not problems, rows, problems


def run_one(binpath, stem, dt_ms, backend, out_dir, timeout_s):
    scene = gen_worlds.SCENES[stem]
    world = gen_worlds.world_path(stem, dt_ms)
    if not os.path.exists(world):
        gen_worlds.generate(stems=[stem], dts=[dt_ms])
    run_id = "%s_dt%d_%s" % (stem, dt_ms, backend)
    npz_path = os.path.join(out_dir, run_id + ".npz")
    log_path = os.path.join(out_dir, run_id + ".engine.log")
    console_log = os.path.join(out_dir, run_id + ".console.log")

    deviations = ["newtonSolver \"mujoco\" pinned in the world (XPBD contact "
                  "pathologies: locked wheel pairs, cannot hold grasps)"]
    if scene["test"] == "T5":
        deviations.append(
            "T5 torque phase is a supervisor addTorque couple on the two links "
            "flanking the middle joint (exact for this planar chain), not "
            "Motor.setTorque: the Newton backend gives motorized hinges a "
            "hardcoded position servo that pins the joint and pumps linear "
            "momentum (~17 kg*m/s measured); the couple keeps both joints "
            "passive under both backends")
    if scene["test"] == "T7":
        deviations.append(
            "initial spin: recorder tries supervisor setVelocity first (a t=0 "
            "setVelocity used to be dropped on Newton — pre-registration "
            "immediate message; now queued + drained after finalize) and "
            "falls back to a closed-loop torque-impulse spin-up if the "
            "achieved omega is off; the method actually used is in the "
            ".meta.json 'spin' block")
    if scene.get("softCFM") is not None:
        deviations.append(
            "HISTORICAL: this scene once declared ContactProperties softCFM=%g "
            "to stop the engine default 0.001 mixing into the ODE bounce "
            "constraint and killing restitution (measured first peak 0.02 m at "
            "dt=1 ms vs 0.64 analytic). ODE was deleted (bdc02139) and the "
            "Newton backend never read the field; the world no longer declares "
            "it, and restitution now comes from newtonContactKd" % scene["softCFM"])
    # Shared launcher owns OMNISIM_HOME/WEBOTS_HOME, OMNISIM_LOG_PATH,
    # backend select and (on Linux) the runtime vars; scene knobs stay here.
    env = engine_launch.build_env(backend, log_path, repo=REPO)
    env["OMNIBENCH_OUT"] = npz_path
    env.pop("OMNISIM_NEWTON_GROUND_MU", None)   # never inherit a stale knob
    env.pop("OMNISIM_NEWTON_CONTACT_KD", None)  # from the parent shell
    # SINGLE-ARM LANE: the `ode` branch here (which appended the _odepin
    # deviation note) went with src/ode in commit bdc02139.
    mu = NEWTON_MU.get(scene["test"])
    if mu is not None:
        env["OMNISIM_NEWTON_GROUND_MU"] = mu
        deviations.append(
            "Newton friction is the global OMNISIM_NEWTON_GROUND_MU=%s "
            "(ContactProperties.coulombFriction=%g is an ODE-path field the "
            "Newton backend ignores)" % (mu, scene["mu"]))
    if scene.get("newton_cone"):
        deviations.append(
            "newtonCone \"elliptic\" + newtonImpratio 10 pinned in the "
            "world (MuJoCo-stock pyramidal cone creeps near the friction-"
            "cone boundary: 181 mm pseudo-slip at 26 deg with mu=0.5; "
            "elliptic+impratio-10 sticks at 0.6 mm). Global Newton default "
            "stays MuJoCo stock pending champion re-verification")
    if scene["test"] == "T1":
        # Newton/MuJoCo has no restitution coefficient; the contact is a
        # damped spring (ke, kd -> solref). Calibrated per SPEC on the
        # dt=1 ms drop: e = exp(-pi*zeta/sqrt(1-zeta^2)),
        # zeta = kd/(2*sqrt(ke*m)); ke=2500 (default), m=1 -> kd=7 gives
        # measured peaks 0.647/0.412/0.263/0.167/0.105 vs analytic
        # 0.640/0.410/0.262/0.168/0.107 (e_eff ~ 0.80). Engine default
        # kd=100 is a DEAD contact (first peak ~0.00 m).
        env["OMNISIM_NEWTON_CONTACT_KD"] = "7"
        deviations.append(
            "Newton has no restitution coefficient; e=0.8 realised via "
            "contact-compliance calibration OMNISIM_NEWTON_CONTACT_KD=7 "
            "(ke=2500 default; damped-spring zeta=0.070 -> e~0.80, "
            "calibrated at dt=1 ms per SPEC; engine-default kd=100 gives "
            "e~0). Soft contact penetrates ~0.08 m at impact — recorded, "
            "not hidden")

    expected_steps = int(round(scene["duration"] * 1000.0 / dt_ms))

    # clear stale outputs so this run's artefacts are unambiguous
    for p in (npz_path, npz_path + ".meta.json", log_path,
              log_path + ".newton.json"):
        if os.path.exists(p):
            os.remove(p)

    def attempt_fn(attempt):
        rc, wall_s, timed_out = engine_launch.launch_once(
            binpath, world, env, console_log, timeout_s,
            extra_args=EXTRA_ARGS, cwd=REPO)
        ok, rows, problems = check_npz(npz_path, expected_steps)
        return rc, wall_s, timed_out, ok, rows, problems

    (rc, wall_s, timed_out, ok, rows, problems), _ = \
        engine_launch.launch_with_retries(
            attempts=LAUNCH_ATTEMPTS, attempt_fn=attempt_fn,
            success_fn=lambda r: r[3] or r[4] > 1,
            label=run_id, log=print)

    verdict = engine_launch.newton_verdict(log_path)
    backend_ok = engine_launch.backend_proven(backend, verdict)
    # The PUBLISHED engine label is decided by the sidecar, never by the
    # --backend argument: an unproven run used to be stamped "omnisim-newton"
    # regardless (and the console called it "ode (no sidecar)", naming a
    # deleted engine). Same rule lane 3 adopted in 17c92a211.
    engine_label, engine_why = engine_launch.engine_attribution(verdict)
    if engine_why:
        deviations.append(engine_why)
    if not backend_ok:
        # Only one arm exists now, so there is only one way to fail this.
        problems.append("Newton verdict sidecar %s -> run was NOT proven "
                        "Newton" % (verdict if verdict.get("present")
                                    else "MISSING"))

    if rows <= 1 and not timed_out:
        print("  [%s] LOUD FAILURE: recorder produced %d rows (exit code %s). "
              "Zero ticks with a clean exit is the STALE-libController "
              "signature (engine<->libController IPC-nonce split hangs every "
              "controller silently). Run `python -m omnisim doctor` before "
              "anything else." % (run_id, rows, rc))
        print("  engine log tail:\n%s" % tail(log_path))
        print("  console tail:\n%s" % tail(console_log))
    elif not ok:
        print("  [%s] FAILED verification: %s" % (run_id, "; ".join(problems)))
        print("  engine log tail:\n%s" % tail(log_path))

    meta = {}
    meta_path = npz_path + ".meta.json"
    if os.path.exists(meta_path):
        with open(meta_path, "r", encoding="utf-8") as f:
            meta = json.load(f)

    row = {
        "suite": "omnibench/v0",
        "test": scene["test"],
        "scene": stem,
        "engine": engine_label,
        "dt_ms": dt_ms,
        "metrics": None,  # score.py computes offline from the npz
        "wall_ms_per_step": meta.get("wall_ms_per_step"),
        "steps": meta.get("steps", max(rows - 1, 0)),
        "sim_seconds": meta.get("sim_seconds", 0.0),
        "npz": os.path.abspath(npz_path),
        "world": os.path.abspath(world),
        "exit_code": rc,
        "timed_out": timed_out,
        "wall_s_total": round(wall_s, 2) if wall_s is not None else None,
        "backend_verdict": verdict,
        "ok": bool(ok and backend_ok and not timed_out),
        "problems": problems,
        "deviations": deviations,
        "utc": datetime.datetime.now(datetime.timezone.utc)
              .strftime("%Y-%m-%dT%H:%M:%SZ"),
    }
    with open(os.path.join(out_dir, run_id + ".run.json"), "w",
              encoding="utf-8") as f:
        json.dump(row, f, indent=2)
    return row


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--test", required=True,
                    help="T1..T7 or 'all' (T2 expands to 7 angle worlds)")
    ap.add_argument("--dt-ms", required=True,
                    help="one of %s or 'all'" % gen_worlds.DTS_MS)
    # "ode" is deliberately still an ACCEPTED token so a stale caller gets the
    # sentence below instead of an argparse "invalid choice" with no reason.
    ap.add_argument("--backend", required=True, choices=["ode", "newton"],
                    help="newton (the only backend). 'ode' is refused with an "
                         "explanation -- src/ode was deleted in bdc02139.")
    ap.add_argument("--out", required=True, help="output directory")
    ap.add_argument("--timeout", type=float, default=180.0,
                    help="wall seconds per run before the tree is killed")
    args = ap.parse_args()

    if args.backend == "ode":
        print("--backend ode is RETIRED: ODE was DELETED (src/ode, commit "
              "bdc02139) and Newton is the only physics backend.")
        print("Lane 1's ODE arm was this suite's cross-backend correctness "
              "REFERENCE; with it gone the lane measures Newton against the "
              "ANALYTIC ground truth each scene encodes, and cannot answer "
              "'is Newton as good as ODE here' at all.")
        print("What lane 1's reference actually IS: the per-scene closed-form "
              "values in common/scenes.py, applied by lane1/score.py -- which "
              "reads no engine's numbers as a reference. MuJoCo and PyBullet "
              "(lane1/run_mujoco.py, run_pybullet.py) are the surviving "
              "external cross-checks; they corroborate, they are not the truth.")
        print("There is NO frozen file of lane-1 ODE values. "
              "tests/goldens/ode_oracle_goldens.json is a DIFFERENT body of "
              "evidence -- 29 measurements over 8 DEVICE-parity families "
              "(raycast, native_inertia, weld, touch_force, receiver/"
              "lightsensor/radar occlusion, kinematic_native), zero T1-T7 "
              "scenes, and two families record that ODE was never a numerical "
              "oracle there at all.")
        print("The ODE arm's own 105 lane-1 rows survive only as history under "
              "tests/benchmarks/omnibench/results*/ and are permanently "
              "UNREPEATABLE: lane 1 has lost its only in-engine corroborating "
              "implementation.")
        print("Re-run with --backend newton.")
        return 2

    binpath = find_bin()
    if binpath is None:
        print("omnisim-bin not found (msys64/mingw64/bin/omnisim-bin.exe on "
              "Windows, bin/omnisim-bin on Linux) — build first.")
        return 2

    tests = sorted(gen_worlds.TESTS) if args.test == "all" else [args.test.upper()]
    for t in tests:
        if t not in gen_worlds.TESTS:
            print("unknown test %r (have %s)" % (t, sorted(gen_worlds.TESTS)))
            return 2
    dts = gen_worlds.DTS_MS if args.dt_ms == "all" else [int(args.dt_ms)]
    for d in dts:
        if d not in gen_worlds.DTS_MS:
            print("dt %r not in the sweep %s" % (d, gen_worlds.DTS_MS))
            return 2

    out_dir = os.path.abspath(args.out)
    os.makedirs(out_dir, exist_ok=True)

    results = []
    for t in tests:
        for stem in gen_worlds.TESTS[t]:
            for dt in dts:
                run_id = "%s_dt%d_%s" % (stem, dt, args.backend)
                print("[omnibench] %s ..." % run_id, flush=True)
                row = run_one(binpath, stem, dt, args.backend, out_dir,
                              args.timeout)
                verdict = row["backend_verdict"]
                vtxt = ("newton/%s degraded=%s" % (verdict.get("solver"),
                                                   verdict.get("degraded"))
                        if verdict.get("present")
                        else "UNVERIFIED (no sidecar)")
                print("  -> %s  steps=%s  wall=%.4g ms/step  backend=%s%s"
                      % ("OK " if row["ok"] else "FAIL", row["steps"],
                         row["wall_ms_per_step"] or float("nan"), vtxt,
                         "" if row["ok"] else "  problems=%s" % row["problems"]),
                      flush=True)
                results.append(row)

    n_ok = sum(1 for r in results if r["ok"])
    print("\n[omnibench] %d/%d runs OK -> %s" % (n_ok, len(results), out_dir))
    return 0 if n_ok == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
