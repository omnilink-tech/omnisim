# OmniBench lane 1R — physics validation against MEASURED motion

Lane 1 asks *does the engine solve the idealised model correctly?* — error
against a closed form. **Lane 1R asks a different question that analytic
scenes cannot: does it match reality?**

That distinction is formal, and Newton's own verification suite states it
better than we would: verification tests "compare simulator output against a
closed-form analytical solution … They are **not** a measure of physical
plausibility, real-world fidelity, or agreement with another simulator; those
belong in separate validation … suites." This is that suite.

## The data

550 tosses of an acrylic cube onto a wooden table, AprilTag/TagSLAM tracked at
148 Hz, from [DAIRLab/dair_pll](https://github.com/DAIRLab/dair_pll) —
**BSD-3-Clause**, vendored under `data/` with its licence. Each trajectory is
121 samples of `quat(4) pos(3) omega(3) vel(3)`, and row 0 is a full initial
condition **including both velocities**. That last property is why this
dataset is replayable and why most public robot datasets are not.

## Why this dataset first

It carries published per-simulator baselines, so our first number is a
**harness check before it is a result** — our backend *is* MuJoCo, so landing
far from MuJoCo's row means the harness is wrong, not the physics
(Acosta, Yang & Posa, *Validating Robotics Simulators on Real-World Impacts*,
RA-L 2022, [arXiv:2110.00541](https://arxiv.org/abs/2110.00541)):

| simulator | position err (% cube width) | rotation err (deg) |
|---|---|---|
| Drake | 13.5 ± 8.2 | 16.5 ± 20.0 |
| Bullet | 14.9 ± 8.9 | 16.5 ± 20.2 |
| MuJoCo | 25.1 ± 10.8 | 21.7 ± 21.4 |

**Expect a poor-looking number, and do not treat that as a bug.** On a
benchmark where the best engine in the world manages 13.5 % error on a tossed
cube, this is the normal state of the field — which is itself the most useful
thing the lane tells a user. Acosta's own finding is that every engine
reproduces inelastic impacts well and *all of them* fail on elastic ones.

## Run it

```bash
python tests/benchmarks/omnibench/lane1r/dataset.py            # self-calibrate
python tests/benchmarks/omnibench/lane1r/dataset.py --probe-conventions
python tests/benchmarks/omnibench/lane1r/run.py   --out results/x --indices 0-49
python tests/benchmarks/omnibench/lane1r/score.py --runs results/x
```

## What was measured rather than assumed

Every one of these would have silently biased the score.

1. **The inertia is published three ways.** URDF 8.1e-4 at 0.1048 m; the
   shipped MJCF 6.167e-4 at 0.10 m; Acosta's Table I 8.1e-3 (10×, a typo).
   Arithmetic decides: a *uniform* 0.10 m 0.37 kg cube is exactly 6.167e-4, so
   the MJCF assumed uniform density. Uniform at 0.1048 m is 6.77e-4 and the
   measured value exceeds it — a hollow cube. **The URDF is canonical.**

2. **The dataset repo's own `DT = 0.0068` is 0.64 % too large**, which is what
   makes the data appear to have g = 9.48 m/s². The published 148 Hz is right.

3. **The tracked lengths carry a ~2.2 % scale factor.** Two independent
   witnesses agree to 0.09 %: settled rest height / half-width = 0.97785, and
   free-flight gravity at 148 Hz / 9.81 = 0.97873. `dataset.calibrate()`
   re-derives this every run. `--scale metric` undoes it; `--scale none` is
   what compares to the baselines above, which were **not** corrected.

4. **Quaternion order and omega frame** (`wxyz`, **body**), determined from
   free-flight kinematics — a cube settles flat under either convention, so
   the data cannot be eyeballed. One-step prediction error: `wxyz_body`
   0.0199° vs 1.84° for the next best, a 93× separation.
   `--probe-conventions` re-derives it and exits 1 on disagreement.

## Traps hit while building this, all fixed

- **`Robot.step()` takes an integer ms** and defaults to
  `int(basic_time_step)`. A world with `basicTimeStep 0.6757` yields
  `step(0)` — an infinite no-op, not an error. Sim rate is therefore
  decoupled from the data rate: record on a 1 ms grid, resample when scoring.
- **A controller's cwd is its own directory**, so a relative output path
  writes nowhere — and the traceback and breadcrumb writers meant to report
  that fail identically, swallowed by their own `except: pass`. Paths from
  the runner are absolute.
- **`simulationQuit(0)` is required.** Without it the controller finishes in
  ~2 s and the engine free-runs to the runner's timeout: 240 s per toss
  instead of 8 s, i.e. 36 hours for the campaign instead of 75 minutes.
- **ASCII in printed output.** A decorative U+26A0 crashes a cp1252 console,
  exactly as it once crashed `run_all.py --help`.

## What this lane does NOT claim

- **No restitution field exists on the Newton path.** The cube's measured
  e = 0.125 cannot be declared, only identified through the contact spring
  (`newtonContactKe`/`Kd`). A fitted e is a fitted e; both the as-authored and
  the identified numbers get reported.
- Acosta's identified friction did not match anyone's measurement either —
  μ = 0.18 by tilt test, but the best-fit values were Drake 0.10, MuJoCo 0.22,
  Bullet 0.36. Contact parameters absorb modelling error. Expect the same.
- **A better score here is not evidence of a better simulator.** Fidelity is
  task dependent: the Drake > Bullet > MuJoCo ordering on rigid impacts
  *inverts* on cloth. Publishing a reproducible score at all is the point;
  winning our own benchmark is worth close to nothing — SimBenchmark was
  written by RaiSim's developers and RaiSim won, and it is discounted for
  exactly that reason.
