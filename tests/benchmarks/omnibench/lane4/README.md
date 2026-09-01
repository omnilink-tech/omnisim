# OmniBench lane 4 — capability coverage and resource envelope

**The question no other lane answers: *what can this simulator actually
simulate, and how much of it, on this machine?***

Lanes 1 and 1R ask whether the physics is *right* (against analytic ground
truth, and against 550 real cube tosses). Lane 2 asks how *fast* it is for one
robot at three batch sizes. Lane 3 asks whether it is *deterministic* and
whether an agent can *drive* it. All four are conditional on a prior question
that was previously answered only in prose: can it simulate the thing you
have at all?

```bash
python tests/benchmarks/omnibench/lane4/run_coverage.py     # 4a capability probes
python tests/benchmarks/omnibench/lane4/run_envelope.py     # 4b scaling + cliffs
python tests/benchmarks/omnibench/lane4/cpu_only.py         # 4c no CUDA visible
python tests/benchmarks/omnibench/lane4/report.py --out ../../../docs/benchmarks/...
```

`run_coverage.py --list` prints the probe table without running anything.

### Running it on more than one machine

This lane's most likely failure is not a wrong verdict, it is a **stale** one --
a row that was true when it was measured and has not been re-run since. A second
machine is a cheap detector for that, and on 2026-08-17 it found one:
`device.lidar` had been published as `no result` for two days while working.

```bash
# render the matrix from THIS machine, with another machine's rows beside it
python .../report.py --cross tests/benchmarks/omnibench/results/<id>/<date>/lane4

# replace one probe's row without re-running the other 50 (recomputes the
# derived summary; leaves every other row byte-identical)
python .../merge_coverage.py /tmp/relidar.jsonl

# attribute a disagreement to an ENGINE change rather than to the machine:
# re-run the probe under the matching revert hatch
python .../run_coverage.py --probes joint.hinge2_motor     --env OMNISIM_NEWTON_BALL_HINGE2=0 --out /tmp/control.jsonl
```

Three rules hold this together, because two machines never run the same engine
binary (one is a MinGW/Windows build, the other gcc/Linux) and so a disagreement
is **not** a machine finding until it has been attributed:

* `results/coverage.jsonl` is **one machine's** matrix. `report.py` keys probes
  by id, so merging a second machine's rows would overwrite verdicts rather than
  add a column -- `merge_coverage.py` refuses a foreign-machine row outright, and
  `--cross` keeps the files separate.
* rows produced by `--env` are marked `CONTROL RUN` in `deviations` and are also
  refused by the merge: a revert-hatch measurement is not a measurement of the
  shipped default.
* the summary row is **derived**. Recompute it; never hand-edit a percentage.

---

## Why this lane exists

[docs/developer/simulator-comparison.md §4](../../../../docs/developer/simulator-comparison.md)
carries a capability matrix, and marks its own OmniSim column `⊘` —
"self-attested against this checkout … checked by us, by us, with no external
source and no independent audit", the weakest evidence tier that document
defines. Meanwhile AGENTS.md maintains a hand-written list of what is broken
under Newton. Both are prose, and prose drifts in both directions: this lane's
first full run found a capability documented as working that is broken
(`device.touch_bumper`) alongside ones that are broken exactly as documented.

Both of that run's disagreements are now closed (2026-08-13, `doc_mismatch_count: 0`),
and the two closures are worth contrasting because only one of them was a doc bug:

* **`device.touch_bumper`** — the docs were wrong. AGENTS.md and
  [newton-physics-backend.md](../../../../docs/guide/newton-physics-backend.md) both
  listed BUMPER-type contact reads as native and working, and named only the
  *force* type as broken — which sent anyone blocked on the force type to the
  bumper as the workaround. Measured `max=0` over 750 samples, from the same
  mechanism as the force type (the sensor's own `boundingObject` is not a
  collider), so neither is a workaround for the other. Both docs corrected.
* **`phenomenon.implicit_ground_plane`** — the *engine* changed, on purpose.
  The unconditional z=0 plane became a declared substitution for a dropped
  authored `Plane` collider on 2026-08-12, so the row went `works → absent`.
  Re-measured on the current binary: the sphere falls to **z=-75.56** in a
  world declaring no ground. `documented_as` was flipped to `absent`, and the
  assertion's sense was **inverted rather than deleted** — `works` is now the
  regression verdict, and it names `OMNISIM_NEWTON_GROUND_PLANE=1` as the first
  thing to check before reporting one.

A verdict flip is not automatically a defect. A row that changes because the
engine deliberately changed still has to be re-measured and re-documented, and
the audit is what forces that.

Every row here is executed against `omnisim-bin` and judged by an assertion in
physical units, with a machine id, an engine-binary sha256, and a `.newton.json`
backend verdict on every row.

---

## 4a — the capability registry

[`capabilities.py`](capabilities.py) is the single source of truth. Each probe
declares a claim, a world, what to measure, and an assertion whose **docstring
is published as the physical claim being tested**.

### The four verdicts

| verdict | meaning |
|---|---|
| `works` | present, and the measurement lands where physics says it must |
| `degraded` | present and doing something, but missing the physical target — the note says by how much |
| `broken` | present in the schema (the world loads, the device is accepted) and measurably does nothing |
| `absent` | not in the schema, established by the engine **refusing** the declaration |

Plus `inconclusive`, which is **not a capability verdict**: it means the probe's
own instrument failed, and it is excluded from every score.

**`broken` vs `absent` is what the lane is built around**, and neither a static
nor a dynamic test can make that distinction alone:

- a static test sees `BallJoint`'s motor accepted → calls it present
- a dynamic test sees the joint never move → calls it broken

Both run. A capability that parses and then does nothing is strictly worse than
one that is absent, because the world author gets no signal — and it is exactly
what a load-only smoke test reports as PASS.

### Scope

Rigid-body simulation, plus — since round 3 (2026-09-01) — the particle nodes,
measured through the `getParticleStats` supervisor readback below. Rendering
quality is out of OmniBench's scope by design; the Camera and Lidar probes
assert only that an image *exists* and is non-degenerate. Those two probes
drop `--no-rendering` (a render-dependent device otherwise blocks on a frame
that never arrives) and say so in their `deviations`.

### The particle readback (`particles:DEF`, round 3)

Until 2026-09-01 the particle nodes (`Cloth`, `SoftBody`, `GranularGroup`,
`GranularBed`) had **no supervisor accessor for particle state**, so their
rows were capped at `degraded` on an engine self-report (the "registered N
particles" log line — proof the node reached a solver, not that the solver
did anything right). The `Node.getParticleStats(sample_stride=0)` binding
closes that: the prober's `particles:DEF` measure spec records one stats
frame per step — `{status, count, min[3], max[3], centroid[3], non_finite}` —
and the assertions now measure the drape / squash / settle **in metres**
(cloth z-extent growth with the pinned edge held, soft-body fall + arrest +
deformation, granular centroid drop + extent collapse + arrest).

Discipline, per the prober's standing robustness contract: the read is
**getattr-guarded**, so a libController that predates the binding records a
problem and the row lands on `inconclusive` — and a readback that *refuses*
(`status != 0`, `count == 0`) is an environment/instrument condition, **never
`broken`**. Two status codes are load-bearing: `-9` (stale libController) →
`inconclusive`; `-5` (GranularGroup CUDA-inert) → `absent` on that one probe,
because "requires engine CUDA; this build reports it unavailable" is a scope
statement the engine also logs in prose (`GranularGroup is inert: CUDA is not
available`). The MPM sibling (`object.granular_bed_mpm`, new in round 3) gets
its no-CUDA `absent` from the runtime's own **named refusal** (`GranularBed
requires CUDA`, captured via `log_capture`) — the runtime refuses the world
rather than degrading, by design.

Round 3 also gave the prober one new act verb, `rebuild_physics:T` →
`supervisor.simulationRebuildPhysics()` (getattr-guarded the same way, with
an `acted_rebuild_physics` premise record), which is what turned
`phenomenon.runtime_node_deletion` into the honest two-arm probe: the default
arm documents the frozen-model phantom (a deleted floor still holds the box),
the rebuild arm proves the 2026-09-01 verb releases it, and the verdict is
`works` only when the rebuild arm passes. `device.connector_weld` gained the
mating pair its old probe deliberately lacked: a gravity-hang (bodiless
active Connector on a static anchor welds the passive side's body to the
world) asserted against an in-run control twin that must fall. All of the
new assertion arms are covered by `capabilities.py --self-test`, in both
directions.

---

## 4b — the resource envelope

[`run_envelope.py`](run_envelope.py) answers two things per machine.

**How large a scene runs in real time.** Reported as a *bracket* between two
measured sizes — the largest N still at or above 1× realtime and the smallest N
below it — never an interpolation. The cost curve has a fixed conversion term
and a contact-dependent term, and fitting a line through that is how an
unreproducible headline gets published.

**Where the scene starts degrading silently.** On the GPU (`mujoco_warp`) path,
mujoco_warp allocates a fixed constraint buffer and **drops every constraint row
past `njmax` without raising anything a normal run can see** — its own
diagnostic is a `wp.printf` inside a warp kernel, discarded outright by a
GUI-subsystem `omnisim-bin.exe`. The symptom is "the physics feels wrong at
scale", with a clean log and exit code 0. AGENTS.md documents the threshold in
prose (~9 four-wheeled robots against the 256 default); until this lane it was
pinned by nothing, so nobody would notice it moving.

The rover sweep drives every wheel for the whole window: a parked robot is a
different, easier contact problem, and the credibility checklist this suite
follows forbids idle actions in a throughput number.

> The cap exists **only** on the GPU path. CPU `mj_step` grows its `efc` arrays
> dynamically, so there is nothing to truncate and the cliff sweep on
> `--solver mujoco` correctly finds nothing. The summary row says so explicitly
> rather than letting "no overflow" read as "tested and cleared".

Timing reuses `step_cost`'s two-nested-window differencing, which is what
cancels Newton's ~2 s one-time `finalizeWorld()`; every short-run average in
this repo that skipped it was inflated by roughly that amount.

---

## 4c — CPU-only

[`cpu_only.py`](cpu_only.py) hides every CUDA device from the engine
(`CUDA_VISIBLE_DEVICES=-1`) and re-runs a world with a closed-form answer.

This exists because the "hardware floor" claim in the comparison document
stated its mechanism as "via ODE", ODE was deleted, and the replacement
mechanism (CPU `mj_step`) was never re-measured — while the same sentence
appears in a work-package deliverable of an external funding commitment.

> It is **not** a test of GPU-less hardware. The machine still has a driver, a
> CUDA runtime and the GPU wheels. The honest external phrasing is "runs with
> no CUDA device visible to the process".

---

## Adding a probe

Append a `Probe(...)` to [`capabilities.py`](capabilities.py), then
`python gen_worlds.py`. Worlds are **committed**, one `.wbt` per probe: a
capability claim a third party cannot load and re-run is a self-attestation
again, which is the thing this lane exists to remove. `gen_worlds.py --check`
fails if a probe was edited without regenerating.

Write the assertion docstring as a claim a reader can check —
*"a 0.1 m sphere resting on a floor whose top face is at z=0.55 must settle at
z=0.65 ± 5 mm"*, not *"checks the sphere"*.

### Rules learned the hard way

Every one of these came from a wrong verdict this lane produced about itself
before it produced a right one about the engine.

1. **Never publish a `broken` verdict without chasing it to a mechanism.**
   `device.emitter_receiver` first scored `broken` because both radios were on
   the prober, and the engine refuses same-robot delivery *by design*
   (`// robot cannot send message to self`, `OmReceiver::transmitPacket`). The
   probe was measuring a documented rule.
2. **Guard the premise before judging the sensor.** "Never reported contact" is
   only a finding if contact demonstrably happened. The TouchSensor probes
   assert the prober's *rest height* first, and the 10 mm pad protrusion exists
   solely to make "the pad touched" and "the body touched" separable by
   measurement.
3. **Do not encode a guess about the thing you are testing.** The cylinder
   probe originally rotated the geometry "onto its side", assuming an axis
   convention, and failed by 149.6 mm — unattributable between the convention,
   the collider and the engine. Unrotated, accepting either admissible rest
   height, it passes.
4. **Remove the confound rather than reasoning around it.** `ball_motor` and
   `hinge2_motor` run at **gravity 0**: those joints swing freely when their
   motors are ignored, so under gravity "the arm moved" proves nothing. With
   gravity on, `ball_motor` scored `degraded` on 36.2 mm of pure sag.
5. **Filter NaN once, centrally.** Every `enable()`-gated device reads NaN until
   its first post-enable sample, and row 0 is recorded before the first step.
6. **An instrument failure is `inconclusive`, never `broken`.** A hung
   controller, a missing recording, an assertion that raised — none of those are
   statements about the engine.
7. **Use the existing attribution instrument.** When
   `phenomenon.friction_declared_in_world` slid, the tempting conclusion was
   "the declaration never reached the solver". `lane1/translation_audit.py`
   answered it directly — *3/3 geoms carry the declared mu 2* — and the note was
   rewritten to say what was actually measured. ⚠️ **But that only ruled out
   one wrong answer, and the lane then published the OTHER one for four days.**
   See rule 8: the declaration was fine and so was the contact; the *scene* was
   impossible.
8. **CHECK THAT THE SCENE IS PHYSICALLY POSSIBLE BEFORE YOU JUDGE THE ENGINE
   ON IT.** `phenomenon.friction_declared_in_world` published `broken` from
   2026-08-13 to 2026-08-17 on a scene that no engine could have passed. It
   dropped a **0.2 m cube** on a **55°** incline, and a block is in equilibrium
   only if **both** µ ≥ tan θ **and** tan θ < b/h. A cube has b/h = 1.0 against
   tan 55° = 1.428, so it topples at *any* friction — the probe was reading a
   toppling test as a friction test, and the 24.355 m it recorded was a tumble.
   The check is one line of statics and it was never done.

   **The cheap diagnostic, worth reaching for on any `broken`: sweep the
   variable you claim to be testing and see whether the verdict moves.** In
   bare MuJoCo the same cube travels 23.30 m at µ=2, 23.46 m at µ=10 and
   **22.29 m at µ=100**. A finding that will not move when its own variable is
   raised fiftyfold is not about that variable. Rebuilt on a low-CoM slab
   (b/h = 10, topple angle 84.3°) the capability measures `works` at
   0.0008 m — same engine, same incline, same declared µ.
9. **A green that cannot go red is not evidence — ship the failing arm with
   the passing one.** The friction probe had a passing sibling at 30°
   (`phenomenon.friction_holds_shallow_incline`) and it caught nothing, because
   30° is under a cube's 45° topple bound: two probes, one blind spot. The
   repair therefore added `phenomenon.friction_slides_below_coulomb_bound` —
   the identical slab at µ=1.3, *below* tan 55° = 1.4281 and deliberately
   *above* the engine default of 1.0 — which **must slide**, and measured
   1.4402 m against the analytic 1.442 m (ratio 0.999). It fails if the slab
   holds (friction delivered above what the world declared) *and* it fails at
   the µ=1.0 rate (the declaration ignored), so it closes both ways the
   positive arm could pass for the wrong reason. Pair a probe with an arm that
   brackets the bound from the other side, not with one that merely re-passes
   on an easier setting.

10. **A PROBE THAT MEASURES THE SUBJECT AT REST CANNOT CERTIFY A SENSOR WHOSE
    CORRECT READING AT REST IS ZERO.** `device.gyro` published `works` from
    2026-08-10 to 2026-08-17 on this evidence:

    ```
    {'omega_rad_s': [0.0, 0.0, 0.0], 'magnitude_rad_s': 0.0,
     'scope': 'static readability'}
    ```

    The prober was parked. A stationary gyro correctly reads zero, so that row
    is *equally* the signature of a working device and of a dead one — no
    engine defect could have moved it. `device.inertial_unit` was the same
    (roll/pitch/yaw all `0.0`, parked and level) and `device.gps` was a weaker
    version of it: one comparison, at one instant, on a body that never moved,
    two of whose three coordinates were `0.0` — which a GPS frozen at its
    spawn pose passes exactly.

    **This is a third failure mode, distinct from the two above, and the docs
    should name which one a verdict movement was.** Rule 8's friction probe
    measured the *wrong thing* (toppling, not friction). The ball/hinge2 rows
    moved because the *engine* changed. These three probes were **not testing
    anything** — and the gyro's own `scope` string admitted it ("accuracy
    under rotation is NOT tested by this probe") while the verdict was
    published as `works` regardless. A documented limitation does not stop a
    green from being read as a capability claim; only a falsifiable assertion
    does.

    **It was caught by cross-lane disagreement, which is worth keeping as a
    detection method.** The ROS 2 sensor lane measured a URDF Husky's `Gyro`
    reading a constant `[0,0,0]` *while the robot was rotating*. Lane 4 said
    the gyro worked; the ROS lane said it was dead; **both were honest**,
    because this probe had never asked the question.

    The repair is the general shape: **drive the subject, take the ground
    truth from the supervisor rather than from the command, guard the premise,
    and keep the old at-rest reading as the negative arm instead of deleting
    it.** All three now ride a `_turntable()` — a yaw hinge driven at
    2.0 rad/s — with a second device on a body the supervisor proves is
    motionless, so a device that returns *any* constant fails one arm or the
    other. Measured after the repair: gyro `2.0000` rad/s against a
    supervisor-measured `2.0000` (ratio 0.99999992) with the resting gyro at
    `0.0`; IMU yaw travel `3.9936` rad against the supervisor's `3.9936`, plus
    an authored `0.3` rad tilt read as `0.3000`; GPS path length `0.997` m
    against the body's `0.998` m with `max_abs_err_m = 0.0` over 500 samples.

    ⚠️ **The verdicts did not flip — and that is not a reason to skip this.**
    All three still measure `works`, so the headline stayed at 78%. What
    changed is that the rows can now go red. `python capabilities.py
    --self-test` proves they do: 14 synthetic recordings, offline, no engine,
    including the exact old evidence — re-installing the old gyro assertion
    makes the self-test report *"reads [0,0,0] while the body turns: got
    works, expected broken"*. Run it before trusting one of these greens.

11. **Mount the negative arm on a body that can actually answer.**
    `OmGyro::computeValue()` and `OmAccelerometer::computeValue()` read their
    value from `upperSolid()->bodyHandle()` and, when that is null, write **no
    value** and warn *"this node or its parents requires a 'physics' field to
    be functional"*. `OmInertialUnit` and the GPS position channel are
    computed from `matrix()` — the scene graph — and need no body at all.

    So the first draft of rule 10's negative arm, a `Gyro` parked on the
    prober Robot (which is `physics NULL` by default, the intangible-observer
    pose), would have read zero **because it had no body to ask** — a vacuous
    negative arm guarding a vacuous positive one. It hangs off a passive,
    undriven hinge instead, and `quat:REFERENCE` is recorded so "it did not
    move" is measured rather than assumed.

    That same asymmetry **explains the ROS 2 lane's finding, and localises it
    away from the device**. Measured here in one run, one rotating arm, three
    gyros: on the hinge endPoint Solid (a real Newton body) `[0, 0, 2.0000]`;
    on a nested Solid with no physics `[0, 0, 0.0]`; and on a nested Solid
    carrying the URDF importer's **exact** carrier pattern
    (`boundingObject Box { size 0.001 0.001 0.001 }` +
    `physics Physics { density -1 mass 0.001 }`,
    [`OmUrdfImporter.cpp:1103`](../../../../src/omnisim/vrml/OmUrdfImporter.cpp#L1103))
    also `[0, 0, 0.0]`, with the engine warning by name for both. **Declaring
    `physics` on a jointless nested Solid does not give it a Newton body**, so
    the importer's IMU cluster is structurally unable to serve `Gyro` or
    `Accelerometer` while its `InertialUnit` works — which is precisely the
    asymmetry the ROS lane reported. The `Gyro` device is not broken; its
    carrier is not a body.

12. **Before publishing `absent` — or leaving a capability out of the matrix
    entirely — grep `tests/*/worlds/` and `tests/smoke/smoke_worlds.json` for
    it: a SKIPPED test is evidence of known-broken.**
    `tests/api/worlds/accelerometer.omniworld` carried a red, correctly
    diagnosed assertion for years ("Parent of Accelerometer node has no
    physics" — the folded-carrier defect of rule 11) behind `skip: true` in
    the smoke set, whose `skip_reason` even named the mechanism — while this
    matrix had **no accelerometer row at all**. So a capability that was
    documented, measured and known broken was reported by the matrix as
    merely untested: a matrix that cannot see skipped tests silently
    downgrades "known broken" to "no data", which is the opposite of the
    `broken`-vs-`absent` distinction the lane exists to make. The check is
    one grep, it was first done on 2026-09-01, and it produced
    `device.accelerometer` (the device on a real body, gravity kept ON so a
    dead device's zeros cannot fake the at-rest 9.81) and
    `device.imu_nested_carrier` (the folded-carrier arm, expected broken
    today and required to go green when the carrierBodyHandle fix lands).

---

## Files

| file | role |
|---|---|
| [`capabilities.py`](capabilities.py) | the registry: probes, worlds, assertions. Run it directly to validate. |
| [`gen_worlds.py`](gen_worlds.py) | registry → committed `.wbt` (`--check` for drift) |
| [`run_coverage.py`](run_coverage.py) | 4a runner → SPEC rows |
| [`run_envelope.py`](run_envelope.py) | 4b scaling + constraint-overflow detection |
| [`cpu_only.py`](cpu_only.py) | 4c no-CUDA probe |
| [`report.py`](report.py) | rows → the measured capability matrix (markdown); `--cross` adds other machines |
| [`merge_coverage.py`](merge_coverage.py) | top up a sweep with re-measured probes, reproducibly |
| [`controllers/omnibench_prober/`](controllers/omnibench_prober/) | the one generic measurement controller |
| [`controllers/omnibench_emitter/`](controllers/omnibench_emitter/) | second robot for the radio probe |
| [`controllers/envelope_stepper/`](controllers/envelope_stepper/) | 4b step driver (reads nothing, by design) |
| [`controllers/envelope_rover/`](controllers/envelope_rover/) | 4b wheeled load |
| [`worlds/`](worlds/) | one committed `.wbt` per probe |
