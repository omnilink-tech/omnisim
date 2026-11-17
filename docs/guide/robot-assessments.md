# Robot mobility assessments

The [robot-specific US$100 package factory](robot-assessment-packages.md) uses
shared task profiles, customer documents, reports and runtime packaging for the
seven physical-wheel adapters. Start with `assess package-profile --robot jackal`,
then `assess qualify --robot jackal` and `assess package --robot jackal`.
Profiles define scope; each version still requires its own qualifying campaign
and extracted-delivery acceptance. The original Husky archive remains frozen.

## Deliver the fixed-scope Husky package

The [US$100 Husky delivery specification](../../projects/assessments/packages/husky/SCOPE.md)
defines one reference-controller braking assessment, its complete report/evidence,
a pinned offline Windows runtime and reusable customer tests. This scope does not
include integration of a manufacturer's production controller or hardware
validation. The price is for the prepared assessment and handover; software/model
licenses remain in force. Support includes one consolidated clarification round
within 14 days, as specified in the delivery scope.

Build from a **fresh campaign matching the current sources**, not an older
campaign with different implementation hashes:

```powershell
python -m omnisim assess sweep --robots husky --refine 12 --confirm 2 --workers 2 --max-episodes 64 --out .tmp/husky-delivery-campaign
python -m omnisim assess package --campaign .tmp/husky-delivery-campaign --out .tmp/husky-delivery-release
```

The Windows builder refuses incomplete/wrong-scope campaigns, modified evidence,
unreproduced confirmation outcomes, and mismatched source/engine/runtime hashes.
It produces a ZIP, adjacent SHA256 and expanded delivery. It bundles the engine,
its native dependency closure, embedded Python, controller API, robot assets,
source, notices and all 64 episodes. It excludes unrelated robot assets, account
configuration and private checkout content. Package only the release output,
never the full private repository. No separate Python or Git install is needed.
On the build machine, an engine that links CUDA also needs its matching CUDA
Runtime/NVRTC redistributables and toolkit EULA available for the builder to
include. Customers receive those loader libraries and their terms; they do not
need to install the CUDA toolkit for this CPU assessment.

Inside the extracted delivery, customers use:

```powershell
.\assessment.cmd verify
.\assessment.cmd accept --out runs\acceptance-01
.\assessment.cmd rerun case-013 --out runs\case-013-01
.\assessment.cmd suite --out runs\suite-01
```

`verify` checks the file inventory and offline regrades. `accept` runs four fresh
confirmations and compares outcomes and forward excursions with a 5 mm tolerance;
an expected robot failure is part of successful acceptance. The full suite
performs the grid and bounded adaptive search; it is not a comparison assertion.
See the [customer quickstart](../../projects/assessments/packages/husky/QUICKSTART.md)
and [acceptance checklist](../../projects/assessments/packages/husky/CUSTOMER-ACCEPTANCE.md).
Always validate the actual extracted archive. Extraction on the delivery machine
does not establish compatibility with an independent clean Windows computer.
Use a short writable extraction path: the resolved package root must be at most
100 characters so the capsule's files fit below the engine's Windows path limit.
The customer launcher checks this before running a simulation.

`suite` repeats discovery with bounded adaptive refinement. To repeat each exact
delivered spec, iterate the frozen case IDs with `rerun` from the extracted kit:

```powershell
$cases = (Get-Content .\evidence\campaign.json -Raw | ConvertFrom-Json).cases
foreach ($case in $cases) {
    .\assessment.cmd rerun $case.case --out ("runs\frozen-suite-01\" + $case.case)
    $expected = if ($case.status -eq "PASS") { 0 } else { 1 }
    if ($LASTEXITCODE -ne $expected) {
        throw ("Unexpected result for " + $case.case + "; retain the new evidence")
    }
}
```

This checks each case's expected outcome. Compare saved metrics for numerical
changes; `accept` additionally checks its selected pair's 5 mm tolerance.

## Find natural failures across slopes, contact friction and payloads

The schema-2 assessment varies a physical incline, the tyre/surface contact
coefficient, and a rigidly mounted payload. **Braking delay is always zero in
these campaigns.** Positive slope means downhill along the robot's forward axis;
negative values test uphill. Distances and velocities are projected along the
surface, so travelling downhill is not misclassified as falling below the floor.

```powershell
python -m omnisim doctor
python -m omnisim assess sweep --robots husky --refine 12 --confirm 2 --workers 2 --out .tmp/husky-assessment
python -m omnisim assess robots --supported
```

The default grid is 0/5/10/20° downhill × contact μ 0.10/0.20/0.40/0.80 × mounted
payload 0/10/20 kg: 48 cases at a target 0.6 m/s for Husky. `--refine` budgets
additional samples between observed pass/fail friction settings at fixed robot,
slope and load. `--confirm` repeats each side of one qualified braking transition
in fresh engines; first confirmations also capture native stills. Each case keeps
its own unchanged requirement: at most 0.25 m forward excursion after STOP,
settled within 1.5 s. These are experimental requirements, not manufacturer limits.

Open `<output>/report.html` for every case and the observed transition brackets.
`campaign.json` preserves the axes, all attempts, failure modes, per-case specs,
refinement and confirmation relationships. This is structured failure discovery,
not a statistical estimate of deployment reliability or a certified safety map.
No monotonicity is assumed beyond the two sampled endpoints of a reported bracket.

The payload is a box with specified mass, inertia and a fixed joint to the chassis
link. Its default center is 0.35 m above Husky's chassis-frame origin; it is not
loose cargo or a vendor-rated load test. Vary `payload_height_m` and
`payload_offset_x_m` in an individual schema-2 spec for another mounting position.
Wheel-driven models use their existing geometry/motor configuration and their own
command envelope; fleet sweeps choose 70% of that envelope, capped at 0.6 m/s.

Every generated case verifies the **actual compiled solver model**: total mass
(including the importer's documented root mass where applicable), incline
orientation, and friction on both the floor and the robot's colliders. Husky's
imported tyre friction would otherwise override a low world-friction setting.
Only the generated model copy is changed; vendor source assets remain untouched.
`adoption.json` and `compiled-model.txt` retain these checks. A failed adoption
check is ERROR, not a robot failure. Keep the log and dump files when copying
evidence: their hashes are verified on offline regrading.

The task distinguishes acquiring/holding the requested speed from braking after
a qualified approach. Sliding before the drive, inability to reach speed, tipping
and departure are valid task failures, separately labelled from braking-distance
violations. A missing trace, sensor, contact observation, or incomplete episode
remains inconclusive/error. A model that never moves cannot pass.

## Coverage and the rollout to other robot classes

```powershell
python -m omnisim assess robots
python -m omnisim assess sweep --robots all --slopes 0 5 --frictions 0.2 0.8 --payloads 0 --workers 2 --out .tmp/mobile-check
```

`all` means all **registered physical-wheel braking adapters**: Husky, Jackal,
ROSbot, ROSbot XL, and TurtleBot3 Burger/Waffle/Waffle Pi. The adapter handles
two- and four-wheel layouts; geometry and motor names come from the existing
mobile configurations. ROSbot XL is assessed as its current cylinder-wheel
skid-steer model, not mecanum strafing. Availability of an adapter is distinct
from qualification of a particular case or agreement with hardware.

The unfiltered inventory covers URDF model files across `projects/robots/`,
including variants. It explicitly marks other models as `asset_only` with no
braking task. Legged robots need a locomotion/posture controller and fall/recovery
criteria; arms need manipulation/load tasks; drones need flight controllers and
flight criteria. Kinematic OmniTug is excluded from physical braking. Do not
advertise those classes as assessed merely because their assets are present.

Adding another wheeled model uses the existing mobile-config and chat-world
entry, followed by independent generated-model checks and live qualification.
Other classes require a corresponding task/controller adapter rather than
passing them through a wheeled-braking oracle. This capability inventory is the
extension boundary for the broader [service plan](../developer/robot-robustness-service-plan.md).

Measured campaigns and complete attempts:

- [Husky slope/friction/payload campaign](../../tests/assessment/evidence/husky-envelope-20261028/campaign.json).
- [Physical wheeled-model coverage](../../tests/assessment/evidence/mobile-coverage-20261028/campaign.json).
- [Mounted-payload coverage on all seven wheeled configurations](../../tests/assessment/evidence/mobile-payload-coverage-20261028/README.md).

## Original flat-floor fault-injection proof (schema 1)

The original assessment slice measures an unloaded Husky model driven by a dedicated
deterministic wheel-velocity controller. It detects an injected stop-command
delay, reproduces it in a fresh engine, and verifies restoring immediate braking.
This assesses that simulation controller, not Clearpath firmware, hardware, or
the manufacturer's rated stopping distance.

## Run the complete proof

From the repository root, first check the local engine:

```powershell
python -m omnisim doctor
python -m omnisim assess prove --capture --out tests/assessment/evidence/my-new-braking-proof
```

The output directory must not already exist. Every episode starts a fresh owned
engine and controller. The proof runs five immediate-braking baselines, one
delayed-braking failure, a fresh reproduction of that exact failure, immediate
braking restored at the same speed, and confirmation at a separately specified
0.5 m/s. The requirement is unchanged between failure and repair.

`--capture` enables native rendering for the failure, reproduction and repair;
each captures its own request and final still. Other episodes run without
rendering. The report labels the runs and does not substitute footage from one
run for another. Omit the flag for measurements without native stills.

Open `<output>/report.html` for the comparison and measured curves, and
`<output>/proof.json` for machine-readable proof checks. Each case includes:

- `spec.json`: frozen task and controller-delay configuration, with units.
- `manifest.json`: source/model/engine/runtime hashes, process ownership,
  lifecycle outcome, and hashes of recorded evidence.
- `commands.jsonl`: stop-request and brake-application events on the engine clock.
- `trace.jsonl`: one sample per 16 ms, including root pose/velocity, wheel
  positions, contact count and the wheel command adopted for the following step.
- `completion.json`, `engine.log.newton.json`, `engine.log`, `stdout.log`, and
  `fingerprint.json`: completeness, physics proof, logs and machine attribution.
- `verdict.json`, `report.html`, `braking.svg`: derived outcomes and plots.
- `stop-request.jpg` and `stopped.jpg` when capture was enabled and succeeded.

The controller uses motor velocity commands only. It never writes the robot's
pose, velocity, or a supporting force. It reuses Husky wheel geometry and motor
names from the existing mobile configuration. This is an independent deterministic
assessment controller, not OmniLink or a fallback after an OmniLink error; it
does not call a language model or require an OmniKey. OmniLink's credential and
gating requirements remain in force when using OmniLink itself.

## Requirement and validity checks

The default target speed is 0.6 m/s. A stop request requires at least 0.3 m of
measured travel and 0.3 s within the specified achieved-speed band. Starting from
that request, the robot must stay within **0.25 m forward excursion**, settle
below **0.025 m/s** within **1.5 s**, and remain settled for at least **0.4 s**
and through the remaining observation. It is observed for at least 2.5 s after
the request. Rollback over 0.05 m also fails. These are experimental task
requirements, not vendor limits or a safety certification.

The offline oracle requires complete, finite, contiguous samples; a clean
controller completion; finalized non-degraded Newton CPU physics; stable
approach speed; positive travel; all four measured wheel rotations; measured
contacts; and a supported, upright straight-line trajectory. Absent evidence
produces INCONCLUSIVE. Launch/cleanup problems or modified evidence produce
ERROR. A motionless robot cannot pass. A valid observed requirement violation
produces FAIL. Grading uses measured motion, not the configured delay or an
expected outcome label.

The world fixes Newton/MuJoCo on CPU, `newtonGroundMu 0.8`, 16 ms controller steps
and 8 physics substeps. Samples describe the controller's engine-clock instants;
the recorded command delay is the interval until the controller adopts its
braking command, not a hardware latency measurement. The test does not measure
thermal behavior, material damage, sensor robustness, or a general safe operating
envelope. Sampled data cannot rule out events shorter than the sampling period.

## Individual cases, reproduction and offline grading

```powershell
python -m omnisim assess run --out tests/assessment/evidence/my-new-baseline
python -m omnisim assess run --spec tests/assessment/fixtures/delayed_stop.json --out tests/assessment/evidence/my-new-delay
python -m omnisim assess rerun tests/assessment/evidence/my-new-delay --out tests/assessment/evidence/my-new-reproduction
python -m omnisim assess regrade tests/assessment/evidence/my-new-reproduction
```

Exit codes: `run`, `rerun` and `regrade` return 0 for PASS, 1 for FAIL, and 2 for
invalid/inconclusive/error cases. `prove` returns 0 only if the planned outcomes,
offline regrades, reproduction tolerance and matched repair conditions all pass;
an expected individual failure is therefore part of a successful proof.

`rerun` verifies source/model, engine and controller/runtime hashes before
launching. It requires the matching checkout and installed runtime; this first
slice is not a standalone simulator export or full-state checkpoint system.
Saved worlds use repository-relative model paths, so execute through the CLI,
not by moving the copied world out of its project and opening it directly.
`regrade` launches no engine and does not overwrite the original verdict. Preserve
all attempts; reruns use new output directories and link to their parent case.

The frozen proof is recorded under
[`tests/assessment/evidence/husky-braking-20261028/`](../../tests/assessment/evidence/husky-braking-20261028/).
Its `proof.json` records what was actually verified; incomplete or failed proof
checks must not be described as a successful demonstration.

## Development verification

```powershell
python -m pytest tests/assessment/test_braking.py -q --confcutdir=tests/assessment
```

The analytic traces test the request-relative distance oracle independently of
the injected delay setting, including missing physics, frozen motion/wheels,
absent contacts, broken sampling, missing settling, and changed evidence. The
live proof checks the real motor-to-physics-to-measurement path. The broader
[service plan](../developer/robot-robustness-service-plan.md) remains proposed;
general stateful sensor/actuator fault injection and hardware correlation remain
outside the shipped mobility assessment. Schema-2 campaigns add physical terrain,
payload variation and sampled boundary refinement as described above.
