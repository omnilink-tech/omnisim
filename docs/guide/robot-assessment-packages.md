# Robot-specific US$100 assessment packages

The shared package factory supports the seven existing **physical-wheel braking
adapters**. A package profile defines one robot/model and reference-controller
configuration, its experimental task requirements, payload fixtures and asset
notices. A profile alone is not a qualified delivery or evidence of hardware
accuracy. Arms, legged robots, drones and kinematic bases need their own task
adapters; importing an asset does not enable this braking package.

## Build one robot's delivery

```powershell
python -m omnisim doctor
python -m omnisim assess robots --supported
python -m omnisim assess package-profile --robot jackal
python -m omnisim assess qualify --robot jackal --workers 1 --out .tmp/jackal-qualification
python -m omnisim assess package --robot jackal --campaign .tmp/jackal-qualification --out .tmp/jackal-release
```

Use a new output directory for every attempt. `qualify` selects the profile's
speed, mounting, payloads and requirements before running the campaign. `package`
asserts the robot identity; it can infer that identity if `--robot` is omitted.
Start with one worker on a shared machine; higher concurrency must fit the host's
runtime/memory budget. Watchdog timeouts are invalid evidence, not robot failures.
Mixed-robot campaigns, wrong profiles/requirements, invalid or changed evidence,
missing transitions/confirmations and mismatched sources/runtime are refused.
A qualified campaign must contain 48 grid conditions, 12 local refinements and
four confirmations of one qualified passing/failing braking pair. Every attempt
is retained. If that coverage cannot be completed, investigate it; do not relabel
an incomplete campaign as the promised package.

Each delivery contains the selected model and its notices, assessment source,
complete evidence, robot-specific report and US$100 scope, a machine-readable
package profile, checksum inventory, Windows runtime, embedded Python and customer
launcher. Runtime selection includes the Windows activation manifests, Python
path configuration and required CUDA loader redistributables with their terms.
Customers need neither Git, separately installed Python, a CUDA toolkit nor an
AI-provider account for this independent deterministic task. OmniLink remains a
separate credential-gated experience.

## Profile parameters

All profiles use downhill slopes 0/5/10/20 degrees and imposed contact coefficients
0.10/0.20/0.40/0.80, immediate braking, 16 ms sampling and CPU Newton/MuJoCo physics.
Speed is 70% of the registered wheel command envelope, capped at 0.6 m/s. The
stopping-distance requirement is target speed squared / (2 × 0.72 m/s²), capped
at 0.25 m. Thus slower models have a corresponding distance requirement rather
than inheriting Husky's 0.25 m allowance. Settling deadline remains 1.5 s, settled
speed 0.025 m/s, minimum hold 0.4 s and observation at least 2.5 s after STOP.
These are experimental criteria, not manufacturer specifications.

| Profile | Target m/s | Distance limit m | Payload fixtures kg |
|---|---:|---:|---|
| Husky (`husky`) | 0.6 | 0.25 | 0/10/20 |
| Jackal (`jackal`) | 0.4116 | 0.117649 | 0/2/5 |
| ROSbot (`rosbot`) | 0.357 | 0.0885062 | 0/0.5/1 |
| ROSbot XL (`rosbot_xl`) | 0.4032 | 0.112896 | 0/1/2 |
| TurtleBot3 Burger (`tb3_burger`) | 0.1386 | 0.0133403 | 0/0.1/0.25 |
| TurtleBot3 Waffle (`tb3_waffle`) | 0.1386 | 0.0133403 | 0/0.25/0.5 |
| TurtleBot3 Waffle Pi (`tb3_waffle_pi`) | 0.1386 | 0.0133403 | 0/0.25/0.5 |

The rigid box is 0.30 × 0.25 × 0.20 m with its own inertia. Husky's mount center
is 0.35 m above its chassis frame; other mounts use max(0.12 m, 2.3 × registered
wheel radius). The exact mount link/position and adopted solver mass, friction
and incline are retained per case. Payloads are fixture choices, not rated
capacities. ROSbot XL uses the existing cylinder-wheel skid-steer model; lateral
mecanum motion is outside this task.

## Customer acceptance

Extract to a short writable Windows x64 folder (resolved package-root path at
most 100 characters), then use the included launcher:

```powershell
.\assessment.cmd verify
.\assessment.cmd accept --out runs\acceptance-01
.\assessment.cmd rerun case-013 --out runs\case-013-01
.\assessment.cmd suite --out runs\suite-01
```

`verify` checks the runtime/assets/source inventory and offline regrades all
64 delivered episodes. `accept` repeats the actual confirmed boundary endpoints
twice each, requiring matching PASS/FAIL outcomes, valid evidence and at most
5 mm difference in measured forward excursion. A valid expected FAIL is a
successful reproduction. Individual-case exits are 0 PASS, 1 FAIL, 2 invalid/error;
acceptance exits 0 only when its comparison checks pass.

`suite` repeats the profile's grid and bounded adaptive discovery, whose refinement
points can change if outcomes change. For exact repeated inputs, use `rerun` with
the frozen IDs/specs in evidence/campaign.json. Original evidence is kept intact;
new attempts require new directories. New model/controller/runtime versions
need a separate working copy and qualification. The standard price includes the
prepared assessment and one consolidated clarification round within 14 days;
production-controller integration and hardware correlation require separate scope.

Clean-extraction acceptance must use the actual final archive and an isolated
launch environment. Test the bundled runtime, not the developer installation.
Checksums detect changed inventoried files; they are not a digital signature.
Testing an extraction on the delivery machine does not verify independent-PC
compatibility or the physical robot. These reports do not establish exhaustive
failure coverage, structural lifetime or safety certification.

## Extend the factory

The task/profile definitions live in `omnisim/assessment/package_profiles.py`;
shared customer templates live in `projects/assessments/packages/templates/`.
Register a real task adapter/model/control mapping, define a versioned profile
with asset notices and explicit requirements, run qualification, build the
capsule and pass its extracted customer acceptance before offering that delivery.
A new robot class requires its own task, controller and grading work.

The original accepted Husky version 1.0.0 archive remains frozen. Its physical
scope is preserved by the generalized Husky profile; source changes require a
fresh qualification when building a new version. Current release evidence is
recorded under `tests/assessment/evidence/` and linked from the assessment guide.

## Qualified deliveries

Jackal version 1.1.0 passed its 64-episode sequential campaign and extracted
Windows delivery acceptance: 41 PASS / 23 FAIL, all evidence valid. Four fresh
boundary-endpoint confirmations matched their original distances exactly on the
delivery machine; an additional loaded failure/pass pair reproduced through the
customer CLI. Source and runtime came from the extracted kit, with no developer
model dependency, Git/Python on PATH or CUDA toolkit environment variables.
An earlier four-way attempt with pre-physics watchdog errors was rejected and
retained; no task requirement changed for the sequential run.

- [Jackal customer report](../../distribution/robot-assessments/jackal-1.1.0-20261028/jackal-braking-1.1.0-windows/REPORT.html)
- [Jackal Windows ZIP](../../distribution/robot-assessments/jackal-1.1.0-20261028/jackal-braking-1.1.0-windows.zip)
- [Release validation and machine attribution](../../tests/assessment/evidence/JACKAL_PACKAGE_RELEASE_20261028.json)
- [Detailed customer acceptance](../../tests/assessment/evidence/jackal-package-acceptance-1.1.0-20261028/delivery-qa.json)
- [Original accepted Husky release](../../tests/assessment/evidence/HUSKY_DELIVERY_RELEASE_20261028.json)

Measurements are attributed to Windows 11 / AMD64 machine `9722d23d12a3`.
Independent-PC and hardware validation remain unverified. The other five wheel
profiles have definitions and task adapters, not accepted version-1.1 delivery
archives; qualify and check each one before claiming it is ready to hand over.
