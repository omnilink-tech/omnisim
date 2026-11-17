# Husky on mixed rigid surfaces

This reference assessment asks two questions: can Husky stop within the fixed
excursion limit after a friction crossing, and can it follow a 90-degree turn?
It compares continuing at 0.6 m/s with a mapped-surface speed cap of 0.35 m/s.
The cap starts before the patch, at x=0.45 m. It reduces throughput; it does not
increase friction or establish a general controller repair.

The same robot, reference feedback driver, motor limits, initial pose and Newton
physics are used for both speed policies. There is no injected braking delay,
external force, support or robot pose/velocity write. The driver measures body
speed and yaw rate instead of assuming wheel differential equals body yaw.
This is a reference driver, not Clearpath production firmware.

## Run a pair

Run `python -m omnisim doctor` first. A working engine and Newton runtime are
required. Windows has the prebuilt package; Linux requires a source build.
The public v9.2.7 release needs the separately published mixed-surface source
add-on. Use a fresh writable OmniSim installation with the full Husky assets.

```powershell
python -m omnisim assess mixed-run --spec projects/assessments/mixed/examples/brake-before.json --out .tmp/mixed-brake-before-01
python -m omnisim assess mixed-run --spec projects/assessments/mixed/examples/brake-after.json --out .tmp/mixed-brake-after-01
python -m omnisim assess mixed-run --spec projects/assessments/mixed/examples/turn-before.json --out .tmp/mixed-turn-before-01
python -m omnisim assess mixed-run --spec projects/assessments/mixed/examples/turn-after.json --out .tmp/mixed-turn-after-01
python -m omnisim assess mixed-regrade .tmp/mixed-brake-before-01
python -m omnisim assess mixed-rerun .tmp/mixed-brake-before-01 --out .tmp/mixed-brake-repeat-01
```

Exit 0 is PASS, 1 is a valid simulated FAIL, and 2 is invalid/error. Use a new
output directory for every attempt. Rerun checks source, engine and runtime
hashes. Restore the frozen sources if they differ. These are fresh physics
executions, not full-state checkpoint replay.

## Physical fixtures

- Crossing: high-friction approach, then a full-width low-friction patch.
- Split: high-friction approach, then low friction on the left and high friction
  on the right. The grid turns left.
- Controls: uniform high or low friction across the whole floor.

The grid uses high μ 0.8 and low μ 0.04/0.10/0.20. The surface boundary is
x=1.2 m and the task trigger x=1.9 m, beyond the rear axle's crossing. Every
episode observes 24 s at 16 ms with eight configured Newton/MuJoCo CPU substeps.
The payload is a 0.30 × 0.25 × 0.20 m box, rigidly mounted 0.35 m above the
Husky chassis frame, with its own mass and inertia. Experimental loads are not
manufacturer-rated capacities.

Compiled solver dumps verify collider positions, dimensions, friction and total
mass. Robot geoms use μ 0.01 on both sides; with equal explicit friction
priority, MuJoCo combines the robot and patch coefficients by maximum, so the
patch determines contact μ. The same compiled physical model is required for
each selected before/after pair. No friction is changed during a run.

These are **rigid contact patches with imposed coefficients**, not calibrated
concrete, wet flooring, ice, gravel, sand, mud or deformable soil.

## Frozen requirements

Braking requires a qualified 0.3 s approach at the selected operating speed;
maximum forward excursion after immediate STOP ≤0.25 m, lateral excursion
≤0.15 m, rollback ≤0.05 m, and linear speed ≤0.025 m/s plus yaw rate ≤0.05 rad/s
within 2 s, sustained for at least 0.5 s and through the remaining observation.

Turning requires a 90-degree arc of radius 1 m, maximum distance from that arc
≤0.30 m, final goal position error ≤0.25 m, final heading error ≤0.15 rad, actual
completion and a stopped final second. Position/path errors are recomputed
offline from the measured poses. A driver emergency stop after 1 m path error
limits runaway; it does not convert a failed task into a pass.

Both tasks require stability and physical evidence validity. Leaving the finite
floor is an ERROR, not a claimed material-induced tip. Missing runtime,
incomplete/nonfinite/noncontiguous evidence or altered artifacts are errors.

## Qualification protocol

`scripts/dev/qualify_husky_mixed.py` freezes sources, all specs and prospective
inputs before running:

1. 48 grid episodes: two tasks × two layouts × two loads (0/10 kg) × three low
   coefficients × two speed policies.
2. Eight uniform-floor controls: two tasks × high/low × two policies.
3. Eight fresh selected-endpoint confirmations: both task pairs, twice each.
4. Twelve prospective episodes: three new condition sets, both tasks and policies.

The new sets use μ 0.06/0.12/0.15, 5/15 kg, changed boundaries, trigger positions
and radii, and one right-turn input. They were declared before qualification and
are not used to tune the candidate. Development probes remain separately in
`tests/assessment/evidence/husky-mixed-20261029/`.

```powershell
python scripts/dev/qualify_husky_mixed.py --out .tmp/husky-mixed-qualification-01 --workers 2
```

The selected crossing improvement is bounded. Split-surface turning failures
remain in the report even where braking succeeds. Sampled outcomes do not define
a continuous safe operating region or a field failure probability. Physical
robot performance, production firmware and independent-PC compatibility remain
unverified.

The native comparison video uses the same fixed world camera and fixed crop on
both sides, at 1× speed. Its braking chapter shows scene 0–10 s; turning shows
scene 0–24 s. All source trials are measured for 24 s, with metrics and source
trace hashes retained independently of the film.

Measurements are attributed to machine `9722d23d12a3`, Windows AMD64,
Newton 1.5.0 / MuJoCo 3.11 CPU. The website report, source add-on, complete evidence
archive and prepared Windows kit are separate versioned artifacts.

## Measured qualification, 29 October 2026

[All 76 specs and outcomes](../../tests/assessment/evidence/husky-mixed-20261029/qualification-v1/campaign.json):
50 PASS / 26 FAIL / zero ERROR. The 48-case grid has 32 PASS / 16 FAIL. All four
uniform-high-floor controls pass. Both selected task endpoints reproduce twice
with the same checked metrics. The physical solver dump matches exactly between
the two speed policies in each selected pair.

| Selected crossing, flat and unloaded | 0.6 m/s cruise | 0.35 m/s cap |
| --- | --- | --- |
| Maximum forward braking excursion | 0.458186 m, FAIL | 0.155788 m, PASS |
| Maximum distance from the turning arc | 0.962819 m, FAIL | 0.108939 m, PASS |

The selected case IDs are brake `case-000`/`case-001` and turn
`case-024`/`case-025`. Five of six capped prospective cases pass. The seven
remaining capped failures are `case-037`, `039`, `041`, `043`, `045`, `047`
and `071`; all are split-surface turning. They remain failures in the archived v1 report; the separate repair below addresses them.

The native captures match all four selected trace files byte for byte. Public
evidence verification checks 688 files and offline regrades every episode.
The prepared Windows capsule has 9,848 inventoried files and eight fresh
endpoint acceptance runs, with all checked metric differences zero on this
machine. Its final documentation-only repack corrects report text encoding;
every executable, assessment-source, runtime and evidence byte is unchanged.
The actual final ZIP was freshly extracted, verified and offline regraded;
a modified trace was rejected with exit 2 and restored exactly.

See the [published mixed-surface report](https://www.omnilink-agents.com/services/robot-assessments/mixed/husky-v1/report.html)
and [comparison video and public rerun instructions](https://www.omnilink-agents.com/services/robot-assessments#mixed-surfaces).

## Split-surface reference repair — qualification v2

The [93-episode repair protocol](../../tests/assessment/evidence/husky-split-repair-20261029/qualification-v2/campaign.json)
repairs all seven published capped-driver failures under the original turn
requirements. It adds a 0.30 m pre-trigger lateral corridor for both variants.
The intervention combines independent wheel command limits with a mapped
approach and route, a lower 0.18 m/s post-trigger speed target, and completion
only after the endpoint has settled. The nominal approach cap remains 0.35 m/s.
Pre-alignment changes pose and heading at the turn trigger: this is a combined
route-and-control repair with a throughput tradeoff, not an isolated controller
gain change. It assumes localization and a known surface boundary. Only
split-surface left turns activate the repair; other branches retain the old driver.

All seven old endpoints remain FAIL; all seven repaired endpoints PASS. Each
endpoint repeats twice in fresh engines with identical checked metrics.
All 27 unique formerly passing capped inputs pass. New low-side coefficients
0.025/0.065/0.14/0.30 crossed with rigid loads 2.5/7.5/17.5 kg produce 24 paired
prospective runs: 11 of 12 repaired inputs PASS, while all 12 old-driver inputs
FAIL. Total: 59 PASS / 34 FAIL / zero ERROR; 59 of 60 repaired-driver episodes PASS.
The source and complete plan were frozen before qualification, and prospective
inputs were opened after repair, confirmation and regression runs. No tuning
uses these results. Development attempts are retained separately.

The remaining repaired-driver failure is `case-092`: low-side mu 0.30,
17.5 kg. Its last 0.3 s approach briefly reaches 0.420247 m/s, above the fixed
0.420000 m/s upper bound. Turning path, endpoint, heading and stopped-state
checks pass, but the whole episode stays FAIL. No threshold was relaxed.
This is a sampled limitation, not a continuous operating boundary.

The selected hard split, low-side mu 0.04 and no payload, changes maximum path
error from 1.095 m to 0.254 m; the repaired endpoint finishes at 14.592 s,
0.131 m from the goal with 0.00543 rad heading error, then remains stopped.
Both matched variants share the same compiled physical-model hash.
Machine `9722d23d12a3`, Newton 1.5.0 / MuJoCo 3.11 CPU, 16 ms observations,
eight configured substeps. No physical robot or independent-PC validation.

```powershell
python -m omnisim assess mixed-run --spec projects/assessments/mixed/examples/case-037-before.json --out .tmp/split-before-01
python -m omnisim assess mixed-run --spec projects/assessments/mixed/examples/case-037-after.json --out .tmp/split-after-01
python scripts/dev/qualify_husky_split_repair.py --out .tmp/split-qualification-01
```

The public source add-on supplies all 14 repair endpoints and a complete frozen
suite. Use a separate fresh public OmniSim root with full Husky assets. The
original v1 source/runtime and evidence remain separate and unchanged.
See the [repair report](https://www.omnilink-agents.com/services/robot-assessments/mixed/husky-v2/report.html)
and [full comparison and rerun instructions](https://www.omnilink-agents.com/services/robot-assessments#mixed-surfaces).

The new native comparison shows the full 24 seconds at 1x speed; both fresh
capture traces match the selected frozen traces byte for byte. The public
verifier checks 841 files and independently regrades all 93 episodes. A separate
post-qualification repeat of case-092 reproduces its trace and metrics exactly.

Prepared Windows kit 1.1.0: 10,071 inventoried files, 343,472,167-byte ZIP,
SHA256 `392300ba830c74493a237ede9eea89f8f5f7f0e7dd4d027390912dd86ec1d298`.
The actual ZIP was freshly extracted and verified with isolated PATH and fresh
app-data folders. All 14 repair endpoints repeated twice (28 fresh runs), with
zero difference in checked metrics. A modified trace was rejected with exit 2
and restored exactly. This remains delivery-machine validation only.
