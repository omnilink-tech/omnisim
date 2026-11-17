# AgenticSimBench v1 pre-registration

This directory is deliberately separate from the frozen v0.3 files one level
up. `contract.json` is the machine authority; `CLAIM.md` is its short human
reading; `freeze_manifest.json` hashes both plus the guard itself.

Before the first v1 result exists, regenerate after an intentional contract
edit with:

```powershell
python tests/benchmarks/agentbench/preregister/v1/test_contract.py --write
```

After any row with `suite: agenticsimbench/v1` exists anywhere under
`tests/benchmarks/agentbench`, `--write` refuses. Any design change then needs a
new suite id.

## Red-evidence fixtures

Real simulator negative fixtures are generated separately from scored model
rows. R1 currently has four:

```powershell
python tests/benchmarks/agentbench/preregister/v1/run_red_fixture.py `
  --fixture blind `
  --work O:\omnisim\.agentbench-v1-red-work\r1-blind
python tests/benchmarks/agentbench/preregister/v1/run_red_fixture.py `
  --fixture dirty `
  --work O:\omnisim\.agentbench-v1-red-work\r1-dirty
python tests/benchmarks/agentbench/preregister/v1/run_red_fixture.py `
  --fixture undrivable `
  --work O:\omnisim\.agentbench-v1-red-work\r1-undrivable
python tests/benchmarks/agentbench/preregister/v1/run_red_fixture.py `
  --fixture altered_obstacle `
  --work O:\omnisim\.agentbench-v1-red-work\r1-altered-obstacle
```

Together they make all six R1 assertions go red for named physical reasons:
engine error, insufficient joints, altered obstacle geometry, missed goal,
named collision, and an invalid path. The generator refuses to write evidence
unless each observed red set exactly matches its registered set.

R2 uses a shipped OMNIARM6 plus a transparent six-joint measurement rig:

```powershell
python tests/benchmarks/agentbench/preregister/v1/run_r2_red_fixture.py `
  --fixture teleport `
  --work O:\omnisim\.agentbench-v1-red-work\r2-teleport
```

The registered R2 fixtures are `static`, `dirty`, `insufficient`,
`below_ground`, `teleport`, `wrong_order`, and `late`. The teleport
fixture reaches and holds all three targets inside the deadline but fails the
measured step/speed bound only; the late fixture passes every physical clause
and fails the deadline only.

R3's registered real-engine fixtures are `dirty`, `scene_tamper`,
`insufficient_arm`, `bad_start`, `no_lift`, `never_released`,
`wrong_destination`, and `teleport`:

```powershell
python tests/benchmarks/agentbench/preregister/v1/run_r3_red_fixture.py `
  --fixture wrong_destination `
  --work O:\omnisim\.agentbench-v1-red-work\r3-wrong-destination
```

The wrong-destination fixture passes every assertion except final bin
containment. The teleport fixture completes the entire lift/carry/descent/bin
story and fails only the measured speed and single-step bounds.

R4 uses a real six-joint supervisor fixture and the upstream-Webots control
world's frozen maze/table/pad geometry:

```powershell
python tests/benchmarks/agentbench/preregister/v1/run_r4_red_fixture.py `
  --fixture reacquire `
  --work O:\omnisim\.agentbench-v1-red-work\r4-reacquire
```

Its registered fixtures are `dirty`, `scene_tamper`, `insufficient`,
`bad_start`, `collision`, `no_carry`, `reacquire`, `wrong_delivery`, and
`teleport`. Three controls isolate one assertion each: `collision` fails only
R4.5, `reacquire` fails only R4.7 with two measured airborne episodes, and
`wrong_delivery` fails only R4.8 after an otherwise valid >3 m carry. Together
the nine fixtures make every R4 assertion observably red.

Build or verify the cumulative v1 coverage view:

```powershell
python tests/benchmarks/agentbench/preregister/v1/red_coverage.py --write
python tests/benchmarks/agentbench/preregister/v1/red_coverage.py --check
```

This view layers qualifying v1 evidence on the immutable Phase 0 baseline. It
does not rewrite the baseline or credit oracle/null fixtures.

The final seven controls use one driver:

```powershell
python tests/benchmarks/agentbench/preregister/v1/run_final_red_fixtures.py `
  --fixture b2_no_claim `
  --work O:\omnisim\.agentbench-v1-red-work\final-b2-no-claim
```

They close A1.10, B2.1/B2.5, B3.1/B3.3, and C2.1/C2.2. All use real engine
runs. B2 reads both Viewpoints from live Supervisor fields and resolves the
axes/FOV through the same spatial module as `/scene/viewpoint`; B3 varies only
the captured answer over live Husky geometry; C2 separates a dirty but
physically correct run from a clean dynamic fall; A1 strips the attribution
record from an otherwise-valid real 10-Husky adapter bundle and confirms the
grader marks A1.10 invalid.

Current committed view (2026-08-13): **61/61** assertions validated. This is
complete grader validation, not comparative evidence or a marketing result.

## All-primary arm ledger

The frozen claim has 10 tasks and 5 primary simulators, so its publication
surface is exactly 50 cells. Generate or verify the ledger with:

```powershell
python tests/benchmarks/agentbench/preregister/v1/arm_gates.py --write
python tests/benchmarks/agentbench/preregister/v1/arm_gates.py --check
```

The generator starts from `contract.json`, not the set of adapters that happen
to exist. Isaac, Gazebo, or Genesis therefore remain visible as ten BLOCKED
cells each until their adapters are real. A positive oracle without a
simulator-bound null control cannot turn a cell green. Live records bind the
test sources and simulator binary hashes so editing a gate requires rerunning
it.

Current committed view (2026-08-13): **3/50 READY, 47/50 BLOCKED, 0 explicitly
unsupported; 2/5 primary adapters implemented**. The READY cells are OmniSim
R1 and upstream Webots R1/R4. Webots R2025a passed the combined live suite
22/22 in 201.03 s. These are instrument gates, not scored coding-agent results.

## BuildScale freeze

`build_scale_spec.json` operationalizes the contract's five scale levels before
any scored row exists. `preregister/v1/build_scale_core.py` grades all levels through
the same nine simulator-neutral assertions. It measures authoring and physical
behaviour--not bare solver FPS--and requires a clean-directory replay of the
collected artifact.

The freeze binds the spec, core, and its discrimination tests:

```powershell
python -m unittest agentbench.graders.test_build_scale_core `
  agentbench.preregister.v1.test_build_scale_freeze
```

Current status: 15 core controls pass. The real OmniSim level-10 oracle passes
all 9/9 clauses, including a clean-directory replay, while its matched idle
null moves 0/10 robots and fails. Both raw verdict/fact documents and their
hash-bound summary are committed under `build_scale_gate_records/`.

The exhaustive primary ledger is generated from all five frozen comparators
and all five levels, so missing arms cannot disappear:

```powershell
python tests/benchmarks/agentbench/preregister/v1/build_scale_gates.py --write
python tests/benchmarks/agentbench/preregister/v1/build_scale_gates.py --check
```

Current generated view (2026-08-13): **6/25 READY, 19/25 BLOCKED, 0 scored
model rows**. OmniSim and upstream Webots R2025a both have hash-bound levels
10, 25, and 50 oracle PASS / idle-null FAIL records. Neither has a measured
frontier advantage through 50. This is live
instrument/discriminator evidence, not a coding-agent score or comparative
marketing result.
