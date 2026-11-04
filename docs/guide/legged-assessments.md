# Go2 transition and G1 standing assessments

These are two defined reference-controller tasks, alongside the separate
[wheeled braking assessment](robot-assessments.md). They identify a failure,
evaluate bounded settings, and retain fresh regression cases. They do not
qualify every behavior of a robot or run manufacturer firmware.

## Current Go2 assessment v1.1.0

The earlier 15 s request failed at scene time 17.232 s with all four feet in
contact at handover. The guard allowed body tilt up to 0.25 rad; it handed over
at 0.2047 rad. The update tightens that handover limit to 0.12 rad. Gait support
and observed foot checks, the 1-tick blend, robot, learned policy, actuator gains
and physics remain unchanged. The two compiled solver models have identical hashes.

The 17-episode protocol contains two matched 15 s comparisons, four fresh
confirmations, seven candidate regressions at 8/9/10/11/12/13/14 s and four
prospective checks at 6.25/9.45/12.35/14.75 s. The source and candidate were frozen
before the new timings were evaluated. All 14 candidate trials passed; all
three previous-guard 15 s trials failed. No invalid runs occurred in this protocol.
The updated matched trial stood for 5.712 s, resumed walking for 9.504 s and
advanced another 4.096 m, with no fall through the 32 s observation.

The scheduled stand window is 6 s; the safe gate may defer entry. Candidate
verification additionally requires >=5 s actual standing, >=3 s return walking
and >=0.5 m return progress. Missing handover timing evidence is ERROR; skipping
these stages is FAIL. Original fall, observation and progress requirements remain.
No support forces or root pose/velocity writes are introduced.

Measured 2026-10-29 on Windows machine `9722d23d12a3`, Newton 1.5.0 / MuJoCo 3.11
CPU, 16 ms observations, configured Go2 newtonSubsteps 8, flat floor/contact μ 2.0,
no added payload or disturbance. Results do not certify other timings, terrain,
payloads, production firmware or hardware. Windows has the prebuilt engine;
Linux requires a source build.

[Full qualification and frozen source](../../tests/assessment/evidence/go2-handover-fix-20261029/qualification-v2/campaign.json).
The public source add-on's frozen-suite command repeats all 17 current Go2 cases;
the original generic discovery command remains a different 16-case template.

## Earlier study v1.0.0 and unchanged G1 standing assessment

| Reference model | Protocol | Selected example |
| --- | --- | --- |
| Unitree Go2 | 8 timing/configuration grid runs, 4 confirmations, 4 development timing checks, 2 prospective candidate checks | Unconditional 30-tick handover falls at 14.272 s; contact/support checks combined with a 1-tick blend complete the 32 s walk/stand/walk task. |
| Unitree G1 | 6 posture trims, 4 confirmations, 4 longer-duration held-out runs | 0 rad ankle trim falls at 1.728 s; −0.10 rad holds the 32 s standing task and the 48/60 s checks. |

Measured 2026-10-29 on Windows 11 AMD64, machine `9722d23d12a3`, Newton 1.5.0 /
MuJoCo 3.11 CPU. Observation interval 16 ms; Go2 eight physics substeps, G1 four.
The earlier Go2 protocol contains 14 PASS / 4 FAIL; G1 6 PASS / 8 FAIL. Constructed
counts are not a field failure probability. Go2 includes development timing checks at 11/13 s, already used during tuning. A frozen prospective plan tested 9/15 s: 9 s passed, while 15 s fell at 17.232 s. The change improves the shown 12 s request but remains a bounded comparison with a known failure, not a general transition solution. G1 selection uses the grid before its longer-duration checks.

Go2 uses the existing learned walking policy, with a guard requiring the nominal
support phase, four observed calf/foot contacts and tilt below 0.25 rad. The
selected change also shortens the blend from 30 ticks to one; this comparison
does not isolate the contribution of each change. A previous phase-only guard
was rejected after a held-out timing failed. Two final-campaign confirmations
hit the wall watchdog; the invalid attempts are retained under `rejected/` and
identical-input retries filled those protocol slots. No valid robot FAIL was
discarded. The trial uses the reference model's existing simplified collision
configuration, not a full-body collision-geometry validation.

G1 uses the existing standing controller and its `g1_armsdown.json` reference
specification. Only ankle trim changes between the selected pair; actuator
gains and balance feedback remain fixed. The selected 32 s run ends with pelvis
height 0.761 m, peak body tilt 0.0261 rad and two-foot-only final support.
This is physical **standing without a support harness**. It does not establish
unassisted walking; the separate authored humanoid walking demonstrations use a
weight-bearing balance harness. Squat pilots failed and are not a qualified task.

Both tasks initialize the same joint pose once, then use motor control. Neither
uses root pose/velocity writes, external pushes or weight-bearing assistance.
The observer reads motion and contacts; camera tracking does not move the robot.

## Run a fresh comparison in this checkout

```powershell
python -m omnisim doctor
python -m omnisim assess legged-run --spec projects/assessments/legged/specs/go2-baseline.json --out .tmp/go2-before-01
python -m omnisim assess legged-run --spec projects/assessments/legged/specs/go2-candidate.json --out .tmp/go2-after-01
python -m omnisim assess legged-run --spec projects/assessments/legged/specs/g1-stand-baseline.json --out .tmp/g1-before-01
python -m omnisim assess legged-run --spec projects/assessments/legged/specs/g1-stand-candidate.json --out .tmp/g1-after-01
```

Use a new output directory each time. Exit 0 is PASS, 1 is a valid task FAIL,
2 is invalid/error. A working engine/physics runtime, controller Python with
NumPy/ONNX Runtime, and runner Python with psutil are required. Windows has the
only prebuilt engine; Linux needs a source build. Stop on doctor failure.

To repeat discovery, use `legged-campaign --robot go2` or `--robot g1` with a new
`--out` directory. Keep assessment/model/controller files unchanged during a
campaign. Each run records source/runtime hashes, a trace, physical solver
sidecar, model dump, task completion and verdict. Native `--capture` records
frames; it is a fresh execution, not full-state replay.

## Evidence and handover

- [Final Go2 campaign](../../tests/assessment/evidence/legged-20261029/go2-transition-final/campaign.json).
- [G1 standing campaign](../../tests/assessment/evidence/legged-20261029/g1-standing-v1/campaign.json).
- [OmniLink service and public source add-on](https://www.omnilink-agents.com/services/robot-assessments#improvements).

Public OmniSim v9.2.7 does not include this assessment CLI; the service page
supplies a source-only add-on and separate instructions. New builds can give
different results. The $100 service prepares one reference model/task report,
evidence and a Windows runtime with verify, accept and frozen-case rerun commands.
Custom model/controller integration, new training, hardware correlation and
unlimited optimization require separate scope. These are independent simulated
reference configurations, not robot manufacturer endorsement or physical safety
certification.

The generic Go2 discovery command produces the original 16-case template. The published 18-case protocol adds two later prospective checks and reclassifies 11/13 s as development checks. The public source add-on supplies `legged-frozen-suite.py` to repeat all 18 Go2 or 14 G1 frozen conditions. Expected FAIL cases count as successful reproductions; changed labels or invalid runs return exit 2.
