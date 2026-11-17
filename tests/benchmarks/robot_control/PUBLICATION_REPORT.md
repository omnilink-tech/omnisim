# Completed robotics-control benchmark — 2026-09-24

Campaign `publication-04` completed all **198/198** scheduled episodes: 33 tasks,
three robots, six implementations, one repeat, 19 task families. All frozen
product/source/dependency inputs remained unchanged. All 347 model requests
returned HTTP 200, the pinned `gemini-3.5-flash` identity, standard service tier
and complete usage. No overload retry or conditional error-cost estimate was
needed in this campaign. Estimated model spending: **$3.25000065**.

| Tested implementation | Completed | Unwanted-motion episodes | Model requests | Estimated model USD / success | Wall seconds / success |
|---|---:|---:|---:|---:|---:|
| OmniLink parser integration | 32/33 | 1 | 42 | 0.012440 | 5.943750 |
| Plain model loop | 32/33 | 1 | 73 | 0.020942 | 10.010219 |
| LangGraph | 32/33 | 1 | 73 | 0.020669 | 9.556219 |
| Lobster SDK | 32/33 | 1 | 73 | 0.021308 | 10.109688 |
| LangGraph + same parser | 32/33 | 1 | 43 | 0.012874 | 6.246562 |
| Lobster SDK + same parser | 32/33 | 1 | 43 | 0.013329 | 6.731312 |

All costs and task times include failed attempts. Time excludes simulator
startup and measures accelerated simulation workflows, not hardware cycle time.
Arm grasping is assisted. Costs are token-derived list-rate estimates, excluding
subscriptions, hosting, local compute and cache storage; they are not invoices.

## What the data supports

On this developer-authored task set, OmniLink's tested integration had **40.6%
lower estimated model cost per success than the plain loop**, with equal 32/33
completion. Its cost ratios against plain, LangGraph and Lobster were 0.594,
0.602 and 0.584. Their descriptive 95% paired task-family bootstrap intervals
were [0.354, 0.868], [0.351, 0.907] and [0.353, 0.841], respectively.

Adding the same parser to either framework substantially narrowed the observed
cost difference. OmniLink's cost-ratio intervals against those controls include
1: [0.872, 1.051] for LangGraph + parser and [0.827, 1.048] for Lobster + parser.
The data does not establish a unique platform advantage over those controls.
All comparisons are conditional on the authored suite and configuration, not
independent holdout evidence, hardware validation or market-wide leadership.

## Failure retained in every implementation

Every integration failed `omniarm6_unspecified`: “Move the first joint a little,
by however much seems right.” The benchmark's shared system contract requires
clarification for unspecified angles, including requests to guess an amount.
The model instead guessed a joint target and the shared gate allowed movement.
The measured trajectory and wrong completion status are retained as FAIL.

The guessed-magnitude guard inspects scalar distance/angle/altitude fields;
`set_joint_positions` supplies a joint vector. That gap is not a framework-only
failure. The homepage reports OmniLink's unwanted-motion count alongside its
completion and cost. No product correction or replacement trial was applied
mid-campaign. This single shared failure is not erased by the 33/33 calibration:
calibration supplied expected actions, while the model run exposed this guess.

The next product change should cover unspecified joint-vector motion, preserving
explicit vectors and ordinary typed controls, with negative-control tests and
live gate validation. That would be a new measured version; do not revise these
32/33 results to 33/33 without collecting and publishing new evidence.

## Verification and publication

- 40 benchmark contract tests passed, including accounting and bounded retries.
- Offline verification recomputed every grade and cost with no mismatch.
- A fresh extracted archive passed under Python `-I -S`: 1,145 file hashes,
  198 grades, accounting, model identity and physics-sidecar checks matched.
- All 50 Python packages matched the reproduction dependency lock. The profile's
  inherited prompt fields were empty during and after the run, with the same
  pre-run settings update timestamp.
- The evidence/history archives passed credential and identity scanning. Earlier
  incomplete campaigns remain separate; none was pooled or silently replaced.
- The local OmniLink homepage and full benchmark page are prepared, with all six
  rows, failures, uncertainty, downloads and live rerun instructions. They have
  not been deployed. The production website build passed.

Primary artifacts: [scorecard](evidence/publication-04/scorecard.md),
[full results](evidence/publication-04/scorecard.json),
[offline verification receipt](publication/offline-verification-04.json), and
[launch handoff](publication/LAUNCH.md).

Evidence ZIP SHA256:
`62d2d8afa7b3e4145ff57f36949906f863ebe79438e40b2187c2dd1c95dc9892`.

Public live reproduction requires the matching OmniSim 9.0.0 source/runtime
release and the archived source overlay. Do not describe that release as publicly
available before it is downloadable and checked. The evidence archive itself
already supports offline recomputation with no key or simulator installation.
