# Publication campaign 01 — fixed before model calls

This campaign publishes descriptive results on the complete developer-authored
robot-control-development-v1 suite. It does not claim an independent holdout,
universal market leadership, or a full hosted-product comparison. Earlier pilots
in this directory informed product fixes; their results remain available.

## Fixed design

- 33 tasks, three repeats, six implementations: **594 episodes**, 99 per arm.
- Every task in `suites/development.json` is included, with equal episode weight.
- The implementations are OmniLink parser integration, plain model loop,
  LangGraph, Lobster SDK, LangGraph with the same parser, and Lobster with the
  same parser. All share the model, tool gate, robot fixtures and action budgets.
- Freeze: `locks/publication-01.json`. Evidence: `evidence/publication-01`.
- Random order and seed, model, rates, source hashes and machine are in the lock.
- Use the same Gemini 3.1 Flash-Lite model and entitled HuskySwarm profile as
  development. Estimated token rates per million: input $0.25, cached input
  $0.025, output including thoughts $1.50. These are reference rates, not an
  OmniLink bill or total cost of ownership. Price source:
  https://ai.google.dev/gemini-api/docs/pricing (checked 2026-09-24).
- $4.50 soft stop-new-request budget, leaving room under the previously stated
  $5 development target. Unknown usage/model changes stop further calls.
- Run sequentially on the same Windows machine, with a fresh engine per episode.
  No additional benchmark or calibration runs concurrently on this machine.

## Stopping and inclusion

Stop after all 594 episodes, or on the runner's recorded interruption/budget or
infrastructure condition. Never stop for a favorable score. Never add trials,
exclude failures, replace trials or change product/adapters/oracles after looking
at results. If a material defect invalidates the campaign, preserve the complete
attempt and disclose why; a replacement requires a new named protocol and lock.
An incomplete campaign can be shown only as incomplete, with no headline ranking.

PASS, FAIL and ERROR remain in the denominator. All model spending and task time
including failures count toward cost/time per success. Publish every arm, task,
repeat, failure, model request and measured trace. Do not pool pilot versions.

## Analysis and page rules

Use the existing SPEC.md family-cluster paired bootstrap and frozen relative
rule. Intervals are descriptive, conditional on these authored tasks. Three
repeats do not turn correlated tasks into 99 independent problem families.
Unadjusted pairwise intervals do not establish a multiple-comparison-wide winner.
The suite remains development, so the automated leadership gate stays blocked.

The homepage may report the measured completion fraction and observed costs on
this named suite, linking the full six-arm table, protocol and evidence. Any
relative figure must name its specific implementation baseline and disclose that
tasks were used during development. Publish ties/losses and parser controls with
equal visibility. No "best", "beats everyone", hardware safety, generalization,
or product-level total-cost claim is permitted by this campaign.

Timing covers accelerated simulation workflows and excludes simulator startup.
Arm grasping is assisted. No observed unwanted motion is not a safety guarantee;
the observer samples wall time and tool boundaries, not every physics sub-step.

## Reproduction deliverable

Provide immutable raw rows, lock, source snapshots, physics logs, an archive
manifest, offline regrading/accounting verification, and commands for a new live
run. Include the uncommitted benchmark/product changes needed on the base commit.
External reruns require an OmniKey and an entitled profile; model answers, service
state and wall times need not be bit-identical. Independent replication has not
yet occurred and must not be implied by "reproducible".
