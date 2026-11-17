# Next capability milestone: reliable completion under disturbances

Owner decision: 2026-09-24. Publish the completed, verified development results
now. This document records subsequent work; it does not authorize another paid
campaign or announce capabilities as shipped.

## What the current evidence establishes

The cached Opus campaign (`opus-cached-comparison-01`) covers 33 development
tasks, 19 task families, three simulated robots, six configurations and one
repeat: 198 episodes. OmniLink's tested control integration completed 33/33.
Four other configurations also completed 33/33; the plain loop completed 32/33.
All configurations recorded zero unwanted-motion episodes under the sampled
test criteria. Model cost per success was approximately 45% lower for OmniLink
than the plain loop; frameworks with the same OmniLink command handling matched
completion and were slightly cheaper. This supports an efficiency benefit on
the measured suite, not exclusive capability or market-wide leadership.

The current experiment shares tools, controllers, state observations and
safeguards. It isolates integration and command-handling effects; it is not an
evaluation of all features of complete products. Keep every configuration and
earlier attempt visible. Keep original evidence immutable.

## Next question

Can the complete OmniLink control experience in OmniSim finish unfamiliar robotics
jobs more reliably, with fewer human interventions and lower cost, when actual
motion differs from commanded motion?

Prioritize detection and recovery from partial execution without duplicated
successful actions. Candidate disturbances include wheel slip, blocked motion,
changed obstacles, failed grasps, delayed or stale observations and interrupted
multi-step jobs. Separate safe stopping from successful task completion.

Secondary directions are adaptation to unfamiliar robot descriptions with
measured calibration, and a reusable skill library with explicit preconditions,
measured completion and recovery behavior. These are development proposals,
not claims that the complete behavior already exists.

## Before the next measurement

1. Independently review a harder task specification, scoring tolerances and
   realistic disturbance distribution. Hold back test instances and seeds
   from development; maintain separate development tasks.
2. Compare complete configured systems. Allow strong robotics integrations and
   document equal setup/tuning budgets, available observations, model budgets
   and hardware. Do not constrain competitors merely to produce a win.
3. Use component-removal and component-sharing controls to attribute gains.
   Publish ties and losses alongside primary comparisons.
4. Preregister task families, repetitions, stopping rules, failure handling,
   statistical analysis, spend cap and release/runtime hashes before requests.
5. Score completion, constraint violations, unwanted motion, interventions,
   recovery success, all-attempt model cost per success and wall time. Include
   failed attempts in cost and time. Distinguish setup effort from runtime cost.
6. Release runnable benchmark source and matching runtime/assets, raw traces,
   all attempts and offline verification. Have someone outside implementation
   reproduce the live run before claiming independent reproducibility.

## Claim boundaries

No promise that competitors can never match the system. No hardware-reliability
or learned sim-to-real claim without physical validation. Assisted grasp is
still assisted. Existing pose snapshots are not complete physics checkpoints;
candidate simulation rollouts would require additional validated state handling.
Do not advertise replay, run-diff or other unshipped facilities as available.

The success criterion is a reproducible advantage on a declared task domain,
supported by enough independent task families and repeated measurements. A
favorable single run or a test designed around known competitor failures is
insufficient for a broad capability claim.
