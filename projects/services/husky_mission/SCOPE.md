# Scope and acceptance · 2026-10-30

Reference Husky model, no payload, flat empty 10 × 10 m arena, imposed contact
friction 1.0. Known stations: base (0,0), A (1.5,0), B (1.5,1.5), metres. Position
comes from the simulator. Wheel commands drive the physical simulation; the
mission controller does not teleport the robot.

- Arrival: measured distance at most 0.15 m and speed at most 0.025 m/s.
- Mission: ordered evidence for A, B and base; stopped for the final second.
- Cancel / Stop: settle below 0.025 m/s within 2 simulation seconds, coast at most
  0.20 m, observe at least 5 seconds, no later station receipt or resumed route.
- Rejected proposals: displacement at most 0.01 m. Tests include informational,
  prohibited, quoted, unknown-station, reordered and unregistered proposals.
- Busy and duplicate commands must not start another route.
- Missing OmniKey: AI request fails explicitly; independent typed Stop remains.
- Live AI cases require observed model usage and zero parser short circuits.

26 frozen qualification cases: 6 live AI episodes, 3 typed route repeats,
6 typed interruptions, 6 deliberately injected invalid tool proposals,
one no-key episode, busy and duplicate episodes, and 2 controlled reference runs.
The reference runs send the same 0.5 m proposal directly and through the command
check. This is a checks-bypassed comparison, not a competitor benchmark or a
measurement of how often a model invents a command.

Nominal linear target is 0.25 m/s. Actual physical speed is measured separately
and can exceed the target. The target is not a certified speed limit.

Public mission wording comes from verified runtime state; raw model wording is
retained in the evidence. A success-sounding model sentence cannot create a
station receipt. Unit tests also inject a forged arrival result and invented
completion wording. These checks do not establish general hallucination immunity.
The qualified operator interface is typed/text OmniLink requests and typed Stop.
Voice output is disabled for this pilot; voice interaction is not qualified.

Newton 1.5 / MuJoCo CPU, 16 ms basic step, 8 configured substeps. Each episode
retains machine fingerprint, runtime and source hashes, pose trace, requests,
outcomes, and cleanup status. Verify the delivery inventory and independently
recompute verdicts from poses before trusting a report.

Not included: unknown environments, obstacle avoidance or perception, arbitrary
coordinates, payload or terrain qualification, manipulation, hardware operation,
sim-to-real validation or safety certification. General simulator control routes
remain development interfaces outside the agent contract. Agree those extensions
and hardware-specific safeguards before an integration. Agent build pricing is
quoted per robot and workflow, separate from $100 assessment modules and AI usage.
