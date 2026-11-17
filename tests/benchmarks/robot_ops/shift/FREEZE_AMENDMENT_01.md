# FREEZE AMENDMENT 01 — warehouse shift v1: ERROR re-runs run together

Recorded 2026-09-26, about 23:20 local, while wave 1 was running and before
any re-run had started. It changes one procedure in [FREEZE.md](FREEZE.md).

## What happened

In wave 0 (`evidence/shift-holdout-v1-wave0`), Google rate-limited the
project from minute 19 to minute 29 of the shift. For ten minutes every
competitor request was answered 429 ("g1-engine is rate-limited upstream by
Google"). The prompt-token volume at minute 19 was below the earlier peaks,
so this was not our load growing.

Every competitor arm lost 12–18 replies and is ERROR, as preregistered.
OmniLink made no model call in that window (its parser handled the
deliveries), lost nothing, and is scored. Wave 1 ran almost clean through the
same minutes (three 429s in 22 minutes).

## The change

FREEZE.md says an ERROR episode is re-run "as its own wave of that single
arm", which would mean six single-arm waves one after another. Instead,
**every ERROR episode of the campaign is re-run once, all together, as one
wave** (`--parallel` equal to the number of episodes) into
`evidence/shift-holdout-v1-rerun`, after wave 2 ends and after the health
gate passes.

- **Why.** Every scored episode ran with other arms beside it. A re-run
  alone would face a quieter machine and provider than the episodes it is
  compared with. It also takes about 35 minutes instead of about 3.3 hours.
- **Who decided, and on what.** The owner, from the error evidence above,
  before any re-run result existed. No scored outcome informed it.
- **Unchanged.** An episode that is still ERROR after its re-run is excluded
  from both sides of every comparison it belongs to, and reported. Nothing
  else in FREEZE.md changes.
