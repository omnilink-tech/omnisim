# ops-bench pilot (before the freeze)

Authorised by the owner on 2026-09-25, under PREREGISTRATION.md "a capped
pilot, at most $3, on the DEVELOPMENT suite only".

**Purpose: find integration bugs in each arm's MODEL path**, which has never
run: OmniLink's relay tools (`set_boundary`, `schedule_action`, ...) with a
real model, and the six competitor arms. It is **not** evidence for any
claim. It uses development tasks the developer wrote and fixed OmniLink
against, and it runs once. Its numbers are never pooled with the holdout
campaign.

- **Model:** `g1-engine` / `gemini-3.8-flash`, the same for every arm. The
  owner chose it on 2026-09-25 over the relay's older default,
  `gemini-3.5-flash`, which was also answering 503 "overloaded" that day.
  3.8 needed adding to OmniLink's model allowlist (OmniLink `46228cc6`) and a
  platform deploy.
- **Rates:** $0.75 / $0.075 / $3.75 per million tokens (input / cached /
  output including thinking), Google's published paid-tier price through
  2026-12-31 (ai.google.dev/gemini-api/docs/pricing, read 2026-09-25). These
  are estimates, not invoices.
- **Cap:** $3.00, enforced before each episode (`--cap-usd 3`).
- **Arms:** all 7.
- **Tasks (development suite), chosen before running** to cover each
  model-dependent family:
  - `keepout_after_chatter_husky`: a rule, chatter, and a question;
  - `interrupt_goback_husky`: a correction needing the model;
  - `report_stopped_tb3`: a grounded report;
  - `shift_tb3_dev`: a long shift.
- **Order:** task-major, arm order rotated. If the cap stops the run,
  earlier tasks have every arm.

**What counts as an integration bug** (fixed in every affected arm before the
freeze, and listed in FREEZE.md):

- a transport or parsing failure that is the harness's fault;
- a tool the model cannot call because of how it is offered;
- an accounting gap;
- a crash.

A model simply doing the task badly is **not** a bug and is not "fixed".
