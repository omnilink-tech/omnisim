# Publication campaign 02 — service recovery replacement

Frozen before any scored request in this campaign. Campaign 01 stopped at 16 of
594 episodes after a shared-service HTTP 503 without usage. Its records and
unknown cost are retained, not overwritten, pooled, or relabelled as successful.
The approved, unscored diagnostic resend is recorded in
`transport-diagnosis-01.json`: HTTP 200, the required model identity, complete
usage, estimated model cost $0.00050950. This establishes recovery, not the
unobserved root cause of the original 503.

This is one replacement campaign, not a search for favorable outcomes. Keep the
same 33 development tasks, six implementations, three repeats (594 episodes),
random seed/order, model/profile, price rates, robot fixtures, product code,
model prompt, planning limits and grading oracle from campaign 01. The only
runner change adds response digests and structured service-error metadata.

Use `locks/publication-02.json` and a new `evidence/publication-02` directory.
The soft stop-new-request budget is $4.20. Prior known development costs remain
disclosed; campaign 01's failed request has unknown cost and is never called
free. This soft limit is not an exact provider billing ceiling.

All 594 attempts are scheduled. Every PASS, FAIL and ERROR stays in the attempt
denominator, and all known failed-attempt costs/times are counted. Stop after
completion or the existing budget/unknown-usage guard; no automatic transport
retry, no switching model, no replacing individual results. If another transport
failure interrupts the campaign, retain it and diagnose it before deciding on
any explicitly documented amendment. A poor comparative result is not a reason
to repeat this campaign.

The scope, analysis and publication rules in CAMPAIGN_01.md remain in force.
The suite is developer-authored and used during development; a complete run is
a descriptive measurement, not an independent holdout or market-wide ranking.
The additional publication-error records cannot convert unknown usage to zero.
