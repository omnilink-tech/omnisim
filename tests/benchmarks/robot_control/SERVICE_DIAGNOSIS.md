# Service interruption and accounting history

2026-09-24. These are transport observations, not robotics scores.

Three separately frozen scored campaigns stopped on HTTP 503 responses without
usage, as their then-current runner required. They remain incomplete: campaign
01 stopped at 16/594, campaign 02 at 2/594, and campaign 03 at 16/198. The first
two used gemini-3.1-flash-lite; the third used gemini-3.5-flash. Their failed
request bodies were retained, but the provider error text was not, so their
exact individual upstream causes and costs cannot be recovered from those rows.

The authorized unscored diagnostics are retained in transport-diagnosis-01.json
through transport-diagnosis-05.json. Replays sometimes succeeded. Diagnosis 03
captured Google's structured high-demand 503 UNAVAILABLE response. Diagnosis 04
received a 404 stating gemini-2.5-flash-lite was unavailable to the account.
Diagnosis 05 was five consecutive successful PONG responses on gemini-3.5-flash;
this short availability check did not guarantee campaign-wide availability.
No diagnostic executed a robot action or contributed to a score.

Campaign 04 keeps the last model and task design and introduces one bounded
retry for the exact recognized overload response. The original failed request,
null usage, provider error, response digest and retry waiting time remain in
evidence. Unrecognized missing usage still stops model spending. Repeated model
rejection ends an episode as ERROR, which stays in the denominator.

The recognized error's zero token-charge estimate is explicitly conditional on
applying Google's published failed-request billing policy to the 503 error class.
It is not a verified invoice or measured zero tokens. The public table discloses
rejection counts and the effect of adding a hypothetical $0.01 per rejection.
No earlier missing error body is reconstructed, and no old unknown cost becomes
zero. Campaigns use their own frozen rates and are not pooled across versions.

Reference: [Google's failed-request billing FAQ](https://ai.google.dev/gemini-api/docs/billing#am-i-charged-for-failed-requests),
checked 2026-09-24. The narrow classification and stopping rules are defined in
[CAMPAIGN_04.md](CAMPAIGN_04.md), not inferred after looking at comparative scores.
