# Publication campaign 04 — bounded provider-overload recovery

Declared before scored requests. Campaigns 01–03 remain incomplete and intact.
Gemini 3.5 Flash also returned HTTP 503 after successful availability probes.
Keep that model and the 198-episode design from campaign 03: all 33 development
tasks, all six integrations, one repeat, same seed/order, product, grader, tool
limits, models/rates and robot fixtures. No model shopping or selective episodes.

The only runtime change handles the precisely recognized Google provider error:
HTTP 503; no output, usage or model identity; inner error code 503, status
UNAVAILABLE, and the exact high-demand message captured by diagnosis 03. Keep
each original request and response digest. For this error only, wait at least
60 seconds (honor a numeric Retry-After up to 120), then retry once with the same
request body. A second rejection ends the episode as ERROR. All requests stay
in the request count, and the episode wall time includes the rejection and wait.
The 90-second action-dispatch deadline remains active. Do not retry unknown
errors, missing successful usage, or a changed returned model.

## Explicit conditional cost estimate

Google's billing FAQ states that failed 400/500 requests do not incur token
charges. Applying that error-class policy to the recognized 503 rejection is an
**accounting assumption**, not an invoice check. Store usage and model as null;
never fabricate zero tokens or a returned model. Estimate $0 token charge only
for the exact recognized rejection, labelled
`google_5xx_no_token_charge_assumption_v1`, with its policy link. All other
unknown usage still stops spending. Successful requests use recorded token
usage, including thinking, at $1.50/$0.15/$9.00 per million input/cached/output.

Sources checked 2026-09-24:
- https://ai.google.dev/gemini-api/docs/billing#am-i-charged-for-failed-requests
- https://ai.google.dev/gemini-api/docs/pricing
- https://ai.google.dev/gemini-api/docs/troubleshooting

Publish rejection counts by implementation beside the estimate, the assumption,
and sensitivity estimates adding $0.01 per rejected request. Do not describe
those rejected requests as having measured zero tokens. Do not retroactively
apply this classification to prior runs that lack the retained error body.
Latency includes service congestion and is not a framework-only speed measure.

## Freeze, stopping and claims

Use `locks/publication-04.json` and `evidence/publication-04`. Soft model cap:
$4.00, with the existing $0.10 reserve before starting a request. This is a
stop-new-requests estimate, not a guaranteed invoice ceiling. Run sequentially.
Preserve every PASS, FAIL, ERROR and retry. Stop only at coverage completion or
the frozen budget/unknown-usage/infrastructure rules; no score-driven changes.

The new accounting and bounded retry have 40 passing contract tests, including
unknown-error rejection, retained null usage, full cost and retry bounds. Robot
behavior and calibration remain the previously verified product version.

This remains a developer-authored development-suite measurement. The same
restrictions and family-bootstrap analysis as campaign 01 apply. A completed
run supports scoped descriptive numbers, not an independent holdout, whole-
platform total-cost claim or market leadership. Publish ties and losses too.
