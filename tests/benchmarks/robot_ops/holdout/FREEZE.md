# FREEZE — ops-bench holdout v1

Frozen 2026-09-25 before the first scored episode, under
[PREREGISTRATION.md](PREREGISTRATION.md). After this file is committed, nothing
changes until the campaign ends.

## What is frozen

| Item | Value |
|---|---|
| Code commit (product, bridges, runner, adapters, grader) | `4707517fb` |
| Holdout | `suites/holdout_v1.json`, SHA-256 `1e42469731544a4de434bf124a9d0a4397bfb6498941d3aef146e3c4e6c4fff2` |
| Tasks | 24 (4 each in F1, F2, F3, F4, F6, F9). None dropped: all 24 calibrate (`oracle` PASS, `oracle_bad` FAIL), across `holdout-calibration-01` to `-04` |
| Arms | `omnilink`, `plain_basic`, `langgraph_basic`, `lobster_basic`, `plain_full`, `langgraph_full`, `lobster_full` |
| Repeats | 2 per task per arm: 336 episodes |
| Engine / model | OmniLink `g1-engine`, `gemini-3.8-flash`, for every arm. Google credential: the Vertex AI service account in project `omnilink-509715`. The OmniLink platform revision is `omnilink-00237-4wl` (Cloud Run, europe-west1) |
| Rates (USD per million tokens) | input 0.75, cached 0.075, output including thinking 3.75 |
| Spend cap | $40 for the whole campaign, including the ERROR re-runs |
| Per-episode competitor request cap | 60 (the runner default) |
| Machine | `9722d23d12a3`, AMD64 Family 25 Model 80, 16 cores, RTX 3060 Laptop, Windows 11, Python 3.12.9, newton 1.5.0 / mujoco 3.11.0, engine binary sha256 `d8473e670ff0dedd`, Controller.dll `0bd8db2b5018d7fe` |

## The command

```bash
tests/benchmarks/harness_comparison/.venv/Scripts/python.exe -m omnisim ops-bench run \
  --suite tests/benchmarks/robot_ops/suites/holdout_v1.json \
  --arms omnilink plain_basic langgraph_basic lobster_basic plain_full langgraph_full lobster_full \
  --repeats 2 --engine g1-engine --model gemini-3.8-flash \
  --cap-usd 40 --rates 0.75,0.075,3.75 \
  --out tests/benchmarks/robot_ops/evidence/holdout-v1
```

The interpreter is the harness-comparison venv because it has LangGraph. The
OmniKey comes from `OMNI_KEY`/the default key file, and is never recorded.

## Procedures fixed in advance

- **ERROR re-runs.** After the main run, each ERROR episode (task, arm,
  repeat) is re-run once, as a separate invocation of the same command with
  `--only <task> --arms <arm> --repeats 1` into
  `evidence/holdout-v1-reruns/`. Its `--cap-usd` is $40 minus the main run's
  spend. An episode that errors again is excluded from both sides of every
  pair it belongs to, and is reported.
- **Interruption.** If the run process dies (the machine sleeps, the
  process crashes), only complete task blocks are kept. The remaining tasks
  are run with the same command and `--only <remaining task ids>` into
  `evidence/holdout-v1-cont-<n>/`, with the cap reduced by the spend so far.
  The interruption is reported.
- **Analysis.** As stated in the preregistration: task-level bootstrap (2,000
  draws, seed 250925), Claim A against `*_basic` and Claim B against `*_full`.

## Bugs found by the pilot and fixed before the freeze

The pilot used the development suite only (`evidence/pilot-01` to `-03`,
$0.58 of the $3 cap in pilot-03).

1. **The model was unavailable.** `gemini-3.5-flash` returned 503
   overloaded, and `gemini-3.8-flash` was not on the platform's allowlist.
   It was added to the OmniLink platform (`api/_gemini-models.ts`, OmniLink commit
   `46228cc6`) and deployed.
2. **Rate limiting locked out the credential.** The platform cooled its
   only Google credential for 60 s after a 429 that carried no retry hint.
   Vertex returns exactly that kind of 429 from its shared quota. So about 10
   good requests were followed by a minute of local 429s, for every arm. The
   fix, in OmniLink commit `2c6dc956`: back off 1, 2 and 4 s
   inside the call, then cool the credential for 10 s. A burst test passed
   20/20. This affects every arm's transport equally.
3. **Provider failures were graded FAIL.** They are now graded ERROR (as
   infrastructure) inside `grade()`, so the offline regrade agrees with the
   live run. This applies to all arms.
4. **A halt note hid a provider error.** OmniLink's "I stopped what I was
   doing first." prefix made an error reply look like an answer. The runner
   now strips that prefix before deciding whether the reply answered.
5. **The full tier had no tools.** The mobile bridge's `GET /tools` called
   `relay.tool_defs()`, but `tool_defs` is a property, so the request failed
   with `'list' object is not callable`. The competitor full tier depends on
   this route. Fixed in `3f12732ed`.
6. **Competitor model timeout.** Competitor calls gave up after 90 s. The
   relay's own timeout is 120 s (`OMNILINK_TIMEOUT`). One pilot-03
   `langgraph_full` reply was lost to a ReadTimeout that the relay would have
   waited out. Competitor calls now wait 120 s (`3f12732ed`).

## Disclosures

- **What the developer saw of the holdout.** Before the freeze, the
  developer saw the task ids and failing check names from calibration, as the
  preregistration allows. For the F9 shift task
  `f9_shift_x_min_block_husky`, the developer also saw some of the scripted
  oracle reply text while debugging the oracle's cancel handling. The
  developer never saw the operator prompts.
- **Engine fallback.** The OmniLink platform tries a fallback engine when
  the primary fails. That is impossible here, because only Google is
  configured. Every request's returned model id is recorded in
  `competitor_requests[].model_returned` and in `relay_usage`.
- **Latency differs by design.** Every arm shares one transport. OmniLink's
  bridge answers most operator messages with its deterministic parser and
  never calls the model for them. That is the product under test, not a
  transport difference. In pilot-03, single model calls reached 33 s, and
  arms that make several calls in a row feel that more.
- **Shared machine.** Other work sessions on this machine run short
  fast-mode engines, not ops-bench campaigns, during the run. Episodes are
  REALTIME, and the arm order rotates per task, so load affects all arms
  alike. It is still disclosed.
- **Not part of the frozen tree.**
  `projects/samples/demos/controllers/omnilink_mobile_bridge/tests/` is
  untracked work from another session. The bridge does not import it.
