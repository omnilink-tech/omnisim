# Robotics control benchmark

A reproducible, simulation-based comparison of **specific robotic-control implementations**. The first suite covers Husky, TurtleBot3 Burger and OmniArm 6. It measures task completion, instruction compliance, state grounding, multi-turn execution, recovery and cost per completed job. It is not a full hosted-product ranking or a hardware validation.

The v4 harness comparison remains unchanged in `../harness_comparison/`. This suite deliberately includes its known failure classes as development regressions. No bundled task set is an untouched holdout.

See [REPORT.md](REPORT.md) for the development runs, observed product failures and publication requirements.
The product fixes and their follow-up validation are in [FIX_REPORT.md](FIX_REPORT.md).
The publication campaign history is in `CAMPAIGN_01.md` through
`CAMPAIGN_04.md`. Campaigns 01–03 stopped on shared-service HTTP 503 responses;
their incomplete records remain available. Campaign 04 specifies one bounded
retry, conditional error-cost accounting, Gemini 3.5 Flash and 198 trials.
The publication builder generates the page and reproduction commands from a
complete, verified campaign; it refuses partial coverage.

## Reproduce

For the public source distribution, first download the benchmark's evidence
archive and follow its `REPRODUCE.md`. Raw private working evidence is excluded
from source snapshots. The archive supplies the dependency lock and compiled
Lobster SDK that live under the private harness-comparison path below, together
with the exact measured source overlay. Historical local evidence links in the
development reports refer to the separately published history archive.

Requirements: a built OmniSim checkout passing `python -m omnisim doctor`, Python 3.12, Node 22 or newer, and an existing valid OmniKey. Windows has the prebuilt simulator package; other platforms require a build. Simulator asset files and the benchmark sources must come from the frozen revision/source snapshot. The first physical validation is on Windows; other platforms need their own calibration.

Use a dedicated Python environment and the exact dependency lock:

```powershell
python -m venv .venv-control-bench
.venv-control-bench/Scripts/python -m pip install -r tests/benchmarks/harness_comparison/requirements.lock.txt
```

Run the commands below with that environment's interpreter (shown as `python`). Prepare the pinned Lobster SDK at `tests/benchmarks/harness_comparison/vendor/lobster`, commit `71bd8145055498116de35623dbea0d7dda9b4cd5`; install its locked dependencies and compile it using the instructions in its README. The benchmark runs its actual SDK, and LangGraph's actual graph runtime. Missing dependencies are not replaced with mocks.

```powershell
python -m omnisim control-bench validate
python -m omnisim control-bench freeze --suite tests/benchmarks/robot_control/suites/calibration.json --lock calibration-lock.json --arms oracle --repeats 1
python -m omnisim control-bench run --lock calibration-lock.json --out calibration-results --calibrate --key-file <existing-key-file>
python -m omnisim control-bench verify calibration-results
```

Calibration reads expected actions and checks the real fixtures/graders. It makes no model requests and **cannot** establish an agent score. Every live bridge still requires an OmniKey. Calibration failures must be diagnosed before interpreting model runs.

Then run the small development pilot (six tasks, six implementations, one repeat):

```powershell
python -m omnisim control-bench freeze --suite tests/benchmarks/robot_control/suites/pilot.json --lock pilot-lock.json --repeats 1 --cap-usd 5
python -m omnisim control-bench run --lock pilot-lock.json --out pilot-results --key-file <existing-key-file>
python -m omnisim control-bench verify pilot-results
python -m omnisim control-bench report pilot-results
```

The full development pack has 33 episodes per implementation/repeat. Omit `--suite` to select it. Default repeats: three. Default arms: OmniLink parser integration, plain model-loop executor, LangGraph, Lobster SDK, LangGraph plus the same parser, and Lobster plus the same parser. The latter two are attribution controls: any equal performance must remain visible. No adapter receives expected paths, fault schedules or grading thresholds as agent input.

Model calls use the same existing entitled OmniLink profile, API transport and pinned model for every implementation. `--profile` and `--model` are selected **before freezing**. The default HuskySwarm profile is the earlier benchmark's profile, not a portable entitlement: reproducing accounts must select their own entitled profile and disclose that configuration. Missing access stops the run; it never silently chooses another model/profile.

The default rates are the earlier campaign's Gemini 3.1 Flash-Lite list-rate estimates, not invoices. Pass current verified per-million-token rates at freeze with `--input-rate`, `--cached-rate`, and `--output-rate`. Provider-reported tokens include cache and thoughts. Unknown usage stops further spending and blocks claims, except for the exact provider rejection and explicitly conditional cost estimate defined in CAMPAIGN_04.md. That exception retains null usage and must not be called measured zero tokens. The cap stops new requests with a conservative $0.10 reserve; because the existing transport does not certify requested provider output limits, it is not an exact billing ceiling and one in-flight response can exceed it.

## Outputs

- `lock.json`: suite, task provenance, adapters, repeats, random seed, model/profile, price basis, source/asset/runtime hashes, dependency versions and machine data.
- `rows.jsonl`: append-only episode evidence, measured samples, tool traces, model request metadata and all failures.
- `engines/`: each episode's owned simulator logs and Newton physics sidecar.
- `source/`: reviewable source snapshots. Heavy runtime/assets are hash-pinned, not bundled.
- `completion.json`: final coverage, source/dependency integrity and evidence digests.
- `scorecard.json` / `scorecard.md`: results, robot breakdowns, intervals and reasons a leadership claim is withheld. JSON is the input for a future website component; it never invents a winner.

`verify` checks coverage, digests and source snapshots, then independently regrades saved observations using the matching frozen oracle. It needs no model calls or running simulator. If your installed oracle differs, use the matching checkout; verification never executes arbitrary archived Python. These checks establish internal consistency, not independent attestation of who collected the data.

A fresh model run need not reproduce identical answers or timing. Preserve the suite, implementation configuration and decision rule, record your own machine, and publish the new freeze and all trials. If a different platform/build changes binary hashes, create a new lock and disclose that difference. Compare implementations within that run; do not silently compare wall times from different machines.

To check the native mobile `/prompt` clarification path and a guessed `/tool`
motion refusal in an owned engine, run this separately after timing comparisons:

```powershell
python tests/benchmarks/robot_control/native_gate_smoke.py --key-file <existing-key-file> --out native-gate-results
```

It saves the replies, measured observations, product source hashes and physics
proof. It requires the native prompt response to come from the parser and the
robot to remain still in both checks.

Each episode starts a fresh private simulator process. Startup/setup costs are separately recorded; task latency includes planner/runtime construction, observation, planning, execution and recovery. Only the process tree launched by this runner is cleaned up. Output directories and freezes are never overwritten. Fixes require a new lock and output directory.

Warp's kernel cache lives in `.tmp/control-bench-warp-cache`; the first calibration can take longer while kernels compile. Restricted process sandboxes can prevent engine/controller cleanup. Run on a host where the runner can manage its own child processes; cleanup failures are recorded as infrastructure errors.

## Before a headline campaign

1. Finish physical calibration on each robot and resolve fixture/grader defects. Product failures remain failures; do not tune the oracle to the product.
2. Use this pack for development and targeted regressions. Repair unwanted actuation and duplicate retries before release.
3. Obtain a separately authored/reviewed task set including paraphrases, both condition branches and varied initial states. Record its provenance and `split: holdout`. That label is a declaration to audit, not proof of independence.
4. Freeze product code, competitor adapters, tasks, budgets and decision rule before model calls. Review competitor adapters and allow equivalent optimization opportunities.
5. Run the complete campaign, publish all trials including stopped attempts, and retain the earlier versions. Never select a favorable run after inspecting results.
6. Add independent product-door, navigation/perception and fleet lanes before expanding claims to those capabilities. This first version does not test them.

See [SPEC.md](SPEC.md) for the scoring contract. References for implemented runtime patterns: [LangGraph workflows and agents](https://docs.langchain.com/oss/python/langgraph/workflows-agents) and [Lobster SDK](https://github.com/openclaw/lobster).
