# Codex on the published warehouse shift v1

Three native Codex repeats completed on 2026-09-30. **Mean commitments kept:
76.8%**, compared with OmniLink’s historical **89.9%**. Codex recorded
one check flagged unsafe in one of three episodes.

| Repeat | Commitments kept | Unsafe checks | Recorded model rounds |
|---|---:|---:|---:|
| 0 | 72.7% | 1 | 109 |
| 1 | 78.8% | 0 | 119 |
| 2 | 78.8% | 0 | 116 |

Configuration: Codex app-server 0.159.0; `gpt-6.1-sol`; high reasoning;
same full robot tools and verbatim robot brief. Native Codex owns its loop
and history; commands go through the shared gated bridge. Three independent
sessions ran in three separate processes and owned simulators. The operator
timeline, disturbances, grader and dropped checks 5/10 are unchanged.

This is a **retrospective system comparison**, not a new blind holdout or
a model-controlled test. The original seven configurations used Gemini 3.5
Flash on September 27 with seven simultaneous simulators. Codex ran later
with a different model, a later build, three simultaneous simulators, and
no six-round inner-loop cap. Both used machine `9722d23d12a3`, Windows 11,
16 logical processors, RTX 3060 Laptop GPU, Newton/MuJoCo on CPU. Physics
finalisation is proved by each episode’s non-degraded Newton sidecar.

All three runs dipped below 0.9× real time on the original worst-five-second
window metric (0.147×, 0.210×, 0.260×).
As in the original study, a real-time-clean sensitivity comparison cannot
be computed. The score retains these runs; timing conditions are a limitation.

The integration developer knows the published test. Each tested Codex
session started fresh, with no repository, filesystem, test script, grader,
prior results, external MCP, plugins or web access. This measures the
documented robot-tool configuration, not unrestricted Codex engineering.
No statistical superiority claim is added.

## Sensitivity to reply grading

The original score contains keyword-graded replies. The following use the
same exclusions as the original report and do not replace the primary score:

- without keyword reply checks: 86.1% mean (post hoc).
- without any reply check: 84.8% mean (post hoc).

## Usage and evidence

Codex used the owner’s signed-in account. Billed dollar cost is unavailable,
not zero. Token totals are cumulative per isolated thread, not summed again
per operator turn. Model rounds count observed token-usage notifications;
interrupted calls may lack final usage. Exact per-repeat counters, real-time
windows, source hashes, model/settings and scores are in
[CODEX_SHIFT_V1_RESULTS.json](CODEX_SHIFT_V1_RESULTS.json).

The initial wave had one FAIL and two infrastructure ERROR results caused by
overlapping native interruptions with stale turn IDs. A narrow transport
correction was documented before rerunning the entire three-repeat wave with
a new source freeze. No prompt, tools, timeline or grading change was made.
The primary score uses that corrected wave. All six attempts and both frozen
source versions are retained in the archive; the initial outcomes and errors
are listed in the results JSON. No further result-based rerun was allowed.
Development-only pilot evidence is not part of the shift score:
the first pilot could not create its temporary workspace; the second ran
and failed the halt criterion. No prompt tuning followed that result.

The adapter/source hashes were frozen before the scored runs per
[CODEX_EXTENSION.md](../shift/CODEX_EXTENSION.md). Offline regrading from all
three recorded traces and replies must match every stored grade before
publication. The downloadable archive contains the frozen sources, full
pose traces, replies, native tool events and standard-library-only verifier.
Account quota/credit notifications and the local Codex home path are omitted;
the explicitly supplied OmniKey is checked for accidental inclusion.

[Results and downloadable evidence](https://www.omnilink-agents.com/benchmarks/operations-shift/index.html#codex-extension).
Extract `codex-evidence.zip` and run `python -I -S verify_codex.py` to check
every file hash, all six attempt grades, the primary mean and robustness scores offline.
Hashes establish consistency, not independent validation of the run.

After evidence packaging, the shipped CLI corrected legacy Codex cost displays
to unavailable and rejected provider-rate assumptions for the signed-in arm.
The archive retains the actual frozen run sources; these reporting-only changes
and a license header do not alter the recorded scores or agent behaviour.
