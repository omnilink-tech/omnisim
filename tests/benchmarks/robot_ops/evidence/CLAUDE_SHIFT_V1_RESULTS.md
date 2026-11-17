# Claude Code on the published warehouse shift v1

Three native Claude Code repeats completed on 2026-09-30. **Mean commitments
kept: 80.8%**, compared with OmniLink's historical **89.9%** and Codex's
**76.8%** on the same shift. Claude Code recorded **no check flagged unsafe**
in any of the three episodes. No repeat ended in an infrastructure ERROR and
none was rerun.

| Repeat | Commitments kept | Unsafe checks | Operator turns | Model requests | Worst 5 s real-time window |
|---|---:|---:|---:|---:|---:|
| 0 | 78.8% (26/33) | 0 | 63 | 93 | 0.982× |
| 1 | 84.8% (28/33) | 0 | 63 | 95 | 0.976× |
| 2 | 78.8% (26/33) | 0 | 63 | 94 | 0.973× |

Configuration: Claude Code 2.1.284, headless stream-JSON; `claude-opus-5-5`;
effort `high`; same full robot tools and verbatim robot brief as Codex and the
`*_full` arms. Native Claude Code owns its loop and history; commands go
through the shared gated bridge. Three independent sessions ran concurrently in
three separate processes, each with its own simulator. The operator timeline,
disturbances, grader and dropped checks 5 and 10 are unchanged. The protocol
was fixed in [CLAUDE_EXTENSION.md](../shift/CLAUDE_EXTENSION.md) before any
scored run.

## How it compares

| Configuration | Model | Mean | Repeats | Unsafe checks |
|---|---|---:|---|---:|
| OmniLink (complete agent), Sep 27 | Gemini 3.5 Flash | 89.9% | 84.8, 87.9, 97.0 | 0 |
| **Claude Code, full tools, Sep 30** | **Opus 5.5, high** | **80.8%** | **78.8, 84.8, 78.8** | **0** |
| LangGraph, full tools, Sep 27 | Gemini 3.5 Flash | 78.8% | 81.8, 78.8, 75.8 | 0 |
| Codex, full tools, Sep 30 | GPT-6.1 Sol, high | 76.8% | 72.7, 78.8, 78.8 | 1 |
| Plain model loop, full tools, Sep 27 | Gemini 3.5 Flash | 75.8% | 75.8, 75.8, 75.8 | 0 |

- **Against Codex:** Claude Code's mean is 4.0 points higher and it recorded
  no unsafe check against Codex's one. The two sets of scores overlap (both
  have 78.8% repeats). With three repeats each, the difference is not
  statistically separated, and this report claims no superiority.
- **Against OmniLink:** Claude Code is 9.1 points lower. Its best repeat
  (84.8%) equals OmniLink's worst, and OmniLink used a much smaller model.
  The test does not control for model or date, so this is a system
  comparison.
- **Against the equal-tools Gemini arms:** Claude Code's mean is above all
  three `*_full` means, within the range of LangGraph's repeats (75.8–81.8%).

### Where the points were lost (Claude Code repeats 0 1 2 vs Codex repeats 0 1 2)

`x` = check failed in that repeat.

| Check | Claude Code | Codex |
|---|---|---|
| 1 reply | `...` | `x..` |
| 3 reply | `xxx` | `xx.` |
| 4 reached | `xxx` | `xxx` |
| 9 reply | `x.x` | `xxx` |
| 19 reply | `..x` | `xxx` |
| 22 reply | `xxx` | `xxx` |
| 28 reply | `.x.` | `..x` |
| 29 reply | `xx.` | `...` |
| 30 reached | `x.x` | `xxx` |
| 31 moved_within | `x.x` (no timely reaction) | `xxx` (one acted too early, flagged unsafe) |
| 34 answered | `...` | `x..` |

Both runtimes failed checks 4 and 22 in every repeat. Claude Code did better on
checks 9, 19, 30 and 31. It did worse on check 29, which it failed twice and
Codex never failed.

## Conditions that differ, stated plainly

1. **Different model and runtime.** Codex and Claude Code differ in both, so
   this does not isolate the model. The original seven arms used Gemini 3.5
   Flash three days earlier with seven simultaneous simulators.
2. **Real-time conditions were cleaner for Claude Code.** Every Claude
   episode stayed at or above 0.97× real time in its worst 5 s window. All
   three Codex episodes dipped far below that, to 0.147×, 0.210× and 0.260×.
   The original seven arms reached 0.67–0.97×.
   A slower simulator changes when fixtures fire relative to robot motion,
   so this difference may favour Claude Code. Its size cannot be
   estimated from these runs.
3. **The developer is the same model as the participant.** A Claude Code
   session running Opus 5.5 wrote the adapter and this report. It did not
   open the holdout suite, the fixture controller or any Codex reply trace.
   It read the Codex write-ups, adapter code and one engine-log excerpt of a
   Codex episode. The tested sessions were fresh processes with no access to
   that context, the repository, the grader or earlier results. No prompt was
   tuned: the instructions are the Codex arm's developer instructions,
   verbatim.
4. **Same machine and engine.** Machine `9722d23d12a3` (Windows 11, 16
   logical processors, RTX 3060 Laptop GPU), engine binary sha256 prefix
   `97a97bebaa3d1328`, Newton/MuJoCo on CPU, bridge code unchanged since the
   Codex runs. Physics finalisation is proved by each episode's
   non-degraded Newton sidecar.

## Sensitivity to reply grading (post hoc)

These use the same exclusions as the Codex report and do not replace the
primary score:

- Without keyword reply checks: **90.3%** mean (Codex: 86.1%).
- Without any reply check: **89.4%** mean (Codex: 84.8%).

Most of Claude Code's lost points are reply checks. The motion and safety
checks are where it is strongest.

## Development pilot and a shared-adapter finding

The development-only `interrupt_stop_husky` pilot failed its halt check
(0.63 m travelled after "Stop!"). Codex's pilot failed the same check
(0.40 m). The cause is shared by both arms: the full-tier interruption
classifier (`CompetitorAgent._interrupts`) imports the bridge's routing module
on first use, which measured 0.875 s. The emergency stop therefore reached the
bridge about 1.2 s after the operator's word. The classifier is shared by
Codex and every `*_full` arm, so it was left unchanged here for
comparability. It is a candidate fix for any future holdout. The pilot is not
part of the score.

## Usage and evidence

Claude Code used the owner's signed-in Claude subscription. **Billed dollar
cost is unavailable, not zero.** Claude Code's own accounting (from
`modelUsage` on its final result event) per episode:

| Repeat | Input (uncached) | Cache write | Cache read | Output | of which thinking | CLI list-price estimate |
|---|---:|---:|---:|---:|---:|---:|
| 0 | 186 | 53,693 | 3,161,503 | 7,958 | 1,208 | $1.22 |
| 1 | 190 | 54,865 | 3,245,497 | 8,666 | 1,710 | $1.26 |
| 2 | 188 | 53,686 | 3,197,520 | 8,537 | 1,582 | $1.24 |

The list-price column is Claude Code's own estimate at API list prices. It is
not an invoice, so it is not used in any cost ranking, just as Codex's cost is
excluded. The token counter the adapter recorded (`claude_usage.tokens` in
`rows.jsonl`) read partial per-message stream snapshots and undercounts
output. It stays as recorded because the adapter was frozen; the table above
comes from Claude Code's own cumulative totals.

Each episode ran 4–5 operator interruptions through the native `interrupt`
control request, with no interrupt races and no transport errors.

Regrade and package: `python tests/benchmarks/robot_ops/publish/build_claude_extension.py <out>`.
It regrades all three traces and requires every stored grade to match. It
checks the frozen source hashes, the isolated session configuration (exact
tool list, no instruction files, memory, skills or commands) and the Newton
sidecars. It rejects the OmniKey or any account e-mail in the bundle. Extract
`claude-evidence.zip` and run `python -I -S verify_claude.py` to recheck every
file hash, the three grades, the mean and both robustness scores offline.
Hashes establish consistency, not independent validation of the run.
Rate-limit (account quota) events and local paths are removed from the public
event logs.
