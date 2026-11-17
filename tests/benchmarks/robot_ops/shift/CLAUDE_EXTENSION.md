# Claude Code extension of the published warehouse shift

Protocol fixed 2026-09-30 before any scored Claude shift run. It copies the
[Codex extension](CODEX_EXTENSION.md) clause for clause; where the two native
runtimes differ, the difference is named here.

This is a retrospective extension of shift v1, not a new blind holdout or
a model-controlled comparison. The published shift and results are known
to the adapter developer, a Claude Code session running the same model as
the participant. That session did not open the holdout suite, the fixture
controller or any Codex reply trace; it read the Codex write-ups, adapter
code and one engine-log excerpt of a Codex episode. Each tested Claude
session starts fresh, receives only live operator messages, the full bridge
tool schemas and the same robot brief; it cannot access the repository,
fixture controller, suite, grader, earlier traces or earlier scores, and it
shares no context with the developer session.

- Task: unchanged `suites/shift_holdout_v1.json`, 30-minute real-time shift.
- Score: the existing grader and analysis; exclude checks 5 and 10 as in v1.
- Participant: native Claude Code 2.1.284 (headless, stream-JSON), model
  `claude-opus-5-5`, effort `high` (Codex: `high` reasoning).
- Three independent repeats, numbered 0–2, in three separate runner processes.
- All three repeats run concurrently on machine `9722d23d12a3`, the same
  machine and the same engine binary (sha256 prefix `97a97bebaa3d1328`) as
  the Codex runs. The bridge and controller code is unchanged since then.
- Same full tool surface and robot brief fetched from `/tools` and `/main_task`.
  The tools reach Claude through a standard-library stdio MCP server
  (`claude_mcp_stdio.py`) that forwards every call to the adapter; the tool
  names carry Claude Code's `mcp__robot__` prefix, schemas are unchanged.
- The robot brief and the same fixed instructions the Codex arm received as
  developer instructions are appended to Claude Code's default system prompt
  (Codex: appended to its default base instructions).
- Robot commands use gated `/tool` with the original operator words. Stop
  uses the same emergency escape hatch as the existing full-tier adapters.
- Incoming orders use the existing full-tier interruption classifier and
  interrupt the native Claude turn (stream-JSON `interrupt` control request);
  questions and remarks queue normally.
- Claude Code owns the model/tool loop and history. Every built-in tool is
  disabled (`--tools ""`), as are setting sources, skills, slash commands,
  session persistence and every MCP server other than the robot. The session
  must report exactly the bridge tools, a connected robot server, no skills,
  no commands, no memory files and no discoverable CLAUDE.md/AGENTS.md, or
  the episode is an infrastructure ERROR.
- One native turn per operator message, at most 180 seconds per turn and
  1,000 operator turns per episode. Native internal model rounds are recorded;
  no six-round inner-loop limit is imposed.
- Claude Code uses the owner's signed-in Claude account (a subscription).
  Token usage is recorded. Billed dollar cost is unavailable and must not be
  presented as zero; Claude Code's own list-price estimate is recorded as
  `cli_reported_cost_usd` and is not a billed cost.
- Historical rows use Gemini 3.5 Flash and earlier code/builds and had seven
  simultaneous simulators. Claude uses a different model and runs later with
  three simulators, like Codex. These differences accompany any displayed score.
- No statistical superiority claim will be added from this extension.
- A completed low score or unsafe result remains in the report. Only
  infrastructure ERROR episodes may be rerun once; all attempts are retained.

Interrupt races: the Codex adapter's transport amendment tolerates a native
rejection of a turn that already ended. The Claude adapter is written with the
equivalent rule from the start: a rejected interrupt is recorded as
`interrupt_race` and cancellation still blocks bridge calls; a dead Claude
process is still fatal.

Development-only checks: a fake-bridge smoke test (read state, then an
operator stop interrupting a blocking drive); eight adapter contract tests
mirroring the Codex ones; one `interrupt_stop_husky` pilot on the development
suite, as Codex had. The pilot does not contribute to the scored shift and no
prompt tuning follows its result.

The machine fingerprint, frozen source hashes, suite hash, raw Claude events,
bridge results, pose traces, manifest and per-check grades are retained in
the extension evidence. Regrade offline before publishing any number.
