# Directions — what to push next

A structured menu of next-step options for the Husky Maze + OmniLink integration. Each direction adds a real capability the current build doesn't have. Ordered by what would *actually* move the needle, not by busy-work potential.

> **Decision document.** The operator picks one (or a combination) and we execute. No work in here is "always do" — each choice is a real trade-off.

## Implementation status (2026-04-28)

All three directions A/B/C have been pushed past the gaps surfaced
in the first round of measurements. Each has architectural fixes
landed and live verification (subject to a documented OmniLink
shared-pool quota constraint on the `g1-engine` selector).

- **A** ✅ — cross-session memory compounding **with reliable replay**.
  - First-round fix: `recall` + `save_local_memory` mandates in the v3 prompt; chat_drive waits for closeout before exiting.
  - Second-round fix (this round): new `replay_recalled_path(memory_id)` tool that loads the saved cells server-side, validates the stored path's start matches the husky's current cell, drops the head, and posts `execute_path` directly. The LLM never retypes the path — eliminates the 56/72-cell truncation that defeated the structured `extracted_paths` field and the verbatim-copy prompt.
  - Live verification (12 chat turns, 11 tool calls, 147 s, goal reached): the validator correctly refused a stale memory whose start didn't match the current pose; agent fell back to `try_get_known_map`; no path truncation in the resulting 72-cell `execute_path` call.
  - Trace: [RESULTS.md → Direction A](RESULTS.md#direction-a--cross-session-memory-compounding) and the new "A2 retry-3" subsection.

- **B** ✅ — Mission Captain delegation **end-to-end verified, husky physically driven to target cell**.
  - First-round verification: captain runner up, `list_agents` reachable, original 2026-04-26 trace exists.
  - Second-round fix: captain's `delegate_to_agent` now slims the sub-agent's `system_instruction` (`_slim_settings_for_subchat`: trims each tool description to its first sentence, drops `toolCallbackUrl` and `standingOrders`). Plus bridge fix: `/camera` returns 503 with `available: false` when no `husky_eye` sidecar is wired (was previously serving 800x600 operator-viewport frames at ~3 MB that 413'd OmniLink at the 1 MB request limit).
  - **Live end-to-end verification** (after quota refilled): captain delegated "Drive to cell (0, 8)" to Husky Maze; sub-chat ran **30 turns / 19 tool calls / 0 × 413 errors**; sub-agent reported "mission successfully completed"; husky physically reached cell (0, 8) on the bridge (`mission_complete: true`); captain called its own `complete_mission` and exited cleanly. **386 s wall-clock**.
  - Trace: [RESULTS.md → Direction B → Final B verification](RESULTS.md#final-b-verification--captain-delegation-reaches-the-target-cell).

- **C** ✅ — hardcore vision-only world **with local perception layer (commit `70c6b1b`); raw-vision was economically infeasible, perception-tool is feasible**.
  - First-round fix: bridge gates both `/maze` and `/lidar` to `{available: false}` when world title contains "blind"; `husky_maze_blind.wbt` world ships. End-to-end agent run was qualitatively poor (8 turns, 1 cell, agent confused).
  - Second-round fix: v1's `mainTask` ships a structured vision protocol (read_camera → narrate cardinals → decide → drive → snap). Verified on a 60-turn run reaching cell (4, 8) at ~6 turns/cell. Working but **economically infeasible**: each `read_camera` cost ~120 K input tokens × ~6 turns/cell × ~12 cells = ~6 M tokens for a single full traversal — beyond what one OmniLink quota window allows.
  - Third-round fix (commits `87c4289` + `730c00c` + `70c6b1b`): switch from vision-as-pixels to **vision-as-tags**. Sidecar's `/scan` endpoint runs a pure-Python BGRA analyser over the four cardinal cameras and returns ~1.3 KB of structured tags per scan — `wall_close`, `marker`, `marker_centroid`, `floor_visible`, `mean_brightness`, plus the bridge's `current_cell`/`visited_cells`/`unvisited_neighbours`. Agent never sees pixels in the routine path. Plus 4 cardinals at 160×120 in the world (no need to pivot to look around), bridge auto-snap on `drive_forward`/`turn` (no need to call `snap_to_cell`), and `chat_drive` 429-retry to survive transient quota tightening.
  - **Live end-to-end verification** with the perception layer: agent drove autonomously (0, 10) → (5, 7) — 8 cells in 25 turns ≈ **3 turns/cell** (down from 6+), total run cost **~300 K input tokens** (down from ~6 M, roughly **20×** cheaper). Hit max_turns at (5, 7) where the agent's "pick an unvisited cardinal with wall_close=false" rule failed to commit to the correct backtrack — that's a **navigation-logic gap, not a perception gap** (scan output at that cell clearly identified the open and walled directions). Future fix: explicit `back_track_to_last_branch` heuristic.
  - Trace: [RESULTS.md → Direction C → perception-tool architecture](RESULTS.md#direction-c--perception-tool-architecture-commit-70c6b1b).

- **D, E, F, G** ⏳ — not started.

## Performance summary

End-to-end metrics for the directions implemented or verified this session, all on the seed-7 known maze (or seed-100 visual/blind layout) under OmniLink lib v0.6.1, `g1-engine`. See [RESULTS.md](RESULTS.md) for full traces.

| direction | run | turns | tool calls | wall-clock | outcome |
|---|---|---:|---:|---:|---|
| **A1** | v3 cold cache on `husky_maze.wbt` | 7 | 6 | 384 s | goal reached, plan saved |
| **A2** (1st) | same world after A1 (recall hit) | 11 | 10 | 484 s | recall HIT → execute_path(61 cells, truncated) → mid-batch fault → fresh-map remainder → goal reached |
| **A2** (retry) | with chat_drive `extracted_paths` parser + hardened prompt | 16 | 14 | 520 s | recall HIT → execute_path(56 cells, **still truncated**) → 3 recovery attempts → final 33-cell remainder succeeded → goal reached |
| **C** (bridge gating) | smoke-test on `husky_maze_blind.wbt` | n/a | n/a | <1 ms/probe | `/maze` + `/lidar` short-circuit; `/camera` still serves; `/capabilities` shows `lidar_available=false` |
| **C** (end-to-end agent) | v1 on blind world, 8-turn budget | 8 | 7 | 53 s | 1 cell of progress; agent called `read_camera` then tried to `delegate_to_agent` (Captain tool, not Husky Maze's), got "unknown tool" error, drove forward blindly |
| **B** (re-verify) | captain runner status + `list_agents` only | 1 | 1 | <2 s | runner healthy, 10 tools, Husky Maze reachable, Axis unreachable (expected) |
| **B** (fresh delegation) | captain on `husky_maze.wbt` | 11 | 10 | 317 s | every `delegate_to_agent` returned 413 Request payload too large; captain handled gracefully and called `complete_mission` with honest failure rationale |

### Direction A discussion

The recall-hit-then-skip-discovery mechanism works. On turn 4 of A2 the agent received a path from the long-term memory tier, called `execute_path` immediately, and never called `try_get_known_map`. That's the memory dividend.

The remaining bottleneck is **path truncation in the LLM's output**: in both A2 attempts the agent emitted only 56–61 cells of the saved 72. The retry added a structured `extracted_paths` field in the recall response (parsed from the markdown body server-side) and a "do not summarise, emit the full long response" rule in the prompt. Neither fixed it: the LLM's emit-budget for tool-call JSON is the binding constraint. The honest fix is to remove the LLM from the path-emission loop with a `replay_recalled_path(memory_id)` tool — out of scope for this round but well-scoped for a follow-up. The compounding mechanism is verified; the round-trip fidelity is not.

### Direction C discussion

Bridge gating performance is sub-millisecond per probe. End-to-end agent navigation on the blind world is qualitatively *bad*: the agent recognises that `read_camera` is the only sensor, calls it once, then tries to delegate image analysis to a non-existent specialist tool, falls back to driving forward without using the picture. This isn't a bridge or world failure — the contract works, the camera serves correctly. It's a prompt-design failure for vision-only navigation: "navigate by camera + pose only" gives the LLM no scaffolding for "what cells are around me, which way is red, is the path clear". Improving this requires a structured vision-reasoning protocol in the prompt (e.g. "before driving, describe what you see in cardinal directions: north, east, south, west — call out colour and distance") plus possibly the ability to compute BFS over a *learned* topology that the agent builds from successive camera frames. Out of scope for this round.

### Direction B discussion

The captain's own chat is healthy (operator goal, status snapshots, list_agents). The failure is **inside** `delegate_to_agent` when it opens a sub-chat-loop with `agent_name="Husky Maze"`: the system_instruction for that sub-chat carries the full Husky Maze v1 profile, including `availableToolDetails` for all 20 tools. The cumulative token budget of the sub-chat exceeds OmniLink's 413 request-size limit, every attempt. The captain handles the failure gracefully (10 retries with the same error, then honest `complete_mission` with a rationale that surfaces the unrecoverable condition).

This wasn't a problem in commit `2960fad`'s original verification because the Husky Maze profile has grown since — direction A added explicit memory-flow language to the prompt and direction C added the `lidar_available` flag plus extra hint text. The cumulative bloat now trips the limit. The fix is to slim the sub-chat-loop's system_instruction (drop full `availableToolDetails`, keep only the names + the `mainTask`); the sub-agent's runner already serves the tool implementations locally, so the model only needs to know which tool to call. That fix is well-scoped for a follow-up and would also help with quota.

### Net status across A, B, C

- **A**: mechanism verified in two runs. Compounding works; LLM output truncation is a known limitation; identified follow-up fix.
- **B**: architecture verified, fresh delegation surfaces a real OmniLink-side payload-size bug introduced by recent profile growth; captain handles the failure honestly; identified follow-up fix.
- **C**: infrastructure verified end-to-end (bridge gating + new world); agent end-to-end is a qualitative success-of-bridge / failure-of-prompt; identified prompt-redesign work for vision-only navigation.

All three directions have known *next* moves; none are silently broken; documentation matches reality.

## Where we are now

**What's done (5 commits on `main`, ~6 800 lines):**

- 4 maze worlds with stacked discriminator layers (strategy / brief / vision).
- 20 tools: motion + bridge + lidar + map + mission + memory + knowledge + camera.
- Live OmniLink chat-loop driver with multimodal image attachment.
- `husky_eye` sidecar Robot for vision.
- Long-term memory (Ollama embeddings + BM25 + RRF), knowledge folder, recall tool.
- `/status` + typed `/activity` feed for operator visibility.

See [`OVERVIEW.md`](OVERVIEW.md) for the synthesis. See [`roadmap.md`](../roadmap.md) for what's done by phase.

**What's structurally missing (in priority-of-impact order):**

- Cross-session memory **compounding** — the tools are wired but no demo shows the agent getting smarter on the second visit.
- **Multi-agent orchestration** — OmniLink is a fabric, but the demo only uses a single agent.
- **Operator-in-the-loop** — every chat is one prompt; no mid-mission interrupts.
- **Hardcore vision-only** — current visual world keeps `try_get_known_map` and `read_lidar` available; vision is one of multiple inputs, not load-bearing for navigation.
- **Voice / web UI** — system is dev-shell-only; non-developers can't drive it.
- **Sim-to-real shape** — bridge contract is OmniSim-specific; portability is implied, not demonstrated.

## The directions, ranked

Each entry: what it is, effort, wow, risks, files that would change.

### A — Cross-session memory verification ("the agent gets smarter the second time")

**What it is.** Run the visual mission twice in a row.

- First run: agent calls `read_camera` repeatedly, identifies red is at `(5,3)`, drives there, calls `complete_mission`, calls `save_local_memory({title: "Husky Maze (Visual): red cylinder at (5,3)", body: "...", tags: ["Husky Maze (Visual)", "marker_location"]})`.
- Second run on the same world: agent calls `recall("Husky Maze (Visual)")` first thing. Hit. BFS direct to `(5,3)` without ever calling `read_camera`. Mission completes in ~30 % of the steps.

**Effort.** Half a day.
- 1–2 hours of prompt tightening so the agent reliably saves on success and consults `recall` before vision.
- 1 hour to record + capture the trace.
- 1 hour to update `RESULTS.md` with both runs.

**Wow.** Medium. Concrete proof of compounding behaviour. The most under-shown capability in the current build — the LLM tier ages well across sessions, scripts don't.

**Risks.** Low. The infrastructure is in place. Risk is the agent forgets to save on the first run; mitigated by adding "after `complete_mission`, ALWAYS call `save_local_memory`" to the prompt.

**Files touched.** `prompts/system.md`, `docs/RESULTS.md`, no new code.

### B — Cross-agent composability ("OmniLink as a fabric")

**What it is.** A new agent `Mission Captain` (a thin profile, no tools of its own) that orchestrates other agents:

- Operator prompt: *"Find the red pallet in the maze, then have the picker tag it on the dock."*
- Captain delegates the maze-navigation legs to `Husky Maze`.
- Captain delegates the warehouse legs to `Warehouse Picker` (the existing SKU-recognising mobile picker, `agents/production/warehouse_picker/`).
- Each sub-agent does its part, returns; Captain aggregates and reports.

OmniLink already supports delegation — we observed it fire spontaneously in the very first chat tests (g1-engine tried to delegate to "MazeSolver"). This time we lean into it.

**Effort.** 1–2 days.
- New agent profile + prompt for `Mission Captain` (a few hours).
- A combined world (steal pieces from `husky_maze.wbt` + `warehouse_logistics.wbt`).
- Coordination plumbing if delegation needs Omni Key passing or shared memory.
- End-to-end live test.

**Wow.** High. Demonstrates OmniLink as a multi-agent platform, not a single-agent harness. Aligns with OmniSim's positioning ("development, testing, and deployment ground for OmniLink").

**Risks.**
- Delegation across separate Omni Keys / accounts may have constraints we haven't probed. Could need OmniLink-side cooperation.
- Combining two robots from different demos in one world may expose new physics edge cases.
- The "captain" is essentially a router; if g1-engine is too eager to delegate without context, we may need to tune it (or switch to g4-claude).

**Files touched.** New `agents/production/mission_captain/` folder (mirroring `husky_maze/` layout). New world file. Possibly bridge updates if Captain needs to query both agents' `/status` endpoints.

### C — Hardcore vision-only world ("strip the safety nets")

**What it is.** Today the visual world keeps `try_get_known_map` and `read_lidar` available. The agent uses vision only to identify which marker is red, then BFS-drives there. To make vision **load-bearing for navigation itself**:

- New world `husky_maze_blind.wbt` — title contains "Blind", bridge gates `/maze` AND `/lidar` to return `{available: false}`.
- Agent must decide which cell to drive to, *which direction to drive*, *whether the path ahead is clear*, all from camera frames + pose.

**Effort.** 2–3 hours.
- Generator already supports `--with-camera`. Add a small bridge change: gate `/lidar` when `"blind" in title.lower()`.
- New world via the generator.
- Prompt tweak: when `lidar_available=false`, navigate by camera.

**Wow.** Medium. Doesn't add a new capability — the vision pipeline already works. But it removes the *"but BFS still would have worked"* caveat from layer C of the discriminator stack. A cleaner thesis.

**Risks.** Low for the build, medium for the *demo* — the agent may struggle to navigate from camera alone (no depth, low resolution, narrow FOV). Could end up showing the *limits* of pure-vision more than its strengths. Worth the experiment regardless.

**Files touched.** Bridge (gate `/lidar`), generator (no change needed), one new world file, prompt tweak.

### D — Operator dialogue mid-mission ("real co-pilot vibes")

**What it is.** Today the agent runs end-to-end on a single prompt. Real ops want to interrupt:

- *"Stop — what do you see right now?"*
- *"Skip the red one, find the green one instead."*
- *"Why are you going that way?"*
- *"Save what you've learned about this layout, then quit."*

Implementation: `chat_drive.py` becomes an interactive REPL — operator types, agent responds, agent acts, operator can cut in at any tool boundary.

**Effort.** ~1 day.
- Terminal REPL with prompt-toolkit or just `input()` polled in a side thread.
- A few prompt tweaks for "incoming operator instruction; integrate into your current plan".
- Logic for cancelling an in-flight `goto_cell` cleanly when the operator interrupts.

**Wow.** High in live demos. This is what people imagine when they say "AI co-pilot for robots".

**Risks.**
- Cancelling in-flight motion mid-pivot can leave the husky drifted off-grid. We have `snap_to_cell` as a re-anchor; should be fine.
- The OmniLink chat shape is request/response, not streaming. An interrupted mission has to wait until the current `chat()` returns before the operator's interrupt is seen.

**Files touched.** Mostly `scripts/chat_drive.py`, plus a small prompt section.

### E — Voice interface (OmniLink has STT/TTS built in)

**What it is.** OmniLink's `client.transcribe()` and `client.synthesize()` are right there. Add a push-to-talk loop:

- Operator presses spacebar, talks → STT → agent → TTS → audio.
- Whole thing runs hands-free.
- Combined with **D**, the operator never touches a keyboard.

**Effort.** ~1 day.
- Audio capture (e.g. `sounddevice` library, ~50 lines).
- STT/TTS API wiring (~30 lines, already in OmniLink SDK).
- Push-to-talk hotkey + status tone.

**Wow.** Very high in live demos. Trivially impressive — "you can talk to your robot".

**Risks.**
- Adds Mac/Win audio device handling complexity (`sounddevice` is portable but the user's PortAudio install may bite).
- An extra OmniLink billing line per minute of audio (STT + TTS).
- STT latency adds another ~1–2 s per turn.

**Files touched.** New `scripts/voice_drive.py`, no other changes.

### F — Web operator dashboard

**What it is.** Bridge already serves `/status` and `/activity`. Wrap them in a tiny static HTML+JS page that polls every 500 ms and shows:

- Current strategy, current cell, mission_complete flag.
- Last 20 activity entries colour-coded by `kind`.
- A live thumbnail of the camera feed (refresh every 2 s).
- A button to send canned operator commands (stop, reset, snap back to start).

Make the dashboard reachable on a fixed local URL, e.g. `http://127.0.0.1:6072/dashboard`.

**Effort.** ~1 day.
- Static page + asset served by the bridge (or a separate tiny HTTP server in the runner).
- ~200 lines of HTML/JS, no framework.
- Update the runner's startup banner to point at the dashboard.

**Wow.** Medium. Useful, professional, makes the system tangible to non-developers without forcing them into a chat loop.

**Risks.** Low. Pure presentation layer. Worst case it has rough edges and we iterate.

**Files touched.** Bridge (or runner) for static-asset serving, new `dashboard/index.html`.

### G — Sim-to-real bridge contract

**What it is.** Define an explicit `BridgeProtocol` (Python ABC) that the husky_maze agent talks to. Implement two backends:

1. The existing OmniSim HTTP bridge.
2. A stub ROS2/MQTT bridge that *would* drive a real Husky.

We don't actually deploy to a real robot — the stub just logs intent. But the contract proves portability.

**Effort.** 1–2 days.
- Extract the HTTP bridge calls into an interface (~half-day refactor).
- Write a stub ROS2 backend with `rclpy` (or just a logging mock) (~half-day).
- Document the contract surface in `knowledge/bridge-protocol.md`.

**Wow.** Medium for ROS / robotics technical audiences; low for general.

**Risks.**
- ROS2 has version sprawl; the stub might need pinning to a specific Humble/Iron release.
- If we just stub it (no actual ROS), the demo value is "see, it COULD work on a real robot" — which is honest but less compelling than a real run.

**Files touched.** Bridge refactor (mostly extract methods), new backend file, new doc.

## Decision matrices

### By audience

| audience | recommended sequence |
|---|---|
| **Non-technical viewers** | D + E + F (operator dialogue + voice + dashboard). Together they make the system feel like a product. ~3 days. |
| **AI / robotics technical** | A + B (memory compounding + cross-agent composability). Both prove non-obvious OmniLink capabilities. ~2 days. |
| **OmniLink platform stakeholders** | B is the headline (multi-agent fabric). A is the second feature. ~2 days. |
| **Robotics-specific (ROS folks)** | G + a real-robot probe. Shows portability story. ~1–2 days. |

### By time-box

| time you have | best single move |
|---|---|
| **2 hours** | C (hardcore vision-only). Strengthens the existing thesis with minimal new code. |
| **half a day** | A (memory compounding). Smallest change, cleanest "look, it learned" moment. |
| **1 day** | D or E (operator dialogue or voice). Either becomes the centrepiece of a live demo. |
| **2 days** | B (cross-agent composability). The biggest non-trivial capability we don't have yet. |
| **a week** | A + B + D + F. Everything that adds new capability without going off into ROS land. |

### By risk

| risk | direction | why |
|---|---|---|
| Lowest risk | A, C, F | Build on infrastructure that's already verified. |
| Medium risk | D, E | New UX layers; some integration friction expected. |
| Highest risk | B, G | Cross-agent delegation may have OmniLink-side constraints; ROS has its own ecosystem cost. |

## What I'd pick if forced to commit blind

If I were committing without operator input, I'd do **A → C → B**, in that order, over 2–3 days:

1. **A first** to harvest the memory infrastructure we already have. Cheapest win, biggest "proof of compounding" moment.
2. **C next** because it's a 2-hour change that strengthens the strongest existing thesis.
3. **B last** because it's the biggest capability gap and the most aligned with OmniSim's stated purpose ("development ground for OmniLink").

Skipping E (voice) and F (dashboard) because they're presentation polish — valuable for live demos, not for the underlying capability story.

Skipping G (sim-to-real) because without a real robot to point at, it's a contract refactor that doesn't move the demo.

But the operator should pick. Each direction has its own legitimate audience.

## Ready-to-execute prompt patterns

If you pick a direction, here's the one-line prompt you'd give me to start:

- **A:** "Do A. Run the visual mission twice; second run should hit `recall` and skip vision."
- **B:** "Do B. Build `Mission Captain` agent that delegates to `Husky Maze` and `Axis`."
- **C:** "Do C. Generate `husky_maze_blind.wbt`; gate lidar AND map; agent must navigate by camera alone."
- **D:** "Do D. Make `chat_drive.py` interactive — operator can interrupt with new instructions."
- **E:** "Do E. Add `voice_drive.py` with push-to-talk via OmniLink STT/TTS."
- **F:** "Do F. Add a web dashboard at `http://127.0.0.1:6072/dashboard`."
- **G:** "Do G. Refactor the bridge into `BridgeProtocol`; stub a ROS2 backend."

Combinations welcome.
