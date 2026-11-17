# OmniLink agent patterns

Cross-demo design patterns extracted from the OmniSim agent builds (`husky_maze`, `mission_captain`, `warehouse_foreman`, `warehouse_patrol`). These are the choices that paid off — each one earned its place by being the lesson learned from a specific failure mode documented in that demo's docs/RESULTS.md.

> **On the `warehouse_*` citations below.** The Warehouse Foreman, Warehouse Picker and Patrol Squad demos were removed from the tree on 2026-07-19 (their results predated the `b18bd7a3` teleport-recovery removal and were never re-measured). The citations are kept because they are the **provenance** of these patterns — each names the run that produced the lesson. They are historical references, not paths you can open; recover them from git history if you need the detail. The patterns themselves are live and apply to every new agent.

This is the doc to read **before** starting a new agent. The default for every new demo should be: do the thing this doc recommends; if you need to deviate, write down why in the demo's docs.

---

## 1. Perception-as-tool, not pixels-to-LLM

**Pattern.** Vision belongs in a sidecar process that runs deterministic image analysis and exposes a structured-tag API. The agent calls a tool, gets back JSON like `{tag_color: "green", marker_fraction: 0.0038, marker_centroid: {x_norm: 0.49, y_norm: 0.62}}`. The agent NEVER sees pixels in the steady-state path.

**Why.** A 320×240 PNG attached as `image_url` part costs ~12 k input tokens. A structured tag digest is ~80 tokens. **150× per-query difference.** Plus: deterministic across runs (the LLM may misread on any given snap), preserves cacheability (the per-turn input doesn't drown out the cacheable system prefix), and faster (no LLM-pixel-decoding latency).

**How.**
- Eye sidecar process (e.g. `husky_eye`) owns the OmniSim Camera. Pure-Python BGRA classifier returns `{tag_color, marker_pixels, marker_fraction, marker_centroid, color_fractions, ...}` per camera view.
- Bridge proxies sidecar `/scan` so the agent only knows about one HTTP target.
- Agent gets a `scan_for_tag` (or `scan_surroundings` etc.) tool with description like "PREFERRED tag-identification tool — never sends pixels to you. ~150× cheaper than read_camera."
- Keep `read_camera` as an explicit FALLBACK tool for genuinely ambiguous frames. Discriminator argument still holds (the agent CAN read pixels when the structured digest is uncertain), the cost story still holds (agents prefer the cheap path).

**Counter-evidence we considered.** "But the LLM should see pixels — that's the agent value." Reality: the LLM's job is reasoning over structured state, not OCR. Vision-as-tool is what makes the agent layer scale. If the demo *requires* the LLM to do free-form scene description, fine — but verify whether you actually need it before making it the default.

**Where this applied:**
- `husky_maze_visual.omniworld` — `scan_surroundings` tool returns `{open: [cardinals], blocked: [cardinals], marker: {color, world_cardinal, ...}}` per turn
- `warehouse_foreman` iter-3 — switched from `read_camera` to `scan_for_tag`; cost dropped 32 % and cache hit rate doubled

---

## 2. Ground-truth verification, not agent claims

**Pattern.** Every tool that performs a side-effect must have a corresponding ground-truth read endpoint. Tools that claim success must call that endpoint to verify, not just trust the actuator's "command accepted" return.

**Why.** Agents will report whatever the tool returns. The tool will return whatever the bridge's optimistic acknowledgement says. The bridge's optimistic acknowledgement is "I sent the command" — it doesn't know if the wheels turned, the pallet moved, or the arm reached. **Without a ground-truth verification path, the demo lies.**

**How.**
- Bridge exposes `/state`, `/solid?def=NAME`, `/read_tcp_pose` etc. — supervisor reads of actual world state.
- Tools that perform actions call these endpoints in the same dispatch and incorporate the result into the tool's return value (e.g. `push_pallet_to` returns `{status: "off_target", delivery: {actual_position, delivery_error_m, delivered: false}}` when ground truth disagrees with intent).
- Status `"ok"` on the tool's return ONLY when ground-truth agrees with the goal.
- The agent's prompt instructs: "if the tool returns status=off_target, do NOT claim mission complete; report honestly and either retry or escalate."

**Where this applied:**
- `warehouse_foreman/tools/picker.py` — `push_pallet_to` re-reads `/solid?def=LOAD_GREEN` between every push segment AND for final delivery confirmation
- `husky_maze` — bridge tracks `mission_complete: bool` set only by the agent's `complete_mission` action AFTER `goal_reached: true`

**The failure mode this prevents.** Earlier in `warehouse_foreman` an agent claimed "GREEN delivered to dock" while the pallet was actually 16 m away in a wall corner. The user caught it from the screenshot. After adding `/solid` + verification, that class of failure is structurally impossible to reach.

### 2b. Ground-truth grading survives a locomotion defect — self-report does not

**Pattern.** When a locomotion or actuation bug is found *after* results were published, the results do not all fall together. Partition them by **how each number was graded**, and retract only the partition whose grading depended on the broken layer.

**Why.** The instinct after finding a defect is to retract the whole page. That over-corrects, and it destroys measurements that were never contaminated. A result graded by a supervisor read of world state is independent of *how* the robot got there — so a defect in the getting-there does not touch it.

**How — the partition.**
- **Survives:** outcomes graded by supervisor ground truth (`GET /solid?def=…`). The pallet either was inside the dock zone or it wasn't; a teleport in the route doesn't change where the pallet ended up. Detection findings (which crates moved, by how far) likewise stand.
- **Survives:** measurements of a *different layer* entirely — `$/hr`, `$/mission`, tokens per turn. These measure the LLM, not the wheels, and no locomotion defect can reach them.
- **Retract:** any claim the robot *navigated* the route it was asked to navigate, and any per-waypoint or wall-clock figure that assumes locomotion succeeded honestly.
- A demo with **no** ground-truth grading has no surviving partition and must be retracted wholesale — which is precisely the difference between the `husky_maze` retraction and the caveat banners on the two warehouse demos.

**Corollary — this is the argument for building `/solid` at all.** The endpoint's value is not only that it catches a lying agent in the moment (§2 above); it is that results graded through it remain quotable after a defect is found in a layer *below* the grading. Demos graded by self-report have to be thrown away.

**Terminology trap.** Distinguish an *operator* teleport (deliberately repositioning objects to create ground truth for a test) from a *recovery* teleport (the bug — snapping a stuck robot to its target instead of reporting the fault). The first is legitimate test setup. Sweep notes reading "after teleport" have meant both, in the same repo.

> Extracted 2026-07-19 from the caveat banners on `warehouse_foreman` and `warehouse_patrol`, whose numbers predated commit `b18bd7a3` (wedge-recovery teleport removal). Both demos have since been removed from the tree; the reasoning is kept here because it is general.

---

## 3. Multi-agent orchestration with strict honesty contract

**Pattern.** When one agent delegates to another, the orchestrator's "did this leg succeed" check must be a state-transition observation (counter increment OR fault-flag flip), NOT the sub-agent's narration of its own success. The orchestrator's `mainTask` includes a hard rule: "if any leg's `success=false`, NEVER call `complete_mission` — investigate or escalate."

**Why.** Sub-agents will narrate completion under failure conditions ("Mission complete!" with the pallet still in the rack). Orchestrators that trust narration become credulous narrators of their specialists' false claims. This compounds — the operator gets a 3-deep layer of "everyone says it worked."

**How.**
- Each runner exposes `/status` with a `complete_calls_this_session` counter that increments only when the agent's `complete_mission` tool actually fires.
- Orchestrator snapshots the counter BEFORE delegation, then checks for an increment after each chat turn. Returns `success=true` only on observed increment from baseline (or `mission_complete` flag transition for agents that surface it via the bridge).
- Orchestrator's `mainTask` enforces the strict honesty rule explicitly: "Only when EVERY delegated leg returned success=true do you call your own complete_mission."
- `mission_complete` and `complete_calls_this_session` fields propagate up so the chain of trust is end-to-end inspectable (foreman sees picker's counter, chat_drive sees foreman's counter, operator sees chat_drive's exit).

**Where this applied:**
- `mission_captain/tools/orchestration.py` — generic `delegate_to_agent` that snapshots the sub-agent's pre-status and only declares completion on a transition
- `warehouse_foreman` — added the "graceful exit on consecutive no-tool-call turns" branch for sub-agents that have no `complete_mission` tool

---

## 4. Hint-first scanning, not exhaustive

**Pattern.** When the orchestrator has knowledge the specialist could use (pallet coordinates, room labels, suspected target locations), pass it in the delegation. The specialist's prompt should verify the hint FIRST and fall back to exhaustive scanning only if the hint disagrees with sensor reality.

**Why.** The specialist's prompt phrasing affects how much motion (and how many tokens) it spends. "Scan all six pallets" produces 5× more vantage drives + camera snaps + reorientations than "verify the hint position". And in our case the hint was right — the agent's vision call confirmed it on the first try. The four extra scans were pure waste.

**How.**
- Orchestrator's delegation message includes the hint AND the alternatives: "GREEN should be at (3, 5). Pallet positions: red (-3, 5), green (3, 5), blue (9, 5), yellow (-3, -5), magenta (3, -5), cyan (9, -5)."
- Specialist's `mainTask` workflow: (a) drive to the hint position, (b) snap, (c) if confirmed → act on it, (d) if disconfirmed → THEN scan the alternatives one at a time.
- The vision discriminator is preserved (agent reads ground truth from sensor, not from the hint) but the typical happy-path takes 1 scan instead of N.

**Where this applied:**
- `warehouse_foreman` iter-1 — picker prompt rewritten to "trust the brief's hint, scan one"; cost dropped 72 %
- `mission_captain` — captain's delegation messages include cell coordinates rather than just colour names

---

## 5. Durable shared cache for explicit context caching on serverless

**Pattern.** When using Gemini's (or Anthropic's) explicit context-cache APIs from a stateless serverless deploy, the `(cache_key → cache_name)` lookup MUST be backed by durable shared storage (Supabase / Redis / KV). Process-local memory alone gives 1–4 % hit rate because the serverless host cold-starts each request onto a fresh container instance.

**Why.** Without a shared lookup, every cold-start instance and every parallel container instance re-creates the cache (paid at full input rate) instead of finding the existing one. The cache becomes pure overhead. With a shared lookup, the cache amortises across the entire fleet and you actually pay the discount rate on repeats.

**How (as implemented in OmniLink's server-side cache layer):**
- Tier 1 — process-local `Map` (warm-instance fast path, no round-trip)
- Tier 2 — a durable shared table (e.g. Supabase/Postgres) keyed by `cache_key`, storing the provider `cache_name` and an `expires_at`
- Lookup order: Map → Supabase → create new
- Write order: create → Map AND fire-and-forget Supabase upsert
- Race-safe: parallel instances may both create + upsert; whichever lands last wins
- Opportunistic GC: on lookup, expired rows get deleted inline

**Measurement plumbing (don't ship caching without this):**
- Server: extract `cachedContentTokenCount` (Gemini) and `cache_read_input_tokens` (Anthropic) from response usage; write to a `cached_input_units` column on the per-event log
- DB: rollup view exposes `cached_input_units_24h/7d/30d` alongside `input_units_*`
- Library: `UsageDelta.cached_input_units`, `cache_hit_ratio`, `fresh_input_units`
- Runner: `/status.usage` includes the new fields
- Operator surface: HUD or chat_drive end-of-run summary shows cache hit % and dollar savings

Without measurement plumbing, you can't tell the difference between "caching is working but our prefix is small" and "caching is broken." Both look like a low number on the bill.

**Where this applied:**
- `warehouse_foreman` iter-2 — added the Supabase tier; hit rate ticked up but real win came in iter-3 when perception-as-tool reduced per-turn input enough that cache savings became significant

---

## 6. Local sub-chat-loops for orchestration, not server-side delegation

**Pattern.** When an orchestrator needs to drive a specialist that exposes its tool surface on `127.0.0.1`, the orchestrator runs the sub-chat-loop in its own process — it doesn't ask the OmniLink platform to dispatch the specialist's tools server-side.

**Why.** The OmniLink platform can't reach `127.0.0.1:51520` from the internet. Server-side function-call dispatch only works for tools that are themselves callable from the platform (REST APIs, web hooks). Local hardware-driving agents always have local-only `/tool` endpoints.

**How.**
- `tools/orchestration.py` `delegate_to_agent`:
  - Loads the specialist's profile from OmniLink
  - Slims the profile (compress tool descriptions to first sentence — full descriptions blow past the platform's 413 payload limit)
  - Loops: chat → dispatch tool calls against the specialist's local `/tool` endpoint → fold results back as next user message → repeat
  - Exits on completion-counter increment OR consecutive no-tool-call turns
- The orchestrator's chat() to the platform is shorter (just the orchestrator's narration + the high-level delegation result) — the specialist's verbose tool-loop happens out of band.

**Where this applied:**
- `mission_captain` — local sub-chat-loop driving the Husky Maze specialist
- `warehouse_foreman` — same pattern, one specialist (Picker) under one orchestrator

---

## 7. Cross-protocol bridge shims for productized-agent reuse

**Pattern.** When you want to reuse a productized OmniLink agent (Axis, etc.) against a new bridge whose protocol differs, add a compatibility shim in the bridge — don't fork the agent.

**Why.** The productized agent has its own release cadence, tests, prompt, memory. Forking duplicates maintenance. Shimming keeps the productized agent consumable as-is and isolates the protocol translation in the bridge (where it belongs — protocols are bridge-side concerns).

**How.**
- Bridge `do_POST` checks the request path:
  - `/action` → native protocol (used by anything written for the current bridge)
  - `/{verb}` → axis-shape compat shim, routes to the same dispatch table with the action name derived from the path
- For coordinate-frame translations (like world ↔ robot-local), do them at the shim boundary — the native `/action` path stays in robot-local coords (its existing contract), the shim translates on the way in and back out.

**Where this applied:**
- A 6-DOF arm bridge — Axis-compat shim with world↔robot-local coordinate translation. Axis (built against an older per-endpoint protocol) drove the arm with zero code changes on the Axis side.

---

## 8. Default to durable observability for cost-curve work

**Pattern.** Anything that affects cost gets measured end-to-end before being optimised. The runner's `/status.usage` block is the canonical source of truth; the HUD and chat_drive summary read from it; `RESULTS.md` cites the snapshot rather than the in-process estimate.

**Why.** Optimisations are cheap to claim, expensive to verify. Without measurement plumbing, "we made it 2× faster" is a guess. With it, every iteration's cost shows up in the same comparison table and the user can see whether the lever you pulled actually moved the needle.

**How.**
- `omnilink-lib.usage_meter.UsageMeter` snapshots the platform's 24h-rolled token + credit counters at runner startup; `snapshot()` returns the delta over any window.
- Runner's `/status.usage` exposes `input_units, cached_input_units, fresh_input_units, output_units, cache_hit_ratio, tokens_per_hour, credits_per_hour, elapsed_s`.
- chat_drive prints the delta at end-of-run.
- HUD polls `/status` every 1.5s and renders the live numbers in the OmniSim dock pane.
- `RESULTS.md` uses the snapshot's per-mission cost (gross AND effective with cache) as the headline metric.

**Where this applied:**
- All warehouse_foreman iterations are comparable because every run captures the same metrics into the same table.
- Husky Maze CSV has cost-per-mission for every strategy/world combination — iterations over time become quantitatively meaningful.

---

## 9. Fix the tool, not the prompt

**Pattern.** When an agent underperforms at driving a robot, suspect the tool contract before
the system prompt. Measure the primitive against ground truth first; only then look at the
model.

**Why.** An agent has no independent access to the world — every belief it holds about what the
robot did came from a tool result. A tool that returns the value it was *asked for* installs a
false belief, and the agent then reports that belief confidently, because reporting what its
tools said is what a well-behaved agent does. Its honesty is bounded above by the tool's
honesty, and no prompt raises that ceiling.

Three measurements from this repo, all with the model held constant:

- The mobile bridge's `turn` delivered **56.7% of the commanded angle** — reproducible to four
  decimals — while its own source comment documented it as "~−19%". Rewriting the control law
  took it to **mean |error| 0.44°, max 0.97°, n=8** (commit `52f3f6ca`). No prompt, model, or
  agent configuration changed.
- Case 5 of the swarm showcase went **2/4 → 4/4 within 0.08 m** by adding `drive_to_xy` /
  `drive_radial`. Same model, same robot, same prompt family. The fix was taking the
  trigonometry away from the model, not explaining it better.
- The anti-fabrication study found 26% of turns contained a fabrication, every one about the
  agent's own past actions, and **not one fabrication came from a tool read**.

**How.**
- Motion results carry `{commanded, achieved, error, settled}` — never the argument echoed back.
  Unmeasured is `null`, never a number you did not measure.
- Actions complete before returning, or the description says plainly that they do not *and*
  hands back something to wait on that cannot fire before the motion started.
- One call does what the agent wants: `drive_to(x, y)` rather than `turn` then `drive`. The tool
  owns the geometry, the frame, and the units.
- A second command arriving while busy is **rejected** (`409`, with a hint), never silently
  clobbered onto a single motion slot.
- If the tool is wrong by a known amount, that amount goes in the description until it is fixed.
  It costs a string and it is the most-skipped item on the list.

**Where this applied:** `omnilink_mobile_bridge` (`turn`), `husky_swarm` (`drive_to_xy`).

**The limit, stated so this does not become dogma:** better tools ≠ more tools — selection
accuracy degrades as the surface grows, so consolidation is part of the pattern. And this is
proven at the *primitive* level here; the task-level lift on `omnilink-bench` is the next
measurement and has not been made. Full write-up, evidence and limits:
[docs/developer/tool-design-for-agents.md](../docs/developer/tool-design-for-agents.md).

---

## Quick checklist for a new agent

When starting a new demo (Patrol Squad, Drone Surveyor, Tour Guide, …), work through this list before writing the first line of `_agent.py`:

- [ ] Does every motion tool return ACHIEVED rather than commanded, and complete before it returns? (Pattern 9 — check this before writing a line of prompt)
- [ ] What's the bridge's ground-truth verification endpoint? (`/state`, `/solid`, `/read_tcp_pose`, …) — write it BEFORE writing the action endpoints
- [ ] What's the perception sidecar? Pure-Python tag classifier returning structured digest, NOT pixels to LLM. Add `read_camera` as a fallback only.
- [ ] What goes in the orchestrator's hint? (Coordinates, room labels, suspected targets) Specialist verifies one, falls back to exhaustive only on miss.
- [ ] Does the orchestrator have a strict honesty contract in its `mainTask`? "Never claim if any leg returned success=false."
- [ ] Is caching plumbed end-to-end? Lookup → process Map → Supabase → create. Measurement → cached_input_units column → /status field → chat_drive summary.
- [ ] If you're driving a productized agent (Axis etc.) against a new bridge, is the protocol shim on the bridge side or did you accidentally fork the agent?
- [ ] Does the runner's `/status` give you tokens/hour, cache hit %, and dollars/hour? If not, you can't measure your optimisations.

If all of these are checked before the first chat() call, the demo will land in the cheap regime by default and the cost-iteration cycle gets shorter.
