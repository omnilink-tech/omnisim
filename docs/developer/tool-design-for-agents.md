# Tool design for agents: the contract is the product

**Status:** principle + evidence. 2026-07-26.
**Thesis:** for an LLM driving a physical system, **the tool contract is a first-order term in
task success, and the cheapest one to fix.** If you have one hour to spend making an agent
better at driving a robot, spend it on the tool, not on the system prompt — a broken primitive
caps every agent that calls it, and no prompt lifts that cap.

⚠️ **This does not say the tool term beats the model term.** Every measurement in §1 holds the
model *constant*, so none of them can rank the two. Held the other way — same tool surface,
varying the model — the spread is 4/4 to 0/4 (§5, first bullet). The experiment that would
actually rank them, a model ladder at fixed tool surface, is **Phase M of
[agent-edge-validation-plan.md](agent-edge-validation-plan.md) and has not been run**
(that file's own status line: "Nothing in this file is a result"). The defensible short form is
the one [AGENTS.md](../../AGENTS.md) uses: *suspect the tool before the prompt.*

This document exists because we proved it on ourselves, twice, by accident.

---

## Contents

- [0. The one-paragraph version](#0-the-one-paragraph-version)
- [1. What we measured](#1-what-we-measured)
- [2. Why it works this way](#2-why-it-works-this-way)
- [3. The four properties of a tool an agent can succeed with](#3-the-four-properties-of-a-tool-an-agent-can-succeed-with)
- [4. External evidence](#4-external-evidence)
- [5. The limits — where this stops being true](#5-the-limits--where-this-stops-being-true)
- [6. What this means for how we judge our own surface](#6-what-this-means-for-how-we-judge-our-own-surface)
- [7. The checklist](#7-the-checklist)

---

## 0. The one-paragraph version

An LLM agent has no independent access to the world. Every belief it holds about what the
robot did came from a tool result. So a tool that returns the value it was *asked for* — rather
than the value the robot *achieved* — does not merely fail to help: it actively installs a false
belief, and the agent will then report that false belief confidently and correctly, because
reporting what its tools told it is exactly what a well-behaved agent does. **The agent's
honesty is bounded above by the tool's honesty.** You cannot prompt your way past that
ceiling, and every hour spent trying is an hour not spent raising it.

---

## 1. What we measured

### 1.1 The turn primitive: −43% → 0.44°, with the model out of the loop entirely

The mobile bridge's `turn` tool delivered **56.7% of the commanded angle**. Not noise —
reproducible to four decimal places across trials, including sign reversal. Its own source
comment documented the defect as "~−19%" and declared a corrector unworkable, so it was left
alone and never re-measured.

Measured on `omnilink_husky_swarm.omniworld` (Newton/MuJoCo, RTX 3060 laptop, machine
`9722d23d12a3`), through the shipped tool:

| commanded | before | after | error after |
|---|---|---|---|
| +90° | +51.0° | **+90.02°** | +0.02° |
| −90° | −51.0° | **−89.75°** | +0.25° |
| +45° | +32.0° | **+45.43°** | +0.43° |
| +180° | (wrong direction) | **+179.03°** | +0.97° |
| −180° | (wrong direction) | **−179.03°** | −0.97° |
| +270° | (unreachable) | **+270.05°** | +0.05° |
| −30° | — | **−30.47°** | −0.47° |
| +10° | — | **+10.36°** | +0.36° |

**Mean |error| 0.44°, max 0.97°, n=8.** `drive` unchanged at 0.993 / 0.997 — no regression.

**No prompt changed. No model changed. No agent configuration changed.** The fix is entirely
inside [`omnilink_mobile_bridge.py`](../../projects/samples/demos/controllers/omnilink_mobile_bridge/omnilink_mobile_bridge.py)
(commit `52f3f6ca`). Any agent-level metric that depends on turning accurately was capped at
"57% of what you asked for" and no amount of prompting could have lifted it.

### 1.2 The controlled A/B: same model, 2/4 → 4/4

Recorded independently in [`agents/production/husky_swarm/README.md`](../../agents/production/husky_swarm/README.md).
Same model, same robot, same simulator, same relay, same prompt family — **only the tool
surface changed**.

Asked to move each of four Huskies 1 m outward with every coordinate supplied, the model got
the two `+x` quadrants right and drove the two `−x` quadrants down the wrong diagonal, then
reported all four succeeded. **2/4.**

The fix was not a better prompt about trigonometry. It was `drive_to_xy` and `drive_radial` —
closed-loop, absolute-coordinate, returning `error_m` and `arrived`. The tool took the
geometry away from the model. **4/4, within 0.08 m.**

### 1.3 The fabrication study: the model was never the liar

From this repo's own anti-fabrication stress test, recorded in
[`action_journal.py`](../../packages/omnisim-bridges/src/omnisim_bridges/action_journal.py):

> 26% of turns contained a fabrication. Every one of them concerned the agent's own past
> actions. **Not one fabrication came from a tool read — every claim sourced from a tool was
> correct.**

That is the whole thesis in one measurement. The model faithfully reports what tools tell it.
The mobile tools told it `{"accepted": true, "distance": <what you asked for>}` **0.01 s after
the call, when the robot had moved 1.9 cm of a commanded 1 m.** The journal built to catch the
symptom states its own ceiling in its docstring: *"If a caller wants 'how far did I actually
move', the underlying tool result has to carry that."* It didn't.

---

## 2. Why it works this way

Three properties of LLM agents make the tool contract load-bearing in a way it is not for
human operators:

1. **No independent world access.** A human driving a robot sees it move. An agent sees a JSON
   object. The JSON *is* the world, epistemically. A wrong JSON is not a degraded observation,
   it is a confidently wrong one.
2. **No sense of elapsed physical time.** An agent cannot notice that 1.8 s should have passed.
   If a tool returns instantly with `accepted: true`, nothing in the agent's experience
   distinguishes that from completion. Asking it to "wait for the robot" in a prompt is asking
   it to simulate a clock it does not have.
3. **Errors compound multiplicatively.** An *n*-step task at per-step reliability *p* succeeds
   at *pⁿ*. A tool at 0.57 fidelity is not "a bit off" over four steps — it is 0.11. This is
   why capability lanes collapse while single-step lanes look fine, and why per-step tool
   fidelity buys more than anything applied at the top of the loop.

The corollary: **prompt engineering is a multiplier on the tool contract, not a substitute
for it.** Multiplying a broken contract by a better prompt gets you a confidently-narrated
broken contract, which is worse than an obviously broken one because it passes review.

---

## 3. The four properties of a tool an agent can succeed with

### P1 — Complete before you return, or say plainly that you did not

The single largest structural defect a robot tool can have is returning before the action
happened. If the action is genuinely asynchronous, the tool must (a) say so in its
description, in words, and (b) hand back something the agent can wait on — a sequence number,
a handle, a `wait: true` option. "Poll until idle" is only acceptable if `idle` cannot be true
before the motion started; see the TOCTOU trap in P4.

### P2 — Return achieved, never commanded

A motion result should carry `{commanded, achieved, error, settled}`. Echoing the argument back
is worse than returning nothing, because it looks like a measurement. If the tool genuinely
cannot measure the outcome, it must say `achieved: null` — never a number it did not measure.

This is the same rule OmniBench applies to benchmark rows (*"unmeasured cost is `null`, never
`0.0`"*), applied one layer down.

### P3 — The tool owns the geometry, the frames, and the units

Never make the model compose a rotation with a translation, transform between frames, or call
`atan2`. Expose `drive_to(x, y)` in a named frame; do the trigonometry in Python where it is
exact. Every tool description must state its frame (world vs body), its units, and whether the
call blocks. Two tools in the same list defaulting to different frames — as the arm surface
does today, `set_tcp_target` in base frame next to `place.xyz` in world frame — is a defect
even though both are individually documented.

### P4 — Publish your own error, and refuse ambiguity rather than accepting it

A tool that is ±40% and whose description implies exactness is worse than one that says so.
Until the turn was fixed, the honest description would have been *"yaw undershoots by 20–45%
under the current solver; read `yaw` back and re-issue the residual."* Shipping a −43%
actuator behind a description implying exactness was the most expensive silence in our surface.

Corollaries with teeth:

- **A single-slot motion register must reject, not clobber.** Ours silently cancelled the
  previous motion, so `turn` then `drive` — dispatched back-to-back with zero delay by the
  relay — aborted the turn milliseconds in and drove on a barely-rotated heading. Both calls
  returned `accepted: true`. A `409 {"error": "busy", "hint": "wait for mode==idle"}` is a
  worse-looking API and a far better one.
- **A wait primitive must not be able to return true before the work started.** `idle` alone
  is a TOCTOU race: poll fast enough and you observe the pre-motion idle. Gate on a sequence
  number, not a mode string.
- **Publish a capability list.** An agent that cannot discover the verb set will guess, and
  the guesses land as 404s that cost a turn each.

---

## 4. External evidence

Cited to keep our own two data points honest — these are other people's measurements, not
ours, and they point the same way.

| finding | measurement | source |
|---|---|---|
| Async actions collapse the same model | ReAct + GPT-4o: **47% synchronous → 11% asynchronous** tasks | Robotouille, [arXiv 2502.05227](https://arxiv.org/abs/2502.05227) |
| Execution feedback is the biggest lever | object-recognition feedback 45%, success-detection 40%, **both combined 90%** | Inner Monologue, [arXiv 2207.05608](https://arxiv.org/abs/2207.05608) |
| Action-space level dominates | GPT-4o **57.7%** high-level nav vs **28.9%** low-level manipulation | EmbodiedBench, [arXiv 2502.09560](https://arxiv.org/abs/2502.09560) |
| Affordance grounding matters | PaLM-SayCan 84% planning; **67%** with grounding ablated | SayCan, [arXiv 2204.01691](https://arxiv.org/abs/2204.01691) |
| Self-critique *without* external signal is net-negative | GPT-3.5 fixed 7.6% of wrong answers, **broke 8.8% of correct ones** | [arXiv 2310.01798](https://arxiv.org/abs/2310.01798) |
| Multi-turn tool use is where agents fail | best GPT-4o agent **>60% pass¹ but <25% pass⁸** | τ-bench, [arXiv 2406.12045](https://arxiv.org/abs/2406.12045) |
| Code over the tool API beats one JSON call per turn | **+20.7 pp absolute**, 30% fewer actions | CodeAct, [arXiv 2402.01030](https://arxiv.org/abs/2402.01030) |

The Inner Monologue result is the one to internalise: a physics simulator is precisely the
crisp external verifier that literature says is *required* for self-correction to work at all.
We are sitting on the ingredient other people have to approximate. It is worth very little
until the tools actually carry it back to the model.

---

## 5. The limits — where this stops being true

Stated plainly, because a principle without limits is marketing.

- **Model choice is a large, separately-measured term, and we have never raced it against the
  tool term.** Holding the tool surface fixed and varying the model, the sibling record in
  [`agents/production/husky_swarm/README.md`](../../agents/production/husky_swarm/README.md)
  (cross-engine benchmark, 2026-07-22) has Gemini 3 Flash at **4/4**, Grok 4.1 Fast at **4/4**,
  the best open model tested at **2/4**, and several at **0/4** — on the same four tasks, the
  same relay and the same tools. Its hard tier separates them further: Gemini **5/6** vs Grok
  **2/6**, and its own conclusion is explicit — *"it is not a mechanical or tool-format
  deficiency; the tool calls it does make are excellent. The gap is inference and refusal."*
  So a tool fix cannot substitute for a capable model, the two terms have never been varied
  against each other in this tree (Phase M, unrun), and **nothing here licenses "the tool
  matters more than the model."**
- **We have proven this at the primitive level, not yet at the task level.** Turn accuracy went
  39° → 0.44°, and the mobile bridge now satisfies the §3 properties in full (commit
  `a2a8da5d`, 7/7 verified: `wait` returning achieved, `drive_to(x,y)`, `409 busy`, published
  capabilities). The lift in *agent task success* on
  [`omnilink-bench`](../../tests/benchmarks/omnilink_tasks/) is still the next measurement and
  **it has not been made.** The strongest task-level evidence we hold
  is §1.2, which is one case in one showcase. **Do not quote a task-level number until one
  exists.**
- **Better tools ≠ more tools.** Tool-selection accuracy degrades with surface size — one
  measured example has Gemini 2.0 Flash at 87.4% with 500 tools and 65% with 2,000
  ([arXiv 2602.23367](https://arxiv.org/abs/2602.23367)); OpenAI's guidance is to keep fewer
  than ~20 live at a turn. Consolidating `turn` + `drive` into `drive_to` is a *reduction* in
  surface and an increase in capability. That is the shape to aim for.
- **A tool cannot supply reasoning the model does not have.** P3 moves geometry into the tool
  precisely because spatial composition is a known weakness. It does not follow that any task
  can be tool-shaped into success.
- **This says nothing about multi-agent delegation.** The current external evidence there runs
  the other way (CooperBench: two agents ~25% vs a single agent ~50% on the same work;
  [arXiv 2601.13295](https://arxiv.org/abs/2601.13295)). Fixing tools and adding agents are not
  the same intervention, and only one of them has evidence behind it here.
- **Latency is a real cost and must be reported.** The fixed turn takes 8–55 s against ~3 s
  before. That is close to the physical floor — an achieved ~0.10 rad/s means a 270° turn needs
  ~46 s of actual spinning — but "correct and slower" is a trade to state, not to hide.

---

## 6. What this means for how we judge our own surface

[`agent-native-api.md`](agent-native-api.md) scores our harness against ROS 2
`simulation_interfaces` at **6 ahead, 5 partial, 10 missing**. That is a useful inventory and
it measures the wrong axis for this purpose: it counts **which verbs exist**, not **whether the
verbs that exist tell the truth**.

By verb count our mobile surface looked fine — it had `drive_forward`, `turn`, `set_velocity`,
`stop_robot`, `get_robot_state`. By contract it was: no completion signal, no achieved value, no
absolute-coordinate verb, a motion register that silently cancelled the previous command, and a
−43% actuator documented as −19%.

**A simulator that calls itself agent-native should be judged on its tool contracts, not its
endpoint count.** Both numbers are worth publishing; only one of them predicted the benchmark
result.

---

## 7. The checklist

Before you ship a tool an LLM will call:

- [ ] Does the call **complete** before it returns? If not, does the description say so in
      words, and is there something to wait on that cannot fire early?
- [ ] Does the result carry **achieved**, not commanded? Is unmeasured reported as `null`
      rather than as a number?
- [ ] Is there **one call** that does what the agent wants, or must it compose two? (If it must
      compose, can the two be merged?)
- [ ] Does the description state **frame, units, blocking behaviour, and the tool's own
      accuracy**?
- [ ] Does a **second command while busy** get rejected, or does it silently clobber?
- [ ] Can the agent **discover** this tool exists, and its bounds, from inside the session?
- [ ] Have you measured the tool **against ground truth** — not against its own return value?
- [ ] If the tool is wrong by a known amount, **is that amount in the description** until it
      is fixed?

The last one is the cheapest and the most often skipped. A one-line honest description is worth
more than a quarter of prompt engineering, and it costs a string.

---

## See also

- [`agents/AGENT_PATTERNS.md`](../../agents/AGENT_PATTERNS.md) — pattern 9 is the short form of this
- [`PROTOCOL.md`](../../PROTOCOL.md) — the wire contract for bridges
- [`agent-native-api.md`](agent-native-api.md) — the verb-level capability inventory (§6 above)
- [`tests/benchmarks/omnilink_tasks/`](../../tests/benchmarks/omnilink_tasks/) — where the
  task-level lift will be measured
