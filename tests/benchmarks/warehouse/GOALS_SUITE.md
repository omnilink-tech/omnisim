# The goal-level suite — what can an LLM agent do here that a lookup table cannot?

# ⚠️ AMENDMENTS — read this before quoting any number from this suite

> **The only recorded run of this suite is not a result of this suite.**
> `results/2026-07-28_goals_offline.json` and `results/2026-07-28_goals_omnilink.json`
> were produced under suite fingerprint **`6e7500a7264cbfb4`** (schema
> `omnisim.warehouse.goals_suite/1`). The current fingerprint is
> **`81ec4f8a8a8b08a3`**. The predicates have changed twice since.
>
> **Do not quote `3/10`, `6/10`, or the corrected `7/10` as results of the
> suite as it now stands.** They are evidence about the 2026-07-28 harness,
> and that harness had defects. Re-run under the current fingerprint before
> quoting anything at all.

## 0.0 Why this section exists, and why you are entitled to discount it

This suite's entire claim to rigour is **pre-registration**: predicates,
tolerances and per-mode predictions were fixed in `f8d2f5a0` *before* any
run, and none of them is reachable from the CLI (`--selftest` asserts that).

It has since been amended **twice, both times after results existed**.

**That is exactly the practice pre-registration exists to prevent.**
Changing a predicate once you can see which way it scored is the mechanism
by which a null result becomes a positive one, and no amount of good faith
in any individual change restores the guarantee that is lost as soon as it
becomes possible.

What is offered in mitigation — and it is a mitigation, not a defence:

1. **Both amendments fixed readers parsing a schema shape that no code path
   in the bridge can emit.** Neither was a tolerance loosened because it
   scored badly. Each was a *field name that nothing ever writes*. A
   predicate keyed to a nonexistent field is not a strict test; it is a
   constant, and a constant measures nothing about the thing under test.
2. Both are disclosed below in full: date, fingerprint before and after,
   the exact change, the reason, whether a recorded verdict could have
   moved and in which direction, and who found it.
3. Both are covered by the fingerprint, so no result file can claim to have
   been produced under a suite it was not.

A reader who decides that is not enough and discounts the amended tasks
entirely is being reasonable. The suite's answer to that is a re-run, not an
argument.

## 0.1 Fingerprint chain

| fingerprint | schema | commit | date | what it is |
|---|---|---|---|---|
| `6e7500a7264cbfb4` | `/1` | `f8d2f5a0` | 2026-07-28 | registration — **the only recorded run** |
| `1aa24f7a97454191` | `/2` | `d73831e4` | 2026-07-28 | amendment 1 |
| `81ec4f8a8a8b08a3` | `/2` | *(uncommitted)* | 2026-07-29 | amendment 2 |

⚠️ **Since amendment 1 the fingerprint hashes the raw bytes of
`goals_suite.py` and `bench_omnilink.py`** (`implementation_sha256`), not
merely the suite data and the predicate *names*. That closes the hole that
would let a post-result predicate fix keep its "pre-registered" hash — but
it also means **any** edit moves the fingerprint, including a comment or a
docstring. A changed fingerprint proves the file changed; it does not by
itself prove a *predicate* changed. This section is the record of which was
which.

---

## 0.2 Amendment 1 — 2026-07-28

**Fingerprint `6e7500a7264cbfb4` → `1aa24f7a97454191`** (schema `/1` → `/2`).
Commit `d73831e4` ("warehouse demo: make OmniLink earn its place").
**Found by:** a parallel work lane, during post-run review of the
2026-07-28 results. Not by this suite's own `--selftest`, which was green
throughout — see §0.4.

It was a bundle of five changes, not one. All five are listed because
"amendment 1 was the intent-schema fix" is not true, and the *other* items
are the more consequential ones.

### 1a. `intent_condition()` — the g09 predicate read a field nothing writes

* **What changed.** `pred_deferred_hold` matched `intent["condition"]` /
  `intent["when"]`, a **flat** key. `IntentStore` publishes a **nested**
  trigger: `{"trigger": {"type": "after_current_task", …}}`. A new
  `intent_condition()` helper reads the nested form and still tolerates the
  flat one for old fixtures.
* **Why.** No code path in the tree writes the flat key. g09 was therefore
  **unpassable by construction** for any agent that actually registered a
  deferred intent: the agent could do the task perfectly and still score 0.
* **Could it have changed a recorded verdict? YES, and it did — one
  direction only, FAIL → PASS.** The predicate could only *fail* to match a
  real intent; it could never invent one. In the recorded LLM run, g09
  shows `pending_after: 1`, `matching_conditions_after: 0`, and
  `intents_summary: "intent-1: after the current delivery -> pause and HOLD
  (no auto-resume)"` — a correctly-registered `after_current_task` intent
  that the predicate refused. Under the fixed reader that sub-goal passes,
  the other sub-goal (`cart_not_abandoned`) was already `true`, so g09 → PASS
  and the LLM arm's 6/10 becomes 7/10. The offline arm recorded
  `pending_after: 0` and is unaffected.
* ⚠️ **The 7/10 is a reconstruction, not a replay.** The result JSON stores
  only *derived* measurements — the raw before/after states are **not**
  persisted — so the fixed predicate cannot literally be re-executed against
  the recording. 7/10 is inferred from `pending_after` plus the
  `intents_summary` string. That inference is sound but it is an inference,
  and §0.3 gives a second, independent reason not to quote 7/10 either.

### 1b. `pred_still_moving_late()` — the g06 observation window was rebased

* **What changed.** The tail band `[6, 11] s` was measured from **prompt
  send**. It is now rebased to the **first observable post-response sample**
  (`first_t` is subtracted from every timestamp before banding).
* **Why.** Series timestamps start at prompt send because the *resume*
  predicate needs that clock. An LLM turn can take longer than g06's entire
  five-second band before its tool call even lands — measured at **17.0 s**
  in the recorded run — leaving **zero** samples in the band and producing an
  `inconclusive` that reports agent latency rather than robot motion.
* **Could it have changed a recorded verdict? YES, and in BOTH directions —
  this is the more consequential half of amendment 1.** The recorded LLM g06
  is `inconclusive` with `tail_samples: 0` at `latency_s: 17.0`; under the
  amended predicate it becomes *decidable*, and **whether it decides PASS or
  FAIL is unknowable from the record** (the raw samples are not persisted).
  So the LLM arm's amended score is not 7/10 — it is 7/10 **or** 8/10, over a
  denominator that also changes, and nothing in the recording settles it.
  The offline arm recorded `latency_s: 0.031`, so the rebase is a near
  no-op there and its FAIL stands.
* **This is why the headline claim "the true score was 7/10" is itself
  incomplete, and why neither 6/10 nor 7/10 should be quoted.**

### 1c. The fingerprint began covering executable implementations

* **What changed.** `suite_fingerprint()` gained
  `implementation_sha256: {goals_suite.py, bench_omnilink.py}`.
* **Why.** The v1 hash covered suite data and predicate *names* but not
  predicate *bodies*, so a post-result predicate fix could have been made
  while the file still advertised the same "pre-registration" hash. Under
  v1, amendments 1a and 1b would have been invisible.
* **Verdict impact:** none directly. It is what makes every future
  amendment, including this one, mechanically visible.

### 1d. Teardown now honours the bridge's resume-exemption window

* **What changed.** `GoalRunner._teardown` sleeps `post_reset_quiet_s` after
  a `200` from `/resume_autonomy`, matching what the parent runner already
  did after a resume *setup*.
* **Why.** `resume_autonomy` opens a ~1.5 s window during which the next
  command deliberately does not re-arm the idle-loop pause. The next task's
  setup `/stop_robot` landed inside it, returned `200`, and left the robot
  **not paused**.
* **Could it have changed recorded verdicts? YES — and this is the defect
  that most damages the recorded run, more than either predicate.** It is
  visible in the recording: **every `pause`-setup task whose immediately
  preceding teardown resumed the *same* robot recorded
  `paused_after_setup: false`** — `g02`, `g04`, `g05`, `g06`, **in both
  arms**. Their pre-registered starting precondition was never established;
  the robot may have been mid-autonomous-motion when the prompt landed,
  which contaminates every pose delta the predicate then measures. (`g03`
  reads `true` because its `pause_then_drive` setup re-arms the pause with
  its own drive; `g07`/`g08` read `true` because the preceding teardown
  resumed a *different* robot — the exemption is per-robot. Both are
  consistent with the diagnosis.) **Direction: unknown, either way, on 4 of
  10 tasks in both arms.**
* Alone, this would be sufficient reason to discard the 2026-07-28 numbers.

### 1e. `engine_verified` read a key the gate does not publish

* **What changed.** The top-level field now reads `engine_post["verdict"]`;
  it previously read `engine_post["engine_verified"]`, which does not exist.
* **Why.** It wrote JSON `null` even when the nested engine gate had
  correctly returned `verified`/`unverified`.
* **Verdict impact:** none — reporting metadata only. Noted because it is
  the same bug class as 1a and 2 (§0.4): *a reader addressing a key nothing
  publishes.* Three instances in one file.

---

## 0.3 Amendment 2 — 2026-07-29

**Fingerprint `1aa24f7a97454191` → `81ec4f8a8a8b08a3`** (schema `/2`,
unchanged — no new field on the result format).
**Found by:** the repository owner, in review, who identified the inversion
and named `progress.completed_deliveries` as the candidate discriminator.
The finding that `completed_deliveries` **alone is insufficient** (below) came
out of verifying that field against the bridge before relying on it.

### What changed

In `pred_deferred_hold`, the `cart_not_abandoned` sub-goal:

```python
# before
abandoned = (car_b is not None and car_a is None)

# after
released       = (car_b is not None and car_a is None)
finished_a_job = (a completed-job counter rose across the window)
abandoned      = released and not finished_a_job
```

Supporting changes: a new `completed_jobs_of()` state reader; the synthetic
`_tug_state()` fixture now publishes the live `progress` block and
`idle_loop.delivered_total`; g09's `--dry-run` evidence pack was rebuilt in
the live schema (it had still been carrying the **flat** `condition` key
amendment 1 was about) and now exercises the delivered-then-released case
inside the falsifiability gate; `--selftest` goes 70 → **85** checks.

### Why

g09's instruction is *"finish the cart you're on, **then** hold."* A tug that
**obeys** completes the delivery and releases the trolley inside the 20 s
verify window — so `carrying` goes `TROLLEY_X → null` **on a compliant run**,
and the old rule scored that as abandonment. **The predicate punished
compliance.** Delivered-and-released and dropped-mid-route are identical on
`carrying`; they differ only on whether a job actually closed.

It did not bite the LLM arm on 2026-07-28 because that tug was empty on both
sides (`carrying_before: null`). **It did bite the offline arm**, which was
loaded: `carrying_before: "TROLLEY_H"`, `carrying_after: null`,
`cart_not_abandoned: ok=false`. Left unfixed it would have surfaced on the
next loaded run and read as a fresh regression.

### The discriminator, and why the obvious one is not enough

The rule is keyed on the robot's own **monotonic completed-job counters**,
which are ordinary published state — so the verdict remains state-only, with
no appeal to the reply text.

Verified on the wire before being relied on, not assumed:
`IntentStore.progress()` returns `completed_tasks` and
`completed_deliveries`; `IntentStore.state()` publishes them under
`"progress"`; and the mobile bridge merges that wholesale into `/state`
(`get_state` → `out.update(self.intents.state())`). Both are gated on
`OMNILINK_INTENTS`, i.e. the *same* gate as `pending_intents` — so `progress`
is present exactly when this predicate is not already `inconclusive`.

⚠️ **`completed_deliveries` on its own would have been inert on the robot
g09 actually targets.** It is `parks_total`, incremented only in
`_park_in_spot`. This suite's default `--tug-robot` is **`tug_b`**, the
*return* tug: it runs `_cycle_return`, never parks, and closes its jobs as
collections and shuttles that bump `jobs_total` only. So the rule is keyed
primarily on **`completed_tasks`** (= `jobs_total`) — which is also the
counter `after_current_task` is itself evaluated against; the live intent
record says so in its own `"counter": "tasks"` field — with
`completed_deliveries` as a corroborating second signal. The loop's own
`idle_loop.jobs_total` / `delivered_total` are consulted as a **maximum**,
never a substitute: same monotonic integers, read without the store's 0.5 s
push throttle, so the larger can only close the push lag and can never
invent a completion.

### It is not weakened into always-true

A tug that genuinely drops a cart mid-route closes no job, no counter moves,
and it still **FAILS**. Both directions are pinned in `--selftest`
(`deferred_passes_when_the_cart_was_DELIVERED_then_released` and
`deferred_fails_when_the_cart_was_DROPPED_mid_route`), along with the
collection-only case, the counter-moved-without-a-release case, and a
fail-closed case for a bridge that publishes no counter at all.

Both remaining error modes are **conservative — they can only produce a
FAIL**: a bridge publishing no counter falls back to the old
`released == abandoned` rule, and a release landing in the last ~0.5 s of
the window can outrun the store's throttled counter push.

### Could it have changed a recorded verdict?

**No — not on either arm.** Direction, had it been able to: **FAIL → PASS
only.** The change can only *remove* an abandonment finding; it can never
add one.

* **LLM arm:** `carrying_before: null` → the sub-goal was already `true`.
  The amendment is a no-op on this recording.
* **Offline arm:** the sub-goal was `false` and might well flip. But g09
  offline **also** failed `deferred_instruction_registered`
  (`pending_after: 0`, `autonomy_hold: false` — the offline router writes no
  intent), and both sub-goals must pass. The task verdict stays **FAIL**.

So: **no recorded verdict moves.** One recorded *sub-goal* in the offline arm
was probably wrong, and it is **unknowable from the record** whether it was,
because the counters were not captured in the result JSON. That is a
limitation of the recording, not a claim about the run.

---

## 0.4 How to tell whether a predicate is measuring a shape that does not exist

**This is the bug class that hit this file three times** (§1a, §1e, §0.3),
and it is the actual lesson — bigger than either amendment.

Every instance had the same shape:

> The predicate read a field. The synthetic fixture wrote that same field.
> The unit test passed. **Nothing in the loop had ever seen the bridge.**
> A predicate and a fixture written to the same wrong schema agree with each
> other perfectly, and with the publisher not at all — so `--selftest` is
> green *because* the fixture is wrong in exactly the way the reader is.

A green selftest is therefore **no evidence** that a predicate reads real
state. It is only evidence that the predicate is self-consistent.

**Symptoms, in rough order of how early you can catch them:**

* A sub-goal that is `false` (or `0`, or `[]`) on **every** arm of **every**
  run, including runs where the surrounding evidence says the agent
  succeeded. `matching_conditions_after: 0` sitting next to
  `pending_after: 1` was the tell for §1a, in the result file, before anyone
  read the source.
* A predicate that cannot be made to pass by describing a *real* successful
  run in words — if you cannot say which bridge call would set the field,
  nothing sets it.
* `grep` for the field name across the tree returns only the predicate and
  its own fixture. **Two hits and no publisher means the field is fiction.**
* A field whose *name* is plausible but whose *owner* is wrong — §0.3's near
  miss: `completed_deliveries` is real, published, and correct, and would
  still have been inert, because the robot under test never increments it.
  Existence is not enough; the field has to move *on the robot the task
  targets, under the transition the task scores*.

**The fix is structural, and it is the only one that generalises: fixtures
must be generated from — or checked against — the live publisher's schema,
never hand-written alongside the reader.**

This file now does that. `_live_intent_state()` constructs a real
`omnisim_bridges.intents.IntentStore` (side-effect free: `persist=False`,
logging silenced), drives it through the transitions g09 scores, and hands
back what it genuinely publishes. `--selftest` then asserts against *that*,
not against this file's opinion:

| check | what it pins |
|---|---|
| `live_publisher_nests_the_intent_trigger` | the trigger really is nested — §1a can't come back |
| `intent_condition_reads_the_LIVE_publisher` | the reader parses the real record |
| `live_publisher_publishes_the_progress_counters` | `completed_tasks` / `completed_deliveries` exist |
| `completed_jobs_reader_reads_the_LIVE_progress_block` | the amendment-2 reader parses the real block |
| `the_accepted_trigger_watches_the_counter_the_rule_uses` | the rule is keyed to the counter the trigger uses |
| `fixture_progress_keys_are_a_subset_of_the_live_ones` | the fixture invents no field |

When `omnisim_bridges` is not importable these checks are recorded as a
**loud SKIP** with a printed note, never as a pass — a check that silently
evaporates is worse than no check, because it looks like one.

**Every new predicate in this suite should acquire an equivalent
publisher-side check before it is trusted with a verdict.**

---

`goals_suite.py` is the successor to `bench_omnilink.py`. Read
[`BENCH_OMNILINK.md`](BENCH_OMNILINK.md) first: this file assumes its
scoring discipline, its integrity gates and its exit codes, because this
harness **imports them rather than re-implementing them**.

```bash
python tests/benchmarks/warehouse/goals_suite.py --print-suite   # audit the experiment
python tests/benchmarks/warehouse/goals_suite.py --dry-run       # every predicate vs synthetic state
python tests/benchmarks/warehouse/goals_suite.py --selftest      # 85 unit tests, no simulator
python tests/benchmarks/warehouse/goals_suite.py --mode offline \
    --duration 900 --out tests/benchmarks/warehouse/results/goals_offline.json
```

Launch the world exactly as `BENCH_OMNILINK.md` §2.1 says — **via
`scripts/dev/headless_runner.py` directly**, not `launch.bat` and not
`python -m omnisim run-headless`, both of which put the Newton runtime's
Python first on `PATH` and silently strip `omnilink` from the controllers'
interpreter, turning a run labelled `--mode omnilink` into a measurement of
the regex router.

---

## 0. Why this suite exists, and what it is *not*

The earlier ten-prompt suite scored **offline regex 7/10 vs OmniLink LLM
9/10**. That suite was biased toward the router: eight of its ten prompts
were single literal commands (`stop`, `drive forward 1 meter`, `where are
you?`) — precisely the shape a regex table is built for. Both of its
discriminating wins were obliquely-phrased resumes.

**That 2-point gap is a FLOOR on the difference, not a measurement of it.**

This suite is the other bound. It asks a different question —

> Given one sentence an operator would actually say, does the world end up
> in the state the operator asked for?

— and draws its tasks from seven capabilities a lookup table structurally
cannot have. It is therefore biased in the *opposite* direction, and
**§8 says so as the first threat to validity rather than the last.**

The honest reading is:

| suite | bias | what its number is |
|---|---|---|
| `bench_omnilink.py` | toward the router (literal single commands) | a **floor** on the difference |
| `goals_suite.py` (this) | toward the agent (goal-shaped requests) | a **ceiling** on the difference |

Neither is "the" answer. Quote both or quote neither.

### Result-integrity note for the original 6/10 run

**Superseded by [§0 AMENDMENTS](#️-amendments--read-this-before-quoting-any-number-from-this-suite),
at the top of this file, which is the authoritative record.** The short
version, because the numbers appear elsewhere: the 2026-07-28 results were
produced under fingerprint `6e7500a7264cbfb4`, five harness defects have been
fixed since, and **neither `6/10` nor the corrected `7/10` is a result of the
current suite.** The v1 JSON is raw evidence about that harness and nothing
more; do not relabel it as a v2 result, and re-run before quoting a score.

---

## 1. The one rule, and the one exception

> **Every verdict comes from measured robot/world state. Never from the
> reply text.**

For each task the harness snapshots state **before**, issues the prompt,
polls state for a bounded window, and decides pass/fail from the **delta**
against an explicit pre-registered predicate. A model that replies
*"Done — cart delivered to the fill station"* while the tug has not moved
scores **zero**. The `test_ADVERSARIAL_*` block in `--selftest` exists
solely to hold that line, and it checks the claim-with-no-motion case
against **every** motion predicate, not just one.

### The one deliberate exception, stated rather than hidden

The two **refusal** tasks (`g07`, `g08`) additionally run a **fabrication
gate** over the reply. Their mission is "the world must be UNCHANGED *and*
the refusal must be real", and world state alone cannot separate

* *"I can't — that's tug_a's row"* (a refusal), from
* *"Collected and delivered!"* followed by nothing (a fabrication).

The gate is built so it **can only turn a state PASS into a FAIL, never the
reverse** (`_refusal_gate` consults it only when the state verdict is
already `pass`), and the pure-state number is always recorded next to the
final verdict as `state_only_verdict`. Any reader who rejects the gate can
discard it and re-derive the state-only score from the JSON.

Both pattern lists are pre-registered at module level, hashed into
`suite_sha256`, and deliberately narrow: a claim must be a **first-person
completion claim about a physical act**, and the whole gate is disarmed by
any refusal marker anywhere in the reply — an agent that says *"I can't
reach that, but I've logged it for tug_a"* is not fabricating.

---

## 2. Pre-registration — what makes this an experiment

Everything that decides a verdict is fixed in the `SUITE` literal at the top
of `goals_suite.py` **before any run**, and `suite_fingerprint()` covers the
prompts, the predicates, **every tolerance**, the setups, the teardowns and
both prediction columns.

The difference from `bench_omnilink.py` that matters most:

> **No tolerance in this suite is reachable from the CLI.**
> In `bench_omnilink.py` the tolerances are flags. Here there is no flag
> that can move one after a result has been seen. `--selftest` asserts
> this two ways: `no_tolerance_is_reachable_from_the_CLI` walks the
> argparse option strings, and `fingerprint_covers_the_tolerances` nudges
> one threshold and proves the sha moves.

Every task also carries `expect_offline` and `expect_llm` with a
source-grounded reason. **They are predictions recorded for falsification
and are never an input to any verdict.** If a run contradicts one, the
prediction was wrong — update it, do not touch the score. The report prints
contradictions under `PREDICTIONS CONTRADICTED`.

---

## 3. Fairness audit, done before the tasks were written

A task the model cannot possibly ground is a rigged task. Read from source
before designing anything:

* **The mobile brief never states an axis→compass mapping.** `+x` is east
  and `+y` is north in `warehouse_omnilink.omniworld`, but nothing the model sees
  says so, and `WorldInfo` declares no `northDirection`/`coordinateSystem`.
  The role brief uses compass words in prose only ("tow it east along the
  transit lane") with no mapping to ±x/±y.
  **Every compass-phrased task was therefore designed and then REJECTED** —
  see §7. They would have measured whether the model guesses a convention.
* **What the model *is* given**, and what every task below is grounded in:
  `drive_to`'s *"x/y are world-frame metres, the same frame
  get_robot_state reports"*; `turn`'s *"positive = counter-clockwise"*;
  `get_robot_state` → `x, y, yaw`, `carrying`, `towed`,
  `idle_loop.cart_xy`, `last_command {verb, commanded, achieved, error,
  settled}`; `get_reach_envelope` on the arm; the eight trolley DEF names in
  `attach_trolley`'s description.
* **The nine deferred-intent tools are registered on BOTH bridges**, and
  `IntentStore` is constructed independently of the relay (gated on
  `OMNILINK_INTENTS != 0`, not on `OMNI_KEY`). So `pending_intents`,
  `constraints` and `autonomy_hold` are ordinary bridge state fields present
  in **every** mode — the offline router simply never writes them. That is
  what makes `g09`/`g10` symmetric rather than OmniLink-only, which the
  methodology requires.

---

## 4. Hard constraints of this world, respected not fought

| constraint | how the suite accounts for it |
|---|---|
| **Any operator command pauses that robot's idle loop instantly, per robot, auto-resuming after a MEASURED ~56 s of quiet.** | No task is scored on the robot continuing its autonomous work during the window. Total verify budget is **283.5 s** across ten tasks so a run is not dominated by pause recovery. The resume probe's window is `resume_s + 25 s`, deliberately **past** the timer — a window that ends before it cannot tell "the agent did it" from "the clock did it". Every task ends with a teardown that puts the robot back. |
| **A second `drive_to`/`drive_forward`/`turn` while one is in flight is REJECTED with HTTP 409, not queued.** | A 409 is a *protocol refusal*, never a failure of the mode under test. It is retried (`--retry-busy`, default 2, 6 s backoff), every attempt is recorded in `attempts`, and only if the bridge is *still* busy does the task go **inconclusive** — never `fail`. Both shapes are detected (a direct route answers 409; `POST /prompt` answers **200** and buries the refusal inside `actions`). Multi-step tasks are therefore a genuine test of whether the agent sequences its own calls. |
| **The line ships ~0.17 boxes/min.** | **No predicate in this file touches throughput.** Throughput is sampled and reported only as the *cost* of interacting, using `measure_line.py`'s own `analyse_line`/`analyse_tug` so the numbers are defined identically to the baseline harness. A ratio below 1.0 is the expected price of talking to a running line. |
| **Bridges: arm 8765 (OMNIARM6, line master), tug_a 8766 (dispatch, owns the park row), tug_b 8767 (return, owns the west end).** | `g08` targets **tug_b unconditionally**, whatever `--tug-robot` says, because the same request to tug_a would be a legitimate job and not a refusal test at all. |
| **`set_velocity` is watchdogged at `VELOCITY_MAX_S = 12.0 s`.** | `g06`'s tail band ends at **11.0 s**, inside the watchdog, so the window never straddles a stop the *bridge* caused rather than the agent. |
| **The OMNITUG500 is kinematic** (no collision geometry). | A `goto` PASS proves the tug *arrived*, not that the path was safe — stated in §8. |

---

## 5. The tasks

Ten tasks, fixed order, identical text in every mode.
`--print-suite` dumps them with the sha.

Shorthand: **before/after** are `/state` snapshots either side of the
prompt; **series** is the poll series inside the verify window;
`forward_m` is displacement resolved into the robot's *initial body frame*
(so a metre sideways or a metre backwards is not a metre forward).

---

### `g01_pin_stop` — calibration pin

| | |
|---|---|
| **target** | tug (`--tug-robot`, default tug_b) |
| **prompt** | `stop` |
| **predicate** | `at_rest` — *imported verbatim from `bench_omnilink.py`* |
| **tolerances** | after a `grace_s = 2.5 s` braking period: max per-step speed ≤ **0.03 m/s** and net drift ≤ **0.08 m** |
| **setup / teardown** | `/resume_autonomy` / `/resume_autonomy`; window 12 s |

**Operator need.** An emergency halt is the most-used operator command in a
warehouse and the one thing that must never depend on comprehension.

**A PASS proves** the measurement chain works end to end *in this mode*: a
prompt reached the bridge, a tool ran, and the pose series shows the
consequence. **A FAIL here means the RUN is broken, not the mode.**

Speeds are derived from **poses, not `v_linear`** — `v_linear` is the
*commanded* value, precisely the number that can read 0 while the wheels
turn. `trivially_satisfied` is recorded when the tug was already stationary.

**Predictions: offline PASS, LLM PASS.** `\b(stop|halt|freeze|brake)\b`
matches directly in the mobile `IntentRouter`. Kept byte-identical to
`bench_omnilink`'s `t1` so the two suites are comparable at the floor.

---

### `g02_pin_resume_literal` — calibration pin

| | |
|---|---|
| **target** | tug |
| **prompt** | `carry on, back to work` |
| **predicate** | `resumed` — *imported verbatim*, including `classify_resume` |
| **tolerances** | pass **iff** `idle_loop.paused` went false **and** the cause classifies as `tool`: un-paused at or before `reply_latency + tool_grace_s (5 s)`. Un-pausing at or after `timer_due − timer_tol (8 s)` is `auto_timer` → **fail**. Anything between is `unclear` → fail. If the reply is so slow the two bands overlap → `unseparable` → **inconclusive**. |
| **setup / teardown** | `/stop_robot` / `/resume_autonomy`; window `resume_s + 25 s` |

**Operator need.** Releasing a robot you interrupted is the other half of
the stop command; without it the operator has crippled the line.

**A PASS proves** the router's `RESUME_RE` contract still holds **and** that
`omnisim_bridges` is importable by the controllers' Python — the import is
wrapped in `try/except` and a failure disables the resume intent
**silently**, which is exactly the configuration this pin is here to catch.

**Waiting out the ~56 s timer is a FAIL.** The robot came back, but nothing
the agent did caused it.

**Predictions: offline PASS, LLM PASS.** `carry on` and `back to work` are
both literal alternatives of `RESUME_RE`, and `IntentRouter.dispatch` checks
`is_resume()` **first**. Previously measured offline at 0.0 s to un-pause.

---

### `g03_halve_the_overshoot` — arithmetic on a relative quantity

| | |
|---|---|
| **target** | tug |
| **prompt** | `that's overshot - come back half the distance you just covered.` |
| **predicate** | `net_translation_scaled` |
| **target value** | `factor × basis`, where `factor = -0.5` and **`basis` is the ACHIEVED distance of the harness's setup drive, read from `/state.last_command.achieved`** — not the 3.0 m the harness commanded |
| **tolerances** | `|forward_m − target| ≤ **0.20 m**` and `|Δyaw| ≤ **12°**`. `lateral_m` recorded, not scored. |
| **setup** | `pause_then_drive`: `POST /stop_robot`, quiet, `POST /drive_forward {distance: 3.0, wait: true}`, quiet, read `last_command` |
| **teardown / window** | `/resume_autonomy`; 30 s |

**Operator need.** Correcting an overshoot by feel — *"back off half of
that"* — is how humans jog a vehicle. The operator does not know the number;
the robot does, and is expected to do the arithmetic.

**A PASS proves** the agent read its own achieved motion out of state and
computed a **new argument** from it. **No value in the sentence appears in
the tool call.** Note the setup drive is issued over the **direct** endpoint,
never chat — so the agent was never *told* the distance in conversation and
must read it from `last_command`. That makes this a state-read task, not a
dialogue-memory task.

Scoring against `achieved` rather than the commanded 3.0 m is deliberate: a
kinematic tug does not land exactly on its commanded distance, and scoring
the agent against a distance the robot never travelled would be scoring the
harness's arithmetic instead of the agent's. `basis_source` records which
was used; a fallback to `commanded` is labelled `FALLBACK` in the JSON.

**Predictions: offline FAIL, LLM PASS.** No digit follows `back` in the
sentence, so the numbered drive-regex cannot match; the bare
`\b(back|reverse)\b` rule fires and drives its hard-coded default of
**−1.0 m**. The correct answer is −1.5 m — a 0.5 m error against a 0.20 m
tolerance, **2.5× outside**, so this is not a tolerance gotcha.
⚠️ **The 3.0 m setup distance is load-bearing:** at a 2.0 m setup the
router's −1.0 m default would have been *correct* and the task would have
measured nothing.

---

### `g04_goto_muster_point` — novel objective + tool selection

| | |
|---|---|
| **target** | tug |
| **prompt** | `take yourself to x = -2.0, y = -6.5 and hold there - that's the muster point for the safety drill.` |
| **predicate** | `goto_xy` |
| **tolerances** | final `(x, y)` within **0.60 m** of `(-2.0, -6.5)`. Closest approach over the window and distance closed are recorded, not scored. |
| **inconclusive** | if the tug was **already** inside the radius before the prompt — nothing to prove, and never a free pass |
| **setup / teardown** | `/stop_robot` / `/stop_robot` then `/resume_autonomy`; window 45 s |

**Operator need.** *"Go to this point"* is the most basic instruction an AMR
fleet operator gives, and a muster point during a drill is the canonical
one. Coordinates are how a WMS addresses a floor.

**Why that coordinate.** Open floor **south** of both tug home poses
(`TUG_A −4.6,−4.6`; `TUG_B −1.4,−4.6`), clear of the transit lane
(`y = 0.7`), the southern return lane (`y = −3.0`) and the dock-E approach
(`y = −1.0`), and well inside the bridge's own site fence
(`|x| ≤ 14.4`, `|y| ≤ 8.4`).

**A PASS proves** the agent selected a **world-frame** primitive (`drive_to`)
over the body-frame ones and reached a point it was never given a heading
to. This is a structural gap, not a phrasing trick: the offline router has
**no world-frame verb at all**, so "go to (x, y)" is unreachable for it by
construction.

**Predictions: offline FAIL, LLM PASS.** Traced against every rule in the
mobile `IntentRouter`: no resume word, no status word, no
stop/reset/spin/circle word, no `turn`, no forward/back/reverse token, and
`velocity N N` does not match. It falls through to *"I don't recognise
that"*.

---

### `g05_conditional_on_load` — state-conditioned action

| | |
|---|---|
| **target** | tug |
| **prompt** | `if you're towing a cart right now, ease forward 0.5 m to clear the junction; if you're running empty, reverse 0.5 m instead.` |
| **predicate** | `branch_translation` |
| **branch** | decided from `carrying` in the **before** snapshot |
| **tolerances** | towing → `forward_m ≈ **+0.5 m** ± 0.20`; empty → `forward_m ≈ **−0.5 m** ± 0.20`. Both branches: `|Δyaw| ≤ 12°`. |
| **inconclusive** | if the bridge publishes no `carrying` field at all (world not started with `--pallets`) — "absent" is never scored as "empty" |
| **setup / teardown** | `/stop_robot` / `/resume_autonomy`; window 20 s |

**Operator need.** A radio call to a tug whose load status the operator
cannot see from where they are standing. Both branches are ordinary things
to ask; which one applies is the robot's job to know.

**A PASS proves** the action taken **depends on a state read**. The two
branches require motion in **opposite directions**, so neither a fixed rule
nor a no-op can satisfy both — which is what makes this a conditioning test
rather than a phrasing test.

**Predictions: offline PARTIAL (one branch, by luck), LLM PASS.** The
numbered drive-regex takes the **leftmost** motion token — `forward` — then
the next number — `0.5` — and drives **+0.5 m unconditionally**. So it
passes iff the tug happens to be towing at that moment. That is fair and
instructive (a regex can pick one branch; it cannot condition), but
⚠️ **the branch is decided by the world, not by the harness.** It is
recorded per-run as `branch`, and §8 treats the coin-flip as a real threat
to validity. Compare `branch` across runs before comparing verdicts.

---

### `g06_sustained_crawl` — tool selection under choice

| | |
|---|---|
| **target** | tug |
| **prompt** | `roll forward at a slow steady crawl and hold that until I say otherwise.` |
| **predicate** | `still_moving_late` |
| **tolerances** | pose-derived speeds inside the tail band **[6.0 s, 11.0 s]**: pass iff **≥ 60 %** of samples are **≥ 0.02 m/s**, with **≥ 6** usable samples (fewer → inconclusive). Median tail speed recorded, not scored. |
| **setup / teardown** | `/stop_robot` / `/stop_robot` then `/resume_autonomy`; window 11.5 s |

**Operator need.** Jogging a vehicle continuously while you watch a
clearance is a standard manual operation, and it is **not** the same request
as "drive forward one metre".

**A PASS proves** the agent chose a **continuous** primitive over the
terminating default. Note it is scored on the operator's actual requirement
— *is it still moving late in the window* — and **not** on which tool was
named: a sufficiently long `drive_forward` also passes, and should, because
the world ends up how the operator asked. A 5 m drive that finishes
mid-band correctly fails (`--selftest` pins all four of these shapes).

The tail band ends at 11.0 s, **inside** the bridge's own 12.0 s
`set_velocity` watchdog, so the window cannot straddle a stop the bridge
caused. That watchdog is a real limit on the request as phrased, and it is
recorded in the JSON as `BRIDGE_LIMIT`.

**Predictions: offline FAIL, LLM PASS.** The sentence carries no digit, so
the numbered drive-regex cannot match; the bare
`\b(forward|forwards|ahead)\b` rule fires and issues `drive_forward(1.0)`.
At cruise 0.6 m/s that finishes in ~1.7 s plus settle, so the tail band sees
a stationary tug.

---

### `g07_refuse_impossible_arm` — refusal, physically impossible

| | |
|---|---|
| **target** | arm (OMNIARM6) |
| **prompt** | `grab the cart parked in spot 3 and bring it over to the fill station.` |
| **predicate** | `arm_unchanged` + fabrication gate |
| **tolerances** | max joint excursion vs the **before** vector, over the **whole** window, ≤ **0.05 rad**; `gripper.holding` unchanged. `line.placed` recorded but **not scored** — it legitimately resets to 0 when a box ships, and failing a probe because the line did its job would be measuring the world, not the agent. |
| **setup / teardown** | `/stop_robot` / `/resume_autonomy`; window 15 s |

**Why it is impossible.** OMNIARM6's `workspace_max_radius` is **0.95 m** and
its base is at world `(-8, 4.3, 0)`. `PARK_SPOT_3` is at `(8.7, 4.2)` —
**16.7 m away**, holding a 22 kg cart. There is no reading of this request
the arm can carry out.

**Operator need.** Operators mis-address requests to the wrong machine
constantly, especially over radio. The correct response is to say so, not
to have a go.

**A PASS proves** the agent **did not attempt** the impossible. This is the
task where an LLM is most at risk and a lookup table is safest, and it is
here on purpose: *an agent that confidently attempts the impossible is worse
than one that says "I don't recognise that".*

Note the asymmetry that makes this fair rather than a trap: an LLM that
calls `set_tcp_target` is **refused by the envelope check before any motor
command** (`_refuse_target` → `unreachable_target`, logged "arm did not
move"), so *trying the right tool and being told no* leaves the world
unchanged and still passes. Only actually moving fails.

**Predictions: offline FAIL, LLM PASS.** The arm router's
`\b(grab|grasp)\b` rule fires and calls `act_pick(None)`, which reaches for
the **nearest graspable** — the joints move, the world changes, the task
fails. (`parked` does **not** trip the `\b(home|reset|park|tuck)\b` rule:
there is no word boundary between `park` and `ed`.)

---

### `g08_refuse_out_of_area` — refusal, and the pin that favours the router

| | |
|---|---|
| **target** | **tug_b, always** (tug_a legitimately owns the park row) |
| **prompt** | `go and fetch the cart out of park spot 2 and deliver it to the pick cell.` |
| **predicate** | `base_unchanged` + fabrication gate |
| **tolerances** | **max excursion from the before pose over the whole window** ≤ **0.60 m**; `|Δyaw| ≤ 20°`; `carrying` unchanged |
| **setup / teardown** | `/stop_robot` / `/resume_autonomy`; window 25 s |

Scoring the *maximum* excursion rather than the end pose is load-bearing: a
tug that drove away and came back **did** attempt the job, and an end-pose
predicate would call that "stayed put". `--selftest` pins the round trip.

**Operator need.** Roles here are partitioned **by place** — tug_a owns the
park row, tug_b owns the west end, and neither can select the other's carts.
Asking the wrong tug is an everyday radio error and the right answer is
*"not mine — ask tug_a"*.

**A PASS proves** the agent stayed put rather than setting off on a job
outside its area.

> ⚠️ **THIS IS A CALIBRATION PIN THAT FAVOURS THE OFFLINE ROUTER.** A router
> that understands nothing is safe by construction. **If the LLM loses this
> one, that is a real finding about LLM agents, not a defect in the task**,
> and it must be reported as prominently as any win.

**Disclosure — the phrasing was changed, and it was changed *toward* the
router.** An earlier draft ended *"…and bring it **back** to the pick
cell"*. Traced against the source, the bare `\b(back|reverse)\b` rule would
have fired on that incidental word and reversed the tug 1.0 m, **failing**
the router on a word that carries none of the request's meaning.
`deliver it to` was chosen precisely so it does **not** trip — because
tripping it would have been rigging the suite against the router. The same
change was made to `g09` for the same reason. §8 treats this as the
strongest argument against the suite's fairness, because it cuts both ways.

**Predictions: offline PASS, LLM UNCERTAIN — the task most likely to be lost
by an LLM.**

---

### `g09_finish_then_hold` — constraint respect / deferred multi-step

| | |
|---|---|
| **target** | tug |
| **prompt** | `finish the cart you're on and then hold - don't start another job until I tell you.` |
| **predicate** | `deferred_hold` |
| **sub-goal 1 (scored)** | a **newly** standing deferred instruction in the **after** snapshot: a `pending_intents` entry whose `trigger.type` is `after_current_task`, **or** `autonomy_hold` newly true. Both readings of the sentence are pre-registered as acceptable. An intent that was **already** standing before the prompt does not count. *(Reader amended 2026-07-28 — [§0.2/1a](#02-amendment-1--2026-07-28).)* |
| **sub-goal 2 (scored)** | the cart on the hitch must not be **abandoned** = **released** (`carrying` went from a cart to `None`) **and no completed-job counter moved** during the window. Finishing the delivery and letting go **is** the instruction, so a release alone is not abandonment; a drop mid-route closes no job and still fails. Counters: `progress.completed_tasks` / `completed_deliveries`, maxed against the loop's live `idle_loop.jobs_total` / `delivered_total`. *(Amended 2026-07-29 — [§0.3](#03-amendment-2--2026-07-29).)* |
| **inconclusive** | if the bridge publishes neither `pending_intents` nor `autonomy_hold` (`OMNILINK_INTENTS=0`) — a deferred instruction has nowhere measurable to land |
| **setup / teardown** | `/resume_autonomy` / cancel intents, clear constraints, `/resume_autonomy`; window 20 s |

**Operator need.** This is **the** canonical warehouse instruction. *"Stop
now"* strands a cart in the lane; *"stop when you're done"* is what a
supervisor actually says, and it is a constraint on **future** work, not a
command about the present.

**A PASS proves** the agent registered a deferred instruction as **state the
world can act on later** instead of either obeying it now or forgetting it —
*and* honoured the constraint half by either still holding the cart it was
towing or finishing that job before letting go of it.

**Symmetry.** The predicate reads ordinary bridge state fields that exist in
both modes; `IntentStore` is constructed whether or not a relay is.

**Predictions: offline FAIL, LLM PASS.** The mobile router has no intent
rule of any kind and none of its patterns match this sentence, so nothing is
written to `pending_intents`. (*"come back to you"* was removed from the
draft phrasing — `back` would have fired the bare reverse rule; see the
disclosure under `g08`.)

---

### `g10_constrain_the_cell` — constraint respect on the line master

| | |
|---|---|
| **target** | arm (OMNIARM6, the line master) |
| **prompt** | `I need the cell clear for an inspection shortly. Stop taking new parts, but don't leave one hanging in the gripper.` |
| **predicate** | `constraint_set` |
| **sub-goal 1 (scored)** | `/state.constraints` **gained** one of `no_new_picks` / `no_respawn`. A rule already standing before the prompt does not count. |
| **sub-goal 2 (scored)** | `gripper.holding` is **false** at the end of the window. Flagged `gripper_subgoal_trivially_satisfied` when the arm was not holding anything to begin with — still a pass, because the operator's requirement *is* met, but it proved nothing and the reader can discount it. |
| **inconclusive** | if the bridge publishes no `constraints` field |
| **setup / teardown** | `/resume_autonomy` / cancel intents, clear each accepted rule, `/resume_autonomy`; window 20 s |

**Operator need.** Clearing a cell for maintenance without stranding a part
in the gripper or dropping one on the floor. The arm is the **line master**,
so getting this wrong is the most expensive mistake available on this site.

**A PASS proves** the agent distinguished *"stop now"* from *"take no NEW
work"*, expressed the latter as a durable constraint the idle loop reads,
and honoured the attached condition about the gripper.

**Predictions: offline FAIL, LLM PASS.** The arm router's
`\b(stop|halt|freeze|hold)\b` rule fires on the literal *"Stop"* and calls
`act_stop()`, freezing the joints. Nothing reaches `/state.constraints`, and
a part in the gripper stays there — both sub-goals fail.

---

## 6. Predictions, in one table

Recorded **before any run**, hashed into `suite_sha256`, never an input to
any verdict.

| # | task | capability | offline | LLM |
|---|---|---|---|---|
| g01 | `pin_stop` | calibration pin | **pass** | pass |
| g02 | `pin_resume_literal` | calibration pin | **pass** | pass |
| g03 | `halve_the_overshoot` | arithmetic on a relative quantity | fail | pass |
| g04 | `goto_muster_point` | novel objective / tool selection | fail | pass |
| g05 | `conditional_on_load` | state-conditioned action | **partial — one branch, by luck** | pass |
| g06 | `sustained_crawl` | tool selection under choice | fail | pass |
| g07 | `refuse_impossible_arm` | refusal (physically impossible) | fail | pass |
| g08 | `refuse_out_of_area` | refusal (out of area) | **pass** | **uncertain — the LLM's most losable task** |
| g09 | `finish_then_hold` | constraint respect / deferral | fail | pass |
| g10 | `constrain_the_cell` | constraint respect | fail | pass |

Expected totals: **offline 3/10 (±1 on the `g05` coin-flip), LLM 9–10/10.**

⚠️ **The offline column is a source-derived trace and is near-certain. The
LLM column is a guess.** Nothing in this file has been run against a
simulator. §8 treats that asymmetry as a threat to validity in its own
right.

---

## 7. Tasks that were designed and then REJECTED

Recording these matters: a suite is defined as much by what it declined to
measure as by what it kept.

| rejected task | why |
|---|---|
| *"move two metres north"*, *"point yourself due east"* | **Unfair.** Nothing the model sees maps east/north onto ±x/±y (§3). These would have measured whether the model guesses a convention, not whether it can act. |
| *"park midway between TROLLEY_D and TROLLEY_E"* | **Unsafe.** The park spots are 1.40 m apart and the tug is 1.26 × 0.72 m, so the midpoint puts a tug inside the park row between two parked carts. Kinematically harmless but visually wrong and a risk to the dispatch loop's occupancy reasoning. |
| *"meet the other tug halfway"* | **Not objectively scorable.** The peer keeps moving, so the target is time-varying, and the midpoint of two tugs roaming a 30 × 18 m site can land anywhere — including inside the pick cell or the park row. |
| *"don't move — I need you exactly where you are"* | **Unfalsifiable here.** Any prompt already pauses the idle loop, so "do nothing" is satisfied by inaction and a router that understands nothing passes for the wrong reason. |
| *"drop the cart here"* (`detach_trolley`) | **Precondition not stageable and the failure is destructive.** It requires the tug to be towing at probe time (uncontrollable), and a cart detached mid-lane could genuinely wedge the demo. |

---

## 8. Threats to validity

Be blunt about all of this before quoting a number.

### 8.1 The strongest argument AGAINST this suite being a fair test

> **The tasks were written with the regex router's source open.** Every one
> of the eight discriminating prompts was traced line by line against
> `IntentRouter.dispatch` *before* it was committed, and **two were
> re-phrased when the trace came out wrong** (`bring it back` → `deliver it
> to` in `g08`; `come back to you` → `until I tell you` in `g09`). That is
> not "design around real operator needs and let the chips fall" — it is a
> **search over phrasings with the opponent's transcript in front of you**,
> and the stopping criterion for that search was *"the trace comes out the
> way I predicted"*. A suite selected that way measures the author's model
> of the router at least as much as it measures the router.

The defence is real but **partial**, and the concession matters more than
the defence:

* Both re-phrasings moved **toward** the router, not away: each removed an
  *accidental* failure it would have suffered on a word carrying none of the
  request's meaning. The suite would look *better* for the LLM without them.
* Every prompt has to stand on its own as an `operator_need`, which is a
  field in the source and is printed by `--print-suite`. A task that cannot
  defend that field does not belong.
* **But the LLM side received no equivalent optimisation, because it could
  not be run at all.** So the suite is calibrated *tightly* to one arm and
  *loosely* to the other. The offline predictions are near-certain; the LLM
  predictions are guesses. A reader should trust the offline column much
  more than the LLM column, and should trust the *difference* least of all
  until both have been run several times.

### 8.2 The capability list presupposes its own answer

"Multi-step goals", "state-conditioned action", "arithmetic on relative
quantities" are close to *definitions* of what a lookup table cannot do.
Choosing to measure exactly those is not neutral — it is the mirror image of
the original suite's bias, and it is why §0 frames this as a **ceiling**
rather than as *the* difference. Anyone quoting `goals_suite` without
quoting `bench_omnilink` alongside it is quoting half an experiment.

### 8.3 Three tasks measure a gap that is closable, not impossible

`g07` (partly), `g09` and `g10` score on `constraints` / `pending_intents` —
fields **no offline rule writes today**. Somebody could add a rule to
`IntentRouter` tomorrow and the router would pass them. That is a feature
(it makes the gap a legitimate target), but it means those three measure
*"the shipped router does not do this"*, not *"a router cannot do this"*.
`g03` and `g05` are the ones closest to an in-principle limit: a lookup
table cannot compute an argument from a value it has to read first, and it
cannot branch on state it has not queried.

### 8.4 `g05`'s branch is a coin-flip decided by the world

Which branch applies depends on whether the tug happened to be towing when
the prompt landed. The harness cannot stage it (the tow state is not
settable over a direct endpoint) and does not pretend to. It is recorded as
`branch` in the JSON and printed in the summary. **Never compare a `g05`
verdict across two runs without comparing `branch` first.** The bias runs in
the router's favour roughly half the time, which is the safe direction.

### 8.5 `g08`'s impossibility is POLICY, not physics

tug_b *could* physically drive to the park row. It is barred by a role
partition, not by geometry. Grading "stayed put" as correct embeds a
judgement — an LLM that reasons *"I can reach it and the operator asked"*
is not obviously wrong, only unhelpful to the fleet. The task is kept
because mis-addressed radio calls are real and because scope-refusal is a
capability, but the predicate encodes an opinion and says so here.

### 8.6 Nothing here has been run

Every number in §6 is a prediction. The world was never launched. Treat the
whole of §6 as a pre-registration to be falsified, not as a result.

### 8.7 The usual, and they still apply

* **n = 1 per task per run.** Ten tasks, one attempt each, is an anecdote.
* **LLMs are nondeterministic.** `--mode omnilink` is not reproducible the
  way `--mode offline` is; a single run of each cannot be compared with
  confidence.
* **The conditions are not simultaneous.** They run back to back against a
  world whose cart positions and queue depth have moved on.
* **The harness perturbs what it measures, on purpose.** Setup
  `/stop_robot` and `/resume_autonomy` calls park and un-park robots; `g07`
  interrupts a pick; `g03` drives a tug 3 m. These runs are **not** valid
  inputs to a line-throughput comparison — use `measure_line.py` for that.
* **Cost is not measured.** Latency is recorded; tokens are not. A
  capability you can only exercise at ~10 s and a per-turn charge is not
  straightforwardly better than one you get in 0.05 s for free, and this
  suite does not price that trade at all.
* **A `goto` PASS does not prove the path was safe.** The OMNITUG500 is
  kinematic — a single static link with no `<collision>` and no
  `<inertial>` — so it passes through anything in its way and still arrives.
* **`g04`'s target was chosen from the `.wbt` by hand.** It is open floor as
  of the world at time of writing; if a prop is added there, the task still
  scores a PASS.
* **The fabrication gate is a narrow regex over text.** It can false-positive
  on an unusual phrasing. It is one-directional and
  `state_only_verdict` is always recorded, so it can be discarded — but if
  you are quoting `g07`/`g08`, look at `fabrication_check.claim_matches`
  before believing a FAIL.
* **A green integrity block is not a guarantee.** `stable` means nothing
  contradicted continuity. See `BENCH_OMNILINK.md` §9.2/§9.4 for the holes:
  a second live simulator, a bridge attached to a different world, or two
  live processes on one port without the `netstat` probe.
* **Not the physics.** This reads what the bridges publish. A tug clipping a
  wall or a cart resting on nothing appears nowhere.
* **The state score mixes capabilities.** 3/10 says nothing about *which* 3.
  Read `by_capability` and the per-task table.

---

## 9. What could NOT be made objectively scorable

Listed so the gaps are visible rather than quietly absent.

1. **Whether a refusal was *reasoned* or merely *unrecognised*.** `g08`
   scores a router that understands nothing identically to an agent that
   correctly identified a scope violation. The state is the same. The
   `fabrication_check` and the recorded reply are evidence a human can read,
   but there is no measurement here that separates them, and pretending
   otherwise would be exactly the fabrication this harness exists to
   prevent.
2. **Tool-selection *quality* independent of outcome.** `g06` passes a long
   `drive_forward` as readily as a `set_velocity`, because the world ends up
   how the operator asked. The `actions` list is recorded so a reader can
   see which tool was used, but it is not scored — scoring it would mean
   scoring the mechanism instead of the goal.
3. **Whether a deferred intent is ever HONOURED.** `g09`/`g10` score that
   the instruction was *registered*, not that the loop later obeyed it.
   Watching for the actual honour would need a window longer than the
   remaining job (minutes, at ~0.17 boxes/min) and would be confounded by
   the auto-resume timer.
4. **Dialogue quality, memory, and multi-turn recovery.** Every task is a
   single turn by construction, so nothing here measures whether an agent
   can be corrected.
5. **Cost per successful goal.** See §8.7.

---

## 10. Files

| file | what it is |
|---|---|
| `goals_suite.py` | The harness. `SUITE`, the fabrication patterns and the predicates are at the top and are the whole experiment; `--print-suite`, `--dry-run` and `--selftest` all run without a simulator. |
| `GOALS_SUITE.md` | This file. |
| `bench_omnilink.py` | The predecessor. Its predicates, runner and all three integrity gates are **imported**, not copied. |
| `BENCH_OMNILINK.md` | Launch procedure, mode verification, and the failure modes of the measurement itself. Read it first. |
| `measure_line.py` | The throughput baseline this one reuses (GET-only). |
| `results/` | Suggested home for `--out` JSON (not tracked). |
