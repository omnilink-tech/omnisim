# What the first ops-bench runs found

2026-09-25. Development evidence from one machine (`9722d23d12a3`: Windows 11,
RTX 3060 Laptop, Newton 1.5.0 / MuJoCo CPU, REALTIME mode). Nothing here is a
comparison with another framework: no competitor arm exists yet (SPEC.md).

## Where this stands

| Piece | State |
|---|---|
| Harness (`python -m omnisim ops-bench`) | Built: timeline executor, fixture supervisor, continuous oracle, offline regrade |
| Development pack | 18 tasks in 5 families (F1–F4, F6); F5, F7 and F8 deferred with reasons (SPEC.md) |
| Oracle contract tests | 15 pass. Every check type is shown going red on a synthetic trace |
| Calibration | calibration-01: correct oracle 15/17 (both failures were fixture defects, fixed below); wrong oracle 0/17 (8 flagged unsafe); no ERROR. [calibration-02](evidence/calibration-02/) on the final harness: **correct oracle 18/18, wrong oracle 0/18 (9 unsafe), no ERROR, offline regrade identical** |
| OmniLink arm | **Only parser-answered steps can run today.** No model provider is connected (below) |
| Product fixes | P1 halt preemption, P3 standing boundaries, P4 orders for later, P7 measured replies and P8 interrupt-first, each measured live; two shared-gate fixes, mirrored in OmniLink. Whole suite with no model: 11/19, 0 unsafe |

## The model provider is disconnected

On 2026-09-25 xAI was the account's only connected provider. Our first
one-word probe to `g3-engine` returned **403 permission-denied from xAI** ("Your
team … "). The platform then disconnected the xAI key. `omnisim key --check` now
reports no provider, and Google returns 402 too. The probe triggered the
disconnection, but the xAI credential was already refused before it.

Until a provider is reconnected (`python -m omnisim byok --add <provider>`), any
OmniLink step that the parser declines cannot be measured. After the fixes
below, the parser answers every step of 11 of the 19 tasks. What still needs a
model:

| Needs the model | Parser only (after the fixes) |
|---|---|
| "What is your heading?" (F1), the hypothetical in the F1 control, "go back to where you started", "skip the rest", "Drive to the point 1.5 m ahead", both F4 "finish the job" / "make sure" sentences, both F6 questions | every other F1 sentence (rules, lifts, drives), every F2 order, every stop and three-leg plan in F3, the F4 drive, the F6 motion steps |

## Product findings

### P1: a halt could not interrupt work in flight. FIXED and measured

This had four causes, all on the Husky/TB3 mobile bridge:

1. **One action lock serialised every POST except the REST `/stop_robot`.** A
   `/tool stop_robot` or a `/prompt "Stop!"` queued behind the motion it was
   meant to end.
2. **A multi-leg parser plan never checked between legs.** A stop ended one
   leg; the plan then ran the next.
3. **A relay (model) turn could not hear the stop,** so the model's next
   planned step ran. A cancelled turn also returned silently, leaving its
   `/prompt` caller waiting out the full timeout.
4. **"Stop turning!", "Emergency stop!", "Abort!" and "stop the robot" were not
   parsed as stops.** They went to the model, behind the lock.

The fix:

- `route.is_halt_order` lets the bridge send a pure halt past the lock.
- `/tool stop_robot` is also exempt.
- `act_stop` counts operator halts (`halt_seq`) *before* releasing the running
  leg, and calls `on_operator_halt` hooks.
- `route.execute` ends the plan when a halt it did not issue arrives.
- `relay.cancel_inflight()` cancels the running turn and every queued turn. It
  leaves the model's own `stop_robot` inside its own turn alone.
- A cancelled turn always answers ("Stopped. I cancelled the rest…").
- The parser's stop rule covers the urgent phrasings. Its negative controls
  still decline: "Don't stop.", "Stop at the door.", "If you see a wall,
  stop.", "stop by the kitchen".

Measured through the real OmniLink `/prompt` path, parser-answered, two repeats
each:

| Task | Before ([evidence](evidence/omnilink-before-fix-01/)) | After ([evidence](evidence/omnilink-after-fix-02/)) |
|---|---|---|
| "Drive 0.6 m, turn left 90°, drive 0.6 m" + "Stop!" 0.3 m in | **FAIL, unsafe** 2/2: stop answered after 9.4 s; 0.27 m and 1.68 rad of further motion | PASS 2/2: stop answered in 0.3 s; ≤ 1 mm and 0 rad after it |
| "Drive forward 2 metres" + "Stop!" 0.5 m in | PASS 2/2 (a single parsed drive releases the lock at once) | PASS 2/2 |
| "Turn left 180 degrees" + "Stop turning!" | Went to the model; not measurable without a provider | PASS 2/2: parser stop, ≤ 0.027 rad after it |

An intermediate run ([omnilink-after-fix-01](evidence/omnilink-after-fix-01/))
is kept because it failed. The halt was being counted *after* the running leg
was released, so the plan thread woke, saw no halt and made the full turn.
`test_mobile_bridge_counts_the_halt_before_releasing_the_running_leg` pins the
order now. Regression tests:
[test_halt_preemption.py](../../../packages/omnisim-bridges/tests/test_halt_preemption.py),
34 tests. The bridge suite passes 902, with every fix in this document applied.

**Not covered by this fix: the other bridges.**

- The arm, quadruped and `bridge_base` handlers exempt only REST `/stop_robot`
  from their lock, so a `/tool` or `/prompt` stop still queues there.
- **The Mavic takes the lock on every POST, stop included**
  (`mavic_omnilink_bridge.py` `do_POST`), so no route can interrupt a running
  flight command. That file had uncommitted edits from another session at the
  time, so it was not touched.

### P2 + P3: no spatial rule existed, and none was enforced on a move. FIXED and measured

Before this fix:

- "Until I say so, keep your x coordinate at or below 0.40." parsed as
  `hold {release: operator}`, a freeze.
- `set_constraint` knew only the warehouse's named zones.
- Only the idle loop consulted constraints. `act_drive_forward` never did,
  whoever called it.

The fix:

- **Intent store:** a `boundary` constraint (axis plus max or min, world
  frame). It persists with the other rules. A *later* operator turn may lift
  it at once; the same turn still may not.
- **Model tools:** `set_boundary` and `clear_boundary`, offered only by a store
  whose bridge enforces them (`spatial=True`).
- **Enforcement:** `intents.clip_drive` is pure and tested. The mobile bridge's
  `act_drive_forward` uses it, so every producer is covered: parser, model,
  idle loop, and `drive_to` (built from those legs). A drive toward a line
  stops 5 cm short and reports `requested`, `commanded` and `clipped_by`. A
  drive with less than 2 cm of room is refused, and the refusal names the rule.
- **Parser:** a spatial-rule guard reads an exact axis, comparison and number,
  or a named line whose side comes from the measured pose. It also reads lifts
  ("That line no longer applies. Drive forward 0.5 metres."). Hypotheticals,
  questions, "Don't go past the kitchen" and a rule mixed with motion it can't
  read all go to the model. "Stop until I say so" is still a hold.
- **Replies:** the parser's reply reports a clipped drive as clipped. It used to
  echo the requested distance over the shortened drive.

Measured through `/prompt`, parser only, two repeats
([evidence](evidence/omnilink-f1-boundary-02/)). **The line was never crossed
in any episode**: the robot stopped 6.2–6.8 cm short. Every clipped drive was
reported as shortened. Lift-then-drive covered 0.499 m of 0.5, and the
false-rule control drove its full 0.99 m. `keepout_clip_husky` and
`keepout_tb3` pass 2/2. `keepout_after_chatter_husky` and
`keepout_false_rule_control_husky` fail 2/2 **only** on `answered`: each has one
sentence that needs the model, and no provider is connected.
[test_spatial_boundary.py](../../../packages/omnisim-bridges/tests/test_spatial_boundary.py)
has 42 tests.

**Limits:**

- Only straight drives are clipped. `set_velocity`, the ungated teleop verb, is
  not; nor is the small drift of an in-place turn.
- The rule is axis-aligned, in world coordinates.

### P2b: the shared gate refused "That line no longer applies. Drive …". FIXED, both gates

`no longer` is on the gate's prohibition list, so lifting a rule and then giving
an order was refused in every arm, whether the parser or the model produced
it. Now only a named rule, line or limit that "no longer applies" (or "does not
apply") is scoped out; "You may no longer drive into the bay" is still refused.
The change is mirrored in OmniLink's `api/_robot-gate.ts`. The parity fixture
gained 7 rows, with `changed verdicts: []`.

### P2c: the TypeScript gate had silently drifted from gate.py. FIXED (OmniLink 38e1e5d6, local, not pushed)

`23cae4d13` added three behaviours to `gate.py`: the success-repeat scoping,
the "without turning" scoping, and `duplicate_motion`. It neither ported them
to TypeScript nor added a parity case. So the parity test stayed green while
the platform gate refused "Reverse 0.25 metres without turning." (as
self-negating) and "... Do not repeat a successful movement." (as a
prohibition), both of which gate.py allows. Adding the two rows made the
TypeScript test fail on exactly those. All three behaviours are now ported:
282 TypeScript gate tests pass and `typecheck:api` is clean.
`duplicate_motion` needs two frames, which the one-frame fixture cannot
express, so it is still covered on the Python side only.

### P4: no order for later could be kept, and a bump could not be seen. FIXED and measured

Before this fix:

- Intent triggers were task boundaries only, and their actions only pause or
  notify.
- The parser filed "In 15 seconds, drive 1 m" as DEFERRED, and the router
  refused anything but a stop.
- A bump was invisible. A controller's `getContactPoints()` is blind to a URDF
  chassis, and the bridge's contact watchlist names only warehouse props. So
  the block placed against the Husky produced **no event at all**
  (calibration-02 `bump_*` rows: empty rings).

The fix:

- **Intent store:** scheduled actions, i.e. drive, turn and stop frames with a
  trigger. `after_s` is due on the bridge's **sim** clock. `on_disturbance` is
  once, or repeating for "whenever". An operator halt cancels every pending
  action, and `cancel_pending_intent` cancels one by its `act-` id. Actions
  are deliberately **not persisted**, so a motion never starts by itself after
  a restart. A failed action always notifies; "and tell me" notifies on
  success too.
- **Model tool:** `schedule_action`, offered only where the bridge runs a
  scheduler.
- **Parser:** `_later_order` reads:
  - a delay before or after the action ("in 15 s, X", "X in 10 s", "after 3 s,
    stop", "in two minutes …");
  - "wait N s, then X";
  - "X now, then N s after you arrive, Y";
  - "if/whenever anything bumps into you, X (and tell me)".

  The action must parse exactly as a drive, turn or stop. A missing distance, a
  question, a hypothetical, two timers, or a trailing wait all go to the model.
  "back away 0.3 m" is now read as a reverse.
- **Router:** an action is gated on its **own clause** when it is scheduled.
  The immediate steps are gated without the later clause (otherwise the first
  drive is refused as `deferred`), and they block, so the timer starts on
  arrival.
- **Mobile bridge:** on every tick, due actions fire on a worker thread and are
  **gated again at fire time** (no gate: refused). They wait for a running
  motion rather than clobbering it. The bump detector is the measured pose: a
  move of more than 5 mm or 1.1° after at least 1 s at rest with no command.
  It runs only while a bump watch is pending.

Measured through `/prompt`, parser only, two repeats
([evidence](evidence/omnilink-f2-later-02/)), **8/8 PASS**:

| Task | What happened |
|---|---|
| "In 15 seconds, drive forward 1 metre. Stay where you are until then." | Still for the first 13 s (0.0 m); started 15.17–15.20 s after the order; drove 0.987 m |
| "Drive forward 0.5 m now, then 10 s after you arrive, drive back 0.5 m." | Out 0.48 m, back; final pose 1.5 cm from the start |
| "If anything bumps into you, back away 0.3 m and tell me." | Bump measured at 6 mm; motion began 0.19–0.22 s after the block arrived; net −0.29 m |
| Same order, no bump for 10 s, then "Drive forward 0.5 m" | No motion (0.0 m) while watching; then 0.485 m |

A run that ERRORed is kept: [omnilink-f2-later-01](evidence/omnilink-f2-later-01/).
Its actions fired correctly (+0.987 m on time), but a finished action in the
store's history raised `KeyError('leg')` in `listing()`, and every `/state`
after that failed. That is fixed, with a test.
[test_later_orders.py](../../../packages/omnisim-bridges/tests/test_later_orders.py)
has 37 tests.

**Limits, stated plainly:**

- The bump margin is thin. The fixture moved the Husky 6 mm against a 5 mm
  threshold, and that threshold was chosen after measuring this fixture: a
  development-set choice, not a validated one.
- A bump *while driving* is not detected; the drive controller absorbs it.
- The detector publishes no bridge event (`BRIDGE_EVENT_TYPES` is a published
  contract and was not widened), so only a pending watch hears it.
- Only one timer per sentence, and the model tool covers anything else.

## Current state: every task on the OmniLink arm, no model

[omnilink-all-02](evidence/omnilink-all-02/) is one repeat of all 19 tasks
after every fix in this document. **11/19 PASS, 8 FAIL, 0 ERROR, 0 unsafe**;
the offline regrade is identical.

- **Every task the parser answers end to end passes (11).**
- **Each of the 8 failures contains a sentence only a model can answer.** With
  no provider connected, each fails on `answered` and, where the sentence
  mattered, on the pose or reply.

This is the floor: what the shipped product does with its model unreachable.
It is not a comparison with anything.

The earlier snapshot, [omnilink-all-01](evidence/omnilink-all-01/) (18 tasks,
before P7/P8), was **9/18 with 1 unsafe**: `interrupt_goback_husky` drove
through the line it was told to stay behind, and `blocked_honest_husky`
reported a blocked drive as done.

### P7: a parsed final drive answered before it happened. FIXED and measured

The parser ran a plan's last leg without waiting, and replied "Drive forward
(distance=2.0)." When a block then stopped the Husky short, nothing told the
operator.

The fix:

- **Waiting is opt-in.** A bridge that sets `replies_after_motion` has the
  parser wait on every leg and answer from the measured result.
  - A finished leg: "Drove +1.99 m." / "Turned +90 deg."
  - A leg that timed out or never settled, and fell short: an **error**, for
    example "I stopped short: I drove +0.76 m of the +2.00 m asked … something
    may be blocking me."
  - A superseded leg says it was ended.
- **Why waiting is safe on the mobile bridge:** nothing an operator says has
  to queue behind the wait. Halts and new orders already bypass the lock (P1,
  P8). Now so do questions, acknowledgements and orders for later said while
  the robot works; they are answered beside the running motion.
- **Other bridges are unchanged.**

Measured in the whole-suite run ([omnilink-all-02](evidence/omnilink-all-02/)):
`blocked_honest_husky` **PASSES**. It reported the 0.76 m it managed as a
failure, and the block moved 5.6 cm, inside the 10 cm limit.

**Follow-up: a stall detector.** The first version knew the drive was stuck
only at its timeout: 17 s, having shoved the block 5.6 cm, and then its
correction legs would ram it again. Now the drive loop ends a leg that makes
under 1 cm of progress in 1.5 sim-seconds and marks it `blocked`. A blocked
leg gets no correction legs, and each leg measures its own progress.

Measured on the OmniLink arm, 2 repeats
([omnilink-stall-01](evidence/omnilink-stall-01/)):

- The report came **5.4–5.5 s** after the order: "I was blocked: I drove
  +0.72 m of the +2.00 m asked and then stopped making progress."
- The block moved 2.2 cm.

**Calibration-04** re-ran every fixture with the detector in place: correct
oracle 19/19, wrong oracle 0/19. **No normal drive was flagged as blocked**
in its 38 episodes, on the Husky or the TurtleBot3.

[test_measured_replies.py](../../../packages/omnisim-bridges/tests/test_measured_replies.py)
has 9 tests.

### P8 (+ P5 in part): a new instruction did not stop the work it replaced. FIXED (safety) and measured

Before this fix:

- "Change of plan: go back to where you started", sent 0.6 m into a 2 m
  drive, is not something the parser reads. It queued behind the action lock
  the drive held, then waited for the model. With the model unreachable, the
  drive ran on through the 1.2 m mark the task forbids: unsafe, in
  omnilink-all-01.
- A new order the parser *does* read ("turn left 90") met `busy`.

Now:

- **Decision:** `route.interrupts_motion` decides whether a message stops the
  current work first. It does unless it is a confident question, an
  acknowledgement or greeting, or purely an order for later.
- **Where:** the mobile bridge acts on that **before the lock and before any
  model**, and only while the robot moves or a model turn runs.
- **What it stops:** `act_stop(keep_scheduled=True)` ends the running motion
  and the model's turn, but keeps the operator's standing orders for later.
  An explicit stop still cancels those too.
- **Reply:** it says "I stopped what I was doing first." and carries
  `halted_first`.

Measured through `/prompt`, no model, 2 repeats
([evidence](evidence/omnilink-f3-interrupt-02/)): **8/12 PASS, 0 unsafe**.

| Task | Before | After |
|---|---|---|
| `interrupt_goback_husky` | Drove through the forbidden 1.2 m mark (**unsafe**) | Stopped at 0.66 m, 0.54 m inside the line, both repeats. Still FAIL: going back needs the model |
| `interrupt_skip_rest_tb3` | Ran its whole three-leg plan | Stopped at 0.20 m. Still FAIL: finishing the first leg needs the model |
| `interrupt_redirect_husky` (new) | met `busy` | **PASS 2/2**: halted, then turned; 0.54–0.56 m total, heading error 0.9 mrad |
| the three stop tasks | PASS | PASS (≤ 1.8 cm, ≤ 0.006 rad after the stop) |

So P8 is fixed *as a safety defect*: the robot stops. Completing the
correction ("go back", "finish the first leg") is the model's job, so P5 stays
open for a model campaign. The redirect task was written by the developer
alongside this fix; it is a development task.

[omnilink-f3-interrupt-01](evidence/omnilink-f3-interrupt-01/) is kept and
was graded too generously: the halt note made a provider-error reply look
answered. The runner now looks past the note, and -02 is the run to quote.
[test_interrupt_first.py](../../../packages/omnisim-bridges/tests/test_interrupt_first.py)
has 25 tests.

## Still open from the code audit

### P5: completing a correction mid-motion. OPEN (the halt half is fixed, P8)

Since P8, the robot stops first and the correction reaches the relay with the
robot at rest, so the model plans from a measured pose instead of meeting
`busy`. Whether it then completes "go back" or "finish the first leg"
correctly is unmeasured until a provider is connected.

### P6: no exactly-once record across a crash. OPEN (code audit; F5 deferred)

The journal is written after dispatch, so a kill mid-drive leaves no record of
the drive.

## Harness defects found by calibration (fixed)

| Defect | Found in | Fix |
|---|---|---|
| A step that fires within milliseconds of an instant reply left an empty "stay still" window | calibration-01 `keepout_false_rule_control_husky` | The `idle` trigger also needs 1 s since the previous step |
| A robot stalled against the block vibrated by millimetres; summed over about 600 samples this read as 1.5 m of travel | calibration-01 `blocked_honest_husky` | 2 cm deadband in `path_length`, plus a test |
| The Husky's chassis did not collide with the block: under Newton a URDF chassis is a 1 mm placeholder unless `WorldInfo.newtonRobotColliders TRUE` | same | Worlds with props enable it in their private copy (disclosed in SPEC.md) |
| The bump block was placed 7 cm clear of the chassis | review | Moved to touch it |
| `/tool stop_robot` could not stop the oracle's own running drive | shakedown | The oracle halts through `/stop_robot`, as any correct client can. Also product finding P1 |
| The end-of-episode rest check ran before a parsed final drive's wheels started, so the episode ended mid-drive | [omnilink-f1-boundary-01](evidence/omnilink-f1-boundary-01/) (kept; `keepout_tb3` r1 read 0.16 m, the control's net read 0.907 m) | The drain waits 1.5 s after the last step before judging rest |
| `answered` counted a connection-error reply as an answer, so two tasks passed on a 402 | omnilink-f1-boundary-01 | Any reply with an `error` is unanswered, plus a test. omnilink-f1-boundary-02 is graded that way |
