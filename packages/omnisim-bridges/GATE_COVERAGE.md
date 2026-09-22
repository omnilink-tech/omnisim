# Which actuation paths pass through the safety gate

Audited 2026-09-21. **Re-audited later the same day** after the command
surface was generalized across the four robot classes (mobile, arm,
quadruped, drone); rows that moved are marked. Reviewed again 2026-09-22
against the OmniLink access policy, and **re-audited 2026-09-22 (v9)** for
the per-surface rails and the collapse of the six gate wrappers into one.
**Read this before assuming a path is protected.**

`/prompt` and `/tool` are the vetted paths on all four bridges, and both
require an OmniKey. The direct REST verbs are not vetted — that is the
"NOT gated" table below, and it is the whole point of this page.

## ⚠️ Scope: which bridges this page audits, and one it does not

This page audits the **five OmniLink bridges** — `bridge_base` and the
mobile, arm, quadruped and flying controllers — plus the relay and the
agent-side surfaces in the addendum. Say the scope out loud, because there
is a bridge outside it and it is not a hypothetical one:

**`projects/samples/demos/controllers/husky_omnilink_bridge/` is a seventh
bridge and the gate does not reach it at all.** Verified 2026-09-22 by
reading the file:

- it **does not import `omnisim_bridges`** in any form (it imports
  `omnisim.Supervisor` and `_omnilink_relay.http_security` for the token
  guard). It is a standalone HTTP server with its own handler, so it
  shares no `bridge_base`, no `serve_tool`, no `vet_toolcall` and no gate;
- it serves `POST /action` — the §5.4 typed dispatch — plus `/state`,
  `/capabilities`, `/protocol`, `/lidar`, `/camera`, `/scan`, `/mission`,
  `/maze`, `/admin/reload` and `/admin/teleport_solid`. It serves **no
  `/prompt` and no `/tool`**, so it has no vetted path to fall back on;
- it is the bridge the `husky_maze` production agent drives.
  `agents/production/husky_maze/scripts/chat_drive.py` reads `toolCalls`
  from a model and posts each one to the runner's local tool server, whose
  `tools/husky.py` turns it into `bridge_post("action", …)`. So
  **model-produced frames reach motors there with nothing in between** —
  the local tool server validates the argument *schema* only, which is the
  `runner_base` row already in the "NOT gated" table.

⚠️ **`/action` is not inherently an ungated verb**, which is exactly why
this deserves a line rather than a silence: the flying bridge's `/action`
**is** gated (see the table below). The difference is per bridge, not per
verb.

**Not a call to fix it during a release cut.** It is an evaluation rig
rather than a shipped demo, and re-plumbing it now is how regressions
arrive. It is named here so nobody rediscovers it by accident, and so the
file whose job is to be the authoritative list of what the gate does not
reach actually contains it.

⚠️ **Since v9 some of this is discoverable over the wire, and it does not
replace this page.** Every bridge now publishes
`/capabilities.safety_gate` (PROTOCOL.md §5.2.1) with `gated_paths` and
`ungated_paths` filled in for its own route table, so a client can finally
tell a gated route from an ungated one without reading source. Three
things that field **cannot** tell it, and this page can:

- the arm's **learned verbs**, promoted to `POST /<verb>` routes at
  runtime, can never appear in a static `ungated_paths` list;
- the ungated surfaces that live **outside** the bridges entirely (the
  addendum at the foot of this page);
- *why* a path is ungated — `/stop_robot` by design, `/reset_to_home` by
  accident.

`gate.check(utterance, frames, authorized=(), surface=None)` is the
deterministic veto, with `gate.reject_toolcall(tool, args, utterance="",
surface=None)` for one bare call. Before this audit it was called in
exactly one place, on a branch that is **off** in any bridge with an
`OMNI_KEY` attached — so in the shipping configuration **zero** tool calls
were vetted. The `surface` argument arrived on 2026-09-21 and is what the
section below is about.

## Gated

| path | file:line | notes |
|---|---|---|
| deterministic parser | `route.py` `execute()` | pre-existing; only runs when the parser handles the turn. (Cited as `route.py:285` in the 2026-09-21 audit; line numbers drift, the symbol does not) |
| **LLM relay dispatch** | `relay.py` `_dispatch_one` | **added 2026-09-21.** Every model tool call, with the utterance, so all 8 rules apply |
| **platform `/tool` callback** | `bridge_base.py` `_dispatch_tool` | **added 2026-09-21.** No utterance, so the 4 intent rules cannot fire; magnitude, schema and deixis rails do |
| **each bridge's own `/tool`** | mobile, arm, quadruped **and flying** controllers | **added 2026-09-21; the five handlers were collapsed into one on 2026-09-22, see below.** Gating `bridge_base` did NOT cover these -- every bridge shipped its own handler, and a gated `bridge_base` still let `distance: 300` through and hung the HTTP call for 90 s while the robot drove it |
| **the flying bridge's `/prompt` and `/tool`** | `mavic_omnilink_bridge.py` | **NEW, later on 2026-09-21.** It had neither endpoint, so it was the one robot with no vetted path at all |
| **the flying bridge's `POST /action`** | `mavic_omnilink_bridge.py` `_route_post` | **NEW, later on 2026-09-21.** Its oldest surface and the one the mission runners use. Verbs are mapped onto the gate's vocabulary (`set_yaw`→`turn`, `stop`→`stop_robot`); an action with no mapping passes through, because refusing what this bridge has always accepted is a behaviour change, not a safety fix |

✅ **2026-09-22: five `/tool` handlers became one, and six fail-closed
wrappers became one.** `bridge_base.serve_tool` is now the single `/tool`
implementation — the reference handler's and the four controller bridges'
— and `bridge_base.vet_toolcall` the single fail-closed wrapper (those five
plus the model relay's `_dispatch_one`). That is not a new check; it is the removal of
the failure mode this page was written about — `b977772b0` added a gate to
the reference handler and missed all four bridges, and the Mavic's own copy
had omitted `grasp`, `joint` and `tcp`. A safety check that has to be
applied in six places is a safety check that will be applied in five.

Two consequences worth stating rather than assuming:

- **`/tool` strips the transport fields before the gate sees them** —
  `tool`, `id`, `utterance`, `request_id`, `surface`. The flying bridge's
  long-standing `id` defect (an `id` landing in the argument set and coming
  back `unknown_arg`) is closed by that alone.
- **Every bridge now honours `utterance` on `/tool`**, not just the flying
  one, so the four intent rules can fire on that path when the caller has a
  sentence to send. Without one they stay silent by design; a caller with
  an operator sentence should use `/prompt`.

**A refusal is now also an event, and it carries a machine-readable rule.**
`serve_tool` and the `/prompt` parser path file `gate.refused` on the
bridge's own event ring (PROTOCOL.md §5.9) with the tool, the reason, the
origin and a `rule`; `rule` also rides in the `400 refused_by_gate` body as
`details.rule` (§11.1), so a `/tool` caller does not have to re-parse prose
either. Before this, a refusal on `/tool` was the one thing on that path
nobody saw: the caller got a 400 and the robot's own agent learned nothing.

✅ **The flying bridge's `POST /action` carries both as of 2026-09-22.** It
still refuses before it reaches the shared handler — the verb has to be
mapped onto a gate tool first — so it now calls the same two shared
producers itself: `refusal_rule` for `details.rule` beside
`details.action` and `details.tool`, and `bridge_base._emit_refusal` for a
`gate.refused` event with `origin: "action"`. It calls THOSE rather than
spelling the fallback again, because the two producers must name
`gate_unavailable` identically or a consumer branching on `rule` cannot
tell which of them wrote the event. Until then this was the one gated path
whose refusals nobody could see and nobody could branch on.

⚠️ **`rule` is never an empty string.** Both producers parse it out of the
same `"<rule>: <detail>"` reason and both fall back to
**`gate_unavailable`** — spelled identically — when there is no parsable
rule head, which is what the fail-closed wrapper's "safety gate unavailable
(ImportError)" produces. An empty rule was the behaviour on the `/tool`
origin until it was fixed on 2026-09-22, and it was *worse* than a missing
one: it looks populated to a schema check while being useless for the
automated reaction the field exists to enable. Measured on a live Husky, a
correctly refused 300 m drive filed `rule: ""` beside
`reason: "implausible: distance=300 …"`. The fallback is a named
non-answer, never a guessed member of the enumeration — a client must never
be told `interrogative` when the truth is that nothing was checked.

⚠️ A refusal of a **model's** tool call inside a relay turn is *not*
ringed — it is already in the `actions[]` the agent is reading.

⚠️ **A tool the gate has never heard of is not merely unknown, it is
UNCHECKED** -- `check()` emits `unknown_tool` and skips every other rule
for that frame, and every call site drops `unknown_tool`. Until
2026-09-21 `SPECS` was hand-written and had drifted from the bridges:
**ten of the arm's nineteen tools were absent**, so `grasp`,
`set_tcp_target`, `set_joint_positions`, `set_gripper_width` and
`run_learned_skill` were in the "NOT gated" table without anyone having
put them there. `gate.register_tools()` now builds the table from each
bridge's own `Tool.parameters` at relay start-up. **A bridge that does
not register its tools is a bridge whose tools are not vetted.**

⚠️ **REGISTRATION HAPPENS IN THE RELAY CONSTRUCTOR** (`relay.py`), which
is the one place every bridge's real tool set is known -- and which only
runs when a relay attaches, i.e. when `OMNI_KEY` is present. The exposure
is bounded rather than absent: on the mobile, arm and quadruped bridges
`/tool` resolves names through `relay.tools`, so with no relay it returns
`503 tool_not_registered` and nothing dispatches at all. Measured
2026-09-21 by running the quadruped shift without a key: every dispatch
came back 503, 14 commands "sent", 0.47 m of sway and no commanded
motion. The flying bridge is the exception -- its `/tool` registry is
built in-process from `act_*`, so it is gated without a platform key,
which is the shape the other three should eventually take.

The gate **fails closed** on physical tools: if it cannot be imported or
raises, motion is refused. Read-only tools still answer — refusing to
report a position because a *motion* check is unavailable would be a
self-inflicted outage. ⚠️ That wrapper is the one thing the gate cannot do
for itself, and since 2026-09-22 there is exactly one copy of it
(`bridge_base.vet_toolcall`). **A bridge author copying this endpoint must
copy the wrapper with it: a safety check that vanishes with its import is
not a safety check.**

It does not police tool *existence*. `unknown_tool` is dropped because
the caller has already resolved the name against the bridge's own
registry, and overruling that breaks bridges with runtime-named tools
(the arm's learned verbs can never appear in `SPECS`).

## Which rail a shared verb gets — and the four things that are easy to get wrong here

The gate's magnitude checks are per-quantity, and one family of them is
per-**surface**: `move_body{forward, lateral, vertical}` is a 120 m climb
and a 50 m flight on a drone, and a 1.0 m body shift on anything else.
Every other rail — `distance`, `speed`/`v`/`linear`, `w`/`angular`,
`altitude`, `angle_rad` — is surface-blind.

**The rule, exactly** (`gate.check`, and it is what a port must reproduce):

1. the **caller's** `surface` always wins;
2. with none, fall back to the surface recorded on the tool's spec — but
   **only when exactly one** robot class ever registered that tool;
3. when **two or more** registered it, discard the recorded surface and
   take the **ground / body-shift** rail;
4. `takeoff` / `land` / `hover` are air **by name**.

### 1. ⚠️ "No surface means the strictest rail" is FALSE as a blanket rule

It is true of a **shared** tool and false of every other, and the blanket
version is not a conservative default — it is a live false refusal.
Measured on the shipped tree, where **only the Mavic registers
`move_body`**: `check(..., [move_body{vertical: 119}])` with **no** surface
returns `[]` — allowed, on the 120 m air rail. A port that hard-codes
"null → 1.0 m" passes the parity fixture (which registers the quadruped for
`move_body` too) and is wrong the day one process serves the tool alone.
`bridge_base.vet_toolcall`'s docstring said the wrong thing here until
2026-09-22 and it was load-bearing: a client that skipped passing `surface`
believing it was choosing the safe option was choosing a different rail
than it thought. ⚠️ **It was not one copy — it was six**, each written
independently from the one before it, and a documentation pass that read
the code rather than the handoffs is what found the last four:

| # | site | what it cost |
|---|---|---|
| 1 | `bridge_base.vet_toolcall` docstring | the original |
| 2 | `bridge_base.safety_gate_block` comment | published a body-shift rail for a bridge that cannot know which rail applies |
| 3 | `BridgeBase.surface` class comment | told every bridge author that `None` was the safe default |
| 4 | `relay.OmniLinkRelay.__init__` comment | stated it as the general rule while describing a case where the ground rail *was* what happened |
| 5 | `tests/test_gate_registry.py` module comment | taught the rule to whoever reads the test that pins it |
| 6 | the same file's assertion message | printed it at the moment a bridge author is most likely to believe it |

All six were corrected on 2026-09-22, along with two adjacent pages that
described the fallback incompletely (`docs/guide/omnilink-add-your-robot.md`
had it wrong in the other direction too: a caller surface the gate does not
know does **not** fall back to the spec's, it takes the ground rail,
because a non-empty caller surface always wins). If you are about to write
this sentence again, the four-step rule is the block immediately above.

**A bridge that knows its class must still always pass it.** The fallback
is a guess; the argument is a fact.

### 2. ⚠️ A `surface` arriving over HTTP is accepted and IGNORED

`POST /tool` accepts a top-level `surface` field, strips it from the
argument set, and **throws the value away**. The bridge's own surface is
authoritative.

- **Stripped**, because otherwise it lands in the argument set and the gate
  answers `400 unknown_arg: surface is not a parameter of drive_forward` —
  which would refuse *every* tool call against *every* shipped bridge. The
  OmniLink platform sends the field whenever the agent's profile records a
  surface, so this line is what keeps the door open; a caller that must
  also work against an older bridge probes once per endpoint and drops the
  field the `unknown_arg` names.
- **Value dropped**, which is the half that matters. A caller who could
  declare `"drone"` would buy the 120 m altitude rail for a quadruped's
  `move_body{vertical}` — the rail chosen by the party it protects against.
  That is a client-chosen safety setting, which is the order-dependent
  defect this argument was introduced to kill, wearing a different hat.

**This is a security property, not an oversight. Do not "fix" it by
threading the caller's value through.**

### 3. Registration order no longer decides a rail, and that is PROVEN

Recording one surface per spec made the rail depend on which bridge
registered first: register the drone before the quadruped and a 40 m "body
shift" was allowed; reverse the order and every real drone climb was
refused. An order-dependent safety property is worse than a surface-blind
one, because it looks correct in whichever order you happen to test.

The parity generator now regenerates every post-registration verdict under
**all 24 permutations** of (drone, quadruped, arm, mobile) and refuses to
write the fixture if any permutation disagrees, naming the rows. The guard
is not vacuous: it was run red on purpose against the pre-`a85fca20f`
behaviour, fired on 7 rows, exited 1, and left the fixture untouched.
Output is stable across `PYTHONHASHSEED`. The fixture
(`tests/benchmarks/gate_parity/cases.json`) also publishes the
registration table itself, under `registry`, so a port replays what the
gate was actually taught instead of hand-copying it — a hand-written mirror
of a generated table is the stale-fixture defect one level up.

### 4. Three honest limits on the surface work

- **`get_reach_envelope` is the only `physical: false` tool in the
  registry, and that carve-out is load-bearing.** It is what lets a
  read-only tool answer a question instead of being refused as an
  `interrogative`. Proven by running the guard red: with `physical` forced
  false across the board, `arm grasp{width: 0.05}` on *"how much reach have
  you got left?"* went `['interrogative'] → []` — a question closing a
  gripper — and the generator refused to write. A registration may **raise**
  a tool to physical and may never lower one.
- **The `takeoff` / `land` / `hover` air-by-name branch is currently
  UNREACHABLE.** It only affects the `forward` / `lateral` / `vertical`
  loop, and none of those three tools declares any of those arguments —
  not in the hand-written `SPECS`, not in the Mavic's registry
  (`takeoff{altitude}`, `land{}`, `hover{}`). It must still be ported; it
  goes live the day a bridge publishes e.g. `hover{vertical}`. **Record it
  as live-on-arrival, not as exercised** — a `takeoff{altitude: 200}` under
  `surface="quadruped"` is refused by the surface-blind `altitude` rail,
  which is a different code path.
- **`move_body` is drone-only on the shipped tree**, so the shared-tool
  fallback (rule 3 above) **has no production trigger today.** The
  quadruped bridge serves no such verb. The parity fixture registers both
  classes for it deliberately, to pin the fallback before something needs
  it — do not read the fixture as evidence that two producers exist.

`vertical_rail` in `/capabilities.safety_gate` publishes which rail this
bridge's `move_body{vertical}` takes, and is **`null` when the bridge
declares no surface** — because a surface-less bridge genuinely cannot say,
without knowing what else registered the tool. Publishing a guess there
would publish a safety property nobody checked.

## NOT gated — known, unfixed

These reach an actuator with nothing in between. Listed so nobody assumes
otherwise.

| path | where | who can drive it |
|---|---|---|
| **direct REST actuators** | `/drive_forward`, `/turn`, `/drive_to`, `/set_velocity`, `/stop_robot`, `/reset_to_home`, grippers, joints — in all four bridges and `bridge_base` | any HTTP caller with the token |
| ~~keyword ladders~~ | ⚠️ **RETIRED 2026-09-22 — row closed.** Every bridge's `IntentRouter.dispatch` and the shared `intent_router` module are deleted. No bridge has a command path underneath the parser any more, and none is coming back: an OmniKey is required for every chat turn on every plan, and `/prompt` with no relay answers `401 omnikey_required` before the sentence is interpreted at all. History, because it is the reason the row existed: this entry named all four bridges until 2026-09-21, when `884a7aaba` was found to have already gated the mobile ladder at the utterance (`_utterance_forbids_motion`) without the table saying so. The other three were ungated for their whole life. A coverage audit that is wrong in the safe direction still teaches a reader the wrong thing about which code to copy | — |
| ~~**Mavic `POST /action`**~~ | ⚠️ **GATED 2026-09-21**, see the table above. `goto_waypoint` has no mapping onto the gate's vocabulary and so still passes through unchecked | the OmniLink agent |
| `_reground_if_unread` | `relay.py` | name-constrained to read-only tools *by convention*, not by check |
| arm learned verbs | promoted to `POST /<verb>` endpoints at runtime | any HTTP caller |
| agent-side `/tool` servers | `agents/production/husky_swarm/*`, `agents/production/_lib/runner_base.py` | platform LLM |
| **`husky_omnilink_bridge`'s `POST /action`** | `projects/samples/demos/controllers/husky_omnilink_bridge/` — a standalone server that imports no part of `omnisim_bridges` | a model, through `husky_maze`'s `chat_drive.py` → local tool server → `bridge_post("action", …)`. **Outside this page's scope until now; see "Scope" above** |

Two of those deserve emphasis. `_act_goto` and `_act_move_body(lateral=)`
on the Mavic still have **no entry in `route._ADAPTERS`**, so `goto_waypoint`
remains reachable only by a path with no rail on it -- the `/action` gating
above covers the verbs that map, and that one does not map. And
`bridge_base`'s REST surface is the reference every external bridge copies.

**Since v9 each bridge publishes its own half of this table** as
`/capabilities.safety_gate.ungated_paths`, exact for its route table:

| bridge | `surface` | gated | ungated |
|---|---|---|---|
| `omnilink_mobile_bridge` | `mobile` | `/prompt`, `/tool` | `/set_velocity`, `/drive_forward`, `/turn`, `/drive_to`, `/drive_to_waypoint`, `/stop_robot`, `/reset_to_home`, `/attach_trolley`, `/grab_pallet`, `/detach_trolley`, `/release_pallet`, `/resume_autonomy`, `/resume_idle_loop`, `/intents` |
| `omnilink_arm_bridge` | `arm` | `/prompt`, `/tool` | `/set_joint_positions`, `/set_tcp_target`, `/solve_ik`, `/open_gripper`, `/close_gripper`, `/set_gripper_width`, `/grasp`, `/release`, `/stop_robot`, `/reset_to_home`, `/pick`, `/place` |
| `omnilink_quadruped_bridge` | `quadruped` | `/prompt`, `/tool` | `/stop_robot`, `/stand`, `/sit`, `/walk`, `/wave`, `/reset_to_home` |
| `mavic_omnilink_bridge` | `drone` | `/prompt`, `/tool`, `/action` | `/action` (in BOTH halves on purpose — see below) |
| `bridge_base` reference | none unless the subclass sets one | `/prompt`, `/tool` | the twelve direct verbs it serves |

⚠️ Two things that table cannot say, and this page must. The Mavic's
`/action` is **PARTIALLY vetted and sits in both halves**, with a
`safety_gate_note` naming the split: vetted `takeoff`, `land`, `hover`,
`set_yaw`, `stop`, `reset` (all mapped onto gate verbs), unvetted
`goto_waypoint`, `set_gimbal_pitch`, `complete_mission` (no mapping,
passed through unchanged) — gated is not the same as "every verb on it is
checked", and the protocol has no per-verb field to say so in. And **the
arm's learned verbs are absent from both halves**: they become routes at
runtime, so no static list can carry them, and they are ungated.

⚠️ **The Mavic's row was wrong in BOTH directions until 2026-09-22, and
this page repeated it.** `ungated_paths` read `["/reset",
"/complete_mission"]` — two `action` VERBS published as HTTP routes, when
this bridge's only POST routes are `/prompt`, `/tool` and `/action` and
`POST /reset` has always been a 404. `/reset` was described as unvetted
although `_ACTION_AS_TOOL` maps it to `reset_to_home` and puts it through
`vet_toolcall` (wrong in the conservative direction, which teaches a
reader the block cannot be trusted either way), and `goto_waypoint` and
`set_gimbal_pitch` — the two verbs that genuinely are unvetted, one of
which flies the aircraft to a coordinate — appeared in neither half. The
two lists and the note are now DERIVED from `ACTION_VERBS` and
`_ACTION_AS_TOOL` in the controller, and
[`tests/test_mavic_gate_publication.py`](tests/test_mavic_gate_publication.py)
fails the moment a verb reaches the router without reaching the
publication. ⛔ **An empty `ungated_paths` was the tempting fix and is the
worst available answer** — with every other POST route on that bridge
fully gated, "routes only" would have emptied the array, and an empty
load-bearing half reads as "everything here is vetted".

⚠️ **`present: true` with `ungated_paths` omitted must never be read as
"everything here is vetted."** That is why the field is the load-bearing
half of the block, and why publishing it was the point of shipping the
block at all.

⚠️ **`/stop_robot` is ungated ON PURPOSE** and should stay that way. It is
tier `SAFE`, the de-escalation carve-out: the cost of a spurious stop is a
pause, and the cost of a refused stop is whatever the robot was about to
hit. Those are not symmetric. `/reset_to_home` next to it is ungated by
accident, not by design -- the same verb arriving on `/tool` or `/prompt`
IS vetted.

## Why the gap survived review

The relay's dispatch site already had `with handle.execution_gate:` — a
cancellation `RLock`, unrelated to safety. Reading that line as a safety
check is the easy mistake, and it is why the hole outlived several
readings of the file.

## What the gate is not

A world model. It does not know the arena, so its magnitude checks are
**rails** ("nobody meant this"), not bounds. Nothing in the stack limits a
robot to the floor it is standing on: `act_drive_forward` and
`act_drive_to` take a distance and drive it. A two-hour soak walked a
Husky off a 12 m floor to (−6.86, 4.75) and nothing objected.

## Addendum — further ungated surfaces (full-tree scan)

- **`agents/production/_lib/runner_base.py`** `/tool` — validates the
  arg *schema* only. That is the gate's rule 6 alone, with no
  interrogative, prohibition, self-negation, magnitude, sign-conflict,
  deixis or confirm-required check. Shared base for the production
  agents.
- **`tests/benchmarks/omnilink_tasks/ol_driver.py`** — the harness runs
  its own model loop and posts to `/tool`, mirroring the relay. So that
  benchmark did not exercise the gate either.
- **Vendored OmniLink SDK examples** under
  `msys64/.../site-packages/omnilink/examples/` are live `/tool`
  handlers if anyone runs them.
- **`.local-runs/lab-offer-publish/`** is a published snapshot with the
  same holes at older line numbers — a copy, not a separate path, but it
  means the ungated `/tool` pattern has already shipped outward.

## ⚠️ What interpbench does and does not prove

`tests/benchmarks/interpbench` calls `gate.check()` **directly** to model
the combined path. Its numbers are a fair measure of the gate's
*decisions* — which frames it refuses and which it lets through — and
they are NOT an end-to-end test of the wired product. Nothing in that
benchmark posts to a bridge.

✅ **That end-to-end test now exists**: `e2e_gate.py` launches the world,
stands in for the model server so a relay attaches, posts five ungateable
frames to a live `/tool`, and judges by the robot's own pose. **8/8** --
the five refused with HTTP 400 and zero motion, and the two controls moved
0.987 m and 90 degrees, which is what stops "refuses everything" from
looking like success.

Its first run proved nothing and said so: every case returned 503,
including the controls, because `/tool` needs a relay and no key was
attached. Five cases "passed" by being refused for the wrong reason. The
controls caught it.
