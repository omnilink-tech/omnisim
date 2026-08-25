# The OmniLink Warehouse — operator guide

`warehouse_omnilink.omniworld` is a production line that keeps running whether or
not anybody is watching it, and stops to talk to you when somebody is. Three
robots staff a 30 × 18 m site: an **OmniArm 6** with a suction gripper at
the pick cell, and two **OmniTug 500** tugs (`tug_a`, `tug_b`) that move
carts. Each robot runs its **own** OmniLink chat bridge on its own port, so
you talk to one machine at a time; a site-level **Warehouse-Foreman** agent
sits above all three and answers questions about the line as a whole.

This guide is the operator's view: launch it, talk to it, understand what
you are looking at, and know what it does *not* do yet. The pitch lives in
[DEMOS.md](../../../../../DEMOS.md); the world's own header comment is the
design record and carries the geometry contracts.

> **Which version is this?** This describes the demo at the current `main`.
> The throughput work that older copies of this file flagged as WIP
> (`9b0588d6`) is no longer in that state: it was **reverted whole** at
> `8977c5e0`, then **redone and gate-verified** at `347ab0c6` (2026-07-20),
> so it is in the demo you are running — see
> [Known gaps](#known-gaps-read-this-before-you-judge-the-demo) for the
> numbers and for what the gate did *not* cover.
>
> ⚠️ **If your copy of this file says `0e32f24d` is the last verified state
> and offers a `git checkout 0e32f24d -- <the three demo paths>` recovery
> command, do not run it.** It was written before the revert-and-redo above.
> Today it would discard ~23 commits — the gate-verified throughput fix, and
> all the conversational-control, presence and navigation work since.

## The line, in five stages

| # | Stage | Who does it | What you see |
|---|---|---|---|
| 1 | **CONVEYOR** | the line master (OmniArm 6's bridge) | Three open-top totes (`BOX_1..3`) ride the raised inbound belt in through dock 1 and queue. The frontmost stops on the painted `FILL_STOP` line beside the arm. |
| 2 | **FILL** | OmniArm 6 (`--idle-loop pick`) | The arm picks 5 cm cubes off the `FEEDER` tray with the suction pad and drops them into the box at the fill stop until it hits its target count. The tray holds **eight** (`GRASP_PART_A..H`, four columns of two) — one per part a full three-box lap consumes, since parts only come back when a load ships. |
| 3 | **LOAD** | the line master | The filled box (parts included — one rigid ensemble) turns off the main belt onto the `OUTFEED_SPUR`, rides it south at belt height, and settles onto the deck of the empty cart standing at the fill station. |
| 4 | **DISPATCH** | `tug_a` (`--idle-loop dispatch`) | The tug docks the loaded cart's hitch, tows it east along the transit lane, reads every cart's live pose to pick a **free** spot in the six-spot park row (`PARK_SPOT_1..6`), pushes in from the south and detaches. ~12 s later the load "ships": the box recycles to the belt entry, its parts respawn on the feeder, and the cart is now a parked empty. When the row is **full**, the next job is a **collection** instead: dock the oldest parked cart, tow it to `CART_OUTBOUND_INFEED`, and the `OUTBOUND_CONVEYOR` carries it out through dock E. |
| 5 | **RETURN** | `tug_b` (`--idle-loop trolley_return`) | Empties come back in through dock 2 on the `CART_LANE_CONVEYOR`, `tug_b` docks one at `CART_LANE_PICKUP` and stages it at `CART_STAGE` → `CONVEYOR_STATION`; the in-floor `FILL_CONVEYOR` then runs the cart the last stretch north to the fill spot in front of the arm. |

Eight carts (`TROLLEY_PAYLOAD`, `TROLLEY_B..H`) circulate a **closed,
conserved ring** — lane → stage → station → fill → park → collect → dock E →
dock 2 → lane. Nothing is spawned and nothing is consumed, so the line
cannot run dry and the park row cannot deadlock.

## Launch it

```bat
launch.bat projects\samples\demos\worlds\flagship\warehouse_omnilink.omniworld
```

The autonomous loop needs **no** LLM, no key and no network — it starts on
its own a few seconds after the three bridge controllers boot. Chat is the
optional layer on top.

## The agent-native proof: recover a hidden crew hold

The line automation is intentionally unable to clear a durable human hold.
Doing that on its own would violate the operator's instruction. The
Warehouse-Foreman is valuable at exactly this boundary: cross-robot exception
handling where the correct action depends on live state and fresh human
authority.

The causal demo hides an accidental hold on one of the three robots (the
target is selected outside the model's prompt), confirms automation preserves
it, and then gives the Foreman one sentence. A pass requires measured evidence
that it:

1. read all three robots before acting;
2. found the held robot rather than being told its name;
3. commanded exactly that robot through its own OmniLink agent;
4. read back that the hold cleared; and
5. left the other two robots untouched.

```bash
python tests/benchmarks/warehouse/foreman_recovery.py --selftest
python tests/benchmarks/warehouse/foreman_recovery.py \
  --output tests/benchmarks/warehouse/results/foreman_recovery.json
```

This is the cleanest answer to "why not just run the demo automatically?":
automation owns normal production; OmniLink owns a novel, language-mediated
exception and produces an auditable delegation trace.

Startup mode comes from your saved preferences, not from `launch.bat` (which
passes no `--mode`). If the world opens **paused**, nothing steps and nothing
moves: press play, or pass the flag explicitly.

```bat
launch.bat projects\samples\demos\worlds\flagship\warehouse_omnilink.omniworld --mode=realtime
```

Load check without the GUI:

```bash
python scripts/dev/omnisim_dev.py run-headless \
  projects/samples/demos/worlds/flagship/warehouse_omnilink.omniworld --duration 30
```

Physics is Newton with the **CPU MuJoCo** solver, pinned in `WorldInfo`
(`newtonSolver "mujoco"`). Measured ~0.95–1.0× realtime on an RTX 3060
laptop with all three loops running; the GPU `mujoco_warp` build measured
0.3–0.4× here because the loops teleport bodies every tick and each teleport
costs a GPU re-sync. Don't "upgrade" this world to warp.

## Pick a chat mode

The bridges decide at controller start, in this order:

| Mode | How you get it | What it unlocks |
|---|---|---|
| **Offline (regex)** | the default — nothing set, no Ollama running | Literal single commands only, matched by regex: `stop`, `carry on`, `home`, `forward 1 m`, `turn left 90 degrees`, `wave hello`, `open the gripper`, `where are you`, plus status ("what are you doing?"). No tool *choice*, no reasoning, no memory — a phrasing off the list is simply not understood. Zero setup, zero cost. |
| **Local Ollama** | leave `OMNI_KEY` unset and have an Ollama server answering on `http://127.0.0.1:11434` | Real native tool calling: the model picks the tool, does arithmetic on relative moves, reads state before answering, and can call `resume_autonomy`. Zero account, zero cost, runs on your GPU. Setup + model picks: [`chat/LOCAL_OLLAMA.md`](../chat/LOCAL_OLLAMA.md). |
| **OmniLink platform** | `set OMNI_KEY=olink_...` | Everything local mode does, plus cross-session memory, the robot appearing as an agent at omnilink-agents.com with its tool surface, usage telemetry, voice — **and the Warehouse-Foreman**, which is platform-side delegation and exists only in this mode. |

`OMNI_KEY` **and** a live Ollama gives you *hybrid*: inference stays local
and free, OmniLink adds memory/profile/telemetry/fallback around it. The
bridge logs which one it chose (`HYBRID relay ON`, `local Ollama relay ON`,
`OmniLink relay ON`) and the chat panel labels it.

## Talking to the robots

Right-click a robot → *Show Robot Window*. Each robot opens its own chat
panel (`window "omnilink_chat"`). One robot, one panel, one port.

| Robot | DEF | Bridge | Port | Agent name |
|---|---|---|---|---|
| OmniArm 6 pick arm (suction) | `OMNIARM6` | `omnilink_arm_bridge` | `8765` | `OmniSim-omniarm6` |
| dispatch tug | `TUG_A` | `omnilink_mobile_bridge` | `8766` | `OmniSim-tug_a` |
| return tug | `TUG_B` | `omnilink_mobile_bridge` | `8767` | `OmniSim-tug_b` |

### OmniArm 6 — the pick cell, and the line master

Its `/state` carries a `line` block (`fill_box`, `fill_state`, `placed`,
`target`, `loaded`, `queued`, `in_transit`) that is the **authoritative**
record of the line. Both tugs follow it, and the Foreman asks the arm — not
the tugs — about boxes and fill counts.

| Say | Tool it triggers |
|---|---|
| "how is the line doing?" / "what's in the box right now?" | `get_robot_state` (read the `line` block) |
| "go home" / "park the arm" | `reset_to_home` |
| "stop" | `stop_robot` |
| "wave hello" | `wave` |
| "move the end effector to 0.4, 0.2, 0.5" | `set_tcp_target` |
| "move joint 3 to -0.9" | `set_joint_positions` |
| "let go of that" | `release` |
| "carry on" / "back to work" *(all modes, incl. offline)* | `resume_autonomy` |

Also registered: `open_gripper`, `close_gripper`, `grasp`, `pick`, `place`,
`learn_skill`, `run_learned_skill`.

### tug_a / tug_b — the tugs

Both expose the same tool set. `attach_trolley` / `detach_trolley` are
registered only because the world opted in with `--pallets`.

| Say | Tool it triggers |
|---|---|
| "what are you doing?" / "what are you carrying?" | `get_robot_state` (`idle_loop`: `mode`, `leg`, `paused`, `cycles`, `conveying`, `parked`) |
| "forward 2 m" / "back 50 cm" | `drive_forward` |
| "turn left 90 degrees" / "turn around" | `turn` |
| "spin" / "drive in a circle" | `set_velocity` |
| "stop" | `stop_robot` |
| "go back to where you started" | `reset_to_home` |
| "dock to TROLLEY_C" *(LLM modes)* | `attach_trolley` — only succeeds with the tug's **rear** inside the dock radius of the hitch bar |
| "drop the cart here" *(LLM modes)* | `detach_trolley` |
| "back to work" / "carry on" *(all modes, incl. offline)* | `resume_autonomy` |

Ask `tug_a` about the park row and `tug_b` about the cart lane and fill
conveyor — the roles are partitioned **by place**, and each tug is briefed
on its own half. Neither can select the other's carts.

### Driving them without chat

Every bridge is a plain loopback HTTP server, so scripts can do what the
chat panel does:

```bash
curl -s http://127.0.0.1:8765/state          # arm + line block
curl -s http://127.0.0.1:8766/state          # tug_a pose + idle_loop
curl -s -X POST http://127.0.0.1:8767/prompt -d '{"text":"forward 1 m"}'
curl -s -X POST http://127.0.0.1:8766/resume_autonomy -d '{}'
```

The bridges bind loopback only; a non-loopback bind is refused unless you
set `OMNISIM_BRIDGE_TOKEN`.

## Pause and resume

The interruption contract is the same on all three robots and is what makes
the demo feel alive rather than scripted:

- **Any** operator command — a chat prompt or a direct bridge tool call —
  pauses that robot's idle loop **instantly**. Your motion simply replaces
  the loop's.
- Pausing is **per robot**. Chatting to `tug_a` does not stop the arm or
  `tug_b`; they keep working around it, which is the interesting part.
- The loop resumes by itself after a quiet window (`--idle-resume-s`).
  **This world sets `12` explicitly on all three robots**, because the
  bridge default of `60.0` was a latent demo-killer: the arm re-checks the
  pause *inside* its motion wait, so a single chat turn can leave it frozen
  mid-transfer with a cube welded to the suction pad for the whole window —
  landing the worst frame of the demo directly after the best one.
  Expect **a little under 12 s**, not exactly 12: the window is measured
  from the last operator command and the loop only re-tests it on its own
  poll, which at the 60 s default measured **56.0 s and 55.9 s** (~93% of
  nominal) in two runs. The same ~7% shortfall on a 12 s window is ~11 s.
  That ratio has **not** been re-measured at 12 s — treat "about ten
  seconds" as the honest expectation until it has been.
- To resume **immediately**, something has to call `resume_autonomy`. In the
  LLM modes that is the model's tool call — say "carry on" / "back to work".
  Replying "resuming now" *without* the tool call leaves the robot parked,
  because the turn itself is what holds the pause. Offline, a phrase on the
  router's resume list does it directly (see
  [Known gaps](#known-gaps-read-this-before-you-judge-the-demo)); a phrase
  off that list does not, and you wait out the window.
- On resume the loop **re-reads the world** instead of trusting saved
  state: if it is still towing something it finishes that job, otherwise it
  picks up from wherever things actually are. You can move a tug across the
  site mid-cycle and it will cope.

`paused` is visible in `get_robot_state` → `idle_loop.paused`, so the
Foreman can tell "somebody is talking to it" apart from "it is stuck".

## The Foreman — site-level questions

`warehouse_foreman.py` is **not** an OmniSim controller. It is a standalone
local tool-loop that pushes an OmniLink agent profile named
`Warehouse-Foreman`, reads `OmniSim-omniarm6`, `OmniSim-tug_a` and
`OmniSim-tug_b` through their loopback bridges, and lets the cloud model
synthesize or issue a command through a robot's own agent. The orchestration
is local because the platform-side `delegate_to_agent` path cannot reach
loopback robot tools, executes only the first delegation in a turn, and has no
synthesis turn; the script's module docstring records the verified failure in
detail. This demo therefore proves a **local crew loop using OmniLink as its
reasoner**, not platform-side robot delegation.

```bat
set OMNI_KEY=olink_...
python projects\samples\demos\controllers\_omnilink_relay\warehouse_foreman.py
python projects\samples\demos\controllers\_omnilink_relay\warehouse_foreman.py ask "why is the line slow?"
```

Use it for questions no single robot can answer — pipeline diagnosis. Its
brief tells it to look for the **starved** or **blocked** stage, because the
symptom always shows up one stage away from the cause: arm idle with no box
→ conveyor stage; arm holding a part with nowhere to put it → no cart at the
fill station, a `tug_b` question; box filled but not departing → `tug_a`.
For a question about one machine ("what's your pose?"), talk to that machine
directly — the Foreman is a round trip you don't need.

## What is autonomous, and what is commanded

Useful when you can't tell whether the world is running itself or reacting
to you. The invariant the world is built around is **nothing moves without a
visible cause** — a cart is only ever *towed*, *on a conveyor*, *on a deck*,
or *parked*.

| Moving part | Driven by | Autonomous? |
|---|---|---|
| The three totes on the raised belt | the arm bridge's line master, kinematically | yes, always |
| Feeder parts | the arm's suction pad; respawn on ship | yes |
| `FILL_CONVEYOR`, `CART_LANE_CONVEYOR`, `OUTBOUND_CONVEYOR` and their pusher dogs (`*_DOG`) | the mobile bridges | yes — the dogs are real bodies driven alongside the cart, then recirculated |
| The eight carts | towed by a tug, or run by a conveyor | yes |
| OmniArm 6 joints | its own pick loop — **until you type**, then you | pausable |
| `tug_a` / `tug_b` wheels | their own idle loops — **until you type**, then you | pausable |
| Dock doors, floor paint, fences, park-spot markings, zone outlines | nothing; visual-only props with no `boundingObject` | n/a |

The load-bearing DEFs — `FILL_STOP`, `CONVEYOR_STATION`, `CART_STAGE`,
`CART_LANE_PICKUP`, `CART_INBOUND_ENTRY`, `CART_OUTBOUND_INFEED`,
`CART_OUTBOUND_EXIT`, `PARK_SPOT_1..6`, `OUTFEED_SPUR` — are read **live**
from the world by the bridges. They look like paint but they are
configuration: move one and the choreography moves with it.

## Environment knobs

| Var | Default | Meaning |
|---|---|---|
| `OMNI_KEY` | unset | OmniLink platform key. Set → platform routing + Foreman. |
| `OMNISIM_OLLAMA` | `1` | Set `0` to force the offline regex router even with Ollama up. |
| `OLLAMA_MODEL` / `OLLAMA_BASE_URL` | `qwen2.5:3b` / `http://127.0.0.1:11434` | Local-LLM selection — see [`chat/LOCAL_OLLAMA.md`](../chat/LOCAL_OLLAMA.md). |
| `OMNILINK_ENGINE` | unset | Explicitly choose a cloud engine; overrides hybrid mode. |
| `OMNILINK_IDLE_LOG` | unset | Path to append every `[line]` and `[idle-*]` loop event to. **The way to see loop events from the GUI binary.** |
| `OMNILINK_LINE_HEARTBEAT` | unset | Set to make the line master log a heartbeat every 10 s. |
| `OMNISIM_BRIDGE_TOKEN` | unset | Bearer token; required for a non-loopback bridge bind. |
| `OMNILINK_AGENT_TAG` | unset | **Set this for ANY run that is not the shipped demo** (tests, parallel worlds, scratch ports). See below. |
| `OMNILINK_INTENT_PERSIST` / `OMNILINK_JOURNAL_PERSIST` | `1` | Set `0` to stop deferred intents / the action journal surviving a reload. |
| `OMNILINK_INTENT_STATE_DIR` | temp dir | Where those two files live. |

### Always tag a non-demo run

The agent name (`OmniSim-<robot>`) is the key for **both** the platform
profile — which carries `toolCallbackUrl` — and the conversation memory. A
test run that reuses a production robot id therefore does not sit beside the
real demo, it *takes it over*: it repoints the live profile at ports that die
with it, and its prompts land in the real agent's durable history.

Both happened, on the same day. Two verification runs on scratch ports left
the three warehouse profiles pointing at dead sockets, and adversarial probes
("you never stopped, you're making that up") ended up in the shipped agents'
memory — including the model's own capitulation and two fabrications, which
are then replayed as context on every later turn. A demo opened fresh was
being primed by a transcript in which the robot lies and folds under pressure.

So:

```bash
OMNILINK_AGENT_TAG=my-test   # -> OmniSim-tug_a~my-test, its own profile + memory
```

Leave it unset only for the real demo.

Loop events carry both sim time and wall time (`t=… w=…`), so the realtime
factor of any single leg — a tow, a pick, a conveyor run — can be derived
offline from one run instead of by a polling loop that perturbs what it
measures.

## Troubleshooting

- **Nothing moves at all.** The sim is probably paused — startup mode comes
  from saved preferences. Press play or relaunch with `--mode=realtime`.
- **The world opened but the robots never tick.** This tree has a known
  per-launch flake where controllers come up at **zero ticks**. Relaunch
  once; it usually clears. If it is *every* launch, it is not the flake —
  run `python -m omnisim doctor`, which gates the engine↔libController ABI
  mismatch that silently hangs every controller.
- **I can't see the loop's log lines.** The windowed simulator binary has
  **no capturable stdout**, so `[line]` / `[idle-dispatch]` / `[idle-trolley_return]`
  events never reach your console. Set `OMNILINK_IDLE_LOG=<path>` before
  launching and tail that file instead.
- **The chat panel says `local intent (regex)` and I wanted the LLM.** The
  Ollama probe runs once at controller start. Check
  `curl http://127.0.0.1:11434/api/version`, then reload the world.
- **A robot won't go back to work.** Offline mode *does* have a resume
  intent, but it is a fixed phrase list — try "carry on", "back to work",
  "keep going", "unpause". A phrasing off that list is not recognised and
  the robot waits out the quiet window instead (measured ~56 s), so either
  rephrase or
  `curl -X POST http://127.0.0.1:<port>/resume_autonomy -d '{}'`. If *no*
  phrase works, the `omnisim_bridges` package is probably not importable by
  the controller's Python, which disables the intent silently.
- **`attach_trolley` keeps failing.** The magnet only closes when the tug's
  **rear** is inside the dock radius of the hitch bar. Drive close, then
  turn so the tail faces the hitch, then attach.
- **A load check "passed" in seconds.** A failed load can still exit 0 —
  check the log for stepping physics and three bridge controllers starting,
  not the exit code.

## Known gaps (read this before you judge the demo)

Honest list. None of these are hidden by the demo; all of them are visible
if you watch it for a few minutes.

- **The arm still stands idle a lot — but it is no longer waiting on a
  cart.** It used to: median wait **63.2 s**, max 180.9 s, because `tug_a`'s
  takt was 191 s/box against a 26–40 s fill. That half is fixed and
  gate-verified at `347ab0c6` — extending the fill conveyor south to
  `CART_PICKUP` so a loaded cart clears the station instead of holding it,
  plus collecting on the same trip, took takt to 139 s and the arm's wait
  median **63.2 s → 10.0 s** (max → 30.2 s, the cart component of it
  53.8 → 0.6 s) over a 25 min run with **29 picks and 0 no-grip**. The
  residual ~10 s is not waiting — the cart is already there — it is the
  glide plus belt advance. So the binding constraint moved: the arm is now
  bound by the **three recycling boxes**, not by cart logistics. It is still
  visibly idle, and that is honest to watch for:
  `tests/benchmarks/warehouse/measure_line.py` measured **0.172 boxes/min
  with the arm not working 69%** of a 699.5 s run (RTX 3060 laptop, machine
  `9722d23d12a3`, realtime 0.974×).
  **One cause of that idle time has since been removed on the world side:
  the feeder ran out of stock.** A 906-sample / 3-dispatch-cycle trace put
  the TCP motionless **482 s of 605 s, in blocks of 58–114 s**, while the
  pick cycle itself ran healthy (11.3 s wall against 10.8 s commanded) — so
  the arm was not slow, it was empty. `FILL_TARGETS = (3, 2)` cycled over
  three boxes commits 3+2+3 = **8 parts per lap** against a tray stocked
  with **6**, and parts only respawn when a load ships, which chains the
  cell's takt to `tug_a`'s ~145 s round trip. The tray now carries **8**
  (a fourth column, `GRASP_PART_G/H`, on a tabletop widened 0.52 → 0.68 m
  in x). ⚠️ **This is an arithmetic fix, not a measured one** — the
  `measure_line.py` numbers above predate it and have not been re-run, so
  do not quote a new idle percentage until they have been.
- **The other half of that throughput plan was dropped, not deferred.**
  Kitting the box *to demand* (a `FILL_MIN`/`FILL_MAX` band) went out with
  the revert and was never redone: the arm fills a fixed
  `FILL_TARGETS = (3, 2)` (`omnilink_arm_bridge.py:4070`) and `FILL_MIN`
  appears nowhere in the tree. If you ever do raise parts-per-box, `347ab0c6`
  records the envelope the gripper is actually proven on — radius ≤ 0.72 m,
  pitch ≥ 0.16 x / 0.18 y, about four more parts — and the WIP failed
  precisely by shipping past it. (The `GRASP_PART_G/H` column added above
  spends two of that headroom, and spends it *inward*: those pads sit at
  0.22/0.27 m XY radius against the 0.38–0.70 m the original six use, on
  the same 0.16 × 0.18 pitch. It raises tray **stock**, not
  parts-per-box — the two are independent.)
- **Single-run throughput deltas on this demo are not yet separable from
  noise.** Across two runs of *identical* shipped code, `tug_a`'s path
  economy varied 23 → 99 deg/m. Treat one run as an anecdote; both harnesses
  in `tests/benchmarks/warehouse/` say so in their own output.
- **RTF is the one gate `347ab0c6` missed.** Realtime factor dipped
  0.983 → 0.922 for the first ~10 min before recovering to 0.976; the
  measured cause is concurrent kinematic movers (0.935 with one, 0.867 with
  two) now running at ~3× the old duty cycle. The untested mitigation is
  raising the fill conveyor speed 0.26 → 0.40 m/s, not done because it
  changes how the demo looks.
- **Offline mode resumes on set phrases, but not on novel ones.** (Earlier
  revisions of this file said the regex router had no resume intent at all.
  That was wrong.) Both bridges check `shared_is_resume()` **first** in
  `IntentRouter.dispatch` — `omnilink_mobile_bridge.py:2207`,
  `omnilink_arm_bridge.py:2670` — before any motion rule, because "back to
  work" contains "back", which the mobile router would otherwise read as
  reverse 1 m. It is backed by `RESUME_RE` in
  [`packages/omnisim-bridges/src/omnisim_bridges/intent_router.py`](../../../../../packages/omnisim-bridges/src/omnisim_bridges/intent_router.py):
  `resume`, `carry on`, `keep going` / `keep working` / `keep at it`,
  `continue`, `as you were`, `proceed`, `back to work` / `back to it`,
  `get back to work`, `go back to work`, `restart your work/loop/autonomy`,
  `unpause`. Measured offline: "carry on, back to work" un-paused the tug at
  **0.0 s**. Two real limits remain. It is **conditional on the
  `omnisim_bridges` package being importable** — the import is wrapped in
  `try/except` and falls back to `shared_is_resume = None`, which disables
  the intent entirely, silently. And a resume phrased around that list is
  simply not understood: it falls through to the quiet timer, measured at
  **56.0 s and 55.9 s** — those figures are against the bridge's `60.0 s`
  default; this world now runs the window at **12 s** (see
  [Pause and resume](#pause-and-resume)), so the fall-through wait is
  roughly a sixth of that, but it has not been re-measured. So offline
  resume is a phrase list, not comprehension — which is exactly the thing
  an LLM mode adds.
- **The OmniTug 500 is kinematic.** Its URDF is a single static link with no
  `<collision>` and no `<inertial>`, so the importer emits no physics and no
  bounding object: the tug is a visual prop the supervisor drives. It will
  not collide with anything, and it cannot push a cart through contact.
- **The trolleys are pinned physics ensembles.** They *are* real physics
  bodies (mass 22 kg, deck collider), but while a tug tows one its pose is
  written by the supervisor every couple of ticks — follow-the-leader
  trailer kinematics, not a simulated hitch. Riders (`BOX_*`, `GRASP_*`) are
  re-pinned on the deck the same way. After detach the pinning stops and the
  contents simply rest in place.
- **The one off-stage teleport** is the dock E → dock 2 handoff: a collected
  cart leaving the building is returned as an empty, cart for cart, entirely
  inside a doorway. It is deliberate (it is what makes the ring conserved
  and endless), but it is a teleport.
- **The Foreman's brief is coarser than this world.** (It used to be
  outright stale — "dispatch bay 2", a "buffer spot", two trolleys
  alternating roles. Those are gone; `MAIN_TASK` was rewritten at
  `eb390083` and now names the park row, the fill station and the floor
  conveyor correctly.) What it still does not carry is the *detail*: not the
  six named park spots, not the eight-cart conserved ring, not the three
  separate conveyors by name, not the collect-when-full run out through
  dock E. So its diagnostic method is sound and its stage names are right,
  but it cannot tell you where a specific cart is. Prefer asking it about
  starved/blocked stages over asking it where something is.
