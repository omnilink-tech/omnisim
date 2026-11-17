# Track D loop verification rigs

The rigs behind the reproducibility and journal figures in the `## [v9.0.0]` section of
[`CHANGELOG.md`](../../../CHANGELOG.md) and the honesty table in
[`docs/developer/positioning.md`](../../../docs/developer/positioning.md). They are
committed, not left in a scratch directory, because a figure nobody can re-run is not
evidence.

Both need `OMNI_KEY` in the environment. Neither needs a model provider.

## `determinism_probe.py` — does an OmniLink-driven run reproduce?

```bash
python tests/benchmarks/trackd_loop/determinism_probe.py --trials 3 --out free.json
python tests/benchmarks/trackd_loop/determinism_probe.py --trials 3 --lockstep --pace held --out held.json
```

Launches `omnilink_husky.omniworld`, drives four sentences the deterministic parser
answers through `POST /prompt`, and hashes what the agent can see: each measured result
and the settled pose after each command. The probe aborts if any sentence is answered by
anything but the parser, so it can never quietly measure a model.

⚠️ **Why not `agents/production/husky_maze/scripts/ab_arms.py`.** That rig drives
`husky_omnilink_bridge`, a standalone controller that imports no `omnisim_bridges` at
all. It has none of the sim-time work, none of the step budgets and no hold, so a number
from it would describe an untouched code path.

⚠️ **The client paces itself on the wall clock, on purpose.** Between commands it polls
until the base is at rest, which is how every HTTP client behaves and is the coupling
under test. `--pace` chooses what "at rest" means: `velocity` (the default, the naive
rule), `idle` (the bridge's motion slot is empty) or `held` (idle and the world
re-frozen, the only rule that is correct under lockstep).

### Result, free-running (`evidence/determinism_free_2026-09-22.json`)

Machine `9722d23d12a3` (16 cores, RTX 3060 Laptop, Windows 11), engine `136900a45`,
solver newton/MuJoCo `cpu/mj_step`, the bitwise-deterministic one.

| | |
|---|---|
| trials | 3 |
| distinct trace hashes | **3** |
| positions | identical to 4 dp in every run |
| sim time at the same point | 2.496 to 6.72 s |
| final x | **0.9874, 0.9874, 1.9843** m |

The physics is deterministic; the coupling is not. In two runs the third command came
back `busy` and was never executed; in the third it ran. Identical commands left the
robot a full metre apart. The earlier measurement of this residual (2026-09-19) was one
hundredth of a radian of yaw; this is a different outcome, not a rounding difference.

### Result, lockstep

Same machine, same engine, 3 trials each. `--pace` is how the client decides the robot
is at rest before its next command.

| Run | Hold | Pace | Resets | Distinct hashes | Evidence |
|---|---|---|---|---|---|
| A | off | velocity | 0 | 3 | `determinism_free_2026-09-22.json` |
| B | off | idle | 0 | 3 | `determinism_free_idlepaced_2026-09-22.json` |
| C | **on** | velocity | 0 | 3 | `determinism_lockstep_velocitypaced_2026-09-22.json` |
| D | **on** | **held** | 0 | **1 — identical** | `determinism_lockstep_holdpaced_2026-09-22.json` |

In run D all four commands executed in every trial and the world reached the same rest
points at the same sim times: 3.472, 9.12, 12.544 and 12.608 s.

**Read the table as two findings, not one.**

1. **The hold closes the gap, but only for a client that waits on it.** Run B shows that
   waiting on the bridge's own idle flag is not enough without the hold: rest points
   still land seconds apart in sim time. Run C shows the hold is not enough with a naive
   client: under a hold the wheels read zero because the world is frozen, not because the
   motion is done, so the velocity rule sends the next command early, it comes back
   `busy`, and the runs diverge. Only D, the hold plus a client pacing on `held`, is
   reproducible. That is a requirement on any client that wants this property, including
   `ab_arms.py --lockstep`.
2. **Getting here fixed a real defect.** The first lockstep attempt reset the connection.
   The cause was not a deadlock: a bridge holding the world makes no calls to the engine,
   so when the engine died the bridge never noticed, kept serving on its port for up to a
   lease, and the next trial's requests landed on that orphan. The hold now checks the
   engine every 0.5 s and the bridge exits 0.35 s after its engine, like a free-running
   one. See the commit that carries this table.

⚠️ This is reproducibility of a RUN on the CPU solver, with parser-answered commands.
It is not replay, and it says nothing about a run whose commands a model chose.

## `journal_cross_machine.py` — does the action journal cross a machine?

```bash
python tests/benchmarks/trackd_loop/journal_cross_machine.py
```

Two relays with disjoint state directories, so the second cannot read the first's local
journal file. The only channel between them is the platform's memory API.

### Result, 2026-09-22, same machine as above

```
[A] recorded 3 entries; synced with no model turn
[platform] memory rows: 1; marks present: 3/3
[B] local state dir before boot: EMPTY
[B] get_action_history returned 3 rows; marks restored: 3/3
=== VERDICT: CROSSES MACHINES (platform 3/3, restored 3/3)
```

Before the fix in `25bd8088a` the same run synced nothing: the journal only reached the
platform after a completed model turn, and this relay never makes one.

Side effects: one memory row under the scratch name `OmniSim-trackd-journalprobe`,
cleared at the end. No profile is created, so the account's profile limit is untouched.

## `wake_live.py` — does a robot event reach the platform as one bounded wake?

```bash
python tests/benchmarks/trackd_loop/wake_live.py
```

A real `EventRing` feeding a real relay on a real OmniKey, against the live platform.
No engine: the ring is fed directly, the way a detector feeds it.

### Result, 2026-09-22, same machine as above

```
[1] motion.unsettled (policy: no wake) -> replies 0 (want 0)
[2] motion.timed_out -> reply after 0.9s, kind='error'  (402 BYOK_REQUIRED)
[3] burst of 20 within the window -> extra replies 0 (want 0)
=== VERDICT: WAKE REACHES THE PLATFORM, BOUNDED
```

⚠️ **That 402 was mislabelled, and the mislabel is how a day of diagnosis went wrong.**
The relay turned every 402 into "needs a model-provider key". The platform's real
reason was `AGENT_LIMIT_REACHED`: the account held 41 agent profiles on a plan
allowing 25, while its provider key worked. The relay now reports the code the
platform states (`packages/omnisim-bridges/tests/test_relay_402_diagnosis.py`).

### Result, 2026-09-23, after clearing twenty unused profiles

A new scratch name counts as a new agent, so on a full account run it under an
existing profile: `TRACKD_WAKE_AGENT=OmniSim-husky`.

```
[1] motion.unsettled (policy: no wake) -> replies 0 (want 0)
[2] motion.timed_out -> reply after 9.1s, kind='agent'
      'It looks like my attempt to drive forward timed out after covering
       1.2 meters of the requested 5.0 meters.'
[3] burst of 20 within the window -> extra replies 0 (want 0)
=== VERDICT: WAKE REACHES THE PLATFORM, BOUNDED  model_answered: True
```

The model's words name the figures the event carried: 5.0 commanded, 1.2 achieved.
This is the wake closed end to end.

Side effects: heartbeat rows under the agent used, deleted at the end. Under an existing
profile, one probe turn and its reply also land in that agent's conversation memory.
