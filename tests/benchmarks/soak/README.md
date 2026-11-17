# soak — does the deterministic core survive hours of the same job?

```bash
python tests/benchmarks/soak/run.py --hours 2
```

The cost thesis rests on a claim that is easy to state and easy to fake:
an OmniLink agent can drive a repetitive robot job indefinitely for nothing,
because no model is in the loop. This benchmark exists to try to break that.

## What it drives

Forward 2 m, turn left 90°, four times. That path **closes** — the robot
should arrive back where it started. So the residual after each lap is
*drift*, a number, rather than a wander that eventually runs out of arena
and tells you nothing.

The world is `chat/omnilink_husky.omniworld`, the surface is the mobile
bridge's `POST /prompt`, and no key is passed, so any nonzero token count
would mean a model got into the loop. The run asserts it stays zero.

## What would falsify the claim

All of these are recorded, because a benchmark that can only pass is not
evidence:

| | |
|---|---|
| `errors` | a command that fails, at any point |
| `drift` | lap-closure error growing without bound |
| `escape` | the robot leaving the arena |
| `leak` | engine RSS climbing steadily |
| `slowdown` | seconds-per-lap degrading |
| `death` | the engine or bridge going away |

## ⚠️ Two ways this benchmark has lied, and what stops each now

Both were failures of the *instrument*, not of the system under test. They
are written down because the pattern matters more than either bug: a
passing endurance run is the single easiest result in this repo to fake by
accident.

**1. It reported 25 perfect laps while the robot never moved.** A leftover
fake Ollama recorder on port 11434 had been adopted as the relay and was
answering every command with a canned "ok". Closure was 0.000 m — *perfect*
— and errors were zero, because nothing had happened at all.

> Now: a pre-flight sends one drive command and aborts unless the robot
> actually moves ≥ 0.5 m, and every lap asserts a max excursion. Absence of
> errors is not evidence of work.

**2. It reported two clean hours while dropping ~5% of its commands.** The
bridge refuses an order that arrives while it is still moving. It said so
in plain words — `"I could not turn: busy"` — and marked the action
`result: "err"`. But there was no top-level `error` field, and the runner
checked that field. The dropped orders desynced the square, it stopped
closing at lap 15, and the robot walked off the 12 m floor to
`(-6.86, 4.75)`. Zero errors logged, start to finish.

The commands themselves were never inaccurate. The last turn before the
escape was commanded 1.5707963 rad and achieved 1.5712195 — 0.00042 rad of
error. What failed was the reporting.

> Now: a failed action sets a top-level `error`
> (`omnisim_bridges.route.reply_payload`), the runner confirms each command
> against the bridge's own `last_command.seq` rather than inferring
> completion from pose, and an arena escape aborts the run.

Pose-settling is *not* command completion, and that is the subtle half:
`act_drive_forward` moves in phases with brief pauses, so three still
readings can land inside one of them. The sequence counter is the truth.

## A gap this found and did not close

**Nothing in the stack bounds the robot to the world.** `act_drive_forward`
takes a distance and drives it; the bridge does not know the floor is 12 m
across. The soak now *detects* an escape, but detection is an instrument,
not a fix. A real bound needs a decision about where world extent comes
from, and that decision has not been made.

## Output

Per-lap rows land in the `--out` JSON: closure, excursion, lap seconds,
engine RSS, tokens, cumulative errors. The summary compares first-half to
second-half lap times (slowdown) and first to last closure (drift).

Exit status is nonzero if anything was recorded in `errors`.
