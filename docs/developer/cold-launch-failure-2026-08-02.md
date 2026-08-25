# The intermittent cold-launch failure (open, 2026-08-02)

**Status: reproduced, characterised, NOT root-caused.** Written down because it
is the single most expensive defect in this tree and every hypothesis I tested
was wrong. Whoever picks it up should start from the ruled-out list rather than
repeat it.

## The symptom

Roughly **one cold headless launch in three** ends like this:

```
=== OmniSim Log Started (pid=12528): 2026-08-02 22:59:20 ===
WARNING: DEF CUBE Solid > Physics : Both 'density' and 'mass' specified: ...
Qt Warning: QWaitCondition: Destroyed while threads are still waiting
Qt Warning: QThreadStorage: entry 3 destroyed before end of thread ...
Qt Warning: QObject::killTimer: Timers cannot be stopped from another thread
```

- exit code **1**, within about **one second**
- the world **parses** (the parse warning is there)
- the physics backend never initialises — no `[OmNewtonBackend]` line at all
- no controller starts
- **stderr and stdout are empty**
- the same world, same command, run again, works

The Qt warnings say the process went down while worker threads were still
running.

## Why it is expensive

It is indistinguishable, to any caller, from *the thing being measured having
failed*. A benchmark cell reads "the run produced no data" and scores it
against the simulator, the robot or the agent. Measured today: **2 of 4**
sequential phase-B runs produced no motion file, and the graders correctly but
misleadingly reported them as failures of the deliverable. Several hours of
this session went into diagnoses that were really this bug wearing someone
else's name.

## Ruled out

| hypothesis | how it was killed |
|---|---|
| TCP port exhaustion (range was 11 wide, killed engines hold ports) | plausible, and the range is now 60 — but reproduced with **0 ports held** and **0 engines running** |
| stale per-port tmp dir (`%LOCALAPPDATA%/Temp/webots-<id>/`, GC'd only after 1 h) | deleting `webots-123*` before the run does not prevent it |
| a `commandLineError` path (bad flag, unopenable TCP server) | those now log to the file (`66f0c378`) and **nothing appears** |
| `OmLog::fatal` | it writes `FATAL: ` and `fileLogWrite` flushes **per call**, so it would be visible |
| `QApplication::exit(1)` in `loadInitialWorld` | logs `Invalid world file` first; absent, and the world clearly parsed |
| a slow Newton/warp import being cut off by the timeout | the failure takes **~1 s**; a 150 s timeout fails identically |
| my own 2026-08-02 engine changes | pre-dates them, and reproduces on worlds that do not touch any changed field |
| the world file itself | the *same file* alternates pass/fail across consecutive runs |

## The one thing that was added

`main()` now logs its exit code before closing the log (`omnisim exited with
code %1`). **It does not fire for this bug** — which is itself a clue: the
process is not unwinding to the end of `main`, so it is neither a clean
`QApplication::exit` nor a `OmLog::fatal`. It is either a crash that Windows
reports as 1, or an `exit()` on a path not yet found.

## Where to look next

1. Run under a debugger and break on `exit` / `abort` / first-chance
   exceptions — the failure is frequent enough (1 in 3) to catch quickly.
2. Add a trace line at each stage of `OmApplication::loadWorld` and diff a
   good run against a bad one; the last line reached names the phase.
3. The Qt teardown warnings suggest a thread started during load is being
   destroyed while running — look at what the load path spawns before the
   physics backend is initialised.

## Workaround in force

`ladder.adapters.omnisim.evidence.LAUNCH_ATTEMPTS = 4` (measured: 2 attempts
gave 2/4 runs, 4 attempts gave 4/4). It is a workaround and is labelled as one
in its own docstring. **Any batch runner over this engine needs the same
retry**, and should treat "no data" as a possible instrument failure rather
than a result.
