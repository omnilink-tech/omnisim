# Real-time pacing and jitter

Hardware-in-the-loop needs OmniSim to track wall clock. A device on the other
end of a cable — a flight controller, a CAN bus, a robot arm — runs on wall
clock and will not wait for a simulator that is merely correct. Before this
measurement nothing in the repo had ever checked whether the engine tracks wall
clock, so every HIL claim rested on the assumption that `--mode=realtime` means
what it says.

It does, but only above a hardware-imposed floor, and it fails silently in two
distinct ways below it.

## The instrument

Three pieces, in `packages/omnisim-hil/`:

- `controllers/hil_timing_probe/hil_timing_probe.py` — a controller that does
  nothing but step and time itself. Per tick it records `robot.getTime()` (sim
  seconds) against `time.perf_counter()` (wall seconds). libController exposes
  no wall-clock or real-time-factor accessor, so the wall side has to come from
  the controller process's own clock. The loop does two clock reads and two
  stores into a preallocated list; nothing is formatted or written until the
  recording is finished, because the instrument must not perturb what it
  measures.
- `tools/measure_realtime.py` — launches `omnisim-bin` headless, waits for the
  recording, and reduces it to a real-time factor, a jitter distribution, a
  cumulative drift and an overrun count. Machine attribution comes from
  `projects/policies/common/env_fingerprint.py`, with a documented minimal
  fallback if that cannot be imported.
- `worlds/hil_timing_probe*.omniworld` — near-empty probe worlds. The scene is
  minimal on purpose: this measures the engine's pacing floor, not a scene's
  cost.

```bash
python packages/omnisim-hil/tools/measure_realtime.py --mode realtime --ticks 1200 --step-ms 8
python packages/omnisim-hil/tools/measure_realtime.py --mode fast     --ticks 1200 --step-ms 8
```

## What was measured

Machine `9722d23d12a3` (RTX 3060 laptop, Windows 11), engine binary
`04cfdcdab318a1c9`, commit `355c70bc5`, Newton finalised on CPU `mj_step` for
every run. One machine, one run per configuration except where noted.

| world | basicTimeStep | mode | realtime factor | median wall interval | jitter p99 | final drift |
|---|---|---|---|---|---|---|
| `hil_timing_probe` | 8 | realtime | **0.51587** | 15.478 ms | 8.534 ms | −9002 ms over 9.59 s sim |
| `hil_timing_probe` | 8 | fast | 13.46442 | 0.561 ms | n/a | +8880 ms |
| `hil_timing_probe_fractional` | 8.5 | realtime | 0.54707 | 31.064 ms (2 steps) | 15.632 ms | −16875 ms |
| `hil_timing_probe_20ms` | 20 | realtime | **1.00052** | 15.772 ms | 12.100 ms | +6.2 ms over 11.98 s sim |
| `hil_timing_probe_20p5ms` | 20.5 | realtime | **1.02551** | 45.850 ms (2 steps) | 11.151 ms | +611 ms over 24.56 s sim |

Drift sign convention: positive means the sim clock is ahead of the wall clock.

## Finding 1: realtime mode cannot pace a step below about 15.6 ms

At `basicTimeStep 8` the engine runs at **0.516x real time**. The median wall
interval is 15.478 ms for an 8 ms step — almost exactly the 15.6 ms default
timer granularity of Windows. The `QTimer` at `OmSimulationWorld.cpp:403` is
constructed `Qt::PreciseTimer`, but on this box it is not achieving a period
below the system tick.

The fast-mode control arm is what makes this unambiguous. The same scene steps
in **0.561 ms** median when pacing is removed, so the 15.5 ms in realtime mode
is 99.6% idle wait, not scene cost. The engine is not too slow to hit 8 ms; it
is being woken too late to try.

The `basicTimeStep 20` run is the confirming evidence for the mechanism. Its
intervals are bimodal at roughly 15.8 ms and 31.6 ms — one and two quanta of the
same tick — beating together to a mean of 19.99 ms and a realtime factor of
1.00052. The engine paces correctly on average as soon as the requested period
is above the quantum, and quantises to that grid below it.

The practical consequence is that `basicTimeStep 8`, which the friction and
grasp guidance recommends, runs at half speed under `--mode=realtime` on this
platform. A HIL rig built on it would feed a device sim time at half the rate
the device expects, with nothing in the engine reporting the shortfall.

## Finding 2: the fractional-basicTimeStep truncation hazard reproduces

`OmWorldInfo::basicTimeStep()` (`OmWorldInfo.hpp:193`) returns a `double` and
`QTimer::start` takes an
`int`, so a fractional basic time step is truncated on the wall-clock side while
the sim clock still advances the full fractional amount. The prediction from
reading the code is a world that runs *fast* by exactly the truncated fraction.

The 8.5 ms world could not settle this, because 8 and 8.5 both floor to the same
~15.6 ms quantum — the granularity masks the hazard. Testing it required a
fractional step *above* the quantum, which is why the 20/20.5 ms pair exists.

At `basicTimeStep 20.5` the measured realtime factor is **1.02551**, against a
predicted `20.5 / 20 = 1.025`. A repeat run gave 1.02481. The hazard reproduces
to within 0.2% of the value the code reading predicts, and it is exactly as
silent as predicted: the run logs no error, exits 0, and Newton finalises
normally while the sim clock gains **611 ms over 24.6 seconds** — about 25 ms
per second, without bound. Over ten minutes that is fifteen seconds of
divergence from the device.

The masking is worth stating on its own: on any platform whose timer quantum
exceeds the basic time step, this hazard is invisible, and the sim looks merely
slow rather than wrong.

## Finding 3: mean pacing being correct does not make the jitter usable

The `basicTimeStep 20` run holds real time to 1.00052 with a final drift of
6.2 ms, and would pass any check written only on the real-time factor. Its p99
jitter is **12.1 ms on a 20 ms step**, because individual ticks are quantised to
the 15.6 ms grid and only average out. 28.7% of its ticks overran nominal by
more than 20%.

For a device that cares about when a packet arrives rather than how many arrive
per minute, that distribution is the number that matters. This is why the tool
reports a two-part verdict and why the run above prints `realtime HELD, jitter
p99 MISSED`.

## The verdict is two-sided, and was not at first

The tool's realtime check is a band, not a floor. It began as `factor >= 0.98`,
which reported the 20.5 ms world — running 2.5% fast, drifting without bound —
as having held real time. For hardware in the loop a sim running fast is not a
lesser fault than one running slow; it is the same divergence with the sign
flipped, and it is the harder one to notice. The default band is 0.98 to 1.02
and both bounds are printed with every verdict, along with the jitter budget
(25% of the nominal step by default) and a scope line naming the machine, the
scene and the tick count.

## Limits

n is small: one run per configuration, two for the 20.5 ms case, all on one
Windows laptop. Nothing here measures Linux, where the timer quantum and the
`Qt::PreciseTimer` implementation both differ and the 15.6 ms floor may simply
not exist. Nothing here measures a loaded scene, a scene whose step cost
approaches the pacing period, or the behaviour when physics genuinely cannot
keep up — that last case is where the coalescing `mHasWaitingStep` bool at
`OmControlledWorld.hpp:84` would start dropping steps, and it has not been
exercised. The overrun threshold (20% over nominal) and the jitter budget (25%
of a step) are choices, not physical constants.
