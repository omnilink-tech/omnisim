# omnisim-hil — hardware-in-the-loop for aircraft

**The question this package answers: can OmniSim be the simulator in a
hardware-in-the-loop rig — the bench where a delivery company flies every
software build before it touches the fleet?**

The pattern is standard in aerospace and it is worth naming precisely, because
the three tiers have very different costs and this package only reaches the
first two.

| tier | what runs | what it catches |
|---|---|---|
| **software-in-the-loop** | the production flight software, against simulated physics, no hardware | logic, control law, mission regressions — cheaply and in parallel |
| **hardware-in-the-loop** | the real flight computer on a bench, fed simulated sensors over its real buses | timing, driver, and integration faults SIL cannot see |
| **iron bird** | real servos and power system, physically moving | electrical and mechanical integration |

This package delivers the SIL tier fully and the simulator half of the HIL tier:
the flight software is a separate process speaking the same MAVLink HIL protocol
PX4 and ArduPilot use, so a real autopilot — SITL or a board on a desk — is a
substitution, not a rewrite. The iron-bird tier is bench I/O and is out of scope.

---

## Why this needed building at all

OmniSim has no aerodynamics. Its solver knows contact, joints and gravity;
`Fluid` and `ImmersionProperties` went out with the ODE deletion and now
hard-ERROR at parse. Before this package there was no fixed-wing aircraft in the
tree and no lift model anywhere. A wing was a brick.

So the aerodynamics live in a controller, applying forces through
`Supervisor.addForceWithOffset`. That choice is what surfaced the defect below.

---

## What it contains

| path | what it is |
|---|---|
| `omnisim_hil/aero.py` | fixed-wing flight model: component build-up over lifting surfaces, stall blend, propeller with inflow |
| `omnisim_hil/atmosphere.py` | ISA atmosphere, plus steady wind and band-limited turbulence |
| `omnisim_hil/mavlink.py` | stdlib-only MAVLink v2 codec, byte-verified against pymavlink |
| `omnisim_hil/frames.py` | the one place ENU/FLU becomes NED/FRD |
| `omnisim_hil/vec3.py` | dependency-free vector and rotation helpers |
| `controllers/hil_aircraft/` | the simulator-side controller: applies aero forces, speaks MAVLink |
| `controllers/hil_timing_probe/` | records sim time against wall clock, per tick |
| `autopilot/omnisim_autopilot.py` | the flight software: a separate process that binds UDP, decodes the sensor stream and flies a mission |
| `autopilot/missions/` | mission files, in local ENU metres |
| `run_hil_demo.py` | one command: autopilot + engine + a measured report |
| `tools/measure_realtime.py` | the real-time and jitter instrument |
| `worlds/` | the aircraft, and the timing probe worlds |

The runtime imports nothing outside the standard library. That is deliberate:
the interpreter OmniSim spawns for controllers is whatever `python` resolves to
on PATH, and on this machine it has neither pymavlink nor pyserial. A HIL rig
that only runs on a correctly-provisioned box is a HIL rig that does not run.

---

## The defects this work found

### The frame conversion was wrong on the body axes

`frames.py` used ONE matrix for both of its conversions, on the stated reasoning
that ENU→NED and FLU→FRD are "the same involution". They are not. ENU→NED swaps
east and north and negates up; FLU→FRD keeps forward and negates left and up.
Applying the world matrix to a body vector maps the nose onto the right wing:
`flu_to_frd((1,0,0))` returned `(0,1,0)`, so the aircraft reported forward
acceleration as lateral, and `quaternion_frd_ned` on a level aircraft flying
EAST returned the identity — a reported heading of north.

Nothing caught it because nothing consumed it: the only user was the outgoing
MAVLink stream, and until the autopilot existed there was no receiver. It also
survives every property an involution test can check — the wrong matrix is also
its own inverse, also a proper rotation, also round-trips. `tests/test_frames.py`
now pins both maps against named physical cases (level pointing east reads 90
degrees; nose up reads positive pitch) plus an independent cross-check that
converting the MATRIX agrees with converting the vectors, which is the kind of
assertion that could have caught it.

### The external-wrench path was dead

Building the aero model surfaced a repo-wide engine defect that had been live
since the ODE deletion.

`OmSolidMerger::mBody` is assigned `NULL` in its constructor
([`OmSolidMerger.cpp:34`](../../src/omnisim/nodes/utils/OmSolidMerger.cpp#L34))
and **assigned nowhere else in the tree** — the ODE deletion removed its only
writer. Three call sites still gated on it, so all three were dead on every
world:

* `wb_supervisor_node_add_force` and `wb_supervisor_node_add_force_with_offset`
  ([`OmSupervisorUtilities.cpp`](../../src/omnisim/nodes/utils/OmSupervisorUtilities.cpp)) —
  every call warned `"can't be used with a kinematic Solid"` about bodies Newton
  itself reported as dynamic, and returned before reaching the Newton path.
* `Propeller` ([`OmPropeller.cpp:218`](../../src/omnisim/nodes/OmPropeller.cpp#L218)) —
  returned before thrust was even computed, so no propeller in OmniSim produced
  any force, and `getTorqueFeedback` read zero on the one motor where it is real.

`wb_supervisor_node_add_torque` was unaffected, because it already gated on the
Newton body handle. That asymmetry is what the fix generalises: all three now
ask `bodyHandle()`.

**How it stayed hidden.** The failure is silent in the direction that reads as
success — a world loads, steps, logs no ERROR and exits 0 while every external
force is dropped. The Mavic drone demo's lift runs through `addForceWithOffset`,
and its sitting motionless on the ground was attributed to no takeoff command
having been issued. The repo's own pinned regression
`tests/test_newton_external_wrench.py`, written specifically to catch a wrench
regression, was RED on the shipped binary and had not been run since. And the
documented revert hatch `OMNISIM_NEWTON_NO_EXT_FORCE=1` was **inert**, because
the call site that consults it was unreachable.

Verified fixed three independent ways: that pinned test went RED to green;
OmniBench lane 4 probes `device.propeller_thrust` and
`phenomenon.supervisor_external_force` went `broken` to `works`; and the
aircraft, which had been free-falling 45 m in 3.02 s, flew.

---

## What is measured

Machine `9722d23d12a3` (RTX 3060 laptop, Windows 11), CPU `mj_step`. Everything
below is n=1 on one machine unless stated; Linux is unmeasured throughout.

**Real-time pacing — the HIL floor.** Real-time mode is a `QTimer` in the engine,
decoupled from rendering, so headless real-time works. But:

| basicTimeStep | realtime factor | p99 jitter | note |
|---|---|---|---|
| 8 ms | **0.516** | 8.5 ms | cannot be paced: the wall interval pins to the ~15.6 ms OS timer quantum |
| 20 ms | 1.0005 | 12.1 ms | correct mean, but 28.7% of ticks overran by >20% |
| 20.5 ms | 1.0255 | 11.2 ms | the fractional-timestep hazard, silent at exit 0 |

Three consequences for anyone building on this. **Do not use `basicTimeStep 8`
for a real-time run on Windows** — it runs at half speed, and the same scene
steps in 0.561 ms under `--mode=fast`, so that is idle wait, not scene cost.
**Keep `basicTimeStep` an integer**: it is truncated into `QTimer::start(int)`,
so 20.5 ms paces at 20 ms while the sim clock advances 20.5, gaining 2.5% per
second without bound and without any diagnostic. **A correct mean is not a
usable jitter** — judge a rig on the distribution, not the factor.

**Flight model.** 32 offline tests in `tests/test_aero.py` assert physical
properties rather than recorded numbers: static pitch and yaw stability, rate
damping, a stall that reduces lift, adverse yaw, control authority scaling with
dynamic pressure, and a propeller whose thrust falls with airspeed. Two real
model bugs were caught by them before anything flew — induced drag that ignored
the aileron contribution, which made adverse yaw exactly zero, and a trim
calculation that ignored wing incidence and was wrong by 65%.

**Flight.** 60 s of sim time, hand-launched at 17 m/s: altitude held to within
0.44 m of a 45 m target with 0.6 mm standard deviation, 774 m covered, angle of
attack 0.061 rad against a 0.30 rad stall. That run used the controller's own
internal stabiliser, i.e. it measures the FLIGHT MODEL, not any flight software.

**A mission, flown by the separate autopilot process.** `run_hil_demo.py`, three
waypoints, hand-launched at 17 m/s, headless `--mode=fast`:

| condition | waypoints | distance | settled altitude error | airspeed | bad CRC |
|---|---|---|---|---|---|
| calm | 3 of 3 | 834.4 m | mean 0.61 m, worst 2.59 m | 14.1–17.0 m/s | 0 of 13755 |
| `--wind 4 --turbulence 2` | 3 of 3 | 845.6 m | mean 0.64 m, worst 1.98 m | 12.9–17.4 m/s | 0 of 13901 |

"Settled" excludes the 5 s after each commanded altitude step, because the
commanded profile steps at every waypoint and the raw worst-case error is
dominated by the size of the step the autopilot was asked to fly. The raw
figures are in the report too (mean 1.53 m, worst 10.35 m calm).

The wind was verified to have reached the aircraft rather than assumed: binning
ground-speed-minus-airspeed by heading gives ±0.07 m/s in every octant in calm
air, and +3.8 to +4.1 m/s eastbound against −3.6 m/s westbound with the wind on.
The upwind leg to `ZONE_B` took 25.0 s against 18.9 s downwind.

**What the mission runs do NOT show.** The airspeed floor never had to save the
aircraft — it engaged for 2 ticks during the launch transient and never again,
so it is proven by `tests/test_autopilot.py` and not by a flight. And the
autopilot has no state estimator: it takes attitude and position from
`HIL_STATE_QUATERNION`, which is simulator ground truth. A real autopilot's EKF
is a large share of its failure surface and none of that share is under test.

---

## Running it

```bash
# The whole rig: autopilot process + engine + a measured report.
python packages/omnisim-hil/run_hil_demo.py --duration 90
python packages/omnisim-hil/run_hil_demo.py --duration 90 --wind 4 --turbulence 2
python packages/omnisim-hil/run_hil_demo.py --gui --duration 120     # windowed

# The autopilot on its own, against a real PX4/ArduPilot HIL stream or a
# hand-rolled one. It binds the port; the aircraft sends to it.
python packages/omnisim-hil/autopilot/omnisim_autopilot.py \
  --port 14560 --mission packages/omnisim-hil/autopilot/missions/delivery_route.json

# The flight model alone, no autopilot, no link. Proves the airframe flies.
HIL_INTERNAL_AUTOPILOT=1 HIL_MAX_SECONDS=60 \
  python -m omnisim run-world packages/omnisim-hil/worlds/hil_delivery_aircraft.omniworld

# The real-time instrument.
python packages/omnisim-hil/tools/measure_realtime.py --mode realtime --ticks 1200 --step-ms 20

# Offline tests: no engine, no GPU.
python -m pytest packages/omnisim-hil/tests -q
```

---

## What this is not

The flight model is a build-up, not CFD. Surfaces do not shade one another,
there is no propeller slipstream over the tail, no ground effect, no
compressibility and no unsteady terms. It produces an airframe whose trim,
stability and control response are of the right sign and roughly the right
magnitude — enough to exercise flight software, not enough to certify it.

The aircraft's coefficients are representative of a light electric UAV in this
class. They are not measured from any real aircraft and no part of this is
derived from any manufacturer's data.

And the honest limit of the whole approach: this validates flight software
against a *model*. Component models composed from datasheets compose their
errors, and the gap between a good model and a real airframe is exactly the
sim-to-real gap this package narrows rather than closes. What makes a HIL rig
worth its cost is the loop being closed against real hardware and real
measurements — the simulator is one half of it.
