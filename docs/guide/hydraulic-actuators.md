# Hydraulic actuators

A motor moves a joint at whatever torque you ask, up to a limit. A hydraulic machine (an excavator,
a crane, a press, a walking machine built from excavator arms) moves it with a **cylinder** fed by a
**pump** through a **valve**, and that is a different machine:

- its **speed** is set by oil flow: the pump's flow, shared between every cylinder on it, divided by
  the piston area on the pressure side and turned into joint speed through the cylinder's moment arm;
- its **force** is capped by the **relief valve**: relief pressure × that area × the arm, and the arm
  changes with the joint angle;
- a **push** (bore area) is stronger and slower than a **pull** (annulus area);
- a load above relief **pushes the cylinder back**;
- real parts are imperfect: valves have **deadband, lag and slew**; an **open-centre** valve sends oil
  to the lightest load first; pumps lose flow when **worn** and are capped by their engine's
  **power**; cylinders and spools **leak**, so a held load **creeps**; a cylinder stops at the **ends
  of its stroke**.

`omnisim.hydraulics` models all of this and drives an ordinary OmniSim joint with it, from your
controller. No engine change, no special node: a `HingeJoint` (or `SliderJoint`) with a motor and a
position sensor is enough.

## Quick start

```python
from omnisim import Robot
from omnisim.hydraulics import Cylinder, Mount, Pump, Valve, Circuit, Drive, velocity_toward

robot = Robot()
dt_ms = int(robot.getBasicTimeStep())

boom = Cylinder.mm(bore=63, rod=35, stroke=300, retracted=480)    # millimetres
circuit = Circuit(Pump(flow_lpm=10.5, relief_bar=160), Valve.proportional())
circuit.add("boom", boom, Mount.fit(boom, q_lo=-0.5, q_hi=0.8, ra=0.30))
drive = Drive(robot, [circuit], {"boom": ("boom_motor", "boom_sensor")})

while robot.step(dt_ms) != -1:
    q = drive.joints["boom"].sensor.getValue()
    report = drive.step({"boom": velocity_toward(q, 0.5)})          # rad/s you ASK for
    r = report["boom"]
    # r["requested"], r["delivered"] (what the oil allows), r["achieved"] (measured),
    # r["torque"], r["cap"], r["use"], r["pressure_bar"], r["at_relief"],
    # r["length"], r["stroke_fraction"], r["creep"], r["spool"]
```

`drive.step()` takes the joint **velocities you ask for** and returns, per joint, what the oil
**delivered** and what the joint **achieved**. Read both: asking for 2 rad/s from a 10 L/min pump gets
you about 0.25.

### The world side

Give the joint's motor a `maxTorque` (or `maxForce`) **several times the cylinder's relief torque**.
It is the joint's **stiffness**, not its strength: the engine's position servo is
`ke = 10 × maxTorque`, `kd = 0.5 × maxTorque`, and the drive caps the strength at relief itself. At
10 × the relief torque a joint holding a full relief load deflects 0.01 rad. Give it `minPosition` /
`maxPosition` for the joint's range; the drive adds the stroke's ends inside it.

## The parts

| Part | What you give | Imperfections |
|---|---|---|
| `Cylinder.mm(bore, rod, stroke, retracted)` | sizes in mm (`Cylinder(...)` takes metres) | `leak_lpm_at_100bar`: piston-seal leakage, so a held load creeps even behind a load-holding valve |
| `Mount.fit(cyl, q_lo, q_hi, ra, sign)` | the joint range and the parent pin's radius; solves the child pin so the range uses the stroke | — |
| `Mount.pins(ra, rb, phi0, sign)` / `Mount.linear(L0)` / `Mount(fn)` | your own pin layout, a straight slider, or any linkage as `q -> length` | — |
| `Pump(flow_lpm, relief_bar)` | no-load flow and relief pressure | `power_kw` (flow ≤ efficiency × power / pressure), `wear` (fraction of flow lost at relief) |
| `Valve.proportional()` | pressure-compensated electro-hydraulic spools; requests are scaled together when they exceed the pump, so the machine slows down but keeps its path | `deadband`, `lag_s`, `slew_per_s`, `spool_leak` |
| `Valve.open_centre()` | a manual sectional valve (levers, maybe pushed by servos): **oil goes where the pressure is lowest** | the same, with cheap-valve defaults (15 % overlap, 0.12 s lag) |
| `Circuit(pump, valve).add(name, cyl, mount, load_holding=True)` | which cylinders share which pump | `load_holding=False`: no counterbalance valve, so the spool's leak adds to the creep |

All defaults are typical values, not measurements of any particular part. A study that matters
should use its own parts' numbers, and say which ones it assumed.

## What is measured

The engine pin [`tests/test_hydraulics_engine.py`](../../tests/test_hydraulics_engine.py) runs a
boom (payload 300 kg at 1.5 m; 63/35 × 300 mm cylinder; 10.5 L/min, 160 bar pump; proportional valve)
on the engine, CPU `newtonSolver "mujoco"`, 2 ms step. On 2026-11-08:

| Check | Model | Engine |
|---|---|---|
| Torque holding the payload, inferred from the servo law | 4,414 N·m (its weight) | 4,414 N·m |
| Rise speed (asked 2 rad/s) at q = 0.05 / 0.17 / 0.30 rad | 0.233 / 0.249 / 0.272 rad/s (flow ÷ area ÷ arm) | 0.233 / 0.249 / 0.272 rad/s |
| 1,100 kg payload asked to rise | above relief: pushed down, `at_relief` | pushed down, `at_relief`, no flow delivered |
| Leaky cylinder (0.5 L/min at 100 bar), held 3 s | 0.0185 rad of creep | 0.0183 rad |

The engine-free half, [`tests/test_hydraulics.py`](../../tests/test_hydraulics.py), checks the
circuit against hand calculations (flow sharing, open-centre priority, relief, deadband, lag, pump
wear and power) and the drive on a plant that integrates the same servo law.

## What it does not model

- **Oil compressibility and hose volume**, beyond the valve's lag; **pressure dynamics** inside a
  control step: each step the circuit is solved quasi-statically.
- **Line losses, heat, cavitation, pressure spikes**.
- **Torque feedback**: the Newton backend does not measure joint torque yet (`getTorqueFeedback`
  reads 0), so the drive infers each cylinder's load from the servo law. That is exact while the joint
  is inside its cap and the gains are the engine's; if you override them (`OMNISIM_NEWTON_TARGET_KE`,
  or a custom servo), pass `gains={"boom": (ke, kd)}` to `Drive`.
- **Structural strength**: the drive tells you the force in each cylinder (`torque / arm`); whether
  the pins, links and frame carry it is your structural check.

## Why it exists

In the 2026-11-07 agent test on Hacksmith's Megahex (a six-legged walker built from towable
excavators), all seven sessions, inside and outside OmniSim, wrote their own hydraulic model before
their machine could move like one, and the ones that modelled the machine as it was built (stock
cylinders, lever valves) were the ones that reproduced why it failed. This module is that model as a
part of OmniSim, so an agent starts from it instead of writing one.
