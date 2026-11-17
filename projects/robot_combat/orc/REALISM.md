# Foundry physics: what is modeled and what is verified

Foundry now uses physical motor torques, explicit mass properties, and an
energy-bounded component damage model. It remains a rigid-body game prototype.
The numerical checks below establish internal consistency, not agreement with
a measured real robot. They do not establish AAA performance or an open world.

## Driving and weapons

The design uses an **AmpFlow A28-150 electrical model**, with parameters from
the manufacturer's [motor table](https://www.ampflow.com/electric-motors/high-performance/3-inch-76mm/)
and [performance calculator](https://www.ampflow.com/electric-motor-calculator/):
24 V, resistance 0.064 ohm, Kv 256.9 RPM/V, and 4.4 A no-load current.
Kt and Ke are reciprocal SI conversions of Kv. The unit check compares the
published 5500 RPM / 40 A / 190 oz-in operating point, allowing for rounding.
These are published specifications, not measurements made by OmniSim.

The following are **authored design assumptions**, not manufacturer claims:

| Parameter | Foundry value |
|---|---:|
| Four independent drive reductions | 19.2:1, 90% efficiency |
| Drive current limit | 40 A per wheel |
| Maximum drive duty | 75% |
| ANVIL weapon reduction | 8.26:1 |
| RAZOR weapon reduction | 4.38:1 |
| Weapon current limit | 100 A |
| Shared battery | 24 V, 12 Ah, 12 milliohm resistance |
| Wheel radius / tire friction | 0.17 m / 0.9 |
| Other surface friction | 0.45 |

Motor current depends on applied voltage and measured relative shaft speed.
Current limits, back EMF, transmission losses and shared supply sag limit
delivered torque and power. The pack loses energy under load. Regenerated
energy is dissipated; it does not refill the battery. Neutral drive commands
apply dynamic braking. Switching a weapon off while fighting lets it coast;
the end-of-match command brakes it.

Every motor applies equal and opposite torques to the part and its chassis.
The joints are passive, so there is no hidden velocity servo, added servo
armature, or required global torque-mode environment setting. No gameplay
controller sets a robot's pose or velocity to propel it.

Each default robot has a 48 kg chassis, four 2.4 kg wheels and an 8 kg weapon: **65.6 kg
total**. Centers of mass and inertia tensors are explicit. Inertias approximate
the parts as boxes/cylinders; the visuals are not a CAD-derived solid material
mass model. The rotor mass describes an assumed lightweight/hollow assembly.
Bearing drag is an equivalent viscous loss. Motor inductance, rotor/gear
inertia, thermal failure, battery chemistry, tire deformation and aerodynamic
drag remain unmodeled. Friction and grip have not been fitted to track tests.

## Garage builds and strategies

The local garage allows either slot to use either weapon. Names and paint are
cosmetic. Both sides use the same physical catalog:

| Choice | Effect in the simulation |
|---|---|
| Light / balanced / reinforced frame | 36 / 48 / 60 kg chassis mass; inertia scales with mass |
| Frame mount package | Assumed shaft diameter factors 0.90 / 1.00 / 1.10; bending yield scales with diameter cubed and stiffness with its fourth power |
| Torque / balanced / speed gearing | Drive reduction 24 / 19.2 / 14; same motor, current limit and efficiency |
| 6 / 8 / 10 kg rotor | Rotor inertia scales with mass; fixed shape, motor and transmission |
| Pressure / Flanker / Counter-striker | Different approach, retreat and spin-up decisions, expressed only as motor commands |

Total mass ranges from 51.6 to 79.6 kg. Additional mount and bracing mass is
included in each chassis package. These are authored engineering assumptions:
the package does not generate new structural CAD, simulate plate deformation,
or establish a measured relation between armor thickness and failure. Rotor
mass assumes a different internal construction within the same external shape.
The damage estimator, generated mass properties, motors, detached bodies and
garage figures all use the same selected specification. No build gets a
scripted damage multiplier or a motion impulse.

Garage speed and energy figures assume a charged pack and no external load;
they exclude shared battery sag at that operating point. Strategy pilots have
access to simulated opponent positions and wheel state, rather than simulated
vision. Their selections are fixed before the match. The physics verification
below is evidence for the stated default builds; it does not calibrate every
custom design or promise a knockout in every match.

## Collision representation

Wheels, drum and spindle support use closed 64-sided convex collision meshes.
Their axial dimensions match the authored parts. This avoids the engine's
legacy thin-cylinder-to-capsule substitution, which expanded a 0.122 m-wide
Foundry tire to a 0.462 m-wide collision proxy. The polygonal surface still
approximates a circular tire and does not model rubber deformation. Mesh
colliders now receive the authored friction coefficients through the native
backend; previously those coefficients were ignored on meshes.

## Collision damage

The Newton/MuJoCo solver determines motion and collisions at a 1 ms physics
substep, with 8 ms control updates. Damage requires paired contact points
between the two robots. This version does not damage robots from walls, the
floor or loose debris.
Paired contacts must share the same world point within 1 micrometre; nearby
floor contacts are not treated as robot-to-robot hits.

The public contact API supplies position, owner and penetration depth, but
does not expose the native solver's contact normal or force to this controller.
**Logged peak loads are estimates, not measured solver forces.** The model:

1. Estimates a target surface normal from an oriented box/cylinder proxy.
2. Uses pre-impact point velocities, including actual weapon rotation, to
   estimate closing speed and effective mass along that normal.
3. Estimates incident energy and peak load from component compliance.
4. Limits absorbed energy using the change in normal relative motion and a
   shared whole-assembly energy-loss budget, allowing for motor work and
   gravity. Multiple contact points cannot each spend the full budget.
5. Allocates that budget between components by compliance. Only estimated
   plastic work above the assumed yield load consumes integrity. Sub-yield
   contact has no automatic wear or health drain.

Sustained contact retains one peak-work budget. Re-contact after at least
32 ms of separation starts a new episode. The HUD percentage is remaining
modeled plastic-work capacity; it is not a generic speed-to-HP formula.

The strength assumptions are in `shared/foundry_physics.py`: shaft bending
section moduli, assumed yield stresses, compliant mount stiffness and a
plastic travel allowance. They describe simplified **mount failure**, not
finite-element armor fracture. There are no measured coupons or crash tests
supporting those assumptions yet. Effective mass omits some articulated
chassis compliance; proxy normals, contact sampling, terrain friction in the
assembly loss budget and the episode filter also limit accuracy. No damage
routine adds an impact impulse or launch velocity.

## Detachment and the engine fix

A detached part keeps its collider, mass, inertia, world pose and full velocity.
The velocity is staged **before** physics finalization. Restoring it after the
first rebuilt step would overwrite a collision that had already happened.

The engine rebuild now also restores surviving articulated joint velocities
from the captured body motion. Previously it restored free bases but reset
remaining wheel/weapon spin. Ordinary public `setVelocity` calls on articulated
children remain unsupported; the change is specific to restoring a rebuild.

The horizontal blade is forward of the chassis at a low striking height. Its
complete tooth sweep clears the chassis and wheels, and its support lies below
the blade. Raising it above the chassis had made it miss much of the opponent's
body. The drum is also forward and clear of the floor. Parent/child collision
filtering must not conceal overlap that becomes a spurious collision on release.

The native compound-collider walker now preserves mesh children alongside
primitive children. Previously it silently skipped the mesh whenever a sibling
primitive registered. That removed ANVIL's main hull from the physics model
after adding its support arm. An isolated regression reproduces the missing
hull on the old binary and checks mesh support, authored transforms and a
rebuild on the corrected binary. A separate sliding comparison verifies that
mesh friction changes physical motion. Foundry stops with an error if a chassis
penetrates the yard floor beyond its conservative 5 cm geometry tolerance.

## Physical immobilization

Damage percentages and wheel counts no longer declare defeat or switch off
working motors. A robot must fail to demonstrate motion for ten seconds while
fresh, non-neutral drive requests continue. The monitor conservatively bounds
all displacement below 0.15 m and rotation below 0.25 rad during that interval.
Any opponent contact restarts the count. These are adjudication tolerances,
not claims of mathematically zero motion.

The AI attempts forward, reverse and both turns during recovery; the opponent
withdraws through ordinary drive torques. A chassis damage indicator remains
diagnostic: this version does not deform or fracture the hull. Wheel and weapon
mount failures remove the owning joint and produce physically moving debris.
The main top armor panels also have collision shapes. A robot overturned by
combat may therefore rest on its armor with its drive wheels clear of the floor.
Inverted driving commands preserve forward and steering intent; inversion does
not disable a motor in software. A combat-induced overturn is logged separately
from mount fracture.

The match verifier independently recomputes motion from recorded poses,
requires continued drive requests before the result, checks that mount failure
or a combat-induced overturn preceded the count-out and that the winner demonstrated mobility, and rejects
time limits, missing observations, excess plastic-work spending, mass loss,
controller errors and significant chassis-floor penetration.

## Verification

Run the sequential, temperature-guarded numerical checks:

```console
python projects/robot_combat/orc/verify_foundry.py
python projects/robot_combat/orc/verify_foundry.py --cases match --render
python -m unittest discover -s tests -p test_orc_foundry.py
```

The physical probes cover free spin and reaction torque, grounded drum spin, straight driving and
braking, preserving a spinning assembly through a rebuild, and releasing a
spinning weapon in zero gravity. They save full readings and a machine
fingerprint under `_scratch/foundry_realism/`. They fail on numerical acceptance
checks; merely loading a world is insufficient.

The [full-match evidence](verification/full_match_cpu.json) records the
September 13, 2026 tests on machine `9722d23d12a3`, CPU MuJoCo, RTX 3060 Laptop
GPU. It includes source and binary hashes because the engine was rebuilt from
the working tree. The earlier [probe record](verification/realism_cpu.json)
predates the low blade and dimensionally correct collision meshes; it is
retained as historical evidence, not the current geometry's performance.

The full headless match ended at simulation time **144.728 s**, including its
three-second countdown. ANVIL overturned in combat, later lost its front-right
wheel and weapon, and could not recover. RAZOR had lost both left wheels but
remained mobile. No damage percentage or wheel-count rule stopped either drive.
The final ten seconds contained fresh, non-neutral drive requests throughout,
without paired opponent contact. Independent pose analysis measured ANVIL's
translation span at **0.08148 m** and rotation span at **0.07065 rad**; RAZOR's
preceding 15-second translation span was **3.820 m**. Small reaction motion
remained. The result therefore proves failed powered recovery within the
declared tolerances, not mathematically zero movement.

That match passed the independent mobility, mass, plastic-work-budget and floor
checks, with zero engine errors/warnings and a **64°C GPU peak**. It is one
successful physical knockout, not evidence that every encounter ends the same
way. Controller scheduling and chaotic impacts can produce different matches.

Two subsequent rendered matches reached the three-minute time limit and
correctly **failed the knockout check**. The first lost one wheel; the second
lost four wheels across both robots, but both still moved. Each complete movie
is 185 seconds, 1280×720 at 25 FPS (countdown and two seconds of aftermath
included). Both had zero engine errors/warnings; GPU peaks were **67°C** and
**64°C**. Their recordings and failed assessments are retained in the evidence
record. Neither movie is presented as a recording of the successful knockout.
The separate knockout-aftermath still renders the successful run's saved world
with its poses frozen; it is a visualization of the saved state, not a new run.

With the corrected collision meshes, isolated ANVIL spin reached **734.2 RPM**:
motor work was **1911.0185 J**, kinetic energy **1911.0163 J**, a relative
difference of **0.000114%**. The grounded drum reached **1,388 RPM**. Straight
driving reached **4.126 m/s** with a **4.182 m/s** wheel-rim speed, and dynamic
braking stopped it in **0.860 m**. The zero-gravity rebuild and release energy
changes were **0.000062%** and **0.000106%**. These isolated conservation checks
do not validate arbitrary crash outcomes. Spin/drum/rebuild probes preceded
the final top-armor additions; drive/release probes were repeated afterward.

All **33 Foundry unit tests** passed. The broad engine-free suite passed
**1,329 tests**, skipped 11 and deselected 40 engine tests; its 46 warnings are
retained in the raw log. Two new native-engine regressions passed separately,
checking mixed mesh/primitive support through a rebuild and the physical effect
of mesh friction. Three existing compound-collider warning checks also passed.
Afterward, an additional freshness criterion was added to the independent
match assessor. The 33 Foundry tests were rerun, including a stale-command
rejection, and the saved match readings were reassessed without changing them.

The launcher samples GPU temperature every 0.5 s, begins at or below 65°C and
stops its process tree at 75°C. It also stops on sensor failure. CPU temperature
is not monitored. Sampling provides a margin below the requested 80°C ceiling,
not a hardware guarantee. Use the guarded launcher for interactive sessions.

To validate real-world accuracy next, replace the assumed grip/strength/mass
properties with measurements from a specific robot, fit acceleration, braking
and spin-up curves, expose native contact forces, and compare controlled impact
and breakage tests. Until then, describe Foundry as physically simulated with
an estimated component damage model.
