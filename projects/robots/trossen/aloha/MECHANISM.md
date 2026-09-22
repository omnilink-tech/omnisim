# Spring-loaded battery compartment reconstruction

This variant reconstructs the **mechanical task** seen in the public ALOHA
recording: grasp near one battery end, engage the negative terminal while
tilted, compress the spring, release, and press the raised end into its seat.
The real remote's CAD, spring measurements and force traces are unavailable.
The dimensions, material coefficients, pads and controller below are estimates.
This is neither an exact action replay nor a hardware-validated policy.

## Physical model

The negative terminal is a 2 g rigid body on a passive slider with 10 mm of
travel. A controller applies equal and opposite forces to it and the remote
at the same point, at every physics step:

```
F = 0.15 - 100 q - 0.18 dq/dt  [N]
```

Compression makes `q` negative. The force includes 0.15 N preload, 100 N/m
stiffness and 0.18 N s/m damping. The visible coil changes length with the
measured slider displacement; its individual wire turns are not deformable
collision bodies. The fixed positive terminal, floor, divider, side walls and
retaining lip are colliders. Terminal contact is mechanical; electrical
conductivity, plastic deformation and battery damage are not simulated.

The battery collider is a 48-sided convex cylinder, 49 mm long and 13.6 mm in
diameter, with flat ends. The engine's Cylinder collision approximation has
rounded capsule ends, so it is inappropriate for this terminal contact.
`OMNISIM_NEWTON_MULTICCD=1` enables multiple contact points for convex meshes.
The separate engine regression checks a flat cylinder settling on a table;
the opt-in leaves the engine's default behavior unchanged.

The spring variant uses `aloha_left_spring.urdf` and `aloha_right_spring.urdf`.
These preserve the original arm geometry, inertias and joints. Estimated
14 x 4 x 12 mm pads sit over the tapered jaw tips, at finger-local
`[0.078, +/-0.026, 0]` m. They are visible and have matching colliders.
The original prototype URDFs remain available separately.

Friction coefficients are estimated: 3 for the rubber jaw contacts, 0.3 for
battery/compartment contacts and 0.5 for the table. The world uses the CPU
Newton/MuJoCo solver, a 1 ms step, compound colliders, elliptic friction,
`newtonImpratio 10`, `newtonCondim 6` and five no-slip iterations.
Numerical contact gains are `newtonContactKe 160000` and
`newtonContactKd 800`; finger contact stiffness is overridden to 2000.
These numerical contact gains are separate from the 100 N/m physical spring.

## Control and evidence

The left arm braces the remote. The right arm uses `Motor.setPosition` and
physical finger contact to carry the battery. A standard-library URDF solver
converts Cartesian targets into motor commands. During insertion it uses
simulator-provided battery/gripper poses, remote pose and spring displacement.
That privileged state would need a real sensing/estimation system for hardware.

Compression feedback backs the tool away as slider travel runs short. After
the initial release, the empty fingertips push down and toward the spring;
the spring can finish seating just after the fingertips clear. Nothing welds
the battery to the gripper or writes its pose during the task. The two props
are repositioned during the initial 1.8 s setup before recording starts.

The video and pose/contact trace sample at 50 Hz. A separate contact audit
samples every 1 ms physics step. The success gate requires:

- Sustained bilateral airborne contact and at least 80 mm of transport.
- At least 100 ms of held, tilted contact with 5 mm or more compression.
- Seating while held, or a contacting fingertip press that lowers the battery
  at least 1 mm and is followed by seating within 0.5 s of the last press contact.
- Full-cylinder containment, floor contact, both terminal contacts, at least
  3 mm of residual spring compression, and a nearly horizontal battery after release.
- A settled final body and no sampled terminal/housing penetration over 1.5 mm,
  including the audit at every physics step. Missing measurements fail.

The 1.5 mm penetration ceiling is a numerical acceptance limit, not a claim
of sub-millimeter CAD accuracy. The stricter carry-slip quality check is
reported separately with a 10 mm limit.

## Run the experiment

Start a dedicated harness with these environment settings in PowerShell:

```powershell
$env:OMNISIM_NEWTON_FINGER_KE = '2000'
$env:OMNISIM_NEWTON_MULTICCD = '1'
.\omnisim.bat harness --port 6889 --supervisor-port 6890
```

Then run these commands sequentially from the repository root. Use a new
output directory for every run:

```sh
python projects/robots/trossen/aloha/replay.py --mode spring-insertion --view insertion --capture --harness http://127.0.0.1:6889 --out-dir outputs/aloha-spring
python projects/robots/trossen/aloha/replay.py --mode spring-insertion --view insertion --harness http://127.0.0.1:6889 --out-dir outputs/aloha-spring-repeat
python projects/robots/trossen/aloha/replay.py --mode spring-insertion --no-grip --fixed-targets outputs/aloha-spring/executed_targets.json --harness http://127.0.0.1:6889 --out-dir outputs/aloha-spring-no-grip
python projects/robots/trossen/aloha/probe_spring.py --case spring-law --harness http://127.0.0.1:6889 --out-dir outputs/spring-law
python projects/robots/trossen/aloha/probe_spring.py --case spring-law --no-spring --harness http://127.0.0.1:6889 --out-dir outputs/spring-disabled
python projects/robots/trossen/aloha/probe_spring.py --case seated --harness http://127.0.0.1:6889 --out-dir outputs/seated-retention
python projects/robots/trossen/aloha/probe_spring.py --case drop --harness http://127.0.0.1:6889 --out-dir outputs/gravity-drop
```

The no-grip control uses the successful run's saved motor targets, with only
the right fingers held open and object feedback disabled. The force probe
applies 0.75 N: the spring law predicts 6 mm of compression and a return to
its stop when the load is removed. The isolated seated/drop probes apply a
0.6 N upward force for 0.4 s, then check retention. They are mechanism tests,
not robot insertions.

Use the replay CLI for coordinated evidence and pass/fail checks. Opening
the spring world directly runs the motion with temporary controller outputs,
but does not certify a complete contact audit. Rebuild the saved targets with
`author_insertion.py` and the world/models with `build_spring_scene.py`.

The general negative-spring/positive-terminal topology is supported by
[Keystone's battery-contact drawings](https://www.keyelco.com/pdfs/M55p5.pdf).
This does not identify the particular remote or calibrate the estimates.
The [ALOHA project](https://tonyzhaozh.github.io/aloha/) and the
[saved dataset provenance](../../../../sim-to-real/aloha-battery/source/README.md)
identify the real reference. See [PROVENANCE.md](PROVENANCE.md) for robot assets
and licenses, and the [study archive](../../../../sim-to-real/aloha-battery/README.md)
for measured results and videos.
