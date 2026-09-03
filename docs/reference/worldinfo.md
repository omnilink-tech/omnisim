## WorldInfo

```
WorldInfo {
  SFString title                          ""         # any string
  MFString info                           [ ]        # any string
  SFString window                         "<none>"   # any string
  SFFloat  gravity                        9.81       # [0, inf)
  SFFloat  CFM                            0.00001    # (0, inf)
  SFFloat  ERP                            0.2        # [0, 1]
  SFString physics                        "<none>"   # any string
  SFFloat  basicTimeStep                  32         # [1, inf)
  SFFloat  FPS                            60         # [1, inf)
  SFInt32  optimalThreadCount             1          # [1, inf)
  SFFloat  physicsDisableTime             1          # [0, inf)
  SFFloat  physicsDisableLinearThreshold  0.01       # [0, inf)
  SFFloat  physicsDisableAngularThreshold 0.01       # [0, inf)
  SFNode   defaultDamping                 NULL       # {Damping, PROTO}
  SFFloat  inkEvaporation                 0          # [0, inf)
  SFString coordinateSystem               "ENU"      # {"ENU", "NUE", "EUN"}
  SFString gpsCoordinateSystem            "local"    # {"WGS84", "local"}
  SFVec3f  gpsReference                   0 0 0      # any vector
  SFFloat  lineScale                      0.1        # [0, inf)
  SFFloat  dragForceScale                 30.0       # (0, inf)
  SFFloat  dragTorqueScale                5.0        # (0, inf)
  SFInt32  randomSeed                     0          # {-1, [0, inf)}
  MFNode   contactProperties              []         # {ContactProperties, PROTO}
  SFString broadphase                     "simple"   # {"simple", "sap", "quadtree", "auto"}
  SFString newtonSolver                   ""         # {"", "auto", "mujoco", "mujoco_warp"}
  SFInt32  newtonSubsteps                 1          # [1, inf)
  SFString newtonCone                     ""         # {"", "pyramidal", "elliptic"}
  SFFloat  newtonImpratio                 0          # {0, [1, inf)}
  SFInt32  newtonCondim                   0          # {0 = leave as built (== 3), 1, 3, 4, 6}
  SFInt32  newtonNoslipIterations         0          # {0 = off (MuJoCo stock), >=1 friction-only post-solve sweeps; CPU mj_step only}
  SFFloat  newtonGroundMu                 -1         # {negative = unset -> 1.0; 0 = FRICTIONLESS; (0, ~6]}
  SFFloat  newtonContactKe                0          # {0 = engine default 2500, (0, inf)}
  SFFloat  newtonContactKd                0          # {0 = engine default 100, (0, inf)}
  SFInt32  newtonIterations               0          # {0 = solver default, [1, inf)}
  SFInt32  newtonLsIterations             0          # {0 = solver default, [1, inf)}
  SFInt32  newtonNjmax                    0          # {0 = engine default 256, -1 = auto (undersized, see below), [1, inf)}
  SFInt32  newtonNconmax                  0          # {0 = engine default 256, -1 = auto (undersized, see below), [1, inf)}
  SFBool   newtonStatics                  FALSE      # {TRUE, FALSE} -- SCHEMA default only; the ENGINE default is ON (see below)
  SFBool   newtonRobotColliders           FALSE      # {TRUE, FALSE}
  SFBool   newtonCompoundColliders        FALSE      # {TRUE, FALSE}
  SFString defaultPhysicsBackend          ""         # {"", "ode", "newton"}
  SFString defaultRenderBackend           ""         # {"", "wren", "wgpu"}
}
```

> ⚠️ **SEVERAL OF THESE FIELDS NO LONGER DO ANYTHING.** ODE was deleted on 2026-08-08
> (commit `bdc02139`) and Newton/MuJoCo is the only physics backend. The following are still
> *declared* — removing a field turns every legacy world that sets it into a parse ERROR,
> which takes a headless run's exit code to 1 — but they are **not read**, and changing them
> has no effect:
>
> `CFM` · `ERP` · `physics` · `optimalThreadCount` · `physicsDisableTime` ·
> `physicsDisableLinearThreshold` · `physicsDisableAngularThreshold` · `broadphase` ·
> `contactProperties` · `defaultDamping`
>
> Each is marked in place below. For contact behaviour the Newton fields are
> `newtonGroundMu` / `newtonContactKe` / `newtonContactKd`. There is **no** replacement for
> body sleep (Newton has none), for a physics plugin (the feature was removed), or for
> damping. `defaultPhysicsBackend` still accepts `"ode"` but the value no longer selects a
> working solver — see its entry. Background: [Physics (Newton)](../guide/newton-physics-backend.md).

The [WorldInfo](#worldinfo) node provides general information on the simulated world:

- The `title` field should briefly describe the purpose of the world.

- The `info` field should give additional information, like the author who created the world, the date of creation and a description of the purpose of the world.
Several character strings can be used.

- The `window` field refers to a window plugin for the world.
This can be useful for having a supervisor window displaying information about each robot on a web simulation for example.

- The `gravity` field defines the gravitational acceleration along the vertical axis to be used in physics simulation.
The gravity is set by default to the gravity found on earth.
You should change it if you want to simulate rover robots on Mars, for example.

- ⚠️ The `ERP` field is **retired and not read**. It was ODE's *Error Reduction Parameter*, controlling
what proportion of a contact joint's error the solver fixed each step.
The field is still parsed so legacy worlds load; changing it has no effect.
The Newton/MuJoCo equivalent of contact stiffness/softness is `newtonContactKe` and `newtonContactKd`.

- ⚠️ The `CFM` field is **retired and not read**. It was ODE's *Constraint Force Mixing* parameter,
which together with `ERP` controlled contact spongyness. The field is still parsed so legacy worlds
load; changing it has no effect. Use `newtonContactKe` / `newtonContactKd` — and, for a heavy body
penetrating the ground, raise `newtonContactKe` rather than lowering anything.

- The `physics` field is **retired**. It used to name a user-compiled physics plugin (a shared library built against the ODE C API); that feature was removed from OmniSim together with ODE.
The field is still *parsed*, so old worlds keep loading, but its value is **ignored** and a world that sets it to anything other than `"<none>"` gets one parse-time warning.
Delete the field from your `WorldInfo` to silence the warning; to model hydrodynamic forces, wind or non-uniform friction, apply them from a `Supervisor` controller instead.

- The `basicTimeStep` field defines the duration of the simulation step executed by OmniSim.
It is a floating point value expressed in milliseconds where the minimum value is 1.
Setting this field to a high value will accelerate the simulation, but will decrease the accuracy and the stability, especially for physics computations and collision detection.
It is usually recommended to tune this value in order to find a suitable speed/accuracy trade-off.

- The `FPS` (frames per second) field represents the maximum rate at which the 3D display of the main window is refreshed in `Real-time` and `Run` mode.
It is particularly usefull to limit the refresh rate, in order to speed up simulations having a small `basicTimeStep` value.

- ⚠️ The `optimalThreadCount` field is **retired and not read**. It set the number of threads ODE
used for physics, clamped by `Number of threads` in the *Preferences*. The value is still resolved
and handed to `OmOdeContext::setNumberOfThreads`, which is now a stub that stores it and nothing
consumes it. Newton parallelism is a property of the solver, not this field: the CPU `mj_step` path
is single-threaded per world, and the way to use more hardware is either
`newtonSolver "mujoco_warp"` (GPU-batched, for training) or **multiple `omnisim-bin` processes**
in parallel, which is the supported shape for batch work.

- ⚠️ The `physicsDisableTime`, `physicsDisableLinearThreshold` and `physicsDisableAngularThreshold`
fields are **retired and not read**. They configured ODE's automatic body disabling ("body sleep"),
which put an idle Solid to sleep after `physicsDisableTime` seconds below the two velocity
thresholds, mapping onto `dBodySetAutoDisableTime` / `...LinearThreshold` / `...AngularThreshold`.
**Newton has no body sleep**, so there is no replacement and nothing to tune — `OmSolidMerger::setOdeAutoDisable()`
is now an empty function. The fields are still parsed so legacy worlds load.
One consequence worth knowing: an idle Newton body **does** generate contact points, so an empty
contact query is no longer explained away by sleep. The `?wake=1` parameter on the HTTP harness's
`/sim/contacts` existed for exactly that ODE behaviour and is now a no-op that still costs two steps.

- ⚠️ The `defaultDamping` field is **retired and not read**. It named a Damping (archived 2026-09-02, see [docs/ARCHIVE.md](../ARCHIVE.md)) node
supplying default linear/angular damping for every [Solid](solid.md), applied through ODE's
`dBodySetDamping`. `OmSolidMerger::setOdeDamping()` is now an empty function and damping is not
plumbed to Newton, so neither this field nor a per-`Physics` `damping` node has any effect. There is
no `WorldInfo`-level replacement; damp a body from its controller, or model the loss in the joint.

- If the `inkEvaporation` field is set to a non-null value, the colors of the ground textures will slowly turn to white.
This is useful on a white-textured ground in conjunction with a [Pen](pen.md) device, in order to have the track drawn by the [Pen](pen.md) device disappear progressively.
The `inkEvaporation` field should be a positive floating point value defining the speed of evaporation.
This evaporation process is a computationally expensive task, hence the ground textures are updated only every `WorldInfo.basicTimeStep` * `WorldInfo.displayRefresh` milliseconds (even in fast mode).
Also, it is recommended to use ground textures with low resolution to speed up this process.
As with the pen device, the modified ground textures can be seen only through infra-red distance sensors, and not through cameras (as the ground textures are not updated on the controller side).

- The `coordinateSystem` field indicates the [axis convention](https://en.wikipedia.org/wiki/Axes_conventions) of the global coordinate system, defining the cartesian and gravity directions.
Currently it supports "ENU" (default), "NUE" and "EUN".
"ENU" means East along the X-positive axis, North along the Y-positive axis and Up along the Z-positive axis.
It is the most widely used axis convention in robotics, including OmniSim and ROS.
"NUE" means North along the X-positive axis, Up along the Y-positive axis and East along the Z-positive axis.
"EUN" means East along the X-positive axis, Up along the Y-positive axis and North along the Z-positive axis.
It is similar to "NUE" but with the North and East inverted.
Changing the coordinate system will affect the return values of the [Accelerometer](accelerometer.md), [Altimeter](altimeter.md), [Compass](compass.md), [InertialUnit](inertialunit.md) and [GPS](gps.md) devices.

> ⚠️ **On the Newton backend this field was IGNORED until 2026-08-08, and the failure was silent.** The Newton builder was constructed with a hardcoded z-up axis and never read `coordinateSystem`, so every `NUE` / `EUN` (Y-up) world got two things wrong at once: `WorldInfo.gravity` is projected onto the builder's up vector, and the projection of a Y-up world's `(0, -g, 0)` onto `(0, 0, 1)` is **exactly zero** — so nothing fell (measured: a ball released at y=3 read y=3.000 at step 15360) — while the implicit ground plane took its **normal** from the same wrong axis and stood up as a **vertical wall** along the world's East axis (the same ball drifted to z=+384 m along it; in a world whose `DistanceSensor`s sit at up=0 the wall is exactly where they are and they all read 0.000000). ODE masked it as the fall-back backend, and a headless PASS cannot see it: 210 of the 719 worlds in this tree are `NUE`, none of them pins a backend, and the readiness sweep scored all 140 `tests/api` worlds PASS because they loaded and stepped and logged nothing. The field is now plumbed to `newton.ModelBuilder.up_axis` *before* the implicit ground plane is created (both the plane normal and the gravity direction are baked at add time, so no later moment works). `OMNISIM_NEWTON_COORD_SYSTEM` is value-parsed and **default on** — `=0`/`false`/`off` pins the builder back to the historical z-up so the pre-fix physics can be bisected, and re-running a `NUE` world under it emits a warning saying so. `ENU` is unaffected either way: it resolves to the z-up the constructor already had. Pinned by `tests/test_newton_coordinate_system.py`; the axis the builder actually used is recorded per world in `.build_tmp/newton_solver.log` (`up_axis=... up_vector=... gravity_scalar=...`).

- The `gpsCoordinateSystem` field is used to indicate in which coordinate system are expressed the coordinates returned by the GPS devices.
If the value is `WGS84` the World Geodetic System `WGS84` and an Universal Transverse Mercatoris projection are used to compute the latitude, altitude and longitude from the cartesian coordinates of the device, otherwise, if the value is `local` the cartesian coordinates of the device are directly returned as is.

- The `gpsReference` field is used to specify the GPS reference point.
The value should be set in meters for each coordinates if the `gpsCoordinateSystem` is `local`, and in decimal degree and meters for respectively the latitude, longitude and height if the `gpsCoordinateSystem` is `WGS84`.
The reference vector is simply added to the position of the GPS device in case of `local` coordinates.
In case of `WGS84` coordinates, the latitude, longitude and altitude should correspond to the latitude, longitude and altitude of the center of the cartesian coordinate system of OmniSim.

- The `lineScale` field allows the user to control the size of the optionally rendered arbitrary-sized lines or objects such as the connector and the hinge axes, the local coordinate systems and centers of mass of solid nodes, the rays of light sensors, the point light representations, the camera frustums, or the offsets used for drawing bounding objects and the laser beam.
Increasing the `lineScale` value can help in case of depth fighting problems between the red spot of a laser beam and the detected object.
The `lineScale` value is also used to define the step of the zoom using the mouse wheel: increasing the `lineScale` value will make the mouse wheel zoom faster.
The value of this field is somehow arbitrary, but setting this value equal to the average size of a robot (expressed in meter) is likely to be a good initial choice.

- ⚠️ The `dragForceScale` and `dragTorqueScale` fields are **retired in effect: the GUI drag applies no force or torque at all.** They set the order of magnitude of the [force](../guide/the-3d-window.md#applying-a-force-to-a-solid-object-with-physics) and [torque](../guide/the-3d-window.md#applying-a-torque-to-a-solid-object-with-physics) the 3D view's alt-drag was meant to put on a [Solid](solid.md) (*F* [N] = `dragForceScale` × `Solid.mass` × *d*<sup>3</sup> and *T* [Nm] = `dragTorqueScale` × `Solid.mass` × *d*<sup>3</sup>, *d* being the drag vector in metres). The value still sizes the on-screen arrow and the newton figure the view prints — but `OmSolid::addForceAtPosition` and `OmSolid::addTorque`, the two functions the drag calls, are **empty functions** since the ODE deletion, so nothing moves. This is the one place in this file where a number you read on screen does not correspond to a force in the simulation.

  The supervisor's `add_force` / `add_torque` are **not** affected: they route through `OmSolid::applyExternalForceNewton`, which is wired up. Use those when you need to push a body.

- The `randomSeed` field defines the seed of the internal random number generator used by OmniSim.
This seed has an influence, for example, on the noise pattern generated by some sensors.
The value of the seed should be either non-negative or -1, if the value is -1 a time-based seed is then used.
Using a time-based seed makes simulations non-reproducible.

- ⚠️ The `contactProperties` field is **retired and not read.** It held
ContactProperties (archived 2026-09-02, see [docs/ARCHIVE.md](../ARCHIVE.md)) nodes describing the behaviour when [Solid](solid.md)
nodes collide, and it was an **ODE-path** declaration throughout.

  > **`coulombFriction` in particular is not consulted.** Friction on the Newton path comes from
  > `newtonGroundMu` (or `OMNISIM_NEWTON_GROUND_MU`), whose default is **1.0** regardless of what
  > this field says — so a world declaring `coulombFriction [ 5 ]` gets 1.0. This is measured, and
  > it cost real time twice: once in our own findings, and once by an agent that rediscovered it
  > from scratch while trying to make a two-finger grasp hold. **Set `newtonGroundMu`.**
  > The old escape hatch — "pin `physicsBackend "ode"` on the Solids you are tuning" — **no longer
  > works and must not be used**: ODE was deleted on 2026-08-08, and a Solid pinned to `"ode"` is
  > not simulated at all.
  >
  > One narrow exception exists so that a world which *deliberately* chose the Newton path is not
  > ignored: if the world pins `defaultPhysicsBackend "newton"` (or
  > `OMNISIM_NEWTON_BRIDGE_CONTACT_PROPERTIES` is set), the first positive `coulombFriction` value
  > is **bridged** to Newton's ground friction and the engine logs that it did so. An ordinary
  > world is never re-frictioned this way — 240 live worlds declare `coulombFriction` and were
  > tuned under an effective mu of 1.0, so silently adopting their values would change the physics
  > of every one of them.

The following fields are OmniSim-specific additions on top of the upstream node set:

- ⚠️ The `broadphase` field is **retired and not read**. It selected ODE's broadphase collision
algorithm (`"simple"` brute force, `"sap"` sweep-and-prune, `"quadtree"`, `"auto"`); the ODE-context
side was never completed even while ODE shipped, and ODE is now gone. Newton's broadphase is the
solver's own and is not configurable from the world file.

- The `newtonSolver` field selects the Newton solver **profile**. The solver itself is **SolverMuJoCo, always** — XPBD was **removed on 2026-08-07**, one day after the default flipped away from it: zero of 725 tracked worlds ever selected it, newton's own docs say it does not operate on articulations, it measured *slower* than CPU mujoco at every scale tried (0.11×–0.67×, interleaved A/B), and on the shipped 10-Husky swarm it drove the robots 0.97 m where mujoco and ODE — measured while ODE still shipped — agreed on 2.4 m within 1% ([the record](../developer/ode-retirement-campaign.md)). What the field now selects is CPU vs GPU: `""` / `"auto"` / `"mujoco"` = the reference CPU `mj_step` (deterministic — bitwise reproducible at 336 contacts); `"mujoco_warp"` = the same solver batched on the GPU — worthwhile **only for parallel training** (at nworld=1 it measured 9.06× slower than CPU), needs a CUDA GPU plus an even `newtonSubsteps`, and is not run-to-run reproducible. A world still declaring `"xpbd"` gets the parser's invalid-value warning and the default. If `SolverMuJoCo` fails to construct, the world load **fails loudly** — there is nothing left to substitute, since ODE was deleted on 2026-08-08.

- The `newtonSubsteps` field sets the number of Newton solver sub-steps per physics tick. `1` (default) does one solver step per tick, preserving the exact physics and determinism of existing worlds and trained RL policies. `N > 1` splits each tick into `N` sub-steps of `dt/N`, re-colliding each — it shrinks integrator truncation and helps high closing-speed contact, but it is **not a free win**: the same OmniBench sweep that flipped lane-1 T4 energy and T7 angular-momentum in Newton's favour at N=4 also degraded T2 sliding accuracy **12×**, most likely because MuJoCo's `solref` / `solimp` are not rescaled. The RL deploy stack runs N=8. On `newtonSolver "mujoco_warp"` the CUDA-graph replay requires an **even** N. The `OMNISIM_NEWTON_SUBSTEPS` env var still overrides this field. Applies to every world (Newton is the only backend).

> ⚠ **`newtonGroundMu`'s unset sentinel is NEGATIVE, and this changed on 2026-08-09.**
> It used to be `0`, which collided with a legal physical value: **μ = 0 is a
> frictionless world**, and a world that wanted one could not say so in its own
> file — only through `OMNISIM_NEWTON_GROUND_MU`, which put the physics in the
> shell instead of the scene. The field now defaults to `-1`, anything negative
> means unset (engine default 1.0), and `0` declares frictionless. Fixed at all
> four sites that had baked in the old rule: this schema, `OmSolid`'s
> `resolvedNewtonGroundMu` (the single resolution point), the C++ prefs gate, and
> the embedded runtime's `set_contact_solver_params`. Verified live: undeclared →
> 1.0, `newtonGroundMu 0` → 0.0, `0.5` → 0.5, `2` → 2.0.
>
> **A world written with `newtonGroundMu 0` behaves differently on a binary older
> than that fix**, where 0 still reads as unset → 1.0. That is a genuine version
> split; the migration writes an explanatory comment beside the value for exactly
> this reason.

- The `newtonCone` field selects the MuJoCo friction-cone type for Newton worlds running `SolverMuJoCo` (`newtonSolver "mujoco"` / `"mujoco_warp"`). `""` (default) keeps MuJoCo stock (pyramidal). `"elliptic"` removes the inscribed-pyramid creep near the friction-cone boundary (millimetre-scale pseudo-slip below the static-friction transition angle); pair it with `newtonImpratio 10` for accurate static-friction hold. The `OMNISIM_NEWTON_CONE` env var still overrides. The global default is deliberately not flipped: it changes contact physics for every Newton world (a flip is a follow-up gated on champion re-verification). Applies to every world (Newton/`SolverMuJoCo` is the only backend).

- The `newtonImpratio` field sets MuJoCo's `impratio` (ratio of frictional-to-normal constraint impedance) for Newton worlds running `SolverMuJoCo`. `0` (default) means unset (MuJoCo stock `1`). Raise it (e.g. `10`) together with `newtonCone "elliptic"` to stiffen tangential contact. The `OMNISIM_NEWTON_IMPRATIO` env var still overrides. Applies to every world (Newton/`SolverMuJoCo` is the only backend).

- The `newtonCondim` field sets MuJoCo's contact dimensionality (`mjModel.geom_condim`). `0` (default) leaves the model exactly as newton built it, which — **measured on every world dumped so far** — is `3` on every geom. `1` = frictionless (`mu` is not consulted at all), `3` = sliding friction only, `4` = **+ torsional**, `6` = + torsional and rolling. ⚠ At `3` the torsional and rolling coefficients newton writes into `geom_friction[1]` and `[2]` are **never read**, so a part pinched between two gripper pads is free to spin about the contact normal at zero cost and a cylinder rolls forever on flat ground. Measured on [`omniarm6_real_pick_place.omniworld`](../../projects/samples/demos/worlds/flagship/omniarm6_real_pick_place.omniworld) with the pads centred on the part: at `3` the block rotates inside the gripper during a 0.47 m carry and levers the pads from 24 mm apart to 41 mm, arriving tipped on 3 of 3 runs; at `4` the same run keeps two contacts per pad and places it standing. On the off-centre variant of the same grasp, carry contacts went 2/2 → 3/3 and peak pad penetration 1.12 → 0.96 mm. ⚠ **It is GLOBAL, not per-shape.** newton *does* expose a per-shape `mujoco:condim`, but only through `SolverMuJoCo.register_custom_attributes(builder)` which the engine does not call, and on the CPU `mj_step` path a post-finalize per-shape write would not reach `mj_model` anyway; so this patches `geom_condim` on the compiled model after solver construction (the same route `OMNISIM_NEWTON_ROLL_MU` takes) and every geom gets the same value. Setting `4` or `6` also floors `geom_friction[1]` at `0.005` so the added dimension is not a no-op. Verify with `OMNISIM_NEWTON_DUMP_MJMODEL=<path>` — the per-geom lines carry `condim=`. The `OMNISIM_NEWTON_CONDIM` env var still overrides. Applies to every world (Newton/`SolverMuJoCo` is the only backend).

- The `newtonNoslipIterations` field sets MuJoCo's `mjOption.noslip_iterations` for Newton worlds running `SolverMuJoCo` on the **CPU** `mj_step` path. `0` (default) is also MuJoCo's own stock value, so every existing world is byte-identical. It is a Gauss-Seidel pass over the **friction constraints only**, run after the main solve, and what it removes is the tangential **drift** a soft friction constraint accumulates under a sustained load — the failure where a pinch holds its commanded normal force and the part still creeps out. It is therefore a different remedy from more force, a firmer contact or more `newtonIterations`, none of which address residual drift. ⚠ **CPU only.** `mujoco_warp`'s `Option` struct has no noslip field at all and its `put_model` **raises** on a non-zero value, so the engine applies this to `mj_model` after solver construction and, on `newtonSolver "mujoco_warp"`, logs one WARNING and ignores it rather than failing the build. Verify it landed: the engine's world-finalise line and the `.newton.json` backend sidecar carry `+noslip=<n>` in their `solver` string, and `OMNISIM_NEWTON_DUMP_MJMODEL` prints `noslip=` on its `opt` line. ⚠ **It is not a universal grasp fix.** Measured on ladder0 rung 8 (a 0.2 kg part pinched at 3 N per pad at µ = 3, i.e. 9× the Coulomb bound, lifted 0.15 m and carried 0.45 m): on OmniSim's contact parameterisation it moved `carry_rel` from 0.4747 m to 0.4796 m at 5 iterations — the payload was dropped either way — while `newtonCone "elliptic"` + `newtonImpratio 10` took the same scene to 0.0026 m. On bare MuJoCo's stock `solref`, the same pass fixes the same scene outright. Try it when a grasp creeps, do not assume it. The `OMNISIM_NEWTON_NOSLIP` env var still overrides and is **value-parsed**, so `=0` forces the pass off for a world that declares the field. Applies to every world (Newton/`SolverMuJoCo` is the only backend).

- The `newtonNjmax` and `newtonNconmax` fields set MuJoCo's constraint-row (`njmax`) and contact (`nconmax`) buffer caps for Newton worlds running `SolverMuJoCo`. `0` (default) means unset, which keeps the engine's built-in `256` — so every existing world is byte-identical. A positive value raises the cap. **Multi-robot worlds must raise these.** mujoco_warp does not read the MJCF `<size njmax>` tag and its auto-estimate is far too small, so the engine pins 256 — which is a *fleet* ceiling, not just a legged-contact floor. Measured on a flat-ground 10-Husky world (`newtonSolver "mujoco_warp"`, pyramidal cone): **each 4-wheel-drive Husky rests on 8 wheel-ground contacts = 32 constraint rows** (pyramidal contributes `2 × (condim − 1) = 4` rows per contact), so ten of them peak at **`nefc` 320 / `ncon` 80** — over the 256 default from tick 4 onwards. Rule of thumb: **N wheeled robots ≈ 32·N constraint rows**, i.e. the 256 default runs out at 9 robots (8 × 32 = 256 sits exactly on the boundary). Overflow is *silent and damaging*: mujoco_warp truncates the constraint vector, degrading contact solving, and its only warning is a `wp.printf` from inside the solver kernel — which floods (one line per tick) where stdout is captured and is discarded entirely on Windows, where `omnisim-bin.exe` is a GUI-subsystem binary. Detection is therefore **not optional**: whenever the `"mujoco_warp"` path is live the engine samples `nefc`/`ncon` host-side every 30 ticks and, on the first overflow, emits **one** `[OmNewtonBackend] CONSTRAINT BUFFER OVERFLOW ...` **warning into the engine log** — the log `run-headless` parses and the Newton verdict sidecar sits beside — naming the observed peaks against the caps mujoco_warp actually allocated. One warning per world, never per-tick spam. `OMNISIM_NEWTON_CONSTRAINT_STATS=<every N steps>` additionally traces *every* new peak into `.build_tmp/newton_solver.log` and sets the sampling cadence; `=-1` turns the watch off. (The CPU `newtonSolver "mujoco"` path is not watched and does not need to be — MuJoCo-C grows its `efc` arrays dynamically, so there is no cap to overflow.)

- ⚠️ **`-1` ("auto") is not "sized for this world" — it is systematically *undersized*, and usually worse than the `0` default.** newton delegates the sizing to mujoco_warp's `_default_njmax` / `_default_nconmax` (`mujoco_warp/_src/io.py`), which are computed from a **single `mj_forward` at t=0** and, for any world with no heightfield, flex or SDF geom — i.e. every arena world in this tree — collapse to a **64** floor. On the 10-robot fleet world above that is ~64 against a 320 peak: **~80% of constraint rows dropped**, with exactly the same silence as any other overflow. It is retained (and still resolves the same way, so no existing world moves) only for scenes whose contact set is already complete at t=0, and **setting it now logs a warning at world build**. Prefer `0`, or a positive value with headroom. Do **not** size to the measured peak either: `newtonNjmax 320` on that fleet puts the buffer exactly at capacity and moves results by **8.81 m** versus 512/2048/4096, which agree with each other to ~1e-4 (see [determinism-scope.md](../benchmarks/determinism-scope.md), §3). The `OMNISIM_NEWTON_NJMAX` / `OMNISIM_NEWTON_NCONMAX` env vars still override these fields. Applies to every world (Newton/`SolverMuJoCo` is the only backend).

- The `newtonStatics` field controls whether a world's top-level static colliders (those with a `boundingObject` and no joint parent) register as mass-zero Newton bodies so dynamic Newton bodies collide against them. **Statics are ON by default since 2026-08-07** — the schema default is still `FALSE`, but an unset/`FALSE` field now means "engine default (on)", while `TRUE` *pins* statics on so that not even the revert hatch can take them away. The old default left every static collider intangible: a ball dropped onto a floor topped at z=0.55 fell through and settled at z=0.0996 on the implicit z=0 plane. `OMNISIM_NEWTON_STATICS` is value-parsed: `=0`/`false`/`off` reverts to the pre-flip behaviour for the run (except in worlds declaring `TRUE`), anything else forces on. Pinned by `tests/test_newton_static_floor_collides.py`. Applies to every world (Newton is the only backend). ⚠️ **The `z=0.0996` above is HISTORY, not what the revert hatch does today.** That landing depended on the implicit z=0 plane being added unconditionally, which stopped on 2026-08-12 (see `OMNISIM_NEWTON_GROUND_PLANE` below). Re-measured 2026-08-13 on `tests/physics/worlds/gravity_rest_height.omniworld` (whose floor is a `Box`, so there is no dropped `Plane` to substitute for): under `OMNISIM_NEWTON_STATICS=0` the faller no longer lands at all — z=-3.039 at step 240, z=-287.959 at step 1920, still accelerating. The smoke gate is unaffected; only the number is. Set `OMNISIM_NEWTON_GROUND_PLANE=1` alongside it to reproduce the historical z=0.0996 control exactly.

- **`OMNISIM_NEWTON_GROUND_PLANE`** (env var only — there is no `WorldInfo` field) controls Newton's **implicit ground plane**. ⚠️ **Until 2026-08-12 this plane was added UNCONDITIONALLY**, giving every Newton world an undeclared, infinite collision surface at up-axis 0 that appears in no world file and in no scene tree — which is why a world whose floor sits at z=0 could not tell a working collider from a broken one, and why `run-headless --fail-on-runaway` could not see the AgentBench C2 fall-through case: with the floor's `boundingObject` removed the box did not run away, it rested quietly at **z=0.099892** on a surface no line of the world declares. It is now a **declared substitution**: added at finalize **if and only if** the world declared a `Plane` collider that had to be dropped (newton's MuJoCo converter cannot build a `Plane` attached to our weld-pinned statics), and the choice is logged either way — `[OmNewtonBackend] implicit ground plane: ADDED|not added (<reason>)`. A world that declares no static collision surface at all now genuinely has none, so a body with nothing under it falls. Value-parsed: `=1` restores the unconditional plane exactly (the pre-2026-08-12 build, for a bisect), `=0` refuses it even for the substitution case. Measured by OmniBench lane 4's `phenomenon.implicit_ground_plane` row, which flipped `works → absent` on this change (sphere falls to **z=-75.56** in a world declaring no ground).

- The `newtonRobotColliders` field, when `TRUE`, gives robot-wrapper bodies (e.g. a `URDFRobot` chassis [Solid](solid.md)) their own `boundingObject` as a Newton collider, so the robot body — not just its wheels or feet — collides with scene geometry. `FALSE` (default) uses wheel/foot-only collision, avoiding a chassis envelope that would pin the body and starve the wheels of load. The `OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE` env var still forces it on. Applies to every world (Newton is the only backend).

- The `newtonCompoundColliders` field, when `TRUE`, registers every collider in a compound `boundingObject` (a `Group` of offset primitives on one rigid body — e.g. a movable bin's floor plus four walls) as its own Newton shape, instead of only the first child. `FALSE` (default) is first-child-only, keeping existing worlds' physics byte-for-byte unchanged. The `OMNISIM_NEWTON_COMPOUND_COLLIDERS` env var still forces it on. Applies to every world (Newton is the only backend).

- The `defaultPhysicsBackend` field sets a world-level default physics backend. When set, it supplies the backend for any [Solid](solid.md) still on the `"auto"` default with no explicit ancestor choice, letting you pin a whole world without editing every Solid. `""` (default) is inert; an explicit per-Solid `physicsBackend` still wins.

  > ⚠️ **`"ode"` still parses but no longer selects a working solver.** ODE was deleted on
  > 2026-08-08 (`bdc02139`). Reading the engine source: `OmOdeBackend::isAvailable()` is now
  > `false` and every one of its operations returns `-1`, and
  > `OmSolid::flushPendingNewtonRegistrations` **skips** any Solid whose effective backend resolves
  > to `"ode"` — so such a Solid is registered with no solver and **is not simulated**. The value
  > remains accepted only because an *undeclared* field is a parse ERROR that would break every
  > legacy world that sets it. **The engine now warns once per such Solid** ("This Solid asks for
  > physicsBackend \"ode\", which no longer selects a physics engine…", `OmSolid.cpp`) — but a log
  > from a binary older than 2026-08-08 will not carry it, so on an older run a frozen body or a
  > prop that never falls is still worth checking for `"ode"` before you suspect anything else.
  >
  > `""` (leave it out) or `"newton"` are the only values you should write. Setting
  > `defaultPhysicsBackend "newton"` additionally opts the world into having its
  > `contactProperties.coulombFriction` bridged to Newton's ground friction — see
  > `contactProperties` above.

  Background: [Physics (Newton)](../guide/newton-physics-backend.md) and the [Newton runtime bundle](../developer/newton-runtime-bundle.md).

- The `defaultRenderBackend` field sets a world-level default renderer (`""`, `"wren"`, or `"wgpu"`). When set, it supplies the backend for any `Viewpoint`/`Camera` whose `renderBackend` is empty or `"auto"`. `""` (default) is inert; an explicit per-node `renderBackend` still wins. See the [default-flip plan](../developer/default-flip-plan.md).


## Newton contact and solver fields

Five fields that were reachable **only** through `OMNISIM_NEWTON_*` environment
variables until 2026-08-02. That gap meant a `.wbt` was not a complete
description of its own physics: a tuned two-finger friction grasp, written out
and handed to somebody else, came back with default friction and a soft contact
and did not hold. Four of the five default to `0` = *not declared*, which keeps
the engine's built-in value and every existing world byte-identical, and the
matching environment variable still overrides the field.

⚠️ **`newtonGroundMu` is the exception: its unset sentinel is `-1`, not `0`.**
`0` is a legal declaration meaning *frictionless* — see the warning above. Read
the "default when unset" column below as "negative" for that row and "0" for
every other.

| field | default when unset | env override | what it is for |
|---|---|---|---|
| `newtonGroundMu` | 1.0 (unset = **negative**; `0` means frictionless) | `OMNISIM_NEWTON_GROUND_MU` | tangential friction. **This, not `contactProperties`, is friction on the Newton path.** Measured: sphere feet slide ~1 m while merely standing at 1.0 and plant inside 4 cm at 2.0; a two-finger pinch needed 3. Above ~6 the floor destabilises even at `basicTimeStep 8`. |
| `newtonContactKe` | 2500 | `OMNISIM_NEWTON_CONTACT_KE` | contact stiffness. Raise (8000 measured) so fingers stop **at** an object's face instead of over-penetrating and squirting it out; lower to soften a dense layer of small parts. |
| `newtonContactKd` | 100 | `OMNISIM_NEWTON_CONTACT_KD` | contact damping, paired with `newtonContactKe`. |
| `newtonIterations` | solver's own | `OMNISIM_NEWTON_ITERS` | MuJoCo solver iterations. Raise for frictional contact that must **hold** rather than creep (150 measured). |
| `newtonLsIterations` | solver's own | `OMNISIM_NEWTON_LS_ITERS` | line-search iterations, paired with the above (50 measured alongside 150). |

All five apply to every world — Newton/`SolverMuJoCo` is the only backend, on both the
CPU (`newtonSolver ""` / `"mujoco"`) and GPU (`"mujoco_warp"`) paths.
