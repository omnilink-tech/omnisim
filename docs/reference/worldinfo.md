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
  SFString newtonSolver                   ""         # {"", "auto", "xpbd", "mujoco", "mujoco_warp"}
  SFInt32  newtonSubsteps                 1          # [1, inf)
  SFBool   newtonStatics                  FALSE      # {TRUE, FALSE}
  SFBool   newtonRobotColliders           FALSE      # {TRUE, FALSE}
  SFBool   newtonCompoundColliders        FALSE      # {TRUE, FALSE}
  SFString defaultPhysicsBackend          ""         # {"", "ode", "newton"}
  SFString defaultRenderBackend           ""         # {"", "wren", "wgpu"}
}
```

The [WorldInfo](#worldinfo) node provides general information on the simulated world:

- The `title` field should briefly describe the purpose of the world.

- The `info` field should give additional information, like the author who created the world, the date of creation and a description of the purpose of the world.
Several character strings can be used.

- The `window` field refers to a window plugin for the world.
This can be useful for having a supervisor window displaying information about each robot on a web simulation for example.

- The `gravity` field defines the gravitational acceleration along the vertical axis to be used in physics simulation.
The gravity is set by default to the gravity found on earth.
You should change it if you want to simulate rover robots on Mars, for example.

- The `ERP` field defines the *Error Reduction Parameter* use by ODE to manage contacts joints.
This applies by default to all contact joints, except those whose contact properties are defined in a [ContactProperties](contactproperties.md) node.
The ERP specifies what proportion of the contact joint error will be fixed during the next simulation step.
If ERP=0 then no correcting force is applied and the bodies will eventually drift apart as the simulation proceeds.
If ERP=1 then the simulation will attempt to fix all joint error during the next time step.
However, setting ERP=1 is not recommended, as the joint error will not be completely fixed due to various internal approximations.
A value of ERP=0.1 to 0.8 is recommended (0.2 is the default).

- The `CFM` field defines the *Constraint Force Mixing* use by ODE to manage contacts joints.
This applies by default to all contact joints, except those whose contact properties are defined in a [ContactProperties](contactproperties.md) node.
Along with the ERP, the CFM controls the spongyness and springyness of the contact joint.
If a simulation includes heavy masses, then decreasing the CFM value for contacts will prevent heavy objects from penetrating the ground.
CFM should be strictly positive, it cannot be set to 0.
If CFM is close to zero, the constraint will be hard.
If CFM is large, it will be possible to violate the constraint by *pushing on it* (for example, for contact constraints by forcing the two contacting objects together).
In other words the constraint will be soft, and the softness will increase as CFM increases.
What is actually happening here is that the constraint is allowed to be violated by an amount proportional to CFM times the restoring force that is needed to enforce the constraint (see ODE documentation for more details).

- The `physics` field refers to a physics plugin which allows the user to program custom physics effects using the ODE API.
See [this chapter](physics-plugin.md) for a description on how to set up a physics plugin.
This is especially useful for modeling hydrodynamic forces, wind, non-uniform friction, etc.

- The `basicTimeStep` field defines the duration of the simulation step executed by OmniSim.
It is a floating point value expressed in milliseconds where the minimum value is 1.
Setting this field to a high value will accelerate the simulation, but will decrease the accuracy and the stability, especially for physics computations and collision detection.
It is usually recommended to tune this value in order to find a suitable speed/accuracy trade-off.

- The `FPS` (frames per second) field represents the maximum rate at which the 3D display of the main window is refreshed in `Real-time` and `Run` mode.
It is particularly usefull to limit the refresh rate, in order to speed up simulations having a small `basicTimeStep` value.

- The `optimalThreadCount` defines the actual number of threads used for the computation of the physics of the world.
Note that the value of this field can not be higher than the value set for `Number of threads` in the *Preferences*.
Changing the value of this field can have a non-negligeable impact on the speed of the simulation.
For simulations involving several robots physically independent from each other, setting a value greater than 1 can significantly improve the speed of simulation.
In other cases, it may however reduce the simulation speed due to the overhead of the multi-threading.
Note that using more than 1 thread can result in non replicable simulations.

- The `physicsDisableTime` determines the amount of simulation time (in seconds) before the idle solids are automatically disabled from the physics computation.
Set this to zero to disable solids as soon as they become idle.
This field matchs directly with the ODE's `dBodySetAutoDisableTime` function.
This feature can improve significantly the speed of the simulation if the solids are static most of the time.
The solids are enabled again after any interaction (collision, movement, ...).

- The `physicsDisableLinearThreshold` determines the solid's linear velocity threshold (in meter/seconds) for automatic disabling.
The body's linear velocity magnitude must be less than this threshold for it to be considered idle.
This field is only useful if `physicsDisableTime` is greater than zero.
This field matchs directly with the ODE's `dBodySetAutoDisableLinearThreshold` function.

- The `physicsDisableAngularThreshold` determines the solid's angular velocity threshold (in radian/seconds) for automatic disabling.
The body's angular velocity magnitude must be less than this threshold for it to be considered idle.
This field is only useful if `physicsDisableTime` is greater than zero.
This field matchs directly with the `dBodySetAutoDisableAngularThreshold` ODE function.

- The `defaultDamping` field allows to specifiy a [Damping](damping.md) node that defines the default damping parameters that must be applied to each [Solid](solid.md) in the simulation.

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

- The `dragForceScale` field allows the user to set the order of magnitude of the [force applied](../guide/the-3d-window.md#applying-a-force-to-a-solid-object-with-physics) to a [Solid](solid.md) inside the OmniSim interface. The force is computed as follows: *F* [N] = `dragForceScale` * `Solid.mass` * *d*<sup>3</sup>, where *d* corresponds to the vector created by dragging the mouse (in meters).

- The `dragTorqueScale` field allows the user to set the order of magnitude of the [torque applied](../guide/the-3d-window.md#applying-a-torque-to-a-solid-object-with-physics) to a [Solid](solid.md) inside the OmniSim interface. The torque is computed as follows: *T* [Nm] = `dragTorqueScale` * `Solid.mass` * *d*<sup>3</sup>, where *d* corresponds to the vector created by dragging the mouse (in meters).

- The `randomSeed` field defines the seed of the internal random number generator used by OmniSim.
This seed has an influence, for example, on the noise pattern generated by some sensors.
The value of the seed should be either non-negative or -1, if the value is -1 a time-based seed is then used.
Using a time-based seed makes simulations non-reproducible.

- The `contactProperties` field allows to specifiy a number of [ContactProperties](contactproperties.md) nodes that define the behavior when [Solid](solid.md) nodes collide.

The following fields are OmniSim-specific additions on top of the upstream node set:

- The `broadphase` field selects the ODE broadphase collision algorithm. `"simple"` (default) is the brute-force *O(N²)* pass; `"sap"` is sweep-and-prune (good for clustered worlds); `"quadtree"` suits large outdoor scenes; `"auto"` picks `quadtree` when the world AABB exceeds 500 m on either horizontal axis, otherwise `sap`.

- The `newtonSolver` field selects the solver used by the Newton physics backend. `""` / `"auto"` / `"xpbd"` use the GPU XPBD solver (the default performance path); `"mujoco"` uses the CPU `mj_step` solver (robust frictional contact, required for friction pinch grasps that XPBD cannot hold); `"mujoco_warp"` runs that solver on the GPU and needs a CUDA GPU plus an even `newtonSubsteps`. Ignored unless a [Solid](solid.md) resolves to the Newton backend.

- The `newtonSubsteps` field sets the number of Newton solver sub-steps per physics tick. `1` (default) does one solver step per tick, preserving the exact physics and determinism of existing worlds and trained RL policies. `N > 1` splits each tick into `N` sub-steps of `dt/N`, re-colliding each — required for high closing-speed contacts where the positional solve would otherwise diverge. The `OMNISIM_NEWTON_SUBSTEPS` env var still overrides this field. Ignored on ODE-backed Solids.

- The `newtonStatics` field, when `TRUE`, registers a world's top-level static colliders (those with a `boundingObject` and no joint parent) as mass-zero Newton bodies so dynamic Newton bodies collide against them. `FALSE` (default) keeps statics on the ODE side. The `OMNISIM_NEWTON_STATICS` env var still forces it on. Ignored on ODE-backed Solids.

- The `newtonRobotColliders` field, when `TRUE`, gives robot-wrapper bodies (e.g. a `URDFRobot` chassis [Solid](solid.md)) their own `boundingObject` as a Newton collider, so the robot body — not just its wheels or feet — collides with scene geometry. `FALSE` (default) uses wheel/foot-only collision, avoiding a chassis envelope that would pin the body and starve the wheels of load. The `OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE` env var still forces it on. Ignored on ODE-backed Solids.

- The `newtonCompoundColliders` field, when `TRUE`, registers every collider in a compound `boundingObject` (a `Group` of offset primitives on one rigid body — e.g. a movable bin's floor plus four walls) as its own Newton shape, instead of only the first child. `FALSE` (default) is first-child-only, keeping existing worlds' physics byte-for-byte unchanged. The `OMNISIM_NEWTON_COMPOUND_COLLIDERS` env var still forces it on. Ignored on ODE-backed Solids.

- The `defaultPhysicsBackend` field sets a world-level default physics backend (`""`, `"ode"`, or `"newton"`). When set, it supplies the backend for any [Solid](solid.md) still on the `"auto"` default with no explicit ancestor choice, letting you pin a whole world without editing every Solid. `""` (default) is inert; an explicit per-Solid `physicsBackend` still wins. See the [default-flip plan](../developer/default-flip-plan.md) and the [Newton runtime bundle](../developer/newton-runtime-bundle.md) docs.

- The `defaultRenderBackend` field sets a world-level default renderer (`""`, `"wren"`, or `"wgpu"`). When set, it supplies the backend for any `Viewpoint`/`Camera` whose `renderBackend` is empty or `"auto"`. `""` (default) is inert; an explicit per-node `renderBackend` still wins. See the [default-flip plan](../developer/default-flip-plan.md).
