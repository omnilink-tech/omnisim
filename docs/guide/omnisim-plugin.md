## OmniSim Plugin

OmniSim functionality can be extended with user-implemented plugins.

The plugin types available today are attached to a robot rather than to the world:

- **Robot window** plugins, which draw a robot's own UI panel — see the [Robot Window Plugin](../reference/robot-window-plugin.md) chapter of the OmniSim Reference Manual.
- **Remote control** plugins, which route a controller's device calls to a real robot instead of the simulated one — see [Controller Plugin](controller-plugin.md).

> **Note**: the *physics* plugin — a user-compiled shared library that added custom ODE
instructions to the world's physics step (`webots_physics_init` / `_collide` / `_step` and the
rest of the callback set), named by the `WorldInfo.physics` field — was **removed** from OmniSim
together with the ODE backend, which was deleted on 2026-08-08 (commit `bdc02139`).
Its type vocabulary was ODE's own (`dBodyID`, `dGeomID`, `dJointID`), so it could not be carried
over to Newton, and there is **no Newton equivalent plugin API**.
The `WorldInfo.physics` field is still parsed so old worlds keep loading, but its value is
ignored and produces one parse-time warning.
To apply custom forces (wind, thrust, hydrodynamic drag, non-uniform friction), use a
`Supervisor` controller and the `wb_supervisor_node_add_force`/`add_torque` family instead.
