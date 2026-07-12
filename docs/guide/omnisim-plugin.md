## OmniSim Plugin

OmniSim functionality can be extended with user-implemented plugins.

### Physics Plugin

A *physics* plugin offers the possibility to add custom ODE instructions to the default physics behavior of OmniSim.
For instance, it is possible to add or measure forces.
When adding forces, it is possible to simulate new types of environments or devices.
For example, a wind can be simulated as a constant unidirectional force applied to each object in the world and proportional to the size of the object.
The reactor of an airplane can be simulated by adding a force of varying intensity, etc.

OmniSim distribution comes with some implementations and usage examples for these plugins.
You will find more info on this topic in [OmniSim Reference Manual](../reference/physics-plugin.md).
