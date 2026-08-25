# Introduction

This manual contains the specification of the nodes and fields of the `.wbt` world description language used in OmniSim.
It also specifies the functions available to operate on these nodes from controller programs.

OmniSim is a fork of Webots and inherits its node set and controller API. The OmniSim nodes and APIs are open specifications which can be freely reused without authorisation, and OmniSim continues that tradition.
The API can be freely ported and adapted to operate on any robotics platform using the remote-control and/or cross-compilation frameworks.
This benefits the robotics community by improving interoperability between different robotics applications.

> ⚠️ **The physics engine is Newton/MuJoCo, and it is the only one.** ODE, the CPU rigid-body
> engine inherited from Webots, was deleted on 2026-08-08 (commit `bdc02139`). Several nodes and
> fields in this manual therefore describe things that no longer happen — each such page and field
> is marked in place. Two nodes are gone outright (`Fluid`, `ImmersionProperties`), as is the ODE
> physics-plugin API. For what the physics does and does not do today, and for the
> `WorldInfo.newton*` tuning fields that replaced ODE's, see
> [Physics (Newton)](../guide/newton-physics-backend.md).

## Sections

- [Nodes and Functions](nodes-and-functions.md)
