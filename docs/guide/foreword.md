## Foreword

OmniSim is a 3D robotics simulator purpose-built for agentic development and control. The guiding principle is simple: **you talk to it; you don't configure it.** Claude Code (or another coding agent) builds and iterates on the simulator and its worlds; OmniLink (or another runtime client) drives them.

OmniSim is a fork of [Webots](https://github.com/cyberbotics/webots), an open-source 3D mobile robot simulator originally developed as a research tool to investigate control algorithms in mobile robotics. Webots has been released under the [Apache 2.0 license](https://www.apache.org/licenses/LICENSE-2.0) since December 2018; OmniSim inherits that license, along with the world-file format, the controller API, and the PROTO system. Where this user guide still refers to "Webots" (it was inherited from the upstream manual), the underlying behaviour generally applies to OmniSim as well.

This guide will get you started using OmniSim. It assumes a minimal working knowledge of mobile robotics, of C, C++, or Python programming, and of the VRML97-derived world description language used for `.wbt` files.

We hope you enjoy working with OmniSim.
