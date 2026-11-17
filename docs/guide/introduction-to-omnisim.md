## Introduction to OmniSim

### What is OmniSim?

If you are learning robotics, building a robot, or have an idea but don't have access to the hardware, OmniSim is built for you.

OmniSim is more than a simulator. It is **an open-source robotics workshop designed for agentic development** — a place where an agent has everything it needs to work on any robotic system. You can simulate complete robotic systems with high-fidelity physics, build digital twins, connect simulation to real robots, and give AI agents a workshop to program, test, and debug your system. You can do much of that simply by talking to it.

- **No robot?** Simulate one.
- **Already have a robot?** Build its digital twin — from its own URDF or CAD.
- **Don't know how to program it?** Let an agent help build and debug it.

The goal is simple: make robotics accessible to anyone with an idea. OmniSim is completely free and open source.

The deepest bench in the workshop is the one for finding out what went wrong. Robots fail in ways you cannot see — a gripper that drops the box two times in five, a joint quietly pinned against its limit, a contact that never happened. So you run your controller and then ask the scene: every contact, every joint limit hit, every grip, every damage event, and every line your controller printed, on one cursor-paged HTTP stream. On the CPU solver (`newtonSolver "mujoco"`, the default) the same scene runs bitwise identically, so a failure you can reproduce is a failure you can fix — and where the instruments cannot see, they say so instead of guessing.

> **Two boundaries worth knowing up front.** "Connect to real robots" means the *control surface*: the same agent tools and bridge protocol address the simulated robot and the physical one, but the shipped bridges run against mock drivers and no policy trained here has been validated on hardware — see the [sim-to-real guide](omnilink-sim-to-real.md). And a *digital twin* here is a model built from your robot's own description, not live-synced telemetry from the machine.

It is built to be driven by AI coding agents: Claude Code (or another coding agent) builds and iterates on the simulator and its worlds; OmniLink (or another runtime client) drives them.

Under the hood, OmniSim is a 3D simulation environment that lets you create virtual worlds with full physics — mass, joints, friction, contacts. You can populate them with passive objects and active mobile robots (wheeled, legged, aerial). Robots can be equipped with the usual battery of sensors and actuators (distance sensors, cameras, motors, touch sensors, emitters, receivers, and more) and programmed individually with the controller of your choice.

OmniSim is a fork of [Webots](https://github.com/cyberbotics/webots) and inherits its world-file format, controller API, PROTO system, sample robot models, and controller program examples. New OmniSim-specific work — agentic authoring workflows, the OmniWorld procedural generation library, runtime/build performance — lives in the [developer book](../developer/README.md).

### What can I do with OmniSim?

OmniSim is well suited for research, educational, and agent-driven robotics work. Typical use cases inherited from Webots include:

- Mobile robot prototyping (academic research, the automotive industry, aeronautics, the vacuum cleaner industry, the toy industry, hobbyists, etc.)
- Robot locomotion research (legged, humanoids, quadrupeds, etc.)
- Multi-agent research (swarm intelligence, collaborative mobile robot groups, etc.)
- Adaptive behavior research (genetic algorithms, neural networks, AI, etc.)
- Teaching robotics (robotics lectures, C/C++ / Python programming lectures, etc.)
- Robot contests and benchmarks

What OmniSim adds on top of that base is a debugging surface, and a workflow optimised for coding agents to drive it: scenario authoring, build, validation and runtime control are all reachable over plain HTTP and JSON — no ROS, no DDS, no in-process Python, no editor plugin — so an agent can load a world, run it, inspect the scene and read back a structured account of what happened without touching the GUI.

**You can stop time, and that is the part worth learning first.** `POST /sim/pause` freezes the scene and *keeps* it frozen across calls, and `POST /sim/step` then walks it forward one basic step at a time — so you can read the scene at an instant instead of chasing a moving target. `POST /sim/break` arms a **breakpoint**: you name an event you care about — a contact beginning, a joint hitting its limit, a grip being lost — the simulation runs on its own, and the moment that event happens it freezes itself right there. That is what a breakpoint buys you here: the scene is still standing at the moment it went wrong, instead of half a second past it. The reliable way to use it is the way you would use a debugger in any other language: **pause first, then step.** If you leave the simulation free-running, a break can only be noticed at the end of the controller's current turn, and the engine can travel a long way inside one of those — far enough that a fast event can slip past unnoticed altogether.

Two things it still cannot do. It cannot **rewind**: there is no recording, no replay, and `POST /sim/snapshot` is not a true checkpoint — it saves positions and joint angles but never velocities, so a restored scene does not carry on the way the original did. You catch the moment live; you do not re-run it. And worlds load in *light* mode by default, which silences the contact, grip and joint-limit event types — load with `{"light": false}` when you are debugging rather than authoring. A breakpoint on one of those silenced types is **refused with an explanation** rather than quietly armed, so you find out immediately instead of waiting for a break that could never fire.

### What do I need to know to use OmniSim?

You will need a minimal amount of technical knowledge to develop your own simulations:

- A basic knowledge of the C, C++, or Python programming language is necessary to program your own robot controllers.
If you would rather not write controller code at all, the [OmniLink chat demos](omnilink-chat-demos.md) let you drive a robot in plain English — one world per robot, no programming required.
- If you don't want to use existing robot models provided within OmniSim and would like to create your own robot models, or add special objects in the simulated environments, you will need a basic knowledge of 3D computer graphics and the VRML97-derived `.wbt` description language.
That will allow you to create 3D models in OmniSim or import them from 3D modeling software. OmniSim also imports URDF directly — see the `URDFRobot` node.

### How do I get user support?

OmniSim is a young, agentic-first fork. The most direct way to get help — and the right place for any OmniSim-specific bug or question — is the OmniSim repository's [issue tracker](https://github.com/omnilink-tech/omnisim/issues).

For background on behaviour **inherited from upstream Webots**, the following upstream community channels can also help (these belong to the Webots project, not OmniSim):

- [Robotics StackExchange](https://robotics.stackexchange.com/questions/tagged/webots) with the `webots` tag — technical questions about behaviour inherited from Webots.
- [Webots GitHub Discussions](https://github.com/cyberbotics/webots/discussions) and [Webots GitHub Issues](https://github.com/cyberbotics/webots/issues) — upstream behaviour and project history.

OmniSim-specific discussion and bug tracking happens in the OmniSim repository.

### OmniSim Simulation

An OmniSim simulation is composed of the following items:

1. A world file (`.omniworld`; legacy `.wbt` files are read too, never written) that defines one or several robots and their environment.
The world file may depend on external PROTO files (`.proto`) and textures.
2. One or several controller programs for those robots (in C/C++ / Python).

### What is a World?

A world, in OmniSim, is a 3D description of the properties of robots and their environment.
It contains a description of every object: position, orientation, geometry, appearance (like color or brightness), physical properties, type of object, etc.
Worlds are organised as hierarchical structures where objects can contain other objects (like in VRML97).
For example, a robot can contain two wheels, a distance sensor and a joint which itself contains a camera, etc.
A world file does not contain the controller code of the robots; it only specifies the name of the controller required for each robot.
Worlds are saved in `.omniworld` files (a legacy `.wbt` is read forever and saved back as `.omniworld`).
The world files are stored in the `worlds` subdirectory of each project.

### What is a Controller?

A controller is a computer program that controls a robot specified in a world file.
Controllers can be written in any of the programming languages supported by OmniSim: C, C++, or Python.
When a simulation starts, OmniSim launches the specified controllers, each as a separate process, and associates the controller processes with the simulated robots.
Note that several robots can use the same controller code; a distinct process will be launched for each robot.

Some programming languages need to be compiled (C and C++), other languages need to be interpreted (Python).
For example, C and C++ controllers are compiled to platform-dependent binary executables (e.g. `.exe` on Windows).
Python controllers are interpreted by the Python run-time (which must be installed).
The source files and binary files of each controller are stored together in a controller directory.
A controller directory is placed in the `controllers` subdirectory of each project.

### What is a Supervisor Controller?

The [Supervisor](../reference/supervisor.md) controller is the controller of a [Robot](../reference/robot.md) whose `supervisor` field is set to `TRUE`. It can execute operations that can normally only be carried out by a human operator and not by a real robot.
The [Supervisor](../reference/supervisor.md) controller can be written in any of the supported programming languages.
However, in contrast with a regular [Robot](../reference/robot.md) controller, the [Supervisor](../reference/supervisor.md) controller has access to privileged operations.
The privileged operations include simulation control — for example, moving the robots to a random position, making a video capture of the simulation, etc.
