## Interfacing OmniSim to Third Party Software with TCP/IP

### Overview

OmniSim offers programming APIs for C/C++ and Python.
It is also possible to interface OmniSim with other programming languages or software packages (*Lisp*<sup>TM</sup>, *LabView*<sup>TM</sup>, MATLAB, a game engine, a web front-end, etc.) over a TCP/IP connection.

There are three supported ways to do this, from lowest to highest level:

| Mechanism | Port | Use it when |
|---|---|---|
| **Extern controllers** | `1234` (auto-scans to `1244`) | You want to run a robot's controller as a separate process — possibly on another machine, possibly in a language OmniSim has no binding for. This is the built-in mechanism; see [Running Extern Robot Controllers](running-extern-robot-controllers.md). |
| **A robot bridge** | `8765` | You want a stable, robot-shaped HTTP surface (`/set_joint_positions`, `/drive_forward`, `/get_robot_state`, …) that any HTTP client — or an LLM agent — can drive. This is the OmniLink bridge pattern. |
| **The validation harness** | `6789` | You want to drive the *simulator* rather than a robot: load worlds, step, reset, screenshot, read the scene tree, stream events. |

All three are specified in [PROTOCOL.md](../../PROTOCOL.md), the canonical OmniSim Wire Protocol document. If you are writing a tool outside this repository that talks to OmniSim, that is the contract to implement against.

### Writing your own TCP/IP controller

If none of the above fits, you can define your own protocol. Write a controller (in C, C++, or Python) that opens a socket and relays between OmniSim's controller API and your third-party program. The reference implementations to copy are the OmniLink bridges, which do exactly this:

- [`projects/samples/demos/controllers/omnilink_arm_bridge/`]({{ url.github_tree }}/projects/samples/demos/controllers/omnilink_arm_bridge) — an arm; owns the motors, serves HTTP, dispatches requests to `motor.setPosition()`.
- [`projects/samples/demos/controllers/omnilink_mobile_bridge/`]({{ url.github_tree }}/projects/samples/demos/controllers/omnilink_mobile_bridge) — a wheeled base; same shape, dispatches to `motor.setVelocity()`.

The pattern is: a socket server on one thread, the OmniSim `robot.step()` loop on the main thread, and a lock-guarded command buffer between them. A controller must never block the simulation step waiting on the network.

### Main advantages

You can have several simulated robots in the same world, each running its own instance of the same TCP/IP controller on a different port, letting your third-party software drive several robots over several connections. Give each robot a distinct `name` and read it with `robot_get_name` (C) / `Robot.getName()` (Python) to decide which port to open.

You can also spread controller programs across a network of machines — useful when the controller runs something computationally expensive (learning algorithms, planners, a large model).

Finally, set the robot to synchronous or asynchronous mode depending on whether the simulator should wait for your commands:

- **Synchronous** (`synchronization TRUE`, the default) — the simulator waits for each controller's commands before stepping. Deterministic; the right choice for reproducible experiments.
- **Asynchronous** (`synchronization FALSE`) — the simulator runs as fast as it can, without waiting. Combine this with real-time mode so robots behave like real robots on an asynchronous link.

### Limitations

The main drawback of TCP/IP interfacing is bandwidth. If your robot has a camera or a lidar, the protocol has to ship every frame or scan across the socket, which can be network-intensive. Mitigate it with a fast local link, a lower sensor resolution, or by compressing the payload before sending it.

Latency is the other cost: every round trip adds delay between sensing and actuation. If your control law is sensitive to that (balance, force control), keep the loop inside an OmniSim controller and expose only high-level commands over TCP/IP — which is exactly what the bridge pattern above does.
