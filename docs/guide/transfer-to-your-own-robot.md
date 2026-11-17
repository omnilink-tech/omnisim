## Transfer to your own Robot

In mobile robot simulation, it is often useful to transfer the results onto real mobile robots.
The controller API is designed to be portable: the same control program can be re-pointed at a real robot by re-implementing the API calls against your hardware.
This section explains how to develop your own transfer system to your own mobile robot.

> **What OmniSim does and does not claim.** The *programming interface* is portable — that is the subject of this page, and it is a real property. **Behavioural transfer is not claimed.** The simulation is an approximation of the real robot's physics, so some tuning is always necessary, and OmniSim ships **no validated sim-to-real result**: no policy trained here has been certified on physical hardware. Treat every number you obtain in simulation as a hypothesis to be re-verified on the robot.
>
> The upstream Webots project shipped ready-made transfer systems for several robots (*e-puck*<sup>TM</sup>, *DARwIn-OP*<sup>TM</sup>, *Khepera*<sup>TM</sup>, *Hemisson*<sup>TM</sup>). **Those robots and their transfer code are not carried into OmniSim** — they remain available in the [upstream Webots repository](https://github.com/cyberbotics/webots) as a reference. The mechanisms described below are the general recipes; you supply the robot-specific half.
>
> For the agent-facing path — driving a real robot with the same OmniLink agent and the same tool surface that drives the simulated one — see [From sim to real](omnilink-sim-to-real.md) and the mock-driver starter kit in [`agents/bridges/`](../../agents/bridges/).

### Remote Control

#### Remote Control Overview

Often, the easiest way to transfer your control program to a real robot is to develop a remote control system.
In this case, your control program runs on the computer, but instead of sending commands to and reading sensor data from the simulated robot, it sends commands to and reads sensor data from the real robot.
Developing such a remote control system can be achieved in a very simple manner by writing your own implementation of the OmniSim API functions as a small library.
For example, you will probably have to implement the `wb_motor_set_velocity` function to send a specific command to the real robot with the wheel speeds as an argument.
This command can be sent to the real robot via the serial port of the PC, or any other PC-robot interface you have.
You will probably need to make some unit conversions, since your robot may not use the same units of measurement as the ones used in OmniSim.
The same applies for reading sensor values from the real robot.

#### Developing a Remote Control Plugin

OmniSim already provides some facilities to implement a remote control library and in particular it is possible to develop it as a controller plugin.
Once set in the corresponding field of the [Robot](../reference/robot.md) node, this remote control plugin will be executed automatically when running the controller.
Implementation details are described in [this section](controller-plugin.md#remote-control-plugin).

#### Special Functions

The `wb_robot_init` function must be the first called function.
It performs the controller library's initialization.

The `wb_robot_step` function should be called repeatedly (typically in an infinite loop).
It requests that the simulator performs a simulation step of ms milliseconds; that is, to advance the simulation by this amount of time.

The `wb_robot_cleanup` function should be called at the end of a program in order to leave the controller in a clean fashion.

#### Running your Real Robot

Once linked with your own remote control plugin, you can control your real robot by running the simulation in OmniSim.
It might be useful to also add a robot window (see [this section](controller-plugin.md#robot-window)) to graphically display specific sensor values, motor commands or a stop button.

Such a remote control system is designed to be implemented in C/C++ as explained in [this section](controller-plugin.md); however, it can also be implemented in other programming languages by creating a wrapper.

### Cross-Compilation

#### Cross-Compilation Overview

Developing a cross-compilation system will allow you to recompile your OmniSim controller for the embedded processor of your own real robot.
Hence, the source code you wrote for the OmniSim simulation will be executed on the real robot itself, and there is no need to have a permanent PC connection with the robot as with the remote control system.
This is only possible if the processor on your robot can be programmed respectively in C, C++, or Python.
It is not possible for a processor that can be programmed only in assembler or another specific language.

OmniSim does **not** ship a ready-made cross-compilation system for any robot — the upstream Webots examples (e-puck, Hemisson) were not carried into the fork. The recipe below is the general one; consult the [upstream repository](https://github.com/cyberbotics/webots/tree/master/projects/robots) if you want to read a worked implementation.

#### Developing a Custom Library

Unlike the remote control system, the cross-compilation system requires the source code of your OmniSim controller to be recompiled using the cross-compilation tools specific to your own robot.
You will also need to rewrite the OmniSim include files to be specific to your own robot.
In simple cases, rewriting only the include files you actually use is enough.
In more complex cases, you will also need to write some C source files to be used as a replacement for the OmniSim "Controller" library, but running on the real robot.
You should then recompile your OmniSim controller with your robot cross-compilation system and link it with your robot library.
The resulting file should be uploaded onto the real robot for local execution.

How much work this is depends on the robot: some need only a handful of replacement include files, while others need a full vendor C library alongside them.

### Interpreted Language

In some cases, it may be better to implement an interpreted language system.
This is useful if your real robot already uses an interpreted language, like Basic or a graph-based control language.
In this case, the transfer is very easy since you can directly transfer the code of your program that will be interpreted to the real robot.
