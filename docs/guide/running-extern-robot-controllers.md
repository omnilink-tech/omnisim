# Running Extern Robot Controllers

This chapter describes extern robot controllers and how to use them.

## Introduction

Normally, OmniSim launches automatically the robot controller specified in the `controller` field of each [Robot](../reference/robot.md) node.
However, if this field is set to `<extern>`, no controller is launched and the robot will behave like if its `controller` field was an empty string, that is, the robot will not be controlled.
But as soon as an OmniSim controller is launched manually on the same computer, it will attempt to connect to this `<extern>` robot controller in order to control this robot.
It is also possible to connect `<extern>` controllers from remote computers using a TCP connection.

## Usefulness

Running an extern robot controller requires that the controller is launched manually.
This may seem inconvenient, but in several cases, it turns out to be very useful, because the user has full control over the controller process.
For example, it may run it within a debugging environment, like *gdb*, a command line tool like *$ shell*, or within some Integrated Development Environment (IDE), such as *Visual C++*, *Eclipse* or *PyCharm*.
Also, the standard output and error streams (`stdout` and `stderr`) remain under the user control and are not sent to the OmniSim console.
It is even possible to read the standard input stream (`stdin`) like with any standard program.
Moreover, starting external controllers remotely allows to run OmniSim on a different machine than the controller, which can be useful if the specifications required by the two processes are very different.

> **Note**: If the `robot.synchronization` field is set to `TRUE` OmniSim will wait for the extern controller to be launched, otherwise the simulation will run whether the controller is started or not.

## Launcher

OmniSim is distributed with a controller launcher.
It must be used to start any extern controller file.
Compatible file types are listed below:
* **Executables**: no extension on Linux/macOS and `.exe` on Windows.
* **Python**: `.py`.

The `OMNISIM_HOME` environment variable must be set to the installation folder of OmniSim.
For example:

```bash
export OMNISIM_HOME=/home/username/omnisim
```

If you are using the snap version of OmniSim, please refer to the corresponding section: [Running Extern Robot Controller with the Snap Version of OmniSim](#running-extern-robot-controller-with-the-snap-version-of-omnisim).

The following command line should be used to start a controller:

%tab-component "os"

%tab "Windows"

```bash
omnisim-controller.exe [options] path/to/controller/file [controller-args]
```

%tab-end

%tab "Linux"

```bash
$OMNISIM_HOME/omnisim-controller [options] path/to/controller/file [controller-args]
```

%tab-end

%tab "macOS"

```bash
$OMNISIM_HOME/Contents/MacOS/omnisim-controller [options] path/to/controller/file [controller-args]
```

%tab-end

%end

> **Note**: The controller file path can be absolute or relative to the directory from which the launcher is started.

### Options

The following options are available when starting an extern controller with the launcher.
Concrete use cases are discussed in the [Setup](#setup) section.

```
  --help
    Display this help message and exit.

  --protocol=<ipc|tcp>
    Define the protocol to use to communicate between the controller and OmniSim.
    `ipc` is used by default.
    `ipc` should be used when OmniSim is running on the same machine as the extern controller.
    `tcp` should be used when connecting to a remote instance of OmniSim.

  --ip-address=<ip-address>
    The IP address of the remote machine on which the OmniSim instance is running.
    This option should only be used with the `tcp` protocol (i.e. remote controllers).

  --port=<port>
    Define the port to which the controller should connect. 1234 is used by default, as it is the default port for OmniSim.
    This setting allows you to connect to a specific instance of OmniSim if there are multiple instances running on the target machine.
    The port of an OmniSim instance can be set at its launch.

  --robot-name=<robot-name>
    Target a specific robot by specifying its name in case multiple robots wait for an extern controller in the OmniSim instance.

  --stdout-redirect
    Redirect the stdout of the controller to the OmniSim console.

  --stderr-redirect
    Redirect the stderr of the controller to the OmniSim console.
```

## Setup

To run a local extern controller, both OmniSim and the controller should run from the same user account.
This is not needed for a remote extern controller where OmniSim and the controller run on different machines.
Different use cases are detailed here from the most simple to the most complex:

### Single Simulation and Single Local Extern Robot Controller

You are running a single OmniSim simulation simultaneously on the same machine and this simulation has only one robot that you want to control from an extern controller.
In this case, you simply need to set the `controller` field of this robot to `<extern>` and to launch the controller with the launcher.
No specific option is needed in this case, as default parameters will automatically target the correct instance of OmniSim and the single available robot.
Once an extern controller is connected to the robot, any other attempt to connect to that robot will be refused by OmniSim and the controller attempting to connect will terminate immediately.

### Single Simulation and Multiple Local Extern Robot Controllers

You are running a single OmniSim simulation simultaneously on the same machine and this simulation has several robots that you want to control from extern controllers.
In this case, for each robot that you want to control externally, you should set their `controller` field to `<extern>`.
Then, you should set the `--robot-name` option of the controller launcher to match the `name` field of the [Robot](../reference/robot.md) node you want to control.
The started controller will connect to the extern robot whose `name` matches the one provided in the command line.
This operation can be repeated in a new terminal for each robot in the simulation.

### Multiple Concurrent Simulations and Single Local Extern Robot Controller

If you are running multiple simulations simultaneously on the same machine, and each simulation has only one robot that you want to control from an extern controller, then you need to indicate to the controller to which instance of OmniSim it should try to connect.
This can be achieved by setting the `--port` option of the launcher to the TCP port of the target OmniSim instance (defined with the equivalent `--port` command line option at OmniSim launch) to which you want to connect your controller.

### Multiple Concurrent Simulations and Multiple Local Extern Robot Controllers

If you are running multiple simulations simultaneously on the same machine, and each simulation has several robots that you want to control from extern controllers, then you need to indicate to each controller to which instance of OmniSim and to which robot it should try to connect.
To achieve this, simply set the launcher `--port` option to the TCP port of the target OmniSim instance (set with the equivalent `--port` command line option when launching OmniSim) and the `--robot-name` option to the name of the target robot to which you want to connect your controller.

### Remote Extern Controllers

`<extern>` controllers can also be started from a remote machine.
In this case, when starting the controller with the launcher, the `--protocol` option should be set to `tcp`.
The `--ip-address` option must be set to the IP address of the remote machine on which the target instance of OmniSim is running.
If multiple instances of OmniSim are running on the remote machine, the `--port` option must be set to the TCP port (defined with the `--port` command line option at OmniSim launch) of the OmniSim instance to which you want to connect your controller.
Finally, if the target instance contains multiple robots waiting for an extern controller connection, the `--robot-name` option can be set to the name of the robot to which you want to connect your controller.

It is possible to restrict the IP addresses that can connect to an OmniSim instance.
To do this, the allowed IP addresses can be added in the format `X.X.X.X` in the OmniSim preferences in the `Network` tab.
It is also possible to allow a range of addresses using a subnet mask in [CIDR](https://en.wikipedia.org/wiki/Classless_Inter-Domain_Routing) notation, with the following format: `X.X.X.X/<netmask>`.
Note that if the list is left empty, all incoming connections are allowed.

### Running Extern Robot Controller with the Snap Version of OmniSim

In order to compile and execute extern controllers, the following environment variables should be set:
```
export OMNISIM_HOME=/snap/webots/current/usr/share/webots
export LD_LIBRARY_PATH=$OMNISIM_HOME/lib/controller
```

Because of the snap sand-boxing system, OmniSim has to use a special temporary folder to share information with robot controllers.
When you launch the snap version of OmniSim, the launcher computes the `WEBOTS_TMPDIR` environment variable if it is not already set.
This variable is computed from the `SNAP_USER_COMMON` environment variable which typically points to `/home/username/snap/webots/common`, a folder accessible by both OmniSim and your own programs.
Similarly, the libController will automatically check this folder and its contents to determine if it should use it to communicate with OmniSim.
It is recommended that you do not override this `WEBOTS_TMPDIR` environment variable, unless you want to experiment a different mechanism.

## Example Usage

1. Set OMNISIM_HOME to the OmniSim installation directory, for example:

  ```bash
  export OMNISIM_HOME=/usr/local/omnisim
  ```

2. Open a world that contains a robot with a known controller, e.g. `projects/samples/demos/worlds/showcase/jackal_drive.wbt`.
3. If the simulation was running, stop it and revert it.
4. Open the robot node in the scene tree and change its `controller` field from its current name to `<extern>`.
5. Save the simulation, restart it and run it.
6. In a terminal, launch the same controller binary as an extern process:

  ```bash
  $OMNISIM_HOME/omnisim-controller $OMNISIM_HOME/projects/default/controllers/<controller_name>/<controller_name>
  ```

  **Note**: To connect to a remote OmniSim instance, start the controller with:

  ```bash
  $OMNISIM_HOME/omnisim-controller --protocol=tcp --ip-address=127.0.0.1 $OMNISIM_HOME/projects/default/controllers/<controller_name>/<controller_name>
  ```

  Simply replace `127.0.0.1` by the IP address of your remote machine.
7. You should see the robot in `jackal_drive.wbt` (or whichever world you chose) being controlled by the extern process you just started.
