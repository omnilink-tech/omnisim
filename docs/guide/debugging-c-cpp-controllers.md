## Debugging C/C++ Controllers

### Controller Processes

In the OmniSim environment, the OmniSim application and each robot C/C++ controller are executed in distinct operating system processes.
For example, when the "husky_fleet_arena.omniworld" world is executed, there is a total of eleven processes in memory; one for OmniSim and one for each of the ten Husky controllers.
To debug a C/C++ controller with Microsoft Visual Studio, please see [here](using-your-ide.md#visual-studio).

When a controller process performs an illegal instruction, it is terminated by the operating system while the OmniSim process and the other controller processes remain active.
Although OmniSim is still active, the simulation blocks because it waits for data from the terminated controller.
So if you come across a situation where your simulation stops unexpectedly, but the OmniSim GUI is still responsive, this usually indicates that the controller has crashed .
This can easily be confirmed by listing the active processes at this moment: For example on Linux, type:

```sh
$ ps -e
...
12751 pts/1    00:00:16 omnisim-bin
13294 pts/1    00:00:00 husky_random
13296 pts/1    00:00:00 husky_random
13297 pts/1    00:00:00 husky_random
13298 pts/1    00:00:00 husky_random
13299 pts/1    00:00:00 husky_random
13300 pts/1    00:00:00 husky_random
13301 pts/1    00:00:00 husky_random
13302 pts/1    00:00:00 husky_random
13303 pts/1    00:00:00 husky_random
13304 pts/1    00:00:00 husky_random <defunct>
...
```

On macOS, use rather `ps -x` and on Windows use the *Task Manager* for this.
If one of your robot controllers is missing in the list (or appearing as *defunct*) this confirms that it has crashed and therefore blocked the simulation.
In this example one of the "husky\_random" controllers has crashed.
Note that the crash of a controller is almost certainly caused by an error in the controller code, because an error in OmniSim would have caused OmniSim to crash.
Fortunately, the GNU debugger (`gdb`) can usually help finding the reason of the crash.
The following example assumes that there is a problem with a controller and indicates how to proceed with the debugging.

### Using the GNU Debugger with a Controller

On Windows GDB can be installed from the MSYS2 environment with the `mingw-w64-x86_64-gdb` package, alongside the build dependencies covered in OmniSim's [developer quickstart](../developer/quickstart.md#2-install-build-dependencies).

The first step is to recompile the controller with the `debug` target, in order to add debugging information to the executable file. 
You must recompile the controller directly in a terminal, as the OmniSim text editor `Build` button will omit debugging information from the build:

```sh
$ make clean
$ make debug
...
```

Once you have recompiled the controller, you will need to ensure the controller of the [Robot](../reference/robot.md) node is set to be [extern](running-extern-robot-controllers.md).
If it is not, this can be set from the scene tree:
Hit the `Pause` and `Reset` buttons, set the `controller` field of the Robot node to `<extern>` and save the world file.
From a terminal, go to the folder containing your controller program and start it with `gdb`:

```sh
$ gdb my_controller
```

In `gdb`, type for example:

```sh
(gdb) break my_controller.c:50
(gdb) run
```

Then, run the OmniSim simulation, using the `Run` button (you may also use the `Step`, `Real-Time` or `Fast` button).
Your controller program will start controlling the extern robot in OmniSim.
Once the break point is reached, you will be able to query variables, setup new break points, etc.

Then, the `cont` command will instruct the debugger to resume the execution of the process.
You may also use the `step` function to proceed step-by-step.

The controller's execution can be interrupted at any time (<kbd>ctrl</kbd>-<kbd>C</kbd>), in order to query variables, set up break points, etc.
When a crash occurs, `gdb` prints a diagnostic message similar to this:

```
Program received signal SIGSEGV, Segmentation fault.
[Switching to Thread -1208314144 (LWP 16448)]
0x00cd6dd5 in _IO_str_overflow_internal () from /lib/tls/libc.so.6
```

This indicates the location of the problem.
You can examine the call stack more precisely by using the `where` command of `gdb`.
For example type:

```sh
(gdb) where
#0 0x00cd6dd5 in _IO_str_overflow_internal() from /lib/tls/libc.so.6
#1 0x00cd596f in _IO_default_xsputn_internal() from /lib/tls/libc.so.6
#2 0x00cca9c1 in _IO_padn_internal() from /lib/tls/libc.so.6
#3 0x00cb17ea in vfprintf() from /lib/tls/libc.so.6
#4 0x00ccb9cb in vsprintf() from /lib/tls/libc.so.6
#5 0x00cb8d4b in sprintf() from /lib/tls/libc.so.6
#6 0x08048972 in run(duration=0) at husky_random.c:106
#7 0x08048b0a in main() at husky_random.c:140
```

By examining carefully the call stack you can locate the source of the error.
In this example we will assume that the `sprintf` function is OK, because it is in a system library.
Therefore it seems that the problem is caused by an illegal use of the `sprintf` function in the `run` function.
The line 106 of the source file "husky\_random.c" must be examined closely.
While the controller is still in memory you can query the values of some variables in order to understand what happened.
For example, you can use the `frame` and `print` commands:

```sh
(gdb) frame 6
#6  0x08048953 in run (duration=0) at husky_random.c:106
106         sprintf(time_string, "%02d:%02d", (int) (time / 60),
 (int) time % 60);
(gdb) print time_string
$1 = 0x0
```

The `frame` command instructs the debugger to select the specified stack frame, and the `print` command prints the current value of an expression.
In this simple example we clearly see that the problem is caused by a NULL (0x0) *time\_string* argument passed to the `sprintf` function.
The next steps are to: 
1. Fix the problem
2. Recompile the controller 
3. Reload the world to give it another try.

Once it works and gives the correct output you can remove the *-g* flag from the Makefile.
