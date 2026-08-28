## Compiling Controllers in a Terminal

It is possible to compile OmniSim controllers in a terminal instead of using the built-in editor.
In this case you need to define the `OMNISIM_HOME` environment variable and make it point to OmniSim installation directory.
The `OMNISIM_HOME` variable is used to locate OmniSim header files and libraries in the Makefiles.
Setting an environment variable depends on the platform (and shell), here are some examples:

### Linux

`OMNISIM_HOME` must point to the root of your OmniSim checkout. On Linux that is
a source build — there is no native package yet, see
[Installation Procedure](installation-procedure.md). From inside the checkout:

```sh
$ export OMNISIM_HOME=$(pwd)
```

Or add this line (with the absolute checkout path) to your "~/.bash\_profile" file.

(macOS is not supported: there is no package, no verified build, and Newton
physics is unverified. Use Windows or Ubuntu 24.04.)

Once these environment variables are defined, you should be able to compile in a terminal, with the `make` command.
Like with the editor buttons, it is possible to build the whole project, or only a single binary file, e.g.:

```sh
$ make
$ make clean
$ make my_robot.o
```

### Windows

On Windows you must use the MSYS2 terminal to compile the controllers.
MSYS2 is a UNIX-like terminal that can be used to invoke UNIX commands.
Please follow the MSYS2 setup instructions in the [OmniSim Developer Quickstart](../developer/quickstart.md) to install it.

You will also have to set the `OMNISIM_HOME` environment variable to point to the root of your OmniSim source checkout, and add the path to the OmniSim binaries (`msys64/mingw64/bin` inside the checkout) to the MSYS2 `PATH` environment variable. For example, if your checkout is at `C:\omnisim`:

```bash
export OMNISIM_HOME="C:\omnisim"
export PATH=$PATH:/C/omnisim/msys64/mingw64/bin
```

(`build_omni.bat` derives `OMNISIM_HOME` automatically from its own location, so when you build with that wrapper you do not need to set it by hand — see [AGENTS.md §2](../../AGENTS.md#2-build-only-if-the-binary-is-missing) and the [Developer Quickstart](../developer/quickstart.md).)

For convenience, the two above lines can be appended to your `~/.bash_profile` file of MSYS2.

Once MSYS2 is installed and the environment variables are defined, you should be able to compile controllers by invoking `make` in the MSYS2 terminal, e.g.:

```sh
$ make
$ make clean
```
