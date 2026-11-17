## Starting OmniSim

The first time you start OmniSim, it will open the "Welcome to OmniSim!" menu with a list of possible starting points.

### Linux

There is **no native Linux package**. This section used to say "open a terminal
and type `omnisim-bin`", which cannot work: nothing installs that binary onto
your PATH, and even a source-built one needs `LD_LIBRARY_PATH` pointing at the
vendored Qt before it will start.

From a source build, launch through the CLI, which sets the runtime environment
for you:

```bash
cd $OMNISIM_HOME
python3 -m omnisim doctor          # is this install coherent?
python3 -m omnisim demo            # the flagship demo
python3 -m omnisim run-world projects/samples/demos/worlds/showcase/warehouse_husky.omniworld
```

There is also a container, but ⚠️ **it is not published** — no
`ghcr.io/omnilink-tech/omnisim` tag exists on the registry, so `docker pull` and
`docker run ghcr.io/...` both fail. Build it yourself first:

```bash
docker build -f docker/Dockerfile.runtime -t omnisim:local .
docker run --rm omnisim:local doctor
```

See [`docker/README.md`](../../docker/README.md).

Running `bin/omnisim-bin` directly requires setting `LD_LIBRARY_PATH`,
`QT_QPA_PLATFORM` and `WEBOTS_TMPDIR` yourself -- see the Linux section of the
[developer quickstart](../developer/quickstart.md).

### macOS

**macOS is not supported: there is no package, no verified build, and Newton
physics is unverified. Use Windows or Ubuntu 24.04.** This section previously
told you to "double-click on the OmniSim icon", which describes an installer
that does not exist.

The container is not a substitute. It is unpublished (above), and it has no
`linux/arm64` build — on Apple Silicon it would run under x86-64 emulation,
untested, over physics unverified on the platform.

### Windows

Open the directory in which you installed OmniSim and double-click on the `launch.bat` script, or run `omnisim-bin.exe` directly from `<install-dir>\msys64\mingw64\bin\`.

You can also start OmniSim from a DOS console (`cmd.exe`) by typing `omnisim-bin` or `omnisim-bin.exe`.
This command works only if executed from the `<install-dir>\msys64\mingw64\bin` directory, or from any directory if that path was added to your `Path` environment variable.
**Note:** The bundled launchers are `omnisim.exe` (console) and `omnisimw.exe` (windowed — runs in the background and returns immediately); both are thin wrappers that spawn `omnisim-bin.exe`. The legacy names `webots.exe` / `webotsw.exe` are shipped as byte-identical copies and keep working.

To drive OmniSim from the command line rather than the GUI, use `omnisim.bat`
in the install root:

```bat
omnisim.bat doctor          REM is this install coherent? VERDICT + non-zero exit on failure
omnisim.bat demo            REM the flagship demo
omnisim.bat demos           REM list every runnable demo
```

It resolves an interpreter itself — a system Python first, falling back to the
CPython 3.12 bundled with the Newton runtime — so it works whether or not you
installed Python. With Python 3.12 on `PATH`, `python -m omnisim <command>` from
the install root is equivalent.

### Command Line Arguments

Following command line options are available when starting OmniSim from a Terminal (Linux/Mac) or a Command Prompt (Windows):

```
Usage: omnisim-bin [options] [worldfile]

Options:

  --help
    Display this help message and exit.

  --version
    Display version information and exit.

  --sysinfo
    Display information about the system and exit.

  --mode=<mode>
    Choose the startup mode, overriding application preferences. The <mode>
    argument must be either pause, realtime or fast.

  --no-rendering
    Disable rendering in the main 3D view.

  --fullscreen
    Start OmniSim in fullscreen.

  --minimize
    Minimize the OmniSim window on startup.

  --batch
    Prevent OmniSim from creating blocking pop-up windows.

  --clear-cache
    Clear the cache of OmniSim on startup.

  --stdout
    Redirect the stdout of the controllers to the terminal.

  --stderr
    Redirect the stderr of the controllers to the terminal.

  --port
    Change the TCP port used by OmniSim (default value is 1234).

  --stream[=<mode>]
    Start the OmniSim streaming server. The <mode> argument should be either
    w3d (default) or mjpeg.

  --extern-urls
    Print on stdout the URL of extern controllers that should be started.

  --heartbeat[=<time>]
    Print a dot (.) on stdout every second or <time> milliseconds if specified.

  --log-performance=<file>[,<steps>]
    Measure the performance of OmniSim and log it in the file specified in the
    <file> argument. The optional <steps> argument is an integer value that
    specifies how many steps are logged. If the --sysinfo option is used, the
    system information is prepended into the log file.

  convert
    Convert a PROTO file to a URDF, WBO, or WRL file.

```

The optional `worldfile` argument specifies the name of a world file to open —
`.omniworld`, or a legacy `.wbt`, which OmniSim still reads.
If it is not specified, OmniSim attempts to open the most recently opened file.

The `--minimize` option is used to minimize (iconize) OmniSim window on startup.
This also skips the splash screen and the eventual Welcome Dialog.
This option can be used to avoid cluttering the screen with windows when automatically launching OmniSim from scripts.
Note that `--minimize` only minimizes the window and skips the splash and Welcome dialog; it does not change the simulation mode (use `--mode=fast` to force `Fast` mode).

The `--mode=<mode>` option can be used to start OmniSim in the specified simulation mode.
The three valid simulation modes are: `pause`, `realtime` and `fast`; they correspond to the simulation control buttons of OmniSim' graphical user interface. (`--mode=run` is deprecated and falls back to `fast`.)
This option overrides, but does not modify, the startup mode saved in OmniSim' preferences.
For example, type `omnisim-bin --mode=pause filename.omniworld` to start OmniSim in `pause` mode.

The `--stdout` and `--stderr` options have the effect of redirecting OmniSim console output to the calling terminal or process.
For example, this can be used to redirect the controllers output to a file or to pipe it to a shell command.
`--stdout` redirects the *stdout* stream of the controllers, while `--stderr` redirects the *stderr* stream.
Note that the *stderr* stream may also contain OmniSim error or warning messages.

The `--port` option changes the default TCP port used by OmniSim for serving robot windows, web streaming and extern controllers. By default, OmniSim sets up its TCP server on port 1234. When starting multiple OmniSim instances, the ports are configured with consecutive values of 1234.

The `--stream` option enables the OmniSim streaming server in either `w3d` (default) or `mjpeg` mode.
You can get more information about web streaming in [this section](web-streaming.md).

For example, the following command will start OmniSim with the streaming server enabled on the TCP port '1235' in 'mjpeg' mode: `omnisim-bin --port=1235 --stream=mjpeg`

The `convert` subcommand allows conversion of a PROTO file to a URDF, WBO, or WRL file.
You can use a `-p` flag to override default PROTO parameters.
Usage example:
```
omnisim-bin convert -p translation="0 0 0.5" ${OMNISIM_HOME}/projects/objects/factory/forklift/protos/Forklift.proto -o forklift.urdf
```
For more details use: `omnisim-bin convert --help`.

### Safe Mode

It may happen that OmniSim cannot start because it is blocked on a world causing an OmniSim or OpenGL crash.
In this case, OmniSim can be started in safe mode.
The safe mode forces OmniSim to start with an empty world, reduces all the OpenGL options, and stores those preferences.
To do this, simply set the environment variable `OMNISIM_SAFE_MODE` in the environment running OmniSim.
(The legacy `WEBOTS_SAFE_MODE` spelling is still honoured when `OMNISIM_SAFE_MODE` is unset — see [`OmGuiApplication.cpp:294-296`](../../src/omnisim/gui/OmGuiApplication.cpp#L294) — but write the canonical name.)

Once successfully started this way, you must unset this environment variable, open again your world and increase [the OpenGL preferences](preferences.md#opengl).
This action may cause a new crash.

#### On Windows

From the Windows graphical user interface:
1. Open the `Environment Variables` system dialog box. To do so, look for "environment variable" in the `search bar` of the Windows `start menu`, click on `Edit the system environment variables`, this will open up the `System Properties` dialog box to the `Advanced` tab. Click on the `Environment Variables` button at the bottom.
2. Add a new `OMNISIM_SAFE_MODE` user environment variable. To do so, in the `user variables` panel, click on the `New` button and add a `New User Variable` named `OMNISIM_SAFE_MODE` with a value of `true`.
3. Start OmniSim as usual.

From the `cmd` command prompt:
```
setx OMNISIM_SAFE_MODE true
```

#### On Linux

```bash
export OMNISIM_SAFE_MODE=true
cd $OMNISIM_HOME
python3 -m omnisim run-world <world>
```

Set the variable in the shell that launches the simulator, then launch through
the CLI as in the [Linux](#linux) section above — nothing puts `omnisim-bin` on
your `PATH`, and running it directly needs `LD_LIBRARY_PATH`, `QT_QPA_PLATFORM`
and `WEBOTS_TMPDIR` set by hand.

(macOS is not supported; see above.)
