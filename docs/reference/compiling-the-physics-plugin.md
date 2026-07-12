## Compiling the Physics Plugin

When a plugin is created using the **File / New / New Physics Plugin...** menu item, OmniSim will automatically add a suitable ".c" or ".cpp" source file and a Makefile to the plugin's directory.
Your plugin can be compiled with OmniSim text editor or manually by using `gcc` and `make` commands in a terminal.
On Windows, you can also use Microsoft Visual Studio to compile the plugin.
In this case, please note that the plugin should be dynamically linked to the ODE library.
On Windows the ODE shared library ("ode.dll") ships in the `msys64/mingw64/bin` directory of your OmniSim installation; link against it with `-lode` (the OmniSim Makefile system adds `-L"$(WEBOTS_LIB_PATH)" -lode` automatically when `USE_ODE` is set).
Under Linux, you don't need to link the shared library with anything.
