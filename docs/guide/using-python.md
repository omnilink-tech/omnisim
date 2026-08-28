## Using Python

### Introduction

The Python API has been designed taking inspiration from the C++ API.
This implies that their class hierarchy, their class names and their function names are almost identical.
The Python API is currently composed of a set of about 25 classes having about 200 public functions located in the module called *omnisim*, which is the only module name: the legacy *controller* alias was removed, so a controller written against it must change `from controller import ...` to `from omnisim import ...`.
The classes are either representations of a node of the scene tree (such as Robot, LED, etc.) or utility classes (such as Motion, ImageRef, etc.).
A complete description of these functions can be found in the reference guide while the instructions about the common way to program a Python controller can be found in [this chapter](programming-fundamentals.md).

The Python controller API is stdlib-only and works on Python 3.8 and later. That is not the same as the version OmniSim itself needs: install **Python 3.12**, which is what the engine links against — see [System Requirements](system-requirements.md#python).

Alternatively to the OmniSim built-in editor, [PyCharm](https://www.jetbrains.com/pycharm) can be used to edit and launch Python controllers, see the [Using PyCharm with OmniSim](using-your-ide.md#pycharm) chapter for a step-by-step procedure.

### Installation

OmniSim starts Python using the standard `python` command line.
As a consequence, it executes the first `python` binary found in the current `PATH`.
If you want to use a different version of Python, please install it if needed and configure your environment so that it becomes the default `python` version when called from the command line in a terminal.
Alternatively, you can change the default Python command from the OmniSim Preferences in the General tab.
If you set it for example to `python3.12` instead of `python`, this version of Python will be used by default, if available from the command line.
It is also possible to set a different version of Python for each robot controller by editing the `[python]` section of the `runtime.ini` file in each robot controller directory and setting the `COMMAND` value to `python3`, `python3.12`, etc.
If specified in the `runtime.ini` file of a controller, this Python command will be executed instead of the default one to launch this controller.
On Linux, it is also possible to override this value by setting a standard Python shebang header line in your main python controller file, for example:

```python
#!/usr/bin/env python3.12
```

On Windows, the shebang header line option is not supported.
However, it is parsed and a warning is displayed in case of mismatch, e.g., if the version specified on the shebang header line mismatches the actual version of Python used by OmniSim.

#### Linux Installation

Ubuntu 24.04 ships Python 3.12 as its system `python3`, which is what the engine
links against — leave it alone rather than installing a second interpreter. To
check what you have: `python3 --version`.

Do not put Newton's wheels in a virtualenv: the engine's embedded interpreter
ignores them. See [System Requirements](system-requirements.md#python).

#### Windows Installation

You can install the latest version of Python from the official [Python website](https://www.python.org).
Then, you have to modify your `PATH` environment variable to add the path to the `python.exe` binary which is located in the main installation folder.
To check this was done properly, you can open a DOS console (`CMD.EXE`) and type `python --version`.
If it displays the correct Python version, then, everything is setup properly and you should be able to run the Python example provided with OmniSim in the `OMNISIM_HOME/projects/languages/python/worlds/example.omniworld` world file.

### Libraries

Some Python controllers rely on the [OpenCV](http://opencv.org/) and [NumPy](http://numpy.org/) packages (for example, image-processing or computer-vision controllers).
These packages have to be installed on the system in order to correctly run such a controller.
Using Python *pip*, the *NumPy* package is automatically installed with *opencv-python* package.

#### Linux Libraries

Use the `pip` command to install OpenCV:

```sh
sudo apt-get install python3-pip
sudo pip3 install opencv-python
```

#### Windows Libraries

Open the DOS console (CMD.EXE) and type:

```sh
PYTHON_PATH\Scripts\pip.exe install opencv-python
```

Where `PYTHON_PATH` is the path to the Python installation directory, for example `C:\Python36`.
