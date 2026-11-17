## Introduction

OmniSim can execute controllers written in compiled (C/C++) or interpreted (Python) languages.
The compilation or interpretation process requires extra software that must usually be installed separately.
On Windows, the MinGW C/C++ compiler is provided by MSYS2, which must be installed separately (from https://www.msys2.org/); the in-repo `$OMNISIM_HOME/msys64/mingw64/bin/` directory contains only the built runtime binaries and DLLs, not the compiler toolchain. See [the Developer Quickstart](../developer/quickstart.md) for the full Windows setup.
For any other language or platform the software development tools must be installed separately.
Note that OmniSim uses very standard tools that may already be present in a standard installation.
Otherwise the instructions in this chapter will advise you about the installation of your software development tools.
