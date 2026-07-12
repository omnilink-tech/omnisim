:: script running gdb on OmniSim in a DOS console

@ECHO off

ECHO Setup environment variables...
SET PWD=%~d0
SET OMNISIM_HOME=%~dp0\..\..\..
SET PATH=%PWD%\msys64\mingw64\bin;%PWD%\msys64\usr\bin;%PATH%

ECHO Generate the gdb input file...
SET INPUT=%OMNISIM_HOME%\omnisim_debug_input.txt
IF EXIST %INPUT% (
  DEL %INPUT%
)
ECHO run >> %INPUT%
ECHO where >> %INPUT%
ECHO quit >> %INPUT%

ECHO Delete the gdb output file if exists
SET OUTPUT=%OMNISIM_HOME%\omnisim_debug_output.txt
IF EXIST %OUTPUT% (
  DEL %OUTPUT%
)

ECHO Starting OmniSim in debug mode...
gdb.exe omnisim-bin.exe < %INPUT% >> %OUTPUT%

ECHO Delete the gdb input file...
DEL %INPUT%
