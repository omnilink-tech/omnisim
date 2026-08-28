@echo off
REM ---------------------------------------------------------------------------
REM  OmniSim CLI launcher.  `omnisim doctor`, `omnisim demo`, `omnisim harness`.
REM
REM  WHY THIS FILE EXISTS
REM  README.md and BETA.md both open with `python -m omnisim doctor`. That needs
REM  two things the Windows package does not provide:
REM
REM    1. A SYSTEM PYTHON. The 613 MB payload bundles CPython 3.12 at
REM       msys64\mingw64\bin\newton-runtime\python.exe -- the engine's embedded
REM       physics interpreter -- but nothing puts a python.exe on the user's
REM       PATH. On a clean Windows box `python` opens the Microsoft Store, so
REM       the documented first command fails without ever naming OmniSim.
REM       Measured: that bundled interpreter runs `-m omnisim` correctly.
REM
REM    2. THE RIGHT WORKING DIRECTORY. `python -m omnisim` resolves the package
REM       off sys.path, which includes the cwd -- so it works in the install
REM       root and nowhere else. This sets PYTHONPATH instead of cd'ing, so a
REM       relative path the user typed still means what they meant.
REM
REM  Interpreter order is deliberate: a SYSTEM python wins. The bundled one is
REM  the fallback, because the engine spawns every Python controller as the bare
REM  command "python.exe" resolved from PATH, and the bundled interpreter has no
REM  omnisim_bridges on its sys.path -- the OmniLink bridges then come up quietly
REM  missing their deferred-intent tools. See the long note in launch.bat.
REM ---------------------------------------------------------------------------
setlocal

set "OMNISIM_HOME=%~dp0"
if "%OMNISIM_HOME:~-1%"=="\" set "OMNISIM_HOME=%OMNISIM_HOME:~0,-1%"
REM Pin the legacy alias too, so a stale system WEBOTS_HOME (an old Webots
REM install) cannot leak into the simulator or its controllers.
set "WEBOTS_HOME=%OMNISIM_HOME%"
set "PYTHONPATH=%OMNISIM_HOME%;%PYTHONPATH%"

set "OMNI_PY="
for /f "delims=" %%P in ('where python 2^>nul') do if not defined OMNI_PY set "OMNI_PY=%%P"
if defined OMNI_PY (
    REM A Microsoft Store alias stub is on PATH by default and is NOT a Python:
    REM running it opens the Store. Treat it as absent.
    if not "%OMNI_PY%"=="%OMNI_PY:WindowsApps=%" set "OMNI_PY="
)

if not defined OMNI_PY (
    set "OMNI_PY=%OMNISIM_HOME%\msys64\mingw64\bin\newton-runtime\python.exe"
    if exist "%OMNISIM_HOME%\msys64\mingw64\bin\newton-runtime\python.exe" (
        echo [OmniSim] No system Python on PATH - using the bundled interpreter.
        echo [OmniSim] Install Python 3.12 from python.org for the full OmniLink demos.
        echo.
        REM Tail, never front: this is only so controllers find AN interpreter.
        set "PATH=%PATH%;%OMNISIM_HOME%\msys64\mingw64\bin\newton-runtime"
    ) else (
        echo [OmniSim] ERROR: no Python found.
        echo   Install Python 3.12 from https://www.python.org/downloads/
        echo   ^(tick "Add python.exe to PATH"^), then re-run this command.
        exit /b 1
    )
)

REM Tell the CLI how it was reached, so the "next command" it prints back is one
REM this user can actually type. A reader who got here BECAUSE they have no
REM system Python must not be answered with `python -m omnisim demo`.
set "OMNISIM_INVOKED_AS=omnisim.bat"
"%OMNI_PY%" -m omnisim %*
exit /b %ERRORLEVEL%
