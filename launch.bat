@echo off
setlocal

set "OMNISIM_HOME=%~dp0"
if "%OMNISIM_HOME:~-1%"=="\" set "OMNISIM_HOME=%OMNISIM_HOME:~0,-1%"
REM Pin the legacy alias too so a stale system WEBOTS_HOME (old Webots
REM install) cannot leak into the simulator or its controllers.
set "WEBOTS_HOME=%OMNISIM_HOME%"

set "OMNISIM_BIN=%OMNISIM_HOME%\msys64\mingw64\bin\omnisim-bin.exe"
if not exist "%OMNISIM_BIN%" (
    echo [OmniSim] ERROR: simulator binary not found:
    echo     %OMNISIM_BIN%
    echo.
    echo The bundled msys64 runtime is missing from this checkout.
    echo Build it first with build_omni.bat, or restore the msys64\ directory.
    exit /b 1
)

REM ── PATH ORDER IS LOAD-BEARING: IT CHOOSES THE CONTROLLER INTERPRETER ───────
REM mingw64\bin must lead. Qt6 plus libgcc_s_seh-1 / libstdc++-6 /
REM libwinpthread-1 live there and the GUI and every compiled controller
REM need them.
set "PATH=%OMNISIM_HOME%\msys64\mingw64\bin;%PATH%"

REM newton-runtime goes ON THE TAIL, not the front. It used to be PREPENDED,
REM and that silently degraded the OmniLink demos:
REM
REM   The engine spawns every Python controller as the BARE COMMAND
REM   "python.exe" (WbLanguageTools::pythonCommand -> QProcess), resolved from
REM   this PATH. Prepending newton-runtime therefore handed every bridge the
REM   bundled physics interpreter, which has no `omnisim_bridges` on its
REM   sys.path -- so the deferred-intent tool layer and the shared
REM   status/resume intents fell back to their "package absent" stubs. The
REM   fallback is a bare `except Exception: pass`, and controller stdout is
REM   invisible on this binary (see the interpreter report below), so the
REM   demo came up quietly missing features with no message anywhere.
REM
REM Prepending was NEVER what made Newton work, so moving it costs nothing.
REM The engine's embedded interpreter is not resolved through PATH at all:
REM omnisim-bin.exe loads python312.dll from its OWN directory (Windows
REM searches the application directory BEFORE PATH), and the python312._pth
REM sitting beside that DLL switches CPython to isolated path config and
REM points sys.path at newton-runtime\{Lib,DLLs,site-packages}. Measured
REM 2026-07-28: msys64\mingw64\bin\python312.dll is byte-identical
REM (sha256 b6227a50...) to the newton-runtime copy, and 44 launches ran
REM through scripts\dev\headless_runner.py -- which never puts newton-runtime
REM on PATH -- with the backend-verdict sidecar inspected on five of them,
REM every one reading {"backend":"newton","degraded":false,"finalised":true,
REM "solver":"MuJoCo (cpu/mj_step, WorldInfo.newtonSolver)"} in
REM <log>.newton.json. Newton is unaffected by this line.
REM
REM Keeping it on the TAIL preserves the one real service it provides: on a
REM box with no system Python at all it is still found, so controllers start
REM (degraded) instead of every one of them dying "Python was not found".
if exist "%OMNISIM_HOME%\msys64\mingw64\bin\newton-runtime\python.exe" (
    set "PATH=%PATH%;%OMNISIM_HOME%\msys64\mingw64\bin\newton-runtime"
)

REM ── NAME THE CONTROLLER INTERPRETER, HERE, BEFORE THE ENGINE STARTS ─────────
REM This is the only place an operator can see it. omnisim-bin.exe is a Windows
REM GUI-SUBSYSTEM binary and WbLog::appendStdout / appendStderr (which is where
REM every controller print() and traceback goes) write to std::cout/std::cerr
REM and emit a Qt signal -- they never call WbLog::fileLog(), unlike
REM info()/warning()/error(). Measured: a full headless run of
REM warehouse_omnilink.omniworld produced a 0-byte stdout capture and a 0-byte stderr
REM capture while omnisim_log.txt filled normally. So a bridge that degrades,
REM or dies, says nothing you will ever read. Print the interpreter up front.
set "CTRL_PY="
for /f "delims=" %%P in ('where python 2^>nul') do if not defined CTRL_PY set "CTRL_PY=%%P"
if not defined CTRL_PY goto :no_ctrl_python
echo [OmniSim] controller interpreter: %CTRL_PY%
REM Pure-batch substring test (if deleting the needle changes the string, it
REM was there). NOT `echo ... ^| find`: this script runs from whatever shell
REM the operator has, and a Git-for-Windows / MSYS `find` earlier on PATH
REM shadows C:\Windows\System32\find.exe and errors out -- observed here.
if not "%CTRL_PY%"=="%CTRL_PY:newton-runtime=%" (
    echo [OmniSim] WARNING: that is the BUNDLED Newton interpreter, not a system Python.
    echo [OmniSim]          The OmniLink bridges will silently lose their omnisim_bridges
    echo [OmniSim]          extras ^(deferred-intent tools, shared status/resume intents^).
    echo [OmniSim]          Install Python 3 and put it on PATH before demoing.
)
goto :ctrl_python_done
:no_ctrl_python
echo [OmniSim] WARNING: no python.exe on PATH - every Python controller will fail to start.
:ctrl_python_done

REM No-args launch opens the OmniSim demo launcher: right-click the
REM floating orb robot -> "Show Robot Window" -> pick a demo from the
REM side-panel gallery and click Launch. Pass an explicit world path as
REM the first argument to skip the launcher (e.g. for headless runs).
set "DEFAULT_WORLD=%OMNISIM_HOME%\projects\samples\demos\worlds\omnilink_launcher.omniworld"

echo Starting OmniSim...
echo OMNISIM_HOME=%OMNISIM_HOME%
echo.
REM --stdout/--stderr matter beyond logging: without attached output
REM streams, the windows-subsystem binary can die during the embedded
REM CPython init of the v4 Newton probe (invalid stdio handles) before
REM the window even appears. Reproducible on worlds with physics; the
REM flags make the launch reliable and the engine log visible.
if "%~1"=="" (
    echo No world specified - opening the demo launcher.
    REM First-run install conformance gate: mandatory but bypassable, and
    REM FAILS OPEN -- only a deliberate FAIL (exit 42) blocks the launch.
    REM Clears instantly once a valid stamp exists for this build.
    REM Opt out on a dev box with:  python -m omnisim verify-install --never-ask
    call :conformance_gate
    if errorlevel 1 exit /b 1
    echo Right-click the orb robot in the scene to open the demo gallery.
    echo.
    "%OMNISIM_BIN%" "%DEFAULT_WORLD%" --stdout --stderr
) else (
    "%OMNISIM_BIN%" %* --stdout --stderr
)
set "RC=%ERRORLEVEL%"
echo.
echo Exit code: %RC%
exit /b %RC%

:conformance_gate
REM Run from OMNISIM_HOME so `python -m omnisim` resolves the package. Capture
REM the exit code BEFORE popd (popd resets ERRORLEVEL). Only 42 (a deliberate
REM conformance FAIL) blocks; missing python / import errors fall through.
pushd "%OMNISIM_HOME%"
python -m omnisim verify-install --gate
set "GATE_RC=%ERRORLEVEL%"
popd
if "%GATE_RC%"=="42" (
    echo.
    echo [OmniSim] Install conformance check FAILED - not launching.
    echo   Diagnose: python -m omnisim verify-install
    echo   Bypass:   set OMNISIM_SKIP_CONFORMANCE=1   then relaunch
    exit /b 1
)
exit /b 0
