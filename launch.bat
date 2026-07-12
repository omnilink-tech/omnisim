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

set "PATH=%OMNISIM_HOME%\msys64\mingw64\bin;%PATH%"

REM No-args launch opens the OmniSim demo launcher: right-click the
REM floating orb robot -> "Show Robot Window" -> pick a demo from the
REM side-panel gallery and click Launch. Pass an explicit world path as
REM the first argument to skip the launcher (e.g. for headless runs).
set "DEFAULT_WORLD=%OMNISIM_HOME%\projects\samples\demos\worlds\omnilink_launcher.wbt"

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
