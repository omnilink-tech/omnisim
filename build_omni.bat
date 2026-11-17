@echo off
REM Derive the install root from this script's location.
set "OMNISIM_HOME=%~dp0"
if "%OMNISIM_HOME:~-1%"=="\" set "OMNISIM_HOME=%OMNISIM_HOME:~0,-1%"
REM Pin the legacy alias too so a stale system WEBOTS_HOME (old Webots
REM install) cannot leak into sub-makes or spawned tools.
set "WEBOTS_HOME=%OMNISIM_HOME%"
if not defined MSYS64_HOME set "MSYS64_HOME=C:\msys64"
set "PATH=%MSYS64_HOME%\mingw64\bin;%MSYS64_HOME%\usr\bin;%PATH%"
cd /d "%OMNISIM_HOME%"
set "OMNISIM_HOME_FWD=%OMNISIM_HOME:\=/%"

REM Preflight: the two one-time setup steps a plain `git clone` does NOT do.
REM Without them the compiler still runs for a few minutes and then fails deep in
REM an unrelated file ("glm/glm.hpp: No such file", "QtCore/QObject: No such
REM file"), which reads like a broken repo rather than a missing setup step.
REM Say so up front instead. See docs/developer/quickstart.md sections 3 and 4.
if not exist "%OMNISIM_HOME%\src\glm\glm\glm.hpp" (
  echo.
  echo [build_omni] The git submodules are not checked out ^(src\glm is empty^).
  echo [build_omni] A plain "git clone" does not fetch them. Run:
  echo.
  echo     git submodule update --init --recursive
  echo.
  echo [build_omni] See docs\developer\quickstart.md section 4.
  exit /b 1
)
if not exist "%OMNISIM_HOME%\include\qt\QtCore\QtCore\QObject" (
  echo.
  echo [build_omni] The Qt6 header mirror at include\qt\ is missing.
  echo [build_omni] From an MSYS2 MINGW64 shell, run:
  echo.
  echo     for d in /mingw64/include/qt6/Qt*; do n=$^(basename $d^); \
  echo       mkdir -p include/qt/$n/$n; cp -r "$d/." include/qt/$n/$n/; done
  echo.
  echo [build_omni] See docs\developer\quickstart.md section 3.
  exit /b 1
)
REM Default to half the logical cores and below-normal priority so the
REM build never freezes the desktop (big C++ TUs are RAM-hungry; full
REM parallelism thrashes laptops). Override with MAKE_JOBS=N.
if not defined MAKE_JOBS set /a MAKE_JOBS=%NUMBER_OF_PROCESSORS%/2
if %MAKE_JOBS% LSS 1 set MAKE_JOBS=1
start "" /belownormal /b /wait "%MSYS64_HOME%\usr\bin\make.exe" OMNISIM_HOME=%OMNISIM_HOME_FWD% -j%MAKE_JOBS% %*
exit /b %ERRORLEVEL%
