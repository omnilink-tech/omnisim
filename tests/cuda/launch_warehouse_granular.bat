@echo off
REM One-click launcher for the warehouse-Husky-meets-granular-spheres demo.
REM Double-click this file in File Explorer to open it. Calls the canonical
REM launch.bat at the repo root so PATH + OMNISIM_HOME setup is identical.

setlocal
set "SCRIPT_DIR=%~dp0"
set "REPO_ROOT=%SCRIPT_DIR%..\.."
call "%REPO_ROOT%\launch.bat" "%SCRIPT_DIR%warehouse_husky_granular.omniworld" --mode=fast
