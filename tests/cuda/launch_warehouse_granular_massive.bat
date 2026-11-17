@echo off
REM One-click launcher for the 10 000-particle "ball pit" variant.
REM Uses the uniform-grid broadphase to keep 10× the previous count
REM running real-time. Double-click in File Explorer to open.

setlocal
set "SCRIPT_DIR=%~dp0"
set "REPO_ROOT=%SCRIPT_DIR%..\.."
call "%REPO_ROOT%\launch.bat" "%SCRIPT_DIR%warehouse_husky_granular_massive.omniworld" --mode=fast
