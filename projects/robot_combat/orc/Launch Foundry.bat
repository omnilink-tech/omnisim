@echo off
setlocal
where python >nul 2>nul
if not errorlevel 1 (
  python "%~dp0launch_foundry.py" %*
  goto done
)
if exist "%LOCALAPPDATA%\Programs\Python\Python312\python.exe" (
  "%LOCALAPPDATA%\Programs\Python\Python312\python.exe" "%~dp0launch_foundry.py" %*
  goto done
)
where py >nul 2>nul
if not errorlevel 1 (
  py -3 "%~dp0launch_foundry.py" %*
  goto done
)
echo Python was not found. Run launch_foundry.py with your Python installation.
:done
pause
