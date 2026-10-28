@echo off
cd /d "%~dp0"
where pwsh.exe >nul 2>nul
if errorlevel 1 (
  echo Install PowerShell 7 first, then run this launcher again.
  pause
  exit /b 1
)
echo Disconnect servo 5V and close the UI. This loads V8d into SRAM only.
pause
pwsh.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\download-red-ball-contour.ps1"
pause
