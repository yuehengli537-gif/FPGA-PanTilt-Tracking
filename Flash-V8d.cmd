@echo off
cd /d "%~dp0"
where pwsh.exe >nul 2>nul
if errorlevel 1 (
  echo Install PowerShell 7 first, then run this launcher again.
  pause
  exit /b 1
)
echo Disconnect servo 5V and close the UI. This OVERWRITES the Flash boot image with V8d.
pause
pwsh.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\flash-red-ball-contour.ps1"
pause
