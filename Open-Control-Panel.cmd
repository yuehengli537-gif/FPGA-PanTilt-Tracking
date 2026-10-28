@echo off
cd /d "%~dp0"
where pwsh.exe >nul 2>nul
if errorlevel 1 (
  echo Install PowerShell 7 first, then run this launcher again.
  pause
  exit /b 1
)
pwsh.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\start-servo-ui.ps1"
if errorlevel 1 pause
