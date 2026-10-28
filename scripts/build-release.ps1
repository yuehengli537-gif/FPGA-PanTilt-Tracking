param(
    [string]$GowinHome = 'D:\goayun\Gowin\Gowin_V1.9.11.03_Education_x64',
    [string]$BuildDirectory = ''
)
$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
if (-not $BuildDirectory) { $BuildDirectory = Join-Path $root '.build\v8d' }
& (Join-Path $PSScriptRoot 'build.ps1') -GowinHome $GowinHome -BuildDirectory $BuildDirectory -CameraUart -ShapeDiagnostics
