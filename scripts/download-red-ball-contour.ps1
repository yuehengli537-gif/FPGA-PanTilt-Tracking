param(
    [string]$GowinHome = 'D:\goayun\Gowin\Gowin_V1.9.11.03_Education_x64',
    [int]$CableIndex = 4,
    [int]$Channel = 0,
    [ValidateSet('0.5MHz','2.5MHz')][string]$Frequency = '0.5MHz'
)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
$archive = Join-Path $projectRoot 'bitstreams\camera_uart_shape_diag_v8d_20261028.zip'
$restorePath = Join-Path $projectRoot 'reports\runtime\camera_contour_v8d_restore'
Write-Output 'Keep servo 5V off, close the UI, connect USB-JTAG and HDMI monitor before programming.'
New-Item -ItemType Directory -Force -Path $restorePath | Out-Null
Expand-Archive -LiteralPath $archive -DestinationPath $restorePath -Force
& (Join-Path $PSScriptRoot 'download-built.ps1') -GowinHome $GowinHome -CableIndex $CableIndex -Channel $Channel -Frequency $Frequency -BitstreamFile (Join-Path $restorePath 'OV5640_HDMI720P.fs') -ExpectedSHA256 'BEC75258735164B027CC1262CC8FB56B59B3E12350BF323288FB8FFC85606456'
Write-Output 'V8d six-direction diagnostic loaded in SRAM. Orange is only a raw red region; green is a confirmed target. PWM starts disabled; Flash unchanged.'
