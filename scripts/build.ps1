param(
    [string]$GowinHome = 'D:\gaoyun\Gowin\Gowin_V1.9.11.03_Education_x64',
    [string]$BuildDirectory = 'C:\fpga_work\tang_primer20k_camera',
    [switch]$EnableServo,
    [switch]$CameraUart,
    [switch]$ShapeDiagnostics,
    [switch]$ServoDiagnostics,
    [ValidateSet('Tracking','Neutral','Limited')]
    [string]$ServoProfile = 'Tracking',
    [ValidateSet('Both','Pan','Tilt')]
    [string]$ServoAxes = 'Both',
    [switch]$ReversePan,
    [switch]$ReverseTilt,
    [ValidateRange(1300,1500)][int]$LimitedPanMinUs = 1460,
    [ValidateRange(1500,1700)][int]$LimitedPanMaxUs = 1540,
    [ValidateRange(1300,1500)][int]$LimitedTiltMinUs = 1460,
    [ValidateRange(1500,1700)][int]$LimitedTiltMaxUs = 1540
)
$ErrorActionPreference = 'Stop'
if ($ShapeDiagnostics -and -not $CameraUart) { throw 'ShapeDiagnostics requires CameraUart (PWM remains disabled at boot).' }
if ($CameraUart -and ($EnableServo -or $ServoDiagnostics -or $ServoProfile -ne 'Tracking' -or $ServoAxes -ne 'Both')) {
    throw 'CameraUart owns both PWM outputs; do not combine with legacy servo options.'
}
if (-not $EnableServo -and -not $CameraUart -and ($ServoDiagnostics -or $ServoProfile -ne 'Tracking' -or $ServoAxes -ne 'Both' -or $ReversePan -or $ReverseTilt)) {
    throw 'Servo profile, axes and direction options require -EnableServo.'
}
foreach ($limitOption in @('LimitedPanMinUs','LimitedPanMaxUs','LimitedTiltMinUs','LimitedTiltMaxUs')) {
    if ($PSBoundParameters.ContainsKey($limitOption) -and (-not $EnableServo -or $ServoProfile -ne 'Limited')) {
        throw 'Explicit Limited pulse limits require -EnableServo -ServoProfile Limited.'
    }
}
$servoMode = @{ Tracking = 0; Neutral = 1; Limited = 2 }[$ServoProfile]
$servoMask = @{ Both = 3; Pan = 1; Tilt = 2 }[$ServoAxes]
$compiler = Join-Path $GowinHome 'IDE\bin\gw_sh.exe'
if (-not (Test-Path -LiteralPath $compiler)) { throw 'Set -GowinHome to your installed Gowin directory.' }
$buildPath = [IO.Path]::GetFullPath($BuildDirectory)
if ($buildPath -match '[^\x00-\x7F]') { throw 'Use an ASCII build directory; Gowin has path encoding issues.' }
$projectRoot = Split-Path $PSScriptRoot -Parent
$sourceDirectory = Join-Path $projectRoot 'fpga'
New-Item -ItemType Directory -Path $buildPath -Force | Out-Null
Copy-Item -Path (Join-Path $sourceDirectory '*') -Destination $buildPath -Recurse -Force
if ($CameraUart) {
    $stagedTopFile = Join-Path $buildPath 'src\top.v'
    $stagedTop = Get-Content -Raw -LiteralPath $stagedTopFile
    $defaultSetting = 'parameter integer SERVO_UART_ENABLE = 0'
    if (-not $stagedTop.Contains($defaultSetting)) { throw 'Missing safe UART default.' }
    $stagedTop = $stagedTop.Replace($defaultSetting, 'parameter integer SERVO_UART_ENABLE = 1')
    foreach ($setting in @(@('SERVO_PAN_REVERSE', [int][bool]$ReversePan), @('SERVO_TILT_REVERSE', [int][bool]$ReverseTilt))) {
        $original = 'parameter integer ' + $setting[0] + ' = 0'
        if (-not $stagedTop.Contains($original)) { throw ('Missing default: ' + $original) }
        $stagedTop = $stagedTop.Replace($original, ('parameter integer ' + $setting[0] + ' = ' + $setting[1]))
    }
    [IO.File]::WriteAllText($stagedTopFile, $stagedTop, [Text.UTF8Encoding]::new($false))
}
if ($ShapeDiagnostics) {
    $stagedTopFile = Join-Path $buildPath 'src\top.v'
    $stagedTop = Get-Content -Raw -LiteralPath $stagedTopFile
    $oldDebug = '.SHOW_SERVO_DEBUG(SERVO_UART_ENABLE ? 0 : SERVO_DIAGNOSTICS)'
    if (-not $stagedTop.Contains($oldDebug)) { throw 'Missing expected diagnostic parameter mapping.' }
    $stagedTop = $stagedTop.Replace($oldDebug, '.SHOW_SERVO_DEBUG(2)')
    [IO.File]::WriteAllText($stagedTopFile, $stagedTop, [Text.UTF8Encoding]::new($false))
}
if ($EnableServo) {
    $stagedTopFile = Join-Path $buildPath 'src\top.v'
    $stagedTop = Get-Content -Raw -LiteralPath $stagedTopFile
    $disabledSetting = 'parameter integer SERVO_ENABLE = 0'
    if (-not $stagedTop.Contains($disabledSetting)) { throw 'Expected safe SERVO_ENABLE=0 default was not found.' }
    $stagedTop = $stagedTop.Replace($disabledSetting, 'parameter integer SERVO_ENABLE = 1')
    foreach ($setting in @(
        @('SERVO_DIAGNOSTICS', 0, [int][bool]$ServoDiagnostics),
        @('SERVO_TEST_MODE', 0, $servoMode),
        @('SERVO_AXIS_MASK', 3, $servoMask),
        @('SERVO_PAN_REVERSE', 0, [int][bool]$ReversePan),
        @('SERVO_TILT_REVERSE', 0, [int][bool]$ReverseTilt),
        @('SERVO_LIMITED_PAN_MIN_US', 1460, $LimitedPanMinUs),
        @('SERVO_LIMITED_PAN_MAX_US', 1540, $LimitedPanMaxUs),
        @('SERVO_LIMITED_TILT_MIN_US', 1460, $LimitedTiltMinUs),
        @('SERVO_LIMITED_TILT_MAX_US', 1540, $LimitedTiltMaxUs)
    )) {
        $original = 'parameter integer ' + $setting[0] + ' = ' + $setting[1]
        if (-not $stagedTop.Contains($original)) { throw ('Missing default: ' + $original) }
        $stagedTop = $stagedTop.Replace($original, ('parameter integer ' + $setting[0] + ' = ' + $setting[2]))
    }
    [IO.File]::WriteAllText($stagedTopFile, $stagedTop, [Text.UTF8Encoding]::new($false))
}
[ordered]@{
    camera_uart = [bool]$CameraUart
    shape_diagnostic = [bool]$ShapeDiagnostics
    show_servo_debug = $(if ($ShapeDiagnostics) { 2 } elseif ($ServoDiagnostics) { 1 } else { 0 })
    servo_enabled_at_boot = [bool]$EnableServo
    servo_enabled = [bool]$EnableServo
    servo_diagnostics = [bool]$ServoDiagnostics
    servo_profile = $ServoProfile
    servo_axes = $ServoAxes
    reverse_pan = [bool]$ReversePan
    reverse_tilt = [bool]$ReverseTilt
    limited_pan_min_us = $LimitedPanMinUs
    limited_pan_max_us = $LimitedPanMaxUs
    limited_tilt_min_us = $LimitedTiltMinUs
    limited_tilt_max_us = $LimitedTiltMaxUs
    source_project = $projectRoot
} |
    ConvertTo-Json | Set-Content -LiteralPath (Join-Path $buildPath 'build_variant.json') -Encoding UTF8
# Stage shared B arithmetic RTL inside the ASCII build directory. The source
# project references these files directly, so simulation and hardware use one copy.
$stagedProjectFile = Join-Path $buildPath 'OV5640_HDMI720P.gprj'
$stagedProject = Get-Content -Raw -LiteralPath $stagedProjectFile
$sharedDirectory = Join-Path $buildPath 'src\shared_algorithm'
foreach ($relativeSource in @('../algorithm_sim/rtl/rgb565_to_gray.v', '../algorithm_sim/rtl/brightness_adjust.v',
    '../algorithm_sim/rtl/stream_window_3x3.v', '../algorithm_sim/rtl/gaussian_3x3.v',
    '../algorithm_sim/rtl/sobel_3x3.v', '../algorithm_sim/rtl/spatial_filter_top.v', '../algorithm_sim/rtl/stream_delay.v',
    '../servo_uart/src/uart_rx.v', '../servo_uart/src/uart_tx.v', '../servo_uart/src/servo_uart_control.v')) {
    $sourceAttribute = 'path="' + $relativeSource + '"'
    if ($stagedProject.Contains($sourceAttribute)) {
        New-Item -ItemType Directory -Path $sharedDirectory -Force | Out-Null
        $sharedSource = [IO.Path]::GetFullPath((Join-Path $sourceDirectory $relativeSource))
        $sharedName = [IO.Path]::GetFileName($sharedSource)
        Copy-Item -LiteralPath $sharedSource -Destination (Join-Path $sharedDirectory $sharedName) -Force
        $stagedProject = $stagedProject.Replace($sourceAttribute, ('path="src/shared_algorithm/' + $sharedName + '"'))
    }
}
# Preserve Gowin's nonstandard version="1" declaration rather than parsing as XML.
[IO.File]::WriteAllText($stagedProjectFile, $stagedProject, [Text.UTF8Encoding]::new($false))
Push-Location -LiteralPath $buildPath
try {
    & $compiler 'build.tcl'
    if ($LASTEXITCODE -ne 0) { throw 'Gowin compilation failed; inspect compiler diagnostics.' }
} finally { Pop-Location }
Write-Output "Build output: $buildPath\impl\pnr\OV5640_HDMI720P.fs"
Write-Output 'The verified recovery bitstream is stored separately in bitstreams/.'
