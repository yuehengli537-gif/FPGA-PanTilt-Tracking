param([string]$IverilogDirectory = '')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
if (-not $IverilogDirectory) { $IverilogDirectory = Join-Path (Split-Path $projectRoot -Parent) '.tools\iverilog\bin' }
$compiler = Join-Path $IverilogDirectory 'iverilog.exe'
$runtime = Join-Path $IverilogDirectory 'vvp.exe'
if (-not (Test-Path -LiteralPath $compiler) -or -not (Test-Path -LiteralPath $runtime)) { throw 'Icarus tools not found.' }
$workPath = Join-Path $projectRoot 'reports\runtime\camera_servo_pixels'
New-Item -ItemType Directory -Force -Path $workPath | Out-Null
$runId = (Get-Date -Format 'yyyyMMdd_HHmmss') + '_' + [guid]::NewGuid().ToString('N').Substring(0,8)
$runPath = Join-Path $workPath $runId
New-Item -ItemType Directory -Path $runPath | Out-Null
$sources = @('uart_rx.v','uart_tx.v','servo_uart_control.v') | ForEach-Object { Join-Path $projectRoot ('servo_uart\src\' + $_) }
$sources += @('servo_measurement_cdc.v','camera_servo_uart.v','pan_tilt_controller.v',
    'red_ball_detector.v','red_ball_lock.v','red_ball_tracking.v','red_ball_overlay.v','display_spatial_ops.v') |
    ForEach-Object { Join-Path $projectRoot ('fpga\src\image_processing\' + $_) }
$sources += @('rgb565_to_gray.v','brightness_adjust.v','stream_window_3x3.v','gaussian_3x3.v',
    'sobel_3x3.v','spatial_filter_top.v','stream_delay.v') |
    ForEach-Object { Join-Path $projectRoot ('algorithm_sim\rtl\' + $_) }
$hostModel = Join-Path $projectRoot 'servo_uart\tb\uart_host_model.sv'
$bench = Join-Path $projectRoot 'algorithm_sim\tb\tb_camera_servo_pixels.sv'
$inputs = @($sources) + @($hostModel,$bench)
$hashes = @{}
foreach ($source in $inputs) { $hashes[$source] = (Get-FileHash -LiteralPath $source -Algorithm SHA256).Hash }
$simFile = Join-Path $runPath 'tb_camera_servo_pixels.vvp'
if (Test-Path -LiteralPath $simFile) { throw 'Refusing to reuse a simulation artifact.' }
$result = [ordered]@{
    simulation_passed=$false; status='running'; run_directory=$runPath;
    compiler_exit_code=$null; compiler_clean=$false; compiler_known_exit_anomaly=$false;
    simulation_exit_code=$null; source_sha256=$hashes
}
function Save-Result {
    $result | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $runPath 'result.json') -Encoding utf8
    $result | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $workPath 'result.json') -Encoding utf8
}
Save-Result
try {
    # Preserve the known native driver exit anomaly, as in test-camera-servo-uart.
    $ErrorActionPreference = 'Continue'
    & $compiler -g2012 -Wall -s tb_camera_servo_pixels -o $simFile @sources $hostModel $bench 2>&1 |
        Tee-Object -FilePath (Join-Path $runPath 'compile.log')
    $compileExit = $LASTEXITCODE
    $ErrorActionPreference = 'Stop'
    $knownDriverCrash = ($compileExit -eq -1073741819)
    $result.compiler_exit_code=$compileExit
    $result.compiler_clean=($compileExit -eq 0)
    $result.compiler_known_exit_anomaly=$knownDriverCrash
    if ($compileExit -ne 0 -and -not $knownDriverCrash) { throw ('Compile failed, exit=' + $compileExit) }
    if (-not (Test-Path -LiteralPath $simFile)) { throw 'No fresh VVP was generated.' }
    if ($knownDriverCrash) {
        $vvpText = Get-Content -Raw -LiteralPath $simFile
        $footer = [regex]::Match($vvpText, '(?ms)^:file_names ([0-9]+);\r?\n(?<table>.*)\z')
        if (-not $footer.Success) { throw 'Incomplete VVP after compiler crash.' }
        $fileLines = [regex]::Matches($footer.Groups['table'].Value, '(?m)^\s+"[^"]*";\r?$')
        if ($fileLines.Count -ne [int]$footer.Groups[1].Value) { throw 'Truncated VVP footer.' }
        Write-Warning 'Icarus driver exit 0xC0000005; fresh complete VVP will be simulated and anomaly retained.'
    }
    $result.vvp_sha256=(Get-FileHash -LiteralPath $simFile -Algorithm SHA256).Hash
    Save-Result
    $ErrorActionPreference = 'Continue'
    $simulation = & $runtime $simFile 2>&1 | Tee-Object -FilePath (Join-Path $runPath 'simulation.log')
    $simulationExit = $LASTEXITCODE
    $ErrorActionPreference = 'Stop'
    $result.simulation_exit_code=$simulationExit
    if ($simulationExit -ne 0 -or ($simulation -join "`n") -notlike '*PASS tb_camera_servo_pixels:*') {
        throw ('Simulation failed or required PASS marker missing, exit=' + $simulationExit)
    }
    foreach ($source in $inputs) {
        if ((Get-FileHash -LiteralPath $source -Algorithm SHA256).Hash -ne $hashes[$source]) {
            throw ('Input changed during simulation: ' + $source)
        }
    }
    $result.simulation_passed=$true
    $result.status='passed'
    Write-Output ('PASS complete pixel-to-UART-PWM simulation. Evidence: ' + $runPath)
} catch {
    $result.status='failed'
    $result.error=$_.Exception.Message
    throw
} finally {
    $result.completed_at=(Get-Date).ToString('o')
    Save-Result
}
