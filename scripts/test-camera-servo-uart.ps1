param([string]$IverilogDirectory = '')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
if (-not $IverilogDirectory) { $IverilogDirectory = Join-Path (Split-Path $projectRoot -Parent) '.tools\iverilog\bin' }
$compiler = Join-Path $IverilogDirectory 'iverilog.exe'
$runtime = Join-Path $IverilogDirectory 'vvp.exe'
if (-not (Test-Path -LiteralPath $compiler) -or -not (Test-Path -LiteralPath $runtime)) { throw 'Icarus tools not found.' }
$workPath = Join-Path $projectRoot 'reports\runtime\camera_servo_uart'
New-Item -ItemType Directory -Force -Path $workPath | Out-Null
$runId = (Get-Date -Format 'yyyyMMdd_HHmmss') + '_' + [guid]::NewGuid().ToString('N').Substring(0,8)
$runPath = Join-Path $workPath $runId
New-Item -ItemType Directory -Path $runPath | Out-Null
$sources = @('uart_rx.v','uart_tx.v','servo_uart_control.v') | ForEach-Object { Join-Path $projectRoot ('servo_uart\src\' + $_) }
$sources += @('servo_measurement_cdc.v','camera_servo_uart.v') | ForEach-Object { Join-Path $projectRoot ('fpga\src\image_processing\' + $_) }
$hostModel = Join-Path $projectRoot 'servo_uart\tb\uart_host_model.sv'
$tests = @(
    @{Name='tb_servo_measurement_cdc'; Marker='PASS tb_servo_measurement_cdc:'},
    @{Name='tb_camera_servo_uart'; Marker='PASS tb_camera_servo_uart:'}
)
$hashes = @{}
foreach ($source in $sources) { $hashes[[IO.Path]::GetFileName($source)] = (Get-FileHash -LiteralPath $source -Algorithm SHA256).Hash }
$results = @()
foreach ($test in $tests) {
    $simFile = Join-Path $runPath ($test.Name + '.vvp')
    if (Test-Path -LiteralPath $simFile) { throw 'Refusing to reuse a simulation artifact.' }
    $bench = Join-Path $projectRoot ('algorithm_sim\tb\' + $test.Name + '.sv')
    # Windows PowerShell 5 treats native stderr warnings as ErrorRecords.
    # Capture them without terminating; exit code + fresh artifact are checked below.
    $ErrorActionPreference = 'Continue'
    & $compiler -g2012 -Wall -s $test.Name -o $simFile @sources $hostModel $bench 2>&1 |
        Tee-Object -FilePath (Join-Path $runPath ($test.Name + '_compile.log'))
    $compileExit = $LASTEXITCODE
    $ErrorActionPreference = 'Stop'
    $knownDriverCrash = ($compileExit -eq -1073741819)
    if ($compileExit -ne 0 -and -not $knownDriverCrash) { throw ('Compile failed: ' + $test.Name + ', exit=' + $compileExit) }
    if (-not (Test-Path -LiteralPath $simFile)) { throw ('No fresh VVP was generated: ' + $test.Name) }
    # This machine's Icarus12 driver can exit 0xC0000005 after emitting a full
    # VVP (also reproduced with the old verified servo RTL). Do not mask it.
    # Continue only for this known code, a new artifact, and a complete footer;
    # actual simulation must independently exit zero and emit its PASS marker.
    if ($knownDriverCrash) {
        $vvpText = Get-Content -Raw -LiteralPath $simFile
        $footer = [regex]::Match($vvpText, '(?ms)^:file_names ([0-9]+);\r?\n(?<table>.*)\z')
        if (-not $footer.Success) { throw ('Incomplete VVP after compiler crash: ' + $test.Name) }
        $fileLines = [regex]::Matches($footer.Groups['table'].Value, '(?m)^\s+"[^"]*";\r?$')
        if ($fileLines.Count -ne [int]$footer.Groups[1].Value) { throw ('Truncated VVP footer: ' + $test.Name) }
        Write-Warning ('Icarus driver exit 0xC0000005 for ' + $test.Name + '; fresh complete VVP will be simulated, compiler anomaly retained in results.')
    }
    $ErrorActionPreference = 'Continue'
    $simulation = & $runtime $simFile 2>&1 | Tee-Object -FilePath (Join-Path $runPath ($test.Name + '.log'))
    $simulationExit = $LASTEXITCODE
    $ErrorActionPreference = 'Stop'
    if ($simulationExit -ne 0 -or ($simulation -join "`n") -notlike ('*' + $test.Marker + '*')) {
        throw ('Simulation failed or required PASS marker missing: ' + $test.Name + ', exit=' + $simulationExit)
    }
    $results += [ordered]@{
        name=$test.Name; compiler_exit_code=$compileExit; compiler_clean=($compileExit -eq 0);
        compiler_known_exit_anomaly=$knownDriverCrash; simulation_exit_code=$simulationExit;
        simulation_passed=$true; vvp_sha256=(Get-FileHash -LiteralPath $simFile -Algorithm SHA256).Hash;
        testbench_sha256=(Get-FileHash -LiteralPath $bench -Algorithm SHA256).Hash
    }
}
foreach ($source in $sources) {
    if ((Get-FileHash -LiteralPath $source -Algorithm SHA256).Hash -ne $hashes[[IO.Path]::GetFileName($source)]) {
        throw 'RTL changed during simulation; result cannot be certified.'
    }
}
$result = [ordered]@{
    simulation_passed=$true; compiler_clean=(@($results | Where-Object { -not $_.compiler_clean }).Count -eq 0);
    run_directory=$runPath; tests=$results; source_sha256=$hashes;
    host_model_sha256=(Get-FileHash -LiteralPath $hostModel -Algorithm SHA256).Hash;
    completed_at=(Get-Date).ToString('o')
}
$result | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $runPath 'result.json') -Encoding utf8
$result | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $workPath 'result.json') -Encoding utf8
Write-Output ('PASS camera UART CDC/integration simulations; compiler exit codes recorded. Evidence: ' + $runPath)
