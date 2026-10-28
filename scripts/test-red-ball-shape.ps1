param(
    [string]$IverilogDirectory = '',
    [string[]]$Only = @(),
    [switch]$Full720p,
    [switch]$Smoke720p
)
$ErrorActionPreference = 'Stop'
# powershell -File receives a comma list as one argument; support it explicitly.
$Only = @($Only | ForEach-Object { $_ -split ',' } | Where-Object { $_ })
$projectRoot = Split-Path $PSScriptRoot -Parent
if (-not $IverilogDirectory) { $IverilogDirectory = Join-Path (Split-Path $projectRoot -Parent) '.tools\iverilog\bin' }
$compiler = Join-Path $IverilogDirectory 'iverilog.exe'
$runtime = Join-Path $IverilogDirectory 'vvp.exe'
if (-not (Test-Path -LiteralPath $compiler) -or -not (Test-Path -LiteralPath $runtime)) { throw 'Icarus tools not found.' }
$workPath = Join-Path $projectRoot 'reports\runtime\red_ball_shape'
New-Item -ItemType Directory -Force -Path $workPath | Out-Null
$runId = (Get-Date -Format 'yyyyMMdd_HHmmss') + '_' + [guid]::NewGuid().ToString('N').Substring(0,8)
$runPath = Join-Path $workPath $runId
New-Item -ItemType Directory -Path $runPath | Out-Null
$tests = @(
    @{name='shape'; top='tb_red_ball_shape'; parameters=@{}; marker='PASS red_ball_shape:'},
    @{name='contour'; top='tb_red_ball_contour'; parameters=@{}; marker='PASS red_ball_contour:'},
    @{name='contour3'; top='tb_red_ball_contour'; parameters=@{CANDIDATES=3}; marker='PASS red_ball_contour:'},
    @{name='minrun'; top='tb_red_ball_minrun'; parameters=@{}; marker='PASS red_ball_minrun:'},
    @{name='merge'; top='tb_red_ball_merge'; parameters=@{}; marker='PASS red_ball_merge'},
    @{name='merge3'; top='tb_red_ball_merge'; parameters=@{CANDIDATES=3}; marker='PASS red_ball_merge'},
    @{name='detector64'; top='tb_red_ball_detector'; parameters=@{WIDTH=64;HEIGHT=48}; marker='PASS red ball detector'},
    @{name='detector160'; top='tb_red_ball_detector'; parameters=@{WIDTH=160;HEIGHT=90}; marker='PASS red ball detector'},
    @{name='multicolor'; top='tb_red_ball_multicolor'; parameters=@{}; marker='PASS multicolor:'},
    @{name='overlay'; top='tb_red_ball_overlay'; parameters=@{}; marker='PASS red_ball_overlay:'},
    @{name='lock'; top='tb_red_ball_lock'; parameters=@{}; marker='PASS red_ball_lock'},
    @{name='lock_single'; top='tb_red_ball_lock'; parameters=@{SINGLE=1}; marker='PASS red_ball_lock'},
    @{name='pipeline320'; top='tb_red_ball_pipeline'; parameters=@{}; marker='PASS pipeline'},
    @{name='follow_model'; top='tb_red_ball_follow'; parameters=@{}; marker='PASS ideal image-space follow model:'}
)
if ($Full720p) {
    $tests += @{name='detector720p'; top='tb_red_ball_detector'; parameters=@{WIDTH=1280;HEIGHT=720}; marker='PASS red ball detector'}
    $tests += @{name='pipeline720p'; top='tb_red_ball_pipeline'; parameters=@{WIDTH=1280;HEIGHT=720;H_TOTAL=1650;FRAME_COUNT=11}; marker='PASS pipeline'}
}
if ($Smoke720p) {
    $tests += @{name='merge720p_smoke'; top='tb_red_ball_merge'; parameters=@{WIDTH=1280;HEIGHT=720;SMOKE=1}; marker='PASS red_ball_merge'}
    $tests += @{name='pipeline720p_smoke'; top='tb_red_ball_pipeline'; parameters=@{WIDTH=1280;HEIGHT=720;H_TOTAL=1650;FRAME_COUNT=4}; marker='PASS pipeline'}
}
if ($Only.Count -gt 0) {
    $known = @($tests | ForEach-Object { $_.name })
    foreach ($name in $Only) { if ($name -notin $known) { throw ('Unknown test: ' + $name) } }
    $tests = @($tests | Where-Object { $_.name -in $Only })
}
$sourceFiles = @(Get-ChildItem -LiteralPath (Join-Path $projectRoot 'algorithm_sim\rtl') -Filter '*.v')
$sourceFiles += @(Get-ChildItem -LiteralPath (Join-Path $projectRoot 'fpga\src\image_processing') -Filter '*.v')
$sourceFiles += @(Get-ChildItem -LiteralPath (Join-Path $projectRoot 'algorithm_sim\tb') -Filter 'tb_red_ball*.sv')
$hashes = @{}
foreach ($source in $sourceFiles) {
    $hashes[$source.FullName] = (Get-FileHash -LiteralPath $source.FullName -Algorithm SHA256).Hash
    Copy-Item -LiteralPath $source.FullName -Destination (Join-Path $runPath $source.Name)
}
$sourceNames = @($sourceFiles | ForEach-Object { $_.Name })
$result = [ordered]@{simulation_passed=$false;status='running';run_directory=$runPath;tests=@();source_sha256=$hashes}
function Save-Result {
    $json = $result | ConvertTo-Json -Depth 8
    [IO.File]::WriteAllText((Join-Path $runPath 'result.json'),$json+[Environment]::NewLine,[Text.UTF8Encoding]::new($false))
    [IO.File]::WriteAllText((Join-Path $workPath 'result.json'),$json+[Environment]::NewLine,[Text.UTF8Encoding]::new($false))
}
Save-Result
Push-Location $runPath
try {
    foreach ($test in $tests) {
        $name = $test.name
        Write-Output ('RUN '+$name)
        $simFile = $name + '.vvp'
        if (Test-Path -LiteralPath $simFile) { throw 'Refusing stale simulation executable.' }
        $arguments = @('-g2012','-Wall','-s',$test.top,'-o',$simFile)
        foreach ($key in ($test.parameters.Keys | Sort-Object)) { $arguments += ('-P'+$test.top+'.'+$key+'='+$test.parameters[$key]) }
        $arguments += $sourceNames
        $entry = [ordered]@{name=$name;parameters=$test.parameters;passed=$false;compiler_exit_code=$null;compiler_clean=$false;compiler_known_exit_anomaly=$false;simulation_exit_code=$null}
        $ErrorActionPreference = 'Continue'
        & $compiler @arguments 2>&1 | Out-File -LiteralPath ($name+'_compile.log') -Encoding utf8
        $compileExit = $LASTEXITCODE
        $ErrorActionPreference = 'Stop'
        $entry.compiler_exit_code=$compileExit
        $entry.compiler_clean=($compileExit -eq 0)
        $entry.compiler_known_exit_anomaly=($compileExit -eq -1073741819)
        $result.tests += $entry
        Save-Result
        if ($compileExit -ne 0 -and -not $entry.compiler_known_exit_anomaly) { throw ('Compile failed: '+$name+' exit='+$compileExit) }
        if (-not (Test-Path -LiteralPath $simFile)) { throw ('No fresh VVP for '+$name) }
        if ($entry.compiler_known_exit_anomaly) {
            $vvpText = Get-Content -LiteralPath $simFile -Raw
            $footer = [regex]::Match($vvpText, '(?ms)^:file_names ([0-9]+);\r?\n(?<table>.*)\z')
            if (-not $footer.Success) { throw 'Incomplete VVP after compiler crash.' }
            $fileLines = [regex]::Matches($footer.Groups['table'].Value, '(?m)^\s+"[^"]*";\r?$')
            if ($fileLines.Count -ne [int]$footer.Groups[1].Value) { throw 'Truncated VVP footer.' }
            Write-Warning 'Icarus driver exit 0xC0000005; fresh complete VVP will be simulated; compiler anomaly retained.'
        }
        $entry.vvp_sha256=(Get-FileHash -LiteralPath $simFile -Algorithm SHA256).Hash
        $ErrorActionPreference = 'Continue'
        $simulation = & $runtime $simFile 2>&1 | Tee-Object -FilePath ($name+'_simulation.log')
        $simulationExit = $LASTEXITCODE
        $ErrorActionPreference = 'Stop'
        $entry.simulation_exit_code=$simulationExit
        $text = $simulation -join [Environment]::NewLine
        if ($simulationExit -ne 0 -or -not $text.Contains($test.marker)) { throw ('Simulation failed or PASS missing: '+$name+' exit='+$simulationExit) }
        $entry.passed=$true
        $entry.messages=@($simulation | Where-Object { $_ -like 'PASS*' -or $_ -like 'LIMITATION:*' -or $_ -like 'KNOWN LIMITATION:*' })
        $trace = switch ($test.top) {
            'tb_red_ball_shape' {'shape_trace.csv'}
            'tb_red_ball_contour' {'contour_trace.csv'}
            'tb_red_ball_merge' {'merge_trace.csv'}
            'tb_red_ball_pipeline' {'occlusion_trace.csv'}
            'tb_red_ball_multicolor' {'multicolor_trace.csv'}
            'tb_red_ball_follow' {'follow_model_trace.csv'}
            default {$null}
        }
        if ($trace -and (Test-Path -LiteralPath $trace)) { Copy-Item -LiteralPath $trace -Destination ($name+'_'+$trace) }
        Save-Result
    }
    foreach ($source in $sourceFiles) {
        if ((Get-FileHash -LiteralPath $source.FullName -Algorithm SHA256).Hash -ne $hashes[$source.FullName]) { throw ('Input changed during verification: '+$source.FullName) }
    }
    $result.simulation_passed=$true
    $result.status='passed'
    Write-Output ('PASS shape and regression tests. Evidence: '+$runPath)
} catch {
    $result.status='failed'
    $result.error=$_.Exception.Message
    throw
} finally {
    Pop-Location
    $result.completed_at=(Get-Date).ToString('o')
    Save-Result
}
