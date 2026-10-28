param(
    [string]$GowinHome = 'D:\goayun\Gowin\Gowin_V1.9.11.03_Education_x64',
    [int]$CableIndex = 4,
    [int]$Channel = 0,
    [switch]$CheckOnly
)
$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$programmer = Join-Path $GowinHome 'Programmer\bin\programmer_cli.exe'
if (-not (Test-Path -LiteralPath $programmer)) { throw 'Set -GowinHome to the installed Gowin directory.' }
$archive = Join-Path $root 'bitstreams\camera_uart_shape_diag_v8d_20261028.zip'
$expectedHash = 'BEC75258735164B027CC1262CC8FB56B59B3E12350BF323288FB8FFC85606456'
$report = Join-Path $root ('reports\runtime\flash_contour_' + (Get-Date -Format 'yyyyMMdd_HHmmss_fff'))
New-Item -ItemType Directory -Force -Path $report | Out-Null
Expand-Archive -LiteralPath $archive -DestinationPath (Join-Path $report 'image')
$fs = Join-Path $report 'image\OV5640_HDMI720P.fs'
if ($fs -match '[^\x00-\x7F]') { throw 'Use an ASCII project path for the programmer.' }
$hash = (Get-FileHash -LiteralPath $fs -Algorithm SHA256).Hash
if ($hash -ne $expectedHash) { throw 'Bitstream SHA256 mismatch; no programming attempted.' }
Write-Output ('V8d SHA256: ' + $hash)
Write-Output ('Report: ' + $report)
if ($CheckOnly) { Write-Output 'File checked; no hardware accessed.'; return }

$common = @('--device','GW2A-18C','--cable-index',"$CableIndex",'--channel',"$Channel",'--frequency','0.5MHz')
function Invoke-LoggedProgrammer([string]$Name, [string[]]$CliArguments, [int]$TimeoutSeconds = 20) {
    $stdout = Join-Path $report ($Name + '.stdout.log')
    $stderr = Join-Path $report ($Name + '.stderr.log')
    $arguments = ($CliArguments | ForEach-Object {
        if ($_ -match '["\r\n]') { throw 'Invalid programmer argument.' }
        '"' + $_ + '"'
    }) -join ' '
    $process = Start-Process -FilePath $programmer -ArgumentList $arguments -WindowStyle Hidden -PassThru -RedirectStandardOutput $stdout -RedirectStandardError $stderr
    $timedOut = -not $process.WaitForExit($TimeoutSeconds * 1000)
    if ($timedOut) { $process.Kill($true); $process.WaitForExit() }
    $process.Refresh()
    @{ pid=$process.Id; exit_code=$process.ExitCode; timed_out=$timedOut; timeout_seconds=$TimeoutSeconds } | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $report ($Name + '.process.json')) -Encoding utf8
    $joined = (Get-Content -LiteralPath $stdout -Raw) + "`n" + (Get-Content -LiteralPath $stderr -Raw)
    Write-Host $joined
    if ($timedOut -or $process.ExitCode -ne 0 -or $joined -match '(?i)\bError\b|\bfailed\b|mismatch|not found|0xFFFFFFFF') {
        throw ('Programmer failed at ' + $Name + '; inspect ' + $report + '. Do not assume the boot image is valid after an interrupted write.')
    }
    return $joined
}
Write-Output 'PERMANENT V8d FLASH UPDATE. Keep servo 5V OFF, UI closed and board USB power stable.'
$scan = Invoke-LoggedProgrammer 'scan' ($common + @('--scan'))
if ($scan -notmatch '0x0000081B' -or $scan -notmatch '\b1 device\(s\) found!') { throw 'Expected single GW2A-18C not found; no Flash write.' }
$identity = Invoke-LoggedProgrammer 'flash_id' ($common + @('--operation_index','51'))
if ($identity -notmatch 'Detected Flash id:\s*(0x[0-9A-Fa-f]+)') { throw 'Flash ID unreadable; no Flash write.' }
$flashId = $Matches[1]
if ($flashId -ne '0x0B4017') { throw ('Unexpected Flash ID ' + $flashId + '; this script targets the previously verified board. No Flash write.') }
$resultPath = Join-Path $report 'result.json'
$result = [ordered]@{
    variant='V8d'; bitstream_sha256=$hash; archive='bitstreams/camera_uart_shape_diag_v8d_20261028.zip';
    fpga_id='0x0000081B'; flash_id=$flashId; operation_index=9; operation='exFlash Erase,Program,Verify';
    address='0x000000'; frequency='0.5MHz'; flash_written_verified=$false;
    cold_boot_verified=$null; software_reload_attempted=$false; pwm_starts_disabled=$true
}
$result | ConvertTo-Json | Set-Content -LiteralPath $resultPath -Encoding utf8
$burn = Invoke-LoggedProgrammer 'erase_program_verify' ($common + @('--operation_index','9','--spiaddr','0x000000','--fsFile',$fs)) 180
if ($burn -notmatch 'Finished\.' -or $burn -notmatch 'exFlash Erase,Program,Verify' -or $burn -notmatch 'SPI program and verify success!') { throw 'Flash completion not confirmed; inspect log.' }
$result.flash_written_verified = $true
$result | ConvertTo-Json | Set-Content -LiteralPath $resultPath -Encoding utf8
Write-Output 'V8d Flash write and readback verification passed. No SRAM reload performed.'
Write-Output 'Keep servo 5V off. Fully power off the board including USB and HDMI, then reconnect without downloading to verify cold boot.'
