param(
    [Parameter(Mandatory = $true)]
    [string]$BitstreamFile,
    [string]$GowinHome = 'D:\gaoyun\Gowin\Gowin_V1.9.11.03_Education_x64',
    [string]$ExpectedSHA256 = '',
    [int]$CableIndex = 4,
    [int]$Channel = 0,
    [ValidateSet('2.5MHz','2MHz','15MHz','10MHz','1.5MHz','1.1MHz','0.9MHz','0.75MHz','0.5MHz','0.3MHz','0.4MHz','0.1MHz','0.02MHz')]
    [string]$Frequency = '2.5MHz'
)
$ErrorActionPreference = 'Stop'
$programmer = Join-Path $GowinHome 'Programmer\bin\programmer_cli.exe'
if (-not (Test-Path -LiteralPath $programmer)) { throw 'Set -GowinHome to your installed Gowin directory.' }
$bitstreamPath = (Resolve-Path -LiteralPath $BitstreamFile).Path
if ([IO.Path]::GetExtension($bitstreamPath) -ne '.fs') { throw 'Specify a Gowin .fs bitstream.' }
if ($bitstreamPath -match '[^\x00-\x7F]') { throw 'Use an ASCII bitstream path.' }
$actualHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $bitstreamPath).Hash
if ($ExpectedSHA256 -and $actualHash -ne $ExpectedSHA256) { throw 'Bitstream SHA256 mismatch; download cancelled.' }
$projectRoot = Split-Path $PSScriptRoot -Parent
$logs = Join-Path $projectRoot 'reports\runtime'
New-Item -ItemType Directory -Path $logs -Force | Out-Null
$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$scan = & $programmer --cable-index $CableIndex --channel $Channel --frequency $Frequency --scan 2>&1
$scanExit = $LASTEXITCODE
$scan | Tee-Object -FilePath (Join-Path $logs ('built_scan_' + $stamp + '.log'))
if ($scanExit -ne 0 -or ($scan -join "`n") -notmatch '0x0000081B' -or
    ($scan -join "`n") -notmatch '\b1 device\(s\) found!') {
    throw 'Expected single FPGA not found; download cancelled.'
}
$logFile = Join-Path $logs ('built_download_' + $stamp + '.log')
@(('Bitstream: ' + $bitstreamPath), ('SHA256: ' + $actualHash),
    ('CableIndex: ' + $CableIndex + '; Channel: ' + $Channel + '; Frequency: ' + $Frequency)) | Set-Content -LiteralPath $logFile -Encoding utf8
Write-Output ('Downloading to SRAM: ' + $bitstreamPath)
Write-Output ('SHA256: ' + $actualHash)
$download = & $programmer --device GW2A-18C --cable-index $CableIndex --channel $Channel --frequency $Frequency --operation_index 2 --fsFile $bitstreamPath 2>&1 |
    Tee-Object -FilePath $logFile -Append | ForEach-Object { Write-Host $_; $_ }
$downloadExit = $LASTEXITCODE
$downloadText = $download -join "`n"
if ($downloadExit -ne 0 -or $downloadText -notmatch 'Finished\.' -or
    $downloadText -match '(?i)mismatch|\bError\b|\bfailed\b' -or
    $downloadText -match '(?i)(User Code|Status Code) is:\s*0xFFFFFFFF') {
    throw 'SRAM programming or status verification failed; inspect download log and actual HDMI picture.'
}
Write-Output 'SRAM programming completed. Confirm the actual HDMI picture.'
