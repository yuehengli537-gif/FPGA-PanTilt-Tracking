param([switch]$Demo, [string]$Port)
$ErrorActionPreference = 'Stop'
if ($Demo -and $Port) { throw 'Choose either Demo or a real serial Port.' }
if ($Port -and $Port -notmatch '^COM[0-9]+$') { throw 'Port must be a COM port name, such as COM12.' }
$projectRoot = Split-Path $PSScriptRoot -Parent
$pythonWindowless = Join-Path $projectRoot '.tools\servo-ui-venv\Scripts\pythonw.exe'
if (-not (Test-Path -LiteralPath $pythonWindowless)) {
    & (Join-Path $PSScriptRoot 'setup-servo-ui.ps1')
}
$logDirectory = Join-Path $projectRoot 'reports\runtime\desktop_control'
New-Item -ItemType Directory -Force -Path $logDirectory | Out-Null
$launchArguments = @('-m','desktop_control.app')
if ($Demo) { $launchArguments += '--demo' }
if ($Port) { $launchArguments += @('--port', $Port) }
$process = Start-Process -FilePath $pythonWindowless -ArgumentList $launchArguments -WorkingDirectory $projectRoot -WindowStyle Normal -RedirectStandardOutput (Join-Path $logDirectory 'ui.stdout.log') -RedirectStandardError (Join-Path $logDirectory 'ui.stderr.log') -PassThru
if ($process.WaitForExit(1500) -and $process.ExitCode -ne 0) {
    Get-Content -LiteralPath (Join-Path $logDirectory 'ui.stderr.log')
    throw 'UI exited during startup.'
}
