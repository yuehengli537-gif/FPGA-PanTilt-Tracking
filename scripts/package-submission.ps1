param(
    [string]$PythonExecutable = 'python',
    [string]$RarExecutable = 'C:\Program Files\WinRAR\Rar.exe',
    [string]$OutputDirectory = ''
)
$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
if (-not $OutputDirectory) { $OutputDirectory = Join-Path (Split-Path $root -Parent) 'submission-artifacts' }
& $PythonExecutable (Join-Path $PSScriptRoot 'verify-submission.py') --write-manifest
if ($LASTEXITCODE -ne 0) { throw 'Source verification failed.' }
& $PythonExecutable (Join-Path $PSScriptRoot 'package_submission.py') --rar $RarExecutable --output-directory $OutputDirectory
if ($LASTEXITCODE -ne 0) { throw 'RAR creation or verification failed.' }
