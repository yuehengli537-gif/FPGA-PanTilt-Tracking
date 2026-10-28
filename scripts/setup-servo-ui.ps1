param([string]$PythonExecutable = '')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
if (-not $PythonExecutable) {
    $bundledPython = Join-Path $env:USERPROFILE '.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
    if (Test-Path -LiteralPath $bundledPython) { $PythonExecutable = $bundledPython }
    else {
        $pythonCommand = Get-Command python -ErrorAction SilentlyContinue
        if ($pythonCommand -and $pythonCommand.Source -notmatch 'WindowsApps') { $PythonExecutable = $pythonCommand.Source }
    }
}
if (-not $PythonExecutable) { throw 'Install Python 3.10+ with tkinter, then set -PythonExecutable.' }
$environmentPath = Join-Path $projectRoot '.tools\servo-ui-venv'
& $PythonExecutable -m venv $environmentPath
if ($LASTEXITCODE -ne 0) { throw 'Python environment setup failed.' }
$environmentPython = Join-Path $environmentPath 'Scripts\python.exe'
& $environmentPython -m pip install --disable-pip-version-check -r (Join-Path $projectRoot 'desktop_control\requirements.txt')
if ($LASTEXITCODE -ne 0) { throw 'Serial dependency installation failed.' }
& $environmentPython -c 'import tkinter, serial; print("Servo UI dependencies ready")'
if ($LASTEXITCODE -ne 0) { throw 'Tkinter or pySerial unavailable.' }
