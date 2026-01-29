$ErrorActionPreference = "Stop"

$venvPy = Join-Path $PSScriptRoot ".venv/Scripts/python.exe"
if (-not (Test-Path $venvPy)) {
    Write-Error "Venv not found. Run host/setup_venv.ps1 first."
    exit 1
}

& $venvPy - << 'PY'
import sys
from serial.tools import list_ports
ports = list(list_ports.comports())
if not ports:
    print('No serial ports found.')
else:
    for p in ports:
        print(f"{p.device}: {p.description}")
PY
