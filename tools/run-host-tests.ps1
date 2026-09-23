$ErrorActionPreference = 'Stop'
& python (Join-Path $PSScriptRoot 'run-host-tests.py') @args
exit $LASTEXITCODE
