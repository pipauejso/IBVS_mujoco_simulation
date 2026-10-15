$ErrorActionPreference = "Stop"
$simulationArgs = @($args)
$mujocoPython = & (Join-Path $PSScriptRoot "scripts/resolve_mujoco_python.ps1")
Write-Host "Python: $mujocoPython"
& $mujocoPython -B (Join-Path $PSScriptRoot "mujoco_for_ur/main.py") @simulationArgs
exit $LASTEXITCODE
