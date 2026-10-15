param([switch]$Record, [string]$Python)
$ErrorActionPreference = "Stop"
$mujocoPython = & (Join-Path $PSScriptRoot "scripts/resolve_mujoco_python.ps1") -Python $Python
Write-Host "Python: $mujocoPython"
Push-Location $PSScriptRoot
try {
    & $mujocoPython -B -m unittest discover -s tests -v
    if ($LASTEXITCODE -ne 0) { throw "Regression checks failed" }
    foreach ($scenario in @("translation", "depth", "rotation", "tilt", "combined")) {
        $runArgs = @("-B",
            "mujoco_for_ur/main.py", "--headless", "--demo", "--scenario", $scenario)
        if ($Record) { $runArgs += "--record" }
        & $mujocoPython @runArgs
        if ($LASTEXITCODE -ne 0) { throw "Reproduction failed: $scenario" }
    }
    & $mujocoPython -B scripts/summarize_results.py
    if ($LASTEXITCODE -ne 0) { throw "Result aggregation failed" }
} finally {
    Pop-Location
}
