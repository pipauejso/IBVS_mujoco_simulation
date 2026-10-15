param([string]$Python)
$ErrorActionPreference = "Stop"
if ($Python) {
    if (-not (Test-Path -LiteralPath $Python -PathType Leaf)) {
        throw "Python executable not found: $Python"
    }
    (Resolve-Path -LiteralPath $Python).Path
    exit 0
}

# Resolve the executable itself: old conda run can choose base Python when
# the requested environment is already active on Windows.
$envJson = & conda env list --json
if ($LASTEXITCODE -ne 0) { throw "Could not list conda environments" }
$environments = ($envJson | ConvertFrom-Json).envs
foreach ($envPath in $environments) {
    if ((Split-Path -Leaf $envPath) -eq "mujoco") {
        $candidate = Join-Path $envPath "python.exe"
        if (Test-Path -LiteralPath $candidate -PathType Leaf) {
            $candidate
            exit 0
        }
    }
}
throw "Conda environment 'mujoco' not found. Supply -Python with an explicit python.exe path."
