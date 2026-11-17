# Run the adaptive ballistic-intercept demo or its fixed-cup control.
param(
    [ValidateSet("adaptive", "fixed")]
    [string]$Mode = "adaptive",
    [int]$Duration = 90,
    [int]$Port = 1277,
    [switch]$Headless
)

$ErrorActionPreference = "Stop"
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
$world = "$root\projects\samples\demos\worlds\showcase\omniarm6_adaptive_intercept.omniworld"
$outDir = "$root\.local-runs\adaptive-intercept"
New-Item -ItemType Directory -Force -Path $outDir | Out-Null

$env:OMNISIM_HOME = $root
$env:OMNISIM_NEWTON_STATICS = "1"
$env:OMNISIM_NEWTON_SUBSTEPS = "4"
Remove-Item Env:OMNISIM_NEWTON_MJWARP -ErrorAction SilentlyContinue
$env:OMNISIM_NEWTON_COMPOUND_COLLIDERS = "1"
$env:OMNISIM_NEWTON_TARGET_KE = "8000"
$env:OMNISIM_NEWTON_TARGET_KD = "30"
$env:OMNIARM6_TC_THROW_X = "1.78"
$env:OMNISIM_INTERCEPT_MODE = $Mode
$env:OMNISIM_INTERCEPT_RESULT = "$outDir\adaptive_intercept_$Mode.json"
$env:OMNISIM_INTERCEPT_OBSERVER_OUT = "$outDir\adaptive_intercept_deflection_$Mode.json"
$env:OMNIARM6_TC_THROW_OUT = "$outDir\adaptive_intercept_throw_$Mode.log"
$env:OMNISIM_LOG_PATH = "$outDir\adaptive_intercept_engine_$Mode.log"

$runner = "$root\scripts\dev\headless_runner.py"
$completionLog = [System.IO.Path]::ChangeExtension($env:OMNISIM_INTERCEPT_RESULT, ".log")
Remove-Item -LiteralPath $env:OMNISIM_INTERCEPT_RESULT -ErrorAction SilentlyContinue
Remove-Item -LiteralPath $completionLog -ErrorAction SilentlyContinue
Remove-Item -LiteralPath $env:OMNISIM_INTERCEPT_OBSERVER_OUT -ErrorAction SilentlyContinue
Remove-Item -LiteralPath $env:OMNIARM6_TC_THROW_OUT -ErrorAction SilentlyContinue
$args = @(
    "-u", $runner, $world,
    "--duration", $Duration,
    "--wait-for-step",
    "--port", $Port,
    "--completion-log", $completionLog,
    "--completion-pattern", "RESULT",
    "--completion-grace", "2"
)
if (-not $Headless) {
    $args += @("--gui", "--realtime")
}
python @args
