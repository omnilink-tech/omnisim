# Closed-loop POLICY parity probe -- DEPLOY launcher.
# Runs a trained G1 stand ONNX (g1_policy_probe controller) closed-loop in
# omnisim-bin and dumps a per-tick trace, then (with -Compare) diffs it against
# the SAME policy run in the trainer eval (gpu_mjwarp_g1_stand_trainer --eval
# --dump-trace). End-to-end train==deploy validation.
#
# Gains/friction/substeps MATCH the trainer's forced parity config exactly
# (ke=400/kd=60, GROUND_MU=1.0 = the MJCF, SUBSTEPS=4), + USE_LINK_COM=1.
param(
    [string]$Onnx = "projects/policies/research/training/runs/g1_parity_stand/policy.onnx",
    [int]$Settle = 40,
    [int]$Ticks = 200,
    [int]$Duration = 160,
    [switch]$Gui,
    [switch]$RecordSettle,
    [switch]$Welded,
    [switch]$Compare
)
$ErrorActionPreference = "Stop"
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
$world = if ($Welded) { "$root\projects\policies\research\worlds\g1_policy_probe_welded.omniworld" }
         else { "$root\projects\policies\research\worlds\g1_policy_probe.omniworld" }
$outdir = "$root\_scratch\parity"
New-Item -ItemType Directory -Force -Path $outdir | Out-Null
$deployTrace = "$outdir\g1_policy_deploy.json"
$trainerTrace = "$outdir\g1_policy_trainer.json"
Remove-Item $deployTrace -ErrorAction SilentlyContinue

$env:OMNISIM_HOME = $root
$env:PYTHONUTF8   = "1"
$env:PYTHONPATH   = $root
$env:OMNISIM_NEWTON_STATICS      = "1"
$env:OMNISIM_NEWTON_SUBSTEPS     = "4"
$env:OMNISIM_NEWTON_FORCE_MUJOCO = "1"
$env:OMNISIM_NEWTON_MJWARP       = "1"
$env:OMNISIM_URDF_USE_INERTIA    = "1"
$env:OMNISIM_NEWTON_BASE_GUARD   = "1"
$env:OMNISIM_NEWTON_SEED_POSE    = "1"
$env:OMNISIM_NEWTON_TARGET_KE    = "400"
$env:OMNISIM_NEWTON_TARGET_KD    = "60"
$env:OMNISIM_NEWTON_GROUND_MU    = "1.0"   # == the trainer MJCF ground friction
$env:OMNISIM_NEWTON_USE_LINK_COM = "1"
$env:OMNISIM_REQUIRE_NEWTON      = "1"
$env:OMNISIM_LOG_PATH            = "$outdir\omnisim_log_g1_policy.txt"

$env:G1_POLICY_ONNX = if ([System.IO.Path]::IsPathRooted($Onnx)) { $Onnx } else { "$root\$Onnx" }
$env:PROBE_TRACE    = $deployTrace
$env:POLICY_SETTLE  = "$Settle"
$env:POLICY_TICKS   = "$Ticks"
if ($RecordSettle) { $env:PROBE_RECORD_SETTLE = "1" } else { Remove-Item Env:\PROBE_RECORD_SETTLE -ErrorAction SilentlyContinue }

Write-Host "=== G1 closed-loop policy probe (deploy) onnx=$(Split-Path $env:G1_POLICY_ONNX -Leaf) ke=400 kd=60 ===" -ForegroundColor Cyan
if ($Gui) {
    python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration
} else {
    python -u "$root\scripts\dev\headless_runner.py" $world --duration $Duration 2>&1 | Select-Object -Last 12
}
if (-not (Test-Path $deployTrace)) { Write-Host "  NO deploy trace -- check $($env:OMNISIM_LOG_PATH)" -ForegroundColor Yellow; exit 2 }
Write-Host "  deploy trace: $deployTrace" -ForegroundColor Green

if ($Compare) {
    Write-Host "`n=== closed-loop parity diff (trainer-eval vs deploy) ===" -ForegroundColor Cyan
    python -u "$root\projects\policies\research\training\g1_parity_compare.py" $trainerTrace $deployTrace --policy
}
