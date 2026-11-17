# Run the Unitree B2 WALK deploy world headless and report forward progress.
#
# THE OMNIQUAD/G1 WALK RECIPE PORTED TO GO2 (gait-residual RL on the
# stiffness-matched model). The trainer MJCF matches the deploy joint drive
# EXACTLY:
#   C:\tmp\b2_newton.xml (kp=1400/kv=35) <-> OMNISIM_NEWTON_TARGET_KE=1400/KD=35
#
# Gait reference: foot-space trot model (projects/policies/control/gait/b2_trot_gait.py)
# -- stance feet slide at exactly -vx, quintic swing, duty 0.6 four-foot
# overlap, IK-realized, stride ramps in from the model's own standing pose.
# Bare-model baseline (512 envs, KE=1400): 100% upright, ~0.22 m/s.
#
# Engine: OMNISIM_NEWTON_MJWARP=1 matches the trainer engine exactly.
#
# Usage:  powershell -File scripts/dev/run_b2_walk_deploy.ps1 [-Duration 240] [-Policy <onnx>] [-Bare] [-Gui]
param(
    [int]$Duration = 240,
    [string]$Policy = "",
    [switch]$Bare,
    [switch]$Gui,
    [switch]$Realtime,
    [switch]$CpuEngine
)
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
$world = "$root\projects\policies\research\worlds\b2_walk_deploy.omniworld"
if ($Policy -eq "") {
    $Policy = "$root\projects\policies\research\inference\policies\gpu_b2_walk_main\policy.onnx"
}
$log = "$root\_scratch\b2_walk_deploy.log"
Remove-Item $log -ErrorAction SilentlyContinue

$env:OMNISIM_HOME                = $root
$env:OMNISIM_NEWTON_FORCE_MUJOCO = "1"
$env:OMNISIM_REQUIRE_NEWTON  = "1"   # assert Newton engaged -- fail loud, no silent ODE fallback
$env:OMNISIM_REQUIRE_MUJOCO_SOLVER = "1"   # assert Newton MuJoCo solver (not XPBD) -- the trained engine
if ($CpuEngine) { Remove-Item Env:OMNISIM_NEWTON_MJWARP -ErrorAction SilentlyContinue }
else            { $env:OMNISIM_NEWTON_MJWARP = "1" }
$env:OMNISIM_URDF_USE_INERTIA    = "1"
$env:OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE = "1"
$env:OMNISIM_NEWTON_SEED_POSE    = "1"
$env:OMNISIM_NEWTON_GROUND_MU    = "2.0"
$env:OMNISIM_NEWTON_SUBSTEPS     = "8"
# Joint drive MUST match the trainer MJCF (kp=1400/kv=35).
$env:OMNISIM_NEWTON_TARGET_KE    = "1400"
$env:OMNISIM_NEWTON_TARGET_KD    = "35"
if ($Bare) { Remove-Item Env:B2_POLICY_ONNX -ErrorAction SilentlyContinue; $env:B2_POLICY_ONNX = "$root\__no_policy__" }
else       { $env:B2_POLICY_ONNX = $Policy }
# Gait config -- MUST match the policy's training run.
$env:B2_GAIT_VX     = "0.5"
$env:B2_GAIT_FREQ   = "1.3"
$env:B2_GAIT_DUTY   = "0.6"
$env:B2_GAIT_STEP_H = "0.08"
$env:B2_GAIT_BODY_H = "0.50"
$env:B2_GAIT_RAMP_S = "1.0"
$env:B2_ACT_SCALE   = "0.15"
$env:B2_DEPLOY_LOG  = $log
$env:B2_DEPLOY_TRACE = "1"

$guiArg = ""
if ($Gui) { $guiArg = "--gui" }
$rtArg = ""
if ($Realtime) { $rtArg = "--realtime" }
if ($Bare) { Write-Host "Running b2_walk_deploy.wbt for ${Duration}s wall (BARE gait model)..." }
else       { Write-Host "Running b2_walk_deploy.wbt for ${Duration}s wall (trot model + RL policy)..." }
python -u "$root\scripts\dev\headless_runner.py" $world --duration $Duration $guiArg $rtArg 2>&1 | Select-Object -Last 4

Write-Host "`n===== B2 WALK RESULT ====="
if (Test-Path $log) {
    $fall = (Select-String -Path $log -Pattern "FALL" | Measure-Object).Count
    $last = Get-Content $log | Where-Object { $_ -match "^\[t=" } | Select-Object -Last 1
    if ($fall -eq 0) { Write-Host "  NO FALL. last: $last" }
    else {
        $fline = (Select-String -Path $log -Pattern "FALL" | Select-Object -First 1).Line
        Write-Host "  FELL: $fline"
    }
} else {
    Write-Host "  (no deploy log produced)"
}
