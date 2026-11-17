# Run the Unitree Go2 WALK deploy world headless and report forward progress.
#
# THE OMNIQUAD/G1 WALK RECIPE PORTED TO GO2 (gait-residual RL on the
# stiffness-matched model). The trainer MJCF matches the deploy joint drive
# EXACTLY:
#   C:\tmp\go2_newton.xml (kp=250/kv=6) <-> OMNISIM_NEWTON_TARGET_KE=250/KD=6
#
# Gait reference: foot-space trot model (projects/policies/control/gait/go2_trot_gait.py)
# -- stance feet slide at exactly -vx, quintic swing, duty 0.6 four-foot
# overlap, IK-realized, stride ramps in from the model's own standing pose.
# Bare-model baseline (512 envs, KE=250): 100% upright, ~0.22 m/s.
#
# Engine: OMNISIM_NEWTON_MJWARP=1 matches the trainer engine exactly.
#
# Usage:  powershell -File scripts/dev/run_go2_walk_deploy.ps1 [-Duration 240] [-Policy <onnx>] [-Bare] [-Gui]
param(
    [int]$Duration = 240,
    [string]$Policy = "",
    [switch]$Bare,
    [switch]$Gui,
    [switch]$Realtime,
    [switch]$CpuEngine
)
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
$world = "$root\projects\policies\research\worlds\go2_walk_deploy.omniworld"
if ($Policy -eq "") {
    $Policy = "$root\projects\policies\research\inference\policies\gpu_go2_walk_main\policy.onnx"
}
$log = "$root\_scratch\go2_walk_deploy.log"
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
# Joint drive MUST match the trainer MJCF (kp=250/kv=6).
$env:OMNISIM_NEWTON_TARGET_KE    = "250"
$env:OMNISIM_NEWTON_TARGET_KD    = "6"
if ($Bare) { Remove-Item Env:GO2_POLICY_ONNX -ErrorAction SilentlyContinue; $env:GO2_POLICY_ONNX = "$root\__no_policy__" }
else       { $env:GO2_POLICY_ONNX = $Policy }
# Gait config -- MUST match the policy's training run.
$env:GO2_GAIT_VX     = "0.4"
$env:GO2_GAIT_FREQ   = "1.8"
$env:GO2_GAIT_DUTY   = "0.6"
$env:GO2_GAIT_STEP_H = "0.05"
$env:GO2_GAIT_BODY_H = "0.30"
$env:GO2_GAIT_RAMP_S = "1.0"
$env:GO2_ACT_SCALE   = "0.15"
$env:GO2_DEPLOY_LOG  = $log
$env:GO2_DEPLOY_TRACE = "1"

$guiArg = ""
if ($Gui) { $guiArg = "--gui" }
$rtArg = ""
if ($Realtime) { $rtArg = "--realtime" }
if ($Bare) { Write-Host "Running go2_walk_deploy.wbt for ${Duration}s wall (BARE gait model)..." }
else       { Write-Host "Running go2_walk_deploy.wbt for ${Duration}s wall (trot model + RL policy)..." }
python -u "$root\scripts\dev\headless_runner.py" $world --duration $Duration $guiArg $rtArg 2>&1 | Select-Object -Last 4

Write-Host "`n===== GO2 WALK RESULT ====="
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
