# Run the OmniQuad WALK deploy world headless and report forward progress.
#
# THE G1 WALK RECIPE PORTED TO OMNIQUAD (gait-residual RL on the
# stiffness-matched model). The trainer MJCF matches the deploy joint
# drive EXACTLY:
#   C:\tmp\omniquad_newton_fixed2.xml (kp=500/kv=60) <-> OMNISIM_NEWTON_TARGET_KE=500/KD=60
#
# Gait reference: foot-space trot model (projects/policies/control/gait/omniquad_trot_gait.py)
# -- stance feet slide at exactly -vx, quintic swing, duty 0.6 four-foot
# overlap, IK-realized, stride ramps in from the model's own standing pose.
# Bare-model baseline in the trainer: NEVER falls in 40 s (512 envs).
#
# Engine: OMNISIM_NEWTON_MJWARP=1 matches the trainer engine exactly.
#
# Usage:  powershell -File scripts/dev/run_omniquad_walk_deploy.ps1 [-Duration 240] [-Policy <onnx>] [-Bare]
param(
    [int]$Duration = 240,
    [string]$Policy = "",
    [switch]$Bare,
    [switch]$CpuEngine
)
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
$world = "$root\projects\policies\research\worlds\omniquad_walk_deploy.omniworld"
if ($Policy -eq "") {
    $Policy = "$root\projects\policies\research\inference\policies\gpu_omniquad_walk_main\policy.onnx"
}
$log = "$root\_scratch\omniquad_walk_deploy.log"
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
# Joint drive MUST match the trainer MJCF (kp=500/kv=60).
$env:OMNISIM_NEWTON_TARGET_KE    = "500"
$env:OMNISIM_NEWTON_TARGET_KD    = "60"
if ($Bare) { Remove-Item Env:OMNIQUAD_POLICY_ONNX -ErrorAction SilentlyContinue; $env:OMNIQUAD_POLICY_ONNX = "$root\__no_policy__" }
else       { $env:OMNIQUAD_POLICY_ONNX = $Policy }
# Gait config -- MUST match the policy's training run.
$env:OMNIQUAD_GAIT_VX     = "0.4"
$env:OMNIQUAD_GAIT_FREQ   = "1.4"
$env:OMNIQUAD_GAIT_DUTY   = "0.6"
$env:OMNIQUAD_GAIT_STEP_H = "0.06"
$env:OMNIQUAD_GAIT_BODY_H = "0.55"
$env:OMNIQUAD_GAIT_RAMP_S = "1.0"
$env:OMNIQUAD_ACT_SCALE   = "0.15"
$env:OMNIQUAD_DEPLOY_LOG  = $log
$env:OMNIQUAD_DEPLOY_TRACE = "1"

if ($Bare) { Write-Host "Running omniquad_walk_deploy.wbt for ${Duration}s wall (BARE gait model)..." }
else       { Write-Host "Running omniquad_walk_deploy.wbt for ${Duration}s wall (trot model + RL policy)..." }
python -u "$root\scripts\dev\headless_runner.py" $world --duration $Duration 2>&1 | Select-Object -Last 4

Write-Host "`n===== OMNIQUAD WALK RESULT ====="
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
