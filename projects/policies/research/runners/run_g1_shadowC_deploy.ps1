# DEPLOY-TEST the improved-shadow (C) policy gpu_g1_walk30_shadowC in OmniSim
# Newton. Identical to run_g1_walk_deploy.ps1 -Arms (the recipe walk30 warm-
# started from) PLUS the improved-shadow env: G1_GAIT_LATERAL/YAW=human so the
# deploy reference matches training. The question: does the trainer-side splay
# reduction (28.8->21.7 deg) survive the mujoco_warp->Newton gap, or does it
# collapse like the prior lateral gaits (walk27_lat / walk28_com)?
#
# Usage: run_g1_shadowC_deploy.ps1 [-Duration 120] [-Chunk 4] [-Gui] [-CpuEngine]
param(
    [int]$Duration = 120,
    [int]$Chunk = 4,
    [string]$Run = "gpu_g1_walk30_shadowC",
    [double]$ResScale = 1.0,        # G1_FRONTAL_RES_SCALE -- MUST match training
    [switch]$Gui,
    [switch]$CpuEngine
)
$root   = (Resolve-Path "$PSScriptRoot\..\..\..\..").Path
$world  = "$root\projects\policies\research\worlds\g1_walk_arms_deploy.omniworld"
$policy = "$root\projects\policies\research\training\runs\${Run}_c$Chunk\policy.onnx"
if (-not (Test-Path $policy)) { Write-Host "missing $policy"; exit 1 }
$log = "$root\_scratch\g1_shadowC_deploy.log"
Remove-Item $log -ErrorAction SilentlyContinue

$env:OMNISIM_HOME            = $root
$env:OMNISIM_NEWTON_STATICS      = "1"
$env:OMNISIM_NEWTON_SUBSTEPS     = "4"
$env:OMNISIM_NEWTON_FORCE_MUJOCO = "1"
$env:OMNISIM_REQUIRE_NEWTON  = "1"   # assert Newton engaged -- fail loud, no silent ODE fallback
$env:OMNISIM_REQUIRE_MUJOCO_SOLVER = "1"   # assert Newton MuJoCo solver (not XPBD) -- the trained engine
if ($CpuEngine) { Remove-Item Env:OMNISIM_NEWTON_MJWARP -ErrorAction SilentlyContinue }
else            { $env:OMNISIM_NEWTON_MJWARP = "1" }
$env:OMNISIM_URDF_USE_INERTIA    = "1"
$env:OMNISIM_NEWTON_BASE_GUARD   = "1"
$env:OMNISIM_NEWTON_SEED_POSE    = "1"
$env:OMNISIM_NEWTON_SEED_REBUILD = "1"
$env:OMNISIM_NEWTON_TARGET_KE    = "100"
$env:OMNISIM_NEWTON_TARGET_KD    = "5"
$env:OMNISIM_NEWTON_GROUND_MU    = "2.0"

$env:G1_BALANCE_FALLBACK = "0"
$env:G1_POLICY_ONNX      = $policy
# Shape-c8 recipe (walk30 warm-started from it) -- deploy MUST match training.
$env:G1_GAIT_MODEL    = "human"
$env:G1_GAIT_STYLE    = "winter"
$env:G1_GAIT_VX       = "0.4"
$env:G1_GAIT_RAMP_S   = "2.0"
$env:G1_GAIT_A_LAT    = "0.05"
$env:G1_GAIT_A_ANKLE  = "0.35"
$env:G1_GAIT_A_ARM    = "0.25"
$env:G1_GAIT_HIP_SCALE = "0.9"
$env:G1_OBS_STACK     = "4"
$env:G1_OBS_LOOKAHEAD = "0.1,0.4"
Remove-Item Env:G1_START_PHASE -ErrorAction SilentlyContinue
# THE upgrade: improved-shadow frontal/transverse plane (match training).
$env:G1_GAIT_LATERAL  = "human"
$env:G1_GAIT_YAW      = "human"
$env:G1_FRONTAL_RES_SCALE = "$ResScale"   # cap policy hip-roll/yaw residual (match training)
# Deploy balance PD baseline (part of the trained baseline) + roll crutch.
$env:G1_BAL_KP_P = "-1.5"; $env:G1_BAL_KD_P = "-0.2"
$env:G1_BAL_KP_R = "-3.0"; $env:G1_BAL_KD_R = "-0.5"
$env:G1_DEPLOY_LOG = $log
# Joint-angle trace for the deploy splay measurement (analyze offline).
$env:G1_TRACE_REF  = "$root\_scratch\g1_shadowC_trace.csv"

$guiArgs = @(); if ($Gui) { $guiArgs = @("--gui", "--realtime") }
Write-Host "Deploy-testing walk30_shadowC_c$Chunk (shadow C: lateral+yaw human) for ${Duration}s wall..."
python -u "$root\scripts\dev\headless_runner.py" $world @guiArgs --duration $Duration 2>&1 | Select-Object -Last 4

Write-Host "`n===== walk30_shadowC DEPLOY RESULT ====="
if (Test-Path $log) {
    $fall = (Select-String -Path $log -Pattern "FALL" | Measure-Object).Count
    $last = Get-Content $log | Select-Object -Last 1
    if ($fall -eq 0) { Write-Host "  NO FALL. last: $last" }
    else             { Write-Host "  $fall FALL line(s). last: $last" }
} else { Write-Host "  (no deploy log produced)" }
