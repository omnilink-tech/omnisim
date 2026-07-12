# Deploy the G1 push-recovery stand+wave RL policy UNDER CUBE-RAIN.
# The policy (trained by gpu_mjwarp_g1_stand_trainer with --wave-ref + strong
# --dr-push) shadows the stand+wave ghost and is meant to ACTIVELY hold the
# stance through cube impacts -- vs the deterministic baseline (topples ~2.7s).
#
# Gains MATCH training (g1_full_kp100 => KE=100/KD=5); using the proven stand's
# stiff KE=400 would be a sim2deploy mismatch (see project_g1_walk: ALWAYS match
# deploy gains to the trainer MJCF). NOTE: a stationary RL stand is the project's
# known sim-to-deploy frontier (shadowing.md) -- this run tests whether push-DR
# training crosses it.
#
# Usage: run_g1_standwave_push_deploy.ps1 [-Duration 30] [-Gui] [-NoRain]
param(
    [int]$Duration = 30,
    [switch]$Gui,
    [switch]$NoRain,
    [switch]$SideThrow,         # cubes thrown from the sides one-at-a-time (cube_thrower supervisor)
    [switch]$Fallback,          # pure-pose deterministic baseline (no policy) for a fair A/B
    [double]$ThrowSpeed = 4.0,
    [double]$ThrowPeriod = 3.0,
    [string]$Policy = "",
    [double]$Ke = 400,
    [double]$Kd = 60,
    [double]$AnkBias = -0.04,   # centres the deploy baseline (self-stands at -0.04)
    [double]$ActScale = 0.3,    # residual authority -- MUST match the trainer's G1_RES_SCALE (0.3 = the DEPLOYING small residual)
    [switch]$Wave               # add the gentle wave (off by default for the push test)
)
$root  = (Resolve-Path "$PSScriptRoot\..\..").Path
$world = if ($SideThrow) { "$root\projects\policies\worlds\g1_standwave_push_throw.wbt" }
         elseif ($NoRain) { "$root\projects\policies\worlds\g1_stand_arms_deploy.wbt" }
         else            { "$root\projects\policies\worlds\g1_standwave_push_rain.wbt" }
# Default to the SMALL residual that actually deploys (the 0.6 stepping residual
# self-topples in deploy); pass -Policy + -ActScale to override.
$onnx  = if ($Policy) { $Policy } else { "$root\projects\policies\research\training\runs\gpu_g1_standpush_nowave_h100\policy.onnx" }
$csv   = "$root\_scratch\g1_standwave_replay.csv"
$log   = "$root\_scratch\stand\g1_standwave_push_deploy.log"
New-Item -ItemType Directory -Force -Path (Split-Path $log) | Out-Null
Remove-Item $log -ErrorAction SilentlyContinue

if (-not (Test-Path $onnx)) { Write-Error "no policy ONNX at $onnx (train first)"; exit 1 }

$env:OMNISIM_HOME      = $root
$env:PYTHONUTF8        = "1"
# deploy-faithful Newton solver config (same as the proven stand path)
$env:OMNISIM_NEWTON_STATICS      = "1"
$env:OMNISIM_NEWTON_SUBSTEPS     = "4"
$env:OMNISIM_NEWTON_FORCE_MUJOCO = "1"
$env:OMNISIM_NEWTON_MJWARP       = "1"
$env:OMNISIM_URDF_USE_INERTIA    = "1"
$env:OMNISIM_NEWTON_BASE_GUARD   = "1"
$env:OMNISIM_NEWTON_SEED_POSE    = "1"
$env:OMNISIM_NEWTON_SEED_REBUILD = "1"
# MANDATORY CoM parity: the trainer balances with link-COM; without this the deploy
# body_ipos differs by up to 0.154 m -> a balancing humanoid tips in <0.5s. The
# trainer comment is explicit: "the deploy MUST set that for nominal parity".
$env:OMNISIM_NEWTON_USE_LINK_COM = "1"
# gains MATCH the trainer MJCF (g1_full_kp100)
$env:OMNISIM_NEWTON_TARGET_KE    = "$Ke"
$env:OMNISIM_NEWTON_TARGET_KD    = "$Kd"
$env:OMNISIM_NEWTON_GROUND_MU    = "2.0"
# RL RESIDUAL on top of the self-stable deterministic baseline (KE=400 + bias).
$env:G1_POLICY_ONNX      = $onnx
$env:G1_BALANCE_FALLBACK = if ($Fallback) { "1" } else { "0" }
# Centre the deploy baseline so it self-stands; the residual rides on top.
$env:G1_ANK_PITCH_BIAS   = "$AnkBias"
$env:G1_ARM_POSE         = "down"      # arms down (low, symmetric CoM) — no aggressive forward raise
# ankle PD OFF (policy owns balance corrections; analytic roll-PD destabilises deploy).
$env:G1_ARMS_ANKLE_KP    = "0"
$env:G1_ARMS_ANKLE_KD    = "0"
$env:G1_ARMS_ANKLE_KP_R  = "0"
$env:G1_ARMS_ANKLE_KD_R  = "0"
$env:G1_ACT_SCALE        = "$ActScale"   # residual authority -- MUST match trainer G1_RES_SCALE
# side-throw supervisor config
$env:THROW_SPEED         = "$ThrowSpeed"
$env:THROW_PERIOD        = "$ThrowPeriod"
$env:THROW_Z             = "0.12"
$env:THROW_START         = "2.5"
# Gentle wave only if asked (the aggressive standwave ref shoves CoM forward and
# was the deploy saboteur; default off for the push-recovery test).
if ($Wave) { $env:G1_STANDWAVE_REF = $csv } else { Remove-Item Env:\G1_STANDWAVE_REF -ErrorAction SilentlyContinue }
$env:G1_DEPLOY_LOG       = $log
$env:OMNISIM_LOG_PATH    = "$root\_scratch\stand\omnisim_log_g1_push.txt"

Write-Host "=== G1 push-recovery stand+wave deploy (policy=$([System.IO.Path]::GetFileName($onnx)) ke=$Ke rain=$(-not $NoRain)) ===" -ForegroundColor Cyan
if ($Gui) {
    python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration
} else {
    python -u "$root\scripts\dev\headless_runner.py" $world --duration $Duration 2>&1 | Select-Object -Last 6
}

Write-Host "`n===== G1 PUSH-RECOVERY RESULT =====" -ForegroundColor Cyan
if (Test-Path $log) {
    $fall = (Select-String -Path $log -Pattern "FALL" | Measure-Object).Count
    $last = Get-Content $log | Select-Object -Last 1
    if ($fall -eq 0) { Write-Host "  PASS - no fall. last: $last" -ForegroundColor Green }
    else             { Write-Host "  $fall FALL line(s). last: $last" -ForegroundColor Yellow }
} else { Write-Host "  (no deploy log produced)" }
