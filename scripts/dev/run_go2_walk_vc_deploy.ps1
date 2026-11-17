# VELOCITY-CONDITIONED Go2 walk deploy + the time-aware WALK/STOP/WALK
# milestone (the OmniQuad/G1 walk29_vc recipe, ported to Go2).
#
# One policy (gpu_go2_walk_vc_main) trained with a commandable forward speed
# (--vx-cmd-max 0.45) INCLUDING 0 = stand. The deploy appends the normalised
# command to the obs (GO2_VX_CMD_MAX) and drives it from the schedule's stride
# scale _w: full trot at _w=1, stand on four feet at _w=0. Default schedule is
# "walk a bit, stop a bit, repeat" -- walk GO2_WALK_FOR_S, stand
# GO2_STAND_FOR_S, repeating; the SAME policy decelerates + stands, then
# resumes. For a quadruped the stand is statically stable.
#
# Usage:
#   run_go2_walk_vc_deploy.ps1 [-Duration 40] [-WalkFor 6] [-StandFor 4] [-Gui] [-Realtime]
#   -WalkFor 0   ->  no stop, pure continuous walk
#   -Gui -Realtime -> open a live OmniSim window paced 1.0x to WATCH it
param(
    [int]$Duration = 40,
    [double]$WalkFor = 6.0,
    [double]$StandFor = 4.0,
    [double]$BlendS = 1.0,
    [string]$Policy = "",
    [switch]$Gui,
    [switch]$Realtime,
    [switch]$CpuEngine
)
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
if ($Policy -eq "") {
    $Policy = "$root\projects\policies\research\inference\policies\gpu_go2_walk_vc_main\policy.onnx"
}
if (-not (Test-Path $Policy)) { Write-Host "missing $Policy"; exit 1 }
$world = "$root\projects\policies\research\worlds\go2_walk_deploy.omniworld"
$log   = "$root\_scratch\go2_walk_vc_deploy.log"
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
$env:OMNISIM_NEWTON_TARGET_KE    = "250"
$env:OMNISIM_NEWTON_TARGET_KD    = "6"
$env:GO2_POLICY_ONNX = $Policy
# Gait config -- MUST match the policy's training run.
$env:GO2_GAIT_VX="0.4"; $env:GO2_GAIT_FREQ="1.8"; $env:GO2_GAIT_DUTY="0.6"
$env:GO2_GAIT_STEP_H="0.05"; $env:GO2_GAIT_BODY_H="0.30"; $env:GO2_GAIT_RAMP_S="1.0"
$env:GO2_ACT_SCALE="0.15"; $env:GO2_SETTLE_S="1.5"
# Velocity conditioning + the time-aware walk<->stop schedule.
$env:GO2_VX_CMD_MAX  = "0.45"        # MUST match trainer --vx-cmd-max
$env:GO2_WALK_FOR_S  = "$WalkFor"    # 0 = continuous walk (no stop)
$env:GO2_STAND_FOR_S = "$StandFor"
$env:GO2_MODE_BLEND_S = "$BlendS"
$env:GO2_DEPLOY_LOG  = $log
$env:GO2_DEPLOY_TRACE = "1"

$flags = @($world, "--duration", $Duration)
if ($Gui)      { $flags += "--gui" }
if ($Realtime) { $flags += "--realtime" }
if ($WalkFor -gt 0) {
    Write-Host "Running ${Duration}s: walk ${WalkFor}s / STAND ${StandFor}s, repeating (velocity-conditioned)..."
} else {
    Write-Host "Running ${Duration}s continuous walk (vx=0.4, no stop)..."
}
python -u "$root\scripts\dev\headless_runner.py" @flags 2>&1 | Select-Object -Last 4

Write-Host "`n===== GO2 VC WALK/STOP RESULT ====="
if (Test-Path $log) {
    $fall = (Select-String -Path $log -Pattern "FALL" | Measure-Object).Count
    if ($fall -eq 0) { Write-Host "  NO FALL." } else {
        Write-Host "  FELL: $((Select-String -Path $log -Pattern 'FALL' | Select-Object -First 1).Line)"
    }
    # show the speed at a couple of walk and stand moments
    Get-Content $log | Where-Object { $_ -match "^\[t=" } |
        Where-Object { $_ -match "t=(2|5|8|12|15)s" } | ForEach-Object { Write-Host "  $_" }
}
