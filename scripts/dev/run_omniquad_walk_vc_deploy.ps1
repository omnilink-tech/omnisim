# VELOCITY-CONDITIONED OmniQuad walk deploy + the time-aware WALK/STOP/WALK
# milestone (the G1 walk29_vc recipe, ported to OmniQuad).
#
# One policy (gpu_omniquad_vc) trained with a commandable forward speed
# (--vx-cmd-max 0.45) INCLUDING 0 = stand. The deploy appends the normalised
# command to the obs (OMNIQUAD_VX_CMD_MAX) and drives it from the schedule's stride
# scale _w: full trot at _w=1, stand on four feet at _w=0. Default schedule is
# the user's "walk a bit, stop a bit, repeat" -- walk OMNIQUAD_WALK_FOR_S, stand
# OMNIQUAD_STAND_FOR_S, repeating; the SAME policy decelerates + stands, then
# resumes. For a quadruped the stand is statically stable.
#
# Usage:
#   run_omniquad_walk_vc_deploy.ps1 [-Chunk 0] [-Duration 40] [-WalkFor 5] [-StandFor 5] [-Gui]
#   -WalkFor 0   ->  no stop, pure continuous walk (deploy sanity at vx=0.4)
#   -Gui         ->  open a live OmniSim window (paced 1.0x) to WATCH it
param(
    [int]$Chunk = 0,            # 0 = latest gpu_omniquad_vc_c* found
    [int]$Duration = 40,
    [double]$WalkFor = 5.0,
    [double]$StandFor = 5.0,
    [double]$BlendS = 1.0,
    [switch]$CpuEngine,
    [switch]$Gui
)
$root = (Resolve-Path "$PSScriptRoot\..\..").Path

# Default = the shipped velocity-conditioned walker; -Chunk N picks a raw
# gpu_omniquad_vc3_c<N> training chunk instead.
if ($Chunk -gt 0) {
    $policy = "$root\projects\policies\research\training\runs\gpu_omniquad_vc3_c$Chunk\policy.onnx"
    Write-Host "OmniQuad VC deploy using chunk: gpu_omniquad_vc3_c$Chunk"
} else {
    $policy = "$root\projects\policies\research\inference\policies\gpu_omniquad_walk_vc_main\policy.onnx"
    Write-Host "OmniQuad VC deploy using policy: gpu_omniquad_walk_vc_main"
}
if (-not (Test-Path $policy)) { Write-Host "missing $policy"; exit 1 }

$world = "$root\projects\policies\research\worlds\omniquad_walk_deploy.omniworld"
$log   = "$root\_scratch\omniquad_walk_vc_deploy.log"

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
$env:OMNISIM_NEWTON_TARGET_KE    = "500"
$env:OMNISIM_NEWTON_TARGET_KD    = "60"
$env:OMNIQUAD_POLICY_ONNX = $policy
# Gait config -- MUST match the policy's training run.
$env:OMNIQUAD_GAIT_VX="0.4"; $env:OMNIQUAD_GAIT_FREQ="1.4"; $env:OMNIQUAD_GAIT_DUTY="0.6"
$env:OMNIQUAD_GAIT_STEP_H="0.06"; $env:OMNIQUAD_GAIT_BODY_H="0.55"; $env:OMNIQUAD_GAIT_RAMP_S="1.0"
$env:OMNIQUAD_ACT_SCALE="0.15"; $env:OMNIQUAD_SETTLE_S="1.5"
# Velocity conditioning + the time-aware walk<->stop schedule.
$env:OMNIQUAD_VX_CMD_MAX  = "0.45"        # MUST match trainer --vx-cmd-max
$env:OMNIQUAD_WALK_FOR_S  = "$WalkFor"    # 0 = continuous walk (no stop)
$env:OMNIQUAD_STAND_FOR_S = "$StandFor"
$env:OMNIQUAD_MODE_BLEND_S = "$BlendS"
$env:OMNIQUAD_DEPLOY_LOG  = $log
$env:OMNIQUAD_DEPLOY_TRACE = "1"

if ($WalkFor -gt 0) {
    Write-Host "Running ${Duration}s: walk ${WalkFor}s / STAND ${StandFor}s, repeating (velocity-conditioned)..."
} else {
    Write-Host "Running ${Duration}s continuous walk (vx=0.4, no stop)..."
}
if ($Gui) {
    python -u "$root\scripts\dev\headless_runner.py" $world --duration $Duration --gui --realtime 2>&1 | Select-Object -Last 4
} else {
    python -u "$root\scripts\dev\headless_runner.py" $world --duration $Duration 2>&1 | Select-Object -Last 4
}

Write-Host "`n===== OMNIQUAD VC WALK RESULT ====="
if (Test-Path $log) {
    $fall = (Select-String -Path $log -Pattern "FALL" | Measure-Object).Count
    if ($fall -eq 0) { Write-Host "  NO FALL." } else { Write-Host "  $fall FALL line(s)." }
    Get-Content $log | Where-Object { $_ -match "^\[t=" } | ForEach-Object { Write-Host "  $_" }
} else {
    Write-Host "  (no deploy log produced)"
}
