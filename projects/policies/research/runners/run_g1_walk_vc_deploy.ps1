# VELOCITY-CONDITIONED G1 walk deploy + the STOP-IN-THE-MIDDLE milestone.
#
# One policy (gpu_g1_walk29_vc) trained with a commandable forward speed
# (--vx-cmd-max 0.45) INCLUDING 0 = stand. The deploy appends the normalised
# command to the obs (G1_VX_CMD_MAX) and drives it from the stride scale _w:
# full walk at _w=1, stand at _w=0. The G1_STAND_AT_S schedule ramps _w->0 so
# the SAME policy decelerates and settles into a stand mid-walk, then resumes.
#
# This is the velocity-conditioned successor to run_g1_walk_deploy.ps1 -Arms
# (shape-c8 recipe: human/winter gait, frame-stack+lookahead obs, stiff roll
# PD). Only the policy + the vx-command wiring differ.
#
# Usage:
#   run_g1_walk_vc_deploy.ps1 [-Chunk 8] [-Duration 40] [-StandAt 12] [-StandFor 5] [-CpuEngine] [-Gui]
#   -StandAt 0  ->  no stop, pure continuous walk (deploy sanity at vx=0.4)
#   -Gui        ->  open a live OmniSim window (paced 1.0x) to WATCH the milestone
param(
    [int]$Chunk = 0,            # 0 = latest gpu_g1_walk29_vc_c* found
    [int]$Duration = 40,
    [double]$StandAt = 12.0,
    [double]$StandFor = 5.0,
    [switch]$CpuEngine,
    [switch]$Gui,               # open a live OmniSim window (paced 1.0x) to WATCH
    [double]$LaunchHold = 0.0   # 0 = launch WALKING (robust: stepping catches
)                               #   lateral falls). >0 = launch-from-stand settle.
$root = (Resolve-Path "$PSScriptRoot\..\..").Path

# Resolve the policy: explicit chunk, else the highest-numbered vc chunk.
if ($Chunk -gt 0) {
    $run = "gpu_g1_walk29_vc_c$Chunk"
} else {
    $dirs = Get-ChildItem "$root\projects\policies\research\training\runs" -Directory -Filter "gpu_g1_walk29_vc_c*" |
            Where-Object { Test-Path "$($_.FullName)\policy.onnx" } |
            Sort-Object { [int]($_.Name -replace '.*_c','') }
    if (-not $dirs) { Write-Host "no gpu_g1_walk29_vc_c* policy.onnx found"; exit 1 }
    $run = $dirs[-1].Name
}
$policy = "$root\projects\policies\research\training\runs\$run\policy.onnx"
if (-not (Test-Path $policy)) { Write-Host "missing $policy"; exit 1 }
Write-Host "VC deploy using policy: $run"

$world = "$root\projects\policies\worlds\g1_walk_arms_deploy.wbt"
$log   = "$root\_scratch\g1_walk_vc_deploy.log"
$trace = "$root\_scratch\g1_walk_vc_trace.csv"
Remove-Item $log   -ErrorAction SilentlyContinue
Remove-Item $trace -ErrorAction SilentlyContinue

# --- shape-c8 recipe (must match the warm-start base) ---
$env:G1_GAIT_MODEL    = "human"
$env:G1_GAIT_STYLE    = "winter"
$env:G1_GAIT_VX       = "0.4"
$env:G1_GAIT_RAMP_S   = "2.0"
$env:G1_OBS_STACK     = "4"
$env:G1_OBS_LOOKAHEAD = "0.1,0.4"
$env:G1_GAIT_A_LAT    = "0.05"
$env:G1_GAIT_A_ANKLE  = "0.35"
$env:G1_GAIT_A_ARM    = "0.25"
$env:G1_GAIT_HIP_SCALE = "0.9"
Remove-Item Env:G1_START_PHASE -ErrorAction SilentlyContinue

# --- velocity conditioning + the stop-in-the-middle schedule ---
$env:G1_VX_CMD_MAX  = "0.45"        # MUST match trainer --vx-cmd-max
$env:G1_STAND_AT_S  = "$StandAt"    # ramp the command to 0 (stand) at this sim time
$env:G1_STAND_FOR_S = "$StandFor"
$env:G1_MODE_BLEND_S = "1.5"        # _w (=vx command) ramp time
$env:G1_VX_LAUNCH_HOLD_S = "$LaunchHold"   # launch-from-stand settle; 0 = launch walking
$env:G1_VX_PHASE_FREEZE  = "0.0"    # MUST match training (freeze off) -- else OOD launch

# --- engine: match the trainer exactly (mujoco_warp, kp100) ---
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
$env:G1_BAL_KP_P = "-1.5"; $env:G1_BAL_KD_P = "-0.2"
$env:G1_BAL_KP_R = "-3.0"; $env:G1_BAL_KD_R = "-0.5"
$env:G1_DEPLOY_LOG = $log
$env:G1_TRACE_REF  = $trace

if ($StandAt -gt 0) {
    Write-Host "Running ${Duration}s: walk -> STAND at ${StandAt}s for ${StandFor}s -> walk (vx-conditioned)..."
} else {
    Write-Host "Running ${Duration}s continuous walk (vx=0.4, no stop)..."
}
if ($Gui) {
    # Live window, paced to real time (the bridge runs 4-5x faster than RT).
    python -u "$root\scripts\dev\headless_runner.py" $world --duration $Duration --gui --realtime 2>&1 | Select-Object -Last 4
} else {
    python -u "$root\scripts\dev\headless_runner.py" $world --duration $Duration 2>&1 | Select-Object -Last 4
}

Write-Host "`n===== G1 VC WALK RESULT ====="
if (Test-Path $log) {
    $fall = (Select-String -Path $log -Pattern "FALL" | Measure-Object).Count
    $last = Get-Content $log | Select-Object -Last 1
    if ($fall -eq 0) { Write-Host "  no fall. last: $last" }
    else             { Write-Host "  $fall fall line(s). last: $last" }
} else {
    Write-Host "  (no deploy log produced)"
}
