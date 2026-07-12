# WATCH the velocity-conditioned Go2 WALK/STOP/WALK beside its ghost.
#
# The Spot/G1 walk29_vc recipe ported to Go2: the real robot (VC policy +
# trot model, Newton/mjwarp) walks, STOPS, and resumes on the schedule while
# the translucent ghost plays the SAME schedule beside it -- the ghost stands
# on four feet during the stop windows, in lock-step. The gap between them is
# the RL correction.
#
# Usage:  run_go2_walk_vc_ghost_demo.ps1 [-Duration 60] [-WalkFor 6] [-StandFor 4] [-Fast]
param(
    [int]$Duration = 60,
    [double]$WalkFor = 6.0,
    [double]$StandFor = 4.0,
    [double]$BlendS = 1.0,
    [switch]$Fast
)
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
$world = "$root\projects\policies\research\worlds\go2_walk_ghost_demo.wbt"
$policy = "$root\projects\policies\research\inference\policies\gpu_go2_walk_vc_main\policy.onnx"
if (-not (Test-Path $policy)) { Write-Host "missing $policy"; exit 1 }

$env:OMNISIM_HOME                = $root
$env:OMNISIM_NEWTON_FORCE_MUJOCO = "1"
$env:OMNISIM_NEWTON_MJWARP       = "1"
$env:OMNISIM_URDF_USE_INERTIA    = "1"
$env:OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE = "1"
$env:OMNISIM_NEWTON_SEED_POSE    = "1"
$env:OMNISIM_NEWTON_GROUND_MU    = "2.0"
$env:OMNISIM_NEWTON_SUBSTEPS     = "8"
$env:OMNISIM_NEWTON_TARGET_KE    = "250"
$env:OMNISIM_NEWTON_TARGET_KD    = "6"
$env:GO2_POLICY_ONNX = $policy
$env:GO2_GAIT_VX="0.4"; $env:GO2_GAIT_FREQ="1.8"; $env:GO2_GAIT_DUTY="0.6"
$env:GO2_GAIT_STEP_H="0.05"; $env:GO2_GAIT_BODY_H="0.30"; $env:GO2_GAIT_RAMP_S="1.0"
$env:GO2_ACT_SCALE="0.15"
# Velocity conditioning + walk<->stop schedule (real robot AND ghost share it).
$env:GO2_VX_CMD_MAX  = "0.45"
$env:GO2_WALK_FOR_S  = "$WalkFor"
$env:GO2_STAND_FOR_S = "$StandFor"
$env:GO2_MODE_BLEND_S = "$BlendS"
# Hologram look.
$env:GO2_GHOST_Y = "0.7"; $env:GO2_GHOST_ALPHA = "0.6"; $env:GO2_GHOST_TINT = "0.62,0.82,1.0"

Write-Host "Opening go2_walk_ghost_demo.wbt (VC): walk ${WalkFor}s / STAND ${StandFor}s, repeating. Close the window to stop."
if ($Fast) { python -u "$root\scripts\dev\headless_runner.py" $world --gui --duration $Duration }
else       { python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration }
