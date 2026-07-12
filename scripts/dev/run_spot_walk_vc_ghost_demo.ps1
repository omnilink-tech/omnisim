# WATCH Spot walk/STOP/walk beside its gait-model GHOST -- the time-aware
# "stop-in-the-middle" milestone ported from the G1 (run_g1_walk_vc_ghost_demo).
#
# Two Spots in the ghost world: the real RL robot (trot model + residual,
# Newton/mjwarp) and a translucent pale-blue HOLOGRAM playing the pure trot
# model. BOTH follow the same time-aware schedule -- walk SPOT_WALK_FOR_S
# seconds, then STAND SPOT_STAND_FOR_S seconds, repeating. The trot model is
# blended toward the standing pose as the command -> 0, so the robot decelerates
# and stands on four feet (statically stable), then accelerates and resumes.
# The ghost stops and stands in lock-step, so the stop is visible against the
# ideal target.
#
# By default it runs the EXISTING straight-walk policy (gpu_spot_walk_main, 48-d
# obs): stopping a quadruped is just the statically-stable standing pose, so no
# velocity-conditioned retrain is needed for the stop. Pass -VcCmdMax 0.45 +
# -Policy <vc.onnx> to run a velocity-conditioned policy (49-d obs).
#
# Real-time paced. Close the window to stop.
# Usage:
#   run_spot_walk_vc_ghost_demo.ps1 [-Duration 60] [-WalkFor 5] [-StandFor 5]
#                                   [-Policy <onnx>] [-VcCmdMax 0] [-Fast]
param(
    [int]$Duration = 60,
    [double]$WalkFor = 5.0,
    [double]$StandFor = 5.0,
    [double]$BlendS = 1.0,
    [string]$Policy = "",
    [double]$VcCmdMax = 0.45,
    [switch]$Fast
)
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
$world = "$root\projects\policies\research\worlds\spot_walk_ghost_demo.wbt"
if ($Policy -eq "") {
    # The velocity-conditioned walker (gpu_spot_vc3_c1): walks/stops/resumes on
    # command, zero falls, dead straight. Pass -VcCmdMax 0 -Policy <walker.onnx>
    # to instead run a plain non-VC walker (continuous walk, no clean stop).
    $Policy = "$root\projects\policies\research\inference\policies\gpu_spot_walk_vc_main\policy.onnx"
}
if (-not (Test-Path $Policy)) { Write-Host "missing policy $Policy"; exit 1 }

$env:OMNISIM_HOME                = $root
$env:OMNISIM_NEWTON_FORCE_MUJOCO = "1"
$env:OMNISIM_NEWTON_MJWARP       = "1"
$env:OMNISIM_URDF_USE_INERTIA    = "1"
$env:OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE = "1"
$env:OMNISIM_NEWTON_SEED_POSE    = "1"
$env:OMNISIM_NEWTON_GROUND_MU    = "2.0"
$env:OMNISIM_NEWTON_SUBSTEPS     = "8"
$env:OMNISIM_NEWTON_TARGET_KE    = "500"
$env:OMNISIM_NEWTON_TARGET_KD    = "60"
$env:SPOT_POLICY_ONNX = $Policy
# Gait config -- shared by the real controller AND the ghost (MUST match the
# policy's training run).
$env:SPOT_GAIT_VX     = "0.4"
$env:SPOT_GAIT_FREQ   = "1.4"
$env:SPOT_GAIT_DUTY   = "0.6"
$env:SPOT_GAIT_STEP_H = "0.06"
$env:SPOT_GAIT_BODY_H = "0.55"
$env:SPOT_GAIT_RAMP_S = "1.0"
$env:SPOT_ACT_SCALE   = "0.15"
$env:SPOT_SETTLE_S    = "1.5"
# Time-aware WALK<->STOP schedule (the new bit) -- shared by both robots.
$env:SPOT_WALK_FOR_S  = "$WalkFor"
$env:SPOT_STAND_FOR_S = "$StandFor"
$env:SPOT_MODE_BLEND_S = "$BlendS"
$env:SPOT_VX_CMD_MAX  = "$VcCmdMax"     # 0 = existing 48-d policy; >0 = VC policy
$env:SPOT_DEPLOY_LOG  = "$root\_scratch\spot_walk_vc_ghost_demo.log"
# Hologram look.
$env:SPOT_GHOST_Y     = "1.4"
$env:SPOT_GHOST_ALPHA = "0.6"
$env:SPOT_GHOST_TINT  = "0.62,0.82,1.0"

Write-Host "Opening spot_walk_ghost_demo.wbt: real Spot + hologram ghost, walk ${WalkFor}s / STOP ${StandFor}s repeating."
Write-Host "Watch BOTH walk, then stand together, then resume. Close the window to stop."
if ($Fast) { python -u "$root\scripts\dev\headless_runner.py" $world --gui --duration $Duration }
else       { python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration }
