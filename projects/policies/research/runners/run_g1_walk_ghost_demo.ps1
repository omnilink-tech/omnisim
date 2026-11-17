# WATCH the gait-model GHOST walk beside the real robot, live in OmniSim.
#
# Two G1s: the real RL-stabilised robot (physics, Newton) and a kinematic,
# physics-free GHOST that plays the PURE human-gait MODEL reference -- "the
# ideal we optimise toward". The ghost is offset to the side and its leg
# cycle is locked to the real robot's forward progress, so you can watch the
# ideal target and the actual robot stride together; the difference between
# them is exactly what the RL layer is correcting for.
#
# Real-time paced by default (-Fast for unpaced). Close the window to stop.
# Usage:  powershell -File scripts/dev/run_g1_walk_ghost_demo.ps1 [-Duration 120] [-Fast]
param(
    [int]$Duration = 120,
    [switch]$Fast
)
$root = (Resolve-Path "$PSScriptRoot\..\..\..\..").Path
$world = "$root\projects\policies\research\worlds\g1_walk_ghost_demo.omniworld"

$env:OMNISIM_HOME            = $root
$env:OMNISIM_NEWTON_STATICS      = "1"
$env:OMNISIM_NEWTON_SUBSTEPS     = "4"
$env:OMNISIM_NEWTON_FORCE_MUJOCO = "1"
$env:OMNISIM_NEWTON_MJWARP       = "1"
$env:OMNISIM_URDF_USE_INERTIA    = "1"
$env:OMNISIM_NEWTON_BASE_GUARD   = "1"
$env:OMNISIM_NEWTON_SEED_POSE    = "1"
$env:OMNISIM_NEWTON_SEED_REBUILD = "1"
$env:OMNISIM_NEWTON_TARGET_KE    = "100"
$env:OMNISIM_NEWTON_TARGET_KD    = "5"
$env:OMNISIM_NEWTON_GROUND_MU    = "2.0"
$env:G1_BALANCE_FALLBACK         = "0"
# The real robot runs the HOLISTIC-SHAPE walk policy gpu_g1_walk26_shape_c8.
# WARNING (corrected 2026-06-19): the old "297 m zero falls" claim is a TRAINER/
# old-path number that does NOT reproduce in deploy -- this policy deploy-topples
# in ~1 s. The real, reproducing deploy walk is finite (ft_pdoff_clamp, +5.9 m /
# 33.8 s). Canonical status: docs/developer/rl-current-state.md.
$env:G1_POLICY_ONNX  = "$root\projects\policies\research\training\runs\gpu_g1_walk26_shape_c8\policy.onnx"
$env:G1_GAIT_MODEL   = "human"
$env:G1_GAIT_STYLE   = "winter"
$env:G1_GAIT_VX      = "0.4"
$env:G1_GAIT_FREQ    = "1.3"
$env:G1_GAIT_RAMP_S  = "2.0"
$env:G1_GAIT_A_LAT   = "0.05"
$env:G1_GAIT_A_ARM   = "0.25"
$env:G1_GAIT_HIP_SCALE = "0.9"
$env:G1_OBS_STACK     = "4"        # frame-stack memory (must match training)
$env:G1_OBS_LOOKAHEAD = "0.1,0.4"  # reference forecast (must match training)
Remove-Item Env:G1_START_PHASE -ErrorAction SilentlyContinue
$env:G1_BAL_KP_P = "-1.5"; $env:G1_BAL_KD_P = "-0.2"
$env:G1_BAL_KP_R = "-3.0"; $env:G1_BAL_KD_R = "-0.5"   # stiffer roll: shape reward left lateral free
$env:G1_GHOST_Y  = "1.1"     # sideways offset of the ghost from the real robot
# Private log path: the default omnisim_log.txt may be locked by a concurrent session.
$env:OMNISIM_LOG_PATH = "$root\_scratch\omnisim_log_ghost_demo.txt"

Write-Host "Opening the ghost demo: real robot + kinematic gait-model ghost. Close the window to stop."
if ($Fast) { python -u "$root\scripts\dev\headless_runner.py" $world --gui --duration $Duration }
else       { python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration }
