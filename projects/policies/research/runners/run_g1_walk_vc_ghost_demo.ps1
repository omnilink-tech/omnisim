# WATCH the velocity-conditioned STOP-IN-THE-MIDDLE milestone beside the GHOST.
#
# Two G1s in the ghost world: the real RL robot runs the velocity-conditioned
# policy (gpu_g1_walk29_vc_c10) and stops on command mid-walk; the kinematic
# physics-free GHOST plays the pure gait-model reference. The ghost's leg cycle
# is locked to the REAL robot's FORWARD PROGRESS (g1_ghost.py), so when the real
# robot stops, the ghost FREEZES mid-stride too -- and both resume together.
# That makes the stop-in-the-middle visible against the ideal target.
#
# Real-time paced. Close the window to stop.
# Usage:
#   run_g1_walk_vc_ghost_demo.ps1 [-Duration 60] [-StandAt 6] [-StandFor 5] [-Chunk 10] [-Fast]
param(
    [int]$Duration = 60,
    [double]$StandAt = 6.0,
    [double]$StandFor = 5.0,
    [int]$Chunk = 10,
    [switch]$Fast
)
$root  = (Resolve-Path "$PSScriptRoot\..\..\..\..").Path
$world = "$root\projects\policies\research\worlds\g1_walk_ghost_demo.omniworld"
$policy = "$root\projects\policies\research\training\runs\gpu_g1_walk29_vc_c$Chunk\policy.onnx"
if (-not (Test-Path $policy)) { Write-Host "missing $policy"; exit 1 }

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

$env:G1_BALANCE_FALLBACK = "0"
$env:G1_POLICY_ONNX      = $policy
# shape-c8 recipe (the VC policy was warm-started from it; deploy MUST match).
$env:G1_GAIT_MODEL   = "human"
$env:G1_GAIT_STYLE   = "winter"
$env:G1_GAIT_VX      = "0.4"
$env:G1_GAIT_FREQ    = "1.3"
$env:G1_GAIT_RAMP_S  = "2.0"
$env:G1_GAIT_A_LAT   = "0.05"
$env:G1_GAIT_A_ANKLE = "0.35"
$env:G1_GAIT_A_ARM   = "0.25"
$env:G1_GAIT_HIP_SCALE = "0.9"
$env:G1_OBS_STACK     = "4"
$env:G1_OBS_LOOKAHEAD = "0.1,0.4"
Remove-Item Env:G1_START_PHASE -ErrorAction SilentlyContinue
$env:G1_BAL_KP_P = "-1.5"; $env:G1_BAL_KD_P = "-0.2"
$env:G1_BAL_KP_R = "-3.0"; $env:G1_BAL_KD_R = "-0.5"

# --- velocity conditioning + the stop-in-the-middle schedule ---
$env:G1_VX_CMD_MAX      = "0.45"      # MUST match trainer --vx-cmd-max
$env:G1_VX_PHASE_FREEZE = "0.0"       # MUST match training (freeze off)
$env:G1_VX_LAUNCH_HOLD_S = "0.0"      # launch walking (robust)
$env:G1_STAND_AT_S   = "$StandAt"
$env:G1_STAND_FOR_S  = "$StandFor"
$env:G1_MODE_BLEND_S = "1.5"

$env:G1_GHOST_Y  = "1.1"              # sideways offset of the ghost
$env:OMNISIM_LOG_PATH = "$root\_scratch\omnisim_log_vc_ghost_demo.txt"

Write-Host "VC ghost demo (chunk $Chunk): real robot + gait-model ghost, STOP at ${StandAt}s for ${StandFor}s."
Write-Host "Watch BOTH the robot and the ghost pause mid-stride, then resume together. Close the window to stop."
if ($Fast) { python -u "$root\scripts\dev\headless_runner.py" $world --gui --duration $Duration }
else       { python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration }
