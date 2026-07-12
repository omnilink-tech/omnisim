# WATCH Spot walk beside its gait-model GHOST in a real OmniSim window.
#
# The G1 ghost demo ported to Spot: the real robot (trot model + RL
# residual, Newton/mjwarp) walks while a translucent pale-blue HOLOGRAM
# Spot plays the PURE trot model beside it, phase-locked to the real
# robot's forward progress. The gap between them IS the RL correction.
#
# Config MUST match scripts/dev/run_spot_walk_deploy.ps1.
#
# Usage:  powershell -File scripts/dev/run_spot_walk_ghost_demo.ps1 [-Duration 180] [-Fast]
param(
    [int]$Duration = 180,
    [switch]$Fast
)
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
$world = "$root\projects\policies\research\worlds\spot_walk_ghost_demo.wbt"

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
$env:SPOT_POLICY_ONNX = "$root\projects\policies\research\inference\policies\gpu_spot_walk_main\policy.onnx"
# Gait config -- shared by the real controller AND the ghost.
$env:SPOT_GAIT_VX     = "0.4"
$env:SPOT_GAIT_FREQ   = "1.4"
$env:SPOT_GAIT_DUTY   = "0.6"
$env:SPOT_GAIT_STEP_H = "0.06"
$env:SPOT_GAIT_BODY_H = "0.55"
$env:SPOT_GAIT_RAMP_S = "1.0"
$env:SPOT_ACT_SCALE   = "0.15"
# Hologram look.
$env:SPOT_GHOST_Y     = "1.4"
$env:SPOT_GHOST_ALPHA = "0.6"
$env:SPOT_GHOST_TINT  = "0.62,0.82,1.0"

Write-Host "Opening spot_walk_ghost_demo.wbt: real Spot + hologram trot-model ghost. Close the window to stop."
if ($Fast) { python -u "$root\scripts\dev\headless_runner.py" $world --gui --duration $Duration }
else       { python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration }
