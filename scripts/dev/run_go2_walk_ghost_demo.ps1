# WATCH the Unitree Go2 walk beside its gait-model GHOST in a real OmniSim window.
#
# The OmniQuad/G1 ghost demo ported to Go2: the real robot (trot model + RL
# residual, Newton/mjwarp) walks while a translucent pale-blue HOLOGRAM Go2
# plays the PURE trot model beside it, phase-locked to the real robot's
# forward progress. The gap between them IS the RL correction.
#
# Config MUST match scripts/dev/run_go2_walk_deploy.ps1.
#
# Usage:  powershell -File scripts/dev/run_go2_walk_ghost_demo.ps1 [-Duration 180] [-Fast]
param(
    [int]$Duration = 180,
    [switch]$Fast
)
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
$world = "$root\projects\policies\research\worlds\go2_walk_ghost_demo.omniworld"

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
$env:GO2_POLICY_ONNX = "$root\projects\policies\research\inference\policies\gpu_go2_walk_main\policy.onnx"
# Gait config -- shared by the real controller AND the ghost.
$env:GO2_GAIT_VX     = "0.4"
$env:GO2_GAIT_FREQ   = "1.8"
$env:GO2_GAIT_DUTY   = "0.6"
$env:GO2_GAIT_STEP_H = "0.05"
$env:GO2_GAIT_BODY_H = "0.30"
$env:GO2_GAIT_RAMP_S = "1.0"
$env:GO2_ACT_SCALE   = "0.15"
# Hologram look.
$env:GO2_GHOST_Y     = "0.7"
$env:GO2_GHOST_ALPHA = "0.6"
$env:GO2_GHOST_TINT  = "0.62,0.82,1.0"

Write-Host "Opening go2_walk_ghost_demo.wbt: real Go2 + hologram trot-model ghost. Close the window to stop."
if ($Fast) { python -u "$root\scripts\dev\headless_runner.py" $world --gui --duration $Duration }
else       { python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration }
