# WATCH OmniQuad get up from the ground beside its ideal GHOST.
#
# Real OmniQuad (RL get-up, teleport start) rises belly-flat -> stand next to a
# translucent hologram replaying the MPPI-generated, verifier-certified get-up
# reference. The gap between them is the RL tracking error.
# Usage: run_omniquad_getup_ghost_demo.ps1 [-Duration 30] [-Fast]
param([int]$Duration = 30, [switch]$Fast)
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
$world = "$root\projects\policies\research\worlds\omniquad_getup_ghost_demo.omniworld"

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
$env:OMNISIM_LOG_PATH = "$root\_scratch\omnisim_log_getup_ghost.txt"
$env:OMNIQUAD_GETUP_REF   = "$root\projects\policies\research\shadowing\ghosts\omniquad_getup_ghost.npz"
$env:OMNIQUAD_GETUP_POLICY = "$root\projects\policies\research\inference\policies\gpu_omniquad_getup_main\policy.onnx"
$env:OMNIQUAD_GETUP_RES_SCALE = "0.15"
$env:OMNIQUAD_SETTLE_S    = "1.0"
$env:OMNIQUAD_GETUP_HOLD_FADE_S = "1.2"
$env:OMNIQUAD_GETUP_LOG   = "$root\_scratch\omniquad_getup_ghost_real.log"
$env:OMNIQUAD_GHOST_Y     = "1.4"
$env:OMNIQUAD_GHOST_ALPHA = "0.6"
$env:OMNIQUAD_GHOST_TINT  = "0.62,0.82,1.0"

Write-Host "Opening omniquad_getup_ghost_demo.wbt: real OmniQuad gets up beside its ideal ghost. Close the window to stop."
if ($Fast) { python -u "$root\scripts\dev\headless_runner.py" $world --gui --duration $Duration }
else       { python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration }
