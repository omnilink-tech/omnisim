# WATCH OmniQuad leap forward beside its ideal GHOST.
# Real OmniQuad (RL, jump ghost) leaps next to a translucent hologram replaying the
# MPPI-generated leap. The gap between them is the RL tracking error.
# Usage: run_omniquad_jump_ghost_demo.ps1 [-Duration 12] [-Fast] [-Policy <onnx>]
param([int]$Duration = 12, [switch]$Fast, [string]$Policy = "")
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
$world = "$root\projects\policies\research\worlds\omniquad_jump_ghost_demo.omniworld"
if ($Policy -eq "") { $Policy = "$root\projects\policies\research\training\runs\gpu_omniquad_jump_a\policy.onnx" }

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
$env:OMNISIM_LOG_PATH = "$root\_scratch\omnisim_log_jump_ghost.txt"
$env:OMNIQUAD_GETUP_REF   = "$root\projects\policies\research\shadowing\ghosts\omniquad_jump_ghost.npz"
$env:OMNIQUAD_GETUP_POLICY = $Policy
$env:OMNIQUAD_GETUP_RES_SCALE = "0.25"
$env:OMNIQUAD_SETTLE_S    = "0.6"
$env:OMNIQUAD_GETUP_HOLD_FADE_S = "0.8"
$env:OMNIQUAD_GETUP_LOG   = "$root\_scratch\omniquad_jump_ghost_real.log"
$env:OMNIQUAD_GHOST_Y     = "1.4"
$env:OMNIQUAD_GHOST_ALPHA = "0.6"
$env:OMNIQUAD_GHOST_TINT  = "0.62,0.82,1.0"

Write-Host "Opening omniquad_jump_ghost_demo.wbt: real OmniQuad leaps beside its ideal ghost. Close the window to stop."
if ($Fast) { python -u "$root\scripts\dev\headless_runner.py" $world --gui --duration $Duration }
else       { python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration }
