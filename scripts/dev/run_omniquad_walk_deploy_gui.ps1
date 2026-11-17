# WATCH OmniQuad walk live in a real OmniSim window.
#
# The G1 GUI launcher pattern: headless_runner.py --gui so the full
# Newton/mujoco_warp env config (the engine the policy was trained on)
# propagates reliably while a real 3D window opens. The foot-space trot
# model + RL residual drive the legs; the heading hold keeps it straight.
# Paced at 1.0x wall clock (--realtime); pass -Fast to run unpaced.
#
# Config MUST match scripts/dev/run_omniquad_walk_deploy.ps1.
#
# Usage:  powershell -File scripts/dev/run_omniquad_walk_deploy_gui.ps1 [-Duration 120] [-Fast]
param(
    [int]$Duration = 120,
    [switch]$Fast,
    [string]$Policy = ""
)
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
$world = "$root\projects\policies\research\worlds\omniquad_walk_deploy.omniworld"
if ($Policy -eq "") {
    $Policy = "$root\projects\policies\research\inference\policies\gpu_omniquad_walk_main\policy.onnx"
}

$env:OMNISIM_HOME                = $root
$env:OMNISIM_NEWTON_FORCE_MUJOCO = "1"
$env:OMNISIM_REQUIRE_NEWTON  = "1"   # assert Newton engaged -- fail loud, no silent ODE fallback
$env:OMNISIM_REQUIRE_MUJOCO_SOLVER = "1"   # assert Newton MuJoCo solver (not XPBD) -- the trained engine
$env:OMNISIM_NEWTON_MJWARP       = "1"
$env:OMNISIM_URDF_USE_INERTIA    = "1"
$env:OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE = "1"
$env:OMNISIM_NEWTON_SEED_POSE    = "1"
$env:OMNISIM_NEWTON_GROUND_MU    = "2.0"
$env:OMNISIM_NEWTON_SUBSTEPS     = "8"
# Joint drive MUST match the trainer MJCF (kp=500/kv=60).
$env:OMNISIM_NEWTON_TARGET_KE    = "500"
$env:OMNISIM_NEWTON_TARGET_KD    = "60"
$env:OMNIQUAD_POLICY_ONNX            = $Policy
# Gait config -- MUST match the policy's training run.
$env:OMNIQUAD_GAIT_VX     = "0.4"
$env:OMNIQUAD_GAIT_FREQ   = "1.4"
$env:OMNIQUAD_GAIT_DUTY   = "0.6"
$env:OMNIQUAD_GAIT_STEP_H = "0.06"
$env:OMNIQUAD_GAIT_BODY_H = "0.55"
$env:OMNIQUAD_GAIT_RAMP_S = "1.0"
$env:OMNIQUAD_ACT_SCALE   = "0.15"

Write-Host "Opening omniquad_walk_deploy.wbt in the GUI. Watch OmniQuad trot; close the window to stop."
if ($Fast) { python -u "$root\scripts\dev\headless_runner.py" $world --gui --duration $Duration }
else       { python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration }
