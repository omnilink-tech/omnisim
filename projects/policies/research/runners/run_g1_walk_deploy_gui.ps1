# WATCH G1 walk live in a real OmniSim window.
#
# Uses headless_runner.py --gui so the full Newton/mujoco_warp env config (the
# engine the walk policy was trained on) propagates reliably (os.environ.copy ->
# Popen) while a real 3D window opens. The gait CPG + RL residual walk policy drive
# the legs. Close the window to stop.
#
# Config MUST match scripts/dev/run_g1_walk_deploy.ps1 (the walk15 recipe:
# kp100-matched joint drive, ankle counter-rotation gait, trainer ankle-PD on).
#
# Since the Newton bridge perf pass (per-step readback caches, collide skip,
# dirty-gated state copy-in, CUDA-graph substeps) the sim runs 4-5x FASTER
# than realtime, so the window is paced at 1.0x wall clock (--realtime) and
# the robot walks at true speed. Pass -Fast to run unpaced instead.
#
# -Arms: full-body (legs+arms) variant -- see run_g1_walk_deploy.ps1.
#
# Usage:  powershell -File scripts/dev/run_g1_walk_deploy_gui.ps1 [-Duration 120] [-Fast] [-Arms]
param(
    [int]$Duration = 120,
    [switch]$Fast,
    [switch]$Arms,
    [switch]$Cold    # skip the warm-up world reload (which ENDS a --gui run early);
                     # the DR-trained walk policy is robust enough to walk from cold
)
$root = (Resolve-Path "$PSScriptRoot\..\..\..\..").Path
if ($Arms) {
    # FULL-BODY, HOLISTIC-SHAPE gait: gpu_g1_walk26_shape_c8.
    # WARNING (corrected 2026-06-19): the old "297 m zero falls" claim is a
    # TRAINER/old-path number that does NOT reproduce in deploy -- this policy
    # deploy-topples in ~1 s. The real, reproducing deploy walk is finite:
    # gpu_newton_g1_walk_ft_pdoff_clamp walks +5.9 m / 33.8 s. Prefer
    # scripts/dev/g1_deploy_launch.py; canonical: docs/developer/rl-current-state.md.
    $world  = "$root\projects\policies\research\worlds\g1_walk_arms_deploy.omniworld"
    $policy = "$root\projects\policies\research\training\runs\gpu_g1_walk26_shape_c8\policy.onnx"
    $env:G1_GAIT_MODEL    = "human"
    $env:G1_GAIT_STYLE    = "winter"   # measured human joint kinematics
    $env:G1_GAIT_VX       = "0.4"
    $env:G1_GAIT_RAMP_S   = "2.0"
    $env:G1_OBS_STACK     = "4"
    $env:G1_OBS_LOOKAHEAD = "0.1,0.4"
} else {
    $world  = "$root\projects\policies\research\worlds\g1_walk_deploy.omniworld"
    $policy = "$root\projects\policies\research\training\runs\gpu_g1_walk15_c12\policy.onnx"
    Remove-Item Env:G1_GAIT_MODEL -ErrorAction SilentlyContinue
}

$env:OMNISIM_HOME            = $root
$env:OMNISIM_NEWTON_STATICS      = "1"
$env:OMNISIM_NEWTON_SUBSTEPS     = "4"
$env:OMNISIM_NEWTON_FORCE_MUJOCO = "1"
$env:OMNISIM_REQUIRE_NEWTON  = "1"   # assert Newton engaged -- fail loud, no silent ODE fallback
$env:OMNISIM_REQUIRE_MUJOCO_SOLVER = "1"   # assert Newton MuJoCo solver (not XPBD) -- the trained engine
$env:OMNISIM_NEWTON_MJWARP       = "1"
$env:OMNISIM_URDF_USE_INERTIA    = "1"
$env:OMNISIM_NEWTON_BASE_GUARD   = "1"
$env:OMNISIM_NEWTON_SEED_POSE    = "1"
$env:OMNISIM_NEWTON_SEED_REBUILD = "1"
# Joint drive MUST match the trainer MJCF (g1_legs_kp100.mjcf.xml).
$env:OMNISIM_NEWTON_TARGET_KE    = "100"
$env:OMNISIM_NEWTON_TARGET_KD    = "5"
$env:OMNISIM_NEWTON_GROUND_MU    = "2.0"
$env:G1_BALANCE_FALLBACK         = "0"
$env:G1_POLICY_ONNX              = $policy
# walk15 gait config -- MUST match the policy's training.
$env:G1_GAIT_A_LAT               = "0.05"
$env:G1_GAIT_A_ANKLE             = "0.35"
# Counter-phase arm swing (the full-body n6 policy trained with it).
if ($Arms) { $env:G1_GAIT_A_ARM = "0.25"; $env:G1_GAIT_HIP_SCALE = "0.9" } else { $env:G1_GAIT_A_ARM = "0" }   # v3 trained at hip pre-comp 0.9 (deploy MUST match)
# Start phase: sine policies start at 0 (gait==NOMINAL); the human model
# defaults to DS_PHASE (double support) inside the controller -- do not override.
if ($Arms) { Remove-Item Env:G1_START_PHASE -ErrorAction SilentlyContinue } else { $env:G1_START_PHASE = "0" }
# Trainer's analytic ankle-PD baseline -- part of the policy's baseline.
$env:G1_BAL_KP_P                 = "-1.5"
$env:G1_BAL_KD_P                 = "-0.2"
# shape-c8 (-Arms) needs a stiffer roll PD to catch the launch roll.
if ($Arms) { $env:G1_BAL_KP_R = "-3.0"; $env:G1_BAL_KD_R = "-0.5" }
else       { $env:G1_BAL_KP_R = "-1.5"; $env:G1_BAL_KD_R = "-0.2" }

Write-Host "Opening g1_walk_deploy.wbt in the GUI (walk15 policy). Watch G1 walk; close the window to stop."
# The warm-up reload (default) reloads the world once at startup; in a --gui run
# that reload ENDS the session early. -Cold skips it so the window stays open.
$coldArg = if ($Cold) { "--cold" } else { "" }
if ($Fast) { python -u "$root\scripts\dev\headless_runner.py" $world --gui $coldArg --duration $Duration }
else       { python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime $coldArg --duration $Duration }





