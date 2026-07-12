# Run the G1 WALKING deploy world headless and report forward progress.
#
# Recipe (2026-06-10, walk15): gait-residual RL trained on the STIFFNESS-MATCHED
# model. The trainer MJCF must match the deploy joint drive EXACTLY:
#   g1_legs_kp100.mjcf.xml (kp=100/kv=5)  <->  OMNISIM_NEWTON_TARGET_KE=100/KD=5
# (The historic ~1.7 s falls were a 20x stiffness mismatch: training on kp=20/kv=3
# while the deploy ran KE=400/KD=60. No DR covers 20x.)
#
# Gait reference: hip 0.35 / knee 0.45 / ankle counter-rotation 0.35 (keeps the
# foot flat through the hip swing -- without it the CPG drifts BACKWARD) / lateral
# sway 0.05, 1.3 Hz, start phase 0 (gait == NOMINAL at phase 0, no target snap).
# The trainer's analytic ankle PD baseline (kp -1.5 / kd -0.2, clamp 0.2) is part
# of the policy's training baseline, so the deploy MUST enable it (G1_BAL_*).
#
# Engine: OMNISIM_NEWTON_MJWARP=1 matches the trainer engine exactly (GPU
# mujoco_warp; ~8x slower than realtime at nworld=1). Unsetting MJWARP runs
# CPU MuJoCo: slightly different contact behaviour -- the policy walks there
# too but it is a distribution shift (c10 walked 2.2 m on CPU vs 6.6 m on
# mjwarp; keep MJWARP=1 for the real result).
#
# RESULT (2026-06-10, gpu_g1_walk15_c12 on mjwarp): +25.9 m in 68.5 s, ZERO
# falls, |roll| <= 0.27 throughout -- crossed the platform end (x=+10 m) at
# t~26 s and kept walking. In-trainer eval: 36.5 s mean first-fall, 14.3 m
# mean / 15.4 m median forward distance (1024 envs, no-DR).
#
# -Arms runs the FULL-BODY (23-DOF legs+arms) variant: g1_walk_arms_deploy.wbt
# + the gpu_g1_walk16_arms policy (walk15 recipe warm-started onto the
# full-body MJCF g1_full_kp100.mjcf.xml; the controller auto-detects and pins
# the arm joints at the trainer's ARM_NOMINAL).
#
# Usage:  powershell -File scripts/dev/run_g1_walk_deploy.ps1 [-Duration 490] [-CpuEngine] [-Arms]
param(
    [int]$Duration = 490,
    [switch]$CpuEngine,
    [switch]$Arms
)
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
if ($Arms) {
    # FULL-BODY, HOLISTIC-SHAPE gait (1B-step run + shape reward): policy
    # gpu_g1_walk26_shape_c8.
    # WARNING (corrected 2026-06-19): the old "walks 297 m ZERO falls" claim is a
    # TRAINER/old-path number that does NOT reproduce in deploy -- this policy now
    # deploy-topples in ~1 s (FALL@1.06s, _scratch/g1_walk_shapec8_deploy.log). The
    # real, reproducing deploy walk is finite: gpu_newton_g1_walk_ft_pdoff_clamp
    # walks +5.9 m / 33.8 s then falls. Prefer scripts/dev/g1_deploy_launch.py;
    # canonical status in docs/developer/rl-current-state.md. Kept here for
    # comparison runs.
    $world  = "$root\projects\policies\worlds\g1_walk_arms_deploy.wbt"
    $policy = "$root\projects\policies\research\training\runs\gpu_g1_walk26_shape_c8\policy.onnx"
    $env:G1_GAIT_MODEL    = "human"
    $env:G1_GAIT_STYLE    = "winter"   # measured human joint kinematics
    $env:G1_GAIT_VX       = "0.4"
    $env:G1_GAIT_RAMP_S   = "2.0"
    $env:G1_OBS_STACK     = "4"        # frame-stack memory (trainer --obs-stack 4)
    $env:G1_OBS_LOOKAHEAD = "0.1,0.4"  # reference forecast (trainer --obs-lookahead)
} else {
    $world  = "$root\projects\policies\worlds\g1_walk_deploy.wbt"
    $policy = "$root\projects\policies\research\training\runs\gpu_g1_walk15_c12\policy.onnx"
    Remove-Item Env:G1_GAIT_MODEL -ErrorAction SilentlyContinue
}
$log = "$root\_scratch\g1_walk_deploy.log"
Remove-Item $log -ErrorAction SilentlyContinue

$env:OMNISIM_HOME            = $root
$env:OMNISIM_NEWTON_STATICS      = "1"
$env:OMNISIM_NEWTON_SUBSTEPS     = "4"
$env:OMNISIM_NEWTON_FORCE_MUJOCO = "1"
$env:OMNISIM_REQUIRE_NEWTON  = "1"   # assert Newton engaged -- fail loud, no silent ODE fallback
$env:OMNISIM_REQUIRE_MUJOCO_SOLVER = "1"   # assert Newton MuJoCo solver (not XPBD) -- the trained engine
if ($CpuEngine) { Remove-Item Env:OMNISIM_NEWTON_MJWARP -ErrorAction SilentlyContinue }
else            { $env:OMNISIM_NEWTON_MJWARP = "1" }
$env:OMNISIM_URDF_USE_INERTIA    = "1"
$env:OMNISIM_NEWTON_BASE_GUARD   = "1"
$env:OMNISIM_NEWTON_SEED_POSE    = "1"
$env:OMNISIM_NEWTON_SEED_REBUILD = "1"
# Joint drive MUST match the trainer MJCF (g1_legs_kp100.mjcf.xml).
$env:OMNISIM_NEWTON_TARGET_KE    = "100"
$env:OMNISIM_NEWTON_TARGET_KD    = "5"
$env:OMNISIM_NEWTON_GROUND_MU    = "2.0"
# RL walk policy (required; the open-loop gait alone does not balance).
$env:G1_BALANCE_FALLBACK         = "0"
$env:G1_POLICY_ONNX              = $policy
# Gait config -- MUST match the policy's training (walk15 recipe).
$env:G1_GAIT_A_LAT               = "0.05"
$env:G1_GAIT_A_ANKLE             = "0.35"
# Counter-phase arm swing (the full-body n6 policy trained with it).
if ($Arms) { $env:G1_GAIT_A_ARM = "0.25"; $env:G1_GAIT_HIP_SCALE = "0.9" } else { $env:G1_GAIT_A_ARM = "0" }   # v3 trained at hip pre-comp 0.9 (deploy MUST match)
# Start phase: sine policies start at 0 (gait==NOMINAL); the human model
# defaults to DS_PHASE (double support) inside the controller -- do not override.
if ($Arms) { Remove-Item Env:G1_START_PHASE -ErrorAction SilentlyContinue } else { $env:G1_START_PHASE = "0" }
# Trainer's analytic ankle PD baseline (G1_TRAIN_BAL_* defaults) -- part of the
# policy's baseline, must be ON in deploy.
$env:G1_BAL_KP_P                 = "-1.5"
$env:G1_BAL_KD_P                 = "-0.2"
# Roll gain: the shape-c8 policy (-Arms) needs a stiffer lateral PD to catch
# the launch roll the sagittal shape reward left unconstrained; the legs
# sine policy keeps the original.
if ($Arms) { $env:G1_BAL_KP_R = "-3.0"; $env:G1_BAL_KD_R = "-0.5" }
else       { $env:G1_BAL_KP_R = "-1.5"; $env:G1_BAL_KD_R = "-0.2" }
$env:G1_DEPLOY_LOG               = $log

Write-Host "Running g1_walk_deploy.wbt for ${Duration}s wall (gait + walk15 RL policy)..."
python -u "$root\scripts\dev\headless_runner.py" $world --duration $Duration 2>&1 | Select-Object -Last 4

Write-Host "`n===== G1 WALK RESULT ====="
if (Test-Path $log) {
    $fall = (Select-String -Path $log -Pattern "FALL" | Measure-Object).Count
    $last = Get-Content $log | Select-Object -Last 1
    if ($fall -eq 0) { Write-Host "  no fall. last: $last" }
    else             { Write-Host "  $fall fall line(s). last: $last" }
} else {
    Write-Host "  (no deploy log produced)"
}





