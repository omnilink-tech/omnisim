# Run the Spot Newton walking deploy headless and report straight-walk metrics.
#
# Recipe (2026-06-11, post self-collision fix): the W6 "Spot deploy collapse"
# was intra-robot self-collision introduced by W1 native meshes (chassis hull
# vs upper-leg hulls); the bridge now filters intra-robot pairs by default
# (Webots selfCollision FALSE semantics). On the honest post-fix geometry:
#   - OMNISIM_NEWTON_TARGET_KE=500 / KD=200 (the May-era 250/60 leaned on the
#     phantom AABB-box leg colliders and is too soft now)
#   - OMNISIM_NEWTON_SUBSTEPS=8 (2 ms physics, matches the mjwarp trainer)
#   - SPOT_GAIT_STEP_HEIGHT=0.09 (the default 0.05 swing arc grazes the
#     ground and drags the body BACKWARD)
# Policy: a residual ONNX trained through this exact stack via
#   python projects/policies/research/training/train_residual_newton.py (KE/KD/SPOT_GAIT_*
#   are honored from the environment).
#
# VERIFIED STATE 2026-06-11 (evening): with the spot_gait yaw-sweep fix +
# the tuned heading lock below, the model-only walker holds heading to
# +5.6 deg over 30 s (-1.4 deg over 60 s!), lateral +0.19 m at 30 s,
# 100% upright, ZERO falls, fully deterministic: forward +2.79 m / 30 s
# (the legacy verify bar wants >3.0 m -- calibrated for the pre-W1
# box-collider physics; distance is the open item, not straightness).
# Over 60 s a slow lateral crab (~+2.8 m) leaks in at locked heading --
# constant vy trims and vy feedback both destabilize the gait, so the
# crab is the residual policy's job (training infra now reliable after
# the env port-collision fix in spot_residual_env).
#
# Usage:  powershell -File scripts/dev/run_spot_walk_newton.ps1 [-Duration 30] [-Policy <onnx>]
param(
    [int]$Duration = 30,
    [string]$Policy = ""
)
$root = (Resolve-Path "$PSScriptRoot\..\..").Path

$env:OMNISIM_HOME                       = $root
$env:OMNISIM_URDF_USE_INERTIA           = "1"
$env:OMNISIM_NEWTON_FORCE_MUJOCO        = "1"
$env:OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE = "1"
$env:OMNISIM_NEWTON_SEED_POSE           = "1"
$env:OMNISIM_NEWTON_TARGET_KE           = "500"
$env:OMNISIM_NEWTON_TARGET_KD           = "200"
$env:OMNISIM_NEWTON_SUBSTEPS            = "8"
$env:OMNISIM_NEWTON_GROUND_MU           = "2.0"
$env:SPOT_GAIT_STEP_HEIGHT              = "0.09"
# Heading lock (model-only path) -- kp/clip/lat2yaw tuned on the post-fix
# gait; stronger gains overcorrect and destabilize (the wz authority is
# weak, ~0.1x, through compliant legs on point feet).
$env:SPOT_HEADING_LOCK                  = "1"
$env:SPOT_HEADING_KP                    = "1.0"
$env:SPOT_HEADING_CLIP                  = "0.20"
$env:SPOT_HEADING_LAT2YAW               = "0.10"
# Same hold for the residual-deploy path (zero kd / zero sidestep = the
# verified configuration; the deploy controller's hold defaults are
# ODE-era and misbehave on the current physics).
$env:SPOT_HOLD_KP_YAW                   = "1.0"
$env:SPOT_HOLD_KD_YAW                   = "0"
$env:SPOT_HOLD_KP_LAT2YAW               = "0.10"
$env:SPOT_HOLD_WZ_MAX                   = "0.20"
$env:SPOT_HOLD_KP_LAT                   = "0"
$env:SPOT_HOLD_VY_MAX                   = "0"

if ($Policy -eq "") {
    python "$root\projects\policies\research\tools\verify_straight_walk.py" --no-policy `
        --world "$root\projects\policies\worlds\spot_model_walk_demo.wbt" --duration $Duration
} else {
    python "$root\projects\policies\research\tools\verify_straight_walk.py" --policy $Policy `
        --world "$root\projects\policies\worlds\spot_residual_deploy.wbt" --duration $Duration
}
