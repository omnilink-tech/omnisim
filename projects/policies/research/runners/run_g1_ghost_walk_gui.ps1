# WATCH the GHOST-BUILT G1 walk live in a real OmniSim window.
#
# This policy was trained ENTIRELY from a generated, feasible ghost
# (projects/policies/control/gait/g1_squat_ghost.py) via the Shadowing imitation reward on the
# canonical raw-mujoco_warp trainer -- NO Unitree weights. It is an honest
# from-scratch walk: it strides ~0.2 m/s and covers several metres before falling
# (it sits at the documented ~2 m from-scratch durability wall; the BC clone's 89 m
# comes from cloning Unitree's already-solved policy). Close the window to stop.
#
# Usage:  powershell -File scripts/dev/run_g1_ghost_walk_gui.ps1 [-Duration 120] [-Policy <onnx>] [-Vx 0.3] [-Fast]
param(
    [int]$Duration = 120,
    [string]$Policy = "",
    [double]$Vx = 0.3,
    [switch]$Fast
)
$root = (Resolve-Path "$PSScriptRoot\..\..\..\..").Path
$world  = "$root\projects\policies\research\worlds\g1_walk_canon_deploy.omniworld"
if (-not $Policy) {
    $Policy = "$root\projects\policies\research\controllers\g1_ghost_walk\g1_ghost_walk_v6.onnx"  # best deploy transfer (~1.4s upright)
}

$env:OMNISIM_HOME                  = $root
$env:OMNISIM_REQUIRE_NEWTON        = "1"   # assert Newton (the trained engine); no silent ODE
$env:OMNISIM_REQUIRE_MUJOCO_SOLVER = "1"
$env:OMNISIM_NEWTON_FORCE_MUJOCO   = "1"
$env:OMNISIM_NEWTON_MJWARP         = "1"
$env:OMNISIM_NEWTON_STATICS        = "1"
$env:OMNISIM_NEWTON_SUBSTEPS       = "4"
$env:OMNISIM_URDF_USE_INERTIA      = "1"
$env:OMNISIM_NEWTON_BASE_GUARD     = "1"
$env:OMNISIM_NEWTON_SEED_POSE      = "1"
$env:OMNISIM_NEWTON_SEED_REBUILD   = "1"
# CANON deploy joint drive -- MUST match the trainer (g1_cfg kp300/kd15, GROUND_MU 1.0).
$env:OMNISIM_NEWTON_TARGET_KE      = "300"
$env:OMNISIM_NEWTON_TARGET_KD      = "15"
$env:OMNISIM_NEWTON_GROUND_MU      = "1.0"
# the clean canon controller: obs -> onnx -> motors (no gait CPG, no balance PD).
$env:G1_CANON_ONNX                 = $Policy
$env:G1_CMD_VX                     = "$Vx"

Write-Host "Opening g1_walk_canon_deploy.wbt with the GHOST-BUILT policy:`n  $Policy`nWatch G1 walk; close the window to stop."
if ($Fast) { python -u "$root\scripts\dev\headless_runner.py" $world --gui --duration $Duration }
else       { python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration }
