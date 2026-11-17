# WATCH the seated G1 mimic its translucent ghost, live in OmniSim.
#
# Two seated G1s side by side: a kinematic translucent GHOST that DISPLAYS a
# shared seated arm-gesture reference (right-arm wave + a slow left-arm lift),
# and the real, Newton-simulated robot that MIMICS it by position-tracking the
# SAME reference. Both pelvises are pinned (staticBase) so there is no balance
# problem -- this isolates and validates the shadow -> ghost -> mimic pipeline.
#
# Default: DETERMINISTIC control, pelvis pinned. With -Policy: the trained RL
# balance policy drives the legs+waist and the robot sits UNPINNED, dynamically
# balancing on the chair while the arms still follow the ghost wave.
# Real-time paced by default.
# Usage:
#   powershell -File scripts/dev/run_g1_sit_mimic.ps1 [-Duration 120] [-Fast] [-Headless] [-Policy]
param(
    [int]$Duration = 120,
    [switch]$Fast,
    [switch]$Headless,
    [switch]$Policy
)
$root = (Resolve-Path "$PSScriptRoot\..\..\..\..").Path
$world = "$root\projects\policies\research\worlds\g1_sit_mimic.omniworld"

$env:OMNISIM_HOME                = $root
# Newton (SolverMuJoCo) env for the real robot. The world also sets
# newtonSolver "mujoco"; these match the proven staticBase-arm deploy env.
$env:OMNISIM_NEWTON_STATICS      = "1"
$env:OMNISIM_NEWTON_SUBSTEPS     = "4"
$env:OMNISIM_NEWTON_FORCE_MUJOCO = "1"
$env:OMNISIM_URDF_USE_INERTIA    = "1"
$env:OMNISIM_NEWTON_BASE_GUARD   = "1"
# Spawn the robot ALREADY in the seated pose (seed the joints to the controller's
# first position targets) instead of straight-legged. Without this the legs fall
# straight at spawn, drive the feet into the floor, and JAM there -- the hip servo
# then can't lift the planted feet, so the legs sprawl on the ground while the
# welded pelvis stays up (the "butt on the chair, body on the floor" failure).
$env:OMNISIM_NEWTON_SEED_POSE    = "1"
$env:OMNISIM_NEWTON_SEED_REBUILD = "1"
# Stiffen the position-control servo so the waving arm tracks the reference with
# low phase lag (the default ke=20 is heavily overdamped -> ~0.2 rad lag at the
# fastest part of the wave). 400 is the proven G1 deploy joint stiffness. Only
# the real robot has Newton hinges; the ghost is kinematic, so this is harmless
# to it.
$env:OMNISIM_NEWTON_TARGET_KE    = "400"
# RL balance policy (-Policy): the g1_seated_mimic controller loads this ONNX,
# drives legs+waist as a residual, and runs UNPINNED (it does the balancing).
if ($Policy) {
    # gpu_g1_sit_back2 = the robust backrest policy: unpinned, actively RL-balances
    # seated against the chair back, does the full arm wave (~11 deg rock).
    $env:G1_SIT_POLICY = "$root\projects\policies\research\training\runs\gpu_g1_sit_back2\policy.onnx"
    # ACHIEVABLE GHOST: the ghost replays the robot's recorded achievable motion
    # (the physically-real seated wave, since a bolt-upright sit is impossible for
    # this no-torso-pitch G1), so the robot matches the ghost ~100%. Drop this env
    # to compare against the idealized (unreachable) ghost instead.
    $env:G1_GHOST_REPLAY = "$root\projects\policies\research\training\runs\gpu_g1_sit_back2\achieved.csv"
    Write-Host "RL balance policy ON (unpinned) + achievable ghost: $env:G1_SIT_POLICY"
} else {
    Remove-Item Env:\G1_SIT_POLICY -ErrorAction SilentlyContinue
    Remove-Item Env:\G1_GHOST_REPLAY -ErrorAction SilentlyContinue
}
# Per-tick tracking telemetry from the mimic controller.
$env:G1_SIT_LOG                  = "$root\_scratch\g1_sit_mimic.log"
# Private sim log (the default omnisim_log.txt may be locked by a concurrent session).
$env:OMNISIM_LOG_PATH            = "$root\_scratch\omnisim_log_sit_mimic.txt"

if ($Headless) {
    Write-Host "Seated mimic (HEADLESS verify) -- $Duration s. Telemetry: $env:G1_SIT_LOG"
    python -u "$root\scripts\dev\headless_runner.py" $world --duration $Duration
} elseif ($Fast) {
    Write-Host "Seated mimic (GUI, unpaced) -- close the window to stop."
    python -u "$root\scripts\dev\headless_runner.py" $world --gui --duration $Duration
} else {
    Write-Host "Seated mimic (GUI, real-time) -- close the window to stop."
    python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration
}
