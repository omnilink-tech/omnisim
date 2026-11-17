# Run the DETERMINISTIC BRAIN in OmniSim (Newton) -- no RL, no ONNX.
#
# This is the OmniSim milestone path for the deterministic-brain project
# (docs/developer/g1-deterministic-brain.md). It drives the PHYSICS G1 with the
# ONE consolidated brain projects/policies/control/g1_brain.py: the hand-written
# controller (ghost feedforward + deterministic balance feedback) instead of the
# PPO policy. G1_BRAIN=1 makes the deploy controller call the brain for the full
# joint command each tick (control.g1_brain is now the default module).
#
# Same world / physics / gait recipe as run_g1_openloop_baseline.ps1 and
# run_g1_walk_vc_deploy.ps1, so brain-vs-openloop-vs-RL is apples-to-apples.
# The open-loop baseline (run_g1_openloop_baseline.ps1) is the number the
# brain's feedback layers must beat.
#
# Tune any gain from here WITHOUT editing code -- BrainConfig.from_env reads
# G1_BRAIN_<FIELD>, e.g.:
#   $env:G1_BRAIN_KP_AR = "-4.0"; run_g1_brain_deploy.ps1
#   $env:G1_BRAIN_KFP_Y = "0.12"; run_g1_brain_deploy.ps1
#
# Usage:  run_g1_brain_deploy.ps1 [-Duration 40] [-Gui] [-CpuEngine] [-Legs] [-Style ik|winter]
param(
    [int]$Duration = 40,
    [switch]$Gui,
    [switch]$CpuEngine,
    [switch]$Legs,          # legs-only world (faster); default is full-body (arms, matches the ghost)
    [string]$Style = "ik"   # brain feedforward: "ik" foot-space (capture-point) | "winter" (ghost look)
)
$root  = (Resolve-Path "$PSScriptRoot\..\..\..\..").Path
if ($Legs) { $world = "$root\projects\policies\research\worlds\g1_walk_deploy.omniworld" }
else       { $world = "$root\projects\policies\research\worlds\g1_walk_arms_deploy.omniworld" }
$log   = "$root\_scratch\g1_brain_deploy.log"
Remove-Item $log -ErrorAction SilentlyContinue

# --- THE BRAIN (deterministic controller) ON ---
$env:G1_BRAIN        = "1"
$env:G1_BRAIN_MODULE = "projects.policies.control.g1_brain"   # the one consolidated brain (also the default)
$env:G1_BRAIN_STYLE  = $Style
# brain owns balance + clock -> no velocity-command (VC) layer
Remove-Item Env:G1_VX_CMD_MAX -ErrorAction SilentlyContinue

# --- gait model: match the ghost / VC deploy recipe exactly (the brain's
#     feedforward layer reads these via G1_GAIT_*; BrainConfig mirrors the
#     ones it needs as defaults, but we set them so the trace/arms agree) ---
$env:G1_GAIT_MODEL    = "human"
$env:G1_GAIT_STYLE    = "winter"
$env:G1_GAIT_VX       = "0.4"
$env:G1_GAIT_FREQ     = "1.3"
$env:G1_GAIT_RAMP_S   = "2.0"
$env:G1_GAIT_A_LAT    = "0.05"
$env:G1_GAIT_A_ANKLE  = "0.35"
$env:G1_GAIT_A_ARM    = "0.25"
$env:G1_GAIT_HIP_SCALE = "0.9"
Remove-Item Env:G1_START_PHASE -ErrorAction SilentlyContinue
$env:G1_OBS_STACK     = "4"
$env:G1_OBS_LOOKAHEAD = "0.1,0.4"

# --- NO policy: the brain replaces it. Fallback=1 keeps the (skipped)
#     pre-brain path cheap (no ONNX session); the brain overrides the command. ---
$env:G1_BALANCE_FALLBACK = "1"
Remove-Item Env:G1_POLICY_ONNX -ErrorAction SilentlyContinue

# --- engine: match the trainer/deploy exactly (mujoco_warp, kp100) ---
$env:OMNISIM_HOME                = $root
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
$env:OMNISIM_NEWTON_TARGET_KE    = "100"
$env:OMNISIM_NEWTON_TARGET_KD    = "5"
$env:OMNISIM_NEWTON_GROUND_MU    = "2.0"
# private log path so a concurrent session's run doesn't clash
$env:OMNISIM_LOG_PATH = "$root\_scratch\omnisim_log_brain_deploy.txt"
$env:G1_DEPLOY_LOG    = $log

$variant = if ($Legs) { "legs" } else { "full-body (arms)" }
Write-Host "DETERMINISTIC BRAIN deploy (control.g1_brain, style=$Style, $variant): ${Duration}s wall, Newton/mjwarp, no RL..."
if ($Gui) {
    python -u "$root\scripts\dev\headless_runner.py" $world --duration $Duration --gui --realtime 2>&1 | Select-Object -Last 4
} else {
    python -u "$root\scripts\dev\headless_runner.py" $world --duration $Duration 2>&1 | Select-Object -Last 4
}

Write-Host "`n===== G1 DETERMINISTIC BRAIN RESULT ====="
if (Test-Path $log) {
    $fall = (Select-String -Path $log -Pattern "FALL" | Measure-Object).Count
    $fallLine = Select-String -Path $log -Pattern "FALL" | Select-Object -First 1
    $last = Get-Content $log | Select-Object -Last 1
    if ($fall -eq 0) { Write-Host "  no FALL line. last: $last" }
    else             { Write-Host "  first FALL: $($fallLine.Line.Trim())"; Write-Host "  last: $last" }
} else {
    Write-Host "  (no deploy log produced)"
}
