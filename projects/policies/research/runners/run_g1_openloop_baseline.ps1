# BASELINE for the deterministic "brain" project.
#
# Drives the PHYSICS G1 with the OPEN-LOOP ghost reference and ZERO feedback
# (G1_BALANCE_FALLBACK=1 -> targets = pure gait-model baseline, no policy, no
# balance PD). This measures how fast the pure feedforward gait model -- the
# ghost -- falls when put on the real physics robot. That fall time / distance
# is the number the deterministic brain's feedback layers must beat.
#
# Same world / physics / gait recipe as run_g1_walk_vc_deploy.ps1 so the
# comparison is apples-to-apples.
#
# Usage:  run_g1_openloop_baseline.ps1 [-Duration 20] [-Gui] [-CpuEngine]
param(
    [int]$Duration = 20,
    [switch]$Gui,
    [switch]$CpuEngine
)
$root  = (Resolve-Path "$PSScriptRoot\..\..\..\..").Path
$world = "$root\projects\policies\research\worlds\g1_walk_arms_deploy.omniworld"
$log   = "$root\_scratch\g1_openloop_baseline.log"
Remove-Item $log -ErrorAction SilentlyContinue

# --- gait model: match the ghost / VC deploy recipe exactly ---
$env:G1_GAIT_MODEL    = "human"
$env:G1_GAIT_STYLE    = "winter"
$env:G1_GAIT_VX       = "0.4"
$env:G1_GAIT_RAMP_S   = "2.0"
$env:G1_GAIT_A_LAT    = "0.05"
$env:G1_GAIT_A_ANKLE  = "0.35"
$env:G1_GAIT_A_ARM    = "0.25"
$env:G1_GAIT_HIP_SCALE = "0.9"
Remove-Item Env:G1_START_PHASE -ErrorAction SilentlyContinue
# Frame-stack/lookahead are policy-obs knobs; irrelevant in fallback but set
# them so the controller code paths match the deploy.
$env:G1_OBS_STACK     = "4"
$env:G1_OBS_LOOKAHEAD = "0.1,0.4"

# --- NO policy, NO balance feedback -> pure open-loop ghost on physics ---
$env:G1_BALANCE_FALLBACK = "1"
Remove-Item Env:G1_POLICY_ONNX -ErrorAction SilentlyContinue
# leave all G1_BAL_* unset -> balance gains default to 0

# --- engine: match the trainer/deploy exactly (mujoco_warp, kp100) ---
$env:OMNISIM_HOME                = $root
$env:OMNISIM_NEWTON_STATICS      = "1"
$env:OMNISIM_NEWTON_SUBSTEPS     = "4"
$env:OMNISIM_NEWTON_FORCE_MUJOCO = "1"
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
$env:OMNISIM_LOG_PATH = "$root\_scratch\omnisim_log_brain.txt"
$env:G1_DEPLOY_LOG    = $log

Write-Host "OPEN-LOOP GHOST baseline: ${Duration}s, no feedback (fallback)..."
if ($Gui) {
    python -u "$root\scripts\dev\headless_runner.py" $world --duration $Duration --gui --realtime 2>&1 | Select-Object -Last 4
} else {
    python -u "$root\scripts\dev\headless_runner.py" $world --duration $Duration 2>&1 | Select-Object -Last 4
}

Write-Host "`n===== OPEN-LOOP GHOST BASELINE RESULT ====="
if (Test-Path $log) {
    $fallLine = Select-String -Path $log -Pattern "FALL" | Select-Object -First 1
    $last = Get-Content $log | Select-Object -Last 1
    if ($fallLine) { Write-Host "  first FALL: $($fallLine.Line.Trim())" }
    else           { Write-Host "  no FALL line (survived upright)" }
    Write-Host "  last: $last"
} else {
    Write-Host "  (no deploy log produced)"
}
