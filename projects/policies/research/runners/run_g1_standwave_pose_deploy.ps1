# DEPLOY the stand+wave with the PROVEN PURE-POSE stand recipe (no RL policy, ankle PD OFF).
#
# Hypothesis (from the review): the stand+wave fell in deploy NOT because of the wave, but
# because the standwave path stacked TWO destabilisers on top of the legs:
#   (1) the RL leg residual (destabilises stiff mujoco_warp contact), and
#   (2) the hardcoded analytic ankle PD (KP_ANKLE_PITCH=-1.5) which kicks roll at handover.
# The PROVEN g1 stand (run_g1_stand_deploy.ps1) stands indefinitely by turning BOTH off:
# a stiff position hold (KE=400/KD=60) of the statically-stable deeper-squat NOMINAL, no RL,
# no ankle PD. This script applies that EXACT recipe to the full body and drives ONLY the
# right arm along the slow wave reference (feedforward). The stiff stance absorbs the small,
# slow, near-body arm perturbation -- the lowest-risk path to a deployed stand+wave.
#
# Usage:
#   powershell -File scripts/dev/run_g1_standwave_pose_deploy.ps1 [-Duration 60]      # headless survival check (solo world)
#   powershell -File scripts/dev/run_g1_standwave_pose_deploy.ps1 -Gui [-Duration 60] # GUI visual (real robot + translucent ghost)
#   add -Regen to force-regenerate the ghost npz.
param(
    [int]$Duration = 60,
    [switch]$Gui,
    [switch]$Regen
)
$root = (Resolve-Path "$PSScriptRoot\..\..\..\..").Path
$worldSolo = "$root\projects\policies\research\worlds\g1_standwave_deploy_solo.omniworld"
$worldDuo  = "$root\projects\policies\research\worlds\g1_standwave_deploy.omniworld"
$npz   = "$root\_scratch\g1_standwave_ghost_generated.npz"
$csv   = "$root\_scratch\g1_standwave_replay.csv"
$mjcf  = "$root\projects\robots\unitree\g1\urdf\g1_full_kp100.mjcf.xml"
$log   = "$root\_scratch\g1_standwave_pose_deploy.log"
Remove-Item $log -ErrorAction SilentlyContinue

$env:OMNISIM_HOME      = $root
$env:PYTHONUTF8        = "1"
# --- the deploy-faithful Newton solver config, IDENTICAL to the PROVEN g1 stand ---
$env:OMNISIM_NEWTON_STATICS      = "1"
$env:OMNISIM_NEWTON_SUBSTEPS     = "4"
$env:OMNISIM_NEWTON_FORCE_MUJOCO = "1"
$env:OMNISIM_NEWTON_MJWARP       = "1"
$env:OMNISIM_URDF_USE_INERTIA    = "1"
$env:OMNISIM_NEWTON_BASE_GUARD   = "1"
$env:OMNISIM_NEWTON_SEED_POSE    = "1"
$env:OMNISIM_NEWTON_SEED_REBUILD = "1"
# --- STIFF position PD (the proven stand value; NOT the soft kp100 the RL path used) ---
$env:OMNISIM_NEWTON_TARGET_KE    = "400"
$env:OMNISIM_NEWTON_TARGET_KD    = "60"
$env:OMNISIM_NEWTON_GROUND_MU    = "2.0"
# --- PURE-POSE control: no RL residual, ankle PD OFF (the two things that toppled the RL path) ---
$env:G1_BALANCE_FALLBACK = "1"      # pure NOMINAL hold, no policy residual
$env:G1_ARMS_ANKLE_KP    = "0"      # ankle-PD OFF (it destabilised roll at handover)
$env:G1_ARMS_ANKLE_KD    = "0"
$env:G1_ARM_POSE         = "down"   # symmetric arms-down hold (CoM-neutral) when not waving
$env:G1_DEPLOY_LOG       = $log
$env:G1_STANDWAVE_LOG    = $log
$env:OMNISIM_LOG_PATH    = "$root\_scratch\omnisim_log_standwave_pose_deploy.txt"
# make sure no stray policy path is inherited (fallback ignores it, but keep it clean)
Remove-Item Env:\G1_POLICY_ONNX -ErrorAction SilentlyContinue

if ($Regen -or -not (Test-Path $npz)) {
    Write-Host "Generating the stand+wave ghost..."
    python -u "$root\projects\policies\research\shadowing\generate_g1_standwave.py"
}
Write-Host "Converting ghost -> replay CSV..."
python -u "$root\projects\policies\research\shadowing\ghost_to_replay_csv.py" --npz $npz --mjcf $mjcf --out $csv
$env:G1_STANDWAVE_REF  = $csv     # the real robot's arms shadow this (legs hold NOMINAL)
$env:G1_GHOST_REPLAY   = $csv     # the translucent ghost replays the same (GUI/duo world only)
$env:G1_GHOST_ALPHA    = "0.55"
$env:G1_STANDWAVE_LOOP = "1"

if ($Gui) {
    Write-Host "Deploy (GUI, real-time) -- real robot stands & waves, ghost beside it. Close window to stop."
    python -u "$root\scripts\dev\headless_runner.py" $worldDuo --gui --realtime --duration $Duration
} else {
    Write-Host "Deploy (headless survival check, solo world) for ${Duration}s -- pure-pose stand + feedforward wave..."
    python -u "$root\scripts\dev\headless_runner.py" $worldSolo --duration $Duration 2>&1 | Select-Object -Last 6
}

Write-Host "`n===== STAND+WAVE (PURE-POSE) DEPLOY RESULT ====="
if (Test-Path $log) {
    $fall = (Select-String -Path $log -Pattern "FALL" | Measure-Object).Count
    $last = Get-Content $log | Select-Object -Last 1
    if ($fall -eq 0) { Write-Host "  PASS - no fall. last: $last" }
    else             { Write-Host "  $fall FALL line(s). last: $last" }
} else {
    Write-Host "  (no deploy log produced)"
}
