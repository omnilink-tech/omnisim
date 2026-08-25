# DEPLOY the parity-trained full-body G1 stand (and optional feedforward wave).
#
# This mirrors the H100 training run gpu_g1_arms_stand_parity_c1 with EVERY train<->deploy
# mismatch locked down -- the gaps found 2026-06-23 that made earlier stands fail:
#   * OMNISIM_NEWTON_USE_LINK_COM=1  -> true URDF link COM (the deploy default puts link COMs
#     at the ORIGIN, body_ipos off by up to 0.154 m; this alone moved the legs-only tip 1.30->1.89s).
#   * mujoco_warp engine (NOT silent XPBD) -- the trainer's engine; verify the omnisim log says mjwarp.
#   * KE=100/KD=5 -> matches the g1_full_kp100 MJCF kp=100/kv=5 the policy trained on.
#   * ankle PD LEFT AT DEFAULT (-1.5) -> matches the trainer baseline (do NOT zero it for an RL policy;
#     train and deploy baselines must be identical).
#   * deeper-squat NOMINAL, body-frame ang-vel, proj-g, joint order -- already in g1_stand_arms_deploy.
#
# Runs --cold by default (the headless WARM reload currently CRASHES the 23-DOF Newton robot --
# a separate engine bug to fix if cold tracking proves insufficient).
#
# Usage:
#   powershell -File scripts/dev/run_g1_arms_stand_parity_deploy.ps1 [-Duration 30]          # stand-only survival check
#   powershell -File scripts/dev/run_g1_arms_stand_parity_deploy.ps1 -Wave [-Duration 60]    # + feedforward wave
#   powershell -File scripts/dev/run_g1_arms_stand_parity_deploy.ps1 -Gui -Wave              # GUI visual (real robot + ghost)
param(
    [int]$Duration = 30,
    [switch]$Wave,
    [switch]$Gui,
    [switch]$Warm
)
$root = (Resolve-Path "$PSScriptRoot\..\..\..\..").Path
$run  = "gpu_g1_arms_stand_parity_c1"
$pol  = "$root\projects\policies\research\training\runs\$run\policy.onnx"
$worldSolo = "$root\projects\policies\research\worlds\g1_standwave_deploy_solo.omniworld"
$worldDuo  = "$root\projects\policies\research\worlds\g1_standwave_deploy.omniworld"
$npz   = "$root\_scratch\g1_standwave_ghost_generated.npz"
$csv   = "$root\_scratch\g1_standwave_replay.csv"
$mjcf  = "$root\projects\robots\unitree\g1\urdf\g1_full_kp100.mjcf.xml"
$log   = "$root\_scratch\g1_arms_stand_parity_deploy.log"
Remove-Item $log -ErrorAction SilentlyContinue

if (-not (Test-Path $pol)) {
    Write-Host "WARN: policy not found at $pol -- train it locally first (training is in-engine/local;"
    Write-Host "      the cloud/Modal pull path was removed in ef46a52e). See projects/policies/training/."
}

$env:OMNISIM_HOME      = $root
$env:PYTHONUTF8        = "1"
# --- deploy-faithful Newton solver config (matches the trainer engine) ---
$env:OMNISIM_NEWTON_STATICS      = "1"
$env:OMNISIM_NEWTON_SUBSTEPS     = "4"
$env:OMNISIM_NEWTON_FORCE_MUJOCO = "1"
$env:OMNISIM_NEWTON_MJWARP       = "1"
$env:OMNISIM_URDF_USE_INERTIA    = "1"
$env:OMNISIM_NEWTON_BASE_GUARD   = "1"
$env:OMNISIM_NEWTON_SEED_POSE    = "1"
$env:OMNISIM_NEWTON_SEED_REBUILD = "1"
$env:OMNISIM_NEWTON_USE_LINK_COM = "1"     # <-- TRUE link COM = trainer parity (the key fix)
$env:OMNISIM_NEWTON_TARGET_KE    = "100"    # match g1_full_kp100 kp=100
$env:OMNISIM_NEWTON_TARGET_KD    = "5"       # match kv=5
$env:OMNISIM_NEWTON_GROUND_MU    = "2.0"
$env:G1_NJMAX                    = "256"    # match the trainer constraint budget
# ankle PD: LEAVE DEFAULT (-1.5) to match the trainer baseline -> do NOT set G1_ARMS_ANKLE_KP/KD.
Remove-Item Env:\G1_ARMS_ANKLE_KP -ErrorAction SilentlyContinue
Remove-Item Env:\G1_ARMS_ANKLE_KD -ErrorAction SilentlyContinue
Remove-Item Env:\G1_BALANCE_FALLBACK -ErrorAction SilentlyContinue
if (Test-Path $pol) { $env:G1_POLICY_ONNX = $pol }
$env:G1_ARM_POSE       = "down"
$env:G1_DEPLOY_LOG     = $log
$env:G1_STANDWAVE_LOG  = $log
$env:OMNISIM_LOG_PATH  = "$root\_scratch\omnisim_log_arms_stand_parity.txt"

if ($Wave) {
    if (-not (Test-Path $npz)) { python -u "$root\projects\policies\research\shadowing\generate_g1_standwave.py" }
    python -u "$root\projects\policies\research\shadowing\ghost_to_replay_csv.py" --npz $npz --mjcf $mjcf --out $csv
    $env:G1_STANDWAVE_REF = $csv     # feedforward wave on the arms; legs balance via the policy
    $env:G1_GHOST_REPLAY  = $csv
    $env:G1_GHOST_ALPHA   = "0.55"
    $env:G1_STANDWAVE_LOOP = "1"
}
$coldFlag = if ($Warm) { "" } else { "--cold" }   # cold avoids the 23-DOF warm-reload crash

if ($Gui) {
    python -u "$root\scripts\dev\headless_runner.py" $worldDuo --gui --realtime --duration $Duration $coldFlag
} else {
    python -u "$root\scripts\dev\headless_runner.py" $worldSolo --duration $Duration $coldFlag 2>&1 | Select-Object -Last 6
}

Write-Host "`n===== PARITY STAND$(if($Wave){'+WAVE'}) DEPLOY RESULT ====="
if (Test-Path $log) {
    $fall = (Select-String -Path $log -Pattern "FALL" | Measure-Object).Count
    $last = Get-Content $log | Select-Object -Last 1
    if ($fall -eq 0) { Write-Host "  PASS - no fall. last: $last" }
    else             { Write-Host "  $fall FALL line(s). last: $last" }
} else {
    Write-Host "  (no deploy log produced)"
}
