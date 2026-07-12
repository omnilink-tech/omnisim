# DEPLOY the stand+wave SHADOWER: the real, Newton-simulated G1 stands, balances, and waves
# its right hand -- shadowing the translucent ghost beside it. Legs hold the statically-stable
# stand; arms track the wave reference (the SAME ghost replay CSV). The base balances itself
# (the stable stance absorbs the light arm wave) -- no balance policy required.
#
# Usage: powershell -File scripts/dev/run_g1_standwave_deploy.ps1 [-Duration 60] [-Fast] [-Regen]
param(
    [int]$Duration = 60,
    [switch]$Fast,
    [switch]$Regen
)
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
$world = "$root\projects\policies\worlds\g1_standwave_deploy.wbt"
$npz   = "$root\_scratch\g1_standwave_ghost_generated.npz"
$csv   = "$root\_scratch\g1_standwave_replay.csv"
$mjcf  = "$root\projects\robots\unitree\g1\urdf\g1_full_kp100.mjcf.xml"
$log   = "$root\_scratch\g1_standwave_deploy.log"
Remove-Item $log -ErrorAction SilentlyContinue

$env:OMNISIM_HOME      = $root
$env:PYTHONUTF8        = "1"
# --- deploy-faithful Newton solver config (legged humanoid, stiff position PD) ---
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
$env:OMNISIM_NEWTON_TARGET_KE    = "100"    # MATCH the trainer g1_full_kp100.mjcf kp=100
$env:OMNISIM_NEWTON_TARGET_KD    = "5"       # MATCH kv=5 (deploy==train actuators -> policy transfers)
$env:OMNISIM_NEWTON_GROUND_MU    = "2.0"
# --- the stand+wave balance policy (legs balance THROUGH the wave; trained on g1_full_kp100) ---
$swpol = "$root\projects\policies\research\training\runs\gpu_g1_standwave_kp100\policy.onnx"
if (Test-Path $swpol) { $env:G1_POLICY_ONNX = $swpol; Write-Host "Policy: $swpol" }
else { Write-Host "WARN: no gpu_g1_standwave policy -- controller falls back to gpu_g1_arms_stand_robust (may roll)" }
# --- ghost (translucent) + the shared reference CSV both controllers read ---
$env:G1_GHOST_ALPHA    = "0.55"
$env:G1_STANDWAVE_LOOP = "1"
$env:G1_STANDWAVE_LOG  = $log
$env:OMNISIM_LOG_PATH  = "$root\_scratch\omnisim_log_standwave_deploy.txt"

if ($Regen -or -not (Test-Path $npz)) {
    Write-Host "Generating the stand+wave ghost..."
    python -u "$root\projects\policies\research\shadowing\generate_g1_standwave.py"
}
Write-Host "Converting ghost -> replay CSV..."
python -u "$root\projects\policies\research\shadowing\ghost_to_replay_csv.py" --npz $npz --mjcf $mjcf --out $csv
$env:G1_GHOST_REPLAY   = $csv     # the translucent ghost replays this
$env:G1_STANDWAVE_REF  = $csv     # the real robot shadows the same reference

if ($Fast) {
    Write-Host "Deploy (GUI, unpaced) -- close the window to stop."
    python -u "$root\scripts\dev\headless_runner.py" $world --gui --duration $Duration
} else {
    Write-Host "Deploy (GUI, real-time) -- real robot shadows the ghost (stand + wave)."
    python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration
}

Write-Host "`n===== STAND+WAVE DEPLOY RESULT ====="
if (Test-Path $log) {
    $fall = (Select-String -Path $log -Pattern "FALL" | Measure-Object).Count
    $last = Get-Content $log | Select-Object -Last 1
    if ($fall -eq 0) { Write-Host "  PASS - no fall. last: $last" }
    else             { Write-Host "  $fall FALL line(s). last: $last" }
} else {
    Write-Host "  (no deploy log produced)"
}
