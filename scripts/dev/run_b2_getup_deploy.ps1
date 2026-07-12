# Deploy the shadowing pipeline END-TO-END on a CONTACT-HANDOFF B2 motion: B2
# starts BELLY-FLAT on the floor and GETS UP, shadowing its MPPI-generated,
# verifier-certified get-up ghost, RL-stabilised (Component 3 in the loop).
#
# Pipeline: generate_b2_getup.py (MPPI C1) -> ghost_verifier (C2) ->
# gpu_mjwarp_b2_getup_trainer (C3, --track-ref) -> THIS deploy (Stage D).
#
# Usage: run_b2_getup_deploy.ps1 [-Duration 20] [-Gui] [-Loop] [-Policy <onnx>]
param(
    [int]$Duration = 20,
    [switch]$Gui,
    [switch]$Loop,
    [switch]$CpuEngine,
    [string]$Policy = ""
)
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
$world = "$root\projects\policies\research\worlds\b2_getup_deploy.wbt"
$log   = "$root\_scratch\b2_getup_deploy.log"
if ($Policy -eq "") {
    # The heavy-DR get-up policy (gpu_b2_getup_dr) that deploys in Newton: rises
    # belly-flat -> stable stand, no flip. Needs the teleport start (in the controller).
    $Policy = "$root\projects\policies\research\inference\policies\gpu_b2_getup_main\policy.onnx"
}

$env:OMNISIM_HOME                = $root
$env:OMNISIM_NEWTON_FORCE_MUJOCO = "1"
$env:OMNISIM_REQUIRE_NEWTON  = "1"   # assert Newton engaged -- fail loud, no silent ODE fallback
$env:OMNISIM_REQUIRE_MUJOCO_SOLVER = "1"   # assert Newton MuJoCo solver (not XPBD) -- the trained engine
if ($CpuEngine) { Remove-Item Env:OMNISIM_NEWTON_MJWARP -ErrorAction SilentlyContinue }
else            { $env:OMNISIM_NEWTON_MJWARP = "1" }
$env:OMNISIM_URDF_USE_INERTIA    = "1"
$env:OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE = "1"
$env:OMNISIM_NEWTON_SEED_POSE    = "1"
$env:OMNISIM_NEWTON_GROUND_MU    = "2.0"
$env:OMNISIM_NEWTON_SUBSTEPS     = "8"
$env:OMNISIM_NEWTON_TARGET_KE    = "1400"
$env:OMNISIM_NEWTON_TARGET_KD    = "35"
$env:OMNISIM_LOG_PATH            = "$root\_scratch\omnisim_log_b2_getup.txt"
$env:B2_GETUP_REF  = "$root\projects\policies\research\shadowing\ghosts\b2_getup_ghost.npz"
$env:B2_GETUP_LOG  = $log
$env:B2_GETUP_RES_SCALE = "0.15"
$env:B2_SETTLE_S   = "1.5"
if ($Loop) { $env:B2_GETUP_LOOP = "1" }
if (Test-Path $Policy) { $env:B2_GETUP_POLICY = $Policy; Write-Host "RL policy: $Policy" }
else { Write-Host "NO policy at $Policy -- FEEDFORWARD (expect a sag)" }

if ($Gui) {
    python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration 2>&1 | Select-Object -Last 3
} else {
    python -u "$root\scripts\dev\headless_runner.py" $world --duration $Duration 2>&1 | Select-Object -Last 3
}

Write-Host "`n===== SPOT GET-UP RESULT ====="
if (Test-Path $log) { Get-Content $log }
else { Write-Host "  (no log produced)" }
