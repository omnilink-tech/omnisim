# Deploy the shadowing pipeline END-TO-END on a CONTACT-HANDOFF Spot motion: Spot
# starts BELLY-FLAT on the floor and GETS UP, shadowing its MPPI-generated,
# verifier-certified get-up ghost, RL-stabilised (Component 3 in the loop).
#
# Pipeline: generate_spot_getup.py (MPPI C1) -> ghost_verifier (C2) ->
# gpu_mjwarp_spot_getup_trainer (C3, --track-ref) -> THIS deploy (Stage D).
#
# Usage: run_spot_getup_deploy.ps1 [-Duration 20] [-Gui] [-Loop] [-Policy <onnx>]
param(
    [int]$Duration = 20,
    [switch]$Gui,
    [switch]$Loop,
    [switch]$CpuEngine,
    [string]$Policy = ""
)
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
$world = "$root\projects\policies\research\worlds\spot_getup_deploy.wbt"
$log   = "$root\_scratch\spot_getup_deploy.log"
if ($Policy -eq "") {
    # The heavy-DR get-up policy (gpu_spot_getup_dr) that deploys in Newton: rises
    # belly-flat -> stable stand, no flip. Needs the teleport start (in the controller).
    $Policy = "$root\projects\policies\research\inference\policies\gpu_spot_getup_main\policy.onnx"
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
$env:OMNISIM_NEWTON_TARGET_KE    = "500"
$env:OMNISIM_NEWTON_TARGET_KD    = "60"
$env:OMNISIM_LOG_PATH            = "$root\_scratch\omnisim_log_spot_getup.txt"
$env:SPOT_GETUP_REF  = "$root\projects\policies\research\shadowing\ghosts\spot_getup_ghost.npz"
$env:SPOT_GETUP_LOG  = $log
$env:SPOT_GETUP_RES_SCALE = "0.15"
$env:SPOT_SETTLE_S   = "1.5"
if ($Loop) { $env:SPOT_GETUP_LOOP = "1" }
if (Test-Path $Policy) { $env:SPOT_GETUP_POLICY = $Policy; Write-Host "RL policy: $Policy" }
else { Write-Host "NO policy at $Policy -- FEEDFORWARD (expect a sag)" }

if ($Gui) {
    python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration 2>&1 | Select-Object -Last 3
} else {
    python -u "$root\scripts\dev\headless_runner.py" $world --duration $Duration 2>&1 | Select-Object -Last 3
}

Write-Host "`n===== SPOT GET-UP RESULT ====="
if (Test-Path $log) { Get-Content $log }
else { Write-Host "  (no log produced)" }
