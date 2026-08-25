# Deploy the shadowing pipeline END-TO-END on a NON-gait OmniQuad motion: the OmniQuad
# SHADOWS its MPPI-generated, verifier-certified crouch-recover ghost in OmniSim
# Newton. Feedforward by default (the ghost is feasible by construction); pass
# -Policy <onnx> to add the RL residual (Component 3).
#
# Pipeline: generate_omniquad_crouch.py (MPPI, Component 1) -> ghost_verifier (C2,
# certified PASS) -> THIS deploy (Stage D). The crouch ghost lives at
# projects/policies/research/shadowing/ghosts/omniquad_crouch_ghost.npz.
#
# Usage: run_omniquad_crouch_deploy.ps1 [-Duration 30] [-Loop] [-Gui] [-Policy <onnx>]
param(
    [int]$Duration = 30,
    [switch]$Loop,
    [switch]$Gui,
    [switch]$CpuEngine,
    [string]$Policy = ""
)
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
$world = "$root\projects\policies\research\worlds\omniquad_crouch_deploy.omniworld"
$log   = "$root\_scratch\omniquad_crouch_deploy.log"

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
# Match the ghost's dynamics (the MPPI generator used the deploy-matched kp=500/kv=60).
$env:OMNISIM_NEWTON_TARGET_KE    = "500"
$env:OMNISIM_NEWTON_TARGET_KD    = "60"
$env:OMNISIM_LOG_PATH            = "$root\_scratch\omnisim_log_omniquad_crouch.txt"
$env:OMNIQUAD_CROUCH_REF  = "$root\projects\policies\research\shadowing\ghosts\omniquad_crouch_ghost.npz"
$env:OMNIQUAD_CROUCH_LOG  = $log
$env:OMNIQUAD_SETTLE_S    = "1.0"
if ($Loop)   { $env:OMNIQUAD_CROUCH_LOOP = "1" } else { Remove-Item Env:OMNIQUAD_CROUCH_LOOP -ErrorAction SilentlyContinue }
if ($Policy -ne "") { $env:OMNIQUAD_CROUCH_POLICY = $Policy } else { Remove-Item Env:OMNIQUAD_CROUCH_POLICY -ErrorAction SilentlyContinue }

if ($Policy -ne "") { Write-Host "OmniQuad crouch deploy (RL residual: $Policy)" }
else                { Write-Host "OmniQuad crouch deploy (FEEDFORWARD -- shadowing the certified ghost)" }
if ($Gui) {
    python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration 2>&1 | Select-Object -Last 3
} else {
    python -u "$root\scripts\dev\headless_runner.py" $world --duration $Duration 2>&1 | Select-Object -Last 3
}

Write-Host "`n===== OMNIQUAD CROUCH RESULT ====="
if (Test-Path $log) {
    $fall = (Select-String -Path $log -Pattern "FALL" | Measure-Object).Count
    if ($fall -eq 0) { Write-Host "  NO FALL." } else { Write-Host "  FELL." }
    Get-Content $log
} else { Write-Host "  (no log produced)" }
