# Deploy the Spot FORWARD LEAP in OmniSim Newton -- the flight-phase shadowing test.
# Spot shadows its MPPI jump ghost (RL tracker) and leaps. The landing impact is the
# worst mujoco_warp->Newton gap; this is where we find the limit.
# Usage: run_spot_jump_deploy.ps1 [-Duration 12] [-Gui] [-Policy <onnx>]
param([int]$Duration = 12, [switch]$Gui, [switch]$CpuEngine, [string]$Policy = "")
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
$world = "$root\projects\policies\research\worlds\spot_jump_deploy.wbt"
$log   = "$root\_scratch\spot_jump_deploy.log"
if ($Policy -eq "") { $Policy = "$root\projects\policies\research\training\runs\gpu_spot_jump_a\policy.onnx" }

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
$env:OMNISIM_LOG_PATH            = "$root\_scratch\omnisim_log_spot_jump.txt"
# Reuse the generic reference-tracking controller (SPOT_GETUP_* knobs) with the JUMP ghost.
$env:SPOT_GETUP_REF  = "$root\projects\policies\research\shadowing\ghosts\spot_jump_ghost.npz"
$env:SPOT_GETUP_LOG  = $log
$env:SPOT_GETUP_RES_SCALE = "0.25"
$env:SPOT_SETTLE_S   = "0.6"
$env:SPOT_GETUP_HOLD_FADE_S = "0.8"
if (Test-Path $Policy) { $env:SPOT_GETUP_POLICY = $Policy; Write-Host "RL policy: $Policy" }
else { Write-Host "NO policy at $Policy -- FEEDFORWARD (the leap will likely fail open-loop)" }

if ($Gui) { python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration 2>&1 | Select-Object -Last 3 }
else      { python -u "$root\scripts\dev\headless_runner.py" $world --duration $Duration 2>&1 | Select-Object -Last 3 }

Write-Host "`n===== SPOT JUMP RESULT ====="
if (Test-Path $log) { Get-Content $log } else { Write-Host "  (no log)" }
