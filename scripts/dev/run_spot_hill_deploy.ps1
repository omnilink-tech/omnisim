# Run the Spot HILL-WALK deploy world: the physically-simulated Spot (Newton)
# shadows its hill ghost up a 15-deg slope, over the crest, and down -- with the
# statically-stable CRAWL gait. Mirrors run_b2_hill_deploy.ps1 (Spot gains/gait).
#
# Usage:  powershell -File scripts/dev/run_spot_hill_deploy.ps1 [-Duration 60] [-Policy <onnx>] [-Gui] [-Realtime] [-Bare]
param(
    [int]$Duration = 60,
    [string]$Policy = "",
    [switch]$Bare,
    [switch]$Gui,
    [switch]$Realtime
)
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
$world = "$root\projects\policies\research\worlds\spot_hill_deploy.wbt"
$ghost = "$root\projects\policies\research\shadowing\ghosts\spot_hill_ghost.npz"
if ($Policy -eq "") { $Policy = "$root\projects\policies\research\training\runs\gpu_spot_hill_h100_15\policy.onnx" }
$log = "$root\_scratch\spot_hill_deploy.log"
Remove-Item $log -ErrorAction SilentlyContinue

$env:OMNISIM_HOME                = $root
$env:OMNISIM_NEWTON_FORCE_MUJOCO = "1"
$env:OMNISIM_REQUIRE_NEWTON  = "1"   # assert Newton engaged -- fail loud, no silent ODE fallback
$env:OMNISIM_REQUIRE_MUJOCO_SOLVER = "1"   # assert Newton MuJoCo solver (not XPBD) -- the trained engine
$env:OMNISIM_NEWTON_MJWARP       = "1"
$env:OMNISIM_NEWTON_STATICS      = "1"     # collidable hill boxes
$env:OMNISIM_URDF_USE_INERTIA    = "1"
$env:OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE = "1"
$env:OMNISIM_NEWTON_SEED_POSE    = "1"
$env:OMNISIM_NEWTON_GROUND_MU    = "2.0"
$env:OMNISIM_NEWTON_SUBSTEPS     = "8"
$env:OMNISIM_NEWTON_TARGET_KE    = "500"   # MUST match the trainer MJCF (spot)
$env:OMNISIM_NEWTON_TARGET_KD    = "60"
if ($Bare) { $env:SPOT_POLICY_ONNX = "$root\__no_policy__" } else { $env:SPOT_POLICY_ONNX = $Policy }
$env:SPOT_HILL_GHOST    = $ghost           # slope-conditioned obs (+2)
$env:SPOT_HILL_LOOKAHEAD = "0.4"           # MUST match training --hill-lookahead
# CRAWL gait config -- MUST match the hill training run.
$env:SPOT_GAIT_MODULE = "projects.policies.control.gait.spot_crawl_gait"
$env:SPOT_GAIT_VX     = "0.35"
$env:SPOT_GAIT_FREQ   = "1.2"
$env:SPOT_GAIT_DUTY   = "0.85"
$env:SPOT_GAIT_STEP_H = "0.06"
$env:SPOT_GAIT_BODY_H = "0.55"
$env:SPOT_GAIT_RAMP_S = "1.0"
$env:SPOT_ACT_SCALE   = "0.18"
$env:SPOT_DEPLOY_LOG  = $log
$env:HILL_ROBOT     = "spot"
$env:HILL_GHOST_NPZ = $ghost
$env:HILL_GHOST_ALPHA = "0.5"

$guiArg = ""; if ($Gui) { $guiArg = "--gui" }
$rtArg = "";  if ($Realtime) { $rtArg = "--realtime" }
Write-Host "Running spot_hill_deploy.wbt for ${Duration}s ($(if($Bare){'BARE crawl'}else{'crawl + RL hill policy'}))..."
python -u "$root\scripts\dev\headless_runner.py" $world --duration $Duration $guiArg $rtArg 2>&1 | Select-Object -Last 4

Write-Host "`n===== SPOT HILL-WALK RESULT ====="
if (Test-Path $log) {
    $fall = (Select-String -Path $log -Pattern "FALL" | Measure-Object).Count
    $last = Get-Content $log | Where-Object { $_ -match "^\[t=" } | Select-Object -Last 1
    if ($fall -eq 0) { Write-Host "  NO FALL. last: $last" }
    else { Write-Host "  FELL: $((Select-String -Path $log -Pattern 'FALL' | Select-Object -First 1).Line)" }
} else { Write-Host "  (no deploy log produced)" }
