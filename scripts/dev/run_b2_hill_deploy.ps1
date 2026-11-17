# Run the Unitree B2 HILL-WALK deploy world: the PHYSICALLY-SIMULATED B2 (Newton)
# shadows its hill ghost up a 15-deg slope, over the crest, and down -- beside the
# translucent ideal ghost. Same recipe as run_b2_walk_deploy.ps1 (trot model + RL
# residual on the stiffness-matched MJCF) plus:
#   * OMNISIM_NEWTON_STATICS=1  -> the hill boxes are COLLIDABLE static colliders
#   * B2_HILL_GHOST=<npz>       -> the slope-conditioned obs (target pitch + height)
#   * gait vx=0.35              -> the hill ghost's climb speed
#
# Usage:  powershell -File scripts/dev/run_b2_hill_deploy.ps1 [-Duration 60] [-Policy <onnx>] [-Gui] [-Realtime] [-Bare]
param(
    [int]$Duration = 60,
    [string]$Policy = "",
    [switch]$Bare,
    [switch]$Gui,
    [switch]$Realtime
)
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
$world = "$root\projects\policies\research\worlds\b2_hill_deploy.omniworld"
$ghost = "$root\projects\policies\research\shadowing\ghosts\b2_hill_ghost.npz"
if ($Policy -eq "") { $Policy = "$root\projects\policies\research\training\runs\gpu_b2_hill_h100_15\policy.onnx" }
$log = "$root\_scratch\b2_hill_deploy.log"
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
$env:OMNISIM_NEWTON_TARGET_KE    = "1400"  # MUST match the trainer MJCF
$env:OMNISIM_NEWTON_TARGET_KD    = "35"
if ($Bare) { $env:B2_POLICY_ONNX = "$root\__no_policy__" } else { $env:B2_POLICY_ONNX = $Policy }
$env:B2_HILL_GHOST  = $ghost               # enables the slope-conditioned obs (+2)
$env:B2_HILL_LOOKAHEAD = "0.4"             # MUST match training --hill-lookahead
# CRAWL gait config -- MUST match the hill training run (statically stable on slopes).
$env:B2_GAIT_MODULE = "projects.policies.control.gait.b2_crawl_gait"
$env:B2_GAIT_VX     = "0.35"
$env:B2_GAIT_FREQ   = "1.0"
$env:B2_GAIT_DUTY   = "0.85"
$env:B2_GAIT_STEP_H = "0.10"
$env:B2_GAIT_BODY_H = "0.50"
$env:B2_GAIT_RAMP_S = "1.0"
$env:B2_ACT_SCALE   = "0.18"
$env:B2_DEPLOY_LOG  = $log
# the translucent ghost alongside (kinematic replay)
$env:HILL_ROBOT     = "b2"
$env:HILL_GHOST_NPZ = $ghost
$env:HILL_GHOST_ALPHA = "0.5"

$guiArg = ""; if ($Gui) { $guiArg = "--gui" }
$rtArg = "";  if ($Realtime) { $rtArg = "--realtime" }
Write-Host "Running b2_hill_deploy.wbt for ${Duration}s ($(if($Bare){'BARE gait'}else{'trot + RL hill policy'}))..."
python -u "$root\scripts\dev\headless_runner.py" $world --duration $Duration $guiArg $rtArg 2>&1 | Select-Object -Last 4

Write-Host "`n===== B2 HILL-WALK RESULT ====="
if (Test-Path $log) {
    $fall = (Select-String -Path $log -Pattern "FALL" | Measure-Object).Count
    $last = Get-Content $log | Where-Object { $_ -match "^\[t=" } | Select-Object -Last 1
    if ($fall -eq 0) { Write-Host "  NO FALL. last: $last" }
    else { Write-Host "  FELL: $((Select-String -Path $log -Pattern 'FALL' | Select-Object -First 1).Line)" }
} else { Write-Host "  (no deploy log produced)" }
