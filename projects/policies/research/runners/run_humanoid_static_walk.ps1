# DETERMINISTIC QUASI-STATIC walk deploy for a humanoid -- the walking sibling of
# run_humanoid_stand_deploy.ps1. Same proven Newton solver config + stiff position
# hold; the controller (humanoid_static_walk) layers a slow, statically-stable gait
# on the squat nominal. NO RL. Every gait knob is a -param so the gait can be swept
# headless (forward distance vs no-fall) without editing code.
#
# Usage:
#   powershell -File scripts/dev/run_humanoid_static_walk.ps1 -Robot g1 -Gui
#   powershell -File scripts/dev/run_humanoid_static_walk.ps1 -Robot g1 -Duration 30 -Stride 0.18
param(
    [Parameter(Mandatory=$false)][ValidateSet("g1","h1","valkyrie")][string]$Robot = "g1",
    [int]$Duration = 30,
    [switch]$Gui,
    [double]$Ke = 400,         # Newton position-PD stiffness (spec default if unset)
    [double]$Kd = 60,
    # ── gait knobs (empty => controller/spec default) ──
    [string]$Period = "",      # full-stride period, s (slow => quasi-static)
    [string]$Stride = "",      # hip-pitch swing amplitude, rad (forward step size)
    [string]$Lift = "",        # swing-leg knee lift, rad
    [string]$Lateral = "",     # lateral weight-shift (ankle-roll), rad
    [string]$HipRoll = "",     # hip-roll share of the lateral shift, rad
    [string]$FwdLean = "",     # constant forward CoM bias, rad
    [string]$Duty = "",        # swing fraction of each leg's cycle
    [string]$LatLead = "",     # lateral lead vs swing mid (cycles)
    [double]$LatSign = 1,      # sign of the lateral lean (tune once per robot)
    [double]$StrideSign = 1,   # sign of forward progress (tune once per robot)
    [string]$RampS = "",       # ramp gait amplitudes 0->full over this many s
    [string]$StartS = "",      # stand still this long before walking
    [string]$AnkleFlat = "",   # ankle counter-rotation fraction (flat foot)
    [string]$Shuffle = "",     # 1 = no-lift double-support shuffle (default), 0 = lift gait
    [string]$KneeSlide = "",   # knee-extend per hip slide (keeps the foot on the ground)
    [string]$StanceWidth = "", # base hip-roll abduction, rad (widen the base)
    [string]$SquatExtra = "",  # extra knee flex, rad (lower the CoM)
    [double]$StopAt = 0,       # hold stand after this forward distance, m (0=off)
    [switch]$Lean,             # enable the stand's reactive fore/aft trim on top
    [string]$ExtraLog = ""
)
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
$world = "$root\projects\policies\worlds\${Robot}_static_walk.wbt"
$spec  = "$root\projects\policies\controllers\humanoid_stand_deploy\specs\${Robot}.json"
$log   = if ($ExtraLog) { $ExtraLog } else { "$root\_scratch\walk\${Robot}_static_walk.log" }
New-Item -ItemType Directory -Force -Path (Split-Path $log) | Out-Null
Remove-Item $log -ErrorAction SilentlyContinue

if (-not (Test-Path $world)) { Write-Error "no world $world"; exit 1 }
if (-not (Test-Path $spec))  { Write-Error "no spec $spec";  exit 1 }

# Per-robot deploy stiffness from the spec unless overridden.
$specObj = Get-Content $spec -Raw | ConvertFrom-Json
if (-not $PSBoundParameters.ContainsKey('Ke') -and $specObj.PSObject.Properties.Name -contains 'deploy_ke') { $Ke = [double]$specObj.deploy_ke }
if (-not $PSBoundParameters.ContainsKey('Kd') -and $specObj.PSObject.Properties.Name -contains 'deploy_kd') { $Kd = [double]$specObj.deploy_kd }

$env:OMNISIM_HOME      = $root
$env:PYTHONUTF8        = "1"
# --- deploy-faithful Newton solver config, IDENTICAL to the proven stand ---
$env:OMNISIM_NEWTON_STATICS      = "1"
$env:OMNISIM_NEWTON_SUBSTEPS     = "4"
$env:OMNISIM_NEWTON_FORCE_MUJOCO = "1"
$env:OMNISIM_NEWTON_MJWARP       = "1"
$env:OMNISIM_URDF_USE_INERTIA    = "1"
$env:OMNISIM_NEWTON_BASE_GUARD   = "1"
$env:OMNISIM_NEWTON_SEED_POSE    = "1"
$env:OMNISIM_NEWTON_SEED_REBUILD = "1"
# Fail loudly instead of silently falling back to ODE: an ODE fallback runs the
# Newton-tuned position-PD gains under the wrong solver and the articulation
# numerically EXPLODES (bz -> millions), producing garbage that looks like a
# physics result. Assert Newton actually drove the world.
$env:OMNISIM_REQUIRE_NEWTON      = "1"
$env:OMNISIM_NEWTON_TARGET_KE    = "$Ke"
$env:OMNISIM_NEWTON_TARGET_KD    = "$Kd"
$env:OMNISIM_NEWTON_GROUND_MU    = "2.0"
# --- controller config ---
$env:HUMANOID_STAND_SPEC = $spec
$env:HUMANOID_STAND_LOG  = $log
$env:OMNISIM_DEPLOY_LOG  = $log
$env:OMNISIM_LOG_PATH    = "$root\_scratch\walk\omnisim_log_${Robot}_static_walk.txt"
$env:WALK_LAT_SIGN       = "$LatSign"
$env:WALK_STRIDE_SIGN    = "$StrideSign"
$env:WALK_STOP_AT        = "$StopAt"
$env:WALK_LEAN           = if ($Lean) { "1" } else { "0" }
# Optional gait overrides (only set when the caller passes a value).
function SetEnvIf($envName, $val) { if ($val -ne "") { Set-Item -Path "Env:\$envName" -Value $val } else { Remove-Item -Path "Env:\$envName" -ErrorAction SilentlyContinue } }
SetEnvIf "WALK_PERIOD"     $Period
SetEnvIf "WALK_STRIDE"     $Stride
SetEnvIf "WALK_LIFT"       $Lift
SetEnvIf "WALK_LATERAL"    $Lateral
SetEnvIf "WALK_HIP_ROLL"   $HipRoll
SetEnvIf "WALK_FWD_LEAN"   $FwdLean
SetEnvIf "WALK_DUTY"       $Duty
SetEnvIf "WALK_LAT_LEAD"   $LatLead
SetEnvIf "WALK_RAMP_S"     $RampS
SetEnvIf "WALK_START_S"    $StartS
SetEnvIf "WALK_ANKLE_FLAT" $AnkleFlat
SetEnvIf "WALK_SHUFFLE"      $Shuffle
SetEnvIf "WALK_KNEE_SLIDE"   $KneeSlide
SetEnvIf "WALK_STANCE_WIDTH" $StanceWidth
SetEnvIf "WALK_SQUAT_EXTRA"  $SquatExtra

Write-Host "=== $Robot deterministic quasi-static walk (ke=$Ke kd=$Kd) ===" -ForegroundColor Cyan
if ($Gui) {
    python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration
} else {
    python -u "$root\scripts\dev\headless_runner.py" $world --duration $Duration 2>&1 | Select-Object -Last 16
}

Write-Host "`n===== $Robot STATIC-WALK RESULT =====" -ForegroundColor Cyan
if (Test-Path $log) {
    $fall = (Select-String -Path $log -Pattern "FALL" | Measure-Object).Count
    $last = Get-Content $log | Select-Object -Last 1
    if ($fall -eq 0) { Write-Host "  PASS - no fall. last: $last" -ForegroundColor Green }
    else             { Write-Host "  $fall FALL line(s). last: $last" -ForegroundColor Yellow }
} else {
    Write-Host "  (no deploy log produced)"
}
