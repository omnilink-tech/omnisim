# Deterministic WALK-DEPLOY: the PHYSICS humanoid tracks its verified-feasible
# walking SHADOW under OmniSim Newton with a stiff position-PD (no RL). This
# MEASURES the sim-to-deploy gap before any RL compute is spent: how well the
# real Newton articulation follows the shadow, and how long it stays upright.
#
#   run_humanoid_walk_deploy.ps1 -Robot h1 [-Duration 20] [-Ke 200] [-Vx 0.45] [-Gui]
#
# -Robot accepts h1 only. "valkyrie" was removed 2026-08-24: the robot package
# went in be41986f8 and with it ${Robot}_walk_deploy.omniworld, so the switch
# offered a value that could only ever fail at world-load.
param(
    [Parameter(Mandatory = $true)][ValidateSet("h1")][string]$Robot,
    [int]$Duration = 20,
    [switch]$Gui,
    [double]$Ke = 200,          # Newton position-PD stiffness (walk: softer than the stand hold)
    [double]$Kd = 12,
    [double]$Vx = -1,           # forward speed override (-1 = gait default)
    [double]$Freq = -1,         # cadence override
    [double]$Settle = 0.3,      # short settle = launch-OOD fix: keeps the launch
    #                             pitch in the trainer's band (long settle leans the
    #                             CoM-fwd squat to -0.2, OOD -> the policy can't engage)
    [double]$AnkBias = 0.0,     # backward ankle-pitch trim (neg = lean back; CoM centring)
    [string]$Onnx = "",         # trained policy .onnx -> RL deploy (else shadow-only)
    [switch]$PureRl,            # full-authority pure-RL (target=nominal+act_scale*act, no gait)
    [double]$ActScale = 1.0,    # pure-RL action scale (MUST equal the trainer's H1_ACT_SCALE)
    [int]$ObsHistory = 1,       # frame-stack K (MUST equal the trainer's --obs-history)
    [double]$ResScale = -1,     # residual scale (shadow+residual mode; -1 = controller default)
    [string]$ExtraLog = ""
)
$root  = (Resolve-Path "$PSScriptRoot\..\..").Path
$world = "$root\projects\policies\research\worlds\${Robot}_walk_deploy.omniworld"
$log   = if ($ExtraLog) { $ExtraLog } else { "$root\_scratch\walk\${Robot}_walk.log" }
New-Item -ItemType Directory -Force -Path (Split-Path $log) | Out-Null
Remove-Item $log -ErrorAction SilentlyContinue
if (-not (Test-Path $world)) { Write-Error "no world $world"; exit 1 }

$env:OMNISIM_HOME      = $root
$env:PYTHONUTF8        = "1"
# --- deploy-faithful Newton solver config (IDENTICAL to the proven stand) ---
$env:OMNISIM_NEWTON_STATICS      = "1"
$env:OMNISIM_NEWTON_SUBSTEPS     = "4"
$env:OMNISIM_NEWTON_FORCE_MUJOCO = "1"
$env:OMNISIM_NEWTON_MJWARP       = "1"
$env:OMNISIM_URDF_USE_INERTIA    = "1"
$env:OMNISIM_NEWTON_BASE_GUARD   = "1"
$env:OMNISIM_NEWTON_SEED_POSE    = "1"
$env:OMNISIM_NEWTON_SEED_REBUILD = "1"
$env:OMNISIM_NEWTON_TARGET_KE    = "$Ke"
$env:OMNISIM_NEWTON_TARGET_KD    = "$Kd"
$env:OMNISIM_NEWTON_GROUND_MU    = "2.0"
# --- walk controller config ---
$env:HUMANOID_WALK_ROBOT = $Robot
$env:HW_SETTLE_S         = "$Settle"
$env:HW_ANK_BIAS         = "$AnkBias"
if ($Vx -ge 0)   { $env:HW_VX = "$Vx" }   else { Remove-Item Env:HW_VX -ErrorAction SilentlyContinue }
if ($Freq -ge 0) { $env:HW_FREQ = "$Freq" } else { Remove-Item Env:HW_FREQ -ErrorAction SilentlyContinue }
if ($Onnx)     { $env:HUMANOID_WALK_ONNX = (Resolve-Path $Onnx).Path } else { Remove-Item Env:HUMANOID_WALK_ONNX -ErrorAction SilentlyContinue }
if ($PureRl)   { $env:HUMANOID_WALK_PURE_RL = "1"; $env:H1_ACT_SCALE = "$ActScale"; $env:HUMANOID_WALK_OBS_HISTORY = "$ObsHistory" } else { Remove-Item Env:HUMANOID_WALK_PURE_RL -ErrorAction SilentlyContinue }
if ($ResScale -ge 0) { $env:HUMANOID_WALK_RES_SCALE = "$ResScale" } else { Remove-Item Env:HUMANOID_WALK_RES_SCALE -ErrorAction SilentlyContinue }
$env:HUMANOID_WALK_LOG   = $log
$env:OMNISIM_DEPLOY_LOG  = $log
$env:OMNISIM_LOG_PATH    = "$root\_scratch\walk\omnisim_log_${Robot}_walk.txt"

Write-Host "=== $Robot WALK-DEPLOY (shadow tracking under Newton, KE=$Ke) -- measuring the sim-to-deploy gap ===" -ForegroundColor Cyan
if ($Gui) {
    python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration
} else {
    python -u "$root\scripts\dev\headless_runner.py" $world --duration $Duration 2>&1 | Select-Object -Last 6
}

Write-Host "`n===== $Robot WALK RESULT =====" -ForegroundColor Cyan
if (Test-Path $log) {
    $fall = Select-String -Path $log -Pattern "FALL@" | Select-Object -First 1
    $last = Get-Content $log | Select-Object -Last 1
    if ($fall) { Write-Host "  $($fall.Line.Trim())" -ForegroundColor Yellow }
    else       { Write-Host "  no fall logged" -ForegroundColor Green }
    Write-Host "  last: $last"
} else { Write-Host "  (no deploy log)" }
