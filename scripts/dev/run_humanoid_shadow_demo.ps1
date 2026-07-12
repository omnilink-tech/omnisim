# WATCH the designed SHADOW walk in 3D in OmniSim -- a single kinematic,
# physics-free humanoid (H1 or Valkyrie) self-walking across the floor playing
# the pure human-gait MODEL. This is the "ghost first" preview: judge whether
# the motion is natural + stable BEFORE any RL training. It cannot fall (no
# physics), so what you see is purely the reference the policy will track.
#
#   run_humanoid_shadow_demo.ps1 -Robot h1 [-Duration 60] [-Alpha 0.4] [-Fast]
#     -Robot  h1 | valkyrie
#     -Alpha  hologram transparency (0 = solid, 0.45 = faint; default 0.35)
#     -Fast   unpaced (default: real-time 1:1)
param(
    [Parameter(Mandatory = $true)][ValidateSet("h1", "valkyrie")][string]$Robot,
    [int]$Duration = 60,
    [double]$Alpha = 0.35,
    [switch]$Fast
)
$root  = (Resolve-Path "$PSScriptRoot\..\..").Path
$world = "$root\projects\policies\worlds\${Robot}_shadow_demo.wbt"
if (-not (Test-Path $world)) { Write-Error "no world $world"; exit 1 }

$env:OMNISIM_HOME          = $root
$env:PYTHONUTF8            = "1"
$env:HUMANOID_GHOST_ROBOT  = $Robot           # selects gait module + joint map
$env:HUMANOID_GHOST_ALPHA  = "$Alpha"         # hologram look (0 = solid)
$env:HUMANOID_GHOST_Y      = "0"              # centred
$env:OMNISIM_LOG_PATH      = "$root\_scratch\omnisim_log_${Robot}_shadow.txt"
$env:HUMANOID_GHOST_LOG    = "$root\_scratch\${Robot}_shadow_ghost.log"
# GaitParams default to the robot's own (size-tuned) gait module; override here
# if needed, e.g. $env:HG_VX = "0.5"; $env:HG_FREQ = "1.1".

New-Item -ItemType Directory -Force -Path "$root\_scratch" | Out-Null
Write-Host "=== $Robot SHADOW demo: the pure gait model walking in 3D (no physics, no RL) ==="
Write-Host "Watch: natural stride, feet landing flat, weight rocking onto the stance foot. Close the window to stop."
if ($Fast) { python -u "$root\scripts\dev\headless_runner.py" $world --gui --duration $Duration }
else       { python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration }
