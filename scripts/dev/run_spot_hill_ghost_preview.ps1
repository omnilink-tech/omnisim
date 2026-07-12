# WATCH the Spot hill-walk GHOST in a real OmniSim window (ghost-first sign-off).
#
# A translucent, physics-free Spot hologram replays its generated up-over-down
# hill ghost (Shadowing Component 1) over the matching 15-deg hill: walk flat ->
# pitch up the ramp -> over the crest -> pitch down -> flat. No RL, no physics --
# this is the reference the tracker will shadow. Close the window to stop.
#
# Usage:  powershell -File scripts/dev/run_spot_hill_ghost_preview.ps1 [-Duration 60] [-Fast]
param(
    [int]$Duration = 60,
    [switch]$Fast
)
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
$world = "$root\projects\policies\research\worlds\spot_hill_ghost_preview.wbt"

$env:OMNISIM_HOME     = $root
$env:HILL_ROBOT       = "spot"
$env:HILL_GHOST_NPZ   = "$root\projects\policies\research\shadowing\ghosts\spot_hill_ghost.npz"
$env:HILL_GHOST_ALPHA = "0.55"
$env:HILL_GHOST_TINT  = "0.62,0.82,1.0"
$env:HILL_LOOP        = "1"

Write-Host "Opening spot_hill_ghost_preview.wbt: translucent Spot ghost walking up & over & down a 15-deg hill."
if ($Fast) { python -u "$root\scripts\dev\headless_runner.py" $world --gui --duration $Duration }
else       { python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration }
