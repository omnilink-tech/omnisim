# A/B COMPARE the two achievable sit->stand->sit stands, side by side, looped:
#   LEFT  (blue)  = STRAIGHT-LEG variant (v9): straight knees, ~33deg forward bow
#   RIGHT (green) = UPRIGHT-TORSO variant (v10): moderate knee, torso vertical
# Each ghost replays the robot's ACTUAL sim-achieved motion (its customData CSV).
#
# Usage: powershell -File scripts/dev/run_g1_sitstand_compare.ps1 [-Duration 60]
param([int]$Duration = 60)
$root = (Resolve-Path "$PSScriptRoot\..\..\..\..").Path
$world = "$root\projects\policies\research\worlds\g1_sitstand_compare.omniworld"

$env:OMNISIM_HOME     = $root
$env:G1_SITSTAND_LOOP  = "1"
$env:G1_GHOST_ALPHA    = "0.6"
$env:OMNISIM_LOG_PATH  = "$root\_scratch\omnisim_log_sitstand_compare.txt"
Remove-Item Env:\G1_GHOST_REPLAY -ErrorAction SilentlyContinue   # per-ghost via customData
Remove-Item Env:\G1_GHOST_TINT   -ErrorAction SilentlyContinue   # name-based tints

Write-Host "A/B compare -- LEFT(blue)=straight-leg  RIGHT(green)=upright-torso. Close the window to stop."
python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration
