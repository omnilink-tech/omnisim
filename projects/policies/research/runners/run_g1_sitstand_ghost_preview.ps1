# PREVIEW the sit->stand->sit GHOST motion ALONE (no physics robot), looped, so we
# design + agree on the reference BEFORE building any mimic (the ghost-first rule).
#
# A single translucent kinematic G1 plays projects/policies/control/gait/g1_sitstand.py:
#   seated -> rise -> stand 5s -> sit back, repeating.
# Tweak g1_sitstand.py (SEATED/STANDING poses, timeline, Z/X heights) until the
# motion is what we want, then re-run this to re-check.
#
# Usage: powershell -File scripts/dev/run_g1_sitstand_ghost_preview.ps1 [-Duration 60] [-Fast]
param(
    [int]$Duration = 60,
    [switch]$Fast
)
$root = (Resolve-Path "$PSScriptRoot\..\..\..\..").Path
$world = "$root\projects\policies\research\worlds\g1_sitstand_ghost_preview.omniworld"

$env:OMNISIM_HOME     = $root
$env:G1_SITSTAND_LOOP  = "1"     # loop the motion so it can be watched repeatedly
$env:G1_GHOST_ALPHA    = "0.55"
$env:OMNISIM_LOG_PATH  = "$root\_scratch\omnisim_log_sitstand_ghost_preview.txt"
# idealized ghost (NO replay) -- this is the DESIGNED reference, not an achieved clip
Remove-Item Env:\G1_GHOST_REPLAY -ErrorAction SilentlyContinue

if ($Fast) {
    Write-Host "Ghost preview (GUI, unpaced) -- close the window to stop."
    python -u "$root\scripts\dev\headless_runner.py" $world --gui --duration $Duration
} else {
    Write-Host "Ghost preview (GUI, real-time) -- watch the sit->stand->sit reference loop."
    python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration
}
