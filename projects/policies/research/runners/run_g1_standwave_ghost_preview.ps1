# PREVIEW the stand-and-wave GHOST motion ALONE (no physics robot), looped, so we design +
# agree on the reference BEFORE building the mimic (the ghost-first rule).
#
# A single translucent kinematic G1 stands, balances, and waves its right hand, replaying the
# generated ghost (projects/policies/research/shadowing/generate_g1_standwave.py). Regenerate the ghost +
# the replay CSV, then re-run this to re-check the motion.
#
# Usage: powershell -File scripts/dev/run_g1_standwave_ghost_preview.ps1 [-Duration 60] [-Fast] [-Regen]
param(
    [int]$Duration = 60,
    [switch]$Fast,
    [switch]$Regen
)
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
$world = "$root\projects\policies\worlds\g1_standwave_ghost_preview.wbt"
$npz   = "$root\_scratch\g1_standwave_ghost_generated.npz"
$csv   = "$root\_scratch\g1_standwave_replay.csv"
$mjcf  = "$root\projects\robots\unitree\g1\urdf\g1_full_kp100.mjcf.xml"

$env:OMNISIM_HOME      = $root
$env:PYTHONUTF8        = "1"
$env:G1_GHOST_ALPHA    = "0.55"
$env:G1_STANDWAVE_LOOP = "1"
$env:OMNISIM_LOG_PATH  = "$root\_scratch\omnisim_log_standwave_ghost_preview.txt"

if ($Regen -or -not (Test-Path $npz)) {
    Write-Host "Generating the stand+wave ghost..."
    python -u "$root\projects\policies\research\shadowing\generate_g1_standwave.py"
}
Write-Host "Converting ghost -> replay CSV..."
python -u "$root\projects\policies\research\shadowing\ghost_to_replay_csv.py" --npz $npz --mjcf $mjcf --out $csv
$env:G1_GHOST_REPLAY = $csv

if ($Fast) {
    Write-Host "Ghost preview (GUI, unpaced) -- close the window to stop."
    python -u "$root\scripts\dev\headless_runner.py" $world --gui --duration $Duration
} else {
    Write-Host "Ghost preview (GUI, real-time) -- watch the stand + wave reference loop."
    python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration
}
