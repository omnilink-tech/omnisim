# PREVIEW the BRACEGUARD defensive GHOST motion ALONE (no physics robot), looped, so we
# design + agree on the reference BEFORE training the mimic (the ghost-first rule).
#
# A single translucent kinematic G1 holds the secure guard stance and plays ORIENT-AND-BAR:
# it yaws the waist to face the incoming cube and throws both forearms into a two-hand bar
# across its torso front, then returns to guard -- on loop. Replays the certified ghost
# (projects/policies/research/shadowing/ghosts/g1_braceguard_replay.csv; regenerate with -Regen).
#
# Usage: powershell -File scripts/dev/run_g1_braceguard_ghost_preview.ps1 [-Duration 120] [-Fast] [-Regen]
param(
    [int]$Duration = 120,
    [switch]$Fast,
    [switch]$Regen
)
$root = (Resolve-Path "$PSScriptRoot\..\..\..\..").Path
$world = "$root\projects\policies\research\worlds\g1_braceguard_ghost_preview.omniworld"
$npz   = "$root\_scratch\g1_braceguard_ghost.npz"
$csv   = "$root\projects\policies\research\shadowing\ghosts\g1_braceguard_replay.csv"
$mjcf  = "$root\projects\robots\unitree\g1\urdf\g1_full_kp100.mjcf.xml"

$env:OMNISIM_HOME      = $root
$env:PYTHONUTF8        = "1"
$env:G1_GHOST_ALPHA    = "0.55"
$env:G1_GHOST_TINT     = "0.62,0.82,1.0"
$env:G1_STANDWAVE_LOOP = "1"
$env:OMNISIM_LOG_PATH  = "$root\_scratch\omnisim_log_braceguard_ghost_preview.txt"

if ($Regen) {
    Write-Host "Regenerating the braceguard ghost..."
    python -u "$root\projects\policies\research\shadowing\generate_g1_braceguard.py"
    python -u "$root\projects\policies\research\shadowing\ghost_to_replay_csv.py" --npz $npz --mjcf $mjcf --out $csv
}
if (-not (Test-Path $csv)) { Write-Error "no replay CSV at $csv (run with -Regen)"; exit 1 }
$env:G1_GHOST_REPLAY = $csv

if ($Fast) {
    Write-Host "BraceGuard ghost preview (GUI, unpaced) -- close the window to stop."
    python -u "$root\scripts\dev\headless_runner.py" $world --gui --duration $Duration
} else {
    Write-Host "BraceGuard ghost preview (GUI, real-time) -- watch the secure stance + two-hand bar loop."
    python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration
}
