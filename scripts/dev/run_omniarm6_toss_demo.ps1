# OMNIARM6 toss-to-place (Shadowing) demo launcher.
#   .\scripts\dev\run_omniarm6_toss_demo.ps1            # GUI, real-time
#   .\scripts\dev\run_omniarm6_toss_demo.ps1 -Headless  # headless, auto-quit after one throw
param(
    [int]$Duration = 600,
    [switch]$Headless
)
$ErrorActionPreference = "Stop"
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
$world = "$root\projects\samples\demos\worlds\flagship\omniarm6_toss_demo.omniworld"

$env:OMNISIM_HOME                      = $root
$env:OMNISIM_NEWTON_STATICS            = "1"
$env:OMNISIM_NEWTON_SUBSTEPS           = "4"
$env:OMNISIM_NEWTON_MJWARP             = "1"
$env:OMNISIM_NEWTON_COMPOUND_COLLIDERS = "1"
# Swing-tracking gains. These are SIMULATION servo gains (how hard the position controller
# chases its setpoint) -- they are NOT robot limits, and they do not let the arm exceed any.
# The engine still clamps actuator torque to the URDF effort (194/102/34 N.m) and hard-clamps
# joint velocity to the URDF rated speed (3.1415/3.4906 rad/s) on every step, so the swing is
# datasheet-legal either way; the stiffer servo just tracks the certified ghost more tightly
# (lands ~1 cm from the bin centre vs ~15 cm at the engine's default gains).
#
# NOTE: OMNISIM_NEWTON_NO_EFFORT_LIMIT is deliberately NOT set. It used to be -- it DELETED the
# arm's rated torque cap. That was a workaround for the real bug (the vacuum hold teleported the
# cube every tick, which forced a MuJoCo<->Newton resync that wiped the arm's joint velocity, so
# the arm needed ~2x rated torque to brute-force through it). The hold is now force-coupled
# (see omniarm6_toss_deploy.py) and the throw lands WITHIN the OMNIARM6's rated torque. Do not re-add it.
$env:OMNISIM_NEWTON_TARGET_KE          = "8000"
$env:OMNISIM_NEWTON_TARGET_KD          = "30"
$env:OMNIARM6_TOSS_LEAD                   = "0.0"
$env:OMNIARM6_TOSS_OUT                    = "$root\_toss_result.txt"

if ($Headless) {
    $env:OMNIARM6_TOSS_AUTOQUIT = "1"
    python -u "$root\scripts\dev\headless_runner.py" $world --duration $Duration
} else {
    $env:OMNIARM6_TOSS_AUTOQUIT = "0"
    python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration
}
Write-Host "`n--- result ---"
if (Test-Path "$root\_toss_result.txt") { Get-Content "$root\_toss_result.txt" }
