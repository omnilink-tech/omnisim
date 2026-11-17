# OMNIARM6 two-arm THROW & CATCH (Shadowing) demo launcher.
#   .\scripts\dev\run_omniarm6_throw_catch_demo.ps1            # GUI, real-time
#   .\scripts\dev\run_omniarm6_throw_catch_demo.ps1 -Headless  # headless
param(
    [int]$Duration = 600,
    [switch]$Headless
)
$ErrorActionPreference = "Stop"
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
$world = "$root\projects\samples\demos\worlds\flagship\omniarm6_throw_catch.omniworld"

$env:OMNISIM_HOME                      = $root
$env:OMNISIM_NEWTON_STATICS            = "1"
$env:OMNISIM_NEWTON_SUBSTEPS           = "4"
$env:OMNISIM_NEWTON_MJWARP             = "1"
$env:OMNISIM_NEWTON_COMPOUND_COLLIDERS = "1"
# Stiffen the Newton position servo so the thrower tracks the fast certified swing tightly.
# These are SIMULATION servo gains, not robot limits: the engine still clamps actuator torque to
# the URDF effort (194/102/34 N.m) and hard-clamps joint velocity to the URDF rated speed
# (3.1415/3.4906 rad/s) every step, so the swing stays datasheet-legal.
#
# OMNISIM_NEWTON_NO_EFFORT_LIMIT is deliberately NOT set -- it used to be, and it DELETED the
# arm's rated torque cap. See run_omniarm6_toss_demo.ps1 / omniarm6_toss_deploy.py: the real bug was
# the teleporting vacuum hold, now force-coupled. Do not re-add it.
$env:OMNISIM_NEWTON_TARGET_KE          = "8000"
$env:OMNISIM_NEWTON_TARGET_KD          = "30"
$env:OMNIARM6_TOSS_LEAD                   = "0.0"

if ($Headless) {
    python -u "$root\scripts\dev\headless_runner.py" $world --duration $Duration
} else {
    python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration
}
