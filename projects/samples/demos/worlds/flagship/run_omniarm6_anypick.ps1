# Launch the OmniArm 6 ANYPICK demo (mixed-shape bin picking + shape sort) in the
# OmniSim GUI. AnyPick generalises the cube-only suction pick to ARBITRARY
# parts: the bin holds cubes, capsule tubes and L-shaped tubes dumped together;
# the vacuum cup picks each part on a per-shape, orientation-aware suction
# anchor and sorts the parts by shape into three trays.
#   OMNISIM_NEWTON_STATICS=1            tray walls are real Newton static colliders
#   OMNISIM_NEWTON_COMPOUND_COLLIDERS=1 the L-tubes + the movable bin are
#                                       compound multi-box bodies
#   OMNISIM_WARMUP_TOKEN                per-launch token so the controller reloads
#                                       once at startup to dodge the cold-first-load
#                                       bug. See docs/developer/real-grasp-and-the-
#                                       cold-first-load-trap.md
#
# Usage:  powershell -File run_omniarm6_anypick.ps1
$root = (Resolve-Path "$PSScriptRoot\..\..\..\..\..").Path
$env:OMNISIM_HOME                      = $root
$env:OMNISIM_NEWTON_STATICS            = "1"
$env:OMNISIM_NEWTON_COMPOUND_COLLIDERS = "1"
$env:OMNISIM_NEWTON_GROUND_MU          = "1.5"
$env:OMNISIM_WARMUP_TOKEN              = "gui$PID"
& "$root\msys64\mingw64\bin\omnisim-bin.exe" "$PSScriptRoot\omniarm6_anypick.wbt"
