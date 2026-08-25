# Launch the OmniArm 6 bin-picking + colour-sort demo in the OmniSim GUI.
# An OmniArm 6 with a real SUCTION (vacuum) cup empties a dense 36-cube bin top-down
# (grips each cube's top face, lifts straight up, sorts by colour) -- no fingers,
# no shaking. The cup is a tool link in omniarm6_suction.urdf; the held cube sits
# flush against the cup lip.
#   OMNISIM_NEWTON_STATICS=1   the bin + tray walls are real Newton static colliders
#   OMNISIM_WARMUP_TOKEN       per-launch token so the controller reloads once at
#                              startup to dodge the cold-first-load bug (the arm
#                              under-tracks targets on a fresh build). See
#                              docs/developer/real-grasp-and-the-cold-first-load-trap.md
#
# Usage:  powershell -ExecutionPolicy Bypass -File run_omniarm6_bin_picking.ps1
$root = (Resolve-Path "$PSScriptRoot\..\..\..\..\..").Path
$env:OMNISIM_HOME                      = $root
$env:OMNISIM_NEWTON_STATICS            = "1"   # static tray walls
$env:OMNISIM_NEWTON_COMPOUND_COLLIDERS = "1"   # the MOVABLE bin is one multi-box body
$env:OMNISIM_NEWTON_GROUND_MU          = "1.5"
$env:OMNISIM_WARMUP_TOKEN              = "gui$PID"
& "$root\msys64\mingw64\bin\omnisim-bin.exe" "$PSScriptRoot\omniarm6_bin_picking.wbt"
