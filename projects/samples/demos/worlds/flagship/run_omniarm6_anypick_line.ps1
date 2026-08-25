# Launch the OmniArm 6 ANYPICK LINE demo (L-conveyor feeds bins of mixed parts)
# in the OmniSim GUI. An L-shaped conveyor brings three bins -- each arriving
# in a different orientation -- to a pick station in front of the robot; the
# contact-honest AnyPick empties each bin (sorting cubes / tubes / L-tubes
# into three trays) and the emptied bin departs along the other leg of the L.
#   OMNISIM_NEWTON_STATICS=1            belts + tray walls are Newton static colliders
#   OMNISIM_NEWTON_COMPOUND_COLLIDERS=1 the bins + L-tubes are compound multi-box bodies
#   OMNISIM_WARMUP_TOKEN                per-launch token: the controller reloads once
#                                       at startup (cold-first-load dodge)
#
# Usage:  powershell -File run_omniarm6_anypick_line.ps1
$root = (Resolve-Path "$PSScriptRoot\..\..\..\..\..").Path
$env:OMNISIM_HOME                      = $root
$env:OMNISIM_NEWTON_STATICS            = "1"
$env:OMNISIM_NEWTON_COMPOUND_COLLIDERS = "1"
$env:OMNISIM_NEWTON_GROUND_MU          = "1.5"
$env:OMNISIM_NEWTON_ROLL_MU            = "0.01"  # rolling resistance: tubes STOP
$env:OMNISIM_WARMUP_TOKEN              = "gui$PID"
& "$root\msys64\mingw64\bin\omnisim-bin.exe" "$PSScriptRoot\omniarm6_anypick_line.wbt"
