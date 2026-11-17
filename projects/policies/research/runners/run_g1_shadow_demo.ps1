# WATCH the improved SHADOW walk in 3D in OmniSim -- a single kinematic,
# physics-free G1 self-walking across the floor playing the pure gait MODEL.
# Pick which shadow upgrade to view; the difference is the FRONTAL/TRANSVERSE
# plane (hip-roll weight transfer + hip-yaw), so watch the legs rock/turn.
#
#   run_g1_shadow_demo.ps1 [-Mode C] [-Duration 60] [-Alpha 0.4] [-Fast]
#     -Mode  legacy | A | B | C   (default C = human 3D kinematics)
#            legacy = sway only (the old flat shadow)
#            A      = LIPM lateral weight transfer + flat-foot ankle-roll
#            B      = achieved (our 297m walk, feasible -- shows the splay)
#            C      = human 3D normative kinematics (hip-roll + hip-yaw)
param(
    [ValidateSet("legacy", "A", "B", "C")][string]$Mode = "C",
    [int]$Duration = 60,
    [double]$Alpha = 0.4,
    [switch]$Fast
)
$root  = (Resolve-Path "$PSScriptRoot\..\..\..\..").Path
$world = "$root\projects\policies\research\worlds\g1_shadow_demo.omniworld"

$env:OMNISIM_HOME = $root
# Shared gait params (match the deployed winter shadow).
$env:G1_GAIT_VX   = "0.4"
$env:G1_GAIT_FREQ = "1.3"
$env:G1_GAIT_RAMP_S = "2.0"
$env:G1_GAIT_A_ARM  = "0.25"
$env:G1_GAIT_HIP_SCALE = "0.9"
$env:G1_GAIT_A_LAT  = "0.05"
$env:G1_GAIT_STYLE  = "winter"
$env:G1_GAIT_LATERAL = "sway"
$env:G1_GAIT_YAW     = "none"
Remove-Item Env:G1_GAIT_LAT_HIP_AMP -ErrorAction SilentlyContinue
Remove-Item Env:G1_GAIT_STEP_WIDTH  -ErrorAction SilentlyContinue

switch ($Mode) {
    "legacy" { }                                          # sway / none defaults
    "A"      { $env:G1_GAIT_LATERAL = "lipm" }
    "B"      { $env:G1_GAIT_STYLE   = "achieved" }
    "C"      { $env:G1_GAIT_LATERAL = "human"; $env:G1_GAIT_YAW = "human" }
}

$env:G1_GHOST_SELF_WALK = "1"     # walk on its own clock (no real robot)
$env:G1_GHOST_Y     = "0"         # centred
$env:G1_GHOST_ALPHA = "$Alpha"    # 0 = solid, 0.6 = faint hologram
$env:OMNISIM_LOG_PATH = "$root\_scratch\omnisim_log_shadow_demo.txt"
$env:G1_GHOST_LOG     = "$root\_scratch\g1_shadow_ghost.log"

Write-Host "Shadow demo: MODE=$Mode  (legacy/A/B/C). The pure gait model walking in 3D."
Write-Host "Watch the FRONTAL plane -- hip-roll weight transfer (+hip-yaw for C). Close the window to stop."
if ($Fast) { python -u "$root\scripts\dev\headless_runner.py" $world --gui --duration $Duration }
else       { python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration }
