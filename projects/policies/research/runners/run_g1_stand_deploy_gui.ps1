# Open the G1 standing deploy world in the OmniSim GUI so you can WATCH it stand
# live. Same config as run_g1_stand_deploy.ps1 (the headless version), but opens a
# real window instead of running headless.
#
# G1 stands here with ZERO policy (the deeper-squat NOMINAL is statically stable;
# see run_g1_stand_deploy.ps1 header for the two root causes that were fixed:
# forward CoM at the old pose + the destabilising analytic ankle PD). If you open
# g1_stand_deploy.wbt by double-clicking it FALLS -- the stand needs the stiff
# joints + spawn-seed set below, which are launch env vars not yet baked into the
# world. This script sets them, then launches the GUI.
#
# Usage (from a normal PowerShell):
#   powershell -ExecutionPolicy Bypass -File scripts/dev/run_g1_stand_deploy_gui.ps1
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
$bin = "$root\msys64\mingw64\bin\omnisim-bin.exe"
$world = "$root\projects\policies\worlds\g1_stand_deploy.wbt"

$env:OMNISIM_HOME             = $root
$env:WEBOTS_HOME              = $root
$env:PATH                     = "$root\msys64\mingw64\bin;" + $env:PATH
# Ground for the forced-MuJoCo model + 250 Hz physics:
$env:OMNISIM_NEWTON_STATICS   = "1"
$env:OMNISIM_NEWTON_SUBSTEPS  = "4"
$env:OMNISIM_URDF_USE_INERTIA = "1"
$env:OMNISIM_NEWTON_BASE_GUARD= "1"
# newtonSolver "mujoco" is baked in the world now, but keep FORCE_MUJOCO for older binaries:
$env:OMNISIM_NEWTON_FORCE_MUJOCO = "1"
$env:OMNISIM_REQUIRE_NEWTON  = "1"   # assert Newton engaged -- fail loud, no silent ODE fallback
$env:OMNISIM_REQUIRE_MUJOCO_SOLVER = "1"   # assert Newton MuJoCo solver (not XPBD) -- the trained engine
$env:OMNISIM_NEWTON_MJWARP    = "1"
# spawn SEEDED in the deeper-squat NOMINAL (no straight->squat fold):
$env:OMNISIM_NEWTON_SEED_POSE    = "1"
$env:OMNISIM_NEWTON_SEED_REBUILD = "1"
# stiff position PD:
$env:OMNISIM_NEWTON_TARGET_KE = "400"
$env:OMNISIM_NEWTON_TARGET_KD = "60"
$env:OMNISIM_NEWTON_GROUND_MU = "2.0"
# Pure-NOMINAL stand (analytic ankle PD is OFF by default -- it destabilised roll).
# Set G1_POLICY_ONNX=...runs/gpu_g1_stand_v7/policy.onnx to add the RL residual.
$env:G1_BALANCE_FALLBACK      = "1"

Write-Host "Opening g1_stand_deploy.wbt in the GUI (ke=400, deeper-squat NOMINAL, seeded, MuJoCo)..."
Write-Host "G1 should settle briefly then stand indefinitely. Close the window to stop."
# Full window (NOT --batch / --no-rendering / --minimize) so you can watch the 3D view.
& $bin $world --mode=fast --stdout --stderr
