# Watch UNITREE'S OFFICIAL H1 walk policy stride in OmniSim's renderer.
#
# Runs unitree_rl_gym's pre_train/h1 motion.pt on Unitree's faithful 10-DOF legs
# model (h1_legs_clean.urdf -- 51.65 kg, full upper-body visuals on the pelvis,
# box feet) through OmniSim's mujoco_warp Newton engine. Verified: the H1 walks
# 0 -> 25 m+ dead-straight, never falls, at ~0.41 m/s, z steady ~1.035 m.
#
#   powershell -ExecutionPolicy Bypass -File projects\policies\worlds\run_h1_unitree_walk.ps1
#
# Same recipe as the G1 (run_g1_unitree_walk.ps1) -- only the model, gains, obs
# width (41), default pose, and box feet differ.

$ErrorActionPreference = "Stop"
$Repo = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $PSScriptRoot))
Set-Location $Repo

$env:OMNISIM_HOME                     = $Repo
$env:OMNISIM_NEWTON_FORCE_MUJOCO      = "1"
$env:OMNISIM_NEWTON_MJWARP            = "1"
$env:OMNISIM_NEWTON_SUBSTEPS          = "2"   # EVEN -> CUDA-graph replay -> ~1.1x realtime (odd=1 ran 0.3x)
$env:OMNISIM_NEWTON_STATICS           = "1"
$env:OMNISIM_NEWTON_COMPOUND_COLLIDERS = "1"
$env:OMNISIM_URDF_USE_INERTIA         = "1"
$env:OMNISIM_NEWTON_USE_LINK_COM      = "1"
$env:OMNISIM_NEWTON_TARGET_KE         = "0"
$env:OMNISIM_NEWTON_TARGET_KD         = "0"
$env:OMNISIM_NEWTON_GROUND_MU         = "1.0"
$env:H1_CMD_VX                        = "0.5"
# wgpu (Vulkan) main view: WREN/OpenGL contends with mujoco_warp's CUDA on the one
# GPU and halves throughput (~0.4x live). Vulkan coexists cleanly -> ~1.1x, so
# realtime mode below plays it smoothly at 1x. THIS is what makes the GUI realtime.
$env:OMNISIM_WGPU_MAINVIEW_FORCE      = "1"

$bin = Join-Path $Repo "msys64\mingw64\bin\omnisim-bin.exe"
# fast mode + wgpu renders ~1.1x realtime (smooth). NOTE: --mode=realtime currently
# stalls to ~0.1x with the wgpu present path (known wgpu app-loop pacing gap), so use
# fast here -- it plays at roughly real speed instead of throttling.
& $bin "projects\policies\research\worlds\h1_unitree_deploy.wbt" --mode=fast $args
