# Watch UNITREE'S OFFICIAL G1 walk policy stride in OmniSim's renderer.
#
# Runs unitree_rl_gym's pre_train/g1 motion.pt on Unitree's faithful 12-DOF
# model (4-sphere-per-foot contact) through OmniSim's mujoco_warp Newton engine.
# Verified: the G1 walks 0 -> 20 m upright, never falls, at ~0.45 m/s.
#
#   powershell -ExecutionPolicy Bypass -File projects\policies\worlds\run_g1_unitree_walk.ps1
#
# The two physics fields it needs (newtonStatics, newtonCompoundColliders) are
# baked into the .wbt; the rest are model-fidelity launch knobs set below.

$ErrorActionPreference = "Stop"
$Repo = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $PSScriptRoot))
Set-Location $Repo

$env:OMNISIM_HOME                     = $Repo
$env:OMNISIM_NEWTON_FORCE_MUJOCO      = "1"   # SolverMuJoCo (robust frictional contact)
$env:OMNISIM_NEWTON_MJWARP            = "1"   # mujoco_warp (GPU) -- matches the verified run
$env:OMNISIM_NEWTON_SUBSTEPS          = "2"   # EVEN -> CUDA-graph replay -> ~1.15x realtime (odd=1 ran 0.25x)
$env:OMNISIM_NEWTON_STATICS           = "1"   # also a baked field; belt-and-suspenders
$env:OMNISIM_NEWTON_COMPOUND_COLLIDERS = "1"  # 4 foot spheres per ankle (the stride key)
$env:OMNISIM_URDF_USE_INERTIA         = "1"   # use the URDF <inertial> blocks
$env:OMNISIM_NEWTON_USE_LINK_COM      = "1"   # true link COM, not link-origin
$env:OMNISIM_NEWTON_TARGET_KE         = "0"   # controller drives by setTorque -> no motor PD
$env:OMNISIM_NEWTON_TARGET_KD         = "0"
$env:OMNISIM_NEWTON_GROUND_MU         = "1.0"
$env:G1_CMD_VX                        = "0.5" # forward command (m/s)
# wgpu (Vulkan) main view: WREN/OpenGL contends with mujoco_warp's CUDA on the one
# GPU and halves throughput (~0.4x live). Vulkan coexists cleanly -> ~1.2x, so
# realtime mode below plays it smoothly at 1x. THIS is what makes the GUI realtime.
$env:OMNISIM_WGPU_MAINVIEW_FORCE      = "1"

$bin = Join-Path $Repo "msys64\mingw64\bin\omnisim-bin.exe"
# fast mode + wgpu renders ~1.2x realtime (smooth). NOTE: --mode=realtime currently
# stalls to ~0.1x with the wgpu present path (known wgpu app-loop pacing gap), so use
# fast here -- it plays at roughly real speed instead of throttling.
& $bin "projects\policies\research\worlds\g1_unitree_deploy.wbt" --mode=fast $args
