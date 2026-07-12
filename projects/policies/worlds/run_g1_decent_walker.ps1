# ⭐ FLAGSHIP HUMANOID DEMO — "the decent walker" (owner-designated 2026-07-06).
#
# The Unitree G1 walking the OFFICIAL Unitree gait on the PUPPET rig: a fully
# physical robot carried by a visible overhead harness (lam=0.9) beside its
# GHOST hologram, driven by the LSTM+foresight champion wr_decent_walker.pt
# (WBMATCH4 0.868 on the honest shape-only ruler; exam-verified, K=2048).
# Arms swing naturally through vertical, clear of the thighs (ghost v3).
#
#   powershell -ExecutionPolicy Bypass -File projects\policies\worlds\run_g1_decent_walker.ps1
#
# The demo needs the full deploy env (engine hook + corridors + harness), which a
# bare world launch cannot carry -- always start it via this script (or the bash
# equivalent in docs). Owner-verified live: stable, zero falls; live pace is
# currently below the trainer's (open stride-gap thread; see rl-current-state.md).

$ErrorActionPreference = "Stop"
$Repo = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $PSScriptRoot))
Set-Location $Repo

$env:OMNISIM_HOME                      = $Repo
$env:OMNISIM_NEWTON_COMPOUND_COLLIDERS = "1"   # 4-sphere feet = the ghost recording's plant

$bash = (Get-Command bash -ErrorAction SilentlyContinue).Source
if (-not $bash) { $bash = "C:\Program Files\Git\bin\bash.exe" }   # Git Bash (the dev-standard shell here)
$RepoFwd = $Repo -replace "\\","/"
& $bash "projects/policies/training/run_walk_rl.sh" 900 flagshipdemo deploy gui `
  OMNISIM_INENGINE_PYMOD=projects.policies.training.g1_walk_recipe:g1_walk_recipe_deploy `
  WALK_WORLD=projects/policies/worlds/g1_walk_puppet.wbt `
  WHOLE_BODY=1 STAND_SEED=1 STAND_Z=0.75 STAND_POSE=unitree `
  WALK_WARM_TICKS=30 WALK_MJW_RESET=0 OMNISIM_NEWTON_DISABLE_JOINT_CLAMP=1 `
  POLICY_ARCH=lstm POLICY_LAYERS=1 PPO_HID=256 `
  REF_OBS=1 REF_OBS_K=3 REF_OBS_STRIDE=4 `
  GHOST_LUT_JSON=$RepoFwd/projects/policies/controllers/g1_ghost/ghost_official_full_v3_lut.json `
  G1_GHOST_LUT=$RepoFwd/projects/policies/controllers/g1_ghost/ghost_official_full_v3_lut.json `
  GHOST_RESIDUAL=0.10 PHASE_LEAD=0.45 `
  ARM_RESIDUAL=0.10 ELB_HANG_REF=1.6 ELBOW_TARGET=1.6 ELBOW_RESIDUAL=0.10 `
  SHRY_TARGET=0.15 SHRY_YAW_TARGET=0 SHRY_RESIDUAL=0.10 `
  HARNESS_LAM0=0.9 HARNESS_KP=600 HARNESS_KD=60 HARNESS_FY=400 `
  HARNESS_KZ=2000 HARNESS_DZ=150 HARNESS_Z0=0.70 HARNESS_ATT_GHOST=1 `
  VX_MAX=0.45 `
  RES_POLICY=$RepoFwd/projects/policies/training/runs/wr_decent_walker.pt
