# Run the G1 standing deploy world headless and report survival.
#
# STATUS 2026-06-10: G1 STANDS in the real deploy (OmniSim Newton/mujoco_warp).
# Root causes of the long-standing fall, both found + fixed this session:
#   1. The deploy model's whole-body CoM sat ~5 mm AHEAD of its foot front at the
#      old NOMINAL (hip -0.20 / knee 0.42), so it tipped FORWARD at ~1.3 s under
#      ANY control -- verified in PLAIN mujoco, so NOT a sim2sim gap and NOT the
#      policy. (The OmniSim URDF importer places the foot ~35 mm further back than
#      newton's native add_urdf, which stands at the shallow pose.) FIX: a deeper
#      squat NOMINAL (hip -0.30 / knee 0.52) recentres the CoM behind the foot
#      front -> statically stable (holds 15 s in plain mujoco, pitch settles +0.04).
#   2. The analytic ankle balance PD DESTABILISED roll (its finite-diff roll_rate
#      kicks ankle_roll at handover) -> the robot fell in ROLL even at the stable
#      pose. FIX: the analytic PD is now OFF by default; balance is the RL policy's
#      job (residual on pure NOMINAL), and the static pose stands with zero control.
# With the new NOMINAL + PD-off this script STANDS indefinitely (roll 0, pitch +0.04,
# bz 0.776, 0 falls). See docs/developer/rl-phase-a-validation-log.md.
#
# Usage:  powershell -File scripts/dev/run_g1_stand_deploy.ps1 [-Duration 60]
param(
    [int]$Duration = 60
)
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
$world = "$root\projects\policies\worlds\g1_stand_deploy.wbt"
$log = "$root\_scratch\g1_stand_deploy.log"
Remove-Item $log -ErrorAction SilentlyContinue

$env:OMNISIM_HOME            = $root
# --- the deploy-faithful Newton solver config (legged humanoid) ---
$env:OMNISIM_NEWTON_STATICS      = "1"      # static-collider ground for forced-MuJoCo
$env:OMNISIM_NEWTON_SUBSTEPS     = "4"      # 16 ms control / 250 Hz physics
$env:OMNISIM_NEWTON_FORCE_MUJOCO = "1"      # the deploy solver
$env:OMNISIM_REQUIRE_NEWTON  = "1"   # assert Newton engaged -- fail loud, no silent ODE fallback
$env:OMNISIM_REQUIRE_MUJOCO_SOLVER = "1"   # assert Newton MuJoCo solver (not XPBD) -- the trained engine
$env:OMNISIM_NEWTON_MJWARP       = "1"      # GPU mujoco_warp (same engine the trainer uses)
$env:OMNISIM_URDF_USE_INERTIA    = "1"
$env:OMNISIM_NEWTON_BASE_GUARD   = "1"      # graceful-fail guard for any transient tip
# --- spawn SEEDED in the (deeper-squat) NOMINAL so there is no straight->squat fold ---
$env:OMNISIM_NEWTON_SEED_POSE    = "1"
$env:OMNISIM_NEWTON_SEED_REBUILD = "1"      # rebuild solver on the mujoco_warp engine from the seeded pose
# --- stiff position PD ---
$env:OMNISIM_NEWTON_TARGET_KE    = "400"
$env:OMNISIM_NEWTON_TARGET_KD    = "60"
$env:OMNISIM_NEWTON_GROUND_MU    = "2.0"
# --- control: the analytic ankle PD is OFF by default (it destabilised roll). The
#     deeper-squat NOMINAL is statically stable; set G1_POLICY_ONNX to add the RL
#     residual for perturbation rejection, or leave fallback for a pure-pose stand. ---
$env:G1_BALANCE_FALLBACK         = "1"      # pure NOMINAL hold (statically stable)
$env:G1_DEPLOY_LOG               = $log

Write-Host "Running g1_stand_deploy.wbt for ${Duration}s (new NOMINAL, PD off, seeded)..."
python -u "$root\scripts\dev\headless_runner.py" $world --duration $Duration 2>&1 | Select-Object -Last 4

Write-Host "`n===== G1 STAND RESULT ====="
if (Test-Path $log) {
    $fall = (Select-String -Path $log -Pattern "FALL" | Measure-Object).Count
    $last = Get-Content $log | Select-Object -Last 1
    if ($fall -eq 0) { Write-Host "  PASS - no fall. last: $last" }
    else             { Write-Host "  FAIL - $fall fall line(s). last: $last" }
} else {
    Write-Host "  (no deploy log produced)"
}
