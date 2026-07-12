# Drive a quadruped across terrain with the DETERMINISTIC in-engine locomotion MPC
# (OMNISIM_INENGINE_MPC_LOCO -> projects/policies/research/mpc/quad_mpc_engine.py).
# No RL policy: the MPC owns the leg joint targets (trot + planned MPPI residual)
# rolled out in the deploy's own mujoco_warp solver. The controller runs BARE
# (gait keep-alive); the engine MPC overrides the leg targets each tick.
#
# Default world = the BIGFOOT terrain world (foot-redesign: point sphere feet ->
# wide box contact-patch soles; offline A/B ~+85% forward speed, holds heading).
# Pass -Orig to drive the stock point-foot rough track for the A/B control.
#
# Usage:
#   powershell -File scripts/dev/run_quad_mpc_engine.ps1 [-Duration 30] [-Orig] [-Gui]
#                  [-Yaw 6] [-Wz 2] [-K 64] [-H 16]
param(
    [int]$Duration = 30,
    [switch]$Orig,           # drive the stock point-foot rough track (A/B control)
    [switch]$Gui,
    [double]$Yaw = 6.0,      # MPC heading weight (0 = off; the patch foot tolerates it)
    [double]$Wz  = 2.0,
    [int]$K = 64,            # MPPI samples (the rollout is CUDA-graph captured -> K is cheap)
    [int]$H = 16,            # horizon (control ticks)
    [int]$RollSub = 0        # rollout substeps (0 = match the live substeps; lower = faster plan)
)
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
if ($Orig) { $world = "$root\projects\policies\research\worlds\spot_rough_track.wbt" }
else       { $world = "$root\projects\policies\research\worlds\spot_terrain_mpc_bigfoot.wbt" }
$env:OMNISIM_INENGINE_MPC_LOG = "$root\_scratch\spot_terrain_mpc_loco.log"
Remove-Item $env:OMNISIM_INENGINE_MPC_LOG -ErrorAction SilentlyContinue
$log = "$root\_scratch\spot_terrain_mpc.log"
New-Item -ItemType Directory -Force -Path "$root\_scratch" | Out-Null
Remove-Item $log -ErrorAction SilentlyContinue

# --- Newton deploy env (matches run_spot_walk_deploy.ps1) + collidable terrain ---
$env:OMNISIM_HOME = $root
$env:OMNISIM_LOG_PATH = "$root\_scratch\omnisim_log_terrain_mpc.txt"
$env:OMNISIM_NEWTON_FORCE_MUJOCO = "1"; $env:OMNISIM_REQUIRE_NEWTON = "1"
$env:OMNISIM_REQUIRE_MUJOCO_SOLVER = "1"; $env:OMNISIM_NEWTON_MJWARP = "1"
$env:OMNISIM_NEWTON_STATICS = "1"        # make the static terrain boxes collidable
$env:OMNISIM_URDF_USE_INERTIA = "1"; $env:OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE = "1"
$env:OMNISIM_NEWTON_SEED_POSE = "1"; $env:OMNISIM_NEWTON_GROUND_MU = "2.0"
$env:OMNISIM_NEWTON_SUBSTEPS = "8"
$env:OMNISIM_NEWTON_TARGET_KE = "500"; $env:OMNISIM_NEWTON_TARGET_KD = "60"

# --- Deterministic in-engine locomotion MPC ---
$env:OMNISIM_INENGINE_MPC_LOCO = "1"
$env:MPC_LOCO_GAIT_MODULE = "projects.policies.control.gait.spot_trot_gait"
$env:MPC_LOCO_VX = "0.4"; $env:MPC_LOCO_FREQ = "1.4"; $env:MPC_LOCO_DUTY = "0.6"
$env:MPC_LOCO_STEP_H = "0.06"; $env:MPC_LOCO_BODY_H = "0.55"
$env:MPC_LOCO_K = "$K"; $env:MPC_LOCO_H = "$H"; $env:MPC_LOCO_EVERY = "2"
$env:MPC_LOCO_SIGMA = "0.10"; $env:MPC_LOCO_RESMAX = "0.35"; $env:MPC_LOCO_LAM = "0.2"
$env:MPC_LOCO_WARM = "4"; $env:MPC_LOCO_YAW = "$Yaw"; $env:MPC_LOCO_WZ = "$Wz"
if ($RollSub -gt 0) { $env:MPC_LOCO_ROLL_SUB = "$RollSub" }
# The rollout is captured into a CUDA graph (~30 ms/plan vs ~1.5 s for the
# Python step-loop). Set MPC_LOCO_NOGRAPH=1 to force the slow fallback.

# --- Bare keep-alive controller (no RL residual; the MPC owns the leg targets) ---
$env:SPOT_POLICY_ONNX = "$root\__no_policy__"
$env:SPOT_GAIT_VX = "0.4"; $env:SPOT_GAIT_FREQ = "1.4"; $env:SPOT_GAIT_DUTY = "0.6"
$env:SPOT_GAIT_STEP_H = "0.06"; $env:SPOT_GAIT_BODY_H = "0.55"; $env:SPOT_GAIT_RAMP_S = "1.0"
$env:SPOT_DEPLOY_LOG = $log; $env:SPOT_DEPLOY_TRACE = "1"

$foot = if ($Orig) { "ORIG point-foot" } else { "BIGFOOT box-patch" }
$guiArg = ""; if ($Gui) { $guiArg = "--gui" }
Write-Host "[terrain MPC] $foot | deterministic loco MPC K=$K H=$H yaw=$Yaw wz=$Wz | dur=${Duration}s"
python -u "$root\scripts\dev\headless_runner.py" $world --duration $Duration $guiArg 2>&1 | Select-Object -Last 4

Write-Host "----- result -----"
if (Test-Path $log) {
    $pts = Get-Content $log | Where-Object { $_ -match '^\[t=' } | ForEach-Object {
        if ($_ -match 't=([\d.]+)s.*x=([+\-.\d]+).*y=([+\-.\d]+).*bz=([\d.]+).*roll=([+\-.\d]+).*pitch=([+\-.\d]+)') {
            [pscustomobject]@{ t=[double]$Matches[1]; x=[double]$Matches[2]; y=[double]$Matches[3]; bz=[double]$Matches[4]; roll=[double]$Matches[5]; pitch=[double]$Matches[6] }
        }
    }
    if ($pts) {
        $last = $pts[-1]; $maxx = ($pts | Measure-Object x -Maximum).Maximum
        $fall = $pts | Where-Object { $_.bz -lt 0.20 -or [math]::Abs($_.roll) -gt 1.0 -or [math]::Abs($_.pitch) -gt 1.0 } | Select-Object -First 1
        Write-Host ("  forward x={0:N2} m (max {1:N2})  y-drift={2:N2} m  final t={3:N1}s bz={4:N2}" -f $last.x,$maxx,$last.y,$last.t,$last.bz)
        if ($fall) { Write-Host ("  FELL @ t={0:N0}s x={1:N2} (bz={2:N2} roll={3:N2} pitch={4:N2})" -f $fall.t,$fall.x,$fall.bz,$fall.roll,$fall.pitch) }
        else { Write-Host "  no collapse detected (upright through the run)" }
    } else { Write-Host "  (no [t=] telemetry -- check $log and the mpc log)" }
} else { Write-Host "  (no deploy log at $log)" }
