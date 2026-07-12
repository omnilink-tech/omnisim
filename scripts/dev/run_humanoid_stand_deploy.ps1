# Deterministic PURE-POSE stand deploy for a humanoid, using the EXACT proven
# G1 recipe (run_g1_standwave_pose_deploy.ps1): stiff Newton position hold of a
# statically-stable squat NOMINAL, no RL policy, ankle PD off by default.
# Generic across robots -- the robot is selected by -Robot and described by
# projects/policies/controllers/humanoid_stand_deploy/specs/<robot>.json.
#
# Usage:
#   powershell -File scripts/dev/run_humanoid_stand_deploy.ps1 -Robot h1 [-Duration 20]
#   powershell -File scripts/dev/run_humanoid_stand_deploy.ps1 -Robot valkyrie -Gui
#   powershell -File scripts/dev/run_humanoid_stand_deploy.ps1 -Robot g1 -Ke 800
param(
    [Parameter(Mandatory=$true)][ValidateSet("h1","valkyrie","g1")][string]$Robot,
    [int]$Duration = 20,
    [switch]$Gui,
    [double]$Ke = 400,        # Newton position-PD stiffness (raise for heavy robots)
    [double]$Kd = 60,
    [double]$AnkleKp = 0,     # ankle balance PD (0 = pure pose)
    [double]$AnkleKd = 0,
    [double]$AnkBias = 0,     # fore/aft trim: ankle-pitch bias (neg = lean back)
    [double]$HipBias = 0,     # fore/aft trim: hip-pitch bias
    [double]$BalClamp = 0.2,  # max ankle balance correction (rad)
    [switch]$Lean,            # reactive fore/aft ankle lean (catches a cube from behind)
    [switch]$CaptureStep,     # deterministic protective step when the CoM escapes
    [double]$TestPush = 0,    # clean one-shot pelvis push (m/s) to trigger a fall
    [double]$TestPushAng = 0, # test-push direction (rad; 0=+x forward)
    [double]$TestPushT = 1.5, # when the test push fires (s)
    [switch]$Wave,            # ride a feedforward arm wave on top of the stand
    [switch]$Throw,           # throw cubes at the robot (push-recovery test)
    [switch]$Rain,            # drop cubes onto the robot from above (reliable variant)
    [double]$ThrowSpeed = 4.5,# cube launch speed (m/s) toward the torso
    [double]$ThrowPeriod = 2.0,# seconds between throws
    [switch]$ArmsDown,        # G1: arms at the sides (not forward), balanced by reactive arm swing
    [switch]$OneLeg,          # G1: shift weight onto one leg, CoM held over the support foot (torque-ankle CoP)
    [switch]$Manip,           # G1: pick/move/place cubes off a table while standing (composable with -Throw)
    [switch]$ArmMotion,       # G1: loop a routine of bilateral arm exercises (3-D arm motion) while balancing (composable with -Throw)
    [switch]$Squat,           # G1: quasi-static deep knee-bend (squat) and return, repeated, while balancing
    [switch]$Mpc,             # G1: drive the stand with the SAMPLING-MPC brain (g1_mpc spec) instead of reactive overlays
    [switch]$Wbc,             # G1: drive the stand with the in-engine WHOLE-BODY CONTROLLER (operational-space CoM+orientation FF on the servo, g1_tsid spec)
    [switch]$StepMpc,         # G1: in-engine PUSH-RECOVERY MPPI on the position servo (g1_step_mpc.py); settles on the deterministic stand, then the MPC owns the targets
    [switch]$Cmpc,            # G1: CENTROIDAL-MPC + WBC balance (g1_centroidal stack; force QP -> joint FF, + arm CAM). M4 = standing/double-support balance.
    [switch]$Cold,            # disable the warm reload (faster iteration; GUI-friendly, no window reload)
    [string]$ExtraLog = ""
)
$root  = (Resolve-Path "$PSScriptRoot\..\..").Path
# Prefer a dedicated <robot>_hstand_deploy.wbt for this generic harness; fall back
# to <robot>_stand_deploy.wbt. (G1 needs the _hstand_ name so it does NOT collide
# with the pre-existing g1_stand_deploy.wbt, which runs G1's own ONNX controller.)
# -Rain drops cubes from above (reliable); -Throw launches them horizontally.
if ($Manip) {
    # Table-manipulation world (table + DEF GRASP_CUBE* + side-ring DEF CUBE* for
    # composing with -Throw). Lives under research/worlds like the other G1 worlds.
    $world = "$root\projects\policies\research\worlds\g1_hstand_manip.wbt"
} elseif ($ArmMotion) {
    # Arm-motion world (stand + side-ring DEF CUBE* for composing with -Throw).
    $world = "$root\projects\policies\research\worlds\g1_arm_motion.wbt"
} elseif ($Rain) {
    $world = "$root\projects\policies\worlds\${Robot}_hstand_cuberain.wbt"
} elseif ($Throw) {
    $world = "$root\projects\policies\worlds\${Robot}_hstand_cubethrow.wbt"
} else {
    # Prefer the full-arm _hstand_deploy world (projects/policies/worlds OR research/worlds)
    # before the legs-only _stand_deploy fallback. G1's lives in research/worlds.
    $cands = @(
        "$root\projects\policies\worlds\${Robot}_hstand_deploy.wbt",
        "$root\projects\policies\research\worlds\${Robot}_hstand_deploy.wbt",
        "$root\projects\policies\worlds\${Robot}_stand_deploy.wbt"
    )
    $world = $cands[0]
    foreach ($c in $cands) { if (Test-Path $c) { $world = $c; break } }
}
# Some robots' hstand worlds (G1) live under research/worlds; fall back there.
if (-not (Test-Path $world)) {
    $alt = $world -replace [regex]::Escape("\projects\policies\worlds\"), "\projects\policies\research\worlds\"
    if (Test-Path $alt) { $world = $alt }
}
# Spec select: -OneLeg (weight-shift onto one leg) or -ArmsDown (arms at the sides)
# or the default stand. The arms-down spec uses a reactive arm swing; the one-leg
# spec uses the torque-ankle CoP to hold the CoM over the support foot.
$specName = if ($Wbc) { "${Robot}_tsid" } elseif ($Mpc) { "${Robot}_mpc" } elseif ($Manip) { "${Robot}_manip" } elseif ($ArmMotion) { "${Robot}_arm_motion" } elseif ($OneLeg) { "${Robot}_oneleg" } elseif ($Squat) { "${Robot}_squat" } elseif ($ArmsDown) { "${Robot}_armsdown" } else { $Robot }
$spec  = "$root\projects\policies\controllers\humanoid_stand_deploy\specs\${specName}.json"
$log   = if ($ExtraLog) { $ExtraLog } else { "$root\_scratch\stand\${Robot}_hstand.log" }
New-Item -ItemType Directory -Force -Path (Split-Path $log) | Out-Null
Remove-Item $log -ErrorAction SilentlyContinue

if (-not (Test-Path $world)) { Write-Error "no world $world"; exit 1 }
if (-not (Test-Path $spec))  { Write-Error "no spec $spec";  exit 1 }

# Per-robot deploy stiffness lives in the spec (deploy_ke/deploy_kd). Use it as
# the default unless the caller passed -Ke/-Kd explicitly. Heavy robots
# (Valkyrie) need a stiffer ankle hold than the light default.
$specObj = Get-Content $spec -Raw | ConvertFrom-Json
if (-not $PSBoundParameters.ContainsKey('Ke') -and $specObj.PSObject.Properties.Name -contains 'deploy_ke') { $Ke = [double]$specObj.deploy_ke }
if (-not $PSBoundParameters.ContainsKey('Kd') -and $specObj.PSObject.Properties.Name -contains 'deploy_kd') { $Kd = [double]$specObj.deploy_kd }
# Likewise the fore/aft trim: don't clobber the spec's ank_bias/hip_bias with the
# param default (0) unless the caller passed it explicitly.
if (-not $PSBoundParameters.ContainsKey('AnkBias') -and $specObj.PSObject.Properties.Name -contains 'ank_bias') { $AnkBias = [double]$specObj.ank_bias }
if (-not $PSBoundParameters.ContainsKey('HipBias') -and $specObj.PSObject.Properties.Name -contains 'hip_bias') { $HipBias = [double]$specObj.hip_bias }

$env:OMNISIM_HOME      = $root
$env:PYTHONUTF8        = "1"
# The humanoid_stand_deploy controller lives in projects/policies/controllers, but some
# worlds (G1's) are archived under projects/policies/research/worlds, whose project root
# is projects/policies/research -- so OmniSim can't find the controller there. Add the
# rl project as an extra search path so the controller resolves either way.
$env:WEBOTS_EXTRA_PROJECT_PATH = "$root\projects\policies"
# --- deploy-faithful Newton solver config, IDENTICAL to the proven G1 stand ---
$env:OMNISIM_NEWTON_STATICS      = "1"
$env:OMNISIM_NEWTON_SUBSTEPS     = "4"
$env:OMNISIM_NEWTON_FORCE_MUJOCO = "1"
$env:OMNISIM_NEWTON_MJWARP       = "1"
$env:OMNISIM_URDF_USE_INERTIA    = "1"
$env:OMNISIM_NEWTON_BASE_GUARD   = "1"
$env:OMNISIM_NEWTON_SEED_POSE    = "1"
$env:OMNISIM_NEWTON_SEED_REBUILD = "1"
# --- STIFF position PD (proven stand value; -Ke raises it for heavy robots) ---
$env:OMNISIM_NEWTON_TARGET_KE    = "$Ke"
$env:OMNISIM_NEWTON_TARGET_KD    = "$Kd"
$env:OMNISIM_NEWTON_GROUND_MU    = "2.0"
# --- generic stand controller config ---
$env:HUMANOID_STAND_SPEC = $spec
# Arms-down balances via reactive arm swing + return-to-home integral. That
# integral absorbs the cold-load articulation offset, so it runs robustly COLD
# (verified) -- no warm reload, which in the GUI reloads the world and disrupts
# the live window.
if ($ArmsDown) { $env:HSTAND_WARMUP_RELOAD = "0" }
# One-leg weight-shift: enable the one_leg controller; runs cold (CoP feedback is
# robust to the cold-load offset), warm reload off to keep the GUI window stable.
if ($OneLeg) { $env:HSTAND_ONELEG = "1"; $env:HSTAND_WARMUP_RELOAD = "0" }
# Table manipulation: enable the manip overlay; warm-reload ON so the arm tracks
# crisply for the grasp (the cold-load articulation undershoot otherwise misses
# the cube). For a live GUI demo, pass -WarmReload off via the spec if the reload
# disrupts the window; headless is unaffected.
if ($Manip) { $env:HSTAND_MANIP = "1"; $env:HSTAND_WARMUP_RELOAD = "1" }
# Arm-motion exercises: enable the overlay. Runs COLD like arms-down (the
# return-to-home integrals absorb the cold-load offset, so it's robust without a
# warm reload, which keeps the live GUI window stable). No grasp precision needed.
if ($ArmMotion) { $env:HSTAND_ARM_MOTION = "1"; $env:HSTAND_WARMUP_RELOAD = "0" }
# Squat: enable the squat overlay. Warm reload ON -- the RISE phase (standing back
# up) is the unstable part of the motion and is timing-sensitive, so we want the
# crisp warm articulation (the cold first-load under-tracks and topples the rise).
# For a live GUI demo pass -Cold if the reload disrupts the window.
if ($Squat) { $env:HSTAND_SQUAT = "1"; $env:HSTAND_WARMUP_RELOAD = "1" }
# MPC brain: enable the sampling-MPC planner. Warm reload ON -- the cold-load
# articulation undershoot otherwise biases the very planner state we read.
if ($Mpc) { $env:HSTAND_MPC = "1"; $env:HSTAND_WARMUP_RELOAD = "1" }
# WBC: in-engine whole-body controller as operational-space feedforward on the
# position servo (servo holds the centered squat; the WBC adds a per-tick CoM +
# torso-orientation balance torque). Tick-rate so the CUDA graph stays on. Stands
# + survives the designed cube-defense throws. Warm reload ON.
if ($Wbc) {
    $env:HSTAND_WARMUP_RELOAD = "1"
    $env:OMNISIM_INENGINE_TSID = "1"; $env:TSID_TICK = "1"; $env:TSID_FF = "1"
    $env:TSID_KP_COM = "35"; $env:TSID_KD_COM = "12"; $env:TSID_KP_ORI = "90"; $env:TSID_KD_ORI = "18"
    $env:TSID_KP_POST = "0"; $env:TSID_KD_POST = "0"; $env:TSID_KD_JOINT = "0"
    $env:TSID_TAUMAX = "120"; $env:TSID_FF_GRAV = "0"
}
# StepMpc: in-engine push-recovery MPPI on the POSITION SERVO. The deterministic
# stand (default g1 spec, lean + arm-balance overlays) settles the robot during the
# warmup window; then g1_step_mpc captures that pose as the nominal and OWNS the servo
# targets = nominal + MPPI-optimized residual (rolled out in mujoco_warp). Servo mode
# (NO torque), per-TICK (the servo holds between ticks -> CUDA graph stays on, fast).
# Warm reload ON so the captured nominal matches a stabilised session.
if ($StepMpc) {
    $env:HSTAND_WARMUP_RELOAD = "1"
    $env:OMNISIM_INENGINE_PYMOD = "projects.policies.research.mpc.g1_step_mpc:step_mpc"
    Remove-Item Env:\OMNISIM_INENGINE_PYMOD_SS -ErrorAction SilentlyContinue
    Remove-Item Env:\OMNISIM_NEWTON_TORQUE_MODE -ErrorAction SilentlyContinue
    # Capture the in-engine MPC diagnostics (_mpc_log only goes to stderr->DEVNULL
    # in headless otherwise). Sits next to the deploy log.
    $env:OMNISIM_INENGINE_MPC_LOG = if ($ExtraLog) {
        [System.IO.Path]::ChangeExtension($log, $null) + "_mpc.txt"
    } else { "$root\_scratch\stand\smpc_mpc_${Robot}.txt" }
    Remove-Item $env:OMNISIM_INENGINE_MPC_LOG -ErrorAction SilentlyContinue
}
# Cmpc: centroidal-MPC + WBC balance (the user's G1-Newton-MPC plan, M4). The force QP
# computes per-foot GRFs to drive CoM+torso to reference; the WBC maps them to joint
# feedforward on the ke=400 servo; the arm CAM adds angular-momentum balance. Stands +
# tracks CoM to ~1mm; deploys in-engine (no obs gap). Warm reload ON.
if ($Cmpc) {
    $env:HSTAND_WARMUP_RELOAD = "1"
    $env:OMNISIM_INENGINE_PYMOD = "projects.policies.research.mpc.g1_centroidal.driver:cmpc_step"
    $env:CMPC_STAGE = "4"; $env:CMPC_WARM_TICKS = "200"; $env:CMPC_CAM = "1"
    $env:CMPC_KP_COM = "80"; $env:CMPC_KD_COM = "18"
    Remove-Item Env:\OMNISIM_INENGINE_PYMOD_SS -ErrorAction SilentlyContinue
    Remove-Item Env:\OMNISIM_NEWTON_TORQUE_MODE -ErrorAction SilentlyContinue
    $env:OMNISIM_INENGINE_MPC_LOG = if ($ExtraLog) {
        [System.IO.Path]::ChangeExtension($log, $null) + "_cmpc.txt"
    } else { "$root\_scratch\stand\cmpc_${Robot}.txt" }
    Remove-Item $env:OMNISIM_INENGINE_MPC_LOG -ErrorAction SilentlyContinue
}
# -Cold overrides any warm reload (fast iteration; the grasp uses the live hand
# link pose so it still attaches, just with a touch more cold-load slop).
if ($Cold) { $env:HSTAND_WARMUP_RELOAD = "0" }
$env:HSTAND_ANKLE_KP     = "$AnkleKp"
$env:HSTAND_ANKLE_KD     = "$AnkleKd"
$env:HSTAND_ANK_BIAS     = "$AnkBias"
$env:HSTAND_HIP_BIAS     = "$HipBias"
$env:HSTAND_BAL_CLAMP    = "$BalClamp"
$env:HSTAND_WAVE         = if ($Wave) { "1" } else { "0" }
# Lean is per-robot: G1's spec enables it (its stand is marginal), H1/Valkyrie
# leave it off (they hold passively and the lean only destabilises them). Only
# OVERRIDE the spec when -Lean is passed explicitly; otherwise let the spec decide.
if ($PSBoundParameters.ContainsKey('Lean')) { $env:HSTAND_LEAN = if ($Lean) { "1" } else { "0" } }
else { Remove-Item Env:\HSTAND_LEAN -ErrorAction SilentlyContinue }
$env:HSTAND_CAPTURE_STEP = if ($CaptureStep) { "1" } else { "0" }
$env:HSTAND_TEST_PUSH    = "$TestPush"
$env:HSTAND_TEST_PUSH_ANG = "$TestPushAng"
$env:HSTAND_TEST_PUSH_T  = "$TestPushT"
$env:HSTAND_THROW        = if ($Throw) { "1" } else { "0" }
$env:HSTAND_THROW_SPEED  = "$ThrowSpeed"
$env:HSTAND_THROW_PERIOD = "$ThrowPeriod"
$env:HUMANOID_STAND_LOG  = $log
$env:OMNISIM_DEPLOY_LOG  = $log
# Engine log: per-run unique when -ExtraLog is given (so parallel runs / a concurrent
# session don't lock each other out of the shared default), else the shared default.
if ($ExtraLog) {
    $env:OMNISIM_LOG_PATH = [System.IO.Path]::ChangeExtension($log, $null) + "_omnisim.txt"
} else {
    $env:OMNISIM_LOG_PATH = "$root\_scratch\stand\omnisim_log_${Robot}_hstand.txt"
}

Write-Host "=== $Robot deterministic stand (ke=$Ke kd=$Kd ankle_pd=$AnkleKp/$AnkleKd) ===" -ForegroundColor Cyan
if ($Gui) {
    python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration
} else {
    python -u "$root\scripts\dev\headless_runner.py" $world --duration $Duration 2>&1 | Select-Object -Last 10
}

Write-Host "`n===== $Robot STAND RESULT =====" -ForegroundColor Cyan
if (Test-Path $log) {
    $fall = (Select-String -Path $log -Pattern "FALL" | Measure-Object).Count
    $settle = Select-String -Path $log -Pattern "settle done" | Select-Object -First 1
    $last = Get-Content $log | Select-Object -Last 1
    if ($settle) { Write-Host "  $($settle.Line.Trim())" }
    if ($fall -eq 0) { Write-Host "  PASS - no fall. last: $last" -ForegroundColor Green }
    else             { Write-Host "  $fall FALL line(s). last: $last" -ForegroundColor Yellow }
    if ($Squat) {
        $done = Select-String -Path $log -Pattern "SQUAT_DONE" | Select-Object -Last 1
        if ($done) { Write-Host "  [SQUAT] $($done.Line.Trim())" -ForegroundColor Green }
        else       { Write-Host "  [SQUAT] no SQUAT_DONE line (reps may not have completed in -Duration)" -ForegroundColor Yellow }
    }
    if ($Manip) {
        $grasp = (Select-String -Path $log -Pattern "MANIP_GRASP attached" | Measure-Object).Count
        $miss  = (Select-String -Path $log -Pattern "MANIP_GRASP_MISS" | Measure-Object).Count
        $place = (Select-String -Path $log -Pattern "MANIP_PLACE .* OK" | Measure-Object).Count
        $succ  = Select-String -Path $log -Pattern "MANIP_SUCCESS" | Select-Object -Last 1
        Write-Host "  [MANIP] grasped=$grasp miss=$miss placed_ok=$place" -ForegroundColor Cyan
        if ($succ) { Write-Host "  [MANIP] $($succ.Line.Trim())" -ForegroundColor Green }
    }
    if ($ArmMotion) {
        $on = Select-String -Path $log -Pattern "ARM_MOTION on:" | Select-Object -First 1
        $ex = (Select-String -Path $log -Pattern "ARM_MOTION -> " | Measure-Object).Count
        if ($on) { Write-Host "  [ARM_MOTION] $($on.Line.Trim())" -ForegroundColor Cyan }
        Write-Host "  [ARM_MOTION] exercise transitions: $ex" -ForegroundColor Cyan
    }
} else {
    Write-Host "  (no deploy log produced)"
}
