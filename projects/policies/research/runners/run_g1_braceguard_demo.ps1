# G1 BRACEGUARD push-recovery demo + honest A/B/C survival test.
#
# Three deterministic (NO-RL, pure-pose fallback) configs against the SAME side-throw
# cube sequence (cube_thrower rings the robot with 8 cubes and fires each at the torso):
#
#   baseline  the committed stand: NOMINAL squat, arms DOWN, no defense  (the ~13.8 s
#             passive-ankle resister the last commit found was the best deployable hold)
#   secure    LAYER A: deeper squat (knee +0.08, hip_pitch -0.04 => -0.34) + WIDER base
#             (hip_roll +/-0.10 abduction) + matched ankle_roll +/-0.10 (soles flat) +
#             a low symmetric GUARD arm pose. Pure pose -- lowers the CoM, widens the
#             support polygon. No block.
#   block     LAYER A + LAYER B: same secure stance, PLUS the reactive ORIENT-AND-BAR
#             hand-block. The thrower publishes each cube's bearing + flight-time to a
#             threat channel; for front-hemisphere cubes the robot yaws the waist to
#             FACE the cube (planted feet) and throws BOTH forearms into a two-hand BAR,
#             timed to be OUT across the cube's arrival. Rear/side cubes -> Layer A only.
#
# All three are deterministic feedforward (the proven sim-to-deploy-crossing config:
# stiff KE=400 legs + feedforward arms, NO policy). Survival is read from each deploy
# log (FALL@<t> sim-seconds) so the comparison is apples-to-apples.
#
# Usage:
#   run_g1_braceguard_demo.ps1 [-Mode all|baseline|secure|block] [-Duration 45] [-Gui]
#                              [-ThrowSpeed 4.0] [-ThrowPeriod 3.0] [-AnkBias -0.04]
#                              [-Lead 280]
param(
    [ValidateSet("all", "baseline", "secure", "block")]
    [string]$Mode = "all",
    [int]$Duration = 45,
    [switch]$Gui,
    [double]$ThrowSpeed = 4.0,
    [double]$ThrowPeriod = 3.0,
    [double]$AnkBias = -0.04,    # CoM-centering ankle trim (the deploy-vs-mujoco lean fix)
    [double]$Lead = 280,         # ms before impact to reach BAR-OUT
    # Secure-stance geometry (overridable for the fore/aft re-centering sweep). Defaults
    # are the BraceGuard spec; the deeper squat shifts the deploy CoM, so AnkBias must be
    # re-centered for it (a deeper squat tips backward at the baseline's -0.04).
    [double]$KneeBias = 0.08,        # knee 0.52 -> +KneeBias
    [double]$HipPitchBias = -0.04,   # hip_pitch -0.30 -> +HipPitchBias
    [double]$HipRoll = 0.10,         # abduction (wider base)
    [double]$AnkleRoll = 0.10,       # matched sole-flat counter-rotation
    [ValidateSet("guard", "down", "spec")]
    [string]$ArmPose = "guard"       # arm hold pose for secure/block (down isolates the stance)
)
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
$world = "$root\projects\policies\worlds\g1_standwave_push_throw.wbt"
$scratch = "$root\_scratch\stand"
New-Item -ItemType Directory -Force -Path $scratch | Out-Null

# ── Newton deploy-fidelity config (IDENTICAL to run_g1_standwave_push_deploy.ps1; the
#    proven faithful solver setup -- do NOT drop any of these or the stand tips early). ──
$env:OMNISIM_HOME              = $root
$env:PYTHONUTF8                = "1"
$env:OMNISIM_NEWTON_STATICS    = "1"
$env:OMNISIM_NEWTON_SUBSTEPS   = "4"
$env:OMNISIM_NEWTON_FORCE_MUJOCO = "1"
$env:OMNISIM_NEWTON_MJWARP     = "1"
$env:OMNISIM_URDF_USE_INERTIA  = "1"
$env:OMNISIM_NEWTON_BASE_GUARD = "1"
$env:OMNISIM_NEWTON_SEED_POSE  = "1"
$env:OMNISIM_NEWTON_SEED_REBUILD = "1"
$env:OMNISIM_NEWTON_USE_LINK_COM = "1"   # MANDATORY CoM parity (else tips in <0.5s)
$env:OMNISIM_NEWTON_TARGET_KE  = "400"
$env:OMNISIM_NEWTON_TARGET_KD  = "60"
$env:OMNISIM_NEWTON_GROUND_MU  = "2.0"

# ── Deterministic stand: NO policy, pure pose hold (the deployable push-resister). ──
$env:G1_BALANCE_FALLBACK = "1"
$env:G1_ANK_PITCH_BIAS   = "$AnkBias"
$env:G1_ARMS_ANKLE_KP    = "0"
$env:G1_ARMS_ANKLE_KD    = "0"
$env:G1_ARMS_ANKLE_KP_R  = "0"
$env:G1_ARMS_ANKLE_KD_R  = "0"
$env:G1_WARMUP_RELOAD    = "1"           # warm articulation (cold-load trap)
Remove-Item Env:\G1_STANDWAVE_REF -ErrorAction SilentlyContinue

# ── Side-throw supervisor. ──
$env:THROW_SPEED  = "$ThrowSpeed"
$env:THROW_PERIOD = "$ThrowPeriod"
$env:THROW_Z      = "0.12"
$env:THROW_START  = "2.5"

function Run-Config([string]$name) {
    $log = "$scratch\g1_braceguard_$name.log"
    $threat = "$scratch\g1_threat_$name.json"
    Remove-Item $log, $threat, "$threat.tmp" -ErrorAction SilentlyContinue
    $env:G1_DEPLOY_LOG    = $log
    $env:OMNISIM_LOG_PATH = "$scratch\omnisim_log_braceguard_$name.txt"

    # Per-config knobs (reset the optional ones each time).
    Remove-Item Env:\G1_KNEE_BIAS, Env:\G1_HIP_ROLL_NOMINAL, Env:\G1_ANKLE_ROLL_NOMINAL, `
                Env:\G1_HIP_PITCH_BIAS, Env:\G1_THREAT_CHANNEL, Env:\G1_BLOCK_LEAD_MS `
                -ErrorAction SilentlyContinue
    switch ($name) {
        "baseline" {
            $env:G1_ARM_POSE = "down"
        }
        "secure" {
            $env:G1_ARM_POSE          = "$ArmPose"
            $env:G1_KNEE_BIAS         = "$KneeBias"
            $env:G1_HIP_PITCH_BIAS    = "$HipPitchBias"
            $env:G1_HIP_ROLL_NOMINAL  = "$HipRoll"
            $env:G1_ANKLE_ROLL_NOMINAL= "$AnkleRoll"
        }
        "block" {
            $env:G1_ARM_POSE          = "$ArmPose"
            $env:G1_KNEE_BIAS         = "$KneeBias"
            $env:G1_HIP_PITCH_BIAS    = "$HipPitchBias"
            $env:G1_HIP_ROLL_NOMINAL  = "$HipRoll"
            $env:G1_ANKLE_ROLL_NOMINAL= "$AnkleRoll"
            $env:G1_THREAT_CHANNEL    = $threat
            $env:G1_BLOCK_LEAD_MS     = "$Lead"
        }
    }

    Write-Host "`n=== BRACEGUARD [$name] (ke=400 fallback, throw=$ThrowSpeed m/s every $ThrowPeriod s) ===" -ForegroundColor Cyan
    if ($Gui) {
        python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration
    } else {
        python -u "$root\scripts\dev\headless_runner.py" $world --duration $Duration 2>&1 | Select-Object -Last 4
    }

    # Survival = first FALL@<t> in the deploy log (sim seconds); else "no fall".
    $surv = "no/empty log"
    if ((Test-Path $log) -and ((Get-Item $log).Length -gt 0)) {
        $fall = Select-String -Path $log -Pattern "FALL@([0-9.]+)s" | Select-Object -First 1
        if ($fall) {
            $surv = "FALL@" + $fall.Matches[0].Groups[1].Value + "s"
        } else {
            $lastLine = (Select-String -Path $log -Pattern "t=\s*[0-9.]+s" | Select-Object -Last 1).Line
            if ($lastLine) {
                $m = [regex]::Match([string]$lastLine, "t=\s*([0-9.]+)s")
                $surv = "NO FALL through t=" + $m.Groups[1].Value + "s"
            } else { $surv = "ran (no t= line yet)" }
        }
    }
    [pscustomobject]@{ Config = $name; Survival = $surv; Log = $log }
}

$modes = if ($Mode -eq "all") { @("baseline", "secure", "block") } else { @($Mode) }
$results = foreach ($m in $modes) { Run-Config $m }

Write-Host "`n===== BRACEGUARD A/B/C SURVIVAL =====" -ForegroundColor Green
$results | Format-Table -AutoSize
Write-Host "Logs in $scratch (g1_braceguard_<config>.log)" -ForegroundColor DarkGray
