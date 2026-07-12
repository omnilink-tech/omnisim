# IMPROVED-SHADOW (C) WIDE-DR. Tests option A: does WIDER DR around the
# (already-matched) contact center claw back deploy fidelity? The contact-match
# diagnostic showed trainer<->deploy contact params are identical, so walk31's
# solref DR helped via robustness to the solver-behaviour SPREAD, not center-
# correction. Hypothesis: a wider spread -> a policy that needs LESS splay for
# balance -> the same gentle frontal-track pulls the deploy splay below walk31's
# 26deg WITHOUT falling. Keep walk31's gentle rw-frontal-track 1.0 + NO residual
# cap (walk32 showed capping kills stability). Warm from walk31. More chunks +
# swept seeds = more solver-behaviour samples (DR is per-run). See g1-universal-tracker.md.
#
# Usage: shadowC_widedr_run_driver.ps1 [-Chunks 6] [-ItersPerChunk 1000] [-Solref 0.65]
param([int]$Chunks = 6, [int]$ItersPerChunk = 1000, [double]$Solref = 0.65)
$ErrorActionPreference = "Continue"
$root    = (Resolve-Path "$PSScriptRoot\..\..\..").Path
$trainer = "$root\projects\policies\research\training\gpu_mjwarp_g1_walk_trainer.py"
$mjcf    = "$root\projects\robots\unitree\g1\urdf\g1_full_kp100.mjcf.xml"
$runtag  = "gpu_g1_walk33_shadowCwide"
$warm    = "$root\projects\policies\research\training\runs\gpu_g1_walk31_shadowCdr_c6\policy.pt"

$arch = "--obs-stack 4 --obs-lookahead 0.1,0.4 --hidden-dims 512,512,512"
$gait = "--hold-arms --gait-model human --gait-style winter --gait-a-arm 0.25 --gait-a-lat 0.05 --vx-target 0.4 --gait-freq 1.3 --gait-ramp-s 2.0 --seed-gait-pose --rest-start-frac 0.4 --gait-hip-scale 0.9"
# Gentle frontal-track (walk31's), NO residual cap.
$shadow = "--gait-lateral human --gait-yaw human --rw-frontal-track 1.0 --frontal-sigma 0.01"
# WIDER DR: bigger contact spread + push + friction + mass (timeconst clamped safe).
$dr   = "--dr-mass-scale 0.15 --dr-friction-scale 0.45 --dr-damping-scale 0.30 --dr-actuator-kp-scale 0.20 --dr-actuator-kv-scale 0.20 --dr-solref-scale $Solref --dr-push-prob 0.03 --dr-push-vmax 1.0 --dr-obs-noise 0.015 --dr-action-latency-max 2 --dr-init-q-band 0.10 --dr-init-vx-bias 0.4 --dr-init-tilt-band 0.06 --dr-init-vel-band 0.12"
$rw   = "--rw-sched -5 --rw-slip -0.5 --vel-l1 -0.3 --upright 1.5 --vel 2.0 --rw-shape 2.5 --shape-sigma 0.01"

for ($c = 1; $c -le $Chunks; $c++) {
    if ($c -le 3) { $lr = "1e-4"; $ent = "0.0005"; $cl = "-1.65" }
    else          { $lr = "5e-5"; $ent = "0.0003"; $cl = "-1.8" }
    $save = "$root\projects\policies\research\training\runs\$runtag`_c$c\policy.pt"
    $init = "--init-from `"$warm`""
    if ($c -gt 1) { $init = "--init-from `"$root\projects\policies\research\training\runs\$runtag`_c$($c-1)\policy.pt`"" }
    $cmd = "python -u `"$trainer`" --envs 4096 --iters $ItersPerChunk --rollout 12 --max-ep 1875 --lr $lr --ent-coef $ent --log-std-clamp $cl --dr-seed $(300 + $c * 13) $arch $gait $shadow $dr $rw $init --mjcf `"$mjcf`" --save `"$save`""
    Invoke-Expression "$cmd *> `"$root\_scratch\shadowCwide_c$c.train.log`""
    if ($LASTEXITCODE -ne 0) { "TRAIN FAILED chunk $c" | Add-Content "$root\_scratch\shadowCwide_run_curve.csv"; break }
    "chunk $c done" | Add-Content "$root\_scratch\shadowCwide_run_curve.csv"
}
"DONE shadowCwide chunks=$Chunks" | Add-Content "$root\_scratch\shadowCwide_run_curve.csv"
