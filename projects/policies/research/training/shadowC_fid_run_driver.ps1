# IMPROVED-SHADOW (C) FIDELITY push. walk31 (contact DR) cracked the STABILITY
# half -- it deploys +131m/337s -- but the policy still SPLAYS 26deg in deploy
# (commanded 3.8) by piling residual on top for balance. Now that stability has
# headroom, push fidelity two ways: (1) --rw-frontal-track 2.5 (pay much more
# for matching the slim lateral), (2) --frontal-res-scale 0.3 (cap the policy's
# hip-roll/yaw residual to ~5deg so it CAN'T splay 22deg on top). Warm from
# walk31 to KEEP its contact-DR robustness. See docs/developer/g1-improved-shadow.md.
#
# Usage: shadowC_fid_run_driver.ps1 [-Chunks 4] [-ItersPerChunk 1000] [-Front 2.5] [-ResScale 0.3]
param([int]$Chunks = 4, [int]$ItersPerChunk = 1000, [double]$Front = 2.5, [double]$ResScale = 0.3)
$ErrorActionPreference = "Continue"
$root    = (Resolve-Path "$PSScriptRoot\..\..\..").Path
$trainer = "$root\projects\policies\research\training\gpu_mjwarp_g1_walk_trainer.py"
$mjcf    = "$root\projects\robots\unitree\g1\urdf\g1_full_kp100.mjcf.xml"
$runtag  = "gpu_g1_walk32_shadowCfid"
$warm    = "$root\projects\policies\research\training\runs\gpu_g1_walk31_shadowCdr_c6\policy.pt"

$arch = "--obs-stack 4 --obs-lookahead 0.1,0.4 --hidden-dims 512,512,512"
$gait = "--hold-arms --gait-model human --gait-style winter --gait-a-arm 0.25 --gait-a-lat 0.05 --vx-target 0.4 --gait-freq 1.3 --gait-ramp-s 2.0 --seed-gait-pose --rest-start-frac 0.4 --gait-hip-scale 0.9"
# Shadow C + STRONGER frontal-track + TIGHT frontal residual.
$shadow = "--gait-lateral human --gait-yaw human --rw-frontal-track $Front --frontal-sigma 0.01 --frontal-res-scale $ResScale"
# KEEP walk31's contact DR (the stability that lets us push fidelity).
$dr   = "--dr-mass-scale 0.12 --dr-friction-scale 0.30 --dr-damping-scale 0.20 --dr-actuator-kp-scale 0.15 --dr-actuator-kv-scale 0.15 --dr-solref-scale 0.5 --dr-push-prob 0.02 --dr-push-vmax 0.7 --dr-obs-noise 0.01 --dr-action-latency-max 1 --dr-init-q-band 0.08 --dr-init-vx-bias 0.4 --dr-init-tilt-band 0.05 --dr-init-vel-band 0.1"
$rw   = "--rw-sched -5 --rw-slip -0.5 --vel-l1 -0.3 --upright 1.5 --vel 2.0 --rw-shape 2.5 --shape-sigma 0.01"

for ($c = 1; $c -le $Chunks; $c++) {
    if ($c -le 2) { $lr = "1e-4"; $ent = "0.0005"; $cl = "-1.65" }
    else          { $lr = "5e-5"; $ent = "0.0003"; $cl = "-1.8" }
    $save = "$root\projects\policies\research\training\runs\$runtag`_c$c\policy.pt"
    $init = "--init-from `"$warm`""
    if ($c -gt 1) { $init = "--init-from `"$root\projects\policies\research\training\runs\$runtag`_c$($c-1)\policy.pt`"" }
    $cmd = "python -u `"$trainer`" --envs 4096 --iters $ItersPerChunk --rollout 12 --max-ep 1875 --lr $lr --ent-coef $ent --log-std-clamp $cl --dr-seed $(200 + $c * 7) $arch $gait $shadow $dr $rw $init --mjcf `"$mjcf`" --save `"$save`""
    Invoke-Expression "$cmd *> `"$root\_scratch\shadowCfid_c$c.train.log`""
    if ($LASTEXITCODE -ne 0) { "TRAIN FAILED chunk $c" | Add-Content "$root\_scratch\shadowCfid_run_curve.csv"; break }
    "chunk $c done" | Add-Content "$root\_scratch\shadowCfid_run_curve.csv"
}
"DONE shadowCfid chunks=$Chunks" | Add-Content "$root\_scratch\shadowCfid_run_curve.csv"
