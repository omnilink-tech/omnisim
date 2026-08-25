# IMPROVED-SHADOW (C) + CONTACT-MATCHED DR. The plain shadow-C run (walk30)
# reduced the trainer splay but COLLAPSED in Newton deploy (6.6s) -- it overfit
# mujoco_warp's contact. This run adds CONTACT-softness DR (--dr-solref-scale:
# randomize the contact time-constant/damping/impedance) + wider friction/push,
# and sweeps dr-seed across MORE chunks so the policy spans a band of contact
# behaviours instead of one -> should survive the warp->Newton gap. Warm-start
# from the deployable champion (walk26_shape_c8). See docs/developer/g1-improved-shadow.md.
#
# Usage: shadowC_dr_run_driver.ps1 [-Chunks 6] [-ItersPerChunk 1000] [-Front 1.0] [-Solref 0.5]
param([int]$Chunks = 6, [int]$ItersPerChunk = 1000, [double]$Front = 1.0, [double]$Solref = 0.5)
$ErrorActionPreference = "Continue"
$root    = (Resolve-Path "$PSScriptRoot\..\..\..\..").Path
$trainer = "$root\projects\policies\research\training\gpu_mjwarp_g1_walk_trainer.py"
$mjcf    = "$root\projects\robots\unitree\g1\urdf\g1_full_kp100.mjcf.xml"
$runtag  = "gpu_g1_walk31_shadowCdr"
$warm    = "$root\projects\policies\research\training\runs\gpu_g1_walk26_shape_c8\policy.pt"

$arch = "--obs-stack 4 --obs-lookahead 0.1,0.4 --hidden-dims 512,512,512"
$gait = "--hold-arms --gait-model human --gait-style winter --gait-a-arm 0.25 --gait-a-lat 0.05 --vx-target 0.4 --gait-freq 1.3 --gait-ramp-s 2.0 --seed-gait-pose --rest-start-frac 0.4 --gait-hip-scale 0.9"
$shadow = "--gait-lateral human --gait-yaw human --rw-frontal-track $Front --frontal-sigma 0.01"
# HEAVIER + CONTACT DR: solref (the warp<->Newton knob) + wider friction/push.
$dr   = "--dr-mass-scale 0.12 --dr-friction-scale 0.30 --dr-damping-scale 0.20 --dr-actuator-kp-scale 0.15 --dr-actuator-kv-scale 0.15 --dr-solref-scale $Solref --dr-push-prob 0.02 --dr-push-vmax 0.7 --dr-obs-noise 0.01 --dr-action-latency-max 1 --dr-init-q-band 0.08 --dr-init-vx-bias 0.4 --dr-init-tilt-band 0.05 --dr-init-vel-band 0.1"
$rw   = "--rw-sched -5 --rw-slip -0.5 --vel-l1 -0.3 --upright 1.5 --vel 2.0 --rw-shape 2.5 --shape-sigma 0.01"

for ($c = 1; $c -le $Chunks; $c++) {
    if ($c -le 3) { $lr = "1e-4"; $ent = "0.0005"; $cl = "-1.65" }
    else          { $lr = "5e-5"; $ent = "0.0003"; $cl = "-1.8" }
    $save = "$root\projects\policies\research\training\runs\$runtag`_c$c\policy.pt"
    $init = "--init-from `"$warm`""
    if ($c -gt 1) { $init = "--init-from `"$root\projects\policies\research\training\runs\$runtag`_c$($c-1)\policy.pt`"" }
    # per-chunk dr-seed sweep -> each chunk samples a DIFFERENT contact point.
    $cmd = "python -u `"$trainer`" --envs 4096 --iters $ItersPerChunk --rollout 12 --max-ep 1875 --lr $lr --ent-coef $ent --log-std-clamp $cl --dr-seed $(160 + $c * 7) $arch $gait $shadow $dr $rw $init --mjcf `"$mjcf`" --save `"$save`""
    Invoke-Expression "$cmd *> `"$root\_scratch\shadowCdr_c$c.train.log`""
    if ($LASTEXITCODE -ne 0) { "TRAIN FAILED chunk $c" | Add-Content "$root\_scratch\shadowCdr_run_curve.csv"; break }
    "chunk $c done" | Add-Content "$root\_scratch\shadowCdr_run_curve.csv"
}
"DONE shadowCdr chunks=$Chunks" | Add-Content "$root\_scratch\shadowCdr_run_curve.csv"
