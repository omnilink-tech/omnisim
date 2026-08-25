# LONG-RUN driver: from-scratch G1 walk, the locked architecture
# (frame-stack 4 + lookahead [0.1,0.4] + 3x512 net) at ~1B steps, as a
# self-checkpointing ladder of chunks. Each chunk warm-starts from the
# previous, trains ItersPerChunk iters, evaluates (no-DR), and appends ONE
# improvement-curve datapoint to the curve CSV. lr + entropy anneal across
# the run. The CSV is what the live monitor tails; the agent decides when
# to stop (plateau/regression) -- the driver just runs the ladder.
#
# Usage: long_run_driver.ps1 [-Chunks 20] [-ItersPerChunk 1000]
param(
    [int]$Chunks = 20,
    [int]$ItersPerChunk = 1000
)
$ErrorActionPreference = "Continue"
$root   = (Resolve-Path "$PSScriptRoot\..\..\..\..").Path
$trainer= "$root\projects\policies\research\training\gpu_mjwarp_g1_walk_trainer.py"
$mjcf   = "$root\projects\robots\unitree\g1\urdf\g1_full_kp100.mjcf.xml"
$runtag = "gpu_g1_walk25_long"
$curve  = "$root\_scratch\long_run_curve.csv"
$bestf  = "$root\_scratch\long_run_best.txt"

# locked architecture + validated robust+style recipe
$arch = "--obs-stack 4 --obs-lookahead 0.1,0.4 --hidden-dims 512,512,512"
$gait = "--hold-arms --gait-model human --gait-style winter --gait-a-arm 0.25 --gait-a-lat 0.05 --vx-target 0.4 --gait-freq 1.3 --gait-ramp-s 2.0 --seed-gait-pose --rest-start-frac 0.4 --gait-hip-scale 0.9"
$dr   = "--dr-mass-scale 0.10 --dr-friction-scale 0.20 --dr-damping-scale 0.20 --dr-actuator-kp-scale 0.15 --dr-actuator-kv-scale 0.15 --dr-push-prob 0.01 --dr-push-vmax 0.5 --dr-obs-noise 0.01 --dr-action-latency-max 1 --dr-init-q-band 0.08 --dr-init-vx-bias 0.4 --dr-init-tilt-band 0.05 --dr-init-vel-band 0.1"
$rw   = "--rw-sched -5 --rw-slip -0.5 --vel-l1 -0.3 --upright 1.5 --vel 2.0 --rw-swing-track 2.0 --track-sigma 0.04 --rw-track-vel 0.3"

if (-not (Test-Path $curve)) {
    "chunk,steps_total,reward,fwd_dist_m,firstfall_steps,lr,note" | Set-Content $curve
}
$bestDist = -1.0
$stepsPerChunk = 4096 * 12 * $ItersPerChunk

for ($c = 1; $c -le $Chunks; $c++) {
    # lr + entropy + std-clamp anneal across the run
    if     ($c -le 12) { $lr = "2e-4"; $ent = "0.0008"; $cl = "-1.5" }
    elseif ($c -le 17) { $lr = "1e-4"; $ent = "0.0005"; $cl = "-1.65" }
    else               { $lr = "5e-5"; $ent = "0.0003"; $cl = "-1.8" }

    $run  = "$runtag`_c$c"
    $save = "$root\projects\policies\research\training\runs\$run\policy.pt"
    $init = ""
    if ($c -gt 1) { $init = "--init-from `"$root\projects\policies\research\training\runs\$runtag`_c$($c-1)\policy.pt`"" }

    $cmd = "python -u `"$trainer`" --envs 4096 --iters $ItersPerChunk --rollout 12 --max-ep 1875 --lr $lr --ent-coef $ent --log-std-clamp $cl --dr-seed $(89 + $c) $arch $gait $dr $rw $init --mjcf `"$mjcf`" --save `"$save`""
    Invoke-Expression "$cmd *> `"$root\_scratch\long_c$c.train.log`""
    if ($LASTEXITCODE -ne 0) {
        "$c,$($c*$stepsPerChunk),NA,NA,NA,$lr,TRAIN_FAILED" | Add-Content $curve
        break
    }

    # eval (no DR), parse fwd dist + first-fall
    $ev = (Invoke-Expression "python -u `"$trainer`" --eval --envs 1024 --eval-steps 2500 --no-dr --max-ep 2500 $arch $gait --mjcf `"$mjcf`" --save `"$save`" 2>`$null") -join "`n"
    $dist = "NA"; $ff = "NA"
    if ($ev -match "fwd dist \(m, before first fall\): mean=([0-9.]+)") { $dist = $Matches[1] }
    if ($ev -match "FIRST-FALL step: mean=([0-9.]+)") { $ff = $Matches[1] }
    $rew = "NA"
    $rl = (Get-Content "$root\_scratch\long_c$c.train.log" | Select-String "ep_rew/step~" | Select-Object -Last 1)
    if ($rl -and $rl.Line -match "ep_rew/step~\+?([0-9.]+)") { $rew = $Matches[1] }

    $note = ""
    if ($dist -ne "NA" -and [double]$dist -gt $bestDist) { $bestDist = [double]$dist; $note = "BEST"; "best=$run dist=$dist" | Set-Content $bestf }
    "$c,$($c*$stepsPerChunk),$rew,$dist,$ff,$lr,$note" | Add-Content $curve
}
"DONE chunks=$Chunks bestDist=$bestDist" | Add-Content $curve
