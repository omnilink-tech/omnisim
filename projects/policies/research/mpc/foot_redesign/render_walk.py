# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Render the bigfoot-G1 offline walk to a video so it can be WATCHED.

Runs the SAME MPPI-on-gait walk that stayed up >=10 s (bigfoot legs MJCF + wider stance,
lateral-tuned config) in mujoco_warp (= the Newton deploy solver), recording the real
world's qpos every control tick, then replays those REAL physics states through MuJoCo's
offscreen renderer with a tracking side camera and ffmpeg-encodes an mp4.

  python projects/policies/research/mpc/foot_redesign/render_walk.py --robot g1_big --secs 12

Honest note: the frames are a visualization of the actual physics trajectory (not a
re-sim, not kinematic puppetry) — the qpos shown is exactly what the solver produced.
"""
from __future__ import annotations
import argparse, importlib, math, os, subprocess, sys
from pathlib import Path
import numpy as np

os.environ.setdefault("MUJOCO_GL", "glfw")
HERE = Path(__file__).resolve().parent
REPO = next(p for p in HERE.parents if (p / "AGENTS.md").exists())
sys.path.insert(0, str(REPO))

import mujoco
import warp as wp  # noqa
import mujoco_warp as mjw

M = "projects/policies/research/mpc/foot_redesign/models"
MODELS = {
    "g1_big":  (f"{M}/g1_bigfoot_legs.mjcf.xml", "projects.policies.control.gait.g1_human_gait"),
    "g1_orig": (f"{M}/g1_orig_legs.mjcf.xml",    "projects.policies.control.gait.g1_human_gait"),
}
hw = importlib.import_module("projects.policies.research.mpc.humanoid_walk_offline")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="g1_big", choices=list(MODELS))
    ap.add_argument("--secs", type=float, default=12.0)
    ap.add_argument("--vx", type=float, default=0.12)
    ap.add_argument("--freq", type=float, default=0.80)
    ap.add_argument("--step-width", type=float, default=0.16)
    ap.add_argument("--K", type=int, default=96)
    ap.add_argument("--H", type=int, default=22)
    ap.add_argument("--sub", type=int, default=8)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--out", default=str(REPO / "_scratch/foot_redesign/g1_bigfoot_walk.mp4"))
    a = ap.parse_args()

    mjcf, gaitmod = MODELS[a.robot]
    stg = importlib.import_module(gaitmod)
    gp = stg.GaitParams(vx=a.vx, freq=a.freq, style="ik", lateral="lipm", ramp_s=2.0)
    if hasattr(gp, "step_width"):
        gp.step_width = a.step_width

    mjm = mujoco.MjModel.from_xml_path(str(REPO / mjcf))
    ctrl = hw.build_ctrl(mjm, mujoco, False)
    NJ = len(ctrl)
    to_qpos = np.array([c["qpos"] for c in ctrl], np.int32)
    to_ctrl = np.array([c["ctrl"] for c in ctrl], np.int32)
    is_leg = np.array([c["kind"] == "leg" for c in ctrl])
    src = np.array([c["src"] for c in ctrl], np.int32)
    nq, nv, nu = mjm.nq, mjm.nv, mjm.nu
    dt_ctrl = a.sub * mjm.opt.timestep
    omega = 2.0 * math.pi * gp.freq
    K, H, sub = a.K, a.H, a.sub
    res_max = np.full(NJ, 0.18)
    sigma = np.full(NJ, 0.05)

    def gait(phase, tss):
        legs, arms, _ = stg.targets_np(phase, gp, t_since_start=tss)
        legs = np.asarray(legs, np.float64); arms = np.asarray(arms, np.float64)
        return np.where(is_leg, legs[src], arms[np.clip(src, 0, len(arms) - 1)])

    def seed(data, nw, qpos, qvel):
        data.qpos.assign(np.broadcast_to(qpos, (nw, nq)).astype(np.float32).reshape(data.qpos.numpy().shape))
        data.qvel.assign(np.broadcast_to(qvel, (nw, nv)).astype(np.float32).reshape(data.qvel.numpy().shape))

    def set_ctrl(data, nw, qt):
        c = data.ctrl.numpy().reshape(nw, nu)
        c[:, to_ctrl] = qt.astype(c.dtype); c[:, to_ctrl + 1] = 0.0
        data.ctrl.assign(c.reshape(data.ctrl.numpy().shape))

    mjd = mujoco.MjData(mjm); mujoco.mj_forward(mjm, mjd)
    real = mjw.put_data(mjm, mjd, nworld=1, njmax=256, nconmax=256)
    rm = mjw.put_model(mjm)
    roll = mjw.put_data(mjm, mjd, nworld=K, njmax=256, nconmax=256)

    stand = gait(stg.DS_PHASE, 0.0)
    q0 = mjd.qpos.copy().astype(np.float64)
    q0[0:3] = [0, 0, gp.pelvis_height]; q0[3:7] = [1, 0, 0, 0]
    for i in range(NJ):
        q0[to_qpos[i]] = stand[i]
    seed(real, 1, q0, np.zeros(nv)); mjw.forward(rm, real)

    phase = stg.DS_PHASE; nom_res = np.zeros(NJ)
    rng = np.random.default_rng(0)
    n_ticks = int(a.secs / dt_ctrl)
    zref = gp.pelvis_height - 0.03; z_fall = 0.55 * (gp.pelvis_height / 0.755)
    W_VX, W_UP, W_RATE, W_YAW, W_Y, W_H, W_VZ = 8.0, 30.0, 9.0, 13.0, 75.0, 150.0, 40.0

    def rpy(qw, qx, qy, qz):
        return (np.arctan2(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy)),
                np.arcsin(np.clip(2*(qw*qy-qz*qx), -1, 1)),
                np.arctan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz)))

    traj = []
    fell_at = None
    print(f"[render] simulating {a.robot} walk {a.secs}s ({n_ticks} ticks)...")
    for t in range(n_ticks):
        rq = real.qpos.numpy().reshape(nq); rv = real.qvel.numpy().reshape(nv)
        tss = t * dt_ctrl
        seed(roll, K, rq, rv)
        noise = rng.normal(0, 1, (K, NJ)) * sigma[None, :]
        deltas = np.clip(nom_res[None, :] + noise, -res_max[None, :], res_max[None, :]); deltas[0] = nom_res
        for h in range(H):
            qt = gait(phase + h * omega * dt_ctrl, tss + h * dt_ctrl)[None, :] + deltas
            set_ctrl(roll, K, qt)
            for _ in range(sub):
                mjw.step(rm, roll)
        wp.synchronize()
        q = roll.qpos.numpy().reshape(K, nq); v = roll.qvel.numpy().reshape(K, nv)
        r_, p_, yw_ = rpy(q[:, 3], q[:, 4], q[:, 5], q[:, 6])
        J = (W_VX*(v[:, 0]-gp.vx)**2 + W_UP*(r_*r_+p_*p_) + W_RATE*(v[:, 3]**2+v[:, 4]**2)
             + W_YAW*yw_*yw_ + W_Y*q[:, 1]**2 + W_H*np.maximum(0, zref-q[:, 2])**2
             + W_VZ*np.maximum(0, -v[:, 2])**2 + 40.0*(np.maximum(0, v[:, 0]-gp.vx)**2+np.maximum(0, p_)**2))
        J += ((q[:, 2] < z_fall) | (np.abs(r_) > 0.7) | (np.abs(p_) > 0.7)) * 300.0
        w = np.exp(-(J-J.min())/0.10); w /= w.sum()+1e-9
        nom_res = np.clip((w[:, None]*deltas).sum(0), -res_max, res_max)
        set_ctrl(real, 1, (gait(phase, tss)+nom_res)[None, :])
        for _ in range(sub):
            mjw.step(rm, real)
        phase += omega * dt_ctrl
        rq = real.qpos.numpy().reshape(nq)
        traj.append(rq.copy())
        rr, rp, _ = rpy(rq[3], rq[4], rq[5], rq[6])
        if rq[2] < z_fall or abs(rr) > 1.0 or abs(rp) > 1.0:
            fell_at = tss; print(f"[render] FELL @ {tss:.1f}s"); break
    xfin = traj[-1][0]
    print(f"[render] {'FELL @%.1fs'%fell_at if fell_at else 'UPRIGHT'} | walked x={xfin:+.2f} m in {len(traj)*dt_ctrl:.1f}s")

    # --- render the recorded REAL physics states ---
    print(f"[render] rendering {len(traj)} states -> frames...")
    W, Hh = 640, 480
    rnd = mujoco.Renderer(mjm, Hh, W)
    cam = mujoco.MjvCamera(); cam.azimuth = 110; cam.elevation = -16; cam.distance = 3.4
    every = max(1, int(round((1.0/dt_ctrl)/a.fps)))
    fdir = REPO / "_scratch/foot_redesign/frames"; fdir.mkdir(parents=True, exist_ok=True)
    for f in fdir.glob("*.png"):
        f.unlink()
    dd = mujoco.MjData(mjm)
    fi = 0
    for i in range(0, len(traj), every):
        dd.qpos[:] = traj[i]; mujoco.mj_forward(mjm, dd)
        cam.lookat[:] = [traj[i][0], traj[i][1], 0.5]
        rnd.update_scene(dd, cam)
        mujoco.imageio_imwrite = None
        img = rnd.render()
        import PIL.Image
        PIL.Image.fromarray(img).save(fdir / f"f{fi:05d}.png")
        fi += 1
    print(f"[render] {fi} frames; encoding mp4...")
    out = a.out
    subprocess.run(["ffmpeg", "-y", "-framerate", str(a.fps), "-i", str(fdir/"f%05d.png"),
                    "-c:v", "libx264", "-pix_fmt", "yuv420p", "-vf", "scale=640:480", out],
                   check=True, capture_output=True)
    print(f"[render] WROTE {out}")


if __name__ == "__main__":
    main()
