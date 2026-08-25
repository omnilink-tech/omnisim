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

"""omniarm6_toss -- the Skill Factory's fully-implemented reference recipe.

Learns "toss the held cube into a bin at (x, y)" for the OmniArm 6, standing
on the shipped toss-Shadowing machinery (docs/developer/omniarm6-toss-shadowing.md):

  design   : the designed sagittal swing is played through the SELF-LIMITING
             OmniArm 6 MuJoCo model (make_omniarm6_mjcf.py: forcerange = URDF effort,
             joint damping = effort/vel_limit, so every state is torque- AND
             velocity-feasible by construction). Aiming is release-instant
             selection over the swing's ballistic release map. A bin off the
             sagittal plane is aimed by a constant base yaw (joint1) -- a yaw
             rotation about gravity leaves the swing dynamics identical.
  validate : the REAL gates from ghost_verifier.verify_arm -- peak joint
             velocity ratio, peak torque ratio via inverse dynamics on the
             damping-zeroed "true" arm, open-loop tracking drift, ballistic
             consistency, and REACHES-THE-BIN -- plus URDF joint-range checks
             on the full output trajectory. An unreachable bin FAILS here.
  train    : REAL learning -- REINFORCE (numpy, the omniarm6_grasp_train pattern)
             over 7 swing parameters (windup/release pose deltas, swing
             duration, continuous release instant), objective = landing error
             at the bin, under DOMAIN RANDOMIZATION (payload mass 0.16-0.24 kg
             x release-timing jitter +/-8 ms). Every candidate is a full
             MuJoCo rollout per DR mass; nothing is canned. ~3 min default.
  certify  : N=5 held-out replays (fresh mass + timing draws, exact-mass model
             compiles) must EACH land within 5 cm; plus the verify_arm dynamic
             gates re-run on the trained swing. Fails honestly if unmet.

Output: learned_skill.json -- a joint trajectory at the bridge control dt
(16 ms = the chat world's basicTimeStep) with the release mapped to a timed
gripper "open" event, replayable by the OmniArm 6 chat bridge.
"""

from __future__ import annotations

import io
import json
import math
import os
import sys
import time
from contextlib import redirect_stdout

import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
_SHADOWING = os.path.join(REPO, "projects", "policies", "research", "shadowing")
_SCRIPTS_DEV = os.path.join(REPO, "scripts", "dev")
for _p in (REPO, _SHADOWING, _SCRIPTS_DEV):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import generate_omniarm6_toss as gt  # noqa: E402  (TossGhostGenerator, release_map, ...)
import make_omniarm6_mjcf as mk  # noqa: E402  (build_spec, _urdf_limits)
from ghost_verifier import verify_arm  # noqa: E402

from learn_runner import StageFailure  # noqa: E402

MJCF = os.path.join(REPO, "projects", "robots", "omnisim", "omniarm6", "mjcf",
                    "omniarm6_throw.mjcf.xml")
URDF = os.path.join(REPO, "projects", "robots", "omnisim", "omniarm6", "omniarm6.urdf")

JOINTS = list(gt.JOINTS)
BIN_RIM_Z = gt.BIN_RIM_Z          # 0.20 m -- the generator/verifier landing height
SIM_DT = gt.SIM_DT                # 2 ms physics substep (must match the ghost, see doc)
GHOST_DT = 0.01                   # control grid the swing is generated on

# Launch pose this recipe generates its swing from. It was the OmniArm 6
# chat-bridge home pose when the skill was trained; the bridge has since moved
# to [-0.1732, 0.1855, 0.7417, 0.0, 2.2144, 0.0] (omnilink_arm_bridge/
# _arm_configs.py OMNIARM6["home_pose"]). Held here deliberately -- the trained
# trajectory is relative to this pose, so changing it invalidates the ghost.
HOME_POSE = [0.0, -0.6, 1.2, 0.0, -1.0, 0.0]

# Output-trajectory timeline (s).
T_GRIP = 0.5                      # hold home while the gripper closes on the cube
T_RAMP = 2.0                      # smoothstep home -> windup
T_SETTLE = 0.3                    # settle at windup before the swing
T_TAIL = 0.4                      # hold the follow-through pose at the end

# ── training configuration (the demo-certified defaults) ─────────────
N_PARAM = 7
# theta -> physical scales: [windup j2, j3, j5, release j2, j5, swing-duration, release-time]
SCALES = np.array([0.25, 0.25, 0.25, 0.20, 0.20, 0.12, 0.05])
DR_MASSES = (0.16, 0.176, 0.192, 0.208, 0.224, 0.24)   # kg, payload DR grid
JITTER_S = 0.008                  # release-timing jitter, +/- (trains timing robustness)
MASS_RANGE = (0.16, 0.24)         # certify draws from the same physical range
SIGMA0, SIGMA1 = 0.35, 0.10       # exploration anneal
LR = 0.35                         # step on the normalized policy gradient
CERT_N = 5
CERT_BAR_CM = 5.0
POSE_MARGIN = 0.05                # rad clearance kept from URDF joint range


def _smoothstep(a: float) -> float:
    a = min(1.0, max(0.0, a))
    return a * a * (3.0 - 2.0 * a)


def _generator_for_mass(mass_kg: float) -> "gt.TossGhostGenerator":
    """A TossGhostGenerator on the self-limiting model rebuilt at a DR payload
    mass (spec -> xml string -> model; no file round-trip)."""
    xml = mk.build_spec(payload_mass=float(mass_kg)).to_xml()
    gen = gt.TossGhostGenerator.__new__(gt.TossGhostGenerator)
    gen.m = mujoco.MjModel.from_xml_string(xml)
    gen.m.opt.timestep = SIM_DT
    gen.d = mujoco.MjData(gen.m)
    gen.sim_dt = SIM_DT
    gen.bid = mujoco.mj_name2id(gen.m, mujoco.mjtObj.mjOBJ_BODY, "payload")
    return gen


class TossRecipe:
    name = "omniarm6_toss"
    verb = "toss"

    def __init__(self, params: dict, emit, out_dir: str):
        self.emit = emit
        self.out_dir = out_dir
        self.bin_x = float(params.get("bin_x", 1.3))
        self.bin_y = float(params.get("bin_y", 0.0))
        self.seed = int(params.get("seed", 0))
        self.dt_out = float(params.get("dt_ms", 16.0)) / 1000.0
        self.iters = int(params.get("iters", 200))
        self.batch = int(params.get("batch", 24))
        self.r = math.hypot(self.bin_x, self.bin_y)
        self.yaw = math.atan2(self.bin_y, self.bin_x)
        if self.r < 0.2:
            raise ValueError(f"bin at ({self.bin_x}, {self.bin_y}) is inside the arm base")
        self.rng = np.random.default_rng(self.seed)
        self.limits = mk._urdf_limits(URDF)   # {joint: (lo, hi, effort, vel)}
        # filled by the stages
        self.gen = None                # nominal-mass generator
        self.g_design = None           # designed ghost rollout
        self.k_design = None           # designed release grid index
        self.frontier = None
        self.design_mets = {}
        self.gates = {}
        self.best_theta = np.zeros(N_PARAM)
        self.best_score_cm = None
        self.trained_s = 0.0
        self.cert = {}
        self.g_final = None
        self.t_rel_final = None

    # ── swing parameterization ───────────────────────────────────────
    def _poses(self, theta):
        """theta -> (windup, release, t_release_key, t_total), URDF-range clamped."""
        w = list(gt.WINDUP)
        r = list(gt.RELEASE)
        w[1] += theta[0] * SCALES[0]
        w[2] += theta[1] * SCALES[1]
        w[4] += theta[2] * SCALES[2]
        r[1] += theta[3] * SCALES[3]
        r[4] += theta[4] * SCALES[4]
        for pose in (w, r):
            for i, jn in enumerate(JOINTS):
                lo, hi, _e, _v = self.limits[jn]
                pose[i] = min(hi - POSE_MARGIN, max(lo + POSE_MARGIN, pose[i]))
        t_release = min(0.85, max(0.35, gt.T_RELEASE + theta[5] * SCALES[5]))
        t_total = t_release + 0.40
        return w, r, t_release, t_total

    def _rollout(self, generator, theta):
        w, r, t_rel_key, t_total = self._poses(theta)
        return generator.generate(windup=w, release=r, follow=gt.FOLLOW,
                                  t_hold=gt.T_HOLD, t_release=t_rel_key,
                                  t_total=t_total, dt=GHOST_DT)

    @staticmethod
    def _interp_state(g, t):
        """Linear-interpolated (tcp, tcpvel) at continuous swing time t."""
        f = t / g["dt"]
        n = g["tcp"].shape[0]
        k = int(f)
        if k >= n - 1:
            return g["tcp"][-1], g["tcpvel"][-1]
        a = f - k
        return (g["tcp"][k] * (1 - a) + g["tcp"][k + 1] * a,
                g["tcpvel"][k] * (1 - a) + g["tcpvel"][k + 1] * a)

    def _landing_err_cm(self, g, t_rel):
        """Landing error (cm) of a release at continuous time t_rel, in the
        sagittal training frame (bin at x=self.r, y=0). None-safe: an invalid
        release state returns a large honest penalty."""
        p, v = self._interp_state(g, t_rel)
        if v[0] <= 0.3 or p[2] < 0.4:
            return 300.0
        land = gt.ballistic_landing(p, v)
        if land is None:
            return 300.0
        return 100.0 * math.hypot(land[0] - self.r, land[1])

    def _t_rel(self, theta):
        _w, _r, t_rel_key, t_total = self._poses(theta)
        t_nom = self.k_design * GHOST_DT + (t_rel_key - gt.T_RELEASE)
        return min(t_total - 0.02, max(gt.T_HOLD, t_nom + theta[6] * SCALES[6]))

    def _objective_cm(self, theta, generators, jitters):
        """Robust landing cost over the DR set: one full MuJoCo rollout per DR
        mass, each scored at several release-timing jitters (jitter is a cheap
        post-processing of the rollout), aggregated as mean + 0.5*worst so the
        objective tracks the certify bar (EVERY replay must land inside), plus
        a velocity-feasibility penalty. jitters: (n_mass, k) seconds."""
        t_rel = self._t_rel(theta)
        errs, velr = [], 0.0
        for gen_m, jrow in zip(generators, jitters):
            g = self._rollout(gen_m, theta)
            velr = max(velr, gt.feasibility(g))
            for jit in jrow:
                errs.append(self._landing_err_cm(g, t_rel + float(jit)))
        cost = float(np.mean(errs)) + 0.5 * float(np.max(errs)) \
            + 400.0 * max(0.0, velr - 1.0)
        return cost, velr

    # ── stage: design ────────────────────────────────────────────────
    def design(self):
        self.emit.stage("design", "running",
                        f"designing reference throw for bin at "
                        f"({self.bin_x:.2f}, {self.bin_y:.2f}) m  (range {self.r:.2f} m, "
                        f"yaw {math.degrees(self.yaw):+.1f} deg)", 2)
        if not os.path.exists(MJCF):
            buf = io.StringIO()
            with redirect_stdout(buf):
                mk.generate()
            self.emit.log("regenerated self-limiting OmniArm 6 MJCF (forcerange = URDF "
                          "effort, damping = effort/vel_limit)")
        self.gen = gt.TossGhostGenerator(mjcf=MJCF, sim_dt=SIM_DT)
        g = self.gen.generate()
        rmap = gt.release_map(g)
        if not rmap:
            raise StageFailure("design", "designed swing produced no valid forward "
                               "release -- swing keyframes need redesign")
        self.frontier = max(r[1] for r in rmap)
        k, land_x, spd, ang, rel_z = min(rmap, key=lambda r: abs(r[1] - self.r))
        self.g_design, self.k_design = g, k
        velr = gt.feasibility(g)
        self.design_mets = dict(release_speed=spd, release_angle_deg=ang,
                                release_z=rel_z, predicted_land=land_x,
                                frontier=self.frontier, vel_ratio=velr)
        detail = (f"release |v|={spd:.2f} m/s @ {ang:+.1f} deg (z={rel_z:.2f} m, "
                  f"t={k * GHOST_DT:.2f} s) -> predicted landing {land_x:.3f} m vs "
                  f"target {self.r:.2f} m; feasible frontier {self.frontier:.2f} m; "
                  f"peak vel ratio {velr:.2f}")
        self.emit.stage("design", "pass", detail, 10)

    # ── stage: validate ──────────────────────────────────────────────
    def _save_ghost(self, g, k, path, land):
        save = {kk: vv for kk, vv in g.items() if kk != "joints"}
        save["joints"] = np.array(g["joints"])
        save["release_k"] = k
        save["bin_x"] = self.r
        save["land"] = np.array(land)
        save["max_range"] = self.frontier
        save["vel_limits"] = gt.VEL_LIMITS
        save["bin_rim_z"] = BIN_RIM_Z
        save["kin_reach_x"] = gt.KIN_REACH_X
        save["sim_dt"] = SIM_DT
        np.savez(path, **save)
        return path

    def _run_verify_arm(self, g, k):
        land = gt.ballistic_landing(g["tcp"][k], g["tcpvel"][k])
        if land is None:
            land = (0.0, 0.0, 0.0)
        path = os.path.join(self.out_dir, "_ghost_check.npz")
        self._save_ghost(g, k, path, land)
        gnpz = np.load(path, allow_pickle=True)
        buf = io.StringIO()
        with redirect_stdout(buf):
            passed, score, mets = verify_arm(gnpz, MJCF, SIM_DT, False)
        gnpz.close()
        return passed, score, mets

    def validate(self):
        self.emit.stage("validate", "running",
                        "running feasibility gates (velocity / torque inverse-dyn / "
                        "open-loop tracking / reaches-the-bin / URDF joint ranges)", 12)
        passed, score, mets = self._run_verify_arm(self.g_design, self.k_design)

        # URDF joint-range gate over the FULL output trajectory (incl. base yaw
        # aim and the home->windup ramp -- everything the bridge will replay).
        range_viol = []
        for i, jn in enumerate(JOINTS):
            lo, hi, _e, _v = self.limits[jn]
            qcol = self.g_design["q"][:, i] + (self.yaw if i == 0 else 0.0)
            qmin = min(float(qcol.min()), HOME_POSE[i], self.yaw if i == 0 else HOME_POSE[i])
            qmax = max(float(qcol.max()), HOME_POSE[i], self.yaw if i == 0 else HOME_POSE[i])
            if qmin < lo or qmax > hi:
                range_viol.append(f"{jn} [{qmin:.2f},{qmax:.2f}] outside [{lo:.2f},{hi:.2f}]")

        vel_r, tor_r = mets["vel_ratio"], mets["tor_ratio"]
        drift, reach_err = mets["max_drift"], mets["reach_err"]
        self.gates["validate"] = dict(vel_ratio=round(vel_r, 3), tor_ratio=round(tor_r, 3),
                                      track_drift_rad=round(drift, 4),
                                      design_reach_err_m=round(reach_err, 4),
                                      joint_ranges_ok=not range_viol, score=round(score, 3))
        if range_viol:
            raise StageFailure("validate", "URDF joint-range gate FAIL: " + "; ".join(range_viol))
        if not passed:
            if reach_err >= 0.05:
                raise StageFailure(
                    "validate",
                    f"REACHES-THE-BIN gate FAIL: best release lands {reach_err:.2f} m from "
                    f"the bin at {self.r:.2f} m -- beyond the feasible throw frontier "
                    f"({self.frontier:.2f} m) for the rated joints (194/102/34 N.m, "
                    f"3.14/3.49 rad/s). This bin is unreachable; not training.")
            if vel_r > 1.05:
                raise StageFailure("validate", f"VELOCITY gate FAIL: peak joint speed "
                                   f"{vel_r:.2f}x the URDF rated limit")
            if tor_r >= 1.0:
                raise StageFailure("validate", f"TORQUE gate FAIL: inverse dynamics needs "
                                   f"{tor_r:.2f}x the URDF effort limit")
            raise StageFailure("validate", f"open-loop tracking drift {drift * 1000:.0f} mrad "
                               f"exceeds the 200 mrad gate")
        self.emit.stage("validate", "pass",
                        f"all gates PASS: velocity {vel_r:.2f}/1.00, torque {tor_r:.2f}/1.00 "
                        f"(inverse dyn), open-loop drift {drift * 1000:.0f} mrad, landing "
                        f"{reach_err * 100:.1f} cm from bin, joint ranges OK "
                        f"(feasibility score {score:.2f})", 18)

    # ── stage: train ─────────────────────────────────────────────────
    def train(self):
        t0 = time.perf_counter()
        self.emit.stage("train", "running",
                        f"REINFORCE over {N_PARAM} swing params, batch {self.batch}, "
                        f"{self.iters} iters, DR: mass {DR_MASSES[0]:.2f}-{DR_MASSES[-1]:.2f} kg "
                        f"x release jitter +/-{JITTER_S * 1000:.0f} ms "
                        f"({self.batch * len(DR_MASSES)} rollouts/iter)", 20)

        # one generator per DR mass (self-limiting model recompiled per mass)
        generators = [_generator_for_mass(m_kg) for m_kg in DR_MASSES]

        # fixed held-in evaluation grid: every mass scored at -J/0/+J jitter --
        # deterministic, so scores are comparable across iterations
        eval_jitters = np.tile(np.array([-JITTER_S, 0.0, JITTER_S]), (len(DR_MASSES), 1))

        mu = np.zeros(N_PARAM)
        best_theta, best_cm = mu.copy(), None
        n_roll = 0
        # design baseline (theta = 0): training starts from the validated design
        base_cm, _ = self._objective_cm(mu, generators, eval_jitters)
        best_cm = base_cm
        n_roll += len(DR_MASSES)
        self.emit.stage("train", "running",
                        f"iter 0/{self.iters}  design-baseline robust landing err "
                        f"{base_cm:.2f} cm (mean + 0.5*worst over DR grid)", 20)
        for it in range(1, self.iters + 1):
            sigma = SIGMA0 + (SIGMA1 - SIGMA0) * min(1.0, it / (0.7 * self.iters))
            eps = self.rng.standard_normal((self.batch, N_PARAM))
            thetas = np.clip(mu + sigma * eps, -2.0, 2.0)
            rewards = np.empty(self.batch)
            for b in range(self.batch):
                jit = self.rng.uniform(-JITTER_S, JITTER_S, (len(DR_MASSES), 3))
                cost, _velr = self._objective_cm(thetas[b], generators, jit)
                rewards[b] = -cost
                n_roll += len(DR_MASSES)
            adv = (rewards - rewards.mean()) / (rewards.std() + 1e-6)
            mu = np.clip(mu + LR * (adv[:, None] * eps).mean(axis=0), -2.0, 2.0)

            # deterministic eval of the current mean policy on the FIXED grid;
            # the returned skill is the best-ever mean policy, never a regression
            cur_cm, cur_vel = self._objective_cm(mu, generators, eval_jitters)
            n_roll += len(DR_MASSES)
            if cur_cm < best_cm:
                best_cm, best_theta = cur_cm, mu.copy()
            if it % 5 == 0 or it == self.iters:
                pct = 20.0 + 70.0 * it / self.iters
                self.emit.stage(
                    "train", "running",
                    f"iter {it}/{self.iters}  robust landing err {cur_cm:.2f} cm  "
                    f"best {best_cm:.2f} cm (design {base_cm:.2f})  sigma {sigma:.2f}  "
                    f"vel ratio {cur_vel:.2f}", pct)

        self.best_theta, self.best_score_cm = best_theta, best_cm
        self.trained_s = time.perf_counter() - t0
        if best_cm is None or best_cm > 100.0:
            raise StageFailure("train", f"optimization did not converge: best robust "
                               f"landing error {best_cm:.1f} cm after {self.iters} iters")
        self.emit.stage("train", "pass",
                        f"converged: robust landing err {best_cm:.2f} cm "
                        f"(design baseline {base_cm:.2f} cm) over the DR grid "
                        f"({n_roll} MuJoCo rollouts, {self.trained_s:.0f} s)", 90)

    # ── stage: certify ───────────────────────────────────────────────
    def certify(self):
        self.emit.stage("certify", "running",
                        f"bar: landing err <= {CERT_BAR_CM:.0f} cm on each of {CERT_N} "
                        f"held-out replays (fresh mass + timing draws) + dynamic gates "
                        f"on the trained swing", 92)
        rng = np.random.default_rng(self.seed + 1000)   # held-out stream
        t_rel = self._t_rel(self.best_theta)
        errs = []
        for i in range(CERT_N):
            m_kg = float(rng.uniform(*MASS_RANGE))
            jit = float(rng.uniform(-JITTER_S, JITTER_S))
            g = self._rollout(_generator_for_mass(m_kg), self.best_theta)
            e = self._landing_err_cm(g, t_rel + jit)
            errs.append(e)
            self.emit.log(f"certify replay {i + 1}/{CERT_N}: mass {m_kg:.3f} kg, "
                          f"release jitter {jit * 1000:+.1f} ms -> landing err {e:.2f} cm")

        # dynamic gates re-run on the trained nominal-mass swing
        self.g_final = self._rollout(self.gen, self.best_theta)
        self.t_rel_final = t_rel
        k_near = int(round(t_rel / GHOST_DT))
        k_near = min(self.g_final["q"].shape[0] - 1, max(0, k_near))
        _passed, score, mets = self._run_verify_arm(self.g_final, k_near)
        dyn_ok = (mets["vel_ratio"] <= 1.05 and mets["tor_ratio"] < 1.0
                  and mets["max_drift"] < 0.20 and mets["land_err"] < 0.02)
        replays_ok = all(e <= CERT_BAR_CM for e in errs)
        self.cert = dict(replay_err_cm=[round(e, 2) for e in errs],
                         bar_cm=CERT_BAR_CM, n=CERT_N,
                         mean_err_cm=round(float(np.mean(errs)), 2),
                         max_err_cm=round(float(np.max(errs)), 2),
                         trained_vel_ratio=round(mets["vel_ratio"], 3),
                         trained_tor_ratio=round(mets["tor_ratio"], 3),
                         trained_track_drift_rad=round(mets["max_drift"], 4),
                         dynamic_gates_ok=dyn_ok)
        if not replays_ok:
            raise StageFailure("certify",
                               f"bar NOT met: replay errors {[f'{e:.1f}' for e in errs]} cm "
                               f"(bar {CERT_BAR_CM:.0f} cm each)")
        if not dyn_ok:
            raise StageFailure("certify",
                               f"trained swing broke a dynamic gate: velocity "
                               f"{mets['vel_ratio']:.2f}, torque {mets['tor_ratio']:.2f}, "
                               f"drift {mets['max_drift'] * 1000:.0f} mrad")
        self.emit.stage("certify", "pass",
                        f"{CERT_N}/{CERT_N} replays within {CERT_BAR_CM:.0f} cm "
                        f"(mean {np.mean(errs):.2f}, max {np.max(errs):.2f} cm); trained "
                        f"swing gates OK (vel {mets['vel_ratio']:.2f}, torque "
                        f"{mets['tor_ratio']:.2f})", 98)

    # ── output assembly ──────────────────────────────────────────────
    def export(self) -> str:
        g = self.g_final
        q = g["q"][:, :6]
        n = q.shape[0]
        swing_T = (n - 1) * GHOST_DT

        def swing_q(t):
            f = min(max(t, 0.0) / GHOST_DT, n - 1)
            k = int(f)
            if k >= n - 1:
                return q[-1]
            a = f - k
            return q[k] * (1 - a) + q[k + 1] * a

        home = np.array(HOME_POSE)
        windup = q[0].copy()
        yaw_vec = np.array([self.yaw, 0, 0, 0, 0, 0])
        t_swing0 = T_GRIP + T_RAMP + T_SETTLE
        total_T = t_swing0 + swing_T + T_TAIL
        traj = []
        t = 0.0
        while t <= total_T + 1e-9:
            if t < T_GRIP:
                qt = home
            elif t < T_GRIP + T_RAMP:
                a = _smoothstep((t - T_GRIP) / T_RAMP)
                qt = home + a * (windup + yaw_vec - home)
            elif t < t_swing0:
                qt = windup + yaw_vec
            else:
                qt = swing_q(min(t - t_swing0, swing_T)) + yaw_vec
            traj.append([round(float(x), 5) for x in qt])
            t += self.dt_out

        t_open = t_swing0 + self.t_rel_final
        p, v = self._interp_state(g, self.t_rel_final)
        skill = {
            "verb": self.verb,
            "kind": "joint_traj",
            "dt": self.dt_out,
            "traj": traj,
            "gripper_events": [[0.0, "close"], [round(t_open, 4), "open"]],
            "meta": {
                "recipe": self.name,
                "robot": "omniarm6",
                "trained_s": round(self.trained_s, 1),
                "score": round(self.cert["mean_err_cm"] / 100.0, 4),
                "expects": (f"cube held at gripper at start; bin at "
                            f"({self.bin_x:.2f}, {self.bin_y:.2f}) m from the arm base, "
                            f"rim z~{BIN_RIM_Z:.2f} m"),
                "bin": [self.bin_x, self.bin_y],
                "seed": self.seed,
                "release": {"t_s": round(t_open, 4),
                            "speed_mps": round(float(np.linalg.norm(v)), 3),
                            "angle_deg": round(math.degrees(math.atan2(v[2], v[0])), 1),
                            "tcp_z_m": round(float(p[2]), 3)},
                "theta": [round(float(x), 4) for x in self.best_theta],
                "gates": {**self.gates.get("validate", {}), "certify": self.cert},
            },
        }
        path = os.path.join(self.out_dir, "learned_skill.json")
        with open(path, "w", encoding="utf-8") as fh:
            # verify_arm metrics arrive as numpy scalars; .item() maps them to
            # native bool/float so the skill file is plain JSON
            json.dump(skill, fh, indent=1,
                      default=lambda o: o.item() if hasattr(o, "item") else str(o))
        # keep the trained ghost next to it for provenance / re-verification
        self._save_ghost(self.g_final, int(round(self.t_rel_final / GHOST_DT)),
                         os.path.join(self.out_dir, "trained_ghost.npz"),
                         gt.ballistic_landing(p, v) or (0.0, 0.0, 0.0))
        return path

    # ── entry point the runner calls ─────────────────────────────────
    def run(self) -> str:
        self.design()
        self.validate()
        self.train()
        self.certify()
        return self.export()

    def summary_line(self, wall_s: float) -> str:
        return (f"learned '{self.verb}' to bin ({self.bin_x:.2f}, {self.bin_y:.2f}) m: "
                f"certified {self.cert['n']}/{self.cert['n']} replays within "
                f"{self.cert['bar_cm']:.0f} cm (mean {self.cert['mean_err_cm']:.2f} cm), "
                f"trained {self.trained_s:.0f} s, total {wall_s:.0f} s")
