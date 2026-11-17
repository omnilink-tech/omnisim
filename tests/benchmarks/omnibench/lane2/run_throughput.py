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

"""OmniBench Lane 2 — three-tier throughput (SPEC.md, credibility checklist).

Model: the Go2 quadruped MJCF that OmniSim's own Newton backend EXPORTS
(projects/policies/research/training/mjcf/go2_newton.xml, an
OMNISIM_NEWTON_SAVE_MJCF dump — kp=250/kv=6 PD pairs, 24 actuators, the exact
model the deploy SolverMuJoCo steps). Zero translation confound: the raw
baseline and the Newton-solver tier step the SAME model description.

Tiers (each its own row; never summed or conflated):

  A  `sim_only` on TWO engines:
     - `mujoco-warp-raw`  : MjModel -> mjw.put_model/put_data -> mjw.step
       (CUDA-graph captured like the production trainer). The baseline.
     - `omnisim-newton`   : newton.ModelBuilder.add_mjcf(same file) ->
       SolverMuJoCo(use_mujoco_cpu=False) substep loop, collision ON — the
       engine's embedded deploy-solver path (probe_newton_batch_throughput
       methodology; NO cuda graph, conservative lower bound, recorded).
       Tier `Ag` is the SAME omnisim-newton path with warp CUDA-graph
       capture of the 8-substep period (capture_begin/capture_end, the raw
       tier's machinery) — the like-for-like partner of the graphed raw
       baseline; rows carry cuda_graph=true.
       NOTE this is the embedded solver in-process, not the full omnisim-bin;
       full-engine overhead is tier C's story.
     Actions: uniform random position targets inside each joint's range,
     redrawn EVERY control step (never idle). Idle-scene guard: mean active
     contact count per world must be > 0, sampled after the timed window.

  B  `sim_infer` (engine mujoco-warp-raw): tier A raw loop + the REAL Go2
     Shadowing champion (gpu_go2_shadow_main/policy.onnx, the skill-library
     verified artifact) in the loop — obs built exactly like the trainer's
     _build_obs_t (48-dim), actions decoded ghost(phase) + ff + 0.15*a.

  C  `sim_train` (engine omnisim-newton): steady-state env-steps/s of a SHORT
     in-engine PPO run through omnisim-bin via the existing
     run_quad_walk_rl.sh -> quad_walk_recipe.py (train == deploy bit-exact).
     Iterations/envs are CLI-configurable; this NEVER launches a long training.

One control step = 16 ms = 8 x 2 ms physics substeps (the deploy loop);
env-steps/s = batch * control_steps / wall — the same metric the trainers log.

Every row: SPEC JSON-lines schema + machine fingerprint
(projects/policies/common/env_fingerprint.py). OOM at a batch size is recorded
as a dropped row, never faked. Honesty rules from SPEC.md apply.

Run (from PowerShell — native warp/CUDA, same guidance as the sibling probes):

  # verification on this box (3060 6GB):
  python tests/benchmarks/omnibench/lane2/run_throughput.py --tiers raw,A --batch 256
  python tests/benchmarks/omnibench/lane2/run_throughput.py --tiers B --batch 256
  python tests/benchmarks/omnibench/lane2/run_throughput.py --tiers C --tierc-envs 256 --tierc-iters 15

  # the 4090 campaign:
  python tests/benchmarks/omnibench/lane2/run_throughput.py --tiers raw,A,B --batch 256 1024 4096
  python tests/benchmarks/omnibench/lane2/run_throughput.py --tiers C --tierc-envs 4096 --tierc-iters 30
"""
from __future__ import annotations

import argparse
import math
import os
import re
import subprocess
import sys
import time
import traceback
from pathlib import Path

LANE2 = Path(__file__).resolve().parent
REPO = LANE2.parents[3]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))
if str(LANE2) not in sys.path:      # a site-packages 'tests' pkg can shadow
    sys.path.insert(0, str(LANE2))  # the repo's namespace path — import direct

import _row  # noqa: E402

# ── contract constants (deploy loop; see gpu_mjwarp_go2_walk_trainer.py) ──
DT = 0.016                # one control step (OmniSim basicTimeStep)
SUBSTEPS = 8              # 8 x 2 ms — matches OMNISIM_NEWTON_SUBSTEPS=8
PHYS_DT = 0.002
NJ = 12
SEED = 42                 # SPEC global seed
TEST_NAME = "throughput_go2"

DEFAULT_MJCF = REPO / "projects/policies/research/training/mjcf/go2_newton.xml"
DEFAULT_ONNX = (REPO / "projects/policies/research/inference/policies/"
                       "gpu_go2_shadow_main/policy.onnx")
DEFAULT_GHOST = REPO / "projects/policies/ghosts/go2/go2_shadow_ghost_lut.json"

# Controller-order joint limits (FL,FR,RL,RR x hip,thigh,calf — trainer's).
JOINT_LIMITS_LO = [-1.0472, -0.5236, -2.7227] * 4
JOINT_LIMITS_HI = [+1.0472, +3.1316, -0.83776] * 4


def _log(msg: str) -> None:
    print(f"[lane2] {msg}", flush=True)


def _is_oom(exc: BaseException) -> bool:
    s = f"{type(exc).__name__}: {exc}".lower()
    return any(t in s for t in ("out of memory", "cuda_error_out_of_memory",
                                "cumem", "outofmemory", "alloc"))


def _dropped_row(engine: str, tier: str, batch: int, reason: str, extra=None):
    return _row.make_row(
        test=TEST_NAME, engine=engine, dt_ms=DT * 1000.0,
        metrics={"tier": tier, "batch": batch, "status": "dropped",
                 "reason": reason, **(extra or {})},
        wall_ms_per_step=0.0, steps=0, sim_seconds=0.0,
        deviations=[f"batch={batch} dropped: {reason}"])


# ────────────────────────────────────────────────────────────────────
# Shared model plumbing (raw + sim_infer path)
# ────────────────────────────────────────────────────────────────────
class MjBundle:
    """The engine-exported Go2 MjModel + controller-order index maps.

    Joint classification is copied from the production trainer
    (gpu_mjwarp_go2_walk_trainer.BatchedSpotWalkEnv.__init__) so the mapping is
    identical to what trained the champion — names in the dump are anonymous.
    """

    def __init__(self, mjcf: Path, self_collision: str):
        import mujoco
        import numpy as np
        self.mujoco, self.np = mujoco, np
        self.mjcf = str(mjcf)
        self.self_collision = self_collision
        m = mujoco.MjModel.from_xml_path(self.mjcf)
        m.opt.timestep = PHYS_DT
        self.model_modified = False
        if self_collision == "on":
            # The engine's own export structurally disables robot self-collision
            # (pairwise <exclude> for every body pair AND contype/conaffinity
            # masks that zero robot-robot pairs). "on" re-enables it: this is a
            # MODIFIED model, recorded as such — the verbatim export is the
            # zero-confound default.
            for g in range(m.ngeom):
                if m.geom_bodyid[g] != 0:      # robot geoms (body 0 = world)
                    m.geom_contype[g] |= 1
                    m.geom_conaffinity[g] |= 1
            # MjModel cannot drop <exclude> pairs post-compile; strip them from
            # the XML instead.
            import re as _re
            xml = Path(self.mjcf).read_text(encoding="utf-8")
            xml2 = _re.sub(r"<contact>.*?</contact>", "", xml, flags=_re.S)
            xml2 = xml2.replace('contype="4" conaffinity="2147483643"',
                                'contype="5" conaffinity="2147483647"')
            m = mujoco.MjModel.from_xml_string(xml2)
            m.opt.timestep = PHYS_DT
            self.model_modified = True
        self.m = m
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        self.d0 = d

        # ── controller-order maps (trainer's classification, verbatim logic) ──
        qpos_idx, qvel_idx, ctrl_pos_idx = ([0] * NJ, [0] * NJ, [0] * NJ)
        label = {}
        k_hinge = 0
        for j in range(m.njnt):
            if m.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE:
                continue
            ax = m.jnt_axis[j]
            rng_j = m.jnt_range[j]
            pos = d.xpos[m.jnt_bodyid[j]]
            leg = ("FL" if (pos[0] > 0 and pos[1] > 0) else
                   "FR" if (pos[0] > 0) else
                   "RL" if (pos[1] > 0) else "RR")
            joint = ("hip_x" if abs(ax[0]) > 0.5 else
                     "knee" if rng_j[1] < 0.0 else "hip_y")
            label[(leg, joint)] = (m.jnt_qposadr[j], m.jnt_dofadr[j], k_hinge)
            k_hinge += 1
        for ci, leg in enumerate(("FL", "FR", "RL", "RR")):
            for cj, joint in enumerate(("hip_x", "hip_y", "knee")):
                qadr, vadr, hidx = label[(leg, joint)]
                i = ci * 3 + cj
                qpos_idx[i] = int(qadr)
                qvel_idx[i] = int(vadr)
                ctrl_pos_idx[i] = 2 * hidx   # ctrl interleaved [pos, vel]/hinge
        assert m.nu == 2 * NJ, f"expected 24 actuators, got {m.nu}"
        self.qpos_idx, self.qvel_idx, self.ctrl_pos_idx = qpos_idx, qvel_idx, ctrl_pos_idx

    def model_stats(self) -> dict:
        m = self.m
        return {"nbody": int(m.nbody), "ngeom": int(m.ngeom), "nu": int(m.nu),
                "nq": int(m.nq), "nv": int(m.nv), "nexclude": int(m.nexclude),
                "timestep": float(m.opt.timestep),
                "self_collision": self.self_collision,
                "model_modified": self.model_modified}

    def seed_qpos(self, joints=None, base_z: float = 0.34):
        np = self.np
        qp = self.d0.qpos.copy().astype(np.float64)
        qp[0:3] = [0.0, 0.0, base_z]
        qp[3:7] = [1.0, 0.0, 0.0, 0.0]
        if joints is not None:
            for i in range(NJ):
                qp[self.qpos_idx[i]] = float(joints[i])
        return qp


def _standing_pose():
    """Standing pose + body height from the repo's own gait module (the same
    nominal the trainer/champion used)."""
    from projects.policies.control.gait import go2_trot_gait as stg
    gp = stg.GaitParams()
    return stg.standing_pose(gp), float(gp.body_height)


class _RawSim:
    """Batched mujoco_warp stepping of the MjBundle with torch views + optional
    CUDA graph of the 8-substep control period (the trainer's exact machinery)."""

    def __init__(self, bundle: MjBundle, n: int, use_graph: bool):
        import warp as wp
        import mujoco_warp as mjw
        import torch
        self.wp, self.mjw, self.torch = wp, mjw, torch
        self.n = n
        self.bundle = bundle
        self.device = wp.get_device("cuda:0")
        self.tdev = torch.device("cuda:0")
        m, d0 = bundle.m, bundle.d0
        with wp.ScopedDevice(self.device):
            self.mw_m = mjw.put_model(m)
            # 128 (not the trainer's 64): random-flail actions hit more
            # simultaneous contacts + joint limits than a tame trot, and an
            # njmax=64 run printed "nefc overflow ... increase njmax to 72" —
            # dropped constraint rows would silently flatter the timing.
            njmax = int(os.environ.get("LANE2_NJMAX", "128"))
            nconmax = int(os.environ.get("LANE2_NCONMAX", "128"))
            self.njmax, self.nconmax = njmax, nconmax
            self.mw_d = mjw.put_data(m, d0, nworld=n, njmax=njmax, nconmax=nconmax)
        self.qpos_t = wp.to_torch(self.mw_d.qpos).view(n, m.nq)
        self.qvel_t = wp.to_torch(self.mw_d.qvel).view(n, m.nv)
        self.ctrl_t = wp.to_torch(self.mw_d.ctrl).view(n, m.nu)
        self.qpos_idx_t = torch.tensor(bundle.qpos_idx, dtype=torch.long, device=self.tdev)
        self.qvel_idx_t = torch.tensor(bundle.qvel_idx, dtype=torch.long, device=self.tdev)
        self.ctrl_pos_idx_t = torch.tensor(bundle.ctrl_pos_idx, dtype=torch.long, device=self.tdev)
        self.jl_lo_t = torch.tensor(JOINT_LIMITS_LO, device=self.tdev)
        self.jl_hi_t = torch.tensor(JOINT_LIMITS_HI, device=self.tdev)
        self.graph = None
        self.graph_wanted = use_graph
        if use_graph:
            try:
                with wp.ScopedDevice(self.device):
                    for _ in range(SUBSTEPS):
                        mjw.step(self.mw_m, self.mw_d)
                    wp.synchronize()
                    wp.capture_begin(force_module_load=False)
                    for _ in range(SUBSTEPS):
                        mjw.step(self.mw_m, self.mw_d)
                    self.graph = wp.capture_end()
            except Exception as e:
                _log(f"CUDA graph capture failed ({e}); falling back to direct step")
                self.graph = None
        self.graph_capture_failed = use_graph and self.graph is None

    def seed(self, qpos):
        torch, np = self.torch, self.bundle.np
        self.qpos_t.copy_(torch.tensor(np.asarray(qpos, dtype=np.float32),
                                       device=self.tdev).unsqueeze(0).expand(self.n, -1))
        self.qvel_t.zero_()
        self.ctrl_t.zero_()

    def set_targets(self, targets_t):
        """targets_t (n, 12) controller-order position targets -> ctrl."""
        self.ctrl_t.index_copy_(1, self.ctrl_pos_idx_t,
                                self.torch.clamp(targets_t, self.jl_lo_t, self.jl_hi_t))

    def control_step(self):
        wp, mjw = self.wp, self.mjw
        with wp.ScopedDevice(self.device):
            if self.graph is not None:
                wp.capture_launch(self.graph)
            else:
                for _ in range(SUBSTEPS):
                    mjw.step(self.mw_m, self.mw_d)

    def contacts_per_world(self, sample_steps: int = 32) -> float:
        """Mean active contacts per world, sampled over continuing steps AFTER
        the timed window (each read syncs, so it must not pollute the timing)."""
        wp = self.wp
        tot, k = 0.0, 0
        for _ in range(sample_steps):
            self.control_step()
            wp.synchronize()
            try:
                tot += float(self.mw_d.nacon.numpy()[0])
                k += 1
            except Exception:
                return float("nan")
        return (tot / k / self.n) if k else float("nan")

    def health(self) -> dict:
        import numpy as np
        qp = self.qpos_t.detach().cpu().numpy()
        bz = qp[:, 2]
        return {"finite": bool(np.isfinite(qp).all()),
                "base_z_mean": float(np.nanmean(bz)),
                "base_z_min": float(np.nanmin(bz)),
                "base_z_max": float(np.nanmax(bz))}


def _finish_row(engine, tier, batch, wall_s, steps, extra_metrics, deviations,
                machine=None):
    env_sps = batch * steps / wall_s
    ms_per_ctrl = wall_s / steps * 1000.0
    metrics = {
        "tier": tier, "batch": batch,
        "env_steps_per_s": round(env_sps),
        "phys_steps_per_s": round(env_sps * SUBSTEPS),
        "ms_per_control_step": round(ms_per_ctrl, 4),
        "substeps": SUBSTEPS,
        **extra_metrics,
    }
    return _row.make_row(
        test=TEST_NAME, engine=engine, dt_ms=DT * 1000.0, metrics=metrics,
        wall_ms_per_step=round(ms_per_ctrl, 4), steps=steps,
        sim_seconds=round(steps * DT, 3), deviations=deviations,
        machine=machine)


# ────────────────────────────────────────────────────────────────────
# Tier A baseline — engine `mujoco-warp-raw`, tier `sim_only`
# ────────────────────────────────────────────────────────────────────
def run_raw(bundle: MjBundle, batch: int, steps: int, warmup: int,
            use_graph: bool) -> dict:
    import torch
    sim = _RawSim(bundle, batch, use_graph)
    nominal, body_h = _standing_pose()
    sim.seed(bundle.seed_qpos(nominal, base_z=body_h + 0.03))
    gen = torch.Generator(device="cuda:0"); gen.manual_seed(SEED)
    lo, hi = sim.jl_lo_t, sim.jl_hi_t

    def random_targets():
        u = torch.rand(batch, NJ, device=sim.tdev, generator=gen)
        return lo + u * (hi - lo)

    for _ in range(warmup):
        sim.set_targets(random_targets())
        sim.control_step()
    sim.wp.synchronize()
    t0 = time.perf_counter()
    for _ in range(steps):
        sim.set_targets(random_targets())
        sim.control_step()
    sim.wp.synchronize()
    wall = time.perf_counter() - t0

    contacts = sim.contacts_per_world()
    health = sim.health()
    deviations = [
        "actions = uniform random position targets in joint range on the 12 "
        "position actuators, redrawn every control step; the 12 paired "
        "velocity-damping actuators are held at 0 (the trainer/deploy "
        "convention for this PD-pair export)",
        "self-collision structurally OFF in the engine's own exported model "
        "(78 pairwise excludes + contype/conaffinity masks) — the verbatim "
        "export is used for the zero-translation-confound baseline"
        if bundle.self_collision == "export" else
        "MODIFIED model: robot self-collision enabled (excludes stripped, "
        "contype/conaffinity opened) — NOT the verbatim engine export",
        "contact count sampled over 32 control steps after the timed window "
        "(reads sync the GPU)",
    ]
    if sim.graph_capture_failed:
        deviations.append("CUDA graph capture FAILED — direct mjw.step loop timed")
    if not (contacts > 0):
        deviations.append(f"IDLE-SCENE GUARD FAILED: mean contacts/world={contacts}")
    extra = {
        **bundle.model_stats(),
        "mean_contacts_per_world": None if math.isnan(contacts) else round(contacts, 2),
        "idle_guard_ok": bool(contacts > 0),
        "cuda_graph": sim.graph is not None,
        "njmax": sim.njmax, "nconmax": sim.nconmax,
        **health,
        "actions": "uniform_random_joint_range",
    }
    return _finish_row("mujoco-warp-raw", "sim_only", batch, wall, steps, extra,
                       deviations)


# ────────────────────────────────────────────────────────────────────
# Tier A — engine `omnisim-newton` (embedded deploy solver), tier `sim_only`
# ────────────────────────────────────────────────────────────────────
def _target_q(obj):
    """The position-target array, across newton's 1.5 rename.

    newton 1.5 REMOVED `joint_target_pos` on the ModelBuilder, the Model and
    `Control` in favour of `joint_target_q`; the old name is now a
    `RemovedAttribute` descriptor that RAISES on access. This whole tier went
    `status=dropped` on the first 1.5 run — a row with `env_steps_per_s: None`
    inside an rc=0 campaign, i.e. the OmniSim arm of lane 2 silently vanished
    while the raw mujoco-warp arm kept reporting.

    Resolved per call via getattr so ONE source drives both runtimes, matching
    `omnisim_newton_runtime._ctl_target_pos`. The rename is layout-preserving
    for us — `joint_target_q` is DOF-shaped exactly like the old field unless
    `newton.use_coord_layout_targets` is enabled, and it defaults False — and
    every index below is a DOF index. Do not enable that global without
    reindexing this function.
    """
    a = getattr(obj, "joint_target_q", None)
    if a is None:
        a = getattr(obj, "joint_target_pos", None)
    if a is None:
        raise AttributeError(
            "%s exposes neither joint_target_q (newton >=1.3) nor "
            "joint_target_pos (<1.5)" % type(obj).__name__)
    return a


def run_newton(mjcf: Path, batch: int, steps: int, warmup: int,
               self_collision: str, use_graph: bool = False) -> dict:
    import numpy as np
    import warp as wp
    import newton
    rng = np.random.default_rng(SEED)
    SUB_DT = DT / SUBSTEPS

    # newton 1.2's MJCF importer rejects single-element solref (the engine dump
    # writes solref="0.02"); pad with the MuJoCo default dampratio 1.0 —
    # semantics-preserving (MJCF pads unset trailing elements with defaults).
    xml = Path(mjcf).read_text(encoding="utf-8")
    xml_fixed, n_pad = re.subn(r'solref="([-0-9.eE]+)"', r'solref="\1 1"', xml)
    import tempfile
    tmp = Path(tempfile.gettempdir()) / "lane2_go2_newton_solref.xml"
    tmp.write_text(xml_fixed, encoding="utf-8")

    one = newton.ModelBuilder()
    one.add_mjcf(str(tmp))
    ndof = len(one.joint_target_ke)
    nq1 = len(one.joint_q)
    assert ndof == 6 + NJ and nq1 == 7 + NJ, \
        f"unexpected dof layout from add_mjcf: ndof={ndof} nq={nq1}"
    # Deploy-parity PD on the 12 hinge dofs (go2_newton.xml kp=250/kv=6; the
    # launcher's OMNISIM_NEWTON_TARGET_KE/KD for go2) — same recipe as
    # probe_newton_batch_throughput.py. Hinge dofs follow the 6 free dofs, in
    # MJCF file order == controller order (FL,FR,RL,RR x hip,thigh,calf).
    pv = int(newton.JointTargetMode.POSITION_VELOCITY)
    nominal, body_h = _standing_pose()
    for i in range(NJ):
        d = 6 + i
        one.joint_target_ke[d] = 250.0
        one.joint_target_kd[d] = 6.0
        one.joint_target_mode[d] = pv
        _target_q(one)[d] = float(nominal[i])
        one.joint_q[7 + i] = float(nominal[i])
    one.joint_q[2] = body_h + 0.03
    # NOTE: no add_ground_plane() — the engine's MJCF export already carries
    # its own ground plane geom and add_mjcf imports it (verified: 14 shapes
    # per world; adding another produced a duplicate coincident plane).

    main_b = newton.ModelBuilder()
    main_b.replicate(one, world_count=batch, spacing=(3.0, 3.0, 0.0))
    model = main_b.finalize()
    # njmax/nconmax match the raw path's caps (random-flail actions overflow
    # the default: "nefc overflow - please increase njmax to 66" at 1024).
    njmax = int(os.environ.get("LANE2_NJMAX", "128"))
    nconmax = int(os.environ.get("LANE2_NCONMAX", "128"))
    solver = newton.solvers.SolverMuJoCo(model, use_mujoco_cpu=False,
                                         njmax=njmax, nconmax=nconmax)
    state_a, state_b = model.state(), model.state()
    control = model.control()
    contacts = model.contacts() if hasattr(model, "contacts") else None
    newton.eval_fk(model, model.joint_q, model.joint_qd, state_a)

    tp0 = _target_q(model).numpy().copy()            # (batch*ndof,) layout
    tp = tp0.reshape(batch, ndof).copy()
    lo = np.array(JOINT_LIMITS_LO, dtype=tp.dtype)
    hi = np.array(JOINT_LIMITS_HI, dtype=tp.dtype)

    def set_random_targets():
        tp[:, 6:6 + NJ] = lo + rng.random((batch, NJ)) * (hi - lo)
        _target_q(control).assign(tp.reshape(-1))

    def control_substeps():
        nonlocal state_a, state_b
        for _ in range(SUBSTEPS):
            state_a.clear_forces()
            if contacts is not None:
                model.collide(state_a, contacts)
                solver.step(state_a, state_b, control, contacts, SUB_DT)
            else:
                solver.step(state_a, state_b, control, None, SUB_DT)
            state_a, state_b = state_b, state_a
        # SUBSTEPS is even, so (state_a, state_b) end up bound to the SAME
        # warp arrays each period — the launch pattern is period-invariant,
        # which is what makes CUDA-graph capture of one period replayable.
        assert SUBSTEPS % 2 == 0

    # ── optional CUDA-graph capture of one 8-substep control period, the
    #    same machinery the raw tier uses (_RawSim.__init__): warm one
    #    period so modules/allocs exist, then capture_begin/capture_end. ──
    graph = None
    graph_capture_failed = False
    if use_graph:
        try:
            with wp.ScopedDevice(wp.get_device("cuda:0")):
                set_random_targets()
                control_substeps()          # warmup: load modules, allocs
                wp.synchronize()
                wp.capture_begin(force_module_load=False)
                control_substeps()
                graph = wp.capture_end()
        except Exception as e:
            _log(f"newton CUDA graph capture failed ({e}); "
                 "falling back to direct step loop")
            graph = None
            graph_capture_failed = True

    def control_step():
        if graph is not None:
            wp.capture_launch(graph)
        else:
            control_substeps()

    for _ in range(warmup):
        set_random_targets()
        control_step()
    wp.synchronize()
    t0 = time.perf_counter()
    for _ in range(steps):
        set_random_targets()
        control_step()
    wp.synchronize()
    wall = time.perf_counter() - t0

    # Best-effort contact count on the Newton contacts object.
    ncon_pw = float("nan")
    if contacts is not None:
        for attr in ("rigid_contact_count", "contact_count", "count"):
            v = getattr(contacts, attr, None)
            if v is None:
                continue
            try:
                arr = v.numpy()
                ncon_pw = float(np.asarray(arr).sum()) / batch
                break
            except Exception:
                continue
    body_q = state_a.body_q.numpy()
    finite = bool(np.isfinite(body_q).all())
    # Per-world base z: body_q rows are rigid bodies (13/world + no ground body);
    # base body is the first body of each world.
    nb_pw = body_q.shape[0] // batch
    base_z = body_q.reshape(batch, nb_pw, -1)[:, 0, 2]

    if graph is not None:
        graph_note = (
            "CUDA-graph capture ON: one 8-substep control period "
            "(model.collide + SolverMuJoCo.step x8) captured via warp "
            "capture_begin/capture_end and replayed with capture_launch — "
            "the same machinery as the raw mujoco-warp baseline (closes the "
            "graphed-vs-ungraphed methodology asymmetry of the v0 rows)")
    elif graph_capture_failed:
        graph_note = ("CUDA-graph capture FAILED on the Newton SolverMuJoCo "
                      "path — direct step loop timed (see console for the "
                      "capture exception)")
    else:
        graph_note = (
            "no CUDA-graph capture on the Newton SolverMuJoCo path (matches "
            "probe_newton_batch_throughput.py methodology) — conservative "
            "lower bound; the raw baseline row records whether it used a graph")
    deviations = [
        graph_note,
        "this row steps OmniSim's embedded deploy solver (newton SolverMuJoCo) "
        "in-process; it is NOT the full omnisim-bin engine loop — that is tier C",
        "actions = uniform random position targets in joint range each control "
        "step (position-velocity PD, kp=250/kv=6, deploy parity)",
        "self-collision: model built by newton add_mjcf from the engine's own "
        "export, which structurally disables robot self-collision (excludes + "
        "contact masks); robot-ground contacts ON",
        f"single-element solref padded to MuJoCo-default '(t, 1)' in {n_pad} "
        "geoms to work around newton 1.2's MJCF importer (semantics-preserving)",
    ]
    idle_ok = (ncon_pw > 0) if not math.isnan(ncon_pw) else None
    if idle_ok is None:
        # Secondary guard: the robot must be resting near the ground, not in
        # free fall / fallen through the plane.
        idle_ok = bool(np.isfinite(base_z).all() and base_z.mean() > 0.02
                       and base_z.mean() < 1.0)
        deviations.append("Newton contacts object exposes no readable contact "
                          "count on this build — idle guard fell back to a "
                          "base-height sanity check (robot resting on ground)")
    if not idle_ok:
        deviations.append("IDLE-SCENE GUARD FAILED")
    extra = {
        "nbody": int(getattr(model, "body_count", 0)) // max(batch, 1),
        "shape_count_total": int(len(model.shape_type.numpy()))
        if hasattr(model, "shape_type") else None,
        # NOTE: newton's contacts object counts allocated CANDIDATE contact
        # points (broad-phase pairs x points), not active narrow-phase contacts
        # — numerically larger than the raw path's nacon; comparable only as a
        # >0 idle guard, never across engines.
        "mean_contact_candidates_per_world": None if math.isnan(ncon_pw) else round(ncon_pw, 2),
        "idle_guard_ok": bool(idle_ok),
        "finite": finite,
        "base_z_mean": float(base_z.mean()),
        "cuda_graph": graph is not None,
        "cuda_graph_wanted": bool(use_graph),
        "path": "newton-solvermujoco-standalone",
        "njmax": njmax, "nconmax": nconmax,
        "kp": 250.0, "kd": 6.0,
        "actions": "uniform_random_joint_range",
        "self_collision": "export",
    }
    return _finish_row("omnisim-newton", "sim_only", batch, wall, steps, extra,
                       deviations)


# ────────────────────────────────────────────────────────────────────
# Tier B — engine `mujoco-warp-raw`, tier `sim_infer` (real champion policy)
# ────────────────────────────────────────────────────────────────────
def _load_policy(onnx_path: Path, backend: str):
    """Returns (infer(obs_np(n,48))->act_np(n,12), descriptor dict) or raises."""
    if backend == "onnx":
        import onnxruntime as ort
        avail = ort.get_available_providers()
        providers = (["CUDAExecutionProvider"] if "CUDAExecutionProvider" in avail
                     else []) + ["CPUExecutionProvider"]
        sess = ort.InferenceSession(str(onnx_path), providers=providers)
        iname = sess.get_inputs()[0].name
        used = sess.get_providers()[0]

        def infer(obs_np):
            return sess.run(None, {iname: obs_np})[0]
        return infer, {"policy": onnx_path.relative_to(REPO).as_posix(),
                       "policy_backend": "onnx", "onnx_provider": used,
                       "proxy_policy": False}
    if backend == "torch":
        import torch
        pt = onnx_path.with_name("policy.pt")
        mod = torch.jit.load(str(pt), map_location="cuda:0")
        mod.eval()

        def infer(obs_np):
            with torch.no_grad():
                t = torch.from_numpy(obs_np).cuda()
                return mod(t).cpu().numpy()
        return infer, {"policy": pt.relative_to(REPO).as_posix(),
                       "policy_backend": "torchscript-cuda",
                       "proxy_policy": False}
    raise ValueError(backend)


def run_infer(bundle: MjBundle, batch: int, steps: int, warmup: int,
              use_graph: bool, onnx_path: Path, ghost_path: Path,
              policy_backend: str) -> dict:
    import json as _json
    import numpy as np
    import torch

    # ── policy (REAL champion; if it cannot be loaded the caller records
    #    tier B as not_run — we never silently substitute a random MLP) ──
    infer, pdesc = _load_policy(onnx_path, policy_backend)

    # ── ghost lut (the champion's action decode: ghost(phase)+ff+0.15*a) ──
    gd = _json.loads(Path(ghost_path).read_text(encoding="utf-8"))
    leg_lut = torch.tensor(gd["leg_lut"], dtype=torch.float32, device="cuda:0")
    ff_lut = (torch.tensor(gd["ffdq_lut"], dtype=torch.float32, device="cuda:0")
              if "ffdq_lut" in gd else None)
    nb = int(gd["nb"])
    freq = float(gd["freq"])
    gait = gd.get("gait", {})
    body_h = float(gait.get("body_height", 0.30))
    res_scale = 0.15                       # QUAD_RES_SCALE the champion trained with
    phase_dt = 2.0 * math.pi * freq * DT

    sim = _RawSim(bundle, batch, use_graph)
    from projects.policies.control.gait import go2_trot_gait as stg
    nominal_np = stg.standing_pose(stg.GaitParams()).astype(np.float32)
    nominal = torch.tensor(nominal_np, device=sim.tdev)
    sim.seed(bundle.seed_qpos(gd["leg_lut"][0], base_z=body_h + 0.03))

    phase = torch.zeros(batch, device=sim.tdev)
    last_action = torch.zeros(batch, NJ, device=sim.tdev)
    prev_q = sim.qpos_t.index_select(1, sim.qpos_idx_t).clone()
    wz_cmd = torch.zeros(batch, 1, device=sim.tdev)

    def lut(table, ph):
        x = torch.remainder(ph, 2.0 * math.pi) / (2.0 * math.pi) * nb
        b0 = torch.floor(x)
        f = (x - b0).unsqueeze(1)
        i0 = b0.long() % nb
        i1 = (i0 + 1) % nb
        return table[i0] * (1 - f) + table[i1] * f

    def build_obs():
        """The trainer's _build_obs_t, verbatim math (48-dim)."""
        nonlocal prev_q
        qp, qv = sim.qpos_t, sim.qvel_t
        vlin = qv[:, 0:3]
        vang = qv[:, 3:6]
        w, x, y, z = qp[:, 3], qp[:, 4], qp[:, 5], qp[:, 6]
        gx = -2 * (x * z - w * y)
        gy = -2 * (y * z + w * x)
        gz = -(1 - 2 * (x * x + y * y))
        pg = torch.stack([gx, gy, gz], dim=1)
        q = qp.index_select(1, sim.qpos_idx_t)
        qd = (q - prev_q) / DT
        prev_q = q.clone()
        gaitv = torch.stack([torch.sin(phase), torch.cos(phase)], dim=1)
        obs = torch.cat([vlin, vang, pg, q - nominal, qd, last_action, gaitv,
                         wz_cmd], dim=1)
        obs = torch.nan_to_num(obs, nan=0.0, posinf=10.0, neginf=-10.0)
        return torch.clamp(obs, -10.0, 10.0)

    def one_control_step():
        nonlocal phase, last_action
        obs_np = build_obs().detach().cpu().numpy().astype(np.float32)
        act_np = infer(obs_np)
        a = torch.clamp(torch.from_numpy(np.asarray(act_np)).to(sim.tdev), -1, 1)
        center = lut(leg_lut, phase)
        if ff_lut is not None:
            center = center + lut(ff_lut, phase)   # GHOST-FF (ramp r=1, steady)
        sim.set_targets(center + res_scale * a)
        sim.control_step()
        phase = torch.remainder(phase + phase_dt, 2.0 * math.pi)
        last_action = a

    for _ in range(warmup):
        one_control_step()
    sim.wp.synchronize()
    t0 = time.perf_counter()
    for _ in range(steps):
        one_control_step()
    sim.wp.synchronize()
    wall = time.perf_counter() - t0

    contacts = sim.contacts_per_world()
    health = sim.health()
    vx_mean = float(sim.qvel_t[:, 0].mean().item())
    deviations = [
        "obs built with the trainer's _build_obs_t math (48-dim); actions "
        "decoded ghost(phase)+ffdq+0.15*a (champion's deploy decode); the "
        "per-episode stride ramp is held at steady state (r=1) and there are "
        "no episode resets during the timed window",
        "self-collision structurally OFF in the engine's own exported model "
        "(verbatim export)" if bundle.self_collision == "export" else
        "MODIFIED model: robot self-collision enabled",
    ]
    if pdesc.get("onnx_provider") == "CPUExecutionProvider":
        deviations.append("policy inference on CPU (onnxruntime has no CUDA EP "
                          "in this environment) — obs GPU->CPU + infer + "
                          "action CPU->GPU are all inside the timed loop, as "
                          "deploy does it")
    if sim.graph_capture_failed:
        deviations.append("CUDA graph capture FAILED — direct mjw.step loop timed")
    if not (contacts > 0):
        deviations.append(f"IDLE-SCENE GUARD FAILED: mean contacts/world={contacts}")
    extra = {
        **bundle.model_stats(),
        **pdesc,
        "ghost": Path(ghost_path).relative_to(REPO).as_posix(),
        "res_scale": res_scale,
        "mean_contacts_per_world": None if math.isnan(contacts) else round(contacts, 2),
        "idle_guard_ok": bool(contacts > 0),
        "cuda_graph": sim.graph is not None,
        "njmax": sim.njmax, "nconmax": sim.nconmax,
        "mean_vx": round(vx_mean, 3),
        **health,
    }
    return _finish_row("mujoco-warp-raw", "sim_infer", batch, wall, steps,
                       extra, deviations)


# ────────────────────────────────────────────────────────────────────
# Tier C — engine `omnisim-newton` (full omnisim-bin), tier `sim_train`
# ────────────────────────────────────────────────────────────────────
def _bash_exe() -> str:
    """Git Bash, explicitly (shared resolver in common/engine_launch.py). On
    Windows a bare 'bash' can resolve to WSL's bash (System32), which mangles
    repo paths (/mnt/o/...) and runs the wrong python — measured failure mode,
    not hypothetical. LANE2_BASH / OMNIBENCH_BASH override.

    Tier C deliberately goes through the production run_quad_walk_rl.sh
    launcher (train == deploy bit-exact), not engine_launch.launch_once —
    that script owns the engine child on this path."""
    from common import engine_launch  # _row put omnibench on sys.path
    return engine_launch.bash_exe()


def run_train(envs: int, iters: int, dur_s: int, tag: str,
              extra_kv: list[str]) -> dict:
    """Short in-engine PPO run via the existing recipe; parses the recipe's own
    'env-steps/s (window N)' log lines for the steady-state figure."""
    launcher = REPO / "projects/policies/training/run_quad_walk_rl.sh"
    res_log = REPO / f"_scratch/foot_redesign/{tag}_rl.txt"
    sim_log = REPO / f"_scratch/foot_redesign/{tag}_omnisim.txt"
    console = REPO / f"_scratch/foot_redesign/{tag}_console.txt"
    kvs = [
        "QUAD_ROBOT=go2",
        f"QUAD_ENVS={envs}",
        f"QUAD_ITERS={iters}",
        "QUAD_FAST_RESET=1",
        "MPC_NJMAX=64", "MPC_NCONMAX=64",
        # Save the throwaway smoke policy into scratch — NEVER the recipe's
        # default runs/gpu_go2_inengine/policy.pt (a benchmark must not clobber
        # a locally-kept champion checkpoint).
        f"RES_POLICY={(REPO / '_scratch/foot_redesign').as_posix()}/{tag}_policy.pt",
        f"STARTUP_S={max(300, dur_s // 2)}", f"STALL_S={max(300, dur_s // 2)}",
    ] + list(extra_kv)
    # Relative POSIX path + cwd=REPO: a backslashed absolute path is mangled by
    # bash into "O:omnisim..." (exit 127, measured).
    rel = launcher.relative_to(REPO).as_posix()
    cmd = [_bash_exe(), rel, str(dur_s), tag, "train", "headless"] + kvs
    # Headless Linux (RunPod): run_quad_walk_rl.sh -> headless_runner.py
    # launches omnisim-bin WITHOUT the xvfb-run wrapper that
    # common/engine_launch.py gives the lane-1/3 direct launches. On the
    # 2026-07-24 pod campaign the engine died at Qt-platform init before the
    # trainer started ("could not connect to display" -> SIGABRT rc=-6,
    # started=False, both tier-C attempts) while every xvfb-wrapped lane-1
    # engine run on the same pod passed. Wrap the whole launcher so every
    # child inherits the virtual display.
    xvfb_wrapped = False
    if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
        import shutil
        xvfb = shutil.which("xvfb-run")
        if xvfb:
            cmd = [xvfb, "-a"] + cmd
            xvfb_wrapped = True
        else:
            _log("tier C WARNING: no DISPLAY and no xvfb-run on PATH - the "
                 "engine will abort at Qt platform init (apt install xvfb)")
    _log(f"tier C: {' '.join(cmd)}")
    t0 = time.perf_counter()
    proc = subprocess.run(cmd, cwd=str(REPO), capture_output=True, text=True,
                          timeout=dur_s + 600)
    wall = time.perf_counter() - t0
    log_text = res_log.read_text(encoding="utf-8", errors="replace") if res_log.exists() else ""

    # quad_walk_recipe logs: "... {fps:,.0f} env-steps/s (window {win:,.0f})"
    win = [float(m.group(2).replace(",", ""))
           for m in re.finditer(r"([\d,]+)\s+env-steps/s \(window ([\d,]+)\)", log_text)]
    cum = [float(m.group(1).replace(",", ""))
           for m in re.finditer(r"([\d,]+)\s+env-steps/s \(window", log_text)]
    started = bool(re.search(r"WALK-QUAD start", log_text))
    done = bool(re.search(r'"state": "DONE"', (res_log.with_suffix(".txt.status")
                                               .read_text(encoding="utf-8", errors="replace")
                                               if res_log.with_suffix(".txt.status").exists()
                                               else "")))
    if not win:
        reason = ("in-engine trainer produced no env-steps/s window samples; "
                  f"started={started} rc={proc.returncode}; see {console}")
        # The launcher console (FAIL line + engine stderr tail) is the actual
        # diagnostic — proc.stdout alone missed the Qt-abort root cause on
        # the 2026-07-24 pod campaign. Put both tails in the row.
        con_tail = ""
        if console.exists():
            con_tail = console.read_text(encoding="utf-8",
                                         errors="replace")[-1200:]
        row = _dropped_row("omnisim-newton", "sim_train", envs, reason,
                           extra={"iters": iters, "rl_log": str(res_log),
                                  "xvfb_wrapped": xvfb_wrapped,
                                  "console_tail": (proc.stdout or "")[-800:],
                                  "console_file_tail": con_tail})
        row["metrics"]["status"] = "not_run"
        return row

    steady = sorted(win[1:] or win)[len(win[1:] or win) // 2]   # median, first window excluded
    machine = _row.fingerprint(engine_log_path=str(sim_log))
    metrics = {
        "tier": "sim_train", "batch": envs,
        "env_steps_per_s": round(steady),
        "env_steps_per_s_windows": [round(w) for w in win],
        "env_steps_per_s_cumulative_last": round(cum[-1]) if cum else None,
        "iters": iters,
        "substeps": SUBSTEPS,
        "recipe": "projects/policies/training/quad_walk_recipe.py (in-engine, "
                  "through omnisim-bin)",
        "mode": "legacy-trot (no QUAD_GHOST)" if not any(
            k.startswith("QUAD_GHOST") for k in extra_kv) else "shadowing",
        "trainer_done": done,
        "launcher_rc": proc.returncode,
        "xvfb_wrapped": xvfb_wrapped,
        "world": "projects/policies/research/worlds/go2_walk_deploy.omniworld",
    }
    deviations = [
        "steady-state = median of the trainer's windowed env-steps/s samples, "
        "first window excluded (warmup/compile); full training NOT run — "
        f"{iters} PPO iterations only",
        "env-steps/s here includes PPO rollout + update + engine round-trip "
        "(the trainer's own metric): this is the tier-C number, not comparable "
        "to tier A stepping rates except as the OmniSim/raw overhead ratio "
        "at the SAME batch size",
        "QUAD_FAST_RESET=1 (the measured production fast path)",
    ]
    ms_per_ctrl = envs / steady * 1000.0  # per batched control step
    return _row.make_row(
        test=TEST_NAME, engine="omnisim-newton", dt_ms=DT * 1000.0,
        metrics=metrics, wall_ms_per_step=round(ms_per_ctrl, 4),
        steps=0, sim_seconds=round(wall, 1), deviations=deviations,
        machine=machine)


# ────────────────────────────────────────────────────────────────────
# CLI
# ────────────────────────────────────────────────────────────────────
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tiers", default="raw,A,B",
                    help="comma list of raw|A|Ag|B|C (raw = mujoco-warp-raw "
                         "sim_only baseline; A = omnisim-newton embedded-solver "
                         "sim_only, ungraphed v0 methodology; Ag = same but "
                         "with CUDA-graph capture, the like-for-like partner "
                         "of the graphed raw baseline; B = sim_infer on raw; "
                         "C = in-engine sim_train)")
    ap.add_argument("--batch", type=int, nargs="+", default=[256, 1024, 4096],
                    help="batch sizes for tiers raw/A/B (OOM at a size is "
                         "recorded as a dropped row, then the sweep continues)")
    ap.add_argument("--steps", type=int, default=300,
                    help="timed control steps per point (each = 8 substeps)")
    ap.add_argument("--warmup", type=int, default=20)
    ap.add_argument("--mjcf", type=Path, default=DEFAULT_MJCF)
    ap.add_argument("--self-collision", choices=["export", "on"], default="export",
                    help="'export' = the engine's verbatim exported model "
                         "(self-collision structurally off — recorded as a "
                         "deviation); 'on' = modified model with robot "
                         "self-collision enabled (recorded as modified)")
    ap.add_argument("--no-graph", action="store_true",
                    help="disable CUDA-graph capture on the raw/B path (for a "
                         "like-for-like ungraphed comparison with tier A newton)")
    ap.add_argument("--onnx", type=Path, default=DEFAULT_ONNX)
    ap.add_argument("--ghost", type=Path, default=DEFAULT_GHOST)
    ap.add_argument("--policy-backend", choices=["onnx", "torch"], default="onnx")
    ap.add_argument("--tierc-envs", type=int, default=256,
                    help="K parallel rollout worlds for tier C (256 fits the "
                         "6 GB 3060; the 4090 campaign uses 4096+)")
    ap.add_argument("--tierc-iters", type=int, default=15,
                    help="PPO iterations for tier C (short probe, never a "
                         "real training)")
    ap.add_argument("--tierc-dur", type=int, default=1800,
                    help="wall-clock cap (s) handed to the launcher watchdog")
    ap.add_argument("--tierc-kv", nargs="*", default=[],
                    help="extra KEY=VALUE env pairs passed to run_quad_walk_rl.sh")
    ap.add_argument("--tag", default=None, help="tier C run tag (default auto)")
    ap.add_argument("--out", type=Path, default=None,
                    help="JSONL output (default lane2/results/throughput.jsonl)")
    args = ap.parse_args(argv)

    tiers = [t.strip() for t in args.tiers.split(",") if t.strip()]
    out = args.out or _row.default_out("throughput.jsonl")
    if Path(out).is_dir():
        # A directory --out crashed emit() with IsADirectoryError on the
        # 2026-07-24 pod campaign — AFTER a 30-min tier-C run, losing the row.
        out = Path(out) / "throughput.jsonl"
        _log(f"--out is a directory; writing {out}")
    rows = []

    def emit(row):
        rows.append(row)
        _row.write_row(out, row)
        m = row["metrics"]
        _log(f"ROW engine={row['engine']} tier={m.get('tier')} "
             f"batch={m.get('batch')} env-steps/s={m.get('env_steps_per_s')} "
             f"status={m.get('status', 'ok')}")

    gpu_tiers = [t for t in tiers if t in ("raw", "A", "Ag", "B")]
    if gpu_tiers:
        try:
            import warp as wp
            wp.init()
        except Exception as e:
            _log(f"SKIP GPU tiers: warp unavailable ({type(e).__name__}: {e})")
            gpu_tiers = []

    bundle = None
    if any(t in gpu_tiers for t in ("raw", "B")):
        bundle = MjBundle(args.mjcf, args.self_collision)
        _log(f"MjModel: {bundle.model_stats()}")

    for batch in args.batch:
        for tier in gpu_tiers:
            engine = "omnisim-newton" if tier in ("A", "Ag") else "mujoco-warp-raw"
            tname = {"raw": "sim_only", "A": "sim_only", "Ag": "sim_only",
                     "B": "sim_infer"}[tier]
            try:
                if tier == "raw":
                    emit(run_raw(bundle, batch, args.steps, args.warmup,
                                 not args.no_graph))
                elif tier in ("A", "Ag"):
                    if args.self_collision == "on":
                        emit(_dropped_row(engine, tname, batch,
                                          "self-collision 'on' variant not "
                                          "implemented on the newton add_mjcf "
                                          "path (raw-only)"))
                    else:
                        emit(run_newton(args.mjcf, batch, args.steps,
                                        args.warmup, args.self_collision,
                                        use_graph=(tier == "Ag")))
                elif tier == "B":
                    emit(run_infer(bundle, batch, args.steps, args.warmup,
                                   not args.no_graph, args.onnx, args.ghost,
                                   args.policy_backend))
            except Exception as e:
                traceback.print_exc()
                reason = f"{type(e).__name__}: {e}"
                if _is_oom(e):
                    reason = f"OOM on this GPU: {reason}"
                row = _dropped_row(engine, tname, batch, reason)
                if tier == "B" and not _is_oom(e):
                    row["metrics"]["status"] = "not_run"
                emit(row)

    if "C" in tiers:
        tag = args.tag or f"lane2c_{time.strftime('%m%d_%H%M%S')}"
        try:
            emit(run_train(args.tierc_envs, args.tierc_iters, args.tierc_dur,
                           tag, args.tierc_kv))
        except Exception as e:
            traceback.print_exc()
            row = _dropped_row("omnisim-newton", "sim_train", args.tierc_envs,
                               f"{type(e).__name__}: {e}")
            row["metrics"]["status"] = "not_run"
            emit(row)

    _log(f"wrote {len(rows)} row(s) -> {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
