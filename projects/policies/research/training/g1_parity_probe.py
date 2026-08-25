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

"""Deterministic open-loop physics probe -- TRAINER side.

Builds the legs-only G1 the TRAINER way (newton ``add_urdf``, primitive feet --
reusing build_g1_native_prim's URDF strip) and applies ``SPEC.probe_targets(k)``
each control step under the EXACT deploy solver (``SolverMuJoCo`` + mjwarp),
dumping a per-tick trace (joint angles + base pose). The DEPLOY side (the
``g1_parity_probe`` controller running inside omnisim-bin) applies the SAME
scripted targets and dumps the SAME trace schema; ``g1_parity_compare.py`` then
diffs the two.

WHY this is new: every prior G1 trainer<->deploy parity proof
(``g1_golden_parity.py``, ``test_g1_physics_spec_conformance.py``) compared a
Python-built trainer model against ``g1_deploy_runtime.py`` -- a Python *extract*
of the C++ source -- and stepped BOTH through the same in-process solver. NOTHING
ever stepped the real ``omnisim-bin`` binary on a ``.wbt`` and compared the
trajectory. This probe closes that hole: it is a deterministic, no-RL controlled
experiment (identical inputs -> compare outputs), so any divergence is pure
physics/stepping, not policy or durability.

Lanes (``--static-base`` is the chaos-free numerical-equality lane and the
default; a free floating base is an unstable inverted pendulum whose chaos would
confound a real physics gap with chaotic amplification):

  --static-base  weld the floating base (``add_urdf(floating=False)``); the
                 deploy mirrors this with ``staticBase TRUE`` on the URDFRobot.
                 Joint trajectory is then the parity signal.
  --free-base    realistic floating base on the ground (qualitative; expect
                 chaos near the ~1.4 s G1 fall).
  --no-ground    drop the ground plane (pure articulation + gravity + joint PD,
                 no contact) -- isolates the articulation dynamics.

Run from a native Windows shell (PowerShell) so warp/CUDA initialises
(see reference_verify_newton_powershell). Output: a JSON trace at
``--out`` (default ``_scratch/parity/trainer_trace.json``).
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import warp as wp
import newton

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(_REPO))

from projects.policies.research.backends import g1_physics_spec as SPEC  # noqa: E402
from projects.policies.research.training.build_g1_native_prim import _build_prim_urdf_xml  # noqa: E402
from projects.policies.research.training.build_g1_native import NJ, SPAWN_Z  # noqa: E402

TRACE_SCHEMA = 1


def _quat_xyzw_to_rotmat(qx, qy, qz, qw):
    """Body-to-world rotation matrix (row-major 9) from a (x,y,z,w) quaternion --
    the SAME representation the deploy's getOrientation() returns, so the two
    traces are directly comparable."""
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return [
        1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy),
        2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
        2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy),
    ]


def build(static_base: bool, add_ground: bool, spawn_z: float,
          ke: float, kd: float, match_contact: bool = True):
    """Build ONE legs-only prim G1 the trainer way. Returns
    (model, dof0, qpos0) where dof0/qpos0 are the first ACTUATED indices (6/7 for
    a floating base, 0/0 when welded)."""
    mb = newton.ModelBuilder()
    if match_contact:
        # Match the deploy's contact stiffness/friction. The deploy
        # (g1_deploy_runtime.py) sets default_shape_cfg.ke/kd/mu BEFORE building so
        # every shape (foot boxes + ground) inherits them; the trainer previously
        # used Newton's defaults -> a real contact mismatch that showed up as a
        # slow base/joint drift in the free-base stand. SPEC is the single source.
        mb.default_shape_cfg.ke = SPEC.CONTACT_KE
        mb.default_shape_cfg.kd = SPEC.CONTACT_KD
        mb.default_shape_cfg.mu = SPEC.GROUND_MU
    urdf_xml = _build_prim_urdf_xml()
    mb.add_urdf(urdf_xml,
                xform=wp.transform((0.0, 0.0, spawn_z), (0.0, 0.0, 0.0, 1.0)),
                floating=(not static_base))
    dof0 = 0 if static_base else 6
    qpos0 = 0 if static_base else 7
    pv = int(newton.JointTargetMode.POSITION_VELOCITY)
    t0 = SPEC.probe_targets(0)
    for j in range(NJ):
        d = dof0 + j
        mb.joint_target_ke[d] = ke
        mb.joint_target_kd[d] = kd
        mb.joint_target_mode[d] = pv
        # effort: keep the URDF's per-joint effort limits (what the deploy uses);
        # do NOT override -> ankle 35 / knee 139 / hip 88 Nm match on both sides.
        mb.joint_target_q[d] = float(t0[j])
        # NOTE: do NOT seed joint_q -- spawn straight-legged (all-zero), exactly
        # like the deploy's natural spawn. The settle ramp (probe_settle_target)
        # then drives BOTH sides from this identical IC to the same PD equilibrium
        # before recording, so there is no launch-IC asymmetry to confound the diff.
    if add_ground:
        mb.add_ground_plane()
    return mb.finalize(), dof0, qpos0


def run(model, dof0, qpos0, *, static_base, spawn_z, n_ticks, settle_ticks,
        substeps, dt, seq, amp, period, record_settle=False):
    solver = newton.solvers.SolverMuJoCo(model, use_mujoco_cpu=False)
    state_a, state_b = model.state(), model.state()
    control = model.control()
    contacts = model.contacts() if hasattr(model, "contacts") else None
    newton.eval_fk(model, model.joint_q, model.joint_qd, state_a)
    sub_dt = dt / substeps

    def _apply_and_step(targets):
        nonlocal state_a, state_b
        tp = control.joint_target_q.numpy()
        tp[dof0:dof0 + NJ] = np.asarray(targets, dtype=tp.dtype)
        control.joint_target_q.assign(tp)
        for _ in range(substeps):
            state_a.clear_forces()
            if contacts is not None:
                model.collide(state_a, contacts)
                solver.step(state_a, state_b, control, contacts, sub_dt)
            else:
                solver.step(state_a, state_b, control, None, sub_dt)
            state_a, state_b = state_b, state_a

    ticks = []

    def _capture(k, target, phase):
        q = state_a.joint_q.numpy()[qpos0:qpos0 + NJ]
        if static_base:
            base_pos = [0.0, 0.0, float(spawn_z)]
            base_rot = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        else:
            bq = state_a.body_q.numpy()[0]
            base_pos = [float(bq[0]), float(bq[1]), float(bq[2])]
            base_rot = _quat_xyzw_to_rotmat(float(bq[3]), float(bq[4]),
                                            float(bq[5]), float(bq[6]))
        ticks.append({
            "k": k, "phase": phase,
            "target": [float(x) for x in target],
            "q": [float(x) for x in q.tolist()],
            "base_pos": base_pos, "base_rot": base_rot,
        })

    # SETTLE: ramp from the straight-leg spawn to the pose, then hold, converging
    # to the position-PD equilibrium -- identical recipe on the deploy. Recorded
    # only when record_settle (diagnostic: watch the transient from spawn).
    for s in range(settle_ticks):
        st = SPEC.probe_settle_target(s, settle_ticks)
        _apply_and_step(st)
        if record_settle:
            _capture(s - settle_ticks, st, "settle")

    for k in range(n_ticks):
        tk = SPEC.probe_targets(k, sequence=seq, amp=amp, period_ticks=period)
        _apply_and_step(tk)
        _capture(k, tk, "probe")
    return ticks


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default=str(_REPO / "_scratch/parity/trainer_trace.json"))
    grp = ap.add_mutually_exclusive_group()
    grp.add_argument("--static-base", dest="static_base", action="store_true")
    grp.add_argument("--free-base", dest="static_base", action="store_false")
    ap.set_defaults(static_base=SPEC.PROBE_STATIC_BASE)
    ap.add_argument("--no-ground", dest="add_ground", action="store_false")
    ap.set_defaults(add_ground=True)
    ap.add_argument("--ticks", type=int, default=SPEC.PROBE_DURATION_TICKS)
    ap.add_argument("--settle", type=int, default=SPEC.PROBE_SETTLE_TICKS)
    ap.add_argument("--sequence", default=SPEC.PROBE_SEQUENCE,
                    choices=["hold", "sinusoid"])
    ap.add_argument("--amp", type=float, default=SPEC.PROBE_SINE_AMP)
    ap.add_argument("--period", type=int, default=SPEC.PROBE_SINE_PERIOD_TICKS)
    ap.add_argument("--ke", type=float, default=SPEC.PROBE_KE)
    ap.add_argument("--kd", type=float, default=SPEC.PROBE_KD)
    ap.add_argument("--spawn-z", type=float, default=SPAWN_Z)
    ap.add_argument("--record-settle", action="store_true",
                    help="diagnostic: also record the settle transient")
    ap.add_argument("--no-contact-match", dest="match_contact",
                    action="store_false",
                    help="diagnostic: use Newton default contact (don't match deploy ke/kd/mu)")
    ap.set_defaults(match_contact=True)
    args = ap.parse_args(argv)

    wp.init()
    model, dof0, qpos0 = build(args.static_base, args.add_ground, args.spawn_z,
                               args.ke, args.kd, match_contact=args.match_contact)
    ticks = run(model, dof0, qpos0,
                static_base=args.static_base, spawn_z=args.spawn_z,
                n_ticks=args.ticks, settle_ticks=args.settle,
                substeps=SPEC.SUBSTEPS, dt=SPEC.DT,
                seq=args.sequence, amp=args.amp, period=args.period,
                record_settle=args.record_settle)

    out = {
        "schema": TRACE_SCHEMA,
        "side": "trainer",
        "meta": {
            "construction": "add_urdf",
            "urdf": "g1_legs_omnisim.urdf (prim feet)",
            "sequence": args.sequence, "amp": args.amp, "period": args.period,
            "settle_ticks": args.settle,
            "static_base": args.static_base, "add_ground": args.add_ground,
            "ke": args.ke, "kd": args.kd, "spawn_z": args.spawn_z,
            "substeps": SPEC.SUBSTEPS, "dt": SPEC.DT, "njoints": NJ,
            "joint_order": list(SPEC.LEGS_JOINTS),
            "use_link_com": "trainer add_urdf carries true URDF COM (always)",
        },
        "ticks": ticks,
    }
    outp = Path(args.out)
    outp.parent.mkdir(parents=True, exist_ok=True)
    outp.write_text(json.dumps(out), encoding="utf-8")
    qf = np.asarray(ticks[-1]["q"])
    print(f"[g1_parity_probe:trainer] wrote {outp}  ticks={len(ticks)} "
          f"static_base={args.static_base} seq={args.sequence} "
          f"final_q[knee]={qf[3]:+.4f}/{qf[9]:+.4f} "
          f"final_base_z={ticks[-1]['base_pos'][2]:.4f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
