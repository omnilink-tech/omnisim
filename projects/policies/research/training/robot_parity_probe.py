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

"""Robot-agnostic deterministic physics probe -- TRAINER side.

Generalizes g1_parity_probe.py to ANY robot described by a parity-probe spec
(``projects/policies/research/backends/parity_probe_spec.py``, driven by the deterministic-stand
JSON). Builds the robot the TRAINER way (newton ``add_urdf`` from the SAME URDF
the deploy loads), configures POSITION_VELOCITY actuators at the spec's deploy
gains, applies the SAME scripted targets the deploy controller applies, and dumps
a per-tick trace. ``g1_parity_compare.py`` then diffs the trainer trace against
the deploy (binary) trace.

Lanes: ``--static-base`` (welded; the chaos-free machine-precision lane) /
``--free-base`` (realistic stand). Run from PowerShell so warp/CUDA initialises.

    python projects/policies/research/training/robot_parity_probe.py --robot h1 --static-base \
        --no-ground --spawn-z <lifted> --out _scratch/parity/h1_trainer.json
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

from projects.policies.research.backends import parity_probe_spec as PP  # noqa: E402

TRACE_SCHEMA = 1


def _quat_xyzw_to_rotmat(qx, qy, qz, qw):
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return [
        1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy),
        2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
        2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy),
    ]


def build(spec, static_base, add_ground, spawn_z, match_contact=True):
    mb = newton.ModelBuilder()
    if match_contact:
        mb.default_shape_cfg.ke = spec.contact_ke
        mb.default_shape_cfg.kd = spec.contact_kd
        mb.default_shape_cfg.mu = spec.ground_mu
    mb.add_urdf(str(spec.urdf),
                xform=wp.transform((0.0, 0.0, spawn_z), (0.0, 0.0, 0.0, 1.0)),
                floating=(not static_base),
                # The deploy (OmNewtonBackend) FILTERS intra-robot self-collision;
                # add_urdf defaults to enable_self_collisions=True, which gives the
                # trainer phantom self-contacts the deploy lacks (full-mesh legs
                # touching the body at the crouch -> the legs get shoved, hip_x most,
                # e.g. OmniQuad). Disable to match the deploy. Ground contact (free lane)
                # is unaffected -- the ground plane is a separate body, not self-collision.
                enable_self_collisions=False)
    free = 0 if static_base else 6
    qfree = 0 if static_base else 7
    pv = int(newton.JointTargetMode.POSITION_VELOCITY)
    for jn in spec.joint_order:
        d = spec.dof_index(jn, free)
        mb.joint_target_ke[d] = spec.ke
        mb.joint_target_kd[d] = spec.kd
        mb.joint_target_mode[d] = pv
        # keep the URDF effort limits (the deploy uses them); do NOT seed joint_q
        # -- spawn straight, ramp+settle to the pose (matches the deploy).
    if add_ground:
        mb.add_ground_plane()
    return mb.finalize(), free, qfree


def run(spec, model, free, qfree, *, static_base, spawn_z, n_ticks, settle_ticks,
        seq, record_settle):
    solver = newton.solvers.SolverMuJoCo(model, use_mujoco_cpu=False)
    state_a, state_b = model.state(), model.state()
    control = model.control()
    contacts = model.contacts() if hasattr(model, "contacts") else None
    newton.eval_fk(model, model.joint_q, model.joint_qd, state_a)
    sub_dt = spec.dt / spec.substeps
    qadr = [spec.qpos_index(jn, qfree) for jn in spec.joint_order]
    dadr = [spec.dof_index(jn, free) for jn in spec.joint_order]

    def _apply_and_step(targets):
        nonlocal state_a, state_b
        tp = control.joint_target_q.numpy()
        for i, d in enumerate(dadr):
            tp[d] = targets[i]
        control.joint_target_q.assign(tp)
        for _ in range(spec.substeps):
            state_a.clear_forces()
            if contacts is not None:
                model.collide(state_a, contacts)
                solver.step(state_a, state_b, control, contacts, sub_dt)
            else:
                solver.step(state_a, state_b, control, None, sub_dt)
            state_a, state_b = state_b, state_a

    ticks = []

    def _capture(k, target, phase):
        jq = state_a.joint_q.numpy()
        q = [float(jq[a]) for a in qadr]
        if static_base:
            base_pos = [0.0, 0.0, float(spawn_z)]
            base_rot = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        else:
            bq = state_a.body_q.numpy()[0]
            base_pos = [float(bq[0]), float(bq[1]), float(bq[2])]
            base_rot = _quat_xyzw_to_rotmat(float(bq[3]), float(bq[4]),
                                            float(bq[5]), float(bq[6]))
        ticks.append({"k": k, "phase": phase,
                      "target": [float(x) for x in target],
                      "q": q, "base_pos": base_pos, "base_rot": base_rot})

    for s in range(settle_ticks):
        st = spec.settle_target(s)
        _apply_and_step(st)
        if record_settle:
            _capture(s - settle_ticks, st, "settle")
    for k in range(n_ticks):
        tk = spec.targets(k, sequence=seq)
        _apply_and_step(tk)
        _capture(k, tk, "probe")
    return ticks


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--robot", required=True)
    ap.add_argument("--out", required=True)
    grp = ap.add_mutually_exclusive_group()
    grp.add_argument("--static-base", dest="static_base", action="store_true")
    grp.add_argument("--free-base", dest="static_base", action="store_false")
    ap.set_defaults(static_base=True)
    ap.add_argument("--no-ground", dest="add_ground", action="store_false")
    ap.set_defaults(add_ground=True)
    ap.add_argument("--ticks", type=int, default=None)
    ap.add_argument("--settle", type=int, default=None)
    ap.add_argument("--sequence", default="sinusoid", choices=["hold", "sinusoid"])
    ap.add_argument("--spawn-z", type=float, default=None)
    ap.add_argument("--record-settle", action="store_true")
    ap.add_argument("--no-contact-match", dest="match_contact", action="store_false")
    ap.set_defaults(match_contact=True)
    args = ap.parse_args(argv)

    spec = PP.load(args.robot)
    n_ticks = args.ticks if args.ticks is not None else spec.duration_ticks
    settle = args.settle if args.settle is not None else spec.settle_ticks
    spawn_z = args.spawn_z if args.spawn_z is not None else (
        spec.lifted_z() if args.static_base else spec.spawn_z)

    wp.init()
    model, free, qfree = build(spec, args.static_base, args.add_ground, spawn_z,
                               match_contact=args.match_contact)
    ticks = run(spec, model, free, qfree, static_base=args.static_base,
                spawn_z=spawn_z, n_ticks=n_ticks, settle_ticks=settle,
                seq=args.sequence, record_settle=args.record_settle)

    out = {
        "schema": TRACE_SCHEMA, "side": "trainer",
        "meta": {
            "robot": spec.name, "construction": "add_urdf",
            "urdf": str(spec.urdf.name), "sequence": args.sequence,
            "settle_ticks": settle, "static_base": args.static_base,
            "add_ground": args.add_ground, "ke": spec.ke, "kd": spec.kd,
            "spawn_z": spawn_z, "substeps": spec.substeps, "dt": spec.dt,
            "njoints": len(spec.joint_order), "joint_order": spec.joint_order,
        },
        "ticks": ticks,
    }
    outp = Path(args.out)
    outp.parent.mkdir(parents=True, exist_ok=True)
    outp.write_text(json.dumps(out), encoding="utf-8")
    print(f"[robot_parity_probe:{spec.name}] wrote {outp} ticks={len(ticks)} "
          f"static_base={args.static_base} seq={args.sequence} "
          f"final_base_z={ticks[-1]['base_pos'][2]:.4f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
