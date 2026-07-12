"""Conformance test for the shared G1 obs/IC layer (``g1_env_core``).

Locks the single-source observation + initial-condition definitions so the
trainer and the deploy controller can never silently diverge again (the
documented H1 corollary: matched model + matched solver still diverge because
of the obs pipeline + the launch state).

Numpy-only importable so the existing ``g1-spec-conformance.yml`` CI lane
(numpy + pytest, no torch/newton/CUDA) runs it. The torch-parity checks
self-skip when torch is absent.

Run:
    python -m pytest tests/test_g1_env_core_parity.py -v
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from projects.policies.research.backends import g1_physics_spec as SPEC
from projects.policies.research.backends import g1_env_core as core


# ---------------------------------------------------------------------------
# Layout / contract.
# ---------------------------------------------------------------------------
def test_core_obs_dim_matches_spec():
    # The 50-d core obs the trainer and deploy share == SPEC.OBS_DIM.
    assert core.CORE_OBS_DIM == 50
    assert core.CORE_OBS_DIM == SPEC.OBS_DIM
    assert core.N_LEG == len(SPEC.LEGS_JOINTS) == 13


def test_settle_steps():
    assert core.settle_steps(0.016) == math.ceil(0.3 / 0.016) == 19
    assert core.settle_steps() == core.settle_steps(SPEC.DT)
    # longer dt -> fewer settle steps.
    assert core.settle_steps(0.05) == math.ceil(0.3 / 0.05)


def test_nominal_full_order():
    nf = core.nominal_full()
    assert len(nf) == 23
    assert nf[:13] == list(SPEC.NOMINAL_LEGS)
    assert nf[13:] == list(SPEC.ARM_NOMINAL)


# ---------------------------------------------------------------------------
# Projected gravity: upright sentinel + quat==matrix equivalence.
# ---------------------------------------------------------------------------
def test_proj_gravity_upright():
    q = np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float64)  # identity (w,x,y,z)
    pg = core.proj_gravity_from_quat(q)
    np.testing.assert_allclose(pg[0], [0.0, 0.0, -1.0], atol=1e-12)


def test_proj_gravity_quat_equals_matrix():
    # A non-trivial tilt: roll 0.2, pitch -0.35, yaw 0.5.
    roll, pitch, yaw = 0.2, -0.35, 0.5
    q = _euler_to_quat_wxyz(roll, pitch, yaw)
    pg_quat = core.proj_gravity_from_quat(np.array([q], dtype=np.float64))[0]
    R = core.quat_wxyz_to_R_rowmajor(q)
    pg_mat = core.proj_gravity_from_matrix(R)
    np.testing.assert_allclose(pg_quat, pg_mat, atol=1e-10)
    # both equal R^T (0,0,-1); magnitude of a unit vector.
    np.testing.assert_allclose(np.linalg.norm(pg_quat), 1.0, atol=1e-10)


# ---------------------------------------------------------------------------
# Angular-velocity frame: the deploy R^T fix, round-trip + identity.
# ---------------------------------------------------------------------------
def test_world_ang_to_body_identity():
    R = core.quat_wxyz_to_R_rowmajor([1.0, 0.0, 0.0, 0.0])  # identity
    omega = (0.3, -0.7, 1.1)
    out = core.world_ang_to_body_matrix(R, omega)
    np.testing.assert_allclose(out, omega, atol=1e-12)


def test_world_ang_to_body_roundtrip():
    # If the trainer's body-frame omega is rotated to world (R @ omega_body) and
    # then back through the deploy's R^T, it must recover omega_body exactly --
    # i.e. the deploy fix and the trainer's qvel[3:6] are the SAME quantity.
    q = _euler_to_quat_wxyz(0.15, 0.4, -0.6)
    R = np.array(core.quat_wxyz_to_R_rowmajor(q), dtype=np.float64).reshape(3, 3)
    omega_body = np.array([0.5, -0.2, 0.9])
    omega_world = R @ omega_body          # body -> world
    out = core.world_ang_to_body_matrix(R.reshape(-1), omega_world)  # R^T world
    np.testing.assert_allclose(out, omega_body, atol=1e-10)


# ---------------------------------------------------------------------------
# JointVelEstimator: finite-diff recovers a constant velocity (numpy).
# ---------------------------------------------------------------------------
def test_jointvel_recovers_constant_velocity_raw():
    dt = SPEC.DT
    est = core.JointVelEstimator(dt, tau=0.0)   # raw finite-diff, alpha=1
    assert est.alpha == 1.0
    v = np.array([1.0, -2.0, 0.5])
    q = np.zeros(3)
    est.reset(q)
    out = None
    for k in range(1, 6):
        q = v * (k * dt)
        out = est.update(q)
    # raw finite-diff of a linear ramp = exact velocity.
    np.testing.assert_allclose(out, v, atol=1e-9)


def test_jointvel_lowpass_converges():
    dt = SPEC.DT
    est = core.JointVelEstimator(dt, tau=0.05)  # smoothed
    assert 0.0 < est.alpha < 1.0
    v = np.array([1.0])
    q = np.zeros(1)
    est.reset(q)
    out = None
    for k in range(1, 200):
        q = v * (k * dt)
        out = est.update(q)
    np.testing.assert_allclose(out, v, atol=1e-3)


def test_jointvel_reset_rows_zeros_velocity():
    dt = SPEC.DT
    est = core.JointVelEstimator(dt, tau=0.0)
    q = np.zeros((4, 3))
    est.reset(q)
    # advance one env into motion
    q2 = q.copy()
    q2[0] = [0.1, 0.2, 0.3]
    est.update(q2)
    # reset env 0 only -> its next qd must be 0 (no fake teleport velocity)
    mask = np.array([True, False, False, False])
    est.reset_rows(q2, mask)
    out = est.update(q2)        # no motion since reset
    np.testing.assert_allclose(out[0], [0, 0, 0], atol=1e-12)


# ---------------------------------------------------------------------------
# assemble_walk_obs: order + dim.
# ---------------------------------------------------------------------------
def test_assemble_layout_numpy():
    N = 2
    lin = np.full((N, 3), 1.0)
    ang = np.full((N, 3), 2.0)
    pg = np.full((N, 3), 3.0)
    q = np.full((N, 13), 4.0)
    qd = np.full((N, 13), 5.0)
    la = np.full((N, 13), 6.0)
    gs = np.full((N, 2), 7.0)
    obs = core.assemble_walk_obs(lin, ang, pg, q, qd, la, gs, sanitize=False)
    assert obs.shape == (N, 50)
    # check block boundaries carry the right marker value
    assert obs[0, 0] == 1.0 and obs[0, 3] == 2.0 and obs[0, 6] == 3.0
    assert obs[0, 9] == 4.0 and obs[0, 22] == 5.0 and obs[0, 35] == 6.0
    assert obs[0, 48] == 7.0


def test_assemble_sanitize_clamps():
    N = 1
    big = np.full((N, 3), 1e6)
    nan = np.full((N, 3), np.nan)
    z3 = np.zeros((N, 3))
    z13 = np.zeros((N, 13))
    z2 = np.zeros((N, 2))
    obs = core.assemble_walk_obs(big, nan, z3, z13, z13, z13, z2, sanitize=True)
    assert obs.max() <= 10.0 and obs.min() >= -10.0
    assert not np.isnan(obs).any()


# ---------------------------------------------------------------------------
# numpy <-> torch parity (the single-impl, two-backend guarantee).
# ---------------------------------------------------------------------------
def _torch_or_skip():
    try:
        import torch  # noqa: F401
        return torch
    except Exception:
        pytest.skip("torch not installed -- numpy-only CI lane")


def test_torch_matches_numpy_proj_gravity():
    torch = _torch_or_skip()
    q = np.array([_euler_to_quat_wxyz(0.2, -0.35, 0.5),
                  _euler_to_quat_wxyz(-0.1, 0.25, 1.2)], dtype=np.float64)
    pg_np = core.proj_gravity_from_quat(q)
    pg_t = core.proj_gravity_from_quat(torch.tensor(q))
    np.testing.assert_allclose(pg_np, pg_t.numpy(), atol=1e-12)


def test_torch_matches_numpy_assemble():
    torch = _torch_or_skip()
    rng = np.random.RandomState(0)
    parts = [rng.randn(3, w).astype(np.float64)
             for w in (3, 3, 3, 13, 13, 13, 2)]
    obs_np = core.assemble_walk_obs(*parts, sanitize=True)
    obs_t = core.assemble_walk_obs(*[torch.tensor(p) for p in parts],
                                   sanitize=True)
    np.testing.assert_allclose(obs_np, obs_t.numpy(), atol=1e-12)


def test_torch_matches_numpy_jointvel():
    torch = _torch_or_skip()
    dt = SPEC.DT
    v = np.array([1.0, -2.0, 0.5])
    est_np = core.JointVelEstimator(dt, tau=0.05)
    est_t = core.JointVelEstimator(dt, tau=0.05)
    est_np.reset(np.zeros(3))
    est_t.reset(torch.zeros(3, dtype=torch.float64))
    out_np = out_t = None
    for k in range(1, 30):
        q = v * (k * dt)
        out_np = est_np.update(q.copy())
        out_t = est_t.update(torch.tensor(q))
    np.testing.assert_allclose(out_np, out_t.numpy(), atol=1e-12)


# ---------------------------------------------------------------------------
# helper.
# ---------------------------------------------------------------------------
def _euler_to_quat_wxyz(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return [w, x, y, z]
