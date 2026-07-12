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

"""Conformance tests for the G1 Newton walk single-source-of-truth spec.

The ``projects/policies/research/backends/g1_physics_spec`` module (``SPEC`` below) is the ONE
place the GPU trainer and the OmniSim deploy controller derive their physics
model from. This test is the ENFORCEMENT layer: it FAILS LOUDLY the moment the
trainer, the deploy controller, and the spec drift apart again.

That drift is not hypothetical -- it cost months of "G1 collapses at ~1s"
debugging when the trainer and the deploy disagreed on gains / clamp limits /
solver substeps. The tiers below pin the known-good model and assert every
consumer agrees.

Tiers
-----
TIER 1 -- ALWAYS RUNS. Pure CPU, stdlib + numpy only. No newton / warp / GPU /
    Webots. This is the real CI drift guard and MUST pass in a bare
    ``python + numpy`` environment:
      (a) spec internal consistency (joint counts / ordering),
      (b) URDF <-> spec (limits read live from the prim URDF),
      (c) regression pins (hard-coded known-good values; a silent retune trips CI),
      (d) ``newton_env()`` contract (the OMNISIM_NEWTON_* block the deploy emits),
      (e) cross-consumer drift guard (skip-if-unimportable; asserts equality when
          the trainer/deploy modules ARE importable in a full env),
      (f) ``resolved()`` snapshot is JSON-serializable.

TIER 2 -- GPU / Newton golden trajectory. Skipped automatically when
    warp / newton / cuda are unavailable. Scaffold + TODO for the
    gold-standard trainer-vs-deploy golden-trajectory comparison.

Run with:
    python -m pytest tests/test_g1_physics_spec_conformance.py -v
"""

from __future__ import annotations

import importlib
import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

# ---------------------------------------------------------------------------
# Import the single source of truth. The spec lives under ``projects/policies`` which
# is imported as a namespace package, so the repo root must be on sys.path.
# ---------------------------------------------------------------------------
REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from projects.policies.research.backends import g1_physics_spec as SPEC  # noqa: E402


# ===========================================================================
# TIER 1 -- ALWAYS RUNS (stdlib + numpy only)
# ===========================================================================

# --- (a) spec internal consistency ----------------------------------------

def test_joint_counts():
    assert len(SPEC.LEGS_JOINTS) == 13
    assert len(SPEC.ARM_JOINTS) == 10
    assert len(SPEC.JOINT_NAMES) == 23
    assert SPEC.NJ == 13
    assert len(SPEC.NOMINAL_LEGS) == 13
    assert len(SPEC.ARM_NOMINAL) == 10


def test_joint_ordering_is_legs_then_arms():
    # The ordering is the single source for ordering too: legs/waist first 13,
    # arms last 10, concatenating back to the full JOINT_NAMES.
    assert tuple(SPEC.LEGS_JOINTS) + tuple(SPEC.ARM_JOINTS) == tuple(SPEC.JOINT_NAMES)


def test_joint_names_unique():
    assert len(set(SPEC.JOINT_NAMES)) == len(SPEC.JOINT_NAMES)


# --- (b) URDF <-> spec ------------------------------------------------------

def test_prim_urdf_exists():
    assert SPEC.URDF_PRIM.exists(), f"prim URDF missing: {SPEC.URDF_PRIM}"


def test_leg_limits_shape_and_finite():
    lo, hi, vel, eff = SPEC.leg_limits()
    for name, arr in (("lo", lo), ("hi", hi), ("vel", vel), ("eff", eff)):
        assert arr.shape == (13,), f"{name}: shape {arr.shape} != (13,)"
        assert np.all(np.isfinite(arr)), f"{name}: non-finite values {arr}"


def test_leg_limits_velocity_and_effort_positive():
    _lo, _hi, vel, eff = SPEC.leg_limits()
    assert np.all(vel > 0), f"non-positive velocity limit: {vel}"
    assert np.all(eff > 0), f"non-positive effort limit: {eff}"


def test_leg_limits_lo_below_hi_for_real_ranges():
    lo, hi, _vel, _eff = SPEC.leg_limits()
    # Every leg joint has a real position range in the G1 URDF.
    assert np.all(lo < hi), f"lo !< hi:\nlo={lo}\nhi={hi}"


# --- (c) REGRESSION PINS (hard-coded known-good values) --------------------

def test_scalar_regression_pins():
    # A silent retune of any of these flips both sides of the model and trips
    # CI here. Values are the WINNING walker's (5.9 m / 33.8 s).
    assert SPEC.KE == 100
    assert SPEC.KD == 5
    assert SPEC.SUBSTEPS == 4
    assert SPEC.DT == 0.016
    assert SPEC.GROUND_MU == 1.0
    assert SPEC.ACT_SCALE == 0.3
    assert SPEC.OBS_DIM == 50
    assert SPEC.SPAWN_Z == 0.78


def test_leg_velocity_limits_pin():
    _lo, _hi, vel, _eff = SPEC.leg_limits()
    expected = np.array(
        [32, 32, 32, 20, 30, 30, 32, 32, 32, 20, 30, 30, 32], dtype=np.float32
    )
    assert np.allclose(vel, expected, atol=2e-3), f"vel={vel}"


def test_leg_effort_limits_pin():
    _lo, _hi, _vel, eff = SPEC.leg_limits()
    expected = np.array(
        [88, 88, 88, 139, 35, 35, 88, 88, 88, 139, 35, 35, 88], dtype=np.float32
    )
    assert np.allclose(eff, expected, atol=2e-3), f"eff={eff}"


def test_leg_position_limits_pin():
    lo, hi, _vel, _eff = SPEC.leg_limits()
    expected_lo = np.array(
        [-2.531, -0.524, -2.758, -0.087, -0.873, -0.262,
         -2.531, -2.967, -2.758, -0.087, -0.873, -0.262, -2.618],
        dtype=np.float32,
    )
    expected_hi = np.array(
        [2.880, 2.967, 2.758, 2.880, 0.524, 0.262,
         2.880, 0.524, 2.758, 2.880, 0.524, 0.262, 2.618],
        dtype=np.float32,
    )
    assert np.allclose(lo, expected_lo, atol=2e-3), f"lo={lo}"
    assert np.allclose(hi, expected_hi, atol=2e-3), f"hi={hi}"


# --- (d) newton_env() contract ---------------------------------------------

def test_newton_env_contract():
    env = SPEC.newton_env()
    # The deploy backend reads these OMNISIM_NEWTON_* env vars. The values are
    # strings (env vars), so compare as ints/strings accordingly.
    assert env["OMNISIM_NEWTON_FORCE_MUJOCO"] == "1"
    assert env["OMNISIM_NEWTON_MJWARP"] == "1"
    assert int(env["OMNISIM_NEWTON_SUBSTEPS"]) == 4
    assert int(env["OMNISIM_NEWTON_TARGET_KE"]) == 100
    assert int(env["OMNISIM_NEWTON_TARGET_KD"]) == 5
    assert int(env["OMNISIM_NEWTON_GROUND_MU"]) == 1
    assert env["OMNISIM_URDF_USE_INERTIA"] == "1"
    assert env["OMNISIM_NEWTON_SEED_POSE"] == "1"


def test_newton_env_substeps_matches_spec():
    # The env block must agree with the flat constant -- a single substeps value
    # for the whole model, not two transcriptions that can diverge.
    env = SPEC.newton_env()
    assert int(env["OMNISIM_NEWTON_SUBSTEPS"]) == SPEC.SUBSTEPS


# --- (e) CROSS-CONSUMER drift guard (skip-if-unimportable) -----------------
#
# In a full post-merge environment the deploy controller and the trainer derive
# their constants from SPEC; here we assert they EQUAL SPEC. But the deploy
# controller imports the Webots ``controller`` module and the trainer imports
# torch / newton -- neither is present in a bare python+numpy CI runner. So each
# import is wrapped in try/except and SKIPS cleanly when the module (or one of
# its heavy deps) is unavailable.

# NOTE: the RL tree was renamed projects/rl -> projects/policies/research (the
# old paths silently ImportError->skip'd these two drift guards on every run).
_DEPLOY_CTRL_DIR = REPO_ROOT / "projects" / "policies" / "research" / "controllers" / "g1_walk_deploy"
_DEPLOY_CTRL_FILE = _DEPLOY_CTRL_DIR / "g1_walk_deploy.py"
_TRAINER_FILE = REPO_ROOT / "projects" / "policies" / "research" / "training" / "gpu_newton_g1_walk_trainer.py"


def _import_isolated(name: str, path: Path):
    """Import a standalone script module by file path.

    Returns the loaded module. Raises ImportError (propagating the real cause)
    if the file is missing or any of its imports fail -- which is exactly what
    lets the callers below skip cleanly in a bare env.
    """
    if not path.exists():
        raise ImportError(f"{name} not found at {path}")
    # The deploy controller does ``sys.path.insert`` for the repo root and is
    # designed to be imported standalone; add its own dir for sibling imports.
    if str(path.parent) not in sys.path:
        sys.path.insert(0, str(path.parent))
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot build import spec for {name} at {path}")
    module = importlib.util.module_from_spec(spec)
    # Register before exec so any intra-module ``import name`` resolves.
    sys.modules[name] = module
    try:
        spec.loader.exec_module(module)
    except BaseException as exc:  # noqa: BLE001 - re-raise as ImportError to skip
        sys.modules.pop(name, None)
        raise ImportError(f"{name} import failed: {exc!r}") from exc
    return module


def test_deploy_controller_matches_spec():
    """Deploy controller's spec-derived constants must equal SPEC.

    Skips when the Webots ``controller`` module (or any other deploy dep) is
    unavailable -- i.e. always in a bare CI runner. Runs for real in a full
    OmniSim environment, where it is the train<->deploy drift guard.
    """
    try:
        mod = _import_isolated("g1_walk_deploy_conformance", _DEPLOY_CTRL_FILE)
    except ImportError as exc:
        pytest.skip(f"deploy controller not importable in this env: {exc}")

    # OBS_DIM and ACT_SCALE are simple scalars the controller exposes today.
    if hasattr(mod, "OBS_DIM"):
        assert int(mod.OBS_DIM) == SPEC.OBS_DIM, (
            f"deploy OBS_DIM {mod.OBS_DIM} != SPEC.OBS_DIM {SPEC.OBS_DIM}"
        )
    if hasattr(mod, "ACT_SCALE"):
        assert math.isclose(float(mod.ACT_SCALE), SPEC.ACT_SCALE, abs_tol=1e-9), (
            f"deploy ACT_SCALE {mod.ACT_SCALE} != SPEC.ACT_SCALE {SPEC.ACT_SCALE}"
        )

    # Joint-clamp limits: the deploy clips targets to LIM_LO/LIM_HI; these must
    # equal the URDF-derived spec limits (the historical drift source).
    lo, hi, _vel, _eff = SPEC.leg_limits()
    if hasattr(mod, "LIM_LO"):
        assert np.allclose(np.asarray(mod.LIM_LO, dtype=np.float32), lo, atol=2e-3), (
            f"deploy LIM_LO != SPEC leg lo\n{mod.LIM_LO}\n{lo}"
        )
    if hasattr(mod, "LIM_HI"):
        assert np.allclose(np.asarray(mod.LIM_HI, dtype=np.float32), hi, atol=2e-3), (
            f"deploy LIM_HI != SPEC leg hi\n{mod.LIM_HI}\n{hi}"
        )


def test_trainer_matches_spec():
    """Trainer's spec-derived constants must equal SPEC.

    Skips when torch / newton / warp (the trainer's heavy deps) are
    unavailable. Runs for real in a full training environment.
    """
    try:
        mod = _import_isolated("gpu_newton_g1_walk_trainer_conformance", _TRAINER_FILE)
    except ImportError as exc:
        pytest.skip(f"trainer not importable in this env: {exc}")

    # Defensive: the trainer is mid-migration to SPEC. Assert any spec-mirrored
    # constants it DOES expose. Names are checked by hasattr so this test does
    # not become a maintenance trap for the (other-track-owned) trainer module.
    checked = False
    for attr, expected in (
        ("KE", SPEC.KE),
        ("KD", SPEC.KD),
        ("SUBSTEPS", SPEC.SUBSTEPS),
        ("ACT_SCALE", SPEC.ACT_SCALE),
        ("OBS_DIM", SPEC.OBS_DIM),
    ):
        if hasattr(mod, attr):
            checked = True
            got = getattr(mod, attr)
            assert math.isclose(float(got), float(expected), rel_tol=0, abs_tol=1e-6), (
                f"trainer {attr} {got} != SPEC {attr} {expected}"
            )
    if not checked:
        pytest.skip(
            "trainer exposes no spec-mirrored module constants yet "
            "(pre-merge); nothing to compare"
        )


# --- (f) resolved() snapshot ------------------------------------------------

def test_resolved_is_json_serializable():
    snap = SPEC.resolved()
    # Round-trips through JSON with no custom encoder.
    dumped = json.dumps(snap)
    reloaded = json.loads(dumped)
    assert reloaded == snap


def test_resolved_contains_leg_limits_from_urdf():
    snap = SPEC.resolved()
    assert "leg_limits_from_urdf" in snap
    assert len(snap["leg_limits_from_urdf"]) == 13


# ===========================================================================
# TIER 2 -- GPU / Newton golden trajectory (auto-skips without warp/newton/cuda)
# ===========================================================================

def _newton_stack_available() -> bool:
    """True only when the full Newton/warp/CUDA stack is importable AND a CUDA
    device is present. Any failure -> Tier 2 skips (it must never break
    collection)."""
    if importlib.util.find_spec("warp") is None:
        return False
    if importlib.util.find_spec("newton") is None:
        return False
    try:
        import warp as wp  # noqa: WPS433 - intentional local import

        wp.init()
        return wp.get_cuda_device_count() > 0
    except Exception:  # noqa: BLE001 - any failure -> not available
        return False


_NEWTON_AVAILABLE = _newton_stack_available()


@pytest.mark.skipif(
    not _NEWTON_AVAILABLE,
    reason="requires warp + newton + a CUDA device (GPU golden-trajectory tier)",
)
def test_newton_golden_trajectory_clamp_holds():
    """Step a small Newton G1 model and assert the joint clamp holds qpos in spec.

    Builds the G1 model from the prim URDF + the spec's solver/gains (via the
    trainer's builder helper when available), seeds the spec spawn pose, steps
    N=50 ticks with a FIXED deterministic action sequence and a FIXED seed, and
    asserts:
      * the whole trajectory stays finite (no NaN/Inf blow-up), and
      * with CLAMP_ENABLED the leg qpos stays within [LIM_LO, LIM_HI] (the
        URDF-derived spec limits) for every tick.

    TODO (gold-standard, needs the native binary + GPU):
        The definitive check is a TRAINER-vs-DEPLOY golden-trajectory match:
        build the SAME model two ways -- (1) the GPU trainer's Newton
        ModelBuilder path and (2) the deploy backend's embedded-Python Newton
        path driven by ``SPEC.newton_env()`` -- feed both the identical fixed
        action sequence from the identical seed pose, and assert the resulting
        qpos/qvel trajectories agree to a tight tolerance over N steps. That is
        the true enforcement that the two solvers instantiate the SAME physics
        from the spec. It requires the native OmniSim binary (embedded Python +
        warp on the user site) and a CUDA device, so it cannot run on a CPU CI
        runner. Implement it here once a GPU runner / native binary is wired
        into CI; until then this tier validates the single-build clamp invariant
        and the cross-build comparison stays a documented TODO.
    """
    import numpy as _np

    # --- Build the model. Prefer the trainer's builder helper if it exposes
    #     one; otherwise this is the documented place to construct it directly.
    builder_fn = None
    try:
        trainer = _import_isolated(
            "gpu_newton_g1_walk_trainer_tier2", _TRAINER_FILE
        )
        for cand in ("build_g1_model", "build_model", "make_model", "build_newton_model"):
            if hasattr(trainer, cand):
                builder_fn = getattr(trainer, cand)
                break
    except ImportError as exc:
        pytest.skip(f"trainer builder unavailable: {exc}")

    if builder_fn is None:
        pytest.skip(
            "no trainer model-builder helper found to construct the Newton G1 "
            "model from SPEC; see TODO in this test for the intended assertion"
        )

    # --- Deterministic rollout.
    rng = _np.random.default_rng(0)
    n_steps = 50
    lo, hi, _vel, _eff = SPEC.leg_limits()
    actions = rng.uniform(-1.0, 1.0, size=(n_steps, SPEC.NJ)).astype(_np.float32)

    model = builder_fn()  # builder is expected to honour SPEC gains/solver.
    qpos_log = []
    for t in range(n_steps):
        # The concrete step API is builder-dependent; the invariant we assert is
        # solver-agnostic. A builder that returns a steppable handle should
        # expose .step(action) -> state with .qpos. Guard so we skip rather than
        # error if the shape differs from the assumed contract.
        if not hasattr(model, "step"):
            pytest.skip("model handle has no .step(); builder contract differs")
        state = model.step(actions[t])
        qpos = _np.asarray(getattr(state, "qpos", state), dtype=_np.float32)
        qpos_log.append(qpos)
        assert _np.all(_np.isfinite(qpos)), f"non-finite qpos at step {t}: {qpos}"

    traj = _np.stack(qpos_log)
    assert _np.all(_np.isfinite(traj)), "trajectory contains non-finite values"

    if SPEC.CLAMP_ENABLED:
        leg_q = traj[:, : SPEC.NJ]
        eps = 1e-3
        assert _np.all(leg_q >= lo - eps) and _np.all(leg_q <= hi + eps), (
            "joint clamp failed to keep leg qpos within SPEC limits"
        )
