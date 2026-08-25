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
import warp as wp
import newton
import json as _json
import os as _os
from time import perf_counter as _perf

# ---- PRE-IMPORT mujoco / mujoco_warp ---------------------------------------
# These are imported LAZILY by newton, on the first line of
# SolverMuJoCo.__init__ (its import_mujoco() classmethod). That puts ~0.9 s on
# the CRITICAL PATH of world finalize, where nothing can overlap it -- and it is
# a per-PROCESS cost, so every fresh-process load pays it in full while a second
# world in the same process pays ~20 ms.
#
# MEASURED 2026-08-14, machine 9722d23d12a3, warm warp cache, bare interpreter:
#
#     import warp + newton            1251 ms   <- already pre-imported here
#     import mujoco                     66 ms   <- was NOT
#     import mujoco_warp               859 ms   <- was NOT
#     builder.finalize(1 body)          13 ms
#     SolverMuJoCo(1 body)              58 ms   <- the actual construction
#     SolverMuJoCo (2nd, same process)  22 ms
#
# i.e. the ~1.7 s "SolverMuJoCo" finalize phase is overwhelmingly IMPORTS, not
# model building -- a one-body solver builds in 58 ms and the engine's own
# profiler attributes 1678 ms to that phase on a two-body cloth world.
#
# This module is what OmPhysicsBackendRegistry::startNewtonRuntimePreload()
# loads on a BACKGROUND thread while the engine is still parsing the .wbt
# (OmNewtonBackend::preloadRuntime -> the helper module import). Importing
# mujoco here therefore moves that ~0.9 s off finalize and overlaps it with
# world parse, for free, on a thread that already exists.
#
# Guarded, and deliberately so: if either package is missing or broken this
# must be a NO-OP, not a runtime that fails to import -- newton would simply
# take the lazy path later exactly as before, and the failure would surface
# there with its own message instead of as "the physics runtime is gone".
# OMNISIM_NEWTON_PREIMPORT_MUJOCO=0 disables it, which is also the A/B lever
# for re-measuring the win.
if (_os.environ.get("OMNISIM_NEWTON_PREIMPORT_MUJOCO") or "1").strip() != "0":
    try:
        import mujoco as _mj_preimport  # noqa: F401
        import mujoco_warp as _mjw_preimport  # noqa: F401
    except Exception:  # noqa: BLE001
        pass

# OMNISIM_DEBUG_LAUNCH=<file>: append the whole mj state (qpos/qvel/ctrl,
# actuator + constraint generalized forces, every contact with its force and
# frame) once per step, and dump the compiled mjModel to <file>.mjb on the
# first one. Resolved ONCE at import: this is read on the step path and the
# per-tick environ lookup is not free at 250 Hz x N worlds.
_LAUNCH_DBG = _os.environ.get("OMNISIM_DEBUG_LAUNCH")

# P3.10e: this helper was rewritten after a 6-probe XPBD characterization
# (see scripts/xpbd_probes/notes.md). The old version used the wrong
# Newton API and gain values, hiding two compounding bugs:
#
#   1. `builder.add_body()` AUTO-ADDS a phantom 6-DOF FREE joint per body.
#      Subsequent `add_joint_revolute` calls add a SECOND joint instead of
#      replacing the free one, so every OmSolid had two joints fighting
#      each other -- chassis decoupled from wheels because XPBD couldn't
#      satisfy both. Fix: use `add_link` (no auto-joint), then *one*
#      explicit joint per body, then `add_articulation([joint_indices])`.
#
#   2. `target_kd=1.0` is ~500x too small for velocity drive. Newton's
#      official velocity-control test (newton/tests/test_joint_controllers.py)
#      uses `target_ke=0, target_kd=500` -- the wheel literally does not
#      spin with kd=1 + XPBD's default angular damping. Fix: OmBasicJoint
#      hardcodes target_ke=0, target_kd=500 for motorized hinges and
#      passes them through addJointRevolute. `_add_revolute_to_builder`
#      then honors those per-joint values (with OMNISIM_NEWTON_TARGET_KE /
#      OMNISIM_NEWTON_TARGET_KD env vars acting as opt-in overrides only
#      when explicitly set -- the Spot residual recipe sets KE=250/KD=60
#      explicitly). actuator_mode=POSITION_VELOCITY +
#      `SolverXPBD(angular_damping=0.0)` so the spin isn't damped out.
#      A 2026-05-28 regression had env defaults (20/3) applied
#      UNCONDITIONALLY, with the per-joint `target_ke` silently dropped
#      by `add_joint_revolute`'s pending_revolutes append -- the huskies
#      didn't drive (P6 capture surfaced this: Newton 0 events vs ODE
#      149 in head-on damage scenario). Fix lands the override-only-
#      when-set semantics and stores target_ke alongside target_kd in
#      pending_revolutes. Regression probe:
#      `scripts/xpbd_probes/probe_husky_motor_minimal.py` sweeps the
#      (ke, kd) matrix and verifies kd=500 drives + kd=3 freezes.
#
#   3. Tight URDF-spec inertia tensors (e.g. husky chassis ixx=0.6) are too
#      "light" rotationally and chassis perturbations get amplified into
#      wheel velocity oscillations. Fix: floor every body's inertia at
#      0.1*mass*identity (= roughly geometric block inertia for a meter-
#      scale rectangular body of that mass).
#
# Probe 6 (mini-husky) drove 4.05 m / 5 s = 98%; probe 7 (real husky URDF
# geometry) drove 4.04 m / 10 s = 97.8% with this exact config.

class _ClampClean(Exception):
    """Internal: joints are all in range, skip the slow clamp path."""



def _mj_full_mass(mj, m, d, nv, np_mod):
    """Dense mass matrix, across mujoco's mj_fullM signature change.

    mujoco <=3.8:  mj_fullM(m, dst, d.qM)   -- sparse qM passed explicitly
    mujoco >=3.9:  mj_fullM(m, d, dst)      -- takes MjData; d.qM may not exist

    Verified empirically on the vendored builds: the 3.8 form raises
    AttributeError on 3.11.0, and the 3.11 form raises TypeError on 3.8.1. Try
    the modern form first, fall back to the legacy one, so ONE source drives
    both runtimes during (and after) the newton 1.2 -> 1.5 migration.
    """
    dst = np_mod.zeros((nv, nv))
    try:
        mj.mj_fullM(m, d, dst)            # mujoco >= 3.9
    except (TypeError, AttributeError):
        mj.mj_fullM(m, dst, d.qM)         # mujoco <= 3.8
    return dst

class World:
    def __init__(self):
        # up_axis=Axis.Z is the CONSTRUCTION default, not the world's answer:
        # ENU (the WorldInfo.coordinateSystem schema default, and 509 of the 719
        # worlds in this tree) is z-up, so this is right for them and stays
        # byte-identical. A Y-up world (NUE / EUN) overrides it through
        # set_up_axis(), which C++ calls IMMEDIATELY after this constructor and
        # BEFORE add_ground_plane() -- see that method for why the ordering is
        # the whole fix.
        self.builder = newton.ModelBuilder(up_axis=newton.Axis.Z)
        # CONTACT-ENVELOPE PIN (newton >=1.4). From 1.4 SolverMuJoCo propagates
        # the authored shape gap into MuJoCo's `geom_gap`/`pair_gap`, and any
        # shape without an explicit gap inherits `ModelBuilder.rigid_gap`, whose
        # default is 0.1. Combined with MuJoCo >=3.9 semantics -- contacts are
        # DETECTED at `dist < margin + gap` while forces still act at
        # `dist < margin` -- every geom would gain a 10 cm detection envelope.
        # Forces would stay correct, but `get_contacts()` walks mj_data.contact
        # without filtering on `efc_address >= 0`, so /sim/contacts,
        # getContactPoints and /sim/grips would report every pair within 10 cm
        # as TOUCHING, and ncon would balloon against the njmax/nconmax caps.
        # Pinning it to 0.0 keeps the detection envelope exactly what it has
        # always been. ⚠ `legacy_margin_gap=True` is NOT the lever here: it only
        # applies to ModelBuilder.add_mjcf()/add_usd(), and this engine builds
        # its model programmatically from the scene graph.
        # On 1.2.0 the field exists but is never propagated to MuJoCo, so this
        # is inert there -- verified by the gate battery before the runtime
        # switch, which is what makes one source safe on both runtimes.
        try:
            self.builder.rigid_gap = 0.0
        except Exception:                      # noqa: BLE001 - older/newer field name
            pass
        # Resolved up axis as a bare letter, for logging + the finalize()
        # telemetry line. Overwritten by set_up_axis().
        self._up_axis_name = "Z"
        # P3.10e: kd=500 default, POSITION_VELOCITY actuators. Per-joint
        # values get overridden by add_joint_revolute below; this is just
        # the global default that any unset joint inherits.
        self.builder.default_joint_cfg.armature = 0.01
        self.builder.default_joint_cfg.target_ke = 0.0
        self.builder.default_joint_cfg.target_kd = 500.0
        # Contact friction (MuJoCo tangential mu). Default 1.0 keeps the
        # validated wheeled-robot contact (Husky drove 98% at mu=1.0).
        # OMNISIM_NEWTON_GROUND_MU lets legged robots raise it: a friction
        # probe showed sphere feet SLIDE ~1 m while merely standing at
        # mu=1.0 but PLANT (<4 cm) at mu=2.0 -- slipping feet give no
        # lateral restoring force, which is why Spot's roll went marginal
        # under the trot. Applies to both default_shape_cfg (ground plane
        # + box/sphere bounding objects) and _SHAPE_CFG (URDF feet).
        # env var > WorldInfo.newtonGroundMu > 1.0. set_contact_solver_params()
        # runs BEFORE finalize but may run AFTER this constructor, and it
        # re-applies all three, so this is the pre-declaration baseline.
        #
        # RESET THE CLASS-LEVEL CONTACT STATE FIRST. _contact_world and
        # _SHAPE_CFG are class attributes, so without this a SECOND world
        # loaded in the same process inherited the PREVIOUS world's declared
        # friction/compliance -- a world that declared nothing got whatever
        # the last world asked for. A fresh World starts from env + engine
        # defaults; its own declaration (if any) arrives via
        # set_contact_solver_params before the ground plane is added.
        World._contact_world = {"mu": None, "ke": None, "kd": None}
        World._SHAPE_CFG = None
        self._ground_mu = World._contact_value(
            "OMNISIM_NEWTON_GROUND_MU", World._contact_world.get("mu"), 1.0)
        self.builder.default_shape_cfg.mu = self._ground_mu
        # Contact compliance. SolverMuJoCo maps a shape's (ke, kd) to the geom's
        # MuJoCo solref (convert_solref). Default 2500/100 is firm; lowering ke
        # SOFTENS contacts -- needed so the gripper plowing a dense cube layer on
        # a DYNAMIC bin floor can't inject launch energy and eject neighbours
        # (the static ground never did). Env-tunable so it can be dialled in
        # without a rebuild.
        self.builder.default_shape_cfg.ke = World._contact_value(
            "OMNISIM_NEWTON_CONTACT_KE", World._contact_world.get("ke"), 2500.0)
        self.builder.default_shape_cfg.kd = World._contact_value(
            "OMNISIM_NEWTON_CONTACT_KD", World._contact_world.get("kd"), 100.0)

        self.model = None
        self.solver = None
        self.state_a = None
        self.state_b = None
        self.joint_targets = {}        # slot_id -> rad/s, per-step writes
        self.joint_targets_pos = {}    # slot_id -> rad (position), per-step writes
        self.joint_forces = {}         # slot_id -> Nm (joint torque via control.joint_f), per-step writes
        self.control = None
        self.body_indices = []         # all bodies added (for root detection)
        # P8.2: indices of STATIC collider bodies (add_static_body). In
        # finalize() each is pinned with a 0-DOF FIXED joint to the world +
        # nominal mass/inertia (see the static-collider block there) so it is
        # immovable yet MuJoCo-compilable -- the original mass=0 FREE-joint
        # recipe broke MuJoCo (which rejects mass<mjMINVAL moving bodies).
        # Collision against dynamic bodies still fires.
        self.static_body_indices = []
        # Plane shapes requested on STATIC bodies are deferred to
        # finalize(): newton's MuJoCo converter raises "Planes can only
        # be attached to static bodies" for our weld-pinned statics, so
        # under a MuJoCo solver preference they are dropped (the default
        # ground plane, attached to the true static world, already
        # covers z=0) and under XPBD they are added as before.
        self._deferred_static_planes = []
        # Revolute joints are *queued* during build (not pushed to the
        # builder yet); finalize() resolves topology and adds them in
        # parent-before-child order. Pre-finalize we hand the caller a
        # stable "slot id" so they can address the joint for target_vel
        # writes; finalize() maps slot -> real builder joint index.
        self.pending_revolutes = []    # list of dicts (one per add_joint_revolute call)
        self.slot_to_real_idx = {}     # slot_id -> builder joint index, set in finalize()
        self.joint_indices = []        # builder joint indices (in articulation order)

        # ---- CLOTH (deformable particles) --------------------------------
        # OmniSim's cloth lives on SolverVBD; the ROBOT stays on SolverMuJoCo
        # and the two are coupled by SolverCoupledProxy over ONE Model. This is
        # the architecture upstream's example_proxy_joint_gripper.py uses (a
        # MuJoCo palm + prismatic fingers gripping a VBD soft grid), and it is
        # the only one that keeps the MuJoCo joint features OmniSim's working
        # grasp depends on -- the `effortLimit * 10` PD servo, armature and
        # target_mode -- none of which SolverVBD implements.
        #
        # Every field below stays falsy/None on a world with no cloth, and each
        # cloth branch in finalize()/step() is guarded on that, so a rigid world
        # takes byte-identically the same code it took before cloth existed.
        self.cloth_grids = []          # one dict per add_cloth_grid() call
        self.soft_grids = []           # one dict per add_soft_grid() call (OmSoftBody)
        self.cloth_particle_start = -1  # first particle index owned by cloth
        self.cloth_particle_end = -1    # one past the last
        self.solver_soft = None        # SolverVBD instance, cloth worlds only.
                                       # ⚠ _mjc_batch_substeps_ok() keys its
                                       # refusal on THIS NAME -- the batched CPU
                                       # path drives mj_step directly and never
                                       # calls solver.step(), so a second solver
                                       # would be silently skipped. Renaming this
                                       # attribute without updating that guard
                                       # makes cloth freeze with no error.
        self.solver_mjc = None         # the SolverMuJoCo *inside* the coupled
                                       # solver; see _mjc_solver().
        self.collision_pipeline = None  # newton.CollisionPipeline, cloth only
        # ---- PER-BODY CLOTH COUPLING (Solid.newtonClothCoupling) ----------
        # newton body index -> +1 (couple) / -1 (do not couple). A body absent
        # from this dict declared nothing. EMPTY on every world that does not
        # use the field, which is what keeps the default roster (every body)
        # bit-identical. See _resolve_coupled_bodies.
        self._cloth_coupling = {}

    def set_cloth_coupling(self, body_index, mode):
        """Record one Solid's newtonClothCoupling declaration.

        Called from the C++ Newton flush once per Solid that declares a
        non-zero value, AFTER that Solid has a newton body index and BEFORE
        finalize() builds the coupled solver. 0 (unset) never reaches here.
        """
        try:
            b, m = int(body_index), int(mode)
        except (TypeError, ValueError):
            return
        if b < 0 or m == 0:
            return
        self._cloth_coupling[b] = 1 if m > 0 else -1

    def set_solver_preference(self, name):
        # Per-world choice from WorldInfo.newtonSolver, plumbed by the C++
        # backend before finalize(). The SOLVER is SolverMuJoCo, always --
        # XPBD was removed 2026-08-07 (one day after the default flipped to
        # MuJoCo, 7b431e81: nothing in 725 worlds ever selected XPBD, nothing
        # in OmniSim needs what it is for, and it measured slower AND
        # physically wrong on the shipped Husky swarm; full record in
        # docs/developer/ode-retirement-campaign.md #0.5). What remains of
        # this preference is CPU vs GPU within MuJoCo: "mujoco_warp" (or
        # OMNISIM_NEWTON_MJWARP=1) selects the batched GPU path, anything
        # else the reference CPU mj_step. "xpbd" is no longer in the
        # WorldInfo.newtonSolver schema enum, so a stale world declaring it
        # gets the parser's invalid-value warning and the default (MuJoCo).
        self._solver_pref = str(name) if name else None

    def set_substeps(self, n):
        # Per-world sub-step count from WorldInfo.newtonSubsteps, plumbed by the
        # C++ backend before the first step. Folds the OMNISIM_NEWTON_SUBSTEPS
        # launch knob into the world file; the env var (read in step()) still
        # takes precedence. Stored here; step() resolves env > world > 1.
        try:
            self._n_substeps_world = max(1, int(n))
        except (TypeError, ValueError):
            self._n_substeps_world = 1

    @staticmethod
    def _coord_system_enabled():
        # OMNISIM_NEWTON_COORD_SYSTEM: value-parsed, DEFAULT ON. This is a BUG
        # FIX, not a feature -- before it, WorldInfo.coordinateSystem was read
        # ZERO times on this path -- so the default has to be on. "0"/"false"/
        # "off"/"no" pins the builder back to the historical hardcoded z-up so a
        # regression can be bisected against the pre-fix physics; unset (or any
        # other value) honours the world file. Note ENU is unaffected either way:
        # it resolves to Axis.Z, which is what the constructor already set.
        import os as _cs
        v = (_cs.environ.get("OMNISIM_NEWTON_COORD_SYSTEM") or "").strip().lower()
        if v == "":
            return True
        return v not in ("0", "false", "off", "no")

    def set_up_axis(self, name):
        # WorldInfo.coordinateSystem -> the builder's up axis. Accepts either a
        # coordinate-system triple ("ENU" / "NUE" / "EUN" -- the axis is wherever
        # the "U" sits, exactly as OmWorldInfo::updateGravityBasis() resolves it)
        # or a bare axis letter ("X"/"Y"/"Z").
        #
        # ⚠ CALL ORDER IS THE ENTIRE FIX. C++ calls this from ensureWorldOpen()
        # immediately after World() and BEFORE add_ground_plane() / before any
        # body or shape is added, because newton bakes the up vector into the
        # implicit ground plane's NORMAL at add time:
        #   ModelBuilder.add_ground_plane() -> add_shape_plane(plane=(*up_vector, -height))
        # A later call cannot move a plane that already exists. (up_axis itself is
        # a plain mutable attribute -- `self.up_axis: Axis = Axis.from_any(up_axis)`
        # in newton/_src/sim/builder.py -- and nothing in __init__ consumes it, so
        # assigning it right after construction is equivalent to having passed it.)
        #
        # WHY THIS EXISTS. The builder was constructed with a HARDCODED
        # up_axis=Axis.Z and `coordinateSystem` appeared ZERO times in this file,
        # so the 210 NUE (Y-up) worlds in this tree -- 29% of 719, every one of
        # them on bare "auto" physicsBackend -- got BOTH halves of their world
        # model wrong:
        #   * set_gravity() projects WorldInfo's gravity VECTOR onto
        #     builder.up_vector. In NUE that vector is (0,-g,0) and up_vector was
        #     (0,0,1), so the dot product is EXACTLY ZERO and the builder ran at
        #     gravity 0. Measured: a ball released at y=3 read y=3.000 at step
        #     15360 -- it never fell, in any of the 210.
        #   * add_ground_plane() then placed an infinite plane with normal +Z at
        #     z=0. In NUE, z is EAST -- a HORIZONTAL axis -- so the "floor" was a
        #     VERTICAL WALL through the middle of the scene. The same ball drifted
        #     to z=+384 m along it, and in tests/physics/worlds/
        #     template_deterministic.wbt the wall lands exactly where the four
        #     DistanceSensors sit, so all four read 0.000000.
        # ODE masked all of this by being the fall-back backend, and the readiness
        # sweep scored those worlds PASS because a log verdict cannot see gravity:
        # they loaded, they stepped, they logged nothing. Deleting src/ode without
        # this would have broken all 210 silently.
        #
        # Only ENU (Z) and NUE/EUN (Y) are reachable from the schema
        # (WorldInfo.wrl restricts the field to those three), so X-up is accepted
        # here but never produced by a world file.
        if not World._coord_system_enabled():
            return
        s = str(name or "").strip().upper()
        if len(s) == 3 and "U" in s:
            ch = "XYZ"[s.index("U")]
        else:
            ch = s[:1]
        if ch not in ("X", "Y", "Z"):
            return          # unknown / absent -> keep the constructed default
        try:
            self.builder.up_axis = newton.Axis.from_string(ch)
        except Exception as _e:
            self._newton_log("set_up_axis(%r) REFUSED by newton: %s (keeping %s)"
                             % (name, _e, self._up_axis_name))
            return
        if ch != self._up_axis_name:
            self._newton_log("up_axis %s -> %s (WorldInfo.coordinateSystem %r); "
                             "gravity and the implicit ground plane now follow it"
                             % (self._up_axis_name, ch, str(name)))
        self._up_axis_name = ch

    def set_gravity(self, gx, gy, gz):
        # WorldInfo.gravity plumbed from C++ BEFORE finalize(). ModelBuilder
        # gravity is a scalar along up_vector; project the world vector onto it.
        #
        # The projection is LOSSLESS for every world OmniSim can author:
        # WorldInfo.gravity is an SFDouble magnitude and OmWorldInfo derives
        # mGravityVector = -upVector * gravity, so the vector is always
        # anti-parallel to up by construction. It is only lossless, though, once
        # builder.up_vector IS the world's up -- set_up_axis() above must have
        # run, which is why C++ plumbs the axis in ensureWorldOpen() and the
        # gravity here (both pre-finalize). Before that fix a Y-up world
        # projected (0,-g,0) onto (0,0,1) and got 0.
        up = self.builder.up_vector
        self.builder.gravity = (float(gx) * up[0] + float(gy) * up[1]
                                + float(gz) * up[2])
        # Any component of the requested gravity PERPENDICULAR to up is dropped by
        # the scalar projection. Unreachable from a .wbt today (see above), so say
        # so once rather than letting a future vector-gravity field vanish here.
        _res = ((float(gx) - self.builder.gravity * up[0]) ** 2
                + (float(gy) - self.builder.gravity * up[1]) ** 2
                + (float(gz) - self.builder.gravity * up[2]) ** 2)
        if _res > 1e-12 and not getattr(self, "_gravity_residual_logged", False):
            self._gravity_residual_logged = True
            self._newton_log(
                "set_gravity(%g,%g,%g) is NOT parallel to up_axis %s: the "
                "component perpendicular to up (|r|=%.4g m/s^2) is DROPPED -- "
                "newton's ModelBuilder carries gravity as a scalar along "
                "up_vector." % (gx, gy, gz, self._up_axis_name, _res ** 0.5))

    def set_contact_cone(self, cone, impratio):
        # Per-world MuJoCo friction-cone type + impratio (WorldInfo.newtonCone /
        # newtonImpratio), plumbed from C++ before finalize(). "" / 0 = unset
        # (MuJoCo stock: pyramidal, impratio 1). The OMNISIM_NEWTON_CONE /
        # OMNISIM_NEWTON_IMPRATIO env vars still win (resolved in finalize()).
        # Only consumed by the SolverMuJoCo construction path.
        self._cone_world = str(cone) if cone else None
        try:
            self._impratio_world = float(impratio)
        except (TypeError, ValueError):
            self._impratio_world = 0.0

    def set_contact_condim(self, condim):
        # Per-world MuJoCo contact dimensionality (WorldInfo.newtonCondim),
        # plumbed from C++ before finalize(). 0 = unset -> whatever the model
        # already carries (measured: EVERY geom in EVERY OmniSim world is
        # condim 3), so an existing world is byte-identical.
        #
        # ⚠ WHY 3 IS NOT ALWAYS ENOUGH. condim 3 is sliding friction only: the
        # torsional and rolling coefficients newton writes into
        # geom_friction[1:2] are simply not consulted, so a part pinched
        # between two pads is free to SPIN about the contact normal at zero
        # cost. 4 adds torsional friction (the one a two-finger grasp wants;
        # NVIDIA's own newton gripper example sets condim=4 on the finger
        # shapes), 6 adds rolling as well. The OMNISIM_NEWTON_CONDIM env var
        # still wins (resolved in finalize()).
        try:
            self._condim_world = int(condim)
        except (TypeError, ValueError):
            self._condim_world = 0

    def set_noslip_iterations(self, iters):
        # Per-world MuJoCo NOSLIP post-solve iterations
        # (WorldInfo.newtonNoslipIterations -> mjOption.noslip_iterations),
        # plumbed from C++ before finalize(). 0 = unset, which is also MuJoCo's
        # own stock value, so an existing world is byte-identical.
        #
        # WHAT IT IS. A Gauss-Seidel pass over the FRICTION constraints only,
        # run after the main solve. MuJoCo's soft constraint model lets a
        # tangential contact DRIFT under a sustained load even while the normal
        # force is held exactly at its commanded value, and this pass exists to
        # remove that drift. It is therefore the specific remedy for "the
        # gripper is squeezing hard and the part still creeps out", which is a
        # different failure from "the grip is too weak" and does not respond to
        # more force, a firmer contact, or more solver iterations.
        #
        # ⚠ CPU ONLY. mujoco_warp does not implement noslip -- its Option
        # struct has no such field and put_model() RAISES on a non-zero one --
        # so this is applied to mj_model after construction and only on the CPU
        # mj_step path; finalize() warns once and continues otherwise. The
        # OMNISIM_NEWTON_NOSLIP env var still wins and is VALUE-parsed, so =0
        # is the exact-revert hatch for a world that declares the field.
        try:
            self._noslip_iters_world = int(iters)
        except (TypeError, ValueError):
            self._noslip_iters_world = 0

    def set_cloth_self_contact(self, mode):
        # Per-world cloth/soft PARTICLE self-contact
        # (WorldInfo.newtonClothSelfContact -> SolverVBD's
        # particle_enable_self_contact and the self-contact radius/margin that
        # accompany it), plumbed from C++ before finalize(). -1 = unset, so a
        # world that does not declare the field keeps the runtime default (ON)
        # and is byte-identical to before this method existed.
        #
        # ⚠ THERE IS NO CORRECT DEFAULT, WHICH IS WHY IT HAD TO BECOME A FIELD.
        # DRAPING needs self-contact ON: newton's own default is OFF and with
        # it off a fold passes straight THROUGH itself, with no error, no
        # warning and no contact record to notice it by. GRASPING needs it OFF:
        # measured on the patch world a pinched fold tracks to -22.11 mm with
        # it on and -0.92 mm with it off -- 24x -- because the fabric gathered
        # between the pads pushes ITSELF back out of the jaws.
        #
        # Until 2026-08-15 the only way to say either was
        # OMNISIM_CLOTH_SELF_CONTACT, i.e. the shell, not the file. Every
        # deformable-grasp world here needs it off; forgetting the variable did
        # not fail, it slipped 24x in silence, and reads as "cloth grasping
        # does not work in OmniSim" rather than as a missing launch flag.
        # The env var still wins (env > field > default) and is value-parsed,
        # so =1 is the exact-revert hatch for a world declaring 0.
        try:
            m = int(mode)
        except (TypeError, ValueError):
            m = -1
        self._cloth_self_contact_world = m if m in (0, 1) else -1

    def set_contact_solver_params(self, mu, ke, kd, iters, ls_iters):
        # Per-world contact friction / compliance / solver iteration counts
        # (WorldInfo.newtonGroundMu, newtonContactKe, newtonContactKd,
        # newtonIterations, newtonLsIterations), plumbed from C++ before
        # finalize(). 0 = unset on every one of them, so an existing world is
        # byte-identical, and the OMNISIM_NEWTON_* env vars still win.
        #
        # ⚠ WHY THESE FIELDS EXIST AT ALL. Until 2026-08-02 these five knobs
        # were reachable ONLY through process environment variables, with no
        # .wbt representation. A world file was therefore NOT a complete
        # description of its own physics: an agent tuned a working two-finger
        # friction grasp here, wrote the world out, and the next person to load
        # it got default friction and a soft contact and no grasp. Measured on
        # the capability ladder -- the working configuration could not be
        # handed to anybody, including to our own grader, which re-ran the
        # world bare and scored the result a failure.
        def _f(v):
            try:
                return float(v)
            except (TypeError, ValueError):
                return 0.0

        def _i(v):
            try:
                return int(v)
            except (TypeError, ValueError):
                return 0

        World._contact_world = {
            # mu >= 0 is DECLARED (0 = frictionless, a legal value); only a
            # negative means unset. ke/kd keep `> 0` -- zero stiffness or
            # damping is nobody's configuration, so 0 stays their unset.
            "mu": (_f(mu) if _f(mu) >= 0.0 else None),
            "ke": (_f(ke) if _f(ke) > 0.0 else None),
            "kd": (_f(kd) if _f(kd) > 0.0 else None),
        }
        self._iters_world = _i(iters)
        self._ls_iters_world = _i(ls_iters)
        # The shape config is CLASS-cached and may already have been built from
        # the defaults, so drop it or the world's own friction never reaches a
        # shape added after this call.
        World._SHAPE_CFG = None
        cw = World._contact_world
        if cw["mu"] is not None:
            self._ground_mu = cw["mu"]
            self.builder.default_shape_cfg.mu = cw["mu"]
        if cw["ke"] is not None:
            self.builder.default_shape_cfg.ke = cw["ke"]
        if cw["kd"] is not None:
            self.builder.default_shape_cfg.kd = cw["kd"]

    def set_constraint_buffers(self, njmax, nconmax):
        # Per-world MuJoCo constraint-row / contact buffer caps
        # (WorldInfo.newtonNjmax / newtonNconmax), plumbed from C++ before
        # finalize(). 0 = unset -> the built-in 256 default below is kept, so
        # every existing world is byte-identical. A positive value raises the
        # cap (measured: 32 rows per 4WD Husky on its 8 wheel contacts, so 10
        # robots peak at 320 and blow through 256); a NEGATIVE value asks newton for its own
        # auto-estimate. The OMNISIM_NEWTON_NJMAX / OMNISIM_NEWTON_NCONMAX env
        # vars still win (resolved in finalize()). Only consumed by the
        # SolverMuJoCo construction path.
        try:
            self._njmax_world = int(njmax)
        except (TypeError, ValueError):
            self._njmax_world = 0
        try:
            self._nconmax_world = int(nconmax)
        except (TypeError, ValueError):
            self._nconmax_world = 0

    def _newton_log(self, msg):
        # Append one line to the newton runtime log (OMNISIM_NEWTON_LOG, else
        # .build_tmp/newton_solver.log). Embedded-Python stdout does not reach
        # the host reliably -- on Windows omnisim-bin is a GUI-subsystem binary
        # and stdout is discarded outright -- so a file is the only durable
        # channel from inside the runtime.
        try:
            import os as _lo, time as _lt
            p = _lo.environ.get("OMNISIM_NEWTON_LOG",
                                _lo.path.join(_lo.getcwd(), ".build_tmp",
                                              "newton_solver.log"))
            _lo.makedirs(_lo.path.dirname(p), exist_ok=True)
            # ⚠ ENCODING IS EXPLICIT BECAUSE THE DEFAULT SILENTLY EATS LINES.
            # open() without encoding uses the locale codec -- cp1252 on this
            # Windows box -- and a single non-cp1252 character (a "⚠", an arrow,
            # a degree sign) raises UnicodeEncodeError, which the except below
            # then swallows whole. The line does not appear truncated or
            # mangled; it does not appear AT ALL, and the run looks like the
            # code path never executed. MEASURED: the cloth mujoco_warp
            # disclosure added alongside this went missing exactly that way
            # while the flip it was reporting had demonstrably happened.
            # errors="replace" so a stray character degrades one glyph instead
            # of losing the message.
            with open(p, "a", encoding="utf-8", errors="replace") as f:
                f.write("%s %s\n" % (_lt.strftime("%H:%M:%S"), msg))
        except Exception:
            pass

    def _sample_constraint_peaks(self, verbose=False):
        # Host-side replacement for mujoco_warp's un-rate-limitable kernel
        # printf: track the PEAK constraint-row (nefc) and contact (nacon)
        # counts against the caps mujoco_warp ACTUALLY allocated, and latch the
        # FIRST overflow into self._constraint_overflow_msg so C++ step() can
        # raise it once as a OmLog::warning -- i.e. into the engine log that
        # run-headless parses and the Newton verdict sidecar sits next to.
        #
        # Why this must not stay opt-in (N15): mujoco_warp DROPS the row on
        # overflow --
        #     mujoco_warp/_src/constraint.py
        #         efcid = wp.atomic_add(nefc_out, worldid, 1)
        #         if efcid >= njmax_in:
        #             return
        # -- while nefc keeps counting, and the only warning it emits is a
        # wp.printf from INSIDE a warp kernel (mujoco_warp/_src/forward.py:
        # `if nefc > njmax_in: wp.printf("nefc overflow ...")`). On Windows
        # omnisim-bin.exe is a GUI-subsystem binary, so that print is discarded
        # outright; where stdout IS captured it lands in <log>.stdout, which
        # scripts/dev/headless_runner.py opens purely as a sink and never reads
        # back. Exit code, engine log and sidecar therefore all stay clean while
        # contact solving silently degrades, and which rows get dropped is a
        # nondeterministic GPU atomic race.
        #
        # Peaks are monotone, so `verbose` costs a handful of lines for a whole
        # run rather than one per tick. Reading a warp array forces a device
        # sync, hence the coarse sampling cadence set up in finalize(). This is
        # a READ of solver telemetry only -- it changes no state and no physics.
        d = getattr(self.solver, "mjw_data", None)
        if d is None:
            return

        def _peak(name):
            a = getattr(d, name, None)
            if a is None:
                return None
            try:
                import numpy as _np
                return int(_np.max(a.numpy()))
            except AttributeError:
                try:
                    return int(a)
                except (TypeError, ValueError):
                    return None

        def _cap(name, requested_attr):
            # Prefer the cap mujoco_warp ACTUALLY allocated (Data.njmax /
            # Data.naconmax are plain ints) over what we asked for: with
            # newtonNjmax -1 ("auto") the request is 0 here and only the
            # allocated value tells the truth -- which is exactly the N16
            # case where newton's _default_njmax collapses to ~64.
            v = getattr(d, name, None)
            try:
                v = int(v)
            except (TypeError, ValueError):
                v = 0
            return v if v > 0 else int(getattr(self, requested_attr, 0) or 0)
        nefc = _peak("nefc")
        cap_j = _cap("njmax", "_njmax_applied")
        # Contacts: this runtime counts narrowphase contacts in Data.nacon and
        # compares them to Data.naconmax (forward.py does exactly that pairing);
        # older schemas named them ncon/nconmax. Resolve counter and cap as a
        # PAIR so we never compare a count from one schema to a cap from another.
        ncon = _peak("nacon")
        if ncon is None:
            ncon = _peak("ncon")
            cap_c = _cap("nconmax", "_nconmax_applied")
        else:
            cap_c = _cap("naconmax", "_nconmax_applied")
        prev_j = int(getattr(self, "_cs_peak_nefc", 0))
        prev_c = int(getattr(self, "_cs_peak_ncon", 0))
        if not ((nefc is not None and nefc > prev_j)
                or (ncon is not None and ncon > prev_c)):
            return
        self._cs_peak_nefc = max(prev_j, nefc or 0)
        self._cs_peak_ncon = max(prev_c, ncon or 0)
        over = ((cap_j and self._cs_peak_nefc > cap_j)
                or (cap_c and self._cs_peak_ncon > cap_c))
        if verbose or over:
            self._newton_log(
                "constraint peak: nefc=%d/%s ncon=%d/%s tick=%d%s"
                % (self._cs_peak_nefc, cap_j or "auto",
                   self._cs_peak_ncon, cap_c or "auto",
                   int(getattr(self, "_cs_tick", 0)),
                   "  *** OVERFLOW: raise WorldInfo.newtonNjmax / newtonNconmax ***"
                   if over else ""))
        if over and not getattr(self, "_constraint_overflow_msg", ""):
            # Latched ONCE per world; C++ step() picks it up and emits a single
            # OmLog::warning. Never per-tick spam.
            self._constraint_overflow_msg = (
                "CONSTRAINT BUFFER OVERFLOW at tick %d: peak nefc=%d vs njmax=%s, "
                "peak ncon=%d vs nconmax=%s. mujoco_warp SILENTLY DROPS every "
                "constraint row past njmax (and every contact past nconmax), so "
                "contact solving is degrading and the drop order is a "
                "nondeterministic GPU atomic race -- the run will still exit 0 "
                "with a clean log. Fix: raise WorldInfo.newtonNjmax / "
                "newtonNconmax (leave HEADROOM -- sizing to the measured peak is "
                "itself a trap, docs/benchmarks/determinism-scope.md sec.3), or set "
                "OMNISIM_NEWTON_NJMAX / OMNISIM_NEWTON_NCONMAX."
                % (int(getattr(self, "_cs_tick", 0)),
                   self._cs_peak_nefc, cap_j or "auto",
                   self._cs_peak_ncon, cap_c or "auto"))

    def add_ground_plane(self):
        """REQUEST the implicit ground plane. Does not add it -- see finalize().

        C++ calls this from ``ensureWorldOpen()``, before the first Solid is
        known, so nothing here can tell whether the world declares a ground of
        its own. It used to add the plane unconditionally, and that gave every
        Newton world an UNDECLARED, INFINITE collision surface at up-axis 0 that
        appears in no world file and in no scene tree.

        ⚠ WHAT THAT COST, measured on ladder0 rung 1 with the floor Solid's
        ``boundingObject`` removed and nothing else changed: the 0.2 m box did
        not fall. It settled at **z = 0.099892** -- its own half-height, resting
        on a surface no line of the world declares -- in a world that also says
        ``newtonStatics TRUE``. That is exactly the AgentBench C2 fall-through
        case, and it is why ``run-headless --fail-on-runaway`` could no longer
        catch it: the body rests quietly on nothing instead of running away, so
        both the broken world and the fixed one look identical.

        IT IS LOAD-BEARING, THOUGH, AND IN ONE PLACE ONLY. ``add_shape_plane``
        cannot build an authored ``Plane`` collider on this path -- newton's
        MuJoCo converter raises on a plane attached to our weld-pinned statics,
        so every one is deferred and then DROPPED in ``finalize()``. For a world
        whose only ground is a ``Plane``, this implicit plane IS that plane. So
        the plane is not removed; it is made a **declared substitution**, added
        in ``finalize()`` if and only if the world declared a ``Plane`` collider
        that had to be dropped, and logged either way. A world that declares no
        static collision surface at all now genuinely has none.

        ``OMNISIM_NEWTON_GROUND_PLANE=1`` restores the unconditional plane
        exactly (the pre-2026-08-12 build, for a bisect); ``=0`` refuses it even
        for the substitution case.
        """
        # newton composes it as add_shape_plane(plane=(*builder.up_vector,
        # -height)), reading builder.up_axis AT THE MOMENT OF THE CALL -- which
        # is why set_up_axis() must already have run (C++ orders both inside
        # ensureWorldOpen(), and finalize() is later still, so the deferral
        # cannot regress the Y-up case that fix corrected).
        self._want_ground_plane = True

    def add_body(self, mass, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0,
                 ixx=0.0, iyy=0.0, izz=0.0, ixy=0.0, ixz=0.0, iyz=0.0,
                 cx=None, cy=None, cz=None):
        # Inertia tensor: when the caller supplies non-zero diagonal
        # values, use them directly (URDF inertial block from the
        # OmSolid's Physics node). Otherwise fall back to the
        # Husky-tuned mass-threshold preset so legacy worlds without
        # explicit inertia keep working.
        m = float(mass)
        ixx = float(ixx); iyy = float(iyy); izz = float(izz)
        if ixx > 0.0 and iyy > 0.0 and izz > 0.0:
            inertia = wp.mat33(
                (ixx, float(ixy), float(ixz)),
                (float(ixy), iyy, float(iyz)),
                (float(ixz), float(iyz), izz),
            )
        elif m >= 10.0:
            inertia = wp.mat33(
                (0.034 * m, 0.0, 0.0),
                (0.0, 0.097 * m, 0.0),
                (0.0, 0.0, 0.104 * m),
            )
        else:
            inertia = wp.mat33(
                (0.0094 * m, 0.0, 0.0),
                (0.0, 0.0167 * m, 0.0),
                (0.0, 0.0, 0.0094 * m),
            )
        # Link center of mass. Defaults to the link ORIGIN (com=None) for
        # backward compatibility -- this generic runtime is shared by every
        # Newton robot (Spot, huskies, combat bots, ...), all validated against
        # COM-at-origin. The caller (OmSolid.cpp) supplies cx/cy/cz only when
        # OMNISIM_NEWTON_USE_LINK_COM is set (opt-in true URDF COM offset). The
        # URDF inertia tensor above is already about the COM frame, so passing
        # com + that inertia is the physically correct pairing.
        com = (wp.vec3(float(cx), float(cy), float(cz))
               if (cx is not None and cy is not None and cz is not None)
               else None)
        idx = self.builder.add_link(
            xform=wp.transform((float(x), float(y), float(z)),
                               (float(qx), float(qy), float(qz), float(qw))),
            # (newton >=1.4 REMOVED ModelBuilder.add_link(armature=...). Omitting
            # it is byte-equivalent on 1.2.0: armature=None falls back to
            # _default_body_armature, which is 0.0 and which this engine never
            # overrides, and 0.0 adds eye*0.0 to the inertia. One source drives
            # both runtimes.)
            com=com,
            inertia=inertia,
            mass=m,
            label=f"body_{len(self.body_indices)}",
        )
        self.body_indices.append(idx)
        return idx

    def add_static_body(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        # P8.1 statics-on-Newton. Newton treats mass=0 links as static
        # (xform pinned by the solver) but still tests their attached
        # shapes against dynamic-body shapes in the contact phase. The
        # caller attaches collision geometry via add_shape_* after this.
        # Inertia stays zero — Newton ignores it when mass=0 since the
        # body never integrates.
        idx = self.builder.add_link(
            xform=wp.transform((float(x), float(y), float(z)),
                               (float(qx), float(qy), float(qz), float(qw))),
            # (newton >=1.4 REMOVED ModelBuilder.add_link(armature=...). Omitting
            # it is byte-equivalent on 1.2.0: armature=None falls back to
            # _default_body_armature, which is 0.0 and which this engine never
            # overrides, and 0.0 adds eye*0.0 to the inertia. One source drives
            # both runtimes.)
            inertia=wp.mat33((0.0, 0.0, 0.0),
                             (0.0, 0.0, 0.0),
                             (0.0, 0.0, 0.0)),
            mass=0.0,
            label=f"static_body_{len(self.body_indices)}",
        )
        self.body_indices.append(idx)
        self.static_body_indices.append(idx)
        return idx

    def add_kinematic_body(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        # Kernel blocker #4 (_scratch/design_kinematic_inertia.md Part 1):
        # a KINEMATIC body -- a movable static obstacle. Same build recipe as
        # add_static_body (zero-mass link; finalize() pins it to the WORLD
        # with a 0-DOF FIXED joint), because a world-fixed root is exactly
        # what SolverMuJoCo exports as a MuJoCo MOCAP body (solver_mujoco.py:
        # `is_fixed_root = parent == -1 and j_type == JointType.FIXED` ->
        # body_kwargs["mocap"]=True, verified on the bundled newton 1.2.0).
        # A mocap body is static-with-settable-pose: it never integrates, but
        # MuJoCo's own collision tests its geoms against dynamic bodies every
        # step, giving the ODE kinematic semantics (one-way coupling: pushes
        # dynamics, is never pushed back; two mocap bodies generate no
        # response between themselves). The engine keeps ownership of the
        # pose exactly as it does under ODE (motor profiles / supervisor
        # writes / FK all engine-side) and pushes it per change through
        # set_kinematic_pose below. The caller attaches collision geometry
        # via add_shape_* after this, exactly like add_static_body.
        idx = self.builder.add_link(
            xform=wp.transform((float(x), float(y), float(z)),
                               (float(qx), float(qy), float(qz), float(qw))),
            # (newton >=1.4 REMOVED ModelBuilder.add_link(armature=...). Omitting
            # it is byte-equivalent on 1.2.0: armature=None falls back to
            # _default_body_armature, which is 0.0 and which this engine never
            # overrides, and 0.0 adds eye*0.0 to the inertia. One source drives
            # both runtimes.)
            inertia=wp.mat33((0.0, 0.0, 0.0),
                             (0.0, 0.0, 0.0),
                             (0.0, 0.0, 0.0)),
            mass=0.0,
            label=f"kinematic_body_{len(self.body_indices)}",
        )
        self.body_indices.append(idx)
        # Shares the static pinning path in finalize() (FIXED joint to the
        # world at the spawn pose + nominal mass/inertia for the MuJoCo
        # compiler) -- that pin is what makes it a fixed root, hence mocap.
        self.static_body_indices.append(idx)
        if not hasattr(self, "kinematic_body_indices"):
            self.kinematic_body_indices = []
        self.kinematic_body_indices.append(idx)
        return idx

    def set_kinematic_pose(self, body_idx, x, y, z, qx, qy, qz, qw):
        # Per-tick kinematic pose write, effective NEXT step (mj_kinematics
        # copies mocap_pos/quat into xpos/xquat at the start of the next
        # mj_step). Quaternion arrives xyzw (the engine/warp wire convention,
        # same as add_body) and MuJoCo stores WXYZ -- reorder here.
        #
        # ⚠ DO NOT route this through notify_model_changed(JOINT_PROPERTIES):
        # _update_joint_properties writes ONLY mjw_data (solver_mujoco.py:
        # 6236-6237) and _update_mjc_data copies qpos/qvel only, so on the
        # default CPU mj_step path the notify route provably never moves a
        # mocap body (_scratch/design_kinematic_inertia.md sec 1.4b). Write
        # the mocap arrays DIRECTLY, branching CPU vs warp like every other
        # direct write in this runtime (weld_engage, touch_force).
        # Returns 0 on success, -1 when the body is unknown / not a mocap
        # (fixed-root) body / the solver is not up yet.
        sv = self._mjc_solver()
        if sv is None:
            return -1
        m = getattr(sv, "mj_model", None)
        if m is None:
            return -1
        _, body_map = self._constraint_maps()
        mb = body_map.get(int(body_idx))
        if mb is None:
            return -1
        mocapid = int(m.body_mocapid[mb])
        if mocapid < 0:
            return -1
        px, py, pz = float(x), float(y), float(z)
        rx, ry, rz, rw = float(qx), float(qy), float(qz), float(qw)
        if getattr(sv, "use_mujoco_cpu", False):
            d = getattr(sv, "mj_data", None)
            if d is None:
                return -1
            d.mocap_pos[mocapid] = (px, py, pz)
            d.mocap_quat[mocapid] = (rw, rx, ry, rz)     # MUJOCO WXYZ ORDER
        else:
            # mujoco_warp: mocap arrays are per-world warp arrays
            # [nworld, nmocap, 3|4]; numpy round-trip + assign (a host
            # data write -- compatible with a captured step graph, whose
            # kernels read the same buffers).
            dw = getattr(sv, "mjw_data", None)
            if dw is None:
                return -1
            try:
                mp = dw.mocap_pos.numpy()
                mq = dw.mocap_quat.numpy()
                mp[0, mocapid] = (px, py, pz)
                mq[0, mocapid] = (rw, rx, ry, rz)        # MUJOCO WXYZ ORDER
                dw.mocap_pos.assign(mp)
                dw.mocap_quat.assign(mq)
            except Exception:
                return -1
        # Best-effort: keep newton's OWN view of the body fresh. The fixed
        # root's canonical pose is model.joint_X_p (the mocap kernel derives
        # mocap_pos from it on the warp notify path), and eval_fk re-derives
        # body_q from it every tick -- so without this write the newton-side
        # contact readback (get_contacts serves newton's narrow-phase buffer)
        # and body_xform would report the kinematic body at its SPAWN pose
        # for ever. Physics is already correct from the mocap write above;
        # this only keeps the diagnostics/readback honest.
        try:
            j = getattr(self, "_body_fixed_joint", {}).get(int(body_idx))
            if j is not None and self.model is not None:
                jxp = self.model.joint_X_p.numpy()
                jxp[j] = (px, py, pz, rx, ry, rz, rw)    # warp transform: p + q(xyzw)
                self.model.joint_X_p.assign(jxp)
        except Exception:
            pass
        return 0

    # density=0 on every collision shape -- mass comes from add_link's
    # `mass=` argument and we don't want shapes silently inflating it.
    # The default density 1000 added ~19 kg per wheel-sphere in probe 9
    # (chassis traveled 47% of theoretical instead of 98%); zero density
    # is mandatory.
    _SHAPE_CFG = None
    #: Per-world contact values from WorldInfo, set by set_contact_solver_params.
    #: Class-level because _shape_cfg is a classmethod with a class-level cache
    #: and cannot see instance state; None on any key means "not declared".
    _contact_world = {"mu": None, "ke": None, "kd": None}

    @staticmethod
    def _contact_value(env_name, world_value, default):
        """env var > WorldInfo field > default -- the house precedence.

        The env var stays on top so an existing launch script keeps working;
        the world field exists so a .wbt can describe its own physics instead
        of depending on the shell that started it.
        """
        import os as _muos
        raw = _muos.environ.get(env_name)
        if raw:
            try:
                return float(raw)
            except (TypeError, ValueError):
                pass
        if world_value is not None:
            return float(world_value)
        return float(default)

    @classmethod
    def _shape_cfg(cls):
        if cls._SHAPE_CFG is None:
            _cw = cls._contact_world
            _mu = cls._contact_value("OMNISIM_NEWTON_GROUND_MU", _cw.get("mu"), 1.0)
            _ke = cls._contact_value("OMNISIM_NEWTON_CONTACT_KE", _cw.get("ke"), 2500.0)
            _kd = cls._contact_value("OMNISIM_NEWTON_CONTACT_KD", _cw.get("kd"), 100.0)
            cls._SHAPE_CFG = newton.ModelBuilder.ShapeConfig(density=0.0, mu=_mu, ke=_ke, kd=_kd)
        return cls._SHAPE_CFG

    def add_shape_sphere(self, body_idx, radius, cx=0.0, cy=0.0, cz=0.0):
        return self.builder.add_shape_sphere(
            int(body_idx),
            xform=wp.transform((float(cx), float(cy), float(cz)),
                               (0.0, 0.0, 0.0, 1.0)),
            radius=float(radius),
            cfg=self._shape_cfg(),
        )

    def add_shape_box(self, body_idx, hx, hy, hz, cx=0.0, cy=0.0, cz=0.0, ke=-1.0,
                      qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        # (qx,qy,qz,qw) is the collider's authored orientation in the body frame (identity
        # default). It is appended AFTER ke so the existing positional order is untouched.
        # A rotated box collider was previously flattened to axis-aligned.
        cfg = self._shape_cfg()
        if ke and float(ke) > 0.0:                    # per-shape SOFT contact:
            # a low ke -> larger MuJoCo solref timeconst -> compliant contact.
            # Used for tote/bin CONTENTS (e.g. cubes on a dynamic bin floor) so
            # the gripper plowing the layer can't inject launch energy; MuJoCo
            # mixes the contact toward the softer geom, so only the soft side
            # needs marking while the structure/grasp stays firm.
            import os as _os2
            _mu = float(_os2.environ.get("OMNISIM_NEWTON_GROUND_MU", "1.0"))
            _kd = float(_os2.environ.get("OMNISIM_NEWTON_SOFT_KD", "150"))
            cfg = newton.ModelBuilder.ShapeConfig(density=0.0, mu=_mu,
                                                  ke=float(ke), kd=_kd)
        return self.builder.add_shape_box(
            int(body_idx),
            xform=wp.transform((float(cx), float(cy), float(cz)),
                               (float(qx), float(qy), float(qz), float(qw))),
            hx=float(hx), hy=float(hy), hz=float(hz),
            cfg=cfg,
        )

    def add_shape_cylinder(self, body_idx, radius, half_height, cx=0.0, cy=0.0, cz=0.0,
                           qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        # W1.2 (newton-ode-replacement-plan.md): Newton's native cylinder narrow-phase locks wheels against
        # the ground (probe 7), so substitute a CAPSULE of the same radius + half_height instead of the old
        # sphere. A capsule's central segment is a true cylinder of the correct rolling radius -- a LINE
        # contact of the right width, strictly more faithful than a point-contact sphere -- and the capsule
        # narrow-phase is Newton's most robust (no lock).
        #
        # AXIS: both are Z-ALIGNED, so the substitution needs no correction. (qx,qy,qz,qw) is the
        # collider's AUTHORED orientation in the body frame, composed by OmSolid's boundingObject
        # walkers from the Pose/Transform chain; identity default keeps a positional caller working.
        import os as _os
        if _os.environ.get("OMNISIM_NEWTON_CYLINDER_AS_SPHERE"):
            # Revert lever (matches OMNISIM_NEWTON_MESH_TO_ODE): the pre-W1.2 point-contact sphere, kept so
            # the capsule fit can be A/B'd and disabled if a world ever regresses.
            return self.builder.add_shape_sphere(
                int(body_idx),
                xform=wp.transform((float(cx), float(cy), float(cz)), (0.0, 0.0, 0.0, 1.0)),
                radius=float(radius), cfg=self._shape_cfg())
        # ⚠ THIS USED TO APPLY A FIXED -90 DEG ABOUT X AND IT WAS WRONG.
        #
        # The stated premise -- "a Webots Cylinder bounding object extends along
        # its body-local Y" -- is false. An OmniSim Cylinder is Z-ALIGNED
        # (docs/reference/cylinder.md; OmCylinder::rescale), a URDF <cylinder>
        # is Z-aligned (ROS convention), and a newton capsule is Z-aligned.
        # Nothing needed rotating. The -90 was generalised from WHEEL worlds,
        # where the rotation actually lives on the authored Pose.
        #
        # Two defects compounded to make that look correct:
        #   1. the collider walkers accumulated Pose TRANSLATION ONLY, dropping
        #      any rotation authored inside a boundingObject, and
        #   2. this function invented a fixed -90 about X.
        # On Husky the wheel Pose carries rpy 1.570795 about X, so the dropped
        # rotation and the invented one differed only in SIGN -- and a capsule
        # is symmetric about its centre, so +90 and -90 about X describe the
        # SAME collider. The two cancelled and wheels rolled. On the OMNIARM6 arm
        # every collision origin is rpy "0 0 0", so there was nothing to cancel:
        # each arm capsule was tipped from Z to Y and ended up lying CROSSWISE
        # through its own link (link2's became a 0.49 m bar spanning
        # y -0.115..+0.375). Only parent-child shape pairs are contact-excluded,
        # so non-adjacent links then interpenetrated deeply in folded poses --
        # which is what a park pose is -- and the articulation was ejected
        # rather than held.
        #
        # DATED: the capsule substitution landed 2026-06-08 (W1.2,
        # newton-ode-replacement-plan.md:161). A compiled artifact from
        # 2026-06-05 still shows these colliders as point spheres, so the OMNIARM6
        # demos were validated BEFORE the arm grew collision bars, and nothing
        # in tests/ pinned this axis -- which is why it survived two months.
        #
        # BOTH halves are now fixed: OmSolid's walkers compose the authored
        # Pose rotation and hand it over as (qx,qy,qz,qw), and the invented -90
        # is gone. Correct for unrotated colliders, and unchanged for wheels by
        # the symmetry above. Gate on the 8-Husky regression (56.579 m) and on
        # tests/test_newton_cylinder_axis.py, which pins the axis.
        #
        # ⚠ AXIAL EXTENT: A CAPSULE IS LONGER THAN THE CYLINDER IT REPLACES.
        # newton's capsule spans (half_height + radius) each way from its centre
        # -- the cylindrical core PLUS a hemispherical cap at each end -- while
        # the authored cylinder spans half_height. Substituting the authored
        # half_height verbatim therefore grows the collider by one radius at
        # EACH end, i.e. by 2*radius overall, along an axis the author never
        # gave it. Shorten the core so the capsule's TOTAL extent matches the
        # cylinder's: the substitute stays inside the authored bounds instead of
        # protruding out of them.
        #
        # WHY THIS MATTERS AND WHY IT ONLY SURFACED NOW. While every cylinder
        # was being tipped 90 deg (the invented -90, fixed just above) the
        # overhang pointed SIDEWAYS, into free space, and merely made links
        # fatter than authored. With the axis corrected the overhang points
        # ALONG the link -- and for a link whose authored cylinder already sits
        # near a surface, the two extra radii push the collider THROUGH it.
        # MEASURED on omniarm6_2f140_pick_place: the base-yaw link body_6 sits at
        # the world origin carrying an r=0.10 / half_height=0.115 cylinder at
        # z=0.17, so the authored solid spans z 0.055..0.285 -- clear of the
        # floor -- while the capsule spanned -0.045..0.385 and sank 45 mm into
        # the ground plane. At this world's mu=6 that buried contact locked the
        # base yaw outright: joint 1 was commanded -59.6 deg for the carry and
        # never left +0.2 deg, so the arm never swung to the place table, the
        # place descent then tried to make up 0.88 m in flight, and the block
        # was flung to the floor at (1.240, 0.214) instead of placed.
        #
        # ⚠ DEGENERATE CASE, DELIBERATELY LEFT ALONE: when half_height <= radius
        # the correction would drive the core to zero or negative and collapse
        # the capsule to a SPHERE -- precisely the point-contact collider W1.2
        # replaced because it locked wheels. A Husky wheel is exactly this shape
        # (length 0.1143 -> half_height 0.05715, radius 0.1651), so wheels keep
        # today's slightly-over-long capsule and the 8-Husky datum is untouched.
        # A disc-shaped collider has no good capsule stand-in; over-long beats
        # point contact, and the honest fix for that case is a real cylinder
        # geom, not a different capsule.
        _r_sub = float(radius)
        _h_sub = float(half_height)
        if _h_sub > _r_sub:
            _h_sub -= _r_sub
        return self.builder.add_shape_capsule(
            int(body_idx),
            xform=wp.transform((float(cx), float(cy), float(cz)),
                               (float(qx), float(qy), float(qz), float(qw))),
            radius=_r_sub,
            half_height=_h_sub,
            cfg=self._shape_cfg(),
        )

    def add_shape_capsule(self, body_idx, radius, half_height, cx=0.0, cy=0.0, cz=0.0,
                          qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        # An authored Capsule boundingObject used to lose BOTH its offset and its rotation here --
        # this call carried no xform at all, so a capsule authored inside a Pose collided at the
        # body origin, unrotated, however the .wbt placed it. Same Z-aligned convention as the
        # cylinder above (OmCapsule is Z-aligned; so is a newton capsule).
        return self.builder.add_shape_capsule(
            int(body_idx),
            xform=wp.transform((float(cx), float(cy), float(cz)),
                               (float(qx), float(qy), float(qz), float(qw))),
            radius=float(radius),
            half_height=float(half_height),
            cfg=self._shape_cfg())

    def add_shape_plane(self, body_idx, cx=0.0, cy=0.0, cz=0.0):
        # Infinite static ground plane (e.g. a Floor's boundingObject), local normal +Z (the OmPlane
        # convention -- see OmPlane::createOdeGeom -- which matches add_shape_plane's xform-local-Z normal).
        # The body's world transform orients it; (cx,cy,cz) is the local offset. Large width/length so it's
        # effectively infinite. newton-ode-replacement-plan.md W1.1.
        #
        # ALWAYS deferred to finalize() (was: only planes on weld-pinned statics). newton's MuJoCo
        # converter raises "Planes can only be attached to static bodies" for a plane on ANY non-worldbody
        # -- not just our weld-pinned statics but ALSO an ordinary Solid body (e.g. a RectangleArena floor,
        # whose Solid is a dynamic-by-default body). That rejection silently failed the WHOLE mjwarp build
        # and dropped the sim to XPBD -- a DIFFERENT solver -- so an mujoco_warp-trained policy collapsed
        # (the long-running G1 deploy gap). Deferring lets finalize() resolve it once the solver preference
        # is known: dropped under MuJoCo (the default ground plane already provides the z=0 floor), re-added
        # verbatim under XPBD. (var name kept for churn; it now holds non-static planes too.)
        self._deferred_static_planes.append(
            (int(body_idx), float(cx), float(cy), float(cz)))
        return 0

    def add_shape_mesh(self, body_idx, vertices, indices, n_vertices, cx=0.0, cy=0.0, cz=0.0,
                       qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        # Native triangle-mesh collision (newton-ode-replacement-plan.md W1) -- replaces the old AABB-box
        # approximation. `vertices` is a flat [x0,y0,z0, x1,y1,z1, ...] list, `indices` a flat 3-per-triangle
        # list of vertex indices. compute_inertia=False: the body's mass + inertia are already set by
        # add_body() (from the Solid's Physics node), and a collision mesh is often not closed (two-manifold)
        # so Newton's auto inertia would be invalid + costly; the shape cfg has density=0 so the mesh adds no
        # mass regardless.
        import numpy as _np
        verts = _np.asarray(vertices, dtype=_np.float32).reshape(int(n_vertices), 3)
        tris = _np.asarray(indices, dtype=_np.int32)
        mesh = newton.Mesh(verts, tris, compute_inertia=False, is_solid=True)
        # (qx,qy,qz,qw): the collider's authored orientation in the body frame. Was hard identity,
        # which tipped any mesh collision authored inside a rotated Pose -- which the URDF importer
        # emits routinely.
        return self.builder.add_shape_mesh(
            int(body_idx),
            xform=wp.transform((float(cx), float(cy), float(cz)),
                               (float(qx), float(qy), float(qz), float(qw))),
            mesh=mesh,
            cfg=self._shape_cfg(),
        )

    def add_shape_heightfield(self, body_idx, heights, x_dimension, y_dimension,
                              x_spacing, y_spacing, cx=0.0, cy=0.0, cz=0.0):
        # NATIVE heightfield collision for an ElevationGrid boundingObject. Before this the
        # ElevationGrid fell off the end of OmSolid's geometry if/else chain and registered NO
        # collider at all -- measured on the lane-4 probe, a 1 kg sphere dropped on a flat grid
        # reached z=-42.89 and was still accelerating at 3 s (before 2026-08-12 the implicit z=0
        # plane masked this as a rest at 0.0996; it no longer does, so terrain worlds now fall
        # for ever).
        #
        # THREE CONVERSIONS, none of them guessable:
        #
        # 1. hx/hy are HALF-EXTENTS, not cell spacing -- newton's field spans [-hx, +hx]
        #    (newton/_src/geometry/types.py, Heightfield docstring). OmniSim authors an
        #    ElevationGrid by CELL SPACING and a dimension COUNT, so the span is
        #    spacing * (dimension - 1) and the half-extent is half of that.
        #
        # 2. VRML puts the grid's (0,0) CORNER at the geometry origin; newton centres the field
        #    on its xform. The +width/2, +depth/2 shift is the same correction OmElevationGrid
        #    already applied on the ODE path (mLocalOdeGeomOffsetPosition, OmElevationGrid.cpp:413).
        #
        # 3. add_shape_heightfield takes NO body argument -- a newton heightfield is always a
        #    static, world-attached shape (zero mass, zero inertia by construction). So the
        #    owning Solid's world transform has to be BAKED into the xform here rather than
        #    inherited from a body. We read it back out of the builder, which is the same
        #    xform add_link() was given.
        #
        # Axis convention matches with no rotation: OmniSim lays an ElevationGrid's vertices out
        # as (x*xSpacing, y*ySpacing, height) -- grid in local XY, elevation along local Z
        # (OmElevationGrid.cpp:598) -- which is exactly newton's Heightfield layout.
        import numpy as _np
        nx = int(x_dimension)
        ny = int(y_dimension)
        if nx < 2 or ny < 2:
            print("[omnisim-newton] ElevationGrid needs xDimension and yDimension >= 2 to be a "
                  "collider (got %d x %d); registering no heightfield." % (nx, ny), flush=True)
            return -1
        data = _np.asarray(heights, dtype=_np.float32)
        if data.size != nx * ny:
            print("[omnisim-newton] ElevationGrid height[] has %d values but xDimension*yDimension "
                  "is %d; registering no heightfield." % (data.size, nx * ny), flush=True)
            return -1
        # OmniSim's height[] is row-major with x varying fastest (index = y * xDimension + x),
        # so the natural reshape is (ny, nx) == (nrow, ncol).
        data = data.reshape(ny, nx)
        width = float(x_spacing) * (nx - 1)   # total span along local X
        depth = float(y_spacing) * (ny - 1)   # total span along local Y
        # Compose the owning body's world xform with the corner->centre correction, so the
        # terrain lands where the .wbt says it does.
        bx, by, bz = 0.0, 0.0, 0.0
        bq = (0.0, 0.0, 0.0, 1.0)
        try:
            bq_row = self.builder.body_q[int(body_idx)]
            bx, by, bz = float(bq_row[0]), float(bq_row[1]), float(bq_row[2])
            bq = (float(bq_row[3]), float(bq_row[4]), float(bq_row[5]), float(bq_row[6]))
        except (IndexError, TypeError, ValueError):
            # body_idx < 0 (world-attached) or a body the builder does not know: fall back to
            # the local offset alone, which is correct for a terrain authored at the origin.
            pass
        body_xf = wp.transform((bx, by, bz), bq)
        local_xf = wp.transform(
            (float(cx) + width * 0.5, float(cy) + depth * 0.5, float(cz)),
            (0.0, 0.0, 0.0, 1.0))
        hfield = newton.Heightfield(data, nrow=ny, ncol=nx,
                                    hx=width * 0.5, hy=depth * 0.5)
        return self.builder.add_shape_heightfield(
            xform=wp.transform_multiply(body_xf, local_xf),
            heightfield=hfield,
            cfg=self._shape_cfg(),
        )

    # ------------------------------------------------------------------
    # CLOTH AUTHORING
    # ------------------------------------------------------------------

    def has_cloth(self):
        """True once any PARTICLE SOURCE has been authored into the builder.

        Every cloth branch in finalize() and step() is gated on this, so the
        rigid-only path is unchanged (and unchanged-cost) on the 863 worlds in
        this tree that carry no particles.

        ⚠ THE NAME IS HISTORICAL AND NOW NARROWER THAN THE MEANING. Every branch
        this gates -- builder.color(), the CUDA device pin, the coupled-solver
        construction -- is about PARTICLES EXISTING, not about fabric. A
        volumetric soft body is particles + tetrahedra and needs all three, so it
        answers yes here too. Kept under the old name because the six-site
        `solver_soft` sentinel and the ownership assertions in
        _build_cloth_coupled_solver already spell "cloth" throughout; renaming
        the whole family is a separate, mechanical change.

        Soft bodies are safe to fold in here for a reason that is worth stating,
        because the Rope work proved the converse: a soft body adds PARTICLES
        ONLY, so the "mjc" ModelView still owns every rigid body and the
        index-identity check below stays true. MEASURED at 1 and 3 bodies:
        view("mjc").body_count equals the parent model's. A rod, by contrast,
        adds rigid capsule BODIES and moves them out of "mjc", which breaks
        raycast / weld / TouchSensor readbacks that index by parent body id.
        """
        return bool(self.cloth_grids) or bool(self.soft_grids)

    def has_soft_bodies(self):
        """True once any volumetric (tet FEM) soft body has been authored."""
        return bool(self.soft_grids)

    def soft_body_count(self):
        return len(self.soft_grids)

    def cloth_particle_count(self):
        """Number of particles owned by cloth, 0 when there is none."""
        if self.cloth_particle_start < 0:
            return 0
        return int(self.cloth_particle_end - self.cloth_particle_start)

    def add_cloth_grid(self, pos_x, pos_y, pos_z,
                       qx=0.0, qy=0.0, qz=0.0, qw=1.0,
                       dim_x=16, dim_y=16, cell_x=0.05, cell_y=0.05,
                       mass=0.1, particle_radius=0.01,
                       tri_ke=1.0e5, tri_ka=-1.0, tri_kd=-1.0,
                       edge_ke=0.01, edge_kd=-1.0,
                       fix_left=False, fix_right=False,
                       fix_top=False, fix_bottom=False,
                       vx=0.0, vy=0.0, vz=0.0,
                       label=None):
        """Author one rectangular cloth sheet. Returns (particle_start, particle_end).

        This is the C++-callable entry point: it takes only scalars (the ctypes
        FFI boundary this runtime is driven across cannot pass wp.vec3 / wp.quat),
        composes them into warp types, and forwards to
        ``newton.ModelBuilder.add_cloth_grid``, whose full signature is
        keyword-only (builder.py:8931 in the vendored newton 1.5.0).

        Geometry follows newton's own convention: the grid is authored in the
        XY plane of its local frame, ``(dim_x + 1) * (dim_y + 1)`` particles,
        origin at ``pos`` (the grid's CORNER, not its centre -- a caller wanting
        a centred sheet passes ``pos = centre - (dim*cell)/2``). ``rot`` turns
        that plane into whatever orientation the world wants; a horizontal sheet
        in a z-up world is the identity quaternion.

        Args mirror upstream's, with two OmniSim-side conveniences:
          * ``tri_ka`` / ``tri_kd`` / ``edge_kd`` default to ``-1.0`` meaning
            "derive from tri_ke / edge_ke" using the ratios upstream's
            example_mujoco_vbd_coupled_solver.py:268-284 uses (ka = ke,
            kd = 1e-2 * ke) -- so a caller that knows only one stiffness number
            still gets a physically sensible sheet. ⚠ What those terms MEAN,
            per the VBD kernel (which is authoritative -- an upstream comment
            calling ``tri_ka`` "shear stiffness" is wrong): ``tri_ke`` is the
            in-plane DISTORTION term and covers stretch and shear jointly,
            ``tri_ka`` is the AREA / dilation term, and ``edge_ke`` is bending
            across a shared edge. A "stiffer cloth" knob is ``tri_ke``; a
            "resists ballooning" knob is ``tri_ka``.
          * Parameters that are INERT under SolverVBD and therefore not
            exposed here at all: ``tri_drag`` / ``tri_lift`` (aerodynamics),
            ``particle_ke/kd/kf/mu``, ``soft_contact_kf`` and
            ``soft_contact_restitution``. VBD reads
            ``model.soft_contact_ke/kd/mu`` for BOTH shape contact and
            self-contact -- there is no separate self-contact material -- so
            those three (set in _build_cloth_coupled_solver) are the whole
            contact-material story.
          * the quaternion is xyzw (warp's order, and OmniSim's everywhere else
            in this file), NOT mujoco's wxyz.

        ⚠ Cloth is what forces the whole coupled-solver path in finalize(): it
        sets ``self.cloth_grids``, and from that one fact follow the VBD entry,
        the CollisionPipeline, ``builder.color(include_bending=True)`` and the
        CUDA device pin. Calling this AFTER finalize() does nothing useful --
        the builder has already been consumed -- so it must run during the
        build phase like every other add_* on this class.
        """
        if self.builder is None:
            raise RuntimeError("add_cloth_grid called with no builder (post-finalize?)")
        p_start = int(len(self.builder.particle_q))
        _tri_ke = float(tri_ke)
        _tri_ka = float(tri_ka) if float(tri_ka) >= 0.0 else _tri_ke
        _tri_kd = float(tri_kd) if float(tri_kd) >= 0.0 else 1.0e-2 * _tri_ke
        _edge_ke = float(edge_ke)
        _edge_kd = float(edge_kd) if float(edge_kd) >= 0.0 else 1.0e-2 * _edge_ke
        self.builder.add_cloth_grid(
            pos=wp.vec3(float(pos_x), float(pos_y), float(pos_z)),
            rot=wp.quat(float(qx), float(qy), float(qz), float(qw)),
            vel=wp.vec3(float(vx), float(vy), float(vz)),
            dim_x=int(dim_x),
            dim_y=int(dim_y),
            cell_x=float(cell_x),
            cell_y=float(cell_y),
            mass=float(mass),
            fix_left=bool(fix_left),
            fix_right=bool(fix_right),
            fix_top=bool(fix_top),
            fix_bottom=bool(fix_bottom),
            tri_ke=_tri_ke,
            tri_ka=_tri_ka,
            tri_kd=_tri_kd,
            edge_ke=_edge_ke,
            edge_kd=_edge_kd,
            particle_radius=float(particle_radius),
            label=(str(label) if label else None),
        )
        p_end = int(len(self.builder.particle_q))
        rec = {
            "start": p_start, "end": p_end,
            "dim_x": int(dim_x), "dim_y": int(dim_y),
            "cell_x": float(cell_x), "cell_y": float(cell_y),
            "mass": float(mass), "particle_radius": float(particle_radius),
            "tri_ke": _tri_ke, "tri_ka": _tri_ka, "tri_kd": _tri_kd,
            "edge_ke": _edge_ke, "edge_kd": _edge_kd,
            "label": (str(label) if label else "cloth_%d" % len(self.cloth_grids)),
        }
        self.cloth_grids.append(rec)
        # The union range over every sheet. add_cloth_grid appends contiguously,
        # so with N sheets this is [first sheet's start, last sheet's end) and
        # covers all of them -- which is what the VBD entry and the proxy
        # mapping want (one particle list, not N).
        if self.cloth_particle_start < 0:
            self.cloth_particle_start = p_start
        self.cloth_particle_end = p_end
        self._newton_log(
            "cloth: %s particles [%d,%d) grid %dx%d cell %.4gx%.4g mass %.4g "
            "radius %.4g tri_ke %.4g edge_ke %.4g"
            % (rec["label"], p_start, p_end, int(dim_x), int(dim_y),
               float(cell_x), float(cell_y), float(mass), float(particle_radius),
               _tri_ke, _edge_ke))
        return (p_start, p_end)

    @staticmethod
    def _clean_cloth_mesh(verts, tris):
        """Weld duplicates, drop degenerate/duplicate triangles, drop orphans.

        Returns (verts, tris, stats). ⚠ ORDER-PRESERVING AND AN EXACT IDENTITY ON
        ALREADY-CLEAN INPUT -- that property is load-bearing, not tidiness. The
        C++ renderer keeps its OWN copy of the vertex list (it has to: it draws
        the mesh before the first readback exists) and pairs it with the particle
        range this call returns, so any silent REORDER here would leave the drawn
        surface indexing the right count of wrong particles -- a shirt rendered
        as noise, with no error anywhere. That rules out the obvious
        ``np.unique(axis=0)`` one-liner, which sorts lexicographically.

        WHY IT RUNS AT ALL WHEN OmCloth HAS ALREADY CLEANED. Two callers, two
        different levels of trust: OmCloth cleans because it needs the final
        vertex order for its vertex buffer, and this cleans because
        ``add_cloth_mesh`` is reachable from any Python driving this runtime, and
        the three defects below are silent in newton and catastrophic in the
        world. From OmCloth this pass is a no-op and every stat is 0; the caller
        asserts exactly that by comparing particle counts.

        The three, all verified in the vendored newton 1.5.0 source:

          1. ORPHAN VERTICES BECOME KINEMATIC PINS. ``add_cloth_mesh`` adds every
             particle at ``mass=0.0`` (builder.py:9154) and then accumulates
             ``density * area / 3`` onto the three corners of each triangle
             (:9173-9178). A vertex no triangle references is never credited, so
             it keeps mass 0 -- which in every newton solver means infinite
             inverse mass, i.e. IMMOVABLE. A stray vertex in an exported garment
             is therefore a nail driven through the fabric in mid-air.
          2. DUPLICATE VERTICES ARE NEVER WELDED. Bending edges are derived by
             ``MeshAdjacency`` keyed on integer INDICES, so two coincident
             vertices at a UV seam are two unrelated particles: the garment has
             no stretch and no bending across that seam and splits open there.
          3. DEGENERATE TRIANGLES desync the parallel arrays. They are dropped
             with a bare ``print`` and ``tri_areas`` then no longer lines up with
             ``tri_indices`` (builder.py:8592-8594).

        A duplicate FACE is dropped too, though newton says nothing about it: it
        is two elements over one patch of fabric, so that patch silently gets
        double stiffness and double mass.
        """
        import numpy as _np
        stats = {"welded": 0, "degenerate": 0, "duplicate_tris": 0, "orphans": 0}
        verts = _np.asarray(verts, dtype=_np.float64).reshape(-1, 3)
        tris = _np.asarray(tris, dtype=_np.int64).reshape(-1, 3)

        # Drop triangles that index outside the vertex array BEFORE anything else
        # reads through them -- a bad index would otherwise fault numpy's fancy
        # indexing with a message naming none of this.
        bad = (tris < 0).any(axis=1) | (tris >= len(verts)).any(axis=1)
        if bad.any():
            stats["out_of_range"] = int(bad.sum())
            tris = tris[~bad]

        # (1) Weld, first-occurrence order. `first[k]` is where cluster k debuts;
        # ranking clusters by that debut reproduces the input order exactly when
        # there is nothing to weld, which np.unique's sorted output would not.
        uniq, first, inv = _np.unique(verts, axis=0, return_index=True, return_inverse=True)
        if len(uniq) != len(verts):
            order = _np.argsort(first)
            rank = _np.empty(len(uniq), dtype=_np.int64)
            rank[order] = _np.arange(len(uniq), dtype=_np.int64)
            stats["welded"] = int(len(verts) - len(uniq))
            verts = verts[first[order]]
            tris = rank[inv.reshape(-1)[tris]]

        # (2) Degenerate: two corners are the same particle after welding.
        deg = (tris[:, 0] == tris[:, 1]) | (tris[:, 1] == tris[:, 2]) | (tris[:, 0] == tris[:, 2])
        if deg.any():
            stats["degenerate"] = int(deg.sum())
            tris = tris[~deg]

        # (3) Duplicate faces, order preserved by re-sorting the kept indices.
        _u, keep = _np.unique(_np.sort(tris, axis=1), axis=0, return_index=True)
        if len(keep) != len(tris):
            stats["duplicate_tris"] = int(len(tris) - len(keep))
            tris = tris[_np.sort(keep)]

        # (4) Orphans, again order-preserving (the mask keeps ascending order).
        used = _np.zeros(len(verts), dtype=bool)
        if len(tris):
            used[tris.reshape(-1)] = True
        if not used.all():
            remap = _np.full(len(verts), -1, dtype=_np.int64)
            remap[used] = _np.arange(int(used.sum()), dtype=_np.int64)
            stats["orphans"] = int((~used).sum())
            verts = verts[used]
            tris = remap[tris]
        return verts, tris, stats

    def add_cloth_mesh(self, pos_x, pos_y, pos_z,
                       qx=0.0, qy=0.0, qz=0.0, qw=1.0,
                       vertices=None, indices=None, n_vertices=0,
                       density=0.3, particle_radius=0.01,
                       tri_ke=1.0e5, tri_ka=-1.0, tri_kd=-1.0,
                       edge_ke=0.01, edge_kd=-1.0,
                       scale=1.0, pin_top_band=0.0, vx=0.0, vy=0.0, vz=0.0,
                       label=None):
        """Author one cloth sheet from an ARBITRARY TRIANGLE MESH -- a garment.

        Returns (particle_start, particle_end). The mesh twin of add_cloth_grid
        above, and C++-callable in the same sense: `vertices` is a FLAT
        [x0,y0,z0, x1,y1,z1, ...] sequence and `indices` a FLAT 3-per-triangle
        sequence, the same shape add_shape_mesh already takes across this FFI,
        because the ctypes boundary cannot pass wp.vec3 / wp.quat.

        ⚠ `density` IS NOT `add_cloth_grid`'s `mass`, AND THE UNITS DIFFER.
        add_cloth_grid takes a PER-PARTICLE mass in kg; upstream's add_cloth_mesh
        takes a mass PER UNIT AREA in kg/m^2, and distributes it by triangle area
        so a fine patch and a coarse patch of the same fabric weigh the same. The
        default 0.3 kg/m^2 is a light cotton jersey; the vendored T-shirt is
        0.6695 m^2 of surface, so it lands at ~0.20 kg. Passing a grid's 0.001
        here would give a garment weighing under a gram.

        PINNING is `pin_top_band`, in metres, and it is the ONE thing here with
        no upstream equivalent. add_cloth_grid's fix_left/right/top/bottom name
        the extremes of a RECTANGULAR LATTICE, which a garment has none of, so
        upstream's mesh entry point offers no pinning at all. The rule this
        substitutes is the simplest one that means something on any mesh: pin
        every vertex within `pin_top_band` metres of the mesh's MAXIMUM LOCAL Z,
        measured in the mesh's own frame BEFORE scale/rot/pos. On a garment
        authored as worn that band is the shoulders and the neck, so a shirt
        hangs the way it would on a hanger. 0.0 disables it.

        ⚠ IT IS NOT OPTIONAL IN PRACTICE TODAY, and the reason is measured, not
        stylistic. UNPINNED CLOTH RESTING ON A STATIC BODY SINKS THROUGH IT on
        this stack: a 616-particle shirt dropped on a static box passed through
        it and kept falling, and so did a matched 196-particle GRID sheet from
        the same height onto the same box (faster, in fact -- z = -1.20 by 8.4 s
        against the shirt's -0.42 at 5.4 s). So the defect is the existing
        cloth-vs-statics path, NOT mesh authoring -- but the consequence is that
        "drape it over something" is not yet a durable way to support a garment,
        and pinning is. The shipped grid drape world looks fine only because its
        sheet is pinned and merely brushes the box.

        Pinning is applied AFTER upstream's call, which is forced rather than
        chosen: add_cloth_mesh adds every particle at mass 0 and then ACCUMULATES
        density*area/3 from each incident triangle, so a mass zeroed before that
        would simply be filled back in.

        Order of operations upstream (builder.py:9146-9151) is SCALE, then ROT,
        then POS -- so `scale` multiplies the mesh's own coordinates before the
        rigid placement, and a mesh already authored in metres wants scale=1.

        Stiffness sentinels follow add_cloth_grid exactly: tri_ka / tri_kd /
        edge_kd NEGATIVE means "derive from the matching ke" (ka = ke,
        kd = 1e-2 * ke). Zero is a literal zero on all five.
        """
        if self.builder is None:
            raise RuntimeError("add_cloth_mesh called with no builder (post-finalize?)")
        import numpy as _np
        if vertices is None or indices is None:
            raise RuntimeError("add_cloth_mesh: vertices and indices are required")
        v = _np.asarray(vertices, dtype=_np.float64).reshape(-1, 3)
        if n_vertices and int(n_vertices) != len(v):
            raise RuntimeError("add_cloth_mesh: n_vertices=%d but got %d vertex triples"
                               % (int(n_vertices), len(v)))
        t = _np.asarray(indices, dtype=_np.int64)
        if t.size % 3:
            raise RuntimeError("add_cloth_mesh: %d indices is not a multiple of 3" % t.size)
        t = t.reshape(-1, 3)

        v, t, stats = self._clean_cloth_mesh(v, t)
        name = str(label) if label else "cloth_mesh_%d" % len(self.cloth_grids)
        if any(stats.values()):
            # Loud, with counts, and never fatal: a garment that needed cleaning
            # still simulates, but the author has to know their asset was edited
            # under them -- especially the orphan count, which would otherwise
            # present as invisible nails holding the fabric in the air.
            self._newton_log(
                "cloth: %s MESH CLEANED before authoring -- %s. These are silent "
                "defects in newton: an orphan vertex keeps mass 0 and becomes a "
                "KINEMATIC PIN, an unwelded duplicate splits the fabric with no "
                "stretch or bending across the seam, and a degenerate triangle "
                "desyncs tri_areas from tri_indices."
                % (name, ", ".join("%s=%d" % (k, n) for k, n in sorted(stats.items()) if n)))
        if len(v) == 0 or len(t) == 0:
            self._newton_log("cloth: %s registered NOTHING -- %d vertices, %d triangles "
                             "survived cleaning" % (name, len(v), len(t)))
            return (-1, -1)

        _tri_ke = float(tri_ke)
        _tri_ka = float(tri_ka) if float(tri_ka) >= 0.0 else _tri_ke
        _tri_kd = float(tri_kd) if float(tri_kd) >= 0.0 else 1.0e-2 * _tri_ke
        _edge_ke = float(edge_ke)
        _edge_kd = float(edge_kd) if float(edge_kd) >= 0.0 else 1.0e-2 * _edge_ke

        p_start = int(len(self.builder.particle_q))
        self.builder.add_cloth_mesh(
            pos=wp.vec3(float(pos_x), float(pos_y), float(pos_z)),
            rot=wp.quat(float(qx), float(qy), float(qz), float(qw)),
            scale=float(scale),
            vel=wp.vec3(float(vx), float(vy), float(vz)),
            # Upstream indexes `vertices` with numpy anyway (np.array(vertices)),
            # so handing it the array avoids materialising 3N Python floats.
            vertices=v,
            indices=t.reshape(-1).tolist(),
            density=float(density),
            tri_ke=_tri_ke, tri_ka=_tri_ka, tri_kd=_tri_kd,
            edge_ke=_edge_ke, edge_kd=_edge_kd,
            particle_radius=float(particle_radius),
            # newton's own quality checks (sliver triangles, extreme interior
            # angles, non-manifold edges). Free at load, and after the cleaning
            # above anything it still reports is a real property of the asset.
            validate_mesh=True,
            label=name,
        )
        p_end = int(len(self.builder.particle_q))

        # ---- PINNING. Zero mass is what newton reads as "kinematic": every
        # solver inverts it, so 0 becomes infinite inverse mass and the particle
        # stops integrating. add_cloth_grid does exactly this for a fixed edge
        # (builder.py:9053-9055) and also clears the ACTIVE flag, so both are
        # done here to match -- the flag is what keeps the particle out of the
        # active set rather than merely immovable within it.
        n_pinned = 0
        band = float(pin_top_band)
        if band > 0.0:
            zloc = v[:, 2]
            keep = zloc >= (float(zloc.max()) - band)
            idx = _np.nonzero(keep)[0]
            try:
                _inactive = ~int(newton.ParticleFlags.ACTIVE)
            except Exception:                            # noqa: BLE001
                _inactive = None
            for i in idx:
                p = p_start + int(i)
                self.builder.particle_mass[p] = 0.0
                if _inactive is not None:
                    self.builder.particle_flags[p] = int(self.builder.particle_flags[p]) & _inactive
            n_pinned = int(len(idx))
            if n_pinned == len(v):
                # Every particle pinned is a garment that cannot move at all,
                # and it reads in the world exactly like "cloth is broken".
                self._newton_log(
                    "cloth: %s pin_top_band %.4g m pinned ALL %d particles -- the band spans the whole mesh, "
                    "so nothing will ever move. Reduce it." % (name, band, n_pinned))

        rec = {
            "start": p_start, "end": p_end,
            "pinned": n_pinned,
            # ⚠ SNAPSHOTTED, not derivable. A grid sheet's winding is recomputed
            # from dim_x/dim_y by cloth_topology_packed(); a garment's exists only
            # here, and the builder is consumed at finalize(). Same reason and
            # same layout as soft_grids["surface"]: mesh-local int32 triples.
            "surface": t.reshape(-1).astype(_np.int32).tobytes(),
            "n_tris": int(len(t)),
            "mesh": True,
            "dim_x": 0, "dim_y": 0, "cell_x": 0.0, "cell_y": 0.0,
            "mass": float(density), "particle_radius": float(particle_radius),
            "tri_ke": _tri_ke, "tri_ka": _tri_ka, "tri_kd": _tri_kd,
            "edge_ke": _edge_ke, "edge_kd": _edge_kd,
            "label": name,
        }
        self.cloth_grids.append(rec)
        if self.cloth_particle_start < 0:
            self.cloth_particle_start = p_start
        self.cloth_particle_end = p_end
        # The mass is the number an author can sanity-check against a real
        # garment (a cotton T-shirt is 150-200 g), and it is the one thing
        # density alone does not tell them. ⚠ Read AFTER pinning, so it is the
        # FREE mass -- pinned particles have been zeroed and a heavily pinned
        # garment legitimately reports less than its fabric weighs.
        try:
            _m = float(_np.asarray(self.builder.particle_mass[p_start:p_end]).sum())
        except Exception:                                # noqa: BLE001
            _m = float("nan")
        self._newton_log(
            "cloth: %s particles [%d,%d) MESH %d verts %d tris density %.4g kg/m^2 "
            "free-mass %.4g kg radius %.4g scale %.4g tri_ke %.4g edge_ke %.4g pinned %d"
            % (name, p_start, p_end, len(v), len(t), float(density), _m,
               float(particle_radius), float(scale), _tri_ke, _edge_ke, n_pinned))
        return (p_start, p_end)

    def add_soft_grid(self, pos_x, pos_y, pos_z,
                      qx=0.0, qy=0.0, qz=0.0, qw=1.0,
                      dim_x=4, dim_y=4, dim_z=4,
                      cell_x=0.05, cell_y=0.05, cell_z=0.05,
                      density=1000.0, k_mu=1.0e4, k_lambda=1.0e4, k_damp=1.0,
                      particle_radius=0.01,
                      fix_left=False, fix_right=False,
                      fix_top=False, fix_bottom=False,
                      vx=0.0, vy=0.0, vz=0.0,
                      label=None):
        """Author one volumetric (tetrahedral FEM) soft block.

        Returns (particle_start, particle_end). C++-callable in exactly the same
        sense as add_cloth_grid above: scalars only, because the ctypes FFI
        boundary cannot pass wp.vec3 / wp.quat, so the warp types are composed
        here and forwarded to newton.ModelBuilder.add_soft_grid (keyword-only).

        Geometry follows newton's convention: `pos` is the block's minimum
        CORNER, the lattice is (dim_x+1)*(dim_y+1)*(dim_z+1) particles at
        cell_* spacing, tetrahedralised 5 tets per cell, and `rot` orients it.
        k_mu / k_lambda are the Lame parameters (Pa), k_damp viscous damping.

        ⚠ fix_left / fix_right pin local x == 0 / x == dim_x, and fix_top /
        fix_bottom pin local **y** == dim_y / y == 0. There is NO pin on the z
        faces, and "top" is the local +Y face, NOT world up. To pin the world-top
        face of a block in a Z-up world, rotate +90 deg about X (qx = qw =
        0.7071) so local +Y maps to world +Z, and pass fix_top. MEASURED that
        way: the pinned face held 1.200000 exactly while the free end sagged
        16.2 mm.

        ⚠ The surface triangles newton generates are for COLLISION and RENDER
        only. tri_ke / tri_ka / tri_kd and edge_ke / edge_kd are left at newton's
        0.0 default, so they add no elastic force -- the elasticity is entirely
        in the tets. That is also why finalize()'s builder.color(
        include_bending=True) is harmless here: the active-edge mask is
        (ke != 0) | (kd != 0), so a soft body's zero-stiffness edges are simply
        excluded from the colouring graph rather than costing anything.

        ⚠ MEASURED LIMIT, do not present soft bodies as symmetric: SOFT-ON-RIGID
        is stable (a 15.6 kg soft block presses a 2 kg dynamic box from 0.060000
        to 0.059385 and holds), but RIGID-ON-SOFT -- a rigid body supported ONLY
        by particles -- gains energy and is ejected. That reproduces in pure
        newton with OmniSim absent, in both coupled and uncoupled solvers, is not
        a timestep artifact (8x smaller dt made it worse) and is not
        self-contact. It scales with soft_contact_ke. Newton-side, mechanism
        unidentified.

        Like cloth, authoring one of these is what forces the coupled-solver path
        in finalize(): it makes has_cloth() true, and from that follow
        builder.color(), the SolverVBD entry, the CollisionPipeline and the CUDA
        device pin.
        """
        if self.builder is None:
            raise RuntimeError("add_soft_grid called with no builder (post-finalize?)")
        if int(dim_x) < 1 or int(dim_y) < 1 or int(dim_z) < 1:
            raise RuntimeError("add_soft_grid: dim_x/dim_y/dim_z must all be >= 1 "
                               "(got %d x %d x %d)" % (int(dim_x), int(dim_y), int(dim_z)))
        import numpy as _np
        p_start = int(len(self.builder.particle_q))
        t_start = int(len(self.builder.tri_indices))
        self.builder.add_soft_grid(
            pos=wp.vec3(float(pos_x), float(pos_y), float(pos_z)),
            rot=wp.quat(float(qx), float(qy), float(qz), float(qw)),
            vel=wp.vec3(float(vx), float(vy), float(vz)),
            dim_x=int(dim_x), dim_y=int(dim_y), dim_z=int(dim_z),
            cell_x=float(cell_x), cell_y=float(cell_y), cell_z=float(cell_z),
            density=float(density),
            k_mu=float(k_mu), k_lambda=float(k_lambda), k_damp=float(k_damp),
            fix_left=bool(fix_left), fix_right=bool(fix_right),
            fix_top=bool(fix_top), fix_bottom=bool(fix_bottom),
            particle_radius=float(particle_radius),
            label=(str(label) if label else None),
        )
        p_end = int(len(self.builder.particle_q))
        # The render surface is newton's own -- the open faces of the tet mesh.
        # Unlike a cloth sheet, whose winding cloth_topology_packed() re-derives
        # analytically from dim_x/dim_y, this CANNOT be recomputed later: the
        # builder is consumed at finalize(). Snapshot it now, rebased to
        # block-local indices so it pairs with particle_positions_packed(start,
        # end) exactly as the cloth topology does.
        _tris = _np.asarray(self.builder.tri_indices[t_start:], dtype=_np.int32)
        rec = {
            "start": p_start, "end": p_end,
            "surface": ((_tris - p_start).reshape(-1).astype(_np.int32).tobytes()
                        if len(_tris) else b""),
            "n_tris": int(len(_tris)),
            "dim_x": int(dim_x), "dim_y": int(dim_y), "dim_z": int(dim_z),
            "cell_x": float(cell_x), "cell_y": float(cell_y), "cell_z": float(cell_z),
            "density": float(density), "k_mu": float(k_mu),
            "k_lambda": float(k_lambda), "k_damp": float(k_damp),
            "particle_radius": float(particle_radius),
            "label": (str(label) if label else "soft_%d" % len(self.soft_grids)),
        }
        self.soft_grids.append(rec)
        # Share cloth's union particle range: both sources append contiguously
        # into ONE builder, and the VBD entry and the proxy mapping want a single
        # particle list, not one per source.
        if self.cloth_particle_start < 0:
            self.cloth_particle_start = p_start
        self.cloth_particle_end = p_end
        self._newton_log(
            "soft: %s particles [%d,%d) grid %dx%dx%d cell %.4g density %.4g "
            "k_mu %.4g k_lambda %.4g k_damp %.4g radius %.4g tris %d"
            % (rec["label"], p_start, p_end, int(dim_x), int(dim_y), int(dim_z),
               float(cell_x), float(density), float(k_mu), float(k_lambda),
               float(k_damp), float(particle_radius), rec["n_tris"]))
        return (p_start, p_end)

    def soft_surface_packed(self, grid_index=0):
        """Triangle index buffer for one soft block: tight int32 triples,
        block-local, 12 bytes per triangle.

        The soft-body twin of cloth_topology_packed(). Topology never changes --
        VBD moves vertices, never re-tetrahedralises -- so the renderer uploads
        this ONCE as GL_STATIC_DRAW and then streams only
        particle_positions_packed() per tick, which is the whole point of the
        WrDynamicMesh split. Returns b"" rather than raising when the index is
        out of range, matching every other packed reader here.
        """
        i = int(grid_index)
        if i < 0 or i >= len(self.soft_grids):
            return b""
        return self.soft_grids[i]["surface"]

    def soft_particle_range(self, grid_index=0):
        """(start, end) particle range of one soft block, for the readback."""
        i = int(grid_index)
        if i < 0 or i >= len(self.soft_grids):
            return (-1, -1)
        g = self.soft_grids[i]
        return (int(g["start"]), int(g["end"]))

    def _mjc_solver(self):
        """The SolverMuJoCo that owns the RIGID model.

        Without cloth this IS ``self.solver`` (``solver_mjc`` stays None), so
        every caller below is byte-identical to the pre-cloth code. WITH cloth
        ``self.solver`` is a ``SolverCoupledProxy`` and the MuJoCo solver is its
        ``"mjc"`` entry -- so anything reaching for ``mj_model`` / ``mj_data`` /
        the ``mjc_*_to_newton_*`` maps must come through here or it reads None
        and silently degrades (raycast returns [], welds return -1, the touch
        readback returns []).

        ⚠ The inner solver is built against a ``ModelView``, not the parent
        Model. OmniSim's "mjc" entry deliberately owns EVERY body, joint and
        shape, so that view is an identity compaction and the index maps stay
        parent-model-valid; finalize() asserts that body count and logs a
        WARNING if it ever stops being true. Do not add a second body-owning
        entry without re-checking those maps.
        """
        s = getattr(self, "solver_mjc", None)
        return s if s is not None else getattr(self, "solver", None)

    def add_body_force(self, body_idx, fx, fy, fz, tx, ty, tz):
        # W3.1 external-wrench injection (newton-ode-replacement-plan.md): accumulate a per-tick WORLD-frame
        # body wrench (force + torque about the body's reference point) for body_idx. step() writes the sum
        # into state.body_f each substep and clears the accumulator after the tick -- matching ODE's
        # addBodyForce semantics (a force the controller re-applies every tick). spatial_vector layout is
        # [Fx,Fy,Fz, Tx,Ty,Tz], world frame (verified in isolation: slots 0-2 translate, 3-5 rotate; a force
        # in a slot moves the body along that WORLD axis regardless of the body's orientation).
        bi = int(body_idx)
        if bi < 0:
            return
        if not hasattr(self, "_ext_wrench"):
            self._ext_wrench = {}
        w = self._ext_wrench.get(bi)
        if w is None:
            self._ext_wrench[bi] = [float(fx), float(fy), float(fz), float(tx), float(ty), float(tz)]
        else:
            w[0] += float(fx); w[1] += float(fy); w[2] += float(fz)
            w[3] += float(tx); w[4] += float(ty); w[5] += float(tz)

    def set_body_vel(self, body_idx, x, y, z, angular):
        # W3.2 mid-step velocity set (newton-ode-replacement-plan.md): directly write a Newton body's spatial
        # velocity. body_qd = [vx,vy,vz, wx,wy,wz] (verified: slots 0-2 linear, 3-5 angular, world frame).
        # angular=0 writes the linear half, =1 the angular half; the other half is preserved (read-mod-write,
        # like reset_body_pose). Persistent state, so unlike add_body_force it is NOT re-applied each tick.
        if self.state_a is None:
            # FIX 5 (t=0 setVelocity drop): a Supervisor setVelocity arriving
            # BEFORE finalize() (immediate messages run before the first
            # flush/finalize in OmSimulationWorld::step) used to be silently
            # lost -- the Newton body then registered pose-only, zero
            # velocity. Queue the write; finalize() drains the queue after
            # its closing eval_fk. Last write per (body, half) wins.
            if not hasattr(self, "_pending_body_vel"):
                self._pending_body_vel = {}
            self._pending_body_vel[(int(body_idx), 1 if int(angular) else 0)] = (
                float(x), float(y), float(z))
            return
        bi = int(body_idx)
        qd = self.state_a.body_qd.numpy()
        if not (0 <= bi < len(qd)):
            return
        o = 3 if int(angular) else 0
        qd[bi][o] = float(x); qd[bi][o + 1] = float(y); qd[bi][o + 2] = float(z)
        self.state_a.body_qd.assign(qd)
        # FREE BODY: the generalized velocity lives in joint_qd, and eval_fk OVERWRITES body_qd
        # from joint_qd every step -- so a body_qd-only write is silently lost (a Supervisor
        # node.setVelocity on a free body did nothing: it fell straight down). Mirror the
        # reset_body_pose free-body fix: also write the body's FREE joint qd. Newton free-joint
        # layout is [linear(3), angular(3)] in the WORLD frame (verified: spatial.py twist =
        # (v, omega); example_robot_policy reads root_lin_vel_w=joint_qd[:3], ang=joint_qd[3:6]),
        # so the SAME offset `o` selects the half to write.
        try:
            if self.model is not None:
                jqds = self.model.joint_qd_start.numpy()
                jc = self.model.joint_child.numpy()
                jqd = self.state_a.joint_qd.numpy()
                nqd = len(jqd)
                for j in range(len(jc)):
                    if int(jc[j]) != bi:
                        continue
                    s = int(jqds[j])
                    e = int(jqds[j + 1]) if j + 1 < len(jqds) else nqd
                    if e - s == 6 and s + 6 <= nqd:        # free joint = 6 dof
                        jqd[s + o] = float(x); jqd[s + o + 1] = float(y); jqd[s + o + 2] = float(z)
                        self.state_a.joint_qd.assign(jqd)
                    break
        except Exception:
            pass
        # Newton state changed outside the solver -> the MuJoCo-side data
        # must be refreshed from it on the next tick (see step()'s
        # update_data_interval gating).
        self._mjc_dirty = True

    def _raycast_maps(self):
        # Cached once after finalize: mjc geom id -> newton shape (world 0),
        # and newton shape -> newton body. Static for the world's lifetime.
        sv = self._mjc_solver()
        if not hasattr(self, "_g2s_np"):
            import numpy as _np
            self._g2s_np = sv.mjc_geom_to_newton_shape.numpy()[0]
            sb = self.model.shape_body
            self._sb_np = sb.numpy() if hasattr(sb, "numpy") else _np.asarray(sb)
        return self._g2s_np, self._sb_np

    # ---- GPU-path readback guard (internal parity plan, item W1.1) ----
    #
    # THE DEFECT. newton's SolverMuJoCo.step() writes `self.mj_data` ONLY on
    # its `use_mujoco_cpu` branch (solver_mujoco.py:3831-3838). The
    # mujoco_warp branch steps `mjw_data` on the GPU and never touches
    # mj_data again -- but mj_data still EXISTS on that path, because
    # put_data() seeds mjw_data from it at build time (:7092 compiles the
    # model + allocates MjData, :7461 copies it to the GPU). So every reader
    # of mj_data must be CPU-scoped or it answers, confidently and with no
    # error of any kind, against the scene AS AUTHORED AT t=0.
    #
    # A frozen answer is worse than no answer: it is indistinguishable from a
    # real one. weld_engage / weld_release (-2), touch_force ([]),
    # _capture_constraint_readbacks and _refresh_mj_cartesian already carry
    # this guard. raycast_batch and get_contacts did NOT, so under
    # newtonSolver "mujoco_warp" the whole ray-sensor family (DistanceSensor,
    # Receiver, LightSensor, Radar, Camera recognition occlusion) and the
    # whole contact family (getContactPoints, /sim/contacts, /sim/grips, the
    # damage tracker) were served from the build pose. Both decline here now.
    #
    # NOT fixed here, and deliberately: reading the GPU's own mjw_data.contact
    # arrays back would make contacts genuinely work under mujoco_warp. That
    # needs a per-world geom->shape->body map and a GPU verification pass; it
    # is the follow-up, not this guard.
    #
    # A/B hatch, value-parsed: OMNISIM_NEWTON_GPU_STALE_READBACK=1 restores
    # the pre-fix behaviour exactly (=0/unset keeps the guard).
    def _gpu_readback_declined(self, what):
        sv = self._mjc_solver()
        if sv is None or getattr(sv, "use_mujoco_cpu", False):
            return False                 # CPU mj_step: mj_data IS the stepped data
        if not hasattr(self, "_stale_readback_ok"):
            _v = _os.environ.get("OMNISIM_NEWTON_GPU_STALE_READBACK", "").strip().lower()
            self._stale_readback_ok = _v not in ("", "0", "false", "off", "no")
        if self._stale_readback_ok:
            return False
        seen = getattr(self, "_gpu_readback_warned", None)
        if seen is None:
            seen = self._gpu_readback_warned = set()
        if what not in seen:
            seen.add(what)
            print("[OmNewtonBackend] WARNING: the mj_data %s readback is NOT VALID on the GPU "
                  "newtonSolver \"mujoco_warp\" -- newton steps mjw_data on the device and "
                  "leaves mj_data FROZEN AT THE BUILD POSE, so it would answer against the "
                  "scene as authored at t=0. Declining it. %s Use the CPU newtonSolver "
                  "\"mujoco\" (the default) if you need the full-fidelity answer."
                  % (what,
                     "Ray sensors (DistanceSensor/Receiver/LightSensor/Radar/Camera "
                     "recognition occlusion) report NO new hits and keep their previous "
                     "verdicts; there is no second ray service on this solver."
                     if what == "raycast" else
                     "getContactPoints / /sim/contacts / /sim/grips / the damage tracker "
                     "fall back to newton's own narrow phase, which IS live here: body "
                     "PAIRS are correct, but contact POINTS are shape support points in "
                     "shape0's body frame (NOT world witnesses) and depth reads 0."),
                  flush=True)
        return True

    def raycast_batch(self, rays, exclude_bodies=(), max_skips=8):
        """The Newton-side ray service (replaces ODE ray geoms for sensors).

        rays: flat [px,py,pz, dx,dy,dz, max_len] * N, world frame; direction
        need not be unit. Returns flat [dist, newton_body, nx, ny, nz] * N;
        dist = -1.0 on miss (within max_len), newton_body = -1 for static
        world geometry with no Solid (e.g. the implicit ground plane).

        exclude_bodies: newton body indices the rays must pass through -- the
        caster's own robot, replicating odeNearCallback's same-robot rule.
        mj_ray's bodyexclude excludes exactly ONE mj body and a robot is many,
        so exclusion is done by advancing the origin just past excluded hits
        and re-casting (bounded by max_skips).

        Serves off the live CPU mj_model/mj_data -- the same physics the world
        is stepping -- so a hit is by construction consistent with contact
        behaviour. Requires the Cartesian-fresh contract (mj_step1 after the
        last substep) so answers are at t+dt. flg_static=1 and no geomgroup
        filter: rays see statics, matching ODE ray semantics.

        CPU mj_step ONLY -- see _gpu_readback_declined. Under mujoco_warp
        mj_data is frozen at the build pose, so this returns [] (C++ -> -1)
        and every ray consumer keeps its previous verdict.
        """
        sv = self._mjc_solver()
        if self._gpu_readback_declined("raycast"):
            return []          # -> C++ -1; consumers keep their previous verdicts
        import numpy as np
        import mujoco
        m = getattr(sv, "mj_model", None)
        d = getattr(sv, "mj_data", None)
        if m is None or d is None:
            return []          # unavailable -> C++ returns -1, caller keeps the ODE path
        g2s, sb = self._raycast_maps()
        excl = frozenset(int(b) for b in exclude_bodies)
        geomid = np.zeros(1, np.int32)
        normal = np.zeros(3, np.float64)
        # This bundle's mj_ray (mujoco 3.8.1) also returns the hit NORMAL --
        # which SONAR's reflection-cone test needs. Probe the signature once
        # so an older/newer wheel that lacks the argument still answers rays
        # (normals then read 0,0,0 and the C++ side treats them as absent).
        if not hasattr(self, "_mj_ray_takes_normal"):
            try:
                import inspect as _insp
                self._mj_ray_takes_normal = "normal" in str(_insp.signature(mujoco.mj_ray))
            except Exception:
                self._mj_ray_takes_normal = False

        def _cast(p, v):
            if self._mj_ray_takes_normal:
                return mujoco.mj_ray(m, d, p, v, None, 1, -1, geomid, normal)
            return mujoco.mj_ray(m, d, p, v, None, 1, -1, geomid)

        out = []
        for i in range(0, len(rays), 7):
            pnt = np.array(rays[i:i + 3], np.float64)
            vec = np.array(rays[i + 3:i + 6], np.float64)
            maxlen = float(rays[i + 6])
            n = np.linalg.norm(vec)
            if n < 1e-12:
                out.extend((-1.0, -1, 0.0, 0.0, 0.0))
                continue
            vec = vec / n
            base = 0.0
            hit = (-1.0, -1, 0.0, 0.0, 0.0)
            for _ in range(max_skips):
                normal[:] = 0.0
                t = _cast(pnt, vec)
                if t < 0.0 or base + t > maxlen:
                    break
                gi = int(geomid[0])
                shp = int(g2s[gi]) if 0 <= gi < len(g2s) else -1
                body = int(sb[shp]) if 0 <= shp < len(sb) else -1
                if body in excl:
                    pnt = pnt + (t + 1e-6) * vec
                    base += t + 1e-6
                    continue
                hit = (base + t, body,
                       float(normal[0]), float(normal[1]), float(normal[2]))
                break
            out.extend(hit)
        return out

    def get_contacts(self):
        # ⚠ ON THE CPU mj_step PATH, ANSWER FROM THE SOLVER'S OWN CONTACTS.
        # Everything below this block reads newton's narrow-phase, which under
        # SolverMuJoCo is a SHADOW: step() skips model.collide entirely (mj_step
        # runs its own collision), so refilling _contacts_cache here answers
        # from a collision engine that did not step the world. Three of the four
        # quantities it published were also wrong for any caller, measured on
        # projects/samples/demos/worlds/flagship/omniarm6_real_pick_place.wbt:
        #   * rigid_contact_point0 is in the LOCAL FRAME OF SHAPE 0's BODY and
        #     was published as a WORLD point. A block-resting-on-table contact
        #     came back as (-0.0250, -0.0251, 0.1000) -- exactly that point in
        #     PICK_TABLE's own frame (table centre z=0.10, half-height 0.10, so
        #     its top face is local z=+0.10), i.e. reported ~0.2 m below where
        #     the block actually is, at a world position nothing occupies.
        #   * it is a SUPPORT point, not a surface witness (the comment further
        #     down says so), so even transformed it names a shape extremum: the
        #     finger-pad entries were literally the pad box's 8 corners.
        #   * depth was hard-coded 0.0, so penetration was unmeasurable.
        # The net effect was that an agent trying to prove a two-finger grasp
        # got a plausible-looking contact list in which the block "touched"
        # link4 half a metre away and no depth was ever non-zero.
        # mjData.contact carries all of it honestly: pos is WORLD, dist is the
        # signed gap (negative = penetration), frame[0:3] is the world normal,
        # and mj_contactForce is the force the constraint solver actually
        # applied. OMNISIM_NEWTON_MJ_CONTACTS=0 reverts to the newton path.
        try:
            import os as _cco
            _cc_on = _cco.environ.get("OMNISIM_NEWTON_MJ_CONTACTS", "1").strip().lower()
        except Exception:
            _cc_on = "1"
        _csv = self._mjc_solver()
        _cm = getattr(_csv, "mj_model", None) if _csv is not None else None
        _cd = getattr(_csv, "mj_data", None) if _csv is not None else None
        # ⚠ CPU mj_step ONLY -- see _gpu_readback_declined. Under mujoco_warp
        # mj_data exists but is FROZEN AT THE BUILD POSE, and the block below
        # would not merely be stale: `_cd.ncon` on a never-stepped MjData is 0,
        # so it took the `_cn <= 0: return []` early exit and published "nothing
        # is touching" for the whole run, unconditionally, with no error. That
        # early return also made the newton narrow-phase fallback further down
        # unreachable on the GPU path. Declining the mj_data block here restores
        # it: `self.state_a` IS live under mujoco_warp (SolverMuJoCo writes
        # state_out every step on both branches), so the fallback answers from
        # the current pose. ⚠ Its caveats then apply and are why the warning is
        # loud: contact POINTS are shape support points in shape0's body frame,
        # and depth is hard-coded 0. Body PAIRS -- what /sim/grips and the damage
        # tracker key on -- are live and correct.
        if self._gpu_readback_declined("contacts"):
            _cm = _cd = None
        if _cc_on not in ("0", "false", "off", "no") and _cm is not None and _cd is not None:
            try:
                import numpy as _cnp
                import mujoco as _cmj
                _cg2s, _csb = self._raycast_maps()
                _cn = int(_cd.ncon)
                if _cn <= 0:
                    return []
                _ccon = _cd.contact
                _cgeo = getattr(_ccon, "geom", None)
                if _cgeo is None:
                    _cg1, _cg2 = _ccon.geom1, _ccon.geom2
                else:
                    _cg1, _cg2 = _cgeo[:, 0], _cgeo[:, 1]
                _cpos, _cdist, _cfrm = _ccon.pos, _ccon.dist, _ccon.frame
                _cf = _cnp.zeros(6, dtype=_cnp.float64)
                _cout = []
                for i in range(_cn):
                    _a, _b = int(_cg1[i]), int(_cg2[i])
                    _sa = int(_cg2s[_a]) if 0 <= _a < len(_cg2s) else -1
                    _sb2 = int(_cg2s[_b]) if 0 <= _b < len(_cg2s) else -1
                    _ba = int(_csb[_sa]) if 0 <= _sa < len(_csb) else -1
                    _bb = int(_csb[_sb2]) if 0 <= _sb2 < len(_csb) else -1
                    try:
                        _cmj.mj_contactForce(_cm, _cd, i, _cf)
                        _fm = float(_cnp.linalg.norm(_cf[:3]))
                    except Exception:
                        _fm = 0.0
                    _dg = float(_cdist[i])
                    _cout.extend((_ba, _bb,
                                  float(_cpos[i][0]), float(_cpos[i][1]), float(_cpos[i][2]),
                                  float(_cfrm[i][0]), float(_cfrm[i][1]), float(_cfrm[i][2]),
                                  -_dg if _dg < 0.0 else 0.0, _fm))
                return _cout
            except Exception:
                pass            # fall through to the newton narrow-phase below
        # PERF: in MuJoCo-contacts mode step() skips newton's model.collide
        # (the solver runs its own collision inside mjwarp and ignores the
        # contacts arg; newton's narrow-phase costs ~25 ms/substep on a
        # mesh-floor world). Refill lazily here so touch-sensor / damage
        # readbacks still work when actually used.
        if getattr(self, "_collide_stale", False) \
                and self.model is not None and self.state_a is not None:
            try:
                if getattr(self, "_contacts_cache", None) is None:
                    self._contacts_cache = self.model.contacts() if hasattr(self.model, 'contacts') else None
                if self._contacts_cache is not None:
                    self._collide_prof(self.state_a, self._contacts_cache)
                    self._collide_stale = False
            except Exception:
                pass
        # W4.1 native contact readback (newton-ode-replacement-plan.md): snapshot the rigid contacts Newton's
        # narrow-phase produced this step (model.collide fills _contacts_cache every substep) as a flat list
        # the C++ side unpacks, 9 values per contact: [bodyA, bodyB, px,py,pz, nx,ny,nz, |force|]. Shapes ->
        # bodies via model.shape_body (cached). bodyA/bodyB are Newton body indices (== the C++ mNewtonBodyIndex
        # space, the same the pose readback + body_f/body_qd writes use). |force| is 0 under XPBD's positional
        # solve (no explicit contact force at rest) -- the damage magnitude can come from closing velocity.
        c = getattr(self, "_contacts_cache", None)
        if c is None or self.model is None:
            return []
        try:
            n = int(c.rigid_contact_count.numpy()[0])
        except Exception:
            return []
        if n <= 0:
            return []
        n = min(n, int(c.rigid_contact_max))
        if not hasattr(self, "_shape_body_np"):
            sb = self.model.shape_body
            self._shape_body_np = sb.numpy() if hasattr(sb, "numpy") else __import__("numpy").asarray(sb)
        import numpy as _np
        sbn = self._shape_body_np
        s0 = c.rigid_contact_shape0.numpy(); s1 = c.rigid_contact_shape1.numpy()
        p0 = c.rigid_contact_point0.numpy()
        nrm = c.rigid_contact_normal.numpy()
        frc = c.rigid_contact_force.numpy()
        out = []  # 10 values per contact: bodyA, bodyB, px,py,pz, nx,ny,nz, depth, |force|
        for i in range(n):
            a_sh = int(s0[i]); b_sh = int(s1[i])
            if a_sh < 0 and b_sh < 0:
                continue  # unused buffer slot
            nx, ny, nz = float(nrm[i][0]), float(nrm[i][1]), float(nrm[i][2])
            if (nx * nx + ny * ny + nz * nz) < 1e-12:
                continue  # degenerate/empty slot (zero normal)
            a_bd = int(sbn[a_sh]) if 0 <= a_sh < len(sbn) else -1
            b_bd = int(sbn[b_sh]) if 0 <= b_sh < len(sbn) else -1
            px, py, pz = float(p0[i][0]), float(p0[i][1]), float(p0[i][2])
            # depth: XPBD resolves contacts positionally (penetration ~0 at rest -- verified: a box rests
            # exactly on the plane), and point0/point1 are SUPPORT points, not surface witnesses (their normal
            # projection is the shape extent, e.g. 0.1 for a 0.2-box, not the penetration). Newton exposes no
            # clean penetration field here (force + diff_distance are unpopulated under XPBD), so report 0 --
            # honest for resolved contacts. The damage tracker keys on closing velocity, not depth. Impact-
            # penetration is a follow-up (would need the pre-solve overlap or a solver that reports it).
            depth = 0.0
            out.extend((a_bd, b_bd, px, py, pz, nx, ny, nz, depth, float(_np.linalg.norm(frc[i]))))
        return out

    # ---- Weld slots (Connector / VacuumGripper) + TouchSensor mount force ----
    # _scratch/design_weld_touch.md. MuJoCo equality-weld slots are
    # pre-allocated INACTIVE at build time (mjModel.eq_* arrays are
    # compile-time sized, so a slot never allocated can never be locked) and
    # engaged/released at runtime by toggling mjData.eq_active + retargeting
    # mjModel.eq_obj1id/eq_obj2id -- both verified runtime-writable on the CPU
    # path (bundled mujoco 3.8.1). CPU SolverMuJoCo ONLY in phase 1 (the
    # engine default): newton's GPU notify path can push eq_active/eq_data but
    # NOT retarget eq_obj*id, and direct mjw_model surgery is graph-capture
    # hostile -- weld_engage/weld_release return -2 there and the C++ side
    # warns once instead of failing silently.
    #
    # INVARIANT these direct mj_model/mj_data writes rely on: the engine NEVER
    # calls solver.notify_model_changed (grep of this file: zero call sites),
    # so nothing re-stomps eq_active/eq_data from newton's own Model arrays.
    # If a future feature starts notifying CONSTRAINT_PROPERTIES, mirror the
    # toggles into model.equality_constraint_enabled as well.

    def add_weld_slot(self, body_idx):
        # BUILD phase only. Placeholder = an INACTIVE weld of the device's
        # body to the WORLD (a body1==body2 placeholder is refused at compile
        # -- "element repeated in equality constraint" -- while body-to-world
        # compiles fine; verified). Returns the slot id (>= 0) or -1.
        if self.model is not None:
            return -1              # too late: eq arrays are compile-sized
        try:
            eq = self.builder.add_equality_constraint_weld(
                body1=int(body_idx), body2=-1, enabled=False)
        except Exception:
            return -1
        if not hasattr(self, "_weld_slots"):
            self._weld_slots = []
        self._weld_slots.append(int(eq))     # newton eq index
        return len(self._weld_slots) - 1

    def _constraint_maps(self):
        # Lazy per-finalize caches: newton eq idx -> mjc eq idx and newton
        # body idx -> mjc body idx (world 0 rows of the solver's own maps --
        # never assume emission order). mjc ids are only valid for the current
        # finalize; every world (re)build rolls a FRESH World instance, so a
        # new build starts with neither attribute and staleness cannot leak.
        sv = self._mjc_solver()
        if not hasattr(self, "_eq_n2m"):
            e2n = sv.mjc_eq_to_newton_eq.numpy()[0]
            self._eq_n2m = {int(n): mi for mi, n in enumerate(e2n) if int(n) >= 0}
        if not hasattr(self, "_body_n2m"):
            b2n = sv.mjc_body_to_newton.numpy()[0]
            self._body_n2m = {int(n): mi for mi, n in enumerate(b2n) if int(n) >= 0}
        return self._eq_n2m, self._body_n2m

    def _weld_eq(self, slot):
        slots = getattr(self, "_weld_slots", None)
        if slots is None or not (0 <= int(slot) < len(slots)):
            return None
        eq_map, _ = self._constraint_maps()
        return eq_map.get(slots[int(slot)])

    def weld_engage(self, slot, body_a, body_b):
        # Activate `slot` welding body_a <-> body_b AT THEIR CURRENT RELATIVE
        # POSE (zero-snap). body_b < 0 = weld to the WORLD. Returns 0, -1
        # (error), or -2 (unsupported on this solver -- mujoco_warp).
        #
        # eq_data encoding, probed live on the bundled mujoco 3.8.1
        # (scratchpad weld_grid_probe, 2026-08-07 -- the design sketch's
        # anchor=0 + relpos=current write holds two NEARBY bodies but sags and
        # lever-arms when the weld point is far from body2's origin, e.g. any
        # body-to-world weld away from (0,0,0): a 1 kg body at (3,2,1) sagged
        # to z=0.065 under it, and holds to 4e-4 under the encoding below,
        # reading exactly (0, 0, 9.81) in its efc force rows):
        #   eq_data[0:3]  anchor      = body_a's origin in body_b's frame
        #   eq_data[3:6]  relpos      = 0
        #   eq_data[6:10] relquat     = q_a^-1 * q_b   (MUJOCO WXYZ ORDER)
        #   eq_data[10]   torquescale = 1
        sv = self._mjc_solver()
        if sv is None:
            return -1
        if not getattr(sv, "use_mujoco_cpu", False):
            return -2
        m = getattr(sv, "mj_model", None)
        d = getattr(sv, "mj_data", None)
        if m is None or d is None:
            return -1
        eq = self._weld_eq(slot)
        if eq is None:
            return -1
        a, b = int(body_a), int(body_b)
        if a < 0 and b < 0:
            return -1
        if a < 0:
            # Bodiless side first -> swap so obj1 is the real body (mirrors
            # ODE's dJointAttach body1==0 swap; the C++ caller tracks the
            # inversion for the rupture sign, exactly like mIsJointInversed).
            a, b = b, -1
        _, body_map = self._constraint_maps()
        ma = body_map.get(a)
        mb = body_map.get(b) if b >= 0 else 0
        if ma is None or mb is None:
            return -1
        import numpy as _np
        import mujoco as _mj
        _mj.mj_kinematics(m, d)   # defensive freshness; idempotent at fixed qpos
        pa, qa = d.xpos[ma].copy(), d.xquat[ma].copy()   # xquat is wxyz
        pb, qb = d.xpos[mb].copy(), d.xquat[mb].copy()
        qan, qbn = _np.zeros(4), _np.zeros(4)
        _mj.mju_negQuat(qan, qa)
        _mj.mju_negQuat(qbn, qb)
        anchor = _np.zeros(3)
        _mj.mju_rotVecQuat(anchor, pa - pb, qbn)
        relq = _np.zeros(4)
        _mj.mju_mulQuat(relq, qan, qb)
        m.eq_obj1id[eq] = int(ma)
        m.eq_obj2id[eq] = int(mb)
        m.eq_data[eq, :] = 0.0
        m.eq_data[eq, 0:3] = anchor
        m.eq_data[eq, 6:10] = relq
        m.eq_data[eq, 10] = 1.0
        d.eq_active[eq] = 1
        if not hasattr(self, "_weld_engaged"):
            self._weld_engaged = {}
        self._weld_engaged[int(slot)] = int(eq)
        return 0

    def weld_release(self, slot):
        # Deactivate `slot`. Retargeting back to the world placeholder is
        # optional hygiene and skipped: an inactive slot contributes nothing.
        sv = self._mjc_solver()
        if sv is None:
            return -1
        if not getattr(sv, "use_mujoco_cpu", False):
            return -2
        d = getattr(sv, "mj_data", None)
        eq = self._weld_eq(slot)
        if d is None or eq is None:
            return -1
        d.eq_active[eq] = 0
        if hasattr(self, "_weld_engaged"):
            self._weld_engaged.pop(int(slot), None)
        if hasattr(self, "_weld_force_snap"):
            self._weld_force_snap.pop(int(slot), None)
        return 0

    def weld_force(self, slot):
        # World-frame constraint wrench of an ACTIVE slot as of the LAST
        # completed tick: [fx,fy,fz, tx,ty,tz] -- the force the weld applies
        # ON obj1/body_a (probed: a hanging 1 kg welded to the world reads
        # +9.81 z, the constraint HOLDING obj1 up; matches ODE dJointFeedback
        # f1-on-node[0] once the caller's inversion flag is applied). Zeros
        # when inactive or not yet stepped (dJointFeedback-before-first-step
        # semantics). Served from the _capture_constraint_readbacks snapshot
        # taken BEFORE the end-of-tick mj_step1 Cartesian refresh: mj_step1
        # re-runs mj_makeConstraint, which re-instantiates the efc arrays
        # UNSOLVED, so a post-refresh live read would report a phantom zero
        # wrench every tick.
        snap = getattr(self, "_weld_force_snap", None)
        if snap is None:
            return [0.0] * 6
        return list(snap.get(int(slot), [0.0] * 6))

    def touch_force(self, body_idx):
        # ODE-f1-compatible mount wrench of a WELDED (fixed-joint, un-folded)
        # child body: [fx,fy,fz, tx,ty,tz], world-aligned axes, as of the LAST
        # completed tick. Source: mjData.cfrc_int after mj_rnePostConstraint,
        # NEGATED. cfrc_int is the interaction force ON the body FROM its
        # parent (the canonical 100kg-robot-on-100kg-sensor stack reads
        # [0,0,-981] force on the sensor body), while ODE's dJointFeedback f1
        # is the force on the PARENT: OmTouchSensor::createJoint attaches
        # dJointAttach(joint, parentBody, body), so node[0] is the parent --
        # exactly the negation. The sign is not cosmetic: the canonical
        # world's sensitive x-axis points DOWN (touch_sensor_force.wbt,
        # rotation 0 0 1 -1.5708 under NUE), so f1 = +981*up dotted with it
        # lands in computeValue's kept-negative branch -> 981 N, and the
        # un-negated cfrc_int would land in the rejected branch -> 0.
        # tests/test_newton_touch_force_parity.py pins this end to end.
        # Torque caveat: cfrc_int's c-frame sits at the body COM, not at the
        # ODE joint anchor, so rows 3-5 are reference-point-shifted relative
        # to ODE t1; only the force rows are consumed by TouchSensor.
        # The first read REGISTERS the body; rows arrive from the next tick's
        # snapshot (zeros until then -- same as ODE feedback before step 1).
        sv = self._mjc_solver()
        if sv is None:
            return []
        if not getattr(sv, "use_mujoco_cpu", False):
            return []
        if not hasattr(self, "_touch_bodies"):
            self._touch_bodies = set()
        self._touch_bodies.add(int(body_idx))
        snap = getattr(self, "_cfrc_snap", None)
        if snap is None:
            return [0.0] * 6
        _, body_map = self._constraint_maps()
        mb = body_map.get(int(body_idx))
        if mb is None or mb >= len(snap):
            return []
        t = snap[mb]                 # cfrc layout: [torque(3), force(3)]
        return [-float(t[3]), -float(t[4]), -float(t[5]),
                -float(t[0]), -float(t[1]), -float(t[2])]

    def _capture_constraint_readbacks(self):
        # Called at the END of step(), after the last substep and BEFORE
        # _refresh_mj_cartesian (whose mj_step1 rebuilds the efc arrays
        # unsolved -- see weld_force). Read-only numpy copies plus
        # mj_rnePostConstraint, which fills only the diagnostic cfrc_* arrays
        # mj_step never reads -- physics and the bitwise-determinism claim are
        # untouched, and nothing here runs at all unless a weld is engaged or
        # a touch body is registered.
        eng = getattr(self, "_weld_engaged", None)
        touch = getattr(self, "_touch_bodies", None)
        if not eng and not touch:
            return
        sv = self._mjc_solver()
        if sv is None or not getattr(sv, "use_mujoco_cpu", False):
            return
        m = getattr(sv, "mj_model", None)
        d = getattr(sv, "mj_data", None)
        if m is None or d is None:
            return
        import mujoco as _mj
        if eng:
            eq_rows = {}
            eq_watch = set(eng.values())
            eq_t = int(_mj.mjtConstraint.mjCNSTR_EQUALITY)
            for i in range(int(d.nefc)):
                if int(d.efc_type[i]) != eq_t:
                    continue
                eid = int(d.efc_id[i])
                if eid in eq_watch:
                    eq_rows.setdefault(eid, []).append(float(d.efc_force[i]))
            snap = {}
            for slot, eq in eng.items():
                r = eq_rows.get(eq, [])
                snap[slot] = (r + [0.0] * 6)[:6]
            self._weld_force_snap = snap
        if touch:
            _mj.mj_rnePostConstraint(m, d)
            self._cfrc_snap = d.cfrc_int.copy()

    def add_joint_revolute(self, parent_idx, child_idx,
                           ax, ay, az,
                           parent_anchor_x, parent_anchor_y, parent_anchor_z,
                           child_anchor_x, child_anchor_y, child_anchor_z,
                           target_ke=0.0, target_kd=0.0,
                           limit_lower=0.0, limit_upper=0.0,
                           effort_limit=0.0, velocity_limit=0.0,
                           child_rot_x=0.0, child_rot_y=0.0,
                           child_rot_z=0.0, child_rot_w=1.0):
        # Don't push to builder yet -- the caller (OmBasicJoint flush)
        # can feed joints in any order (e.g. leaf-first on nested PROTO
        # finalisation chains for Spot). Adding to the builder eagerly
        # forced us to also add FREE joints eagerly for "yet-unseen
        # parents", which then conflicts when that same body turns out
        # to be a revolute child later in the queue -- Newton rejects
        # with "Body N has multiple parents in this articulation".
        # Queue + topo-sort at finalize() removes that whole class of
        # bug regardless of how the callers order their adds.
        slot = len(self.pending_revolutes)
        self.pending_revolutes.append(dict(
            kind="revolute",
            parent=int(parent_idx),
            child=int(child_idx),
            axis=(float(ax), float(ay), float(az)),
            p_anchor=(float(parent_anchor_x), float(parent_anchor_y), float(parent_anchor_z)),
            c_anchor=(float(child_anchor_x), float(child_anchor_y), float(child_anchor_z)),
            # P-fix: child link's authored rotation relative to its joint parent
            # (R_child^T * R_parent). Identity for axis-aligned children (URDF
            # wheels, huskies) -> byte-unchanged; non-identity only for
            # hand-authored child Solids with an off-axis `rotation` (e.g.
            # BattleBot wheels/weapon `rotation 1 0 0 1.5708`). Baked into the
            # joint's child_xform at finalize so the revolute constraint doesn't
            # project the authored orientation away. See _add_revolute_to_builder.
            c_rot=(float(child_rot_x), float(child_rot_y),
                   float(child_rot_z), float(child_rot_w)),
            target_ke=float(target_ke),
            target_kd=float(target_kd),
            limit_lower=float(limit_lower),
            limit_upper=float(limit_upper),
            effort_limit=float(effort_limit),
            velocity_limit=float(velocity_limit),
        ))
        return slot

    def add_joint_prismatic(self, parent_idx, child_idx,
                            ax, ay, az,
                            parent_anchor_x, parent_anchor_y, parent_anchor_z,
                            child_anchor_x, child_anchor_y, child_anchor_z,
                            target_ke=0.0, target_kd=0.0,
                            limit_lower=0.0, limit_upper=0.0,
                            effort_limit=0.0, velocity_limit=0.0):
        # Linear/slider joint (e.g. parallel-gripper fingers). Queues into the
        # SAME pending list as revolutes so it joins the one articulation in
        # finalize()'s BFS (a finger's parent body is the gripper base, which
        # is fixed-merged onto the arm wrist -- it must be reachable from the
        # root or the finger body floats off as an orphan). finalize() ->
        # _add_revolute_to_builder dispatches on kind to add_joint_prismatic.
        slot = len(self.pending_revolutes)
        self.pending_revolutes.append(dict(
            kind="prismatic",
            parent=int(parent_idx),
            child=int(child_idx),
            axis=(float(ax), float(ay), float(az)),
            p_anchor=(float(parent_anchor_x), float(parent_anchor_y), float(parent_anchor_z)),
            c_anchor=(float(child_anchor_x), float(child_anchor_y), float(child_anchor_z)),
            target_ke=float(target_ke),
            target_kd=float(target_kd),
            limit_lower=float(limit_lower),
            limit_upper=float(limit_upper),
            effort_limit=float(effort_limit),
            velocity_limit=float(velocity_limit),
        ))
        return slot

    def add_joint_hinge2(self, parent_idx, child_idx,
                         ax1, ay1, az1, ax2, ay2, az2,
                         parent_anchor_x, parent_anchor_y, parent_anchor_z,
                         child_anchor_x, child_anchor_y, child_anchor_z):
        # Hinge2 / universal: 2-DoF rotation about two axes sharing one anchor (a caster's steer + roll, or
        # a car front wheel). Queues into the SAME pending list so it joins the one articulation in
        # finalize()'s BFS; _add_revolute_to_builder dispatches kind=="hinge2" to a native Newton d6 joint
        # with two free angular DoF. PASSIVE -- the driven variant is add_joint_hinge2_motorized below
        # (OMNISIM_NEWTON_BALL_HINGE2). newton-ode-replacement-plan.md W2.
        slot = len(self.pending_revolutes)
        self.pending_revolutes.append(dict(
            kind="hinge2",
            parent=int(parent_idx),
            child=int(child_idx),
            axis=(float(ax1), float(ay1), float(az1)),
            axis2=(float(ax2), float(ay2), float(az2)),
            p_anchor=(float(parent_anchor_x), float(parent_anchor_y), float(parent_anchor_z)),
            c_anchor=(float(child_anchor_x), float(child_anchor_y), float(child_anchor_z)),
        ))
        return slot

    def add_joint_ball(self, parent_idx, child_idx,
                       parent_anchor_x, parent_anchor_y, parent_anchor_z,
                       child_anchor_x, child_anchor_y, child_anchor_z):
        # Ball / spherical joint: 3-DoF rotation about a shared anchor, zero relative translation (common in
        # legged/soft rigs). Queues into the SAME pending list so it joins the one articulation in finalize()'s
        # BFS; _add_revolute_to_builder dispatches kind=="ball" to Newton's native add_joint_ball
        # (JointType.BALL, a quaternion-based spherical constraint -- gimbal-free, unlike a d6 Euler triple).
        # No axes needed (the constraint is just "free rotation about the anchor"). PASSIVE -- the driven
        # variant is add_joint_ball_motorized below (OMNISIM_NEWTON_BALL_HINGE2).
        # newton-ode-replacement-plan.md W2.2.
        slot = len(self.pending_revolutes)
        self.pending_revolutes.append(dict(
            kind="ball",
            parent=int(parent_idx),
            child=int(child_idx),
            p_anchor=(float(parent_anchor_x), float(parent_anchor_y), float(parent_anchor_z)),
            c_anchor=(float(child_anchor_x), float(child_anchor_y), float(child_anchor_z)),
        ))
        return slot

    def _env_float(self, key, default):
        # Env-var float read with a safe default. Same contract as the local
        # `_gain` closure in _add_revolute_to_builder, hoisted to a method so
        # the ball / hinge2 branches (which return BEFORE that closure exists)
        # can read the shared limit-spring knobs too.
        import os as _efos
        v = _efos.environ.get(key)
        try:
            return float(v) if v not in (None, "") else default
        except ValueError:
            return default

    def ball_hinge2_enabled(self):
        # OMNISIM_NEWTON_BALL_HINGE2 (value-parsed, DEFAULT OFF): the MOTORISED
        # + angle-readback layer for BallJoint / Hinge2Joint. OFF leaves the
        # passive W2 / W2.2 registration (add_joint_hinge2 / add_joint_ball)
        # byte-identical -- the engine simply never calls the *_motorized verbs
        # and every readback stays on the ODE path. Mirrors the C++-side
        # OmBasicJoint::newtonBallHinge2Enabled(), which is what actually gates
        # the call sites; this copy exists for the paths that live entirely
        # inside the runtime (the post-step joint clamp).
        g = getattr(self, "_bh2_gate", None)
        if g is None:
            import os as _bhos
            v = (_bhos.environ.get("OMNISIM_NEWTON_BALL_HINGE2") or "").strip().lower()
            # DEFAULT ON since 2026-08-08 (mirrors the C++ gate): with ODE deleted
            # there is no passive fallback worth preserving, so an unset var means
            # the motorised path. bool(v) was the default-OFF term.
            g = bool(v) and v not in ("0", "false", "off", "no")
            self._bh2_gate = g
        return g

    def add_joint_hinge2_motorized(self, parent_idx, child_idx,
                                   p_anchor, c_anchor, c_rot,
                                   axis1, axis2, gains, limits, efforts, vel_limits):
        # Hinge2 / universal, MOTORISED (OMNISIM_NEWTON_BALL_HINGE2). Same
        # native d6 as the passive add_joint_hinge2 above, but each angular DoF
        # carries its OWN JointDofConfig so axis1 and axis2 take their motors,
        # limits, effort and velocity ceilings INDEPENDENTLY (Webots Hinge2Joint
        # has motor+motor2 and jointParameters+jointParameters2).
        #
        # WHY NO INTERMEDIATE MASSLESS BODY. newton's MuJoCo converter emits a
        # d6's angular axes as N SEPARATE mjJNT_HINGE elements on the SAME child
        # body (solver_mujoco.py, the `elif j_type in supported_joint_types`
        # branch: axname = "<joint>_ang0" / "_ang1", each with its own range,
        # armature, actfrcrange and its own position+velocity actuators), and
        # MuJoCo composes a body's joints in declaration order -- so _ang1's
        # axis is carried by _ang0's rotation. That IS the universal joint, and
        # it is exactly ODE's Hinge2 semantics (axis2 attached to body2). A
        # zero-mass intermediate link would instead inject zero-mass DOFs into
        # the dynamic articulation, which is the singular-mass-matrix failure
        # this file already documents for mass=0 free-jointed bodies.
        #
        # gains = (ke1, kd1, ke2, kd2); limits = (lo1, hi1, lo2, hi2);
        # efforts = (e1, e2); vel_limits = (v1, v2). c_rot = the child link's
        # authored rotation relative to its joint parent, baked into child_xform
        # exactly as the revolute path does, so joint angle 0 == authored pose.
        slot = len(self.pending_revolutes)
        self.pending_revolutes.append(dict(
            kind="hinge2",
            motorized=True,
            parent=int(parent_idx),
            child=int(child_idx),
            axis=tuple(float(v) for v in axis1),
            axis2=tuple(float(v) for v in axis2),
            p_anchor=tuple(float(v) for v in p_anchor),
            c_anchor=tuple(float(v) for v in c_anchor),
            c_rot=tuple(float(v) for v in c_rot),
            gains=tuple(float(v) for v in gains),
            limits=tuple(float(v) for v in limits),
            efforts=tuple(float(v) for v in efforts),
            vel_limits=tuple(float(v) for v in vel_limits),
        ))
        return slot

    def add_joint_ball_motorized(self, parent_idx, child_idx,
                                 p_anchor, p_quat, c_anchor, c_quat,
                                 gains, limits, efforts, vel_limits):
        # Ball / spherical, MOTORISED (OMNISIM_NEWTON_BALL_HINGE2). A Webots
        # BallJoint can carry up to THREE RotationalMotors (device / device2 /
        # device3) about axis / axis2 / axis3.
        #
        # ⚠ THE MuJoCo CONVERTER IGNORES A BALL JOINT'S PER-DOF AXES. It emits
        # ONE mjJNT_BALL element plus three actuators whose gear vectors are
        # e_x / e_y / e_z (solver_mujoco.py, the `elif j_type == JointType.BALL`
        # branch: args["gear"][i] = 1.0), i.e. the three motors act about the
        # JOINT FRAME's own axes, not about whatever vectors we put in the
        # JointDofConfigs. So the Webots axis triad is carried by the joint
        # frame's ROTATION instead: the engine passes p_quat = the quaternion of
        # the orthonormal basis [axis | axis2 | axis3] and c_quat = childRelRot *
        # p_quat, so joint-frame x/y/z ARE axis/axis2/axis3 and the readback's
        # Rx*Ry*Rz decomposition of joint_q lands on Webots' own
        # position/position2/position3 convention.
        #
        # ⚠ AND THE BALL ELEMENT IS EMITTED `limited: False`, so per-axis
        # min/maxStop / min/maxPosition are NOT enforced by the solver. The
        # engine warns about that at registration; the limits still ride along
        # in the builder arrays (harmless, and correct if upstream ever grows
        # ball limits).
        slot = len(self.pending_revolutes)
        self.pending_revolutes.append(dict(
            kind="ball",
            motorized=True,
            parent=int(parent_idx),
            child=int(child_idx),
            p_anchor=tuple(float(v) for v in p_anchor),
            p_quat=tuple(float(v) for v in p_quat),
            c_anchor=tuple(float(v) for v in c_anchor),
            c_quat=tuple(float(v) for v in c_quat),
            gains=tuple(float(v) for v in gains),
            limits=tuple(float(v) for v in limits),
            efforts=tuple(float(v) for v in efforts),
            vel_limits=tuple(float(v) for v in vel_limits),
        ))
        return slot

    def add_joint_fixed(self, parent_idx, child_idx):
        # FIXED (0-DOF) tree joint: welds child to parent RIGIDLY at their
        # CURRENT builder poses. First consumer is the force-TouchSensor
        # un-fold (_scratch/design_weld_touch.md T1): a force-type sensor must
        # be its own body so its mount wrench is readable via cfrc_int, and
        # newton's MuJoCo conversion keeps a FIXED-joint child as a separate
        # WELDED mjc body with its own geoms and no joint element
        # (solver_mujoco.py 4844-4851, 5205-5207) -- exactly what per-body
        # contact attribution and cfrc_int need. Queues into the SAME pending
        # list so finalize()'s BFS emits it parent-before-child, the child is
        # never given an orphan FREE joint, and the self-collision filter sees
        # the parent-child edge; _add_revolute_to_builder dispatches
        # kind=="fixed". The joint frame is the CHILD's frame: parent_xform =
        # the child's pose in the parent's frame (computed from the spawn
        # poses the builder already holds -- build-phase poses don't move),
        # child_xform = identity.
        pi, ci = int(parent_idx), int(child_idx)
        pq = self.builder.body_q[pi]
        cq = self.builder.body_q[ci]

        # xyzw quaternion helpers, plain python (warp's transform math is
        # kernel-scope; builder.body_q entries expose .p / .q).
        def _conj(q):
            return (-q[0], -q[1], -q[2], q[3])

        def _mul(a, b):
            ax, ay, az, aw = a
            bx, by, bz, bw = b
            return (aw * bx + ax * bw + ay * bz - az * by,
                    aw * by - ax * bz + ay * bw + az * bx,
                    aw * bz + ax * by - ay * bx + az * bw,
                    aw * bw - ax * bx - ay * by - az * bz)

        def _rot(q, v):
            qv = _mul(_mul(q, (v[0], v[1], v[2], 0.0)), _conj(q))
            return (qv[0], qv[1], qv[2])

        pinv = _conj(tuple(pq.q))
        dp = (cq.p[0] - pq.p[0], cq.p[1] - pq.p[1], cq.p[2] - pq.p[2])
        rel_p = _rot(pinv, dp)
        rel_q = _mul(pinv, tuple(cq.q))
        slot = len(self.pending_revolutes)
        self.pending_revolutes.append(dict(
            kind="fixed",
            parent=pi,
            child=ci,
            p_anchor=tuple(float(v) for v in rel_p),
            c_anchor=(0.0, 0.0, 0.0),
            p_quat=tuple(float(v) for v in rel_q),
        ))
        return slot

    def diag_dump_joint_q(self):
        """One-shot dump of joint_q to identify whether the MuJoCo
        solver is actually writing to it."""
        out = []
        try:
            q = self.state_a.joint_q.numpy().tolist()
            out.append(f"state.joint_q len={len(q)} vals={[round(v,3) for v in q]}")
        except Exception as _e:
            out.append(f"state.joint_q read failed: {_e}")
        try:
            if self.control is not None:
                attrs = [a for a in dir(self.control) if not a.startswith('_')]
                out.append(f"control attrs: {attrs}")
            else:
                out.append("control is None")
        except Exception as _e:
            out.append(f"control inspect failed: {_e}")
        try:
            out.append(f"slot_to_real_idx={dict(self.slot_to_real_idx)}")
            out.append(f"q_start={self.model.joint_q_start.numpy().tolist()}")
            out.append(f"qd_start={self.model.joint_qd_start.numpy().tolist()}")
        except Exception as _e:
            out.append(f"start arrays read failed: {_e}")
        return "\n".join(out)

    def get_joint_angle(self, slot_id):
        """Read the live joint angle (radians) for a revolute slot, or 0.0
        if the slot/model isn't set up yet. Needed by the position-bridge
        in OmBasicJoint -- Webots' hinge->position() reads from ODE only,
        so the Newton-backed joints have to be queried directly."""
        if self.model is None:
            return 0.0
        real_idx = self.slot_to_real_idx.get(int(slot_id))
        if real_idx is None:
            return 0.0
        # (XPBD REMOVED 2026-08-07: it integrated maximal coords and never
        # maintained joint_q, so an eval_ik refresh ran here once per step.
        # SolverMuJoCo maintains joint_q itself; no refresh is needed.)
        # joint_q is indexed via joint_q_start. For revolute joints this
        # is a single scalar (the angle in radians).
        if not hasattr(self, "_q_start_cache"):
            self._q_start_cache = self.model.joint_q_start.numpy()
        q_start = self._q_start_cache
        if real_idx >= len(q_start):
            return 0.0
        q_idx = int(q_start[real_idx])
        # state_a.joint_q is populated by the solver each step. Fallback
        # to the model's joint_q if the state doesn't expose it yet.
        # PERF: cache joint_q per step (same pattern as body_xform's
        # body_q cache). A 13-sensor humanoid otherwise pays 13 full
        # GPU->CPU syncs per tick just reading its position sensors.
        # Invalidated at the end of each step().
        q_arr = getattr(self, "_joint_q_cache", None)
        if q_arr is None:
            try:
                q_arr = self.state_a.joint_q.numpy()
            except AttributeError:
                q_arr = self.model.joint_q.numpy()
            self._joint_q_cache = q_arr
        if q_idx >= len(q_arr):
            return 0.0
        return float(q_arr[q_idx])

    def _joint_q_slice(self, slot_id, n):
        """The n joint_q coordinates of `slot_id`, or None.

        Shared read path for the multi-DoF readbacks (hinge2's two angles, a
        ball joint's quaternion). Uses the SAME per-step _joint_q_cache that
        get_joint_angle uses, so a scene reading many multi-DoF sensors still
        pays one GPU->CPU sync per tick, not one per sensor.
        """
        if self.model is None:
            return None
        real_idx = self.slot_to_real_idx.get(int(slot_id))
        if real_idx is None:
            return None
        if not hasattr(self, "_q_start_cache"):
            self._q_start_cache = self.model.joint_q_start.numpy()
        q_start = self._q_start_cache
        if real_idx >= len(q_start):
            return None
        q_idx = int(q_start[real_idx])
        q_arr = getattr(self, "_joint_q_cache", None)
        if q_arr is None:
            try:
                q_arr = self.state_a.joint_q.numpy()
            except AttributeError:
                q_arr = self.model.joint_q.numpy()
            self._joint_q_cache = q_arr
        if q_idx + n > len(q_arr):
            return None
        return [float(v) for v in q_arr[q_idx:q_idx + n]]

    def get_joint_angle_dof(self, slot_id, dof):
        """Angle (rad) of one DoF of a multi-DoF joint, or 0.0.

        For a d6 (Hinge2) the joint's coordinates are one scalar per DoF in
        declaration order, so dof 0 == axis1 and dof 1 == axis2. dof 0 is
        identical to get_joint_angle(slot_id); the separate entry point exists
        because get_joint_angle documents (and is relied on for) the 1-DoF
        revolute contract.
        """
        vals = self._joint_q_slice(slot_id, int(dof) + 1)
        if vals is None:
            return 0.0
        return vals[int(dof)]

    def get_joint_ball_quat(self, slot_id):
        """A BALL joint's relative rotation as an (x, y, z, w) quaternion.

        newton stores a ball joint's position as a 4D quaternion (xyzw) in
        joint_q -- NOT as three angles -- so the caller (OmBallJoint) decomposes
        it into Webots' position/position2/position3 triple. Identity when the
        model isn't up yet, which reads as "at the authored pose".
        """
        vals = self._joint_q_slice(slot_id, 4)
        if vals is None:
            return (0.0, 0.0, 0.0, 1.0)
        return (vals[0], vals[1], vals[2], vals[3])


    def _ik_slot_map(self):
        """slot_id -> (q_offset, q_width, dof_offset, dof_width, limit_lo, limit_hi).

        THE WHOLE POINT OF THE IK API. `slot_to_real_idx` is the only map from an
        OmniSim joint id to a newton BUILDER joint index, and joint_q_start /
        joint_qd_start are the only maps from that index into the flat joint_q /
        joint_qd vectors -- exactly the two arrays get_joint_angle() and
        _joint_q_slice() already walk. Everything else in solve_ik is expressed
        through this so a caller never sees a raw newton coordinate offset.

        Widths come from the NEXT joint's start, so a multi-coord joint (a ball's
        4, a free root's 7) sizes itself instead of being assumed scalar.
        """
        m = getattr(self, "_ik_slot_map_cache", None)
        if m is not None:
            return m
        if self.model is None:
            return {}
        qs = self.model.joint_q_start.numpy()
        ds = self.model.joint_qd_start.numpy()
        nq, nd = int(self.model.joint_coord_count), int(self.model.joint_dof_count)
        m = {}
        for slot, real in self.slot_to_real_idx.items():
            real = int(real)
            if real >= len(qs) or real >= len(ds):
                continue
            q0 = int(qs[real]);  q1 = int(qs[real + 1]) if real + 1 < len(qs) else nq
            d0 = int(ds[real]);  d1 = int(ds[real + 1]) if real + 1 < len(ds) else nd
            # Limits come from the AUTHORED spec, like the step() clamp does --
            # never from model.joint_limit_lower indexed by a coordinate. That
            # array is DOF-shaped ([joint_dof_count]), and on any world with a
            # free/ball joint dof != coord, so a coord index into it is silently
            # off (measured here: coord_count 15 vs dof_count 14 -> IndexError at
            # the last slot, and a WRONG limit on every slot after the free root
            # if the array had happened to be long enough).
            spec = (self.pending_revolutes[int(slot)]
                    if int(slot) < len(self.pending_revolutes) else {})
            lo = float(spec.get("limit_lower", 0.0))
            hi = float(spec.get("limit_upper", 0.0))
            if hi <= lo:                       # newton's "no limit" convention
                lo, hi = float("-inf"), float("inf")
            m[int(slot)] = (q0, q1 - q0, d0, d1 - d0, lo, hi)
        self._ik_slot_map_cache = m
        return m

    def ik_slots(self):
        """Every slot solve_ik can drive, sorted. Flat ints, FFI-safe."""
        if self.model is None:
            return []
        return sorted(self._ik_slot_map().keys())

    def _ik_solver(self, link_index, n_problems, slots, want_rot, want_limits,
                   tool_offset):
        """Cached IKSolver + its target buffers.

        ⚠ Construction compiles a warp kernel specialised on
        (joint_dof_count, n_residuals, arch) -- measured 8.3 s cold on a 6R arm,
        153 ms per solve after. The key below therefore includes EVERYTHING that
        changes the residual mix or the buffers, so an agent alternating call
        shapes reuses instead of recompiling. The slot set is in the key because
        it selects the joint_dof_mask; the COMPILE key is unaffected by it, so
        different slot sets on one model share the compiled tile and only pay a
        Python-side rebuild.
        """
        key = (int(link_index), int(n_problems), tuple(slots), bool(want_rot),
               bool(want_limits), tuple(round(float(v), 9) for v in tool_offset))
        cache = getattr(self, "_ik_cache", None)
        if cache is None:
            cache = self._ik_cache = {}
        hit = cache.get(key)
        if hit is not None:
            return hit
        import numpy as np
        n = int(n_problems)
        dev = self.model.device
        smap = self._ik_slot_map()
        # ---- joint_dof_mask: solve ONLY the requested slots ---------------
        # Without it the optimiser is free to move every coordinate in the
        # model, including a floating robot's 6-DOF root and other robots'
        # joints -- it will happily "reach" a target by translating a body the
        # caller never asked about and cannot write back, and the reported
        # residual then describes a pose that will never exist.
        mask_np = np.zeros(int(self.model.joint_dof_count), dtype=bool)
        for s in slots:
            d0, dw = smap[int(s)][2], smap[int(s)][3]
            mask_np[d0:d0 + dw] = True
        mask = wp.array(mask_np, dtype=wp.bool, device=dev)
        off = wp.vec3(float(tool_offset[0]), float(tool_offset[1]),
                      float(tool_offset[2])) if len(tool_offset) == 3 else wp.vec3(0.0, 0.0, 0.0)
        pos = wp.zeros(n, dtype=wp.vec3, device=dev)
        objs = [newton.ik.IKObjectivePosition(
            link_index=int(link_index), link_offset=off,
            target_positions=pos, weight=1.0)]
        rot = None
        if want_rot:
            rot = wp.zeros(n, dtype=wp.quat, device=dev)
            objs.append(newton.ik.IKObjectiveRotation(
                link_index=int(link_index), link_offset_rotation=wp.quat_identity(),
                target_rotations=rot, weight=0.5))
        if want_limits:
            # NOTE: a SOFT weighted residual, not a constraint -- solve_ik clamps.
            objs.append(newton.ik.IKObjectiveJointLimit(
                joint_limit_lower=self.model.joint_limit_lower,
                joint_limit_upper=self.model.joint_limit_upper, weight=0.1))
        solver = newton.ik.IKSolver(model=self.model, n_problems=n,
                                    objectives=objs, joint_dof_mask=mask)
        nq = int(self.model.joint_coord_count)
        buf = (solver, pos, rot,
               wp.zeros((n, nq), dtype=wp.float32, device=dev),
               wp.zeros((n, nq), dtype=wp.float32, device=dev), nq, mask)
        cache[key] = buf
        return buf

    def solve_ik(self, link_index, targets, slots=(), rotations=(), seeds=(),
                 iterations=64, clamp_to_limits=1, tool_offset=()):
        """Batched IK against the LIVE model the physics solver steps.

        link_index  : newton body index of the end effector (what body_xform()
                      takes, i.e. OmSolid's mNewtonBodyIndex).
        targets     : flat [x,y,z] * n_problems, world frame.
        slots       : OmniSim joint SLOT ids to solve for, in the order the
                      answer comes back. Empty => every slot in slot_to_real_idx
                      (sorted) -- which is every revolute/prismatic OmniSim
                      registered, and never a free/fixed root, because those
                      carry no slot.
        rotations   : optional flat [qx,qy,qz,qw] * n_problems. Empty => position only.
        seeds       : optional flat [angle per slot] * n_problems (warm start).
                      Empty => seed every problem from the LIVE joint angles.
        tool_offset : optional [x,y,z] TCP offset in the end effector's own
                      frame (a gripper's grasp point, not its link origin).
        returns     : flat [angle per slot] * n_problems, THEN n_problems
                      residual norms in METRES -- measured by forward kinematics
                      on exactly the vector the caller will write back (live q
                      with only these slots replaced, after clamping), so a
                      caller can reject a problem instead of driving to it.

        Reads and writes nothing in state_a: the solver owns its buffers. Safe
        to call mid-step; it does NOT move the robot.
        """
        import numpy as np
        if self.model is None:
            return []
        t = np.asarray(targets, dtype=np.float32).reshape(-1, 3)
        n = int(t.shape[0])
        if n == 0:
            return []
        smap = self._ik_slot_map()
        slots = [int(s) for s in slots] if len(slots) else sorted(smap.keys())
        bad = [s for s in slots if s not in smap]
        if bad:
            raise ValueError("solve_ik: unknown joint slot(s) %s" % bad)
        # Multi-coord slots (a motorised ball joint) are quaternion-integrated;
        # newton requires those masked all-or-nothing and the scalar write-back
        # below would be wrong for them. Refuse rather than return nonsense.
        wide = [s for s in slots if smap[s][1] != 1]
        if wide:
            raise ValueError("solve_ik: slot(s) %s are multi-coordinate; only "
                             "1-coord (revolute/prismatic) slots are supported" % wide)
        want_rot = len(rotations) > 0
        tool = tuple(tool_offset) if len(tool_offset) == 3 else (0.0, 0.0, 0.0)
        solver, pos, rot, q_in, q_out, nq, _mask = self._ik_solver(
            link_index, n, slots, want_rot, bool(int(clamp_to_limits)), tool)
        qi = np.array([smap[s][0] for s in slots], dtype=np.int64)
        pos.assign(t)
        if want_rot:
            rot.assign(np.asarray(rotations, dtype=np.float32).reshape(-1, 4))
        # Live joint_q is the seed for every coordinate; the masked ones can
        # only ever hold it, which is what makes the write-back consistent.
        live = getattr(self, "_joint_q_cache", None)
        if live is None:
            try:
                live = self.state_a.joint_q.numpy()
            except AttributeError:
                live = self.model.joint_q.numpy()
        live = np.asarray(live, np.float32)[:nq]
        q_seed = np.tile(live, (n, 1))
        if len(seeds):
            q_seed[:, qi] = np.asarray(seeds, dtype=np.float32).reshape(n, len(slots))
        q_in.assign(q_seed)
        solver.step(q_in, q_out, iterations=int(iterations))
        ang = q_out.numpy()[:, qi].astype(np.float32)
        if int(clamp_to_limits):
            # ⚠ IKObjectiveJointLimit is a WEIGHTED RESIDUAL, not a constraint
            # (measured: 4.86 rad past a limit with the objective active), so a
            # value handed to setPosition unclamped is an illegal command.
            lo = np.array([smap[s][4] for s in slots], np.float32)
            hi = np.array([smap[s][5] for s in slots], np.float32)
            ang = np.clip(ang, lo, hi)
        # ---- residual: FK on what the caller will actually apply -----------
        q_apply = np.tile(live, (n, 1))
        q_apply[:, qi] = ang
        st = self.model.state()
        qd = wp.zeros(int(self.model.joint_dof_count), dtype=wp.float32,
                      device=self.model.device)
        res = []
        for i in range(n):
            newton.eval_fk(self.model, wp.array(q_apply[i], dtype=wp.float32,
                                                device=self.model.device), qd, st)
            xf = st.body_q.numpy()[int(link_index)]
            p = np.asarray(xf[:3], np.float64)
            if tool != (0.0, 0.0, 0.0):
                x, y, z, w = (float(v) for v in xf[3:7])
                v = np.asarray(tool, np.float64)
                tv = 2.0 * np.cross([x, y, z], v)
                p = p + tv * w + np.cross([x, y, z], tv) + v
            res.append(float(np.linalg.norm(p - t[i])))
        return [float(v) for v in ang.reshape(-1)] + res

    @staticmethod
    def _target_key(slot_id, dof):
        # Multi-DoF addressing (OMNISIM_NEWTON_BALL_HINGE2): the per-tick target
        # dicts are keyed by SLOT for the 1-DoF joints that have always used
        # them, and by (SLOT, DOF) for a specific DoF of a hinge2 / ball joint.
        # dof 0 keeps the bare-int key so every pre-existing call site produces
        # byte-identical dict contents.
        return int(slot_id) if int(dof) == 0 else (int(slot_id), int(dof))

    @staticmethod
    def _split_target_key(key):
        return key if isinstance(key, tuple) else (key, 0)

    def set_joint_target_vel(self, slot_id, vel, dof=0):
        # Stash target by slot id. step() translates slot -> real builder
        # joint index -> DOF index. Pre-finalize calls just queue.
        self.joint_targets[self._target_key(slot_id, dof)] = float(vel)
        if self.model is not None:
            return
        # Pre-finalize: also try to seed builder.joint_target_vel if the
        # real joint already exists (i.e. finalize was called and then
        # this slot got mapped). On the first build pass this is a no-op.
        real = self.slot_to_real_idx.get(int(slot_id))
        if real is None:
            return
        try:
            qd_start = self.builder.joint_qd_start[real]
            # newton 1.5 renamed the builder array too (joint_target_vel ->
            # joint_target_qd); resolve by attribute so one source drives both.
            _bt = getattr(self.builder, "joint_target_qd", None)
            if _bt is None:
                _bt = self.builder.joint_target_vel
            _bt[qd_start + int(dof)] = float(vel)
        except Exception:
            pass

    def reset_body_pose(self, body_idx, x, y, z, qx, qy, qz, qw):
        """Warp a Newton body to the given world pose and zero its
        velocities. Called from OmSolid when a Supervisor write hits
        the Solid's translation/rotation field. Without this, the
        ODE-side reset doesn't propagate to Newton and the body_q
        drifts away from where Webots is rendering after a few
        hundred episodes -- the constraint solver eventually fails
        with NaN once the divergence is large enough."""
        if self.model is None or self.state_a is None:
            return
        try:
            bidx = int(body_idx)
            # In-place .assign() instead of reassigning a fresh wp.array.
            # Reassigning self.state_a.body_q = wp.array(...) leaks: the
            # solver holds a reference to the original array and the new
            # allocations pile up on the GPU across thousands of episode
            # resets, eventually killing the embedded interpreter
            # (the ~200k-step ConnectionResetError crash ceiling).
            bq = self.state_a.body_q.numpy()
            if 0 <= bidx < len(bq):
                bq[bidx] = (float(x), float(y), float(z),
                            float(qx), float(qy), float(qz), float(qw))
                self.state_a.body_q.assign(bq)
            bqd = self.state_a.body_qd.numpy()
            if 0 <= bidx < len(bqd):
                bqd[bidx] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                self.state_a.body_qd.assign(bqd)
            # Re-seed the leg joints to the standing pose on reset so EVERY
            # episode starts standing (the step() seed is one-shot -- only
            # the first spawn). Only for the articulation root (body 0, the
            # chassis) and only when a standing pose was captured. Update the
            # free root's q (joint_q[0:7] = pos+quat) to the reset pose so
            # eval_fk places the chassis where the supervisor asked, then the
            # legs fold to the seeded standing angles.
            sjq = getattr(self, "_standing_jq", None)
            if sjq is not None and bidx == 0 and len(sjq) >= 7:
                jq2 = sjq.copy()
                jq2[0] = float(x); jq2[1] = float(y); jq2[2] = float(z)
                jq2[3] = float(qx); jq2[4] = float(qy)
                jq2[5] = float(qz); jq2[6] = float(qw)
                self.model.joint_q.assign(jq2)
                self.state_a.joint_q.assign(jq2)
                self.state_a.joint_qd.assign(self.model.joint_qd.numpy() * 0.0)
                newton.eval_fk(self.model, self.model.joint_q,
                               self.model.joint_qd, self.state_a)
            else:
                # GENERIC FREE BODY (not the chassis root): update its FREE
                # joint's 7-DOF q (pos+quat) in the CURRENT state so eval_fk
                # places it where the Supervisor asked WITHOUT disturbing the
                # arm. Without this, only body 0 stuck and every other free
                # body's pose-set was lost to eval_fk -- so a Supervisor
                # teleport (or a controller-side grasp "weld" that tracks the
                # tote to the gripper each tick) never moved the body under
                # MuJoCo. Find the body's free joint by its 7-DOF q span and
                # eval_fk from state_a.joint_q (the live pose), not model.joint_q
                # (the seeded pose, which would snap the whole robot back).
                jqs = self.model.joint_q_start.numpy()
                jc = self.model.joint_child.numpy()
                jq = self.state_a.joint_q.numpy()
                nq = len(jq)
                for j in range(len(jc)):
                    if int(jc[j]) != bidx:
                        continue
                    s = int(jqs[j])
                    e = int(jqs[j + 1]) if j + 1 < len(jqs) else nq
                    if e - s == 7 and s + 7 <= nq:        # free joint = pos+quat
                        jq[s + 0] = float(x); jq[s + 1] = float(y); jq[s + 2] = float(z)
                        jq[s + 3] = float(qx); jq[s + 4] = float(qy)
                        jq[s + 5] = float(qz); jq[s + 6] = float(qw)
                        self.state_a.joint_q.assign(jq)
                        self.model.joint_q.assign(jq)
                        newton.eval_fk(self.model, self.state_a.joint_q,
                                       self.state_a.joint_qd, self.state_a)
                    break
            # Newton state changed outside the solver -> refresh the
            # MuJoCo-side data from it on the next tick.
            self._mjc_dirty = True
        except Exception:
            pass

    def reset_joints_to_defaults(self):
        """Reset every revolute joint angle and velocity to the
        builder's initial values (joints at angle 0). Called once
        after a Supervisor reset so the actuator's PD starts from a
        defined state instead of drifting joint_q."""
        if self.model is None or self.state_a is None:
            return
        try:
            # model.joint_q holds the standing pose once the step() seed has
            # run (and reset_body_pose updates its free-root to the reset
            # pose). Restore it to BOTH state buffers (the working initial
            # seed set both; setting only state_a left a stale state_b that
            # could be read after the buffer swap) + zero velocities + FK.
            jq = self.model.joint_q.numpy()
            zqd = self.model.joint_qd.numpy() * 0.0
            for st in (self.state_a, self.state_b):
                if st is not None and hasattr(st, "joint_q"):
                    try:
                        st.joint_q.assign(jq)
                        st.joint_qd.assign(zqd)
                    except Exception:
                        pass
            newton.eval_fk(self.model, self.model.joint_q,
                           self.model.joint_qd, self.state_a)
            self._mjc_dirty = True
            # MuJoCo caches qpos at solver construction and does NOT pick up
            # a runtime state.joint_q assign -- so without this rebuild the
            # solver keeps simulating from the PREVIOUS (fallen) qpos and
            # every episode after the first collapses (frozen-actor proof:
            # ep_len 476->76 even though Newton state was restored to
            # standing). Rebuilding from the now-standing model.joint_q is
            # exactly what the step() seed does for the first spawn.
            import os as _rro
            if _rro.environ.get("OMNISIM_NEWTON_FORCE_MUJOCO"):
                try:
                    self.solver = newton.solvers.SolverMuJoCo(
                        self.model, use_mujoco_cpu=True)
                    # New solver = stale step graph.
                    self._step_graph = None
                    # ...and a new mj_model, which means every post-construction
                    # opt patch finalize() applied is gone. noslip is the one
                    # that decides whether a grasp holds at all, so re-apply it
                    # here rather than let episode 2 quietly run different
                    # physics from episode 1.
                    _rrn = int(getattr(self, "_noslip_iters_world", 0) or 0)
                    _rrv = _rro.environ.get("OMNISIM_NEWTON_NOSLIP")
                    if _rrv not in (None, ""):
                        _rrn = int(_rrv)
                    if _rrn > 0 and getattr(self.solver, "mj_model", None) is not None:
                        self.solver.mj_model.opt.noslip_iterations = _rrn
                except Exception:
                    pass
            self._reset_n = getattr(self, "_reset_n", 0) + 1
            if self._reset_n <= 20:
                import os as _ro
                _lp = _ro.environ.get("OMNISIM_NEWTON_LOG")
                if _lp:
                    import time as _rt
                    with open(_lp, "a") as _f:
                        _f.write(f"reset_joints#{self._reset_n} @{_rt.strftime('%H:%M:%S')} "
                                 f"step={getattr(self, '_stepn', 0)}: "
                                 f"knees={[round(float(jq[i]), 2) for i in (9, 12, 15, 18)]} "
                                 "(reset_joints_to_defaults called)\n")
        except Exception:
            pass

    def set_joint_target_pos(self, slot_id, pos, dof=0):
        # Companion to set_joint_target_vel: stash a position setpoint
        # for the POSITION_VELOCITY actuator. step() writes it into
        # control.joint_target each tick. Without this, target_ke acts
        # against an implicit 0 setpoint and the joint snaps back to
        # zero radians regardless of what the controller asked for.
        self.joint_targets_pos[self._target_key(slot_id, dof)] = float(pos)

    def set_joint_force(self, slot_id, tau, dof=0):
        # Torque-mode sink: stash a raw joint torque (Nm). step() writes it
        # into control.joint_f (applied generalized force -> MuJoCo
        # qfrc_applied), ADDITIVE over any POSITION_VELOCITY PD. For pure
        # torque control build the joint in EFFORT mode (no PD actuator) via
        # OMNISIM_NEWTON_TORQUE_MODE. Must be re-sent every tick: joint_f is
        # NOT auto-zeroed across steps.
        self.joint_forces[self._target_key(slot_id, dof)] = float(tau)

    def _multi_dof_cfgs(self, j, axes):
        """JointDofConfig list for a MOTORISED multi-DoF joint (hinge2 / ball).

        One config per entry of `axes`, reading the per-axis (ke, kd) pairs from
        j["gains"], the (lo, hi) pairs from j["limits"] and the scalars from
        j["efforts"] / j["vel_limits"]. A DoF with kd == 0 carries NO motor and
        is left free (actuator_mode NONE) -- that is how a Hinge2 with a motor on
        axis1 only, or a passive BallJoint DoF, stays genuinely passive instead of
        acquiring a phantom PD with zero gains.
        """
        Dof = newton.ModelBuilder.JointDofConfig
        limit_ke = self._env_float("OMNISIM_NEWTON_LIMIT_KE", 10000.0)
        limit_kd = self._env_float("OMNISIM_NEWTON_LIMIT_KD", 100.0)
        armature = self._env_float("OMNISIM_NEWTON_JOINT_ARMATURE", 0.0)
        gains, limits = j["gains"], j["limits"]
        efforts, vel_limits = j["efforts"], j["vel_limits"]
        cfgs = []
        for i, axis in enumerate(axes):
            ke, kd = gains[2 * i], gains[2 * i + 1]
            lo, hi = limits[2 * i], limits[2 * i + 1]
            motorised = kd > 0.0 or ke > 0.0
            kw = dict(
                axis=axis,
                target_ke=ke,
                target_kd=kd,
                limit_ke=limit_ke,
                limit_kd=limit_kd,
                armature=armature if motorised else 0.0,
                actuator_mode=(newton.JointTargetMode.POSITION_VELOCITY if motorised
                               else newton.JointTargetMode.NONE),
            )
            # Newton reads 0.0/0.0 as "unset", and JointDofConfig's own defaults
            # (+/-MAXVAL, 1e6) already mean "unlimited" -- so only pass a bound
            # that was actually declared.
            if lo != hi:
                kw["limit_lower"] = lo
                kw["limit_upper"] = hi
            if efforts[i] > 0.0:
                kw["effort_limit"] = efforts[i]
            if vel_limits[i] > 0.0:
                kw["velocity_limit"] = vel_limits[i]
            cfgs.append(Dof(**kw))
        return cfgs

    def _add_revolute_to_builder(self, j):
        """Push one queued revolute spec to the Newton builder. Returns
        the builder's joint index."""
        if j.get("kind") == "hinge2" and j.get("motorized"):
            # MOTORISED universal joint (OMNISIM_NEWTON_BALL_HINGE2): a native d6
            # whose two angular DoF each carry their own motor / limits / effort
            # ceiling. The converter turns them into two independently-actuated
            # mjJNT_HINGE elements on the child body, composed in order -- see
            # add_joint_hinge2_motorized for why no intermediate body is needed.
            return self.builder.add_joint_d6(
                parent=j["parent"], child=j["child"],
                angular_axes=self._multi_dof_cfgs(j, (j["axis"], j["axis2"])),
                parent_xform=wp.transform(j["p_anchor"], (0.0, 0.0, 0.0, 1.0)),
                child_xform=wp.transform(j["c_anchor"], j.get("c_rot", (0.0, 0.0, 0.0, 1.0))),
                collision_filter_parent=True)
        if j.get("kind") == "ball" and j.get("motorized"):
            # MOTORISED spherical joint (OMNISIM_NEWTON_BALL_HINGE2). The DoF axes
            # are the JOINT FRAME's own unit axes because the MuJoCo converter's
            # ball actuators are gear vectors in that frame; the Webots axis triad
            # rides in p_quat / c_quat. add_joint (not add_joint_ball) because only
            # the generic entry point accepts per-DoF configs for JointType.BALL.
            return self.builder.add_joint(
                newton.JointType.BALL,
                parent=j["parent"], child=j["child"],
                angular_axes=self._multi_dof_cfgs(
                    j, ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))),
                parent_xform=wp.transform(j["p_anchor"], j["p_quat"]),
                child_xform=wp.transform(j["c_anchor"], j["c_quat"]),
                collision_filter_parent=True)
        if j.get("kind") == "hinge2":
            # 2-DoF universal / Hinge2: a native Newton d6 joint with TWO free angular axes sharing the
            # anchor (e.g. a platform-cart caster: steer axis + roll axis, or a car front wheel). No motor
            # -- the cart wheels steer/spin freely; see the motorized branch above. Unspecified linear +
            # the 3rd angular DoF stay LOCKED, so this is exactly a 2-DoF joint. newton-ode-replacement W2.
            Dof = newton.ModelBuilder.JointDofConfig
            return self.builder.add_joint_d6(
                parent=j["parent"], child=j["child"],
                angular_axes=[Dof(axis=j["axis"]), Dof(axis=j["axis2"])],
                parent_xform=wp.transform(j["p_anchor"], (0.0, 0.0, 0.0, 1.0)),
                child_xform=wp.transform(j["c_anchor"], (0.0, 0.0, 0.0, 1.0)),
                collision_filter_parent=True)
        if j.get("kind") == "ball":
            # 3-DoF spherical joint: native Newton ball joint (JointType.BALL -- quaternion position, 3D
            # angular velocity; gimbal-free). No motor path here -- see the motorized branch above, which
            # goes through add_joint so it can carry per-DoF actuator configs.
            # newton-ode-replacement-plan.md W2.2.
            return self.builder.add_joint_ball(
                parent=j["parent"], child=j["child"],
                parent_xform=wp.transform(j["p_anchor"], (0.0, 0.0, 0.0, 1.0)),
                child_xform=wp.transform(j["c_anchor"], (0.0, 0.0, 0.0, 1.0)),
                collision_filter_parent=True)
        if j.get("kind") == "fixed":
            # 0-DOF weld (the TouchSensor un-fold, add_joint_fixed above).
            # p_anchor/p_quat = the child's spawn pose in the parent's frame,
            # child_xform = identity -- so the child is welded exactly where
            # the .wbt placed it. collision_filter_parent is deliberately not
            # passed: the two shipped add_joint_fixed call sites in finalize()
            # don't pass it either, and the finalize() self-collision filter
            # already excludes intra-robot shape pairs (this queued joint
            # contributes its parent-child edge to that adjacency).
            return self.builder.add_joint_fixed(
                parent=j["parent"], child=j["child"],
                parent_xform=wp.transform(j["p_anchor"], j["p_quat"]),
                child_xform=wp.transform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)))
        is_motor = j["target_kd"] > 0.0
        j["_is_motor"] = is_motor   # recorded for the finalize() joint diag
        mode = (newton.JointTargetMode.POSITION_VELOCITY if is_motor
                else newton.JointTargetMode.NONE)
        # Position spring + damping for legged-robot motors. The original
        # Husky-wheel config used target_ke=0 + target_kd=500 (pure
        # velocity tracking) because wheels don't need position lock,
        # but a quadruped's legs MUST hold a setpoint -- target_ke=0
        # means F = kd * vel_error which goes to zero at rest, gravity
        # then drifts the joints away from nominal and the robot
        # collapses. ke=200 produced "Spot stands intact but on its
        # knees" -- the leg-load gravity torque (~10 N*m per hip from
        # chassis weight transmitted through bent legs) had it tracking
        # NOMINAL_POSE with a steady-state error of ~0.05 rad per joint,
        # which translated to ~30 cm chassis drop from spawn. Bumped to
        # ke=1500, kd=30 so the steady-state error per joint drops to
        # well under a degree and the chassis sits at full standing
        # height. Critical damping at this stiffness is kd~17 for an
        # ~0.05 kg*m^2 leg; 30 is mildly over-damped, conservative for
        # the policy's gait-frequency commands.
        # With per-joint effort_limit / velocity_limit from the URDF in
        # place, the solver clips actuator force/velocity to the
        # physical maximums regardless of PD gain. We can therefore
        # use ODE-equivalent gentle gains (kp=20 was the URDF-tuned
        # value the existing Spot RL policy was trained against);
        # peak forces are bounded at the real motor torque ceiling
        # rather than by gain choice. Soft gains avoid the actuator
        # punching the body across the floor on each gait cycle that
        # we saw with the previous ke=1500 config.
        # Position-spring stiffness for the leg motors. ke=20 (the
        # previous value) was set on the theory that the URDF effort
        # limits make gain choice irrelevant -- but a soft spring only
        # reaches the effort cap at a LARGE angular error, so under
        # gravity the legs sag until the error is huge and the robot
        # collapses sideways (verified: holding NOMINAL_POSE with no
        # gait/policy, roll climbs 0.1->2.3 rad and the chassis sinks
        # 0.61->0.23 m within 0.7 s). ke=1500/kd=30 is the proven
        # full-standing-height gain; the per-joint effort_limit added
        # alongside still clips peak force, so the old "actuator punches
        # the body across the floor" failure does not return. Tunable via
        # env (read at articulation-build time) so gains can be swept
        # without recompiling the backend.
        import os as _kos
        def _gain(_k, _d):
            _v = _kos.environ.get(_k)
            try:
                return float(_v) if _v not in (None, "") else _d
            except ValueError:
                return _d
        # Env vars act as OPT-IN OVERRIDES only: when explicitly set
        # they win (so the Spot residual recipe + humanoid-balance
        # follow-ups can still sweep gains without recompiling), but
        # when unset we fall back to the per-joint target_ke /
        # target_kd that OmBasicJoint::flushPendingNewtonRegistrations
        # passes through addJointRevolute. That preserves the
        # husky-wheel verified config (ke=0, kd=500 -- see file header
        # lines 86-101) while leaving the legged-robot env-tuning
        # lever intact. Before 2026-05-28 fix: env defaults 20.0/3.0
        # were applied UNCONDITIONALLY, so the file-header config never
        # actually reached the builder for husky wheels; the result
        # was that huskies didn't drive (P6 surface symptom -- 0
        # damage events on Newton vs 149 on ODE in the same scenario).
        def _gain_override(_k):
            _v = _kos.environ.get(_k)
            if _v in (None, ""):
                return None
            try:
                return float(_v)
            except ValueError:
                return None
        _ke_override = _gain_override("OMNISIM_NEWTON_TARGET_KE")
        _kd_override = _gain_override("OMNISIM_NEWTON_TARGET_KD")
        # Per-joint values from the C++ caller (OmBasicJoint, which
        # hardcodes ke=0/kd=500 for motorized hinges). For backward
        # compatibility, if the per-joint ke wasn't stored (queued
        # specs predating the target_ke fix), fall back to 20.0 so
        # any leg-tuned robot that never set explicit ke keeps
        # working.
        _joint_ke = float(j.get("target_ke", 20.0))
        _joint_kd = float(j["target_kd"])
        TARGET_KE = _ke_override if _ke_override is not None else _joint_ke
        TARGET_KD = _kd_override if _kd_override is not None else _joint_kd
        # Finger-specific stiffness override (prismatic/slider joints only = the
        # gripper fingers). A thin-wall pinch needs a STIFF finger servo to develop
        # grip force from a small interference (the per-joint ke=effort*10 only
        # saturates a 50 N finger at a ~100 mm error -- useless on a 6 mm wall, so
        # a plain pinch is limp). The global OMNISIM_NEWTON_TARGET_KE would stiffen
        # the ARM hinges too and make them violent; this scopes the stiffness to the
        # fingers alone, leaving the arm at its safe per-joint gains. effort_limit
        # still caps the steady force, so a high finger ke means "reach the grip
        # force at a small interference", not "punch the part across the room".
        if j.get("kind") == "prismatic":
            _fke = _gain_override("OMNISIM_NEWTON_FINGER_KE")
            _fkd = _gain_override("OMNISIM_NEWTON_FINGER_KD")
            if _fke is not None:
                TARGET_KE = _fke
            if _fkd is not None:
                TARGET_KD = _fkd
        # Arm-hinge stiffness scope (companion to FINGER_KE above). The global
        # OMNISIM_NEWTON_TARGET_KE is tuned for the STIFF leg stand (ke~800); applied to
        # the low-torque ARM hinges it makes them VIOLENT when they MOVE -- a waving arm
        # saturates its 25 N.m motor at ke=800 and whips, and the hand at the chain's end
        # looks disconnected. Scope a gentle ke/kd to the arm revolute joints, identified
        # by their low effort_limit: on the G1, arm hinges are 25 N.m while ankles are 35,
        # hips 88, knees 139 -- so the default threshold 30 catches arms WITHOUT softening
        # the ankles/legs that must stay stiff to balance. Opt-in: default unset -> no
        # change (backward compatible). Tune the split via OMNISIM_NEWTON_ARM_EFFORT_THRESH.
        elif j.get("kind") == "revolute":
            _arm_ke = _gain_override("OMNISIM_NEWTON_ARM_KE")
            _arm_kd = _gain_override("OMNISIM_NEWTON_ARM_KD")
            if _arm_ke is not None or _arm_kd is not None:
                _arm_thresh = _gain("OMNISIM_NEWTON_ARM_EFFORT_THRESH", 30.0)
                if float(j.get("effort_limit", 0.0)) <= _arm_thresh:
                    if _arm_ke is not None:
                        TARGET_KE = _arm_ke
                    if _arm_kd is not None:
                        TARGET_KD = _arm_kd
        # Joint armature = rotor inertia + gearbox inertia, the part
        # of joint inertia that's NOT modelled by the limb's mass. URDF
        # doesn't specify this; defaulting to 0 means joints have only
        # the limb's small inertia, so external forces (e.g. a cube
        # collision dragging the chassis) can whip the joint at
        # arbitrary velocity — the URDF velocity_limit only caps
        # actuator-driven motion, not externally-driven motion.
        # Real Spot-class motors have substantial rotor+gear inertia;
        # ~0.05 kg·m² is realistic and makes external-force-driven
        # joint motion respect the velocity_limit. Tunable via env so
        # the rebuilt binary doesn't silently change Newton dynamics
        # for other sessions.
        ARMATURE = _gain("OMNISIM_NEWTON_JOINT_ARMATURE", 0.0)
        # Position-limit spring stiffness. Default 10000 is soft enough
        # that hard impacts can push joints momentarily past the URDF
        # range; raise via env if you need stricter enforcement.
        LIMIT_KE = _gain("OMNISIM_NEWTON_LIMIT_KE", 10000.0)
        LIMIT_KD = _gain("OMNISIM_NEWTON_LIMIT_KD", 100.0)
        # TORQUE MODE (OMNISIM_NEWTON_TORQUE_MODE=1): build motorized joints in
        # EFFORT mode with NO PD (ke=kd=0) so a controller's setJointForce
        # (-> control.joint_f) is the SOLE drive -- enables torque-mode balance
        # laws (capture-point / WBC) the position-mode PD structurally cannot
        # express. NOTE: global -- ALL motorized joints (incl. arms) become PD-
        # free, so the controller must command torque to every motor each tick.
        if is_motor and _kos.environ.get("OMNISIM_NEWTON_TORQUE_MODE") not in (None, "", "0"):
            mode = newton.JointTargetMode.EFFORT
            TARGET_KE = 0.0
            TARGET_KD = 0.0
        # Newton uses 0.0/0.0 as "no limit"; the URDF spec convention
        # for position limits is also (0,0) when unset. Only pass
        # non-zero limits through.
        joint_kwargs = dict(
            parent=j["parent"],
            child=j["child"],
            axis=j["axis"],
            parent_xform=wp.transform(j["p_anchor"], (0.0, 0.0, 0.0, 1.0)),
            child_xform=wp.transform(j["c_anchor"], j.get("c_rot", (0.0, 0.0, 0.0, 1.0))),
            target_pos=0.0,
            target_vel=0.0,
            target_ke=TARGET_KE if is_motor else 0.0,
            target_kd=TARGET_KD if is_motor else 0.0,
            armature=ARMATURE if is_motor else 0.0,
            limit_ke=LIMIT_KE,
            limit_kd=LIMIT_KD,
            actuator_mode=mode,
            collision_filter_parent=True,
        )
        if j["limit_lower"] != j["limit_upper"]:
            joint_kwargs["limit_lower"] = j["limit_lower"]
            joint_kwargs["limit_upper"] = j["limit_upper"]
        if j["effort_limit"] > 0.0 and not _kos.environ.get("OMNISIM_NEWTON_NO_EFFORT_LIMIT"):
            joint_kwargs["effort_limit"] = j["effort_limit"]
        if j["velocity_limit"] > 0.0:
            joint_kwargs["velocity_limit"] = j["velocity_limit"]
        # Prismatic (slider) joints -- gripper fingers -- share the entire
        # queue/topo-sort/gain path; only the builder call differs.
        if j.get("kind") == "prismatic":
            return self.builder.add_joint_prismatic(**joint_kwargs)
        return self.builder.add_joint_revolute(**joint_kwargs)

    def runtime_report(self):
        """Compact JSON naming the runtime that drove THIS world, and its device.

        Goes into the `.newton.json` verdict sidecar so every run is
        SELF-DESCRIBING. Without it, the moment a second newton/warp/mujoco
        version exists on a machine, no measurement in the tree can say which
        one produced it -- old and new results become indistinguishable after
        the fact, which is exactly how a performance campaign launders a
        regression. Land this BEFORE staging a second runtime, not after.

        `device` is here for the same reason: the CPU-device pin (0fc15a998)
        was worth 1.4-2.4x, so "which device held the model" is part of what a
        number means.
        """
        import json as _rj
        out = {}
        for mod in ("newton", "warp", "mujoco", "mujoco_warp"):
            try:
                _m = __import__(mod)
                v = getattr(_m, "__version__", None)
                if v is None:
                    from importlib.metadata import version as _mv
                    v = _mv(mod.replace("_", "-"))
                out[mod] = str(v)
            except Exception:
                out[mod] = None
        try:
            out["device"] = str(self.model.device)
        except Exception:
            out["device"] = None
        return _rj.dumps(out, sort_keys=True, separators=(",", ":"))

    def _ctl_target_pos(self):
        """`Control` position-target array, across the 1.5 rename.

        newton 1.5 REMOVED `Control.joint_target_pos` / `joint_target_vel` in
        favour of `joint_target_q` / `joint_target_qd` (deprecated in 1.3; the
        old names are now `RemovedAttribute` descriptors that RAISE on access,
        which is why the first 1.5 run flooded 64k warnings of
        "step raised: Control.joint_target_vel was removed in Newton 1.5").

        The rename is layout-preserving FOR US: `joint_target_q` is DOF-shaped
        exactly like the old field unless `newton.use_coord_layout_targets` is
        turned on, and it defaults to False (verified on the vendored 1.5.0).
        Every index in this file is a qd/DOF index, so nothing needs reindexing
        -- but if that global is ever flipped, `joint_target_q` becomes
        coord-shaped and every position target here would land on the wrong
        joint SILENTLY. Do not enable it without reindexing this file.

        Resolved per call via getattr so ONE source drives both runtimes.
        """
        c = self.control
        a = getattr(c, "joint_target_q", None)
        return a if a is not None else getattr(c, "joint_target_pos", None)

    def _ctl_target_vel(self):
        """`Control` velocity-target array; see _ctl_target_pos. DOF-shaped on
        both runtimes, matching `State.joint_qd`."""
        c = self.control
        a = getattr(c, "joint_target_qd", None)
        return a if a is not None else getattr(c, "joint_target_vel", None)

    def _apply_cpu_bvh_workaround(self):
        """newton#3805: on the CPU solver the collision BVH goes STALE.

        THE BUG. From newton 1.3.0 `SolverMuJoCo` synchronises inertial frames
        into the compiled `mj_model` AFTER compilation (`_sync_mjw_inertias_to_
        mjc_cpu`, absent in 1.2.0 -- source-verified 0 occurrences at tag v1.2.0
        vs 2 at v1.4.0/v1.5.0). MuJoCo builds its mid-phase BVH at COMPILE time
        from geom AABBs expressed in the body inertial frame, and nothing
        rebuilds it when `body_iquat` moves underneath it. The tree is then
        consulted with stale bounds, so genuine contacts are culled BEFORE the
        narrow phase ever sees them. It is scoped to `use_mujoco_cpu=True`
        (mjwarp has its own path) -- i.e. precisely OmniSim's default -- and
        upstream has it open with no PR and no milestone.

        WHY DISABLING MID-PHASE IS THE FIX AND NOT A BODGE. The mid-phase is a
        pure ACCELERATION structure: it culls candidate pairs before narrow
        phase. Disabling it makes MuJoCo test the pairs directly, so the contact
        SET becomes a superset of (never smaller than) the culled one -- correct
        results, more work. There is no public API to rebuild `mjModel.bvh_*`
        after the fact, so the alternative would be to un-do newton's own inertia
        sync, which would change the physics rather than the search.

        SCOPE. Applied only when the runtime actually has the defect (newton
        >= 1.3) AND we are on the CPU solver. `OMNISIM_NEWTON_MIDPHASE` is the
        override: `=1` forces mid-phase back ON (for A/B measurement of the cost
        and to reproduce the bug), `=0` forces it off. Logged once with the
        verdict so a run is self-describing.

        It also records `iquat_nonidentity`: the count of bodies whose inertial
        frame is NOT axis-aligned. If that is 0 the sync is a no-op and the bug
        cannot bite this model -- worth knowing before attributing anything to
        it.
        """
        sv = self._mjc_solver()
        if sv is None or not getattr(sv, "use_mujoco_cpu", False):
            return
        mjm = getattr(sv, "mj_model", None)
        mj = getattr(sv, "_mujoco", None)
        if mjm is None or mj is None:
            return
        try:
            import numpy as _np
            ver = tuple(int(x) for x in str(getattr(newton, "__version__", "0")).split(".")[:2])
        except Exception:                      # noqa: BLE001
            return
        affected = ver >= (1, 3)
        env = _os.environ.get("OMNISIM_NEWTON_MIDPHASE")
        if env is not None:
            disable = env.strip().lower() in ("0", "false", "off", "no")
        else:
            disable = affected
        # Diagnostic: can the stale-BVH sync even change anything here?
        nonident = -1
        try:
            iq = _np.asarray(mjm.body_iquat)
            if iq.size:
                nonident = int((_np.abs(iq[:, 0]) < 0.999999).sum())
        except Exception:                      # noqa: BLE001
            pass
        if disable:
            try:
                mjm.opt.disableflags |= int(mj.mjtDisableBit.mjDSBL_MIDPHASE)
            except Exception as _e:            # noqa: BLE001
                self._newton_log("bvh workaround FAILED to apply: %r" % (_e,))
                return
        self._newton_log(
            "newton#3805 CPU-BVH workaround: newton=%s affected=%s midphase=%s "
            "iquat_nonidentity=%d (disableflags=0x%x)"
            % (getattr(newton, "__version__", "?"), affected,
               "DISABLED" if disable else "on", nonident, int(mjm.opt.disableflags)))

    def _cloth_env_float(self, key, default):
        v = _os.environ.get(key)
        if v in (None, ""):
            return float(default)
        try:
            return float(v)
        except ValueError:
            self._newton_log("cloth: %s=%r is not a float, using %r" % (key, v, default))
            return float(default)

    def _cloth_env_int(self, key, default):
        v = _os.environ.get(key)
        if v in (None, ""):
            return int(default)
        try:
            return int(v)
        except ValueError:
            self._newton_log("cloth: %s=%r is not an int, using %r" % (key, v, default))
            return int(default)

    def _cloth_env_flag(self, key, default):
        v = _os.environ.get(key)
        if v in (None, ""):
            return bool(default)
        return v.strip().lower() not in ("0", "false", "off", "no")

    def _build_cloth_collision_pipeline(self, model, label):
        """A CollisionPipeline sized for soft contacts. Returns the pipeline.

        OmniSim has never built one of these: the rigid path lets SolverMuJoCo
        run its own collision and only keeps newton's narrow phase alive to
        feed get_contacts(). Particles have no such shortcut -- SolverVBD reads
        the `soft_contact_*` arrays a CollisionPipeline writes, and nothing
        else produces them, so on a cloth world this object IS the collision
        detection.

        ``enable_rigid_soft_full_surface_contact=True`` is the water-tight
        rigid-soft pass (collide.py:817): it adds soft EDGE and FACE records on
        top of the per-vertex ones, so a rigid feature that passes BETWEEN two
        cloth vertices is still caught. Upstream's
        example_vbd_gripper_soft_grid.py exists specifically to demonstrate
        that a grid whose only mesh feature crossing the jaws is an interior
        diagonal EDGE is gripped with the flag and slips out without it -- i.e.
        for a coarse sheet against a thin gripper this is the difference
        between a grasp and a drop, and it is not recoverable at runtime
        because the flag SIZES the soft-contact buffer at construction.

        ⚠ It also needs a volume SDF on every participating mesh/convex rigid
        shape. OmniSim's bounding objects are mostly analytic primitives (which
        ignore it), but URDF collision meshes are not, and nothing in this
        runtime provisions SDFs today. If construction refuses, fall back to
        the per-vertex pass and SAY SO -- a silently degraded grip that drops
        the sheet is exactly the failure this file's rules are written against.
        """
        margin = self._cloth_env_float("OMNISIM_CLOTH_CONTACT_MARGIN", 0.01)
        full = self._cloth_env_flag("OMNISIM_CLOTH_FULL_SURFACE_CONTACT", True)
        if full:
            try:
                p = newton.CollisionPipeline(
                    model,
                    soft_contact_margin=margin,
                    enable_rigid_soft_full_surface_contact=True,
                )
                self._newton_log("cloth: %s CollisionPipeline margin=%.4g "
                                 "full_surface=ON" % (label, margin))
                return p
            except Exception as _e:               # noqa: BLE001
                self._newton_log(
                    "cloth: %s CollisionPipeline REFUSED full-surface rigid-soft "
                    "contact (%r) -- falling back to the per-VERTEX pass. A rigid "
                    "feature passing between two cloth vertices will now be MISSED; "
                    "a coarse sheet can slip out of a gripper. Provision volume SDFs "
                    "on the participating mesh shapes to restore it."
                    % (label, _e))
        p = newton.CollisionPipeline(model, soft_contact_margin=margin)
        self._newton_log("cloth: %s CollisionPipeline margin=%.4g full_surface=off"
                         % (label, margin))
        return p

    # ------------------------------------------------------------------
    # WHOLE-WORLD VBD  --  WorldInfo.newtonSolver "vbd"
    # ------------------------------------------------------------------

    def _vbd_world(self):
        """True when this world asked for ONE SolverVBD to own EVERYTHING --
        rigid bodies, joints AND particles -- rather than the coupled
        SolverMuJoCo + SolverVBD pair that "mujoco+vbd" builds.

        WHY THIS IS A SEPARATE MODE AND NOT A TUNING FLAG. The coupled path is
        the right default and stays it: it keeps every MuJoCo joint feature
        OmniSim's rigid worlds are built on (the effortLimit*10 PD servo,
        armature, POSITION_VELOCITY target mode). But it is structurally
        incapable of one thing -- a rigid body and a cloth particle relaxed in
        the SAME sweep. Across a proxy the cloth only ever sees a jaw whose
        pose was synced from the PREVIOUS rigid solve, so a pinch is always
        resolved against a one-step-stale pad. newton's own cloth-grasp example
        (``examples/vbd/example_vbd_gripper_soft_grid.py``) puts the gripper and
        the grid on ONE SolverVBD for exactly that reason, and it is the only
        configuration in the package where a rigid gripper lifts a soft mesh.

        ⚠ THE TRADE IS LARGE AND IT IS NOT NEGOTIABLE AWAY. Under VBD there is
        no mj_model at all, so raycast-backed sensors, Connector/VacuumGripper
        welds, TouchSensor and the MuJoCo contact readback are all gone; and
        five joint features are IGNORED IN SILENCE by newton itself (its
        unsupported list at solver_vbd.py:119-135 is a DOCSTRING -- verified,
        no warning, no validation, the arrays are simply never read by any VBD
        kernel). ``_vbd_capability_report`` exists to convert that silence into
        a warning naming each field THIS world actually uses.

        OMNISIM_NEWTON_VBD_WORLD is value-parsed: =1 forces the mode on a world
        that did not declare it, =0 refuses it on one that did.
        """
        import os as _o
        _env = (_o.environ.get("OMNISIM_NEWTON_VBD_WORLD") or "").strip().lower()
        if _env not in ("", "auto"):
            return _env not in ("0", "false", "off", "no")
        return (getattr(self, "_solver_pref", None) or "").strip().lower() == "vbd"

    def _vbd_solver_kwargs(self, rigid_contact_hard_default):
        """The SolverVBD constructor kwargs shared by BOTH VBD paths.

        Extracted from _build_cloth_coupled_solver unchanged, so the coupled
        path builds a byte-identical kwarg dict to the one it built before this
        function existed. The ONE parameter is ``rigid_contact_hard``, and it
        genuinely differs between the two callers -- see its comment below.
        """
        # Self-contact radius/margin default to the authored particle radius.
        # newton's own defaults are 0.2 m for BOTH, which on a 5 cm cell would
        # make every particle "self-contact" most of the sheet -- and 0.2/0.2
        # also sits exactly ON the boundary that passes newton's own validation
        # while violating its own advice that the MARGIN be comfortably larger
        # than the radius (the margin is the band in which a contact is
        # DETECTED; equal to the radius means a fast-moving fold can cross the
        # whole band inside one substep and be missed). Upstream's
        # example_mujoco_vbd_coupled_solver.py:141-147 pins 0.01 alongside a
        # 0.01 particle radius; this scales both to the authored radius and
        # opens the margin to 1.5x it.
        # ⚠ Every PARTICLE SOURCE, not just cloth. Reading only cloth_grids here
        # meant a world whose particles all came from a SoftBody silently fell
        # back to the 0.01 default below, ignoring its authored particleRadius --
        # a wrong self-contact radius, with nothing in the log to say so.
        _radii = [g["particle_radius"] for g in (self.cloth_grids + self.soft_grids)
                  if g["particle_radius"] > 0.0]
        _self_r = min(_radii) if _radii else 0.01
        _self_m = 1.5 * _self_r
        _self_pref = int(getattr(self, "_cloth_self_contact_world", -1))
        return {
            "iterations": self._cloth_env_int("OMNISIM_CLOTH_VBD_ITERATIONS", 10),
            "friction_epsilon": self._cloth_env_float("OMNISIM_CLOTH_FRICTION_EPSILON", 0.01),
            # ⚠ NON-NEGOTIABLE. newton's default is False. Without it a fold
            # passes straight THROUGH itself -- the cloth is not solid against
            # its own surface -- and there is no error, no warning and no
            # contact record to notice it by. A draped or folded sheet is the
            # normal case for cloth, so the default is wrong for every world
            # this runtime will ever author.
            # env > WorldInfo.newtonClothSelfContact > default(True). The
            # world field is how a GRASP world says "off" in its own file; see
            # set_cloth_self_contact() for why neither value is correct for
            # every world and why the gap is 24x.
            "particle_enable_self_contact": self._cloth_env_flag(
                "OMNISIM_CLOTH_SELF_CONTACT",
                bool(_self_pref) if _self_pref >= 0 else True),
            "particle_self_contact_radius": self._cloth_env_float(
                "OMNISIM_CLOTH_SELF_CONTACT_RADIUS", _self_r),
            "particle_self_contact_margin": self._cloth_env_float(
                "OMNISIM_CLOTH_SELF_CONTACT_MARGIN", _self_m),
            # ⚠ NON-NEGOTIABLE. newton's default is 256 per body. MEASURED:
            # 266 soft contacts on a body with only 153 particles in play, so
            # 256 overflows on a scene far smaller than any real one -- and the
            # overflow's ONLY signal is a wp.printf from inside a warp kernel.
            # That is invisible here twice over: omnisim-bin is a GUI-subsystem
            # binary on Windows (stdout is discarded outright, see _newton_log)
            # and the embedded interpreter's stdout does not reach the host
            # reliably anywhere. So the symptom is "the grip feels wrong", with
            # no message, exactly like the njmax/nconmax cliff documented in
            # AGENTS.md. 2048 is 8x the default at a per-body cost of a few
            # ints; raise it further if a dense sheet is being pinched.
            # ⚠ A STANDALONE GRIPPER+CLOTH PROOF MEASURED A PEAK OF 354-482 ON
            # ONE JAW BODY at 20 mm cells under a 50 mm pad, so 256 is not
            # marginal here, it is a factor of two short. Scale with cloth
            # resolution and read the peak back off
            # solver.body_particle_contact_counts (NOT
            # body_particle_contact_overflow_max, which is written only AFTER an
            # overflow and so reads 0 on a healthy run -- it cannot be used as a
            # headroom gauge).
            "rigid_body_particle_contact_buffer_size": self._cloth_env_int(
                "OMNISIM_CLOTH_RIGID_PARTICLE_BUFFER", 2048),
            # Body-body contacts. On the COUPLED path these are between PROXY
            # bodies and must be soft (penalty-only) per
            # example_proxy_joint_gripper.py:104 -- the real rigid-rigid contact
            # is MuJoCo's job on the other side of the coupling, and hard AL
            # duals here would fight it. On the WHOLE-WORLD path there is no
            # other side: VBD *is* the rigid solver, so newton's own default
            # (hard = AL duals + C0 stabilisation) is the correct one and
            # penalty-only would let bodies sink into the floor.
            "rigid_contact_hard": self._cloth_env_flag(
                "OMNISIM_CLOTH_RIGID_CONTACT_HARD", rigid_contact_hard_default),
        }

    def _resolve_coupled_bodies(self, bodies):
        """Which rigid bodies the CLOTH solver may see (the Proxy roster).

        THE PROBLEM THIS SOLVES. Until this existed the Proxy declared
        ``bodies=list(range(n_bodies))`` -- EVERY rigid body in the world became
        a live proxy inside the cloth's collision view. Tables, walls, every
        link of every arm. newton supports narrowing this natively and its own
        Franka cable demo lists only the hand + fingers, because
        ``_apply_entry_shape_visibility`` (solver_coupled.py:805-838) CLEARS
        ``COLLIDE_SHAPES | COLLIDE_PARTICLES | HYDROELASTIC`` on every shape
        whose body is not a proxy -- so a narrow roster is both cheaper and
        more controllable, and a full one is a scene-wide broadphase against
        the sheet.

        ⚠ IT IS SAFE FOR THE RIGID SIDE, AND THAT IS NOT AN ASSUMPTION. The
        clearing writes through ``view._cow_array("shape_flags")`` -- copy on
        write, view-local -- so it never mutates the parent model and MuJoCo's
        own rigid-rigid collision is untouched by any choice made here.

        ⚠ BUT IT IS NOT SAFE FOR THE CLOTH. A body left off the roster is
        INVISIBLE to the sheet: the fabric falls straight through it, silently.
        The one thing that survives narrowing is WORLD/STATIC shape (newton's
        body -1, which includes the implicit z=0 ground plane), because
        ``_entry_visible_shapes`` adds ``body_shapes[-1]`` whenever the entry
        declares no explicit shapes. An authored floor Solid is a real body and
        is NOT that -- so a world that narrows the roster and forgets its table
        gets a sheet on the ground plane, or through the world.

        RESOLUTION, in precedence order:
          1. OMNISIM_CLOTH_COUPLED_BODIES -- "all", or a comma-separated index
             list. The launch-time override, as every newton* field has one.
          2. ALLOWLIST, if any Solid declared newtonClothCoupling 1: exactly
             those bodies, and nothing else.
          3. DENYLIST otherwise: every body except those declaring -1.
        A world using none of the field takes case 3 with an empty denylist,
        i.e. the full roster it had before -- bit-identical.
        """
        import os as _o
        roster = getattr(self, "_cloth_coupling", None) or {}
        env = (_o.environ.get("OMNISIM_CLOTH_COUPLED_BODIES") or "").strip()
        if env:
            if env.lower() == "all":
                self._newton_log("cloth: OMNISIM_CLOTH_COUPLED_BODIES=all -- every body "
                                 "is a cloth proxy (any Solid.newtonClothCoupling "
                                 "declaration is overridden).")
                return list(bodies), "env:all"
            try:
                want = {int(t) for t in env.replace(";", ",").split(",") if t.strip()}
            except ValueError:
                self._newton_log("cloth: OMNISIM_CLOTH_COUPLED_BODIES=%r is not "
                                 "\"all\" or a comma-separated index list -- IGNORED." % env)
                want = None
            if want is not None:
                out = [b for b in bodies if b in want]
                unknown = sorted(want - set(bodies))
                if unknown:
                    self._newton_log("cloth: OMNISIM_CLOTH_COUPLED_BODIES names %d body "
                                     "index(es) that do not exist in this world (%s) -- "
                                     "ignored." % (len(unknown), unknown[:8]))
                if out:
                    return out, "env:list"
                self._newton_log("cloth: OMNISIM_CLOTH_COUPLED_BODIES=%r selected NO "
                                 "existing body -- falling back to every body." % env)
                return list(bodies), "env:list_empty_fallback"

        if not roster:
            return list(bodies), "all"

        allow = sorted(b for b, m in roster.items() if m > 0)
        deny = sorted(b for b, m in roster.items() if m < 0)
        if allow:
            out = [b for b in bodies if roster.get(b, 0) > 0]
            mode = "allowlist"
            if deny:
                self._newton_log(
                    "cloth: newtonClothCoupling declares BOTH %d body(ies) as coupled and "
                    "%d as not-coupled. The allowlist wins and the -1 declarations are "
                    "redundant -- anything not named +1 is already excluded."
                    % (len(allow), len(deny)))
        else:
            out = [b for b in bodies if roster.get(b, 0) >= 0]
            mode = "denylist"

        if not out:
            # Refusing outright would kill the world; honouring it would build a
            # Proxy newton rejects (its validation needs at least one body,
            # particle or joint). Fall back, and say so at full volume -- a
            # silent fallback here is exactly the declared-but-unread failure
            # this tree keeps getting bitten by.
            self._newton_log(
                "cloth: ⚠ Solid.newtonClothCoupling resolved to ZERO coupled bodies "
                "(%s over %d bodies). A cloth that can see no rigid body falls through "
                "everything, and a Proxy with an empty roster is rejected outright -- so "
                "FALLING BACK to every body. Check the field's sign: +1 couples, -1 "
                "excludes, 0 is unset." % (mode, len(bodies)))
            return list(bodies), "empty_fallback"
        return out, mode

    def _vbd_capability_report(self, model):
        """Name every feature THIS world uses that SolverVBD will ignore.

        ⚠ THE WHOLE POINT IS THAT NEWTON DOES NOT DO THIS. Its unsupported
        list (solver_vbd.py:119-135) is a class docstring: grepping the entire
        vbd solver directory for those attribute names returns only the three
        docstring lines themselves, so `joint_armature`, `joint_friction`,
        `joint_effort_limit`, `joint_velocity_limit` and `joint_target_mode`
        are read by no kernel and dropped with no message. A world that
        declares `Motor.maxTorque` and gets no torque ceiling would otherwise
        have to be diagnosed from behaviour alone.

        Reports only what is ACTUALLY PRESENT, so a world that uses none of it
        gets no noise. Everything here is a read of the finalized model, so it
        describes the model the solver is about to be handed, not the .wbt.
        """
        import numpy as _np

        def _arr(name):
            a = getattr(model, name, None)
            if a is None:
                return None
            try:
                v = _np.asarray(a.numpy()).reshape(-1)
            except Exception:                            # noqa: BLE001
                return None
            return v if v.size else None

        notes = []
        # --- joint features newton drops in silence -----------------------
        arm = _arr("joint_armature")
        if arm is not None and float(_np.abs(arm).max()) > 0.0:
            notes.append("joint_armature (max %.4g, from OMNISIM_NEWTON_JOINT_ARMATURE "
                         "or the motorised-joint default) -- rotor inertia is GONE, so "
                         "motorised joints will feel lighter and may need lower gains"
                         % float(_np.abs(arm).max()))
        fric = _arr("joint_friction")
        if fric is not None and float(_np.abs(fric).max()) > 0.0:
            notes.append("joint_friction (max %.4g) -- dry joint friction is GONE"
                         % float(_np.abs(fric).max()))
        eff = _arr("joint_effort_limit")
        if eff is not None:
            fin = eff[_np.isfinite(eff)]
            # newton's "unlimited" sentinel is a huge finite value, not inf.
            if fin.size and float(fin.min()) < 1.0e5:
                notes.append("joint_effort_limit (min %.4g N or N*m, from Motor.maxTorque / "
                             "the URDF effort limit) -- the torque CEILING IS NOT ENFORCED, "
                             "so a joint can pull arbitrarily hard" % float(fin.min()))
        vel = _arr("joint_velocity_limit")
        if vel is not None:
            fin = vel[_np.isfinite(vel)]
            if fin.size and float(fin.min()) < 1.0e5:
                notes.append("joint_velocity_limit (min %.4g, from Motor.maxVelocity) -- "
                             "the speed ceiling IS NOT ENFORCED" % float(fin.min()))
        mode = _arr("joint_target_mode")
        if mode is not None and int(_np.max(mode)) > 0:
            notes.append("joint_target_mode (POSITION_VELOCITY on %d DoF) -- VBD has ONE "
                         "drive law and applies it whenever target_ke or target_kd is "
                         "non-zero, so the mode selection is inert. The gains themselves "
                         "(joint_target_ke / joint_target_kd) ARE honoured."
                         % int(_np.count_nonzero(mode)))
        # --- joint TYPES: DISTANCE raises rather than degrades ------------
        jt = _arr("joint_type")
        if jt is not None:
            try:
                _dist = int(newton.JointType.DISTANCE)
            except Exception:                            # noqa: BLE001
                _dist = 5
            n_dist = int(_np.count_nonzero(jt == _dist))
            if n_dist:
                # Pre-empt newton's own NotImplementedError from deep inside
                # _init_joint_constraint_layout, which names a joint index and
                # nothing an author can act on.
                raise RuntimeError(
                    "newtonSolver \"vbd\": this world has %d DISTANCE joint(s), which "
                    "SolverVBD does not implement (it supports BALL, FIXED, FREE, "
                    "REVOLUTE, PRISMATIC, D6, CABLE). Use newtonSolver \"mujoco+vbd\" "
                    "for this world, or remove the DISTANCE joint." % n_dist)
        # --- everything mj_model-backed disappears with MuJoCo ------------
        _weld_slots = getattr(self, "_weld_slots", None)
        if _weld_slots:
            notes.append("Connector / VacuumGripper WELDS (%d) -- these are MuJoCo "
                         "equality constraints and VBD has no equality solver, so they "
                         "WILL NOT HOLD" % len(_weld_slots))
        for note in notes:
            self._newton_log("newtonSolver \"vbd\": IGNORED -- %s" % note)
        self._newton_log(
            "newtonSolver \"vbd\": no mj_model exists on this path, so raycast-backed "
            "devices (DistanceSensor / Receiver / LightSensor / Radar / Camera-recognition "
            "occlusion), Connector / VacuumGripper welds, TouchSensor and the MuJoCo "
            "contact readback (getContactPoints, GET /sim/contacts, /sim/grips) are ALL "
            "unavailable. Rigid contact is AVBD (penalty / augmented-Lagrangian), NOT "
            "MuJoCo's cone -- newtonCone / newtonImpratio / newtonIterations / "
            "newtonLsIterations / newtonNoslipIterations / newtonNjmax / newtonNconmax are "
            "all MuJoCo fields and none of them is read here.")
        return notes

    def _build_vbd_world_solver(self):
        """ONE SolverVBD owning every rigid body, joint and particle.

        This is newton's proven single-solver cloth-grasp configuration
        (examples/vbd/example_vbd_gripper_soft_grid.py) made reachable from a
        world file. Construction order is CollisionPipeline -> Contacts ->
        SolverVBD, which is what solver_vbd.py:174-175 specifies for CUDA-graph
        capture (the example itself builds them the other way round and gets
        away with it only because it captures no graph).
        """
        model = self.model

        # Same soft-contact material as the coupled path, and it must be
        # written after finalize() and before the first step because these live
        # on the MODEL, not on a solver.
        # ⚠ model.soft_contact_ke is AVERAGED with the contacting shape's own
        # material (arithmetic mean for ke/kd, geometric for mu), so authoring
        # 1e3 against a default shape ke of 2.5e3 yields an effective 1750.
        model.soft_contact_ke = self._cloth_env_float("OMNISIM_CLOTH_SOFT_KE", 1.0e3)
        model.soft_contact_kd = self._cloth_env_float("OMNISIM_CLOTH_SOFT_KD", 1.0e1)
        model.soft_contact_mu = self._cloth_env_float(
            "OMNISIM_CLOTH_SOFT_MU", float(getattr(self, "_ground_mu", 1.0) or 1.0))

        self._vbd_capability_report(model)

        # newton REQUIRES body_q to agree with joint_q at solver construction
        # (solver_vbd.py:169-172: "VBD uses model.body_q as the structural rest
        # pose and reads model.joint_q for drive/limit rest-angle offsets"), and
        # the gripper example calls eval_fk immediately before constructing for
        # exactly this reason. OmniSim authors body poses and joint angles from
        # the same .wbt so they should already agree; this makes that an
        # enforced invariant rather than a hope. OMNISIM_NEWTON_VBD_EVAL_FK=0 is
        # the hatch if a world is ever found whose joint_q does NOT reproduce
        # its authored body poses.
        if self._cloth_env_flag("OMNISIM_NEWTON_VBD_EVAL_FK", True):
            try:
                newton.eval_fk(model, model.joint_q, model.joint_qd, model)
            except Exception as _e:                      # noqa: BLE001
                self._newton_log("newtonSolver \"vbd\": eval_fk before solver "
                                 "construction failed (%r) -- continuing with the "
                                 "authored body_q." % (_e,))

        vbd_kwargs = self._vbd_solver_kwargs(rigid_contact_hard_default=True)

        self.collision_pipeline = self._build_cloth_collision_pipeline(model, "world")
        self._contacts_cache = self.collision_pipeline.contacts()
        self.solver_soft = newton.solvers.SolverVBD(model, **vbd_kwargs)
        self.solver = self.solver_soft
        # ⚠ LOAD-BEARING: _mjc_solver() returns None from here, and every
        # mj_model-backed readback in this file is guarded on that. Setting it
        # to anything else would make those readbacks answer about a model that
        # does not exist.
        self.solver_mjc = None
        # ⚠ SolverVBD IS A MAXIMAL-COORDINATE SOLVER: it integrates body_q /
        # body_qd / particle_q / particle_qd and NEVER writes state.joint_q. It
        # reads model.joint_q exactly once, at construction, to build
        # joint_rest_angle. So every joint-space READBACK in this file --
        # PositionSensor, Motor.getPositionSensor(), JointParameters.position,
        # /robot/<def>/joints -- would report the AUTHORED seed for ever while
        # the robot moves correctly underneath it.
        #
        # This is not hypothetical and it is not new: measured on the cloth-fold
        # world, every joint read exactly 0.0000 against non-zero commands while
        # the engine's own step log showed the bodies AT the commanded
        # coordinates (gantry_x cmd -0.2800 -> body x -0.280; jaw cmd +0.0372 ->
        # pad moved +0.037). It reads as a dead robot and it is a dead SENSOR.
        # The same gap existed for XPBD, was fixed in 7a329c191, and was deleted
        # along with XPBD in 94f042225; this path reintroduced a maximal-
        # coordinate solver without restoring the refresh.
        #
        # _step_impl consumes this flag to run newton.eval_ik() once per tick.
        self._solver_maintains_joint_q = False

        n_bodies = int(getattr(model, "body_count", 0) or 0)
        n_joints = int(getattr(model, "joint_count", 0) or 0)
        n_particles = int(getattr(model, "particle_count", 0) or 0)
        self._newton_log(
            "newtonSolver \"vbd\": ONE SolverVBD owns the whole world -- %d bodies, "
            "%d joints, %d particles. iterations=%d self_contact=%s r=%.4g "
            "rigid_contact_hard=%s soft_buf=%d | soft_ke=%.4g soft_kd=%.4g soft_mu=%.4g"
            % (n_bodies, n_joints, n_particles, vbd_kwargs["iterations"],
               vbd_kwargs["particle_enable_self_contact"],
               vbd_kwargs["particle_self_contact_radius"],
               vbd_kwargs["rigid_contact_hard"],
               vbd_kwargs["rigid_body_particle_contact_buffer_size"],
               model.soft_contact_ke, model.soft_contact_kd, model.soft_contact_mu))

    def _build_cloth_coupled_solver(self, mjc_kwargs):
        """Robot on SolverMuJoCo + cloth on SolverVBD, coupled over ONE Model.

        THE ARCHITECTURE, and why it is not the obvious one. The obvious move
        is to put everything on SolverVBD, which simulates rigid bodies too.
        It is wrong here: OmniSim's working grasp is built on MuJoCo joint
        features VBD does not implement -- the `effortLimit * 10` position PD
        servo every Newton joint is constructed with (see the header of this
        file and docs/guide/friction-grasp.md), armature, and the
        POSITION_VELOCITY target mode. Moving the robot to VBD would silently
        change every actuator in the tree.

        So: the robot keeps the EXACT SolverMuJoCo it would have had (same
        **mjc_kwargs -- same impratio, cone, iterations, njmax/nconmax), the
        cloth particles go on SolverVBD, and SolverCoupledProxy exchanges them
        over one shared Model. This is the wiring of upstream's
        example_proxy_joint_gripper.py:110-152 (a MuJoCo palm + prismatic
        fingers gripping a VBD grid -- our exact scenario, CPU-tested upstream)
        and example_mujoco_vbd_coupled_solver.py:161-193 (a MuJoCo articulated
        chain on VBD cloth, contact forces fed back).

        HOW THE COUPLING WORKS, in one paragraph, because the failure modes
        follow from it: the "mjc" entry owns every body and joint; the "vbd"
        entry owns only the particles; and a Proxy mapping hands VBD a set of
        virtual proxy bodies that mirror MuJoCo's. Each step VBD solves the
        cloth against those proxies and the contact impulses it computes are
        fed back to MuJoCo as forces (mode="lagged": source begin-poses and
        end-velocities are synced, then the lagged feedback is prepared so it
        is not double-counted). The robot therefore FEELS the cloth without
        MuJoCo ever knowing particles exist.
        """
        from newton.solvers.experimental.coupled import SolverCoupledProxy

        mjc_kwargs = dict(mjc_kwargs)
        model = self.model

        # ---- RIGID CONTACT OWNERSHIP: THE SILENT-CONTACTLESS-WORLD TRAP ----
        # ⚠ `use_mujoco_cpu=True` + `use_mujoco_contacts=False` IS A WORLD WITH
        # NO CONTACTS AT ALL, AND NOTHING SAYS SO. Source-verified in the
        # vendored newton 1.5.0: SolverMuJoCo.step() (solver_mujoco.py:3829-3849)
        # injects the newton `Contacts` object into MuJoCo on its GPU branch
        # ONLY -- `_convert_contacts_to_mjwarp(...)` is called at :3845-3847,
        # inside `if not self.mjw_model.opt.run_collision_detection`. The CPU
        # branch at :3831-3838 calls mj_step and never looks at `contacts`. So
        # with MuJoCo's own detection switched off and newton's never delivered,
        # the robot falls through the floor. The only combination the
        # constructor rejects is `enable_sleeping and not use_mujoco_contacts`
        # (:3510-3512); this one it accepts silently.
        #
        # OmniSim's default solver is `use_mujoco_cpu=True` (the deterministic
        # reference), so the trap is directly on our path. Two configurations
        # are therefore legal here and the third is REFUSED:
        #
        #  (A) DEFAULT, and the one MEASURED to work: keep MuJoCo's own contact
        #      detection (`use_mujoco_contacts` untouched = True). MuJoCo
        #      resolves rigid-rigid exactly as on a rigid world; the
        #      CollisionPipeline resolves particle-vs-rigid and particle-self
        #      for VBD; the proxy feeds VBD's contact impulses back. MEASURED
        #      on this machine (RTX 3060 laptop, newton 1.5.0), two boxes
        #      placed 20 m from the cloth so it cannot touch them: rest heights
        #      0.099892244 / 0.149892256 rigid-only vs 0.099892229 / 0.149892226
        #      coupled -- a 1.5e-08 m difference, i.e. float noise, NOT the
        #      double-resolution one might fear from having both solvers alive.
        #      Meanwhile a box UNDER the sheet is pressed from 0.099892 to
        #      0.098761 by its weight, which is the coupling doing its job.
        #  (B) OPT-IN, and upstream's own shape: `use_mujoco_contacts=False`, so
        #      newton's pipeline is the single source of contact for both
        #      solvers. Every multiphysics example ships this -- and every one
        #      of them also runs the GPU branch, because SolverMuJoCo's
        #      `use_mujoco_cpu` DEFAULTS TO FALSE (:3436) and none of them
        #      overrides it. Selecting (B) therefore FORCES mujoco_warp, since
        #      that is the only branch on which the flag functions at all.
        #      ⚠ That is not free: AGENTS.md scopes OmniSim's bitwise
        #      determinism claim to the CPU mj_step path and records it as
        #      REFUTED on mujoco_warp (0 of 24 same-config cold pairs bitwise).
        #      So (B) buys upstream-parity contact handling at the cost of
        #      determinism, which is why it is not the default.
        #  (C) REFUSED: `use_mujoco_contacts=False` while staying on CPU
        #      mj_step. This is the trap above. A wrong result is worse than a
        #      lost one -- raise instead of shipping a contactless world.
        #
        # ⚠ AND NEVER `disable_contacts=True`. upstream's
        # example_proxy_joint_gripper.py:118 passes it, and copying that would
        # be a mistake here: it sets mjDSBL_CONTACT and nconmax=0, so MuJoCo
        # resolves NO contact whatsoever. It is harmless in that example only
        # because its scene's sole contact IS the soft block -- no floor, no
        # table. Every OmniSim world has a floor.
        _newton_contacts = self._cloth_env_flag("OMNISIM_CLOTH_NEWTON_CONTACTS", False)
        if _newton_contacts:
            mjc_kwargs["use_mujoco_contacts"] = False
            if mjc_kwargs.get("use_mujoco_cpu", True):
                mjc_kwargs["use_mujoco_cpu"] = False
                self._newton_log(
                    "cloth: OMNISIM_CLOTH_NEWTON_CONTACTS=1 -> use_mujoco_contacts=False, "
                    "which the CPU mj_step branch CANNOT honour (it never consumes the "
                    "newton Contacts object) -- forcing use_mujoco_cpu=False (mujoco_warp). "
                    "⚠ bitwise determinism is REFUTED on that path; see "
                    "docs/benchmarks/determinism-scope.md.")
        elif mjc_kwargs.get("use_mujoco_contacts", True) is False:
            raise RuntimeError(
                "cloth: refusing to build a world with use_mujoco_contacts=False on the "
                "CPU mj_step solver -- MuJoCo's own collision would be off and newton's "
                "contacts are only injected on the mujoco_warp branch "
                "(solver_mujoco.py:3845), so NOTHING would collide and nothing would "
                "say so. Either leave MuJoCo's contacts on, or set "
                "OMNISIM_CLOTH_NEWTON_CONTACTS=1 to move the whole world to mujoco_warp.")
        n_bodies = int(getattr(model, "body_count", 0) or 0)
        n_joints = int(getattr(model, "joint_count", 0) or 0)
        p_start = int(self.cloth_particle_start)
        p_end = int(self.cloth_particle_end)
        particles = list(range(p_start, p_end))
        if not particles:
            raise RuntimeError("cloth path entered with no particles "
                               "[%d,%d)" % (p_start, p_end))

        # ---- SOFT-CONTACT MATERIAL --------------------------------------
        # The starting point measured for OmniSim cloth, and the same triple
        # example_vbd_gripper_soft_grid.py:98-100 uses for its grip: soft
        # enough not to crush the mesh, sticky enough (mu=1.0) to be held.
        # These live on the MODEL, not on a solver, so they must be written
        # after finalize() and before the first step.
        model.soft_contact_ke = self._cloth_env_float("OMNISIM_CLOTH_SOFT_KE", 1.0e3)
        model.soft_contact_kd = self._cloth_env_float("OMNISIM_CLOTH_SOFT_KD", 1.0e1)
        # ⚠ CLOTH FRICTION NOW DEFAULTS TO THE WORLD'S OWN DECLARED FRICTION,
        # not to a hardcoded 1.0. A world that says `newtonGroundMu 6` was
        # getting mu=1.0 on every particle contact, so the fabric slid on
        # surfaces the same world had declared grippy -- a declared-but-unread
        # field, which is precisely the failure mode this tree has been bitten
        # by before (WorldInfo.contactProperties reaching nothing while the
        # solver ran mu=1.0).
        #
        # MEASURED, on omniarm6_cloth_fold with 25% of the sheet overhanging a
        # table edge: at the old hardcoded 1.0 the sheet poured over the edge
        # and ended on the floor (y 0.294..0.543); raising cloth friction alone
        # kept it near the table (y 0.052..0.311). Friction is applied and it
        # is the governing parameter, so it must be the one the world states.
        #
        # Note this is only the PARTICLE side of the pair -- newton averages it
        # with each shape's own material (geometric mean for mu), so the
        # effective value still differs per surface. The env var remains the
        # override for tuning fabric independently of the rigid world.
        model.soft_contact_mu = self._cloth_env_float(
            "OMNISIM_CLOTH_SOFT_MU", float(getattr(self, "_ground_mu", 1.0) or 1.0))

        # ---- VBD CONFIGURATION ------------------------------------------
        # Shared with the whole-world VBD path; see _vbd_solver_kwargs for the
        # rationale on each entry. rigid_contact_hard=False is the COUPLED
        # value: these body-body contacts are between PROXY bodies and the real
        # rigid-rigid contact is MuJoCo's job on the other side of the
        # coupling, so hard AL duals here would fight it.
        vbd_kwargs = self._vbd_solver_kwargs(rigid_contact_hard_default=False)

        # Proxy joints: keep the robot's joints enabled inside the VBD view so
        # its proxy bodies track the MuJoCo joint targets structurally
        # (example_proxy_joint_gripper.py:136 does this for its two prismatic
        # fingers). DEFAULT OFF here: that example has 3 joints, a URDF robot
        # in this tree has 30+, and every enabled proxy joint becomes a penalty
        # constraint VBD must satisfy each iteration. The alternative --
        # example_mujoco_vbd_coupled_solver.py's articulated chain, whose
        # comment reads "The rigid solver owns the joints; VBD sees the links
        # as disjoint proxy bodies" -- syncs poses instead and is cheaper.
        # Turn this on if a precise pinch on cloth proves to need it.
        _proxy_joints = self._cloth_env_flag("OMNISIM_CLOTH_PROXY_JOINTS", False)
        # A proxy joint constrains a pair of PROXY bodies. With a narrowed
        # roster most joints have at least one endpoint that is not a proxy at
        # all, so shipping the full joint list would ask the coupler to
        # constrain bodies it was never given. Narrowing the joint list
        # correctly is a real piece of work (it needs the joint->body map and a
        # decision about half-in joints) and nothing here needs it yet, so the
        # honest move is to refuse the combination out loud rather than ship a
        # guess.
        if _proxy_joints and len(proxy_bodies) != n_bodies:
            self._newton_log(
                "cloth: OMNISIM_CLOTH_PROXY_JOINTS=1 is IGNORED because "
                "Solid.newtonClothCoupling narrowed the proxy roster to %d of %d bodies "
                "-- a proxy joint whose endpoints are not both proxies is not "
                "representable here. Widen the roster or drop the flag."
                % (len(proxy_bodies), n_bodies))
            _proxy_joints = False
        if _proxy_joints:
            vbd_kwargs["rigid_joint_linear_ke"] = self._cloth_env_float(
                "OMNISIM_CLOTH_JOINT_LINEAR_KE", 2.0e7)
            vbd_kwargs["rigid_joint_angular_ke"] = self._cloth_env_float(
                "OMNISIM_CLOTH_JOINT_ANGULAR_KE", 2.0e6)

        # ---- PURE-CLOTH WORLD (no rigid bodies at all) -------------------
        # SolverCoupledProxy.Proxy validation REQUIRES a mapping with at least
        # one body, particle or joint, and a coupled solver with nothing to
        # couple is pure overhead. A world that is only cloth runs VBD alone.
        if n_bodies == 0:
            self.solver_soft = newton.solvers.SolverVBD(model, **vbd_kwargs)
            self.solver = self.solver_soft
            self.solver_mjc = None
            self.collision_pipeline = self._build_cloth_collision_pipeline(model, "world")
            self._contacts_cache = self.collision_pipeline.contacts()
            self._newton_log(
                "cloth: %d particles, NO rigid bodies -> SolverVBD alone (no "
                "coupling). mj_model-backed features (raycast sensors, welds, "
                "TouchSensor, joint readback) are unavailable on this world."
                % len(particles))
            return

        bodies = list(range(n_bodies))
        joints = list(range(n_joints))
        # WHICH bodies the CLOTH may see. This narrows ONLY the Proxy; the
        # "mjc" entry below still owns every body and joint, which the
        # index-identity check at the tail of this function depends on.
        proxy_bodies, _coupling_mode = self._resolve_coupled_bodies(bodies)
        if len(proxy_bodies) != n_bodies:
            # Print the roster, not just its size. A cloth falling through a
            # table is diagnosed by asking whether the table is in this list,
            # and that question has to be answerable from the log alone.
            self._newton_log(
                "cloth: proxy roster NARROWED to %d of %d bodies (%s): %s. ⚠ Every body "
                "NOT in this list is invisible to the cloth -- the sheet passes straight "
                "through it. Only world/static shapes (newton body -1, which is the "
                "implicit z=0 ground plane) survive narrowing; an authored floor or table "
                "Solid is a real body and must be named to be seen."
                % (len(proxy_bodies), n_bodies, _coupling_mode,
                   proxy_bodies if len(proxy_bodies) <= 24
                   else (proxy_bodies[:24] + ["...+%d" % (len(proxy_bodies) - 24)])))

        # ---- OWNERSHIP IS A TRAP IN BOTH DIRECTIONS ----------------------
        # SolverCoupled._build_owner_map / _build_entries:
        #   * an index owned by NO entry is FROZEN in every view -- inverse
        #     mass zeroed, shapes stripped of their collide flags -- with no
        #     warning. A body left out here would simply stop moving.
        #   * but if NO entry declares ANY body, the disabling pass is skipped
        #     wholesale and EVERY entry integrates EVERY body: silent double
        #     integration, i.e. gravity applied twice.
        # Both are avoided by declaring the FULL body and joint sets on "mjc"
        # and the FULL particle set on "vbd". Assert exactly that rather than
        # trusting the ranges: `particles` is derived from the cloth range, and
        # if anything ever introduces a particle outside it (a second particle
        # source, a reordering in add_cloth_grid) that particle would be frozen
        # and the sheet would have an invisible dead patch.
        _n_particles = int(getattr(model, "particle_count", 0) or 0)
        if len(particles) != _n_particles or particles[0] != 0 or particles[-1] != _n_particles - 1:
            raise RuntimeError(
                "cloth: the vbd entry would own particles [%d,%d) of %d in the model. "
                "Every particle must be owned by exactly one entry -- an unowned "
                "particle is silently FROZEN (inv mass zeroed) with no warning."
                % (p_start, p_end, _n_particles))
        if len(bodies) != n_bodies or len(joints) != n_joints:
            raise RuntimeError("cloth: mjc entry must own every body and joint")

        # The outer pipeline and its contact buffer are built BEFORE the
        # solvers, not after: SolverVBD sizes internal state against the
        # contact layout it will be handed, and the CUDA-graph capture later in
        # step() bakes in whatever object graph exists at capture time. Order
        # is CollisionPipeline -> Contacts -> solvers, which is also the order
        # upstream's examples construct them in.
        self.collision_pipeline = self._build_cloth_collision_pipeline(model, "world")
        self._contacts_cache = self.collision_pipeline.contacts()

        # The factories run inside SolverCoupledProxy.__init__, each receiving
        # its entry's ModelView. Capture the constructed sub-solvers on the way
        # past: `self.solver.solver("mjc")` would also work (solver_coupled.py:
        # 2087) but grabbing them here means _mjc_solver() is valid even if the
        # accessor is ever renamed upstream.
        def _mjc_factory(view, _kw=dict(mjc_kwargs)):
            s = newton.solvers.SolverMuJoCo(view, **_kw)
            self.solver_mjc = s
            return s

        def _vbd_factory(view, _kw=dict(vbd_kwargs)):
            s = newton.solvers.SolverVBD(model=view, **_kw)
            self.solver_soft = s
            return s

        def _proxy_pipeline(view):
            return self._build_cloth_collision_pipeline(view, "vbd-proxy")

        # ---- SPLIT PROXY: statics heavy, dynamic links honest -------------
        # ⚠ ONE mass_scale FOR ALL PROXY BODIES IS A TRAP, measured both ways
        # on the fold demo. The scale exists because a weld-pinned STATIC gets
        # a nominal 1 kg and its pinning joint is DISABLED in the vbd view, so
        # an unscaled table is a FREE 1 kg body under 56k penalty springs --
        # the garment gets launched and the solve NaNs. But applying the same
        # 10000x to the DYNAMIC links turns a 0.02 kg jaw pad into a 200 kg
        # proxy: the cloth cannot push back on it inside the VBD solve, the
        # harvested wrench then lands on the real 0.02 kg pad in MuJoCo, and
        # the jaws oscillate and PUNCH THROUGH the fabric (measured: the pad
        # gap went NEGATIVE -20 mm mid-lift -- the "gripper glitch").
        #
        # So: statics get OMNISIM_CLOTH_PROXY_MASS_SCALE (the anti-launch fix),
        # dynamics get OMNISIM_CLOTH_PROXY_MASS_SCALE_DYNAMIC (default 1.0 --
        # honest masses, so a light gripper stays a light gripper).
        _ms_static = self._cloth_env_float("OMNISIM_CLOTH_PROXY_MASS_SCALE", 1.0)
        _ms_dynamic = self._cloth_env_float(
            "OMNISIM_CLOTH_PROXY_MASS_SCALE_DYNAMIC", 1.0)
        _static_set = set(getattr(self, "static_body_indices", ()) or ())
        _pb_static = [b for b in proxy_bodies if b in _static_set]
        _pb_dynamic = [b for b in proxy_bodies if b not in _static_set]

        def _mk_proxy(_bodies, _scale):
            return SolverCoupledProxy.Proxy(
                source="mjc",
                destination="vbd",
                # NOT `bodies` -- see _resolve_coupled_bodies. Equal to
                # `bodies` on every world that does not use
                # Solid.newtonClothCoupling.
                bodies=_bodies,
                joints=(joints if _proxy_joints else ()),
                mass_scale=_scale,
                mode=(_os.environ.get("OMNISIM_CLOTH_COUPLING_MODE") or "lagged"),
                # UNDER-RELAXATION OF THE COUPLING ITERATION. Default 1.0 is
                # newton's own value. A parallel standalone build of this exact
                # architecture measured 1.0 diverging to NaN on 3 of 15
                # identical invocations while 0.7 ran 12/12 clean; treat a
                # coupled cloth+robot run as non-deterministic on CUDA and
                # reach for this knob before blaming the grasp.
                proxy_relaxation=self._cloth_env_float(
                    "OMNISIM_CLOTH_PROXY_RELAXATION", 1.0),
                collision_pipeline=_proxy_pipeline,
                collide_interval=1,
            )

        _proxies = []
        if _pb_static and abs(_ms_static - _ms_dynamic) > 1e-12:
            _proxies.append(_mk_proxy(_pb_static, _ms_static))
            if _pb_dynamic:
                _proxies.append(_mk_proxy(_pb_dynamic, _ms_dynamic))
            self._newton_log(
                "cloth: split proxy -- %d static bodies at mass_scale %g, "
                "%d dynamic at %g" % (len(_pb_static), _ms_static,
                                      len(_pb_dynamic), _ms_dynamic))
        else:
            # No statics in the roster, or identical scales: one proxy, the
            # exact previous behaviour.
            _proxies.append(_mk_proxy(list(proxy_bodies), _ms_static))

        self.solver = SolverCoupledProxy(
            model=model,
            entries=[
                SolverCoupledProxy.Entry(
                    name="mjc",
                    solver=_mjc_factory,
                    bodies=bodies,
                    joints=joints,
                    # shapes deliberately left empty: solver_coupled.py:1057-1065
                    # then gives the entry every shape of its own bodies PLUS the
                    # world/static shapes (the ground plane), which is exactly the
                    # rigid world SolverMuJoCo would have seen uncoupled.
                ),
                SolverCoupledProxy.Entry(
                    name="vbd",
                    solver=_vbd_factory,
                    particles=particles,
                ),
            ],
            coupling=SolverCoupledProxy.Config(
                proxies=_proxies,
                iterations=self._cloth_env_int("OMNISIM_CLOTH_PROXY_ITERATIONS", 1),
            ),
        )

        # ---- THE INDEX-IDENTITY CHECK ------------------------------------
        # Everything in this file that reads mj_model / mj_data / the
        # mjc_*_to_newton_* maps (raycast, welds, TouchSensor, the pose check,
        # the contact readback) resolves through _mjc_solver() and then indexes
        # with PARENT-model ids. That is only sound while the "mjc" ModelView is
        # an identity compaction of the parent, which it is because that entry
        # owns every body. Assert it out loud rather than assume it: a future
        # entry that takes bodies away from "mjc" would shift those maps and
        # every one of those readbacks would answer about the wrong body,
        # silently.
        try:
            _view = self.solver.view("mjc")
            _vb = int(getattr(_view, "body_count", -1))
            if _vb != n_bodies:
                self._newton_log(
                    "cloth: ⚠ mjc view has %d bodies but the parent model has %d "
                    "-- the view is NOT an identity compaction, so raycast / weld / "
                    "TouchSensor readbacks that index by parent body id are now "
                    "WRONG. Do not trust them on this world." % (_vb, n_bodies))
        except Exception as _e:                   # noqa: BLE001
            self._newton_log("cloth: mjc view body-count check unavailable: %r" % (_e,))

        # (The OUTER pipeline + its Contacts were built above, before the
        # solvers. Upstream keeps both it and the per-proxy one
        # -- example_proxy_joint_gripper.py:88 and :143 -- for the same reason:
        # the coupled solver refreshes destination proxy contacts itself, the
        # outer buffer serves the substep loop and everything else.)
        self._newton_log(
            "cloth: SolverCoupledProxy up -- mjc=%d bodies/%d joints, vbd=%d "
            "particles [%d,%d), proxy_bodies=%d/%d (%s) proxy_joints=%s mode=%s "
            "iterations=%d | soft_ke=%.4g soft_kd=%.4g soft_mu=%.4g | "
            "self_contact=%s r=%.4g buf=%d"
            % (n_bodies, n_joints, len(particles), p_start, p_end,
               len(proxy_bodies), n_bodies, _coupling_mode,
               _proxy_joints, (_os.environ.get("OMNISIM_CLOTH_COUPLING_MODE") or "lagged"),
               self._cloth_env_int("OMNISIM_CLOTH_PROXY_ITERATIONS", 1),
               model.soft_contact_ke, model.soft_contact_kd, model.soft_contact_mu,
               vbd_kwargs["particle_enable_self_contact"],
               vbd_kwargs["particle_self_contact_radius"],
               vbd_kwargs["rigid_body_particle_contact_buffer_size"]))

    def _fin_mark(self, label):
        """Record a finalize phase boundary (OMNISIM_NEWTON_STEP_PROFILE=1).

        finalizeWorld costs 1.9-4.9 s -- measured up to 9.4 s under load and
        10-15 s on many-robot worlds -- INSIDE the physics bracket, and until
        2026-08-09 nobody had ever decomposed it. It dominates CI smoke,
        harness hot-reload and every short headless run, so it is a wall-clock
        lever rather than a per-step one. Marks are cheap and only recorded
        when profiling is on.
        """
        if not self._prof_on():
            return
        _t = _perf()
        marks = getattr(self, "_fin_marks", None)
        if marks is None:
            marks = self._fin_marks = []
            self._fin_t0 = _t
        marks.append((label, _t - self._fin_t0))

    def _fin_report(self):
        marks = getattr(self, "_fin_marks", None)
        if not marks:
            return ""
        out, prev = [], 0.0
        for label, t in marks:
            out.append("%s %.0fms" % (label, (t - prev) * 1000.0))
            prev = t
        return "finalize phases: " + " + ".join(out) + (" = %.0fms total" % (marks[-1][1] * 1000.0))

    def finalize(self):
        self._fin_mark("enter")
        # ---- Deferred static planes ---------------------------------
        # Added here (not at add_shape_plane time) because the choice is
        # solver-dependent: newton's MuJoCo converter raises "Planes can
        # only be attached to static bodies" for our weld-pinned statics,
        # so under a MuJoCo preference the plane is dropped -- the default
        # ground plane (a true static) already provides the z=0 floor.
        _dropped_planes = len(self._deferred_static_planes)
        if self._deferred_static_planes:
            # The solver is always MuJoCo (XPBD removed 2026-08-07), and
            # newton's MuJoCo converter raises on planes attached to our
            # weld-pinned statics -- so deferred planes are always dropped;
            # the implicit ground plane below stands in for them. (When this
            # was solver-conditional it had to MUST-MIRROR the construction
            # test, and a desync was the old G1 silent-fallback gap.) ⚠ Still
            # true and still unfixed: a RAISED or TILTED authored Plane is
            # silently replaced by the implicit z=0 plane here -- currently
            # zero live worlds hit this (audited 2026-08-07), and the campaign
            # doc records it as a latent-defect guard.
            self._deferred_static_planes = []

        # ---- The implicit ground plane ------------------------------
        # See add_ground_plane's docstring for the whole argument. In short:
        # this plane is a SUBSTITUTE for an authored `Plane` collider the
        # MuJoCo converter cannot accept, and it is added only when there was
        # one to substitute for. A world that declares no ground now has none,
        # so a body with nothing under it falls -- which is what makes
        # `run-headless --fail-on-runaway` able to see the AgentBench C2
        # fall-through case again.
        if getattr(self, "_want_ground_plane", False):
            # ⚠ NOT the module-level _os: finalize() does its own
            # `import os as _os` further down, which makes `_os` a LOCAL name
            # for this whole function and therefore UNBOUND here. Reading it
            # raised UnboundLocalError inside finalize -- measured: no
            # .newton.json sidecar, and every body frozen at its authored pose
            # for the whole run.
            import os as _gpos
            _gp_env = _gpos.environ.get("OMNISIM_NEWTON_GROUND_PLANE")
            if _gp_env is not None and _gp_env.strip() != "":
                _gp_on = _gp_env.strip().lower() not in ("0", "false", "off",
                                                         "no")
                _gp_why = "OMNISIM_NEWTON_GROUND_PLANE=%s" % _gp_env.strip()
            else:
                _gp_on = _dropped_planes > 0
                _gp_why = ("substituting for %d authored Plane collider(s) the "
                           "MuJoCo converter cannot build" % _dropped_planes
                           if _gp_on else
                           "no authored Plane collider to substitute for")
            if _gp_on:
                self.builder.add_ground_plane()
            self._newton_log(
                "[OmNewtonBackend] implicit ground plane: %s (%s)"
                % ("ADDED" if _gp_on else "not added", _gp_why))
            self._ground_plane_added = bool(_gp_on)

        # ---- Topology resolution ------------------------------------
        # Build parent->children adjacency from the queued revolutes,
        # detect bodies that aren't a revolute's child (those are
        # articulation roots), and emit joints in BFS order so every
        # joint's parent has already been added.
        children_of = {}     # parent_body -> [(slot_id, child_body)]
        all_children = set()
        for slot, j in enumerate(self.pending_revolutes):
            if j["parent"] >= 0:
                children_of.setdefault(j["parent"], []).append((slot, j["child"]))
            if j["child"] >= 0:
                all_children.add(j["child"])

        # ---- FREE joints for the roots (added FIRST so add_articulation
        # sees them at the lowest joint indices). --------------------
        # Only bodies that participate in the revolute graph at all
        # (as parent or child) need a chassis-like FREE root; standalone
        # bodies are handled in the orphan pass below.
        body_set = set(self.body_indices)
        connected = set(children_of.keys()) | all_children
        roots = sorted([b for b in body_set if b in connected and b not in all_children])
        root_static_set = set(self.static_body_indices)
        for root in roots:
            if root in root_static_set:
                # staticBase robot root (a static-base arm): anchor to the world
                # with a FIXED joint (0 DOF) instead of a 6-DOF free joint. A
                # mass=0 body with a FREE joint would inject zero-mass DOFs
                # into the dynamic webots_world articulation (singular mass
                # matrix -- the reason jointless statics are quarantined into
                # the separate "statics" articulation below). A FIXED joint
                # pins the base so its revolute children articulate off it:
                # the "bolted to the floor" staticBase semantics under Newton.
                #
                # MuJoCo refuses to compile a body in an articulation with
                # mass/inertia below mjMINVAL (even a fixed/pinned one), so the
                # mass=0 from add_static_body trips "mass and inertia of moving
                # bodies must be larger than mjMINVAL". The FIXED joint pins
                # this root regardless of its mass, so give it a nominal
                # mass+inertia purely to satisfy the compiler -- it changes no
                # dynamics under XPBD (0-DOF fixed) or MuJoCo (welded root).
                self.builder.body_mass[root] = 1.0
                self.builder.body_inv_mass[root] = 1.0
                self.builder.body_inertia[root] = wp.mat33(
                    (0.1, 0.0, 0.0), (0.0, 0.1, 0.0), (0.0, 0.0, 0.1)
                )
                # parent_xform pins the weld at the root's SPAWN world pose
                # (parent=-1 => the joint frame is in world coords), exactly like
                # the standalone static-collider weld below. WITHOUT it the fixed
                # joint defaults to the world ORIGIN, so a staticBase robot spawned
                # above z=0 (e.g. a probe pelvis at z=1.2) gets pinned to (0,0,0):
                # its links then spawn driven through the floor -> huge contact
                # constraint forces slam the joints OFF the root (the hips
                # free-swing to their stops while distal joints look fine). A
                # static-base arm hides this because it spawns near the origin AND
                # its first joint is a yaw with no gravity load. Pinning at
                # body_q[root] welds the base where the .wbt placed it.
                jf = self.builder.add_joint_fixed(
                    parent=-1, child=root,
                    parent_xform=self.builder.body_q[root])
                # body -> its world-pinning FIXED joint. set_kinematic_pose
                # uses it to refresh model.joint_X_p so newton's FK-derived
                # body_q tracks the mocap pose (readback consistency).
                if not hasattr(self, "_body_fixed_joint"):
                    self._body_fixed_joint = {}
                self._body_fixed_joint[root] = jf
            else:
                jf = self.builder.add_joint_free(child=root)
            self.joint_indices.append(jf)

        # ---- BFS push of revolutes ----------------------------------
        from collections import deque
        queue = deque(roots)
        while queue:
            parent = queue.popleft()
            # Sort children by slot id for deterministic ordering (same
            # output for identical input across runs / threads).
            for (slot, child) in sorted(children_of.get(parent, []),
                                        key=lambda sc: sc[0]):
                idx = self._add_revolute_to_builder(self.pending_revolutes[slot])
                self.slot_to_real_idx[slot] = idx
                self.joint_indices.append(idx)
                queue.append(child)

        # Defensive: any revolute we didn't reach via BFS (disconnected
        # subgraph -- shouldn't happen with a well-formed URDF, but the
        # safety net stops us silently dropping joints). Treat the
        # orphan's parent as a new root if it isn't already.
        already_rooted = set(roots)
        for slot, j in enumerate(self.pending_revolutes):
            if slot in self.slot_to_real_idx:
                continue
            p = j["parent"]
            if p not in already_rooted and p not in all_children:
                jf = self.builder.add_joint_free(child=p)
                self.joint_indices.append(jf)
                already_rooted.add(p)
            idx = self._add_revolute_to_builder(j)
            self.slot_to_real_idx[slot] = idx
            self.joint_indices.append(idx)

        # FREE joints for any body that participates in no revolute at
        # all (single-body world, free-floating debris). Without an
        # explicit FREE the solver leaves the body unreferenced. P8.2:
        # mass=0 STATIC bodies get their free joint routed into a
        # SEPARATE "statics" articulation (their zero-mass free DOFs must
        # not be integrated with the dynamic webots_world articulation,
        # per the P8.1-verified recipe); they stay pinned because the
        # solver can't accelerate a zero-mass/zero-inertia body, while
        # cross-articulation contacts against dynamic bodies still fire.
        # Newton requires each articulation's joints to be a CONTIGUOUS
        # builder-index range, so create ALL the dynamic orphan free
        # joints first (contiguous with the revolutes/root frees already
        # in self.joint_indices) and ONLY THEN the static free joints
        # (a separate contiguous block). Interleaving them trips
        # "Joints must be contiguous ... gap between 1 and 3".
        static_set = set(self.static_body_indices)
        for body_idx in self.body_indices:
            if body_idx not in connected and body_idx not in static_set:
                j = self.builder.add_joint_free(child=body_idx)
                self.joint_indices.append(j)
        # P8.2 MuJoCo fix: pin standalone static colliders (bin walls,
        # obstacles) with a FIXED joint to the world + nominal mass/inertia,
        # mirroring the staticBase robot root above -- NOT a mass=0 FREE joint.
        # MuJoCo refuses to compile a mass<mjMINVAL free-jointed ("moving")
        # body, so the old free-joint-in-a-"statics"-articulation recipe
        # silently fell back to XPBD and broke MuJoCo friction grasps. A 0-DOF
        # fixed weld pins the collider under BOTH solvers (mass is irrelevant
        # to a welded body) while cross-body contacts against dynamic bodies
        # still fire, so they live in the main webots_world articulation like
        # any other welded root.
        for body_idx in self.body_indices:
            if body_idx not in connected and body_idx in static_set:
                self.builder.body_mass[body_idx] = 1.0
                self.builder.body_inv_mass[body_idx] = 1.0
                # A welded static collider needs a nominal inertia (MuJoCo rejects
                # mass/inertia < mjMINVAL even for a fixed body). An ISOTROPIC
                # diag(0.1,0.1,0.1) is MAXIMALLY eig3-degenerate, so newton's
                # SolverMuJoCo assigns a COMPOUND (multi-shape) welded body an
                # ARBITRARY rotated body_iquat -> mj_step drops box-box contacts on
                # one body-local half (a compound static bin's +y floor then lets
                # parts fall through). For a multi-shape static body use a strictly-
                # distinct DESCENDING diagonal -> eig3 returns identity (no frame
                # tilt). A SINGLE-shape static body keeps the isotropic value (a lone
                # centered collider is contact-symmetric under any frame) -- so every
                # single-collider static (floors/walls/obstacles, the only kind in
                # non-compound worlds) is byte-identical. A static body only gets >=2
                # shapes under the compound-collider opt-in, so this is scoped to
                # compound static props exactly like the dynamic-body inertia fix.
                # Dynamically inert either way (the body is welded / never integrates).
                _n_static_shapes = 0
                for _sb in self.builder.shape_body:
                    if _sb == body_idx:
                        _n_static_shapes += 1
                if _n_static_shapes >= 2:
                    self.builder.body_inertia[body_idx] = wp.mat33(
                        (0.11, 0.0, 0.0), (0.0, 0.10, 0.0), (0.0, 0.0, 0.09)
                    )
                else:
                    self.builder.body_inertia[body_idx] = wp.mat33(
                        (0.1, 0.0, 0.0), (0.0, 0.1, 0.0), (0.0, 0.0, 0.1)
                    )
                # parent_xform pins the weld at the collider's SPAWN world pose
                # (parent=-1 => the joint frame is in world coordinates).
                # Without it the fixed joint defaults to the world origin and
                # the collider collapses to (0,0,0) instead of staying where
                # the .wbt placed it.
                j = self.builder.add_joint_fixed(
                    parent=-1, child=body_idx,
                    parent_xform=self.builder.body_q[body_idx])
                # body -> pinning joint, for set_kinematic_pose's joint_X_p
                # refresh (see the roots loop above).
                if not hasattr(self, "_body_fixed_joint"):
                    self._body_fixed_joint = {}
                self._body_fixed_joint[body_idx] = j
                self.joint_indices.append(j)

        # ---- Webots selfCollision=FALSE semantics ---------------------
        # ODE/Webots never collides a robot with itself unless the Robot
        # node's selfCollision field is TRUE (default FALSE) -- every
        # controller/policy in this repo was built on that behaviour.
        # Newton has no such notion: once W1 registered URDF collision
        # MESHES natively, MuJoCo's own collision saw e.g. Spot's chassis
        # hull overlapping all four upper-leg hulls AT THE STANDING POSE
        # (mjcontacts at step 1: chassis-vs-upper x4) and the constant
        # internal wrench shoved the robot backward and tipped it in ~1.2 s
        # -- the W6 "Spot deploy collapse". (The GPU trainer hit the SAME
        # geometry on its raw MJCF export and fixed it by zeroing
        # contype/conaffinity on the upper geoms; this is the bridge-side
        # equivalent.) Filter every intra-robot shape pair, where a robot =
        # a connected component of the articulated-joint graph. The pairs
        # go into builder.shape_collision_filter_pairs, which BOTH contact
        # paths honor (newton's broad-phase directly; SolverMuJoCo via its
        # graph-coloring into contype/conaffinity). Robot-vs-world,
        # robot-vs-free-body and robot-vs-robot contacts are untouched.
        # Opt back into self collisions with OMNISIM_NEWTON_SELF_COLLISION=1.
        import os as _scos
        if not _scos.environ.get("OMNISIM_NEWTON_SELF_COLLISION"):
            _adj = {}
            for _scj in self.pending_revolutes:
                _scp, _scc = _scj["parent"], _scj["child"]
                if _scp >= 0 and _scc >= 0:
                    _adj.setdefault(_scp, set()).add(_scc)
                    _adj.setdefault(_scc, set()).add(_scp)
            _seen = set()
            _n_filtered = 0
            _shape_body = self.builder.shape_body
            _filter_pairs = self.builder.shape_collision_filter_pairs
            _have = set(_filter_pairs)
            for _start in sorted(_adj.keys()):
                if _start in _seen:
                    continue
                _comp = set()
                _stack = [_start]
                while _stack:
                    _x = _stack.pop()
                    if _x in _comp:
                        continue
                    _comp.add(_x)
                    _stack.extend(_adj.get(_x, ()))
                _seen |= _comp
                if len(_comp) < 2:
                    continue
                _shapes = [_si for _si, _sb in enumerate(_shape_body)
                           if _sb in _comp]
                for _ii in range(len(_shapes)):
                    for _kk in range(_ii + 1, len(_shapes)):
                        _pr = (_shapes[_ii], _shapes[_kk])
                        if _pr not in _have:
                            _filter_pairs.append(_pr)
                            _have.add(_pr)
                            _n_filtered += 1
            if _n_filtered:
                try:
                    _lp = _scos.environ.get("OMNISIM_NEWTON_LOG")
                    if _lp:
                        with open(_lp, "a") as _lf:
                            _lf.write("selfcollision-filter: %d intra-robot "
                                      "shape pairs excluded\n" % _n_filtered)
                except Exception:
                    pass

        self._fin_mark("topology")
        # newton raises "Cannot create an articulation with no joints" on an
        # empty list (builder.py:3062). Every world with a body has at least the
        # free joint add_body() emits, so this guard is unreachable for them and
        # cannot change what they build -- it exists for a world that is ONLY
        # cloth, where there is no rigid body to carry a joint at all and the
        # unguarded call is a hard crash in finalize().
        if self.joint_indices:
            self.builder.add_articulation(self.joint_indices, label="webots_world")
        self._fin_mark("add_articulation")

        # ---- CLOTH: GRAPH COLOURING (REQUIRED, AND ONLY WHEN CLOTH EXISTS) --
        # SolverVBD is a vertex-block-descent solver: it sweeps INDEPENDENT
        # sets of vertices in parallel, and those sets come from a graph
        # colouring of the cloth mesh. newton's own docstring for
        # ModelBuilder.color() (builder.py:10145) is explicit that finalize()
        # does NOT colour implicitly, so without this call SolverVBD gets empty
        # `particle_color_groups` and the cloth does not move.
        #
        # include_bending=True because add_cloth_grid() adds BENDING EDGES
        # (builder.py:10151: set it True "if your model contains bending edges
        # (added via add_edge) that participate in bending constraints"), which
        # every cloth grid does -- they are what makes a sheet resist folding.
        # Colouring without them lets two vertices that share a bending
        # constraint land in the same colour and be relaxed simultaneously,
        # which is the classic VBD race: it does not crash, it just makes the
        # bending term wrong. Upstream calls plain `builder.color()` in
        # example_proxy_joint_gripper.py:77 because a SOFT (tetrahedral) grid
        # has no bending edges; ours is a CLOTH grid, so ours needs the flag.
        #
        # ⚠ GATED ON has_cloth(). color() walks the whole particle/edge graph
        # and is not free; more importantly it also populates
        # `body_color_groups`, so calling it unconditionally would change what
        # finalize() hands every rigid-only world. It stays off unless cloth
        # was authored.
        # ⚠ ALSO REQUIRED BY newtonSolver "vbd" EVEN WITH NO CLOTH AT ALL, and
        # it is a hard failure rather than a degradation: SolverVBD raises
        # ValueError("model.body_color_groups is empty but rigid bodies are
        # present! ... you must call ModelBuilder.color() ... before calling
        # ModelBuilder.finalize()") at solver_vbd.py:850-857. color() populates
        # body_color_groups as well as the particle graph, so the whole-world
        # VBD path needs it for its BODIES, not for a sheet.
        if self.has_cloth() or self._vbd_world():
            _t_color = _perf()
            self.builder.color(include_bending=True)
            self._newton_log("%s: builder.color(include_bending=True) over %d "
                             "particles / %d bodies in %.0f ms"
                             % ("cloth" if self.has_cloth() else "vbd-world",
                                len(self.builder.particle_q),
                                int(getattr(self.builder, "body_count", 0) or 0),
                                (_perf() - _t_color) * 1000.0))
            self._fin_mark("cloth_color")

        # DEVICE SELECTION (physics-step-cost-optimization-plan.md).
        # warp defaults to cuda:0 when a GPU exists, so on the CPU
        # SolverMuJoCo path the newton Model/State arrays used to live on the
        # GPU while mj_step ran on the CPU -- every tick round-tripped state
        # across PCIe for a simulation that never touched the GPU. Measured
        # on this machine (RTX 3060 laptop, warp 1.13): .numpy() readback
        # 0.0538 ms on cuda vs 0.0020 on cpu (27x), wp.array create 0.0444
        # vs 0.0158, wp.launch 0.0184 vs 0.0074 -- and the tick makes about
        # a dozen such calls (joint clamp 2, base guard 2 + copies,
        # _update_newton_state 2 launches + 2 creates, readback caches).
        # Pin the model (and hence state/control, which allocate on
        # model.device) to CPU whenever the solver is CPU mj_step; the
        # mujoco_warp path must stay on the GPU and is untouched.
        # OMNISIM_NEWTON_MODEL_DEVICE overrides ("cpu" / "cuda" / "auto").
        # Same predicate the solver construction below uses for
        # use_mujoco_cpu, evaluated early (finalize precedes it).
        import os as _devos  # NOT _os: this function rebinds _os later, which
                             # would make it a local and NameError here.
        _dev_env = (_devos.environ.get("OMNISIM_NEWTON_MODEL_DEVICE") or "").strip().lower()
        _wants_warp = (bool(_devos.environ.get("OMNISIM_NEWTON_MJWARP"))
                       or getattr(self, "_solver_pref", None) == "mujoco_warp")
        _want_cpu = (_dev_env == "cpu") or (_dev_env in ("", "auto") and not _wants_warp)
        # ⚠ CLOTH OVERRIDES THE CPU PIN. The pin above exists because the CPU
        # mj_step path never touches the GPU, so keeping the model there saved
        # 2.1-3.6x of PCIe round trips. SolverVBD is the opposite case: it is a
        # warp solver and its per-particle Gauss-Seidel sweeps are the whole
        # cost. MEASURED at 289 particles: CPU 6.7 fps vs CUDA + CUDA-graph
        # 164 fps -- a 24x swing that dwarfs the readback saving the pin buys
        # back, and 6.7 fps is not a simulation anyone can drive.
        #
        # So a world carrying cloth finalizes on CUDA even though its robot
        # still steps through the CPU mj_step. That is a DELIBERATE inversion
        # of the rule two paragraphs up, it is the only place in this file
        # where the two solvers want different devices, and it is logged with
        # its reason so a later step-cost campaign does not "fix" it back.
        # An EXPLICIT OMNISIM_NEWTON_MODEL_DEVICE=cpu still wins (that is what
        # an override is for) -- but it is the slow configuration, and it says
        # so in the log.
        #
        # The same reasoning covers newtonSolver "vbd" with no cloth in it: the
        # solver is still SolverVBD, still a warp solver, and there is no CPU
        # mj_step on that path for the pin to be saving anything for.
        _cloth_dev_note = ""
        if self.has_cloth() or self._vbd_world():
            _who = "cloth" if self.has_cloth() else "newtonSolver \"vbd\""
            if _dev_env == "cpu":
                _cloth_dev_note = (" (%s wanted cuda; OMNISIM_NEWTON_MODEL_DEVICE=cpu "
                                   "overrides -- expect ~6.7 fps at a few hundred "
                                   "particles)" % _who)
            elif _want_cpu:
                _want_cpu = False
                _cloth_dev_note = " (forced to cuda by %s: SolverVBD is a warp solver)" % _who
        if _want_cpu:
            self.model = self.builder.finalize(device="cpu")
        else:
            self.model = self.builder.finalize()
        self._fin_mark("builder.finalize")

        # --- Per-BODY particle-contact friction override --------------------
        # OMNISIM_CLOTH_BODY_MU="<newtonBodyIdx>:<mu>,..."  e.g. "6:64,7:64"
        #
        # WHY THIS EXISTS. A cloth pinch needs the gripper PADS to be stickier
        # than the TABLE, and the only friction lever the runtime exposed was
        # global: soft_contact_mu applies to every particle-rigid contact at
        # once. Measured on the fold demo, that global-ness is exactly the
        # failure: mu 1 holds a clean 22 mm pinch at close and extrudes it
        # during the lift; mu 2 glues the garment to the table too, so the
        # closing jaws must drag fabric across a sticky table and get PRIED
        # OPEN (131 mm gap excess); mu 1000 (newton's own cloth-lift value)
        # NaNs outright.
        #
        # newton already supports the per-shape half of the answer: VBD's
        # contact-material prep mixes per contact as
        #     mu = sqrt(soft_contact_mu * shape_material_mu[shape])
        # (rigid_vbd_kernels.py, _average_contact_material -- geometric mean),
        # so writing shape_material_mu on JUST the pad shapes makes the pads
        # sticky while the table keeps its own value. With soft_contact_mu 1
        # and a pad override of 64, the pad contacts run at mu 8 and the table
        # stays at 1.
        #
        # Keyed by newton BODY index (model.shape_body maps shapes to bodies)
        # because body NAMES never reach this runtime -- the C++ side logs
        # them but registers bodies by index only. ⚠ That makes the value
        # WORLD-SPECIFIC and fragile against world edits; the proper form is a
        # per-Solid schema field (newtonClothMu) plumbed from OmSolid, which
        # is the documented follow-up. Env > nothing, for now.
        _body_mu_spec = __import__("os").environ.get(
            "OMNISIM_CLOTH_BODY_MU", "").strip()
        if _body_mu_spec and self.model is not None:
            try:
                _overrides = {}
                for _tok in _body_mu_spec.split(","):
                    _b, _v = _tok.split(":")
                    _overrides[int(_b)] = float(_v)
                _sb = self.model.shape_body.numpy()
                _mu = self.model.shape_material_mu.numpy()
                _hit = []
                for _si in range(len(_sb)):
                    _bidx = int(_sb[_si])
                    if _bidx in _overrides:
                        _mu[_si] = _overrides[_bidx]
                        _hit.append("shape %d (body %d) -> mu %g"
                                    % (_si, _bidx, _overrides[_bidx]))
                if _hit:
                    self.model.shape_material_mu.assign(_mu)
                    self._newton_log(
                        "cloth: OMNISIM_CLOTH_BODY_MU overrode %d shape(s): %s"
                        % (len(_hit), "; ".join(_hit)))
                else:
                    self._newton_log(
                        "cloth: OMNISIM_CLOTH_BODY_MU matched NO shapes "
                        "(bodies asked: %s) -- check the body indices against "
                        "the step log's b<N> roster" % sorted(_overrides))
            except Exception as _bm_e:                    # noqa: BLE001
                self._newton_log(
                    "cloth: OMNISIM_CLOTH_BODY_MU ignored (%r)" % (_bm_e,))
        if self.has_cloth() or self._vbd_world():
            self._newton_log("%s: model device=%s%s"
                             % ("cloth" if self.has_cloth() else "vbd-world",
                                self.model.device, _cloth_dev_note))
        try:
            _lp = _devos.environ.get("OMNISIM_NEWTON_LOG")
            if _lp:
                with open(_lp, "a") as _lf:
                    _lf.write("model device: %s\n" % (self.model.device,))
        except Exception:
            pass

        # One-shot per-joint diagnostic (gated by env so it's normally
        # inert): which revolutes ended up motorized vs free-spinning, the
        # slot->real mapping, and each joint's seeded angle. This is how we
        # find a leg whose motor wasn't seen at flush time (target_kd=0 ->
        # passive joint that sags/collapses under load).
        import os as _dos
        _diag_path = _dos.environ.get("OMNISIM_NEWTON_JOINTDIAG")
        if _diag_path:
            try:
                q_start = self.model.joint_q_start.numpy()
                jq = self.model.joint_q.numpy()
                # Per-DOF actuator gains MuJoCo will use: gainprm=[ke] per
                # joint. A zero ke on one joint => its position actuator
                # has no drive (the suspected per-joint bug).
                _qd_start = self.model.joint_qd_start.numpy()
                try:
                    _ke = self.model.joint_target_ke.numpy()
                    _kd_m = self.model.joint_target_kd.numpy()
                    _mode = self.model.joint_target_mode.numpy()
                except Exception:
                    _ke = _kd_m = _mode = None
                lines = []
                for slot, j in enumerate(self.pending_revolutes):
                    real = self.slot_to_real_idx.get(slot)
                    ang = None
                    mke = mkd = mmode = None
                    if real is not None and real < len(q_start):
                        qi = int(q_start[real])
                        if qi < len(jq):
                            ang = round(float(jq[qi]), 4)
                        if _ke is not None and real < len(_qd_start):
                            _di = int(_qd_start[real])
                            if _di < len(_ke):
                                mke = round(float(_ke[_di]), 1)
                                mkd = round(float(_kd_m[_di]), 1)
                                mmode = int(_mode[_di])
                    _ax = tuple(round(float(v), 3) for v in j["axis"])
                    _pa = tuple(round(float(v), 3) for v in j["p_anchor"])
                    _ca = tuple(round(float(v), 3) for v in j["c_anchor"])
                    lines.append(
                        f"slot={slot} real_idx={real} parent={j['parent']} child={j['child']} "
                        f"motor={j.get('_is_motor')} seed_angle={ang} "
                        f"model_ke={mke} model_kd={mkd} mode={mmode} "
                        # multi-DoF (ball/hinge2) and 0-DoF (fixed) specs carry per-axis
                        # tuples, not these scalars -- the diagnostic used to KeyError on
                        # exactly the joint types it was needed for (same defect class as
                        # the post-step clamp). Report what the spec actually has.
                        f"eff={j.get('effort_limit', j.get('efforts', '-'))} "
                        f"lim=[{j.get('limit_lower', '-')},{j.get('limit_upper', '-')}] "
                        f"kind={j.get('kind', 'revolute')} motorized={bool(j.get('motorized'))} "
                        f"gains={j.get('gains', '-')} limits={j.get('limits', '-')} "
                        f"axis={_ax} p_anchor={_pa} c_anchor={_ca}")
                with open(_diag_path, "w") as _f:
                    _f.write(f"n_revolutes={len(self.pending_revolutes)} "
                             f"n_bodies={len(self.body_indices)}\n")
                    _f.write("\n".join(lines) + "\n")
            except Exception as _e:
                try:
                    with open(_diag_path, "w") as _f:
                        _f.write(f"jointdiag failed: {_e!r}\n")
                except Exception:
                    pass
        # SOLVER: SolverMuJoCo, unconditionally. XPBD was REMOVED 2026-08-07 --
        # after the default flip (7b431e81) nothing selected it and nothing in
        # OmniSim needs what it is for (cloth / soft bodies / particles; the
        # CUDA granular path couples to ODE, not Newton). The full record is
        # docs/developer/ode-retirement-campaign.md #0.5; the short version:
        # newton's own docs say XPBD does not operate on articulations, zero
        # of 725 worlds ever asked for it, it measured SLOWER than CPU mujoco
        # at every scale tried, and on the shipped 10-Husky swarm it drove the
        # robots 0.97 m where mujoco and ODE agree on 2.4 m.
        _solver_kind = "unknown"
        _solver_error = None
        import os as _os
        # _pref selects CPU vs warp WITHIN MuJoCo ("mujoco_warp" or
        # OMNISIM_NEWTON_MJWARP=1 for the GPU path; anything else = CPU
        # mj_step, the deterministic reference).
        _pref = getattr(self, "_solver_pref", None)
        # "mujoco"      -> SolverMuJoCo on the reference CPU mj_step (deterministic).
        # "mujoco_warp" -> SolverMuJoCo on the GPU mujoco_warp backend. Identical
        #   physics setup; the GPU path only pays off in FAST mode at a large
        #   basicTimeStep with an EVEN substep count (the CUDA-graph replay in
        #   step() needs even substeps -- an odd count re-issues ~800 kernel
        #   launches/tick and runs SLOWER than CPU). Requires a CUDA GPU + warp;
        #   falls back to CPU mj_step construction if warp can't init.
        # Pinned True since XPBD's removal -- kept as a name only so the block
        # below keeps its indentation (it was `if _force_mujoco:` when there
        # was an else to take).
        # ...and now there IS an else again, for exactly one value:
        # newtonSolver "vbd" replaces SolverMuJoCo outright with a single
        # SolverVBD that owns rigid bodies AND particles. Every other value
        # (including "" / "auto" / "mujoco" / "mujoco_warp" / "mujoco+vbd")
        # leaves _force_mujoco True and takes the untouched block below, so a
        # world that does not name "vbd" is bit-identical.
        _force_mujoco = not self._vbd_world()
        if not _force_mujoco:
            try:
                self._build_vbd_world_solver()
                self._fin_mark("SolverVBD")
                _solver_kind = ("VBD (whole world: rigid + particles, "
                                "WorldInfo.newtonSolver \"vbd\")")
            except Exception as _ve:
                # No fallback. A world that asked for one solver and silently
                # got another is the exact failure this file spent the XPBD
                # removal eliminating -- and here the two differ in what a
                # grasp can even do, not just in speed.
                _solver_error = _ve
                self._newton_log(
                    "newtonSolver \"vbd\": SolverVBD construction FAILED (%r). This "
                    "world has NO physics. Remove the newtonSolver \"vbd\" pin to fall "
                    "back to the MuJoCo path deliberately -- it is not done for you, "
                    "because the two solvers are not interchangeable." % (_ve,))
                raise
        # One-shot MJCF export: OMNISIM_NEWTON_SAVE_MJCF=<path> dumps the
        # EXACT MuJoCo model Newton built (inertias, ke/kd servos, friction,
        # joint limits) so a GPU-batched mujoco_warp trainer can load the
        # identical physics -- zero sim-to-sim gap with this deploy backend.
        _save_mjcf = _os.environ.get("OMNISIM_NEWTON_SAVE_MJCF")
        if _force_mujoco:
            # use_mujoco_cpu=True -> reference MuJoCo mj_step (C lib);
            # =False -> mujoco_warp. These are DIFFERENT implementations and
            # a policy tuned to one can be unstable in the other. The GPU
            # trainer runs mujoco_warp, so OMNISIM_NEWTON_MJWARP=1 makes the
            # deploy use mujoco_warp too -> same engine -> policy transfers.
            _use_cpu = not (bool(_os.environ.get("OMNISIM_NEWTON_MJWARP")) or _pref == "mujoco_warp")
            # ⚠ CLOTH MOVES THE MUJOCO ENTRY TO THE GPU TOO -- and this is a
            # PHYSICS CHANGE on coupled cloth worlds, deliberately taken.
            #
            # The CPU-mj_step default is correct for a rigid world because the
            # model is pinned to the CPU device, so mj_step touches no GPU and
            # nworld=1 mujoco_warp would only add launch overhead. On a CLOTH
            # world that premise is already false: the block above FORCES the
            # model to cuda:0 for SolverVBD's sake, so a CPU mj_step entry is
            # the one configuration that gets the worst of both devices --
            #   1. it copies state GPU->host->GPU EVERY SUBSTEP, and
            #   2. that d2h memcpy cannot be recorded into a CUDA graph, so
            #      _cloth_graph_ok() must refuse the capture for the WHOLE tick,
            #      including the VBD kernels that have nothing to do with mj.
            # (2) is the expensive half and it is not obvious: MEASURED on the
            # 289-particle drape, the CPU mj_step itself costs 0.84 ms of a
            # 29.29 ms step, but the capture it blocks is worth ~10x on the
            # whole tick, because the cloth path is LAUNCH-bound -- step cost is
            # flat from 81 to 2401 particles and scales with VBD ITERATION count
            # (~2.42 ms/iteration ungraphed vs ~0.18 ms graphed).
            #
            # MEASURED end to end, newton_cloth_drape, engine-level steady state
            # (machine 9722d23d12a3, RTX 3060 laptop, newton 1.5.0):
            #   CPU mj entry  28.4 ms/step = 0.28x realtime
            #   GPU mj entry   2.35 ms/step = 3.40x realtime   (12.1x, crosses 1.0x)
            #
            # WHAT IT COSTS. mujoco_warp and mj_step are different
            # implementations, so rigid trajectories in a coupled cloth world
            # shift, and bitwise determinism is REFUTED on mujoco_warp (it holds
            # only on CPU mj_step) -- see docs/benchmarks/determinism-scope.md.
            # Scoped as narrowly as the win allows: ONLY when cloth has already
            # forced the model to CUDA. A rigid world, a CPU-pinned world and a
            # newtonSolver "vbd" world (no mjc entry at all) are all untouched,
            # which is why the 8-Husky 56.579 m datum is byte-identical.
            # OMNISIM_NEWTON_CLOTH_CPU_MJ=1 restores the CPU entry exactly.
            if (_use_cpu and self.has_cloth()
                    and "cuda" in str(self.model.device).lower()
                    and not _os.environ.get("OMNISIM_NEWTON_CLOTH_CPU_MJ")):
                _use_cpu = False
                self._newton_log(
                    "cloth: MuJoCo entry moved to mujoco_warp because the model is "
                    "on %s (cloth forced it there). A CPU mj_step entry would copy "
                    "state GPU<->host every substep AND block the CUDA-graph capture "
                    "for the whole tick -- measured 28.4 -> 2.35 ms/step, 0.28x -> "
                    "3.40x realtime on newton_cloth_drape. WARNING: this CHANGES the "
                    "rigid physics (mujoco_warp != mj_step) and forfeits bitwise "
                    "determinism. OMNISIM_NEWTON_CLOTH_CPU_MJ=1 restores the CPU "
                    "entry." % (self.model.device,))
            _kw = {"use_mujoco_cpu": _use_cpu}
            # Contact-stability knobs for DENSE manipulation (env-tunable; unset
            # -> MuJoCo defaults = exact current physics). MuJoCo recommends a
            # HIGH impratio + ELLIPTIC cone + more iterations for grasping /
            # stacking so dense contacts don't slip and launch -- the tiltable-
            # bin cube-ejection failure mode. See the SolverMuJoCo docstring.
            # Env var wins; else the per-world WorldInfo.newtonImpratio /
            # newtonCone (plumbed via set_contact_cone); else MuJoCo stock.
            _impr = _os.environ.get("OMNISIM_NEWTON_IMPRATIO")
            if _impr:
                _kw["impratio"] = float(_impr)
            elif getattr(self, "_impratio_world", 0.0) > 0.0:
                _kw["impratio"] = float(self._impratio_world)
            _cone = _os.environ.get("OMNISIM_NEWTON_CONE")
            if _cone:
                _kw["cone"] = _cone
            elif getattr(self, "_cone_world", None):
                _kw["cone"] = self._cone_world
            # env var > WorldInfo.newtonIterations / newtonLsIterations > the
            # solver's own default. 0 on the field means "not declared", so an
            # existing world is byte-identical.
            _iters = _os.environ.get("OMNISIM_NEWTON_ITERS")
            if _iters:
                _kw["iterations"] = int(_iters)
            elif int(getattr(self, "_iters_world", 0)) > 0:
                _kw["iterations"] = int(self._iters_world)
            _lsiters = _os.environ.get("OMNISIM_NEWTON_LS_ITERS")
            if _lsiters:
                _kw["ls_iterations"] = int(_lsiters)
            elif int(getattr(self, "_ls_iters_world", 0)) > 0:
                _kw["ls_iterations"] = int(self._ls_iters_world)
            # Constraint buffer caps. mujoco_warp auto-estimates these too
            # small for hard multi-contact footstrikes (the G1 trainer hit
            # the same "nefc overflow" -> dropped constraints -> foot
            # penetration -> mid-walk explosion; it pins njmax/nconmax=256).
            # Same 256 default here -- but 256 is ALSO too small the other way
            # round, for a multi-robot fleet: MEASURED 32 constraint rows per
            # 4WD Husky (8 wheel-ground contacts x 4 rows each on a pyramidal
            # cone), so 10 robots peak at nefc=320 and the kernel per-tick-
            # printfs "nefc overflow" while silently truncating the vector.
            # Precedence (matches the cone knobs): env var, else the per-world
            # WorldInfo.newtonNjmax / newtonNconmax (plumbed via
            # set_constraint_buffers), else 256. <=0 (world field -1, or the
            # env var set to 0) restores newton's own auto-estimate.
            _njmax_env = _os.environ.get("OMNISIM_NEWTON_NJMAX")
            _nconmax_env = _os.environ.get("OMNISIM_NEWTON_NCONMAX")
            if _njmax_env is not None:
                _njmax = int(_njmax_env)
            elif int(getattr(self, "_njmax_world", 0)) != 0:
                _njmax = int(self._njmax_world)
            else:
                _njmax = 256
            if _nconmax_env is not None:
                _nconmax = int(_nconmax_env)
            elif int(getattr(self, "_nconmax_world", 0)) != 0:
                _nconmax = int(self._nconmax_world)
            else:
                _nconmax = 256
            if _njmax > 0:
                _kw["njmax"] = _njmax
            if _nconmax > 0:
                _kw["nconmax"] = _nconmax
            # Remembered for the constraint-buffer telemetry below (0 = auto).
            self._njmax_applied = _njmax if _njmax > 0 else 0
            self._nconmax_applied = _nconmax if _nconmax > 0 else 0
            if _save_mjcf:
                _kw["save_to_mjcf"] = _save_mjcf
            try:
                if self.has_cloth():
                    # CLOTH PATH: the same SolverMuJoCo, built with the same
                    # **_kw, wrapped in a SolverCoupledProxy alongside a
                    # SolverVBD that owns the particles. See
                    # _build_cloth_coupled_solver for the full rationale.
                    self._build_cloth_coupled_solver(_kw)
                else:
                    self.solver = newton.solvers.SolverMuJoCo(self.model, **_kw)
                self._fin_mark("SolverMuJoCo")
                self._apply_cpu_bvh_workaround()
                # Provenance for the sidecar/log: name the ACTUAL reason the
                # solver was chosen. "default" is the honest label for an
                # unpinned world since the 2026-08-07 flip -- the old code
                # attributed every non-pinned selection to FORCE_MUJOCO=1,
                # which after the flip would claim an env var nobody set.
                if _pref in ("mujoco", "mujoco_warp"):
                    _why = "WorldInfo.newtonSolver"
                elif _os.environ.get("OMNISIM_NEWTON_FORCE_MUJOCO", "").strip().lower() not in ("", "0", "false", "off", "no"):
                    _why = "FORCE_MUJOCO=1"
                else:
                    _why = "default"
                # Read the device branch back OFF THE SOLVER rather than from
                # the local `_use_cpu`. They are the same value on every rigid
                # world (it is the kwarg we just passed), but the cloth path can
                # OVERRIDE it -- OMNISIM_CLOTH_NEWTON_CONTACTS=1 forces
                # mujoco_warp because use_mujoco_contacts=False does not
                # function on the CPU branch. Reporting the requested value
                # instead of the effective one would put "cpu/mj_step" in the
                # log AND in the .newton.json verdict sidecar for a run that
                # was actually GPU -- and the sidecar exists precisely so a
                # measurement can say what produced it.
                _eff_cpu = bool(getattr(self._mjc_solver(), "use_mujoco_cpu", _use_cpu))
                _solver_kind = ("MuJoCo (%s, %s)" %
                                ("cpu/mj_step" if _eff_cpu else "mujoco_warp", _why))
                if self.has_cloth():
                    _solver_kind += (" + VBD cloth via SolverCoupledProxy (%s contacts)"
                                     % ("newton" if _os.environ.get("OMNISIM_CLOTH_NEWTON_CONTACTS",
                                                                    "").strip().lower()
                                        not in ("", "0", "false", "off", "no") else "mujoco"))
            except Exception as _me:
                # SolverMuJoCo construction failed. This used to fall back to
                # SolverXPBD "so the sim still runs instead of crashing" --
                # which is a world simulated by a solver nobody chose, with
                # different contact physics, under a log saying MuJoCo was
                # wanted. XPBD is REMOVED (2026-08-07), and per the same rule
                # that retired the silent ODE fall-back (85fa6bde): a wrong
                # result is worse than a lost one. Log the full construction
                # error and REFUSE.
                import traceback as _mtb
                raise RuntimeError(
                    "SolverMuJoCo construction failed and there is no other "
                    "solver to substitute (XPBD removed 2026-08-07, ODE "
                    "deleted 2026-08-08): %r -- there is NO backend left to "
                    "pin to, so the WORLD has to be fixed. || %s"
                    % (_me, _mtb.format_exc()[-1400:]))
        # Cross-tree manipulation scenes must choose the MuJoCo solve topology BEFORE the first
        # step.  Applying mjDSBL_ISLAND later from a controller is too late: a free prop resting
        # on a separate static support can enter the island path during startup and poison the
        # articulated state before controller tick 0 (observed as finite step 30 -> all-NaN step
        # 60 in g1_box_grasp).  Opt-in keeps every existing world byte-identical.
        if _force_mujoco and _os.environ.get("OMNISIM_NEWTON_DISABLE_ISLAND"):
            import mujoco as _mjis
            _island_models = (getattr(self.solver, "mjw_model", None),
                              getattr(self.solver, "mj_model", None))
            _island_applied = 0
            for _ism in _island_models:
                if _ism is not None:
                    _ism.opt.disableflags = (int(_ism.opt.disableflags)
                                             | int(_mjis.mjtDisableBit.mjDSBL_ISLAND))
                    _island_applied += 1
            if not _island_applied:
                raise RuntimeError("OMNISIM_NEWTON_DISABLE_ISLAND=1 but SolverMuJoCo exposed no model")
            _solver_kind += ", islands=off"
        # PER-WORLD CONTACT DIMENSIONALITY (WorldInfo.newtonCondim, or
        # OMNISIM_NEWTON_CONDIM=<n> which wins). 0 = unset -> leave the model
        # exactly as newton built it, which is condim 3 on every geom of every
        # OmniSim world measured to date. 1 = frictionless, 3 = sliding only,
        # 4 = + torsional, 6 = + rolling.
        #
        # ⚠ WHY THIS HAS TO PATCH mj_model DIRECTLY. newton DOES expose a
        # per-shape `mujoco:condim` custom attribute, but only via
        # SolverMuJoCo.register_custom_attributes(builder), which OmniSim never
        # calls -- and on the CPU mj_step path a post-finalize per-shape write
        # would not reach mj_model anyway. Patching geom_condim after
        # construction is the same proven route OMNISIM_NEWTON_ROLL_MU below
        # already takes. Both mj_model (CPU) and mjw_model (GPU) are written so
        # the knob means the same thing on either path.
        try:
            import os as _cdo
            _cdv = _cdo.environ.get("OMNISIM_NEWTON_CONDIM")
            _cdn = int(_cdv) if _cdv not in (None, "") else int(getattr(self, "_condim_world", 0) or 0)
            if _cdn > 0:
                import numpy as _cdnp
                _cdm = getattr(self.solver, "mj_model", None)
                if _cdm is not None:
                    _cdm.geom_condim[:] = _cdn
                    if _cdn >= 4:
                        # condim 4/6 CONSULT geom_friction[1] (torsional) and
                        # [2] (rolling); newton leaves them at 0.005 / 0.0. A
                        # zero rolling coefficient is fine (condim 6 then just
                        # adds nothing), but a zero torsional one would make
                        # condim 4 a no-op, so floor it the way ROLL_MU does.
                        _cdm.geom_friction[:, 1] = _cdnp.maximum(_cdm.geom_friction[:, 1], 0.005)
                _cdw = getattr(self.solver, "mjw_model", None)
                if _cdw is not None:
                    try:
                        _cdc = _cdw.geom_condim.numpy()
                        _cdc[:] = _cdn
                        _cdw.geom_condim.assign(_cdc)
                        if _cdn >= 4:
                            _cdf = _cdw.geom_friction.numpy()
                            _cdf[:, 1] = _cdnp.maximum(_cdf[:, 1], 0.005)
                            _cdw.geom_friction.assign(_cdf)
                    except Exception:
                        pass
                _solver_kind += " +condim=%d" % _cdn
        except Exception as _cde:
            _solver_kind += " condim_FAILED:" + repr(_cde)[:140]
        # PER-WORLD NOSLIP PASS (WorldInfo.newtonNoslipIterations, or
        # OMNISIM_NEWTON_NOSLIP=<n> which wins and is VALUE-parsed so =0 is the
        # exact-revert hatch). 0 = unset == MuJoCo's own stock 0, so every
        # existing world is byte-identical.
        #
        # WHAT IT FIXES, and why nothing else does. MuJoCo solves contact as a
        # SOFT constraint, and a tangential one drifts under a sustained load
        # even while the normal force sits exactly at its commanded value --
        # so a two-finger pinch squeezing far above the Coulomb bound still
        # watches the part creep out a millimetre at a time. More force, a
        # firmer contact and more main-solver iterations do not address it,
        # because it is not an insufficiency, it is residual drift.
        # noslip_iterations runs a friction-only Gauss-Seidel pass AFTER the
        # main solve to remove exactly that. MEASURED on ladder0 rung 8 (0.2 kg
        # part, 3 N/pad, mu=3 = 9x the Coulomb bound): 0 creeps 56 mm during a
        # 1.5 s lift and drops the part; >=1 carries it, and the answer is
        # identical from 3 upward -- a solver budget, not a tuned value.
        #
        # ⚠ WHY IT IS PATCHED HERE RATHER THAN PASSED TO SolverMuJoCo. There is
        # no kwarg for it, and SolverMuJoCo.__init__ calls mujoco_warp.put_model
        # UNCONDITIONALLY -- on the CPU path too -- while put_model RAISES
        # NotImplementedError on any non-zero noslip_iterations. Setting it
        # before construction would therefore abort the build on every path.
        # After construction, mj_model.opt is ours and mj_step honours it.
        #
        # ⚠ CPU ONLY, and it says so out loud. mujoco_warp's Option struct has
        # no noslip field at all, so on that path the request is DECLINED with
        # a warning rather than silently ignored -- a knob that reads as
        # applied and does nothing is how a measurement gets attributed to the
        # wrong thing.
        try:
            import os as _nso
            _nsv = _nso.environ.get("OMNISIM_NEWTON_NOSLIP")
            _nsn = (int(_nsv) if _nsv not in (None, "")
                    else int(getattr(self, "_noslip_iters_world", 0) or 0))
            if _nsn > 0:
                _nscpu = bool(getattr(self.solver, "use_mujoco_cpu", False))
                _nsm = getattr(self.solver, "mj_model", None)
                if _nsm is not None and _nscpu:
                    _nsm.opt.noslip_iterations = int(_nsn)
                    _solver_kind += " +noslip=%d" % _nsn
                elif not _nscpu:
                    print("[OmNewtonBackend] WARNING: noslip_iterations=%d requested but "
                          "the solver is mujoco_warp, which does not implement the noslip "
                          "pass -- IGNORED. Use newtonSolver \"mujoco\" (CPU mj_step) if "
                          "the grasp depends on it." % _nsn, flush=True)
                    _solver_kind += " noslip_UNSUPPORTED_ON_WARP"
                else:
                    _solver_kind += " noslip_NO_MJMODEL"
        except Exception as _nse:
            _solver_kind += " noslip_FAILED:" + repr(_nse)[:140]
        # OPT-IN ROLLING FRICTION (OMNISIM_NEWTON_ROLL_MU=<coef>): MuJoCo's
        # default condim=3 contacts have NO rolling resistance, so cylinders
        # and capsules roll forever on flat ground (the AnyPick line's tube-
        # containment failure mode: a nudged tube never stops on its own).
        # Setting condim=6 plus a rolling coefficient on every geom gives
        # contacts physical rolling (and a little torsional) resistance.
        # Unset -> exact current physics for every existing world.
        try:
            import os as _rfo
            _rmu = _rfo.environ.get("OMNISIM_NEWTON_ROLL_MU")
            if _rmu:
                import numpy as _rnp
                _rv = float(_rmu)
                _mjm2 = getattr(self.solver, "mj_model", None)
                if _mjm2 is not None:
                    _mjm2.geom_condim[:] = 6
                    _mjm2.geom_friction[:, 1] = _rnp.maximum(
                        _mjm2.geom_friction[:, 1], 0.005)      # torsional
                    _mjm2.geom_friction[:, 2] = _rv            # rolling
                _mjwm2 = getattr(self.solver, "mjw_model", None)
                if _mjwm2 is not None:
                    try:
                        _rf = _mjwm2.geom_friction.numpy()
                        _rf[:, 1] = _rnp.maximum(_rf[:, 1], 0.005)
                        _rf[:, 2] = _rv
                        _mjwm2.geom_friction.assign(_rf)
                        _rc = _mjwm2.geom_condim.numpy()
                        _rc[:] = 6
                        _mjwm2.geom_condim.assign(_rc)
                    except Exception:
                        pass
                _solver_kind += " +roll_mu=%g" % _rv
        except Exception as _re:
            _solver_kind += " roll_mu_FAILED:" + repr(_re)[:140]
        # Expose the resolved solver to C++ finalizeWorld() so a SILENT
        # MuJoCo->XPBD fall-back (a DIFFERENT engine an mujoco_warp-trained
        # policy won't survive) is surfaced as a LOUD OmLog::warn, not just a
        # line in newton_solver.log.
        self._solver_kind = _solver_kind
        self._solver_error = "" if _solver_error is None else str(_solver_error)
        # ---- Constraint-buffer overflow watch (N15) -------------------
        # ON BY DEFAULT whenever a mujoco_warp Data buffer exists, because
        # nothing else in the process can see the truncation: mujoco_warp drops
        # constraint rows past njmax in silence and its own diagnostic is a
        # wp.printf inside a warp kernel that never reaches the engine log (see
        # _sample_constraint_peaks). Sampling is a READ of solver telemetry at a
        # coarse cadence -- it advances no state, writes no model field and
        # changes no physics; the only observable difference in a healthy world
        # is that nothing is logged at all.
        #   env unset          -> watch on, every 30 ticks, log ONLY on overflow
        #   OMNISIM_NEWTON_CONSTRAINT_STATS=N (N>0) -> cadence N + the historical
        #                         every-new-peak trace in newton_solver.log
        #   ...=0              -> every tick + trace (unchanged legacy meaning)
        #   ...=""  or  ...=-1 -> watch OFF (escape hatch)
        self._constraint_overflow_msg = ""
        self._cs_verbose = False
        self._cs_every = 0
        try:
            _cs_env = _os.environ.get("OMNISIM_NEWTON_CONSTRAINT_STATS")
            if _cs_env:
                _cs_n = int(_cs_env)
                self._cs_every = max(1, _cs_n) if _cs_n >= 0 else 0
                self._cs_verbose = self._cs_every > 0
            elif (_cs_env is None
                  and getattr(self.solver, "mjw_data", None) is not None
                  and not getattr(self.solver, "use_mujoco_cpu", False)):
                # mujoco_warp (GPU) only: newton builds mjw_data even in
                # use_mujoco_cpu mode, but that path steps mj_data through
                # MuJoCo-C, whose efc arrays grow dynamically -- there is no
                # njmax cap there and so nothing to truncate.
                # Coarse on purpose: reading a warp array forces a device sync.
                self._cs_every = 30
        except Exception:
            self._cs_every = 0
        # Log to a known file -- embedded Python stdout doesn't reach
        # the host process reliably. OMNISIM_NEWTON_LOG overrides the
        # path; default lives under the repo's .build_tmp/ where ignored
        # runtime artifacts already collect.
        try:
            import os as __os
            __log = __os.environ.get("OMNISIM_NEWTON_LOG",
                                     __os.path.join(__os.getcwd(),
                                                    ".build_tmp",
                                                    "newton_solver.log"))
            __os.makedirs(__os.path.dirname(__log), exist_ok=True)
            with open(__log, "a") as f:
                import time as _t
                f.write(f"{_t.strftime('%H:%M:%S')} solver_kind={_solver_kind} mujoco_err={_solver_error}\n")
                # Resolved world up axis + the gravity vector it produced. A wrong
                # up axis is invisible in every other line of this log (a Y-up
                # world at gravity 0 loads and steps and warns about nothing), so
                # record what the builder actually ran with.
                f.write("%s up_axis=%s up_vector=%s gravity_scalar=%s\n"
                        % (_t.strftime('%H:%M:%S'),
                           getattr(self, "_up_axis_name", "?"),
                           tuple(self.builder.up_vector), self.builder.gravity))
                # Constraint-buffer budget, recorded once per world build so an
                # overflow is diagnosable AFTER the fact. mujoco_warp's own
                # diagnostic is a wp.printf INSIDE the solver kernel: it fires
                # every tick (a 10-robot world measured tens of thousands of
                # lines a minute), cannot be rate-limited from here because it
                # is third-party device code, and on Windows the engine is a
                # GUI-subsystem binary whose stdout is discarded -- so the
                # truncation it warns about is otherwise INVISIBLE. njmax /
                # nconmax come from WorldInfo.newtonNjmax / newtonNconmax (env
                # vars win); newton raises a too-small njmax to the INITIAL
                # nefc at construction, so the initial counts below are the
                # floor, not the requirement -- the peak is what matters, see
                # OMNISIM_NEWTON_CONSTRAINT_STATS.
                if _force_mujoco:
                    _md0 = getattr(self.solver, "mj_data", None)
                    f.write("%s constraint buffers: njmax=%s nconmax=%s (0=newton auto) "
                            "initial nefc=%s ncon=%s nv=%s\n"
                            % (_t.strftime('%H:%M:%S'),
                               getattr(self, "_njmax_applied", 0),
                               getattr(self, "_nconmax_applied", 0),
                               getattr(_md0, "nefc", "?"), getattr(_md0, "ncon", "?"),
                               getattr(getattr(self.solver, "mj_model", None), "nv", "?")))
        except Exception:
            pass
        # REQUIRE the MuJoCo solver (OMNISIM_REQUIRE_MUJOCO_SOLVER=1): an RL/legged policy is
        # trained under mujoco_warp; if a deploy world silently resolves to XPBD -- a DIFFERENT
        # engine -- because it forgot the WorldInfo.newtonSolver "mujoco" pin AND no FORCE_MUJOCO
        # env was set, the policy collapses (the "long-running G1 deploy gap"). This is the mirror
        # of OMNISIM_REQUIRE_NEWTON one level deeper (Newton-MuJoCo vs Newton-XPBD): fail LOUD here
        # instead of degrading silently. Default unset -> no behaviour change. The solver line was
        # already logged above, so the crash is fully diagnosable.
        # (OMNISIM_REQUIRE_MUJOCO_SOLVER is now trivially satisfied: XPBD was
        # removed 2026-08-07 and every Newton world runs SolverMuJoCo -- a
        # construction failure raises above rather than substituting a solver.
        # The env var is still accepted from deploy launchers; it just has
        # nothing left to guard.)
        # One-shot MuJoCo-model introspection dump (OMNISIM_NEWTON_DUMP_MJMODEL=
        # <path>): geoms/bodies/actuators/excludes of the EXACT mjModel the
        # solver stepped -- the readable fallback for SAVE_MJCF when the spec
        # writer can't serialize (e.g. mesh assets). Gated -> inert when unset.
        try:
            import os as _dmo
            _dmp = _dmo.environ.get("OMNISIM_NEWTON_DUMP_MJMODEL")
            _mjm = getattr(self.solver, "mj_model", None)
            if _dmp and _mjm is not None:
                import mujoco as _mj
                _L = []
                _L.append("nq=%d nv=%d nbody=%d ngeom=%d nu=%d nexclude=%d" % (
                    _mjm.nq, _mjm.nv, _mjm.nbody, _mjm.ngeom, _mjm.nu,
                    _mjm.nexclude))
                _L.append("opt: timestep=%g cone=%d iterations=%d gravity=%s" % (
                    _mjm.opt.timestep, _mjm.opt.cone, _mjm.opt.iterations,
                    list(_mjm.opt.gravity)))
                for _b in range(_mjm.nbody):
                    _bn = _mj.mj_id2name(_mjm, _mj.mjtObj.mjOBJ_BODY, _b) or "?"
                    # body_inertia is the DIAGONAL in the body's principal frame.
                    # Dumped because a wrong inertia is one of the defect classes
                    # this model has actually shipped (a husky-wheel preset applied
                    # to every Solid without an explicit inertiaMatrix), and it is
                    # checkable from first principles: the principal moments must be
                    # positive and satisfy the triangle inequality Ia + Ib >= Ic.
                    _L.append("body %d %-24s mass=%.6g ipos=%s pos=%s inertia=%s" % (
                        _b, _bn, _mjm.body_mass[_b],
                        [round(float(v), 3) for v in _mjm.body_ipos[_b]],
                        [round(float(v), 3) for v in _mjm.body_pos[_b]],
                        [float(v) for v in _mjm.body_inertia[_b]]))
                # Joint ranges: a limit the .wbt declared and the model did not get
                # is invisible to every trajectory metric until something hits it.
                for _j in range(_mjm.njnt):
                    _jn = _mj.mj_id2name(_mjm, _mj.mjtObj.mjOBJ_JOINT, _j) or "?"
                    # actfrcrange is where the world's `maxTorque` ACTUALLY
                    # lands (newton writes it per joint, clamping the summed
                    # actuator force). It is the only non-default constraint
                    # on a plain motorised hinge, so a servo that saturates
                    # against it -- and therefore preserves only the SIGN of
                    # its command -- is visible here and nowhere else.
                    _L.append("jnt %d %-24s type=%d limited=%d range=%s "
                              "stiffness=%.6g actfrc=%s (lim=%d)" % (
                        _j, _jn, int(_mjm.jnt_type[_j]), int(_mjm.jnt_limited[_j]),
                        [float(v) for v in _mjm.jnt_range[_j]],
                        float(_mjm.jnt_stiffness[_j]),
                        [float(v) for v in _mjm.jnt_actfrcrange[_j]],
                        int(_mjm.jnt_actfrclimited[_j])))
                # opt-level contact configuration: condim decides whether mu is
                # even CONSULTED (condim=1 = frictionless contact regardless of
                # geom_friction -- the exact failure a per-geom fric=[2,..] dump
                # line would otherwise hide), cone/impratio decide how it binds.
                _L.append("opt cone=%d impratio=%s noslip=%d o_solref=%s timestep=%s" % (
                    int(_mjm.opt.cone), _mjm.opt.impratio,
                    int(_mjm.opt.noslip_iterations),
                    [round(float(v), 4) for v in _mjm.opt.o_solref],
                    _mjm.opt.timestep))
                for _g in range(_mjm.ngeom):
                    _gn = _mj.mj_id2name(_mjm, _mj.mjtObj.mjOBJ_GEOM, _g) or "?"
                    # ⚠ geom_quat IS DUMPED BECAUSE A WRONG ONE IS INVISIBLE EVERYWHERE ELSE.
                    # An orientation defect does not change a geom's name, size, position or
                    # friction, so a dump without this field looks completely healthy while the
                    # collider lies along the wrong axis. That is exactly how a hard-coded -90 deg
                    # about X on every cylinder survived two months: it cancelled on wheels (whose
                    # Pose authors +90, and a capsule is symmetric about its centre) and tipped
                    # every arm capsule Z->Y, and nothing anyone could read said so. mujoco stores
                    # it (w, x, y, z); the .wbt authors axis-angle.
                    _L.append("geom %d %-24s body=%d type=%d condim=%d contype=%d conaff=%d "
                              "fric=%s size=%s pos=%s quat=%s solref=%s solimp=%s" % (
                        _g, _gn, _mjm.geom_bodyid[_g], _mjm.geom_type[_g],
                        _mjm.geom_condim[_g],
                        _mjm.geom_contype[_g], _mjm.geom_conaffinity[_g],
                        [round(float(v), 3) for v in _mjm.geom_friction[_g]],
                        [round(float(v), 4) for v in _mjm.geom_size[_g]],
                        [round(float(v), 3) for v in _mjm.geom_pos[_g]],
                        [round(float(v), 4) for v in _mjm.geom_quat[_g]],
                        [round(float(v), 4) for v in _mjm.geom_solref[_g]],
                        [round(float(v), 4) for v in _mjm.geom_solimp[_g]]))
                for _u in range(_mjm.nu):
                    # ⚠ THESE ARE THE FIELDS maxTorque DOES **NOT** LAND IN,
                    # and they are dumped to say so out loud. Both read
                    # [0,0]/lim=0 (unlimited) on every revolute: newton's
                    # MuJoCo converter leaves the shared actuator dict's
                    # ctrlrange/ctrllimited commented out, and sets
                    # `forcerange` only on the BALL-joint branch.
                    #
                    # A world's `maxTorque` becomes the JOINT-level clamp
                    # `jnt_actfrcrange` (dumped in the jnt loop above), which
                    # limits the SUMMED actuator force per DOF. Reading an
                    # unlimited actuator_forcerange as "no torque clamp
                    # anywhere" is the wrong conclusion and cost a diagnosis
                    # once already. `maxVelocity` reaches the solver not at
                    # all -- newton's converter never reads velocity_limit
                    # ("MuJoCo doesn't have velocity limit"); OmniSim's own
                    # post-step |qd| saturation is the only thing enforcing it.
                    _L.append("act %d gain=%s bias=%s trn=%s "
                              "forcerange=%s (lim=%d) ctrlrange=%s (lim=%d)" % (
                        _u, [round(float(v), 1) for v in _mjm.actuator_gainprm[_u][:3]],
                        [round(float(v), 1) for v in _mjm.actuator_biasprm[_u][:3]],
                        list(_mjm.actuator_trnid[_u]),
                        [round(float(v), 4) for v in _mjm.actuator_forcerange[_u]],
                        int(_mjm.actuator_forcelimited[_u]),
                        [round(float(v), 4) for v in _mjm.actuator_ctrlrange[_u]],
                        int(_mjm.actuator_ctrllimited[_u])))
                for _d in range(_mjm.nv):
                    _L.append("dof %d damping=%.2f armature=%.4f frictionloss=%.3f" % (
                        _d, _mjm.dof_damping[_d], _mjm.dof_armature[_d],
                        _mjm.dof_frictionloss[_d]))
                with open(_dmp, "w") as _df:
                    _df.write("\n".join(_L) + "\n")
        except Exception as _de:
            try:
                _dmp2 = _dmo.environ.get("OMNISIM_NEWTON_DUMP_MJMODEL")
                if _dmp2:
                    with open(_dmp2, "w") as _df:
                        _df.write("DUMP FAILED: %r\n" % (_de,))
            except Exception:
                pass
        self.state_a = self.model.state()
        self.state_b = self.model.state()
        self._fin_mark("state_alloc")
        # eval_fk so initial body_q reflects builder.joint_q.
        newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, self.state_a)
        # FIX 5 (t=0 setVelocity drop): drain velocity writes queued by
        # set_body_vel() while state_a was still None (pre-finalize
        # Supervisor immediate messages). Re-dispatching through
        # set_body_vel now writes body_qd + the free-joint joint_qd half
        # and marks the MuJoCo-side data dirty.
        _pend = getattr(self, "_pending_body_vel", None)
        if _pend:
            self._pending_body_vel = {}
            for (_bi, _ang), (_vx, _vy, _vz) in _pend.items():
                self.set_body_vel(_bi, _vx, _vy, _vz, _ang)

    # ---- In-engine MPC (sampling rollouts in the SAME solver the deploy steps) ----
    def _mpc_log(self, msg):
        import os as _o, sys as _s
        line = "[inengine-mpc] " + msg + "\n"
        try:
            _s.stderr.write(line); _s.stderr.flush()
        except Exception:
            pass
        p = _o.environ.get("OMNISIM_INENGINE_MPC_LOG")
        if p:
            try:
                with open(p, "a") as _f:
                    _f.write(line)
            except Exception:
                pass

    def _mpc_rollout_buffers(self, K):
        # Build (once, cached) a K-world batched mujoco_warp Model/Data from the
        # SAME compiled CPU MjModel the live SolverMuJoCo steps -> the rollout
        # physics is byte-identical to the deploy. Returns (mjw, model, data, dims).
        import os as _o
        import mujoco as _mj
        import mujoco_warp as _mjw
        sol = self.solver
        mjm = getattr(sol, "mj_model", None)
        if mjm is None:
            return None
        if getattr(self, "_roll_K", None) != int(K):
            _mjd = _mj.MjData(mjm)
            self._roll_mjw = _mjw
            self._roll_m = _mjw.put_model(mjm)
            self._roll_d = _mjw.put_data(
                mjm, _mjd, nworld=int(K),
                njmax=int(_o.environ.get("MPC_NJMAX", "256")),
                nconmax=int(_o.environ.get("MPC_NCONMAX", "256")))
            self._roll_K = int(K)
            self._roll_dims = (int(mjm.nq), int(mjm.nv), int(mjm.nu))
        return (self._roll_mjw, self._roll_m, self._roll_d, self._roll_dims)

    def _mpc_seed_from_live(self, K):
        # Broadcast the live (nworld=1) deploy state into all K rollout worlds.
        import numpy as _np
        live = getattr(self.solver, "mjw_data", None)
        if live is None:
            return False
        nq, nv, nu = self._roll_dims
        rd = self._roll_d

        def _bc(arr, vec):
            cur = arr.numpy()
            v = _np.asarray(vec, dtype=cur.dtype).reshape(-1)
            if cur.ndim >= 2:
                w = cur.shape[-1]
                tgt = _np.broadcast_to(v[:w], (cur.shape[0], w)).astype(cur.dtype)
            else:
                w = cur.shape[0] // int(K)
                tgt = _np.tile(v[:w], int(K)).astype(cur.dtype)
            arr.assign(tgt.reshape(cur.shape))
        _bc(rd.qpos, live.qpos.numpy().reshape(-1)[:nq])
        _bc(rd.qvel, live.qvel.numpy().reshape(-1)[:nv])
        _bc(rd.ctrl, live.ctrl.numpy().reshape(-1)[:nu])
        return True

    def mpc_selftest(self, K, H):
        # Latency probe: K-world x H-step batched rollout in the deploy solver.
        # Measures COLD (incl. warp JIT) vs WARM (CUDA-graph captured) per-plan
        # time -- the realistic number for an in-engine MPC. Env knobs: MPC_REPEAT
        # (warm iters), MPC_NOGRAPH (disable graph capture).
        import time as _t, os as _o, numpy as _np
        buf = self._mpc_rollout_buffers(int(K))
        if buf is None:
            self._mpc_log("no mj_model on solver %r -- cannot roll out"
                          % (type(self.solver).__name__,))
            return
        _mjw, rm, rd, (nq, nv, nu) = buf
        K = int(K); H = int(H)
        sub = int(getattr(self, "_n_substeps", 4))
        R = int(_o.environ.get("MPC_REPEAT", "8"))
        use_graph = _o.environ.get("MPC_NOGRAPH") is None

        def _sync():
            try:
                wp.synchronize()
            except Exception:
                pass
        # ---- COLD plan (first call: includes warp kernel JIT) ----
        self._mpc_seed_from_live(K)
        _mjw.forward(rm, rd)
        _sync()
        t0 = _t.perf_counter()
        for _h in range(H):
            for _s in range(sub):
                _mjw.step(rm, rd)
        _sync()
        cold = (_t.perf_counter() - t0) * 1000.0
        # ---- seed cost (host->device broadcast of the live state into K worlds) ----
        _sync(); t0 = _t.perf_counter()
        self._mpc_seed_from_live(K)
        _sync()
        seed_ms = (_t.perf_counter() - t0) * 1000.0
        # ---- build a CUDA-graph of the H*sub step sequence (kernels now compiled) ----
        graph = None
        if use_graph:
            try:
                dev = getattr(self.model, "device", None)
                _ctx = wp.ScopedDevice(dev) if dev is not None else None
                if _ctx is not None:
                    _ctx.__enter__()
                _sync()
                wp.capture_begin(force_module_load=False)
                try:
                    for _h in range(H):
                        for _s in range(sub):
                            _mjw.step(rm, rd)
                    graph = wp.capture_end()
                finally:
                    if _ctx is not None:
                        _ctx.__exit__(None, None, None)
            except Exception as _e:
                self._mpc_log("graph capture failed (%r); timing plain loop" % (_e,))
                graph = None
        # ---- WARM timing: R plans, re-seeding each (realistic receding horizon) ----
        warm = []
        for _it in range(R):
            self._mpc_seed_from_live(K)
            _sync()
            t0 = _t.perf_counter()
            if graph is not None:
                wp.capture_launch(graph)
            else:
                for _h in range(H):
                    for _s in range(sub):
                        _mjw.step(rm, rd)
            _sync()
            warm.append((_t.perf_counter() - t0) * 1000.0)
        wa = _np.array(warm)
        outq = rd.qpos.numpy().reshape(K, nq)
        self._mpc_log(
            "K=%d H=%d sub=%d nq=%d nu=%d graph=%s | COLD=%.1fms | "
            "WARM min=%.1f mean=%.1f max=%.1f ms | seed=%.2fms | "
            "plan~=%.1fms (warm_min+seed) vs 16ms budget | bz0=%.3f"
            % (K, H, sub, nq, nu, graph is not None, cold,
               wa.min(), wa.mean(), wa.max(), seed_ms, wa.min() + seed_ms,
               float(outq[0, 2])))

    # ---- In-engine sampling-MPC stand planner (the deploy solver IS the predictor) ----
    def _mpc_build_maps(self):
        # Build (once) the balance-joint maps from the compiled MjModel: name ->
        # mjc joint id -> (position actuator, qpos adr, qvel adr) and -> Newton DOF
        # (via the solver's mjc_jnt_to_newton_dof). Returns True once ready.
        if getattr(self, "_mpc_ready", None) is not None:
            return self._mpc_ready
        import numpy as _np
        try:
            import mujoco as _mj
        except Exception as _e:
            self._mpc_log("maps: mujoco import failed %r" % (_e,)); self._mpc_ready = False; return False
        sol = self.solver
        mjm = getattr(sol, "mj_model", None)
        m2nd = getattr(sol, "mjc_jnt_to_newton_dof", None)
        if mjm is None or m2nd is None:
            self._mpc_log("maps: solver lacks mj_model/mjc_jnt_to_newton_dof"); self._mpc_ready = False; return False
        m2nd = m2nd.numpy()
        if m2nd.ndim == 2:
            m2nd = m2nd[0]
        # joint -> its position actuator (affine pos servo: biasprm[1] != 0)
        act_of_joint = {}
        for a in range(int(mjm.nu)):
            if int(mjm.actuator_trntype[a]) == int(_mj.mjtTrn.mjTRN_JOINT):
                j = int(mjm.actuator_trnid[a, 0])
                if float(mjm.actuator_biasprm[a, 1]) != 0.0 and j not in act_of_joint:
                    act_of_joint[j] = a
        # Diagnostic dump (joint names differ across importers) so matching is fixable.
        _names = []
        for j in range(int(mjm.njnt)):
            _nm = _mj.mj_id2name(mjm, _mj.mjtObj.mjOBJ_JOINT, j)
            _names.append(_nm if _nm is not None else "")
        self._mpc_log("mjm njnt=%d nu=%d m2nd_len=%d actuated_joints=%s"
                      % (int(mjm.njnt), int(mjm.nu), len(m2nd), sorted(act_of_joint.keys())))
        self._mpc_log("mjm joint names=%s" % (_names,))
        # Select balance joints by NEWTON DOF (authoritative, via m2nd). The
        # SolverMuJoCo "joint_<N>" names are NOT URDF order, but the Newton DOF
        # layout is: free base = DOF 0-5, then the 23 revolutes in URDF/spec order
        # from DOF 6 (left leg 6-11, right leg 12-17, waist 18, left arm 19-23,
        # right arm 24-28). So the G1 balance joints are these DOFs:
        DOF_SIG = {6: 0.10, 12: 0.10,    # hip_pitch   L/R  (dof 6, 12)
                   7: 0.08, 13: 0.08,    # hip_roll    L/R  (dof 7, 13)
                   10: 0.10, 16: 0.10,   # ankle_pitch L/R  (dof 10, 16)
                   11: 0.06, 17: 0.06,   # ankle_roll  L/R  (dof 11, 17)
                   19: 0.16, 24: 0.16,   # shoulder_pitch L/R (dof 19, 24)
                   20: 0.12, 25: 0.12}   # shoulder_roll  L/R (dof 20, 25)
        # Per-joint residual CAP (rad): arms swing freely (the primary balance lever,
        # like the deterministic's 1.5-rad arm swing); HIPS are clamped tiny so the
        # planner can't deepen the squat and SINK the body; ankles moderate.
        DOF_RMAX = {6: 0.12, 12: 0.12,   # hip_pitch  (tiny -> no body sink)
                    7: 0.10, 13: 0.10,   # hip_roll
                    10: 0.35, 16: 0.35,  # ankle_pitch
                    11: 0.22, 17: 0.22,  # ankle_roll
                    19: 1.20, 24: 1.20,  # shoulder_pitch (swing free)
                    20: 0.90, 25: 0.90}  # shoulder_roll
        SUBSTR = {"hip_pitch": (0.10, 0.12), "hip_roll": (0.08, 0.10),
                  "ankle_pitch": (0.10, 0.35), "ankle_roll": (0.06, 0.22),
                  "shoulder_pitch": (0.16, 1.20), "shoulder_roll": (0.12, 0.90)}
        bal = []
        sig = []
        rmax = []
        for j, nm in enumerate(_names):
            if j not in act_of_joint or j >= len(m2nd):
                continue
            ndof = int(m2nd[j])
            if ndof < 0:
                continue
            pair = next((v for k, v in SUBSTR.items() if k in nm.lower()), None)  # real names
            if pair is not None:
                s, rm = pair
            else:
                s = DOF_SIG.get(ndof); rm = DOF_RMAX.get(ndof)                    # else by DOF
            if s is None or rm is None:
                continue
            bal.append((nm, j, int(act_of_joint[j]), int(ndof)))
            sig.append(s)
            rmax.append(rm)
        if not bal:
            self._mpc_log("maps: NO balance joints resolved -- planner disabled"); self._mpc_ready = False; return False
        self._bal = bal
        self._bal_act = _np.array([b[2] for b in bal], dtype=_np.int32)
        self._bal_dof = _np.array([b[3] for b in bal], dtype=_np.int32)
        self._mpc_sigma = _np.array(sig, dtype=_np.float64)
        self._mpc_resmax = _np.array(rmax, dtype=_np.float64)
        self._mpc_nom = _np.zeros(len(bal), dtype=_np.float64)
        self._mpc_rng = _np.random.default_rng(0)
        # CoM-centering integral state: acts on the ankle_pitch joints (DOF 10/16),
        # the flat-foot fore/aft CoM lever. Sign + gain tuned via MPC_TRIM_KI.
        self._ankp_mask = _np.array([1.0 if b[3] in (10, 16) else 0.0 for b in bal],
                                    dtype=_np.float64)
        self._itp = 0.0
        self._mpc_ibias = _np.zeros(len(bal), dtype=_np.float64)
        self._mpc_ready = True
        self._mpc_log("maps OK: %d balance joints %s | newton_dof=%s"
                      % (len(bal), [b[0] for b in bal], [b[3] for b in bal]))
        return True

    def _mpc_seed_qv(self, K):
        import numpy as _np
        live = self.solver.mjw_data
        nq, nv, nu = self._roll_dims
        rd = self._roll_d

        def _bc(arr, vec):
            cur = arr.numpy()
            v = _np.asarray(vec, dtype=cur.dtype).reshape(-1)
            if cur.ndim >= 2:
                w = cur.shape[-1]; tgt = _np.broadcast_to(v[:w], (cur.shape[0], w)).astype(cur.dtype)
            else:
                w = cur.shape[0] // int(K); tgt = _np.tile(v[:w], int(K)).astype(cur.dtype)
            arr.assign(tgt.reshape(cur.shape))
        _bc(rd.qpos, live.qpos.numpy().reshape(-1)[:nq])
        _bc(rd.qvel, live.qvel.numpy().reshape(-1)[:nv])
        return live.ctrl.numpy().reshape(-1)[:nu]

    def _mpc_plan(self, K, H):
        import os as _o, numpy as _np
        _mjw, rm, rd, (nq, nv, nu) = self._roll_buf
        sub = int(getattr(self, "_n_substeps", 4))
        lam = float(_o.environ.get("MPC_LAM", "0.2"))
        iters = int(_o.environ.get("MPC_ITERS", "2"))
        res_max = float(_o.environ.get("MPC_RESMAX", "1.0"))   # SCALE on per-joint caps
        rmv = self._mpc_resmax * res_max                        # per-joint cap vector
        zref = float(_o.environ.get("MPC_ZREF", "0.74"))
        sigscale = float(_o.environ.get("MPC_SIGMA_SCALE", "2.0"))
        wT = float(_o.environ.get("MPC_WTILT", "6.0"))
        wR = float(_o.environ.get("MPC_WROLL", "6.0"))
        wY = 2.0
        wRate = float(_o.environ.get("MPC_WRATE", "0.8"))   # base ang-vel damping
        wVx, wVy, wBz, wRes = 2.0, 3.0, 8.0, 0.3
        NR = len(self._bal)
        rng = self._mpc_rng
        bal_act = self._bal_act
        ibias = getattr(self, "_mpc_ibias", None)           # CoM-centering integral

        def _setctrl(knots, base_ctrl):
            cur = rd.ctrl.numpy()
            flat2 = _np.broadcast_to(base_ctrl, (int(K), nu)).copy()
            flat2[:, bal_act] += knots
            if ibias is not None:
                flat2[:, bal_act] += ibias        # plan around the trimmed setpoint
            rd.ctrl.assign(flat2.reshape(cur.shape).astype(cur.dtype))

        def _rpy(q):
            w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
            roll = _np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
            pitch = _np.arcsin(_np.clip(2 * (w * y - z * x), -1, 1))
            yaw = _np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            return roll, pitch, yaw
        # Capture the H*sub rollout step-sequence into a CUDA graph ONCE; re-launch
        # it every plan after seeding qpos/qvel/ctrl in place. This is the ~30ms/plan
        # path; without it the plain step loop is ~50x slower (unusable real-time).
        g = getattr(self, "_roll_graph", None)
        if g is None and _o.environ.get("MPC_NOGRAPH") is None:
            try:
                self._mpc_seed_qv(K)
                _mjw.forward(rm, rd)
                for _s in range(sub):
                    _mjw.step(rm, rd)            # warm kernels before capture
                wp.synchronize()
                dev = getattr(self.model, "device", None)
                _ctx = wp.ScopedDevice(dev) if dev is not None else None
                if _ctx is not None:
                    _ctx.__enter__()
                wp.capture_begin(force_module_load=False)
                try:
                    for _h in range(int(H)):
                        for _s in range(sub):
                            _mjw.step(rm, rd)
                    g = wp.capture_end()
                finally:
                    if _ctx is not None:
                        _ctx.__exit__(None, None, None)
                self._roll_graph = g
                self._mpc_log("rollout CUDA graph captured (H=%d sub=%d)" % (int(H), sub))
            except Exception as _e:
                self._mpc_log("graph capture failed %r; plain loop" % (_e,))
                self._roll_graph = False
                g = False
        for _it in range(iters):
            base_ctrl = self._mpc_seed_qv(K)
            noise = rng.normal(0.0, 1.0, size=(int(K), NR)) * (self._mpc_sigma * sigscale)
            knots = _np.clip(self._mpc_nom[None, :] + noise, -rmv[None, :], rmv[None, :])
            knots[0] = self._mpc_nom
            _setctrl(knots, base_ctrl)
            if g:
                wp.capture_launch(g)
            else:
                try:
                    _mjw.forward(rm, rd)
                except Exception:
                    pass
                for _h in range(int(H)):
                    for _s in range(sub):
                        _mjw.step(rm, rd)
            try:
                wp.synchronize()
            except Exception:
                pass
            qp = rd.qpos.numpy().reshape(int(K), nq)
            qv = rd.qvel.numpy().reshape(int(K), nv)
            z = qp[:, 2]
            roll, pitch, yaw = _rpy(qp[:, 3:7])
            vx, vy = qv[:, 0], qv[:, 1]
            rrate, prate = qv[:, 3], qv[:, 4]    # base angular velocity (roll/pitch)
            J = (wT * pitch * pitch + wR * roll * roll + wY * yaw * yaw
                 + wVx * vx * vx + wVy * vy * vy
                 + wRate * (prate * prate + rrate * rrate)
                 + wBz * _np.maximum(0.0, zref - z) ** 2
                 + wRes * (knots * knots).sum(axis=1))
            fell = (z < 0.45) | (_np.abs(roll) > 0.8) | (_np.abs(pitch) > 0.8)
            J = J + fell * 1000.0
            w = _np.exp(-(J - J.min()) / lam)
            w = w / (w.sum() + 1e-9)
            self._mpc_nom = _np.clip((w[:, None] * knots).sum(axis=0), -rmv, rmv)

    def mpc_stand_step(self):
        import os as _o
        if not self._mpc_build_maps():
            return
        K = int(_o.environ.get("MPC_K", "96"))
        H = int(_o.environ.get("MPC_H", "28"))
        buf = self._mpc_rollout_buffers(K)
        if buf is None:
            return
        self._roll_buf = buf
        # ---- CoM-centering integral: integrate the live base pitch and bias the
        # ankle_pitch setpoint to drive a steady fore/aft lean to zero (the MPPI
        # residual handles the fast balance; this absorbs the slow offset). Gain +
        # sign via MPC_TRIM_KI (0 = off). ----
        import numpy as _np2
        trim_ki = float(_o.environ.get("MPC_TRIM_KI", "0.0"))
        if trim_ki != 0.0:
            try:
                import math as _m
                q = self.solver.mjw_data.qpos.numpy().reshape(-1)
                qw, qx, qy, qz = float(q[3]), float(q[4]), float(q[5]), float(q[6])
                pitch_live = _m.asin(max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx))))
                tc = float(_o.environ.get("MPC_TRIM_CLAMP", "0.30"))
                dt = float(getattr(self, "_n_substeps", 4)) * 0.004
                self._itp = max(-1e3, min(1e3, self._itp + pitch_live * dt))
                bias_ankp = max(-tc, min(tc, trim_ki * self._itp))
                self._mpc_ibias = bias_ankp * self._ankp_mask
            except Exception as _e:
                self._mpc_log("trim error: %r" % (_e,))
        every = max(1, int(_o.environ.get("MPC_REPLAN_EVERY", "1")))
        self._mpc_planctr = int(getattr(self, "_mpc_planctr", 0))
        warm = int(_o.environ.get("MPC_WARMTICKS", "4"))
        if self._mpc_planctr >= warm and (self._mpc_planctr - warm) % every == 0:
            try:
                self._mpc_plan(K, H)
            except Exception as _e:
                self._mpc_log("plan error: %r" % (_e,))
        self._mpc_planctr += 1
        # Apply residual + integral bias ON TOP of the controller's nominal targets.
        try:
            tp = self._ctl_target_pos().numpy()
            for _i in range(len(self._bal)):
                _d = int(self._bal_dof[_i])
                if 0 <= _d < len(tp):
                    tp[_d] += float(self._mpc_nom[_i]) + float(self._mpc_ibias[_i])
            self._ctl_target_pos().assign(tp)
            self._mjc_dirty = True
        except Exception as _e:
            self._mpc_log("apply error: %r" % (_e,))

    # ---- Whole-body CoM-balance controller (instant, analytic, model-based) ----
    # Each tick: read the live CoM + CoM Jacobian from the deploy's own model,
    # solve (damped least-squares) for the balance-joint position changes that drive
    # the CoM back over its settled support point, and apply them as offsets on top
    # of the nominal hold. INSTANT (no warm-start ramp) -> reacts like the
    # deterministic's proportional balance, unlike the sampling-MPC residual.
    def wbc_stand_step(self):
        import os as _o, numpy as _np
        if not self._mpc_build_maps():     # reuse the balance-joint maps
            return
        try:
            import mujoco as _mj
        except Exception as _e:
            self._mpc_log("wbc: mujoco import failed %r" % (_e,)); return
        sol = self.solver
        mjm = getattr(sol, "mj_model", None)
        live = getattr(sol, "mjw_data", None)
        if mjm is None or live is None:
            return
        # cache the CPU MjData + the balance joints' mjc DOF columns
        if getattr(self, "_wbc_d", None) is None:
            self._wbc_d = _mj.MjData(mjm)
            self._wbc_mjdof = _np.array([int(mjm.jnt_dofadr[b[1]]) for b in self._bal],
                                        dtype=_np.int32)
            # foot bodies = the body each ankle_roll joint (newton dof 11/17) drives
            self._wbc_feet = [int(mjm.jnt_bodyid[b[1]]) for b in self._bal
                              if b[3] in (11, 17)]
            self._wbc_ref = None
            self._wbc_log_ctr = 0
        d = self._wbc_d
        # sync the CPU model to the live deploy state (mjc layout, nworld=1)
        lq = live.qpos.numpy().reshape(-1)[:mjm.nq]
        lv = live.qvel.numpy().reshape(-1)[:mjm.nv]
        d.qpos[:] = lq
        d.qvel[:] = lv
        _mj.mj_forward(mjm, d)
        try:
            _mj.mj_subtreeVel(mjm, d)
            com_vel = _np.array(d.subtree_linvel[1][:2], dtype=_np.float64)
        except Exception:
            com_vel = _np.zeros(2)
        com = _np.array(d.subtree_com[1][:2], dtype=_np.float64)
        if self._wbc_ref is None:
            self._wbc_ref = com.copy()      # settled balanced CoM = the target
        Kp = float(_o.environ.get("WBC_KP", "6.0"))
        Kd = float(_o.environ.get("WBC_KD", "1.2"))
        lam = float(_o.environ.get("WBC_LAM", "0.02"))
        dqmax = float(_o.environ.get("WBC_DQMAX", "0.6"))
        gain = float(_o.environ.get("WBC_GAIN", "1.0"))   # sign/scale (flip if wrong)
        dcom = gain * (Kp * (self._wbc_ref - com) - Kd * com_vel)   # desired CoM shift
        nv = int(mjm.nv)
        jacp = _np.zeros((3, nv), dtype=_np.float64)
        _mj.mj_jacSubtreeCom(mjm, d, jacp, 1)             # whole-robot CoM Jacobian
        # CONTACT-CONSISTENT CoM Jacobian: with the feet planted, moving the balance
        # joints also moves the floating base (J_feet @ v = 0). Project that out so
        # the CoM sensitivity reflects the planted-feet inverted-pendulum reality.
        bd = self._wbc_mjdof
        Jf = _np.zeros((3 * len(self._wbc_feet), nv), dtype=_np.float64)
        _jp = _np.zeros((3, nv)); _jr = _np.zeros((3, nv))
        for _k, _fb in enumerate(self._wbc_feet):
            _mj.mj_jacBodyCom(mjm, d, _jp, _jr, _fb)
            Jf[3 * _k:3 * _k + 3] = _jp
        Jf_base = Jf[:, 0:6]                               # (6,6) for 2 feet
        Jf_bal = Jf[:, bd]                                 # (6, nbal)
        Pb = -_np.linalg.pinv(Jf_base) @ Jf_bal           # base vel induced by dq_bal
        Jsupp = jacp[:, bd] + jacp[:, 0:6] @ Pb           # (3, nbal) support-consistent
        J = Jsupp[0:2]                                     # horizontal CoM (2, nbal)
        JJt = J @ J.T + lam * _np.eye(2)
        dq = J.T @ _np.linalg.solve(JJt, dcom)            # (nbal,) DLS
        dq = _np.clip(dq, -dqmax, dqmax)
        tp = self._ctl_target_pos().numpy()
        for _i in range(len(self._bal)):
            _d = int(self._bal_dof[_i])
            if 0 <= _d < len(tp):
                tp[_d] += float(dq[_i])
        self._ctl_target_pos().assign(tp)
        self._mjc_dirty = True
        self._wbc_log_ctr += 1
        if self._wbc_log_ctr % 60 == 1:
            self._mpc_log("wbc: com_err=(%.3f,%.3f) com_vel=(%.3f,%.3f) |dq|=%.3f"
                          % (self._wbc_ref[0]-com[0], self._wbc_ref[1]-com[1],
                             com_vel[0], com_vel[1], float(_np.abs(dq).max())))

    # ---- QP-TSID whole-body controller (torque-level momentum/inverse-dynamics) --
    # Weighted inverse dynamics solved as one least-squares (no external QP solver):
    # decision vars [qddot(nv), contact-wrench f(6/foot)]; HARD-weighted equality
    # rows = floating-base dynamics + foot no-acceleration; soft task rows = CoM
    # (keep over support) + base-orientation (keep torso upright -> arms counter-
    # rotate = the angular-momentum strategy) + posture. Recovers actuated torques
    # tau = (M qddot + h - J_c^T f)[act] and drives them via control.joint_f.
    # Requires OMNISIM_NEWTON_TORQUE_MODE=1 (joints in EFFORT mode, joint_f = sole
    # drive). Enable with OMNISIM_INENGINE_TSID=1.
    def tsid_stand_step(self):
        import os as _o, numpy as _np
        try:
            import mujoco as _mj
        except Exception as _e:
            self._mpc_log("tsid: mujoco import failed %r" % (_e,)); return
        sol = self.solver
        mjm = getattr(sol, "mj_model", None)
        live = getattr(sol, "mjw_data", None)
        m2nj = getattr(sol, "mjc_jnt_to_newton_dof", None)   # mjc joint -> newton dof
        if mjm is None or live is None or m2nj is None:
            self._mpc_log("tsid: solver missing mj_model/mjw_data/jntmap"); return
        nv = int(mjm.nv); nq = int(mjm.nq)
        if getattr(self, "_tsid_d", None) is None:
            # Identify the ROBOT dofs by NEWTON dof (torque mode has nu=0 -> no
            # actuators; the world may also contain cubes whose free joints we skip).
            # Newton layout: robot base = dof 0-5, robot revolutes = dof 6-28.
            _mj_ = m2nj.numpy(); _mj_ = _mj_[0] if _mj_.ndim == 2 else _mj_
            base = None; bqadr = 0; act = []; feet = []
            for jj in range(int(mjm.njnt)):
                nd = int(_mj_[jj])
                if nd < 0:
                    continue
                if int(mjm.jnt_type[jj]) == int(_mj.mjtJoint.mjJNT_FREE) and nd == 0:
                    base = list(range(int(mjm.jnt_dofadr[jj]), int(mjm.jnt_dofadr[jj]) + 6))
                    bqadr = int(mjm.jnt_qposadr[jj])
                elif 6 <= nd <= 28:
                    act.append((nd, int(mjm.jnt_dofadr[jj]), int(mjm.jnt_qposadr[jj])))
                if nd in (11, 17):
                    feet.append(int(mjm.jnt_bodyid[jj]))
            act.sort()
            self._tsid_d = _mj.MjData(mjm)
            self._tsid_base = base if base is not None else [0, 1, 2, 3, 4, 5]
            self._tsid_bqadr = bqadr
            self._tsid_act_nd = [a[0] for a in act]      # newton dof (== control.joint_f index)
            self._tsid_act_md = [a[1] for a in act]      # mjc dof
            self._tsid_act_qa = [a[2] for a in act]      # mjc qpos adr
            self._tsid_feet = feet
            self._tsid_ref = None; self._tsid_qnom = None; self._tsid_ctr = 0
            # Newton/mujoco_warp applies gravity through its OWN warp path; the
            # CPU planning model (sol.mj_model) can have opt.gravity == 0, which
            # makes mj_forward's qfrc_bias gravity-free -> our gravity-comp
            # torque is ~0 -> the robot gets no holding torque and collapses
            # (verified: FF |tau|=0.8 at t=0 == max leg gravity-bias 0.8 N*m).
            # The CPU model is used ONLY for our inverse dynamics here (the
            # solver steps via its warp model), so force the physical gravity.
            try:
                _gz = float(mjm.opt.gravity[2])
            except Exception:
                _gz = 0.0
            if abs(_gz) < 1.0:
                try:
                    mjm.opt.gravity[0] = 0.0
                    mjm.opt.gravity[1] = 0.0
                    mjm.opt.gravity[2] = -9.81
                except Exception:
                    pass
            try:
                _gz2 = float(mjm.opt.gravity[2])
            except Exception:
                _gz2 = _gz
            self._mpc_log("tsid: init nv=%d nact=%d feet=%s base=%s grav_z=%s->%s"
                          % (nv, len(act), feet, self._tsid_base, _gz, _gz2))
        d = self._tsid_d
        base = self._tsid_base; amd = self._tsid_act_md; aqa = self._tsid_act_qa
        and_ = self._tsid_act_nd; feet = self._tsid_feet
        if not amd or len(feet) < 2:
            self._mpc_log("tsid: bad maps nact=%d feet=%d" % (len(amd), len(feet))); return
        # ONE-TIME self-seed: the engine-side seed block maps angles by Newton
        # DEVICE slot order, which differs from the URDF/nd joint order -- so an
        # env angle list there lands on the WRONG joints (verified: the knee got
        # the ankle's value). The TSID already resolved each actuated joint's mjc
        # qpos address (aqa) in nd order, so seed the squat HERE, straight into the
        # live solver state, before this substep steps. TSID_SEED_Q is the actuated
        # joints in nd order (== spec joint_order); TSID_SEED_BASEZ sets the base
        # height so the squat's feet rest on the ground (no first-step drop).
        if not getattr(self, "_tsid_seeded", False):
            self._tsid_seeded = True
            _sq = _o.environ.get("TSID_SEED_Q")
            if _sq:
                try:
                    _sqv = [float(_x) for _x in _sq.split(",") if _x != ""]
                    _qpf = live.qpos.numpy(); _sh = _qpf.shape
                    _fl = _qpf.reshape(-1).copy(); _ns = 0
                    for _i in range(min(len(aqa), len(_sqv))):
                        _qi = int(aqa[_i])
                        if 0 <= _qi < _fl.shape[0]:
                            _fl[_qi] = _sqv[_i]; _ns += 1
                    _bz = _o.environ.get("TSID_SEED_BASEZ")
                    if _bz:
                        _bzi = self._tsid_bqadr + 2
                        if 0 <= _bzi < _fl.shape[0]:
                            _fl[_bzi] = float(_bz)
                    live.qpos.assign(_fl.reshape(_sh))
                    _qdf = live.qvel.numpy(); _qsh = _qdf.shape
                    _qfl = _qdf.reshape(-1).copy(); _qfl[:] = 0.0
                    live.qvel.assign(_qfl.reshape(_qsh))
                    self._mpc_log("tsid: self-seeded %d joints (base_z=%s)" % (_ns, _bz))
                except Exception as _se:
                    self._mpc_log("tsid self-seed failed %r" % (_se,))
        d.qpos[:] = live.qpos.numpy().reshape(-1)[:nq]
        d.qvel[:] = live.qvel.numpy().reshape(-1)[:nv]
        _mj.mj_forward(mjm, d)
        if not getattr(self, "_tsid_dbg", False):
            self._tsid_dbg = True
            try:
                _hd = _np.array(d.qfrc_bias, dtype=_np.float64)
                _qd0 = _np.array(d.qpos, dtype=_np.float64)
                _legq = [(self._tsid_act_nd[_i], round(float(_qd0[self._tsid_act_qa[_i]]), 3),
                          round(float(_hd[amd[_i]]), 2)) for _i in range(len(amd))]
                try:
                    _msm = float(mjm.body_subtreemass[1])
                except Exception:
                    _msm = -1.0
                self._mpc_log("tsid_dbg: mrob=%.2f base_z=%.3f |h_full|=%.1f legs(nd,q,h)=%s"
                              % (_msm, float(_qd0[self._tsid_bqadr+2]),
                                 float(_np.abs(_hd).max()), _legq[:6]))
            except Exception as _de:
                self._mpc_log("tsid_dbg failed %r" % (_de,))
        try:
            _mj.mj_subtreeVel(mjm, d)
            com_vel = _np.array(d.subtree_linvel[1], dtype=_np.float64)
        except Exception:
            com_vel = _np.zeros(3)
        com = _np.array(d.subtree_com[1], dtype=_np.float64)
        qp = _np.array(d.qpos, dtype=_np.float64); qd = _np.array(d.qvel, dtype=_np.float64)
        if self._tsid_ref is None:
            self._tsid_ref = com.copy(); self._tsid_qnom = qp.copy()
        Kp_c = float(_o.environ.get("TSID_KP_COM", "30.0")); Kd_c = float(_o.environ.get("TSID_KD_COM", "9.0"))
        Kp_o = float(_o.environ.get("TSID_KP_ORI", "80.0")); Kd_o = float(_o.environ.get("TSID_KD_ORI", "16.0"))
        Kp_p = float(_o.environ.get("TSID_KP_POST", "20.0")); Kd_p = float(_o.environ.get("TSID_KD_POST", "5.0"))
        w_com = float(_o.environ.get("TSID_W_COM", "20.0")); w_ori = float(_o.environ.get("TSID_W_ORI", "12.0"))
        w_post = float(_o.environ.get("TSID_W_POST", "0.4")); w_freg = float(_o.environ.get("TSID_W_FREG", "0.3"))
        kd_j = float(_o.environ.get("TSID_KD_JOINT", "2.0"))   # joint-vel damping (EFFORT mode has none)
        taumax = float(_o.environ.get("TSID_TAUMAX", "100.0"))
        gain = float(_o.environ.get("TSID_GAIN", "1.0"))
        tausign = float(_o.environ.get("TSID_TAU_SIGN", "1.0"))   # Newton joint_f vs mjc tau
        a_com = gain * (Kp_c * (self._tsid_ref - com) - Kd_c * com_vel)
        bq = self._tsid_bqadr
        qw, qx, qy, qz = qp[bq+3], qp[bq+4], qp[bq+5], qp[bq+6]
        roll = _np.arctan2(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy))
        pitch = _np.arcsin(max(-1.0, min(1.0, 2*(qw*qy-qz*qx))))
        wb = [qd[base[3]], qd[base[4]], qd[base[5]]]
        a_ori = gain * _np.array([-Kp_o*roll - Kd_o*wb[0],
                                  -Kp_o*pitch - Kd_o*wb[1],
                                  -Kd_o*wb[2]], dtype=_np.float64)
        nact = len(amd)
        a_post = _np.array([Kp_p*(self._tsid_qnom[aqa[i]] - qp[aqa[i]]) - Kd_p*qd[amd[i]]
                            for i in range(nact)], dtype=_np.float64)
        # ---- FEEDFORWARD mode (TSID_FF=1): operational-space balance torque ADDED
        # on top of a position servo (NOT torque-mode). The servo gives substep-rate
        # stability+gravity hold; this adds CoM (J_com^T F) + torso-orientation
        # (J_baseR^T tau -> swings the arms = angular-momentum strategy). Avoids the
        # 62.5Hz pure-torque instability and the contact-force QP. ----
        if _o.environ.get("TSID_FF") not in (None, "", "0"):
            try:
                mrob = float(mjm.body_subtreemass[1])
            except Exception:
                mrob = 34.0
            Jcom = _np.zeros((3, nv)); _mj.mj_jacSubtreeCom(mjm, d, Jcom, 1)
            JbR = _np.zeros((3, nv)); _jpb = _np.zeros((3, nv))
            _mj.mj_jacBody(mjm, d, _jpb, JbR, 1)
            Fcom = _np.array([a_com[0]*mrob, a_com[1]*mrob, 0.0], dtype=_np.float64)
            tau_full = Jcom.T @ Fcom + JbR.T @ a_ori          # (nv,) operational-space
            _hf = _np.array(d.qfrc_bias, dtype=_np.float64)   # gravity + Coriolis
            _gravw = float(_o.environ.get("TSID_FF_GRAV", "1.0"))   # gravity-comp weight
            tau = _np.array([tau_full[md] for md in amd]) \
                + _gravw * _np.array([_hf[md] for md in amd]) \
                - kd_j * _np.array([qd[md] for md in amd]) \
                + _np.array(a_post)                            # a_post here = posture torque
            # ANGULAR-MOMENTUM (arm-swing) recovery -- the lever the pure CoM+ori FF
            # lacks: drive the shoulder_pitch joints with a torque ~ (pitch + pitch-rate)
            # so the arms swing and their reaction rotates the torso back (the capture
            # strategy the deterministic stand uses via arm_balance). roll -> shoulder_roll
            # (antisymmetric L/R). nd19/24 = shoulder_pitch L/R, nd20/25 = shoulder_roll L/R
            # (G1 URDF order). Env-tunable, default 0 (off); sign of k_arm* picks the
            # convention (Newton joint_f vs arm-forward).
            k_armp = float(_o.environ.get("TSID_K_ARMP", "0.0"))
            k_armr = float(_o.environ.get("TSID_K_ARMR", "0.0"))
            if k_armp != 0.0 or k_armr != 0.0:
                _kar = float(_o.environ.get("TSID_K_ARM_KD", "0.2"))   # tilt-rate weight
                _ap_cmd = k_armp * (pitch + _kar * wb[1])
                _ar_cmd = k_armr * (roll + _kar * wb[0])
                for _ai in range(nact):
                    _nda = int(and_[_ai])
                    if _nda == 19 or _nda == 24:
                        tau[_ai] += _ap_cmd
                    elif _nda == 20:
                        tau[_ai] += _ar_cmd
                    elif _nda == 25:
                        tau[_ai] -= _ar_cmd
            if not _np.all(_np.isfinite(tau)):
                self._mpc_log("tsid_ff: non-finite -> skip"); return
            tau = tausign * _np.clip(tau, -taumax, taumax)
            if self.control is None:
                self.control = self.model.control()
            jf = self.control.joint_f.numpy(); jf[:] = 0.0
            for _i in range(nact):
                nd = int(and_[_i])
                if 0 <= nd < len(jf):
                    jf[nd] = float(tau[_i])
            self.control.joint_f.assign(jf)
            self._mjc_dirty = True
            self._tsid_ctr += 1
            if self._tsid_ctr % 60 == 1:
                self._mpc_log("tsid_ff: roll=%.3f pitch=%.3f com_err=(%.3f,%.3f) |h|=%.1f |tau|=%.1f"
                              % (roll, pitch, self._tsid_ref[0]-com[0], self._tsid_ref[1]-com[1],
                                 float(_np.abs(_np.array([_hf[md] for md in amd])).max()),
                                 float(_np.abs(tau).max())))
            return
        M = _mj_full_mass(_mj, mjm, d, nv, _np)
        h = _np.array(d.qfrc_bias, dtype=_np.float64)
        Jcom = _np.zeros((3, nv)); _mj.mj_jacSubtreeCom(mjm, d, Jcom, 1)
        nf = 6 * len(feet)
        Jc = _np.zeros((nf, nv)); _jp = _np.zeros((3, nv)); _jr = _np.zeros((3, nv))
        for _k, _fb in enumerate(feet):
            _mj.mj_jacBodyCom(mjm, d, _jp, _jr, _fb)
            Jc[6*_k:6*_k+3] = _jp; Jc[6*_k+3:6*_k+6] = _jr
        JcT = Jc.T
        ncol = nv + nf
        rA = []; rb = []; rw = []
        def _add(Ab, bb, wv):
            rA.append(Ab); rb.append(bb); rw.append(_np.full(len(bb), wv))
        # base dynamics (robot base rows): M[base] qdd - JcT[base] f = -h[base]
        A1 = _np.zeros((6, ncol)); A1[:, :nv] = M[base]; A1[:, nv:] = -JcT[base]; _add(A1, -h[base], 1e6)
        # foot no-acceleration
        A2 = _np.zeros((nf, ncol)); A2[:, :nv] = Jc; _add(A2, _np.zeros(nf), 1e5)
        # CoM task
        A3 = _np.zeros((3, ncol)); A3[:, :nv] = Jcom; _add(A3, a_com, w_com)
        # base orientation task (robot base angular dofs)
        A4 = _np.zeros((3, ncol))
        for _k in range(3):
            A4[_k, base[3+_k]] = 1.0
        _add(A4, a_ori, w_ori)
        # posture task (robot actuated dofs)
        A5 = _np.zeros((nact, ncol))
        for _i in range(nact):
            A5[_i, amd[_i]] = 1.0
        _add(A5, a_post, w_post)
        # contact-force regularization toward a PHYSICAL gravity-support distribution
        # (each foot carries an equal share of the robot weight, vertical). Without
        # friction-cone inequalities this keeps the equality-LS contact force sane
        # (positive normal, no huge tangential) so the recovered torque is physical.
        try:
            _rm = float(mjm.body_subtreemass[1])
        except Exception:
            _rm = 34.0
        _fref = _np.zeros(nf)
        for _k in range(len(feet)):
            _fref[6*_k+2] = _rm * 9.81 / max(1, len(feet))   # Fz share per foot
        A6 = _np.zeros((nf, ncol)); A6[:, nv:] = _np.eye(nf); _add(A6, _fref, w_freg)
        A = _np.vstack(rA); b = _np.concatenate(rb); sw = _np.sqrt(_np.concatenate(rw))
        x, _r, _rk, _sv = _np.linalg.lstsq(sw[:, None]*A, sw*b, rcond=None)
        qdd = x[:nv]; f = x[nv:]
        # FRICTION-CONE PROJECTION of the contact wrench (the inequality the
        # equality-LS can't enforce): per foot, normal Fz>=0, |F_tangential|<=mu*Fz,
        # and CoP within the foot (|tau_x|,|tau_y|<=Fz*half_len). Keeps the recovered
        # torque PHYSICAL so it doesn't blow up.
        mu = float(_o.environ.get("TSID_MU", "0.8"))
        cop = float(_o.environ.get("TSID_FOOT_COP", "0.07"))
        for _k in range(len(feet)):
            _b = 6*_k
            fz = f[_b+2]
            if fz < 0.0:
                fz = 0.0
            f[_b+2] = fz
            _ft = (f[_b]**2 + f[_b+1]**2) ** 0.5
            if _ft > mu*fz and _ft > 1e-9:
                _s = (mu*fz)/_ft
                f[_b] *= _s; f[_b+1] *= _s
            f[_b+3] = min(max(f[_b+3], -fz*cop), fz*cop)
            f[_b+4] = min(max(f[_b+4], -fz*cop), fz*cop)
        Mact = M[amd]; JcTact = JcT[amd]
        tau = Mact @ qdd + h[amd] - JcTact @ f
        tau = tau - kd_j * _np.array([qd[md] for md in amd])   # passive-like joint damping
        if not _np.all(_np.isfinite(tau)):
            self._mpc_log("tsid: non-finite tau -> skip (state likely diverged)"); return
        tau = tausign * _np.clip(tau, -taumax, taumax)
        if self.control is None:
            self.control = self.model.control()
        jf = self.control.joint_f.numpy(); jf[:] = 0.0
        for _i in range(nact):
            nd = int(and_[_i])
            if 0 <= nd < len(jf):
                jf[nd] = float(tau[_i])
        self.control.joint_f.assign(jf)
        self._mjc_dirty = True
        self._tsid_ctr += 1
        if self._tsid_ctr % 60 == 1:
            self._mpc_log("tsid: roll=%.3f pitch=%.3f com_err=(%.3f,%.3f,%.3f) |h|=%.1f |tau|=%.1f"
                          % (roll, pitch, self._tsid_ref[0]-com[0], self._tsid_ref[1]-com[1],
                             self._tsid_ref[2]-com[2], float(_np.abs(h[amd]).max()),
                             float(_np.abs(tau).max())))

    # --- step profiling (OMNISIM_NEWTON_STEP_PROFILE=1) ----------------------
    # Exists because the per-step cost of this path was 300x the bare solver's
    # and nobody could say WHERE. Off by default; when off, each wrapper is one
    # attribute read and a direct delegation.
    def _prof_on(self):
        v = getattr(self, "_prof_flag", None)
        if v is None:
            v = bool(_os.environ.get("OMNISIM_NEWTON_STEP_PROFILE"))
            self._prof_flag = v
            self._prof_solve = 0.0
            self._prof_py = 0.0
            self._prof_ctrl = 0.0
            self._prof_out = 0.0
            self._prof_mj = 0.0
            self._prof_pre = 0.0
            self._prof_collide = 0.0
        return v

    def _mjc_batch_substeps_ok(self, tsid_on, pymod_ss, ext_bf, contacts):
        """May this tick run its substeps as one conversion-bracketed batch?

        Deliberately conservative -- every condition here is something that
        would OBSERVE or MUTATE the intermediate substates the batch skips
        over, so each one is a correctness requirement, not a tuning knob.
        """
        # ⚠ A SECOND SOLVER MUST NEVER SEE THIS PATH. The batched branch does
        # NOT call solver.step() at all -- it reaches into SolverMuJoCo's
        # private members and drives mj_step directly. Any additional solver
        # (e.g. SolverVBD for cloth/particles) registered the ordinary way
        # would therefore be SILENTLY SKIPPED on every batched tick: the cloth
        # simply never advances, no exception is raised and nothing is logged.
        # This is the default path on CPU passive scenes, so the failure would
        # look like "cloth is frozen" with no clue as to why. Refuse to batch
        # whenever a soft solver exists; that costs the batching win only on
        # worlds that actually carry particles.
        if getattr(self, "solver_soft", None) is not None:
            return False
        if self._n_substeps < 1:
            return False
        _kill = getattr(self, "_batch_killed", None)
        if _kill is None:
            _kill = bool(_os.environ.get("OMNISIM_NEWTON_NO_SUBSTEP_BATCH"))
            self._batch_killed = _kill
        if _kill:
            return False                      # A/B switch: forces the per-substep path
        sv = getattr(self, "solver", None)
        if sv is None or not getattr(sv, "use_mujoco_cpu", False):
            return False                      # mjwarp/XPBD have their own paths
        if tsid_on or pymod_ss:
            return False                      # these RECOMPUTE control per substep
        if ext_bf is not None:
            return False                      # external body forces re-applied per substep
        if getattr(self, "_joint_f_ever", False) or self.joint_forces:
            # FORCE-MODE control must take the proven per-substep path.
            # Measured 2026-08-07 on friction_grasp_minimal.wbt (the shipped
            # flagship pinch demo): through this batch the grip SLIPPED
            # (lifted -0.0003 m) while the per-substep path -- forced by ANY
            # of NO_SUBSTEP_BATCH / KEEP_COLLIDE / EAGER_COPYIN -- HELD at
            # +0.2817 m, byte-matching the July-18 reference binary. The
            # batch's "physics bit-identical" verification (899eb425,
            # 7b3762f7: 9 decimal places over 800 steps) was done on PASSIVE
            # box stacks; no force-driven world was ever A/B'd through it,
            # and force mode is exactly where it diverges. Velocity/position
            # targets are unaffected (the 10-Husky swarm drives 2.385 m
            # through this batch, matching ODE within 1%).
            #
            # _joint_f_ever, not this tick's dict: joint_forces is CONSUMED
            # into control.joint_f earlier in the same tick (the first
            # version of this gate checked the dict here and never fired),
            # and joint_f persists across ticks until explicitly zeroed --
            # so once a world has used force mode, the proven path owns it
            # for the rest of the run. Root cause of the force-path
            # divergence is NOT yet isolated -- do not re-admit force mode
            # here without a grasp-verdict A/B.
            return False
        if contacts is not None and not self._skip_collide:
            return False                      # newton narrow-phase runs per substep
        if getattr(sv, "update_data_interval", 0) > 0:
            return False                      # newton is re-syncing state INTO mujoco
        if getattr(self, "_mjc_dirty", False):
            return False                      # a pose/target write must land the slow way
        for attr in ("mj_model", "mj_data", "_mujoco"):
            if getattr(sv, attr, None) is None:
                return False
        return True

    def _refresh_mj_cartesian(self, sv):
        """Leave mj_data's Cartesian arrays at t+dt, not at t.

        THE DEFECT. `mj_step` is `mj_forward` then the integrator, and the
        integrator advances ONLY qpos, qvel and time. Nothing recomputes the
        position/velocity-dependent arrays afterwards, so when a tick returns
        qpos is at t+dt while xpos / xquat / xipos / subtree_com / cvel are
        still at t. Measured on a moving probe (free body + pendulum, dt 4 ms):
        max |xpos - body_q| = 1.4126e-02 m, which is exactly |v|*dt, while the
        deviation against the PREVIOUS tick's body_q is 2.2e-07 -- a pure TIME
        offset, not a frame offset. After this call the same probe reads
        2.67e-07 m; the two columns swap, which is the signature.

        WHY IT MATTERS. Every plan to make this path cheaper ends at serving
        poses out of mj_data instead of re-deriving them with
        eval_articulation_fk (~0.41-0.49 ms/step, the single largest item in
        the tick). On stale arrays that readback would report every body in
        every Newton world ONE TICK LATE, everywhere, silently -- and the
        lane-1 gate would not catch it, because those scenes are scored on
        settled or smooth trajectories. Commit ee614170 concluded the arrays
        were already current, having sampled a SETTLED scene where a time
        offset is invisible by construction; ccaab865 corrected it. So this is
        a precondition for the native-tick path, not an optimisation of it.

        WHY mj_step1. mj_step == mj_step1 (+ mj_step2). Running one more
        mj_step1 after the last substep recomputes exactly the position and
        velocity kinematics at the new qpos/qvel and touches nothing the
        integrator owns, so the physics is untouched: it changes no qpos, no
        qvel, no time, and the next tick's mj_step recomputes all of it anyway.
        It is NOT free -- it costs one extra mj_step1 per engine tick (mj_step
        measures ~0.034 ms in total here, so order 0.02 ms) -- which buys the
        removal of a 0.41-0.49 ms FK. The zero-cost form folds this call into
        the NEXT tick's leading step1 (step1/step2 pairs instead of mj_step),
        but that needs the "mj_data is position-current at tick entry"
        invariant to hold across every write into qpos/qvel, so it is a
        separate change with its own proof obligation.

        CPU mj_step only. mujoco_warp keeps its own data path; XPBD has no
        mj_data at all. Escape hatch: OMNISIM_NEWTON_STALE_CARTESIAN=1 restores
        the old behaviour for A/B.
        """
        if sv is None or not getattr(sv, "use_mujoco_cpu", False):
            return
        if not hasattr(self, "_fresh_cart_off"):
            self._fresh_cart_off = bool(_os.environ.get("OMNISIM_NEWTON_STALE_CARTESIAN"))
        if self._fresh_cart_off:
            return
        mj = getattr(sv, "_mujoco", None)
        m, d = getattr(sv, "mj_model", None), getattr(sv, "mj_data", None)
        if mj is None or m is None or d is None:
            return
        _s1 = getattr(mj, "mj_step1", None)
        if _s1 is None:
            return
        _s1(m, d)

    def _mjc_clamp_needed(self):
        """Is any clampable joint actually outside its limits right now?

        Reads MuJoCo's own qpos/qvel -- the arrays the solver just integrated
        -- through the newton-joint -> mj-index maps the conversion kernel
        uses, so it inspects exactly the values the slow path would. Returns
        True (do the full clamp) whenever it cannot answer confidently, so a
        setup failure can only cost time, never enforcement.

        Verified equivalence on the 8-Husky world: |joint_q - qpos| 1.5e-05
        rad, |joint_qd - qvel| 1.2e-07 rad/s (OMNISIM_NEWTON_MJ_POSE_CHECK).
        """
        sv = self._mjc_solver()
        d = getattr(sv, "mj_data", None) if sv is not None else None
        if d is None or not getattr(sv, "use_mujoco_cpu", False):
            return True                      # mjwarp / no mj_data: slow path
        try:
            import numpy as _np
            if not hasattr(self, "_fastclamp"):
                mjq = getattr(sv, "mj_q_start", None)
                mjqd = getattr(sv, "mj_qd_start", None)
                if mjq is None or mjqd is None:
                    self._fastclamp = None
                else:
                    mjq, mjqd = mjq.numpy(), mjqd.numpy()
                    qs = self.model.joint_q_start.numpy()
                    qds = self.model.joint_qd_start.numpy()
                    qi, qdi, vlim, lo, hi = [], [], [], [], []
                    for _sl, _rl in self.slot_to_real_idx.items():
                        try:
                            spec = self.pending_revolutes[_sl]
                        except (IndexError, KeyError, TypeError):
                            continue
                        if not isinstance(spec, dict) or "velocity_limit" not in spec:
                            continue         # multi-DoF / fixed: slow path owns them
                        if _rl >= len(mjq) or _rl >= len(qs):
                            self._fastclamp = None
                            break
                        qi.append(int(mjq[_rl]))
                        qdi.append(int(mjqd[_rl]))
                        vlim.append(float(spec["velocity_limit"]))
                        lo.append(float(spec["limit_lower"]))
                        hi.append(float(spec["limit_upper"]))
                    else:
                        self._fastclamp = (
                            _np.asarray(qi, dtype=_np.intp), _np.asarray(qdi, dtype=_np.intp),
                            _np.asarray(vlim), _np.asarray(lo), _np.asarray(hi))
            fc = self._fastclamp
            if fc is None:
                return True
            qi, qdi, vlim, lo, hi = fc
            if len(qi) == 0:
                return False
            qpos, qvel = _np.asarray(d.qpos), _np.asarray(d.qvel)
            if qi.max(initial=-1) >= len(qpos) or qdi.max(initial=-1) >= len(qvel):
                return True
            q, qd = qpos[qi], qvel[qdi]
            if _np.any((vlim > 0.0) & (_np.abs(qd) > vlim)):
                return True
            ranged = lo != hi
            if _np.any(ranged & ((q < lo) | (q > hi))):
                return True
            return False
        except Exception:
            return True                      # never trade enforcement for speed

    def _contact_friction_probe(self):
        """OMNISIM_NEWTON_CONTACT_FRICTION_PROBE=1: print live contact params.

        VERIFICATION ONLY -- changes nothing. Exists because of a measured
        min(mu, ~1.0) clamp: geoms carried fric=[2.0,...] in the compiled
        mjModel (verified via DUMP_MJMODEL) yet a box slid down a 45-degree
        ramp as if mu were ~0.9. The geom dump cannot see what the CONTACT
        actually carries at solve time -- this can.
        """
        if not getattr(self, "_cfp_on", None):
            self._cfp_on = bool(_os.environ.get("OMNISIM_NEWTON_CONTACT_FRICTION_PROBE"))
            self._cfp_done = False
        if not self._cfp_on or self._cfp_done:
            return
        sv = self._mjc_solver()
        d = getattr(sv, "mj_data", None) if sv is not None else None
        if d is None or d.ncon == 0:
            return
        self._cfp_done = True
        lines = ["ncon=%d" % d.ncon]
        for i in range(min(int(d.ncon), 8)):
            c = d.contact[i]
            lines.append("contact %d geoms=%s dim=%d mu=%s solref=%s solreffriction=%s includemargin=%.4g" % (
                i, list(c.geom), int(c.dim),
                [round(float(v), 4) for v in c.friction],
                [round(float(v), 4) for v in c.solref],
                [round(float(v), 4) for v in c.solreffriction],
                float(c.includemargin)))
        try:
            with open(_os.environ.get("OMNISIM_NEWTON_CONTACT_FRICTION_PROBE_OUT",
                                      "contact_friction_probe.txt"), "w") as f:
                f.write("\n".join(lines) + "\n")
        except OSError:
            pass

    def _mj_pose_check(self):
        """Is mj_data.xpos/xquat identical to the body_q newton recomputes?

        VERIFICATION ONLY (OMNISIM_NEWTON_MJ_POSE_CHECK=1) -- changes nothing.

        _update_newton_state does not read poses out of MuJoCo: it converts
        joint coordinates and then runs eval_articulation_fk to REDERIVE body
        transforms that mj_step already computed into xpos/xquat. That is the
        bulk of its ~0.26 ms/step. Reading those arrays directly would skip
        both the coordinate conversion and the FK -- but only if newton's body
        frame is MuJoCo's body frame, and only if the quaternion order is
        handled (MuJoCo stores wxyz, warp transforms store xyzw).

        ⚠ ANSWERED, AND THE ANSWER IS NO -- xpos is ONE STEP STALE.
        mj_step is forward() then _advance(), and _advance updates ONLY qpos,
        qvel and time. Nothing recomputes the Cartesian arrays afterwards:
        newton's CPU step goes control -> mj_step -> _update_newton_state with
        no kinematics call. So on return, qpos is at t+dt while xpos / xquat /
        xipos / subtree_com / cvel are still at t.

        An earlier reading of this check (commit ee614170) concluded the
        opposite and it was WRONG. It sampled a SETTLED scene, where a time
        offset is invisible by construction -- there is no motion to lag -- and
        read the resulting 1.4e-08 m as proof of equality. Re-run on a MOVING
        scene (lane-1 T4, the 3-link pendulum) the same check reports a
        per-step deviation of 1.0e-02 to 1.2e-02 m, which is |v|*dt, not noise.
        The at-rest sample only ever proved there is no FRAME offset.

        Consequence for anyone optimising the readback: serving poses straight
        out of mj_data would report every body one tick late, everywhere,
        silently. The fix is not to abandon the idea but to use the documented
        split -- mj_step2 (integrate) then mj_step1 (recompute position and
        velocity kinematics at t+dt) -- which costs the same total work per
        tick and leaves the Cartesian arrays fresh.

        This measures that assumption instead of assuming it. It reports the
        worst position and quaternion deviation seen, so the answer is a number
        rather than a belief.
        """
        sv = self._mjc_solver()
        if sv is None or not getattr(sv, "use_mujoco_cpu", False):
            return
        d, m = getattr(sv, "mj_data", None), getattr(sv, "mjc_body_to_newton", None)
        if d is None or m is None:
            return
        try:
            import numpy as _np
            if not hasattr(self, "_mj_to_newton_np"):
                self._mj_to_newton_np = m.numpy()[0]        # world 0
                self._mj_pose_worst = (0.0, 0.0)
            bq = self.state_a.body_q.numpy()
            xpos, xquat = d.xpos, d.xquat
            # Tier 1c velocity term: check the cvel->body_qd derivation
            # the (now REMOVED) mj-direct fill used, against what the state
            # actually holds.
            # Meaningful under OMNISIM_NEWTON_MJ_DIRECT=0 (FK filled body_qd,
            # so a disagreement means the formula/convention is wrong); under
            # the direct path it degenerates to a self-check.
            # ⚠ SCOPE: valid only on ticks that took the BATCHED substep path.
            # _refresh_mj_cartesian runs only there, so on a per-substep-path
            # tick cvel is legitimately one tick stale and this term reads
            # |dv| ~= |a|*dt spikes (measured 1.816 m/s on the rest-height
            # ball's impact tick, profiled run 2026-08-08) that say nothing
            # about the formula. Verified on batched ticks the same day: max
            # 1.6e-07 over 766 ticks including impact (OMNISIM_NEWTON_VEL_TRACE
            # per-tick dump). That fill is gone as of 2026-08-09, so this term
            # is now purely a convention check on
            # non-batched ticks, so the spike does not indicate a served-value
            # error either.
            bqd = self.state_a.body_qd.numpy()
            _cv = getattr(d, "cvel", None)
            _xip = getattr(d, "xipos", None)
            _scom = getattr(d, "subtree_com", None)
            _rootid = _np.asarray(sv.mj_model.body_rootid) if getattr(sv, "mj_model", None) is not None else None
            worst_p, worst_q = self._mj_pose_worst
            dp_now = dq_now = 0.0
            for mj_id, nb in enumerate(self._mj_to_newton_np):
                nb = int(nb)
                # mj body 0 is ALWAYS the world body -- comparing it against a
                # real newton body is what made the first run of this check
                # report a growing millimetre "deviation" and read like a frame
                # offset. It was measuring the world against a falling box.
                if mj_id == 0 or nb < 0 or nb >= len(bq):
                    continue
                dp = float(_np.max(_np.abs(_np.asarray(xpos[mj_id]) - bq[nb][:3])))
                _xi = getattr(d, "xipos", None)
                if _xi is not None:
                    _dpi = float(_np.max(_np.abs(_np.asarray(_xi[mj_id]) - bq[nb][:3])))
                    self._mj_xipos_worst = max(getattr(self, "_mj_xipos_worst", 0.0), _dpi)
                # mujoco wxyz -> warp xyzw
                w, x, y, z = (float(v) for v in xquat[mj_id])
                dq = float(_np.max(_np.abs(_np.array([x, y, z, w]) - bq[nb][3:7])))
                worst_p, worst_q = max(worst_p, dp), max(worst_q, dq)
                dp_now, dq_now = max(dp_now, dp), max(dq_now, dq)
                if (_cv is not None and _xip is not None and _scom is not None
                        and _rootid is not None and nb < len(bqd)):
                    _w3 = _np.asarray(_cv[mj_id][0:3])
                    _vl = _np.asarray(_cv[mj_id][3:6])
                    _vcom = _vl + _np.cross(_w3, _np.asarray(_xip[mj_id])
                                            - _np.asarray(_scom[int(_rootid[mj_id])]))
                    _dva = float(_np.max(_np.abs(_w3 - bqd[nb][3:6])))
                    _dvl = float(_np.max(_np.abs(_vcom - bqd[nb][0:3])))
                    _dv = max(_dva, _dvl)
                    if _dv > getattr(self, "_mj_vel_worst", 0.0):
                        self._mj_vel_worst = _dv
                        self._mj_vel_worst_detail = (
                            "nb=%d ang %.3e lin %.3e | fk_qd=%s | w=%s v=%s vcom=%s"
                            % (nb, _dva, _dvl, _np.round(bqd[nb], 4).tolist(),
                               _np.round(_w3, 4).tolist(), _np.round(_vl, 4).tolist(),
                               _np.round(_vcom, 4).tolist()))
                    _tf = _os.environ.get("OMNISIM_NEWTON_VEL_TRACE")
                    if _tf and nb == 1:
                        try:
                            with open(_tf, "a") as _f:
                                _f.write("step=%s qvel=%s cvel=%s fk=%s\n" % (
                                    getattr(self, "_stepn", -1),
                                    _np.round(_np.asarray(d.qvel), 4).tolist(),
                                    _np.round(_np.asarray(_cv[mj_id]), 4).tolist(),
                                    _np.round(bqd[nb], 4).tolist()))
                        except OSError:
                            pass
            self._mj_pose_worst = (worst_p, worst_q)
            # ALSO report THIS step's deviation, not just the running max. The
            # max is dominated by the opening drop: at ~1 m/s and dt=4 ms one
            # step of motion is ~4 mm, so a one-step time offset in this check
            # and a genuine frame offset both show up as "a few mm". They are
            # distinguishable at rest -- a time offset goes to zero, a frame
            # offset does not.
            # Item 1 probe: can the joint-limit clamp read MuJoCo's qpos/qvel
            # directly instead of newton's joint_q/joint_qd? Only if the two
            # index into the same value for a revolute. mj_q_start/mj_qd_start
            # are the newton-joint -> mj-qpos/qvel maps the conversion kernel
            # itself uses, so this compares exactly what a re-source would read.
            try:
                _mjq = getattr(sv, "mj_q_start", None)
                _mjqd = getattr(sv, "mj_qd_start", None)
                if _mjq is not None and self.slot_to_real_idx:
                    if not hasattr(self, "_probe_mjq"):
                        self._probe_mjq = _mjq.numpy()
                        self._probe_mjqd = _mjqd.numpy()
                        self._probe_qs = self.model.joint_q_start.numpy()
                        self._probe_qds = self.model.joint_qd_start.numpy()
                    _jq = self.state_a.joint_q.numpy()
                    _jqd = self.state_a.joint_qd.numpy()
                    _qp, _qv = _np.asarray(d.qpos), _np.asarray(d.qvel)
                    _wq = _wqd = 0.0
                    for _sl, _rl in self.slot_to_real_idx.items():
                        try:
                            _spec = self.pending_revolutes[_sl]
                        except (IndexError, KeyError, TypeError):
                            continue
                        if not isinstance(_spec, dict) or "velocity_limit" not in _spec:
                            continue     # multi-DoF / fixed: not a scalar hinge
                        _ni, _mi = int(self._probe_qs[_rl]), int(self._probe_mjq[_rl])
                        _ndi, _mdi = int(self._probe_qds[_rl]), int(self._probe_mjqd[_rl])
                        if 0 <= _ni < len(_jq) and 0 <= _mi < len(_qp):
                            _wq = max(_wq, abs(float(_jq[_ni]) - float(_qp[_mi])))
                        if 0 <= _ndi < len(_jqd) and 0 <= _mdi < len(_qv):
                            _wqd = max(_wqd, abs(float(_jqd[_ndi]) - float(_qv[_mdi])))
                    self._mj_jointq_worst = max(getattr(self, "_mj_jointq_worst", 0.0), _wq)
                    self._mj_jointqd_worst = max(getattr(self, "_mj_jointqd_worst", 0.0), _wqd)
            except Exception as _je:
                self._mj_jointq_msg = repr(_je)[:120]
            # ⭐ THE SIGNATURE TEST (ee614170 -> ccaab865): a TIME offset
            # collapses when you compare against the PREVIOUS tick's body_q; a
            # FRAME offset does not. This is what tells "mj_data is one tick
            # stale here" apart from "newton's body frame is not mj's body
            # frame", which decides whether poses may ever be served from
            # mj_data on articulated robots.
            _prev = getattr(self, "_prev_bq", None)
            if _prev is not None and len(_prev) == len(bq):
                _dp_prev = 0.0
                for mj_id, nb in enumerate(self._mj_to_newton_np):
                    nb = int(nb)
                    if mj_id == 0 or nb < 0 or nb >= len(_prev):
                        continue
                    _dp_prev = max(_dp_prev, float(_np.max(_np.abs(
                        _np.asarray(xpos[mj_id]) - _prev[nb][:3]))))
                self._mj_prev_worst = max(getattr(self, "_mj_prev_worst", 0.0), _dp_prev)
                self._mj_prev_now = _dp_prev
            self._prev_bq = bq.copy()
            self._mj_pose_now = (dp_now, dq_now)
            self._mj_pose_msg = ("max |xpos-body_q| = %.3e m, max |xipos-body_q| = %.3e m, "
                                 "max |xquat-body_q| = %.3e | THIS STEP: pos %.3e quat %.3e"
                                 " | max |cvel-derived qd - body_qd| = %.3e [%s]"
                                 " | JOINT vs mj: q %.3e qd %.3e %s"
                                 " | vs PREV-tick body_q: max %.3e now %.3e"
                                 " | ticks batched=%d generic=%d"
                                 % (worst_p, getattr(self, "_mj_xipos_worst", float("nan")), worst_q,
                                    dp_now, dq_now, getattr(self, "_mj_vel_worst", float("nan")),
                                    getattr(self, "_mj_vel_worst_detail", ""),
                                    getattr(self, "_mj_jointq_worst", float("nan")),
                                    getattr(self, "_mj_jointqd_worst", float("nan")),
                                    getattr(self, "_mj_jointq_msg", ""),
                                    getattr(self, "_mj_prev_worst", float("nan")),
                                    getattr(self, "_mj_prev_now", float("nan")),
                                    getattr(self, "_n_batched", 0),
                                    getattr(self, "_n_generic", 0)))
        except Exception as _e:
            self._mj_pose_msg = "pose check failed: %r" % (_e,)

    def _prof_wrap_solver(self):
        # Time the two conversion halves of SolverMuJoCo.step (newton -> mjc
        # control in, mjc data -> newton state out) that bracket mj_step.
        # mj_step itself costs ~0.005 ms on these scenes while the whole
        # solver.step costs ~0.64, so the question is which half owns it.
        if getattr(self, "_prof_wrapped", False):
            return
        self._prof_wrapped = True
        self._prof_ctrl = 0.0
        self._prof_out = 0.0
        self._prof_mj = 0.0
        sv = self._mjc_solver()
        for name, slot in (("_apply_mjc_control", "_prof_ctrl"), ("_update_newton_state", "_prof_out")):
            fn = getattr(sv, name, None)
            if fn is None:
                continue

            def mk(fn=fn, slot=slot):
                def wrapped(*a, **kw):
                    _t = _perf()
                    try:
                        return fn(*a, **kw)
                    finally:
                        setattr(self, slot, getattr(self, slot) + (_perf() - _t))
                return wrapped
            setattr(sv, name, mk())
        mj = getattr(sv, "_mujoco", None)
        if mj is not None and hasattr(mj, "mj_step"):
            _orig = mj.mj_step

            def _timed_mj_step(*a, __o=_orig, **kw):
                _t = _perf()
                try:
                    return __o(*a, **kw)
                finally:
                    self._prof_mj += _perf() - _t
            sv._mujoco_mj_step_orig = _orig
            try:
                sv._mujoco = type("_MjShim", (), {"__getattr__": staticmethod(lambda n: getattr(_orig.__self__ if hasattr(_orig, "__self__") else mj, n))})()
                sv._mujoco.mj_step = _timed_mj_step
            except Exception:
                pass

    def _sstep(self, *a, **kw):
        if not self._prof_on():
            return self.solver.step(*a, **kw)
        self._prof_wrap_solver()
        if not getattr(self, "_prof_pre_done", True):
            # everything our step() did BEFORE handing off to the solver
            self._prof_pre += _perf() - self._step_t0
            self._prof_pre_done = True
        _t = _perf()
        r = self.solver.step(*a, **kw)
        self._prof_solve += _perf() - _t
        return r

    def _cloth_overflow_check(self):
        """Sample VBD's contact-buffer high-water marks and WARN on overflow.

        ⚠ THIS IS THE ONLY RELIABLE SIGNAL. VBD's contact buffers overflow
        silently in the sense that matters to us:
          * PARTICLE self-contact overflow is fully silent -- the kernels set a
            `resize_flags` entry that nothing ever reads, and consumption
            clamps with `wp.min`, so the extra contacts are simply dropped.
          * RIGID / body-particle overflow does warn, but only through a
            `wp.printf` from INSIDE a warp kernel. On Windows omnisim-bin is a
            GUI-subsystem binary whose stdout is discarded outright, and the
            embedded interpreter's stdout does not reach the host reliably
            anywhere else either (this is exactly why _newton_log writes a
            file). So that warning does not exist for us.
        The symptom of either is "the grip feels wrong" / "the fold went
        through itself" with nothing in any log -- the same shape as the
        njmax/nconmax cliff AGENTS.md documents for mujoco_warp. Reading
        `body_particle_contact_overflow_max` against the allocated size turns
        it into a number.

        Sampled every OMNISIM_CLOTH_OVERFLOW_INTERVAL steps (default 240 = a
        few seconds of sim time; 0 disables) because it is a GPU->host readback.
        It runs OUTSIDE the substep loop, so it never lands inside a CUDA-graph
        capture. Warns once per world, then stays quiet.
        """
        n = self._cloth_env_int("OMNISIM_CLOTH_OVERFLOW_INTERVAL", 240)
        if n <= 0 or getattr(self, "_cloth_overflow_warned", False):
            return
        if int(getattr(self, "_stepn", 0)) % n:
            return
        sv = self.solver_soft
        try:
            for attr, cap_attr, what in (
                    ("body_particle_contact_overflow_max",
                     "body_particle_contact_buffer_pre_alloc", "body-particle"),
                    ("body_body_contact_overflow_max",
                     "body_body_contact_buffer_pre_alloc", "body-body")):
                arr = getattr(sv, attr, None)
                if arr is None:
                    continue
                peak = int(arr.numpy()[0])
                cap = int(getattr(sv, cap_attr, 0) or 0)
                if cap and peak > cap:
                    self._cloth_overflow_warned = True
                    self._newton_log(
                        "cloth: ⚠ %s CONTACT BUFFER OVERFLOW -- peak %d vs "
                        "capacity %d at step %s. Contacts past the cap are "
                        "DROPPED, so the grip/drape is wrong and nothing else "
                        "will tell you. Raise "
                        "OMNISIM_CLOTH_RIGID_PARTICLE_BUFFER above %d."
                        % (what, peak, cap, getattr(self, "_stepn", "?"), peak))
        except Exception as _e:                   # noqa: BLE001
            # Never let telemetry break a tick; report once and stop trying.
            self._cloth_overflow_warned = True
            self._newton_log("cloth: overflow telemetry unavailable: %r" % (_e,))

    def _cloth_graph_ok(self):
        """May the cloth path arm the CUDA-graph capture? Usually NO.

        MEASURED, first run of the coupled solver on this machine (RTX 3060
        laptop, newton 1.5.0): arming the capture on a cloth world whose "mjc"
        entry is the CPU ``mj_step`` solver aborts it with

            Warp CUDA error 906: operation would make the legacy stream depend
            on a capturing blocking stream (wp_cuda_context_set_stream)
            Warp CUDA error 906: ... (wp_memcpy_d2h)

        and the reason is structural, not a bug to chase: ``use_mujoco_cpu``
        means the solver copies state GPU->HOST, runs mj_step on the CPU, and
        copies back, EVERY substep. A device-to-host memcpy cannot be recorded
        into a CUDA graph, so the capture can only ever fail. The existing
        failure handling caught it and fell back to the direct path, so the run
        was CORRECT -- but it printed two CUDA errors per world and paid a
        pointless capture attempt, and "it errors and then works" is exactly
        the shape that gets mis-diagnosed later.

        Refuse up front and say why once. The GPU ``mujoco_warp`` path has no
        host round trip and is left free to graph -- that is the configuration
        the 164 fps figure belongs to; a CPU-mjc cloth world still gets the
        device pin's win (CUDA VBD vs CPU VBD) without the graph on top.
        ``OMNISIM_CLOTH_FORCE_GRAPH=1`` re-arms it to re-measure the claim.
        """
        if self.solver_soft is None:
            return True                       # rigid world: unchanged predicate
        if self._cloth_env_flag("OMNISIM_CLOTH_FORCE_GRAPH", False):
            return True
        mjc = getattr(self, "solver_mjc", None)
        if mjc is not None and getattr(mjc, "use_mujoco_cpu", False):
            if not getattr(self, "_cloth_graph_note", False):
                self._cloth_graph_note = True
                self._newton_log(
                    "cloth: CUDA-graph capture DISABLED -- the mjc entry is the "
                    "CPU mj_step solver and its per-substep GPU<->host copy "
                    "cannot be recorded into a graph (warp error 906). Pin "
                    "WorldInfo.newtonSolver \"mujoco_warp\" to graph the tick, "
                    "or OMNISIM_CLOTH_FORCE_GRAPH=1 to re-measure.")
            return False
        return True

    def _collide(self, state, contacts):
        """Run narrow phase into `contacts`.

        Rigid worlds keep the historical route -- `Model.collide()`, which
        lazily builds and caches newton's DEFAULT pipeline. Cloth worlds must
        NOT take it: their pipeline is constructed explicitly with a soft
        contact margin and the full-surface rigid-soft pass, and both of those
        are fixed at ALLOCATION time (collide.py:825 -- the flag sizes the
        soft-contact buffer). Model.collide() would quietly hand back the
        default pipeline instead, whose soft-contact buffer has no edge/face
        headroom, and the cloth would lose exactly the contacts it was
        configured to catch.
        """
        p = self.collision_pipeline
        if p is not None:
            return p.collide(state, contacts)
        return self.model.collide(state, contacts)

    def _collide_prof(self, *a, **kw):
        if not self._prof_on():
            return self._collide(*a, **kw)
        _t = _perf()
        r = self._collide(*a, **kw)
        self._prof_collide += _perf() - _t
        return r

    def _clamp_velocity_servo_gains(self, dt):
        """Bound each pure-velocity motor's gain by the joint's OWN inertia.

        ⚠ WHY. A motorised hinge is built with ``target_kd = 500`` -- a
        constant hardcoded in ``OmBasicJoint`` and chosen for XPBD (see this
        file's header: "Newton's official velocity-control test uses
        target_ke=0, target_kd=500 -- the wheel literally does not spin with
        kd=1"). XPBD was REMOVED on 2026-08-07 and the constant stayed. Newton's
        MuJoCo converter turns it into a ``velocity`` actuator with kv = 500
        N.m.s/rad, and MuJoCo's implicit integrators solve

            (M + dt * D) qacc = qfrc,     D carries the actuator's d(force)/d(qvel) = -kv

        so the joint's APPARENT inertia in every acceleration solve becomes
        ``M + dt*kv``.  On the ladder0 rung-4 rover (wheel M = 0.001354 kg m^2,
        dt = 4 ms) that is 0.001354 + 2.0 -- **1478x the wheel's real inertia**.

        MEASURED consequences on that scene, all on the engine's own compiled
        mjModel replayed in bare MuJoCo 3.8.1 (so none of this is our stepping
        loop):

        * free in the air, no contact at all, a wheel commanded 4 rad/s with
          its 20 N.m torque limit gains **0.04 rad/s per step** where its own
          inertia allows 59.1 -- it takes 93 steps to reach 90% of the command
          instead of ~1;
        * during those 93 steps the wheel is a near-rigid, near-infinitely-heavy
          body in the contact solve, so contact impulses are spent throwing the
          5 kg chassis instead of spinning the wheel: the rover pogos (38 mm of
          axle bounce) and peaks at **4.116 m/s -- 10.29x its 0.400 m/s rolling
          speed**, which no wheel can do.

        Falsified three ways, each isolating the ``dt*kv/M`` group:
        pre-spinning the wheels to the commanded speed drops the overrun to
        1.12x at the SAME kv; giving the joint real armature equal to dt*kv
        (2.0) drops it to 1.00x; and halving dt halves the overrun
        (10.29 / 5.32 / 2.65 / 1.41 at dt = 4 / 2 / 1 / 0.5 ms). The friction
        cone is NOT involved (elliptic scores 9.12x).

        THE BOUND. Requiring ``dt*kv <= M`` caps the inertia inflation at 2x.
        This is the same value the un-tuned MuJoCo reference arm picks for the
        same scene ("kv = I/dt, half the explicit-Euler stability bound 2I/dt")
        and it costs nothing in tracking: at the bound the servo halves its
        velocity error every step, so it reaches the command in ~5 steps rather
        than 93.

        SCOPE, deliberately narrow. Only joints in PURE VELOCITY mode
        (target_ke == 0, i.e. the wheel population the 500 was written for) are
        touched, and only ever DOWNWARD. A position-mode joint's kd is a
        damping term inside a tuned PD, so every legged / RL / grasp
        configuration is left bit-identical -- as is any joint whose gain the
        operator set explicitly through OMNISIM_NEWTON_TARGET_KD /
        _FINGER_KD / _ARM_KD. ``OMNISIM_NEWTON_VELOCITY_GAIN_CLAMP=0`` is the
        exact-revert hatch (verified: it reproduces the 9.62x overrun exactly).

        REACHES BOTH SOLVER PATHS since 2026-08-13. It was CPU-``mj_step``-only
        when first written, and that gap was itself a measurable defect: on
        ``mujoco_warp`` the rung-6 rover commanded to a full stop travelled
        **1.09 m BACKWARDS**, which is the identical distance the CPU arm travels
        with this clamp switched off. See the scope comment below for the three
        numbers that tie them together.
        """
        if getattr(self, "_kv_clamped", False):
            return
        self._kv_clamped = True
        if _os.environ.get("OMNISIM_NEWTON_VELOCITY_GAIN_CLAMP",
                           "1").strip().lower() in ("0", "false", "off", "no"):
            return
        # An explicitly commanded gain is the operator's number, not ours.
        for _k in ("OMNISIM_NEWTON_TARGET_KD", "OMNISIM_NEWTON_FINGER_KD",
                   "OMNISIM_NEWTON_ARM_KD"):
            if _os.environ.get(_k) not in (None, ""):
                self._newton_log("[OmNewtonBackend] velocity-gain clamp "
                                 "skipped: %s is set explicitly" % _k)
                return
        sv = self._mjc_solver()
        # SCOPE: BOTH solver paths since 2026-08-13. It used to be CPU mj_step
        # only, on the reasoning that `mujoco_warp` bakes the gains into the GPU
        # model at construction and a post-hoc mj_model edit cannot reach the
        # kernels. The first half is true; the conclusion was not. SolverMuJoCo
        # builds mj_model/mj_data on BOTH paths (it compiles the spec, then
        # put_model()s it), so the BOUND is computable either way -- what had to
        # change is only WHERE the clamped number is written: mj_model for CPU,
        # and mjw_model.actuator_{gain,bias}prm for the GPU, the same dual-write
        # the per-world condim and roll-mu knobs already use. A wp.array assign()
        # writes the existing buffer in place, so a captured CUDA graph sees it.
        #
        # WHAT THE GAP COST, measured on the ladder0 rung-6 rover (drive at a
        # wall, stop below a threshold) with the SAME binary:
        #     mujoco_warp, shipped          stop_gap 1.59025 m, min_gap 0.49792
        #     CPU mj_step, shipped          stop_gap 0.48473 m  (== min_gap)
        #     CPU mj_step, CLAMP=0          stop_gap 1.59024 m, min_gap 0.49792
        # i.e. commanding the wheels to ZERO threw the rover 1.09 m BACKWARDS on
        # the GPU path, and REVERTING THE CPU FIX reproduces the GPU number to
        # five decimals. That is what makes them one defect rather than two that
        # look alike: the CPU arm with the clamp off is not merely also bad, it
        # is bad by the same 1.0923 m. Confirmed from the other side too --
        # mujoco_warp with the gain set to this clamp's own bound (M/dt) stops in
        # 0.47751 m with zero overrun, all six rung-6 checks green.
        if not getattr(sv, "use_mujoco_cpu", False):
            self._kv_target_warp = True
        m = getattr(sv, "mj_model", None)
        d = getattr(sv, "mj_data", None)
        mj = getattr(sv, "_mujoco", None)
        if m is None or d is None or mj is None or dt <= 0.0:
            return
        try:
            import numpy as _np
            # qM only: kinematics -> com -> mass matrix. Deliberately NOT
            # mj_forward, which would also touch the constraint and warm-start
            # state this must not perturb before the first step.
            # ⚠ mj_crb stopped filling d.qM in MuJoCo 3.8 (mj_makeM does);
            # measured on 3.8.1, the crb-only sequence returns an ALL-ZERO
            # mass matrix, which would read as "no inertia" and clamp every
            # gain to zero -- i.e. a robot that silently does not drive. Hence
            # both the version probe and the positive-inertia guard below.
            mj.mj_kinematics(m, d)
            mj.mj_comPos(m, d)
            (getattr(mj, "mj_makeM", None) or mj.mj_crb)(m, d)
            full = _mj_full_mass(mj, m, d, m.nv, _np)
            if not _np.any(_np.diag(full) > 0.0):
                self._newton_log("[OmNewtonBackend] velocity-gain clamp "
                                 "skipped: mass matrix read all-zero")
                return
            # A joint carries a POSITION servo when any actuator on it has a
            # non-zero length coefficient in its affine bias (biasprm[1] = -kp).
            has_position = set()
            vel_actuators = []
            for a in range(m.nu):
                jid = int(m.actuator_trnid[a][0])
                if jid < 0:
                    continue
                kp = float(m.actuator_biasprm[a][1])
                kv = float(m.actuator_biasprm[a][2])
                if kp != 0.0:
                    has_position.add(jid)
                if kv < 0.0 and float(m.actuator_gainprm[a][0]) > 0.0:
                    vel_actuators.append((a, jid))
            clamped = []
            for a, jid in vel_actuators:
                if jid in has_position:
                    continue                       # tuned PD -- not ours
                dof = int(m.jnt_dofadr[jid])
                if not (0 <= dof < m.nv):
                    continue
                inertia = float(full[dof][dof])
                if inertia <= 0.0:
                    continue
                kv_max = inertia / dt
                kv = float(m.actuator_gainprm[a][0])
                if kv <= kv_max:
                    continue
                m.actuator_gainprm[a][0] = kv_max
                m.actuator_biasprm[a][2] = -kv_max
                clamped.append((jid, kv, kv_max, inertia, a))
            # THE GPU HALF. mj_model is the model mujoco_warp was BUILT from,
            # not the one it steps, so on that path the loop above has changed
            # nothing the kernels will ever read. Push the same numbers into the
            # live device arrays. Layout is (nworld_or_1, nu) of vec10f, and the
            # leading axis is written whole so a batched world set gets it too.
            #
            # ⚠ THIS BRANCH MUST NEVER FAIL QUIETLY. Everything above is
            # bookkeeping until this write lands, so a silent failure here would
            # leave a mujoco_warp world with the un-clamped gain while the log
            # said "clamped" -- worse than the gap it replaces, because the gap
            # at least announced itself. Any failure is reported AS a failure and
            # the clamp is declared not applied.
            warp_applied = None
            if clamped and getattr(self, "_kv_target_warp", False):
                wm = getattr(sv, "mjw_model", None)
                if wm is None:
                    warp_applied = "mjw_model absent"
                else:
                    try:
                        g = wm.actuator_gainprm.numpy()
                        b = wm.actuator_biasprm.numpy()
                        for _jid, _old, _new, _I, _a in clamped:
                            g[:, _a, 0] = _new
                            b[:, _a, 2] = -_new
                        wm.actuator_gainprm.assign(g)
                        wm.actuator_biasprm.assign(b)
                        warp_applied = "ok"
                    except Exception as _we:           # noqa: BLE001
                        warp_applied = repr(_we)[:160]
            if clamped:
                lo = min(c[2] for c in clamped)
                hi = max(c[2] for c in clamped)
                where = ("mj_model (cpu/mj_step)" if warp_applied is None
                         else "mjw_model (mujoco_warp): %s" % warp_applied)
                self._newton_log(
                    "[OmNewtonBackend] velocity-servo gain clamped on %d of %d "
                    "velocity-mode joints: kd %.4g -> %.4g..%.4g "
                    "(bound = joint inertia / dt, dt=%.4g s; the implicit "
                    "integrator inflates a joint's apparent inertia to "
                    "M + dt*kd) -> %s" % (len(clamped), len(vel_actuators),
                                          clamped[0][1], lo, hi, dt, where))
                if warp_applied not in (None, "ok"):
                    self._newton_log(
                        "[OmNewtonBackend] WARNING: velocity-servo gain clamp "
                        "did NOT reach the GPU model (%s) -- this mujoco_warp "
                        "world is running the UN-CLAMPED gain, and a joint "
                        "commanded to zero can throw its body. Re-run on "
                        "newtonSolver \"mujoco\" until this is fixed."
                        % warp_applied)
        except Exception as exc:                   # noqa: BLE001
            self._newton_log("[OmNewtonBackend] velocity-gain clamp skipped: "
                             "%r" % (exc,))

    def _launch_debug(self, path):
        """Append one JSON line per step: the whole mj state + contacts.

        Gated on OMNISIM_DEBUG_LAUNCH=<file>; inert when unset. Unlike
        OMNISIM_DEBUG_JOINTS (which OVERWRITES, so only the last step
        survives) this is a HISTORY -- a launch transient is only visible
        as a time series.
        """
        try:
            sv = self._mjc_solver()
            d = getattr(sv, "mj_data", None)
            m = getattr(sv, "mj_model", None)
            if d is None or m is None:
                return
            n = getattr(self, "_ldbg_n", 0)
            self._ldbg_n = n + 1
            rec = {
                "step": n,
                "time": float(d.time),
                "qpos": [round(float(v), 6) for v in d.qpos],
                "qvel": [round(float(v), 6) for v in d.qvel],
                "ctrl": [round(float(v), 6) for v in d.ctrl],
                "qfrc_actuator": [round(float(v), 4) for v in d.qfrc_actuator],
                "qfrc_constraint": [round(float(v), 4)
                                    for v in d.qfrc_constraint],
                "ncon": int(d.ncon),
                "pre": getattr(self, "_ldbg_pre", None),
            }
            self._ldbg_pre = None
            if n == 0:
                rec["module"] = __file__
                try:
                    getattr(sv, "_mujoco").mj_saveModel(m, path + ".mjb", None)
                    rec["mjb"] = path + ".mjb"
                except Exception as _me:           # noqa: BLE001
                    rec["mjb_err"] = repr(_me)
                rec["njnt"] = int(m.njnt)
                rec["jnt_type"] = [int(v) for v in m.jnt_type]
                rec["jnt_bodyid"] = [int(v) for v in m.jnt_bodyid]
                rec["jnt_axis"] = [[round(float(c), 4) for c in a]
                                   for a in m.jnt_axis]
                rec["jnt_qposadr"] = [int(v) for v in m.jnt_qposadr]
                rec["jnt_dofadr"] = [int(v) for v in m.jnt_dofadr]
                rec["nu"] = int(m.nu)
                rec["actuator_trnid"] = [[int(c) for c in a]
                                         for a in m.actuator_trnid]
                rec["actuator_gainprm"] = [round(float(a[0]), 4)
                                           for a in m.actuator_gainprm]
                rec["actuator_biasprm"] = [[round(float(c), 4) for c in a[:3]]
                                           for a in m.actuator_biasprm]
                rec["actuator_ctrlrange"] = [[round(float(c), 4) for c in a]
                                             for a in m.actuator_ctrlrange]
                rec["actuator_forcerange"] = [[round(float(c), 4) for c in a]
                                              for a in m.actuator_forcerange]
                rec["body_mass"] = [round(float(v), 6) for v in m.body_mass]
                rec["geom_type"] = [int(v) for v in m.geom_type]
                rec["geom_size"] = [[round(float(c), 5) for c in a]
                                    for a in m.geom_size]
                rec["geom_bodyid"] = [int(v) for v in m.geom_bodyid]
                rec["dof_damping"] = [round(float(v), 5) for v in m.dof_damping]
                rec["dof_armature"] = [round(float(v), 6)
                                       for v in m.dof_armature]
                rec["geom_solref"] = [[round(float(c), 6) for c in a]
                                      for a in m.geom_solref]
                rec["geom_solimp"] = [[round(float(c), 6) for c in a]
                                      for a in m.geom_solimp]
                rec["geom_friction"] = [[round(float(c), 6) for c in a]
                                        for a in m.geom_friction]
                rec["geom_condim"] = [int(v) for v in m.geom_condim]
                rec["geom_margin"] = [round(float(v), 6) for v in m.geom_margin]
                rec["geom_gap"] = [round(float(v), 6) for v in m.geom_gap]
                rec["geom_priority"] = [int(v) for v in m.geom_priority]
                rec["body_inertia"] = [[round(float(c), 8) for c in a]
                                       for a in m.body_inertia]
                rec["body_ipos"] = [[round(float(c), 6) for c in a]
                                    for a in m.body_ipos]
                rec["opt"] = {
                    "timestep": float(m.opt.timestep),
                    "iterations": int(m.opt.iterations),
                    "ls_iterations": int(m.opt.ls_iterations),
                    "solver": int(m.opt.solver),
                    "integrator": int(m.opt.integrator),
                    "cone": int(m.opt.cone),
                    "impratio": float(m.opt.impratio),
                    # The noslip pass changes what a friction contact DOES, so
                    # a dump that omits it cannot explain why two runs of the
                    # same world disagree about whether a grasp held.
                    "noslip_iterations": int(m.opt.noslip_iterations),
                    "noslip_tolerance": float(m.opt.noslip_tolerance),
                    "gravity": [float(v) for v in m.opt.gravity],
                    "o_solref": [float(v) for v in m.opt.o_solref],
                    "o_solimp": [float(v) for v in m.opt.o_solimp],
                    "o_margin": float(m.opt.o_margin),
                    "tolerance": float(m.opt.tolerance),
                    "enableflags": int(m.opt.enableflags),
                    "disableflags": int(m.opt.disableflags),
                }
            cons = []
            _mj = getattr(sv, "_mujoco", None)
            _fbuf = None
            if _mj is not None:
                try:
                    import numpy as _np
                    _fbuf = _np.zeros(6, dtype=float)
                except Exception:                  # noqa: BLE001
                    _fbuf = None
            for i in range(min(int(d.ncon), 24)):
                c = d.contact[i]
                e = {
                    "g": [int(c.geom1), int(c.geom2)],
                    "dist": round(float(c.dist), 6),
                    "pos": [round(float(v), 5) for v in c.pos],
                    "dim": int(c.dim),
                    "fric": [round(float(v), 4) for v in c.friction],
                    "solref": [round(float(v), 6) for v in c.solref],
                    "solimp": [round(float(v), 5) for v in c.solimp],
                    "frame": [round(float(v), 4) for v in c.frame],
                }
                if _fbuf is not None:
                    try:
                        _mj.mj_contactForce(m, d, i, _fbuf)
                        e["force"] = [round(float(v), 3) for v in _fbuf]
                    except Exception:              # noqa: BLE001
                        pass
                cons.append(e)
            rec["contact"] = cons
            with open(path, "a") as fh:
                fh.write(_json.dumps(rec) + "\n")
        except Exception as exc:                       # noqa: BLE001
            try:
                with open(path, "a") as fh:
                    fh.write('{"err": %s}\n' % _json.dumps(repr(exc)))
            except OSError:
                pass

    def step(self, dt):
        if not self._prof_on():
            r = self._step_impl(dt)
        else:
            _t = _perf()
            self._step_t0 = _t
            self._prof_pre_done = False
            r = self._step_impl(dt)
            self._prof_py += _perf() - _t
        if _LAUNCH_DBG:
            self._launch_debug(_LAUNCH_DBG)
        if self._cloth_attach_on():
            self._cloth_attach_tick()
        if self._cloth_tlm_on():
            self._cloth_telemetry()
        return r

    def _cloth_tlm_on(self):
        # Cheap per-step gate. Resolved ONCE and cached, because this sits in
        # the step hot path and an os.environ lookup per tick is exactly the
        # kind of API-layer cost the step-cost campaign found dominating the
        # named computation it was blamed on.
        g = getattr(self, "_cloth_tlm_path", 0)
        if g == 0:
            import os as _o
            g = _o.environ.get("OMNISIM_CLOTH_TELEMETRY") or None
            self._cloth_tlm_path = g
            self._cloth_tlm_every = max(1, int(
                _o.environ.get("OMNISIM_CLOTH_TELEMETRY_EVERY") or 25))
            self._cloth_tlm_full = bool(_o.environ.get("OMNISIM_CLOTH_TELEMETRY_FULL"))
            self._cloth_tlm_n = 0
        return g is not None and self.has_cloth()

    def _cloth_telemetry(self):
        # WHY THIS EXISTS. Cloth was, until this function, UNOBSERVABLE from
        # outside the engine: particle positions are read back only for the
        # renderer, there is no supervisor accessor and no HTTP endpoint, so a
        # controller could drive a gripper into a sheet and have no way to tell
        # a grasp from a miss. That is disqualifying under this tree's standing
        # rule that a grasp must be proven GEOMETRICALLY -- "it looked right in
        # the viewport" is not evidence, and a contact read is a weaker claim
        # than a lifted part.
        #
        # It writes JSONL (one self-describing record per sample) rather than a
        # binary blob so a failed run is still readable with a text editor, and
        # it is OFF unless OMNISIM_CLOTH_TELEMETRY names a path -- this is a
        # measurement instrument, not a feature, and it must cost exactly zero
        # in a run that did not ask for it.
        self._cloth_tlm_n += 1
        if (self._cloth_tlm_n - 1) % self._cloth_tlm_every:
            return
        try:
            import numpy as _np, json as _json
            pq = getattr(self.state_a, "particle_q", None)
            if pq is None:
                return
            lo = self.cloth_particle_start if self.cloth_particle_start >= 0 else 0
            hi = self.cloth_particle_end if self.cloth_particle_end >= 0 else 0
            a = _np.asarray(pq.numpy())[lo:hi, 0:3]
            if a.size == 0:
                return
            rec = {
                "step": self._cloth_tlm_n - 1,
                "n": int(a.shape[0]),
                "centroid": [round(float(v), 6) for v in a.mean(axis=0)],
                "bbox_min": [round(float(v), 6) for v in a.min(axis=0)],
                "bbox_max": [round(float(v), 6) for v in a.max(axis=0)],
            }
            # NaN is the failure mode this stack actually exhibits (the coupled
            # proxy diverges rather than erroring), so report it as a COUNT
            # instead of letting it poison the aggregates silently.
            nn = int(_np.count_nonzero(~_np.isfinite(a)))
            if nn:
                rec["nonfinite"] = nn
            # ⚠ THE FIRST NUMBER TO LOOK AT WHEN CLOTH GOES THROUGH SOMETHING.
            # Particle-vs-rigid contact is generated by the CollisionPipeline
            # into the soft_contact_* arrays and consumed by SolverVBD. A sheet
            # that falls through a floor with soft_contacts == 0 is a CONTACT
            # GENERATION failure (flags, visibility, pipeline), which is a
            # completely different bug from a sheet that falls through with
            # contacts > 0 (too soft, too few substeps, tunnelling). Without
            # this the two are indistinguishable from the trajectory alone,
            # and they have opposite fixes.
            try:
                c = getattr(self, "_contacts_cache", None)
                if c is not None:
                    sc = getattr(c, "soft_contact_count", None)
                    if sc is not None:
                        rec["soft_contacts"] = int(_np.asarray(sc.numpy()).reshape(-1)[0])
            except Exception:                        # noqa: BLE001
                pass
            if self._cloth_tlm_full:
                rec["q"] = [[round(float(c), 6) for c in p] for p in a]
            with open(self._cloth_tlm_path, "a", encoding="utf-8") as fh:
                fh.write(_json.dumps(rec) + "\n")
        except Exception as e:                       # noqa: BLE001
            # Telemetry must never take down a run it was only observing.
            self._newton_log("cloth telemetry: disabled after error: %s" % (e,))
            self._cloth_tlm_path = None

    # ---- CLOTH PARTICLE ATTACH (kinematic grab) -------------------------
    # WHY THIS EXISTS. The fold campaign proved -- 12 laptop configurations,
    # 8 cloud seeds with valid negative controls, one genuine 5.5 mm pinch
    # that still shed under lift -- that SolverVBD cannot hold cloth by
    # FRICTION: its contact friction is velocity-regularised
    # (friction_epsilon), so the tangential force vanishes at zero slip and a
    # static pinch creeps out under gravity, and its drives ignore effort
    # limits so there is no squeeze budget to trade. (Upstream's own Franka
    # cloth example DOES friction-grasp -- it survives because its scripted
    # lift is short relative to the creep rate u* ~ W*eps_u/(2*mu*N); a fold
    # with a 0.5 m traverse is not.) So: a grab PINS the nearest cloth
    # particles to a rigid body (the pad), release un-pins them.
    # The pin is the exact runtime form of build-time fixed particles:
    # both particle_mass and particle_inv_mass go to 0 -- VBD's forward_step
    # gates on inv_mass == 0 and its local solve gates on mass == 0, so BOTH
    # must be zeroed or the solve kernel keeps moving the vertex.
    #
    # Wire protocol (controller-driveable without a new IPC surface): the
    # engine env names a command file in OMNISIM_CLOTH_ATTACH_CMD; the
    # controller (which inherits the engine environment) appends one JSON
    # line per command and the runtime consumes new complete lines each tick:
    #   {"op":"attach","body":6,"point":[x,y,z],"radius":0.035,"max":0}
    #   {"op":"detach"}
    # Every applied command is ACKed by appending {"seq","op","attached"} to
    # <cmd>.ack, so the controller can VERIFY the grab count instead of
    # believing its own request -- a grab that selected 0 particles must be
    # reported as 0, never assumed. A new attach implicitly releases the
    # previous group.
    def _cloth_attach_on(self):
        g = getattr(self, "_cloth_attach_path", 0)
        if g == 0:
            import os as _o
            g = _o.environ.get("OMNISIM_CLOTH_ATTACH_CMD") or None
            self._cloth_attach_path = g
            self._cloth_attach_ofs = 0
            self._cloth_attach_seq = 0
            self._cloth_attach_group = None
        return g is not None and self.has_cloth()

    @staticmethod
    def _quat_to_mat(q):
        import numpy as _np
        x, y, z, w = (float(v) for v in q)
        n = (x * x + y * y + z * z + w * w) or 1.0
        s = 2.0 / n
        return _np.array([
            [1 - s * (y * y + z * z), s * (x * y - w * z), s * (x * z + w * y)],
            [s * (x * y + w * z), 1 - s * (x * x + z * z), s * (y * z - w * x)],
            [s * (x * z - w * y), s * (y * z + w * x), 1 - s * (x * x + y * y)],
        ])

    def _cloth_attach_apply(self, cmd):
        import numpy as _np
        op = cmd.get("op")
        if op == "detach":
            g = self._cloth_attach_group
            n = 0
            if g is not None:
                m = self.model.particle_mass.numpy()
                im = self.model.particle_inv_mass.numpy()
                m[g["idx"]] = g["mass"]
                im[g["idx"]] = g["inv"]
                self.model.particle_mass.assign(m)
                self.model.particle_inv_mass.assign(im)
                # Released particles resume from rest: qd was held at zero
                # while pinned, which is also the honest release velocity for
                # a place that happens at the bottom of a lower phase.
                n = int(g["idx"].size)
                self._cloth_attach_group = None
            return {"op": "detach", "released": n}
        if op != "attach":
            return {"op": str(op), "error": "unknown op"}
        if self._cloth_attach_group is not None:
            self._cloth_attach_apply({"op": "detach"})
        body = int(cmd.get("body", -1))
        point = _np.asarray(cmd.get("point", ()), dtype=float).reshape(-1)
        radius = float(cmd.get("radius", 0.035))
        if body < 0 or point.size != 3:
            return {"op": "attach", "attached": 0, "error": "need body + point[3]"}
        pq = _np.asarray(self.state_a.particle_q.numpy())[:, 0:3]
        lo = self.cloth_particle_start if self.cloth_particle_start >= 0 else 0
        hi = self.cloth_particle_end if self.cloth_particle_end >= 0 else pq.shape[0]
        seg = pq[lo:hi]
        d = _np.linalg.norm(seg - point[None, :], axis=1)
        sel = _np.nonzero(d <= radius)[0]
        cap = int(cmd.get("max", 0))
        if cap > 0 and sel.size > cap:
            sel = sel[_np.argsort(d[sel])[:cap]]
        idx = (sel + lo).astype(_np.int64)
        if idx.size == 0:
            # A miss is a RESULT (the negative control depends on it), not an
            # error: ack attached=0 and pin nothing.
            return {"op": "attach", "attached": 0, "body": body,
                    "point": [round(float(v), 4) for v in point]}
        bq = _np.asarray(self.state_a.body_q.numpy())[body]
        R = self._quat_to_mat(bq[3:7])
        m = self.model.particle_mass.numpy()
        im = self.model.particle_inv_mass.numpy()
        self._cloth_attach_group = {
            "body": body,
            "idx": idx,
            # local = R^T (p - t), stored as rows: (p - t) @ R
            "local": (pq[idx] - bq[0:3][None, :]) @ R,
            "mass": m[idx].copy(),
            "inv": im[idx].copy(),
        }
        m[idx] = 0.0
        im[idx] = 0.0
        self.model.particle_mass.assign(m)
        self.model.particle_inv_mass.assign(im)
        return {"op": "attach", "attached": int(idx.size), "body": body,
                "point": [round(float(v), 4) for v in point],
                "radius": radius}

    def _cloth_attach_tick(self):
        try:
            import json as _json
            import os as _o
            import numpy as _np
            p = self._cloth_attach_path
            try:
                sz = _o.path.getsize(p)
            except OSError:
                sz = 0
            if sz > self._cloth_attach_ofs:
                with open(p, "rb") as fh:
                    fh.seek(self._cloth_attach_ofs)
                    chunk = fh.read()
                # Consume only COMPLETE lines: the controller writes
                # line-buffered, but a torn read must not half-parse.
                nl = chunk.rfind(b"\n")
                if nl >= 0:
                    self._cloth_attach_ofs += nl + 1
                    for raw in chunk[:nl].splitlines():
                        raw = raw.strip()
                        if not raw:
                            continue
                        try:
                            ack = self._cloth_attach_apply(_json.loads(raw))
                        except Exception as _e:              # noqa: BLE001
                            ack = {"error": repr(_e), "raw": raw[:200].decode(
                                "utf-8", "replace")}
                        self._cloth_attach_seq += 1
                        ack["seq"] = self._cloth_attach_seq
                        self._newton_log("cloth attach: %s" % (ack,))
                        try:
                            with open(p + ".ack", "a", encoding="utf-8") as af:
                                af.write(_json.dumps(ack) + "\n")
                        except OSError:
                            pass
            g = self._cloth_attach_group
            if g is not None:
                bq = _np.asarray(self.state_a.body_q.numpy())[g["body"]]
                R = self._quat_to_mat(bq[3:7])
                world = g["local"] @ R.T + bq[0:3][None, :]
                arr = _np.asarray(self.state_a.particle_q.numpy())
                arr[g["idx"], 0:3] = world
                self.state_a.particle_q.assign(arr)
                qd = _np.asarray(self.state_a.particle_qd.numpy())
                qd[g["idx"], 0:3] = 0.0
                self.state_a.particle_qd.assign(qd)
        except Exception as e:                               # noqa: BLE001
            # The grab channel must never take down the run it serves.
            self._newton_log("cloth attach: disabled after error: %r" % (e,))
            self._cloth_attach_path = None

    def _step_impl(self, dt):
        # One-time pose seed: write the initial POSITION targets straight
        # into joint_q so the robot spawns in its commanded pose (Spot's
        # crouch) instead of all-zero (straight legs). Newton's builder
        # seeds every revolute at angle 0; without this the controller has
        # to fold the legs into stance while the body is already falling,
        # and under MuJoCo that transient tips the robot every time (verified:
        # joint diag showed seed_angle=0.0 on all 12 joints, robot collapses
        # in ~0.7s regardless of stiffness). Opt-in via OMNISIM_NEWTON_SEED_POSE;
        # only touches position-target joints, so velocity-driven robots
        # (Husky wheels use joint_targets, not joint_targets_pos) are unaffected.
        import os as _seedos
        _seed_q_env = _seedos.environ.get("OMNISIM_NEWTON_SEED_Q")
        if (not getattr(self, "_pose_seeded", False)
                and (self.joint_targets_pos or _seed_q_env)
                and self.model is not None):
            if _seedos.environ.get("OMNISIM_NEWTON_SEED_POSE"):
                _seedmsg = "seed: skipped"
                try:
                    if not hasattr(self, "_q_start_cache"):
                        self._q_start_cache = self.model.joint_q_start.numpy()
                    qs = self._q_start_cache
                    jq = self.model.joint_q.numpy()
                    _n = 0
                    # Seed source: an explicit per-slot angle list passed in the
                    # ENGINE environment (OMNISIM_NEWTON_SEED_Q, comma-separated,
                    # slot order == joint creation order) takes precedence over the
                    # controller's live position targets. This decouples the spawn
                    # pose from controller/warmup-reload timing -- in TORQUE mode
                    # the controller's setPosition can be missed by the seed window,
                    # leaving the robot straight-legged (verified: a torque WBC then
                    # reads ~0 gravity bias and collapses in one tick). The env list
                    # is always available, so the seed is deterministic.
                    if _seed_q_env:
                        try:
                            _sv = [float(_x) for _x in _seed_q_env.split(",") if _x != ""]
                            _seed_pairs = list(enumerate(_sv))
                        except Exception:
                            _seed_pairs = list(self.joint_targets_pos.items())
                    else:
                        _seed_pairs = list(self.joint_targets_pos.items())
                    for _slot, _p in _seed_pairs:
                        _real = self.slot_to_real_idx.get(_slot)
                        if _real is None or _real >= len(qs):
                            continue
                        # Never seed a multi-DoF slot: a (slot, dof) key doesn't
                        # resolve above anyway, and a BALL joint's coordinates are
                        # a QUATERNION -- writing an angle into coordinate 0 would
                        # corrupt it into a non-unit rotation.
                        if (isinstance(_slot, int) and 0 <= _slot < len(self.pending_revolutes)
                                and self.pending_revolutes[_slot].get("kind") in ("ball", "hinge2")):
                            continue
                        _qi = int(qs[_real])
                        if 0 <= _qi < len(jq):
                            jq[_qi] = _p
                            _n += 1
                    # Seed the MODEL (initial config) AND both STATE
                    # buffers' joint_q -- the MuJoCo solver reads the
                    # per-step joint config from state.joint_q, so seeding
                    # only model.joint_q is silently ignored (the robot
                    # keeps starting straight-legged). eval_fk then makes
                    # body_q consistent with the seeded joint_q.
                    self.model.joint_q.assign(jq)
                    for _st in (self.state_a, self.state_b):
                        if _st is not None and hasattr(_st, "joint_q"):
                            try:
                                _st.joint_q.assign(jq)
                            except Exception:
                                pass
                    newton.eval_fk(self.model, self.model.joint_q,
                                   self.model.joint_qd, self.state_a)
                    self._mjc_dirty = True
                    # Remember the standing joint_q so every episode reset
                    # can restore it (reset_body_pose). Without this, only
                    # the first spawn is seeded -- every subsequent episode
                    # resets to straight legs and collapses before the
                    # controller's short settle can recover (training ep_len
                    # drops from ~590 to ~90 after the first reset).
                    self._standing_jq = jq.copy()
                    # MuJoCo also caches qpos at solver construction. The
                    # rebuild forces the standing config as the initial
                    # state, but it can desync a single joint's qpos --
                    # seeding state.joint_q (read each step) may suffice on
                    # its own. Gate the rebuild so we can compare; default
                    # OFF (state seed only). Set OMNISIM_NEWTON_SEED_REBUILD=1
                    # to restore the rebuild.
                    if (_seedos.environ.get("OMNISIM_NEWTON_FORCE_MUJOCO")
                            and _seedos.environ.get("OMNISIM_NEWTON_SEED_REBUILD")):
                        # Rebuild on the SAME engine the deploy runs (mujoco_warp
                        # when OMNISIM_NEWTON_MJWARP=1), NOT CPU mj_step. A CPU
                        # rebuild silently switched the stepping engine and a
                        # seeded CPU model still tips G1 (verified 1.42 s), whereas
                        # a seeded mujoco_warp model holds (the harness does exactly
                        # this). Matching the engine is what makes the spawn-seed
                        # actually prevent the straight->squat fold tip.
                        _seed_cpu = not _seedos.environ.get("OMNISIM_NEWTON_MJWARP")
                        _seed_kw = {"use_mujoco_cpu": _seed_cpu}
                        _snj = int(_seedos.environ.get("OMNISIM_NEWTON_NJMAX", "256"))
                        _snc = int(_seedos.environ.get("OMNISIM_NEWTON_NCONMAX", "256"))
                        if _snj > 0:
                            _seed_kw["njmax"] = _snj
                        if _snc > 0:
                            _seed_kw["nconmax"] = _snc
                        self.solver = newton.solvers.SolverMuJoCo(
                            self.model, **_seed_kw)
                        # New solver = new internal mjw buffers -> any
                        # captured step graph is stale.
                        self._step_graph = None
                    _seedmsg = (f"seed: applied {_n} joints (+state), rebuilt solver\n"
                                f"  targets(slot:pos)={ {int(s):round(float(p),2) for s,p in self.joint_targets_pos.items()} }\n"
                                f"  slot2real={ {int(k):int(v) for k,v in self.slot_to_real_idx.items()} }\n"
                                f"  jq_revolute={[round(float(v),3) for v in jq[7:]]}")
                except Exception as _se:
                    _seedmsg = f"seed: FAILED {_se!r}"
                try:
                    _lp = _seedos.environ.get("OMNISIM_NEWTON_LOG")
                    if _lp:
                        with open(_lp, "a") as _lf:
                            _lf.write(_seedmsg + "\n")
                except Exception:
                    pass
            self._pose_seeded = True
        # ⚠ CAPTURE "did this tick carry control" BEFORE the block below
        # CONSUMES (clears) the three dicts. The batched-substep path's gate
        # further down decides whether to run _apply_mjc_control -- the copy
        # that moves newton's `control` into mj_data.ctrl -- and it used to
        # re-read these same dicts, ~350 lines after they had been emptied. So
        # the gate was permanently False and, once a world became eligible for
        # batching, mj_data.ctrl was NEVER WRITTEN AGAIN: whatever value was
        # latched at the transition drove the actuators for the rest of the
        # run. Measured on a stock URDFRobot Husky: 0.997 / 1.026 / 1.034 /
        # 1.041 m/s for commanded none / 6.0 / 0.0 / 12.0 rad/s -- commanding
        # ZERO did not stop it, and setPosition/setTorque were equally inert.
        # The same hazard was already known one branch down (see the
        # _joint_f_ever comment at the joint_forces clear), just not applied
        # to the velocity/position dicts.
        _tick_has_ctrl = bool(self.joint_targets or self.joint_targets_pos
                              or self.joint_forces)
        if _tick_has_ctrl:
            if self.control is None:
                self.control = self.model.control()
            # Index translation: joint_target_vel is indexed by qd DOF,
            # joint_target (position) is indexed by q DOF. They differ
            # for the FREE joint (7 q values vs 6 qd values). Caching
            # both starts arrays avoids per-tick GPU->CPU copies.
            if not hasattr(self, "_qd_start_cache"):
                self._qd_start_cache = self.model.joint_qd_start.numpy()
            if not hasattr(self, "_q_start_cache"):
                self._q_start_cache = self.model.joint_q_start.numpy()
            qd_start = self._qd_start_cache
            q_start = self._q_start_cache
            if self.joint_targets:
                # PERF: persistent HOST mirror of joint_target_vel. The
                # bridge owns every write to this array, so reading it back
                # from the GPU each tick (a full device sync) is wasted --
                # mirror on host, assign down only.
                vel_arr = getattr(self, "_target_vel_host", None)
                if vel_arr is None:
                    vel_arr = self._ctl_target_vel().numpy()
                    self._target_vel_host = vel_arr
                for key, v in self.joint_targets.items():
                    slot, dof = self._split_target_key(key)
                    real_idx = self.slot_to_real_idx.get(slot)
                    if real_idx is None or real_idx >= len(qd_start):
                        continue
                    dof_idx = int(qd_start[real_idx]) + dof
                    if 0 <= dof_idx < len(vel_arr):
                        vel_arr[dof_idx] = v
                # In-place assign (not reassign) -- a fresh wp.array per
                # step leaks GPU memory the solver still references.
                self._ctl_target_vel().assign(vel_arr)
                self.joint_targets = {}
            if self.joint_targets_pos and self._ctl_target_pos() is not None:
                pos_arr = getattr(self, "_target_pos_host", None)
                if pos_arr is None:
                    pos_arr = self._ctl_target_pos().numpy()
                    self._target_pos_host = pos_arr
                for key, p in self.joint_targets_pos.items():
                    slot, dof = self._split_target_key(key)
                    real_idx = self.slot_to_real_idx.get(slot)
                    if real_idx is None or real_idx >= len(qd_start):
                        continue
                    # control.joint_target_pos is indexed by DOF (qd), NOT q.
                    # The MuJoCo solver's apply_mjc_control_kernel reads
                    # joint_target_pos[qd_index]; writing at the q index put
                    # every revolute joint's target one slot off (the free
                    # root has 7 q / 6 qd), so each joint received the
                    # PREVIOUS joint's target and the legs could not hold a
                    # pose. (joint_target_vel above is already qd-indexed.)
                    # NOTE this is also why the multi-DoF offset added here is
                    # the DOF index and never the coordinate index -- a BALL
                    # joint has 3 DOFs but 4 coordinates.
                    dof_idx = int(qd_start[real_idx]) + dof
                    if 0 <= dof_idx < len(pos_arr):
                        pos_arr[dof_idx] = p
                self._ctl_target_pos().assign(pos_arr)
                self.joint_targets_pos = {}
            if self.joint_forces and hasattr(self.control, "joint_f"):
                # Torque sink: control.joint_f is DOF(qd)-indexed like
                # joint_target_vel. Zero the WHOLE array each tick (joint_f is
                # NOT auto-cleared by the solver), then write the commanded
                # slots; any joint not commanded this tick gets 0 Nm. For pure
                # torque control the joint must be EFFORT mode (no PD), else the
                # PD actuator force adds on top.
                f_arr = getattr(self, "_joint_f_host", None)
                if f_arr is None:
                    f_arr = self.control.joint_f.numpy()
                    self._joint_f_host = f_arr
                f_arr[:] = 0.0
                for key, tau in self.joint_forces.items():
                    slot, dof = self._split_target_key(key)
                    real_idx = self.slot_to_real_idx.get(slot)
                    if real_idx is None or real_idx >= len(qd_start):
                        continue
                    dof_idx = int(qd_start[real_idx]) + dof
                    if 0 <= dof_idx < len(f_arr):
                        f_arr[dof_idx] = tau
                self.control.joint_f.assign(f_arr)
                self.joint_forces = {}
                # Force mode is now LIVE in control.joint_f -- and joint_f is
                # NOT auto-cleared by the solver, so the torque persists across
                # ticks until an explicit zero write. The batch gate below must
                # therefore key on "this world has ever used force mode", not
                # on this tick's (already-consumed) joint_forces dict.
                self._joint_f_ever = True

        # W6 diagnostic (OMNISIM_DEBUG_JOINTS=<file>): per motorized joint, dump the commanded position
        # target (control.joint_target_pos) vs the actual joint_q and the effective ke, overwriting each step.
        # Localizes Spot's leg collapse: target==crouch & q==collapsed -> actuator too weak; target==0/wrong ->
        # the setpoint isn't reaching the DoF. Gated -> inert when unset.
        import os as _jdbg
        _jf = _jdbg.environ.get("OMNISIM_DEBUG_JOINTS")
        if _jf and self.control is not None and self._ctl_target_pos() is not None:
            try:
                if not hasattr(self, "_q_start_cache"):
                    self._q_start_cache = self.model.joint_q_start.numpy()
                if not hasattr(self, "_qd_start_cache"):
                    self._qd_start_cache = self.model.joint_qd_start.numpy()
                _tp = self._ctl_target_pos().numpy()
                _tv = self._ctl_target_vel().numpy()
                _jq = self.state_a.joint_q.numpy()
                _ke = self.model.joint_target_ke.numpy()
                _qs = self._q_start_cache; _qds = self._qd_start_cache
                _lines = []
                for _slot, _real in sorted(self.slot_to_real_idx.items()):
                    if _real >= len(_qs) or _real >= len(_qds):
                        continue
                    _qi = int(_qs[_real]); _di = int(_qds[_real])
                    _lines.append("slot=%d real=%d dof=%d target_pos=%s target_vel=%s joint_q=%s ke=%s" % (
                        _slot, _real, _di,
                        ("%.3f" % _tp[_di]) if _di < len(_tp) else "NA",
                        ("%.3f" % _tv[_di]) if _di < len(_tv) else "NA",
                        ("%.3f" % _jq[_qi]) if _qi < len(_jq) else "NA",
                        ("%.1f" % _ke[_di]) if _di < len(_ke) else "NA"))
                # The LAST link in the chain: newton's control array is only a
                # staging buffer -- what the solver actually integrates is
                # mj_data.ctrl, written by _apply_mjc_control. Dumping both
                # sides is what separates "the command never arrived" from
                # "the command arrived and the servo ignored it".
                try:
                    _sd = self.solver.mj_data
                    _lines.append("mj_ctrl=%s" % [round(float(c), 3)
                                                  for c in _sd.ctrl])
                    _lines.append("mj_qvel=%s" % [round(float(v), 3)
                                                  for v in _sd.qvel])
                except Exception as _ce:
                    _lines.append("mj_ctrl ERR %r" % (_ce,))
                with open(_jf, "w") as _fh:
                    _fh.write("\n".join(_lines) + "\n")
            except Exception as _je:
                with open(_jf, "w") as _fh:
                    _fh.write("ERR " + repr(_je) + "\n")

        # P4 perf: cache the contact buffer. model.contacts() allocates a
        # fresh ContactBuffer on every call -- on a 10-husky world that
        # accounted for the per-step cost climbing from 12 ms to 225 ms
        # over the first few seconds (allocator fragmentation + GC).
        # Probe 11 (340 fps pure Python) calls model.contacts() once
        # before the loop; we mirror that here.
        if not hasattr(self, "_contacts_cache"):
            self._contacts_cache = self.model.contacts() if hasattr(self.model, 'contacts') else None
        contacts = self._contacts_cache
        # Substepping (OMNISIM_NEWTON_SUBSTEPS, default 1 = unchanged).
        # XPBD's positional constraint solve diverges to NaN when a body
        # moves a large fraction of a contact's size in one step -- e.g.
        # the husky head-on world commands 50 rad/s wheels (8.3 m/s
        # surface speed = 0.13 m / 16 ms tick), and a single solver.step
        # at dt=16 ms blows up at step 1 regardless of spawn height
        # (empirically: low-speed 2.5 rad/s is stable, high-speed NaNs).
        # Splitting the tick into N sub-steps of dt/N (re-running collide
        # each sub-step) keeps per-substep displacement small and the
        # constraint solve convergent. Default 1 preserves the exact
        # physics (and determinism) of every existing world + trained RL
        # policy; high-closing-speed worlds opt in to N>1.
        if not hasattr(self, "_n_substeps"):
            import os as _ss
            # Precedence: OMNISIM_NEWTON_SUBSTEPS env (launch override, keeps the
            # smoke runner + RL recipes working) > WorldInfo.newtonSubsteps
            # (set_substeps, folds the knob into the .wbt) > 1 (unchanged).
            _env_ss = _ss.environ.get("OMNISIM_NEWTON_SUBSTEPS")
            try:
                if _env_ss is not None:
                    self._n_substeps = max(1, int(_env_ss))
                else:
                    self._n_substeps = max(1, int(getattr(self, "_n_substeps_world", 1)))
            except ValueError:
                self._n_substeps = 1
        # In-engine MPC planner (OMNISIM_INENGINE_MPC=1): override the balance-joint
        # position targets with an MPPI plan rolled out in THIS solver, before the
        # substep loop reads self.control. Falls back silently to the plain hold.
        # Flag cached at first use (item 3): this ran an os.environ lookup
        # every tick on every world, MPC or not.
        import os as _impco   # alias is used by later lines in this method
        if not hasattr(self, "_inengine_mpc_on"):
            self._inengine_mpc_on = bool(_impco.environ.get("OMNISIM_INENGINE_MPC"))
        if self._inengine_mpc_on:
            try:
                self.mpc_stand_step()
            except Exception as _e:
                self._mpc_log("stand_step error: %r" % (_e,))
        # In-engine quad LOCOMOTION MPC (OMNISIM_INENGINE_MPC_LOCO=1). The logic
        # lives in an external module so it can be iterated WITHOUT a C++ rebuild;
        # this hook just imports + calls it. Additive + env-gated (default off).
        if _impco.environ.get("OMNISIM_INENGINE_MPC_LOCO"):
            try:
                import sys as _ls
                _lr = _impco.environ.get("OMNISIM_HOME")
                if _lr and _lr not in _ls.path:
                    _ls.path.insert(0, _lr)
                import importlib as _li
                _lm = _li.import_module("projects.policies.research.mpc.quad_mpc_engine")
                _lm.loco_step(self)
            except Exception as _e:
                self._mpc_log("loco_step error: %r" % (_e,))
        # Generic in-engine control-module hook: OMNISIM_INENGINE_PYMOD="pkg.mod:func"
        # imports the module ONCE (per process) and calls func(self) each tick, giving
        # the module full live mujoco dynamics + the joint_f torque sink. For iterating
        # control logic (e.g. the deterministic torque-WBC walking controller) WITHOUT a
        # C++ rebuild -- edit the .py, re-launch. Additive + env-gated (default off).
        _pymod = _impco.environ.get("OMNISIM_INENGINE_PYMOD")
        # PER-SUBSTEP mode (OMNISIM_INENGINE_PYMOD_SS=1): stiff TORQUE control is
        # unstable at the 62.5Hz tick (overshoot -> divergence); run the module in the
        # substep loop (250Hz) instead. Disables the CUDA graph (Python in the loop).
        _pymod_ss = bool(_pymod) and _impco.environ.get("OMNISIM_INENGINE_PYMOD_SS") not in (None, "", "0")
        if _pymod:
            try:
                if not hasattr(self, "_pymod_fn"):
                    import sys as _ps
                    _pr = _impco.environ.get("OMNISIM_HOME")
                    if _pr and _pr not in _ps.path:
                        _ps.path.insert(0, _pr)
                    import importlib as _pi
                    _mp, _fnn = _pymod.split(":")
                    self._pymod_fn = getattr(_pi.import_module(_mp), _fnn)
                if not _pymod_ss:
                    self._pymod_fn(self)
            except Exception as _e:
                self._mpc_log("pymod error: %r" % (_e,))
        if _impco.environ.get("OMNISIM_INENGINE_WBC"):
            try:
                self.wbc_stand_step()
            except Exception as _e:
                self._mpc_log("wbc_step error: %r" % (_e,))
        # TSID modes:
        #  * PER-SUBSTEP (default, pure torque): recomputed inside the substep loop
        #    below at 250Hz -- pure torque control is unstable at the 62.5Hz tick.
        #    This DISABLES the CUDA graph (Python in the substep loop) -> slow.
        #  * PER-TICK (TSID_TICK=1, for FF-on-servo): the operational-space FF is a
        #    bounded balance correction ADDED on top of a position servo that already
        #    gives per-substep stability, so the FF can run once per tick (zero-order
        #    held across the substeps). This keeps the CUDA graph -> fast sim.
        _tsid_any = _impco.environ.get("OMNISIM_INENGINE_TSID") not in (None, "", "0")
        _tsid_tick = _tsid_any and _impco.environ.get("TSID_TICK") not in (None, "", "0")
        _tsid_on = _tsid_any and not _tsid_tick   # per-substep (graph-disabling) path
        if _tsid_tick:
            try:
                self.tsid_stand_step()
            except Exception as _e:
                self._mpc_log("tsid_tick error: %r" % (_e,))
        sub_dt = float(dt) / self._n_substeps
        # One-shot, and here rather than in finalize() because the bound is
        # inertia/dt and the INTEGRATOR's dt (sub_dt, not the engine tick) is
        # first known now -- nothing plumbs basicTimeStep into this runtime.
        if not getattr(self, "_kv_clamped", False):
            self._clamp_velocity_servo_gains(sub_dt)
        # W3.1: external body wrenches queued this tick via add_body_force -> a (body_count,6) host array
        # written into state.body_f AFTER clear_forces() each substep (clear_forces zeros body_f). Cleared
        # after the tick so the controller must re-apply every tick (ODE addBodyForce semantics).
        _ext = getattr(self, "_ext_wrench", None)
        _ext_bf = None
        if _ext and self.model is not None:
            import numpy as _npw
            _ext_bf = _npw.zeros((self.model.body_count, 6), dtype=_npw.float32)
            for _bi, _w in _ext.items():
                if 0 <= _bi < self.model.body_count:
                    _ext_bf[_bi] = _w
        # PERF (realtime deploy): two per-substep costs that are pure waste
        # in MuJoCo-contacts mode:
        #  1. newton's model.collide() -- SolverMuJoCo with use_mujoco_contacts
        #     runs its OWN collision inside mjwarp and ignores the contacts
        #     arg entirely; newton's narrow-phase costs ~25 ms/substep on a
        #     mesh-floor world (it alone held the G1 deploy at 0.15x
        #     realtime). The newton contact buffer only feeds get_contacts(),
        #     which refills lazily on demand. Escape hatch:
        #     OMNISIM_NEWTON_KEEP_COLLIDE=1. (A world-reload explosion was
        #     initially blamed on this skip; the real culprit was the missing
        #     teardownWorld() lifecycle hook -- see ~OmSimulationWorld.)
        #  2. _update_mjc_data (newton state -> MuJoCo qpos/qvel copy-in,
        #     gated by solver.update_data_interval) -- only needed on ticks
        #     where the bridge wrote newton state EXTERNALLY (pose reset,
        #     joint-limit clamp, setVelocity, seed, ext wrench). Sites that
        #     mutate newton state set self._mjc_dirty.
        if not hasattr(self, "_skip_collide"):
            import os as _sco
            self._skip_collide = (
                getattr(self.solver, "_use_mujoco_contacts", False)
                and not _sco.environ.get("OMNISIM_NEWTON_KEEP_COLLIDE"))
            # ⚠ NEVER SKIP THE NARROW PHASE ON A CLOTH WORLD. The skip is sound
            # only because SolverMuJoCo collides internally and newton's buffer
            # is then a convenience for get_contacts(). SolverVBD has no such
            # internal pass: the soft_contact_* arrays the CollisionPipeline
            # writes ARE its collision input, so skipping collide() makes the
            # cloth pass through everything -- itself, the floor, the gripper --
            # with no error. (In practice the coupled solver has no
            # `_use_mujoco_contacts` attribute so this is already False; pin it
            # anyway, because "already False by accident" is not a contract.)
            if self.solver_soft is not None:
                self._skip_collide = False
        _dirty_tick = True
        if hasattr(self.solver, "update_data_interval"):
            if not hasattr(self, "_eager_copyin"):
                import os as _eco
                # Escape hatch: OMNISIM_NEWTON_EAGER_COPYIN=1 restores the
                # legacy copy-newton-state-in-every-substep behaviour (for
                # triaging dirty-gating regressions).
                self._eager_copyin = bool(_eco.environ.get("OMNISIM_NEWTON_EAGER_COPYIN"))
            _dirty = (self._eager_copyin
                      or getattr(self, "_mjc_dirty", True)
                      or (_ext_bf is not None))
            self.solver.update_data_interval = 1 if _dirty else 0
            self._mjc_dirty = False
            _dirty_tick = _dirty
        elif self.solver_soft is not None:
            # SolverCoupledProxy does not expose `update_data_interval` (the
            # copy-in gate belongs to the SolverMuJoCo *inside* it, and the
            # coupled solver owns that transfer itself). Without this branch
            # _dirty_tick would stay pinned True for ever and the CUDA-graph
            # capture below could never arm -- which is not a small loss on
            # cloth: MEASURED 6.7 fps on CPU vs 164 fps on CUDA + graph at 289
            # particles. Mirror the same dirty accounting so a tick that wrote
            # newton state externally still takes the direct path.
            _dirty_tick = (getattr(self, "_mjc_dirty", True) or (_ext_bf is not None))
            self._mjc_dirty = False
        # PERF: CUDA-graph the clean-tick substep sequence (mjwarp only) --
        # the same trick the GPU trainer uses. At nworld=1 a tick costs
        # ~800 tiny kernel launches; replaying a captured graph removes
        # nearly all of that launch overhead. newton's collide is INCLUDED
        # in the capture (identical work to the legacy loop, minus launch
        # overhead). Only for clean ticks (no external state writes, no ext
        # wrench) with an EVEN substep count (so the state_a/state_b buffer
        # roles are tick-invariant). Dirty ticks (resets etc.) take the
        # direct path. Disable via OMNISIM_NEWTON_NO_GRAPH=1.
        _graph = getattr(self, "_step_graph", None)
        # (XPBD's opt-in CUDA-graph copyback machinery was removed with the
        # solver, 2026-08-07. The copyback trick -- copy the post-tick state
        # into the canonical buffer INSIDE the capture so an odd substep count
        # graphs too -- remains the specified route to graphing mujoco_warp at
        # the default substep count; see step-cost-2026-08-06.md #5. It should
        # be rebuilt FOR mjwarp with its own validation, not resurrected from
        # the XPBD form.)
        # ODD SUBSTEP COUNTS GRAPH TOO, via an in-capture copyback.
        #
        # The even-count requirement was never about physics -- it is about
        # WHICH BUFFER the tick's result lands in. The substep loop ping-pongs
        # state_a/state_b, so an even count returns the data to the buffer it
        # started in and a captured graph is tick-invariant; an odd count
        # leaves it in the other one, and the second replay would then read a
        # stale buffer. Recording ONE extra device-to-device state copy at the
        # end of the capture makes the odd case tick-invariant as well: every
        # replay starts and ends with the canonical data in the same buffer.
        #
        # ⚠ THIS IS NOT A PHYSICS CHANGE. Identical substep count, identical
        # sub_dt, identical kernels in identical order -- only the buffer the
        # answer is left in differs, and that is invisible to every reader.
        #
        # WHY IT MATTERS HERE: OmniSim's default substep count is 1, i.e. ODD,
        # so every cloth world shipped to date was excluded from the graph by
        # this predicate alone. MEASURED on the 289-particle drape scene, the
        # cloth path is LAUNCH-BOUND, not compute-bound -- step cost is flat
        # from 81 to 2401 particles (30.6 / 29.9 / 28.7 / 30.6 / 30.7 ms) and
        # scales linearly with VBD iteration count (~2.42 ms per iteration) --
        # so collapsing the launches is the whole game. The pre-existing
        # workaround was to declare an even substep count, which buys the graph
        # at the price of doubling the physics; this buys it at the price of one
        # state copy. OMNISIM_NEWTON_NO_ODD_GRAPH=1 restores the even-only gate.
        _odd_graph_ok = not _os.environ.get("OMNISIM_NEWTON_NO_ODD_GRAPH")
        _can_graph = (_ext_bf is None
                      and not _pymod_ss           # per-substep module also recomputes in the loop
                      and not _tsid_on            # TSID recomputes torque per-substep (Python in loop)
                      and getattr(self, "_stepn", 0) > 10
                      and not getattr(self, "_graph_failed", False)
                      and not _dirty_tick
                      and (self._n_substeps % 2 == 0 or _odd_graph_ok)
                      and self._cloth_graph_ok())
        if _can_graph and _graph is None:
            import os as _go
            try:
                import warp as _wpg
                if (_go.environ.get("OMNISIM_NEWTON_NO_GRAPH")
                        or "cuda" not in str(self.model.device).lower()):
                    raise RuntimeError("graph disabled or not on cuda")
                # Remember the buffer BINDING the capture starts from. The
                # recorded kernels are bound to these exact arrays, so every
                # replay must begin with the canonical data in _cap_a0.
                _cap_a0, _cap_b0 = self.state_a, self.state_b
                with _wpg.ScopedDevice(self.model.device):
                    _wpg.synchronize()
                    _wpg.capture_begin(force_module_load=False)
                    try:
                        for _sub in range(self._n_substeps):
                            self.state_a.clear_forces()
                            if contacts is not None and not self._skip_collide:
                                self._collide_prof(self.state_a, contacts)
                                self._sstep(self.state_a, self.state_b,
                                                 self.control, contacts, sub_dt)
                            else:
                                self._sstep(self.state_a, self.state_b,
                                                 self.control, None, sub_dt)
                            self.state_a, self.state_b = self.state_b, self.state_a
                        if self.state_a is not _cap_a0:
                            # Odd substep count: the answer is in the other
                            # buffer. Record the copy that puts it back, so the
                            # graph is tick-invariant. State.assign() is a set of
                            # wp.array.assign device copies -- capturable; if a
                            # future newton makes it allocate, capture_end raises
                            # and the except below falls back to the direct path.
                            _cap_a0.assign(self.state_a)
                            self.state_a, self.state_b = _cap_a0, _cap_b0
                    finally:
                        self._step_graph = _wpg.capture_end()
                self._graph_dt = sub_dt
                _graph = self._step_graph
            except Exception:
                self._graph_failed = True
                self._step_graph = None
                _graph = None
        if (_can_graph and _graph is not None
                and getattr(self, "_graph_dt", None) == sub_dt):
            # Replay: kernels read the SAME control/state GPU buffers the
            # capture recorded, so this tick's joint targets (assigned
            # above) flow through. Capture only RECORDS launches, so the
            # capture tick itself also replays once to advance physics.
            import warp as _wpg
            with _wpg.ScopedDevice(self.model.device):
                _wpg.capture_launch(_graph)
            if self._skip_collide:
                self._collide_stale = True
        elif self._mjc_batch_substeps_ok(_tsid_on, _pymod_ss, _ext_bf, contacts):
            # BATCHED SUBSTEPS (CPU MuJoCo). The generic loop below calls
            # solver.step() once per substep, and EACH of those pays newton's
            # two conversions -- newton->mjc control in, mjc data->newton state
            # out -- around a single mj_step. Measured with
            # OMNISIM_NEWTON_STEP_PROFILE on a 5-box stack: ctrl_in 0.32 ms +
            # state_out 0.26 ms around an mj_step of 0.04 ms, so at the RL
            # substep count of 8 a control step pays those 0.58 ms EIGHT times
            # to do 0.35 ms of physics.
            #
            # The conversions only have to bracket the whole tick. Control does
            # not change between substeps (the controller writes targets once
            # per engine tick) and nothing reads the intermediate states -- the
            # per-substep Python hooks that WOULD read them are exactly what
            # _mjc_batch_substeps_ok() refuses to batch over.
            #
            # Physics is unchanged: this runs the same mj_step the same number
            # of times at the same sub_dt. What is skipped is round-tripping
            # newton state in and out between them. state_prev only carries the
            # kinematic DOFs MuJoCo does not integrate (newton's own docstring),
            # and those are identical either way; velocities come from
            # mj_data.qvel, which is correct after N substeps.
            self._n_batched = getattr(self, "_n_batched", 0) + 1
            _sv = self.solver
            self.state_a.clear_forces()
            # SKIP THE CONTROL CONVERSION WHEN THERE IS NO CONTROL. newton's
            # _apply_mjc_control allocates three warp arrays (ctrl / qfrc /
            # xfrc) and launches kernels on the CPU path before it looks at
            # whether anything is actually driven -- measured at 0.32 ms/step
            # on a world whose only bodies are five falling boxes with no
            # motors at all, i.e. 7x the 0.04 ms mj_step it precedes.
            #
            # Passive scenes are not a corner case here: props, stacks, dropped
            # parts and most of the lane-1 correctness scenes have no actuator.
            # A world that DOES drive joints takes the normal path.
            # ⚠ SKIPPING IS ONLY SAFE ONCE THE BUFFERS ARE ALREADY ZERO.
            # _apply_mjc_control does not just write control -- it OVERWRITES
            # mj_data.ctrl / qfrc_applied / xfrc_applied wholesale. Skip it on
            # the tick after a wrench was applied and MuJoCo keeps applying that
            # wrench for ever, because nothing ever clears it.
            #
            # Measured after this skip first shipped: a 1 N force applied for
            # 1 s of a 2 s run left the body at v = 1.996 m/s and x = 1.996 m
            # instead of 1.0 and 1.5 -- the force never stopped. Same shape on
            # torque (704 rad/s against an analytic 353). So the skip has to run
            # ONE more time after the last tick that had control, to zero them.
            # ⚠ Read the flag captured at the TOP of this method, never the
            # dicts -- they were consumed and cleared long before this line.
            _have_ctrl = _tick_has_ctrl
            if _have_ctrl or getattr(self, "_mjc_ctrl_written", False):
                _sv._apply_mjc_control(self.model, self.state_a, self.control, _sv.mj_data)
            self._mjc_ctrl_written = _have_ctrl
            _sv.mj_model.opt.timestep = sub_dt
            _mjstep = _sv._mujoco.mj_step
            if _LAUNCH_DBG:
                _d = _sv.mj_data
                self._ldbg_pre = {
                    "qpos": [round(float(v), 6) for v in _d.qpos],
                    "qvel": [round(float(v), 6) for v in _d.qvel],
                    "ctrl": [round(float(v), 6) for v in _d.ctrl],
                    "ncon": int(_d.ncon),
                    "path": "batched",
                }
            for _sub in range(self._n_substeps):
                _mjstep(_sv.mj_model, _sv.mj_data)
            # Weld/touch readbacks snapshot the SOLVED efc/cfrc state; must
            # run before the mj_step1 refresh re-instantiates efc unsolved.
            self._capture_constraint_readbacks()
            self._refresh_mj_cartesian(_sv)
            # (Tier 1c REMOVED 2026-08-09: a hand-rolled mj_data->State fill
            # lived here behind OMNISIM_NEWTON_MJ_DIRECT. It measured a LOSS on
            # both devices (+0.19 ms at N=5, +0.29 at N=50) because its own warp
            # plumbing cost what the FK cost, so it shipped default-off -- and it
            # was the tree's ONLY `newton._src` import, i.e. its single largest
            # upgrade-fragility. Dead weight carrying the highest risk: deleted.
            # The idea is not refuted, but its v2 must avoid warp writes entirely;
            # see physics-step-cost-optimization-plan.md.)
            _sv._update_newton_state(self.model, self.state_b, _sv.mj_data, state_prev=self.state_a)
            _sv._step += self._n_substeps
            self.state_a, self.state_b = self.state_b, self.state_a
            if self._skip_collide:
                self._collide_stale = True
        else:
            self._n_generic = getattr(self, "_n_generic", 0) + 1
            for _sub in range(self._n_substeps):
                # PER-SUBSTEP TSID (250Hz sim-time torque control -- stable where the
                # 62.5Hz tick rate was not). Recompute the whole-body torque from the
                # current substate and write control.joint_f before this substep.
                if _tsid_on:
                    try:
                        self.tsid_stand_step()
                    except Exception as _e:
                        self._mpc_log("tsid_ss error: %r" % (_e,))
                if _pymod_ss and getattr(self, "_pymod_fn", None) is not None:
                    try:
                        self._pymod_fn(self)
                    except Exception as _e:
                        self._mpc_log("pymod_ss error: %r" % (_e,))
                # clear_forces resets external force accumulation each (sub)step
                # (required by XPBD; see Newton's example_basic_urdf simulate()).
                self.state_a.clear_forces()
                if _ext_bf is not None:
                    self.state_a.body_f.assign(_ext_bf)
                if contacts is not None and not self._skip_collide:
                    self._collide_prof(self.state_a, contacts)
                    self._sstep(self.state_a, self.state_b, self.control, contacts, sub_dt)
                else:
                    if self._skip_collide:
                        self._collide_stale = True
                    self._sstep(self.state_a, self.state_b, self.control, None, sub_dt)
                self.state_a, self.state_b = self.state_b, self.state_a
            # solver.step() always runs _apply_mjc_control, so the MuJoCo
            # control buffers hold this tick's values. The fast path above uses
            # this to know it must issue one more (zeroing) write before it may
            # start skipping.
            self._mjc_ctrl_written = True
            # Same freshness contract as the batched path: whichever route the
            # tick took, mj_data's Cartesian arrays describe t+dt on return.
            # Here it is for READERS ONLY -- newton's own _update_newton_state
            # has already run inside solver.step() off qpos/qvel, so this cannot
            # change what the tick computed.
            # Weld/touch readbacks snapshot the SOLVED efc/cfrc state; must
            # run before the mj_step1 refresh re-instantiates efc unsolved.
            self._capture_constraint_readbacks()
            self._refresh_mj_cartesian(self._mjc_solver())
        if _ext:
            self._ext_wrench = {}  # consumed this tick; re-applied next tick by the controller
        self._contact_friction_probe()   # env-gated one-shot; inert unless asked
        if self.solver_soft is not None:
            self._cloth_overflow_check()
        # In-engine MPC latency probe (one-shot, gated by OMNISIM_INENGINE_MPC_SELFTEST).
        # Runs after a few warm-up ticks so the live solver state is populated.
        import os as _mpco   # alias is used by later lines in this method
        if not hasattr(self, "_mpc_selftest_on"):
            self._mpc_selftest_on = bool(_mpco.environ.get("OMNISIM_INENGINE_MPC_SELFTEST"))
        if (not getattr(self, "_mpc_selftest_done", False)
                and self._mpc_selftest_on):
            self._mpc_tick = int(getattr(self, "_mpc_tick", 0)) + 1
            if self._mpc_tick >= 6:
                self._mpc_selftest_done = True
                try:
                    self.mpc_selftest(int(_mpco.environ.get("MPC_K", "100")),
                                      int(_mpco.environ.get("MPC_H", "22")))
                except Exception as _e:
                    self._mpc_log("selftest error: %r" % (_e,))
        # W4.1 verification dump (OMNISIM_DEBUG_CONTACTS=<file>): overwrite the file each step with this step's
        # native contacts so a fixture can confirm the readback works in the binary. Gated -> zero cost off.
        import os as _cdbg
        _cf = _cdbg.environ.get("OMNISIM_DEBUG_CONTACTS")
        if _cf:
            try:
                flat = self.get_contacts()
                lines = ["contacts=%d" % (len(flat) // 10)]
                for k in range(0, len(flat), 10):
                    lines.append("bodyA=%d bodyB=%d p=(%.3f,%.3f,%.3f) n=(%.2f,%.2f,%.2f) depth=%.4f f=%.3f"
                                 % tuple(flat[k:k + 10]))
                with open(_cf, "w") as _fh:
                    _fh.write("\n".join(lines) + "\n")
            except Exception:
                pass

        # Constraint-buffer overflow watch. NOT opt-in any more (N15): this is
        # the only thing in the process that can SEE a constraint-buffer
        # overflow. mujoco_warp truncates the constraint vector in silence and
        # warns only via a wp.printf inside a warp kernel -- discarded outright
        # by a GUI-subsystem omnisim-bin.exe, and sunk into an unread
        # <log>.stdout everywhere else. _cs_every is resolved once in finalize()
        # (default 30 ticks when the mujoco_warp path is live, 0 = watch off);
        # _cs_verbose adds the historical every-new-peak newton_solver.log trace.
        # Wrapped so a telemetry failure can never disturb the simulation.
        _cs_every = int(getattr(self, "_cs_every", 0))
        if _cs_every > 0:
            try:
                self._cs_tick = int(getattr(self, "_cs_tick", 0)) + 1
                if self._cs_tick % _cs_every == 0:
                    self._sample_constraint_peaks(
                        bool(getattr(self, "_cs_verbose", False)))
            except Exception:
                pass

        # ---- Joint-space readback refresh (maximal-coordinate solvers) --
        # Solvers that integrate MAXIMAL coordinates never maintain
        # state.joint_q, so joint-space readback is stale unless we derive it
        # from the body poses. SolverMuJoCo maintains joint_q itself and sets
        # no flag here, so this is a no-op on the default path.
        #
        # ⚠ IT MUST RUN *BEFORE* THE JOINT CLAMP BELOW, not lazily inside
        # get_joint_angle(). _mjc_clamp_needed() is True whenever there is no
        # mj_data, so under VBD the clamp runs every tick, pre-fills
        # self._joint_q_cache from the stale array and sets
        # _joint_q_cache_fresh -- and every consumer (get_joint_angle,
        # _joint_q_slice, get_joint_angle_dof, get_joint_ball_quat,
        # readback_packed) only fills that cache when it is None. A lazy
        # refresh would therefore be a silent no-op, which is exactly the
        # shape of the bug this fixes.
        #
        # eval_ik writes ONLY joint_q/joint_qd, never body_q, so dynamics are
        # untouched. For a prismatic joint it computes
        # q = dot(x_c - x_p, quat_rotate(q_p, axis)) -- the same expression
        # VBD's own drive kernel regulates -- so the number reported is exactly
        # the coordinate being controlled.
        if (self.model is not None
                and not getattr(self, "_solver_maintains_joint_q", True)):
            try:
                newton.eval_ik(self.model, self.state_a,
                               self.state_a.joint_q, self.state_a.joint_qd)
            except Exception as _ik_e:                      # noqa: BLE001
                if not getattr(self, "_eval_ik_warned", False):
                    self._eval_ik_warned = True
                    self._newton_log(
                        "joint readback: eval_ik refresh failed (%r) -- "
                        "PositionSensor / JointParameters.position will report "
                        "the authored value on this solver path." % (_ik_e,))

        # ---- Hard joint-limit enforcement -----------------------------
        # The solver step can momentarily produce joint velocities past
        # the URDF velocity_limit or positions past [lower, upper] under
        # high-impact transients (e.g. external collisions whipping a
        # limb). The URDF velocity_limit and joint range are HARD
        # physical limits in the real robot -- the motor cannot spin
        # faster than its back-EMF allows, and the joint cannot rotate
        # past its mechanical end-stops. We enforce the same as a
        # post-step state clamp so the simulator can never expose
        # readings that are impossible in real hardware.
        #
        # The clamp acts on the state buffers AFTER the solver has
        # produced the integrated step, and writes the corrected values
        # back so the next solver step starts from a physical state.
        # When a joint position is clamped, its velocity is zeroed too
        # (real end-stops absorb the impact -- the energy goes into
        # deformation/heat, in sim it dissipates).
        #
        # Off-switch: OMNISIM_NEWTON_DISABLE_JOINT_CLAMP=1 (for legacy
        # studies; defaults to ENFORCEMENT ON).
        if not hasattr(self, "_joint_clamp_on"):
            import os as _bnos
            self._joint_clamp_on = (_bnos.environ.get(
                "OMNISIM_NEWTON_DISABLE_JOINT_CLAMP", "0") == "0")
        # ⚠ SKIPPED ON MAXIMAL-COORDINATE SOLVERS. There the clamp is dead for
        # dynamics -- VBD never reads state.joint_q, and enforces limits itself
        # through its own limit slot -- but it is NOT harmless: it costs two
        # GPU->CPU syncs plus a Python loop every tick, and now that the
        # readback above is honest it would overwrite the true coordinate with
        # a clamped one whenever a body is genuinely past its stop, i.e. swap
        # the old lie for a new one.
        if (self.model is not None and self.pending_revolutes
                and self.slot_to_real_idx and self._joint_clamp_on
                and getattr(self, "_solver_maintains_joint_q", True)):
            try:
                if not hasattr(self, "_q_start_cache"):
                    self._q_start_cache = self.model.joint_q_start.numpy()
                if not hasattr(self, "_qd_start_cache"):
                    self._qd_start_cache = self.model.joint_qd_start.numpy()
                # FAST PRE-CHECK against MuJoCo's own qpos/qvel (item 1,
                # verified half). The clamp fires only on a violation, but it
                # used to pay for the check every tick: two warp readbacks plus
                # a per-joint Python loop with dict lookups, on every world,
                # forever. mj qpos/qvel are the authority the solver just
                # integrated, and for a scalar hinge they hold the same value
                # newton's joint_q/joint_qd do -- measured agreement 1.5e-05 rad
                # and 1.2e-07 rad/s on the 8-Husky world (probe:
                # OMNISIM_NEWTON_MJ_POSE_CHECK, "JOINT vs mj"). So: vectorised
                # numpy test on mj arrays first, and touch newton state ONLY
                # when something is actually out of range, where the original
                # (unchanged) path below then does the work.
                #
                # ⚠ This is the JOINT half only. The body half of the same idea
                # -- serving body_q from mj xpos -- is NOT safe on articulated
                # robots: the same probe measures |xpos - body_q| up to 2.8e-02
                # m on the Husky (vs 2.7e-07 on box worlds), i.e. the frames are
                # not interchangeable there. Do not "finish" this by extending
                # it to bodies without resolving that first.
                _clamp_needed = self._mjc_clamp_needed()
                if not _clamp_needed:
                    raise _ClampClean()
                _q_arr = self.state_a.joint_q.numpy()
                _qd_arr = self.state_a.joint_qd.numpy()
                _q_changed = False
                _qd_changed = False
                _bh2_clamp = self.ball_hinge2_enabled()
                for _slot, _real in self.slot_to_real_idx.items():
                    _spec = self.pending_revolutes[_slot]
                    # Multi-DoF (ball / hinge2) and 0-DoF (fixed) specs carry no
                    # scalar velocity_limit / limit_lower keys, so the reads below
                    # raise KeyError and the WHOLE clamp (every joint's) is lost to
                    # the outer except. Skip them instead: their per-DoF ranges are
                    # already enforced by the MuJoCo hinge elements the d6 converts
                    # into (and a BALL joint must never be clamped coordinate-wise
                    # at all -- its coordinates are a quaternion, not angles).
                    # UNCONDITIONAL, deliberately: this is a live defect, not a
                    # flag-dependent one. add_joint_fixed specs (the force-TouchSensor
                    # un-fold) already lack the key today, so one such sensor silently
                    # disables the velocity AND position clamp for every joint in the
                    # world. Preserving that byte-for-byte is not worth it -- the clamp
                    # is a safety limit, and losing it quietly is the worse outcome.
                    if "velocity_limit" not in _spec:
                        continue
                    _qi = int(self._q_start_cache[_real])
                    _qdi = int(self._qd_start_cache[_real])
                    # Velocity-limit clamp.
                    _vlim = _spec["velocity_limit"]
                    if _vlim > 0.0 and 0 <= _qdi < len(_qd_arr):
                        _v = float(_qd_arr[_qdi])
                        if _v > _vlim:
                            _qd_arr[_qdi] = _vlim
                            _qd_changed = True
                        elif _v < -_vlim:
                            _qd_arr[_qdi] = -_vlim
                            _qd_changed = True
                    # Position-range clamp. URDF (0,0) means "no limit".
                    # Mimics a real mechanical end-stop: clamp q into
                    # range, but only absorb the velocity that was
                    # driving the joint INTO the stop. Velocity heading
                    # back toward the valid range passes through, so a
                    # joint sitting at q=lo with an actuator pulling it
                    # up isn't frozen on the stop.
                    _lo, _hi = _spec["limit_lower"], _spec["limit_upper"]
                    if _lo != _hi and 0 <= _qi < len(_q_arr):
                        _q = float(_q_arr[_qi])
                        if _q < _lo:
                            _q_arr[_qi] = _lo
                            _q_changed = True
                            if 0 <= _qdi < len(_qd_arr) and _qd_arr[_qdi] < 0.0:
                                _qd_arr[_qdi] = 0.0
                                _qd_changed = True
                        elif _q > _hi:
                            _q_arr[_qi] = _hi
                            _q_changed = True
                            if 0 <= _qdi < len(_qd_arr) and _qd_arr[_qdi] > 0.0:
                                _qd_arr[_qdi] = 0.0
                                _qd_changed = True
                if _q_changed:
                    self.state_a.joint_q.assign(_q_arr)
                if _qd_changed:
                    self.state_a.joint_qd.assign(_qd_arr)
                if _q_changed or _qd_changed:
                    # Newton state changed outside the solver -> refresh
                    # the MuJoCo-side data from it on the next tick.
                    self._mjc_dirty = True
                # PERF: the clamp already read the post-step joint_q --
                # reuse it as this tick's sensor-readback cache instead of
                # paying another GPU sync in get_joint_angle(). (Safe on
                # MuJoCo, which maintains joint_q itself; the XPBD caveat
                # that used to gate this went with the solver.)
                self._joint_q_cache = _q_arr
                self._joint_q_cache_fresh = True
                # body_q stays one tick stale (still reflects pre-clamp
                # joint_q). The next solver step re-derives body_q from
                # the now-clamped joint_q, so the staleness self-corrects
                # within one tick. Calling eval_fk here used to crash
                # silently inside the try/except, rolling back the
                # joint_q assign and re-exposing the violation.
            except _ClampClean:
                pass      # fast path: nothing out of range, newton state untouched
            except Exception:
                # Never let the clamp crash the sim; an empty pass keeps
                # the engine running with un-clamped state (the violation
                # would be caught by tests/engine/joint_limits anyway).
                pass

        # ---- Base-state divergence guard (opt-in) ---------------------
        # The post-step JOINT clamp above bounds only articulated DOFs; the
        # FLOATING-BASE body_q/body_qd are NOT joint DOFs. When a contact
        # solve produces a large impulse (a biped tipping over, or a high
        # closing-speed collision) the base position can diverge to absurd
        # magnitudes -- observed in the G1 stand deploy: once the robot
        # tips at t~1.55 s, pelvis bz blows up to 1e4..1e5 m. Nothing
        # downstream catches it (the joint clamp can't -- it's not a
        # joint), so the simulator exposes a non-physical pose and any
        # controller reading body_xform() sees garbage.
        #
        # This guard freezes the whole articulation at its last finite,
        # in-bounds state whenever body_q goes non-finite OR any base
        # coordinate exceeds OMNISIM_NEWTON_BASE_GUARD_MAX metres (default
        # 1000). It is a STRICT NO-OP for any physically-valid state
        # (finite + in-bounds passes through untouched), so it cannot
        # change a healthy sim's trajectory or determinism -- which is
        # exactly why it is safe to default ON. DEFAULT ON
        # (default-flip-plan.md §4.2 N3): when Newton is the default backend,
        # a world that diverges must never silently expose a NaN/exploded pose
        # to a controller (plan principle #4 "a flip never silently degrades a
        # world"); the guard converts that failure mode into a logged freeze
        # at the last good pose. Off-switch for legacy studies:
        # OMNISIM_NEWTON_BASE_GUARD=0.
        if not hasattr(self, "_base_guard"):
            import os as _bgos
            self._base_guard = _bgos.environ.get("OMNISIM_NEWTON_BASE_GUARD", "1") == "1"
            try:
                self._base_guard_max = float(_bgos.environ.get("OMNISIM_NEWTON_BASE_GUARD_MAX", "1000.0"))
            except ValueError:
                self._base_guard_max = 1000.0
            self._base_guard_tripped = False
            self._last_good_body_q = None
            self._last_good_body_qd = None
        if self._base_guard:
            try:
                import numpy as _bgnp
                _bq = self.state_a.body_q.numpy()
                _ok = bool(_bgnp.isfinite(_bq).all()) and bool(
                    (_bgnp.abs(_bq[:, :3]) <= self._base_guard_max).all())
                if _ok:
                    # Healthy: remember it; pass through untouched.
                    #
                    # ⚠ DO NOT "OPTIMISE" THE body_qd SYNC AWAY. It looks
                    # redundant -- the divergence branch below zeroes velocity
                    # rather than restoring it, so _last_good_body_qd is only
                    # read for its shape -- but this assignment ALSO refreshes
                    # the _body_qd_cache readback cache, and the invalidation
                    # block further down is SKIPPED whenever the guard marks the
                    # caches fresh. Drop it and body_vel() serves a stale
                    # velocity for the rest of the run.
                    #
                    # Tried on 2026-08-05 and reverted the same session: it
                    # bought nothing measurable (the apparent 0.28 ms saving was
                    # an artefact of the sync moving outside the profiled
                    # region) and it broke lane-1 T3, whose rolling sphere
                    # stopped moving entirely -- a_meas 6.0e-18 against an
                    # analytic 2.3966. The 5-box bench world stayed
                    # bit-identical throughout, so the bench did not catch it;
                    # the correctness lane did.
                    _bqd = self.state_a.body_qd.numpy()
                    self._last_good_body_q = _bq.copy()
                    self._last_good_body_qd = _bqd.copy()
                    # PERF: the guard already paid the post-step body_q /
                    # body_qd syncs -- reuse them as this tick's readback
                    # caches (body_xform/body_vel) instead of re-reading.
                    self._body_q_cache = _bq
                    self._body_qd_cache = _bqd
                elif self._last_good_body_q is not None:
                    # Diverged: restore last good pose, zero all velocities.
                    _zqd = _bgnp.zeros_like(self._last_good_body_qd)
                    self.state_a.body_q.assign(self._last_good_body_q)
                    self.state_a.body_qd.assign(_zqd)
                    self._mjc_dirty = True
                    self._body_q_cache = self._last_good_body_q
                    self._body_qd_cache = _zqd
                    self._body_cache_fresh = True
                    if not self._base_guard_tripped:
                        self._base_guard_tripped = True
                        try:
                            print("[OmNewtonBackend] base-divergence guard tripped "
                                  "(body_q non-finite or beyond %.0f m); freezing "
                                  "articulation at last good pose. Set "
                                  "OMNISIM_NEWTON_BASE_GUARD=0 to disable."
                                  % self._base_guard_max, flush=True)
                        except Exception:
                            pass
            except Exception:
                # Never let the guard crash the sim.
                pass

        _pc = getattr(self, "_mj_pose_check_on", None)
        if _pc is None:
            _pc = bool(_os.environ.get("OMNISIM_NEWTON_MJ_POSE_CHECK"))
            self._mj_pose_check_on = _pc
        if _pc:
            self._mj_pose_check()

        # Invalidate the per-step readback caches -- next body_xform() /
        # get_joint_angle() call does one fresh GPU->CPU transfer, then
        # every readback within the tick reuses it. When the base guard
        # (body) or joint clamp (joint_q) already re-read the post-step
        # state this tick, their reads ARE the caches -- keep them.
        if getattr(self, "_body_cache_fresh", False):
            self._body_cache_fresh = False
        else:
            self._body_q_cache = None
            self._body_qd_cache = None
        if getattr(self, "_joint_q_cache_fresh", False):
            self._joint_q_cache_fresh = False
        else:
            self._joint_q_cache = None
        # Same contract for the cloth vertex readback: one GPU->CPU transfer
        # per tick, reused by every particle_positions_packed() call within it.
        # Unconditional and cheap (a None store) -- there is no "fresh" path
        # that could have re-read particle_q this tick, and on a rigid world
        # the attribute is only ever this None.
        self._particle_q_cache = None

        # Contact diagnostic (OMNISIM_NEWTON_CONTACT_DIAG=1): every 60 steps
        # append the MUJOCO-side contact count + solver flags to the newton
        # log. Tells "mujoco never sees the ground" apart from "contacts
        # exist but are soft" when a world misbehaves.
        import os as _cdo
        if _cdo.environ.get("OMNISIM_NEWTON_CONTACT_DIAG"):
            _sn0 = getattr(self, "_stepn", 0)
            _every = 1 if _cdo.environ.get("OMNISIM_NEWTON_CONTACT_DIAG") == "2" else 60
            if _sn0 % _every == 0:
                try:
                    _ncon = "n/a"
                    _mjwd = getattr(self.solver, "mjw_data", None)
                    if _mjwd is not None and hasattr(_mjwd, "ncon"):
                        _ncon = str(_mjwd.ncon.numpy().tolist())
                    else:
                        _mjd = getattr(self.solver, "mj_data", None)
                        if _mjd is not None:
                            _ncon = str(int(_mjd.ncon))
                    _rcd = "n/a"
                    _mjwm = getattr(self.solver, "mjw_model", None)
                    if _mjwm is not None:
                        _rcd = str(bool(_mjwm.opt.run_collision_detection))
                    _lp = _cdo.environ.get("OMNISIM_NEWTON_LOG")
                    if _lp:
                        with open(_lp, "a") as _cf:
                            _cf.write(
                                f"contact_diag inst={id(self) % 100000} step={_sn0} ncon={_ncon} "
                                f"run_coll_det={_rcd} "
                                f"use_mjc_contacts={getattr(self.solver, '_use_mujoco_contacts', 'n/a')} "
                                f"skip_collide={getattr(self, '_skip_collide', 'n/a')} "
                                f"graph={'Y' if getattr(self, '_step_graph', None) is not None else 'N'}"
                                f"{' FAILED' if getattr(self, '_graph_failed', False) else ''} "
                                f"solver={type(self.solver).__name__} "
                                f"cpu={getattr(self.solver, 'use_mujoco_cpu', 'n/a')}\n")
                except Exception as _cde:
                    try:
                        _lp = _cdo.environ.get("OMNISIM_NEWTON_LOG")
                        if _lp:
                            with open(_lp, "a") as _cf:
                                _cf.write(f"contact_diag failed: {_cde!r}\n")
                    except Exception:
                        pass

        # Per-step joint+body diagnostic for the first few steps (opt-in
        # via OMNISIM_NEWTON_STEPDIAG): post-step angle per slot and body
        # z per body. Used to catch a single joint blowing out on step 1.
        self._stepn = getattr(self, "_stepn", 0) + 1
        if self._stepn <= 5 or (self._stepn % 60 == 0 and self._stepn <= 1200):
            import os as _spo
            _sp = _spo.environ.get("OMNISIM_NEWTON_STEPDIAG")
            if _sp:
                try:
                    qs = self.model.joint_q_start.numpy()
                    qds = self.model.joint_qd_start.numpy()
                    jq = self.state_a.joint_q.numpy()
                    jqd = self.state_a.joint_qd.numpy()
                    ang = {}
                    omega = {}
                    for _s, _r in self.slot_to_real_idx.items():
                        _qi = int(qs[_r])
                        _qdi = int(qds[_r])
                        ang[int(_s)] = round(float(jq[_qi]), 3) if _qi < len(jq) else None
                        omega[int(_s)] = round(float(jqd[_qdi]), 3) if _qdi < len(jqd) else None
                    _bq = self.state_a.body_q.numpy()
                    _bqd = self.state_a.body_qd.numpy()
                    bz = [round(float(z), 3) for z in _bq[:, 2]]
                    # Chassis = body 2 (parent of the wheel revolutes).
                    # Track its world x + linear-x velocity to tell a
                    # frozen articulation (no integration at all) from a
                    # spinning-wheels-but-no-traction case.
                    _ch = 2 if _bq.shape[0] > 2 else 0
                    chassis_x = round(float(_bq[_ch][0]), 4)
                    # body_qd layout = [vx,vy,vz, wx,wy,wz] (world frame)
                    chassis_vx = round(float(_bqd[_ch][0]), 4)
                    wheel_wy = [round(float(_bqd[i][4]), 3)
                                for i in range(3, min(7, _bqd.shape[0]))]
                    # 2026-05-28: also dump control.joint_target_vel for
                    # the wheel slots so we can verify the per-step push
                    # is reaching the buffer. Useful for diagnosing
                    # cases where the buffer write succeeds but the
                    # solver still doesn't apply torque (which is the
                    # ground truth this revealed for the husky-freeze
                    # bug -- ctrl_tv=2.5 but omega=0).
                    tv = None
                    try:
                        if self.control is not None:
                            tv_arr = self._ctl_target_vel().numpy()
                            tv = {int(_s): round(float(tv_arr[int(qds[_r])]), 3)
                                   for _s, _r in self.slot_to_real_idx.items()
                                   if int(qds[_r]) < len(tv_arr)}
                    except Exception:
                        pass
                    tp = None
                    try:
                        if self.control is not None and self._ctl_target_pos() is not None:
                            tp_arr = self._ctl_target_pos().numpy()
                            tp = {int(_s): round(float(tp_arr[int(qds[_r])]), 3)
                                   for _s, _r in self.slot_to_real_idx.items()
                                   if int(qds[_r]) < len(tp_arr)}
                    except Exception:
                        pass
                    _mjc = ""
                    try:
                        _md = getattr(self.solver, "mj_data", None)
                        if _md is not None:
                            _ncon = int(_md.ncon)
                            _pairs = []
                            for _ci in range(min(_ncon, 60)):
                                _c = _md.contact[_ci]
                                _pairs.append((int(_c.geom1), int(_c.geom2)))
                            _mjc = f"ncon={_ncon} geompairs={_pairs}"
                    except Exception as _ce:
                        _mjc = f"contact-dump-failed {_ce!r}"
                    with open(_sp, "a") as _f:
                        _f.write(f"step={self._stepn} angle(slot)={ang}\n"
                                 f"  omega(slot)={omega}\n"
                                 f"  ctrl_tv(slot)={tv}\n"
                                 f"  ctrl_tp(slot)={tp}\n"
                                 f"  chassis_x={chassis_x} chassis_vx={chassis_vx} "
                                 f"wheel_wy={wheel_wy}\n"
                                 f"  body_z={bz}\n"
                                 f"  mjcontacts: {_mjc}\n")
                except Exception as _e:
                    try:
                        with open(_sp, "a") as _f:
                            _f.write(f"step={self._stepn} stepdiag failed: {_e!r}\n")
                    except Exception:
                        pass

    def body_xform(self, idx):
        # body_q layout: [x, y, z, qx, qy, qz, qw] per body. XPBD updates
        # body_q directly, so no eval_ik needed for position readback.
        # P4: cache body_q.numpy() per step. Without this, Webots calls
        # getBodyXform once per Newton-backed Solid per step, each call
        # doing a full GPU->CPU transfer of ALL body poses. With 10
        # huskies (50 bodies) the readback alone dominated step time.
        # The cache is invalidated at the end of each step().
        cache = getattr(self, "_body_q_cache", None)
        if cache is None:
            cache = self.state_a.body_q.numpy()
            self._body_q_cache = cache
        q = cache[int(idx)]
        return (float(q[0]), float(q[1]), float(q[2]),
                float(q[3]), float(q[4]), float(q[5]), float(q[6]))

    def body_vel(self, idx):
        # Newton body_qd layout = [vx, vy, vz, wx, wy, wz] in WORLD frame
        # (verified empirically), which is exactly Webots getVelocity()
        # order. getVelocity() returns 0 under Newton without this because
        # the OmSolid's velocity fields were only ever filled by ODE. The
        # RL policy is a velocity-feedback balancer, so a zero velocity obs
        # made it (and in-OmniSim training) fail. Cached per step like body_q.
        cache = getattr(self, "_body_qd_cache", None)
        if cache is None:
            cache = self.state_a.body_qd.numpy()
            self._body_qd_cache = cache
        v = cache[int(idx)]
        return (float(v[0]), float(v[1]), float(v[2]),
                float(v[3]), float(v[4]), float(v[5]))

    def body_translations_packed(self, max_bodies):
        # R3.7b: bulk snapshot of every registered body's translation
        # (xyz) plus a 0-padding w channel, packed as a tight float32
        # bytes object of length min(max_bodies, body_count) * 16.
        # The wgpu storage buffer at @group(0) @binding(1) of
        # OmWgpuShaders::kTriangleInstanced reads vec4<f32> per body,
        # so this layout copies straight through wgpuQueueWriteBuffer
        # with no rewrite.
        #
        # Reuses the same body_q cache as body_xform() — a single
        # GPU->CPU transfer per step, even if both the per-body
        # supervisor read path and the wgpu-render path are active.
        import numpy as np
        cache = getattr(self, "_body_q_cache", None)
        if cache is None:
            cache = self.state_a.body_q.numpy()
            self._body_q_cache = cache
        count = min(int(max_bodies), len(cache))
        if count <= 0:
            return b""
        # cache[i] = [x, y, z, qx, qy, qz, qw]; we want [x, y, z, 0]
        out = np.zeros((count, 4), dtype=np.float32)
        out[:, 0:3] = cache[:count, 0:3]  # w stays 0
        return out.tobytes()

    def particle_positions_packed(self, particle_start=-1, particle_end=-1):
        # CLOTH READBACK for the C++ renderer: every particle's world position
        # as a TIGHT float32 xyz blob -- count * 12 bytes, no padding, no
        # interleaved attributes -- so the C++ side can memcpy it straight into
        # a vertex buffer whose stride is 12.
        #
        # ⚠ THIS LAYOUT DIFFERS DELIBERATELY FROM body_translations_packed()
        # ABOVE, which pads each body to a vec4 (16 B) because the wgpu storage
        # buffer at @group(0) @binding(1) of kTriangleInstanced reads
        # vec4<f32> per body and WGSL storage arrays require that alignment.
        # Cloth particles are MESH VERTICES, not per-instance uniforms: they
        # feed a triangle-list draw whose index buffer is the cloth topology,
        # and a vertex attribute has no such alignment rule. Padding them would
        # cost 33% of the transfer for nothing. Do not "make them consistent".
        #
        # Convention shared with body_translations_packed(): float32 (the
        # renderer's precision, half the bytes of the float64 readback_packed()
        # uses for physics), one GPU->CPU transfer per tick reused through a
        # per-step cache that step() invalidates, and an empty bytes object --
        # never an exception -- when there is nothing to report.
        #
        # particle_start/particle_end default to -1 = "all cloth particles"
        # (the union range every add_cloth_grid() call extended). Pass an
        # explicit sub-range to draw one sheet of several; the range recorded
        # per sheet is returned by add_cloth_grid() itself.
        import numpy as np
        if self.state_a is None:
            return b""
        cache = getattr(self, "_particle_q_cache", None)
        if cache is None:
            pq = getattr(self.state_a, "particle_q", None)
            if pq is None:
                return b""
            try:
                cache = pq.numpy()
            except Exception:                     # noqa: BLE001
                return b""
            self._particle_q_cache = cache
        n = len(cache)
        if n == 0:
            return b""
        lo = int(particle_start)
        hi = int(particle_end)
        if lo < 0:
            lo = self.cloth_particle_start if self.cloth_particle_start >= 0 else 0
        if hi < 0:
            hi = self.cloth_particle_end if self.cloth_particle_end >= 0 else n
        lo = max(0, min(lo, n))
        hi = max(lo, min(hi, n))
        if hi == lo:
            return b""
        # np.ascontiguousarray, not .tobytes() on the slice: particle_q is
        # (N, 3) float32 already on both devices, so this is normally a no-copy
        # view, but a strided sub-range would otherwise pack wrong.
        return np.ascontiguousarray(cache[lo:hi, 0:3], dtype=np.float32).tobytes()

    def cloth_topology_packed(self, grid_index=0):
        # The triangle index buffer for one authored cloth sheet, as tight
        # int32 triples, so the renderer can build its mesh ONCE at load and
        # then only stream positions each tick (topology never changes -- VBD
        # moves vertices, it does not retriangulate).
        #
        # Indices are LOCAL to the sheet (0-based within the sheet's particle
        # range), matching what particle_positions_packed(start, end) returns
        # for that same sheet. Returns b"" for an unknown sheet.
        import numpy as np
        if grid_index < 0 or grid_index >= len(self.cloth_grids):
            return b""
        g = self.cloth_grids[grid_index]
        # A MESH sheet (a garment) has no analytic winding to re-derive -- its
        # triangle list is whatever the asset said, cleaned -- so add_cloth_mesh
        # snapshotted it at authoring time, exactly as add_soft_grid does for a
        # tet mesh's open faces. Serve that verbatim.
        if g.get("mesh"):
            return g.get("surface", b"")
        # add_cloth_grid lays out (dim_x+1) x (dim_y+1) vertices row-major in y
        # and splits each cell into two triangles (builder.py:8987-9005, the
        # non-reverse_winding branch: [v0,v1,v3] and [v1,v2,v3]).
        nx = int(g["dim_x"]) + 1
        tris = []
        for y in range(1, int(g["dim_y"]) + 1):
            for x in range(1, nx):
                v0 = (y - 1) * nx + (x - 1)
                v1 = (y - 1) * nx + x
                v2 = y * nx + x
                v3 = y * nx + (x - 1)
                tris.append((v0, v1, v3))
                tris.append((v1, v2, v3))
        if not tris:
            return b""
        return np.asarray(tris, dtype=np.int32).reshape(-1).tobytes()

    def readback_packed(self):
        # Tier 1a (physics-step-cost-optimization-plan.md): the whole
        # per-tick readback in ONE crossing. The engine used to call
        # body_xform + body_vel once per Solid and get_joint_angle once
        # per hinge sensor, every tick -- each a PyObject_CallMethod
        # with by-name lookup, format parsing and 13 boxed PyFloats per
        # body (~400 crossings ~= 1-1.6 ms/tick at 200 bodies). This
        # returns every body's pose+velocity and every revolute slot's
        # angle as one float64 bytes blob the C++ side memcpys and then
        # serves per-Solid reads from for the rest of the tick.
        #
        # Layout: <i64 nbody><i64 nslot> then nbody rows of
        # [x,y,z, qx,qy,qz,qw, vx,vy,vz, wx,wy,wz] (13 f64, exactly the
        # values body_xform + body_vel return for that index), then
        # nslot f64 angles indexed BY SLOT ID with 0.0 for slots that
        # get_joint_angle would answer 0.0 for (missing mapping, idx out
        # of range) -- the same contract, so serving from this blob is
        # value-identical to the per-call path.
        #
        # Reuses the same per-step caches as the per-call readbacks
        # (_body_q_cache / _body_qd_cache / _joint_q_cache), so this is
        # still one GPU->CPU transfer per array per tick, and a blob
        # built here is bitwise-consistent with any per-call fallback
        # read made in the same tick.
        import numpy as np
        import struct
        bq = getattr(self, "_body_q_cache", None)
        if bq is None:
            bq = self.state_a.body_q.numpy()
            self._body_q_cache = bq
        bqd = getattr(self, "_body_qd_cache", None)
        if bqd is None:
            bqd = self.state_a.body_qd.numpy()
            self._body_qd_cache = bqd
        n = min(len(bq), len(bqd))
        body = np.empty((n, 13), dtype=np.float64)
        if n > 0:
            body[:, 0:7] = bq[:n]
            body[:, 7:13] = bqd[:n]
        angles = np.zeros(0, dtype=np.float64)
        try:
            if self.model is not None and self.slot_to_real_idx:
                if not hasattr(self, "_q_start_cache"):
                    self._q_start_cache = self.model.joint_q_start.numpy()
                q_start = self._q_start_cache
                q_arr = getattr(self, "_joint_q_cache", None)
                if q_arr is None:
                    try:
                        q_arr = self.state_a.joint_q.numpy()
                    except AttributeError:
                        q_arr = self.model.joint_q.numpy()
                    self._joint_q_cache = q_arr
                angles = np.zeros(max(self.slot_to_real_idx) + 1,
                                  dtype=np.float64)
                for slot, real_idx in self.slot_to_real_idx.items():
                    if real_idx < len(q_start):
                        q_idx = int(q_start[real_idx])
                        if q_idx < len(q_arr):
                            angles[slot] = q_arr[q_idx]
        except Exception:
            # Same posture as the per-call path: a joint-side hiccup
            # yields 0.0 angles, never a failed body readback.
            angles = np.zeros(0, dtype=np.float64)
        return (struct.pack("<qq", n, len(angles))
                + body.tobytes() + angles.tobytes())
