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

"""VERBATIM extract of kNewtonRuntimeSource from WbNewtonBackend.cpp -- the deploy's physics runtime. Keep in sync.

This module is a byte-for-byte copy of the embedded Python source the
OmniSim native backend executes at deploy time (the ``class World`` that
builds + steps the Newton model). It is extracted here so the golden-
parity test can build the LITERAL deploy model in-process and diff its
compiled MuJoCo model against the trainer's. ``check_in_sync()`` (and the
paired pytest) re-reads the R"PY(...)PY" block out of the .cpp and asserts
it still equals the body of this module, so the copy can never silently
drift from the deploy. Regenerate with _gen_deploy_runtime.py.
"""
import warp as wp
import newton

# P3.10e: this helper was rewritten after a 6-probe XPBD characterization
# (see scripts/xpbd_probes/notes.md). The old version used the wrong
# Newton API and gain values, hiding two compounding bugs:
#
#   1. `builder.add_body()` AUTO-ADDS a phantom 6-DOF FREE joint per body.
#      Subsequent `add_joint_revolute` calls add a SECOND joint instead of
#      replacing the free one, so every WbSolid had two joints fighting
#      each other -- chassis decoupled from wheels because XPBD couldn't
#      satisfy both. Fix: use `add_link` (no auto-joint), then *one*
#      explicit joint per body, then `add_articulation([joint_indices])`.
#
#   2. `target_kd=1.0` is ~500x too small for velocity drive. Newton's
#      official velocity-control test (newton/tests/test_joint_controllers.py)
#      uses `target_ke=0, target_kd=500` -- the wheel literally does not
#      spin with kd=1 + XPBD's default angular damping. Fix: WbBasicJoint
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

class World:
    def __init__(self):
        self.builder = newton.ModelBuilder(up_axis=newton.Axis.Z)
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
        import os as _muos
        self._ground_mu = float(_muos.environ.get("OMNISIM_NEWTON_GROUND_MU", "1.0"))
        self.builder.default_shape_cfg.mu = self._ground_mu
        # Contact compliance. SolverMuJoCo maps a shape's (ke, kd) to the geom's
        # MuJoCo solref (convert_solref). Default 2500/100 is firm; lowering ke
        # SOFTENS contacts -- needed so the gripper plowing a dense cube layer on
        # a DYNAMIC bin floor can't inject launch energy and eject neighbours
        # (the static ground never did). Env-tunable so it can be dialled in
        # without a rebuild.
        self.builder.default_shape_cfg.ke = float(_muos.environ.get("OMNISIM_NEWTON_CONTACT_KE", "2500"))
        self.builder.default_shape_cfg.kd = float(_muos.environ.get("OMNISIM_NEWTON_CONTACT_KD", "100"))

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

    def set_solver_preference(self, name):
        # Per-world solver choice from WorldInfo.newtonSolver, plumbed by the
        # C++ backend before finalize(). "mujoco" selects SolverMuJoCo (robust
        # frictional contact, required for pinch grasps); "" / "auto" / "xpbd"
        # keep the default GPU XPBD path. See the solver block in finalize().
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

    def add_ground_plane(self):
        self.builder.add_ground_plane()

    def add_body(self, mass, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0,
                 ixx=0.0, iyy=0.0, izz=0.0, ixy=0.0, ixz=0.0, iyz=0.0,
                 cx=None, cy=None, cz=None):
        # Inertia tensor: when the caller supplies non-zero diagonal
        # values, use them directly (URDF inertial block from the
        # WbSolid's Physics node). Otherwise fall back to the
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
        # COM-at-origin. The caller (WbSolid.cpp) supplies cx/cy/cz only when
        # OMNISIM_NEWTON_USE_LINK_COM is set (opt-in true URDF COM offset). The
        # URDF inertia tensor above is already about the COM frame, so passing
        # com + that inertia is the physically correct pairing.
        com = (wp.vec3(float(cx), float(cy), float(cz))
               if (cx is not None and cy is not None and cz is not None)
               else None)
        idx = self.builder.add_link(
            xform=wp.transform((float(x), float(y), float(z)),
                               (float(qx), float(qy), float(qz), float(qw))),
            armature=0.0,
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
            armature=0.0,
            inertia=wp.mat33((0.0, 0.0, 0.0),
                             (0.0, 0.0, 0.0),
                             (0.0, 0.0, 0.0)),
            mass=0.0,
            label=f"static_body_{len(self.body_indices)}",
        )
        self.body_indices.append(idx)
        self.static_body_indices.append(idx)
        return idx

    # density=0 on every collision shape -- mass comes from add_link's
    # `mass=` argument and we don't want shapes silently inflating it.
    # The default density 1000 added ~19 kg per wheel-sphere in probe 9
    # (chassis traveled 47% of theoretical instead of 98%); zero density
    # is mandatory.
    _SHAPE_CFG = None
    @classmethod
    def _shape_cfg(cls):
        if cls._SHAPE_CFG is None:
            import os as _muos
            _mu = float(_muos.environ.get("OMNISIM_NEWTON_GROUND_MU", "1.0"))
            _ke = float(_muos.environ.get("OMNISIM_NEWTON_CONTACT_KE", "2500"))
            _kd = float(_muos.environ.get("OMNISIM_NEWTON_CONTACT_KD", "100"))
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

    def add_shape_box(self, body_idx, hx, hy, hz, cx=0.0, cy=0.0, cz=0.0, ke=-1.0):
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
                               (0.0, 0.0, 0.0, 1.0)),
            hx=float(hx), hy=float(hy), hz=float(hz),
            cfg=cfg,
        )

    def add_shape_cylinder(self, body_idx, radius, half_height, cx=0.0, cy=0.0, cz=0.0):
        # W1.2 (newton-ode-replacement-plan.md): Newton's native cylinder narrow-phase locks wheels against
        # the ground (probe 7), so substitute a CAPSULE of the same radius + half_height instead of the old
        # sphere. A capsule's central segment is a true cylinder of the correct rolling radius -- a LINE
        # contact of the right width, strictly more faithful than a point-contact sphere -- and the capsule
        # narrow-phase is Newton's most robust (no lock). Newton capsules extend along local Z; a Webots
        # Cylinder bounding object extends along its body-local Y (the wheel Solid's own pose orients it --
        # e.g. BattleBot's `rotation 1 0 0 1.5708`, the cylinder unrotated within the body), so rotate the
        # capsule -90 deg about X to map local Z -> body Y. (A cylinder rotated WITHIN its bounding object
        # stays unhandled -- the same translation-only limitation every primitive shape has on this path.)
        import os as _os
        if _os.environ.get("OMNISIM_NEWTON_CYLINDER_AS_SPHERE"):
            # Revert lever (matches OMNISIM_NEWTON_MESH_TO_ODE): the pre-W1.2 point-contact sphere, kept so
            # the capsule fit can be A/B'd and disabled if a world ever regresses.
            return self.builder.add_shape_sphere(
                int(body_idx),
                xform=wp.transform((float(cx), float(cy), float(cz)), (0.0, 0.0, 0.0, 1.0)),
                radius=float(radius), cfg=self._shape_cfg())
        import math as _m
        h = -_m.pi / 4.0  # half of -90 deg, for the quaternion below
        q = (_m.sin(h), 0.0, 0.0, _m.cos(h))  # -90 deg about X: maps local Z -> body Y
        return self.builder.add_shape_capsule(
            int(body_idx),
            xform=wp.transform((float(cx), float(cy), float(cz)), q),
            radius=float(radius),
            half_height=float(half_height),
            cfg=self._shape_cfg(),
        )

    def add_shape_capsule(self, body_idx, radius, half_height):
        return self.builder.add_shape_capsule(int(body_idx),
                                              radius=float(radius),
                                              half_height=float(half_height),
                                              cfg=self._shape_cfg())

    def add_shape_plane(self, body_idx, cx=0.0, cy=0.0, cz=0.0):
        # Infinite static ground plane (e.g. a Floor's boundingObject), local normal +Z (the WbPlane
        # convention -- see WbPlane::createOdeGeom -- which matches add_shape_plane's xform-local-Z normal).
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

    def add_shape_mesh(self, body_idx, vertices, indices, n_vertices, cx=0.0, cy=0.0, cz=0.0):
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
        return self.builder.add_shape_mesh(
            int(body_idx),
            xform=wp.transform((float(cx), float(cy), float(cz)), (0.0, 0.0, 0.0, 1.0)),
            mesh=mesh,
            cfg=self._shape_cfg(),
        )

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

    def get_contacts(self):
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
                    self.model.collide(self.state_a, self._contacts_cache)
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

    def add_joint_revolute(self, parent_idx, child_idx,
                           ax, ay, az,
                           parent_anchor_x, parent_anchor_y, parent_anchor_z,
                           child_anchor_x, child_anchor_y, child_anchor_z,
                           target_ke=0.0, target_kd=0.0,
                           limit_lower=0.0, limit_upper=0.0,
                           effort_limit=0.0, velocity_limit=0.0,
                           child_rot_x=0.0, child_rot_y=0.0,
                           child_rot_z=0.0, child_rot_w=1.0):
        # Don't push to builder yet -- the caller (WbBasicJoint flush)
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
        # with two free angular DoF. newton-ode-replacement-plan.md W2.
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
        # No axes needed (the constraint is just "free rotation about the anchor"). Passive: ball target
        # pos/vel control is MuJoCo-only upstream, so a motorised ball joint is a follow-up.
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
        in WbBasicJoint -- Webots' hinge->position() reads from ODE only,
        so the Newton-backed joints have to be queried directly."""
        if self.model is None:
            return 0.0
        real_idx = self.slot_to_real_idx.get(int(slot_id))
        if real_idx is None:
            return 0.0
        # XPBD integrates maximal coords (body_q) and does NOT maintain the
        # generalized joint_q, so the angle stays at its seed forever -- the
        # position sensors read frozen while the body actually moves (arms
        # ran open-loop and flailed). Recompute joint_q from body_q via
        # eval_ik, once per step, so the readback is live. MuJoCo maintains
        # joint_q itself, so this is gated to XPBD.
        if getattr(self, "_solver_is_xpbd", False):
            _sn = getattr(self, "_stepn", 0)
            if getattr(self, "_ik_refresh_step", -1) != _sn:
                try:
                    newton.eval_ik(self.model, self.state_a,
                                   self.state_a.joint_q, self.state_a.joint_qd)
                except Exception:
                    pass
                self._ik_refresh_step = _sn
                # eval_ik rewrote joint_q on-device; drop any cached copy.
                self._joint_q_cache = None
        # joint_q is indexed via joint_q_start. For revolute joints this
        # is a single scalar (the angle in radians).
        if not hasattr(self, "_q_start_cache"):
            self._q_start_cache = self.model.joint_q_start.numpy()
        q_start = self._q_start_cache
        if real_idx >= len(q_start):
            return 0.0
        q_idx = int(q_start[real_idx])
        # state_a.joint_q is populated by the solver each step (XPBD
        # writes it via eval_ik in MuJoCo; XPBD-via-Warp updates it from
        # body_q each step). Fallback to the model's joint_q if the
        # state doesn't expose it yet.
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

    def set_joint_target_vel(self, slot_id, vel):
        # Stash target by slot id. step() translates slot -> real builder
        # joint index -> DOF index. Pre-finalize calls just queue.
        self.joint_targets[int(slot_id)] = float(vel)
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
            self.builder.joint_target_vel[qd_start] = float(vel)
        except Exception:
            pass

    def reset_body_pose(self, body_idx, x, y, z, qx, qy, qz, qw):
        """Warp a Newton body to the given world pose and zero its
        velocities. Called from WbSolid when a Supervisor write hits
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

    def set_joint_target_pos(self, slot_id, pos):
        # Companion to set_joint_target_vel: stash a position setpoint
        # for the POSITION_VELOCITY actuator. step() writes it into
        # control.joint_target each tick. Without this, target_ke acts
        # against an implicit 0 setpoint and the joint snaps back to
        # zero radians regardless of what the controller asked for.
        self.joint_targets_pos[int(slot_id)] = float(pos)

    def set_joint_force(self, slot_id, tau):
        # Torque-mode sink: stash a raw joint torque (Nm). step() writes it
        # into control.joint_f (applied generalized force -> MuJoCo
        # qfrc_applied), ADDITIVE over any POSITION_VELOCITY PD. For pure
        # torque control build the joint in EFFORT mode (no PD actuator) via
        # OMNISIM_NEWTON_TORQUE_MODE. Must be re-sent every tick: joint_f is
        # NOT auto-zeroed across steps.
        self.joint_forces[int(slot_id)] = float(tau)

    def _add_revolute_to_builder(self, j):
        """Push one queued revolute spec to the Newton builder. Returns
        the builder's joint index."""
        if j.get("kind") == "hinge2":
            # 2-DoF universal / Hinge2: a native Newton d6 joint with TWO free angular axes sharing the
            # anchor (e.g. a platform-cart caster: steer axis + roll axis, or a car front wheel). No motor
            # -- the cart wheels steer/spin freely; motorised hinge2 is a follow-up. Unspecified linear +
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
            # angular velocity; gimbal-free). No motor path -- ball target pos/vel control is MuJoCo-only
            # upstream, while the passive spherical constraint holds under XPBD + MuJoCo alike.
            # newton-ode-replacement-plan.md W2.2.
            return self.builder.add_joint_ball(
                parent=j["parent"], child=j["child"],
                parent_xform=wp.transform(j["p_anchor"], (0.0, 0.0, 0.0, 1.0)),
                child_xform=wp.transform(j["c_anchor"], (0.0, 0.0, 0.0, 1.0)),
                collision_filter_parent=True)
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
        # target_kd that WbBasicJoint::flushPendingNewtonRegistrations
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
        # Per-joint values from the C++ caller (WbBasicJoint, which
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

    def finalize(self):
        # ---- Deferred static planes ---------------------------------
        # Added here (not at add_shape_plane time) because the choice is
        # solver-dependent: newton's MuJoCo converter raises "Planes can
        # only be attached to static bodies" for our weld-pinned statics,
        # so under a MuJoCo preference the plane is dropped -- the default
        # ground plane (a true static) already provides the z=0 floor.
        if self._deferred_static_planes:
            import os as _dpo
            _dp_pref = (getattr(self, "_solver_pref", None) or "").lower()
            # Drop deferred planes whenever the solver WILL be MuJoCo -- by
            # WorldInfo.newtonSolver preference OR the OMNISIM_NEWTON_FORCE_MUJOCO
            # env (the RL deploy path). MUST mirror the _force_mujoco test in the
            # solver-construction block below: if it doesn't, a force-mujoco-env
            # deploy (empty _solver_pref) re-adds the plane here and SolverMuJoCo
            # then crashes on it -> silent XPBD fallback (the G1 deploy gap).
            _will_mujoco = (_dp_pref in ("mujoco", "mujoco_warp")
                            or bool(_dpo.environ.get("OMNISIM_NEWTON_FORCE_MUJOCO")))
            if _will_mujoco:
                self._deferred_static_planes = []
            else:
                for _b, _cx, _cy, _cz in self._deferred_static_planes:
                    self.builder.add_shape_plane(
                        body=_b,
                        xform=wp.transform((_cx, _cy, _cz), (0.0, 0.0, 0.0, 1.0)),
                        width=1000.0, length=1000.0,
                        cfg=self._shape_cfg(),
                    )
                self._deferred_static_planes = []

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
                # staticBase robot root (a bolted-down arm): anchor to the world
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
                # free-swing to their stops while distal joints look fine). Bolted-down
                # arms hid this because they spawn near the origin AND their
                # first joint is a yaw with no gravity load. Pinning at body_q[root]
                # welds the base where the .wbt placed it.
                jf = self.builder.add_joint_fixed(
                    parent=-1, child=root,
                    parent_xform=self.builder.body_q[root])
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

        self.builder.add_articulation(self.joint_indices, label="webots_world")

        self.model = self.builder.finalize()

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
                        f"eff={j['effort_limit']} lim=[{j['limit_lower']},{j['limit_upper']}] "
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
        # XPBD as primary: GPU-native via Warp, so we actually use the
        # available CUDA device instead of sitting on MuJoCo's CPU path.
        # The "three problems with XPBD" the codebase used to document
        # -- multi-articulation cliff at body~30, head-on contact-
        # resolution asymmetry, first-run disintegration on multi-husky
        # worlds -- all traced to the joint-anchor rotation projection
        # bug in WbBasicJoint (committed fix 16dcabe). MuJoCo (CPU)
        # stays as a safety fallback if XPBD construction fails on a
        # future newton release.
        _solver_kind = "unknown"
        _solver_error = None
        # Env-var override: OMNISIM_NEWTON_FORCE_MUJOCO=1 skips XPBD and
        # goes straight to MuJoCo CPU. This is the deploy path for RL
        # policies trained in MJX -- MJX is a JAX wrapper around the
        # same MuJoCo solver, so forcing this fallback gives bit-identical
        # physics across train and deploy (closes the sim-to-sim gap
        # that XPBD vs MJX otherwise opens). Costs the GPU acceleration
        # for the policy-deploy world, which is single-robot so it
        # doesn't matter.
        import os as _os
        # Solver choice. Per-world preference (WorldInfo.newtonSolver, plumbed
        # from C++ via set_solver_preference) selects MuJoCo for worlds that
        # need its robust frictional contact -- e.g. a pinch grasp, which XPBD
        # structurally cannot hold. OMNISIM_NEWTON_FORCE_MUJOCO still forces it
        # for dev/RL-deploy sweeps. Default stays XPBD (GPU, the perf path).
        _pref = getattr(self, "_solver_pref", None)
        # "mujoco"      -> SolverMuJoCo on the reference CPU mj_step (deterministic).
        # "mujoco_warp" -> SolverMuJoCo on the GPU mujoco_warp backend. Identical
        #   physics setup; the GPU path only pays off in FAST mode at a large
        #   basicTimeStep with an EVEN substep count (the CUDA-graph replay in
        #   step() needs even substeps -- an odd count re-issues ~800 kernel
        #   launches/tick and runs SLOWER than CPU). Requires a CUDA GPU + warp;
        #   falls back to CPU mj_step construction if warp can't init.
        _force_mujoco = (_pref in ("mujoco", "mujoco_warp")) or bool(_os.environ.get("OMNISIM_NEWTON_FORCE_MUJOCO"))
        # One-shot MJCF export: OMNISIM_NEWTON_SAVE_MJCF=<path> dumps the
        # EXACT MuJoCo model Newton built (inertias, ke/kd servos, friction,
        # joint limits) so a GPU-batched mujoco_warp trainer can load the
        # identical physics -- zero sim-to-sim gap with this deploy backend.
        _save_mjcf = _os.environ.get("OMNISIM_NEWTON_SAVE_MJCF")
        # XPBD integrates maximal coords (body_q) and does not maintain the
        # generalized joint_q; flag it so the joint-angle readback refreshes
        # joint_q from body_q via eval_ik (MuJoCo maintains joint_q itself).
        self._solver_is_xpbd = False
        if _force_mujoco:
            # use_mujoco_cpu=True -> reference MuJoCo mj_step (C lib);
            # =False -> mujoco_warp. These are DIFFERENT implementations and
            # a policy tuned to one can be unstable in the other. The GPU
            # trainer runs mujoco_warp, so OMNISIM_NEWTON_MJWARP=1 makes the
            # deploy use mujoco_warp too -> same engine -> policy transfers.
            _use_cpu = not (bool(_os.environ.get("OMNISIM_NEWTON_MJWARP")) or _pref == "mujoco_warp")
            _kw = {"use_mujoco_cpu": _use_cpu}
            # Contact-stability knobs for DENSE manipulation (env-tunable; unset
            # -> MuJoCo defaults = exact current physics). MuJoCo recommends a
            # HIGH impratio + ELLIPTIC cone + more iterations for grasping /
            # stacking so dense contacts don't slip and launch -- the tiltable-
            # bin cube-ejection failure mode. See the SolverMuJoCo docstring.
            _impr = _os.environ.get("OMNISIM_NEWTON_IMPRATIO")
            if _impr:
                _kw["impratio"] = float(_impr)
            _cone = _os.environ.get("OMNISIM_NEWTON_CONE")
            if _cone:
                _kw["cone"] = _cone
            _iters = _os.environ.get("OMNISIM_NEWTON_ITERS")
            if _iters:
                _kw["iterations"] = int(_iters)
            _lsiters = _os.environ.get("OMNISIM_NEWTON_LS_ITERS")
            if _lsiters:
                _kw["ls_iterations"] = int(_lsiters)
            # Constraint buffer caps. mujoco_warp auto-estimates these too
            # small for hard multi-contact footstrikes (the G1 trainer hit
            # the same "nefc overflow" -> dropped constraints -> foot
            # penetration -> mid-walk explosion; it pins njmax/nconmax=256).
            # Same defaults here; 0 restores newton's auto-estimate.
            _njmax = int(_os.environ.get("OMNISIM_NEWTON_NJMAX", "256"))
            _nconmax = int(_os.environ.get("OMNISIM_NEWTON_NCONMAX", "256"))
            if _njmax > 0:
                _kw["njmax"] = _njmax
            if _nconmax > 0:
                _kw["nconmax"] = _nconmax
            if _save_mjcf:
                _kw["save_to_mjcf"] = _save_mjcf
            try:
                self.solver = newton.solvers.SolverMuJoCo(self.model, **_kw)
                _why = "WorldInfo.newtonSolver" if _pref in ("mujoco", "mujoco_warp") else "FORCE_MUJOCO=1"
                _solver_kind = ("MuJoCo (%s, %s)" %
                                ("cpu/mj_step" if _use_cpu else "mujoco_warp", _why))
            except Exception as _me:
                # Capture the FULL construction error (preconditions raise
                # ValueError, etc.) so it surfaces in newton_solver.log, then
                # fall back to XPBD so the sim still runs instead of crashing.
                import traceback as _mtb
                _solver_error = "MUJOCO_CONSTRUCT_FAILED " + repr(_me) + " || " + _mtb.format_exc()[-1400:]
                self.solver = newton.solvers.SolverXPBD(self.model, iterations=10, angular_damping=0.0)
                self._solver_is_xpbd = True
                _solver_kind = "XPBD (MuJoCo construct FAILED)"
        else:
            try:
                import os as _xos
                _xpbd_iters = max(1, int(_xos.environ.get("OMNISIM_NEWTON_XPBD_ITERS", "10")))
                self.solver = newton.solvers.SolverXPBD(
                    self.model, iterations=_xpbd_iters, angular_damping=0.0,
                )
                _solver_kind = "XPBD(iters=%d)" % _xpbd_iters
                self._solver_is_xpbd = True
            except Exception as _e:
                _solver_error = repr(_e)
                self.solver = newton.solvers.SolverMuJoCo(self.model, use_mujoco_cpu=True)
                _solver_kind = "MuJoCo (cpu, XPBD fallback)"
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
            if _rmu and not self._solver_is_xpbd:
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
        # policy won't survive) is surfaced as a LOUD WbLog::warn, not just a
        # line in newton_solver.log.
        self._solver_kind = _solver_kind
        self._solver_error = "" if _solver_error is None else str(_solver_error)
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
        except Exception:
            pass
        # REQUIRE the MuJoCo solver (OMNISIM_REQUIRE_MUJOCO_SOLVER=1): an RL/legged policy is
        # trained under mujoco_warp; if a deploy world silently resolves to XPBD -- a DIFFERENT
        # engine -- because it forgot the WorldInfo.newtonSolver "mujoco" pin AND no FORCE_MUJOCO
        # env was set, the policy collapses (the "long-running G1 deploy gap"). This is the mirror
        # of OMNISIM_REQUIRE_NEWTON one level deeper (Newton-MuJoCo vs Newton-XPBD): fail LOUD here
        # instead of degrading silently. Default unset -> no behaviour change. The solver line was
        # already logged above, so the crash is fully diagnosable.
        import os as _rqo
        if _rqo.environ.get("OMNISIM_REQUIRE_MUJOCO_SOLVER") and self._solver_is_xpbd:
            raise RuntimeError(
                "OMNISIM_REQUIRE_MUJOCO_SOLVER=1 but the Newton solver resolved to XPBD "
                "(solver_kind=%r). A mujoco_warp-trained policy will not transfer to XPBD. Fix: add "
                "`newtonSolver \"mujoco\"` to the world's WorldInfo (run scripts/dev/check_deploy_solver.py "
                "--fix), or set OMNISIM_NEWTON_FORCE_MUJOCO=1. MuJoCo construct error (if any): %s"
                % (self._solver_kind, self._solver_error or "<none>"))
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
                    _L.append("body %d %-24s mass=%.3f ipos=%s pos=%s" % (
                        _b, _bn, _mjm.body_mass[_b],
                        [round(float(v), 3) for v in _mjm.body_ipos[_b]],
                        [round(float(v), 3) for v in _mjm.body_pos[_b]]))
                for _g in range(_mjm.ngeom):
                    _gn = _mj.mj_id2name(_mjm, _mj.mjtObj.mjOBJ_GEOM, _g) or "?"
                    _L.append("geom %d %-24s body=%d type=%d contype=%d conaff=%d "
                              "fric=%s size=%s pos=%s solref=%s" % (
                        _g, _gn, _mjm.geom_bodyid[_g], _mjm.geom_type[_g],
                        _mjm.geom_contype[_g], _mjm.geom_conaffinity[_g],
                        [round(float(v), 3) for v in _mjm.geom_friction[_g]],
                        [round(float(v), 4) for v in _mjm.geom_size[_g]],
                        [round(float(v), 3) for v in _mjm.geom_pos[_g]],
                        [round(float(v), 4) for v in _mjm.geom_solref[_g]]))
                for _u in range(_mjm.nu):
                    _L.append("act %d gain=%s bias=%s trn=%s" % (
                        _u, [round(float(v), 1) for v in _mjm.actuator_gainprm[_u][:3]],
                        [round(float(v), 1) for v in _mjm.actuator_biasprm[_u][:3]],
                        list(_mjm.actuator_trnid[_u])))
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
        # eval_fk so initial body_q reflects builder.joint_q.
        newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, self.state_a)

    def step(self, dt):
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
        if (not getattr(self, "_pose_seeded", False) and self.joint_targets_pos
                and self.model is not None):
            import os as _seedos
            if _seedos.environ.get("OMNISIM_NEWTON_SEED_POSE"):
                _seedmsg = "seed: skipped"
                try:
                    if not hasattr(self, "_q_start_cache"):
                        self._q_start_cache = self.model.joint_q_start.numpy()
                    qs = self._q_start_cache
                    jq = self.model.joint_q.numpy()
                    _n = 0
                    for _slot, _p in self.joint_targets_pos.items():
                        _real = self.slot_to_real_idx.get(_slot)
                        if _real is None or _real >= len(qs):
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
        if self.joint_targets or self.joint_targets_pos or self.joint_forces:
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
                    vel_arr = self.control.joint_target_vel.numpy()
                    self._target_vel_host = vel_arr
                for slot, v in self.joint_targets.items():
                    real_idx = self.slot_to_real_idx.get(slot)
                    if real_idx is None or real_idx >= len(qd_start):
                        continue
                    dof_idx = int(qd_start[real_idx])
                    if 0 <= dof_idx < len(vel_arr):
                        vel_arr[dof_idx] = v
                # In-place assign (not reassign) -- a fresh wp.array per
                # step leaks GPU memory the solver still references.
                self.control.joint_target_vel.assign(vel_arr)
                self.joint_targets = {}
            if self.joint_targets_pos and hasattr(self.control, "joint_target_pos"):
                pos_arr = getattr(self, "_target_pos_host", None)
                if pos_arr is None:
                    pos_arr = self.control.joint_target_pos.numpy()
                    self._target_pos_host = pos_arr
                for slot, p in self.joint_targets_pos.items():
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
                    dof_idx = int(qd_start[real_idx])
                    if 0 <= dof_idx < len(pos_arr):
                        pos_arr[dof_idx] = p
                self.control.joint_target_pos.assign(pos_arr)
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
                for slot, tau in self.joint_forces.items():
                    real_idx = self.slot_to_real_idx.get(slot)
                    if real_idx is None or real_idx >= len(qd_start):
                        continue
                    dof_idx = int(qd_start[real_idx])
                    if 0 <= dof_idx < len(f_arr):
                        f_arr[dof_idx] = tau
                self.control.joint_f.assign(f_arr)
                self.joint_forces = {}

        # W6 diagnostic (OMNISIM_DEBUG_JOINTS=<file>): per motorized joint, dump the commanded position
        # target (control.joint_target_pos) vs the actual joint_q and the effective ke, overwriting each step.
        # Localizes Spot's leg collapse: target==crouch & q==collapsed -> actuator too weak; target==0/wrong ->
        # the setpoint isn't reaching the DoF. Gated -> inert when unset.
        import os as _jdbg
        _jf = _jdbg.environ.get("OMNISIM_DEBUG_JOINTS")
        if _jf and self.control is not None and hasattr(self.control, "joint_target_pos"):
            try:
                if not hasattr(self, "_q_start_cache"):
                    self._q_start_cache = self.model.joint_q_start.numpy()
                if not hasattr(self, "_qd_start_cache"):
                    self._qd_start_cache = self.model.joint_qd_start.numpy()
                _tp = self.control.joint_target_pos.numpy()
                _tv = self.control.joint_target_vel.numpy()
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
        sub_dt = float(dt) / self._n_substeps
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
        #     teardownWorld() lifecycle hook -- see ~WbSimulationWorld.)
        #  2. _update_mjc_data (newton state -> MuJoCo qpos/qvel copy-in,
        #     gated by solver.update_data_interval) -- only needed on ticks
        #     where the bridge wrote newton state EXTERNALLY (pose reset,
        #     joint-limit clamp, setVelocity, seed, ext wrench). Sites that
        #     mutate newton state set self._mjc_dirty.
        if not hasattr(self, "_skip_collide"):
            import os as _sco
            self._skip_collide = (
                not getattr(self, "_solver_is_xpbd", False)
                and getattr(self.solver, "_use_mujoco_contacts", False)
                and not _sco.environ.get("OMNISIM_NEWTON_KEEP_COLLIDE"))
        _dirty_tick = True
        if (not getattr(self, "_solver_is_xpbd", False)
                and hasattr(self.solver, "update_data_interval")):
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
        # XPBD CUDA-graph opt-in. The MuJoCo path below graphs only EVEN
        # substep counts (so the state_a/state_b ping-pong returns to its start
        # buffer). XPBD is launch-bound at small body counts -- a tick is
        # ~hundreds of tiny kernel launches a replayed graph elides (prototype
        # scripts/xpbd_probes/prototype_xpbd_cudagraph.py: 3.14 -> 0.60 ms/step,
        # 5.3x, bit-identical to the un-graphed step at 1 step; longer-horizon
        # drift stays within the solver's own run-to-run nondeterminism band).
        # We make it graphable for ANY substep count by copying the full
        # post-tick state back into the canonical buffer inside the capture
        # (_use_copyback), so a single (odd) substep also works. Opt-in (default
        # off) until validated against the RL fixtures, since it changes the
        # default substep=1 execution path; enable with OMNISIM_NEWTON_XPBD_GRAPH=1.
        if not hasattr(self, "_xpbd_graph_optin"):
            import os as _xgo
            self._xpbd_graph_optin = bool(_xgo.environ.get("OMNISIM_NEWTON_XPBD_GRAPH"))
        _use_copyback = (getattr(self, "_solver_is_xpbd", False)
                         and self._xpbd_graph_optin)
        _can_graph = (_ext_bf is None
                      and getattr(self, "_stepn", 0) > 10
                      and not getattr(self, "_graph_failed", False)
                      and ((not _dirty_tick
                            and not getattr(self, "_solver_is_xpbd", False)
                            and self._n_substeps % 2 == 0)
                           or _use_copyback))
        if _can_graph and _graph is None:
            import os as _go
            try:
                import warp as _wpg
                if (_go.environ.get("OMNISIM_NEWTON_NO_GRAPH")
                        or "cuda" not in str(self.model.device).lower()):
                    raise RuntimeError("graph disabled or not on cuda")
                with _wpg.ScopedDevice(self.model.device):
                    _wpg.synchronize()
                    _wpg.capture_begin(force_module_load=False)
                    _canon_a, _canon_b = self.state_a, self.state_b
                    try:
                        for _sub in range(self._n_substeps):
                            self.state_a.clear_forces()
                            if contacts is not None and not self._skip_collide:
                                self.model.collide(self.state_a, contacts)
                                self.solver.step(self.state_a, self.state_b,
                                                 self.control, contacts, sub_dt)
                            else:
                                self.solver.step(self.state_a, self.state_b,
                                                 self.control, None, sub_dt)
                            self.state_a, self.state_b = self.state_b, self.state_a
                        if _use_copyback and self.state_a is not _canon_a:
                            # Odd substep count left the result in the OTHER
                            # buffer. Copy the full post-tick state back into the
                            # canonical state_a buffer so every replay reads/
                            # writes the SAME physical buffers and leaves the
                            # result where the post-step readbacks (base guard,
                            # joint clamp, sensors) look for it.
                            if not hasattr(self, "_graph_state_fields"):
                                self._graph_state_fields = [
                                    _nm for _nm in ("body_q", "body_qd", "body_f",
                                                    "joint_q", "joint_qd")
                                    if isinstance(getattr(self.state_a, _nm, None), _wpg.array)
                                    and isinstance(getattr(_canon_a, _nm, None), _wpg.array)]
                            for _nm in self._graph_state_fields:
                                _wpg.copy(getattr(_canon_a, _nm), getattr(self.state_a, _nm))
                    finally:
                        if _use_copyback:
                            # Restore canonical refs: the captured graph leaves
                            # the result in _canon_a, so the Python refs must
                            # match it every tick for downstream readbacks.
                            self.state_a, self.state_b = _canon_a, _canon_b
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
        else:
            for _sub in range(self._n_substeps):
                # clear_forces resets external force accumulation each (sub)step
                # (required by XPBD; see Newton's example_basic_urdf simulate()).
                self.state_a.clear_forces()
                if _ext_bf is not None:
                    self.state_a.body_f.assign(_ext_bf)
                if contacts is not None and not self._skip_collide:
                    self.model.collide(self.state_a, contacts)
                    self.solver.step(self.state_a, self.state_b, self.control, contacts, sub_dt)
                else:
                    if self._skip_collide:
                        self._collide_stale = True
                    self.solver.step(self.state_a, self.state_b, self.control, None, sub_dt)
                self.state_a, self.state_b = self.state_b, self.state_a
        if _ext:
            self._ext_wrench = {}  # consumed this tick; re-applied next tick by the controller
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
        import os as _bnos
        if (self.model is not None and self.pending_revolutes
                and self.slot_to_real_idx
                and _bnos.environ.get(
                    "OMNISIM_NEWTON_DISABLE_JOINT_CLAMP", "0") == "0"):
            try:
                if not hasattr(self, "_q_start_cache"):
                    self._q_start_cache = self.model.joint_q_start.numpy()
                if not hasattr(self, "_qd_start_cache"):
                    self._qd_start_cache = self.model.joint_qd_start.numpy()
                _q_arr = self.state_a.joint_q.numpy()
                _qd_arr = self.state_a.joint_qd.numpy()
                _q_changed = False
                _qd_changed = False
                for _slot, _real in self.slot_to_real_idx.items():
                    _spec = self.pending_revolutes[_slot]
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
                # paying another GPU sync in get_joint_angle(). NOT under
                # XPBD: there joint_q is stale until get_joint_angle's
                # eval_ik refresh, so the clamp's read must not pre-empt it.
                if not getattr(self, "_solver_is_xpbd", False):
                    self._joint_q_cache = _q_arr
                    self._joint_q_cache_fresh = True
                # body_q stays one tick stale (still reflects pre-clamp
                # joint_q). The next solver step re-derives body_q from
                # the now-clamped joint_q, so the staleness self-corrects
                # within one tick. Calling eval_fk here used to crash
                # silently inside the try/except, rolling back the
                # joint_q assign and re-exposing the violation.
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
                    _bqd = self.state_a.body_qd.numpy()
                    self._last_good_body_q = _bq.copy()
                    self._last_good_body_qd = _bqd.copy()
                    # PERF: the guard already paid the post-step body_q /
                    # body_qd syncs -- reuse them as this tick's readback
                    # caches (body_xform/body_vel) instead of re-reading.
                    self._body_q_cache = _bq
                    self._body_qd_cache = _bqd
                    self._body_cache_fresh = True
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
                            print("[WbNewtonBackend] base-divergence guard tripped "
                                  "(body_q non-finite or beyond %.0f m); freezing "
                                  "articulation at last good pose. Set "
                                  "OMNISIM_NEWTON_BASE_GUARD=0 to disable."
                                  % self._base_guard_max, flush=True)
                        except Exception:
                            pass
            except Exception:
                # Never let the guard crash the sim.
                pass

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
                            tv_arr = self.control.joint_target_vel.numpy()
                            tv = {int(_s): round(float(tv_arr[int(qds[_r])]), 3)
                                   for _s, _r in self.slot_to_real_idx.items()
                                   if int(qds[_r]) < len(tv_arr)}
                    except Exception:
                        pass
                    tp = None
                    try:
                        if self.control is not None and hasattr(self.control, "joint_target_pos"):
                            tp_arr = self.control.joint_target_pos.numpy()
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
        # the WbSolid's velocity fields were only ever filled by ODE. The
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
        # WbWgpuShaders::kTriangleInstanced reads vec4<f32> per body,
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


# ===========================================================================
# Sync guard -- NOT part of the verbatim deploy source. These helpers re-read
# the embedded Python block out of WbNewtonBackend.cpp and assert it still
# matches the verbatim body above, so this extract can never drift from the
# deploy without the test going red.
# ===========================================================================
from pathlib import Path as _Path  # noqa: E402

_CPP_PATH = (next(_p for _p in _Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
             / "src" / "omnisim" / "physics" / "WbNewtonBackend.cpp")
_START_MARK = 'kNewtonRuntimeSource = R"PY('
_END_MARK = '\n)PY";'
_GUARD_MARK = "# " + "=" * 75 + "\n# Sync guard"


def _normalize(text):
    """Strip trailing whitespace per line + leading/trailing blank lines so the
    comparison is insensitive to editor whitespace policy and to the single
    newline that separates the docstring from the verbatim body."""
    lines = [ln.rstrip() for ln in text.split("\n")]
    while lines and lines[0] == "":
        lines.pop(0)
    while lines and lines[-1] == "":
        lines.pop()
    return "\n".join(lines)


def deploy_runtime_source_from_cpp(cpp_path=None):
    """Return the verbatim Python block embedded in WbNewtonBackend.cpp."""
    p = _Path(cpp_path) if cpp_path is not None else _CPP_PATH
    cpp = p.read_text(encoding="utf-8")
    i = cpp.index(_START_MARK) + len(_START_MARK)
    assert cpp[i] == "\n", 'expected a newline right after R"PY('
    i += 1
    j = cpp.index(_END_MARK, i)
    return cpp[i:j]


def _this_module_verbatim_body():
    """The verbatim deploy block stored in THIS module: everything between the
    end of the module docstring and the sync-guard delimiter below."""
    src = _Path(__file__).read_text(encoding="utf-8")
    # End of the module docstring: the close of the SECOND triple-quote.
    first = src.index('"""')
    body_start = src.index('"""', first + 3) + 3
    body_end = src.index(_GUARD_MARK)
    return src[body_start:body_end]


def check_in_sync(cpp_path=None):
    """Assert this module's verbatim body == the .cpp embedded block (normalized
    for trailing whitespace). Raises AssertionError with a hint on drift."""
    cpp_block = _normalize(deploy_runtime_source_from_cpp(cpp_path))
    mod_block = _normalize(_this_module_verbatim_body())
    if cpp_block != mod_block:
        cl = cpp_block.split("\n")
        ml = mod_block.split("\n")
        first = next((k for k in range(min(len(cl), len(ml)))
                      if cl[k] != ml[k]), min(len(cl), len(ml)))
        msg = ("g1_deploy_runtime.py has DRIFTED from "
               "WbNewtonBackend.cpp's kNewtonRuntimeSource.\n"
               "  cpp lines=%d module lines=%d\n"
               "  first differing line index=%d\n" % (len(cl), len(ml), first))
        if first < len(cl):
            msg += "  cpp[%d]=%r\n" % (first, cl[first])
        if first < len(ml):
            msg += "  mod[%d]=%r\n" % (first, ml[first])
        raise AssertionError(msg)
    return True


if __name__ == "__main__":
    check_in_sync()
    print("g1_deploy_runtime.py IS IN SYNC with WbNewtonBackend.cpp "
          "kNewtonRuntimeSource.")
