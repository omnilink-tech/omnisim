// Copyright 2026 OmniLink
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "WbNewtonBackend.hpp"

#include "WbLog.hpp"

// P2/P3 of cuda-newton-physics-plan.md:
//   - P2: bring up an embedded CPython, import warp + newton.
//   - P3.0: validate the FFI surface beyond imports
//     (newton.ModelBuilder() instantiates from C++).
//   - P3.1 (this file): expose the per-world simulation surface
//     (begin/add/finalize/step/read-back) on WbNewtonBackend itself,
//     backed by an inline `newton_runtime` Python helper module so
//     the C++ side stays one method call away from any Newton API.
//
// Any failure path keeps mAvailable=false (or returns -1 from a method
// call) and the runtime fall-back at WbPhysicsBackendRegistry::resolve
// silently routes the body to ODE -- the safety net the plan promises.
//
// The choice of stable CPython API over pybind11 is deliberate: the
// call surface is small (Py_Initialize, PyImport_ImportModule,
// PyObject_CallMethod, reference counts) and pybind11 would drag in
// a header-only compile-time dependency for no payoff.

#ifdef OMNISIM_WITH_NEWTON
// Wrap Python.h so its <pyconfig.h>'s #define-pragma soup never leaks
// into the rest of the translation unit.
#pragma push_macro("slots")
#undef slots
#include <Python.h>
#pragma pop_macro("slots")
#endif

#include <cstdint>
#include <cstring>

#include <QtCore/QFile>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QString>

// ---- WbBodyHandle pack/unpack ---------------------------------------
//
// Newton body handles encode the body's integer index from addBody() into
// a void* with a +1 offset so that 0 == invalid. Same convention used by
// both ON and OFF builds so the bridge code in WbSolid.cpp can stamp a
// handle on a Newton-resident solid regardless of build flag.

WbBodyHandle WbNewtonBackend::handleFromIndex(int idx) {
  // idx + 1 so 0 stays reserved for "no body". intptr_t round-trip keeps
  // the cast portable across 32/64-bit pointer sizes.
  return reinterpret_cast<WbBodyHandle>(static_cast<uintptr_t>(idx + 1));
}

int WbNewtonBackend::indexFromHandle(WbBodyHandle h) {
  return static_cast<int>(reinterpret_cast<uintptr_t>(h)) - 1;
}

#ifdef OMNISIM_WITH_NEWTON
// Inline Python helper. Lives in its own dictionary so the names don't
// pollute __main__. Edits here MUST be kept in sync with the
// standalone smoke check at newton_embed_smoke.cpp -- both files
// embed the same source verbatim, and divergence between them would
// hide bugs that the smoke check is designed to catch out-of-process.
static const char *kNewtonRuntimeSource = R"PY(
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

    # ---- In-engine MPC (sampling rollouts in the SAME solver the deploy steps) ----
)PY"
R"PY(    def _mpc_log(self, msg):
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
            tp = self.control.joint_target_pos.numpy()
            for _i in range(len(self._bal)):
                _d = int(self._bal_dof[_i])
                if 0 <= _d < len(tp):
                    tp[_d] += float(self._mpc_nom[_i]) + float(self._mpc_ibias[_i])
            self.control.joint_target_pos.assign(tp)
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
        tp = self.control.joint_target_pos.numpy()
        for _i in range(len(self._bal)):
            _d = int(self._bal_dof[_i])
            if 0 <= _d < len(tp):
                tp[_d] += float(dq[_i])
        self.control.joint_target_pos.assign(tp)
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
        M = _np.zeros((nv, nv)); _mj.mj_fullM(mjm, d, M)
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
        # In-engine MPC planner (OMNISIM_INENGINE_MPC=1): override the balance-joint
        # position targets with an MPPI plan rolled out in THIS solver, before the
        # substep loop reads self.control. Falls back silently to the plain hold.
        import os as _impco
        if _impco.environ.get("OMNISIM_INENGINE_MPC"):
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
                      and not _pymod_ss           # per-substep module also recomputes in the loop
                      and not _tsid_on            # TSID recomputes torque per-substep (Python in loop)
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
                    self.model.collide(self.state_a, contacts)
                    self.solver.step(self.state_a, self.state_b, self.control, contacts, sub_dt)
                else:
                    if self._skip_collide:
                        self._collide_stale = True
                    self.solver.step(self.state_a, self.state_b, self.control, None, sub_dt)
                self.state_a, self.state_b = self.state_b, self.state_a
        if _ext:
            self._ext_wrench = {}  # consumed this tick; re-applied next tick by the controller
        # In-engine MPC latency probe (one-shot, gated by OMNISIM_INENGINE_MPC_SELFTEST).
        # Runs after a few warm-up ticks so the live solver state is populated.
        import os as _mpco
        if (not getattr(self, "_mpc_selftest_done", False)
                and _mpco.environ.get("OMNISIM_INENGINE_MPC_SELFTEST")):
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
)PY";
#endif  // OMNISIM_WITH_NEWTON

// The opaque runtime struct lives entirely in this TU so its members
// can name PyObject without the header pulling Python.h.
struct WbNewtonRuntimeState {
#ifdef OMNISIM_WITH_NEWTON
  PyObject *helperGlobals = nullptr;  // dict; owns kNewtonRuntimeSource's namespace
  PyObject *worldClass = nullptr;     // borrowed ref to helper's `World` class
  PyObject *world = nullptr;          // owned ref to a World instance, or nullptr
#endif
  // Tri-state world lifecycle:
  //   - openForBuild=false, running=false -> no world; need beginWorld()
  //   - openForBuild=true,  running=false -> builder accepting addBody/addShape
  //   - openForBuild=false, running=true  -> finalised; step/readback
  bool openForBuild = false;
  bool running = false;

  // P3.2.e: step counter so we can log body-0 position at a few
  // milestone ticks for numerical verification of the readback path.
  // Logs only at hand-picked counts (1, 30, 60, 120) so the output
  // stays bounded; once we trust the path this can be gated behind
  // an env var or removed.
  long long stepCount = 0;
};

#ifdef OMNISIM_WITH_NEWTON
namespace {

  // Logs the active Python error (if any) via WbLog::warning prefixed
  // with `step`, then clears it. Returns -1 unconditionally so call
  // sites can `return reportPyError(...)`.
  int reportPyError(const char *step) {
    if (PyErr_Occurred()) {
      PyObject *type = nullptr, *value = nullptr, *tb = nullptr;
      PyErr_Fetch(&type, &value, &tb);
      PyErr_NormalizeException(&type, &value, &tb);
      QString detail;
      if (value != nullptr) {
        PyObject *str = PyObject_Str(value);
        if (str != nullptr) {
          const char *cstr = PyUnicode_AsUTF8(str);
          if (cstr != nullptr)
            detail = QString::fromUtf8(cstr);
          Py_DECREF(str);
        }
      }
      Py_XDECREF(type);
      Py_XDECREF(value);
      Py_XDECREF(tb);
      WbLog::warning(QString("[WbNewtonBackend] %1 raised: %2").arg(QString::fromUtf8(step)).arg(detail));
    } else {
      WbLog::warning(QString("[WbNewtonBackend] %1 failed (no Python error)").arg(QString::fromUtf8(step)));
    }
    return -1;
  }

  // One-shot bring-up: initialises an embedded CPython if none is
  // running yet, imports warp + newton, runs the FFI smoke check,
  // execs the helper module, caches the World class.
  bool tryInitNewtonRuntime(WbNewtonRuntimeState *runtime) {
    static bool tried = false;
    static bool ok = false;
    if (tried)
      return ok;
    tried = true;

    if (!Py_IsInitialized()) {
      // Py_InitializeEx(0) skips signal handler installation so it
      // doesn't steal SIGINT/SIGTERM from Webots' Qt event loop.
      Py_InitializeEx(0);
      if (!Py_IsInitialized()) {
        WbLog::warning("[WbNewtonBackend] Py_InitializeEx failed; falling back to ODE");
        return false;
      }
    }

    // --- Robust stdio for the embedded interpreter (Newton-default fix) ---
    // warp/newton print a startup banner to sys.stdout during import +
    // ModelBuilder(). Under a headless launch whose stdout is routed to
    // DEVNULL, the embedded interpreter's sys.stdout/sys.stderr come up as
    // None (or a closed fd), so the banner write raises e.g. "'NoneType'
    // object has no attribute 'write'" / "[Errno 9] Bad file descriptor",
    // the FFI smoke below fails, and we SILENTLY FALL BACK TO ODE -- which
    // is what made Newton-configured worlds (e.g. the quadruped walk
    // deploys) intermittently collapse on a default headless run. Ensure
    // Python has writable stdio (devnull when the parent's is None/broken)
    // so the banner write succeeds harmlessly. No-op when stdio is already
    // writable, so GUI / normal-stdout runs are byte-unchanged.
    PyRun_SimpleString(
      "import os as _os, sys as _sys\n"
      "for _n in ('stdout', 'stderr'):\n"
      "    _s = getattr(_sys, _n, None)\n"
      "    _ok = False\n"
      "    if _s is not None:\n"
      "        try:\n"
      "            _s.write(''); _s.flush(); _ok = True\n"
      "        except Exception:\n"
      "            _ok = False\n"
      "    if not _ok:\n"
      "        try: setattr(_sys, _n, open(_os.devnull, 'w'))\n"
      "        except Exception: pass\n");
    PyErr_Clear();

    PyObject *warp = PyImport_ImportModule("warp");
    if (warp == nullptr) {
      PyErr_Clear();
      WbLog::warning("[WbNewtonBackend] `import warp` failed; install with"
                     " `pip install warp-lang`. Falling back to ODE.");
      return false;
    }
    Py_DECREF(warp);

    PyObject *newton = PyImport_ImportModule("newton");
    if (newton == nullptr) {
      PyErr_Clear();
      WbLog::warning("[WbNewtonBackend] `import newton` failed; install with"
                     " `pip install \"newton[examples]\"`. Falling back to ODE.");
      return false;
    }

    // FFI smoke: instantiate newton.ModelBuilder() to confirm the call
    // surface beyond imports works. Throwaway -- the helper module
    // will create its own builder per World.
    PyObject *modelBuilderClass = PyObject_GetAttrString(newton, "ModelBuilder");
    if (modelBuilderClass == nullptr) {
      PyErr_Clear();
      Py_DECREF(newton);
      WbLog::warning("[WbNewtonBackend] newton.ModelBuilder attribute missing;"
                     " API drift? Falling back to ODE.");
      return false;
    }
    PyObject *emptyArgs = PyTuple_New(0);
    PyObject *builder = PyObject_CallObject(modelBuilderClass, emptyArgs);
    Py_DECREF(emptyArgs);
    Py_DECREF(modelBuilderClass);
    if (builder == nullptr) {
      reportPyError("FFI smoke (newton.ModelBuilder())");
      Py_DECREF(newton);
      return false;
    }
    Py_DECREF(builder);
    Py_DECREF(newton);

    // Bring up the helper module in a private dict so its names don't
    // collide with anything else the embedded interpreter is doing.
    PyObject *globals = PyDict_New();
    PyObject *builtins = PyEval_GetBuiltins();  // borrowed
    PyDict_SetItemString(globals, "__builtins__", builtins);
    PyObject *result = PyRun_String(kNewtonRuntimeSource, Py_file_input, globals, globals);
    if (result == nullptr) {
      reportPyError("helper module exec");
      Py_DECREF(globals);
      return false;
    }
    Py_DECREF(result);

    PyObject *worldClass = PyDict_GetItemString(globals, "World");  // borrowed
    if (worldClass == nullptr) {
      WbLog::warning("[WbNewtonBackend] helper module missing `World` class");
      Py_DECREF(globals);
      return false;
    }

    runtime->helperGlobals = globals;  // owned
    runtime->worldClass = worldClass;  // borrowed; safe as long as helperGlobals lives

    WbLog::info("[WbNewtonBackend] warp + newton imports OK; FFI smoke OK"
                " (newton.ModelBuilder()); helper module loaded;"
                " opt-in via `physicsBackend \"newton\"`");
    ok = true;
    return true;
  }

  void releaseWorld(WbNewtonRuntimeState *runtime) {
    if (runtime->world != nullptr) {
      Py_DECREF(runtime->world);
      runtime->world = nullptr;
    }
  }

}  // namespace
#endif  // OMNISIM_WITH_NEWTON

WbNewtonBackend::WbNewtonBackend() : mAvailable(false), mRuntime(nullptr) {
#ifdef OMNISIM_WITH_NEWTON
  mRuntime = new WbNewtonRuntimeState();
  mAvailable = tryInitNewtonRuntime(mRuntime);
  if (!mAvailable) {
    delete mRuntime;
    mRuntime = nullptr;
    // OMNISIM_REQUIRE_NEWTON (opt-in): fail LOUDLY at construction instead of
    // silently degrading to ODE when the Newton runtime won't come up at all.
    // This guards the WHOLE-RUNTIME-missing case (warp/newton import or FFI
    // smoke failure -- e.g. the headless stdout/banner FFI-smoke failure fixed
    // alongside this); it is intentionally opt-in so a genuine ODE-only clone
    // without the bundled runtime is unaffected. The complementary case -- the
    // runtime IS up but an individual articulation / joint / solver would
    // silently downgrade to ODE -- is enforced by DEFAULT on a Newton-capable
    // build via WbPhysicsBackendRegistry::newtonEnforced() (WbSolid /
    // WbBasicJoint flush, finalizeWorld); opt out of that with
    // OMNISIM_ALLOW_ODE_FALLBACK=1. (Note: OMNISIM_FORCE_ODE / OMNISIM_LEGACY
    // short-circuit resolve() before the Newton backend is ever constructed,
    // so an explicit ODE choice still wins and neither guard fires.)
    if (!qEnvironmentVariableIsEmpty("OMNISIM_REQUIRE_NEWTON"))
      WbLog::fatal(
        "[WbNewtonBackend] OMNISIM_REQUIRE_NEWTON is set but the Newton runtime is"
        " unavailable (warp/newton import or FFI smoke failed). Refusing to silently"
        " fall back to ODE. Install the runtime (pip install warp-lang"
        " \"newton[examples]\", or `make bundle-newton-runtime`), or unset"
        " OMNISIM_REQUIRE_NEWTON to allow the ODE fall-back.");
  }
#endif
}

WbNewtonBackend::~WbNewtonBackend() {
#ifdef OMNISIM_WITH_NEWTON
  if (mRuntime != nullptr) {
    releaseWorld(mRuntime);
    if (mRuntime->helperGlobals != nullptr) {
      Py_DECREF(mRuntime->helperGlobals);
      mRuntime->helperGlobals = nullptr;
    }
    delete mRuntime;
    mRuntime = nullptr;
  }
  // We deliberately do NOT Py_Finalize() here. CPython is hostile
  // to Init/Finalize cycles in the same process; the registry's
  // singleton lifetime is the process lifetime, so process teardown
  // reclaims the interpreter cleanly.
#endif
}

#ifdef OMNISIM_WITH_NEWTON

int WbNewtonBackend::beginWorld() {
  if (!mAvailable || mRuntime == nullptr)
    return -1;
  releaseWorld(mRuntime);
  mRuntime->openForBuild = false;
  mRuntime->running = false;
  PyObject *world = PyObject_CallObject(mRuntime->worldClass, nullptr);
  if (world == nullptr)
    return reportPyError("World()");
  mRuntime->world = world;
  mRuntime->openForBuild = true;
  return 0;
}

int WbNewtonBackend::ensureWorldOpen() {
  if (!mAvailable || mRuntime == nullptr)
    return -1;
  if (mRuntime->openForBuild || mRuntime->running)
    return 0;
  if (beginWorld() != 0)
    return -1;
  if (addGroundPlane() != 0)
    return -1;
  WbLog::info("[WbNewtonBackend] world opened (default ground plane added)");
  return 0;
}

void WbNewtonBackend::teardownWorld() {
  if (!mAvailable || mRuntime == nullptr)
    return;
  if (mRuntime->world == nullptr && !mRuntime->openForBuild && !mRuntime->running)
    return;  // nothing to tear down
  releaseWorld(mRuntime);
  mRuntime->openForBuild = false;
  mRuntime->running = false;
  WbLog::info("[WbNewtonBackend] world torn down (next load re-opens a fresh Newton world)");
}

bool WbNewtonBackend::isWorldOpenForBuild() const {
  return mAvailable && mRuntime != nullptr && mRuntime->openForBuild;
}

bool WbNewtonBackend::isWorldRunning() const {
  return mAvailable && mRuntime != nullptr && mRuntime->running;
}

int WbNewtonBackend::addGroundPlane() {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  PyObject *r = PyObject_CallMethod(mRuntime->world, "add_ground_plane", nullptr);
  if (r == nullptr)
    return reportPyError("add_ground_plane");
  Py_DECREF(r);
  return 0;
}

int WbNewtonBackend::addBody(double mass, double x, double y, double z,
                             double qx, double qy, double qz, double qw,
                             double ixx, double iyy, double izz,
                             double ixy, double ixz, double iyz,
                             bool hasCom, double cx, double cy, double cz) {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  // Default path (hasCom=false): the exact 14-arg call this backend has always
  // made -> the Python add_body leaves cx/cy/cz=None -> link COM at the origin
  // (legacy behavior every existing Newton robot is validated against). Only
  // when the caller opts in (OMNISIM_NEWTON_USE_LINK_COM) do we append the true
  // link COM as 3 extra positional args.
  PyObject *r = hasCom
      ? PyObject_CallMethod(mRuntime->world, "add_body",
                            "(ddddddddddddddddd)",
                            mass, x, y, z, qx, qy, qz, qw,
                            ixx, iyy, izz, ixy, ixz, iyz, cx, cy, cz)
      : PyObject_CallMethod(mRuntime->world, "add_body",
                            "(dddddddddddddd)",
                            mass, x, y, z, qx, qy, qz, qw,
                            ixx, iyy, izz, ixy, ixz, iyz);
  if (r == nullptr)
    return reportPyError("add_body");
  long idx = PyLong_AsLong(r);
  Py_DECREF(r);
  return static_cast<int>(idx);
}

int WbNewtonBackend::addStaticBody(double x, double y, double z,
                                   double qx, double qy, double qz, double qw) {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  PyObject *r = PyObject_CallMethod(mRuntime->world, "add_static_body",
                                     "(ddddddd)",
                                     x, y, z, qx, qy, qz, qw);
  if (r == nullptr)
    return reportPyError("add_static_body");
  long idx = PyLong_AsLong(r);
  Py_DECREF(r);
  return static_cast<int>(idx);
}

int WbNewtonBackend::addShapeSphere(int bodyIdx, double radius,
                                    double cx, double cy, double cz) {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  PyObject *r = PyObject_CallMethod(mRuntime->world, "add_shape_sphere", "(idddd)",
                                     bodyIdx, radius, cx, cy, cz);
  if (r == nullptr)
    return reportPyError("add_shape_sphere");
  Py_DECREF(r);
  return 0;
}

int WbNewtonBackend::addShapeBox(int bodyIdx, double hx, double hy, double hz,
                                 double cx, double cy, double cz, double ke) {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  PyObject *r = PyObject_CallMethod(mRuntime->world, "add_shape_box", "(iddddddd)",
                                     bodyIdx, hx, hy, hz, cx, cy, cz, ke);
  if (r == nullptr)
    return reportPyError("add_shape_box");
  Py_DECREF(r);
  return 0;
}

int WbNewtonBackend::setBodyVel(int bodyIdx, double x, double y, double z, int angular) {
  // W3.2: write a Newton body's linear (angular=0) or angular (angular=1) velocity directly into body_qd.
  // Valid DURING simulation (no openForBuild) -- the world + state must exist.
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr)
    return -1;
  PyObject *r = PyObject_CallMethod(mRuntime->world, "set_body_vel", "(idddi)",
                                    bodyIdx, x, y, z, angular);
  if (r == nullptr)
    return reportPyError("set_body_vel");
  Py_DECREF(r);
  return 0;
}

int WbNewtonBackend::addBodyForce(int bodyIdx, double fx, double fy, double fz,
                                  double tx, double ty, double tz) {
  // W3.1: queue a world-frame external wrench for a Newton body (consumed by step() into state.body_f).
  // Valid DURING simulation, so -- unlike the addShape* builders -- it does NOT require openForBuild; the
  // world object just has to exist.
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr)
    return -1;
  PyObject *r = PyObject_CallMethod(mRuntime->world, "add_body_force", "(idddddd)",
                                    bodyIdx, fx, fy, fz, tx, ty, tz);
  if (r == nullptr)
    return reportPyError("add_body_force");
  Py_DECREF(r);
  return 0;
}

int WbNewtonBackend::getContacts(std::vector<WbNewtonContact> &out) const {
  // W4.1/W4.2: pull this step's native rigid contacts from the embedded runtime (get_contacts returns a flat
  // list, 10 values per contact: bodyA,bodyB, point(3), normal(3), depth, |force|). GIL is held on this
  // thread (single embedded interpreter, called from the step thread) -- same as the other Py call methods.
  out.clear();
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr)
    return -1;
  PyObject *r = PyObject_CallMethod(mRuntime->world, "get_contacts", nullptr);
  if (r == nullptr)
    return reportPyError("get_contacts");
  int count = -1;
  if (PyList_Check(r)) {
    const Py_ssize_t len = PyList_Size(r);
    out.reserve((size_t)(len / 10));
    for (Py_ssize_t i = 0; i + 10 <= len; i += 10) {
      WbNewtonContact c;
      c.bodyA = (int)PyLong_AsLong(PyList_GetItem(r, i));
      c.bodyB = (int)PyLong_AsLong(PyList_GetItem(r, i + 1));
      c.point[0] = PyFloat_AsDouble(PyList_GetItem(r, i + 2));
      c.point[1] = PyFloat_AsDouble(PyList_GetItem(r, i + 3));
      c.point[2] = PyFloat_AsDouble(PyList_GetItem(r, i + 4));
      c.normal[0] = PyFloat_AsDouble(PyList_GetItem(r, i + 5));
      c.normal[1] = PyFloat_AsDouble(PyList_GetItem(r, i + 6));
      c.normal[2] = PyFloat_AsDouble(PyList_GetItem(r, i + 7));
      c.depth = PyFloat_AsDouble(PyList_GetItem(r, i + 8));
      c.forceMag = PyFloat_AsDouble(PyList_GetItem(r, i + 9));
      out.push_back(c);
    }
    count = (int)out.size();
  }
  Py_DECREF(r);
  return count;
}

int WbNewtonBackend::addShapeCylinder(int bodyIdx, double radius, double halfHeight,
                                      double cx, double cy, double cz) {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  PyObject *r = PyObject_CallMethod(mRuntime->world, "add_shape_cylinder", "(iddddd)",
                                     bodyIdx, radius, halfHeight, cx, cy, cz);
  if (r == nullptr)
    return reportPyError("add_shape_cylinder");
  Py_DECREF(r);
  return 0;
}

int WbNewtonBackend::addShapeCapsule(int bodyIdx, double radius, double halfHeight) {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  PyObject *r = PyObject_CallMethod(mRuntime->world, "add_shape_capsule", "(idd)",
                                     bodyIdx, radius, halfHeight);
  if (r == nullptr)
    return reportPyError("add_shape_capsule");
  Py_DECREF(r);
  return 0;
}

int WbNewtonBackend::addShapePlane(int bodyIdx, double cx, double cy, double cz) {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  PyObject *r = PyObject_CallMethod(mRuntime->world, "add_shape_plane", "(iddd)", bodyIdx, cx, cy, cz);
  if (r == nullptr)
    return reportPyError("add_shape_plane");
  Py_DECREF(r);
  return 0;
}

// Native triangle-mesh collision (newton-ode-replacement-plan.md W1). vertices = flat 3*nVertices doubles,
// indices = flat 3*nTriangles vertex indices. Marshalled into Python lists once at world load; hold the
// GIL across the PyList/FFI work (the joint wrappers do too).
int WbNewtonBackend::addShapeMesh(int bodyIdx, const double *vertices, int nVertices,
                                  const int *indices, int nTriangles,
                                  double cx, double cy, double cz) {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  if (vertices == nullptr || indices == nullptr || nVertices <= 0 || nTriangles <= 0)
    return -1;
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *pyV = PyList_New(static_cast<Py_ssize_t>(3) * nVertices);
  PyObject *pyI = PyList_New(static_cast<Py_ssize_t>(3) * nTriangles);
  if (pyV == nullptr || pyI == nullptr) {
    Py_XDECREF(pyV);
    Py_XDECREF(pyI);
    PyGILState_Release(gstate);
    return -1;
  }
  for (Py_ssize_t i = 0; i < static_cast<Py_ssize_t>(3) * nVertices; ++i)
    PyList_SetItem(pyV, i, PyFloat_FromDouble(vertices[i]));  // SetItem steals the new ref
  for (Py_ssize_t i = 0; i < static_cast<Py_ssize_t>(3) * nTriangles; ++i)
    PyList_SetItem(pyI, i, PyLong_FromLong(indices[i]));
  PyObject *r = PyObject_CallMethod(mRuntime->world, "add_shape_mesh", "(iOOiddd)",
                                     bodyIdx, pyV, pyI, nVertices, cx, cy, cz);
  Py_DECREF(pyV);
  Py_DECREF(pyI);
  if (r == nullptr) {
    const int err = reportPyError("add_shape_mesh");
    PyGILState_Release(gstate);
    return err;
  }
  Py_DECREF(r);
  PyGILState_Release(gstate);
  return 0;
}

int WbNewtonBackend::addJointRevolute(int parentIdx, int childIdx,
                                      double ax, double ay, double az,
                                      double pX, double pY, double pZ,
                                      double cX, double cY, double cZ,
                                      double targetKe, double targetKd,
                                      double limitLower, double limitUpper,
                                      double effortLimit, double velocityLimit,
                                      double crx, double cry, double crz, double crw) {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  // G1 fix 2026-05-28: explicitly hold the GIL across the FFI call.
  // Without this, the second add_joint_revolute call into the embedded
  // Python interpreter would hang indefinitely on G1's biped articulation
  // — symptom matched a classic GIL contention with a Qt-side Python
  // touchpoint we haven't fully audited. PyGILState_Ensure is idempotent
  // when already held by this thread, so the wrap is safe for callers
  // that already had the GIL.
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *r = PyObject_CallMethod(mRuntime->world, "add_joint_revolute",
                                     "(iiddddddddddddddddddd)",
                                     parentIdx, childIdx,
                                     ax, ay, az,
                                     pX, pY, pZ,
                                     cX, cY, cZ,
                                     targetKe, targetKd,
                                     limitLower, limitUpper,
                                     effortLimit, velocityLimit,
                                     crx, cry, crz, crw);
  if (r == nullptr) {
    const int err = reportPyError("add_joint_revolute");
    PyGILState_Release(gstate);
    return err;
  }
  long idx = PyLong_AsLong(r);
  Py_DECREF(r);
  PyGILState_Release(gstate);
  return static_cast<int>(idx);
}

int WbNewtonBackend::addJointHinge2(int parentIdx, int childIdx,
                                    double ax1, double ay1, double az1,
                                    double ax2, double ay2, double az2,
                                    double pX, double pY, double pZ,
                                    double cX, double cY, double cZ) {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  PyGILState_STATE gstate = PyGILState_Ensure();  // hold the GIL across the FFI call (see addJointRevolute)
  PyObject *r = PyObject_CallMethod(mRuntime->world, "add_joint_hinge2",
                                     "(iidddddddddddd)",
                                     parentIdx, childIdx,
                                     ax1, ay1, az1, ax2, ay2, az2,
                                     pX, pY, pZ,
                                     cX, cY, cZ);
  if (r == nullptr) {
    const int err = reportPyError("add_joint_hinge2");
    PyGILState_Release(gstate);
    return err;
  }
  long idx = PyLong_AsLong(r);
  Py_DECREF(r);
  PyGILState_Release(gstate);
  return static_cast<int>(idx);
}

int WbNewtonBackend::addJointBall(int parentIdx, int childIdx,
                                  double pX, double pY, double pZ,
                                  double cX, double cY, double cZ) {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  PyGILState_STATE gstate = PyGILState_Ensure();  // hold the GIL across the FFI call (see addJointRevolute)
  PyObject *r = PyObject_CallMethod(mRuntime->world, "add_joint_ball",
                                     "(iidddddd)",
                                     parentIdx, childIdx,
                                     pX, pY, pZ,
                                     cX, cY, cZ);
  if (r == nullptr) {
    const int err = reportPyError("add_joint_ball");
    PyGILState_Release(gstate);
    return err;
  }
  long idx = PyLong_AsLong(r);
  Py_DECREF(r);
  PyGILState_Release(gstate);
  return static_cast<int>(idx);
}

int WbNewtonBackend::addJointPrismatic(int parentIdx, int childIdx,
                                       double ax, double ay, double az,
                                       double pX, double pY, double pZ,
                                       double cX, double cY, double cZ,
                                       double targetKe, double targetKd,
                                       double limitLower, double limitUpper,
                                       double effortLimit, double velocityLimit) {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *r = PyObject_CallMethod(mRuntime->world, "add_joint_prismatic",
                                     "(iiddddddddddddddd)",
                                     parentIdx, childIdx,
                                     ax, ay, az,
                                     pX, pY, pZ,
                                     cX, cY, cZ,
                                     targetKe, targetKd,
                                     limitLower, limitUpper,
                                     effortLimit, velocityLimit);
  if (r == nullptr) {
    const int err = reportPyError("add_joint_prismatic");
    PyGILState_Release(gstate);
    return err;
  }
  long idx = PyLong_AsLong(r);
  Py_DECREF(r);
  PyGILState_Release(gstate);
  return static_cast<int>(idx);
}

int WbNewtonBackend::setJointTargetVelocity(int jointIdx, double vel) {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr)
    return -1;
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *r = PyObject_CallMethod(mRuntime->world, "set_joint_target_vel",
                                     "(id)", jointIdx, vel);
  if (r == nullptr) {
    const int err = reportPyError("set_joint_target_vel");
    PyGILState_Release(gstate);
    return err;
  }
  Py_DECREF(r);
  PyGILState_Release(gstate);
  return 0;
}

int WbNewtonBackend::setJointTargetPosition(int jointIdx, double pos) {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr)
    return -1;
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *r = PyObject_CallMethod(mRuntime->world, "set_joint_target_pos",
                                     "(id)", jointIdx, pos);
  if (r == nullptr) {
    const int err = reportPyError("set_joint_target_pos");
    PyGILState_Release(gstate);
    return err;
  }
  Py_DECREF(r);
  PyGILState_Release(gstate);
  return 0;
}

int WbNewtonBackend::setJointForce(int jointIdx, double tau) {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr)
    return -1;
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *r = PyObject_CallMethod(mRuntime->world, "set_joint_force",
                                     "(id)", jointIdx, tau);
  if (r == nullptr) {
    const int err = reportPyError("set_joint_force");
    PyGILState_Release(gstate);
    return err;
  }
  Py_DECREF(r);
  PyGILState_Release(gstate);
  return 0;
}

QString WbNewtonBackend::diagDumpJointQ() const {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->running)
    return QStringLiteral("(newton not running)");
  PyObject *r = PyObject_CallMethod(mRuntime->world, "diag_dump_joint_q", nullptr);
  if (r == nullptr) {
    PyErr_Clear();
    return QStringLiteral("(call failed)");
  }
  const char *cstr = PyUnicode_AsUTF8(r);
  QString s = cstr ? QString::fromUtf8(cstr) : QStringLiteral("(decode failed)");
  Py_DECREF(r);
  return s;
}

void WbNewtonBackend::resetBodyPose(int bodyIdx, double x, double y, double z,
                                    double qx, double qy, double qz, double qw) {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->running)
    return;
  // Format is i + 7 d (bodyIdx + pos[3] + quat[4]). A stray 8th 'd' here used to
  // read a garbage value off the stack so PyObject_CallMethod raised and the
  // call failed SILENTLY every time -- which is why a Supervisor pose-write to a
  // free body never reached Newton (the long-standing "free-body teleport doesn't
  // stick under MuJoCo" symptom). 8 specifiers for 8 args:
  PyObject *r = PyObject_CallMethod(mRuntime->world, "reset_body_pose",
                                     "(iddddddd)",
                                     bodyIdx, x, y, z, qx, qy, qz, qw);
  if (r == nullptr) {
    PyErr_Clear();
    return;
  }
  Py_DECREF(r);
}

void WbNewtonBackend::resetJointsToDefaults() {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->running)
    return;
  PyObject *r = PyObject_CallMethod(mRuntime->world, "reset_joints_to_defaults", nullptr);
  if (r == nullptr) {
    PyErr_Clear();
    return;
  }
  Py_DECREF(r);
}

double WbNewtonBackend::getJointAngle(int jointIdx) const {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->running)
    return 0.0;
  PyObject *r = PyObject_CallMethod(mRuntime->world, "get_joint_angle",
                                     "(i)", jointIdx);
  if (r == nullptr) {
    PyErr_Clear();  // const method — surface the error silently
    return 0.0;
  }
  const double a = PyFloat_AsDouble(r);
  Py_DECREF(r);
  return a;
}

// Drop a race-free backend-verdict sidecar ("<engine-log>.newton.json") stating
// that Newton finalised the world and which solver actually built. env_fingerprint
// reads this in preference to scraping the engine log, so the on-screen physics
// label never mislabels a Newton run as ODE regardless of log size/position.
// WbLog::initFileLog removed any stale prior-run copy when it truncated the log at
// startup, so this file's mere presence == "Newton drove THIS run".
static void writeNewtonVerdictSidecar(const QString &solver) {
  const QString logPath = WbLog::logFilePath();
  if (logPath.isEmpty())
    return;  // no file log this run -> nowhere to co-locate; the log scrape still works
  QJsonObject obj;
  obj.insert(QStringLiteral("backend"), QStringLiteral("newton"));
  obj.insert(QStringLiteral("solver"), solver);
  obj.insert(QStringLiteral("finalised"), true);
  obj.insert(QStringLiteral("degraded"),
             solver.contains(QStringLiteral("FAILED")) || solver.contains(QStringLiteral("XPBD fallback")));
  QFile f(logPath + QStringLiteral(".newton.json"));
  if (f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
    f.write(QJsonDocument(obj).toJson(QJsonDocument::Compact));
    f.close();
  }
}

int WbNewtonBackend::finalizeWorld() {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  PyGILState_STATE gstate = PyGILState_Ensure();
  // Re-assert the sticky solver preference onto THIS world right before
  // finalizing it. The build-phase set_solver_preference() call may have
  // targeted an earlier (now-discarded) world object during a GUI
  // multi-build load; this guarantees the world we're about to finalize
  // sees the requested solver.
  if (!mSolverPref.isEmpty()) {
    PyObject *sp = PyObject_CallMethod(mRuntime->world, "set_solver_preference", "(s)",
                                       mSolverPref.toUtf8().constData());
    if (sp == nullptr)
      PyErr_Clear();
    else
      Py_DECREF(sp);
  }
  PyObject *r = PyObject_CallMethod(mRuntime->world, "finalize", nullptr);
  if (r == nullptr) {
    const int err = reportPyError("finalize");
    PyGILState_Release(gstate);
    return err;
  }
  Py_DECREF(r);
  mRuntime->openForBuild = false;
  mRuntime->running = true;
  // Read back which solver actually got built (Python stores it on the world as
  // _solver_kind / _solver_error) so a SILENT fall-back from the requested
  // MuJoCo (mujoco_warp) engine to XPBD -- a DIFFERENT solver an mujoco_warp-
  // trained RL policy will NOT survive -- is surfaced LOUDLY here instead of
  // hiding in .build_tmp/newton_solver.log (root cause of the G1 deploy gap:
  // a world plane on a non-static body silently failed the mjwarp build).
  QString solverKind, solverErr;
  {
    PyObject *sk = PyObject_GetAttrString(mRuntime->world, "_solver_kind");
    if (sk != nullptr) {
      if (PyUnicode_Check(sk))
        solverKind = QString::fromUtf8(PyUnicode_AsUTF8(sk));
      Py_DECREF(sk);
    } else
      PyErr_Clear();
    PyObject *se = PyObject_GetAttrString(mRuntime->world, "_solver_error");
    if (se != nullptr) {
      if (PyUnicode_Check(se))
        solverErr = QString::fromUtf8(PyUnicode_AsUTF8(se));
      Py_DECREF(se);
    } else
      PyErr_Clear();
  }
  PyGILState_Release(gstate);
  WbLog::info(QString("[WbNewtonBackend] world finalised (solver=%1)")
                  .arg(solverKind.isEmpty()
                           ? QString("see .build_tmp/newton_solver.log")
                           : solverKind));
  writeNewtonVerdictSidecar(solverKind);
  if (solverKind.contains("FAILED") || solverKind.contains("XPBD fallback")) {
    const QString msg =
        QString("[WbNewtonBackend] *** MuJoCo solver was REQUESTED but FAILED to build -- "
                "FELL BACK TO '%1'. XPBD is a DIFFERENT physics engine; an mujoco_warp-trained "
                "policy will behave differently and may collapse. Fix the model so SolverMuJoCo "
                "constructs (details in .build_tmp/newton_solver.log). Error: %2")
            .arg(solverKind)
            .arg(solverErr.isEmpty() ? QString("(none captured)") : solverErr.left(400));
    // Newton enforcement (2026-06-29 default: no silent physics downgrade). A
    // requested-but-failed MuJoCo build silently swapping in XPBD is the same
    // class of bug as a silent Newton->ODE fall-back (it broke the G1 deploy),
    // so escalate it to a hard error under enforcement. Opt out with
    // OMNISIM_ALLOW_ODE_FALLBACK=1 (or FORCE_ODE/LEGACY); the Python runtime's
    // OMNISIM_REQUIRE_MUJOCO_SOLVER=1 asserts the same thing one layer earlier.
    if (WbPhysicsBackendRegistry::newtonEnforced())
      WbLog::fatal(msg);
    else
      WbLog::warning(msg);
  }
  return 0;
}

int WbNewtonBackend::setSolverPreference(const QString &name) {
  // Plumb the WorldInfo.newtonSolver choice to the runtime BEFORE finalize()
  // builds the solver. "mujoco" -> SolverMuJoCo (robust frictional contact for
  // grasps); anything else keeps the default GPU XPBD. Called from WbSolid's
  // Newton flush (build phase), so the world must be open for build.
  // Cache the request regardless of build state; finalizeWorld() re-asserts
  // it onto the world it actually finalizes, so a rebuild that resets the
  // Python-side _solver_pref can't silently drop us to XPBD.
  mSolverPref = name;
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  WbLog::info(QString("[WbNewtonBackend] solver preference set to '%1'").arg(name));
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *r = PyObject_CallMethod(mRuntime->world, "set_solver_preference", "(s)",
                                    name.toUtf8().constData());
  if (r == nullptr) {
    const int err = reportPyError("set_solver_preference");
    PyGILState_Release(gstate);
    return err;
  }
  Py_DECREF(r);
  PyGILState_Release(gstate);
  return 0;
}

int WbNewtonBackend::setNewtonSubsteps(int n) {
  // Plumb the WorldInfo.newtonSubsteps choice to the runtime BEFORE the first
  // step so a contact-heavy world (e.g. a head-on at full drive speed) gets
  // its XPBD sub-steps declaratively in the .wbt instead of via an env var.
  // The OMNISIM_NEWTON_SUBSTEPS env var still wins (resolved in step()).
  // n<=1 is the unchanged single-step path. Called during the build phase.
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr)
    return -1;
  if (n <= 1)
    return 0;  // default; nothing to push
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *r = PyObject_CallMethod(mRuntime->world, "set_substeps", "(i)", n);
  if (r == nullptr) {
    const int err = reportPyError("set_substeps");
    PyGILState_Release(gstate);
    return err;
  }
  Py_DECREF(r);
  PyGILState_Release(gstate);
  return 0;
}

int WbNewtonBackend::step(double dt) {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->running)
    return -1;
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *r = PyObject_CallMethod(mRuntime->world, "step", "(d)", dt);
  if (r == nullptr) {
    const int err = reportPyError("step");
    PyGILState_Release(gstate);
    return err;
  }
  Py_DECREF(r);
  PyGILState_Release(gstate);
  ++mRuntime->stepCount;

  // P3.2.e numerical verification: log body 0's position at hand-picked
  // step counts so we can confirm the trajectory from the WbLog output
  // alone (no controller needed). Free-fall under g=9.81 from z=1.5
  // expected to hit the ground around step 35 at 16ms timesteps;
  // sphere should rest at z ≈ radius (0.12) thereafter.
  const long long c = mRuntime->stepCount;
  if (c == 1 || c == 30 || c == 60 || c == 120 || c == 240 || c == 480
      || c == 960 || c == 1920 || c == 3840 || c == 7680 || c == 15360
      || c == 30720 || c == 61440) {
    QString line = QString("[WbNewtonBackend] step %1 dt=%2s").arg(c).arg(dt);
    // Dump every body's position so we can spot a body that's drifted
    // away from its joint anchor (the "body parts falling off" case).
    // Stops at the first body that doesn't exist; 32-body cap to keep
    // the log line bounded.
    for (int b = 0; b < 32; ++b) {
      PyObject *r = PyObject_CallMethod(mRuntime->world, "body_xform", "(i)", b);
      if (r == nullptr) {
        PyErr_Clear();
        break;
      }
      double bx = 0, by = 0, bz = 0, qx = 0, qy = 0, qz = 0, qw = 0;
      if (PyArg_ParseTuple(r, "ddddddd", &bx, &by, &bz, &qx, &qy, &qz, &qw))
        line += QString(" b%1=(%2,%3,%4)").arg(b)
                    .arg(bx, 0, 'f', 3)
                    .arg(by, 0, 'f', 3)
                    .arg(bz, 0, 'f', 3);
      Py_DECREF(r);
    }
    WbLog::info(line);
  }
  return 0;
}

int WbNewtonBackend::getBodyXform(int bodyIdx, double xform[7]) const {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->running)
    return -1;
  PyObject *r = PyObject_CallMethod(mRuntime->world, "body_xform", "(i)", bodyIdx);
  if (r == nullptr)
    return reportPyError("body_xform");
  double x = 0, y = 0, z = 0, qx = 0, qy = 0, qz = 0, qw = 0;
  if (!PyArg_ParseTuple(r, "ddddddd", &x, &y, &z, &qx, &qy, &qz, &qw)) {
    PyErr_Clear();
    Py_DECREF(r);
    WbLog::warning("[WbNewtonBackend] body_xform: tuple parse failed");
    return -1;
  }
  Py_DECREF(r);
  xform[0] = x;
  xform[1] = y;
  xform[2] = z;
  xform[3] = qx;
  xform[4] = qy;
  xform[5] = qz;
  xform[6] = qw;
  return 0;
}

int WbNewtonBackend::getBodyVelocity(int bodyIdx, double vel[6]) const {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->running)
    return -1;
  PyObject *r = PyObject_CallMethod(mRuntime->world, "body_vel", "(i)", bodyIdx);
  if (r == nullptr)
    return reportPyError("body_vel");
  double vx = 0, vy = 0, vz = 0, wx = 0, wy = 0, wz = 0;
  if (!PyArg_ParseTuple(r, "dddddd", &vx, &vy, &vz, &wx, &wy, &wz)) {
    PyErr_Clear();
    Py_DECREF(r);
    WbLog::warning("[WbNewtonBackend] body_vel: tuple parse failed");
    return -1;
  }
  Py_DECREF(r);
  vel[0] = vx; vel[1] = vy; vel[2] = vz;  // linear (world)
  vel[3] = wx; vel[4] = wy; vel[5] = wz;  // angular (world)
  return 0;
}

int WbNewtonBackend::snapshotBodyTranslations(int maxBodies, float *xyzw) const {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->running)
    return -1;
  if (xyzw == nullptr || maxBodies <= 0)
    return -1;
  PyObject *r = PyObject_CallMethod(mRuntime->world, "body_translations_packed", "(i)", maxBodies);
  if (r == nullptr)
    return reportPyError("body_translations_packed");
  if (!PyBytes_Check(r)) {
    Py_DECREF(r);
    WbLog::warning("[WbNewtonBackend] body_translations_packed: return not bytes");
    return -1;
  }
  const Py_ssize_t n = PyBytes_Size(r);
  if (n < 0 || (n % 16) != 0) {
    Py_DECREF(r);
    WbLog::warning("[WbNewtonBackend] body_translations_packed: bad size");
    return -1;
  }
  const int count = static_cast<int>(n / 16);
  if (count > maxBodies) {
    Py_DECREF(r);
    WbLog::warning("[WbNewtonBackend] body_translations_packed: count > max");
    return -1;
  }
  if (count > 0) {
    const char *src = PyBytes_AsString(r);
    std::memcpy(xyzw, src, static_cast<size_t>(n));
  }
  Py_DECREF(r);
  return count;
}

// ---- Dispatcher overrides (ON path) ---------------------------------

int WbNewtonBackend::getBodyPosition(WbBodyHandle body, double pos[3]) const {
  double xform[7];
  if (getBodyXform(indexFromHandle(body), xform) != 0)
    return -1;
  pos[0] = xform[0];
  pos[1] = xform[1];
  pos[2] = xform[2];
  return 0;
}

int WbNewtonBackend::getBodyQuaternion(WbBodyHandle body, double q[4]) const {
  double xform[7];
  if (getBodyXform(indexFromHandle(body), xform) != 0)
    return -1;
  // Newton stores quaternions as [qx, qy, qz, qw]; the dispatcher's
  // convention follows ODE: q[0]=w, q[1..3]=xyz. Swap on the way out
  // so callers see the same ordering whether the body lives on ODE
  // or Newton.
  q[0] = xform[6];
  q[1] = xform[3];
  q[2] = xform[4];
  q[3] = xform[5];
  return 0;
}

int WbNewtonBackend::getBodyLinearVel(WbBodyHandle body, double v[3]) const {
  double vel[6];
  if (getBodyVelocity(indexFromHandle(body), vel) != 0)
    return -1;
  v[0] = vel[0];
  v[1] = vel[1];
  v[2] = vel[2];
  return 0;
}

int WbNewtonBackend::getBodyAngularVel(WbBodyHandle body, double v[3]) const {
  double vel[6];
  if (getBodyVelocity(indexFromHandle(body), vel) != 0)
    return -1;
  v[0] = vel[3];
  v[1] = vel[4];
  v[2] = vel[5];
  return 0;
}

int WbNewtonBackend::getBodyPointVel(WbBodyHandle body, const double point[3], double v[3]) const {
  // v_point = v_body_origin + omega x (point - body_origin), all in world.
  const int idx = indexFromHandle(body);
  double xform[7];
  if (getBodyXform(idx, xform) != 0)
    return -1;
  double vel[6];
  if (getBodyVelocity(idx, vel) != 0)
    return -1;
  const double rx = point[0] - xform[0];
  const double ry = point[1] - xform[1];
  const double rz = point[2] - xform[2];
  const double wx = vel[3], wy = vel[4], wz = vel[5];
  // cross(omega, r) = (wy*rz - wz*ry, wz*rx - wx*rz, wx*ry - wy*rx)
  v[0] = vel[0] + (wy * rz - wz * ry);
  v[1] = vel[1] + (wz * rx - wx * rz);
  v[2] = vel[2] + (wx * ry - wy * rx);
  return 0;
}

int WbNewtonBackend::getJointHingeAngle(WbJointHandle joint, double *angleOut) const {
  // Newton joint handles share the body-handle packing scheme: idx+1
  // packed into the void*. indexFromHandle is type-agnostic so we can
  // reuse it for the joint index too.
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->running)
    return -1;
  *angleOut = getJointAngle(indexFromHandle(joint));
  return 0;
}

int WbNewtonBackend::reset() {
  // Idempotent no-op when Newton isn't running. When the world IS
  // running, delegate to resetJointsToDefaults() which re-FKs the
  // articulation chain. Per-body pose reset is already handled by the
  // Solid-side syncNewtonPoseFromFields signal cascade; this method
  // covers the joint_q / eval_fk corner that fires only at the root
  // chassis.
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->running)
    return 0;
  resetJointsToDefaults();
  return 0;
}

#else  // !OMNISIM_WITH_NEWTON

// On NEWTON=OFF builds these are no-op stubs returning -1. The
// dispatcher never routes solids here in that mode (mAvailable=false),
// so they're unreachable from production paths -- the stubs exist
// only to keep the linker happy with the public API.

int WbNewtonBackend::beginWorld() { return -1; }
int WbNewtonBackend::ensureWorldOpen() { return -1; }
bool WbNewtonBackend::isWorldOpenForBuild() const { return false; }
bool WbNewtonBackend::isWorldRunning() const { return false; }
int WbNewtonBackend::addGroundPlane() { return -1; }
int WbNewtonBackend::addBody(double, double, double, double, double, double, double, double,
                             double, double, double, double, double, double,
                             bool, double, double, double) { return -1; }
int WbNewtonBackend::addStaticBody(double, double, double, double, double, double, double) { return -1; }
int WbNewtonBackend::addShapeSphere(int, double, double, double, double) { return -1; }
int WbNewtonBackend::addShapeBox(int, double, double, double, double, double, double, double) { return -1; }
int WbNewtonBackend::addShapeCylinder(int, double, double, double, double, double) { return -1; }
int WbNewtonBackend::addBodyForce(int, double, double, double, double, double, double) { return -1; }
int WbNewtonBackend::setBodyVel(int, double, double, double, int) { return -1; }
int WbNewtonBackend::getContacts(std::vector<WbNewtonContact> &out) const { out.clear(); return -1; }
int WbNewtonBackend::addShapeCapsule(int, double, double) { return -1; }
int WbNewtonBackend::addShapePlane(int, double, double, double) { return -1; }
int WbNewtonBackend::addShapeMesh(int, const double *, int, const int *, int,
                                  double, double, double) { return -1; }
int WbNewtonBackend::addJointRevolute(int, int, double, double, double,
                                      double, double, double,
                                      double, double, double,
                                      double, double,
                                      double, double,
                                      double, double,
                                      double, double, double, double) { return -1; }
int WbNewtonBackend::addJointHinge2(int, int, double, double, double,
                                    double, double, double,
                                    double, double, double,
                                    double, double, double) { return -1; }
int WbNewtonBackend::addJointBall(int, int, double, double, double,
                                  double, double, double) { return -1; }
int WbNewtonBackend::addJointPrismatic(int, int, double, double, double,
                                       double, double, double,
                                       double, double, double,
                                       double, double,
                                       double, double,
                                       double, double) { return -1; }
int WbNewtonBackend::setJointTargetVelocity(int, double) { return -1; }
int WbNewtonBackend::setJointTargetPosition(int, double) { return -1; }
int WbNewtonBackend::setJointForce(int, double) { return -1; }
double WbNewtonBackend::getJointAngle(int) const { return 0.0; }
void WbNewtonBackend::resetBodyPose(int, double, double, double, double, double, double, double) {}
void WbNewtonBackend::resetJointsToDefaults() {}
QString WbNewtonBackend::diagDumpJointQ() const { return QStringLiteral("(stub)"); }
int WbNewtonBackend::finalizeWorld() { return -1; }
int WbNewtonBackend::setSolverPreference(const QString &) { return -1; }
int WbNewtonBackend::setNewtonSubsteps(int) { return -1; }
int WbNewtonBackend::step(double) { return -1; }
int WbNewtonBackend::getBodyXform(int, double[7]) const { return -1; }
int WbNewtonBackend::getBodyVelocity(int, double[6]) const { return -1; }
// snapshotBodyTranslations is called UNCONDITIONALLY by WbCamera.cpp + main.cpp
// (runtime `nb ? ... : -1` checks, no #ifdef guard), so the NEWTON=OFF build
// needs this stub or the pure-legacy link fails. Its absence silently broke
// `make OMNISIM_WITH_NEWTON=OFF` — the reversibility escape hatch the Stage 3
// default-flip depends on (default-flip-plan.md §4.4).
int WbNewtonBackend::snapshotBodyTranslations(int, float *) const { return -1; }

// Dispatcher overrides — OFF path stubs. Always return -1, which the
// dispatcher's fall-through layer handles by silently routing to ODE.
int WbNewtonBackend::getBodyPosition(WbBodyHandle, double[3]) const { return -1; }
int WbNewtonBackend::getBodyQuaternion(WbBodyHandle, double[4]) const { return -1; }
int WbNewtonBackend::getBodyLinearVel(WbBodyHandle, double[3]) const { return -1; }
int WbNewtonBackend::getBodyAngularVel(WbBodyHandle, double[3]) const { return -1; }
int WbNewtonBackend::getBodyPointVel(WbBodyHandle, const double[3], double[3]) const { return -1; }
int WbNewtonBackend::getJointHingeAngle(WbJointHandle, double *) const { return -1; }
int WbNewtonBackend::reset() { return 0; }
void WbNewtonBackend::teardownWorld() {}

#endif  // OMNISIM_WITH_NEWTON
