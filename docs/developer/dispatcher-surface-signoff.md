# Dispatcher-surface sign-off — C2 of the architectural baseline

**Purpose:** the architectural baseline ([architectural-baseline.md](architectural-baseline.md) §2 C2)
requires a one-page sign-off that the two backend-dispatch surfaces — `WbPhysicsBackend` and
`WbRenderBackend` — are **closed for extension by ordinary feature work**: no future op (a Newton
capability, a wgpu feature) should require *adding* a new virtual to the abstract surface, only
*overriding* an existing one. This is what makes the structure "final" in the baseline sense.

This doc has two independently-owned halves, mirroring the B1 two-arm split:
- **§1 Physics (`WbPhysicsBackend`)** — owned by the Newton/physics session. ✅ **SIGNED OFF 2026-06-07.**
- **§2 Render (`WbRenderBackend`)** — owned by the render session. ✅ **SIGNED OFF 2026-06-07.**

---

## 1. Physics — `WbPhysicsBackend` — ✅ SIGNED OFF (2026-06-07)

**Verdict: the surface is closed for extension by ordinary feature work.** Newton's capability climb
proceeds by *overriding existing virtuals that currently inherit the `-1` "unsupported" default* — it
adds no new virtuals. The two boundaries that are deliberately *not* on the abstract surface
(world-construction, collision/contact) are stable design choices, documented below, not accidental gaps.

### 1.1 Evidence

[WbPhysicsBackend.hpp](../../src/omnisim/physics/WbPhysicsBackend.hpp) declares the surface:
- **46 `virtual` declarations** = the virtual destructor + 3 pure-virtual lifecycle methods
  (`kind`, `name`, `isAvailable`) + 42 operation methods (`reset` + body pose/velocity/force/config +
  joint read/write/param/lifecycle). The "42 ops" the baseline C2 note refers to.
- **ODE — the reference backend — overrides all 45 non-destructor virtuals.**
  ([WbOdeBackend.hpp](../../src/omnisim/physics/WbOdeBackend.hpp): 45 `override`.) Because ODE
  performs *every* physics op the engine actually exercises and it fully implements the surface, the
  surface is **complete with respect to the engine's physics vocabulary**. There is no op the engine
  does today that lacks a virtual.
- **Newton declares exactly 11 `override`s, inheriting the ~35 remaining operation virtuals.**
  ([WbNewtonBackend.hpp](../../src/omnisim/physics/WbNewtonBackend.hpp): the lifecycle quartet
  (`~`, `kind`, `name`, `isAvailable`) + the 6 read methods
  `getBodyPosition/Quaternion/LinearVel/AngularVel/PointVel` + `getJointHingeAngle` + `reset`.) The
  inherited slots (default `-1` "unsupported"; `reset` defaults to no-op `0`) are exactly the Newton
  capability backlog (force writes, slider/AMotor reads, damping, auto-disable, body mass, joint
  params/lifecycle…). **Each of those is a future *override*, not a future *new virtual*.**
  *(An earlier in-code comment in `WbNewtonBackend.hpp` saying "other 20 methods inherit" was stale — it
  predated the P1.6 joint-op widening that grew the surface; the count is now ~35. Behaviour was
  unchanged; the comment number just lagged. The in-code comment has since been reconciled to the
  current "~35" count.)*

### 1.2 Fixpoint check — no un-dispatched per-entity op remains

The P1.5/P1.6 "gradual widening" moved every per-entity ODE op at the high-level node callsites onto
the dispatcher. A grep for the hot ODE accessors (`dBodyGetPosition/LinearVel/AngularVel`,
`dBodyAddForce/Torque`, `dJointGetHingeAngle/SliderPosition`, `dBodyGetPointVel`) in
[src/omnisim/nodes/](../../src/omnisim/nodes/) returns only **5 residuals, none of which needs a new
virtual**:
- `WbGranularGroup.cpp:285`, `WbHingeJoint.cpp:397` — **comments**, not calls.
- `WbHingeJoint.cpp:362` (`dJointGetHingeAngle`), `WbSliderJoint.cpp:253` (`dJointGetSliderPosition`),
  `WbRotationalMotor.cpp:144` (`dBodyGetPosition`) — each sits inside an **ODE-specific
  spring/damper-motor or ODE joint-feedback (`dJointFeedback`) computation**, and each already has a
  matching virtual on the surface (`getJointHingeAngle`, `getJointSliderPosition`, `getBodyPosition`).
  They stay inline because Newton drives that behaviour through its own PD/target-control bridge
  (`setJointTarget{Position,Velocity}`), not through these read-then-apply ODE idioms. That is
  **capability divergence, not a missing surface**.

### 1.3 The deliberate boundaries (closed by design, not by accident)

Two physics concerns are intentionally *not* on the abstract per-op surface. Stating them is part of
the sign-off so a future contributor doesn't mistake them for gaps to "fix" by widening the surface:

1. **World construction is backend-concrete, by design.** Newton builds its world through its own
   integer-index API (`beginWorld`/`ensureWorldOpen`/`addBody`/`addStaticBody`/`addShape*`/
   `addJointRevolute`/`addJointPrismatic`/`finalizeWorld`/`setSolverPreference`/`setNewtonSubsteps`),
   while ODE constructs through `WbOdeContext`. The abstract surface covers per-entity *operations*
   (read/write a body or joint that already exists), **not** model assembly. This asymmetry is
   deliberate: solver model-building differs fundamentally (ODE incremental vs Newton
   build-then-finalize ping-pong) and forcing it behind one virtual table would be a leaky
   abstraction. A hypothetical third backend would likewise supply its own construction path. **Not a
   surface gap; a stable layer boundary.**
2. **Collision/contact is a separate seam.** Cross-backend contact (ODE static colliders ↔ Newton
   dynamic bodies, baseline §1) is handled by its own bridge at the collision/cluster layer, not via
   per-body virtuals on this surface. That seam is also closed (it routes through the existing
   collision detection, not new dispatch virtuals).

### 1.4 The one bounded exception

`dBodySetMass` (full `dMass`: scalar + center + inertia tensor) is **deliberately called direct**, not
dispatched (see the header comment at `WbPhysicsBackend.hpp` ~line 170). It fires only at world load,
and a generic version would need a parallel `dMass`-mirror struct on the ODE-header-free surface — low
leverage. `getBodyMass` (scalar read, the only field callers actually consume) *is* on the surface.
If a future backend ever needs to intercept full mass *setup*, that is the single op that would add a
virtual — and it is world-load-only, not per-step, and explicitly scoped out today. This is the lone
known place the surface could grow, and it is bounded and documented.

### 1.5 What *would* require a new virtual (and why it's out of baseline scope)

Honesty: the surface is not "closed" against literally everything. It would grow a virtual only if the
engine expanded its **physics vocabulary** — e.g. wiring up an ODE primitive the engine has never used
(a new joint type, a new accessor), or a deliberate future decision to make *world construction*
polymorphic. Neither is "ordinary feature work within the migration": the baseline hosts Newton/wgpu on
the *existing* engine vocabulary, and the Newton capability climb (baseline §4) is defined entirely as
overriding the ~35 currently-inherited operation slots. **Within that scope, the surface is final.**

**Sign-off:** `WbPhysicsBackend` is closed for extension by ordinary feature work. — Newton/physics
session, 2026-06-07.

---

## 2. Render — `WbRenderBackend` — ✅ SIGNED OFF (2026-06-07)

**Verdict: the surface is closed for extension by ordinary feature work — but for a structurally
DIFFERENT reason than physics, and the difference is the point.** Where `WbPhysicsBackend` is a *fat*
vtable (42 operation virtuals, ODE overrides all, Newton overrides a subset), `WbRenderBackend` is a
*thin selection marker* (3 identity virtuals, no operation virtuals); the render operations live in a
shared CONCRETE layer dispatched "select-then-concrete." Ordinary wgpu feature work extends that concrete
layer — shaders, render features, platform surfaces — which is *exactly* the remaining work the baseline
§0.5/§4 define ("a wgpu shader/feature, a platform surface impl"), and none of it adds a virtual to
`WbRenderBackend`.

### 2.1 Evidence — the surface is a thin marker, by design

[WbRenderBackend.hpp](../../src/omnisim/render/WbRenderBackend.hpp) declares **3 pure-virtual methods
beyond the destructor: `kind()`, `name()`, `isAvailable()`** — identity/lookup only, **zero render-op
virtuals**. Both backends implement 100% of it:
- **WREN — the reference backend** ([WbWrenBackend.hpp](../../src/omnisim/render/WbWrenBackend.hpp)):
  overrides all 3 (`kind→Wren`, `name→"wren"`, `isAvailable→true`). It is explicitly a marker — "real WREN
  rendering still happens through the existing call-site code; the backend is only consulted at lookup time
  to decide whether to short-circuit to a non-WREN path."
- **wgpu** ([WbVulkanBackend.hpp](../../src/omnisim/render/WbVulkanBackend.hpp)): overrides all 3
  (`kind→Vulkan`, `name→"vulkan"`, `isAvailable` = a real device probe) and adds **4 NON-virtual concrete
  accessors** — `device()/queue()/instance()/adapter()` — exposing the raw wgpu handles to the render
  machinery. These are NOT new virtuals on the abstract surface (see §2.4).

So unlike physics — where the surface IS the operation vocabulary — the render surface is *only the
selection key*. That is the shipped, deliberate design.

### 2.2 Where the render operations actually live (the deliberate boundary)

The ops the header's R0 comment once reserved for "R1+ operation virtuals" (createFramebuffer,
submitDrawList, renderToTexture) were **never added as virtuals** — the render arm shipped a different,
equally-valid dispatch: **select the backend (the `renderBackend` field → `kind()`/`isAvailable()`), then
drive a shared CONCRETE render layer.** That layer is:
- `WbWgpuSceneRenderer` (`collectWorldDraws`, `buildViewProj`, `ensureTarget`) — the ONE pipeline used by
  BOTH the main view (`WbView3D::renderMainFrameViaWgpu`) and the sensor cameras (`WbCamera`/`WbLidar`/
  `WbRangeFinder`). They share it; no per-path virtual needed.
- `WbWgpuRenderTarget` / `WbWgpuSurface` / `WbWgpuGlBlit` — render target, on-screen surface, and the
  offscreen→GL-blit present path (platform/presentation-specific).
- `WbWgpuMeshCache` / `WbWgpuTextureCache` — per-consumer GPU state.

This is the render twin of physics' two off-surface seams (world-construction, collision/contact): a
deliberate layer boundary, not a gap. The backend answers "which device/queue?" (concrete accessors); the
*operations* are concrete helpers selected after dispatch. A third backend would supply its own concrete
render path the same way.

### 2.3 Fixpoint check — dispatch is centralized; no stray per-op branch needs a virtual

Every render-backend branch keys off the SAME two predicates — the `renderBackend()` field and
`kind()`/`isAvailable()` — resolved through `WbRenderBackendRegistry::resolve()` (with the
`OMNISIM_FORCE_WREN` short-circuit). Callsites:
- **Main view:** `WbView3D::renderMainFrameViaWgpu` (consults `vp->renderBackend()`, checks
  `kind()==Vulkan && isAvailable()`, else byte-identical WREN).
- **Sensors:** `WbCamera`/`WbLidar`/`WbRangeFinder` resolve their backend, then call
  `WbWgpuSceneRenderer::ensureTarget(backend, …)`.
- **GUI pane + probes:** `WbWgpuView`, `main.cpp` probes — the same `vulkanBackend()` lazy-singleton lookup.

None is a per-op branch that "should be a virtual": they are all the same select-then-concrete shape, and
the operation itself lives once, in the shared helper. There is no scattered `if (wgpu) … else …` per
render op.

### 2.4 The render analogue of physics' bounded exception

Physics keeps `dBodySetMass` (a config-shaped op) OFF the surface; render keeps the **raw device handles**
(`device()/queue()/instance()/adapter()`) as concrete accessors on `WbVulkanBackend`, not abstract
virtuals. Render is more hardware-coupled than physics (caches upload through the queue; surfaces need the
instance/adapter), so the backend exposing its concrete resources — while the abstract surface stays a pure
selector — is the correct, bounded asymmetry. WREN needs none of these (it has no device), which is exactly
why they must NOT live on the shared surface.

### 2.5 The one stale artifact (flagged + reconciled, like physics' stale count)

The header comment at `WbRenderBackend.hpp:67-72` still described the original R0 plan — "R1+ widens it to
wrap framebuffer creation, draw-list submission, render-to-texture … one method at a time." The shipped
architecture **superseded that** with select-then-concrete dispatch (the render ops live in
`WbWgpuSceneRenderer`/`WbWgpuSurface`, not on the surface); the surface stayed a thin marker and the arm
shipped fully (main view + sensors + materials + GGX/sRGB + shadows) without those virtuals. That comment
is reconciled to reality in the same change as this sign-off. Formalizing the concrete ops into surface
virtuals later remains *possible* but is a deliberate refactor with no functional gain — **not** ordinary
feature work (the same status as physics' "make world-construction polymorphic").

### 2.6 What WOULD require a new virtual (and why it's out of baseline scope)

`WbRenderBackend` grows a virtual only on a **deliberate decision to move the render-operation vocabulary
onto the surface** (the superseded R1+ plan) — a structural refactor, not feature work. Ordinary wgpu work
in the baseline — lighting/shadow parity, sensor-RTT parity, IBL/TAA, Metal/Linux surfaces, the default
flip — all extend the CONCRETE layer (`WbWgpuSceneRenderer`, `WbWgpuSurface`, shaders) or add a platform
impl, exactly as baseline §0.5 / §4 define the remaining work. **Within that scope, the surface is final.**

*Honest tradeoff:* the thin-marker design gives less compile-time enforcement than physics' fat vtable — a
new backend overrides only `kind/name/isAvailable`, and the real dispatch lives in concrete code a
contributor must extend by hand. That is a real style/enforcement difference, not a "deep architectural
change needed": the structure hosts every remaining render item, which is the baseline's bar.

**Sign-off:** `WbRenderBackend` is closed for extension by ordinary feature work — a thin selection marker
whose render operations live in a shared concrete layer (the deliberate boundary). — Render session, 2026-06-07.
