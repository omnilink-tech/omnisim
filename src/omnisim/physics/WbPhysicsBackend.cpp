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

#include "WbPhysicsBackend.hpp"

#include "WbNewtonBackend.hpp"
#include "WbOdeBackend.hpp"

#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>

WbPhysicsBackendKind WbPhysicsBackendKindFromString(const char *name) {
  if (name == nullptr || *name == '\0')
    return WbPhysicsBackendKind::Ode;  // schema default; user-facing default flip is Phase D
  if (std::strcmp(name, "ode") == 0)
    return WbPhysicsBackendKind::Ode;
  if (std::strcmp(name, "newton") == 0)
    return WbPhysicsBackendKind::Newton;
  if (std::strcmp(name, "auto") == 0)
    return WbPhysicsBackendKind::Auto;
  return WbPhysicsBackendKind::Unknown;
}

// Default "unsupported" impls for the P1.5 widened ops. Backends that
// can answer the query override; backends that can't inherit the -1
// and the caller falls back to whatever it would have done before the
// widening (typically: stay on the direct ODE call path until the
// dispatcher learns to route across backends). Keeping the defaults
// here -- rather than forcing every backend to stub each new op -- is
// what makes gradual widening one-method-at-a-time safe.
int WbPhysicsBackend::getBodyPosition(WbBodyHandle, double[3]) const {
  return -1;
}

int WbPhysicsBackend::setBodyPosition(WbBodyHandle, const double[3]) const {
  return -1;
}

int WbPhysicsBackend::getBodyQuaternion(WbBodyHandle, double[4]) const {
  return -1;
}

int WbPhysicsBackend::setBodyQuaternion(WbBodyHandle, const double[4]) const {
  return -1;
}

int WbPhysicsBackend::getBodyLinearVel(WbBodyHandle, double[3]) const {
  return -1;
}

int WbPhysicsBackend::setBodyLinearVel(WbBodyHandle, const double[3]) const {
  return -1;
}

int WbPhysicsBackend::getBodyAngularVel(WbBodyHandle, double[3]) const {
  return -1;
}

int WbPhysicsBackend::setBodyAngularVel(WbBodyHandle, const double[3]) const {
  return -1;
}

int WbPhysicsBackend::addBodyForceAtPos(WbBodyHandle, const double[3], const double[3]) const {
  return -1;
}

int WbPhysicsBackend::addBodyForceAtRelPos(WbBodyHandle, const double[3], const double[3]) const {
  return -1;
}

int WbPhysicsBackend::addBodyTorque(WbBodyHandle, const double[3]) const {
  return -1;
}

int WbPhysicsBackend::setBodyForce(WbBodyHandle, const double[3]) const {
  return -1;
}

int WbPhysicsBackend::setBodyTorque(WbBodyHandle, const double[3]) const {
  return -1;
}

int WbPhysicsBackend::getBodyPointVel(WbBodyHandle, const double[3], double[3]) const {
  return -1;
}

int WbPhysicsBackend::setBodyEnabled(WbBodyHandle, bool) const {
  return -1;
}

int WbPhysicsBackend::isBodyEnabled(WbBodyHandle) const {
  return -1;
}

int WbPhysicsBackend::getBodyMass(WbBodyHandle, double *) const {
  return -1;
}

int WbPhysicsBackend::setBodyMaxAngularSpeed(WbBodyHandle, double) const {
  return -1;
}

int WbPhysicsBackend::setBodyDamping(WbBodyHandle, double, double) const {
  return -1;
}

int WbPhysicsBackend::setBodyDampingDefaults(WbBodyHandle) const {
  return -1;
}

int WbPhysicsBackend::setBodyAutoDisableFlag(WbBodyHandle, bool) const {
  return -1;
}

int WbPhysicsBackend::setBodyAutoDisableLinearThreshold(WbBodyHandle, double) const {
  return -1;
}

int WbPhysicsBackend::setBodyAutoDisableAngularThreshold(WbBodyHandle, double) const {
  return -1;
}

int WbPhysicsBackend::setBodyAutoDisableTime(WbBodyHandle, double) const {
  return -1;
}

// P1.6 joint-op defaults. Same -1 sentinel as the body ops -- a backend
// that can't read joint angles inherits "unsupported" and callers stay
// on their current direct-call path.
int WbPhysicsBackend::getJointHingeAngle(WbJointHandle, double *) const {
  return -1;
}

int WbPhysicsBackend::getJointHingeAngleRate(WbJointHandle, double *) const {
  return -1;
}

int WbPhysicsBackend::getJointSliderPosition(WbJointHandle, double *) const {
  return -1;
}

int WbPhysicsBackend::getJointAMotorAngle(WbJointHandle, int, double *) const {
  return -1;
}

int WbPhysicsBackend::getJointAMotorAngleRate(WbJointHandle, int, double *) const {
  return -1;
}

int WbPhysicsBackend::addJointHingeTorque(WbJointHandle, double) const {
  return -1;
}

int WbPhysicsBackend::addJointSliderForce(WbJointHandle, double) const {
  return -1;
}

int WbPhysicsBackend::addJointAMotorTorques(WbJointHandle, double, double, double) const {
  return -1;
}

int WbPhysicsBackend::addJointHinge2Torques(WbJointHandle, double, double) const {
  return -1;
}

int WbPhysicsBackend::setJointHingeParam(WbJointHandle, WbJointParam, double) const {
  return -1;
}

int WbPhysicsBackend::setJointSliderParam(WbJointHandle, WbJointParam, double) const {
  return -1;
}

int WbPhysicsBackend::setJointAMotorParam(WbJointHandle, int, WbJointParam, double) const {
  return -1;
}

int WbPhysicsBackend::setJointLMotorParam(WbJointHandle, int, WbJointParam, double) const {
  return -1;
}

int WbPhysicsBackend::setJointHinge2Param(WbJointHandle, int, WbJointParam, double) const {
  return -1;
}

int WbPhysicsBackend::setJointBallParam(WbJointHandle, int, WbJointParam, double) const {
  return -1;
}

int WbPhysicsBackend::reset() {
  // Default: no-op. ODE inherits this directly; Newton overrides to
  // flush Python-side state.
  return 0;
}

int WbPhysicsBackend::setJointEnabled(WbJointHandle, bool) const {
  return -1;
}

int WbPhysicsBackend::isJointEnabled(WbJointHandle) const {
  return -1;
}

const char *WbPhysicsBackendKindToString(WbPhysicsBackendKind kind) {
  switch (kind) {
    case WbPhysicsBackendKind::Ode:
      return "ode";
    case WbPhysicsBackendKind::Newton:
      return "newton";
    case WbPhysicsBackendKind::Auto:
      return "auto";
    case WbPhysicsBackendKind::Unknown:
    default:
      return "unknown";
  }
}

namespace {
  // Two separate once-flags so we can lazy-init Newton: pure-ODE worlds
  // shouldn't pay the embedded-Python startup cost just because the
  // dispatcher exists. The Newton ctor's `import warp; import newton`
  // runs only when somebody actually asks for the Newton backend
  // (resolve(Newton) or newtonBackend()).
  std::once_flag gOdeFlag;
  std::once_flag gNewtonFlag;
  std::unique_ptr<WbPhysicsBackend> gOde;
  std::unique_ptr<WbPhysicsBackend> gNewton;

  void doInitOde() {
    gOde.reset(new WbOdeBackend());
  }

  void doInitNewton() {
    gNewton.reset(new WbNewtonBackend());
  }

  // OMNISIM_FORCE_ODE (any non-empty value) forces every backend
  // resolution to ODE -- even Solids that resolve to "auto" or
  // explicitly "newton". This lets a Newton-ON build reproduce the
  // canonical ODE physics path for deterministic-regression tests.
  // Phase D's `"auto" -> newton` default otherwise silently routes
  // existing ODE-tuned worlds onto Newton (e.g. the accelerometer /
  // contact_points smoke worlds -- the latter is built on ODE-specific
  // ContactProperties softCFM/softERP), whose physics differs from
  // their golden values and breaks the §16 "Newton-ON: existing worlds
  // stay identical" test-matrix row. The deterministic test runner
  // exports this; normal runs never see it. Read once via a function-
  // local static (env is immutable mid-process), so resolve() stays
  // cheap.
  bool forceOde() {
    static const bool kForce = []() {
      // OMNISIM_LEGACY (any non-empty value) is the umbrella legacy switch: it forces
      // BOTH physics->ODE here AND render->WREN in WbRenderBackend's resolve(), returning
      // the whole process to the pre-migration stack with one lever (default-flip-plan.md
      // §3.1). OMNISIM_FORCE_ODE is the physics-only lever. Either one short-circuits to ODE.
      const char *const legacy = std::getenv("OMNISIM_LEGACY");
      if (legacy != nullptr && legacy[0] != '\0')
        return true;
      const char *const v = std::getenv("OMNISIM_FORCE_ODE");
      return v != nullptr && v[0] != '\0';
    }();
    return kForce;
  }
}  // namespace

namespace WbPhysicsBackendRegistry {

  void initialise() {
    // Eager-init ODE only. Newton stays cold until somebody asks for it.
    std::call_once(gOdeFlag, doInitOde);
  }

  WbPhysicsBackend *odeBackend() {
    std::call_once(gOdeFlag, doInitOde);
    return gOde.get();
  }

  WbPhysicsBackend *newtonBackend() {
    std::call_once(gNewtonFlag, doInitNewton);
    return gNewton.get();
  }

  // Returns the requested backend if it's available, else falls back
  // to ODE. This is the load-bearing safety net documented in
  // cuda-newton-physics-plan.md: a world that asks for Newton on a
  // box without CUDA / NVIDIA hardware gets ODE and runs identically.
  // resolve() itself stays cheap and silent; whether a downgrade is a
  // quiet fall-back or a hard error is decided at the world-load
  // enforcement sites via newtonEnforced() (WbSolid / WbBasicJoint
  // flush, WbNewtonBackend::finalizeWorld), NOT here.
  WbPhysicsBackend *resolve(WbPhysicsBackendKind kind) {
    // OMNISIM_FORCE_ODE short-circuits all resolution to ODE (see
    // forceOde() above). Determinism-test escape hatch for Newton-ON
    // builds; inert otherwise.
    if (forceOde())
      return odeBackend();
    switch (kind) {
      case WbPhysicsBackendKind::Newton:
      case WbPhysicsBackendKind::Auto: {
        // Lazy Newton init: paying the Python import cost only here
        // is the difference between "pure-ODE worlds load instantly"
        // and "every omnisim-bin startup spends 200ms warming up CUDA".
        //
        // Newton and Auto share the same resolve path -- both fall
        // back to ODE when Newton is unavailable. The difference is
        // user-facing semantics:
        //   - "newton": user asked specifically; fall-back is a
        //     degraded mode that the caller should log.
        //   - "auto":  user asked "do the right thing"; fall-back is
        //     the expected behavior and no log is needed.
        // The caller (WbSolid::physicsBackend) handles the messaging
        // distinction; this resolver just picks the best available.
        WbPhysicsBackend *newton = newtonBackend();
        if (newton != nullptr && newton->isAvailable())
          return newton;
        return odeBackend();
      }
      case WbPhysicsBackendKind::Ode:
      case WbPhysicsBackendKind::Unknown:
      default:
        return odeBackend();
    }
  }

  bool newtonEnforced() {
    // OMNISIM_FORCE_ODE / OMNISIM_LEGACY: the user explicitly asked for
    // ODE, so there is no Newton requirement to enforce. (These also
    // short-circuit resolve() above, so the Newton backend is never even
    // constructed.)
    if (forceOde())
      return false;
    // OMNISIM_ALLOW_ODE_FALLBACK: dedicated opt-out that restores the
    // pre-2026-06-29 graceful per-articulation fall-back (Newton-
    // unsupported articulations quietly run on ODE) without forcing the
    // WHOLE process onto ODE the way FORCE_ODE does. Read once -- env is
    // immutable mid-process.
    static const bool kAllowFallback = []() {
      const char *const v = std::getenv("OMNISIM_ALLOW_ODE_FALLBACK");
      return v != nullptr && v[0] != '\0';
    }();
    if (kAllowFallback)
      return false;
    // OMNISIM_REQUIRE_NEWTON: explicit assertion -- enforce even if (for
    // some reason) the availability probe below were to disagree. The
    // constructor-time guard in WbNewtonBackend already fatals when the
    // runtime itself fails to load under this flag; this keeps the
    // downstream per-body/per-joint sites consistent with that intent.
    static const bool kRequire = []() {
      const char *const v = std::getenv("OMNISIM_REQUIRE_NEWTON");
      return v != nullptr && v[0] != '\0';
    }();
    if (kRequire)
      return true;
    // Default: enforce whenever the Newton runtime actually came up. A
    // runtime-absent (ODE-only) clone returns false here and keeps
    // running ODE, unaffected; a Newton-capable build can never silently
    // downgrade a fraction of a world to ODE.
    WbPhysicsBackend *const newton = newtonBackend();
    return newton != nullptr && newton->isAvailable();
  }

}  // namespace WbPhysicsBackendRegistry
