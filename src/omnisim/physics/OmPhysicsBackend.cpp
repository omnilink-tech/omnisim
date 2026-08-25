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

#include "OmPhysicsBackend.hpp"

#include "OmLog.hpp"
#include "OmNewtonBackend.hpp"
#include "OmOdeBackend.hpp"

#include <cstdlib>
#include <chrono>
#include <cstring>
#include <future>
#include <memory>
#include <mutex>

OmPhysicsBackendKind OmPhysicsBackendKindFromString(const char *name) {
  if (name == nullptr || *name == '\0')
    // Auto, NOT Ode. The schema default is "auto" (Solid.wrl / Robot.wrl), so
    // an empty string reaching here means a missing or unset field rather than
    // a choice -- and mapping that to the retired selector would hand out the
    // inert stub, i.e. silently give the Solid no physics at all. Unreachable
    // from a valid world (the SFString is enumerated), which is exactly why it
    // must not be the trap you fall into when something upstream changes.
    return OmPhysicsBackendKind::Auto;
  if (std::strcmp(name, "ode") == 0)
    return OmPhysicsBackendKind::Ode;
  if (std::strcmp(name, "newton") == 0)
    return OmPhysicsBackendKind::Newton;
  if (std::strcmp(name, "auto") == 0)
    return OmPhysicsBackendKind::Auto;
  return OmPhysicsBackendKind::Unknown;
}

// Default "unsupported" impls for the P1.5 widened ops. Backends that
// can answer the query override; backends that can't inherit the -1
// and the caller falls back to whatever it would have done before the
// widening (typically: stay on the direct ODE call path until the
// dispatcher learns to route across backends). Keeping the defaults
// here -- rather than forcing every backend to stub each new op -- is
// what makes gradual widening one-method-at-a-time safe.
int OmPhysicsBackend::getBodyPosition(OmBodyHandle, double[3]) const {
  return -1;
}

int OmPhysicsBackend::setBodyPosition(OmBodyHandle, const double[3]) const {
  return -1;
}

int OmPhysicsBackend::getBodyQuaternion(OmBodyHandle, double[4]) const {
  return -1;
}

int OmPhysicsBackend::setBodyQuaternion(OmBodyHandle, const double[4]) const {
  return -1;
}

int OmPhysicsBackend::getBodyLinearVel(OmBodyHandle, double[3]) const {
  return -1;
}

int OmPhysicsBackend::setBodyLinearVel(OmBodyHandle, const double[3]) const {
  return -1;
}

int OmPhysicsBackend::getBodyAngularVel(OmBodyHandle, double[3]) const {
  return -1;
}

int OmPhysicsBackend::setBodyAngularVel(OmBodyHandle, const double[3]) const {
  return -1;
}

int OmPhysicsBackend::addBodyForceAtPos(OmBodyHandle, const double[3], const double[3]) const {
  return -1;
}

int OmPhysicsBackend::addBodyForceAtRelPos(OmBodyHandle, const double[3], const double[3]) const {
  return -1;
}

int OmPhysicsBackend::addBodyTorque(OmBodyHandle, const double[3]) const {
  return -1;
}

int OmPhysicsBackend::setBodyForce(OmBodyHandle, const double[3]) const {
  return -1;
}

int OmPhysicsBackend::setBodyTorque(OmBodyHandle, const double[3]) const {
  return -1;
}

int OmPhysicsBackend::getBodyPointVel(OmBodyHandle, const double[3], double[3]) const {
  return -1;
}

int OmPhysicsBackend::setBodyEnabled(OmBodyHandle, bool) const {
  return -1;
}

int OmPhysicsBackend::isBodyEnabled(OmBodyHandle) const {
  return -1;
}

int OmPhysicsBackend::getBodyMass(OmBodyHandle, double *) const {
  return -1;
}

int OmPhysicsBackend::setBodyMaxAngularSpeed(OmBodyHandle, double) const {
  return -1;
}

int OmPhysicsBackend::setBodyDamping(OmBodyHandle, double, double) const {
  return -1;
}

int OmPhysicsBackend::setBodyDampingDefaults(OmBodyHandle) const {
  return -1;
}

int OmPhysicsBackend::setBodyAutoDisableFlag(OmBodyHandle, bool) const {
  return -1;
}

int OmPhysicsBackend::setBodyAutoDisableLinearThreshold(OmBodyHandle, double) const {
  return -1;
}

int OmPhysicsBackend::setBodyAutoDisableAngularThreshold(OmBodyHandle, double) const {
  return -1;
}

int OmPhysicsBackend::setBodyAutoDisableTime(OmBodyHandle, double) const {
  return -1;
}

// P1.6 joint-op defaults. Same -1 sentinel as the body ops -- a backend
// that can't read joint angles inherits "unsupported" and callers stay
// on their current direct-call path.
int OmPhysicsBackend::getJointHingeAngle(OmJointHandle, double *) const {
  return -1;
}

int OmPhysicsBackend::getJointHingeAngleRate(OmJointHandle, double *) const {
  return -1;
}

int OmPhysicsBackend::getJointSliderPosition(OmJointHandle, double *) const {
  return -1;
}

int OmPhysicsBackend::getJointAMotorAngle(OmJointHandle, int, double *) const {
  return -1;
}

int OmPhysicsBackend::getJointAMotorAngleRate(OmJointHandle, int, double *) const {
  return -1;
}

int OmPhysicsBackend::addJointHingeTorque(OmJointHandle, double) const {
  return -1;
}

int OmPhysicsBackend::addJointSliderForce(OmJointHandle, double) const {
  return -1;
}

int OmPhysicsBackend::addJointAMotorTorques(OmJointHandle, double, double, double) const {
  return -1;
}

int OmPhysicsBackend::addJointHinge2Torques(OmJointHandle, double, double) const {
  return -1;
}

int OmPhysicsBackend::setJointHingeParam(OmJointHandle, OmJointParam, double) const {
  return -1;
}

int OmPhysicsBackend::setJointSliderParam(OmJointHandle, OmJointParam, double) const {
  return -1;
}

int OmPhysicsBackend::setJointAMotorParam(OmJointHandle, int, OmJointParam, double) const {
  return -1;
}

int OmPhysicsBackend::setJointLMotorParam(OmJointHandle, int, OmJointParam, double) const {
  return -1;
}

int OmPhysicsBackend::setJointHinge2Param(OmJointHandle, int, OmJointParam, double) const {
  return -1;
}

int OmPhysicsBackend::setJointBallParam(OmJointHandle, int, OmJointParam, double) const {
  return -1;
}

int OmPhysicsBackend::reset() {
  // Default: no-op. ODE inherits this directly; Newton overrides to
  // flush Python-side state.
  return 0;
}

int OmPhysicsBackend::setJointEnabled(OmJointHandle, bool) const {
  return -1;
}

int OmPhysicsBackend::isJointEnabled(OmJointHandle) const {
  return -1;
}

const char *OmPhysicsBackendKindToString(OmPhysicsBackendKind kind) {
  switch (kind) {
    case OmPhysicsBackendKind::Ode:
      return "ode";
    case OmPhysicsBackendKind::Newton:
      return "newton";
    case OmPhysicsBackendKind::Auto:
      return "auto";
    case OmPhysicsBackendKind::Unknown:
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
  std::unique_ptr<OmPhysicsBackend> gOde;
  std::unique_ptr<OmPhysicsBackend> gNewton;
  std::mutex gNewtonPreloadMutex;
  std::shared_future<bool> gNewtonPreload;
  std::chrono::steady_clock::time_point gNewtonPreloadStarted;

  bool asyncPreloadDisabled() {
    const char *const value = std::getenv("OMNISIM_NEWTON_ASYNC_PRELOAD");
    if (value == nullptr)
      return false;
    return std::strcmp(value, "0") == 0 || std::strcmp(value, "false") == 0 || std::strcmp(value, "off") == 0;
  }

  void awaitNewtonPreload() {
    std::shared_future<bool> preload;
    std::chrono::steady_clock::time_point started;
    {
      const std::lock_guard<std::mutex> lock(gNewtonPreloadMutex);
      preload = gNewtonPreload;
      started = gNewtonPreloadStarted;
    }
    if (!preload.valid())
      return;
    const auto waitStarted = std::chrono::steady_clock::now();
    const bool ok = preload.get();
    OmNewtonBackend::adoptAsyncPreloadedRuntime();
    if (!qEnvironmentVariableIsEmpty("OMNISIM_NEWTON_PRELOAD_PROFILE")) {
      const auto now = std::chrono::steady_clock::now();
      const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - started).count();
      const auto waited = std::chrono::duration_cast<std::chrono::milliseconds>(now - waitStarted).count();
      OmLog::info(QString("[runtime-cycle] Newton preload %1 in %2 ms; blocking wait %3 ms")
                    .arg(ok ? "succeeded" : "failed").arg(elapsed).arg(waited));
    }
  }

  void doInitOde() {
    gOde.reset(new OmOdeBackend());
  }

  void doInitNewton() {
    awaitNewtonPreload();
    gNewton.reset(new OmNewtonBackend());
  }

  // RETIRED ODE-SELECTION SWITCHES.
  //
  // OMNISIM_FORCE_ODE used to force every backend resolution to ODE (even
  // Solids that resolved to "auto" or explicitly "newton") so a Newton-capable
  // build could reproduce the canonical ODE physics path for deterministic
  // regression; OMNISIM_LEGACY was the umbrella that did that AND forced
  // render->WREN; OMNISIM_ALLOW_ODE_FALLBACK let a Newton-unsupported
  // articulation degrade to ODE instead of being refused.
  //
  // None of them can do anything now: src/ode was deleted and Newton is the
  // only physics backend. OmOdeBackend survives only as an inert dispatcher
  // stub whose every op returns -1, so honouring these variables would hand
  // the caller a world with NO physics rather than legacy physics.
  //
  // They are IGNORED, but not silently: a determinism oracle that exports
  // OMNISIM_FORCE_ODE and believes it received an ODE run would draw a wrong
  // conclusion from a Newton one, and in this tree a wrong result is worse
  // than a lost one. So each set variable produces one warning naming itself.
  // Read once via a function-local static (env is immutable mid-process), so
  // the resolution path stays cheap.
  //
  // OMNISIM_LEGACY's RENDER arm is retired too (F1, wren-deletion-runbook.md):
  // OmRenderBackendRegistry::resolve() warns about it separately and resolves to
  // wgpu regardless (warnRetiredWrenSelectors in OmRenderBackend.cpp).
  void warnRetiredOdeSelectors() {
    static const bool kWarned = []() {
      struct RetiredVar {
        const char *name;
        const char *usedTo;
      };
      static const RetiredVar kRetired[] = {
        {"OMNISIM_FORCE_ODE", "force every Solid onto the ODE backend"},
        {"OMNISIM_LEGACY", "force physics onto ODE (its render arm is retired too since F1: rendering resolves "
                           "to wgpu, see OmRenderBackend's own warning)"},
        {"OMNISIM_ALLOW_ODE_FALLBACK", "let a Newton-unsupported articulation quietly degrade to ODE"}};
      for (const RetiredVar &r : kRetired) {
        const char *const v = std::getenv(r.name);
        if (v != nullptr && v[0] != '\0')
          OmLog::warning(QString("[physics] %1 is set but RETIRED and IGNORED: it used to %2. ODE has been "
                                 "deleted and Newton is the only physics backend, so there is nothing to "
                                 "select. Unset the variable -- this run is Newton either way.")
                           .arg(QString::fromLatin1(r.name))
                           .arg(QString::fromLatin1(r.usedTo)));
      }
      return true;
    }();
    (void)kWarned;
  }
}  // namespace

namespace OmPhysicsBackendRegistry {

  void initialise() {
    // Eager-init ODE only. Newton stays cold until somebody asks for it.
    std::call_once(gOdeFlag, doInitOde);
  }

  void startNewtonRuntimePreload() {
    if (asyncPreloadDisabled())
      return;
    const std::lock_guard<std::mutex> lock(gNewtonPreloadMutex);
    if (gNewton || gNewtonPreload.valid())
      return;
    gNewtonPreloadStarted = std::chrono::steady_clock::now();
    gNewtonPreload = std::async(std::launch::async, []() { return OmNewtonBackend::preloadRuntimeAsyncWorker(); }).share();
  }

  void waitForNewtonRuntimePreload() {
    awaitNewtonPreload();
  }

  OmPhysicsBackend *odeBackend() {
    std::call_once(gOdeFlag, doInitOde);
    return gOde.get();
  }

  OmPhysicsBackend *newtonBackend() {
    std::call_once(gNewtonFlag, doInitNewton);
    return gNewton.get();
  }

  // Returns the requested backend if it's available, else the ODE
  // dispatcher stub. ⚠ That stub is NO LONGER A WORKING FALL-BACK: src/ode is
  // gone, so every OmOdeBackend op returns -1 and a world routed here has no
  // physics at all. The safety net that used to make "Newton unavailable ->
  // ODE and runs identically" true is therefore also gone; the only thing
  // standing between an absent Newton runtime and a physics-free world is the
  // enforcement decided at the world-load sites via newtonEnforced() (OmSolid
  // / OmBasicJoint flush, OmNewtonBackend::finalizeWorld), NOT here.
  // resolve() itself stays cheap and silent.
  OmPhysicsBackend *resolve(OmPhysicsBackendKind kind) {
    warnRetiredOdeSelectors();
    switch (kind) {
      case OmPhysicsBackendKind::Newton:
      case OmPhysicsBackendKind::Auto: {
        // Lazy Newton init: paying the embedded-Python import cost only
        // here (not at every omnisim-bin startup) is why the ctor is not
        // eager.
        //
        // Newton and Auto share the same resolve path -- both land on the
        // inert ODE stub when the Newton runtime is unavailable. The
        // difference is user-facing semantics:
        //   - "newton": user asked specifically; a downgrade is an error
        //     the caller should report.
        //   - "auto":  user asked "do the right thing"; historically the
        //     downgrade was expected and unlogged.
        // The caller (OmSolid::physicsBackend) handles the messaging
        // distinction; this resolver just picks the best available.
        OmPhysicsBackend *newton = newtonBackend();
        if (newton != nullptr && newton->isAvailable())
          return newton;
        // ⚠ NO PHYSICS BACKEND EXISTS FOR THIS SOLID, AND THAT USED TO BE FINE.
        // Before ODE was deleted (bdc02139) this fall-through handed the Solid a
        // working CPU solver, so staying silent was correct. What it returns now
        // is OmOdeBackend's inert stub -- every verb returns -1 -- so the Solid
        // gets no gravity, no contact and no joints, while the world loads
        // clean. A clone that simply forgot `make -C src/omnisim
        // bundle-newton-runtime` would therefore watch every demo stand still
        // with nothing in the log to explain it.
        //
        // Reported once per process at ERROR severity, deliberately: an ERROR
        // takes a headless run's exit code to 1, so CI and `run-headless` fail
        // loudly instead of certifying a frozen scene, while the GUI still opens
        // the world so someone can look at it and read the message. Not a FATAL
        // because refusing to load leaves the operator with less information
        // than a world they can inspect plus a sentence telling them why nothing
        // moves.
        static bool sWarnedNoBackend = false;
        if (!sWarnedNoBackend) {
          sWarnedNoBackend = true;
          OmLog::error(QObject::tr(
            "[physics] NO PHYSICS BACKEND IS AVAILABLE: the Newton runtime did not come up, and ODE has been "
            "removed, so there is nothing left to simulate with. Every Solid in this world will have no gravity, "
            "no contact and no joint dynamics -- the scene will load and then stand still. This is a broken "
            "install, not a world bug: run `python -m omnisim doctor`, and on Windows stage the runtime with "
            "`make -C src/omnisim bundle-newton-runtime` (on Linux, pip install torch warp-lang newton mujoco "
            "mujoco-warp into the SYSTEM python3, not a venv)."), false, OmLog::ODE);
        }
        return odeBackend();
      }
      case OmPhysicsBackendKind::Ode:
      case OmPhysicsBackendKind::Unknown:
      default:
        return odeBackend();
    }
  }

  bool newtonEnforced() {
    // OMNISIM_FORCE_ODE / OMNISIM_LEGACY / OMNISIM_ALLOW_ODE_FALLBACK all used
    // to switch enforcement OFF, because each of them named a legitimate way to
    // end up on ODE. There is no such way any more, so none of them may relax
    // enforcement: "let it degrade to ODE" now means "let it run with no
    // physics", which is exactly the wrong-result-instead-of-lost-result
    // failure the enforcement exists to prevent. The variables are warned about
    // and ignored (warnRetiredOdeSelectors above).
    warnRetiredOdeSelectors();
    // OMNISIM_REQUIRE_NEWTON: explicit assertion -- enforce even if (for
    // some reason) the availability probe below were to disagree. The
    // constructor-time guard in OmNewtonBackend already fatals when the
    // runtime itself fails to load under this flag; this keeps the
    // downstream per-body/per-joint sites consistent with that intent.
    static const bool kRequire = []() {
      const char *const v = std::getenv("OMNISIM_REQUIRE_NEWTON");
      return v != nullptr && v[0] != '\0';
    }();
    if (kRequire)
      return true;
    // Default: enforce whenever the Newton runtime actually came up, so a
    // Newton-capable build can never silently downgrade a fraction of a world.
    // ⚠ A clone with NO Newton runtime still returns false here -- but it no
    // longer "keeps running ODE, unaffected", because there is no ODE. It runs
    // on the inert OmOdeBackend stub, i.e. with no physics. That case is only
    // caught if the operator sets OMNISIM_REQUIRE_NEWTON; making it loud by
    // default is a behaviour change the owner has to call.
    OmPhysicsBackend *const newton = newtonBackend();
    return newton != nullptr && newton->isAvailable();
  }

}  // namespace OmPhysicsBackendRegistry
