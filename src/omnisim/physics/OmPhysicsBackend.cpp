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

#include <cstdlib>
#include <chrono>
#include <cstring>
#include <future>
#include <memory>
#include <mutex>

OmPhysicsBackendKind OmPhysicsBackendKindFromString(const char *name) {
  if (name == nullptr || *name == '\0')
    // The schema default is "auto" (Solid.wrl / Robot.wrl), so an empty string
    // reaching here means a missing or unset field rather than a choice.
    return OmPhysicsBackendKind::Auto;
  // "ode" is deliberately NOT recognised: it is a retired selector and falls
  // through to Unknown, which resolve() maps to Newton like everything else.
  if (std::strcmp(name, "newton") == 0)
    return OmPhysicsBackendKind::Newton;
  if (std::strcmp(name, "auto") == 0)
    return OmPhysicsBackendKind::Auto;
  return OmPhysicsBackendKind::Unknown;
}

// Default "unsupported" impls. The backend overrides what it can answer and
// inherits the -1 for the rest, so callers can branch on the return code
// without knowing which ops a given build supports.
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

// Joint-op defaults. Same -1 sentinel as the body ops.
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
  // Default: no-op. Newton overrides to flush Python-side state.
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
  // Lazy init: the Newton ctor's `import warp; import newton` runs only when
  // somebody actually asks for the backend (resolve() or newtonBackend()),
  // normally after startNewtonRuntimePreload() has overlapped the import with
  // world parsing.
  std::once_flag gNewtonFlag;
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

  void doInitNewton() {
    awaitNewtonPreload();
    gNewton.reset(new OmNewtonBackend());
  }
}  // namespace

namespace OmPhysicsBackendRegistry {

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

  OmPhysicsBackend *newtonBackend() {
    std::call_once(gNewtonFlag, doInitNewton);
    return gNewton.get();
  }

  // Every kind resolves to Newton: "newton" and "auto" by definition, and
  // Unknown (which is where the retired "ode" lands) because a Solid must never
  // be handed to no solver by naming a backend that does not exist -- that was
  // the trap closed on 2026-09-02. OmSolid owns the once-per-world warning for
  // retired values; this resolver stays cheap and silent about them.
  //
  // When the Newton runtime is UNAVAILABLE this still returns the (non-null)
  // Newton object: its isAvailable() is false, nothing registers a body with
  // it, and every handle-keyed op returns -1 -- callers such as OmGyro
  // dereference the result without a null check. The world loads and stands
  // still, and the message below says so.
  OmPhysicsBackend *resolve(OmPhysicsBackendKind kind) {
    (void)kind;
    OmPhysicsBackend *const newton = newtonBackend();
    if (newton != nullptr && newton->isAvailable())
      return newton;
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
        "[physics] NO PHYSICS BACKEND IS AVAILABLE: the Newton runtime did not come up, and Newton is the only "
        "backend, so there is nothing to simulate with. Every Solid in this world will have no gravity, "
        "no contact and no joint dynamics -- the scene will load and then stand still. This is a broken "
        "install, not a world bug: run `python -m omnisim doctor`, and on Windows stage the runtime with "
        "`make -C src/omnisim bundle-newton-runtime` (on Linux, pip install torch warp-lang newton mujoco "
        "mujoco-warp into the SYSTEM python3, not a venv)."), false, OmLog::ODE);
    }
    return newton;
  }

  bool newtonEnforced() {
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
    // Newton-capable build can never silently drop a fraction of a world.
    // ⚠ A clone with NO Newton runtime returns false here and runs with no
    // physics (resolve() above logs the ERROR). OMNISIM_REQUIRE_NEWTON makes
    // that a FATAL; making it loud by default is the owner's call.
    OmPhysicsBackend *const newton = newtonBackend();
    return newton != nullptr && newton->isAvailable();
  }

}  // namespace OmPhysicsBackendRegistry
