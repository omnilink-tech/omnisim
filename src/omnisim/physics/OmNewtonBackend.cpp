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

#include "OmNewtonBackend.hpp"

#include "OmLog.hpp"

// P2/P3 of cuda-newton-physics-plan.md:
//   - P2: bring up an embedded CPython, import warp + newton.
//   - P3.0: validate the FFI surface beyond imports
//     (newton.ModelBuilder() instantiates from C++).
//   - P3.1 (this file): expose the per-world simulation surface
//     (begin/add/finalize/step/read-back) on OmNewtonBackend itself,
//     backed by the bundled `omnisim_newton_runtime` Python helper module so
//     the C++ side stays one method call away from any Newton API.
//
// Any failure path keeps mAvailable=false (or returns -1 from a method call).
// There is NO safety net any more: ODE was deleted, so resolve() hands out an
// inert tombstone and logs one error -- the world loads and stands still. That
// is deliberate; a wrong result is worse than a lost one.
//
// The choice of stable CPython API over pybind11 is deliberate: the
// call surface is small (Py_Initialize, PyImport_ImportModule,
// PyObject_CallMethod, reference counts) and pybind11 would drag in
// a header-only compile-time dependency for no payoff.

#ifdef OMNISIM_WITH_NEWTON
#include <stdio.h>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#ifdef _WIN32
#include <io.h>
#else
#include <unistd.h>
#include <fcntl.h>
#endif
// Wrap Python.h so its <pyconfig.h>'s #define-pragma soup never leaks
// into the rest of the translation unit.
#pragma push_macro("slots")
#undef slots
#include <Python.h>
#pragma pop_macro("slots")
#endif

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <fstream>
#include <string>

// Deliberately no direct Qt includes (core-evolution-plan.md, Phase Q2): physics/ is
// future-core code. Messages are built as std::string and converted to the logger's
// QString (which arrives transitively through OmLog.hpp) only at the OmLog boundary.

// ---- OmBodyHandle pack/unpack ---------------------------------------
//
// Newton body handles encode the body's integer index from addBody() into
// a void* with a +1 offset so that 0 == invalid. Same convention used by
// both ON and OFF builds so the bridge code in OmSolid.cpp can stamp a
// handle on a Newton-resident solid regardless of build flag.

OmBodyHandle OmNewtonBackend::handleFromIndex(int idx) {
  // idx + 1 so 0 stays reserved for "no body". intptr_t round-trip keeps
  // the cast portable across 32/64-bit pointer sizes.
  return reinterpret_cast<OmBodyHandle>(static_cast<uintptr_t>(idx + 1));
}

int OmNewtonBackend::indexFromHandle(OmBodyHandle h) {
  return static_cast<int>(reinterpret_cast<uintptr_t>(h)) - 1;
}

// The Python implementation lives in omnisim_newton_runtime.py. Keeping it out
// of this translation unit makes runtime-only edits build-free and avoids
// recompiling/relinking a 5,000-line C++ source whenever Python glue changes.

// The opaque runtime struct lives entirely in this TU so its members
// can name PyObject without the header pulling Python.h.
struct OmNewtonRuntimeState {
#ifdef OMNISIM_WITH_NEWTON
  PyObject *helperModule = nullptr;   // owns the imported omnisim_newton_runtime module
  PyObject *worldClass = nullptr;     // owned reference to helperModule.World
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

  // N15/N16 diagnostics latches. Both exist purely so a warning is emitted
  // ONCE per world instead of once per tick / once per builder flush; neither
  // is read by any physics path. Reset in beginWorld().
  bool constraintOverflowLogged = false;  // N15: runtime nefc/ncon overflow
  bool autoConstraintCapWarned = false;   // N16: newtonNjmax/Nconmax -1 resolve
  // Same shape, but this one latches an ERROR rather than a warning: a world
  // whose finalize() raised is retried on every tick (nothing was built, so it
  // never closes for build), and the measured SolidReference case emitted 4254
  // copies of the same line. Reported once per world; reset in beginWorld().
  bool finalizeFailureReported = false;

  // Tier 1a step-readback snapshot (physics-step-cost-optimization-plan.md):
  // filled by the first getBodyXform / getBodyVelocity / getJointAngle after
  // a step() via ONE readback_packed() crossing; every later per-body /
  // per-joint read that tick is a plain array lookup instead of a
  // PyObject_CallMethod round-trip (2 crossings per Solid + 1 per hinge
  // sensor per tick before this). Invalidated at the end of step() --
  // mirroring the python-side per-step cache lifetime -- and by every state
  // mutator (resetBodyPose, setBodyVel, setKinematicPose,
  // resetJointsToDefaults, world lifecycle), so a post-teleport read
  // refetches instead of serving the pre-teleport pose.
  bool snapValid = false;
  int snapBodyCount = 0;
  int snapSlotCount = 0;
  std::vector<double> snapBody;       // snapBodyCount x 13: [x y z qx qy qz qw vx vy vz wx wy wz]
  std::vector<double> snapSlotAngle;  // revolute angle per joint slot id

  // Last value pushed per joint slot for the change-detected target setters
  // (physics-step-cost-optimization-plan.md §3 item 5). NaN = never pushed, so
  // the first write of a run always crosses.
  std::vector<double> lastTargetVel;
  std::vector<double> lastTargetPos;
};

#ifdef OMNISIM_WITH_NEWTON
// Tier 1a: defined next to the readback getters (getBodyXform et al.);
// declared here because getJointAngle precedes the definition.
static bool ensureStepSnapshot(OmNewtonRuntimeState *rt);

namespace {

  // During the asynchronous cold import, avoid touching OmLog's GUI-facing
  // pending-message queues from a std::thread. The file sink is mutex-backed;
  // once the GUI thread adopts the runtime, normal console logging resumes.
  static std::atomic<bool> gAsyncPreloadWorker(false);
  static std::atomic<bool> gAsyncPreloadReleasedGil(false);

  void newtonWarning(const QString &message) {
    if (gAsyncPreloadWorker.load(std::memory_order_relaxed))
      OmLog::fileLog(QString("WARNING: ") + message);
    else
      OmLog::warning(message);
  }

  void newtonInfo(const QString &message) {
    if (gAsyncPreloadWorker.load(std::memory_order_relaxed))
      OmLog::fileLog(QString("INFO: ") + message);
    else
      OmLog::info(message);
  }

  void newtonError(const QString &message) {
    if (gAsyncPreloadWorker.load(std::memory_order_relaxed))
      OmLog::fileLog(QString("ERROR: ") + message);
    else
      OmLog::error(message);
  }

  // Fetches the active Python error and CLEARS it; `detail` receives its
  // str() text. Returns true when there was one to fetch.
  //
  // Split out of reportPyError() so the FATAL sibling below can reuse the
  // same fetch: there is exactly one live Python error and it has to be
  // consumed exactly once, whichever severity ends up reporting it.
  bool takePyError(std::string &detail) {
    detail.clear();
    if (!PyErr_Occurred())
      return false;
    PyObject *type = nullptr, *value = nullptr, *tb = nullptr;
    PyErr_Fetch(&type, &value, &tb);
    PyErr_NormalizeException(&type, &value, &tb);
    if (value != nullptr) {
      PyObject *str = PyObject_Str(value);
      if (str != nullptr) {
        const char *cstr = PyUnicode_AsUTF8(str);
        if (cstr != nullptr)
          detail = cstr;
        Py_DECREF(str);
      }
    }
    Py_XDECREF(type);
    Py_XDECREF(value);
    Py_XDECREF(tb);
    return true;
  }

  // Logs the active Python error (if any) via OmLog::warning prefixed
  // with `step`, then clears it. Returns -1 unconditionally so call
  // sites can `return reportPyError(...)`.
  //
  // WARNING is the correct level for the ~60 call sites that use this, and
  // they must STAY warnings: each one is ONE FEATURE declining -- a shape that
  // did not register, a joint target that did not push, a readback that came
  // back empty. The world still has physics; something inside it is less than
  // it should be, the run is degraded and not void, and blanket-promoting the
  // lot would turn every cosmetic decline into a failed exit code.
  //
  // For the failures where the world ends up with NO physics AT ALL, use
  // reportPyErrorFatal() instead.
  int reportPyError(const char *step) {
    std::string detail;
    if (takePyError(detail))
      newtonWarning(QString::fromStdString(std::string("[OmNewtonBackend] ") + step + " raised: " + detail));
    else
      newtonWarning(QString::fromStdString(std::string("[OmNewtonBackend] ") + step + " failed (no Python error)"));
    return -1;
  }

  // The FATAL sibling of reportPyError(), for the one failure class where the
  // world gets NO Newton world at all: finalize() raising.
  //
  // ⚠ WHY THIS IS AN ERROR AND NOT A WARNING. A raise out of world.finalize()
  // means SolverMuJoCo was never constructed, so there is no solver, no
  // contacts and no integration -- every body in the scene stays frozen at its
  // authored pose for the entire run, and Newton is the only backend so there
  // is nothing to degrade to. Logged at WARNING (as it was until 2026-08-16)
  // that total loss was INVISIBLE to the default validation lane: `run-headless`
  // counts lines starting with "ERROR:"/"FATAL:", so a world with no physics
  // whatsoever printed `0 errors, N warnings ... PASS` and exited 0. Only
  // `--fail-on-warning` caught it, and `--fail-on-runaway` could not -- a body
  // frozen at its authored pose is indistinguishable from one legally at rest.
  //
  // Measured 2026-08-16 on the one reachable producer: a loop-closing
  // SolidReference, where newton raises ValueError "Multiple joints lead to
  // body N" out of topological_sort -- MuJoCo is a tree-articulation solver
  // and cannot close a kinematic loop. `run-headless --duration 10` on that
  // world went `0 errors, 4254 warnings ... PASS` (exit 0) to `1 errors,
  // 1 warnings ... FAIL` (exit 1), with the control world -- the same world
  // minus the loop joint -- still PASSing at exit 0.
  //
  // ⚠ A `Cone` boundingObject is documented as a second producer (GeoType.CONE
  // is absent from newton's geom_type_mapping, so registering one would raise
  // KeyError(9)); it is NOT, and the message deliberately does not name it.
  // Measured on the same day: OmCone has no isSuitableForInsertionInBoundingObject,
  // so the parser skips the node long before the builder sees it -- the world
  // logs "Cone geometry node cannot be used in bounding object", falls back to
  // a sphere collider and finalizes normally. Naming an unreachable cause in
  // an error message is how the "Body N has multiple parents" myth got started.
  //
  // ⚠ THE VERDICT LEADS THE LINE, AND THE EXCEPTION TRAILS IT. Measured while
  // writing this: the Python detail is a WRAPPED TRACEBACK, ten-odd lines of
  // it, so a message that appended the consequence after the detail put "THIS
  // WORLD HAS NO PHYSICS" ten lines below the header -- past every log tail,
  // every `grep ^ERROR`, and the one line `run-headless` echoes. Whatever is
  // said first has to be self-contained.
  int reportPyErrorFatal(const char *step) {
    std::string detail;
    const bool had = takePyError(detail);
    newtonError(QString::fromStdString(
      "[OmNewtonBackend] " + std::string(step) +
      " FAILED -- THIS WORLD HAS NO PHYSICS. No Newton world was built, so nothing in it will fall, "
      "collide, actuate or report a contact: every body stays frozen at its authored pose for the whole "
      "run, and Newton is the only physics backend so there is no fallback to degrade to. Do not read a "
      "pose, a rest height or a contact from this run -- it measured nothing. The known cause is a "
      "loop-closing SolidReference, for which newton says either \"Multiple joints lead to body N\" or "
      "\"Body N has multiple parents in this articulation\": MuJoCo is a tree-articulation solver and "
      "cannot close a kinematic loop, so model the mechanism as a tree and drive the dependent joint "
      "from a controller. For anything else, the exception below names what the solver refused. " +
      (had ? ("The Python exception was: " + detail) : std::string("No Python error was set."))));
    return -1;
  }

  // Why Newton is unavailable, when it is. The distinction that matters is
  // MISSING vs BROKEN:
  //
  //   MISSING  `import warp` / `import newton` failed. This is a genuine
  //            ODE-only clone with no runtime installed. Falling back is the
  //            correct behaviour and always has been.
  //   BROKEN   the runtime IS installed and would not come up -- the embedded
  //            interpreter failed, the FFI smoke failed, our own helper module
  //            failed. That is a MALFUNCTION, and quietly running ODE instead
  //            turns it into wrong physics rather than a missing feature.
  //
  // Retiring the silent fall-back (2026-08-05, owner's call) means BROKEN
  // refuses by default. MISSING still falls back, because "the runtime is not
  // installed" is not a malfunction and an ODE-only clone must keep working.
  enum class NewtonUnavailable { None, Missing, Broken };
  static NewtonUnavailable gNewtonUnavailable = NewtonUnavailable::None;
  static std::string gNewtonUnavailableDetail;
  // Process-lived import cache. preloadRuntime() can populate this before an
  // OmNewtonBackend instance exists; each instance takes its own references.
  static PyObject *gNewtonHelperModule = nullptr;
  static PyObject *gNewtonWorldClass = nullptr;

  void attachCachedRuntime(OmNewtonRuntimeState *runtime) {
    if (runtime == nullptr || gNewtonHelperModule == nullptr || gNewtonWorldClass == nullptr)
      return;
    Py_INCREF(gNewtonHelperModule);
    Py_INCREF(gNewtonWorldClass);
    runtime->helperModule = gNewtonHelperModule;
    runtime->worldClass = gNewtonWorldClass;
  }

  // One-shot bring-up: initialises an embedded CPython if none is
  // running yet, imports warp + newton, runs the FFI smoke check,
  // imports the helper module, caches the World class.
  bool tryInitNewtonRuntime(OmNewtonRuntimeState *runtime) {
    static std::mutex initMutex;
    const std::lock_guard<std::mutex> lock(initMutex);
    static bool tried = false;
    static bool ok = false;
    if (tried) {
      if (ok && runtime != nullptr && runtime->helperModule == nullptr)
        attachCachedRuntime(runtime);
      return ok;
    }
    tried = true;

    // --- fault injection: OMNISIM_NEWTON_SIMULATE_BROKEN ---------------------
    // Makes the "installed but will not come up" state reproducible ON DEMAND.
    // It exists because the real defect is INTERMITTENT (measured at roughly
    // half of cold launches) and the refusal it triggers is a safety property:
    // an assertion nobody has watched fail is not evidence, and a batch of ten
    // healthy launches verifies nothing about what happens on the eleventh.
    //
    // Shadowing the runtime from outside does not work -- the embedded
    // interpreter ignores PYTHONPATH, verified 2026-08-05 by putting a
    // ModelBuilder-less `newton` stub on it and still getting "FFI smoke OK" --
    // so the hook has to live in-process.
    //
    // Test-only, opt-in, and it fails CLOSED: it reports the runtime broken,
    // which is the conservative direction. Nothing reads it unless it is set.
    if (!qEnvironmentVariableIsEmpty("OMNISIM_NEWTON_SIMULATE_BROKEN")) {
      gNewtonUnavailable = NewtonUnavailable::Broken;
      gNewtonUnavailableDetail = "fault injection (OMNISIM_NEWTON_SIMULATE_BROKEN)";
      newtonWarning("[OmNewtonBackend] OMNISIM_NEWTON_SIMULATE_BROKEN is set; reporting the runtime as broken.");
      ok = false;
      return false;
    }

    if (!Py_IsInitialized()) {
      // ⚠ THIS IS THE COLD-LAUNCH FAILURE, AND IT COST A DAY TO FIND.
      //
      // The old code called Py_InitializeEx(0), which takes CPython's legacy
      // init path. That path CONFIGURES C STDIO -- it opens the process's
      // console to build sys.stdin/stdout/stderr. `omnisim-bin.exe` is a
      // GUI-SUBSYSTEM binary and has no console, so whenever no usable console
      // handle was inherited from the parent, init_sys_streams failed with
      //
      //     Fatal Python error: init_sys_streams: can't initialize sys
      //     standard streams
      //     ValueError: Cannot open console input buffer for writing
      //
      // and CPython's Py_FatalError exited the process on the spot. Exit code
      // 1, roughly one second in, the world already parsed (parsing is C++ and
      // runs before this), no Newton, no controller, and the explanation
      // written to a stderr that a GUI binary discards -- so the symptom was
      // "the run produced nothing" with an empty log. Measured at roughly one
      // launch in three, worse back-to-back, and it ate whole benchmark cells:
      // every clause of a 42-minute run came back vacuous with rc=1.
      //
      // It also explains why main()'s own exit-code logging never fired: a
      // fatal Python error does not unwind, it exits.
      //
      // PyConfig with configure_c_stdio = 0 is the documented way to embed in
      // an application that has no console. install_signal_handlers = 0 keeps
      // the previous behaviour of not stealing SIGINT/SIGTERM from the Qt
      // event loop, which is what Py_InitializeEx(0) was there for.
      // ⚠ configure_c_stdio = 0 IS NOT SUFFICIENT ON ITS OWN, AND NEITHER IS
      // CHECKING THE DESCRIPTORS. init_sys_streams builds sys.stdout/sys.stderr
      // from fds 1 and 2, and that step fails intermittently even when both are
      // valid, writable files -- measured twice on 2026-08-05 under an OmniBench
      // runner that hands the engine `stdout=<open log file>`, killing the T6
      // Newton row both times with
      //
      //     embedded Python init failed (can't initialize sys standard streams)
      //
      // An earlier attempt here reopened only descriptors that failed a
      // validity probe. That probe never fired on the real failure -- the
      // descriptors were fine -- so it fixed nothing, which is why the fix is
      // now a RETRY rather than a precondition.
      //
      // Attempt 1 runs untouched, so a healthy launch is byte-unchanged and
      // keeps whatever stdout it was given. Only if that fails do we point
      // stdio at the null device and try once more: a physics backend does not
      // need stdout, and warp/newton's import banner is already discarded by
      // the stdio repair below. Losing the banner is worth strictly more than
      // losing Newton for the whole run.
      PyStatus pyStatus;
      bool initialised = false;
      for (int attempt = 0; attempt < 2 && !initialised; ++attempt) {
        if (attempt == 1) {
#ifdef _WIN32
          const char *const nullDevice = "NUL";
#else
          const char *const nullDevice = "/dev/null";
#endif
          FILE *const streams[3] = {stdin, stdout, stderr};
          const char *const modes[3] = {"r", "w", "w"};
          for (int fd = 0; fd < 3; ++fd)
            (void)freopen(nullDevice, modes[fd], streams[fd]);
          newtonWarning(QString("[OmNewtonBackend] embedded Python init failed (%1); retrying with stdio pointed at "
                                 "%2 -- the interpreter could not build sys.stdout/sys.stderr from the descriptors "
                                 "it was given.")
                           .arg(pyStatus.err_msg ? pyStatus.err_msg : "no message")
                           .arg(nullDevice));
        }
        PyConfig pyConfig;
        PyConfig_InitPythonConfig(&pyConfig);
        pyConfig.install_signal_handlers = 0;
        pyConfig.configure_c_stdio = 0;
#ifdef _WIN32
        // This is the flag that targets the component that actually fails.
        // configure_c_stdio = 0 stops CPython configuring the C runtime's
        // stdio, but init_sys_streams still BUILDS sys.stdin/stdout/stderr,
        // and on Windows it builds them with io._WindowsConsoleIO -- which is
        // what threw the original "Cannot open console input buffer for
        // writing" in a GUI-subsystem binary with no console, and what still
        // throws the surviving "can't initialize sys standard streams".
        //
        // legacy_windows_stdio = 1 makes CPython use io.FileIO for those three
        // streams instead, so the console is never consulted at all.
        // Documented, Windows-only, inert elsewhere.
        pyConfig.legacy_windows_stdio = 1;
#endif
        pyStatus = Py_InitializeFromConfig(&pyConfig);
        PyConfig_Clear(&pyConfig);
        initialised = !PyStatus_Exception(pyStatus) && Py_IsInitialized();
        if (initialised && attempt == 1)
          newtonWarning("[OmNewtonBackend] the retry succeeded; Newton is available and its stdout is discarded.");
      }
      if (!initialised) {
        gNewtonUnavailable = NewtonUnavailable::Broken;
        gNewtonUnavailableDetail = "the embedded interpreter would not initialise (retry with null stdio also failed)";
        newtonWarning(QString("[OmNewtonBackend] embedded Python init failed after a retry (%1)")
                         .arg(pyStatus.err_msg ? pyStatus.err_msg : "no message"));
        return false;
      }
    }

    // --- Robust stdio for the embedded interpreter (Newton-default fix) ---
    // warp/newton print a startup banner to sys.stdout during import +
    // ModelBuilder(). Under a headless launch whose stdout is routed to
    // DEVNULL, the embedded interpreter's sys.stdout/sys.stderr come up as
    // None (or a closed fd), so the banner write raises e.g. "'NoneType'
    // object has no attribute 'write'" / "[Errno 9] Bad file descriptor",
    // the FFI smoke below fails, and the world gets NO physics -- which is what
    // made Newton-configured worlds (e.g. the quadruped walk deploys)
    // intermittently collapse on a default headless run. (When this was first
    // diagnosed the failure silently degraded to ODE instead, which was worse:
    // 5 of 10 launches ran a backend nobody chose, under a log saying Newton
    // had been requested, and nothing downstream could tell them apart.) Ensure
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
      gNewtonUnavailable = NewtonUnavailable::Missing;
      gNewtonUnavailableDetail = "`import warp` failed";
      newtonWarning("[OmNewtonBackend] `import warp` failed; install with"
                     " `pip install warp-lang`. THERE IS NO FALLBACK: ODE was removed, so"
                     " this world will run with NO physics -- bodies will not fall, collide"
                     " or move. Fix the runtime; `python -m omnisim doctor` reports it.");
      return false;
    }
    Py_DECREF(warp);

    PyObject *newton = PyImport_ImportModule("newton");
    if (newton == nullptr) {
      PyErr_Clear();
      gNewtonUnavailable = NewtonUnavailable::Missing;
      gNewtonUnavailableDetail = "`import newton` failed";
      newtonWarning("[OmNewtonBackend] `import newton` failed; install with"
                     " `pip install \"newton[examples]\"`. THERE IS NO FALLBACK: ODE was"
                     " removed, so this world will run with NO physics. Fix the runtime;"
                     " `python -m omnisim doctor` reports it.");
      return false;
    }

    // FFI smoke: instantiate newton.ModelBuilder() to confirm the call
    // surface beyond imports works. Throwaway -- the helper module
    // will create its own builder per World.
    PyObject *modelBuilderClass = PyObject_GetAttrString(newton, "ModelBuilder");
    if (modelBuilderClass == nullptr) {
      PyErr_Clear();
      Py_DECREF(newton);
      gNewtonUnavailable = NewtonUnavailable::Broken;
      gNewtonUnavailableDetail = "newton is installed but has no ModelBuilder";
      newtonWarning("[OmNewtonBackend] newton.ModelBuilder attribute missing;"
                     " API drift? THERE IS NO FALLBACK: ODE was removed, so this world will"
                     " run with NO physics. The installed newton is not a version this"
                     " engine can drive.");
      return false;
    }
    PyObject *builder = nullptr;
    for (int attempt = 0; attempt < 2 && builder == nullptr; ++attempt) {
      PyObject *emptyArgs = PyTuple_New(0);
      builder = PyObject_CallObject(modelBuilderClass, emptyArgs);
      Py_DECREF(emptyArgs);
      if (builder == nullptr && attempt == 0) {
        // Warp's Windows cache initialization has a check-then-mkdir race
        // across concurrently starting OmniSim processes (WinError 183: the
        // other process created the directory first). Once it exists, the
        // same smoke call succeeds; retry once instead of permanently marking
        // Newton broken for this process.
        PyErr_Clear();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }
    Py_DECREF(modelBuilderClass);
    if (builder == nullptr) {
      gNewtonUnavailable = NewtonUnavailable::Broken;
      gNewtonUnavailableDetail = "the FFI smoke check failed";
      reportPyError("FFI smoke (newton.ModelBuilder())");
      Py_DECREF(newton);
      return false;
    }
    Py_DECREF(builder);
    Py_DECREF(newton);

    // The installed/bundled module is the normal path. Developer binaries also
    // support a source-checkout fallback so editing the Python runtime does not
    // require copying it beside the executable or rebuilding C++ first.
    PyObject *helperModule = PyImport_ImportModule("omnisim_newton_runtime");
    if (helperModule == nullptr) {
      PyErr_Clear();
      PyRun_SimpleString(
        "import os as _os, pathlib as _pl, sys as _sys\n"
        "_roots = [_os.environ.get('OMNISIM_HOME'), _os.environ.get('WEBOTS_HOME'), _os.getcwd()]\n"
        "try:\n"
        "    _roots.append(str(_pl.Path(_sys.executable).resolve().parents[3]))\n"
        "except Exception:\n"
        "    pass\n"
        "for _root in _roots:\n"
        "    if not _root: continue\n"
        "    _candidate = _pl.Path(_root) / 'src' / 'omnisim' / 'physics'\n"
        "    if (_candidate / 'omnisim_newton_runtime.py').is_file():\n"
        "        _sys.path.insert(0, str(_candidate)); break\n");
      PyErr_Clear();
      helperModule = PyImport_ImportModule("omnisim_newton_runtime");
    }
    if (helperModule == nullptr) {
      gNewtonUnavailable = NewtonUnavailable::Broken;
      gNewtonUnavailableDetail = "the bundled omnisim_newton_runtime module could not be imported";
      reportPyError("import omnisim_newton_runtime");
      return false;
    }

    PyObject *worldClass = PyObject_GetAttrString(helperModule, "World");
    if (worldClass == nullptr) {
      gNewtonUnavailable = NewtonUnavailable::Broken;
      gNewtonUnavailableDetail = "our own helper module has no World class";
      newtonWarning("[OmNewtonBackend] helper module missing `World` class");
      Py_DECREF(helperModule);
      return false;
    }

    // Keep one process-lifetime reference so preloadRuntime() can finish before
    // an actual backend instance exists. CPython itself is deliberately never
    // finalized in this process, so these follow the interpreter's lifetime.
    gNewtonHelperModule = helperModule;
    gNewtonWorldClass = worldClass;
    attachCachedRuntime(runtime);

    // NOTE for matchers: keep "[OmNewtonBackend]" + "imports OK" on this line
    // (env_fingerprint.py's _RE_IMPORTS_OK greps for it), and do NOT let it
    // contain "world finalised (solver=" -- that is the
    // the finalise/fallback verdict markers. This line is NOT a verdict: the
    // backend is chosen at world finalise (default physicsBackend "auto"
    // resolves to Newton when this runtime is present); the authoritative
    // verdict is the <log>.newton.json sidecar written at finalise.
    newtonInfo("[OmNewtonBackend] warp + newton imports OK; FFI smoke OK"
                " (newton.ModelBuilder()); helper module loaded -- not a backend"
                " verdict: the backend is chosen at world finalise (default"
                " physicsBackend \"auto\" resolves to Newton when the runtime is"
                " present); authoritative verdict: the <log>.newton.json sidecar");
    ok = true;
    return true;
  }

  void releaseWorld(OmNewtonRuntimeState *runtime) {
    if (runtime->world != nullptr) {
      Py_DECREF(runtime->world);
      runtime->world = nullptr;
    }
  }

}  // namespace
#endif  // OMNISIM_WITH_NEWTON

OmNewtonBackend::OmNewtonBackend() : mAvailable(false), mRuntime(nullptr) {
#ifdef OMNISIM_WITH_NEWTON
  mRuntime = new OmNewtonRuntimeState();
  mAvailable = tryInitNewtonRuntime(mRuntime);
  if (!mAvailable) {
    delete mRuntime;
    mRuntime = nullptr;
    // OMNISIM_REQUIRE_NEWTON (opt-in): fail LOUDLY at construction instead of
    // loading a world that will stand still when the Newton runtime won't come
    // up at all. Still useful post-ODE: the default is one logged error and a
    // motionless scene, which a batch job can miss; this makes it non-zero exit.
    // This guards the WHOLE-RUNTIME-missing case (warp/newton import or FFI
    // smoke failure -- e.g. the headless stdout/banner FFI-smoke failure fixed
    // alongside this); it is intentionally opt-in so a genuine ODE-only clone
    // without the bundled runtime is unaffected. The complementary case -- the
    // runtime IS up but an individual articulation / joint / solver would
    // silently downgrade to ODE -- is enforced by DEFAULT on a Newton-capable
    // build via OmPhysicsBackendRegistry::newtonEnforced() (OmSolid /
    // OmBasicJoint flush, finalizeWorld). The opt-outs that used to relax both
    // -- OMNISIM_ALLOW_ODE_FALLBACK, OMNISIM_FORCE_ODE, OMNISIM_LEGACY -- are
    // RETIRED: each named a legitimate route onto ODE, and ODE is gone, so they
    // now warn and are ignored (OmPhysicsBackend.cpp).
    // ⚠ THE SILENT FALL-BACK IS RETIRED (2026-08-05, owner's call).
    //
    // A runtime that is INSTALLED and will not come up is a malfunction, and
    // running ODE instead of saying so does not degrade gracefully -- it
    // produces a world simulated by a backend nobody chose, with different
    // contact behaviour, different friction semantics and different contact
    // visibility, under a log that says Newton was requested. That corrupts
    // results rather than losing them, which is strictly worse.
    //
    // Measured 2026-08-05: 5 of 10 cold launches on this machine failed
    // interpreter bring-up and continued on ODE. Nothing downstream could tell
    // those runs from Newton runs except by reading the .newton.json sidecar,
    // and no benchmark cell was doing that.
    //
    // MISSING historically "still fell back", because "the runtime is not
    // installed" is not a malfunction and an ODE-only clone had to keep working.
    // ⚠ SINCE ODE'S DELETION THAT FALL-BACK LEADS NOWHERE: OmOdeBackend is an
    // inert dispatcher stub, so a runtime-absent clone runs with no physics at
    // all rather than with legacy physics. OMNISIM_REQUIRE_NEWTON is still the
    // only thing that makes that state visible; promoting it to the default is a
    // behaviour change the owner has to call.
    //
    // There is no escape hatch for BROKEN any more. It used to be
    // OMNISIM_ALLOW_ODE_FALLBACK=1; with ODE deleted that variable can only buy
    // a physics-free run, so it warns and is ignored.
    // ⚠ THE REFUSAL CANNOT FIRE HERE. This constructor runs BEFORE the world
    // is parsed, so at this point nobody has said which backend they want --
    // and refusing here rejected worlds that explicitly asked for ODE, which
    // is the one case that must always keep working. Measured immediately
    // after writing it: 4 of 4 `defaultPhysicsBackend "ode"` worlds refused.
    //
    // The reason is recorded instead, and the refusal happens in
    // OmSolid's Newton flush, once the world's own choice is known. See
    // OmNewtonBackend::refuseIfBrokenAndNewtonWanted.
    if (gNewtonUnavailable == NewtonUnavailable::Missing && !qEnvironmentVariableIsEmpty("OMNISIM_REQUIRE_NEWTON"))
      OmLog::fatal(
        "[OmNewtonBackend] OMNISIM_REQUIRE_NEWTON is set but the Newton runtime is"
        " not installed (warp/newton import failed). Newton is now the ONLY physics"
        " backend -- ODE has been removed -- so there is nothing to fall back to and"
        " this world cannot be simulated. Install the runtime (pip install warp-lang"
        " \"newton[examples]\", or `make bundle-newton-runtime`).");
  }
#endif
}

bool OmNewtonBackend::preloadRuntime() {
#ifdef OMNISIM_WITH_NEWTON
  OmNewtonRuntimeState probe;
  const bool ok = tryInitNewtonRuntime(&probe);
  Py_XDECREF(probe.worldClass);
  Py_XDECREF(probe.helperModule);
  return ok;
#else
  return false;
#endif
}

bool OmNewtonBackend::preloadRuntimeAsyncWorker() {
#ifdef OMNISIM_WITH_NEWTON
  gAsyncPreloadWorker.store(true, std::memory_order_release);
  const bool ok = preloadRuntime();
  gAsyncPreloadWorker.store(false, std::memory_order_release);
  if (Py_IsInitialized()) {
    // Py_InitializeFromConfig gives this worker the GIL. Release it before the
    // future becomes ready; the main simulator thread adopts it exactly once.
    (void)PyEval_SaveThread();
    gAsyncPreloadReleasedGil.store(true, std::memory_order_release);
  }
  return ok;
#else
  return false;
#endif
}

void OmNewtonBackend::adoptAsyncPreloadedRuntime() {
#ifdef OMNISIM_WITH_NEWTON
  if (gAsyncPreloadReleasedGil.exchange(false, std::memory_order_acq_rel))
    // OmniSim intentionally keeps the embedded interpreter and its GIL alive
    // for the process lifetime, matching the pre-preload ownership model.
    (void)PyGILState_Ensure();
#endif
}

bool OmNewtonBackend::refuseIfBrokenAndNewtonWanted(bool newtonWanted) {
  // Called once the world's backend choice IS known. Returns true when it
  // refused (and has already logged a FATAL, which exits).
  //
  // The silent fall-back is retired (2026-08-05): a runtime that is INSTALLED
  // and will not come up is a malfunction, and running ODE instead produces a
  // world simulated by a backend nobody chose -- different contact behaviour,
  // different friction semantics, different contact visibility -- under a log
  // saying Newton was requested. That corrupts results rather than losing
  // them. Measured: 5 of 10 cold launches degraded this way and nothing
  // downstream could tell those runs from Newton runs.
  //
  // MISSING is untouched: it is still governed by OMNISIM_REQUIRE_NEWTON. (The
  // reason it was left alone -- "an ODE-only clone must keep working" -- no
  // longer holds now that ODE is deleted, but changing it is the owner's call.)
#ifdef OMNISIM_WITH_NEWTON
  if (!newtonWanted || gNewtonUnavailable != NewtonUnavailable::Broken)
    return false;
  // OMNISIM_ALLOW_ODE_FALLBACK used to turn this refusal off by accepting an ODE
  // run. It cannot do that any more -- ODE has been removed -- so it is warned
  // about and ignored rather than silently buying a physics-free world.
  if (!qEnvironmentVariableIsEmpty("OMNISIM_ALLOW_ODE_FALLBACK"))
    OmLog::warning("[OmNewtonBackend] OMNISIM_ALLOW_ODE_FALLBACK is set but RETIRED and IGNORED: it used to accept an "
                   "ODE run when Newton would not come up, and ODE has been removed. The refusal below stands.");
  OmLog::fatal(QString("[OmNewtonBackend] this world asked for the Newton backend, and the Newton runtime is INSTALLED "
                       "but did not come up: %1. Newton is the only physics backend -- ODE has been removed -- so "
                       "there is no other backend to run this world on, and running it on nothing would be a wrong "
                       "result rather than a degraded one. Fix the runtime.")
                 .arg(QString::fromStdString(gNewtonUnavailableDetail)));
  return true;
#else
  (void)newtonWanted;
  return false;
#endif
}

OmNewtonBackend::~OmNewtonBackend() {
#ifdef OMNISIM_WITH_NEWTON
  if (mRuntime != nullptr) {
    releaseWorld(mRuntime);
    if (mRuntime->worldClass != nullptr) {
      Py_DECREF(mRuntime->worldClass);
      mRuntime->worldClass = nullptr;
    }
    if (mRuntime->helperModule != nullptr) {
      Py_DECREF(mRuntime->helperModule);
      mRuntime->helperModule = nullptr;
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

int OmNewtonBackend::beginWorld() {
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
  // Fresh world -> re-arm the one-shot diagnostics (a reload must be able to
  // report the same problem again). Diagnostics only; no physics state here.
  mRuntime->constraintOverflowLogged = false;
  mRuntime->autoConstraintCapWarned = false;
  mRuntime->finalizeFailureReported = false;
  mRuntime->snapValid = false;  // Tier 1a: never serve a prior world's snapshot
  clearJointTargetCache();      // item 5: a new world re-pushes every target
  return 0;
}

int OmNewtonBackend::ensureWorldOpen() {
  if (!mAvailable || mRuntime == nullptr)
    return -1;
  if (mRuntime->openForBuild || mRuntime->running)
    return 0;
  if (beginWorld() != 0)
    return -1;
  // The WORLD'S UP AXIS FIRST, before literally anything is added.
  // WorldInfo.coordinateSystem decides two things newton bakes in at add time:
  // the implicit ground plane's NORMAL (add_ground_plane() reads
  // builder.up_vector when it composes the plane equation) and the direction
  // setWorldGravity()'s projection resolves to. Both are unfixable afterwards,
  // which is why this sits above even the contact params.
  //
  // Until 2026-08-08 the runtime hardcoded up_axis=Axis.Z and never read the
  // field, so the 210 NUE (Y-up) worlds in this tree ran at gravity ZERO -- the
  // projection of (0,-g,0) onto (0,0,1) -- behind an infinite plane whose normal
  // pointed EAST, i.e. a vertical wall through the scene. Nothing warned; the
  // readiness sweep scored them PASS, because a log verdict cannot see gravity.
  applyCoordinateSystemToWorld();
  // Contact params BEFORE the ground plane and before any registration adds a
  // shape. newton's ModelBuilder copies cfg.mu/ke/kd into the shape AT ADD
  // TIME, so a WorldInfo.newtonGroundMu applied after the registration loop
  // (where this used to happen, at the tail of flushPendingNewtonRegistrations)
  // reached NOTHING -- measured on the 55-degree ramp comparator: declared
  // mu 2.0, box slid anyway, because every shape including the implicit ground
  // plane was created at the default. OmSolid caches the world's values into
  // the m*Pref members before the loop; this is the moment they land.
  applyContactSolverParamsToWorld();
  if (addGroundPlane() != 0)
    return -1;
  // ⚠ REQUESTS the implicit ground plane; it is not added here any more, and
  // this line must not say it was. Since 2026-08-12 the runtime only adds the
  // plane at finalize, and only as a declared substitute for an authored
  // `Plane` collider newton's MuJoCo converter cannot build -- because an
  // unconditional plane gave every world an UNDECLARED infinite collider at
  // up-axis 0 that caught bodies which should have fallen (measured: a 0.2 m
  // box in a world with no collidable floor at all settled at z=0.099892).
  // The runtime logs the ACTUAL decision, with its reason, to the newton log
  // (`[OmNewtonBackend] implicit ground plane: ...`).
  OmLog::info("[OmNewtonBackend] world opened (implicit ground plane requested; "
              "added at finalize only if the world declared a Plane collider)");
  return 0;
}

int OmNewtonBackend::applyContactSolverParamsToWorld() {
  // Push the cached WorldInfo contact/solver prefs into the CURRENT Python
  // world. Split out of setContactSolverParams() so ensureWorldOpen() can
  // apply them to a world that has just been constructed -- the caching call
  // from OmSolid runs before the world exists.
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr)
    return -1;
  // mu's unset sentinel is NEGATIVE (ke/kd keep 0): mu=0 is a legal physical
  // value -- a frictionless world -- and `<= 0` made it unreachable from the
  // .wbt. See WorldInfo.wrl newtonGroundMu.
  if (mGroundMuPref < 0.0 && mContactKePref <= 0.0 && mContactKdPref <= 0.0 && mIterationsPref <= 0 &&
      mLsIterationsPref <= 0)
    return 0;  // nothing declared; leave env + engine defaults in charge
  OmLog::info(QString("[OmNewtonBackend] contact/solver params from WorldInfo: mu=%1 ke=%2 kd=%3 iters=%4 ls_iters=%5")
                .arg(mGroundMuPref)
                .arg(mContactKePref)
                .arg(mContactKdPref)
                .arg(mIterationsPref)
                .arg(mLsIterationsPref));
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *r = PyObject_CallMethod(mRuntime->world, "set_contact_solver_params", "(dddii)", mGroundMuPref,
                                    mContactKePref, mContactKdPref, mIterationsPref, mLsIterationsPref);
  const int rc = (r == nullptr) ? reportPyError("set_contact_solver_params") : 0;
  Py_XDECREF(r);
  PyGILState_Release(gstate);
  return rc;
}

int OmNewtonBackend::setCoordinateSystem(const std::string &cs) {
  // Cache WorldInfo.coordinateSystem for the world ensureWorldOpen() is about to
  // construct. OmSolid calls this at the HEAD of
  // flushPendingNewtonRegistrations, where no Newton world exists yet -- the
  // caching IS the point, and rc -1 is the expected answer on that call. The
  // world is opened lazily from inside that same function's registration loop,
  // so there is no earlier seam and no way to pass this to the World()
  // constructor.
  mCoordSystemPref = cs;
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  // A world that opened before this arrived already has its ground plane, so
  // this apply is only meaningful for a caller that plumbs before the first
  // registration. Kept for symmetry with the other set*() prefs.
  return applyCoordinateSystemToWorld();
}

int OmNewtonBackend::applyCoordinateSystemToWorld() {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr)
    return -1;
  if (mCoordSystemPref.empty())
    return 0;  // never declared -> the runtime keeps its z-up default (== ENU)
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *r = PyObject_CallMethod(mRuntime->world, "set_up_axis", "(s)", mCoordSystemPref.c_str());
  const int rc = (r == nullptr) ? reportPyError("set_up_axis") : 0;
  Py_XDECREF(r);
  // Read back the axis the runtime actually resolved rather than the one we
  // asked for: OMNISIM_NEWTON_COORD_SYSTEM=0 (the bisection hatch) makes
  // set_up_axis a no-op, and an unrecognised field value is declined there too.
  // Logging the request would then name an axis the physics does not have.
  std::string applied;
  if (rc == 0) {
    PyObject *ua = PyObject_GetAttrString(mRuntime->world, "_up_axis_name");
    if (ua != nullptr) {
      if (PyUnicode_Check(ua)) {
        const char *c = PyUnicode_AsUTF8(ua);
        applied = c ? c : "";
      }
      Py_DECREF(ua);
    } else
      PyErr_Clear();
  }
  PyGILState_Release(gstate);
  // What the field ASKS for, by the same rule OmWorldInfo::updateGravityBasis()
  // uses: the up axis is wherever the "U" sits. Z for ENU, Y for NUE and EUN.
  std::string wanted("Z");
  {
    const std::string::size_type u = mCoordSystemPref.find('U');
    if (mCoordSystemPref.size() == 3 && u != std::string::npos)
      wanted = std::string(1, "XYZ"[u]);
  }
  // SILENT for a z-up world. ENU is the schema default and 511 of the 719 worlds
  // in this tree resolve to it, so logging there would add a line to every
  // existing engine log for a call that changes nothing -- the runtime already
  // records up_axis for EVERY world in .build_tmp/newton_solver.log, which is
  // where per-run attribution belongs. A NON-z-up world is the newsworthy case
  // and gets one line, including when the bisection hatch declined it (that run
  // is deliberately reproducing the pre-2026-08-08 defect and must say so).
  if (rc == 0 && !(wanted == "Z" && (applied == "Z" || applied.empty()))) {
    const std::string got = applied.empty() ? std::string("(unread)") : applied;
    if (got == wanted)
      OmLog::info(QString::fromStdString("[OmNewtonBackend] WorldInfo.coordinateSystem \"" + mCoordSystemPref +
                                         "\" -> up_axis " + got +
                                         ": gravity direction and the implicit ground-plane normal follow it"));
    else
      OmLog::warning(QString::fromStdString(
        "[OmNewtonBackend] WorldInfo.coordinateSystem \"" + mCoordSystemPref + "\" asks for up_axis " + wanted +
        " but the Newton builder is running up_axis " + got +
        " -- OMNISIM_NEWTON_COORD_SYSTEM is turned off, which reproduces the pre-fix defect ON PURPOSE: gravity "
        "projects to 0 along a perpendicular up vector and the implicit ground plane stands up as a vertical wall. "
        "Unset the variable for correct physics."));
  }
  return rc;
}

void OmNewtonBackend::teardownWorld() {
  if (!mAvailable || mRuntime == nullptr)
    return;
  if (mRuntime->world == nullptr && !mRuntime->openForBuild && !mRuntime->running)
    return;  // nothing to tear down
  releaseWorld(mRuntime);
  mRuntime->openForBuild = false;
  mRuntime->running = false;
  mRuntime->snapValid = false;
  clearJointTargetCache();
  OmLog::info("[OmNewtonBackend] world torn down (next load re-opens a fresh Newton world)");
}

bool OmNewtonBackend::isWorldOpenForBuild() const {
  return mAvailable && mRuntime != nullptr && mRuntime->openForBuild;
}

bool OmNewtonBackend::isWorldRunning() const {
  return mAvailable && mRuntime != nullptr && mRuntime->running;
}

int OmNewtonBackend::addGroundPlane() {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  PyObject *r = PyObject_CallMethod(mRuntime->world, "add_ground_plane", nullptr);
  if (r == nullptr)
    return reportPyError("add_ground_plane");
  Py_DECREF(r);
  return 0;
}

int OmNewtonBackend::addBody(double mass, double x, double y, double z,
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

int OmNewtonBackend::addStaticBody(double x, double y, double z,
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

int OmNewtonBackend::addKinematicBody(double x, double y, double z,
                                      double qx, double qy, double qz, double qw) {
  // Kernel blocker #4 (_scratch/design_kinematic_inertia.md Part 1): BUILD
  // phase. Registers a KINEMATIC body -- a fixed-root (hence MuJoCo MOCAP)
  // body whose collision shapes are tested against dynamic bodies while its
  // pose stays engine-owned and is pushed per change via setKinematicPose.
  // The caller attaches shapes via addShape* exactly like addStaticBody.
  // Returns the body's index (>= 0) or -1.
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  PyObject *r = PyObject_CallMethod(mRuntime->world, "add_kinematic_body",
                                     "(ddddddd)",
                                     x, y, z, qx, qy, qz, qw);
  if (r == nullptr)
    return reportPyError("add_kinematic_body");
  long idx = PyLong_AsLong(r);
  Py_DECREF(r);
  return static_cast<int>(idx);
}

int OmNewtonBackend::setKinematicPose(int bodyIdx, double x, double y, double z,
                                      double qx, double qy, double qz, double qw) {
  // RUN phase: push the engine-computed world pose of a kinematic (mocap)
  // body into mj_data.mocap_pos/mocap_quat -- effective on the NEXT step.
  // Quaternion is xyzw on the wire (the house body_q convention); the
  // runtime reorders to MuJoCo's wxyz. Works for any fixed-root body
  // (kinematic OR static registration -- both are mocap-exported); -1 when
  // the body is unknown or not a fixed root. Same GIL discipline as the
  // weld verbs (this can be called from field-update signal paths).
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->running)
    return -1;
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *r = PyObject_CallMethod(mRuntime->world, "set_kinematic_pose", "(iddddddd)",
                                    bodyIdx, x, y, z, qx, qy, qz, qw);
  if (r == nullptr) {
    const int err = reportPyError("set_kinematic_pose");
    PyGILState_Release(gstate);
    return err;
  }
  const long rc = PyLong_AsLong(r);
  Py_DECREF(r);
  PyGILState_Release(gstate);
  return rc == 0 ? 0 : -1;
}

int OmNewtonBackend::addShapeSphere(int bodyIdx, double radius,
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

int OmNewtonBackend::addShapeBox(int bodyIdx, double hx, double hy, double hz,
                                 double cx, double cy, double cz, double ke,
                                 double qx, double qy, double qz, double qw) {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  // 1 int + 11 doubles: hx,hy,hz, cx,cy,cz, ke, qx,qy,qz,qw. The quaternion is
  // APPENDED after ke so the existing positional order is untouched (the python
  // side defaults it to identity, so an older bundle still accepts the old call).
  PyObject *r = PyObject_CallMethod(mRuntime->world, "add_shape_box", "(iddddddddddd)",
                                     bodyIdx, hx, hy, hz, cx, cy, cz, ke, qx, qy, qz, qw);
  if (r == nullptr)
    return reportPyError("add_shape_box");
  Py_DECREF(r);
  return 0;
}

int OmNewtonBackend::setBodyVel(int bodyIdx, double x, double y, double z, int angular) {
  // W3.2: write a Newton body's linear (angular=0) or angular (angular=1) velocity directly into body_qd.
  // Valid DURING simulation (no openForBuild) -- the world + state must exist.
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr)
    return -1;
  mRuntime->snapValid = false;  // Tier 1a: body_qd rewritten below
  PyObject *r = PyObject_CallMethod(mRuntime->world, "set_body_vel", "(idddi)",
                                    bodyIdx, x, y, z, angular);
  if (r == nullptr)
    return reportPyError("set_body_vel");
  Py_DECREF(r);
  return 0;
}

int OmNewtonBackend::addBodyForce(int bodyIdx, double fx, double fy, double fz,
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

void OmNewtonBackend::warnGpuReadbackOnWarp(bool rays) const {
  // internal parity plan, item W1.1. Fires on FIRST USE of an mj_data-backed readback
  // under the GPU solver, once per surface per world. Loud by design: the whole
  // defect class here is a service that answers instead of failing, so the one
  // thing that must never happen is a quiet degradation.
  bool &flag = rays ? mWarnedGpuRays : mWarnedGpuContacts;
  if (flag)
    return;
  flag = true;
  if (rays)
    OmLog::warning(
      "[OmNewtonBackend] Ray sensors (DistanceSensor / Receiver / LightSensor / Radar / Camera recognition "
      "occlusion) are NOT SERVED on the GPU newtonSolver \"mujoco_warp\": newton steps mjw_data on the device "
      "and leaves the mj_data this service raycasts against FROZEN AT THE BUILD POSE, so it would report the "
      "scene as authored at t=0. The service is DECLINING instead -- affected sensors report no hit and keep "
      "their previous verdict. Use the CPU newtonSolver \"mujoco\" (the default) if this world needs them.");
  else
    OmLog::warning(
      "[OmNewtonBackend] Native contact readback is DEGRADED on the GPU newtonSolver \"mujoco_warp\": mj_data "
      "is frozen at the build pose there (its ncon stays 0, which used to publish \"nothing is touching\" for "
      "the whole run), so getContactPoints / /sim/contacts / /sim/grips / the damage tracker now fall back to "
      "newton's own narrow phase. That IS live, and body PAIRS are correct -- but contact POINTS are shape "
      "support points in the first shape's body frame, NOT world witnesses, and depth reads 0. Use the CPU "
      "newtonSolver \"mujoco\" (the default) for full-fidelity contacts.");
}

int OmNewtonBackend::getContacts(std::vector<OmNewtonContact> &out) const {
  // W4.1/W4.2: pull this step's native rigid contacts from the embedded runtime (get_contacts returns a flat
  // list, 10 values per contact: bodyA,bodyB, point(3), normal(3), depth, |force|). GIL is held on this
  // thread (single embedded interpreter, called from the step thread) -- same as the other Py call methods.
  out.clear();
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr)
    return -1;
  if (mSolverIsMuJoCoWarp)
    warnGpuReadbackOnWarp(false);   // W1.1: degraded, not declined -- see the helper
  PyObject *r = PyObject_CallMethod(mRuntime->world, "get_contacts", nullptr);
  if (r == nullptr)
    return reportPyError("get_contacts");
  int count = -1;
  if (PyList_Check(r)) {
    const Py_ssize_t len = PyList_Size(r);
    out.reserve((size_t)(len / 10));
    for (Py_ssize_t i = 0; i + 10 <= len; i += 10) {
      OmNewtonContact c;
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

int OmNewtonBackend::raycastBatch(int n, const double *raysIn, OmNewtonRayHit *out,
                                  const int *excludeBodies, int nExclude) const {
  // Kernel blocker #1 (ode-retirement-campaign.md): the Newton-side answer to
  // ODE ray geoms. Same GIL discipline as getContacts (single embedded
  // interpreter, called from the main/step thread).
  if (n <= 0 || raysIn == nullptr || out == nullptr)
    return -1;
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->running)
    return -1;
  // W1.1: mj_data is frozen at the build pose under mujoco_warp, so the runtime
  // declines. Short-circuit here too -- there is nothing to gain from crossing
  // the FFI to be told so, and this is the only place that can log it.
  if (mSolverIsMuJoCoWarp) {
    warnGpuReadbackOnWarp(true);
    return -1;
  }
  PyObject *const rays = PyList_New((Py_ssize_t)n * 7);
  for (Py_ssize_t i = 0; i < (Py_ssize_t)n * 7; ++i)
    PyList_SET_ITEM(rays, i, PyFloat_FromDouble(raysIn[i]));
  PyObject *const excl = PyList_New(nExclude);
  for (int i = 0; i < nExclude; ++i)
    PyList_SET_ITEM(excl, i, PyLong_FromLong(excludeBodies[i]));
  PyObject *const r = PyObject_CallMethod(mRuntime->world, "raycast_batch", "(OO)", rays, excl);
  Py_DECREF(rays);
  Py_DECREF(excl);
  if (r == nullptr)
    return reportPyError("raycast_batch");
  int count = -1;
  if (PyList_Check(r) && PyList_Size(r) == (Py_ssize_t)n * 5) {
    for (int i = 0; i < n; ++i) {
      OmNewtonRayHit &h = out[i];
      h.dist = PyFloat_AsDouble(PyList_GetItem(r, (Py_ssize_t)i * 5));
      h.newtonBody = (int)PyLong_AsLong(PyList_GetItem(r, (Py_ssize_t)i * 5 + 1));
      h.normal[0] = PyFloat_AsDouble(PyList_GetItem(r, (Py_ssize_t)i * 5 + 2));
      h.normal[1] = PyFloat_AsDouble(PyList_GetItem(r, (Py_ssize_t)i * 5 + 3));
      h.normal[2] = PyFloat_AsDouble(PyList_GetItem(r, (Py_ssize_t)i * 5 + 4));
    }
    count = n;
  }
  Py_DECREF(r);
  return count;
}

int OmNewtonBackend::solveIk(int linkBodyIdx, int nTargets, const double *targets, const double *rotations,
                             const std::vector<int> &jointSlots, const double *toolOffset, int iterations,
                             std::vector<double> &anglesOut, std::vector<double> &residualsOut) const {
  // Batched IK preview (internal parity plan, item W2.1): World.solve_ik on the live
  // model. Same PyList build / size-validated unpack as raycastBatch, same
  // explicit GIL discipline as addWeldSlot (this is reachable from the
  // supervisor's immediate-message path, not only the step thread).
  anglesOut.clear();
  residualsOut.clear();
  if (linkBodyIdx < 0 || nTargets <= 0 || targets == nullptr || jointSlots.empty())
    return -1;
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->running)
    return -1;
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *const targetList = PyList_New((Py_ssize_t)nTargets * 3);
  for (Py_ssize_t i = 0; i < (Py_ssize_t)nTargets * 3; ++i)
    PyList_SET_ITEM(targetList, i, PyFloat_FromDouble(targets[i]));
  PyObject *const slotList = PyList_New((Py_ssize_t)jointSlots.size());
  for (size_t i = 0; i < jointSlots.size(); ++i)
    PyList_SET_ITEM(slotList, (Py_ssize_t)i, PyLong_FromLong(jointSlots[i]));
  PyObject *const rotationList = PyList_New(rotations ? (Py_ssize_t)nTargets * 4 : 0);
  if (rotations)
    for (Py_ssize_t i = 0; i < (Py_ssize_t)nTargets * 4; ++i)
      PyList_SET_ITEM(rotationList, i, PyFloat_FromDouble(rotations[i]));
  PyObject *const seedList = PyList_New(0);  // empty => seed from the LIVE joint angles
  PyObject *const toolList = PyList_New(toolOffset ? 3 : 0);
  if (toolOffset)
    for (Py_ssize_t i = 0; i < 3; ++i)
      PyList_SET_ITEM(toolList, i, PyFloat_FromDouble(toolOffset[i]));
  // solve_ik(link_index, targets, jointSlots, rotations, seeds, iterations, clamp_to_limits, tool_offset)
  PyObject *const r = PyObject_CallMethod(mRuntime->world, "solve_ik", "(iOOOOiiO)", linkBodyIdx, targetList, slotList,
                                          rotationList, seedList, iterations > 0 ? iterations : 64, 1, toolList);
  Py_DECREF(targetList);
  Py_DECREF(slotList);
  Py_DECREF(rotationList);
  Py_DECREF(seedList);
  Py_DECREF(toolList);
  if (r == nullptr) {
    const int err = reportPyError("solve_ik");
    PyGILState_Release(gstate);
    return err;
  }
  // Contract: n*len(jointSlots) angles, then n residual norms (metres).
  const Py_ssize_t expected = (Py_ssize_t)nTargets * (Py_ssize_t)jointSlots.size() + nTargets;
  int rc = -1;
  if (PyList_Check(r) && PyList_Size(r) == expected) {
    const Py_ssize_t nAngles = (Py_ssize_t)nTargets * (Py_ssize_t)jointSlots.size();
    anglesOut.reserve((size_t)nAngles);
    for (Py_ssize_t i = 0; i < nAngles; ++i)
      anglesOut.push_back(PyFloat_AsDouble(PyList_GetItem(r, i)));
    residualsOut.reserve((size_t)nTargets);
    for (Py_ssize_t i = 0; i < (Py_ssize_t)nTargets; ++i)
      residualsOut.push_back(PyFloat_AsDouble(PyList_GetItem(r, nAngles + i)));
    rc = 0;
  } else
    OmLog::warning(QString("[OmNewtonBackend] solve_ik returned an unexpected shape (expected %1 values)")
                     .arg((qlonglong)expected));
  Py_DECREF(r);
  PyGILState_Release(gstate);
  return rc;
}

int OmNewtonBackend::addWeldSlot(int bodyIdx) {
  // Weld-slot API (Connector / VacuumGripper -- _scratch/design_weld_touch.md).
  // BUILD phase: reserves one INACTIVE MuJoCo equality-weld constraint
  // anchored on bodyIdx (weld-to-world placeholder). mjModel.eq_* arrays are
  // compile-time sized, so a slot never allocated here can never be engaged
  // later. Returns the slot id (>= 0) or -1.
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  PyGILState_STATE gstate = PyGILState_Ensure();  // hold the GIL across the FFI call (see addJointRevolute)
  PyObject *r = PyObject_CallMethod(mRuntime->world, "add_weld_slot", "(i)", bodyIdx);
  if (r == nullptr) {
    const int err = reportPyError("add_weld_slot");
    PyGILState_Release(gstate);
    return err;
  }
  const long slot = PyLong_AsLong(r);
  Py_DECREF(r);
  PyGILState_Release(gstate);
  return static_cast<int>(slot);
}

// One-shot mujoco_warp warning shared by weldEngage/weldRelease: phase 1 welds
// are CPU SolverMuJoCo only (the default) -- the GPU notify path cannot
// retarget eq_obj*id at runtime, and direct mjw_model surgery is graph-capture
// hostile. Loud by design: a silently-dead lock is the exact defect class this
// feature removes.
static void warnWeldsUnsupportedOnWarpOnce() {
  static bool warned = false;
  if (warned)
    return;
  warned = true;
  OmLog::warning(
    "[OmNewtonBackend] Connector/VacuumGripper welds require the CPU newtonSolver \"mujoco\" (the default) -- this "
    "world runs the GPU \"mujoco_warp\" solver, where runtime weld retargeting is not supported yet. Locked devices "
    "will NOT hold. Remove the newtonSolver \"mujoco_warp\" pin (or OMNISIM_NEWTON_MJWARP) to use welds.");
}

int OmNewtonBackend::weldEngage(int slot, int bodyA, int bodyB) {
  // RUN phase: activate `slot` welding bodyA <-> bodyB at their CURRENT
  // relative pose (zero-snap; the helper computes the eq_data write from the
  // live xpos/xquat). bodyB < 0 welds bodyA to the WORLD. When bodyA < 0 the
  // helper swaps so obj1 is the real body -- mirror ODE's dJointAttach
  // body1==0 swap and track the inversion caller-side for the rupture sign.
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->running)
    return -1;
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *r = PyObject_CallMethod(mRuntime->world, "weld_engage", "(iii)", slot, bodyA, bodyB);
  if (r == nullptr) {
    const int err = reportPyError("weld_engage");
    PyGILState_Release(gstate);
    return err;
  }
  const long rc = PyLong_AsLong(r);
  Py_DECREF(r);
  PyGILState_Release(gstate);
  if (rc == -2) {
    warnWeldsUnsupportedOnWarpOnce();
    return -1;
  }
  return rc == 0 ? 0 : -1;
}

int OmNewtonBackend::weldRelease(int slot) {
  // RUN phase: deactivate `slot` (mjData.eq_active[slot] = 0). 0/-1.
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->running)
    return -1;
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *r = PyObject_CallMethod(mRuntime->world, "weld_release", "(i)", slot);
  if (r == nullptr) {
    const int err = reportPyError("weld_release");
    PyGILState_Release(gstate);
    return err;
  }
  const long rc = PyLong_AsLong(r);
  Py_DECREF(r);
  PyGILState_Release(gstate);
  if (rc == -2) {
    warnWeldsUnsupportedOnWarpOnce();
    return -1;
  }
  return rc == 0 ? 0 : -1;
}

// Shared list-of-6-doubles unpack for weldForce/touchForce. Returns 0 and
// fills out[6] on success; -1 on any other shape (incl. the helper's empty
// "unavailable" list, matching raycastBatch's convention).
static int unpackWrench6(PyObject *r, double out[6]) {
  if (!PyList_Check(r) || PyList_Size(r) != 6)
    return -1;
  for (Py_ssize_t i = 0; i < 6; ++i)
    out[i] = PyFloat_AsDouble(PyList_GetItem(r, i));
  return 0;
}

int OmNewtonBackend::weldForce(int slot, double out[6]) const {
  // RUN phase: world-frame constraint wrench of an ACTIVE slot from the last
  // completed tick's SOLVED efc rows (snapshotted runtime-side before the
  // Cartesian refresh): out[0..2] = force ON bodyA/obj1, out[3..5] = torque.
  // Zeros when inactive or not yet stepped (matches dJointFeedback-not-yet-
  // populated semantics). 0/-1.
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->running)
    return -1;
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *r = PyObject_CallMethod(mRuntime->world, "weld_force", "(i)", slot);
  if (r == nullptr) {
    const int err = reportPyError("weld_force");
    PyGILState_Release(gstate);
    return err;
  }
  const int rc = unpackWrench6(r, out);
  Py_DECREF(r);
  PyGILState_Release(gstate);
  return rc;
}

int OmNewtonBackend::touchForce(int bodyIdx, double out[6]) const {
  // RUN phase: ODE-f1-compatible mount wrench of a welded (fixed-joint,
  // un-folded) child body -- the NEGATED MuJoCo cfrc_int after
  // mj_rnePostConstraint, world-aligned axes, from the last completed tick:
  // out[0..2] force, out[3..5] torque. The negation makes it byte-compatible
  // with the ODE math in OmTouchSensor::computeValue (f1 = force on the
  // PARENT, node[0] of the mount joint). First call registers the body; rows
  // arrive from the next tick's snapshot. 0/-1.
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->running)
    return -1;
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *r = PyObject_CallMethod(mRuntime->world, "touch_force", "(i)", bodyIdx);
  if (r == nullptr) {
    const int err = reportPyError("touch_force");
    PyGILState_Release(gstate);
    return err;
  }
  const int rc = unpackWrench6(r, out);
  Py_DECREF(r);
  PyGILState_Release(gstate);
  return rc;
}

int OmNewtonBackend::addShapeCylinder(int bodyIdx, double radius, double halfHeight,
                                      double cx, double cy, double cz,
                                      double qx, double qy, double qz, double qw) {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  // 1 int + 9 doubles: radius, halfHeight, cx,cy,cz, qx,qy,qz,qw. The quaternion
  // is the AUTHORED collider orientation (see the header note): the python side
  // no longer invents a -90 deg about X, so an unrotated Cylinder now stays
  // Z-aligned, which is what OmCylinder and URDF both mean by a cylinder.
  PyObject *r = PyObject_CallMethod(mRuntime->world, "add_shape_cylinder", "(iddddddddd)",
                                     bodyIdx, radius, halfHeight, cx, cy, cz, qx, qy, qz, qw);
  if (r == nullptr)
    return reportPyError("add_shape_cylinder");
  Py_DECREF(r);
  return 0;
}

int OmNewtonBackend::addShapeCapsule(int bodyIdx, double radius, double halfHeight,
                                     double cx, double cy, double cz,
                                     double qx, double qy, double qz, double qw) {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  // Same 1 int + 9 doubles as the cylinder above. An authored Capsule used to
  // lose BOTH its offset and its rotation here -- the call carried neither.
  PyObject *r = PyObject_CallMethod(mRuntime->world, "add_shape_capsule", "(iddddddddd)",
                                     bodyIdx, radius, halfHeight, cx, cy, cz, qx, qy, qz, qw);
  if (r == nullptr)
    return reportPyError("add_shape_capsule");
  Py_DECREF(r);
  return 0;
}

int OmNewtonBackend::addShapePlane(int bodyIdx, double cx, double cy, double cz) {
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
int OmNewtonBackend::addShapeMesh(int bodyIdx, const double *vertices, int nVertices,
                                  const int *indices, int nTriangles,
                                  double cx, double cy, double cz,
                                  double qx, double qy, double qz, double qw) {
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
  // 1 int + 2 objects + 1 int + 7 doubles: cx,cy,cz then qx,qy,qz,qw.
  PyObject *r = PyObject_CallMethod(mRuntime->world, "add_shape_mesh", "(iOOiddddddd)",
                                     bodyIdx, pyV, pyI, nVertices, cx, cy, cz, qx, qy, qz, qw);
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

// Native heightfield collision for an ElevationGrid boundingObject (terrain). heights = a flat
// xDimension*yDimension array in the node's row-major order (index = y*xDimension + x). Marshalled
// into a Python list once at world load, GIL held across the FFI work, exactly like addShapeMesh.
//
// See the header for why bodyIdx is only a transform lookup here: newton heightfields are always
// world-static, so add_shape_heightfield takes no body.
int OmNewtonBackend::addShapeHeightfield(int bodyIdx, const double *heights,
                                         int xDimension, int yDimension,
                                         double xSpacing, double ySpacing,
                                         double cx, double cy, double cz) {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  if (heights == nullptr || xDimension < 2 || yDimension < 2)
    return -1;
  // Cap the sample count before Python is entered, for the same reason addClothGrid does: a typo'd
  // dimension would otherwise allocate the product inside the interpreter and take the process down
  // there, in a message that names neither the world nor the node.
  const long long samples = static_cast<long long>(xDimension) * static_cast<long long>(yDimension);
  if (samples > 4000000LL) {
    OmLog::warning(QString("[OmNewtonBackend] add_shape_heightfield refused: %1 x %2 would need "
                           "%3 height samples (cap 4000000)")
                     .arg(xDimension)
                     .arg(yDimension)
                     .arg(samples));
    return -1;
  }
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *pyH = PyList_New(static_cast<Py_ssize_t>(samples));
  if (pyH == nullptr) {
    PyGILState_Release(gstate);
    return -1;
  }
  for (Py_ssize_t i = 0; i < static_cast<Py_ssize_t>(samples); ++i)
    PyList_SetItem(pyH, i, PyFloat_FromDouble(heights[i]));  // SetItem steals the new ref
  // Wire order MUST stay in lockstep with omnisim_newton_runtime.py's World.add_shape_heightfield:
  //   body_idx, heights, x_dimension, y_dimension, x_spacing, y_spacing, cx, cy, cz
  PyObject *r = PyObject_CallMethod(mRuntime->world, "add_shape_heightfield", "(iOiiddddd)",
                                    bodyIdx, pyH, xDimension, yDimension,
                                    xSpacing, ySpacing, cx, cy, cz);
  Py_DECREF(pyH);
  if (r == nullptr) {
    const int err = reportPyError("add_shape_heightfield");
    PyGILState_Release(gstate);
    return err;
  }
  Py_DECREF(r);
  PyGILState_Release(gstate);
  return 0;
}

int OmNewtonBackend::addClothGrid(const double pos[3], const double quat[4],
                                  int dimX, int dimY, double cellX, double cellY,
                                  double mass, double particleRadius,
                                  double triKe, double triKa, double triKd,
                                  double edgeKe, double edgeKd,
                                  int fixFlags, int *endOut) {
  // BUILD phase, exactly like the addShape* family above: the cloth's particles
  // go into the ModelBuilder, so this has to land before finalizeWorld().
  //
  // Wire order -- MUST stay in lockstep with omnisim_newton_runtime.py's
  // World.add_cloth_grid, whose positional order this mirrors exactly:
  //   pos_x, pos_y, pos_z,               3 doubles
  //   qx, qy, qz, qw,                    4 doubles  (w LAST, warp order)
  //   dim_x, dim_y,                      2 ints     (CELLS, not vertices)
  //   cell_x, cell_y, mass, radius,      4 doubles
  //   tri_ke, tri_ka, tri_kd,            3 doubles  (negative = derive)
  //   edge_ke, edge_kd,                  2 doubles  (negative = derive)
  //   fix_left, fix_right, fix_top,      4 ints used as bools -- ⚠ note the
  //   fix_bottom                           TOP/BOTTOM order, which is NOT the
  //                                        left/right/bottom/top order of the
  //                                        OmNewtonClothFix enum
  // The runtime's remaining args (vx, vy, vz, label) keep their defaults.
  //
  // Returns a (particle_start, particle_end) TUPLE, not a scalar.
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  if (pos == nullptr || quat == nullptr || dimX <= 0 || dimY <= 0)
    return -1;
  // Guard the one arithmetic that can overflow an int before it reaches the
  // runtime: a typo'd dimX/dimY (say 100000) would otherwise allocate the
  // product's worth of particles inside Python and take the process down there,
  // where the message names neither the world nor the node.
  const long long verts = (static_cast<long long>(dimX) + 1) * (static_cast<long long>(dimY) + 1);
  if (verts > 4000000LL) {
    OmLog::warning(QString("[OmNewtonBackend] add_cloth_grid refused: %1 x %2 cells would need "
                           "%3 particles (cap 4000000)")
                     .arg(dimX)
                     .arg(dimY)
                     .arg(verts));
    return -1;
  }
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *r = PyObject_CallMethod(mRuntime->world, "add_cloth_grid",
                                    // 7d (pos+quat) 2i (dims) 9d (cell/mass/radius
                                    // + 3 tri + 2 edge) 4i (fix) = 22 args.
                                    "(dddddddiidddddddddiiii)",
                                    pos[0], pos[1], pos[2],
                                    quat[0], quat[1], quat[2], quat[3],
                                    dimX, dimY,
                                    cellX, cellY, mass, particleRadius,
                                    triKe, triKa, triKd,
                                    edgeKe, edgeKd,
                                    (fixFlags & OmClothFixLeft) ? 1 : 0,
                                    (fixFlags & OmClothFixRight) ? 1 : 0,
                                    (fixFlags & OmClothFixTop) ? 1 : 0,
                                    (fixFlags & OmClothFixBottom) ? 1 : 0);
  if (r == nullptr) {
    const int err = reportPyError("add_cloth_grid");
    PyGILState_Release(gstate);
    return err;
  }
  long start = -1, end = -1;
  if (!PyArg_ParseTuple(r, "ll", &start, &end)) {
    PyErr_Clear();
    Py_DECREF(r);
    PyGILState_Release(gstate);
    OmLog::warning("[OmNewtonBackend] add_cloth_grid: expected a (start, end) tuple");
    return -1;
  }
  Py_DECREF(r);
  PyGILState_Release(gstate);
  if (end <= start)
    return -1;  // an empty range is a cloth that registered nothing
  if (endOut != nullptr)
    *endOut = static_cast<int>(end);
  return static_cast<int>(start);
}

// One cloth sheet from an arbitrary triangle mesh -- a garment. Marshalling is addShapeMesh's,
// verbatim (two PyLists built once at world load with the GIL held across the whole FFI section);
// the return contract is addClothGrid's (a (start, end) tuple, -1 on failure).
int OmNewtonBackend::addClothMesh(const double pos[3], const double quat[4],
                                  const double *vertices, int nVertices,
                                  const int *indices, int nTriangles,
                                  double density, double particleRadius,
                                  double triKe, double triKa, double triKd,
                                  double edgeKe, double edgeKd,
                                  double scale, double pinTopBand, int *endOut) {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  if (pos == nullptr || quat == nullptr || vertices == nullptr || indices == nullptr)
    return -1;
  if (nVertices <= 0 || nTriangles <= 0)
    return -1;
  // Same ceiling, and the same reason, as addClothGrid / addShapeHeightfield: refuse an
  // absurd allocation HERE, where the message can name the count, rather than inside the
  // embedded interpreter where it cannot.
  if (static_cast<long long>(nVertices) > 4000000LL) {
    OmLog::warning(QString("[OmNewtonBackend] add_cloth_mesh refused: %1 vertices (cap 4000000)")
                     .arg(nVertices));
    return -1;
  }
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

  // Wire order MUST stay in lockstep with omnisim_newton_runtime.py's World.add_cloth_mesh,
  // whose positional order this mirrors exactly:
  //   pos_x, pos_y, pos_z                3 d
  //   qx, qy, qz, qw                     4 d   (w LAST, warp order)   -- 7 d so far
  //   vertices, indices                  2 O
  //   n_vertices                         1 i
  //   density, particle_radius           2 d
  //   tri_ke, tri_ka, tri_kd             3 d   (negative = derive)
  //   edge_ke, edge_kd                   2 d   (negative = derive)
  //   scale, pin_top_band                2 d                          -- 9 d after the i
  // The runtime's remaining args (vx, vy, vz, label) keep their defaults.
  //
  // ⚠ COUNT THE FORMAT CHARACTERS AGAINST THE ARGUMENTS BEFORE EDITING THIS CALL. A
  // mismatch is not a compile error and not a Python exception -- PyObject_CallMethod reads
  // whatever is next in the varargs list, so one missing 'd' silently shifts every later
  // argument by one and the sheet is built with, say, tri_kd as its bending stiffness.
  // "(ddddddd" = 7, "OO" = 2, "i" = 1, "ddddddddd" = 9  ->  19 format chars, 19 arguments.
  PyObject *r = PyObject_CallMethod(mRuntime->world, "add_cloth_mesh",
                                    "(dddddddOOiddddddddd)",
                                    pos[0], pos[1], pos[2],          // 3 d
                                    quat[0], quat[1], quat[2], quat[3],  // 4 d  (7)
                                    pyV, pyI, nVertices,             // O O i
                                    density, particleRadius,         // 2 d  (2)
                                    triKe, triKa, triKd,             // 3 d  (5)
                                    edgeKe, edgeKd,                  // 2 d  (7)
                                    scale, pinTopBand);              // 2 d  (9)
  Py_DECREF(pyV);
  Py_DECREF(pyI);
  if (r == nullptr) {
    const int err = reportPyError("add_cloth_mesh");
    PyGILState_Release(gstate);
    return err;
  }
  long start = -1, end = -1;
  if (!PyArg_ParseTuple(r, "ll", &start, &end)) {
    PyErr_Clear();
    Py_DECREF(r);
    PyGILState_Release(gstate);
    OmLog::warning("[OmNewtonBackend] add_cloth_mesh: expected a (start, end) tuple");
    return -1;
  }
  Py_DECREF(r);
  PyGILState_Release(gstate);
  if (end <= start)
    return -1;  // an empty range is a cloth that registered nothing
  if (endOut != nullptr)
    *endOut = static_cast<int>(end);
  return static_cast<int>(start);
}

// One volumetric (tet FEM) soft block. Structurally the twin of addClothGrid above -- same
// build-phase guard, same GIL discipline, same (start, end) tuple return -- because a soft body
// rides the same SolverVBD entry and the same coupled path cloth does.
int OmNewtonBackend::addSoftGrid(const double pos[3], const double quat[4],
                                 int dimX, int dimY, int dimZ,
                                 double cellX, double cellY, double cellZ,
                                 double density, double kMu, double kLambda, double kDamp,
                                 double particleRadius, int fixFlags, int *endOut) {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  if (pos == nullptr || quat == nullptr || dimX <= 0 || dimY <= 0 || dimZ <= 0)
    return -1;
  // Cap the particle count before Python is entered, exactly as addClothGrid does: a typo'd
  // dimension would otherwise allocate the product inside the interpreter and take the process
  // down there, in a message naming neither the world nor the node. A soft grid is a VOLUME, so
  // this overflows on much smaller-looking numbers than a sheet does -- 160^3 is already 4.1 M.
  const long long verts = (static_cast<long long>(dimX) + 1) *
                          (static_cast<long long>(dimY) + 1) *
                          (static_cast<long long>(dimZ) + 1);
  if (verts > 4000000LL) {
    OmLog::warning(QString("[OmNewtonBackend] add_soft_grid refused: %1 x %2 x %3 cells would need "
                           "%4 particles (cap 4000000)")
                     .arg(dimX)
                     .arg(dimY)
                     .arg(dimZ)
                     .arg(verts));
    return -1;
  }
  PyGILState_STATE gstate = PyGILState_Ensure();
  // Wire order MUST stay in lockstep with omnisim_newton_runtime.py's World.add_soft_grid:
  //   pos_x, pos_y, pos_z, qx, qy, qz, qw    7 doubles (w LAST, warp order)
  //   dim_x, dim_y, dim_z                    3 ints
  //   cell_x, cell_y, cell_z,                3 doubles
  //   density, k_mu, k_lambda, k_damp,       4 doubles
  //   particle_radius                        1 double
  //   fix_left, fix_right, fix_top,          4 ints used as bools -- ⚠ note the TOP/BOTTOM
  //   fix_bottom                                order, which is NOT the enum's declaration order
  PyObject *r = PyObject_CallMethod(mRuntime->world, "add_soft_grid",
                                    "(dddddddiiiddddddddiiii)",
                                    pos[0], pos[1], pos[2],
                                    quat[0], quat[1], quat[2], quat[3],
                                    dimX, dimY, dimZ,
                                    cellX, cellY, cellZ,
                                    density, kMu, kLambda, kDamp,
                                    particleRadius,
                                    (fixFlags & OmClothFixLeft) ? 1 : 0,
                                    (fixFlags & OmClothFixRight) ? 1 : 0,
                                    (fixFlags & OmClothFixTop) ? 1 : 0,
                                    (fixFlags & OmClothFixBottom) ? 1 : 0);
  if (r == nullptr) {
    const int err = reportPyError("add_soft_grid");
    PyGILState_Release(gstate);
    return err;
  }
  long start = -1, end = -1;
  if (!PyArg_ParseTuple(r, "ll", &start, &end)) {
    PyErr_Clear();
    Py_DECREF(r);
    PyGILState_Release(gstate);
    OmLog::warning("[OmNewtonBackend] add_soft_grid: expected a (start, end) tuple");
    return -1;
  }
  Py_DECREF(r);
  PyGILState_Release(gstate);
  if (end <= start)
    return -1;  // an empty range is a soft body that registered nothing
  if (endOut != nullptr)
    *endOut = static_cast<int>(end);
  return static_cast<int>(start);
}

// The tet mesh's open faces, block-local, as int32 triples. Snapshotted by the runtime at
// authoring time because newton's ModelBuilder is consumed at finalize() and the winding cannot
// be re-derived afterwards the way a cloth sheet's can.
int OmNewtonBackend::softSurfaceTriangles(int gridIndex, int *indices, int maxTriangles) const {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr)
    return -1;
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *r = PyObject_CallMethod(mRuntime->world, "soft_surface_packed", "(i)", gridIndex);
  if (r == nullptr) {
    const int err = reportPyError("soft_surface_packed");
    PyGILState_Release(gstate);
    return err;
  }
  char *buf = nullptr;
  Py_ssize_t len = 0;
  if (PyBytes_AsStringAndSize(r, &buf, &len) != 0 || buf == nullptr) {
    PyErr_Clear();
    Py_DECREF(r);
    PyGILState_Release(gstate);
    return -1;
  }
  // 3 int32 per triangle. A short or ragged read means the runtime and this file have drifted
  // apart, which is worth a warning rather than a silently truncated mesh.
  const Py_ssize_t stride = static_cast<Py_ssize_t>(3 * sizeof(int));
  if (len % stride != 0) {
    Py_DECREF(r);
    PyGILState_Release(gstate);
    OmLog::warning(QString("[OmNewtonBackend] soft_surface_packed returned %1 bytes, not a "
                           "multiple of %2").arg(static_cast<long long>(len))
                     .arg(static_cast<long long>(stride)));
    return -1;
  }
  const int nTris = static_cast<int>(len / stride);
  if (indices == nullptr) {          // count-only query
    Py_DECREF(r);
    PyGILState_Release(gstate);
    return nTris;
  }
  const int copy = (nTris < maxTriangles) ? nTris : maxTriangles;
  if (copy > 0)
    memcpy(indices, buf, static_cast<size_t>(copy) * static_cast<size_t>(stride));
  Py_DECREF(r);
  PyGILState_Release(gstate);
  return copy;
}

int OmNewtonBackend::addJointRevolute(int parentIdx, int childIdx,
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

int OmNewtonBackend::addJointHinge2(int parentIdx, int childIdx,
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

int OmNewtonBackend::addJointBall(int parentIdx, int childIdx,
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

int OmNewtonBackend::addJointHinge2Motorized(int parentIdx, int childIdx,
                                             const double axis1[3], const double axis2[3],
                                             const double parentAnchor[3], const double childAnchor[3],
                                             const double childRot[4],
                                             const double gains[4], const double limits[4],
                                             const double efforts[2], const double velLimits[2]) {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  PyGILState_STATE gstate = PyGILState_Ensure();  // hold the GIL across the FFI call (see addJointRevolute)
  // Nested tuples rather than a 20-double flat format string: the helper's
  // signature takes vectors, so the grouping is checked by the call itself.
  PyObject *r = PyObject_CallMethod(
    mRuntime->world, "add_joint_hinge2_motorized",
    "(ii(ddd)(ddd)(dddd)(ddd)(ddd)(dddd)(dddd)(dd)(dd))",
    parentIdx, childIdx,
    parentAnchor[0], parentAnchor[1], parentAnchor[2],
    childAnchor[0], childAnchor[1], childAnchor[2],
    childRot[0], childRot[1], childRot[2], childRot[3],
    axis1[0], axis1[1], axis1[2],
    axis2[0], axis2[1], axis2[2],
    gains[0], gains[1], gains[2], gains[3],
    limits[0], limits[1], limits[2], limits[3],
    efforts[0], efforts[1],
    velLimits[0], velLimits[1]);
  if (r == nullptr) {
    const int err = reportPyError("add_joint_hinge2_motorized");
    PyGILState_Release(gstate);
    return err;
  }
  long idx = PyLong_AsLong(r);
  Py_DECREF(r);
  PyGILState_Release(gstate);
  return static_cast<int>(idx);
}

int OmNewtonBackend::addJointBallMotorized(int parentIdx, int childIdx,
                                           const double parentAnchor[3], const double parentQuat[4],
                                           const double childAnchor[3], const double childQuat[4],
                                           const double gains[6], const double limits[6],
                                           const double efforts[3], const double velLimits[3]) {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  PyGILState_STATE gstate = PyGILState_Ensure();  // hold the GIL across the FFI call (see addJointRevolute)
  PyObject *r = PyObject_CallMethod(
    mRuntime->world, "add_joint_ball_motorized",
    "(ii(ddd)(dddd)(ddd)(dddd)(dddddd)(dddddd)(ddd)(ddd))",
    parentIdx, childIdx,
    parentAnchor[0], parentAnchor[1], parentAnchor[2],
    parentQuat[0], parentQuat[1], parentQuat[2], parentQuat[3],
    childAnchor[0], childAnchor[1], childAnchor[2],
    childQuat[0], childQuat[1], childQuat[2], childQuat[3],
    gains[0], gains[1], gains[2], gains[3], gains[4], gains[5],
    limits[0], limits[1], limits[2], limits[3], limits[4], limits[5],
    efforts[0], efforts[1], efforts[2],
    velLimits[0], velLimits[1], velLimits[2]);
  if (r == nullptr) {
    const int err = reportPyError("add_joint_ball_motorized");
    PyGILState_Release(gstate);
    return err;
  }
  long idx = PyLong_AsLong(r);
  Py_DECREF(r);
  PyGILState_Release(gstate);
  return static_cast<int>(idx);
}

int OmNewtonBackend::addJointFixed(int parentIdx, int childIdx) {
  // FIXED (0-DOF) tree joint welding childIdx to parentIdx at their CURRENT
  // registered poses (the helper derives the relative transform from
  // builder.body_q, so no anchors cross the FFI). Used by the force-
  // TouchSensor un-fold: newton's MuJoCo conversion keeps a FIXED-joint child
  // as a separate WELDED mjc body with its own geoms and no joint element,
  // which is what per-body contact attribution and the cfrc_int mount-force
  // readback need. Same queue/topo-sort path as addJointRevolute.
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  PyGILState_STATE gstate = PyGILState_Ensure();  // hold the GIL across the FFI call (see addJointRevolute)
  PyObject *r = PyObject_CallMethod(mRuntime->world, "add_joint_fixed", "(ii)", parentIdx, childIdx);
  if (r == nullptr) {
    const int err = reportPyError("add_joint_fixed");
    PyGILState_Release(gstate);
    return err;
  }
  long idx = PyLong_AsLong(r);
  Py_DECREF(r);
  PyGILState_Release(gstate);
  return static_cast<int>(idx);
}

int OmNewtonBackend::addJointPrismatic(int parentIdx, int childIdx,
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

int OmNewtonBackend::setJointTargetVelocity(int jointIdx, double vel) {
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

namespace {
  // true when `v` differs from the cached value for `idx` (or there is none),
  // and records it. NaN never matches itself, so a NaN target always pushes.
  bool targetChanged(std::vector<double> &cache, int idx, double v) {
    if (idx < 0)
      return true;
    if (static_cast<size_t>(idx) >= cache.size())
      cache.resize(static_cast<size_t>(idx) + 1, std::numeric_limits<double>::quiet_NaN());
    if (cache[idx] == v)
      return false;
    cache[idx] = v;
    return true;
  }
}  // namespace

int OmNewtonBackend::setJointTargetVelocityIfChanged(int jointIdx, double vel) {
  if (!mAvailable || mRuntime == nullptr)
    return -1;
  if (!targetChanged(mRuntime->lastTargetVel, jointIdx, vel))
    return 0;
  return setJointTargetVelocity(jointIdx, vel);
}

int OmNewtonBackend::setJointTargetPositionIfChanged(int jointIdx, double pos) {
  if (!mAvailable || mRuntime == nullptr)
    return -1;
  if (!targetChanged(mRuntime->lastTargetPos, jointIdx, pos))
    return 0;
  return setJointTargetPosition(jointIdx, pos);
}

void OmNewtonBackend::clearJointTargetCache() {
  if (mRuntime == nullptr)
    return;
  mRuntime->lastTargetVel.clear();
  mRuntime->lastTargetPos.clear();
}

int OmNewtonBackend::setJointTargetPosition(int jointIdx, double pos) {
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

int OmNewtonBackend::setJointForce(int jointIdx, double tau) {
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

std::string OmNewtonBackend::diagDumpJointQ() const {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->running)
    return "(newton not running)";
  PyObject *r = PyObject_CallMethod(mRuntime->world, "diag_dump_joint_q", nullptr);
  if (r == nullptr) {
    PyErr_Clear();
    return "(call failed)";
  }
  const char *cstr = PyUnicode_AsUTF8(r);
  const std::string s = cstr ? cstr : "(decode failed)";
  Py_DECREF(r);
  return s;
}

void OmNewtonBackend::resetBodyPose(int bodyIdx, double x, double y, double z,
                                    double qx, double qy, double qz, double qw) {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->running)
    return;
  // Format is i + 7 d (bodyIdx + pos[3] + quat[4]). A stray 8th 'd' here used to
  // read a garbage value off the stack so PyObject_CallMethod raised and the
  // call failed SILENTLY every time -- which is why a Supervisor pose-write to a
  // free body never reached Newton (the long-standing "free-body teleport doesn't
  // stick under MuJoCo" symptom). 8 specifiers for 8 args:
  mRuntime->snapValid = false;  // Tier 1a: a read after this teleport must refetch
  PyObject *r = PyObject_CallMethod(mRuntime->world, "reset_body_pose",
                                     "(iddddddd)",
                                     bodyIdx, x, y, z, qx, qy, qz, qw);
  if (r == nullptr) {
    PyErr_Clear();
    return;
  }
  Py_DECREF(r);
}

void OmNewtonBackend::resetJointsToDefaults() {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->running)
    return;
  mRuntime->snapValid = false;  // Tier 1a: joint_q rewritten below
  clearJointTargetCache();      // item 5: reset clears targets python-side
  PyObject *r = PyObject_CallMethod(mRuntime->world, "reset_joints_to_defaults", nullptr);
  if (r == nullptr) {
    PyErr_Clear();
    return;
  }
  Py_DECREF(r);
}

int OmNewtonBackend::setJointTargetVelocityDof(int jointIdx, int dof, double vel) {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr)
    return -1;
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *r = PyObject_CallMethod(mRuntime->world, "set_joint_target_vel",
                                     "(idi)", jointIdx, vel, dof);
  if (r == nullptr) {
    const int err = reportPyError("set_joint_target_vel");
    PyGILState_Release(gstate);
    return err;
  }
  Py_DECREF(r);
  PyGILState_Release(gstate);
  return 0;
}

int OmNewtonBackend::setJointTargetPositionDof(int jointIdx, int dof, double pos) {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr)
    return -1;
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *r = PyObject_CallMethod(mRuntime->world, "set_joint_target_pos",
                                     "(idi)", jointIdx, pos, dof);
  if (r == nullptr) {
    const int err = reportPyError("set_joint_target_pos");
    PyGILState_Release(gstate);
    return err;
  }
  Py_DECREF(r);
  PyGILState_Release(gstate);
  return 0;
}

int OmNewtonBackend::setJointForceDof(int jointIdx, int dof, double tau) {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr)
    return -1;
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *r = PyObject_CallMethod(mRuntime->world, "set_joint_force",
                                     "(idi)", jointIdx, tau, dof);
  if (r == nullptr) {
    const int err = reportPyError("set_joint_force");
    PyGILState_Release(gstate);
    return err;
  }
  Py_DECREF(r);
  PyGILState_Release(gstate);
  return 0;
}

double OmNewtonBackend::getJointAngleDof(int jointIdx, int dof) const {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->running)
    return 0.0;
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *r = PyObject_CallMethod(mRuntime->world, "get_joint_angle_dof",
                                     "(ii)", jointIdx, dof);
  if (r == nullptr) {
    PyErr_Clear();  // const-ish read path -- surface the error silently, like getJointAngle
    PyGILState_Release(gstate);
    return 0.0;
  }
  const double a = PyFloat_AsDouble(r);
  Py_DECREF(r);
  PyGILState_Release(gstate);
  return a;
}

int OmNewtonBackend::getJointBallQuat(int jointIdx, double out[4]) const {
  out[0] = out[1] = out[2] = 0.0;
  out[3] = 1.0;
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->running)
    return -1;
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *r = PyObject_CallMethod(mRuntime->world, "get_joint_ball_quat",
                                     "(i)", jointIdx);
  if (r == nullptr) {
    PyErr_Clear();
    PyGILState_Release(gstate);
    return -1;
  }
  int rc = -1;
  if (PySequence_Check(r) && PySequence_Size(r) == 4) {
    rc = 0;
    for (int i = 0; i < 4; ++i) {
      PyObject *item = PySequence_GetItem(r, i);
      if (item == nullptr) {
        PyErr_Clear();
        rc = -1;
        break;
      }
      out[i] = PyFloat_AsDouble(item);
      Py_DECREF(item);
    }
    if (PyErr_Occurred() != nullptr) {
      PyErr_Clear();
      rc = -1;
    }
  }
  Py_DECREF(r);
  PyGILState_Release(gstate);
  return rc;
}

double OmNewtonBackend::getJointAngle(int jointIdx) const {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->running)
    return 0.0;
  // Snapshot rows are indexed by slot id with 0.0 fill for unmapped jointSlots --
  // the exact get_joint_angle contract. A slot past the snapshot (packed
  // blob predates the joint, or the joint side of the blob failed) falls
  // through to the per-call read.
  if (jointIdx >= 0 && ensureStepSnapshot(mRuntime) && jointIdx < mRuntime->snapSlotCount)
    return mRuntime->snapSlotAngle[jointIdx];
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
// OmLog::initFileLog removed any stale prior-run copy when it truncated the log at
// startup, so this file's mere presence == "Newton drove THIS run".
// Minimal JSON string escaping for the sidecar's one free-text field (the solver
// string can carry a Python error fragment). Quotes, backslashes and control
// characters only -- the schema's other values are fixed literals.
static std::string jsonEscape(const std::string &in) {
  std::string out;
  out.reserve(in.size() + 8);
  for (const char c : in) {
    switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        if ((unsigned char)c < 0x20) {
          char buf[8];
          snprintf(buf, sizeof(buf), "\\u%04x", c);
          out += buf;
        } else
          out += c;
    }
  }
  return out;
}

static void writeNewtonVerdictSidecar(const std::string &solver,
                                      const std::string &runtimeJson) {
  const std::string logPath = OmLog::logFilePath().toStdString();
  if (logPath.empty())
    return;  // no file log this run -> nowhere to co-locate; the log scrape still works
  const bool degraded =
    solver.find("FAILED") != std::string::npos || solver.find("XPBD fallback") != std::string::npos;
  std::ofstream f(logPath + ".newton.json", std::ios::binary | std::ios::trunc);
  if (!f.is_open())
    return;
  // Same compact schema QJsonDocument produced (keys are emitted alphabetically to
  // match the previous byte layout consumers may diff against).
  // `runtime` carries the newton/warp/mujoco versions + the model device, so a
  // result can never be mistaken for one produced by a different runtime. Keys
  // stay alphabetical (backend, degraded, finalised, runtime, solver) to match
  // the byte layout consumers may diff against. Only a well-formed object is
  // spliced; anything else degrades to {} rather than corrupting the JSON.
  const bool rtOk = runtimeJson.size() >= 2 && runtimeJson.front() == '{' && runtimeJson.back() == '}';
  f << "{\"backend\":\"newton\",\"degraded\":" << (degraded ? "true" : "false")
    << ",\"finalised\":true,\"runtime\":" << (rtOk ? runtimeJson : std::string("{}")) << ",\"solver\":\""
    << jsonEscape(solver) << "\"}";
}

int OmNewtonBackend::finalizeWorld() {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  PyGILState_STATE gstate = PyGILState_Ensure();
  // Re-assert the sticky solver preference onto THIS world right before
  // finalizing it. The build-phase set_solver_preference() call may have
  // targeted an earlier (now-discarded) world object during a GUI
  // multi-build load; this guarantees the world we're about to finalize
  // sees the requested solver.
  if (!mSolverPref.empty()) {
    PyObject *sp = PyObject_CallMethod(mRuntime->world, "set_solver_preference", "(s)", mSolverPref.c_str());
    if (sp == nullptr)
      PyErr_Clear();
    else
      Py_DECREF(sp);
  }
  // Same re-assert for the sticky per-world cone/impratio (WorldInfo
  // newtonCone / newtonImpratio): the build-phase set_contact_cone() may have
  // targeted a discarded world object during a GUI multi-build load.
  if (!mConePref.empty() || mImpratioPref > 0.0) {
    PyObject *cp = PyObject_CallMethod(mRuntime->world, "set_contact_cone", "(sd)", mConePref.c_str(), mImpratioPref);
    if (cp == nullptr)
      PyErr_Clear();
    else
      Py_DECREF(cp);
  }
  // Same re-assert for the sticky per-world contact dimensionality (WorldInfo
  // newtonCondim): the build-phase set_contact_condim() may have targeted a
  // discarded world object during a GUI multi-build load.
  if (mCondimPref > 0) {
    PyObject *dp = PyObject_CallMethod(mRuntime->world, "set_contact_condim", "(i)", mCondimPref);
    if (dp == nullptr)
      PyErr_Clear();
    else
      Py_DECREF(dp);
  }
  // Same re-assert for the sticky per-world noslip iteration count (WorldInfo
  // newtonNoslipIterations): the build-phase set_noslip_iterations() may have
  // targeted a discarded world object during a GUI multi-build load.  It
  // matters more here than for most of these, because the value is applied to
  // mj_model AFTER SolverMuJoCo is constructed -- i.e. inside finalize(), on
  // whichever world object this call is about to finalize.
  if (mNoslipItersPref > 0) {
    PyObject *np = PyObject_CallMethod(mRuntime->world, "set_noslip_iterations", "(i)", mNoslipItersPref);
    if (np == nullptr)
      PyErr_Clear();
    else
      Py_DECREF(np);
  }
  // Same re-assert for the sticky per-world cloth self-contact mode (WorldInfo
  // newtonClothSelfContact). This one matters MORE than most: the value is
  // consumed while finalize() assembles the SolverVBD kwargs, i.e. on
  // whichever world object this call is about to finalize -- so a build-phase
  // write that landed on a discarded world would silently give the grasp
  // world the draping default, which is the exact 24x slip the field exists
  // to prevent.
  if (mClothSelfContactPref >= 0) {
    PyObject *cs = PyObject_CallMethod(mRuntime->world, "set_cloth_self_contact", "(i)", mClothSelfContactPref);
    if (cs == nullptr)
      PyErr_Clear();
    else
      Py_DECREF(cs);
  }
  // Same re-assert for the sticky per-world constraint-buffer caps (WorldInfo
  // newtonNjmax / newtonNconmax): the build-phase set_constraint_buffers() may
  // have targeted a discarded world object during a GUI multi-build load.
  if (mNjmaxPref != 0 || mNconmaxPref != 0) {
    PyObject *bp = PyObject_CallMethod(mRuntime->world, "set_constraint_buffers", "(ii)", mNjmaxPref, mNconmaxPref);
    if (bp == nullptr)
      PyErr_Clear();
    else
      Py_DECREF(bp);
  }
  // Same re-assert for the sticky per-world contact/solver params (WorldInfo
  // newtonGroundMu / newtonContactKe / newtonContactKd / newtonIterations /
  // newtonLsIterations): the build-phase set_contact_solver_params() may have
  // targeted a discarded world object during a GUI multi-build load.
  if (mGroundMuPref >= 0.0 || mContactKePref > 0.0 || mContactKdPref > 0.0 || mIterationsPref > 0 ||
      mLsIterationsPref > 0) {
    PyObject *sp = PyObject_CallMethod(mRuntime->world, "set_contact_solver_params", "(dddii)", mGroundMuPref,
                                       mContactKePref, mContactKdPref, mIterationsPref, mLsIterationsPref);
    if (sp == nullptr)
      PyErr_Clear();
    else
      Py_DECREF(sp);
  }
  PyObject *r = PyObject_CallMethod(mRuntime->world, "finalize", nullptr);
  if (r == nullptr) {
    // FATAL, not a decline: nothing was built, so the whole world is inert.
    // See reportPyErrorFatal() for why this one call site is an ERROR while
    // every other reportPyError() stays a WARNING.
    //
    // Latched like the N15/N16 diagnostics, and for the same reason: a failed
    // finalize BUILDS NOTHING, so openForBuild stays true and OmSimulationWorld
    // ::step() calls back in on every tick. Un-latched that produced one line
    // per tick -- the measured SolidReference case logged 4254 of them, which is
    // how the one message that matters got buried. The retry itself is left
    // exactly as it was; only the repetition is dropped.
    int err;
    if (!mRuntime->finalizeFailureReported) {
      mRuntime->finalizeFailureReported = true;
      err = reportPyErrorFatal("finalize");
    } else {
      // Consume the exception anyway -- a Python error left set corrupts the
      // next unrelated call across the FFI -- but say nothing more.
      PyErr_Clear();
      err = -1;
    }
    PyGILState_Release(gstate);
    return err;
  }
  Py_DECREF(r);
  mRuntime->openForBuild = false;
  mRuntime->running = true;
  mRuntime->snapValid = false;  // Tier 1a: state arrays were just (re)built
  clearJointTargetCache();      // item 5: finalize rebuilt the control arrays
  // Read back which solver actually got built (Python stores it on the world as
  // _solver_kind / _solver_error) so a SILENT fall-back from the requested
  // MuJoCo (mujoco_warp) engine to XPBD -- a DIFFERENT solver an mujoco_warp-
  // trained RL policy will NOT survive -- is surfaced LOUDLY here instead of
  // hiding in .build_tmp/newton_solver.log (root cause of the G1 deploy gap:
  // a world plane on a non-static body silently failed the mjwarp build).
  std::string solverKind, solverErr;
  {
    PyObject *sk = PyObject_GetAttrString(mRuntime->world, "_solver_kind");
    if (sk != nullptr) {
      if (PyUnicode_Check(sk)) {
        const char *cstr = PyUnicode_AsUTF8(sk);
        solverKind = cstr ? cstr : "";
      }
      Py_DECREF(sk);
    } else
      PyErr_Clear();
    PyObject *se = PyObject_GetAttrString(mRuntime->world, "_solver_error");
    if (se != nullptr) {
      if (PyUnicode_Check(se)) {
        const char *cstr = PyUnicode_AsUTF8(se);
        solverErr = cstr ? cstr : "";
      }
      Py_DECREF(se);
    } else
      PyErr_Clear();
  }
  // W1.1 (internal parity plan): remember whether the solver that ACTUALLY built
  // is the GPU one, so the mj_data-backed readbacks can decline instead of
  // answering against the build pose. Matched on the effective-solver token the
  // runtime writes -- "MuJoCo (mujoco_warp, <why>)" -- and not on a bare
  // substring, because later suffixes carry the word WARP too (noslip). The
  // requested value is deliberately NOT used: a requested mujoco_warp that fell
  // back to CPU mj_step reads mj_data legitimately.
  mSolverIsMuJoCoWarp = solverKind.find("(mujoco_warp") != std::string::npos;
  if (mSolverIsMuJoCoWarp && !qEnvironmentVariableIsEmpty("OMNISIM_NEWTON_GPU_STALE_READBACK")) {
    const QString v = QString::fromUtf8(qgetenv("OMNISIM_NEWTON_GPU_STALE_READBACK")).trimmed().toLower();
    if (v != "0" && v != "false" && v != "off" && v != "no")
      mSolverIsMuJoCoWarp = false;   // exact-revert hatch: pre-fix behaviour, A/B only
  }
  mWarnedGpuRays = mWarnedGpuContacts = false;   // per world, never per process
  // finalizeWorld phase decomposition (OMNISIM_NEWTON_STEP_PROFILE=1). This
  // one-time cost sits INSIDE the physics bracket and dominates short runs,
  // CI and harness hot-reload; it had never been decomposed before 2026-08-09.
  {
    PyObject *rep = PyObject_CallMethod(mRuntime->world, "_fin_report", nullptr);
    if (rep != nullptr) {
      if (PyUnicode_Check(rep)) {
        const char *c = PyUnicode_AsUTF8(rep);
        if (c != nullptr && c[0] != '\0')
          OmLog::info(QString("[OmNewtonBackend] ") + c);
      }
      Py_DECREF(rep);
    } else
      PyErr_Clear();
  }
  PyGILState_Release(gstate);
  OmLog::info(QString::fromStdString("[OmNewtonBackend] world finalised (solver=" +
                                     (solverKind.empty() ? std::string("see .build_tmp/newton_solver.log") : solverKind) +
                                     ")"));
  std::string runtimeJson;
  {
    PyGILState_STATE g2 = PyGILState_Ensure();
    PyObject *rr = PyObject_CallMethod(mRuntime->world, "runtime_report", nullptr);
    if (rr != nullptr) {
      if (PyUnicode_Check(rr)) {
        const char *c = PyUnicode_AsUTF8(rr);
        if (c != nullptr)
          runtimeJson = c;
      }
      Py_DECREF(rr);
    } else
      PyErr_Clear();
    PyGILState_Release(g2);
  }
  writeNewtonVerdictSidecar(solverKind, runtimeJson);
  if (solverKind.find("FAILED") != std::string::npos || solverKind.find("XPBD fallback") != std::string::npos) {
    const QString msg = QString::fromStdString(
      "[OmNewtonBackend] *** MuJoCo solver was REQUESTED but FAILED to build -- "
      "FELL BACK TO '" +
      solverKind +
      "'. XPBD is a DIFFERENT physics engine; an mujoco_warp-trained "
      "policy will behave differently and may collapse. Fix the model so SolverMuJoCo "
      "constructs (details in .build_tmp/newton_solver.log). Error: " +
      (solverErr.empty() ? std::string("(none captured)") : solverErr.substr(0, 400)));
    // Newton enforcement (2026-06-29 default: no silent physics downgrade). A
    // requested-but-failed MuJoCo build silently swapping in XPBD is the same
    // class of bug as a silent Newton->ODE fall-back (it broke the G1 deploy),
    // so escalate it to a hard error under enforcement. Opt out with
    // OMNISIM_ALLOW_ODE_FALLBACK=1 (or FORCE_ODE/LEGACY) -- all three retired; the Python runtime's
    // OMNISIM_REQUIRE_MUJOCO_SOLVER=1 asserts the same thing one layer earlier.
    if (OmPhysicsBackendRegistry::newtonEnforced())
      OmLog::fatal(msg);
    else
      OmLog::warning(msg);
  }
  return 0;
}

int OmNewtonBackend::setSolverPreference(const std::string &name) {
  // Plumb the WorldInfo.newtonSolver choice to the runtime BEFORE finalize()
  // builds the solver. "mujoco" -> SolverMuJoCo (robust frictional contact for
  // grasps); anything else keeps the default GPU XPBD. Called from OmSolid's
  // Newton flush (build phase), so the world must be open for build.
  // Cache the request regardless of build state; finalizeWorld() re-asserts
  // it onto the world it actually finalizes, so a rebuild that resets the
  // Python-side _solver_pref can't silently drop us to XPBD.
  mSolverPref = name;
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  OmLog::info(QString::fromStdString("[OmNewtonBackend] solver preference set to '" + name + "'"));
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *r = PyObject_CallMethod(mRuntime->world, "set_solver_preference", "(s)", name.c_str());
  if (r == nullptr) {
    const int err = reportPyError("set_solver_preference");
    PyGILState_Release(gstate);
    return err;
  }
  Py_DECREF(r);
  PyGILState_Release(gstate);
  return 0;
}

int OmNewtonBackend::setClothCoupling(int bodyIndex, int mode) {
  // Per-Solid cloth-proxy visibility. Unlike the solver preference this is NOT
  // cached and re-asserted at finalize: it is keyed on a newton BODY INDEX, and
  // those indices are only valid within the build that produced them. A rebuild
  // re-registers every Solid (mNewtonBodyIndex resets to -1), so the flush
  // re-sends the whole roster against the new indices. Replaying a stale one
  // would silently point at whatever body now holds that index.
  if (bodyIndex < 0 || mode == 0)
    return -1;
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *r = PyObject_CallMethod(mRuntime->world, "set_cloth_coupling", "(ii)", bodyIndex,
                                    (mode > 0) ? 1 : -1);
  if (r == nullptr) {
    const int err = reportPyError("set_cloth_coupling");
    PyGILState_Release(gstate);
    return err;
  }
  Py_DECREF(r);
  PyGILState_Release(gstate);
  return 0;
}

int OmNewtonBackend::setContactCone(const std::string &cone, double impratio) {
  // Plumb the WorldInfo.newtonCone / newtonImpratio choice to the runtime
  // BEFORE finalize() constructs SolverMuJoCo. "" / 0 = MuJoCo stock
  // (pyramidal cone, impratio 1) -- exact current physics for every existing
  // world. The OMNISIM_NEWTON_CONE / OMNISIM_NEWTON_IMPRATIO env vars still
  // win (resolved runtime-side). Cache regardless of build state;
  // finalizeWorld() re-asserts onto the world it actually finalizes (same
  // multi-build race as mSolverPref).
  mConePref = cone;
  mImpratioPref = impratio;
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  if (cone.empty() && impratio <= 0.0)
    return 0;  // default; nothing to push
  OmLog::info(QString::fromStdString("[OmNewtonBackend] contact cone set to '" + cone + "', impratio " +
                                     std::to_string(impratio)));
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *r = PyObject_CallMethod(mRuntime->world, "set_contact_cone", "(sd)", cone.c_str(), impratio);
  const int rc = (r == nullptr) ? reportPyError("set_contact_cone") : 0;
  Py_XDECREF(r);
  PyGILState_Release(gstate);
  return rc;
}

int OmNewtonBackend::setContactCondim(int condim) {
  // Plumb the WorldInfo.newtonCondim choice to the runtime. 0 = unset, leaving
  // the model's own condim (3 on every geom of every OmniSim world measured to
  // date) -- exact current physics for every existing world. The
  // OMNISIM_NEWTON_CONDIM env var still wins (resolved runtime-side). Cached
  // regardless of build state; finalizeWorld() re-asserts onto the world it
  // actually finalizes (same multi-build race as mSolverPref).
  mCondimPref = condim;
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  if (condim <= 0)
    return 0;  // default; nothing to push
  OmLog::info(QString("[OmNewtonBackend] contact condim set to %1").arg(condim));
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *r = PyObject_CallMethod(mRuntime->world, "set_contact_condim", "(i)", condim);
  const int rc = (r == nullptr) ? reportPyError("set_contact_condim") : 0;
  Py_XDECREF(r);
  PyGILState_Release(gstate);
  return rc;
}

int OmNewtonBackend::setNoslipIterations(int iters) {
  // Plumb the WorldInfo.newtonNoslipIterations choice to the runtime. 0 =
  // unset, which is ALSO MuJoCo's own stock value, so every existing world is
  // byte-identical and there is nothing to push. The OMNISIM_NEWTON_NOSLIP env
  // var still wins and is value-parsed runtime-side, so =0 forces the pass off
  // for a world that declares the field -- the exact-revert hatch. Cached
  // regardless of build state; finalizeWorld() re-asserts onto the world it
  // actually finalizes (same multi-build race as mSolverPref).
  mNoslipItersPref = iters;
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  if (iters <= 0)
    return 0;  // default; nothing to push
  OmLog::info(QString("[OmNewtonBackend] noslip iterations set to %1").arg(iters));
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *r = PyObject_CallMethod(mRuntime->world, "set_noslip_iterations", "(i)", iters);
  const int rc = (r == nullptr) ? reportPyError("set_noslip_iterations") : 0;
  Py_XDECREF(r);
  PyGILState_Release(gstate);
  return rc;
}

int OmNewtonBackend::setClothSelfContact(int mode) {
  // Plumb WorldInfo.newtonClothSelfContact to the runtime BEFORE finalize():
  // the SolverVBD kwargs (particle_enable_self_contact + the self-contact
  // radius and margin) are assembled there, so a later write is a no-op.
  // -1 = unset, so a world that does not declare the field is byte-identical
  // to before this existed, and OMNISIM_CLOTH_SELF_CONTACT still wins.
  //
  // This was environment-ONLY until 2026-08-15, and that was a worse gap than
  // the usual "the file is incomplete": every deformable-GRASP world in this
  // tree needs the value OFF, forgetting it produced no error and no warning,
  // and the failure was a 24x slip (-22.11 mm pinch tracking on vs -0.92 mm
  // off) that reads as "cloth grasping does not work" rather than as a
  // missing launch variable. Cached like mNoslipItersPref so finalizeWorld()
  // can re-assert onto the world it actually finalizes (the multi-build race).
  mClothSelfContactPref = mode;
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  if (mode < 0)
    return 0;  // unset; nothing to push
  OmLog::info(QString("[OmNewtonBackend] cloth particle self-contact preference %1 "
                      "(WorldInfo.newtonClothSelfContact; applies only where a SolverVBD is built, "
                      "and OMNISIM_CLOTH_SELF_CONTACT still overrides)")
                .arg(mode ? "ON" : "OFF"));
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *r = PyObject_CallMethod(mRuntime->world, "set_cloth_self_contact", "(i)", mode);
  const int rc = (r == nullptr) ? reportPyError("set_cloth_self_contact") : 0;
  Py_XDECREF(r);
  PyGILState_Release(gstate);
  return rc;
}

int OmNewtonBackend::setContactSolverParams(double mu, double ke, double kd, int iters, int lsIters) {
  // Plumb WorldInfo.newtonGroundMu / newtonContactKe / newtonContactKd /
  // newtonIterations / newtonLsIterations to the runtime BEFORE finalize().
  // 0 on any of them = unset, so every existing world is byte-identical, and
  // the matching OMNISIM_NEWTON_* env var still wins (resolved runtime-side).
  //
  // These five were environment-ONLY until 2026-08-02, which meant a .wbt was
  // not a complete description of its own physics: a working friction grasp
  // could not be handed to anyone, because the file carried the scene and the
  // shell carried the contact model. Cached like mConePref so finalizeWorld()
  // can re-assert onto the world it actually finalizes (the multi-build race).
  mGroundMuPref = mu;
  mContactKePref = ke;
  mContactKdPref = kd;
  mIterationsPref = iters;
  mLsIterationsPref = lsIters;
  // ⚠ ORDER IS THE WHOLE GAME HERE. newton's ModelBuilder copies cfg.mu/ke/kd
  // into each shape AT ADD TIME, so these values only matter if they reach the
  // Python world BEFORE the ground plane and the registration loop add shapes.
  // OmSolid calls this at the HEAD of flushPendingNewtonRegistrations (world
  // not open yet -> the caching above is the useful part, rc -1 is expected)
  // and ensureWorldOpen() applies the cache right after the world is
  // constructed via applyContactSolverParamsToWorld(). The apply below serves
  // callers that plumb AFTER the world opened; for anything already added it
  // is too late by construction -- that was the measured 55-degree-ramp
  // defect: declared mu 2.0, box slid at the default anyway.
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  const int rc = applyContactSolverParamsToWorld();
  return rc;
}

int OmNewtonBackend::setConstraintBuffers(int njmax, int nconmax) {
  // Plumb the WorldInfo.newtonNjmax / newtonNconmax choice to the runtime
  // BEFORE finalize() constructs SolverMuJoCo. 0 / 0 = unset, keeping the
  // runtime's built-in 256 -- exact current physics for every existing world.
  // A positive value raises the cap (a 10-Husky fleet measures nefc=320 and
  // otherwise floods the log with kernel-side "nefc overflow" while
  // silently truncating the constraint vector); a negative value asks newton
  // for its own auto-estimate. The OMNISIM_NEWTON_NJMAX /
  // OMNISIM_NEWTON_NCONMAX env vars still win (resolved runtime-side). Cache
  // regardless of build state; finalizeWorld() re-asserts onto the world it
  // actually finalizes (same multi-build race as mSolverPref).
  mNjmaxPref = njmax;
  mNconmaxPref = nconmax;
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  if (njmax == 0 && nconmax == 0)
    return 0;  // default; nothing to push
  OmLog::info(QString::fromStdString("[OmNewtonBackend] constraint buffers set to njmax " + std::to_string(njmax) +
                                     ", nconmax " + std::to_string(nconmax) + " (0 = engine default 256)"));
  // N16: -1 ("auto") is kept -- changing what it resolves to would move the
  // physics of any world already using it -- but it is NO LONGER advertised
  // silently. newton hands the sizing to mujoco_warp's _default_njmax /
  // _default_nconmax (mujoco_warp/_src/io.py), which are computed from ONE
  // mj_forward at t=0 and collapse to a 64-row / 64-contact floor for any world
  // with no heightfield, flex or SDF geom -- i.e. every arena world in this
  // tree. On the 10-Husky fleet world that is ~64 against a measured peak of
  // nefc=320: ~80% of constraint rows dropped, versus 256 - 320 = 20% at the
  // plain default. So "auto" is systematically WORSE than leaving the field at
  // 0, and the user must be told at resolve time rather than discovering it as
  // lateral drift. The runtime overflow watch will also catch it, but only once
  // it has already started corrupting the solve; this fires at world build.
  if ((njmax < 0 || nconmax < 0) && !mRuntime->autoConstraintCapWarned) {
    mRuntime->autoConstraintCapWarned = true;
    OmLog::warning(QString::fromStdString(
      std::string("[OmNewtonBackend] WorldInfo.newtonNjmax/newtonNconmax -1 (\"auto\") requests newton's own "
                  "estimate, which is sized from the CONTACT COUNT AT t=0 ONLY and floors at 64. For any scene "
                  "that ACCUMULATES contacts (a settling fleet, a legged robot, a pile) it resolves far below "
                  "the engine default 256 and constraint rows are then dropped SILENTLY. Prefer 0 (the 256 "
                  "default) or an explicit positive value with headroom -- e.g. 512 for ten 4WD robots, whose "
                  "measured peak is nefc=320. Current request: njmax ") +
      std::to_string(njmax) + ", nconmax " + std::to_string(nconmax)));
  }
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *r = PyObject_CallMethod(mRuntime->world, "set_constraint_buffers", "(ii)", njmax, nconmax);
  const int rc = (r == nullptr) ? reportPyError("set_constraint_buffers") : 0;
  Py_XDECREF(r);
  PyGILState_Release(gstate);
  return rc;
}

int OmNewtonBackend::setNewtonSubsteps(int n) {
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

int OmNewtonBackend::setWorldGravity(double gx, double gy, double gz) {
  // Plumb WorldInfo.gravity to the runtime BEFORE finalize(). Without this the
  // builder always ran at the library default -9.81 regardless of the world
  // file (omnibench T5/T7: gravity-0 scenes fell onto the implicit ground
  // plane and read as a momentum "leak" / "spin brake").
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->openForBuild)
    return -1;
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *r = PyObject_CallMethod(mRuntime->world, "set_gravity", "(ddd)", gx, gy, gz);
  const int rc = (r == nullptr) ? reportPyError("set_gravity") : 0;
  Py_XDECREF(r);
  PyGILState_Release(gstate);
  return rc;
}

int OmNewtonBackend::step(double dt) {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->running)
    return -1;
  // Step profiling (OMNISIM_NEWTON_STEP_PROFILE=1): time the whole crossing
  // into Python and, every 500 steps, split it against what the runtime says it
  // spent inside the solver and the narrow phase. Measured on this path:
  // bare mj_step costs ~0.005 ms/step on scenes where this path costs ~1.15 ms,
  // so the question "where does the rest go" needed an answer, not a guess.
  static const bool profiling = !qEnvironmentVariableIsEmpty("OMNISIM_NEWTON_STEP_PROFILE");
  const auto tCallStart = profiling ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point();

  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *r = PyObject_CallMethod(mRuntime->world, "step", "(d)", dt);
  if (r == nullptr) {
    const int err = reportPyError("step");
    PyGILState_Release(gstate);
    return err;
  }
  Py_DECREF(r);
  // Tier 1a: the python step() just invalidated its per-step readback caches;
  // drop the C++-side packed snapshot with them so the first getter of the
  // new tick refetches post-step state.
  mRuntime->snapValid = false;
  if (profiling) {
    mNewtonProfCallSec += std::chrono::duration<double>(std::chrono::steady_clock::now() - tCallStart).count();
    if ((mRuntime->stepCount % 500) == 499) {
      double solve = 0.0, collide = 0.0;
      double pyTotal = 0.0;
      double ctrlS = 0.0, outS = 0.0, mjS = 0.0;
      { PyObject *v = PyObject_GetAttrString(mRuntime->world, "_mj_pose_msg");
        if (v != nullptr) { if (PyUnicode_Check(v)) { const char *c = PyUnicode_AsUTF8(v);
            if (c != nullptr && c[0] != '\0') OmLog::info(QString("[OmNewtonBackend] mj-pose check: ") + c); }
          Py_DECREF(v); } else PyErr_Clear(); }
      double preS = 0.0;
      { PyObject *v = PyObject_GetAttrString(mRuntime->world, "_prof_pre");
        if (v != nullptr) { const double d = PyFloat_AsDouble(v); if (!PyErr_Occurred()) preS = d; else PyErr_Clear(); Py_DECREF(v); } else PyErr_Clear(); }
      for (const char *attr : {"_prof_ctrl", "_prof_out", "_prof_mj"}) {
        PyObject *v = PyObject_GetAttrString(mRuntime->world, attr);
        if (v != nullptr) { const double d = PyFloat_AsDouble(v); if (!PyErr_Occurred()) (attr[6] == 'c' ? ctrlS : (attr[6] == 'o' ? outS : mjS)) = d; else PyErr_Clear(); Py_DECREF(v); } else PyErr_Clear();
      }
      for (const char *attr : {"_prof_solve", "_prof_collide", "_prof_py"}) {
        PyObject *v = PyObject_GetAttrString(mRuntime->world, attr);
        if (v != nullptr) {
          const double d = PyFloat_AsDouble(v);
          if (!PyErr_Occurred())
            (attr[6] == 's' ? solve : (attr[6] == 'c' ? collide : pyTotal)) = d;
          else
            PyErr_Clear();
          Py_DECREF(v);
        } else
          PyErr_Clear();
      }
      const double n = (double)(mRuntime->stepCount + 1);
      const double callMs = mNewtonProfCallSec * 1000.0 / n;
      const double solveMs = solve * 1000.0 / n, collideMs = collide * 1000.0 / n;
      const double pyMs = pyTotal * 1000.0 / n;
      OmLog::info(QString("[OmNewtonBackend] step profile over %1 steps: total %2 = solver %3 + collide %4 "
                          "+ glue %5 ms (%6%% glue); python-body %7, C++/GIL crossing %8")
                    .arg((long long)n)
                    .arg(callMs, 0, 'f', 4)
                    .arg(solveMs, 0, 'f', 4)
                    .arg(collideMs, 0, 'f', 4)
                    .arg(callMs - solveMs - collideMs, 0, 'f', 4)
                    .arg(100.0 * (callMs - solveMs - collideMs) / (callMs > 0 ? callMs : 1), 0, 'f', 1)
                    .arg(pyMs, 0, 'f', 4)
                    .arg(callMs - pyMs, 0, 'f', 4)
                  + QString(" | solver split: ctrl_in %1 + mj_step %2 + state_out %3 ms | our glue: pre %4 + post %5")
                      .arg(ctrlS * 1000.0 / n, 0, 'f', 4)
                      .arg(mjS * 1000.0 / n, 0, 'f', 4)
                      .arg(outS * 1000.0 / n, 0, 'f', 4)
                      .arg(preS * 1000.0 / n, 0, 'f', 4)
                      .arg(pyMs - solveMs - preS * 1000.0 / n, 0, 'f', 4));
    }
  }
  // N15: surface a mujoco_warp constraint-buffer overflow ONCE, in the engine
  // log -- the log run-headless parses and the Newton verdict sidecar sits next
  // to. The runtime latches a message on the world the first time the sampled
  // peak nefc/ncon passes the ALLOCATED njmax/nconmax. mujoco_warp itself only
  // warns via a wp.printf inside a warp kernel: discarded outright by the
  // GUI-subsystem omnisim-bin.exe, and sunk into an unread <log>.stdout
  // elsewhere -- so exit code, engine log and sidecar all stayed clean while
  // constraint rows were dropped. Polled on the same coarse cadence the runtime
  // samples on -- the latch is sticky, so a coarse poll only costs a few ticks
  // of latency on a once-per-world warning -- and not at all once it has fired.
  std::string overflowMsg;
  if (!mRuntime->constraintOverflowLogged && (mRuntime->stepCount % 30) == 0) {
    PyObject *om = PyObject_GetAttrString(mRuntime->world, "_constraint_overflow_msg");
    if (om != nullptr) {
      if (PyUnicode_Check(om)) {
        const char *cstr = PyUnicode_AsUTF8(om);
        if (cstr != nullptr && cstr[0] != '\0')
          overflowMsg = cstr;
      }
      Py_DECREF(om);
    } else
      PyErr_Clear();
  }
  PyGILState_Release(gstate);
  if (!overflowMsg.empty()) {
    mRuntime->constraintOverflowLogged = true;  // ONE warning per world, never per-tick spam
    OmLog::warning(QString::fromStdString("[OmNewtonBackend] " + overflowMsg));
  }
  ++mRuntime->stepCount;

  // P3.2.e numerical verification: log body 0's position at hand-picked
  // step counts so we can confirm the trajectory from the OmLog output
  // alone (no controller needed). Free-fall under g=9.81 from z=1.5
  // expected to hit the ground around step 35 at 16ms timesteps;
  // sphere should rest at z ≈ radius (0.12) thereafter.
  const long long c = mRuntime->stepCount;
  if (c == 1 || c == 30 || c == 60 || c == 120 || c == 240 || c == 480
      || c == 960 || c == 1920 || c == 3840 || c == 7680 || c == 15360
      || c == 30720 || c == 61440) {
    // NOTE: the "[OmNewtonBackend] step N" prefix is a verdict marker parsed by
    // scripts/dev/headless_runner.py (_NEWTON_STEP_RE) -- keep the format stable.
    char head[64];
    snprintf(head, sizeof(head), "[OmNewtonBackend] step %lld dt=%gs", c, dt);
    std::string line = head;
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
      if (PyArg_ParseTuple(r, "ddddddd", &bx, &by, &bz, &qx, &qy, &qz, &qw)) {
        char body[96];
        snprintf(body, sizeof(body), " b%d=(%.3f,%.3f,%.3f)", b, bx, by, bz);
        line += body;
      }
      Py_DECREF(r);
    }
    OmLog::info(QString::fromStdString(line));
  }
  return 0;
}

// Tier 1a: fetch the whole tick's readback (all body poses+velocities, all
// revolute slot angles) in ONE Python crossing and cache it C++-side. Called
// lazily by the first per-body/per-joint getter after a step(); a false
// return means "use the per-call path" (world not running, python error,
// malformed blob) -- never an error surfaced to the caller, because the
// per-call path is value-identical and still there.
static bool ensureStepSnapshot(OmNewtonRuntimeState *rt) {
  if (rt->snapValid)
    return true;
  if (rt->world == nullptr || !rt->running)
    return false;
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *r = PyObject_CallMethod(rt->world, "readback_packed", nullptr);
  bool ok = false;
  if (r == nullptr) {
    PyErr_Clear();  // fall back to the per-call readbacks
  } else if (PyBytes_Check(r)) {
    const Py_ssize_t sz = PyBytes_Size(r);
    const Py_ssize_t header = 2 * (Py_ssize_t)sizeof(long long);
    if (sz >= header) {
      const char *src = PyBytes_AsString(r);
      long long nbody = 0, nslot = 0;
      std::memcpy(&nbody, src, sizeof nbody);
      std::memcpy(&nslot, src + sizeof nbody, sizeof nslot);
      const Py_ssize_t expect =
        header + (Py_ssize_t)((nbody * 13 + nslot) * (long long)sizeof(double));
      if (nbody >= 0 && nslot >= 0 && sz == expect) {
        rt->snapBody.resize((size_t)nbody * 13);
        rt->snapSlotAngle.resize((size_t)nslot);
        const char *p = src + header;
        if (nbody > 0)
          std::memcpy(rt->snapBody.data(), p, (size_t)nbody * 13 * sizeof(double));
        if (nslot > 0)
          std::memcpy(rt->snapSlotAngle.data(), p + (size_t)nbody * 13 * sizeof(double),
                      (size_t)nslot * sizeof(double));
        rt->snapBodyCount = (int)nbody;
        rt->snapSlotCount = (int)nslot;
        rt->snapValid = true;
        ok = true;
      }
    }
  }
  Py_XDECREF(r);
  PyGILState_Release(gstate);
  return ok;
}

int OmNewtonBackend::getBodyXform(int bodyIdx, double xform[7]) const {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->running)
    return -1;
  // Serve from the per-tick snapshot when it covers this body; an index past
  // the snapshot (a body spawned after this tick's fetch) falls through to
  // the per-call read, which is value-identical (same python-side caches).
  if (ensureStepSnapshot(mRuntime) && bodyIdx >= 0 && bodyIdx < mRuntime->snapBodyCount) {
    std::memcpy(xform, &mRuntime->snapBody[(size_t)bodyIdx * 13], 7 * sizeof(double));
    return 0;
  }
  PyObject *r = PyObject_CallMethod(mRuntime->world, "body_xform", "(i)", bodyIdx);
  if (r == nullptr)
    return reportPyError("body_xform");
  double x = 0, y = 0, z = 0, qx = 0, qy = 0, qz = 0, qw = 0;
  if (!PyArg_ParseTuple(r, "ddddddd", &x, &y, &z, &qx, &qy, &qz, &qw)) {
    PyErr_Clear();
    Py_DECREF(r);
    OmLog::warning("[OmNewtonBackend] body_xform: tuple parse failed");
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

int OmNewtonBackend::getBodyVelocity(int bodyIdx, double vel[6]) const {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->running)
    return -1;
  if (ensureStepSnapshot(mRuntime) && bodyIdx >= 0 && bodyIdx < mRuntime->snapBodyCount) {
    std::memcpy(vel, &mRuntime->snapBody[(size_t)bodyIdx * 13 + 7], 6 * sizeof(double));
    return 0;
  }
  PyObject *r = PyObject_CallMethod(mRuntime->world, "body_vel", "(i)", bodyIdx);
  if (r == nullptr)
    return reportPyError("body_vel");
  double vx = 0, vy = 0, vz = 0, wx = 0, wy = 0, wz = 0;
  if (!PyArg_ParseTuple(r, "dddddd", &vx, &vy, &vz, &wx, &wy, &wz)) {
    PyErr_Clear();
    Py_DECREF(r);
    OmLog::warning("[OmNewtonBackend] body_vel: tuple parse failed");
    return -1;
  }
  Py_DECREF(r);
  vel[0] = vx; vel[1] = vy; vel[2] = vz;  // linear (world)
  vel[3] = wx; vel[4] = wy; vel[5] = wz;  // angular (world)
  return 0;
}

int OmNewtonBackend::snapshotBodyTranslations(int maxBodies, float *xyzw) const {
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->running)
    return -1;
  if (xyzw == nullptr || maxBodies <= 0)
    return -1;
  PyObject *r = PyObject_CallMethod(mRuntime->world, "body_translations_packed", "(i)", maxBodies);
  if (r == nullptr)
    return reportPyError("body_translations_packed");
  if (!PyBytes_Check(r)) {
    Py_DECREF(r);
    OmLog::warning("[OmNewtonBackend] body_translations_packed: return not bytes");
    return -1;
  }
  const Py_ssize_t n = PyBytes_Size(r);
  if (n < 0 || (n % 16) != 0) {
    Py_DECREF(r);
    OmLog::warning("[OmNewtonBackend] body_translations_packed: bad size");
    return -1;
  }
  const int count = static_cast<int>(n / 16);
  if (count > maxBodies) {
    Py_DECREF(r);
    OmLog::warning("[OmNewtonBackend] body_translations_packed: count > max");
    return -1;
  }
  if (count > 0) {
    const char *src = PyBytes_AsString(r);
    std::memcpy(xyzw, src, static_cast<size_t>(n));
  }
  Py_DECREF(r);
  return count;
}

int OmNewtonBackend::snapshotParticlePositions(int particleStart, int particleEnd, float *xyz) const {
  // Deliberately the same shape as snapshotBodyTranslations above -- packed
  // bytes out of Python, one memcpy in, ONE FFI crossing for the whole cloth.
  // Two differences, both on purpose:
  //   - the stride is 12 bytes (xyz) not 16 (xyzw), because the destination is
  //     a WrDynamicMesh vertex array, which is tightly packed;
  //   - the argument is a half-open RANGE, not a max count, because the runtime
  //     slices before it packs. With several sheets that means each one is
  //     transferred alone instead of every caller receiving all of them and
  //     discarding the part it does not own.
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->running)
    return -1;
  if (xyz == nullptr)
    return -1;
  // (-1, -1) is the runtime's "every cloth particle" sentinel; any other
  // inverted or negative range is a caller bug, not a request.
  if (!(particleStart == -1 && particleEnd == -1) && (particleStart < 0 || particleEnd <= particleStart))
    return -1;
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *r = PyObject_CallMethod(mRuntime->world, "particle_positions_packed", "(ii)",
                                    particleStart, particleEnd);
  if (r == nullptr) {
    const int err = reportPyError("particle_positions_packed");
    PyGILState_Release(gstate);
    return err;
  }
  if (!PyBytes_Check(r)) {
    Py_DECREF(r);
    PyGILState_Release(gstate);
    OmLog::warning("[OmNewtonBackend] particle_positions_packed: return not bytes");
    return -1;
  }
  const Py_ssize_t n = PyBytes_Size(r);
  if (n < 0 || (n % 12) != 0) {
    Py_DECREF(r);
    PyGILState_Release(gstate);
    OmLog::warning("[OmNewtonBackend] particle_positions_packed: bad size");
    return -1;
  }
  const int count = static_cast<int>(n / 12);
  // The runtime clamps the requested range to what exists, so a short answer is
  // possible (a cloth removed, or a range past the end). Refuse to memcpy more
  // than the caller sized its buffer for.
  if (particleStart >= 0 && count > particleEnd - particleStart) {
    Py_DECREF(r);
    PyGILState_Release(gstate);
    OmLog::warning("[OmNewtonBackend] particle_positions_packed: more particles than the range asked for");
    return -1;
  }
  if (count > 0) {
    const char *src = PyBytes_AsString(r);
    std::memcpy(xyz, src, static_cast<size_t>(n));
  }
  Py_DECREF(r);
  PyGILState_Release(gstate);
  return count;
}

int OmNewtonBackend::particleCount() const {
  // Valid in BOTH phases (unlike the readback above): OmCloth calls it right
  // after registering, while the world is still open for build, to size its
  // vertex buffers before a single step has run.
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr)
    return -1;
  PyGILState_STATE gstate = PyGILState_Ensure();
  PyObject *r = PyObject_CallMethod(mRuntime->world, "cloth_particle_count", nullptr);
  if (r == nullptr) {
    const int err = reportPyError("cloth_particle_count");
    PyGILState_Release(gstate);
    return err;
  }
  const long n = PyLong_AsLong(r);
  Py_DECREF(r);
  PyGILState_Release(gstate);
  return static_cast<int>(n);
}

// ---- Dispatcher overrides (ON path) ---------------------------------

int OmNewtonBackend::getBodyPosition(OmBodyHandle body, double pos[3]) const {
  double xform[7];
  if (getBodyXform(indexFromHandle(body), xform) != 0)
    return -1;
  pos[0] = xform[0];
  pos[1] = xform[1];
  pos[2] = xform[2];
  return 0;
}

int OmNewtonBackend::getBodyQuaternion(OmBodyHandle body, double q[4]) const {
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

int OmNewtonBackend::getBodyLinearVel(OmBodyHandle body, double v[3]) const {
  double vel[6];
  if (getBodyVelocity(indexFromHandle(body), vel) != 0)
    return -1;
  v[0] = vel[0];
  v[1] = vel[1];
  v[2] = vel[2];
  return 0;
}

int OmNewtonBackend::getBodyAngularVel(OmBodyHandle body, double v[3]) const {
  double vel[6];
  if (getBodyVelocity(indexFromHandle(body), vel) != 0)
    return -1;
  v[0] = vel[3];
  v[1] = vel[4];
  v[2] = vel[5];
  return 0;
}

int OmNewtonBackend::getBodyPointVel(OmBodyHandle body, const double point[3], double v[3]) const {
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

int OmNewtonBackend::getJointHingeAngle(OmJointHandle joint, double *angleOut) const {
  // Newton joint handles share the body-handle packing scheme: idx+1
  // packed into the void*. indexFromHandle is type-agnostic so we can
  // reuse it for the joint index too.
  if (!mAvailable || mRuntime == nullptr || mRuntime->world == nullptr || !mRuntime->running)
    return -1;
  *angleOut = getJointAngle(indexFromHandle(joint));
  return 0;
}

int OmNewtonBackend::reset() {
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

int OmNewtonBackend::beginWorld() { return -1; }
int OmNewtonBackend::ensureWorldOpen() { return -1; }
int OmNewtonBackend::applyContactSolverParamsToWorld() { return -1; }
int OmNewtonBackend::setCoordinateSystem(const std::string &) { return -1; }
int OmNewtonBackend::applyCoordinateSystemToWorld() { return -1; }
bool OmNewtonBackend::isWorldOpenForBuild() const { return false; }
bool OmNewtonBackend::isWorldRunning() const { return false; }
int OmNewtonBackend::addGroundPlane() { return -1; }
int OmNewtonBackend::addBody(double, double, double, double, double, double, double, double,
                             double, double, double, double, double, double,
                             bool, double, double, double) { return -1; }
int OmNewtonBackend::addStaticBody(double, double, double, double, double, double, double) { return -1; }
int OmNewtonBackend::addKinematicBody(double, double, double, double, double, double, double) { return -1; }
int OmNewtonBackend::setKinematicPose(int, double, double, double, double, double, double, double) { return -1; }
int OmNewtonBackend::addShapeSphere(int, double, double, double, double) { return -1; }
int OmNewtonBackend::addShapeBox(int, double, double, double, double, double, double, double,
                                 double, double, double, double) { return -1; }
int OmNewtonBackend::addShapeCylinder(int, double, double, double, double, double,
                                      double, double, double, double) { return -1; }
int OmNewtonBackend::addBodyForce(int, double, double, double, double, double, double) { return -1; }
int OmNewtonBackend::setBodyVel(int, double, double, double, int) { return -1; }
int OmNewtonBackend::getContacts(std::vector<OmNewtonContact> &out) const { out.clear(); return -1; }
int OmNewtonBackend::raycastBatch(int, const double *, OmNewtonRayHit *, const int *, int) const { return -1; }
int OmNewtonBackend::solveIk(int, int, const double *, const double *, const std::vector<int> &, const double *, int,
                             std::vector<double> &, std::vector<double> &) const { return -1; }
int OmNewtonBackend::addShapeCapsule(int, double, double, double, double, double,
                                     double, double, double, double) { return -1; }
int OmNewtonBackend::addShapePlane(int, double, double, double) { return -1; }
int OmNewtonBackend::addShapeMesh(int, const double *, int, const int *, int,
                                  double, double, double,
                                  double, double, double, double) { return -1; }
// Cloth/particle stubs. OmCloth guards every call behind a null/isAvailable()
// check, so these are unreachable in practice -- they exist for the same reason
// snapshotBodyTranslations' stub does (below): the OFF build must still link.
// -1 from addClothGrid is what makes a Cloth node inert rather than fatal on a
// NEWTON=OFF build: the node parses, warns once, and renders nothing.
int OmNewtonBackend::addClothGrid(const double *, const double *, int, int, double, double,
                                  double, double, double, double, double, double, double,
                                  int, int *) { return -1; }
// Same contract for a mesh-authored garment: -1 leaves the Cloth node inert.
int OmNewtonBackend::addClothMesh(const double *, const double *, const double *, int,
                                  const int *, int, double, double,
                                  double, double, double, double, double,
                                  double, double, int *) { return -1; }
int OmNewtonBackend::snapshotParticlePositions(int, int, float *) const { return -1; }
int OmNewtonBackend::particleCount() const { return -1; }
// Same contract for the volumetric soft body: -1 makes an OmSoftBody node inert
// (parses, warns once, renders nothing) rather than fatal on a NEWTON=OFF build.
int OmNewtonBackend::addSoftGrid(const double *, const double *, int, int, int,
                                 double, double, double, double, double, double, double,
                                 double, int, int *) { return -1; }
int OmNewtonBackend::softSurfaceTriangles(int, int *, int) const { return -1; }
int OmNewtonBackend::addJointRevolute(int, int, double, double, double,
                                      double, double, double,
                                      double, double, double,
                                      double, double,
                                      double, double,
                                      double, double,
                                      double, double, double, double) { return -1; }
int OmNewtonBackend::addJointHinge2(int, int, double, double, double,
                                    double, double, double,
                                    double, double, double,
                                    double, double, double) { return -1; }
int OmNewtonBackend::addJointBall(int, int, double, double, double,
                                  double, double, double) { return -1; }
int OmNewtonBackend::addJointFixed(int, int) { return -1; }
int OmNewtonBackend::addWeldSlot(int) { return -1; }
int OmNewtonBackend::weldEngage(int, int, int) { return -1; }
int OmNewtonBackend::weldRelease(int) { return -1; }
int OmNewtonBackend::weldForce(int, double[6]) const { return -1; }
int OmNewtonBackend::touchForce(int, double[6]) const { return -1; }
int OmNewtonBackend::addJointPrismatic(int, int, double, double, double,
                                       double, double, double,
                                       double, double, double,
                                       double, double,
                                       double, double,
                                       double, double) { return -1; }
int OmNewtonBackend::setJointTargetVelocity(int, double) { return -1; }
int OmNewtonBackend::setJointTargetPosition(int, double) { return -1; }
int OmNewtonBackend::setJointForce(int, double) { return -1; }
double OmNewtonBackend::getJointAngle(int) const { return 0.0; }
void OmNewtonBackend::resetBodyPose(int, double, double, double, double, double, double, double) {}
void OmNewtonBackend::resetJointsToDefaults() {}
std::string OmNewtonBackend::diagDumpJointQ() const { return "(stub)"; }
int OmNewtonBackend::finalizeWorld() { return -1; }
int OmNewtonBackend::setSolverPreference(const std::string &) { return -1; }
int OmNewtonBackend::setClothCoupling(int, int) { return -1; }
int OmNewtonBackend::setNewtonSubsteps(int) { return -1; }
int OmNewtonBackend::setContactCone(const std::string &, double) { return -1; }
int OmNewtonBackend::setContactCondim(int) { return -1; }
int OmNewtonBackend::setNoslipIterations(int) { return -1; }
int OmNewtonBackend::setClothSelfContact(int) { return -1; }
int OmNewtonBackend::setConstraintBuffers(int, int) { return -1; }
int OmNewtonBackend::setContactSolverParams(double, double, double, int, int) { return -1; }
int OmNewtonBackend::setWorldGravity(double, double, double) { return -1; }
int OmNewtonBackend::step(double) { return -1; }
int OmNewtonBackend::getBodyXform(int, double[7]) const { return -1; }
int OmNewtonBackend::getBodyVelocity(int, double[6]) const { return -1; }
// snapshotBodyTranslations is called UNCONDITIONALLY by OmCamera.cpp + main.cpp
// (runtime `nb ? ... : -1` checks, no #ifdef guard), so the NEWTON=OFF build
// needs this stub or the pure-legacy link fails. Its absence silently broke
// `make OMNISIM_WITH_NEWTON=OFF` — the reversibility escape hatch the Stage 3
// default-flip depends on (default-flip-plan.md §4.4).
int OmNewtonBackend::snapshotBodyTranslations(int, float *) const { return -1; }

// Dispatcher overrides -- OFF path stubs. Always return -1. Callers must treat
// that as "not available", NOT as "try the other engine": there isn't one.
int OmNewtonBackend::getBodyPosition(OmBodyHandle, double[3]) const { return -1; }
int OmNewtonBackend::getBodyQuaternion(OmBodyHandle, double[4]) const { return -1; }
int OmNewtonBackend::getBodyLinearVel(OmBodyHandle, double[3]) const { return -1; }
int OmNewtonBackend::getBodyAngularVel(OmBodyHandle, double[3]) const { return -1; }
int OmNewtonBackend::getBodyPointVel(OmBodyHandle, const double[3], double[3]) const { return -1; }
int OmNewtonBackend::getJointHingeAngle(OmJointHandle, double *) const { return -1; }
int OmNewtonBackend::reset() { return 0; }
void OmNewtonBackend::teardownWorld() {}

#endif  // OMNISIM_WITH_NEWTON
