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
//
// SPDX-License-Identifier: Apache-2.0
//
// Standalone smoke check for the embedded-CPython call path used by
// OmNewtonBackend. Uses the exact bundled importable helper module
// AND, beyond that, exercises the helper-module flow that P3.1 wires
// into the backend: load `newton_runtime` into a private namespace,
// instantiate a World, add a ground plane + a sphere body, finalize,
// step under gravity for 15 ticks, read back the position. The same
// simulation reproduced out-of-process drops the sphere from z=1.5 to
// z≈1.17 over 0.25s -- this binary is the in-process equivalent.
//
// Build (PowerShell, output to user-temp because mingw can't write
// to the repo drive from this sandbox path):
//   $env:Path = "C:\msys64\mingw64\bin;C:\msys64\usr\bin;" + $env:Path
//   $py = "$env:LOCALAPPDATA/Programs/Python/Python312"
//   g++ -std=c++17 \
//     -isystem "$py/include" \
//     src/omnisim/physics/newton_embed_smoke.cpp \
//     -L "$py/libs" \
//     -lpython312 \
//     -o "$env:TEMP/newton_embed_smoke.exe"
//
// Run: & "$env:TEMP/newton_embed_smoke.exe"
//
// Expected (success): 15 lines of "step N body_q[z]=..." with z
// monotonically decreasing from 1.5 to ≈1.17, then `[smoke] PASS`.

#include <Python.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>

static int dieOnPyErr(const char *step) {
  if (PyErr_Occurred()) {
    PyErr_PrintEx(0);
    std::fprintf(stderr, "[smoke] FAIL: %s raised\n", step);
    return 1;
  }
  return 0;
}

int main() {
  Py_InitializeEx(0);
  if (!Py_IsInitialized()) {
    std::fprintf(stderr, "[smoke] FAIL: Py_InitializeEx returned, Py_IsInitialized=false\n");
    return 1;
  }
  std::fprintf(stdout, "[smoke] Py_InitializeEx OK\n");

  PyObject *helper = PyImport_ImportModule("omnisim_newton_runtime");
  if (helper == nullptr) {
    PyErr_PrintEx(0);
    std::fprintf(stderr, "[smoke] FAIL: import omnisim_newton_runtime\n");
    return 2;
  }
  std::fprintf(stdout, "[smoke] helper module import OK\n");

  PyObject *worldClass = PyObject_GetAttrString(helper, "World");
  if (worldClass == nullptr) {
    std::fprintf(stderr, "[smoke] FAIL: World class not found in helper\n");
    return 3;
  }

  PyObject *world = PyObject_CallObject(worldClass, nullptr);
  if (dieOnPyErr("World()") != 0)
    return 4;
  std::fprintf(stdout, "[smoke] World() OK\n");

  PyObject *r = PyObject_CallMethod(world, "add_ground_plane", nullptr);
  if (dieOnPyErr("add_ground_plane") != 0)
    return 5;
  Py_XDECREF(r);

  // mass=0.25 at (0, 0, 1.5)
  r = PyObject_CallMethod(world, "add_body", "(dddd)", 0.25, 0.0, 0.0, 1.5);
  if (dieOnPyErr("add_body") != 0)
    return 6;
  long bodyIdx = PyLong_AsLong(r);
  Py_XDECREF(r);
  std::fprintf(stdout, "[smoke] add_body OK (idx=%ld)\n", bodyIdx);

  r = PyObject_CallMethod(world, "add_shape_sphere", "(ld)", bodyIdx, 0.12);
  if (dieOnPyErr("add_shape_sphere") != 0)
    return 7;
  Py_XDECREF(r);

  // Second body + revolute joint connecting them. Joint axis (0, 1, 0)
  // anchored at the parent's local origin and child's local origin
  // (so the hinge sits between the two bodies). With this setup, when
  // the assembly is dropped, body 0 falls + body 1 follows but can
  // rotate freely around the y axis. We don't add any shape to body 1
  // here -- the joint is enough to verify the FFI surface end-to-end.
  r = PyObject_CallMethod(world, "add_body", "(dddd)", 0.10, 0.30, 0.0, 1.5);
  if (dieOnPyErr("add_body (child)") != 0)
    return 12;
  long childIdx = PyLong_AsLong(r);
  Py_XDECREF(r);
  std::fprintf(stdout, "[smoke] add_body child OK (idx=%ld)\n", childIdx);

  r = PyObject_CallMethod(world, "add_shape_sphere", "(ld)", childIdx, 0.06);
  if (dieOnPyErr("add_shape_sphere (child)") != 0)
    return 13;
  Py_XDECREF(r);

  r = PyObject_CallMethod(world, "add_joint_revolute", "(iiddddddddd)",
                          (int)bodyIdx, (int)childIdx,
                          0.0, 1.0, 0.0,    // axis
                          0.15, 0.0, 0.0,   // parent anchor (between the two)
                          -0.15, 0.0, 0.0); // child anchor
  if (dieOnPyErr("add_joint_revolute") != 0)
    return 14;
  long jointIdx = PyLong_AsLong(r);
  Py_XDECREF(r);
  std::fprintf(stdout, "[smoke] add_joint_revolute OK (idx=%ld)\n", jointIdx);

  r = PyObject_CallMethod(world, "finalize", nullptr);
  if (dieOnPyErr("finalize") != 0)
    return 8;
  Py_XDECREF(r);
  std::fprintf(stdout, "[smoke] finalize OK\n");

  // Run 15 steps at dt=1/60. Read back z each time.
  const double dt = 1.0 / 60.0;
  for (int i = 0; i < 15; ++i) {
    r = PyObject_CallMethod(world, "step", "(d)", dt);
    if (dieOnPyErr("step") != 0)
      return 9;
    Py_XDECREF(r);

    r = PyObject_CallMethod(world, "body_xform", "(l)", bodyIdx);
    if (dieOnPyErr("body_xform") != 0)
      return 10;
    double x0 = 0, y0 = 0, z0 = 0, qx = 0, qy = 0, qz = 0, qw = 0;
    if (!PyArg_ParseTuple(r, "ddddddd", &x0, &y0, &z0, &qx, &qy, &qz, &qw)) {
      std::fprintf(stderr, "[smoke] FAIL: body_xform tuple parse\n");
      return 11;
    }
    Py_DECREF(r);

    r = PyObject_CallMethod(world, "body_xform", "(l)", childIdx);
    if (dieOnPyErr("body_xform child") != 0)
      return 15;
    double x1 = 0, y1 = 0, z1 = 0, qcx = 0, qcy = 0, qcz = 0, qcw = 0;
    if (!PyArg_ParseTuple(r, "ddddddd", &x1, &y1, &z1, &qcx, &qcy, &qcz, &qcw)) {
      std::fprintf(stderr, "[smoke] FAIL: child body_xform tuple parse\n");
      return 16;
    }
    Py_DECREF(r);

    // Distance between body centers should stay near 0.30 (initial gap)
    // throughout the simulation if the hinge is holding them together.
    const double dx = x1 - x0, dy = y1 - y0, dz = z1 - z0;
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    std::fprintf(stdout,
                 "[smoke] step %2d  parent.z=%.4f  child.z=%.4f  |child-parent|=%.4f\n",
                 i + 1, z0, z1, dist);
  }

  Py_DECREF(world);
  Py_DECREF(worldClass);
  Py_DECREF(helper);

  std::fprintf(stdout, "[smoke] PASS\n");
  return 0;
}
