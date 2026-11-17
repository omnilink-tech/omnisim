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

"""Engine-free pin: both Assimp node walks map NORMALS with the normal matrix.

OmMesh and OmCadShape mapped every normal with the node's full 4x4 transform
until 2026-10-23 -- translation included, and no inverse-transpose -- so a mesh
on a translated node (every Collada geometry instanced more than once keeps its
node) shaded with normals bent toward the node's offset, and a non-uniform node
scale tilted them off the surface. Vertices were already right (the full 4x4,
root-first since 2026-10-15). Colliders never read normals, so only shading moved.

Both walks now map normals with omAssimpNormalMatrix() (the inverse-transpose of
the upper-left 3x3, as sign(det) * cofactor) and renormalise through
omAssimpTransformNormal(), both inline in OmAssimpIoSystem.hpp;
OMNISIM_MESH_NORMAL_MATRIX_FIX=0 restores the 4x4 product.

The second test compiles a small driver against that header and the vendored
Assimp headers and checks the math itself: a translation leaves a normal alone,
a rotation rotates it, a non-uniform scale or a shear keeps it perpendicular to
the transformed surface, a mirror keeps it outward, and a zero scale yields no
NaN. It skips when no C++ compiler or no Assimp headers are found.

    python -m pytest tests/test_mesh_normal_matrix.py -v
"""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
NODES = REPO / "src" / "omnisim" / "nodes"


def test_both_assimp_node_walks_map_normals_with_the_normal_matrix():
    """Engine-free: neither walk applies the 4x4 to a normal unless the hatch is set."""
    header = (NODES / "OmAssimpIoSystem.hpp").read_text(encoding="utf-8")
    assert "inline aiMatrix3x3 omAssimpNormalMatrix(const aiMatrix4x4 &transform)" in header
    assert "inline aiVector3D omAssimpTransformNormal(" in header
    helper = (NODES / "OmAssimpIoSystem.cpp").read_text(encoding="utf-8")
    assert "OMNISIM_MESH_NORMAL_MATRIX_FIX" in helper
    for name in ("OmMesh.cpp", "OmCadShape.cpp"):
        src = (NODES / name).read_text(encoding="utf-8")
        assert "omAssimpNormalMatrix(transform)" in src, name + " must build the normal matrix per node"
        assert "omAssimpTransformNormal(normalMatrix, mesh->mNormals[j])" in src, name
        # the 4x4 product survives only as the hatch's else-branch
        assert src.count("transform * mesh->mNormals[j]") == 1, name
        assert "normalMatrixFix ? omAssimpTransformNormal" in src, name
        # both walks must visit every node; OmCadShape read only the root's meshes until 2026-10-23
        assert "queue.push_back(node->mChildren[i])" in src, name + " must queue each node's children"


def _compiler():
    for name in ("g++", "c++", "clang++"):
        path = shutil.which(name)
        if path:
            return path
    msys = Path("C:/msys64/mingw64/bin/g++.exe")
    return str(msys) if msys.is_file() else None


def _assimp_include():
    """The directory that holds assimp/ (the engine builds with -I<repo>/dependencies)."""
    roots = [REPO]
    home = os.environ.get("OMNISIM_HOME")
    if home:
        roots.append(Path(home))
    for root in roots:
        for inc in (root / "dependencies", root / "include" / "libassimp" / "include"):
            if (inc / "assimp" / "matrix3x3.h").is_file() and (inc / "assimp" / "IOSystem.hpp").is_file():
                return inc
    return None


_DRIVER = r"""
#include "OmAssimpIoSystem.hpp"

#include <cmath>
#include <cstdio>
#include <string>

// IOSystem.hpp's inline constructor emits its vtable here; these two out-of-line
// virtuals live in libassimp, which the driver does not link.
bool Assimp::IOSystem::ComparePaths(const char *, const char *) const {
  return false;
}
const std::string &Assimp::IOSystem::CurrentDirectory() const {
  static const std::string none;
  return none;
}

static int failures = 0;

static void expect(bool ok, const char *what) {
  if (!ok) {
    std::printf("FAIL %s\n", what);
    ++failures;
  }
}

static bool near(const aiVector3D &a, const aiVector3D &b) {
  return std::fabs(a.x - b.x) < 1e-5 && std::fabs(a.y - b.y) < 1e-5 && std::fabs(a.z - b.z) < 1e-5;
}

static aiVector3D cross(const aiVector3D &a, const aiVector3D &b) {
  return aiVector3D(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

static aiVector3D mapNormal(const aiMatrix4x4 &m, const aiVector3D &n) {
  return omAssimpTransformNormal(omAssimpNormalMatrix(m), n);
}

// The surface through tangents t1, t2 with normal t1 x t2 must map to the surface
// through m*t1, m*t2: the mapped normal is unit, perpendicular to both, and on the
// side the mapped normal direction (m applied to the normal as a displacement) is.
static void expectSurfaceNormal(const aiMatrix4x4 &m, const aiVector3D &t1, const aiVector3D &t2,
                                const char *what) {
  const aiVector3D n = cross(t1, t2).Normalize();
  const aiVector3D got = mapNormal(m, n);
  const aiMatrix3x3 r(m);
  const aiVector3D mt1 = r * t1, mt2 = r * t2, side = r * n;
  expect(std::fabs(got.Length() - 1) < 1e-5, what);
  expect(std::fabs(got * mt1) < 1e-5 * mt1.Length(), what);
  expect(std::fabs(got * mt2) < 1e-5 * mt2.Length(), what);
  expect(got * side > 0, what);
}

int main() {
  const aiVector3D x(1, 0, 0), y(0, 1, 0), z(0, 0, 1);

  // a translation alone leaves every normal as it is (the 4x4 product added it)
  aiMatrix4x4 t;
  aiMatrix4x4::Translation(aiVector3D(0, 0.15f, 25), t);
  expect(near(mapNormal(t, z), z), "translation leaves +Z");
  expect(near(mapNormal(t, x), x), "translation leaves +X");

  // rotation + translation: the normal turns with the rotation only
  aiMatrix4x4 rot, rt;
  aiMatrix4x4::RotationZ(static_cast<ai_real>(M_PI / 2), rot);
  aiMatrix4x4::Translation(aiVector3D(3, -4, 5), rt);
  expect(near(mapNormal(rt * rot, x), y), "Rz(90) maps +X to +Y");
  // the Collada up-axis fix on the root: Rx(90)
  const aiMatrix4x4 upAxis(1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1);
  expect(near(mapNormal(upAxis, y), z), "the up-axis fix maps +Y to +Z");

  // a non-uniform scale: the 45-degree face of a box squashed 4x along y
  aiMatrix4x4 s;
  aiMatrix4x4::Scaling(aiVector3D(1, 4, 1), s);
  expectSurfaceNormal(s, aiVector3D(1, -1, 0), z, "non-uniform scale");
  expect(near(mapNormal(s, aiVector3D(1, 1, 0).Normalize()), aiVector3D(4, 1, 0).Normalize()),
         "non-uniform scale: (1,1,0) -> (4,1,0)/|.|");

  // a general affine node transform (rotation, shear, scale, translation)
  const aiMatrix4x4 g(1.2f, 0.3f, -0.4f, 7, 0.1f, 0.8f, 0.5f, -2, -0.2f, 0.6f, 2.0f, 0.5f, 0, 0, 0, 1);
  expectSurfaceNormal(g, x, y, "affine, z face");
  expectSurfaceNormal(g, y, z, "affine, x face");
  expectSurfaceNormal(g, aiVector3D(1, 2, 3), aiVector3D(-2, 0.5f, 1), "affine, oblique face");

  // a mirror (det < 0): an outward normal stays outward
  aiMatrix4x4 mirror;
  aiMatrix4x4::Scaling(aiVector3D(-1, 1, 1), mirror);
  expect(near(mapNormal(mirror, x), aiVector3D(-1, 0, 0)), "mirror x keeps +X outward (-> -X)");
  expectSurfaceNormal(mirror * g, aiVector3D(1, 2, 3), aiVector3D(-2, 0.5f, 1), "mirrored affine");

  // a zero scale is degenerate: no NaN reaches the renderer
  aiMatrix4x4 flat;
  aiMatrix4x4::Scaling(aiVector3D(1, 1, 0), flat);
  const aiVector3D f = mapNormal(flat, aiVector3D(1, 1, 1).Normalize());
  expect(f.x == f.x && f.y == f.y && f.z == f.z, "zero scale yields no NaN");

  if (failures == 0)
    std::printf("OK\n");
  return failures == 0 ? 0 : 1;
}
"""


def test_normal_matrix_math(tmp_path):
    """Compile the real header and check the inverse-transpose against surface geometry."""
    cxx = _compiler()
    if cxx is None:
        pytest.skip("no C++ compiler on PATH")
    inc = _assimp_include()
    if inc is None:
        pytest.skip("no Assimp headers (dependencies/assimp is gitignored; build setup provides it)")
    src = tmp_path / "normal_matrix_driver.cpp"
    src.write_text(_DRIVER, encoding="utf-8")
    exe = tmp_path / ("normal_matrix_driver" + (".exe" if os.name == "nt" else ""))
    cmd = [cxx, "-std=c++17", "-O1", "-D_USE_MATH_DEFINES", "-I" + str(NODES), "-I" + str(inc),
           str(src), "-o", str(exe)]
    if os.name == "nt":
        cmd.append("-static")  # no MinGW runtime DLLs needed on PATH to run it
    env = dict(os.environ)
    bindir = str(Path(cxx).parent)
    env["PATH"] = bindir + os.pathsep + env.get("PATH", "")  # cc1plus / as / ld next to the driver
    built = subprocess.run(cmd, capture_output=True, text=True, env=env, timeout=180)
    assert built.returncode == 0, "the driver did not compile:\n" + built.stderr[-3000:]
    ran = subprocess.run([str(exe)], capture_output=True, text=True, timeout=60)
    assert ran.returncode == 0 and "OK" in ran.stdout, ran.stdout + ran.stderr
