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

#include "OmCloth.hpp"

#include "OmAppearance.hpp"
#include "OmLog.hpp"
#include "OmMFString.hpp"
#include "OmMatrix3.hpp"
#include "OmPbrAppearance.hpp"
#include "OmSFNode.hpp"
// OmNewtonBackend.hpp also pulls in OmPhysicsBackend.hpp, which is where the
// OmPhysicsBackendRegistry namespace lives -- there is no separate registry
// header, and OmSolid.cpp gets it the same way.
#include "OmNewtonBackend.hpp"
#include "OmQuaternion.hpp"
#include "OmRotation.hpp"
#include "OmSFBool.hpp"
#include "OmSFColor.hpp"
#include "OmSFDouble.hpp"
#include "OmSFInt.hpp"
#include "OmSFRotation.hpp"
#include "OmSFVector3.hpp"
#include "OmSimulationWorld.hpp"
#include "OmSolid.hpp"
#include "OmUrl.hpp"
#include "OmVector3.hpp"
#include "OmDeformableFrameListener.hpp"

#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>

// Qt-include ratchet exception (2026-09-02): QFileInfo is used as a COMPLETE type below
// (exists() / fileName() on the mesh path) and no header this file already includes supplies it.
#include <QtCore/QFileInfo>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>
#include <vector>

void OmCloth::init() {
  mTranslation = findSFVector3("translation");
  mRotation = findSFRotation("rotation");
  mUrl = findMFString("url");
  mDensity = findSFDouble("density");
  mMeshScale = findSFDouble("meshScale");
  mPinTopBand = findSFDouble("pinTopBand");
  mDimX = findSFInt("dimX");
  mDimY = findSFInt("dimY");
  mCellX = findSFDouble("cellX");
  mCellY = findSFDouble("cellY");
  mMass = findSFDouble("mass");
  mParticleRadius = findSFDouble("particleRadius");
  mTriKe = findSFDouble("triKe");
  mTriKa = findSFDouble("triKa");
  mTriKd = findSFDouble("triKd");
  mEdgeKe = findSFDouble("edgeKe");
  mFixLeft = findSFBool("fixLeft");
  mFixRight = findSFBool("fixRight");
  mFixBottom = findSFBool("fixBottom");
  mFixTop = findSFBool("fixTop");
  mDiffuseColor = findSFColor("diffuseColor");
  mAppearance = findSFNode("appearance");
  mCastShadows = findSFBool("castShadows");

  mParticleStart = -1;
  mParticleEnd = -1;
  mRegistrationDone = false;
  mMeshVertices.clear();
  mMeshIndices.clear();
  mMeshUvs.clear();
  mTexCoords.clear();
  mWorldBoundsDirty = true;
  mWorldBoundsValid = false;
  for (int c = 0; c < 3; ++c) {
    mWorldBoundsMin[c] = 0.0;
    mWorldBoundsMax[c] = 0.0;
  }
}

OmAppearance *OmCloth::appearance() const {
  return mAppearance ? dynamic_cast<OmAppearance *>(mAppearance->value()) : NULL;
}

OmPbrAppearance *OmCloth::pbrAppearance() const {
  return mAppearance ? dynamic_cast<OmPbrAppearance *>(mAppearance->value()) : NULL;
}

OmAbstractAppearance *OmCloth::abstractAppearance() const {
  return mAppearance ? dynamic_cast<OmAbstractAppearance *>(mAppearance->value()) : NULL;
}

OmCloth::OmCloth(OmTokenizer *tokenizer) : OmBaseNode("Cloth", tokenizer) {
  init();
}

OmCloth::OmCloth(const OmCloth &other) : OmBaseNode(other) {
  init();
}

OmCloth::OmCloth(const OmNode &other) : OmBaseNode(other) {
  init();
}

OmCloth::~OmCloth() {
  OmDeformableFrameListener::instance()->unsubscribeCloth(this);
}

int OmCloth::dimX() const {
  return mDimX ? mDimX->value() : 0;
}

int OmCloth::dimY() const {
  return mDimY ? mDimY->value() : 0;
}

double OmCloth::cellX() const {
  return mCellX ? mCellX->value() : 0.0;
}

double OmCloth::cellY() const {
  return mCellY ? mCellY->value() : 0.0;
}

double OmCloth::mass() const {
  return mMass ? mMass->value() : 0.0;
}

int OmCloth::particleCount() const {
  // A garment is one particle per mesh vertex, and the mesh has already been
  // cleaned, so this is the count the solver will report back.
  if (isMeshCloth())
    return static_cast<int>(mMeshVertices.size() / 3);
  // newton's add_cloth_grid counts CELLS, so the particle grid is one larger in
  // each direction. Getting this wrong is the classic off-by-one that leaves
  // the last row of triangles indexing past the end of the readback.
  const int nx = dimX();
  const int ny = dimY();
  if (nx <= 0 || ny <= 0)
    return 0;
  return (nx + 1) * (ny + 1);
}

int OmCloth::fixFlags() const {
  int flags = OmClothFixNone;
  if (mFixLeft && mFixLeft->value())
    flags |= OmClothFixLeft;
  if (mFixRight && mFixRight->value())
    flags |= OmClothFixRight;
  if (mFixBottom && mFixBottom->value())
    flags |= OmClothFixBottom;
  if (mFixTop && mFixTop->value())
    flags |= OmClothFixTop;
  return flags;
}

namespace {
  // Exact-equality key for the weld below. EXACT, not epsilon-tolerant, and
  // that is a compatibility requirement rather than a shortcut: the runtime
  // welds with numpy's exact np.unique, and the two passes must agree or the
  // particle count check in onPhysicsStepStarted() fires and the garment goes
  // inert. An epsilon weld here would silently merge vertices numpy would keep.
  struct OmClothVertexKey {
    double x, y, z;
    bool operator<(const OmClothVertexKey &o) const {
      if (x != o.x)
        return x < o.x;
      if (y != o.y)
        return y < o.y;
      return z < o.z;
    }
  };

  // -0.0 and 0.0 are DIFFERENT bit patterns that compare EQUAL. numpy welds
  // them; a bitwise map would not. Normalising here keeps the two sides in
  // agreement -- `v == 0.0` is true for -0.0, so this replaces it with +0.0.
  inline double omClothNormalizeZero(double v) {
    return (v == 0.0) ? 0.0 : v;
  }
}  // namespace

bool OmCloth::loadMeshFromUrl() {
  mMeshVertices.clear();
  mMeshIndices.clear();
  mMeshUvs.clear();
  if (mUrl == NULL || mUrl->size() == 0 || mUrl->item(0).isEmpty())
    return false;

  const QString path = OmUrl::computePath(this, "url", mUrl, 0, true);
  if (path.isEmpty() || !QFileInfo(path).exists()) {
    // computePath has already warned about a missing file; say what it COSTS,
    // because an unresolved url leaves a Cloth that parses and does nothing.
    parsingWarn(tr("'url' could not be resolved to a readable file, so this Cloth has no geometry and will "
                   "neither render nor simulate."));
    return false;
  }

  // ⚠ THIS IS DELIBERATELY NOT OmMesh's IMPORT CONFIGURATION, and the difference
  // is the point rather than an oversight.
  //
  // OmMesh asks for aiProcess_GenSmoothNormals because a rigid mesh is shaded
  // from stored normals. For cloth that flag is actively harmful: normals are a
  // per-vertex attribute, aiProcess_JoinIdenticalVertices only merges vertices
  // whose attributes ALL match, and a garment's seam vertices differ in normal.
  // Generating normals first therefore PREVENTS the weld -- and an unwelded
  // seam is newton trap #2, a garment with no stretch and no bending across it
  // that splits open under gravity. So we strip normals, tangents and colours
  // (RemoveComponent + the RVC mask). Cloth normals are recomputed each frame
  // from the simulated positions in uploadPositions() anyway, so nothing is
  // lost.
  //
  // ⚠ TEXTURE COORDINATES ARE THE ONE EXCEPTION AND THEY ARE KEPT. They used to
  // be stripped here too, for the same reason as normals -- and that was the
  // wrong trade the moment `appearance` gave a Cloth a real texture, because a
  // garment's parameterisation is authored, not derivable. Keeping them does NOT
  // reintroduce the unwelded-seam trap: this function does its own weld below,
  // on POSITION ALONE, so whatever assimp declines to merge is merged here
  // anyway. What survives is a UV per PARTICLE (the first one seen at that
  // position), which costs a texel of smear along each seam instead of a tear.
  //
  // ⚠ On a file with no UV set -- the shipped tshirt.obj is one -- keeping the
  // component in the mask is a NO-OP: there is nothing to remove, assimp's weld
  // sees the same attributes it saw before, and the vertex ORDER (which IS the
  // particle order, and therefore the physics) is unchanged.
  //
  // PreTransformVertices replaces OmMesh's manual node-tree walk: it bakes each
  // node's transform into its vertices and flattens the hierarchy, which is all
  // this node wants from the scene graph.
  Assimp::Importer importer;
  importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS,
                              aiComponent_NORMALS | aiComponent_TANGENTS_AND_BITANGENTS |
                                aiComponent_COLORS | aiComponent_BONEWEIGHTS |
                                aiComponent_ANIMATIONS | aiComponent_TEXTURES | aiComponent_LIGHTS |
                                aiComponent_CAMERAS);
  const aiScene *const scene =
    importer.ReadFile(path.toUtf8().constData(), aiProcess_ValidateDataStructure | aiProcess_Triangulate |
                                                   aiProcess_JoinIdenticalVertices |
                                                   aiProcess_PreTransformVertices | aiProcess_RemoveComponent);
  if (scene == NULL || scene->mRootNode == NULL || scene->mNumMeshes == 0) {
    parsingWarn(tr("'url' file '%1' could not be read as a mesh: %2")
                  .arg(QFileInfo(path).fileName())
                  .arg(importer.GetErrorString()));
    return false;
  }

  // Concatenate every mesh in the file. A garment exported per-material arrives
  // as several meshes that are one piece of fabric; keeping only the first would
  // silently drop the sleeves.
  std::vector<double> verts;
  std::vector<double> uvs;
  std::vector<int> tris;
  // A garment exported per material can arrive as several meshes, and only some
  // of them need carry a UV channel. Half a UV set is worse than none -- the
  // untextured pieces would sample texel (0, 0) and read as a flat colour patch
  // -- so the whole asset is treated as unparameterised unless EVERY piece is.
  bool hasUvs = true;
  for (unsigned int m = 0; m < scene->mNumMeshes; ++m) {
    const aiMesh *const mesh = scene->mMeshes[m];
    if (mesh == NULL || mesh->mNumVertices == 0 || mesh->mNumFaces == 0)
      continue;
    if (!mesh->HasTextureCoords(0))
      hasUvs = false;
  }
  for (unsigned int m = 0; m < scene->mNumMeshes; ++m) {
    const aiMesh *const mesh = scene->mMeshes[m];
    if (mesh == NULL || mesh->mNumVertices == 0 || mesh->mNumFaces == 0)
      continue;
    const int base = static_cast<int>(verts.size() / 3);
    for (unsigned int v = 0; v < mesh->mNumVertices; ++v) {
      verts.push_back(omClothNormalizeZero(static_cast<double>(mesh->mVertices[v].x)));
      verts.push_back(omClothNormalizeZero(static_cast<double>(mesh->mVertices[v].y)));
      verts.push_back(omClothNormalizeZero(static_cast<double>(mesh->mVertices[v].z)));
      if (hasUvs) {
        uvs.push_back(static_cast<double>(mesh->mTextureCoords[0][v].x));
        uvs.push_back(static_cast<double>(mesh->mTextureCoords[0][v].y));
      }
    }
    for (unsigned int f = 0; f < mesh->mNumFaces; ++f) {
      const aiFace &face = mesh->mFaces[f];
      if (face.mNumIndices != 3)
        continue;  // aiProcess_Triangulate should have made this impossible
      for (int k = 0; k < 3; ++k)
        tris.push_back(base + static_cast<int>(face.mIndices[k]));
    }
  }

  const int rawVerts = static_cast<int>(verts.size() / 3);
  const int rawTris = static_cast<int>(tris.size() / 3);
  if (rawVerts == 0 || rawTris == 0) {
    parsingWarn(tr("'url' file '%1' contains no triangles.").arg(QFileInfo(path).fileName()));
    return false;
  }

  // Every vertex must be finite before anything indexes through it: one NaN
  // poisons the whole VBD solve, and the resulting all-NaN sheet reports as
  // "cloth vanished" with nothing naming the asset.
  for (std::size_t i = 0; i < verts.size(); ++i) {
    if (!std::isfinite(verts[i])) {
      parsingWarn(tr("'url' file '%1' contains a non-finite vertex coordinate; the mesh is rejected because a "
                     "single NaN propagates through the whole cloth solve.")
                    .arg(QFileInfo(path).fileName()));
      return false;
    }
  }

  // ---- CLEAN. Mirrors World._clean_cloth_mesh in the runtime step for step,
  // and MUST stay order-preserving + an identity on clean input: the particle
  // at index i has to stay mesh vertex i, or the drawn surface indexes the
  // right COUNT of wrong particles and the garment renders as noise with no
  // error anywhere. See the header note on mMeshVertices.
  //
  // It runs here as well as in the runtime because each side needs it for a
  // different reason -- this one owns the vertex order for the vertex buffer,
  // that one protects any other caller of add_cloth_mesh -- and because all
  // three defects are silent in newton and damaging in the world.
  int welded = 0, degenerate = 0, duplicateTris = 0, orphans = 0, outOfRange = 0;

  // (1) Weld duplicates, first-occurrence order.
  std::map<OmClothVertexKey, int> seen;
  std::vector<int> remap(rawVerts, -1);
  std::vector<double> weldedVerts;
  std::vector<double> weldedUvs;
  weldedVerts.reserve(verts.size());
  if (hasUvs)
    weldedUvs.reserve(uvs.size());
  int uvSeamCollapses = 0;
  for (int v = 0; v < rawVerts; ++v) {
    const OmClothVertexKey key = {verts[v * 3], verts[v * 3 + 1], verts[v * 3 + 2]};
    const std::map<OmClothVertexKey, int>::const_iterator it = seen.find(key);
    if (it != seen.end()) {
      remap[v] = it->second;
      ++welded;
      // A duplicate that carried a DIFFERENT uv is a UV seam: the exporter split
      // it purely for the texture, and the weld is now discarding one side of
      // that split. Counted so the parse warning can distinguish "your asset had
      // genuinely doubled geometry" from "your asset has seams, which is normal".
      if (hasUvs && (uvs[v * 2] != weldedUvs[it->second * 2] || uvs[v * 2 + 1] != weldedUvs[it->second * 2 + 1]))
        ++uvSeamCollapses;
      continue;
    }
    const int index = static_cast<int>(weldedVerts.size() / 3);
    seen.insert(std::make_pair(key, index));
    remap[v] = index;
    weldedVerts.push_back(key.x);
    weldedVerts.push_back(key.y);
    weldedVerts.push_back(key.z);
    if (hasUvs) {
      weldedUvs.push_back(uvs[v * 2]);
      weldedUvs.push_back(uvs[v * 2 + 1]);
    }
  }

  // (2) + (3) Drop out-of-range, degenerate and duplicate triangles.
  std::vector<int> keptTris;
  keptTris.reserve(tris.size());
  // Keyed on the SORTED corner triple, so a face and its reversal count as the
  // same patch of fabric -- which they are, and which is what makes the pair
  // double-stiff and double-mass if both are kept.
  std::map<std::vector<int>, bool> faceSeen;
  const int weldedCount = static_cast<int>(weldedVerts.size() / 3);
  for (int t = 0; t < rawTris; ++t) {
    int a = tris[t * 3], b = tris[t * 3 + 1], c = tris[t * 3 + 2];
    if (a < 0 || b < 0 || c < 0 || a >= rawVerts || b >= rawVerts || c >= rawVerts) {
      ++outOfRange;
      continue;
    }
    a = remap[a];
    b = remap[b];
    c = remap[c];
    if (a == b || b == c || a == c) {
      ++degenerate;
      continue;
    }
    std::vector<int> key(3);
    key[0] = a;
    key[1] = b;
    key[2] = c;
    std::sort(key.begin(), key.end());
    if (faceSeen.find(key) != faceSeen.end()) {
      ++duplicateTris;
      continue;
    }
    faceSeen.insert(std::make_pair(key, true));
    keptTris.push_back(a);
    keptTris.push_back(b);
    keptTris.push_back(c);
  }

  // (4) Drop orphans -- vertices no surviving triangle references. THE most
  // damaging of the four if it survives: add_cloth_mesh gives every particle
  // mass 0 and then credits mass only through the triangles that touch it, so
  // an orphan keeps mass 0, which newton reads as infinite inverse mass. It
  // becomes an immovable nail holding the fabric in mid-air.
  std::vector<int> used(weldedCount, 0);
  for (std::size_t k = 0; k < keptTris.size(); ++k)
    used[keptTris[k]] = 1;
  std::vector<int> compact(weldedCount, -1);
  int next = 0;
  for (int v = 0; v < weldedCount; ++v) {
    if (used[v])
      compact[v] = next++;
    else
      ++orphans;
  }
  mMeshVertices.reserve(static_cast<std::size_t>(next) * 3);
  if (hasUvs)
    mMeshUvs.reserve(static_cast<std::size_t>(next) * 2);
  for (int v = 0; v < weldedCount; ++v) {
    if (compact[v] < 0)
      continue;
    mMeshVertices.push_back(weldedVerts[v * 3]);
    mMeshVertices.push_back(weldedVerts[v * 3 + 1]);
    mMeshVertices.push_back(weldedVerts[v * 3 + 2]);
    if (hasUvs) {
      mMeshUvs.push_back(weldedUvs[v * 2]);
      mMeshUvs.push_back(weldedUvs[v * 2 + 1]);
    }
  }
  mMeshIndices.reserve(keptTris.size());
  for (std::size_t k = 0; k < keptTris.size(); ++k)
    mMeshIndices.push_back(compact[keptTris[k]]);

  if (mMeshVertices.empty() || mMeshIndices.empty()) {
    parsingWarn(tr("'url' file '%1' had no usable triangles left after cleaning.").arg(QFileInfo(path).fileName()));
    mMeshVertices.clear();
    mMeshIndices.clear();
    mMeshUvs.clear();
    return false;
  }

  if (welded || degenerate || duplicateTris || orphans || outOfRange) {
    // Loud with counts, never fatal. The author has to know their asset was
    // edited under them -- especially `orphans`, which would otherwise present
    // as invisible pins, and `welded`, which is the difference between a
    // garment that holds together and one that splits along its seams.
    parsingWarn(tr("'url' mesh '%1' was CLEANED before simulating: welded %2 duplicate vertices, dropped %3 "
                   "degenerate and %4 duplicate triangles, %5 orphan vertices, %6 out-of-range triangles. "
                   "These are silent defects in newton: an orphan vertex keeps mass 0 and becomes a KINEMATIC "
                   "PIN, and an unwelded duplicate leaves the fabric with no stretch or bending across that "
                   "seam.")
                  .arg(QFileInfo(path).fileName())
                  .arg(welded)
                  .arg(degenerate)
                  .arg(duplicateTris)
                  .arg(orphans)
                  .arg(outOfRange));
  }
  OmLog::info(tr("Cloth '%1': loaded mesh '%2' -- %3 vertices, %4 triangles, %5.")
                .arg(usefulName())
                .arg(QFileInfo(path).fileName())
                .arg(static_cast<int>(mMeshVertices.size() / 3))
                .arg(static_cast<int>(mMeshIndices.size() / 3))
                .arg(hasUvs ? tr("kept the asset's UV set (%1 seam vertices collapsed by the position weld)")
                                .arg(uvSeamCollapses) :
                              tr("NO UV set in the file, so a textured 'appearance' will be mapped by a planar XY "
                                 "projection of the rest pose -- a placeholder, and it will look like one on "
                                 "anything but a flat sheet")));
  return true;
}

void OmCloth::downloadAssets() {
  OmBaseNode::downloadAssets();
  // The appearance owns ImageTexture children whose `url` may be remote; without
  // this the first frame renders before the texture exists and never repaints.
  if (abstractAppearance())
    abstractAppearance()->downloadAssets();
}

void OmCloth::preFinalize() {
  OmBaseNode::preFinalize();

  // Same forwarding OmShape::preFinalize does, and for the same reason: an
  // appearance is an ordinary node that has to run its own finalize chain
  // (OmPbrAppearance::preFinalize is where the BRDF LUT is baked and every map
  // is resolved). ⚠ It must be here, ABOVE the mesh-mode early return.
  if (abstractAppearance())
    abstractAppearance()->preFinalize();

  // MESH MODE FIRST: `url` is what selects it, and everything below this block
  // validates the GRID fields, which a garment does not have.
  //
  // ⚠ THE TWO MODES ARE MUTUALLY EXCLUSIVE. A mesh cloth's shape comes entirely
  // from the file, so dimX / dimY / cellX / cellY and the four fix* pins mean
  // nothing here -- and silently ignoring an authored field is exactly how a
  // world ends up looking broken for a reason nothing in the log mentions. So
  // each one that has been moved off its default is named.
  if (mUrl != NULL && mUrl->size() > 0 && !mUrl->item(0).isEmpty()) {
    loadMeshFromUrl();   // warns with specifics on every failure path
    if (isMeshCloth()) {
      if ((mFixLeft && mFixLeft->value()) || (mFixRight && mFixRight->value()) ||
          (mFixBottom && mFixBottom->value()) || (mFixTop && mFixTop->value()))
        parsingWarn(tr("'fixLeft'/'fixRight'/'fixBottom'/'fixTop' are ignored when 'url' is set: they name the "
                       "edges of a rectangular lattice and a mesh has none. A mesh Cloth is unpinned -- support "
                       "it with rigid geometry, i.e. drape it over something. (fixTop defaults to TRUE, so this "
                       "fires even if you did not set it; clear it to silence this.)"));
      if ((mDimX && mDimX->value() != 20) || (mDimY && mDimY->value() != 20))
        parsingWarn(tr("'dimX'/'dimY' are ignored when 'url' is set -- the mesh supplies the geometry."));
      if (mMass && mMass->value() != 0.001)
        parsingWarn(tr("'mass' is ignored when 'url' is set. A mesh cloth is weighed by 'density' in kg/m^2 "
                       "(mass per unit AREA, distributed by triangle area), not by a per-particle mass."));
      if (mMeshScale && mMeshScale->value() <= 0.0) {
        parsingWarn(tr("'meshScale' must be strictly positive; got %1.").arg(mMeshScale->value()));
        mMeshScale->setValue(1.0);
      }
      if (mDensity && mDensity->value() < 0.0) {
        parsingWarn(tr("'density' must not be negative; got %1.").arg(mDensity->value()));
        mDensity->setValue(0.0);
      }
      return;
    }
    // The load failed. Fall through so the grid fields are still validated and
    // the node degrades to whatever patch they describe, rather than to nothing.
  }

  // Clamp to a grid that can actually be built. Every one of these would
  // otherwise reach newton as a degenerate cloth (zero triangles, zero-area
  // cells, zero-mass particles), where the failure surfaces as a solver
  // exception naming none of these fields.
  if (mDimX && mDimX->value() < 1) {
    parsingWarn(tr("'dimX' must be at least 1 (it counts CELLS, not vertices); got %1.").arg(mDimX->value()));
    mDimX->setValue(1);
  }
  if (mDimY && mDimY->value() < 1) {
    parsingWarn(tr("'dimY' must be at least 1 (it counts CELLS, not vertices); got %1.").arg(mDimY->value()));
    mDimY->setValue(1);
  }
  if (mCellX && mCellX->value() <= 0.0) {
    parsingWarn(tr("'cellX' must be strictly positive; got %1.").arg(mCellX->value()));
    mCellX->setValue(0.05);
  }
  if (mCellY && mCellY->value() <= 0.0) {
    parsingWarn(tr("'cellY' must be strictly positive; got %1.").arg(mCellY->value()));
    mCellY->setValue(0.05);
  }
  if (mMass && mMass->value() <= 0.0) {
    parsingWarn(tr("'mass' is the PER-PARTICLE mass and must be strictly positive; got %1.").arg(mMass->value()));
    mMass->setValue(0.001);
  }
}

void OmCloth::postFinalize() {
  OmBaseNode::postFinalize();

  if (abstractAppearance())
    abstractAppearance()->postFinalize();
  // Replacing or editing the appearance repaints the fabric in place -- no
  // reload, no rebuilt mesh, no touched particles. mAppearance's own changed()
  // covers "a different node was plugged in"; the appearance's covers "a field
  // inside it moved", which is what a texture swap or a roughness tweak is.
  if (mAppearance)
    connect(mAppearance, &OmSFNode::changed, this, &OmCloth::updateAppearance);
  updateAppearance();

  // ⚠ THE GEOMETRY MUST EXIST EVEN WHEN THE RENDERER DOES NOT, because
  // worldBounds() is an agent-facing read and "the run had no GL context" is
  // not an answer to "where is the cloth". buildTopology() + computeRestPositions()
  // normally run from createWrenObjects(). D1.4: that hook now runs
  // unconditionally at finalize, so this GUARDED SECOND CALL is ordinarily a
  // no-op; it stays as cheap insurance for any path that finalizes without the
  // hook having run.
  if (mPositions.empty() && particleCount() > 0) {
    buildTopology();
    computeRestPositions();
  }

  // Registration is DEFERRED to the first physics step, not done here. See the
  // long comment in onPhysicsStepStarted() for why -- doing it in postFinalize
  // would open the Newton world before WorldInfo's declarations reach the
  // backend, silently costing this world its declared friction and up axis.
  OmSimulationWorld *const world = OmSimulationWorld::instance();
  if (world != NULL)
    connect(world, &OmSimulationWorld::physicsStepStarted, this, &OmCloth::onPhysicsStepStarted);
}

void OmCloth::onPhysicsStepStarted() {
  if (mRegistrationDone)
    return;
  // One shot whatever the outcome: a failure that re-fires every tick would
  // flood the log (and GET /sim/events) at the basic time step.
  mRegistrationDone = true;

  OmPhysicsBackend *const raw = OmPhysicsBackendRegistry::newtonBackend();
  if (raw == NULL || !raw->isAvailable()) {
    OmLog::warning(tr("Cloth '%1' is inert: the Newton runtime is not available, so there is no solver to "
                      "simulate its particles. The patch renders at its authored rest pose and will not move. Run `python -m omnisim doctor`; on Windows stage the runtime with `make -C src/omnisim bundle-newton-runtime`, on Linux pip newton/warp/mujoco into the system python3 (docs/developer/newton-runtime-bundle.md).")
                     .arg(usefulName()));
    return;
  }
  OmNewtonBackend *const newton = static_cast<OmNewtonBackend *>(raw);

  // ⚠ DO NOT REPLACE THIS WITH A BARE ensureWorldOpen().
  //
  // The Newton world is opened LAZILY, from inside
  // OmSolid::flushPendingNewtonRegistrations(), and that function does one
  // thing first that nothing else does: it caches WorldInfo's coordinateSystem
  // and its contact/solver declaration (newtonGroundMu / newtonContactKe /
  // newtonContactKd / iterations) into the backend BEFORE the world is
  // constructed. newton bakes the up axis into the implicit ground plane's
  // normal and copies mu/ke/kd into every shape AT ADD TIME, so a world opened
  // ahead of that cache can never be given them afterwards -- the exact defect
  // that made a declared newtonGroundMu 2.0 slide at the default on the
  // 55-degree ramp comparator.
  //
  // So: drive the ordinary flush first and let it open + configure the world.
  // It is idempotent (per-Solid registration is one-shot and the function is
  // called every tick by OmSimulationWorld::step anyway), and this runs from
  // physicsStepStarted, which is emitted EARLIER in the same step than the
  // ordinary flush/finalize seam -- so the cloth lands in the same build the
  // solids do, before finalizeWorld() closes it.
  OmSolid::flushPendingNewtonRegistrations();
  if (!newton->isWorldOpenForBuild()) {
    // A cloth-only world: no Solid opened it. The flush above still cached the
    // WorldInfo declarations, so opening it now applies them in the right
    // order. (The solver preference is plumbed by the flush's tail on the next
    // pass, which happens later in this same step, and finalizeWorld()
    // re-asserts it -- so "mujoco+vbd" still reaches the runtime in time.)
    if (newton->ensureWorldOpen() != 0) {
      OmLog::warning(tr("Cloth '%1' is inert: the Newton world could not be opened.").arg(usefulName()));
      return;
    }
  }

  // By value, not by const reference: the null-guard ternary yields a prvalue,
  // and binding a reference to it is a lifetime subtlety this code does not
  // need to have.
  const OmVector3 t = mTranslation ? mTranslation->value() : OmVector3();
  const OmRotation r = mRotation ? mRotation->value() : OmRotation();
  const OmQuaternion q = r.toQuaternion();
  const double pos[3] = {t.x(), t.y(), t.z()};
  // xyzw on the wire -- the house body_q layout, w LAST. OmQuaternion stores
  // w FIRST, so this reorder is load-bearing, not cosmetic.
  const double quat[4] = {q.x(), q.y(), q.z(), q.w()};

  // ⚠ TWO DIFFERENT "UNSET" CONVENTIONS MEET HERE, so this block is a
  // translation, not a set of defaults duplicated for convenience.
  //   .wrl side:     0 means "leave it to the runtime" (the convention every
  //                  WorldInfo newton* field uses, so authors already know it).
  //   runtime side:  NEGATIVE means "derive from the matching ke"; ZERO IS A
  //                  LITERAL ZERO STIFFNESS and would silently produce a sheet
  //                  with no stretch resistance at all.
  // So an unset field must never be forwarded as 0. The two that CAN be derived
  // (triKa, triKd) go out as -1; the three with nothing to derive from
  // (triKe, edgeKe, particleRadius) go out as the runtime's own default value.
  // Keep these three literals in step with omnisim_newton_runtime.py's
  // add_cloth_grid defaults.
  const double triKe = (mTriKe && mTriKe->value() > 0.0) ? mTriKe->value() : 1.0e5;
  const double triKa = (mTriKa && mTriKa->value() > 0.0) ? mTriKa->value() : -1.0;
  const double triKd = (mTriKd && mTriKd->value() > 0.0) ? mTriKd->value() : -1.0;
  const double edgeKe = (mEdgeKe && mEdgeKe->value() > 0.0) ? mEdgeKe->value() : 0.01;
  const double edgeKd = -1.0;  // always derived; not exposed as a field yet
  const double radius = (mParticleRadius && mParticleRadius->value() > 0.0) ? mParticleRadius->value() : 0.01;

  int particleEnd = -1;
  if (isMeshCloth()) {
    const double density = (mDensity && mDensity->value() > 0.0) ? mDensity->value() : 0.3;
    const double meshScale = (mMeshScale && mMeshScale->value() > 0.0) ? mMeshScale->value() : 1.0;
    const double pinTopBand = (mPinTopBand && mPinTopBand->value() > 0.0) ? mPinTopBand->value() : 0.0;
    mParticleStart = newton->addClothMesh(pos, quat, mMeshVertices.data(), particleCount(), mMeshIndices.data(),
                                          static_cast<int>(mMeshIndices.size() / 3), density, radius, triKe,
                                          triKa, triKd, edgeKe, edgeKd, meshScale, pinTopBand, &particleEnd);
    mParticleEnd = particleEnd;
    // ⚠ THE CORRESPONDENCE CHECK, and the reason the runtime's cleaning pass is
    // allowed to exist at all. Particle i must be mesh vertex i: the renderer
    // draws mMeshIndices against a readback of [start, end), so a count that
    // disagrees means the runtime cleaned something this node did not, every
    // index is now off, and the garment would render as noise. Refuse to draw
    // it rather than show that.
    if (mParticleStart >= 0 && mParticleEnd - mParticleStart != particleCount()) {
      OmLog::warning(tr("Cloth '%1' is inert: Newton registered %2 particles for a mesh of %3 vertices. The "
                        "runtime cleaned the mesh further than this node did, so particle i is no longer mesh "
                        "vertex i and the rendered surface would not match the simulated fabric.")
                       .arg(usefulName())
                       .arg(mParticleEnd - mParticleStart)
                       .arg(particleCount()));
      mParticleStart = -1;
      mParticleEnd = -1;
      return;
    }
  } else {
    mParticleStart = newton->addClothGrid(pos, quat, dimX(), dimY(), cellX(), cellY(), mass(), radius,
                                          triKe, triKa, triKd, edgeKe, edgeKd, fixFlags(), &particleEnd);
    mParticleEnd = particleEnd;
  }
  if (mParticleStart < 0) {
    // By far the most likely cause, and the one an author can fix, is the
    // solver: SolverVBD only exists on the coupled path. Name the field and the
    // value rather than reporting a bare failure.
    OmLog::warning(tr("Cloth '%1' registered no particles and will not move. The usual cause is that this world "
                      "does not declare WorldInfo { newtonSolver \"mujoco+vbd\" } -- that is the only solver "
                      "profile that builds the SolverVBD a Cloth needs. The patch still renders at its authored "
                      "rest pose.")
                     .arg(usefulName()));
    return;
  }
  if (isMeshCloth())
    OmLog::info(tr("Cloth '%1': registered %2 particles (mesh, %3 triangles) at Newton particle offset %4.")
                  .arg(usefulName())
                  .arg(particleCount())
                  .arg(static_cast<int>(mMeshIndices.size() / 3))
                  .arg(mParticleStart));
  else
    OmLog::info(tr("Cloth '%1': registered %2 particles (%3 x %4 cells) at Newton particle offset %5.")
                  .arg(usefulName())
                  .arg(particleCount())
                  .arg(dimX())
                  .arg(dimY())
                  .arg(mParticleStart));
}

void OmCloth::computeRestPositions() {
  // Every path below rewrites mPositions; the mesh branch returns early, so the
  // bounds cache is invalidated up here where both branches see it.
  mWorldBoundsDirty = true;

  const OmVector3 t = mTranslation ? mTranslation->value() : OmVector3();
  const OmRotation r = mRotation ? mRotation->value() : OmRotation();
  const OmMatrix3 m = r.toMatrix3();

  if (isMeshCloth()) {
    // ⚠ SCALE, THEN ROTATE, THEN TRANSLATE -- upstream's order for
    // add_cloth_mesh (builder.py:9146-9151), reproduced here rather than
    // reinvented. Get it wrong and the drawn rest pose disagrees with the
    // particles the solver starts from, so the garment visibly JUMPS on the
    // first readback.
    const double s = (mMeshScale && mMeshScale->value() > 0.0) ? mMeshScale->value() : 1.0;
    const int n = particleCount();
    for (int i = 0; i < n; ++i) {
      const OmVector3 local(mMeshVertices[i * 3] * s, mMeshVertices[i * 3 + 1] * s,
                            mMeshVertices[i * 3 + 2] * s);
      const OmVector3 world = t + m * local;
      mPositions[i * 3 + 0] = static_cast<float>(world.x());
      mPositions[i * 3 + 1] = static_cast<float>(world.y());
      mPositions[i * 3 + 2] = static_cast<float>(world.z());
    }
    return;
  }

  const int nx = dimX() + 1;
  const int ny = dimY() + 1;
  const double dx = cellX();
  const double dy = cellY();

  // Row-major, Y outer / X inner, patch lying in local Z = 0 -- the identical
  // layout newton's add_cloth_grid produces, so index i here is particle i
  // there.
  int i = 0;
  for (int y = 0; y < ny; ++y) {
    for (int x = 0; x < nx; ++x, ++i) {
      const OmVector3 world = t + m * OmVector3(x * dx, y * dy, 0.0);
      mPositions[i * 3 + 0] = static_cast<float>(world.x());
      mPositions[i * 3 + 1] = static_cast<float>(world.y());
      mPositions[i * 3 + 2] = static_cast<float>(world.z());
    }
  }
}

void OmCloth::buildTopology() {
  const int n = particleCount();

  mPositions.assign(static_cast<std::size_t>(n) * 3, 0.0f);
  mVertexNormals.assign(static_cast<std::size_t>(n) * 3, 0.0f);
  mIndices.clear();
  // mPositions was just resized and zeroed -- anything the bounds cache holds
  // now describes a geometry that no longer exists. (This function has an early
  // return for mesh cloth, so the flag is set here rather than at the tail.)
  mWorldBoundsDirty = true;

  if (isMeshCloth()) {
    // The asset's own index list, verbatim. There is no rule to re-derive it
    // from -- that is the whole difference between a garment and a patch -- so
    // the ONE copy loaded in preFinalize is what both the renderer and the
    // solver are given, and they cannot disagree.
    mIndices.reserve(mMeshIndices.size());
    for (std::size_t k = 0; k < mMeshIndices.size(); ++k)
      mIndices.push_back(static_cast<unsigned int>(mMeshIndices[k]));
    return;
  }

  const int cellsX = dimX();
  const int cellsY = dimY();
  const int nx = cellsX + 1;
  mIndices.reserve(static_cast<std::size_t>(cellsX) * cellsY * 6);

  // ⚠ THIS WINDING IS COPIED FROM newton's ModelBuilder.add_cloth_grid AND MUST
  // TRACK IT. For each cell it emits (v0, v1, v3) then (v1, v2, v3) with
  //   v0 = (x-1, y-1)   v1 = (x, y-1)   v2 = (x, y)   v3 = (x-1, y)
  // over a vertex index of y * (dimX + 1) + x. Deriving our triangles from the
  // same rule -- rather than from a screenshot or a guess -- is what makes the
  // rendered surface and the simulated particle set the same object.
  for (int y = 1; y <= cellsY; ++y) {
    for (int x = 1; x <= cellsX; ++x) {
      const unsigned int v0 = static_cast<unsigned int>((y - 1) * nx + (x - 1));
      const unsigned int v1 = static_cast<unsigned int>((y - 1) * nx + x);
      const unsigned int v2 = static_cast<unsigned int>(y * nx + x);
      const unsigned int v3 = static_cast<unsigned int>(y * nx + (x - 1));
      mIndices.push_back(v0);
      mIndices.push_back(v1);
      mIndices.push_back(v3);
      mIndices.push_back(v1);
      mIndices.push_back(v2);
      mIndices.push_back(v3);
    }
  }
}

void OmCloth::uploadPositions() {
  const int n = particleCount();
  const std::size_t need = static_cast<std::size_t>(n) * 3;
  if (n <= 0 || mPositions.size() < need || mVertexNormals.size() < need)
    return;

  // ⚠ ANGLE-WEIGHTED, NOT AREA-WEIGHTED, AND ON CLOTH THAT IS NOT A REFINEMENT.
  //
  // This loop used to sum the UN-normalised edge cross products, whose magnitude
  // is 2*area -- so each face was weighted by its own area, for free, which is
  // the standard cheap choice and the right one for a RIGID mesh whose triangle
  // areas never change.
  //
  // A cloth solver changes them every step, and that is the whole problem: under
  // area weighting a vertex normal becomes a function of the LOCAL STRAIN around
  // it, i.e. of the one quantity that moves most per frame. The shading then
  // swims across exactly the folds and gathers the demo exists to show, because
  // a fold is where the solver is stretching one incident triangle and
  // compressing its neighbour. ANGLE weighting -- weight each unit face normal by
  // the interior angle it subtends AT THIS VERTEX -- is invariant to any scaling
  // of the incident triangles, so it depends only on the surface's SHAPE at the
  // corner and is stable frame to frame. It is also the weighting the literature
  // settled on for irregular meshes (Thurmer & Wuthrich 1998; Max, "Weights for
  // Computing Vertex Normals from Facet Normals", JGT 4(2) 1999; Jin/Lewis/West,
  // The Visual Computer 21:71-82, 2005), and the one MeshLab rates "good" where
  // it calls area weighting "simple but dangerous".
  //
  // COST: 4 sqrt + 3 acos per triangle instead of none. On the shipped garments
  // (1157 tri, or 4674 for the hi-res one) that is ~5k-20k acos per frame, well
  // under the per-frame FFI readback it follows. If it ever does show up in a
  // profile, Max's MWSELR weight -- sin(theta)/(|e1||e2|), obtained by dividing
  // the RAW cross product by |e1|^2 |e2|^2 -- approximates this with no trig at
  // all; it is a weaker guarantee (it still varies with edge length, so it is
  // only partly strain-free) but far better than area. It was not needed.
  //
  // ⚠ NO SEAM WELD IS NEEDED HERE, and that is worth stating because it is the
  // usual defect in code shaped like this. Accumulating over INDEX-BUFFER
  // vertices leaves a permanent lighting crease along every UV seam whenever the
  // exporter split a vertex there -- the two halves never see each other's
  // triangles. It cannot happen in this node: mIndices is mMeshIndices, which
  // loadMeshFromUrl() has already remapped through an EXACT-POSITION weld (the
  // weld is mandatory for the physics, not for the shading -- an unwelded seam
  // has no stretch and no bending across it), so the render indices always
  // address position-welded vertices. A grid patch has no duplicates by
  // construction. Measured on the two shipped assets, neither is even split to
  // begin with: tshirt.obj is 616 positions / 616 UVs and tshirt_hi.obj is
  // 2394 / 2394, so the weld is an identity on both.
  std::memset(mVertexNormals.data(), 0, mVertexNormals.size() * sizeof(float));
  for (std::size_t k = 0; k + 2 < mIndices.size(); k += 3) {
    const unsigned int i0 = mIndices[k], i1 = mIndices[k + 1], i2 = mIndices[k + 2];
    const float *const p0 = &mPositions[i0 * 3];
    const float *const p1 = &mPositions[i1 * 3];
    const float *const p2 = &mPositions[i2 * 3];
    const float e1[3] = {p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]};
    const float e2[3] = {p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]};
    // The face normal, now NORMALISED: the weight has to be the angle alone, so
    // the face's own area must not ride in on the normal's length as well.
    float fn[3] = {e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2],
                   e1[0] * e2[1] - e1[1] * e2[0]};
    const float fnLen = std::sqrt(fn[0] * fn[0] + fn[1] * fn[1] + fn[2] * fn[2]);
    // A collapsed triangle has no normal AND no interior angles, so it
    // contributes nothing rather than contributing garbage. A vertex all of
    // whose triangles collapse still lands in the degenerate-fan fallback below.
    if (!(fnLen > 1e-20f))
      continue;
    fn[0] /= fnLen;
    fn[1] /= fnLen;
    fn[2] /= fnLen;

    // Unit edge directions round the loop, v0->v1, v1->v2, v2->v0. Each is used
    // by the TWO corners it touches, so this is 3 normalisations per triangle
    // rather than 6.
    float u[3][3] = {{e1[0], e1[1], e1[2]},
                     {p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]},
                     {-e2[0], -e2[1], -e2[2]}};
    bool edgesUsable = true;
    for (int e = 0; e < 3 && edgesUsable; ++e) {
      const float len = std::sqrt(u[e][0] * u[e][0] + u[e][1] * u[e][1] + u[e][2] * u[e][2]);
      if (!(len > 1e-20f)) {
        edgesUsable = false;  // unreachable given fnLen > 0; cheap insurance against denormals
        break;
      }
      u[e][0] /= len;
      u[e][1] /= len;
      u[e][2] /= len;
    }
    if (!edgesUsable)
      continue;

    // The interior angle at a corner is between the two edges LEAVING it, and
    // the loop above gives one of each pair reversed -- hence the negated dots.
    const float cosAngle[3] = {-(u[0][0] * u[2][0] + u[0][1] * u[2][1] + u[0][2] * u[2][2]),
                               -(u[1][0] * u[0][0] + u[1][1] * u[0][1] + u[1][2] * u[0][2]),
                               -(u[2][0] * u[1][0] + u[2][1] * u[1][1] + u[2][2] * u[1][2])};
    const unsigned int idx[3] = {i0, i1, i2};
    for (int c = 0; c < 3; ++c) {
      // ⚠ CLAMP BEFORE acos. The dot of two float unit vectors lands a few ulps
      // outside [-1, 1] on a sliver triangle, and acos(1.0000001f) is NaN -- a
      // single one of which propagates through the accumulation and renders that
      // vertex's whole fan black, intermittently, on one frame in thousands.
      const float cosClamped = cosAngle[c] < -1.0f ? -1.0f : (cosAngle[c] > 1.0f ? 1.0f : cosAngle[c]);
      const float weight = std::acos(cosClamped);
      mVertexNormals[idx[c] * 3 + 0] += fn[0] * weight;
      mVertexNormals[idx[c] * 3 + 1] += fn[1] * weight;
      mVertexNormals[idx[c] * 3 + 2] += fn[2] * weight;
    }
  }

  // D1.4: WREN's per-frame coordinate/normal re-stream is gone. mVertexNormals
  // deliberately stays the RAW angle-weighted accumulation: wgpuVertexStream()
  // owns the normalisation (with the +Z degenerate-fan fallback that used to
  // live in the WREN upload loop), and it runs it per frame at interleave time.
}

// ---- wgpu MAIN-VIEW DRAW SOURCE ---------------------------------------------
//
// The interleave is the wgpu vertex layout (pos3 + norm3 + uv2 float, stride 32).
// mVertexNormals holds the RAW angle-weighted accumulation from
// uploadPositions(); this is the one place it is normalised (+Z fallback for a
// fully degenerate fan). The loop is one pass over the particles.
bool OmCloth::wgpuVertexStream(std::vector<unsigned char> &out) const {
  const std::size_t n = mPositions.size() / 3;
  if (n == 0 || mIndices.empty() || mVertexNormals.size() < n * 3 || mTexCoords.size() < n * 2)
    return false;
  out.resize(n * 32);
  unsigned char *const dst = out.data();
  for (std::size_t i = 0; i < n; ++i) {
    float rec[8];
    rec[0] = mPositions[i * 3 + 0];
    rec[1] = mPositions[i * 3 + 1];
    rec[2] = mPositions[i * 3 + 2];
    // ⚠ mVertexNormals is the RAW angle-weighted accumulation -- normalise here,
    // exactly as uploadPositions() does on its way into WREN, +Z fallback and all.
    float nx = mVertexNormals[i * 3 + 0];
    float ny = mVertexNormals[i * 3 + 1];
    float nz = mVertexNormals[i * 3 + 2];
    const float len = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (len > 1e-12f) {
      nx /= len;
      ny /= len;
      nz /= len;
    } else {
      nx = 0.0f;
      ny = 0.0f;
      nz = 1.0f;
    }
    rec[3] = nx;
    rec[4] = ny;
    rec[5] = nz;
    rec[6] = mTexCoords[i * 2 + 0];
    rec[7] = mTexCoords[i * 2 + 1];
    std::memcpy(dst + i * 32, rec, 32);
  }
  return true;
}

bool OmCloth::wgpuCastShadows() const {
  return mCastShadows ? mCastShadows->value() : true;
}

void OmCloth::wgpuFallbackDiffuse(float rgb3[3]) const {
  rgb3[0] = static_cast<float>(mDiffuseColor ? mDiffuseColor->red() : 0.8);
  rgb3[1] = static_cast<float>(mDiffuseColor ? mDiffuseColor->green() : 0.2);
  rgb3[2] = static_cast<float>(mDiffuseColor ? mDiffuseColor->blue() : 0.2);
}

void OmCloth::animateMesh() {
  // Not simulated (no runtime, or no VBD solver): the rest pose is already in
  // mPositions and never changes, so there is nothing to re-upload. Returning
  // here is what keeps a misconfigured Cloth from burning a readback per frame.
  if (mParticleStart < 0)
    return;

  OmPhysicsBackend *const raw = OmPhysicsBackendRegistry::newtonBackend();
  if (raw == NULL || !raw->isAvailable())
    return;
  const OmNewtonBackend *const newton = static_cast<const OmNewtonBackend *>(raw);
  if (!newton->isWorldRunning())
    return;

  const int n = particleCount();
  if (n <= 0 || mParticleEnd - mParticleStart != n || mPositions.size() < static_cast<std::size_t>(n) * 3)
    return;

  // Straight into the vertex array: the runtime slices OUR half-open range
  // server-side, so there is no staging buffer and no discarded transfer even
  // when the world holds several sheets.
  const int got = newton->snapshotParticlePositions(mParticleStart, mParticleEnd, mPositions.data());
  if (got != n)
    return;  // short read -- keep the previous frame's geometry rather than tearing

  // The fabric has moved, so the cached AABB describes last frame. Only the flag
  // is set here: the rescan is deferred to whoever asks, which on a frame nobody
  // queries is nobody. A short read above leaves the previous frame's geometry
  // AND its bounds, which stay consistent with each other.
  mWorldBoundsDirty = true;

  uploadPositions();
}

void OmCloth::refreshWorldBounds() const {
  mWorldBoundsDirty = false;
  mWorldBoundsValid = false;

  const int n = particleCount();
  if (n <= 0 || mPositions.size() < static_cast<std::size_t>(n) * 3)
    return;

  double lo[3] = {0.0, 0.0, 0.0};
  double hi[3] = {0.0, 0.0, 0.0};
  bool any = false;
  for (int i = 0; i < n; ++i) {
    const float *const p = &mPositions[i * 3];
    // ⚠ A NON-FINITE PARTICLE IS SKIPPED, NOT PROPAGATED. A cloth solve that
    // blows up leaves NaNs in the readback, and min/max with a NaN is
    // implementation-defined enough that one of them can poison every
    // component -- which would hand a camera-framing computation a NaN target
    // and aim it nowhere, silently. Measuring the particles that ARE finite is
    // both more useful and more honest than reporting NaN bounds; when NONE is
    // finite the accessors report no bounds at all.
    if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2]))
      continue;
    if (!any) {
      for (int c = 0; c < 3; ++c) {
        lo[c] = p[c];
        hi[c] = p[c];
      }
      any = true;
      continue;
    }
    for (int c = 0; c < 3; ++c) {
      const double v = p[c];
      if (v < lo[c])
        lo[c] = v;
      if (v > hi[c])
        hi[c] = v;
    }
  }
  if (!any)
    return;

  for (int c = 0; c < 3; ++c) {
    mWorldBoundsMin[c] = lo[c];
    mWorldBoundsMax[c] = hi[c];
  }
  mWorldBoundsValid = true;
}

bool OmCloth::worldBounds(OmVector3 &minCorner, OmVector3 &maxCorner) const {
  if (mWorldBoundsDirty)
    refreshWorldBounds();
  if (!mWorldBoundsValid)
    return false;
  minCorner = OmVector3(mWorldBoundsMin[0], mWorldBoundsMin[1], mWorldBoundsMin[2]);
  maxCorner = OmVector3(mWorldBoundsMax[0], mWorldBoundsMax[1], mWorldBoundsMax[2]);
  return true;
}

bool OmCloth::worldCenterAndRadius(OmVector3 &center, double &radius) const {
  if (mWorldBoundsDirty)
    refreshWorldBounds();
  if (!mWorldBoundsValid)
    return false;
  const double sx = mWorldBoundsMax[0] - mWorldBoundsMin[0];
  const double sy = mWorldBoundsMax[1] - mWorldBoundsMin[1];
  const double sz = mWorldBoundsMax[2] - mWorldBoundsMin[2];
  center = OmVector3(mWorldBoundsMin[0] + 0.5 * sx, mWorldBoundsMin[1] + 0.5 * sy, mWorldBoundsMin[2] + 0.5 * sz);
  // The sphere that CIRCUMSCRIBES the box (half its diagonal), which is what a
  // "fit this in frame" computation needs -- an inscribed radius would clip the
  // corners of a flat sheet, and a flat sheet is the common case here.
  radius = 0.5 * std::sqrt(sx * sx + sy * sy + sz * sz);
  return true;
}

void OmCloth::createWrenObjects() {
  OmBaseNode::createWrenObjects();

  if (particleCount() <= 0)
    return;

  buildTopology();
  computeRestPositions();
  buildTextureCoordinates();
  uploadPositions();

  // The appearance is an ordinary node and drives its own texture CPU loads
  // from its createWrenObjects() chain; the wgpu material path reads what it
  // latches. Same order OmShape::createWrenObjects uses.
  if (abstractAppearance())
    abstractAppearance()->createWrenObjects();

  // D1.4: fabric double-sidedness (no back-face culling, gl_FrontFacing
  // two-sided shading) is now wholly the wgpu renderer's concern; the measured
  // front-normal-on-back-face notes that used to live here moved with the WREN
  // renderable they described.

  // Subscribe for the per-frame vertex stream. The listener only fires when sim
  // time has actually advanced, so a paused simulation costs nothing.
  OmDeformableFrameListener::instance()->subscribeCloth(this);
}

void OmCloth::buildTextureCoordinates() {
  const int n = particleCount();
  mTexCoords.assign(static_cast<std::size_t>(n) * 2, 0.0f);
  if (n <= 0)
    return;

  if (isMeshCloth()) {
    // THE ASSET'S OWN PARAMETERISATION when it had one. This is the only mapping
    // that can put a print on a chest and a rib on a collar, and no rule can
    // re-derive it -- which is why it is now carried through the import and the
    // weld rather than discarded (see loadMeshFromUrl).
    if (mMeshUvs.size() == static_cast<std::size_t>(n) * 2) {
      for (int i = 0; i < n; ++i) {
        mTexCoords[i * 2 + 0] = static_cast<float>(mMeshUvs[i * 2 + 0]);
        // OBJ/glTF put v=0 at the BOTTOM of the image, GL sampling here puts it
        // at the top; flipping once, in the one place UVs enter the renderer,
        // keeps every authored texture the right way up.
        mTexCoords[i * 2 + 1] = 1.0f - static_cast<float>(mMeshUvs[i * 2 + 1]);
      }
      return;
    }
    // FALLBACK: a planar XY projection of the rest pose, normalised over its own
    // bounding box. It is a PLACEHOLDER for an asset that shipped without UVs --
    // fine on a flat panel, and visibly wrong on anything with a sleeve, because
    // front and back of a tube project onto the same texels.
    float lo[2] = {mPositions[0], mPositions[1]};
    float hi[2] = {mPositions[0], mPositions[1]};
    for (int i = 1; i < n; ++i) {
      for (int c = 0; c < 2; ++c) {
        lo[c] = std::min(lo[c], mPositions[i * 3 + c]);
        hi[c] = std::max(hi[c], mPositions[i * 3 + c]);
      }
    }
    const float spanX = (hi[0] - lo[0]) > 1e-9f ? (hi[0] - lo[0]) : 1.0f;
    const float spanY = (hi[1] - lo[1]) > 1e-9f ? (hi[1] - lo[1]) : 1.0f;
    for (int i = 0; i < n; ++i) {
      mTexCoords[i * 2 + 0] = (mPositions[i * 3] - lo[0]) / spanX;
      mTexCoords[i * 2 + 1] = (mPositions[i * 3 + 1] - lo[1]) / spanY;
    }
    return;
  }

  // Grid patch: the lattice IS the parameterisation, so a texture lands square
  // on the sheet with no authoring at all. Unchanged from the original.
  const int nx = dimX() + 1;
  const int ny = dimY() + 1;
  const float invX = 1.0f / static_cast<float>(dimX());
  const float invY = 1.0f / static_cast<float>(dimY());
  int i = 0;
  for (int y = 0; y < ny; ++y) {
    for (int x = 0; x < nx; ++x, ++i) {
      mTexCoords[i * 2 + 0] = x * invX;
      mTexCoords[i * 2 + 1] = y * invY;
    }
  }
}

void OmCloth::updateAppearance() {
  // Field edits inside the appearance repaint without a reload; the connection
  // is re-made here (rather than only in postFinalize) because the appearance
  // NODE itself can be swapped at runtime.
  if (pbrAppearance())
    connect(pbrAppearance(), &OmPbrAppearance::changed, this, &OmCloth::updateAppearance, Qt::UniqueConnection);
  else if (appearance())
    connect(appearance(), &OmAppearance::changed, this, &OmCloth::updateAppearance, Qt::UniqueConnection);

  if (!areWrenObjectsInitialized())
    return;

  // A swapped-in appearance node still has to run its own setup chain (texture
  // CPU loads in particular); the wgpu renderer reads whatever it latches.
  if (abstractAppearance())
    abstractAppearance()->createWrenObjects();
}

void OmCloth::reset(const QString &id) {
  OmBaseNode::reset(id);
  if (abstractAppearance())
    abstractAppearance()->reset(id);
}
