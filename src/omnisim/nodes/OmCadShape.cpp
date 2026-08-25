// Copyright 1996-2024 Cyberbotics Ltd.
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
// Modifications copyright 2026 OmniLink, licensed under the Apache License, Version 2.0.

#include "OmCadShape.hpp"

#include "OmApplicationInfo.hpp"
#include "OmBackground.hpp"
#include "OmBoundingSphere.hpp"
#include "OmDownloadManager.hpp"
#include "OmDownloader.hpp"
#include "OmField.hpp"
#include "OmLog.hpp"
#include "OmMFString.hpp"
#include "OmMatrix4.hpp"
#include "OmNetwork.hpp"
#include "OmNodeUtilities.hpp"
#include "OmPbrAppearance.hpp"
#include "OmPose.hpp"
#include "OmRgb.hpp"
#include "OmSolid.hpp"
#include "OmTransform.hpp"
#include "OmUrl.hpp"
#include "OmViewpoint.hpp"
#include "OmWorld.hpp"

#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>

#include <QtCore/QtGlobal>

#include <cmath>
#include <cstring>
#include <utility>

namespace {

  // The wgpu vertex layout OmWgpuMeshCache::acquire() expects: pos3 (12) + norm3 (12) + uv2 (8).
  constexpr size_t kCadWgpuStride = 32;

  // D1.4: the OMNISIM_WGPU_NATIVE_CADSHAPE hatch is RETIRED -- the native submesh path is
  // unconditional now that its WREN-readback fallback arm no longer exists. The
  // OMNISIM_WGPU_CADSHAPE_AUDIT diagnostic went with it: it existed only to compare the pose
  // matrix against wr_transform_get_matrix, and there is no WREN matrix left to compare to.

  // 64-bit FNV-1a, chainable (pass the previous result as `hash` to keep hashing).
  uint64_t fnv1a64(const void *data, size_t length, uint64_t hash) {
    const unsigned char *p = static_cast<const unsigned char *>(data);
    for (size_t i = 0; i < length; ++i) {
      hash ^= static_cast<uint64_t>(p[i]);
      hash *= 1099511628211ULL;
    }
    return hash;
  }

  constexpr uint64_t kFnv1a64Offset = 1469598103934665603ULL;

}  // namespace

void OmCadShape::init() {
  mUrl = findMFString("url");
  mCcw = findSFBool("ccw");
  mCastShadows = findSFBool("castShadows");
  mIsPickable = findSFBool("isPickable");

  mPbrAppearances.clear();

  mWgpuSubmeshes.clear();

  mObjMaterials.clear();
  mMaterialDownloaders.clear();

  mDownloader = NULL;
  mBoundingSphere = NULL;
}

OmCadShape::OmCadShape(OmTokenizer *tokenizer) : OmBaseNode("CadShape", tokenizer) {
  init();
}

OmCadShape::OmCadShape(const OmCadShape &other) : OmBaseNode(other) {
  init();
}

OmCadShape::OmCadShape(const OmNode &other) : OmBaseNode(other) {
  init();
}

OmCadShape::~OmCadShape() {
  if (areWrenObjectsInitialized())
    deleteWrenObjects();

  delete mBoundingSphere;
}

void OmCadShape::downloadAssets() {
  if (mUrl->size() == 0)
    return;

  const QString &completeUrl = OmUrl::computePath(this, "url", mUrl->item(0));
  if (!OmUrl::isWeb(completeUrl) ||
      (OmNetwork::instance()->isCachedWithMapUpdate(completeUrl) && areMaterialAssetsAvailable(completeUrl)))
    return;

  delete mDownloader;
  mDownloader = OmDownloadManager::instance()->createDownloader(QUrl(completeUrl), this);
  if (!OmWorld::instance()->isLoading())  // URL changed from the scene tree or supervisor
    connect(mDownloader, &OmDownloader::complete, this, &OmCadShape::downloadUpdate);
  mDownloader->download();
}

void OmCadShape::downloadUpdate() {
  updateUrl();
  OmWorld::instance()->viewpoint()->emit refreshRequired();
}

void OmCadShape::retrieveMaterials() {
  const QString &completeUrl = OmUrl::computePath(this, "url", mUrl->item(0));

  qDeleteAll(mMaterialDownloaders);
  mMaterialDownloaders.clear();

  const QStringList rawMaterials = objMaterialList(completeUrl);
  foreach (const QString &material, rawMaterials) {
    const QString newUrl = OmUrl::combinePaths(material, completeUrl);
    if (!newUrl.isEmpty()) {
      mObjMaterials.insert(material, newUrl);
      // prepare a downloader
      OmDownloader *downloader = OmDownloadManager::instance()->createDownloader(QUrl(newUrl), this);
      connect(downloader, &OmDownloader::complete, this, &OmCadShape::materialDownloadTracker);
      mMaterialDownloaders.push_back(downloader);
    }
  }

  // start all downloads only when the vector is entirely populated (to avoid racing conditions)
  assert(mMaterialDownloaders.size() == mObjMaterials.size());
  foreach (OmDownloader *downloader, mMaterialDownloaders)
    downloader->download();
}

void OmCadShape::materialDownloadTracker() {
  bool finished = true;
  foreach (const OmDownloader *downloader, mMaterialDownloaders) {
    if (!downloader->hasFinished())
      finished = false;

    if (!downloader->error().isEmpty()) {
      warn(downloader->error());  // failure downloading or file does not exist (404)
      return;
    }
  }

  if (finished)
    updateUrl();
}

void OmCadShape::preFinalize() {
  OmBaseNode::preFinalize();
  updateUrl();
}

void OmCadShape::postFinalize() {
  OmBaseNode::postFinalize();

  connect(mUrl, &OmMFString::changed, this, &OmCadShape::updateUrl);
  connect(mCcw, &OmSFBool::changed, this, &OmCadShape::updateCcw);
  connect(mCastShadows, &OmSFBool::changed, this, &OmCadShape::updateCastShadows);
  connect(mIsPickable, &OmSFBool::changed, this, &OmCadShape::updateIsPickable);

  // D1.4: the backgroundColorChanged -> createWrenObjects rebuild is gone. It existed only to
  // re-bind the background irradiance into the WREN PBR materials; the wgpu renderer owns the
  // background/IBL coupling itself, and a full assimp re-import per sky edit would be pure cost.

  mBoundingSphere = new OmBoundingSphere(this);
  recomputeBoundingSphere();

  // apply segmentation color
  const OmSolid *solid = OmNodeUtilities::findUpperSolid(this);
  OmRgb color(0.0, 0.0, 0.0);
  while (solid) {
    if (solid->recognitionColorSize() > 0) {
      color = solid->recognitionColor(0);
      break;
    }
    solid = OmNodeUtilities::findUpperSolid(solid);
  }
  setSegmentationColor(color);
}

void OmCadShape::updateUrl() {
  // we want to replace the windows backslash path separators (if any) with cross-platform forward slashes
  const int n = mUrl->size();
  for (int i = 0; i < n; i++) {
    QString item = mUrl->item(i);
    mUrl->blockSignals(true);
    mUrl->setItem(i, item.replace("\\", "/"));
    mUrl->blockSignals(false);
  }

  const QString &completeUrl = OmUrl::computePath(this, "url", mUrl, 0, true);
  if (completeUrl.isEmpty() || completeUrl == OmUrl::missingTexture()) {
    if (areWrenObjectsInitialized())
      deleteWrenObjects();
    return;
  }

  if (OmUrl::isWeb(completeUrl)) {
    if (mDownloader && !mDownloader->error().isEmpty()) {
      warn(mDownloader->error());  // failure downloading or file does not exist (404)
      if (areWrenObjectsInitialized())
        deleteWrenObjects();
      delete mDownloader;
      mDownloader = NULL;
      return;
    }

    if (!OmNetwork::instance()->isCachedWithMapUpdate(completeUrl)) {
      if (mDownloader && mDownloader->hasFinished()) {
        delete mDownloader;
        mDownloader = NULL;
      }

      downloadAssets();  // URL was changed from the scene tree or supervisor
      return;
    }
  }

  const QString extension = completeUrl.mid(completeUrl.lastIndexOf('.') + 1).toLower();
  if (extension == "obj" && OmUrl::isWeb(completeUrl)) {
    // ensure any mtl referenced by the obj file are also downloaded
    if (areMaterialAssetsAvailable(completeUrl)) {
      mObjMaterials.clear();
      // generate mapping between referenced files and cached files
      const QStringList rawMaterials = objMaterialList(completeUrl);
      foreach (const QString &material, rawMaterials) {
        const QString adjustedUrl = OmUrl::combinePaths(material, completeUrl);
        assert(OmNetwork::instance()->isCachedNoMapUpdate(adjustedUrl));
        if (!mObjMaterials.contains(material))
          mObjMaterials.insert(material, adjustedUrl);
      }
    } else {
      retrieveMaterials();
      return;
    }
  }

  if (areWrenObjectsInitialized())
    createWrenObjects();
}

bool OmCadShape::areMaterialAssetsAvailable(const QString &url) {
  QStringList rawMaterials = objMaterialList(url);  // note: 'dae' files will generate an empty list
  foreach (const QString &material, rawMaterials) {
    if (!OmNetwork::instance()->isCachedWithMapUpdate(OmUrl::combinePaths(material, url)))
      return false;
  }
  return true;
}

QStringList OmCadShape::objMaterialList(const QString &url) const {
  const QString extension = url.mid(url.lastIndexOf('.') + 1).toLower();
  if (extension != "obj")
    return QStringList();

  QStringList materials;
  QFile objFile;
  if (OmNetwork::instance()->isCachedWithMapUpdate(url))
    objFile.setFileName(OmNetwork::instance()->get(url));
  else  // local file
    objFile.setFileName(url);
  if (objFile.open(QIODevice::ReadOnly)) {
    QString content = QString(objFile.readAll());
    content = content.replace("\r\n", "\n");

    QStringList lines = content.split('\n', Qt::SkipEmptyParts);
    foreach (const QString &line, lines) {
      QString cleanLine = line.trimmed();
      if (!cleanLine.startsWith("mtllib"))
        continue;

      materials << cleanLine.replace("mtllib ", "").replace("\"", "").trimmed();
    }
  } else
    warn(tr("File '%1' cannot be read.").arg(url));

  objFile.close();

  return materials;
}

void OmCadShape::updateCcw() {
  // D1.4: the WREN per-renderable front-face flip is gone. The wgpu draw reads
  // `ccw` off the node at collect time.
}

bool OmCadShape::wgpuCastShadows() const {
  return mCastShadows->value();
}

// ---- WREN-retirement W1b ------------------------------------------------------------------

bool OmCadShape::wgpuNativeSubmesh(int i, WgpuSubmesh &out) const {
  if (i < 0 || static_cast<size_t>(i) >= mWgpuSubmeshes.size())
    return false;
  const WgpuSubmeshData &data = mWgpuSubmeshes[static_cast<size_t>(i)];
  if (data.vertexBytes.empty() || data.indices.empty())
    return false;
  out.vertexBytes = data.vertexBytes.data();
  out.vertexBytesLen = data.vertexBytes.size();
  out.indices = data.indices.data();
  out.indexBytesLen = data.indices.size() * sizeof(unsigned int);
  out.indexCount = static_cast<unsigned int>(data.indices.size());
  out.contentKey = data.contentKey;
  return true;
}

bool OmCadShape::wgpuWorldMatrix(OmMatrix4 &out) const {
  // Every per-submesh WREN transform used to be an IDENTITY child of the enclosing pose's
  // transform, so all N submesh transforms carried the same world matrix as that pose.
  // OmAbstractPose::updateMatrix() computes the same composition on the engine side, and it
  // is already the matrix source the OmShape path uses via OmGeometry::matrix(). The
  // equality was verified numerically (the retired OMNISIM_WGPU_CADSHAPE_AUDIT diagnostic)
  // before the WREN transforms were deleted.
  const OmPose *const pose = upperPose();
  out = pose ? pose->matrix() : OmMatrix4();
  return true;
}

void OmCadShape::wgpuAuditMatrices() const {
  // D1.4: permanently a no-op. This diagnostic compared wgpuWorldMatrix() against
  // wr_transform_get_matrix on the per-submesh WREN transforms; those transforms no longer
  // exist, and the equality it was built to prove was established while they did (maxAbsDelta
  // read 0 on the audited worlds). The entry point is kept so callers need no change.
}

void OmCadShape::updateCastShadows() {
  // D1.4: no WREN renderables to update -- wgpuCastShadows() reads the field live.
}

void OmCadShape::updateIsPickable() {
  // D1.4: WREN picking is gone; scene picking is served by the wgpu picker, which reads the
  // field through the node.
}

void OmCadShape::setSegmentationColor(const OmRgb &) {
  // D1.4: the WREN segmentation materials are gone. Kept as the target of
  // updateSegmentationColor() so the tree-wide recolour walk still terminates here.
}

void OmCadShape::createWrenObjects() {
  OmBaseNode::createWrenObjects();

  deleteWrenObjects();

  if (mUrl->size() == 0)
    return;

  const QString &completeUrl = OmUrl::computePath(this, "url", mUrl->item(0));
  if (completeUrl.isEmpty())
    return;
  const QString extension = completeUrl.mid(completeUrl.lastIndexOf('.') + 1).toLower();

  Assimp::Importer importer;
  importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS,
                              aiComponent_CAMERAS | aiComponent_LIGHTS | aiComponent_BONEWEIGHTS | aiComponent_ANIMATIONS);

  unsigned int flags = aiProcess_ValidateDataStructure | aiProcess_Triangulate | aiProcess_GenSmoothNormals |
                       aiProcess_JoinIdenticalVertices | aiProcess_OptimizeGraph | aiProcess_RemoveComponent |
                       aiProcess_FlipUVs;

  const aiScene *scene;
  if (extension != "dae" && extension != "obj") {
    warn(tr("Invalid URL '%1'. CadShape node expects file in Collada ('.dae') or Wavefront ('.obj') format.").arg(completeUrl));
    return;
  }

  if (OmUrl::isWeb(completeUrl)) {
    if (!OmNetwork::instance()->isCachedWithMapUpdate(completeUrl)) {
      if (mDownloader == NULL)  // never attempted to download it, try now
        downloadAssets();
      return;
    }

    QFile file(OmNetwork::instance()->get(completeUrl));
    if (!file.open(QIODevice::ReadOnly)) {
      warn(tr("File could not be read: '%1'").arg(completeUrl));
      return;
    }

    QByteArray data = file.readAll();
    // for remote 'obj' files that reference materials, this reference needs to be changed to point to the cached asset instead
    if (extension == "obj") {
      QMapIterator<QString, QString> it(mObjMaterials);
      while (it.hasNext()) {
        it.next();
        data.replace(it.key().toUtf8(), OmNetwork::instance()->get(it.value()).toUtf8());
      }
    }

    scene = importer.ReadFileFromMemory(data.constData(), data.size(), flags, extension.toUtf8().constData());
  } else
    scene = importer.ReadFile(completeUrl.toStdString().c_str(), flags);

  if (!scene) {
    warn(tr("Invalid data, please verify mesh file: %1").arg(importer.GetErrorString()));
    return;
  }

  // Assimp fix for up_axis, adapted from https://github.com/assimp/assimp/issues/849
  if (extension == "dae")  // rotate around X by 90° to swap Y and Z axis
    scene->mRootNode->mTransformation =
      aiMatrix4x4(1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1) * scene->mRootNode->mTransformation;

  std::list<aiNode *> queue;
  queue.push_back(scene->mRootNode);

  aiNode *node;
  while (!queue.empty()) {
    node = queue.front();
    queue.pop_front();

    for (unsigned int i = 0; i < node->mNumMeshes; ++i) {
      const aiMesh *mesh = scene->mMeshes[node->mMeshes[i]];

      // compute absolute transform of this node from all the parents
      const int vertices = mesh->mNumVertices;
      const int faces = mesh->mNumFaces;
      if (vertices < 3)  // silently ignore meshes with less than 3 vertices as they are invalid
        continue;

      if (vertices > 100000)
        warn(tr("Mesh '%1' has more than 100'000 vertices, it is recommended to reduce the number of vertices.")
               .arg(mesh->mName.C_Str()));

      aiMatrix4x4 transform;
      const aiNode *current = node;
      while (current != NULL) {
        transform *= current->mTransformation;
        current = current->mParent;
      }

      // create the arrays
      // std::vector, not new[]: the `currentIndexIndex == 0` early-continue below skipped the
      // delete[]s and leaked all four arrays for every submesh whose faces were all degenerate.
      // Owning containers make that structurally impossible, and W1b keeps the data anyway.
      int currentCoordIndex = 0;
      std::vector<float> coordData(static_cast<size_t>(3 * vertices));
      int currentNormalIndex = 0;
      std::vector<float> normalData(static_cast<size_t>(3 * vertices));
      int currentTexCoordIndex = 0;
      std::vector<float> texCoordData(static_cast<size_t>(2 * vertices));
      int currentIndexIndex = 0;
      std::vector<unsigned int> indexData(static_cast<size_t>(3 * faces));

      for (size_t j = 0; j < mesh->mNumVertices; ++j) {
        // extract the coordinate
        const aiVector3D vertice = transform * mesh->mVertices[j];
        coordData[currentCoordIndex++] = vertice[0];
        coordData[currentCoordIndex++] = vertice[1];
        coordData[currentCoordIndex++] = vertice[2];
        // extract the normal
        const aiVector3D normal = transform * mesh->mNormals[j];
        normalData[currentNormalIndex++] = normal[0];
        normalData[currentNormalIndex++] = normal[1];
        normalData[currentNormalIndex++] = normal[2];
        // extract the texture coordinate
        if (mesh->HasTextureCoords(0)) {
          texCoordData[currentTexCoordIndex++] = mesh->mTextureCoords[0][j].x;
          texCoordData[currentTexCoordIndex++] = mesh->mTextureCoords[0][j].y;
        } else {
          texCoordData[currentTexCoordIndex++] = 0.5;
          texCoordData[currentTexCoordIndex++] = 0.5;
        }
      }

      // create the index array
      for (size_t j = 0; j < mesh->mNumFaces; ++j) {
        const aiFace face = mesh->mFaces[j];
        if (face.mNumIndices < 3)  // we want to skip lines
          continue;
        assert(face.mNumIndices == 3);
        indexData[currentIndexIndex++] = face.mIndices[0];
        indexData[currentIndexIndex++] = face.mIndices[1];
        indexData[currentIndexIndex++] = face.mIndices[2];
      }

      if (currentIndexIndex == 0)  // if all faces turned out to be invalid, ignore the mesh
        continue;

      // WREN-retirement W1b (unconditional since D1.4): RETAIN the wgpu-ready copy of this
      // submesh. The wgpu renderer used to recover these bytes by reading WREN's GL buffers
      // back; the interleave below is byte-identical to what that readback produced.
      // Index-parallel to mPbrAppearances, pushed in the same iteration so the two can never
      // drift.
      {
        WgpuSubmeshData sub;
        sub.vertexBytes.resize(static_cast<size_t>(vertices) * kCadWgpuStride);
        for (int v = 0; v < vertices; ++v) {
          unsigned char *const dst = sub.vertexBytes.data() + static_cast<size_t>(v) * kCadWgpuStride;
          std::memcpy(dst + 0, &coordData[3 * v], 12);
          std::memcpy(dst + 12, &normalData[3 * v], 12);
          std::memcpy(dst + 24, &texCoordData[2 * v], 8);
        }
        sub.indices.assign(indexData.begin(), indexData.begin() + currentIndexIndex);
        // CONTENT key (see OmCadShape.hpp): the counts are folded in first so two submeshes whose
        // concatenated payloads happen to coincide at different splits cannot collide. Bit 63 is
        // set last — user-space heap pointers are canonical 47-bit, so the tag can never alias the
        // pointer keys OmWgpuMeshCache's other callers pass.
        const uint64_t counts[2] = {static_cast<uint64_t>(vertices), static_cast<uint64_t>(currentIndexIndex)};
        uint64_t hash = fnv1a64(counts, sizeof(counts), kFnv1a64Offset);
        hash = fnv1a64(sub.vertexBytes.data(), sub.vertexBytes.size(), hash);
        hash = fnv1a64(sub.indices.data(), sub.indices.size() * sizeof(unsigned int), hash);
        sub.contentKey = hash | (static_cast<uint64_t>(1) << 63);
        mWgpuSubmeshes.push_back(std::move(sub));
      }

      // retrieve material properties
      const aiMaterial *material = scene->mMaterials[mesh->mMaterialIndex];

      // determine how image textures referenced in the collada/wavefront file will be searched for
      const QString &referenceUrl = OmUrl::computePath(this, "url", mUrl->item(0));

      // init from assimp material
      OmNode *previousParent = OmNode::globalParentNode();
      OmNode::setGlobalParentNode(this);
      OmPbrAppearance *pbrAppearance = new OmPbrAppearance(material, referenceUrl);
      OmNode::setGlobalParentNode(previousParent);
      pbrAppearance->preFinalize();
      pbrAppearance->postFinalize();
      connect(pbrAppearance, &OmPbrAppearance::changed, this, &OmCadShape::updateAppearance);

      if (pbrAppearance->transparency() > 0.999)
        warn(tr("Mesh '%1' created but it is fully transparent.").arg(mesh->mName.C_Str()));

      mPbrAppearances.push_back(pbrAppearance);
    }
  }
}

void OmCadShape::updateAppearance() {
  // D1.4: no WREN materials to rebuild -- the wgpu path reads each submesh's
  // OmPbrAppearance node (wgpuAppearance(i)) live at collect time.
}

void OmCadShape::deleteWrenObjects() {
  // D1.4: only the engine-side submesh state remains to tear down.
  for (OmPbrAppearance *appearance : mPbrAppearances)
    delete appearance;
  mPbrAppearances.clear();

  // W1b: the retained wgpu copies are index-parallel to mPbrAppearances, so they die with
  // it. The renderer's cached draw list is invalidated through the node's destroyed()
  // signal, and wgpuNativeSubmesh() declines an out-of-range index in the meantime.
  mWgpuSubmeshes.clear();
}

void OmCadShape::recomputeBoundingSphere() const {
  assert(mBoundingSphere);
  mBoundingSphere->empty();

  // D1.4: computed from the retained submesh vertices instead of asking WREN's static mesh
  // for its bounding sphere. Construction: the submesh's box centre, with the radius of the
  // farthest vertex from it -- always a valid enclosing sphere (at worst slightly larger
  // than a minimal one), which is what OmBoundingSphere consumers require.
  for (const WgpuSubmeshData &sub : mWgpuSubmeshes) {
    const size_t n = sub.vertexBytes.size() / kCadWgpuStride;
    if (n == 0)
      continue;
    float lo[3], hi[3];
    for (size_t v = 0; v < n; ++v) {
      float pos[3];
      std::memcpy(pos, sub.vertexBytes.data() + v * kCadWgpuStride, 12);
      if (v == 0) {
        for (int c = 0; c < 3; ++c) {
          lo[c] = pos[c];
          hi[c] = pos[c];
        }
        continue;
      }
      for (int c = 0; c < 3; ++c) {
        if (pos[c] < lo[c])
          lo[c] = pos[c];
        if (pos[c] > hi[c])
          hi[c] = pos[c];
      }
    }
    const OmVector3 center(0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1]), 0.5 * (lo[2] + hi[2]));
    double radiusSq = 0.0;
    for (size_t v = 0; v < n; ++v) {
      float pos[3];
      std::memcpy(pos, sub.vertexBytes.data() + v * kCadWgpuStride, 12);
      const double dx = pos[0] - center.x();
      const double dy = pos[1] - center.y();
      const double dz = pos[2] - center.z();
      const double dSq = dx * dx + dy * dy + dz * dz;
      if (dSq > radiusSq)
        radiusSq = dSq;
    }
    const OmBoundingSphere meshBoundingSphere(NULL, center, std::sqrt(radiusSq));
    mBoundingSphere->enclose(&meshBoundingSphere);
  }
}

const OmVector3 OmCadShape::absoluteScale() const {
  const OmTransform *const up = upperTransform();
  return up ? up->absoluteScale() : OmVector3(1.0, 1.0, 1.0);
}

void OmCadShape::exportNodeFields(OmWriter &writer) const {
  OmBaseNode::exportNodeFields(writer);

  if (mUrl->size() == 0)
    return;

  if (!(writer.isW3d() || writer.isProto())) {
    findField("url", true)->write(writer);
    return;
  }

  // export model
  OmField urlFieldCopy(*findField("url", true));
  for (int i = 0; i < mUrl->size(); ++i) {
    const QString &resolvedURL = OmUrl::computePath(this, "url", mUrl, i);
    OmMFString *urlFieldValue = dynamic_cast<OmMFString *>(urlFieldCopy.value());
    urlFieldValue->setItem(i, exportResource(mUrl->item(i), resolvedURL, writer.relativeMeshesPath(), writer));
  }

  // export materials
  if (writer.isW3d()) {  // only needs to be included in the w3d, when converting to base node it shouldn't be included
    const QString &parentUrl = OmUrl::computePath(this, "url", mUrl, 0);
    for (const QString &material : objMaterialList(parentUrl)) {
      QString materialUrl = OmUrl::combinePaths(material, parentUrl);
      OmMFString *urlFieldValue = dynamic_cast<OmMFString *>(urlFieldCopy.value());
      urlFieldValue->addItem(exportResource(material, materialUrl, writer.relativeMeshesPath(), writer));
    }
  }

  // if it's an animation or a scene, export the textures to the 'textures' folder
  for (int i = 0; i < mPbrAppearances.size(); ++i)
    mPbrAppearances[i]->exportShallowNode(writer);

  urlFieldCopy.write(writer);
}

QStringList OmCadShape::customExportedFields() const {
  QStringList fields;
  fields << "url";
  return fields;
}

QString OmCadShape::cadPath() const {
  return OmUrl::computePath(this, "url", mUrl, 0);
}

QStringList OmCadShape::fieldsToSynchronizeWithW3d() const {
  QStringList fields;
  fields << "ccw"
         << "castShadows"
         << "isPickable"
         << "url";
  return fields;
}
