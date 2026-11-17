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

#ifndef OM_CAD_SHAPE_HPP
#define OM_CAD_SHAPE_HPP

#include "OmBaseNode.hpp"

#include <QtCore/QMap>

#include <assimp/material.h>

#include <cstddef>
#include <cstdint>
#include <vector>

class OmBoundingSphere;
class OmMFString;
class OmDownloader;
class OmMatrix4;
class OmPbrAppearance;
class OmRgb;

struct aiMaterial;

class OmCadShape : public OmBaseNode {
  Q_OBJECT

public:
  explicit OmCadShape(OmTokenizer *tokenizer = NULL);
  OmCadShape(const OmCadShape &other);
  explicit OmCadShape(const OmNode &other);
  virtual ~OmCadShape() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_CAD_SHAPE; }
  void downloadAssets() override;
  void preFinalize() override;
  void postFinalize() override;
  void updateSegmentationColor(const OmRgb &color) override { setSegmentationColor(color); }

  const OmVector3 absoluteScale() const;

  // R4 wgpu collection: expose the CAD submeshes (a .glb/.dae/.obj imports as N submeshes, each
  // retained as a wgpu-ready CPU copy + a PBRAppearance) so OmWgpuSceneRenderer can draw
  // URDF-imported robots — a mesh-visual arm ships CadShape visuals, not OmShape, and was
  // otherwise skipped by collectShapeDraws entirely (the "robot doesn't render" gap). D1.4: the
  // WREN-object accessors (wgpuMesh/wgpuTransform) and the OMNISIM_WGPU_NATIVE_CADSHAPE hatch are
  // gone — wgpuNativeSubmesh() + wgpuWorldMatrix() below are the only, unconditional path.
  int wgpuMeshCount() const { return static_cast<int>(mWgpuSubmeshes.size()); }
  OmPbrAppearance *wgpuAppearance(int i) const { return mPbrAppearances.value(i, nullptr); }
  bool wgpuCastShadows() const;  // castShadows field (defined in .cpp; OmSFBool is incomplete here)

  // ---- WREN-retirement W1b: WREN-free submesh geometry + world matrix ----
  //
  // The wgpu renderer used to recover these submeshes by reading WREN's GL buffers back and to
  // take their world matrix from the WREN transform. Both are served from the engine side:
  // createWrenObjects() RETAINS the arrays it already builds, and the world matrix comes from
  // the node's own upperPose()->matrix().
  //
  // `vertexBytes` is the interleaved 32-byte-stride layout OmWgpuMeshCache::acquire() wants
  // (pos3 @0, norm3 @12, uv2 @24) — byte-identical to what the legacy WREN readback produced.
  //
  // `contentKey` is a 64-bit FNV-1a hash of (counts + vertex bytes + index bytes) with bit 63 set
  // so it can never alias a heap-pointer key (every other acquire() caller passes a pointer).
  // Content-keying is LOAD-BEARING, not cosmetic: two CadShapes on the same .dae share ONE
  // content key and therefore ONE wgpu upload. A per-node key would multiply GPU mesh memory by
  // the instance count.
  struct WgpuSubmesh {
    const unsigned char *vertexBytes = nullptr;  // interleaved pos3 + norm3 + uv2, 32-byte stride
    size_t vertexBytesLen = 0;
    const unsigned int *indices = nullptr;  // uint32 triangle list
    size_t indexBytesLen = 0;
    unsigned int indexCount = 0;
    uint64_t contentKey = 0;  // OmWgpuMeshCache key: FNV-1a content hash, bit 63 set
  };
  bool wgpuNativeSubmesh(int i, WgpuSubmesh &out) const;  // false => nothing to draw at that index
  bool wgpuWorldMatrix(OmMatrix4 &out) const;

  // D1.4: permanently a no-op. This diagnostic verified the pose-matrix substitution against
  // WREN's own per-submesh transform matrices; the equality was established numerically before
  // those transforms were deleted. Kept so callers need no change.
  void wgpuAuditMatrices() const;

  QStringList fieldsToSynchronizeWithW3d() const override;

protected:
  void exportNodeFields(OmWriter &writer) const override;
  QStringList customExportedFields() const override;
  OmBoundingSphere *boundingSphere() const override { return mBoundingSphere; }
  void recomputeBoundingSphere() const;

private slots:
  void updateUrl();
  void updateCcw();
  void updateCastShadows();
  void updateIsPickable();
  void updateAppearance();

  void downloadUpdate();
  void materialDownloadTracker();

private:
  OmCadShape &operator=(const OmCadShape &);  // non copyable
  OmNode *clone() const override { return new OmCadShape(*this); }
  void init();

  OmDownloader *mDownloader;
  mutable OmBoundingSphere *mBoundingSphere;

  // node fields
  OmMFString *mUrl;
  OmSFBool *mCcw;
  OmSFBool *mCastShadows;
  OmSFBool *mIsPickable;

  // per-submesh appearance nodes, index-parallel to mWgpuSubmeshes
  QVector<OmPbrAppearance *> mPbrAppearances;

  // W1b: CPU-side wgpu-ready copy of each submesh (unconditional since D1.4).
  struct WgpuSubmeshData {
    std::vector<unsigned char> vertexBytes;  // 32-byte stride: pos3 + norm3 + uv2
    std::vector<unsigned int> indices;
    uint64_t contentKey = 0;
  };
  std::vector<WgpuSubmeshData> mWgpuSubmeshes;

  // methods and variables to handle obj materials
  QMap<QString, QString> mObjMaterials;  // maps materials as referenced in the obj to their remote counterpart
  QVector<OmDownloader *> mMaterialDownloaders;
  QStringList objMaterialList(const QString &url) const;
  bool areMaterialAssetsAvailable(const QString &url);
  void retrieveMaterials();

  const QString vrmlPbrAppearance(const aiMaterial *material);
  bool addTextureMap(QString &vrml, const aiMaterial *material, const QString &mapName, aiTextureType textureType);
  QString cadPath() const;

  void setSegmentationColor(const OmRgb &color);

  void createWrenObjects() override;
  void deleteWrenObjects();
};

#endif
