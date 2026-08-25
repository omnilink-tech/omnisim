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

#include "OmPaintTexture.hpp"

#include "OmAppearance.hpp"
#include "OmBoundingSphere.hpp"
#include "OmGeometry.hpp"
#include "OmImageTexture.hpp"
#include "OmIndexedFaceSet.hpp"
#include "OmLog.hpp"
#include "OmMathsUtilities.hpp"
#include "OmPbrAppearance.hpp"
#include "OmPerformanceLog.hpp"
#include "OmRgb.hpp"
#include "OmShape.hpp"
#include "OmWorld.hpp"

static QList<OmPaintTexture *> gPaintTextures;
static QList<OmPaintTexture *> gEvaporationTextures;

// Min & max texture sizes (when no texture is attached to material, size has to be computed)
static const OmVector2 MAX_TEXTURE_SIZE(1024, 1024);
static const OmVector2 MIN_TEXTURE_SIZE(32, 32);

// W3/P3 non-WREN cache key. Bit 62 is set so a paint-layer id can never collide with the two
// other key spaces that share OmWgpuTextureCache: OmWgpuSceneRenderer::stableTexId's hashed
// OmImageTexture keys and OmWgpuMeshAdapter's primitive tags. The counter is monotonic and never
// reused, so a world reload cannot hand a stale cache entry to a new paint layer the way a raw
// `this` pointer could.
static const uint64_t kWgpuIdTag = 0x4000000000000000ull;
static uint64_t gWgpuIdCounter = 0;

bool OmPaintTexture::hasAny() {
  return !gPaintTextures.isEmpty();
}

OmPaintTexture *OmPaintTexture::findPaintTexture(const OmShape *shape) {
  foreach (OmPaintTexture *texture, gPaintTextures) {
    if (texture->mShape == shape)
      return texture;
  }

  return NULL;
}

OmPaintTexture *OmPaintTexture::paintTexture(const OmShape *shape) {
  OmPaintTexture *paintTexture = findPaintTexture(shape);
  if (paintTexture)
    return paintTexture;  // already exists

  // check if it is possible to paint on the shape
  if (isPaintable(shape))
    return new OmPaintTexture(shape);

  return NULL;
}

bool OmPaintTexture::isPaintable(const OmShape *shape) {
  return shape->geometry();
}

void OmPaintTexture::init() {
  cleanup();
}

void OmPaintTexture::cleanup() {
  QList<OmPaintTexture *>::iterator it = gPaintTextures.begin();
  while (it != gPaintTextures.end()) {
    delete *it;
    ++it;
  }

  gPaintTextures.clear();
  gEvaporationTextures.clear();
}

void OmPaintTexture::clearAllTextures() {
  foreach (OmPaintTexture *const pt, gPaintTextures)
    pt->clearTexture();
  gEvaporationTextures.clear();
}

void OmPaintTexture::clearTexture() {
  int size = mTextureSize.x() * mTextureSize.y();

  for (int i = 0; i < size; i++) {
    int k = 4 * i;
    mData[k] = 1.0f;
    mData[++k] = 1.0f;
    mData[++k] = 1.0f;
    mData[++k] = 0.0f;
  }
  ++mWgpuRevision;  // W3/P3: mData changed -> the renderer's mirror is stale

  if (mEvaporation)
    memset(mEvaporation, 0, size * sizeof(double));
}

OmPaintTexture::OmPaintTexture(const OmShape *shape)
  : mShape(shape),
    mEvaporation(NULL),
    mWgpuId(kWgpuIdTag | ++gWgpuIdCounter),
    mWgpuRevision(1),
    mWgpuPackedRevision(0) {
  // add painting layer. D1.4: the size still follows the appearance's own texture when one is
  // loaded (hasImage() replaces the old `wrenTexture() != NULL` load test); the WREN drawable
  // texture and the material bind are gone -- the wgpu renderer resolves this layer via
  // wgpuRgba8() and binds it itself.
  const OmAppearance *appearance = shape->appearance();
  const OmPbrAppearance *pbrAppearance = shape->pbrAppearance();
  if (appearance && appearance->texture() && appearance->texture()->hasImage())
    mTextureSize = computeTextureSize(appearance->texture()->width(), appearance->texture()->height());
  else if (pbrAppearance && pbrAppearance->baseColorMap() && pbrAppearance->baseColorMap()->hasImage())
    mTextureSize = computeTextureSize(pbrAppearance->baseColorMap()->width(), pbrAppearance->baseColorMap()->height());
  else {
    const OmVector2 size = computeDefaultTextureSize();
    mTextureSize = computeTextureSize(size.x(), size.y());
  }

  // initialize paint texture
  int size = mTextureSize.x() * mTextureSize.y();
  mData = new float[4 * size];
  for (int i = 0; i < size; i++) {
    int k = 4 * i;
    mData[k] = 1.0f;
    mData[++k] = 1.0f;
    mData[++k] = 1.0f;
    mData[++k] = 0.0f;
  }

  // initialize evaporations
  if (OmWorld::instance()->worldInfo()->inkEvaporation() > 0.0) {
    mEvaporation = new double[size];
    memset(mEvaporation, 0, size * sizeof(double));
  }

  gPaintTextures.append(this);
}

OmPaintTexture::~OmPaintTexture() {
  delete[] mData;
  if (mEvaporation)
    delete[] mEvaporation;
}

void OmPaintTexture::prePhysicsStep(double ms) {
  const OmWorldInfo *wi = OmWorld::instance()->worldInfo();
  double ie = wi->inkEvaporation();

  if (ie) {
    double factor = exp(-ie * ms / 1000.0);
    for (int i = gEvaporationTextures.size() - 1; i >= 0; --i) {
      bool isDone = true;
      gEvaporationTextures[i]->evaporateInk(factor, isDone);
      if (isDone)
        gEvaporationTextures.removeAt(i);
    }
  }
}

void OmPaintTexture::evaporateInk(double evaporationFactor, bool &isDone) {
  assert(mEvaporation);

  const int w = mTextureSize.x();
  const int h = mTextureSize.y();

  isDone = true;
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const int n = y * w + x;
      const int m = 4 * n;
      if (mEvaporation[n] > 0.001) {
        mEvaporation[n] *= evaporationFactor;
        mData[m + 3] = static_cast<float>(mEvaporation[n]);
        isDone = false;
      }
    }
  }
  if (!isDone)
    ++mWgpuRevision;  // W3/P3: one bump per evaporation pass, not per pixel
}

void OmPaintTexture::paint(const OmRay &ray, float leadSize, const OmRgb &color, float density) {
  OmVector2 uv;
  bool validCollision = mShape->geometry()->pickUVCoordinate(uv, ray, 1);
  if (!validCollision)
    return;  // no valid collision

  if (uv.x() > 1.0 || uv.x() < 0.0 || uv.y() > 1.0 || uv.y() < 0.0)
    return;  // outside

  const int w = mTextureSize.x();
  const int h = mTextureSize.y();
  const int x = uv.x() * w;
  const int y = uv.y() * h;

  OmBoundingSphere *bs = mShape->geometry()->boundingSphere();
  bs->recomputeIfNeeded();
  int size = static_cast<int>(leadSize * mOriginalTextureSize.x() / bs->scaledRadius());
  if (size < 1)
    size = 1;

  const int halfSize = size / 2;

  int ox = x - halfSize;
  int oy = y - halfSize;
  if (ox < 0)
    ox = 0;

  if (oy < 0)
    oy = 0;

  int sx = x + halfSize;
  int sy = y + halfSize;
  if (sx >= w)
    sx = w - 1;

  if (sy >= h)
    sy = h - 1;

  if (mEvaporation && !gEvaporationTextures.contains(this))
    gEvaporationTextures.append(this);

  const int halfSize2 = (halfSize * halfSize);
  for (int ty = oy; ty <= sy; ++ty) {
    int rowOffset = ty * w;
    int dataIndex = (rowOffset + ox) << 2;
    int circleCondition = halfSize2 - (ty - y) * (ty - y);
    for (int tx = ox; tx <= sx; ++tx) {
      if ((tx - x) * (tx - x) <= circleCondition) {
        const float previousDensity = mData[dataIndex + 3];
        const float oldDensityRatio = density + previousDensity < 1e-15 ? 0.0f : previousDensity / (density + previousDensity);
        mData[dataIndex] = oldDensityRatio * mData[dataIndex] + (1.0f - oldDensityRatio) * color.blue();
        mData[dataIndex + 1] = oldDensityRatio * mData[dataIndex + 1] + (1.0f - oldDensityRatio) * color.green();
        mData[dataIndex + 2] = oldDensityRatio * mData[dataIndex + 2] + (1.0f - oldDensityRatio) * color.red();
        mData[dataIndex + 3] += density;
        if (mData[dataIndex + 3] > 1.0f)
          mData[dataIndex + 3] = 1.0f;

        if (mEvaporation) {
          mEvaporation[rowOffset + tx] += (double)density;
          if (mEvaporation[rowOffset + tx] > 1.0f)
            mEvaporation[rowOffset + tx] = 1.0f;
        }
      }

      dataIndex += 4;  // compute index of the next pixel
    }
  }
  ++mWgpuRevision;  // W3/P3: ink landed -> the renderer's mirror is stale
}

void OmPaintTexture::pickColor(const OmVector2 &uv, OmRgb &pickedColor, float *pickedDensity) const {
  const int w = mTextureSize.x();
  const int h = mTextureSize.y();
  int x = uv.x() * w;
  int y = uv.y() * h;
  while (x < 0)
    x += w;
  while (y < 0)
    y += h;
  while (x >= w)
    x -= w;
  while (y >= h)
    y -= h;

  const int index = (y * w + x) * 4;
  pickedColor.setValue(mData[index + 2], mData[index + 1], mData[index]);
  if (pickedDensity != NULL)
    *pickedDensity = mData[index + 3];
}

// W3/P3: float BGRA -> tightly packed RGBA8, repacked only when the ink actually changed.
//
// ROW ORDER IS DELIBERATELY UNCHANGED, and that is a measurement, not an assumption. paint()
// indexes mData by `y = uv.y() * h` with uv from OmGeometry::pickUVCoordinate, whose v convention
// (OmPlane.cpp:280, `v = -y/sy + 0.5`) puts v=0 at local +Y -- exactly where
// OmWgpuMeshAdapter::buildUnitRectangle puts v=0. wgpu texture row 0 IS v=0, so a straight
// row-order copy lands the ink where the pen wrote it. (WREN needs the opposite because
// wren::StaticMesh::createQuad's texCoord runs the other way and GL's texel origin cancels it.)
const std::vector<unsigned char> &OmPaintTexture::wgpuRgba8() {
  const int w = static_cast<int>(mTextureSize.x());
  const int h = static_cast<int>(mTextureSize.y());
  if (w <= 0 || h <= 0 || mData == NULL) {
    mRgba8.clear();
    return mRgba8;
  }
  if (mWgpuPackedRevision == mWgpuRevision && mRgba8.size() == static_cast<size_t>(w) * h * 4u)
    return mRgba8;  // unchanged since the last pack

  mRgba8.resize(static_cast<size_t>(w) * static_cast<size_t>(h) * 4u);
  const size_t n = static_cast<size_t>(w) * static_cast<size_t>(h);
  for (size_t i = 0; i < n; ++i) {
    const float *src = &mData[i * 4];  // BGRA, see paint()
    unsigned char *dst = &mRgba8[i * 4];
    const float v[4] = {src[2], src[1], src[0], src[3]};
    for (int c = 0; c < 4; ++c) {
      const float f = v[c] < 0.0f ? 0.0f : (v[c] > 1.0f ? 1.0f : v[c]);
      dst[c] = static_cast<unsigned char>(f * 255.0f + 0.5f);
    }
  }
  mWgpuPackedRevision = mWgpuRevision;
  return mRgba8;
}

OmVector2 OmPaintTexture::computeTextureSize(int imageTextureWidth, int imageTextureHeight) {
  const int width = imageTextureWidth;
  const int height = imageTextureHeight;
  mOriginalTextureSize.setXy(imageTextureWidth, imageTextureHeight);

  const OmVector2 sizeFactor = mShape->geometry()->nonRecursiveTextureSizeFactor();
  return OmVector2(width * sizeFactor.x(), height * sizeFactor.y());
}

OmVector2 OmPaintTexture::computeDefaultTextureSize() {
  // Get max scale factor
  const OmVector3 scale = mShape->geometry()->absoluteScale();
  double maxScale = scale.x() > scale.y() ? scale.x() : scale.y();
  if (scale.z() > maxScale)
    maxScale = scale.z();

  // Compute size based on bounding sphere radius
  // If the sphere radius is greater than 10 (arbitrary value), then we use the max size
  const float sphereRadius = maxScale * mShape->geometry()->boundingSphere()->scaledRadius();
  const OmVector2 size =
    sphereRadius > 10.0 ? MAX_TEXTURE_SIZE : MIN_TEXTURE_SIZE + (MAX_TEXTURE_SIZE - MIN_TEXTURE_SIZE) * sphereRadius / 10.0;

  // Return closest power of two dimensions
  const OmVector2 nextPowerOf2Size(OmMathsUtilities::nextPowerOf2(size.x()), OmMathsUtilities::nextPowerOf2(size.y()));
  const OmVector2 previousPowerOf2Size = nextPowerOf2Size / 2;

  if (abs(nextPowerOf2Size.x() - size.x()) > abs(previousPowerOf2Size.x() - size.x()))
    return previousPowerOf2Size;
  else
    return nextPowerOf2Size;
}
