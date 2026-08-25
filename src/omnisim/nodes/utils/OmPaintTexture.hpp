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

#ifndef OM_PAINT_TEXTURE_HPP
#define OM_PAINT_TEXTURE_HPP

#include <QtCore/QObject>

#include <cstdint>
#include <vector>

#include "OmVector2.hpp"

class OmRay;
class OmRgb;
class OmShape;

// this class is used to simulate texture painting with the Pen device and ink evaporation
class OmPaintTexture : public QObject {
  Q_OBJECT

public:
  explicit OmPaintTexture(const OmShape *shape);
  ~OmPaintTexture();

  void paint(const OmRay &ray, float leadSize, const OmRgb &color, float density);

  const OmShape *shape() { return mShape; }

  static bool isPaintable(const OmShape *shape);

  void pickColor(const OmVector2 &uv, OmRgb &pickedColor, float *pickedDensity = NULL) const;
  void clearTexture();

  // simulate ink evaporation
  static void prePhysicsStep(double ms);

  // get paint texture, create if not found
  static OmPaintTexture *paintTexture(const OmShape *shape);

  // find existing paint texture, return NULL if not found
  static OmPaintTexture *findPaintTexture(const OmShape *shape);

  // initialize list of paint textures
  static void init();

  // free all paint textures and associated memory
  static void cleanup();

  static void clearAllTextures();

  // ---- W3/P3: the paint layer, as the wgpu renderer reads it --------------------------------
  //
  // The ink lives CPU-side as a float BGRA array (`mData`), written by paint() and by the
  // evaporation pass, so a wgpu upload needs NO GPU readback -- only a float->RGBA8 repack.
  // These accessors are the whole render surface (the WREN drawable texture is gone -- D1.4);
  // the CPU read path (pickColor) is untouched by construction.
  //
  // `wgpuRevision()` increments on every write to `mData`. A consumer that caches the upload
  // compares it against the revision it last uploaded, so an idle Pen costs zero GPU traffic and
  // N cameras + the main view each keep their own cache honest without talking to each other.

  // True when ANY paint texture exists. A world with no Pen never allocates one, so this is the
  // one test a collector needs before doing any per-shape work at all.
  static bool hasAny();

  // Stable, collision-free cache key for this paint layer (see kWgpuIdTag in the .cpp).
  uint64_t wgpuTextureId() const { return mWgpuId; }
  int wgpuWidth() const { return static_cast<int>(mTextureSize.x()); }
  int wgpuHeight() const { return static_cast<int>(mTextureSize.y()); }
  uint64_t wgpuRevision() const { return mWgpuRevision; }

  // Tightly packed width*height*4 RGBA8 bytes, repacked from mData on first call after a write
  // (mData is stored BGRA -- see paint(), which writes blue into component 0). Empty only if the
  // texture size is degenerate.
  const std::vector<unsigned char> &wgpuRgba8();

private:
  const OmShape *mShape;
  float *mData;          // texture data
  double *mEvaporation;  // texture evaporation
  OmVector2 mOriginalTextureSize;
  OmVector2 mTextureSize;
  uint64_t mWgpuId;                   // stable cache key (see kWgpuIdTag)
  uint64_t mWgpuRevision;             // bumped on every mData write
  uint64_t mWgpuPackedRevision;       // revision mRgba8 was packed from (0 = never)
  std::vector<unsigned char> mRgba8;  // lazily repacked RGBA8 mirror of mData

  OmVector2 computeTextureSize(int imageTextureWidth, int imageTextureHeight);
  OmVector2 computeDefaultTextureSize();

  // static functions for managind evaporation
  void evaporateInk(double evaporationFactor, bool &isDone);
};

#endif
