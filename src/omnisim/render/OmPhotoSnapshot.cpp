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
#include "OmPhotoSnapshot.hpp"
#include "OmWgpuImageAdapter.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>

OmPhotoScene omPhotoSnapshot(const std::vector<OmWgpuSolidDraw> &draws) {
  OmPhotoScene scene;
  std::map<uint64_t, int> textures;
  auto copyTexture = [&](const QImage *image) {
    if (!image) return -1;
    // Different nodes can own QImage wrappers sharing the same decoded pixels.
    // Deduplicate their storage identity, not the wrapper's address.
    const uint64_t key = OmWgpuImageAdapter::photoTextureKey(image);
    const auto known = textures.find(key);
    if (known != textures.end()) return known->second;
    OmPhotoTexture texture;
    if (!OmWgpuImageAdapter::copyPhotoTexture(image, texture)) return -1;
    const int id = static_cast<int>(scene.textures.size());
    textures[key] = id;
    scene.textures.push_back(std::move(texture));
    return id;
  };
  auto linear = [](float c) { return c <= 0.04045f ? c / 12.92f : std::pow((c + 0.055f) / 1.055f, 2.4f); };
  for (const auto &draw : draws) {
    if (!draw.cpuPositions || !draw.cpuIndices || !draw.modelMatrix16 || (draw.baseColorA <= 0 && !draw.photoRefraction))
      continue;
    OmPhotoMaterial material;
    material.color[0] = linear(draw.baseColorR);
    material.color[1] = linear(draw.baseColorG);
    material.color[2] = linear(draw.baseColorB);
    material.emission[0] = std::max(0.0f, draw.emissiveR);
    material.emission[1] = std::max(0.0f, draw.emissiveG);
    material.emission[2] = std::max(0.0f, draw.emissiveB);
    material.opacity = draw.baseColorA;
    material.roughness = 1 - draw.specularStrength;
    material.metalness = draw.photoMetalness;
    material.normalStrength = draw.photoNormalStrength;
    material.castShadows = draw.castShadows;
    material.refraction = draw.photoRefraction;
    material.emissiveTwoSided = draw.emissiveTwoSided;
    material.indexOfRefraction = draw.photoIor;
    material.attenuationDistance = draw.photoAttenuationDistance;
    std::copy(draw.photoAttenuationColor,draw.photoAttenuationColor+3,material.attenuationColor);
    material.albedoMap = copyTexture(draw.photoMaps[0]);
    material.roughnessMap = copyTexture(draw.photoMaps[1]);
    material.metalnessMap = copyTexture(draw.photoMaps[2]);
    material.normalMap = copyTexture(draw.photoMaps[3]);
    const uint32_t materialId = static_cast<uint32_t>(scene.materials.size());
    scene.materials.push_back(material);
    const auto &positions = *draw.cpuPositions;
    const auto &indices = *draw.cpuIndices;
    const auto *attributes = draw.cpuAttributes;
    const size_t vertices = positions.size() / 3;
    const float *matrix = draw.modelMatrix16;
    float normal[9];
    // Columns of the inverse transpose are cross products of the other two
    // columns. Unlike multiplying by the model matrix, this handles nonuniform scale.
    for (int c = 0; c < 3; ++c) {
      const float *a = matrix + ((c + 1) % 3) * 4, *b = matrix + ((c + 2) % 3) * 4;
      normal[c*3] = a[1]*b[2]-a[2]*b[1];
      normal[c*3+1] = a[2]*b[0]-a[0]*b[2];
      normal[c*3+2] = a[0]*b[1]-a[1]*b[0];
    }
    const float determinant = matrix[0]*normal[0]+matrix[1]*normal[1]+matrix[2]*normal[2];
    if (std::abs(determinant) > 1e-12f)
      for (float &v : normal) v /= determinant;
    for (size_t i = 0; i + 2 < indices.size(); i += 3) {
      if (indices[i] >= vertices || indices[i + 1] >= vertices || indices[i + 2] >= vertices) continue;
      OmniLightTriangle triangle;
      triangle.material = materialId;
      OmPhotoAttributes attr;
      float *out[3] = {triangle.v0, triangle.v1, triangle.v2};
      for (int corner = 0; corner < 3; ++corner) {
        const size_t id = indices[i + corner];
        for (int r = 0; r < 3; ++r) {
          out[corner][r] = matrix[12 + r];
          for (int c = 0; c < 3; ++c) out[corner][r] += matrix[c*4+r] * positions[id * 3 + c];
        }
        if (attributes && attributes->size() >= (id + 1) * 5) {
          const float *a = attributes->data() + id * 5;
          for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) attr.normals[corner * 3 + r] += normal[c*3+r] * a[c];
          attr.uv[corner * 2] = draw.uvA[0] * a[3] + draw.uvA[1] * a[4] + draw.uvB[0];
          attr.uv[corner * 2 + 1] = draw.uvA[2] * a[3] + draw.uvA[3] * a[4] + draw.uvB[1];
        }
      }
      scene.triangles.push_back(triangle);
      scene.attributes.push_back(attr);
    }
  }
  return scene;
}
