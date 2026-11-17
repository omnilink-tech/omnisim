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
#ifndef OM_PHOTO_HPP
#define OM_PHOTO_HPP

#include "OmniLight.hpp"
#include <array>
#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

// Immutable, CPU-owned scene snapshot. No Qt objects, GPU handles or live nodes
// cross into the renderer; changing/reloading a world cannot invalidate a render.
struct OmPhotoTexture {
  int width = 0, height = 0;
  std::vector<uint8_t> rgba;
};
struct OmPhotoMaterial {
  float color[3] = {0.5f, 0.5f, 0.5f};  // linear light
  float emission[3] = {0, 0, 0};
  float roughness = 0.5f, metalness = 0, opacity = 1, normalStrength = 1;
  int albedoMap = -1, roughnessMap = -1, metalnessMap = -1, normalMap = -1;
  bool castShadows = true;
  bool emissiveTwoSided = true;
  // Opt-in solid dielectric. Volume thickness comes from closed geometry.
  bool refraction = false;
  float indexOfRefraction = 1.5f;
  float attenuationColor[3] = {1, 1, 1};
  float attenuationDistance = 1;
};
struct OmPhotoAttributes {
  float normals[9] = {};  // world-space shading normals, one per vertex
  float uv[6] = {};       // authored TextureTransform already applied
};
struct OmPhotoScene {
  std::vector<OmniLightTriangle> triangles;
  std::vector<OmPhotoAttributes> attributes;
  std::vector<OmPhotoMaterial> materials;
  std::vector<OmPhotoTexture> textures;
  std::vector<OmniLightLocal> lights;
  // Lat-long environment in linear radiance, including the authored background.
  int skyWidth = 0, skyHeight = 0;
  std::vector<float> sky;
  float sunTo[3] = {0, 0, 1}, sunEnergy[3] = {0, 0, 0};
  float sunAngularRadius = 0.00465f;
  float eye[3] = {}, forward[3] = {1, 0, 0}, right[3] = {0, -1, 0}, up[3] = {0, 0, 1};
  float horizontalFov = 0.785f, exposure = 1.0f;
};
struct OmPhotoSettings {
  int width = 960, height = 540, samples = 64, bounces = 6;
  // Finite work is a product contract, independent of the agent's local policy.
  double timeLimitSeconds = 60;
  uint64_t seed = 1;
  bool denoise = true;
  bool lightSampling = true;
  bool adaptiveSampling = true;
  float relativeError = 0.035f;
};
struct OmPhotoResult {
  std::vector<float> linearRgb;
  std::vector<float> albedoGuide, normalGuide, depthGuide, variance;
  std::vector<uint32_t> sampleCounts;
  std::vector<float> denoisedRgb;
  std::string denoiser;
  double averageSamples = 0;
  int completedSamples = 0;
  double seconds = 0;
  bool cancelled = false, timeLimited = false;
  std::string error;
};

// Single worker, progressive whole-image samples, deterministic for a fixed seed.
// Cancellation/time limit never publishes a partially completed sample.
OmPhotoResult omRenderPhoto(const OmPhotoScene &scene, const OmPhotoSettings &settings,
                            const std::atomic<bool> &cancel, std::atomic<int> *progress = nullptr);
// AgX display transform; linear HDR pixels remain available in OmPhotoResult.
std::vector<float> omPhotoReduceGrain(const OmPhotoResult &result, int width, int height);
std::vector<uint8_t> omPhotoDisplay(const OmPhotoResult &result, float exposure,
                                   int width = 0, int height = 0);

#endif
