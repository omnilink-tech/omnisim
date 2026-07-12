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

#include "WbWrenAtmosphericSky.hpp"

#include "WbLog.hpp"
#include "WbWrenOpenGlContext.hpp"
#include "WbWrenShaders.hpp"

#include <wren/material.h>
#include <wren/post_processing_effect.h>
#include <wren/shader_program.h>
#include <wren/texture.h>
#include <wren/texture_cubemap.h>
#include <wren/texture_cubemap_baker.h>
#include <wren/texture_rtt.h>

#include <QtCore/QString>

#include <cstring>

// SESSION 2 (transmittance LUT live).
//
// The class hosts a single ``WrPostProcessingEffect`` containing three
// passes — transmittance, multiscattering, sky-view — connected in
// order.  WREN's PPE machinery handles FBO allocation, per-pass
// viewport, and the fullscreen quad render for us; we just bind
// shaders, sizes, and uniforms, then call apply().
//
// Status of each pass's GLSL math (mirrored in
// docs/developer/rendering-roadmap.md §7.2):
//   - sky_transmittance.frag   : implemented (Hillaire §5.1, session 2)
//   - sky_multiscattering.frag : implemented (Hillaire §5.2, session 3)
//   - sky_view.frag            : implemented (Hillaire §5.3 + visual-
//                                parity fix that drops multi-scattering
//                                + ground-bounce contributions)
//   - sky_apply.frag           : implemented (session 4)
//   - sky_to_cubemap.frag      : implemented (session 5 — sky-view LUT
//                                to 128² cubemap for PBR irradiance)
//
// Enabling ``Background.atmosphericSky "mars"`` today gives an end-to-
// end procedural sky AND procedural PBR irradiance, with PBRAppearance
// reflections matching the warm Mars sky.  Earth preset, sensor-camera
// safety, and volumetric fog remain open — see rendering-roadmap.md
// §7.3.

namespace {
  // Hillaire's recommended LUT resolutions.
  constexpr int kTransmittanceWidth = 256;
  constexpr int kTransmittanceHeight = 64;
  constexpr int kMultiScatteringWidth = 32;
  constexpr int kMultiScatteringHeight = 32;
  constexpr int kSkyViewWidth = 192;
  constexpr int kSkyViewHeight = 108;
}  // namespace

WbWrenAtmosphericSky::AtmosphereParameters WbWrenAtmosphericSky::AtmosphereParameters::earth() {
  AtmosphereParameters p;
  p.bottomRadiusKm = 6360.0f;
  p.topRadiusKm = 6460.0f;
  p.rayleighScattering = QVector3D(5.802e-6f, 13.558e-6f, 33.1e-6f);
  p.rayleighDensityExpScale = -1.0f / 8.0f;
  p.mieScattering = QVector3D(3.996e-6f, 3.996e-6f, 3.996e-6f);
  p.mieAbsorption = QVector3D(4.4e-6f, 4.4e-6f, 4.4e-6f);
  p.mieDensityExpScale = -1.0f / 1.2f;
  p.miePhaseG = 0.8f;
  p.absorptionExtinction = QVector3D(0.65e-6f, 1.881e-6f, 0.085e-6f);
  p.groundAlbedo = 0.3f;
  return p;
}

WbWrenAtmosphericSky::AtmosphereParameters WbWrenAtmosphericSky::AtmosphereParameters::mars() {
  AtmosphereParameters p;
  p.bottomRadiusKm = 3389.5f;
  p.topRadiusKm = 3489.5f;
  p.rayleighScattering = QVector3D(19.918e-7f, 13.57e-7f, 5.75e-7f);
  p.rayleighDensityExpScale = -1.0f / 11.1f;
  p.mieScattering = QVector3D(2.0e-5f, 1.7e-5f, 1.4e-5f);
  p.mieAbsorption = QVector3D(1.0e-5f, 1.2e-5f, 1.5e-5f);
  p.mieDensityExpScale = -1.0f / 11.0f;
  p.miePhaseG = 0.76f;
  p.absorptionExtinction = QVector3D(0.15e-6f, 0.3e-6f, 0.85e-6f);
  p.groundAlbedo = 0.15f;
  return p;
}

WbWrenAtmosphericSky::WbWrenAtmosphericSky() :
  mParams(AtmosphereParameters::earth()),
  // CONVENTION: TOWARD sun.  Default = sun overhead = noon.  See
  // setSunDirection() doc in the header.
  mSunDirection(0.0f, 1.0f, 0.0f),
  // Default illuminance MATCHES docs/developer/atmospheric-sky-preview.html
  // exactly so the in-Webots sky is byte-equivalent to the preview the
  // user signed off on.  Earth = (22, 22, 22); Mars uses the
  // dimmer-and-warmer override in setPreset().  Tonemap (preview's
  // filmic) is applied inside sky_apply.frag and survives
  // hdr_resolve via the alpha=0 pass-through path.
  mSunIlluminance(22.0f, 22.0f, 22.0f),
  // 100 m above the planet surface — matches the preview's
  // `origin.y = bottomRadiusKm + 0.1`.  Small bias above ground so
  // looking-down rays don't immediately graze the surface.
  mCameraHeightKm(0.1f),
  mAtmosphereDirty(true),
  mSkyViewDirty(true),
  mBakeEffect(NULL),
  mTransmittancePass(NULL),
  mMultiScatteringPass(NULL),
  mSkyViewPass(NULL) {
  buildBakeEffect();
}

WbWrenAtmosphericSky::~WbWrenAtmosphericSky() {
  destroyBakeEffect();
}

bool WbWrenAtmosphericSky::setPreset(const QString &preset) {
  AtmosphereParameters next = mParams;
  if (preset.isEmpty() || preset == "earth")
    next = AtmosphereParameters::earth();
  else if (preset == "mars")
    next = AtmosphereParameters::mars();
  else
    return false;

  if (memcmp(&next, &mParams, sizeof(AtmosphereParameters)) != 0) {
    mParams = next;
    mAtmosphereDirty = true;
    mSkyViewDirty = true;
    // Per-preset illuminance — matches docs/developer/atmospheric-sky-preview.html
    // exactly.  Mars is dimmer + warmer than Earth at the surface
    // (~43% solar constant plus a warm shift from the dust column).
    if (preset == "mars") {
      mSunIlluminance = QVector3D(10.0f, 8.5f, 7.0f);
    } else {
      mSunIlluminance = QVector3D(22.0f, 22.0f, 22.0f);
    }
  }
  return true;
}

void WbWrenAtmosphericSky::setSunDirection(const QVector3D &direction) {
  const QVector3D normalised = direction.normalized();
  if (normalised != mSunDirection) {
    mSunDirection = normalised;
    mSkyViewDirty = true;
  }
}

void WbWrenAtmosphericSky::setSunIlluminance(const QVector3D &illuminance) {
  if (illuminance != mSunIlluminance) {
    mSunIlluminance = illuminance;
  }
}

void WbWrenAtmosphericSky::setCameraHeightKm(float heightKm) {
  if (heightKm != mCameraHeightKm) {
    mCameraHeightKm = heightKm;
    mSkyViewDirty = true;
  }
}

void WbWrenAtmosphericSky::update() {
  if (!mBakeEffect)
    return;
  if (!mAtmosphereDirty && !mSkyViewDirty)
    return;

  // Push the atmosphere parameters to every pass that consumes them.
  // (Sky-view also gets sun direction and camera altitude.)  PPE
  // apply() runs every pass in append order, so transmittance bakes
  // first, then multiscattering using transmittance, then sky-view
  // using both.  The connect() calls made in buildBakeEffect() route
  // those dependencies.
  pushAtmosphereParamsToPass(mTransmittancePass);
  pushAtmosphereParamsToPass(mMultiScatteringPass);
  pushAtmosphereParamsToPass(mSkyViewPass);

  const float sunDir[3] = {mSunDirection.x(), mSunDirection.y(), mSunDirection.z()};
  const float phaseG = mParams.miePhaseG;
  wr_post_processing_effect_pass_set_program_parameter(mSkyViewPass, "miePhaseG",
                                                       reinterpret_cast<const char *>(&phaseG));
  wr_post_processing_effect_pass_set_program_parameter(mSkyViewPass, "sunDirection",
                                                       reinterpret_cast<const char *>(sunDir));
  wr_post_processing_effect_pass_set_program_parameter(mSkyViewPass, "cameraHeightKm",
                                                       reinterpret_cast<const char *>(&mCameraHeightKm));

  WbWrenOpenGlContext::makeWrenCurrent();
  wr_post_processing_effect_apply(mBakeEffect);
  WbWrenOpenGlContext::doneWren();

  mAtmosphereDirty = false;
  mSkyViewDirty = false;
}

WrTextureCubeMap *WbWrenAtmosphericSky::bakeAndReleaseIrradianceCubeMap() {
  // Two-step bake:
  //   (1) project the sky-view LUT onto a cubemap so the existing IBL
  //       pipeline has a cube to consume;
  //   (2) pre-filter that cubemap with
  //       ``wr_texture_cubemap_bake_specular_irradiance``, producing
  //       the mip chain WbPbrAppearance reads at varying roughness.
  //
  // Ownership of the returned cube transfers to the caller — see the
  // header doc for why the impl deliberately keeps no reference.
  if (!mSkyViewPass)
    return NULL;
  WrTextureRtt *skyView = wr_post_processing_effect_pass_get_output_texture(mSkyViewPass, 0);
  if (!skyView)
    return NULL;

  // Push sun illuminance into the sky-to-cubemap shader so the cube
  // values are in the same absolute-luminance scale sky_apply emits.
  WrShaderProgram *toCube = WbWrenShaders::skyToCubemapShader();
  const float sunIll[3] = {mSunIlluminance.x(), mSunIlluminance.y(), mSunIlluminance.z()};
  wr_shader_program_set_custom_uniform_value(toCube, "sunIlluminance", WR_SHADER_PROGRAM_UNIFORM_TYPE_VEC3F,
                                             reinterpret_cast<const char *>(sunIll));

  // 128² source cube is the smallest size that survives the IBL
  // specular pre-filter's mip pyramid without obvious banding at
  // low roughness.  Larger sizes cost more for very small visual gain
  // on PBR rocks at typical viewing distances.
  WrTextureCubeMap *skyCube = wr_texture_cubemap_bake_from_2d(skyView, toCube, 128);
  if (!skyCube)
    return NULL;
  WrTextureCubeMap *irradiance = wr_texture_cubemap_bake_specular_irradiance(
    skyCube, WbWrenShaders::iblSpecularIrradianceBakingShader(), 128);
  wr_texture_delete(WR_TEXTURE(skyCube));
  return irradiance;
}

void WbWrenAtmosphericSky::bindForSkybox(WrMaterial *material) const {
  if (!material)
    return;

  WrShaderProgram *applyShader = WbWrenShaders::skyApplyShader();
  wr_material_set_default_program(material, applyShader);

  // sky_apply.frag now ray-marches per-pixel (matches the preview HTML
  // byte-for-byte), so it needs the full atmosphere parameter set.
  const float rayleighScat[3] = {mParams.rayleighScattering.x(), mParams.rayleighScattering.y(),
                                 mParams.rayleighScattering.z()};
  const float mieScat[3] = {mParams.mieScattering.x(), mParams.mieScattering.y(), mParams.mieScattering.z()};
  const float mieAbs[3] = {mParams.mieAbsorption.x(), mParams.mieAbsorption.y(), mParams.mieAbsorption.z()};
  const float ozone[3] = {mParams.absorptionExtinction.x(), mParams.absorptionExtinction.y(),
                          mParams.absorptionExtinction.z()};
  wr_shader_program_set_custom_uniform_value(applyShader, "bottomRadiusKm", WR_SHADER_PROGRAM_UNIFORM_TYPE_FLOAT,
                                             reinterpret_cast<const char *>(&mParams.bottomRadiusKm));
  wr_shader_program_set_custom_uniform_value(applyShader, "topRadiusKm", WR_SHADER_PROGRAM_UNIFORM_TYPE_FLOAT,
                                             reinterpret_cast<const char *>(&mParams.topRadiusKm));
  wr_shader_program_set_custom_uniform_value(applyShader, "rayleighScattering", WR_SHADER_PROGRAM_UNIFORM_TYPE_VEC3F,
                                             reinterpret_cast<const char *>(rayleighScat));
  wr_shader_program_set_custom_uniform_value(applyShader, "rayleighDensityExpScale",
                                             WR_SHADER_PROGRAM_UNIFORM_TYPE_FLOAT,
                                             reinterpret_cast<const char *>(&mParams.rayleighDensityExpScale));
  wr_shader_program_set_custom_uniform_value(applyShader, "mieScattering", WR_SHADER_PROGRAM_UNIFORM_TYPE_VEC3F,
                                             reinterpret_cast<const char *>(mieScat));
  wr_shader_program_set_custom_uniform_value(applyShader, "mieAbsorption", WR_SHADER_PROGRAM_UNIFORM_TYPE_VEC3F,
                                             reinterpret_cast<const char *>(mieAbs));
  wr_shader_program_set_custom_uniform_value(applyShader, "mieDensityExpScale", WR_SHADER_PROGRAM_UNIFORM_TYPE_FLOAT,
                                             reinterpret_cast<const char *>(&mParams.mieDensityExpScale));
  wr_shader_program_set_custom_uniform_value(applyShader, "miePhaseG", WR_SHADER_PROGRAM_UNIFORM_TYPE_FLOAT,
                                             reinterpret_cast<const char *>(&mParams.miePhaseG));
  wr_shader_program_set_custom_uniform_value(applyShader, "absorptionExtinction", WR_SHADER_PROGRAM_UNIFORM_TYPE_VEC3F,
                                             reinterpret_cast<const char *>(ozone));
  wr_shader_program_set_custom_uniform_value(applyShader, "groundAlbedo", WR_SHADER_PROGRAM_UNIFORM_TYPE_FLOAT,
                                             reinterpret_cast<const char *>(&mParams.groundAlbedo));

  const float sunDir[3] = {mSunDirection.x(), mSunDirection.y(), mSunDirection.z()};
  const float sunIll[3] = {mSunIlluminance.x(), mSunIlluminance.y(), mSunIlluminance.z()};
  // ~0.535° solar disc; matches the preview's default.
  const float discCos = 0.99995f;
  wr_shader_program_set_custom_uniform_value(applyShader, "sunDirection", WR_SHADER_PROGRAM_UNIFORM_TYPE_VEC3F,
                                             reinterpret_cast<const char *>(sunDir));
  wr_shader_program_set_custom_uniform_value(applyShader, "sunIlluminance", WR_SHADER_PROGRAM_UNIFORM_TYPE_VEC3F,
                                             reinterpret_cast<const char *>(sunIll));
  wr_shader_program_set_custom_uniform_value(applyShader, "sunDiscCosRadius", WR_SHADER_PROGRAM_UNIFORM_TYPE_FLOAT,
                                             reinterpret_cast<const char *>(&discCos));
  wr_shader_program_set_custom_uniform_value(applyShader, "cameraHeightKm", WR_SHADER_PROGRAM_UNIFORM_TYPE_FLOAT,
                                             reinterpret_cast<const char *>(&mCameraHeightKm));

  // Sky-view LUT is still bound at slot 0 — it's no longer used by
  // sky_apply (which ray-marches inline) but the slot stays wired so
  // future variants of sky_apply that want LUT-accelerated paths can
  // re-enable it without a binding change.
  if (mSkyViewPass) {
    WrTextureRtt *skyViewLut = wr_post_processing_effect_pass_get_output_texture(mSkyViewPass, 0);
    if (skyViewLut)
      wr_material_set_texture(material, WR_TEXTURE(skyViewLut), 0);
  }
}

void WbWrenAtmosphericSky::buildBakeEffect() {
  WbWrenOpenGlContext::makeWrenCurrent();

  mBakeEffect = wr_post_processing_effect_new();
  // No final-blit step: the LUTs ARE the output, sampled later via
  // mSkyViewPass->getOutputTexture(0).  ``set_result_program`` is
  // required by PPE::apply()'s assert; ``set_result_frame_buffer``
  // left NULL so renderToResultFrameBuffer() short-circuits.
  wr_post_processing_effect_set_result_program(mBakeEffect, WbWrenShaders::passThroughShader());

  auto makePass = [](const char *name, WrShaderProgram *program, int w, int h, int inputs) {
    WrPostProcessingEffectPass *pass = wr_post_processing_effect_pass_new();
    wr_post_processing_effect_pass_set_name(pass, name);
    wr_post_processing_effect_pass_set_program(pass, program);
    wr_post_processing_effect_pass_set_output_size(pass, w, h);
    wr_post_processing_effect_pass_set_input_texture_count(pass, inputs);
    wr_post_processing_effect_pass_set_output_texture_count(pass, 1);
    wr_post_processing_effect_pass_set_output_texture_format(pass, 0, WR_TEXTURE_INTERNAL_FORMAT_RGBA16F);
    wr_post_processing_effect_pass_set_clear_before_draw(pass, true);
    return pass;
  };

  mTransmittancePass =
    makePass("SkyTransmittance", WbWrenShaders::skyTransmittanceShader(), kTransmittanceWidth, kTransmittanceHeight, 0);
  mMultiScatteringPass = makePass("SkyMultiScattering", WbWrenShaders::skyMultiscatteringShader(), kMultiScatteringWidth,
                                  kMultiScatteringHeight, 1);
  mSkyViewPass =
    makePass("SkyView", WbWrenShaders::skyViewShader(), kSkyViewWidth, kSkyViewHeight, 2);

  wr_post_processing_effect_append_pass(mBakeEffect, mTransmittancePass);
  wr_post_processing_effect_append_pass(mBakeEffect, mMultiScatteringPass);
  wr_post_processing_effect_append_pass(mBakeEffect, mSkyViewPass);

  // Wire transmittance -> multiscattering input 0.
  // Wire transmittance -> sky-view input 0.
  // Wire multiscattering -> sky-view input 1.
  wr_post_processing_effect_connect(mBakeEffect, mTransmittancePass, 0, mMultiScatteringPass, 0);
  wr_post_processing_effect_connect(mBakeEffect, mTransmittancePass, 0, mSkyViewPass, 0);
  wr_post_processing_effect_connect(mBakeEffect, mMultiScatteringPass, 0, mSkyViewPass, 1);

  wr_post_processing_effect_setup(mBakeEffect);

  WbWrenOpenGlContext::doneWren();
}

void WbWrenAtmosphericSky::destroyBakeEffect() {
  WbWrenOpenGlContext::makeWrenCurrent();
  if (mBakeEffect) {
    wr_post_processing_effect_delete(mBakeEffect);
    mBakeEffect = NULL;
    // The passes are owned by the effect; deleting the effect deletes
    // them.  Just null the cached pointers.
    mTransmittancePass = NULL;
    mMultiScatteringPass = NULL;
    mSkyViewPass = NULL;
  }
  WbWrenOpenGlContext::doneWren();
}

void WbWrenAtmosphericSky::pushAtmosphereParamsToPass(WrPostProcessingEffectPass *pass) const {
  if (!pass)
    return;
  const float rayleighScat[3] = {mParams.rayleighScattering.x(), mParams.rayleighScattering.y(),
                                 mParams.rayleighScattering.z()};
  const float mieScat[3] = {mParams.mieScattering.x(), mParams.mieScattering.y(), mParams.mieScattering.z()};
  const float mieAbs[3] = {mParams.mieAbsorption.x(), mParams.mieAbsorption.y(), mParams.mieAbsorption.z()};
  const float ozone[3] = {mParams.absorptionExtinction.x(), mParams.absorptionExtinction.y(),
                          mParams.absorptionExtinction.z()};
  wr_post_processing_effect_pass_set_program_parameter(pass, "bottomRadiusKm",
                                                       reinterpret_cast<const char *>(&mParams.bottomRadiusKm));
  wr_post_processing_effect_pass_set_program_parameter(pass, "topRadiusKm",
                                                       reinterpret_cast<const char *>(&mParams.topRadiusKm));
  wr_post_processing_effect_pass_set_program_parameter(pass, "rayleighScattering",
                                                       reinterpret_cast<const char *>(rayleighScat));
  wr_post_processing_effect_pass_set_program_parameter(pass, "rayleighDensityExpScale",
                                                       reinterpret_cast<const char *>(&mParams.rayleighDensityExpScale));
  wr_post_processing_effect_pass_set_program_parameter(pass, "mieScattering", reinterpret_cast<const char *>(mieScat));
  wr_post_processing_effect_pass_set_program_parameter(pass, "mieAbsorption", reinterpret_cast<const char *>(mieAbs));
  wr_post_processing_effect_pass_set_program_parameter(pass, "mieDensityExpScale",
                                                       reinterpret_cast<const char *>(&mParams.mieDensityExpScale));
  wr_post_processing_effect_pass_set_program_parameter(pass, "absorptionExtinction",
                                                       reinterpret_cast<const char *>(ozone));
  wr_post_processing_effect_pass_set_program_parameter(pass, "groundAlbedo",
                                                       reinterpret_cast<const char *>(&mParams.groundAlbedo));
}
