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

#ifndef WB_WREN_ATMOSPHERIC_SKY_HPP
#define WB_WREN_ATMOSPHERIC_SKY_HPP

// Hillaire 2020 atmospheric sky.  Hooks into ``WbBackground`` when the
// ``atmosphericSky`` field is non-empty and replaces the static cubemap
// skybox with three small precomputed LUTs (transmittance, multiple
// scattering, sky-view) and a fullscreen apply pass.
//
// This is the implementation of T1.3 from
// docs/developer/rendering-roadmap.md.  Session-by-session progress
// is tracked in §7 of that same file (the atmospheric-sky tracker).
// When changing any of the four passes' GLSL math, update §7 in the
// same commit so the doc stays load-bearing.

#include <QtGui/QVector3D>

struct WrFrameBuffer;
struct WrMaterial;
struct WrRenderable;
struct WrShaderProgram;
struct WrStaticMesh;
struct WrTextureCubeMap;
struct WrTextureRtt;

class WbWrenAtmosphericSky {
public:
  // Per-planet atmosphere coefficients.  All distances in kilometres;
  // scattering / absorption coefficients in inverse metres at sea level
  // (Hillaire 2020 §2 canonical units).  Two presets ship:
  //
  // - earth: Bruneton 2008 / Hillaire 2020 published constants, blue
  //   Rayleigh, weak forward-scattering Mie, ozone absorption layer.
  // - mars: ~6 mbar CO2 + suspended dust.  Rayleigh ~1% of Earth and
  //   biased red; Mie strong and forward-scattering (dust); ozone
  //   replaced by a broad CO2 absorption term.  Tuned to land near the
  //   butterscotch/salmon look reported from Curiosity/Perseverance
  //   colour-calibrated panoramas.
  struct AtmosphereParameters {
    float bottomRadiusKm;
    float topRadiusKm;
    QVector3D rayleighScattering;
    float rayleighDensityExpScale;
    QVector3D mieScattering;
    QVector3D mieAbsorption;
    float mieDensityExpScale;
    float miePhaseG;
    QVector3D absorptionExtinction;
    float groundAlbedo;

    static AtmosphereParameters earth();
    static AtmosphereParameters mars();
  };

  WbWrenAtmosphericSky();
  ~WbWrenAtmosphericSky();

  // Preset names match ``Background.atmosphericSky`` field values.
  // Returns true if recognised; false leaves parameters unchanged and
  // the caller should fall back to the cubemap path.
  bool setPreset(const QString &preset);

  // CONVENTION: ``direction`` is FROM the camera TOWARD the sun
  // (i.e., pointing up at noon).  Webots ``DirectionalLight.direction``
  // uses the opposite sign (direction the light *travels*); the
  // caller is responsible for negating before handing it here.  This
  // matches Hillaire's published phase-function math and the sign
  // expected by sky_view.frag / sky_apply.frag.
  void setSunDirection(const QVector3D &direction);
  void setSunIlluminance(const QVector3D &illuminance);

  // Camera altitude above the planet ground (km).  Refreshes the sky-
  // view LUT but NOT the slower transmittance / multiscattering LUTs.
  void setCameraHeightKm(float heightKm);

  // Re-render whichever LUTs are stale.  Cheap to call every frame; the
  // class internally tracks which inputs changed.  In SESSION 1 the
  // shaders write magenta — turning atmosphericSky on before the math
  // lands gives a magenta sky which is the intended visual sentinel.
  void update();

  // Swap ``material``'s default program to sky_apply and push the
  // sun / sky-view uniforms.  Caller is responsible for restoring the
  // material's previous program if it later disables atmospheric mode.
  void bindForSkybox(WrMaterial *material) const;

  // Bake a fresh pre-filtered specular-irradiance cubemap from the
  // current sky-view LUT and transfer ownership to the caller.
  // Returns nullptr if the sky-view LUT is not yet baked.  The caller
  // (typically ``WbBackground``) is responsible for calling
  // ``wr_texture_delete`` on the result.  We deliberately do NOT
  // store the cube on the impl — that would double-own with the
  // ``WbBackground::mIrradianceCubeTexture`` slot and bake/destroy
  // re-entrance would double-free.
  WrTextureCubeMap *bakeAndReleaseIrradianceCubeMap();

private:
  void buildBakeEffect();
  void destroyBakeEffect();
  void pushAtmosphereParamsToPass(struct WrPostProcessingEffectPass *pass) const;

  AtmosphereParameters mParams;
  QVector3D mSunDirection;
  QVector3D mSunIlluminance;
  float mCameraHeightKm;

  bool mAtmosphereDirty;  // params changed -> rebake transmittance + multi-scatter.
  bool mSkyViewDirty;     // sun / camera changed -> rebake sky-view only.

  // The three LUTs live inside a single ``WrPostProcessingEffect`` —
  // the effect machinery already does FBO allocation, viewport
  // bookkeeping, and per-pass quad rendering, so we don't have to.
  // Each pass corresponds to one LUT; the multiscattering and sky-view
  // passes are connected to read the transmittance LUT (and, for
  // sky-view, multiscattering too) via the connection API.
  struct WrPostProcessingEffect *mBakeEffect;
  struct WrPostProcessingEffectPass *mTransmittancePass;
  struct WrPostProcessingEffectPass *mMultiScatteringPass;
  struct WrPostProcessingEffectPass *mSkyViewPass;

};

#endif  // WB_WREN_ATMOSPHERIC_SKY_HPP
