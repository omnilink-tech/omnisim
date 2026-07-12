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

#ifndef SCENE_HPP
#define SCENE_HPP

#include "GlslLayout.hpp"
#include "Primitive.hpp"

#include <vector>

#include <wren/scene.h>

namespace wren {

  class Camera;
  class DirectionalLight;
  class LightNode;
  class PointLight;
  class Renderable;
  class SpotLight;
  class Transform;
  class Viewport;
  class ShaderProgram;
  class ShadowVolumeCaster;

  // Contains all elements making up a Scene.
  // The same Scene may be rendered through different viewports at different points in time.
  class Scene {
  public:
    // The corresponding enum in scene.h needs to be kept up to date
    enum FogType { FOG_TYPE_NONE, FOG_TYPE_EXPONENTIAL, FOG_TYPE_EXPONENTIAL2, FOG_TYPE_LINEAR };

    // Defines the basis for the fog distance computation
    // The The corresponding enum in scene.h needs to be kept up to date
    enum FogDepthType { FOG_DEPTH_TYPE_PLANE, FOG_DEPTH_TYPE_POINT };

    static Scene *instance();
    static void destroy() { delete cInstance; }

    static void init();
    void reset();
    static void applyPendingUpdates();

    static void getMainBuffer(int width, int height, unsigned int format, unsigned int data_type, void *buffer);
    void initFrameCapture(int pixelBufferCount, unsigned int *pixelBufferIds, int frameSize);
    static void bindPixelBuffer(int buffer);
    void *mapPixelBuffer(unsigned int accessMode);
    static void unMapPixelBuffer();
    void terminateFrameCapture();

    void addLight(LightNode *light);
    void removeLight(LightNode *light);

    void setFog(WrSceneFogType fogType = WR_SCENE_FOG_TYPE_LINEAR,
                WrSceneFogDepthType depthType = WR_SCENE_FOG_DEPTH_TYPE_PLANE, const glm::vec4 &color = gVec4Ones,
                float density = 1.0f, float start = 0.0f, float end = 1.0f);
    void setSkybox(Renderable *renderable);
    void setHdrClearQuad(Renderable *renderable);

    void setFogProgram(ShaderProgram *program) { mFogProgram = program; }
    void setShadowVolumeProgram(ShaderProgram *program) { mShadowVolumeProgram = program; }

    void enableDepthReset(bool enabled) { mClearDepth = enabled; }
    void enableSkybox(bool enabled) { mRenderSkybox = enabled; }
    void enableHdrClear(bool enabled) { mHdrClear = enabled; }
    void enableTranslucence(bool enabled) { mTranslucence = enabled; }

    Transform *root() { return mRoot; }
    Viewport *mainViewport() { return mMainViewport; }
    Viewport *currentViewport() { return mCurrentViewport; }

    const std::vector<DirectionalLight *> &directionalLights() const { return mDirectionalLightsActive; }
    const std::vector<PointLight *> &pointLights() const { return mPointLightsActive; }
    const std::vector<SpotLight *> &spotLights() const { return mSpotLightsActive; }

    void enqueueRenderable(Renderable *renderable);

    int computeNodeCount() const;
    static void printSceneTree();
    void render(bool culling, bool offScreen = false);
    void renderToViewports(const std::vector<Viewport *> &viewports, bool culling, bool offScreen = false);

    void addFrameListener(void (*listener)()) { mListeners.push_back(listener); }
    void removeFrameListener(void (*listener)());

    // Per-pass GPU timing — Tier 2 instrumentation. Off by default; webots
    // turns it on when OMNISIM_RENDERER_TIMINGS is set. See the public C
    // API in <wren/scene.h>.
    void setPassTimingEnabled(bool enabled);
    void getLastPassTimingsNs(unsigned long long *forwardNs, unsigned long long *postProcessNs) const;

    // Sub-pass forward breakdown:
    //   ambient = ambient/emissive draw + AO apply
    //   perLight = stencil shadow volumes + diffuse-specular per directional/point/spot light
    //   forwardResidual = fog + translucent + no-stencil-program + auxiliary draws between
    //                     the per-light loop and post-processing
    // The three together equal forwardNs (modulo nanosecond rounding).
    void getLastForwardSubPassTimingsNs(unsigned long long *ambientNs, unsigned long long *perLightNs,
                                        unsigned long long *forwardResidualNs) const;

    // Aggregate render-call totals since last reset. Used to measure the
    // *full* per-frame rendering load including per-sensor renders — each
    // WbWrenCamera / WbLidar / WbRangeFinder issues its own
    // renderToViewport call, and on a husky-heavy world that's where the
    // simulation's actual GPU cost lives. Last-call timings (above) only
    // reflect the most recent viewport processed; aggregate timings sum
    // across every viewport call for the window between resets.
    //   callCount      — total renderToViewport calls attempted
    //   harvestedCount — calls whose timing was captured (others lapped the
    //                    4-slot ring before the GPU returned results)
    //   forwardNs/postNs — summed across harvested calls only; divide by
    //                      harvestedCount for per-call average
    void getAggregateRenderTimings(unsigned long long *forwardNs, unsigned long long *postNs,
                                   unsigned int *callCount, unsigned int *harvestedCount) const;
    void resetAggregateRenderTimings();

    // Render-count counters — Tier 2 instrumentation, off by default.
    // When enabled, the main-queue partition results are accumulated each
    // frame so we can measure how many renderables exist, how many pass
    // the visibility-flag gate, and how many survive frustum culling.
    // Drives the T2.2 decision: is the bottleneck culling cost (lots of
    // visible renderables far from the camera), or draw-call submission
    // (visible == drawn but each renderable is its own glDrawElements)?
    void setRenderCountsEnabled(bool enabled) { mRenderCountsEnabled = enabled; }
    void getLastRenderCounts(int *enqueued, int *visible, int *drawn) const;

    // Total triangles submitted from the main draw queue last frame (sum
    // of triangleCount across drawn renderables). Used to disambiguate
    // submission-bound vs triangle-bound forward-GPU cost.
    long long getLastTrianglesDrawn() const { return mLastTrianglesDrawn; }

    // Per-Mesh draw-call histogram — built when render counts are enabled.
    // Returns the top entries sorted by draw_count descending. See
    // WrSceneMeshHistogramEntry in <wren/scene.h>. Each entry has the
    // raw mesh pointer's low 32 bits, count of renderables that drew it
    // last frame, and vertex / triangle counts of the mesh.
    struct MeshHistogramEntry {
      unsigned int meshIdLow32;
      int drawCount;
      int vertexCount;
      int triangleCount;
    };
    static const int kMaxMeshHistogramEntries = 8;
    int getTopMeshHistogram(MeshHistogramEntry *out, int maxEntries) const;

    // Instancing-candidate run detector — built when render counts are
    // enabled. Walks the post-state-sort opaque draw range and counts
    // runs of consecutive renderables sharing the same sortingId (i.e.
    // same mesh + material + faceCulling). Runs of length ≥ threshold
    // are the candidates that would collapse into one glDrawElements-
    // Instanced call once §8.2 Item 1 / §11.1 T2.2.d of
    // docs/developer/rendering-roadmap.md lands its draw-path merger.
    //
    // This accessor itself does NOT change the draw path -- it only
    // measures. Three stats:
    //   runCount: number of runs whose length is ≥ kInstancingRunThreshold
    //   maxRunLength: peak length of any run this frame
    //   mergeable: total renderables that fall inside a ≥-threshold run
    //              (the upper bound on draw calls saved if every run
    //              gets merged into one instanced call)
    void getLastInstancingStats(int *runCount, int *maxRunLength, int *mergeable) const;
    static const int kInstancingRunThreshold = 4;

  private:
    typedef std::vector<Renderable *>::iterator RenderQueueIterator;
    typedef std::vector<ShadowVolumeCaster *>::iterator ShadowVolumeIterator;
    typedef std::vector<Renderable *> RenderQueue;

    Scene();
    ~Scene();

    void prepareRender();
    void renderToViewport(bool culling);
    void updateFogUniformBuffer();

    RenderQueueIterator partitionByVisibility(RenderQueueIterator first, RenderQueueIterator last);
    RenderQueueIterator partitionByViewability(RenderQueueIterator first, RenderQueueIterator last);
    static RenderQueueIterator partitionByTranslucency(RenderQueueIterator first, RenderQueueIterator last);
    static RenderQueueIterator partitionByUseMaterial(RenderQueueIterator first, RenderQueueIterator last);
    static RenderQueueIterator partitionByStencilProgram(RenderQueueIterator first, RenderQueueIterator last);
    static RenderQueueIterator partitionByShadowReceiving(RenderQueueIterator first, RenderQueueIterator last);

    ShadowVolumeIterator partitionShadowsByVisibility(ShadowVolumeIterator first, ShadowVolumeIterator last, LightNode *light);

    static void sortRenderQueueByState(RenderQueueIterator first, RenderQueueIterator last);
    void sortRenderQueueByDistance(RenderQueueIterator first, RenderQueueIterator last) const;

    static void renderDefault(RenderQueueIterator first, RenderQueueIterator last, bool disableDepthTest = false);
    void renderStencilPerLight(LightNode *light, RenderQueueIterator first, RenderQueueIterator firstShadowReceiver,
                               RenderQueueIterator last);
    void renderStencilShadowVolumesDepthPass(ShadowVolumeCaster *shadowVolume, LightNode *light);
    void renderStencilShadowVolumesDepthFail(ShadowVolumeCaster *shadowVolume, LightNode *light);
    static void renderStencilAmbientEmissive(RenderQueueIterator first, RenderQueueIterator last);
    static void renderStencilDiffuseSpecular(RenderQueueIterator first, RenderQueueIterator last, LightNode *light,
                                             bool applyShadows = true);
    void renderStencilFog(RenderQueueIterator first, RenderQueueIterator last) const;
    static void renderStencilWithoutProgram(RenderQueueIterator first, RenderQueueIterator last);
    static void renderTranslucent(RenderQueueIterator first, RenderQueueIterator last, bool disableDepthTest = false);

    static Scene *cInstance;

    size_t mFrameCounter;

    Transform *mRoot;
    Viewport *mMainViewport;     // main window viewport
    Viewport *mCurrentViewport;  // viewport currently being rendered to

    std::vector<DirectionalLight *> mDirectionalLightsActive;
    std::vector<PointLight *> mPointLightsActive;
    std::vector<SpotLight *> mSpotLightsActive;

    std::vector<DirectionalLight *> mDirectionalLightsInactive;
    std::vector<PointLight *> mPointLightsInactive;
    std::vector<SpotLight *> mSpotLightsInactive;

    GlslLayout::LightRenderable mLightRenderable;

    std::vector<void (*)()> mListeners;
    std::vector<ShadowVolumeCaster *> mShadowVolumeQueue;
    std::vector<RenderQueue> mRenderQueues;
    Renderable *mSkybox;
    Renderable *mHdrClearQuad;
    ShaderProgram *mFogProgram;
    ShaderProgram *mShadowVolumeProgram;

    bool mRenderSkybox;
    bool mHdrClear;
    bool mTranslucence;
    bool mClearDepth;

    bool mIsFogDirty;
    GlslLayout::Fog mFog;

    int mPixelBufferCount;
    unsigned int *mPixelBufferIds;

    // Pass-timing ring of timestamp queries. Marker order per frame:
    //   start -> (ambient + AO done) -> (per-light loops done) -> postStart -> postEnd
    // Five queries per ring slot give us a 4-bucket forward breakdown
    // (ambient, perLight, forwardResidual, postProcess). Skipped entirely
    // when mPassTimingEnabled is false.
    static const int kPassTimingRing = 4;
    bool mPassTimingEnabled;
    bool mPassTimingInitialized;
    bool mPassTimingActiveThisFrame;
    unsigned int mPassTimingStartIds[kPassTimingRing];
    unsigned int mPassTimingAmbientEndIds[kPassTimingRing];
    unsigned int mPassTimingLightsEndIds[kPassTimingRing];
    unsigned int mPassTimingPostStartIds[kPassTimingRing];
    unsigned int mPassTimingPostEndIds[kPassTimingRing];
    bool mPassTimingInFlight[kPassTimingRing];
    int mPassTimingWriteIdx;
    unsigned long long mLastForwardNs;
    unsigned long long mLastPostProcessNs;
    unsigned long long mLastAmbientNs;
    unsigned long long mLastPerLightNs;
    unsigned long long mLastForwardResidualNs;

    // Aggregate accumulators across every renderToViewport call (main +
    // sensors) since the last reset. WbWrenWindow harvests + resets these
    // each ~120 main frames so we get rolling averages.
    unsigned long long mAggForwardNs;
    unsigned long long mAggPostNs;
    unsigned int mAggCallCount;
    unsigned int mAggHarvestedCount;

    void passTimingBeginRender();
    void passTimingMarkAmbientEnd();
    void passTimingMarkLightsEnd();
    void passTimingMarkPostStart();
    void passTimingMarkPostEnd();

    bool mRenderCountsEnabled;
    int mLastEnqueuedCount;
    int mLastVisibleCount;
    int mLastDrawnCount;
    long long mLastTrianglesDrawn;

    MeshHistogramEntry mLastTopMeshes[kMaxMeshHistogramEntries];
    int mLastTopMeshesFilled;

    int mLastInstancingRunCount;
    int mLastInstancingMaxRunLength;
    int mLastInstancingMergeable;
  };

}  // namespace wren

#endif  // SCENE_HPP
