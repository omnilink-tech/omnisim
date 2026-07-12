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

#include "Scene.hpp"

#include "Camera.hpp"
#include "ColorUtils.hpp"
#include "Config.hpp"
#include "Constants.hpp"
#include "ContainerUtils.hpp"
#include "Debug.hpp"
#include "DirectionalLight.hpp"
#include "GlState.hpp"
#include "GlUser.hpp"
#include "Id.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "Node.hpp"
#include "Overlay.hpp"
#include "PbrMaterial.hpp"
#include "PhongMaterial.hpp"
#include "PointLight.hpp"
#include "Renderable.hpp"
#include "ShaderProgram.hpp"
#include "ShadowVolumeCaster.hpp"
#include "SpotLight.hpp"
#include "Transform.hpp"
#include "UniformBuffer.hpp"
#include "Viewport.hpp"

#include <algorithm>
#include <unordered_map>

#include <wren/renderable.h>
#include <wren/scene.h>

#ifdef __EMSCRIPTEN__
#include <GL/gl.h>
#include <GLES3/gl3.h>
#else
#include <glad/glad.h>
#endif

#include <algorithm>
#include <memory>

#define INVERSE_LOG2 1.442695

namespace wren {

  Scene *Scene::cInstance = NULL;

  Scene *Scene::instance() {
    if (!Scene::cInstance)
      Scene::cInstance = new Scene();

    return Scene::cInstance;
  }

  void Scene::init() {
    if (glstate::isInitialized())
      return;

    glstate::init();
  }

  void Scene::applyPendingUpdates() {
    // Apply OpenGL state changes
    GlUser::applyGl();
  }

  void Scene::reset() {
    IdPhongMaterial::resetCounter();
    IdPbrMaterial::resetCounter();
    IdMesh::resetCounter();

    mDirectionalLightsActive.clear();
    mPointLightsActive.clear();
    mSpotLightsActive.clear();

    mDirectionalLightsInactive.clear();
    mPointLightsInactive.clear();
    mSpotLightsInactive.clear();

    // Apply modifications before reset
    GlUser::applyGl();

    debug::printCacheContents();

    // Check for leaks
    assert(!PhongMaterial::cachedItemCount());
    assert(!PbrMaterial::cachedItemCount());
    assert(!StaticMesh::cachedItemCount());
    assert(!Texture2d::cachedItemCount());
  }

  void Scene::getMainBuffer(int width, int height, unsigned int format, unsigned int data_type, void *buffer) {
    glstate::bindFrameBuffer(instance()->mMainViewport->frameBuffer()->glName());
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glFinish();
    glReadPixels(0, 0, width, height, format, data_type, buffer);
  }

  void Scene::initFrameCapture(int pixelBufferCount, unsigned int *pixelBufferIds, int frameSize) {
    mPixelBufferCount = pixelBufferCount;
    mPixelBufferIds = pixelBufferIds;
    glGenBuffers(mPixelBufferCount, mPixelBufferIds);
    for (int i = 0; i < mPixelBufferCount; ++i) {
      glBindBuffer(GL_PIXEL_PACK_BUFFER, mPixelBufferIds[i]);
      glBufferData(GL_PIXEL_PACK_BUFFER, frameSize, NULL, GL_STREAM_READ);
    }
  }

  void Scene::bindPixelBuffer(int buffer) {
    glBindBuffer(GL_PIXEL_PACK_BUFFER, buffer);
  }

  void *Scene::mapPixelBuffer(unsigned int accessMode) {
#ifdef __EMSCRIPTEN__
    return NULL;
#else
    return glMapBuffer(GL_PIXEL_PACK_BUFFER, accessMode);
#endif
  }
  void Scene::unMapPixelBuffer() {
    glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
  }

  void Scene::terminateFrameCapture() {
    glDeleteBuffers(mPixelBufferCount, mPixelBufferIds);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    mPixelBufferCount = 0;
    mPixelBufferIds = NULL;
  }

  void Scene::addLight(LightNode *light) {
    switch (light->type()) {
      case LightNode::TYPE_DIRECTIONAL:
        if (light->on())
          mDirectionalLightsActive.push_back(reinterpret_cast<DirectionalLight *>(light));
        else
          mDirectionalLightsInactive.push_back(reinterpret_cast<DirectionalLight *>(light));
        break;
      case LightNode::TYPE_POINT:
        if (light->on())
          mPointLightsActive.push_back(reinterpret_cast<PointLight *>(light));
        else
          mPointLightsInactive.push_back(reinterpret_cast<PointLight *>(light));
        break;
      case LightNode::TYPE_SPOT:
        if (light->on())
          mSpotLightsActive.push_back(reinterpret_cast<SpotLight *>(light));
        else
          mSpotLightsInactive.push_back(reinterpret_cast<SpotLight *>(light));
        break;
      default:
        assert(false);
    }
  }

  void Scene::removeLight(LightNode *light) {
    switch (light->type()) {
      case LightNode::TYPE_DIRECTIONAL:
        if (light->on())
          containerutils::removeElementFromVector(mDirectionalLightsActive, reinterpret_cast<DirectionalLight *>(light));
        else
          containerutils::removeElementFromVector(mDirectionalLightsInactive, reinterpret_cast<DirectionalLight *>(light));
        break;
      case LightNode::TYPE_POINT:
        if (light->on())
          containerutils::removeElementFromVector(mPointLightsActive, reinterpret_cast<PointLight *>(light));
        else
          containerutils::removeElementFromVector(mPointLightsInactive, reinterpret_cast<PointLight *>(light));
        break;
      case LightNode::TYPE_SPOT:
        if (light->on())
          containerutils::removeElementFromVector(mSpotLightsActive, reinterpret_cast<SpotLight *>(light));
        else
          containerutils::removeElementFromVector(mSpotLightsInactive, reinterpret_cast<SpotLight *>(light));
        break;
      default:
        assert(false);
    }
  }

  void Scene::setFog(WrSceneFogType fogType, WrSceneFogDepthType depthType, const glm::vec4 &color, float density, float start,
                     float end) {
    mIsFogDirty = true;
    mFog.mMode = glm::vec2(fogType, depthType);
    mFog.mColor = colorutils::srgbToLinear(color);
    // To speed up computation of a floating point exponent, we use the fact that e^(x) = 2^(x/log_e(2))
    // The operation is faster under the assumption that exp2() is faster than exp()
    mFog.mParams = glm::vec4(density * INVERSE_LOG2, density * density * INVERSE_LOG2, end, 1.0f / glm::abs(end - start));
  }

  void Scene::setSkybox(Renderable *renderable) {
    mSkybox = renderable;
    if (mSkybox) {
      mSkybox->defaultMaterial()->setEffectiveProgram(Material::MATERIAL_PROGRAM_DEFAULT);
      mSkybox->setEffectiveMaterial(mSkybox->defaultMaterial());
    }
  }

  void Scene::setHdrClearQuad(Renderable *renderable) {
    mHdrClearQuad = renderable;
    if (mHdrClearQuad) {
      mHdrClearQuad->defaultMaterial()->setEffectiveProgram(Material::MATERIAL_PROGRAM_DEFAULT);
      mHdrClearQuad->setEffectiveMaterial(mHdrClearQuad->defaultMaterial());
    }
  }

  int Scene::computeNodeCount() const {
    return 1 + mRoot->computeChildCount();
  }
  void Scene::printSceneTree() {
    debug::printSceneTree();
  }

  void Scene::render(bool culling, bool offScreen) {
    assert(glstate::isInitialized());

    ++mFrameCounter;
    DEBUG("\nScene::render: total nodes=" << Scene::computeNodeCount() << ", frame=" << mFrameCounter);

    // debug::printCacheContents();
    // debug::printSceneTree();

    renderToViewports({mMainViewport}, culling, offScreen);
  }

  void Scene::renderToViewports(const std::vector<Viewport *> &viewports, bool culling, bool offScreen) {
    assert(glstate::isInitialized());

    DEBUG("Notify frame listeners...");

    for (auto listener : mListeners)
      listener();

    DEBUG("\nScene::renderToViewports: viewports.size()=" << viewports.size());

    prepareRender();

    for (Viewport *viewport : viewports) {
      mCurrentViewport = viewport;

      glstate::setDefaultState();
      mCurrentViewport->updateUniforms();
      mCurrentViewport->bind();
      mCurrentViewport->clear();

      // Simplified rendering in case use material is set (for example when picking)
      if (Renderable::useMaterial()) {
        DEBUG("Rendering using material \"" << Renderable::useMaterial() << "\"");
        for (int i = 0; i < WR_RENDERABLE_DRAWING_ORDER_COUNT; ++i) {
          if (!mRenderQueues[i].size())
            continue;

          RenderQueueIterator firstInvisibleRenderable =
            partitionByVisibility(mRenderQueues[i].begin(), mRenderQueues[i].end());
          RenderQueueIterator firstCulledRenderable =
            partitionByViewability(mRenderQueues[i].begin(), firstInvisibleRenderable);

          // Only render Renderables that have the optional material currently in use
          RenderQueueIterator firstWithoutUseMaterial = partitionByUseMaterial(mRenderQueues[i].begin(), firstCulledRenderable);
          if (firstWithoutUseMaterial == mRenderQueues[i].begin())
            continue;

          if (mClearDepth && i > 0)
            glClear(GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

          sortRenderQueueByState(mRenderQueues[i].begin(), firstWithoutUseMaterial);
          renderDefault(mRenderQueues[i].begin(), firstWithoutUseMaterial, i > 0);
          mCurrentViewport->applyPostProcessing();
        }
      } else {
        renderToViewport(culling);
        if (!offScreen && mCurrentViewport == mMainViewport && mCurrentViewport->frameBuffer()) {
          glstate::bindDrawFrameBuffer(0);
          mCurrentViewport->frameBuffer()->blit(0, true, false, false, 0, 0, 0, 0, 0, 0,
                                                mCurrentViewport->width() * mCurrentViewport->pixelRatio(),
                                                mCurrentViewport->height() * mCurrentViewport->pixelRatio());
        }
      }

      mCurrentViewport->finishRender();
    }

    glstate::checkError();
  }

  void Scene::enqueueRenderable(Renderable *renderable) {
    mRenderQueues[renderable->drawingOrder()].push_back(renderable);

    if (!renderable->isTranslucent() && renderable->castShadows() && renderable->shadowVolumeCaster() &&
        renderable->drawingOrder() == WR_RENDERABLE_DRAWING_ORDER_MAIN &&
        renderable->drawingMode() == WR_RENDERABLE_DRAWING_MODE_TRIANGLES)
      mShadowVolumeQueue.push_back(renderable->shadowVolumeCaster());
  }

  void Scene::removeFrameListener(void (*listener)()) {
    mListeners.erase(std::remove(mListeners.begin(), mListeners.end(), listener), mListeners.end());
  }

  Scene::Scene() :
    mFrameCounter(0),
    mRoot(NULL),
    mMainViewport(NULL),
    mCurrentViewport(NULL),
    mRenderQueues(WR_RENDERABLE_DRAWING_ORDER_COUNT),
    mSkybox(NULL),
    mHdrClearQuad(NULL),
    mFogProgram(NULL),
    mShadowVolumeProgram(NULL),
    mRenderSkybox(true),
    mHdrClear(true),
    mTranslucence(true),
    mClearDepth(true),
    mPixelBufferCount(0),
    mPixelBufferIds(NULL),
    mPassTimingEnabled(false),
    mPassTimingInitialized(false),
    mPassTimingActiveThisFrame(false),
    mPassTimingWriteIdx(0),
    mLastForwardNs(0),
    mLastPostProcessNs(0),
    mLastAmbientNs(0),
    mLastPerLightNs(0),
    mLastForwardResidualNs(0),
    mAggForwardNs(0),
    mAggPostNs(0),
    mAggCallCount(0),
    mAggHarvestedCount(0),
    mRenderCountsEnabled(false),
    mLastEnqueuedCount(0),
    mLastVisibleCount(0),
    mLastDrawnCount(0),
    mLastTrianglesDrawn(0),
    mLastTopMeshesFilled(0),
    mLastInstancingRunCount(0),
    mLastInstancingMaxRunLength(0),
    mLastInstancingMergeable(0) {
    for (int i = 0; i < kPassTimingRing; ++i) {
      mPassTimingStartIds[i] = 0;
      mPassTimingAmbientEndIds[i] = 0;
      mPassTimingLightsEndIds[i] = 0;
      mPassTimingPostStartIds[i] = 0;
      mPassTimingPostEndIds[i] = 0;
      mPassTimingInFlight[i] = false;
    }
    mRoot = Transform::createTransform();
    mMainViewport = Viewport::createViewport();
    mMainViewport->setCamera(Camera::createCamera());
    setFog(WR_SCENE_FOG_TYPE_NONE);
  }

  Scene::~Scene() {
    // Cleanup static meshes used for AABBs & bounding spheres draw
    config::cleanup();

    reset();

    if (mMainViewport) {
      Node::deleteNode(mMainViewport->camera());
      Viewport::deleteViewport(mMainViewport);
    }

    if (mRoot)
      Node::deleteNode(mRoot);
  }

  void Scene::prepareRender() {
    // Perform accumulated OpenGL state changes
    GlUser::applyGl();

    // Update the scene tree and enqueue Renderables
    for (RenderQueue &renderQueue : mRenderQueues)
      renderQueue.clear();

    mShadowVolumeQueue.clear();

    mRoot->updateFromParent();

    // cppcheck-suppress reademptycontainer
    DEBUG("Number of shadow-casting Renderables: " << mShadowVolumeQueue.size());

    // Update fog uniform buffer
    updateFogUniformBuffer();
  }

  void Scene::setPassTimingEnabled(bool enabled) {
    mPassTimingEnabled = enabled;
  }

  void Scene::getLastPassTimingsNs(unsigned long long *forwardNs, unsigned long long *postProcessNs) const {
    if (forwardNs)
      *forwardNs = mLastForwardNs;
    if (postProcessNs)
      *postProcessNs = mLastPostProcessNs;
  }

  void Scene::getLastForwardSubPassTimingsNs(unsigned long long *ambientNs, unsigned long long *perLightNs,
                                             unsigned long long *forwardResidualNs) const {
    if (ambientNs)
      *ambientNs = mLastAmbientNs;
    if (perLightNs)
      *perLightNs = mLastPerLightNs;
    if (forwardResidualNs)
      *forwardResidualNs = mLastForwardResidualNs;
  }

  void Scene::getAggregateRenderTimings(unsigned long long *forwardNs, unsigned long long *postNs,
                                        unsigned int *callCount, unsigned int *harvestedCount) const {
    if (forwardNs)
      *forwardNs = mAggForwardNs;
    if (postNs)
      *postNs = mAggPostNs;
    if (callCount)
      *callCount = mAggCallCount;
    if (harvestedCount)
      *harvestedCount = mAggHarvestedCount;
  }

  void Scene::resetAggregateRenderTimings() {
    mAggForwardNs = 0;
    mAggPostNs = 0;
    mAggCallCount = 0;
    mAggHarvestedCount = 0;
  }

  void Scene::getLastRenderCounts(int *enqueued, int *visible, int *drawn) const {
    if (enqueued)
      *enqueued = mLastEnqueuedCount;
    if (visible)
      *visible = mLastVisibleCount;
    if (drawn)
      *drawn = mLastDrawnCount;
  }

  int Scene::getTopMeshHistogram(MeshHistogramEntry *out, int maxEntries) const {
    const int n = std::min(maxEntries, mLastTopMeshesFilled);
    for (int i = 0; i < n; ++i)
      out[i] = mLastTopMeshes[i];
    return mLastTopMeshesFilled;
  }

  void Scene::getLastInstancingStats(int *runCount, int *maxRunLength, int *mergeable) const {
    if (runCount)
      *runCount = mLastInstancingRunCount;
    if (maxRunLength)
      *maxRunLength = mLastInstancingMaxRunLength;
    if (mergeable)
      *mergeable = mLastInstancingMergeable;
  }

  void Scene::passTimingBeginRender() {
    mPassTimingActiveThisFrame = false;
    if (!mPassTimingEnabled)
      return;

    // Lazy-init the GL queries on the first enabled call. The arrays of
    // four are a 4-frame ring so the GPU can run 3-4 frames ahead without
    // forcing the CPU to stall waiting for results.
    if (!mPassTimingInitialized) {
      glGenQueries(kPassTimingRing, mPassTimingStartIds);
      glGenQueries(kPassTimingRing, mPassTimingAmbientEndIds);
      glGenQueries(kPassTimingRing, mPassTimingLightsEndIds);
      glGenQueries(kPassTimingRing, mPassTimingPostStartIds);
      glGenQueries(kPassTimingRing, mPassTimingPostEndIds);
      mPassTimingInitialized = true;
    }

    // Count this attempt — incremented even when the timing query is
    // dropped because the ring is full. Lets us compute total
    // renderToViewport calls per logging window vs. fraction captured.
    ++mAggCallCount;

    // Harvest any results that are now available, regardless of which slot.
    for (int i = 0; i < kPassTimingRing; ++i) {
      if (!mPassTimingInFlight[i])
        continue;
      GLint available = 0;
      glGetQueryObjectiv(mPassTimingPostEndIds[i], GL_QUERY_RESULT_AVAILABLE, &available);
      if (!available)
        continue;
      GLuint64 startNs = 0, ambientEndNs = 0, lightsEndNs = 0, postStartNs = 0, postEndNs = 0;
      glGetQueryObjectui64v(mPassTimingStartIds[i], GL_QUERY_RESULT, &startNs);
      glGetQueryObjectui64v(mPassTimingAmbientEndIds[i], GL_QUERY_RESULT, &ambientEndNs);
      glGetQueryObjectui64v(mPassTimingLightsEndIds[i], GL_QUERY_RESULT, &lightsEndNs);
      glGetQueryObjectui64v(mPassTimingPostStartIds[i], GL_QUERY_RESULT, &postStartNs);
      glGetQueryObjectui64v(mPassTimingPostEndIds[i], GL_QUERY_RESULT, &postEndNs);
      mPassTimingInFlight[i] = false;
      const unsigned long long forwardNs = postStartNs - startNs;
      const unsigned long long postNs = postEndNs - postStartNs;
      mLastForwardNs = forwardNs;
      mLastPostProcessNs = postNs;
      mLastAmbientNs = ambientEndNs - startNs;
      mLastPerLightNs = lightsEndNs - ambientEndNs;
      mLastForwardResidualNs = postStartNs - lightsEndNs;
      // Aggregate sums across every harvested viewport render (main +
      // sensor cameras + lidars + range finders). WbWrenWindow resets
      // these on its rolling-average cadence.
      mAggForwardNs += forwardNs;
      mAggPostNs += postNs;
      ++mAggHarvestedCount;
    }

    // If the slot we want to write to is still in-flight, the ring lapped
    // (GPU is more than 4 frames behind CPU). Drop this frame rather than
    // forcing a stall.
    if (mPassTimingInFlight[mPassTimingWriteIdx])
      return;

    glQueryCounter(mPassTimingStartIds[mPassTimingWriteIdx], GL_TIMESTAMP);
    mPassTimingActiveThisFrame = true;
  }

  void Scene::passTimingMarkAmbientEnd() {
    if (!mPassTimingActiveThisFrame)
      return;
    glQueryCounter(mPassTimingAmbientEndIds[mPassTimingWriteIdx], GL_TIMESTAMP);
  }

  void Scene::passTimingMarkLightsEnd() {
    if (!mPassTimingActiveThisFrame)
      return;
    glQueryCounter(mPassTimingLightsEndIds[mPassTimingWriteIdx], GL_TIMESTAMP);
  }

  void Scene::passTimingMarkPostStart() {
    if (!mPassTimingActiveThisFrame)
      return;
    glQueryCounter(mPassTimingPostStartIds[mPassTimingWriteIdx], GL_TIMESTAMP);
  }

  void Scene::passTimingMarkPostEnd() {
    if (!mPassTimingActiveThisFrame)
      return;
    glQueryCounter(mPassTimingPostEndIds[mPassTimingWriteIdx], GL_TIMESTAMP);
    mPassTimingInFlight[mPassTimingWriteIdx] = true;
    mPassTimingWriteIdx = (mPassTimingWriteIdx + 1) % kPassTimingRing;
    mPassTimingActiveThisFrame = false;
  }

  void Scene::renderToViewport(bool culling) {
    DEBUG("Scene::renderToViewport, viewport = " << mCurrentViewport);

    passTimingBeginRender();

    LightNode::updateUniforms();

    // hdr-clear first, irrespective of winding order
    if (mHdrClear && mHdrClearQuad && mCurrentViewport->isSkyboxEnabled()) {
      DEBUG("performing HDR clear!");

      glstate::setBlend(false);
      glstate::setDepthClamp(false);
      glstate::setDepthMask(false);
      glstate::setDepthTest(false);
      glstate::setStencilTest(false);
      glstate::setColorMask(true, true, true, true);
      glstate::setFrontFace(GL_CCW);

      mHdrClearQuad->render();

      if (mCurrentViewport->camera()->flipY())
        glstate::setFrontFace(GL_CW);
    }

    // Render skybox first
    if (mSkybox && mRenderSkybox && mCurrentViewport->isSkyboxEnabled()) {
      DEBUG("Rendering skybox");

      glstate::setBlend(false);

// GL_DEPTH_CLAMP is not available in webgl, so it is a way to work around
// It must be here and not in glstate::setDepthClamp because we need to access the camera
#ifdef __EMSCRIPTEN__
      float near = mCurrentViewport->camera()->nearDistance();
      mCurrentViewport->camera()->setNear(0.05);
      float far = mCurrentViewport->camera()->farDistance();
      mCurrentViewport->camera()->setFar(1000000.0f);
      mCurrentViewport->camera()->updateUniforms();
#endif
      glstate::setDepthClamp(true);
      glstate::setDepthMask(false);
      glstate::setDepthTest(true);
      glstate::setDepthFunc(GL_LESS);
      glstate::setStencilTest(false);
      glstate::setColorMask(true, true, true, true);

      mSkybox->render();

#ifdef __EMSCRIPTEN__
      mCurrentViewport->camera()->setNear(near);
      mCurrentViewport->camera()->setFar(far);
      mCurrentViewport->camera()->updateUniforms();
#endif
    }

    RenderQueue *renderQueue = &mRenderQueues[WR_RENDERABLE_DRAWING_ORDER_MAIN];
    DEBUG("Rendering queue 0, number of Renderables: " << renderQueue->size());

    RenderQueueIterator firstInvisibleRenderable = partitionByVisibility(renderQueue->begin(), renderQueue->end());
    DEBUG("Number of visible Renderables: " << firstInvisibleRenderable - renderQueue->begin());

    RenderQueueIterator firstCulledRenderable =
      culling ? partitionByViewability(renderQueue->begin(), firstInvisibleRenderable) : renderQueue->end();
    DEBUG("Number of non-culled Renderables: " << firstCulledRenderable - renderQueue->begin());

    if (mRenderCountsEnabled) {
      mLastEnqueuedCount = static_cast<int>(renderQueue->size());
      mLastVisibleCount = static_cast<int>(firstInvisibleRenderable - renderQueue->begin());
      mLastDrawnCount = static_cast<int>(firstCulledRenderable - renderQueue->begin());

      // Build per-geometry draw-count histogram over the drawn set.
      // Group by mesh->sortingId(), which is the cache-key-derived ID
      // shared across StaticMesh wrappers that point at the same
      // GPU buffers. That makes the histogram count "geometric
      // duplicates" — the actual instancing candidates — rather than
      // unique wrapper objects. Per-StaticMesh wrappers are cheap;
      // per-geometry uploads are not, and only the latter benefits
      // from instancing.
      //
      // Same loop also accumulates total triangles drawn this frame —
      // used to back-compute GPU triangle throughput and decide whether
      // forward GPU time is submission-bound or triangle-throughput-bound.
      struct Bucket {
        int count;
        int vertexCount;
        int triangleCount;
      };
      std::unordered_map<size_t, Bucket> counts;
      counts.reserve(static_cast<size_t>(mLastDrawnCount));
      long long trianglesDrawn = 0;
      for (auto it = renderQueue->begin(); it < firstCulledRenderable; ++it) {
        Mesh *m = (*it)->mesh();
        if (!m)
          continue;
        const size_t key = m->sortingId();
        Bucket &b = counts[key];
        if (b.count == 0) {
          b.vertexCount = m->vertexCount();
          b.triangleCount = m->triangleCount();
        }
        ++b.count;
        trianglesDrawn += static_cast<long long>(m->triangleCount());
      }
      mLastTrianglesDrawn = trianglesDrawn;

      std::vector<std::pair<size_t, Bucket>> sorted(counts.begin(), counts.end());
      std::sort(sorted.begin(), sorted.end(),
                [](const std::pair<size_t, Bucket> &a, const std::pair<size_t, Bucket> &b) {
                  return a.second.count > b.second.count;
                });

      const int filled = std::min(static_cast<int>(sorted.size()), kMaxMeshHistogramEntries);
      for (int i = 0; i < filled; ++i) {
        mLastTopMeshes[i].meshIdLow32 = static_cast<unsigned int>(sorted[i].first);
        mLastTopMeshes[i].drawCount = sorted[i].second.count;
        mLastTopMeshes[i].vertexCount = sorted[i].second.vertexCount;
        mLastTopMeshes[i].triangleCount = sorted[i].second.triangleCount;
      }
      mLastTopMeshesFilled = filled;
    }

    RenderQueueIterator firstOpaqueRenderable = renderQueue->begin();
    if (mTranslucence)
      firstOpaqueRenderable = partitionByTranslucency(renderQueue->begin(), firstCulledRenderable);

    DEBUG("Number of opaque Renderables: " << firstCulledRenderable - firstOpaqueRenderable);
    DEBUG("Number of translucent & z-sorted Renderables: " << firstOpaqueRenderable - renderQueue->begin());

    sortRenderQueueByState(firstOpaqueRenderable, firstCulledRenderable);
    sortRenderQueueByDistance(renderQueue->begin(), firstOpaqueRenderable);

    // Instancing-candidate run detector (item 1 of large-world-
    // optimization-plan.md, step 1 of N: telemetry only, no draw-path
    // change yet). After the state sort, renderables sharing the same
    // sortingId are contiguous. Walk and count runs of length ≥
    // kInstancingRunThreshold; each such run is a draw-call cluster
    // that the eventual instanced-draw path will collapse into a single
    // glDrawElementsInstanced. The numbers here drive the threshold
    // tuning before the merger actually lands.
    //
    // Gated on mRenderCountsEnabled so worlds that don't ask for
    // telemetry pay zero overhead (one branch instead of a per-frame
    // walk over the opaque queue).
    if (mRenderCountsEnabled) {
      int runCount = 0;
      int maxRunLength = 0;
      int mergeable = 0;
      auto runStart = firstOpaqueRenderable;
      while (runStart < firstCulledRenderable) {
        const size_t key = (*runStart)->sortingId();
        auto runEnd = runStart + 1;
        while (runEnd < firstCulledRenderable && (*runEnd)->sortingId() == key)
          ++runEnd;
        const int runLength = static_cast<int>(runEnd - runStart);
        if (runLength > maxRunLength)
          maxRunLength = runLength;
        if (runLength >= kInstancingRunThreshold) {
          ++runCount;
          mergeable += runLength;
        }
        runStart = runEnd;
      }
      mLastInstancingRunCount = runCount;
      mLastInstancingMaxRunLength = maxRunLength;
      mLastInstancingMergeable = mergeable;
    }

    DEBUG("Rendering opaque renderables");
    if (config::areShadowsEnabled() && mCurrentViewport->areShadowsEnabled() && LightNode::activeLightsCastingShadows() > 0) {
      DEBUG("Rendering ambient and emissive illumination");

      RenderQueueIterator firstWithoutStencilProgram = partitionByStencilProgram(firstOpaqueRenderable, firstCulledRenderable);
      RenderQueueIterator firstShadowReceiver = partitionByShadowReceiving(firstOpaqueRenderable, firstWithoutStencilProgram);

      renderStencilAmbientEmissive(firstOpaqueRenderable, firstWithoutStencilProgram);

      mCurrentViewport->applyAmbientOcclusion();
      passTimingMarkAmbientEnd();

      for (size_t i = 0; i < mDirectionalLightsActive.size(); ++i) {
        mLightRenderable.mActiveLights = glm::ivec4(i, -1, -1, -1);
        glstate::uniformBuffer(WR_GLSL_LAYOUT_UNIFORM_BUFFER_LIGHT_RENDERABLE)->writeValue(&mLightRenderable);

        renderStencilPerLight(mDirectionalLightsActive[i], firstOpaqueRenderable, firstShadowReceiver,
                              firstWithoutStencilProgram);
      }

      for (size_t i = 0; i < mPointLightsActive.size(); ++i) {
        mLightRenderable.mActiveLights = glm::ivec4(-1, i, -1, -1);
        glstate::uniformBuffer(WR_GLSL_LAYOUT_UNIFORM_BUFFER_LIGHT_RENDERABLE)->writeValue(&mLightRenderable);

        renderStencilPerLight(mPointLightsActive[i], firstOpaqueRenderable, firstShadowReceiver, firstWithoutStencilProgram);
      }

      for (size_t i = 0; i < mSpotLightsActive.size(); ++i) {
        mLightRenderable.mActiveLights = glm::ivec4(-1, -1, i, -1);
        glstate::uniformBuffer(WR_GLSL_LAYOUT_UNIFORM_BUFFER_LIGHT_RENDERABLE)->writeValue(&mLightRenderable);

        renderStencilPerLight(mSpotLightsActive[i], firstOpaqueRenderable, firstShadowReceiver, firstWithoutStencilProgram);
      }

      passTimingMarkLightsEnd();

      if (mFog.mMode.x > 0.0f) {
        DEBUG("Rendering fog");
        renderStencilFog(firstOpaqueRenderable, firstWithoutStencilProgram);
      }

      // Render renderables which don't have stencil shadow shaders
      renderStencilWithoutProgram(firstWithoutStencilProgram, firstCulledRenderable);
    } else {
      renderDefault(firstOpaqueRenderable, firstCulledRenderable);
      mCurrentViewport->applyAmbientOcclusion();
      // No per-light pass on this branch; mark both end-points here so
      // the breakdown stays consistent (perLight = 0, ambient covers
      // the whole forward draw + AO).
      passTimingMarkAmbientEnd();
      passTimingMarkLightsEnd();
    }

    if (firstOpaqueRenderable - renderQueue->begin()) {
      DEBUG("Rendering translucent renderables");
      renderTranslucent(renderQueue->begin(), firstOpaqueRenderable);
    }

    // Draw bounding volumes if enabled
    if (config::showBoundingSpheres()) {
      DEBUG("Drawing bounding spheres");
      for (auto it = renderQueue->begin(); it < renderQueue->end(); ++it)
        config::drawBoundingSphere((*it)->boundingSphere());
    }

    if (config::showAabbs()) {
      DEBUG("Drawing AABBs");
      for (auto it = renderQueue->begin(); it < renderQueue->end(); ++it)
        config::drawAabb((*it)->aabb());
    }

    // we want multiple effects to overwrite the viewport's post-processing effect framebuffer so we can't use GL_LESS or from
    // the second effect onwards the effects won't get written to the viewport's framebuffer
    glstate::setDepthFunc(GL_LEQUAL);

    // Render post-processing queue (preserving depth). Bracketed by GPU
    // timestamp markers so the Tier 2 instrumentation in scene.h can
    // expose forward vs post-process breakdown.
    passTimingMarkPostStart();
    mCurrentViewport->applyPostProcessing();
    passTimingMarkPostEnd();

    // Draw Renderables not part of main queue
    for (int i = WR_RENDERABLE_DRAWING_ORDER_MAIN + 1; i < WR_RENDERABLE_DRAWING_ORDER_COUNT; ++i) {
      if (!mRenderQueues[i].size())
        continue;

      if (mClearDepth)
        glClear(GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

      renderQueue = &mRenderQueues[i];
      DEBUG("Rendering queue " << i << ", number of Renderables: " << renderQueue->size());

      firstInvisibleRenderable = partitionByVisibility(renderQueue->begin(), renderQueue->end());
      firstCulledRenderable =
        culling ? partitionByViewability(renderQueue->begin(), firstInvisibleRenderable) : renderQueue->end();
      firstOpaqueRenderable = renderQueue->begin();

      if (mTranslucence)
        firstOpaqueRenderable = partitionByTranslucency(renderQueue->begin(), firstCulledRenderable);

      sortRenderQueueByState(firstOpaqueRenderable, firstCulledRenderable);
      sortRenderQueueByDistance(renderQueue->begin(), firstOpaqueRenderable);

      if (firstCulledRenderable - firstOpaqueRenderable) {
        DEBUG("Rendering opaque renderables");
        renderDefault(firstOpaqueRenderable, firstCulledRenderable, true);
      }

      if (firstOpaqueRenderable - renderQueue->begin()) {
        DEBUG("Rendering translucent renderables");
        renderTranslucent(renderQueue->begin(), firstOpaqueRenderable, true);
      }
    }
    mCurrentViewport->applyAntiAliasing();
    mCurrentViewport->drawOverlays();
  }

  void Scene::updateFogUniformBuffer() {
    if (!mIsFogDirty)
      return;

    glstate::uniformBuffer(WR_GLSL_LAYOUT_UNIFORM_BUFFER_FOG)->writeValue(&mFog);

    mIsFogDirty = false;
  }

  Scene::RenderQueueIterator Scene::partitionByVisibility(RenderQueueIterator first, RenderQueueIterator last) {
    return std::partition(
      first, last, [this](const Renderable *r) -> bool { return mCurrentViewport->visibilityMask() & r->visibilityFlags(); });
  }

  Scene::RenderQueueIterator Scene::partitionByViewability(RenderQueueIterator first, RenderQueueIterator last) {
    return std::partition(first, last, [this](Renderable *r) -> bool {
      return !r->sceneCulling() || mCurrentViewport->camera()->isAabbVisible(r->aabb());
    });
  }

  Scene::RenderQueueIterator Scene::partitionByTranslucency(RenderQueueIterator first, RenderQueueIterator last) {
    return std::partition(first, last, [](const Renderable *r) -> bool { return r->isTranslucent(); });
  }

  Scene::RenderQueueIterator Scene::partitionByUseMaterial(RenderQueueIterator first, RenderQueueIterator last) {
    std::string useMaterial(Renderable::useMaterial());

    return std::partition(first, last,
                          [&useMaterial](const Renderable *r) -> bool { return r->optionalMaterial(useMaterial); });
  }

  Scene::RenderQueueIterator Scene::partitionByStencilProgram(RenderQueueIterator first, RenderQueueIterator last) {
    return std::partition(first, last,
                          [](const Renderable *r) -> bool { return r->effectiveMaterial()->stencilAmbientEmissiveProgram(); });
  }

  Scene::RenderQueueIterator Scene::partitionByShadowReceiving(RenderQueueIterator first, RenderQueueIterator last) {
    return std::partition(first, last, [](const Renderable *r) -> bool { return !r->receiveShadows(); });
  }

  Scene::ShadowVolumeIterator Scene::partitionShadowsByVisibility(ShadowVolumeIterator first, ShadowVolumeIterator last,
                                                                  LightNode *light) {
    return std::partition(first, last, [this, &light](ShadowVolumeCaster *shadowVolume) -> bool {
      return mCurrentViewport->camera()->isAabbVisible(shadowVolume->aabb(light));
    });
  }

  void Scene::sortRenderQueueByState(RenderQueueIterator first, RenderQueueIterator last) {
    std::sort(first, last, [](const Renderable *a, const Renderable *b) -> bool { return a->sortingId() > b->sortingId(); });
  }

  void Scene::sortRenderQueueByDistance(RenderQueueIterator first, RenderQueueIterator last) const {
    for (auto it = first; it < last; ++it)
      (*it)->recomputeBoundingSphereInViewSpace(mCurrentViewport->camera()->view());

    std::sort(first, last, [](const Renderable *a, const Renderable *b) -> bool {
      const float aDistance =
        a->isInViewSpace() ? fabs(a->parent()->position()[2]) : glm::length2(a->boundingSphereInViewSpace().mCenter);
      const float bDistance =
        b->isInViewSpace() ? fabs(b->parent()->position()[2]) : glm::length2(b->boundingSphereInViewSpace().mCenter);
      return aDistance > bDistance;
    });
  }

  void Scene::renderDefault(RenderQueueIterator first, RenderQueueIterator last, bool disableDepthTest) {
    glstate::setBlend(false);
    glstate::setDepthClamp(false);
    glstate::setDepthMask(true);
    glstate::setDepthTest(!disableDepthTest);
    glstate::setDepthFunc(GL_LESS);
    glstate::setStencilTest(false);
    glstate::setCullFace(true);
    glstate::setColorMask(true, true, true, true);

    for (auto it = first; it < last; ++it) {
      assert((*it)->defaultMaterial());

      (*it)->effectiveMaterial()->setEffectiveProgram(Material::MATERIAL_PROGRAM_DEFAULT);
      (*it)->render();
    }
  }

  static bool affectedByLight(Renderable *renderable, LightNode *light) {
    bool visible = true;
    // Light culling
    if (light->type() != LightNode::TYPE_DIRECTIONAL) {
      PositionalLight *positionalLight = static_cast<PositionalLight *>(light);
      const primitive::Sphere &boundingSphere = renderable->boundingSphere();
      const float distance = glm::distance(boundingSphere.mCenter, positionalLight->position());
      const float radius = positionalLight->radius() + boundingSphere.mRadius;
      // Check if light is too far away
      visible = (distance <= radius &&
                 (distance < boundingSphere.mRadius ||  // Light is inside the boundingSphere, necessary because pow(distance -
                                                        // boundingSphere.mRadius, 2) can be very big in this case.
                  positionalLight->attenuationConstant() +
                      (distance - boundingSphere.mRadius) *
                        (positionalLight->attenuationLinear() +
                         (distance - boundingSphere.mRadius) * positionalLight->attenuationQuadratic()) <
                    2000.0f));
      // In the shaders, the attenuation is used as such:
      // attenuationFactor = 1/(attenuation[0] + distanceToLight * (attenuation[1] + distanceToLight * attenuation[2])
      // color = 1/attenuationFactor * color * ...
      // Due to this there is no well-defined upper bound for the attenuationFactor.
      // 2000 is a trade-off value in which the remaining light is very weak but still visible. The light become really
      // invisible around 8000.
    }
    return visible;
  }

  void Scene::renderStencilPerLight(LightNode *light, RenderQueueIterator first, RenderQueueIterator firstShadowReceiver,
                                    RenderQueueIterator last) {
    if (light->castShadows()) {
      assert(mShadowVolumeProgram);
      mShadowVolumeProgram->bind();

      Camera *camera = mCurrentViewport->camera();
      const primitive::Plane &farPlane = camera->frustum().plane(Frustum::FRUSTUM_PLANE_FAR);

      const primitive::Aabb &cameraAabb = camera->aabb();
      glm::vec3 cameraToLightInv;
      if (light->type() != LightNode::TYPE_DIRECTIONAL) {
        const PositionalLight *positionalLight = static_cast<PositionalLight *>(light);
        cameraToLightInv = 1.0f / glm::normalize(positionalLight->position() - camera->position());
      } else {
        const DirectionalLight *directionalLight = static_cast<DirectionalLight *>(light);
        cameraToLightInv = 1.0f / -directionalLight->direction();
      }

      ShadowVolumeIterator firstInvisibleShadowVolume =
        partitionShadowsByVisibility(mShadowVolumeQueue.begin(), mShadowVolumeQueue.end(), light);
      for (ShadowVolumeIterator it = mShadowVolumeQueue.begin(); it < firstInvisibleShadowVolume; ++it) {
        const primitive::Aabb renderableAabb = (*it)->renderable()->aabb();

        // Check if the renderable is affected by current light
        if (!affectedByLight((*it)->renderable(), light) || !primitive::isAabbAbovePlane(farPlane, renderableAabb))
          continue;

        // Use depth fail if camera stands in the shadow volume
        if (primitive::aabbCollision(cameraAabb, (*it)->aabb(light)) ||
            primitive::rayIntersectAabb(camera->position(), cameraToLightInv, renderableAabb, false))
          renderStencilShadowVolumesDepthFail(*it, light);
        else
          renderStencilShadowVolumesDepthPass(*it, light);

        if (config::showShadowAabbs()) {
          config::drawAabb((*it)->aabb(light));
          mShadowVolumeProgram->bind();
        }
      }

      if (first != firstShadowReceiver)
        renderStencilDiffuseSpecular(first, firstShadowReceiver, light, false);
      renderStencilDiffuseSpecular(firstShadowReceiver, last, light);

      glClear(GL_STENCIL_BUFFER_BIT);
    } else
      renderStencilDiffuseSpecular(first, last, light, false);
  }

  void Scene::renderStencilShadowVolumesDepthPass(ShadowVolumeCaster *shadowVolume, LightNode *light) {
    glstate::setDepthClamp(true);
    glstate::setDepthMask(false);
    glstate::setDepthTest(true);
    glstate::setDepthFunc(GL_LESS);
    glstate::setStencilTest(true);
    glstate::setStencilFunc(GL_ALWAYS, 0, ~0);
    glstate::setCullFace(false);
    glstate::setColorMask(false, false, false, false);

    // Special case for cw triangles
    if (shadowVolume->renderable()->invertFrontFace()) {
      glstate::setStencilOpFront(GL_KEEP, GL_KEEP, GL_DECR_WRAP);
      glstate::setStencilOpBack(GL_KEEP, GL_KEEP, GL_INCR_WRAP);
    } else {
      glstate::setStencilOpFront(GL_KEEP, GL_KEEP, GL_INCR_WRAP);
      glstate::setStencilOpBack(GL_KEEP, GL_KEEP, GL_DECR_WRAP);
    }

    // Compute silhouette without caps
    shadowVolume->computeSilhouette(light, false);

    glUniformMatrix4fv(mShadowVolumeProgram->uniformLocation(WR_GLSL_LAYOUT_UNIFORM_MODEL_TRANSFORM), 1, false,
                       glm::value_ptr(shadowVolume->renderable()->parent()->matrix()));
    shadowVolume->renderSides(light);
  }

  void Scene::renderStencilShadowVolumesDepthFail(ShadowVolumeCaster *shadowVolume, LightNode *light) {
    glstate::setDepthClamp(true);
    glstate::setDepthMask(false);
    glstate::setDepthTest(true);
    glstate::setDepthFunc(GL_GEQUAL);
    glstate::setStencilTest(true);
    glstate::setStencilFunc(GL_ALWAYS, 0, ~0);
    glstate::setCullFace(false);
    glstate::setColorMask(false, false, false, false);

    // Special case for cw triangles
    if (shadowVolume->renderable()->invertFrontFace()) {
      glstate::setStencilOpFront(GL_KEEP, GL_KEEP, GL_INCR_WRAP);
      glstate::setStencilOpBack(GL_KEEP, GL_KEEP, GL_DECR_WRAP);
    } else {
      glstate::setStencilOpFront(GL_KEEP, GL_KEEP, GL_DECR_WRAP);
      glstate::setStencilOpBack(GL_KEEP, GL_KEEP, GL_INCR_WRAP);
    }

    // Compute silhouette with caps
    shadowVolume->computeSilhouette(light, true);

    glUniformMatrix4fv(mShadowVolumeProgram->uniformLocation(WR_GLSL_LAYOUT_UNIFORM_MODEL_TRANSFORM), 1, false,
                       glm::value_ptr(shadowVolume->renderable()->parent()->matrix()));
    shadowVolume->renderSides(light);

    shadowVolume->renderCaps(light);
  }

  void Scene::renderStencilAmbientEmissive(RenderQueueIterator first, RenderQueueIterator last) {
    glstate::setBlend(false);
    glstate::setDepthClamp(false);
    glstate::setDepthMask(true);
    glstate::setDepthTest(true);
    glstate::setDepthFunc(GL_LESS);
    glstate::setStencilTest(false);
    glstate::setCullFace(true);
    glstate::setColorMask(true, true, true, true);

    for (auto it = first; it < last; ++it) {
      assert((*it)->effectiveMaterial()->stencilAmbientEmissiveProgram());

      (*it)->effectiveMaterial()->setEffectiveProgram(Material::MATERIAL_PROGRAM_STENCIL_AMBIENT_EMISSIVE);
      (*it)->render();
    }
  }

  void Scene::renderStencilDiffuseSpecular(RenderQueueIterator first, RenderQueueIterator last, LightNode *light,
                                           bool applyShadows) {
    glstate::setBlend(true);
    glstate::setBlendEquation(GL_FUNC_ADD);
    glstate::setBlendFunc(GL_ONE, GL_ONE);
    glstate::setDepthClamp(false);
    glstate::setDepthMask(false);
    glstate::setDepthTest(true);
    glstate::setDepthFunc(GL_LEQUAL);
    glstate::setCullFace(true);
    glstate::setColorMask(true, true, true, true);

    if (applyShadows) {
      glstate::setStencilTest(true);
      glstate::setStencilFunc(GL_EQUAL, 0, ~0);
      glstate::setStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
    } else
      glstate::setStencilTest(false);

    for (auto it = first; it < last; ++it) {
      if (!affectedByLight(*it, light))
        continue;

      assert((*it)->effectiveMaterial()->stencilDiffuseSpecularProgram());

      (*it)->effectiveMaterial()->setEffectiveProgram(Material::MATERIAL_PROGRAM_STENCIL_DIFFUSE_SPECULAR);
      (*it)->render();
    }
  }

  void Scene::renderStencilFog(RenderQueueIterator first, RenderQueueIterator last) const {
    glstate::setBlend(true);
    glstate::setBlendEquation(GL_FUNC_ADD);
    glstate::setBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glstate::setDepthClamp(false);
    glstate::setDepthMask(false);
    glstate::setDepthTest(true);
    glstate::setDepthFunc(GL_LEQUAL);
    glstate::setStencilTest(false);
    glstate::setCullFace(true);
    glstate::setColorMask(true, true, true, true);

    for (auto it = first; it < last; ++it)
      (*it)->renderWithoutMaterial(mFogProgram);
  }

  void Scene::renderStencilWithoutProgram(RenderQueueIterator first, RenderQueueIterator last) {
    glstate::setBlend(true);
    glstate::setBlendEquation(GL_FUNC_ADD);
    glstate::setBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glstate::setDepthClamp(false);
    glstate::setDepthMask(true);
    glstate::setDepthTest(true);
    glstate::setDepthFunc(GL_LESS);
    glstate::setStencilTest(false);
    glstate::setCullFace(true);
    glstate::setColorMask(true, true, true, true);

    for (auto it = first; it < last; ++it) {
      assert((*it)->effectiveMaterial());

      (*it)->effectiveMaterial()->setEffectiveProgram(Material::MATERIAL_PROGRAM_DEFAULT);
      (*it)->render();
    }
  }

  void Scene::renderTranslucent(RenderQueueIterator first, RenderQueueIterator last, bool disableDepthTest) {
    glstate::setBlend(true);
    glstate::setBlendEquation(GL_FUNC_ADD);
    glstate::setBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glstate::setDepthClamp(false);
    glstate::setDepthMask(false);
    glstate::setDepthTest(!disableDepthTest);
    glstate::setDepthFunc(GL_LESS);
    glstate::setStencilTest(false);
    glstate::setCullFace(true);
    glstate::setColorMask(true, true, true, true);

    for (auto it = first; it < last; ++it) {
      if (!(*it)->effectiveMaterial())
        continue;

      (*it)->effectiveMaterial()->setEffectiveProgram(Material::MATERIAL_PROGRAM_DEFAULT);
      (*it)->render();
    }
  }

}  // namespace wren

// C interface implementation
WrScene *wr_scene_get_instance() {
  return reinterpret_cast<WrScene *>(wren::Scene::instance());
}

void wr_scene_destroy() {
  wren::Scene::destroy();
}

void wr_scene_init(WrScene *scene) {
  reinterpret_cast<wren::Scene *>(scene)->init();
}

void wr_scene_apply_pending_updates(WrScene *scene) {
  reinterpret_cast<wren::Scene *>(scene)->applyPendingUpdates();
}

void wr_scene_get_main_buffer(int width, int height, unsigned int format, unsigned int data_type, void *buffer) {
  wren::Scene::getMainBuffer(width, height, format, data_type, buffer);
}

void wr_scene_init_frame_capture(WrScene *scene, int pixel_buffer_count, unsigned int *pixel_buffer_ids, int frame_size) {
  reinterpret_cast<wren::Scene *>(scene)->initFrameCapture(pixel_buffer_count, pixel_buffer_ids, frame_size);
}

void wr_scene_bind_pixel_buffer(WrScene *scene, int buffer) {
  reinterpret_cast<wren::Scene *>(scene)->bindPixelBuffer(buffer);
}

void *wr_scene_map_pixel_buffer(WrScene *scene, unsigned int access_mode) {
  return reinterpret_cast<wren::Scene *>(scene)->mapPixelBuffer(access_mode);
}

void wr_scene_unmap_pixel_buffer(WrScene *scene) {
  reinterpret_cast<wren::Scene *>(scene)->unMapPixelBuffer();
}

void wr_scene_terminate_frame_capture(WrScene *scene) {
  reinterpret_cast<wren::Scene *>(scene)->terminateFrameCapture();
}

void wr_scene_render(WrScene *scene, const char *material_name, bool culling, bool offScreen) {
  if (material_name)
    wren::Renderable::setUseMaterial(material_name);

  reinterpret_cast<wren::Scene *>(scene)->render(culling, offScreen);

  wren::Renderable::setUseMaterial(NULL);
}

void wr_scene_render_to_viewports(WrScene *scene, int count, WrViewport **viewports, const char *material_name, bool culling,
                                  bool offScreen) {
  if (material_name)
    wren::Renderable::setUseMaterial(material_name);

  wren::Viewport **start = reinterpret_cast<wren::Viewport **>(viewports);
  std::vector<wren::Viewport *> viewportsVector(start, start + count);
  reinterpret_cast<wren::Scene *>(scene)->renderToViewports(viewportsVector, culling, offScreen);

  wren::Renderable::setUseMaterial(NULL);
}

void wr_scene_reset(WrScene *scene) {
  reinterpret_cast<wren::Scene *>(scene)->reset();
}

void wr_scene_set_ambient_light(const float *ambient_light) {
  wren::LightNode::setAmbientLight(glm::make_vec3(ambient_light));
}

int wr_scene_get_active_spot_light_count(WrScene *scene) {
  return reinterpret_cast<wren::Scene *>(scene)->spotLights().size();
}

int wr_scene_get_active_point_light_count(WrScene *scene) {
  return reinterpret_cast<wren::Scene *>(scene)->pointLights().size();
}

int wr_scene_get_active_directional_light_count(WrScene *scene) {
  return reinterpret_cast<wren::Scene *>(scene)->directionalLights().size();
}

void wr_scene_set_fog(WrScene *scene, WrSceneFogType fogType, WrSceneFogDepthType depthType, const float *color, float density,
                      float start, float end) {
  reinterpret_cast<wren::Scene *>(scene)->setFog(
    fogType, depthType, color != NULL ? glm::vec4(color[0], color[1], color[2], 1.0f) : wren::gVec4Ones, density, start, end);
}

void wr_scene_set_skybox(WrScene *scene, WrRenderable *renderable) {
  reinterpret_cast<wren::Scene *>(scene)->setSkybox(reinterpret_cast<wren::Renderable *>(renderable));
}

void wr_scene_set_hdr_clear_quad(WrScene *scene, WrRenderable *renderable) {
  reinterpret_cast<wren::Scene *>(scene)->setHdrClearQuad(reinterpret_cast<wren::Renderable *>(renderable));
}

void wr_scene_set_fog_program(WrScene *scene, WrShaderProgram *program) {
  reinterpret_cast<wren::Scene *>(scene)->setFogProgram(reinterpret_cast<wren::ShaderProgram *>(program));
}

void wr_scene_set_shadow_volume_program(WrScene *scene, WrShaderProgram *program) {
  reinterpret_cast<wren::Scene *>(scene)->setShadowVolumeProgram(reinterpret_cast<wren::ShaderProgram *>(program));
}

void wr_scene_enable_depth_reset(WrScene *scene, bool enable) {
  reinterpret_cast<wren::Scene *>(scene)->enableDepthReset(enable);
}

void wr_scene_enable_skybox(WrScene *scene, bool enable) {
  reinterpret_cast<wren::Scene *>(scene)->enableSkybox(enable);
}

void wr_scene_enable_hdr_clear(WrScene *scene, bool enable) {
  reinterpret_cast<wren::Scene *>(scene)->enableHdrClear(enable);
}

void wr_scene_enable_translucence(WrScene *scene, bool enable) {
  reinterpret_cast<wren::Scene *>(scene)->enableTranslucence(enable);
}

int wr_scene_compute_node_count(WrScene *scene) {
  return reinterpret_cast<wren::Scene *>(scene)->computeNodeCount();
}

WrTransform *wr_scene_get_root(WrScene *scene) {
  return reinterpret_cast<WrTransform *>(reinterpret_cast<wren::Scene *>(scene)->root());
}

WrViewport *wr_scene_get_viewport(WrScene *scene) {
  return reinterpret_cast<WrViewport *>(reinterpret_cast<wren::Scene *>(scene)->mainViewport());
}

void wr_scene_add_frame_listener(WrScene *scene, void (*listener)()) {
  reinterpret_cast<wren::Scene *>(scene)->addFrameListener(listener);
}

void wr_scene_remove_frame_listener(WrScene *scene, void (*listener)()) {
  reinterpret_cast<wren::Scene *>(scene)->removeFrameListener(listener);
}

void wr_scene_set_pass_timing_enabled(bool enabled) {
  wren::Scene::instance()->setPassTimingEnabled(enabled);
}

void wr_scene_get_last_pass_timings_ns(unsigned long long *forward_ns, unsigned long long *post_process_ns) {
  wren::Scene::instance()->getLastPassTimingsNs(forward_ns, post_process_ns);
}

void wr_scene_get_last_forward_subpass_timings_ns(unsigned long long *ambient_ns, unsigned long long *per_light_ns,
                                                  unsigned long long *forward_residual_ns) {
  wren::Scene::instance()->getLastForwardSubPassTimingsNs(ambient_ns, per_light_ns, forward_residual_ns);
}

void wr_scene_get_aggregate_render_timings_ns(unsigned long long *forward_ns, unsigned long long *post_ns,
                                              unsigned int *call_count, unsigned int *harvested_count) {
  wren::Scene::instance()->getAggregateRenderTimings(forward_ns, post_ns, call_count, harvested_count);
}

void wr_scene_reset_aggregate_render_timings() {
  wren::Scene::instance()->resetAggregateRenderTimings();
}

void wr_scene_set_render_counts_enabled(bool enabled) {
  wren::Scene::instance()->setRenderCountsEnabled(enabled);
}

void wr_scene_get_last_render_counts(int *enqueued, int *visible, int *drawn) {
  wren::Scene::instance()->getLastRenderCounts(enqueued, visible, drawn);
}

long long wr_scene_get_last_triangles_drawn() {
  return wren::Scene::instance()->getLastTrianglesDrawn();
}

void wr_scene_get_last_instancing_stats(int *run_count, int *max_run_length, int *mergeable) {
  wren::Scene::instance()->getLastInstancingStats(run_count, max_run_length, mergeable);
}

void wr_scene_get_top_mesh_histogram(WrSceneMeshHistogramEntry *out, int max_entries, int *filled) {
  wren::Scene::MeshHistogramEntry localOut[wren::Scene::kMaxMeshHistogramEntries];
  const int total = wren::Scene::instance()->getTopMeshHistogram(localOut, max_entries);
  if (filled)
    *filled = total < max_entries ? total : max_entries;
  if (out && max_entries > 0) {
    const int n = total < max_entries ? total : max_entries;
    for (int i = 0; i < n; ++i) {
      out[i].mesh_id_low32 = localOut[i].meshIdLow32;
      out[i].draw_count = localOut[i].drawCount;
      out[i].vertex_count = localOut[i].vertexCount;
      out[i].triangle_count = localOut[i].triangleCount;
    }
  }
}
