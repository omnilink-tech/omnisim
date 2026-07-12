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

#include "WbWgpuRenderTarget.hpp"

#include <QtGui/QImage>

#include "WbLog.hpp"
#include "WbVulkanBackend.hpp"
#include "WbWgpuShaders.hpp"

#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
#    include "webgpu/webgpu.h"
#    include "webgpu/wgpu.h"
#    include <cstring>
#  endif
#endif

#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
namespace {
  // wgpu buffer-to-buffer copy requires the bytes-per-row to be a
  // multiple of 256 (the COPY_BYTES_PER_ROW_ALIGNMENT). For RGBA8
  // that means a row stride of ceil(width*4 / 256) * 256.
  constexpr uint32_t kCopyBytesPerRowAlignment = 256;
  uint32_t alignedBytesPerRow(uint32_t widthPx) {
    const uint32_t unpadded = widthPx * 4u;
    const uint32_t r = unpadded % kCopyBytesPerRowAlignment;
    return r == 0 ? unpadded : unpadded + (kCopyBytesPerRowAlignment - r);
  }

  struct MapCapture {
    bool done = false;
    bool ok = false;
  };
  void onMap(WGPUMapAsyncStatus status, WGPUStringView /*msg*/, void *userdata1, void * /*userdata2*/) {
    auto *cap = static_cast<MapCapture *>(userdata1);
    cap->ok = (status == WGPUMapAsyncStatus_Success);
    cap->done = true;
  }
}  // namespace
#  endif
#endif

WbWgpuRenderTarget::WbWgpuRenderTarget(WbVulkanBackend *backend, uint32_t width, uint32_t height)
  : mBackend(backend), mWidth(width), mHeight(height) {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (!backend || !backend->isAvailable() || !backend->device() || !backend->queue() || width == 0 || height == 0)
    return;
  WGPUDevice device = static_cast<WGPUDevice>(backend->device());

  // 1. Color attachment texture. TextureBinding: the bloom post-pass samples the resolved scene.
  WGPUTextureDescriptor texDesc = {};
  texDesc.usage = WGPUTextureUsage_RenderAttachment | WGPUTextureUsage_CopySrc | WGPUTextureUsage_TextureBinding;
  texDesc.dimension = WGPUTextureDimension_2D;
  texDesc.size = {width, height, 1};
  texDesc.format = WGPUTextureFormat_RGBA8Unorm;
  texDesc.mipLevelCount = 1;
  texDesc.sampleCount = 1;
  WGPUTexture tex = wgpuDeviceCreateTexture(device, &texDesc);
  if (!tex) {
    WbLog::info("[WbWgpuRenderTarget] CreateTexture failed");
    return;
  }
  mTexture = tex;

  // 2. Default view for use as a render-pass color attachment.
  WGPUTextureView view = wgpuTextureCreateView(tex, nullptr);
  if (!view) {
    WbLog::info("[WbWgpuRenderTarget] TextureCreateView failed");
    return;
  }
  mView = view;

  // 3. Readback buffer. Stride must be aligned per
  //    COPY_BYTES_PER_ROW_ALIGNMENT.
  const uint32_t stride = alignedBytesPerRow(width);
  mReadbackBufferSize = static_cast<size_t>(stride) * static_cast<size_t>(height);
  WGPUBufferDescriptor bufDesc = {};
  bufDesc.size = mReadbackBufferSize;
  bufDesc.usage = WGPUBufferUsage_CopyDst | WGPUBufferUsage_MapRead;
  bufDesc.mappedAtCreation = false;
  WGPUBuffer rb = wgpuDeviceCreateBuffer(device, &bufDesc);
  if (!rb) {
    WbLog::info("[WbWgpuRenderTarget] readback Buffer creation failed");
    return;
  }
  mReadbackBuffer = rb;

  mUsable = true;
#  else
  (void)backend;
  (void)width;
  (void)height;
#  endif
#else
  (void)backend;
  (void)width;
  (void)height;
#endif
}

WbWgpuRenderTarget::~WbWgpuRenderTarget() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  for (void *pp : {mTexShadowPipelineMsaaRev, mTexShadowPipelineHdrRev, mSkyPipelineRev,
                   mSkyPipelineHdrRev, mScnClipDepthPipelineRev})
    if (pp)
      wgpuRenderPipelineRelease(static_cast<WGPURenderPipeline>(pp));
  for (void *vv : {mScnMsaaDepthViewF32, mScnDepthViewF32})
    if (vv)
      wgpuTextureViewRelease(static_cast<WGPUTextureView>(vv));
  for (void *tt : {mScnMsaaDepthTexF32, mScnDepthTexF32})
    if (tt)
      wgpuTextureRelease(static_cast<WGPUTexture>(tt));
  releaseTexShadowBgCache();
  for (void *bg : {mSsaoBgEstimate, mSsaoBgApply, mSsaoSceneBg})
    if (bg)
      wgpuBindGroupRelease(static_cast<WGPUBindGroup>(bg));
  for (void *ub : {mSsaoUb, mSsaoApplyUb})
    if (ub)
      wgpuBufferRelease(static_cast<WGPUBuffer>(ub));
  for (void *p : {mSsaoPipeline, mSsaoApplyPipeline})
    if (p)
      wgpuRenderPipelineRelease(static_cast<WGPURenderPipeline>(p));
  if (mSsaoShaderModule)
    wgpuShaderModuleRelease(static_cast<WGPUShaderModule>(mSsaoShaderModule));
  if (mSsaoEstimateBgl)
    wgpuBindGroupLayoutRelease(static_cast<WGPUBindGroupLayout>(mSsaoEstimateBgl));
  if (mAoDepthView)
    wgpuTextureViewRelease(static_cast<WGPUTextureView>(mAoDepthView));
  if (mAoDepthTex)
    wgpuTextureRelease(static_cast<WGPUTexture>(mAoDepthTex));
  if (mAgxBg)
    wgpuBindGroupRelease(static_cast<WGPUBindGroup>(mAgxBg));
  if (mAgxUb)
    wgpuBufferRelease(static_cast<WGPUBuffer>(mAgxUb));
  if (mAgxPipeline)
    wgpuRenderPipelineRelease(static_cast<WGPURenderPipeline>(mAgxPipeline));
  if (mAgxBgl)
    wgpuBindGroupLayoutRelease(static_cast<WGPUBindGroupLayout>(mAgxBgl));
  if (mAgxShaderModule)
    wgpuShaderModuleRelease(static_cast<WGPUShaderModule>(mAgxShaderModule));
  for (void *v : {mHdrView, mHdrMsaaView})
    if (v)
      wgpuTextureViewRelease(static_cast<WGPUTextureView>(v));
  for (void *t : {mHdrTex, mHdrMsaaTex})
    if (t)
      wgpuTextureRelease(static_cast<WGPUTexture>(t));
  for (void *p : {mTexShadowPipelineHdr, mSkyPipelineHdr})
    if (p)
      wgpuRenderPipelineRelease(static_cast<WGPURenderPipeline>(p));
  for (void *bg : {mBloomBgExtract, mBloomBgBlurH, mBloomBgBlurV, mBloomBgComposite})
    if (bg)
      wgpuBindGroupRelease(static_cast<WGPUBindGroup>(bg));
  for (void *ub : {mBloomUbExtract, mBloomUbBlurH, mBloomUbBlurV, mBloomUbComposite})
    if (ub)
      wgpuBufferRelease(static_cast<WGPUBuffer>(ub));
  for (void *v : {mBloomViewA, mBloomViewB})
    if (v)
      wgpuTextureViewRelease(static_cast<WGPUTextureView>(v));
  for (void *t : {mBloomTexA, mBloomTexB})
    if (t)
      wgpuTextureRelease(static_cast<WGPUTexture>(t));
  for (void *p : {mBloomExtractPipeline, mBloomBlurPipeline, mBloomCompositePipeline})
    if (p)
      wgpuRenderPipelineRelease(static_cast<WGPURenderPipeline>(p));
  if (mBloomBgl)
    wgpuBindGroupLayoutRelease(static_cast<WGPUBindGroupLayout>(mBloomBgl));
  if (mBloomShaderModule)
    wgpuShaderModuleRelease(static_cast<WGPUShaderModule>(mBloomShaderModule));
  if (mSkyBindGroup)
    wgpuBindGroupRelease(static_cast<WGPUBindGroup>(mSkyBindGroup));
  if (mSkyUniformBuffer)
    wgpuBufferRelease(static_cast<WGPUBuffer>(mSkyUniformBuffer));
  if (mSkyBindGroupLayout)
    wgpuBindGroupLayoutRelease(static_cast<WGPUBindGroupLayout>(mSkyBindGroupLayout));
  if (mSkyPipeline)
    wgpuRenderPipelineRelease(static_cast<WGPURenderPipeline>(mSkyPipeline));
  if (mSkyShaderModule)
    wgpuShaderModuleRelease(static_cast<WGPUShaderModule>(mSkyShaderModule));
  if (mReadbackBufferB)
    wgpuBufferRelease(static_cast<WGPUBuffer>(mReadbackBufferB));
  if (mTexShadowPipelineMsaa)
    wgpuRenderPipelineRelease(static_cast<WGPURenderPipeline>(mTexShadowPipelineMsaa));
  if (mScnMsaaColorView)
    wgpuTextureViewRelease(static_cast<WGPUTextureView>(mScnMsaaColorView));
  if (mScnMsaaColorTex)
    wgpuTextureRelease(static_cast<WGPUTexture>(mScnMsaaColorTex));
  if (mScnMsaaDepthView)
    wgpuTextureViewRelease(static_cast<WGPUTextureView>(mScnMsaaDepthView));
  if (mScnMsaaDepthTex)
    wgpuTextureRelease(static_cast<WGPUTexture>(mScnMsaaDepthTex));
  if (mScnDepthView)
    wgpuTextureViewRelease(static_cast<WGPUTextureView>(mScnDepthView));
  if (mScnDepthTexture)
    wgpuTextureRelease(static_cast<WGPUTexture>(mScnDepthTexture));
  if (mScnUniformBuffer)
    wgpuBufferRelease(static_cast<WGPUBuffer>(mScnUniformBuffer));
  if (mScnBindGroupLayout)
    wgpuBindGroupLayoutRelease(static_cast<WGPUBindGroupLayout>(mScnBindGroupLayout));
  if (mScnPipeline)
    wgpuRenderPipelineRelease(static_cast<WGPURenderPipeline>(mScnPipeline));
  if (mScnShaderModule)
    wgpuShaderModuleRelease(static_cast<WGPUShaderModule>(mScnShaderModule));
  if (mScnDepthPipeline)
    wgpuRenderPipelineRelease(static_cast<WGPURenderPipeline>(mScnDepthPipeline));
  if (mScnDepthShaderModule)
    wgpuShaderModuleRelease(static_cast<WGPUShaderModule>(mScnDepthShaderModule));
  if (mScnAgxPipeline)
    wgpuRenderPipelineRelease(static_cast<WGPURenderPipeline>(mScnAgxPipeline));
  if (mScnAgxShaderModule)
    wgpuShaderModuleRelease(static_cast<WGPUShaderModule>(mScnAgxShaderModule));
  if (mScnTexPipeline)
    wgpuRenderPipelineRelease(static_cast<WGPURenderPipeline>(mScnTexPipeline));
  if (mScnTexShaderModule)
    wgpuShaderModuleRelease(static_cast<WGPUShaderModule>(mScnTexShaderModule));
  if (mScnTexBindGroupLayout)
    wgpuBindGroupLayoutRelease(static_cast<WGPUBindGroupLayout>(mScnTexBindGroupLayout));
  if (mScnTexSampler)
    wgpuSamplerRelease(static_cast<WGPUSampler>(mScnTexSampler));
  if (mScnDefaultWhiteView)
    wgpuTextureViewRelease(static_cast<WGPUTextureView>(mScnDefaultWhiteView));
  if (mScnDefaultWhiteTex)
    wgpuTextureRelease(static_cast<WGPUTexture>(mScnDefaultWhiteTex));
  if (mScnDefaultBlackView)
    wgpuTextureViewRelease(static_cast<WGPUTextureView>(mScnDefaultBlackView));
  if (mScnDefaultBlackTex)
    wgpuTextureRelease(static_cast<WGPUTexture>(mScnDefaultBlackTex));
  if (mScnDefaultNormalView)
    wgpuTextureViewRelease(static_cast<WGPUTextureView>(mScnDefaultNormalView));
  if (mScnDefaultNormalTex)
    wgpuTextureRelease(static_cast<WGPUTexture>(mScnDefaultNormalTex));
  if (mTexShadowPipeline)
    wgpuRenderPipelineRelease(static_cast<WGPURenderPipeline>(mTexShadowPipeline));
  if (mTexShadowShaderModule)
    wgpuShaderModuleRelease(static_cast<WGPUShaderModule>(mTexShadowShaderModule));
  if (mTexShadowBindGroupLayout)
    wgpuBindGroupLayoutRelease(static_cast<WGPUBindGroupLayout>(mTexShadowBindGroupLayout));
  if (mTexShadowSampler)
    wgpuSamplerRelease(static_cast<WGPUSampler>(mTexShadowSampler));
  if (mLightUniformBuffer)
    wgpuBufferRelease(static_cast<WGPUBuffer>(mLightUniformBuffer));
  if (mPickPipeline)
    wgpuRenderPipelineRelease(static_cast<WGPURenderPipeline>(mPickPipeline));
  if (mPickShaderModule)
    wgpuShaderModuleRelease(static_cast<WGPUShaderModule>(mPickShaderModule));
  if (mLinePipeline)
    wgpuRenderPipelineRelease(static_cast<WGPURenderPipeline>(mLinePipeline));
  if (mLineNoDepthPipeline)
    wgpuRenderPipelineRelease(static_cast<WGPURenderPipeline>(mLineNoDepthPipeline));
  if (mLineShaderModule)
    wgpuShaderModuleRelease(static_cast<WGPUShaderModule>(mLineShaderModule));
  if (mOverlayPipeline)
    wgpuRenderPipelineRelease(static_cast<WGPURenderPipeline>(mOverlayPipeline));
  if (mOverlayShaderModule)
    wgpuShaderModuleRelease(static_cast<WGPUShaderModule>(mOverlayShaderModule));
  if (mOverlayBindGroupLayout)
    wgpuBindGroupLayoutRelease(static_cast<WGPUBindGroupLayout>(mOverlayBindGroupLayout));
  if (mOverlayUniformBuffer)
    wgpuBufferRelease(static_cast<WGPUBuffer>(mOverlayUniformBuffer));
  if (mTexQuadPipeline)
    wgpuRenderPipelineRelease(static_cast<WGPURenderPipeline>(mTexQuadPipeline));
  if (mTexQuadShaderModule)
    wgpuShaderModuleRelease(static_cast<WGPUShaderModule>(mTexQuadShaderModule));
  if (mTexQuadBindGroupLayout)
    wgpuBindGroupLayoutRelease(static_cast<WGPUBindGroupLayout>(mTexQuadBindGroupLayout));
  if (mTexQuadSampler)
    wgpuSamplerRelease(static_cast<WGPUSampler>(mTexQuadSampler));
  if (mTexQuadUniformBuffer)
    wgpuBufferRelease(static_cast<WGPUBuffer>(mTexQuadUniformBuffer));
  if (mScnShadowPipeline)
    wgpuRenderPipelineRelease(static_cast<WGPURenderPipeline>(mScnShadowPipeline));
  if (mScnShadowShaderModule)
    wgpuShaderModuleRelease(static_cast<WGPUShaderModule>(mScnShadowShaderModule));
  if (mScnShadowBindGroupLayout)
    wgpuBindGroupLayoutRelease(static_cast<WGPUBindGroupLayout>(mScnShadowBindGroupLayout));
  if (mScnShadowSampler)
    wgpuSamplerRelease(static_cast<WGPUSampler>(mScnShadowSampler));
  if (mShadowMapView)
    wgpuTextureViewRelease(static_cast<WGPUTextureView>(mShadowMapView));
  if (mShadowMapTexture)
    wgpuTextureRelease(static_cast<WGPUTexture>(mShadowMapTexture));
  if (mShadowMapDepthView)
    wgpuTextureViewRelease(static_cast<WGPUTextureView>(mShadowMapDepthView));
  if (mShadowMapDepthTexture)
    wgpuTextureRelease(static_cast<WGPUTexture>(mShadowMapDepthTexture));
  if (mShadowUniformBuffer)
    wgpuBufferRelease(static_cast<WGPUBuffer>(mShadowUniformBuffer));
  // T1.2 CSM (multi-cascade) resources.
  if (mCsmPipeline)
    wgpuRenderPipelineRelease(static_cast<WGPURenderPipeline>(mCsmPipeline));
  if (mCsmShaderModule)
    wgpuShaderModuleRelease(static_cast<WGPUShaderModule>(mCsmShaderModule));
  if (mCsmBindGroupLayout)
    wgpuBindGroupLayoutRelease(static_cast<WGPUBindGroupLayout>(mCsmBindGroupLayout));
  if (mCsmSampler)
    wgpuSamplerRelease(static_cast<WGPUSampler>(mCsmSampler));
  for (void *v : mCsmShadowLayerViews)
    if (v)
      wgpuTextureViewRelease(static_cast<WGPUTextureView>(v));
  if (mCsmShadowArrayView)
    wgpuTextureViewRelease(static_cast<WGPUTextureView>(mCsmShadowArrayView));
  if (mCsmShadowArrayTexture)
    wgpuTextureRelease(static_cast<WGPUTexture>(mCsmShadowArrayTexture));
  if (mCsmShadowDepthView)
    wgpuTextureViewRelease(static_cast<WGPUTextureView>(mCsmShadowDepthView));
  if (mCsmShadowDepthTexture)
    wgpuTextureRelease(static_cast<WGPUTexture>(mCsmShadowDepthTexture));
  if (mCsmDepthUniformBuffer)
    wgpuBufferRelease(static_cast<WGPUBuffer>(mCsmDepthUniformBuffer));
  if (mCsmUniformBuffer)
    wgpuBufferRelease(static_cast<WGPUBuffer>(mCsmUniformBuffer));
  // T1.4 TAA resources.
  if (mTaaPipeline)
    wgpuRenderPipelineRelease(static_cast<WGPURenderPipeline>(mTaaPipeline));
  if (mTaaShaderModule)
    wgpuShaderModuleRelease(static_cast<WGPUShaderModule>(mTaaShaderModule));
  if (mTaaBindGroupLayout)
    wgpuBindGroupLayoutRelease(static_cast<WGPUBindGroupLayout>(mTaaBindGroupLayout));
  if (mTaaSampler)
    wgpuSamplerRelease(static_cast<WGPUSampler>(mTaaSampler));
  if (mTaaUniformBuffer)
    wgpuBufferRelease(static_cast<WGPUBuffer>(mTaaUniformBuffer));
  for (int i = 0; i < 2; ++i) {
    if (mTaaHistoryView[i])
      wgpuTextureViewRelease(static_cast<WGPUTextureView>(mTaaHistoryView[i]));
    if (mTaaHistoryTexture[i])
      wgpuTextureRelease(static_cast<WGPUTexture>(mTaaHistoryTexture[i]));
  }
  // T1.3 fog resources.
  if (mFogPipeline)
    wgpuRenderPipelineRelease(static_cast<WGPURenderPipeline>(mFogPipeline));
  if (mFogShaderModule)
    wgpuShaderModuleRelease(static_cast<WGPUShaderModule>(mFogShaderModule));
  if (mFogBindGroupLayout)
    wgpuBindGroupLayoutRelease(static_cast<WGPUBindGroupLayout>(mFogBindGroupLayout));
  if (mFogSampler)
    wgpuSamplerRelease(static_cast<WGPUSampler>(mFogSampler));
  if (mFogUniformBuffer)
    wgpuBufferRelease(static_cast<WGPUBuffer>(mFogUniformBuffer));
  // kSolidLitTexturedCsm (full material × multi-cascade) resources.
  if (mTexCsmPipeline)
    wgpuRenderPipelineRelease(static_cast<WGPURenderPipeline>(mTexCsmPipeline));
  if (mTexCsmShaderModule)
    wgpuShaderModuleRelease(static_cast<WGPUShaderModule>(mTexCsmShaderModule));
  if (mTexCsmBindGroupLayout)
    wgpuBindGroupLayoutRelease(static_cast<WGPUBindGroupLayout>(mTexCsmBindGroupLayout));
  if (mTexCsmSampler)
    wgpuSamplerRelease(static_cast<WGPUSampler>(mTexCsmSampler));
  if (mCsmLightUniformBuffer)
    wgpuBufferRelease(static_cast<WGPUBuffer>(mCsmLightUniformBuffer));
  if (mScnDepthF32Pipeline)
    wgpuRenderPipelineRelease(static_cast<WGPURenderPipeline>(mScnDepthF32Pipeline));
  if (mScnDepthF32ShaderModule)
    wgpuShaderModuleRelease(static_cast<WGPUShaderModule>(mScnDepthF32ShaderModule));
  if (mScnClipDepthPipeline)
    wgpuRenderPipelineRelease(static_cast<WGPURenderPipeline>(mScnClipDepthPipeline));
  if (mScnClipDepthShaderModule)
    wgpuShaderModuleRelease(static_cast<WGPUShaderModule>(mScnClipDepthShaderModule));
  if (mScnRangeF32Pipeline)
    wgpuRenderPipelineRelease(static_cast<WGPURenderPipeline>(mScnRangeF32Pipeline));
  if (mScnRangeF32ShaderModule)
    wgpuShaderModuleRelease(static_cast<WGPUShaderModule>(mScnRangeF32ShaderModule));
  if (mDepthF32View)
    wgpuTextureViewRelease(static_cast<WGPUTextureView>(mDepthF32View));
  if (mDepthF32Texture)
    wgpuTextureRelease(static_cast<WGPUTexture>(mDepthF32Texture));
  if (mInstStorageBuffer)
    wgpuBufferRelease(static_cast<WGPUBuffer>(mInstStorageBuffer));
  if (mInstBindGroupLayout)
    wgpuBindGroupLayoutRelease(static_cast<WGPUBindGroupLayout>(mInstBindGroupLayout));
  if (mInstPipeline)
    wgpuRenderPipelineRelease(static_cast<WGPURenderPipeline>(mInstPipeline));
  if (mInstShaderModule)
    wgpuShaderModuleRelease(static_cast<WGPUShaderModule>(mInstShaderModule));
  if (mTexSampler)
    wgpuSamplerRelease(static_cast<WGPUSampler>(mTexSampler));
  if (mTexBindGroupLayout)
    wgpuBindGroupLayoutRelease(static_cast<WGPUBindGroupLayout>(mTexBindGroupLayout));
  if (mTexPipeline)
    wgpuRenderPipelineRelease(static_cast<WGPURenderPipeline>(mTexPipeline));
  if (mTexShaderModule)
    wgpuShaderModuleRelease(static_cast<WGPUShaderModule>(mTexShaderModule));
  if (mMvpBindGroup)
    wgpuBindGroupRelease(static_cast<WGPUBindGroup>(mMvpBindGroup));
  if (mMvpBindGroupLayout)
    wgpuBindGroupLayoutRelease(static_cast<WGPUBindGroupLayout>(mMvpBindGroupLayout));
  if (mMvpUniformBuffer)
    wgpuBufferRelease(static_cast<WGPUBuffer>(mMvpUniformBuffer));
  if (mMvpPipeline)
    wgpuRenderPipelineRelease(static_cast<WGPURenderPipeline>(mMvpPipeline));
  if (mMvpShaderModule)
    wgpuShaderModuleRelease(static_cast<WGPUShaderModule>(mMvpShaderModule));
  if (mMeshPipeline)
    wgpuRenderPipelineRelease(static_cast<WGPURenderPipeline>(mMeshPipeline));
  if (mMeshShaderModule)
    wgpuShaderModuleRelease(static_cast<WGPUShaderModule>(mMeshShaderModule));
  if (mTrianglePipeline)
    wgpuRenderPipelineRelease(static_cast<WGPURenderPipeline>(mTrianglePipeline));
  if (mTriangleShaderModule)
    wgpuShaderModuleRelease(static_cast<WGPUShaderModule>(mTriangleShaderModule));
  if (mReadbackBuffer)
    wgpuBufferRelease(static_cast<WGPUBuffer>(mReadbackBuffer));
  if (mView)
    wgpuTextureViewRelease(static_cast<WGPUTextureView>(mView));
  if (mTexture)
    wgpuTextureRelease(static_cast<WGPUTexture>(mTexture));
#  endif
#endif
}

// 256-byte minimum offset between dynamic-uniform-binding entries
// (`minUniformBufferOffsetAlignment` per the WebGPU spec). The
// actual payload (struct Scene in kSolidLit) is 192 B; padded to
// 256 B per draw.
static constexpr uint32_t kScnUniformStride = 256;
// T1.2 CSM: render-local max cascade count (mirrors WbWgpuSceneRenderer::kMaxCascades,
// kept independent so the render layer carries no up-dependency on nodes/).
static constexpr uint32_t kCsmMaxCascades = 4;

bool WbWgpuRenderTarget::ensureScenePipeline() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (mScnPipeline)
    return true;
  if (!mUsable)
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());

  WGPUShaderSourceWGSL wgsl = {};
  wgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
  wgsl.code.data = WbWgpuShaders::kSolidLit;
  wgsl.code.length = WGPU_STRLEN;
  WGPUShaderModuleDescriptor smDesc = {};
  smDesc.nextInChain = &wgsl.chain;
  WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &smDesc);
  if (!sm) {
    WbLog::info("[WbWgpuRenderTarget] scn-pipeline CreateShaderModule failed");
    return false;
  }
  mScnShaderModule = sm;

  // Bind-group layout: one dynamic-offset uniform buffer entry of
  // size 192 B (the Scene struct).
  WGPUBindGroupLayoutEntry bglEntry = {};
  bglEntry.binding = 0;
  bglEntry.visibility = WGPUShaderStage_Vertex | WGPUShaderStage_Fragment;
  bglEntry.buffer.type = WGPUBufferBindingType_Uniform;
  bglEntry.buffer.hasDynamicOffset = 1;
  bglEntry.buffer.minBindingSize = 192;
  WGPUBindGroupLayoutDescriptor bglDesc = {};
  bglDesc.entryCount = 1;
  bglDesc.entries = &bglEntry;
  WGPUBindGroupLayout bgl = wgpuDeviceCreateBindGroupLayout(device, &bglDesc);
  if (!bgl) {
    WbLog::info("[WbWgpuRenderTarget] scn-pipeline CreateBindGroupLayout failed");
    return false;
  }
  mScnBindGroupLayout = bgl;

  WGPUPipelineLayoutDescriptor plDesc = {};
  plDesc.bindGroupLayoutCount = 1;
  plDesc.bindGroupLayouts = &bgl;
  WGPUPipelineLayout plLayout = wgpuDeviceCreatePipelineLayout(device, &plDesc);
  if (!plLayout) {
    WbLog::info("[WbWgpuRenderTarget] scn-pipeline CreatePipelineLayout failed");
    return false;
  }

  // Depth attachment so multiple Solids occlude correctly.
  WGPUTextureDescriptor dtDesc = {};
  dtDesc.usage = WGPUTextureUsage_RenderAttachment;
  dtDesc.dimension = WGPUTextureDimension_2D;
  dtDesc.size = {mWidth, mHeight, 1};
  dtDesc.format = WGPUTextureFormat_Depth24Plus;
  dtDesc.mipLevelCount = 1;
  dtDesc.sampleCount = 1;
  WGPUTexture dt = wgpuDeviceCreateTexture(device, &dtDesc);
  if (!dt) {
    WbLog::info("[WbWgpuRenderTarget] scn-pipeline depth texture failed");
    return false;
  }
  mScnDepthTexture = dt;
  WGPUTextureView dv = wgpuTextureCreateView(dt, nullptr);
  if (!dv) {
    WbLog::info("[WbWgpuRenderTarget] scn-pipeline depth view failed");
    return false;
  }
  mScnDepthView = dv;

  WGPUVertexAttribute attrs[3] = {};
  attrs[0].format = WGPUVertexFormat_Float32x3;
  attrs[0].offset = 0;
  attrs[0].shaderLocation = 0;
  attrs[1].format = WGPUVertexFormat_Float32x3;
  attrs[1].offset = 12;
  attrs[1].shaderLocation = 1;
  attrs[2].format = WGPUVertexFormat_Float32x2;
  attrs[2].offset = 24;
  attrs[2].shaderLocation = 2;
  WGPUVertexBufferLayout vbLayout = {};
  vbLayout.arrayStride = 32;
  vbLayout.stepMode = WGPUVertexStepMode_Vertex;
  vbLayout.attributeCount = 3;
  vbLayout.attributes = attrs;

  WGPUColorTargetState colorTarget = {};
  colorTarget.format = WGPUTextureFormat_RGBA8Unorm;
  colorTarget.writeMask = WGPUColorWriteMask_All;

  WGPUFragmentState fs = {};
  fs.module = sm;
  fs.entryPoint.data = "fs_main";
  fs.entryPoint.length = WGPU_STRLEN;
  fs.targetCount = 1;
  fs.targets = &colorTarget;

  WGPUDepthStencilState depthState = {};
  depthState.format = WGPUTextureFormat_Depth24Plus;
  depthState.depthWriteEnabled = WGPUOptionalBool_True;
  depthState.depthCompare = WGPUCompareFunction_Less;
  depthState.stencilFront.compare = WGPUCompareFunction_Always;
  depthState.stencilFront.failOp = WGPUStencilOperation_Keep;
  depthState.stencilFront.depthFailOp = WGPUStencilOperation_Keep;
  depthState.stencilFront.passOp = WGPUStencilOperation_Keep;
  depthState.stencilBack = depthState.stencilFront;
  depthState.stencilReadMask = 0xFFFFFFFFu;
  depthState.stencilWriteMask = 0xFFFFFFFFu;

  WGPURenderPipelineDescriptor pipeDesc = {};
  pipeDesc.layout = plLayout;
  pipeDesc.vertex.module = sm;
  pipeDesc.vertex.entryPoint.data = "vs_main";
  pipeDesc.vertex.entryPoint.length = WGPU_STRLEN;
  pipeDesc.vertex.bufferCount = 1;
  pipeDesc.vertex.buffers = &vbLayout;
  pipeDesc.primitive.topology = WGPUPrimitiveTopology_TriangleList;
  pipeDesc.primitive.frontFace = WGPUFrontFace_CCW;
  pipeDesc.primitive.cullMode = WGPUCullMode_None;
  pipeDesc.depthStencil = &depthState;
  pipeDesc.multisample.count = 1;
  pipeDesc.multisample.mask = 0xFFFFFFFFu;
  pipeDesc.fragment = &fs;
  WGPURenderPipeline pipe = wgpuDeviceCreateRenderPipeline(device, &pipeDesc);
  wgpuPipelineLayoutRelease(plLayout);
  if (!pipe) {
    WbLog::info("[WbWgpuRenderTarget] scn-pipeline CreateRenderPipeline failed");
    return false;
  }
  mScnPipeline = pipe;
  return true;
#  else
  return false;
#  endif
#else
  return false;
#endif
}

// R5: build the depth/distance scene pipeline. Reuses the lit path's
// bind-group layout + depth view (so ensureScenePipeline runs first); only the
// shader + pipeline differ. kSolidDistance outputs linear view-space distance.
bool WbWgpuRenderTarget::ensureSceneDepthPipeline() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (mScnDepthPipeline)
    return true;
  if (!ensureScenePipeline())  // creates mScnBindGroupLayout + mScnDepthView (shared)
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());

  WGPUShaderSourceWGSL wgsl = {};
  wgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
  wgsl.code.data = WbWgpuShaders::kSolidDistance;
  wgsl.code.length = WGPU_STRLEN;
  WGPUShaderModuleDescriptor smDesc = {};
  smDesc.nextInChain = &wgsl.chain;
  WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &smDesc);
  if (!sm) {
    WbLog::info("[WbWgpuRenderTarget] scn-depth CreateShaderModule failed");
    return false;
  }
  mScnDepthShaderModule = sm;

  WGPUBindGroupLayout bgl = static_cast<WGPUBindGroupLayout>(mScnBindGroupLayout);
  WGPUPipelineLayoutDescriptor plDesc = {};
  plDesc.bindGroupLayoutCount = 1;
  plDesc.bindGroupLayouts = &bgl;
  WGPUPipelineLayout plLayout = wgpuDeviceCreatePipelineLayout(device, &plDesc);
  if (!plLayout) {
    WbLog::info("[WbWgpuRenderTarget] scn-depth CreatePipelineLayout failed");
    return false;
  }

  WGPUVertexAttribute attrs[3] = {};
  attrs[0].format = WGPUVertexFormat_Float32x3; attrs[0].offset = 0;  attrs[0].shaderLocation = 0;
  attrs[1].format = WGPUVertexFormat_Float32x3; attrs[1].offset = 12; attrs[1].shaderLocation = 1;
  attrs[2].format = WGPUVertexFormat_Float32x2; attrs[2].offset = 24; attrs[2].shaderLocation = 2;
  WGPUVertexBufferLayout vbLayout = {};
  vbLayout.arrayStride = 32;
  vbLayout.stepMode = WGPUVertexStepMode_Vertex;
  vbLayout.attributeCount = 3;
  vbLayout.attributes = attrs;

  WGPUColorTargetState colorTarget = {};
  colorTarget.format = WGPUTextureFormat_RGBA8Unorm;
  colorTarget.writeMask = WGPUColorWriteMask_All;

  WGPUFragmentState fs = {};
  fs.module = sm;
  fs.entryPoint.data = "fs_main";
  fs.entryPoint.length = WGPU_STRLEN;
  fs.targetCount = 1;
  fs.targets = &colorTarget;

  WGPUDepthStencilState depthState = {};
  depthState.format = WGPUTextureFormat_Depth24Plus;
  depthState.depthWriteEnabled = WGPUOptionalBool_True;
  depthState.depthCompare = WGPUCompareFunction_Less;
  depthState.stencilFront.compare = WGPUCompareFunction_Always;
  depthState.stencilFront.failOp = WGPUStencilOperation_Keep;
  depthState.stencilFront.depthFailOp = WGPUStencilOperation_Keep;
  depthState.stencilFront.passOp = WGPUStencilOperation_Keep;
  depthState.stencilBack = depthState.stencilFront;
  depthState.stencilReadMask = 0xFFFFFFFFu;
  depthState.stencilWriteMask = 0xFFFFFFFFu;

  WGPURenderPipelineDescriptor pipeDesc = {};
  pipeDesc.layout = plLayout;
  pipeDesc.vertex.module = sm;
  pipeDesc.vertex.entryPoint.data = "vs_main";
  pipeDesc.vertex.entryPoint.length = WGPU_STRLEN;
  pipeDesc.vertex.bufferCount = 1;
  pipeDesc.vertex.buffers = &vbLayout;
  pipeDesc.primitive.topology = WGPUPrimitiveTopology_TriangleList;
  pipeDesc.primitive.frontFace = WGPUFrontFace_CCW;
  pipeDesc.primitive.cullMode = WGPUCullMode_None;
  pipeDesc.depthStencil = &depthState;
  pipeDesc.multisample.count = 1;
  pipeDesc.multisample.mask = 0xFFFFFFFFu;
  pipeDesc.fragment = &fs;
  WGPURenderPipeline pipe = wgpuDeviceCreateRenderPipeline(device, &pipeDesc);
  wgpuPipelineLayoutRelease(plLayout);
  if (!pipe) {
    WbLog::info("[WbWgpuRenderTarget] scn-depth CreateRenderPipeline failed");
    return false;
  }
  mScnDepthPipeline = pipe;
  return true;
#  else
  return false;
#  endif
#else
  return false;
#endif
}

// T1.1: build the AgX-tonemapped scene pipeline. Identical to the lit pipeline
// (RGBA8 color + Depth24Plus, same vertex + bind-group layout) — only the
// fragment shader differs (kSolidLitAgX). Reuses mScnBindGroupLayout +
// mScnDepthView built by ensureScenePipeline.
bool WbWgpuRenderTarget::ensureSceneAgxPipeline() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (mScnAgxPipeline)
    return true;
  if (!ensureScenePipeline())  // creates mScnBindGroupLayout + mScnDepthView (shared)
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());

  WGPUShaderSourceWGSL wgsl = {};
  wgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
  wgsl.code.data = WbWgpuShaders::kSolidLitAgX;
  wgsl.code.length = WGPU_STRLEN;
  WGPUShaderModuleDescriptor smDesc = {};
  smDesc.nextInChain = &wgsl.chain;
  WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &smDesc);
  if (!sm) {
    WbLog::info("[WbWgpuRenderTarget] scn-agx CreateShaderModule failed");
    return false;
  }
  mScnAgxShaderModule = sm;

  WGPUBindGroupLayout bgl = static_cast<WGPUBindGroupLayout>(mScnBindGroupLayout);
  WGPUPipelineLayoutDescriptor plDesc = {};
  plDesc.bindGroupLayoutCount = 1;
  plDesc.bindGroupLayouts = &bgl;
  WGPUPipelineLayout plLayout = wgpuDeviceCreatePipelineLayout(device, &plDesc);
  if (!plLayout) {
    WbLog::info("[WbWgpuRenderTarget] scn-agx CreatePipelineLayout failed");
    return false;
  }

  WGPUVertexAttribute attrs[3] = {};
  attrs[0].format = WGPUVertexFormat_Float32x3; attrs[0].offset = 0;  attrs[0].shaderLocation = 0;
  attrs[1].format = WGPUVertexFormat_Float32x3; attrs[1].offset = 12; attrs[1].shaderLocation = 1;
  attrs[2].format = WGPUVertexFormat_Float32x2; attrs[2].offset = 24; attrs[2].shaderLocation = 2;
  WGPUVertexBufferLayout vbLayout = {};
  vbLayout.arrayStride = 32;
  vbLayout.stepMode = WGPUVertexStepMode_Vertex;
  vbLayout.attributeCount = 3;
  vbLayout.attributes = attrs;

  WGPUColorTargetState colorTarget = {};
  colorTarget.format = WGPUTextureFormat_RGBA8Unorm;
  colorTarget.writeMask = WGPUColorWriteMask_All;

  WGPUFragmentState fs = {};
  fs.module = sm;
  fs.entryPoint.data = "fs_main";
  fs.entryPoint.length = WGPU_STRLEN;
  fs.targetCount = 1;
  fs.targets = &colorTarget;

  WGPUDepthStencilState depthState = {};
  depthState.format = WGPUTextureFormat_Depth24Plus;
  depthState.depthWriteEnabled = WGPUOptionalBool_True;
  depthState.depthCompare = WGPUCompareFunction_Less;
  depthState.stencilFront.compare = WGPUCompareFunction_Always;
  depthState.stencilFront.failOp = WGPUStencilOperation_Keep;
  depthState.stencilFront.depthFailOp = WGPUStencilOperation_Keep;
  depthState.stencilFront.passOp = WGPUStencilOperation_Keep;
  depthState.stencilBack = depthState.stencilFront;
  depthState.stencilReadMask = 0xFFFFFFFFu;
  depthState.stencilWriteMask = 0xFFFFFFFFu;

  WGPURenderPipelineDescriptor pipeDesc = {};
  pipeDesc.layout = plLayout;
  pipeDesc.vertex.module = sm;
  pipeDesc.vertex.entryPoint.data = "vs_main";
  pipeDesc.vertex.entryPoint.length = WGPU_STRLEN;
  pipeDesc.vertex.bufferCount = 1;
  pipeDesc.vertex.buffers = &vbLayout;
  pipeDesc.primitive.topology = WGPUPrimitiveTopology_TriangleList;
  pipeDesc.primitive.frontFace = WGPUFrontFace_CCW;
  pipeDesc.primitive.cullMode = WGPUCullMode_None;
  pipeDesc.depthStencil = &depthState;
  pipeDesc.multisample.count = 1;
  pipeDesc.multisample.mask = 0xFFFFFFFFu;
  pipeDesc.fragment = &fs;
  WGPURenderPipeline pipe = wgpuDeviceCreateRenderPipeline(device, &pipeDesc);
  wgpuPipelineLayoutRelease(plLayout);
  if (!pipe) {
    WbLog::info("[WbWgpuRenderTarget] scn-agx CreateRenderPipeline failed");
    return false;
  }
  mScnAgxPipeline = pipe;
  return true;
#  else
  return false;
#  endif
#else
  return false;
#endif
}

// R4 step-3c-A.1: flat picking pipeline (kSolidPick). Identical to the lit/AgX
// pipelines (RGBA8 + Depth24Plus, same vertex + 1-entry bind-group layout); only
// the fragment shader differs (emits baseColor unshaded for an ID round-trip).
bool WbWgpuRenderTarget::ensurePickPipeline() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (mPickPipeline)
    return true;
  if (!ensureScenePipeline())  // creates mScnBindGroupLayout + mScnDepthView (shared)
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());

  WGPUShaderSourceWGSL wgsl = {};
  wgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
  wgsl.code.data = WbWgpuShaders::kSolidPick;
  wgsl.code.length = WGPU_STRLEN;
  WGPUShaderModuleDescriptor smDesc = {};
  smDesc.nextInChain = &wgsl.chain;
  WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &smDesc);
  if (!sm) {
    WbLog::info("[WbWgpuRenderTarget] pick CreateShaderModule failed");
    return false;
  }
  mPickShaderModule = sm;

  WGPUBindGroupLayout bgl = static_cast<WGPUBindGroupLayout>(mScnBindGroupLayout);
  WGPUPipelineLayoutDescriptor plDesc = {};
  plDesc.bindGroupLayoutCount = 1;
  plDesc.bindGroupLayouts = &bgl;
  WGPUPipelineLayout plLayout = wgpuDeviceCreatePipelineLayout(device, &plDesc);
  if (!plLayout) {
    WbLog::info("[WbWgpuRenderTarget] pick CreatePipelineLayout failed");
    return false;
  }

  WGPUVertexAttribute attrs[3] = {};
  attrs[0].format = WGPUVertexFormat_Float32x3; attrs[0].offset = 0;  attrs[0].shaderLocation = 0;
  attrs[1].format = WGPUVertexFormat_Float32x3; attrs[1].offset = 12; attrs[1].shaderLocation = 1;
  attrs[2].format = WGPUVertexFormat_Float32x2; attrs[2].offset = 24; attrs[2].shaderLocation = 2;
  WGPUVertexBufferLayout vbLayout = {};
  vbLayout.arrayStride = 32;
  vbLayout.stepMode = WGPUVertexStepMode_Vertex;
  vbLayout.attributeCount = 3;
  vbLayout.attributes = attrs;

  WGPUColorTargetState colorTarget = {};
  colorTarget.format = WGPUTextureFormat_RGBA8Unorm;  // linear — exact ID round-trip
  colorTarget.writeMask = WGPUColorWriteMask_All;

  WGPUFragmentState fs = {};
  fs.module = sm;
  fs.entryPoint.data = "fs_main";
  fs.entryPoint.length = WGPU_STRLEN;
  fs.targetCount = 1;
  fs.targets = &colorTarget;

  WGPUDepthStencilState depthState = {};
  depthState.format = WGPUTextureFormat_Depth24Plus;
  depthState.depthWriteEnabled = WGPUOptionalBool_True;
  depthState.depthCompare = WGPUCompareFunction_Less;
  depthState.stencilFront.compare = WGPUCompareFunction_Always;
  depthState.stencilFront.failOp = WGPUStencilOperation_Keep;
  depthState.stencilFront.depthFailOp = WGPUStencilOperation_Keep;
  depthState.stencilFront.passOp = WGPUStencilOperation_Keep;
  depthState.stencilBack = depthState.stencilFront;
  depthState.stencilReadMask = 0xFFFFFFFFu;
  depthState.stencilWriteMask = 0xFFFFFFFFu;

  WGPURenderPipelineDescriptor pipeDesc = {};
  pipeDesc.layout = plLayout;
  pipeDesc.vertex.module = sm;
  pipeDesc.vertex.entryPoint.data = "vs_main";
  pipeDesc.vertex.entryPoint.length = WGPU_STRLEN;
  pipeDesc.vertex.bufferCount = 1;
  pipeDesc.vertex.buffers = &vbLayout;
  pipeDesc.primitive.topology = WGPUPrimitiveTopology_TriangleList;
  pipeDesc.primitive.frontFace = WGPUFrontFace_CCW;
  pipeDesc.primitive.cullMode = WGPUCullMode_None;
  pipeDesc.depthStencil = &depthState;
  pipeDesc.multisample.count = 1;
  pipeDesc.multisample.mask = 0xFFFFFFFFu;
  pipeDesc.fragment = &fs;
  WGPURenderPipeline pipe = wgpuDeviceCreateRenderPipeline(device, &pipeDesc);
  wgpuPipelineLayoutRelease(plLayout);
  if (!pipe) {
    WbLog::info("[WbWgpuRenderTarget] pick CreateRenderPipeline failed");
    return false;
  }
  mPickPipeline = pipe;
  return true;
#  else
  return false;
#  endif
#else
  return false;
#endif
}

// R4 step-3c-A: line/wireframe pipeline. Identical to the pick pipeline (kSolidPick
// shader, mScnBindGroupLayout, RGBA8 target, Depth24Plus) EXCEPT the primitive
// topology is LineList — so a vertex buffer of segment endpoints draws colored lines.
bool WbWgpuRenderTarget::ensureLinePipeline() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (mLinePipeline)
    return true;
  if (!ensureScenePipeline())  // creates mScnBindGroupLayout + mScnDepthView (shared)
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());

  WGPUShaderSourceWGSL wgsl = {};
  wgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
  wgsl.code.data = WbWgpuShaders::kSolidPick;  // transform by viewProj*model, flat baseColor
  wgsl.code.length = WGPU_STRLEN;
  WGPUShaderModuleDescriptor smDesc = {};
  smDesc.nextInChain = &wgsl.chain;
  WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &smDesc);
  if (!sm) {
    WbLog::info("[WbWgpuRenderTarget] line CreateShaderModule failed");
    return false;
  }
  mLineShaderModule = sm;

  WGPUBindGroupLayout bgl = static_cast<WGPUBindGroupLayout>(mScnBindGroupLayout);
  WGPUPipelineLayoutDescriptor plDesc = {};
  plDesc.bindGroupLayoutCount = 1;
  plDesc.bindGroupLayouts = &bgl;
  WGPUPipelineLayout plLayout = wgpuDeviceCreatePipelineLayout(device, &plDesc);
  if (!plLayout) {
    WbLog::info("[WbWgpuRenderTarget] line CreatePipelineLayout failed");
    return false;
  }

  WGPUVertexAttribute attrs[3] = {};
  attrs[0].format = WGPUVertexFormat_Float32x3; attrs[0].offset = 0;  attrs[0].shaderLocation = 0;
  attrs[1].format = WGPUVertexFormat_Float32x3; attrs[1].offset = 12; attrs[1].shaderLocation = 1;
  attrs[2].format = WGPUVertexFormat_Float32x2; attrs[2].offset = 24; attrs[2].shaderLocation = 2;
  WGPUVertexBufferLayout vbLayout = {};
  vbLayout.arrayStride = 32;
  vbLayout.stepMode = WGPUVertexStepMode_Vertex;
  vbLayout.attributeCount = 3;
  vbLayout.attributes = attrs;

  WGPUColorTargetState colorTarget = {};
  colorTarget.format = WGPUTextureFormat_RGBA8Unorm;
  colorTarget.writeMask = WGPUColorWriteMask_All;

  WGPUFragmentState fs = {};
  fs.module = sm;
  fs.entryPoint.data = "fs_main";
  fs.entryPoint.length = WGPU_STRLEN;
  fs.targetCount = 1;
  fs.targets = &colorTarget;

  WGPUDepthStencilState depthState = {};
  depthState.format = WGPUTextureFormat_Depth24Plus;
  depthState.depthWriteEnabled = WGPUOptionalBool_True;
  depthState.depthCompare = WGPUCompareFunction_LessEqual;  // <= so lines coplanar with a face still show
  depthState.stencilFront.compare = WGPUCompareFunction_Always;
  depthState.stencilFront.failOp = WGPUStencilOperation_Keep;
  depthState.stencilFront.depthFailOp = WGPUStencilOperation_Keep;
  depthState.stencilFront.passOp = WGPUStencilOperation_Keep;
  depthState.stencilBack = depthState.stencilFront;
  depthState.stencilReadMask = 0xFFFFFFFFu;
  depthState.stencilWriteMask = 0xFFFFFFFFu;

  WGPURenderPipelineDescriptor pipeDesc = {};
  pipeDesc.layout = plLayout;
  pipeDesc.vertex.module = sm;
  pipeDesc.vertex.entryPoint.data = "vs_main";
  pipeDesc.vertex.entryPoint.length = WGPU_STRLEN;
  pipeDesc.vertex.bufferCount = 1;
  pipeDesc.vertex.buffers = &vbLayout;
  pipeDesc.primitive.topology = WGPUPrimitiveTopology_LineList;  // <-- the only real difference
  pipeDesc.primitive.frontFace = WGPUFrontFace_CCW;
  pipeDesc.primitive.cullMode = WGPUCullMode_None;
  pipeDesc.depthStencil = &depthState;
  pipeDesc.multisample.count = 1;
  pipeDesc.multisample.mask = 0xFFFFFFFFu;
  pipeDesc.fragment = &fs;
  WGPURenderPipeline pipe = wgpuDeviceCreateRenderPipeline(device, &pipeDesc);
  wgpuPipelineLayoutRelease(plLayout);
  if (!pipe) {
    WbLog::info("[WbWgpuRenderTarget] line CreateRenderPipeline failed");
    return false;
  }
  mLinePipeline = pipe;
  return true;
#  else
  return false;
#  endif
#else
  return false;
#endif
}

// R4 step-3c-A: always-on-top line pipeline. Identical to ensureLinePipeline EXCEPT
// depthCompare=Always + depthWrite=false, so markers (joint axes, COM, contact) that
// sit inside geometry stay visible instead of being occluded. Reuses mLineShaderModule.
bool WbWgpuRenderTarget::ensureLineNoDepthPipeline() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (mLineNoDepthPipeline)
    return true;
  if (!ensureLinePipeline())  // builds mLineShaderModule + mScnBindGroupLayout (reused)
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());

  WGPUBindGroupLayout bgl = static_cast<WGPUBindGroupLayout>(mScnBindGroupLayout);
  WGPUPipelineLayoutDescriptor plDesc = {};
  plDesc.bindGroupLayoutCount = 1;
  plDesc.bindGroupLayouts = &bgl;
  WGPUPipelineLayout plLayout = wgpuDeviceCreatePipelineLayout(device, &plDesc);
  if (!plLayout)
    return false;

  WGPUVertexAttribute attrs[3] = {};
  attrs[0].format = WGPUVertexFormat_Float32x3; attrs[0].offset = 0;  attrs[0].shaderLocation = 0;
  attrs[1].format = WGPUVertexFormat_Float32x3; attrs[1].offset = 12; attrs[1].shaderLocation = 1;
  attrs[2].format = WGPUVertexFormat_Float32x2; attrs[2].offset = 24; attrs[2].shaderLocation = 2;
  WGPUVertexBufferLayout vbLayout = {};
  vbLayout.arrayStride = 32;
  vbLayout.stepMode = WGPUVertexStepMode_Vertex;
  vbLayout.attributeCount = 3;
  vbLayout.attributes = attrs;

  WGPUColorTargetState colorTarget = {};
  colorTarget.format = WGPUTextureFormat_RGBA8Unorm;
  colorTarget.writeMask = WGPUColorWriteMask_All;

  WGPUFragmentState fs = {};
  fs.module = static_cast<WGPUShaderModule>(mLineShaderModule);
  fs.entryPoint.data = "fs_main";
  fs.entryPoint.length = WGPU_STRLEN;
  fs.targetCount = 1;
  fs.targets = &colorTarget;

  WGPUDepthStencilState depthState = {};
  depthState.format = WGPUTextureFormat_Depth24Plus;
  depthState.depthWriteEnabled = WGPUOptionalBool_False;       // don't write depth
  depthState.depthCompare = WGPUCompareFunction_Always;        // always draw on top
  depthState.stencilFront.compare = WGPUCompareFunction_Always;
  depthState.stencilFront.failOp = WGPUStencilOperation_Keep;
  depthState.stencilFront.depthFailOp = WGPUStencilOperation_Keep;
  depthState.stencilFront.passOp = WGPUStencilOperation_Keep;
  depthState.stencilBack = depthState.stencilFront;
  depthState.stencilReadMask = 0xFFFFFFFFu;
  depthState.stencilWriteMask = 0xFFFFFFFFu;

  WGPURenderPipelineDescriptor pipeDesc = {};
  pipeDesc.layout = plLayout;
  pipeDesc.vertex.module = static_cast<WGPUShaderModule>(mLineShaderModule);
  pipeDesc.vertex.entryPoint.data = "vs_main";
  pipeDesc.vertex.entryPoint.length = WGPU_STRLEN;
  pipeDesc.vertex.bufferCount = 1;
  pipeDesc.vertex.buffers = &vbLayout;
  pipeDesc.primitive.topology = WGPUPrimitiveTopology_LineList;
  pipeDesc.primitive.frontFace = WGPUFrontFace_CCW;
  pipeDesc.primitive.cullMode = WGPUCullMode_None;
  pipeDesc.depthStencil = &depthState;
  pipeDesc.multisample.count = 1;
  pipeDesc.multisample.mask = 0xFFFFFFFFu;
  pipeDesc.fragment = &fs;
  WGPURenderPipeline pipe = wgpuDeviceCreateRenderPipeline(device, &pipeDesc);
  wgpuPipelineLayoutRelease(plLayout);
  if (!pipe) {
    WbLog::info("[WbWgpuRenderTarget] line(no-depth) CreateRenderPipeline failed");
    return false;
  }
  mLineNoDepthPipeline = pipe;
  return true;
#  else
  return false;
#  endif
#else
  return false;
#endif
}

// R4 step-3c-A: full-screen overlay pipeline (kFullScreenOverlay) — alpha-blended flat
// colour, no vertex buffer (full-screen triangle from vertex_index), no depth.
bool WbWgpuRenderTarget::ensureOverlayPipeline() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (mOverlayPipeline)
    return true;
  if (!mUsable)
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());

  WGPUShaderSourceWGSL wgsl = {};
  wgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
  wgsl.code.data = WbWgpuShaders::kFullScreenOverlay;
  wgsl.code.length = WGPU_STRLEN;
  WGPUShaderModuleDescriptor smDesc = {};
  smDesc.nextInChain = &wgsl.chain;
  WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &smDesc);
  if (!sm) {
    WbLog::info("[WbWgpuRenderTarget] overlay CreateShaderModule failed");
    return false;
  }
  mOverlayShaderModule = sm;

  WGPUBindGroupLayoutEntry bglEntry = {};
  bglEntry.binding = 0;
  bglEntry.visibility = WGPUShaderStage_Fragment;
  bglEntry.buffer.type = WGPUBufferBindingType_Uniform;
  bglEntry.buffer.minBindingSize = 16;
  WGPUBindGroupLayoutDescriptor bglDesc = {};
  bglDesc.entryCount = 1;
  bglDesc.entries = &bglEntry;
  WGPUBindGroupLayout bgl = wgpuDeviceCreateBindGroupLayout(device, &bglDesc);
  if (!bgl)
    return false;
  mOverlayBindGroupLayout = bgl;

  WGPUPipelineLayoutDescriptor plDesc = {};
  plDesc.bindGroupLayoutCount = 1;
  plDesc.bindGroupLayouts = &bgl;
  WGPUPipelineLayout plLayout = wgpuDeviceCreatePipelineLayout(device, &plDesc);
  if (!plLayout)
    return false;

  // Straight-alpha "src over dst" blending.
  WGPUBlendState blend = {};
  blend.color.operation = WGPUBlendOperation_Add;
  blend.color.srcFactor = WGPUBlendFactor_SrcAlpha;
  blend.color.dstFactor = WGPUBlendFactor_OneMinusSrcAlpha;
  blend.alpha.operation = WGPUBlendOperation_Add;
  blend.alpha.srcFactor = WGPUBlendFactor_One;
  blend.alpha.dstFactor = WGPUBlendFactor_OneMinusSrcAlpha;
  WGPUColorTargetState colorTarget = {};
  colorTarget.format = WGPUTextureFormat_RGBA8Unorm;
  colorTarget.blend = &blend;
  colorTarget.writeMask = WGPUColorWriteMask_All;

  WGPUFragmentState fs = {};
  fs.module = sm;
  fs.entryPoint.data = "fs_main";
  fs.entryPoint.length = WGPU_STRLEN;
  fs.targetCount = 1;
  fs.targets = &colorTarget;

  WGPURenderPipelineDescriptor pipeDesc = {};
  pipeDesc.layout = plLayout;
  pipeDesc.vertex.module = sm;
  pipeDesc.vertex.entryPoint.data = "vs_main";
  pipeDesc.vertex.entryPoint.length = WGPU_STRLEN;
  pipeDesc.vertex.bufferCount = 0;  // full-screen triangle from vertex_index
  pipeDesc.primitive.topology = WGPUPrimitiveTopology_TriangleList;
  pipeDesc.primitive.frontFace = WGPUFrontFace_CCW;
  pipeDesc.primitive.cullMode = WGPUCullMode_None;
  pipeDesc.depthStencil = nullptr;  // 2D screen-space, no depth
  pipeDesc.multisample.count = 1;
  pipeDesc.multisample.mask = 0xFFFFFFFFu;
  pipeDesc.fragment = &fs;
  WGPURenderPipeline pipe = wgpuDeviceCreateRenderPipeline(device, &pipeDesc);
  wgpuPipelineLayoutRelease(plLayout);
  if (!pipe) {
    WbLog::info("[WbWgpuRenderTarget] overlay CreateRenderPipeline failed");
    return false;
  }
  mOverlayPipeline = pipe;
  return true;
#  else
  return false;
#  endif
#else
  return false;
#endif
}

bool WbWgpuRenderTarget::drawFullScreenOverlay(const float *color4, void *rgba8) {
  if (!mUsable || !rgba8 || !color4)
    return false;
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (!ensureOverlayPipeline())
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUQueue queue = static_cast<WGPUQueue>(mBackend->queue());

  if (!mOverlayUniformBuffer) {
    WGPUBufferDescriptor ud = {};
    ud.size = 16;
    ud.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
    mOverlayUniformBuffer = wgpuDeviceCreateBuffer(device, &ud);
    if (!mOverlayUniformBuffer)
      return false;
  }
  wgpuQueueWriteBuffer(queue, static_cast<WGPUBuffer>(mOverlayUniformBuffer), 0, color4, 16);

  WGPUBindGroupEntry e = {};
  e.binding = 0;
  e.buffer = static_cast<WGPUBuffer>(mOverlayUniformBuffer);
  e.offset = 0;
  e.size = 16;
  WGPUBindGroupDescriptor bd = {};
  bd.layout = static_cast<WGPUBindGroupLayout>(mOverlayBindGroupLayout);
  bd.entryCount = 1;
  bd.entries = &e;
  WGPUBindGroup bg = wgpuDeviceCreateBindGroup(device, &bd);
  if (!bg)
    return false;

  WGPUCommandEncoderDescriptor encDesc = {};
  WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(device, &encDesc);
  if (!encoder) {
    wgpuBindGroupRelease(bg);
    return false;
  }
  WGPURenderPassColorAttachment colorAttach = {};
  colorAttach.view = static_cast<WGPUTextureView>(mView);
  colorAttach.loadOp = WGPULoadOp_Load;  // composite over current contents
  colorAttach.storeOp = WGPUStoreOp_Store;
  colorAttach.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
  WGPURenderPassDescriptor passDesc = {};
  passDesc.colorAttachmentCount = 1;
  passDesc.colorAttachments = &colorAttach;  // no depth attachment (pipeline has none)
  WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &passDesc);
  if (!pass) {
    wgpuCommandEncoderRelease(encoder);
    wgpuBindGroupRelease(bg);
    return false;
  }
  wgpuRenderPassEncoderSetPipeline(pass, static_cast<WGPURenderPipeline>(mOverlayPipeline));
  wgpuRenderPassEncoderSetBindGroup(pass, 0, bg, 0, nullptr);
  wgpuRenderPassEncoderDraw(pass, 3, 1, 0, 0);  // full-screen triangle
  wgpuRenderPassEncoderEnd(pass);
  wgpuRenderPassEncoderRelease(pass);

  const uint32_t stride = alignedBytesPerRow(mWidth);
  WGPUTexelCopyTextureInfo src = {};
  src.texture = static_cast<WGPUTexture>(mTexture);
  src.aspect = WGPUTextureAspect_All;
  WGPUTexelCopyBufferInfo dst = {};
  dst.buffer = static_cast<WGPUBuffer>(mReadbackBuffer);
  dst.layout.bytesPerRow = stride;
  dst.layout.rowsPerImage = mHeight;
  WGPUExtent3D extent = {mWidth, mHeight, 1};
  wgpuCommandEncoderCopyTextureToBuffer(encoder, &src, &dst, &extent);

  WGPUCommandBufferDescriptor cmdDesc = {};
  WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(encoder, &cmdDesc);
  wgpuCommandEncoderRelease(encoder);
  if (!cmd) {
    wgpuBindGroupRelease(bg);
    return false;
  }
  wgpuQueueSubmit(queue, 1, &cmd);
  wgpuCommandBufferRelease(cmd);
  wgpuBindGroupRelease(bg);

  MapCapture cap;
  WGPUBufferMapCallbackInfo mapCb = {};
  mapCb.mode = WGPUCallbackMode_AllowSpontaneous;
  mapCb.callback = onMap;
  mapCb.userdata1 = &cap;
  (void)wgpuBufferMapAsync(static_cast<WGPUBuffer>(mReadbackBuffer), WGPUMapMode_Read, 0,
                           mReadbackBufferSize, mapCb);
  for (int i = 0; i < 1000 && !cap.done; ++i)
    wgpuDevicePoll(device, true, nullptr);
  if (!cap.done || !cap.ok)
    return false;
  const void *mapped = wgpuBufferGetConstMappedRange(static_cast<WGPUBuffer>(mReadbackBuffer), 0,
                                                     mReadbackBufferSize);
  if (!mapped) {
    wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
    return false;
  }
  const uint32_t rowBytes = mWidth * 4u;
  uint8_t *out = static_cast<uint8_t *>(rgba8);
  const uint8_t *in = static_cast<const uint8_t *>(mapped);
  for (uint32_t y = 0; y < mHeight; ++y)
    std::memcpy(out + y * rowBytes, in + y * stride, rowBytes);
  wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
  return true;
#  else
  (void)color4;
  (void)rgba8;
  return false;
#  endif
#else
  (void)color4;
  (void)rgba8;
  return false;
#endif
}

bool WbWgpuRenderTarget::ensureTexturedQuadPipeline() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (mTexQuadPipeline)
    return true;
  if (!mUsable)
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());

  WGPUShaderSourceWGSL wgsl = {};
  wgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
  wgsl.code.data = WbWgpuShaders::kTexturedQuad;
  wgsl.code.length = WGPU_STRLEN;
  WGPUShaderModuleDescriptor smDesc = {};
  smDesc.nextInChain = &wgsl.chain;
  WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &smDesc);
  if (!sm) {
    WbLog::info("[WbWgpuRenderTarget] tex-quad CreateShaderModule failed");
    return false;
  }
  mTexQuadShaderModule = sm;

  WGPUSamplerDescriptor sampDesc = {};
  sampDesc.addressModeU = WGPUAddressMode_ClampToEdge;
  sampDesc.addressModeV = WGPUAddressMode_ClampToEdge;
  sampDesc.addressModeW = WGPUAddressMode_ClampToEdge;
  sampDesc.magFilter = WGPUFilterMode_Linear;
  sampDesc.minFilter = WGPUFilterMode_Linear;
  sampDesc.mipmapFilter = WGPUMipmapFilterMode_Nearest;
  sampDesc.maxAnisotropy = 1;
  WGPUSampler samp = wgpuDeviceCreateSampler(device, &sampDesc);
  if (!samp)
    return false;
  mTexQuadSampler = samp;

  WGPUBindGroupLayoutEntry bglEntries[3] = {};
  bglEntries[0].binding = 0;
  bglEntries[0].visibility = WGPUShaderStage_Vertex;  // rect used in the vertex stage
  bglEntries[0].buffer.type = WGPUBufferBindingType_Uniform;
  bglEntries[0].buffer.minBindingSize = 16;
  bglEntries[1].binding = 1;
  bglEntries[1].visibility = WGPUShaderStage_Fragment;
  bglEntries[1].texture.sampleType = WGPUTextureSampleType_Float;
  bglEntries[1].texture.viewDimension = WGPUTextureViewDimension_2D;
  bglEntries[1].texture.multisampled = 0;
  bglEntries[2].binding = 2;
  bglEntries[2].visibility = WGPUShaderStage_Fragment;
  bglEntries[2].sampler.type = WGPUSamplerBindingType_Filtering;
  WGPUBindGroupLayoutDescriptor bglDesc = {};
  bglDesc.entryCount = 3;
  bglDesc.entries = bglEntries;
  WGPUBindGroupLayout bgl = wgpuDeviceCreateBindGroupLayout(device, &bglDesc);
  if (!bgl)
    return false;
  mTexQuadBindGroupLayout = bgl;

  WGPUPipelineLayoutDescriptor plDesc = {};
  plDesc.bindGroupLayoutCount = 1;
  plDesc.bindGroupLayouts = &bgl;
  WGPUPipelineLayout plLayout = wgpuDeviceCreatePipelineLayout(device, &plDesc);
  if (!plLayout)
    return false;

  WGPUBlendState blend = {};  // straight-alpha src-over (opaque texture → just overwrites)
  blend.color.operation = WGPUBlendOperation_Add;
  blend.color.srcFactor = WGPUBlendFactor_SrcAlpha;
  blend.color.dstFactor = WGPUBlendFactor_OneMinusSrcAlpha;
  blend.alpha.operation = WGPUBlendOperation_Add;
  blend.alpha.srcFactor = WGPUBlendFactor_One;
  blend.alpha.dstFactor = WGPUBlendFactor_OneMinusSrcAlpha;
  WGPUColorTargetState colorTarget = {};
  colorTarget.format = WGPUTextureFormat_RGBA8Unorm;
  colorTarget.blend = &blend;
  colorTarget.writeMask = WGPUColorWriteMask_All;

  WGPUFragmentState fs = {};
  fs.module = sm;
  fs.entryPoint.data = "fs_main";
  fs.entryPoint.length = WGPU_STRLEN;
  fs.targetCount = 1;
  fs.targets = &colorTarget;

  WGPURenderPipelineDescriptor pipeDesc = {};
  pipeDesc.layout = plLayout;
  pipeDesc.vertex.module = sm;
  pipeDesc.vertex.entryPoint.data = "vs_main";
  pipeDesc.vertex.entryPoint.length = WGPU_STRLEN;
  pipeDesc.vertex.bufferCount = 0;  // quad generated from vertex_index + rect uniform
  pipeDesc.primitive.topology = WGPUPrimitiveTopology_TriangleList;
  pipeDesc.primitive.frontFace = WGPUFrontFace_CCW;
  pipeDesc.primitive.cullMode = WGPUCullMode_None;
  pipeDesc.depthStencil = nullptr;
  pipeDesc.multisample.count = 1;
  pipeDesc.multisample.mask = 0xFFFFFFFFu;
  pipeDesc.fragment = &fs;
  WGPURenderPipeline pipe = wgpuDeviceCreateRenderPipeline(device, &pipeDesc);
  wgpuPipelineLayoutRelease(plLayout);
  if (!pipe) {
    WbLog::info("[WbWgpuRenderTarget] tex-quad CreateRenderPipeline failed");
    return false;
  }
  mTexQuadPipeline = pipe;
  return true;
#  else
  return false;
#  endif
#else
  return false;
#endif
}

bool WbWgpuRenderTarget::drawTexturedInset(void *textureView, const float ndcRect[4], void *rgba8) {
  if (!mUsable || !rgba8 || !textureView || !ndcRect)
    return false;
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (!ensureTexturedQuadPipeline())
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUQueue queue = static_cast<WGPUQueue>(mBackend->queue());

  if (!mTexQuadUniformBuffer) {
    WGPUBufferDescriptor ud = {};
    ud.size = 16;
    ud.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
    mTexQuadUniformBuffer = wgpuDeviceCreateBuffer(device, &ud);
    if (!mTexQuadUniformBuffer)
      return false;
  }
  wgpuQueueWriteBuffer(queue, static_cast<WGPUBuffer>(mTexQuadUniformBuffer), 0, ndcRect, 16);

  WGPUBindGroupEntry e[3] = {};
  e[0].binding = 0;
  e[0].buffer = static_cast<WGPUBuffer>(mTexQuadUniformBuffer);
  e[0].offset = 0;
  e[0].size = 16;
  e[1].binding = 1;
  e[1].textureView = static_cast<WGPUTextureView>(textureView);
  e[2].binding = 2;
  e[2].sampler = static_cast<WGPUSampler>(mTexQuadSampler);
  WGPUBindGroupDescriptor bd = {};
  bd.layout = static_cast<WGPUBindGroupLayout>(mTexQuadBindGroupLayout);
  bd.entryCount = 3;
  bd.entries = e;
  WGPUBindGroup bg = wgpuDeviceCreateBindGroup(device, &bd);
  if (!bg)
    return false;

  WGPUCommandEncoderDescriptor encDesc = {};
  WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(device, &encDesc);
  if (!encoder) {
    wgpuBindGroupRelease(bg);
    return false;
  }
  WGPURenderPassColorAttachment colorAttach = {};
  colorAttach.view = static_cast<WGPUTextureView>(mView);
  colorAttach.loadOp = WGPULoadOp_Load;  // composite over current contents
  colorAttach.storeOp = WGPUStoreOp_Store;
  colorAttach.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
  WGPURenderPassDescriptor passDesc = {};
  passDesc.colorAttachmentCount = 1;
  passDesc.colorAttachments = &colorAttach;
  WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &passDesc);
  if (!pass) {
    wgpuCommandEncoderRelease(encoder);
    wgpuBindGroupRelease(bg);
    return false;
  }
  wgpuRenderPassEncoderSetPipeline(pass, static_cast<WGPURenderPipeline>(mTexQuadPipeline));
  wgpuRenderPassEncoderSetBindGroup(pass, 0, bg, 0, nullptr);
  wgpuRenderPassEncoderDraw(pass, 6, 1, 0, 0);  // 2 triangles
  wgpuRenderPassEncoderEnd(pass);
  wgpuRenderPassEncoderRelease(pass);

  const uint32_t stride = alignedBytesPerRow(mWidth);
  WGPUTexelCopyTextureInfo src = {};
  src.texture = static_cast<WGPUTexture>(mTexture);
  src.aspect = WGPUTextureAspect_All;
  WGPUTexelCopyBufferInfo dst = {};
  dst.buffer = static_cast<WGPUBuffer>(mReadbackBuffer);
  dst.layout.bytesPerRow = stride;
  dst.layout.rowsPerImage = mHeight;
  WGPUExtent3D extent = {mWidth, mHeight, 1};
  wgpuCommandEncoderCopyTextureToBuffer(encoder, &src, &dst, &extent);

  WGPUCommandBufferDescriptor cmdDesc = {};
  WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(encoder, &cmdDesc);
  wgpuCommandEncoderRelease(encoder);
  if (!cmd) {
    wgpuBindGroupRelease(bg);
    return false;
  }
  wgpuQueueSubmit(queue, 1, &cmd);
  wgpuCommandBufferRelease(cmd);
  wgpuBindGroupRelease(bg);

  MapCapture cap;
  WGPUBufferMapCallbackInfo mapCb = {};
  mapCb.mode = WGPUCallbackMode_AllowSpontaneous;
  mapCb.callback = onMap;
  mapCb.userdata1 = &cap;
  (void)wgpuBufferMapAsync(static_cast<WGPUBuffer>(mReadbackBuffer), WGPUMapMode_Read, 0,
                           mReadbackBufferSize, mapCb);
  for (int i = 0; i < 1000 && !cap.done; ++i)
    wgpuDevicePoll(device, true, nullptr);
  if (!cap.done || !cap.ok)
    return false;
  const void *mapped = wgpuBufferGetConstMappedRange(static_cast<WGPUBuffer>(mReadbackBuffer), 0,
                                                     mReadbackBufferSize);
  if (!mapped) {
    wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
    return false;
  }
  const uint32_t rowBytes = mWidth * 4u;
  uint8_t *out = static_cast<uint8_t *>(rgba8);
  const uint8_t *in = static_cast<const uint8_t *>(mapped);
  for (uint32_t y = 0; y < mHeight; ++y)
    std::memcpy(out + y * rowBytes, in + y * stride, rowBytes);
  wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
  return true;
#  else
  (void)textureView;
  (void)ndcRect;
  (void)rgba8;
  return false;
#  endif
#else
  (void)textureView;
  (void)ndcRect;
  (void)rgba8;
  return false;
#endif
}

// T1.2 CSM sub-step 3b: build the shadow-receiving lit pipeline (kSolidLitShadow).
// Unlike the other scene pipelines (1-entry mScnBindGroupLayout), this has its
// R4 material fidelity: textured-lit pipeline (kSolidLitTextured). Its OWN 6-entry layout:
// Scene uniform @0 (dynamic, 192 B) + filterable albedo @1 + roughness @2 + metalness @3 +
// normal @4 + linear sampler @5. Absent maps bind 1×1 defaults (white/black/flat) so an
// albedo-only draw is byte-identical. Reuses ensureScenePipeline's Depth24Plus depth view;
// RGBA8Unorm target. Built lazily on the first textured draw.
bool WbWgpuRenderTarget::ensureSceneTexPipeline() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (mScnTexPipeline)
    return true;
  if (!ensureScenePipeline())  // mScnDepthView + the shared uniform buffer
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());

  WGPUShaderSourceWGSL wgsl = {};
  wgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
  wgsl.code.data = WbWgpuShaders::kSolidLitTextured;
  wgsl.code.length = WGPU_STRLEN;
  WGPUShaderModuleDescriptor smDesc = {};
  smDesc.nextInChain = &wgsl.chain;
  WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &smDesc);
  if (!sm) {
    WbLog::info("[WbWgpuRenderTarget] scn-tex CreateShaderModule failed");
    return false;
  }
  mScnTexShaderModule = sm;

  WGPUSamplerDescriptor sampDesc = {};
  sampDesc.addressModeU = WGPUAddressMode_Repeat;
  sampDesc.addressModeV = WGPUAddressMode_Repeat;
  sampDesc.addressModeW = WGPUAddressMode_Repeat;
  sampDesc.magFilter = WGPUFilterMode_Linear;
  sampDesc.minFilter = WGPUFilterMode_Linear;
  sampDesc.mipmapFilter = WGPUMipmapFilterMode_Linear;
  sampDesc.lodMinClamp = 0.0f;
  // Full trilinear: textures now upload complete mip chains (WbWgpuTextureCache); the old 1.0 clamp
  // pinned sampling to the top level, so minified textures (the aerial city's roads) shimmered with
  // aliasing noise WREN doesn't have.
  sampDesc.lodMaxClamp = 32.0f;
  sampDesc.compare = WGPUCompareFunction_Undefined;
  // Anisotropic filtering: trilinear alone blurs grazing-angle textures (street markings/asphalt
  // receding into the distance); 8x keeps them crisp. Requires all three filters Linear (they are).
  sampDesc.maxAnisotropy = 8;
  WGPUSampler samp = wgpuDeviceCreateSampler(device, &sampDesc);
  if (!samp) {
    WbLog::info("[WbWgpuRenderTarget] scn-tex CreateSampler failed");
    return false;
  }
  mScnTexSampler = samp;

  // 1×1 default textures for the map slots a draw may lack — chosen so an albedo-only draw
  // is byte-identical: white roughness (factor unchanged), black metalness (dielectric),
  // flat normal (128,128,255 → tangent +Z → no perturbation).
  auto make1x1 = [&](uint8_t r, uint8_t g, uint8_t b, const char *tag, WGPUTexture &tex,
                     WGPUTextureView &view) -> bool {
    WGPUTextureDescriptor td = {};
    td.dimension = WGPUTextureDimension_2D;
    td.size = {1, 1, 1};
    td.format = WGPUTextureFormat_RGBA8Unorm;
    td.mipLevelCount = 1;
    td.sampleCount = 1;
    td.usage = WGPUTextureUsage_TextureBinding | WGPUTextureUsage_CopyDst;
    tex = wgpuDeviceCreateTexture(device, &td);
    if (!tex) {
      WbLog::info("[WbWgpuRenderTarget] scn-tex default CreateTexture failed");
      return false;
    }
    const uint8_t px[4] = {r, g, b, 255};
    WGPUTexelCopyTextureInfo dst = {};
    dst.texture = tex;
    dst.aspect = WGPUTextureAspect_All;
    WGPUTexelCopyBufferLayout lay = {};
    lay.bytesPerRow = 4;
    lay.rowsPerImage = 1;
    WGPUExtent3D ext = {1, 1, 1};
    wgpuQueueWriteTexture(static_cast<WGPUQueue>(mBackend->queue()), &dst, px, 4, &lay, &ext);
    WGPUTextureViewDescriptor vd = {};
    vd.format = WGPUTextureFormat_RGBA8Unorm;
    vd.dimension = WGPUTextureViewDimension_2D;
    vd.mipLevelCount = 1;
    vd.arrayLayerCount = 1;
    view = wgpuTextureCreateView(tex, &vd);
    (void)tag;
    return view != nullptr;
  };
  {
    WGPUTexture wt = nullptr, bt = nullptr, nt = nullptr;
    WGPUTextureView wv = nullptr, bv = nullptr, nv = nullptr;
    if (!make1x1(255, 255, 255, "white", wt, wv) || !make1x1(0, 0, 0, "black", bt, bv) ||
        !make1x1(128, 128, 255, "normal", nt, nv)) {
      WbLog::info("[WbWgpuRenderTarget] scn-tex default-texture build failed");
      return false;
    }
    mScnDefaultWhiteTex = wt;
    mScnDefaultWhiteView = wv;
    mScnDefaultBlackTex = bt;
    mScnDefaultBlackView = bv;
    mScnDefaultNormalTex = nt;
    mScnDefaultNormalView = nv;
  }

  WGPUBindGroupLayoutEntry bglEntries[6] = {};
  bglEntries[0].binding = 0;
  bglEntries[0].visibility = WGPUShaderStage_Vertex | WGPUShaderStage_Fragment;
  bglEntries[0].buffer.type = WGPUBufferBindingType_Uniform;
  bglEntries[0].buffer.hasDynamicOffset = 1;
  bglEntries[0].buffer.minBindingSize = 192;
  for (int t = 1; t <= 4; ++t) {  // albedo@1, roughness@2, metalness@3, normal@4
    bglEntries[t].binding = static_cast<uint32_t>(t);
    bglEntries[t].visibility = WGPUShaderStage_Fragment;
    bglEntries[t].texture.sampleType = WGPUTextureSampleType_Float;  // filterable RGBA8
    bglEntries[t].texture.viewDimension = WGPUTextureViewDimension_2D;
    bglEntries[t].texture.multisampled = 0;
  }
  bglEntries[5].binding = 5;
  bglEntries[5].visibility = WGPUShaderStage_Fragment;
  bglEntries[5].sampler.type = WGPUSamplerBindingType_Filtering;
  WGPUBindGroupLayoutDescriptor bglDesc = {};
  bglDesc.entryCount = 6;
  bglDesc.entries = bglEntries;
  WGPUBindGroupLayout bgl = wgpuDeviceCreateBindGroupLayout(device, &bglDesc);
  if (!bgl) {
    WbLog::info("[WbWgpuRenderTarget] scn-tex CreateBindGroupLayout failed");
    return false;
  }
  mScnTexBindGroupLayout = bgl;

  WGPUPipelineLayoutDescriptor plDesc = {};
  plDesc.bindGroupLayoutCount = 1;
  plDesc.bindGroupLayouts = &bgl;
  WGPUPipelineLayout plLayout = wgpuDeviceCreatePipelineLayout(device, &plDesc);
  if (!plLayout)
    return false;

  WGPUVertexAttribute attrs[3] = {};
  attrs[0].format = WGPUVertexFormat_Float32x3; attrs[0].offset = 0;  attrs[0].shaderLocation = 0;
  attrs[1].format = WGPUVertexFormat_Float32x3; attrs[1].offset = 12; attrs[1].shaderLocation = 1;
  attrs[2].format = WGPUVertexFormat_Float32x2; attrs[2].offset = 24; attrs[2].shaderLocation = 2;
  WGPUVertexBufferLayout vbLayout = {};
  vbLayout.arrayStride = 32;
  vbLayout.stepMode = WGPUVertexStepMode_Vertex;
  vbLayout.attributeCount = 3;
  vbLayout.attributes = attrs;

  WGPUColorTargetState colorTarget = {};
  colorTarget.format = WGPUTextureFormat_RGBA8Unorm;
  colorTarget.writeMask = WGPUColorWriteMask_All;

  WGPUFragmentState fs = {};
  fs.module = sm;
  fs.entryPoint.data = "fs_main";
  fs.entryPoint.length = WGPU_STRLEN;
  fs.targetCount = 1;
  fs.targets = &colorTarget;

  WGPUDepthStencilState depthState = {};
  depthState.format = WGPUTextureFormat_Depth24Plus;
  depthState.depthWriteEnabled = WGPUOptionalBool_True;
  depthState.depthCompare = WGPUCompareFunction_Less;
  depthState.stencilFront.compare = WGPUCompareFunction_Always;
  depthState.stencilFront.failOp = WGPUStencilOperation_Keep;
  depthState.stencilFront.depthFailOp = WGPUStencilOperation_Keep;
  depthState.stencilFront.passOp = WGPUStencilOperation_Keep;
  depthState.stencilBack = depthState.stencilFront;
  depthState.stencilReadMask = 0xFFFFFFFFu;
  depthState.stencilWriteMask = 0xFFFFFFFFu;

  WGPURenderPipelineDescriptor pipeDesc = {};
  pipeDesc.layout = plLayout;
  pipeDesc.vertex.module = sm;
  pipeDesc.vertex.entryPoint.data = "vs_main";
  pipeDesc.vertex.entryPoint.length = WGPU_STRLEN;
  pipeDesc.vertex.bufferCount = 1;
  pipeDesc.vertex.buffers = &vbLayout;
  pipeDesc.primitive.topology = WGPUPrimitiveTopology_TriangleList;
  pipeDesc.primitive.frontFace = WGPUFrontFace_CCW;
  pipeDesc.primitive.cullMode = WGPUCullMode_None;
  pipeDesc.depthStencil = &depthState;
  pipeDesc.multisample.count = 1;
  pipeDesc.multisample.mask = 0xFFFFFFFFu;
  pipeDesc.fragment = &fs;
  WGPURenderPipeline pipe = wgpuDeviceCreateRenderPipeline(device, &pipeDesc);
  wgpuPipelineLayoutRelease(plLayout);
  if (!pipe) {
    WbLog::info("[WbWgpuRenderTarget] scn-tex CreateRenderPipeline failed");
    return false;
  }
  mScnTexPipeline = pipe;
  return true;
#  else
  return false;
#  endif
#else
  return false;
#endif
}

bool WbWgpuRenderTarget::selfTestTextured(unsigned char albedoOut[3], unsigned char specOut[3],
                                          unsigned char metalOut[3], unsigned char normalOut[3]) {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (!mUsable || !ensureSceneTexPipeline())
    return false;  // pipeline build = naga-validates the 6-binding WGSL (the dominant risk)
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUQueue queue = static_cast<WGPUQueue>(mBackend->queue());

  // Two 1×1 textures: a known albedo (0.1, 0.3, 0.5) and a black roughness (forces
  // effectiveRoughness→0 → max specular). Created/destroyed locally (test-only).
  auto make1x1 = [&](uint8_t r, uint8_t g, uint8_t b, WGPUTexture &tex, WGPUTextureView &view) -> bool {
    WGPUTextureDescriptor td = {};
    td.dimension = WGPUTextureDimension_2D;
    td.size = {1, 1, 1};
    td.format = WGPUTextureFormat_RGBA8Unorm;
    td.mipLevelCount = 1;
    td.sampleCount = 1;
    td.usage = WGPUTextureUsage_TextureBinding | WGPUTextureUsage_CopyDst;
    tex = wgpuDeviceCreateTexture(device, &td);
    if (!tex)
      return false;
    const uint8_t px[4] = {r, g, b, 255};
    WGPUTexelCopyTextureInfo dst = {};
    dst.texture = tex;
    dst.aspect = WGPUTextureAspect_All;
    WGPUTexelCopyBufferLayout lay = {};
    lay.bytesPerRow = 4;
    lay.rowsPerImage = 1;
    WGPUExtent3D ext = {1, 1, 1};
    wgpuQueueWriteTexture(queue, &dst, px, 4, &lay, &ext);
    WGPUTextureViewDescriptor vd = {};
    vd.format = WGPUTextureFormat_RGBA8Unorm;
    vd.dimension = WGPUTextureViewDimension_2D;
    vd.mipLevelCount = 1;
    vd.arrayLayerCount = 1;
    view = wgpuTextureCreateView(tex, &vd);
    return view != nullptr;
  };
  WGPUTexture albTex = nullptr, roughTex = nullptr, metalTex = nullptr, tiltTex = nullptr;
  WGPUTextureView albView = nullptr, roughView = nullptr, metalView = nullptr, tiltView = nullptr;
  bool texOk = make1x1(26, 77, 128, albTex, albView) && make1x1(0, 0, 0, roughTex, roughView) &&
               make1x1(255, 255, 255, metalTex, metalView) &&
               make1x1(255, 128, 255, tiltTex, tiltView);  // normalMap (1,0,1)→45° tangent tilt

  // Full-screen quad (pos3 + normal3 + uv2, stride 32), normal toward the camera.
  const float quad[32] = {
    -0.8f, -0.8f, 0.0f, 0, 0, 1, 0, 0,  0.8f, -0.8f, 0.0f, 0, 0, 1, 1, 0,
     0.8f,  0.8f, 0.0f, 0, 0, 1, 1, 1, -0.8f,  0.8f, 0.0f, 0, 0, 1, 0, 1,
  };
  const uint32_t idx[6] = {0, 1, 2, 0, 2, 3};
  WGPUBufferDescriptor vbDesc = {};
  vbDesc.usage = WGPUBufferUsage_Vertex | WGPUBufferUsage_CopyDst;
  vbDesc.size = sizeof(quad);
  WGPUBuffer vbuf = wgpuDeviceCreateBuffer(device, &vbDesc);
  WGPUBufferDescriptor ibDesc = {};
  ibDesc.usage = WGPUBufferUsage_Index | WGPUBufferUsage_CopyDst;
  ibDesc.size = sizeof(idx);
  WGPUBuffer ibuf = wgpuDeviceCreateBuffer(device, &ibDesc);
  if (vbuf)
    wgpuQueueWriteBuffer(queue, vbuf, 0, quad, sizeof(quad));
  if (ibuf)
    wgpuQueueWriteBuffer(queue, ibuf, 0, idx, sizeof(idx));

  bool ok = texOk && vbuf && ibuf;
  if (ok) {
    const float identity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    const float light[4] = {0.0f, 0.0f, -1.0f, 1.0f};  // dir + ambient=1 (intensity saturates)
    const float camPos[3] = {0.0f, 0.0f, 5.0f};        // head-on → guaranteed specular highlight
    WbWgpuClearColor black;
    black.r = black.g = black.b = 0.0f;
    WbWgpuSolidDraw d;
    d.modelMatrix16 = identity;
    d.baseColorR = d.baseColorG = d.baseColorB = d.baseColorA = 1.0f;
    d.specularStrength = 0.4f;  // pad1.w → with white roughness, specStr stays 0.4
    d.vertexBuffer = vbuf;
    d.indexBuffer = ibuf;
    d.indexCount = 6;
    d.textureView = albView;
    std::vector<unsigned char> buf(static_cast<size_t>(mWidth) * mHeight * 4, 0);
    const size_t c = (static_cast<size_t>(mHeight / 2) * mWidth + mWidth / 2) * 4;
    // Render A: default-white roughness (roughnessView null) → specStr = 0.4.
    d.roughnessView = nullptr;
    ok = clearAndDrawScene(black, identity, light, &d, 1, buf.data(), false, 1.0f, false, camPos, false);
    if (ok) {
      albedoOut[0] = buf[c];
      albedoOut[1] = buf[c + 1];
      albedoOut[2] = buf[c + 2];
    }
    // Render B: black roughness → effRough 0 → specStr 1.0 → brighter (white) highlight.
    d.roughnessView = roughView;
    std::memset(buf.data(), 0, buf.size());
    ok = ok && clearAndDrawScene(black, identity, light, &d, 1, buf.data(), false, 1.0f, false, camPos, false);
    if (ok) {
      specOut[0] = buf[c];
      specOut[1] = buf[c + 1];
      specOut[2] = buf[c + 2];
    }
    // Render C: metalness = 1 with the default-white (moderately rough) roughness — under the
    // GGX BRDF a perfectly-smooth metal saturates to white, so a moderate roughness lets the
    // albedo tint through. A metal has NO diffuse and tints its specular by the albedo (F0 =
    // albedo) → the centre comes out albedo-coloured (blue-dominant), NOT white. Proves metalness.
    d.metalnessView = metalView;
    d.roughnessView = nullptr;  // default-white → effRough ≈ 0.6 (moderate)
    std::memset(buf.data(), 0, buf.size());
    ok = ok && clearAndDrawScene(black, identity, light, &d, 1, buf.data(), false, 1.0f, false, camPos, false);
    if (ok) {
      metalOut[0] = buf[c];
      metalOut[1] = buf[c + 1];
      metalOut[2] = buf[c + 2];
    }
    // Render D: dielectric (metal back to default-black) + the SAME smooth black roughness as
    // render B + a TILTED normal map. Same lighting/roughness as B, only the normal differs —
    // the perturbed normal no longer faces the half-vector head-on, so B's sharp white highlight
    // collapses → markedly dimmer. Isolates + proves the normalMap perturbation.
    d.metalnessView = nullptr;
    d.roughnessView = roughView;  // black/smooth, matching render B (isolate the normal effect)
    d.normalView = tiltView;
    std::memset(buf.data(), 0, buf.size());
    ok = ok && clearAndDrawScene(black, identity, light, &d, 1, buf.data(), false, 1.0f, false, camPos, false);
    if (ok) {
      normalOut[0] = buf[c];
      normalOut[1] = buf[c + 1];
      normalOut[2] = buf[c + 2];
    }
  }

  if (albView) wgpuTextureViewRelease(albView);
  if (albTex) wgpuTextureRelease(albTex);
  if (roughView) wgpuTextureViewRelease(roughView);
  if (roughTex) wgpuTextureRelease(roughTex);
  if (metalView) wgpuTextureViewRelease(metalView);
  if (metalTex) wgpuTextureRelease(metalTex);
  if (tiltView) wgpuTextureViewRelease(tiltView);
  if (tiltTex) wgpuTextureRelease(tiltTex);
  if (vbuf) wgpuBufferRelease(vbuf);
  if (ibuf) wgpuBufferRelease(ibuf);
  return ok;
#  else
  (void)albedoOut;
  (void)specOut;
  (void)metalOut;
  (void)normalOut;
  return false;
#  endif
#else
  (void)albedoOut;
  (void)specOut;
  (void)metalOut;
  (void)normalOut;
  return false;
#endif
}

bool WbWgpuRenderTarget::selfTestInset(unsigned char insideOut[3], unsigned char outsideOut[3]) {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (!mUsable || !ensureTexturedQuadPipeline())
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUQueue queue = static_cast<WGPUQueue>(mBackend->queue());

  // Known RED 1×1 texture (the stand-in for a device image).
  WGPUTexture redTex = nullptr;
  WGPUTextureView redView = nullptr;
  {
    WGPUTextureDescriptor td = {};
    td.dimension = WGPUTextureDimension_2D;
    td.size = {1, 1, 1};
    td.format = WGPUTextureFormat_RGBA8Unorm;
    td.mipLevelCount = 1;
    td.sampleCount = 1;
    td.usage = WGPUTextureUsage_TextureBinding | WGPUTextureUsage_CopyDst;
    redTex = wgpuDeviceCreateTexture(device, &td);
    if (!redTex)
      return false;
    const uint8_t px[4] = {255, 0, 0, 255};
    WGPUTexelCopyTextureInfo dst = {};
    dst.texture = redTex;
    dst.aspect = WGPUTextureAspect_All;
    WGPUTexelCopyBufferLayout lay = {};
    lay.bytesPerRow = 4;
    lay.rowsPerImage = 1;
    WGPUExtent3D ext = {1, 1, 1};
    wgpuQueueWriteTexture(queue, &dst, px, 4, &lay, &ext);
    WGPUTextureViewDescriptor vd = {};
    vd.format = WGPUTextureFormat_RGBA8Unorm;
    vd.dimension = WGPUTextureViewDimension_2D;
    vd.mipLevelCount = 1;
    vd.arrayLayerCount = 1;
    redView = wgpuTextureCreateView(redTex, &vd);
  }

  const float identity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  const float light[4] = {0.0f, 0.0f, -1.0f, 1.0f};
  WbWgpuClearColor navy;
  navy.r = 0.0f;
  navy.g = 0.0f;
  navy.b = 0.2f;  // (0,0,51) — distinct from red
  std::vector<unsigned char> buf(static_cast<size_t>(mWidth) * mHeight * 4, 0);
  // Clear the target to navy (0 draws), then composite the red texture into a top-right rect.
  bool ok = clearAndDrawScene(navy, identity, light, nullptr, 0, buf.data());
  const float rect[4] = {0.2f, 0.2f, 0.95f, 0.95f};  // NDC top-right sub-rect
  ok = ok && redView && drawTexturedInset(redView, rect, buf.data());
  if (ok) {
    // Inside the inset (NDC ~0.575,0.575) → red; outside (NDC -0.8,-0.8) → navy. y is flipped.
    auto px = [&](double ndcx, double ndcy) -> size_t {
      const int x = static_cast<int>((ndcx + 1.0) * 0.5 * mWidth);
      const int y = static_cast<int>((1.0 - (ndcy + 1.0) * 0.5) * mHeight);
      const int cx = x < 0 ? 0 : (x >= static_cast<int>(mWidth) ? static_cast<int>(mWidth) - 1 : x);
      const int cy = y < 0 ? 0 : (y >= static_cast<int>(mHeight) ? static_cast<int>(mHeight) - 1 : y);
      return (static_cast<size_t>(cy) * mWidth + cx) * 4;
    };
    const size_t in = px(0.575, 0.575), out = px(-0.8, -0.8);
    insideOut[0] = buf[in];
    insideOut[1] = buf[in + 1];
    insideOut[2] = buf[in + 2];
    outsideOut[0] = buf[out];
    outsideOut[1] = buf[out + 1];
    outsideOut[2] = buf[out + 2];
  }

  if (redView) wgpuTextureViewRelease(redView);
  if (redTex) wgpuTextureRelease(redTex);
  return ok;
#  else
  (void)insideOut;
  (void)outsideOut;
  return false;
#  endif
#else
  (void)insideOut;
  (void)outsideOut;
  return false;
#endif
}

// R4 lighting rung 1: textured+shadowed pipeline (kSolidLitTexturedShadow). 9-entry layout —
// Scene uniform @0 (dynamic, 192 B) + albedo/rough/metal/normal @1–4 (filterable) + filtering
// sampler @5 + shadow map @6 (unfilterable R32Float) + non-filtering shadow sampler @7 + shared
// LightU uniform @8 (lightViewProj + shadowParams, 80 B). Reuses ensureSceneTexPipeline's sampler
// + ensureSceneShadowPipeline's mShadowMapView/mScnShadowSampler. Building it naga-validates the
// merged material+shadow WGSL.
bool WbWgpuRenderTarget::ensureTexturedShadowPipeline() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (mTexShadowPipeline)
    return true;
  if (!ensureSceneTexPipeline() || !ensureSceneShadowPipeline())  // material sampler + shadow map/sampler
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());

  WGPUShaderSourceWGSL wgsl = {};
  wgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
  wgsl.code.data = WbWgpuShaders::kSolidLitTexturedShadow;
  wgsl.code.length = WGPU_STRLEN;
  WGPUShaderModuleDescriptor smDesc = {};
  smDesc.nextInChain = &wgsl.chain;
  WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &smDesc);
  if (!sm) {
    WbLog::info("[WbWgpuRenderTarget] tex-shadow CreateShaderModule failed");
    return false;
  }
  mTexShadowShaderModule = sm;

  // Non-filtering sampler for the R32Float shadow map.
  WGPUSamplerDescriptor ss = {};
  ss.addressModeU = WGPUAddressMode_ClampToEdge;
  ss.addressModeV = WGPUAddressMode_ClampToEdge;
  ss.addressModeW = WGPUAddressMode_ClampToEdge;
  ss.magFilter = WGPUFilterMode_Nearest;
  ss.minFilter = WGPUFilterMode_Nearest;
  ss.mipmapFilter = WGPUMipmapFilterMode_Nearest;
  ss.maxAnisotropy = 1;
  mTexShadowSampler = wgpuDeviceCreateSampler(device, &ss);
  if (!mTexShadowSampler)
    return false;

  WGPUBindGroupLayoutEntry e[9] = {};
  e[0].binding = 0;
  e[0].visibility = WGPUShaderStage_Vertex | WGPUShaderStage_Fragment;
  e[0].buffer.type = WGPUBufferBindingType_Uniform;
  e[0].buffer.hasDynamicOffset = 1;
  e[0].buffer.minBindingSize = 224;  // 192 + uvA/uvB (TextureTransform affine)
  for (int t = 1; t <= 4; ++t) {  // albedo, roughness, metalness, normal
    e[t].binding = static_cast<uint32_t>(t);
    e[t].visibility = WGPUShaderStage_Fragment;
    e[t].texture.sampleType = WGPUTextureSampleType_Float;
    e[t].texture.viewDimension = WGPUTextureViewDimension_2D;
  }
  e[5].binding = 5;
  e[5].visibility = WGPUShaderStage_Fragment;
  e[5].sampler.type = WGPUSamplerBindingType_Filtering;
  e[6].binding = 6;  // shadow map (R32Float → unfilterable)
  e[6].visibility = WGPUShaderStage_Fragment;
  e[6].texture.sampleType = WGPUTextureSampleType_UnfilterableFloat;
  e[6].texture.viewDimension = WGPUTextureViewDimension_2D;
  e[7].binding = 7;
  e[7].visibility = WGPUShaderStage_Fragment;
  e[7].sampler.type = WGPUSamplerBindingType_NonFiltering;
  e[8].binding = 8;  // shared LightU (lightViewProj + shadowParams + hemisphere sky/ground/up)
  e[8].visibility = WGPUShaderStage_Fragment;
  e[8].buffer.type = WGPUBufferBindingType_Uniform;
  e[8].buffer.minBindingSize = 144;
  WGPUBindGroupLayoutDescriptor bglDesc = {};
  bglDesc.entryCount = 9;
  bglDesc.entries = e;
  WGPUBindGroupLayout bgl = wgpuDeviceCreateBindGroupLayout(device, &bglDesc);
  if (!bgl) {
    WbLog::info("[WbWgpuRenderTarget] tex-shadow CreateBindGroupLayout failed");
    return false;
  }
  mTexShadowBindGroupLayout = bgl;

  WGPUPipelineLayoutDescriptor plDesc = {};
  plDesc.bindGroupLayoutCount = 1;
  plDesc.bindGroupLayouts = &bgl;
  WGPUPipelineLayout plLayout = wgpuDeviceCreatePipelineLayout(device, &plDesc);
  if (!plLayout)
    return false;

  WGPUVertexAttribute attrs[3] = {};
  attrs[0].format = WGPUVertexFormat_Float32x3; attrs[0].offset = 0;  attrs[0].shaderLocation = 0;
  attrs[1].format = WGPUVertexFormat_Float32x3; attrs[1].offset = 12; attrs[1].shaderLocation = 1;
  attrs[2].format = WGPUVertexFormat_Float32x2; attrs[2].offset = 24; attrs[2].shaderLocation = 2;
  WGPUVertexBufferLayout vbLayout = {};
  vbLayout.arrayStride = 32;
  vbLayout.stepMode = WGPUVertexStepMode_Vertex;
  vbLayout.attributeCount = 3;
  vbLayout.attributes = attrs;

  WGPUColorTargetState colorTarget = {};
  colorTarget.format = WGPUTextureFormat_RGBA8Unorm;
  colorTarget.writeMask = WGPUColorWriteMask_All;
  WGPUFragmentState fs = {};
  fs.module = sm;
  fs.entryPoint.data = "fs_main";
  fs.entryPoint.length = WGPU_STRLEN;
  fs.targetCount = 1;
  fs.targets = &colorTarget;

  WGPUDepthStencilState depthState = {};
  depthState.format = WGPUTextureFormat_Depth24Plus;
  depthState.depthWriteEnabled = WGPUOptionalBool_True;
  depthState.depthCompare = WGPUCompareFunction_Less;
  depthState.stencilFront.compare = WGPUCompareFunction_Always;
  depthState.stencilBack.compare = WGPUCompareFunction_Always;
  depthState.stencilReadMask = 0xFFFFFFFFu;
  depthState.stencilWriteMask = 0xFFFFFFFFu;

  WGPURenderPipelineDescriptor pipeDesc = {};
  pipeDesc.layout = plLayout;
  pipeDesc.vertex.module = sm;
  pipeDesc.vertex.entryPoint.data = "vs_main";
  pipeDesc.vertex.entryPoint.length = WGPU_STRLEN;
  pipeDesc.vertex.bufferCount = 1;
  pipeDesc.vertex.buffers = &vbLayout;
  pipeDesc.primitive.topology = WGPUPrimitiveTopology_TriangleList;
  pipeDesc.primitive.frontFace = WGPUFrontFace_CCW;
  pipeDesc.primitive.cullMode = WGPUCullMode_None;
  pipeDesc.depthStencil = &depthState;
  pipeDesc.multisample.count = 1;
  pipeDesc.multisample.mask = 0xFFFFFFFFu;
  pipeDesc.fragment = &fs;
  WGPURenderPipeline pipe = wgpuDeviceCreateRenderPipeline(device, &pipeDesc);
  if (!pipe) {
    wgpuPipelineLayoutRelease(plLayout);
    WbLog::info("[WbWgpuRenderTarget] tex-shadow CreateRenderPipeline failed");
    return false;
  }
  mTexShadowPipeline = pipe;
  // MSAA 4x variant of the same pipeline (identical state, multisample.count = 4) — pass 2 renders
  // into a 4x target + resolves, killing the jaggies/moiré of the 1x path. Optional: a null here
  // (unsupported count) just keeps the 1x path.
  pipeDesc.multisample.count = 4;
  mTexShadowPipelineMsaa = wgpuDeviceCreateRenderPipeline(device, &pipeDesc);
  // HDR variant: identical MSAA pipeline targeting RGBA16Float — the scene's >1 values (emissive,
  // sun) survive into the AgX tonemap pass instead of clamping at the RGBA8 write.
  colorTarget.format = WGPUTextureFormat_RGBA16Float;
  mTexShadowPipelineHdr = wgpuDeviceCreateRenderPipeline(device, &pipeDesc);
  // Reversed-Z variants: Greater compare + Depth32Float (the float exponent density lands at the
  // FAR plane after reversal — near-uniform precision, no far-field decal z-fighting).
  depthState.format = WGPUTextureFormat_Depth32Float;
  depthState.depthCompare = WGPUCompareFunction_Greater;
  mTexShadowPipelineHdrRev = wgpuDeviceCreateRenderPipeline(device, &pipeDesc);
  colorTarget.format = WGPUTextureFormat_RGBA8Unorm;
  mTexShadowPipelineMsaaRev = wgpuDeviceCreateRenderPipeline(device, &pipeDesc);
  wgpuPipelineLayoutRelease(plLayout);
  return true;
#  else
  return false;
#  endif
#else
  return false;
#endif
}

// Lazily create the 4x-multisampled color + depth attachments (sized to the target) that pass 2
// renders into before resolving to mTexture. False → caller uses the 1x path (no MSAA).
bool WbWgpuRenderTarget::ensureMsaaTargets() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (mScnMsaaColorView && mScnMsaaDepthView)
    return true;
  if (!mUsable)
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUTextureDescriptor td = {};
  td.usage = WGPUTextureUsage_RenderAttachment;
  td.dimension = WGPUTextureDimension_2D;
  td.size = {mWidth, mHeight, 1};
  td.format = WGPUTextureFormat_RGBA8Unorm;
  td.mipLevelCount = 1;
  td.sampleCount = 4;
  mScnMsaaColorTex = wgpuDeviceCreateTexture(device, &td);
  if (!mScnMsaaColorTex)
    return false;
  mScnMsaaColorView = wgpuTextureCreateView(static_cast<WGPUTexture>(mScnMsaaColorTex), nullptr);
  td.format = WGPUTextureFormat_Depth24Plus;
  mScnMsaaDepthTex = wgpuDeviceCreateTexture(device, &td);
  if (!mScnMsaaDepthTex)
    return false;
  mScnMsaaDepthView = wgpuTextureCreateView(static_cast<WGPUTexture>(mScnMsaaDepthTex), nullptr);
  return mScnMsaaColorView && mScnMsaaDepthView;
#  else
  return false;
#  endif
#else
  return false;
#endif
}

// Analytic atmospheric sky dome (kSkyAtmosphere): opaque fullscreen triangle drawn FIRST in the
// MSAA scene pass (depth write off, compare Always — geometry depth-tests over it). One 96 B SkyU
// uniform; bind group cached (the buffer never reallocates).
// SSAO resources: the full-res R32Float camera-depth prepass target, the estimate pipeline
// (kSsaoEstimate over the shared bloom {uniform,texture,sampler} layout), the multiply-apply
// pipeline (kBloomPost fs_composite with Dst×Zero blending), their uniforms and bind groups.
// Requires the bloom infrastructure (layout, blur pipelines, half-res ping-pong textures).
bool WbWgpuRenderTarget::ensureSsaoResources() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (mSsaoPipeline && mSsaoBgApply)
    return true;
  if (!mUsable || !ensureBloomPipelines(0.55f) || !ensureSceneClipDepthF32Pipeline() || !mScnDepthView ||
      !mTexShadowSampler)
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUQueue queue = static_cast<WGPUQueue>(mBackend->queue());

  WGPUTextureDescriptor td = {};
  td.usage = WGPUTextureUsage_RenderAttachment | WGPUTextureUsage_TextureBinding;
  td.dimension = WGPUTextureDimension_2D;
  td.size = {mWidth, mHeight, 1};
  td.format = WGPUTextureFormat_R32Float;
  td.mipLevelCount = 1;
  td.sampleCount = 1;
  mAoDepthTex = wgpuDeviceCreateTexture(device, &td);
  if (!mAoDepthTex)
    return false;
  mAoDepthView = wgpuTextureCreateView(static_cast<WGPUTexture>(mAoDepthTex), nullptr);
  if (!mAoDepthView)
    return false;

  WGPUShaderSourceWGSL wgsl = {};
  wgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
  wgsl.code.data = WbWgpuShaders::kSsaoEstimate;
  wgsl.code.length = WGPU_STRLEN;
  WGPUShaderModuleDescriptor smDesc = {};
  smDesc.nextInChain = &wgsl.chain;
  WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &smDesc);
  if (!sm) {
    WbLog::info("[WbWgpuRenderTarget] ssao CreateShaderModule failed");
    return false;
  }
  mSsaoShaderModule = sm;

  // The estimate samples the R32Float depth — NOT filterable: it needs its own layout with an
  // UnfilterableFloat texture + a NonFiltering sampler (the shadow-map sampler). The apply pass
  // samples RGBA8 and keeps the shared bloom layout.
  WGPUBindGroupLayoutEntry de[3] = {};
  de[0].binding = 0;
  de[0].visibility = WGPUShaderStage_Fragment;
  de[0].buffer.type = WGPUBufferBindingType_Uniform;
  de[0].buffer.minBindingSize = 16;
  de[1].binding = 1;
  de[1].visibility = WGPUShaderStage_Fragment;
  de[1].texture.sampleType = WGPUTextureSampleType_UnfilterableFloat;
  de[1].texture.viewDimension = WGPUTextureViewDimension_2D;
  de[2].binding = 2;
  de[2].visibility = WGPUShaderStage_Fragment;
  de[2].sampler.type = WGPUSamplerBindingType_NonFiltering;
  WGPUBindGroupLayoutDescriptor dbglDesc = {};
  dbglDesc.entryCount = 3;
  dbglDesc.entries = de;
  WGPUBindGroupLayout estBgl = wgpuDeviceCreateBindGroupLayout(device, &dbglDesc);
  if (!estBgl)
    return false;
  mSsaoEstimateBgl = estBgl;

  WGPUBindGroupLayout bgl = static_cast<WGPUBindGroupLayout>(mBloomBgl);
  WGPUPipelineLayoutDescriptor plDesc = {};
  plDesc.bindGroupLayoutCount = 1;
  plDesc.bindGroupLayouts = &bgl;
  WGPUPipelineLayout plLayout = wgpuDeviceCreatePipelineLayout(device, &plDesc);
  if (!plLayout)
    return false;
  WGPUPipelineLayoutDescriptor eplDesc = {};
  eplDesc.bindGroupLayoutCount = 1;
  eplDesc.bindGroupLayouts = &estBgl;
  WGPUPipelineLayout eplLayout = wgpuDeviceCreatePipelineLayout(device, &eplDesc);
  if (!eplLayout) {
    wgpuPipelineLayoutRelease(plLayout);
    return false;
  }

  // Estimate pipeline (no blend, RGBA8 half-res target).
  {
    WGPUColorTargetState colorTarget = {};
    colorTarget.format = WGPUTextureFormat_RGBA8Unorm;
    colorTarget.writeMask = WGPUColorWriteMask_All;
    WGPUFragmentState fs = {};
    fs.module = sm;
    fs.entryPoint.data = "fs_main";
    fs.entryPoint.length = WGPU_STRLEN;
    fs.targetCount = 1;
    fs.targets = &colorTarget;
    WGPURenderPipelineDescriptor pd = {};
    pd.layout = eplLayout;
    pd.vertex.module = sm;
    pd.vertex.entryPoint.data = "vs_main";
    pd.vertex.entryPoint.length = WGPU_STRLEN;
    pd.primitive.topology = WGPUPrimitiveTopology_TriangleList;
    pd.primitive.frontFace = WGPUFrontFace_CCW;
    pd.primitive.cullMode = WGPUCullMode_None;
    pd.multisample.count = 1;
    pd.multisample.mask = 0xFFFFFFFFu;
    pd.fragment = &fs;
    mSsaoPipeline = wgpuDeviceCreateRenderPipeline(device, &pd);
  }
  // Apply pipeline: fs_composite from the bloom shader + MULTIPLY blending (out = src × dst).
  {
    WGPUBlendState mulBlend = {};
    mulBlend.color.operation = WGPUBlendOperation_Add;
    mulBlend.color.srcFactor = WGPUBlendFactor_Dst;
    mulBlend.color.dstFactor = WGPUBlendFactor_Zero;
    mulBlend.alpha.operation = WGPUBlendOperation_Add;
    mulBlend.alpha.srcFactor = WGPUBlendFactor_Zero;
    mulBlend.alpha.dstFactor = WGPUBlendFactor_One;
    WGPUColorTargetState colorTarget = {};
    colorTarget.format = WGPUTextureFormat_RGBA8Unorm;
    colorTarget.blend = &mulBlend;
    colorTarget.writeMask = WGPUColorWriteMask_All;
    WGPUFragmentState fs = {};
    fs.module = static_cast<WGPUShaderModule>(mBloomShaderModule);
    fs.entryPoint.data = "fs_composite";
    fs.entryPoint.length = WGPU_STRLEN;
    fs.targetCount = 1;
    fs.targets = &colorTarget;
    WGPURenderPipelineDescriptor pd = {};
    pd.layout = plLayout;
    pd.vertex.module = static_cast<WGPUShaderModule>(mBloomShaderModule);
    pd.vertex.entryPoint.data = "vs_main";
    pd.vertex.entryPoint.length = WGPU_STRLEN;
    pd.primitive.topology = WGPUPrimitiveTopology_TriangleList;
    pd.primitive.frontFace = WGPUFrontFace_CCW;
    pd.primitive.cullMode = WGPUCullMode_None;
    pd.multisample.count = 1;
    pd.multisample.mask = 0xFFFFFFFFu;
    pd.fragment = &fs;
    mSsaoApplyPipeline = wgpuDeviceCreateRenderPipeline(device, &pd);
  }
  wgpuPipelineLayoutRelease(plLayout);
  wgpuPipelineLayoutRelease(eplLayout);
  if (!mSsaoPipeline || !mSsaoApplyPipeline) {
    WbLog::info("[WbWgpuRenderTarget] ssao CreateRenderPipeline failed");
    return false;
  }

  // Uniforms: estimate params written per exec; apply strength fixed at 1.0 (pure multiply).
  for (void **ub : {&mSsaoUb, &mSsaoApplyUb}) {
    WGPUBufferDescriptor bd = {};
    bd.size = 16;
    bd.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
    *ub = wgpuDeviceCreateBuffer(device, &bd);
    if (!*ub)
      return false;
  }
  const float one4[4] = {1.0f, 0, 0, 0};
  wgpuQueueWriteBuffer(queue, static_cast<WGPUBuffer>(mSsaoApplyUb), 0, one4, 16);

  struct {
    void **bg;
    void *ub;
    void *texView;
    WGPUBindGroupLayout layout;
    void *sampler;
  } bgs[2] = {
    {&mSsaoBgEstimate, mSsaoUb, mAoDepthView, estBgl, mTexShadowSampler},  // non-filtering (R32F)
    {&mSsaoBgApply, mSsaoApplyUb, mBloomViewA, bgl, mScnTexSampler},
  };
  for (auto &b : bgs) {
    WGPUBindGroupEntry be[3] = {};
    be[0].binding = 0;
    be[0].buffer = static_cast<WGPUBuffer>(b.ub);
    be[0].size = 16;
    be[1].binding = 1;
    be[1].textureView = static_cast<WGPUTextureView>(b.texView);
    be[2].binding = 2;
    be[2].sampler = static_cast<WGPUSampler>(b.sampler);
    WGPUBindGroupDescriptor bgd = {};
    bgd.layout = b.layout;
    bgd.entryCount = 3;
    bgd.entries = be;
    *b.bg = wgpuDeviceCreateBindGroup(device, &bgd);
    if (!*b.bg)
      return false;
  }
  return true;
#  else
  return false;
#  endif
#else
  return false;
#endif
}

// HDR + AgX: the RGBA16F scene targets (4x MSAA attachment + 1x resolve) and the tonemap pass that
// maps them to display LDR (mView). The HDR pipeline VARIANTS (textured-shadow + sky) are built by
// their own ensure functions; this requires them and adds the textures + AgX resources.
bool WbWgpuRenderTarget::ensureHdrPipelines() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (mAgxPipeline && mAgxBg && mHdrMsaaView)
    return true;
  if (!mUsable || !mTexShadowPipelineHdr || !mScnTexSampler)
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUQueue queue = static_cast<WGPUQueue>(mBackend->queue());

  // RGBA16F targets: 4x MSAA attachment + 1x resolve (sampled by the tonemap).
  WGPUTextureDescriptor td = {};
  td.usage = WGPUTextureUsage_RenderAttachment;
  td.dimension = WGPUTextureDimension_2D;
  td.size = {mWidth, mHeight, 1};
  td.format = WGPUTextureFormat_RGBA16Float;
  td.mipLevelCount = 1;
  td.sampleCount = 4;
  mHdrMsaaTex = wgpuDeviceCreateTexture(device, &td);
  td.sampleCount = 1;
  td.usage = WGPUTextureUsage_RenderAttachment | WGPUTextureUsage_TextureBinding;
  mHdrTex = wgpuDeviceCreateTexture(device, &td);
  if (!mHdrMsaaTex || !mHdrTex)
    return false;
  mHdrMsaaView = wgpuTextureCreateView(static_cast<WGPUTexture>(mHdrMsaaTex), nullptr);
  mHdrView = wgpuTextureCreateView(static_cast<WGPUTexture>(mHdrTex), nullptr);
  if (!mHdrMsaaView || !mHdrView)
    return false;

  // AgX tonemap pass: {uniform, texture, sampler} → fullscreen → RGBA8 (mView).
  WGPUShaderSourceWGSL wgsl = {};
  wgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
  wgsl.code.data = WbWgpuShaders::kAgxTonemapPost;
  wgsl.code.length = WGPU_STRLEN;
  WGPUShaderModuleDescriptor smDesc = {};
  smDesc.nextInChain = &wgsl.chain;
  WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &smDesc);
  if (!sm) {
    WbLog::info("[WbWgpuRenderTarget] agx CreateShaderModule failed");
    return false;
  }
  mAgxShaderModule = sm;
  WGPUBindGroupLayoutEntry e[3] = {};
  e[0].binding = 0;
  e[0].visibility = WGPUShaderStage_Fragment;
  e[0].buffer.type = WGPUBufferBindingType_Uniform;
  e[0].buffer.minBindingSize = 16;
  e[1].binding = 1;
  e[1].visibility = WGPUShaderStage_Fragment;
  e[1].texture.sampleType = WGPUTextureSampleType_Float;
  e[1].texture.viewDimension = WGPUTextureViewDimension_2D;
  e[2].binding = 2;
  e[2].visibility = WGPUShaderStage_Fragment;
  e[2].sampler.type = WGPUSamplerBindingType_Filtering;
  WGPUBindGroupLayoutDescriptor bglDesc = {};
  bglDesc.entryCount = 3;
  bglDesc.entries = e;
  WGPUBindGroupLayout bgl = wgpuDeviceCreateBindGroupLayout(device, &bglDesc);
  if (!bgl)
    return false;
  mAgxBgl = bgl;
  WGPUPipelineLayoutDescriptor plDesc = {};
  plDesc.bindGroupLayoutCount = 1;
  plDesc.bindGroupLayouts = &bgl;
  WGPUPipelineLayout plLayout = wgpuDeviceCreatePipelineLayout(device, &plDesc);
  if (!plLayout)
    return false;
  WGPUColorTargetState colorTarget = {};
  colorTarget.format = WGPUTextureFormat_RGBA8Unorm;
  colorTarget.writeMask = WGPUColorWriteMask_All;
  WGPUFragmentState fs = {};
  fs.module = sm;
  fs.entryPoint.data = "fs_main";
  fs.entryPoint.length = WGPU_STRLEN;
  fs.targetCount = 1;
  fs.targets = &colorTarget;
  WGPURenderPipelineDescriptor pd = {};
  pd.layout = plLayout;
  pd.vertex.module = sm;
  pd.vertex.entryPoint.data = "vs_main";
  pd.vertex.entryPoint.length = WGPU_STRLEN;
  pd.primitive.topology = WGPUPrimitiveTopology_TriangleList;
  pd.primitive.frontFace = WGPUFrontFace_CCW;
  pd.primitive.cullMode = WGPUCullMode_None;
  pd.multisample.count = 1;
  pd.multisample.mask = 0xFFFFFFFFu;
  pd.fragment = &fs;
  mAgxPipeline = wgpuDeviceCreateRenderPipeline(device, &pd);
  wgpuPipelineLayoutRelease(plLayout);
  if (!mAgxPipeline) {
    WbLog::info("[WbWgpuRenderTarget] agx CreateRenderPipeline failed");
    return false;
  }
  WGPUBufferDescriptor ubd = {};
  ubd.size = 16;
  ubd.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
  mAgxUb = wgpuDeviceCreateBuffer(device, &ubd);
  if (!mAgxUb)
    return false;
  const float one[4] = {1.0f, 0, 0, 0};
  wgpuQueueWriteBuffer(queue, static_cast<WGPUBuffer>(mAgxUb), 0, one, 16);
  WGPUBindGroupEntry be[3] = {};
  be[0].binding = 0;
  be[0].buffer = static_cast<WGPUBuffer>(mAgxUb);
  be[0].size = 16;
  be[1].binding = 1;
  be[1].textureView = static_cast<WGPUTextureView>(mHdrView);
  be[2].binding = 2;
  be[2].sampler = static_cast<WGPUSampler>(mScnTexSampler);
  WGPUBindGroupDescriptor bgd = {};
  bgd.layout = bgl;
  bgd.entryCount = 3;
  bgd.entries = be;
  mAgxBg = wgpuDeviceCreateBindGroup(device, &bgd);
  return mAgxBg != nullptr;
#  else
  return false;
#  endif
#else
  return false;
#endif
}

// Bloom chain: shader module + shared 3-entry layout + 3 pipelines (extract/blur/composite) +
// half-res ping-pong textures + per-stage 16 B uniforms (written once) + cached bind groups.
bool WbWgpuRenderTarget::ensureBloomPipelines(float strength) {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (mBloomCompositePipeline && mBloomBgComposite)
    return true;
  if (!mUsable || !mView || !mScnTexSampler)
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUQueue queue = static_cast<WGPUQueue>(mBackend->queue());

  WGPUShaderSourceWGSL wgsl = {};
  wgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
  wgsl.code.data = WbWgpuShaders::kBloomPost;
  wgsl.code.length = WGPU_STRLEN;
  WGPUShaderModuleDescriptor smDesc = {};
  smDesc.nextInChain = &wgsl.chain;
  WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &smDesc);
  if (!sm) {
    WbLog::info("[WbWgpuRenderTarget] bloom CreateShaderModule failed");
    return false;
  }
  mBloomShaderModule = sm;

  WGPUBindGroupLayoutEntry e[3] = {};
  e[0].binding = 0;
  e[0].visibility = WGPUShaderStage_Fragment;
  e[0].buffer.type = WGPUBufferBindingType_Uniform;
  e[0].buffer.minBindingSize = 16;
  e[1].binding = 1;
  e[1].visibility = WGPUShaderStage_Fragment;
  e[1].texture.sampleType = WGPUTextureSampleType_Float;
  e[1].texture.viewDimension = WGPUTextureViewDimension_2D;
  e[2].binding = 2;
  e[2].visibility = WGPUShaderStage_Fragment;
  e[2].sampler.type = WGPUSamplerBindingType_Filtering;
  WGPUBindGroupLayoutDescriptor bglDesc = {};
  bglDesc.entryCount = 3;
  bglDesc.entries = e;
  WGPUBindGroupLayout bgl = wgpuDeviceCreateBindGroupLayout(device, &bglDesc);
  if (!bgl)
    return false;
  mBloomBgl = bgl;
  WGPUPipelineLayoutDescriptor plDesc = {};
  plDesc.bindGroupLayoutCount = 1;
  plDesc.bindGroupLayouts = &bgl;
  WGPUPipelineLayout plLayout = wgpuDeviceCreatePipelineLayout(device, &plDesc);
  if (!plLayout)
    return false;

  // One pipeline per entry point; composite gets additive blending.
  WGPUBlendState addBlend = {};
  addBlend.color.operation = WGPUBlendOperation_Add;
  addBlend.color.srcFactor = WGPUBlendFactor_One;
  addBlend.color.dstFactor = WGPUBlendFactor_One;
  addBlend.alpha.operation = WGPUBlendOperation_Add;
  addBlend.alpha.srcFactor = WGPUBlendFactor_One;
  addBlend.alpha.dstFactor = WGPUBlendFactor_One;
  const char *entries[3] = {"fs_extract", "fs_blur", "fs_composite"};
  void **pipes[3] = {&mBloomExtractPipeline, &mBloomBlurPipeline, &mBloomCompositePipeline};
  for (int i = 0; i < 3; ++i) {
    WGPUColorTargetState colorTarget = {};
    colorTarget.format = WGPUTextureFormat_RGBA8Unorm;
    colorTarget.writeMask = WGPUColorWriteMask_All;
    if (i == 2)
      colorTarget.blend = &addBlend;
    WGPUFragmentState fs = {};
    fs.module = sm;
    fs.entryPoint.data = entries[i];
    fs.entryPoint.length = WGPU_STRLEN;
    fs.targetCount = 1;
    fs.targets = &colorTarget;
    WGPURenderPipelineDescriptor pd = {};
    pd.layout = plLayout;
    pd.vertex.module = sm;
    pd.vertex.entryPoint.data = "vs_main";
    pd.vertex.entryPoint.length = WGPU_STRLEN;
    pd.primitive.topology = WGPUPrimitiveTopology_TriangleList;
    pd.primitive.frontFace = WGPUFrontFace_CCW;
    pd.primitive.cullMode = WGPUCullMode_None;
    pd.multisample.count = 1;
    pd.multisample.mask = 0xFFFFFFFFu;
    pd.fragment = &fs;
    *pipes[i] = wgpuDeviceCreateRenderPipeline(device, &pd);
    if (!*pipes[i]) {
      wgpuPipelineLayoutRelease(plLayout);
      WbLog::info("[WbWgpuRenderTarget] bloom CreateRenderPipeline failed");
      return false;
    }
  }
  wgpuPipelineLayoutRelease(plLayout);

  // Half-res ping-pong textures.
  const uint32_t bw = mWidth / 2 > 0 ? mWidth / 2 : 1, bh = mHeight / 2 > 0 ? mHeight / 2 : 1;
  WGPUTextureDescriptor td = {};
  td.usage = WGPUTextureUsage_RenderAttachment | WGPUTextureUsage_TextureBinding;
  td.dimension = WGPUTextureDimension_2D;
  td.size = {bw, bh, 1};
  td.format = WGPUTextureFormat_RGBA8Unorm;
  td.mipLevelCount = 1;
  td.sampleCount = 1;
  mBloomTexA = wgpuDeviceCreateTexture(device, &td);
  mBloomTexB = wgpuDeviceCreateTexture(device, &td);
  if (!mBloomTexA || !mBloomTexB)
    return false;
  mBloomViewA = wgpuTextureCreateView(static_cast<WGPUTexture>(mBloomTexA), nullptr);
  mBloomViewB = wgpuTextureCreateView(static_cast<WGPUTexture>(mBloomTexB), nullptr);
  if (!mBloomViewA || !mBloomViewB)
    return false;

  // Per-stage uniforms, written once: LDR bright-pass threshold; blur steps at half-res texel
  // scale; composite strength.
  struct {
    void **ub;
    float v[4];
  } ubs[4] = {
    {&mBloomUbExtract, {0.82f, 0, 0, 0}},
    {&mBloomUbBlurH, {2.0f / bw, 0, 0, 0}},
    {&mBloomUbBlurV, {0, 2.0f / bh, 0, 0}},
    {&mBloomUbComposite, {strength, 0, 0, 0}},
  };
  for (auto &u : ubs) {
    WGPUBufferDescriptor bd = {};
    bd.size = 16;
    bd.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
    *u.ub = wgpuDeviceCreateBuffer(device, &bd);
    if (!*u.ub)
      return false;
    wgpuQueueWriteBuffer(queue, static_cast<WGPUBuffer>(*u.ub), 0, u.v, 16);
  }

  // Cached bind groups: extract(scene→A), blurH(A→B), blurV(B→A), composite(A→scene).
  struct {
    void **bg;
    void *ub;
    void *texView;
  } bgs[4] = {
    {&mBloomBgExtract, mBloomUbExtract, mView},
    {&mBloomBgBlurH, mBloomUbBlurH, mBloomViewA},
    {&mBloomBgBlurV, mBloomUbBlurV, mBloomViewB},
    {&mBloomBgComposite, mBloomUbComposite, mBloomViewA},
  };
  for (auto &b : bgs) {
    WGPUBindGroupEntry be[3] = {};
    be[0].binding = 0;
    be[0].buffer = static_cast<WGPUBuffer>(b.ub);
    be[0].size = 16;
    be[1].binding = 1;
    be[1].textureView = static_cast<WGPUTextureView>(b.texView);
    be[2].binding = 2;
    be[2].sampler = static_cast<WGPUSampler>(mScnTexSampler);
    WGPUBindGroupDescriptor bgd = {};
    bgd.layout = bgl;
    bgd.entryCount = 3;
    bgd.entries = be;
    *b.bg = wgpuDeviceCreateBindGroup(device, &bgd);
    if (!*b.bg)
      return false;
  }
  return true;
#  else
  (void)strength;
  return false;
#  endif
#else
  (void)strength;
  return false;
#endif
}

// Reversed-Z float depth attachments (4x for pass 2, 1x for the SSAO prepass), lazily created.
bool WbWgpuRenderTarget::ensureReversedDepthTargets() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (mScnMsaaDepthViewF32 && mScnDepthViewF32)
    return true;
  if (!mUsable)
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUTextureDescriptor td = {};
  td.usage = WGPUTextureUsage_RenderAttachment;
  td.dimension = WGPUTextureDimension_2D;
  td.size = {mWidth, mHeight, 1};
  td.format = WGPUTextureFormat_Depth32Float;
  td.mipLevelCount = 1;
  td.sampleCount = 4;
  mScnMsaaDepthTexF32 = wgpuDeviceCreateTexture(device, &td);
  td.sampleCount = 1;
  mScnDepthTexF32 = wgpuDeviceCreateTexture(device, &td);
  if (!mScnMsaaDepthTexF32 || !mScnDepthTexF32)
    return false;
  mScnMsaaDepthViewF32 = wgpuTextureCreateView(static_cast<WGPUTexture>(mScnMsaaDepthTexF32), nullptr);
  mScnDepthViewF32 = wgpuTextureCreateView(static_cast<WGPUTexture>(mScnDepthTexF32), nullptr);
  return mScnMsaaDepthViewF32 && mScnDepthViewF32;
#  else
  return false;
#  endif
#else
  return false;
#endif
}

bool WbWgpuRenderTarget::ensureSkyPipeline() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (mSkyPipeline && mSkyBindGroup)
    return true;
  if (!mUsable)
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());

  WGPUShaderSourceWGSL wgsl = {};
  wgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
  wgsl.code.data = WbWgpuShaders::kSkyAtmosphere;
  wgsl.code.length = WGPU_STRLEN;
  WGPUShaderModuleDescriptor smDesc = {};
  smDesc.nextInChain = &wgsl.chain;
  WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &smDesc);
  if (!sm) {
    WbLog::info("[WbWgpuRenderTarget] sky CreateShaderModule failed");
    return false;
  }
  mSkyShaderModule = sm;

  WGPUBindGroupLayoutEntry bglEntry = {};
  bglEntry.binding = 0;
  bglEntry.visibility = WGPUShaderStage_Fragment;
  bglEntry.buffer.type = WGPUBufferBindingType_Uniform;
  bglEntry.buffer.minBindingSize = 96;
  WGPUBindGroupLayoutDescriptor bglDesc = {};
  bglDesc.entryCount = 1;
  bglDesc.entries = &bglEntry;
  WGPUBindGroupLayout bgl = wgpuDeviceCreateBindGroupLayout(device, &bglDesc);
  if (!bgl)
    return false;
  mSkyBindGroupLayout = bgl;

  WGPUPipelineLayoutDescriptor plDesc = {};
  plDesc.bindGroupLayoutCount = 1;
  plDesc.bindGroupLayouts = &bgl;
  WGPUPipelineLayout plLayout = wgpuDeviceCreatePipelineLayout(device, &plDesc);
  if (!plLayout)
    return false;

  WGPUColorTargetState colorTarget = {};
  colorTarget.format = WGPUTextureFormat_RGBA8Unorm;
  colorTarget.writeMask = WGPUColorWriteMask_All;
  WGPUFragmentState fs = {};
  fs.module = sm;
  fs.entryPoint.data = "fs_main";
  fs.entryPoint.length = WGPU_STRLEN;
  fs.targetCount = 1;
  fs.targets = &colorTarget;

  // The MSAA scene pass has a depth attachment → declare a matching depth state that neither
  // tests nor writes (the dome sits behind everything).
  WGPUDepthStencilState depthState = {};
  depthState.format = WGPUTextureFormat_Depth24Plus;
  depthState.depthWriteEnabled = WGPUOptionalBool_False;
  depthState.depthCompare = WGPUCompareFunction_Always;
  depthState.stencilFront.compare = WGPUCompareFunction_Always;
  depthState.stencilBack.compare = WGPUCompareFunction_Always;
  depthState.stencilReadMask = 0xFFFFFFFFu;
  depthState.stencilWriteMask = 0xFFFFFFFFu;

  WGPURenderPipelineDescriptor pipeDesc = {};
  pipeDesc.layout = plLayout;
  pipeDesc.vertex.module = sm;
  pipeDesc.vertex.entryPoint.data = "vs_main";
  pipeDesc.vertex.entryPoint.length = WGPU_STRLEN;
  pipeDesc.vertex.bufferCount = 0;  // full-screen triangle from vertex_index
  pipeDesc.primitive.topology = WGPUPrimitiveTopology_TriangleList;
  pipeDesc.primitive.frontFace = WGPUFrontFace_CCW;
  pipeDesc.primitive.cullMode = WGPUCullMode_None;
  pipeDesc.depthStencil = &depthState;
  pipeDesc.multisample.count = 4;  // drawn only inside the MSAA pass
  pipeDesc.multisample.mask = 0xFFFFFFFFu;
  pipeDesc.fragment = &fs;
  WGPURenderPipeline pipe = wgpuDeviceCreateRenderPipeline(device, &pipeDesc);
  if (!pipe) {
    wgpuPipelineLayoutRelease(plLayout);
    WbLog::info("[WbWgpuRenderTarget] sky CreateRenderPipeline failed");
    return false;
  }
  mSkyPipeline = pipe;
  // HDR variant (RGBA16Float target) for the AgX path — the dome draws inside the HDR scene pass.
  colorTarget.format = WGPUTextureFormat_RGBA16Float;
  mSkyPipelineHdr = wgpuDeviceCreateRenderPipeline(device, &pipeDesc);
  // Reversed-Z variants: the pipeline's depth FORMAT must match the F32 attachment (compare is
  // Always + write off either way).
  depthState.format = WGPUTextureFormat_Depth32Float;
  mSkyPipelineHdrRev = wgpuDeviceCreateRenderPipeline(device, &pipeDesc);
  colorTarget.format = WGPUTextureFormat_RGBA8Unorm;
  mSkyPipelineRev = wgpuDeviceCreateRenderPipeline(device, &pipeDesc);
  wgpuPipelineLayoutRelease(plLayout);

  if (!mSkyUniformBuffer) {
    WGPUBufferDescriptor ud = {};
    ud.size = 96;
    ud.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
    mSkyUniformBuffer = wgpuDeviceCreateBuffer(device, &ud);
    if (!mSkyUniformBuffer)
      return false;
  }
  WGPUBindGroupEntry be = {};
  be.binding = 0;
  be.buffer = static_cast<WGPUBuffer>(mSkyUniformBuffer);
  be.size = 96;
  WGPUBindGroupDescriptor bd = {};
  bd.layout = bgl;
  bd.entryCount = 1;
  bd.entries = &be;
  mSkyBindGroup = wgpuDeviceCreateBindGroup(device, &bd);
  return mSkyBindGroup != nullptr;
#  else
  return false;
#  endif
#else
  return false;
#endif
}

// R4 lighting convergence: the full-material × multi-cascade pipeline (kSolidLitTexturedCsm). Identical
// to ensureTexturedShadowPipeline EXCEPT @6 is a texture_2d_array (the cascade shadow map) and @8 (LightU)
// is 336 B (4 light VPs + params/splits + hemisphere). Building it naga-validates the merged shader.
bool WbWgpuRenderTarget::ensureTexturedCsmPipeline() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (mTexCsmPipeline)
    return true;
  if (!ensureSceneTexPipeline())  // material sampler @5 + default 1×1 maps + scene layout/depth
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());

  WGPUShaderSourceWGSL wgsl = {};
  wgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
  wgsl.code.data = WbWgpuShaders::kSolidLitTexturedCsm;
  wgsl.code.length = WGPU_STRLEN;
  WGPUShaderModuleDescriptor smDesc = {};
  smDesc.nextInChain = &wgsl.chain;
  WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &smDesc);
  if (!sm) {
    WbLog::info("[WbWgpuRenderTarget] tex-csm CreateShaderModule failed");
    return false;
  }
  mTexCsmShaderModule = sm;

  WGPUSamplerDescriptor ss = {};  // non-filtering (R32Float cascade array)
  ss.addressModeU = WGPUAddressMode_ClampToEdge;
  ss.addressModeV = WGPUAddressMode_ClampToEdge;
  ss.addressModeW = WGPUAddressMode_ClampToEdge;
  ss.magFilter = WGPUFilterMode_Nearest;
  ss.minFilter = WGPUFilterMode_Nearest;
  ss.mipmapFilter = WGPUMipmapFilterMode_Nearest;
  ss.maxAnisotropy = 1;
  mTexCsmSampler = wgpuDeviceCreateSampler(device, &ss);
  if (!mTexCsmSampler)
    return false;

  WGPUBindGroupLayoutEntry e[9] = {};
  e[0].binding = 0;
  e[0].visibility = WGPUShaderStage_Vertex | WGPUShaderStage_Fragment;
  e[0].buffer.type = WGPUBufferBindingType_Uniform;
  e[0].buffer.hasDynamicOffset = 1;
  e[0].buffer.minBindingSize = 192;
  for (int t = 1; t <= 4; ++t) {  // albedo, roughness, metalness, normal
    e[t].binding = static_cast<uint32_t>(t);
    e[t].visibility = WGPUShaderStage_Fragment;
    e[t].texture.sampleType = WGPUTextureSampleType_Float;
    e[t].texture.viewDimension = WGPUTextureViewDimension_2D;
  }
  e[5].binding = 5;
  e[5].visibility = WGPUShaderStage_Fragment;
  e[5].sampler.type = WGPUSamplerBindingType_Filtering;
  e[6].binding = 6;  // cascade shadow ARRAY (R32Float → unfilterable)
  e[6].visibility = WGPUShaderStage_Fragment;
  e[6].texture.sampleType = WGPUTextureSampleType_UnfilterableFloat;
  e[6].texture.viewDimension = WGPUTextureViewDimension_2DArray;
  e[7].binding = 7;
  e[7].visibility = WGPUShaderStage_Fragment;
  e[7].sampler.type = WGPUSamplerBindingType_NonFiltering;
  e[8].binding = 8;  // CsmLightU (4 light VPs + params/splits + hemisphere)
  e[8].visibility = WGPUShaderStage_Fragment;
  e[8].buffer.type = WGPUBufferBindingType_Uniform;
  e[8].buffer.minBindingSize = 336;
  WGPUBindGroupLayoutDescriptor bglDesc = {};
  bglDesc.entryCount = 9;
  bglDesc.entries = e;
  WGPUBindGroupLayout bgl = wgpuDeviceCreateBindGroupLayout(device, &bglDesc);
  if (!bgl) {
    WbLog::info("[WbWgpuRenderTarget] tex-csm CreateBindGroupLayout failed");
    return false;
  }
  mTexCsmBindGroupLayout = bgl;

  WGPUPipelineLayoutDescriptor plDesc = {};
  plDesc.bindGroupLayoutCount = 1;
  plDesc.bindGroupLayouts = &bgl;
  WGPUPipelineLayout plLayout = wgpuDeviceCreatePipelineLayout(device, &plDesc);
  if (!plLayout)
    return false;

  WGPUVertexAttribute attrs[3] = {};
  attrs[0].format = WGPUVertexFormat_Float32x3; attrs[0].offset = 0;  attrs[0].shaderLocation = 0;
  attrs[1].format = WGPUVertexFormat_Float32x3; attrs[1].offset = 12; attrs[1].shaderLocation = 1;
  attrs[2].format = WGPUVertexFormat_Float32x2; attrs[2].offset = 24; attrs[2].shaderLocation = 2;
  WGPUVertexBufferLayout vbLayout = {};
  vbLayout.arrayStride = 32;
  vbLayout.stepMode = WGPUVertexStepMode_Vertex;
  vbLayout.attributeCount = 3;
  vbLayout.attributes = attrs;

  WGPUColorTargetState colorTarget = {};
  colorTarget.format = WGPUTextureFormat_RGBA8Unorm;
  colorTarget.writeMask = WGPUColorWriteMask_All;
  WGPUFragmentState fs = {};
  fs.module = sm;
  fs.entryPoint.data = "fs_main";
  fs.entryPoint.length = WGPU_STRLEN;
  fs.targetCount = 1;
  fs.targets = &colorTarget;

  WGPUDepthStencilState depthState = {};
  depthState.format = WGPUTextureFormat_Depth24Plus;
  depthState.depthWriteEnabled = WGPUOptionalBool_True;
  depthState.depthCompare = WGPUCompareFunction_Less;
  depthState.stencilFront.compare = WGPUCompareFunction_Always;
  depthState.stencilBack.compare = WGPUCompareFunction_Always;
  depthState.stencilReadMask = 0xFFFFFFFFu;
  depthState.stencilWriteMask = 0xFFFFFFFFu;

  WGPURenderPipelineDescriptor pipeDesc = {};
  pipeDesc.layout = plLayout;
  pipeDesc.vertex.module = sm;
  pipeDesc.vertex.entryPoint.data = "vs_main";
  pipeDesc.vertex.entryPoint.length = WGPU_STRLEN;
  pipeDesc.vertex.bufferCount = 1;
  pipeDesc.vertex.buffers = &vbLayout;
  pipeDesc.primitive.topology = WGPUPrimitiveTopology_TriangleList;
  pipeDesc.primitive.frontFace = WGPUFrontFace_CCW;
  pipeDesc.primitive.cullMode = WGPUCullMode_None;
  pipeDesc.depthStencil = &depthState;
  pipeDesc.multisample.count = 1;
  pipeDesc.multisample.mask = 0xFFFFFFFFu;
  pipeDesc.fragment = &fs;
  WGPURenderPipeline pipe = wgpuDeviceCreateRenderPipeline(device, &pipeDesc);
  wgpuPipelineLayoutRelease(plLayout);
  if (!pipe) {
    WbLog::info("[WbWgpuRenderTarget] tex-csm CreateRenderPipeline failed");
    return false;
  }
  mTexCsmPipeline = pipe;
  return true;
#  else
  return false;
#  endif
#else
  return false;
#endif
}

// R4 lighting convergence: full-material × multi-cascade render. N light clip-depth passes into the
// cascade array, then one kSolidLitTexturedCsm lit pass (per-draw material bind groups + the array @6 +
// the CsmLightU @8). The material path of clearAndDrawSceneTexturedShadowed × the cascades of
// clearAndDrawSceneCsm. Reads back the lit RGBA8 into `rgba8`.
bool WbWgpuRenderTarget::clearAndDrawSceneTexturedCsm(
  const WbWgpuClearColor &color, const float *viewProj16, const float *cascadeLightViewProjs,
  const float *cascadeSplitsFar4, uint32_t cascadeCount, const float *lightDirAmbient4,
  const WbWgpuSolidDraw *draws, uint32_t numDraws, float shadowStrength, float depthBias,
  const float *cameraWorldPos3, const float *hemiSky4, const float *hemiGround4, const float *worldUp3,
  void *rgba8) {
  if (!mUsable || !rgba8 || !viewProj16 || !cascadeLightViewProjs || !cascadeSplitsFar4 ||
      !lightDirAmbient4)
    return false;
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (cascadeCount < 1)
    cascadeCount = 1;
  if (cascadeCount > kCsmMaxCascades)
    cascadeCount = kCsmMaxCascades;
  if (!ensureSceneClipDepthF32Pipeline() || !ensureTexturedCsmPipeline())
    return false;
  const uint32_t kShadowRes = 1024;
  if (!ensureShadowMapArray(kShadowRes, cascadeCount))
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUQueue queue = static_cast<WGPUQueue>(mBackend->queue());

  const uint32_t slotCount = numDraws == 0 ? 1u : numDraws;

  // --- Scene uniform (material path; mirrors clearAndDrawSceneTexturedShadowed). ---
  const size_t needed = static_cast<size_t>(slotCount) * kScnUniformStride;
  if (needed > mScnUniformBufferSize) {
    if (mScnUniformBuffer)
      wgpuBufferRelease(static_cast<WGPUBuffer>(mScnUniformBuffer));
    WGPUBufferDescriptor ubDesc = {};
    ubDesc.size = needed;
    ubDesc.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
    mScnUniformBuffer = wgpuDeviceCreateBuffer(device, &ubDesc);
    if (!mScnUniformBuffer)
      return false;
    mScnUniformBufferSize = needed;
  }
  std::vector<uint8_t> p2(needed, 0);
  for (uint32_t i = 0; i < numDraws; ++i) {
    uint8_t *slot = p2.data() + static_cast<size_t>(i) * kScnUniformStride;
    std::memcpy(slot + 0, viewProj16, 64);
    if (draws[i].modelMatrix16)
      std::memcpy(slot + 64, draws[i].modelMatrix16, 64);
    const float baseColor[4] = {draws[i].baseColorR, draws[i].baseColorG, draws[i].baseColorB,
                                draws[i].baseColorA};
    std::memcpy(slot + 128, baseColor, 16);
    std::memcpy(slot + 144, lightDirAmbient4, 16);
    if (cameraWorldPos3)
      std::memcpy(slot + 164, cameraWorldPos3, 12);
    // Emissive (pad1.xyz, same convention as the AgX path): self-lit surfaces — shop strips,
    // traffic lights, headlights — glow independently of the sun/day-night dimming.
    const float emissive[3] = {draws[i].emissiveR, draws[i].emissiveG, draws[i].emissiveB};
    std::memcpy(slot + 176, emissive, 12);
    std::memcpy(slot + 188, &draws[i].specularStrength, 4);
  }
  wgpuQueueWriteBuffer(queue, static_cast<WGPUBuffer>(mScnUniformBuffer), 0, p2.data(), needed);

  // --- Depth-pass uniform: one slot per (cascade, draw): {lightViewProj[c], model}. ---
  const size_t depthNeeded = static_cast<size_t>(cascadeCount) * slotCount * kScnUniformStride;
  if (depthNeeded > mCsmDepthUniformBufferSize) {
    if (mCsmDepthUniformBuffer)
      wgpuBufferRelease(static_cast<WGPUBuffer>(mCsmDepthUniformBuffer));
    WGPUBufferDescriptor ubDesc = {};
    ubDesc.size = depthNeeded;
    ubDesc.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
    mCsmDepthUniformBuffer = wgpuDeviceCreateBuffer(device, &ubDesc);
    if (!mCsmDepthUniformBuffer)
      return false;
    mCsmDepthUniformBufferSize = depthNeeded;
  }
  std::vector<uint8_t> pdepth(depthNeeded, 0);
  for (uint32_t c = 0; c < cascadeCount; ++c) {
    const float *lvp = cascadeLightViewProjs + static_cast<size_t>(c) * 16;
    for (uint32_t i = 0; i < numDraws; ++i) {
      uint8_t *slot = pdepth.data() + (static_cast<size_t>(c) * slotCount + i) * kScnUniformStride;
      std::memcpy(slot + 0, lvp, 64);
      if (draws[i].modelMatrix16)
        std::memcpy(slot + 64, draws[i].modelMatrix16, 64);
    }
  }
  wgpuQueueWriteBuffer(queue, static_cast<WGPUBuffer>(mCsmDepthUniformBuffer), 0, pdepth.data(),
                       depthNeeded);

  WGPUBindGroupEntry de = {};
  de.binding = 0;
  de.buffer = static_cast<WGPUBuffer>(mCsmDepthUniformBuffer);
  de.offset = 0;
  de.size = 192;
  WGPUBindGroupDescriptor dd = {};
  dd.layout = static_cast<WGPUBindGroupLayout>(mScnBindGroupLayout);
  dd.entryCount = 1;
  dd.entries = &de;
  WGPUBindGroup dbg = wgpuDeviceCreateBindGroup(device, &dd);
  if (!dbg)
    return false;

  // --- CsmLightU (336 B): 4 light VPs + shadowParams(strength,bias,hemiOn,cascadeCount) +
  // cascadeSplits + skyColor + groundColor + upDir. ---
  if (!mCsmLightUniformBuffer) {
    WGPUBufferDescriptor ud = {};
    ud.size = 336;
    ud.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
    mCsmLightUniformBuffer = wgpuDeviceCreateBuffer(device, &ud);
    if (!mCsmLightUniformBuffer) {
      wgpuBindGroupRelease(dbg);
      return false;
    }
  }
  {
    static const float kIdent[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    uint8_t lu[336] = {0};
    for (uint32_t c = 0; c < 4; ++c) {
      const float *vpc =
        (c < cascadeCount) ? (cascadeLightViewProjs + static_cast<size_t>(c) * 16) : kIdent;
      std::memcpy(lu + static_cast<size_t>(c) * 64, vpc, 64);
    }
    const float hemiOn = hemiSky4 ? 1.0f : 0.0f;
    const float sp[4] = {shadowStrength, depthBias, hemiOn, static_cast<float>(cascadeCount)};
    std::memcpy(lu + 256, sp, 16);
    std::memcpy(lu + 272, cascadeSplitsFar4, 16);
    if (hemiSky4)
      std::memcpy(lu + 288, hemiSky4, 16);
    if (hemiGround4)
      std::memcpy(lu + 304, hemiGround4, 16);
    if (worldUp3)
      std::memcpy(lu + 320, worldUp3, 12);
    wgpuQueueWriteBuffer(queue, static_cast<WGPUBuffer>(mCsmLightUniformBuffer), 0, lu, 336);
  }

  WGPUCommandEncoderDescriptor encDesc = {};
  WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(device, &encDesc);
  if (!encoder) {
    wgpuBindGroupRelease(dbg);
    return false;
  }

  // ===== N LIGHT-DEPTH PASSES: clip.z into each cascade layer =====
  for (uint32_t c = 0; c < cascadeCount; ++c) {
    WGPURenderPassColorAttachment ca = {};
    ca.view = static_cast<WGPUTextureView>(mCsmShadowLayerViews[c]);
    ca.loadOp = WGPULoadOp_Clear;
    ca.storeOp = WGPUStoreOp_Store;
    ca.clearValue = {1.0, 0.0, 0.0, 1.0};
    ca.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
    WGPURenderPassDepthStencilAttachment da = {};
    da.view = static_cast<WGPUTextureView>(mCsmShadowDepthView);
    da.depthLoadOp = WGPULoadOp_Clear;
    da.depthStoreOp = WGPUStoreOp_Store;
    da.depthClearValue = 1.0f;
    WGPURenderPassDescriptor pd = {};
    pd.colorAttachmentCount = 1;
    pd.colorAttachments = &ca;
    pd.depthStencilAttachment = &da;
    WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &pd);
    if (!pass) {
      wgpuCommandEncoderRelease(encoder);
      wgpuBindGroupRelease(dbg);
      return false;
    }
    wgpuRenderPassEncoderSetPipeline(pass, static_cast<WGPURenderPipeline>(mScnClipDepthPipeline));
    for (uint32_t i = 0; i < numDraws; ++i) {
      const WbWgpuSolidDraw &d = draws[i];
      if (!d.vertexBuffer || !d.indexBuffer || d.indexCount == 0)
        continue;
      const uint32_t dyn =
        static_cast<uint32_t>((static_cast<size_t>(c) * slotCount + i) * kScnUniformStride);
      wgpuRenderPassEncoderSetBindGroup(pass, 0, dbg, 1, &dyn);
      wgpuRenderPassEncoderSetVertexBuffer(pass, 0, static_cast<WGPUBuffer>(d.vertexBuffer), 0,
                                           WGPU_WHOLE_SIZE);
      wgpuRenderPassEncoderSetIndexBuffer(pass, static_cast<WGPUBuffer>(d.indexBuffer),
                                          WGPUIndexFormat_Uint32, 0, WGPU_WHOLE_SIZE);
      wgpuRenderPassEncoderDrawIndexed(pass, d.indexCount, 1, 0, 0, 0);
    }
    wgpuRenderPassEncoderEnd(pass);
    wgpuRenderPassEncoderRelease(pass);
  }

  // ===== LIT PASS: kSolidLitTexturedCsm (material + multi-cascade shadow) =====
  std::vector<WGPUBindGroup> bgs;
  {
    WGPURenderPassColorAttachment ca = {};
    ca.view = static_cast<WGPUTextureView>(mView);
    ca.loadOp = WGPULoadOp_Clear;
    ca.storeOp = WGPUStoreOp_Store;
    ca.clearValue = {color.r, color.g, color.b, color.a};
    ca.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
    WGPURenderPassDepthStencilAttachment da = {};
    da.view = static_cast<WGPUTextureView>(mScnDepthView);
    da.depthLoadOp = WGPULoadOp_Clear;
    da.depthStoreOp = WGPUStoreOp_Store;
    da.depthClearValue = 1.0f;
    WGPURenderPassDescriptor pd = {};
    pd.colorAttachmentCount = 1;
    pd.colorAttachments = &ca;
    pd.depthStencilAttachment = &da;
    WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &pd);
    if (!pass) {
      wgpuCommandEncoderRelease(encoder);
      wgpuBindGroupRelease(dbg);
      return false;
    }
    wgpuRenderPassEncoderSetPipeline(pass, static_cast<WGPURenderPipeline>(mTexCsmPipeline));
    for (uint32_t i = 0; i < numDraws; ++i) {
      const WbWgpuSolidDraw &d = draws[i];
      if (!d.vertexBuffer || !d.indexBuffer || d.indexCount == 0)
        continue;
      WGPUBindGroupEntry e[9] = {};
      e[0].binding = 0;
      e[0].buffer = static_cast<WGPUBuffer>(mScnUniformBuffer);
      e[0].size = 192;
      e[1].binding = 1;
      e[1].textureView = static_cast<WGPUTextureView>(d.textureView ? d.textureView : mScnDefaultWhiteView);
      e[2].binding = 2;
      e[2].textureView = static_cast<WGPUTextureView>(d.roughnessView ? d.roughnessView : mScnDefaultWhiteView);
      e[3].binding = 3;
      e[3].textureView = static_cast<WGPUTextureView>(d.metalnessView ? d.metalnessView : mScnDefaultBlackView);
      e[4].binding = 4;
      e[4].textureView = static_cast<WGPUTextureView>(d.normalView ? d.normalView : mScnDefaultNormalView);
      e[5].binding = 5;
      e[5].sampler = static_cast<WGPUSampler>(mScnTexSampler);
      e[6].binding = 6;
      e[6].textureView = static_cast<WGPUTextureView>(mCsmShadowArrayView);
      e[7].binding = 7;
      e[7].sampler = static_cast<WGPUSampler>(mTexCsmSampler);
      e[8].binding = 8;
      e[8].buffer = static_cast<WGPUBuffer>(mCsmLightUniformBuffer);
      e[8].size = 336;
      WGPUBindGroupDescriptor bd = {};
      bd.layout = static_cast<WGPUBindGroupLayout>(mTexCsmBindGroupLayout);
      bd.entryCount = 9;
      bd.entries = e;
      WGPUBindGroup bg = wgpuDeviceCreateBindGroup(device, &bd);
      if (!bg)
        continue;
      bgs.push_back(bg);
      const uint32_t dyn = i * kScnUniformStride;
      wgpuRenderPassEncoderSetBindGroup(pass, 0, bg, 1, &dyn);
      wgpuRenderPassEncoderSetVertexBuffer(pass, 0, static_cast<WGPUBuffer>(d.vertexBuffer), 0,
                                           WGPU_WHOLE_SIZE);
      wgpuRenderPassEncoderSetIndexBuffer(pass, static_cast<WGPUBuffer>(d.indexBuffer),
                                          WGPUIndexFormat_Uint32, 0, WGPU_WHOLE_SIZE);
      wgpuRenderPassEncoderDrawIndexed(pass, d.indexCount, 1, 0, 0, 0);
    }
    wgpuRenderPassEncoderEnd(pass);
    wgpuRenderPassEncoderRelease(pass);
  }

  const uint32_t stride = alignedBytesPerRow(mWidth);
  WGPUTexelCopyTextureInfo src = {};
  src.texture = static_cast<WGPUTexture>(mTexture);
  src.aspect = WGPUTextureAspect_All;
  WGPUTexelCopyBufferInfo dst = {};
  dst.buffer = static_cast<WGPUBuffer>(mReadbackBuffer);
  dst.layout.bytesPerRow = stride;
  dst.layout.rowsPerImage = mHeight;
  WGPUExtent3D extent = {mWidth, mHeight, 1};
  wgpuCommandEncoderCopyTextureToBuffer(encoder, &src, &dst, &extent);

  WGPUCommandBufferDescriptor cmdDesc = {};
  WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(encoder, &cmdDesc);
  wgpuCommandEncoderRelease(encoder);
  if (!cmd) {
    wgpuBindGroupRelease(dbg);
    for (WGPUBindGroup b : bgs)
      wgpuBindGroupRelease(b);
    return false;
  }
  wgpuQueueSubmit(queue, 1, &cmd);
  wgpuCommandBufferRelease(cmd);

  MapCapture cap;
  WGPUBufferMapCallbackInfo mapCb = {};
  mapCb.mode = WGPUCallbackMode_AllowSpontaneous;
  mapCb.callback = onMap;
  mapCb.userdata1 = &cap;
  (void)wgpuBufferMapAsync(static_cast<WGPUBuffer>(mReadbackBuffer), WGPUMapMode_Read, 0,
                           mReadbackBufferSize, mapCb);
  for (int i = 0; i < 1000 && !cap.done; ++i)
    wgpuDevicePoll(device, true, nullptr);
  wgpuBindGroupRelease(dbg);
  for (WGPUBindGroup b : bgs)
    wgpuBindGroupRelease(b);
  if (!cap.done || !cap.ok)
    return false;

  const void *mapped =
    wgpuBufferGetConstMappedRange(static_cast<WGPUBuffer>(mReadbackBuffer), 0, mReadbackBufferSize);
  if (!mapped) {
    wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
    return false;
  }
  const uint32_t rowBytes = mWidth * 4u;
  uint8_t *out = static_cast<uint8_t *>(rgba8);
  const uint8_t *in = static_cast<const uint8_t *>(mapped);
  for (uint32_t y = 0; y < mHeight; ++y)
    std::memcpy(out + y * rowBytes, in + y * stride, rowBytes);
  wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
  return true;
#  else
  (void)color; (void)viewProj16; (void)cascadeLightViewProjs; (void)cascadeSplitsFar4;
  (void)cascadeCount; (void)lightDirAmbient4; (void)draws; (void)numDraws; (void)shadowStrength;
  (void)depthBias; (void)cameraWorldPos3; (void)hemiSky4; (void)hemiGround4; (void)worldUp3; (void)rgba8;
  return false;
#  endif
#else
  (void)color; (void)viewProj16; (void)cascadeLightViewProjs; (void)cascadeSplitsFar4;
  (void)cascadeCount; (void)lightDirAmbient4; (void)draws; (void)numDraws; (void)shadowStrength;
  (void)depthBias; (void)cameraWorldPos3; (void)hemiSky4; (void)hemiGround4; (void)worldUp3; (void)rgba8;
  return false;
#endif
}

// OWN 3-entry layout: ShadowScene uniform @0 (240 B: viewProj+model+lightViewProj
// + baseColor+light+shadowParams) + shadow texture @1 + sampler @2. The shadow
// map is R32Float, which is NON-FILTERABLE in core WebGPU, so the texture entry
// is unfilterable-float and the sampler is non-filtering (the shader samples
// exact texel via textureSampleLevel LOD 0). Reuses ensureScenePipeline's
// Depth24Plus depth view. Building this is what naga-VALIDATES the 3a WGSL —
// the first time kSolidLitShadow is actually compiled by the driver.
bool WbWgpuRenderTarget::ensureSceneShadowPipeline() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (mScnShadowPipeline)
    return true;
  if (!ensureScenePipeline())  // creates mScnDepthView (shared Depth24Plus attachment)
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());

  WGPUShaderSourceWGSL wgsl = {};
  wgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
  wgsl.code.data = WbWgpuShaders::kSolidLitShadow;
  wgsl.code.length = WGPU_STRLEN;
  WGPUShaderModuleDescriptor smDesc = {};
  smDesc.nextInChain = &wgsl.chain;
  WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &smDesc);
  if (!sm) {
    WbLog::info("[WbWgpuRenderTarget] scn-shadow CreateShaderModule failed");
    return false;
  }
  mScnShadowShaderModule = sm;

  // Non-filtering sampler (R32Float is non-filterable in core WebGPU).
  WGPUSamplerDescriptor sampDesc = {};
  sampDesc.addressModeU = WGPUAddressMode_ClampToEdge;
  sampDesc.addressModeV = WGPUAddressMode_ClampToEdge;
  sampDesc.addressModeW = WGPUAddressMode_ClampToEdge;
  sampDesc.magFilter = WGPUFilterMode_Nearest;
  sampDesc.minFilter = WGPUFilterMode_Nearest;
  sampDesc.mipmapFilter = WGPUMipmapFilterMode_Nearest;
  sampDesc.lodMinClamp = 0.0f;
  sampDesc.lodMaxClamp = 1.0f;
  sampDesc.compare = WGPUCompareFunction_Undefined;
  sampDesc.maxAnisotropy = 1;
  WGPUSampler samp = wgpuDeviceCreateSampler(device, &sampDesc);
  if (!samp) {
    WbLog::info("[WbWgpuRenderTarget] scn-shadow CreateSampler failed");
    return false;
  }
  mScnShadowSampler = samp;

  // 3-entry bind-group layout: uniform @0, unfilterable-float texture @1,
  // non-filtering sampler @2.
  WGPUBindGroupLayoutEntry bglEntries[3] = {};
  bglEntries[0].binding = 0;
  bglEntries[0].visibility = WGPUShaderStage_Vertex | WGPUShaderStage_Fragment;
  bglEntries[0].buffer.type = WGPUBufferBindingType_Uniform;
  bglEntries[0].buffer.hasDynamicOffset = 1;
  bglEntries[0].buffer.minBindingSize = 240;  // ShadowScene: 3 mat4 + 3 vec4
  bglEntries[1].binding = 1;
  bglEntries[1].visibility = WGPUShaderStage_Fragment;
  bglEntries[1].texture.sampleType = WGPUTextureSampleType_UnfilterableFloat;
  bglEntries[1].texture.viewDimension = WGPUTextureViewDimension_2D;
  bglEntries[1].texture.multisampled = 0;
  bglEntries[2].binding = 2;
  bglEntries[2].visibility = WGPUShaderStage_Fragment;
  bglEntries[2].sampler.type = WGPUSamplerBindingType_NonFiltering;
  WGPUBindGroupLayoutDescriptor bglDesc = {};
  bglDesc.entryCount = 3;
  bglDesc.entries = bglEntries;
  WGPUBindGroupLayout bgl = wgpuDeviceCreateBindGroupLayout(device, &bglDesc);
  if (!bgl) {
    WbLog::info("[WbWgpuRenderTarget] scn-shadow CreateBindGroupLayout failed");
    return false;
  }
  mScnShadowBindGroupLayout = bgl;

  WGPUPipelineLayoutDescriptor plDesc = {};
  plDesc.bindGroupLayoutCount = 1;
  plDesc.bindGroupLayouts = &bgl;
  WGPUPipelineLayout plLayout = wgpuDeviceCreatePipelineLayout(device, &plDesc);
  if (!plLayout) {
    WbLog::info("[WbWgpuRenderTarget] scn-shadow CreatePipelineLayout failed");
    return false;
  }

  WGPUVertexAttribute attrs[3] = {};
  attrs[0].format = WGPUVertexFormat_Float32x3; attrs[0].offset = 0;  attrs[0].shaderLocation = 0;
  attrs[1].format = WGPUVertexFormat_Float32x3; attrs[1].offset = 12; attrs[1].shaderLocation = 1;
  attrs[2].format = WGPUVertexFormat_Float32x2; attrs[2].offset = 24; attrs[2].shaderLocation = 2;
  WGPUVertexBufferLayout vbLayout = {};
  vbLayout.arrayStride = 32;
  vbLayout.stepMode = WGPUVertexStepMode_Vertex;
  vbLayout.attributeCount = 3;
  vbLayout.attributes = attrs;

  WGPUColorTargetState colorTarget = {};
  colorTarget.format = WGPUTextureFormat_RGBA8Unorm;
  colorTarget.writeMask = WGPUColorWriteMask_All;

  WGPUFragmentState fs = {};
  fs.module = sm;
  fs.entryPoint.data = "fs_main";
  fs.entryPoint.length = WGPU_STRLEN;
  fs.targetCount = 1;
  fs.targets = &colorTarget;

  WGPUDepthStencilState depthState = {};
  depthState.format = WGPUTextureFormat_Depth24Plus;
  depthState.depthWriteEnabled = WGPUOptionalBool_True;
  depthState.depthCompare = WGPUCompareFunction_Less;
  depthState.stencilFront.compare = WGPUCompareFunction_Always;
  depthState.stencilFront.failOp = WGPUStencilOperation_Keep;
  depthState.stencilFront.depthFailOp = WGPUStencilOperation_Keep;
  depthState.stencilFront.passOp = WGPUStencilOperation_Keep;
  depthState.stencilBack = depthState.stencilFront;
  depthState.stencilReadMask = 0xFFFFFFFFu;
  depthState.stencilWriteMask = 0xFFFFFFFFu;

  WGPURenderPipelineDescriptor pipeDesc = {};
  pipeDesc.layout = plLayout;
  pipeDesc.vertex.module = sm;
  pipeDesc.vertex.entryPoint.data = "vs_main";
  pipeDesc.vertex.entryPoint.length = WGPU_STRLEN;
  pipeDesc.vertex.bufferCount = 1;
  pipeDesc.vertex.buffers = &vbLayout;
  pipeDesc.primitive.topology = WGPUPrimitiveTopology_TriangleList;
  pipeDesc.primitive.frontFace = WGPUFrontFace_CCW;
  pipeDesc.primitive.cullMode = WGPUCullMode_None;
  pipeDesc.depthStencil = &depthState;
  pipeDesc.multisample.count = 1;
  pipeDesc.multisample.mask = 0xFFFFFFFFu;
  pipeDesc.fragment = &fs;
  WGPURenderPipeline pipe = wgpuDeviceCreateRenderPipeline(device, &pipeDesc);
  wgpuPipelineLayoutRelease(plLayout);
  if (!pipe) {
    WbLog::info("[WbWgpuRenderTarget] scn-shadow CreateRenderPipeline failed");
    return false;
  }
  mScnShadowPipeline = pipe;
  return true;
#  else
  return false;
#  endif
#else
  return false;
#endif
}

// T1.2 CSM sub-step 3c: lazily build the GPU-resident SAMPLEABLE shadow map (+
// its own Depth24Plus occlusion attachment for the light pass). Unlike
// mDepthF32Texture (RenderAttachment|CopySrc, for CPU readback), this is
// RenderAttachment|TextureBinding so the light-depth pass renders into it and
// the lit pass samples it directly — no CPU round-trip (the GPU-resident point
// the checklist calls for). Sized to the render target (square-ish shadow map
// at the camera resolution; a real CSM would size cascades independently).
bool WbWgpuRenderTarget::ensureShadowMapTexture() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (mShadowMapView && mShadowMapDepthView)
    return true;
  if (!mUsable)
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());

  WGPUTextureDescriptor texDesc = {};
  texDesc.usage = WGPUTextureUsage_RenderAttachment | WGPUTextureUsage_TextureBinding |
                  WGPUTextureUsage_CopySrc;  // CopySrc: OMNISIM_WGPU_SHADOWMAP_DUMP diagnostics
  texDesc.dimension = WGPUTextureDimension_2D;
  texDesc.size = {mWidth, mHeight, 1};
  texDesc.format = WGPUTextureFormat_R32Float;
  texDesc.mipLevelCount = 1;
  texDesc.sampleCount = 1;
  WGPUTexture tex = wgpuDeviceCreateTexture(device, &texDesc);
  if (!tex) {
    WbLog::info("[WbWgpuRenderTarget] shadow-map CreateTexture failed");
    return false;
  }
  mShadowMapTexture = tex;
  WGPUTextureView view = wgpuTextureCreateView(tex, nullptr);
  if (!view) {
    WbLog::info("[WbWgpuRenderTarget] shadow-map TextureCreateView failed");
    return false;
  }
  mShadowMapView = view;

  WGPUTextureDescriptor dtDesc = {};
  dtDesc.usage = WGPUTextureUsage_RenderAttachment;
  dtDesc.dimension = WGPUTextureDimension_2D;
  dtDesc.size = {mWidth, mHeight, 1};
  dtDesc.format = WGPUTextureFormat_Depth24Plus;
  dtDesc.mipLevelCount = 1;
  dtDesc.sampleCount = 1;
  WGPUTexture dt = wgpuDeviceCreateTexture(device, &dtDesc);
  if (!dt) {
    WbLog::info("[WbWgpuRenderTarget] shadow-map depth CreateTexture failed");
    return false;
  }
  mShadowMapDepthTexture = dt;
  WGPUTextureView dv = wgpuTextureCreateView(dt, nullptr);
  if (!dv) {
    WbLog::info("[WbWgpuRenderTarget] shadow-map depth view failed");
    return false;
  }
  mShadowMapDepthView = dv;
  return true;
#  else
  return false;
#  endif
#else
  return false;
#endif
}

// T1.2 CSM sub-step 3c: two-pass shadowed render. See header. Pass 1 → light
// clip-depth into the sampleable shadow map; pass 2 → lit+shadow into the color
// target sampling it. Self-contained; clearAndDrawScene is byte-untouched.
bool WbWgpuRenderTarget::clearAndDrawSceneShadowed(const WbWgpuClearColor &color,
                                                   const float *viewProj16,
                                                   const float *lightViewProj16,
                                                   const float *lightDirAmbient4,
                                                   const WbWgpuSolidDraw *draws, uint32_t numDraws,
                                                   float shadowStrength, float depthBias,
                                                   void *rgba8) {
  if (!mUsable || !rgba8 || !viewProj16 || !lightViewProj16 || !lightDirAmbient4)
    return false;
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (!ensureSceneClipDepthF32Pipeline())  // pass-1 pipeline (clip.z) + R32Float infra
    return false;
  if (!ensureSceneShadowPipeline())        // pass-2 pipeline + 3-entry layout + sampler
    return false;
  if (!ensureShadowMapTexture())           // GPU-resident sampleable shadow map
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUQueue queue = static_cast<WGPUQueue>(mBackend->queue());

  const uint32_t slotCount = numDraws == 0 ? 1u : numDraws;
  const size_t needed = static_cast<size_t>(slotCount) * kScnUniformStride;

  // --- Pass-1 uniform: reuse the scene uniform buffer with the clip-depth Scene
  // layout {viewProj=lightViewProj, model} (the clip-depth shader only reads
  // those two). Grown like clearAndDrawSceneDepthF32. ---
  if (needed > mScnUniformBufferSize) {
    if (mScnUniformBuffer)
      wgpuBufferRelease(static_cast<WGPUBuffer>(mScnUniformBuffer));
    WGPUBufferDescriptor ubDesc = {};
    ubDesc.size = needed;
    ubDesc.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
    WGPUBuffer ub = wgpuDeviceCreateBuffer(device, &ubDesc);
    if (!ub)
      return false;
    mScnUniformBuffer = ub;
    mScnUniformBufferSize = needed;
  }
  std::vector<uint8_t> p1(needed, 0);
  for (uint32_t i = 0; i < numDraws; ++i) {
    uint8_t *slot = p1.data() + static_cast<size_t>(i) * kScnUniformStride;
    std::memcpy(slot + 0, lightViewProj16, 64);
    if (draws[i].modelMatrix16)
      std::memcpy(slot + 64, draws[i].modelMatrix16, 64);
  }
  wgpuQueueWriteBuffer(queue, static_cast<WGPUBuffer>(mScnUniformBuffer), 0, p1.data(), needed);

  WGPUBindGroupEntry p1e = {};
  p1e.binding = 0;
  p1e.buffer = static_cast<WGPUBuffer>(mScnUniformBuffer);
  p1e.offset = 0;
  p1e.size = 192;
  WGPUBindGroupDescriptor p1d = {};
  p1d.layout = static_cast<WGPUBindGroupLayout>(mScnBindGroupLayout);
  p1d.entryCount = 1;
  p1d.entries = &p1e;
  WGPUBindGroup p1bg = wgpuDeviceCreateBindGroup(device, &p1d);
  if (!p1bg)
    return false;

  // --- Pass-2 uniform: separate buffer with the ShadowScene layout (240 B
  // payload, 256 B stride): viewProj, model, lightViewProj, baseColor, light,
  // shadowParams(strength,bias,0,0). ---
  if (needed > mShadowUniformBufferSize) {
    if (mShadowUniformBuffer)
      wgpuBufferRelease(static_cast<WGPUBuffer>(mShadowUniformBuffer));
    WGPUBufferDescriptor ubDesc = {};
    ubDesc.size = needed;
    ubDesc.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
    WGPUBuffer ub = wgpuDeviceCreateBuffer(device, &ubDesc);
    if (!ub) {
      wgpuBindGroupRelease(p1bg);
      return false;
    }
    mShadowUniformBuffer = ub;
    mShadowUniformBufferSize = needed;
  }
  std::vector<uint8_t> p2(needed, 0);
  for (uint32_t i = 0; i < numDraws; ++i) {
    uint8_t *slot = p2.data() + static_cast<size_t>(i) * kScnUniformStride;
    std::memcpy(slot + 0, viewProj16, 64);
    if (draws[i].modelMatrix16)
      std::memcpy(slot + 64, draws[i].modelMatrix16, 64);
    std::memcpy(slot + 128, lightViewProj16, 64);
    float baseColor[4] = {draws[i].baseColorR, draws[i].baseColorG, draws[i].baseColorB,
                          draws[i].baseColorA};
    std::memcpy(slot + 192, baseColor, 16);
    std::memcpy(slot + 208, lightDirAmbient4, 16);
    float shadowParams[4] = {shadowStrength, depthBias, 0.0f, 0.0f};
    std::memcpy(slot + 224, shadowParams, 16);
  }
  wgpuQueueWriteBuffer(queue, static_cast<WGPUBuffer>(mShadowUniformBuffer), 0, p2.data(), needed);

  WGPUBindGroupEntry p2e[3] = {};
  p2e[0].binding = 0;
  p2e[0].buffer = static_cast<WGPUBuffer>(mShadowUniformBuffer);
  p2e[0].offset = 0;
  p2e[0].size = 240;
  p2e[1].binding = 1;
  p2e[1].textureView = static_cast<WGPUTextureView>(mShadowMapView);
  p2e[2].binding = 2;
  p2e[2].sampler = static_cast<WGPUSampler>(mScnShadowSampler);
  WGPUBindGroupDescriptor p2d = {};
  p2d.layout = static_cast<WGPUBindGroupLayout>(mScnShadowBindGroupLayout);
  p2d.entryCount = 3;
  p2d.entries = p2e;
  WGPUBindGroup p2bg = wgpuDeviceCreateBindGroup(device, &p2d);
  if (!p2bg) {
    wgpuBindGroupRelease(p1bg);
    return false;
  }

  WGPUCommandEncoderDescriptor encDesc = {};
  WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(device, &encDesc);
  if (!encoder) {
    wgpuBindGroupRelease(p1bg);
    wgpuBindGroupRelease(p2bg);
    return false;
  }

  // ===== PASS 1: light clip-depth → shadow map =====
  {
    WGPURenderPassColorAttachment ca = {};
    ca.view = static_cast<WGPUTextureView>(mShadowMapView);
    ca.loadOp = WGPULoadOp_Clear;
    ca.storeOp = WGPUStoreOp_Store;
    ca.clearValue = {1.0, 0.0, 0.0, 1.0};  // far = 1.0 (nothing occludes)
    ca.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
    WGPURenderPassDepthStencilAttachment da = {};
    da.view = static_cast<WGPUTextureView>(mShadowMapDepthView);
    da.depthLoadOp = WGPULoadOp_Clear;
    da.depthStoreOp = WGPUStoreOp_Store;
    da.depthClearValue = 1.0f;
    da.depthReadOnly = 0;
    WGPURenderPassDescriptor pd = {};
    pd.colorAttachmentCount = 1;
    pd.colorAttachments = &ca;
    pd.depthStencilAttachment = &da;
    WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &pd);
    if (!pass) {
      wgpuCommandEncoderRelease(encoder);
      wgpuBindGroupRelease(p1bg);
      wgpuBindGroupRelease(p2bg);
      return false;
    }
    wgpuRenderPassEncoderSetPipeline(pass, static_cast<WGPURenderPipeline>(mScnClipDepthPipeline));
    for (uint32_t i = 0; i < numDraws; ++i) {
      const WbWgpuSolidDraw &d = draws[i];
      if (!d.vertexBuffer || !d.indexBuffer || d.indexCount == 0 || !d.castShadows)
        continue;
      const uint32_t dyn = i * kScnUniformStride;
      wgpuRenderPassEncoderSetBindGroup(pass, 0, p1bg, 1, &dyn);
      wgpuRenderPassEncoderSetVertexBuffer(pass, 0, static_cast<WGPUBuffer>(d.vertexBuffer), 0,
                                           WGPU_WHOLE_SIZE);
      wgpuRenderPassEncoderSetIndexBuffer(pass, static_cast<WGPUBuffer>(d.indexBuffer),
                                          WGPUIndexFormat_Uint32, 0, WGPU_WHOLE_SIZE);
      wgpuRenderPassEncoderDrawIndexed(pass, d.indexCount, 1, 0, 0, 0);
    }
    wgpuRenderPassEncoderEnd(pass);
    wgpuRenderPassEncoderRelease(pass);
  }

  // ===== PASS 2: lit + shadow → color target =====
  {
    WGPURenderPassColorAttachment ca = {};
    ca.view = static_cast<WGPUTextureView>(mView);
    ca.loadOp = WGPULoadOp_Clear;
    ca.storeOp = WGPUStoreOp_Store;
    ca.clearValue = {color.r, color.g, color.b, color.a};
    ca.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
    WGPURenderPassDepthStencilAttachment da = {};
    da.view = static_cast<WGPUTextureView>(mScnDepthView);
    da.depthLoadOp = WGPULoadOp_Clear;
    da.depthStoreOp = WGPUStoreOp_Store;
    da.depthClearValue = 1.0f;
    da.depthReadOnly = 0;
    WGPURenderPassDescriptor pd = {};
    pd.colorAttachmentCount = 1;
    pd.colorAttachments = &ca;
    pd.depthStencilAttachment = &da;
    WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &pd);
    if (!pass) {
      wgpuCommandEncoderRelease(encoder);
      wgpuBindGroupRelease(p1bg);
      wgpuBindGroupRelease(p2bg);
      return false;
    }
    wgpuRenderPassEncoderSetPipeline(pass, static_cast<WGPURenderPipeline>(mScnShadowPipeline));
    for (uint32_t i = 0; i < numDraws; ++i) {
      const WbWgpuSolidDraw &d = draws[i];
      if (!d.vertexBuffer || !d.indexBuffer || d.indexCount == 0)
        continue;
      const uint32_t dyn = i * kScnUniformStride;
      wgpuRenderPassEncoderSetBindGroup(pass, 0, p2bg, 1, &dyn);
      wgpuRenderPassEncoderSetVertexBuffer(pass, 0, static_cast<WGPUBuffer>(d.vertexBuffer), 0,
                                           WGPU_WHOLE_SIZE);
      wgpuRenderPassEncoderSetIndexBuffer(pass, static_cast<WGPUBuffer>(d.indexBuffer),
                                          WGPUIndexFormat_Uint32, 0, WGPU_WHOLE_SIZE);
      wgpuRenderPassEncoderDrawIndexed(pass, d.indexCount, 1, 0, 0, 0);
    }
    wgpuRenderPassEncoderEnd(pass);
    wgpuRenderPassEncoderRelease(pass);
  }

  // Copy the lit color target to the readback buffer.
  const uint32_t stride = alignedBytesPerRow(mWidth);
  WGPUTexelCopyTextureInfo src = {};
  src.texture = static_cast<WGPUTexture>(mTexture);
  src.aspect = WGPUTextureAspect_All;
  WGPUTexelCopyBufferInfo dst = {};
  dst.buffer = static_cast<WGPUBuffer>(mReadbackBuffer);
  dst.layout.bytesPerRow = stride;
  dst.layout.rowsPerImage = mHeight;
  WGPUExtent3D extent = {mWidth, mHeight, 1};
  wgpuCommandEncoderCopyTextureToBuffer(encoder, &src, &dst, &extent);

  WGPUCommandBufferDescriptor cmdDesc = {};
  WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(encoder, &cmdDesc);
  wgpuCommandEncoderRelease(encoder);
  if (!cmd) {
    wgpuBindGroupRelease(p1bg);
    wgpuBindGroupRelease(p2bg);
    return false;
  }
  wgpuQueueSubmit(queue, 1, &cmd);
  wgpuCommandBufferRelease(cmd);

  MapCapture cap;
  WGPUBufferMapCallbackInfo mapCb = {};
  mapCb.mode = WGPUCallbackMode_AllowSpontaneous;
  mapCb.callback = onMap;
  mapCb.userdata1 = &cap;
  (void)wgpuBufferMapAsync(static_cast<WGPUBuffer>(mReadbackBuffer), WGPUMapMode_Read, 0,
                           mReadbackBufferSize, mapCb);
  for (int i = 0; i < 1000 && !cap.done; ++i)
    wgpuDevicePoll(device, true, nullptr);
  wgpuBindGroupRelease(p1bg);
  wgpuBindGroupRelease(p2bg);
  if (!cap.done || !cap.ok)
    return false;

  const void *mapped = wgpuBufferGetConstMappedRange(static_cast<WGPUBuffer>(mReadbackBuffer), 0,
                                                    mReadbackBufferSize);
  if (!mapped) {
    wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
    return false;
  }
  const uint32_t rowBytes = mWidth * 4u;
  uint8_t *out = static_cast<uint8_t *>(rgba8);
  const uint8_t *in = static_cast<const uint8_t *>(mapped);
  for (uint32_t y = 0; y < mHeight; ++y)
    std::memcpy(out + y * rowBytes, in + y * stride, rowBytes);
  wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
  return true;
#  else
  (void)color; (void)viewProj16; (void)lightViewProj16; (void)lightDirAmbient4;
  (void)draws; (void)numDraws; (void)shadowStrength; (void)depthBias; (void)rgba8;
  return false;
#  endif
#else
  (void)color; (void)viewProj16; (void)lightViewProj16; (void)lightDirAmbient4;
  (void)draws; (void)numDraws; (void)shadowStrength; (void)depthBias; (void)rgba8;
  return false;
#endif
}

// ===========================================================================
// T1.2 CSM (multi-cascade) — the ON-GPU half: an N-layer R32Float shadow-map
// array + the kSolidLitCsm lit pipeline + the N+1-pass render + a headless
// self-test. Engine translation of the GPU-proven docs/developer/
// csm_render_prototype.py design (which validated the LIVE kSolidLitCsm string +
// buildCascadeLightViewProjs on the RTX 3060). Self-contained — clearAndDrawScene
// and clearAndDrawSceneShadowed stay byte-identical.
// ===========================================================================

// Lazily (re)build the N-layer SAMPLEABLE R32Float shadow-map array (one layer per
// cascade) with per-layer 2D render views (light-depth pass targets) + a 2D-array
// sample view (lit pass) + a shared Depth24Plus occlusion attachment. Rebuilds when
// (res, cascades) changes; clamps cascades to [1, kMaxCascades].
bool WbWgpuRenderTarget::ensureShadowMapArray(uint32_t res, uint32_t cascades) {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (!mUsable)
    return false;
  if (cascades < 1)
    cascades = 1;
  if (cascades > kCsmMaxCascades)
    cascades = kCsmMaxCascades;
  if (mCsmShadowArrayTexture && mCsmShadowRes == res && mCsmCascadeCount == cascades)
    return true;
  // Changed size/count → release the prior array + views before rebuilding.
  for (void *v : mCsmShadowLayerViews)
    if (v)
      wgpuTextureViewRelease(static_cast<WGPUTextureView>(v));
  mCsmShadowLayerViews.clear();
  if (mCsmShadowArrayView) {
    wgpuTextureViewRelease(static_cast<WGPUTextureView>(mCsmShadowArrayView));
    mCsmShadowArrayView = nullptr;
  }
  if (mCsmShadowArrayTexture) {
    wgpuTextureRelease(static_cast<WGPUTexture>(mCsmShadowArrayTexture));
    mCsmShadowArrayTexture = nullptr;
  }
  if (mCsmShadowDepthView) {
    wgpuTextureViewRelease(static_cast<WGPUTextureView>(mCsmShadowDepthView));
    mCsmShadowDepthView = nullptr;
  }
  if (mCsmShadowDepthTexture) {
    wgpuTextureRelease(static_cast<WGPUTexture>(mCsmShadowDepthTexture));
    mCsmShadowDepthTexture = nullptr;
  }
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());

  WGPUTextureDescriptor texDesc = {};
  texDesc.usage = WGPUTextureUsage_RenderAttachment | WGPUTextureUsage_TextureBinding;
  texDesc.dimension = WGPUTextureDimension_2D;
  texDesc.size = {res, res, cascades};  // depthOrArrayLayers = cascades
  texDesc.format = WGPUTextureFormat_R32Float;
  texDesc.mipLevelCount = 1;
  texDesc.sampleCount = 1;
  WGPUTexture tex = wgpuDeviceCreateTexture(device, &texDesc);
  if (!tex) {
    WbLog::info("[WbWgpuRenderTarget] csm shadow-array CreateTexture failed");
    return false;
  }
  mCsmShadowArrayTexture = tex;

  WGPUTextureViewDescriptor avd = {};
  avd.format = WGPUTextureFormat_R32Float;
  avd.dimension = WGPUTextureViewDimension_2DArray;
  avd.baseArrayLayer = 0;
  avd.arrayLayerCount = cascades;
  avd.baseMipLevel = 0;
  avd.mipLevelCount = 1;
  WGPUTextureView aview = wgpuTextureCreateView(tex, &avd);
  if (!aview) {
    WbLog::info("[WbWgpuRenderTarget] csm shadow-array array-view failed");
    return false;
  }
  mCsmShadowArrayView = aview;

  for (uint32_t c = 0; c < cascades; ++c) {
    WGPUTextureViewDescriptor lvd = {};
    lvd.format = WGPUTextureFormat_R32Float;
    lvd.dimension = WGPUTextureViewDimension_2D;
    lvd.baseArrayLayer = c;
    lvd.arrayLayerCount = 1;
    lvd.baseMipLevel = 0;
    lvd.mipLevelCount = 1;
    WGPUTextureView lv = wgpuTextureCreateView(tex, &lvd);
    if (!lv) {
      WbLog::info("[WbWgpuRenderTarget] csm shadow-array layer-view failed");
      return false;
    }
    mCsmShadowLayerViews.push_back(lv);
  }

  WGPUTextureDescriptor dtDesc = {};
  dtDesc.usage = WGPUTextureUsage_RenderAttachment;
  dtDesc.dimension = WGPUTextureDimension_2D;
  dtDesc.size = {res, res, 1};
  dtDesc.format = WGPUTextureFormat_Depth24Plus;
  dtDesc.mipLevelCount = 1;
  dtDesc.sampleCount = 1;
  WGPUTexture dt = wgpuDeviceCreateTexture(device, &dtDesc);
  if (!dt) {
    WbLog::info("[WbWgpuRenderTarget] csm shadow-array depth CreateTexture failed");
    return false;
  }
  mCsmShadowDepthTexture = dt;
  WGPUTextureView dv = wgpuTextureCreateView(dt, nullptr);
  if (!dv) {
    WbLog::info("[WbWgpuRenderTarget] csm shadow-array depth view failed");
    return false;
  }
  mCsmShadowDepthView = dv;

  mCsmShadowRes = res;
  mCsmCascadeCount = cascades;
  return true;
#  else
  (void)res;
  (void)cascades;
  return false;
#  endif
#else
  (void)res;
  (void)cascades;
  return false;
#endif
}

// Build the kSolidLitCsm lit pipeline + its 3-entry layout (CsmScene uniform @0,
// 448 B dynamic + texture_2d_array @1 unfilterable-float + non-filtering sampler @2).
// Mirrors ensureSceneShadowPipeline; building it naga-VALIDATES kSolidLitCsm in-engine.
bool WbWgpuRenderTarget::ensureSceneCsmPipeline() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (mCsmPipeline)
    return true;
  if (!ensureScenePipeline())  // creates mScnDepthView (shared Depth24Plus attachment for the lit pass)
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());

  WGPUShaderSourceWGSL wgsl = {};
  wgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
  wgsl.code.data = WbWgpuShaders::kSolidLitCsm;
  wgsl.code.length = WGPU_STRLEN;
  WGPUShaderModuleDescriptor smDesc = {};
  smDesc.nextInChain = &wgsl.chain;
  WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &smDesc);
  if (!sm) {
    WbLog::info("[WbWgpuRenderTarget] csm CreateShaderModule failed");
    return false;
  }
  mCsmShaderModule = sm;

  WGPUSamplerDescriptor sampDesc = {};
  sampDesc.addressModeU = WGPUAddressMode_ClampToEdge;
  sampDesc.addressModeV = WGPUAddressMode_ClampToEdge;
  sampDesc.addressModeW = WGPUAddressMode_ClampToEdge;
  sampDesc.magFilter = WGPUFilterMode_Nearest;
  sampDesc.minFilter = WGPUFilterMode_Nearest;
  sampDesc.mipmapFilter = WGPUMipmapFilterMode_Nearest;
  sampDesc.lodMinClamp = 0.0f;
  sampDesc.lodMaxClamp = 1.0f;
  sampDesc.compare = WGPUCompareFunction_Undefined;
  sampDesc.maxAnisotropy = 1;
  WGPUSampler samp = wgpuDeviceCreateSampler(device, &sampDesc);
  if (!samp) {
    WbLog::info("[WbWgpuRenderTarget] csm CreateSampler failed");
    return false;
  }
  mCsmSampler = samp;

  WGPUBindGroupLayoutEntry bglEntries[3] = {};
  bglEntries[0].binding = 0;
  bglEntries[0].visibility = WGPUShaderStage_Vertex | WGPUShaderStage_Fragment;
  bglEntries[0].buffer.type = WGPUBufferBindingType_Uniform;
  bglEntries[0].buffer.hasDynamicOffset = 1;
  bglEntries[0].buffer.minBindingSize = 448;  // CsmScene: viewProj+model + 4 light VPs + 4 vec4
  bglEntries[1].binding = 1;
  bglEntries[1].visibility = WGPUShaderStage_Fragment;
  bglEntries[1].texture.sampleType = WGPUTextureSampleType_UnfilterableFloat;
  bglEntries[1].texture.viewDimension = WGPUTextureViewDimension_2DArray;
  bglEntries[1].texture.multisampled = 0;
  bglEntries[2].binding = 2;
  bglEntries[2].visibility = WGPUShaderStage_Fragment;
  bglEntries[2].sampler.type = WGPUSamplerBindingType_NonFiltering;
  WGPUBindGroupLayoutDescriptor bglDesc = {};
  bglDesc.entryCount = 3;
  bglDesc.entries = bglEntries;
  WGPUBindGroupLayout bgl = wgpuDeviceCreateBindGroupLayout(device, &bglDesc);
  if (!bgl) {
    WbLog::info("[WbWgpuRenderTarget] csm CreateBindGroupLayout failed");
    return false;
  }
  mCsmBindGroupLayout = bgl;

  WGPUPipelineLayoutDescriptor plDesc = {};
  plDesc.bindGroupLayoutCount = 1;
  plDesc.bindGroupLayouts = &bgl;
  WGPUPipelineLayout plLayout = wgpuDeviceCreatePipelineLayout(device, &plDesc);
  if (!plLayout) {
    WbLog::info("[WbWgpuRenderTarget] csm CreatePipelineLayout failed");
    return false;
  }

  WGPUVertexAttribute attrs[3] = {};
  attrs[0].format = WGPUVertexFormat_Float32x3; attrs[0].offset = 0;  attrs[0].shaderLocation = 0;
  attrs[1].format = WGPUVertexFormat_Float32x3; attrs[1].offset = 12; attrs[1].shaderLocation = 1;
  attrs[2].format = WGPUVertexFormat_Float32x2; attrs[2].offset = 24; attrs[2].shaderLocation = 2;
  WGPUVertexBufferLayout vbLayout = {};
  vbLayout.arrayStride = 32;
  vbLayout.stepMode = WGPUVertexStepMode_Vertex;
  vbLayout.attributeCount = 3;
  vbLayout.attributes = attrs;

  WGPUColorTargetState colorTarget = {};
  colorTarget.format = WGPUTextureFormat_RGBA8Unorm;
  colorTarget.writeMask = WGPUColorWriteMask_All;

  WGPUFragmentState fs = {};
  fs.module = sm;
  fs.entryPoint.data = "fs_main";
  fs.entryPoint.length = WGPU_STRLEN;
  fs.targetCount = 1;
  fs.targets = &colorTarget;

  WGPUDepthStencilState depthState = {};
  depthState.format = WGPUTextureFormat_Depth24Plus;
  depthState.depthWriteEnabled = WGPUOptionalBool_True;
  depthState.depthCompare = WGPUCompareFunction_Less;
  depthState.stencilFront.compare = WGPUCompareFunction_Always;
  depthState.stencilFront.failOp = WGPUStencilOperation_Keep;
  depthState.stencilFront.depthFailOp = WGPUStencilOperation_Keep;
  depthState.stencilFront.passOp = WGPUStencilOperation_Keep;
  depthState.stencilBack = depthState.stencilFront;
  depthState.stencilReadMask = 0xFFFFFFFFu;
  depthState.stencilWriteMask = 0xFFFFFFFFu;

  WGPURenderPipelineDescriptor pipeDesc = {};
  pipeDesc.layout = plLayout;
  pipeDesc.vertex.module = sm;
  pipeDesc.vertex.entryPoint.data = "vs_main";
  pipeDesc.vertex.entryPoint.length = WGPU_STRLEN;
  pipeDesc.vertex.bufferCount = 1;
  pipeDesc.vertex.buffers = &vbLayout;
  pipeDesc.primitive.topology = WGPUPrimitiveTopology_TriangleList;
  pipeDesc.primitive.frontFace = WGPUFrontFace_CCW;
  pipeDesc.primitive.cullMode = WGPUCullMode_None;
  pipeDesc.depthStencil = &depthState;
  pipeDesc.multisample.count = 1;
  pipeDesc.multisample.mask = 0xFFFFFFFFu;
  pipeDesc.fragment = &fs;
  WGPURenderPipeline pipe = wgpuDeviceCreateRenderPipeline(device, &pipeDesc);
  wgpuPipelineLayoutRelease(plLayout);
  if (!pipe) {
    WbLog::info("[WbWgpuRenderTarget] csm CreateRenderPipeline failed");
    return false;
  }
  mCsmPipeline = pipe;
  return true;
#  else
  return false;
#  endif
#else
  return false;
#endif
}

// T1.2 CSM render: `cascadeCount` light clip-depth passes (one per array layer, each
// with that cascade's light VP) then ONE lit pass with kSolidLitCsm sampling the array.
// See the header. Self-contained; reads back the lit RGBA8 into `rgba8`.
bool WbWgpuRenderTarget::clearAndDrawSceneCsm(const WbWgpuClearColor &color, const float *viewProj16,
                                              const float *cascadeLightViewProjs,
                                              const float *cascadeSplitsFar4, uint32_t cascadeCount,
                                              const float *lightDirAmbient4,
                                              const WbWgpuSolidDraw *draws, uint32_t numDraws,
                                              float shadowStrength, float depthBias, void *rgba8) {
  if (!mUsable || !rgba8 || !viewProj16 || !cascadeLightViewProjs || !cascadeSplitsFar4 ||
      !lightDirAmbient4)
    return false;
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (cascadeCount < 1)
    cascadeCount = 1;
  if (cascadeCount > kCsmMaxCascades)
    cascadeCount = kCsmMaxCascades;
  if (!ensureSceneClipDepthF32Pipeline())  // light clip-depth pass pipeline (mScnClipDepthPipeline)
    return false;
  if (!ensureSceneCsmPipeline())           // lit pass + 3-entry layout + sampler
    return false;
  const uint32_t kShadowRes = 1024;
  if (!ensureShadowMapArray(kShadowRes, cascadeCount))  // N-layer sampleable shadow array
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUQueue queue = static_cast<WGPUQueue>(mBackend->queue());

  const uint32_t slotCount = numDraws == 0 ? 1u : numDraws;

  // --- Light-depth uniforms: one slot per (cascade, draw): {lightViewProj[c], model}
  // in the clip-depth Scene layout (256 B stride; the shader reads the first two mat4s). ---
  const size_t depthNeeded = static_cast<size_t>(cascadeCount) * slotCount * kScnUniformStride;
  if (depthNeeded > mCsmDepthUniformBufferSize) {
    if (mCsmDepthUniformBuffer)
      wgpuBufferRelease(static_cast<WGPUBuffer>(mCsmDepthUniformBuffer));
    WGPUBufferDescriptor ubDesc = {};
    ubDesc.size = depthNeeded;
    ubDesc.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
    mCsmDepthUniformBuffer = wgpuDeviceCreateBuffer(device, &ubDesc);
    if (!mCsmDepthUniformBuffer)
      return false;
    mCsmDepthUniformBufferSize = depthNeeded;
  }
  std::vector<uint8_t> pdepth(depthNeeded, 0);
  for (uint32_t c = 0; c < cascadeCount; ++c) {
    const float *lvp = cascadeLightViewProjs + static_cast<size_t>(c) * 16;
    for (uint32_t i = 0; i < numDraws; ++i) {
      uint8_t *slot = pdepth.data() + (static_cast<size_t>(c) * slotCount + i) * kScnUniformStride;
      std::memcpy(slot + 0, lvp, 64);
      if (draws[i].modelMatrix16)
        std::memcpy(slot + 64, draws[i].modelMatrix16, 64);
    }
  }
  wgpuQueueWriteBuffer(queue, static_cast<WGPUBuffer>(mCsmDepthUniformBuffer), 0, pdepth.data(),
                       depthNeeded);

  WGPUBindGroupEntry de = {};
  de.binding = 0;
  de.buffer = static_cast<WGPUBuffer>(mCsmDepthUniformBuffer);
  de.offset = 0;
  de.size = 192;
  WGPUBindGroupDescriptor dd = {};
  dd.layout = static_cast<WGPUBindGroupLayout>(mScnBindGroupLayout);
  dd.entryCount = 1;
  dd.entries = &de;
  WGPUBindGroup dbg = wgpuDeviceCreateBindGroup(device, &dd);
  if (!dbg)
    return false;

  // --- Lit-pass CsmScene uniform: 448 B payload, 512 B stride per draw (256-aligned offset). ---
  const size_t kCsmStride = 512;
  const size_t litNeeded = static_cast<size_t>(slotCount) * kCsmStride;
  if (litNeeded > mCsmUniformBufferSize) {
    if (mCsmUniformBuffer)
      wgpuBufferRelease(static_cast<WGPUBuffer>(mCsmUniformBuffer));
    WGPUBufferDescriptor ubDesc = {};
    ubDesc.size = litNeeded;
    ubDesc.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
    mCsmUniformBuffer = wgpuDeviceCreateBuffer(device, &ubDesc);
    if (!mCsmUniformBuffer) {
      wgpuBindGroupRelease(dbg);
      return false;
    }
    mCsmUniformBufferSize = litNeeded;
  }
  static const float kIdent[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  const float splits4[4] = {cascadeSplitsFar4[0], cascadeSplitsFar4[1], cascadeSplitsFar4[2],
                            cascadeSplitsFar4[3]};
  std::vector<uint8_t> plit(litNeeded, 0);
  for (uint32_t i = 0; i < numDraws; ++i) {
    uint8_t *slot = plit.data() + static_cast<size_t>(i) * kCsmStride;
    std::memcpy(slot + 0, viewProj16, 64);  // viewProj @0
    if (draws[i].modelMatrix16)
      std::memcpy(slot + 64, draws[i].modelMatrix16, 64);  // model @64
    for (uint32_t c = 0; c < 4; ++c) {       // lightViewProj[0..3] @128 (identity-fill the tail)
      const float *vpc =
        (c < cascadeCount) ? (cascadeLightViewProjs + static_cast<size_t>(c) * 16) : kIdent;
      std::memcpy(slot + 128 + static_cast<size_t>(c) * 64, vpc, 64);
    }
    const float baseColor[4] = {draws[i].baseColorR, draws[i].baseColorG, draws[i].baseColorB,
                                draws[i].baseColorA};
    std::memcpy(slot + 384, baseColor, 16);        // baseColor @384
    std::memcpy(slot + 400, lightDirAmbient4, 16);  // light (dir + ambient) @400
    std::memcpy(slot + 416, splits4, 16);          // cascadeSplits (far view-depths) @416
    const float shadowParams[4] = {shadowStrength, depthBias, static_cast<float>(cascadeCount), 0.0f};
    std::memcpy(slot + 432, shadowParams, 16);     // shadowParams @432
  }
  wgpuQueueWriteBuffer(queue, static_cast<WGPUBuffer>(mCsmUniformBuffer), 0, plit.data(), litNeeded);

  WGPUBindGroupEntry le[3] = {};
  le[0].binding = 0;
  le[0].buffer = static_cast<WGPUBuffer>(mCsmUniformBuffer);
  le[0].offset = 0;
  le[0].size = 448;
  le[1].binding = 1;
  le[1].textureView = static_cast<WGPUTextureView>(mCsmShadowArrayView);
  le[2].binding = 2;
  le[2].sampler = static_cast<WGPUSampler>(mCsmSampler);
  WGPUBindGroupDescriptor ld = {};
  ld.layout = static_cast<WGPUBindGroupLayout>(mCsmBindGroupLayout);
  ld.entryCount = 3;
  ld.entries = le;
  WGPUBindGroup lbg = wgpuDeviceCreateBindGroup(device, &ld);
  if (!lbg) {
    wgpuBindGroupRelease(dbg);
    return false;
  }

  WGPUCommandEncoderDescriptor encDesc = {};
  WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(device, &encDesc);
  if (!encoder) {
    wgpuBindGroupRelease(dbg);
    wgpuBindGroupRelease(lbg);
    return false;
  }

  // ===== N LIGHT-DEPTH PASSES: clip.z into each array layer =====
  for (uint32_t c = 0; c < cascadeCount; ++c) {
    WGPURenderPassColorAttachment ca = {};
    ca.view = static_cast<WGPUTextureView>(mCsmShadowLayerViews[c]);
    ca.loadOp = WGPULoadOp_Clear;
    ca.storeOp = WGPUStoreOp_Store;
    ca.clearValue = {1.0, 0.0, 0.0, 1.0};  // far = 1.0 (nothing occludes)
    ca.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
    WGPURenderPassDepthStencilAttachment da = {};
    da.view = static_cast<WGPUTextureView>(mCsmShadowDepthView);
    da.depthLoadOp = WGPULoadOp_Clear;
    da.depthStoreOp = WGPUStoreOp_Store;
    da.depthClearValue = 1.0f;
    da.depthReadOnly = 0;
    WGPURenderPassDescriptor pd = {};
    pd.colorAttachmentCount = 1;
    pd.colorAttachments = &ca;
    pd.depthStencilAttachment = &da;
    WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &pd);
    if (!pass) {
      wgpuCommandEncoderRelease(encoder);
      wgpuBindGroupRelease(dbg);
      wgpuBindGroupRelease(lbg);
      return false;
    }
    wgpuRenderPassEncoderSetPipeline(pass, static_cast<WGPURenderPipeline>(mScnClipDepthPipeline));
    for (uint32_t i = 0; i < numDraws; ++i) {
      const WbWgpuSolidDraw &d = draws[i];
      if (!d.vertexBuffer || !d.indexBuffer || d.indexCount == 0)
        continue;
      const uint32_t dyn =
        static_cast<uint32_t>((static_cast<size_t>(c) * slotCount + i) * kScnUniformStride);
      wgpuRenderPassEncoderSetBindGroup(pass, 0, dbg, 1, &dyn);
      wgpuRenderPassEncoderSetVertexBuffer(pass, 0, static_cast<WGPUBuffer>(d.vertexBuffer), 0,
                                           WGPU_WHOLE_SIZE);
      wgpuRenderPassEncoderSetIndexBuffer(pass, static_cast<WGPUBuffer>(d.indexBuffer),
                                          WGPUIndexFormat_Uint32, 0, WGPU_WHOLE_SIZE);
      wgpuRenderPassEncoderDrawIndexed(pass, d.indexCount, 1, 0, 0, 0);
    }
    wgpuRenderPassEncoderEnd(pass);
    wgpuRenderPassEncoderRelease(pass);
  }

  // ===== LIT + SHADOW PASS: kSolidLitCsm sampling the array =====
  {
    WGPURenderPassColorAttachment ca = {};
    ca.view = static_cast<WGPUTextureView>(mView);
    ca.loadOp = WGPULoadOp_Clear;
    ca.storeOp = WGPUStoreOp_Store;
    ca.clearValue = {color.r, color.g, color.b, color.a};
    ca.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
    WGPURenderPassDepthStencilAttachment da = {};
    da.view = static_cast<WGPUTextureView>(mScnDepthView);
    da.depthLoadOp = WGPULoadOp_Clear;
    da.depthStoreOp = WGPUStoreOp_Store;
    da.depthClearValue = 1.0f;
    da.depthReadOnly = 0;
    WGPURenderPassDescriptor pd = {};
    pd.colorAttachmentCount = 1;
    pd.colorAttachments = &ca;
    pd.depthStencilAttachment = &da;
    WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &pd);
    if (!pass) {
      wgpuCommandEncoderRelease(encoder);
      wgpuBindGroupRelease(dbg);
      wgpuBindGroupRelease(lbg);
      return false;
    }
    wgpuRenderPassEncoderSetPipeline(pass, static_cast<WGPURenderPipeline>(mCsmPipeline));
    for (uint32_t i = 0; i < numDraws; ++i) {
      const WbWgpuSolidDraw &d = draws[i];
      if (!d.vertexBuffer || !d.indexBuffer || d.indexCount == 0)
        continue;
      const uint32_t dyn = static_cast<uint32_t>(static_cast<size_t>(i) * kCsmStride);
      wgpuRenderPassEncoderSetBindGroup(pass, 0, lbg, 1, &dyn);
      wgpuRenderPassEncoderSetVertexBuffer(pass, 0, static_cast<WGPUBuffer>(d.vertexBuffer), 0,
                                           WGPU_WHOLE_SIZE);
      wgpuRenderPassEncoderSetIndexBuffer(pass, static_cast<WGPUBuffer>(d.indexBuffer),
                                          WGPUIndexFormat_Uint32, 0, WGPU_WHOLE_SIZE);
      wgpuRenderPassEncoderDrawIndexed(pass, d.indexCount, 1, 0, 0, 0);
    }
    wgpuRenderPassEncoderEnd(pass);
    wgpuRenderPassEncoderRelease(pass);
  }

  const uint32_t stride = alignedBytesPerRow(mWidth);
  WGPUTexelCopyTextureInfo src = {};
  src.texture = static_cast<WGPUTexture>(mTexture);
  src.aspect = WGPUTextureAspect_All;
  WGPUTexelCopyBufferInfo dst = {};
  dst.buffer = static_cast<WGPUBuffer>(mReadbackBuffer);
  dst.layout.bytesPerRow = stride;
  dst.layout.rowsPerImage = mHeight;
  WGPUExtent3D extent = {mWidth, mHeight, 1};
  wgpuCommandEncoderCopyTextureToBuffer(encoder, &src, &dst, &extent);

  WGPUCommandBufferDescriptor cmdDesc = {};
  WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(encoder, &cmdDesc);
  wgpuCommandEncoderRelease(encoder);
  if (!cmd) {
    wgpuBindGroupRelease(dbg);
    wgpuBindGroupRelease(lbg);
    return false;
  }
  wgpuQueueSubmit(queue, 1, &cmd);
  wgpuCommandBufferRelease(cmd);

  MapCapture cap;
  WGPUBufferMapCallbackInfo mapCb = {};
  mapCb.mode = WGPUCallbackMode_AllowSpontaneous;
  mapCb.callback = onMap;
  mapCb.userdata1 = &cap;
  (void)wgpuBufferMapAsync(static_cast<WGPUBuffer>(mReadbackBuffer), WGPUMapMode_Read, 0,
                           mReadbackBufferSize, mapCb);
  for (int i = 0; i < 1000 && !cap.done; ++i)
    wgpuDevicePoll(device, true, nullptr);
  wgpuBindGroupRelease(dbg);
  wgpuBindGroupRelease(lbg);
  if (!cap.done || !cap.ok)
    return false;

  const void *mapped =
    wgpuBufferGetConstMappedRange(static_cast<WGPUBuffer>(mReadbackBuffer), 0, mReadbackBufferSize);
  if (!mapped) {
    wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
    return false;
  }
  const uint32_t rowBytes = mWidth * 4u;
  uint8_t *out = static_cast<uint8_t *>(rgba8);
  const uint8_t *in = static_cast<const uint8_t *>(mapped);
  for (uint32_t y = 0; y < mHeight; ++y)
    std::memcpy(out + y * rowBytes, in + y * stride, rowBytes);
  wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
  return true;
#  else
  (void)color; (void)viewProj16; (void)cascadeLightViewProjs; (void)cascadeSplitsFar4;
  (void)cascadeCount; (void)lightDirAmbient4; (void)draws; (void)numDraws;
  (void)shadowStrength; (void)depthBias; (void)rgba8;
  return false;
#  endif
#else
  (void)color; (void)viewProj16; (void)cascadeLightViewProjs; (void)cascadeSplitsFar4;
  (void)cascadeCount; (void)lightDirAmbient4; (void)draws; (void)numDraws;
  (void)shadowStrength; (void)depthBias; (void)rgba8;
  return false;
#endif
}

// T1.2 CSM render-layer self-test: build the floor+caster geometry, render it through
// clearAndDrawSceneCsm at strength 0 then 0.8, project the two probe points via the passed
// camViewProj16, and read back the floor pixel under the caster (shadowed), a floor pixel to the
// side (lit), and the strength-0 reference. The camera/light/cascade matrices are supplied by the
// caller (WbWgpuSceneRenderer::csmSelfTest, the nodes layer that owns WbMatrix4 + the cascade fit),
// so this stays render-self-contained (no up-dependency on maths/ or nodes/).
bool WbWgpuRenderTarget::selfTestCsm(const float *camViewProj16, const float *cascadeLightViewProjs,
                                     const float *cascadeSplitsFar4, uint32_t cascadeCount,
                                     const float *lightDirAmbient4, unsigned char shadowedOut[3],
                                     unsigned char litSideOut[3], unsigned char shadowOffOut[3],
                                     int *cascadeSelected) {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (!mUsable || !camViewProj16 || !cascadeLightViewProjs || !cascadeSplitsFar4 ||
      !lightDirAmbient4)
    return false;
  if (!ensureSceneCsmPipeline())  // pipeline build naga-validates kSolidLitCsm in-engine
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUQueue queue = static_cast<WGPUQueue>(mBackend->queue());

  // Scene: a large floor (z=0) + an elevated caster (z=ZCAST), each a quad of 6
  // pos3/norm3(+Z)/uv2 verts in WORLD space → identity model (mirrors the prototype).
  const float ZCAST = 3.0f;
  auto quad = [](float cx, float cy, float z, float hx, float hy, float *out /*48 floats*/) {
    const float px[6][3] = {{cx - hx, cy - hy, z}, {cx + hx, cy - hy, z}, {cx + hx, cy + hy, z},
                            {cx - hx, cy - hy, z}, {cx + hx, cy + hy, z}, {cx - hx, cy + hy, z}};
    const float uv[6][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 0}, {1, 1}, {0, 1}};
    for (int i = 0; i < 6; ++i) {
      float *v = out + i * 8;
      v[0] = px[i][0]; v[1] = px[i][1]; v[2] = px[i][2];
      v[3] = 0; v[4] = 0; v[5] = 1;
      v[6] = uv[i][0]; v[7] = uv[i][1];
    }
  };
  float floorV[48], casterV[48];
  quad(0, 0, 0, 30, 30, floorV);
  quad(0, 0, ZCAST, 2, 2, casterV);
  const uint32_t idx[6] = {0, 1, 2, 3, 4, 5};

  auto mkbuf = [&](const void *data, size_t sz, WGPUBufferUsage usage) -> WGPUBuffer {
    WGPUBufferDescriptor bd = {};
    bd.usage = usage | WGPUBufferUsage_CopyDst;
    bd.size = sz;
    WGPUBuffer b = wgpuDeviceCreateBuffer(device, &bd);
    if (b)
      wgpuQueueWriteBuffer(queue, b, 0, data, sz);
    return b;
  };
  WGPUBuffer fvb = mkbuf(floorV, sizeof(floorV), WGPUBufferUsage_Vertex);
  WGPUBuffer cvb = mkbuf(casterV, sizeof(casterV), WGPUBufferUsage_Vertex);
  WGPUBuffer ibuf = mkbuf(idx, sizeof(idx), WGPUBufferUsage_Index);
  if (!fvb || !cvb || !ibuf) {
    if (fvb) wgpuBufferRelease(fvb);
    if (cvb) wgpuBufferRelease(cvb);
    if (ibuf) wgpuBufferRelease(ibuf);
    return false;
  }

  WbWgpuClearColor black;
  black.r = black.g = black.b = 0.0f;
  black.a = 1.0f;

  const float ident[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  WbWgpuSolidDraw fd{};
  fd.modelMatrix16 = ident;
  fd.baseColorR = fd.baseColorG = fd.baseColorB = 0.8f;
  fd.baseColorA = 1.0f;
  fd.vertexBuffer = fvb;
  fd.indexBuffer = ibuf;
  fd.indexCount = 6;
  WbWgpuSolidDraw cd = fd;
  cd.vertexBuffer = cvb;
  WbWgpuSolidDraw draws[2] = {fd, cd};

  std::vector<unsigned char> bufA(static_cast<size_t>(mWidth) * mHeight * 4, 0);
  std::vector<unsigned char> bufB(static_cast<size_t>(mWidth) * mHeight * 4, 0);
  bool ok = clearAndDrawSceneCsm(black, camViewProj16, cascadeLightViewProjs, cascadeSplitsFar4,
                                 cascadeCount, lightDirAmbient4, draws, 2, 0.0f, 2e-3f, bufA.data());
  ok = ok && clearAndDrawSceneCsm(black, camViewProj16, cascadeLightViewProjs, cascadeSplitsFar4,
                                  cascadeCount, lightDirAmbient4, draws, 2, 0.8f, 2e-3f, bufB.data());

  if (ok) {
    auto project = [&](double wx, double wy, double wz, int &ox, int &oy) {
      const double cx =
        camViewProj16[0] * wx + camViewProj16[4] * wy + camViewProj16[8] * wz + camViewProj16[12];
      const double cy =
        camViewProj16[1] * wx + camViewProj16[5] * wy + camViewProj16[9] * wz + camViewProj16[13];
      const double cw =
        camViewProj16[3] * wx + camViewProj16[7] * wy + camViewProj16[11] * wz + camViewProj16[15];
      const double w = (cw != 0.0) ? cw : 1.0;
      ox = static_cast<int>((cx / w * 0.5 + 0.5) * mWidth);
      oy = static_cast<int>((0.5 - cy / w * 0.5) * mHeight);
    };
    auto clampi = [](int v, int hi) { return v < 0 ? 0 : (v > hi ? hi : v); };
    int sx, sy, lxp, lyp;
    project(0, 0, 0, sx, sy);    // floor directly under the caster → shadowed
    project(10, 0, 0, lxp, lyp);  // floor to the side, same plane → stays lit
    sx = clampi(sx, mWidth - 1); sy = clampi(sy, mHeight - 1);
    lxp = clampi(lxp, mWidth - 1); lyp = clampi(lyp, mHeight - 1);
    const size_t si = (static_cast<size_t>(sy) * mWidth + sx) * 4;
    const size_t li = (static_cast<size_t>(lyp) * mWidth + lxp) * 4;
    shadowOffOut[0] = bufA[si]; shadowOffOut[1] = bufA[si + 1]; shadowOffOut[2] = bufA[si + 2];
    shadowedOut[0] = bufB[si];  shadowedOut[1] = bufB[si + 1];  shadowedOut[2] = bufB[si + 2];
    litSideOut[0] = bufB[li];   litSideOut[1] = bufB[li + 1];   litSideOut[2] = bufB[li + 2];
    if (cascadeSelected) {
      const double cw = camViewProj16[15];  // clip.w at world (0,0,0)
      int sel = 0;
      if (cw > cascadeSplitsFar4[0]) sel = 1;
      if (cw > cascadeSplitsFar4[1]) sel = 2;
      if (cw > cascadeSplitsFar4[2]) sel = 3;
      const int maxc = (cascadeCount >= 1) ? static_cast<int>(cascadeCount) - 1 : 0;
      *cascadeSelected = sel < maxc ? sel : maxc;
    }
  }

  wgpuBufferRelease(fvb);
  wgpuBufferRelease(cvb);
  wgpuBufferRelease(ibuf);
  return ok;
#  else
  (void)camViewProj16; (void)cascadeLightViewProjs; (void)cascadeSplitsFar4; (void)cascadeCount;
  (void)lightDirAmbient4; (void)shadowedOut; (void)litSideOut; (void)shadowOffOut;
  (void)cascadeSelected;
  return false;
#  endif
#else
  (void)camViewProj16; (void)cascadeLightViewProjs; (void)cascadeSplitsFar4; (void)cascadeCount;
  (void)lightDirAmbient4; (void)shadowedOut; (void)litSideOut; (void)shadowOffOut;
  (void)cascadeSelected;
  return false;
#endif
}

// ===========================================================================
// T1.4 TAA — the temporal-resolve post pass (kTaaResolve) + a headless self-test.
// A fullscreen image op (current ⊕ reprojected history, neighborhood-clamped), the
// engine port of docs/developer/taa-preview.html. Pure render-layer (texture in /
// texture out — no scene, no matrices), so it needs no maths/nodes up-dependency.
// ===========================================================================

// Build the TAA temporal-resolve pipeline (kTaaResolve) + its 4-entry layout (TaaParams
// uniform @0 + curTex @1 + histTex @2 + filtering sampler @3). Fullscreen, no vertex buffer,
// no depth. Building it naga-VALIDATES kTaaResolve in-engine. Mirrors ensureTexturedQuadPipeline.
bool WbWgpuRenderTarget::ensureTaaResolvePipeline() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (mTaaPipeline)
    return true;
  if (!mUsable)
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());

  WGPUShaderSourceWGSL wgsl = {};
  wgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
  wgsl.code.data = WbWgpuShaders::kTaaResolve;
  wgsl.code.length = WGPU_STRLEN;
  WGPUShaderModuleDescriptor smDesc = {};
  smDesc.nextInChain = &wgsl.chain;
  WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &smDesc);
  if (!sm) {
    WbLog::info("[WbWgpuRenderTarget] taa CreateShaderModule failed");
    return false;
  }
  mTaaShaderModule = sm;

  WGPUSamplerDescriptor sampDesc = {};
  sampDesc.addressModeU = WGPUAddressMode_ClampToEdge;
  sampDesc.addressModeV = WGPUAddressMode_ClampToEdge;
  sampDesc.addressModeW = WGPUAddressMode_ClampToEdge;
  sampDesc.magFilter = WGPUFilterMode_Linear;   // linear → sub-pixel history reprojection
  sampDesc.minFilter = WGPUFilterMode_Linear;
  sampDesc.mipmapFilter = WGPUMipmapFilterMode_Nearest;
  sampDesc.maxAnisotropy = 1;
  WGPUSampler samp = wgpuDeviceCreateSampler(device, &sampDesc);
  if (!samp) {
    WbLog::info("[WbWgpuRenderTarget] taa CreateSampler failed");
    return false;
  }
  mTaaSampler = samp;

  WGPUBindGroupLayoutEntry bglEntries[4] = {};
  bglEntries[0].binding = 0;
  bglEntries[0].visibility = WGPUShaderStage_Fragment;
  bglEntries[0].buffer.type = WGPUBufferBindingType_Uniform;
  bglEntries[0].buffer.minBindingSize = 32;  // TaaParams: 2 vec4
  bglEntries[1].binding = 1;
  bglEntries[1].visibility = WGPUShaderStage_Fragment;
  bglEntries[1].texture.sampleType = WGPUTextureSampleType_Float;
  bglEntries[1].texture.viewDimension = WGPUTextureViewDimension_2D;
  bglEntries[1].texture.multisampled = 0;
  bglEntries[2].binding = 2;
  bglEntries[2].visibility = WGPUShaderStage_Fragment;
  bglEntries[2].texture.sampleType = WGPUTextureSampleType_Float;
  bglEntries[2].texture.viewDimension = WGPUTextureViewDimension_2D;
  bglEntries[2].texture.multisampled = 0;
  bglEntries[3].binding = 3;
  bglEntries[3].visibility = WGPUShaderStage_Fragment;
  bglEntries[3].sampler.type = WGPUSamplerBindingType_Filtering;
  WGPUBindGroupLayoutDescriptor bglDesc = {};
  bglDesc.entryCount = 4;
  bglDesc.entries = bglEntries;
  WGPUBindGroupLayout bgl = wgpuDeviceCreateBindGroupLayout(device, &bglDesc);
  if (!bgl) {
    WbLog::info("[WbWgpuRenderTarget] taa CreateBindGroupLayout failed");
    return false;
  }
  mTaaBindGroupLayout = bgl;

  WGPUPipelineLayoutDescriptor plDesc = {};
  plDesc.bindGroupLayoutCount = 1;
  plDesc.bindGroupLayouts = &bgl;
  WGPUPipelineLayout plLayout = wgpuDeviceCreatePipelineLayout(device, &plDesc);
  if (!plLayout)
    return false;

  WGPUColorTargetState colorTarget = {};
  colorTarget.format = WGPUTextureFormat_RGBA8Unorm;
  colorTarget.writeMask = WGPUColorWriteMask_All;

  WGPUFragmentState fs = {};
  fs.module = sm;
  fs.entryPoint.data = "fs_main";
  fs.entryPoint.length = WGPU_STRLEN;
  fs.targetCount = 1;
  fs.targets = &colorTarget;

  WGPURenderPipelineDescriptor pipeDesc = {};
  pipeDesc.layout = plLayout;
  pipeDesc.vertex.module = sm;
  pipeDesc.vertex.entryPoint.data = "vs_main";
  pipeDesc.vertex.entryPoint.length = WGPU_STRLEN;
  pipeDesc.vertex.bufferCount = 0;  // fullscreen quad generated from vertex_index
  pipeDesc.primitive.topology = WGPUPrimitiveTopology_TriangleList;
  pipeDesc.primitive.frontFace = WGPUFrontFace_CCW;
  pipeDesc.primitive.cullMode = WGPUCullMode_None;
  pipeDesc.depthStencil = nullptr;
  pipeDesc.multisample.count = 1;
  pipeDesc.multisample.mask = 0xFFFFFFFFu;
  pipeDesc.fragment = &fs;
  WGPURenderPipeline pipe = wgpuDeviceCreateRenderPipeline(device, &pipeDesc);
  wgpuPipelineLayoutRelease(plLayout);
  if (!pipe) {
    WbLog::info("[WbWgpuRenderTarget] taa CreateRenderPipeline failed");
    return false;
  }
  mTaaPipeline = pipe;
  return true;
#  else
  return false;
#  endif
#else
  return false;
#endif
}

// T1.4 TAA temporal-resolve render. See the header. Mirrors drawTexturedInset's fullscreen
// no-vertex-buffer draw, but with two input textures + the TaaParams uniform, writing the
// resolved frame into the RGBA8 target and reading it back into rgba8.
bool WbWgpuRenderTarget::resolveTaa(void *curView, void *histView, const float motionPx2[2],
                                    float feedback, bool taaEnabled, bool clampEnabled, void *rgba8) {
  if (!mUsable || !rgba8 || !curView || !histView || !motionPx2)
    return false;
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (!ensureTaaResolvePipeline())
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUQueue queue = static_cast<WGPUQueue>(mBackend->queue());

  if (!mTaaUniformBuffer) {
    WGPUBufferDescriptor ud = {};
    ud.size = 32;
    ud.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
    mTaaUniformBuffer = wgpuDeviceCreateBuffer(device, &ud);
    if (!mTaaUniformBuffer)
      return false;
  }
  const float params[8] = {
    static_cast<float>(mWidth), static_cast<float>(mHeight), motionPx2[0],          motionPx2[1],
    feedback,                   taaEnabled ? 1.0f : 0.0f,    clampEnabled ? 1.0f : 0.0f, 0.0f,
  };
  wgpuQueueWriteBuffer(queue, static_cast<WGPUBuffer>(mTaaUniformBuffer), 0, params, sizeof(params));

  WGPUBindGroupEntry e[4] = {};
  e[0].binding = 0;
  e[0].buffer = static_cast<WGPUBuffer>(mTaaUniformBuffer);
  e[0].offset = 0;
  e[0].size = 32;
  e[1].binding = 1;
  e[1].textureView = static_cast<WGPUTextureView>(curView);
  e[2].binding = 2;
  e[2].textureView = static_cast<WGPUTextureView>(histView);
  e[3].binding = 3;
  e[3].sampler = static_cast<WGPUSampler>(mTaaSampler);
  WGPUBindGroupDescriptor bd = {};
  bd.layout = static_cast<WGPUBindGroupLayout>(mTaaBindGroupLayout);
  bd.entryCount = 4;
  bd.entries = e;
  WGPUBindGroup bg = wgpuDeviceCreateBindGroup(device, &bd);
  if (!bg)
    return false;

  WGPUCommandEncoderDescriptor encDesc = {};
  WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(device, &encDesc);
  if (!encoder) {
    wgpuBindGroupRelease(bg);
    return false;
  }
  WGPURenderPassColorAttachment ca = {};
  ca.view = static_cast<WGPUTextureView>(mView);
  ca.loadOp = WGPULoadOp_Clear;
  ca.storeOp = WGPUStoreOp_Store;
  ca.clearValue = {0.0, 0.0, 0.0, 1.0};
  ca.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
  WGPURenderPassDescriptor pd = {};
  pd.colorAttachmentCount = 1;
  pd.colorAttachments = &ca;
  WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &pd);
  if (!pass) {
    wgpuCommandEncoderRelease(encoder);
    wgpuBindGroupRelease(bg);
    return false;
  }
  wgpuRenderPassEncoderSetPipeline(pass, static_cast<WGPURenderPipeline>(mTaaPipeline));
  wgpuRenderPassEncoderSetBindGroup(pass, 0, bg, 0, nullptr);
  wgpuRenderPassEncoderDraw(pass, 6, 1, 0, 0);  // fullscreen quad
  wgpuRenderPassEncoderEnd(pass);
  wgpuRenderPassEncoderRelease(pass);

  const uint32_t stride = alignedBytesPerRow(mWidth);
  WGPUTexelCopyTextureInfo src = {};
  src.texture = static_cast<WGPUTexture>(mTexture);
  src.aspect = WGPUTextureAspect_All;
  WGPUTexelCopyBufferInfo dst = {};
  dst.buffer = static_cast<WGPUBuffer>(mReadbackBuffer);
  dst.layout.bytesPerRow = stride;
  dst.layout.rowsPerImage = mHeight;
  WGPUExtent3D extent = {mWidth, mHeight, 1};
  wgpuCommandEncoderCopyTextureToBuffer(encoder, &src, &dst, &extent);

  WGPUCommandBufferDescriptor cmdDesc = {};
  WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(encoder, &cmdDesc);
  wgpuCommandEncoderRelease(encoder);
  if (!cmd) {
    wgpuBindGroupRelease(bg);
    return false;
  }
  wgpuQueueSubmit(queue, 1, &cmd);
  wgpuCommandBufferRelease(cmd);
  wgpuBindGroupRelease(bg);

  MapCapture cap;
  WGPUBufferMapCallbackInfo mapCb = {};
  mapCb.mode = WGPUCallbackMode_AllowSpontaneous;
  mapCb.callback = onMap;
  mapCb.userdata1 = &cap;
  (void)wgpuBufferMapAsync(static_cast<WGPUBuffer>(mReadbackBuffer), WGPUMapMode_Read, 0,
                           mReadbackBufferSize, mapCb);
  for (int i = 0; i < 1000 && !cap.done; ++i)
    wgpuDevicePoll(device, true, nullptr);
  if (!cap.done || !cap.ok)
    return false;
  const void *mapped =
    wgpuBufferGetConstMappedRange(static_cast<WGPUBuffer>(mReadbackBuffer), 0, mReadbackBufferSize);
  if (!mapped) {
    wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
    return false;
  }
  const uint32_t rowBytes = mWidth * 4u;
  uint8_t *out = static_cast<uint8_t *>(rgba8);
  const uint8_t *in = static_cast<const uint8_t *>(mapped);
  for (uint32_t y = 0; y < mHeight; ++y)
    std::memcpy(out + y * rowBytes, in + y * stride, rowBytes);
  wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
  return true;
#  else
  (void)curView; (void)histView; (void)motionPx2; (void)feedback;
  (void)taaEnabled; (void)clampEnabled; (void)rgba8;
  return false;
#  endif
#else
  (void)curView; (void)histView; (void)motionPx2; (void)feedback;
  (void)taaEnabled; (void)clampEnabled; (void)rgba8;
  return false;
#endif
}

// T1.4 TAA headless self-test. Exercises each resolve branch with uniform 1x1 current/history
// textures: the feedback blend, the neighborhood-clamp ghost suppressor, the TAA-off passthrough,
// and off-screen history rejection. See the header.
bool WbWgpuRenderTarget::selfTestTaa(unsigned char blendOut[3], unsigned char clampOnOut[3],
                                     unsigned char clampOffOut[3], unsigned char taaOffOut[3],
                                     unsigned char offscreenOut[3]) {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (!mUsable || !ensureTaaResolvePipeline())  // pipeline build naga-validates kTaaResolve in-engine
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUQueue queue = static_cast<WGPUQueue>(mBackend->queue());

  auto make1x1 = [&](uint8_t r, uint8_t g, uint8_t b, WGPUTexture &tex, WGPUTextureView &view) -> bool {
    WGPUTextureDescriptor td = {};
    td.dimension = WGPUTextureDimension_2D;
    td.size = {1, 1, 1};
    td.format = WGPUTextureFormat_RGBA8Unorm;
    td.mipLevelCount = 1;
    td.sampleCount = 1;
    td.usage = WGPUTextureUsage_TextureBinding | WGPUTextureUsage_CopyDst;
    tex = wgpuDeviceCreateTexture(device, &td);
    if (!tex)
      return false;
    const uint8_t px[4] = {r, g, b, 255};
    WGPUTexelCopyTextureInfo dstT = {};
    dstT.texture = tex;
    dstT.aspect = WGPUTextureAspect_All;
    WGPUTexelCopyBufferLayout lay = {};
    lay.bytesPerRow = 4;
    lay.rowsPerImage = 1;
    WGPUExtent3D ext = {1, 1, 1};
    wgpuQueueWriteTexture(queue, &dstT, px, 4, &lay, &ext);
    WGPUTextureViewDescriptor vd = {};
    vd.format = WGPUTextureFormat_RGBA8Unorm;
    vd.dimension = WGPUTextureViewDimension_2D;
    vd.mipLevelCount = 1;
    vd.arrayLayerCount = 1;
    view = wgpuTextureCreateView(tex, &vd);
    return view != nullptr;
  };
  WGPUTexture whiteTex = nullptr, blackTex = nullptr;
  WGPUTextureView whiteView = nullptr, blackView = nullptr;
  bool ok = make1x1(255, 255, 255, whiteTex, whiteView) && make1x1(0, 0, 0, blackTex, blackView);

  if (ok) {
    std::vector<unsigned char> buf(static_cast<size_t>(mWidth) * mHeight * 4, 0);
    const size_t c = (static_cast<size_t>(mHeight / 2) * mWidth + mWidth / 2) * 4;
    const float noMotion[2] = {0.0f, 0.0f};
    const float bigMotion[2] = {1.0e5f, 0.0f};
    auto sample = [&](unsigned char o[3]) { o[0] = buf[c]; o[1] = buf[c + 1]; o[2] = buf[c + 2]; };
    // 1) Feedback blend: cur=white, hist=black, fb 0.9, clamp OFF → mix(1,0,0.9)=0.1 → ~26.
    ok = resolveTaa(whiteView, blackView, noMotion, 0.9f, true, false, buf.data());
    if (ok) sample(blendOut);
    // 2) Clamp ON: cur=black (uniform), hist=white → 3x3 AABB [0,0] clamps hist→0 → ~0 (ghost gone).
    std::memset(buf.data(), 0, buf.size());
    ok = ok && resolveTaa(blackView, whiteView, noMotion, 0.9f, true, true, buf.data());
    if (ok) sample(clampOnOut);
    // 3) Clamp OFF: same inputs → mix(0,1,0.9)=0.9 → ~230 (the ghost the clamp removes).
    std::memset(buf.data(), 0, buf.size());
    ok = ok && resolveTaa(blackView, whiteView, noMotion, 0.9f, true, false, buf.data());
    if (ok) sample(clampOffOut);
    // 4) TAA OFF: cur=white → passthrough → ~255.
    std::memset(buf.data(), 0, buf.size());
    ok = ok && resolveTaa(whiteView, blackView, noMotion, 0.9f, false, false, buf.data());
    if (ok) sample(taaOffOut);
    // 5) Off-screen history: huge motion → history rejected (fb=0) → cur (white) → ~255.
    std::memset(buf.data(), 0, buf.size());
    ok = ok && resolveTaa(whiteView, blackView, bigMotion, 0.9f, true, false, buf.data());
    if (ok) sample(offscreenOut);
  }

  if (whiteView) wgpuTextureViewRelease(whiteView);
  if (whiteTex) wgpuTextureRelease(whiteTex);
  if (blackView) wgpuTextureViewRelease(blackView);
  if (blackTex) wgpuTextureRelease(blackTex);
  return ok;
#  else
  (void)blendOut; (void)clampOnOut; (void)clampOffOut; (void)taaOffOut; (void)offscreenOut;
  return false;
#  endif
#else
  (void)blendOut; (void)clampOnOut; (void)clampOffOut; (void)taaOffOut; (void)offscreenOut;
  return false;
#endif
}

// T1.4 TAA: lazily (re)build the two RGBA8 ping-pong history buffers (RenderAttachment so the
// resolve renders into them + TextureBinding so the next frame samples them + CopySrc for readback),
// sized to the target. Rebuilds on resize; resets the ping on (re)build.
bool WbWgpuRenderTarget::ensureTaaHistory() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (!mUsable)
    return false;
  if (mTaaHistoryTexture[0] && mTaaHistoryW == mWidth && mTaaHistoryH == mHeight)
    return true;
  for (int i = 0; i < 2; ++i) {
    if (mTaaHistoryView[i]) {
      wgpuTextureViewRelease(static_cast<WGPUTextureView>(mTaaHistoryView[i]));
      mTaaHistoryView[i] = nullptr;
    }
    if (mTaaHistoryTexture[i]) {
      wgpuTextureRelease(static_cast<WGPUTexture>(mTaaHistoryTexture[i]));
      mTaaHistoryTexture[i] = nullptr;
    }
  }
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  for (int i = 0; i < 2; ++i) {
    WGPUTextureDescriptor td = {};
    td.usage = WGPUTextureUsage_RenderAttachment | WGPUTextureUsage_TextureBinding |
               WGPUTextureUsage_CopySrc;
    td.dimension = WGPUTextureDimension_2D;
    td.size = {mWidth, mHeight, 1};
    td.format = WGPUTextureFormat_RGBA8Unorm;
    td.mipLevelCount = 1;
    td.sampleCount = 1;
    WGPUTexture tex = wgpuDeviceCreateTexture(device, &td);
    if (!tex) {
      WbLog::info("[WbWgpuRenderTarget] taa-history CreateTexture failed");
      return false;
    }
    mTaaHistoryTexture[i] = tex;
    WGPUTextureView v = wgpuTextureCreateView(tex, nullptr);
    if (!v) {
      WbLog::info("[WbWgpuRenderTarget] taa-history view failed");
      return false;
    }
    mTaaHistoryView[i] = v;
  }
  mTaaHistoryPing = 0;
  mTaaHistoryW = mWidth;
  mTaaHistoryH = mHeight;
  return true;
#  else
  return false;
#  endif
#else
  return false;
#endif
}

// T1.4 TAA ping-pong accumulator. See the header. Resolves curView against history[src] into
// history[dst], swaps, and optionally reads history[dst] back — so repeated calls build the
// temporal EMA. Self-contained (mirrors resolveTaa, but targets a history buffer not mView).
bool WbWgpuRenderTarget::accumulateTaa(void *curView, bool reset, float feedback,
                                       const float motionPx2[2], bool clampEnabled,
                                       void *rgba8OrNull) {
  if (!mUsable || !curView || !motionPx2)
    return false;
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (!ensureTaaResolvePipeline() || !ensureTaaHistory())
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUQueue queue = static_cast<WGPUQueue>(mBackend->queue());

  const uint32_t dst = mTaaHistoryPing & 1u;
  const uint32_t srcIdx = dst ^ 1u;

  if (!mTaaUniformBuffer) {
    WGPUBufferDescriptor ud = {};
    ud.size = 32;
    ud.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
    mTaaUniformBuffer = wgpuDeviceCreateBuffer(device, &ud);
    if (!mTaaUniformBuffer)
      return false;
  }
  // reset → taa-off (ctrl.y=0) so the shader outputs cur, seeding history[dst]=cur.
  const float params[8] = {
    static_cast<float>(mWidth), static_cast<float>(mHeight), motionPx2[0],          motionPx2[1],
    feedback,                   reset ? 0.0f : 1.0f,         clampEnabled ? 1.0f : 0.0f, 0.0f,
  };
  wgpuQueueWriteBuffer(queue, static_cast<WGPUBuffer>(mTaaUniformBuffer), 0, params, sizeof(params));

  WGPUBindGroupEntry e[4] = {};
  e[0].binding = 0;
  e[0].buffer = static_cast<WGPUBuffer>(mTaaUniformBuffer);
  e[0].offset = 0;
  e[0].size = 32;
  e[1].binding = 1;
  e[1].textureView = static_cast<WGPUTextureView>(curView);
  e[2].binding = 2;
  e[2].textureView = static_cast<WGPUTextureView>(mTaaHistoryView[srcIdx]);
  e[3].binding = 3;
  e[3].sampler = static_cast<WGPUSampler>(mTaaSampler);
  WGPUBindGroupDescriptor bd = {};
  bd.layout = static_cast<WGPUBindGroupLayout>(mTaaBindGroupLayout);
  bd.entryCount = 4;
  bd.entries = e;
  WGPUBindGroup bg = wgpuDeviceCreateBindGroup(device, &bd);
  if (!bg)
    return false;

  WGPUCommandEncoderDescriptor encDesc = {};
  WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(device, &encDesc);
  if (!encoder) {
    wgpuBindGroupRelease(bg);
    return false;
  }
  WGPURenderPassColorAttachment ca = {};
  ca.view = static_cast<WGPUTextureView>(mTaaHistoryView[dst]);
  ca.loadOp = WGPULoadOp_Clear;
  ca.storeOp = WGPUStoreOp_Store;
  ca.clearValue = {0.0, 0.0, 0.0, 1.0};
  ca.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
  WGPURenderPassDescriptor pd = {};
  pd.colorAttachmentCount = 1;
  pd.colorAttachments = &ca;
  WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &pd);
  if (!pass) {
    wgpuCommandEncoderRelease(encoder);
    wgpuBindGroupRelease(bg);
    return false;
  }
  wgpuRenderPassEncoderSetPipeline(pass, static_cast<WGPURenderPipeline>(mTaaPipeline));
  wgpuRenderPassEncoderSetBindGroup(pass, 0, bg, 0, nullptr);
  wgpuRenderPassEncoderDraw(pass, 6, 1, 0, 0);
  wgpuRenderPassEncoderEnd(pass);
  wgpuRenderPassEncoderRelease(pass);

  if (rgba8OrNull) {  // copy the just-written history buffer for readback
    const uint32_t stride = alignedBytesPerRow(mWidth);
    WGPUTexelCopyTextureInfo src = {};
    src.texture = static_cast<WGPUTexture>(mTaaHistoryTexture[dst]);
    src.aspect = WGPUTextureAspect_All;
    WGPUTexelCopyBufferInfo cdst = {};
    cdst.buffer = static_cast<WGPUBuffer>(mReadbackBuffer);
    cdst.layout.bytesPerRow = stride;
    cdst.layout.rowsPerImage = mHeight;
    WGPUExtent3D extent = {mWidth, mHeight, 1};
    wgpuCommandEncoderCopyTextureToBuffer(encoder, &src, &cdst, &extent);
  }

  WGPUCommandBufferDescriptor cmdDesc = {};
  WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(encoder, &cmdDesc);
  wgpuCommandEncoderRelease(encoder);
  if (!cmd) {
    wgpuBindGroupRelease(bg);
    return false;
  }
  wgpuQueueSubmit(queue, 1, &cmd);
  wgpuCommandBufferRelease(cmd);
  wgpuBindGroupRelease(bg);

  mTaaHistoryPing = srcIdx;  // next call writes the other buffer + samples this one

  if (!rgba8OrNull)
    return true;

  MapCapture cap;
  WGPUBufferMapCallbackInfo mapCb = {};
  mapCb.mode = WGPUCallbackMode_AllowSpontaneous;
  mapCb.callback = onMap;
  mapCb.userdata1 = &cap;
  (void)wgpuBufferMapAsync(static_cast<WGPUBuffer>(mReadbackBuffer), WGPUMapMode_Read, 0,
                           mReadbackBufferSize, mapCb);
  for (int i = 0; i < 1000 && !cap.done; ++i)
    wgpuDevicePoll(device, true, nullptr);
  if (!cap.done || !cap.ok)
    return false;
  const void *mapped =
    wgpuBufferGetConstMappedRange(static_cast<WGPUBuffer>(mReadbackBuffer), 0, mReadbackBufferSize);
  if (!mapped) {
    wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
    return false;
  }
  const uint32_t rowBytes = mWidth * 4u;
  const uint32_t stride2 = alignedBytesPerRow(mWidth);
  uint8_t *out = static_cast<uint8_t *>(rgba8OrNull);
  const uint8_t *in = static_cast<const uint8_t *>(mapped);
  for (uint32_t y = 0; y < mHeight; ++y)
    std::memcpy(out + y * rowBytes, in + y * stride2, rowBytes);
  wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
  return true;
#  else
  (void)curView; (void)reset; (void)feedback; (void)motionPx2; (void)clampEnabled; (void)rgba8OrNull;
  return false;
#  endif
#else
  (void)curView; (void)reset; (void)feedback; (void)motionPx2; (void)clampEnabled; (void)rgba8OrNull;
  return false;
#endif
}

// T1.4 TAA history convergence self-test. See the header. Seeds history black, accumulates a
// white frame repeatedly (fb 0.9, no motion) → the EMA must rise monotonically toward white.
bool WbWgpuRenderTarget::selfTestTaaAccum(unsigned char afterResetOut[3], unsigned char afterFewOut[3],
                                          unsigned char afterManyOut[3]) {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (!mUsable || !ensureTaaResolvePipeline() || !ensureTaaHistory())
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUQueue queue = static_cast<WGPUQueue>(mBackend->queue());

  auto make1x1 = [&](uint8_t r, uint8_t g, uint8_t b, WGPUTexture &tex, WGPUTextureView &view) -> bool {
    WGPUTextureDescriptor td = {};
    td.dimension = WGPUTextureDimension_2D;
    td.size = {1, 1, 1};
    td.format = WGPUTextureFormat_RGBA8Unorm;
    td.mipLevelCount = 1;
    td.sampleCount = 1;
    td.usage = WGPUTextureUsage_TextureBinding | WGPUTextureUsage_CopyDst;
    tex = wgpuDeviceCreateTexture(device, &td);
    if (!tex)
      return false;
    const uint8_t px[4] = {r, g, b, 255};
    WGPUTexelCopyTextureInfo dstT = {};
    dstT.texture = tex;
    dstT.aspect = WGPUTextureAspect_All;
    WGPUTexelCopyBufferLayout lay = {};
    lay.bytesPerRow = 4;
    lay.rowsPerImage = 1;
    WGPUExtent3D ext = {1, 1, 1};
    wgpuQueueWriteTexture(queue, &dstT, px, 4, &lay, &ext);
    WGPUTextureViewDescriptor vd = {};
    vd.format = WGPUTextureFormat_RGBA8Unorm;
    vd.dimension = WGPUTextureViewDimension_2D;
    vd.mipLevelCount = 1;
    vd.arrayLayerCount = 1;
    view = wgpuTextureCreateView(tex, &vd);
    return view != nullptr;
  };
  WGPUTexture whiteTex = nullptr, blackTex = nullptr;
  WGPUTextureView whiteView = nullptr, blackView = nullptr;
  bool ok = make1x1(255, 255, 255, whiteTex, whiteView) && make1x1(0, 0, 0, blackTex, blackView);

  if (ok) {
    std::vector<unsigned char> buf(static_cast<size_t>(mWidth) * mHeight * 4, 0);
    const size_t c = (static_cast<size_t>(mHeight / 2) * mWidth + mWidth / 2) * 4;
    const float noMotion[2] = {0.0f, 0.0f};
    auto sample = [&](unsigned char o[3]) { o[0] = buf[c]; o[1] = buf[c + 1]; o[2] = buf[c + 2]; };
    ok = accumulateTaa(blackView, true, 0.9f, noMotion, false, buf.data());  // seed history = black
    if (ok) sample(afterResetOut);                                           // ~0
    for (int i = 0; i < 3 && ok; ++i)
      ok = accumulateTaa(whiteView, false, 0.9f, noMotion, false, buf.data());
    if (ok) sample(afterFewOut);                                             // mid (rising)
    for (int i = 0; i < 40 && ok; ++i)
      ok = accumulateTaa(whiteView, false, 0.9f, noMotion, false, (i == 39) ? buf.data() : nullptr);
    if (ok) sample(afterManyOut);                                            // ~white (converged)
  }

  if (whiteView) wgpuTextureViewRelease(whiteView);
  if (whiteTex) wgpuTextureRelease(whiteTex);
  if (blackView) wgpuTextureViewRelease(blackView);
  if (blackTex) wgpuTextureRelease(blackTex);
  return ok;
#  else
  (void)afterResetOut; (void)afterFewOut; (void)afterManyOut;
  return false;
#  endif
#else
  (void)afterResetOut; (void)afterFewOut; (void)afterManyOut;
  return false;
#endif
}

// ===========================================================================
// T1.3 fog — the analytic distance-fog resolve post pass (kFogResolve) + a headless
// self-test. Fullscreen image op (scene colour + metric depth in → fogged colour out),
// the foundation toward volumetric fog. Pure render-layer; mirrors the TAA resolve.
// ===========================================================================

// Build the fog-resolve pipeline (kFogResolve) + its 4-entry layout (FogParams uniform @0 +
// sceneTex @1 Float + depthTex @2 UnfilterableFloat + non-filtering sampler @3). Fullscreen, no
// vertex buffer, no depth. Building it naga-VALIDATES kFogResolve in-engine.
bool WbWgpuRenderTarget::ensureFogResolvePipeline() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (mFogPipeline)
    return true;
  if (!mUsable)
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());

  WGPUShaderSourceWGSL wgsl = {};
  wgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
  wgsl.code.data = WbWgpuShaders::kFogResolve;
  wgsl.code.length = WGPU_STRLEN;
  WGPUShaderModuleDescriptor smDesc = {};
  smDesc.nextInChain = &wgsl.chain;
  WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &smDesc);
  if (!sm) {
    WbLog::info("[WbWgpuRenderTarget] fog CreateShaderModule failed");
    return false;
  }
  mFogShaderModule = sm;

  WGPUSamplerDescriptor sampDesc = {};  // non-filtering (depth is unfilterable R32Float)
  sampDesc.addressModeU = WGPUAddressMode_ClampToEdge;
  sampDesc.addressModeV = WGPUAddressMode_ClampToEdge;
  sampDesc.addressModeW = WGPUAddressMode_ClampToEdge;
  sampDesc.magFilter = WGPUFilterMode_Nearest;
  sampDesc.minFilter = WGPUFilterMode_Nearest;
  sampDesc.mipmapFilter = WGPUMipmapFilterMode_Nearest;
  sampDesc.maxAnisotropy = 1;
  WGPUSampler samp = wgpuDeviceCreateSampler(device, &sampDesc);
  if (!samp) {
    WbLog::info("[WbWgpuRenderTarget] fog CreateSampler failed");
    return false;
  }
  mFogSampler = samp;

  WGPUBindGroupLayoutEntry bglEntries[4] = {};
  bglEntries[0].binding = 0;
  bglEntries[0].visibility = WGPUShaderStage_Fragment;
  bglEntries[0].buffer.type = WGPUBufferBindingType_Uniform;
  bglEntries[0].buffer.minBindingSize = 32;  // FogParams: 2 vec4
  bglEntries[1].binding = 1;
  bglEntries[1].visibility = WGPUShaderStage_Fragment;
  bglEntries[1].texture.sampleType = WGPUTextureSampleType_Float;
  bglEntries[1].texture.viewDimension = WGPUTextureViewDimension_2D;
  bglEntries[1].texture.multisampled = 0;
  bglEntries[2].binding = 2;
  bglEntries[2].visibility = WGPUShaderStage_Fragment;
  bglEntries[2].texture.sampleType = WGPUTextureSampleType_UnfilterableFloat;
  bglEntries[2].texture.viewDimension = WGPUTextureViewDimension_2D;
  bglEntries[2].texture.multisampled = 0;
  bglEntries[3].binding = 3;
  bglEntries[3].visibility = WGPUShaderStage_Fragment;
  bglEntries[3].sampler.type = WGPUSamplerBindingType_NonFiltering;
  WGPUBindGroupLayoutDescriptor bglDesc = {};
  bglDesc.entryCount = 4;
  bglDesc.entries = bglEntries;
  WGPUBindGroupLayout bgl = wgpuDeviceCreateBindGroupLayout(device, &bglDesc);
  if (!bgl) {
    WbLog::info("[WbWgpuRenderTarget] fog CreateBindGroupLayout failed");
    return false;
  }
  mFogBindGroupLayout = bgl;

  WGPUPipelineLayoutDescriptor plDesc = {};
  plDesc.bindGroupLayoutCount = 1;
  plDesc.bindGroupLayouts = &bgl;
  WGPUPipelineLayout plLayout = wgpuDeviceCreatePipelineLayout(device, &plDesc);
  if (!plLayout)
    return false;

  WGPUColorTargetState colorTarget = {};
  colorTarget.format = WGPUTextureFormat_RGBA8Unorm;
  colorTarget.writeMask = WGPUColorWriteMask_All;

  WGPUFragmentState fs = {};
  fs.module = sm;
  fs.entryPoint.data = "fs_main";
  fs.entryPoint.length = WGPU_STRLEN;
  fs.targetCount = 1;
  fs.targets = &colorTarget;

  WGPURenderPipelineDescriptor pipeDesc = {};
  pipeDesc.layout = plLayout;
  pipeDesc.vertex.module = sm;
  pipeDesc.vertex.entryPoint.data = "vs_main";
  pipeDesc.vertex.entryPoint.length = WGPU_STRLEN;
  pipeDesc.vertex.bufferCount = 0;
  pipeDesc.primitive.topology = WGPUPrimitiveTopology_TriangleList;
  pipeDesc.primitive.frontFace = WGPUFrontFace_CCW;
  pipeDesc.primitive.cullMode = WGPUCullMode_None;
  pipeDesc.depthStencil = nullptr;
  pipeDesc.multisample.count = 1;
  pipeDesc.multisample.mask = 0xFFFFFFFFu;
  pipeDesc.fragment = &fs;
  WGPURenderPipeline pipe = wgpuDeviceCreateRenderPipeline(device, &pipeDesc);
  wgpuPipelineLayoutRelease(plLayout);
  if (!pipe) {
    WbLog::info("[WbWgpuRenderTarget] fog CreateRenderPipeline failed");
    return false;
  }
  mFogPipeline = pipe;
  return true;
#  else
  return false;
#  endif
#else
  return false;
#endif
}

// T1.3 fog resolve render. See the header. Mirrors resolveTaa's fullscreen no-vertex-buffer draw.
bool WbWgpuRenderTarget::resolveFog(void *sceneView, void *depthView, const float fogColor3[3],
                                    float density, bool enabled, void *rgba8) {
  if (!mUsable || !rgba8 || !sceneView || !depthView || !fogColor3)
    return false;
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (!ensureFogResolvePipeline())
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUQueue queue = static_cast<WGPUQueue>(mBackend->queue());

  if (!mFogUniformBuffer) {
    WGPUBufferDescriptor ud = {};
    ud.size = 32;
    ud.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
    mFogUniformBuffer = wgpuDeviceCreateBuffer(device, &ud);
    if (!mFogUniformBuffer)
      return false;
  }
  const float params[8] = {
    fogColor3[0], fogColor3[1], fogColor3[2], density,   // fogColor (rgb) + density
    0.0f,         0.0f,         0.0f,         enabled ? 1.0f : 0.0f,  // params (height reserved) + enabled
  };
  wgpuQueueWriteBuffer(queue, static_cast<WGPUBuffer>(mFogUniformBuffer), 0, params, sizeof(params));

  WGPUBindGroupEntry e[4] = {};
  e[0].binding = 0;
  e[0].buffer = static_cast<WGPUBuffer>(mFogUniformBuffer);
  e[0].offset = 0;
  e[0].size = 32;
  e[1].binding = 1;
  e[1].textureView = static_cast<WGPUTextureView>(sceneView);
  e[2].binding = 2;
  e[2].textureView = static_cast<WGPUTextureView>(depthView);
  e[3].binding = 3;
  e[3].sampler = static_cast<WGPUSampler>(mFogSampler);
  WGPUBindGroupDescriptor bd = {};
  bd.layout = static_cast<WGPUBindGroupLayout>(mFogBindGroupLayout);
  bd.entryCount = 4;
  bd.entries = e;
  WGPUBindGroup bg = wgpuDeviceCreateBindGroup(device, &bd);
  if (!bg)
    return false;

  WGPUCommandEncoderDescriptor encDesc = {};
  WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(device, &encDesc);
  if (!encoder) {
    wgpuBindGroupRelease(bg);
    return false;
  }
  WGPURenderPassColorAttachment ca = {};
  ca.view = static_cast<WGPUTextureView>(mView);
  ca.loadOp = WGPULoadOp_Clear;
  ca.storeOp = WGPUStoreOp_Store;
  ca.clearValue = {0.0, 0.0, 0.0, 1.0};
  ca.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
  WGPURenderPassDescriptor pd = {};
  pd.colorAttachmentCount = 1;
  pd.colorAttachments = &ca;
  WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &pd);
  if (!pass) {
    wgpuCommandEncoderRelease(encoder);
    wgpuBindGroupRelease(bg);
    return false;
  }
  wgpuRenderPassEncoderSetPipeline(pass, static_cast<WGPURenderPipeline>(mFogPipeline));
  wgpuRenderPassEncoderSetBindGroup(pass, 0, bg, 0, nullptr);
  wgpuRenderPassEncoderDraw(pass, 6, 1, 0, 0);
  wgpuRenderPassEncoderEnd(pass);
  wgpuRenderPassEncoderRelease(pass);

  const uint32_t stride = alignedBytesPerRow(mWidth);
  WGPUTexelCopyTextureInfo src = {};
  src.texture = static_cast<WGPUTexture>(mTexture);
  src.aspect = WGPUTextureAspect_All;
  WGPUTexelCopyBufferInfo dst = {};
  dst.buffer = static_cast<WGPUBuffer>(mReadbackBuffer);
  dst.layout.bytesPerRow = stride;
  dst.layout.rowsPerImage = mHeight;
  WGPUExtent3D extent = {mWidth, mHeight, 1};
  wgpuCommandEncoderCopyTextureToBuffer(encoder, &src, &dst, &extent);

  WGPUCommandBufferDescriptor cmdDesc = {};
  WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(encoder, &cmdDesc);
  wgpuCommandEncoderRelease(encoder);
  if (!cmd) {
    wgpuBindGroupRelease(bg);
    return false;
  }
  wgpuQueueSubmit(queue, 1, &cmd);
  wgpuCommandBufferRelease(cmd);
  wgpuBindGroupRelease(bg);

  MapCapture cap;
  WGPUBufferMapCallbackInfo mapCb = {};
  mapCb.mode = WGPUCallbackMode_AllowSpontaneous;
  mapCb.callback = onMap;
  mapCb.userdata1 = &cap;
  (void)wgpuBufferMapAsync(static_cast<WGPUBuffer>(mReadbackBuffer), WGPUMapMode_Read, 0,
                           mReadbackBufferSize, mapCb);
  for (int i = 0; i < 1000 && !cap.done; ++i)
    wgpuDevicePoll(device, true, nullptr);
  if (!cap.done || !cap.ok)
    return false;
  const void *mapped =
    wgpuBufferGetConstMappedRange(static_cast<WGPUBuffer>(mReadbackBuffer), 0, mReadbackBufferSize);
  if (!mapped) {
    wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
    return false;
  }
  const uint32_t rowBytes = mWidth * 4u;
  uint8_t *out = static_cast<uint8_t *>(rgba8);
  const uint8_t *in = static_cast<const uint8_t *>(mapped);
  for (uint32_t y = 0; y < mHeight; ++y)
    std::memcpy(out + y * rowBytes, in + y * stride, rowBytes);
  wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
  return true;
#  else
  (void)sceneView; (void)depthView; (void)fogColor3; (void)density; (void)enabled; (void)rgba8;
  return false;
#  endif
#else
  (void)sceneView; (void)depthView; (void)fogColor3; (void)density; (void)enabled; (void)rgba8;
  return false;
#endif
}

// T1.3 fog headless self-test. White scene at a NEAR vs FAR uniform depth → far fogs heavily,
// near stays ~scene; plus fog-off passthrough. See the header.
bool WbWgpuRenderTarget::selfTestFog(unsigned char nearOut[3], unsigned char farOut[3],
                                     unsigned char offOut[3]) {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (!mUsable || !ensureFogResolvePipeline())  // pipeline build naga-validates kFogResolve in-engine
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUQueue queue = static_cast<WGPUQueue>(mBackend->queue());

  // White RGBA8 scene.
  WGPUTexture sceneTex = nullptr;
  WGPUTextureView sceneView = nullptr;
  {
    WGPUTextureDescriptor td = {};
    td.dimension = WGPUTextureDimension_2D;
    td.size = {1, 1, 1};
    td.format = WGPUTextureFormat_RGBA8Unorm;
    td.mipLevelCount = 1;
    td.sampleCount = 1;
    td.usage = WGPUTextureUsage_TextureBinding | WGPUTextureUsage_CopyDst;
    sceneTex = wgpuDeviceCreateTexture(device, &td);
    if (!sceneTex)
      return false;
    const uint8_t px[4] = {255, 255, 255, 255};
    WGPUTexelCopyTextureInfo dstT = {};
    dstT.texture = sceneTex;
    dstT.aspect = WGPUTextureAspect_All;
    WGPUTexelCopyBufferLayout lay = {};
    lay.bytesPerRow = 4;
    lay.rowsPerImage = 1;
    WGPUExtent3D ext = {1, 1, 1};
    wgpuQueueWriteTexture(queue, &dstT, px, 4, &lay, &ext);
    WGPUTextureViewDescriptor vd = {};
    vd.format = WGPUTextureFormat_RGBA8Unorm;
    vd.dimension = WGPUTextureViewDimension_2D;
    vd.mipLevelCount = 1;
    vd.arrayLayerCount = 1;
    sceneView = wgpuTextureCreateView(sceneTex, &vd);
  }
  // R32Float metric-depth textures: a NEAR distance and a FAR distance.
  auto make1x1F32 = [&](float value, WGPUTexture &tex, WGPUTextureView &view) -> bool {
    WGPUTextureDescriptor td = {};
    td.dimension = WGPUTextureDimension_2D;
    td.size = {1, 1, 1};
    td.format = WGPUTextureFormat_R32Float;
    td.mipLevelCount = 1;
    td.sampleCount = 1;
    td.usage = WGPUTextureUsage_TextureBinding | WGPUTextureUsage_CopyDst;
    tex = wgpuDeviceCreateTexture(device, &td);
    if (!tex)
      return false;
    WGPUTexelCopyTextureInfo dstT = {};
    dstT.texture = tex;
    dstT.aspect = WGPUTextureAspect_All;
    WGPUTexelCopyBufferLayout lay = {};
    lay.bytesPerRow = 4;
    lay.rowsPerImage = 1;
    WGPUExtent3D ext = {1, 1, 1};
    wgpuQueueWriteTexture(queue, &dstT, &value, 4, &lay, &ext);
    WGPUTextureViewDescriptor vd = {};
    vd.format = WGPUTextureFormat_R32Float;
    vd.dimension = WGPUTextureViewDimension_2D;
    vd.mipLevelCount = 1;
    vd.arrayLayerCount = 1;
    view = wgpuTextureCreateView(tex, &vd);
    return view != nullptr;
  };
  WGPUTexture nearTex = nullptr, farTex = nullptr;
  WGPUTextureView nearView = nullptr, farView = nullptr;
  bool ok = sceneView && make1x1F32(1.0f, nearTex, nearView) && make1x1F32(100.0f, farTex, farView);

  if (ok) {
    std::vector<unsigned char> buf(static_cast<size_t>(mWidth) * mHeight * 4, 0);
    const size_t c = (static_cast<size_t>(mHeight / 2) * mWidth + mWidth / 2) * 4;
    const float fogColor[3] = {0.1f, 0.3f, 0.8f};  // blue-ish
    const float density = 0.02f;
    auto sample = [&](unsigned char o[3]) { o[0] = buf[c]; o[1] = buf[c + 1]; o[2] = buf[c + 2]; };
    // near: dist 1m → fogFactor ~0.02 → ~white.
    ok = resolveFog(sceneView, nearView, fogColor, density, true, buf.data());
    if (ok) sample(nearOut);
    // far: dist 100m → fogFactor ~0.865 → heavily fog-coloured (blue).
    std::memset(buf.data(), 0, buf.size());
    ok = ok && resolveFog(sceneView, farView, fogColor, density, true, buf.data());
    if (ok) sample(farOut);
    // off: fog disabled → scene passthrough (white) regardless of depth.
    std::memset(buf.data(), 0, buf.size());
    ok = ok && resolveFog(sceneView, farView, fogColor, density, false, buf.data());
    if (ok) sample(offOut);
  }

  if (sceneView) wgpuTextureViewRelease(sceneView);
  if (sceneTex) wgpuTextureRelease(sceneTex);
  if (nearView) wgpuTextureViewRelease(nearView);
  if (nearTex) wgpuTextureRelease(nearTex);
  if (farView) wgpuTextureViewRelease(farView);
  if (farTex) wgpuTextureRelease(farTex);
  return ok;
#  else
  (void)nearOut; (void)farOut; (void)offOut;
  return false;
#  endif
#else
  (void)nearOut; (void)farOut; (void)offOut;
  return false;
#endif
}

// R4 lighting rung 1, increment 2: TWO-PASS TEXTURED+SHADOWED render. Pass 1 renders the scene's
// clip-depth from the light into the sampleable shadow map (identical to clearAndDrawSceneShadowed
// pass 1). Pass 2 renders the lit scene with kSolidLitTexturedShadow — the full material path
// (albedo/roughness/metalness/normal + GGX + sRGB) modulated by the PCF shadow term — reading the
// shadow map + a shared LightU uniform. Per-draw material bind groups (absent maps → 1×1 defaults;
// default-white albedo makes a non-textured draw render its flat baseColor). Reads back into rgba8.
// Release every cached textured-shadowed bind group (per-draw cache + the pass-1 group). Called on
// destruction and whenever a buffer baked into the groups is reallocated.
void WbWgpuRenderTarget::releaseTexShadowBgCache() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  for (auto &kv : mTexShadowBgCache)
    if (kv.second)
      wgpuBindGroupRelease(static_cast<WGPUBindGroup>(kv.second));
  mTexShadowBgCache.clear();
  mTexShadowBgCacheScnBuf = nullptr;
  mTexShadowBgCacheLightBuf = nullptr;
  if (mTexShadowP1Bg)
    wgpuBindGroupRelease(static_cast<WGPUBindGroup>(mTexShadowP1Bg));
  mTexShadowP1Bg = nullptr;
  mTexShadowP1BgBuf = nullptr;
#  endif
#endif
}

bool WbWgpuRenderTarget::clearAndDrawSceneTexturedShadowed(
  const WbWgpuClearColor &color, const float *viewProj16, const float *lightViewProj16,
  const float *lightDirAmbient4, const WbWgpuSolidDraw *draws, uint32_t numDraws, float shadowStrength,
  float depthBias, const float *cameraWorldPos3, const float *hemiSky4, const float *hemiGround4,
  const float *worldUp3, void *rgba8, bool asyncReadback, const float *skyUniform96,
  float directScale, const float *fogParams4, float bloomStrength, float agxExposure,
  float ssaoStrength, bool reversedZ) {
  // rgba8 == nullptr → render-only (window-swap present samples the texture directly; the whole
  // readback section is skipped — the CPU never touches pixels).
  if (!mUsable || !viewProj16 || !lightViewProj16 || !lightDirAmbient4)
    return false;
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (!ensureSceneClipDepthF32Pipeline() || !ensureShadowMapTexture() || !ensureTexturedShadowPipeline())
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUQueue queue = static_cast<WGPUQueue>(mBackend->queue());

  const uint32_t slotCount = numDraws == 0 ? 1u : numDraws;
  const size_t needed = static_cast<size_t>(slotCount) * kScnUniformStride;

  // Pass-1 (clip-depth) uniform → mShadowUniformBuffer (lightViewProj + model per slot).
  if (needed > mShadowUniformBufferSize) {
    if (mShadowUniformBuffer)
      wgpuBufferRelease(static_cast<WGPUBuffer>(mShadowUniformBuffer));
    WGPUBufferDescriptor ubDesc = {};
    ubDesc.size = needed;
    ubDesc.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
    mShadowUniformBuffer = wgpuDeviceCreateBuffer(device, &ubDesc);
    if (!mShadowUniformBuffer)
      return false;
    mShadowUniformBufferSize = needed;
  }
  std::vector<uint8_t> p1(needed, 0);
  for (uint32_t i = 0; i < numDraws; ++i) {
    uint8_t *slot = p1.data() + static_cast<size_t>(i) * kScnUniformStride;
    std::memcpy(slot + 0, lightViewProj16, 64);
    if (draws[i].modelMatrix16)
      std::memcpy(slot + 64, draws[i].modelMatrix16, 64);
  }
  wgpuQueueWriteBuffer(queue, static_cast<WGPUBuffer>(mShadowUniformBuffer), 0, p1.data(), needed);
  // Pass-1 bind group: cached across calls (it only references mShadowUniformBuffer via dynamic
  // offsets); rebuilt when that buffer is reallocated. Was created + released every frame.
  if (!mTexShadowP1Bg || mTexShadowP1BgBuf != mShadowUniformBuffer) {
    if (mTexShadowP1Bg)
      wgpuBindGroupRelease(static_cast<WGPUBindGroup>(mTexShadowP1Bg));
    WGPUBindGroupEntry p1e = {};
    p1e.binding = 0;
    p1e.buffer = static_cast<WGPUBuffer>(mShadowUniformBuffer);
    p1e.size = 192;
    WGPUBindGroupDescriptor p1d = {};
    p1d.layout = static_cast<WGPUBindGroupLayout>(mScnBindGroupLayout);
    p1d.entryCount = 1;
    p1d.entries = &p1e;
    mTexShadowP1Bg = wgpuDeviceCreateBindGroup(device, &p1d);
    mTexShadowP1BgBuf = mShadowUniformBuffer;
  }
  WGPUBindGroup p1bg = static_cast<WGPUBindGroup>(mTexShadowP1Bg);
  if (!p1bg)
    return false;

  // Pass-2 (material Scene) uniform → mScnUniformBuffer (same 192 B layout as clearAndDrawScene).
  if (needed > mScnUniformBufferSize) {
    if (mScnUniformBuffer)
      wgpuBufferRelease(static_cast<WGPUBuffer>(mScnUniformBuffer));
    WGPUBufferDescriptor ubDesc = {};
    ubDesc.size = needed;
    ubDesc.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
    mScnUniformBuffer = wgpuDeviceCreateBuffer(device, &ubDesc);
    if (!mScnUniformBuffer)
      return false;  // p1bg is cache-owned (mTexShadowP1Bg) — do not release on error paths
    mScnUniformBufferSize = needed;
  }
  std::vector<uint8_t> p2(needed, 0);
  for (uint32_t i = 0; i < numDraws; ++i) {
    uint8_t *slot = p2.data() + static_cast<size_t>(i) * kScnUniformStride;
    std::memcpy(slot + 0, viewProj16, 64);
    if (draws[i].modelMatrix16)
      std::memcpy(slot + 64, draws[i].modelMatrix16, 64);
    const float baseColor[4] = {draws[i].baseColorR, draws[i].baseColorG, draws[i].baseColorB,
                                draws[i].baseColorA};
    std::memcpy(slot + 128, baseColor, 16);
    std::memcpy(slot + 144, lightDirAmbient4, 16);
    if (cameraWorldPos3)
      std::memcpy(slot + 164, cameraWorldPos3, 12);
    // Emissive (pad1.xyz, same convention as the AgX path): self-lit surfaces — shop strips,
    // traffic lights, headlights — glow independently of the sun/day-night dimming.
    const float emissive[3] = {draws[i].emissiveR, draws[i].emissiveG, draws[i].emissiveB};
    std::memcpy(slot + 176, emissive, 12);
    std::memcpy(slot + 188, &draws[i].specularStrength, 4);
    std::memcpy(slot + 192, draws[i].uvA, 16);  // TextureTransform affine (uvA / uvB)
    std::memcpy(slot + 208, draws[i].uvB, 8);
  }
  wgpuQueueWriteBuffer(queue, static_cast<WGPUBuffer>(mScnUniformBuffer), 0, p2.data(), needed);

  // Shared LightU uniform (lightViewProj + shadowParams + hemisphere sky/ground/up, 128 B).
  if (!mLightUniformBuffer) {
    WGPUBufferDescriptor ud = {};
    ud.size = 144;
    ud.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
    mLightUniformBuffer = wgpuDeviceCreateBuffer(device, &ud);
    if (!mLightUniformBuffer)
      return false;
  }
  {
    // shadowParams.z = hemisphere-IBL enabled (1 when sky params are supplied, else 0 → the shader
    // falls back to the flat scalar ambient = byte-identical to the pre-hemisphere path).
    const float hemiOn = hemiSky4 ? 1.0f : 0.0f;
    uint8_t lu[144] = {0};
    std::memcpy(lu + 0, lightViewProj16, 64);
    const float sp[4] = {shadowStrength, depthBias, hemiOn, directScale};  // .w = day-night direct dimmer
    std::memcpy(lu + 64, sp, 16);
    if (hemiSky4)
      std::memcpy(lu + 80, hemiSky4, 16);   // skyColor.rgb + ambient-intensity scale in .w
    if (hemiGround4)
      std::memcpy(lu + 96, hemiGround4, 16);  // groundColor.rgb
    if (worldUp3)
      std::memcpy(lu + 112, worldUp3, 12);   // upDir.xyz
    if (fogParams4)
      std::memcpy(lu + 128, fogParams4, 16);  // fog: display-space color.rgb + density (0 = off)
    wgpuQueueWriteBuffer(queue, static_cast<WGPUBuffer>(mLightUniformBuffer), 0, lu, 144);
  }

  // The cached pass-2 bind groups bake in mScnUniformBuffer + mLightUniformBuffer; if either was
  // reallocated above, every cached group is stale → drop them all (they recreate on miss below).
  if (mTexShadowBgCacheScnBuf != mScnUniformBuffer || mTexShadowBgCacheLightBuf != mLightUniformBuffer) {
    for (auto &kv : mTexShadowBgCache)
      if (kv.second)
        wgpuBindGroupRelease(static_cast<WGPUBindGroup>(kv.second));
    mTexShadowBgCache.clear();
    mTexShadowBgCacheScnBuf = mScnUniformBuffer;
    mTexShadowBgCacheLightBuf = mLightUniformBuffer;
  }

  WGPUCommandEncoderDescriptor encDesc = {};
  WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(device, &encDesc);
  if (!encoder)
    return false;

  // ===== PASS 1: light clip-depth → shadow map =====
  {
    WGPURenderPassColorAttachment ca = {};
    ca.view = static_cast<WGPUTextureView>(mShadowMapView);
    ca.loadOp = WGPULoadOp_Clear;
    ca.storeOp = WGPUStoreOp_Store;
    ca.clearValue = {1.0, 0.0, 0.0, 1.0};
    ca.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
    WGPURenderPassDepthStencilAttachment da = {};
    da.view = static_cast<WGPUTextureView>(mShadowMapDepthView);
    da.depthLoadOp = WGPULoadOp_Clear;
    da.depthStoreOp = WGPUStoreOp_Store;
    da.depthClearValue = 1.0f;
    WGPURenderPassDescriptor pd = {};
    pd.colorAttachmentCount = 1;
    pd.colorAttachments = &ca;
    pd.depthStencilAttachment = &da;
    WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &pd);
    if (!pass) {
      wgpuCommandEncoderRelease(encoder);
      return false;
    }
    wgpuRenderPassEncoderSetPipeline(pass, static_cast<WGPURenderPipeline>(mScnClipDepthPipeline));
    for (uint32_t i = 0; i < numDraws; ++i) {
      const WbWgpuSolidDraw &d = draws[i];
      if (!d.vertexBuffer || !d.indexBuffer || d.indexCount == 0 || !d.castShadows)
        continue;
      const uint32_t dyn = i * kScnUniformStride;
      wgpuRenderPassEncoderSetBindGroup(pass, 0, p1bg, 1, &dyn);
      wgpuRenderPassEncoderSetVertexBuffer(pass, 0, static_cast<WGPUBuffer>(d.vertexBuffer), 0, WGPU_WHOLE_SIZE);
      wgpuRenderPassEncoderSetIndexBuffer(pass, static_cast<WGPUBuffer>(d.indexBuffer), WGPUIndexFormat_Uint32, 0,
                                          WGPU_WHOLE_SIZE);
      wgpuRenderPassEncoderDrawIndexed(pass, d.indexCount, 1, 0, 0, 0);
    }
    wgpuRenderPassEncoderEnd(pass);
    wgpuRenderPassEncoderRelease(pass);
  }

  // ===== PASS 2: textured + shadowed lit → color target =====
  bool hdr = false;  // set inside pass 2; the AgX tonemap pass below keys on it
  bool revActive = false;  // set inside pass 2; the SSAO prepass keys on it
  {
    // MSAA 4x when available: render into the multisampled attachment and RESOLVE into mView (the
    // single-sample texture the readback copies) — same output size/format, edges antialiased. The
    // 1x path is the unchanged fallback.
    const bool msaa = mTexShadowPipelineMsaa && ensureMsaaTargets();
    // HDR + AgX (agxExposure > 0): the scene renders into RGBA16F instead (4x → 1x resolve), and
    // the tonemap pass after this one maps it to display LDR in mView. MSAA-path only.
    hdr = agxExposure > 0.0f && msaa && ensureHdrPipelines();
    // Reversed-Z (caller-requested, MSAA path only): float depth + Greater variants. Falls back to
    // standard Z if the variants/attachments are unavailable.
    const bool rev = reversedZ && msaa && mTexShadowPipelineMsaaRev && (!hdr || mTexShadowPipelineHdrRev) &&
                     ensureReversedDepthTargets();
    revActive = rev;
    WGPURenderPassColorAttachment ca = {};
    ca.view = static_cast<WGPUTextureView>(hdr ? mHdrMsaaView : (msaa ? mScnMsaaColorView : mView));
    if (msaa)
      ca.resolveTarget = static_cast<WGPUTextureView>(hdr ? mHdrView : mView);
    ca.loadOp = WGPULoadOp_Clear;
    ca.storeOp = WGPUStoreOp_Store;
    ca.clearValue = {color.r, color.g, color.b, color.a};
    ca.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
    WGPURenderPassDepthStencilAttachment da = {};
    da.view = static_cast<WGPUTextureView>(rev ? mScnMsaaDepthViewF32
                                               : (msaa ? mScnMsaaDepthView : mScnDepthView));
    da.depthLoadOp = WGPULoadOp_Clear;
    da.depthStoreOp = WGPUStoreOp_Store;
    da.depthClearValue = rev ? 0.0f : 1.0f;  // reversed-Z clears to FAR = 0
    WGPURenderPassDescriptor pd = {};
    pd.colorAttachmentCount = 1;
    pd.colorAttachments = &ca;
    pd.depthStencilAttachment = &da;
    WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &pd);
    if (!pass) {
      wgpuCommandEncoderRelease(encoder);
      return false;
    }
    // Atmospheric sky dome behind the scene (MSAA path only — the pipeline is built count=4).
    // Drawn first with depth write off; geometry depth-tests over it. Falls back to the flat
    // clear colour when absent/unbuildable.
    if (skyUniform96 && msaa && ensureSkyPipeline() && (!hdr || mSkyPipelineHdr) &&
        (!rev || (hdr ? mSkyPipelineHdrRev : mSkyPipelineRev))) {
      wgpuQueueWriteBuffer(queue, static_cast<WGPUBuffer>(mSkyUniformBuffer), 0, skyUniform96, 96);
      wgpuRenderPassEncoderSetPipeline(
        pass, static_cast<WGPURenderPipeline>(rev ? (hdr ? mSkyPipelineHdrRev : mSkyPipelineRev)
                                                  : (hdr ? mSkyPipelineHdr : mSkyPipeline)));
      wgpuRenderPassEncoderSetBindGroup(pass, 0, static_cast<WGPUBindGroup>(mSkyBindGroup), 0, nullptr);
      wgpuRenderPassEncoderDraw(pass, 3, 1, 0, 0);
    }
    uint32_t dbgDrawn = 0, dbgNoBuf = 0, dbgNoBg = 0, dbgMaxIdx = 0;
    wgpuRenderPassEncoderSetPipeline(
      pass, static_cast<WGPURenderPipeline>(
              rev ? (hdr ? mTexShadowPipelineHdrRev : mTexShadowPipelineMsaaRev)
                  : (hdr ? mTexShadowPipelineHdr : (msaa ? mTexShadowPipelineMsaa : mTexShadowPipeline))));
    for (uint32_t i = 0; i < numDraws; ++i) {
      const WbWgpuSolidDraw &d = draws[i];
      if (!d.vertexBuffer || !d.indexBuffer || d.indexCount == 0) {
        ++dbgNoBuf;
        continue;
      }
      // Bind groups vary ONLY by the 4 resolved texture views (uniforms bind via dynamic offset;
      // samplers/shadow map are per-target constants) → look one up in the cache, create on miss.
      // A 3523-draw scene with ~57 textures used to create+release 3523 groups per frame here.
      const std::array<const void *, 4> key = {
        d.textureView ? d.textureView : mScnDefaultWhiteView,
        d.roughnessView ? d.roughnessView : mScnDefaultWhiteView,
        d.metalnessView ? d.metalnessView : mScnDefaultBlackView,
        d.normalView ? d.normalView : mScnDefaultNormalView};
      WGPUBindGroup bg = nullptr;
      auto it = mTexShadowBgCache.find(key);
      if (it != mTexShadowBgCache.end())
        bg = static_cast<WGPUBindGroup>(it->second);
      else {
        WGPUBindGroupEntry e[9] = {};
        e[0].binding = 0;
        e[0].buffer = static_cast<WGPUBuffer>(mScnUniformBuffer);
        e[0].size = 224;  // 192 + uvA/uvB
        e[1].binding = 1;
        e[1].textureView = static_cast<WGPUTextureView>(const_cast<void *>(key[0]));
        e[2].binding = 2;
        e[2].textureView = static_cast<WGPUTextureView>(const_cast<void *>(key[1]));
        e[3].binding = 3;
        e[3].textureView = static_cast<WGPUTextureView>(const_cast<void *>(key[2]));
        e[4].binding = 4;
        e[4].textureView = static_cast<WGPUTextureView>(const_cast<void *>(key[3]));
        e[5].binding = 5;
        e[5].sampler = static_cast<WGPUSampler>(mScnTexSampler);
        e[6].binding = 6;
        e[6].textureView = static_cast<WGPUTextureView>(mShadowMapView);
        e[7].binding = 7;
        e[7].sampler = static_cast<WGPUSampler>(mTexShadowSampler);
        e[8].binding = 8;
        e[8].buffer = static_cast<WGPUBuffer>(mLightUniformBuffer);
        e[8].size = 144;
        WGPUBindGroupDescriptor bd = {};
        bd.layout = static_cast<WGPUBindGroupLayout>(mTexShadowBindGroupLayout);
        bd.entryCount = 9;
        bd.entries = e;
        bg = wgpuDeviceCreateBindGroup(device, &bd);
        if (bg)
          mTexShadowBgCache.emplace(key, bg);
      }
      if (!bg) {
        ++dbgNoBg;
        continue;
      }
      const uint32_t dyn = i * kScnUniformStride;
      wgpuRenderPassEncoderSetBindGroup(pass, 0, bg, 1, &dyn);
      wgpuRenderPassEncoderSetVertexBuffer(pass, 0, static_cast<WGPUBuffer>(d.vertexBuffer), 0, WGPU_WHOLE_SIZE);
      wgpuRenderPassEncoderSetIndexBuffer(pass, static_cast<WGPUBuffer>(d.indexBuffer), WGPUIndexFormat_Uint32, 0,
                                          WGPU_WHOLE_SIZE);
      wgpuRenderPassEncoderDrawIndexed(pass, d.indexCount, 1, 0, 0, 0);
      ++dbgDrawn;
      if (d.indexCount > dbgMaxIdx)
        dbgMaxIdx = d.indexCount;
    }
    wgpuRenderPassEncoderEnd(pass);
    wgpuRenderPassEncoderRelease(pass);
    // R4 lighting-debug: report pass-2 draw accounting (which draws actually submitted) so a
    // missing surface (e.g. the ground plane) shows up as drawn<numDraws or a skip bucket.
    if (const char *ep = std::getenv("OMNISIM_WGPU_ERRLOG")) {
      if (FILE *fp = std::fopen(ep, "a")) {
        std::fprintf(fp, "[texshadow-pass2] numDraws=%u drawn=%u skipNoBuf=%u skipNoBg=%u maxIdxCount=%u\n",
                     numDraws, dbgDrawn, dbgNoBuf, dbgNoBg, dbgMaxIdx);
        std::fclose(fp);
      }
    }
  }

  // ===== AgX tonemap: HDR resolve → exposure → AgX curve → display LDR in mView. Bloom and the
  // readback below then operate on the tonemapped frame exactly as before. =====
  if (hdr) {
    const float exp4[4] = {agxExposure, 0.0f, 0.0f, 0.0f};
    wgpuQueueWriteBuffer(queue, static_cast<WGPUBuffer>(mAgxUb), 0, exp4, 16);
    WGPURenderPassColorAttachment ca = {};
    ca.view = static_cast<WGPUTextureView>(mView);
    ca.loadOp = WGPULoadOp_Clear;
    ca.storeOp = WGPUStoreOp_Store;
    ca.clearValue = {0.0, 0.0, 0.0, 1.0};
    ca.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
    WGPURenderPassDescriptor pd = {};
    pd.colorAttachmentCount = 1;
    pd.colorAttachments = &ca;
    WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &pd);
    if (pass) {
      wgpuRenderPassEncoderSetPipeline(pass, static_cast<WGPURenderPipeline>(mAgxPipeline));
      wgpuRenderPassEncoderSetBindGroup(pass, 0, static_cast<WGPUBindGroup>(mAgxBg), 0, nullptr);
      wgpuRenderPassEncoderDraw(pass, 3, 1, 0, 0);
      wgpuRenderPassEncoderEnd(pass);
      wgpuRenderPassEncoderRelease(pass);
    }
  }

  // ===== SSAO (optional): camera clip-depth prepass → half-res AO estimate → shared blur →
  // MULTIPLY onto the scene. Runs before bloom (halos must not be darkened); reuses the bloom
  // ping-pong textures, which are free again by the time bloom runs. =====
  if (ssaoStrength > 0.0f && ensureSsaoResources()) {
    if (!mSsaoSceneBg || mSsaoSceneBgBuf != mScnUniformBuffer) {
      if (mSsaoSceneBg)
        wgpuBindGroupRelease(static_cast<WGPUBindGroup>(mSsaoSceneBg));
      WGPUBindGroupEntry se = {};
      se.binding = 0;
      se.buffer = static_cast<WGPUBuffer>(mScnUniformBuffer);
      se.size = 192;
      WGPUBindGroupDescriptor sd = {};
      sd.layout = static_cast<WGPUBindGroupLayout>(mScnBindGroupLayout);
      sd.entryCount = 1;
      sd.entries = &se;
      mSsaoSceneBg = wgpuDeviceCreateBindGroup(device, &sd);
      mSsaoSceneBgBuf = mScnUniformBuffer;
    }
    if (mSsaoSceneBg) {
      const bool revPreU = revActive && mScnClipDepthPipelineRev;
      const float sp4[4] = {18.0f / static_cast<float>(mWidth), ssaoStrength,
                            revPreU ? 1.0f : 0.0f, 0.0f};
      wgpuQueueWriteBuffer(queue, static_cast<WGPUBuffer>(mSsaoUb), 0, sp4, 16);
      // Camera clip-depth prepass (the pass-2 scene uniform already holds camera VP + models).
      {
        WGPURenderPassColorAttachment ca = {};
        ca.view = static_cast<WGPUTextureView>(mAoDepthView);
        ca.loadOp = WGPULoadOp_Clear;
        ca.storeOp = WGPUStoreOp_Store;
        ca.clearValue = {1.0, 0.0, 0.0, 1.0};
        ca.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
        WGPURenderPassDepthStencilAttachment da = {};
        const bool revPre = revPreU;  // attachment must match pipeline + the ub flag
        da.view = static_cast<WGPUTextureView>(revPre ? mScnDepthViewF32 : mScnDepthView);
        da.depthLoadOp = WGPULoadOp_Clear;
        da.depthStoreOp = WGPUStoreOp_Store;
        da.depthClearValue = revPre ? 0.0f : 1.0f;
        WGPURenderPassDescriptor pd = {};
        pd.colorAttachmentCount = 1;
        pd.colorAttachments = &ca;
        pd.depthStencilAttachment = &da;
        WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &pd);
        if (pass) {
          wgpuRenderPassEncoderSetPipeline(
            pass, static_cast<WGPURenderPipeline>(
                    revPre ? mScnClipDepthPipelineRev : mScnClipDepthPipeline));
          for (uint32_t i = 0; i < numDraws; ++i) {
            const WbWgpuSolidDraw &d = draws[i];
            if (!d.vertexBuffer || !d.indexBuffer || d.indexCount == 0)
              continue;
            const uint32_t dyn = i * kScnUniformStride;
            wgpuRenderPassEncoderSetBindGroup(pass, 0, static_cast<WGPUBindGroup>(mSsaoSceneBg), 1, &dyn);
            wgpuRenderPassEncoderSetVertexBuffer(pass, 0, static_cast<WGPUBuffer>(d.vertexBuffer), 0,
                                                 WGPU_WHOLE_SIZE);
            wgpuRenderPassEncoderSetIndexBuffer(pass, static_cast<WGPUBuffer>(d.indexBuffer),
                                                WGPUIndexFormat_Uint32, 0, WGPU_WHOLE_SIZE);
            wgpuRenderPassEncoderDrawIndexed(pass, d.indexCount, 1, 0, 0, 0);
          }
          wgpuRenderPassEncoderEnd(pass);
          wgpuRenderPassEncoderRelease(pass);
        }
      }
      // Estimate → blur H → blur V → multiply-apply.
      struct {
        void *pipe;
        void *bg;
        void *target;
        bool load;
      } aoStages[4] = {
        {mSsaoPipeline, mSsaoBgEstimate, mBloomViewA, false},
        {mBloomBlurPipeline, mBloomBgBlurH, mBloomViewB, false},
        {mBloomBlurPipeline, mBloomBgBlurV, mBloomViewA, false},
        {mSsaoApplyPipeline, mSsaoBgApply, mView, true},
      };
      for (const auto &s : aoStages) {
        WGPURenderPassColorAttachment ca = {};
        ca.view = static_cast<WGPUTextureView>(s.target);
        ca.loadOp = s.load ? WGPULoadOp_Load : WGPULoadOp_Clear;
        ca.storeOp = WGPUStoreOp_Store;
        ca.clearValue = {1.0, 1.0, 1.0, 1.0};
        ca.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
        WGPURenderPassDescriptor pd = {};
        pd.colorAttachmentCount = 1;
        pd.colorAttachments = &ca;
        WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &pd);
        if (!pass)
          break;
        wgpuRenderPassEncoderSetPipeline(pass, static_cast<WGPURenderPipeline>(s.pipe));
        wgpuRenderPassEncoderSetBindGroup(pass, 0, static_cast<WGPUBindGroup>(s.bg), 0, nullptr);
        wgpuRenderPassEncoderDraw(pass, 3, 1, 0, 0);
        wgpuRenderPassEncoderEnd(pass);
        wgpuRenderPassEncoderRelease(pass);
      }
    }
  }

  // ===== BLOOM post-passes (optional): extract bright pixels (half-res) → blur H → blur V →
  // additive composite back onto the scene texture. All on the resolved 1x texture, appended to
  // the same encoder so the readback below picks the bloomed frame up unchanged. =====
  if (bloomStrength > 0.0f && ensureBloomPipelines(bloomStrength)) {
    // The composite strength is per-call (the infra may have been created by SSAO with a default).
    const float bs4[4] = {bloomStrength, 0.0f, 0.0f, 0.0f};
    wgpuQueueWriteBuffer(queue, static_cast<WGPUBuffer>(mBloomUbComposite), 0, bs4, 16);
    struct {
      void *pipe;
      void *bg;
      void *target;
      bool load;
    } stages[4] = {
      {mBloomExtractPipeline, mBloomBgExtract, mBloomViewA, false},
      {mBloomBlurPipeline, mBloomBgBlurH, mBloomViewB, false},
      {mBloomBlurPipeline, mBloomBgBlurV, mBloomViewA, false},
      {mBloomCompositePipeline, mBloomBgComposite, mView, true},  // additive onto the scene
    };
    for (const auto &s : stages) {
      WGPURenderPassColorAttachment ca = {};
      ca.view = static_cast<WGPUTextureView>(s.target);
      ca.loadOp = s.load ? WGPULoadOp_Load : WGPULoadOp_Clear;
      ca.storeOp = WGPUStoreOp_Store;
      ca.clearValue = {0.0, 0.0, 0.0, 1.0};
      ca.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
      WGPURenderPassDescriptor pd = {};
      pd.colorAttachmentCount = 1;
      pd.colorAttachments = &ca;
      WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &pd);
      if (!pass)
        break;
      wgpuRenderPassEncoderSetPipeline(pass, static_cast<WGPURenderPipeline>(s.pipe));
      wgpuRenderPassEncoderSetBindGroup(pass, 0, static_cast<WGPUBindGroup>(s.bg), 0, nullptr);
      wgpuRenderPassEncoderDraw(pass, 3, 1, 0, 0);
      wgpuRenderPassEncoderEnd(pass);
      wgpuRenderPassEncoderRelease(pass);
    }
  }

  if (!rgba8) {
    // Render-only: finish + submit without any texture→buffer copy or map.
    WGPUCommandBufferDescriptor cmdDescNb = {};
    WGPUCommandBuffer cmdNb = wgpuCommandEncoderFinish(encoder, &cmdDescNb);
    wgpuCommandEncoderRelease(encoder);
    if (!cmdNb)
      return false;
    wgpuQueueSubmit(queue, 1, &cmdNb);
    wgpuCommandBufferRelease(cmdNb);
    return true;
  }

  const uint32_t stride = alignedBytesPerRow(mWidth);
  // Async pipelining: ping-pong the copy destination so the pending map (last frame, other buffer)
  // can complete while this frame's copy lands. Lazily create the second buffer.
  if (asyncReadback && !mReadbackBufferB) {
    WGPUBufferDescriptor abd = {};
    abd.size = mReadbackBufferSize;
    abd.usage = WGPUBufferUsage_CopyDst | WGPUBufferUsage_MapRead;
    mReadbackBufferB = wgpuDeviceCreateBuffer(device, &abd);
    if (!mReadbackBufferB)
      asyncReadback = false;  // fall back to the synchronous path
  }
  WGPUBuffer rbBufs[2] = {static_cast<WGPUBuffer>(mReadbackBuffer),
                          static_cast<WGPUBuffer>(mReadbackBufferB)};
  WGPUTexelCopyTextureInfo src = {};
  src.texture = static_cast<WGPUTexture>(mTexture);
  src.aspect = WGPUTextureAspect_All;
  WGPUTexelCopyBufferInfo dst = {};
  dst.buffer = asyncReadback ? rbBufs[mAsyncWhich] : rbBufs[0];
  dst.layout.bytesPerRow = stride;
  dst.layout.rowsPerImage = mHeight;
  WGPUExtent3D extent = {mWidth, mHeight, 1};
  wgpuCommandEncoderCopyTextureToBuffer(encoder, &src, &dst, &extent);

  WGPUCommandBufferDescriptor cmdDesc = {};
  WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(encoder, &cmdDesc);
  wgpuCommandEncoderRelease(encoder);
  if (!cmd)
    return false;
  wgpuQueueSubmit(queue, 1, &cmd);
  wgpuCommandBufferRelease(cmd);

  // Diagnostic: dump the light-depth shadow map (pass-1 output) once, normalized to 8-bit gray.
  // The five receiver-side acne cures all failed on spot.wbt — this shows what pass 1 actually
  // stored. Env-gated; sync map; first call only.
  {
    static bool sShadowDumped = false;
    const char *sdPath = std::getenv("OMNISIM_WGPU_SHADOWMAP_DUMP");
    if (sdPath && !sShadowDumped) {
      sShadowDumped = true;
      WGPUCommandEncoderDescriptor edd = {};
      WGPUCommandEncoder enc2 = wgpuDeviceCreateCommandEncoder(device, &edd);
      if (enc2) {
        WGPUTexelCopyTextureInfo src2 = {};
        src2.texture = static_cast<WGPUTexture>(mShadowMapTexture);
        src2.aspect = WGPUTextureAspect_All;
        WGPUTexelCopyBufferInfo dst2 = {};
        dst2.buffer = static_cast<WGPUBuffer>(mReadbackBuffer);
        dst2.layout.bytesPerRow = alignedBytesPerRow(mWidth);
        dst2.layout.rowsPerImage = mHeight;
        WGPUExtent3D ext2 = {mWidth, mHeight, 1};
        wgpuCommandEncoderCopyTextureToBuffer(enc2, &src2, &dst2, &ext2);
        WGPUCommandBufferDescriptor cbd2 = {};
        WGPUCommandBuffer cmd2 = wgpuCommandEncoderFinish(enc2, &cbd2);
        wgpuCommandEncoderRelease(enc2);
        if (cmd2) {
          wgpuQueueSubmit(queue, 1, &cmd2);
          wgpuCommandBufferRelease(cmd2);
          MapCapture cap2;
          WGPUBufferMapCallbackInfo cb2 = {};
          cb2.mode = WGPUCallbackMode_AllowSpontaneous;
          cb2.callback = onMap;
          cb2.userdata1 = &cap2;
          (void)wgpuBufferMapAsync(static_cast<WGPUBuffer>(mReadbackBuffer), WGPUMapMode_Read, 0,
                                   mReadbackBufferSize, cb2);
          for (int i = 0; i < 1000 && !cap2.done; ++i)
            wgpuDevicePoll(device, true, nullptr);
          if (cap2.done && cap2.ok) {
            const void *mp = wgpuBufferGetConstMappedRange(static_cast<WGPUBuffer>(mReadbackBuffer), 0,
                                                           mReadbackBufferSize);
            if (mp) {
              const uint32_t strideF = alignedBytesPerRow(mWidth) / 4u;
              const float *fp = static_cast<const float *>(mp);
              QImage img(static_cast<int>(mWidth), static_cast<int>(mHeight), QImage::Format_Grayscale8);
              for (uint32_t y = 0; y < mHeight; ++y) {
                uchar *row = img.scanLine(static_cast<int>(y));
                for (uint32_t x = 0; x < mWidth; ++x) {
                  const float v = fp[static_cast<size_t>(y) * strideF + x];
                  row[x] = static_cast<uchar>(std::max(0.0f, std::min(1.0f, v)) * 255.0f);
                }
              }
              img.save(QString::fromLocal8Bit(sdPath), "PNG");
            }
            wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
          }
        }
      }
    }
  }

  if (asyncReadback) {
    // Deliver the PREVIOUS frame: its map (issued last call, other buffer) has had a whole frame to
    // complete, so the poll below is normally instant — never a same-frame GPU wait.
    if (mAsyncPending) {
      MapCapture *pcap = reinterpret_cast<MapCapture *>(&mAsyncCap);
      for (int i = 0; i < 1000 && !pcap->done; ++i)
        wgpuDevicePoll(device, true, nullptr);
      WGPUBuffer prev = rbBufs[1 - mAsyncWhich];
      if (pcap->done && pcap->ok) {
        const void *mapped = wgpuBufferGetConstMappedRange(prev, 0, mReadbackBufferSize);
        if (mapped) {
          const uint32_t rowBytes2 = mWidth * 4u;
          uint8_t *out2 = static_cast<uint8_t *>(rgba8);
          const uint8_t *in2 = static_cast<const uint8_t *>(mapped);
          for (uint32_t y = 0; y < mHeight; ++y)
            std::memcpy(out2 + y * rowBytes2, in2 + y * stride, rowBytes2);
        }
        wgpuBufferUnmap(prev);
      }
      mAsyncPending = false;
    }
    // Queue this frame's map; it resolves while the caller blits/steps — collected next call.
    mAsyncCap.done = false;
    mAsyncCap.ok = false;
    WGPUBufferMapCallbackInfo acb = {};
    acb.mode = WGPUCallbackMode_AllowSpontaneous;
    acb.callback = onMap;
    acb.userdata1 = &mAsyncCap;
    (void)wgpuBufferMapAsync(rbBufs[mAsyncWhich], WGPUMapMode_Read, 0, mReadbackBufferSize, acb);
    wgpuDevicePoll(device, false, nullptr);  // pump once, no wait
    mAsyncPending = true;
    mAsyncWhich = 1 - mAsyncWhich;
    return true;  // rgba8 holds frame N-1 (or is untouched on the very first call)
  }

  MapCapture cap;
  WGPUBufferMapCallbackInfo mapCb = {};
  mapCb.mode = WGPUCallbackMode_AllowSpontaneous;
  mapCb.callback = onMap;
  mapCb.userdata1 = &cap;
  (void)wgpuBufferMapAsync(static_cast<WGPUBuffer>(mReadbackBuffer), WGPUMapMode_Read, 0,
                           mReadbackBufferSize, mapCb);
  for (int i = 0; i < 1000 && !cap.done; ++i)
    wgpuDevicePoll(device, true, nullptr);
  if (!cap.done || !cap.ok)
    return false;
  const void *mapped = wgpuBufferGetConstMappedRange(static_cast<WGPUBuffer>(mReadbackBuffer), 0,
                                                     mReadbackBufferSize);
  if (!mapped) {
    wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
    return false;
  }
  const uint32_t rowBytes = mWidth * 4u;
  uint8_t *out = static_cast<uint8_t *>(rgba8);
  const uint8_t *in = static_cast<const uint8_t *>(mapped);
  for (uint32_t y = 0; y < mHeight; ++y)
    std::memcpy(out + y * rowBytes, in + y * stride, rowBytes);
  wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
  return true;
#  else
  (void)color; (void)viewProj16; (void)lightViewProj16; (void)lightDirAmbient4; (void)draws;
  (void)numDraws; (void)shadowStrength; (void)depthBias; (void)cameraWorldPos3; (void)rgba8;
  (void)hemiSky4; (void)hemiGround4; (void)worldUp3;
  return false;
#  endif
#else
  (void)color; (void)viewProj16; (void)lightViewProj16; (void)lightDirAmbient4; (void)draws;
  (void)numDraws; (void)shadowStrength; (void)depthBias; (void)cameraWorldPos3; (void)rgba8;
  (void)hemiSky4; (void)hemiGround4; (void)worldUp3;
  return false;
#endif
}

bool WbWgpuRenderTarget::clearAndDrawScene(const WbWgpuClearColor &color,
                                            const float *viewProj16,
                                            const float *lightDirAmbient4,
                                            const WbWgpuSolidDraw *draws, uint32_t numDraws,
                                            void *rgba8, bool depthMode, float farPlane,
                                            bool agxMode, const float *cameraWorldPos3,
                                            bool pickMode, bool srgbEncode) {
  if (!mUsable || !rgba8 || !viewProj16 || !lightDirAmbient4)
    return false;
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (!ensureScenePipeline())
    return false;
  if (depthMode && !ensureSceneDepthPipeline())
    return false;
  // T1.1: AgX honored only when not in depth mode (depth wins). Build lazily;
  // on failure fall back to the plain lit pipeline.
  const bool useAgx = agxMode && !depthMode && ensureSceneAgxPipeline();
  // R4 step-3c-A.1: flat picking pass (wins over all others). baseColor carries
  // an encoded ID; the fragment emits it unshaded for an exact RGBA8 round-trip.
  const bool usePick = pickMode && ensurePickPipeline();
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUQueue queue = static_cast<WGPUQueue>(mBackend->queue());

  // Grow scene uniform buffer to fit numDraws * 256-byte slots.
  // Always allocate at least one slot so an empty-scene call still
  // has a valid buffer to bind (the render pass will just clear).
  // NOTE: `slots` is reserved by Qt's moc — even renaming a local
  // can collide on the right transitive include. Use `slotCount`.
  const uint32_t slotCount = numDraws == 0 ? 1u : numDraws;
  const size_t needed = static_cast<size_t>(slotCount) * kScnUniformStride;
  if (needed > mScnUniformBufferSize) {
    if (mScnUniformBuffer)
      wgpuBufferRelease(static_cast<WGPUBuffer>(mScnUniformBuffer));
    WGPUBufferDescriptor ubDesc = {};
    ubDesc.size = needed;
    ubDesc.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
    ubDesc.mappedAtCreation = false;
    WGPUBuffer ub = wgpuDeviceCreateBuffer(device, &ubDesc);
    if (!ub)
      return false;
    mScnUniformBuffer = ub;
    mScnUniformBufferSize = needed;
  }

  // Pack the Scene struct for each draw into a single host buffer.
  // 256-byte stride; 192 B payload + 64 B padding per slot.
  std::vector<uint8_t> hostBuf(needed, 0);
  for (uint32_t i = 0; i < numDraws; ++i) {
    uint8_t *slot = hostBuf.data() + static_cast<size_t>(i) * kScnUniformStride;
    std::memcpy(slot + 0, viewProj16, 64);
    if (draws[i].modelMatrix16)
      std::memcpy(slot + 64, draws[i].modelMatrix16, 64);
    float baseColor[4] = {draws[i].baseColorR, draws[i].baseColorG, draws[i].baseColorB,
                          draws[i].baseColorA};
    std::memcpy(slot + 128, baseColor, 16);
    std::memcpy(slot + 144, lightDirAmbient4, 16);
    // pad0.x (offset 160) is shared by the two mutually-exclusive modes that use
    // it: depthMode → far plane (kSolidDistance divides clip.w by it); agxMode →
    // exposure scale (kSolidLitAgX multiplies the lit colour by it pre-tonemap).
    // farPlane carries both values from the caller (it is unused by the AgX
    // shader otherwise). Plain lit path leaves it zero.
    if (depthMode || useAgx) {
      std::memcpy(slot + 160, &farPlane, 4);
    } else {
      // Plain-lit (kSolidLit) path: pad0.x doubles as the sRGB-output flag. 0 → encode sRGB
      // (display surfaces — the pane + screenshots, the default for every caller); 1 → linear
      // output (sensor RTT, e.g. the camera device, which keeps its R5-landed linear space).
      // farPlane is unused by kSolidLit so there is no clash, and srgbEncode=true writes 0 =
      // the prior zero-init → byte-identical for all existing display callers.
      const float linearFlag = srgbEncode ? 0.0f : 1.0f;
      std::memcpy(slot + 160, &linearFlag, 4);
    }
    // T1.1 HDR source: per-draw emissive (color × intensity) in pad1.xyz
    // (offset 176). Only kSolidLitAgX reads it (added to the lit colour pre-AgX
    // so a >1 emissive gives the tonemap genuine HDR to compress); every other
    // shader ignores pad1, so writing it is harmless and an all-zero emissive
    // (the PBRAppearance default) leaves their output byte-identical.
    float emissive[3] = {draws[i].emissiveR, draws[i].emissiveG, draws[i].emissiveB};
    std::memcpy(slot + 176, emissive, 12);
    // T1.1 specular foundation: camera world position in pad0.yzw (offset 164)
    // when supplied. No shader reads it yet, so this is byte-identical until a
    // specular term lands; null leaves pad0.yzw zero.
    if (cameraWorldPos3)
      std::memcpy(slot + 164, cameraWorldPos3, 12);
    // T1.1 specular: smoothness (1-roughness) in pad1.w (offset 188). Only
    // kSolidLitAgX reads it; 0 (roughness=1, or any non-AgX path) → no specular,
    // byte-identical.
    std::memcpy(slot + 188, &draws[i].specularStrength, 4);
    // 192..255 stays zero (slack).
  }
  wgpuQueueWriteBuffer(queue, static_cast<WGPUBuffer>(mScnUniformBuffer), 0, hostBuf.data(),
                       needed);

  // One bind group reused across all draws via dynamic offset.
  WGPUBindGroupEntry bgEntry = {};
  bgEntry.binding = 0;
  bgEntry.buffer = static_cast<WGPUBuffer>(mScnUniformBuffer);
  bgEntry.offset = 0;
  bgEntry.size = 192;  // payload only; offset comes from SetBindGroup
  WGPUBindGroupDescriptor bgDesc = {};
  bgDesc.layout = static_cast<WGPUBindGroupLayout>(mScnBindGroupLayout);
  bgDesc.entryCount = 1;
  bgDesc.entries = &bgEntry;
  WGPUBindGroup bg = wgpuDeviceCreateBindGroup(device, &bgDesc);
  if (!bg)
    return false;

  WGPUCommandEncoderDescriptor encDesc = {};
  WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(device, &encDesc);
  if (!encoder) {
    wgpuBindGroupRelease(bg);
    return false;
  }

  WGPURenderPassColorAttachment colorAttach = {};
  colorAttach.view = static_cast<WGPUTextureView>(mView);
  colorAttach.loadOp = WGPULoadOp_Clear;
  colorAttach.storeOp = WGPUStoreOp_Store;
  colorAttach.clearValue = {color.r, color.g, color.b, color.a};
  colorAttach.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;

  WGPURenderPassDepthStencilAttachment depthAttach = {};
  depthAttach.view = static_cast<WGPUTextureView>(mScnDepthView);
  depthAttach.depthLoadOp = WGPULoadOp_Clear;
  depthAttach.depthStoreOp = WGPUStoreOp_Store;
  depthAttach.depthClearValue = 1.0f;
  depthAttach.depthReadOnly = 0;
  // wgpu spec: when the pipeline has a depthStencil format, the
  // render pass MUST provide a matching depthStencilAttachment.
  // We always attach it.

  WGPURenderPassDescriptor passDesc = {};
  passDesc.colorAttachmentCount = 1;
  passDesc.colorAttachments = &colorAttach;
  passDesc.depthStencilAttachment = &depthAttach;
  WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &passDesc);
  if (!pass) {
    wgpuCommandEncoderRelease(encoder);
    wgpuBindGroupRelease(bg);
    return false;
  }
  WGPURenderPipeline litPipe = static_cast<WGPURenderPipeline>(
    usePick ? mPickPipeline
            : (depthMode ? mScnDepthPipeline : (useAgx ? mScnAgxPipeline : mScnPipeline)));
  // R4 material fidelity: only the plain lit path samples albedo (pick/depth/agx keep
  // their flat semantics). Textured draws switch to mScnTexPipeline + a per-draw bind
  // group; flat draws stay on litPipe + the shared bg. texBgs released post-submit.
  const bool texReady = !usePick && !depthMode && !useAgx && ensureSceneTexPipeline();
  std::vector<WGPUBindGroup> texBgs;
  for (uint32_t i = 0; i < numDraws; ++i) {
    const WbWgpuSolidDraw &d = draws[i];
    if (!d.vertexBuffer || !d.indexBuffer || d.indexCount == 0)
      continue;
    const uint32_t dynOffset = i * kScnUniformStride;
    WGPUBindGroup useBg = bg;
    WGPURenderPipeline usePipe = litPipe;
    if (texReady && d.textureView) {
      WGPUBindGroupEntry e[6] = {};
      e[0].binding = 0;
      e[0].buffer = static_cast<WGPUBuffer>(mScnUniformBuffer);
      e[0].offset = 0;
      e[0].size = 192;
      e[1].binding = 1;
      e[1].textureView = static_cast<WGPUTextureView>(d.textureView);
      e[2].binding = 2;
      e[2].textureView = static_cast<WGPUTextureView>(
        d.roughnessView ? d.roughnessView : mScnDefaultWhiteView);
      e[3].binding = 3;
      e[3].textureView = static_cast<WGPUTextureView>(
        d.metalnessView ? d.metalnessView : mScnDefaultBlackView);
      e[4].binding = 4;
      e[4].textureView = static_cast<WGPUTextureView>(
        d.normalView ? d.normalView : mScnDefaultNormalView);
      e[5].binding = 5;
      e[5].sampler = static_cast<WGPUSampler>(mScnTexSampler);
      WGPUBindGroupDescriptor bd = {};
      bd.layout = static_cast<WGPUBindGroupLayout>(mScnTexBindGroupLayout);
      bd.entryCount = 6;
      bd.entries = e;
      WGPUBindGroup tbg = wgpuDeviceCreateBindGroup(device, &bd);
      if (tbg) {
        texBgs.push_back(tbg);
        useBg = tbg;
        usePipe = static_cast<WGPURenderPipeline>(mScnTexPipeline);
      }
    }
    wgpuRenderPassEncoderSetPipeline(pass, usePipe);
    wgpuRenderPassEncoderSetBindGroup(pass, 0, useBg, 1, &dynOffset);
    wgpuRenderPassEncoderSetVertexBuffer(pass, 0, static_cast<WGPUBuffer>(d.vertexBuffer), 0,
                                         WGPU_WHOLE_SIZE);
    wgpuRenderPassEncoderSetIndexBuffer(pass, static_cast<WGPUBuffer>(d.indexBuffer),
                                        WGPUIndexFormat_Uint32, 0, WGPU_WHOLE_SIZE);
    wgpuRenderPassEncoderDrawIndexed(pass, d.indexCount, 1, 0, 0, 0);
  }
  wgpuRenderPassEncoderEnd(pass);
  wgpuRenderPassEncoderRelease(pass);
  // Per-draw textured bind groups: SetBindGroup took its own ref (held by the encoder
  // → command buffer until execution), so releasing ours now is safe.
  for (WGPUBindGroup tb : texBgs)
    wgpuBindGroupRelease(tb);

  const uint32_t stride = alignedBytesPerRow(mWidth);
  WGPUTexelCopyTextureInfo src = {};
  src.texture = static_cast<WGPUTexture>(mTexture);
  src.aspect = WGPUTextureAspect_All;
  WGPUTexelCopyBufferInfo dst = {};
  dst.buffer = static_cast<WGPUBuffer>(mReadbackBuffer);
  dst.layout.bytesPerRow = stride;
  dst.layout.rowsPerImage = mHeight;
  WGPUExtent3D extent = {mWidth, mHeight, 1};
  wgpuCommandEncoderCopyTextureToBuffer(encoder, &src, &dst, &extent);

  WGPUCommandBufferDescriptor cmdDesc = {};
  WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(encoder, &cmdDesc);
  wgpuCommandEncoderRelease(encoder);
  if (!cmd) {
    wgpuBindGroupRelease(bg);
    return false;
  }
  wgpuQueueSubmit(queue, 1, &cmd);
  wgpuCommandBufferRelease(cmd);

  MapCapture cap;
  WGPUBufferMapCallbackInfo mapCb = {};
  mapCb.mode = WGPUCallbackMode_AllowSpontaneous;
  mapCb.callback = onMap;
  mapCb.userdata1 = &cap;
  (void)wgpuBufferMapAsync(static_cast<WGPUBuffer>(mReadbackBuffer),
                            WGPUMapMode_Read, 0, mReadbackBufferSize, mapCb);
  for (int i = 0; i < 1000 && !cap.done; ++i)
    wgpuDevicePoll(device, true, nullptr);
  wgpuBindGroupRelease(bg);
  if (!cap.done || !cap.ok)
    return false;

  const void *mapped = wgpuBufferGetConstMappedRange(static_cast<WGPUBuffer>(mReadbackBuffer), 0,
                                                    mReadbackBufferSize);
  if (!mapped) {
    wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
    return false;
  }
  const uint32_t rowBytes = mWidth * 4u;
  uint8_t *out = static_cast<uint8_t *>(rgba8);
  const uint8_t *in = static_cast<const uint8_t *>(mapped);
  for (uint32_t y = 0; y < mHeight; ++y)
    std::memcpy(out + y * rowBytes, in + y * stride, rowBytes);
  wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
  return true;
#  else
  (void)color;
  (void)viewProj16;
  (void)lightDirAmbient4;
  (void)draws;
  (void)numDraws;
  (void)rgba8;
  return false;
#  endif
#else
  (void)color;
  (void)viewProj16;
  (void)lightDirAmbient4;
  (void)draws;
  (void)numDraws;
  (void)rgba8;
  return false;
#endif
}

void wbWgpuAppendInstanceReport(void *instance, const char *path, long frame) {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (!path || !instance)
    return;
  WGPUGlobalReport rep = {};
  wgpuGenerateReport(static_cast<WGPUInstance>(instance), &rep);
  FILE *fp = std::fopen(path, "a");
  if (!fp)
    return;
  const WGPUHubReport &h = rep.hub;
  // a = ever-allocated registry slots (high-water), k = currently kept-from-user (live),
  // r = released-by-user. A per-frame leak shows a/k climbing without bound for one type.
  auto pr = [fp](const char *tag, const WGPURegistryReport &r) {
    std::fprintf(fp, " %s(a=%" PRIu64 " k=%" PRIu64 " r=%" PRIu64 ")", tag,
                 static_cast<uint64_t>(r.numAllocated), static_cast<uint64_t>(r.numKeptFromUser),
                 static_cast<uint64_t>(r.numReleasedFromUser));
  };
  std::fprintf(fp, "frame=%ld", frame);
  pr("tex", h.textures);
  pr("texView", h.textureViews);
  pr("buf", h.buffers);
  pr("bg", h.bindGroups);
  pr("bgl", h.bindGroupLayouts);
  pr("cmdBuf", h.commandBuffers);
  pr("renderPipe", h.renderPipelines);
  pr("sampler", h.samplers);
  pr("shader", h.shaderModules);
  pr("querySet", h.querySets);
  pr("surf", rep.surfaces);
  std::fprintf(fp, "\n");
  std::fclose(fp);
#  else
  (void)instance;
  (void)path;
  (void)frame;
#  endif
#else
  (void)instance;
  (void)path;
  (void)frame;
#endif
}

void WbWgpuRenderTarget::appendResourceReport(const char *path, long frame) const {
  wbWgpuAppendInstanceReport(mBackend ? mBackend->instance() : nullptr, path, frame);
}

bool WbWgpuRenderTarget::clearAndDrawLines(const WbWgpuClearColor &clearColor,
                                           const float *viewProj16, const float *color4,
                                           const float *lineVerts, uint32_t vertexCount, void *rgba8,
                                           bool loadExisting, bool depthTest) {
  if (!mUsable || !rgba8 || !viewProj16 || !color4 || !lineVerts)
    return false;
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  const bool pipeOk = depthTest ? ensureLinePipeline() : ensureLineNoDepthPipeline();
  if (!pipeOk || vertexCount < 2)
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUQueue queue = static_cast<WGPUQueue>(mBackend->queue());

  // One uniform slot: viewProj + identity model (lineVerts are already world-space) +
  // the flat line colour. Reuses the shared scene uniform buffer + bind-group layout.
  const size_t needed = kScnUniformStride;
  if (needed > mScnUniformBufferSize) {
    if (mScnUniformBuffer)
      wgpuBufferRelease(static_cast<WGPUBuffer>(mScnUniformBuffer));
    WGPUBufferDescriptor ubDesc = {};
    ubDesc.size = needed;
    ubDesc.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
    WGPUBuffer ub = wgpuDeviceCreateBuffer(device, &ubDesc);
    if (!ub)
      return false;
    mScnUniformBuffer = ub;
    mScnUniformBufferSize = needed;
  }
  std::vector<uint8_t> host(needed, 0);
  static const float kIdentity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  std::memcpy(host.data() + 0, viewProj16, 64);
  std::memcpy(host.data() + 64, kIdentity, 64);
  std::memcpy(host.data() + 128, color4, 16);
  wgpuQueueWriteBuffer(queue, static_cast<WGPUBuffer>(mScnUniformBuffer), 0, host.data(), needed);

  // Segment-endpoint vertex buffer (stride 32; normal/uv slots ignored by kSolidPick).
  WGPUBufferDescriptor vbDesc = {};
  vbDesc.size = static_cast<uint64_t>(vertexCount) * 32u;
  vbDesc.usage = WGPUBufferUsage_Vertex | WGPUBufferUsage_CopyDst;
  WGPUBuffer vbuf = wgpuDeviceCreateBuffer(device, &vbDesc);
  if (!vbuf)
    return false;
  wgpuQueueWriteBuffer(queue, vbuf, 0, lineVerts, vbDesc.size);

  WGPUBindGroupEntry bgEntry = {};
  bgEntry.binding = 0;
  bgEntry.buffer = static_cast<WGPUBuffer>(mScnUniformBuffer);
  bgEntry.offset = 0;
  bgEntry.size = 192;
  WGPUBindGroupDescriptor bgDesc = {};
  bgDesc.layout = static_cast<WGPUBindGroupLayout>(mScnBindGroupLayout);
  bgDesc.entryCount = 1;
  bgDesc.entries = &bgEntry;
  WGPUBindGroup bg = wgpuDeviceCreateBindGroup(device, &bgDesc);
  if (!bg) {
    wgpuBufferRelease(vbuf);
    return false;
  }

  WGPUCommandEncoderDescriptor encDesc = {};
  WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(device, &encDesc);
  if (!encoder) {
    wgpuBindGroupRelease(bg);
    wgpuBufferRelease(vbuf);
    return false;
  }

  WGPURenderPassColorAttachment colorAttach = {};
  colorAttach.view = static_cast<WGPUTextureView>(mView);
  colorAttach.loadOp = loadExisting ? WGPULoadOp_Load : WGPULoadOp_Clear;
  colorAttach.storeOp = WGPUStoreOp_Store;
  colorAttach.clearValue = {clearColor.r, clearColor.g, clearColor.b, clearColor.a};
  colorAttach.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;

  WGPURenderPassDepthStencilAttachment depthAttach = {};
  depthAttach.view = static_cast<WGPUTextureView>(mScnDepthView);
  depthAttach.depthLoadOp = loadExisting ? WGPULoadOp_Load : WGPULoadOp_Clear;
  depthAttach.depthStoreOp = WGPUStoreOp_Store;
  depthAttach.depthClearValue = 1.0f;
  depthAttach.depthReadOnly = 0;

  WGPURenderPassDescriptor passDesc = {};
  passDesc.colorAttachmentCount = 1;
  passDesc.colorAttachments = &colorAttach;
  passDesc.depthStencilAttachment = &depthAttach;
  WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &passDesc);
  if (!pass) {
    wgpuCommandEncoderRelease(encoder);
    wgpuBindGroupRelease(bg);
    wgpuBufferRelease(vbuf);
    return false;
  }
  wgpuRenderPassEncoderSetPipeline(
    pass, static_cast<WGPURenderPipeline>(depthTest ? mLinePipeline : mLineNoDepthPipeline));
  const uint32_t dyn = 0;
  wgpuRenderPassEncoderSetBindGroup(pass, 0, bg, 1, &dyn);
  wgpuRenderPassEncoderSetVertexBuffer(pass, 0, vbuf, 0, WGPU_WHOLE_SIZE);
  wgpuRenderPassEncoderDraw(pass, vertexCount, 1, 0, 0);
  wgpuRenderPassEncoderEnd(pass);
  wgpuRenderPassEncoderRelease(pass);

  const uint32_t stride = alignedBytesPerRow(mWidth);
  WGPUTexelCopyTextureInfo src = {};
  src.texture = static_cast<WGPUTexture>(mTexture);
  src.aspect = WGPUTextureAspect_All;
  WGPUTexelCopyBufferInfo dst = {};
  dst.buffer = static_cast<WGPUBuffer>(mReadbackBuffer);
  dst.layout.bytesPerRow = stride;
  dst.layout.rowsPerImage = mHeight;
  WGPUExtent3D extent = {mWidth, mHeight, 1};
  wgpuCommandEncoderCopyTextureToBuffer(encoder, &src, &dst, &extent);

  WGPUCommandBufferDescriptor cmdDesc = {};
  WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(encoder, &cmdDesc);
  wgpuCommandEncoderRelease(encoder);
  if (!cmd) {
    wgpuBindGroupRelease(bg);
    wgpuBufferRelease(vbuf);
    return false;
  }
  wgpuQueueSubmit(queue, 1, &cmd);
  wgpuCommandBufferRelease(cmd);

  MapCapture cap;
  WGPUBufferMapCallbackInfo mapCb = {};
  mapCb.mode = WGPUCallbackMode_AllowSpontaneous;
  mapCb.callback = onMap;
  mapCb.userdata1 = &cap;
  (void)wgpuBufferMapAsync(static_cast<WGPUBuffer>(mReadbackBuffer), WGPUMapMode_Read, 0,
                           mReadbackBufferSize, mapCb);
  for (int i = 0; i < 1000 && !cap.done; ++i)
    wgpuDevicePoll(device, true, nullptr);
  wgpuBindGroupRelease(bg);
  wgpuBufferRelease(vbuf);
  if (!cap.done || !cap.ok)
    return false;
  const void *mapped = wgpuBufferGetConstMappedRange(static_cast<WGPUBuffer>(mReadbackBuffer), 0,
                                                     mReadbackBufferSize);
  if (!mapped) {
    wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
    return false;
  }
  const uint32_t rowBytes = mWidth * 4u;
  uint8_t *out = static_cast<uint8_t *>(rgba8);
  const uint8_t *in = static_cast<const uint8_t *>(mapped);
  for (uint32_t y = 0; y < mHeight; ++y)
    std::memcpy(out + y * rowBytes, in + y * stride, rowBytes);
  wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
  return true;
#  else
  (void)clearColor;
  (void)viewProj16;
  (void)color4;
  (void)lineVerts;
  (void)vertexCount;
  (void)rgba8;
  (void)loadExisting;
  return false;
#  endif
#else
  (void)clearColor;
  (void)viewProj16;
  (void)color4;
  (void)lineVerts;
  (void)vertexCount;
  (void)rgba8;
  (void)loadExisting;
  return false;
#endif
}

// R5b: build the F32 real-meters depth pipeline. Renders into its own
// R32Float color texture; reuses the lit path's bind-group layout + depth
// attachment (ensureScenePipeline runs first). Only the color-target format +
// the kSolidDistanceF32 fragment differ.
bool WbWgpuRenderTarget::ensureSceneDepthF32Pipeline() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (mScnDepthF32Pipeline)
    return true;
  if (!ensureScenePipeline())  // mScnBindGroupLayout + mScnDepthView (shared)
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());

  // R32Float color attachment (+ CopySrc so we can read it back). R32Float is
  // a core-WebGPU renderable + copyable format, so the clip.w → color.r values
  // survive the readback as exact float32 meters.
  WGPUTextureDescriptor texDesc = {};
  texDesc.usage = WGPUTextureUsage_RenderAttachment | WGPUTextureUsage_CopySrc;
  texDesc.dimension = WGPUTextureDimension_2D;
  texDesc.size = {mWidth, mHeight, 1};
  texDesc.format = WGPUTextureFormat_R32Float;
  texDesc.mipLevelCount = 1;
  texDesc.sampleCount = 1;
  WGPUTexture tex = wgpuDeviceCreateTexture(device, &texDesc);
  if (!tex) {
    WbLog::info("[WbWgpuRenderTarget] scn-depth-f32 CreateTexture failed");
    return false;
  }
  mDepthF32Texture = tex;
  WGPUTextureView view = wgpuTextureCreateView(tex, nullptr);
  if (!view) {
    WbLog::info("[WbWgpuRenderTarget] scn-depth-f32 TextureCreateView failed");
    return false;
  }
  mDepthF32View = view;

  WGPUShaderSourceWGSL wgsl = {};
  wgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
  wgsl.code.data = WbWgpuShaders::kSolidDistanceF32;
  wgsl.code.length = WGPU_STRLEN;
  WGPUShaderModuleDescriptor smDesc = {};
  smDesc.nextInChain = &wgsl.chain;
  WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &smDesc);
  if (!sm) {
    WbLog::info("[WbWgpuRenderTarget] scn-depth-f32 CreateShaderModule failed");
    return false;
  }
  mScnDepthF32ShaderModule = sm;

  WGPUBindGroupLayout bgl = static_cast<WGPUBindGroupLayout>(mScnBindGroupLayout);
  WGPUPipelineLayoutDescriptor plDesc = {};
  plDesc.bindGroupLayoutCount = 1;
  plDesc.bindGroupLayouts = &bgl;
  WGPUPipelineLayout plLayout = wgpuDeviceCreatePipelineLayout(device, &plDesc);
  if (!plLayout) {
    WbLog::info("[WbWgpuRenderTarget] scn-depth-f32 CreatePipelineLayout failed");
    return false;
  }

  WGPUVertexAttribute attrs[3] = {};
  attrs[0].format = WGPUVertexFormat_Float32x3; attrs[0].offset = 0;  attrs[0].shaderLocation = 0;
  attrs[1].format = WGPUVertexFormat_Float32x3; attrs[1].offset = 12; attrs[1].shaderLocation = 1;
  attrs[2].format = WGPUVertexFormat_Float32x2; attrs[2].offset = 24; attrs[2].shaderLocation = 2;
  WGPUVertexBufferLayout vbLayout = {};
  vbLayout.arrayStride = 32;
  vbLayout.stepMode = WGPUVertexStepMode_Vertex;
  vbLayout.attributeCount = 3;
  vbLayout.attributes = attrs;

  WGPUColorTargetState colorTarget = {};
  colorTarget.format = WGPUTextureFormat_R32Float;
  colorTarget.writeMask = WGPUColorWriteMask_All;

  WGPUFragmentState fs = {};
  fs.module = sm;
  fs.entryPoint.data = "fs_main";
  fs.entryPoint.length = WGPU_STRLEN;
  fs.targetCount = 1;
  fs.targets = &colorTarget;

  WGPUDepthStencilState depthState = {};
  depthState.format = WGPUTextureFormat_Depth24Plus;
  depthState.depthWriteEnabled = WGPUOptionalBool_True;
  depthState.depthCompare = WGPUCompareFunction_Less;
  depthState.stencilFront.compare = WGPUCompareFunction_Always;
  depthState.stencilFront.failOp = WGPUStencilOperation_Keep;
  depthState.stencilFront.depthFailOp = WGPUStencilOperation_Keep;
  depthState.stencilFront.passOp = WGPUStencilOperation_Keep;
  depthState.stencilBack = depthState.stencilFront;
  depthState.stencilReadMask = 0xFFFFFFFFu;
  depthState.stencilWriteMask = 0xFFFFFFFFu;

  WGPURenderPipelineDescriptor pipeDesc = {};
  pipeDesc.layout = plLayout;
  pipeDesc.vertex.module = sm;
  pipeDesc.vertex.entryPoint.data = "vs_main";
  pipeDesc.vertex.entryPoint.length = WGPU_STRLEN;
  pipeDesc.vertex.bufferCount = 1;
  pipeDesc.vertex.buffers = &vbLayout;
  pipeDesc.primitive.topology = WGPUPrimitiveTopology_TriangleList;
  pipeDesc.primitive.frontFace = WGPUFrontFace_CCW;
  pipeDesc.primitive.cullMode = WGPUCullMode_None;
  pipeDesc.depthStencil = &depthState;
  pipeDesc.multisample.count = 1;
  pipeDesc.multisample.mask = 0xFFFFFFFFu;
  pipeDesc.fragment = &fs;
  WGPURenderPipeline pipe = wgpuDeviceCreateRenderPipeline(device, &pipeDesc);
  wgpuPipelineLayoutRelease(plLayout);
  if (!pipe) {
    WbLog::info("[WbWgpuRenderTarget] scn-depth-f32 CreateRenderPipeline failed");
    return false;
  }
  mScnDepthF32Pipeline = pipe;
  return true;
#  else
  return false;
#  endif
#else
  return false;
#endif
}

// T1.2 CSM: build the clip.z (NDC depth) pipeline for shadow-map depth. Reuses
// the R32Float target + bind-group layout + depth attachment that
// ensureSceneDepthF32Pipeline creates; only the fragment shader differs
// (kSolidClipDepthF32 writes clip.z, which — unlike clip.w — is well-defined
// under the orthographic light projection a shadow map uses).
bool WbWgpuRenderTarget::ensureSceneClipDepthF32Pipeline() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (mScnClipDepthPipeline)
    return true;
  if (!ensureSceneDepthF32Pipeline())  // creates the R32Float target + bind-group layout
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());

  WGPUShaderSourceWGSL wgsl = {};
  wgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
  wgsl.code.data = WbWgpuShaders::kSolidClipDepthF32;
  wgsl.code.length = WGPU_STRLEN;
  WGPUShaderModuleDescriptor smDesc = {};
  smDesc.nextInChain = &wgsl.chain;
  WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &smDesc);
  if (!sm) {
    WbLog::info("[WbWgpuRenderTarget] scn-clipdepth-f32 CreateShaderModule failed");
    return false;
  }
  mScnClipDepthShaderModule = sm;

  WGPUBindGroupLayout bgl = static_cast<WGPUBindGroupLayout>(mScnBindGroupLayout);
  WGPUPipelineLayoutDescriptor plDesc = {};
  plDesc.bindGroupLayoutCount = 1;
  plDesc.bindGroupLayouts = &bgl;
  WGPUPipelineLayout plLayout = wgpuDeviceCreatePipelineLayout(device, &plDesc);
  if (!plLayout) {
    WbLog::info("[WbWgpuRenderTarget] scn-clipdepth-f32 CreatePipelineLayout failed");
    return false;
  }

  WGPUVertexAttribute attrs[3] = {};
  attrs[0].format = WGPUVertexFormat_Float32x3; attrs[0].offset = 0;  attrs[0].shaderLocation = 0;
  attrs[1].format = WGPUVertexFormat_Float32x3; attrs[1].offset = 12; attrs[1].shaderLocation = 1;
  attrs[2].format = WGPUVertexFormat_Float32x2; attrs[2].offset = 24; attrs[2].shaderLocation = 2;
  WGPUVertexBufferLayout vbLayout = {};
  vbLayout.arrayStride = 32;
  vbLayout.stepMode = WGPUVertexStepMode_Vertex;
  vbLayout.attributeCount = 3;
  vbLayout.attributes = attrs;

  WGPUColorTargetState colorTarget = {};
  colorTarget.format = WGPUTextureFormat_R32Float;
  colorTarget.writeMask = WGPUColorWriteMask_All;

  WGPUFragmentState fs = {};
  fs.module = sm;
  fs.entryPoint.data = "fs_main";
  fs.entryPoint.length = WGPU_STRLEN;
  fs.targetCount = 1;
  fs.targets = &colorTarget;

  WGPUDepthStencilState depthState = {};
  depthState.format = WGPUTextureFormat_Depth24Plus;
  depthState.depthWriteEnabled = WGPUOptionalBool_True;
  depthState.depthCompare = WGPUCompareFunction_Less;
  depthState.stencilFront.compare = WGPUCompareFunction_Always;
  depthState.stencilFront.failOp = WGPUStencilOperation_Keep;
  depthState.stencilFront.depthFailOp = WGPUStencilOperation_Keep;
  depthState.stencilFront.passOp = WGPUStencilOperation_Keep;
  depthState.stencilBack = depthState.stencilFront;
  depthState.stencilReadMask = 0xFFFFFFFFu;
  depthState.stencilWriteMask = 0xFFFFFFFFu;

  WGPURenderPipelineDescriptor pipeDesc = {};
  pipeDesc.layout = plLayout;
  pipeDesc.vertex.module = sm;
  pipeDesc.vertex.entryPoint.data = "vs_main";
  pipeDesc.vertex.entryPoint.length = WGPU_STRLEN;
  pipeDesc.vertex.bufferCount = 1;
  pipeDesc.vertex.buffers = &vbLayout;
  pipeDesc.primitive.topology = WGPUPrimitiveTopology_TriangleList;
  pipeDesc.primitive.frontFace = WGPUFrontFace_CCW;
  pipeDesc.primitive.cullMode = WGPUCullMode_None;
  pipeDesc.depthStencil = &depthState;
  pipeDesc.multisample.count = 1;
  pipeDesc.multisample.mask = 0xFFFFFFFFu;
  pipeDesc.fragment = &fs;
  WGPURenderPipeline pipe = wgpuDeviceCreateRenderPipeline(device, &pipeDesc);
  wgpuPipelineLayoutRelease(plLayout);
  if (!pipe) {
    WbLog::info("[WbWgpuRenderTarget] scn-clipdepth-f32 CreateRenderPipeline failed");
    return false;
  }
  mScnClipDepthPipeline = pipe;
  // Reversed-Z variant for the camera-depth SSAO prepass (Greater + Depth32Float).
  // Reversed-Z variant (Greater + Depth32Float) for the camera-depth SSAO prepass. NOTE: the
  // standard pipeline's layout has ALREADY been released at this point — reusing pipeDesc.layout
  // here was a use-after-free that panicked wgpu-native at startup (bisect-confirmed). Build a
  // fresh pipeline layout from the still-alive bind group layout.
  {
    WGPUBindGroupLayout bglR = static_cast<WGPUBindGroupLayout>(mScnBindGroupLayout);
    WGPUPipelineLayoutDescriptor plDescR = {};
    plDescR.bindGroupLayoutCount = 1;
    plDescR.bindGroupLayouts = &bglR;
    WGPUPipelineLayout plR = wgpuDeviceCreatePipelineLayout(device, &plDescR);
    if (plR) {
      WGPUDepthStencilState dsR = depthState;
      dsR.format = WGPUTextureFormat_Depth32Float;
      dsR.depthCompare = WGPUCompareFunction_Greater;
      pipeDesc.layout = plR;
      pipeDesc.depthStencil = &dsR;
      mScnClipDepthPipelineRev = wgpuDeviceCreateRenderPipeline(device, &pipeDesc);
      wgpuPipelineLayoutRelease(plR);
    }
  }
  return true;
#  else
  return false;
#  endif
#else
  return false;
#endif
}

bool WbWgpuRenderTarget::clearAndDrawSceneDepthF32(float farClear, const float *viewProj16,
                                                   const WbWgpuSolidDraw *draws, uint32_t numDraws,
                                                   float *outMeters, bool clipDepth) {
  if (!mUsable || !outMeters || !viewProj16)
    return false;
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (!ensureSceneDepthF32Pipeline())
    return false;
  // T1.2 CSM: clipDepth selects the clip.z (NDC depth) pipeline for shadow-map
  // depth; default false keeps the clip.w metric-depth (RangeFinder) behaviour.
  if (clipDepth && !ensureSceneClipDepthF32Pipeline())
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUQueue queue = static_cast<WGPUQueue>(mBackend->queue());

  // Reuse the scene uniform buffer (same Scene-struct layout as the lit path;
  // the F32 shader only reads viewProj + model, but we keep the full 256-byte
  // stride so the dynamic-offset bind group is identical).
  const uint32_t slotCount = numDraws == 0 ? 1u : numDraws;
  const size_t needed = static_cast<size_t>(slotCount) * kScnUniformStride;
  if (needed > mScnUniformBufferSize) {
    if (mScnUniformBuffer)
      wgpuBufferRelease(static_cast<WGPUBuffer>(mScnUniformBuffer));
    WGPUBufferDescriptor ubDesc = {};
    ubDesc.size = needed;
    ubDesc.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
    ubDesc.mappedAtCreation = false;
    WGPUBuffer ub = wgpuDeviceCreateBuffer(device, &ubDesc);
    if (!ub)
      return false;
    mScnUniformBuffer = ub;
    mScnUniformBufferSize = needed;
  }

  std::vector<uint8_t> hostBuf(needed, 0);
  for (uint32_t i = 0; i < numDraws; ++i) {
    uint8_t *slot = hostBuf.data() + static_cast<size_t>(i) * kScnUniformStride;
    std::memcpy(slot + 0, viewProj16, 64);
    if (draws[i].modelMatrix16)
      std::memcpy(slot + 64, draws[i].modelMatrix16, 64);
    // baseColor (128), light (144), pad0/pad1 (160..255) stay zero — the F32
    // distance shader ignores them.
  }
  wgpuQueueWriteBuffer(queue, static_cast<WGPUBuffer>(mScnUniformBuffer), 0, hostBuf.data(),
                       needed);

  WGPUBindGroupEntry bgEntry = {};
  bgEntry.binding = 0;
  bgEntry.buffer = static_cast<WGPUBuffer>(mScnUniformBuffer);
  bgEntry.offset = 0;
  bgEntry.size = 192;
  WGPUBindGroupDescriptor bgDesc = {};
  bgDesc.layout = static_cast<WGPUBindGroupLayout>(mScnBindGroupLayout);
  bgDesc.entryCount = 1;
  bgDesc.entries = &bgEntry;
  WGPUBindGroup bg = wgpuDeviceCreateBindGroup(device, &bgDesc);
  if (!bg)
    return false;

  WGPUCommandEncoderDescriptor encDesc = {};
  WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(device, &encDesc);
  if (!encoder) {
    wgpuBindGroupRelease(bg);
    return false;
  }

  WGPURenderPassColorAttachment colorAttach = {};
  colorAttach.view = static_cast<WGPUTextureView>(mDepthF32View);
  colorAttach.loadOp = WGPULoadOp_Clear;
  colorAttach.storeOp = WGPUStoreOp_Store;
  // R32Float target: only .r is stored. Pixels with no geometry read farClear
  // (the RangeFinder "nothing hit" / maxRange value).
  colorAttach.clearValue = {farClear, 0.0, 0.0, 1.0};
  colorAttach.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;

  WGPURenderPassDepthStencilAttachment depthAttach = {};
  depthAttach.view = static_cast<WGPUTextureView>(mScnDepthView);
  depthAttach.depthLoadOp = WGPULoadOp_Clear;
  depthAttach.depthStoreOp = WGPUStoreOp_Store;
  depthAttach.depthClearValue = 1.0f;
  depthAttach.depthReadOnly = 0;

  WGPURenderPassDescriptor passDesc = {};
  passDesc.colorAttachmentCount = 1;
  passDesc.colorAttachments = &colorAttach;
  passDesc.depthStencilAttachment = &depthAttach;
  WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &passDesc);
  if (!pass) {
    wgpuCommandEncoderRelease(encoder);
    wgpuBindGroupRelease(bg);
    return false;
  }
  // T1.2 CSM: select the clip.z pipeline when clipDepth (shadow-map depth);
  // otherwise the clip.w F32 metric-depth pipeline. (This selection was the
  // sub-step-2b bug: it had been left hardcoded to mScnDepthF32Pipeline, so the
  // clipDepth path rendered clip.w — ≡1 under the ortho light proj — giving the
  // 1.0 readback while the projection math was actually correct.)
  wgpuRenderPassEncoderSetPipeline(
      pass, static_cast<WGPURenderPipeline>(clipDepth ? mScnClipDepthPipeline
                                                      : mScnDepthF32Pipeline));

  for (uint32_t i = 0; i < numDraws; ++i) {
    const WbWgpuSolidDraw &d = draws[i];
    if (!d.vertexBuffer || !d.indexBuffer || d.indexCount == 0)
      continue;
    const uint32_t dynOffset = i * kScnUniformStride;
    wgpuRenderPassEncoderSetBindGroup(pass, 0, bg, 1, &dynOffset);
    wgpuRenderPassEncoderSetVertexBuffer(pass, 0, static_cast<WGPUBuffer>(d.vertexBuffer), 0,
                                         WGPU_WHOLE_SIZE);
    wgpuRenderPassEncoderSetIndexBuffer(pass, static_cast<WGPUBuffer>(d.indexBuffer),
                                        WGPUIndexFormat_Uint32, 0, WGPU_WHOLE_SIZE);
    wgpuRenderPassEncoderDrawIndexed(pass, d.indexCount, 1, 0, 0, 0);
  }
  wgpuRenderPassEncoderEnd(pass);
  wgpuRenderPassEncoderRelease(pass);

  // R32Float is 4 B/pixel — same row stride as RGBA8, so the existing readback
  // buffer + alignment math apply unchanged. Copy from the F32 color texture.
  const uint32_t stride = alignedBytesPerRow(mWidth);
  WGPUTexelCopyTextureInfo src = {};
  src.texture = static_cast<WGPUTexture>(mDepthF32Texture);
  src.aspect = WGPUTextureAspect_All;
  WGPUTexelCopyBufferInfo dst = {};
  dst.buffer = static_cast<WGPUBuffer>(mReadbackBuffer);
  dst.layout.bytesPerRow = stride;
  dst.layout.rowsPerImage = mHeight;
  WGPUExtent3D extent = {mWidth, mHeight, 1};
  wgpuCommandEncoderCopyTextureToBuffer(encoder, &src, &dst, &extent);

  WGPUCommandBufferDescriptor cmdDesc = {};
  WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(encoder, &cmdDesc);
  wgpuCommandEncoderRelease(encoder);
  if (!cmd) {
    wgpuBindGroupRelease(bg);
    return false;
  }
  wgpuQueueSubmit(queue, 1, &cmd);
  wgpuCommandBufferRelease(cmd);

  MapCapture cap;
  WGPUBufferMapCallbackInfo mapCb = {};
  mapCb.mode = WGPUCallbackMode_AllowSpontaneous;
  mapCb.callback = onMap;
  mapCb.userdata1 = &cap;
  (void)wgpuBufferMapAsync(static_cast<WGPUBuffer>(mReadbackBuffer),
                            WGPUMapMode_Read, 0, mReadbackBufferSize, mapCb);
  for (int i = 0; i < 1000 && !cap.done; ++i)
    wgpuDevicePoll(device, true, nullptr);
  wgpuBindGroupRelease(bg);
  if (!cap.done || !cap.ok)
    return false;

  const void *mapped = wgpuBufferGetConstMappedRange(static_cast<WGPUBuffer>(mReadbackBuffer), 0,
                                                    mReadbackBufferSize);
  if (!mapped) {
    wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
    return false;
  }
  // Each row is mWidth float32s; the GPU buffer row is `stride` bytes (padded).
  const uint32_t rowFloatBytes = mWidth * 4u;
  uint8_t *out = reinterpret_cast<uint8_t *>(outMeters);
  const uint8_t *in = static_cast<const uint8_t *>(mapped);
  for (uint32_t y = 0; y < mHeight; ++y)
    std::memcpy(out + y * rowFloatBytes, in + y * stride, rowFloatBytes);
  wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
  return true;
#  else
  (void)farClear;
  (void)viewProj16;
  (void)draws;
  (void)numDraws;
  (void)outMeters;
  return false;
#  endif
#else
  (void)farClear;
  (void)viewProj16;
  (void)draws;
  (void)numDraws;
  (void)outMeters;
  return false;
#endif
}

// R5d: build the Lidar radial-range pipeline. Reuses everything the F32 depth
// pipeline set up (bind-group layout, depth attachment, R32Float color target);
// only the kSolidRangeF32 shader + pipeline differ.
bool WbWgpuRenderTarget::ensureSceneRangeF32Pipeline() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (mScnRangeF32Pipeline)
    return true;
  if (!ensureSceneDepthF32Pipeline())  // mScnBindGroupLayout + mScnDepthView + mDepthF32Texture
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());

  WGPUShaderSourceWGSL wgsl = {};
  wgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
  wgsl.code.data = WbWgpuShaders::kSolidRangeF32;
  wgsl.code.length = WGPU_STRLEN;
  WGPUShaderModuleDescriptor smDesc = {};
  smDesc.nextInChain = &wgsl.chain;
  WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &smDesc);
  if (!sm) {
    WbLog::info("[WbWgpuRenderTarget] scn-range-f32 CreateShaderModule failed");
    return false;
  }
  mScnRangeF32ShaderModule = sm;

  WGPUBindGroupLayout bgl = static_cast<WGPUBindGroupLayout>(mScnBindGroupLayout);
  WGPUPipelineLayoutDescriptor plDesc = {};
  plDesc.bindGroupLayoutCount = 1;
  plDesc.bindGroupLayouts = &bgl;
  WGPUPipelineLayout plLayout = wgpuDeviceCreatePipelineLayout(device, &plDesc);
  if (!plLayout) {
    WbLog::info("[WbWgpuRenderTarget] scn-range-f32 CreatePipelineLayout failed");
    return false;
  }

  WGPUVertexAttribute attrs[3] = {};
  attrs[0].format = WGPUVertexFormat_Float32x3; attrs[0].offset = 0;  attrs[0].shaderLocation = 0;
  attrs[1].format = WGPUVertexFormat_Float32x3; attrs[1].offset = 12; attrs[1].shaderLocation = 1;
  attrs[2].format = WGPUVertexFormat_Float32x2; attrs[2].offset = 24; attrs[2].shaderLocation = 2;
  WGPUVertexBufferLayout vbLayout = {};
  vbLayout.arrayStride = 32;
  vbLayout.stepMode = WGPUVertexStepMode_Vertex;
  vbLayout.attributeCount = 3;
  vbLayout.attributes = attrs;

  WGPUColorTargetState colorTarget = {};
  colorTarget.format = WGPUTextureFormat_R32Float;
  colorTarget.writeMask = WGPUColorWriteMask_All;

  WGPUFragmentState fs = {};
  fs.module = sm;
  fs.entryPoint.data = "fs_main";
  fs.entryPoint.length = WGPU_STRLEN;
  fs.targetCount = 1;
  fs.targets = &colorTarget;

  WGPUDepthStencilState depthState = {};
  depthState.format = WGPUTextureFormat_Depth24Plus;
  depthState.depthWriteEnabled = WGPUOptionalBool_True;
  depthState.depthCompare = WGPUCompareFunction_Less;
  depthState.stencilFront.compare = WGPUCompareFunction_Always;
  depthState.stencilFront.failOp = WGPUStencilOperation_Keep;
  depthState.stencilFront.depthFailOp = WGPUStencilOperation_Keep;
  depthState.stencilFront.passOp = WGPUStencilOperation_Keep;
  depthState.stencilBack = depthState.stencilFront;
  depthState.stencilReadMask = 0xFFFFFFFFu;
  depthState.stencilWriteMask = 0xFFFFFFFFu;

  WGPURenderPipelineDescriptor pipeDesc = {};
  pipeDesc.layout = plLayout;
  pipeDesc.vertex.module = sm;
  pipeDesc.vertex.entryPoint.data = "vs_main";
  pipeDesc.vertex.entryPoint.length = WGPU_STRLEN;
  pipeDesc.vertex.bufferCount = 1;
  pipeDesc.vertex.buffers = &vbLayout;
  pipeDesc.primitive.topology = WGPUPrimitiveTopology_TriangleList;
  pipeDesc.primitive.frontFace = WGPUFrontFace_CCW;
  pipeDesc.primitive.cullMode = WGPUCullMode_None;
  pipeDesc.depthStencil = &depthState;
  pipeDesc.multisample.count = 1;
  pipeDesc.multisample.mask = 0xFFFFFFFFu;
  pipeDesc.fragment = &fs;
  WGPURenderPipeline pipe = wgpuDeviceCreateRenderPipeline(device, &pipeDesc);
  wgpuPipelineLayoutRelease(plLayout);
  if (!pipe) {
    WbLog::info("[WbWgpuRenderTarget] scn-range-f32 CreateRenderPipeline failed");
    return false;
  }
  mScnRangeF32Pipeline = pipe;
  return true;
#  else
  return false;
#  endif
#else
  return false;
#endif
}

bool WbWgpuRenderTarget::clearAndDrawSceneRangeF32(float farClear, const float *viewProj16,
                                                   const float *view16,
                                                   const WbWgpuSolidDraw *draws, uint32_t numDraws,
                                                   float *outMeters) {
  if (!mUsable || !outMeters || !viewProj16 || !view16)
    return false;
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (!ensureSceneRangeF32Pipeline())
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUQueue queue = static_cast<WGPUQueue>(mBackend->queue());

  // Range uniform per slot: viewProj(0) + view(64) + model(128) = 192 B; same
  // 256-byte stride + minBindingSize as the Scene struct, so the scene
  // bind-group layout applies unchanged.
  const uint32_t slotCount = numDraws == 0 ? 1u : numDraws;
  const size_t needed = static_cast<size_t>(slotCount) * kScnUniformStride;
  if (needed > mScnUniformBufferSize) {
    if (mScnUniformBuffer)
      wgpuBufferRelease(static_cast<WGPUBuffer>(mScnUniformBuffer));
    WGPUBufferDescriptor ubDesc = {};
    ubDesc.size = needed;
    ubDesc.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
    ubDesc.mappedAtCreation = false;
    WGPUBuffer ub = wgpuDeviceCreateBuffer(device, &ubDesc);
    if (!ub)
      return false;
    mScnUniformBuffer = ub;
    mScnUniformBufferSize = needed;
  }

  std::vector<uint8_t> hostBuf(needed, 0);
  for (uint32_t i = 0; i < numDraws; ++i) {
    uint8_t *slot = hostBuf.data() + static_cast<size_t>(i) * kScnUniformStride;
    std::memcpy(slot + 0, viewProj16, 64);
    std::memcpy(slot + 64, view16, 64);
    if (draws[i].modelMatrix16)
      std::memcpy(slot + 128, draws[i].modelMatrix16, 64);
  }
  wgpuQueueWriteBuffer(queue, static_cast<WGPUBuffer>(mScnUniformBuffer), 0, hostBuf.data(),
                       needed);

  WGPUBindGroupEntry bgEntry = {};
  bgEntry.binding = 0;
  bgEntry.buffer = static_cast<WGPUBuffer>(mScnUniformBuffer);
  bgEntry.offset = 0;
  bgEntry.size = 192;
  WGPUBindGroupDescriptor bgDesc = {};
  bgDesc.layout = static_cast<WGPUBindGroupLayout>(mScnBindGroupLayout);
  bgDesc.entryCount = 1;
  bgDesc.entries = &bgEntry;
  WGPUBindGroup bg = wgpuDeviceCreateBindGroup(device, &bgDesc);
  if (!bg)
    return false;

  WGPUCommandEncoderDescriptor encDesc = {};
  WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(device, &encDesc);
  if (!encoder) {
    wgpuBindGroupRelease(bg);
    return false;
  }

  WGPURenderPassColorAttachment colorAttach = {};
  colorAttach.view = static_cast<WGPUTextureView>(mDepthF32View);
  colorAttach.loadOp = WGPULoadOp_Clear;
  colorAttach.storeOp = WGPUStoreOp_Store;
  colorAttach.clearValue = {farClear, 0.0, 0.0, 1.0};
  colorAttach.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;

  WGPURenderPassDepthStencilAttachment depthAttach = {};
  depthAttach.view = static_cast<WGPUTextureView>(mScnDepthView);
  depthAttach.depthLoadOp = WGPULoadOp_Clear;
  depthAttach.depthStoreOp = WGPUStoreOp_Store;
  depthAttach.depthClearValue = 1.0f;
  depthAttach.depthReadOnly = 0;

  WGPURenderPassDescriptor passDesc = {};
  passDesc.colorAttachmentCount = 1;
  passDesc.colorAttachments = &colorAttach;
  passDesc.depthStencilAttachment = &depthAttach;
  WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &passDesc);
  if (!pass) {
    wgpuCommandEncoderRelease(encoder);
    wgpuBindGroupRelease(bg);
    return false;
  }
  wgpuRenderPassEncoderSetPipeline(pass, static_cast<WGPURenderPipeline>(mScnRangeF32Pipeline));

  for (uint32_t i = 0; i < numDraws; ++i) {
    const WbWgpuSolidDraw &d = draws[i];
    if (!d.vertexBuffer || !d.indexBuffer || d.indexCount == 0)
      continue;
    const uint32_t dynOffset = i * kScnUniformStride;
    wgpuRenderPassEncoderSetBindGroup(pass, 0, bg, 1, &dynOffset);
    wgpuRenderPassEncoderSetVertexBuffer(pass, 0, static_cast<WGPUBuffer>(d.vertexBuffer), 0,
                                         WGPU_WHOLE_SIZE);
    wgpuRenderPassEncoderSetIndexBuffer(pass, static_cast<WGPUBuffer>(d.indexBuffer),
                                        WGPUIndexFormat_Uint32, 0, WGPU_WHOLE_SIZE);
    wgpuRenderPassEncoderDrawIndexed(pass, d.indexCount, 1, 0, 0, 0);
  }
  wgpuRenderPassEncoderEnd(pass);
  wgpuRenderPassEncoderRelease(pass);

  const uint32_t stride = alignedBytesPerRow(mWidth);
  WGPUTexelCopyTextureInfo src = {};
  src.texture = static_cast<WGPUTexture>(mDepthF32Texture);
  src.aspect = WGPUTextureAspect_All;
  WGPUTexelCopyBufferInfo dst = {};
  dst.buffer = static_cast<WGPUBuffer>(mReadbackBuffer);
  dst.layout.bytesPerRow = stride;
  dst.layout.rowsPerImage = mHeight;
  WGPUExtent3D extent = {mWidth, mHeight, 1};
  wgpuCommandEncoderCopyTextureToBuffer(encoder, &src, &dst, &extent);

  WGPUCommandBufferDescriptor cmdDesc = {};
  WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(encoder, &cmdDesc);
  wgpuCommandEncoderRelease(encoder);
  if (!cmd) {
    wgpuBindGroupRelease(bg);
    return false;
  }
  wgpuQueueSubmit(queue, 1, &cmd);
  wgpuCommandBufferRelease(cmd);

  MapCapture cap;
  WGPUBufferMapCallbackInfo mapCb = {};
  mapCb.mode = WGPUCallbackMode_AllowSpontaneous;
  mapCb.callback = onMap;
  mapCb.userdata1 = &cap;
  (void)wgpuBufferMapAsync(static_cast<WGPUBuffer>(mReadbackBuffer),
                            WGPUMapMode_Read, 0, mReadbackBufferSize, mapCb);
  for (int i = 0; i < 1000 && !cap.done; ++i)
    wgpuDevicePoll(device, true, nullptr);
  wgpuBindGroupRelease(bg);
  if (!cap.done || !cap.ok)
    return false;

  const void *mapped = wgpuBufferGetConstMappedRange(static_cast<WGPUBuffer>(mReadbackBuffer), 0,
                                                    mReadbackBufferSize);
  if (!mapped) {
    wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
    return false;
  }
  const uint32_t rowFloatBytes = mWidth * 4u;
  uint8_t *out = reinterpret_cast<uint8_t *>(outMeters);
  const uint8_t *in = static_cast<const uint8_t *>(mapped);
  for (uint32_t y = 0; y < mHeight; ++y)
    std::memcpy(out + y * rowFloatBytes, in + y * stride, rowFloatBytes);
  wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
  return true;
#  else
  (void)farClear;
  (void)viewProj16;
  (void)view16;
  (void)draws;
  (void)numDraws;
  (void)outMeters;
  return false;
#  endif
#else
  (void)farClear;
  (void)viewProj16;
  (void)view16;
  (void)draws;
  (void)numDraws;
  (void)outMeters;
  return false;
#endif
}

bool WbWgpuRenderTarget::ensureTrianglePipeline() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (mTrianglePipeline)
    return true;
  if (!mUsable)
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());

  // 1. Compile the WGSL shader module.
  WGPUShaderSourceWGSL wgsl = {};
  wgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
  wgsl.code.data = WbWgpuShaders::kTriangleClipSpace;
  wgsl.code.length = WGPU_STRLEN;  // null-terminated
  WGPUShaderModuleDescriptor smDesc = {};
  smDesc.nextInChain = &wgsl.chain;
  WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &smDesc);
  if (!sm) {
    WbLog::info("[WbWgpuRenderTarget] CreateShaderModule failed");
    return false;
  }
  mTriangleShaderModule = sm;

  // 2. Build the render pipeline. No vertex buffers (positions baked
  // into the shader via vertex_index), no bind groups, RGBA8 single
  // color attachment matching mTexture's format, default depth-less.
  WGPUColorTargetState colorTarget = {};
  colorTarget.format = WGPUTextureFormat_RGBA8Unorm;
  colorTarget.writeMask = WGPUColorWriteMask_All;
  // No blend state -> opaque writes (default).

  WGPUFragmentState fs = {};
  fs.module = sm;
  fs.entryPoint.data = "fs_main";
  fs.entryPoint.length = WGPU_STRLEN;
  fs.targetCount = 1;
  fs.targets = &colorTarget;

  WGPURenderPipelineDescriptor pipeDesc = {};
  pipeDesc.layout = nullptr;  // auto-layout (no bind groups in R3.4 step 1)
  pipeDesc.vertex.module = sm;
  pipeDesc.vertex.entryPoint.data = "vs_main";
  pipeDesc.vertex.entryPoint.length = WGPU_STRLEN;
  pipeDesc.vertex.bufferCount = 0;
  pipeDesc.primitive.topology = WGPUPrimitiveTopology_TriangleList;
  pipeDesc.primitive.frontFace = WGPUFrontFace_CCW;
  pipeDesc.primitive.cullMode = WGPUCullMode_None;
  pipeDesc.multisample.count = 1;
  pipeDesc.multisample.mask = 0xFFFFFFFFu;
  pipeDesc.fragment = &fs;

  WGPURenderPipeline pipe = wgpuDeviceCreateRenderPipeline(device, &pipeDesc);
  if (!pipe) {
    WbLog::info("[WbWgpuRenderTarget] CreateRenderPipeline failed");
    return false;
  }
  mTrianglePipeline = pipe;
  return true;
#  else
  return false;
#  endif
#else
  return false;
#endif
}

bool WbWgpuRenderTarget::ensureMeshPipeline() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (mMeshPipeline)
    return true;
  if (!mUsable)
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());

  WGPUShaderSourceWGSL wgsl = {};
  wgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
  wgsl.code.data = WbWgpuShaders::kTriangleVertexBuffer;
  wgsl.code.length = WGPU_STRLEN;
  WGPUShaderModuleDescriptor smDesc = {};
  smDesc.nextInChain = &wgsl.chain;
  WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &smDesc);
  if (!sm) {
    WbLog::info("[WbWgpuRenderTarget] mesh-pipeline CreateShaderModule failed");
    return false;
  }
  mMeshShaderModule = sm;

  // Vertex buffer layout: pos3(float32x3) at offset 0,
  // norm3(float32x3) at offset 12, uv2(float32x2) at offset 24,
  // stride 32. Locations match the WGSL @location decorations.
  WGPUVertexAttribute attrs[3] = {};
  attrs[0].format = WGPUVertexFormat_Float32x3;
  attrs[0].offset = 0;
  attrs[0].shaderLocation = 0;
  attrs[1].format = WGPUVertexFormat_Float32x3;
  attrs[1].offset = 12;
  attrs[1].shaderLocation = 1;
  attrs[2].format = WGPUVertexFormat_Float32x2;
  attrs[2].offset = 24;
  attrs[2].shaderLocation = 2;
  WGPUVertexBufferLayout vbLayout = {};
  vbLayout.arrayStride = 32;
  vbLayout.stepMode = WGPUVertexStepMode_Vertex;
  vbLayout.attributeCount = 3;
  vbLayout.attributes = attrs;

  WGPUColorTargetState colorTarget = {};
  colorTarget.format = WGPUTextureFormat_RGBA8Unorm;
  colorTarget.writeMask = WGPUColorWriteMask_All;

  WGPUFragmentState fs = {};
  fs.module = sm;
  fs.entryPoint.data = "fs_main";
  fs.entryPoint.length = WGPU_STRLEN;
  fs.targetCount = 1;
  fs.targets = &colorTarget;

  WGPURenderPipelineDescriptor pipeDesc = {};
  pipeDesc.layout = nullptr;
  pipeDesc.vertex.module = sm;
  pipeDesc.vertex.entryPoint.data = "vs_main";
  pipeDesc.vertex.entryPoint.length = WGPU_STRLEN;
  pipeDesc.vertex.bufferCount = 1;
  pipeDesc.vertex.buffers = &vbLayout;
  pipeDesc.primitive.topology = WGPUPrimitiveTopology_TriangleList;
  pipeDesc.primitive.frontFace = WGPUFrontFace_CCW;
  pipeDesc.primitive.cullMode = WGPUCullMode_None;
  pipeDesc.multisample.count = 1;
  pipeDesc.multisample.mask = 0xFFFFFFFFu;
  pipeDesc.fragment = &fs;

  WGPURenderPipeline pipe = wgpuDeviceCreateRenderPipeline(device, &pipeDesc);
  if (!pipe) {
    WbLog::info("[WbWgpuRenderTarget] mesh-pipeline CreateRenderPipeline failed");
    return false;
  }
  mMeshPipeline = pipe;
  return true;
#  else
  return false;
#  endif
#else
  return false;
#endif
}

bool WbWgpuRenderTarget::clearAndDrawMesh(const WbWgpuClearColor &color, void *vertexBuffer,
                                          void *indexBuffer, uint32_t indexCount, void *rgba8) {
  if (!mUsable || !rgba8 || !vertexBuffer || !indexBuffer || indexCount == 0)
    return false;
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (!ensureMeshPipeline())
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUQueue queue = static_cast<WGPUQueue>(mBackend->queue());

  WGPUCommandEncoderDescriptor encDesc = {};
  WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(device, &encDesc);
  if (!encoder)
    return false;

  WGPURenderPassColorAttachment colorAttach = {};
  colorAttach.view = static_cast<WGPUTextureView>(mView);
  colorAttach.loadOp = WGPULoadOp_Clear;
  colorAttach.storeOp = WGPUStoreOp_Store;
  colorAttach.clearValue = {color.r, color.g, color.b, color.a};
  colorAttach.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;

  WGPURenderPassDescriptor passDesc = {};
  passDesc.colorAttachmentCount = 1;
  passDesc.colorAttachments = &colorAttach;
  WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &passDesc);
  if (!pass) {
    wgpuCommandEncoderRelease(encoder);
    return false;
  }
  wgpuRenderPassEncoderSetPipeline(pass, static_cast<WGPURenderPipeline>(mMeshPipeline));
  WGPUBuffer vb = static_cast<WGPUBuffer>(vertexBuffer);
  wgpuRenderPassEncoderSetVertexBuffer(pass, 0, vb, 0, WGPU_WHOLE_SIZE);
  WGPUBuffer ib = static_cast<WGPUBuffer>(indexBuffer);
  wgpuRenderPassEncoderSetIndexBuffer(pass, ib, WGPUIndexFormat_Uint32, 0, WGPU_WHOLE_SIZE);
  wgpuRenderPassEncoderDrawIndexed(pass, indexCount, 1, 0, 0, 0);
  wgpuRenderPassEncoderEnd(pass);
  wgpuRenderPassEncoderRelease(pass);

  const uint32_t stride = alignedBytesPerRow(mWidth);
  WGPUTexelCopyTextureInfo src = {};
  src.texture = static_cast<WGPUTexture>(mTexture);
  src.aspect = WGPUTextureAspect_All;
  WGPUTexelCopyBufferInfo dst = {};
  dst.buffer = static_cast<WGPUBuffer>(mReadbackBuffer);
  dst.layout.bytesPerRow = stride;
  dst.layout.rowsPerImage = mHeight;
  WGPUExtent3D extent = {mWidth, mHeight, 1};
  wgpuCommandEncoderCopyTextureToBuffer(encoder, &src, &dst, &extent);

  WGPUCommandBufferDescriptor cmdDesc = {};
  WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(encoder, &cmdDesc);
  wgpuCommandEncoderRelease(encoder);
  if (!cmd)
    return false;
  wgpuQueueSubmit(queue, 1, &cmd);
  wgpuCommandBufferRelease(cmd);

  MapCapture cap;
  WGPUBufferMapCallbackInfo mapCb = {};
  mapCb.mode = WGPUCallbackMode_AllowSpontaneous;
  mapCb.callback = onMap;
  mapCb.userdata1 = &cap;
  (void)wgpuBufferMapAsync(static_cast<WGPUBuffer>(mReadbackBuffer),
                            WGPUMapMode_Read, 0, mReadbackBufferSize, mapCb);
  for (int i = 0; i < 1000 && !cap.done; ++i)
    wgpuDevicePoll(device, true, nullptr);
  if (!cap.done || !cap.ok)
    return false;

  const void *mapped = wgpuBufferGetConstMappedRange(static_cast<WGPUBuffer>(mReadbackBuffer), 0,
                                                    mReadbackBufferSize);
  if (!mapped) {
    wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
    return false;
  }
  const uint32_t rowBytes = mWidth * 4u;
  uint8_t *out = static_cast<uint8_t *>(rgba8);
  const uint8_t *in = static_cast<const uint8_t *>(mapped);
  for (uint32_t y = 0; y < mHeight; ++y)
    std::memcpy(out + y * rowBytes, in + y * stride, rowBytes);
  wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
  return true;
#  else
  (void)color;
  (void)vertexBuffer;
  (void)indexBuffer;
  (void)indexCount;
  (void)rgba8;
  return false;
#  endif
#else
  (void)color;
  (void)vertexBuffer;
  (void)indexBuffer;
  (void)indexCount;
  (void)rgba8;
  return false;
#endif
}

bool WbWgpuRenderTarget::ensureMvpPipeline() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (mMvpPipeline)
    return true;
  if (!mUsable)
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());

  WGPUShaderSourceWGSL wgsl = {};
  wgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
  wgsl.code.data = WbWgpuShaders::kTriangleMVP;
  wgsl.code.length = WGPU_STRLEN;
  WGPUShaderModuleDescriptor smDesc = {};
  smDesc.nextInChain = &wgsl.chain;
  WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &smDesc);
  if (!sm) {
    WbLog::info("[WbWgpuRenderTarget] mvp-pipeline CreateShaderModule failed");
    return false;
  }
  mMvpShaderModule = sm;

  // Uniform buffer: 64 bytes (16 floats) for the mat4x4.
  WGPUBufferDescriptor ubDesc = {};
  ubDesc.size = 64;
  ubDesc.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
  ubDesc.mappedAtCreation = false;
  WGPUBuffer ub = wgpuDeviceCreateBuffer(device, &ubDesc);
  if (!ub) {
    WbLog::info("[WbWgpuRenderTarget] mvp-pipeline uniform-Buffer creation failed");
    return false;
  }
  mMvpUniformBuffer = ub;

  // Bind group layout: one binding, vertex stage, uniform buffer.
  WGPUBindGroupLayoutEntry bglEntry = {};
  bglEntry.binding = 0;
  bglEntry.visibility = WGPUShaderStage_Vertex;
  bglEntry.buffer.type = WGPUBufferBindingType_Uniform;
  bglEntry.buffer.minBindingSize = 64;
  WGPUBindGroupLayoutDescriptor bglDesc = {};
  bglDesc.entryCount = 1;
  bglDesc.entries = &bglEntry;
  WGPUBindGroupLayout bgl = wgpuDeviceCreateBindGroupLayout(device, &bglDesc);
  if (!bgl) {
    WbLog::info("[WbWgpuRenderTarget] mvp-pipeline CreateBindGroupLayout failed");
    return false;
  }
  mMvpBindGroupLayout = bgl;

  // Bind group: bglayout + uniform-buffer slot.
  WGPUBindGroupEntry bgEntry = {};
  bgEntry.binding = 0;
  bgEntry.buffer = ub;
  bgEntry.offset = 0;
  bgEntry.size = 64;
  WGPUBindGroupDescriptor bgDesc = {};
  bgDesc.layout = bgl;
  bgDesc.entryCount = 1;
  bgDesc.entries = &bgEntry;
  WGPUBindGroup bg = wgpuDeviceCreateBindGroup(device, &bgDesc);
  if (!bg) {
    WbLog::info("[WbWgpuRenderTarget] mvp-pipeline CreateBindGroup failed");
    return false;
  }
  mMvpBindGroup = bg;

  // Pipeline layout: explicitly references the bind group layout
  // (auto-layout doesn't work once we have bind groups).
  WGPUPipelineLayoutDescriptor plDesc = {};
  plDesc.bindGroupLayoutCount = 1;
  plDesc.bindGroupLayouts = &bgl;
  WGPUPipelineLayout plLayout = wgpuDeviceCreatePipelineLayout(device, &plDesc);
  if (!plLayout) {
    WbLog::info("[WbWgpuRenderTarget] mvp-pipeline CreatePipelineLayout failed");
    return false;
  }

  // Same vertex attribute layout as kTriangleVertexBuffer.
  WGPUVertexAttribute attrs[3] = {};
  attrs[0].format = WGPUVertexFormat_Float32x3;
  attrs[0].offset = 0;
  attrs[0].shaderLocation = 0;
  attrs[1].format = WGPUVertexFormat_Float32x3;
  attrs[1].offset = 12;
  attrs[1].shaderLocation = 1;
  attrs[2].format = WGPUVertexFormat_Float32x2;
  attrs[2].offset = 24;
  attrs[2].shaderLocation = 2;
  WGPUVertexBufferLayout vbLayout = {};
  vbLayout.arrayStride = 32;
  vbLayout.stepMode = WGPUVertexStepMode_Vertex;
  vbLayout.attributeCount = 3;
  vbLayout.attributes = attrs;

  WGPUColorTargetState colorTarget = {};
  colorTarget.format = WGPUTextureFormat_RGBA8Unorm;
  colorTarget.writeMask = WGPUColorWriteMask_All;

  WGPUFragmentState fs = {};
  fs.module = sm;
  fs.entryPoint.data = "fs_main";
  fs.entryPoint.length = WGPU_STRLEN;
  fs.targetCount = 1;
  fs.targets = &colorTarget;

  WGPURenderPipelineDescriptor pipeDesc = {};
  pipeDesc.layout = plLayout;
  pipeDesc.vertex.module = sm;
  pipeDesc.vertex.entryPoint.data = "vs_main";
  pipeDesc.vertex.entryPoint.length = WGPU_STRLEN;
  pipeDesc.vertex.bufferCount = 1;
  pipeDesc.vertex.buffers = &vbLayout;
  pipeDesc.primitive.topology = WGPUPrimitiveTopology_TriangleList;
  pipeDesc.primitive.frontFace = WGPUFrontFace_CCW;
  pipeDesc.primitive.cullMode = WGPUCullMode_None;
  pipeDesc.multisample.count = 1;
  pipeDesc.multisample.mask = 0xFFFFFFFFu;
  pipeDesc.fragment = &fs;

  WGPURenderPipeline pipe = wgpuDeviceCreateRenderPipeline(device, &pipeDesc);
  // Pipeline layout can be released — the pipeline retains its own ref.
  wgpuPipelineLayoutRelease(plLayout);
  if (!pipe) {
    WbLog::info("[WbWgpuRenderTarget] mvp-pipeline CreateRenderPipeline failed");
    return false;
  }
  mMvpPipeline = pipe;
  return true;
#  else
  return false;
#  endif
#else
  return false;
#endif
}

bool WbWgpuRenderTarget::clearAndDrawMeshMVP(const WbWgpuClearColor &color, const float *viewProj16,
                                             void *vertexBuffer, void *indexBuffer,
                                             uint32_t indexCount, void *rgba8) {
  if (!mUsable || !rgba8 || !viewProj16 || !vertexBuffer || !indexBuffer || indexCount == 0)
    return false;
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (!ensureMvpPipeline())
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUQueue queue = static_cast<WGPUQueue>(mBackend->queue());

  // Upload the view-proj matrix into the uniform buffer.
  // WGSL mat4x4<f32> is column-major; caller is expected to lay
  // their 16 floats out in column-major order (each consecutive
  // 4 floats is one column).
  wgpuQueueWriteBuffer(queue, static_cast<WGPUBuffer>(mMvpUniformBuffer), 0, viewProj16, 64);

  WGPUCommandEncoderDescriptor encDesc = {};
  WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(device, &encDesc);
  if (!encoder)
    return false;

  WGPURenderPassColorAttachment colorAttach = {};
  colorAttach.view = static_cast<WGPUTextureView>(mView);
  colorAttach.loadOp = WGPULoadOp_Clear;
  colorAttach.storeOp = WGPUStoreOp_Store;
  colorAttach.clearValue = {color.r, color.g, color.b, color.a};
  colorAttach.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;

  WGPURenderPassDescriptor passDesc = {};
  passDesc.colorAttachmentCount = 1;
  passDesc.colorAttachments = &colorAttach;
  WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &passDesc);
  if (!pass) {
    wgpuCommandEncoderRelease(encoder);
    return false;
  }
  wgpuRenderPassEncoderSetPipeline(pass, static_cast<WGPURenderPipeline>(mMvpPipeline));
  wgpuRenderPassEncoderSetBindGroup(pass, 0, static_cast<WGPUBindGroup>(mMvpBindGroup), 0, nullptr);
  wgpuRenderPassEncoderSetVertexBuffer(pass, 0, static_cast<WGPUBuffer>(vertexBuffer), 0,
                                       WGPU_WHOLE_SIZE);
  wgpuRenderPassEncoderSetIndexBuffer(pass, static_cast<WGPUBuffer>(indexBuffer),
                                      WGPUIndexFormat_Uint32, 0, WGPU_WHOLE_SIZE);
  wgpuRenderPassEncoderDrawIndexed(pass, indexCount, 1, 0, 0, 0);
  wgpuRenderPassEncoderEnd(pass);
  wgpuRenderPassEncoderRelease(pass);

  const uint32_t stride = alignedBytesPerRow(mWidth);
  WGPUTexelCopyTextureInfo src = {};
  src.texture = static_cast<WGPUTexture>(mTexture);
  src.aspect = WGPUTextureAspect_All;
  WGPUTexelCopyBufferInfo dst = {};
  dst.buffer = static_cast<WGPUBuffer>(mReadbackBuffer);
  dst.layout.bytesPerRow = stride;
  dst.layout.rowsPerImage = mHeight;
  WGPUExtent3D extent = {mWidth, mHeight, 1};
  wgpuCommandEncoderCopyTextureToBuffer(encoder, &src, &dst, &extent);

  WGPUCommandBufferDescriptor cmdDesc = {};
  WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(encoder, &cmdDesc);
  wgpuCommandEncoderRelease(encoder);
  if (!cmd)
    return false;
  wgpuQueueSubmit(queue, 1, &cmd);
  wgpuCommandBufferRelease(cmd);

  MapCapture cap;
  WGPUBufferMapCallbackInfo mapCb = {};
  mapCb.mode = WGPUCallbackMode_AllowSpontaneous;
  mapCb.callback = onMap;
  mapCb.userdata1 = &cap;
  (void)wgpuBufferMapAsync(static_cast<WGPUBuffer>(mReadbackBuffer),
                            WGPUMapMode_Read, 0, mReadbackBufferSize, mapCb);
  for (int i = 0; i < 1000 && !cap.done; ++i)
    wgpuDevicePoll(device, true, nullptr);
  if (!cap.done || !cap.ok)
    return false;

  const void *mapped = wgpuBufferGetConstMappedRange(static_cast<WGPUBuffer>(mReadbackBuffer), 0,
                                                    mReadbackBufferSize);
  if (!mapped) {
    wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
    return false;
  }
  const uint32_t rowBytes = mWidth * 4u;
  uint8_t *out = static_cast<uint8_t *>(rgba8);
  const uint8_t *in = static_cast<const uint8_t *>(mapped);
  for (uint32_t y = 0; y < mHeight; ++y)
    std::memcpy(out + y * rowBytes, in + y * stride, rowBytes);
  wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
  return true;
#  else
  (void)color;
  (void)viewProj16;
  (void)vertexBuffer;
  (void)indexBuffer;
  (void)indexCount;
  (void)rgba8;
  return false;
#  endif
#else
  (void)color;
  (void)viewProj16;
  (void)vertexBuffer;
  (void)indexBuffer;
  (void)indexCount;
  (void)rgba8;
  return false;
#endif
}

bool WbWgpuRenderTarget::ensureTexPipeline() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (mTexPipeline)
    return true;
  if (!mUsable)
    return false;
  // Texture pipeline shares the MVP path's uniform buffer (same
  // mat4 viewProj at binding 0). Make sure the MVP path is built
  // first so mMvpUniformBuffer is valid.
  if (!ensureMvpPipeline())
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());

  WGPUShaderSourceWGSL wgsl = {};
  wgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
  wgsl.code.data = WbWgpuShaders::kTriangleTextured;
  wgsl.code.length = WGPU_STRLEN;
  WGPUShaderModuleDescriptor smDesc = {};
  smDesc.nextInChain = &wgsl.chain;
  WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &smDesc);
  if (!sm) {
    WbLog::info("[WbWgpuRenderTarget] tex-pipeline CreateShaderModule failed");
    return false;
  }
  mTexShaderModule = sm;

  // Sampler: linear filter, clamp address — process-lifetime, one
  // sampler shared by every textured draw.
  WGPUSamplerDescriptor sampDesc = {};
  sampDesc.addressModeU = WGPUAddressMode_ClampToEdge;
  sampDesc.addressModeV = WGPUAddressMode_ClampToEdge;
  sampDesc.addressModeW = WGPUAddressMode_ClampToEdge;
  sampDesc.magFilter = WGPUFilterMode_Linear;
  sampDesc.minFilter = WGPUFilterMode_Linear;
  sampDesc.mipmapFilter = WGPUMipmapFilterMode_Nearest;
  sampDesc.lodMinClamp = 0.0f;
  sampDesc.lodMaxClamp = 1.0f;
  sampDesc.compare = WGPUCompareFunction_Undefined;
  sampDesc.maxAnisotropy = 1;
  WGPUSampler samp = wgpuDeviceCreateSampler(device, &sampDesc);
  if (!samp) {
    WbLog::info("[WbWgpuRenderTarget] tex-pipeline CreateSampler failed");
    return false;
  }
  mTexSampler = samp;

  // Bind-group layout: 3 entries — uniform, texture, sampler.
  WGPUBindGroupLayoutEntry bglEntries[3] = {};
  bglEntries[0].binding = 0;
  bglEntries[0].visibility = WGPUShaderStage_Vertex;
  bglEntries[0].buffer.type = WGPUBufferBindingType_Uniform;
  bglEntries[0].buffer.minBindingSize = 64;
  bglEntries[1].binding = 1;
  bglEntries[1].visibility = WGPUShaderStage_Fragment;
  bglEntries[1].texture.sampleType = WGPUTextureSampleType_Float;
  bglEntries[1].texture.viewDimension = WGPUTextureViewDimension_2D;
  bglEntries[1].texture.multisampled = 0;
  bglEntries[2].binding = 2;
  bglEntries[2].visibility = WGPUShaderStage_Fragment;
  bglEntries[2].sampler.type = WGPUSamplerBindingType_Filtering;
  WGPUBindGroupLayoutDescriptor bglDesc = {};
  bglDesc.entryCount = 3;
  bglDesc.entries = bglEntries;
  WGPUBindGroupLayout bgl = wgpuDeviceCreateBindGroupLayout(device, &bglDesc);
  if (!bgl) {
    WbLog::info("[WbWgpuRenderTarget] tex-pipeline CreateBindGroupLayout failed");
    return false;
  }
  mTexBindGroupLayout = bgl;

  WGPUPipelineLayoutDescriptor plDesc = {};
  plDesc.bindGroupLayoutCount = 1;
  plDesc.bindGroupLayouts = &bgl;
  WGPUPipelineLayout plLayout = wgpuDeviceCreatePipelineLayout(device, &plDesc);
  if (!plLayout) {
    WbLog::info("[WbWgpuRenderTarget] tex-pipeline CreatePipelineLayout failed");
    return false;
  }

  WGPUVertexAttribute attrs[3] = {};
  attrs[0].format = WGPUVertexFormat_Float32x3;
  attrs[0].offset = 0;
  attrs[0].shaderLocation = 0;
  attrs[1].format = WGPUVertexFormat_Float32x3;
  attrs[1].offset = 12;
  attrs[1].shaderLocation = 1;
  attrs[2].format = WGPUVertexFormat_Float32x2;
  attrs[2].offset = 24;
  attrs[2].shaderLocation = 2;
  WGPUVertexBufferLayout vbLayout = {};
  vbLayout.arrayStride = 32;
  vbLayout.stepMode = WGPUVertexStepMode_Vertex;
  vbLayout.attributeCount = 3;
  vbLayout.attributes = attrs;

  WGPUColorTargetState colorTarget = {};
  colorTarget.format = WGPUTextureFormat_RGBA8Unorm;
  colorTarget.writeMask = WGPUColorWriteMask_All;

  WGPUFragmentState fs = {};
  fs.module = sm;
  fs.entryPoint.data = "fs_main";
  fs.entryPoint.length = WGPU_STRLEN;
  fs.targetCount = 1;
  fs.targets = &colorTarget;

  WGPURenderPipelineDescriptor pipeDesc = {};
  pipeDesc.layout = plLayout;
  pipeDesc.vertex.module = sm;
  pipeDesc.vertex.entryPoint.data = "vs_main";
  pipeDesc.vertex.entryPoint.length = WGPU_STRLEN;
  pipeDesc.vertex.bufferCount = 1;
  pipeDesc.vertex.buffers = &vbLayout;
  pipeDesc.primitive.topology = WGPUPrimitiveTopology_TriangleList;
  pipeDesc.primitive.frontFace = WGPUFrontFace_CCW;
  pipeDesc.primitive.cullMode = WGPUCullMode_None;
  pipeDesc.multisample.count = 1;
  pipeDesc.multisample.mask = 0xFFFFFFFFu;
  pipeDesc.fragment = &fs;
  WGPURenderPipeline pipe = wgpuDeviceCreateRenderPipeline(device, &pipeDesc);
  wgpuPipelineLayoutRelease(plLayout);
  if (!pipe) {
    WbLog::info("[WbWgpuRenderTarget] tex-pipeline CreateRenderPipeline failed");
    return false;
  }
  mTexPipeline = pipe;
  return true;
#  else
  return false;
#  endif
#else
  return false;
#endif
}

bool WbWgpuRenderTarget::clearAndDrawMeshTextured(const WbWgpuClearColor &color,
                                                   const float *viewProj16, void *textureView,
                                                   void *vertexBuffer, void *indexBuffer,
                                                   uint32_t indexCount, void *rgba8) {
  if (!mUsable || !rgba8 || !viewProj16 || !textureView || !vertexBuffer || !indexBuffer ||
      indexCount == 0)
    return false;
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (!ensureTexPipeline())
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUQueue queue = static_cast<WGPUQueue>(mBackend->queue());

  wgpuQueueWriteBuffer(queue, static_cast<WGPUBuffer>(mMvpUniformBuffer), 0, viewProj16, 64);

  // Build a fresh bind group per call so each textured draw can
  // bind a different view without us caching N of them.
  WGPUBindGroupEntry bgEntries[3] = {};
  bgEntries[0].binding = 0;
  bgEntries[0].buffer = static_cast<WGPUBuffer>(mMvpUniformBuffer);
  bgEntries[0].offset = 0;
  bgEntries[0].size = 64;
  bgEntries[1].binding = 1;
  bgEntries[1].textureView = static_cast<WGPUTextureView>(textureView);
  bgEntries[2].binding = 2;
  bgEntries[2].sampler = static_cast<WGPUSampler>(mTexSampler);
  WGPUBindGroupDescriptor bgDesc = {};
  bgDesc.layout = static_cast<WGPUBindGroupLayout>(mTexBindGroupLayout);
  bgDesc.entryCount = 3;
  bgDesc.entries = bgEntries;
  WGPUBindGroup bg = wgpuDeviceCreateBindGroup(device, &bgDesc);
  if (!bg)
    return false;

  WGPUCommandEncoderDescriptor encDesc = {};
  WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(device, &encDesc);
  if (!encoder) {
    wgpuBindGroupRelease(bg);
    return false;
  }

  WGPURenderPassColorAttachment colorAttach = {};
  colorAttach.view = static_cast<WGPUTextureView>(mView);
  colorAttach.loadOp = WGPULoadOp_Clear;
  colorAttach.storeOp = WGPUStoreOp_Store;
  colorAttach.clearValue = {color.r, color.g, color.b, color.a};
  colorAttach.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;

  WGPURenderPassDescriptor passDesc = {};
  passDesc.colorAttachmentCount = 1;
  passDesc.colorAttachments = &colorAttach;
  WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &passDesc);
  if (!pass) {
    wgpuCommandEncoderRelease(encoder);
    wgpuBindGroupRelease(bg);
    return false;
  }
  wgpuRenderPassEncoderSetPipeline(pass, static_cast<WGPURenderPipeline>(mTexPipeline));
  wgpuRenderPassEncoderSetBindGroup(pass, 0, bg, 0, nullptr);
  wgpuRenderPassEncoderSetVertexBuffer(pass, 0, static_cast<WGPUBuffer>(vertexBuffer), 0,
                                       WGPU_WHOLE_SIZE);
  wgpuRenderPassEncoderSetIndexBuffer(pass, static_cast<WGPUBuffer>(indexBuffer),
                                      WGPUIndexFormat_Uint32, 0, WGPU_WHOLE_SIZE);
  wgpuRenderPassEncoderDrawIndexed(pass, indexCount, 1, 0, 0, 0);
  wgpuRenderPassEncoderEnd(pass);
  wgpuRenderPassEncoderRelease(pass);

  const uint32_t stride = alignedBytesPerRow(mWidth);
  WGPUTexelCopyTextureInfo src = {};
  src.texture = static_cast<WGPUTexture>(mTexture);
  src.aspect = WGPUTextureAspect_All;
  WGPUTexelCopyBufferInfo dst = {};
  dst.buffer = static_cast<WGPUBuffer>(mReadbackBuffer);
  dst.layout.bytesPerRow = stride;
  dst.layout.rowsPerImage = mHeight;
  WGPUExtent3D extent = {mWidth, mHeight, 1};
  wgpuCommandEncoderCopyTextureToBuffer(encoder, &src, &dst, &extent);

  WGPUCommandBufferDescriptor cmdDesc = {};
  WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(encoder, &cmdDesc);
  wgpuCommandEncoderRelease(encoder);
  if (!cmd) {
    wgpuBindGroupRelease(bg);
    return false;
  }
  wgpuQueueSubmit(queue, 1, &cmd);
  wgpuCommandBufferRelease(cmd);

  MapCapture cap;
  WGPUBufferMapCallbackInfo mapCb = {};
  mapCb.mode = WGPUCallbackMode_AllowSpontaneous;
  mapCb.callback = onMap;
  mapCb.userdata1 = &cap;
  (void)wgpuBufferMapAsync(static_cast<WGPUBuffer>(mReadbackBuffer),
                            WGPUMapMode_Read, 0, mReadbackBufferSize, mapCb);
  for (int i = 0; i < 1000 && !cap.done; ++i)
    wgpuDevicePoll(device, true, nullptr);
  // Bind group can be released now — the submission retains the
  // resources internally.
  wgpuBindGroupRelease(bg);
  if (!cap.done || !cap.ok)
    return false;

  const void *mapped = wgpuBufferGetConstMappedRange(static_cast<WGPUBuffer>(mReadbackBuffer), 0,
                                                    mReadbackBufferSize);
  if (!mapped) {
    wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
    return false;
  }
  const uint32_t rowBytes = mWidth * 4u;
  uint8_t *out = static_cast<uint8_t *>(rgba8);
  const uint8_t *in = static_cast<const uint8_t *>(mapped);
  for (uint32_t y = 0; y < mHeight; ++y)
    std::memcpy(out + y * rowBytes, in + y * stride, rowBytes);
  wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
  return true;
#  else
  (void)color;
  (void)viewProj16;
  (void)textureView;
  (void)vertexBuffer;
  (void)indexBuffer;
  (void)indexCount;
  (void)rgba8;
  return false;
#  endif
#else
  (void)color;
  (void)viewProj16;
  (void)textureView;
  (void)vertexBuffer;
  (void)indexBuffer;
  (void)indexCount;
  (void)rgba8;
  return false;
#endif
}

bool WbWgpuRenderTarget::ensureInstPipeline() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (mInstPipeline)
    return true;
  if (!mUsable)
    return false;
  if (!ensureMvpPipeline())  // reuses mMvpUniformBuffer
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());

  WGPUShaderSourceWGSL wgsl = {};
  wgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
  wgsl.code.data = WbWgpuShaders::kTriangleInstanced;
  wgsl.code.length = WGPU_STRLEN;
  WGPUShaderModuleDescriptor smDesc = {};
  smDesc.nextInChain = &wgsl.chain;
  WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &smDesc);
  if (!sm) {
    WbLog::info("[WbWgpuRenderTarget] inst-pipeline CreateShaderModule failed");
    return false;
  }
  mInstShaderModule = sm;

  WGPUBindGroupLayoutEntry bglEntries[2] = {};
  bglEntries[0].binding = 0;
  bglEntries[0].visibility = WGPUShaderStage_Vertex;
  bglEntries[0].buffer.type = WGPUBufferBindingType_Uniform;
  bglEntries[0].buffer.minBindingSize = 64;
  bglEntries[1].binding = 1;
  bglEntries[1].visibility = WGPUShaderStage_Vertex;
  bglEntries[1].buffer.type = WGPUBufferBindingType_ReadOnlyStorage;
  bglEntries[1].buffer.minBindingSize = 16;  // at least one vec4
  WGPUBindGroupLayoutDescriptor bglDesc = {};
  bglDesc.entryCount = 2;
  bglDesc.entries = bglEntries;
  WGPUBindGroupLayout bgl = wgpuDeviceCreateBindGroupLayout(device, &bglDesc);
  if (!bgl) {
    WbLog::info("[WbWgpuRenderTarget] inst-pipeline CreateBindGroupLayout failed");
    return false;
  }
  mInstBindGroupLayout = bgl;

  WGPUPipelineLayoutDescriptor plDesc = {};
  plDesc.bindGroupLayoutCount = 1;
  plDesc.bindGroupLayouts = &bgl;
  WGPUPipelineLayout plLayout = wgpuDeviceCreatePipelineLayout(device, &plDesc);
  if (!plLayout) {
    WbLog::info("[WbWgpuRenderTarget] inst-pipeline CreatePipelineLayout failed");
    return false;
  }

  WGPUVertexAttribute attrs[3] = {};
  attrs[0].format = WGPUVertexFormat_Float32x3;
  attrs[0].offset = 0;
  attrs[0].shaderLocation = 0;
  attrs[1].format = WGPUVertexFormat_Float32x3;
  attrs[1].offset = 12;
  attrs[1].shaderLocation = 1;
  attrs[2].format = WGPUVertexFormat_Float32x2;
  attrs[2].offset = 24;
  attrs[2].shaderLocation = 2;
  WGPUVertexBufferLayout vbLayout = {};
  vbLayout.arrayStride = 32;
  vbLayout.stepMode = WGPUVertexStepMode_Vertex;
  vbLayout.attributeCount = 3;
  vbLayout.attributes = attrs;

  WGPUColorTargetState colorTarget = {};
  colorTarget.format = WGPUTextureFormat_RGBA8Unorm;
  colorTarget.writeMask = WGPUColorWriteMask_All;

  WGPUFragmentState fs = {};
  fs.module = sm;
  fs.entryPoint.data = "fs_main";
  fs.entryPoint.length = WGPU_STRLEN;
  fs.targetCount = 1;
  fs.targets = &colorTarget;

  WGPURenderPipelineDescriptor pipeDesc = {};
  pipeDesc.layout = plLayout;
  pipeDesc.vertex.module = sm;
  pipeDesc.vertex.entryPoint.data = "vs_main";
  pipeDesc.vertex.entryPoint.length = WGPU_STRLEN;
  pipeDesc.vertex.bufferCount = 1;
  pipeDesc.vertex.buffers = &vbLayout;
  pipeDesc.primitive.topology = WGPUPrimitiveTopology_TriangleList;
  pipeDesc.primitive.frontFace = WGPUFrontFace_CCW;
  pipeDesc.primitive.cullMode = WGPUCullMode_None;
  pipeDesc.multisample.count = 1;
  pipeDesc.multisample.mask = 0xFFFFFFFFu;
  pipeDesc.fragment = &fs;
  WGPURenderPipeline pipe = wgpuDeviceCreateRenderPipeline(device, &pipeDesc);
  wgpuPipelineLayoutRelease(plLayout);
  if (!pipe) {
    WbLog::info("[WbWgpuRenderTarget] inst-pipeline CreateRenderPipeline failed");
    return false;
  }
  mInstPipeline = pipe;
  return true;
#  else
  return false;
#  endif
#else
  return false;
#endif
}

bool WbWgpuRenderTarget::clearAndDrawInstanced(const WbWgpuClearColor &color,
                                                const float *viewProj16,
                                                const float *bodyOffsetsXyz0, uint32_t bodyCount,
                                                void *vertexBuffer, void *indexBuffer,
                                                uint32_t indexCount, void *rgba8) {
  if (!mUsable || !rgba8 || !viewProj16 || !bodyOffsetsXyz0 || bodyCount == 0 || !vertexBuffer ||
      !indexBuffer || indexCount == 0)
    return false;
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (!ensureInstPipeline())
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUQueue queue = static_cast<WGPUQueue>(mBackend->queue());

  wgpuQueueWriteBuffer(queue, static_cast<WGPUBuffer>(mMvpUniformBuffer), 0, viewProj16, 64);

  // Grow storage buffer if needed. Each body = vec4<f32> = 16 bytes.
  const size_t needed = static_cast<size_t>(bodyCount) * 16u;
  if (needed > mInstStorageBufferSize) {
    if (mInstStorageBuffer)
      wgpuBufferRelease(static_cast<WGPUBuffer>(mInstStorageBuffer));
    WGPUBufferDescriptor sbDesc = {};
    sbDesc.size = needed;
    sbDesc.usage = WGPUBufferUsage_Storage | WGPUBufferUsage_CopyDst;
    sbDesc.mappedAtCreation = false;
    WGPUBuffer sb = wgpuDeviceCreateBuffer(device, &sbDesc);
    if (!sb)
      return false;
    mInstStorageBuffer = sb;
    mInstStorageBufferSize = needed;
  }
  wgpuQueueWriteBuffer(queue, static_cast<WGPUBuffer>(mInstStorageBuffer), 0, bodyOffsetsXyz0,
                       needed);

  WGPUBindGroupEntry bgEntries[2] = {};
  bgEntries[0].binding = 0;
  bgEntries[0].buffer = static_cast<WGPUBuffer>(mMvpUniformBuffer);
  bgEntries[0].size = 64;
  bgEntries[1].binding = 1;
  bgEntries[1].buffer = static_cast<WGPUBuffer>(mInstStorageBuffer);
  bgEntries[1].size = needed;
  WGPUBindGroupDescriptor bgDesc = {};
  bgDesc.layout = static_cast<WGPUBindGroupLayout>(mInstBindGroupLayout);
  bgDesc.entryCount = 2;
  bgDesc.entries = bgEntries;
  WGPUBindGroup bg = wgpuDeviceCreateBindGroup(device, &bgDesc);
  if (!bg)
    return false;

  WGPUCommandEncoderDescriptor encDesc = {};
  WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(device, &encDesc);
  if (!encoder) {
    wgpuBindGroupRelease(bg);
    return false;
  }

  WGPURenderPassColorAttachment colorAttach = {};
  colorAttach.view = static_cast<WGPUTextureView>(mView);
  colorAttach.loadOp = WGPULoadOp_Clear;
  colorAttach.storeOp = WGPUStoreOp_Store;
  colorAttach.clearValue = {color.r, color.g, color.b, color.a};
  colorAttach.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;

  WGPURenderPassDescriptor passDesc = {};
  passDesc.colorAttachmentCount = 1;
  passDesc.colorAttachments = &colorAttach;
  WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &passDesc);
  if (!pass) {
    wgpuCommandEncoderRelease(encoder);
    wgpuBindGroupRelease(bg);
    return false;
  }
  wgpuRenderPassEncoderSetPipeline(pass, static_cast<WGPURenderPipeline>(mInstPipeline));
  wgpuRenderPassEncoderSetBindGroup(pass, 0, bg, 0, nullptr);
  wgpuRenderPassEncoderSetVertexBuffer(pass, 0, static_cast<WGPUBuffer>(vertexBuffer), 0,
                                       WGPU_WHOLE_SIZE);
  wgpuRenderPassEncoderSetIndexBuffer(pass, static_cast<WGPUBuffer>(indexBuffer),
                                      WGPUIndexFormat_Uint32, 0, WGPU_WHOLE_SIZE);
  wgpuRenderPassEncoderDrawIndexed(pass, indexCount, bodyCount, 0, 0, 0);
  wgpuRenderPassEncoderEnd(pass);
  wgpuRenderPassEncoderRelease(pass);

  const uint32_t stride = alignedBytesPerRow(mWidth);
  WGPUTexelCopyTextureInfo src = {};
  src.texture = static_cast<WGPUTexture>(mTexture);
  src.aspect = WGPUTextureAspect_All;
  WGPUTexelCopyBufferInfo dst = {};
  dst.buffer = static_cast<WGPUBuffer>(mReadbackBuffer);
  dst.layout.bytesPerRow = stride;
  dst.layout.rowsPerImage = mHeight;
  WGPUExtent3D extent = {mWidth, mHeight, 1};
  wgpuCommandEncoderCopyTextureToBuffer(encoder, &src, &dst, &extent);

  WGPUCommandBufferDescriptor cmdDesc = {};
  WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(encoder, &cmdDesc);
  wgpuCommandEncoderRelease(encoder);
  if (!cmd) {
    wgpuBindGroupRelease(bg);
    return false;
  }
  wgpuQueueSubmit(queue, 1, &cmd);
  wgpuCommandBufferRelease(cmd);

  MapCapture cap;
  WGPUBufferMapCallbackInfo mapCb = {};
  mapCb.mode = WGPUCallbackMode_AllowSpontaneous;
  mapCb.callback = onMap;
  mapCb.userdata1 = &cap;
  (void)wgpuBufferMapAsync(static_cast<WGPUBuffer>(mReadbackBuffer),
                            WGPUMapMode_Read, 0, mReadbackBufferSize, mapCb);
  for (int i = 0; i < 1000 && !cap.done; ++i)
    wgpuDevicePoll(device, true, nullptr);
  wgpuBindGroupRelease(bg);
  if (!cap.done || !cap.ok)
    return false;

  const void *mapped = wgpuBufferGetConstMappedRange(static_cast<WGPUBuffer>(mReadbackBuffer), 0,
                                                    mReadbackBufferSize);
  if (!mapped) {
    wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
    return false;
  }
  const uint32_t rowBytes = mWidth * 4u;
  uint8_t *out = static_cast<uint8_t *>(rgba8);
  const uint8_t *in = static_cast<const uint8_t *>(mapped);
  for (uint32_t y = 0; y < mHeight; ++y)
    std::memcpy(out + y * rowBytes, in + y * stride, rowBytes);
  wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
  return true;
#  else
  (void)color;
  (void)viewProj16;
  (void)bodyOffsetsXyz0;
  (void)bodyCount;
  (void)vertexBuffer;
  (void)indexBuffer;
  (void)indexCount;
  (void)rgba8;
  return false;
#  endif
#else
  (void)color;
  (void)viewProj16;
  (void)bodyOffsetsXyz0;
  (void)bodyCount;
  (void)vertexBuffer;
  (void)indexBuffer;
  (void)indexCount;
  (void)rgba8;
  return false;
#endif
}

bool WbWgpuRenderTarget::clearAndDrawTriangle(const WbWgpuClearColor &color, void *rgba8) {
  if (!mUsable || !rgba8)
    return false;
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (!ensureTrianglePipeline())
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUQueue queue = static_cast<WGPUQueue>(mBackend->queue());

  WGPUCommandEncoderDescriptor encDesc = {};
  WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(device, &encDesc);
  if (!encoder)
    return false;

  WGPURenderPassColorAttachment colorAttach = {};
  colorAttach.view = static_cast<WGPUTextureView>(mView);
  colorAttach.loadOp = WGPULoadOp_Clear;
  colorAttach.storeOp = WGPUStoreOp_Store;
  colorAttach.clearValue = {color.r, color.g, color.b, color.a};
  colorAttach.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;

  WGPURenderPassDescriptor passDesc = {};
  passDesc.colorAttachmentCount = 1;
  passDesc.colorAttachments = &colorAttach;
  WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &passDesc);
  if (!pass) {
    wgpuCommandEncoderRelease(encoder);
    return false;
  }
  wgpuRenderPassEncoderSetPipeline(pass, static_cast<WGPURenderPipeline>(mTrianglePipeline));
  // Single draw: 3 vertices, 1 instance, vertex-id-based positions.
  wgpuRenderPassEncoderDraw(pass, 3, 1, 0, 0);
  wgpuRenderPassEncoderEnd(pass);
  wgpuRenderPassEncoderRelease(pass);

  // Copy texture -> readback buffer (same code path as clearAndRead).
  const uint32_t stride = alignedBytesPerRow(mWidth);
  WGPUTexelCopyTextureInfo src = {};
  src.texture = static_cast<WGPUTexture>(mTexture);
  src.mipLevel = 0;
  src.origin = {0, 0, 0};
  src.aspect = WGPUTextureAspect_All;
  WGPUTexelCopyBufferInfo dst = {};
  dst.buffer = static_cast<WGPUBuffer>(mReadbackBuffer);
  dst.layout.offset = 0;
  dst.layout.bytesPerRow = stride;
  dst.layout.rowsPerImage = mHeight;
  WGPUExtent3D extent = {mWidth, mHeight, 1};
  wgpuCommandEncoderCopyTextureToBuffer(encoder, &src, &dst, &extent);

  WGPUCommandBufferDescriptor cmdDesc = {};
  WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(encoder, &cmdDesc);
  wgpuCommandEncoderRelease(encoder);
  if (!cmd)
    return false;
  wgpuQueueSubmit(queue, 1, &cmd);
  wgpuCommandBufferRelease(cmd);

  MapCapture cap;
  WGPUBufferMapCallbackInfo mapCb = {};
  mapCb.mode = WGPUCallbackMode_AllowSpontaneous;
  mapCb.callback = onMap;
  mapCb.userdata1 = &cap;
  (void)wgpuBufferMapAsync(static_cast<WGPUBuffer>(mReadbackBuffer),
                            WGPUMapMode_Read, 0, mReadbackBufferSize, mapCb);
  for (int i = 0; i < 1000 && !cap.done; ++i)
    wgpuDevicePoll(device, true, nullptr);
  if (!cap.done || !cap.ok)
    return false;

  const void *mapped = wgpuBufferGetConstMappedRange(static_cast<WGPUBuffer>(mReadbackBuffer), 0,
                                                    mReadbackBufferSize);
  if (!mapped) {
    wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
    return false;
  }
  const uint32_t rowBytes = mWidth * 4u;
  uint8_t *out = static_cast<uint8_t *>(rgba8);
  const uint8_t *in = static_cast<const uint8_t *>(mapped);
  for (uint32_t y = 0; y < mHeight; ++y)
    std::memcpy(out + y * rowBytes, in + y * stride, rowBytes);
  wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
  return true;
#  else
  (void)color;
  (void)rgba8;
  return false;
#  endif
#else
  (void)color;
  (void)rgba8;
  return false;
#endif
}

bool WbWgpuRenderTarget::clearAndRead(const WbWgpuClearColor &color, void *rgba8) {
  if (!mUsable || !rgba8)
    return false;
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUQueue queue = static_cast<WGPUQueue>(mBackend->queue());
  WGPUInstance instance = static_cast<WGPUInstance>(mBackend->device());  // unused; instance held separately
  (void)instance;

  // Encode one render pass that clears mView, then a buffer-image
  // copy from the color attachment into the readback buffer.
  WGPUCommandEncoderDescriptor encDesc = {};
  WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(device, &encDesc);
  if (!encoder)
    return false;

  WGPURenderPassColorAttachment colorAttach = {};
  colorAttach.view = static_cast<WGPUTextureView>(mView);
  colorAttach.loadOp = WGPULoadOp_Clear;
  colorAttach.storeOp = WGPUStoreOp_Store;
  colorAttach.clearValue = {color.r, color.g, color.b, color.a};
  colorAttach.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;

  WGPURenderPassDescriptor passDesc = {};
  passDesc.colorAttachmentCount = 1;
  passDesc.colorAttachments = &colorAttach;
  WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &passDesc);
  if (!pass) {
    wgpuCommandEncoderRelease(encoder);
    return false;
  }
  // Solid-color clear: no draw calls in R3.3. R3.4 adds geometry.
  wgpuRenderPassEncoderEnd(pass);
  wgpuRenderPassEncoderRelease(pass);

  // Copy texture -> buffer.
  const uint32_t stride = alignedBytesPerRow(mWidth);
  WGPUTexelCopyTextureInfo src = {};
  src.texture = static_cast<WGPUTexture>(mTexture);
  src.mipLevel = 0;
  src.origin = {0, 0, 0};
  src.aspect = WGPUTextureAspect_All;
  WGPUTexelCopyBufferInfo dst = {};
  dst.buffer = static_cast<WGPUBuffer>(mReadbackBuffer);
  dst.layout.offset = 0;
  dst.layout.bytesPerRow = stride;
  dst.layout.rowsPerImage = mHeight;
  WGPUExtent3D extent = {mWidth, mHeight, 1};
  wgpuCommandEncoderCopyTextureToBuffer(encoder, &src, &dst, &extent);

  WGPUCommandBufferDescriptor cmdDesc = {};
  WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(encoder, &cmdDesc);
  wgpuCommandEncoderRelease(encoder);
  if (!cmd)
    return false;
  wgpuQueueSubmit(queue, 1, &cmd);
  wgpuCommandBufferRelease(cmd);

  // Map readback buffer + busy-poll. wgpu-native v29 doesn't
  // implement wgpuInstanceWaitAny (panics on the unimplemented.rs
  // path), so we use AllowSpontaneous + wgpuDevicePoll which the
  // native impl honors. The poll-with-wait=true blocks until any
  // GPU work has progressed, which is what we need to flush the
  // QueueSubmit before the map callback can fire.
  MapCapture cap;
  WGPUBufferMapCallbackInfo mapCb = {};
  mapCb.mode = WGPUCallbackMode_AllowSpontaneous;
  mapCb.callback = onMap;
  mapCb.userdata1 = &cap;
  (void)wgpuBufferMapAsync(static_cast<WGPUBuffer>(mReadbackBuffer),
                            WGPUMapMode_Read, 0, mReadbackBufferSize, mapCb);
  for (int i = 0; i < 1000 && !cap.done; ++i) {
    wgpuDevicePoll(device, true /*wait*/, nullptr);
  }
  if (!cap.done || !cap.ok)
    return false;

  const void *mapped = wgpuBufferGetConstMappedRange(static_cast<WGPUBuffer>(mReadbackBuffer), 0,
                                                    mReadbackBufferSize);
  if (!mapped) {
    wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
    return false;
  }
  // Strip the row-stride padding into the caller's tight rgba8 buffer.
  const uint32_t rowBytes = mWidth * 4u;
  uint8_t *out = static_cast<uint8_t *>(rgba8);
  const uint8_t *in = static_cast<const uint8_t *>(mapped);
  for (uint32_t y = 0; y < mHeight; ++y)
    std::memcpy(out + y * rowBytes, in + y * stride, rowBytes);
  wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
  return true;
#  else
  (void)color;
  (void)rgba8;
  return false;
#  endif
#else
  (void)color;
  (void)rgba8;
  return false;
#endif
}
