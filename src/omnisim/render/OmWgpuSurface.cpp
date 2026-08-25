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

#include "OmWgpuSurface.hpp"

#include "OmLog.hpp"
#include "OmVulkanBackend.hpp"
#include "OmWgpuRenderTarget.hpp"  // OmWgpuSolidDraw
#include "OmWgpuShaders.hpp"       // kSolidLit

#include <QtCore/QString>  // the surface-source kind is named in the log lines below

#include <cstring>
#include <vector>

#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
#    include "webgpu/webgpu.h"
#    include "webgpu/wgpu.h"
// 256-byte minimum dynamic-uniform offset alignment; the kSolidLit Scene
// payload is 192 B, padded to 256 B per draw. Mirrors OmWgpuRenderTarget.
static constexpr uint32_t kScnUniformStride = 256;

namespace {
  // CopyTextureToBuffer requires bytes-per-row a multiple of 256.
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
  void onMap(WGPUMapAsyncStatus status, WGPUStringView, void *userdata1, void *) {
    auto *cap = static_cast<MapCapture *>(userdata1);
    cap->ok = (status == WGPUMapAsyncStatus_Success);
    cap->done = true;
  }
}  // namespace
#  endif
#endif

bool OmWgpuSurface::formatIsBgra() const {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  return static_cast<WGPUTextureFormat>(mFormat) == WGPUTextureFormat_BGRA8Unorm ||
         static_cast<WGPUTextureFormat>(mFormat) == WGPUTextureFormat_BGRA8UnormSrgb;
#  else
  return false;
#  endif
#else
  return false;
#endif
}

OmWgpuSurface::OmWgpuSurface(OmVulkanBackend *backend, const OmWgpuNativeWindow &nativeWindow,
                             uint32_t width, uint32_t height)
  : mBackend(backend), mWidth(width), mHeight(height) {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (!backend || !backend->isAvailable() || !backend->instance() || !backend->adapter() ||
      !backend->device() || width == 0 || height == 0)
    return;

  // No native handle for this platform (unknown QPA plugin, missing native
  // interface, window not realised yet) — leave mSurface null and mUsable
  // false so the caller keeps its existing presentation path. A Linux user
  // must never get a black window because a surface source is missing.
  const QString sourceName = QString::fromLatin1(nativeWindow.kindName());
  if (!nativeWindow.isValid()) {
    OmLog::info(QString("[OmWgpuSurface] no usable native window handle (kind=%1) — on-screen wgpu "
                        "presentation disabled, caller falls back")
                  .arg(sourceName));
    return;
  }

  WGPUInstance instance = static_cast<WGPUInstance>(backend->instance());
  WGPUAdapter adapter = static_cast<WGPUAdapter>(backend->adapter());

  // One chained surface-source per windowing system. All four live in this
  // scope because surfDesc.nextInChain borrows the chosen one and must stay
  // valid across wgpuInstanceCreateSurface. Struct/field/sType names are
  // wgpu-native v29 (include/webgpu/webgpu.h): every one of these takes
  // void*/uint32_t/uint64_t, so no X11 or Wayland header is needed here.
  WGPUSurfaceSourceWindowsHWND fromHwnd = {};
  WGPUSurfaceSourceXlibWindow fromXlib = {};
  WGPUSurfaceSourceXCBWindow fromXcb = {};
  WGPUSurfaceSourceWaylandSurface fromWayland = {};
  WGPUSurfaceDescriptor surfDesc = {};
  switch (nativeWindow.kind) {
    case OmWgpuNativeWindow::Kind::WindowsHwnd:
      fromHwnd.chain.sType = WGPUSType_SurfaceSourceWindowsHWND;
      fromHwnd.hinstance = nativeWindow.instance;
      fromHwnd.hwnd = nativeWindow.hwnd;
      surfDesc.nextInChain = &fromHwnd.chain;
      break;
    case OmWgpuNativeWindow::Kind::XlibWindow:
      fromXlib.chain.sType = WGPUSType_SurfaceSourceXlibWindow;
      fromXlib.display = nativeWindow.display;  // Display *
      fromXlib.window = nativeWindow.windowId;  // Window (XID), uint64_t field
      surfDesc.nextInChain = &fromXlib.chain;
      break;
    case OmWgpuNativeWindow::Kind::XcbWindow:
      fromXcb.chain.sType = WGPUSType_SurfaceSourceXCBWindow;
      fromXcb.connection = nativeWindow.display;  // xcb_connection_t *
      fromXcb.window = static_cast<uint32_t>(nativeWindow.windowId);  // xcb_window_t is 32-bit
      surfDesc.nextInChain = &fromXcb.chain;
      break;
    case OmWgpuNativeWindow::Kind::WaylandSurface:
      fromWayland.chain.sType = WGPUSType_SurfaceSourceWaylandSurface;
      fromWayland.display = nativeWindow.display;  // wl_display *
      fromWayland.surface = nativeWindow.surface;  // wl_surface *
      surfDesc.nextInChain = &fromWayland.chain;
      break;
    case OmWgpuNativeWindow::Kind::None:
      break;  // unreachable: isValid() rejected it above
  }
  if (!surfDesc.nextInChain)
    return;
  WGPUSurface surface = wgpuInstanceCreateSurface(instance, &surfDesc);
  if (!surface) {
    OmLog::info(QString("[OmWgpuSurface] wgpuInstanceCreateSurface failed (kind=%1)")
                  .arg(sourceName));
    return;
  }
  mSurface = surface;

  WGPUTextureFormat chosen = WGPUTextureFormat_BGRA8Unorm;
  WGPUTextureUsage supportedUsages = WGPUTextureUsage_RenderAttachment;
  WGPUSurfaceCapabilities caps = {};
  if (wgpuSurfaceGetCapabilities(surface, adapter, &caps) == WGPUStatus_Success) {
    if (caps.formatCount > 0 && caps.formats) {
      chosen = caps.formats[0];
      // Prefer a NON-sRGB swapchain: every frame we present is ALREADY display-encoded (the scene
      // shaders gamma-encode their output), so an sRGB surface format encodes a SECOND time and the
      // image washes out bright/white — the window-swap "bright and foggy" bug. Vulkan on Windows
      // lists the sRGB variant first in its capabilities, so formats[0] alone picks it.
      for (size_t i = 0; i < caps.formatCount; ++i)
        if (caps.formats[i] == WGPUTextureFormat_BGRA8Unorm ||
            caps.formats[i] == WGPUTextureFormat_RGBA8Unorm) {
          chosen = caps.formats[i];
          break;
        }
    }
    supportedUsages = caps.usages;
    wgpuSurfaceCapabilitiesFreeMembers(caps);
  }
  mFormat = static_cast<uint32_t>(chosen);
  // RenderAttachment is guaranteed; add CopySrc when supported so the
  // presentScene self-check can read the backbuffer back.
  mCanReadback = (supportedUsages & WGPUTextureUsage_CopySrc) != 0;
  mUsage = WGPUTextureUsage_RenderAttachment | (mCanReadback ? WGPUTextureUsage_CopySrc : 0u);

  configure();
  if (!mConfigured) {
    OmLog::info("[OmWgpuSurface] initial configure failed");
    return;
  }

  mUsable = true;
  OmLog::info(QString("[OmWgpuSurface] on-screen surface created + configured (source=%1)")
                .arg(sourceName));
#  else
  (void)backend;
  (void)nativeWindow;
  (void)width;
  (void)height;
#  endif
#else
  (void)backend;
  (void)nativeWindow;
  (void)width;
  (void)height;
#endif
}

OmWgpuSurface::~OmWgpuSurface() {
  releaseAll();
}

void OmWgpuSurface::releaseAll() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (mScnDepthView)
    wgpuTextureViewRelease(static_cast<WGPUTextureView>(mScnDepthView));
  if (mScnDepthTexture)
    wgpuTextureRelease(static_cast<WGPUTexture>(mScnDepthTexture));
  if (mScnUniformBuffer)
    wgpuBufferRelease(static_cast<WGPUBuffer>(mScnUniformBuffer));
  if (mReadbackBuffer)
    wgpuBufferRelease(static_cast<WGPUBuffer>(mReadbackBuffer));
  if (mScnBindGroupLayout)
    wgpuBindGroupLayoutRelease(static_cast<WGPUBindGroupLayout>(mScnBindGroupLayout));
  if (mScnPipeline)
    wgpuRenderPipelineRelease(static_cast<WGPURenderPipeline>(mScnPipeline));
  if (mScnShaderModule)
    wgpuShaderModuleRelease(static_cast<WGPUShaderModule>(mScnShaderModule));
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
  if (mLineUniformBuffer)
    wgpuBufferRelease(static_cast<WGPUBuffer>(mLineUniformBuffer));
  if (mLinePipeline)
    wgpuRenderPipelineRelease(static_cast<WGPURenderPipeline>(mLinePipeline));
  if (mLineNoDepthPipeline)
    wgpuRenderPipelineRelease(static_cast<WGPURenderPipeline>(mLineNoDepthPipeline));
  if (mLineShaderModule)
    wgpuShaderModuleRelease(static_cast<WGPUShaderModule>(mLineShaderModule));
  if (mSurface) {
    wgpuSurfaceUnconfigure(static_cast<WGPUSurface>(mSurface));
    wgpuSurfaceRelease(static_cast<WGPUSurface>(mSurface));
  }
#  endif
#endif
  mScnDepthView = mScnDepthTexture = mScnUniformBuffer = mReadbackBuffer = nullptr;
  mLineUniformBuffer = mLinePipeline = mLineNoDepthPipeline = mLineShaderModule = nullptr;
  mScnTexPipeline = mScnTexShaderModule = mScnTexBindGroupLayout = mScnTexSampler = nullptr;
  mScnDefaultWhiteTex = mScnDefaultWhiteView = nullptr;
  mScnDefaultBlackTex = mScnDefaultBlackView = nullptr;
  mScnDefaultNormalTex = mScnDefaultNormalView = nullptr;
  mScnBindGroupLayout = mScnPipeline = mScnShaderModule = nullptr;
  mScnUniformBufferSize = 0;
  mReadbackSize = 0;
  mScnDepthW = mScnDepthH = 0;
  mSurface = nullptr;
  mConfigured = false;
  mUsable = false;
}

void OmWgpuSurface::configure() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (!mSurface || !mBackend || !mBackend->device() || mWidth == 0 || mHeight == 0) {
    mConfigured = false;
    return;
  }
  WGPUSurfaceConfiguration cfg = {};
  cfg.device = static_cast<WGPUDevice>(mBackend->device());
  cfg.format = static_cast<WGPUTextureFormat>(mFormat);
  cfg.usage = mUsage ? mUsage : WGPUTextureUsage_RenderAttachment;
  cfg.width = mWidth;
  cfg.height = mHeight;
  cfg.viewFormatCount = 0;
  cfg.viewFormats = nullptr;
  cfg.alphaMode = WGPUCompositeAlphaMode_Auto;
  // Mailbox (when available): present never BLOCKS the GUI thread waiting for vblank — Fifo's
  // block added 10-20 ms of variable latency per frame to the main loop (frame-pacing jitter the
  // user sees as unstable FPS). Mailbox still syncs scan-out (no tearing) but replaces the queued
  // image instead of waiting. Fifo remains the fallback (always supported).
  cfg.presentMode = WGPUPresentMode_Fifo;
  {
    WGPUSurfaceCapabilities pmCaps = {};
    WGPUAdapter ad = static_cast<WGPUAdapter>(mBackend->adapter());
    if (ad && wgpuSurfaceGetCapabilities(static_cast<WGPUSurface>(mSurface), ad, &pmCaps) == WGPUStatus_Success) {
      for (size_t i = 0; i < pmCaps.presentModeCount; ++i)
        if (pmCaps.presentModes[i] == WGPUPresentMode_Mailbox) {
          cfg.presentMode = WGPUPresentMode_Mailbox;
          break;
        }
      wgpuSurfaceCapabilitiesFreeMembers(pmCaps);
    }
  }
  wgpuSurfaceConfigure(static_cast<WGPUSurface>(mSurface), &cfg);
  mConfigured = true;
#  endif
#endif
}

void OmWgpuSurface::resize(uint32_t width, uint32_t height) {
  if (width == 0 || height == 0 || (width == mWidth && height == mHeight))
    return;
  mWidth = width;
  mHeight = height;
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (mSurface)
    configure();  // depth attachment is rebuilt lazily in ensureSceneDepth()
#  endif
#endif
}

#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
bool OmWgpuSurface::ensureSceneDepth() {
  if (mScnDepthView && mScnDepthW == mWidth && mScnDepthH == mHeight)
    return true;
  if (mScnDepthView) {
    wgpuTextureViewRelease(static_cast<WGPUTextureView>(mScnDepthView));
    mScnDepthView = nullptr;
  }
  if (mScnDepthTexture) {
    wgpuTextureRelease(static_cast<WGPUTexture>(mScnDepthTexture));
    mScnDepthTexture = nullptr;
  }
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUTextureDescriptor dtDesc = {};
  dtDesc.usage = WGPUTextureUsage_RenderAttachment;
  dtDesc.dimension = WGPUTextureDimension_2D;
  dtDesc.size = {mWidth, mHeight, 1};
  dtDesc.format = WGPUTextureFormat_Depth24Plus;
  dtDesc.mipLevelCount = 1;
  dtDesc.sampleCount = 1;
  WGPUTexture dt = wgpuDeviceCreateTexture(device, &dtDesc);
  if (!dt)
    return false;
  mScnDepthTexture = dt;
  WGPUTextureView dv = wgpuTextureCreateView(dt, nullptr);
  if (!dv)
    return false;
  mScnDepthView = dv;
  mScnDepthW = mWidth;
  mScnDepthH = mHeight;
  return true;
}

bool OmWgpuSurface::ensureScenePipeline() {
  if (mScnPipeline)
    return true;
  if (!mUsable)
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());

  WGPUShaderSourceWGSL wgsl = {};
  wgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
  wgsl.code.data = OmWgpuShaders::kSolidLit;
  wgsl.code.length = WGPU_STRLEN;
  WGPUShaderModuleDescriptor smDesc = {};
  smDesc.nextInChain = &wgsl.chain;
  WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &smDesc);
  if (!sm) {
    OmLog::info("[OmWgpuSurface] scn CreateShaderModule failed");
    return false;
  }
  mScnShaderModule = sm;

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
    OmLog::info("[OmWgpuSurface] scn CreateBindGroupLayout failed");
    return false;
  }
  mScnBindGroupLayout = bgl;

  WGPUPipelineLayoutDescriptor plDesc = {};
  plDesc.bindGroupLayoutCount = 1;
  plDesc.bindGroupLayouts = &bgl;
  WGPUPipelineLayout plLayout = wgpuDeviceCreatePipelineLayout(device, &plDesc);
  if (!plLayout) {
    OmLog::info("[OmWgpuSurface] scn CreatePipelineLayout failed");
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

  // Color target keyed to the SURFACE format (not RGBA8Unorm) — a render
  // pipeline's target format must match the render-pass attachment, and the
  // surface backbuffer is whatever wgpuSurfaceGetCapabilities preferred.
  WGPUColorTargetState colorTarget = {};
  colorTarget.format = static_cast<WGPUTextureFormat>(mFormat);
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
    OmLog::info("[OmWgpuSurface] scn CreateRenderPipeline failed");
    return false;
  }
  mScnPipeline = pipe;
  return true;
}

// R4 material fidelity: textured-lit pipeline for the LIVE pane. kSolidLitTextured
// (kSolidLit + albedo @1 + sampler @2), keyed to mFormat. Its OWN 3-entry layout +
// a linear/repeat sampler. Reuses ensureScenePipeline's depth view. Per-draw bound.
bool OmWgpuSurface::ensureSceneTexPipeline() {
  if (mScnTexPipeline)
    return true;
  if (!ensureScenePipeline())  // mScnDepthView (built lazily in presentScene) + uniform buf
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());

  WGPUShaderSourceWGSL wgsl = {};
  wgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
  wgsl.code.data = OmWgpuShaders::kSolidLitTextured;
  wgsl.code.length = WGPU_STRLEN;
  WGPUShaderModuleDescriptor smDesc = {};
  smDesc.nextInChain = &wgsl.chain;
  WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &smDesc);
  if (!sm) {
    OmLog::info("[OmWgpuSurface] scn-tex CreateShaderModule failed");
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
  sampDesc.lodMaxClamp = 1.0f;
  sampDesc.compare = WGPUCompareFunction_Undefined;
  sampDesc.maxAnisotropy = 1;
  WGPUSampler samp = wgpuDeviceCreateSampler(device, &sampDesc);
  if (!samp) {
    OmLog::info("[OmWgpuSurface] scn-tex CreateSampler failed");
    return false;
  }
  mScnTexSampler = samp;

  // 1×1 default textures for absent map slots (byte-identical for albedo-only draws): white
  // roughness, black metalness (dielectric), flat normal (128,128,255 → +Z → no perturb).
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
    wgpuQueueWriteTexture(static_cast<WGPUQueue>(mBackend->queue()), &dst, px, 4, &lay, &ext);
    WGPUTextureViewDescriptor vd = {};
    vd.format = WGPUTextureFormat_RGBA8Unorm;
    vd.dimension = WGPUTextureViewDimension_2D;
    vd.mipLevelCount = 1;
    vd.arrayLayerCount = 1;
    view = wgpuTextureCreateView(tex, &vd);
    return view != nullptr;
  };
  {
    WGPUTexture wt = nullptr, bt = nullptr, nt = nullptr;
    WGPUTextureView wv = nullptr, bv = nullptr, nv = nullptr;
    if (!make1x1(255, 255, 255, wt, wv) || !make1x1(0, 0, 0, bt, bv) ||
        !make1x1(128, 128, 255, nt, nv)) {
      OmLog::info("[OmWgpuSurface] scn-tex default-texture build failed");
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
    bglEntries[t].texture.sampleType = WGPUTextureSampleType_Float;
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
    OmLog::info("[OmWgpuSurface] scn-tex CreateBindGroupLayout failed");
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
  colorTarget.format = static_cast<WGPUTextureFormat>(mFormat);
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
    OmLog::info("[OmWgpuSurface] scn-tex CreateRenderPipeline failed");
    return false;
  }
  mScnTexPipeline = pipe;
  return true;
}

// R4 step-3c-A: line-overlay pipeline. kSolidPick shader (transform by viewProj*model,
// flat baseColor) + LineList topology, color target keyed to mFormat, depth-tested
// (LessEqual) against the scene. Reuses mScnBindGroupLayout (so ensureScenePipeline
// must run first). Lets presentScene draw overlays (bounding objects, …) in the pane.
bool OmWgpuSurface::ensureLineOverlayPipeline() {
  if (mLinePipeline)
    return true;
  if (!ensureScenePipeline())  // builds mScnBindGroupLayout (reused below)
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());

  WGPUShaderSourceWGSL wgsl = {};
  wgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
  wgsl.code.data = OmWgpuShaders::kSolidPick;
  wgsl.code.length = WGPU_STRLEN;
  WGPUShaderModuleDescriptor smDesc = {};
  smDesc.nextInChain = &wgsl.chain;
  WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &smDesc);
  if (!sm) {
    OmLog::info("[OmWgpuSurface] line CreateShaderModule failed");
    return false;
  }
  mLineShaderModule = sm;

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
  colorTarget.format = static_cast<WGPUTextureFormat>(mFormat);
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
  depthState.depthCompare = WGPUCompareFunction_LessEqual;
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
    OmLog::info("[OmWgpuSurface] line CreateRenderPipeline failed");
    return false;
  }
  mLinePipeline = pipe;
  return true;
}

// R4 step-3c-A: always-on-top line-overlay pipeline. Like ensureLineOverlayPipeline
// but depthCompare=Always + no depth write, so markers (joint axes, COM, contact)
// inside geometry stay visible in the pane. Reuses mLineShaderModule.
bool OmWgpuSurface::ensureLineNoDepthOverlayPipeline() {
  if (mLineNoDepthPipeline)
    return true;
  if (!ensureLineOverlayPipeline())  // builds mLineShaderModule + mScnBindGroupLayout
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
  colorTarget.format = static_cast<WGPUTextureFormat>(mFormat);
  colorTarget.writeMask = WGPUColorWriteMask_All;

  WGPUFragmentState fs = {};
  fs.module = static_cast<WGPUShaderModule>(mLineShaderModule);
  fs.entryPoint.data = "fs_main";
  fs.entryPoint.length = WGPU_STRLEN;
  fs.targetCount = 1;
  fs.targets = &colorTarget;

  WGPUDepthStencilState depthState = {};
  depthState.format = WGPUTextureFormat_Depth24Plus;
  depthState.depthWriteEnabled = WGPUOptionalBool_False;
  depthState.depthCompare = WGPUCompareFunction_Always;
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
    OmLog::info("[OmWgpuSurface] line(no-depth) CreateRenderPipeline failed");
    return false;
  }
  mLineNoDepthPipeline = pipe;
  return true;
}
#  endif
#endif


// Window-swap present: kTexturedQuad pipeline keyed to the surface format (no depth, no blend).
bool OmWgpuSurface::ensurePresentTexPipeline() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (mPresentPipeline)
    return true;
  if (!mUsable)
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUQueue queue = static_cast<WGPUQueue>(mBackend->queue());
  WGPUShaderSourceWGSL wgsl = {};
  wgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
  wgsl.code.data = OmWgpuShaders::kTexturedQuad;
  wgsl.code.length = WGPU_STRLEN;
  WGPUShaderModuleDescriptor smDesc = {};
  smDesc.nextInChain = &wgsl.chain;
  WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &smDesc);
  if (!sm)
    return false;
  mPresentShaderModule = sm;
  WGPUBindGroupLayoutEntry e[3] = {};
  e[0].binding = 0;
  e[0].visibility = WGPUShaderStage_Vertex | WGPUShaderStage_Fragment;
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
  mPresentBgl = bgl;
  WGPUPipelineLayoutDescriptor plDesc = {};
  plDesc.bindGroupLayoutCount = 1;
  plDesc.bindGroupLayouts = &bgl;
  WGPUPipelineLayout plLayout = wgpuDeviceCreatePipelineLayout(device, &plDesc);
  if (!plLayout)
    return false;
  WGPUColorTargetState colorTarget = {};
  colorTarget.format = static_cast<WGPUTextureFormat>(mFormat);
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
  mPresentPipeline = wgpuDeviceCreateRenderPipeline(device, &pd);
  wgpuPipelineLayoutRelease(plLayout);
  if (!mPresentPipeline)
    return false;
  WGPUBufferDescriptor ud = {};
  ud.size = 16;
  ud.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
  mPresentUb = wgpuDeviceCreateBuffer(device, &ud);
  if (!mPresentUb)
    return false;
  const float rect[4] = {-1.0f, -1.0f, 1.0f, 1.0f};
  wgpuQueueWriteBuffer(queue, static_cast<WGPUBuffer>(mPresentUb), 0, rect, 16);
  WGPUSamplerDescriptor sd = {};
  sd.addressModeU = WGPUAddressMode_ClampToEdge;
  sd.addressModeV = WGPUAddressMode_ClampToEdge;
  sd.addressModeW = WGPUAddressMode_ClampToEdge;
  sd.magFilter = WGPUFilterMode_Linear;
  sd.minFilter = WGPUFilterMode_Linear;
  sd.mipmapFilter = WGPUMipmapFilterMode_Nearest;
  sd.lodMinClamp = 0.0f;
  sd.lodMaxClamp = 0.0f;
  sd.maxAnisotropy = 1;
  mPresentSampler = wgpuDeviceCreateSampler(device, &sd);
  return mPresentSampler != nullptr;
#  else
  return false;
#  endif
#else
  return false;
#endif
}

bool OmWgpuSurface::presentTexture(void *srcTextureView) {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (!mUsable || !srcTextureView || !ensurePresentTexPipeline())
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUQueue queue = static_cast<WGPUQueue>(mBackend->queue());
  WGPUSurface surface = static_cast<WGPUSurface>(mSurface);
  if (!mPresentBg || mPresentSrcView != srcTextureView) {
    if (mPresentBg)
      wgpuBindGroupRelease(static_cast<WGPUBindGroup>(mPresentBg));
    WGPUBindGroupEntry be[3] = {};
    be[0].binding = 0;
    be[0].buffer = static_cast<WGPUBuffer>(mPresentUb);
    be[0].size = 16;
    be[1].binding = 1;
    be[1].textureView = static_cast<WGPUTextureView>(srcTextureView);
    be[2].binding = 2;
    be[2].sampler = static_cast<WGPUSampler>(mPresentSampler);
    WGPUBindGroupDescriptor bd = {};
    bd.layout = static_cast<WGPUBindGroupLayout>(mPresentBgl);
    bd.entryCount = 3;
    bd.entries = be;
    mPresentBg = wgpuDeviceCreateBindGroup(device, &bd);
    mPresentSrcView = srcTextureView;
    if (!mPresentBg)
      return false;
  }
  WGPUSurfaceTexture st = {};
  wgpuSurfaceGetCurrentTexture(surface, &st);
  switch (st.status) {
    case WGPUSurfaceGetCurrentTextureStatus_SuccessOptimal:
    case WGPUSurfaceGetCurrentTextureStatus_SuccessSuboptimal:
      break;
    case WGPUSurfaceGetCurrentTextureStatus_Outdated:
    case WGPUSurfaceGetCurrentTextureStatus_Lost:
      if (st.texture)
        wgpuTextureRelease(st.texture);
      configure();
      return false;
    default:
      if (st.texture)
        wgpuTextureRelease(st.texture);
      return false;
  }
  WGPUTexture tex = st.texture;
  WGPUTextureView view = wgpuTextureCreateView(tex, nullptr);
  if (!view) {
    wgpuTextureRelease(tex);
    return false;
  }
  WGPUCommandEncoderDescriptor encDesc = {};
  WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(device, &encDesc);
  bool ok = false;
  if (encoder) {
    WGPURenderPassColorAttachment ca = {};
    ca.view = view;
    ca.loadOp = WGPULoadOp_Clear;
    ca.storeOp = WGPUStoreOp_Store;
    ca.clearValue = {0.0, 0.0, 0.0, 1.0};
    ca.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
    WGPURenderPassDescriptor pd = {};
    pd.colorAttachmentCount = 1;
    pd.colorAttachments = &ca;
    WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &pd);
    if (pass) {
      wgpuRenderPassEncoderSetPipeline(pass, static_cast<WGPURenderPipeline>(mPresentPipeline));
      wgpuRenderPassEncoderSetBindGroup(pass, 0, static_cast<WGPUBindGroup>(mPresentBg), 0, nullptr);
      wgpuRenderPassEncoderDraw(pass, 6, 1, 0, 0);
      wgpuRenderPassEncoderEnd(pass);
      wgpuRenderPassEncoderRelease(pass);
      WGPUCommandBufferDescriptor cbd = {};
      WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(encoder, &cbd);
      if (cmd) {
        wgpuQueueSubmit(queue, 1, &cmd);
        wgpuCommandBufferRelease(cmd);
        wgpuSurfacePresent(surface);
        ok = true;
      }
    }
    wgpuCommandEncoderRelease(encoder);
  }
  wgpuTextureViewRelease(view);
  wgpuTextureRelease(tex);
  return ok;
#  else
  (void)srcTextureView;
  return false;
#  endif
#else
  (void)srcTextureView;
  return false;
#endif
}

bool OmWgpuSurface::presentClear(float r, float g, float b) {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (!mUsable || !mSurface || !mBackend || !mBackend->device() || !mBackend->queue())
    return false;
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUQueue queue = static_cast<WGPUQueue>(mBackend->queue());
  WGPUSurface surface = static_cast<WGPUSurface>(mSurface);

  WGPUSurfaceTexture st = {};
  wgpuSurfaceGetCurrentTexture(surface, &st);
  switch (st.status) {
    case WGPUSurfaceGetCurrentTextureStatus_SuccessOptimal:
    case WGPUSurfaceGetCurrentTextureStatus_SuccessSuboptimal:
      break;
    case WGPUSurfaceGetCurrentTextureStatus_Outdated:
    case WGPUSurfaceGetCurrentTextureStatus_Lost:
      if (st.texture)
        wgpuTextureRelease(st.texture);
      configure();
      return false;
    default:
      if (st.texture)
        wgpuTextureRelease(st.texture);
      return false;
  }

  WGPUTexture tex = st.texture;
  WGPUTextureView view = wgpuTextureCreateView(tex, nullptr);
  if (!view) {
    wgpuTextureRelease(tex);
    return false;
  }

  WGPUCommandEncoderDescriptor encDesc = {};
  WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(device, &encDesc);
  if (!encoder) {
    wgpuTextureViewRelease(view);
    wgpuTextureRelease(tex);
    return false;
  }

  WGPURenderPassColorAttachment ca = {};
  ca.view = view;
  ca.loadOp = WGPULoadOp_Clear;
  ca.storeOp = WGPUStoreOp_Store;
  ca.clearValue = {r, g, b, 1.0};
  ca.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
  WGPURenderPassDescriptor pd = {};
  pd.colorAttachmentCount = 1;
  pd.colorAttachments = &ca;
  WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &pd);
  if (!pass) {
    wgpuCommandEncoderRelease(encoder);
    wgpuTextureViewRelease(view);
    wgpuTextureRelease(tex);
    return false;
  }
  wgpuRenderPassEncoderEnd(pass);
  wgpuRenderPassEncoderRelease(pass);

  WGPUCommandBufferDescriptor cmdDesc = {};
  WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(encoder, &cmdDesc);
  wgpuCommandEncoderRelease(encoder);
  if (!cmd) {
    wgpuTextureViewRelease(view);
    wgpuTextureRelease(tex);
    return false;
  }
  wgpuQueueSubmit(queue, 1, &cmd);
  wgpuCommandBufferRelease(cmd);

  WGPUStatus pres = wgpuSurfacePresent(surface);
  wgpuTextureViewRelease(view);
  wgpuTextureRelease(tex);
  return pres == WGPUStatus_Success;
#  else
  (void)r;
  (void)g;
  (void)b;
  return false;
#  endif
#else
  (void)r;
  (void)g;
  (void)b;
  return false;
#endif
}

bool OmWgpuSurface::presentScene(const float *viewProj16, const float *lightDirAmbient4,
                                 const OmWgpuSolidDraw *draws, uint32_t numDraws,
                                 float clearR, float clearG, float clearB, void *rgbaOut,
                                 const float *lineVerts, uint32_t lineVertCount,
                                 const float *lineColor4, const float *topVerts,
                                 uint32_t topVertCount, const float *topColor4,
                                 const float *cameraWorldPos3) {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (!mUsable || !mSurface || !mBackend || !mBackend->device() || !mBackend->queue() || !viewProj16 ||
      !lightDirAmbient4)
    return false;
  if (!ensureScenePipeline() || !ensureSceneDepth())
    return false;
  const bool wantReadback = (rgbaOut != nullptr) && mCanReadback;
  if (wantReadback) {
    const size_t rbSize = static_cast<size_t>(alignedBytesPerRow(mWidth)) * mHeight;
    if (rbSize > mReadbackSize) {
      if (mReadbackBuffer)
        wgpuBufferRelease(static_cast<WGPUBuffer>(mReadbackBuffer));
      WGPUBufferDescriptor rbDesc = {};
      rbDesc.size = rbSize;
      rbDesc.usage = WGPUBufferUsage_CopyDst | WGPUBufferUsage_MapRead;
      WGPUBuffer rb = wgpuDeviceCreateBuffer(static_cast<WGPUDevice>(mBackend->device()), &rbDesc);
      if (!rb)
        return false;
      mReadbackBuffer = rb;
      mReadbackSize = rbSize;
    }
  }
  WGPUDevice device = static_cast<WGPUDevice>(mBackend->device());
  WGPUQueue queue = static_cast<WGPUQueue>(mBackend->queue());
  WGPUSurface surface = static_cast<WGPUSurface>(mSurface);

  // Grow the per-draw uniform buffer (256-byte slots). At least one slot.
  const uint32_t slotCount = numDraws == 0 ? 1u : numDraws;
  const size_t needed = static_cast<size_t>(slotCount) * kScnUniformStride;
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

  // Pack the 192-byte Scene struct per draw: viewProj(0) model(64)
  // baseColor(128) lightDirAmbient(144); pads (160+) left zero (plain lit).
  std::vector<uint8_t> hostBuf(needed, 0);
  for (uint32_t i = 0; i < numDraws; ++i) {
    uint8_t *slot = hostBuf.data() + static_cast<size_t>(i) * kScnUniformStride;
    std::memcpy(slot + 0, viewProj16, 64);
    if (draws[i].modelMatrix16)
      std::memcpy(slot + 64, draws[i].modelMatrix16, 64);
    const float baseColor[4] = {draws[i].baseColorR, draws[i].baseColorG, draws[i].baseColorB,
                                draws[i].baseColorA};
    std::memcpy(slot + 128, baseColor, 16);
    std::memcpy(slot + 144, lightDirAmbient4, 16);
    // R4 material fidelity: per-draw specular smoothness (pad1.w) + camera world pos
    // (pad0.yzw) so the textured-lit path's Blinn-Phong specular is correct in the pane.
    std::memcpy(slot + 188, &draws[i].specularStrength, 4);
    if (cameraWorldPos3)
      std::memcpy(slot + 164, cameraWorldPos3, 12);
  }
  wgpuQueueWriteBuffer(queue, static_cast<WGPUBuffer>(mScnUniformBuffer), 0, hostBuf.data(), needed);

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

  // Acquire backbuffer.
  WGPUSurfaceTexture st = {};
  wgpuSurfaceGetCurrentTexture(surface, &st);
  switch (st.status) {
    case WGPUSurfaceGetCurrentTextureStatus_SuccessOptimal:
    case WGPUSurfaceGetCurrentTextureStatus_SuccessSuboptimal:
      break;
    case WGPUSurfaceGetCurrentTextureStatus_Outdated:
    case WGPUSurfaceGetCurrentTextureStatus_Lost:
      if (st.texture)
        wgpuTextureRelease(st.texture);
      wgpuBindGroupRelease(bg);
      configure();
      return false;
    default:
      if (st.texture)
        wgpuTextureRelease(st.texture);
      wgpuBindGroupRelease(bg);
      return false;
  }
  WGPUTexture tex = st.texture;
  WGPUTextureView view = wgpuTextureCreateView(tex, nullptr);
  if (!view) {
    wgpuTextureRelease(tex);
    wgpuBindGroupRelease(bg);
    return false;
  }

  WGPUCommandEncoderDescriptor encDesc = {};
  WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(device, &encDesc);
  if (!encoder) {
    wgpuTextureViewRelease(view);
    wgpuTextureRelease(tex);
    wgpuBindGroupRelease(bg);
    return false;
  }

  WGPURenderPassColorAttachment colorAttach = {};
  colorAttach.view = view;
  colorAttach.loadOp = WGPULoadOp_Clear;
  colorAttach.storeOp = WGPUStoreOp_Store;
  colorAttach.clearValue = {clearR, clearG, clearB, 1.0};
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
    wgpuTextureViewRelease(view);
    wgpuTextureRelease(tex);
    wgpuBindGroupRelease(bg);
    return false;
  }
  // R4 material fidelity: textured draws (carrying an albedo textureView) render through
  // the textured-lit pipeline + a per-draw bind group; flat draws stay on mScnPipeline.
  // texBgs released post-submit. ensureSceneTexPipeline is lazy / safe to fail (→ flat).
  const bool texReady = ensureSceneTexPipeline();
  std::vector<WGPUBindGroup> texBgs;
  wgpuRenderPassEncoderSetPipeline(pass, static_cast<WGPURenderPipeline>(mScnPipeline));
  for (uint32_t i = 0; i < numDraws; ++i) {
    const OmWgpuSolidDraw &d = draws[i];
    if (!d.vertexBuffer || !d.indexBuffer || d.indexCount == 0)
      continue;
    const uint32_t dynOffset = i * kScnUniformStride;
    WGPUBindGroup useBg = bg;
    WGPURenderPipeline usePipe = static_cast<WGPURenderPipeline>(mScnPipeline);
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
  // SetBindGroup held its own ref (encoder → command buffer), so release ours now.
  for (WGPUBindGroup tb : texBgs)
    wgpuBindGroupRelease(tb);

  // R4 step-3c-A: optional line overlays on the lit scene — a depth-tested batch
  // (bounding objects) at uniform slot 0 + an always-on-top batch (markers) at slot 1.
  // Both LOAD colour+depth so they composite over the scene. Resources freed post-submit.
  std::vector<WGPUBuffer> lineVbufs;
  WGPUBindGroup lineBg = nullptr;
  // Build BOTH pipelines up front (no && short-circuit — that would skip the second).
  const bool depthReady = (lineVerts && lineVertCount >= 2 && lineColor4) && ensureLineOverlayPipeline();
  const bool topReady = (topVerts && topVertCount >= 2 && topColor4) && ensureLineNoDepthOverlayPipeline();
  if (depthReady || topReady) {
    const size_t need = static_cast<size_t>(kScnUniformStride) * 2;  // two colour slots
    if (!mLineUniformBuffer) {
      WGPUBufferDescriptor ud = {};
      ud.size = need;
      ud.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
      mLineUniformBuffer = wgpuDeviceCreateBuffer(device, &ud);
    }
    if (mLineUniformBuffer) {
      std::vector<uint8_t> lh(need, 0);
      static const float kId[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
      for (int slot = 0; slot < 2; ++slot) {
        uint8_t *s = lh.data() + static_cast<size_t>(slot) * kScnUniformStride;
        std::memcpy(s + 0, viewProj16, 64);
        std::memcpy(s + 64, kId, 64);
      }
      if (lineColor4)
        std::memcpy(lh.data() + 128, lineColor4, 16);
      if (topColor4)
        std::memcpy(lh.data() + kScnUniformStride + 128, topColor4, 16);
      wgpuQueueWriteBuffer(queue, static_cast<WGPUBuffer>(mLineUniformBuffer), 0, lh.data(), need);

      WGPUBindGroupEntry le = {};
      le.binding = 0;
      le.buffer = static_cast<WGPUBuffer>(mLineUniformBuffer);
      le.offset = 0;
      le.size = 192;
      WGPUBindGroupDescriptor ldsc = {};
      ldsc.layout = static_cast<WGPUBindGroupLayout>(mScnBindGroupLayout);
      ldsc.entryCount = 1;
      ldsc.entries = &le;
      lineBg = wgpuDeviceCreateBindGroup(device, &ldsc);

      auto drawBatch = [&](WGPURenderPipeline pipe, const float *verts, uint32_t count, uint32_t slot) {
        if (!lineBg || !pipe || !verts || count < 2)
          return;
        WGPUBufferDescriptor vd = {};
        vd.size = static_cast<uint64_t>(count) * 32u;
        vd.usage = WGPUBufferUsage_Vertex | WGPUBufferUsage_CopyDst;
        WGPUBuffer vbuf = wgpuDeviceCreateBuffer(device, &vd);
        if (!vbuf)
          return;
        wgpuQueueWriteBuffer(queue, vbuf, 0, verts, vd.size);
        lineVbufs.push_back(vbuf);
        WGPURenderPassColorAttachment lc = {};
        lc.view = view;
        lc.loadOp = WGPULoadOp_Load;
        lc.storeOp = WGPUStoreOp_Store;
        lc.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
        WGPURenderPassDepthStencilAttachment ldepth = {};
        ldepth.view = static_cast<WGPUTextureView>(mScnDepthView);
        ldepth.depthLoadOp = WGPULoadOp_Load;
        ldepth.depthStoreOp = WGPUStoreOp_Store;
        ldepth.depthReadOnly = 0;
        WGPURenderPassDescriptor lpd = {};
        lpd.colorAttachmentCount = 1;
        lpd.colorAttachments = &lc;
        lpd.depthStencilAttachment = &ldepth;
        WGPURenderPassEncoder lpass = wgpuCommandEncoderBeginRenderPass(encoder, &lpd);
        if (lpass) {
          wgpuRenderPassEncoderSetPipeline(lpass, pipe);
          const uint32_t dyn = slot * kScnUniformStride;
          wgpuRenderPassEncoderSetBindGroup(lpass, 0, lineBg, 1, &dyn);
          wgpuRenderPassEncoderSetVertexBuffer(lpass, 0, vbuf, 0, WGPU_WHOLE_SIZE);
          wgpuRenderPassEncoderDraw(lpass, count, 1, 0, 0);
          wgpuRenderPassEncoderEnd(lpass);
          wgpuRenderPassEncoderRelease(lpass);
        }
      };
      if (depthReady)
        drawBatch(static_cast<WGPURenderPipeline>(mLinePipeline), lineVerts, lineVertCount, 0);
      if (topReady)
        drawBatch(static_cast<WGPURenderPipeline>(mLineNoDepthPipeline), topVerts, topVertCount, 1);
    }
  }

  // Self-check: copy the rendered backbuffer to the readback buffer in the
  // same command stream (the surface texture is valid until present).
  const uint32_t rbStride = wantReadback ? alignedBytesPerRow(mWidth) : 0;
  if (wantReadback) {
    WGPUTexelCopyTextureInfo csrc = {};
    csrc.texture = tex;
    csrc.aspect = WGPUTextureAspect_All;
    WGPUTexelCopyBufferInfo cdst = {};
    cdst.buffer = static_cast<WGPUBuffer>(mReadbackBuffer);
    cdst.layout.bytesPerRow = rbStride;
    cdst.layout.rowsPerImage = mHeight;
    WGPUExtent3D cext = {mWidth, mHeight, 1};
    wgpuCommandEncoderCopyTextureToBuffer(encoder, &csrc, &cdst, &cext);
  }

  WGPUCommandBufferDescriptor cmdDesc = {};
  WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(encoder, &cmdDesc);
  wgpuCommandEncoderRelease(encoder);
  if (!cmd) {
    if (lineBg)
      wgpuBindGroupRelease(lineBg);
    for (WGPUBuffer vb : lineVbufs)
      wgpuBufferRelease(vb);
    wgpuTextureViewRelease(view);
    wgpuTextureRelease(tex);
    wgpuBindGroupRelease(bg);
    return false;
  }
  wgpuQueueSubmit(queue, 1, &cmd);
  wgpuCommandBufferRelease(cmd);
  // The submitted command buffer holds its own refs; safe to release now.
  if (lineBg)
    wgpuBindGroupRelease(lineBg);
  for (WGPUBuffer vb : lineVbufs)
    wgpuBufferRelease(vb);

  // Map the readback buffer (blocks via poll) and copy unpadded rows out.
  if (wantReadback) {
    MapCapture cap;
    WGPUBufferMapCallbackInfo mapCb = {};
    mapCb.mode = WGPUCallbackMode_AllowSpontaneous;
    mapCb.callback = onMap;
    mapCb.userdata1 = &cap;
    (void)wgpuBufferMapAsync(static_cast<WGPUBuffer>(mReadbackBuffer), WGPUMapMode_Read, 0,
                             mReadbackSize, mapCb);
    for (int i = 0; i < 1000 && !cap.done; ++i)
      wgpuDevicePoll(static_cast<WGPUDevice>(mBackend->device()), true, nullptr);
    if (cap.done && cap.ok) {
      const uint8_t *mapped = static_cast<const uint8_t *>(
        wgpuBufferGetConstMappedRange(static_cast<WGPUBuffer>(mReadbackBuffer), 0, mReadbackSize));
      if (mapped) {
        uint8_t *out = static_cast<uint8_t *>(rgbaOut);
        const uint32_t rowBytes = mWidth * 4u;
        for (uint32_t y = 0; y < mHeight; ++y)
          std::memcpy(out + static_cast<size_t>(y) * rowBytes,
                      mapped + static_cast<size_t>(y) * rbStride, rowBytes);
      }
      wgpuBufferUnmap(static_cast<WGPUBuffer>(mReadbackBuffer));
    }
  }

  WGPUStatus pres = wgpuSurfacePresent(surface);
  wgpuTextureViewRelease(view);
  wgpuTextureRelease(tex);
  wgpuBindGroupRelease(bg);
  return pres == WGPUStatus_Success;
#  else
  (void)viewProj16;
  (void)lightDirAmbient4;
  (void)draws;
  (void)numDraws;
  (void)clearR;
  (void)clearG;
  (void)clearB;
  (void)rgbaOut;
  (void)lineVerts;
  (void)lineVertCount;
  (void)lineColor4;
  (void)topVerts;
  (void)topVertCount;
  (void)topColor4;
  return false;
#  endif
#else
  (void)viewProj16;
  (void)lightDirAmbient4;
  (void)draws;
  (void)numDraws;
  (void)clearR;
  (void)clearG;
  (void)clearB;
  (void)rgbaOut;
  (void)lineVerts;
  (void)lineVertCount;
  (void)lineColor4;
  (void)topVerts;
  (void)topVertCount;
  (void)topColor4;
  return false;
#endif
}
