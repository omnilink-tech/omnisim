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

#ifndef OM_MOTION_VECTORS_HPP
#define OM_MOTION_VECTORS_HPP

#include "OmMotionHistory.hpp"
#include "OmWgpuShaders.hpp"
#include <webgpu/webgpu.h>

// Private native implementation. Static opaque surfaces use camera reprojection
// without another geometry pass; moving/new surfaces write per-sample history UV.
class OmMotionVectors {
  WGPUDevice mDevice;
  WGPURenderPipeline mPipeline = nullptr, mTransparent = nullptr;
  WGPUTexture mTexture = nullptr, mDummy = nullptr;
  WGPUTextureView mView = nullptr, mDummyView = nullptr;
  WGPUBindGroupLayout mLayout = nullptr;
  WGPUBuffer mUniform = nullptr, mSlots = nullptr, mPositions = nullptr;
  uint64_t mSlotBytes = 0, mPositionBytes = 0;
  uint32_t mWidth = 0, mHeight = 0;
  OmMotionHistory mHistory;

  WGPUTexture texture(uint32_t width, uint32_t height) {
    WGPUTextureDescriptor td = {};
    td.dimension = WGPUTextureDimension_2D;
    td.format = WGPUTextureFormat_RGBA16Float;
    td.size = {width, height, 1}; td.mipLevelCount = 1; td.sampleCount = 4;
    td.usage = WGPUTextureUsage_RenderAttachment | WGPUTextureUsage_TextureBinding;
    return wgpuDeviceCreateTexture(mDevice, &td);
  }
  bool buffer(WGPUBuffer &out, uint64_t &capacity, uint64_t size) {
    if (out && capacity >= size) return true;
    if (out) wgpuBufferRelease(out);
    capacity = std::max<uint64_t>(size, 256);
    WGPUBufferDescriptor bd = {};
    bd.size = capacity; bd.usage = WGPUBufferUsage_Storage | WGPUBufferUsage_CopyDst;
    out = wgpuDeviceCreateBuffer(mDevice, &bd);
    return out != nullptr;
  }
  bool pipeline() {
    if (mPipeline && mTransparent) return true;
    WGPUShaderSourceWGSL source = {};
    source.chain.sType = WGPUSType_ShaderSourceWGSL;
    source.code = {OmWgpuShaders::kObjectMotion, WGPU_STRLEN};
    WGPUShaderModuleDescriptor md = {}; md.nextInChain = &source.chain;
    WGPUShaderModule module = wgpuDeviceCreateShaderModule(mDevice, &md);
    if (!module) return false;
    WGPUBindGroupLayoutEntry e[5] = {};
    e[0].binding = 0; e[0].visibility = WGPUShaderStage_Vertex;
    e[0].buffer.type = WGPUBufferBindingType_Uniform; e[0].buffer.minBindingSize = 128;
    e[1].binding = 1; e[1].visibility = WGPUShaderStage_Vertex | WGPUShaderStage_Fragment;
    e[1].buffer.type = WGPUBufferBindingType_ReadOnlyStorage; e[1].buffer.minBindingSize = 176;
    e[2].binding = 2; e[2].visibility = WGPUShaderStage_Vertex;
    e[2].buffer.type = WGPUBufferBindingType_ReadOnlyStorage; e[2].buffer.minBindingSize = 16;
    e[3].binding = 3; e[3].visibility = WGPUShaderStage_Fragment;
    e[3].texture.sampleType = WGPUTextureSampleType_Float;
    e[3].texture.viewDimension = WGPUTextureViewDimension_2D;
    e[4].binding = 4; e[4].visibility = WGPUShaderStage_Fragment;
    e[4].sampler.type = WGPUSamplerBindingType_Filtering;
    WGPUBindGroupLayoutDescriptor ld = {}; ld.entryCount = 5; ld.entries = e;
    if (!mLayout) mLayout = wgpuDeviceCreateBindGroupLayout(mDevice, &ld);
    if (!mLayout) { wgpuShaderModuleRelease(module); return false; }
    WGPUPipelineLayoutDescriptor pd = {}; pd.bindGroupLayoutCount = 1; pd.bindGroupLayouts = &mLayout;
    WGPUPipelineLayout layout = wgpuDeviceCreatePipelineLayout(mDevice, &pd);
    if (!layout) { wgpuShaderModuleRelease(module); return false; }
    WGPUVertexAttribute attributes[2] = {};
    attributes[0].format = WGPUVertexFormat_Float32x3; attributes[0].shaderLocation = 0;
    attributes[1].format = WGPUVertexFormat_Float32x2; attributes[1].shaderLocation = 2; attributes[1].offset = 24;
    WGPUVertexBufferLayout vb = {}; vb.arrayStride = 32; vb.stepMode = WGPUVertexStepMode_Vertex;
    vb.attributeCount = 2; vb.attributes = attributes;
    WGPUColorTargetState color = {}; color.format = WGPUTextureFormat_RGBA16Float;
    color.writeMask = WGPUColorWriteMask_All;
    WGPUFragmentState fragment = {}; fragment.module = module;
    fragment.entryPoint = {"fs_main", WGPU_STRLEN}; fragment.targetCount = 1; fragment.targets = &color;
    WGPUDepthStencilState depth = {}; depth.format = WGPUTextureFormat_Depth32Float;
    depth.depthWriteEnabled = WGPUOptionalBool_False; depth.depthCompare = WGPUCompareFunction_Equal;
    WGPURenderPipelineDescriptor desc = {}; desc.layout = layout;
    desc.vertex.module = module; desc.vertex.entryPoint = {"vs_main", WGPU_STRLEN};
    desc.vertex.bufferCount = 1; desc.vertex.buffers = &vb;
    desc.fragment = &fragment; desc.depthStencil = &depth;
    desc.primitive.topology = WGPUPrimitiveTopology_TriangleList;
    desc.primitive.frontFace = WGPUFrontFace_CCW; desc.primitive.cullMode = WGPUCullMode_None;
    desc.multisample.count = 4; desc.multisample.mask = 0xFFFFFFFFu;
    if (!mPipeline) mPipeline = wgpuDeviceCreateRenderPipeline(mDevice, &desc);
    // Transparent surfaces do not own the scene depth. Mark their visible
    // coverage as reactive instead of borrowing the opaque background's motion.
    depth.depthCompare = WGPUCompareFunction_GreaterEqual;
    if (!mTransparent) mTransparent = wgpuDeviceCreateRenderPipeline(mDevice, &desc);
    wgpuPipelineLayoutRelease(layout); wgpuShaderModuleRelease(module);
    WGPUBufferDescriptor ub = {}; ub.size = 128;
    ub.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
    if (!mUniform) mUniform = wgpuDeviceCreateBuffer(mDevice, &ub);
    return mPipeline && mTransparent && mUniform;
  }

public:
  explicit OmMotionVectors(WGPUDevice device) : mDevice(device) {
    mDummy = texture(1, 1);
    if (mDummy) mDummyView = wgpuTextureCreateView(mDummy, nullptr);
  }
  ~OmMotionVectors() {
    if (mPipeline) wgpuRenderPipelineRelease(mPipeline);
    if (mTransparent) wgpuRenderPipelineRelease(mTransparent);
    if (mLayout) wgpuBindGroupLayoutRelease(mLayout);
    if (mUniform) wgpuBufferRelease(mUniform);
    if (mSlots) wgpuBufferRelease(mSlots);
    if (mPositions) wgpuBufferRelease(mPositions);
    if (mView) wgpuTextureViewRelease(mView);
    if (mTexture) wgpuTextureRelease(mTexture);
    if (mDummyView) wgpuTextureViewRelease(mDummyView);
    if (mDummy) wgpuTextureRelease(mDummy);
  }
  WGPUTextureView dummy() const { return mDummyView; }
  void reset() { mHistory.reset(); }

  template<class BeginPass, class Visible>
  WGPUTextureView render(WGPUQueue queue, uint32_t width, uint32_t height, WGPUTextureView depth,
                        WGPUSampler sampler, WGPUTextureView white, const float *vp, const float *previousVP,
                        const OmWgpuSolidDraw *draws, uint32_t count, bool historyValid,
                        BeginPass beginPass, Visible visible) {
    mHistory.prepare(draws, count, historyValid, visible);
    if (mHistory.records.empty()) return mDummyView;
    if (!pipeline()) return nullptr;
    if (mWidth != width || mHeight != height || !mView) {
      if (mView) wgpuTextureViewRelease(mView);
      if (mTexture) wgpuTextureRelease(mTexture);
      mTexture = texture(width, height);
      mView = mTexture ? wgpuTextureCreateView(mTexture, nullptr) : nullptr;
      mWidth = width; mHeight = height;
    }
    if (!mView || !buffer(mSlots, mSlotBytes, mHistory.records.size()*sizeof(OmMotionHistory::Slot)) ||
        !buffer(mPositions, mPositionBytes, mHistory.positions.size()*16)) return nullptr;
    float u[32]; std::memcpy(u, vp, 64); std::memcpy(u+16, previousVP, 64);
    wgpuQueueWriteBuffer(queue, mUniform, 0, u, sizeof(u));
    wgpuQueueWriteBuffer(queue, mSlots, 0, mHistory.records.data(), mHistory.records.size()*sizeof(OmMotionHistory::Slot));
    if (!mHistory.positions.empty())
      wgpuQueueWriteBuffer(queue, mPositions, 0, mHistory.positions.data(), mHistory.positions.size()*16);
    // Bind groups are local to this frame: replacing a buffer can recycle its
    // native handle address, so address equality must never preserve an old bind.
    std::map<WGPUTextureView, WGPUBindGroup> groups;
    bool complete = true;
    for (uint32_t i : mHistory.draws) {
      auto view = draws[i].textureView ? static_cast<WGPUTextureView>(draws[i].textureView) : white;
      if (groups.count(view)) continue;
      WGPUBindGroupEntry e[5] = {};
      e[0].binding = 0; e[0].buffer = mUniform; e[0].size = 128;
      e[1].binding = 1; e[1].buffer = mSlots; e[1].size = mSlotBytes;
      e[2].binding = 2; e[2].buffer = mPositions; e[2].size = mPositionBytes;
      e[3].binding = 3; e[3].textureView = view;
      e[4].binding = 4; e[4].sampler = sampler;
      WGPUBindGroupDescriptor bd = {}; bd.layout = mLayout; bd.entryCount = 5; bd.entries = e;
      WGPUBindGroup bg = wgpuDeviceCreateBindGroup(mDevice, &bd);
      groups.emplace(view, bg); complete &= bg != nullptr;
    }
    WGPURenderPassColorAttachment ca = {}; ca.view = mView;
    ca.loadOp = WGPULoadOp_Clear; ca.storeOp = WGPUStoreOp_Store;
    ca.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
    WGPURenderPassDepthStencilAttachment da = {}; da.view = depth; da.depthReadOnly = true;
    WGPURenderPassDescriptor desc = {}; desc.colorAttachmentCount = 1;
    desc.colorAttachments = &ca; desc.depthStencilAttachment = &da;
    auto pass = complete ? beginPass(desc, "motion") : nullptr;
    if (pass) {
      // Glass must overwrite any opaque motion underneath, regardless of list order.
      for (int transparent = 0; transparent < 2; ++transparent) {
        wgpuRenderPassEncoderSetPipeline(pass, transparent ? mTransparent : mPipeline);
        for (uint32_t s = 0; s < mHistory.draws.size(); ++s) {
          const auto &d = draws[mHistory.draws[s]];
          if (d.translucent != static_cast<bool>(transparent)) continue;
          auto view = d.textureView ? static_cast<WGPUTextureView>(d.textureView) : white;
          wgpuRenderPassEncoderSetBindGroup(pass, 0, groups[view], 0, nullptr);
          wgpuRenderPassEncoderSetVertexBuffer(pass, 0, static_cast<WGPUBuffer>(d.vertexBuffer), 0, WGPU_WHOLE_SIZE);
          wgpuRenderPassEncoderSetIndexBuffer(pass, static_cast<WGPUBuffer>(d.indexBuffer), WGPUIndexFormat_Uint32,
                                             0, WGPU_WHOLE_SIZE);
          wgpuRenderPassEncoderDrawIndexed(pass, d.indexCount, 1, 0, 0, s);
        }
      }
      wgpuRenderPassEncoderEnd(pass); wgpuRenderPassEncoderRelease(pass);
    }
    for (auto &entry : groups) if (entry.second) wgpuBindGroupRelease(entry.second);
    return pass ? mView : nullptr;
  }
};
#endif
