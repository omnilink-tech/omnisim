from pathlib import Path
r=Path('src/omnisim/render')
p=r/'OmWgpuRenderTarget.hpp'
s=p.read_text()
s=s.replace('the previous frame\'s UNJITTERED view-proj', 'the previous frame\'s jittered view-proj')
s=s.replace('  void *mTaaMvTex[2]', '  void *mGpuTimer = nullptr;  // private native timestamp ring\n  void *mTaaDepthTex[2] = {nullptr, nullptr};\n  void *mTaaDepthView[2] = {nullptr, nullptr};\n  void *mTaaMvTex[2]',1)
p.write_text(s)
p=r/'OmWgpuRenderTarget.cpp';s=p.read_text()
s=s.replace('#    include "webgpu/wgpu.h"', '#    include "webgpu/wgpu.h"\n#    include "OmGpuTimer.hpp"',1)
a=s.index('OmWgpuRenderTarget::~OmWgpuRenderTarget()')
i=s.index('#  ifdef WB_WGPU_NATIVE_AVAILABLE',a)+len('#  ifdef WB_WGPU_NATIVE_AVAILABLE')
s=s[:i]+'\n  delete static_cast<OmGpuTimer *>(mGpuTimer);'+s[i:]
s=s.replace('for (void *tv : {mTaaMvView[0], mTaaMvView[1]})','for (void *tv : {mTaaMvView[0], mTaaMvView[1], mTaaDepthView[0], mTaaDepthView[1]})',1)
s=s.replace('for (void *tt : {mTaaMvTex[0], mTaaMvTex[1]})','for (void *tt : {mTaaMvTex[0], mTaaMvTex[1], mTaaDepthTex[0], mTaaDepthTex[1]})',1)
# Add historical depth binding and second attachment to the LIVE TAA pipeline.
a=s.index('bool OmWgpuRenderTarget::ensureTaaMvPipeline()');b=s.index('\nbool ',a+5);t=s[a:b]
t=t.replace('le[5]', 'le[6]').replace('ld.entryCount = 5;', 'ld.entryCount = 6;')
t=t.replace('  WGPUBindGroupLayoutDescriptor ld', '''  le[5].binding = 5;
  le[5].visibility = WGPUShaderStage_Fragment;
  le[5].texture.sampleType = WGPUTextureSampleType_UnfilterableFloat;
  le[5].texture.viewDimension = WGPUTextureViewDimension_2D;
  WGPUBindGroupLayoutDescriptor ld''',1)
t=t.replace('WGPUColorTargetState ct = {};\n  ct.format = WGPUTextureFormat_RGBA8Unorm;\n  ct.writeMask = WGPUColorWriteMask_All;', '''WGPUColorTargetState ct[2] = {};
  ct[0].format = WGPUTextureFormat_RGBA8Unorm;
  ct[0].writeMask = WGPUColorWriteMask_All;
  ct[1].format = WGPUTextureFormat_R32Float;
  ct[1].writeMask = WGPUColorWriteMask_Red;''')
t=t.replace('fs.targetCount = 1;\n  fs.targets = &ct;', 'fs.targetCount = 2;\n  fs.targets = ct;')
s=s[:a]+t+s[b:]
# Work within the shared main/sensor rendering function only.
a=s.index('bool OmWgpuRenderTarget::clearAndDrawSceneTexturedShadowed(');b=s.index('\nbool ',a+5);t=s[a:b]
t=t.replace('  // Shared LightU uniform:', '''  // Value-parsed A/B hatch for the former camera-only, unchecked history resolve.
  static const bool temporalValidation = !qEnvironmentVariableIsSet("OMNISIM_WGPU_TAA_VALIDATION") ||
                                         qEnvironmentVariableIntValue("OMNISIM_WGPU_TAA_VALIDATION") != 0;
  float jitteredVP[16];
  std::memcpy(jitteredVP, viewProj16, 64);
  // Shared LightU uniform:''',1)
t=t.replace('      std::memcpy(lu + 960, vpJ, 64);','      std::memcpy(lu + 960, vpJ, 64);\n      std::memcpy(jitteredVP, vpJ, 64);',1)
assert 'std::memcpy(jitteredVP, vpJ' in t
needle='  WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(device, &encDesc);'
t=t.replace(needle, '''  const QByteArray timingPath = qgetenv("OMNISIM_WGPU_GPU_TIMING");
  if (!mGpuTimer && !timingPath.isEmpty() && timingPath != "0")
    mGpuTimer = new OmGpuTimer(device, queue, timingPath.constData(), mWidth, mHeight);
  auto *gpuTimer = static_cast<OmGpuTimer *>(mGpuTimer);
  if (gpuTimer) gpuTimer->begin(device);
''' + needle,1)
# Each invocation records one timestamp pair; repeated cascades/stages share a group name.
import re
matches=list(re.finditer(r'wgpuCommandEncoderBeginRenderPass\(encoder,\s*&([a-zA-Z0-9]+)\)',t))
names=['sunShadow','localShadow','scene','ssr','volume','exposure','tonemap','aoDepth','ao','bloom','taa']
assert len(matches)==len(names),(len(matches),len(names))
for match,name in reversed(list(zip(matches,names))):
    t=t[:match.start()]+f'beginTimedPass({match.group(1)}, "{name}")'+t[match.end():]
loc=t.index(needle)+len(needle)
t=t[:loc]+'''
  auto beginTimedPass = [&](WGPURenderPassDescriptor &desc, const char *name) {
    desc.timestampWrites = gpuTimer ? gpuTimer->stamp(name) : nullptr;
    return wgpuCommandEncoderBeginRenderPass(encoder, &desc);
  };'''+t[loc:]
# Resolve on final encoder. The optional shadow-map diagnostic may submit earlier passes;
# their query results remain valid until this final resolve, so no forced timing wait.
t=t.replace('  if (!rgba8) {\n    // Render-only:', '  if (gpuTimer) gpuTimer->resolve(encoder);\n\n  if (!rgba8) {\n    // Render-only:',1)
t=t.replace('wgpuQueueSubmit(queue, 1, &cmdNb);','wgpuQueueSubmit(queue, 1, &cmdNb);\n    if (gpuTimer) gpuTimer->submitted();',1)
t=t.replace('wgpuQueueSubmit(queue, 1, &cmd);','wgpuQueueSubmit(queue, 1, &cmd);\n  if (gpuTimer) gpuTimer->submitted();',1)
# Depth histories are full float, never quantized into the LDR alpha channel.
t=t.replace('          WGPUTextureDescriptor td = {};', '''          if (mTaaDepthView[i])
            wgpuTextureViewRelease(static_cast<WGPUTextureView>(mTaaDepthView[i]));
          if (mTaaDepthTex[i])
            wgpuTextureRelease(static_cast<WGPUTexture>(mTaaDepthTex[i]));
          WGPUTextureDescriptor td = {};''',1)
needle='                          : nullptr;\n        }\n        mTaaMvW'
t=t.replace(needle,'''                          : nullptr;
          td.format = WGPUTextureFormat_R32Float;
          td.usage = WGPUTextureUsage_RenderAttachment | WGPUTextureUsage_TextureBinding;
          mTaaDepthTex[i] = wgpuDeviceCreateTexture(device, &td);
          mTaaDepthView[i] = mTaaDepthTex[i]
            ? wgpuTextureCreateView(static_cast<WGPUTexture>(mTaaDepthTex[i]), nullptr) : nullptr;
        }
        mTaaMvW''',1)
t=t.replace('      float invVP[16];\n      if (mTaaMvView[0] && mTaaMvView[1] && invert4x4ForSsr(viewProj16, invVP)) {', '''      float invVP[16];
      const float *historyVP = temporalValidation ? jitteredVP : viewProj16;
      if (mTaaMvView[0] && mTaaMvView[1] && mTaaDepthView[0] && mTaaDepthView[1] &&
          invert4x4ForSsr(historyVP, invVP)) {''',1)
t=t.replace('mTaaMvHistValid ? mTaaMvPrevVP : viewProj16','mTaaMvHistValid ? mTaaMvPrevVP : historyVP',1)
t=t.replace('{1.0f, 0.90f, 0.0f, mTaaMvHistValid ? 1.0f : 0.0f}', '{1.0f, 0.90f, temporalValidation ? 1.0f : 0.0f, mTaaMvHistValid ? 1.0f : 0.0f}',1)
t=t.replace('WGPUBindGroupEntry te[5]', 'WGPUBindGroupEntry te[6]',1)
t=t.replace('        WGPUBindGroupDescriptor tbd', '''        te[5].binding = 5;
        te[5].textureView = static_cast<WGPUTextureView>(mTaaDepthView[prev]);
        WGPUBindGroupDescriptor tbd''',1)
t=t.replace('tbd.entryCount = 5;', 'tbd.entryCount = 6;',1)
t=t.replace('          WGPURenderPassDescriptor rp2 = {};\n          rp2.colorAttachmentCount = 1;\n          rp2.colorAttachments = &ca2;', '''          WGPURenderPassColorAttachment attachments[2] = {ca2, ca2};
          attachments[1].view = static_cast<WGPUTextureView>(mTaaDepthView[cur]);
          WGPURenderPassDescriptor rp2 = {};
          rp2.colorAttachmentCount = 2;
          rp2.colorAttachments = attachments;''',1)
t=t.replace('std::memcpy(mTaaMvPrevVP, viewProj16, 64);','std::memcpy(mTaaMvPrevVP, historyVP, 64);',1)
s=s[:a]+t+s[b:];p.write_text(s)
