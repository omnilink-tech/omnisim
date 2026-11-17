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
#include "OmPhotoDenoise.hpp"
#include "third_party/oidn/oidn.h"
#include <chrono>
#include <algorithm>
#include <cmath>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <dlfcn.h>
#endif

bool omPhotoDenoise(OmPhotoResult &r, int width, int height, const std::string &libraryPath,
                    const std::atomic<bool> &cancel, double seconds, std::string &message) {
  const auto start=std::chrono::steady_clock::now();
  const size_t size=static_cast<size_t>(std::max(0,width))*std::max(0,height)*3;
  if (!size || r.linearRgb.size()!=size || r.albedoGuide.size()!=size || r.normalGuide.size()!=size ||
      cancel.load() || seconds<=0) { message="Denoising requires a completed image and time budget."; return false; }
#ifdef _WIN32
  const int n=MultiByteToWideChar(CP_UTF8,0,libraryPath.c_str(),-1,nullptr,0);
  std::wstring wide(n,L'\0'); MultiByteToWideChar(CP_UTF8,0,libraryPath.c_str(),-1,wide.data(),n);
  HMODULE library=LoadLibraryExW(wide.c_str(),nullptr,LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR|LOAD_LIBRARY_SEARCH_DEFAULT_DIRS);
  auto symbol=[&](const char *name) { return library?reinterpret_cast<void *>(GetProcAddress(library,name)):nullptr; };
#else
  void *library=dlopen(libraryPath.c_str(),RTLD_NOW|RTLD_LOCAL);
  auto symbol=[&](const char *name) { return library?dlsym(library,name):nullptr; };
#endif
  if (!library) { message="Open Image Denoise runtime is unavailable."; return false; }
  auto unload=[&]() {
#ifdef _WIN32
    FreeLibrary(library);
#else
    dlclose(library);
#endif
  };
#define OIDN_LOAD(name) const auto name##Fn=reinterpret_cast<decltype(&name)>(symbol(#name)); \
  if (!name##Fn) { message="Open Image Denoise runtime has an incompatible API."; unload(); return false; }
  OIDN_LOAD(oidnNewDevice) OIDN_LOAD(oidnReleaseDevice) OIDN_LOAD(oidnSetDeviceInt)
  OIDN_LOAD(oidnSetDeviceBool) OIDN_LOAD(oidnCommitDevice) OIDN_LOAD(oidnGetDeviceError)
  OIDN_LOAD(oidnNewFilter) OIDN_LOAD(oidnReleaseFilter) OIDN_LOAD(oidnSetSharedFilterImage)
  OIDN_LOAD(oidnSetFilterBool) OIDN_LOAD(oidnCommitFilter) OIDN_LOAD(oidnExecuteFilter)
  OIDN_LOAD(oidnSetFilterProgressMonitorFunction)
#undef OIDN_LOAD
  OIDNDevice device=oidnNewDeviceFn(OIDN_DEVICE_TYPE_CPU);
  if (!device) { message="Open Image Denoise could not create a CPU device."; unload(); return false; }
  oidnSetDeviceIntFn(device,"numThreads",1);
  oidnSetDeviceBoolFn(device,"setAffinity",false);
  oidnCommitDeviceFn(device);
  const char *error=nullptr;
  if (oidnGetDeviceErrorFn(device,&error)!=OIDN_ERROR_NONE) {
    message=error?error:"Open Image Denoise device failed.";
    oidnReleaseDeviceFn(device); unload(); return false;
  }
  OIDNFilter filter=oidnNewFilterFn(device,"RT");
  if (!filter) { message="Open Image Denoise RT filter is unavailable."; oidnReleaseDeviceFn(device); unload(); return false; }
  std::vector<float> output(size);
  auto bind=[&](const char *name,std::vector<float> &pixels) {
    oidnSetSharedFilterImageFn(filter,name,pixels.data(),OIDN_FORMAT_FLOAT3,width,height,0,0,0);
  };
  bind("color",r.linearRgb); bind("albedo",r.albedoGuide); bind("normal",r.normalGuide); bind("output",output);
  oidnSetFilterBoolFn(filter,"hdr",true);
  oidnSetFilterBoolFn(filter,"cleanAux",false); // guides are sampled and antialiased, not noise-free
  struct Monitor { const std::atomic<bool> *cancel; std::chrono::steady_clock::time_point start; double limit; };
  Monitor monitor{&cancel,start,seconds};
  oidnSetFilterProgressMonitorFunctionFn(filter,[](void *ptr,double) {
    const auto &m=*static_cast<Monitor *>(ptr);
    return !m.cancel->load() && std::chrono::duration<double>(std::chrono::steady_clock::now()-m.start).count()<m.limit;
  },&monitor);
  oidnCommitFilterFn(filter); oidnExecuteFilterFn(filter);
  const auto code=oidnGetDeviceErrorFn(device,&error);
  const bool ok=code==OIDN_ERROR_NONE && !cancel.load();
  if (!ok) message=error?error:"Denoising was cancelled.";
  oidnReleaseFilterFn(filter); oidnReleaseDeviceFn(device); unload();
  if (ok) {
    for (float &v:output) if (!std::isfinite(v) || v<0) v=0;
    r.denoisedRgb=std::move(output); r.denoiser="Open Image Denoise 2";
  }
  return ok;
}
