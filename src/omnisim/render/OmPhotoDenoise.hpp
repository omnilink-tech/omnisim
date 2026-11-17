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
#ifndef OM_PHOTO_DENOISE_HPP
#define OM_PHOTO_DENOISE_HPP
#include "OmPhoto.hpp"
// Optional runtime, loaded from a caller-provided absolute path. A failure keeps
// the original HDR image intact and reports the reason; no silent learned filter.
bool omPhotoDenoise(OmPhotoResult &result, int width, int height,
                    const std::string &libraryPath, const std::atomic<bool> &cancel,
                    double timeLimitSeconds, std::string &message);
#endif
