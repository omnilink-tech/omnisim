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

#ifndef OM_WGPU_GL_BLIT_HPP
#define OM_WGPU_GL_BLIT_HPP

//
// OmWgpuGlBlit — R4 3c-B helper. Uploads an RGBA8 image (top-down, as produced by the wgpu offscreen
// render) into a GL texture and blits it to the current GL draw surface (the main-view window's default
// framebuffer), flipping Y so the wgpu top-down image lands right-way-up in the bottom-up GL window.
//
// This lives in its OWN translation unit precisely so it can include <glad/glad.h> (raw GL) WITHOUT
// dragging glad into a Qt TU (OmView3D), where it would clash with Qt's GL headers. glad's function
// pointers are process-global and are loaded by WREN at GL-context creation, so this TU sees the same
// loaded entry points. The caller MUST have made the WREN GL context current first.
//

// Blit `rgba` (w*h*4 bytes, RGBA8, top-down) to the currently-bound GL draw framebuffer (0 = window).
// No-op-safe: if w/h <= 0 or rgba is null it does nothing.
void OmWgpuGlBlitRgbaToScreen(const unsigned char *rgba, int w, int h);

#endif
