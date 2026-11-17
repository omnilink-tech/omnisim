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

#include "OmWgpuGlBlit.hpp"

#include "glad/glad.h"

void OmWgpuGlBlitRgbaToScreen(const unsigned char *rgba, int w, int h) {
  if (!rgba || w <= 0 || h <= 0)
    return;
  // Process-global GL objects reused across frames (one main view). glad's entry points were loaded by
  // WREN when it created the GL context; the caller has made that context current.
  static GLuint sTex = 0;
  static GLuint sReadFbo = 0;
  if (sTex == 0) {
    glGenTextures(1, &sTex);
    glBindTexture(GL_TEXTURE_2D, sTex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  }
  glBindTexture(GL_TEXTURE_2D, sTex);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgba);

  if (sReadFbo == 0)
    glGenFramebuffers(1, &sReadFbo);

  // Save the caller's FBO bindings so we leave GL state as we found it.
  GLint prevDraw = 0, prevRead = 0;
  glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &prevDraw);
  glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &prevRead);

  glBindFramebuffer(GL_READ_FRAMEBUFFER, sReadFbo);
  glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, sTex, 0);
  // Present to the DEFAULT framebuffer (0) — the buffer swapBuffers(this) actually shows for the
  // QWindow-based main view. We must NOT reuse the currently-bound draw FBO: unlike WREN's path,
  // this code never calls wr_scene_render, so the binding left current after makeWrenCurrent() is a
  // stale WREN offscreen FBO. Blitting there put the (correct) wgpu frame off-screen and left the
  // window's default framebuffer empty → the grey/black viewport. Target 0 explicitly.
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

  // glBlitFramebuffer is clipped by the draw FBO's scissor box and masked by its colour mask. With
  // WREN not running to reset them, normalise both so the full-window blit isn't silently clipped.
  glDisable(GL_SCISSOR_TEST);
  glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

  // wgpu RGBA is top-down; the GL window is bottom-up → flip Y by swapping the dst Y extents.
  glBlitFramebuffer(0, 0, w, h, 0, h, w, 0, GL_COLOR_BUFFER_BIT, GL_NEAREST);

  // Restore the caller's bindings.
  glBindFramebuffer(GL_READ_FRAMEBUFFER, static_cast<GLuint>(prevRead));
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, static_cast<GLuint>(prevDraw));
  glBindTexture(GL_TEXTURE_2D, 0);
}
