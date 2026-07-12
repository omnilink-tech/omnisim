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

#ifndef WB_WREN_BACKEND_HPP
#define WB_WREN_BACKEND_HPP

//
// WbWrenBackend — concrete WbRenderBackend that delegates every
// operation to the existing WREN code path. This is the canonical
// backend; every Camera and Viewpoint that doesn't explicitly opt
// into another backend uses this one.
//
// Twin of WbOdeBackend in the physics arm. Note: at R0 of the
// engine-migration plan, this class is just a "marker" implementation
// that reports kind/name/available. Real WREN dispatching still
// happens through the existing call-site code; the backend is only
// consulted at lookup time to decide whether to short-circuit to a
// non-WREN path.
//
// As later phases widen WbRenderBackend with virtual methods
// (createFramebuffer, submitDrawList, renderToTexture, ...), each new
// method gets a WbWrenBackend implementation that calls the existing
// WREN code verbatim -- this is by design, so the no-op refactor
// stays no-op.
//

#include "WbRenderBackend.hpp"

class WbWrenBackend : public WbRenderBackend {
public:
  WbWrenBackend() = default;
  ~WbWrenBackend() override = default;
  WbRenderBackendKind kind() const override { return WbRenderBackendKind::Wren; }
  const char *name() const override { return "wren"; }
  bool isAvailable() const override { return true; }
};

#endif
