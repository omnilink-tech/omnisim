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

#include "OmBillboard.hpp"

OmBillboard::OmBillboard(OmTokenizer *tokenizer) : OmGroup("Billboard", tokenizer) {
}

OmBillboard::OmBillboard(const OmBillboard &other) : OmGroup(other) {
}

OmBillboard::OmBillboard(const OmNode &other) : OmGroup(other) {
}

OmBillboard::~OmBillboard() {
  // D1.4: the dedicated WREN transform node died with the WREN renderer.
}

void OmBillboard::createWrenObjects() {
  // D1.4: the dedicated WREN transform that tracked the Viewpoint (its
  // camera-follow position/rotation pushes and the cameraParametersChanged
  // connect) died with the WREN renderer.
  // D1.4 TODO(excision): billboard camera-facing behaviour needs a wgpu hook.
  // Only the child recursion survives.
  OmBaseNode::createWrenObjects();

  const int size = children().size();
  for (int i = 0; i < size; ++i) {
    OmBaseNode *const n = child(i);
    n->createWrenObjects();
  }
}
