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

#include "OmTextureCoordinate.hpp"
#include "OmMFVector2.hpp"

void OmTextureCoordinate::init() {
  mPoint = findMFVector2("point");
}

OmTextureCoordinate::OmTextureCoordinate(OmTokenizer *tokenizer) : OmBaseNode("TextureCoordinate", tokenizer) {
  init();
}

OmTextureCoordinate::OmTextureCoordinate(const OmTextureCoordinate &other) : OmBaseNode(other) {
  init();
}

OmTextureCoordinate::OmTextureCoordinate(const OmNode &other) : OmBaseNode(other) {
  init();
}

OmTextureCoordinate::~OmTextureCoordinate() {
}

QStringList OmTextureCoordinate::fieldsToSynchronizeWithW3d() const {
  QStringList fields;
  fields << "point";
  return fields;
}
