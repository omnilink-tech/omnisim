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

#include "OmNormal.hpp"

#include "OmMFVector3.hpp"

void OmNormal::init() {
  mVector = findMFVector3("vector");
}

OmNormal::OmNormal(OmTokenizer *tokenizer) : OmBaseNode("Normal", tokenizer) {
  init();
}

OmNormal::OmNormal(const OmNormal &other) : OmBaseNode(other) {
  init();
}

OmNormal::OmNormal(const OmNode &other) : OmBaseNode(other) {
  init();
}

OmNormal::~OmNormal() {
}

QStringList OmNormal::fieldsToSynchronizeWithW3d() const {
  QStringList fields;
  fields << "vector";
  return fields;
}
