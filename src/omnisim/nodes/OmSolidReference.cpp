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

#include "OmSolidReference.hpp"

#include "OmNodeUtilities.hpp"
#include "OmSolid.hpp"

#include <cassert>

const QString OmSolidReference::STATIC_ENVIRONMENT = QString("<static environment>");

void OmSolidReference::init() {
  mName = findSFString("solidName");
}

// Constructors

OmSolidReference::OmSolidReference(OmTokenizer *tokenizer) : OmBaseNode("SolidReference", tokenizer) {
  init();
}

OmSolidReference::OmSolidReference(const OmSolidReference &other) : OmBaseNode(other), mSolid() {
  init();
}

OmSolidReference::OmSolidReference(const OmNode &other) : OmBaseNode(other) {
  init();
}

// Destructor
OmSolidReference::~OmSolidReference() {
}

void OmSolidReference::postFinalize() {
  OmBaseNode::postFinalize();
  connect(mName, &OmSFString::changed, this, &OmSolidReference::changed);
}

void OmSolidReference::updateName() {
  OmSolid *const ts = topSolid();
  assert(ts);
  const QString &nameString = mName->value();
  const bool linkToStaticEnvironment = nameString == STATIC_ENVIRONMENT;
  if (!linkToStaticEnvironment)
    mSolid = QPointer<OmSolid>(ts->findSolid(nameString, upperSolid()));
  else
    mSolid.clear();
  if (!nameString.isEmpty() && !linkToStaticEnvironment && mSolid.isNull())
    parsingWarn(
      tr("SolidReference has an invalid '%1' name or refers to its closest upper solid, which is prohibited.").arg(nameString));
}

QList<const OmBaseNode *> OmSolidReference::findClosestDescendantNodesWithDedicatedWrenNode() const {
  QList<const OmBaseNode *> list;
  if (mSolid)
    list << mSolid;
  return list;
}

QString OmSolidReference::endPointName() const {
  if (mSolid)
    return "\"" + mName->value() + "\"";
  return QString();
}
