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

#ifndef OM_SINGLE_VALUE_HPP
#define OM_SINGLE_VALUE_HPP

//
// Description: abstract base class for single field values
//
// Inherited by:
//   OmSFVector3, OmSFVector2, OmSFString, OmSFRotation, OmSFNode, OmSFInt, OmSFDouble, OmSFColor, WBSFBool
//

#include "OmValue.hpp"
#include "OmVariant.hpp"

class OmSingleValue : public OmValue {
  Q_OBJECT

public:
  virtual ~OmSingleValue() override;

  // return generic value
  virtual OmVariant variantValue() const = 0;
  QString toString(OmPrecision::Level level = OmPrecision::DOUBLE_MAX) const override {
    return variantValue().toSimplifiedStringRepresentation(level);
  }

protected:
  OmSingleValue();

private:
};

#endif
