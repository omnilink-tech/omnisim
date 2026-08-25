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

#ifndef OM_SOLID_DEVICE_HPP
#define OM_SOLID_DEVICE_HPP

//
// Description: abstract base classes for all Webots solid devices
//

#include "OmDevice.hpp"
#include "OmSolid.hpp"

// Generic Webots devices, i.e., all devices but OmMotor and OmPositionSensor
class OmSolidDevice : public OmDevice, public OmSolid {
public:
  // destructor
  virtual ~OmSolidDevice() override;
  const QString &deviceName() const override { return OmSolid::name(); }
  int deviceNodeType() const override { return nodeType(); }

  // device name is unique in Robot scope so the string can be simplified
  QString computeShortUniqueName() const;

protected:
  // all constructors are reserved for derived classes only
  OmSolidDevice(const QString &modelName, OmTokenizer *tokenizer);
  OmSolidDevice(const OmSolidDevice &other);
  explicit OmSolidDevice(const OmNode &other);

  // Re-aim this device's rays at the current poses. Radar and Camera override
  // it and call it themselves immediately before their Newton raycast; it used
  // to be driven by a static "dirty sensors" list that ODE's collision pass
  // drained mid-step, which is gone along with ODE.
  virtual void updateRaysSetupIfNeeded() {}
};

#endif
