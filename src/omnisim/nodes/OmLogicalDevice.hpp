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

// Abstract node class representing a logical device, i.e., a device without physics properties

#ifndef OM_LOGICAL_DEVICE_HPP
#define OM_LOGICAL_DEVICE_HPP

#include "OmBaseNode.hpp"
#include "OmDevice.hpp"
#include "OmSFString.hpp"

class OmLogicalDevice : public OmBaseNode, public OmDevice {
public:
  virtual ~OmLogicalDevice() override;
  const QString &deviceName() const override { return mDeviceName->value(); }
  int deviceNodeType() const override { return nodeType(); }

protected:
  explicit OmLogicalDevice(const QString &modelName, OmTokenizer *tokenizer = NULL);
  OmLogicalDevice(const OmLogicalDevice &other);
  explicit OmLogicalDevice(const OmNode &other);
  bool exportNodeHeader(OmWriter &writer) const override;

protected:
  OmSFString *mDeviceName;

private:
  OmLogicalDevice &operator=(const OmLogicalDevice &);  // non copyable
  void init();
};

#endif
