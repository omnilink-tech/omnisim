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

#ifndef OM_DEVICE_HPP
#define OM_DEVICE_HPP

//
// Description: abstract base classes for all webots devices
//

#include "../../../include/controller/c/omnisim/types.h"  // WbDeviceTag definition

class QDataStream;
class OmDataStream;
class QString;

// logical device class
class OmDevice {
public:
  // destructor
  virtual ~OmDevice();

  virtual double energyConsumption() const { return 0.0; }
  virtual void handleMessage(QDataStream &) {}
  virtual void writeAnswer(OmDataStream &) {}
  virtual void writeConfigure(OmDataStream &) {}
  virtual void powerOn(bool e) { mPowerOn = e; }  // power off when running out of battery
  bool isPowerOn() const { return mPowerOn; }
  bool hasTag() const { return mTag != UNASSIGNED; }
  WbDeviceTag tag() const { return mTag; }
  virtual const QString &deviceName() const = 0;
  virtual int deviceNodeType() const = 0;
  void setTag(WbDeviceTag tag) { mTag = tag; }
  void setIsControllerRunning(bool isRunning) { mIsControllerRunning = isRunning; }

  // for sensors only
  virtual bool refreshSensorIfNeeded() { return false; }

protected:
  // all constructors are reserved for derived classes only
  OmDevice();
  OmDevice(const OmDevice &other);

  bool isControllerRunning() const { return mIsControllerRunning; }
  void *mWindow;  // robot window

private:
  WbDeviceTag mTag;
  static const WbDeviceTag UNASSIGNED = 65535;  // maximum short int value
  bool mPowerOn;
  bool mIsControllerRunning;
  OmDevice &operator=(const OmDevice &);  // non copyable
  void init();
};

#endif
