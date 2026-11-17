/*
 * Description:  Abstraction of an OmniSim robot device
 */

#ifndef DEVICE_HPP
#define DEVICE_HPP

#include <omnisim/device.h>
#include <omnisim/robot.h>

#include <QtCore/QString>

namespace omnisimQtUtils {
  class Device {
  public:
    explicit Device(WbDeviceTag tag);
    virtual ~Device() {}

    WbDeviceTag tag() const { return mTag; }
    WbNodeType type() const { return mType; }
    const QString &name() const { return mName; }
    const QString &category() const { return mCategory; }

  protected:
    WbDeviceTag mTag;
    WbNodeType mType;
    QString mName;
    QString mCategory;
  };
}  // namespace omnisimQtUtils

#endif
