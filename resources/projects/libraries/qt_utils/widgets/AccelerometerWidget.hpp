/*
 * Description:  Widget displaying a webots accelerometer sensor device
 */

#ifndef ACCELEROMETER_WIDGET_HPP
#define ACCELEROMETER_WIDGET_HPP

#include "VectorialSensorWidget.hpp"

namespace omnisimQtUtils {
  class AccelerometerWidget : public VectorialSensorWidget {
    Q_OBJECT

  public:
    explicit AccelerometerWidget(Device *device, QWidget *parent = NULL);
    virtual ~AccelerometerWidget() override {}

  protected slots:
    void enable(bool enable) override;

  protected:
    bool isEnabled() const override;
    const double *values() override;
  };
}  // namespace omnisimQtUtils

#endif
