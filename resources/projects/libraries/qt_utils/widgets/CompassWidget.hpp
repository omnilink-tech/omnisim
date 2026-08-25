/*
 * Description:  Widget displaying a webots compass sensor device
 */

#ifndef COMPASS_WIDGET_HPP
#define COMPASS_WIDGET_HPP

#include "VectorialSensorWidget.hpp"

namespace omnisimQtUtils {
  class CompassWidget : public VectorialSensorWidget {
    Q_OBJECT

  public:
    explicit CompassWidget(Device *device, QWidget *parent = NULL);
    virtual ~CompassWidget() override {}

  protected slots:
    void enable(bool enable) override;

  protected:
    bool isEnabled() const override;
    const double *values() override;
  };
}  // namespace omnisimQtUtils

#endif
