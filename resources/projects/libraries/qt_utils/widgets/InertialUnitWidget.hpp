/*
 * Description:  Widget displaying a webots inertial unit device
 */

#ifndef INERTIAL_UNIT_WIDGET_HPP
#define INERTIAL_UNIT_WIDGET_HPP

#include "VectorialSensorWidget.hpp"

namespace omnisimQtUtils {
  class InertialUnitWidget : public VectorialSensorWidget {
    Q_OBJECT

  public:
    explicit InertialUnitWidget(Device *device, QWidget *parent = NULL);
    virtual ~InertialUnitWidget() override {}

  protected slots:
    void enable(bool enable) override;

  protected:
    bool isEnabled() const override;
    const double *values() override;
  };
}  // namespace omnisimQtUtils

#endif
