/*
 * Description:  Class defining the interclasses common properties
 */

#ifndef COMMON_PROPERTIES_HPP
#define COMMON_PROPERTIES_HPP

namespace omnisimQtUtils {
  class CommonProperties {
  public:
    static int precision() { return 4; }
    static int historySize() { return 200; }

  private:
    CommonProperties() {}
  };
}  // namespace omnisimQtUtils

#endif
